#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <rtems.h>
#include <rtems/error.h>
#include <bsp.h>
#include <bsp/VME.h>
#include <bsp/VMEDMA.h>

#include <padProto.h>
#include <padStream.h>

#include <netinet/in.h>
#include <drvPadUdpComm.h>
#include <drvPadUdpCommIO.h>

#define VMEDIGI_NCHANNELS	4

#define DEBUG

#define BYTES_PER_SAMPLE	(VMEDIGI_NCHANNELS/* channels */ * sizeof(int16_t)/* bytes */)

#define MAXBYTES            (VMEDIGI_CNT_MAX * BYTES_PER_SAMPLE)

#define DMACHANNEL      0

#define MAX_DIGIS		(MAX_BPM  )
#define QDEPTH      	(MAX_BPM*2)
#define MAX_PKTBUFS		(MAX_BPM*5)


#include <vmeDigi.h>
#include <vmeDigiComm.h>

/* cached value of # samples     */
typedef struct VmeDigiCommRec_ {
	VmeDigi		     digi;
	uint32_t	     vmeaddr;
	unsigned         vec;
	uint16_t         nbytes;
	int16_t 	     running;
	int     	     channel;
	unsigned	     simIdx;
	unsigned	     simEnd;
	VmeDigiCommCbRec cb;
	void             *usr_arg;
} VmeDigiCommRec, *VmeDigiComm;

typedef struct VmeDigiPktHdrRec_ {
	struct VmeDigiPktRec_	*next;
	VmeDigiComm              digiComm;
	int                      refcnt;
} VmeDigiPktHdrRec, *VmeDigiPktHdr;

/* Min. size of 'header' */
#define MINHDRSZ (sizeof(VmeDigiPktHdrRec) + UDPCOMM_DATA_ALGN_OFF)
/* Up-align to UDPCOMM alignment boundary */
#define __DO_ALIGN(x,a) ( ( (x) + (a) - 1 ) & ~ ((a)-1) )
#define HDRSZ     __DO_ALIGN(MINHDRSZ, UDPCOMM_DATA_ALGN)

#define PADSZ     (HDRSZ - MINHDRSZ)

typedef struct VmeDigiPktRec_ {
	VmeDigiPktHdrRec         digiInfo;
	/* When padding we assume that the PktHdrRec is < 2*UDPCOMM_DATA_ALGN */
	uint8_t                  pad[PADSZ];
	union {
		uint8_t              raw[2048 - PADSZ - sizeof(VmeDigiPktHdrRec)];
		PadReplyRec          padrply;
	}                        pkt;
} VmeDigiPktRec, *VmeDigiPkt;

/* align to cache block size but at least 16-byte 
 * in case we want to use AltiVec.
 */
static VmeDigiPktRec	bufs[MAX_PKTBUFS] __attribute__((aligned(UDPCOMM_DATA_ALGN))) = { {{0}} };
static int        bufsUsed = 0;

static VmeDigiPkt freeList = 0;

static rtems_id sckq = 0;
static rtems_id irqq = 0;
static rtems_id task = 0;

static int         dmaIsrInstalled = 0;

static struct {
	uint32_t	timestampHi;
	uint32_t	timestampLo;
	uint32_t	xid;
} timestampInfo;

rtems_task_priority vmeCommTaskPriority = 15;

void
vmeCommFreePacket(UdpCommPkt ppacket);

void
vmeCommRefPacket(UdpCommPkt ppacket);

UdpCommPkt
vmeCommAllocPacket(int fd)
{
VmeDigiPkt            rval = 0;
rtems_interrupt_level l;

	rtems_interrupt_disable(l);
		if ( freeList ) {
			rval     = freeList;
			freeList = rval->digiInfo.next;
		} else if ( bufsUsed < MAX_PKTBUFS ) {
			rval    = &bufs[bufsUsed++];
		}
	rtems_interrupt_enable(l);

	if ( rval ) {
		rval->digiInfo.next     = 0;
		rval->digiInfo.digiComm = 0;
		rval->digiInfo.refcnt   = 1;
	}

	return (UdpCommPkt)rval;
}

VmeDigiPkt
vmeDigiPktAlloc(VmeDigiComm digiComm)
{
PadReply              rply;
VmeDigiPkt            rval;

	if ( (rval = (VmeDigiPkt)vmeCommAllocPacket(0)) ) {

		rval->digiInfo.digiComm = digiComm;

		rply                    = &rval->pkt.padrply;

		/* Fill in timestamps + other info from request */
		rply->version           = PADPROTO_VERSION4;
		rply->type              = PADCMD_STRM;
		rply->chnl              = digiComm->channel;
		rply->nBytes            = htons(digiComm->nbytes + sizeof(PadReplyRec));
		rply->timestampHi       = timestampInfo.timestampHi;
		rply->timestampLo       = timestampInfo.timestampLo;
		rply->xid               = timestampInfo.xid; 
		rply->stat              = 0;
		rply->strm_cmd_idx      = 0;
		rply->strm_cmd_flags    = PADCMD_STRM_FLAG_CM | PADRPLY_STRM_FLAG_TYPE_SET(PadDataBpm);
	}

	return rval;
}

static VmeDigiCommRec vmeDigis[MAX_DIGIS] = { {0} };

static inline unsigned
hibit(uint32_t v)
{
unsigned rval=0;
	if ( v & 0xffff0000 ) {
		v &= 0xffff0000;	
		rval |= 0x10;
	}
	if ( v & 0xff00ff00 ) {
		v &= 0xff00ff00;
		rval |= 0x08;
	}
	if ( v & 0xf0f0f0f0 ) {
		v &= 0xf0f0f0f0;
		rval |= 0x04;
	}
	if ( v & 0xcccccccc ) {
		v &= 0xcccccccc;
		rval |= 0x02;
	}
	if ( v & 0xaaaaaaaa )
		rval |= 0x01;
	return rval;
}

static volatile VmeDigiPkt digiPending      = 0;

static volatile VmeDigiPkt dmaInProgress    = 0;

volatile unsigned vmeCommPktsDroppedNoBuf   = 0;
volatile unsigned vmeCommPktsDroppedCbFail  = 0;
volatile unsigned vmeCommPktsDroppedNoQSpc  = 0;
volatile unsigned vmeCommPktsDroppedBadDma  = 0;
volatile unsigned vmeCommPktsDroppedBadDmaStatus  = 0;

volatile unsigned vmeCommDigiIrqs           = 0;
volatile unsigned vmeCommDmaIrqs            = 0;
volatile unsigned vmeCommDigiIrqsLL         = 0;
volatile unsigned vmeCommDmaIrqsLL          = 0;

volatile uint32_t vmeCommLastBadDmaStatus   = 0;


static void
rearm(VmeDigiComm digiComm)
{
	if ( digiComm->running ) 
		vmeDigiArm(digiComm->digi);
}

static int
moreDmaNeeded()
{
rtems_interrupt_level l;

	rtems_interrupt_disable(l);
		if ( (dmaInProgress = digiPending) ) {
			digiPending = dmaInProgress->digiInfo.next;
			/* paranoia */
			dmaInProgress->digiInfo.next = 0;
		}
	rtems_interrupt_enable(l);

	return dmaInProgress != 0;
}

static void
startDma()
{
VmeDigiComm  digiComm;
PadReply     rply;

retry:

	digiComm = dmaInProgress->digiInfo.digiComm;
	rply     = &dmaInProgress->pkt.padrply;
	if ( BSP_VMEDmaStart(DMACHANNEL, BSP_LOCAL2PCI_ADDR(&rply->data[0]), digiComm->vmeaddr + digiComm->simIdx, digiComm->nbytes) ) {
		vmeCommPktsDroppedBadDma++;
		vmeCommFreePacket((UdpCommPkt)dmaInProgress);

		if ( moreDmaNeeded() )
			goto retry;
	}
	if ( digiComm->simIdx ) {
		/* We are in simulation mode; increment the index */
		digiComm->simIdx += digiComm->nbytes;
		if ( digiComm->simIdx >= digiComm->simEnd )
			digiComm->simIdx = digiComm->nbytes;
	}
}

static void post_irq(void *arg)
{
rtems_status_code st;

	st = rtems_message_queue_send( irqq, (void*)&arg, sizeof( VmeDigiComm ) );

	if ( RTEMS_SUCCESSFUL != st ) {
		printk("vmeDigiComm: FATAL ERROR - unable to post to irq queue: %s\n", rtems_status_text( st ));
		rtems_fatal_error_occurred( st );
	}
}

static int
handleDigiIsr(void *arg)
{
VmeDigiComm           digiComm = arg;
VmeDigiPkt            p;
rtems_interrupt_level l;

	vmeCommDigiIrqs++;
	
	if ( ! ( p = vmeDigiPktAlloc(digiComm) ) ) {
		/* no buffer; must drop this packet */
		vmeCommPktsDroppedNoBuf++;
		rearm(digiComm);
		return -1;
	}

	rtems_interrupt_disable(l);
		if ( dmaInProgress ) {
			/* push to list head; this is not necessarily fair
			 * but the easiest. We expect all packets to be
			 * handled eventually...
			 */
			p->digiInfo.next = digiPending;
			digiPending      = p;
	rtems_interrupt_enable(l);
		} else {
			dmaInProgress = p;
	rtems_interrupt_enable(l);
			startDma();
		}

	return 0;
}

void
vmeCommDigiIsr(void *arg, unsigned long vec)
{
VmeDigiComm           digiComm = arg;

	/* Ack interrupt first -- otherwise we're stuck
     * until the DMA controller releases the bus.
     * should be safe because the board is unarmed at
     * this point.
     */
	vmeDigiIrqAck(digiComm->digi);

	handleDigiIsr(arg);
}

void
vmeCommDmaIsr(void *arg)
{
rtems_status_code     st;
VmeDigiComm           digiComm = dmaInProgress->digiInfo.digiComm;
uint32_t              dma_status;

	if ( ! arg && dmaIsrInstalled > 1 ) {
		/* task driven mode */
		vmeCommDmaIrqsLL++;
		post_irq( 0 );
		return;
	}

	vmeCommDmaIrqs++;

	/* irq driven mode and 'real-work' from task driven mode end up here */

	if ( (dma_status = BSP_VMEDmaStatus(DMACHANNEL)) ) {
		vmeCommLastBadDmaStatus = dma_status;
		vmeCommPktsDroppedBadDmaStatus++;	
		vmeCommFreePacket((UdpCommPkt)dmaInProgress);

		if ( arg && digiComm->cb.dma_done )
				digiComm->cb.dma_done( digiComm->channel, digiComm->digi, digiComm->usr_arg, 0 );
	} else {

		if ( arg && digiComm->cb.dma_done ) {
			if ( digiComm->cb.dma_done( digiComm->channel, digiComm->digi, digiComm->usr_arg, &dmaInProgress->pkt.padrply ) ) {
				vmeCommPktsDroppedCbFail++;
				vmeCommFreePacket((UdpCommPkt)dmaInProgress);
				dmaInProgress = 0;
			}
		}

		if ( dmaInProgress ) {
			st = rtems_message_queue_send(sckq, (void*)&dmaInProgress, sizeof(dmaInProgress));

			if ( RTEMS_SUCCESSFUL != st ) {
				/* this should include the case when there is no queue */
				vmeCommPktsDroppedNoQSpc++;
				vmeCommFreePacket((UdpCommPkt)dmaInProgress);
			}
		}
	}

	rearm(digiComm);

	if ( moreDmaNeeded() )
		startDma();
}


rtems_task
vmeCommThread(rtems_task_argument arg)
{
rtems_status_code st;
VmeDigiComm       digiComm;
size_t            sz;
int               dmaPending;

	while ( 1 ) {

		st = rtems_message_queue_receive(
				irqq,
				&digiComm,
				&sz,
				RTEMS_WAIT,
				RTEMS_NO_TIMEOUT);

		if ( RTEMS_SUCCESSFUL != st ) {
			rtems_panic("vmeDigiComm: vmeCommThread unable to receive from msg queue: %s\n", rtems_status_text( st ) );
			/* never get here */
		}
	
		if ( 0 == sz ) /* 'KILL' */
			break;

		if ( digiComm ) {
			dmaPending = ! handleDigiIsr( digiComm );
			/* if acq_done is NULL then vmeCommDigiIsr is directly called from IRQ level */
			digiComm->cb.acq_done( digiComm->channel, digiComm->digi, digiComm->usr_arg, dmaPending );
		} else {
			/* pass nonzero arg to indicate that real work needs to be done */
			vmeCommDmaIsr((void*)-1);
		}

	}

	BSP_VMEDmaInstallISR(DMACHANNEL, 0, 0);

	rtems_message_queue_delete( irqq );
	
	rtems_task_delete( RTEMS_SELF );
}

void
vmeCommDigiIsrLL(void *arg, unsigned long vec)
{
VmeDigiComm           digiComm = arg;

	vmeCommDigiIrqsLL++;

	/* Ack interrupt first -- otherwise the IRQ 
	 * remains pending...
     * Should be safe because the board is unarmed at
     * this point.
     */
	vmeDigiIrqAck(digiComm->digi);

	post_irq( arg );
}

int
vmePadRequest(int sd, int who, int type, uint32_t xid, uint32_t tsHi, uint32_t tsLo, void *cmdData, UdpCommPkt *wantReply, int timeout_ms)
{
int i,min,max;
PadReply rply;

	if ( sd != 0 ) {
		fprintf(stderr,"vmePadRequest(vmeDigiComm) unsupported sd\n");
		return -1;
	}

	if ( wantReply && PADCMD_SQRY != type ) {
		fprintf(stderr,"vmePadRequest(vmeDigiComm) does not implement replies other than queries\n");
		return -1;
	}

	if ( who >= MAX_DIGIS ) {
		fprintf(stderr,"vmePadRequest(vmeDigiComm) channel # too big (%i)\n", who);
		return -1;
	}

	if ( who < 0 ) {
		min=0;   max=MAX_DIGIS;
	} else {
		min=who; max=who+1;
	}

	timestampInfo.xid         = xid;
	timestampInfo.timestampHi = htonl(tsHi); 
	timestampInfo.timestampLo = htonl(tsLo); 

	switch ( PADCMD_GET(type) ) {
		case PADCMD_STRM:
			{
			PadStrmCommandRec *scmd_p = cmdData;
			unsigned           nsmpls = ntohl(scmd_p->nsamples);
			unsigned           nbytes = nsmpls * PADRPLY_STRM_NCHANNELS * sizeof(int16_t);

			for ( i=min; i<max; i++ ) {
				if ( ! vmeDigis[i].digi ) {
					fprintf(stderr,"vmePadRequest(vmeDigiComm) channel #%i -- no module connected\n", who);
					return -1;
				}
				if ( vmeDigis[i].nbytes != nbytes ) {
					vmeDigiSetCount(vmeDigis[i].digi, nsmpls);
					vmeDigis[i].nbytes = nbytes;
				}
				if ( ! vmeDigis[i].running ) {
					vmeDigiArm(vmeDigis[i].digi);
					vmeDigis[i].running = 1;
				}
			}

			if ( (PADCMD_STRM_FLAG_32 | PADCMD_STRM_FLAG_C1 | PADCMD_STRM_FLAG_LE) & scmd_p->flags ) {
				fprintf(stderr,"vmePadRequest(vmeDigiComm) does not implement little-endian, 32-bit data nor single-channel mode\n");
				return -1;
			}
	
			if ( ! (PADCMD_STRM_FLAG_CM & scmd_p->flags) ) {
				fprintf(stderr,"vmePadRequest(vmeDigiComm) does not implement row-major data\n");
				return -1;
			}
			}
		break;

		case PADCMD_SQRY:
			if ( (PADCMD_QUIET & type) ) {
				if ( wantReply )
					*wantReply = 0;
				return 0;
			}
			if ( ! wantReply ) {
				fprintf(stderr,"vmePadRequest(vmeDigiComm) PADCMD_SQRY with NULL wantReply??\n");
				return -1;
			}
			if ( ! (*wantReply = vmeCommAllocPacket(0)) ) {
				/* no available packets */
				return -1;
			}
			rply          = &((VmeDigiPkt)*wantReply)->pkt.padrply;
			rply->version = PADPROTO_VERSION4;
			rply->type    = PADCMD_SQRY | PADCMD_RPLY;
			rply->chnl    = min;
			rply->stat    = 0;
			rply->timestampHi = timestampInfo.timestampHi;
			rply->timestampLo = timestampInfo.timestampLo;
			rply->xid     = xid;
			rply->nBytes  = htons(sizeof(*rply));
			/* we DO support column-major, big-endian, 16-bit, 4-channel mode ONLY */ 
			rply->strm_sqry_sup_on  = PADCMD_STRM_FLAG_CM;
			rply->strm_sqry_sup_off = PADCMD_STRM_FLAG_32 | PADCMD_STRM_FLAG_C1 | PADCMD_STRM_FLAG_LE;
		break;

		case PADCMD_STOP:

			for ( i=min; i<max; i++ ) {
				if ( ! vmeDigis[i].digi ) {
					fprintf(stderr,"vmePadRequest(vmeDigiComm) channel #%i -- no module connected\n", who);
					return -1;
				}
				if ( vmeDigis[i].running ) {
					vmeDigiDisarm(vmeDigis[i].digi);
					vmeDigis[i].running = 0;
				}
			}

		break;

		case PADCMD_NOP:
		break;

		default:
		fprintf(stderr,"vmePadRequest(vmeDigiComm) does not implement cmd type %i\n",type);
		return -1;
	}
	return 0;
}

static inline int ms2ticks(int ms)
{
    if ( ms > 0 ) {
        rtems_interval rate;
        rtems_clock_get(RTEMS_CLOCK_GET_TICKS_PER_SECOND, &rate);
        if ( ms > 50000 ) {
            ms /= 1000;
            ms *= rate;
        } else {
            ms *= rate;
            ms /= 1000;
        }
        if ( 0 == ms ) {
            ms = 1;
        }
    }
    return ms;
}

UdpCommPkt
vmeCommRecv(int sd, int timeout_ms)
{
size_t            sz;
void              *p;
rtems_status_code st;
unsigned          timeout_ticks = ms2ticks(timeout_ms);

	if ( sd != 1 ) {
		fprintf(stderr,"vmeCommRecv(vmeDigiComm) bad socket sd\n");
		return 0;
	}
	st = rtems_message_queue_receive(
		sckq,
		&p,
		&sz,
		timeout_ticks ? RTEMS_WAIT : RTEMS_NO_WAIT,
		timeout_ticks > 0 ? timeout_ticks : RTEMS_NO_TIMEOUT);

	return ( RTEMS_SUCCESSFUL == st ) ? p : 0;
}

typedef union {
	char raw[sizeof(PadRequestRec) + sizeof(PadStrmCommandRec)];
	struct {
		PadRequestRec 		req;
		PadStrmCommandRec	scmd[];
	}	strmReq;
} PadReq;

int
vmeCommSend(int sd, void *buf, int len)
{
PadReq *r = buf;
int    i;

	if ( sd != 0 ) {
		fprintf(stderr,"vmeCommSend(vmeDigiComm) unsupported sd\n");
		return -1;
	}

	if ( len < sizeof(r->strmReq.req) ) {
		return -1;
	}
	if ( PADPROTO_VERSION4 != r->strmReq.req.version ) {
		fprintf(stderr,"vmeCommSend(vmeDigiComm) unsupported PAD protocol version\n");
		return -1;
	}
	if ( sizeof(r->strmReq.scmd[0]) != r->strmReq.req.cmdSize ) {
		fprintf(stderr,"vmeCommSend(vmeDigiComm) command size mismatch\n");
		return -1;
	}
	if ( r->strmReq.req.nCmds < 0 ) {
		fprintf(stderr,"vmeCommSend(vmeDigiComm) command numbers < 0 not implemented\n");
		return -1;
	}
	if ( MAX_DIGIS < r->strmReq.req.nCmds ) {
		fprintf(stderr,"vmeCommSend(vmeDigiComm) max. command number mismatch\n");
		return -1;
	}
	/* Ugly hack to pass this info :-( */
	timestampInfo.timestampHi = r->strmReq.req.timestampHi;
	timestampInfo.timestampLo = r->strmReq.req.timestampLo;
	for ( i=0; i<r->strmReq.req.nCmds; i++ ) {
		if ( vmePadRequest(
							sd,
							i,
							r->strmReq.scmd[i].type,
							r->strmReq.req.xid,
							ntohl(timestampInfo.timestampHi),
							ntohl(timestampInfo.timestampLo),			
							&r->strmReq.scmd[i],
							0,
							-1) )
			return -1;
	}
	return len;
}

static int given = 0;

int 
vmeCommSocket(int port)
{
rtems_status_code st;

	switch ( port ) {
		case PADPROTO_STRM_PORT:
			if ( sckq )
				break;
			st = rtems_message_queue_create(
					rtems_build_name('v','m','D','Q'),
					QDEPTH,
					sizeof(void*),
					RTEMS_FIFO | RTEMS_LOCAL,
					&sckq);
	
			if ( RTEMS_SUCCESSFUL != st ) {
				sckq = 0;
				break;
			}
			return 1;

		default:
			if ( given & 1 )
				break;
			given |= 1;
			return 0;
	}
	return -1;
}

int
vmeCommClose(int sd)
{
	switch ( sd ) {
		case 0:
			if ( given & 1 ) {
				given &= ~1;
				return 0;
			}
		break;

		case 1:
			if ( RTEMS_SUCCESSFUL == rtems_message_queue_delete( sckq ) ) {
				sckq = 0;
				return 0;
			}
			/* fall thru */
		default:
		break;
	}
	return -1;
}

int
vmeCommConnect(int sd, uint32_t didaddr, int port)
{
	return 0;
}


void
vmeCommFreePacket(UdpCommPkt ppacket)
{
rtems_interrupt_level l;
VmeDigiPkt            p = (VmeDigiPkt)ppacket;
	if ( p ) {
		rtems_interrupt_disable(l);
		if ( 0 == --p->digiInfo.refcnt ) {
			p->digiInfo.next     = freeList;
			p->digiInfo.digiComm = 0;
			freeList             = p;
		}
		rtems_interrupt_enable(l);
	}
}

void
vmeCommRefPacket(UdpCommPkt ppacket)
{
rtems_interrupt_level l;
VmeDigiPkt            p = (VmeDigiPkt)ppacket;
	rtems_interrupt_disable(l);
	p->digiInfo.refcnt++;
	rtems_interrupt_enable(l);
}

VmeDigi
vmeCommDigiGet(unsigned channel)
{
	return channel < MAX_DIGIS ? vmeDigis[channel].digi : 0;
}

int
vmeCommDigiConfig(unsigned channel, VME64_Addr csrbase, VME64_Addr a32base, uint8_t irq_vec, uint8_t irq_lvl, VmeDigiCommCb cb, void *usr_arg)
{
rtems_status_code  st;

VmeDigi    digi;

	if ( channel >= MAX_DIGIS ) {
		fprintf(stderr,"channel number too big\n");
		return -1;
	}
	if ( vmeDigis[channel].digi ) {
		fprintf(stderr,"channel %i already configured\n", channel);
		return -1;
	}

#if 0
	/* firmware problem: vector == slot number for now */
	if ( ((csrbase>>19) & 0x1f) != irq_vec ) {
		fprintf(stderr,"Warning: firmware restriction -- irq_vec must be == VME slot number\n");
	}
#endif

	if ( !(digi = vmeDigiSetup(csrbase, a32base, irq_vec, irq_lvl)) ) {
		/* more info should have been printed */
		fprintf(stderr,"vmeDigiSetup() failed\n");
		return -1;
	}

	/* No thread safety; assume 'Config' is called during initialization */
	if ( !dmaIsrInstalled ) {

		BSP_VMEDmaSetup(DMACHANNEL, BSP_VMEDMA_OPT_THROUGHPUT, VME_AM_EXT_SUP_MBLT, 0);

		if ( BSP_VMEDmaInstallISR(DMACHANNEL, vmeCommDmaIsr, 0) ) {
			fprintf(stderr,"unable to install DMA ISR\n");
			return -1;
		}
		dmaIsrInstalled = 1;
	}

	vmeDigis[channel].digi     = digi;
	vmeDigis[channel].vmeaddr  = a32base;
	vmeDigis[channel].vec      = irq_vec;
	vmeDigis[channel].nbytes   = 0;
	vmeDigis[channel].running  = 0;
	vmeDigis[channel].channel  = channel;

	if ( cb )
		vmeDigis[channel].cb   = *cb;
	else
		memset( &vmeDigis[channel].cb, 0, sizeof( vmeDigis[channel].cb ) );
	vmeDigis[channel].usr_arg  = usr_arg;

	cb = &vmeDigis[channel].cb;

	/* Check if we need to start the task */
	if ( ! irqq && ( cb->acq_done || cb->dma_done ) ) {
		/* Yes */
		st = rtems_task_create(
				rtems_build_id('D','I','C','O'), 
				vmeCommTaskPriority,
				1024,
				RTEMS_DEFAULT_MODES,
				RTEMS_DEFAULT_ATTRIBUTES,
				&task);
				
		if ( RTEMS_SUCCESSFUL != st ) {
			fprintf(stderr,"vmeDigiComm: FATAL ERROR - unable to create task: %s\n", rtems_status_text(st));
			return -1;
		}

		st = rtems_message_queue_create(
				rtems_build_name('v','m','D','I'),
				MAX_BPM + 2, /* 1 per digi, DMA + kill message */
				sizeof(VmeDigiComm),
				RTEMS_FIFO | RTEMS_LOCAL,
				&irqq);

		if ( RTEMS_SUCCESSFUL != st ) {
			fprintf(stderr,"vmeDigiComm: FATAL ERROR - unable to create irqq: %s\n", rtems_status_text(st));
			rtems_task_delete( task );
			irqq = 0;
			task = 0;
			return -1;
		}
		
		st = rtems_task_start( task, vmeCommThread, 0 );

		if ( RTEMS_SUCCESSFUL != st ) {
			fprintf(stderr,"vmeDigiComm: FATAL ERROR - unable to start task %s\n", rtems_status_text(st));
			rtems_task_delete( task );
			task = 0;
			rtems_message_queue_delete( irqq );
			irqq = 0;
			return -1;
		}
	}

	/* Do we need to use task driven DMA ? */
	if ( cb->dma_done )
		dmaIsrInstalled = 2;

	/* Only install the low-level ISR if they have an 'acq_done' callback 
	 * otherwise use the vmeCommDigiIsr directly.
     */
	if ( BSP_installVME_isr(irq_vec, cb->acq_done ? vmeCommDigiIsrLL : vmeCommDigiIsr, &vmeDigis[channel]) ) {
		fprintf(stderr, "unable to install DIGI ISR\n");
		vmeDigis[channel].digi = 0;
		return -1;
	}

	if ( cb->cfg_done )
		cb->cfg_done(channel, digi, &vmeDigis[channel].usr_arg);

	vmeDigiIrqEnable(digi);

	BSP_enableVME_int_lvl(irq_lvl);

	return 0;
}

/* Note: the two lsb cannot be written; this means that 
 *       an overflow/-range situation cannot be simulated!
 */
int
vmeDigiCommSetSimMode(int channel, void *data, int nbytes)
{
unsigned nsmpls;
unsigned long a;
unsigned key;

/* Endianness-tester */
const union {
	short s;
	char  c[2];
} isLE = { s: 1 };

	if ( channel < 0 || channel >= MAX_BPM ) {
		fprintf(stderr,"Invalid channel # %i\n",channel);
		return -1;
	}
	if ( 0 == vmeDigis[channel].digi ) {
		fprintf(stderr,"Channel # %i not configured\n",channel);
		return -1;
	}

	nsmpls = vmeDigis[channel].nbytes / PADRPLY_STRM_NCHANNELS / sizeof(int16_t);

	if ( nsmpls > 0 ) {
		if ( BSP_vme2local_adrs(VME_AM_EXT_SUP_DATA, vmeDigis[channel].vmeaddr, &a) ) {
			fprintf(stderr,"Unable to map VME address to PCI\n");
			return -1;
		}

		if ( isLE.c[0] ) {
			fprintf(stderr,"vmeDigiCommSetSimMode: little-endian CPU support is not implemented\n");
			return -1;
		}
		a = BSP_PCI2LOCAL_ADDR(a);

		/* sim-mode on */
		/*
		vmeDigiSetCount(vmeDigis[channel].digi, 1);
		*/

		if ( nbytes > MAXBYTES - vmeDigis[channel].nbytes ) {
			nbytes = MAXBYTES - vmeDigis[channel].nbytes;
		}
		nbytes = nbytes - (nbytes % vmeDigis[channel].nbytes);

		memcpy((void*)(a + vmeDigis[channel].nbytes), data, nbytes);

		vmeDigis[channel].simEnd = nbytes + vmeDigis[channel].nbytes;

		rtems_interrupt_disable(key);
			vmeDigis[channel].simIdx = vmeDigis[channel].nbytes;
		rtems_interrupt_enable(key);

	} else {
		/* sim-mode off */
		rtems_interrupt_disable(key);
			vmeDigis[channel].simIdx = 0;
		rtems_interrupt_enable(key);

		vmeDigis[channel].simEnd = 0;
		/* reset count  */
		/*
		vmeDigiSetCount(vmeDigis[channel].digi, nsmpls);
		*/
	}

	return 0;
}

void
vmeCommDigiCleanup()
{
int     i;
VmeDigi digi;

	vmeCommClose(0);
	vmeCommClose(1);

	/* assume DMA is quiet */
	for ( i=0; i<MAX_DIGIS; i++ ) {
		if ( (digi = vmeDigis[i].digi) ) {
			vmeDigis[i].running = 0;
			vmeDigiDisarm(digi);
			vmeDigiIrqDisable(digi);
			BSP_removeVME_isr(vmeDigis[i].vec, vmeDigis[i].cb.acq_done ? vmeCommDigiIsrLL : vmeCommDigiIsr, vmeDigis +i );
			vmeDigis[i].digi    = 0;
		}
	}

	if ( irqq ) {
		/* send 'KILL' message */
		rtems_message_queue_send( irqq, 0, 0 );
		/* should synchronize with task exit but we dont... */
	} else {
		BSP_VMEDmaInstallISR(DMACHANNEL, 0, 0);
	}
}

void * 
vmeCommBufPtr(UdpCommPkt p)
{
	return &((VmeDigiPkt)p)->pkt;
}

static DrvPadUdpCommIORec io = {
	open:     vmeCommSocket,
	close:    vmeCommClose,
	connect:  vmeCommConnect,
	send:     vmeCommSend,
	recv:     vmeCommRecv,
	bufptr:   vmeCommBufPtr,
	alloc:    vmeCommAllocPacket,
	creatref: vmeCommRefPacket,
	free:     vmeCommFreePacket,
	padIoReq: vmePadRequest,
};

DrvPadUdpCommIO drvPadVmeCommIO = &io;

static void
noop_cfg_done(unsigned channel, VmeDigi digi, void **usr_arg_p)
{
}

static void
noop_acq_done(unsigned channel, VmeDigi digi, void *usr_arg, int dmaPending)
{
}

static int
noop_dma_done(unsigned channel, VmeDigi digi, void *usr_arg, PadReply rply)
{
	return 0;
}

/* 'No-op' callbacks to test task-driven mode */
static VmeDigiCommCbRec noopCbs = {
	cfg_done: noop_cfg_done,
	acq_done: noop_acq_done,
	dma_done: noop_dma_done
};

VmeDigiCommCb vmeCommNoopCbs = &noopCbs;

#ifdef DEBUG
PadStrmCommandRec vmeCommDbgStrmCmd =
{
	type:     PADCMD_STRM,
	flags:    PADCMD_STRM_FLAG_CM,
};

int
vmeCommDbgStrmStart(int channel, unsigned nsamples)
{
	vmeCommDbgStrmCmd.nsamples = htonl(nsamples);
	return vmePadRequest(0, channel, PADCMD_STRM, 0, 0, 0, &vmeCommDbgStrmCmd, 0, 0);
}

#endif
