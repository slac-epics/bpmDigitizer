#include <stdint.h>
#include <string.h>

#include <rtems.h>
#include <bsp.h>
#include <bsp/VME.h>
#include <bsp/VMEDMA.h>

#include <udpComm.h>
#include <padProto.h>
#include <padStream.h>
#if 0
#include <drvPadUdpComm.h>
#else
#warning "FIXME should use MAX_BPM from drvPadUdpComm.h"
#define MAX_BPM 25
#endif

#define VMEDIGI_NCHANNELS	4

#define DEBUG

#define BYTES_PER_SAMPLE	(VMEDIGI_NCHANNELS/* channels */ * sizeof(int16_t)/* bytes */)

#define MAXBYTES            (VMEDIGI_CNT_MAX * BYTES_PER_SAMPLE)

#define DMACHANNEL      0

#define MAX_DIGIS		(MAX_BPM  )
#define QDEPTH      	(MAX_BPM*2)
#define MAX_PKTBUFS		(MAX_BPM*5)


#include <vmeDigi.h>

/* cached value of # samples     */
typedef struct VmeDigiCommRec_ {
	VmeDigi		digi;
	uint32_t	vmeaddr;
	unsigned    vec;
	uint16_t    nbytes;
	int     	running;
	int     	channel;
	unsigned	simIdx;
	unsigned	simEnd;
} VmeDigiCommRec, *VmeDigiComm;

typedef union VmeDigiPktRec_ {
	/* we can put PktRects on a linked list;
	 * the 'next' pointer overlaps the Ethernet
	 * header of 'pkt' but that is OK since
	 * all headers (Ethernet/IP/UDP) are unused
	 * anyways.
	 */
	struct {
		union VmeDigiPktRec_	*next;
		VmeDigiComm              digiComm;
	}                        digiInfo;
	LanIpPacketRec			 pkt;
} VmeDigiPktRec, *VmeDigiPkt;

/* align to cache block size but at least 16-byte 
 * in case we want to use AltiVec.
 */
static VmeDigiPktRec	bufs[MAX_PKTBUFS] __attribute__((aligned(32))) = { {{0}} };
static int        bufsUsed = 0;

static VmeDigiPkt freeList = 0;

static rtems_id msgq = 0;

static struct {
	uint32_t	timestampHi;
	uint32_t	timestampLo;
	uint32_t	xid;
} timestampInfo;

VmeDigiPkt
vmeDigiPktAlloc(VmeDigiComm digiComm)
{
VmeDigiPkt            rval = 0;
rtems_interrupt_level l;
PadReply              rply;

	rtems_interrupt_disable(l);
		if ( freeList ) {
			rval     = freeList;
			freeList = rval->digiInfo.next;
			rval->digiInfo.next = 0;
		} else if ( bufsUsed < MAX_PKTBUFS ) {
			rval                    = &bufs[bufsUsed++];
		}
	rtems_interrupt_enable(l);

	if ( rval ) {

		rval->digiInfo.next     = 0;
		rval->digiInfo.digiComm = digiComm;

		rply                    = (PadReply)lpkt_udphdr(&rval->pkt).pld;

		/* Fill in timestamps + other info from request */
		rply->version           = PADPROTO_VERSION1;
		rply->type              = PADCMD_STRM;
		rply->chnl              = digiComm->channel;
		rply->nBytes            = htons(digiComm->nbytes + sizeof(PadReplyRec));
		rply->timestampHi       = timestampInfo.timestampHi;
		rply->timestampLo       = timestampInfo.timestampLo;
		rply->xid               = timestampInfo.xid; 
		rply->status            = htons(0);
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
volatile unsigned vmeCommPktsDroppedNoQSpc  = 0;
volatile unsigned vmeCommPktsDroppedBadDma  = 0;
volatile unsigned vmeCommPktsDroppedBadDmaStatus  = 0;

volatile unsigned vmeCommDigiIrqs           = 0;
volatile unsigned vmeCommDmaIrqs            = 0;

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
	rply     = (PadReply)lpkt_udphdr(&dmaInProgress->pkt).pld;
	if ( BSP_VMEDmaStart(DMACHANNEL, BSP_LOCAL2PCI_ADDR(&rply->data[0]), digiComm->vmeaddr + digiComm->simIdx, digiComm->nbytes) ) {
		vmeCommPktsDroppedBadDma++;
		udpSockFreeBuf((LanIpPacket)dmaInProgress);

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

void
vmeCommDigiIsr(void *arg, unsigned long vec)
{
VmeDigiComm           digiComm = arg;
VmeDigiPkt            p;
rtems_interrupt_level l;

	vmeCommDigiIrqs++;
	
	if ( ! ( p = vmeDigiPktAlloc(digiComm) ) ) {
		/* no buffer; must drop this packet */
		vmeCommPktsDroppedNoBuf++;
		rearm(digiComm);
		return;
	}

	/* Ack interrupt first -- otherwise we're stuck
     * until the DMA controller releases the bus.
     * should be safe because the board is unarmed at
     * this point.
     */
	vmeDigiIrqAck(digiComm->digi);

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

}

void
vmeCommDmaIsr(void *arg)
{
rtems_status_code     st;
VmeDigiComm           digiComm = dmaInProgress->digiInfo.digiComm;
uint32_t              dma_status;

	vmeCommDmaIrqs++;

	if ( (dma_status = BSP_VMEDmaStatus(DMACHANNEL)) ) {
		vmeCommLastBadDmaStatus = dma_status;
		vmeCommPktsDroppedBadDmaStatus++;	
		udpSockFreeBuf((LanIpPacket)dmaInProgress);
	} else {

		st = rtems_message_queue_send(msgq, (void*)&dmaInProgress, sizeof(dmaInProgress));

		if ( RTEMS_SUCCESSFUL != st ) {
			/* this should include the case when there is no queue */
			vmeCommPktsDroppedNoQSpc++;
			udpSockFreeBuf((LanIpPacket)dmaInProgress);
		}
	}

	rearm(digiComm);

	if ( moreDmaNeeded() )
		startDma();
}

int
padRequest(int sd, int who, int type, uint32_t xid, void *cmdData, UdpCommPkt *wantReply, int timeout_ms)
{
int i,min,max;

	if ( sd != 0 ) {
		fprintf(stderr,"padRequest(vmeDigiComm) unsupported sd\n");
		return -1;
	}

	if ( wantReply ) {
		fprintf(stderr,"padRequest(vmeDigiComm) does not implement replies\n");
		return -1;
	}

	if ( who >= MAX_DIGIS ) {
		fprintf(stderr,"padRequest(vmeDigiComm) channel # too big (%i)\n", who);
		return -1;
	}

	if ( who < 0 ) {
		min=0;   max=MAX_DIGIS;
	} else {
		min=who; max=who+1;
	}

	timestampInfo.xid = xid;

	switch ( PADCMD_GET(type) ) {
		case PADCMD_STRM:
			{
			PadStrmCommandRec *scmd_p = cmdData;
			unsigned           nsmpls = ntohl(scmd_p->nsamples);
			unsigned           nbytes = nsmpls * PADRPLY_STRM_NCHANNELS * sizeof(int16_t);

			for ( i=min; i<max; i++ ) {
				if ( ! vmeDigis[i].digi ) {
					fprintf(stderr,"padRequest(vmeDigiComm) channel #%i -- no module connected\n", who);
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

			if ( PADCMD_STRM_FLAG_LE & scmd_p->flags ) {
				fprintf(stderr,"padRequest(vmeDigiComm) does not implement little-endian data\n");
				return -1;
			}
	
			if ( ! (PADCMD_STRM_FLAG_CM & scmd_p->flags) ) {
				fprintf(stderr,"padRequest(vmeDigiComm) does not implement row-major data\n");
				return -1;
			}
			}
		break;

		case PADCMD_STOP:

			for ( i=min; i<max; i++ ) {
				if ( ! vmeDigis[i].digi ) {
					fprintf(stderr,"padRequest(vmeDigiComm) channel #%i -- no module connected\n", who);
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
		fprintf(stderr,"padRequest(vmeDigiComm) does not implement cmd type %i\n",type);
		return -1;
	}
	return 0;
}

LanIpPacketRec *
udpSockRecv(int sd, int timeout_ticks)
{
size_t            sz;
void              *p;
rtems_status_code st;

	if ( sd != 1 ) {
		fprintf(stderr,"udpSockRecv(vmeDigiComm) bad socket sd\n");
		return 0;
	}
	st = rtems_message_queue_receive(
		msgq,
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
udpSockSend(int sd, void *buf, int len)
{
PadReq *r = buf;
int    i;

	if ( sd != 0 ) {
		fprintf(stderr,"udpSockSend(vmeDigiComm) unsupported sd\n");
		return -1;
	}

	if ( len < sizeof(r->strmReq.req) ) {
		return -1;
	}
	if ( PADPROTO_VERSION1 != r->strmReq.req.version ) {
		fprintf(stderr,"udpSockSend(vmeDigiComm) unsupported PAD protocol version\n");
		return -1;
	}
	if ( sizeof(r->strmReq.scmd[0]) != r->strmReq.req.cmdSize ) {
		fprintf(stderr,"udpSockSend(vmeDigiComm) command size mismatch\n");
		return -1;
	}
	if ( r->strmReq.req.nCmds < 0 ) {
		fprintf(stderr,"udpSockSend(vmeDigiComm) command numbers < 0 not implemented\n");
		return -1;
	}
	if ( MAX_DIGIS < r->strmReq.req.nCmds ) {
		fprintf(stderr,"udpSockSend(vmeDigiComm) max. command number mismatch\n");
		return -1;
	}
	/* Ugly hack to pass this info :-( */
	timestampInfo.timestampHi = r->strmReq.req.timestampHi;
	timestampInfo.timestampLo = r->strmReq.req.timestampLo;
	for ( i=0; i<r->strmReq.req.nCmds; i++ ) {
		if ( padRequest(sd, i, r->strmReq.scmd[i].type, r->strmReq.req.xid, &r->strmReq.scmd[i], 0, -1) )
			return -1;
	}
	return len;
}

static int given = 0;

int 
udpSockCreate(int port)
{
rtems_status_code st;

	switch ( port ) {
		case PADPROTO_STRM_PORT:
			if ( msgq )
				break;
			st = rtems_message_queue_create(
					rtems_build_name('v','m','D','Q'),
					QDEPTH,
					sizeof(void*),
					RTEMS_FIFO | RTEMS_LOCAL,
					&msgq);
	
			if ( RTEMS_SUCCESSFUL != st ) {
				msgq = 0;
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
udpSockDestroy(int sd)
{
	switch ( sd ) {
		case 0:
			if ( given & 1 ) {
				given &= ~1;
				return 0;
			}
		break;

		case 1:
			if ( RTEMS_SUCCESSFUL == rtems_message_queue_delete( msgq ) ) {
				msgq = 0;
				return 0;
			}
			/* fall thru */
		default:
		break;
	}
	return -1;
}

int
udpSockConnect(int sd, uint32_t didaddr, int port)
{
	return 0;
}


void
udpSockFreeBuf(LanIpPacketRec *ppacket)
{
rtems_interrupt_level l;
VmeDigiPkt            p = (VmeDigiPkt)ppacket;
	if ( p ) {
		rtems_interrupt_disable(l);
		p->digiInfo.next = freeList;
		freeList         = p;
		rtems_interrupt_enable(l);
	}
}

int
vmeCommDigiConfig(unsigned channel, VME64_Addr csrbase, VME64_Addr a32base, uint8_t irq_vec, uint8_t irq_lvl)
{
static int dmaIsrInstalled = 0;

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

	if ( BSP_installVME_isr(irq_vec, vmeCommDigiIsr, &vmeDigis[channel]) ) {
		fprintf(stderr, "unable to install DIGI ISR\n");
		vmeDigis[channel].digi = 0;
		return -1;
	}
	vmeDigiIrqEnable(digi);

	BSP_enableVME_int_lvl( irq_lvl );

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

	if ( channel < 0 || channel >= MAX_BPM ) {
		fprintf(stderr,"Invalid channel # %i\n",channel);
		return -1;
	}
	if ( 0 == vmeDigis[channel].digi ) {
		fprintf(stderr,"Channel # %i not configured\n",channel);
		return -1;
	}

	nsmpls = vmeDigis[channel].nbytes / PADRPLY_STRM_NCHANNELS / sizeof(int16_t);

	if ( nbytes > 0 ) {
		if ( BSP_vme2local_adrs(VME_AM_EXT_SUP_DATA, vmeDigis[channel].vmeaddr, &a) ) {
			fprintf(stderr,"Unable to map VME address to PCI\n");
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

	udpSockDestroy(0);
	udpSockDestroy(1);

	/* assume DMA is quiet */
	for ( i=0; i<MAX_DIGIS; i++ ) {
		if ( (digi = vmeDigis[i].digi) ) {
			vmeDigis[i].running = 0;
			vmeDigiDisarm(digi);
			vmeDigiIrqDisable(digi);
			BSP_removeVME_isr(vmeDigis[i].vec, vmeCommDigiIsr, vmeDigis +i );
			vmeDigis[i].digi    = 0;
		}
	}

	BSP_VMEDmaInstallISR(DMACHANNEL, 0, 0);
}

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
	return padRequest(0, channel, PADCMD_STRM, 0, &vmeCommDbgStrmCmd, 0, 0);
}

#endif
