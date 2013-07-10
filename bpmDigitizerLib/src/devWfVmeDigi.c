/* $Id: devWfVmeDigi.c,v 1.6 2012/03/02 00:58:28 sonya Exp $ */

#include <stdlib.h>
#include <string.h>

#include <epicsMessageQueue.h>
#include <epicsThread.h>
#include <dbAccess.h>
#include <devSup.h>
#include <recSup.h>
#include <dbEvent.h>
#include <epicsExport.h>
#include <errlog.h>

#include <waveformRecord.h>

#include <devVmeDigiSupport.h>
#include <devWfVmeDigiSup.h>

typedef struct VmeDigiDPVT_ {
	int                  orarm;
	struct VmeDigiCard_ *card;
} VmeDigiDPVT;

epicsMessageQueueId devWfVmeDigiQ;
epicsThreadId       devWfVmeDigiT;

static void
devWfVmeDigiTsk(void *arg)
{
VmeDigiCard             *c;
int                     i,j,k;
register unsigned short *sp, *dp;
waveformRecord          *wf;

/* Endianness-tester */
const union {
	short s;
	char  c[2];
} isLE = { s: 1 };

	while ( 1 ) {
		epicsMessageQueueReceive(devWfVmeDigiQ, &c, sizeof(c));

		wf = (waveformRecord *)c->prec;

		/* Channel layout in memory is reversed: CH4, CH3, CH2, CH1... */
		sp = (unsigned short*) c->local_a32_addr + NCHAN;
		dp = (unsigned short*) wf->bptr;

		if ( isLE.c[0] ) {
			/* FIXME: this should be optimized if used on a little-endian CPU */	
			static int warn = 0;
			if ( !warn ) {
				warn=1;
				errlogPrintf("WARNING: byte-swapping algorithm (%s:%u) should be improved !\n",
							__FILE__,__LINE__);
			}
			for ( j=k=0; j<NCHAN; j++ ) {
				--sp;
				for ( i=0; i<wf->nelm; i+=NCHAN, k++ ) {
					register unsigned short v = sp[i];
					dp[k] = (v>>8) | (v<<8);
				}
			}
		} else {
			/* FIXME: could use DMA here */
			for ( j=k=0; j<NCHAN; j++ ) {
				--sp;
				for ( i=0; i<wf->nelm; i+=NCHAN, k++ ) {
					register unsigned short v = sp[i];
					dp[k] = v;
				}
			}
		}

		/* 2nd phase of async processing */
		dbScanLock(c->prec);
	    (*c->prec->rset->process)(c->prec);
    	dbScanUnlock(c->prec);
	}
}

static void report(int interest_level)
{
	errlogPrintf("Waveform Device support for 16-bit/130MSPS VME Digitizer\n");
}

static void init(int phase)
{
	if ( 0 == phase ) {
		devWfVmeDigiQ = epicsMessageQueueCreate( MAX_DIGIS, sizeof(VmeDigiCard *) );
		devWfVmeDigiT = epicsThreadCreate(
							"devWfVmeDigiT",
							epicsThreadPriorityLow, 
							epicsThreadGetStackSize(epicsThreadStackMedium),
							devWfVmeDigiTsk,
							0);
	}
}

static long init_record(struct waveformRecord *prec)
{
VmeDigiDPVT *dpvt;
int          idx;

	idx = devWfVmeDigiSupArgcheck(prec, "devWfVmeDigi", 1);

	if ( idx < 0 )
		goto bail;

	if ( prec->nelm > NCHAN*devVmeDigis[idx-1].size ) {
		errlogPrintf("devWfVmeDigi.init_record(): NELM too big\n");
		goto bail;
	}

	vmeDigiSetCount( devVmeDigis[idx-1].digi, prec->nelm/NCHAN );

	devVmeDigis[idx-1].prec = (dbCommon*)prec;
	

	if ( ! (dpvt = malloc(sizeof(*dpvt))) ) {
		errlogPrintf("No memory for VmeDigiDPVT\n");
		goto bail;
	}


	dpvt->orarm = prec->rarm;
	/* convenience pointer back to card struct */
	dpvt->card = &devVmeDigis[idx-1];

	prec->dpvt = dpvt;

	if ( dpvt->orarm ) { 
		if ( dpvt->orarm < 0 )
			vmeDigiSWTrig( dpvt->card->digi );
		else
			vmeDigiArm( dpvt->card->digi );
		dpvt->card->pending = 0;
		vmeDigiIrqEnable( dpvt->card->digi );
	}

	return 0;

bail:
	prec->pact = TRUE;
	return -1;
}

static long read_waveform(struct waveformRecord *prec)
{
VmeDigiDPVT *dpvt     = prec->dpvt;
VmeDigiCard *card     = dpvt->card;
unsigned     post     = 0;
int          needsArm = 0;
int          nord     = prec->nord;

	if ( prec->pact ) {
		/* phase 2 completion */
		prec->udf = FALSE;

		/* do we have to rearm */
		if ( prec->rarm > 1 ) {
			/* auto-rearm */
			needsArm = prec->rarm;
		} else {
			/* disarm (leave IRQ disabled) */
			prec->rarm = dpvt->orarm = 0;
			post |= 2;
		}
		
		nord = prec->nelm;

	} else {
#if 0
		if ( vmeDigiIrqAck( card->digi ) )
#else
		if ( card->pending )
#endif
		{
			/* phase 1 of async. processing  */
			prec->pact = TRUE;

			card->pending = 0;
			/* post request to task */
			epicsMessageQueueSend( devWfVmeDigiQ, &card, sizeof(card));

			return 0;
		}
	}

	if ( prec->rarm != dpvt->orarm ) {
		/* RARM changed */
		if ( ! (needsArm = prec->rarm) )
			vmeDigiDisarm( card->digi );
		dpvt->orarm = prec->rarm;
	}

	if ( needsArm ) {
		if ( needsArm < 0 )
			vmeDigiSWTrig( card->digi );
		else
			vmeDigiArm( card->digi );

		/* don't reset NORD in auto-rearm mode */
		if ( needsArm <= 1 )
			nord = 0;

		card->pending = 0;
		vmeDigiIrqEnable( card->digi );
	}

	if ( nord != prec->nord ) {
		prec->nord = nord;
        db_post_events(prec, &prec->nord, (DBE_VALUE|DBE_LOG));
	}

	if ( post & 2 )
        db_post_events(prec, &prec->rarm, (DBE_VALUE|DBE_LOG));

	return 0;
}

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read;
} devWfVmeDigi = {
	5,
	(DEVSUPFUN)report,
	(DEVSUPFUN)init,
	(DEVSUPFUN)init_record,
	(DEVSUPFUN)0,
	(DEVSUPFUN)read_waveform
};

epicsExportAddress(dset, devWfVmeDigi);
