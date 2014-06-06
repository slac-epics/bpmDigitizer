/* $Id: devWfVmeDigi.c,v 1.6 2012/03/02 00:58:28 sonya Exp $ */

#include <stdio.h>
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
#define	TRACK_HI_RES_DUR	1
#ifdef	TRACK_HI_RES_DUR
#include "HiResTime.h"
#endif	/*	TRACK_HI_RES_DUR	*/

typedef struct VmeDigiDPVT_ {
	int						orarm;
	struct VmeDigiCard_	*	card;
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
							epicsThreadPriorityHigh, 
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
		/* Initial arming of digitizer from init_record() */
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

#ifdef	TRACK_HI_RES_DUR
extern t_HiResTime		hiResTickDigiIsrPrior;
extern t_HiResTime		hiResDurDigiIsrMin;
extern t_HiResTime		hiResDurDigiIsrMax;
extern unsigned int		nTicksHiResDurLate;
extern unsigned int		nLateDigiIsr;
t_HiResTime				hiResTickDigiWave1Prior		= 0LL;
t_HiResTime				hiResTickDigiWave2Prior		= 0LL;
t_HiResTime				hiResTickDigiReArmPrior		= 0LL;
t_HiResTime				hiResDurDigiWave1Min		= 1000000LL;
t_HiResTime				hiResDurDigiWave1Max		= 0LL;
t_HiResTime				hiResDurDigiWave2Min		= 1000000LL;
t_HiResTime				hiResDurDigiWave2Max		= 0LL;
t_HiResTime				hiResDurDigiReArmMin		= 1000000LL;
t_HiResTime				hiResDurDigiReArmMax		= 0LL;
t_HiResTime				hiResDurDigiIsrToWave1Cum	= 0LL;
t_HiResTime				hiResDurDigiIsrToWave1Min	= 1000000LL;
t_HiResTime				hiResDurDigiIsrToWave1Max	= 0LL;
t_HiResTime				hiResDurDigiIsrToWave2Cum	= 0LL;
t_HiResTime				hiResDurDigiIsrToWave2Min	= 1000000LL;
t_HiResTime				hiResDurDigiIsrToWave2Max	= 0LL;
t_HiResTime				hiResDurDigiIsrToReArmCum	= 0LL;
t_HiResTime				hiResDurDigiIsrToReArmMin	= 1000000LL;
t_HiResTime				hiResDurDigiIsrToReArmMax	= 0LL;
t_HiResTime				hiResDurDigiWave1ToWave2Min	= 1000000LL;
t_HiResTime				hiResDurDigiWave1ToWave2Max	= 0LL;
unsigned int			nLateDigiWave1				= 0;
unsigned int			nLateDigiWave2				= 0;
unsigned int			nLateDigiReArm				= 0;
unsigned int			nDigiWave1					= 0;
unsigned int			nTicksHiResReArmLate		= 277583;

int ShowHiResDurDigiWave( unsigned int fReset )
{
	if ( nDigiWave1 == 0 )
	{
		printf( "No waveforms captured yet!\n" );
		return 0;
	}

	printf( "DigiIsr:      HiResDurMin = %.6e, HiResDurMax = %.6e, %u late\n",
			HiResTicksToSeconds( hiResDurDigiIsrMin ),
			HiResTicksToSeconds( hiResDurDigiIsrMax ), nLateDigiIsr	);
	printf( "DigiWave1:    Captured %d waveforms\n", nDigiWave1 );
	printf( "DigiWave1:    HiResDurMin = %.6e, HiResDurMax = %.6e, %u more than 10ms\n",
			HiResTicksToSeconds( hiResDurDigiWave1Min ),
			HiResTicksToSeconds( hiResDurDigiWave1Max ), nLateDigiWave1	);
	printf( "DigiWave2:    HiResDurMin = %.6e, HiResDurMax = %.6e, %u more than 10ms\n",
			HiResTicksToSeconds( hiResDurDigiWave2Min ),
			HiResTicksToSeconds( hiResDurDigiWave2Max ), nLateDigiWave2	);
	printf( "Wave1ToWave2: HiResDurMin = %.6e, HiResDurMax = %.6e\n",
			HiResTicksToSeconds( hiResDurDigiWave1ToWave2Min ),
			HiResTicksToSeconds( hiResDurDigiWave1ToWave2Max )	);
	printf( "IsrToWave1:   HiResDurMin = %.6e, HiResDurMax = %.6e, HiResDurAvg = %.6e\n",
			HiResTicksToSeconds( hiResDurDigiIsrToWave1Min ),
			HiResTicksToSeconds( hiResDurDigiIsrToWave1Max ),
			HiResTicksToSeconds( hiResDurDigiIsrToWave1Cum / nDigiWave1 )	);
	printf( "IsrToWave2:   HiResDurMin = %.6e, HiResDurMax = %.6e, HiResDurAvg = %.6e\n",
			HiResTicksToSeconds( hiResDurDigiIsrToWave2Min ),
			HiResTicksToSeconds( hiResDurDigiIsrToWave2Max ),
			HiResTicksToSeconds( hiResDurDigiIsrToWave2Cum / nDigiWave1 )	);
	printf( "IsrToReArm:   HiResDurMin = %.6e, HiResDurMax = %.6e, HiResDurAvg = %.6e, %d late\n",
			HiResTicksToSeconds( hiResDurDigiIsrToReArmMin ),
			HiResTicksToSeconds( hiResDurDigiIsrToReArmMax ),
			HiResTicksToSeconds( hiResDurDigiIsrToReArmCum / nDigiWave1 ), nLateDigiReArm	);
	if ( fReset )
	{
		hiResDurDigiIsrMin			= 1000000LL;
		hiResDurDigiIsrMax			= 0LL;
		hiResDurDigiWave1Min		= 1000000LL;
		hiResDurDigiWave1Max		= 0LL;
		hiResDurDigiWave2Min		= 1000000LL;
		hiResDurDigiWave2Max		= 0LL;
		hiResDurDigiIsrToWave1Cum	= 1000000LL;
		hiResDurDigiIsrToWave1Min	= 1000000LL;
		hiResDurDigiIsrToWave1Max	= 0LL;
		hiResDurDigiIsrToWave2Cum	= 1000000LL;
		hiResDurDigiIsrToWave2Min	= 1000000LL;
		hiResDurDigiIsrToWave2Max	= 0LL;
		hiResDurDigiIsrToReArmCum	= 1000000LL;
		hiResDurDigiIsrToReArmMin	= 1000000LL;
		hiResDurDigiIsrToReArmMax	= 0LL;
		hiResDurDigiWave1ToWave2Min	= 1000000LL;
		hiResDurDigiWave1ToWave2Max	= 0LL;
		nLateDigiIsr				= 0;
		nLateDigiWave1				= 0;
		nLateDigiWave2				= 0;
		nLateDigiReArm				= 0;
		nDigiWave1					= 0;
	}
	return 0;
}
#endif	/*	TRACK_HI_RES_DUR	*/

static long read_waveform(struct waveformRecord *prec)
{
VmeDigiDPVT *dpvt     = prec->dpvt;
VmeDigiCard *card     = dpvt->card;
unsigned     post     = 0;
int          needsArm = 0;
int          nord     = prec->nord;

	if ( prec->pact ) {
#ifdef	TRACK_HI_RES_DUR
		t_HiResTime		hiResTickDigiWave2 = GetHiResTicks();
		t_HiResTime		hiResDur = hiResTickDigiWave2 - hiResTickDigiWave2Prior;
		if( hiResDurDigiWave2Max < hiResDur )
			hiResDurDigiWave2Max = hiResDur;
		if( hiResDurDigiWave2Min > hiResDur )
			hiResDurDigiWave2Min = hiResDur;
		hiResTickDigiWave2Prior	=  hiResTickDigiWave2;

		// Count the number of Wave2 intervals over 10ms
		nTicksHiResDurLate = (HiResTicksPerSecond() * 0.01);
		if ( hiResDur > nTicksHiResDurLate )
			nLateDigiWave2++;

		hiResDur = hiResTickDigiWave2 - hiResTickDigiWave1Prior;
		if( hiResDurDigiWave1ToWave2Max < hiResDur )
			hiResDurDigiWave1ToWave2Max = hiResDur;
		if( hiResDurDigiWave1ToWave2Min > hiResDur )
			hiResDurDigiWave1ToWave2Min = hiResDur;

		hiResDur = hiResTickDigiWave2 - hiResTickDigiIsrPrior;
		hiResDurDigiIsrToWave2Cum += hiResDur;
		if( hiResDurDigiIsrToWave2Max < hiResDur )
			hiResDurDigiIsrToWave2Max = hiResDur;
		if( hiResDurDigiIsrToWave2Min > hiResDur )
			hiResDurDigiIsrToWave2Min = hiResDur;
#endif	/*	TRACK_HI_RES_DUR	*/

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
		/* The ISR routine, devVmeDigiIsr(), sets pending if
		 * the interrupt was from the digi.
		 * It also posts a callback request to the EPICS record
		 * processing queue, which in turn calls this function.
		 */
		if ( card->pending )
		{
#ifdef	TRACK_HI_RES_DUR
			t_HiResTime		hiResTickDigiWave1 = GetHiResTicks();
			t_HiResTime		hiResDur = hiResTickDigiWave1 - hiResTickDigiWave1Prior;
			nDigiWave1++;
			if( hiResDurDigiWave1Max < hiResDur )
				hiResDurDigiWave1Max = hiResDur;
			if( hiResDurDigiWave1Min > hiResDur )
				hiResDurDigiWave1Min = hiResDur;
			hiResTickDigiWave1Prior	=  hiResTickDigiWave1;

			// Count the number of Wave1 intervals over 10ms
			if ( hiResDur > nTicksHiResDurLate )
				nLateDigiWave1++;

			hiResDur = hiResTickDigiWave1 - hiResTickDigiIsrPrior;
			hiResDurDigiIsrToWave1Cum += hiResDur;
			if( hiResDurDigiIsrToWave1Max < hiResDur )
				hiResDurDigiIsrToWave1Max = hiResDur;
			if( hiResDurDigiIsrToWave1Min > hiResDur )
				hiResDurDigiIsrToWave1Min = hiResDur;
#endif	/*	TRACK_HI_RES_DUR	*/

			/* phase 1 of async. processing  */
			prec->pact = TRUE;

			card->pending = 0;

			/* post request to task
			 * request gets handled by our digi thread devWfVmeDigiTsk()
			 * which in turn calls this process function for the async completion,
			 * re-arming, timestamps, and event posting.
			 */
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
		/*
		 * Here's where we arm the digi for the next acquisition
		 * If this happens late we'll miss a capture!
		 */
#ifdef	TRACK_HI_RES_DUR
		t_HiResTime		hiResTickDigiReArm = GetHiResTicks();
		t_HiResTime		hiResDur = hiResTickDigiReArm - hiResTickDigiReArmPrior;
		if( hiResDurDigiReArmMax < hiResDur )
			hiResDurDigiReArmMax = hiResDur;
		if( hiResDurDigiReArmMin > hiResDur )
			hiResDurDigiReArmMin = hiResDur;
		hiResTickDigiReArmPrior	=  hiResTickDigiReArm;

		hiResDur = hiResTickDigiReArm - hiResTickDigiIsrPrior;
		hiResDurDigiIsrToReArmCum += hiResDur;
		if( hiResDurDigiIsrToReArmMax < hiResDur )
			hiResDurDigiIsrToReArmMax = hiResDur;
		if( hiResDurDigiIsrToReArmMin > hiResDur )
			hiResDurDigiIsrToReArmMin = hiResDur;

		// Count the number of Isr to ReArm intervals over 8.3ms
		nTicksHiResReArmLate = (HiResTicksPerSecond() / 120);
		if ( hiResDur > nTicksHiResReArmLate )
			nLateDigiReArm++;

#endif	/*	TRACK_HI_RES_DUR	*/
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
