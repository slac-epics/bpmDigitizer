/* $Id: devWfVmeDigiSim.c,v 1.3 2012/03/01 23:41:42 sonya Exp $ */

#include <stdlib.h>
#include <string.h>

#include <dbAccess.h>
#include <devSup.h>
#include <recSup.h>
#include <epicsExport.h>
#include <errlog.h>

#include <waveformRecord.h>

#include <devVmeDigiSupport.h>
#include <vmeDigiComm.h>
#include <devWfVmeDigiSup.h>

static void report_sim(int interest_level)
{
	errlogPrintf("Waveform Device support for drvPadUdpComm simulation mode\n"
                "on the SLAC 16-bit/130MSPS VME Digitizer\n");
}

static long init_record_sim(struct waveformRecord *prec)
{
int         idx;

	idx = devWfVmeDigiSupArgcheck(prec, "devWfVmeDigiSim", 0);

	if ( idx < 0 )
		goto bail;

	return 0;

bail:
	prec->pact = TRUE;
	return -1;
}

static long
read_waveform_sim(struct waveformRecord *prec)
{
int idx    = prec->inp.value.vmeio.card - 1;
int nbytes = prec->nord * sizeof(short);
	return vmeDigiCommSetSimMode(idx, prec->bptr, nbytes);
}

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read;
} devWfVmeDigiSim = {
	5,
	(DEVSUPFUN)report_sim,
	(DEVSUPFUN)0,
	(DEVSUPFUN)init_record_sim,
	(DEVSUPFUN)0,
	(DEVSUPFUN)read_waveform_sim
};

epicsExportAddress(dset, devWfVmeDigiSim);
