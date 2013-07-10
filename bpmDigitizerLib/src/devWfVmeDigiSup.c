/* $Id: devWfVmeDigiSup.c,v 1.2 2012/03/02 00:58:28 sonya Exp $ */

#include <stdlib.h>
#include <string.h>

#include <dbAccess.h>
#include <errlog.h>
/*
#include <devSup.h>
#include <recSup.h>
#include <dbEvent.h>
*/

#include <waveformRecord.h>

#include <devVmeDigiSupport.h>

#include <devWfVmeDigiSup.h>

int
devWfVmeDigiSupArgcheck(struct waveformRecord *prec, const char *prefix, int needs_config)
{
int          idx;

	if ( VME_IO != prec->inp.type ) {
		errlogPrintf("%s.init_record(): expect VME_IO link\n",prefix);
		return -1;
	}

	idx = prec->inp.value.vmeio.card;

	if ( idx < 1 || idx > MAX_DIGIS ) {
		errlogPrintf("%s.init_record(): card # %i out of range\n", prefix, idx);
		return -1;
	}

	if ( needs_config && ! devVmeDigis[idx-1].digi ) {
		errlogPrintf("%s.init_record(): card # %i not configured\n", prefix, idx);
		return -1;
	}

	if ( DBR_SHORT != prec->ftvl ) {
		errlogPrintf("%s.init_record(): FTVL must be SHORT\n", prefix);
		return -1;
	}

	if ( prec->nelm % NCHAN != 0 || 0 == prec->nelm ) {
		errlogPrintf("%s.init_record(): NELM must be a (nonzero) multible of %i\n", prefix, NCHAN);
		return -1;
	}

	return idx;
}
