/* $Id: devWfVmeDigiSup.h,v 1.1 2010/08/30 17:22:02 strauman Exp $ */
#ifndef DEV_WV_VME_DIGI_SUP_H
#define DEV_WV_VME_DIGI_SUP_H

#include <waveformRecord.h>

#define NCHAN     4

int
devWfVmeDigiSupArgcheck(struct waveformRecord *prec, const char *prefix, int needs_config);

#endif
