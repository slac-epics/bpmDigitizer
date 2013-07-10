#ifndef DRV_VME_DIGI_AFE_H
#define DRV_VME_DIGI_AFE_H

#include <vmeDigi.h>
#include <vmeDigiComm.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Callbacks for use by vmeDigiComm.c
 * Defined in drvVmeDigiAfeQspi.c
 */
extern VmeDigiCommCb drvVmeDigiAfeCallbacks;

/* 
 * Routine to configure AFE. To be called at init. Register AFE with devBusMapped. 
 * Allocate memory for structure containing afe-specific data and attach this
 * structure to DevBusMappedDev->udata. Thus the data is available to devBusMapped
 * device support. Check existence of AFE by reading FW version. Check correct FW version.
 * Intialize cal color, clear cal 'trigger seen' bit. Store AFE CSR register in cache array.
 */
DevBusMappedDev
drvVmeDigiAfeQspiInit(const char *name, VmeDigi digi);

#endif
