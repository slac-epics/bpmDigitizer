/* This code is a dummy implementation of the mrfApp EVR/EVG driver code
 *
 * Copyright 2008, Stanford University
 * Author: Remi Machet <rmachet@slac.stanford.edu>
 *
 * Released under the GPLv2 licence <http://www.gnu.org/licenses/gpl-2.0.html>
 */

#include "mrfVme64x.h"
#include <epicsExport.h>        /* EPICS Symbol exporting macro definitions                       */
#include <registryFunction.h>   /* EPICS Registry support library                                 */
#include <iocsh.h>              /* EPICS iocsh support library                                    */
#include <epicsStdio.h>

/**************************************************************************************************/
/*                              EPICS iocsh extension                                             */
/*                                                                                                */

/* EPICS command: evrEEPROMFixup - Not supported in Linux, no need for it */
static const iocshFuncDef    evrEEPROMFixupDef     = {"evrEEPROMFixup", 0, NULL};

static void evrEEPROMFixupCall(const iocshArgBuf * args) {
	printf("ERROR: command %s not supported in Linux.\n", __func__);
}

/* Registration APIs */
static void drvMrfRegister() {
}
epicsExportRegistrar(drvMrfRegister);

static void evrEEPROMFixupRegister() {
	iocshRegister(&evrEEPROMFixupDef  , evrEEPROMFixupCall );
}
epicsExportRegistrar(evrEEPROMFixupRegister);
