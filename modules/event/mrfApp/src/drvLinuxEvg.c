/* This code is a dummy implementation of the mrfApp EVR/EVG driver code
 *
 * Copyright 2008, Stanford University
 * Author: Remi Machet <rmachet@slac.stanford.edu>
 *
 * Released under the GPLv2 licence <http://www.gnu.org/licenses/gpl-2.0.html>
 */

#include "drvMrfEg.h"
#include <epicsExport.h>        /* EPICS Symbol exporting macro definitions                       */
#include <registryFunction.h>   /* EPICS Registry support library                                 */
#include <epicsStdio.h>
#include <drvSup.h>

/*** Records and variables ***/

drvet drvMrf200Eg = { DRVETNUMBER };

epicsExportAddress (drvet, drvMrf200Eg);

/*** APIs ***/

long EgCheckCard(int Card)   /* called by dev. needs to be global */
{	return 0;
}
long EgStartRamTask(void)    /* "" */
{	return 0;
}
long EgScheduleRamProgram(int card)
{	return 0;
}
long EgGetRamEvent(EgCardStruct *pParm, long Ram, long Addr)
{	return 0;
}
long EgCheckTaxi(EgCardStruct *pParm)
{	return 0;
}
long EgDisableFifo(EgCardStruct *pParm)
{	return 0;
}
long EgEnableFifo(EgCardStruct *pParm)
{	return 0;
}
long EgCheckFifo(EgCardStruct *pParm)
{	return 0;
}
long EgMasterEnable(EgCardStruct *pParm)
{	return 0;
}
long EgMasterDisable(EgCardStruct *pParm)
{	return 0;
}
long EgRamClockSet(EgCardStruct *pParm, long Ram, long Clock)
{	return 0;
}
long EgRamClockGet(EgCardStruct *pParm, long Ram)
{	return 0;
}
long EgMasterEnableGet(EgCardStruct *pParm)
{	return 0;
}
long EgSetTrigger(EgCardStruct *pParm, unsigned int Channel, unsigned short Event)
{	return 0;
}
long EgGetTrigger(EgCardStruct *pParm, unsigned int Channel)
{	return 0;
}
long EgClearSeq(EgCardStruct *pParm, int channel)
{	return 0;
}
long EgResetAll(EgCardStruct *pParm)
{	return 0;
}
long EgEnableTrigger(EgCardStruct *pParm, unsigned int Channel, int state)
{	return 0;
}
int  EgSeqEnableCheck(EgCardStruct *pParm, unsigned int Seq)
{	return 0;
}
long EgEnableVme(EgCardStruct *pParm, int state)
{	return 0;
}
long EgGenerateVmeEvent(EgCardStruct *pParm, int Event)
{	return 0;
}
long EgSeqTrigger(EgCardStruct *pParm, unsigned int Seq)
{	return 0;
}
long EgSetSeqMode(EgCardStruct *pParm, unsigned int Seq, int Mode)
{	return 0;
}
int  EgReadSeqRam(EgCardStruct *pParm, int channel, unsigned char *pBuf)
{	return 0;
}
int  EgWriteSeqRam(EgCardStruct *pParm, int channel, unsigned char *pBuf)
{	return 0;
}
long EgGetAltStatus(EgCardStruct *pParm, int Ram)
{	return 0;
}
long EgEnableAltRam(EgCardStruct *pParm, int Ram)
{	return 0;
}
long EgSetSingleSeqMode(EgCardStruct *pParm, int Ram)
{	return 0;
}
long EgDisableRam(EgCardStruct *pParm, int Ram)
{	return 0;
}
long EgEnableRam(EgCardStruct *pParm, int Ram)
{	return 0;
}
long EgGetMode(EgCardStruct *pParm, int ram, int *pBusy, int *pEnable)
{	return 0;
}
long EgGetEnableTrigger(EgCardStruct *pParm, unsigned int Channel)
{	return 0;
}
long EgGetBusyStatus(EgCardStruct *pParm, int Ram)
{	return 0;
}
long EgGetEnableMuxDistBus(EgCardStruct *pParm, unsigned int Channel)
{	return 0;
}
long EgGetEnableMuxEvent(EgCardStruct *pParm, unsigned int Channel)
{	return 0;
}
long EgEnableMuxDistBus(EgCardStruct *pParm, unsigned int Channel, int state)
{	return 0;
}
long EgEnableMuxEvent(EgCardStruct *pParm, unsigned int Channel, int state)
{	return 0;
}
long EgGetACinputControl(EgCardStruct *pParm, unsigned int Channel)
{	return 0;
}
long EgSetACinputControl(EgCardStruct *pParm, unsigned int Channel, int state)
{	return 0;
}
long EgGetACinputDivisor(EgCardStruct *pParm, unsigned short DlySel)
{	return 0;
}
long EgSetACinputDivisor(EgCardStruct *pParm, unsigned short Divisor, unsigned short DlySel)
{	return 0;
}
long EgResetMuxCounter(EgCardStruct *pParm, unsigned int Channel)
{	return 0;
}
long EgResetMPX(EgCardStruct *pParm, unsigned int Mask)
{	return 0;
}
long EgGetEnableMuxSeq(EgCardStruct *pParm, unsigned int SeqNum)
{	return 0;
}
long EgEnableMuxSeq(EgCardStruct *pParm, unsigned int SeqNum, int state)
{	return 0;
}
unsigned long EgGetMuxPrescaler(EgCardStruct *pParm, unsigned short Channel)
{	return 0;
}
unsigned long EgSetMuxPrescaler(EgCardStruct *pParm, unsigned short Channel, unsigned long Divisor)
{	return 0;
}
long EgGetFpgaVersion(EgCardStruct *pParm)
{	return 0;
}
int EgSeqRamRead(EgCardStruct *pParm, int ram, unsigned short address, int len)
{	return 0;
}
int EgSeqRamWrite(EgCardStruct *pParm, int ram, unsigned short address, MrfEvgSeqStruct *pSeq)
{	return 0;
}
int EgDataBufferMode(EgCardStruct *pParm, int enable)
{	return 0;
}
int EgDataBufferEnable(EgCardStruct *pParm, int enable)
{	return 0;
}
int EgDataBufferSetSize(EgCardStruct *pParm, unsigned short size)
{	return 0;
}
void EgDataBufferLoad(EgCardStruct *pParm, epicsUInt32 *data, unsigned int nelm)
{	return;
}
void EgDataBufferUpdate(EgCardStruct *pParm, epicsUInt32 data, unsigned int dataIdx)
{	return;
}
void EgDataBufferSend(EgCardStruct *pParm)
{	return;
}
int EgDataBufferInit(int Card, int nelm)
{	return 0;
}
EgCardStruct *EgGetCardStruct (int Card)
{	return (void *)0;
}

/**************************************************************************************************/
/*                              EPICS iocsh extension                                             */
/*                                                                                                */

/* iocsh command: ErConfigure */

/* Registration APIs */
LOCAL void drvMrfEgRegister() {
	printf("WARNING: the linux version of this driver does not support EVG yet.\n");
}
epicsExportRegistrar(drvMrfEgRegister);
