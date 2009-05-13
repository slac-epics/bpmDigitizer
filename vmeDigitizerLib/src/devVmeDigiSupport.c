/* $Id: devVmeDigiSupport.c,v 1.2 2008/11/25 01:39:43 strauman Exp $ */

#include <stdlib.h>
#include <string.h>

#include <epicsExport.h>
#include <devLib.h>
#include <callback.h>
#include <iocsh.h>

#include <devBusMapped.h>

#include <vmeDigi.h>

#include <devVmeDigiSupport.h>

VmeDigiCard devVmeDigis[MAX_DIGIS] = { {0} };

static volatile uint16_t ista_bits[MAX_DIGIS];

static int readSlot(DevBusMappedPvt p, unsigned *pval, dbCommon *prec)
{
uint32_t slot = (uint32_t)p->dev->baseAddr;

	slot = (slot>>19) & 31;

	*pval = slot;
	return 0;
}

static int readSN(DevBusMappedPvt p, unsigned *pval, dbCommon *prec)
{
uint32_t sn = vme64ReadSN((VME64_Addr)p->dev->baseAddr);

	if ( (uint32_t)-1 == sn )
		return -1;

	*pval = sn;
	return 0;
}

static int readCR32(DevBusMappedPvt p, unsigned *pval, dbCommon *prec)
{
	*pval = vme64CSRegRead32((VME64_Addr)p->addr,0);
	return 0;
}

static DevBusMappedAccessRec vmeDigiRdSlot = {
	readSlot,
	0
};

static DevBusMappedAccessRec vmeDigiRdSN = {
	readSN,
	0
};

static DevBusMappedAccessRec vmeDigiRdCR32 = {
	readCR32,
	0
};

static void
devVmeDigiIsr(void *arg)
{
int idx = (int)arg;
unsigned p;

#if 0
	/* broken design; once set we cannot mask interrupts :-( */
	vmeDigiIrqDisable(devVmeDigis[idx].digi);
#else
	vmeDigiIrqDisable(devVmeDigis[idx].digi);
	devVmeDigis[idx].pending    = p = vmeDigiIrqAck(devVmeDigis[idx].digi);
	ista_bits[idx]             |= p | vmeDigiIrqAckUnused(devVmeDigis[idx].digi);
#endif

	if ( devVmeDigis[idx].prec ) {
		callbackRequestProcessCallback(
			& devVmeDigis[idx].cb,
			priorityHigh, 
			devVmeDigis[idx].prec);
	}
}

static int 
nextSlot(int slot)
{
	while ( ++slot <= 21 ) {
		if ( (unsigned long)-1 != vmeDigiDetect( slot << 19 , 1 ) )
			return slot;
	}
	return -1;
}

static int
inst2slot(int instance)
{
int slot = 0;
	while ( instance-- > 0 ) {
		if ( (slot=nextSlot(slot)) < 0 )
			break;
	}
	return slot;
}

static int
doConfig(int instance, int slot, unsigned a32_base, int vec, int lvl)
{
int           idx  = instance - 1;
VmeDigi       digi = 0;
unsigned long csr_local, size;
volatile void *local_a32_addr;
char          name[20];

	if ( instance > MAX_DIGIS ) {
		epicsPrintf("devVmeDigiConfig: instance # %i too big\n", instance);
		return 0;
	}

	if ( devVmeDigis[idx].digi ) {
		epicsPrintf("devVmeDigiConfig: instance # %i already configured\n", instance);
		return 0;
	}

	if ( devInterruptInUseVME(vec) ) {
		epicsPrintf("devVmeDigiConfig: interrupt vector 0x%02x in use\n", vec);
		return 0;
	}


	csr_local = vmeDigiDetect( slot << 19, 0 );


	if ( (unsigned long)-1 == csr_local )
		/* no Digi found in this slot */
		return 0;

	size = vme64ReadADEM( csr_local, 0 );

	/* convert mask into size */
	size = ( ~ (size & ~0xff) ) + 1;

	if ( 0 == size ) {
		epicsPrintf("devVmeDigiConfig: internal error; a32 size == 0 ?\n");
		return 0;
	}

	if ( devRegisterAddress("devVmeDigi", atVMEA32, a32_base, size, &local_a32_addr) ) {
		epicsPrintf("devVmeDigiConfig: A32 address 0x%08x already in use\n", a32_base);
		return 0;
	}

	digi = vmeDigiSetup( slot << 19, a32_base, vec, lvl );

	if ( !digi ) {
		devUnregisterAddress( atVMEA32, a32_base, "devVmeDigi" );
		return 0;
	}

	if ( devConnectInterruptVME( vec, devVmeDigiIsr, (void*)idx) ) {
		devUnregisterAddress( atVMEA32, a32_base, "devVmeDigi" );
		epicsPrintf("devVmeDigiConfig: connecting ISR failed!\n");
		return 0;
	}

	devVmeDigis[idx].digi           = digi;
	devVmeDigis[idx].local_a32_addr = local_a32_addr;
	devVmeDigis[idx].size           = size;

	sprintf(name,"vmeDigi%i",instance);
	devBusMappedRegister(name, (volatile void*)csr_local);
	sprintf(name,"vmeDigi%i-ista",instance);
	devBusMappedRegister(name, (volatile void*)(ista_bits+idx));
	sprintf(name,"rdSlot");
	devBusMappedRegisterIO(name, &vmeDigiRdSlot);
	sprintf(name,"rdSN");
	devBusMappedRegisterIO(name, &vmeDigiRdSN);
	sprintf(name,"rdCR32");
	devBusMappedRegisterIO(name, &vmeDigiRdCR32);

	return size;
}

/* Returns number of configured cards
 *
 * Can call with 
 *    instance < 1           -> all cards are autodetected; 'devVmeDigis' array
 *                              is filled with cards as they are found.
 *
 *    instance >=1, slot <=0 -> slots are scanned for 'instance' of cards
 *
 *    instance >=1, slot >0  -> configure a card in known/fixed slot and
 *                              associate with array entry 'instance-1'.
 */

int
devVmeDigiConfig(int instance, int slot, unsigned a32_base, int vec, int lvl)
{
int size = 0;
	if ( instance < 1 ) {
		/* autodetect all */
		for ( instance = 0, slot = 0; (slot=nextSlot(slot)) > 0;  ) {
			size = doConfig(instance + 1, slot, a32_base, vec, lvl);
			if ( !size )
				break;
			instance++;
			/* compute A32 base and vector of next unit */
			a32_base += size;
			vec++;
		}
	} else {
		if ( slot <= 0 )
			slot = inst2slot(instance);
		if ( slot <= 0 ) {
			epicsPrintf("devVmeDigiConfig: card # %i not found\n", instance);
			return 0;
		}
		instance =  doConfig(instance, slot, a32_base, vec, lvl) ? 1 : 0;
	}
	if ( instance )
		devEnableInterruptLevelVME(lvl);
	return instance;
}

static const struct iocshArg args[] = {
	{	"Card instance -1 | (1..n) ", iocshArgInt },
	{   "VME slot      -1 | (0..20)", iocshArgInt },
	{   "A32 base                  ", iocshArgInt },
	{   "IRQ vector"                , iocshArgInt },
	{   "IRQ level"                 , iocshArgInt },
};

static const struct iocshArg *bloat[] = {
	&args[0],
	&args[1],
	&args[2],
	&args[3],
	&args[4],
};

struct iocshFuncDef devVmeDigiConfigDesc = {
	"devVmeDigiConfig",
	sizeof(bloat)/sizeof(bloat[0]),
	bloat
};

static void
devVmeDigiConfigWrap(const iocshArgBuf *a)
{
	devVmeDigiConfig(a[0].ival, a[1].ival, a[2].ival, a[3].ival, a[4].ival);
}

static void
devVmeDigiRegistrar(void)
{
	iocshRegister( &devVmeDigiConfigDesc, devVmeDigiConfigWrap );
}

epicsExportRegistrar( devVmeDigiRegistrar );
