#include <epicsExport.h>
#include <epicsInterrupt.h>
#include <drvSup.h>
#include <devSup.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <errlog.h>
#include <epicsMutex.h>
#include <cantProceed.h>

#include <stdint.h>

#include <devBusMapped.h>
#include <mbboRecord.h>

#include <drvVmeDigiAfeQspi.h>

#include <padProto.h>
#include <padStream.h>

/*---copied from drvPadBpm.h---*/
#define AFE_REG_CSR     0
#define AFE_REG_CALA    1
#define AFE_REG_ATT1    2
#define AFE_REG_ATT2    3
#define AFE_REG_VER     5
#define AFE_REG_T2AMP   0x10    /* trig -> amp on       */

/* registers are listed in order of time-sequence not in numerical order */
#define AFE_REG_AMP2RF1 0x11    /* amp on -> RF pulse 1 */
#define AFE_REG_RFWIDTH 0x13    /* RF pulse width                             */
#define AFE_REG_OFFTIME 0x14    /* pulse OFF  -> change of control switches   */
#define AFE_REG_RF12RF2 0x12    /* delay to pulse 2                           */

#define AFE_REG_CSR_CALM_GET(x) ((x)&3)
#define AFE_REG_CSR_CALM_RED    0
#define AFE_REG_CSR_CALM_GRN    1

/*---end copied---*/

#define AFE_REG_CSR_ONLN        (1<<4) /* Always 1 when AFE on */
#define AFE_REG_CSR_CNFG        (1<<5) /* Set to 1 by FW when powered on; SW sets to 0 to indicate AFE is configured */
#define AFE_REG_CSR_CALT        (1<<6) /* Set to 1 by FW when cal trigger was seen; SW sets to 0 to acknowledge */

/* During AFE power-up, the FPGA bits are high for ~700 ms, returning 
 * a reply message of 0xffff. Also observed unpredictable bits set 
 * during power-down. To avoid responding to a transient bad read 
 * when checking AFE online and configured do the following:
 * When checking online, also check that (unused) MSB is not set.
 * When checking configured, verify that message header is
 * what is expected. (The latter is done in the routines and not
 * in AFE_NOT_CNFG macro because the message header is different
 * for read and write.)
 */
#define AFE_IS_ONLN(x)  (((x) & AFE_REG_CSR_ONLN) && !((x) & (1<<7)))  /* True if AFE online */
#define AFE_NOT_CNFG(x) (((x) & AFE_REG_CSR_CNFG)) /* True if AFE not configured */

/* Calibration modes */
#define AFE_CALM_RED 0
#define AFE_CALM_GRN 1
#define AFE_CALM_NONE 3
#define AFE_CALM_TOGGLE 4

#define AFE_MSG_READ      0xc0       /* Read message header */
#define AFE_MSG_WRITE     0x80       /* Write (and write/read) message header */
#define AFE_MSG_HEADER(x) ((x)<<8)   /* Shift message header to first byte */
#define AFE_MSG_DATA(x)   ((x)&0xFF) /* Extract data byte from message */
#define AFE_MIN_VER 67               /* Minimum fw version compatible with this sw */

#define AFE_BASE 1 /* devBusMapped error during record processing if base address = 0 */

#undef DEBUG

/* Diagnotic counters for troubleshooting */
volatile epicsUInt32 afeBeam=0;
volatile epicsUInt32 afeCalR=0;
volatile epicsUInt32 afeCalG=0;

typedef struct AfeDataRec_ {
	VmeDigi digi;        /* Digitizer structure */
	unsigned calMode;    /* Cal mode set by CALM record */
	uint8_t regCache[32];/* Cached AFE register contents */
} AfeDataRec, *AfeData;

/*
 * Internal routine to re-setup AFE registers after an AFE outage.
 * Only set CSR and attenuator registers. The other registers use
 * the AFE default and in practice, are never modified. (If we 
 * change this to recover other registers, must handle the case
 * in which the AFE is off when the IOC boots up. In this case, 
 * some register caches contain zeros and we do not want to write
 * zeros to these registers. Software must cache good values once
 * the AFE is one.)
 * 
 * Also check that fw version is correct and print message
 * if it is not.
 * 
 * IMPORTANT: assumes that mutex already locked and caller will
 *            unlock mutex after return 
 *
 * RETURN: 0 on success, -1 on failure (serial data transfer timeout)
 */
int 
drvVmeDigiAfeRecover(VmeDigi digi, AfeData afeData, const char *name)
{
/* Set these registers to cached values */
uint8_t reg[] = { AFE_REG_CALA, AFE_REG_ATT1, AFE_REG_ATT2};uint16_t data;
int i;

	/* Set up CSR, clear 'cal trigger seen' bit, set AFE to 'configured' */
	data = AFE_MSG_HEADER(AFE_MSG_WRITE + AFE_REG_CSR) + 
		(afeData->regCache[AFE_REG_CSR] | AFE_REG_CSR_CALT | AFE_REG_CSR_CNFG);

#ifdef DEBUG
	printf("drvVmeDigiAfeRecover; recovering from outage\n"); 
	printf("drvVmeDigiAfeRecover; writing 0x%04x to %s\n", data, name);
#endif

	if ( vmeDigiQspiWrite( digi, data ) )
		return -1;

	/* Set up remainting registers */
	for ( i = 0; i < sizeof(reg) ; i++ ) {
		data = AFE_MSG_HEADER(AFE_MSG_WRITE + reg[i]) + afeData->regCache[reg[i]];
#ifdef DEBUG
	printf("drvVmeDigiAfeRecover; writing 0x%04x to %s\n", data, name);
#endif
		if ( vmeDigiQspiWrite( digi, data ) )
			return -1;
	}

	/* Check firmware version */
	data = AFE_MSG_HEADER(AFE_MSG_READ + AFE_REG_VER);
     
	if ( vmeDigiQspiWriteRead(digi, data, &data) )
		return -1;
	else if ( (AFE_MSG_DATA(data)) < AFE_MIN_VER )
		errlogPrintf("WARNING: %s AFE Firmware too old or no AFE present -- has %i but I need at least %i\n", name, AFE_MSG_DATA(data), AFE_MIN_VER );

	return 0;
}

/* 
 * Callback to configure AFE. To be called at init. 
 * Call drvVmeDigiAfeQspiSetup to initialize AFE; if return value is not NULL,
 * store in usr_arg_p. Else, don't proceed.
 */
void
afe_cfg_done_cb(unsigned instance, VmeDigi digi, void **usr_arg_p)
{
char name[20];

	sprintf(name,"AFE%i",instance);
	
	if ( ! (*usr_arg_p = (DevBusMappedDev)drvVmeDigiAfeQspiInit(name, digi)) ) 
		cantProceed("FATAL ERROR: Failed to initialize AFE\n");	
}

/* Note: afe_acq_done_cb and afe_dma_done_cb share work of checking if last data 
 * acquisition was a cal cycle and if so, setting data kind.
 * 
 * Callback to toggle cal color and clear 'trigger seen' bit. To be called after 
 * digitizer acquisition complete. Digitizer acquires data for cal and beam events, 
 * so for equal rates of beam, green cal, red cal, pattern will be:
 *
 *    beam - grn cal - red cal - beam - red cal - grn cal - beam - grn cal ...
 *
 * Send message to AFE, setting new cal color and clearing cal 'trigger seen' bit. 
 * Store AFE CSR register in cache array. If DMA was not started, read response and unlock mutex.
 */
void
afe_acq_done_cb(unsigned instance, VmeDigi digi, void *usr_arg, int dmaPending)
{
uint16_t        data;
DevBusMappedDev afe     = usr_arg;
AfeData         afeData = afe->udata;
unsigned        calm    = 0;

	epicsMutexLock( afe->mutex );
 
	/* AFE message; clear 'cal trigger seen' bit */
	data = AFE_MSG_HEADER(AFE_MSG_WRITE + AFE_REG_CSR) + (afeData->regCache[AFE_REG_CSR] | AFE_REG_CSR_CALT);

	/* If in toggle mode, toggle cal color */
	if ( afeData->calMode ==  AFE_CALM_TOGGLE ) {

		switch( calm = AFE_REG_CSR_CALM_GET(afeData->regCache[AFE_REG_CSR]) ) {
			default:
				/* If cal color neither red nor green,
				   (which should not happen), set to green */
				calm = AFE_REG_CSR_CALM_GRN;
				break;

			case AFE_REG_CSR_CALM_GRN:
				calm = AFE_REG_CSR_CALM_RED;
				break;

			case AFE_REG_CSR_CALM_RED:
				calm = AFE_REG_CSR_CALM_GRN;
				break;
		}

		/* Modify message to set new cal color */
		data &= ~AFE_REG_CSR_CALM_GET(-1);
		data |= calm;

	}

	vmeDigiQspiWrite( digi, data );

        /* Store new register contents in cache */
	afeData->regCache[AFE_REG_CSR] = AFE_MSG_DATA(data);

	if ( !dmaPending ) {
		/* DMA could not be started (e.g., lack of buffers) must complete here */
		epicsMutexUnlock( afe->mutex );
		errlogPrintf("WARNING: DMA not started for AFE%u\n", instance);	    
	}    
}

/* 
 * Callback to test for cal cycle and set data kind. To be called after digitizer data 
 * copied to memory. Wait for AFE response. If AFE offline, return. If AFE online
 * but not 'configured' (must have been off), call drvVmeDigiAfeRecover to re-setup
 * AFE registers and then return. Otherwise, If cal trigger seen, set data kind.
 */
int
afe_dma_done_cb(unsigned instance, VmeDigi digi, void *usr_arg, PadReply rply)
{                         
uint16_t        data;
PadDataKind     kind;
DevBusMappedDev afe     = usr_arg;
AfeData         afeData = afe->udata;
int             rval    = -1; /* Returning rval!=0 causes data to be discarded */

	/* rply == NULL If DMA failed */
	if ( !rply )
		goto bail;

	/* If data transfer timeout, return */
	if ( vmeDigiQspiRead( digi, &data ) )
		goto bail;

	/* If AFE not present or off, return */
	if ( ! (AFE_IS_ONLN(data)) )
		goto bail; 

	/* Else if AFE just came online, configure it 
         * (Check that message header is what we expect: 0x8000)
         */		
	else if ( AFE_NOT_CNFG(data) && !((data)&(0x7f00)) && ((data)&(0x8000))) { 

printf("AFE not configured; last data read was 0x%04x\n",data);

		if ( drvVmeDigiAfeRecover(digi, afeData, afe->name) )
			errlogPrintf("WARNING: Failed to set up AFE %u after AFE outage\n",instance); 

	       goto bail;
	}

	/* If this was a calibration cycle then tag the data accordingly in rply */
	if ( (data & AFE_REG_CSR_CALT) ) {

		if ( afeData->calMode == AFE_CALM_TOGGLE ) {
			/* If we are in toggle mode:
			 * We already have switched the color (in afe_acq_done_cb); therefore, if the
			 * current color is 'green' then the data we have corresponds to a 'red' cycle
			 * and vice-versa.
			 */
			kind = AFE_REG_CSR_CALM_GRN == AFE_REG_CSR_CALM_GET( afeData->regCache[AFE_REG_CSR] ) ? PadDataCalRed : PadDataCalGrn;

		}
		else if ( afeData->calMode == AFE_CALM_GRN )
			kind = PadDataCalGrn;

		else if ( afeData->calMode == AFE_CALM_RED )
			kind = PadDataCalRed;

		else /* Cal mode = NONE */
			goto bail;

		rply->strm_cmd_flags &= ~PADRPLY_STRM_FLAG_TYPE_SET(-1);
		rply->strm_cmd_flags |= PADRPLY_STRM_FLAG_TYPE_SET(kind);

		if ( PadDataCalRed == kind ) {
			afeCalR++;
		} else {
			afeCalG++;
		}

	} else {
		afeBeam++;
	}

	rval = 0;
bail:
	epicsMutexUnlock( afe->mutex );
	return rval;
}

static VmeDigiCommCbRec afeCb = {
	cfg_done:    afe_cfg_done_cb,
	acq_done:    afe_acq_done_cb,
	dma_done:    afe_dma_done_cb,
};

/* Public pointer to our callbacks */
VmeDigiCommCb drvVmeDigiAfeCallbacks = &afeCb;

/* Initialize AFE. To be called at init. Register AFE with devBusMapped. 
 * Allocate memory for structure containing afe-specific data; attach this
 * structure to DevBusMappedDev->udata and attach DevBusMappedDev struct to usr_arg_p.
 * This provides access to this data from other callbacks and devBusMapped device support. 
 * Initialize AFE: set cal color, clear cal 'trigger seen' bit, set AFE to 'configured'. 
 * Store AFE CSR register settings in cache array. Check that AFE 'online' bit is set. 
 * If it is not, exit. Check that FW version is correct.
 */ 
DevBusMappedDev
drvVmeDigiAfeQspiInit(const char *name, VmeDigi digi)
{
uint16_t        data;
DevBusMappedDev afe;
AfeData         afeData = 0;

	if ( ! (afe = devBusMappedRegister(name, (volatile void *)(AFE_BASE))) ) {
		errlogPrintf("FATAL ERROR: Unable to register AFE %s with bus mapped devsup\n", name);
		return NULL;
	}

	if ( ! (afeData = calloc(1,sizeof(*afeData))) ) {
		errlogPrintf("FATAL ERROR: No memory for AfeData structure\n");
		return NULL;
	}

        epicsMutexLock( afe->mutex );

	afeData->digi = digi;

	/* Initialize AFE: set cal mode to green, cal oscillator mode to auto, clear 'cal trigger seen' bit, and set AFE to 'configured' */
	data = AFE_MSG_HEADER(AFE_MSG_WRITE + AFE_REG_CSR) + (AFE_REG_CSR_CALM_GRN | AFE_REG_CSR_CALT | AFE_REG_CSR_CNFG);

#ifdef DEBUG
	printf("drvVmeDigiAfeQspiInit; writing %s CSR 0x%04x\n", name, data);
#endif

	/* Cache new register contents */
	afeData->regCache[AFE_REG_CSR] = AFE_MSG_DATA(data) & ~AFE_REG_CSR_CALT & ~AFE_REG_CSR_CNFG;

	afe->udata = afeData;

	if ( vmeDigiQspiWriteRead( digi, data, &data ) )
		errlogPrintf("WARNING: %s digitizer serial interface data transfer TIMEOUT\n", name);	        


	if ( ! (AFE_IS_ONLN(data)) ) {
printf("init: last data is 0x%04x\n",data);
		errlogPrintf("WARNING: %s AFE off or not present\n", name);
		goto bail;
	}

	/* Check firmware version */
	data = AFE_MSG_HEADER(AFE_MSG_READ + AFE_REG_VER);
     
#ifdef DEBUG
	printf("drvVmeDigiAfeQspiInit; writing to %s VER 0x%04x\n", name, data);
#endif

	if ( vmeDigiQspiWriteRead(digi, data, &data) )
		errlogPrintf("WARNING: Failed to read %s FW version\n", name); 	

	else if ( (AFE_MSG_DATA(data)) < AFE_MIN_VER )
		errlogPrintf("WARNING: %s AFE Firmware too old or no AFE present -- has %i but I need at least %i\n", name, AFE_MSG_DATA(data), AFE_MIN_VER );
bail:

#ifdef DEBUG
	printf("afe_cfg_done_cb; %s CSR is now 0x%02x\n", name, afeData->regCache[AFE_REG_CSR]);
#endif

	epicsMutexUnlock( afe->mutex );

	return afe;
}

/* Generic device support method for AFE input record.
 * Send 'read' message to AFE register specified in .INP field and read response. 
 * If there is a data transfer timeout, return error.
 * Else, cache register contents and set .VAL.
 *
 * RETURNS: 0 on success
 *         -1 on failure
 */
static int
qspiReadAfeReg(DevBusMappedPvt pvt, unsigned *pvalue, dbCommon *prec)
{
uint16_t        data;                                        /* AFE message */
uint8_t         a = ((uint32_t)pvt->addr & 0x3f) - AFE_BASE; /* Get AFE register */
int             rval;
DevBusMappedDev afe = pvt->dev;
AfeData         afeData = afe->udata;

	/* Read AFE register a */
	data = AFE_MSG_HEADER(AFE_MSG_READ + a);

	epicsMutexLock( afe->mutex );

#ifdef DEBUG
	printf("qspiReadAfeReg; writing to %s, AFE register 0x%02x 0x%04x\n", afe->name, a, data);
#endif

	if ( (rval = vmeDigiQspiWriteRead(afeData->digi, data, &data )) )
		goto bail;

#ifdef DEBUG
	printf("qspiReadAfeReg; %s, got 0x%04x\n", afe->name, data);
#endif

	if ( a < sizeof(afeData->regCache)/sizeof(afeData->regCache[0]) ) 
		afeData->regCache[a] = AFE_MSG_DATA(data);

	/* Extract data (second byte) */
	*pvalue = AFE_MSG_DATA(data);

bail:

	epicsMutexUnlock( afe->mutex );

	return rval;
}

/* Generic device support method for AFE output record
 * Write .VAL to register specified in .OUT field. If there is a data 
 * transfer timeout, return error. Else, cache register contents.
 *
 * RETURNS: 0 on success
 *         -1 on failure
 */
static int
qspiWriteAfeReg(DevBusMappedPvt pvt, unsigned value, dbCommon *prec)
{
uint16_t        data;                                        /* AFE message */
uint8_t         a = ((uint32_t)pvt->addr & 0x3f) - AFE_BASE; /* Get AFE register to write to */
int             rval;
DevBusMappedDev afe = pvt->dev;
AfeData         afeData = afe->udata;

	/* Write value to AFE register a */
	data = AFE_MSG_HEADER(AFE_MSG_WRITE + a) + value;
        
#ifdef DEBUG
	printf("qspiWriteAfeReg; writing to %s, AFE register 0x%02x 0x%04x\n", afe->name, a, data);
#endif

	epicsMutexLock( afe->mutex );

	if ( (rval = vmeDigiQspiWriteRead(afeData->digi, data, NULL)) )
		goto bail;

	if ( a < sizeof(afeData->regCache)/sizeof(afeData->regCache[0]) ) 
		afeData->regCache[a] = AFE_MSG_DATA(data);

#ifdef DEBUG
	data |= 0x4000;
	data &= ~0xFF;
	vmeDigiQspiWriteRead(afeData->digi, data, &data);
	printf("qspiWriteAfeReg; %s register contents are now 0x%04x\n",afe->name, data);
#endif


bail:
	epicsMutexUnlock( afe->mutex );

        return rval;
}

/* Prototypes for our read/write routines */
static DevBusMappedAccessRec qspiDigiAccess = {
	qspiReadAfeReg,
	qspiWriteAfeReg
};

/* Set calibration mode. Modes 0-3 are settable in CSR register.
 * For these, call qspiWriteAfeReg to write register value. 
 *
 * NOTE: this will write zeros to all other CSR bits. If we were 
 * using a cal oscillator mode != AUTO, that would overwrite that mode
 * and this routine should be changed. At this time, we only use AUTO. 
 * Also, cal mode is typically only set on boot.
 */
static int
qspiWriteAfeCalm(DevBusMappedPvt pvt, unsigned value, dbCommon *prec)
{
DevBusMappedDev afe = pvt->dev;
AfeData afeData = afe->udata;
int rval = 0;

#ifdef DEBUG
	printf("qspiWriteVmeDigiCalm; %s calm val is %i; value is %i\n", afe->name, ((mbboRecord *)prec)->val, value);
#endif

	/* extract 'soft toggle' information via an ugly hack
	 * (we shouldn't have to know what record type 'prec' is
	 * at the driver level!)
	 */
	afeData->calMode = (((mbboRecord *)prec)->val);

	if ( afeData->calMode < 4 )
		rval = qspiWriteAfeReg(pvt, value, prec);

	return rval;
}

/* Prototypes for our read/write routines */
static DevBusMappedAccessRec qspiCalmAccess = {
	qspiReadAfeReg,
	qspiWriteAfeCalm
};

/* VME digitizer AFE device support init */
static long
vmeDigiAfeQspiInit(void)
{
long rval;

	if ( (rval = devBusMappedRegisterIO("qspiDigiAfe",&qspiDigiAccess)) )
		return rval;

	if ( (rval = devBusMappedRegisterIO("softTogAfe",&qspiCalmAccess)) )
		return rval; 

	return 0;
}

static long 
vmeDigiAfeQspiReport(int level)
{
	errlogPrintf("VME digitizer QSPI Support for Communication with AFE\n");
	return 0;
}

static struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
} drvVmeDigiAfeQspi = {
	2,
	vmeDigiAfeQspiReport,
	vmeDigiAfeQspiInit
};

epicsExportAddress(drvet,drvVmeDigiAfeQspi);


