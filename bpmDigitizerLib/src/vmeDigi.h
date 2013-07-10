#ifndef VMEDIGI_H
#define VMEDIGI_H

#include <vme64x.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Handle for a digitizer; all driver functions operate on this */
typedef struct VmeDigiRec_ *VmeDigi;

/* Detect a digitizer at 'csrbase' (a VME CSR address);
 * returns CSR address as seen by CPU on success or
 * (-1) on failure.
 * If 'quiet' is nonzero then error messages are suppressed.
 */
unsigned long
vmeDigiDetect(VME64_Addr csrbase, int quiet);

/* Setup digitizer; csrbase and a32base are VME addresses in
 * CSR and A32, respectively.
 *
 * RETURNS: handle on success; NULL on error -- a diagnostic
 *          message is printed to stderr.
 */
VmeDigi
vmeDigiSetup(VME64_Addr csrbase, VME64_Addr a32base, uint8_t irq_vec, uint8_t irq_lvl);

/* Basic operations */

#define VMEDIGI_CNT_MAX      16384

/* Set acquisition counter.
 *
 * If 'cnt' argument is zero then the counter is unchanged.
 * (Can be used to read current count.)
 *
 * RETURNS: current count.
 */
uint32_t
vmeDigiSetCount(VmeDigi digi, uint32_t cnt);

/*
 * Arm digitizer
 *
 * If the 'armed' bit was already set then
 * it is reset and set again.
 *
 * RETURNS: nonzero if 'armed' bit was set originally,
 *          zero otherwise.
 */
uint16_t
vmeDigiArm(VmeDigi digi);

/*
 * Disarm digitizer
 *
 * RETURNS: nonzero if 'armed' bit was set originally,
 *          zero otherwise.
 */
uint16_t
vmeDigiDisarm(VmeDigi digi);

/*
 * Raise SW trigger; if an external trigger was enabled then
 * the trigger source is temporarily switched to SW and back
 * to its original setting after the SW trigger has been
 * strobed.
 */
void
vmeDigiSWTrig(VmeDigi digi);

/*
 * Raise SW interrupt;
 *
 * NOTE:
 * Should NOT use this routine if AUX_CSR is controlled via devBusMapped!
 */
void
vmeDigiSWIntr(VmeDigi digi);

/*
 * Enable digitizer interrupt
 */
void
vmeDigiIrqEnable(VmeDigi digi);

/*
 * Disable digitizer interrupt
 *
 * RETURNS: 0 if interrupt was already disabled,
 *          nonzero if it was enabled.
 * 
 * NOTE:    The return value can be used for nested
 *          calls to vmeDigiIrqEnable/vmeDigiIrqDisable:
 *
 *          was_enabled = vmeDigiIrqDisable(digi);
 *            ...critical section...
 *          if (was_enabled)
 *          	vmeDigiIrqEnable(digi);
 */
int
vmeDigiIrqDisable(VmeDigi digi);

/*
 * Clear pending interrupts.
 *
 * RETURNS: bitset of all pending interrupts that
 *          were cleared and which are not masked.
 */
unsigned
vmeDigiIrqAck(VmeDigi digi);

/* reset IRQ status bits that are NOT used by ISR(s) */
unsigned
vmeDigiIrqAckUnused(VmeDigi digi);

/*
 * Dump info (manufacturer ID, board ID, revision and S/N)
 * into a string buffer which will be NULL terminated.
 */
void
vmeDigiInfo(VmeDigi digi, char *buf, int len);

/***********************************************************
 * The following routines are used for control of a remote
 * device via the digitizer's serial interface. 16-bit 
 * data tranfers are used. Note that this implementation 
 * does NOT perform locking, but relies on locking being 
 * done at a higher level. 
 * These call an internal 'wait' routine to wait for data 
 * transfers to complete by monitoring SIO_ACTIV bit in
 * auxiliary control register. Interrupts are NOT disabled
 * (do not want to while we busy-wait). Instead we restrict
 * our operations to one register byte which is used only 
 * by the remote serial device driver. (see vmeDigi.c for
 * more info.) 
 ***********************************************************/							   
				   
/* Call 'wait' routine to wait for any previous transfer to
 * complete. Write message (data_out) to serial data register 
 * and set SIO_ACTIV bit in auxiliary control register to 
 * initiate data transfer to remote device.  
 */
int
vmeDigiQspiWrite(void *device, uint16_t data_out);

/* Call 'wait' routine to wait for data transfer to complete.
 * Read response from remote device and store in data_in.
 *
 * RETURNS: 0 on success
 *          nonzero (-1) if transfer timed out
 */
int
vmeDigiQspiRead(void *device, uint16_t *data_in);

/* Write message to remote device and optionally read response.
 * Call vmeDigiQspiWrite to send message (data_out).
 * If data_in is not NULL, call vmeDigiQspiRead to read response 
 * and store in data_in.
 *
 * RETURNS: 0 on success
 *          nonzero (-1) if transfer timed out
 */
int 
vmeDigiQspiWriteRead(void *device, uint16_t data_out, uint16_t *data_in);


#ifdef __cplusplus
};
#endif

#endif
