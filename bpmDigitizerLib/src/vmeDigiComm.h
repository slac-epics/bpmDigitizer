#ifndef VME_DIGI_COMM_H
#define VME_DIGI_COMM_H
/*$Id: vmeDigiComm.h,v 1.4 2012/03/05 11:20:37 strauman Exp $*/

#ifdef __cplusplus
extern "C" {
#endif

#include <vme64x.h>
#include <stdint.h>
#include <vmeDigi.h>
#include <drvPadUdpCommIO.h>
#include <padProto.h>
#include <padStream.h>

/* NOTE: any of the callback members may be NULL if unneeded. */
typedef struct VmeDigiCommCbRec_ {
	/* callback executed at the end of vmeCommDigiConfig; the routine
	 * may user '*usr_arg_p' (the value was passed to 'vmeCommDigiConfig()')
	 * but it may also choose to set this value. If *usr_arg_p is modified
	 * by this callback then 'acq_done' and 'dma_done' will get the modified
	 * value.
	 */
	void     (*cfg_done)(unsigned channel, VmeDigi digi, void **usr_arg_p);
	/* callback executed after digi acquisition is done; digitizer
	 * is disarmed at this point with samples in memory.
	 * If 'dmaPending' is non-zero then a DMA was requested successfully
	 * and 'dma_done' will eventually be called for the acquired data.
	 * If 'dmaPending' is zero then DMA could not be requested (not enough
	 * buffers) and the digitizer was rearmed (effectively throwing
	 * away the current acquisition).
	 */
	void     (*acq_done)(unsigned channel, VmeDigi digi, void *usr_arg, int dmaPending);
	/* callback executed after DMA is done; data are in 'padReply' packet
	 * passed to this callback (and may be modified by this callback).
	 * NOTE:    if DMA failed then a NULL 'rply' is passed. The callback must be
	 *          prepared for this situation.
	 * RETURNS: nonzero status if packet should be dropped. The return value is
	 *          ignored if 'rply == NULL' (since there is no packet to be dropped).
	 */
	int      (*dma_done)(unsigned channel, VmeDigi digi, void *usr_arg, PadReply rply);
} VmeDigiCommCbRec, *VmeDigiCommCb;

/* Configure digitizer at CSR base address 'csrbase' and attach to 'vmeComm' channel 'channel'.
 *
 *   'channel': channel number used to identify this digitizer (raw data transmitted via udpComm
 *              protocol are tagged with this channel number).
 *              
 *   'csrbase': base address in CSR space of this digitizer.
 *   'a32base': address in A32 where this digitizer should be mapped.
 *   'irq_vec': interrupt vector to use (NOTE: due to firmware restriction of some versions
 *              this must be identical with the VME slot number)
 *   'irq_lvl': VME interrupt level to be used for this digitizer.
 *   'callbks': callback table. May be NULL if no callbacks are required.
 *   'usr_arg': pointer which is passed back to the callbacks.
 *
 *    RETURNS : zero on success, nonzero on error.
 */

int
vmeCommDigiConfig(unsigned channel, VME64_Addr csrbase, VME64_Addr a32base, uint8_t irq_vec, uint8_t irq_lvl, VmeDigiCommCb callbks, void *usr_arg);

/* Obtain handle for the digitizer associated with 'channel'.
 * 
 * RETURNS: handle or NULL if no digitizer is configured for
 *          the given channel.
 */
VmeDigi
vmeCommDigiGet(unsigned channel);

/* Write sample data into digitizer memory for simulation
 * purposes. Multiple data sets may be written which
 * are returned by the digitizer consecutively after 
 * multiple triggers.
 * The 'nbytes' argument must be a multiple of
 *  2 (sizeof samples) * 4 (channels) * (nsamples).
 *
 * To leave simulation mode, call with 'nbytes' == 0.
 *
 * NOTES: No endian conversion is done on the data. They
 *        MUST be big-endian (matching the digitizer endian-ness).
 *
 *        The two lsb cannot be written; this means that 
 *        an overflow/-range situation cannot be simulated!
 */
int
vmeDigiCommSetSimMode(int channel, void *data, int nbytes);

/* IO operations table (emulated udpComm operations) */
extern DrvPadUdpCommIO drvPadVmeCommIO;

#ifdef __cplusplus
}
#endif

#endif
