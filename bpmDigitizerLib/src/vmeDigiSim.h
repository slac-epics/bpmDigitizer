#ifndef VME_DIGI_SIM_H
#define VME_DIGI_SIM_H
/*$Id: vmeDigiSim.h,v 1.1 2010/02/17 18:05:13 strauman Exp $*/

#ifdef __cplusplus
extern "C" {
#endif

/* Write sample data into digitizer memory for simulation
 * purposes. Multiple data sets may be written which
 * are returned by the digitizer consecutively after 
 * multiple triggers.
 * The 'nbytes' argument must be a multiple of
 *  2 (sizeof samples) * 4 (channels) * (nsamples).
 *
 * To leave simulation mode, call with 'nbytes' == 0.
 *
 * NOTE: No endian conversion is done on the data. They
 *       MUST be big-endian (matching the digitizer endian-ness).
 */
int
vmeDigiCommSetSimMode(int channel, void *data, int nbytes);

#ifdef __cplusplus
}
#endif

#endif
