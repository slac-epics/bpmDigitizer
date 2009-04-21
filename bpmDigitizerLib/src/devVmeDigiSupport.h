/* $Id: devVmeDigiSupport.h,v 1.2 2008/11/25 01:39:43 strauman Exp $ */
#ifndef DEV_VME_DIGI_SUPPORT_H
#define DEV_VME_DIGI_SUPPORT_H

#include <callback.h>

#include <vmeDigi.h>

#define MAX_DIGIS 8

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VmeDigiCard_ {
	VmeDigi             digi;
	volatile void      *local_a32_addr;
	int                 size;
	dbCommon           *prec;
	volatile unsigned   pending; /* pending interrupts */
	CALLBACK            cb;
} VmeDigiCard;

extern VmeDigiCard devVmeDigis[MAX_DIGIS];

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
devVmeDigiConfig(int instance, int slot, unsigned a32_base, int vec, int lvl);

#ifdef __cplusplus
};
#endif

#endif
