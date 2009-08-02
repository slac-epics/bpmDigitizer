#include <rtems.h>
#include <bsp/VMEDMA.h>
#include <bsp/VME.h>
#include <bsp/vme_am_defs.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include <vmeDigi.h>

#ifdef __STRICT_ANSI__
#include <stdarg.h>
extern int snprintf(char *, size_t, const char *, ...);
#endif

#define VMEDIGI_OFF_IRQ_STS	 0x7fa32
#define VMEDIGI_IRQ_STS_SW_IRQ        (1<<7)
#define VMEDIGI_IRQ_STS_BERR_IRQ      (1<<6)
#define VMEDIGI_IRQ_STS_BCLK_IRQ      (1<<5)
#define VMEDIGI_IRQ_STS_CLKA_IRQ      (1<<4)
#define VMEDIGI_IRQ_STS_OVFL_IRQ      (1<<3)
#define VMEDIGI_IRQ_STS_STRT_IRQ      (1<<2)
#define VMEDIGI_IRQ_STS_STOP_IRQ      (1<<1)
#define VMEDIGI_IRQ_STS_DSPD_IRQ      (1<<0)

#define VMEDIGI_IRQ_STS_2_CSR(x)	  (((x)&0xff)<<8)

#define VMEDIGI_OFF_ACQ_STS	 0x7fa3a

#define VMEDIGI_OFF_DCM0_CSR 0x7fb02
#define VMEDIGI_OFF_DCM1_CSR 0x7fb06
#define VMEDIGI_OFF_DCM2_CSR 0x7fb0a
#define VMEDIGI_OFF_DCM3_CSR 0x7fb0e
#define VMEDIGI_OFF_DCM4_CSR 0x7fb12
#define VMEDIGI_DCM_CSR_BYPASS (1<<9)

#define VMEDIGI_OFF_IRQ_CSR	 0x7fb16
#define VMEDIGI_IRQ_CSR_SW_IRQ        (1<<15)
#define VMEDIGI_IRQ_CSR_BERR_IRQ      (1<<14)
#define VMEDIGI_IRQ_CSR_BCLK_IRQ      (1<<13)
#define VMEDIGI_IRQ_CSR_CLKA_IRQ      (1<<12)
#define VMEDIGI_IRQ_CSR_OVFL_IRQ      (1<<11)
#define VMEDIGI_IRQ_CSR_STRT_IRQ      (1<<10)
#define VMEDIGI_IRQ_CSR_STOP_IRQ      (1<< 9)
#define VMEDIGI_IRQ_CSR_DSPD_IRQ      (1<< 8)
#define VMEDIGI_IRQ_CSR_IRQ_ENBL      (1<< 7)
#define VMEDIGI_IRQ_CSR_IRQ_LVL(l)    (((l)&7)<<4)

#define VMEDIGI_OFF_ACQ_CTR	 0x7fb32

#define VMEDIGI_OFF_AUX_CSR	 0x7fb1a
#define VMEDIGI_AUX_CSR_ADC_PGA       (1<<15)
#define VMEDIGI_AUX_CSR_ADC_RAND      (1<<14)
#define VMEDIGI_AUX_CSR_ADC_DITH      (1<<13)
#define VMEDIGI_AUX_CSR_ADC_SHDN      (1<<12)
#define VMEDIGI_AUX_CSR_SER_PAR       (1<<11)
#define VMEDIGI_AUX_CSR_EXT_ADC_CLK   (1<<10)
#define VMEDIGI_AUX_CSR_ALT_FORMAT    (1<< 9)
#define VMEDIGI_AUX_CSR_SW_INTRP      (1<< 8)
#define VMEDIGI_AUX_CSR_SIO_ACTIV     (1<< 7)
#define VMEDIGI_AUX_CSR_SIO_LOOPBK    (1<< 6)
#define VMEDIGI_AUX_CSR_SIO_NBITS(x)  ((x)&0x3f)

#define VMEDIGI_OFF_ACQ_CSR	 0x7fb36
#define VMEDIGI_ACQ_CSR_SW_ARM            (1<<15)	
#define VMEDIGI_ACQ_CSR_EXT_TRIG          (1<< 3)	
#define VMEDIGI_ACQ_CSR_TRIG_INP2         (1<< 2)	
#define VMEDIGI_ACQ_CSR_TRIG              (1<< 1)	
#define VMEDIGI_ACQ_CSR_SW_GATE           (1<< 0)	

#define VMEDIGI_OFF_IRQ_ID   0x7fb42

#define VMEDIGI_CR_ID					0x0015fac3

/* oldest firmware version we accept */
#define MIN_FW_VERS          0x20001

struct VmeDigiRec_ {
	VME64_Addr base; 	/* as seen from CPU */
	unsigned   irq_msk; /* matches STS layout; shift left by 8 to match CSR layout */
};

unsigned long
vmeDigiDetect(VME64_Addr csrbase, int quiet)
{
unsigned long csr_local;
uint32_t      brdid;

	if ( BSP_vme2local_adrs(VME_AM_CSR, csrbase, &csr_local) ) {
		if ( ! quiet )
			fprintf(stderr,"Unable to map CSR base address 0x%08"myPRIxPTR" to local/CPU address\n", csrbase);
		return (unsigned long)-1;
	}

	if ( vme64CRChecksum(csr_local, quiet) ) {
		if ( ! quiet )
			fprintf(stderr,"No valid VME64 CR Checksum at 0x%08"myPRIxPTR"\n", csrbase);
		return (unsigned long)-1;
	}
	if ( VMEDIGI_CR_ID != (brdid=vme64CSRegRead32(csr_local, VME64_CR_OFF_BRDID)) ) {
		if ( ! quiet )
			fprintf(stderr,"Unknown board ID found: 0x%08"PRIx32"\n", brdid);
		return (unsigned long)-1;
	}

	return csr_local;
}

VmeDigi
vmeDigiSetup(VME64_Addr csrbase, VME64_Addr a32base, uint8_t irq_vec, uint8_t irq_lvl)
{
unsigned long csr_local;
VmeDigi       rval = 0, digi = 0;
unsigned long fw_vers;

	csr_local = vmeDigiDetect(csrbase, 0);

	if ( (unsigned long)-1 == csr_local )
		goto cleanup;

	if ( !(digi = calloc(1,sizeof(*rval))) ) {
		fprintf(stderr,"No memory for VmeDigi struct\n");
		goto cleanup;
	}

	/* Disable */
	vme64_out08(csr_local, VME64_CSR_OFF_BCLR, VME64_CSR_BIT_MODENBL);

	/* Setup ADER */
	if ( vme64SetupADER(csr_local, 0, a32base, VME_AM_EXT_SUP_DATA) ) {
		fprintf(stderr,"Unable to program ADER for function #0\n");
		goto cleanup;
	}
	if ( vme64SetupADER(csr_local, 1, a32base, VME_AM_EXT_SUP_MBLT) ) {
		fprintf(stderr,"Unable to program ADER for function #1 (MBLT)\n");
		goto cleanup;
	}

	/* Check firmware version */
	fw_vers = vme64CSRegRead32(csr_local, VME64_CR_OFF_REVID);

	if ( MIN_FW_VERS > fw_vers ) {
		fprintf(stderr,"Firmware version (0x%lx) too old; need at least 0x%x\n", fw_vers, MIN_FW_VERS);
		goto cleanup;
	}

	/* Enable the thing */
	vme64_out08(csr_local, VME64_CSR_OFF_BSET, VME64_CSR_BIT_MODENBL);

	/* Setup */
	vme64_out16(csr_local, VMEDIGI_OFF_ACQ_CSR, VMEDIGI_ACQ_CSR_TRIG | VMEDIGI_ACQ_CSR_EXT_TRIG);
	/* Enable encoding of overflow condition in LSB */
	vme64_out16(csr_local, VMEDIGI_OFF_AUX_CSR, VMEDIGI_AUX_CSR_ALT_FORMAT);

	vme64_out16(csr_local, VMEDIGI_OFF_DCM0_CSR, VMEDIGI_DCM_CSR_BYPASS);
	vme64_out16(csr_local, VMEDIGI_OFF_DCM1_CSR, VMEDIGI_DCM_CSR_BYPASS);
	vme64_out16(csr_local, VMEDIGI_OFF_DCM2_CSR, VMEDIGI_DCM_CSR_BYPASS);
	vme64_out16(csr_local, VMEDIGI_OFF_DCM3_CSR, VMEDIGI_DCM_CSR_BYPASS);
	vme64_out16(csr_local, VMEDIGI_OFF_DCM4_CSR, VMEDIGI_DCM_CSR_BYPASS);

	vme64_out16(csr_local, VMEDIGI_OFF_IRQ_ID,   irq_vec);

	digi->irq_msk = VMEDIGI_IRQ_STS_STOP_IRQ | VMEDIGI_IRQ_STS_SW_IRQ;

	/* Clear possible pending interrupts */
	vme64_out16(csr_local, VMEDIGI_OFF_IRQ_STS, digi->irq_msk );

	/* Set irq level and enable acquisition STOP */
	vme64_out16(csr_local,
		VMEDIGI_OFF_IRQ_CSR, 
		VMEDIGI_IRQ_CSR_IRQ_LVL(irq_lvl) | VMEDIGI_IRQ_STS_2_CSR(digi->irq_msk)
	);

	digi->base = csr_local;

	rval = digi;
	digi = 0;

cleanup:
	if ( digi )
		free(digi);
	
	return rval;
}

uint32_t
vmeDigiSetCount(VmeDigi digi, uint32_t cnt)
{
	if ( cnt ) {
		if (cnt > VMEDIGI_CNT_MAX) {
			fprintf(stderr,"vmeDigiSetCount(): too large # of samples; setting to %u\n", VMEDIGI_CNT_MAX);
			cnt = VMEDIGI_CNT_MAX;
		}
		vme64_out16(digi->base, VMEDIGI_OFF_ACQ_CTR, VMEDIGI_CNT_MAX - cnt);
	}
	return VMEDIGI_CNT_MAX - vme64_in16(digi->base, VMEDIGI_OFF_ACQ_CTR);
}

uint16_t
vmeDigiArm(VmeDigi digi)
{
uint16_t val;
rtems_interrupt_level lvl;
	rtems_interrupt_disable(lvl);
	val = vme64_in16(digi->base, VMEDIGI_OFF_ACQ_CSR);
	if ( val & VMEDIGI_ACQ_CSR_SW_ARM ) {
		/* if it was already armed flash */
		vme64_out16( digi->base, VMEDIGI_OFF_ACQ_CSR, val & ~VMEDIGI_ACQ_CSR_SW_ARM );
	}
	vme64_out16( digi->base, VMEDIGI_OFF_ACQ_CSR, val | VMEDIGI_ACQ_CSR_SW_ARM );
	rtems_interrupt_enable(lvl);

	return val & VMEDIGI_ACQ_CSR_SW_ARM;
}

uint16_t
vmeDigiDisarm(VmeDigi digi)
{
uint16_t val;
rtems_interrupt_level lvl;

	rtems_interrupt_disable(lvl);
		val = vme64_in16(digi->base, VMEDIGI_OFF_ACQ_CSR);
		vme64_out16( digi->base, VMEDIGI_OFF_ACQ_CSR, val & ~VMEDIGI_ACQ_CSR_SW_ARM );
	rtems_interrupt_enable(lvl);

	return val & VMEDIGI_ACQ_CSR_SW_ARM;
}

void
vmeDigiSWTrig(VmeDigi digi)
{
uint16_t val,nval;

rtems_interrupt_level lvl;

	rtems_interrupt_disable(lvl);
		val = vme64_in16( digi->base, VMEDIGI_OFF_ACQ_CSR );
		if ( val & VMEDIGI_ACQ_CSR_SW_ARM ) {
			/* if it was already armed flash */
			vme64_out16( digi->base, VMEDIGI_OFF_ACQ_CSR, val & ~VMEDIGI_ACQ_CSR_SW_ARM );
		}
		nval  = val | VMEDIGI_ACQ_CSR_SW_ARM;
		nval &= ~ (VMEDIGI_ACQ_CSR_EXT_TRIG|VMEDIGI_ACQ_CSR_SW_GATE);
		/* set ARM, clear EXT trig and SW_GATE */
		vme64_out16( digi->base, VMEDIGI_OFF_ACQ_CSR, nval);
		/* raise SW_GATE */
		vme64_out16( digi->base, VMEDIGI_OFF_ACQ_CSR, nval | VMEDIGI_ACQ_CSR_SW_GATE );
		/* restore original value */
		vme64_out16( digi->base, VMEDIGI_OFF_ACQ_CSR, val );
	rtems_interrupt_enable(lvl);
}

/* Should NOT use this routine if AUX_CSR is controlled via devBusMapped! */
void
vmeDigiSWIntr(VmeDigi digi)
{
uint16_t              val;
rtems_interrupt_level lvl;

	rtems_interrupt_disable(lvl);
		val = vme64_in16( digi->base, VMEDIGI_OFF_AUX_CSR );
		if ( VMEDIGI_AUX_CSR_SW_INTRP & val )
			vme64_out16( digi->base, VMEDIGI_OFF_AUX_CSR, val & ~VMEDIGI_AUX_CSR_SW_INTRP );
		vme64_out16( digi->base, VMEDIGI_OFF_AUX_CSR, val |  VMEDIGI_AUX_CSR_SW_INTRP );
		if ( ! (VMEDIGI_AUX_CSR_SW_INTRP & val) )
			vme64_out16( digi->base, VMEDIGI_OFF_AUX_CSR, val );
	rtems_interrupt_enable(lvl);
}

void
vmeDigiIrqEnable(VmeDigi digi)
{
uint16_t              val;
rtems_interrupt_level lvl;

	rtems_interrupt_disable(lvl);
		val  = vme64_in16( digi->base, VMEDIGI_OFF_IRQ_CSR );
		val |= VMEDIGI_IRQ_CSR_IRQ_ENBL;
		vme64_out16( digi->base, VMEDIGI_OFF_IRQ_CSR, val );
	rtems_interrupt_enable(lvl);
}

int
vmeDigiIrqDisable(VmeDigi digi)
{
uint16_t              val;
rtems_interrupt_level lvl;

	rtems_interrupt_disable(lvl);
		val  = vme64_in16( digi->base, VMEDIGI_OFF_IRQ_CSR );
		vme64_out16( digi->base, VMEDIGI_OFF_IRQ_CSR, val & ~VMEDIGI_IRQ_CSR_IRQ_ENBL );
	rtems_interrupt_enable(lvl);

	rtems_interrupt_enable(lvl);
	return val & VMEDIGI_IRQ_CSR_IRQ_ENBL;
}

static unsigned 
doAck(VmeDigi digi, uint16_t mask)
{
uint16_t val;
	/* no need to disable IRQs -- safe by HW design (clear-on-write semantics) */
	val  = vme64_in16( digi->base, VMEDIGI_OFF_IRQ_STS );
	val &= mask;
	vme64_out16( digi->base, VMEDIGI_OFF_IRQ_STS, val );
	return val;
}

unsigned
vmeDigiIrqAck(VmeDigi digi)
{
uint16_t              val;

	val = doAck(digi, digi->irq_msk);

	/* read something back to make sure VME bridge synchronizes */
	vme64_in16( digi->base, VMEDIGI_OFF_IRQ_CSR );

	return val;
}

/* reset IRQ status bits that are NOT used by ISR(s) */
unsigned
vmeDigiIrqAckUnused(VmeDigi digi)
{
	return doAck(digi,~digi->irq_msk);
}

static int
dmp(char *buf, int len, char *fmt, uint32_t v)
{
int l;
	l = snprintf(buf, len, fmt, v);
	if ( l < 0 )
		return 0; /* nothing was written */

	return l > len ? len : l;
}

/* Dump info to buffer */
void
vmeDigiInfo(VmeDigi digi, char *buf, int len)
{
uint32_t v;
unsigned l;


	if ( 0 == len )
		return;

	len--; /* reserve space for trailing 0 */

	
	if ( 0 == len )
		goto bail;

	v = vme64CSRegRead32( digi->base, VME64_CR_OFF_MANID );
	l = dmp(buf, len, "Manufacturer: 0x%"PRIx32, v);
	buf+=l;
	len-=l;

	if ( 0 == len )
		goto bail;

	v = vme64CSRegRead32( digi->base, VME64_CR_OFF_BRDID );
	l = dmp(buf, len, ", Board (drawing) Rev: 0x%"PRIx32, v);
	buf+=l;
	len-=l;

	if ( 0 == len )
		goto bail;

	v = vme64CSRegRead32( digi->base, VME64_CR_OFF_REVID );
	l = dmp(buf, len, ", Firmware Rev: 0x%"PRIx32, v);
	buf+=l;
	len-=l;

	if ( 0 == len )
		goto bail;

	v = vme64ReadSN( digi->base );
	l = dmp(buf, len, ", S/N 0x%"PRIx32, v);
	buf+=l;
	len-=l;

bail:
	*buf = 0; /* space for this was reserved at the beginning */
}

#if 0
uint32_t tdiff;
rtems_id syncsem;

static void mycb(void *arg)
{
uint32_t x;
	asm volatile("mftb %0":"=r"(x));
	tdiff = x-tdiff;
	rtems_semaphore_release(syncsem);
}

void
_cexpModuleInitialize(void*h)
{
	if ( rtems_semaphore_create(
		rtems_build_name('d','m','a','S'),
		0,
		RTEMS_SIMPLE_BINARY_SEMAPHORE,
		0,
		&syncsem) ) {
		fprintf(stderr,"Creating semaphore failed\n");
	} else {
		if ( BSP_VMEDmaInstallISR(0,mycb,0) )
			fprintf(stderr,"Hooking ISR failed\n");
	}
}


int
_cexpModuleFinalize(void*h)
{
	BSP_VMEDmaInstallISR(0,0,0);
	if ( syncsem )
		rtems_semaphore_delete(syncsem);
	return 0;
}

uint32_t vmedma_base=0x48000000;

uint32_t dodma(uint32_t buf, uint32_t len)
{
	asm volatile("mftb %0":"=r"(tdiff));
	if ( BSP_VMEDmaStart(0, buf, vmedma_base, len) ) {
		fprintf(stderr,"Starting DMA failed\n");
		return -1;
	}
	if ( rtems_semaphore_obtain(syncsem, RTEMS_WAIT, 20) ) {
		fprintf(stderr,"Unable to take semaphore\n");
	}
	return tdiff;
}
#endif
