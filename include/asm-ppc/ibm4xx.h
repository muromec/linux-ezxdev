/*
 *
 *    Copyright (c) 1999 Grant Erickson <grant@lcse.umn.edu>
 *
 *    Module name: ibm4xx.h
 *
 *    Description:
 *	A generic include file which pulls in appropriate include files
 *      for specific board types based on configuration settings.
 *
 */

#ifdef __KERNEL__
#ifndef __ASM_IBM4XX_H__
#define __ASM_IBM4XX_H__

#include <linux/config.h>

#ifdef CONFIG_4xx

#ifndef __ASSEMBLY__
#include <linux/types.h>
/*
 * The "residual" board information structure the boot loader passes
 * into the kernel.
 */
extern unsigned char __res[];

/* Device Control Registers */

#define stringify(s)	tostring(s)
#define tostring(s)	#s

#define mfdcr(rn) mfdcr_or_dflt(rn, 0)

#define mfdcr_or_dflt(rn,default_rval) \
	({unsigned int rval;						\
	if (rn == 0)							\
		rval = default_rval;					\
	else								\
		asm volatile("mfdcr %0," stringify(rn) : "=r" (rval));	\
	rval;})

#define mtdcr(rn, v)  \
do {                  \
	if (rn != 0) \
		asm volatile("mtdcr " stringify(rn) ",%0" : : "r" (v));	\
} while (0)

/* R/W of indirect DCRs make use of standard naming conventions for DCRs */

#define mfdcri(base, reg)			\
({						\
     mtdcr(base##_CFGADDR, base##_##reg);	\
     mfdcr(base##_CFGDATA);			\
})

#define mtdcri(base, reg, data)			\
do {						\
     mtdcr(base##_CFGADDR, base##_##reg);	\
     mtdcr(base##_CFGDATA, data);		\
} while (0)

/* Power Management */

void ppc4xx_cpm_fr(u32 bits, int val);

#endif /* __ASSEMBLY__ */

#endif /* CONFIG_4xx */

#ifdef CONFIG_40x

#if defined(CONFIG_ASH)
#include <platforms/ash.h>
#endif

#if defined (CONFIG_CEDER)
#include <platforms/ceder.h>
#endif

#if defined(CONFIG_CPCI405)
#include <platforms/cpci405.h>
#endif

#if defined(CONFIG_EP405)
#include <platforms/ep405.h>
#endif

#if defined(CONFIG_OAK)
#include <platforms/oak.h>
#endif

#if defined(CONFIG_XILINX_ML300)
#include <platforms/xilinx_ml300.h>
#endif

#if defined(CONFIG_RAINIER)
#include <platforms/rainier.h>
#endif

#if defined(CONFIG_REDWOOD_4)
#include <platforms/redwood.h>
#endif

#if defined(CONFIG_REDWOOD_5)
#include <platforms/redwood5.h>
#endif

#if defined(CONFIG_REDWOOD_6)
#include <platforms/redwood6.h>
#endif

#if defined(CONFIG_WALNUT)
#include <platforms/walnut.h>
#endif

#if defined(CONFIG_BEECH)
#include <platforms/beech.h>
#endif

#if defined(CONFIG_ARCTIC2)
#include <platforms/arctic2.h>
#endif

#if defined(CONFIG_SYCAMORE)
#include <platforms/walnut.h>
#endif

#ifndef PPC4xx_MACHINE_NAME
#define PPC4xx_MACHINE_NAME	"Unidentified 4xx class"
#endif



/* IO_BASE is for PCI I/O.
 * ISA not supported, just here to resolve copilation.
 */

#ifndef _IO_BASE
#define _IO_BASE	0xe8000000	/* The PCI address window */
#define _ISA_MEM_BASE	0
#define PCI_DRAM_OFFSET	0
#endif

/*
 * The "residual" board information structure the boot loader passes
 * into the kernel.
 */

#elif CONFIG_440

#if defined(CONFIG_EBONY)
#include <platforms/ebony.h>
#endif

#endif /* CONFIG_40x */
#endif /* __ASM_IBM4XX_H__ */
#endif /* __KERNEL__ */
