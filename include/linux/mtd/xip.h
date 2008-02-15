/*
 * MTD primitives for XIP support
 *
 * Author:	Nicolas Pitre
 * Created:	Nov 2, 2004
 * Copyright:	(C) 2004 MontaVista Software, Inc.
 *
 * This XIP support for MTD has been loosely inspired
 * by an earlier patch authored by David Woodhouse.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __LINUX_MTD_XIP_H__
#define __LINUX_MTD_XIP_H__

#include <linux/config.h>
#include <asm/hardware.h>


#ifdef CONFIG_MTD_XIP

/*
 * Function that are modifying the flash state away from array mode must
 * obviously not be running from flash.  The __xipram is therefore marking
 * those functions so they get relocated to ram.
 */
#define __xipram __attribute__ ((__section__ (".data")))

/*
 * We really don't want gcc to guess anything.
 * We absolutely _need_ proper inlining.
 */
#define inline __inline__ __attribute__((always_inline))

/*
 * Each architecture has to provide the following macros.  They must access
 * the hardware directly and not rely on any other (XIP) functions since they
 * won't be available when used (flash not in array mode).
 *
 * xip_irqpending()
 *
 * 	return non zero when any hardware interrupt is pending.
 *
 * xip_currtime()
 *
 * 	return a platform specific time reference to be used with
 * 	xip_elapsed_since().
 *
 * xip_elapsed_since(x)
 *
 * 	return in usecs the elapsed timebetween now and the reference x as
 * 	returned by xip_currtime().
 *
 * 	note 1: convertion to usec can be approximated, as long as the
 * 		returned value is <= the real elapsed time.
 * 	note 2: this should be able to cope with a few seconds without
 * 		overflowing.
 */

#if defined(CONFIG_ARCH_SA1100) || defined(CONFIG_ARCH_PXA)

#define xip_irqpending()	(ICIP & ICMR)

/* we sample OSCR and convert desired delta to usec (1/4 ~= 1000000/3686400) */
#define xip_currtime()		(OSCR)
#define xip_elapsed_since(x)	(signed)((OSCR - (x)) / 4)

#elif defined(CONFIG_ARCH_OMAP1610)

#include <asm/arch/timer.h>

#define __read_ih1(reg) *(volatile u32 *)(IO_ADDRESS(OMAP_IH1_BASE + (reg)))
#define __read_ih2_0(reg) *(volatile u32 *)(IO_ADDRESS(OMAP_IH2_0_BASE + (reg)))

#define xip_irqpending()	  ((__read_ih1(IRQ_ITR) & ~__read_ih1(IRQ_MIR)) \
				|| (__read_ih2_0(IRQ_ITR) & ~__read_ih2_0(IRQ_MIR)))

#define xip_currtime()		(mputimer_base(0)->read_tim)
#define xip_elapsed_since(x)	(signed)(((x) - mputimer_base(0)->read_tim) / 6) /*downcounting*/

#else
#error "missing IRQ and timer primitives for XIP MTD support"
#endif

/*
 * xip_cpu_idle() is used when waiting for a delay equal or larger than
 * the system timer tick period.  This should put the CPU into idle mode
 * to save power and to be woken up only when some interrupts are pending.
 * As above, this should not rely upon standard kernel code.
 */

#if defined(CONFIG_CPU_XSCALE)
#define xip_cpu_idle()  asm volatile ("mcr p14, 0, %0, c7, c0, 0" :: "r" (1))
#elif defined(CONFIG_CPU_ARM926T)
#define xip_cpu_idle()  asm volatile ("mcr p15, 0, r0, c7, c0, 4")
#else
#define xip_cpu_idle()  do { } while (0)
#endif

#else

#define __xipram

#endif

#endif /* __LINUX_MTD_XIP_H__ */
