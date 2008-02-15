#ifndef __ASM_XTENSA_IRQ_H
#define __ASM_XTENSA_IRQ_H

/*
 * include/asm-xtensa/irq.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Chris Songer, Kevin Chea
 */

#include <linux/config.h>
#include <asm/platform/hardware.h>

#include <xtensa/config/core.h>

#ifndef PLATFORM_NR_IRQS
# define PLATFORM_NR_IRQS 0
#endif
#define XT_NR_IRQS XCHAL_NUM_INTERRUPTS
#define NR_IRQS (XT_NR_IRQS + PLATFORM_NR_IRQS)

static __inline__ int irq_cannonicalize(int irq)
{
	return (irq);
}

struct irqaction;
extern void disable_irq(unsigned int);
extern void enable_irq(unsigned int);

#endif /* __ASM_XTENSA_IRQ_H */
