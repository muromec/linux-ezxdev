/*
 *  linux/arch/arm/mach-pxa/irq.c
 *  
 *  Generic PXA IRQ handling, GPIO IRQ demultiplexing, etc.
 *
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *  
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *  
 *  Copyright (C) 2005 - Motorola	
 *
 *
 *  history: 
 *  modified by w19962 for Motorola EZX platform
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/arch/irq.h>

#include "generic.h"


/*
 * PXA GPIO edge detection for IRQs:
 * IRQs are generated on Falling-Edge, Rising-Edge, or both.
 * This must be called *before* the appropriate IRQ is registered.
 * Use this instead of directly setting GRER/GFER.
 */

static long GPIO_IRQ_rising_edge[4];
static long GPIO_IRQ_falling_edge[4];

void set_GPIO_IRQ_edge (int gpio_nr, int edge)
{
	long flags;
	local_irq_save(flags);
	set_GPIO_mode(gpio_nr | GPIO_IN);
	if (edge & GPIO_FALLING_EDGE)
		set_bit (gpio_nr, GPIO_IRQ_falling_edge);
	else
		clear_bit (gpio_nr, GPIO_IRQ_falling_edge);
	if (edge & GPIO_RISING_EDGE)
		set_bit (gpio_nr, GPIO_IRQ_rising_edge);
	else
		clear_bit (gpio_nr, GPIO_IRQ_rising_edge);
	irq_desc[IRQ_GPIO(gpio_nr)].valid = 1;
	local_irq_restore(flags);
}

EXPORT_SYMBOL(set_GPIO_IRQ_edge);


/*
 * We don't need to ACK IRQs on the PXA unless they're GPIOs
 * this is for IRQs known as PXA_IRQ([10...31]).
 */

static void pxa_mask_irq(unsigned int irq)
{
	ICMR &= ~(1 << (irq + PXA_IRQ_SKIP));
}

static void pxa_unmask_irq(unsigned int irq)
{
	ICMR |= (1 << (irq + PXA_IRQ_SKIP));
}

#ifdef CONFIG_CPU_BULVERDE
/*
 * We don't need to ACK IRQs on the PXA unless they're GPIOs
 * this is for IRQs known as PXA_IRQ([32...63]).
 */

static void pxa_mask_irq_high(unsigned int irq)
{
	ICMR2 &= ~(1 << (irq - 32 + PXA_IRQ_SKIP));
}

static void pxa_unmask_irq_high(unsigned int irq)
{
	ICMR2 |= (1 << (irq - 32 + PXA_IRQ_SKIP));
}
#endif

/*
 * GPIO IRQs must be acknoledged.  This is for GPIO 0 and 1.
 */

static void pxa_mask_and_ack_GPIO_0_1_irq(unsigned int irq)
{
	ICMR &= ~(1 << (irq + PXA_IRQ_SKIP));
	GEDR0 = (1 << (irq - IRQ_GPIO0));
}

static void pxa_mask_GPIO_0_1_irq(unsigned int irq)
{
	ICMR &= ~(1 << (irq + PXA_IRQ_SKIP));
}

static void pxa_unmask_GPIO_0_1_irq(unsigned int irq)
{
	int gpio = irq - IRQ_GPIO0;
	GRER0 = (GRER0 & ~(1 << gpio))|(GPIO_IRQ_rising_edge[0] & (1 << gpio));
	GFER0 = (GFER0 & ~(1 << gpio))|(GPIO_IRQ_falling_edge[0] & (1 << gpio));
	ICMR |= (1 << (irq + PXA_IRQ_SKIP));
}

/*
 * Demux handler for GPIO>=2 edge detect interrupts
 */

static int GPIO_2_x_enabled[4];	/* enabled i.e. unmasked GPIO IRQs */
static int GPIO_2_x_spurious[4];	/* GPIOs that triggered when masked */

static void pxa_GPIO_IRQ_demux( int irq, void *dev_id,
				struct pt_regs *regs)
{
	int i, gedr, spurious;
		
	while ((gedr = (GEDR0 & ~3))) {
		/*
		 * We don't want to clear GRER/GFER when the corresponding
		 * IRQ is masked because we could miss a level transition
		 * i.e. an IRQ which need servicing as soon as it is
		 * unmasked.  However, such situation should happen only
		 * during the loop below.  Thus all IRQs which aren't
		 * enabled at this point are considered spurious.  Those
		 * are cleared but only de-activated if they happen twice.
		 */
		spurious = gedr & ~GPIO_2_x_enabled[0];
		if (spurious) {
			GEDR0 = spurious;
			GRER0 &= ~(spurious & GPIO_2_x_spurious[0]);
			GFER0 &= ~(spurious & GPIO_2_x_spurious[0]);
			GPIO_2_x_spurious[0] |= spurious;
			gedr ^= spurious;
			if (!gedr) continue;
		}

		for (i = 2; i < 32; ++i) {
			if (gedr & (1<<i)) {
				do_IRQ (IRQ_GPIO(2) + i - 2, regs);
			}
		}
	}
	while ((gedr = GEDR1)) {
		spurious = gedr & ~GPIO_2_x_enabled[1];
		if (spurious) {
			GEDR1 = spurious;
			GRER1 &= ~(spurious & GPIO_2_x_spurious[1]);
			GFER1 &= ~(spurious & GPIO_2_x_spurious[1]);
			GPIO_2_x_spurious[1] |= spurious;
			gedr ^= spurious;
			if (!gedr) continue;
		}

		for (i = 0; i < 32; ++i) {
			if (gedr & (1<<i)) {
				do_IRQ (IRQ_GPIO(32) + i, regs);
			}
		}
	}
	while ((gedr = GEDR2)) {
		spurious = gedr & ~GPIO_2_x_enabled[2];
		if (spurious) {
			GEDR2 = spurious;
			GRER2 &= ~(spurious & GPIO_2_x_spurious[2]);
			GFER2 &= ~(spurious & GPIO_2_x_spurious[2]);
			GPIO_2_x_spurious[2] |= spurious;
			gedr ^= spurious;
			if (!gedr) continue;
		}

		for (i = 0; i < 32; ++i) {
			if (gedr & (1<<i)) {
				do_IRQ (IRQ_GPIO(64) + i, regs);
			}
		}
	}
#ifdef CONFIG_CPU_BULVERDE
	while ((gedr = GEDR3)) {
		spurious = gedr & ~GPIO_2_x_enabled[3];
		if (spurious) {
			GEDR3 = spurious;
			GRER3 &= ~(spurious & GPIO_2_x_spurious[3]);
			GFER3 &= ~(spurious & GPIO_2_x_spurious[3]);
			GPIO_2_x_spurious[3] |= spurious;
			gedr ^= spurious;
			if (!gedr) continue;
		}

		for (i = 0; i < 32; ++i) {
			if (gedr & (1<<i)) {
				do_IRQ (IRQ_GPIO(96) + i, regs);
			}
		}
	}
#endif
}

static struct irqaction GPIO_2_x_irqaction = {
	name:		"GPIO demux",
	handler:	pxa_GPIO_IRQ_demux,
	flags:		SA_INTERRUPT
};

static void pxa_mask_and_ack_GPIO_2_x_irq(unsigned int irq)
{
	int gpio_nr = IRQ_TO_GPIO_2_x(irq);
	int mask = 1 << (gpio_nr & 0x1f);
	int index = gpio_nr >> 5;
	GPIO_2_x_spurious[index] &= ~mask;
	GPIO_2_x_enabled[index] &= ~mask;
	GEDR(gpio_nr) = mask;
}

static void pxa_mask_GPIO_2_x_irq(unsigned int irq)
{
	int gpio_nr = IRQ_TO_GPIO_2_x(irq);
	int mask = 1 << (gpio_nr & 0x1f);
	int index = gpio_nr >> 5;
	GPIO_2_x_spurious[index] &= ~mask;
	GPIO_2_x_enabled[index] &= ~mask;
}

static void pxa_unmask_GPIO_2_x_irq(unsigned int irq)
{
	int gpio_nr = IRQ_TO_GPIO_2_x(irq);
	int mask = 1 << (gpio_nr & 0x1f);
	int index = gpio_nr >> 5;
	
	if (GPIO_2_x_spurious[index] & mask) {
		/*
		 * We don't want to miss an interrupt that would have occurred
		 * while it was masked.  Simulate it if it is the case.
		 */
		int state = GPLR(gpio_nr);
		if (((state & GPIO_IRQ_rising_edge[index]) |
		     (~state & GPIO_IRQ_falling_edge[index])) & mask)
		{
			/* just in case it gets referenced: */
			struct pt_regs dummy;

			memzero(&dummy, sizeof(dummy));
			do_IRQ(irq, &dummy);

			/* we are being called recursively from do_IRQ() */
			return;
		}
	}
	GPIO_2_x_enabled[index] |= mask;

	GRER(gpio_nr) =
		(GRER(gpio_nr) & ~mask) | (GPIO_IRQ_rising_edge[index] & mask);

	GFER(gpio_nr) =
		(GFER(gpio_nr) & ~mask) | (GPIO_IRQ_falling_edge[index] & mask);

}


void __init pxa_init_irq(void)
{
	int irq;

	/* disable all IRQs */
	ICMR = 0;
	ICMR2 = 0;

	/* all IRQs are IRQ, not FIQ */
	ICLR = 0;
	ICLR2 = 0;

	/* clear all GPIO edge detects */
	GFER0 = GFER1 = GFER2 = GFER3 = 0;
	GRER0 = GRER1 = GRER2 = GRER3 = 0;
	GEDR0 = GEDR0;
	GEDR1 = GEDR1;
	GEDR2 = GEDR2;
	GEDR3 = GEDR3;

	/* only unmasked interrupts kick us out of idle */
	ICCR = 1;

	for (irq = PXA_IRQ(PXA_IRQ_SKIP); irq <= PXA_IRQ(31); irq++) {
		irq_desc[irq].valid	= 1;
		irq_desc[irq].probe_ok	= 0;
		irq_desc[irq].mask_ack	= pxa_mask_irq;
		irq_desc[irq].mask	= pxa_mask_irq;
		irq_desc[irq].unmask	= pxa_unmask_irq;
	}

#ifdef CONFIG_CPU_BULVERDE
	for (irq = PXA_IRQ(32); irq <= PXA_IRQ(63); irq++) {
		irq_desc[irq].valid	= 1;
		irq_desc[irq].probe_ok	= 0;
		irq_desc[irq].mask_ack	= pxa_mask_irq_high;
		irq_desc[irq].mask	= pxa_mask_irq_high;
		irq_desc[irq].unmask	= pxa_unmask_irq_high;
	}
#endif

	/*
	 * Note: GPIO IRQs are initially invalid until set_GPIO_IRQ_edge()
	 * is called at least once.
	 */

	for (irq = IRQ_GPIO0; irq <= IRQ_GPIO1; irq++) {
		irq_desc[irq].valid	= 0;
		irq_desc[irq].probe_ok	= 1;
		irq_desc[irq].mask_ack	= pxa_mask_and_ack_GPIO_0_1_irq;
		irq_desc[irq].mask	= pxa_mask_GPIO_0_1_irq;
		irq_desc[irq].unmask	= pxa_unmask_GPIO_0_1_irq;
	}

	for (irq = IRQ_GPIO(2); irq <= IRQ_GPIO_LAST; irq++) {
		irq_desc[irq].valid	= 0;
		irq_desc[irq].probe_ok	= 1;
		irq_desc[irq].mask_ack	= pxa_mask_and_ack_GPIO_2_x_irq;
		irq_desc[irq].mask	= pxa_mask_GPIO_2_x_irq;
		irq_desc[irq].unmask	= pxa_unmask_GPIO_2_x_irq;
	}
	setup_arm_irq( IRQ_GPIO_2_x, &GPIO_2_x_irqaction );
}
