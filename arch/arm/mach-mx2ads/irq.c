/*
 *  linux/arch/arm/mach-mx2ads/irq.c
 *
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/init.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <linux/timer.h>
#include <linux/list.h>

#include <asm/mach/irq.h>

/*
 *
 * We simply use the ENABLE DISABLE registers inside of the mx2
 * to turn on/off specific interrupts.  FIXME- We should
 * also add support for the accelerated interrupt controller
 * by putting offets to irq jump code in the appropriate
 * places.
 *
 */

#ifndef U32
typedef unsigned long U32;
#endif

#define TYPE_BIT_MASK(n) 		(1 << (n))


static void
mx2ads_mask_irq(unsigned int irq)
{
	AITC_INTDISNUM = irq;
}

static void
mx2ads_unmask_irq(unsigned int irq)
{
	AITC_INTENNUM = irq;
}

void mx2ads_mask_fiq(unsigned int irq)
{
	AITC_INTDISNUM = (unsigned long) irq;
}

void mx2ads_unmask_fiq(unsigned int irq)
{
	AITC_INTENNUM = (unsigned long)irq;
}

void mx2ads_set_type_to_fiq(unsigned int irq)
{
	if (irq >= 32)
	{
		AITC_INTTYPEH |= TYPE_BIT_MASK(irq-32);
	}
	else
	{
		AITC_INTTYPEL |= TYPE_BIT_MASK(irq);
	}
}

void mx2ads_set_type_to_irq(unsigned int irq)
{
	if (irq >= 32)
	{
		AITC_INTTYPEH &= ~TYPE_BIT_MASK(irq-32);
	}
	else
	{
		AITC_INTTYPEL &= ~TYPE_BIT_MASK(irq);
	}
}

unsigned int mx2ads_get_fiq_num(void)
{
	return (AITC_FIVECSR);
}

void __init
mx2ads_init_irq(void)
{
	unsigned int i;

	for (i = 0; i < NR_IRQS; i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 1;
		irq_desc[i].mask_ack = mx2ads_mask_irq;
		irq_desc[i].mask = mx2ads_mask_irq;
		irq_desc[i].unmask = mx2ads_unmask_irq;
	}

	/* Disable all interrupts initially. */
	/* In mx2 this is done in the bootloader. */
}
