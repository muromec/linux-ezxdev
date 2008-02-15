/*
 *  linux/arch/arm/mach-pxa/mainstone.c
 *
 *  Support for the Intel HCDDBBVA0 Development Platform.
 *  (go figure how they came up with such name...)
 *  
 *  Author:	Nicolas Pitre
 *  Created:	Nov 05, 2002
 *  Copyright:	MontaVista Software Inc.
 *  
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/bitops.h>

#include <asm/types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/arch/irq.h>
#include <asm/arch/mainstone.h>

#include "generic.h"


static unsigned long mainstone_irq_enabled;

static void mainstone_mask_irq(unsigned int irq)
{
	int mainstone_irq = (irq - MAINSTONE_IRQ(0));
	MST_INTMSKENA = (mainstone_irq_enabled &= ~(1 << mainstone_irq));
}

static void mainstone_unmask_irq(unsigned int irq)
{
	int mainstone_irq = (irq - MAINSTONE_IRQ(0));
	/* the irq can be acknowledged only if deasserted, so it's done here */
	MST_INTSETCLR &= ~(1 << mainstone_irq);
	MST_INTMSKENA = (mainstone_irq_enabled |= (1 << mainstone_irq));
}

void mainstone_irq_demux(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned long pending = MST_INTSETCLR & mainstone_irq_enabled;
	do {
		GEDR(0) = GPIO_bit(0);  /* clear useless edge notification */
		if (likely(pending))
			do_IRQ( MAINSTONE_IRQ(0) + __ffs(pending), regs );
		pending = MST_INTSETCLR & mainstone_irq_enabled;
	} while (pending);
}

static struct irqaction mainstone_irq = {
	name:		"Mainstone FPGA",
	handler:	mainstone_irq_demux,
	flags:		SA_INTERRUPT
};

static void __init mainstone_init_irq(void)
{
	int irq;
	
	pxa_init_irq();

	/* setup extra Mainstone irqs */
	for(irq = MAINSTONE_IRQ(0); irq <= MAINSTONE_IRQ(15); irq++) {
		irq_desc[irq].valid	= 1;
		irq_desc[irq].probe_ok	= 1;
		irq_desc[irq].mask_ack	= mainstone_mask_irq;
		irq_desc[irq].mask	= mainstone_mask_irq;
		irq_desc[irq].unmask	= mainstone_unmask_irq;
	}

	irq_desc[MAINSTONE_IRQ(8)].valid = 0;
	irq_desc[MAINSTONE_IRQ(12)].valid = 0;

	MST_INTMSKENA = 0;
	MST_INTSETCLR = 0;

	set_GPIO_IRQ_edge(0, GPIO_FALLING_EDGE);
	setup_arm_irq(IRQ_GPIO0, &mainstone_irq);
}


static struct map_desc mainstone_io_desc[] __initdata = {
/* virtual        physical       length      domain     r  w  c  b */
 { MST_FPGA_VIRT, MST_FPGA_PHYS, 0x00100000, DOMAIN_IO, 0, 1, 0, 0 },
 { MST_ETH_VIRT,  MST_ETH_PHYS,  0x00100000, DOMAIN_IO, 0, 1, 0, 0 },
   LAST_DESC
};

static void __init mainstone_map_io(void)
{
	pxa_map_io();
	iotable_init(mainstone_io_desc);

#if 0
	/* This enables the BTUART */
	CKEN |= CKEN7_BTUART;
	set_GPIO_mode(GPIO42_BTRXD_MD);
	set_GPIO_mode(GPIO43_BTTXD_MD);
	set_GPIO_mode(GPIO44_BTCTS_MD);
	set_GPIO_mode(GPIO45_BTRTS_MD);

	/* This is for the SMC chip select */
	set_GPIO_mode(GPIO79_nCS_3_MD);

	/* setup sleep mode values */
	PWER  = 0x00000003;
	PFER  = 0x00000000;
	PRER  = 0x00000003;
	PGSR0 = 0x00008000;
	PGSR1 = 0x003F0202;
	PGSR2 = 0x0001C000;
	PCFR  = 0x66;
	PSLR  |= 0x4;
	/*  PCFR |= PCFR_OPDE; */
#endif
}

MACHINE_START(MAINSTONE, "Intel HCDDBBVA0 Development Platform")
	MAINTAINER("MontaVista Software Inc.")
	BOOT_MEM(0xa0000000, 0x40000000, io_p2v(0x40000000))
	MAPIO(mainstone_map_io)
	INITIRQ(mainstone_init_irq)
MACHINE_END
