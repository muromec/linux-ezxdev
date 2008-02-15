/*
 * linux/arch/arm/mach-omap/innovator.c
 *
 * This file contains Innovator-specific code.
 *
 * Copyright (C) 2002 MontaVista Software, Inc.
 *
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/serial.h>
#include <linux/serial_core.h>

#include <asm/serial.h>
#include <asm/string.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/page.h>
#include <asm/pgtable.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/arch/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/arch/innovator.h>
#include <asm/arch/ck.h>
#include <asm/arch/irqs.h>

#include "generic.h"

void
fpga_write(unsigned char val, int reg)
{
	outb(val, reg);
}

unsigned char
fpga_read(int reg)
{
	return inb(reg);
}

/*
 *      FPGA irq routines
 */

static void
fpga_mask_irq(unsigned int irq)
{
	irq -= IH_FPGA_BASE;

	if (irq < 8)
		outb((inb(OMAP1510P1_FPGA_IMR_LO) & ~(1 << irq)),
		     OMAP1510P1_FPGA_IMR_LO);
	else if (irq < 16)
		outb((inb(OMAP1510P1_FPGA_IMR_HI) & ~(1 << (irq - 8))),
		     OMAP1510P1_FPGA_IMR_HI);
	else
		outb((inb(INNOVATOR_FPGA_IMR2) & ~(1 << (irq - 16))),
		     INNOVATOR_FPGA_IMR2);
}


static inline u32
get_fpga_unmasked_irqs(void)
{
	return
		((inb(OMAP1510P1_FPGA_ISR_LO) &
		  inb(OMAP1510P1_FPGA_IMR_LO))) |
		((inb(OMAP1510P1_FPGA_ISR_HI) &
		  inb(OMAP1510P1_FPGA_IMR_HI)) << 8) |
		((inb(INNOVATOR_FPGA_ISR2) &
		  inb(INNOVATOR_FPGA_IMR2)) << 16);
}


static void
fpga_ack_irq(unsigned int irq)
{
	/* Don't need to explicitly ACK FPGA interrupts */
}

static void
fpga_unmask_irq(unsigned int irq)
{
	irq -= IH_FPGA_BASE;

	if (irq < 8)
		outb((inb(OMAP1510P1_FPGA_IMR_LO) | (1 << irq)),
		     OMAP1510P1_FPGA_IMR_LO);
	else if (irq < 16)
		outb((inb(OMAP1510P1_FPGA_IMR_HI) | (1 << (irq - 8))),
		     OMAP1510P1_FPGA_IMR_HI);
	else
		outb((inb(INNOVATOR_FPGA_IMR2) | (1 << (irq - 16))),
		     INNOVATOR_FPGA_IMR2);
}

static void
fpga_mask_ack_irq(unsigned int irq)
{
	fpga_mask_irq(irq);
	fpga_ack_irq(irq);
}

static void
innovator_fpga_IRQ_demux(int irq, void *dev_id, struct pt_regs *regs)
{
	u32 stat;
	int fpga_irq;

	for (;;) {
		stat = get_fpga_unmasked_irqs();

		if (!stat) {
#ifdef CONFIG_INNOVATOR_MISSED_IRQS
			/* We are having a problem with missed Ethernet 
			 * interrupts on Innovators with FPGA Revision 2.0.
			 * When an interrupt is missed, the Ethernet controller 
			 * is always asserting a level interrupt request, but 
			 * no interrupt request is latched in the FPGA 
			 * interrupt status register.  Our (unproven) theory 
			 * is that there is a race condition in the FPGA such 
			 * that if the status register is read at the same time 
			 * a new interrupt request is asserted that the request 
			 * will be cleared before it is ever latched in the 
			 * status register and a zero will be returned.
			 * Our only workaround for this situation is to call 
			 * all of the installed FPGA interrupt handlers after 
			 * we read a zero from the FPGA status register.  This 
			 * seems to be completely effective at eliminating the 
			 * missed Ethernet interrupts.  We're assuming that all 
			 * of the edge-sensitive FPGA interrupts are subject to 
			 * the same problem, although we haven't observed it for 
			 * anything other than Ethernet.
			 */
			u32 mask = inb(OMAP1510P1_FPGA_IMR_LO) 
				| (inb(OMAP1510P1_FPGA_IMR_HI) << 8)
				| (inb(INNOVATOR_FPGA_IMR2) << 16);

			if (!mask)
				break;

			for (fpga_irq = IH_FPGA_BASE;
			     (fpga_irq < (IH_FPGA_BASE + NR_FPGA_IRQS)) && mask;
			     fpga_irq++, mask >>= 1)
			{
				if ((mask & 1) && (fpga_irq != INT_FPGA_TS))
					do_IRQ(fpga_irq, regs);
			}
#endif	/* CONFIG_INNOVATOR_MISSED_IRQS */
			break;
		}

		for (fpga_irq = IH_FPGA_BASE;
			(fpga_irq < (IH_FPGA_BASE + NR_FPGA_IRQS)) && stat;
			fpga_irq++, stat >>= 1) {
			if (stat & 1)
				do_IRQ(fpga_irq, regs);
		}
	}
}

static struct irqaction innovator_fpga_irq = {
        .name           = "Innovator FPGA IRQ demux",
        .handler        = innovator_fpga_IRQ_demux,
        .flags          = SA_INTERRUPT
};

static void
fpga_init_irq(void)
{
	int i;

	outb(0, OMAP1510P1_FPGA_IMR_LO);
	outb(0, OMAP1510P1_FPGA_IMR_HI);
	outb(0, INNOVATOR_FPGA_IMR2);

	/* 
	 * All of the FPGA interrupt request inputs except for the touchscreen are 
	 * edge-sensitive; the touchscreen is level-sensitive.  The edge-sensitive 
	 * interrupts are acknowledged as a side-effect of reading the interrupt 
	 * status register from the FPGA.  The edge-sensitive interrupt inputs 
	 * cause a problem with level interrupt requests, such as Ethernet.  The 
	 * problem occurs when a level interrupt request is asserted while its 
	 * interrupt input is masked in the FPGA, which results in a missed 
	 * interrupt.
	 * 
	 * In an attempt to workaround the problem with missed interrupts, the 
	 * mask_ack routine for all of the FPGA interrupts has been changed from 
	 * fpga_mask_ack_irq() to fpga_ack_irq() so that the specific FPGA interrupt 
	 * being serviced is left unmasked.  We can do this because the FPGA cascade 
	 * interrupt is installed with the SA_INTERRUPT flag, which leaves all 
	 * interrupts masked at the CPU while an FPGA interrupt handler executes.
	 * 
	 * Limited testing indicates that this workaround appears to be effective 
	 * for the smc9194 Ethernet driver used on the Innovator.  It should work 
	 * on other FPGA interrupts as well, but any drivers that explicitly mask 
	 * interrupts at the interrupt controller via disable_irq/enable_irq 
	 * could pose a problem.
	 */
	for (i = IH_FPGA_BASE; i < (IH_FPGA_BASE + NR_FPGA_IRQS); i++) {
		irq_desc[i].valid = 1;
		irq_desc[i].probe_ok = 0;
		if (i == INT_FPGA_TS) {
			/* The touchscreen interrupt is level-sensitive, so 
			 * we'll use the regular mask_ack routine for it.
			 */
			irq_desc[i].mask_ack = fpga_mask_ack_irq;
		}
		else {
			/* All FPGA interrupts except the touchscreen are 
			 * edge-sensitive, so we won't mask them.
			 */
			irq_desc[i].mask_ack = fpga_ack_irq;
		}
		irq_desc[i].mask = fpga_mask_irq;
		irq_desc[i].unmask = fpga_unmask_irq;
	}

	/* The FPGA interrupt line is connected to GPIO13. Claim this pin for
	 * the ARM.
	 */
	outw((inw(GPIO_PIN_CONTROL_REG) | (1 << 13)), GPIO_PIN_CONTROL_REG);

	/* Configure it as an input pin. */
	outw((inw(GPIO_DIR_CONTROL_REG) | (1 << 13)), GPIO_DIR_CONTROL_REG);

	/* Configure it to fire on low-to-high */
	outw((inw(GPIO_INT_CONTROL_REG) | (1 << 13)), GPIO_INT_CONTROL_REG);

	setup_arm_irq(INT_FPGA, &innovator_fpga_irq);
}

void
innovator_init_irq(void)
{
	omap1510_init_irq();
	fpga_init_irq();
}

static int __init innovator_init(void)
{
	if (!machine_is_innovator())
		return -EINVAL;

 	printk("Innovator FPGA Rev %d.%d Board Rev %d\n",
 	       fpga_read(OMAP1510P1_FPGA_REV_HIGH),
 	       fpga_read(OMAP1510P1_FPGA_REV_LOW),
 	       fpga_read(OMAP1510P1_FPGA_BOARD_REV));

#if	0
	/* REVISIT: Is this req'd? If yes, does it need to happen sooner, e.g.
	 * during innovator_map_io time frame?
	 */
 	fpga_write(OMAP1510P1_FPGA_RESET_VALUE, OMAP1510P1_FPGA_POWER);
#endif

	return 0;
}

__initcall(innovator_init);

static void __init
fixup_innovator(struct machine_desc *desc, struct param_struct *params,
	        char **cmdline, struct meminfo *mi)
{
	struct tag *t = (struct tag *)params;

	if (t->hdr.tag != ATAG_CORE) {
		t->hdr.tag = ATAG_CORE;
		t->hdr.size = tag_size(tag_core);
		t->u.core.flags = 0;
		t->u.core.pagesize = PAGE_SIZE;
		t->u.core.rootdev = RAMDISK_MAJOR << 8 | 0;
		t = tag_next(t);

		t->hdr.tag = ATAG_MEM;
		t->hdr.size = tag_size(tag_mem32);
		t->u.mem.start = PHYS_OFFSET;
		t->u.mem.size  = 32 * 1024 * 1024;
		t = tag_next(t);

#ifdef	CONFIG_BLK_DEV_RAM
		t->hdr.tag = ATAG_RAMDISK;
		t->hdr.size = tag_size(tag_ramdisk);
		t->u.ramdisk.flags = 1;
		t->u.ramdisk.size = CONFIG_BLK_DEV_RAM_SIZE;
		t->u.ramdisk.start = 0;
		t = tag_next(t);
#endif

		t->hdr.tag = ATAG_NONE;
		t->hdr.size = 0;
	}

	return;
}

static void __init
innovator_serial_init(void)
{
#ifdef	CONFIG_SERIAL
	struct serial_struct s;

	memset(&s, 0, sizeof(s));

	s.line = 0;	/* ttyS0 */
	s.type = PORT_OMAP;
	s.xmit_fifo_size = 64;
	s.baud_base = BASE_BAUD;
	s.iomem_base = (u8*)OMAP1510_UART1_BASE;
	s.iomem_reg_shift = 2;
	s.io_type = SERIAL_IO_MEM;
	s.irq = INT_UART1;
	s.flags = STD_COM_FLAGS;
	if (early_serial_setup(&s) != 0) {
		panic("Could not register ttyS0!");
	}
	s.line = 1;	/* ttyS1 */
	s.iomem_base = (u8*)OMAP1510_UART2_BASE;
	s.irq = INT_UART2;
	if (early_serial_setup(&s) != 0) {
		panic("Could not register ttyS1!");
	}
#if !(defined (CONFIG_OMAP_SIR_MODULE))
	s.line = 2;	/* ttyS2 */
	s.iomem_base = (u8*)OMAP1510_UART3_BASE;
	s.irq = INT_UART3;
	if (early_serial_setup(&s) != 0) {
		panic("Could not register ttyS2!");
	}
#endif	
#endif
}

#define MAP_DESC(base,start,size,domain,r,w,c,b) { base,start,size,domain,r,w,c,b }
static struct map_desc innovator_io_desc[] __initdata = {
	MAP_DESC(OMAP1510P1_FPGA_BASE,
		 OMAP1510P1_FPGA_START,
		 OMAP1510P1_FPGA_SIZE,
		 DOMAIN_IO,
		 0,1,0,0),
	LAST_DESC
};

static void __init innovator_map_io(void)
{
	omap1510_map_io();
	iotable_init(innovator_io_desc);

	/* Insert any required early board dependent initialization here.
	 */

	innovator_serial_init();

	return;
}

MACHINE_START(INNOVATOR, "TI-Innovator/OMAP1510")
	MAINTAINER("MontaVista Software, Inc.")
	BOOT_MEM(0x10000000, 0xe0000000, 0xe0000000)
	BOOT_PARAMS(0x10000100)
	FIXUP(fixup_innovator)
	MAPIO(innovator_map_io)
	INITIRQ(innovator_init_irq)
MACHINE_END

EXPORT_SYMBOL(fpga_read);
EXPORT_SYMBOL(fpga_write);
