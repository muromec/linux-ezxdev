/*
 * linux/arch/arm/mach-omap/innovator.c
 *
 * This file contains Innovator-specific code.
 *
 * Copyright (C) 2002, 2004 MontaVista Software, Inc.
 *  <source@mvista.com>
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
#include <asm/arch/hardware.h>
#include <asm/setup.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/arch/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/arch/ck.h>
#include <asm/arch/irqs.h>

static void
innovator_init_irq(void)
{
	omap1610_init_irq();
}

static void __init
fixup_innovator(struct machine_desc *desc, struct param_struct *params,
	        char **cmdline, struct meminfo *mi)
{
	struct tag *t = (struct tag *)params;

	setup_initrd(__phys_to_virt(0x10800000), 4 * 1024 * 1024);
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
	s.iomem_base = (u8*)OMAP1610_UART1_BASE;
	s.iomem_reg_shift = 2;
	s.io_type = SERIAL_IO_MEM;
	s.irq = INT_UART1;
	s.flags = STD_COM_FLAGS;
	if (early_serial_setup(&s) != 0) {
		panic("Could not register ttyS0!");
	}
	s.line = 1;	/* ttyS1 */
	s.iomem_base = (u8*)OMAP1610_UART2_BASE;
	s.irq = INT_UART2;
	if (early_serial_setup(&s) != 0) {
		panic("Could not register ttyS1!");
	}
#ifndef CONFIG_OMAP1610_IR
#ifndef CONFIG_OMAP1610_IR_MODULE
	s.line = 2;	/* ttyS2 */
	s.iomem_base = (u8*)OMAP1610_UART3_BASE;
	s.irq = INT_UART3;
	if (early_serial_setup(&s) != 0) {
		panic("Could not register ttyS2!");
	}
	outl(inl(MOD_CONF_CTRL_0) | (1 << 31), MOD_CONF_CTRL_0); /* enable clock for ttyS2 */
#endif	
#endif
#endif
}

static int __init etherbus_init(void)
{
	/* Set Ehernet chip access timings to fastest correct values       
	   assuming Initial frequency (TC_CK) is <=50 MHz */
	writel(0x83002143, EMIFS_CS1_CONFIG); 
	writel(0x212, EMIFS_CS1_CONFIG2);     
	return 0;
}

__initcall(etherbus_init);

#define MAP_DESC(base,start,size,domain,r,w,c,b) { base,start,size,domain,r,w,c,b }
static struct map_desc innovator_io_desc[] __initdata = {
	MAP_DESC(OMAP_ETHR_BASE,
		 OMAP_ETHR_START,
		 OMAP_ETHR_SIZE,
		 DOMAIN_IO,
		 0,1,0,0),
#ifdef CONFIG_OMAP_H2
	MAP_DESC(FPGA_BASE,
		 FPGA_START,
		 FPGA_SIZE,
		 DOMAIN_IO,
		 0,1,0,0),
#endif
	LAST_DESC
};

static void __init innovator_map_io(void)
{
	omap1610_map_io();
	iotable_init(innovator_io_desc);
	innovator_serial_init();
}

MACHINE_START(INNOVATOR, "TI-Innovator/OMAP1610")
	MAINTAINER("MontaVista Software, Inc.")
	BOOT_MEM(0x10000000, 0xe0000000, 0xe0000000)
	BOOT_PARAMS(0x10000100)
	FIXUP(fixup_innovator)
	MAPIO(innovator_map_io)
	INITIRQ(innovator_init_irq)
MACHINE_END
