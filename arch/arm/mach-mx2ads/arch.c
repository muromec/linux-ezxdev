/*
 *  linux/arch/arm/mach-mx1ads/arch.c
 *
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
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
#include <linux/config.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/mtd/physmap.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/arch/pll.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/amba_kmi.h>

extern void mx2ads_map_io(void);
extern void mx2ads_init_irq(void);

#if defined(CONFIG_MTD_PHYSMAP) && defined(CONFIG_MTD_PARTITIONS) && defined(CONFIG_MTD)

static struct mtd_partition mx21ads_mtd_parts[] __initdata = {
	{
		.name           = "Bootloader",
		.size           = 0x00100000,
		.offset         = 0,
		.mask_flags     = MTD_WRITEABLE
	}, {
		.name           = "Kernel",
		.size           = 0x00200000,
		.offset         = MTDPART_OFS_APPEND
	}, {
		.name           = "UserFS",
		.size           = MTDPART_SIZ_FULL,
		.offset         = MTDPART_OFS_APPEND
	}
};

#define mx21ads_num_parts	\
	(sizeof(mx21ads_mtd_parts)/sizeof(struct mtd_partition))

#endif

static int __init
mx21ads_init(void)
{
#if defined(CONFIG_MTD_PHYSMAP) && defined(CONFIG_MTD_PARTITIONS) && defined(CONFIG_MTD)
	/* Use generic physmap driver and
	 * define default static partitions
	 */
	physmap_set_partitions(mx21ads_mtd_parts, mx21ads_num_parts);
#endif
	return 0;
}

__initcall(mx21ads_init);

static void __init
mx2ads_fixup(struct machine_desc *desc, struct param_struct *unused,
		 char **cmdline, struct meminfo *mi)
{
}

MACHINE_START(MX2ADS, "Motorola MX2ADS")
	MAINTAINER("WBSG SPS Motorola SuZhou")
/* 
Now our MX2 physical address is from 0xc0000000, io register is from 0x10000000, 
we map it to 0xe0000000. jimmy
*/

	BOOT_MEM(0xc0000000, 0x10000000, 0xE4000000)
	BOOT_PARAMS(0xc0000100)
	FIXUP(mx2ads_fixup)
	MAPIO(mx2ads_map_io)
	INITIRQ(mx2ads_init_irq)
MACHINE_END
