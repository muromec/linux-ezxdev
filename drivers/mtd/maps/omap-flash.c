/*
 * Flash memory access on OMAP based devices
 *
 * Copyright: (C) 2002 MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * Flash unlock procedure taken from:
 * Flash memory access on EPXA based devices
 *
 * (C) 2000 Nicolas Pitre <nico@cam.org>
 *  Copyright (C) 2001 Altera Corporation
 *  Copyright (C) 2001 Red Hat, Inc.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/hardware.h>
#include <asm/io.h>


#ifndef CONFIG_ARCH_OMAP
#error This is for OMAP architecture only
#endif

#define OMAP_CS0_START 0x00000000
#define OMAP_CS3_START 0x0C000000

static void omap_inval_cache(struct map_info *map, unsigned long from, ssize_t len)
{
	cpu_dcache_invalidate_range(map->map_priv_2 + from,
				    map->map_priv_2 + from + len);
}

static void omap_unlock_all(struct mtd_info * mtd)
{
	int i, j;

	/* Unlock the flash device. */
	if(mtd->unlock){
		for (i = 0; i < mtd->numeraseregions; i++) {
			for (j = 0; j < mtd->eraseregions[i].numblocks; j++){
				mtd->unlock(mtd, mtd->eraseregions[i].offset +
							  j * mtd->eraseregions[i].erasesize,
							  mtd->eraseregions[i].erasesize);
			}
		}
	}
}

extern void omap_set_vpp(int vpp);
extern int  omap_boot_from_cs0(void);

void mtd_omap_set_vpp(struct map_info *map, int vpp)
{
	omap_set_vpp(vpp);
}


#if	defined(CONFIG_MTD_OMAP_0)
static struct map_info omap_map_flash_0 = {
	.name		= "OMAP flash",
	.set_vpp	= mtd_omap_set_vpp,
	.bankwidth	= 2,
	.inval_cache	= omap_inval_cache,
};

static struct mtd_partition *parsed_parts0;
static int nb_parts0;
static struct mtd_info *flash0_mtd;
#endif

#if	defined(CONFIG_MTD_OMAP_1)
static struct map_info omap_map_flash_1 = {
	.name		= "OMAP flash",
	.set_vpp	= mtd_omap_set_vpp,
	.bankwidth	= 2,
	.inval_cache	= omap_inval_cache,
};

static struct mtd_partition *parsed_parts1;
static int nb_parts1;
static struct mtd_info *flash1_mtd;
#endif

/*
 * Here are partition information for all known OMAP-based devices.
 * See include/linux/mtd/partitions.h for definition of the mtd_partition
 * structure.
 *
 * The *_max_flash_size is the maximum possible mapped flash size which
 * is not necessarily the actual flash size.  It must be no more than
 * the value specified in the "struct map_desc *_io_desc" mapping
 * definition for the corresponding machine.
 *
 * Please keep these in alphabetical order, and formatted as per existing
 * entries.  Thanks.
 */

#if	defined(CONFIG_MACH_OMAP_PERSEUS2)
static unsigned long omap_perseus2_max_flash_size = OMAP_FLASH_0_SIZE;
static struct mtd_partition omap_perseus2_partitions[] = {
	{
		.name =		"BootLoader and Params",
		.size =		0x00100000,
		.offset =	0,
		.mask_flags =	MTD_WRITEABLE,	/* force read-only */
	}, {
		.name =		"Kernel",
		.size =		0x00100000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"MainFS",
		.size =		0x01BE0000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"GsmFFS",
		.size =		0x00020000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"GSM",
		.size =		MTDPART_SIZ_FULL,
		.offset =	MTDPART_OFS_APPEND,
	}
};
#endif

#if	defined(CONFIG_OMAP_INNOVATOR)
/* Both Flash 0 and 1 are the same size */
static unsigned long innovator_max_flash_size = OMAP_FLASH_0_SIZE;
#if	defined(CONFIG_MTD_OMAP_0)
static struct mtd_partition innovator_flash_0_partitions[] = {
	{
		name:		"BootLoader",
		size:		0x00020000,
		offset:		0,
		mask_flags:	MTD_WRITEABLE,  /* force read-only */
	}, {
		name:		"Params",
		size:		0x00040000,
		offset:		MTDPART_OFS_APPEND,
		mask_flags:	MTD_WRITEABLE,  /* force read-only */
	}, {
		name:		"Kernel",
		size:		0x00200000,
		offset:		MTDPART_OFS_APPEND,
	}, {
		name:		"Flash0 FileSys",
		size:		MTDPART_SIZ_FULL,
		offset:		MTDPART_OFS_APPEND,
	}
};
#endif

#if	defined(CONFIG_MTD_OMAP_1)
static struct mtd_partition innovator_flash_1_partitions[] = {
	{
		name:		"Flash1 FileSys",
		size:		MTDPART_SIZ_FULL,
		offset:		MTDPART_OFS_APPEND,
	}
};
#endif
#endif

#if	defined(CONFIG_OMAP_PERSEUS)
static unsigned long perseus_max_flash_size = OMAP_FLASH_CS0_SIZE;
static struct mtd_partition perseus_partitions[] = {
	{
		name:		"Bootloader",
		size:		0x00100000,
		offset:		0,
		mask_flags:	MTD_WRITEABLE,  /* force read-only */
	}, {
		name:		"zImage",
		size:		0x000a0000,
		offset:		MTDPART_OFS_APPEND,
	}, {
		name:		"Root FS",
		size:		0x00660000,
		offset:		MTDPART_OFS_APPEND,
	}, {
		name:		"User1 FS",
		size:		0x00400000,
		offset:		MTDPART_OFS_APPEND,
	}, {
		name:		"User2 FS",
		size:		MTDPART_SIZ_FULL,
		offset:		MTDPART_OFS_APPEND,
	}
};
#endif

static const char *part_probes[] __initdata = {"cmdlinepart", "RedBoot", NULL};

#ifdef CONFIG_MTD_OMAP_0
static int __init init_flash_0 (void)
{

	struct mtd_partition *parts;
	int parsed_nr_parts = 0;

	/*
	 * Static partition definition selection
	 */
	nb_parts0 =0;
#if	defined(CONFIG_MACH_OMAP_PERSEUS2)
	if (machine_is_omap_perseus2()) {
		parts = omap_perseus2_partitions;
		nb_parts0 = ARRAY_SIZE(omap_perseus2_partitions);
		omap_map_flash_0.size = omap_perseus2_max_flash_size;
	}
#endif
#if	defined(CONFIG_OMAP_INNOVATOR)
	if (machine_is_innovator()) {
		parts = innovator_flash_0_partitions;
		nb_parts0 = ARRAY_SIZE(innovator_flash_0_partitions);
		omap_map_flash_0.size = innovator_max_flash_size;
	}
#endif
#if	defined(CONFIG_OMAP_PERSEUS)
	if (machine_is_perseus()) {
		parts = perseus_partitions;
		nb_parts0 = ARRAY_SIZE(perseus_partitions);
		omap_map_flash_0.size = perseus_max_flash_size;
	}
#endif

	/*
	 * Now let's probe for the actual flash.  Do it here since
	 * specific machine settings might have been set above.
	 */
	printk(KERN_NOTICE "OMAP flash 0: probing %d-bit flash bus\n",
		omap_map_flash_0.bankwidth*8);
	if (omap_boot_from_cs0) {
		omap_map_flash_0.map_priv_1 = (unsigned long)__ioremap(OMAP_CS0_START, OMAP_FLASH_0_SIZE, 0);
		omap_map_flash_0.map_priv_2 = (unsigned long)__ioremap(OMAP_CS0_START, OMAP_FLASH_0_SIZE, L_PTE_CACHEABLE);
	} else {
		omap_map_flash_0.map_priv_1 = (unsigned long)__ioremap(OMAP_CS3_START, OMAP_FLASH_0_SIZE, 0);
		omap_map_flash_0.map_priv_2 = (unsigned long)__ioremap(OMAP_CS3_START, OMAP_FLASH_0_SIZE, L_PTE_CACHEABLE);
	}
	if (!omap_map_flash_0.map_priv_1 || !omap_map_flash_0.map_priv_2) {
		printk("Failed to ioremap flash 0\n");
		if (omap_map_flash_0.map_priv_1)
			iounmap((void *)omap_map_flash_0.map_priv_1);
		if (omap_map_flash_0.map_priv_2)
			iounmap((void *)omap_map_flash_0.map_priv_2);
		return -EIO;
	}

	omap_map_flash_0.virt = omap_map_flash_0.map_priv_1;
	omap_map_flash_0.cached = (void *)omap_map_flash_0.map_priv_2;

	flash0_mtd = do_map_probe("cfi_probe", &omap_map_flash_0);
	if (!flash0_mtd) {
		if (omap_map_flash_0.map_priv_1)
			iounmap((void *)omap_map_flash_0.map_priv_1);
		if (omap_map_flash_0.map_priv_2)
			iounmap((void *)omap_map_flash_0.map_priv_2);
		return -ENXIO;
	}
	flash0_mtd->owner = THIS_MODULE;
#if defined (CONFIG_ARCH_OMAP1610) || defined (CONFIG_ARCH_OMAP730)
	omap_unlock_all(flash0_mtd);
#endif

	/*
	 * Dynamic partition selection stuff (might override the static ones)
	 */
	parsed_nr_parts =
		parse_mtd_partitions(flash0_mtd, part_probes, &parsed_parts0, 0);

	if (parsed_nr_parts > 0) {
		nb_parts0 = parsed_nr_parts;
		add_mtd_partitions(flash0_mtd, parsed_parts0, parsed_nr_parts);
	} else if (nb_parts0 == 0) {
		printk(KERN_NOTICE "OMAP flash: no partition info available,"
			"registering whole flash at once\n");
		add_mtd_device(flash0_mtd);
	} else {
		printk(KERN_NOTICE "Using static partition definition\n");
		add_mtd_partitions(flash0_mtd, parts, nb_parts0);
	}
	return 0;
}
#endif

#if	defined(CONFIG_MTD_OMAP_1)
static int __init init_flash_1 (void)
{

	struct mtd_partition *parts;
	int parsed_nr_parts = 0;

	/*
	 * Static partition definition selection
	 */
	nb_parts1 = 0;
#if	defined(CONFIG_OMAP_INNOVATOR)
	if (machine_is_innovator()) {
		parts = innovator_flash_1_partitions;
		nb_parts1 = ARRAY_SIZE(innovator_flash_1_partitions);
		omap_map_flash_1.size = innovator_max_flash_size;
	}
#endif

	/*
	 * Now let's probe for the actual flash.  Do it here since
	 * specific machine settings might have been set above.
	 */
	printk(KERN_NOTICE "OMAP flash 1: probing %d-bit flash bus\n",
		omap_map_flash_1.bankwidth*8);
	omap_map_flash_1.map_priv_1 = (unsigned long)__ioremap(OMAP_FLASH_1_START, OMAP_FLASH_1_SIZE, 0);
	omap_map_flash_1.map_priv_2 = (unsigned long)__ioremap(OMAP_FLASH_1_START, OMAP_FLASH_1_SIZE, L_PTE_CACHEABLE);
	if (!omap_map_flash_1.map_priv_1 || !omap_map_flash_1.map_priv_2) {
		printk("Failed to ioremap flash 1\n");
		if (omap_map_flash_1.map_priv_1)
			iounmap((void *)omap_map_flash_1.map_priv_1);
		if (omap_map_flash_1.map_priv_2)
			iounmap((void *)omap_map_flash_1.map_priv_2);
		return -EIO;
	}

	omap_map_flash_1.virt = omap_map_flash_1.map_priv_1;
	omap_map_flash_1.cached = (void *)omap_map_flash_1.map_priv_2;

	flash1_mtd = do_map_probe("cfi_probe", &omap_map_flash_1);
	if (!flash1_mtd) {
		if (omap_map_flash_1.map_priv_1)
			iounmap((void *)omap_map_flash_1.map_priv_1);
		if (omap_map_flash_1.map_priv_2)
			iounmap((void *)omap_map_flash_1.map_priv_2);
		return -ENXIO;
	}
	flash1_mtd->owner = THIS_MODULE;
#if defined (CONFIG_ARCH_OMAP1610) || defined (CONFIG_ARCH_OMAP730)
	omap_unlock_all(flash1_mtd);
#endif

	/*
	 * Dynamic partition selection stuff (might override the static ones)
	 */
	parsed_nr_parts = 
		parse_mtd_partitions(flash1_mtd, part_probes, &parsed_parts1, 0);

	if (parsed_nr_parts > 0) {
		nb_parts1 = parsed_nr_parts;
		add_mtd_partitions(flash1_mtd, parsed_parts1, parsed_nr_parts);
	} else if (nb_parts1 == 0) {
		printk(KERN_NOTICE "OMAP flash 1: no partition info available,"
			"registering whole flash at once\n");
		add_mtd_device(flash1_mtd);
	} else {
		printk(KERN_NOTICE "Using static partition definition\n");
		add_mtd_partitions(flash1_mtd, parts, nb_parts1);
	}
	return 0;
}
#endif

int __init omap_mtd_init(void)
{
	int status;

#ifdef CONFIG_MTD_OMAP_0
	if ((status = init_flash_0 ())) {
		printk(KERN_ERR "OMAP Flash 0: unable to init map\n");
	}
#endif
#ifdef CONFIG_MTD_OMAP_1
	if ((status = init_flash_1 ())) {
		printk(KERN_ERR "OMAP Flash 1: unable to init map\n");
	}
#endif
	return status;
}

static void __exit omap_mtd_cleanup(void)
{
#ifdef CONFIG_MTD_OMAP_0
	if (flash0_mtd) {
		if (nb_parts0)
			del_mtd_partitions(flash0_mtd);
		else
			del_mtd_device(flash0_mtd);
		if (parsed_parts0) {
			kfree(parsed_parts0);
			parsed_parts0 = NULL;
		}
		map_destroy(flash0_mtd);
		flash0_mtd = NULL;

		if (omap_map_flash_0.map_priv_1)
			iounmap((void *)omap_map_flash_0.map_priv_1);
		if (omap_map_flash_0.map_priv_2)
			iounmap((void *)omap_map_flash_0.map_priv_2);
	}
#endif
#ifdef CONFIG_MTD_OMAP_1
	if (flash1_mtd) {
		if (nb_parts1)
			del_mtd_partitions(flash1_mtd);
		else
			del_mtd_device(flash1_mtd);
		if (parsed_parts1) {
			kfree(parsed_parts1);
			parsed_parts1 = NULL;
		}
		map_destroy(flash1_mtd);
		flash1_mtd = NULL;

		if (omap_map_flash_1.map_priv_1)
			iounmap((void *)omap_map_flash_1.map_priv_1);
		if (omap_map_flash_1.map_priv_2)
			iounmap((void *)omap_map_flash_1.map_priv_2);
	}
#endif
}

module_init(omap_mtd_init);
module_exit(omap_mtd_cleanup);

MODULE_AUTHOR("George G. Davis");
MODULE_DESCRIPTION("OMAP CFI map driver");
MODULE_LICENSE("GPL");
