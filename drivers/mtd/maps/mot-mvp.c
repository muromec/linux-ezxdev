/*
 *
 * Mappings for Motorola MVP user and boot flash
 *
 * Author: Troy Benjegerdes <tbenjegerdes@mvista.com>
 *
 * Copyright 2002 MontaVista Software Inc.
 *
 * #include GPLV2 header verbiage
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/config.h>

#include <asm/gt64260.h>
#include <platforms/mvp.h>

static struct mtd_info *bootflash;
static struct mtd_info *userflash;

static __u8 mvp_read8(struct map_info *map, unsigned long ofs)
{
	return __raw_readb(map->map_priv_1 + ofs);
}

static __u16 mvp_read16(struct map_info *map, unsigned long ofs)
{
	return __raw_readw(map->map_priv_1 + ofs);
}

static __u32 mvp_read32(struct map_info *map, unsigned long ofs)
{
	return __raw_readl(map->map_priv_1 + ofs);
}

static void mvp_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	memcpy_fromio(to, map->map_priv_1 + from, len);
}

static void mvp_write8(struct map_info *map, __u8 d, unsigned long adr)
{
	__raw_writeb(d, map->map_priv_1 + adr);
	mb();
}

static void mvp_write16(struct map_info *map, __u16 d, unsigned long adr)
{
	__raw_writew(d, map->map_priv_1 + adr);
	mb();
}

static void mvp_write32(struct map_info *map, __u32 d, unsigned long adr)
{
	__raw_writel(d, map->map_priv_1 + adr);
	mb();
}

static void mvp_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
	memcpy_toio(map->map_priv_1 + to, from, len);
}

static struct map_info mvp_bootflash_map = {
	name: "MVP Boot flash",
	size: MVP_BOOT_FLASH_SIZE,
	buswidth: MVP_BOOT_FLASH_WIDTH,
	read8: mvp_read8,
	read16: mvp_read16,
	read32: mvp_read32,
	copy_from: mvp_copy_from,
	write8: mvp_write8,
	write16: mvp_write16,
	write32: mvp_write32,
	copy_to: mvp_copy_to,
};

static struct map_info mvp_userflash_map = {
	name: "MVP User flash",
	size: MVP_USER_FLASH_SIZE,
	buswidth: MVP_USER_FLASH_WIDTH,
	read8: mvp_read8,
	read16: mvp_read16,
	read32: mvp_read32,
	copy_from: mvp_copy_from,
	write8: mvp_write8,
	write16: mvp_write16,
	write32: mvp_write32,
	copy_to: mvp_copy_to,
};

static struct mtd_partition mvp_bootflash_partitions[] = {
	{
		name: "fs",
		offset: 0x0,
		size: 0xe00000,
	},
	{
		name: "kernel",
		offset: 0xe00000,
		size:   0x100000,
		mask_flags: MTD_WRITEABLE, /* force read-only */
	},
	{
		name: "Dink32",
		offset: 0xf00000,
		size:	0x100000,
		mask_flags: MTD_WRITEABLE, /* force read-only */
	}
};

static struct mtd_partition mvp_userflash_partitions[] = {
	{
		name: "fs",
		offset: 0x0,
		size: 0xe00000,
	},
	{
		name: "kernel",
		offset: 0xe00000,
		size:   0x100000,
	},
	{
		name: "Dink32",
		offset: 0xf00000,
		size:	0x100000,
	}
};

#define NB_OF(x)  (sizeof(x)/sizeof(x[0]))

int __init init_mvp(void)
{

	printk(KERN_NOTICE "userflash: 0x%lx at 0x%x\n",
				mvp_userflash_map.size, MVP_USER_FLASH_BASE);

	mvp_userflash_map.map_priv_1 = (unsigned long)ioremap(
				MVP_USER_FLASH_BASE, mvp_userflash_map.size);
	if (!mvp_userflash_map.map_priv_1) {
		printk("Failed to ioremap userflash\n");
		return -EIO;
	}

	userflash = do_map_probe("cfi_probe", &mvp_userflash_map);
	if (userflash) {
		userflash->module = THIS_MODULE;
		add_mtd_partitions(userflash, mvp_userflash_partitions,
					NB_OF(mvp_userflash_partitions));
	} else {
		printk("map probe failed for userflash\n");
		return -ENXIO;
	}

	printk(KERN_NOTICE " bootflash: 0x%lx at 0x%x\n",
				mvp_bootflash_map.size, MVP_BOOT_FLASH_BASE);

	mvp_bootflash_map.map_priv_1 = (unsigned long)ioremap(
				MVP_BOOT_FLASH_BASE, mvp_bootflash_map.size);
	if (!mvp_bootflash_map.map_priv_1) {
		printk("Failed to ioremap bootflash\n");
		iounmap((void *)mvp_userflash_map.map_priv_1);
		return -EIO;
	}

	bootflash = do_map_probe("cfi_probe", &mvp_bootflash_map);
	if (bootflash) {
		bootflash->module = THIS_MODULE;
		add_mtd_partitions(bootflash, mvp_bootflash_partitions,
					NB_OF(mvp_bootflash_partitions));
					
	} else {
		printk("map probe failed for bootflash\n");
	}
	return 0;
}

static void __exit cleanup_mvp(void)
{
	if (userflash) {
		del_mtd_partitions(userflash);
		map_destroy(userflash);
	}
	if (mvp_userflash_map.map_priv_1) {
		iounmap((void *)mvp_userflash_map.map_priv_1);
		mvp_userflash_map.map_priv_1 = 0;
	}
	if (bootflash) {
		del_mtd_partitions(bootflash);
		map_destroy(bootflash);
	}
	if (mvp_bootflash_map.map_priv_1) {
		iounmap((void *)mvp_bootflash_map.map_priv_1);
		mvp_bootflash_map.map_priv_1 = 0;
	}
}

module_init(init_mvp);
module_exit(cleanup_mvp);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Motorola MVP flash map");
