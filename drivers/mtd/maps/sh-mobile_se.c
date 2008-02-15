/*
 *  sh-mobile_se.c - mapper for SH7300(SH-Mobile) Solution Engine.
 *
 *  Copyright (C) 2003 Takashi Kusuda <kusuda-takashi@hitachi-ul.co.jp>
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/config.h>
#include <linux/errno.h>



__u16
shmse_read16(struct map_info *map, unsigned long ofs)
{
	return *(volatile unsigned short *) (map->phys + ofs);
}
                                                                                                                                                              
void
shmse_write16(struct map_info *map, __u16 d, unsigned long adr)
{
	*(__u16 *) (map->phys + adr) = d;
	mb();
}
                                                                                                                                                              
void
shmse_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	memcpy(to, (void *) (map->phys + from), len);
}

struct map_info shmse_eprom_map;
struct map_info shmse_flash_map = {
	.name = "SH-Mobile SolutionEngine Flash",
#if defined(CONFIG_CPU_SUBTYPE_SH73180)
	.phys = 0xb8000000,
	.size = 0x2000000,
#else
	.phys = 0xa0000000,
	.size = 0x400000,
#endif
	.bankwidth = 2,
	.read = shmse_read16,
	.write = shmse_write16,
	.copy_from = shmse_copy_from,
};

static struct mtd_partition shmse_eprom_partitions[] = {
	{
		.name = "ROM",
		.size = 0x400000,
		.offset = 0
	}
};

static struct mtd_partition shmse_flash_partitions[] = {
	{
		.name = "boot firmware",
		.size = 0x20000,	/* 32KB */
		.offset = 0
	},
	{
		.name = "boot/kernel/filesys",
		.size = MTDPART_SIZ_FULL,
		.offset = MTDPART_OFS_APPEND,
	},
};

static struct mtd_info *shmse_eprom_mtd;
static struct mtd_info *shmse_flash_mtd;

#define N(x) (sizeof(x)/sizeof(*x))

int __init
init_shmse(void)
{
	if (!(shmse_flash_mtd = do_map_probe("cfi_probe", &shmse_flash_map))) {
		shmse_flash_map.phys += 0x00400000;
		shmse_flash_mtd = do_map_probe("cfi_probe", &shmse_flash_map);
	}
	if (shmse_flash_mtd) {
		if (add_mtd_partitions(shmse_flash_mtd, shmse_flash_partitions,
				       N(shmse_flash_partitions)) != 0)
			return -ENXIO;
	} else
		return -ENXIO;

	shmse_eprom_map = shmse_flash_map;
	shmse_eprom_map.name = "SH-Mobile SolutionEngine EPROM";
#if defined(CONFIG_CPU_SUBTYPE_SH73180)
	shmse_eprom_map.phys  = 0xa0000000;
#else
	shmse_eprom_map.phys ^= 0x00400000;
#endif

	shmse_eprom_mtd = do_map_probe("map_rom", &shmse_eprom_map);
	if (shmse_eprom_mtd) {
		if (add_mtd_partitions(shmse_eprom_mtd, shmse_eprom_partitions,
				       N(shmse_eprom_partitions)) != 0)
			return -ENXIO;
		return 0;
	}

	return -ENXIO;
}

static void __exit
cleanup_shmse(void)
{
	if (shmse_flash_mtd) {
		del_mtd_partitions(shmse_flash_mtd);
		map_destroy(shmse_flash_mtd);
	}
	if (shmse_eprom_mtd) {
		del_mtd_partitions(shmse_eprom_mtd);
		map_destroy(shmse_eprom_mtd);
	}
}

MODULE_LICENSE("GPL");
module_init(init_shmse);
module_exit(cleanup_shmse);
