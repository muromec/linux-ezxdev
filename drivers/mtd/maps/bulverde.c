/*
 * $Id:
 *
 * Map driver for the Bulverde developer platform.
 *
 * Copyright:	(C) 2001 MontaVista Software Inc.
 * Copyright    (c) 2005 Motorola Inc.
 *
 * Author:	Echo Engineering (Stolen from Nicolas Pitre's lubbock.c):
 * Modified:	Susan Gu <w15879@motorola.com> 2005/01/15
 *		- Initial map for EzX
 * Modified:	Ru Yi <e5537c@motorola.com> 2005/10/11
 * 		- Change memory layout
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/system.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#define BULVERDE_DEBUGGING 0
#if BULVERDE_DEBUGGING
static unsigned int bulverde_debug = BULVERDE_DEBUGGING;
#else
#define bulverde_debug 0
#endif

#define WINDOW_ADDR 	0
/* total flash memory is 32M, split between two chips */
#define WINDOW_SIZE 	64*1024*1024  //since P2, dataflash size is 64MB
#define WINDOW_CACHE_ADDR 0x02000000   //The starting of the second chip //0x0
#define WINDOW_CACHE_SIZE 0x02000000   //partial language(no use)+ setup partition + rootfs partition //From physical beginning 0x0 to the end of rootfs is 28MB //

/* Added by Susan Gu */
extern struct mtd_info *ezx_mymtd;
unsigned long bulverde_map_cacheable = 0x0;
/* End of modification */

static void bulverde_inval_cache(struct map_info *map, unsigned long from, ssize_t len)
{
	cpu_dcache_invalidate_range(map->map_priv_2 + from, map->map_priv_2 + from + len);
}

/* Modified by Susan */
//static struct map_info bulverde_map = {
struct map_info bulverde_map = {
	.name		= "Bulverde flash",
	.size		= WINDOW_SIZE,
//	.inval_cache	= bulverde_inval_cache,
};

static struct mtd_partition bulverde_partitions[] = {
	{
		name:		"CADDO_SECOND",
		size:		0x00008000,
		offset:		0x00000000,
	},{
		name:		"ITUNES",
		size:		0x00008000,
		offset:		0x00008000,
	},{
		name:		"CADDO_PRIMARY",
		size:		0x00008000,
		offset:		0x00010000,
	},{
		name:		"FOTA_REV",
		size:		0x00008000,
		offset:		0x00018000,
	},{
		name:		"MBM",
		size:		0x00040000,
		offset:		0x00020000,
		mask_flags:	MTD_WRITEABLE  /* force read-only */
	},{
		name:		"CONFIG",
		size:		0x00020000,
		offset:		0x00060000,
	},{
		name:		"BLOB",
		size:		0x00020000,
		offset:		0x00080000, //Logo size is 1 EB
	},{
		name:		"KERNEL",
		size:		0x00100000,
		offset:		0x000A0000,
	},{
		name:		"ROOT",
		size:		0x01720000,
		offset:		0x001A0000, //JFFS2 root size is 23MB
	},{
		name:		"OPT",
		size:		0x02620000,
		offset:		0x019E0000, //JFFS2 opt size is 38MB
	},{
		name:		"TEST_CMD",
		size:		0x00020000,
		offset:		0x018C0000,
	},{
		name:		"LOGO",
		size:		0x00020000,
		offset:		0x018E0000,
	},{
		name:		"FOTA",
		size:		0x000C0000,
		offset:		0x01900000, //FOTA partition is 0xc0000 -- 768KB
	},{
		name:		"RESERVE",
		size:		0x00020000,
		offset:		0x019C0000, //FOTA partition is 0xc0000 -- 768KB
	}
};
#define NB_OF(x)  (sizeof(x)/sizeof(x[0]))

static struct mtd_info *mymtd;
static struct mtd_partition *parsed_parts;

extern int parse_redboot_partitions(struct mtd_info *master, struct mtd_partition **pparts, unsigned long fis_origin);

extern int cfi_intelext_unlockdown(struct mtd_info *mymtd);

static int __init init_bulverde(void)
{
	struct mtd_partition *parts;
	int nb_parts = 0;
	int parsed_nr_parts = 0;
	char *part_type = "static";
	int ret;

	bulverde_map.bankwidth = (BOOT_DEF & 1) ? 2 : 4;

	printk( "Probing Bulverde flash at physical address 0x%08x (%d-bit buswidth)\n",
		WINDOW_ADDR, bulverde_map.bankwidth * 8 );
	bulverde_map.map_priv_1 = (unsigned long)__ioremap(WINDOW_ADDR, WINDOW_SIZE, 0);
	
        bulverde_map.map_priv_2 = NULL;
	if (!bulverde_map.map_priv_1) {

		printk("init_bulverde: Failed to ioremap\n");
		if (bulverde_map.map_priv_1)
			iounmap((void *)bulverde_map.map_priv_1);
		if (bulverde_map.map_priv_2)
			iounmap((void *)bulverde_map.map_priv_2);
		return -EIO;
	}

	if (bulverde_map.map_priv_2 == NULL)
		bulverde_map.inval_cache = NULL;
	else
		bulverde_map.inval_cache = bulverde_inval_cache;

	bulverde_map.virt = bulverde_map.map_priv_1;
	bulverde_map.cached = bulverde_map.map_priv_2;
	mymtd = do_map_probe("cfi_probe", &bulverde_map);
	if (!mymtd) {
		iounmap((void *)bulverde_map.map_priv_1);
		iounmap((void *)bulverde_map.map_priv_2);
		return -ENXIO;
	}
	mymtd->owner = THIS_MODULE;
	ezx_mymtd = mymtd;    //Be used by unlockdown functions //
	
	/* Susan -- unlock/lockdown relative blocks in L18 chip */
	#ifdef CONFIG_INTELFLASH_DEBUG
	printk(KERN_NOTICE "init_bulverde(%d): unlock/lockdown EBs in Intel-Flash-chip\n", current->pid);
	#endif

	ret = cfi_intelext_unlockdown(mymtd);

	if (ret)
	{
#ifdef CONFIG_INTELFLASH_DEBUG
		printk(KERN_NOTICE "cfi_intelext_unlockdown fail\n");
#endif
		iounmap((void *)bulverde_map.map_priv_1);
		iounmap((void *)bulverde_map.map_priv_2);
		return -ENXIO;
	}

	if (WINDOW_CACHE_SIZE)  //There is cacheable memory mapping
	{	
		/* Susan -- ioremap the first flash chip as cacheable( && bufferable? ) */
		bulverde_map_cacheable = (unsigned long)__ioremap(WINDOW_CACHE_ADDR, WINDOW_CACHE_SIZE, L_PTE_CACHEABLE);
		if (!bulverde_map_cacheable) 
		{
			printk("Failed to do cacheable-ioremap which is specific for Ezx product\n");
			iounmap((void *)bulverde_map.map_priv_1);
			iounmap((void *)bulverde_map.map_priv_2);
			return -EIO;
		}
		
	}
	else  // There is not cacheable memory mapping //
		bulverde_map_cacheable = 0xffffffff;

#ifdef CONFIG_MTD_REDBOOT_PARTS
	if (parsed_nr_parts == 0) {
		int ret = parse_redboot_partitions(mymtd, &parsed_parts, 0);

		if (ret > 0) {
			part_type = "RedBoot";
			parsed_nr_parts = ret;
		}
	}
#endif

	if (parsed_nr_parts > 0) {
		parts = parsed_parts;
		nb_parts = parsed_nr_parts;
	} else {
		parts = bulverde_partitions;
		nb_parts = NB_OF(bulverde_partitions);
	}
	if (nb_parts) {
		printk(KERN_NOTICE "Using %s partition definition\n", part_type);
		add_mtd_partitions(mymtd, parts, nb_parts);
	} else {
		add_mtd_device(mymtd);
	}
	
	return 0;
}


static void __exit cleanup_bulverde(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
		if (parsed_parts)
			kfree(parsed_parts);
	}
	if (bulverde_map.map_priv_1)
		iounmap((void *)bulverde_map.map_priv_1);
	if (bulverde_map.map_priv_2)
		iounmap((void *)bulverde_map.map_priv_2);
	return;
}

module_init(init_bulverde);
module_exit(cleanup_bulverde);

