/*
 *  drivers/mtd/nand/omap730p2nand.c
 *
 *  A driver for the NAND flash device on the OMAP730 P2 reference board.
 *
 *  2004 (c) MontaVista Software, Inc. This file is licensed under
 *  the terms of the GNU General Public License version 2.  This program 
 *  is licensed "as is" without any warranty of any kind, whether express 
 *  or implied.
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#ifdef CONFIG_MACH_OMAP_PERSEUS2
#include <asm/arch/perseus2.h>
#else
#error "This target is not supported by this NAND driver."
#endif

/* Since NAND flash shares the WP signal with NOR flash, we use the set_vpp() 
 * method for NOR flash to manipulate the WP signal.
 */
extern void omap_set_vpp(int vpp);

/* configurable options */
#undef BUSWIDTH_IS_16BITS
#define BUSWIDTH_IS_16BITS

#define NAND_DELAY_US	30
#define NAND_SETCLE	0x2
#define NAND_SETALE	0x4

#define NAND_P2_MTD_NAME	"omap730p2-nand"

/*
 * MTD structure for OMAP730 P2 board
 */
static struct mtd_info *omap730p2_mtd = NULL;

/*
 * Define static partitions for flash devices
 */
static struct mtd_partition partition_info_p2[] = {
	{
		.name =		"NAND X-Loader",
		.size =		0x00010000,
		.offset =	0,
	}, {
		.name =		"NAND U-Boot",
		.size =		0x00040000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"NAND U-Boot Params",
		.size =		0x00030000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"NAND Kernel",
		.size =		0x00100000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"NAND Filesystem",
		.size =		MTDPART_SIZ_FULL,
		.offset =	MTDPART_OFS_APPEND,
	}
};

#define NUM_PARTITIONS ARRAY_SIZE(partition_info_p2)

/*
 * Hardware specific access to control lines.
 * The CLE (Command Latch Enable) pin on the NAND flash is wired to address 
 * line 1, so we set it by OR'ing the address with 2.
 * The ALE (Address Latch Enable) pin on the NAND flash is wired to address 
 * line 2, so we set it by OR'ing the address with 4.
 * The CE (Chip Enable) line is wired to the EMIFS Chip Select, so it is 
 * automaticaly activated on a read or write.
 */
static void omap730p2_hwcontrol(struct mtd_info *mtdinfo, int cmd)
{
	struct nand_chip *this = mtdinfo->priv;

	switch(cmd) {
		case NAND_CTL_SETCLE:
        		this->IO_ADDR_W |=  NAND_SETCLE;
			break;
		case NAND_CTL_CLRCLE:
        		this->IO_ADDR_W &= ~NAND_SETCLE;
			break;
		case NAND_CTL_SETALE:
        		this->IO_ADDR_W |=  NAND_SETALE;
			break;
		case NAND_CTL_CLRALE:
        		this->IO_ADDR_W &= ~NAND_SETALE;
			break;
		case NAND_CTL_SETNCE:
		case NAND_CTL_CLRNCE:
			break;
		default:
			break;
	}
}

/*
 *	read device ready pin
 */
static int omap730p2_device_ready(struct mtd_info *minfo)
{
	return get_p2_nand_rnb_state();
}

static const char *part_probes_p2[] = { "cmdlinepart", NULL };

static struct mtd_partition *mtd_parts;

/*
 * Main initialization routine
 */
int __init omap730p2nand_init (void)
{
	struct nand_chip *this;
	int parsed_nr_parts = 0;
	unsigned long omap730p2_base;
	unsigned long phys_base_addr[2];
	int i;

	/* Allocate memory for MTD device structure and private data */
	omap730p2_mtd = kmalloc(sizeof(struct mtd_info) +
			     sizeof(struct nand_chip),
			     GFP_KERNEL);
	if (!omap730p2_mtd) {
		printk(KERN_WARNING 
			"Unable to allocate NAND MTD device structure.\n");
		return -ENOMEM;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *) (&omap730p2_mtd[1]);

	/* Initialize structures */
	memset((char *) omap730p2_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	omap730p2_mtd->priv = this;

	/* initialize I/O function pointers */
	this->hwcontrol = omap730p2_hwcontrol;
	this->dev_ready = omap730p2_device_ready;

	this->options = NAND_BUSWIDTH_16;
	this->chip_delay = NAND_DELAY_US;

	/* ECC mode */
	this->eccmode = NAND_ECC_SOFT;

	/* There are a couple of possibilities for the base address of the NAND 
	 * flash bank.  We'll always look at CS2 first, which is where NAND 
	 * flash will be if NOR flash is also enabled.  If we don't find NAND 
	 * flash there then we'll look at CS3, which is where NAND flash will be 
	 * if NOR flash is disabled. 
	 */
	phys_base_addr[0] = OMAP730_CS2_START;
	phys_base_addr[1] = OMAP730_CS3_BOOT_CS0_START;

	for (i = 0; i < 2; i++) {
		/* map physical address */
		omap730p2_base = (unsigned long)
			ioremap(phys_base_addr[i], SZ_4K);
		if(!omap730p2_base) {
			printk(KERN_WARNING "ioremap of NAND flash failed\n");
			continue;
		}
		/* set address of NAND flash */
		this->IO_ADDR_R = omap730p2_base;
		this->IO_ADDR_W = omap730p2_base;

		/* Scan to find the device and allocate data_buf and oob_buf */
		printk(KERN_NOTICE "Probing for NAND flash at 0x%08lx\n", 
			phys_base_addr[i]);
		if (nand_scan (omap730p2_mtd, 1)) {
			iounmap((void *)omap730p2_base);
			omap730p2_base = 0;
			continue;
		}
		break;
	}
	/* Return if no NAND flash was found */
	if (!omap730p2_base) {
		kfree(omap730p2_mtd);
		return -ENXIO;
	}

	/* The upper-level MTD NAND routines don't currently provide active 
	 * management of the Write Protect (WP) signal, so we'll just disable 
	 * write-protect for now.
	 */
	omap_set_vpp(1);

	omap730p2_mtd->name = NAND_P2_MTD_NAME;
	parsed_nr_parts = parse_mtd_partitions(omap730p2_mtd, part_probes_p2, 
						&mtd_parts, 0);

	/* Register the partitions */
	if (parsed_nr_parts > 0) 
		add_mtd_partitions(omap730p2_mtd, mtd_parts, parsed_nr_parts);
	else {
		printk(KERN_NOTICE "Using static partition definition\n");
		add_mtd_partitions(omap730p2_mtd, partition_info_p2, 
					NUM_PARTITIONS);
	}

	return 0;
}
module_init(omap730p2nand_init);

/*
 * Clean up routine
 */
void __exit omap730p2nand_cleanup (void)
{
	struct nand_chip *this = (struct nand_chip *) &omap730p2_mtd[1];
	unsigned long omap730p2_base = this->IO_ADDR_R;

	/* release our lock on write-protect */
	omap_set_vpp(0);

	/* Free all of the NAND resources */
	nand_release(omap730p2_mtd);

	/* Release the partition table if it was dynamically allocated */
	if (mtd_parts) {
		kfree(mtd_parts);
		mtd_parts = NULL;
	}

	if (omap730p2_base)
		iounmap((void *) omap730p2_base);

	/* Free the MTD device structure */
	kfree(omap730p2_mtd);
}
module_exit(omap730p2nand_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("source@mvista.com");
MODULE_DESCRIPTION("Glue layer for NAND flash on OMAP730 P2 board");
