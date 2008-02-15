/*
 * $Id: xtensa_xt2000.c,v 1.2 2003/01/09 23:19:34 ahennessy Exp $
 *
 * Mapping of the Tensilica XT2000 board with Intel Strata flashes.
 *
 * Based on cstm_mips_ixx.c
 *
 * Copyright 2003 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/config.h>
#include <xtensa/xt2000.h>

#define WINDOW_ADDR XTBOARD_FLASH_VADDR
#define WINDOW_SIZE 0x2000000 
#define WINDOW_BUSWIDTH  4

extern int parse_redboot_partitions(struct mtd_info *master, struct mtd_partition **pparts);

__u8 xtensa_xt2000_read8(struct map_info *map, unsigned long ofs)
{
	return *(__u8 *)(map->map_priv_1 + ofs);
}

__u16 xtensa_xt2000_read16(struct map_info *map, unsigned long ofs)
{
	return *(__u16 *)(map->map_priv_1 + ofs);
}

__u32 xtensa_xt2000_read32(struct map_info *map, unsigned long ofs)
{
	return *(__u32 *)(map->map_priv_1 + ofs);
}

void xtensa_xt2000_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	memcpy_fromio(to, map->map_priv_1 + from, len);
}

void xtensa_xt2000_write8(struct map_info *map, __u8 d, unsigned long adr)
{
	*(__u8 *)(map->map_priv_1 + adr) = d;
}

void xtensa_xt2000_write16(struct map_info *map, __u16 d, unsigned long adr)
{
	*(__u16 *)(map->map_priv_1 + adr) = d;
}

void xtensa_xt2000_write32(struct map_info *map, __u32 d, unsigned long adr)
{
	*(__u32 *)(map->map_priv_1 + adr) = d;
}

void xtensa_xt2000_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
	memcpy_toio(map->map_priv_1 + to, from, len);
}

const struct map_info basic_xtensa_xt2000_map = {
	name:       NULL,
	size:       0,
	buswidth:   0,
	read8:      xtensa_xt2000_read8,
	read16:	    xtensa_xt2000_read16,
	read32:	    xtensa_xt2000_read32,
	copy_from:  xtensa_xt2000_copy_from,
	write8:     xtensa_xt2000_write8,
	write16:    xtensa_xt2000_write16,
	write32:    xtensa_xt2000_write32,
	copy_to:    xtensa_xt2000_copy_to,
	map_priv_1: 0,
	map_priv_2: 0
};

/* board and partition description */

#define MAX_PHYSMAP_PARTITIONS    8
struct xtensa_xt2000_info {
	char *name;
	unsigned long window_addr;
	unsigned long window_size;
	int buswidth;
	int num_partitions;
};

#define PHYSMAP_NUMBER  1  // number of board desc structs needed, one per contiguous flash type 
const struct xtensa_xt2000_info xtensa_xt2000_board_desc[PHYSMAP_NUMBER] = 
{
    {  
        "MTD flash",                   // name
	WINDOW_ADDR,                   // window_addr
	WINDOW_SIZE,                   // window_size
        WINDOW_BUSWIDTH,               // buswidth
	1,                             // num_partitions
    },

};
static struct mtd_partition xtensa_xt2000_partitions[PHYSMAP_NUMBER][MAX_PHYSMAP_PARTITIONS] = {
{
	{
		name: "User FS",
		size:	WINDOW_SIZE,
		offset:	0,
	},
},
};

struct map_info xtensa_xt2000_map[PHYSMAP_NUMBER];

int __init init_xtensa_xt2000(void)
{
	int i;
	int jedec;
        struct mtd_info *mymtd;
        struct mtd_partition *parts;
#ifdef CONFIG_MTD_REDBOOT_PARTS
	int nr_parts = 0;
#endif

	/* Initialize mapping */
	for (i=0;i<PHYSMAP_NUMBER;i++) {
		printk(KERN_NOTICE "xtensa_xt2000 flash device: %lx at %lx\n", xtensa_xt2000_board_desc[i].window_size, xtensa_xt2000_board_desc[i].window_addr);
                memcpy((char *)&xtensa_xt2000_map[i],(char *)&basic_xtensa_xt2000_map,sizeof(struct map_info));
		xtensa_xt2000_map[i].map_priv_1 = (unsigned long)ioremap_nocache(xtensa_xt2000_board_desc[i].window_addr, xtensa_xt2000_board_desc[i].window_size);
		if (!xtensa_xt2000_map[i].map_priv_1) {
			printk(KERN_WARNING "Failed to ioremap\n");
			return -EIO;
	        }
		xtensa_xt2000_map[i].name = xtensa_xt2000_board_desc[i].name;
		xtensa_xt2000_map[i].size = xtensa_xt2000_board_desc[i].window_size;
		xtensa_xt2000_map[i].buswidth = xtensa_xt2000_board_desc[i].buswidth;
		//printk(KERN_NOTICE "xtensa_xt2000: ioremap is %x\n",(unsigned int)(xtensa_xt2000_map[i].map_priv_1));
	}

	for (i=0;i<PHYSMAP_NUMBER;i++) {
                parts = &xtensa_xt2000_partitions[i][0];
		jedec = 0;
		mymtd = (struct mtd_info *)do_map_probe("cfi_probe", &xtensa_xt2000_map[i]);
		if (mymtd) {
			mymtd->module = THIS_MODULE;

	                xtensa_xt2000_map[i].map_priv_2 = (unsigned long)mymtd;
#ifdef CONFIG_MTD_REDBOOT_PARTS
			nr_parts = parse_redboot_partitions(mymtd, &parts);
			if (nr_parts > 0) {
				printk(KERN_NOTICE "Found RedBoot partition table.\n");
				add_mtd_partitions(mymtd, parts, nr_parts);
			}
			else if (nr_parts < 0)
				printk(KERN_NOTICE "Error looking for RedBoot partitions.\n");
#else /* CONFIG_MTD_REDBOOT_PARTS */
		        add_mtd_partitions(mymtd, parts, xtensa_xt2000_board_desc[i].num_partitions);
#endif /* CONFIG_MTD_REDBOOT_PARTS */
		}
		else
	           return -ENXIO;
	}
	return 0;
}

static void __exit cleanup_xtensa_xt2000(void)
{
	int i;
        struct mtd_info *mymtd;

	for (i=0;i<PHYSMAP_NUMBER;i++) {
	        mymtd = (struct mtd_info *)xtensa_xt2000_map[i].map_priv_2;
		if (mymtd) {
			del_mtd_partitions(mymtd);
			map_destroy(mymtd);
		}
		if (xtensa_xt2000_map[i].map_priv_1) {
			iounmap((void *)xtensa_xt2000_map[i].map_priv_1);
			xtensa_xt2000_map[i].map_priv_1 = 0;
		}
	}
}
module_init(init_xtensa_xt2000);
module_exit(cleanup_xtensa_xt2000);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alice Hennessy <ahennessy@mvista.com>");
MODULE_DESCRIPTION("MTD map driver for Tensilica XT2000 boards");
