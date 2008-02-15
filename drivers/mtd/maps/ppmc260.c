/*
 *  Copyright © 2002 Force Computers India, Bharath <bharath.c@smartm.com>
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


#define FLASH_PHYS_ADDR 0x50000000
#define FLASH_SIZE 0x4000000  

/*
 * Two partitions have been defined so far:
 *
 * 1. Partition 1 (0x50000000 - 0x507FFFFF)  -- 8.00MB
 * 2. Partition 2 (0x50800000 - 0x50FFFFFF)  -- 24.0MB
 */

#define FLASH_PARTITION0_ADDR 0x00000000
#define FLASH_PARTITION0_SIZE 0x00800000

#define FLASH_PARTITION1_ADDR 0x00800000
#define FLASH_PARTITION1_SIZE 0x01800000

__u8 pmc260_read8(struct map_info *map, unsigned long ofs)
{
	return __raw_readb(map->map_priv_1 + ofs);
}

__u16 pmc260_read16(struct map_info *map, unsigned long ofs)
{
	return __raw_readw(map->map_priv_1 + ofs);
}

__u32 pmc260_read32(struct map_info *map, unsigned long ofs)
{
	return __raw_readl(map->map_priv_1 + ofs);
}

void pmc260_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	memcpy_fromio(to, map->map_priv_1 + from, len);
}

void pmc260_write8(struct map_info *map, __u8 d, unsigned long adr)
{
	__raw_writeb(d, map->map_priv_1 + adr);
	mb();
}

void pmc260_write16(struct map_info *map, __u16 d, unsigned long adr)
{
	__raw_writew(d, map->map_priv_1 + adr);
	mb();
}

void pmc260_write32(struct map_info *map, __u32 d, unsigned long adr)
{
	__raw_writel(d, map->map_priv_1 + adr);
	mb();
}

void pmc260_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
	memcpy_toio(map->map_priv_1 + to, from, len);
}

struct map_info pmc260_map = {
		name: "Intel flash device on PMC260",
		size: FLASH_SIZE,
		buswidth: 4,
		read8: pmc260_read8,
		read16: pmc260_read16,
		read32: pmc260_read32,
		copy_from: pmc260_copy_from,
		write8: pmc260_write8,
		write16: pmc260_write16,
		write32: pmc260_write32,
		copy_to: pmc260_copy_to
};
/*
struct mtd_partition pmc260_parts[] = {
	{
		name	: "Kernel image",
		offset	: FLASH_PARTITION0_ADDR,
		size	: FLASH_PARTITION0_SIZE
	},
	{
		name	: "Initial ramdisk image",
		offset	: FLASH_PARTITION1_ADDR,
		size	: FLASH_PARTITION1_SIZE
	}
};

*/


struct mtd_partition pmc260_parts[] = {
	{
		name	: "Kernel image",
		offset	: 0,
		size	: 0x400000,  
	},{
		name	: "Initial ramdisk image",
		offset	: MTDPART_OFS_APPEND ,
		size	: 0x1800000, 
	},{
		name	: "JFFS2 partition",
		offset	: MTDPART_OFS_APPEND ,
		size	: MTDPART_SIZ_FULL, 
	}
};


#define PARTITION_COUNT (sizeof(pmc260_parts)/sizeof(struct mtd_partition))

static struct mtd_info *mymtd;

int __init init_pmc260(void)
{	
	printk(KERN_NOTICE "Intel flash device on TYNE: %x at %x\n",
			FLASH_SIZE, FLASH_PHYS_ADDR);
	
	pmc260_map.map_priv_1 = (unsigned long)ioremap( FLASH_PHYS_ADDR, FLASH_SIZE ) ;

	//pmc260_map.map_priv_1 = (unsigned long) FLASH_PHYS_ADDR;

	if (!pmc260_map.map_priv_1) {
		printk("Failed to ioremap\n");
		return -EIO;
	}
	printk(KERN_NOTICE "before do_map_probe\n");
	mymtd = do_map_probe("cfi_probe", &pmc260_map);
	printk(KERN_NOTICE "after do_map_probe\n");
	if (mymtd) {
		mymtd->module = THIS_MODULE;
		add_mtd_partitions(mymtd, pmc260_parts, PARTITION_COUNT);
		printk(KERN_NOTICE "TYNE flash device initialized\n");
		return 0;
	}

	iounmap((void *)pmc260_map.map_priv_1);
	return -ENXIO;
}

static void __exit cleanup_pmc260(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
	}
	if (pmc260_map.map_priv_1) {
		iounmap((void *)pmc260_map.map_priv_1);
		pmc260_map.map_priv_1 = 0;
	}
}

module_init(init_pmc260);
module_exit(cleanup_pmc260);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bharath, C. <bharath.c@smartm.com>");
MODULE_DESCRIPTION("MTD map driver for PMC260 board");
