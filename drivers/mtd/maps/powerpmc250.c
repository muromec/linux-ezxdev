/*
 *  Mapping for Force PowerPMC250
 *
 *  Copyright 2002 MontaVista Software, Inc.
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
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/system.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/cfi.h>
#include <linux/mtd/partitions.h>

typedef union {
        double dbl;
        __u64 u64;
        __u32 u32[2];
        __u16 u16[4];
        __u8  u8[8];
} cfi_union;

/*
 * This mapping relies on support for Intel and AMD flash chips being
 * enabled.  If you choose to enable the 'Specific CFI Flash geometry
 * selection' option, then the CONFIG_MTD_CFI_B8 and CONFIG_MTD_CFI_I4
 * config options must also be set.  These options enable handling of
 * a 64-bit organized flash bank comprised of a pair of 2 x16
 * interleaved chips.
 */

#define BOOT_FLASH	0xff800000
#define BOOT_FLASH_SIZE	0x00800000

#define USER_FLASH	0x7c000000
#define USER_FLASH_SIZE 0x04000000

__u8 boot_read8(struct map_info *map, unsigned long ofs)
{
       return __raw_readb(map->map_priv_1 + ofs);
}

__u16 boot_read16(struct map_info *map, unsigned long ofs)
{
       return __raw_readw(map->map_priv_1 + ofs);
}

__u32 boot_read32(struct map_info *map, unsigned long ofs)
{
       return __raw_readl(map->map_priv_1 + ofs);
}

void boot_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
       memcpy(to, (void *)(map->map_priv_1  + from), len);
}

void boot_write8(struct map_info *map, __u8 d, unsigned long adr)
{
       __raw_writeb(d, map->map_priv_1 + adr);
       mb();
}

void boot_write16(struct map_info *map, __u16 d, unsigned long adr)
{
       __raw_writew(d, map->map_priv_1 + adr);
       mb();
}

void boot_write32(struct map_info *map, __u32 d, unsigned long adr)
{
       __raw_writel(d, map->map_priv_1 + adr);
       mb();
}

void boot_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
       memcpy((void *)(map->map_priv_1  + to), from, len);
}

/* 64 bit accesses */

__u64 read64(unsigned long addr)
{
	volatile double * d = (double *)addr;
	cfi_union temp;

	temp.dbl = *d;
	mb();
	return(temp.u64);
}

void write64(unsigned long addr, __u64 d)
{
	double *target = (double *)addr;
	volatile cfi_union temp;

	temp.u64 = d;
	
	*target = temp.dbl;
	mb();
}

#define ALIGN 7

static spinlock_t cfi_lock = SPIN_LOCK_UNLOCKED;

#define CFI_LOCK	cfi_union temp;	\
			unsigned long flags; \
			spin_lock_irqsave(&cfi_lock, flags); \
			enable_kernel_fp()

#define CFI_UNLOCK	spin_unlock_irqrestore(&cfi_lock, flags)
		
__u8 user_read8(struct map_info *map, unsigned long ofs)
{
	CFI_LOCK;
	temp.u64 = read64(map->map_priv_1 + (ofs & ~ALIGN));
	CFI_UNLOCK;
	return (temp.u8[ofs & ALIGN]);
}

__u16 user_read16(struct map_info *map, unsigned long ofs)
{	
	CFI_LOCK;
	temp.u64 = read64(map->map_priv_1 + (ofs & ~ALIGN));
	CFI_UNLOCK;
	return (temp.u16[(ofs & ALIGN) >> 1 ]);
}

__u32 user_read32(struct map_info *map, unsigned long ofs)
{
	CFI_LOCK;
	temp.u64 = read64(map->map_priv_1 + (ofs & ~ALIGN));
	CFI_UNLOCK;
	return (temp.u32[(ofs & ALIGN) >> 2 ]);
}

__u64 user_read64(struct map_info *map, unsigned long ofs)
{
	CFI_LOCK;
	temp.u64 = read64(map->map_priv_1 + ofs);
	CFI_UNLOCK;
	return (temp.u64);
}

void user_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	/* long held lock? Oh well. */
	CFI_LOCK;

	while(len) {
		unsigned long i;
		unsigned long extra = len;

		if (len > 8)
			extra = 8;
		temp.u64 = read64(map->map_priv_1 + from);
		for (i = 0; i < extra; i++)
			*(char *)(to + i) = temp.u8[i];
		to += extra;
		from += extra;
		len -= extra;
	}

	CFI_UNLOCK;
       //memcpy(to, (void *)(map->map_priv_1  + from), len);
}

void user_write8(struct map_info *map, __u8 d, unsigned long adr)
{
	panic("user flash 8bit write called!\n");
       //__raw_writeb(d, map->map_priv_1 + adr);
}

void user_write16(struct map_info *map, __u16 d, unsigned long adr)
{
	panic("user flash 16bit write called!\n");
       //__raw_writew(d, map->map_priv_1 + adr);
}

void user_write32(struct map_info *map, __u32 d, unsigned long adr)
{
	panic("user flash 32bit write called!\n");
       //__raw_writel(d, map->map_priv_1 + adr);
}

void user_write64(struct map_info *map, __u64 d, unsigned long adr)
{
	CFI_LOCK;
	write64(map->map_priv_1 + adr, d);
	CFI_UNLOCK;
}

void user_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
	panic("user flash user_copy_to called!\n");
       //memcpy((void *)(map->map_priv_1  + to), from, len);
}

static struct map_info boot_map = {
		name: "PowerPMC250 Boot Flash",
		size: BOOT_FLASH_SIZE,
		buswidth: 1,
		read8: boot_read8,
		read16: boot_read16,
		read32: boot_read32,
		copy_from: boot_copy_from,
		write8: boot_write8,
		write16: boot_write16,
		write32: boot_write32,
		copy_to: boot_copy_to
};

static struct map_info user_map = {
		name: "PowerPMC250 User Flash",
		size: USER_FLASH_SIZE,
		buswidth: 8,
		read8: user_read8,
		read16: user_read16,
		read32: user_read32,
		read64: user_read64,
		copy_from: user_copy_from,
		write8: user_write8,
		write16: user_write16,
		write32: user_write32,
		write64: user_write64,
		copy_to: user_copy_to
};

static struct mtd_partition powerpmc250_flash_partitions[] = {
	{
		name: "PowerPMC PowerBoot Firmware",
		offset: 0,
		size:	0x50000,
		mask_flags: MTD_WRITEABLE 	/* read-only */
	},
	{	
		name: "PowerPMC Boot Kernel",
		offset: 0x50000,
		size:	MTDPART_SIZ_FULL
	}
};

#define NR_OF(x) (sizeof(x)/sizeof(x[0]))

static struct mtd_info *mymtd[2] = {NULL, NULL};

int __init init_powerpmc250(void)
{ 
	printk("%s: %08x at %08x\n", user_map.name,
			(unsigned int)user_map.size, USER_FLASH);
	
	boot_map.map_priv_1 = (unsigned long)ioremap(BOOT_FLASH,
			boot_map.size);

	user_map.map_priv_1 = (unsigned long)ioremap(USER_FLASH,
			user_map.size+8);

	if (!boot_map.map_priv_1) {
		printk("Failed to ioremap %08x\n", BOOT_FLASH);
		return -EIO;
	}
	
	if (!user_map.map_priv_1) {
		printk("Failed to ioremap %08x\n", USER_FLASH);
		return -EIO;
	}

	mymtd[0] = do_map_probe("cfi_probe", &user_map);
	if (mymtd[0]){
		mymtd[0]->module = THIS_MODULE;
		add_mtd_device(mymtd[0]);
	} else {
		iounmap((void *)user_map.map_priv_1);
	}

	mymtd[1] = do_map_probe("cfi_probe", &boot_map);
	if (!mymtd[1])
		mymtd[1] = do_map_probe("jedec", &boot_map);
	if (!mymtd)
		mymtd[1] = do_map_probe("map_rom", &boot_map);
	if (mymtd[1]) {
		mymtd[1]->module = THIS_MODULE;
		add_mtd_partitions(mymtd[1], powerpmc250_flash_partitions,
					NR_OF(powerpmc250_flash_partitions));
	} else {
		iounmap((void *)boot_map.map_priv_1);
	}


				
	if(mymtd[0])
		return 0;
	else
		return -ENXIO;
}

static void __exit cleanup_powerpmc250(void)
{
	int i;
	for (i=0; i<2; i++){
		if (mymtd[i]) {
			del_mtd_partitions(mymtd[i]);
			map_destroy(mymtd[i]);
		}
	}
	if (boot_map.map_priv_1) {
		iounmap((void *)boot_map.map_priv_1);
		boot_map.map_priv_1 = 0;
	}
	if (user_map.map_priv_1) {
		iounmap((void *)user_map.map_priv_1);
		user_map.map_priv_1 = 0;
	}
}

module_init(init_powerpmc250);
module_exit(cleanup_powerpmc250);

MODULE_AUTHOR("Troy Benjegerdes <tbenjegerdes@mvista.com>");
MODULE_DESCRIPTION("MTD map driver for Force PowerPMC250");
MODULE_LICENSE("GPL");
