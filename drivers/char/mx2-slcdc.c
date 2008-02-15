/******************************************************************************
	mx2-slcdc.c

	This is the basic release for SLCDC driver,
	which is to support EPSON L2F50032T00 16bit seria mode5-6-5
	This driver acts as a standard character device,
	which can be dinamically loaded.

	Copyright (C) 2003 Motorola Inc Ltd

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
	
	First version 12-15-2003, Karen Kang

********************************************************************************/
#undef SLCDC_MUX_SDHC2
#define SLCDC_MUX_LCDC

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/wrapper.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/tqueue.h>
#include <linux/wait.h>
#include <linux/pm.h>

#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/uaccess.h>	/* get_user,copy_to_user */
#include <asm/irq.h>
#include <asm/arch/gpio.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/arch/pll.h>

static int slcdc_suspended;
static int slcdc_busy;
static char slcdc_irq_inited, slcdc_gpio1_inited, slcdc_gpio2_inited, slcdc_display_on_true;

#if 1 /*CEE LDM*/
#include <linux/device.h>
#include <linux/dpm.h>

static void slcdc_set_clk(void);

extern void mx21_ldm_bus_register(struct device *device,
                          struct device_driver *driver);
extern void mx21_ldm_bus_unregister(struct device *device,
                          struct device_driver *driver);
static int
__ldm_suspend(struct device *dev, u32 state, u32 level);

static int
__ldm_resume(struct device *dev, u32 level);

static int
slcd_freq_scale(struct bus_op_point * op, u32 level)
{
	if (slcdc_busy) {
		slcdc_set_clk();
	}
	return 0;
}

static struct device_driver __driver_ldm = {
	.name = "mx2-slcdc",
	.devclass = NULL,
	.probe = NULL,
	.suspend = __ldm_suspend,
	.resume = __ldm_resume,
	.scale = slcd_freq_scale,
};

static struct device __device_ldm = {
	.name = "SLCD Controller",
	.bus_id = "slcdc",
	.driver = &__driver_ldm,
	.power_state = DPM_POWER_ON,
};

#endif


#define MODULE_NAME "SLCD"

#define SLCDC_IMG_8L	0x00020000

/* definition for SLCDC cmd */
/* same as Epson 10043 & 50052 */

#define SLCDC_CMD_DISON		0xaf00	/* 1. display on */
#define SLCDC_CMD_DISOFF	0xae00	/* 2. display off */
#define SLCDC_CMD_DISNOR	0xa600	/* 3. normal display */
#define SLCDC_CMD_DISINV	0xa700	/* 4. inverse display */
#define SLCDC_CMD_DISCTL	0xca00	/* 5. display control */
#define SLCDC_CMD_SLPIN		0x9500	/* 10.sleep in */
#define SLCDC_CMD_SLPOUT	0x9400	/* 11.sleep out */
#define SLCDC_CMD_SD_PSET	0x7500	/* 12.page address set */
#define SLCDC_CMD_SD_CSET	0x1500	/* 14.column address set */

#define SLCDC_CMD_DATCTL	0xbc	/* 16.data scan direction, etc. */
//#define SLCDC_CMD_DATCTL              0xbc00  /* 16.data scan direction, etc. */
#define SLCDC_CMD_RAMWR		0x5c00	/* 17.writing to memory */
#define SLCDC_CMD_PTLIN		0xa800	/* 19.partial display in */
#define SLCDC_CMD_PTLOUT	0xa900	/* 20.partial display out */

/* value different from 10043 but same as 50052 */
#define SLCDC_CMD_VOLCTR	0xc600	/* 25.Electronic volume control */

/* commands not found in 10043 but in 50052 */
#define SLCDC_CMD_GCP64		0xcb00	/* 6. 64 grayscale pulse positon set */
#define SLCDC_CMD_GCP16		0xcc00	/* 7. 16 grayscale pulse positon set */
#define SLCDC_CMD_GSSET		0xcd00	/* 8. grayscale set */
#define SLCDC_CMD_RAMRD		0x5d00	/* 18.memory read */
#define SLCDC_CMD_ASCSET	0xaa00	/* 21.area scroll set */
#define SLCDC_CMD_SCSTART	0xab00	/* 22.scroll start set */
#define SLCDC_CMD_EPCTIN	0x6100	/* 26.Power IC control for EVR */
#define SLCDC_CMD_EPCTOUT	0x6200	/* 27.Power IC control for EVR */

#define SLCDC_IRQ	60
#define SLCDC_CMD_MEM_SIZE		4
#define SLCDC_WIDTH				240
#define SLCDC_HIGH				320
#define SLCDC_BPP				2     /*Bytes per pixel*/
#define SLCDC_PIXEL_MEM_SIZE	(SLCDC_WIDTH*SLCDC_HIGH*SLCDC_BPP)
#define _SLCDC_DATA_SIZE_	(SLCDC_PIXEL_MEM_SIZE + 32)
#define SLCDC_DATA_MEM_SIZE	(PAGE_ALIGN(_SLCDC_DATA_SIZE_ + PAGE_SIZE * 2))

#define SLCDC_TRANSFER_BUSY		0x4
#define SLCDC_TRANSFER_ERROR	0x10

static u16 *g_slcdc_dbuffer_address;	  /* used for SLCDC data buffer */
static u16 *g_slcdc_dbuffer_phyaddress;  /* physical address for SLCDC data buffer */
static u16 *g_slcdc_cbuffer_address;	  /* used for SLCDC command buffer */
static u16 *g_slcdc_cbuffer_phyaddress;  /* physical address for SLCDC command buffer */

static int g_slcdc_major = 0;
static devfs_handle_t g_devfs_handle;
DECLARE_COMPLETION(slcdc_wait);

static struct pm_dev *g_slcdc_pm;

static int slcdc_open(struct inode *inode, struct file *filp);
static int slcdc_release(struct inode *inode, struct file *filp);
static int slcdc_ioctl(struct inode *inode, struct file *filp, u_int cmd,
		       u_long arg);
static int slcdc_mmap(struct file *filp, struct vm_area_struct *vma);
static void slcdc_display_ramwr(void);

typedef struct {
	u16 *screen_start_address;
	u16 *v_screen_start_address;
} slcdc_par_t;

static slcdc_par_t slcdc_par;

struct file_operations g_slcdc_fops = {
	open:slcdc_open,
	release:slcdc_release,
	ioctl:slcdc_ioctl,
	mmap:slcdc_mmap,
};

#define USING_INTERRUPT_SLCDC

static inline int
slcdc_gpio_init(void)
{
int tmp;

#ifdef SLCDC_MUX_LCDC

	/* configure slcdc gpio setting using pa24~27 (AIN), muxed with LCD panel */
	tmp = mx2_register_gpios(PORT_A, 1 << 24 | 1 << 25 | 1 << 26 | 1 << 27,
			   GPIO | OUTPUT);
	if (tmp < 0) {
		printk(KERN_ERR MODULE_NAME ": failed to register GPIO PORT A 24..27\n");
		return tmp;
	}
	slcdc_gpio1_inited = 1;

	/* OE_ACD as a reset pin */
	tmp = mx2_register_gpios(PORT_A, 1 << 31, GPIO | OUTPUT | OCR_DATA);
	if (tmp < 0) {
		printk(KERN_ERR MODULE_NAME ": failed to register GPIO PORT A 31\n");
		return tmp;
	}
	slcdc_gpio2_inited = 1;
#endif

#ifdef SLCDC_MUX_SDHC2

	/* configure slcdc gpio setting using pb6~9(AIN), muxed with SDHC2 */
	tmp = mx2_register_gpios(PORT_B, 1 << 6 | 1 << 7 | 1 << 8 | 1 << 9,
			   GPIO | OUTPUT);
	if (tmp < 0) {
		printk(KERN_ERR MODULE_NAME ": failed to register GPIO PORT B 6..9\n");
		return tmp;
	}
	slcdc_gpio1_inited = 1;

	/* SD2_D1 as reset pin */
	tmp = mx2_register_gpios(PORT_B, 1 << 5, GPIO | OUTPUT | OCR_DATA);

	if (tmp < 0) {
		printk(KERN_ERR MODULE_NAME ": failed to register GPIO PORT B 5\n");
		return tmp;
	}
	slcdc_gpio2_inited = 1;
#endif
	return 0;
}

static inline void
slcdc_gpio_uninit(void)
{

#ifdef SLCDC_MUX_LCDC
	if (slcdc_gpio1_inited)
		mx2_unregister_gpios(PORT_A, 1 << 24 | 1 << 25 | 1 << 26 | 1 << 27);
	if (slcdc_gpio2_inited)
		mx2_unregister_gpios(PORT_A, 1 << 31);
#endif

#ifdef SLCDC_MUX_SDHC2
	if (slcdc_gpio1_inited)
		mx2_unregister_gpios(PORT_B, 1 << 6 | 1 << 7 | 1 << 8 | 1 << 9);
	if (slcdc_gpio2_inited)
		mx2_unregister_gpios(PORT_B, 1 << 5);
#endif
}

static void
slcdc_reset(int level)
{
	if (level == 0) {
#ifdef SLCDC_MUX_LCDC
		/* set reset pin to low */
		mx2_gpio_set_bit(PORT_A, 31, 0);
#endif

#ifdef SLCDC_MUX_SDHC2
		/* set reset pin to low */
		mx2_gpio_set_bit(PORT_B, 5, 0);
#endif
	} else {
#ifdef SLCDC_MUX_LCDC
		/* set reset pin to high */
		mx2_gpio_set_bit(PORT_A, 31, 1);
#endif
#ifdef SLCDC_MUX_SDHC2
		/* set reset pin to high */
		mx2_gpio_set_bit(PORT_B, 5, 1);
#endif

	}
}

static void
slcdc_init_dev(void)
{

	MAX_SLV_SGPCR(3) |= 0x00040000;
	SYS_PSCR |= 0x00000004;	/* set LCD/SLCD bus master high priority */
	/* enable the slcd clk */
	mx_module_clk_open(HCLK_MODULE_SLCDC);
	mx_module_clk_open(IPG_MODULE_SLCDC);
}

static void
slcdc_init_reg(void)
{
	SLCDC_DBADDR = 0;
	SLCDC_DBUF_SIZE = 0;
	SLCDC_CBADDR = 0;
	SLCDC_CBUF_SIZE = 0;
	SLCDC_CBUF_SSIZE = 0;
	SLCDC_FIFO_CONFIG = 0;
	SLCDC_LCD_CONFIG = 0;
	SLCDC_LCD_TXCONFIG = 0;
	SLCDC_LCD_CTRL_STAT = 0;
	SLCDC_LCD_CLKCONFIG = 0;
}

#define SLCD_CLK_FREQ 10000000

static void slcdc_set_clk(void)
{
u32 tmpclock = mx_module_get_clk(HCLK);
	tmpclock = ((u32)SLCD_CLK_FREQ*128)/tmpclock + ((((u32)SLCD_CLK_FREQ*128)%tmpclock > (tmpclock >> 1)) ? 1 : 0);
	if (!tmpclock) tmpclock = 1;
	if (tmpclock > 63) tmpclock = 63;
	SLCDC_LCD_CLKCONFIG = tmpclock;
}

static void
slcdc_config(int datlen)
{
	u32 xfrmode, sckpol, worddefcom, imgend = 0, worddefwrite =
	    0, worddefdat = 0;
	if (datlen == 8) {
		imgend = 0x2;	/* 8-bit little endian; */
		worddefdat = 0;	/* 8-bit data */
		worddefwrite = 10;
	} else if (datlen == 16) {
		imgend = 0x1;	/* 16-bit little endian; */
		worddefdat = 1;	/* 16-bit data */
		worddefwrite = 1;
	}
	worddefcom = 1;
	xfrmode = 0;		/* serial mode */
	sckpol = 1;		/* falling edge */

	/* config to be little endian serial 16bit */
	SLCDC_LCD_TXCONFIG =
	    (imgend << 16) | (worddefdat << 4) | (worddefcom << 3) | (xfrmode <<
								      2) |
	    (sckpol);

	/* config dma setting */
	SLCDC_FIFO_CONFIG = 5;	/* burst length is 4 32-bit words */
	/* config buffer address setting */
	SLCDC_DBADDR = (u32) (slcdc_par.screen_start_address);
	SLCDC_CBADDR = (u32) g_slcdc_cbuffer_phyaddress;
	/* config clk setting */
	slcdc_set_clk();
	/* set GO 0 */
	SLCDC_LCD_CTRL_STAT = 0;

	udelay(5);

}

/* for command transfer, it is very short, will not use interrupt */
static int
slcdc_send_cmd(u32 length)
{
	u32 status;
	/* disable interrupt */
	SLCDC_LCD_CTRL_STAT &= 0xffffff7f;
	/* set length */
	SLCDC_DBUF_SIZE = length;
	/* set automode 00 for command */
	SLCDC_LCD_CTRL_STAT &= 0x000001ff;
	/* set GO */
	SLCDC_LCD_CTRL_STAT |= 0x1;
	/* polling for data transfer finish */
	status = SLCDC_LCD_CTRL_STAT;
	while ((!(status & SLCDC_TRANSFER_ERROR))
	       && (status & SLCDC_TRANSFER_BUSY)) {
		status = SLCDC_LCD_CTRL_STAT;
	}

	if (status & SLCDC_TRANSFER_ERROR) {
		return 1; /*error*/
	} else {
		SLCDC_LCD_CTRL_STAT |= 0x40;
		SLCDC_LCD_CTRL_STAT |= 0x20;
		return 0;
	}
}

static void
slcdc_send_data(u32 length)
{
	/* enable interrupt */
#ifdef USING_INTERRUPT_SLCDC
	SLCDC_LCD_CTRL_STAT |= ~0xffffff7f;
#endif
	init_completion(&slcdc_wait);
	/* set length */
	SLCDC_DBUF_SIZE = length;
	/* set automode 01 for data */
	SLCDC_LCD_CTRL_STAT |= 0x00000800;
	/* set GO */
	SLCDC_LCD_CTRL_STAT |= 0x1;
#ifdef USING_INTERRUPT_SLCDC
	wait_for_completion(&slcdc_wait);
#else
	while ((SLCDC_LCD_CTRL_STAT & 0x00000004) != 0) ;

	SLCDC_LCD_CTRL_STAT |= 0x40;
	SLCDC_LCD_CTRL_STAT |= 0x20;
#endif
	return;

}

static void
slcdc_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	u32 status;
	status = SLCDC_LCD_CTRL_STAT;
	/* clear interrupt */
	SLCDC_LCD_CTRL_STAT |= 0x40;
	complete(&slcdc_wait);
	return;
}

static inline int
slcdc_init_buffer(void)
{
	u_int required_pages;
	u_int extra_pages;
	u_int order;
	struct page *page;
	u16 *allocated_region;

	/*find order required to allocate enough memory for it */
	required_pages = SLCDC_DATA_MEM_SIZE >> PAGE_SHIFT;
	for (order = 0; required_pages >> order; order++) {;
	}
	extra_pages = (1 << order) - required_pages;

	if ((allocated_region =
	     (u16 *) __get_free_pages(GFP_KERNEL | GFP_DMA, order)) == NULL) {
		printk(KERN_ERR MODULE_NAME ": not enough memory for data buffer\n");
		return -ENOMEM;
	}

	g_slcdc_dbuffer_address =
	    (u16 *) ((u_char *) allocated_region + (extra_pages << PAGE_SHIFT));
	g_slcdc_dbuffer_phyaddress =
	    (u16 *) ((u_char *)
		     __virt_to_phys((u_long) g_slcdc_dbuffer_address));


	/* Free all pages that we don't need but were given to us because */
	/* __get_free_pages() works on powers of 2. */
	for (; extra_pages; extra_pages--)
		free_page((u_int) allocated_region +
			  ((extra_pages - 1) << PAGE_SHIFT));

	/* Set reserved flag for fb memory to allow it to be remapped into */
	/* user space by the common fbmem driver using remap_page_range(). */
	for (page = virt_to_page(g_slcdc_dbuffer_address);
	     page < virt_to_page(g_slcdc_dbuffer_address + SLCDC_DATA_MEM_SIZE);
	     page++) {
		mem_map_reserve(page);
	}
	slcdc_par.screen_start_address =
	    (u16 *) ((u_long) g_slcdc_dbuffer_phyaddress);
	slcdc_par.v_screen_start_address =
	    (u16 *) ((u_long) g_slcdc_dbuffer_address);

	memset(slcdc_par.v_screen_start_address, 0, SLCDC_DATA_MEM_SIZE);

	g_slcdc_cbuffer_address =
	    consistent_alloc(GFP_KERNEL | GFP_DMA, (SLCDC_CMD_MEM_SIZE + 4),
			     (dma_addr_t *) & g_slcdc_cbuffer_phyaddress);

	if (!g_slcdc_cbuffer_address) {
		printk(KERN_ERR MODULE_NAME ": not enough memory for command buffer\n");
		return -ENOMEM;
	}
	g_slcdc_cbuffer_address =
	    (u16 *) ((u_long) g_slcdc_cbuffer_address & 0xfffffff4);
	g_slcdc_cbuffer_phyaddress =
	    (u16 *) ((u_long) g_slcdc_cbuffer_phyaddress & 0xfffffff4);


	return 0;
}

static inline void
slcdc_free_buffer(void)
{
	struct page *page;
	u_int required_pages;
	required_pages = SLCDC_DATA_MEM_SIZE >> PAGE_SHIFT;
	/* free command buffer */
	if (g_slcdc_cbuffer_address != NULL) {
		consistent_free(g_slcdc_cbuffer_address, SLCDC_CMD_MEM_SIZE,
				(dma_addr_t) g_slcdc_cbuffer_phyaddress);
		g_slcdc_cbuffer_address = NULL;
	}

	if (g_slcdc_dbuffer_address != NULL) {
		/* free data buffer       */
		for (page = virt_to_page(g_slcdc_dbuffer_address);
		     page < virt_to_page(g_slcdc_dbuffer_address + SLCDC_DATA_MEM_SIZE);
		     page++) {
			mem_map_unreserve(page);
		}
		for (; required_pages; required_pages--)
			free_page((u_int) g_slcdc_dbuffer_address +
				  ((required_pages - 1) << PAGE_SHIFT));
		g_slcdc_dbuffer_address = NULL;
	}

}

static int
slcdc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long page, pos;
	unsigned long start = (unsigned long) vma->vm_start;
	unsigned long size = (unsigned long) (vma->vm_end - vma->vm_start);

	if (size > SLCDC_DATA_MEM_SIZE)
		return -EINVAL;

	pos = (unsigned long) slcdc_par.v_screen_start_address;

	while (size > 0) {
		page = virt_to_phys((void *) pos);

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		/*  This is an IO map - tell maydump to skip this VMA */
		vma->vm_flags |= VM_IO;

		if (remap_page_range(start, page, PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	return 0;
}

static void
slcdc_display_on(void)
{
	u16 *databuffer = NULL;

	SLCDC_DBADDR = (u32) g_slcdc_cbuffer_phyaddress;

	/* put cmd into cmd buffer */
	databuffer = g_slcdc_cbuffer_address;
	/* put cmd into cmd buffer */
	*databuffer = SLCDC_CMD_DISON;
	/* send cmd */
	slcdc_send_cmd(1);
	/* delay */
	udelay(50);
	slcdc_display_on_true = 1;
}

static void
slcdc_display_off(void)
{
	u16 *databuffer = NULL;
	/* Cause LCD module to enter sleep mode.Bu sure to input display OFF
	   command to turn off the display before inputting the SLPIN command
	   Keep the logic power supply turned on for 40ms
	   after the LCD module enters the sleep mode.
	   put cmd into cmd buffer
	 */
	databuffer = slcdc_par.v_screen_start_address;
	/* put cmd into cmd buffer */
	*databuffer = SLCDC_CMD_DISOFF;
	/* send cmd DISOFF */
	slcdc_send_cmd(1);
	udelay(50);
	slcdc_display_on_true = 0;
}

static void
slcdc_display_normal(void)
{
	u16 *databuffer = NULL;
	databuffer = slcdc_par.v_screen_start_address;
	*databuffer = SLCDC_CMD_DISNOR;
	slcdc_send_cmd(1);
	udelay(1);
}

static void
slcdc_display_inverted(void)
{
	u16 *databuffer = NULL;
	databuffer = slcdc_par.v_screen_start_address;
	*databuffer = SLCDC_CMD_DISINV;
	slcdc_send_cmd(1);
	udelay(1);
}

static void
slcdc_sleep_out(void)
{
	u16 *databuffer = NULL;
	databuffer = slcdc_par.v_screen_start_address;
	*databuffer = SLCDC_CMD_SLPOUT;
	slcdc_send_cmd(1);
	udelay(50);
}

static void
slcdc_sleep_in(void)
{
	u16 *databuffer = NULL;
	databuffer = slcdc_par.v_screen_start_address;
	*databuffer = SLCDC_CMD_SLPIN;
	slcdc_send_cmd(1);
	udelay(50);
}

#if 0 /*function currently not in use*/
static void
slcdc_display_ctl(void)
{
	int i;
	u16 disctl[11] = { 0x1c00, 0x0200, 0x8200, 0x0000,
		0x1e00, 0xe000, 0x0000, 0xdc00,
		0x0000, 0x0200, 0x0000
	};
	u16 *databuffer = NULL;
	/* It make various display timing settings. */
	databuffer = slcdc_par.v_screen_start_address;
	*databuffer = SLCDC_CMD_DISCTL;
	slcdc_send_cmd(1);
	udelay(2);
	/* put parameter into data buffer */
	for (i = 0; i < 11; i++) {
		*databuffer = disctl[i];
		databuffer++;
	}
	/* send data */
	slcdc_send_data(11);

}
#endif

static void
slcdc_page_set(void)
{
/* It specify a page address area in order to access the display data RAM
from the MPU. Be sure to set both starting and ending pages. */
	int i;
	u16 *databuffer = NULL;
	u16 pset[4] = { 0x0000, 0x0000, 0xb100, 0x0000 };
	databuffer = (u16 *) (slcdc_par.v_screen_start_address);
	*databuffer = SLCDC_CMD_SD_PSET;
	slcdc_send_cmd(1);
	udelay(20);

	for (i = 0; i < 4; i++) {
		*databuffer = pset[i];
		databuffer++;
	}
	slcdc_send_data(4);

	udelay(20);
}

static void
slcdc_col_set(void)
{
/* It specify starting and ending collumns. */
	int i;
	u16 cset[4] = { 0x0200, 0x0000, 0xb100, 0x0000 };
	u16 *databuffer = NULL;
	databuffer = (u16 *) (slcdc_par.v_screen_start_address);
	*databuffer = SLCDC_CMD_SD_CSET;
	slcdc_send_cmd(1);
	udelay(20);
	for (i = 0; i < 4; i++) {
		*databuffer = cset[i];
		databuffer++;
	}
	slcdc_send_data(4);
	udelay(20);

}

static void
slcdc_data_ctl(void)
{
	u8 *databuffer = NULL;
	databuffer = (u8 *) (slcdc_par.v_screen_start_address);
	*databuffer = SLCDC_CMD_DATCTL;
	slcdc_send_cmd(1);
	udelay(20);
	*databuffer = 0x28;
	slcdc_send_data(1);
	udelay(20);
}

static void
slcdc_volctl(void)
{
	u16 *databuffer = NULL;
	databuffer = slcdc_par.v_screen_start_address;
	*databuffer = SLCDC_CMD_VOLCTR;
	slcdc_send_cmd(1);
	udelay(20);
	*databuffer = 0x9f00;
	slcdc_send_data(1);
	udelay(20);
}

static void
slcdc_ascset(void)
{
	int i;
	u16 lcd_para_ASCSET[7] = { 0x0000, 0x0000, 0x1f00, 0x0100,
		0x1f00, 0x0100, 0x0300
	};
	u16 *databuffer = NULL;
	databuffer = (u16 *) (slcdc_par.v_screen_start_address);
	*databuffer = SLCDC_CMD_ASCSET;
	slcdc_send_cmd(1);
	udelay(2);
	for (i = 0; i < 7; i++) {
		*databuffer = lcd_para_ASCSET[i];
		databuffer++;
	}
	slcdc_send_data(7);
	udelay(20);
}

static void
slcdc_scstart(void)
{
	int i;
	u16 lcd_para_SCSTART[2] = { 0x00, 0x00 };
	u16 *databuffer = NULL;
	databuffer = (u16 *) (slcdc_par.v_screen_start_address);
	*databuffer = SLCDC_CMD_SCSTART;
	slcdc_send_cmd(1);
	udelay(10);
	for (i = 0; i < 2; i++) {
		*databuffer = lcd_para_SCSTART[i];
		databuffer++;
	}
	slcdc_send_data(2);
	udelay(20);

}

static void
slcdc_config_panel(void)
{
	int i;

	slcdc_config(8);
	udelay(50);
	slcdc_reset(1);		/* pull reset signal high */

	/* set data format */
	udelay(20);
	slcdc_data_ctl();
	udelay(1);
	slcdc_config(16);

	slcdc_sleep_out();

	udelay(20);

	for (i = 0; i < 8; i++) {
		slcdc_volctl();
		udelay(2);
	}
	udelay(50);
	udelay(50);
	udelay(50);

	slcdc_display_on();
	udelay(50);

	/* set col address */
	slcdc_col_set();
	/* set page address */
	slcdc_page_set();
	/* set area in screen to be used for scrolling */
	slcdc_ascset();
	udelay(20);
	/* set top scroll page within the scroll area */
	slcdc_scstart();
	udelay(50);
}

static void
slcdc_display_ramwr(void)
{
	u16 *databuffer = NULL;
		/* Causes the MPU to be a data entry mode,
		  allowing it to serite data in the display memory.
		  Inputting any other cmds other than NOP cancels
		  the data entry mode.
		 */
		SLCDC_DBADDR = (u32) g_slcdc_cbuffer_phyaddress;

		databuffer = g_slcdc_cbuffer_address;
		*databuffer = SLCDC_CMD_RAMWR;
		slcdc_send_cmd(1);
		udelay(2);

		/* this is to display one data per time, it is ok. */
		udelay(50);
		udelay(50);
		SLCDC_DBADDR = (u32) slcdc_par.screen_start_address;

		udelay(50);
		udelay(50);
		slcdc_send_data(38720);
		udelay(50);
		udelay(50);
}

static int
slcdc_ioctl(struct inode *inode, struct file *filp, u_int cmd, u_long arg)
{
	if (slcdc_suspended) return -ENAVAIL;
	switch (cmd) {
	case SLCDC_CMD_DISON:
		slcdc_display_on();
		break;

	case SLCDC_CMD_DISOFF:
		slcdc_display_off();
		break;

	case SLCDC_CMD_DISNOR:
		slcdc_display_normal();
		break;

	case SLCDC_CMD_DISINV:
		slcdc_display_inverted();
		break;

	case SLCDC_CMD_DATCTL:
		break;

	case SLCDC_CMD_RAMWR:
		slcdc_display_ramwr();
		break;

	case SLCDC_CMD_RAMRD:
		break;

	case SLCDC_CMD_PTLIN:
		/* This command is used to display a partial screen
		   for power saving. */
		break;

	case SLCDC_CMD_PTLOUT:
		/* This command is used to exit the partila diaplay mode. */
		break;

	case SLCDC_CMD_GCP64:
		/* make 63 pulse position settings of GCP for 64 gray scales. */
		break;

	case SLCDC_CMD_GCP16:
		break;

	case SLCDC_CMD_GSSET:
		break;

	case SLCDC_CMD_ASCSET:
		/* make partial screen scroll settings. */
		break;

	case SLCDC_CMD_SCSTART:
		/* set a scroll starting page in the scrolling area.
		   Be sure to send this cmd after ASCSET . */
		break;
	default:
		break;
	}
	return 0;
}

static int
slcdc_release(struct inode *inode, struct file *filp)
{
	if (slcdc_suspended) return -ENAVAIL;
	slcdc_busy=0;
	slcdc_display_off();
	slcdc_sleep_in();
	mx_module_clk_close(HCLK_MODULE_SLCDC);
	mx_module_clk_close(IPG_MODULE_SLCDC);
	slcdc_free_buffer();
	return 0;
}

static int
slcdc_open(struct inode *inode, struct file *filp)
{
int tmp;
	if (slcdc_busy) return -EBUSY;
	if (slcdc_suspended) return -ENAVAIL;

	if ((tmp=slcdc_init_buffer()) < 0) {
		slcdc_free_buffer();
		return tmp;
	}

	slcdc_busy=1;

	slcdc_init_dev();

	slcdc_reset(0);		/* pull reset low */

	slcdc_init_reg();

	/* send some command for panel configuration */
	slcdc_config_panel();

	return 0;
}

static void
slcdc_suspend (void)
{
char tmp;
	if (!slcdc_suspended) {
		slcdc_suspended = 1;
		if (slcdc_busy) {
			tmp = slcdc_display_on_true;
			if (tmp)
			  slcdc_display_off();
			slcdc_display_on_true = tmp;
			slcdc_sleep_in();
			mx_module_clk_close(HCLK_MODULE_SLCDC);
			mx_module_clk_close(IPG_MODULE_SLCDC);
		}
	}
}

static void
slcdc_resume (void)
{
	if (slcdc_suspended) {
		slcdc_suspended = 0;
		if (slcdc_busy) {
			mx_module_clk_open(HCLK_MODULE_SLCDC);
			mx_module_clk_open(IPG_MODULE_SLCDC);
			slcdc_sleep_out();
			if (slcdc_display_on_true) {
			  slcdc_display_on();
			  slcdc_display_ramwr();
			}
		}
	}
}

static int
slcdc_pm_handler(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	switch (rqst) {
	case PM_RESUME:
		slcdc_suspend();
		break;
	case PM_SUSPEND:
		slcdc_resume();
		break;
	default:
		break;
	}
	return 0;
}
#if 1 /*CEE LDM*/
static int
__ldm_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		slcdc_suspend();
		break;
	}
	return 0;
}

static int
__ldm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		slcdc_resume();
		break;
	}
	return 0;
}
#endif

static void __init slcdc_cleanup(void);

static int __init
slcdc_init(void)
{
	int tmp;

	if ((tmp=slcdc_gpio_init()) < 0) {
		slcdc_cleanup();
		return tmp;
	}

	g_slcdc_major = devfs_register_chrdev(0, "slcdc", &g_slcdc_fops);
	if (g_slcdc_major < 0) {
		printk(KERN_ERR MODULE_NAME ": Unable to register device major\n");
		slcdc_cleanup();
		return -ENODEV;
	}

	g_devfs_handle = devfs_register(NULL, "slcdc", DEVFS_FL_DEFAULT,
					g_slcdc_major, 0,
					S_IFCHR | S_IRUSR | S_IWUSR,
					&g_slcdc_fops, NULL);

	if (!g_devfs_handle) {
		printk(KERN_ERR MODULE_NAME ": Unable to register device minor\n");
		slcdc_cleanup();
		return -ENODEV;
	}


	tmp = request_irq(SLCDC_IRQ,
			  (void *) slcdc_isr,
			  SA_INTERRUPT | SA_SHIRQ, MODULE_NAME, MODULE_NAME);
	if (tmp) {
		printk(KERN_ERR MODULE_NAME ":  failed to request IRQ\n");
		slcdc_cleanup();
		return -EFAULT;
	}
	slcdc_irq_inited = 1;

	g_slcdc_pm = pm_register(PM_SYS_DEV, PM_SYS_VGA, slcdc_pm_handler);
#if 1 /*CEE LDM*/
	mx21_ldm_bus_register(&__device_ldm, &__driver_ldm);
#endif

	return 0;
}

static void __init
slcdc_cleanup(void)
{
	pm_unregister(g_slcdc_pm);
#if 1 /*CEE LDM*/
	mx21_ldm_bus_unregister(&__device_ldm, &__driver_ldm);
#endif

	/*Do some cleanup work */
	if (slcdc_irq_inited)
		free_irq(SLCDC_IRQ, MODULE_NAME);

	if (g_devfs_handle) {
		devfs_unregister(g_devfs_handle);
	}

	if (g_slcdc_major > 0) {
		devfs_unregister_chrdev(g_slcdc_major, "slcdc");
	}

	slcdc_gpio_uninit();

}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Motorola Inc Ltd");
MODULE_DESCRIPTION("EPSON L2F50032T00 SLCD FOR MX2ADS");
module_init(slcdc_init);
module_exit(slcdc_cleanup);
