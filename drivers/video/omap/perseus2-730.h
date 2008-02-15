/*
 * File: drivers/video/omap/perseus2-730.h
 *
 * This file is the header file for the Perseus2's framebuffer - We utilise
 * an Epson MD-TFT LCD for this device.
 *
 * Copyright (C) 2004 MPC-Data Limited (http://www.mpc-data.co.uk)
 * Author: Simon Dunn <sdunn at mpc-data.co.uk> 
 *         Dave Peverley <dpeverley at mpc-data.co.uk>
 *
 * Copyright (C) 2004 Texas Instruments Limited
 * Author: Jean Pihet
 *
 * Copyright (C) 2001 RidgeRun, Inc. 
 * Author: Alex McMains <aam@ridgerun.com> 2001/09/20
 *         Greg Lonnon <glonnon@ridgerun.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
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
 *
 * Please report all bugs and problems to the author or <support@dsplinux.net>
 *
 * 26/02/2004 SJD:  Modified to use the MV uwire interface
 *
 */

#ifndef _p2omap_730_
#define _p2omap_730_

#include <asm/hardware.h>
#include <asm/arch/ck.h>
#include <asm/arch/dma.h>
#include <asm/arch/perseus2.h>
#include "epson-md-tft.h"

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif


#define NUM_XPIXELS 176
#define NUM_YPIXELS 220

#define COLOR_16BIT

#ifdef COLOR_16BIT
#define BPP 16
#define PALETTE_SIZE 32
#define BPP_MASK 0x4000
#else
#define BPP 8
#define PALETTE_SIZE 512
#define BPP_MASK 0x3000
#endif

#define MAX_FRAMEBUFFER_SIZE (((NUM_XPIXELS) * (NUM_YPIXELS) * (BPP)/8) + \
                              (PALETTE_SIZE) + PAGE_SIZE)

// Put the framebuffer at top of SRAM
#if	0
/*  REVISIT: Caveat Emptor: Whatever value we use, it _must_ be page
 *  aligned. For now, just use the (currently page aligned) default
 *  define from <asm/hardware.h>.
 */
#undef SRAM_FRAMEBUFFER_MEMORY
#define SRAM_FRAMEBUFFER_MEMORY PAGE_ALIGN(OMAP1510_SRAM_BASE + OMAP1510_SRAM_SIZE - MAX_FRAMEBUFFER_SIZE - 8192)
#endif


// we use this to check for valid displays in set_var
struct _displays 
{
        int xres;
        int yres;
        int bpp;
};
static struct _displays displays_supported[] = { {NUM_XPIXELS, NUM_YPIXELS, 16} };

#ifdef COLOR_8BIT
static struct fb_var_screeninfo tifb_default8 = 
{
        xres: NUM_XPIXELS, 
        yres: NUM_YPIXELS, 
        xres_virtual: NUM_XPIXELS, 
        yres_virtual: NUM_YPIXELS, 
        xoffset: 0, 
        yoffset: 0, 
        bits_per_pixel: 8, 
        grayscale : 0,
        red: {0, 8, 0}, 
        green: {0, 8, 0}, 
        blue: {0, 8, 0}, 
        transp: {0, 0, 0},
        nonstd: 0, 
        activate: 0, 
        height: -1, 
        width: -1, 
        accel_flags: 0, 
        pixclock: 171521, 
        left_margin: 64, 
        right_margin: 64, 
        upper_margin: 32, 
        lower_margin: 32,
        hsync_len : 64, 
        vsync_len: 2,
        sync: 0, 
        vmode: FB_VMODE_NONINTERLACED
};
#endif

#ifdef COLOR_16BIT
static struct fb_var_screeninfo tifb_default16 = 
{
        xres: NUM_XPIXELS, 
        yres: NUM_YPIXELS, 
        xres_virtual: NUM_XPIXELS, 
        yres_virtual: NUM_YPIXELS, 
        xoffset: 0, 
        yoffset: 0, 
        bits_per_pixel: 16, 
        grayscale : 0,
        red: {11, 5, 0}, 
        green: {5, 6, 0}, 
        blue: {0, 5, 0}, 
        transp: {0, 0, 0},
        nonstd: 0, 
        activate: 0, 
        height: -1, 
        width: -1, 
        accel_flags: 0, 
        pixclock: 171521, 
        left_margin: 64, 
        right_margin: 64, 
        upper_margin: 32, 
        lower_margin: 32,
        hsync_len : 64, 
        vsync_len: 2,
        sync: 0, 
        vmode: FB_VMODE_NONINTERLACED
};
#endif

extern u16 omap_uwire_data_transfer(u8 cs, u16 data, u8 trans_size, u8 rec_size);
extern void omap_uwire_cs0_configure_mode(u8 edge_rd, u8 edge_wr, u8 lvl, u8 frq, u8 chk);

void epson_md_tft_lcd_init(void)
{
	int i;
	
	/* Initial state after resetting */
	omap_uwire_data_transfer(0, (LCD_DISOFF << 7), 9, 0);
	omap_uwire_data_transfer(0, (LCD_SLPIN << 7), 9, 0);
	omap_uwire_data_transfer(0, (LCD_DISNOR << 7), 9, 0);
	omap_uwire_data_transfer(0, (LCD_GSSET << 7), 9, 0);
	omap_uwire_data_transfer(0, ((INIT_GSSET << 7) | 0x8000), 9, 0);

	/* omap_uwire_data_transfer(0, (LCD_DATCTL << 7), 9, 0); */
	/* omap_uwire_data_transfer(0, ((0x08 << 7) | 0x8000), 9, 0); */

	/* after resetting */
	/* DISCTL */ 
	omap_uwire_data_transfer(0, (LCD_DISCTL << 7), 9, 0); /* Init LCD controller */
	for (i = 0; i < (sizeof(INIT_DISCTL)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(0, ((INIT_DISCTL[i] << 7) | 0x8000), 9, 0);
	
	/* GCP64 */
	omap_uwire_data_transfer(0, (LCD_GCP64 << 7), 9, 0);
	for (i = 0; i < (sizeof(INIT_GCP64)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(0, ((INIT_GCP64[i] << 7) | 0x8000), 9, 0);

	/* GCP16 */
	omap_uwire_data_transfer(0, (LCD_GCP16 << 7), 9, 0);
	for (i = 0; i < (sizeof(INIT_GCP16)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(0, ((INIT_GCP16[i] << 7) | 0x8000), 9, 0);

	/* MD_CSET */
	omap_uwire_data_transfer(0, (LCD_MD_CSET << 7), 9, 0);
	for (i = 0; i < (sizeof(INIT_MD_CSET)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(0, ((INIT_MD_CSET[i] << 7) | 0x8000), 9, 0);

	/* MD_PSET */
	omap_uwire_data_transfer(0, (LCD_MD_PSET << 7), 9, 0);
	for (i = 0; i < (sizeof(INIT_MD_PSET)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(0, ((INIT_MD_PSET[i] << 7) | 0x8000), 9, 0);

	/* SD_CSET */
	omap_uwire_data_transfer(0, (LCD_SD_CSET << 7), 9, 0);
	for (i = 0; i < (sizeof(INIT_SD_CSET)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(0, ((INIT_SD_CSET[i] << 7) | 0x8000), 9, 0);

	/* SD_PSET */
	omap_uwire_data_transfer(0, (LCD_SD_PSET << 7), 9, 0);
	for (i = 0; i < (sizeof(INIT_SD_PSET)/sizeof(unsigned short)); i++)
		omap_uwire_data_transfer(0, ((INIT_SD_PSET[i] << 7) | 0x8000), 9, 0);

	/* DATCTL */
	omap_uwire_data_transfer(0, (LCD_DATCTL << 7), 9, 0);
	omap_uwire_data_transfer(0, ((INIT_DATCTL << 7) | 0x8000), 9, 0);

	/* OSSISEL = d'5 */
	omap_uwire_data_transfer(0, (LCD_OSCISEL << 7), 9, 0);
	omap_uwire_data_transfer(0, ((INIT_OSCISEL << 7) | 0x8000), 9, 0);

	/* 14MSET = d'74 */
	omap_uwire_data_transfer(0, (LCD_14MSET << 7), 9, 0);
	omap_uwire_data_transfer(0, ((INIT_14MSET << 7) | 0x8000), 9, 0);
	
	/* 14MEND */
	omap_uwire_data_transfer(0, (LCD_14MEND << 7), 9, 0);

	/* 3500KSET = d'69 */
	omap_uwire_data_transfer(0, (LCD_3500KSET << 7), 9, 0);
	omap_uwire_data_transfer(0, ((INIT_3500KSET << 7) | 0x8000), 9, 0);

	/* 3500KEND */
	omap_uwire_data_transfer(0, (LCD_3500KEND << 7), 9, 0);

	mdelay(30);
	
	omap_uwire_data_transfer(0, (LCD_SLPOUT << 7), 9, 0);
	mdelay(20);
	
	omap_uwire_data_transfer(0, (LCD_VOLCTL << 7), 9, 0);
	omap_uwire_data_transfer(0, ((INIT_VOLCTL_Ton << 7) | 0x8000), 9, 0);
	mdelay(20);
	
	omap_uwire_data_transfer(0, (LCD_VOLCTL << 7), 9, 0);
	omap_uwire_data_transfer(0, ((INIT_VOLCTL << 7) | 0x8000), 9, 0);
	mdelay(20);
	
	omap_uwire_data_transfer(0, (LCD_DISON << 7), 9, 0);
}


static inline void hardware_enable(void)
{
	DBG("LCD : hardware_enable() called\n");

        ck_set_rate(lcd_ck, (ck_get_rate(tc_ck) / 2));
        ck_enable(lcd_ck);

	/* Setup uWire */
	omap_uwire_cs0_configure_mode(0, 0, 0, 2, 0);

	/* Note : The LCD Reset used to be here but was moved into perseus2.c
	 * as the reset gpio *also* resets the USB XCVR and BT chip! 
	 * That approach seems to work ok in practice.
	 */

	/* Setup LCD via uWire */
	epson_md_tft_lcd_init();
	mdelay(10);
}

static inline void hardware_disable(void)
{
	DBG("LCD : hardware_disable() called\n");
 
#if 0 /* TODO: Do not hold the reset pin else we break USB */
	/* Hold LCD reset */
	set_p2_reset_lcd_usb_bt(LCD_USB_BT_RESET_HOLD);
#endif

	/* Stop the clock */
	ck_disable(lcd_ck);        
}

static inline int lcd_active(void)
{
    // Return true if LCD controller is running.
    if (*ARM_IDLECT2 & (1 << EN_LCDCK))
        return(LCD_READ(TI925_LCD_CONTROL) & (1 << LCD_EN_POS));
    else
        return 0;
}

static inline void lcd_enable(void)
{
	unsigned long value;

	DBG("LCD : lcd_enable() called\n");


	ck_disable(lcd_ck);
	ck_disable(dma_ck);

	mdelay(1);

	/* DMA AUTOGATING_OFF */
	*((volatile __u32 *) OMAP_DMA_GCR) &= ~(GCR_CLK_AUTOGATING_ON);

	/* enable the clock */
	ck_enable(lcd_ck);

	mdelay(5);

	/* Clear Status bits */
	LCD_WRITE(0, TI925_LCD_STATUS);	

        // enable the LCD
        value = LCD_READ(TI925_LCD_CONTROL);
        value |= (1 << LCD_EN_POS);
        LCD_WRITE(value,TI925_LCD_CONTROL);

	/* Clear Status bits */
	LCD_WRITE(0, TI925_LCD_STATUS);	

	/* DMA AUTOGATING_ON */
	*((volatile __u32 *) OMAP_DMA_GCR) |= GCR_CLK_AUTOGATING_ON;

	enable_irq(INT_LCD_CTRL);

	set_p2_backlight_state(BACKLIGHT_ON);

	mdelay(10);
}

static inline void lcd_disable(void)
{
        unsigned long value;


        set_p2_backlight_state(BACKLIGHT_OFF);

	disable_irq(INT_LCD_CTRL);

	if (lcd_active()) {
		/* Wait until current frame finished:
		 * clear VSYNC bit and wait for the LCDStatus[Done] bit.
		 */
		value = LCD_READ(TI925_LCD_STATUS);
		value &= ~(VS_INT);
		LCD_WRITE(value, TI925_LCD_STATUS);
		while (!(LCD_READ(TI925_LCD_STATUS) & VS_INT));

		// disable the LCD and wait for it to be done
		value = LCD_READ(TI925_LCD_CONTROL);
		value &= ~(1 << LCD_EN_POS);
		LCD_WRITE(value,TI925_LCD_CONTROL);

		// Wait until current frame finished.
		//  Poll on LCDStatus[Done] bit.
		for(value=1000; value && (!(LCD_READ(TI925_LCD_STATUS) & DONE_INT)); value--)
			udelay(100);

		/* Stop the clock */
		ck_disable(lcd_ck);

		mdelay(5);
	}
}

static inline unsigned long omap730_dma_reg_addr(int global, int chan, int reg) 
{ 
        return (OMAP730_DMA_BASE + (global << 10) + (chan << 6) + (reg << 1));
}
 
static inline void lcd_write_top_address(unsigned long addr) 
{
        unsigned long reg;

        reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, 
                                    DMA_LCD_TOP_B1_U);

        DBG("%s\n%lx %lx\n", __FUNCTION__, reg, addr >> 16);

        outw(addr >> 16, reg);

        DBG("wrote %lx %x\n", reg, inw(reg));

        reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, 
                                    DMA_LCD_TOP_B1_L);

        DBG("%lx %lx\n", reg, addr & 0xffff);

        outw(addr & 0xffff, reg);

        DBG("wrote %lx %x\n", reg, inw(reg));
}

static inline void lcd_write_bottom_address(unsigned long addr) 
{
        unsigned long reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                                  LCD_CHANNEL, 
                                                  DMA_LCD_BOT_B1_U);

        DBG("%s\n%lx %lx\n", __FUNCTION__, reg, addr >> 16);

        outw(addr >> 16, reg);

        DBG("wrote %lx %x\n", reg, inw(reg));

        reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                    LCD_CHANNEL, DMA_LCD_BOT_B1_L);

        DBG("%lx %lx\n", reg, addr&0xffff);

        outw(addr & 0xffff, reg);

        DBG("wrote %lx %x\n", reg, inw(reg));
}

static inline void lcd_dma_enable(void) 
{
        unsigned long reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, 
                                                  LCD_CHANNEL,
                                                  DMA_LCD_CTRL);

#ifdef CONFIG_FB_SDRAM
	/* 1 block only */
	outw(LCD_BLOCK_MODE, reg);
#else
	/* 1 block only */
	outw(LCD_BLOCK_MODE | LCD_SOURCE_IMIF, reg);
#endif

        DBG("LCD : wrote 0x%08x 0x%08x\n", reg, inl(reg));
}

#ifdef FB_DEBUG 
static inline void print_lcd_regs(void)
{
	DBG("%s\n", __FUNCTION__);

	DBG("control: %8.8x status %8.8x timing0 %8.8x" 
	     " timing1 %8.8x timing2 %8.8x\n", 
	     (int)LCD_READ(TI925_LCD_CONTROL), 
	     (int)LCD_READ(TI925_LCD_STATUS),
	     (int)LCD_READ(TI925_LCD_TIMING0),
	     (int)LCD_READ(TI925_LCD_TIMING1),
	     (int)LCD_READ(TI925_LCD_TIMING2));
}
#else
static inline void print_lcd_regs(void) {}
#endif

static void tifb_init_dma(void)
{
	unsigned long reg;
	
	/*  Reset DMA & put 3.2 mode */
	*((volatile __u16 *) OMAP_DMA_GRST) |= GRST_SW_RESET;
	*((volatile __u16 *) OMAP_DMA_GSCR) |= GSCR_OMAP_3_1_MAPPING_DISABLE;
	
	/* DMA free running */
	/* *((volatile __u32 *) OMAP_DMA_GCR) |= GCR_FREE; */
	
	/* 3.2 mode + other params */
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_CCR);
	outw(CCR_HIGH_PRIO | CCR_EN | CCR_AUTOINT | CCR_REPEAT | OMAP3_1_COMPATIBLE_DISABLE |
	     SRC_AMODE_B1_POSTINC | SRC_AMODE_B2_DOUBLE, reg);
	
	/* Transfer type */
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_LCH_CTRL);
	outw(LCH_TYPE_D, reg);

	/* Source & Dest params */
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_CSDP);
	outw(SRC_DATA_TYPE_16 | SRC_PACK | SRC_BURST_EN_4 | DST_DATA_TYPE_16 | DST_PACK | DST_BURST_EN_4, reg);
	
	/* Palette. The palette was already filled in tifb_init_palette */
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_SRC_EN_B1);
	outw((PALETTE_SIZE / 2), reg);
	
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_SRC_FN_B1);
	outw(1, reg);
	
	/* Not used */
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_SRC_EI_B1);
	outw(0, reg);
	
	/* Not used */
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_SRC_FI_B1_L);
	outw(0, reg);

	/* Video memory */
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_SRC_EN_B2);
	outw(NUM_XPIXELS, reg);
	
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_SRC_FN_B2);
	outw(NUM_YPIXELS, reg);
	
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_SRC_EI_B2);
	outw(1, reg);
	
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_SRC_FI_B2_L);
	outw(1, reg);
	
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_SRC_FI_B2_U);
	outw(0, reg);
	
	/* Configure DMA addresses */
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_TOP_B1_L);
	outw(((__u32)current_par.p_palette_base & 0xFFFF), reg);
	
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_TOP_B1_U);
	outw((((__u32)current_par.p_palette_base >> 16) & 0xFFFF), reg);
	
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_TOP_B2_L);
	outw(((__u32)current_par.p_screen_base & 0xFFFF), reg);
	
	reg = omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, DMA_LCD_TOP_B2_U);
	outw((((__u32)current_par.p_screen_base >> 16) & 0xFFFF), reg);
	
	mdelay(5);
	
	lcd_dma_enable();
	
	mdelay(5);

        DBG("%s: base=%lx\n", __FUNCTION__, current_par.p_palette_base);
        DBG("%s: end=%lx\n", __FUNCTION__, current_par.p_palette_base + size - 2);
        print_lcd_regs();
}

static void __init tifb_init_registers(void)
{
        unsigned long value;
        DBG("%s\n", __FUNCTION__);

        hardware_enable();

        /*
	 * Setup the LCD Control Register
	 */

        value = 0;
        value &= ~(FDD_MSK << FDD_POS); /* FDD to 16 wait cycles */
        value |= (1 << PXL_GATED_POS);  /* PXL_GATED = 1 */
        value |= (1 << LCDTFT_POS);     /* Active TFT */
        value |= (1 << VSYNC_MASK_POS); /* Unmask VSYNC INT */

	LCD_WRITE(value, TI925_LCD_CONTROL);

        /*
	 * Setup the Timing0 Register
	 */

        value = 0;
	value |= ((NUM_XPIXELS - 1) << PPL_POS);   /* Pixels Per Line */
	value |= ((5 - 1) << HSW_POS);             /* HSW width for LCD is 5-1  */
        value |= ((1 - 1) << HFP_POS);             /* H Front Porch is 1-1 */
        value |= ((1 - 1) << HBP_POS);             /* H Back Porch is 1-1 */

        LCD_WRITE(value, TI925_LCD_TIMING0);

        /*
	 * setup the Timing1 Register
	 */

	value = 0;
        value |= ((NUM_YPIXELS - 1) << LPP_POS);   /* Lines Per Panel */
        value |= ((2 - 1) << VSW_POS);             /* VSW width for the LCD is 2-1 */
        value |= ((12 - 1) << VFP_POS);            /* Front Porch is 12 */
        value |= ((1 - 1) << VBP_POS);             /* Back Porch is 1 */

        LCD_WRITE(value, TI925_LCD_TIMING1);

        /*
	 * Setup the Timing2 Register
	 */

        value = 0;           
        value |= (7 << PCD_POS);         /* Pixel Clock Divisor */
        value |= (0 << ACB_POS);         /* AC Bias */
        value |= (0 << ACBI_POS);        /* AC Bias Pin Transitions per IRQ */
        value |= (1 << IVS_POS);         /* Invert VSYNC */
        value |= (1 << IHS_POS);         /* Inverte HSYNC */
        value |= (1 << IPC_POS);         /* Invert Pixel Clock */
        value |= (0 << IEO_POS);         /* Invert Output Enable */

        LCD_WRITE(value, TI925_LCD_TIMING2);

        /* Setup the subpanel */
        value = 0;
        LCD_WRITE(value, TI925_LCD_SUBPANEL);

        mdelay(5);

        /* Setup DMA */
        tifb_init_dma();
        print_lcd_regs();
}

static void lcd_dma_irq_handler(int irq, void *dummy, struct pt_regs *fp)
{
        unsigned long status = inw(omap730_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, 
                                                         LCD_CHANNEL,
                                                         DMA_LCD_CTRL));

        DBG("LCD : lcd_dma_irq_handler()\n");

        if ( status & LCD_BUS_ERROR_IT_COND )
                printk("LCD DMA bus error\n");

        if ( status & LCD_FRAME_2_IT_COND )
                printk("LCD DMA end of block 2\n");

        if ( status & LCD_FRAME_1_IT_COND )
                printk("LCD DMA end of block 1\n");
}
 
static void lcd_irq_handler(int irq, void *dummy, struct pt_regs *fp)
{
	unsigned long status;
	int reset = 0;
	static int abc = 0;
	static int fuf = 0;
	static int sync = 0;
	static int done = 0;


	DBG("%s\n", __FUNCTION__);
	
	status = LCD_READ(TI925_LCD_STATUS);

//	DBG("Status: %08lx\n", status);
	
	if ( status & LP_INT ) 
        {
		DBG("LCD : Load Palette interrupt\n");
		LCD_WRITE(0, TI925_LCD_STATUS);
	}

	if ( status & FUF_INT )  
        {
		DBG("LCD : FUF interrupt\n");
		reset = 1;
		fuf++;
	}

	if ( status & LINE_INT ) 
        {
		DBG("LCD : LINE interrupt\n");
		LCD_WRITE(0, TI925_LCD_STATUS);
	}
	
	if ( status & ABC_INT ) 
        {
		DBG("LCD : ABC interrupt\n");
		LCD_WRITE(0, TI925_LCD_STATUS);
		abc++;
	}
	
	if ( status & SYNC_LOST_INT ) 
        {
		DBG("LCD : sync lost interrupt\n");
		reset = 1;
		sync++;
	}

	if ( status & VS_INT ) 
        {
		DBG("LCD : VSYNC interrupt\n");
		LCD_WRITE(0, TI925_LCD_STATUS);
	}

	if ( status & DONE_INT ) 
        {
		DBG("DONE interrupt\n");

		reset = 1;
		done++;
	}

	if ( (sync % 1024) == 1023 ) 
        {
		DBG(" abc %d fuf %d sync %d done %d\n",abc, fuf, sync, done);
		print_lcd_regs();     
	}

	if ( reset ) 
        {
		// This will clear the error condition and restart the LCD
		unsigned long control;

		DBG("LCD : Will reset LCD\n");
		control = LCD_READ(TI925_LCD_CONTROL);
		LCD_WRITE(control & ~(1 << LCD_EN_POS), TI925_LCD_CONTROL);
		LCD_WRITE(0, TI925_LCD_STATUS);
		LCD_WRITE(control, TI925_LCD_CONTROL);
	}
}

#endif /* _p2omap_730_ */ 

/*
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
