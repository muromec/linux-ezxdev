/*
 * File: fb_omap1510.h
 *
 * This file is the header file for the omap1510 framebuffer.
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
 * key: RRGPLCR (do not remove)
 *
 */

#ifndef _fb_omap1510_h_
#define _fb_omap1510_h_

#include <asm/hardware.h>
#include <asm/arch/ck.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#define NUM_XPIXELS 240
#define NUM_YPIXELS 320

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


#ifdef CONFIG_OMAP_H2

#define PAGE2_AUDIO_CODEC_REGISTERS  2 << 11
#define LEAVE_CS 0x80

extern u16 omap1610_uwire_data_transfer(u8 cs, u16 data, u8 trans_size, u8 rec_size);

static inline void omap1610_tsc2101_write(u8 address, u16 data) 
{
        /* CS = 1, 16 bit transfer, write-only */

	outl(inl(FUNC_MUX_CTRL_7)|(2<<18), FUNC_MUX_CTRL_7); //Configure N15 pin to be uWire CS1
	omap1610_uwire_cs1_configure_mode(1,1,0,2,0);
	omap1610_uwire_data_transfer(LEAVE_CS | 1, (PAGE2_AUDIO_CODEC_REGISTERS | (address << 5)), 16, 0);
        omap1610_uwire_data_transfer(1, data, 16, 0);
}

static inline void hardware_enable(void)
{
        // initialize LCD
	omap1610_tsc2101_write(0x23, 0xCC00);
#ifdef CONFIG_FB_SDRAM	
        ck_set_rate(lcd_ck, (ck_get_rate(tc_ck) / 3));
#else
        ck_set_rate(lcd_ck, (ck_get_rate(tc_ck) / 2));
#endif

        ck_enable(lcd_ck);
        
}

static inline void hardware_disable(void)
{
        // stop LCD
	omap1610_tsc2101_write(0x23, 0x0000);
        ck_disable(lcd_ck);        
}

#else
static inline void hardware_enable(void)
{
        // initialize LCD
        outw(inw(GPIO_DIRECTION_REG)&(~0xe000),GPIO_DIRECTION_REG);
        outw(0xe000,GPIO_SET_DATAOUT_REG);

        ck_set_rate(lcd_ck, (ck_get_rate(tc_ck) / 2));
        ck_enable(lcd_ck);
        
}

static inline void hardware_disable(void)
{
        // stop LCD
        outw(inw(GPIO_DIRECTION_REG)|0xe000,GPIO_DIRECTION_REG);
        ck_disable(lcd_ck);        
}
#endif
static inline int lcd_active(void)
{
    // Return true if LCD controller is running.
    return(LCD_READ(TI925_LCD_CONTROL) & 1);
}

static inline void lcd_enable(void)
{
        unsigned long value;

        // enable the LCD
        value = LCD_READ(TI925_LCD_CONTROL);
        value |= 1;
        LCD_WRITE(value,TI925_LCD_CONTROL);
}

static inline void lcd_disable(void)
{
        unsigned long value;

        // enable the LCD
        value = LCD_READ(TI925_LCD_CONTROL);
        value &= ~1;
        LCD_WRITE(value,TI925_LCD_CONTROL);

        // Wait until current frame finished.
        //  Poll on LCDStatus[Done] bit.
        for(value=1000; value && (!(LCD_READ(TI925_LCD_STATUS) & 0x1)); value--)
		udelay(100);
}

static inline void orl(unsigned long val, unsigned long reg) 
{
        outl((inl(reg) | val),reg);
}

static inline unsigned long omap1510_dma_reg_addr(int global, int chan, int reg) 
{ 
        return (OMAP1610_DMA_BASE + (global << 10) + (chan << 6) + (reg << 1));
}

static inline void lcd_write_top_address(unsigned long addr) 
{
        unsigned long reg;

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, 
                                    DMA_LCD_TOP_B1_U);

        DBG(__FUNCTION__ "\n%lx %lx\n", reg, addr >> 16);

        outw(addr >> 16, reg);

        DBG("wrote %lx %x\n", reg, inw(reg));

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, 
                                    DMA_LCD_TOP_B1_L);

        DBG("%lx %lx\n", reg, addr & 0xffff);

        outw(addr & 0xffff, reg);

        DBG("wrote %lx %x\n", reg, inw(reg));
}

static inline void lcd_write_bottom_address(unsigned long addr) 
{
        unsigned long reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                                  LCD_CHANNEL, 
                                                  DMA_LCD_BOT_B1_U);

        DBG(__FUNCTION__ "\n%lx %lx\n", reg, addr >> 16);

        outw(addr >> 16, reg);

        DBG("wrote %lx %x\n", reg, inw(reg));

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                    LCD_CHANNEL, DMA_LCD_BOT_B1_L);

        DBG("%lx %lx\n", reg, addr&0xffff);

        outw(addr & 0xffff, reg);

        DBG("wrote %lx %x\n", reg, inw(reg));
}

static inline void lcd_dma_enable(void) 
{
        unsigned long reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                                  LCD_CHANNEL,
                                                  DMA_LCD_CTRL);
        DBG("%lx\n",reg);

#if 0
        /*
          Nope.  The LCD doesn't get to set global DMA state.  SKJ
        */
        reg = omap1510_dma_reg_addr(GLOBAL_DMA_ACCESS, 0, 0);
        outw((1<<2),reg);  // dma is free running ????
        DBG("wrote %lx %lx\n",reg,inl(reg));
#endif


        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,LCD_CHANNEL,
                                    DMA_LCD_CCR);
        outw(0x5700,reg);
        DBG("wrote %lx %x\n",reg,(int)inw(reg));

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,LCD_CHANNEL,
                                    DMA_LCD_CSDP);
        outw(0x142,reg);
        DBG("wrote %lx %x\n",reg,(int)inw(reg));

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,LCD_CHANNEL,
                                    DMA_LCD_CTRL);
#ifdef CONFIG_FB_SDRAM
        outw((LCD_BUS_ERROR_IT_IE),reg);
#else
        outw((LCD_BUS_ERROR_IT_IE | LCD_SOURCE_IMIF),reg);
#endif

        DBG("wrote %lx %x\n",reg,(int)inw(reg));

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,LCD_CHANNEL,
                                    DMA_LCD_SRC_EN_B1);
        outw(38408,reg);
        DBG("wrote %lx %lx\n",reg,(int)inw(reg));

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,LCD_CHANNEL,
                                    DMA_LCD_SRC_FN_B1);
        outw(1,reg);
        DBG("wrote %lx %lx\n",reg,(int)inw(reg));

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,LCD_CHANNEL,
                                    DMA_LCD_LCH_CTRL);
        outw(0x4,reg);
        DBG("wrote %lx %lx\n",reg,(int)inw(reg));
}

#ifdef FB_DEBUG 
static inline void print_lcd_regs(void)
{
	DBG(__FUNCTION__ "\n");

	DBG("control: %8.8x status %8.8x timing0 %8.8x" 
	     " timing1 %8.8x timing2 %8.8x line %4d\n", 
	     (int)LCD_READ(TI925_LCD_CONTROL), 
	     (int)LCD_READ(TI925_LCD_STATUS),
	     (int)LCD_READ(TI925_LCD_TIMING0),
	     (int)LCD_READ(TI925_LCD_TIMING1),
	     (int)LCD_READ(TI925_LCD_TIMING2),
	     (int)LCD_READ(LCD_DISPLAYSTATUS)&0x3FF);

}
#else
static inline void print_lcd_regs(void) {}
#endif

static void tifb_init_dma(void)
{
        int size = current_par.palette_size + current_par.screen_size;

        lcd_dma_enable();
        lcd_write_top_address((unsigned long) current_par.p_palette_base);
        lcd_write_bottom_address((unsigned long)current_par.p_palette_base + 
                                 size - 2);

        DBG(__FUNCTION__ " size=%d\n", size);
        DBG(__FUNCTION__ " base=%lx\n", current_par.p_palette_base);
        DBG(__FUNCTION__ " end=%lx\n", current_par.p_palette_base + size - 2);
        print_lcd_regs();
}

static void __init tifb_init_registers(void)
{
        unsigned long value;
        DBG(__FUNCTION__ "\n");

        hardware_enable();

        // setup the LCD Control
        // value = 0;
        // value |= (0<<12); // FDD to 16 wait cycles
        // value |= (1<<3);  // turn on the done mask
        // value |= (1<<9);
        // value |= (1<<7);  // tft

        value = 0x88;
        LCD_WRITE(value, TI925_LCD_CONTROL);

        // setup the Timing0
        value = NUM_XPIXELS -1; // current_par.xres -1;
        value |= ((4-1)<<10);   // HSW width for LCD is 4-1
        value |= ((4-1)<<16);   // H Front Porch is 4-1
        value |= ((40-1)<<24);  // H Back Porch is 40-1

        //value = 0x27030cef;  This is what the above should equal
        LCD_WRITE(value, TI925_LCD_TIMING0);

        // setup the Timing1
        value = NUM_YPIXELS -1; //current_par.yres - 1;
        value |= (2 << 10);     // VSW width for the LCD is 2
        value |= (2 << 16);     // Front Porch is 2
        value |= (8 << 24);     // Back Porch is 8

        //value = 0x0802093f;  This is what the above should equal
        LCD_WRITE(value, TI925_LCD_TIMING1);

        // setup the Timing2
        //value = 2;           //  PCD 
        //value |= (255 << 8); // ACB 
        //value |= (0 << 16);  // ACBI this should be disabled
        //value |= (0 << 20);  // IVS 
        //value |= (0 << 21);  // IHS
        //value |= (0 << 22);  // IPC
        //value |= (0 << 23);  // IEO

        value = 0x00300008;
        LCD_WRITE(value, TI925_LCD_TIMING2);

        // setup dma
        tifb_init_dma();
        print_lcd_regs();

	/* REVISIT: This is req'd to get LCD DMA to free run. */
        ck_disable(dma_ck);

//!!!!!        lcd_enable();
}

static void lcd_dma_irq_handler(int irq, void *dummy, struct pt_regs *fp)
{
        unsigned long status = inw(omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                                         LCD_CHANNEL,
                                                         DMA_LCD_CTRL));
        if ( status & (1<<5) )
                printk("LCD DMA bus error\n");

        if ( status & (1<<4) )
                printk("LCD DMA end of block 2\n");

        if ( status & (1<<3) )
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

	DBG(__FUNCTION__ "\n");
	
	status = LCD_READ(TI925_LCD_STATUS);
	
//	DBG("Status: %08lx\n", status);

	if ( status & (1<<3) ) 
        {
		DBG("ABC interrupt\n");

		status &= (1<<3);
		LCD_WRITE(status,TI925_LCD_STATUS);
		abc++;
	}
	
	if ( status & (1<<5) ) 
        {
		DBG("FUF interrupt\n");

		reset = 1;
		fuf++;
	}

	if ( status & (1<<2) ) 
        {
		DBG("synch interrupt\n");

		reset = 1;
		sync++;
		print_lcd_regs();     
	}

	if ( status & (1<<0) ) 
        {
		DBG("DONE interrupt\n");

		reset = 1;
		done++;
	}

	if ( (sync % 1024) == 0 ) 
        {
		DBG(" abc %d fuf %d sync %d done %d\n",abc, fuf, sync, done);
		print_lcd_regs();     
	}

	if ( reset ) 
        {
		// This will clear the error condition and restart the LCD
		unsigned long control;

		control = inl(IO_ADDRESS(TI925_LCD_CONTROL));
		control &= ~(1<<0);
		outl(control, IO_ADDRESS(TI925_LCD_CONTROL));
		control = inl(IO_ADDRESS(TI925_LCD_CONTROL));
		control |= (1<<0);
		outl(control, IO_ADDRESS(TI925_LCD_CONTROL));
	}
}

#endif /* _fb_omap1510_h_ */ 

/*
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
