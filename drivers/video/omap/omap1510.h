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

#include <asm/arch/fpga.h>
#include <asm/arch/ck.h>
#include <asm/arch/dma.h>

#define NUM_XPIXELS 320
#define NUM_YPIXELS 240
#define BPP 16
#define PALETTE_SIZE 32

#define COLOR_16BIT
//#define COLOR_8BIT

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
#undef SRAM_FRAMEBUFFER_MEMORY
#define SRAM_FRAMEBUFFER_MEMORY (OMAP1510_SRAM_BASE + OMAP1510_SRAM_SIZE - MAX_FRAMEBUFFER_SIZE)

// we use this to check for valid displays in set_var
struct _displays 
{
        int xres;
        int yres;
        int bpp;
};
static struct _displays displays_supported[] = { {320, 240, 16} };

static struct fb_var_screeninfo tifb_default8 = 
{
        xres: 320, 
        yres: 240, 
        xres_virtual: 320, 
        yres_virtual: 240, 
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
        xres: 320, 
        yres: 240, 
        xres_virtual: 320, 
        yres_virtual: 240, 
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

static inline void hardware_enable(void)
{
        // initialize LCD
        fpga_write(0x7, OMAP1510P1_FPGA_LCD_PANEL_CONTROL);

        /* 
           According to the errata, if the framebuffer is in SDRAM, the max clock
           is 1/2 the TC clock rate.  If the framebuffer is in SRAM, the max clock is
           = the TC clock rate.
        */
        ck_set_rate(lcd_ck, (ck_get_rate(tc_ck) / 2));
        ck_enable(lcd_ck);
        
}

static inline void hardware_disable(void)
{
        // stop LCD
        fpga_write(0x0, OMAP1510P1_FPGA_LCD_PANEL_CONTROL);
        ck_disable(lcd_ck);        
}

static inline int lcd_active(void)
{
    // Return true if LCD controller is running.
    return(LCD_READ(TI925_LCD_CONTROL) & 1);
}

static inline void lcd_enable(void)
{
        unsigned long value;

        // enable the LCD
        while (((value = LCD_READ(TI925_LCD_CONTROL)) & 0x1) == 0) {
                value |= 1;
                LCD_WRITE(value,TI925_LCD_CONTROL);
        }
}

//static inline void lcd_disable(void)
void lcd_disable(void)
{
        unsigned long value;
        unsigned long delay = jiffies + 10;  // wait max .1 second

        // Clear the AutoIdle bit in the Global DMA control reg to work around the DMA
        // state machine bug in TI's errata doc.  Have to do this to be able to stop and
        // restart the LCD controller reliably.  We'll turn the bit back on in sys_idle.
        omap_dma_autoidle_set(DISABLE_AUTOIDLE);

        // disable the LCD
        value = LCD_READ(TI925_LCD_CONTROL);
        value &= ~1;
        LCD_WRITE(value,TI925_LCD_CONTROL);

        // Wait until current frame finished.
        //  Poll on LCDStatus[Done] bit.
        while (((LCD_READ(TI925_LCD_STATUS) & 0x1) != 1) &&
               (jiffies < delay)) {
                udelay(10);
        }
}

static inline void orl(unsigned long val, unsigned long reg) 
{
        outl((inl(reg) | val),reg);
}

static inline unsigned long omap1510_dma_reg_addr(int global, int chan, int reg) 
{ 
        return (OMAP1510_DMA_BASE + (global << 10) + (chan << 6) + (reg << 1));
}

static inline void lcd_write_top_address(unsigned long addr) 
{
        unsigned long reg;

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, 
                                    DMA_LCD_TOP_F1_U);

        DBG(__FUNCTION__ "\n%lux %lux\n", reg, addr >> 16);

        outw(addr >> 16, reg);

        DBG("wrote %lux %lux\n", reg, inw(reg));

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, 
                                    DMA_LCD_TOP_F1_L);

        DBG("%lux %lux\n", reg, addr & 0xffff);

        outw(addr & 0xffff, reg);

        DBG("wrote %lux %lux\n", reg, inw(reg));
}

static inline void lcd_write_bottom_address(unsigned long addr) 
{
        unsigned long reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                                  LCD_CHANNEL, 
                                                  DMA_LCD_BOT_F1_U);

        DBG(__FUNCTION__ "\n%lux %lux\n", reg, addr >> 16);

        outw(addr >> 16, reg);

        DBG("wrote %lux %lux\n", reg, inw(reg));

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                    LCD_CHANNEL, DMA_LCD_BOT_F1_L);

        DBG("%lux %lux\n", reg, addr&0xffff);

        outw(addr & 0xffff, reg);

        DBG("wrote %lux %lux\n", reg, inw(reg));
}

static inline void lcd_dma_enable(void) 
{
        unsigned long reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                                  LCD_CHANNEL,
                                                  DMA_LCD_CTRL);
        DBG("%lux\n",reg);

        reg = omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,LCD_CHANNEL,
                                    DMA_LCD_CTRL);
#ifdef CONFIG_FB_SDRAM
        outw((LCD_BUS_ERROR_IT_IE),reg);
#else
        outw((LCD_BUS_ERROR_IT_IE | LCD_SOURCE_IMIF),reg);
#endif
        DBG("wrote %lux %lux\n",reg,inl(reg));
}

#ifdef FB_DEBUG 
static inline void print_lcd_regs(void)
{
	DBG(__FUNCTION__ "\n");

	DBG("control: %8.8x status %8.8x timing0 %8.8x" 
	     " timing1 %8.8x timing2 %8.8x\n", 
	     (int)LCD_READ(TI925_LCD_CONTROL), 
	     (int)LCD_READ(TI925_LCD_STATUS),
	     (int)LCD_READ(TI925_LCD_TIMING0),
	     (int)LCD_READ(TI925_LCD_TIMING1),
	     (int)LCD_READ(TI925_LCD_TIMING2));

        DBG("lcd register %lux power %lux fpga %lux\n", inb(0xe8000002),
            inb(0xe8000005), inw(0xe97ffffe));
}
#else
static inline void print_lcd_regs(void) {}
#endif

static void tifb_init_dma(void)
{
        int size = current_par.palette_size + current_par.screen_size;

        lcd_write_top_address((unsigned long) current_par.p_palette_base);
        lcd_write_bottom_address((unsigned long)current_par.p_palette_base + 
                                 size - 2);
        lcd_dma_enable();

        DBG(__FUNCTION__ " size=%d\n", size);
        DBG(__FUNCTION__ " base=%lux\n", current_par.p_palette_base);
        DBG(__FUNCTION__ " end=%lux\n", current_par.p_palette_base + size - 2);
        print_lcd_regs();
}

static void __init tifb_init_registers(void)
{
        unsigned long value;
        DBG(__FUNCTION__ "\n");

        hardware_enable();

        // setup the LCD Control
        value = 0;
        value |= (0<<12); // FDD to 16 wait cycles
        value |= (1<<3);  // turn on the done mask
        value |= (1<<9);
        value |= (1<<7);  // tft

        value = 0xcec;
        LCD_WRITE(value, TI925_LCD_CONTROL);

        // setup the Timing0
        value = 320 -1;         // current_par.xres -1;
        value |= (11<<10);      // HSW width for LCD is 11
        value |= (13<<16);      // H Front Porch is 13
        value |= ((71-12)<<24); // H Back Porch is 71-12 

        value = 0x2301053f;
        LCD_WRITE(value, TI925_LCD_TIMING0);

        // setup the Timing1
        value = 240 - 1;    //current_par.yres - 1;
        value |= (0 << 10); // VSW width for the LCD is 1 (minus 1)
        value |= (1 << 16); // Front Porch not used
        value |= (0 << 24); // Back Porch not used

        value = 0x040408ef;
        LCD_WRITE(value, TI925_LCD_TIMING1);

        // setup the Timing2
        value = 2;           //  PCD 
        value |= (255 << 8); // ACB 
        value |= (0 << 16);  // ACBI this should be disabled
        value |= (0 << 20);  // IVS 
        value |= (0 << 21);  // IHS
        value |= (0 << 22);  // IPC
        value |= (0 << 23);  // IEO

        value = 0x00300005;
        LCD_WRITE(value, TI925_LCD_TIMING2);

        // setup dma
        tifb_init_dma();
        print_lcd_regs();

        lcd_enable();

}

static void lcd_dma_irq_handler(int irq, void *dummy, struct pt_regs *fp)
{
        unsigned long status = inw(omap1510_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                                         LCD_CHANNEL,
                                                         DMA_LCD_CTRL));
        if ( status & (1<<5) )
                printk("LCD DMA bus error");

        if ( status & (1<<4) )
                printk("LCD DMA end of frame 2");

        if ( status & (1<<3) )
                printk("LCD DMA end of frame 1");
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
	
	if ( status & (1<<3) ) 
        {
		DBG("ABC interrupt");

		status &= (1<<3);
		LCD_WRITE(status,TI925_LCD_STATUS);
		abc++;
	}
	
	if ( status & (1<<5) ) 
        {
		DBG("FUF interrupt");

		reset = 1;
		fuf++;
	}

	if ( status & (1<<2) ) 
        {
		DBG("synch interrupt");

		reset = 1;
		sync++;
	}

	if ( status & (1<<0) ) 
        {
		DBG("DONE interrupt");

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
