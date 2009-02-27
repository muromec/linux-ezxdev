/*
 * File: fb_omap710.h
 *
 * This file is the header file for the omap710 framebuffer.
 *
 * Copyright (C) 2002 RidgeRun, Inc. 
 * Author: Alex McMains <aam@ridgerun.com> 2001/09/20
 *         Greg Lonnon <glonnon@ridgerun.com>
 *         Brian Mahaffy <brianm@ridgerun.com> 2002/01/29 ported to omap710
 *
 *             RIDGERUN, INC. PROPRIETARY INFORMATION
 *  Property of RidgeRun, Inc. For Unrestricted Internal Use Only.
 *  Unauthorized reproduction and/or distribution is strictly prohibited.
 *  This product is protected under copyright law and trade secret law
 *  as an unpublished work.
 *
 *  This source is provided for reference only and rights have not been
 *  granted to modify or redistribute.
 *
 * Please report all bugs/problems to the author or <support@dsplinux.net>
 *
 * key: RRPROCR (do not remove)
 *
 */

#ifndef _fb_omap710_h_
#define _fb_omap710_h_

#include <asm/arch/fpga.h>
#include <asm/arch/ck.h>
#include <asm/arch/dma.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#define NUM_XPIXELS 320
#define NUM_YPIXELS 240
#define BPP 8
#define PALETTE_SIZE 512

#define COLOR_8BIT

#define MAX_FRAMEBUFFER_SIZE (((NUM_XPIXELS) * (NUM_YPIXELS) * (BPP)/8) + \
                              (PALETTE_SIZE) + PAGE_SIZE)
#define BPP_MASK 0x3000

// we use this to check for valid displays in set_var
struct _displays 
{
        int xres;
        int yres;
        int bpp;
};
static struct _displays displays_supported[] = { {320, 240, 8} };

static struct fb_var_screeninfo tifb_default8 = 
{
        xres: 320, 
        yres: 240, 
        xres_virtual: 320, 
        yres_virtual: 240, 
        xoffset: 0, 
        yoffset: 0, 
        bits_per_pixel: BPP, 
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

static inline void hardware_enable(void)
{
        DBG(__FUNCTION__ "\n");
        // initialize LCD
        fpga_write(0x2, FPGA_LCD_PANEL_CONTROL);
        fpga_write(0x6, FPGA_LCD_PANEL_CONTROL);
        udelay(100);

        /* 
           According to the errata, if the framebuffer is in SDRAM, the max clock
           is 1/2 the TC clock rate.  If the framebuffer is in SRAM, the max clock is
           = the TC clock rate.
        */
        ck_set_rate(lcd_ck, (ck_get_rate(tc_ck) / 2));
        ck_enable(lcd_ck);
        ck_enable(lbfree_ck);
        
}

static inline void hardware_disable(void)
{
        DBG(__FUNCTION__ "\n");
        // stop LCD
        fpga_write(0x0, FPGA_LCD_PANEL_CONTROL);

        ck_disable(lcd_ck);        
        ck_disable(lbfree_ck);        
}

static inline int lcd_active(void)
{
    // Return true if LCD controller is running.
    return(LCD_READ(OMAP710_LCD_CONTROL) & 1);
}

static inline void lcd_enable(void)
{
        unsigned long value;

        DBG(__FUNCTION__ "\n");
        // enable the LCD
        value = LCD_READ(OMAP710_LCD_CONTROL);
        value |= 1;
        LCD_WRITE(value,OMAP710_LCD_CONTROL);
        fpga_write(0x7, FPGA_LCD_PANEL_CONTROL);
}

static inline void lcd_disable(void)
{
        unsigned long value;

        // Clear the AutoIdle bit in the Global DMA control reg to work around the DMA
        // state machine bug in TI's errata doc.  Have to do this to be able to stop and
        // restart the LCD controller reliably.  We'll turn the bit back on in sys_idle.
        omap_dma_autoidle_set(DISABLE_AUTOIDLE); 

        DBG(__FUNCTION__ "\n");
        // enable the LCD
        value = LCD_READ(OMAP710_LCD_CONTROL);
        value &= ~1;
        LCD_WRITE(value,OMAP710_LCD_CONTROL);
}

static inline void orl(unsigned long val, unsigned long reg) 
{
        DBG(__FUNCTION__ "\n");
        outl((inl(reg) | val),reg);
}

static inline unsigned long omap710_dma_reg_addr(int global, int chan, int reg) 
{ 
        DBG(__FUNCTION__ " global:%8.8x chan:%8.8x reg:%8.8x\n", global, chan, reg);
        return (OMAP710_DMA_BASE + (global << 10) + (chan << 6) + (reg << 1));
}

static inline void lcd_write_top_address(unsigned long addr) 
{
        unsigned long reg;

        DBG(__FUNCTION__ "\n");
        reg = omap710_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, 
                                    DMA_LCD_TOP_F1_U);

        DBG(__FUNCTION__ "\n%lX %lX\n", reg, addr >> 16);

        outw(addr >> 16, reg);

        DBG("wrote %lX %8.8x\n", reg, inw(reg));

        reg = omap710_dma_reg_addr(NO_GLOBAL_DMA_ACCESS, LCD_CHANNEL, 
                                    DMA_LCD_TOP_F1_L);

        DBG("%lX %lX\n", reg, addr & 0xffff);

        outw(addr & 0xffff, reg);

        DBG("wrote %lX %8.8x\n", reg, inw(reg));
}

static inline void lcd_write_bottom_address(unsigned long addr) 
{
        unsigned long reg = omap710_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                                  LCD_CHANNEL, 
                                                  DMA_LCD_BOT_F1_U);

        DBG(__FUNCTION__ "\n%lX %lX\n", reg, addr >> 16);

        outw(addr >> 16, reg);

        DBG("wrote %lX %8.8x\n", reg, inw(reg));

        reg = omap710_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                    LCD_CHANNEL, DMA_LCD_BOT_F1_L);

        DBG("%lX %lX\n", reg, addr&0xffff);

        outw(addr & 0xffff, reg);

        DBG("wrote %lX %8.8x\n", reg, inw(reg));
}

static inline void lcd_dma_enable(void) 
{
        unsigned long reg = omap710_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                                  LCD_CHANNEL,
                                                  DMA_LCD_CTRL);
	    DBG(__FUNCTION__ "\n");
        DBG("%lX\n",reg);

        //reg = omap710_dma_reg_addr(GLOBAL_DMA_ACCESS, 0, 0);
        //outw((1<<2),reg);  // dma is free running ????
        DBG("wrote %lX %lX\n",reg,inl(reg));

        reg = omap710_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,LCD_CHANNEL,
                                    DMA_LCD_CTRL);

#ifdef CONFIG_FB_SDRAM
        outw((LCD_BUS_ERROR_IT_IE),reg);
#else
        outw((LCD_BUS_ERROR_IT_IE | LCD_SOURCE_IMIF),reg);
#endif
        DBG("wrote %lX %lX\n",reg,inl(reg));
}

#ifdef CONFIG_PM
/*
  Power management.
*/

static int tifb_pm_callback(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	DBG(__FUNCTION__ "\n");

	if (rqst == PM_SUSPEND || rqst == PM_RESUME) {
		int state = (int)data;

                switch (state) {
                case 0:
                        hardware_enable();
                        lcd_enable();
                        break;
                default:
			/* Enter D1-D3.  Disable the LCD controller.  */
                        lcd_disable();
                        hardware_disable();
                        break;
                }
	}
	DBG("done\n");
	return 0;        
}
#endif

#ifdef FB_DEBUG 
static inline void print_lcd_regs(void)
{
	DBG(__FUNCTION__ "\n");

	DBG("control: %8.8x status %8.8x\n\ttiming0 %8.8x" 
	     " timing1 %8.8x timing2 %8.8x\n", 
	     (int)LCD_READ(OMAP710_LCD_CONTROL), 
	     (int)LCD_READ(OMAP710_LCD_STATUS),
	     (int)LCD_READ(OMAP710_LCD_TIMING0),
	     (int)LCD_READ(OMAP710_LCD_TIMING1),
	     (int)LCD_READ(OMAP710_LCD_TIMING2));

//        DBG("lcd register %8.8x power %8.8x fpga %8.8x\n", inb(0xe8000002),
//            inb(0xe8000005), inw(0xe97ffffe));
}
#else
static inline void print_lcd_regs(void) {}
#endif

static void tifb_init_dma(void)
{
        int size = current_par.palette_size + current_par.screen_size;
	    DBG(__FUNCTION__ "\n");

        lcd_write_top_address((unsigned long) current_par.p_palette_base);
        lcd_write_bottom_address((unsigned long)current_par.p_palette_base + 
                                 size - 2);
        lcd_dma_enable();

        DBG(__FUNCTION__ " size=%d\n", size);
        DBG(__FUNCTION__ " base=%lX\n", (unsigned long int)current_par.p_palette_base);
        DBG(__FUNCTION__ " end=%lX\n", (unsigned long int)current_par.p_palette_base + size - 2);
        print_lcd_regs();
}

static void __init tifb_init_registers(void)
{
        unsigned long value;
        DBG(__FUNCTION__ "\n");

        hardware_enable();

        // setup the LCD Control
//        value = 0;
//        value |= (0<<12); // FDD to 16 wait cycles
//        value |= (1<<3);  // turn on the done mask
//        value |= (1<<9);
//        value |= (1<<7);  // tft

        value = 0x00000C64;
        value = 0x00010E6C;
        LCD_WRITE(value, OMAP710_LCD_CONTROL);

        // setup the Timing0
//        value = 320 -1;         // current_par.xres -1;
//        value |= (11<<10);      // HSW width for LCD is 11
//        value |= (13<<16);      // H Front Porch is 13
//        value |= ((71-12)<<24); // H Back Porch is 71-12 

        value = 0x314F093F;
        LCD_WRITE(value, OMAP710_LCD_TIMING0);

        // setup the Timing1
//        value = 240 - 1;    //current_par.yres - 1;
//        value |= (0 << 10); // VSW width for the LCD is 1 (minus 1)
//        value |= (1 << 16); // Front Porch not used
//        value |= (0 << 24); // Back Porch not used

        value = 0x000004EF;
        LCD_WRITE(value, OMAP710_LCD_TIMING1);

        // setup the Timing2
//        value = 2;           //  PCD 
//        value |= (255 << 8); // ACB 
//        value |= (0 << 16);  // ACBI this should be disabled
//        value |= (0 << 20);  // IVS 
//        value |= (0 << 21);  // IHS
//        value |= (0 << 22);  // IPC
//        value |= (0 << 23);  // IEO

        value = 0x00000006;
        LCD_WRITE(value, OMAP710_LCD_TIMING2);

        // setup dma
        tifb_init_dma();
        print_lcd_regs();

        lcd_enable();

        // Not sure why this needs to happen, but SteveK and BrianM determined
        // it was needed by trial and error to get the LCD dma to go on the 710.
        // We just turn off the dma clock and then turn it right back on.
        ck_disable(dma_ck);
        ck_enable(dma_ck);

#ifdef CONFIG_PM
        /*
	 * Note that the console registers this as well, but we want to
	 * power down the display prior to sleeping.
	 */
	pm_register(PM_SYS_DEV, PM_SYS_VGA, tifb_pm_callback);
#endif
}

static void lcd_dma_irq_handler(int irq, void *dummy, struct pt_regs *fp)
{
        unsigned long status = inw(omap710_dma_reg_addr(NO_GLOBAL_DMA_ACCESS,
                                                         LCD_CHANNEL,
                                                         DMA_LCD_CTRL));
	    DBG(__FUNCTION__ "\n");
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
    int missed_int_bug = 0;

	status = LCD_READ(OMAP710_LCD_STATUS);

//	DBG(__FUNCTION__ " status:%8.8x\n",status);
	
	if ( status & (1<<3) ) 
        {
		DBG("ABC interrupt");

        missed_int_bug = 0;
		status &= (1<<3);
		LCD_WRITE(status,OMAP710_LCD_STATUS);
		abc++;
	}
	
	if ( status & (1<<5) ) 
        {
		DBG("FUF interrupt");

        missed_int_bug = 0;
		reset = 1;
		fuf++;
	}

	if ( status & (1<<2) ) 
        {
		DBG("synch interrupt");

        missed_int_bug = 0;
		reset = 1;
		sync++;
	}

	if ( status & (1<<0) ) 
        {
		DBG("DONE interrupt");

        missed_int_bug = 0;
		reset = 1;
		done++;
	}

	if ( (sync % 1024) == 0 ) 
        {
		DBG(" abc %d fuf %d sync %d done %d\n",abc, fuf, sync, done);
		print_lcd_regs();     
	}


	if ( reset || missed_int_bug ) 
        {
		// This will clear the error condition and restart the LCD
		unsigned long control;

		control = inl(IO_ADDRESS(OMAP710_LCD_CONTROL));
		control &= ~(1<<0);
		outl(control, IO_ADDRESS(OMAP710_LCD_CONTROL));
		control = inl(IO_ADDRESS(OMAP710_LCD_CONTROL));
		control |= (1<<0);
		outl(control, IO_ADDRESS(OMAP710_LCD_CONTROL));
	}
}

#endif /* _fb_omap710_h_ */ 

/*
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
