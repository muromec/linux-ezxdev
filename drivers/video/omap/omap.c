/*
 * File: fb.c
 *
 * The frame buffer driver for TI arm9 boards: ti925, omap710, omap1510/1610.
 *
 * Copyright (C) 2001, 2002 RidgeRun, Inc. 
 * Author: Alex McMains <aam@ridgerun.com> 2001/05/10
 * Copyright (C) 2004 MontaVista Software, Inc.
 *  <source@mvista.com>.
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

#include "omap.h"

static struct tifb_par current_par;

#include <linux/config.h>
#include <video/fbcon.h>
#include <video/fbcon-mfb.h>
#include <video/fbcon-cfb4.h>
#include <video/fbcon-cfb8.h>
#include <video/fbcon-cfb16.h>

#undef CONFIG_CEE
#define CONFIG_CEE

#ifdef CONFIG_ARCH_TI925
#include "ti925.h"
#elif defined(CONFIG_ARCH_OMAP710)
#include "omap710.h"
#elif defined(CONFIG_ARCH_OMAP1510)
#if defined(CONFIG_OMAP_INNOVATOR)
#include "innovator.h"
#else
#include "omap1510.h"
#endif
#elif defined(CONFIG_ARCH_OMAP1610)
#include "innovator-1610.h"
#elif defined(CONFIG_ARCH_OMAP730)
#include "perseus2-730.h"
#else
#error "ARCHITECTURE NOT SUPPORTED"
#endif

// NOTE: DEBUG defined in omapfb.h

static u_long tifb_memory;
static struct display disp;
static struct fb_info tifb_info;

#if defined(FBCON_HAS_CFB16) || defined(FBCON_HAS_CFB24) || defined(FBCON_HAS_CFB32)
static union {
#ifdef FBCON_HAS_CFB16
    u16 cfb16[16];
#endif
#ifdef FBCON_HAS_CFB24
    u32 cfb24[16];
#endif
#ifdef FBCON_HAS_CFB32
    u32 cfb32[16];
#endif
} fbcon_cmap;
#endif

static const char desc[]   = "RR TI ARM9 Framebuffer Driver";
static const char author[] = "Copyright (C) 2001 RidgeRun, Inc.";

static int tifb_enable = 1;
static int currcon     = 0;

/********************************
 Standard frame buffer interface
*********************************/

int __init tifb_setup(char *);
static int tifb_get_fix(struct fb_fix_screeninfo *fix, int con, 
                        struct fb_info *info);
static int tifb_get_var(struct fb_var_screeninfo *var, int con,
                        struct fb_info *info);
static int tifb_set_var(struct fb_var_screeninfo *var, int con,
                        struct fb_info *info);
static int tifb_get_cmap(struct fb_cmap *cmap, int kspc, int con,
                         struct fb_info *info);
static int tifb_set_cmap(struct fb_cmap *cmap, int kspc, int con,
                         struct fb_info *info);
static int tifb_pan_display(struct fb_var_screeninfo *var, int con,
                            struct fb_info *info);

/*****************************************
 Interface to the low level console driver
 *****************************************/

static int __init tifb_init(void);
static int tifbcon_switch(int con, struct fb_info *info);
static void tifbcon_blank(int blank, struct fb_info *info);

/*****************
 Internal routines
 *****************/

static void tifb_encode_fix(struct fb_fix_screeninfo *fix,
                            struct fb_var_screeninfo *var);
static int tifb_getcolreg(u_int regno, u_int *red, u_int *green, u_int *blue,
                          u_int *transp, struct fb_info *info);
static int tifb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
                          u_int transp, struct fb_info *info);
static void do_install_cmap(int con, struct fb_info *info);
static void tifb_init_palette(void);
static void tifb_init_palette_color(void);
static void tifb_init_registers(void);
static void tifb_init_dma(void);
static int tifb_map_video_memory(void);

static struct fb_ops tifb_ops =
{
        owner:          THIS_MODULE,
        fb_get_fix:     tifb_get_fix,
        fb_get_var:     tifb_get_var,
        fb_set_var:     tifb_set_var,
        fb_get_cmap:    tifb_get_cmap,
        fb_set_cmap:    tifb_set_cmap,
        fb_pan_display: tifb_pan_display,
};

#ifdef CONFIG_CEE /* MVL-CEE */
#include <linux/device.h>

static int tifbcon_suspend(struct device * dev, u32 state, u32 level);
static int tifbcon_resume(struct device * dev, u32 level);

static struct device_driver tifbcon_driver = {
	name:		"omapfb",
	devclass:	NULL,
	probe:		NULL,
	suspend:	tifbcon_suspend,
	resume:		tifbcon_resume,
	remove:		NULL,
};

static struct device tifbcon_device = {
	name:		"OMAP LCD controller",
	bus_id:		"lcd",
	driver:		NULL,
	power_state:	DPM_POWER_ON,
};


static void tifbcon_ldm_register(void)
{
	extern void mpu_private_register(struct device *device, 
					 struct device_driver *driver);

	mpu_private_register(&tifbcon_device, &tifbcon_driver);
	return;
}
#endif

/************************************
 tifb_get_fix()
 Purpose: 
   Get the fixed part of the display
 *************************************/
static int tifb_get_fix(struct fb_fix_screeninfo *fix, int con, 
                        struct fb_info *info)
{
        struct fb_var_screeninfo *var;
        
        if ( con == -1 )
        {
#ifdef COLOR_8BIT
                var = &tifb_default8;
#elif defined(COLOR_16BIT)
                var = &tifb_default16;
#else
#error "COLOR DEPTH NOT SUPPORTED" 
#endif
        }
        else
                var = &fb_display[con].var;

        tifb_encode_fix(fix, var);
        
        return 0;
}

/******************************************
 tifb_get_var()
 Purpose: 
   Get the user defined part of the display
 *******************************************/
static int tifb_get_var(struct fb_var_screeninfo *var, int con, 
                        struct fb_info *info)
{
        if ( con == -1 )
        {
#ifdef COLOR_8BIT
                *var = tifb_default8;
#elif defined(COLOR_16BIT)
                *var = tifb_default16;
#else
#error "COLOR DEPTH NOT SUPPORTED"
#endif
        }
        else
                *var = fb_display[con].var;
        
        return 0;
}

/******************************************
 tifb_set_var()
 Purpose: 
   Set the user defined part of the display
 *******************************************/
static int tifb_set_var(struct fb_var_screeninfo *var, int con, 
                        struct fb_info *info)
{
        struct display *display;
        int i;
        int ok_display;

        // make sure we support the new display
        ok_display = 0;
        for ( i=0; i < (sizeof(displays_supported)/sizeof(struct _displays)); i++ )
        {
                if ( (var->xres == displays_supported[i].xres) &&
                     (var->yres == displays_supported[i].yres) &&
                     (var->bits_per_pixel == displays_supported[i].bpp) )
                {
                        ok_display = 1;
                        break;
                }
        }

        if ( ! ok_display )
                return -EINVAL;
        
	if ( con >= 0 )
		display = &fb_display[con];
	else
		display = info->disp;	/* used during initialization */

	/* update the display interface */
	display->var            = *var;
	display->screen_base	= current_par.v_screen_base;

#ifdef COLOR_8BIT
	display->visual	        = FB_VISUAL_PSEUDOCOLOR;
#elif defined(COLOR_16BIT)
        display->visual         = FB_VISUAL_TRUECOLOR; 
#else
#error "COLOR DEPTH NOT SUPPORTED"
#endif

	display->type		= FB_TYPE_PACKED_PIXELS;
	display->type_aux	= 0;
	display->ypanstep	= 0;
	display->ywrapstep	= 0;
	display->line_length	= (var->xres_virtual * var->bits_per_pixel) 
                / 8; 
        display->next_line      = (var->xres_virtual * var->bits_per_pixel) 
                / 8;
	display->can_soft_blank = 1;
	display->inverse	= 0;

	DBG(__FUNCTION__ " display->bpp = %d\n", display->var.bits_per_pixel);
  
	switch ( display->var.bits_per_pixel ) 
        {
#ifdef FBCON_HAS_CFB4
	case 4:
		display->dispsw = &fbcon_cfb4;
		break;
#endif
#ifdef FBCON_HAS_CFB8
	case 8: 
		DBG(__FUNCTION__ " &fbcon_cfb8 = %lX\n", (unsigned long int)&fbcon_cfb8);
		DBG(__FUNCTION__ " dispsw pre-set\n");
		display->dispsw = &fbcon_cfb8;
		DBG(__FUNCTION__ " dispsw set\n");
		DBG(__FUNCTION__ " dispsw = %lX\n", (unsigned long int)display->dispsw);
        if (display->dispsw) {
		DBG(__FUNCTION__ " fontwidthmask = %X\n",
                     display->dispsw->fontwidthmask);
        }
		break;
#endif
#ifdef FBCON_HAS_CFB16
	case 16:
		display->dispsw = &fbcon_cfb16;
                display->dispsw_data = fbcon_cmap.cfb16;
		break;
#endif
	default:
		DBG((__FUNCTION__ " arghh! No display that supports"
                                  " fonts!\n"));
		display->dispsw = &fbcon_dummy;
		break;
	}

	{
		struct fb_cmap *cmap;
		cmap = fb_default_cmap(current_par.palette_size);
		fb_set_cmap(cmap, 1, tifb_setcolreg, info);
	}
  
	return 0;
}

/****************************
 tifb_get_cmap()
 Purpose: 
   Get the display's colormap
*****************************/
static int tifb_get_cmap(struct fb_cmap *cmap, int kspc, int con,
                         struct fb_info *info)
{
	if ( (!info->display_fg) || (con == info->display_fg->vc_num) ) 
        { /* current console? */
		return fb_get_cmap(cmap, kspc, tifb_getcolreg, info);
        }
	else if ( fb_display[con].cmap.len ) /* non default colormap? */
        {
		fb_copy_cmap(&fb_display[con].cmap, cmap, kspc ? 0 : 2);
        }
	else 
        {
		int size = fb_display[con].var.bits_per_pixel == 16 ? 32 : 256;
		fb_copy_cmap(fb_default_cmap(size), cmap, kspc ? 0 : 2);
	}

	return 0;
}

/****************************
 tifb_set_cmap()
 Purpose: 
   Set the display's colormap
*****************************/
static int tifb_set_cmap(struct fb_cmap *cmap, int kspc, int con,
                         struct fb_info *info)
{
        int err;
	struct display *disp;

	if ( con >= 0 )
		disp = &fb_display[con];
	else
		disp = info->disp;

	if ( !disp->cmap.len ) 
        {	/* no colormap allocated? */
		int size = disp->var.bits_per_pixel == 16 ? 32 : 256;

		if ((err = fb_alloc_cmap(&disp->cmap, size, 0)))
			return err;
	}

	if ( !info->display_fg || con == info->display_fg->vc_num )
		return fb_set_cmap(cmap, kspc, tifb_setcolreg, info);
	else
		fb_copy_cmap(cmap, &disp->cmap, kspc ? 0 : 1);
	return 0;
}

/***************************
 tifb_pan_display()
 Purpose: 
   Pan or wrap the display
 ***************************/
static int tifb_pan_display(struct fb_var_screeninfo *var, int con,
                            struct fb_info *info)
{
        return 0;
}

/***************************
 tifb_encode_fix()
 Purpose: 
   
 ***************************/
static void tifb_encode_fix(struct fb_fix_screeninfo *fix,
                            struct fb_var_screeninfo *var)
{
        memset(fix, 0, sizeof(struct fb_fix_screeninfo));
        strcpy(fix->id, desc);
        
        fix->smem_start  = (unsigned long) current_par.p_screen_base;
        fix->smem_len    = NUM_XPIXELS * NUM_YPIXELS * (var->bits_per_pixel / 8); //AAMtifb_memorysize;
        fix->type        = FB_TYPE_PACKED_PIXELS;
        fix->type_aux    = 0;
#ifdef COLOR_8BIT
        fix->visual      = FB_VISUAL_PSEUDOCOLOR;
#elif defined(COLOR_16BIT)
        fix->visual      = FB_VISUAL_TRUECOLOR;
#else 
#error "COLOR DEPTH NOT SUPPORTED"
#endif
        fix->ywrapstep   = 1;
        fix->xpanstep    = 1;
        fix->ypanstep    = 1;
        fix->line_length = (var->xres_virtual * var->bits_per_pixel) / 8;
        fix->accel       = FB_ACCEL_NONE;
}

int __init tifb_setup(char *options) 
{
	printk(KERN_INFO "Error: TIFB DMA options: %s\n", options);
	return 0;
}

static int tifbcon_switch(int con, struct fb_info *info)
{
	/* Do we have to save the colormap? */
	if ( fb_display[currcon].cmap.len )
		fb_get_cmap(&fb_display[currcon].cmap, 1, tifb_getcolreg, 
                            info);

        currcon = con;

	/* Install new colormap */
	do_install_cmap(con, info);

	return 0;
}

#ifdef CONFIG_CEE /* MVL-CEE */
static int tifbcon_suspend(struct device * dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
                lcd_disable();
                hardware_disable();
		break;
        }
	
	return 0;
}

static int tifbcon_resume(struct device * dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
                hardware_enable();
                lcd_enable();
		break;
        }
	
	return 0;
}
#endif /* MVL-CEE */

/*  Blank the display. */
static void tifbcon_blank(int blank, struct fb_info *info)
{
	DBG((__FUNCTION__ "\n"));

#ifdef CONFIG_CEE /* MVL-CEE */
	if (blank)
		device_powerdown(&tifbcon_device);
	else
		device_powerup(&tifbcon_device);
#else /* MVL-CEE */
        if (blank) {       // turn off 
                lcd_disable();
                hardware_disable();
        }
        else {             // turn back on
                hardware_enable();
                lcd_enable();
        }
#endif /* MVL-CEE */
}


/*
 *  Read a single color register and split it into
 *  colors/transparent. Return != 0 for invalid regno.
 */
static int tifb_getcolreg(u_int regno, u_int *red, u_int *green, u_int *blue,
                          u_int *transp, struct fb_info *info)
{
	u_short pal_entry;
	unsigned short *palette = (unsigned short *)current_par.v_palette_base;

	if ( regno > (PALETTE_SIZE/2 - 1) )
		return 1;

	pal_entry = palette[regno];

	*red    = (pal_entry & 0x000f) << 12;
	*green  = (pal_entry & 0x00f0) << 8;
	*blue   = (pal_entry & 0x0f00) << 4;
	*transp = 0;

	return 0;
}

/*
 *  Set a single color register. The values supplied are already
 *  rounded down to the hardware's capabilities (according to the
 *  entries in the var structure). Return != 0 for invalid regno.
 */

static int tifb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
                          u_int transp, struct fb_info *info)
{

	u_short pal;
	unsigned short *palette = (unsigned short *)current_par.v_palette_base;

	if ( regno > (PALETTE_SIZE/2 - 1) )
		return 1;

	pal   = ((red   >>  4) & 0xf00);
	pal  |= ((green >>  8) & 0x0f0);
	pal  |= ((blue  >> 12) & 0x00f);
        
	palette[regno] = pal;

	palette[0] |= BPP_MASK;  // encode the bpp in the first palette entry

        switch ( current_par.bpp ) 
        {
        case 8:
                break;
#ifdef FBCON_HAS_CFB16
        case 16:
                fbcon_cmap.cfb16[regno] = pal;
                break;
#endif
        default:
                printk(KERN_WARNING "unsupported video bpp %d\n",current_par.bpp);
                break;
        }

	return 0;
}

static void do_install_cmap(int con, struct fb_info *info)
{

	DBG((__FUNCTION__ "\n"));

	if ( con != currcon )
		return;

	if ( fb_display[con].cmap.len )
		fb_set_cmap(&fb_display[con].cmap, 1, tifb_setcolreg, info);
	else
		fb_set_cmap(
                        fb_default_cmap(1<<fb_display[con].var.bits_per_pixel),
                        1, tifb_setcolreg, info);
}
         
static void tifb_init_palette()
{
        tifb_init_palette_color();
}

static void tifb_init_palette_color()
{
	unsigned short *palette = (unsigned short *)current_par.v_palette_base;
	
	memset(palette, 0, current_par.palette_size);
 
	// setup the bpp
        palette[0] |= BPP_MASK;

//	tifb_init_dma();
}

static int tifb_map_video_memory(void)
{
	struct page *page;

	/* set reserved flag for fb memory to allow it to be remapped into
	 * user space by the fbmem driver 
	 */
	for ( page = virt_to_page(current_par.v_screen_base); 
              page < virt_to_page(current_par.v_screen_base + 
                                  current_par.screen_size); 
              page++) 
        {
		mem_map_reserve(page);
	}

	/* REVISIT: remap the fb memory to a non-buffered, non-cached region */

	memset(current_par.v_screen_base, 0, current_par.screen_size); 

	return 1;
}

/******************************
 tifb_init()
 Purpose: 
   Initialize the frame buffer
 ******************************/
static __init int tifb_init(void)
{
        char *buf;

        if ( !tifb_enable )
                return -ENXIO;

        printk(KERN_INFO "DSPLinux FB (c) 2001 RidgeRun, Inc.\n");

#ifdef CONFIG_CEE /* MVL-CEE */
	tifbcon_ldm_register();
#endif

        if (lcd_active())  // If the LCD is already running (RRload Image), we need to stop it.
        {
            lcd_disable(); // Call the disble routine to stop stuff.
        }

#if CONFIG_FB_SDRAM
        tifb_memory = __get_dma_pages(GFP_KERNEL,
                                      get_order(MAX_FRAMEBUFFER_SIZE));

#else
        tifb_memory = (char *)SRAM_FRAMEBUFFER_MEMORY;
#endif

        buf = (char *)tifb_memory;
        if ( buf == NULL) 
        {
                printk(KERN_INFO "Error: unable to allocate memory for framebuffer");
                return -EINVAL;
        }
        buf += PAGE_SIZE;  // the start of the framebuffer, not the palette

        DBG(__FUNCTION__ " v_screen_base:%lX\n",(unsigned long int)buf);

        // initialize fbinfo
        strcpy(tifb_info.modename, desc);
        strcpy(tifb_info.fontname, "Acorn8x8");
        tifb_info.node       = -1;
        tifb_info.flags      = FBINFO_FLAG_DEFAULT;
        tifb_info.fbops      = &tifb_ops;
        tifb_info.disp       = &disp;
        tifb_info.switch_con = &tifbcon_switch;
        tifb_info.blank      = &tifbcon_blank;
        tifb_info.changevar  = NULL;

        // initialize current_par
#ifdef COLOR_8BIT
        current_par.bpp            = 8;
#elif defined(COLOR_16BIT)
        current_par.bpp            = 16;
#else
#error "COLOR DEPTH NOT SUPPORTED"
#endif
        current_par.screen_size    = (NUM_XPIXELS * NUM_YPIXELS * 
                                      current_par.bpp) / 8;
        current_par.palette_size   = PALETTE_SIZE;
	current_par.p_palette_base = 
                (unsigned char *) virt_to_phys(buf - PALETTE_SIZE);
	current_par.v_palette_base = (buf - PALETTE_SIZE);
	current_par.p_screen_base  = (unsigned char *) virt_to_phys(buf);
	current_par.v_screen_base  = buf;
	current_par.currcon	   = -1;

        // initialize hardware
        tifb_init_palette();
        tifb_init_registers();

        tifb_map_video_memory();
        // setup irq handlers
        if ( request_irq(INT_LCD_CTRL, lcd_irq_handler, 0, 
                         "console/lcd", 0) < 0)
        {
                printk(KERN_INFO "Error: unable to get LCD CTRL irq\n");
		goto free_mem;
        }

#ifndef CONFIG_ARCH_TI925
        if ( request_irq(INT_DMA_LCD, lcd_dma_irq_handler, 0,
                         "console/lcddma", 0) < 0)
        {
                printk(KERN_INFO "Error: unable to get LCD DMA irq\n");
                goto free_lcd_irq;
        }
#endif

        // initialize var_screeninfo
#ifdef COLOR_8BIT
        tifb_set_var(&tifb_default8, -1, &tifb_info);
#elif defined(COLOR_16BIT)
        tifb_set_var(&tifb_default16, -1, &tifb_info);
#else
#error "COLOR DEPTH NOT SUPPORTED"
#endif

        // register frame buffer
        if ( register_framebuffer(&tifb_info) < 0 )
        {
                printk(KERN_INFO "Error: register_framebuffer failed\n");
                goto free_irq;
        }
        
#if defined(CONFIG_ARCH_OMAP1610) || defined(CONFIG_ARCH_OMAP730)
/*!!!!!*/lcd_enable();
#endif
        return 0;

 free_irq:
#ifndef CONFIG_ARCH_TI925
        free_irq(INT_DMA_LCD, 0);
#endif
 free_lcd_irq:
        free_irq(INT_LCD_CTRL, 0);

 free_mem:
#if CONFIG_FB_SDRAM
        free_pages(tifb_memory, get_order(MAX_FRAMEBUFFER_SIZE)); 
#endif
        return -EINVAL;
}

static void __exit tifb_cleanup(void)
{
        unregister_framebuffer(&tifb_info);
        
#if CONFIG_FB_SDRAM
        free_pages(tifb_memory, get_order(MAX_FRAMEBUFFER_SIZE));
#endif
        free_irq(INT_LCD_CTRL, 0);

#ifndef CONFIG_ARCH_TI925
        free_irq(INT_DMA_LCD, 0);
#endif
}

module_init(tifb_init);
module_exit(tifb_cleanup);

/*
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
