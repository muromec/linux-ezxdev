/* 
 * drivers/video/dbmx21fb.c
 *
 * MX21ADS LCD Controller framebuffer driver
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>.
 *
 * This file is based on dbmx21fb.c from Motorola Dragonball MX2 ADS BSP
 * Copyright 2002, 2003 Motorola, Inc. All Rights Reserved.
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
/* Attention:
   This code support only 16-bit-per-pixel mode.
*/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/wrapper.h>
#include <linux/selection.h>
#include <linux/console.h>
#include <linux/kd.h>
#include <linux/vt_kern.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <asm/proc/pgtable.h>

#include <video/fbcon.h>
#include <video/fbcon-mfb.h>
#include <video/fbcon-cfb16.h>

#include <asm/arch/memory.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pll.h>

#include "dbmx21fb.h"

#ifdef CONFIG_MX21_TVOUT
#include "mx21tvout.h"
#endif /* CONFIG_MX21_TVOUT */

#ifdef CONFIG_PM
#include <linux/pm.h>
struct pm_dev *pm;
#endif
#if 1 /*CEE LDM*/
#include <linux/device.h>

extern void mx21_ldm_bus_register(struct device *device,
                          struct device_driver *driver);
extern void mx21_ldm_bus_unregister(struct device *device,
                          struct device_driver *driver);
static int
__ldm_suspend(struct device *dev, u32 state, u32 level);

static int
__ldm_resume(struct device *dev, u32 level);

static struct device_driver __driver_ldm = {
	.name = "dbmx21fb",
	.suspend = __ldm_suspend,
	.resume = __ldm_resume,
};

static struct device __device_ldm = {
	.name = "dbmx21fb",
	.bus_id = "framebuffer",
	.driver = &__driver_ldm,
	.power_state = DPM_POWER_ON,
};

#endif

/* Graphic window defines */

/* Virtual width and height of the graphic window.
If undefined, virtual size is equal to physical size */
/*#define GW_XRES_VIRTUAL 240*/
/*#define GW_YRES_VIRTUAL 320*/

/* Alpha value, range 0 - 255, 0 - totally transparent (unvisible) window, 255 - totally opaque  */
#define GW_ALPHA 160

#define HARDWARE_CURSOR

struct fbcon_font_desc *font;

/* LCD controller parameters */
struct dbmx21fb_par {
	unsigned char *screen_start_address;	/* Screen Start Address */
	unsigned char *v_screen_start_address;	/* Virtual Screen Start Address */
	unsigned long screen_memory_size;	/* screen memory size */
	unsigned int palette_size;
	unsigned int max_xres;
	unsigned int max_yres;
	unsigned int xres;
	unsigned int yres;
	unsigned int xres_virtual;
	unsigned int yres_virtual;
	unsigned int max_bpp;
	unsigned int bits_per_pixel;
	unsigned int currcon;
	unsigned int visual;
	unsigned int TFT:1;
	unsigned int color:1;
	unsigned int sharp:1;

	unsigned short cfb16[16];
};

struct gw_info {
	struct fb_info fb;

	/* overlay specific info */
	unsigned long	xpos;		/* screen position (x, y)*/
	unsigned long	ypos;		
	unsigned long 	format;

	unsigned char *screen_start_address;	/* Screen Start Address */
	unsigned char *v_screen_start_address;	/* Virtual Screen Start Address */
	unsigned long screen_memory_size;	/* screen memory size */
};

#ifdef HARDWARE_CURSOR
/* hardware cursor parameters */
struct dbmx21fb_cursor {
	int startx;
	int starty;
	int blinkenable;
	int blink_rate;
	int width;
	int height;
	int color[3];
	int state;
};

/* Frame buffer of LCD information */
struct dbmx21fb_info {
	struct display_switch dispsw;
	struct dbmx21fb_cursor cursor;
};
#endif				/*  HARDWARE_CURSOR */

static unsigned char *p_framebuffer_memory_address;
static unsigned char *v_framebuffer_memory_address;

static unsigned char *p_GW_framebuffer_memory_address;
static unsigned char *v_GW_framebuffer_memory_address;

/* Fake monspecs to fill in fbinfo structure */
static struct fb_monspecs monspecs __initdata = {
	30000, 70000, 50, 65, 0	/* Generic */
};

/* color map initial */
static unsigned short __attribute__ ((unused)) color4map[16] = {
0x0000, 0x000f, 0x00f0, 0x0f2a, 0x0f00, 0x0f0f, 0x0f88, 0x0ccc,
	    0x0888, 0x00ff, 0x00f8, 0x0f44, 0x0fa6, 0x0f22, 0x0ff0, 0x0fff};

static struct display global_disp;	/* Initial (default) Display Settings */
static struct fb_info fb_info;
static struct fb_var_screeninfo init_var = { };
static struct dbmx21fb_par current_par = { };

static struct display gw_global_disp;	   /* Initial (default) Display Settings for graphic window */
static struct gw_info gw;

/* Frame buffer device API */
static int dbmx21fb_get_fix(struct fb_fix_screeninfo *fix, int con,
			    struct fb_info *info);
static int dbmx21fb_get_var(struct fb_var_screeninfo *var, int con,
			    struct fb_info *info);
static int dbmx21fb_set_var(struct fb_var_screeninfo *var, int con,
			    struct fb_info *info);
static int dbmx21fb_get_cmap(struct fb_cmap *cmap, int kspc, int con,
			     struct fb_info *info);
static int dbmx21fb_set_cmap(struct fb_cmap *cmap, int kspc, int con,
			     struct fb_info *info);

/* Interface to the low level driver  */
static int dbmx21fb_switch(int con, struct fb_info *info);
static void dbmx21fb_blank(int blank, struct fb_info *info);
static int dbmx21fb_updatevar(int con, struct fb_info *info);

/*  Internal routines  */
static int reserve_fb_memory(void);
static void enable_lcd_controller(void);
static void disable_lcd_controller(void);
static int encode_var(struct fb_var_screeninfo *var, struct dbmx21fb_par *par);
static int decode_var(struct fb_var_screeninfo *var, struct dbmx21fb_par *par);


/* Graphic window routines */
static int gw_set_var(struct fb_var_screeninfo *var, int con,
			    struct fb_info *info);
static int gw_get_fix(struct fb_fix_screeninfo *fix, int con,
			    struct fb_info *info);
static int gw_get_var(struct fb_var_screeninfo *var, int con,
			    struct fb_info *info);
static int gw_release(struct fb_info *info, int user);


/* initialization routines */
#ifdef CONFIG_MX21_TVOUT
static int __init init_tvout(void);
#else /* CONFIG_MX21_TVOUT */
static void __init init_lcd_system(void);
static int __init init_lcd(void);
#endif /* CONFIG_MX21_TVOUT */

static void __init init_fbinfo(void);
static void __init gw_init_fbinfo(void);
static int __init reserve_fb_memory(void);

/* frame buffer ops */
static struct fb_ops dbmx21fb_ops = {
	.owner		= THIS_MODULE,
	.fb_get_fix	= dbmx21fb_get_fix,
	.fb_get_var	= dbmx21fb_get_var,
	.fb_set_var	= dbmx21fb_set_var,
	.fb_get_cmap	= dbmx21fb_get_cmap,
	.fb_set_cmap	= dbmx21fb_set_cmap,
};

static struct fb_ops gw_ops = {
	.owner		= THIS_MODULE,
	.fb_release     = gw_release,
	.fb_get_fix	= gw_get_fix,
	.fb_get_var	= gw_get_var,
	.fb_set_var	= gw_set_var,
};

/* Hardware Cursor */
#ifdef HARDWARE_CURSOR
static void dbmx21fb_cursor(struct display *p, int mode, int x, int y);
static int dbmx21fb_set_font(struct display *d, int width, int height);
static unsigned char cursor_color_map[] = { 0xf8 };
static void dbmx21fb_set_cursor_state(struct dbmx21fb_info *fb,
				      unsigned int state);
static void dbmx21fb_set_cursor(struct dbmx21fb_info *fb);
static void dbmx21fb_set_cursor_blink(struct dbmx21fb_info *fb, int blink);

struct display_switch dbmx21fb_cfb16 = {
	.setup		= fbcon_cfb16_setup,
	.bmove		= fbcon_cfb16_bmove,
	.clear		= fbcon_cfb16_clear,
	.putc 		= fbcon_cfb16_putc,
	.putcs 		= fbcon_cfb16_putcs,
	.revc		= fbcon_cfb16_revc,
	.cursor		= dbmx21fb_cursor,
	.set_font 	= dbmx21fb_set_font,
	.fontwidthmask	= FONTWIDTH(4) | FONTWIDTH(8) | FONTWIDTH(16)
};

#endif /* HARDWARE_CURSOR */

/*****************************************************************************
 * Function Name: dbmx21fb_getcolreg()
 *
 * Input: 	regno	: Color register ID
 *		red	: Color map red[]
 *		green	: Color map green[]
 *		blue	: Color map blue[]
 *	transparent	: Flag
 *		info	: Fb_info database
 *
 * Value Returned: int	: Return status.If no error, return 0.
 *	
 * Description: Transfer to fb_xxx_cmap handlers as parameters to
 *		control color registers
 *
******************************************************************************/
#define RED	0xf00
#define GREEN	0xf0
#define BLUE	0x0f
static int
dbmx21fb_getcolreg(unsigned int regno, unsigned int *red, unsigned int *green, unsigned int *blue,
		   unsigned int *trans, struct fb_info *info)
{
	unsigned int val;

	if (regno >= current_par.palette_size)
		return 1;

	val = LCDC_BPLUT_BASE(regno);

	if ((current_par.bits_per_pixel == 4) && (!current_par.color)) {
		*red = *green = *blue = (val & BLUE) << 4;	/* TODO */
		*trans = 0;
	} else {
		*red = (val & RED) << 4;
		*green = (val & GREEN) << 8;
		*blue = (val & BLUE) << 12;
		*trans = 0;
	}

	return 0;
}

/*****************************************************************************
 * Function Name: dbmx21fb_setcolreg()
 *
 * Input: 	regno	: Color register ID
 *		red	: Color map red[]
 *		green	: Color map green[]
 *		blue	: Color map blue[]
 *	transparent	: Flag
 *		info	: Fb_info database
 *
 * Value Returned: int 	: Return status.If no error, return 0.
 *
 * Description: Transfer to fb_xxx_cmap handlers as parameters to
 *		control color registers
 *
 **********************************************************F*******************/
static int
dbmx21fb_setcolreg(unsigned int regno, unsigned int red, unsigned int green, unsigned int blue,
		   unsigned int trans, struct fb_info *info)
{
	unsigned int val = 0;

	if (regno >= current_par.palette_size) {
		return 1;
	}

	if ((current_par.bits_per_pixel == 4) && (!current_par.color))
		val = (blue & 0x00f) << 12;	/* TODO */
	else {
		val = (blue >> 12) & BLUE;
		val |= (green >> 8) & GREEN;
		val |= (red >> 4) & RED;
	}

	if (regno < 16) {
		current_par.cfb16[regno] = regno | regno << 5 | regno << 10;
	}

	LCDC_BPLUT_BASE(regno) = val;

	return 0;
}

/*****************************************************************************
 * Function Name: dbmx21fb_get_cmap()
 *
 * Input: 	cmap	: Ouput data pointer
 *		kspc   	: Kernel space flag
 *		con    	: Console ID
 *		info	: Frame buffer information
 *
 * Value Returned: int	: Return status.If no error, return 0.
 *	
 * Description: Data is copied from hardware or local or system DISPAY,
 *		and copied to cmap.
 *
******************************************************************************/
static int
dbmx21fb_get_cmap(struct fb_cmap *cmap, int kspc, int con, struct fb_info *info)
{
	int err = 0;
		
	if (con == current_par.currcon)
		err = fb_get_cmap(cmap, kspc, dbmx21fb_getcolreg, info);
	else if (fb_display[con].cmap.len)
		fb_copy_cmap(&fb_display[con].cmap, cmap, kspc ? 0 : 2);
	else
		fb_copy_cmap(fb_default_cmap(current_par.palette_size),
			     cmap, kspc ? 0 : 2);

	return err;
}

/*****************************************************************************
 * Function Name: dbmx21fb_set_cmap()
 *
 * Input: 	cmap	: Ouput data pointer
 *		kspc   	: Kernel space flag
 *		con    	: Console ID
 *		info	: Frame buffer information
 *
 * Value Returned: int	: Return status.If no error, return 0.
 *	
 * Description: Copy data from cmap and copy to DISPLAY. If DISPLAy has no cmap,
 * 		allocate memory for it. If DISPLAY is current console and visible,
 * 		then hardware color map shall be set.
 *
******************************************************************************/
static int
dbmx21fb_set_cmap(struct fb_cmap *cmap, int kspc, int con, struct fb_info *info)
{
	int err = 0;

	if (!fb_display[con].cmap.len)
		err = fb_alloc_cmap(&fb_display[con].cmap,
					 current_par.palette_size, 0);

	if (!err) {
		if (con == current_par.currcon) {
			err = fb_set_cmap(cmap, kspc, dbmx21fb_setcolreg, info);
		}
		fb_copy_cmap(cmap, &fb_display[con].cmap, kspc ? 0 : 1);
	}

	return err;
}

/*****************************************************************************
 * Function Name: dbmx21fb_get_var()
 *
 * Input: 	var	: Iuput data pointer
 *		con	: Console ID
 *		info	: Frame buffer information
 *
 * Value Returned: int	: Return status.If no error, return 0.
 *
 * Functions Called: 	encode_var()
 *	
 * Description: Get color map from current, or global display[console]
 * 		used by ioctl
 *
******************************************************************************/
static int
dbmx21fb_get_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	if (con == -1) 
		encode_var(var, &current_par);
	else
		*var = fb_display[con].var;

	return 0;
}

/*****************************************************************************
 * Function Name: dbmx21fb_updatevar()
 *
 * Value Returned: VOID
 *
 * Functions Called: VOID
 *	
 * Description: Fill in display switch with LCD information,
 *
******************************************************************************/
static int
dbmx21fb_updatevar(int con, struct fb_info *info)
{
	return 0;
}

/*****************************************************************************
 * Function Name: dbmx21fb_set_dispsw()
 *
 * Input: 	display		: Iuput data pointer
 *		dbmx21fb_info   	: Frame buffer of LCD information
 *
 * Value Returned: VOID
 *
 * Functions Called: VOID
 *	
 * Description: Fill in display switch with LCD information,
 *
******************************************************************************/
static void
dbmx21fb_set_dispsw(struct display *disp
#ifdef HARDWARE_CURSOR
		    , struct dbmx21fb_info *info
#endif
    )
{
	switch (disp->var.bits_per_pixel) {
#ifdef HARDWARE_CURSOR
	case 12:
	case 16:
		fb_info.fix.visual = FB_VISUAL_DIRECTCOLOR;
		info->dispsw = dbmx21fb_cfb16;
		disp->dispsw = &info->dispsw;
		disp->dispsw_data = current_par.cfb16;
		break;

#else /* HARDWARE_CURSOR */
	/* first step disable the hardware cursor */
	case 12:
	case 16:
		fb_info.fix.visual = FB_VISUAL_DIRECTCOLOR;
		disp->dispsw = &fbcon_cfb16;
		disp->dispsw_data = current_par.cfb16;
		break;

#endif	/* HARDWARE_CURSOR */
	default:
		disp->dispsw = &fbcon_dummy;
		disp->dispsw_data = NULL;
		break;
	}

#ifdef HARDWARE_CURSOR
	if (&info->cursor) {
		info->dispsw.cursor = dbmx21fb_cursor;
		info->dispsw.set_font = dbmx21fb_set_font;
	}
#endif	/* HARDWARE_CURSOR */
}

/*****************************************************************************
 * Function Name: dbmx21fb_set_var()
 *
 * Input: 	var	: Iuput data pointer
 *		con	: Console ID
 *		info	: Frame buffer information
 *
 * Value Returned: int	: Return status.If no error, return 0.
 *
 * Functions Called: 	dbmx21fb_decode_var()
 * 			dbmx21fb_encode_var()
 *  			dbmx21fb_set_dispsw()
 *
 * Description: set current_par by var, also set display data, specially the console
 * 		related fileops, then enable the lcd controller, and set cmap to
 * 		hardware.
 *
******************************************************************************/
static int
dbmx21fb_set_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	struct display *display;
	int err, chgvar = 0;
	struct dbmx21fb_par par;

	if (con >= 0)
		display = &fb_display[con]; /* Display settings for console */
	else
		display = &global_disp;	/* Default display settings */

	/* 
	 * Decode var contents into a par structure, adjusting any 
	 * out of range values. 
	 */
	if ((err = decode_var(var, &par))) {
		return err;
	}

	/* Store adjusted par values into var structure */
	encode_var(var, &par);

	if ((var->activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_TEST)
		return 0;
	else if (((var->activate & FB_ACTIVATE_MASK) != FB_ACTIVATE_NOW) &&
		 ((var->activate & FB_ACTIVATE_MASK) != FB_ACTIVATE_NXTOPEN))
		return -EINVAL;

	if (con >= 0) {
		if ((display->var.xres != var->xres) ||
		    (display->var.yres != var->yres) ||
		    (display->var.xres_virtual != var->xres_virtual) ||
		    (display->var.yres_virtual != var->yres_virtual) ||
		    (display->var.sync != var->sync) ||
		    (display->var.bits_per_pixel != var->bits_per_pixel) ||
		    (memcmp(&display->var.red, &var->red, sizeof(var->red))) ||
		    (memcmp(&display->var.green, &var->green, 
		    		sizeof(var->green))) ||
		    (memcmp(&display->var.blue, &var->blue, sizeof(var->blue))))
			chgvar = 1;
	}

	display->var = *var;
	display->screen_base = par.v_screen_start_address;
	display->visual = par.visual;
	display->type = FB_TYPE_PACKED_PIXELS;
	display->type_aux = 0;
	display->ypanstep = 0;
	display->ywrapstep = 0;
	display->line_length = display->next_line = (var->xres * 16) / 8;

	display->can_soft_blank = 1;
	display->inverse = 0;

	dbmx21fb_set_dispsw(display
#ifdef HARDWARE_CURSOR
			    , (struct dbmx21fb_info *) info
#endif
	    );

	/* 
	 * If the console has changed and the console has defined
	 * a changevar function, call that function. 
	 */
	if (chgvar && info && info->changevar)
		info->changevar(con);	/* TODO */

	/* 
	 * If the current console is selected and it's not truecolor,
	 * update the palette
	 */
	if ((con == current_par.currcon)) {
		struct fb_cmap *cmap;

		current_par = par;	/* TODO */
		if (display->cmap.len)
			cmap = &display->cmap;
		else
			cmap = fb_default_cmap(current_par.palette_size);

		fb_set_cmap(cmap, 1, dbmx21fb_setcolreg, info);
	}

	/* If the current console is selected, activate the new var. */
	if (con == current_par.currcon) {
		init_var = *var;
		enable_lcd_controller();
	}

	return 0;
}

/*****************************************************************************
 * Function Name: dbmx21fb_get_fix()
 *
 * Input: 	fix	: Ouput data pointer
 *		con	: Console ID
 *		info	: Frame buffer information
 *
 * Value Returned: int	: Return status.If no error, return 0.
 *
 * Functions Called: VOID
 *	
 * Description: get fix from display data, current_par data
 * 		used by ioctl
 * 				
******************************************************************************/
static int
dbmx21fb_get_fix(struct fb_fix_screeninfo *fix, int con, struct fb_info *info)
{
	struct display *display;

	memset(fix, 0, sizeof (struct fb_fix_screeninfo));
	strcpy(fix->id, DBMX21_NAME);

	if (con >= 0) {
		display = &fb_display[con]; /* Display settings for console */
	} else
		display = &global_disp;	/* Default display settings */

	fix->smem_start = (unsigned long) current_par.screen_start_address;
	fix->smem_len = current_par.screen_memory_size;
	fix->type = display->type;
	fix->type_aux = display->type_aux;
	fix->xpanstep = 0;
	fix->ypanstep = display->ypanstep;
	fix->ywrapstep = display->ywrapstep;
	fix->visual = display->visual;
	fix->line_length = display->line_length;
	fix->accel = FB_ACCEL_NONE;

	return 0;
}

/*****************************************************************************
 * Function Name: dbmx21fb_blank()
 *
 * Input: 	blank	: Blank flag
 *		info	: Frame buffer database
 *
 * Value Returned: VOID
 *
 * Functions Called: 	enable_lcd_controller()
 * 			disable_lcd_controller()
 *	
 * Description: blank the screen, if blank, disable lcd controller, while if no blank
 * 		set cmap and enable lcd controller
 *
******************************************************************************/
static void
dbmx21fb_blank(int blank, struct fb_info *info)
{
}

/*****************************************************************************
 * Function Name: dbmx21fb_switch()
 *
 * Input:  	info	: Frame buffer database
 *
 * Value Returned: VOID
 *
 * Functions Called:
 *	
 * Description: Switch to another console
 *
******************************************************************************/
static int
dbmx21fb_switch(int con, struct fb_info *info)
{
	struct fb_cmap *cmap;
	
	if (current_par.currcon >= 0) {
		/* Get the colormap for the selected console */
		cmap = &fb_display[current_par.currcon].cmap;

		if (cmap->len)
			fb_get_cmap(cmap, 1, dbmx21fb_getcolreg, info);
	}

	current_par.currcon = con;
	fb_display[con].var.activate = FB_ACTIVATE_NOW;
	dbmx21fb_set_var(&fb_display[con].var, con, info);

	return 0;
}

/*****************************************************************************
 * Function Name: _encode_par()
 *
 * Input:  	var	: Input var data
 *		par	: LCD controller parameters
 *
 * Value Returned: VOID
 *
 * Functions Called:
 *
 * Description: use current_par to set a var structure
 *
******************************************************************************/
static int
encode_var(struct fb_var_screeninfo *var, struct dbmx21fb_par *par)
{

	switch (par->bits_per_pixel) {
	case 12:		/* This case should differ for Active/Passive mode */
	case 16:
		var->red.length = 5;
		var->green.length = 6;
		var->blue.length = 5;
		var->transp.length = 0;
		var->red.offset = 11;
		var->green.offset = 5;
		var->blue.offset = 0;
		var->transp.offset = 0;
		break;
	default:
		return -EINVAL;
	}

	/*
	 * Don't know if really want to zero var on entry.
	 * Look at set_var to see. If so, may need to add extra params to par
	 */
	var->xres = par->xres;
	var->yres = par->yres;
	var->xres_virtual = par->xres_virtual;
	var->yres_virtual = par->yres_virtual;
	var->bits_per_pixel = par->bits_per_pixel;

	return 0;
}

/*****************************************************************************
 * Function Name: decode_var
 *
 * Input:  	var	: Input var data
 *		par	: LCD controller parameters
 *
 * Value Returned: VOID
 *
 * Functions Called: VOID
 *	
 * Description: Get the video params out of 'var'. If a value doesn't fit,
 * 		round it up,if it's too big, return -EINVAL.
 *
 * Cautions: Round up in the following order: bits_per_pixel, xres,
 * 	yres, xres_virtual, yres_virtual, xoffset, yoffset, grayscale,
 * 	bitfields, horizontal timing, vertical timing. 			
 *
******************************************************************************/
static int
decode_var(struct fb_var_screeninfo *var, struct dbmx21fb_par *par)
{
	*par = current_par;
	
	if ((par->xres = var->xres) < MIN_XRES)
		par->xres = MIN_XRES;
	if ((par->yres = var->yres) < MIN_YRES)
		par->yres = MIN_YRES;
	if (par->xres > current_par.max_xres)
		par->xres = current_par.max_xres;
	if (par->yres > current_par.max_yres)
		par->yres = current_par.max_yres;
	par->xres_virtual =
	    var->xres_virtual < par->xres ? par->xres : var->xres_virtual;
	par->yres_virtual =
	    var->yres_virtual < par->yres ? par->yres : var->yres_virtual;
	par->bits_per_pixel = var->bits_per_pixel;

	switch (par->bits_per_pixel) {
	case 12:		/* RGB 444 */
	case 16:		/* RGB 565 */
		par->visual = FB_VISUAL_TRUECOLOR;
		par->palette_size = 256;
		break;
	default:
		return -EINVAL;
	}

	par->screen_start_address =
	    (unsigned char *) ((unsigned long) p_framebuffer_memory_address + 
	    			PAGE_SIZE);
	par->v_screen_start_address =
	    (unsigned char *) ((unsigned long) v_framebuffer_memory_address + 
	    			PAGE_SIZE);

	return 0;
}

/*****************************************************************************
 * Function Name: reserve_fb_memory()
 *
 * Input: VOID
 *
 * Value Returned: VOID
 *
 * Functions Called: 	
 *
 * Description: get data out of var structure and set related LCD controller registers
 *
******************************************************************************/
static int __init
reserve_fb_memory(void)
{
	unsigned int 	required_pages;
	unsigned int 	extra_pages;
	unsigned int 	order;
	struct page 	*page;
	char 		*allocated_region;

	if (v_framebuffer_memory_address != NULL)
		return -EINVAL;

	/* Find order required to allocate enough memory for framebuffer */
	required_pages = ALLOCATED_FB_MEM_SIZE >> PAGE_SHIFT;

	for (order = 0; required_pages >> order; order++)
		;

	extra_pages = (1 << order) - required_pages;

	if ((allocated_region =
	     (char *) __get_free_pages(GFP_KERNEL | GFP_DMA, order)) == NULL) {
		return -ENOMEM;
	}

	v_framebuffer_memory_address =
	    (unsigned char *) allocated_region + (extra_pages << PAGE_SHIFT);

	p_framebuffer_memory_address =
	    (unsigned char *) __virt_to_phys((unsigned long) v_framebuffer_memory_address);

	/* 
	 * Free all pages that we don't need but were given to us because 
	 * __get_free_pages() works on powers of 2. 
	 */
	for (; extra_pages; extra_pages--)
		free_page((unsigned int) allocated_region +
			  ((extra_pages - 1) << PAGE_SHIFT));
	
	/* 
	 * Set reserved flag for fb memory to allow it to be remapped into
	 * user space by the common fbmem driver using remap_page_range(). 
	 */
	
	for (page = virt_to_page(v_framebuffer_memory_address);
	     page <
	     virt_to_page(v_framebuffer_memory_address + ALLOCATED_FB_MEM_SIZE);
	     page++) {
		mem_map_reserve(page);
	}

	current_par.screen_start_address = (unsigned char *) 
		((unsigned long) p_framebuffer_memory_address + PAGE_SIZE);
	current_par.v_screen_start_address = (unsigned char *) 
		((unsigned long) v_framebuffer_memory_address + PAGE_SIZE);

	return (v_framebuffer_memory_address == NULL ? -EINVAL : 0);
}

static int __init
reserve_gw_memory(void)
{
	unsigned int 	required_pages;
	unsigned int 	extra_pages;
	unsigned int 	order;
	struct page 	*page;
	char 		*allocated_region;

	if (v_GW_framebuffer_memory_address != NULL)
		return -EINVAL;

	/* Find order required to allocate enough memory for framebuffer */
	required_pages = ALLOCATED_FB_MEM_SIZE >> PAGE_SHIFT;

	for (order = 0; required_pages >> order; order++)
		;

	extra_pages = (1 << order) - required_pages;

	if ((allocated_region =
	     (char *) __get_free_pages(GFP_KERNEL | GFP_DMA, order)) == NULL) {
		return -ENOMEM;
	}

	v_GW_framebuffer_memory_address =
	    (unsigned char *) allocated_region + (extra_pages << PAGE_SHIFT);

	p_GW_framebuffer_memory_address =
	    (unsigned char *) __virt_to_phys((unsigned long) v_GW_framebuffer_memory_address);

	/*
	 * Free all pages that we don't need but were given to us because
	 * __get_free_pages() works on powers of 2.
	 */
	for (; extra_pages; extra_pages--)
		free_page((unsigned int) allocated_region +
			  ((extra_pages - 1) << PAGE_SHIFT));
	
	/*
	 * Set reserved flag for fb memory to allow it to be remapped into
	 * user space by the common fbmem driver using remap_page_range().
	 */
	
	for (page = virt_to_page(v_GW_framebuffer_memory_address);
	     page <
	     virt_to_page(v_GW_framebuffer_memory_address + ALLOCATED_FB_MEM_SIZE);
	     page++) {
		mem_map_reserve(page);
	}

	gw.screen_start_address = (unsigned char *)
		((unsigned long) p_GW_framebuffer_memory_address + PAGE_SIZE);
	gw.v_screen_start_address = (unsigned char *)
		((unsigned long) v_GW_framebuffer_memory_address + PAGE_SIZE);

	return (v_GW_framebuffer_memory_address == NULL ? -EINVAL : 0);
}

/*****************************************************************************
 * Function Name: enable_lcd_controller()
 *
 * Input: VOID
 *
 * Value Returned: VOID
 *
 * Functions Called: 	
 *	
 * Description: enable Lcd controller, setup registers,
 *		base on current_par value
 *
******************************************************************************/
static void
enable_lcd_controller(void)
{
	mx_module_clk_open(HCLK_MODULE_LCDC);
	mx_module_clk_open(IPG_MODULE_LCDC_PIXCLK);

	*((volatile uint16_t *) (MX2ADS_PER_IOBASE + 0x800000)) |= 0x0200;
}

/*****************************************************************************
 * Function Name: disable_lcd_controller()
 *
 * Input: VOID
 *
 * Value Returned: VOID
 *
 * Functions Called: VOID
 *
 * Description: just disable the LCD controller
 * 		disable lcd interrupt. others, i have no ideas
 *
******************************************************************************/
static void
disable_lcd_controller(void)
{
	*((volatile uint16_t *) (MX2ADS_PER_IOBASE + 0x800000)) &= ~0x0200;

	mx_module_clk_close(HCLK_MODULE_LCDC);
	mx_module_clk_close(IPG_MODULE_LCDC_PIXCLK);
}

#ifdef CONFIG_MX21_TVOUT
static int __init
init_tvout(void)
{
        decode_var(&init_var, &current_par);
        
#ifdef CONFIG_MX21_TVOUT_PAL50
        mx2_tvout_init(640, 438, TVOUT_MODE_PAL625, (unsigned int)
                current_par.screen_start_address);
#else
        mx2_tvout_init(640, 435, TVOUT_MODE_NTSC358, (unsigned int)
                current_par.screen_start_address); 
#endif

        return 0;
}
#else /* CONFIG_MX21_TVOUT */
/*****************************************************************************
 * Function Name: init_lcd_system
 *
 * Input: VOID
 *
 * Value Returned: VOID
 *
 * Functions Called:
 *
 * Description: initialise GPIO
 *
******************************************************************************/
static void __init
init_lcd_system(void)
{
	mx2_register_gpios(PORT_A, 0xFFFFFFFF, PRIMARY);
}

/*****************************************************************************
 * Function Name: init_lcd
 *
 * Input: VOID
 *
 * Value Returned: VOID
 *
 * Functions Called: decode_var()
 *
 * Description: initialize the LCD controller, use current_par for 12bpp
 *
******************************************************************************/
static int __init
init_lcd(void)
{
	unsigned int val;

	decode_var(&init_var, &current_par);

	LCDC_LSSAR = (unsigned int) current_par.screen_start_address;

	val = 0;
	val = current_par.xres / 16;
	val = val << 20;
	val += current_par.yres;

	LCDC_LSR = val;

	val = 0;
	val = current_par.xres_virtual / 2;
	LCDC_LVPWR = val;
	LCDC_LPCR = 0xFB108BC7;

	/* Sharp Configuration Register */
	LCDC_LSCR = 0x00120300;
	LCDC_LHCR = 0x04000F06;
	LCDC_LVCR = 0x04000907;
	LCDC_LPCCR = 0x00A903FF;
	LCDC_LRMCR = 0x00000000;
	LCDC_LDCR = 0x0004000F;

	return 0;
}

#endif /* CONFIG_MX21_TVOUT */

static void
lcd_pm_resume(void)
{
	enable_lcd_controller();
	LCDC_LPCCR |= (1<<8); /* enable PWM for backlight */
	LCDC_LPCCR &= ~0x8000; /* enable normal LD output for the Sharp TFT panel */
}

static void
lcd_pm_suspend(void)
{

	LCDC_LPCCR |= 0x8000; /* disable LD output to zero the Sharp TFT panel */
	LCDC_LPCCR &= ~(1<<8); /* disable PWM for backlight */
	mdelay(10);
	disable_lcd_controller();
}

#ifdef CONFIG_PM
static int
lcd_pm_handler(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	switch (rqst) {
	case PM_RESUME:
		lcd_pm_resume();
		break;
	case PM_SUSPEND:
		lcd_pm_suspend();
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif
#if 1 /*CEE LDM*/
static int
__ldm_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		lcd_pm_suspend();
		break;
	}
	return 0;
}

static int
__ldm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		lcd_pm_resume();
		break;
	}
	return 0;
}
#endif


/*****************************************************************************
 * Function Name: dbmx21fb_init()
 *
 * Input: VOID
 *
 * Value Returned: int 		: Return status.If no error, return 0.
 *
 * Functions Called: 	init_fbinfo()
 *			disable_irq()
 *	 		enable_irq()
 *			init_lcd()
 *			dbmx21fb_init_cursor()
 *
 * Description: initialization module, all of init routine's entry point
 * 		initialize fb_info, init_var, current_par
 * 		and setup interrupt, memory, lcd controller
 *
******************************************************************************/
int __init
dbmx21fb_init(void)
{
	int ret;
	int divider;
	unsigned int reg;

	init_fbinfo();
	if ((ret = reserve_fb_memory()) != 0) {
		printk(KERN_ERR
		       "MX2 Framebuffer: failed to reserve frame buffer memory\n");
		return ret;
	}

	dbmx21fb_set_var(&init_var, -1, &fb_info);

#ifdef CONFIG_MX2TO2
        divider = CLK_MPLL / LCD_FREQ;
        reg = CRM_PCDR1;
        reg &= ~PCDR1_PERDIV3;
        reg |= ((divider - 1) << PCDR1_PERDIV3_SHIFT) & PCDR1_PERDIV3;
        CRM_PCDR1 = reg;
#endif
        
#ifdef CONFIG_MX21_TVOUT
	init_tvout();
#else  /* CONFIG_MX21_TVOUT */
	init_lcd_system();
	init_lcd();
        
#endif /* CONFIG_MX21_TVOUT */
	enable_lcd_controller();

	register_framebuffer(&fb_info);

	gw_init_fbinfo();

	if ((ret = reserve_gw_memory()) != 0) {
		printk(KERN_ERR
		       "MX2 Framebuffer: failed to reserve graphic window memory\n");
		return ret;
	}

	ret = register_framebuffer(&(gw.fb));

#ifdef CONFIG_PM
	pm = pm_register(PM_SYS_DEV, PM_SYS_VGA, lcd_pm_handler);
#endif
#if 1 /*CEE LDM*/
	mx21_ldm_bus_register(&__device_ldm, &__driver_ldm);
#endif
	/* This driver cannot be unloaded at the moment */
	MOD_INC_USE_COUNT;

	return 0;
}

/*****************************************************************************
 * Function Name: dbmx21fb_setup()
 *
 * Input: info	: VOID
 *
 * Value Returned: int	: Return status.If no error, return 0.
 *
 * Functions Called: VOID
 *	
 * Description: basically, this routine used to parse command line parameters, which
 * 		is initialization parameters for lcd controller, such as freq, xres,
 * 		yres, and so on
 *
******************************************************************************/
int __init
dbmx21fb_setup(char *options)
{
	return 0;
}

/*****************************************************************************
 * Function Name: init_fbinfo()
 *
 * Input: VOID
 *
 * Value Returned: VOID
 *
 * Functions Called: VOID
 *
 * Description: while 16bpp is used to store a 12 bits pixels packet, but
 * 		it is not a really 16bpp system. maybe in-compatiable with
 * 		other system or GUI.There are some field in var which specify
 *		the red/green/blue offset in a 16bit word, just little endian is
 * 		concerned
 *
******************************************************************************/
static void __init
init_fbinfo(void)
{
	strcpy(fb_info.modename, DBMX21_NAME);
	strcpy(fb_info.fontname, "Acorn8x8");

	fb_info.node = -1;
	fb_info.flags = 0;
	fb_info.fbops = &dbmx21fb_ops;
	fb_info.monspecs = monspecs;
	fb_info.disp = &global_disp;
	fb_info.changevar = NULL;
	fb_info.switch_con = dbmx21fb_switch;
	fb_info.updatevar = dbmx21fb_updatevar;
	fb_info.blank = dbmx21fb_blank;

	/* setup initial parameters */
	memset(&init_var, 0, sizeof (init_var));

	init_var.transp.length = 0;
	init_var.nonstd = 0;
	init_var.activate = FB_ACTIVATE_NOW;
	init_var.xoffset = 0;
	init_var.yoffset = 0;
	init_var.height = -1;
	init_var.width = -1;
	init_var.vmode = FB_VMODE_NONINTERLACED;

	current_par.max_xres = LCD_MAXX;
	current_par.max_yres = LCD_MAXY;
	current_par.max_bpp = LCD_MAX_BPP;
	init_var.red.length = 5;
	init_var.green.length = 6;
	init_var.blue.length = 5;
	init_var.red.offset = 11;
	init_var.green.offset = 5;
	init_var.blue.offset = 0;

	init_var.grayscale = 16;
	init_var.sync = 0;
	init_var.pixclock = 171521;

	current_par.screen_start_address = NULL;
	current_par.v_screen_start_address = NULL;
	current_par.screen_memory_size = MAX_PIXEL_MEM_SIZE;
	current_par.currcon = -1;

	init_var.xres = current_par.max_xres;
	init_var.yres = current_par.max_yres;
	init_var.xres_virtual = init_var.xres;
	init_var.yres_virtual = init_var.yres;
	init_var.bits_per_pixel = current_par.max_bpp;
}

/* Hardware cursor support */
#ifdef HARDWARE_CURSOR
 /*****************************************************************************
 * Function Name: dbmx21fb_set_cursor_color()
 *
 * Input:   fb	: frame buffer database
 *	    red	: red component level in the cursor
 *	  green	: green component level in the cursor
 *	   blue	: blue component level in the cursor	
 *
 * Value Returned: VOID
 *
 * Functions Called: VOID	
 *	
 * Description: Set color of hardware cursor
 *
 * Modification History:
 *		10 DEC,2001, Zhang Juan
******************************************************************************/
static void
dbmx21fb_set_cursor_color(struct dbmx21fb_info *fb, unsigned char *red,
			  unsigned char *green, unsigned char *blue)
{
	struct dbmx21fb_cursor *c = &fb->cursor;
	unsigned int color;

	c->color[0] = *red;
	c->color[1] = *green;
	c->color[2] = *blue;
	color = (unsigned int) *red;
	color |= (unsigned int) (*green >> 5);
	color |= (unsigned int) (*blue >> 11);

	LCDC_LCCMR = color;
}

 /*****************************************************************************
 * Function Name: dbmx21fb_set_cursor()
 *
 * Input:   fb	: frame buffer database
 *
 * Value Returned: VOID
 *
 * Functions Called: VOID	
 *	
 * Description: Load information of hardware cursor
 *
 * Modification History:
 *		10 DEC,2001, Zhang Juan
******************************************************************************/
static void
dbmx21fb_set_cursor(struct dbmx21fb_info *fb)
{
	struct dbmx21fb_cursor *c = &fb->cursor;
	unsigned int temp, tempReg, x, y;

	x = c->startx << 16;
	if (c->state == LCD_CURSOR_ON)
		x |= CURSOR_ON_MASK;
	else if (c->state == LCD_CURSOR_INVERT_BGD)
		x |= CURSOR_INVERT_MASK;
	else if (c->state == LCD_CURSOR_AND_BGD)
		x |= CURSOR_AND_BGD_MASK;
	else if (c->state == LCD_CURSOR_OR_BGD)
		x |= CURSOR_OR_BGD_MASK;
	else if (c->state == LCD_CURSOR_XOR_BGD)
		x |= CURSOR_XOR_BGD_MASK;
	else
		x = c->startx;

	y = c->starty;

	temp = (unsigned int) x | (unsigned int) y;
	LCDC_LCPR = temp;

	temp = (unsigned int) ((c->width << 8) | (c->height));
	tempReg = (unsigned int) ((temp << 16) | c->blink_rate);

	LCDC_LCWHBR = tempReg;

	if (c->blinkenable)
		dbmx21fb_set_cursor_blink(fb, c->blink_rate);
}

 /*****************************************************************************
 * Function Name: dbmx21fb_set_cursor_blink()
 *
 * Input:   fb  : frame buffer database
 *	 blink	: input blink frequency of cursor	
 *
 * Value Returned: VOID
 *
 * Functions Called: VOID	
 *	
 * Description: Set blink frequency of hardware cursor
 *
 * Modification History:
 *		10 DEC,2001, Zhang Juan
******************************************************************************/
static void
dbmx21fb_set_cursor_blink(struct dbmx21fb_info *fb, int blink)
{
	struct dbmx21fb_cursor *c = &fb->cursor;
	unsigned long temp, tempReg;
	unsigned long PCD, XMAX, YMAX, PCLKDIV2;
	unsigned long tempMicroPeriod;

	if (!c) {
		printk(KERN_ERR "MX2 Framebuffer: cursor will not blink - it is not set\n");
		return;
	}

	c->blink_rate = blink;

	tempReg = LCDC_LSR;
	XMAX = (tempReg & XMAX_MASK) >> 20;
	YMAX = tempReg & YMAX_MASK;
	tempReg = CRM_PCDR;
	PCLKDIV2 = (tempReg & PCLKDIV2_MASK) >> 22;
	tempReg = LCDC_LPCR;
	PCD = tempReg & PCD_MASK;

	temp = (PCLKDIV2 + 1);

	if (!blink) {
		/* disable the blinking cursor function when frequency is 0 */
		tempReg = LCDC_LCWHBR;
		tempReg &= CURSORBLINK_DIS_MASK;
		LCDC_LCWHBR = tempReg;
	} else {

		tempMicroPeriod = temp * XMAX * YMAX * (PCD + 1);
		temp = 96 * 10000000 / (blink * tempMicroPeriod);
		tempReg = LCDC_LCWHBR;
		tempReg |= CURSORBLINK_EN_MASK;
		tempReg |= temp;
		LCDC_LCWHBR = tempReg;
	}
}

 /*****************************************************************************
 * Function Name: dbmx21fb_set_cursor_state()
 *
 * Input:   fb  : frame buffer database
 *	 state	: The status of the cursor to be set. e.g.
 *            		LCD_CURSOR_OFF
 *                      LCD_CURSOR_ON
 *                      LCD_CURSOR_REVERSED
 *                      LCD_CURSOR_ON_WHITE
 *                      LCD_CURSOR_OR_BGD
 *                      LCD_CURSOR_XOR_BGD
 *                      LCD_CURSOR_AND_BGD
 *
 * Value Returned: VOID
 *
 * Functions Called: VOID	
 *	
 * Description: Set state of cursor
 *
 * Modification History:
 *		10 DEC,2001, Zhang Juan
******************************************************************************/
static void
dbmx21fb_set_cursor_state(struct dbmx21fb_info *fb, unsigned int state)
{
	struct dbmx21fb_cursor *c = &fb->cursor;
	unsigned int temp;

	c->state = state;
	temp = LCDC_LCPR;
	temp &= CURSOR_OFF_MASK;

	if (state == LCD_CURSOR_OFF)
		temp = temp;
	else if (state == LCD_CURSOR_ON)
		temp |= CURSOR_ON_MASK;
	else if (state == LCD_CURSOR_INVERT_BGD)
		temp |= CURSOR_INVERT_MASK;
	else if (state == LCD_CURSOR_OR_BGD)
		temp |= CURSOR_OR_BGD_MASK;
	else if (state == LCD_CURSOR_XOR_BGD)
		temp |= CURSOR_XOR_BGD_MASK;
	else if (state == LCD_CURSOR_AND_BGD)
		temp |= CURSOR_AND_BGD_MASK;
	LCDC_LCPR = temp;
}

/*****************************************************************************
 * Function Name: dbmx21fb_cursor()
 *
 * Input:   fb     		: frame buffer database
 *
 * Value Returned: 	cursor : The structure of hardware cursor
 *
 * Functions Called: 	dbmx21fb_set_cursor()
 *			dbmx21fb_set_cursor_state()
 *	
 * Description: The entry for display switch to operate hardware cursor
 *
 * Modification History:
 *		10 DEC,2001, Zhang Juan
******************************************************************************/
static void
dbmx21fb_cursor(struct display *p, int mode, int x, int y)
{
	struct dbmx21fb_info *fb = (struct dbmx21fb_info *) p->fb_info;
	struct dbmx21fb_cursor *c = &fb->cursor;

	if (c == 0)
		return;

	x *= fontwidth(p);
	y *= fontheight(p);

	c->startx = x;
	c->starty = y;

	switch (mode) {
	case CM_ERASE:
		dbmx21fb_set_cursor_state(fb, LCD_CURSOR_OFF);
		break;
	case CM_DRAW:
	case CM_MOVE:
		c->state = LCD_CURSOR_ON;
		dbmx21fb_set_cursor(fb);
		dbmx21fb_set_cursor_state(fb, c->state);
		break;
	}
}

/*****************************************************************************
 * Function Name: dbmx21fb_set_font()
 *
 * Input:   display	: console datebase
 * 	    width	: The new width of cursor to be set.
 * 	    height	: The new height of cursor position to be set
 *	
 * Value Returned: int	: Return status.If no error, return 0.
 *
 * Functions Called: dbmx21fb_set_cursor()
 *		     dbmx21fb_set_cursor_color()
 *	
 * Description: Set  font for cursor
 *
 * Modification History:
 *		10 DEC,2001, Zhang Juan
******************************************************************************/
static int
dbmx21fb_set_font(struct display *d, int width, int height)
{
	struct dbmx21fb_info *fb = (struct dbmx21fb_info *) d->fb_info;
	struct dbmx21fb_cursor *c = &fb->cursor;

	if (!d) {
		return -1;
	}
	
	if (c) {
		if (!width || !height) {
			width = 16;
			height = 16;
		}

		c->width = width;
		c->height = height;

		dbmx21fb_set_cursor(fb);
		dbmx21fb_set_cursor_color(fb, cursor_color_map,
					  cursor_color_map, cursor_color_map);
	}

	return 1;
}
#endif	/* HARDWARE_CURSOR */

/* Graphic window routines */

static int
gw_enable(void)
{

	struct fb_var_screeninfo *dvar = &(gw.fb.var);

	if (!gw.screen_start_address) return -EINVAL;
	unsigned long lgwcr = LGWCR_GWE;       //Graphic window control register: enable graphic window

	/* Graphic window start address register */
	LCDC_LGWSAR = (unsigned int)gw.screen_start_address;

	/* Graphic window size register */
	LCDC_LGWSR = ((dvar->xres / 16) << LGWSR_GWW_BIT) + dvar->yres;

	/* Graphic window virtual page width register */
	LCDC_LGWVPWR = dvar->xres_virtual / 2;

	/* Graphic window position register */
	LCDC_LGWPR = ((gw.xpos & LGWPR_GWXP_MASK) << LGWPR_GWXP_BIT) | (gw.ypos & LGWPR_GWYP_MASK);

#ifdef GW_ALPHA
	lgwcr |= (GW_ALPHA & LGWCR_GWALPHA_MASK) << LGWCR_GWALPHA_BIT;	
#endif
	CRM_PCCR0 &= ~PCCR0_HCLK_LCDC_EN;
	LCDC_LGWCR = lgwcr;
	CRM_PCCR0 |= PCCR0_HCLK_LCDC_EN;

	return 0;
}

static int gw_disable(void)
{
	LCDC_LGWCR &= ~LGWCR_GWE;
	return 0;
}


static int
gw_validate_var( struct fb_var_screeninfo *var)
{
	int xpos, ypos;

	/* must in base frame */
	xpos = (var->nonstd & 0x3ff);
	ypos = ((var->nonstd>>10) & 0x3ff);

	if ( (xpos + var->xres) > current_par.max_xres ) {
		return -EINVAL;
	}

	if ( (ypos + var->yres) > current_par.max_yres ) {
		return -EINVAL;
	}

	return 0;
}

static int dummy_updatevar(int con, struct fb_info *info)
{
	return 0;
}

static void
gw_blank(int blank, struct fb_info *info)
{
	switch(blank)
	{
	case 0: /*unblank*/
		gw_enable();
		break;
	case 1: /*blank*/
		gw_disable();
		break;
	default:
		/* reserved */
		break;
	}
}

static int gw_release(struct fb_info *info, int user)
{
	gw_blank(1, NULL);

	return 0;
}

static int
gw_set_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	struct fb_var_screeninfo *dvar = &(gw.fb.var);
	struct display *display;
	int err=0, pixels_per_line=0;

	if (con >= 0)
		display = &fb_display[con]; /* Display settings for console */
	else
		display = &gw_global_disp; /* Default display settings */



	/* validate parameters*/
	err = gw_validate_var(var);
	if (err) return  err;

	display->var = *var;
	display->screen_base = gw.v_screen_start_address;
	display->type = FB_TYPE_PACKED_PIXELS;
	display->type_aux = 0;
	display->ypanstep = 0;
	display->ywrapstep = 0;
	display->line_length = display->next_line = (var->xres * 16) / 8;

	display->can_soft_blank = 1;
	display->inverse = 0;

	if ( (var->xres == dvar->xres)  &&
		(var->yres == dvar->yres) &&
		(var->xres_virtual == dvar->xres_virtual) &&
		(var->yres_virtual == dvar->yres_virtual) &&
		(var->bits_per_pixel == dvar->bits_per_pixel) )
		goto out2;

	/* update var_screeninfo fields*/

	*dvar = *var;

#ifndef GW_XRES_VIRTUAL
	gw.fb.var.xres_virtual = gw.fb.var.xres;
#else
	gw.fb.var.xres_virtual = GW_XRES_VIRTUAL;
#endif

#ifndef GW_YRES_VIRTUAL
	gw.fb.var.yres_virtual = gw.fb.var.yres;
#else
	gw.fb.var.yres_virtual = GW_YRES_VIRTUAL;
#endif

	pixels_per_line = (gw.fb.var.xres_virtual + 0x1) & (~0x1);
	
	gw.fb.fix.line_length = 2 * pixels_per_line;
	gw.fb.fix.smem_len = gw.fb.fix.line_length * gw.fb.var.yres;
	gw.fb.fix.smem_start = (unsigned long) gw.screen_start_address;


out2:
	gw.xpos = var->nonstd & 0x3ff;
	gw.ypos = (var->nonstd>>10) & 0x3ff;
	gw_enable();

	return 0;
}

static void __init gw_init_fbinfo(void)
{
	strcpy(gw.fb.fix.id, "graphic window");

	gw.fb.fix.type	      = FB_TYPE_PACKED_PIXELS;
	gw.fb.fix.type_aux    = 0;
	gw.fb.fix.xpanstep    = 0;
	gw.fb.fix.ypanstep    = 0;
	gw.fb.fix.ywrapstep   = 0;
	gw.fb.fix.accel	      = FB_ACCEL_NONE;

	gw.fb.var.nonstd      = 0;
	gw.fb.var.activate    = FB_ACTIVATE_NOW;
	gw.fb.var.height      = -1;
	gw.fb.var.width	      = -1;
	gw.fb.var.accel_flags = 0;
	gw.fb.var.vmode	      = FB_VMODE_NONINTERLACED;

	gw.fb.disp = &gw_global_disp;
	gw.fb.blank = &gw_blank;

	/*set up 16 bpp mode*/
	gw.fb.var.red.length = 5;
	gw.fb.var.green.length = 6;
	gw.fb.var.blue.length = 5;
	gw.fb.var.transp.length = 0;
	gw.fb.var.red.offset = 11;
	gw.fb.var.green.offset = 5;
	gw.fb.var.blue.offset = 0;
	gw.fb.var.transp.offset = 0;


	strcpy(gw.fb.modename, "graphic window");
	strcpy(gw.fb.fontname, "null");

	gw.fb.fbops	      = &gw_ops;
	gw.fb.changevar	      = NULL;
	gw.fb.switch_con      = NULL;
	gw.fb.updatevar	      = dummy_updatevar;
	gw.fb.flags	      = FBINFO_FLAG_DEFAULT;
	gw.fb.node	      = -1;
	gw.fb.monspecs	      = monspecs;
	gw.fb.pseudo_palette  = NULL;

	gw.xpos       = 0;
	gw.ypos       = 0;
	gw.format     = -1;

}


static int
gw_get_fix(struct fb_fix_screeninfo *fix, int con, struct fb_info *info)
{
	*fix = gw.fb.fix;
	return 0;
}

static int
gw_get_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	*var = gw.fb.var;
	var->nonstd = (gw.ypos << 10) | gw.xpos;

	return 0;
}

