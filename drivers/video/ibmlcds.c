/*
 * drivers/video/ibmlcds.c -- 
 *    Database of known LCD panels' specs for use with 
 *    IBM Liquid Crystal Display Controller core
 *    - original use in PowerPC 405LP embedded platform
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *  Copyright (C) 2001 David T. Eger   <eger@cc.gatech.edu>
 *                     Matthew Helsley <mhelsle@us.ibm.com>
 *                     Bishop Brock    <bcbrock@us.ibm.com>
 *
 */
/*
 * Below is a collection of structures, each one defining the capabilities
 * and default video mode for an LCD.  The values you enter here for resolution
 * are taken to be the exact capabilities of the LCD, and cannot be changed at
 * run time.  Some of the other characteristics, such as bit-depth and virtual
 * resolution can be changed at run time.  
 *
 * In order to add a new profile, you need to do two things:
 *    (1) Create a new structure describing your LCD panel
 *        This involves getting the proper values to write to the
 *        LCDC's registers.  A lot of them you can figure out, most of them,
 *        you'll need to consult:
 *         - "Liquid Crystal Display Controller Core" 
 *            SA14-2342-00 IBM Microelectronics Division
 *            (NOTE: IBM Confidential)
 *         - linux/drivers/video/ibmlcd.h - contains definition of the 
 *            par structure used here, what the fields mean,  and what 
 *            ranges are valid for the given fields.
 *    (2) Add your structure, and one or more unique aliases to it to 
 *        ibmlcd_configs at the bottom of this file.
 *    (3) Use the alias as a command line argument to the kernel.
 *
 * TODO: + Change the timing information presented here from precise values,
 *         to a usable range, for use with frequency-scaling and dynamic 
 *         reprogramming of clock dividers
 *       + Pass the physical screen size in mm width and height, to put in
 *         the var struct.
 */

#include "ibmlcd.h"
#include <linux/kernel.h>
#include <linux/autoconf.h>

#ifdef CONFIG_BEECH
static const struct ibmlcdfb_par HITACHI_QVGA_STN = {
	/* DCR */
	.reduced_horiz_blanking		= 1,
	.tft_multiplex_ratio		= 1,
	.FPSHIFT_clocks			= 1,
	.pixel_clock_per_shift_clock	= 3,
	.n_scan_mode			= 1,
	.LCD_panel_size			= STN_8BIT,
	.LCD_panel_type			= IBMLCD_COLOR_STN,

	/* DFRMR */
	.FRM_bits			= 4,
	.dither_bits			= 2,
	.native_resolution_bits		= 1,

	/* PSR */
	.FPSHIFT_delay			= 12,
	.FPFRAME_delay			= 12,
	.FP_VEE_EN_delay		= 13,
	.FP_EN_delay			= 14,

	/* ADSR */
	.horiz_pixels			= 320,
	.vert_pixels			= 240,

	/* TDSR */
	.total_horiz_pixels		= 328,
	.total_vert_pixels		= 245,

	.FPLINE_mask_during_v_blank	= 0,
	.FPLINE_polarity_negative	= 0,

	.FPLINE_hoff_start		= 122,
		/* 122 * 8/3 =  325; Yes this looks strange. */
		/* Basic idea - setting hoff_start to zero resulted in screen
		 * being shifted to the right several pixels.  To correct this,
		 * we set the value here to the equivalent of -3 pixels in the
		 * data stream.  -3 = 325 mod 328.
		 * Why the 8/3?  The Controller sees 8 bit values, but sends
		 * 3 bit values packed to the LCD.  Yeah, funky, like I said.
		 * Koji Ishii was the mastermind to figure this value out */
	.FPLINE_hoff_end		= 0,

	.FPFRAME_hoff			= 1,
	.FPFRAME_polarity_negative	= 0,

	.FPFRAME_voff_start		= 0,
	.FPFRAME_voff_end		= 1,

	.FPSHIFT_masking		= IBMLCD_FPSHIFT_NO_MASKING,
	.FPSHIFT_valid_at_positive_edge	= 0,

	.FPDRDY_polarity_negative	= 0,
	.FPDATA_polarity_negative	= 0,

	.pixels_big_endian		= 1,
	.pixel_packing			= IBMLCD_RGB,
	.pixel_size			= IBMLCD_PIX_16BPP,
	.pixel_index_size		= IBMLCD_PAL_8BPP,
	.palette_enable			= 0,
	.enable_surface			= 1,

	.fb_base_address		= 0, /* dummy value */
	.stride				= 320 * 2, /* 320 pixels, 2 bytes per pixel */
	.cursor_enable			= 0,
	.cursor_base_address		= 0, /* dummy value */
	.cursor_x			= 0,
	.cursor_y			= 0,

	.cc0r				= 0,
	.cc0g				= 0,
	.cc0b				= 0,

	.cc1r				= 0xFF,
	.cc1g				= 0xFF,
	.cc1b				= 0xFF,

	/* The Hitachi STN is specified for a Pixel clock in the range of 
	   6 MHz to 8 Mhz.  CLock frequencies are in KHz */
	.pixclk_min			= 6000,
	.pixclk_max			= 8000
};

static const struct ibmlcdfb_par TOSHIBA_VGA_TFT = {
	/* DCR */
	.reduced_horiz_blanking		= 1,
	.tft_multiplex_ratio		= 1,
	.FPSHIFT_clocks			= 1,
	.pixel_clock_per_shift_clock	= 1,
	.n_scan_mode			= 1,
	.LCD_panel_size			= TFT_18BIT,
	.LCD_panel_type			= IBMLCD_COLOR_TFT,

	/* FRMR */
	.FRM_bits			= 0,
	.dither_bits			= 0,
	.native_resolution_bits		= 6,

	/* PSR */
	.FPSHIFT_delay			= 12,
	.FPFRAME_delay			= 12,
	.FP_VEE_EN_delay		= 13,
	.FP_EN_delay			= 14,

	/* ADSR */
	.horiz_pixels			= 640,
	.vert_pixels			= 480,

	/* TDSR */
	.total_horiz_pixels		= 800,
	.total_vert_pixels		= 525,

	.FPLINE_mask_during_v_blank	= 1,
	.FPLINE_polarity_negative	= 1,

	.FPLINE_hoff_start		= 640, /* See comment in the first configuration */
	.FPLINE_hoff_end		= 0,

	.FPFRAME_hoff			= 1,
	.FPFRAME_polarity_negative	= 1,

	.FPFRAME_voff_start		= 480,
	.FPFRAME_voff_end		= 0,

	.FPSHIFT_masking		= IBMLCD_FPSHIFT_NO_MASKING,
	.FPSHIFT_valid_at_positive_edge	= 0,

	.FPDRDY_polarity_negative	= 0,
	.FPDATA_polarity_negative	= 0,

	.pixels_big_endian		= 1,
	.pixel_packing			= IBMLCD_RGB,
	.pixel_size			= IBMLCD_PIX_16BPP,
	.pixel_index_size		= IBMLCD_PAL_8BPP,
	.palette_enable			= 0,
	.enable_surface			= 1,

	.fb_base_address		= 0, /* dummy value */
	.stride				= 640 * 2, /* 640 pixels, 2 bytes per pixel */
	.cursor_enable			= 0,
	.cursor_base_address		= 0, /* dummy value */
	.cursor_x			= 0,
	.cursor_y			= 0,

	.cc0r				= 0,
	.cc0g				= 0,
	.cc0b				= 0,

	.cc1r				= 0xFF,
	.cc1g				= 0xFF,
	.cc1b				= 0xFF,

	/* The Toshiba TFT is specified for a Pixel clock in the range of 
	   21.5 MHz to 28.6 Mhz.  Clock frequencies are in KHz */
	.pixclk_min			= 21500,
	.pixclk_max			= 28600
};

#endif /* CONFIG_BEECH */

#if defined(CONFIG_ARCTIC2)
static const struct ibmlcdfb_par TOSHIBA_LCD_CONFIGA2 = {
	/* DCR */
	.reduced_horiz_blanking		= 1,
	.tft_multiplex_ratio		= 0,
	.FPSHIFT_clocks			= 0,
	.pixel_clock_per_shift_clock	= 1,
	.n_scan_mode			= 1,
	.LCD_panel_size			= TFT_15BIT,
	.LCD_panel_type			= IBMLCD_COLOR_TFT,

	/* FRMR */
	.FRM_bits			= 0,
	.dither_bits			= 0,
	.native_resolution_bits		= 6,

	/* PSR */
	.FPSHIFT_delay			= 12,
	.FPFRAME_delay			= 0,
	.FP_VEE_EN_delay		= 13,
	.FP_EN_delay			= 14,

	/* ADSR */
	.horiz_pixels			= 240,
	.vert_pixels			= 319,

	/* TDSR */
	.total_horiz_pixels		= 280,
	.total_vert_pixels		= 336,

	.FPLINE_mask_during_v_blank	= 0,
	.FPLINE_polarity_negative	= 1,

	.FPLINE_hoff_start		= 252, /* See comment in the first configuration */
	.FPLINE_hoff_end		= 268,

	.FPFRAME_hoff			= 1,
	.FPFRAME_polarity_negative	= 1,

	.FPFRAME_voff_start		= 0x14D,        
	.FPFRAME_voff_end		= 0x14e,

	.FPSHIFT_masking		= IBMLCD_FPSHIFT_NO_MASKING,
	.FPSHIFT_valid_at_positive_edge	= 0,

	.FPDRDY_polarity_negative	= 0,
	.FPDATA_polarity_negative	= 0,

	.pixels_big_endian		= 1,
	.pixel_packing			= IBMLCD_RGB,
	.pixel_size			= IBMLCD_PIX_16BPP,
	.pixel_index_size		= IBMLCD_PAL_8BPP,
	.palette_enable			= 0,
	.enable_surface			= 1,

	.fb_base_address		= 0, /* dummy value */
	.stride				= 240*2, /* 240 pixels, 2 bytes per pixel */
	.cursor_enable			= 0,
	.cursor_base_address		= 0, /* dummy value */
	.cursor_x			= 0,
	.cursor_y			= 0,

	.cc0r				= 0,
	.cc0g				= 0,
	.cc0b				= 0,

	.cc1r				= 0xFF,
	.cc1g				= 0xFF,
	.cc1b				= 0xFF,

	/* The Toshiba TFT is specified for a Pixel clock in the range of
	   XX MHz to YY MHz.  CLock frequencies are in KHz */
	.pixclk_min			= 5000,
	.pixclk_max			= 6000,
};
#endif

/* Format: A sequence of database entries, terminated by two consecutive NULLs
 *	Each entry consists of:
 *	+ a pointer to a (constant) par description for the LCD,
 *	+ 1 or more pointers to (constant) descriptive strings / aliases,
 *	+ a NULL
 */
static const void *ibmlcd_configs[] = {
#ifdef CONFIG_BEECH
	/* First Entry */
	&HITACHI_QVGA_STN,
	"HitachiQVGA-STN",
	"HitachiSC09Q002-AZA",
	NULL,
	/* Second Entry */
	&TOSHIBA_VGA_TFT,
	"ToshibaVGA-TFT",
	"ToshibaLTM04C380K",
	NULL,
#endif /* CONFIG_BEECH */

#ifdef CONFIG_ARCTIC2
	&TOSHIBA_LCD_CONFIGA2,
	"ToshibaTFTA2",
	NULL,
#endif
	/* Last Entry */
	NULL,
};

const struct ibmlcdfb_par *ibmlcd_config_matching(char *description)
{
	void **entry = (void **)ibmlcd_configs;
	while ((*entry) != NULL) {
		char ** name = (char **)entry;

		while (*(++name) != NULL)
			if (strcmp(description, *name) == 0) {
				printk(KERN_INFO
				       "ibmlcdfb: Configuring for panel %s\n",
				       *name);
				return *entry;
			}
		entry = (void **)(++name);
	}
	return NULL;
}

const char *ibmlcd_config_name(const struct ibmlcdfb_par *par)
{
	const void **entry;

	for (entry = ibmlcd_configs; (*entry) != NULL; entry++) {
		if ((*entry) == par)
			return *(entry+1);
		else while (*entry != NULL) 
			entry++;
	}
	return "Invalid LCD Description";
}

/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
