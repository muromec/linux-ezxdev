/* 
 * drivers/video/dbmx21fb.h
 *
 * MX21ADS LCD Controller framebuffer driver defintions
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>.
 *
 * This file is based on dbmx21fb.h from Motorola Dragonball MX2 ADS BSP
 * Copyright 2002, 2003 Motorola, Inc. All Rights Reserved.
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef _DBMX21FB_H
#define _DBMX21FB_H

#define DBMX21_NAME "dbmx21"

#define SHARP_TFT_240x320 /* default value */

#ifndef CONFIG_MX21_TVOUT
#ifdef SHARP_TFT_240x320
#define PANELCFG_VAL_12	0xf8008b48
#define HCFG_VAL_12	0x04000f06
#define VCFG_VAL_12	0x04000907
#define PWMR_VAL	0x0000008a
#define LCD_MAXX	240
#define LCD_MAXY	320
#else
#define PANELCFG_VAL_12	0xf8088c6b
#define HCFG_VAL_12	0x04000f06
#define VCFG_VAL_12	0x04010c03
#define PWMR_VAL	0x00000200
#define LCD_MAXX	320
#define LCD_MAXY	240
#endif

#else /*CONFIG_MX21_TVOUT*/
#define LCD_MAXX	640
#define LCD_MAXY	480
#endif /*CONFIG_MX21_TVOUT*/

#define PANELCFG_VAL_4	0x20008c09
#define PANELCFG_VAL_4C	0x60008c09

#define HCFG_VAL_4	0x04000f07

#define VCFG_VAL_4	0x04010c03

#define REFMCR_VAL_4	0x00000003
#define REFMCR_VAL_12	0x00000003
#define DISABLELCD_VAL	0x00000000

#define DMACR_VAL_4	0x800c0003	/* 12 & 3 TRIGGER */
#define DMACR_VAL_12	0x00020008

#define INTCR_VAL_4	0x00000000
#define INTCR_VAL_12	0x00000000

#define INTSR_UDRERR	0x00000008
#define INTSR_ERRRESP	0x00000004
#define INTSR_EOF	0x00000002
#define INTSR_BOF	0x00000001

#define MIN_XRES        64
#define MIN_YRES        64

#define LCD_MAX_BPP	16
	
#define MAX_PALETTE_NUM_ENTRIES         256
#define MAX_PIXEL_MEM_SIZE \
    ((current_par.max_xres * current_par.max_yres * current_par.max_bpp)/8)
#define MAX_FRAMEBUFFER_MEM_SIZE \
		(MAX_PIXEL_MEM_SIZE + 32)
#define ALLOCATED_FB_MEM_SIZE \
		(PAGE_ALIGN(MAX_FRAMEBUFFER_MEM_SIZE + PAGE_SIZE * 2))

#define FBCON_HAS_CFB4
#define FBCON_HAS_CFB8
#define FBCON_HAS_CFB16

#define MAX_PALETTE_NUM_ENTRIES 	256
#define DEFAULT_CURSOR_BLINK_RATE 	20
#define CURSOR_DRAW_DELAY  		2

/* cursor status */

#define LCD_CURSOR_OFF		0
#define LCD_CURSOR_ON		1

#define LCD_CURSOR_INVERT_BGD	2
#define LCD_CURSOR_AND_BGD	3
#define LCD_CURSOR_OR_BGD       4		
#define LCD_CURSOR_XOR_BGD      5

#define CURSOR_INVERT_MASK	0x80000000
#define CURSOR_AND_BGD_MASK	0xC0000000
#define CURSOR_OR_BGD_MASK	0x50000000
#define CURSOR_XOR_BGD_MASK	0x90000000

/* MASK use for caculating MCDUPLLCLK */
#define PCLKDIV2_MASK		0x03c00000
#define PCD_MASK		0x0000003F
#define XMAX_MASK		0xF3F00000
#define YMAX_MASK		0x000001FF
#define HWAIT1_MASK		0x0000FF00
#define HWAIT2_MASK		0x000000FF
#define HWIDTH_MASK		0xFC000000
#define PASSDIV_MASK		0x00FF0000
#define VWAIT1_MASK		0x0000FF00
#define VWAIT2_MASK		0x000000FF
#define VWIDTH_MASK		0xFC000000
#define CURSORBLINK_DIS_MASK	0x80000000

/*  MASK use for indicating the cursor status  */
#define CURSOR_ON_MASK		0x40000000
#define CURSOR_OFF_MASK		0x0FFFFFFF
#define MAX_CURSOR_WIDTH	31
#define MAX_CURSOR_HEIGHT	31
#define CURSORBLINK_EN_MASK	0x80000000

#define DISPLAY_MODE_MASK	0x80000000

#define LCD_FREQ                (33250000)

/* Graphic window */

#define LGWSR_GWW_BIT           20
#define LGWSR_GWH_BIT           0
#define LGWPR_GWXP_MASK         0x000003FF
#define LGWPR_GWYP_MASK         0x000003FF
#define LGWPR_GWXP_BIT          16
#define LGWPR_GWYP_BIT          0
#define LGWCR_GWCKB_MASK        0x0000001F
#define LGWCR_GWCKB_BIT         0
#define LGWCR_GWCKG_MASK        0x0000003F
#define LGWCR_GWCKG_BIT         6
#define LGWCR_GWCKR_MASK        0x0000001F
#define LGWCR_GWCKR_BIT         12
#define LGWCR_GWALPHA_MASK      0x000000FF
#define LGWCR_GWALPHA_BIT       24

#define LGWCR_GWE               0x00400000
#define LGWCR_GWCKE             0x00800000

#endif /* _DBMX21FB_H */
