/*
 * drivers/video/mx21ads/mx21tvout.h
 *
 * Register definitions for the FS453/454 TVOUT device
 *
 * Author: Ruslan Sushko <rsushko@ru.mvista.com, or source@mvista.com>
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __MX21TVOUT_H__
#define __MX21TVOUT_H__

#include <asm/arch/pll.h> /* Get MPLLin frequency */

#define FS453_GPIO_ACDOE                (31) /*ACD/OE (BLANK) */
#define FS453_GPIO_VSYNC                (29) /* VSYNC */
#define FS453_GPIO_HSYNC                (28) /* HSYNC */
#define FS453_GPIO_RS                   (25) /* Reset signal (TVRST)*/
#define FS453_GPIO_D0                   (24) /* Oscilator power Up (TVCLKENB)*/
#define FS453_GPIO_LD17                 (23)
#define FS453_GPIO_LD16                 (22)
#define FS453_GPIO_LD15                 (21)
#define FS453_GPIO_LD14                 (20)
#define FS453_GPIO_LD13                 (19)
#define FS453_GPIO_LD12                 (18)
#define FS453_GPIO_LD11                 (17)
#define FS453_GPIO_LD10                 (16) /* LCD data */
#define FS453_GPIO_LD09                 (15)
#define FS453_GPIO_LD08                 (14)
#define FS453_GPIO_LD07                 (13)
#define FS453_GPIO_LD06                 (12)
#define FS453_GPIO_LD05                 (11)
#define FS453_GPIO_LD04                 (10)
#define FS453_GPIO_LD03                 (9)
#define FS453_GPIO_LD02                 (8)
#define FS453_GPIO_LD01                 (7)
#define FS453_GPIO_LD00                 (6)
#define FS453_GPIO_LSCLK                (5)  /* LSCLK */


/* GPIO masks */
#define FS453_GPIO_LSCLK_MASK           (1 << FS453_GPIO_LSCLK)
#define FS453_GPIO_LD_MASK              (\
                                        (1 << FS453_GPIO_LD17)|\
                                        (1 << FS453_GPIO_LD16)|\
                                        (1 << FS453_GPIO_LD15)|\
                                        (1 << FS453_GPIO_LD14)|\
                                        (1 << FS453_GPIO_LD13)|\
                                        (1 << FS453_GPIO_LD12)|\
                                        (1 << FS453_GPIO_LD11)|\
                                        (1 << FS453_GPIO_LD10)|\
                                        (1 << FS453_GPIO_LD09)|\
                                        (1 << FS453_GPIO_LD08)|\
                                        (1 << FS453_GPIO_LD07)|\
                                        (1 << FS453_GPIO_LD06)|\
                                        (1 << FS453_GPIO_LD05)|\
                                        (1 << FS453_GPIO_LD04)|\
                                        (1 << FS453_GPIO_LD03)|\
                                        (1 << FS453_GPIO_LD02)|\
                                        (1 << FS453_GPIO_LD01)|\
                                        (1 << FS453_GPIO_LD00))
#define FS453_GPIO_RS_MASK              (1 << FS453_GPIO_RS)
#define FS453_GPIO_D0_MASK              (1 << FS453_GPIO_D0)
#define FS453_GPIO_VSYNC_MASK           (1 << FS453_GPIO_VSYNC)
#define FS453_GPIO_HSYNC_MASK           (1 << FS453_GPIO_HSYNC)
#define FS453_GPIO_ACDOE_MASK           (1 << FS453_GPIO_ACDOE)

#define LSCLK_FREQ              (19000000ull)
#define FS453PLL_FREQ           (266000000ull)

typedef enum {
    TVOUT_MODE_PAL625        = 0,    /* PAL 625 lines/50 Hz */
    TVOUT_MODE_NTSC358       = 1,    /* NTSC 3.58 */
    TVOUT_MODE_MAX           = 2     /* Just for comfort  */
} tvmode_t;

typedef struct {
    unsigned int totlines;      /* total lines */
    unsigned int actlines;      /* num of active lines */
    unsigned int totpixels;     /* total pixels per line */
    unsigned int actpixels;     /* active pixels */
    unsigned int fps;           /* frames per seconds */
} tvmodeparam_t;

typedef struct {
    tvmode_t mode;              /* Video mode */
    unsigned int totlines;      /* total lines */
    unsigned int actlines;      /* num of active lines */
    unsigned int totpixels;     /* total pixels per line */
    unsigned int actpixels;     /* active pixels */
} tvframe_t;

typedef enum {
    TVOUT_PORT_COMPOSIT,
    TVOUT_PORT_SVIDEO
} tvout_port_t;

int mx2_tvout_init(int xres, int yres, tvmode_t mode, unsigned int
        frame_buf_addr);
void mx2_tvout_exit(void);

#endif /* __MX21TVOUT_H__ */
