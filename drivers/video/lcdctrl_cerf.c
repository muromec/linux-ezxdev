/*
 *  lcdctrl_cerf.c
 *
 *  Cerf LCD control for brightness and contrast.
 *
 *  Copyright (C) 2002 Intrinsyc Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    Mar 2002: Initial version [FB]
 * 
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/arch/cerf_ucb1400gpio.h>

#include <video/lcdctrl.h>

/*
 * Set this to zero to remove all the debug statements via
 * dead code elimination.
 */
#define DEBUGGING       0

#if DEBUGGING
static unsigned int lcd_debug = DEBUGGING;
#else
#define lcd_debug       0
#endif

#define LCD_MAX_INTENSITY	 0
#define LCD_MAX_BRIGHTNESS	15
#define LCD_MAX_CONTRAST	100

#define LCD_DEFAULT_INTENSITY	 0
#define LCD_DEFAULT_BRIGHTNESS	14*100/(LCD_MAX_BRIGHTNESS)
#define LCD_DEFAULT_CONTRAST	90*100/(LCD_MAX_CONTRAST)

#define UP	1
#define DOWN	0

/* -- prototypes -- */

static int cerf_lcdctrl_init( int *intensity, int *brightness, int *contrast);
static int cerf_lcdctrl_enable(void);
static int cerf_lcdctrl_disable(void);
static int cerf_lcdctrl_set_intensity( int i);
static int cerf_lcdctrl_set_brightness( int b);
static int cerf_lcdctrl_set_contrast( int c, int sync);

static void cerf_lcdctrl_contrast_step( int direction);

/* -- variables -- */

static int dev_contrast;

/* -- -- */

static struct lcdctrl_device cerf_dev = {
	init: 		cerf_lcdctrl_init,
	enable:		cerf_lcdctrl_enable,
	disable:	cerf_lcdctrl_disable,
	set_intensity:	cerf_lcdctrl_set_intensity,
	set_brightness:	cerf_lcdctrl_set_brightness,
	set_contrast:	cerf_lcdctrl_set_contrast
}; 

static int cerf_lcdctrl_enable( void)
{
	cerf_ucb1400gpio_lcd_enable();

	return 0;
}

static int cerf_lcdctrl_disable( void)
{
	cerf_ucb1400gpio_lcd_disable();

	return 0;
}

static int cerf_lcdctrl_set_intensity( int i)
{
	int dev_intensity = LCD_MAX_INTENSITY*i/100;
	if( lcd_debug)
		printk(KERN_INFO "cerf_lcdctrl_set_intensity: "
			"dev_intensity = %d\n", dev_intensity);
	return 0;
}

static int cerf_lcdctrl_set_brightness( int b)
{
	int dev_brightness = LCD_MAX_BRIGHTNESS*b/100;
	outw( dev_brightness, CERF_PDA_CPLD+CERF_PDA_CPLD_BRIGHTNESS);
	if( lcd_debug)
		printk(KERN_INFO "cerf_lcdctrl_set_brightness: "
			"dev_brightness = %d\n", dev_brightness);
	return 0;
}

static int cerf_lcdctrl_set_contrast( int c, int sync)
{
	int new_dev_contrast = LCD_MAX_CONTRAST*c/100;
	int i;
	int count;
	int direction = UP;
	if( sync == LCD_SYNC_NEEDED)
	{
		/* In order to sync we step down to the lowest contrast level */
		for( i=0; i<LCD_MAX_CONTRAST; i++)
			cerf_lcdctrl_contrast_step(DOWN);
		dev_contrast = 0;
	}

	count = new_dev_contrast - dev_contrast;
	if( count < 0)
	{
		/* new contrast is lower then current setting */
		direction = DOWN;
		count = -count;
	}

	for( i=0; i<count; i++)
		cerf_lcdctrl_contrast_step(direction);

	if( lcd_debug)
		printk(KERN_INFO "cerf_lcdctrl_set_contrast: "
			"dev_contrast = %d\n", new_dev_contrast);
	dev_contrast = new_dev_contrast;

	return 0;
}

/* -- -- */

static void cerf_lcdctrl_contrast_step( int direction)
{
	cerf_ucb1400gpio_lcd_contrast_step( direction);
}

/* -- -- */

static int cerf_lcdctrl_init( int *intensity, int *brightness, int *contrast)
{
	*intensity = LCD_DEFAULT_INTENSITY;
	*brightness = LCD_DEFAULT_BRIGHTNESS;
	*contrast = LCD_DEFAULT_CONTRAST;

	if( lcd_debug)
		printk(KERN_INFO "cerf_lcdctrl_init: OK\n");
	return 0;
}

/* this is the hook for lcdctrl to access to the device specifics */
struct lcdctrl_device *lcdctrl_device_get_ops(void)
{
	return &cerf_dev;
}
