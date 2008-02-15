/*
 *  lcdctrl.c
 *
 *  Generic LCD control for brightness, contrast, etc.
 *  Device specific drivers implement a lcdctrl_device and
 *  provides access to it via lcdctrl_device_get_ops().
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

#include <video/lcdctrl.h>

/*
 * Set this to zero to remove all the debug statements via
 * dead code elimination.
 */
#define DEBUGGING       0

#if DEBUGGING
static unsigned int lcd_debug = DEBUGGING;
#else
#define lcd_debug 	0
#endif

/* -- prototypes -- */

static int lcdctrl_ioctl(struct inode * inode, struct file *filp,
			 unsigned int cmd , unsigned long arg);
static int lcdctrl_open(struct inode *inode, struct file *filp);
static int lcdctrl_close(struct inode *inode, struct file *filp);

/* -- variables -- */

struct lcdctrl_device *lcd_device;

static int intensity;
static int brightness;
static int contrast;

static int enabled;
static int sync_needed;
static int chrdev_major;

static struct file_operations lcdctrl_fops = {
	ioctl:		lcdctrl_ioctl,
	open:		lcdctrl_open,
	release:	lcdctrl_close
};

/* -- ioctl -- */

static int lcdctrl_ioctl(struct inode * inode, struct file *filp,
                         unsigned int cmd , unsigned long arg)
{
	int ret;
	ret = -EINVAL;

	if( lcd_debug)
	    printk(KERN_INFO "lcdctrl_ioctl: cmd=%d, arg=%ld\n", cmd, arg);

	switch(cmd)
	{
		case _LCDCTRL_IOCTL_ON:
			ret = lcdctrl_enable();
			break;
		case _LCDCTRL_IOCTL_OFF:
			ret = lcdctrl_disable();  
			break;
		case _LCDCTRL_IOCTL_INTENSITY:
			if ((arg >=0) && (arg <= 100))
				ret = lcdctrl_set_intensity(arg);
			break;
		case _LCDCTRL_IOCTL_BRIGHTNESS:
			if ((arg >=0) && (arg <= 100))
				ret = lcdctrl_set_brightness(arg);
			break;
		case _LCDCTRL_IOCTL_CONTRAST:
			if ((arg >=0) && (arg <= 100))
				ret = lcdctrl_set_contrast(arg, LCD_NO_SYNC);
			break;  
		case _LCDCTRL_IOCTL_GET_BRIGHTNESS:
			ret = brightness;
			break;
		case _LCDCTRL_IOCTL_GET_CONTRAST:
			ret = contrast;
			break;
		case _LCDCTRL_IOCTL_GET_INTENSITY:
			ret = intensity;
			break;

		default:
			printk(KERN_ERR "lcdctrl_ioctl: invalid ioctl\n");
			break;
	}

	return ret;
}

static int lcdctrl_open(struct inode *inode, struct file *filp)
{
//	MOD_INC_USE_COUNT;
	return 0;
}

static int lcdctrl_close(struct inode *inode, struct file *filp)
{
//	MOD_DEC_USE_COUNT;
	return 0;
}

/* -- -- */

int lcdctrl_enable( void)
{
	int result;

	if( enabled) return 0;

	result = lcd_device->enable();

	lcdctrl_set_intensity( intensity);
	lcdctrl_set_brightness( brightness);
	lcdctrl_set_contrast( contrast, sync_needed);
        sync_needed = LCD_NO_SYNC;

	enabled = 1;
	return result;
}

int lcdctrl_disable( void)
{
	enabled = 0;
	return lcd_device->disable();
}

int lcdctrl_set_intensity( int i)
{
	intensity = i;
	return lcd_device->set_intensity( i);
}

int lcdctrl_set_brightness( int b)
{
	brightness = b;
	return lcd_device->set_brightness( b);
}

int lcdctrl_set_contrast( int c, int sync)
{
	contrast = c;
	return lcd_device->set_contrast( c, sync);
}

int lcdctrl_get_intensity( void)
{
	return intensity;
}

int lcdctrl_get_brightness( void)
{
	return brightness;
}

int lcdctrl_get_contrast( void)
{
	return contrast;
}

/* -- -- */

/* the device specific driver should implement this */
struct lcdctrl_device *lcdctrl_device_get_ops(void);

int lcdctrl_init( void)
{
	int ret;

	lcd_device = lcdctrl_device_get_ops();

	if( !lcd_device)
	{
		printk(KERN_ERR "lcdctrl_init: No lcd_device registered.\n");
		return -EINVAL;
	}

	ret = lcd_device->init( &intensity, &brightness, &contrast);

        sync_needed = LCD_SYNC_NEEDED;

	if( ret == 0)
	{
		chrdev_major = 
			register_chrdev( 0,_LCD_CONTROL_NAME,&lcdctrl_fops);
		if( lcd_debug)
			printk(KERN_INFO "lcdctrl_init: OK\n");
	}
	return ret;
}
