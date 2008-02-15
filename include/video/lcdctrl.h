/*
 *  lcdctrl.h
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
#ifndef __LCD_CONTROL_H
#define __LCD_CONTROL_H

#define _LCDCTRL_IOCTL_ON		1
#define _LCDCTRL_IOCTL_OFF		2
#define _LCDCTRL_IOCTL_INTENSITY	3
#define _LCDCTRL_IOCTL_BRIGHTNESS	4
#define _LCDCTRL_IOCTL_CONTRAST		5
#define _LCDCTRL_IOCTL_GET_BRIGHTNESS	6
#define _LCDCTRL_IOCTL_GET_CONTRAST	7
#define _LCDCTRL_IOCTL_GET_INTENSITY	8

#define _LCD_CONTROL_NAME "lcdctrl"

#define LCD_NO_SYNC     0
#define LCD_SYNC_NEEDED 1

int lcdctrl_enable( void);
int lcdctrl_disable( void);

/* intensity, contrast, and brightness take values
 * between 0..100.
 */
int lcdctrl_set_intensity( int intensity);
int lcdctrl_set_contrast( int contrast, int sync);
int lcdctrl_set_brightness( int brightness);

int lcdctrl_get_intensity( void);
int lcdctrl_get_contrast( void);
int lcdctrl_get_brightness( void);

struct lcdctrl_device
{
	int (*init)( int*, int*, int*);
	int (*enable)(void);
	int (*disable)(void);
	int (*set_intensity)( int i);
	int (*set_brightness)( int b);
	int (*set_contrast)( int c, int sync);
};

int lcdctrl_init( void);

#endif
