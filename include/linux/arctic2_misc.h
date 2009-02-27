/*
 * arctic2_misc.h
 */

#ifndef _ARCTIC2_MISC_H
#define _ARCTIC2_MISC_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define ARCTIC2_LED_CTRL_DDS		0x01
#define ARCTIC2_LED_CTRL_BLUE		0x02
#define ARCTIC2_LED_CTRL_GREEN		0x04
#define ARCTIC2_LED_CTRL_RED		0x08
#define ARCTIC2_LED_CTRL_CLOCKDIV	0xf0
 
#define ARCTIC2_LED_TIMING_COLOR	0xff000000
#define ARCTIC2_LED_TIMING_ON		0x7f00
#define ARCTIC2_LED_TIMING_OFF		0x007f
#define ARCTIC2_LED_MKTIMING(col, on, off) ((unsigned)(col) << 24 | \
                                            (unsigned)(on) << 8 | (off))
#define ARCTIC2_LED_RED			0
#define ARCTIC2_LED_GREEN		1
#define ARCTIC2_LED_BLUE		2
 
#define ARCTIC2_PWR_VOLTAGE		0x0fff

#define ARCTIC2_MISC_IOCTYPE		0xB3
 
#define ARCTIC2_MISC_LEDSTANDBY	_IOW(ARCTIC2_MISC_IOCTYPE, 0, int)
#define ARCTIC2_MISC_LEDCTRL	_IOW(ARCTIC2_MISC_IOCTYPE, 1, uint32_t)
#define ARCTIC2_MISC_LEDTIMING	_IOW(ARCTIC2_MISC_IOCTYPE, 2, uint32_t)

#define ARCTIC2_MISC_FLSTATUS	_IO(ARCTIC2_MISC_IOCTYPE, 3)

#define ARCTIC2_MISC_POWEROFF	_IO(ARCTIC2_MISC_IOCTYPE, 4)
#define ARCTIC2_MISC_PWRSTATUS	_IOR(ARCTIC2_MISC_IOCTYPE, 5, uint32_t)

#endif /* _ARCTIC2_MISC_H */
