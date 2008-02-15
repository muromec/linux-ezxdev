/*
 * Copyright (C) 2004 Motorola, Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 */

/* 
 * History:
 * Originally written by Jay Jia (w20091@motorola.com)
 */
                                                                              
#ifndef _MOTO_KEYLIGHT_H
#define _MOTO_KEYLIGHT_H

#include <linux/types.h>

/* device driver node information */
#define KEYLIGHT_DEV_NAME               "keylight"

/*
 * ioctl definitions
 */
                                                                              
/* ioctl cmd, the third arg should be brightness, which is 0 ~ 255. */
#define KEYLIGHT_SET_BRIGHTNESS _IOW('k', 1, unsigned char)

/* compatible with old version */
#define KEYLIGHT_MAIN_ON        _IO('k', 2)
#define KEYLIGHT_MAIN_OFF       _IO('k', 3)
#define KEYLIGHT_MAIN_SETTING   _IOW('k', 4, unsigned char)

/* obsolete cmd */
#define KEYLIGHT_AUX_ON         _IO('k', 5)
#define KEYLIGHT_AUX_OFF        _IO('k', 6)
#define KEYLIGHT_AUX_SETTING    _IOW('k', 7, unsigned char)

#endif /* _MOTO_KEYLIGHT_H */

