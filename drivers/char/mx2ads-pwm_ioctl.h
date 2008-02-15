/*
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Copyright (C) 2002 Motorola Semiconductors HK Ltd
 * Copyright (C) 2003 MontaVista Software Inc. <source@mvista.com>
 *
 */

#ifndef PWM_DEF_H
#define PWM_DEF_H

#include <linux/ioctl.h>

#define PWM_SAMPLING_8KHZ	8000
#define PWM_SAMPLING_16KHZ	16000
#define PWM_SAMPLING_32KHZ	32000

#define PWM_DATA_8BIT		8
#define PWM_DATA_16BIT		16

#define PWM_SWAP_HCTRL		((unsigned long)0x01<<18)
#define PWM_SWAP_BCTRL		((unsigned long)0x01<<17)

#define	PWM_IOC_MAGIC		'p'
#define	PWM_IOC_SFREQ		_IO(PWM_IOC_MAGIC, 0)
#define	PWM_IOC_SDATALEN	_IO(PWM_IOC_MAGIC, 1)
#define	PWM_IOC_SMODE		_IO(PWM_IOC_MAGIC, 2)
#define	PWM_IOC_SSAMPLE		_IO(PWM_IOC_MAGIC, 3)
#define	PWM_IOC_SPERIOD		_IO(PWM_IOC_MAGIC, 4)
#define PWM_IOC_STOP		_IO(PWM_IOC_MAGIC, 5)
#define PWM_IOC_SWAPDATA	_IO(PWM_IOC_MAGIC, 6)

#define	PWM_PLAY_MODE		1
#define	PWM_TONE_MODE		2

#endif
