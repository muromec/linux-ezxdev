#ifndef PWMLED_H
#define PWMLED_H

/* 
 * drivers/misc/pwm-led.h
 *
 * Copyright (C) 2004-2005 Motorola
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
 * original written by Jay Jia (w20091@motorola.com)
 */


#ifdef __cplusplus
extern "C" {
#endif
	
#define PWMLED_MINOR 		168
#define PWMLED_0_ON    		0xf0
#define PWMLED_1_ON		0xf1
#define PWMLED_0_OFF		0xf2
#define PWMLED_1_OFF		0xf3
#define PWMLED_0_SETTING  	0xf4
#define PWMLED_1_SETTING 	0xf5	

#ifdef __cplusplus
}
#endif
#endif
  
