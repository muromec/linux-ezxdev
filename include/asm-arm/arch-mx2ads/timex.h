/*
 *  linux/include/asm-arm/mx2ads/timex.h
 *
 *  Copyright (C) 1999 ARM Limited
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
 */

#ifndef __ASM_ARCH_TIMEX_H__
#define __ASM_ARCH_TIMEX_H__

extern unsigned long mx2_clock_rate;
extern unsigned long mx2_clock_divider;
extern unsigned long mx2_cycles_per_jiffy;

#define TIM_32KHZ       0x08
#define TIM_INTEN       0x10
#define TIM_PERCLK1     0x02
#define TIM_ENAB        0x01
#define TIM_FRR         0x100 /* Free-Run mode enable */

#define CLOCK_TICK_RATE          1000000
#define TICKS2USECS(x)	(x)
#define TICKS_PER_USEC 1

#endif
