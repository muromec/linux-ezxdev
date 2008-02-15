/*
 *  linux/include/asm-arm/arch-mx2ads/preem_latency.h
 *
 *  Copyright (C) 2004 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef __ASM_ARCH_PREEM_LATENCY_H
#define __ASM_ARCH_PREEM_LATENCY_H

#include <asm/hardware.h>
#include <linux/timex.h>
#include <asm/arch/pll.h>
#include <asm/proc/ptrace.h>

#define INTERRUPTS_ENABLED(x)   (!(x & PSR_I_BIT))

#define readclock_init()
#define readclock(_x)		\
        do {                         \
                (_x) = GPT_TCN(GPT3);  \
        } while (0)

/* We have enabled 2 Mhz timer */

static inline
unsigned long clock_to_usecs(unsigned long ticks)
{
	return TICKS2USECS(ticks);
}

extern inline
unsigned long __noinstrument machinecycles_to_usecs(unsigned long mputicks)
{
	return TICKS2USECS(mputicks);
}

extern inline
unsigned long __noinstrument do_getmachinecycles(void)
{
unsigned long ticks = 0;
	if (!mx2_cycles_per_jiffy) return 0;
	readclock(ticks);
	return ticks;
}


#endif /* __ASM_ARCH_PREEM_LATENCY_H */
