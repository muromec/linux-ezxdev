#ifndef __ASM_XTENSA_TIMEX_H
#define __ASM_XTENSA_TIMEX_H

/*
 * include/asm-xtensa/timex.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#include <xtensa/config/core.h>

#if XCHAL_INT_LEVEL(XCHAL_TIMER0_INTERRUPT) == 1
#define LINUX_TIMER     0
#elif XCHAL_INT_LEVEL(XCHAL_TIMER1_INTERRUPT) == 1
#define LINUX_TIMER     1
#elif XCHAL_INT_LEVEL(XCHAL_TIMER2_INTERRUPT) == 1
#define LINUX_TIMER     2
#else
#error "Bad timer number for Linux configurations!"
#endif

#define LINUX_TIMER_INT         XCHAL_TIMER_INTERRUPT(LINUX_TIMER)
#define LINUX_TIMER_MASK        (1L << LINUX_TIMER_INT)

/* Striving for a 60 Hz system clock, assume 1 MHz during iss and 10
   MHz on the simulation hardware (XT2000?) */

#ifdef XTENSA_ISS
#define	CLOCK_TICK_RATE	1000000
#else
extern unsigned clock_tick_rate;
#define	CLOCK_TICK_RATE	clock_tick_rate
#endif

#define LINUX_TIMER_INTERVAL ( CLOCK_TICK_RATE/HZ )

typedef unsigned long long cycles_t;

extern cycles_t cacheflush_time;

static __inline__ cycles_t get_cycles (void)
{
	return 0;
}

#define vxtime_lock()		do {} while (0)
#define vxtime_unlock()		do {} while (0)

#endif /* __ASM_XTENSA_TIMEX_H */
