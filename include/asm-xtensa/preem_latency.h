/*
 * include/asm-xtensa/preem_latency.h
 * timing support for preempt-stats patch
 */

#ifndef _ASM_PREEM_LATENCY_H
#define _ASM_PREEM_LATENCY_H

#include <asm/xtensa/hal.h>	/* xthal_get_ccount */
#include <asm/timex.h>		/* CLOCK_TICK_RATE */

#define readclock(low) (low = xthal_get_ccount())
#define readclock_init()

#define clock_to_usecs(clocks) ((clocks * 10) / (CLOCK_TICK_RATE / 100000))

#endif /* _ASM_PREEM_LATENCY_H */
