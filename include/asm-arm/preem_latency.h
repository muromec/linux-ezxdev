/*
 * include/asm-arm/preem_latency.h
 * timing support for preempt-stats patch
 */

#ifndef _ASM_PREEM_LATENCY_H
#define _ASM_PREEM_LATENCY_H

static inline unsigned int clock_diff(unsigned int start, unsigned int stop)
{
#ifdef CONFIG_CLOCK_COUNTS_DOWN
#ifdef CONFIG_24BIT_COUNTER
	if (stop < start) {
		stop = 0xffffff - stop; 
	}
#endif /* CONFIG_24BIT_COUNTER */
	return (start - stop);
#else	/* CONFIG_CLOCK_COUNTS_DOWN */
#ifdef CONFIG_24BIT_COUNTER
	if (start < stop) {
		start = 0xffffff - start; 
	}
#endif /* CONFIG_24BIT_COUNTER */
	return (stop - start);
#endif /* CLOCK_COUNTS_DOWN */
}
#include <asm/arch/preem_latency.h>

#endif /* _ASM_PREEM_LATENCY_H */
