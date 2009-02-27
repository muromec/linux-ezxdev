/*
 * include/asm-sh/preem_latency.h
 * timing support for preempt-stats patch
 */

#ifndef _ASM_PREEM_LATENCY_H
#define _ASM_PREEM_LATENCY_H

#include <asm/msr.h>		/* To be sure we have rdtscl definition */
#include <asm/timex.h>		/* To be sure we have cpu_khz declaration */
#include <asm/div64.h>		/* do_div */

/*
 * rdtsc is definded in asm/msr.h as
 * __asm__ __volatile__ ("rdtsc" : "=a" (low) : : "edx")
 */

#define readclock(low) rdtscl(low)
#define readclock_init()
#define clock_to_usecs(clocks) ({ \
	unsigned long long quot = (unsigned long long) (clocks) * 10; \
	do_div(quot, (unsigned long)(cpu_khz / 100)); \
	quot; })

#define        INTR_IENABLE            0x200
#define INTERRUPTS_ENABLED(x)   (x & INTR_IENABLE)
#define TICKS_PER_USEC	cpu_khz / 1000
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

#endif /* _ASM_PREEM_LATENCY_H */
