/*
 * include/asm-sh/preem_latency.h
 * timing support for preempt-stats patch
 */

#ifndef _ASM_PREEM_LATENCY_H
#define _ASM_PREEM_LATENCY_H

#include <linux/config.h>
#include <asm/io.h>
#include <asm/timex.h>
#include <asm/div64.h>

#if defined(__sh3__)
#define TMU1_TCNT     0xfffffea4      /* Long access */
#elif defined(__SH4__)
#define TMU1_TCNT     0xffd80018      /* Long access */
#endif

#define readclock_init()
#define readclock(v) v = (unsigned long)ctrl_inl(TMU1_TCNT)
#define clock_to_usecs(clocks)	((clocks)/(CLOCK_TICK_RATE/1000000))
#define TICKS_PER_USEC CLOCK_TICK_RATE/1000000

#define CLOCK_COUNTS_DOWN

#define INTERRUPTS_ENABLED(x) ((x & 0x000000f0) == 0)

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
#ifdef CONFIG_KFI
static unsigned long __noinstrument do_gettmu1cycles(void)
{
	return 0 - (unsigned long)ctrl_inl(TMU1_TCNT);
}

/*
 * (u32)usec = (u32)cycles * 10^6 / ((u32)CONFIG_SH_PCLK_FREQ/4) [us]
 */
static unsigned long __noinstrument tmu1cycles_to_usecs(unsigned long cycles)
{
	u64 temp;
	temp = 4000000*(u64)cycles + CONFIG_SH_PCLK_FREQ/2;
	temp /= CONFIG_SH_PCLK_FREQ;
	return (unsigned long)temp;
}
#define kfi_readclock do_gettmu1cycles
#define kfi_clock_to_usecs(cycles) tmu1cycles_to_usecs(cycles)
#define do_getmachinecycles do_gettmu1cycles
#define machinecycles_to_usecs(cycles) tmu1cycles_to_usecs(cycles)
#endif
#endif /* _ASM_PREEM_LATENCY_H */
