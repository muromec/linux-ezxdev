/*
 * include/asm-arm/arch-adifcc/preem_latency.h
 *
 * Timing support for preemptible kernel latency measurement times
 *
 * The readclock() macro must return the contents of a monotonic incremental
 * counter. The Intel 80200 Performance Monitor clock counter register is
 * ideal for this purpose as it is a 32-bit up counter which is optionally
 * clocked directly from the processor core clock or the core clock divided
 * by 64.
 *
 * For now, we use the processor core clock frequency option and assume that
 * the frequency is 400MHz. This provides ability to measure delays of up to
 * approx 5.8 seconds before "overflow" occurs. If anything holds a lock this
 * long, we've got serious problems. Anyway, if this isn't long enough, we can
 * use the divide by 64 option for a max delay of approx 375 seconds before
 * overflow.
 *
 *
 * NOTES:
 *
 * 1. This implementation aquires the XScale PMU exclusively for preemptible
 *    latency measurements. The PMU cannot be used for other performance
 *    measurements concurrently with preemption latency measurement support.
 *
 * 2. This initial implementation assumes a constant processor core clock
 *    frequency of 400MHz. These macros should be modified to support
 *    arbitrary processor frequencies.
 *
 * 3. We manually start the PMU rather than starting it via pmu_start() to
 *    to avoid enabling PMU overflow interrupts.
 * 
 */

#include <linux/config.h>
#include <asm/xscale-pmu.h>

#define readclock_init()	do {					\
		unsigned int pmnc = 3; /* PMU_ENABLE | PMU_RESET */	\
		int status = pmu_claim();				\
		if (status < 0)						\
			panic("Can't get PMU lock for preemptible "	\
				"kernel latency measurements!");	\
		__asm__("mcr    p14, 0, %0, c0, c0, 0"  		\
			: : "r" (pmnc));                		\
	} while (0)

#define readclock(x)	do {				\
		__asm__("mrc	p14, 0, %0, c1, c0, 0"	\
			: "=r" (x));			\
	} while (0)

#define clock_to_usecs(x)	((x) / 400) /* Assumes 400Mhz CCLK */
