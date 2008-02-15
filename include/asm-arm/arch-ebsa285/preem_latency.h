/*
 * include/asm-arm/arch-ebsa285/preem_latency.h
 *
 * Support for preemptible kernel latency measurement times
 *
 * The readclock() macro must return the contents of a monotonic incremental
 * counter.
 *
 * NOTES:
 *
 * 1. This implementation uses the DEC21285 TIMER2 exclusively for preemptible
 *    latency measurements. It must not be used for any other purpose during
 *    preemptible kernel latency measurement.
 *
 * 2. This implementation assumes that DEC21285 fclk_in is 50MHz.
 *
 * 3. As currently configured, the maximum latency which can be measured is
 *    approx 5.4 seconds. If latency times exceed approx 5.4 seconds, overflow
 *    will occur.
 *
 */

#include <linux/config.h>
#include <asm/hardware/dec21285.h>

#define readclock_init()	do {					\
        *CSR_TIMER2_CLR = 0;						\
        *CSR_TIMER2_LOAD = 0;						\
        *CSR_TIMER2_CNTL = TIMER_CNTL_ENABLE | TIMER_CNTL_DIV16;	\
	} while (0)

#define readclock(x)	do {						\
        x = ~(*CSR_TIMER2_VALUE) << 8;					\
	} while (0)

#define clock_to_usecs(x)	((x >> 8) / 3)	/* Assumes 50Mhz fclk_in */
