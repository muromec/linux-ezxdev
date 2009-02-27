/*
 * include/asm-arm/arch-sa1100/preem_latency.h
 * timing support for preempt-stats patch
 */

#include "hardware.h"

#define readclock_init()
#define readclock(x)		do { (x) = OSCR; } while (0)
#define clock_to_usecs(x)	((x) / 4) /* 4 is an approximation for 3.6864 */

