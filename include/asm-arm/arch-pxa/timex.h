/*
 * linux/include/asm-arm/arch-pxa/timex.h
 *
 * Author:	Nicolas Pitre
 * Created:	Jun 15, 2001
 * Copyright:	MontaVista Software Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>

#ifndef CONFIG_CPU_BULVERDE

/*
 * PXA250/210 timer
 */
#define CLOCK_TICK_RATE		3686400

#else

/*
 * Bulverde timer
 */
#ifdef CONFIG_ARCH_MAINSTONE
#define CLOCK_TICK_RATE		3249600
#else
#define CLOCK_TICK_RATE		3250000
#endif

#endif

#define CLOCK_TICK_FACTOR	80
