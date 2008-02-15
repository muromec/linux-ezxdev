/*
 * include/asm-xtensa/delay.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001  Tensilica Inc.
 *
 */

#ifndef _ASM_DELAY_H
#define _ASM_DELAY_H

#include <linux/config.h>
#include <asm/processor.h>

extern __inline__ void __delay(unsigned long loops)
{
  __asm__ __volatile__ ("1: addi %0, %0, -1; bnez %0, 1b"
			: "=r" (loops) : "0" (loops));
}

static __inline__ u32 xtensa_get_ccount(void)
{
	u32 ccount;
	asm volatile ("rsr %0, 234; # CCOUNT\n" : "=r" (ccount));
	return ccount;
}

/* For SMP/NUMA systems, change boot_cpu_data to something like
 * local_cpu_data->... where local_cpu_data points to the current
 * cpu. */

static __inline__ void udelay (unsigned long usecs)
{
	unsigned long start = xtensa_get_ccount();
	unsigned long cycles = usecs * boot_cpu_data.udelay_val;

	/* Note: all variables are unsigned (can wrap around)! */
	while (((unsigned long)xtensa_get_ccount()) - start < cycles)
		;
}

#endif /* _ASM_DELAY_H */

