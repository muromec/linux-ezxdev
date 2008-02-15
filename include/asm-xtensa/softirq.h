#ifndef __ASM_XTENSA_SOFTIRQ_H
#define __ASM_XTENSA_SOFTIRQ_H

/*
 * include/asm-xtensa/softirq.h
 *
 * Swiped from MIPS.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#include <asm/atomic.h>
#include <asm/hardirq.h>

#define cpu_bh_disable(cpu)	do { preempt_disable(); local_bh_count(cpu)++; barrier(); } while (0)
#define __cpu_bh_enable(cpu)	do { barrier(); local_bh_count(cpu)--; preempt_enable(); } while (0)

#define local_bh_disable()	cpu_bh_disable(smp_processor_id())
#define __local_bh_enable()	__cpu_bh_enable(smp_processor_id())
#define local_bh_enable()					\
do {								\
	int cpu;						\
								\
	barrier();						\
	cpu = smp_processor_id();				\
	if (!--local_bh_count(cpu) && softirq_pending(cpu))	\
		do_softirq();					\
	preempt_enable();					\
} while (0)

#define in_softirq() (local_bh_count(smp_processor_id()) != 0)

#endif /* __ASM_XTENSA_SOFTIRQ_H */
