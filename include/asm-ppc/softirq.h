/*
 * BK Id: SCCS/s.softirq.h 1.15 09/09/01 12:18:02 trini
 */
#ifdef __KERNEL__
#ifndef __ASM_SOFTIRQ_H
#define __ASM_SOFTIRQ_H

#include <asm/atomic.h>
#include <asm/hardirq.h>

#define local_bh_disable()			\
do {						\
	preempt_disable();			\
	local_bh_count(smp_processor_id())++;	\
	barrier();				\
} while (0)

#define __local_bh_enable()			\
do {						\
	barrier();				\
	local_bh_count(smp_processor_id())--;	\
	preempt_enable();			\
} while (0)

#define _local_bh_enable()				\
do {							\
	if (!--local_bh_count(smp_processor_id())	\
	    && softirq_pending(smp_processor_id())) {	\
		do_softirq();				\
	}						\
} while (0)

#define local_bh_enable()			\
do {						\
	_local_bh_enable();			\
	preempt_enable();			\
} while (0)

#define in_softirq() (preempt_is_disabled() && \
			(local_bh_count(smp_processor_id()) != 0))

#endif	/* __ASM_SOFTIRQ_H */
#endif /* __KERNEL__ */
