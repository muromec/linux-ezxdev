#ifndef __ASM_XTENSA_SMPLOCK_H
#define __ASM_XTENSA_SMPLOCK_H

/*
 * include/asm-xtensa/smplock.h
 *
 * Default SMP lock implementation.  Swiped from MIPS.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#include <linux/interrupt.h>
#include <linux/spinlock.h>

extern spinlock_t kernel_flag;

#ifdef CONFIG_PREEMPT
#define kernel_locked()		preempt_is_disabled()
#else
#define kernel_locked()		spin_is_locked(&kernel_flag)
#endif

/*
 * Release global kernel lock and global interrupt lock
 */
#define release_kernel_lock(task, cpu) \
do { \
	if (task->lock_depth >= 0) \
		spin_unlock(&kernel_flag); \
	release_irqlock(cpu); \
	__sti(); \
} while (0)

/*
 * Re-acquire the kernel lock
 */
#define reacquire_kernel_lock(task) \
do { \
	if (task->lock_depth >= 0) \
		spin_lock(&kernel_flag); \
} while (0)


/*
 * Getting the big kernel lock.
 *
 * This cannot happen asynchronously,
 * so we only need to worry about other
 * CPU's.
 */
extern __inline__ void lock_kernel(void)
{
#ifdef CONFIG_PREEMPT
	if (current->lock_depth == -1)
		spin_lock(&kernel_flag);
	++current->lock_depth;
#else
	if (!++current->lock_depth)
		spin_lock(&kernel_flag);
#endif
}

extern __inline__ void unlock_kernel(void)
{
	if (--current->lock_depth < 0)
		spin_unlock(&kernel_flag);
}

#endif /* __ASM_XTENSA_SMPLOCK_H */
