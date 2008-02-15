#ifndef __ASM_SH_SEMAPHORE_HELPER_H
#define __ASM_SH_SEMAPHORE_HELPER_H

/*
 * include/asm-xtensa/semaphore-helper.h
 *
 * SMP- and interrupt-safe semaphores helper functions.  Derived from SH.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1996 Linus Torvalds
 * Copyright (C) 1999 Andrea Arcangeli
 * Copyright (C) 2001 Tensilica Inc.
 */


/*
 * These two _must_ execute atomically wrt each other.
 */
static __inline__ void wake_one_more(struct semaphore * sem)
{
	atomic_inc((atomic_t *)&sem->sleepers);
}

static __inline__ int waking_non_zero(struct semaphore *sem)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&semaphore_wake_lock, flags);
	if (sem->sleepers > 0) {
		sem->sleepers--;
		ret = 1;
	}
	spin_unlock_irqrestore(&semaphore_wake_lock, flags);
	return ret;
}

/*
 * waking_non_zero_interruptible:
 *	1	got the lock
 *	0	go to sleep
 *	-EINTR	interrupted
 *
 * We must undo the sem->count down_interruptible() increment while we are
 * protected by the spinlock in order to make atomic this atomic_inc() with the
 * atomic_read() in wake_one_more(), otherwise we can race. -arca
 */
static __inline__ int waking_non_zero_interruptible(struct semaphore *sem,
						struct task_struct *tsk)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&semaphore_wake_lock, flags);
	if (sem->sleepers > 0) {
		sem->sleepers--;
		ret = 1;
	} else if (signal_pending(tsk)) {
		atomic_inc(&sem->count);
		ret = -EINTR;
	}
	spin_unlock_irqrestore(&semaphore_wake_lock, flags);
	return ret;
}

/*
 * waking_non_zero_trylock:
 *	1	failed to lock
 *	0	got the lock
 *
 * We must undo the sem->count down_trylock() increment while we are
 * protected by the spinlock in order to make atomic this atomic_inc() with the
 * atomic_read() in wake_one_more(), otherwise we can race. -arca
 */
static __inline__ int waking_non_zero_trylock(struct semaphore *sem)
{
	unsigned long flags;
	int ret = 1;

	spin_lock_irqsave(&semaphore_wake_lock, flags);
	if (sem->sleepers <= 0)
		atomic_inc(&sem->count);
	else {
		sem->sleepers--;
		ret = 0;
	}
	spin_unlock_irqrestore(&semaphore_wake_lock, flags);
	return ret;
}

#endif /* __ASM_SH_SEMAPHORE_HELPER_H */
