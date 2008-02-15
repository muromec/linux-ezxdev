/*
 * <asm/smplock.h>
 *
 * i386 SMP lock implementation
 */
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <asm/current.h>

extern spinlock_cacheline_t kernel_flag_cacheline;  
#define kernel_flag kernel_flag_cacheline.lock      
#ifdef CONFIG_PREEMPT_TIMES
#define lock_bkl() do { \
	current->preempt_count++;  \
	preempt_lock_start(3); \
	_raw_spin_lock(&kernel_flag); \
} while (0)
#else
#define lock_bkl() spin_lock(&kernel_flag)
#endif
#define unlock_bkl() spin_unlock(&kernel_flag)

#ifdef CONFIG_SMP
#define kernel_locked()		spin_is_locked(&kernel_flag)
#else
#ifdef CONFIG_PREEMPT
#define kernel_locked()		preempt_get_count()
#else
#define kernel_locked()		1
#endif
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


#ifdef CONFIG_PREEMPT_TIMES
#define lock_kernel() do { \
	if (current->lock_depth == -1) \
		lock_bkl(); \
	++current->lock_depth; \
} while (0)
 
#define unlock_kernel() do { \
	if (--current->lock_depth < 0) \
		unlock_bkl(); \
} while (0)
#else
/*
 * Getting the big kernel lock.
 *
 * This cannot happen asynchronously,
 * so we only need to worry about other
 * CPU's.
 */
static __inline__ void lock_kernel(void)
{
#ifdef CONFIG_PREEMPT
	if (current->lock_depth == -1)
		spin_lock(&kernel_flag);
	++current->lock_depth;
#else
#if 1
	if (!++current->lock_depth)
		spin_lock(&kernel_flag);
#else
	__asm__ __volatile__(
		"incl %1\n\t"
		"jne 9f"
		spin_lock_string
		"\n9:"
		:"=m" (__dummy_lock(&kernel_flag)),
		 "=m" (current->lock_depth));
#endif
#endif
}

static __inline__ void unlock_kernel(void)
{
	if (current->lock_depth < 0)
		out_of_line_bug();
#if 1
	if (--current->lock_depth < 0)
		spin_unlock(&kernel_flag);
#else
	__asm__ __volatile__(
		"decl %1\n\t"
		"jns 9f\n\t"
		spin_unlock_string
		"\n9:"
		:"=m" (__dummy_lock(&kernel_flag)),
		 "=m" (current->lock_depth));
#endif
}
#endif /* CONFIG_PREEMPT_TIMES */
