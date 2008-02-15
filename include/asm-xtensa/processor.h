#ifndef __ASM_XTENSA_PROCESSOR_H
#define __ASM_XTENSA_PROCESSOR_H

/*
 * include/asm-xtensa/processor.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 *			Marc Gauthier
 *			Kevin Chea
 */


/*
 * User space process size: 1 GB.  Trampolining, which glibc uses for
 * nested functions, requires all code to be in the same gigabyte of
 * memory.
 */
#define TASK_SIZE	0x40000000


#ifndef __ASSEMBLY__

#include <asm/page.h>
#include <asm/types.h>
#include <asm/ptrace.h>
#include <asm/kstack.h>
#include <xtensa/config/core.h>
#include <linux/threads.h>

/* Use WI_MASK to mask off the window-increment bits of a return
 * address. JUST_RETADDR yields just the return address.  If TASK_SIZE
 * grows, it will also need to compute and write the appropriate GB
 * value in the top two bits if the return address.
 *
 * JUST_KRETADDR is a version for kernel addresses.
 */
#define WI_MASK		0x3fffffff

#define JUST_RETADDR(x) ((x) & WI_MASK)
#define JUST_KRETADDR(x) (((x) & WI_MASK) | 0xc0000000)

#if (TASK_SIZE > 0x40000000)
#error Bad window-increment replacement in JUST_RETADDR above
#endif

#if (XCHAL_KSEG_CACHED_VADDR < 0xc0000000)
#error Bad window-increment replacement in JUST_KRETADDR above
#endif


typedef struct {
    unsigned long seg;
} mm_segment_t;

/*
 * Default implementation of macro that returns current
 * instruction pointer ("program counter").
 */
#define current_text_addr() ({ void *pc; __asm__("movi	%0, 1f\n1:":"=r" (pc)); pc; })

/*
 *  CPU type and hardware bug flags. Kept separately for each CPU.
 */
enum cpu_type {
	CPU_T1040BE,
	CPU_T1040LE,
	CPU_XTENSA_NONE
};

struct xtensa_cpuinfo {
/* 	enum cpu_type 	type;		Remove unused field. */
/* 	char		hard_math;	Remove unused field. */

	unsigned long 	udelay_val;
	unsigned long	*pgd_quick;
	unsigned long	*pte_quick;
	unsigned long	pgtable_cache_sz;
};

#if XCHAL_EXTRA_SA_ALIGN < 16
#define EXTRA_ALIGN   16
#else
#define EXTRA_ALIGN   XCHAL_EXTRA_SA_ALIGN
#endif

struct thread_struct {
	
	/* -16, because we always save the user's active window to the
	   kernel's exception stack frame. */
	unsigned long regfile[XCHAL_NUM_AREGS - 16];

	unsigned long areg0;	/* kernel's a0 (return PC + window size) for context switching */
	unsigned long areg1;	/* kernel's a1 (stack pointer) for context switching */
	
	mm_segment_t current_ds;    /* see uaccess.h for example uses */
	struct xtensa_cpuinfo info;

	unsigned long bad_vaddr; /* last user fault */
	unsigned long bad_uaddr; /* last kernel fault accessing user space */
	unsigned long error_code;

	/* Allocate storage for extra state and coprocessor state. */
	unsigned char extra[(XCHAL_EXTRA_SA_SIZE+15)&0xfffffff0]
		__attribute__ ((aligned(EXTRA_ALIGN)));
	unsigned cpregs_ptr[XCHAL_CP_MAX];

#if (XCHAL_CP_MASK & 1)
	unsigned cp0_regs[XCHAL_CP0_SA_SIZE] __attribute__ ((aligned(16)));
#endif
#if (XCHAL_CP_MASK & 2)
	unsigned cp1_regs[XCHAL_CP1_SA_SIZE] __attribute__ ((aligned(16)));
#endif
#if (XCHAL_CP_MASK & 4)
	unsigned cp2_regs[XCHAL_CP2_SA_SIZE] __attribute__ ((aligned(16)));
#endif
#if (XCHAL_CP_MASK & 8)
	unsigned cp3_regs[XCHAL_CP3_SA_SIZE] __attribute__ ((aligned(16)));
#endif
#if (XCHAL_CP_MASK & 16)
	unsigned cp4_regs[XCHAL_CP4_SA_SIZE] __attribute__ ((aligned(16)));
#endif
#if (XCHAL_CP_MASK & 32)
	unsigned cp5_regs[XCHAL_CP5_SA_SIZE] __attribute__ ((aligned(16)));
#endif
#if (XCHAL_CP_MASK & 64)
	unsigned cp6_regs[XCHAL_CP6_SA_SIZE] __attribute__ ((aligned(16)));
#endif
#if (XCHAL_CP_MASK & 128)
	unsigned cp7_regs[XCHAL_CP7_SA_SIZE] __attribute__ ((aligned(16)));
#endif
	
};

extern struct xtensa_cpuinfo boot_cpu_data;

#define cpu_data (&boot_cpu_data)
#define current_cpu_data boot_cpu_data


/* This decides where the kernel will search for a free chunk of vm
 * space during mmap's.
 */
#define TASK_UNMAPPED_BASE	(TASK_SIZE / 2)

#define INIT_THREAD  \
{								\
	regfile:	{0},					\
	areg0:		0,					\
	areg1:		sizeof(init_stack) + (long) &init_stack,\
	current_ds:	{0},					\
	info:		{0},					\
	bad_vaddr:	0,					\
	bad_uaddr:	0,					\
	error_code:	0,					\
}

/*
 * Do necessary setup to start up a newly executed thread.
 */
#define USER_PS_VALUE ( (1 << XCHAL_PS_WOE_SHIFT) + \
                        (1 << XCHAL_PS_CALLINC_SHIFT) + \
                        (USER_RING << XCHAL_PS_RING_SHIFT) + \
                        (1 << XCHAL_PS_PROGSTACK_SHIFT) + \
                        (1 << XCHAL_PS_EXCM_SHIFT) )

/* Clearing a0 terminates the backtrace. */
#define start_thread(regs, new_pc, new_sp) \
	regs->pc = new_pc; \
	regs->ps = USER_PS_VALUE; \
	regs->aregs[1] = new_sp; \
	regs->aregs[0] = 0; \
	regs->wb = 0; \
	regs->ws = 1

/* Forward declaration, a strange C thing */
struct task_struct;
struct mm_struct;

/* Free all resources held by a thread. */
#define release_thread(thread) do { } while(0)

/*
 * create a kernel thread without removing it from tasklists
 */
extern int kernel_thread(int (*fn)(void *), void * arg, unsigned long flags);

/*
 * Bus types
 */
#define EISA_bus 0
#define EISA_bus__is_a_macro /* for versions in ksyms.c */
#define MCA_bus 0
#define MCA_bus__is_a_macro /* for versions in ksyms.c */


/* Copy and release all segment info associated with a VM */
#define copy_segments(p, mm)	do { } while(0)
#define release_segments(mm)	do { } while(0)
#define forget_segments()	do { } while (0)

/* Return saved PC of a blocked thread.  We moved the function guts to
   an external routine to break cyclical dependencies on
   asm/{processor,offset}.h.
 */
extern unsigned long xtensa_thread_saved_pc(struct thread_struct *);
#define thread_saved_pc(t) xtensa_thread_saved_pc(t)

/*  The kernel task struct and kernel stack of a given task live in
 *  2 consecutive pages (aligned) that start with the task struct.
 *  User exceptions cause entry at bottom of kernel stack, where
 *  the user state is saved in a pt_regs struct.  The following
 *  macro (__KSTK_TOS) returns a ptr to that pt_regs for a given task:
 */
#define __KSTK_TOS(tsk) ((struct pt_regs*)((unsigned long)(tsk) + __KSTK_TOS_OFFSET))

extern unsigned long get_wchan(struct task_struct *p);

#define KSTK_EIP(tsk)  (__KSTK_TOS(tsk)->pc)
#define KSTK_ESP(tsk)  (__KSTK_TOS(tsk)->aregs[1])

#define THREAD_SIZE (2*PAGE_SIZE)
#define alloc_task_struct() \
	((struct task_struct *) __get_free_pages(GFP_KERNEL,1))
#define free_task_struct(p)	free_pages((unsigned long)(p),1)
#define get_task_struct(tsk)      atomic_inc(&virt_to_page(tsk)->count)

#define init_task	(init_task_union.task)
#define init_stack	(init_task_union.stack)

#define cpu_relax()  do { } while (0)

#endif /* __ASSEMBLY__ */

#endif /* __ASM_XTENSA_PROCESSOR_H */
