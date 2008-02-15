#ifndef _ASM_SYSTEM_H
#define _ASM_SYSTEM_H

/*
 * include/asm-xtensa/system.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Chris Songer
 *			Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 *			Kevin Chea
 */

#include <asm/xtutil.h>
#include <linux/config.h>
#include <asm/ptrace.h>
#include <linux/kernel.h>

/* 
 * sti -- turns on interrupts. 
 *
 * Xtensa: set PS.INTLEVEL to 0.
 */
extern __inline__ void __sti(void)
{
  __asm__ __volatile__(" rsil  a8, 0\n" : : : "a8");
}


/*
 * cli -- turn off interrupts.
 *
 * Xtensa: set PS.INTLEVEL to LOCKLEVEL
 */
extern __inline__ void __cli(void)
{
  __asm__ __volatile__(" rsil  a8, "XTSTR(LOCKLEVEL)"\n" : : : "a8");
}


#define save_and_cli(x) __save_and_cli( (unsigned long *)&(x) )
extern __inline__ void __save_and_cli( unsigned long *ptr )
{
  __asm__ __volatile__(" rsil  a8, "XTSTR(LOCKLEVEL)"\n"
		       " s32i  a8, %0, 0\n"
		       : /* no outputs */
		       : "a"  (ptr) 
		       : "a8", "memory"
		       );
}

#define restore_flags( m ) __restore_flags( m )
extern __inline__ void __restore_flags( unsigned long mask )
{
    __asm__ __volatile__(" wsr  %0, "XTSTR(PS)"\n"
			 " rsync\n"
			 : : "a" (mask) );
}

#define save_flage( m ) _save_flags( &m )
#if 0  /* XTFIXME:  Why? */
extern __inline__ unsigned long __save_flags( int *a )
#endif
#define __save_flags(x)					\
{							\
  unsigned long tmp;					\
  __asm__ __volatile__(" rsr  %0, "XTSTR(PS)"\n"	\
		       : "=a" (tmp) );			\
  x = tmp;						\
}
#define save_flags(x) __save_flags(x)

#ifdef CONFIG_PREEMPT

/*
 * Assume INTLEVEL != 0 means interrupts are disabled.  If LOCKLEVEL is
 * other than 1 then may need to modify this code to check for 
 * >= LOCKLEVEL.
 */

extern __inline__ int irqs_disabled(void)
{
	unsigned long tmp;
	__asm__ __volatile__(" rsr  %0, "XTSTR(PS)"\n"
			     : "=a" (tmp) );
	return tmp & (PS_INTLEVEL_MASK << PS_INTLEVEL_SHIFT);
}
#endif /* CONFIG_PREEMPT */

#define clear_cpenable() __clear_cpenable()
extern __inline__ void __clear_cpenable(void)
{
#if XCHAL_HAVE_CP
	unsigned long i = 0;
	__asm__ __volatile__ (" wsr  %0, " XTSTR(CPENABLE) "\n"
			      " rsync\n"
			      :  : "a" (i) );
#endif
}

#define sti() __sti()
#define cli() __cli()

/* For spinlocks etc */
#define local_irq_save(x)	__save_and_cli(&(x));
#define local_irq_restore(x)	__restore_flags(x);
#define local_irq_disable()	__cli();
#define local_irq_enable()	__sti();


#define mb()  barrier()
#define rmb() mb()
#define wmb() mb()

#ifdef CONFIG_SMP
#error smp_* not defined
#else
#define smp_mb()	barrier()
#define smp_rmb()	barrier()
#define smp_wmb()	barrier()
#endif

#define set_mb(var, value)  do { var = value; mb(); } while (0)
#define set_wmb(var, value) do { var = value; wmb(); } while (0)

#if !defined (_LANGUAGE_ASSEMBLY)
/*
 * switch_to(n) should switch tasks to task nr n, first
 * checking that n isn't the current task, in which case it does nothing.
 */
extern asmlinkage void *resume(void *last, void *next);
#endif /* !defined (_LANGUAGE_ASSEMBLY) */

#define prepare_to_switch()	do { } while(0)
#define switch_to(prev,next,last)	\
do {					\
	clear_cpenable();		\
	invalidate_page_table();	\
	(last) = resume(prev, next);	\
} while(0)

/*
 * xchg_u32
 *
 * Note that a15 is used here because the register allocation
 * done by the compiler is not guaranteed and a window overflow
 * may not occur between the rsil and wsr instructions. By using
 * a15 in the rsil, the machine is guaranteed to be in a state
 * where no register reference will cause an overflow.
 */

extern __inline__ unsigned long xchg_u32(volatile int * m, unsigned long val)
{
  unsigned long tmp;
  __asm__ __volatile__(
		       "rsil    a15, "XTSTR(LOCKLEVEL)"\n\t"
		       "l32i    %0, %1, 0              \n\t"
		       "s32i    %2, %1, 0              \n\t"
		       "wsr     a15, "XTSTR(PS)"       \n\t" /* wsr save_level, PS */
		       "rsync                          \n\t"
		       : "=&a" (tmp)
		       : "a" (m), "a" (val)
		       : "a15", "memory"
		       );
  return tmp;
}

#define tas(ptr) (xchg((ptr),1))

#if ( __XCC__ == 1 )

/* xt-xcc processes __inline__ differently than xt-gcc and decides to
 * insert an out-of-line copy of function __xchg.  This presents the
 * unresolved symbol at link time of __xchg_called_with_bad_pointer,
 * even though such a function would never be called at run-time.
 * xt-gcc always inlines __xchg, and optimizes away the undefined
 * bad_pointer function.
 */

#define xchg(ptr,x) xchg_u32(ptr,x)

#else  /* assume xt-gcc */

#define xchg(ptr,x) ((__typeof__(*(ptr)))__xchg((unsigned long)(x),(ptr),sizeof(*(ptr))))

/*
 * This only works if the compiler isn't horribly bad at optimizing.
 * gcc-2.5.8 reportedly can't handle this, but I define that one to
 * be dead anyway.
 */

extern void __xchg_called_with_bad_pointer(void);

static __inline__ unsigned long __xchg(unsigned long x, volatile void * ptr, int size)
{
	switch (size) {
		case 4:
			return xchg_u32(ptr, x);
	}
	__xchg_called_with_bad_pointer();
	return x;
}
#endif


extern void set_except_vector(int n, void *addr);

extern void __die(const char *, struct pt_regs *, const char *where,
	unsigned long line) __attribute__((noreturn));
extern void __die_if_kernel(const char *, struct pt_regs *, const char *where,
	unsigned long line);
extern int abs(int);

#define die(msg, regs)							\
	__die(msg, regs, __FILE__ ":"__FUNCTION__, __LINE__)
#define die_if_kernel(msg, regs)					\
	__die_if_kernel(msg, regs, __FILE__ ":"__FUNCTION__, __LINE__)

#endif /* _ASM_SYSTEM_H */
