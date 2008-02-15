#ifndef __ASM_XTENSA_ATOMIC_H
#define __ASM_XTENSA_ATOMIC_H

/*
 * include/asm-xtensa/atomic.h
 *
 * Atomic operations that C can't guarantee us.  Useful for
 * resource counting etc.
 *
 * This Xtensa implementation assumes that the right mechanism
 * for exclusion is for locking interrupts to level 1.
 *
 * Locking interrupts looks like this:
 *
 *    rsil a15, 1
 *    <code>
 *    wsr  a15, PS
 *    rsync
 *
 * Note that a15 is used here because the register allocation
 * done by the compiler is not guaranteed and a window overflow
 * may not occur between the rsil and wsr instructions. By using
 * a15 in the rsil, the machine is guaranteed to be in a state
 * where no register reference will cause an overflow.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Chris Songer
 *			Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 */

#include <linux/config.h>

typedef struct { volatile int counter; } atomic_t;

#ifdef __KERNEL__
#include <asm/xtutil.h>
#include <asm/system.h>

#define ATOMIC_INIT(i)	( (atomic_t) { (i) } )

#define atomic_read(v)		((v)->counter)
#define atomic_set(v,i)		((v)->counter = (i))


/*
 * To get proper branch prediction for the main line, we must branch
 * forward to code at the end of this object's .text section, then
 * branch back to restart the operation.
 */

extern __inline__ void atomic_add(int i, atomic_t * v)
{
    unsigned int vval;

    __asm__ __volatile__( 
	"rsil    a15, "XTSTR(LOCKLEVEL)"\n\t"
	"l32i    %0, %2, 0              \n\t"
	"add     %0, %0, %1             \n\t"
	"s32i    %0, %2, 0              \n\t"
	"wsr     a15, "XTSTR(PS)"       \n\t"
	"rsync                          \n"
	: "=&a" (vval) 
	: "a" (i), "a" (v)
	: "a15", "memory"
	);
}

extern __inline__ void atomic_sub(int i, atomic_t *v)
{
    unsigned int vval;

    __asm__ __volatile__( 
	"rsil    a15, "XTSTR(LOCKLEVEL)"\n\t"
	"l32i    %0, %2, 0              \n\t"
	"sub     %0, %0, %1             \n\t"
	"s32i    %0, %2, 0              \n\t"
	"wsr     a15, "XTSTR(PS)"       \n\t"
	"rsync                          \n"
	: "=&a" (vval) 
	: "a" (i), "a" (v)
	: "a15", "memory"
	);
}

extern __inline__ int atomic_add_return(int i, atomic_t * v)
{
     unsigned int vval;

    __asm__ __volatile__( 
	"rsil    a15,"XTSTR(LOCKLEVEL)"\n\t"
	"l32i    %0, %2, 0             \n\t"
	"add     %0, %0, %1            \n\t"
	"s32i    %0, %2, 0             \n\t"
	"wsr     a15, "XTSTR(PS)"      \n\t"
	"rsync                         \n"
	: "=&a" (vval) 
	: "a" (i), "a" (v)
	: "a15", "memory"
	);

    return vval;
}

extern __inline__ int atomic_sub_return(int i, atomic_t * v)
{
    unsigned int vval;

    __asm__ __volatile__( 
	"rsil    a15,"XTSTR(LOCKLEVEL)"\n\t"
	"l32i    %0, %2, 0             \n\t"
	"sub     %0, %0, %1            \n\t"
	"s32i    %0, %2, 0             \n\t"
	"wsr     a15, "XTSTR(PS)"       \n\t"
	"rsync                         \n"
	: "=&a" (vval) 
	: "a" (i), "a" (v)
	: "a15", "memory"
	);

    return vval;
}

#define atomic_dec_return(v) atomic_sub_return(1,(v))
#define atomic_inc_return(v) atomic_add_return(1,(v))

#define atomic_sub_and_test(i,v) (atomic_sub_return((i), (v)) == 0) 
#define atomic_dec_and_test(v) (atomic_sub_return(1, (v)) == 0)
 
#define atomic_inc(v) atomic_add(1,(v)) 
#define atomic_dec(v) atomic_sub(1,(v))

extern __inline__ void atomic_clear_mask(unsigned int mask, atomic_t *v)
{
    unsigned int all_f = -1;
    unsigned int vval;

    __asm__ __volatile__( 
	"rsil    a15,"XTSTR(LOCKLEVEL)"\n\t"
	"l32i    %0, %2, 0             \n\t"
	"xor     %1, %4, %3            \n\t"
	"and     %0, %0, %4            \n\t"
	"s32i    %0, %2, 0             \n\t"
	"wsr     a15, "XTSTR(PS)"      \n\t"
	"rsync                         \n"
	: "=&a" (vval), "=a" (mask) 
	: "a" (v), "a" (all_f), "1" (mask)
	: "a15", "memory"
	);
}

extern __inline__ void atomic_set_mask(unsigned int mask, atomic_t *v)
{
    unsigned int vval;

    __asm__ __volatile__( 
	"rsil    a15,"XTSTR(LOCKLEVEL)"\n\t"
	"l32i    %0, %2, 0             \n\t"
	"or      %0, %0, %1            \n\t"
	"s32i    %0, %2, 0             \n\t"
	"wsr     a15, "XTSTR(PS)"       \n\t"
	"rsync                         \n"
	: "=&a" (vval) 
	: "a" (mask), "a" (v)
	: "a15", "memory"
	);
}

/* Atomic operations are already serializing */
#define smp_mb__before_atomic_dec()	barrier()
#define smp_mb__after_atomic_dec()	barrier()
#define smp_mb__before_atomic_inc()	barrier()
#define smp_mb__after_atomic_inc()	barrier()

#endif /* __KERNEL__ */

#endif /* __ASM_XTENSA_ATOMIC_H */

