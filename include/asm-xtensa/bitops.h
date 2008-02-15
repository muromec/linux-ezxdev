#ifndef __ASM_XTENSA_BITOPS_H
#define __ASM_XTENSA_BITOPS_H

/*
 * include/asm-xtensa/bitops.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Chris Songer
 *			Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 */

#ifdef __KERNEL__

#include <asm/system.h>
#include <asm/byteorder.h>
#include <linux/bitops.h>
#include <xtensa/config/core.h>


extern __inline__ void set_bit(int nr, volatile void * addr)
{
	int	mask;
	volatile unsigned int *a = addr;
	unsigned long flags;

	a += nr >> 5;
	mask = 1 << (nr & 0x1f);
	save_and_cli(flags);
	*a |= mask;
	restore_flags(flags);
}

/*
 * non-atomic version
 */
static __inline__ void __set_bit(int nr, volatile void *addr)
{
	unsigned long mask = 1 << (nr % (sizeof(unsigned long) * 8));
	unsigned long *p =
	    ((unsigned long *)addr) + (nr / (sizeof(unsigned long) * 8));

	*p |= mask;
}

extern __inline__ void clear_bit(int nr, volatile void * addr)
{
	int	mask;
	volatile unsigned int *a = addr;
	unsigned long flags;

	a += nr >> 5;
	mask = 1 << (nr & 0x1f);
	save_and_cli(flags);
	*a &= ~mask;
	restore_flags(flags);
}

/*
 * non-atomic version
 */
static __inline__ void __clear_bit(int nr, volatile void *addr)
{
	unsigned long mask = 1 << (nr % (sizeof(unsigned long) * 8));
	unsigned long *p =
		((unsigned long *)addr) + (nr / (sizeof(unsigned long) * 8));

	*p &= ~mask;
}

/*
 * clear_bit() doesn't provide any barrier for the compiler.
 */
#define smp_mb__before_clear_bit()	barrier()
#define smp_mb__after_clear_bit()	barrier()

extern __inline__ void change_bit(int nr, volatile void * addr)
{
	int	mask;
	volatile unsigned int *a = addr;
	unsigned long flags;

	a += nr >> 5;
	mask = 1 << (nr & 0x1f);
	save_and_cli(flags);
	*a ^= mask;
	restore_flags(flags);
}

/*
 * non-atomic version
 */
static __inline__ void __change_bit(int nr, volatile void *addr)
{
	unsigned long mask = 1 << (nr & 0x1f);
	unsigned long *p = ((unsigned long *)addr) + (nr >> 5);

	*p ^= mask;
}

extern __inline__ int test_and_set_bit(int nr, volatile void * addr)
{
	int	mask, retval;
	volatile unsigned int *a = addr;
	unsigned long flags;

	a += nr >> 5;
	mask = 1 << (nr & 0x1f);
	save_and_cli(flags);
	retval = (mask & *a) != 0;
	*a |= mask;
	restore_flags(flags);

	return retval;
}

extern __inline__ int test_and_clear_bit(int nr, volatile void * addr)
{
	int	mask, retval;
	volatile unsigned int *a = addr;
	unsigned long flags;

	a += nr >> 5;
	mask = 1 << (nr & 0x1f);
	save_and_cli(flags);
	retval = (mask & *a) != 0;
	*a &= ~mask;
	restore_flags(flags);

	return retval;
}

extern __inline__ int test_and_change_bit(int nr, volatile void * addr)
{
	int	mask, retval;
	volatile unsigned int *a = addr;
	unsigned long flags;

	a += nr >> 5;
	mask = 1 << (nr & 0x1f);
	save_and_cli(flags);
	retval = (mask & *a) != 0;
	*a ^= mask;
	restore_flags(flags);

	return retval;
}

/*
 * non-atomic version; can be reordered
 */
static __inline__ int __test_and_change_bit(int nr, volatile void *addr)
{
	unsigned long mask = 1 << (nr & 0x1f);
	unsigned long *p = ((unsigned long *)addr) + (nr >> 5);
	unsigned long old = *p;

	*p = old ^ mask;
	return (old & mask) != 0;
}


extern __inline__ int test_bit(int nr, const volatile void *addr)
{
	return 1UL & (((const volatile unsigned int *) addr)[nr >> 5] >> (nr & 31));
}

/*
 * ffs: find first bit set. This is defined the same way as
 * the libc and compiler builtin ffs routines, therefore
 * differs in spirit from the below ffz (man ffs).
 */

#define ffs(x) generic_ffs(x)
#if BITS_PER_LONG == 32
#define __ffs(x) (ffs((int)(x)) - 1)
#endif

/*
 * ffz = Find First Zero in word. Undefined if no zero exists,
 * so code should check against ~0UL first..
 *
 * bit 0 is the LSB of addr; bit 32 is the LSB of (addr+1).
 *
 * XTFIXME:  The NSAU insn might work here to provide an optimized
 * implementation.  Also, see coreasm.h for macros using NSAU.
 */
extern __inline__ unsigned long ffz(unsigned long word)
{
	int k;

	word = ~word;
	k = 31;
	if (word & 0x0000ffff) { k -= 16; word <<= 16; }
	if (word & 0x00ff0000) { k -= 8;  word <<= 8;  }
	if (word & 0x0f000000) { k -= 4;  word <<= 4;  }
	if (word & 0x30000000) { k -= 2;  word <<= 2;  }
	if (word & 0x40000000) { k -= 1; }
        return k;
}

extern __inline__ int find_next_zero_bit(void *addr, int size, int offset)
{
	unsigned long *p = ((unsigned long *) addr) + (offset >> 5);
	unsigned long result = offset & ~31UL;
	unsigned long tmp;

	if (offset >= size)
		return size;
	size -= result;
	offset &= 31UL;
	if (offset) {
		tmp = *(p++);
		tmp |= ~0UL >> (32-offset);
		if (size < 32)
			goto found_first;
		if (~tmp)
			goto found_middle;
		size -= 32;
		result += 32;
	}
	while (size & ~31UL) {
		if (~(tmp = *(p++)))
			goto found_middle;
		result += 32;
		size -= 32;
	}
	if (!size)
		return result;
	tmp = *p;

found_first:
	tmp |= ~0UL << size;
found_middle:
	return result + ffz(tmp);
}

#define find_first_zero_bit(addr, size) \
        find_next_zero_bit((addr), (size), 0)

/*
 * Find the first bit set in a 140-bit bitmap.
 * The first 100 bits are unlikely to be set.
 */
static inline int _sched_find_first_bit(unsigned long *b)
{
	if (unlikely(b[0]))
		return __ffs(b[0]);
	if (unlikely(b[1]))
		return __ffs(b[1]) + 32;
	if (unlikely(b[2]))
		return __ffs(b[2]) + 64;
	if (b[3])
		return __ffs(b[3]) + 96;
	return __ffs(b[4]) + 128;
}

#if XCHAL_HAVE_LE
#define ext2_set_bit(nr, addr) test_and_set_bit((nr), (addr))
#define ext2_clear_bit(nr, addr) test_and_clear_bit((nr), (addr))
#define ext2_test_bit(nr, addr) test_bit((nr), (addr))
#define ext2_find_first_zero_bit(addr, size) find_first_zero_bit((addr), (size))
#define ext2_find_next_zero_bit(addr, size, offset) \
                find_next_zero_bit((addr), (size), (offset))

#elif XCHAL_HAVE_BE

extern __inline__ int ext2_set_bit(int nr, volatile void * addr)
{
	int		mask, retval;
	unsigned long	flags;
	volatile unsigned char	*ADDR = (unsigned char *) addr;

	ADDR += nr >> 3;
	mask = 1 << (nr & 0x07);
	save_and_cli(flags);
	retval = (mask & *ADDR) != 0;
	*ADDR |= mask;
	restore_flags(flags);
	return retval;
}

extern __inline__ int ext2_clear_bit(int nr, volatile void * addr)
{
	int		mask, retval;
	unsigned long	flags;
	volatile unsigned char	*ADDR = (unsigned char *) addr;

	ADDR += nr >> 3;
	mask = 1 << (nr & 0x07);
	save_and_cli(flags);
	retval = (mask & *ADDR) != 0;
	*ADDR &= ~mask;
	restore_flags(flags);
	return retval;
}

extern __inline__ int ext2_test_bit(int nr, const volatile void * addr)
{
	int			mask;
	const volatile unsigned char	*ADDR = (const unsigned char *) addr;

	ADDR += nr >> 3;
	mask = 1 << (nr & 0x07);
	return ((mask & *ADDR) != 0);
}

#define ext2_find_first_zero_bit(addr, size) \
        ext2_find_next_zero_bit((addr), (size), 0)

extern __inline__ unsigned long ext2_find_next_zero_bit(void *addr, unsigned long size, unsigned long offset)
{
	unsigned long *p = ((unsigned long *) addr) + (offset >> 5);
	unsigned long result = offset & ~31UL;
	unsigned long tmp;

	if (offset >= size)
		return size;
	size -= result;
	offset &= 31UL;
	if(offset) {
		/* We hold the little endian value in tmp, but then the
		 * shift is illegal. So we could keep a big endian value
		 * in tmp, like this:
		 *
		 * tmp = __swab32(*(p++));
		 * tmp |= ~0UL >> (32-offset);
		 *
		 * but this would decrease preformance, so we change the
		 * shift:
		 */
		tmp = *(p++);
		tmp |= __swab32(~0UL >> (32-offset));
		if(size < 32)
			goto found_first;
		if(~tmp)
			goto found_middle;
		size -= 32;
		result += 32;
	}
	while(size & ~31UL) {
		if(~(tmp = *(p++)))
			goto found_middle;
		result += 32;
		size -= 32;
	}
	if(!size)
		return result;
	tmp = *p;

found_first:
	/* tmp is little endian, so we would have to swab the shift,
	 * see above. But then we have to swab tmp below for ffz, so
	 * we might as well do this here.
	 */
	return result + ffz(__swab32(tmp) | (~0UL << size));
found_middle:
	return result + ffz(__swab32(tmp));
}

#else
#error endianess not defined
#endif /* XCHAL_HAVE_LE */

/* Bitmap functions for the minix filesystem.  */
#define minix_test_and_set_bit(nr,addr) test_and_set_bit(nr,addr)
#define minix_set_bit(nr,addr) set_bit(nr,addr)
#define minix_test_and_clear_bit(nr,addr) test_and_clear_bit(nr,addr)
#define minix_test_bit(nr,addr) test_bit(nr,addr)
#define minix_find_first_zero_bit(addr,size) find_first_zero_bit(addr,size)

#endif /* __KERNEL__ */

#endif /* __ASM_XTENSA_BITOPS_H */
