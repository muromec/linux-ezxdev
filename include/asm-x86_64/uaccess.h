#ifndef __X86_64_UACCESS_H
#define __X86_64_UACCESS_H

/*
 * User space memory access functions
 */
#include <linux/config.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/prefetch.h>
#include <asm/page.h>

#define VERIFY_READ 0
#define VERIFY_WRITE 1

/*
 * The fs value determines whether argument validity checking should be
 * performed or not.  If get_fs() == USER_DS, checking is performed, with
 * get_fs() == KERNEL_DS, checking is bypassed.
 *
 * For historical reasons, these macros are grossly misnamed.
 */

#define MAKE_MM_SEG(s)	((mm_segment_t) { (s) })

#define KERNEL_DS	MAKE_MM_SEG(0xFFFFFFFFFFFFFFFF)
#define USER_DS		MAKE_MM_SEG(PAGE_OFFSET)

#define get_ds()	(KERNEL_DS)
#define get_fs()	(current->addr_limit)
#define set_fs(x)	(current->addr_limit = (x))

#define segment_eq(a,b)	((a).seg == (b).seg)

#define __addr_ok(addr) (!((unsigned long)(addr) & (current->addr_limit.seg)))

/*
 * Uhhuh, this needs 65-bit arithmetic. We have a carry..
 */
#define __range_not_ok(addr,size) ({ \
	unsigned long flag,sum; \
	asm("# range_ok\n\r" \
		"addq %3,%1 ; sbbq %0,%0 ; cmpq %1,%4 ; sbbq $0,%0"  \
		:"=&r" (flag), "=r" (sum) \
		:"1" (addr),"g" ((long)(size)),"g" (current->addr_limit.seg)); \
	flag; })

#define access_ok(type,addr,size) (__range_not_ok(addr,size) == 0)

extern inline int verify_area(int type, const void * addr, unsigned long size)
{
	return access_ok(type,addr,size) ? 0 : -EFAULT;
}


/*
 * The exception table consists of pairs of addresses: the first is the
 * address of an instruction that is allowed to fault, and the second is
 * the address at which the program should continue.  No registers are
 * modified, so it is entirely up to the continuation code to figure out
 * what to do.
 *
 * All the routines below use bits of fixup code that are out of line
 * with the main instruction path.  This means when everything is well,
 * we don't even have to jump over them.  Further, they do not intrude
 * on our cache or tlb entries.
 */

struct exception_table_entry
{
	unsigned long insn, fixup;
};

/* Returns 0 if exception not found and fixup otherwise.  */
extern unsigned long search_exception_table(unsigned long);


/*
 * These are the main single-value transfer routines.  They automatically
 * use the right size if we just have the right pointer type.
 *
 * This gets kind of ugly. We want to return _two_ values in "get_user()"
 * and yet we don't want to do any pointers, because that is too much
 * of a performance impact. Thus we have a few rather ugly macros here,
 * and hide all the uglyness from the user.
 *
 * The "__xxx" versions of the user access functions are versions that
 * do not verify the address space, that must have been done previously
 * with a separate "access_ok()" call (this is used when we do multiple
 * accesses to the same area of user memory).
 */

extern void __get_user_1(void);
extern void __get_user_2(void);
extern void __get_user_4(void);
extern void __get_user_8(void);

#define __get_user_x(size,ret,x,ptr) \
	__asm__ __volatile__("call __get_user_" #size \
		:"=a" (ret),"=d" (x) \
		:"0" (ptr) \
		:"rbx")

/* Careful: we have to cast the result to the type of the pointer for sign reasons */
#define get_user(x,ptr)							\
({	long __ret_gu,__val_gu;						\
	switch(sizeof (*(ptr))) {					\
	case 1:  __get_user_x(1,__ret_gu,__val_gu,ptr); break;		\
	case 2:  __get_user_x(2,__ret_gu,__val_gu,ptr); break;		\
	case 4:  __get_user_x(4,__ret_gu,__val_gu,ptr); break;		\
	case 8:  __get_user_x(8,__ret_gu,__val_gu,ptr); break;		\
	default: __get_user_bad(); break;				\
	}								\
	(x) = (__typeof__(*(ptr)))__val_gu;				\
	__ret_gu;							\
})

extern void __put_user_1(void);
extern void __put_user_2(void);
extern void __put_user_4(void);
extern void __put_user_8(void);

extern void __put_user_bad(void);

#define __put_user_x(size,ret,x,ptr)					\
	__asm__ __volatile__("call __put_user_" #size			\
		:"=a" (ret)						\
		:"0" (ptr),"d" (x)					\
		:"rbx")

#define put_user(x,ptr)							\
  __put_user_check((__typeof__(*(ptr)))(x),(ptr),sizeof(*(ptr)))

#define __get_user(x,ptr) \
  __get_user_nocheck((x),(ptr),sizeof(*(ptr)))
#define __put_user(x,ptr) \
  __put_user_nocheck((__typeof__(*(ptr)))(x),(ptr),sizeof(*(ptr)))

#define __put_user_nocheck(x,ptr,size)			\
({							\
	long __pu_err;					\
	__put_user_size((x),(ptr),(size),__pu_err);	\
	__pu_err;					\
})


#define __put_user_check(x,ptr,size)			\
({							\
	long __pu_err = -EFAULT;			\
	__typeof__(*(ptr)) *__pu_addr = (ptr);		\
	if (access_ok(VERIFY_WRITE,__pu_addr,size))	\
		__put_user_size((x),__pu_addr,(size),__pu_err);	\
	__pu_err;					\
})

#define __put_user_size(x,ptr,size,retval)				\
do {									\
	retval = 0;							\
	switch (size) {							\
	  case 1: __put_user_asm(x,ptr,retval,"b","b","iq"); break;	\
	  case 2: __put_user_asm(x,ptr,retval,"w","w","ir"); break;	\
	  case 4: __put_user_asm(x,ptr,retval,"l","k","ir"); break;	\
	  case 8: __put_user_asm(x,ptr,retval,"q","","ir"); break;	\
	  default: __put_user_bad();					\
	}								\
} while (0)

/* FIXME: this hack is definitely wrong -AK */
struct __large_struct { unsigned long buf[100]; };
#define __m(x) (*(struct __large_struct *)(x))

/*
 * Tell gcc we read from memory instead of writing: this is because
 * we do not write to any memory gcc knows about, so there are no
 * aliasing issues.
 */
#define __put_user_asm(x, addr, err, itype, rtype, ltype)	\
	__asm__ __volatile__(				\
		"1:	mov"itype" %"rtype"1,%2\n"	\
		"2:\n"							\
		".section .fixup,\"ax\"\n"		\
		"3:	movq %3,%0\n"				\
		"	jmp 2b\n"					\
		".previous\n"					\
		".section __ex_table,\"a\"\n"	\
		"	.align 8\n"					\
		"	.quad 1b,3b\n"				\
		".previous"						\
		: "=r"(err)						\
		: ltype (x), "m"(__m(addr)), "i"(-EFAULT), "0"(err))


#define __get_user_nocheck(x,ptr,size)				\
({								\
	long __gu_err, __gu_val;				\
	__get_user_size(__gu_val,(ptr),(size),__gu_err);	\
	(x) = (__typeof__(*(ptr)))__gu_val;			\
	__gu_err;						\
})

extern long __get_user_bad(void);

#define __get_user_size(x,ptr,size,retval)				\
do {									\
	retval = 0;							\
	switch (size) {							\
	  case 1: __get_user_asm(x,ptr,retval,"b","b","=q"); break;	\
	  case 2: __get_user_asm(x,ptr,retval,"w","w","=r"); break;	\
	  case 4: __get_user_asm(x,ptr,retval,"l","k","=r"); break;	\
	  case 8: __get_user_asm(x,ptr,retval,"q","","=r"); break;	\
	  default: (x) = __get_user_bad();				\
	}								\
} while (0)

#define __get_user_asm(x, addr, err, itype, rtype, ltype)	\
	__asm__ __volatile__(					\
		"1:	mov"itype" %2,%"rtype"1\n"		\
		"2:\n"						\
		".section .fixup,\"ax\"\n"			\
		"3:	mov %3,%0\n"				\
		"	xor"itype" %"rtype"1,%"rtype"1\n"	\
		"	jmp 2b\n"				\
		".previous\n"					\
		".section __ex_table,\"a\"\n"			\
		"	.align 8\n"				\
		"	.quad 1b,3b\n"				\
		".previous"					\
		: "=r"(err), ltype (x)				\
		: "m"(__m(addr)), "i"(-EFAULT), "0"(err))

/*
 * Copy To/From Userspace
 * 
 * This relies on an optimized common worker function.
 * 
 * Could do special inline versions for small constant copies, but avoid this
 * for now. It's not clear it is worth it. 
 */

extern unsigned long copy_user_generic(void *to, const void *from, unsigned len); 

extern unsigned long copy_to_user(void *to, const void *from, unsigned len); 
extern unsigned long copy_from_user(void *to, const void *from, unsigned len); 
#define __copy_to_user copy_user_generic
#define __copy_from_user copy_user_generic

long strncpy_from_user(char *dst, const char *src, long count);
long __strncpy_from_user(char *dst, const char *src, long count);
#define strlen_user(str) strnlen_user(str, ~0UL >> 1)
long strnlen_user(const char *str, long n);
unsigned long clear_user(void *mem, unsigned long len);
unsigned long __clear_user(void *mem, unsigned long len);

#endif /* __X86_64_UACCESS_H */
