#ifndef __ASM_SH_DIV64
#define __ASM_SH_DIV64

/* This code only handles 32 bit dividend and quotient */
#define do_div(n,base) ({ \
int __res; \
__res = ((unsigned long) n) % (unsigned) base; \
n = ((unsigned long) n) / (unsigned) base; \
__res; })

/* Do 64 bit dividend divided by 32 bit divisor, giving 32 bit quotient
   At the moment, it always returns a remainder of 0.  If the true
   remainder is required, then code needs to be added to multiply the
   obtained quotient by the divisor, and subtracting this from the
   dividend. */

#define do_div64_32(n,base) ({ \
	unsigned long long __div; \
	unsigned long __base, __hi, __lo; \
	__div = (n); \
	__base = (base); \
	__hi = __div >> 32; \
	__lo = __div; \
	__asm__ ("div0u\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0\n\t" \
		 "div1  %2, %1\n\t" \
		 "rotcl %0" \
		 : "+r" (__lo), "+r" (__hi) \
		 : "r" (__base) \
		 : "t" \
	); \
	(n) = __lo; \
	0;})
#endif /* __ASM_SH_DIV64 */
