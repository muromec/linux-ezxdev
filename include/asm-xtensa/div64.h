#ifndef __ASM_XTENSA_DIV64
#define __ASM_XTENSA_DIV64

/* 
 * include/asm-xtensa/div64.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#define do_div(n,base) ({ \
	int __res; \
	__res = ((unsigned long) n) % (unsigned) base; \
	n = ((unsigned long) n) / (unsigned) base; \
	__res; })

#endif /* __ASM_XTENSA_DIV64 */
