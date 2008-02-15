#ifndef __ASM_XTENSA_CURRENT_H
#define __ASM_XTENSA_CURRENT_H

/*
 * include/asm-xtensa/current.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 *			Kevin Chea
 */


#ifdef _ASMLANGUAGE

#define _CURRENT_SHIFT  13

.macro  _GET_CURRENT reg
	mov   \reg, a1
	srli  \reg, \reg, _CURRENT_SHIFT
	slli  \reg, \reg, _CURRENT_SHIFT
.endm

#else

struct task_struct;

static __inline__ struct task_struct * get_current(void)
{
	struct task_struct *current;
	__asm__ __volatile__ ("mov    %0, a1\n\t"
			      "srli   %0, %0, 13\n\t"
			      "slli   %0, %0, 13\n"
			      : "=a" (current)  );
	return current;
}

#define current get_current()

#endif /* _ASMLANGUAGE */

#endif /* __ASM_XTENSA_CURRENT_H */
