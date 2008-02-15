#ifndef __ASM_XTENSA_KSTACK_H
#define __ASM_XTENSA_KSTACK_H

/*
 * include/asm-xtensa/kstack.h
 *
 * Both C and Assembly source code needs the constant _KSTK_TOS_OFFSET.
 * This file isolates the mess.  Assembly code including this file should 
 * first define _ASMLANGUAGE.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Author:	Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 */


#define KERNEL_STACK_SIZE 8192

/* __KSTK_TOS_OFFSET defines the constant offset from the lowest address in
   the kernel-stack space (i.e., the task_struct structure) to the top of
   an exception stack frame at the bottom of the kernel stack (toward the
   highest address).

   -32, because "everybody's doing it" (padding, probably)
   -16, for spilling registers
   -PT_SIZE, because we'll always stack an exception frame 
*/

#ifdef _ASMLANGUAGE
#include <asm/offset.h>
#define __KSTK_TOS_OFFSET (KERNEL_STACK_SIZE - 32 - 16 - PT_SIZE)

#else  /* not assembly code */
#include <asm/ptrace.h>
#define __KSTK_TOS_OFFSET (KERNEL_STACK_SIZE - 32 - 16 - sizeof(struct pt_regs))

#endif  /* _ASMLANGUAGE */

#endif /* _ASM_XTENSA_KSTACK_H */
