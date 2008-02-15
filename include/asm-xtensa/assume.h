#ifndef __ASM_XTENSA_ATOMIC_H
#define __ASM_XTENSA_ATOMIC_H

/* 
 * include/asm-xtensa/assume.h
 *
 * This file enforces various assumptions about all Xtensa
 * configurations that the architecture-specific code for Linux
 * makes.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#include <xtensa/config/core.h>
#include <xtensa/config/system.h>


#if (XCHAL_HAVE_MMU != 1)
#error Linux must have an MMU!
#endif

#if (XCHAL_MMU_MAX_PTE_PAGE_SIZE != XCHAL_MMU_MIN_PTE_PAGE_SIZE)
#error Only one page size allowed!
#endif

#if (XCHAL_HAVE_WINDOWED != 1)
#error Linux requires the Xtensa Windowed Registers Option.
#endif

#endif /* __ASM_XTENSA_ATOMIC_H */
