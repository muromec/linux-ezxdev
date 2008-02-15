#ifndef __ASM_XTENSA_PARAM_H
#define __ASM_XTENSA_PARAM_H

/*
 * include/asm-xtensa/param.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#include <xtensa/config/core.h>

#ifndef HZ
#define HZ 100
#endif

#define EXEC_PAGESIZE	(1 << XCHAL_MMU_MIN_PTE_PAGE_SIZE)

#ifndef NGROUPS
#define NGROUPS		32
#endif

#ifndef NOGROUP
#define NOGROUP		(-1)
#endif

#ifdef __KERNEL__
# define CLOCKS_PER_SEC	HZ	/* frequency at which times() counts */
#endif

#define MAXHOSTNAMELEN	64	/* max length of hostname */

#endif /* __ASM_XTNESA_PARAM_H */
