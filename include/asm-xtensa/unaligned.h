#ifndef __ASM_XTENSA_UNALIGNED_H
#define __ASM_XTENSA_UNALIGNED_H

/*
 * include/asm-xtensa/unaligned.h
 *
 * Xtensa can't handle unaligned accesses
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#include <linux/string.h>

/* Use memmove here, so gcc does not insert a __builtin_memcpy. */

#define get_unaligned(ptr) \
  ({ __typeof__(*(ptr)) __tmp; memmove(&__tmp, (ptr), sizeof(*(ptr))); __tmp; })

#define put_unaligned(val, ptr)				\
  ({ __typeof__(*(ptr)) __tmp = (val);			\
     memmove((ptr), &__tmp, sizeof(*(ptr)));		\
     (void)0; })

#endif /* __ASM_XTENSA_UNALIGNED_H */
