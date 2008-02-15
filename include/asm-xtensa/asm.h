#ifndef __ASM_XTENSA_ASM_H
#define __ASM_XTENSA_ASM_H

/*
 * include/asm-xtensa/asm.h
 *
 * Useful macros for assembly.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#include <linux/config.h>

#ifdef __cplusplus
#define CPP_ASMLINKAGE extern "C"
#else
#define CPP_ASMLINKAGE
#endif

#define asmlinkage CPP_ASMLINKAGE

#define SYMBOL_NAME_STR(X) #X
#define SYMBOL_NAME(X) X
#ifdef __STDC__
#define SYMBOL_NAME_LABEL(X) X##:
#else
#define SYMBOL_NAME_LABEL(X) X/**/:
#endif

#define __ALIGN .align 4
#define __ALIGN_STR ".align 4"

#ifdef __ASSEMBLY__

#define ALIGN __ALIGN
#define ALIGN_STR __ALIGN_STR

#define ENTRY(name) \
  .globl SYMBOL_NAME(name); \
  ALIGN; \
  SYMBOL_NAME_LABEL(name)

#endif /* _ASMLANGUAGE */

#endif /* __ASM_XTENSA_ASM_H */
