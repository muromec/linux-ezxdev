#ifndef __ASM_XTENSA_XTUTIL_H
#define __ASM_XTENSA_XTUTIL_H

/*
 * include/asm-xtensa/xtutil.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */


/* LOCKLEVEL defines the interrupt level that masks all
 * general-purpose interrupts.
 */
#define LOCKLEVEL 1

/*  String macro expansion defines.  The XTSTR macro expands a macro
 *  into a string.
 */
#define _XTSTR(x) # x
#define XTSTR(x) _XTSTR(x)

/* Some invocations of macro XTSTR are using it to get the register
 * number of PS. The use of PS requires the following headerfile.
 */
#include <xtensa/config/specreg.h>

#endif /* __ASM_XTENSA_XTUTIL_H */
