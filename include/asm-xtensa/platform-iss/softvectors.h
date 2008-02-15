/*
 * include/asm-xtensa/platform-iss/softvectors.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

/*
 * This file defines the platform-dependent functions, that are
 * provided by the iss port 
 */

#ifndef __ASM_XTENSA_ISS_SOFTVECTORS
#define __ASM_XTENSA_ISS_SOFTVECTORS

#include <linux/pci.h>

extern void iss_setup (char **);

/* generic functions (must be implemented) */

#define xtvec_setup iss_setup

#endif /* __ASM_XTENSA_ISS_SOFTVECTORS */

