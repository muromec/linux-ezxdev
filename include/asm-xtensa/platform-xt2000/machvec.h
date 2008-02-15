/*
 * include/asm-xtensa/platform-xt2000/machvec.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

/*
 * This file defines the platform-dependent functions, that are
 * provided by the xt2000 port 
 */

#ifndef __ASM_XTENSA_XT2000_MACHVEC
#define __ASM_XTENSA_XT2000_MACHVEC

#include <linux/pci.h>

extern void xt2000_setup (char **);
extern int xt2000_pci_map_irq (struct pci_dev*, u8, u8);

/* generic functions (must be implemented) */

#define xtvec_setup		xt2000_setup

/* PCI related functions (when the target supports PCI) */

#define xtvec_pci_swizzle	common_swizzle	/* use the 'common swizzler' */
#define xtvec_pci_map_irq	xt2000_pci_map_irq
#define xtvec_pci_fixup		xt2000_pci_fixup

#endif /* __ASM_XTENSA_XT2000_MACHVEC */

