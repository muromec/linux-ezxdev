/*
 * include/asm-xtensa/platform-xt2000/hardware.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

/*
 * This file contains the hardware configuration of the XT2000 board.
 */

#ifndef __ASM_XTENSA_XT2000_HARDWARE
#define __ASM_XTENSA_XT2000_HARDWARE

/* 
 * Memory configuration.
 */

#define PLATFORM_DEFAULT_MEM_START XSHAL_RAM_PADDR
#define PLATFORM_DEFAULT_MEM_SIZE XSHAL_RAM_VSIZE

/*
 * Interrupt configuration.
 */

/* The XT2000 uses the V3 as a cascaded interrupt controller for the PCI bus */
#define PLATFORM_NR_IRQS 3
#define IRQ_PCI_A (XCHAL_NUM_INTERRUPTS + 0)
#define IRQ_PCI_B (XCHAL_NUM_INTERRUPTS + 1)
#define IRQ_PCI_C (XCHAL_NUM_INTERRUPTS + 2)

#endif /* __ASM_XTENSA_XT2000_HARDWARE */
