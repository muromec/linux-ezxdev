/*
 * BK Id: SCCS/s.dma.h 1.14 02/27/02 08:21:07 trini
 */
/*
 * linux/include/asm/dma.h: Defines for using and allocating dma channels.
 * Written by Hennus Bergman, 1992.
 * High DMA channel support & info by Hannu Savolainen
 * and John Boyd, Nov. 1992.
 * Changes for ppc sound by Christoph Nadig
 *
 * Moved most code (ISA-specific) to asm/ppc_isa_dma.h so that 4xx could have
 * a common API - armin Feb, 02
 */

#ifdef __KERNEL__

#include <linux/config.h>
#include <asm/io.h>

/*
 * Note: Adapted for PowerPC by Gary Thomas
 * Modified by Cort Dougan <cort@cs.nmt.edu>
 *
 * None of this really applies for Power Macintoshes.  There is
 * basically just enough here to get kernel/dma.c to compile.
 *
 * There may be some comments or restrictions made here which are
 * not valid for the PReP platform.  Take what you read
 * with a grain of salt.
 */


#ifndef _ASM_DMA_H
#define _ASM_DMA_H

/* The maximum address that we can perform a DMA transfer to on this platform */
/* Doesn't really apply... */
#define MAX_DMA_ADDRESS		0xFFFFFFFF

/* in arch/ppc/kernel/setup.c -- Cort */
extern unsigned long DMA_MODE_WRITE, DMA_MODE_READ;
extern unsigned long ISA_DMA_THRESHOLD;

#ifdef HAVE_REALLY_SLOW_DMA_CONTROLLER
#define dma_outb	outb_p
#else
#define dma_outb	outb
#endif

#define dma_inb		inb

#if defined(CONFIG_PPC4xx_DMA)
#include <asm/ppc4xx_dma.h>
#else
#include <asm/ppc_isa_dma.h>
#endif

#ifndef MAX_DMA_CHANNELS
#define MAX_DMA_CHANNELS	8
#endif
#ifdef CONFIG_PCI
extern int isa_dma_bridge_buggy;
#else
#define isa_dma_bridge_buggy	(0)
#endif


#endif /* _ASM_DMA_H */
#endif /* __KERNEL__ */
