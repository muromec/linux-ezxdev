#ifndef __ASM_XTENSA_DMA_H
#define __ASM_XTENSA_DMA_H


/*
 * include/asm-xtensa/dma.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#include <linux/config.h>
#include <asm/io.h>		/* need byte IO */

#define MAX_DMA_CHANNELS 8

/* The maximum address that we can perform a DMA transfer to on this platform */
/* Don't define MAX_DMA_ADDRESS; it's useless on the SuperH and any
   occurrence should be flagged as an error.  */
/* But... */
/* XXX: This is not applicable to SuperH, just needed for alloc_bootmem */
#define MAX_DMA_ADDRESS      (PAGE_OFFSET+0x10000000)

extern int request_dma(unsigned int dmanr, const char * device_id);	/* reserve a DMA channel */
extern void free_dma(unsigned int dmanr);	/* release it again */

#ifdef CONFIG_PCI
extern int isa_dma_bridge_buggy;
#else
#define isa_dma_bridge_buggy 	(0)
#endif


#endif /* __ASM_XTENSA_DMA_H */
