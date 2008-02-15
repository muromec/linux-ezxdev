/*
 *  linux/include/asm-arm/arch-omap/dma-sound-omap1610.h
 *
 *  derived from:
 *       linux/drivers/sound/omap-audio.h
 *    
 *
 *  Copyright (C) 2004 MontaVista Software, Inc.
 *    <source@mvista.com>
 *  Copyright (C) 1997,1998 Russell King
 *  Copyright (C) 2000 Nicolas Pitre
 *  Copyright (C) 2001 RidgeRun, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_DMA_SOUND_OMAP1610_H
#define __ASM_ARCH_DMA_SOUND_OMAP1610_H


/*
 *audio_stream_t and audio_buf_t moved here from
 *private header of TI OMAP1610 sound driver implemenation
 *linux/drivers/sound/omap-audio.h
 *(arch == arch-omap1610), because dma-omap.c exports
 *a fiew sound-dma-dedicated functions, which need some knowledge
 *about audio_stream_t.dma_regs
 */


/*
 * Buffer Management
 */

typedef struct {
	int offset;		/* current offset */
	char *data;		/* points to actual buffer */
	dma_addr_t dma_addr;	/* physical buffer address */
	int dma_ref;		/* DMA refcount */
	int master;		/* owner for buffer allocation, contain size when true */
} audio_buf_t;

typedef struct {
	char *id;	/* identification string */
	audio_buf_t *buffers;	/* pointer to audio buffer structures */
	u_int usr_head;		/* user fragment index */
	u_int dma_head;		/* DMA fragment index to go */
	u_int dma_tail;		/* DMA fragment index to complete */
	u_int fragsize;		/* fragment i.e. buffer size */
	u_int nbfrags;		/* nbr of fragments i.e. buffers */
	u_int pending_frags;	/* Fragments sent to DMA */
	dma_device_t dma_dev;	/* device identifier for DMA */
	dma_regs_t *dma_regs;	/* points to our DMA registers */
	int bytecount;		/* nbr of processed bytes */
	int fragcount;		/* nbr of fragment transitions */
	struct semaphore sem;	/* account for fragment usage */
	wait_queue_head_t wq;	/* for poll */
	int dma_spinref;	/* DMA is spinning */
	int mapped:1;		/* mmap()'ed buffers */
	int active:1;		/* actually in progress */
	int stopped:1;		/* might be active but stopped */
	int spin_idle:1;	/* have DMA spin on zeros when idle */
} audio_stream_t;



#endif /* __ASM_ARCH_DMA_SOUND_OMAP1610_H */
