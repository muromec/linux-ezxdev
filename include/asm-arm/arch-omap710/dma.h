/*
 *  linux/include/asm-arm/arch-omap/dma.h
 *  derived from:
 *    linux/include/asm-arm/arch-sa1100/dma.h
 *
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
#ifndef __ASM_ARCH_DMA_H
#define __ASM_ARCH_DMA_H

/*
 * This is the maximum DMA address that can be DMAd to.
 */
#define MAX_DMA_ADDRESS		0xffffffff

/*
 * The regular generic DMA interface is inappropriate for the
 * SA1100 DMA model.  None of the SA1100 specific drivers using
 * DMA are portable anyway so it's pointless to try to twist the
 * regular DMA API to accommodate them.
 */
#define MAX_DMA_CHANNELS	0

#define MAX_OMAP_DMA_CHANNELS 9  // + LCD channel, which is handled by LCD controller

/*
 * Maximum physical DMA buffer size, and smaller CUT size is a nice
 * page size.
 */
#define MAX_DMA_SIZE		0x1000000  // Max size is larger, but this is OK for now
#define CUT_DMA_SIZE		0x1000


/*
 * All possible OMAP710 devices a DMA channel can be attached to.
 *  See Table 5-7 in TRM.
 */
typedef enum {
        eDMANotSync,
        eExt1,
        eExt2,
        eExt3,
        eExt4,             // 5
        eExt5,
        eExt6,
        eMicroWireTx,
        eAudioTx,
        eAudioRx,
        eOpticalTx,        // 10
        eOpticalRx,
        eBluetoothTx,
        eBluetoothRx,
        eModemTx,
        eModemRx,          // 15
        eModemDataTx,
        eModemDataRx,
        eUART3Tx,
        eUART3Rx,
        eCameraRx,         // 20
        eUSBTx0,
        eUSBTx1,
        eUSBRx0,
        eUSBRx1,           // 24
} dma_device_t;

typedef enum {
        eDmaIn,
        eDmaOut
} dma_direction_t;

/*
 * DMA control register structure, without LCD registers
 */
typedef struct {
        volatile u16 csdp;      // channel s/d parameters -- working set (current transfer)
        volatile u16 ccr;       // channel control -- working set
        volatile u16 cicr;      // channel interrupt control -- working set
        volatile u16 csr;       // channel status -- working set
        volatile u16 cssa_l;    // source lower bits -- programming set (next transfer)
        volatile u16 cssa_u;    // source upper bits -- programming set
        volatile u16 cdsa_l;    // destn lower bits -- programming set
        volatile u16 cdsa_u;    // destn upper bits -- programming set
        volatile u16 cen;       // channel element number -- programming set
        volatile u16 cfn;       // channel frame number -- programming set
        volatile u16 cfi;       // channel frame index -- programming set
        volatile u16 cei;       // channel element index -- programming set
        volatile u16 cpc;       // channel progress counter
        volatile u16 null[19];  // for alignment
} dma_regs_t;

/*
  Defines for register state.
*/

#define DCSR_ERROR  0x3
#define DCSR_SYNC_SET (1 << 6)
#define DCCR_EN     (1 << 7)
#define DCCR_AI     (1 << 8)
#define DCCR_REPEAT (1 << 9)
#define DCCR_FF     (1 << 10)
#define DCCR_EP     (1 << 11)

/*
  If we're using the iPAQ sound system, we need to remap their DMA to our DMA.
*/
#if defined(CONFIG_SOUND_SA1100) || defined(CONFIG_SOUND_SA1100_MODULE)

#define sa1100_request_dma   omap_request_dma
#define sa1100_free_dma      omap_free_dma
#define sa1100_start_dma     omap_start_dma
#define sa1100_get_dma_pos   omap_get_dma_pos
#define sa1100_reset_dma     omap_reset_dma
#define sa1100_stop_dma      omap_stop_dma
#define sa1100_resume_dma    omap_resume_dma
#define sa1100_clear_dma     omap_clear_dma
#endif

/*
 * Defines for the autoidle set routine.
 */

#define DISABLE_AUTOIDLE 0
#define ENABLE_AUTOIDLE  1

typedef void (*dma_callback_t)(void *data, u16 status);

extern void omap_dma_autoidle_set( int set );
extern int omap_request_dma( dma_device_t device, const char *device_id,
                             dma_callback_t callback, void *data,
                             dma_regs_t **regs );
extern void omap_free_dma( dma_regs_t *regs );
extern int omap_start_dma( dma_regs_t *regs, dma_addr_t dma_ptr, u_int size );
extern dma_addr_t omap_get_dma_pos(dma_regs_t *regs);
extern void omap_reset_dma(dma_regs_t *regs);
extern int omap_dma_setup( dma_device_t device, dma_direction_t direction );
extern int omap_dma_busy( void );

/**
 * 	omap_stop_dma - stop DMA in progress
 * 	@regs: identifier for the channel to use
 *
 * 	This stops DMA without clearing buffer pointers. Unlike
 * 	omap_clear_dma() this allows subsequent use of omap_resume_dma()
 * 	or omap_get_dma_pos().
 *
 * 	The @regs identifier is provided by a successful call to
 * 	omap_request_dma().
 **/

#define omap_stop_dma(regs)	((regs)->ccr &= ~(DCCR_EN))

/**
 * 	omap_resume_dma - resume DMA on a stopped channel
 * 	@regs: identifier for the channel to use
 *
 * 	This resumes DMA on a channel previously stopped with
 * 	omap_stop_dma().
 *
 * 	The @regs identifier is provided by a successful call to
 * 	omap_request_dma().
 **/

#define omap_resume_dma(regs)	((regs)->ccr |= (DCCR_EN))

/**
 * 	omap_clear_dma - clear DMA pointers
 * 	@regs: identifier for the channel to use
 *
 * 	This clear any DMA state so the DMA engine is ready to restart
 * 	with new buffers through omap_start_dma(). Any buffers in flight
 * 	are discarded.
 *
 * 	The @regs identifier is provided by a successful call to
 * 	omap_request_dma().
 **/

#define omap_clear_dma(regs)	((regs)->ccr &= ~(DCCR_REPEAT | DCCR_AI | DCCR_EP | DCCR_EN))


#ifdef CONFIG_IRAM_ALLOC

#include <asm/arch/sizes.h>

// Setup a fake 1MB IRAM region in normal SDRAM.

static inline void 
__arch_adjust_zones(int node, unsigned long *size, unsigned long *holes)
{
          size[0] -= (SZ_1M >> PAGE_SHIFT);
          size[3] = (SZ_1M >> PAGE_SHIFT);
}


#endif

#endif /* _ASM_ARCH_DMA_H */

