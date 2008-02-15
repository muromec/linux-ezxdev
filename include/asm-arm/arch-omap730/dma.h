/*
 *  linux/include/asm-arm/arch-omap730/dma.h
 *
 *  derived from:
 *    linux/include/asm-arm/arch-omap1610/dma.h
 *
 *  Copyright (C) 1997,1998 Russell King
 *  Copyright (C) 2000 Nicolas Pitre
 *  Copyright (C) 2001 RidgeRun, Inc.
 *  Copyright (C) 2004 MPC-Data Limited
 *    Dave Peverley <dpeverley@mpc-data.co.uk>
 *  Copyright (C) 2004 MontaVista Software, Inc.
 *    <source@mvista.com>
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
 * OMAP DMA model.  None of the OMAP specific drivers using
 * DMA are portable anyway so it's pointless to try to twist the
 * regular DMA API to accommodate them.
 */
#define MAX_DMA_CHANNELS	0

#define MAX_OMAP_DMA_CHANNELS 16  /* + LCD channel, which is handled by LCD controller */

/*
 * Maximum physical DMA buffer size, and smaller CUT size is a nice
 * page size.
 */
#define MAX_DMA_SIZE		0x1000000  /* Max size is larger, but this is OK for now */
#define CUT_DMA_SIZE		0x1000

/*
 * All possible OMAP730 devices a DMA channel can be attached to.
 *  See Table 26-1 in TRM.
 */
typedef enum {
        eDMANotSync,       /* 0 */
        eMCSI1Tx,
        eMCSI1Rx,
        eI2CRx,
        eI2CTx,
        eVLYNQ,            /* 5 */
        eSHAMD5,
        eMicroWireTx,
        eMcBSP1Tx,
        eMcBSP1Rx,
        eMcBSP2Tx,        /* 10 */
        eMcBSP2Rx,
        eUART1Tx,
        eUART1Rx,
        eUART2Tx,
        eUART2Rx,          /* 15 */
        eDESin,
        eDESout,
        eSMCTx,
        eSMCRx,
        eCameraRx,         /* 20 */
        eMMCTx,
	eMMCRx,
	eNANDEoB,
	eEACRec,
	eEACPlay,          /* 25 */
        eUSBRx0,
        eUSBRx1,
        eUSBRx2,
        eUSBTx0,
	eUSBTx1,           /* 30 */
	eUSBTx2,
} dma_device_t;

typedef enum {
        eDmaIn,
        eDmaOut
} dma_direction_t;

/*
 * DMA control register structure, without LCD registers
 */
typedef struct {
        volatile u16 csdp;      /* channel s/d parameters -- working set (current transfer) */
        volatile u16 ccr;       /* channel control -- working set */
        volatile u16 cicr;      /* channel interrupt control -- working set */
        volatile u16 csr;       /* channel status -- working set */
        volatile u16 cssa_l;    /* source lower bits -- programming set (next transfer) */
        volatile u16 cssa_u;    /* source upper bits -- programming set */
        volatile u16 cdsa_l;    /* destn lower bits -- programming set */
        volatile u16 cdsa_u;    /* destn upper bits -- programming set */
        volatile u16 cen;       /* channel element number -- programming set */
        volatile u16 cfn;       /* channel frame number -- programming set */
        volatile u16 csfi;      /* channel source frame index -- programming set */
        volatile u16 csei;      /* channel source element index -- programming set */
        volatile u16 csac;      /* channel source address counter */
        volatile u16 cdac;      /* channel dest. address counter */
        volatile u16 cdei;      /* channel dest. element index -- programming set */
        volatile u16 cdfi;      /* channel dest. frame index -- programming set */
        volatile u16 color_l;   /* graphics&LCD channels color - lower bits */
        volatile u16 color_u;   /* graphics&LCD channels color - upper bits */
        volatile u16 ccr2;      /* channel control 2 */
	volatile u16 reserved;  /* reserved */
        volatile u16 clink_ctrl;/* link control */
        volatile u16 lch_ctrl;  /* channel type control */
        volatile u16 null[10];  /* for alignment */
} dma_regs_t;

/* The dma_channel_params_t structure is backported from the new OMAP DMA API in 
 * the 2.6 kernel.
 */
typedef struct dma_channel_params_t{
	int data_type; /* data type 8,16,32 */
	int elem_count; /* number of elements in a frame */
	int frame_count; /* number of frames in a element */

	int src_amode; /* constant , post increment, indexed , double indexed */
	int src_port;
	int src_packing;
	int src_burst;
	int src_start; /* source address : physical */

	int dest_amode; /* constant , post increment, indexed , double indexed */
	int dest_port;
	int dest_packing;
	int dest_burst;
	int dest_start; /* source address : physical */

	dma_device_t sync;	/* trigger attached if the channel is synchronized */
	int frame_sync; /* sycn on element or frame */
	int priority;  /* low or high prio */
	int auto_init; /* auto_init at end of transfer */
	int repeat;	   /* rept operation */
	int end_prog;  /* reinit itself */
	int omap31_comp_disable; /* OMAP3.2 or 3.0/3.1 compatible mode */

	int ie;			/* interrupt enabled */
}dma_channel_params;

/* convert a dma_regs_t pointer to a dma channel index */
#define OMAP_DMA_CH(dma_regs) (((unsigned int) (dma_regs) - (OMAP_DMA_BASE)) \
	/ sizeof(dma_regs_t))
	
/* convert a dma channel index to a dma_regs_t pointer */

#define OMAP_DMA_REGS(dma_ch) ((dma_regs_t *) \
	(OMAP_DMA_BASE + sizeof(dma_regs_t)*(dma_ch)))

/* DMA_CSDP Register Definitions */

#define DCSDP_DST_BURST_EN_BIT     14
#define DCSDP_SRC_BURST_EN_BIT     7
#define   NO_BURST                 0x0
#define   BURST_4                  0x2
#define   BURST_8                  0x3
#define DCSDP_DST_PACK             (1 << 13)
#define DCSDP_DEST_PORT_BIT        9
#define DCSDP_SRC_PORT_BIT         2
#define   PORT_SDRAM               0x0
#define   PORT_EMIFF               PORT_SDRAM /* for OMAP161x compatibility */
#define   PORT_EMIF                0x1
#define   PORT_IMIF                0x2
#define   PORT_TIPB                0x3
#define   PORT_LOCAL               0x4
#define   PORT_API                 0x5
#define DCSDP_SRC_PACK             (1 << 6)
#define DCSDP_DATA_TYPE_BIT        0
#define DCSDP_DATA_TYPE_MASK       (0x3)
#define   DATA_TYPE_S8             0x0
#define   DATA_TYPE_S16            0x1
#define   DATA_TYPE_S32            0x2

/* DMA_CCR Register Defines */

#define DCCR_FS                    (1 << 5)
#define DCCR_PRIO                  (1 << 6)
#define DCCR_EN                    (1 << 7)
#define DCCR_AUTO_INIT             (1 << 8)
#define DCCR_AI                    DCCR_AUTO_INIT /* for OMAP161x compatibility */
#define DCCR_REPEAT                (1 << 9)
#define DCCR_FIFO_FLUSH            (1 << 10)
#define DCCR_END_PROG              (1 << 11) 
#define DCCR_SRC_AMODE_BIT         12
#define DCCR_SRC_AMODE_MASK        (0x3 << 12)
#define DCCR_DST_AMODE_BIT         14
#define DCCR_DST_AMODE_MASK        (0x3 << 14)
#define   AMODE_CONST              0x0
#define   AMODE_POST_INC           0x1
#define   AMODE_SINGLE_INDEX       0x2
#define   AMODE_DOUBLE_INDEX       0x3

/* DMA_CICR Register Defines */

#define DCICR_TOUT_IE              (1 << 0)
#define DCICR_DROP_IE              (1 << 1)

/* DMA_CSR Register Defines */

#define DCSR_TOUT                  (1 << 0)
#define DCSR_DROP                  (1 << 1)
#define DCSR_HALF                  (1 << 2)
#define DCSR_BLOCK                 (1 << 5)
#define DCSR_SYNC_SET              (1 << 6)

/* DMA_LCH_CTRL Reguster Definitions */

#define LCH_TYPE_2D                0   /* non-iterleaved transfers */
#define LCH_TYPE_G                 1   /* graphical */
#define LCH_TYPE_P                 2   /* periferial - most common type */
#define LCH_TYPE_D                 4   /* LCD */
#define LCH_TYPE_PD                15  /* dedicated channel */

/*
 * Defines for the autoidle set routine.
 */

#define DISABLE_AUTOIDLE 0
#define ENABLE_AUTOIDLE  1

typedef void (*dma_callback_t)(void *data);

extern void omap_dma_autoidle_set( int set );
extern int omap_request_dma( dma_device_t device, const char *device_id,
                             dma_callback_t callback, void *data,
                             dma_regs_t **regs );
extern int omap_dma_get_status( dma_regs_t *regs );
extern void omap_free_dma( dma_regs_t *regs );
extern int omap_start_dma( dma_regs_t *regs, dma_addr_t dma_ptr, u_int size );
extern dma_addr_t omap_get_dma_pos(dma_regs_t *regs);
extern void omap_clear_dma(dma_regs_t *regs);
extern void omap_reset_dma(dma_regs_t *regs);
extern int omap_dma_setup( dma_device_t device, dma_direction_t direction );
extern int omap_dma_busy( void );

/* The omap_set_dma_* routines are backported from the new OMAP DMA API in the 
 * 2.6 kernel.
 */
extern void omap_set_dma_transfer_params(int lch, int data_type, int elem_count,
	int frame_count, int frame_sync, dma_device_t sync, int priority,
	int auto_init, int repeat, int end_prog, int omap31_comp_disable,
	int interrupt_enabled);
extern void omap_set_dma_src_params(int lch, int src_port, int src_amode, 
	int src_start, int src_packing, int src_burst);
extern void omap_set_dma_dest_params(int lch, int dest_port, int dest_amode, 
	int dest_start, int dest_packing, int dest_burst);
extern void omap_set_dma_params(int lch, dma_channel_params params);

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

#endif /* _ASM_ARCH_DMA_H */
