/*
 *  linux/include/asm-arm/arch-omap/dma.h
 *
 *  derived from:
 *    linux/include/asm-arm/arch-sa1100/dma.h
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
#define MAX_OMAP_DMA_CHANNELS 16  /* + LCD channel, which is handled by LCD controller. */

/*
 * Maximum physical DMA buffer size, and smaller CUT size is a nice
 * page size.
 */
#define MAX_DMA_SIZE		0x1000000 /* Max size is larger, but this is OK for now. */
#define CUT_DMA_SIZE		0x1000

/*
 * All possible OMAP1610 devices a DMA channel can be attached to.
 *  See Table 8-1 in TRM.
 */
typedef enum {
        eDMANotSync,       /* 0 */
        eMCSI1Tx,
        eMCSI1Rx,
        eI2CRx,
        eI2CTx,
        eExt0,             /* 5 */
        eExt1,
        eMicroWireTx,
        eAudioTx,
        eAudioRx,
        eOpticalTx,        /* 10 */
        eOpticalRx,
        eBluetoothTx,
        eBluetoothRx,
        eModemTx,
        eModemRx,          /* 15 */
        eModemDataTx,
        eModemDataRx,
        eUART3Tx,
        eUART3Rx,
        eCameraRx,         /* 20 */
        eMMCTx,
	eMMCRx,
	eNANDEoB,
	eIRQLCDLINE,
	eMemoryStick,      /* 25 */
        eUSBRx0,
        eUSBRx1,
        eUSBRx2,
        eUSBTx0,
	eUSBTx1,           /* 30 */
	eUSBTx2,
	eDESin,
	eSPITx,
	eSPIRx,
	eSHAMD5,           /* 35 */
	eCCPAttn,
	eCCPFIFOne,
	eCMTAPETx0,
	eCMTAPERx0,
	eCMTAPETx1,        /* 40 */
	eCMTAPERx1,
	eCMTAPETx2,
	eCMTAPERx2,
	eCMTAPETx3,
	eCMTAPERx3,        /* 45 */
	eCMTAPETx4,
	eCMTAPERx4,
	eCMTAPETx5,
	eCMTAPERx5,
	eCMTAPETx6,        /* 50 */
	eCMTAPERx6,
	eCMTAPETx7,
	eCMTAPERx7,
	eMMC2Tx,
	eMMC2Rx,           /* 55 */
	eDESout,
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

/*
  Defines for register state.
*/

#define DCSDP_DST_BURST_EN_BIT 14
#define DCSDP_SRC_BURST_EN_BIT  7
#define   NO_BURST 0x0
#define   BURST_4  0x2
#define   BURST_8  0x3
#define DCSDP_DST_PACK         (1<<13)
#define DCSDP_DEST_PORT_BIT     9
#define DCSDP_SRC_PORT_BIT      2
#define   PORT_EMIFF  0x0
#define   PORT_EMIFS  0x1     /* old EMIF */
#define   PORT_OCP_T1 0x2     /* old IMIF */
#define   PORT_TIPB   0x3
#define   PORT_OCP_T2 0x4     /* old LOCAL */
#define   PORT_MPUI   0x5     /* old TIPB_MPUI */
#define DCSDP_SRC_PACK          (1<<6)
#define DCSDP_DATA_TYPE_BIT     0
#define   DATA_TYPE_S8  0x0
#define   DATA_TYPE_S16 0x1
#define   DATA_TYPE_S32 0x2


#define DCSR_ERROR  0x3
#define DCSR_SYNC_SET (1 << 6)

#define DCCR_FS      (1 << 5)
#define DCCR_PRIO    (1 << 6)
#define DCCR_EN      (1 << 7)
#define DCCR_AI      (1 << 8)
#define DCCR_REPEAT  (1 << 9)
#define DCCR_N31COMP (1 << 10) /* if 0 the channel works in 3.1 compatible mode */
#define DCCR_EP      (1 << 11)
#define DCCR_SRC_AMODE_BIT  12
#define DCCR_SRC_AMODE_MASK (0x3<<12)
#define DCCR_DST_AMODE_BIT  14
#define DCCR_DST_AMODE_MASK (0x3<<14)
#define   AMODE_CONST        0x0
#define   AMODE_POST_INC     0x1
#define   AMODE_SINGLE_INDEX 0x2
#define   AMODE_DOUBLE_INDEX 0x3

#define LCH_TYPE_2D 0  /* non-iterleaved transfers */
#define LCH_TYPE_G  1  /* graphical */
#define LCH_TYPE_P  2  /* periferial - most common type */
#define LCH_TYPE_D  4  /* LCD */
#define LCH_TYPE_PD 15 /* dedicated channel */

/*
 * Defines for the autoidle set routine.
 */

#define DISABLE_AUTOIDLE 0
#define ENABLE_AUTOIDLE  1

typedef void (*dma_callback_t)(void *data);

extern void omap_dma_autoidle_set(int);
extern int omap_request_dma(dma_device_t, const char *, dma_callback_t,
			    void *, dma_regs_t **);
extern void omap_free_dma(dma_regs_t *);
extern int omap_start_dma(dma_regs_t *, dma_addr_t, u_int);
extern dma_addr_t omap_get_dma_pos(dma_regs_t *);
extern void omap_clear_dma(dma_regs_t *);
extern void omap_reset_dma(dma_regs_t *);
extern int omap_dma_setup( dma_device_t, dma_direction_t);
extern int omap_dma_busy(void);

extern int omap_request_sound_dma(dma_device_t, const char *, dma_callback_t,
				  void *, dma_regs_t **);
extern void omap_free_sound_dma(dma_regs_t *);
extern int omap_start_sound_dma(dma_regs_t *, dma_addr_t, u_int);
extern void omap_clear_sound_dma(dma_regs_t *);
extern void omap_reset_sound_dma(dma_regs_t *);

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
