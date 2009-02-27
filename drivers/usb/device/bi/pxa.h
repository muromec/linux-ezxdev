/*
 * linux/drivers/usb/device/bi/pxa.h
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */


#define EP0_PACKETSIZE  0x10

#define UDC_MAX_ENDPOINTS       6

#define UDC_NAME        "PXA"

#ifdef CONFIG_SABINAL_DISCOVERY
/* SHARP A3 specific GPIO assignments */
#define USBD_CONNECT_GPIO	50
#if defined(CONFIG_SABINAL_DISCOVERY_DVT) || defined(CONFIG_SABINAL_DISCOVERY_DVT2)
#define USBD_CONNECT_HIGH	 1   /* Pin high for connect (undef for pin low) */
#else
#undef USBD_CONNECT_HIGH
#endif

#define CKEN11_USB	USB_CE		 /* USB Unit Clock Enable */
#endif

/*
 * DMA Notes
 *
 * DCSRX	DMA Channel Control/Status register	
 * DRCMRX	DMA Request to Channel Map register
 * DDADRX	DMA Descriptor Address register
 *
 * DSADRX	DMA Source Address register (RO)	
 * DTADRX	DMA Target Address register (RO)
 * DCMCDX	DMA Command register (RO)
 *
 *
 */

typedef enum {
	DMA_READY,
	DMA_ACTIVE, 
	DMA_DONE
} dma_status_t;

typedef struct {
	int offset;
	dma_status_t dma_status;

	unsigned char *dma_buf;
	dma_addr_t dma_buf_phys;

#ifndef CONFIG_SABINAL_DISCOVERY
	pxa_dma_desc *dma_desc;
	dma_addr_t dma_desc_phys;
#endif

} usb_buf_t;

typedef struct {
	char *name;
	u_long dcmd;
	volatile u32 *drcmr;
	u_long dev_addr;

	u_int dma_ch;
	usb_buf_t *buffers;

	unsigned char *dma_buf;
	dma_addr_t dma_buf_phys;

#ifndef CONFIG_SABINAL_DISCOVERY
	pxa_dma_desc *dma_desc;
	dma_addr_t dma_desc_phys;
#endif
	int num;
	int size;

        int ep;
        struct usb_endpoint_instance *endpoint;
} usb_stream_t;


/*
 * Reserved Register Definitions
 */
#define UDCRSVD0        __REG(0x40600004)  /* UDC Reserved Register 0 */
#define UDCRSVD1        __REG(0x40600008)  /* UDC Reserved Register 1 */
#define UDCRSVD2        __REG(0x4060000c)  /* UDC Reserved Register 2 */

#define UFNHR           __REG(0x40600060)  /* UDC Frame Number High Register */
#define UFNLR           __REG(0x40600064)  /* UDC Frame Number Low Register */



/*
 *      7654 3210
 * de   1101 1110
 *
 *      fedc ba98
 * 7b   0111 1011
 *
 *
 */

#define UDCRSVD0_DBL0   (1 << 0)                /* enable endpoint 0 double buffer (always zero)) */
#define UDCRSVD0_DBL1   (1 << 1)                /* enable endpoint 1 double buffer (read/write) */
#define UDCRSVD0_DBL2   (1 << 2)                /* enable endpoint 2 double buffer (read/write) */
#define UDCRSVD0_DBL3   (1 << 3)                /* enable endpoint 3 double buffer (read/write) */
#define UDCRSVD0_DBL4   (1 << 4)                /* enable endpoint 4 double buffer (read/write) */
#define UDCRSVD0_DBL5   (1 << 5)                /* enable endpoint 5 double buffer (always zero)) */
#define UDCRSVD0_DBL6   (1 << 6)                /* enable endpoint 6 double buffer (read/write) */
#define UDCRSVD0_DBL7   (1 << 7)                /* enable endpoint 7 double buffer (read/write) */

#define UDCRSVD0_DBL8   (1 << 0)                /* enable endpoint 8 double buffer (read/write) */
#define UDCRSVD0_DBL9   (1 << 1)                /* enable endpoint 9 double buffer (read/write) */
#define UDCRSVD0_DBL10  (1 << 2)                /* enable endpoint 10 double buffer (always zero)) */
#define UDCRSVD0_DBL11  (1 << 3)                /* enable endpoint 11 double buffer (read/write) */
#define UDCRSVD0_DBL12  (1 << 4)                /* enable endpoint 12 double buffer (read/write) */
#define UDCRSVD0_DBL13  (1 << 5)                /* enable endpoint 13 double buffer (read/write) */
#define UDCRSVD0_DBL14  (1 << 6)                /* enable endpoint 14 double buffer (read/write) */
#define UDCRSVD0_DBL15  (1 << 7)                /* enable endpoint 15 double buffer (always zero)) */

/* ep0 states */
#define WAIT_FOR_SETUP          0
#define DATA_STATE_XMIT         1
#define DATA_STATE_NEED_ZLP     2
#define WAIT_FOR_OUT_STATUS     3
/* Added by Stanley */
#define DATA_STATE_RECV         4


// XXX Send to nico@cam.org to include in pxa-regs.h
//

/* c.f. Table 12-20 UDC Control Register (UDCCR) */

#define	UDCCR_REM	(1 << 7)		/* Reset interrupt mask (read/write) */
#define	UDCCR_RSTIR	(1 << 6)		/* Reset interrupt request (read/write 1 to clear */
#define	UDCCR_SRM	(1 << 5)		/* Suspend/resume interrupt mask (read/write) */
#define	UDCCR_SUSIR	(1 << 4)		/* Suspend interrupt request (read/write 1 to clear) */
#define	UDCCR_RESIR	(1 << 3)		/* Resume interrupt request (read/write 1 to clear) */
#define	UDCCR_RSM	(1 << 2)		/* Device Resume (read/write 1 to set) */
#define	UDCCR_UDA	(1 << 1)		/* UDC active (read-only) */
#define	UDCCR_UDE	(1 << 0)		/* UDC enable (read/write) */


/* c.f. Table 12-21 UDC Endpoint 0 Control Status Register (UDCCS0) */

#define	UDCCS0_OPR	(1 << 0)		/* OUT packet ready (read/write 1 to clear) */
#define	UDCCS0_IPR	(1 << 1)		/* IN packet ready (always read 0/write 1 to set) */
#define	UDCCS0_FTF	(1 << 2)		/* Flush Tx FIFO (always read 0/write 1 to set) */
#define	UDCCS0_DRWF	(1 << 3)		/* Device remote wake feature (read only) */
#define	UDCCS0_SST	(1 << 4)		/* Sent stall (read/write 1 to clear) */
#define	UDCCS0_FST	(1 << 5)		/* Force stall (read/write 1 to set) */
#define	UDCCS0_RNE	(1 << 6)		/* Receive FIFO not empty (read only) */
#define	UDCCS0_SA	(1 << 7)		/* Setup Active (read/write 1 to clear) */


/* c.f. Table 12-22 UDC Endpoint X where x is 1, 6, 11 - BULK IN (UDCCSX) */

#define	UDCCS_BI_TFS	(1 << 0)		/* Transmit FIFO service (read only) */
#define	UDCCS_BI_TPC	(1 << 1)		/* Transmit packet complete (read/write 1 to clear) */
#define	UDCCS_BI_FTF	(1 << 2)		/* Flush Tx FIFO (always read 0/write 1 to set) */
#define	UDCCS_BI_TUR	(1 << 3)		/* Transmit FIFO underrun (read/write 1 to clear) */
#define	UDCCS_BI_SST	(1 << 4)		/* Sent STALL (read/write 1 to clear) */
#define	UDCCS_BI_FST	(1 << 5)		/* Force STALL (read/write) */

#define	UDCCS_BI_TSP	(1 << 7)		/* Transmit short packet (always read 0/write 1 to set) */


/* c.f. Table 12-23 UDC Endpoint X where x is 2, 7, 12 - BULK OUT (UDCCSX) */

#define	UDCCS_BO_RFS	(1 << 0)		/* Receive FIFO service (read only) */
#define	UDCCS_BO_RPC	(1 << 1)		/* Receive packet complete (read/write 1 to clear) */

#define	UDCCS_BO_DME	(1 << 3)		/* DMA Enable (read/write) */
#define	UDCCS_BO_SST	(1 << 4)		/* Sent STALL (read/write 1 to clear) */
#define	UDCCS_BO_FST	(1 << 5)		/* Force STALL (read/write) */
#define	UDCCS_BO_RNE	(1 << 6)		/* Receive FIFO not empty (read only) */
#define	UDCCS_BO_RSP	(1 << 7)		/* Receive short packet (read only) */

/* c.f. Table 12-24 UDC Endpoint X where x is 3, 8, 13 - ISO IN (UDCCSX) */

#define	UDCCS_II_TFS	(1 << 0)		/* Transmit FIFO service (read only) */
#define	UDCCS_II_TPC	(1 << 1)		/* Transmit packet complete (read/write 1 to clear) */
#define	UDCCS_II_FTF	(1 << 2)		/* Flush Tx FIFO (always read 0/write 1 to set) */
#define	UDCCS_II_TUR	(1 << 3)		/* Transmit FIFO underrun (read/write 1 to clear) */

#define	UDCCS_II_TSP	(1 << 7)		/* Transmit short packet (always read 0/write 1 to set) */

/* c.f. Table 12-25 UDC Endpoint X where x is 4, 9, 14 - ISO OUT (UDCCSX) */

#define	UDCCS_IO_RFS	(1 << 0)		/* Receive FIFO service (read only) */
#define	UDCCS_IO_RPC	(1 << 1)		/* Receive packet complete (read/write 1 to clear) */
#define	UDCCS_IO_ROF	(1 << 2)		/* Receive overflow (read/write 1 to clear) */
#define	UDCCS_IO_DME	(1 << 3)		/* DMA Enable (read/write) */

#define	UDCCS_IO_RSP	(1 << 7)		/* Receive short packet (read only) */


/* c.f. Table 12-26 UDC Endpoint X where x is 5, 10, 15 - INTERRUPT (UDCCSX) */

#define	UDCCS_INT_TFS	(1 << 0)		/* Transmit FIFO service (read only) */
#define	UDCCS_INT_TPC	(1 << 1)		/* Transmit packet complete (read/write 1 to clear) */
#define	UDCCS_INT_FTF	(1 << 2)		/* Flush Tx FIFO (always read 0/write 1 to set) */
#define	UDCCS_INT_TUR	(1 << 3)		/* Transmit FIFO underrun (read/write 1 to clear) */
#define	UDCCS_INT_SST	(1 << 4)		/* Sent STALL (read/write 1 to clear) */
#define	UDCCS_INT_FST	(1 << 5)		/* Force STALL (read/write) */

#define	UDCCS_INT_TSP	(1 << 7)		/* Transmit short packet (always read 0/write 1 to set) */


/* c.f. Table 12-27 UDC Interrupt Control Register 0 (UICR0) */

#define	UICR0_IM0	(1 << 0)		/* Interrupt mask for endpoint 0 */
#define	UICR0_IM1	(1 << 1)		/* Interrupt mask for endpoint 1 */
#define	UICR0_IM2	(1 << 2)		/* Interrupt mask for endpoint 2 */
#define	UICR0_IM3	(1 << 3)		/* Interrupt mask for endpoint 3 */
#define	UICR0_IM4	(1 << 4)		/* Interrupt mask for endpoint 4 */
#define	UICR0_IM5	(1 << 5)		/* Interrupt mask for endpoint 5 */
#define	UICR0_IM6	(1 << 6)		/* Interrupt mask for endpoint 6 */
#define	UICR0_IM7	(1 << 7)		/* Interrupt mask for endpoint 7 */


/* c.f. Table 12-28 UDC Interrupt Control Register 0 (UICR1) */

#define	UICR1_IM8	(1 << 0)		/* Interrupt mask for endpoint 8 */
#define	UICR1_IM9	(1 << 1)		/* Interrupt mask for endpoint 9 */
#define	UICR1_IM10	(1 << 2)		/* Interrupt mask for endpoint 10 */
#define	UICR1_IM11	(1 << 3)		/* Interrupt mask for endpoint 11 */
#define	UICR1_IM12	(1 << 4)		/* Interrupt mask for endpoint 12 */
#define	UICR1_IM13	(1 << 5)		/* Interrupt mask for endpoint 13 */
#define	UICR1_IM14	(1 << 6)		/* Interrupt mask for endpoint 14 */
#define	UICR1_IM15	(1 << 7)		/* Interrupt mask for endpoint 15 */


/* c.f. Table 12-29 UDC Status / Interrupt Register 0 (USIR0) */

#define	USIR0_IR0	(1 << 0)		/* Interrupt Request endpoint 0 (read/write 1 to clear) */
#define	USIR0_IR1	(1 << 1)		/* Interrupt Request endpoint 1 (read/write 1 to clear) */
#define	USIR0_IR2	(1 << 2)		/* Interrupt Request endpoint 2 (read/write 1 to clear) */
#define	USIR0_IR3	(1 << 3)		/* Interrupt Request endpoint 3 (read/write 1 to clear) */
#define	USIR0_IR4	(1 << 4)		/* Interrupt Request endpoint 4 (read/write 1 to clear) */
#define	USIR0_IR5	(1 << 5)		/* Interrupt Request endpoint 5 (read/write 1 to clear) */
#define	USIR0_IR6	(1 << 6)		/* Interrupt Request endpoint 6 (read/write 1 to clear) */
#define	USIR0_IR7	(1 << 7)		/* Interrupt Request endpoint 7 (read/write 1 to clear) */


/* c.f. Table 12-30 UDC Status / Interrupt Register 1 (USIR1) */

#define	USIR1_IR8	(1 << 0)		/* Interrupt Request endpoint 8  (read/write 1 to clear) */
#define	USIR1_IR9	(1 << 1)		/* Interrupt Request endpoint 9  (read/write 1 to clear) */
#define	USIR1_IR10	(1 << 2)		/* Interrupt Request endpoint 10 (read/write 1 to clear) */
#define	USIR1_IR11	(1 << 3)		/* Interrupt Request endpoint 11 (read/write 1 to clear) */
#define	USIR1_IR12	(1 << 4)		/* Interrupt Request endpoint 12 (read/write 1 to clear) */
#define	USIR1_IR13	(1 << 5)		/* Interrupt Request endpoint 13 (read/write 1 to clear) */
#define	USIR1_IR14	(1 << 6)		/* Interrupt Request endpoint 14 (read/write 1 to clear) */
#define	USIR1_IR15	(1 << 7)		/* Interrupt Request endpoint 15 (read/write 1 to clear) */



/* c.f. Table 12-31 UDC Frame Number High Register (UFNRH)*/

#define	UFNHR_FNMSB	(7 << 0)		/* Most significant 3-bits of 11-bit frame number */

#define	UFNHR_IPE4	(1 << 3)		/* Iso Packet Error Endpont 4 (read/write 1 to clear) */
#define	UFNHR_IPE9	(1 << 4)		/* Iso Packet Error Endpont 9 (read/write 1 to clear) */
#define	UFNHR_IPE14	(1 << 5)		/* Iso Packet Error Endpont 14 (read/write 1 to clear) */
#define	UFNHR_SIM	(1 << 6)		/* SOF interrupt mask (read/write) */
#define	UFNHR_SIR	(1 << 7)		/* SOF Interrupt Request (read/write 1 to clear) */



