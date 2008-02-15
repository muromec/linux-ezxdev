/*
 * linux/drivers/usb/device/bi/wmmx.h
 *
 * Based on work:
 * Copyright (c) 2000, 2001, 2002 Lineo
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *
 * Changes copyright (c) 2003 MontaVista Software, Inc.
 * Author: MontaVista Software, Inc., source@mvista.com
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

#define UDC_MAX_ENDPOINTS       24

#define MAX_LOGICAL_ENDPOINTS   16
#define MAX_LOGICAL_CONFIGURATIONS 4

#define UDC_NAME        "WMMX"

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

	pxa_dma_desc *dma_desc;
	dma_addr_t dma_desc_phys;
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

	pxa_dma_desc *dma_desc;
	dma_addr_t dma_desc_phys;

	int num;
	int size;

        int ep;
        struct usb_endpoint_instance *endpoint;
} usb_stream_t;

/* UDC endpiont0 states */
#define EP0_WAIT_FOR_SETUP          0
#define EP0_DATA_STATE_XMIT         1
#define EP0_DATA_STATE_NEED_ZLP     2
#define EP0_WAIT_FOR_OUT_STATUS     3

/* Endpoint 0 state definitions */
#define WAIT_FOR_SETUP          0
#define DATA_STATE_XMIT         1
#define DATA_STATE_RECV         2
#define DATA_STATE_PENDING_XMIT 3
#define DATA_STATE_NEED_ZLP     4
#define WAIT_FOR_OUT_STATUS     5

