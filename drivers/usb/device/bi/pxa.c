/*
 * linux/drivers/usb/device/bi/pxa.c -- Intel PXA250 USB Device Controller driver. 
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2002, 2003 RTSoft
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *
 * Minor fixes and basic DMA support by:
 *      Dmitry Antipov <antipov@rtsoft.msk.ru>
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

/*
 * Testing notes:
 *
 * This code was developed on the Intel Lubbock with Cotulla PXA250 processor.
 * Note DMA OUT seems works starting from C0 chip revision only.
 */

/*****************************************************************************/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/netdevice.h>
#include <linux/delay.h>

#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/types.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/dma.h>

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-module.h"
#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-inline.h"
#include "usbd-bi.h"

#include "pxa.h"

MODULE_AUTHOR ("sl@lineo.com, tbr@lineo.com, antipov@rtsoft.msk.ru");
MODULE_DESCRIPTION ("Xscale USB Device Bus Interface");
MODULE_LICENSE ("GPL");
USBD_MODULE_INFO ("pxa_bi 0.1-alpha");

#ifdef CONFIG_USBD_PXA_DMA_IN
#if defined(CONFIG_USBD_SERIAL_IN_PKTSIZE)
#define DMA_IN_BUFSIZE CONFIG_USBD_SERIAL_IN_PKTSIZE
#elif defined(CONFIG_USBD_NET_IN_PKTSIZE)
#define DMA_IN_BUFSIZE CONFIG_USBD_NET_IN_PKTSIZE
#else
#error "Can't determine buffer size for DMA IN"
#endif

#if defined(CONFIG_USBD_SERIAL_IN_ENDPOINT)
#define IN_ENDPOINT CONFIG_USBD_SERIAL_IN_ENDPOINT
#elif defined(CONFIG_USBD_NET_IN_ENDPOINT)
#define IN_ENDPOINT CONFIG_USBD_NET_IN_ENDPOINT
#else
#error "Can't determine IN endpoint"
#endif
#endif /* CONFIG_USBD_PXA_DMA_IN */

#ifdef CONFIG_USBD_PXA_DMA_OUT
#if defined(CONFIG_USBD_SERIAL_OUT_PKTSIZE)
#define DMA_OUT_BUFSIZE CONFIG_USBD_SERIAL_OUT_PKTSIZE
#elif defined(CONFIG_USBD_NET_OUT_PKTSIZE)
#define DMA_OUT_BUFSIZE CONFIG_USBD_NET_OUT_PKTSIZE
#else
#error "Can't determine buffer size for DMA OUT"
#endif

#if defined(CONFIG_USBD_SERIAL_OUT_ENDPOINT)
#define OUT_ENDPOINT CONFIG_USBD_SERIAL_OUT_ENDPOINT
#elif defined(CONFIG_USBD_NET_OUT_ENDPOINT)
#define OUT_ENDPOINT CONFIG_USBD_NET_IN_ENDPOINT
#else
#error "Can't determine OUT endpoint"
#endif
#endif /* CONFIG_USBD_PXA_DMA_OUT */

#if defined(CONFIG_USBD_PXA_DMA_IN) || defined(CONFIG_USBD_PXA_DMA_OUT)
/* #define DMA_TRACEMSG(fmt, args...) printk (fmt, ## args) */
#define DMA_TRACEMSG(fmt, args...) 

/* This datatype contains all information required for DMA transfers */
typedef struct usb_pxa_dma {
        int dmach;
        struct usb_endpoint_instance *endpoint;
        int size;
        int ep;
        unsigned char *dma_buf;
        dma_addr_t dma_buf_phys;
} usb_pxa_dma_t;
#endif

unsigned int int_oscr;

#undef PXA_TRACE

#ifdef PXA_TRACE
typedef enum pxa_trace_type {
        pxa_regs, pxa_setup, pxa_xmit, pxa_ccr, pxa_iro
} pxa_trace_type_t;


typedef struct pxa_regs {
        u32     cs0;
        char *  msg;
} pxa_regs_t;

typedef struct pxa_xmit {
        u32     size;
} pxa_xmit_t;

typedef struct pxa_ccr {
        u32     ccr;
        char *  msg;
} pxa_ccr_t;

typedef struct pxa_iro {
        u32     iro;
        char *  msg;
} pxa_iro_t;

typedef struct pxa_trace {
        pxa_trace_type_t        trace_type;
        u32     interrupts;
        u32     ocsr;
        u64     jiffies;
        union {
                pxa_regs_t       regs;
                pxa_xmit_t       xmit;
                pxa_ccr_t        ccr;
                pxa_iro_t        iro;
                struct usb_device_request       setup;
        } trace;
} pxa_trace_t;


#define TRACE_MAX       10000

int trace_next;
pxa_trace_t *pxa_traces;

static __inline__ void PXA_REGS(u32 cs0, char *msg)
{
        if ((trace_next < TRACE_MAX) && pxa_traces) {
                pxa_trace_t *p = pxa_traces + trace_next++;
                p->ocsr = OSCR;
                p->jiffies = jiffies;
                p->interrupts = udc_interrupts;
                p->trace_type = pxa_regs;
                p->trace.regs.cs0 = cs0;
                p->trace.regs.msg = msg;
        }
}

static __inline__ void PXA_SETUP(struct usb_device_request *setup)
{
        if ((trace_next < TRACE_MAX) && pxa_traces) {
                pxa_trace_t *p = pxa_traces + trace_next++;
                p->ocsr = OSCR;
                p->jiffies = jiffies;
                p->interrupts = udc_interrupts;
                p->trace_type = pxa_setup;
                memcpy(&p->trace.setup, setup, sizeof(struct usb_device_request));
        }
}

static __inline__ void PXA_XMIT(u32 size)
{
        if ((trace_next < TRACE_MAX) && pxa_traces) {
                pxa_trace_t *p = pxa_traces + trace_next++;
                p->ocsr = OSCR;
                p->jiffies = jiffies;
                p->interrupts = udc_interrupts;
                p->trace_type = pxa_xmit;
                p->trace.xmit.size = size;
        }
}

static __inline__ void PXA_CCR(u32 ccr, char *msg)
{
        if ((trace_next < TRACE_MAX) && pxa_traces) {
                pxa_trace_t *p = pxa_traces + trace_next++;
                p->ocsr = OSCR;
                p->jiffies = jiffies;
                p->interrupts = udc_interrupts;
                p->trace_type = pxa_ccr;
                p->trace.ccr.ccr = ccr;
                p->trace.ccr.msg = msg;
        }
}

static __inline__ void PXA_IRO(u32 iro, char *msg)
{
        if ((trace_next < TRACE_MAX) && pxa_traces) {
                pxa_trace_t *p = pxa_traces + trace_next++;
                p->ocsr = OSCR;
                p->jiffies = jiffies;
                p->interrupts = udc_interrupts;
                p->trace_type = pxa_iro;
                p->trace.iro.iro = iro;
                p->trace.iro.msg = msg;
        }
}
#else
static __inline__ void PXA_REGS(u32 cs0, char *msg)
{
}

static __inline__ void PXA_SETUP(struct usb_device_request *setup)
{
}

static __inline__ void PXA_XMIT(u32 size)
{
}

static __inline__ void PXA_CCR(u32 ccr, char *msg)
{
}

static __inline__ void PXA_IRO(u32 iro, char *msg)
{
}
#endif

static int udc_suspended;
static struct usb_device_instance *udc_device;	// required for the interrupt handler

/*
 * ep_endpoints - map physical endpoints to logical endpoints
 */
static struct usb_endpoint_instance *ep_endpoints[UDC_MAX_ENDPOINTS];

static struct urb *ep0_urb;

extern unsigned int udc_interrupts;

#if 0
int jifs(void)
{
        static unsigned long jiffies_last;
        int elapsed = jiffies - jiffies_last;

        jiffies_last = jiffies;
        return elapsed;
}
#else
int jifs(void)
{
        static unsigned long jiffies_last;
        int elapsed = OSCR - jiffies_last;

        jiffies_last = OSCR;
        return elapsed;
}
#endif

/* ********************************************************************************************* */
/* IO
 */

/*
 * Map logical to physical
 */

typedef enum ep {
        ep_control, ep_bulk_in, ep_bulk_out, ep_iso_in, ep_iso_out, ep_interrupt
} ep_t;

/*
 * PXA has lots of endpoints, but they have fixed address and type
 * so logical to physical map is limited to masking top bits so we
 * can find appropriate info.
 */


u32 _UDDRN[16] = {
        0x40600080, 0x40600100, 0x40600180, 0x40600200,
        0x40600400, 0x406000A0, 0x40600600, 0x40600680,
        0x40600700, 0x40600900, 0x406000C0, 0x40600B00, 
        0x40600B80, 0x40600C00, 0x40600E00, 0x406000E0, 
};

u32 _UBCRN[16] = {
        0,          0,          0x40600068, 0,
        0x4060006c, 0,          0,          0x40600070,
        0,          0x40600074, 0,          0,
        0x40600078, 0,          0x4060007c, 0, 
};

#define UDCCSN(x)        __REG2(0x40600010, (x) << 2)
#define UDDRN(x)        __REG(_UDDRN[x])
#define UBCRN(x)        __REG(_UBCRN[x])

struct ep_map {
        int             logical;
        ep_t            eptype;
        int             size;
        int             dma_chan;
};

static struct ep_map ep_maps[16] = {
        { logical: 0, eptype: ep_control,   size: 16, },
        { logical: 1, eptype: ep_bulk_in,   size: 64, },

        { logical: 2, eptype: ep_bulk_out,  size: 64, },

        { logical: 3, eptype: ep_iso_in,   size: 256, },
        { logical: 4, eptype: ep_iso_out,  size: 256, },

        { logical: 5, eptype: ep_interrupt , size: 8, },

        { logical: 6, eptype: ep_bulk_in,   size: 64, },
        { logical: 7, eptype: ep_bulk_out,  size: 64, },
                                                      
        { logical: 8, eptype: ep_iso_in,   size: 256, },
        { logical: 9, eptype: ep_iso_out,  size: 256, },
                                                      
        { logical: 10, eptype: ep_interrupt, size: 8, },
                                                      
        { logical: 11, eptype: ep_bulk_in,  size: 64, },
        { logical: 12, eptype: ep_bulk_out, size: 64, },
                                                      
        { logical: 13, eptype: ep_iso_in,  size: 256, },
        { logical: 14, eptype: ep_iso_out, size: 256, },
                                                      
        { logical: 15, eptype: ep_interrupt, size: 8, },
};

static __inline__ void pxa_enable_ep_interrupt(int ep)
{
        ep &= 0xf;
#ifdef CONFIG_USBD_PXA_DMA_OUT
	if (ep == CONFIG_USBD_NET_OUT_ENDPOINT) {
		DMA_TRACEMSG ("PXA: skip enabling EP%d due to DMA\n", CONFIG_USBD_NET_OUT_ENDPOINT);
		return;
	}
#endif
        if (ep < 8) {
                UICR0 &= ~(1<<ep);
        }
        else {
                UICR1 &= ~(1<<(ep-8));
        }
}

static __inline__ void pxa_disable_ep_interrupt(int ep)
{
        ep &= 0xf;
        if (ep < 8) {
                UICR0 = 1<<ep;
        }
        else {
                UICR1 = 1<<(ep-8);
        }
}

static __inline__ void pxa_ep_reset_irs(int ep)
{
        ep &= 0xf;

        if (ep < 8) {
                USIR0 = (1 << ep);
        }
        else {
                USIR1 = (1 << (ep - 8));
        }
}

/* ********************************************************************************************* */
/* Bulk OUT (recv)
 */

static void pxa_out_flush(int ep) {
        unsigned char c;
        while (UDCCSN(ep) & UDCCS_BO_RNE) {
                c = UDDRN(ep);
        }
}

#ifdef CONFIG_USBD_PXA_DMA_OUT

/* 
 * FCS stuff, taken from usbdnet.c 
 */

static __u32 crc32_table[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

#define CRC32_INITFCS     0xffffffff  // Initial FCS value 
#define CRC32_GOODFCS     0xdebb20e3  // Good final FCS value 

#define CRC32_FCS(fcs, c) (((fcs) >> 8) ^ crc32_table[((fcs) ^ (c)) & 0xff])

/* fcs_compute32 - memcpy and calculate fcs */
static __u32 __inline__ fcs_compute32(unsigned char *sp, int len, __u32 fcs)
{
	for (;len-- > 0; fcs = CRC32_FCS(fcs, *sp++));
	return fcs;
}

static usb_pxa_dma_t usb_pxa_dma_out = {-1, NULL, 0, 0, NULL, 0};

/* Do some h/w setup for DMA OUT */
static inline void pxa_rx_dma_hw_init (void)
{
	DCSR(usb_pxa_dma_out.dmach) |= DCSR_NODESC;
	DCMD(usb_pxa_dma_out.dmach) = DCMD_FLOWSRC | DCMD_INCTRGADDR | DCMD_WIDTH1 | 
		DCMD_BURST32 | DCMD_ENDIRQEN | DMA_OUT_BUFSIZE;
	DSADR(usb_pxa_dma_out.dmach) = _UDDRN[usb_pxa_dma_out.ep];
	DTADR(usb_pxa_dma_out.dmach) = usb_pxa_dma_out.dma_buf_phys;
	DRCMR26 = usb_pxa_dma_out.dmach | DRCMR_MAPVLD;
}

static void pxa_rx_dma_irq (int dmach, void *dev_id, struct pt_regs *regs)
{
	int dcsr = DCSR(usb_pxa_dma_out.dmach);
	
	DMA_TRACEMSG ("PXA: out dma IRQ [%d %d], DCSR 0x%x\n", UBCRN(usb_pxa_dma_out.ep), 
		      ep_endpoints[usb_pxa_dma_out.ep]->rcv_packetSize, dcsr);
	/* Stop DMA */
	DCSR(usb_pxa_dma_out.dmach) &= ~DCSR_RUN;
	/* TODO: busy-wait until DMA is really stopped ? */

	if (dcsr & DCSR_BUSERR)
		/* Is there a way to handle this error somehow ? */
                printk (KERN_WARNING "PXA: Bus error in USB RX DMA "
                        "- USB data transfer may be incorrect\n");
        
        else if (dcsr & DCSR_ENDINTR) {
		DMA_TRACEMSG ("PXA: normal IRQ\n");
		if (!usb_pxa_dma_out.endpoint->rcv_urb) {
			usb_pxa_dma_out.endpoint->rcv_urb = first_urb_detached (&usb_pxa_dma_out.endpoint->rdy);
		}
		if (usb_pxa_dma_out.endpoint->rcv_urb) {
			int recvsize;
			unsigned char *cp = usb_pxa_dma_out.endpoint->rcv_urb->buffer + 
				usb_pxa_dma_out.endpoint->rcv_urb->actual_length;
			
			/* Copy all new data to current urb buffer */
			memcpy (cp, usb_pxa_dma_out.dma_buf, DMA_OUT_BUFSIZE);
			
			if (usb_pxa_dma_out.dma_buf[DMA_OUT_BUFSIZE - 1] == 0xff) {
				/* May be end-of-urb data - must calculate FCS and
				   compare with FCS calculated on the host side */
				__u32 oldfcs, newfcs;

				/* See usbdnet.c for explanation of this */
				oldfcs = (usb_pxa_dma_out.dma_buf[DMA_OUT_BUFSIZE - 2] << 24) |
					 (usb_pxa_dma_out.dma_buf[DMA_OUT_BUFSIZE - 3] << 16) |
					 (usb_pxa_dma_out.dma_buf[DMA_OUT_BUFSIZE - 4] << 8) |
					 (usb_pxa_dma_out.dma_buf[DMA_OUT_BUFSIZE - 5]);
				newfcs = fcs_compute32
					 (usb_pxa_dma_out.endpoint->rcv_urb->buffer,
					  usb_pxa_dma_out.endpoint->rcv_urb->actual_length + DMA_OUT_BUFSIZE - 5,
					  CRC32_INITFCS);
				newfcs = ~newfcs;
				/* If FCSs are equal, the last byte of new data
				   is 0xff mark and will be ignored */
				recvsize = (oldfcs == newfcs) ? (DMA_OUT_BUFSIZE - 1) : DMA_OUT_BUFSIZE;
			}
			else
				recvsize = DMA_OUT_BUFSIZE;
			DMA_TRACEMSG ("PXA: urb exists, real size %d\n", recvsize);
			usbd_rcv_complete_irq (usb_pxa_dma_out.endpoint, recvsize, 0);
		}
	}

	/* clear RPC and interrupt */
        UDCCSN(usb_pxa_dma_out.ep) |= UDCCS_BO_RPC;
        pxa_ep_reset_irs(usb_pxa_dma_out.ep);

	/* Clear end-of-interrupt */
        DCSR(usb_pxa_dma_out.dmach) |= DCSR_ENDINTR;

	/* Reinitialize and restart DMA */
	pxa_rx_dma_hw_init ();
	DCSR(usb_pxa_dma_out.dmach) |= DCSR_RUN;
}
#endif /* CONFIG_USBD_PXA_DMA_OUT */

static __inline__ void pxa_out_n(int ep, struct usb_endpoint_instance *endpoint)
{
        if (UDCCSN(ep) & UDCCS_BO_RPC) {

                if (endpoint) {
                        if (!endpoint->rcv_urb) {
                                endpoint->rcv_urb = first_urb_detached (&endpoint->rdy);
                        }
                        if (endpoint->rcv_urb) {

                                int len = 0;
                                unsigned char *cp = endpoint->rcv_urb->buffer + endpoint->rcv_urb->actual_length;

                                if (cp) {
                                        // read available bytes (max packetsize) into urb buffer
                                        int count = MIN(UBCRN(ep) + 1, endpoint->rcv_packetSize);
                                        while (count--) {
                                                len++;
                                                *cp++ = UDDRN(ep);
                                        }
                                }
                                else {
                                        printk(KERN_INFO"read[%d:%d] bad arguements\n", udc_interrupts, jifs());
                                        pxa_out_flush(ep);
                                }
                                // fall through if error of any type, len = 0
                                usbd_rcv_complete_irq (endpoint, len, 0);
                        }
                        else {
                                pxa_out_flush(ep);
                        }
                }
        }
        // clear RPC and interrupt
        UDCCSN(ep) = UDCCS_BO_RPC;
        pxa_ep_reset_irs(ep);
}

/* ********************************************************************************************* */
/* Bulk IN (tx)
 */

#ifdef CONFIG_USBD_PXA_DMA_IN
static usb_pxa_dma_t usb_pxa_dma_in = {-1, NULL, 0, 0, NULL, 0};

static void pxa_tx_dma_irq (int dmach, void *dev_id, struct pt_regs *regs)
{
	int dcsr = DCSR(usb_pxa_dma_in.dmach);

	/* Stop DMA */
	DCSR(usb_pxa_dma_in.dmach) &= ~DCSR_RUN;

	DMA_TRACEMSG ("PXA: in dma IRQ, DCSR 0x%x\n", dcsr);
        if (dcsr & DCSR_BUSERR)
		/* I don't know better way to handle DMA bus error... */
                printk (KERN_WARNING "PXA: Bus error in USB TX DMA "
                        "- USB data transfer may be incorrect\n");
        
        else if (dcsr & DCSR_ENDINTR) {
                /* All seems ok, checking for we have short packet transmitted */
                if ((usb_pxa_dma_in.size < usb_pxa_dma_in.endpoint->tx_packetSize) ||
                    ((usb_pxa_dma_in.endpoint->tx_urb->actual_length 
                      - usb_pxa_dma_in.endpoint->sent ) == usb_pxa_dma_in.size)) {
                        UDCCSN(usb_pxa_dma_in.ep) |= UDCCS_BI_TSP;
		}
                usb_pxa_dma_in.endpoint->last += usb_pxa_dma_in.size;
        }

        /* Clear end-of-interrupt */
        DCSR(usb_pxa_dma_in.dmach) |= DCSR_ENDINTR;

        /* Now we can re-enable irq */
        pxa_enable_ep_interrupt (usb_pxa_dma_in.ep);
}

static void __inline__ pxa_start_n_dma_hw (int ep, struct usb_endpoint_instance *endpoint,
					   char *mem, int size)
{
	/* Prepare for DMA */
	memcpy (usb_pxa_dma_in.dma_buf, mem, size);
	usb_pxa_dma_in.ep = ep;
	usb_pxa_dma_in.endpoint = endpoint;
	usb_pxa_dma_in.size = size;
	
	/* Setup DMA */
	DCSR(usb_pxa_dma_in.dmach) |= DCSR_NODESC;
	DCMD(usb_pxa_dma_in.dmach) = DCMD_FLOWTRG | DCMD_INCSRCADDR | DCMD_WIDTH1 | 
		DCMD_BURST32 | DCMD_ENDIRQEN | size;
	DSADR(usb_pxa_dma_in.dmach) = usb_pxa_dma_in.dma_buf_phys;
	DTADR(usb_pxa_dma_in.dmach) = _UDDRN[ep];

	/* Map DMA (FIXME: assume ep is 1) */
	DRCMR25 = usb_pxa_dma_in.dmach | DRCMR_MAPVLD;
	
	/* Start DMA */
	DCSR(usb_pxa_dma_in.dmach) |= DCSR_RUN;
}

static void pxa_start_n_dma (unsigned int ep, struct usb_endpoint_instance *endpoint)
{
	/* We must disable irq for this endpoint */
	pxa_disable_ep_interrupt (ep);

        if (endpoint->tx_urb) {
                struct urb *urb = endpoint->tx_urb;
                int last = MIN(urb->actual_length - (endpoint->sent + endpoint->last), 
			       endpoint->tx_packetSize);
	        if (last) {
			pxa_start_n_dma_hw (ep, endpoint, urb->buffer + endpoint->sent + 
					    endpoint->last, last);
			return;
		}
	}
        
        /* Do nothing with DMA - re-enable irq */
        pxa_enable_ep_interrupt (ep);
}
#endif /* CONFIG_USBD_PXA_DMA_IN */

static void __inline__ pxa_start_n (unsigned int ep, struct usb_endpoint_instance *endpoint)
{
        if (endpoint->tx_urb) {
                int last;
                struct urb *urb = endpoint->tx_urb;

                if (( last = MIN (urb->actual_length - (endpoint->sent + endpoint->last), endpoint->tx_packetSize))) 
                {
                        int size = last;
                        unsigned char *cp = urb->buffer + endpoint->sent + endpoint->last;

                        while (size--) {
                                UDDRN(ep) = *cp++;
                        }

                        if (( last < endpoint->tx_packetSize ) ||
                                        ( (endpoint->tx_urb->actual_length - endpoint->sent ) == last )) 
                        {
                                UDCCSN(ep) = UDCCS_BI_TSP;
                        }
                        endpoint->last += last;
                }
        }
}

static void __inline__ pxa_in_n (unsigned int ep, struct usb_endpoint_instance *endpoint)
{
        int udccsn;

        pxa_ep_reset_irs(ep);

        // if TPC update tx urb and clear TPC
        if ((udccsn = UDCCSN(ep)) & UDCCS_BI_TPC) {

                UDCCSN(ep) = UDCCS_BI_TPC;
                usbd_tx_complete_irq(endpoint, 0);
        }

        if (udccsn & UDCCS_BI_TFS) {
                pxa_start_n(ep, endpoint);
        }

        // clear underrun, not much we can do about it
        if (udccsn & UDCCS_BI_TUR) {
                UDCCSN(ep) = UDCCS_BI_TUR;
        }
}

/* ********************************************************************************************* */
/* Control (endpoint zero)
 */

#ifdef CONFIG_USBD_EP0_SUPPORT
/* Changed by Stanley */
void pxa_ep0recv(struct usb_endpoint_instance *endpoint, volatile u32 udccs0)
{
	int size=0;
	struct urb *urb = endpoint->rcv_urb;
	unsigned char* cp = (unsigned char*) (urb->buffer + urb->actual_length);
	
	PXA_REGS(udccs0, "         <-- setup data recv");

#ifdef EP0_DEBUG
	printk("%d %x %x\n", urb->actual_length, urb->buffer, cp );
	printk("udccs0=0x%x\n", udccs0 );
#endif

	// check for premature status stage
	if ( !(udccs0&UDCCS0_OPR) && !(udccs0&UDCCS0_IPR) ) {
		if ( urb->device_request.wLength == urb->actual_length ) {
#ifdef EP0_DEBUG
			printk(KERN_INFO"pxa_ep0recv: UDC ep0 receive all data.\n");
#endif
			usbd_recv_setup(ep0_urb);
		} else {
#ifdef EP0_DEBUG
			printk(KERN_INFO"pxa_ep0recv: Get a premature status in stage.\n");
			printk(KERN_INFO"pxa_ep0recv: request.wLength=0x%x, actual_length=0x%x\n", 
			       urb->device_request.wLength, urb->actual_length );
#endif
			if ( urb->buffer )
				kfree( urb->buffer );
		}

		endpoint->state = WAIT_FOR_SETUP;
	}

	// receive more data
	if (( udccs0 & UDCCS0_OPR ) && !(UDCCS0 & UDCCS0_SA) ) {
		while ( UDCCS0&UDCCS0_RNE ) {
			*cp++ = UDDRN(0);
			urb->actual_length++;
		}

		/* Do we need this section? */
		/* need more tests!  -Stanley */
		/* Without the section, the driver still works */
#if 1
		if ( urb->actual_length == urb->device_request.wLength ) {
#ifdef EP0_DEBUG
			printk(KERN_INFO"pxa_ep0recv under test: UDC ep0 receive all data.\n");
#endif
			usbd_recv_setup(ep0_urb);
			endpoint->state = WAIT_FOR_SETUP;
		}
#endif 
		UDCCS0 |= UDCCS0_OPR;
		
		// to allow to enter a premature STATUS IN stage
		UDCCS0 |= UDCCS0_IPR;
	}

	return;
}
#endif 

void pxa_ep0xmit(struct usb_endpoint_instance *endpoint, volatile u32 udccs0)
{
	int short_packet;
	int size;
	struct urb *urb = endpoint->tx_urb;

        PXA_REGS(udccs0, "          --> xmit");

        //printk(KERN_INFO"tx[%d:%d] CS0[%02x]\n", udc_interrupts, jifs(), UDCCS0);

        // check for premature status stage - host abandoned previous IN
        if ((udccs0 & UDCCS0_OPR) && !(UDCCS0 & UDCCS0_SA) ) {

                // clear tx fifo and opr
                UDCCS0 = UDCCS0_FTF | UDCCS0_OPR;
                endpoint->state = WAIT_FOR_SETUP;
                endpoint->tx_urb = NULL;
                PXA_REGS(UDCCS0, "          <-- xmit premature status");
                return;
        }

        // check for stall
        if (udccs0 & UDCCS0_SST) {
                // clear stall and tx fifo
                UDCCS0 = UDCCS0_SST | UDCCS0_FTF;
                endpoint->state = WAIT_FOR_SETUP;
                endpoint->tx_urb = NULL;
                PXA_REGS(UDCCS0, "          <-- xmit stall");
                return;
        }

	/* How much are we sending this time? (May be zero!)
           (Note that later call of tx_complete() will add last to sent.) */
	if (NULL == urb) {
		size = 0;
	} else {
		endpoint->last = size = MIN (urb->actual_length - endpoint->sent, endpoint->tx_packetSize);
	}

	/* Will this be a short packet?
          (It may be the last, but still be full size, in which case we will need a ZLP later.) */
        short_packet = (size < endpoint->tx_packetSize);

        PXA_XMIT(size);

	if (size > 0 && urb->buffer) {
		// Stuff the FIFO
                unsigned char *cp = urb->buffer + endpoint->sent;

		while (size--) {
			UDDRN(0) = *cp++;
		}
        }

        // Is this the end of the data state? (We've sent all the data, plus any required ZLP.)
        if (!endpoint->tx_urb || (endpoint->last < endpoint->tx_packetSize)) {
		// Tell the UDC we are at the end of the packet.
		UDCCS0 = UDCCS0_IPR;
                endpoint->state = WAIT_FOR_OUT_STATUS;
                PXA_REGS(UDCCS0, "          <-- xmit wait for status");
	}

        else if ((endpoint->last == endpoint->tx_packetSize) && 
                        ((endpoint->last + endpoint->sent) == ep0_urb->actual_length)  &&
                        ((ep0_urb->actual_length) < le16_to_cpu(ep0_urb->device_request.wLength)) 
                ) 
        {
		// Tell the UDC we are at the end of the packet.
                endpoint->state = DATA_STATE_NEED_ZLP;
                PXA_REGS(UDCCS0, "          <-- xmit need zlp");
	}
        else {
                PXA_REGS(UDCCS0, "          <-- xmit not finished");
        }
}

void __inline__ pxa_ep0setup(struct usb_endpoint_instance *endpoint, volatile u32 udccs0)
{
        if ((udccs0 & (UDCCS0_SA | UDCCS0_OPR | UDCCS0_RNE)) == (UDCCS0_SA | UDCCS0_OPR | UDCCS0_RNE)) {

                int len = 0;
                int max = 8;
                unsigned char *cp = (unsigned char *)&ep0_urb->device_request;

                PXA_REGS(udccs0, "      --> setup");

                //memset(cp, 0, max);

                while (max-- /*&& (UDCCS0 & UDCCS0_RNE)*/) {
                        len++;
                        *cp++ = UDDR0;
                }

                PXA_SETUP(&ep0_urb->device_request);

#ifdef CONFIG_USBD_EP0_SUPPORT
/* This section is added to give some supports for setup data IN mode. It is necessary 
 * to call usbd_recv_setup * routine at end of receiving all data
 *                           - Stanley
 */
#ifdef EP0_DEBUG
		printk( KERN_INFO"URB request:\n" );
		printk( KERN_INFO"ep0.bmRequestType: 0x%x\n", ep0_urb->device_request.bmRequestType );
		printk( KERN_INFO"ep0.bRequest: 0x%x\n", ep0_urb->device_request.bRequest );
		printk( KERN_INFO"ep0.wlength: 0x%x\n", ep0_urb->device_request.wLength );
#endif

		// Controll Write - Maybe we will receive more data from the host
		if ((ep0_urb->device_request.bmRequestType & USB_REQ_DIRECTION_MASK) == USB_REQ_HOST2DEVICE
			&& le16_to_cpu(ep0_urb->device_request.wLength)!=0 ) 
		{
#ifdef EP0_DEBUG
			printk("HOST2DEVICE\n");
#endif
			endpoint->state = DATA_STATE_RECV;

			// allocate new memory for such a request;
			ep0_urb->buffer = kmalloc( ep0_urb->device_request.wLength, GFP_ATOMIC );
			if ( !ep0_urb->buffer ) {
				panic("pxa_ep0setup: Out of memory!\n");
				return;
			}
			
			ep0_urb->buffer_length = ep0_urb->device_request.wLength;
			ep0_urb->actual_length = 0;
			// the bugs
			endpoint->rcv_urb = ep0_urb;
                        UDCCS0 = UDCCS0_SA | UDCCS0_OPR;
                        PXA_REGS(UDCCS0,"      <-- setup recv");

			return;
		}
#endif

                // process setup packet
                if (usbd_recv_setup(ep0_urb)) {
                        // setup processing failed 
                        UDCCS0 = UDCCS0_FST;
                        endpoint->state = WAIT_FOR_SETUP;
                        PXA_REGS(udccs0, "      --> bad setup FST");
                        return;
                }

                // check direction
                if ((ep0_urb->device_request.bmRequestType & USB_REQ_DIRECTION_MASK) == USB_REQ_HOST2DEVICE) 
                {
                        // Control Write - we are receiving more data from the host
#ifdef EP0_DEBUG
			printk("HOST2DEVICE - zero length\n");
#endif
                        // should we setup to receive data
			// Skip this condition; - Stanley
                        if (le16_to_cpu (ep0_urb->device_request.wLength)) {
                                //printk(KERN_INFO"sl11_ep0: read data %d\n",
                                //      le16_to_cpu(ep0_urb->device_request.wLength));
                                endpoint->rcv_urb = ep0_urb;
                                endpoint->rcv_urb->actual_length = 0;
				endpoint->state = DATA_STATE_RECV;
                                // XXX this has not been tested
                                // pxa_out_0 (0, endpoint);
                                PXA_REGS(UDCCS0,"      <-- setup no response");
                                return;
                        }

                        // allow for premature IN (c.f. 12.5.3 #8)
                        UDCCS0 = UDCCS0_IPR;
                        PXA_REGS(UDCCS0,"      <-- setup nodata");
                }
                else {
                        // Control Read - we are sending data to the host
#ifdef EP0_DEBUG
			printk("DEVICE2HOST\n");
#endif
                        // verify that we have non-zero request length
                        if (!le16_to_cpu (ep0_urb->device_request.wLength)) {
                                udc_stall_ep (0);
                                PXA_REGS(UDCCS0,"      <-- setup stalling zero wLength");
                                return;
                        }
                        // verify that we have non-zero length response
                        if (!ep0_urb->actual_length) {
                                udc_stall_ep (0);
                                PXA_REGS(UDCCS0,"      <-- setup stalling zero response");
                                return;
                        }

                        // start sending
                        endpoint->tx_urb = ep0_urb;
                        endpoint->sent = 0;
                        endpoint->last = 0;
                        endpoint->state = DATA_STATE_XMIT;
			pxa_ep0xmit(endpoint, UDCCS0);

                        // Clear SA and OPR bits
                        UDCCS0 = UDCCS0_SA | UDCCS0_OPR;
                        PXA_REGS(UDCCS0,"      <-- setup data");
                }
        }
}

static __inline__ void pxa_ep0(struct usb_endpoint_instance *endpoint, volatile u32 udccs0)
{
        int j = 0;

        PXA_REGS(udccs0,"  --> ep0");

        if (udccs0 & UDCCS0_SST) {
                pxa_ep_reset_irs(0);
                UDCCS0 = UDCCS0_SST;
                PXA_REGS(udccs0,"  --> ep0 clear SST");
                if (endpoint) {
                        endpoint->state = WAIT_FOR_SETUP;
                        endpoint->tx_urb = NULL;
                }
                return;
        }


        if (!endpoint) {
                printk(KERN_INFO"ep0[%d:%d] endpoint Zero is NULL CS0[%02x]\n", udc_interrupts, jifs(), UDCCS0);
                pxa_ep_reset_irs(0);
                UDCCS0 = UDCCS0_IPR | UDCCS0_OPR | UDCCS0_SA;
                PXA_REGS(UDCCS0,"  ep0 NULL");
                return;
        }

        if (endpoint->tx_urb) {
                usbd_tx_complete_irq (endpoint, 0);
                if (!endpoint->tx_urb) {
                        if (endpoint->state != DATA_STATE_NEED_ZLP) {
                                endpoint->state = WAIT_FOR_OUT_STATUS;
                        }
                }
        }

        if ((endpoint->state != WAIT_FOR_SETUP) && (udccs0 & UDCCS0_SA)) {
                PXA_REGS(udccs0,"  --> ep0 early SA");
                endpoint->state = WAIT_FOR_SETUP;
                endpoint->tx_urb = NULL;
        }

        switch (endpoint->state) {
        case DATA_STATE_NEED_ZLP:
                UDCCS0 = UDCCS0_IPR;
                endpoint->state = WAIT_FOR_OUT_STATUS;
                break;

        case WAIT_FOR_OUT_STATUS:
                if ((udccs0 & (UDCCS0_OPR | UDCCS0_SA)) == UDCCS0_OPR) {
                        UDCCS0 |= UDCCS0_OPR;
                }
                PXA_REGS(UDCCS0,"  --> ep0 WAIT for STATUS");
                endpoint->state = WAIT_FOR_SETUP;

        case WAIT_FOR_SETUP:
                do {
                        pxa_ep0setup(endpoint, UDCCS0);

                        if (udccs0 & UDCCS0_SST) {
                                pxa_ep_reset_irs(0);
                                UDCCS0 = UDCCS0_SST | UDCCS0_OPR | UDCCS0_SA;
                                PXA_REGS(udccs0,"  --> ep0 clear SST");
                                if (endpoint) {
                                        endpoint->state = WAIT_FOR_SETUP;
                                        endpoint->tx_urb = NULL;
                                }
                                return;
                        }

                        if (j++ > 2) {
                                u32 udccs0 = UDCCS0;
                                PXA_REGS(udccs0,"  ep0 wait");
                                if ((udccs0 & (UDCCS0_OPR | UDCCS0_SA | UDCCS0_RNE)) == (UDCCS0_OPR | UDCCS0_SA)) {
                                        UDCCS0 = UDCCS0_OPR | UDCCS0_SA;
                                        PXA_REGS(UDCCS0,"  ep0 force");
                                }
                                else {
                                        UDCCS0 = UDCCS0_OPR | UDCCS0_SA;
                                        PXA_REGS(UDCCS0,"  ep0 force and return");
                                        break;
                                }
                        }

                } while (UDCCS0 & (UDCCS0_OPR | UDCCS0_RNE));
                break;

        case DATA_STATE_XMIT:
                pxa_ep0xmit(endpoint, UDCCS0);
                break;
#ifdef CONFIG_USBD_EP0_SUPPORT
	case DATA_STATE_RECV:
		pxa_ep0recv(endpoint, UDCCS0);
		break;
#endif
        }

        pxa_ep_reset_irs(0);
        PXA_REGS(UDCCS0,"  <-- ep0");
}

/* ********************************************************************************************* */
/* Interrupt Handler
 */

/**
 * int_hndlr - interrupt handler
 *
 */
static void int_hndlr (int irq, void *dev_id, struct pt_regs *regs)
{
        int usiro;
        int udccr;
        int ep;

        int_oscr = OSCR;
	udc_interrupts++;

        // check for common, high priority interrupts first, i.e. per endpoint service requests
        // XXX if ISO supported it might be necessary to give the ISO endpoints priority
        
        while ((usiro = USIR0)) {
                u32 udccs0 = UDCCS0;
                PXA_IRO(usiro, "------------------------> Interrupt");
                for (ep = 0; usiro; usiro >>= 1, ep++) {
                        if (usiro & 1) {
                                switch (ep_maps[ep].eptype) {
                                case ep_control:
                                        pxa_ep0(ep_endpoints[0], udccs0);
                                        //PXA_IRO(USIRO, "<-- Interrupt");
                                        break;

                                case ep_bulk_in:
                                case ep_interrupt:
                                        pxa_in_n(ep, ep_endpoints[ep]);
                                        break;

                                case ep_bulk_out:
#ifdef CONFIG_USBD_PXA_DMA_OUT
					printk (KERN_WARNING "PXA: bulk out with DMA out, ep %d - may be a bug\n", ep);
#else
					pxa_out_n(ep, ep_endpoints[ep]);
#endif
					pxa_ep_reset_irs(ep);
					break;
                                case ep_iso_in:
                                case ep_iso_out:
                                        pxa_ep_reset_irs(ep);
                                        break;
                                }
                        }
                }
        }

        // sof interrupt
        if (UFNHR & UFNHR_SIR) {
                UFNHR = UFNHR_SIR;
        }

        // uncommon interrupts
        if ((udccr = UDCCR) & (UDCCR_RSTIR | UDCCR_RESIR | UDCCR_SUSIR)) {

                // UDC Reset
                if (udccr & UDCCR_RSTIR) {
                        PXA_CCR(udccr, "------------------------> Reset");
                        //printk(KERN_INFO"int_hndlr[%d:%d] Reset\n", udc_interrupts, jifs());
                                        
                        udc_suspended = 0;
                        usbd_device_event (udc_device, DEVICE_RESET, 0);
                        usbd_device_event (udc_device, DEVICE_ADDRESS_ASSIGNED, 0);	
                        UDCCR |= UDCCR_RSTIR;
                }

                // UDC Resume
                if (udccr & UDCCR_RESIR) {
                        PXA_CCR(udccr, "------------------------> Resume");
                        if (udc_suspended) {
                                udc_suspended = 0;
                                usbd_device_event (udc_device, DEVICE_BUS_ACTIVITY, 0);
                        }
                        UDCCR |= UDCCR_RESIR;
                }

                // UDC Suspend
                if (udccr & UDCCR_SUSIR) {
                        PXA_CCR(udccr, "------------------------> Suspend");
                        if (!udc_suspended) {
                                udc_suspended = 1;
                                usbd_device_event (udc_device, DEVICE_BUS_INACTIVE, 0);
                        }
                        UDCCR |= UDCCR_SUSIR;
                }
        }
        PXA_CCR(UDCCR, "<-- Interrupt");
}


/* ********************************************************************************************* */


/* ********************************************************************************************* */
/*
 * Start of public functions.
 */

/**
 * udc_start_in_irq - start transmit
 * @eendpoint: endpoint instance
 *
 * Called by bus interface driver to see if we need to start a data transmission.
 */
void udc_start_in_irq (struct usb_endpoint_instance *endpoint)
{
        if (UDCCSN(endpoint->endpoint_address & 0xf) & UDCCS_BI_TFS) {
#ifdef CONFIG_USBD_PXA_DMA_IN
                if (usb_pxa_dma_in.dma_buf) /* Buffer ok, use DMA */
                        pxa_start_n_dma (endpoint->endpoint_address & 0xf, endpoint);
                else /* Buffer is not allocated, switch to polling mode */
                        pxa_start_n (endpoint->endpoint_address & 0xf, endpoint);
#else
                pxa_start_n (endpoint->endpoint_address & 0xf, endpoint);
#endif
        }
}

/**
 * udc_init - initialize
 *
 * Return non-zero if we cannot see device.
 **/
int udc_init (void)
{
        udc_disable_interrupts (NULL);
#ifdef CONFIG_USBD_PXA_DMA_IN
        if (!(usb_pxa_dma_in.dma_buf = (unsigned char *)consistent_alloc 
              (GFP_KERNEL, DMA_IN_BUFSIZE, &usb_pxa_dma_in.dma_buf_phys))) {
                /* Can't allocate DMAable memory... Make a warning and not use DMA */
                printk (KERN_WARNING "PXA: can't allocate consistent memory for USB DMA IN\n");
                printk (KERN_WARNING "PXA: USB DMA IN is disabled\n");
        }
#endif
#ifdef CONFIG_USBD_PXA_DMA_OUT
        if (!(usb_pxa_dma_out.dma_buf = (unsigned char *)consistent_alloc 
              (GFP_KERNEL, DMA_OUT_BUFSIZE, &usb_pxa_dma_out.dma_buf_phys))) {
                /* The same as above */
                printk (KERN_WARNING "PXA: can't allocate consistent memory for USB DMA OUT\n");
                printk (KERN_WARNING "PXA: USB DMA OUT is disabled\n");
        }
#endif
	return 0;
}


/**
 * udc_start_in - start transmit
 * @eendpoint: endpoint instance
 *
 * Called by bus interface driver to see if we need to start a data transmission.
 */
void udc_start_in (struct usb_endpoint_instance *endpoint)
{
	if (endpoint) {
		unsigned long flags;
		local_irq_save (flags);
                udc_start_in_irq(endpoint);
		local_irq_restore (flags);
	}
}


/**
 * udc_stall_ep - stall endpoint
 * @ep: physical endpoint
 *
 * Stall the endpoint.
 */
void udc_stall_ep (unsigned int ep)
{
	if (ep < UDC_MAX_ENDPOINTS) {
		// stall
	}
}


/**
 * udc_reset_ep - reset endpoint
 * @ep: physical endpoint
 * reset the endpoint.
 *
 * returns : 0 if ok, -1 otherwise
 */
void udc_reset_ep (unsigned int ep)
{
	if (ep < UDC_MAX_ENDPOINTS) {
	}
}


/**
 * udc_endpoint_halted - is endpoint halted
 * @ep:
 *
 * Return non-zero if endpoint is halted
 */
int udc_endpoint_halted (unsigned int ep)
{
	return 0;
}


/**
 * udc_set_address - set the USB address for this device
 * @address:
 *
 * Called from control endpoint function after it decodes a set address setup packet.
 */
void udc_set_address (unsigned char address)
{
}

/**
 * udc_serial_init - set a serial number if available
 */
int __init udc_serial_init (struct usb_bus_instance *bus)
{
	return -EINVAL;
}

/* ********************************************************************************************* */

/**
 * udc_max_endpoints - max physical endpoints 
 *
 * Return number of physical endpoints.
 */
int udc_max_endpoints (void)
{
	return UDC_MAX_ENDPOINTS;
}


/**
 * udc_check_ep - check logical endpoint 
 * @lep:
 *
 * Return physical endpoint number to use for this logical endpoint or zero if not valid.
 */
int udc_check_ep (int logical_endpoint, int packetsize)
{
        // XXX check ep table

        return ( ((logical_endpoint & 0xf) >= UDC_MAX_ENDPOINTS) || (packetsize > 64)) 
                ?  0 : (logical_endpoint & 0xf);
}


/**
 * udc_set_ep - setup endpoint 
 * @ep:
 * @endpoint:
 *
 * Associate a physical endpoint with endpoint_instance
 */
void udc_setup_ep (struct usb_device_instance *device, unsigned int ep,
		   struct usb_endpoint_instance *endpoint)
{
	if (ep < UDC_MAX_ENDPOINTS) {
		ep_endpoints[ep] = endpoint;

		// ep0
		if (ep == 0) {
		}
		// IN
		else if (endpoint->endpoint_address & 0x80) {
#ifdef CONFIG_USBD_PXA_DMA_IN
                        if (ep == IN_ENDPOINT && usb_pxa_dma_in.dmach == -1 &&
                            /* No DMA IN if we can't allocate DMAable buffer in udc_init() */
                            usb_pxa_dma_in.dma_buf) {
                                /* Get DMA irq */
                                if ((usb_pxa_dma_in.dmach = pxa_request_dma
                                     ("USB DMA IN", DMA_PRIO_HIGH, pxa_tx_dma_irq,
				      (void *)&usb_pxa_dma_in)) < 0) {
					printk (KERN_WARNING "PXA: can't allocate DMA IRQ for USB DMA IN\n");
					printk (KERN_WARNING "PXA: USB DMA IN is disabled\n");
					consistent_free (usb_pxa_dma_in.dma_buf,
							 DMA_IN_BUFSIZE, usb_pxa_dma_in.dma_buf_phys);
					usb_pxa_dma_in.dma_buf = NULL;
				}
				else
					DMA_TRACEMSG (KERN_DEBUG "PXA: USB IN DMA channel %d\n", usb_pxa_dma_in.dmach);
                        }
#endif /* CONFIG_USBD_PXA_DMA_IN */
		}
		// OUT
		else if (endpoint->endpoint_address) {
			usbd_fill_rcv (device, endpoint, 5);
			endpoint->rcv_urb = first_urb_detached (&endpoint->rdy);
#ifdef CONFIG_USBD_PXA_DMA_OUT
                        if (ep == OUT_ENDPOINT && usb_pxa_dma_out.dmach == -1 &&
                            /* Again, no DMA OUT if we can't allocate DMAable buffer in udc_init() */
                            usb_pxa_dma_out.dma_buf) {
                                /* Get DMA irq */
                                if ((usb_pxa_dma_out.dmach = pxa_request_dma
                                     ("USB DMA OUT", DMA_PRIO_HIGH, pxa_rx_dma_irq,
				      (void *)&usb_pxa_dma_out)) < 0) {
					printk (KERN_WARNING "PXA: can't allocate DMA IRQ for USB DMA OUT\n");
					printk (KERN_WARNING "PXA: USB DMA OUT is disabled\n");
					consistent_free (usb_pxa_dma_out.dma_buf,
							 DMA_OUT_BUFSIZE, usb_pxa_dma_out.dma_buf_phys);
					usb_pxa_dma_out.dma_buf = NULL;
				}
				else {
					DMA_TRACEMSG (KERN_DEBUG "PXA: USB OUT DMA channel %d\n", usb_pxa_dma_out.dmach);
					usb_pxa_dma_out.ep = ep;
					usb_pxa_dma_out.endpoint = endpoint;
					usb_pxa_dma_out.size = DMA_OUT_BUFSIZE;
					
					pxa_rx_dma_hw_init ();
					/* Enable DME for BULK OUT endpoint */
					UDCCS2 |= UDCCS_BO_DME;
					DMA_TRACEMSG ("PXA: enable DME\n");

					/* Start DMA */
					DCSR(usb_pxa_dma_out.dmach) |= DCSR_RUN;
					DMA_TRACEMSG ("PXA: OUT DMA started [ep %d]\n", ep);
				}
			}
#endif /* CONFIG_USBD_PXA_DMA_OUT */
		}
		pxa_enable_ep_interrupt(ep_endpoints[ep]->endpoint_address);
	}
}

/**
 * udc_disable_ep - disable endpoint
 * @ep:
 *
 * Disable specified endpoint 
 */
void udc_disable_ep (unsigned int ep)
{
	if (ep < UDC_MAX_ENDPOINTS) {
		struct usb_endpoint_instance *endpoint;

		if ((endpoint = ep_endpoints[ep])) {
			ep_endpoints[ep] = NULL;
			usbd_flush_ep (endpoint);
		}
		if (ep == 0) {
                        //printk(KERN_INFO"udc_setup_ep: 0 do nothing\n");
		}
		// IN
		else if (endpoint && endpoint->endpoint_address & 0x80) {
#ifdef CONFIG_USBD_PXA_DMA_IN
                        if (ep == IN_ENDPOINT && usb_pxa_dma_in.dmach != -1) {
				pxa_free_dma (usb_pxa_dma_in.dmach);
				usb_pxa_dma_in.dmach = -1;
			}
#endif
		}
		// OUT
		else if (endpoint && endpoint->endpoint_address) {
#ifdef CONFIG_USBD_PXA_DMA_OUT
                        if (ep == OUT_ENDPOINT && usb_pxa_dma_out.dmach != -1) {
				pxa_free_dma (usb_pxa_dma_out.dmach);
				usb_pxa_dma_out.dmach = -1;
			}
#endif
		}
	}
}

/* ********************************************************************************************* */

/**
 * udc_connected - is the USB cable connected
 *
 * Return non-zeron if cable is connected.
 */
int udc_connected ()
{
	return 1;
}

/**
 * udc_connect - enable pullup resistor
 *
 * Turn on the USB connection by enabling the pullup resistor.
 */
void udc_connect (void)
{
#ifdef CONFIG_SABINAL_DISCOVERY
        /* Set GPIO pin function to I/O */
        GPAFR1_U &= ~(0x3 << ((USBD_CONNECT_GPIO & 0xF) << 1));
        /* Set pin direction to output */
        GPDR(1) |= GPIO_GPIO(USBD_CONNECT_GPIO);
#if defined(USBD_CONNECT_HIGH)
	/* Set GPIO pin high to connect */
	GPSR(1) = GPIO_GPIO(USBD_CONNECT_GPIO);
#else
	/* Set GPIO pin low to connect */
	GPCR(1) = GPIO_GPIO(USBD_CONNECT_GPIO);
#endif
#else
#warning NO USB Device connect
#endif
}

/**
 * udc_disconnect - disable pullup resistor
 *
 * Turn off the USB connection by disabling the pullup resistor.
 */
void udc_disconnect (void)
{
#ifdef CONFIG_SABINAL_DISCOVERY
        /* Set GPIO pin function to I/O */
        GPAFR1_U &= ~(0x3 << ((USBD_CONNECT_GPIO & 0xF) << 1));
        /* Set pin direction to output */
        GPDR(1) |= GPIO_GPIO(USBD_CONNECT_GPIO);
#if defined(USBD_CONNECT_HIGH)
	/* Set GPIO pin low to disconnect */
	GPCR(1) = GPIO_GPIO(USBD_CONNECT_GPIO);
#else
	/* Set GPIO pin high to disconnect */
	GPSR(1) = GPIO_GPIO(USBD_CONNECT_GPIO);
#endif
#else
#warning NO USB Device connect
#endif
}

#if 0
/**
 * udc_int_hndlr_cable - interrupt handler for cable
 */
static void udc_int_hndlr_cable (int irq, void *dev_id, struct pt_regs *regs)
{
	// GPIOn interrupt
}
#endif

/* ********************************************************************************************* */

/**
 * udc_enable_interrupts - enable interrupts
 *
 * Switch on UDC interrupts.
 *
 */
void udc_all_interrupts (struct usb_device_instance *device)
{
        int i;

        UDCCR &= ~UDCCR_SRM;    // enable suspend interrupt
        UDCCR |= UDCCR_REM;     // disable resume interrupt

        // XXX SOF UFNHR &= ~UFNHR_SIM;

        // always enable control endpoint
        pxa_enable_ep_interrupt(0);

        for (i = 1; i < UDC_MAX_ENDPOINTS; i++) {
                if (ep_endpoints[i] && ep_endpoints[i]->endpoint_address) {
			pxa_enable_ep_interrupt(ep_endpoints[i]->endpoint_address);
                }
        }
}


/**
 * udc_suspended_interrupts - enable suspended interrupts
 *
 * Switch on only UDC resume interrupt.
 *
 */
void udc_suspended_interrupts (struct usb_device_instance *device)
{
        UDCCR &= ~UDCCR_REM;    // enable resume interrupt
        UDCCR |= UDCCR_SRM;     // disable suspend interrupt
}


/**
 * udc_disable_interrupts - disable interrupts.
 *
 * switch off interrupts
 */
void udc_disable_interrupts (struct usb_device_instance *device)
{
        UICR0 = UICR1 = 0xff;                   // disable endpoint interrupts
        UFNHR |= UFNHR_SIM;                     // disable sof interrupt
        UDCCR |= UDCCR_REM | UDCCR_SRM;         // disable suspend and resume interrupt
}

/* ********************************************************************************************* */

/**
 * udc_ep0_packetsize - return ep0 packetsize
 */
int udc_ep0_packetsize (void)
{
	return EP0_PACKETSIZE;
}

/**
 * udc_enable - enable the UDC
 *
 * Switch on the UDC
 */
void udc_enable (struct usb_device_instance *device)
{
	// save the device structure pointer
	udc_device = device;

	// ep0 urb
	if (!ep0_urb) {
		if (!(ep0_urb = usbd_alloc_urb (device, device->function_instance_array, 0, 512))) {
			printk (KERN_ERR "udc_enable: usbd_alloc_urb failed\n");
		}
	} 
        else {
		printk (KERN_ERR "udc_enable: ep0_urb already allocated\n");
	}


	// enable UDC

	// c.f. 3.6.2 Clock Enable Register
	CKEN |= CKEN11_USB;

        // c.f. 12.4.11 GPIOn and GPIOx
        // enable cable interrupt
        

        // c.f. 12.5 UDC Operation - after reset on EP0 interrupt is enabled.

        // c.f. 12.6.1.1 UDC Enable
        UDCCR |= UDCCR_UDE | UDCCR_REM;
}


/**
 * udc_disable - disable the UDC
 *
 * Switch off the UDC
 */
void udc_disable (void)
{
        printk ("PXA: call udc_disable\n");

	// disable UDC
        // c.f. 12.6.1.1 UDC Enable
        UDCCR &= ~( UDCCR_UDE | UDCCR_REM );

	// c.f. 3.6.2 Clock Enable Register
	CKEN &= ~CKEN11_USB;

        // disable cable interrupt
        
	// reset device pointer
	udc_device = NULL;

	// ep0 urb

	if (ep0_urb) {
		usbd_dealloc_urb (ep0_urb);
		ep0_urb = 0;
	} else {
		printk (KERN_ERR "udc_disable: ep0_urb already NULL\n");
	}
#ifdef CONFIG_USBD_PXA_DMA_IN
        if (usb_pxa_dma_in.dma_buf) {
                consistent_free (usb_pxa_dma_in.dma_buf,
                                 DMA_IN_BUFSIZE, usb_pxa_dma_in.dma_buf_phys);
                usb_pxa_dma_in.dma_buf = NULL;
        }
#endif
#ifdef CONFIG_USBD_PXA_DMA_OUT
        if (usb_pxa_dma_out.dma_buf) {
                consistent_free (usb_pxa_dma_out.dma_buf,
                                 DMA_OUT_BUFSIZE, usb_pxa_dma_out.dma_buf_phys);
                usb_pxa_dma_out.dma_buf = NULL;
        }
#endif
}


/**
 * udc_startup - allow udc code to do any additional startup
 */
void udc_startup_events (struct usb_device_instance *device)
{
	usbd_device_event (device, DEVICE_INIT, 0);
	usbd_device_event (device, DEVICE_CREATE, 0);
	usbd_device_event (device, DEVICE_HUB_CONFIGURED, 0);
	usbd_device_event (device, DEVICE_RESET, 0);
}


/* ********************************************************************************************* */
#ifdef PXA_TRACE
#ifdef CONFIG_USBD_PROCFS
/* Proc Filesystem *************************************************************************** */
        
/* *    
 * pxa_proc_read - implement proc file system read.
 * @file        
 * @buf         
 * @count
 * @pos 
 *      
 * Standard proc file system read function.
 */         
static ssize_t pxa_proc_read (struct file *file, char *buf, size_t count, loff_t * pos)
{                                  
        unsigned long page;
        int len = 0;
        int index;

        MOD_INC_USE_COUNT;
        // get a page, max 4095 bytes of data...
        if (!(page = get_free_page (GFP_KERNEL))) {
                MOD_DEC_USE_COUNT;
                return -ENOMEM;
        }

        len = 0;
        index = (*pos)++;

        if (index == 0) {
                len += sprintf ((char *) page + len, "   Index     Ints     Jifs    Ticks\n");
        }       
                         

        if (index < trace_next) {

                u64 jifs = 1111;
                u32 ticks = 2222;
                pxa_trace_t *p = pxa_traces + index;
                unsigned char *cp;

                if (index > 0) {
                        u32 ocsr = pxa_traces[index-1].ocsr;
                        ticks = (p->ocsr > ocsr) ? (p->ocsr - ocsr) : (ocsr - p->ocsr) ;
                        jifs = p->jiffies - pxa_traces[index-1].jiffies;
                }
                
                len += sprintf ((char *) page + len, "%8d %8d %8lu ", index, p->interrupts, jifs);

                if (ticks > 1024*1024) {
                        len += sprintf ((char *) page + len, "%8dM ", ticks>>20);
                }
                else {
                        len += sprintf ((char *) page + len, "%8d  ", ticks);
                }
                switch (p->trace_type) {
                case pxa_regs:
                        len += sprintf ((char *) page + len, "CS0[%02x] %s\n", p->trace.regs.cs0, p->trace.regs.msg);
                        break;
                case pxa_setup:
                        cp = (unsigned char *)&p->trace.setup;
                        len += sprintf ((char *) page + len, 
                                        " --               request [%02x %02x %02x %02x %02x %02x %02x %02x]\n", 
                                        cp[0], cp[1], cp[2], cp[3], cp[4], cp[5], cp[6], cp[7]);
                        break;
                case pxa_xmit:
                        len += sprintf ((char *) page + len, 
                                        " --                   sending [%02x]\n", p->trace.xmit.size);
                        break;
                case pxa_ccr:
                        len += sprintf ((char *) page + len, 
                                        "CCR[%02x] %s\n", p->trace.ccr.ccr, p->trace.ccr.msg);
                        break;
                case pxa_iro:
                        len += sprintf ((char *) page + len, 
                                        "IRO[%02x] %s\n", p->trace.iro.iro, p->trace.iro.msg);
                        break;
                }
        }

        if (len > count) {
                len = -EINVAL;
        } else if (len > 0 && copy_to_user (buf, (char *) page, len)) {
                len = -EFAULT;
        }
        free_page (page);
        MOD_DEC_USE_COUNT;
        return len;
}
/* *
 * pxa_proc_write - implement proc file system write.
 * @file
 * @buf
 * @count
 * @pos
 *
 * Proc file system write function, used to signal monitor actions complete.
 * (Hotplug script (or whatever) writes to the file to signal the completion
 * of the script.)  An ugly hack.
 */
static ssize_t pxa_proc_write (struct file *file, const char *buf, size_t count, loff_t * pos)
{
        return count;
}

static struct file_operations pxa_proc_operations_functions = {
        read:pxa_proc_read,
        write:pxa_proc_write,
};

#endif
#endif


/* ********************************************************************************************* */
/**
 * udc_name - return name of USB Device Controller
 */
char *udc_name (void)
{
	return UDC_NAME;
}

/**
 * udc_request_udc_irq - request UDC interrupt
 *
 * Return non-zero if not successful.
 */
int udc_request_udc_irq ()
{
	// request IRQ  and IO region
	if (request_irq (IRQ_USB, int_hndlr, SA_INTERRUPT | SA_SAMPLE_RANDOM,
	     UDC_NAME " USBD Bus Interface", NULL) != 0) 
        {
		printk (KERN_INFO "usb_ctl: Couldn't request USB irq\n");
		return -EINVAL;
	}
	return 0;
}

/**
 * udc_request_cable_irq - request Cable interrupt
 *
 * Return non-zero if not successful.
 */
int udc_request_cable_irq ()
{
#ifdef XXXX_CABLE_IRQ
	// request IRQ  and IO region
	if (request_irq
	    (XXXX_CABLE_IRQ, int_hndlr, SA_INTERRUPT | SA_SAMPLE_RANDOM, UDC_NAME " Cable",
	     NULL) != 0) {
		printk (KERN_INFO "usb_ctl: Couldn't request USB irq\n");
		return -EINVAL;
	}
#endif
	return 0;
}

/**
 * udc_request_udc_io - request UDC io region
 *
 * Return non-zero if not successful.
 */
int udc_request_io ()
{
#ifdef PXA_TRACE
#ifdef CONFIG_USBD_PROCFS
        if (!(pxa_traces = vmalloc(sizeof(pxa_trace_t) * TRACE_MAX))) {
                printk(KERN_ERR"PXA_TRACE malloc failed %p %d\n", pxa_traces, sizeof(pxa_trace_t) * TRACE_MAX);
        }
        else {
                printk(KERN_ERR"PXA_TRACE malloc ok %p\n", pxa_traces);
        }
        PXA_REGS(0,"init");
        PXA_REGS(0,"test");
        {
                struct proc_dir_entry *p;

                // create proc filesystem entries
                if ((p = create_proc_entry ("pxa", 0, 0)) == NULL) {
                        printk(KERN_INFO"PXA PROC FS failed\n");
                }
                else {
                        printk(KERN_INFO"PXA PROC FS Created\n");
                        p->proc_fops = &pxa_proc_operations_functions;
                }
        }
#endif
#endif
	return 0;
}

/**
 * udc_release_release_io - release UDC io region
 */
void udc_release_io ()
{
#ifdef PXA_TRACE
#ifdef CONFIG_USBD_PROCFS
        unsigned long flags;
        save_flags_cli (flags);
        remove_proc_entry ("pxa", NULL);
        if (pxa_traces) {
                pxa_trace_t *p = pxa_traces;
                pxa_traces = 0;
                vfree(p);
        }
        restore_flags (flags);
#endif
#endif

}

/**
 * udc_release_udc_irq - release UDC irq
 */
void udc_release_udc_irq ()
{
	free_irq (IRQ_USB, NULL);
}

/**
 * udc_release_cable_irq - release Cable irq
 */
void udc_release_cable_irq ()
{
#ifdef XXXX_IRQ
	free_irq (XXXX_CABLE_IRQ, NULL);
#endif
}


/**
 * udc_regs - dump registers
 *
 * Dump registers with printk
 */
void udc_regs (void)
{
	printk ("[%d:%d] CCR[%02x] UICR[%02x %02x] UFNH[%02x %02x] UDCCS[%02x %02x %02x %02x]\n", 
                        udc_interrupts, jifs(), UDCCR, UICR1, UICR0, UFNHR, UFNLR, UDCCS0, UDCCS1, UDCCS2, UDCCS5);
}

/*
 * dma_regs - dump DMA state
 * G.N.U., but may be useful for DMA debugging.
 */
static void dma_regs (void)
{
#ifdef CONFIG_USBD_PXA_DMA_IN
	printk ("  DMA IN: DCSR:%x DCMD:%x DSADR:%x DTADR:%x\n",
		DCSR(usb_pxa_dma_in.dmach), DCMD(usb_pxa_dma_in.dmach),
		DSADR(usb_pxa_dma_in.dmach), DTADR(usb_pxa_dma_in.dmach));
#endif
#ifdef CONFIG_USBD_PXA_DMA_OUT
	printk ("  DMA OUT: DCSR:%x DCMD:%x DSADR:%x DTADR:%x\n",
		DCSR(usb_pxa_dma_out.dmach), DCMD(usb_pxa_dma_out.dmach),
		DSADR(usb_pxa_dma_out.dmach), DTADR(usb_pxa_dma_out.dmach));
#endif
}
