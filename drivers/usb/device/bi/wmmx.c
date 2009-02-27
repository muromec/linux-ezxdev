/*
 * wmmx.c -- Bulverde USB Device Controller driver.
 *
 * 09/04/2003
 * Stanley Cai (stanley.cai@intel.com) ported the driver to Bulverde 
 * Mainstone.
 *
 * Changes Copyright (c) 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *      source@mvista.com
 *
 * Based on work:
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
 * 
 *
 */

/*
 * This code was developed on the Intel Mainstone with Bulverde processor
 */
#include <linux/config.h>
#include <linux/module.h>

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-module.h"
#include "../usbd-debug.h"

MODULE_DESCRIPTION ("WMMX USB Device Bus Interface");
USBD_MODULE_INFO ("wmmx_bi 0.2-alpha");
MODULE_LICENSE("GPL");

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <asm/atomic.h>
#include <asm/io.h>

#include <linux/proc_fs.h>
#include <linux/vmalloc.h>

#include <linux/netdevice.h>

#include <asm/irq.h>
#include <asm/system.h>

#include <asm/types.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <asm/hardware.h>
#include <asm/dma.h>

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-inline.h"
#include "usbd-bi.h"

#include "wmmx.h"

#define WMMX_TRACE
//#undef WMMX_TRACE

#define WMMX_XFER_DIR_OUT		0
#define WMMX_XFER_DIR_IN		1

#ifdef WMMX_TRACE

#define TRACE_MAX       		10000
#define WMMX_XFER_MAX_TRACE_SIZE	16

typedef enum wwmx_trace_type {
       wmmx_regs, 
       wmmx_setup, 
       wmmx_xmit, 
       wmmx_ccr, 
       wmmx_iro,
       wmmx_xfer
} wmmx_trace_type_t;

typedef struct wmmx_regs {
        u32     cs0;
        char *  msg;
} wmmx_regs_t;

typedef struct wmmx_xmit {
        u32     size;
} wmmx_xmit_t;

typedef struct wmmx_ccr {
        u32     ccr;
        char *  msg;
} wmmx_ccr_t;

typedef struct wmmx_iro {
        u32     iro;
        char *  msg;
} wmmx_iro_t;


typedef struct wmmx_xfer {
	char	dir;
	u32	size;
	char    data[WMMX_XFER_MAX_TRACE_SIZE];
} wmmx_xfer_t;

typedef struct wmmx_trace {
        wmmx_trace_type_t        trace_type;
        u32     interrupts;
        u32     ocsr;
        u64     jiffies;
        union {
                wmmx_regs_t       regs;
                wmmx_xmit_t       xmit;
                wmmx_ccr_t        ccr;
                wmmx_iro_t        iro;
		wmmx_xfer_t	  xfer;
                struct usb_device_request       setup;
        } trace;
} wmmx_trace_t;

int trace_next;
wmmx_trace_t *wmmx_traces;

static __inline__ void WMMX_REGS(u32 cs0, char *msg)
{
	if ((trace_next < TRACE_MAX) && wmmx_traces) {
		wmmx_trace_t *p = wmmx_traces + trace_next++;

		p->ocsr = OSCR;
		p->jiffies = jiffies;
		p->interrupts = udc_interrupts;
		p->trace_type = wmmx_regs;
		p->trace.regs.cs0 = cs0;
		p->trace.regs.msg = msg;
	}
}

static __inline__ void WMMX_SETUP(struct usb_device_request *setup)
{
	if ((trace_next < TRACE_MAX) && wmmx_traces) {
		wmmx_trace_t *p = wmmx_traces + trace_next++;

		p->ocsr = OSCR;
		p->jiffies = jiffies;
		p->interrupts = udc_interrupts;
		p->trace_type = wmmx_setup;
		memcpy(&p->trace.setup, setup, sizeof(struct usb_device_request));
	}
}

static __inline__ void WMMX_XMIT(u32 size)
{
	if ((trace_next < TRACE_MAX) && wmmx_traces) {
		wmmx_trace_t *p = wmmx_traces + trace_next++;

		p->ocsr = OSCR;
		p->jiffies = jiffies;
		p->interrupts = udc_interrupts;
		p->trace_type = wmmx_xmit;
		p->trace.xmit.size = size;
	}
}

static __inline__ void WMMX_CCR(u32 ccr, char *msg)
{
	if ((trace_next < TRACE_MAX) && wmmx_traces) {
		wmmx_trace_t *p = wmmx_traces + trace_next++;

		p->ocsr = OSCR;
		p->jiffies = jiffies;
		p->interrupts = udc_interrupts;
		p->trace_type = wmmx_ccr;
		p->trace.ccr.ccr = ccr;
		p->trace.ccr.msg = msg;
	}
}

static __inline__ void WMMX_IRO(u32 iro, char *msg)
{
	if ((trace_next < TRACE_MAX) && wmmx_traces) {
		wmmx_trace_t *p = wmmx_traces + trace_next++;

		p->ocsr = OSCR;
		p->jiffies = jiffies;
		p->interrupts = udc_interrupts;
		p->trace_type = wmmx_iro;
		p->trace.iro.iro = iro;
		p->trace.iro.msg = msg;
	}
}

static __inline__ void WMMX_XFER(char dir, u32 size, char *data)
{
	if ((trace_next < TRACE_MAX) && wmmx_traces) {
		wmmx_trace_t *p = wmmx_traces + trace_next++;

		p->ocsr = OSCR;
		p->jiffies = jiffies;
		p->interrupts = udc_interrupts;
		p->trace_type = wmmx_xfer;
		p->trace.xfer.dir = dir;
		p->trace.xfer.size = size;
		if (size > WMMX_XFER_MAX_TRACE_SIZE)
			size = WMMX_XFER_MAX_TRACE_SIZE;
		memcpy(&p->trace.xfer.data, data, size);
	}
}
#else
static __inline__ void WMMX_REGS(u32 cs0, char *msg)
{
}

static __inline__ void WMMX_SETUP(struct usb_device_request *setup)
{
}

static __inline__ void WMMX_XMIT(u32 size)
{
}

static __inline__ void WMMX_CCR(u32 ccr, char *msg)
{
}

static __inline__ void WMMX_IRO(u32 iro, char *msg)
{
}

static __inline__ void WMMX_XFER(char dir, u32 size, char *data)
{
}
#endif

#define WMMX_XFER_DIR_OUT		0
#define WMMX_XFER_DIR_IN		1

static int winhost_flag = 0;
static int udcbegin_flag = 1;

static int udc_suspended;
static struct usb_device_instance *udc_device; 

/*
 *  physical endpoints to logical endpoints
 */
static struct usb_endpoint_instance *phys_ep_to_endpoints[UDC_MAX_ENDPOINTS];
static int logic_to_phys_eps[MAX_LOGICAL_CONFIGURATIONS][MAX_LOGICAL_ENDPOINTS];

static struct urb *ep0_urb;

extern unsigned int udc_interrupts;

static int jifs(void)
{
	static unsigned long jiffies_last;
	int elapsed = OSCR - jiffies_last;

	jiffies_last = OSCR;
	return elapsed;
}

/*
 * Map logical to physical
 */
typedef enum ep {
        ep_control   = 0x0, 
	ep_bulk_in   = 0x5, 
	ep_bulk_out  = 0x4, 
	ep_iso_in    = 0x3, 
	ep_iso_out   = 0x2, 
	ep_interrupt = 0x7
} ep_t;

struct ep_map {
	int             logical;
	ep_t            eptype;
	int             size;
	int             dma_chan;
	volatile u32 *  drcmr;
};

static struct ep_map ep_maps[UDC_MAX_ENDPOINTS]={
	{ logical: 0, eptype: ep_control,   size: 16, },
};


/* Note:  Please don't add any instruction to try print some messages in the 
 * function. Any careless operation will cause the timing runing away.
 */
static __inline__ int  wmmx_logic_to_phys_ep(int logic)
{
	int config;

	config = (UDCCR & UDCCR_ACN) >> UDCCR_ACN_S;
	return logic_to_phys_eps[config][logic];
}

static __inline__ void wmmx_enable_ep_interrupt(int phys_ep, u8 irq_type)
{
	irq_type &= (UDC_INT_FIFOERROR | UDC_INT_PACKETCMP);
        if (phys_ep < 16) {
                UDCICR0 |= irq_type << (phys_ep << 1);
        } else {
                UDCICR1 |= irq_type << ((phys_ep - 16) << 1);
        }
}

static __inline__ void wmmx_disable_ep_interrupt(int phys_ep, u8 irq_type)
{
	irq_type &= (UDC_INT_FIFOERROR | UDC_INT_PACKETCMP);
	if (phys_ep < 16) {
		UDCICR0 &= ~(irq_type << (phys_ep << 1));
	} else {
		UDCICR1 &= ~(irq_type << ((phys_ep - 16) << 1));
        }
}

static __inline__ void wmmx_ep_reset_interrupt_status(int phys_ep)
{
	const int intstat = (UDC_INT_FIFOERROR | UDC_INT_PACKETCMP);
	if (phys_ep < 16) {
		UDCISR0 = (intstat << (phys_ep << 1));
	} else {
		UDCISR1 = (intstat << ((phys_ep - 16) << 1));
	}
}

static __inline__ void wmmx_out_flush(int phys_ep) 
{
	u32 data;
	int len = UDCBCN(phys_ep);
	
	WMMX_XMIT(len);
	
	while (len > 0) {
                data = UDCDN(phys_ep);
		len -= 4;
        }
}

static void wmmx_out_n(int phys_ep, struct usb_endpoint_instance *endpoint)
{
	wmmx_ep_reset_interrupt_status(phys_ep);

	if (UDCCSN(phys_ep) & UDCCSR_PC) {
		if (endpoint) {
			if (!endpoint->rcv_urb) {
				endpoint->rcv_urb = 
					first_urb_detached(&endpoint->rdy);
                        } 
                        if (endpoint->rcv_urb) {
				int len = 0;
				u32 *cp = (u32*)(endpoint->rcv_urb->buffer + 
						 endpoint->rcv_urb->actual_length);
				if (cp) {
					int count = MIN(UDCBCN(phys_ep), 
							endpoint->rcv_packetSize);

					len = count;
					while (count > 0) {
						*cp++ = UDCDN(phys_ep);
						count -= 4;
					}

					/* clear PC and interrupt */
					UDCCSN(phys_ep) = UDCCSR_PC | 
							  (UDCCSR_WR_MASK &
							   UDCCSN(phys_ep));
				} else {
					wmmx_out_flush(phys_ep);
				}        
				
				WMMX_XFER(WMMX_XFER_DIR_OUT, len, endpoint->rcv_urb->buffer +
					  endpoint->rcv_urb->actual_length);

				/* fall through if error of any type, len = 0 */
				usbd_rcv_complete_irq (endpoint, len, 0);

				return;
			} else {
				wmmx_out_flush(phys_ep);
			}
		}
	}

	UDCCSN(phys_ep) = (UDCCSN(phys_ep) & UDCCSR_WR_MASK) |
			  UDCCSR_PC;
}


static void wmmx_start_n(unsigned int phys_ep, 
			 struct usb_endpoint_instance *endpoint)
{
	if (endpoint->tx_urb) {
		int last;
		struct urb *urb = endpoint->tx_urb;

		if ((last = MIN(urb->actual_length - 
				(endpoint->sent + endpoint->last), 
				endpoint->tx_packetSize))) {
			int size = last;
			u32 *data = (u32*)(urb->buffer + endpoint->sent + 
					   endpoint->last);
			u32 *cp;
			int remain = 0;
			u8 *rd;

			WMMX_XFER(WMMX_XFER_DIR_IN, last, (char *)data);

			cp = data;

			remain = size & 0x03;
			size &= ~0x03;

			while (size > 0) {
				UDCDN(phys_ep) = *cp++;
				size -= 4;
			}
			
			if (remain) {
				volatile u8 *reg = 
					(volatile u8 *)PUDCDN(phys_ep);
				rd = (u8 *)cp;
				while (remain--) {
					*reg = *rd++;
				}
			}

			if ((last < endpoint->tx_packetSize) ||
			    ((endpoint->tx_urb->actual_length - 
			      endpoint->sent) == last)) {

				if (last != endpoint->tx_packetSize)
				{
				UDCCSN(phys_ep) = (UDCCSN(phys_ep) & 
						   UDCCSR_WR_MASK) |
						  UDCCSR_SP;
			}
			}
			endpoint->last += last;
		} else {
			usbd_tx_complete_irq (endpoint, 0);
		}
	}
}


static void wmmx_in_n(unsigned int phys_ep, 
		      struct usb_endpoint_instance *endpoint)
{
	int udccsn;

	wmmx_ep_reset_interrupt_status(phys_ep);

	/* if TPC update tx urb and clear TPC */
	if ((udccsn = UDCCSN(phys_ep)) & UDCCSR_PC) {
                UDCCSN(phys_ep) = (UDCCSN(phys_ep) & UDCCSR_WR_MASK) |
				  UDCCSR_PC;
		usbd_tx_complete_irq(endpoint, 0);
	}

	if (udccsn & UDCCSR_FS) {
		wmmx_start_n(phys_ep, endpoint);
	}

	/* clear underrun, not much we can do about it */
	if (udccsn & UDCCSR_TRN) {
		UDCCSN(phys_ep) = (UDCCSN(phys_ep) & UDCCSR_WR_MASK) |
				  UDCCSR_TRN;
	}
}


void wmmx_ep0recv(struct usb_endpoint_instance *endpoint, volatile u32 udccsr0)
{
	struct urb *urb = endpoint->rcv_urb;
	int retval;

	WMMX_REGS(udccsr0, "         <-- setup data recv");

	/* check for premature status stage */
	if (!(udccsr0 & UDCCSR0_OPC) && !(udccsr0 & UDCCSR0_IPR)) {
		if (urb->device_request.wLength == urb->actual_length) {
			retval = usbd_recv_setup(ep0_urb);
			if (retval != 0) {
				UDCCSR0 = UDCCSR0_FST;
				endpoint->state = WAIT_FOR_SETUP;
				WMMX_REGS(udccsr0, "       --> bad setup FST");
			}
		} else {
			urb->actual_length = 0;
		}
		endpoint->state = WAIT_FOR_SETUP; 
	}

	/* receive more data */
	if ((udccsr0 & UDCCSR0_OPC) && !(udccsr0 & UDCCSR0_SA)) {
		int size = urb->device_request.wLength;
		u32* cp = (u32*) (urb->buffer + urb->actual_length);

		while (size > 0) {
			*cp++ = UDCDN(0);
			size -= 4;
		}
		/* resize the actual length of URB structure. */
		urb->actual_length += urb->device_request.wLength;

		UDCCSR0 |= UDCCSR0_OPC;

		/* to allow to enter a premature STATUS IN stage */
		UDCCSR0 |= UDCCSR0_IPR;
	}

	return;
}

void wmmx_ep0xmit(struct usb_endpoint_instance *endpoint, volatile u32 udccsr0)
{
	int short_packet;
	int size;
	struct urb *urb = endpoint->tx_urb;

	WMMX_REGS(udccsr0, "          --> xmit");

	/* check for premature status stage - host abandoned previous IN */
	if ((udccsr0 & UDCCSR0_OPC) && !(udccsr0 & UDCCSR0_SA)) {
		/* clear tx fifo and opr */
		UDCCSR0 |= UDCCSR0_FTF | UDCCSR0_OPC;
		endpoint->state = WAIT_FOR_SETUP;
		endpoint->tx_urb = NULL;
		endpoint->rcv_urb = NULL;
		WMMX_REGS(UDCCSR0, "          <-- xmit premature status");
		return;
	}

        /* check for stall */
	if (udccsr0 & UDCCSR0_SST) {
		/* clear stall and tx fifo */
		UDCCSR0 |= UDCCSR0_SST | UDCCSR0_FTF;
		endpoint->state = WAIT_FOR_SETUP;
		endpoint->tx_urb = NULL;
		endpoint->rcv_urb = NULL;
		WMMX_REGS(UDCCSR0, "          <-- xmit stall");
		return;
	}

	if (NULL == urb) {
		size = 0;
	} else {
		endpoint->last = 
		size = MIN(urb->actual_length - endpoint->sent, 
			   endpoint->tx_packetSize);
	}

	short_packet = (size < endpoint->tx_packetSize);

	WMMX_XMIT(size);
	if (size > 0 && urb->buffer) {
		u32* cp = (u32*)(urb->buffer + endpoint->sent);
		int remain = 0;
		u8 *rd;

		remain = size & 0x03;
		size &= ~0x3;

		while (size) {
			UDCDN(0) = *cp++;
			size -= 4;
		}
		
		if (remain) {
			volatile u8 *reg = (volatile u8 *)PUDCDN(0);
			rd = (u8 *)cp;
			while (remain--) {
				*reg = *rd++;
			}
		}
        }

        if (!endpoint->tx_urb || (endpoint->last < endpoint->tx_packetSize)) {
		/* Tell the UDC we are at the end of the packet. */
		UDCCSR0 |= UDCCSR0_IPR;
                endpoint->state = WAIT_FOR_OUT_STATUS;
                WMMX_REGS(UDCCSR0, "          <-- xmit wait for status");
	} else if ((endpoint->last == endpoint->tx_packetSize) && 
		   ((endpoint->last + endpoint->sent) == 
		    endpoint->tx_urb->actual_length)  &&
                   ((endpoint->tx_urb->actual_length) < 
                    le16_to_cpu(endpoint->tx_urb->device_request.wLength))) {
                endpoint->state = DATA_STATE_NEED_ZLP;
                WMMX_REGS(UDCCSR0, "          <-- xmit need zlp");
	} else {
                WMMX_REGS(UDCCSR0, "          <-- xmit not finished");
	}
}

void wmmx_ep0setup(struct usb_endpoint_instance *endpoint, u32 udccsr0)
{
	if ((udccsr0 & (UDCCSR0_SA | UDCCSR0_OPC | UDCCSR0_RNE))
	         == (UDCCSR0_SA | UDCCSR0_OPC | UDCCSR0_RNE)) {

		u32 dummy;
		int count;
		int len;
		int retval;
                u32 *cp = (u32 *)&ep0_urb->device_request;

                WMMX_REGS(udccsr0, "      --> setup");

		count = len = UDCBCN(0);
		if (count > 8) {
			dbg_rx(6, "Received a setup packet(len(%d)>8\n", len);
			if (!(winhost_flag || udcbegin_flag)) {
			printk(KERN_INFO "Received a setup packet(len(%d)>8)\n",
			       len);
		}
			count = 8;
		}

		while (count > 0) {
                        *cp++ = UDCDN(0);
			count -= 4;
			len -= 4;
		}

		while (len > 0) {
			dummy = UDCDN(0);
			len -= 4;
                }

                WMMX_SETUP(&ep0_urb->device_request);

		if (((ep0_urb->device_request.bmRequestType & 
		      USB_REQ_DIRECTION_MASK) == USB_REQ_HOST2DEVICE) && 
	            le16_to_cpu(ep0_urb->device_request.wLength) != 0) {

			endpoint->state = DATA_STATE_RECV;
			ep0_urb->actual_length = 0;
			endpoint->rcv_urb = ep0_urb;
                        UDCCSR0 = UDCCSR0_SA | UDCCSR0_OPC;
			WMMX_REGS(udccsr0, "     <-- setup recv");

			return;
		}

		retval = usbd_recv_setup(ep0_urb);
		if (retval) {
			UDCCSR0 = UDCCSR0_FST;
			endpoint->state = WAIT_FOR_SETUP;
			WMMX_REGS(udccsr0, "    --> bad SETUP FST");
		}

		if ((ep0_urb->device_request.bmRequestType & 
		     USB_REQ_DIRECTION_MASK) == USB_REQ_HOST2DEVICE) {

                        UDCCSR0 |= UDCCSR0_IPR;
                        WMMX_REGS(UDCCSR0,"      <-- setup nodata");
                } else {
                        /* Control Read */
                        if (!le16_to_cpu(ep0_urb->device_request.wLength)) {
				udc_stall_ep(0);
				WMMX_REGS(UDCCSR0, "      <-- setup stalling "
						   "zero wLength");
				return;
			}
			/* verify that we have non-zero length response */
			if (!ep0_urb->actual_length) {
				udc_stall_ep (0);
				WMMX_REGS(UDCCSR0,"      <-- setup stalling "
						  "zero response");
				return;
			}

			endpoint->tx_urb = ep0_urb;
			endpoint->sent = 0;
			endpoint->last = 0;
			endpoint->state = DATA_STATE_XMIT;
			wmmx_ep0xmit(endpoint, UDCCSR0);

			/* Clear SA and OPR bits */
			UDCCSR0 = UDCCSR0_SA | UDCCSR0_OPC;
			WMMX_REGS(UDCCSR0,"      <-- setup data");
		}
	}
}

static void wmmx_ep0(struct usb_endpoint_instance *endpoint, u32 udccsr0)
{
	int j = 0;

	WMMX_REGS(udccsr0,"  --> ep0");

	if ((endpoint->state != DATA_STATE_PENDING_XMIT) &&
	    (udccsr0 & UDCCSR0_SST)) {
		wmmx_ep_reset_interrupt_status(0);
		UDCCSR0 |= UDCCSR0_SST;
		WMMX_REGS(udccsr0,"  --> ep0 clear SST");
		if (endpoint) {
			endpoint->state = WAIT_FOR_SETUP;
			endpoint->tx_urb = NULL;
			endpoint->rcv_urb = NULL;
		}
		return;
	}

	if (!endpoint) {
		wmmx_ep_reset_interrupt_status(0);
		UDCCSR0 |= UDCCSR0_IPR | UDCCSR0_OPC | UDCCSR0_SA;
		WMMX_REGS(UDCCSR0,"  ep0 NULL");
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

        if ((endpoint->state != DATA_STATE_PENDING_XMIT) &&
	    (endpoint->state != WAIT_FOR_SETUP) && 
	    (udccsr0 & UDCCSR0_SA)) {

		WMMX_REGS(udccsr0,"  --> ep0 early SA");
		endpoint->state = WAIT_FOR_SETUP;
		endpoint->tx_urb = NULL;
		endpoint->rcv_urb = NULL;
	}

	WMMX_REGS(endpoint->state,"  -- endpoint state");
	switch (endpoint->state) {
	case DATA_STATE_NEED_ZLP:
		UDCCSR0 |= UDCCSR0_IPR;
		endpoint->state = WAIT_FOR_OUT_STATUS;
		break;

	case WAIT_FOR_OUT_STATUS:
		if ((udccsr0 & (UDCCSR0_OPC | UDCCSR0_SA)) == UDCCSR0_OPC) {
			UDCCSR0 |= UDCCSR0_OPC;
		}
		WMMX_REGS(UDCCSR0,"  --> ep0 WAIT for STATUS");
		endpoint->state = WAIT_FOR_SETUP;

        case WAIT_FOR_SETUP:
		do {
			wmmx_ep0setup(endpoint, UDCCSR0);

			if (endpoint->state == DATA_STATE_PENDING_XMIT) {
				if (udccsr0 & UDCCSR0_SST) {
					wmmx_ep_reset_interrupt_status(0);
					UDCCSR0 = UDCCSR0_SST | 
						  UDCCSR0_OPC | 
						  UDCCSR0_SA;
					return;
				}
				break;
			}

			if (udccsr0 & UDCCSR0_SST) {
				wmmx_ep_reset_interrupt_status(0);
				UDCCSR0 |= UDCCSR0_SST |
					   UDCCSR0_OPC | 
					   UDCCSR0_SA;
				WMMX_REGS(udccsr0,"  --> ep0 clear SST");
				if (endpoint) {
					endpoint->state = WAIT_FOR_SETUP;
					endpoint->tx_urb = NULL;
					endpoint->rcv_urb = NULL;
				}
				return;
			}

			if (j++ > 2) {
				u32 udccsr0 = UDCCSR0;
                                WMMX_REGS(udccsr0,"  ep0 wait");
                                if ((udccsr0 & 
                                     (UDCCSR0_OPC | UDCCSR0_SA | UDCCSR0_RNE)) 
                                    == (UDCCSR0_OPC | UDCCSR0_SA)) {

                                        UDCCSR0 |= (UDCCSR0_OPC | UDCCSR0_SA);
                                        WMMX_REGS(UDCCSR0,"  ep0 force");
				} else {
					UDCCSR0 |= (UDCCSR0_OPC | UDCCSR0_SA);
					WMMX_REGS(UDCCSR0,"  ep0 force and return");
					break;
				}
			}
		} while (UDCCSR0 & (UDCCSR0_OPC | UDCCSR0_RNE));
		break;

	case DATA_STATE_XMIT:
		wmmx_ep0xmit(endpoint, UDCCSR0);
		break;

	case DATA_STATE_PENDING_XMIT:
		wmmx_ep0xmit(endpoint, UDCCSR0);

	case DATA_STATE_RECV:
		wmmx_ep0recv(endpoint, UDCCSR0);
		break;
        }

	wmmx_ep_reset_interrupt_status(0);
	WMMX_REGS(endpoint->state,"  -- endpoint state result");
	WMMX_REGS(UDCCSR0,"  <-- ep0");
}

static __inline__ int endpoints_have_interrupt(void)
{
	u32 udcisr0, udcisr1;

	udcisr0 = UDCISR0;
	udcisr1 = UDCISR1;

	if (udcisr0)
		return 1;

	if (udcisr1)
		return 1;

	return 0;
}

/* 
 *  Bulverde UDC Interrupt Handler
 */
static void wmmx_int_hndlr (int irq, void *dev_id, struct pt_regs *regs)
{
        u32 udcisr0, udcisr1;
        int udccr = UDCCR;
        int ep;

	udc_interrupts++;

	udcisr0 = UDCISR0;
	udcisr1 = UDCISR1 & 0xffff;

	while (udcisr0 || udcisr1) {
		u32 udccsr0 = UDCCSR0;
		WMMX_IRO(udcisr0, "------------------------> Interrupt");

		/* For Endpoint0, Endpoint A-P */
		for (ep = 0; udcisr0 != 0 && ep < 16 ; udcisr0 >>= 2, ep++) {
			if (udcisr0 & UDC_INT_PACKETCMP) {
				switch (ep_maps[ep].eptype) {
				case ep_control:
					wmmx_ep0(phys_ep_to_endpoints[0], 
						 udccsr0);
					break;

				case ep_bulk_in:
				case ep_interrupt:
					wmmx_in_n(ep, phys_ep_to_endpoints[ep]);
					break;

				case ep_bulk_out:
					while (UDCCSN(ep) & UDCCSR_PC) {
						wmmx_out_n(ep, 
							phys_ep_to_endpoints[ep]);
					}
					break;

				case ep_iso_in:
				case ep_iso_out:
					wmmx_ep_reset_interrupt_status(ep);
					break;
				}
			}
			if (udcisr0 & UDC_INT_FIFOERROR) {
				printk("FIFO error on %d endpoint\n", ep);
			}
		}

		/* For Endpoint Q-X */
		for ( ; udcisr1 != 0 && ep < 24; udcisr1 >>= 2, ep++) {

			/* I will rebuild ep_maps array */
			if (udcisr1 & UDC_INT_PACKETCMP) {
				switch (ep_maps[ep].eptype) {
				case ep_bulk_in:
				case ep_interrupt:
					wmmx_in_n(ep, phys_ep_to_endpoints[ep]);
					break;

				case ep_bulk_out:
					while (UDCCSN(ep) & UDCCSR_PC) {
						wmmx_out_n(ep, 
							phys_ep_to_endpoints[ep]);
					}
					break;

				case ep_iso_in:
				case ep_iso_out:
					wmmx_ep_reset_interrupt_status(ep);
					break;

				default:
					break;
				}
			}
		}
		udcisr0 = UDCISR0;
		udcisr1 = UDCISR1 & 0xffff;
	}

	/* Reset, Suspend, Resume interrupts */
	udcisr1 = UDCISR1;

	/* UDC Reset */
	if (udcisr1 & UDCISR1_IERS) {
		WMMX_CCR(udccr, "------------------------> Reset");

		udc_suspended = 0;
		usbd_device_event (udc_device, DEVICE_RESET, 0);
		usbd_device_event (udc_device, DEVICE_ADDRESS_ASSIGNED, 0);	
		UDCISR1 = UDCISR1_IERS;
	}

	/* UDC Resume */
	if (udcisr1 & UDCISR1_IERU) {
		WMMX_CCR(udccr, "------------------------> Resume");
		if (udc_suspended) {
			udc_suspended = 0;
			usbd_device_event (udc_device, DEVICE_BUS_ACTIVITY, 0);
		}
		UDCISR1 = UDCISR1_IERU;
	}

	/* UDC Suspend */
	if (udcisr1 & UDCISR1_IESU) {
		WMMX_CCR(udccr, "------------------------> Suspend");
		if (!udc_suspended) {
			udc_suspended = 1;
			usbd_device_event (udc_device, DEVICE_BUS_INACTIVE, 0);
		}
		UDCISR1 = UDCISR1_IESU;
	}
		
	/* Configuration Change Interrupt */
	if (udcisr1 & UDCISR1_IECC) {
		WMMX_CCR(udccr, "------------------------> Configuration "
				"Changed");

		UDCCR |= UDCCR_SMAC;
			
		udc_device->configuration = (UDCCR & UDCCR_ACN) >> UDCCR_ACN_S;
		udc_device->interface = (UDCCR & UDCCR_AIN) >> UDCCR_AIN_S;
		udc_device->alternate = (UDCCR & UDCCR_AAISN) >> UDCCR_AAISN_S;

		usbd_device_event(udc_device, DEVICE_CONFIGURED, 0); 

		UDCISR1 = UDCISR1_IECC;
	}
	WMMX_CCR(UDCCR, "<-- Interrupt");
}

/*
 * Setting the endpoint configuration register
 */
static int udc_set_endpoint(struct usb_endpoint_description* endpoint,
			    int configuration, int interface,
			    int alternate, int eps)
{
	u32 config_reg;
	static int phys_endpoints = 0;
	
	phys_endpoints ++;
	config_reg = ((configuration << UDCCONR_CN_S) & UDCCONR_CN) |
		     ((interface << UDCCONR_IN_S) & UDCCONR_IN) |
		     ((alternate << UDCCONR_AISN_S) & UDCCONR_AISN) |
		     ((eps << UDCCONR_EN_S) & UDCCONR_EN);
	
	logic_to_phys_eps[configuration][endpoint->bEndpointAddress & 0xf] = 
		phys_endpoints;

	/* Double Buffering Enabled and Enable the EP */
	config_reg |= UDCCONR_EE | UDCCONR_DE;
	
	if (endpoint->direction == IN) {
		config_reg |= UDCCONR_ED;	/* Direction: IN */
	} else {
		config_reg &= ~UDCCONR_ED;	/* Direction: OUT*/
	}

	config_reg |= ((endpoint->bmAttributes << UDCCONR_ET_S) & UDCCONR_ET) |
		      ((endpoint->wMaxPacketSize << UDCCONR_MPS_S) &
		       UDCCONR_MPS);

	UDCCN(phys_endpoints) = config_reg;

	ep_maps[phys_endpoints].logical = endpoint->bEndpointAddress;
	ep_maps[phys_endpoints].eptype = 
		(((config_reg & UDCCONR_ET) >> UDCCONR_ET_S) << 1) |
		 !!(config_reg & UDCCONR_ED);
	ep_maps[phys_endpoints].size = endpoint->wMaxPacketSize;

	return 0;
}

/*
 * Check the function driver and set UDC Endpoint A-X Configuration Registers
 */
static int udc_check_function(struct usb_function_instance* function)
{
	int total = 1;
	int i,j,k,m;
	struct usb_configuration_description* configuration;
	struct usb_interface_description* interface;
	struct usb_alternate_description* alternate;
	struct usb_endpoint_description* endpoint;

	if (function->function_driver->configurations > 
	    MAX_LOGICAL_CONFIGURATIONS) {
		printk(KERN_ERR"Too many configurations.\n");
		return -1;
	}

	for (i = 0; i < function->function_driver->configurations; i++) {
		configuration = function->function_driver->configuration_description + i;
		for (j = 0; j < configuration->interfaces; j++) {
			int logical_endpoints = 1;
			interface = configuration->interface_list + j;
			for (k = 0; k < interface->alternates; k++) {
				alternate = interface->alternate_list + k;
				for (m = 0; m < alternate->endpoints; m++) {
					if (alternate->endpoints > 
					    MAX_LOGICAL_ENDPOINTS) {
						printk(KERN_ERR "Too many logical endpoints.\n");
						return -1;
					}
					endpoint = alternate->endpoint_list + m;
					if (udc_set_endpoint(endpoint, i+1, 
							     j, k, 
							     logical_endpoints++) < 0)
					{
						printk(KERN_ERR "udc_set_endpoint() error.\n");
						return -1;
					}
					total++;
				}
			}
		}
	}

	return total;
}

/*
 * Enable UDC configuration setting
 */
static int udc_configuration_enable(struct usb_device_instance* device)
{
	struct usb_function_instance* function;
	int count, i;
	
	count = i = 0;

	function = device->function_instance_array + i;
	if (!function) {
		printk(KERN_ERR"No valid function driver!\n");
		return -1;
	}
	count = udc_check_function(function);
	if (count < 0) {
		printk(KERN_ERR"udc_check_function(...) failed.\n");
		return -1;
	}

	return 0;
}

/*
 * Public Functions.
 */

/*
 * udc_start_in_irq - start transmit
 * @eendpoint: endpoint instance
 *
 * Called by bus interface driver to see if we need to 
 * start a data transmission.
 */
void udc_start_in_irq (struct usb_endpoint_instance *endpoint)
{
	int phys_ep = wmmx_logic_to_phys_ep(endpoint->endpoint_address & 0xf);
	
	if (endpoint->endpoint_address == 0) {
		struct urb* urb = endpoint->tx_urb;
		if (!urb) {
			printk(KERN_INFO "udc_start_in_irq() urb is NULL\n");
			return;
		}
		wmmx_ep0xmit(endpoint, UDCCSR0);
	} else if (UDCCSN(phys_ep) & UDCCSR_FS) {
			wmmx_start_n(phys_ep, endpoint);
	}
}

/*
 * udc_init - initialize
 *
 * Return non-zero if we cannot see device.
 */
int udc_init (void)
{
	int i;

	dbg_tx(6, "udc_init:\n");

	udc_disable_interrupts (NULL);

	return 0;
}

/*
 * udc_stall_ep - stall endpoint
 * @ep: physical endpoint
 *
 * Stall the endpoint.
 */
void udc_stall_ep (unsigned int phys_ep)
{
	dbg_usbe(5, "usc_stall_ep ep=%d\n", phys_ep);
	if (phys_ep < UDC_MAX_ENDPOINTS) {
		UDCCSN(phys_ep) |= UDCCSR_FST;
	}
}

/*
 * udc_reset_ep - reset endpoint
 * @ep: physical endpoint
 * reset the endpoint.
 *
 * returns : 0 if ok, -1 otherwise
 */
void udc_reset_ep (unsigned int phys_ep)
{
	if (phys_ep == 0) {
		UDCCSN(phys_ep) |= ((UDCCSN(phys_ep) & UDCCSR_DME) | 
				    UDCCSR_FEF);
	}
}

/*
 * udc_endpoint_halted - is endpoint halted
 * @ep:
 *
 * Return non-zero if endpoint is halted
 */
int udc_endpoint_halted (unsigned int phys_ep)
{
	return 0;
}

/*
 * udc_set_address - set the USB address for this device
 * @address:
 *
 * Called from control endpoint function after it decodes 
 * a set address setup packet.
 */
void udc_set_address (unsigned char address)
{
}

/*
 * udc_serial_init - set a serial number if available
 */
int __init udc_serial_init (struct usb_bus_instance *bus)
{
	return -EINVAL;
}

/*
 * udc_max_endpoints - max physical endpoints 
 *
 * Return number of physical endpoints.
 */
int udc_max_endpoints (void)
{
	return UDC_MAX_ENDPOINTS;
}

/*
 * udc_check_ep - check logical endpoint 
 * @lep:
 *
 * Return physical endpoint number to use for this logical 
 * endpoint or zero if not valid.
 */
int udc_check_ep(int logic_ep, int packetsize)
{
	if (packetsize > 64)
		return 0;
	return wmmx_logic_to_phys_ep(logic_ep & 0x0f);
}


/*
 * udc_set_ep - setup endpoint 
 * @ep:
 * @endpoint:
 *
 * Associate a physical endpoint with endpoint_instance
 */
void udc_setup_ep (struct usb_device_instance *device, 
		   unsigned int phys_ep,
		   struct usb_endpoint_instance *endpoint)
{
	if (phys_ep < UDC_MAX_ENDPOINTS) {
		phys_ep_to_endpoints[phys_ep] = endpoint;

		if (!phys_ep) {
			printk(KERN_DEBUG "udc_setup_ep: 0 do nothing\n");
			wmmx_enable_ep_interrupt(0, (UDC_INT_FIFOERROR | 
						     UDC_INT_PACKETCMP)); 
		} else if (endpoint->endpoint_address & 0x80) { /* IN */
			/* nothing to do for IN (tx) endpoints */
				wmmx_enable_ep_interrupt(phys_ep, 
							 (UDC_INT_FIFOERROR |
							  UDC_INT_PACKETCMP));
		} else {	/* OUT (rx) endpoint */
			if (endpoint->rcv_packetSize) {
				/* Why are we pre-allocating 5 urbs?  Because 
				 * most of the other existing USB controller 
				 * drivers pre-allocate 5 urbs.  We must 
				 * allocate enough urbs so that we never run 
				 * out of urbs in our endpoint->rdy queue 
				 * because the function driver hasn't had a 
				 * chance to process an urb from the 
				 * endpoint->rcv queue and recycle it back to 
				 * the rdy queue.
				 */
				usbd_fill_rcv (device, endpoint, 5);
				endpoint->rcv_urb = 
					first_urb_detached (&endpoint->rdy);
				wmmx_enable_ep_interrupt(phys_ep, 
							 (UDC_INT_FIFOERROR |
							  UDC_INT_PACKETCMP));
			}
		}
	}
}

/*
 * udc_disable_ep - disable endpoint
 * @ep:
 *
 * Disable specified endpoint 
 */
void udc_disable_ep (unsigned int phys_ep)
{
	dbg_init(1, "udc_disable_ep: disable endpoint %d", phys_ep);
	/* if the endpoint use PIO mode, we should disable the endpoint interrupts too. */
	if (phys_ep < UDC_MAX_ENDPOINTS) {
		struct usb_endpoint_instance *endpoint;

		dbg_init(1, "endpoint descr 0x%08x", 
			 phys_ep_to_endpoints[phys_ep]);
		if ((endpoint = phys_ep_to_endpoints[phys_ep])) {
			phys_ep_to_endpoints[phys_ep] = NULL;
			usbd_flush_ep (endpoint);
		} else {
			return;
		}
	}
}

static int udc_connected_status;

/*
 * udc_connected - is the USB cable connected
 *
 * Return non-zeron if cable is connected.
 */
int udc_connected (void)
{
	return udc_connected_status;
}

/*
 * udc_connect - enable pullup resistor
 *
 * Turn on the USB connection by enabling the pullup resistor.
 */
void udc_connect (void)
{
#ifdef CONFIG_ARCH_MAINSTONE
	if (machine_is_mainstone())
		MST_MSCWR2 &= ~MST_MSCWR2_nUSBC_SC;
#endif
	udc_connected_status = 1;
}

/*
 * udc_disconnect - disable pullup resistor
 *
 * Turn off the USB connection by disabling the pullup resistor.
 */
void udc_disconnect (void)
{
#ifdef CONFIG_ARCH_MAINSTONE
	if (machine_is_mainstone())
		MST_MSCWR2 |= MST_MSCWR2_nUSBC_SC;
#endif
	udc_connected_status = 0;
}

/*
 * udc_enable_interrupts - enable interrupts
 *
 * Switch on UDC interrupts.
 *
 */
void udc_all_interrupts (struct usb_device_instance *device)
{
	int i;

	UDCICR1 = (UDCICR1_IESU | 
		   UDCICR1_IERU | 
		   UDCICR1_IERS | 
		   UDCICR1_IECC) & ~UDCICR1_IESOF;

	/* always enable control endpoint */
	wmmx_enable_ep_interrupt(0, /*UDC_INT_FIFOERROR |*/ UDC_INT_PACKETCMP);

	for (i = 1; i < UDC_MAX_ENDPOINTS; i++) {
		if (phys_ep_to_endpoints[i] && 
		    phys_ep_to_endpoints[i]->endpoint_address) {
                        wmmx_enable_ep_interrupt(i, 
						 UDC_INT_FIFOERROR |
						 UDC_INT_PACKETCMP);
		}
	} 
}

/*
 * udc_suspended_interrupts - enable suspended interrupts
 *
 * Switch on only UDC resume interrupt.
 *
 */
void udc_suspended_interrupts (struct usb_device_instance *device)
{
	UDCICR1 = (UDCICR1 | UDCICR1_IERS | UDCICR1_IERU | 
		   UDCICR1_IECC | UDCICR1_IESU) & ~UDCICR1_IESOF;
}

/*
 * udc_disable_interrupts - disable interrupts.
 *
 * switch off interrupts
 */
void udc_disable_interrupts (struct usb_device_instance *device)
{
        /* disable sof interrupt */
        /* disable suspend and resume interrupt */
	/* disable endpoint interrupts */
	UDCICR0 = UDCICR1 = 0x00000000;
}

/*
 * udc_ep0_packetsize - return ep0 packetsize
 */
int udc_ep0_packetsize (void)
{
	return EP0_PACKETSIZE;
}

/*
 * udc_enable - enable the UDC
 *
 * Switch on the UDC
 */
void udc_enable (struct usb_device_instance *device)
{
	udc_device = device;

	dbgENTER(dbgflg_usbdbi_init, 1);
	if (!ep0_urb) {
		ep0_urb = usbd_alloc_urb(device, 
					 device->function_instance_array, 
					 0, 512);
		if (!ep0_urb) {
			printk(KERN_ERR "udc_enable: usbd_alloc_urb failed\n");
		}
	} else {
		printk (KERN_ERR "udc_enable: ep0_urb already allocated\n");
	}

	/* enable UDC clock */
	CKEN |= CKEN11_USB;

	/* enable UDC controller */
	UDCCR = UDCCR_UDE;

	/* 
	 * If UDCCR_EMCE is set, we should reduce the number of the 
	 * configuration automatically or just yelled. 
	 */
	if ((UDCCR & UDCCR_EMCE) == UDCCR_EMCE) {
		printk(KERN_ERR "udc_enable: Endpoint memory configure "
				"error\n");
		UDCCR = UDCCR_EMCE;
	}
}

/*
 * udc_disable - disable the UDC
 *
 * Switch off the UDC
 */
void udc_disable (void)
{
	int i;
	
	dbgENTER(dbgflg_usbdbi_init, 1);
	/* disable UDC controller */
	UDCCR &= ~(UDCCR_UDE);

	/* disable UDC clock */
	CKEN &= ~CKEN11_USB;

	/* reset device pointer */
	udc_device = NULL;

	/* ep0 urb */
	if (ep0_urb) {
		usbd_dealloc_urb (ep0_urb);
		ep0_urb = 0;
	} else
		printk (KERN_ERR "udc_disable: ep0_urb already NULL\n");
}

/*
 * udc_startup_events - allow udc code to do any additional startup
 */
void udc_startup_events (struct usb_device_instance *device)
{
	dbgENTER(dbgflg_usbdbi_init, 1);
	udc_configuration_enable(device);

	usbd_device_event(device, DEVICE_INIT, 0);
	usbd_device_event(device, DEVICE_CREATE, 0);
	usbd_device_event(device, DEVICE_HUB_CONFIGURED, 0);
	usbd_device_event(device, DEVICE_RESET, 0);
}

/* Proc Filesystem */
#ifdef WMMX_TRACE
#ifdef CONFIG_USBD_PROCFS
/*    
 * wmmx_proc_read - implement proc file system read.
 * @file        
 * @buf         
 * @count
 * @pos 
 *      
 * Standard proc file system read function.
 */         
static ssize_t wmmx_proc_read(struct file *file, char *buf, 
			      size_t count, loff_t * pos)
{                                  
	unsigned long page;
	int len = 0;
	int index;
	int i;

	MOD_INC_USE_COUNT;
	/* get a page, max 4095 bytes of data... */
	if (!(page = get_free_page (GFP_KERNEL))) {
		MOD_DEC_USE_COUNT;
		return -ENOMEM;
	}

	len = 0;
	index = (*pos)++;

	if (index == 0) {
		len += sprintf (
			(char *) page + len, 
			"   Index     Ints     Jifs    Ticks\n");
        }       

	if (index < trace_next) {
		u64 jifs = 1111;
		u32 ticks = 2222;
		wmmx_trace_t *p = wmmx_traces + index;
		unsigned char *cp;

		if (index > 0) {
			u32 ocsr = wmmx_traces[index-1].ocsr;
			ticks = (p->ocsr > ocsr) ? 
				(p->ocsr - ocsr) : 
				(ocsr - p->ocsr);
			jifs = p->jiffies - wmmx_traces[index-1].jiffies;
		}
                
		len += sprintf((char *)page + len, "%8d %8d %8llu ", 
			       index, p->interrupts, jifs);

		if (ticks > 1024 * 1024) {
			len += sprintf((char *)page + len, "%8dM ", 
				       ticks>>20);
		} else {
			len += sprintf((char *)page + len, "%8d  ", ticks);
		}
		switch (p->trace_type) {
		case wmmx_regs:
			len += sprintf((char *) page + len, 
				       "CS0[%02x] %s\n", 
				       p->trace.regs.cs0, p->trace.regs.msg);
			break;

		case wmmx_setup:
			cp = (unsigned char *)&p->trace.setup;
			len += sprintf((char *)page + len, 
				       " --               request "
				       "[%02x %02x %02x %02x %02x %02x %02x "
				       "%02x]\n", 
				       cp[0], cp[1], cp[2], cp[3], cp[4], 
				       cp[5], cp[6], cp[7]);
			break;

		case wmmx_xmit:
			len += sprintf((char *) page + len, 
				       " --                   sending [%02x]\n",
				       p->trace.xmit.size);
			break;

		case wmmx_ccr:
			len += sprintf((char *) page + len, 
				       "CCR[%02x] %s\n", 
				       p->trace.ccr.ccr, p->trace.ccr.msg);
			break;

		case wmmx_iro:
			len += sprintf((char *) page + len, 
				       "IRO[%02x] %s\n", 
				       p->trace.iro.iro, p->trace.iro.msg);
			break;

		case wmmx_xfer:
			cp = (unsigned char *)&p->trace.xfer.data;
			len += sprintf((char *)page + len,
				       "%s xfer of %d bytes:",
				       (p->trace.xfer.dir ==
					WMMX_XFER_DIR_OUT) ? "OUT" : "IN",
				       p->trace.xfer.size);
			for (i = 0;
			     i < MIN(p->trace.xfer.size, WMMX_XFER_MAX_TRACE_SIZE);
			     i++)
			{
				len += sprintf((char *)page + len, " %02x",
					       (p->trace.xfer.data[i]) & 0xff);
			}
			len += sprintf((char *)page + len, "\n");
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

/*
 * wmmx_proc_write - implement proc file system write.
 * @file
 * @buf
 * @count
 * @pos
 *
 * Proc file system write function, used to signal monitor actions complete.
 * (Hotplug script (or whatever) writes to the file to signal the completion
 * of the script.)  An ugly hack.
 */
static ssize_t wmmx_proc_write(struct file *file, const char *buf, 
			       size_t count, loff_t * pos)
{
	return count;
}

static struct file_operations wmmx_proc_operations_functions = 
{
	read:wmmx_proc_read,
	write:wmmx_proc_write,
};
#endif
#endif

/*
 * udc_name - return name of USB Device Controller
 */
char *udc_name (void)
{
	return UDC_NAME;
}

/*
 * udc_request_udc_irq - request UDC interrupt
 *
 * Return non-zero if not successful.
 */
int udc_request_udc_irq ()
{
	dbgENTER(dbgflg_usbdbi_init, 1);
	if (request_irq(IRQ_USB, wmmx_int_hndlr, 
			SA_INTERRUPT | SA_SAMPLE_RANDOM,
			UDC_NAME " USBD Bus Interface", NULL) != 0) {

		printk (KERN_INFO "usb_ctl: Couldn't request USB irq\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * udc_request_cable_irq - request Cable interrupt
 *
 * Return non-zero if not successful.
 */
int udc_request_cable_irq()
{
	return 0;
}

/*
 * udc_request_udc_io - request UDC io region
 *
 * Return non-zero if not successful.
 */
int udc_request_io ()
{
#ifdef WMMX_TRACE
#ifdef CONFIG_USBD_PROCFS
	if (!(wmmx_traces = vmalloc(sizeof(wmmx_trace_t) * TRACE_MAX))) {
		printk(KERN_ERR "WMMX_TRACE malloc failed %p %d\n", 
		       wmmx_traces, 
		       sizeof(wmmx_trace_t) * TRACE_MAX);
	} else {
		printk(KERN_ERR "WMMX_TRACE malloc ok %p\n", wmmx_traces);
        }
	WMMX_REGS(0,"init");
	WMMX_REGS(0,"test");
	{
		struct proc_dir_entry *p;

		if ((p = create_proc_entry("wmmx", 0, 0)) == NULL) {
			printk(KERN_INFO"WMMX PROC FS failed\n");
		} else {
			printk(KERN_INFO"WMMX PROC FS Created\n");
			p->proc_fops = &wmmx_proc_operations_functions;
		}
	}
#endif
#endif
	return 0;
}

/*
 * udc_release_release_io - release UDC io region
 */
void udc_release_io ()
{
#ifdef WMMX_TRACE
#ifdef CONFIG_USBD_PROCFS
	unsigned long flags;
	dbgENTER(dbgflg_usbdbi_init, 1);
	save_flags_cli (flags);
	remove_proc_entry ("wmmx", NULL);
	if (wmmx_traces) {
		wmmx_trace_t *p = wmmx_traces;
		wmmx_traces = 0;
		vfree(p);
		dbg_init(1, "trace memory is released");
	}
	restore_flags (flags);
#endif
#endif
}

/*
 * udc_release_udc_irq - release UDC irq
 */
void udc_release_udc_irq ()
{
	dbgENTER(dbgflg_usbdbi_init, 1);
	free_irq (IRQ_USB, NULL);
}

/*
 * udc_release_cable_irq - release Cable irq
 */
void udc_release_cable_irq ()
{
}

/*
 * udc_regs - dump registers
 *
 * Dump registers with printk
 */
void udc_regs (void)
{
	printk ("[%d:%d] CCR[%08x] UDUICR[%08x %08x] UFN[%08x] UDCCS[%08x]\n", 
		udc_interrupts, jifs(), 
		UDCCR, 
		UDCICR0, UDCICR1, 
		UDCFNR, 
		UDCCSR0);
}

void wmmx_print(void)
{
	printk("%08x %08x\n", UDCCSN(1), UDCCSN(2));
}
