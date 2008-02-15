/*
 * sl11_bi/udc.c
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *      Bruce Balden <balden@lineo.com>
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


/*****************************************************************************/

#include <linux/config.h>
#include <linux/module.h>

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-module.h"

MODULE_AUTHOR ("sl@lineo.com, tbr@lineo.com");
MODULE_DESCRIPTION ("USB Device SL11 Bus Interface");

USBD_MODULE_INFO ("sl11_bi 0.1-alpha");

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <asm/atomic.h>
#include <asm/io.h>

#include <linux/netdevice.h>

#include <asm/irq.h>
#include <asm/system.h>

#include <asm/types.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/delay.h>

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-inline.h"
#include "usbd-bi.h"

#include "sl11.h"
#include "sl11-info.h"

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

/*
 * ep_address - endpoint buffer addresses
 *
 * 0-3  epNa
 * 4-7  epNb
 */
static unsigned char ep_address[UDC_MAX_ENDPOINTS * 2] = {
	EP_S_BUF, EP_S_BUF, EP_S_BUF, EP_S_BUF,
	EP_S_BUF, EP_S_BUF, EP_S_BUF, EP_S_BUF
};

/*
 * ep_register - endpoint register addresses
 *
 * 0-3  epNa
 * 4-7  epNb
 */
static unsigned char ep_register[UDC_MAX_ENDPOINTS * 2] = {
	EP0_Control, EP1_Control, EP2_Control, EP3_Control,
	EP0_Control + 8, EP1_Control + 8, EP2_Control + 8, EP3_Control + 8
};

static struct usb_device_instance *udc_device;	// required for the interrupt handler

/*
 * ep_sequence - current sequence number for each endpoint
 */
static unsigned char ep_sequence[UDC_MAX_ENDPOINTS] = {
	0, 0, 0, 0
};

/*
 * ep_next - next ping pong buffer
 */
static unsigned char ep_next[UDC_MAX_ENDPOINTS] = {
	0, 0, 0, 0
};

/*
 * ep_int_mask - interrupt status register endpoint bit masks
 */
static unsigned char ep_int_mask[UDC_MAX_ENDPOINTS] = {
	INT_EP0_DONE, INT_EP1_DONE, INT_EP2_DONE, INT_EP3_DONE
};

/*
 * ep_endpoints - map physical endpoints to logical endpoints
 */
static struct usb_endpoint_instance *ep_endpoints[UDC_MAX_ENDPOINTS];

// static struct urb ep0_urb;
static struct urb *ep0_urb;
static unsigned char usb_address;

static int udc_saw_sof;
static int udc_suspended;
static int udc_addressed;
static struct tq_struct sl11_tq;

extern unsigned int udc_interrupts;
unsigned int udc_ticks;

/* ********************************************************************************************* */

#ifdef CONFIG_X86
/**
 * sl11write_byte - write a byte to the sl11
 * @a: sl11 address
 * @b: byte to write
 */
static __inline__ void sl11write_byte (unsigned char a, unsigned char b)
{
	outb (a, UDC_ADDR);
	outb (b, UDC_ADDR + 1);
}

static __inline__ void _sl11write_byte (unsigned char a, unsigned char b)
{
	printk (KERN_DEBUG "sl11write_byte: Reg: %02x Val: %02x\n", a, b);
	outb (a, UDC_ADDR);
	outb (b, UDC_ADDR + 1);
}


/**
 * sl11write_buffer - write a buffer to the sl11 using auto-increment mode
 * @a: sl11 address
 * @b: pointer to buffer to write
 * @size: number of bytes to write
 */
static __inline__ void sl11write_buffer (unsigned char a, unsigned char *b, unsigned char size)
{
	outb (a, UDC_ADDR);
	while (size--) {
		outb (*b++, UDC_ADDR + 1);
	}
}


/**
 * sl11read_byte - read a byte from the sl11 and return
 * @a: sl11 address
 */
static __inline__ unsigned char sl11read_byte (unsigned char a)
{
	outb (a, UDC_ADDR);
	return inb (UDC_ADDR + 1);
}


/**
 * sl11read_buffer - fill a buffer from the sl11 using auto-increment mode
 * @a: sl11 address
 * @b: pointer to buffer to fill
 * @size: number of bytes to read
 */
static __inline__ void sl11read_buffer (unsigned char a, unsigned char *b, unsigned char size)
{
	outb (a, UDC_ADDR);
	while (size--) {
		*b++ = inb (UDC_ADDR + 1);
	}
}

/**
 * sl11clear - clear sl11 memory
 * @a: sl11 address
 * @size: bytes to clear
 */
static void sl11clear (unsigned char a, unsigned char size)
{
	outb (a, UDC_ADDR);
	while (size--) {
		outb (0, UDC_ADDR + 1);
	}
}


#else
#abort SL11 memory mapped IO not implemented
#endif

/* ********************************************************************************************* */


/**
 * sl11write_epn - write a byte to a sl11 endpoint register
 * @a: endpoint 
 * @b: endpoint register
 * @d: byte to write
 */
static __inline__ void sl11write_epn (unsigned char ep, unsigned char b, unsigned char c)
{
	sl11write_byte (ep_register[ep] + b, c);
}

static __inline__ void _sl11write_epn (unsigned char ep, unsigned char b, unsigned char c)
{
	printk (KERN_DEBUG "sl11write_epn: Reg: %02x Val: %02x\n", ep_register[ep] + b, c);
	_sl11write_byte (ep_register[ep] + b, c);
}


/**
 * sl11read_epn - read a byte from a sl11 endpoint register
 * @a: endpoint 
 * @b: endpoint register
 * @d: byte to write
 */
__inline__ unsigned char sl11read_epn (unsigned char ep, unsigned char b)
{
	return sl11read_byte (ep_register[ep] + b);
}

/* ********************************************************************************************* */

/**
 * sl11_dump
 */
static void sl11_dump (unsigned char a, unsigned char size)
{
	int i;
	for (i = 0; i < size; i++) {
		if ((i % 8) == 0) {
			printk ("\n[%02x]: ", a + i);
		}
		printk ("%02x ", sl11read_byte (a + i));
	}
	printk ("\n");
}

static void sl11_dump_epn (unsigned char ep, char *m)
{
	printk ("ep[%d]: Ctl:%02x Adr:%02x Xfr:%02x Sts:%02x Cnt:%02x Seq:%d%s", ep,
		sl11read_epn (ep, EPN_Control),
		sl11read_epn (ep, EPN_Address),
		sl11read_epn (ep, EPN_XferLen),
		sl11read_epn (ep, EPN_Status),
		sl11read_epn (ep, EPN_Counter), ep_sequence[ep & 0x3], m);
}


/**
 * sl11_probe - initialize
 *
 * 0. Issue a reset to CtrlReg
 *
 * 1. After reset we typically see:
 *
 *          IntStatus       0x40 or 0x60
 *          DATASet         0x10 
 *
 * 2. We test that IntStatus can be changed to 0x00 by writing 0xff to it.
 *
 * 3. If successful clear all chip memory.
 *
 **/
static int sl11_probe (void)
{
	sl11_dump (0, 64);
	printk (KERN_DEBUG "sl11_probe: start\n");

	// reset
	sl11write_byte (CtrlReg, CTRL_USB_RESET);
	udelay (100);		// XXX may not be needed
	sl11write_byte (CtrlReg, 0);

	sl11_dump (0, 64);
	printk (KERN_DEBUG "sl11_probe: RESET\n");

	/*
	 * XXX
	 *
	 * If device is present DATASet&CDS_RESERVED will be true.
	 *
	 * After reset we will see IntStatus & INT_USB_RESET if cable is plugged in.
	 *
	 */

	if (((sl11read_byte (IntStatus) & INT_USB_RESET) != INT_USB_RESET)
	    || ((sl11read_byte (DATASet) & 0x10) != 0x10)) {
		sl11_dump (0, 64);
		printk (KERN_DEBUG "sl11_probe: cannot see " UDC_NAME ", failed initial probe\n");
		//udc_regs();
		//return -EINVAL;
	}
	// reset interrupt status
	sl11write_byte (IntStatus, 0xff);
	if ((sl11read_byte (IntStatus) != 0x00)) {
		sl11_dump (0, 64);
		printk (KERN_DEBUG "sl11_probe: cannot see " UDC_NAME
			", failed interrupt status reset\n");
		udc_regs ();
		return -EINVAL;
	}
	// success!

	// clear all memory
	sl11clear (0, 255);


	printk (KERN_DEBUG "sl11_probe: found and setup\n");

	//sl11_dump(0, 64);
	//udc_regs();

	return 0;
}

/* ********************************************************************************************* */

/**
 * sl11send_data - send packet via endpoint
 * @ep: logical endpoint number
 * @bp: pointer to data
 * @size: bytes to write
 */
static void __inline__ sl11send_data (unsigned char ep, unsigned char *bp, unsigned char size)
{

	// copy data from buffer to chip
	if (bp && size) {
		sl11write_buffer (ep_address[ep], bp, size);
	}

	sl11write_epn (ep, EPN_XferLen, size);

	// arm
	sl11write_epn (ep, EPN_Control,
		       EPN_CTRL_ARM | EPN_CTRL_ENABLE | EPN_CTRL_DIRECTION_IN |
		       (ep_sequence[ep] ? EPN_CTRL_SEQUENCE : 0)
	    );
}


/**
 * s111send_done
 * @ep
 */
static void __inline__ sl11send_done (unsigned ep)
{
	// disarm
	sl11write_epn (ep, EPN_Control, EPN_CTRL_ENABLE | EPN_CTRL_DIRECTION_IN);

	// flip sequence
	ep_sequence[ep] = (~ep_sequence[ep]) & 0x1;
}

/* ********************************************************************************************* */

/**
 * s111read_init
 * @ep: endpoint
 */
static void sl11read_init (unsigned char ep, unsigned char size)
{
	// setup packet size
	sl11write_epn (ep, EPN_XferLen, size);

	// arm
	sl11write_epn (ep, EPN_Control, EPN_CTRL_ARM | EPN_CTRL_ENABLE | EPN_CTRL_DIRECTION_OUT);

}


/**
 * sl11read_done
 * @ep: endpoint
 */
static void sl11read_done (unsigned char ep, unsigned char *bp, unsigned char size)
{
	// disarm
	sl11write_epn (ep, EPN_Control, EPN_CTRL_ENABLE | EPN_CTRL_DIRECTION_OUT);

	// copy data from chip to buffer
	sl11read_buffer (ep_address[ep], bp, size);

}

/* ********************************************************************************************* */

/**
 * sl11_out - process rcv interrupt
 *
 * Queue received data.
 */
static void __inline__ sl11_out (unsigned int ep, struct usb_endpoint_instance *endpoint)
{
	int len;
	int cnt;
	int error = 0;
	int adr;
	int i;
	unsigned char status;
	unsigned char ctl;
	unsigned int cep;


	// loop alternating between A-B register sets looking for ACK in packet status
	//
	for (i = 0; i < 8; i++) {

		cep = ep + (ep_next[ep] ? 4 : 0);
		ep_next[ep] = ~ep_next[ep] & 0x1;

		status = sl11read_epn (cep, EPN_Status);

		//printk(KERN_DEBUG"[%d] out[%d]: seq: %d next: %d sts: %02x\n", udc_interrupts, ep, ep_sequence[ep], ep_next[ep], status);

		if (!(status & EPN_PSTS_ACK)) {
			continue;	//return;
		}
		// check packet status bits for errors
		ctl = sl11read_epn (cep, EPN_Control);
		len = sl11read_epn (cep, EPN_XferLen);
		cnt = sl11read_epn (cep, EPN_Counter);
		adr = sl11read_epn (cep, EPN_Address);

		//printk(KERN_DEBUG"[%d] out[%d]: sts: %02x len: %d cnt: %d adr: %02x seq: %02x bytes: %0d\n", 
		//        udc_interrupts, cep, status, len, cnt, adr, ep_sequence[ep], len - cnt);

		//sl11_dump_epn(ep, " ");
		//sl11_dump_epn(ep + 4, "\n");

		len -= cnt;

		if (status &
		    (EPN_PSTS_ERROR | EPN_PSTS_TIMEOUT | EPN_PSTS_SETUP | EPN_PSTS_OVERFLOW)) {
			printk (KERN_DEBUG "sl11_out[%d]: ERROR\n", cep);
			error = 1;
		}

		if (((status & EPN_PSTS_SEQUENCE) && ep_sequence[ep]) ||
		    (!(status & EPN_PSTS_SEQUENCE) && !ep_sequence[ep])) {
			ep_sequence[ep] = (~ep_sequence[ep]) & 0x1;
		} else {
			// sequence error
			printk (KERN_DEBUG "sl11_out[%d]: SEQ\n", cep);
			error = 1;
		}

		if (endpoint->rcv_urb && len) {
			sl11read_buffer (adr,
					 endpoint->rcv_urb->buffer +
					 endpoint->rcv_urb->actual_length, len);
		}
		// re-arm
		sl11write_epn (cep, EPN_Status, 0);
		sl11write_epn (cep, EPN_Counter, 0);
		sl11write_epn (cep, EPN_XferLen, endpoint->rcv_packetSize);
		//sl11_dump_epn(ep, "\n");
		sl11write_epn (cep, EPN_Control, ctl | EPN_CTRL_ARM);

		if (endpoint->endpoint_address) {
			//printk(KERN_DEBUG"sl11_out: receive len: %d\n", len);
			usbd_rcv_complete_irq (endpoint, len, 0);
		} else {
			printk (KERN_DEBUG "sl11_out: cannot receive no endpoint address len: %d\n",
				len);

		}
	}
}

/* ********************************************************************************************* */

/**
 * sl11_start - start transmit
 * @ep:
 */
static void __inline__ sl11_start (unsigned int ep, struct usb_endpoint_instance *endpoint,
				   int restart)
{
	//printk(KERN_DEBUG"sl11_start: tx_urb: %p sent: %d\n", endpoint->tx_urb, endpoint->sent);

	if (endpoint->tx_urb) {
		struct urb *urb = endpoint->tx_urb;

		//printk(KERN_DEBUG"sl11_start: length: %d\n", endpoint->tx_urb->actual_length);

		if ((urb->actual_length - endpoint->sent) > 0) {
			endpoint->last =
			    MIN (urb->actual_length - endpoint->sent, endpoint->tx_packetSize);
			sl11send_data (ep, urb->buffer + endpoint->sent, endpoint->last);
		} else {
			// XXX ZLP
			endpoint->last = 0;
			sl11send_data (ep, urb->buffer + endpoint->sent, 0);
		}
	}
}


/**
 * sl11_in - process tx interrupt
 * @ep:
 * @endpoint:
 *
 * Determine status of last data sent, queue new data.
 */
static __inline__ void sl11_in (unsigned int ep, struct usb_endpoint_instance *endpoint)
{
	int restart = 0;
	unsigned char status;

	restart = 0;

	//printk(KERN_DEBUG"in[%d]: seq: %d next: %d\n", ep, ep_sequence[ep], ep_next[ep]);

	// check packet status bits for errors
	status = sl11read_epn (ep, EPN_Status);

	if (status & (EPN_PSTS_ERROR | EPN_PSTS_TIMEOUT | EPN_PSTS_SETUP | EPN_PSTS_OVERFLOW)) {
		printk (KERN_DEBUG "sl11_in[%d]: ERROR status: %0x\n", ep, status);
		restart = 1;
	} else {
		// flip sequence
		ep_sequence[ep] = (~ep_sequence[ep]) & 0x1;
	}

	usbd_tx_complete_irq (endpoint, restart);
	sl11_start (ep, endpoint, restart);

}


/* ********************************************************************************************* */

static void sl11_ep0_setaddress (struct usb_endpoint_instance *endpoint)
{
	//printk(KERN_DEBUG"sl11_ep0: finished set address: %d\n", usb_address);
	sl11write_byte (USBAdd, usb_address);
	udc_addressed = usb_address;
	usb_address = 0;
	// send ack
	sl11write_epn (0, EPN_XferLen, endpoint->rcv_packetSize);
	sl11write_epn (0, EPN_Control, EPN_CTRL_ENABLE | EPN_CTRL_ARM);
	printk ("\n");
	sl11_dump_epn (0, "\n");
}

static void sl11_ep0_ending (struct usb_endpoint_instance *endpoint)
{


	//ep_sequence[0] = ~ep_sequence[0];

	if (endpoint->tx_urb) {
		//printk(KERN_DEBUG"sl11_ep0: continue IN actual: %d sent: %d last: %d seq: %d\n", 
		//        endpoint->tx_urb->actual_length, endpoint->sent, endpoint->last, ep_sequence[0]);
		sl11_in (0, endpoint);
	}

	if (endpoint->tx_urb) {
		//printk(KERN_DEBUG"sl11_ep0: still sending\n");
		return;
	}
	//printk(KERN_DEBUG"sl11_ep0: finished sending\n");

	// send ack
	sl11write_epn (0, EPN_XferLen, endpoint->rcv_packetSize);
	sl11write_epn (0, EPN_Control, EPN_CTRL_ENABLE | EPN_CTRL_ARM);
}

static void sl11_ep0_receiving (struct usb_endpoint_instance *endpoint)
{

	// XXX check sequence
	//ep_sequence[ep] = ~ep_sequence[ep];

	// are we trying to read data?
	if (endpoint->rcv_urb) {

		// ok, handle incoming data
		sl11_out (0, endpoint);

		// more to send?
		if (endpoint->rcv_urb) {
			return;
		}
		// send ack
		sl11write_epn (0, EPN_XferLen, 0);
		sl11write_epn (0, EPN_Control, EPN_CTRL_ENABLE | EPN_CTRL_ARM);
		return;
	}
	//printk(KERN_DEBUG"sl11_ep0: received ACK\n");

	sl11write_epn (0, EPN_XferLen, endpoint->rcv_packetSize);
	sl11write_epn (0, EPN_Control, EPN_CTRL_ENABLE | EPN_CTRL_ARM);

	// stall if we where not waiting for data
	//udc_stall_ep(0);
}

/**
 * sl11_ep0 - process an endpoint 0 interrupt
 *
 * Read and process a setup packet
 */
static void sl11_ep0 (struct usb_endpoint_instance *endpoint)
{
	unsigned char control;
	unsigned char status;
	unsigned char *cp;


	//sl11_dump_epn(0, "\n");

	// delayed set address, cannot set until we have received IN
	if (usb_address) {
		sl11_ep0_setaddress (endpoint);
		return;
	}
	// are we sending a reply
	control = sl11read_epn (0, EPN_Control);

	if (control & EPN_CTRL_DIRECTION_IN) {
		sl11_ep0_ending (endpoint);
		return;
	}

	status = sl11read_epn (0, EPN_Status);

	// is the host sending data?
	if (!(status & EPN_PSTS_SETUP)) {
		sl11_ep0_receiving (endpoint);
		return;
	}

	// check if host changed it's mind
	if (endpoint->tx_urb) {
		endpoint->tx_urb = NULL;
		endpoint->last = endpoint->sent = 0;
		printk (KERN_DEBUG "sl11_ep0: host cancelled tx\n");
	}
	if (endpoint->rcv_urb) {
		endpoint->rcv_urb = NULL;
		ep0_urb->actual_length = 0;
		printk (KERN_DEBUG "sl11_ep0: host cancelled rcv\n");
	}
	// read setup packet
	//len = sl11read_epn(0, EPN_XferLen);
	//printk(KERN_DEBUG"sl11_ep0: reading setup\n");

	sl11read_buffer (ep_address[0], (unsigned char *) &ep0_urb->device_request, 8);

	//sl11_dump(0, 255);

	cp = (unsigned char *) &ep0_urb->device_request;
	//printk(KERN_DEBUG"setup: %02x %02x %02x %02x %02x %02x %02x %02x\n",
	//        cp[0], cp[1], cp[2], cp[3], cp[4], cp[5], cp[6], cp[7]);

	// process setup packet
	if (usbd_recv_setup (ep0_urb)) {
		printk (KERN_DEBUG "sl11_ep0: setup failed\n");
		//sl11write_epn(0, )
		//udc_stall_ep(0);
		return;
	}
	//sl11write_epn(0, EPN_XferLen, 0);
	//sl11write_epn(0, EPN_Control, EPN_CTRL_SEQUENCE | EPN_CTRL_DIRECTION_IN | EPN_CTRL_ENABLE | EPN_CTRL_ARM);
	//return; // XXX

	// check data direction
	if ((ep0_urb->device_request.bmRequestType & USB_REQ_DIRECTION_MASK) == USB_REQ_HOST2DEVICE) {


		// should we setup to receive data
		if (le16_to_cpu (ep0_urb->device_request.wLength)) {
			//printk(KERN_DEBUG"sl11_ep0: read data %d\n",le16_to_cpu(ep0_urb->device_request.wLength));
			endpoint->rcv_urb = ep0_urb;
			endpoint->rcv_urb->actual_length = 0;
			sl11_out (0, endpoint);
			return;
		}
		//printk(KERN_DEBUG"sl11_ep0: send ack %d\n",le16_to_cpu(ep0_urb->device_request.wLength));

		// should be finished, send ack
		sl11write_epn (0, EPN_XferLen, 0);
		sl11write_epn (0, EPN_Control,
			       EPN_CTRL_SEQUENCE | EPN_CTRL_DIRECTION_IN | EPN_CTRL_ENABLE |
			       EPN_CTRL_ARM);
		return;
	}
	// we should be sending data back
	//printk(KERN_DEBUG"sl11_ep0: send data: %d\n", le16_to_cpu(ep0_urb->device_request.wLength));

	// verify that we have non-zero request length
	if (!le16_to_cpu (ep0_urb->device_request.wLength)) {
		udc_stall_ep (0);
		return;
	}
	// verify that we have non-zero length response
	if (!ep0_urb->actual_length) {
		udc_stall_ep (0);
		return;
	}
	// start sending
	//printk(KERN_DEBUG"sl11_ep0: start sending\n");
	endpoint->tx_urb = ep0_urb;
	endpoint->sent = 0;
	endpoint->last = 0;
	ep_sequence[0] = 1;
	sl11_start (0, endpoint, 0);

}

/* ********************************************************************************************* */

/**
 * sl11_int_hndlr - interrupt handler
 *
 */
static void sl11_int_hndlr (int irq, void *dev_id, struct pt_regs *regs)
{
	int i;
	unsigned char status;

	int ep;

	udc_interrupts++;

	// loop while interrupt status register is non-zero
	for (i = 0; (i < 10) && (status = sl11read_byte (IntStatus)); i++) {

		//if ((status & ~INT_SOF_RECEIVED)) {
		//    printk(KERN_DEBUG"[%d] %02x\n", udc_interrupts, status);
		//}

		// Common interrupts - Endpoint done
		for (ep = 1; ep < UDC_MAX_ENDPOINTS; ep++) {
			struct usb_endpoint_instance *endpoint;
			if ((status & ep_int_mask[ep]) && (endpoint = ep_endpoints[ep])) {

				// transmit on other than endpoint zero
				if (endpoint->endpoint_address & IN) {
					endpoint->tx_interrupts++;
					sl11_in (ep, endpoint);
				}
				// receive on other than endpoint zero
				else {
					endpoint->rcv_interrupts++;
					sl11_out (ep, endpoint);
				}
			}
		}

		// uncommon interrupts
		if (status & (INT_USB_RESET | INT_EP0_DONE | INT_DMA_DONE | INT_SOF_RECEIVED)) {

			// Reset
			if (status & INT_USB_RESET) {
				printk (KERN_DEBUG "sl11_int_hndlr: RESET: %02x\n", status);
				udc_suspended = 0;
				usbd_device_event (udc_device, DEVICE_RESET, 0);
			}
			// endpoint zero
			if ((status & INT_EP0_DONE)) {
				struct usb_endpoint_instance *endpoint;
				endpoint = ep_endpoints[0];
				endpoint->ep0_interrupts++;
				sl11_ep0 (endpoint);
			}
			// DMA done
			if (status & INT_DMA_DONE) {
				printk (KERN_DEBUG "sl11_int_hndlr: DMA: %02x\n", status);
			}
			// SOF frame
			if (status & INT_SOF_RECEIVED) {
				udc_saw_sof = 1;

				if (udc_suspended) {
					udc_suspended = 0;
					usbd_device_event (udc_device, DEVICE_BUS_ACTIVITY, 0);
					printk (KERN_DEBUG "sl11_tick: RESUMED\n");
				}
			}
		}
		// clear all available status bits
		sl11write_byte (IntStatus, status);
	}
}

/* ********************************************************************************************* */

/**
 * sl11_tick - clock timer task 
 * @data:
 *
 * Run from global clock tick to check if we are suspended.
 */
static void sl11_tick (void *data)
{
	udc_ticks++;

	// is driver active
	if (data) {

		// if not suspended check if we should suspend
		if (udc_addressed && !udc_suspended) {

			int saw_sof = *((int *) data);

			if (!saw_sof) {

				// XXX
				//
				usbd_device_event (udc_device, DEVICE_BUS_INACTIVE, 0);
				udc_suspended = 1;
				printk (KERN_DEBUG "sl11_tick: SUSPENDED\n");
			}

			*(int *) data = 0;
		}
		// re-queue task
		queue_task (&sl11_tq, &tq_timer);
	} else {
		printk (KERN_DEBUG "sl11_tick: not restarting\n");
	}
}

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
	sl11_start (endpoint->endpoint_address & 0xf, endpoint, 0);
}


/**
 * udc_stall_ep - stall endpoint
 * @ep: physical endpoint
 *
 * Stall the endpoint.
 */
void udc_stall_ep (unsigned int ep)
{
	unsigned char status;

	if (ep < UDC_MAX_ENDPOINTS) {
		status = sl11read_epn (ep, EPN_Control);
		sl11write_epn (ep, EPN_Control, status | EPN_CTRL_SENDSTALL);
		udelay (10);	// XXX
		sl11write_epn (ep, EPN_Control, status);
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
		ep_sequence[ep] = 0;
	}
	printk (KERN_DEBUG "udc_reset_ep[%d]:\n", ep);
	if (!ep) {
		if (udc_addressed) {
			printk (KERN_DEBUG "udc_reset_ep[%d]: reseting address\n", ep);
			usb_address = 0;
			udc_addressed = 0;
			sl11write_byte (USBAdd, 0);

			// reset UDC
			sl11write_byte (CtrlReg, CTRL_USB_RESET);
			udelay (100);	// XXX may not be needed
			sl11write_byte (CtrlReg, 0);

			// enable UDC
			sl11write_byte (CtrlReg, CTRL_USB_ENABLE);

		}
		// arm EP0
		sl11write_epn (0, EPN_Control,
			       EPN_CTRL_ARM | EPN_CTRL_ENABLE | EPN_CTRL_DIRECTION_OUT);

		sl11_dump_epn (ep + 0, " ");
		sl11_dump_epn (ep + 4, "\n");
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
	// address cannot be setup until ack received
	usb_address = address;
}

/**
 * udc_serial_init - set a serial number if available
 */
static int __init udc_serial_init (struct usb_bus_instance *bus)
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

	return (((logical_endpoint & 0xf) >= UDC_MAX_ENDPOINTS) || (packetsize > 64)) ?
	    0 : (logical_endpoint & 0xf);
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

		sl11write_epn (ep, EPN_Control, 0);
		sl11write_epn (ep, EPN_XferLen, 0);
		sl11write_epn (ep, EPN_Status, 0);
		sl11write_epn (ep, EPN_Counter, 0);

		ep_address[ep] = EP_S_BUF;
		ep_address[ep * 2] = EP_S_BUF;

		// ep0
		if (ep == 0) {
			sl11write_epn (ep, EPN_Address, EP_S_BUF);
			sl11write_epn (ep, EPN_Control,
				       EPN_CTRL_ARM | EPN_CTRL_ENABLE | EPN_CTRL_DIRECTION_OUT);
			sl11write_epn (ep, EPN_XferLen, endpoint->rcv_packetSize);
			sl11write_epn (ep, EPN_Status, 0);
			sl11write_epn (ep, EPN_Counter, 0);
		}
		// IN
		else if (endpoint->endpoint_address & 0x80) {
			sl11write_epn (ep, EPN_Address, EP_S_BUF);
			sl11write_epn (ep, EPN_Control,
				       EPN_CTRL_ARM | EPN_CTRL_ENABLE | EPN_CTRL_DIRECTION_IN);
		}
		// OUT
		else if (endpoint->endpoint_address) {
			usbd_fill_rcv (device, endpoint, 5);
			endpoint->rcv_urb = first_urb_detached (&endpoint->rdy);

			ep_sequence[ep] = 0;

			sl11write_epn (ep + 0, EPN_Address, EP_A_BUF);
			sl11write_epn (ep + 4, EPN_Address, EP_B_BUF);

			ep_address[ep + 0] = EP_A_BUF;
			ep_address[ep + 4] = EP_B_BUF;

			sl11write_epn (ep + 0, EPN_XferLen, endpoint->rcv_packetSize);
			sl11write_epn (ep + 4, EPN_XferLen, endpoint->rcv_packetSize);

			sl11write_epn (ep + 0, EPN_Control,
				       EPN_CTRL_ARM | EPN_CTRL_ENABLE | EPN_CTRL_DIRECTION_OUT);
			sl11write_epn (ep + 0, EPN_Control,
				       EPN_CTRL_ARM | EPN_CTRL_ENABLE | EPN_CTRL_DIRECTION_OUT |
				       EPN_CTRL_NEXTDATA);

			sl11write_epn (ep + 4, EPN_Control,
				       EPN_CTRL_ARM | EPN_CTRL_ENABLE | EPN_CTRL_DIRECTION_OUT);

			printk (KERN_DEBUG "udc_setup_ep[%d]: setup OUT\n", ep);
			sl11_dump_epn (ep + 0, " ");
			sl11_dump_epn (ep + 4, "\n");
		}
	}
	//sl11_dump(0, 64);
}

/**
 * udc_disable_ep - disable endpoint
 * @ep:
 *
 * Disable specified endpoint 
 */
void udc_disable_ep (unsigned int ep)
{
#if 0
	if (ep < UDC_MAX_ENDPOINTS) {
		struct usb_endpoint_instance *endpoint;

		if ((endpoint = ep_endpoints[ep])) {
			ep_endpoints[ep] = NULL;
			usbd_flush_ep (endpoint);
		}
	}
#endif
}

/* ********************************************************************************************* */

/**
 * udc_connected - is the USB cable connected
 *
 * Return non-zeron if cable is connected.
 */
int udc_connected (void)
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
	// enable UDC
	sl11write_byte (CtrlReg, CTRL_USB_ENABLE);
}


/**
 * udc_disconnect - disable pullup resistor
 *
 * Turn off the USB connection by disabling the pullup resistor.
 */
void udc_disconnect (void)
{
	// disable UDC
	sl11write_byte (CtrlReg, 0);
}

/* ********************************************************************************************* */

/**
 * udc_enable_interrupts - enable interrupts
 *
 * Switch on UDC interrupts.
 *
 */
void udc_all_interrupts (struct usb_device_instance *device)
{
	printk (KERN_DEBUG "udc_enable_interrupts:\n");

	// set interrupt mask
	sl11write_byte (IntEna,
			INT_EP0_DONE | INT_EP1_DONE | INT_EP2_DONE | INT_EP3_DONE |
			INT_SOF_RECEIVED /*| INT_USB_RESET */ );

}


/**
 * udc_suspended_interrupts - enable suspended interrupts
 *
 * Switch on only UDC resume interrupt.
 *
 */
void udc_suspended_interrupts (struct usb_device_instance *device)
{
	printk (KERN_DEBUG "udc_enable_interrupts:\n");

	// set interrupt mask
	sl11write_byte (IntEna, INT_SOF_RECEIVED | INT_USB_RESET);

}


/**
 * udc_disable_interrupts - disable interrupts.
 *
 * switch off interrupts
 */
void udc_disable_interrupts (struct usb_device_instance *device)
{

	printk (KERN_DEBUG "udc_disable_interrupts:\n");

	// reset interrupt mask
	sl11write_byte (IntEna, 0);
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

	printk (KERN_DEBUG "\n");
	printk (KERN_DEBUG "udc_enable: device: %p\n", device);

	// save the device structure pointer
	udc_device = device;

	// ep0 urb
	if (!ep0_urb) {
		if (!(ep0_urb = usbd_alloc_urb (device, device->function_instance_array, 0, 512))) {
			printk (KERN_ERR "udc_enable: usbd_alloc_urb failed\n");
		}
	} else {
		printk (KERN_ERR "udc_enable: ep0_urb already allocated\n");
	}

	//ep0_urb->device = device;
	//usbd_alloc_urb_data(&ep0_urb, 512);

	// enable UDC
	// sl11write_byte(CtrlReg, CTRL_USB_ENABLE);

	// setup tick
	// XXX sl11_tq.sync = 0;
	sl11_tq.routine = sl11_tick;
	sl11_tq.data = &udc_saw_sof;
	queue_task (&sl11_tq, &tq_timer);

	udc_suspended = 1;
}


/**
 * udc_disable - disable the UDC
 *
 * Switch off the UDC
 */
void udc_disable (void)
{
	printk (KERN_DEBUG "************************* udc_disable:\n");

	// tell tick task to stop
	sl11_tq.data = NULL;
	while (sl11_tq.sync) {
		printk (KERN_DEBUG "waiting for sl11_tq to stop\n");
		schedule_timeout (10 * HZ);
	}

	// XXX del_timer() or wait 

	// disable UDC
	// sl11write_byte(CtrlReg, 0);

	// reset device pointer
	udc_device = NULL;

	// ep0 urb
	//kfree(ep0_urb.buffer);

	if (ep0_urb) {
		usbd_dealloc_urb (ep0_urb);
		ep0_urb = 0;
	} else {
		printk (KERN_ERR "udc_disable: ep0_urb already NULL\n");
	}

}


/**
 * udc_startup - allow udc code to do any additional startup
 */
void udc_startup_events (struct usb_device_instance *device)
{
	usbd_device_event (device, DEVICE_INIT, 0);
	usbd_device_event (device, DEVICE_CREATE, 0);
	usbd_device_event (device, DEVICE_HUB_CONFIGURED, 0);
	usbd_device_event (device, DEVICE_RESET, 0);	// XXX should be done from device event
}


/* ********************************************************************************************* */

/**
 * udc_init - initialize USB Device Controller
 * 
 * Get ready to use the USB Device Controller.
 *
 * Register an interrupt handler and IO region. Return non-zero for error.
 */
int udc_init (void)
{
	printk (KERN_DEBUG "udc_init:\n");

	// probe for Sl11
	return sl11_probe ();
}


/**
 * udc_regs - dump registers
 *
 * Dump registers with printk
 */
void udc_regs (void)
{
	sl11_dump_epn (0, " ");

	printk (" Ctrl: %02x Addr: %02x IntE: %02x IntS: %02x Data: %02x SOF: %02x %02x Sus: %d\n",
		sl11read_byte (CtrlReg), sl11read_byte (USBAdd), sl11read_byte (IntEna),
		sl11read_byte (IntStatus), sl11read_byte (DATASet),
		sl11read_byte (SOFHigh), sl11read_byte (SOFLow), udc_suspended);

	sl11_dump_epn (1, " ");
	sl11_dump_epn (5, "\n");

	sl11_dump_epn (2, " ");
	sl11_dump_epn (3, "\n");
}

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
	if (request_irq (UDC_IRQ, sl11_int_hndlr, SA_INTERRUPT | SA_SAMPLE_RANDOM, UDC_NAME, NULL)
	    != 0) {
		printk (KERN_DEBUG "usb_ctl: Couldn't request USB irq\n");
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
	return 0;
}

/**
 * udc_request_udc_io - request UDC io region
 *
 * Return non-zero if not successful.
 */
int udc_request_io ()
{
	int rc = 0;
#ifdef CONFIG_X86
	// reserve io
	{
		unsigned long flags;
		local_irq_save (flags);
		if (check_region (UDC_ADDR, UDC_ADDR_SIZE)) {
			printk (KERN_DEBUG "udc_request_io: failed %x %x\n", UDC_ADDR,
				UDC_ADDR_SIZE);
			rc = -EINVAL;
		}
		request_region (UDC_ADDR, UDC_ADDR_SIZE, UDC_NAME);
		local_irq_restore (flags);
	}
#endif
	return rc;
}

/**
 * udc_release_udc_irq - release UDC irq
 */
void udc_release_udc_irq ()
{
	free_irq (UDC_IRQ, NULL);
}

/**
 * udc_release_cable_irq - release Cable irq
 */
void udc_release_cable_irq ()
{
}

/**
 * udc_release_io - release UDC io region
 */
void udc_release_io ()
{
#ifdef CONFIG_X86
	release_region (UDC_ADDR, UDC_ADDR_SIZE);
#endif
}
