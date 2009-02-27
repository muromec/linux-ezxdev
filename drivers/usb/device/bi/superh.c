/*
 * linux/drivers/usbd/superh_bi/udc.c -- USB Device Controller driver. 
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
MODULE_DESCRIPTION ("USB Device SuperH Bus Interface");
USBD_MODULE_INFO ("superh_bi 0.1-alpha");

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/delay.h>

#include <asm/types.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/sh7727.h>

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-inline.h"
#include "usbd-bi.h"

#include "superh.h"
#include "superh-hardware.h"

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

/*
 * Define what interrupts are high versus low priority
 */

#define F0_HIGH (EP1_FULL | EP2_TR | EP2_EMPTY )
#define F0_LOW  (BRST | SETUP_TS | EP0o_TS | EP0i_TR | EP0i_TS)

#define F1_HIGH (0)
#define F1_LOW  (EP3_TR | EP3_TS | VBUSF)


static struct usb_device_instance *udc_device;	// required for the interrupt handler

/*
 * ep_endpoints - map physical endpoints to logical endpoints
 */
static struct usb_endpoint_instance *ep_endpoints[UDC_MAX_ENDPOINTS];

static struct urb *ep0_urb;
static unsigned char usb_address;

extern unsigned int udc_interrupts;
unsigned int udc_f0_interrupts;
unsigned int udc_f1_interrupts;
unsigned long udc_vbusf_time;

/* ********************************************************************************************* */

/*
 * IO
 */

void and_b (unsigned short mask, unsigned long addr)
{
	ctrl_outb (ctrl_inw (addr) & mask, addr);
}

void or_b (unsigned short mask, unsigned long addr)
{
	ctrl_outb (ctrl_inw (addr) | mask, addr);
}

void and_w (unsigned short mask, unsigned long addr)
{
	ctrl_outw (ctrl_inw (addr) & mask, addr);
}

void or_w (unsigned short mask, unsigned long addr)
{
	ctrl_outw (ctrl_inw (addr) | mask, addr);
}

// IO addresses for each physical ports FIFO
unsigned long ep_address_o[4] = { USBEPDR0O, USBEPDR1, 0L, 0L, };
unsigned long ep_address_i[4] = { USBEPDR0I, 0L, USBEPDR2, USBEPDR3, };

// IO addresses for each physical ports trigger
unsigned char ep_trigger_o[4] = { EP0o_PKTE, EP1_PKTE, 0L, 0L, };
unsigned char ep_trigger_i[4] = { EP0i_PKTE, 0L, EP2_PKTE, EP3_PKTE, };


/**
 * superh_write_buffer - write a buffer to the superh fifo
 * @ep: endpoint
 * @b: pointer to buffer to write
 * @size: number of bytes to write
 */
static /*__inline__*/ void superh_write_buffer (unsigned char ep, unsigned char *b,
						unsigned char size)
{
	if ((ep < UDC_MAX_ENDPOINTS) && ep_address_i[ep]) {
		while (size-- > 0) {
			ctrl_outb (*b++, ep_address_i[ep]);
		}
		ctrl_outb (ep_trigger_i[ep], USBTRG);
	}
}

/**
 * superh_read_buffer - fill a buffer from the superh fifo
 * @ep: endpoint
 * @b: pointer to buffer to fill
 * @size: number of bytes to read
 */
static /*__inline__*/ void superh_read_buffer (unsigned char ep, unsigned char *b,
					       unsigned char size)
{
	if ((ep < UDC_MAX_ENDPOINTS) && ep_address_o[ep]) {
		while (size-- > 0) {
			*b++ = ctrl_inb (ep_address_o[ep]);
		}
		ctrl_outb (ep_trigger_o[ep], USBTRG);
	}
}

/* ********************************************************************************************* */
/* Bulk OUT (recv)
 */
static void /*__inline__*/ superh_out_ep1 (struct usb_endpoint_instance *endpoint)
{
	int size = ctrl_inb (USBEPSZ1);

	if (endpoint && endpoint->rcv_urb && size) {
		// read data
		superh_read_buffer (1, endpoint->rcv_urb->buffer + endpoint->rcv_urb->actual_length,
				    size);
		usbd_rcv_complete_irq (endpoint, size, 0);
	} else {
		// reset fifo
		or_b (EP1_CLEAR, USBFCLR);
		usbd_rcv_complete_irq (endpoint, 0, EINVAL);
	}
}

/* ********************************************************************************************* */
/* Bulk IN (tx)
 */

/**
 * superh_in_epn - process tx interrupt
 * @ep:
 * @endpoint:
 *
 * Determine status of last data sent, queue new data.
 */
static /* __inline__ */ void superh_in_epn (int ep, int restart)
{
	if (ctrl_inb (USBIFR0) & EP2_EMPTY) {

		if ((ep < UDC_MAX_ENDPOINTS) && ep_endpoints[ep]) {
			struct usb_endpoint_instance *endpoint = ep_endpoints[ep];
			usbd_tx_complete_irq (endpoint, restart);
			if (endpoint->tx_urb) {
				struct urb *urb = endpoint->tx_urb;

				if ((urb->actual_length - endpoint->sent) > 0) {
					endpoint->last =
					    MIN (urb->actual_length - endpoint->sent,
						 endpoint->tx_packetSize);
					superh_write_buffer (ep, urb->buffer + endpoint->sent,
							     endpoint->last);
				} else {
					// XXX ZLP
					endpoint->last = 0;
					superh_write_buffer (ep, urb->buffer + endpoint->sent, 0);
				}
				// enable transmit done interrupt
				switch (ep) {
				case 2:
					or_b (EP2_TR | EP2_EMPTY, USBIER0);
					break;
				case 3:
					or_b (EP3_TR | EP3_TS, USBIER1);
					break;
				}
			} else {
				// disable transmit done interrupt
				switch (ep) {
				case 2:
					if (ctrl_inb (USBIER0) & (EP2_TR | EP2_EMPTY)) {
						and_b (~(EP2_TR | EP2_EMPTY), USBIER0);
					}
					break;
				case 3:
					and_b (~(EP3_TR | EP3_TS), USBIER1);
					break;
				}
			}
		}
	}
}

/* ********************************************************************************************* */
/* Control (endpoint zero)
 */

/**
 * superh_in_ep0 - start transmit
 * @ep:
 */
static void /*__inline__*/ superh_in_ep0 (struct usb_endpoint_instance *endpoint)
{
	if (endpoint && endpoint->tx_urb) {
		struct urb *urb = endpoint->tx_urb;

		printk (KERN_DEBUG "superh_in_ep0: length: %d\n", endpoint->tx_urb->actual_length);	// XXX 

		if ((urb->actual_length - endpoint->sent) > 0) {
			endpoint->last =
			    MIN (urb->actual_length - endpoint->sent, endpoint->tx_packetSize);
			superh_write_buffer (0, urb->buffer + endpoint->sent, endpoint->last);
		} else {
			// XXX ZLP
			endpoint->last = 0;
			superh_write_buffer (0, urb->buffer + endpoint->sent, 0);
		}
	}
}

/**
 * superh_ep0_setup
 */
static void superh_ep0_setup (void)
{
	if (ep_endpoints[0]) {
		int i;
		struct usb_endpoint_instance *endpoint = ep_endpoints[0];
		unsigned char *cp = (unsigned char *) &ep0_urb->device_request;

		for (i = 0; i < 8; i++) {
			cp[i] = ctrl_inb (USBEPDR0S);
		}

		// process setup packet
		if (usbd_recv_setup (ep0_urb)) {
			printk (KERN_DEBUG "superh_ep0: setup failed\n");
			return;
		}
		// check data direction
		if ((ep0_urb->device_request.bmRequestType & USB_REQ_DIRECTION_MASK) ==
		    USB_REQ_HOST2DEVICE) {

			// should we setup to receive data
			if (le16_to_cpu (ep0_urb->device_request.wLength)) {
				printk (KERN_DEBUG "superh_ep0: setup to read data %d\n",
					le16_to_cpu (ep0_urb->device_request.wLength));
				endpoint->rcv_urb = ep0_urb;
				endpoint->rcv_urb->actual_length = 0;
				//superh_out(0, endpoint);
				return;
			}

			printk (KERN_DEBUG "superh_ep0: send ack %d\n", le16_to_cpu (ep0_urb->device_request.wLength));	// XXX 

			// should be finished, send ack
			ctrl_outb (EP0s_PKTE, USBTRG);
			return;
		}
		// we should be sending data back

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
		// send ack prior to sending data
		ctrl_outb (EP0s_PKTE, USBTRG);

		// start sending
		endpoint->tx_urb = ep0_urb;
		endpoint->sent = 0;
		endpoint->last = 0;
		superh_in_ep0 (endpoint);
	}
}


/* ********************************************************************************************* */
/* Interrupt Handler(s)
 */


/**
 * superh_int_hndlr_f0 - high priority interrupt handler
 *
 */
static void superh_int_hndlr_f0 (int irq, void *dev_id, struct pt_regs *regs)
{

	unsigned char f0_status;

	udc_interrupts++;
	udc_f0_interrupts++;
	f0_status = ctrl_inb (USBIFR0);

	if (f0_status & EP1_FULL) {
		superh_out_ep1 (ep_endpoints[1]);
		f0_status = ctrl_inb (USBIFR0);
		if (f0_status & EP1_FULL) {
			superh_out_ep1 (ep_endpoints[1]);
		}
	} else if (f0_status & (EP2_TR | EP2_EMPTY)) {
		superh_in_epn (2, 0);	// XXX status?
		ctrl_outb (~(f0_status & EP2_TR), USBIFR0);
	}
}

/**
 * superh_int_hndlr_f1 - low priority interrupt handler
 *
 */
static void superh_int_hndlr_f1 (int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned char f0_status;
	unsigned char f1_status;

	udc_interrupts++;
	udc_f1_interrupts++;
	f0_status = ctrl_inb (USBIFR0);
	f1_status = ctrl_inb (USBIFR1);
	ctrl_outb (~(f0_status & F0_LOW), USBIFR0);
	ctrl_outb (~(f1_status & F1_LOW), USBIFR1);

	//printk(KERN_DEBUG"superh_int_hndlr_f1[%x] status %02x %02x\n", udc_interrupts, f0_status, f1_status); // XXX 

	if (f1_status & VBUSF) {
		/*
		 * XXX vbusf can bounce, probably should disarm vbusf interrupt, for now, just check 
		 * that we don't handle it more than once.
		 */
		if (!udc_vbusf_time || ((jiffies - udc_vbusf_time) > 1)) {
			if (udc_device) {
				if (f1_status & VBUSMN) {
					printk (KERN_DEBUG
						"superh_int_hndlr_f1[%x]: VBUSF VBUSMN set\n",
						udc_interrupts);
				} else {
					printk (KERN_DEBUG
						"superh_int_hndlr_f1[%x]: VBUSF VBUSMN reset\n",
						udc_interrupts);
				}
			}
		}
		udc_vbusf_time = jiffies;
	} else {
		udc_vbusf_time = 0;
	}
	if (f0_status & BRST) {
		printk (KERN_DEBUG "superh_int_hndlr_f1[%x]: BRST bus reset\n", udc_interrupts);

		// reset fifo's and stall's
		or_b (EP3_CLEAR | EP1_CLEAR | EP2_CLEAR | EP0o_CLEAR | EP0i_CLEAR, USBFCLR);
		or_b (0, USBEPSTL);

		if (udc_device->device_state != DEVICE_RESET) {
			printk (KERN_DEBUG "superh_int_hndlr_f1[%x]: DEVICE RESET\n",
				udc_interrupts);
			usbd_device_event_irq (udc_device, DEVICE_RESET, 0);
		}
	}
	if (f0_status & SETUP_TS) {
		printk (KERN_DEBUG "superh_int_hndlr_f1[%x]: SETUP TS\n", udc_interrupts);
		or_b (EP0o_CLEAR | EP0i_CLEAR, USBFCLR);
		superh_ep0_setup ();
		if (udc_device->device_state == STATE_DEFAULT) {
			/*
			 * superh won't give us set address, configuration or interface, but at least
			 * we know that at this point we know that a host is talking to us, close
			 * enough.
			 */
			usbd_device_event (udc_device, DEVICE_ADDRESS_ASSIGNED, 0);
			udc_device->configuration = 0;
			usbd_device_event (udc_device, DEVICE_CONFIGURED, 0);
			udc_device->interface = 1;	// no options
			udc_device->alternate = 0;	// no options
			usbd_device_event (udc_device, DEVICE_SET_INTERFACE, 0);
		}
	}
	if (f0_status & EP0i_TR) {
		usbd_tx_complete_irq (ep_endpoints[0], 0);
		superh_in_ep0 (ep_endpoints[0]);
	}
	if (f0_status & EP0o_TS) {
		printk (KERN_DEBUG "superh_int_hndlr_f1[%x]: ep0o TS\n", udc_interrupts);
	}
	if (f0_status & EP0i_TS) {
		printk (KERN_DEBUG "superh_int_hndlr_f1[%x]: ep0iTS\n", udc_interrupts);
	}
	if (f1_status & EP3_TR) {
		printk (KERN_DEBUG "superh_int_hndlr_f1[%x]: EP3 TR\n", udc_interrupts);
		superh_in_epn (3, 0);	// XXX status?
	}
	if (f1_status & EP3_TS) {
		printk (KERN_DEBUG "superh_int_hndlr_f1[%x]: EP3 TS\n", udc_interrupts);
	}
	superh_in_epn (3, 0);	// XXX status?
}


/* ********************************************************************************************* */


/* ********************************************************************************************* */
/* Start of public functions.  */

/**
 * udc_start_in_irq - start transmit
 * @endpoint: endpoint instance
 *
 * Called by bus interface driver to see if we need to start a data transmission.
 */
void udc_start_in_irq (struct usb_endpoint_instance *endpoint)
{
	if (endpoint) {
		// XXX should verify logical address mapping to physical 2
		superh_in_epn (2, 0);
	}
}

/**
 * udc_init - initialize
 *
 * Return non-zero if we cannot see device.
 **/
int udc_init (void)
{
	// XXX move clock enable to udc_enable, add code to udc_disable to disable them

	// Reset and then Select Function USB1_pwr_en out (USB) c.f. Section 26, Table 26.1 PTE2
	and_w (PN_PB2_MSK, PECR);
	or_w (PN_PB2_OF, PECR);

	// Reset and then Select Function UCLK c.f. Section 26, Table 26.1, PTD6
	and_w (PN_PB6_MSK, PDCR);
	or_w (PN_PB6_OF, PDCR);

	// Stop USB module prior to setting clocks c.f. Section 9.2.3
	and_b (~MSTP14, STBCR3);
	or_b (MSTP14, STBCR3);

	// Select external clock, 1/1 divisor c.f. Section 11.3.1
	or_b (USBDIV_11 | USBCKS_EC, EXCPGCR);

	// Start USB c.f. Section 9.2.3
	and_b (~MSTP14, STBCR3);

	// Disable pullup c.f. Section 23.5.19
	printk (KERN_DEBUG "udc_init:\n");
	printk (KERN_DEBUG "udc_init: Disable Pullup\n");
	or_b (PULLUP_E, USBDMA);
	//and_b(~PULLUP_E, USBDMA);

	// Set port 1 to function, disabled c.f. Section 22.2.1
	or_w (USB_TRANS_TRAN | USB_SEL_FUNC, EXPFC);

	// Enable pullup c.f. Section 23.5.19
	printk (KERN_DEBUG "udc_init:\n");
	printk (KERN_DEBUG "udc_init: Enable Pullup\n");
	and_b (~PULLUP_E, USBDMA);
	//or_b(PULLUP_E, USBDMA);

	// reset fifo's and stall's
	or_b (EP3_CLEAR | EP1_CLEAR | EP2_CLEAR | EP0o_CLEAR | EP0i_CLEAR, USBFCLR);
	or_b (0, USBEPSTL);

	// setup interrupt priority by using the interrupt select registers
	ctrl_outb (F0_LOW, USBISR0);
	ctrl_outb (F1_LOW, USBISR1);

	printk (KERN_DEBUG "udc_init:\n");
	return 0;
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
		// reset
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
	return (((logical_endpoint & 0xf) >= UDC_MAX_ENDPOINTS)
		|| (packetsize > 64)) ? 0 : (logical_endpoint & 0xf);
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
	printk (KERN_DEBUG "udc_setup_ep: ep: %d\n", ep);
	if (ep < UDC_MAX_ENDPOINTS) {

		ep_endpoints[ep] = endpoint;
		// ep0
		if (ep == 0) {
		}
		// IN
		else if (endpoint->endpoint_address & 0x80) {
		}
		// OUT
		else if (endpoint->endpoint_address) {
			usbd_fill_rcv (device, endpoint, 5);
			endpoint->rcv_urb = first_urb_detached (&endpoint->rdy);
		}
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
	}
}

/* ********************************************************************************************* */

/**
 * udc_connected - is the USB cable connected
 *
 * Return non-zero if cable is connected.
 */
int udc_connected ()
{
	return (ctrl_inb (USBIFR1) & VBUSMN) ? 1 : 0;
}

/**
 * udc_connect - enable pullup resistor
 *
 * Turn on the USB connection by enabling the pullup resistor.
 */
void udc_connect (void)
{
	// enable pullup
	printk (KERN_DEBUG "udc_connect:\n");
	and_b (~PULLUP_E, USBDMA);
}

/**
 * udc_disconnect - disable pullup resistor
 *
 * Turn off the USB connection by disabling the pullup resistor.
 */
void udc_disconnect (void)
{
	// enable pullup
	printk (KERN_DEBUG "udc_disconnect:\n");
	or_b (PULLUP_E, USBDMA);
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
	// set interrupt mask
	or_b (BRST | EP1_FULL | SETUP_TS | EP0o_TS | EP0i_TR | EP0i_TS, USBIER0);
	or_b (EP3_TR | EP3_TS | VBUSF, USBIER1);
}

/**
 * udc_suspended_interrupts - enable suspended interrupts
 *
 * Switch on only UDC resume interrupt.
 *
 */
void udc_suspended_interrupts (struct usb_device_instance *device)
{
}

/**
 * udc_disable_interrupts - disable interrupts.
 *
 * switch off interrupts
 */
void udc_disable_interrupts (struct usb_device_instance *device)
{
	// reset interrupt mask
	ctrl_outb (0x0, USBIER0);
	ctrl_outb (0x0, USBIER1);
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
		if ((ep0_urb = usbd_alloc_urb (device, device->function_instance_array, 0, 512))) {
			printk (KERN_ERR "ep0_enable: usbd_alloc_urb failed\n");
		}
	} else {
		printk (KERN_ERR "udc_enable: ep0_urb already allocated\n");
	}

	// XXX enable UDC
}

/**
 * udc_disable - disable the UDC
 *
 * Switch off the UDC
 */
void udc_disable (void)
{
	// XXX disable UDC

	// reset device pointer
	udc_device = NULL;

	// ep0 urb
	if (ep0_urb) {
		usbd_dealloc_urb (ep0_urb);
		ep0_urb = NULL;
	}
}

/**
 * udc_startup_events - allow udc code to do any additional startup
 */
void udc_startup_events (struct usb_device_instance *device)
{
	printk (KERN_DEBUG "udc_startup_events:\n");
	usbd_device_event (device, DEVICE_INIT, 0);
	usbd_device_event (device, DEVICE_CREATE, 0);
	usbd_device_event (device, DEVICE_HUB_CONFIGURED, 0);

	// XXX the following could be snuck into get descriptor LANGID

#if 0
	usbd_device_event (device, DEVICE_ADDRESS_ASSIGNED, 0);
	device->configuration = 0;
	usbd_device_event (device, DEVICE_CONFIGURED, 0);
	device->interface = 1;
	device->alternate = 0;
	usbd_device_event (device, DEVICE_SET_INTERFACE, 0);
#endif
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
 */
int udc_request_udc_irq ()
{
	if (request_irq (USBF0_IRQ, superh_int_hndlr_f0, SA_INTERRUPT | SA_SAMPLE_RANDOM, UDC_NAME
			 " USBD Bus Interface (high priority)", NULL) != 0) {
		printk (KERN_DEBUG "usb_ctl: Couldn't request USB irq F0\n");
		return -EINVAL;
	}
	return 0;
}

/**
 * udc_request_cable_irq - request Cable interrupt
 */
int udc_request_cable_irq ()
{
	if (request_irq (USBF1_IRQ, superh_int_hndlr_f1, SA_INTERRUPT | SA_SAMPLE_RANDOM,
			 UDC_NAME " USBD Bus Interface (low priority)", NULL) != 0) {
		printk (KERN_DEBUG "usb_ctl: Couldn't request USB irq F1\n");
		free_irq (USBF0_IRQ, NULL);
		return -EINVAL;
	}
	return 0;
}

/**
 * udc_request_udc_io - request UDC io region
 */
int udc_request_io ()
{
	return 0;
}

/**
 * udc_release_udc_irq - release UDC irq
 */
void udc_release_udc_irq ()
{
	free_irq (USBF0_IRQ, NULL);
}

/**
 * udc_release_cable_irq - release Cable irq
 */
void udc_release_cable_irq ()
{
	free_irq (USBF1_IRQ, NULL);
}

/**
 * udc_release_release_io - release UDC io region
 */
void udc_release_io ()
{
}

/**
 * udc_regs - dump registers
 */
void udc_regs (void)
{
	printk (KERN_DEBUG
		"[%d:%d:%d] IFR[%02x:%02x] IER[%02x:%02x] ISR[%02x:%02x] DASTS[%02x] EPSTL[%02x] EPSZ1[%02x] DMA[%02x]\n",
		udc_interrupts, udc_f0_interrupts, udc_f1_interrupts,
		ctrl_inb (USBIFR0), ctrl_inb (USBIFR1), ctrl_inb (USBIER0), ctrl_inb (USBIER1),
		ctrl_inb (USBISR0), ctrl_inb (USBISR1), ctrl_inb (USBDASTS), ctrl_inb (USBEPSTL),
		ctrl_inb (USBEPSZ1), ctrl_inb (USBDMA)
	    );
}
