/*
 * linux/drivers/usbd/gen_bi/udc.c -- USB Device Controller driver. 
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
MODULE_DESCRIPTION ("USB Device Prototype Bus Interface");

USBD_MODULE_INFO ("gen_bi 0.1-alpha");

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

//#include "xxxx.h"
#include "gen.h"

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))


static struct usb_device_instance *udc_device;	// required for the interrupt handler

/*
 * ep_endpoints - map physical endpoints to logical endpoints
 */
static struct usb_endpoint_instance *ep_endpoints[UDC_MAX_ENDPOINTS];

static struct urb ep0_urb;
static unsigned char usb_address;

extern unsigned int udc_interrupts;

/* ********************************************************************************************* */
/* IO
 */

/* ********************************************************************************************* */
/* Control (endpoint zero)
 */

/* ********************************************************************************************* */
/* Bulk OUT (recv)
 */

/* ********************************************************************************************* */
/* Bulk IN (tx)
 */

/**
 * send_data - send packet via endpoint
 * @ep: logical endpoint number
 * @bp: pointer to data
 * @size: bytes to write
 */
static void __inline__ send_data (unsigned char ep, unsigned char *bp, unsigned char size)
{

	if (bp && size) {
		// copy data from buffer to chip
	}
	// arm
}


/**
 * start_in - start transmit
 * @ep:
 */
static void __inline__ start_in (unsigned int ep, struct usb_endpoint_instance *endpoint,
				 int restart)
{
	if (endpoint->tx_urb) {
		struct urb *urb = endpoint->tx_urb;

		if ((urb->actual_length - endpoint->sent) > 0) {
			endpoint->last =
			    MIN (urb->actual_length - endpoint->sent, endpoint->tx_packetSize);
			send_data (ep, urb->buffer + endpoint->sent, endpoint->last);
		} else {
			// XXX ZLP
			endpoint->last = 0;
			send_data (ep, urb->buffer + endpoint->sent, 0);
		}
	}
}


/* ********************************************************************************************* */
/* Interrupt Handler
 */

#if 0
/**
 * int_hndlr - interrupt handler
 *
 */
static void int_hndlr (int irq, void *dev_id, struct pt_regs *regs)
{
	udc_interrupts++;

}
#endif


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

}

/**
 * udc_init - initialize
 *
 * Return non-zero if we cannot see device.
 **/
int udc_init (void)
{
	// reset

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
		if (!endpoint->tx_urb) {
			usbd_tx_complete_irq (endpoint, 0);
			start_in (endpoint->endpoint_address & 0x7f, endpoint, 0);
		}
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

#if 0
/**
 * udc_serial_init - set a serial number if available
 */
static int __init udc_serial_init (struct usb_bus_instance *bus)
{
	return -EINVAL;
}
#endif

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
}


/**
 * udc_disconnect - disable pullup resistor
 *
 * Turn off the USB connection by disabling the pullup resistor.
 */
void udc_disconnect (void)
{
}

#if 0
/**
 * udc_int_hndlr_cable - interrupt handler for cable
 */
static void udc_int_hndlr_cable (int irq, void *dev_id, struct pt_regs *regs)
{
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
	printk (KERN_DEBUG "udc_enable_interrupts:\n");
	// set interrupt mask
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
	ep0_urb.device = device;
	usbd_alloc_urb_data (&ep0_urb, 512);

	// enable UDC

}


/**
 * udc_disable - disable the UDC
 *
 * Switch off the UDC
 */
void udc_disable (void)
{
	printk (KERN_DEBUG "************************* udc_disable:\n");

	// disable UDC

	// reset device pointer
	udc_device = NULL;

	// ep0 urb
	kfree (ep0_urb.buffer);
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
#ifdef XXXX_UDC_IRQ
	// request IRQ  and IO region
	if (request_irq
	    (XXXX_UDC_IRQ, int_hndlr, SA_INTERRUPT | SA_SAMPLE_RANDOM,
	     UDC_NAME " USBD Bus Interface", NULL) != 0) {
		printk (KERN_DEBUG "usb_ctl: Couldn't request USB irq\n");
		return -EINVAL;
	}
#endif
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
		printk (KERN_DEBUG "usb_ctl: Couldn't request USB irq\n");
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
#if defined(XXXX_ADDR) && defined(XXXX_ADDR_SIZE)
	// reserve io
	{
		unsigned long flags;
		local_irq_save (flags);
		if (check_region (XXXX_ADDR, XXXX_ADDR_SIZE)) {
			return -EINVAL;
		}
		request_region (XXXX_ADDR, 0x04, UDC_NAME " USBD Bus Interface");
		local_irq_restore (flags);
	}
#endif
	return 0;
}

/**
 * udc_release_udc_irq - release UDC irq
 */
void udc_release_udc_irq ()
{
#ifdef XXXX_IRQ
	free_irq (XXXX_UDC_IRQ, NULL);
#endif
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
 * udc_release_release_io - release UDC io region
 */
void udc_release_io ()
{
#if defined(XXXX_ADDR) && defined(XXXX_ADDR_SIZE)
	release_region (XXXX_ADDR, XXXX_ADDR_SIZE);
#endif
}


/**
 * udc_regs - dump registers
 *
 * Dump registers with printk
 */
void udc_regs (void)
{
	printk ("\n");
}
