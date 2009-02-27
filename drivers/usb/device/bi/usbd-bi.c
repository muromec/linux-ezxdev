/*
 * linux/drivers/usbd/usbd-bi.c - USB Bus Interface Driver
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
 * Copyright (c) 2003 MontaVista Software Inc.
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

#include <linux/config.h>
#include <linux/module.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>

#include <linux/proc_fs.h>

#include <linux/sched.h>
#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>

#include <linux/netdevice.h>
#include <linux/proc_fs.h>

#include <asm/uaccess.h>

#if defined(CONFIG_PM) && !defined(CONFIG_USBD_MONITOR) && !defined(CONFIG_USBD_MONITOR_MODULE)
#include <linux/pm.h>
#endif

#ifdef CONFIG_ARCH_MX2ADS
#ifndef CONFIG_DPM
#define CONFIG_DPM
#endif
#endif

#ifdef CONFIG_DPM	/* MVL-CEE */
#include <linux/device.h>
#endif

#include "../usbd.h"
#include "../usbd-debug.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-inline.h"
#include "usbd-bi.h"

//#define DEBUG 1
#if DEBUG
static unsigned int usbd_bi_dbg = 1;
#else
#define usbd_bi_dbg 0
#endif

/* Module Parameters ************************************************************************* */

static char *dbg = NULL;

MODULE_PARM (dbg, "s");

/* Debug switches (module parameter "dbg=...") *********************************************** */

extern int dbgflg_usbdbi_init;
int dbgflg_usbdbi_intr;
int dbgflg_usbdbi_tick;
int dbgflg_usbdbi_usbe;
int dbgflg_usbdbi_rx;
int dbgflg_usbdbi_tx;
int dbgflg_usbdbi_dma_flg;
int dbgflg_usbdbi_setup;
int dbgflg_usbdbi_ep0;
int dbgflg_usbdbi_udc;
int dbgflg_usbdbi_stall;
int dbgflg_usbdbi_pm;
int dbgflg_usbdbi_pur;

static debug_option dbg_table[] = {
	{&dbgflg_usbdbi_init, NULL, "init", "initialization and termination"},
	{&dbgflg_usbdbi_intr, NULL, "intr", "interrupt handling"},
	{&dbgflg_usbdbi_tick, NULL, "tick", "interrupt status monitoring on clock tick"},
	{&dbgflg_usbdbi_usbe, NULL, "usbe", "USB events"},
	{&dbgflg_usbdbi_rx, NULL, "rx", "USB RX (host->device) handling"},
	{&dbgflg_usbdbi_tx, NULL, "tx", "USB TX (device->host) handling"},
	{&dbgflg_usbdbi_dma_flg, NULL, "dma", "DMA handling"},
	{&dbgflg_usbdbi_setup, NULL, "setup", "Setup packet handling"},
	{&dbgflg_usbdbi_ep0, NULL, "ep0", "End Point 0 packet handling"},
	{&dbgflg_usbdbi_udc, NULL, "udc", "USB Device"},
	{&dbgflg_usbdbi_stall, NULL, "stall", "Testing"},
	{&dbgflg_usbdbi_pm, NULL, "pm", "Power Management"},
	{&dbgflg_usbdbi_pur, NULL, "pur", "USB cable Pullup Resistor"},
	{NULL, NULL, NULL, NULL}
};

/* XXX
static __initdata unsigned char default_dev_addr[ETH_ALEN] = {
        0x40, 0x00, 0x00, 0x00, 0x00, 0x01
};
*/

/* globals */

int have_cable_irq;

/* ticker */

int ticker_terminating;
int ticker_timer_set;

/**
 * kickoff_thread - start management thread
 */
void ticker_kickoff (void);

/**
 * killoff_thread - stop management thread
 */
void ticker_killoff (void);

#if defined(CONFIG_PM) && !defined(CONFIG_USBD_MONITOR) && !defined(CONFIG_USBD_MONITOR_MODULE)
struct pm_dev *pm_dev;		// power management
#endif


/* Bus Interface Callback Functions ********************************************************** */

/**
 * bi_cancel_urb - cancel sending an urb
 * @urb: data urb to cancel
 *
 * Used by the USB Device Core to cancel an urb.
 */
int bi_cancel_urb (struct urb *urb)
{
	dbgENTER (dbgflg_usbdbi_tx, 1);
	//ep2_reset();
	return 0;
}

/**
 * bi_find_endpoint - find endpoint
 * @device: device
 * @endpoint_address: endpoint address
 *
 * The endpoint_address MUST have the IN/OUT bit (0x80) set appropriately to 
 * distinguish IN endpoints from OUT endpoints.
 */
struct usb_endpoint_instance *bi_find_endpoint(struct usb_device_instance *device, int endpoint_address)
{
	if (device && device->bus && device->bus->endpoint_array) {
                int i;
		for (i = 0; i < udc_max_endpoints (); i++) {
			struct usb_endpoint_instance *endpoint;
			if ((endpoint = device->bus->endpoint_array + i)) {
				if (endpoint->endpoint_address == endpoint_address) {
                                        return endpoint;
                                }
			}
		}
	}
        return NULL;
}

/**
 * bi_endpoint_halted - check if endpoint halted
 * @device: device
 * @endpoint: endpoint to check
 *
 * Used by the USB Device Core to check endpoint halt status.
 */
int bi_endpoint_halted (struct usb_device_instance *device, int endpoint_address)
{
        struct usb_endpoint_instance *endpoint;
        if ((endpoint = bi_find_endpoint(device, endpoint_address))) {
                dbg_ep0 (1, "endpoint: %d status: %d", endpoint_address, endpoint->status);
                return endpoint->status;
        }
        dbg_ep0 (0, "endpoint: %02x NOT FOUND", endpoint_address);
	return 0;
}


/**
 * bi_device_feature - handle set/clear feature requests
 * @device: device
 * @endpoint: endpoint to check
 * @flag: set or clear
 *
 * Used by the USB Device Core to check endpoint halt status.
 */
int bi_device_feature (struct usb_device_instance *device, int endpoint_address, int flag)
{
	int ep = 0;
        struct usb_endpoint_instance *endpoint = NULL;
        dbg_ep0 (0, "endpoint: %d flag: %d", endpoint_address, flag);
        if (device && device->bus && device->bus->endpoint_array) {
                endpoint = device->bus->endpoint_array;
                for (ep = 0; ep < udc_max_endpoints (); ep++) {
                        if (endpoint->endpoint_address == endpoint_address)
                                break;
                        endpoint++;
                }
                if (ep == udc_max_endpoints ())
                        endpoint = NULL;
        }
        if (endpoint) {

                dbg_ep0 (1, "endpoint: %d status: %d", ep, endpoint->status);
                if (flag && !endpoint->status) {
                        dbg_ep0 (1, "stalling endpoint");
                        udc_stall_ep (ep);
                        endpoint->status = 1;
                }
                else if (!flag && endpoint->status){
                        dbg_ep0 (1, "reseting endpoint %d", ep);
                        udc_reset_ep (ep);
                        endpoint->status = 0;
                }
                return 0;
        }
        dbg_ep0 (0, "endpoint: %d NOT FOUND", endpoint_address);
        return -EINVAL;
}

/* 
 * bi_disable_endpoints - disable udc and all endpoints
 */
static void bi_disable_endpoints (struct usb_device_instance *device)
{
	int i;
	if (device && device->bus && device->bus->endpoint_array) {
		for (i = 0; i < udc_max_endpoints (); i++) {
			struct usb_endpoint_instance *endpoint;
			if ((endpoint = device->bus->endpoint_array + i)) {
				usbd_flush_ep (endpoint);
			}
		}
	}
}

/* bi_config - commission bus interface driver
 */
static int bi_config (struct usb_device_instance *device)
{
	int i;

	struct usb_interface_descriptor *interface;
	struct usb_endpoint_descriptor *endpoint_descriptor;

	int found_tx = 0;
	int found_rx = 0;

	dbg_init (1, "checking config: config: %d interface: %d alternate: %d",
		  device->configuration, device->interface, device->alternate);

	bi_disable_endpoints (device);

	// audit configuration for compatibility
	if (!(interface = usbd_device_interface_descriptor (device,
                        0, device->configuration, device->interface, device->alternate))) 
        {
		dbg_init (0, "cannot fetch interface descriptor c:%d i:%d a:%d",
			  device->configuration, device->interface, device->alternate);
		return -EINVAL;
	}


	dbg_init (2, "---> endpoints: %d", interface->bNumEndpoints);

	// iterate across all endpoints for this configuration and verify they are valid 
	for (i = 0; i < interface->bNumEndpoints; i++) {
		int transfersize;

		int physical_endpoint;
		int logical_endpoint;

		//dbg_init(0, "fetching endpoint %d", i);
                if (!(endpoint_descriptor = usbd_device_endpoint_descriptor_index (device, 0,
                                device-> configuration, device-> interface, device-> alternate, i))) 
                {
			dbg_init (0, "cannot fetch endpoint descriptor: %d", i);
			continue;
		}
		// XXX check this
                transfersize = usbd_device_endpoint_transfersize (device, 0,
                        device->configuration, device->interface, device->alternate, i); 


                logical_endpoint = endpoint_descriptor->bEndpointAddress;

		if (!
		    (physical_endpoint =
		     udc_check_ep (logical_endpoint, 
		     le16_to_cpu(endpoint_descriptor->wMaxPacketSize)))) {
			dbg_init (2,
				  "endpoint[%d]: l: %02x p: %02x transferSize: %d packetSize: %02x INVALID",
				  i, logical_endpoint, physical_endpoint, transfersize,
				  le16_to_cpu(endpoint_descriptor->wMaxPacketSize));

			dbg_init (0, "invalid endpoint: %d %d", logical_endpoint, physical_endpoint);
			return -EINVAL;
		} 
                else {
			struct usb_endpoint_instance *endpoint =
			    device->bus->endpoint_array + physical_endpoint;

			dbg_init (2, "endpoint[%d]: l: %02x p: %02x transferSize: %d packetSize: %02x FOUND",
				  i, logical_endpoint, physical_endpoint, transfersize,
				  le16_to_cpu(endpoint_descriptor->wMaxPacketSize));

			dbg_init (2, "epd->bEndpointAddress=%02x", endpoint_descriptor->bEndpointAddress);
                        endpoint->endpoint_address = endpoint_descriptor->bEndpointAddress;

			if (le16_to_cpu(endpoint_descriptor->wMaxPacketSize) > 64) {
				dbg_init (0, "incompatible with endpoint size: %x", 
					le16_to_cpu(endpoint_descriptor->wMaxPacketSize));
				return -EINVAL;
			}

			if (endpoint_descriptor->bEndpointAddress & IN) {
				found_tx++;
				endpoint->tx_attributes = endpoint_descriptor->bmAttributes;
				endpoint->tx_transferSize = transfersize & 0xfff;
				endpoint->tx_packetSize = le16_to_cpu(endpoint_descriptor->wMaxPacketSize);
				endpoint->last = 0;
                                if (endpoint->tx_urb) {
                                        dbg_init (1, "CLEARING tx_urb: %p", endpoint->tx_urb);
                                        usbd_dealloc_urb (endpoint->tx_urb);
                                        endpoint->tx_urb = NULL;
                                }
			} else {
				found_rx++;
				endpoint->rcv_attributes = endpoint_descriptor->bmAttributes;
				endpoint->rcv_transferSize = transfersize & 0xfff;
				endpoint->rcv_packetSize = 
					le16_to_cpu(endpoint_descriptor->wMaxPacketSize);
                                if (endpoint->rcv_urb) {
                                        dbg_init (1, "CLEARING rcv_urb: %p", endpoint->tx_urb);
                                        usbd_dealloc_urb (endpoint->rcv_urb);
                                        endpoint->rcv_urb = NULL;
                                }
			}
		}
	}

	// iterate across all endpoints and enable them

        dbg_init(1, "---> device->status: %d", device->status);

	if (device->status == USBD_OK) {
                dbg_init (1, "enabling endpoints");
		for (i = 1; i < device->bus->driver->max_endpoints; i++) {

			struct usb_endpoint_instance *endpoint = device->bus->endpoint_array + i;
			dbg_init (1, "endpoint[%d]: %p addr: %02x transferSize: %d:%d packetSize: %d:%d SETUP",
				  i, endpoint, endpoint->endpoint_address,
				  endpoint->rcv_transferSize, endpoint->tx_transferSize,
				  endpoint->rcv_packetSize, endpoint->tx_packetSize);

			//udc_setup_ep(device, i, endpoint->endpoint_address?  endpoint : NULL);
			udc_setup_ep (device, i, endpoint);
		}
	}

	return 0;
}


/**
 * bi_device_event - handle generic bus event
 * @device: device pointer
 * @event: interrupt event
 *
 * Called by usb core layer to inform bus of an event.
 */
int bi_device_event (struct usb_device_instance *device, usb_device_event_t event, int data)
{

	//printk(KERN_DEBUG "bi_device_event: event: %d\n", event);

	if (!device) {
		return 0;
	}

        dbg_usbe (1,"%s", USBD_DEVICE_EVENTS(event));

	switch (event) {
	case DEVICE_UNKNOWN:
		break;
	case DEVICE_INIT:
		break;
	case DEVICE_CREATE:	// XXX should this stuff be in DEVICE_INIT?
		// enable upstream port

		//ep0_enable(device);

		// for net_create
		bi_config (device);

		// enable udc, enable interrupts, enable connect
                printk(KERN_INFO"bi_device_event: call udc_enable\n");
		udc_enable (device);
                printk(KERN_INFO"bi_device_event: call udc_all_interrupts\n");

                // XXX verify
		udc_suspended_interrupts (device);
		//udc_all_interrupts (device);

		dbg_usbe (1, "CREATE done");
		break;

	case DEVICE_HUB_CONFIGURED:
		udc_connect ();
		break;

	case DEVICE_RESET:
		device->address = 0;
		udc_set_address (device->address);
		udc_reset_ep (0);
                
                // XXX verify
		udc_suspended_interrupts (device);
		dbg_usbe (1, "DEVICE RESET done: %d", device->address);
		break;


	case DEVICE_ADDRESS_ASSIGNED:
		udc_set_address (device->address);
		device->status = USBD_OK;

                // XXX verify
		udc_all_interrupts (device);	// XXX
		break;

	case DEVICE_CONFIGURED:
		device->status = USBD_OK;
		bi_config (device);
		break;

	case DEVICE_DE_CONFIGURED:
		{
			int ep;

			for (ep = 1; ep < udc_max_endpoints (); ep++)
				udc_reset_ep (ep);
		}
		break;


	case DEVICE_SET_INTERFACE:
		bi_config (device);
		break;

	case DEVICE_SET_FEATURE:
		break;

	case DEVICE_CLEAR_FEATURE:
		break;

	case DEVICE_BUS_INACTIVE:
		// disable suspend interrupt
		udc_suspended_interrupts (device);

		// XXX check on linkup and sl11
		// if we are no longer connected then force a reset
		if (!udc_connected ()) {
			usbd_device_event_irq (device, DEVICE_RESET, 0);
		}
		break;

	case DEVICE_BUS_ACTIVITY:
		// enable suspend interrupt
		udc_all_interrupts (device);
		break;

	case DEVICE_POWER_INTERRUPTION:
		break;

	case DEVICE_HUB_RESET:
		break;

	case DEVICE_DESTROY:
		udc_disconnect ();
		bi_disable_endpoints (device);
		udc_disable_interrupts (device);
		udc_disable ();
		break;

	case DEVICE_FUNCTION_PRIVATE:
		break;
	}
	return 0;
}

/**
 * bi_send_urb - start transmit
 * @urb:
 */
int bi_send_urb (struct urb *urb)
{
	unsigned long flags;
	dbg_tx (4, "urb: %p", urb);
	local_irq_save (flags);
	if (urb && urb->endpoint && !urb->endpoint->tx_urb) {
		dbg_tx (2, "urb: %p endpoint: %x", urb, urb->endpoint->endpoint_address);
		usbd_tx_complete_irq (urb->endpoint, 0);
		udc_start_in_irq (urb->endpoint);
	}
	local_irq_restore (flags);

	// Shouldn't need to make this atomic, all we need is a change indicator
	urb->device->usbd_rxtx_timestamp = jiffies;

	return 0;
}


#if defined(CONFIG_PM) && !defined(CONFIG_USBD_MONITOR) && !defined(CONFIG_USBD_MONITOR_MODULE)
static int bi_pm_event (struct pm_dev *pm_dev, pm_request_t request, void *unused);
#endif



struct usb_bus_operations bi_ops = {
	send_urb:bi_send_urb,
	cancel_urb:bi_cancel_urb,
	endpoint_halted:bi_endpoint_halted,
	device_feature:bi_device_feature,
	device_event:bi_device_event,
};

struct usb_bus_driver bi_driver = {
	name:"",
	max_endpoints:0,
	ops:&bi_ops,
	this_module:THIS_MODULE,
};


/* Bus Interface Received Data *************************************************************** */

struct usb_device_instance *device_array[MAX_DEVICES];


/* Bus Interface Support Functions *********************************************************** */

/**
 * udc_cable_event - called from cradle interrupt handler
 */
void udc_cable_event (void)
{
	struct usb_bus_instance *bus;
	struct usb_device_instance *device;
	struct bi_data *data;

	dbgENTER (dbgflg_usbdbi_init, 1);

	// sanity check
	if (!(device = device_array[0]) || !(bus = device->bus) || !(data = bus->privdata)) {
		return;
	}

	{
		unsigned long flags;
		local_irq_save (flags);
		if (udc_connected ()) {
			dbg_init (1, "state: %d connected: %d", device->device_state, 1);;
			if (device->device_state == STATE_ATTACHED) {
				dbg_init (1, "LOADING");
				usbd_device_event_irq (device, DEVICE_HUB_CONFIGURED, 0);
				usbd_device_event_irq (device, DEVICE_RESET, 0);
			}
		} else {
			dbg_init (1, "state: %d connected: %d", device->device_state, 0);;
			if (device->device_state != STATE_ATTACHED) {
				dbg_init (1, "UNLOADING");
				usbd_device_event_irq (device, DEVICE_RESET, 0);
				usbd_device_event_irq (device, DEVICE_POWER_INTERRUPTION, 0);
				usbd_device_event_irq (device, DEVICE_HUB_RESET, 0);
			}
		}
		local_irq_restore (flags);
	}
	dbgLEAVE (dbgflg_usbdbi_init, 1);
}


/* Module Init - the device init and exit routines ******************************************* */

/**
 * bi_udc_init - initialize USB Device Controller
 * 
 * Get ready to use the USB Device Controller.
 *
 * Register an interrupt handler and IO region. Return non-zero for error.
 */
int bi_udc_init (void)
{
	dbg_init (1, "Loading %s", udc_name ());

	bi_driver.name = udc_name ();
	bi_driver.max_endpoints = udc_max_endpoints ();
	bi_driver.maxpacketsize = udc_ep0_packetsize ();

	dbg_init (1, "name: %s endpoints: %d ep0: %d", bi_driver.name, bi_driver.max_endpoints,
		  bi_driver.maxpacketsize);

	// request device IRQ
	if (udc_request_udc_irq ()) {
		dbg_init (0, "name: %s request udc irq failed", udc_name ());
		return -EINVAL;
	}
	// request device IO
	if (udc_request_io ()) {
		udc_release_udc_irq ();
		dbg_init (0, "name: %s request udc io failed", udc_name ());
		return -EINVAL;
	}
	// probe for device
	if (udc_init ()) {
		udc_release_udc_irq ();
		udc_release_io ();
		dbg_init (1, "name: %s probe failed", udc_name ());
		return -EINVAL;
	}

	// optional cable IRQ 
	have_cable_irq = !udc_request_cable_irq ();
	dbg_init (1, "name: %s request cable irq %d", udc_name (), have_cable_irq);

	return 0;
}


/**
 * bi_udc_exit - Stop using the USB Device Controller
 *
 * Stop using the USB Device Controller.
 *
 * Shutdown and free dma channels, de-register the interrupt handler.
 */
void bi_udc_exit (void)
{
	int i;

	dbg_init (1, "Unloading %s", udc_name ());

	for (i = 0; i < udc_max_endpoints (); i++) {
		udc_disable_ep (i);
	}

	// free io and irq
	udc_disconnect ();
	udc_disable ();
	udc_release_io ();
	udc_release_udc_irq ();

	if (have_cable_irq) {
		udc_release_cable_irq ();
	}
}


/* Module Init - the module init and exit routines ************************************* */

#if 0
/* bi_init - commission bus interface driver
 */
static int bi_init (void)
{
	return 0;
}
#endif

/* bi_exit - decommission bus interface driver
 */
static void bi_exit (void)
{
}

#if defined(CONFIG_PM) && !defined(CONFIG_USBD_MONITOR) && !defined(CONFIG_USBD_MONITOR_MODULE)
/* Module Init - Power management ************************************************************ */

/* The power management scheme is simple. Simply do the following:
 *
 *      Event           Call            Equivalent
 *      ------------------------------------------
 *      PM_SUSPEND      bi_exit();      rmmod
 *      PM_RESUME       bi_init();      insmod
 *
 */

static int pm_suspended;

/*
 * usbd_pm_callback
 * @dev:
 * @rqst:
 * @unused:
 *
 * Used to signal power management events.
 */
static int bi_pm_event (struct pm_dev *pm_dev, pm_request_t request, void *unused)
{
	struct usb_device_instance *device;

	dbg_pm (0, "request: %d pm_dev: %p data: %p", request, pm_dev, pm_dev->data);

	if (!(device = pm_dev->data)) {
		dbg_pm (0, "DATA NULL, NO DEVICE");
		return 0;
	}

	switch (request) {
#if defined(CONFIG_IRIS)
	case PM_STANDBY:
	case PM_BLANK:
#endif
	case PM_SUSPEND:
		dbg_pm (0, "PM_SUSPEND");
		if (!pm_suspended) {
			pm_suspended = 1;
			dbg_init (1, "MOD_INC_USE_COUNT %d", GET_USE_COUNT (THIS_MODULE));
			udc_disconnect ();	// disable USB pullup if we can
			udc_disable_interrupts (device);	// disable interupts
			udc_disable ();	// disable UDC
			dbg_pm (0, "PM_SUSPEND: finished");
		}
		break;
#if defined(CONFIG_IRIS)
	case PM_UNBLANK:
#endif
	case PM_RESUME:
		dbg_pm (0, "PM_RESUME");
		if (pm_suspended) {
			// probe for device
			if (udc_init ()) {
				dbg_init (0, "udc_init failed");
				//return -EINVAL;
			}
			udc_enable (device);	// enable UDC
			udc_all_interrupts (device);	// enable interrupts
			udc_connect ();	// enable USB pullup if we can
			//udc_set_address(device->address);
			//udc_reset_ep(0);

			pm_suspended = 0;
			dbg_init (1, "MOD_INC_USE_COUNT %d", GET_USE_COUNT (THIS_MODULE));
			dbg_pm (0, "PM_RESUME: finished");
		}
		break;
	}
	return 0;
}
#endif

#ifdef CONFIG_DPM	/* MVL-CEE */
static int bi_udc_suspend(struct device *dev, u32 state, u32 level);
static int bi_udc_resume(struct device *dev, u32 level);
static int bi_udc_scale(struct bus_op_point *op, u32 level);

static struct device_driver bi_udc_driver_ldm = {
	.name		= "usb-device",
	.devclass	= NULL,
	.probe		= NULL,
	.suspend	= bi_udc_suspend,
	.resume		= bi_udc_resume,
	.scale		= bi_udc_scale,
	.remove		= NULL,
};

static struct device bi_udc_device_ldm = {
	.name		= "USB Device",
	.bus_id		= "usbd",
	.driver		= NULL,
	.power_state	= DPM_POWER_ON,
};

static void
bi_udc_ldm_driver_register(void)
{
#if defined(CONFIG_OMAP_INNOVATOR) || defined (CONFIG_MACH_OMAP_PERSEUS2) 
	extern void mpu_public_driver_register(struct device_driver *driver);
	mpu_public_driver_register(&bi_udc_driver_ldm);
#endif
#ifdef CONFIG_ARCH_MAINSTONE
	extern void pxaopb_driver_register(struct device_driver *driver);
	pxaopb_driver_register(&bi_udc_driver_ldm);
#endif
#ifdef CONFIG_ARCH_MX2ADS
	extern void mx21_ldm_driver_register(struct device_driver *driver);
	mx21_ldm_driver_register(&bi_udc_driver_ldm);
#endif
}

static void
bi_udc_ldm_device_register(void)
{
#if defined (CONFIG_OMAP_INNOVATOR) || defined (CONFIG_MACH_OMAP_PERSEUS2)
	extern void mpu_public_device_register(struct device *device);
	mpu_public_device_register(&bi_udc_device_ldm);
#endif
#ifdef CONFIG_ARCH_MAINSTONE
	extern void pxaopb_device_register(struct device *device);
	pxaopb_device_register(&bi_udc_device_ldm);
#endif
#ifdef CONFIG_ARCH_MX2ADS
	extern void mx21_ldm_device_register(struct device *device);
	mx21_ldm_device_register(&bi_udc_device_ldm);
#endif
}

static void
bi_udc_ldm_driver_unregister(void)
{
#if defined (CONFIG_OMAP_INNOVATOR) || defined (CONFIG_MACH_OMAP_PERSEUS2)
	extern void mpu_public_driver_unregister(struct device_driver *driver);
	mpu_public_driver_unregister(&bi_udc_driver_ldm);
#endif
#ifdef CONFIG_ARCH_MAINSTONE
	extern void pxaopb_driver_unregister(struct device_driver *driver);
	pxaopb_driver_unregister(&bi_udc_driver_ldm);
#endif
#ifdef CONFIG_ARCH_MX2ADS
	extern void mx21_ldm_driver_unregister(struct device_driver *driver);
	mx21_ldm_driver_unregister(&bi_udc_driver_ldm);
#endif
}

static void
bi_udc_ldm_device_unregister(void)
{
#if defined(CONFIG_OMAP_INNOVATOR) || defined (CONFIG_MACH_OMAP_PERSEUS2)
	extern void mpu_public_device_unregister(struct device *device);
	mpu_public_device_unregister(&bi_udc_device_ldm);
#endif
#ifdef CONFIG_ARCH_MAINSTONE
	extern void pxaopb_device_unregister(struct device *device);
	pxaopb_device_unregister(&bi_udc_device_ldm);
#endif
#ifdef CONFIG_ARCH_MX2ADS
	extern void mx21_ldm_device_unregister(struct device *device);
	mx21_ldm_device_unregister(&bi_udc_device_ldm);
#endif
}

static int
bi_udc_scale(struct bus_op_point *op, u32 level)
{
  printk("+++: bi_udc_scale\n");
	/* REVISIT */
	return 0;
}

static int
bi_udc_suspend(struct device *dev, u32 state, u32 level)
{
	struct usb_device_instance *device = device_array[0];

	switch (level) {
	case SUSPEND_POWER_DOWN:
		dbg_pm (0, "SUSPEND_POWER_DOWN");
		dbg_init (1, "MOD_INC_USE_COUNT %d", 
			GET_USE_COUNT (THIS_MODULE));
		udc_disconnect ();		/* disable USB pullup */
		if (device)
			udc_disable_interrupts (device);	/* disable interrupts */
		udc_disable ();			/* disable UDC */
		dbg_pm (0, "SUSPEND_POWER_DOWN: finished");
		break;
	}

	return 0;
}

static int
bi_udc_resume(struct device *dev, u32 level)
{
	int ret = 0;
	struct usb_device_instance *device = device_array[0];

	switch (level) {
	case RESUME_POWER_ON:
		dbg_pm (0, "RESUME_POWER_ON");
		if (udc_init ()) {
			dbg_init (0, "udc_init failed");
		}
		if (device) {
			udc_enable (device);		/* enable UDC */
			udc_all_interrupts (device);	/* enable interrupts */
		}
		udc_connect ();			/* enable USB pullup */
		udc_reset_ep(0);
		dbg_init (1, "MOD_INC_USE_COUNT %d", 
			GET_USE_COUNT (THIS_MODULE));
		dbg_pm (0, "RESUME_POWER_ON: finished");
		break;
	}

	return ret;
}
#endif	/* ifdef CONFIG_DPM */

#ifdef CONFIG_USBD_PROCFS
/* Proc Filesystem *************************************************************************** */

/* *
 * usbd_proc_read - implement proc file system read.
 * @file
 * @buf
 * @count
 * @pos
 *
 * Standard proc file system read function.
 */
static ssize_t usbd_proc_read (struct file *file, char *buf, size_t count, loff_t * pos)
{
        struct usb_device_instance *device;
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

	switch (index) {
        case 0:
		len += sprintf ((char *) page + len, "USBD Status\n");
                break;

        case 1:
		len += sprintf ((char *) page + len, "Cable: %s\n", udc_connected ()? "Plugged" : "Unplugged");
                break;

        case 2:
                if ((device = device_array[0])) {
                        struct usb_function_instance * function_instance;
                        struct usb_bus_instance * bus;

                        len += sprintf ((char *) page + len, "Device status: %s\n", USBD_DEVICE_STATUS(device->status));
                        len += sprintf ((char *) page + len, "Device state: %s\n", USBD_DEVICE_STATE(device->device_state));

                        if ((function_instance = device->function_instance_array+0)) {
                                len += sprintf ((char *) page + len, "Function: %s\n", 
                                                function_instance->function_driver->name);
                        }
                        if ((bus= device->bus)) {
                                len += sprintf ((char *) page + len, "Bus interface: %s\n", 
                                                bus->driver->name);
                        }
                }
                break;

        default:
                break;
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
 * usbd_proc_write - implement proc file system write.
 * @file
 * @buf
 * @count
 * @pos
 *
 * Proc file system write function, used to signal monitor actions complete.
 * (Hotplug script (or whatever) writes to the file to signal the completion
 * of the script.)  An ugly hack.
 */
static ssize_t usbd_proc_write (struct file *file, const char *buf, size_t count, loff_t * pos)
{
        struct usb_device_instance *device;
	size_t n = count;
	char command[64];
	char *cp = command;
	int i = 0;

	MOD_INC_USE_COUNT;
	//printk(KERN_DEBUG "%s: count=%u\n",__FUNCTION__,count);
	while ((n > 0) && (i < 64)) {
		// Not too efficient, but it shouldn't matter
		if (copy_from_user (cp++, buf + (count - n), 1)) {
			count = -EFAULT;
			break;
		}
		*cp = '\0';
		i++;
		n -= 1;
		//printk(KERN_DEBUG "%s: %u/%u %02x\n",__FUNCTION__,count-n,count,c);
	}
	if (!strncmp (command, "plug", 4)) {
		udc_connect ();
	} 
        else if (!strncmp (command, "unplug", 6)) {
		udc_disconnect ();
                if ((device = device_array[0])) {
                        usbd_device_event (device, DEVICE_RESET, 0);
                }
	}
	MOD_DEC_USE_COUNT;
	return (count);
}

static struct file_operations usbd_proc_operations_functions = {
	read:usbd_proc_read,
	write:usbd_proc_write,
};

#endif

/* Module Init - module loading init and exit routines *************************************** */

/* We must be able to use the real bi_init() and bi_exit() functions 
 * from here and the power management event function.
 *
 * We handle power management registration issues from here.
 *
 */

/* bi_modinit - commission bus interface driver
 */
static int __init bi_modinit (void)
{
	struct usb_bus_instance *bus;
	struct usb_device_instance *device;
	struct bi_data *data;


	printk (KERN_INFO "usbd-bi: (dbg=\"%s\")\n", dbg ? dbg : "");

	// process debug options
	if (0 != scan_debug_options ("usbd-bi", dbg_table, dbg)) {
		return -EINVAL;
	}
	// check if we can see the UDC
	if (bi_udc_init ()) {
		return -EINVAL;
	}
	// allocate a bi private data structure
	if ((data = kmalloc (sizeof (struct bi_data), GFP_KERNEL)) == NULL) {
		bi_udc_exit ();
		return -EINVAL;
	}
	memset (data, 0, sizeof (struct bi_data));

	// register this bus interface driver and create the device driver instance
	if ((bus = usbd_register_bus (&bi_driver)) == NULL) {
		kfree (data);
		bi_udc_exit ();
		return -EINVAL;
	}

	bus->privdata = data;

	// see if we can scrounge up something to set a sort of unique device address
	if (!udc_serial_init (bus)) {
		dbg_init (1, "serial: %s %04x\n", bus->serial_number_str, bus->serial_number);
	}


	if ((device = usbd_register_device (NULL, bus, 8)) == NULL) {
		usbd_deregister_bus (bus);
		kfree (data);
		bi_udc_exit ();
		return -EINVAL;
	}
#if defined(CONFIG_PM) && !defined(CONFIG_USBD_MONITOR) && !defined(CONFIG_USBD_MONITOR_MODULE)
	// register with power management 
	pm_dev = pm_register (PM_USB_DEV, PM_SYS_UNKNOWN, bi_pm_event);
	pm_dev->data = device;
#endif

#ifdef CONFIG_DPM	/* MVL-CEE */
	bi_udc_ldm_driver_register();
	bi_udc_ldm_device_register();
#endif

	bus->device = device;
	device_array[0] = device;

	// initialize the device
	{
		struct usb_endpoint_instance *endpoint = device->bus->endpoint_array + 0;

		// setup endpoint zero

		endpoint->endpoint_address = 0;

		endpoint->tx_attributes = 0;
		endpoint->tx_transferSize = 255;
		endpoint->tx_packetSize = udc_ep0_packetsize ();

		endpoint->rcv_attributes = 0;
		endpoint->rcv_transferSize = 0x8;
		endpoint->rcv_packetSize = udc_ep0_packetsize ();

		udc_setup_ep (device, 0, endpoint);
	}

	// hopefully device enumeration will finish this process
        printk(KERN_INFO"bi_modinit: call udc_startup_events\n");
	udc_startup_events (device);

#if defined(CONFIG_PM) && !defined(CONFIG_USBD_MONITOR) && !defined(CONFIG_USBD_MONITOR_MODULE)
	dbg_pm (0, "pm_dev->callback#%p", pm_dev->callback);
	if (!udc_connected ()) {
		/* Fake a call from the PM system to suspend the UDC until it
		   is needed (cable connect, etc) */
		(void) bi_pm_event (pm_dev, PM_SUSPEND, NULL);
		/* There appears to be no constant for this, but inspection
		   of arch/arm/mach-l7200/apm.c:send_event() shows that the
		   suspended state is 3 (i.e. pm_send_all(PM_SUSPEND, (void *)3))
		   corresponding to ACPI_D3. */
		pm_dev->state = 3;
	}
#endif
	if (dbgflg_usbdbi_tick > 0) {
		// start ticker
		ticker_kickoff ();
	}

#ifdef CONFIG_USBD_PROCFS
	{
		struct proc_dir_entry *p;

		// create proc filesystem entries
		if ((p = create_proc_entry ("usbd", 0, 0)) == NULL) {
			return -ENOMEM;
		}
		p->proc_fops = &usbd_proc_operations_functions;
	}
#endif
	dbgLEAVE (dbgflg_usbdbi_init, 1);
	return 0;
}

/* bi_modexit - decommission bus interface driver
 */
static void __exit bi_modexit (void)
{
	struct usb_bus_instance *bus;
	struct usb_device_instance *device;
	struct bi_data *data;

	dbgENTER (dbgflg_usbdbi_init, 1);

#ifdef CONFIG_USBD_PROCFS
	remove_proc_entry ("usbd", NULL);
#endif

        udc_disconnect ();
        udc_disable ();

	if ((device = device_array[0])) {

		// XXX moved to usbd_deregister_device()
		//device->status = USBD_CLOSING;

		// XXX XXX
		if (dbgflg_usbdbi_tick > 0) {
			ticker_killoff ();
		}

		bus = device->bus;
		data = bus->privdata;

		// XXX
		usbd_device_event (device, DEVICE_RESET, 0);
		usbd_device_event (device, DEVICE_POWER_INTERRUPTION, 0);
		usbd_device_event (device, DEVICE_HUB_RESET, 0);

		dbg_init (1, "DEVICE_DESTROY");
		usbd_device_event (device, DEVICE_DESTROY, 0);


		dbg_init (1, "DISABLE ENDPOINTS");
		bi_disable_endpoints (device);

		//dbg_init(1,"UDC_DISABLE");
		//udc_disable();

		dbg_init (1, "BI_UDC_EXIT");
		bi_udc_exit ();

		device_array[0] = NULL;
		//bus->privdata = NULL; // XXX moved to usbd-bus.c usbd_deregister_device()


#if defined(CONFIG_PM) && !defined(CONFIG_USBD_MONITOR) && !defined(CONFIG_USBD_MONITOR_MODULE)
		dbg_init (1, "PM_UNREGISTER(pm_dev#%p)", pm_dev);
		if (pm_dev) {
			pm_unregister_all(bi_pm_event);
		}
#endif

#ifdef CONFIG_DPM	/* MVL-CEE */
		bi_udc_ldm_device_unregister();
		bi_udc_ldm_driver_unregister();
#endif

		dbg_init (1, "DEREGISTER DEVICE");
		usbd_deregister_device (device);
		bus->device = NULL;

		dbg_init (1, "kfree(data#%p)", data);
		if (data) {
			kfree (data);
		}

		if (bus->serial_number_str) {
			kfree (bus->serial_number_str);
		}

		dbg_init (1, "DEREGISTER BUS");
		usbd_deregister_bus (bus);

	} else {
		dbg_init (0, "device is NULL");
	}
	dbg_init (1, "BI_EXIT");
	bi_exit ();
	dbgLEAVE (dbgflg_usbdbi_init, 1);
}



/* ticker */

/* Clock Tick Debug support ****************************************************************** */


#define RETRYTIME 10

int ticker_terminating;
int ticker_timer_set;

unsigned int udc_interrupts;
unsigned int udc_interrupts_last;

static DECLARE_MUTEX_LOCKED (ticker_sem_start);
static DECLARE_MUTEX_LOCKED (ticker_sem_work);

void ticker_tick (unsigned long data)
{
	ticker_timer_set = 0;
	up (&ticker_sem_work);
}

void udc_ticker_poke (void)
{
	up (&ticker_sem_work);
}

int ticker_thread (void *data)
{
	struct timer_list ticker;

	// detach

	lock_kernel ();
	exit_mm (current);
	exit_files (current);
	exit_fs (current);

	// setsid equivalent, used at start of kernel thread, no error checks needed, or at least none made :). 
	current->leader = 1;
	current->session = current->pgrp = current->pid;
	current->tty = NULL;
	current->tty_old_pgrp = 0;

	// Name this thread 
	sprintf (current->comm, "usbd-bi");

	// setup signal handler
	current->exit_signal = SIGCHLD;
	spin_lock (&current->sigmask_lock);
	flush_signals (current);
	spin_unlock (&current->sigmask_lock);

	// XXX Run at a high priority, ahead of sync and friends
	// current->nice = -20;
	current->policy = SCHED_OTHER;

	unlock_kernel ();

	// setup timer
	init_timer (&ticker);
	ticker.data = 0;
	ticker.function = ticker_tick;

	// let startup continue
	up (&ticker_sem_start);

	// process loop
	for (ticker_timer_set = ticker_terminating = 0; !ticker_terminating;) {

		char buf[100];
		char *cp;

		if (!ticker_timer_set) {
			mod_timer (&ticker, jiffies + HZ * RETRYTIME);
		}
		// wait for someone to tell us to do something
		down (&ticker_sem_work);

		if (udc_interrupts != udc_interrupts_last) {
			dbg_tick (3, "--------------");
		}
		// do some work
		memset (buf, 0, sizeof (buf));
		cp = buf;

		if (dbgflg_usbdbi_tick) {
			unsigned long flags;
			local_irq_save (flags);
			dbg_tick (2, "[%d]", udc_interrupts);
			udc_regs ();
			local_irq_restore (flags);
		}

#if 0
		// XXX
#if defined(CONFIG_SA1110_CALYPSO) && defined(CONFIG_PM) && defined(CONFIG_USBD_TRAFFIC_KEEPAWAKE)
		/* Check for rx/tx activity, and reset the sleep timer if present */
		if (device->usbd_rxtx_timestamp != device->usbd_last_rxtx_timestamp) {
			extern void resetSleepTimer (void);
			dbg_tick (7, "resetting sleep timer");
			resetSleepTimer ();
			device->usbd_last_rxtx_timestamp = device->usbd_rxtx_timestamp;
		}
#endif
#endif
#if 0
		/* Check for TX endpoint stall.  If the endpoint has
		   stalled, we have probably run into the DMA/TCP window
		   problem, and the only thing we can do is disconnect
		   from the bus, then reconnect (and re-enumerate...). */
		if (reconnect > 0 && 0 >= --reconnect) {
			dbg_init (0, "TX stall disconnect finished");
			udc_connect ();
		} else if (0 != USBD_STALL_TIMEOUT_SECONDS) {
			// Stall watchdog unleashed
			unsigned long now, tx_waiting;
			now = jiffies;
			tx_waiting = (now - tx_queue_head_timestamp (now)) / HZ;
			if (tx_waiting > USBD_STALL_TIMEOUT_SECONDS) {
				/* The URB at the head of the queue has waited too long */
				reconnect = USBD_STALL_DISCONNECT_DURATION;
				dbg_init (0, "TX stalled, disconnecting for %d seconds", reconnect);
				udc_disconnect ();
			}
		}
#endif
	}

	// remove timer
	del_timer (&ticker);

	// let the process stopping us know we are done and return
	up (&ticker_sem_start);
	return 0;
}

/**
 * kickoff_thread - start management thread
 */
void ticker_kickoff (void)
{
	ticker_terminating = 0;
	kernel_thread (&ticker_thread, NULL, 0);
	down (&ticker_sem_start);
}

/**
 * killoff_thread - stop management thread
 */
void ticker_killoff (void)
{
	if (!ticker_terminating) {
		ticker_terminating = 1;
		up (&ticker_sem_work);
		down (&ticker_sem_start);
	}
}


/* module */

module_init (bi_modinit);
module_exit (bi_modexit);
