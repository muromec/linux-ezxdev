/*
 * linux/drivers/usbd/net_fd/net-fd.c - network function driver
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *      Bruce Balden <balden@lineo.com>
 *
 * Changes copyright (c) 2003 MontaVista Software, Inc.
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
#include <asm/uaccess.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <asm/system.h>
#include <linux/if_ether.h>
#include <linux/autoconf.h>

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-inline.h"
#include "../usbd-arch.h"

#ifdef CONFIG_USBD_TEST_VENDORID
#undef  CONFIG_USBD_VENDORID
#define CONFIG_USBD_VENDORID CONFIG_USBD_TEST_VENDORID
#endif

#ifdef CONFIG_USBD_TEST_PRODUCTID
#undef CONFIG_USBD_PRODUCTID
#define CONFIG_USBD_PRODUCTID CONFIG_USBD_TEST_PRODUCTID
#endif

char* test_string="abcdefghijknmlopqrstuvwxyz";

void test_event ( struct usb_device_instance *device, usb_device_event_t event, int data )
{
	/* NOTHING*/
	printk(KERN_INFO"test_event create!**********************\n");
	printk(KERN_INFO"Now the comein event is %d\n", event );
}

int test_recv_setup( struct urb* urb )
{
	struct usb_device_request * request;
	struct usb_device_instance * device;
	int i;

	device = urb->device;
	request = &urb->device_request;

	/* Judge whether the driver want to handle the packet */
	switch ( request->bmRequestType&USB_REQ_DIRECTION_MASK ) {
	case 0x00:
		printk(KERN_INFO"Host to device.\n");

		printk(KERN_INFO"request->bmRequestType=0x%x\n", request->bmRequestType );
		printk(KERN_INFO"request->bRequest     =0x%x\n", request->bRequest );
		printk(KERN_INFO"request->wIndex       =0x%x\n", request->wIndex );
		printk(KERN_INFO"request->wValue       =0x%x\n", request->wValue );

		/* Get the data from the host and process it */
		printk(KERN_INFO"Packet data:\n");
		for (i=0;i<urb->actual_length;i++ ) printk("%c",*(urb->buffer+i));
		printk("\n");

		/* NOTE: don't forget to free urb->buffer */
		kfree( urb->buffer );

		break;
	case 0x80:
		printk("Device to host.\n");

                printk(KERN_INFO"request->bmRequestType=0x%x\n", request->bmRequestType );
                printk(KERN_INFO"request->bRequest     =0x%x\n", request->bRequest );
                printk(KERN_INFO"request->wIndex       =0x%x\n", request->wIndex );
                printk(KERN_INFO"request->wValue       =0x%x\n", request->wValue );

		/* Build one urb and send it to the host */
		/* Since this is just a test, we send a constant string back to the host */
		urb->buffer = test_string;
		urb->actual_length = strlen(test_string) + 1;
		break;
	}
	return 0;
}

int test_recv_urb( struct urb* urb )
{
	/* DO NOTHING */
	printk(KERN_INFO"test_recv_urb *****************************\n");
	return 0;
}

int test_urb_sent( struct urb* urb, int rc )
{
	/* DO NOTHING */
	printk(KERN_INFO"test_urb_sent *******************\n");
	return 0;
}

static void test_function_init( struct usb_bus_instance *bus,
				struct usb_device_instance* device,
				struct usb_function_instance* function_driver )
{
	printk(KERN_INFO"****************************\n");
}

static void test_function_exit( struct usb_device_instance *device )
{
	printk(KERN_INFO"CLOSING************************\n");
}

struct usb_function_operations test_ops = {
	event: test_event,
	recv_urb: test_recv_urb,
	recv_setup: test_recv_setup,
	urb_sent: test_urb_sent,
	function_init: test_function_init,
	function_exit: test_function_exit,
};

static struct usb_endpoint_description test_default[] = {
	{bEndpointAddress: 1,
	 bmAttributes:BULK,
	 wMaxPacketSize:64,
	 bInterval:0,
	 direction:OUT,
	 transferSize:64, },

	{bEndpointAddress: 2,
	 bmAttributes:BULK,
	 wMaxPacketSize: 64,
	 bInterval:0,
	 direction:IN,
	 transferSize:64, },
};

/* Data Interface Alternate description(s)
 */
static __devinitdata struct usb_alternate_description test_data_alternate_descriptions[] = {
	{iInterface:"Simple Ep0 test Interface - Bulk mode",
	 bAlternateSetting:0,
	 endpoints:sizeof (test_default) /
         sizeof (struct usb_endpoint_description),
	 endpoint_list:test_default,},
};

/* Interface description(s)
 */
static __devinitdata struct usb_interface_description test_interfaces[] = {
	{iInterface:"Simple Serial Data Interface",
	 bInterfaceClass:LINEO_CLASS,
	 bInterfaceSubClass:LINEO_SUBCLASS_SAFESERIAL,
	 bInterfaceProtocol:LINEO_SAFESERIAL_CRC,
	 alternates:sizeof (test_data_alternate_descriptions) /
         sizeof (struct usb_alternate_description),
	 alternate_list:test_data_alternate_descriptions,},
};

struct __devinitdata usb_configuration_description test_description[] = {
	{iConfiguration:"USB Ep0 Test Configuration",
	 bmAttributes:0x40,
	 bMaxPower:1,
	 interfaces:sizeof (test_interfaces) /
	 sizeof (struct usb_interface_description),
	 interface_list:test_interfaces,},
};

/* Device Description
 */
struct __devinitdata usb_device_description test_device_description = {
	bDeviceClass:COMMUNICATIONS_DEVICE_CLASS,
	bDeviceSubClass:0,	// XXX
	bDeviceProtocol:0,	// XXX
	idVendor:CONFIG_USBD_VENDORID,
	idProduct:CONFIG_USBD_PRODUCTID,
	iManufacturer:CONFIG_USBD_MANUFACTURER,
	iProduct:CONFIG_USBD_PRODUCT_NAME,
	iSerialNumber:0,
};

struct usb_function_driver test_driver = {
	name: "Generic Ep0 Tester",
	ops: &test_ops,
	device_description:&test_device_description,
	configurations:sizeof (test_description) / sizeof (struct usb_configuration_description),
	configuration_description:test_description,
	this_module: THIS_MODULE,
};

static int __init test_modinit(void)
{
	if ( usbd_register_function(&test_driver) ) {
		return -EINVAL;
	}

	/* Here please register your own character device */
	return 0;
}

static void __exit test_modexit(void)
{
	usbd_deregister_function(&test_driver);
}

module_init( test_modinit );
module_exit( test_modexit );
