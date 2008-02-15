/*
 * usbd/network_fd/basic.c - Network Function Driver
 *
 *      Copyright (c) 2002, 2003, 2004 Belcarra
 *
 * By: 
 *      Chris Lynne <cl@belcarra.com>
 *      Stuart Lynne <sl@belcarra.com>
 *      Bruce Balden <balden@belcarra.com>
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

#include <usbd-export.h>
#include <usbd-build.h>

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/utsname.h>
#include <linux/netdevice.h>

#include <usbd-chap9.h>
#include <usbd-mem.h>
#include <usbd.h>
#include <usbd-func.h>

#include "network.h"


#ifdef CONFIG_USBD_NETWORK_BASIC
/* USB BASIC Configuration ******************************************************************** */

/* BASIC Communication Interface Class descriptors
 */
static __u8 basic_data_1[] = { 0x07, USB_DT_ENDPOINT, OUT, BULK,     0, 0x00, 0x00,  };
static __u8 basic_data_2[] = { 0x07, USB_DT_ENDPOINT, IN,  BULK,     0, 0x00, 0x00, };
static __u8 basic_comm_1[] = { 0x07, USB_DT_ENDPOINT, IN,  INTERRUPT,0, 0x00, 0x0a, };

static  struct usb_endpoint_descriptor  *basic_default[] = { 
        (struct usb_endpoint_descriptor *) &basic_data_1, 
        (struct usb_endpoint_descriptor *) &basic_data_2, 
        (struct usb_endpoint_descriptor *) &basic_comm_1, };
u8 basic_indexes[] = { BULK_OUT, BULK_IN, INT_IN, };

/* BASIC Data Interface Alternate endpoints
 */
static __u8 basic_data_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09, USB_DT_INTERFACE, 0x00, 0x00, // bInterfaceNumber, bAlternateSetting
        sizeof (basic_default) / sizeof(struct usb_endpoint_descriptor), // bNumEndpoints
        LINEO_CLASS, LINEO_SUBCLASS_BASIC_NET, LINEO_BASIC_NET_CRC, 0x00,
};

static struct usb_alternate_description basic_data_alternate_descriptions[] = {
	{ iInterface: CONFIG_USBD_NETWORK_BASIC_INTF,
                interface_descriptor: (struct usb_interface_descriptor *)&basic_data_alternate_descriptor,
                endpoints:sizeof (basic_default) / sizeof(struct usb_endpoint_descriptor *),
                endpoint_list: basic_default,
                endpoint_indexes: basic_indexes,
                },
};


/* BASIC Data Interface Alternate descriptions and descriptors
 */

/* BASIC Interface descriptions and descriptors
 */
struct usb_interface_description basic_interfaces[] = {
	{ alternates:sizeof (basic_data_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:basic_data_alternate_descriptions,},
};

/* BASIC Configuration descriptions and descriptors
 */
__u8 basic_configuration_descriptor[sizeof(struct usb_configuration_descriptor)] = {
        0x09, USB_DT_CONFIG, 0x00, 0x00, // wLength
        sizeof (basic_interfaces) / sizeof (struct usb_interface_description),
        0x01, 0x00, // bConfigurationValue, iConfiguration
        BMATTRIBUTE, BMAXPOWER,
};

struct usb_configuration_description basic_description[] = {
	{ iConfiguration: CONFIG_USBD_NETWORK_BASIC_DESC,
                configuration_descriptor: (struct usb_configuration_descriptor *)basic_configuration_descriptor,
                bNumInterfaces:sizeof (basic_interfaces) / sizeof (struct usb_interface_description),
                interface_list:basic_interfaces,},

};

/* BASIC Device Description
 */
static struct usb_device_descriptor basic_device_descriptor = {
	bLength: sizeof(struct usb_device_descriptor),
	bDescriptorType: USB_DT_DEVICE,
	bcdUSB: __constant_cpu_to_le16(USB_BCD_VERSION),
	bDeviceClass: COMMUNICATIONS_DEVICE_CLASS,
	bDeviceSubClass: 0x02,
	bDeviceProtocol: 0x00,
	bMaxPacketSize0: 0x00,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_BCDDEVICE),
};

static struct usb_endpoint_request basic_endpoint_requests[ENDPOINTS+1] = {
        { 1, 0, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, MAXFRAMESIZE + 48, MAXFRAMESIZE + 512, },
        { 1, 0, 0, USB_DIR_IN | USB_ENDPOINT_BULK, MAXFRAMESIZE + 48, MAXFRAMESIZE + 512, },
        { 1, 0, 0, USB_DIR_IN | USB_ENDPOINT_INTERRUPT | USB_ENDPOINT_OPT, 16, 64, },
        { 0, },
};

struct usb_device_description basic_device_description = {
        device_descriptor: &basic_device_descriptor,
	iManufacturer: CONFIG_USBD_NETWORK_MANUFACTURER,
	iProduct: CONFIG_USBD_NETWORK_PRODUCT_NAME,
#if !defined(CONFIG_USBD_NO_SERIAL_NUMBER) && defined(CONFIG_USBD_SERIAL_NUMBER_STR)
	iSerialNumber:CONFIG_USBD_SERIAL_NUMBER_STR,
#endif
        endpointsRequested: ENDPOINTS,
        requestedEndpoints: basic_endpoint_requests,
};


void basic_init (struct usb_function_instance *function)
{
        basic_data_alternate_descriptions[0].endpoints = Usb_network_private.have_interrupt ? 3 : 2;
}

struct usb_function_driver basic_function_driver = {
	name: "network-BASIC",
	fops: &network_fd_function_ops,
	device_description: &basic_device_description,
	bNumConfigurations: sizeof (basic_description) / sizeof (struct usb_configuration_description),
	configuration_description: basic_description,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_BCDDEVICE),
};
#endif				/* CONFIG_USBD_NETWORK_BASIC */

