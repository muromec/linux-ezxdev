/*
 * usbd/network_fd/basic22.c - Network Function Driver
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


#ifdef CONFIG_USBD_NETWORK_BASIC2
/* USB BASIC Configuration ******************************************************************** */

/*
 * This provides a slight amplification of the basic configuration, it moves the
 * interrupt endpoint (if available) to a separate interface, so that it is similiar
 * to the cdc configuration
 */

/* BASIC Communication Interface Class descriptors
 */
static __u8 basic2_data_1[] = { 0x07, USB_DT_ENDPOINT, OUT, BULK,     0, 0x00, 0x00,  };
static __u8 basic2_data_2[] = { 0x07, USB_DT_ENDPOINT, IN,  BULK,     0, 0x00, 0x00, };
static __u8 basic2_comm_1[] = { 0x07, USB_DT_ENDPOINT, IN,  INTERRUPT,0, 0x00, 0x0a, };

static __u8 *basic2_comm_endpoints[] = { 
        (struct usb_endpoint_descriptor *) &basic2_comm_1, };

static __u8 *basic2_data_endpoints[] = { 
        (struct usb_endpoint_descriptor *) &basic2_data_1, 
        (struct usb_endpoint_descriptor *) &basic2_data_2, };

u8 basic2_comm_indexes[] = { INT_IN, };
u8 basic2_data_indexes[] = { BULK_OUT, BULK_IN, };


/* BASIC2 Data Interface Alternate endpoints
 */

static __u8 basic2_comm_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09, USB_DT_INTERFACE, 0x00, 0x00, // bInterfaceNumber, bAlternateSetting
        sizeof (basic2_comm_endpoints) / sizeof(struct usb_endpoint_descriptor), // bNumEndpoints
        COMMUNICATIONS_INTERFACE_CLASS, COMMUNICATIONS_NETWORK_SUBCLASS, VENDOR, 0x00,
};
static __u8 basic2_data_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09, USB_DT_INTERFACE, 0x01, 0x00, // bInterfaceNumber, bAlternateSetting
        sizeof (basic2_data_endpoints) / sizeof(struct usb_endpoint_descriptor), // bNumEndpoints
        DATA_INTERFACE_CLASS, COMMUNICATIONS_NO_SUBCLASS, COMMUNICATIONS_NO_PROTOCOL, 0x00,
};

static struct usb_alternate_description basic2_comm_alternate_descriptions[] = {
      { iInterface: CONFIG_USBD_NETWORK_BASIC2_COMM_INTF,
                interface_descriptor: (struct usb_interface_descriptor *)&basic2_comm_alternate_descriptor,
                endpoints:sizeof (basic2_comm_endpoints) / sizeof(struct usb_endpoint_descriptor *),
                endpoint_list:basic2_comm_endpoints,
                endpoint_indexes:basic2_comm_indexes,
                },
};


static struct usb_alternate_description basic2_data_alternate_descriptions[] = {
      { iInterface: CONFIG_USBD_NETWORK_BASIC2_DATA_INTF,
                interface_descriptor: (struct usb_interface_descriptor *)&basic2_data_alternate_descriptor,
                endpoints:sizeof (basic2_data_endpoints) / sizeof(struct usb_endpoint_descriptor *),
                endpoint_list: basic2_data_endpoints,
                endpoint_indexes: basic2_data_indexes,
        },
};


/* BASIC Data Interface Alternate descriptions and descriptors
 */

/* BASIC Interface descriptions and descriptors
 */
static struct usb_interface_description basic2_interfaces[] = {
        { alternates:sizeof (basic2_comm_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:basic2_comm_alternate_descriptions,},

        { alternates:sizeof (basic2_data_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:basic2_data_alternate_descriptions,},
};


/* BASIC Configuration descriptions and descriptors
 */
__u8 basic2_configuration_descriptor[sizeof(struct usb_configuration_descriptor)] = {
        0x09, USB_DT_CONFIG, 0x00, 0x00, // wLength
        sizeof (basic2_interfaces) / sizeof (struct usb_interface_description),
        0x01, 0x00, // bConfigurationValue, iConfiguration
        BMATTRIBUTE, BMAXPOWER,
};

struct usb_configuration_description basic2_description[] = {
	{ iConfiguration: CONFIG_USBD_NETWORK_BASIC2_DESC,
                configuration_descriptor: (struct usb_configuration_descriptor *)basic2_configuration_descriptor,
                bNumInterfaces:sizeof (basic2_interfaces) / sizeof (struct usb_interface_description),
                interface_list:basic2_interfaces,},

};

/* BASIC Device Description
 */
static struct usb_device_descriptor basic2_device_descriptor = {
	bLength: sizeof(struct usb_device_descriptor),
	bDescriptorType: USB_DT_DEVICE,
	bcdUSB: __constant_cpu_to_le16(USB_BCD_VERSION),
	bDeviceClass: COMMUNICATIONS_DEVICE_CLASS,
	bDeviceSubClass: 0x02,
	bDeviceProtocol: 0x00,
	bMaxPacketSize0: 0x00,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_BASIC2_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_BASIC2_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_BASIC2_BCDDEVICE),
};

static struct usb_endpoint_request basic2_endpoint_requests[ENDPOINTS+1] = {
        { 1, 0, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, MAXFRAMESIZE + 48, MAXFRAMESIZE + 512,  },
        { 1, 0, 0, USB_DIR_IN | USB_ENDPOINT_BULK, MAXFRAMESIZE + 48, MAXFRAMESIZE + 512, },
        { 1, 0, 0, USB_DIR_IN | USB_ENDPOINT_INTERRUPT | USB_ENDPOINT_OPT, 16, 64, },
        { 0, },
};

struct usb_device_description basic2_device_description = {
        device_descriptor: &basic2_device_descriptor,
	iManufacturer: CONFIG_USBD_NETWORK_MANUFACTURER,
	iProduct: CONFIG_USBD_NETWORK_PRODUCT_NAME,
#if !defined(CONFIG_USBD_NO_SERIAL_NUMBER) && defined(CONFIG_USBD_SERIAL_NUMBER_STR)
	iSerialNumber:CONFIG_USBD_SERIAL_NUMBER_STR,
#endif
        endpointsRequested: ENDPOINTS,
        requestedEndpoints: basic2_endpoint_requests,
};


void basic2_init (struct usb_function_instance *function)
{
        printk(KERN_INFO"%s:\n", __FUNCTION__);

        basic2_comm_alternate_descriptions[0].endpoints = Usb_network_private.have_interrupt ? 1 : 0;

        printk(KERN_INFO"%s: alternate: %p endpoints: %d\n", __FUNCTION__,
                        basic2_data_alternate_descriptions,
                        basic2_data_alternate_descriptions->endpoints
        );

        printk(KERN_INFO"%s: finis\n", __FUNCTION__);
}


struct usb_function_driver basic2_function_driver = {
	name: "network-BASIC2",
	fops: &network_fd_function_ops,
	device_description: &basic2_device_description,
	bNumConfigurations: sizeof (basic2_description) / sizeof (struct usb_configuration_description),
	configuration_description: basic2_description,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_BASIC2_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_BASIC2_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_BASIC2_BCDDEVICE),
};
#endif				/* CONFIG_USBD_NETWORK_BASIC2 */

