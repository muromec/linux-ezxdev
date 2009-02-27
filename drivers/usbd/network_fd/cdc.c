/*
 * usbd/network_fd/cdc.c - Network Function Driver
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
 *
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



#ifdef CONFIG_USBD_NETWORK_CDC
/* USB CDC Configuration ********************************************************************* */

/* CDC Communication Interface Class descriptors
 */
static __u8 cdc_data_1[] = { 0x07, USB_DT_ENDPOINT, 0 | OUT, BULK,     0, 0x00, 0x00, };
static __u8 cdc_data_2[] = { 0x07, USB_DT_ENDPOINT, 0 | IN,  BULK,     0,  0x00, 0x00, };

static __u8 cdc_comm_1[] = { 0x07, USB_DT_ENDPOINT, 0 | IN,  INTERRUPT,0, 0x00, 0x0a, };

static __u8 cdc_class_1[] = { 0x05, CS_INTERFACE, USB_ST_HEADER, 0x10, 0x01, /* CLASS_BDC_VERSION, CLASS_BDC_VERSION */ };
static __u8 cdc_class_2[] = { 
        0x0d, CS_INTERFACE, USB_ST_ENF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xea, 0x05, /* 1514 maximum frame size */
        0x00, 0x00, 0x00 , };
static __u8 cdc_class_3[] = { 0x05, CS_INTERFACE, USB_ST_UF, 0x00, 0x01, /* bMasterInterface, bSlaveInterface */};


static __u8 *cdc_comm_class_descriptors[] = { 
        (struct usb_generic_class_descriptor *) &cdc_class_1, 
        (struct usb_generic_class_descriptor *) &cdc_class_2, 
        (struct usb_generic_class_descriptor *) &cdc_class_3, };

static __u8 *cdc_data_endpoints[] = { 
        (struct usb_endpoint_descriptor *) &cdc_data_1, 
        (struct usb_endpoint_descriptor *) &cdc_data_2, };
static __u8 *cdc_comm_endpoints[] = { 
        (struct usb_endpoint_descriptor *) &cdc_comm_1, };

u8 cdc_comm_indexes[] = { INT_IN, };
u8 cdc_data_indexes[] = { BULK_OUT, BULK_IN, };


/* Data Interface Alternate descriptions and descriptors
 */
static __u8 cdc_comm_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09, USB_DT_INTERFACE, 0x00, 0x00, // bInterfaceNumber, bAlternateSetting
        sizeof (cdc_comm_endpoints) / sizeof(struct usb_endpoint_descriptor), // bNumEndpoints
        COMMUNICATIONS_INTERFACE_CLASS, COMMUNICATIONS_ENCM_SUBCLASS, COMMUNICATIONS_NO_PROTOCOL, 0x00,
};
static __u8 cdc_nodata_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09, USB_DT_INTERFACE, 0x01, 0x00, // bInterfaceNumber, bAlternateSetting
        0x00, // bNumEndpoints
        DATA_INTERFACE_CLASS, COMMUNICATIONS_NO_SUBCLASS, COMMUNICATIONS_NO_PROTOCOL, 0x00,
};
static __u8 cdc_data_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09, USB_DT_INTERFACE, 0x01, 0x01, // bInterfaceNumber, bAlternateSetting
        sizeof (cdc_data_endpoints) / sizeof(struct usb_endpoint_descriptor), // bNumEndpoints
        DATA_INTERFACE_CLASS, COMMUNICATIONS_NO_SUBCLASS, COMMUNICATIONS_NO_PROTOCOL, 0x00,
};

static struct usb_alternate_description cdc_comm_alternate_descriptions[] = {
	{ iInterface: CONFIG_USBD_NETWORK_CDC_COMM_INTF,
                interface_descriptor: (struct usb_interface_descriptor *)&cdc_comm_alternate_descriptor,
                classes:sizeof (cdc_comm_class_descriptors) / sizeof (struct usb_generic_class_descriptor *),
                class_list: cdc_comm_class_descriptors,
                endpoints:sizeof (cdc_comm_endpoints) / sizeof(struct usb_endpoint_descriptor *),
                endpoint_list: cdc_comm_endpoints,
                endpoint_indexes: cdc_comm_indexes,
        },
};


static struct usb_alternate_description cdc_data_alternate_descriptions[] = {
	{ iInterface: CONFIG_USBD_NETWORK_CDC_NODATA_INTF,
                interface_descriptor: (struct usb_interface_descriptor *)&cdc_nodata_alternate_descriptor, },
	{ iInterface: CONFIG_USBD_NETWORK_CDC_DATA_INTF,
                interface_descriptor: (struct usb_interface_descriptor *)&cdc_data_alternate_descriptor,
                endpoints:sizeof (cdc_data_endpoints) / sizeof(struct usb_endpoint_descriptor *),
                endpoint_list: cdc_data_endpoints,
                endpoint_indexes: cdc_data_indexes,
                },
};

/* Interface descriptions and descriptors
 */
struct usb_interface_description cdc_interfaces[] = {
	{ alternates:sizeof (cdc_comm_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:cdc_comm_alternate_descriptions,},

	{ alternates:sizeof (cdc_data_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:cdc_data_alternate_descriptions,},
};

/* Configuration descriptions and descriptors
 */

__u8 cdc_configuration_descriptor[sizeof(struct usb_configuration_descriptor)] = {
        0x09, USB_DT_CONFIG, 0x00, 0x00, // wLength
        sizeof (cdc_interfaces) / sizeof (struct usb_interface_description),
        0x01, 0x00, // bConfigurationValue, iConfiguration
        BMATTRIBUTE, BMAXPOWER,
};

struct usb_configuration_description cdc_description[] = {
	{ iConfiguration: CONFIG_USBD_NETWORK_CDC_DESC,
                configuration_descriptor: (struct usb_configuration_descriptor *)cdc_configuration_descriptor,
                bNumInterfaces:sizeof (cdc_interfaces) / sizeof (struct usb_interface_description),
                interface_list:cdc_interfaces,},
};



/* Device Description
 */

static struct usb_device_descriptor cdc_device_descriptor = {
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

static struct usb_endpoint_request cdc_endpoint_requests[ENDPOINTS+1] = {
        { 1, 1, 1, USB_DIR_OUT | USB_ENDPOINT_BULK, MAXFRAMESIZE + 48, MAXFRAMESIZE + 512,  },
        { 1, 1, 1, USB_DIR_IN | USB_ENDPOINT_BULK, MAXFRAMESIZE + 48, MAXFRAMESIZE + 512,  },
        { 1, 0, 0, USB_DIR_IN | USB_ENDPOINT_INTERRUPT | USB_ENDPOINT_OPT, 16, 64, },
        { 0, },
};

struct usb_device_description cdc_device_description = {
        device_descriptor: &cdc_device_descriptor,
	iManufacturer: CONFIG_USBD_NETWORK_MANUFACTURER,
	iProduct: CONFIG_USBD_NETWORK_PRODUCT_NAME,
#if !defined(CONFIG_USBD_NO_SERIAL_NUMBER) && defined(CONFIG_USBD_SERIAL_NUMBER_STR)
	iSerialNumber:CONFIG_USBD_SERIAL_NUMBER_STR,
#endif
        endpointsRequested: ENDPOINTS,
        requestedEndpoints: cdc_endpoint_requests,
};



void cdc_init (struct usb_function_instance *function)
{
        struct usb_class_ethernet_networking_descriptor *ethernet;

        cdc_comm_alternate_descriptions[0].endpoints = Usb_network_private.have_interrupt ? 1 : 0;

        // Update the iMACAddress field in the ethernet descriptor
        {
                char address_str[14];
                snprintf(address_str, 13, "%02x%02x%02x%02x%02x%02x",
                                remote_dev_addr[0], remote_dev_addr[1], remote_dev_addr[2], 
                                remote_dev_addr[3], remote_dev_addr[4], remote_dev_addr[5]);

                if ((ethernet = (struct usb_class_ethernet_networking_descriptor *)cdc_class_2)) {
                        if (ethernet->iMACAddress) {
                                usbd_dealloc_string(ethernet->iMACAddress);
                        }
                        ethernet->iMACAddress = usbd_alloc_string(address_str);
                }
        }
}


struct usb_function_driver cdc_function_driver = {
	name: "network-CDC",
	fops: &network_fd_function_ops,
	device_description: &cdc_device_description,
	bNumConfigurations: sizeof (cdc_description) / sizeof (struct usb_configuration_description),
	configuration_description: cdc_description,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_BCDDEVICE),
};
#endif				/* CONFIG_USBD_NETWORK_CDC */

