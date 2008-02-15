/*
 * usbd/network_fd/safe.c - Network Function Driver
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


#ifdef CONFIG_USBD_NETWORK_SAFE
/* USB SAFE  Configuration ******************************************************************** */

/*
 * SAFE Ethernet Configuration
 */

/* Communication Interface Class descriptors
 */

static __u8 safe_data_1[] = { 0x07, USB_DT_ENDPOINT, OUT,  BULK,     0,  0x00, 0x00, };
static __u8 safe_data_2[] = { 0x07, USB_DT_ENDPOINT, IN,   BULK,     0,  0x00, 0x00, };
static __u8 safe_comm_1[] = { 0x07, USB_DT_ENDPOINT, IN,   INTERRUPT,0,  0x00, 0x0a, };

static __u8 safe_class_1[] = { 0x05, CS_INTERFACE, USB_ST_HEADER, 0x10, 0x01, /* CLASS_BDC_VERSION, CLASS_BDC_VERSION */ };
static __u8 safe_class_2[] = { 0x15, CS_INTERFACE, USB_ST_MDLM, 0x00, 0x01, /* bcdVersion, bcdVersion */
                0x5d, 0x34, 0xcf, 0x66, 0x11, 0x18, 0x11, 0xd6,  /* bGUID */
                0xa2, 0x1a, 0x00, 0x01, 0x02, 0xca, 0x9a, 0x7f,  /* bGUID */ };

static __u8 safe_class_3[] = { 0x06, CS_INTERFACE, USB_ST_MDLMD, 0x00, 0x00, 0x00,    /* bDetailData */ };

static __u8 safe_class_4[] = { 0x0d, CS_INTERFACE, USB_ST_ENF, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0xea, 0x05, /* 1514 maximum frame size */
        0x00, 0x00, 0x00 , };


static __u8 *safe_alt_endpoints[] = { 
        (struct usb_endpoint_descriptor *) &safe_data_1, 
        (struct usb_endpoint_descriptor *) &safe_data_2, 
        (struct usb_endpoint_descriptor *) &safe_comm_1, };
static __u8 *safe_comm_class_descriptors[] = { 
        (struct usb_generic_class_descriptor *) &safe_class_1, 
        (struct usb_generic_class_descriptor *) &safe_class_2, 
        (struct usb_generic_class_descriptor *) &safe_class_3, 
        (struct usb_generic_class_descriptor *) &safe_class_4, };

u8 safe_alt_indexes[] = { BULK_OUT, BULK_IN, INT_IN, };

/* Data Interface Alternate descriptions and descriptors
 */
static __u8 safe_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09, USB_DT_INTERFACE, 0x00, 0x00, // bInterfaceNumber, bAlternateSetting
        sizeof (safe_alt_endpoints) / sizeof(struct usb_endpoint_descriptor), // bNumEndpoints
        COMMUNICATIONS_INTERFACE_CLASS, COMMUNICATIONS_MDLM_SUBCLASS, COMMUNICATIONS_NO_PROTOCOL, 0x00,
};

static struct usb_alternate_description safe_alternate_descriptions[] = {
	{ iInterface: CONFIG_USBD_NETWORK_SAFE_INTF,
                interface_descriptor: (struct usb_interface_descriptor *)&safe_alternate_descriptor,
                classes:sizeof (safe_comm_class_descriptors) / sizeof (struct usb_generic_class_descriptor *),
                class_list: safe_comm_class_descriptors,
                endpoints:sizeof (safe_alt_endpoints) / sizeof(struct usb_endpoint_descriptor *),
                endpoint_list: safe_alt_endpoints,
                endpoint_indexes: safe_alt_indexes,
                },
};

/* Interface descriptions and descriptors
 */
static struct usb_interface_description safe_interfaces[] = {
	{ alternates:sizeof (safe_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:safe_alternate_descriptions, },
};


/* Configuration descriptions and descriptors
 */

static __u8 safe_configuration_descriptor[sizeof(struct usb_configuration_descriptor)] = {
        0x09, USB_DT_CONFIG, 0x00, 0x00, // wLength
        sizeof (safe_interfaces) / sizeof (struct usb_interface_description),
        0x01, 0x00, // bConfigurationValue, iConfiguration
        BMATTRIBUTE, BMAXPOWER,
};

struct usb_configuration_description safe_description[] = {
	{ iConfiguration: CONFIG_USBD_NETWORK_SAFE_DESC,
                configuration_descriptor: (struct usb_configuration_descriptor *)safe_configuration_descriptor,
	        bNumInterfaces:sizeof (safe_interfaces) / sizeof (struct usb_interface_description),
                interface_list:safe_interfaces, },
};

/* Device Description
 */

//static __u8 safe_device_descriptor[sizeof(struct usb_device_descriptor)] = {
//        0x12, USB_DT_DEVICE, 
//        0x00, 0x02, // bcdUSB
//        COMMUNICATIONS_DEVICE_CLASS, 
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//};
static struct usb_device_descriptor safe_device_descriptor = {
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

static struct usb_endpoint_request safe_endpoint_requests[ENDPOINTS+1] = {
        { 1, 0, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, MAXFRAMESIZE + 48, MAXFRAMESIZE + 512,  },
        { 1, 0, 0, USB_DIR_IN | USB_ENDPOINT_BULK, MAXFRAMESIZE + 48, MAXFRAMESIZE + 512,  },
        { 1, 0, 0, USB_DIR_IN | USB_ENDPOINT_INTERRUPT | USB_ENDPOINT_OPT, 16, 64, },
        { 0, },
};

struct usb_device_description safe_device_description = {
        device_descriptor: &safe_device_descriptor,
	iManufacturer: CONFIG_USBD_NETWORK_MANUFACTURER,
        iProduct: CONFIG_USBD_NETWORK_PRODUCT_NAME,
#if !defined(CONFIG_USBD_NO_SERIAL_NUMBER) && defined(CONFIG_USBD_SERIAL_NUMBER_STR)
	iSerialNumber:CONFIG_USBD_SERIAL_NUMBER_STR,
#endif
        endpointsRequested: ENDPOINTS,
        requestedEndpoints: safe_endpoint_requests,
};

void safe_init (struct usb_function_instance *function)
{
        struct usb_class_ethernet_networking_descriptor *ethernet;
        int len = 0;
        char buf[255];

        buf[0] = 0;

        safe_alternate_descriptions[0].endpoints = Usb_network_private.have_interrupt ? 3 : 2;

        // Update the iMACAddress field in the ethernet descriptor
        {
                char address_str[14];
                snprintf(address_str, 13, "%02x%02x%02x%02x%02x%02x",
                                remote_dev_addr[0], remote_dev_addr[1], remote_dev_addr[2], 
                                remote_dev_addr[3], remote_dev_addr[4], remote_dev_addr[5]);
                if ((ethernet = (struct usb_class_ethernet_networking_descriptor *)safe_class_4)) {
                        if (ethernet->iMACAddress) {
                                usbd_dealloc_string(ethernet->iMACAddress);
                        }
                        ethernet->iMACAddress = usbd_alloc_string(address_str);
                }
        }


#ifdef CONFIG_USBD_NETWORK_SAFE_PADBEFORE
        blan_class_3[5] |= BMDATA_PADBEFORE;
        len += sprintf(buf + len, "PADBEFORE: %02x ", blan_class_3[5]);
#endif
#ifdef CONFIG_USBD_NETWORK_SAFE_CRC
        safe_class_3[5] |= BMDATA_CRC;
        len += sprintf(buf + len, "CRC: %02x ", blan_class_3[5]);
#endif
#ifdef CONFIG_USBD_NETWORK_SAFE_NONBRIDGE
        blan_class_3[4] |= BMNETWORK_NONBRIDGE;
        len += sprintf(buf + len, "NONBRIDGE: %02x ",blan_class_3[4]);
#endif
        if (strlen(buf))
                printk(KERN_INFO"%s: %s\n", __FUNCTION__, buf);

}

struct usb_function_driver safe_function_driver = {
	name: "network-SAFE",
	fops: &network_fd_function_ops,
	device_description: &safe_device_description,
	bNumConfigurations: sizeof (safe_description) / sizeof (struct usb_configuration_description),
	configuration_description: safe_description,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_NETWORK_BCDDEVICE),
};
#endif				/* CONFIG_USBD_NETWORK_SAFE */

