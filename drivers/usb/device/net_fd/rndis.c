/*
 * linux/drivers/usbd/net_fd/rndis.c 
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
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

#ifdef MODULE
#include <linux/module.h>
#else
#error MODULE not defined
#endif
#include <linux/config.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <net/arp.h>
#include <linux/rtnetlink.h>
#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/atmdev.h>
#include <linux/pkt_sched.h>

#include "usbd.h"
#include "usbd-func.h"

//#include "rndis.h"
#include "netproto.h"


/* Module Parameters ************************************************************************* */

#define MAX_INTERFACES 2


struct usb_net_private {
	int interface;
	struct usb_device_instance *device;
	struct urb *urb;
	rwlock_t rwlock;
};



#define MAX_INTERFACES  2

static int rndis_created;

static struct usb_net_private *net_private_array[MAX_INTERFACES];

static rwlock_t netproto_rwlock = RW_LOCK_UNLOCKED;	// lock for netproto device array access



/* USB Configuration Description ************************************************************* */

#define VENDOR_SPECIFIC_CLASS           0xff
#define VENDOR_SPECIFIC_SUBCLASS        0xff
#define VENDOR_SPECIFIC_PROTOCOL        0xff

#define MTU                             1500+100


/* Configuration Set - endpoints and interface(s)*/

static struct usb_endpoint_description rndis_endpoints[] = {
      {bEndpointAddress: 1, bmAttributes: BULK, wMaxPacketSize: 64, bInterval: 1, direction:IN},
      {bEndpointAddress: 2, bmAttributes: BULK, wMaxPacketSize: 64, bInterval: 0, direction:OUT},
      {bEndpointAddress: 3, bmAttributes: INTERRUPT, wMaxPacketSize: 64, bInterval: 1, direction:IN}
};



static struct usb_class_description rndis_class_descriptions[0] = {
	{ CS_INTERFACE, USB_ST_CMF,  0, { call_management: { bmCapabilities: 0, bDataInterface: 1 }}},
	{ CS_INTERFACE, USB_ST_ACMF, 0, { abstract_control: { bmCapabilities: 0 }}},
	{ CS_INTERFACE, USB_ST_UF,  1, { union_function: { bMasterInterface: 0, bSlaveInterface: { 1 } }}}
};

static struct usb_interface_description rndis_interfaces[] = {
      {iInterface:"Default interface",
	      bInterfaceClass:VENDOR_SPECIFIC_CLASS,
	      bInterfaceSubClass:VENDOR_SPECIFIC_SUBCLASS,
	      bInterfaceProtocol:VENDOR_SPECIFIC_PROTOCOL,

	      classes:sizeof (rndis_class_descriptions) /
	 sizeof (struct usb_class_description),
	      class_list:rndis_class_descriptions,

	      endpoints:sizeof (rndis_endpoints) /
	 sizeof (struct usb_endpoint_description),
	      endpoint_list:rndis_endpoints,
	 }
};


/* Configuration description list */

struct usb_configuration_description rndis_description[] = {
      {iConfiguration: "RNDIS Network Function", bmAttributes: 0, bMaxPower:0,
	      interfaces:sizeof (rndis_interfaces) /
	 sizeof (struct usb_interface_description),
      interface_list:rndis_interfaces}
};

struct usb_device_description rndis_device_description = {
	bDeviceClass:VENDOR,
	bDeviceSubClass:0,
	bDeviceProtocol:0,
	idVendor:USB_VENDOR_COMPAQ,
	idProduct:COMPAQ_ITSY_ID,
	iManufacturer:"Lineo",
	iProduct:"RNDIS Network Driver",
	iSerialNumber:"0123456789"
};


module_init (function_modinit);
module_exit (function_modexit);

MODULE_AUTHOR ("sl@lineo.com");
MODULE_DESCRIPTION ("USB DEVICE Network Function Driver Prototype");
