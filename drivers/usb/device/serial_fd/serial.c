/*
 * serial_fd/serial.c
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

/* 
 * The encapsultaion is designed to overcome difficulties with some USB hardware.
 *
 * While the USB protocol has a CRC over the data while in transit, i.e. while
 * being carried over the bus, there is no end to end protection. If the hardware
 * has any problems getting the data into or out of the USB transmit and receive
 * FIFO's then data can be lost. 
 *
 * This protocol adds a two byte trailer to each USB packet to specify the number
 * of bytes of valid data and a 10 bit CRC that will allow the receiver to verify
 * that the entire USB packet was received without error.
 *
 * This means we now have end to end protection from the class driver to the function
 * driver and back.
 *
 * There is an additional option that can be used to force all transmitted packets
 * to be padded to the maximum packet size. This provides a work around for some
 * devices which have problems with small USB packets.
 *
 * Assuming a packetsize of N:
 *
 *      0..N-2  data and optional padding
 *
 *      N-2     bits 7-2 - number of bytes of valid data
 *              bits 1-0 top two bits of 10 bit CRC
 *      N-1     bottom 8 bits of 10 bit CRC
 *
 *
 *      | Data Length       | 10 bit CRC                                |
 *      + 7 . 6 . 5 . 4 . 3 . 2 . 1 . 0 | 7 . 6 . 5 . 4 . 3 . 2 . 1 . 0 +
 *      
 * The 10 bit CRC is computed across the sent data, followed by the trailer with
 * the length set and the CRC set to zero. The CRC is then OR'd into the trailer.
 *
 * When received a 10 bit CRC is computed over the entire frame including the trailer
 * and should be equal to zero.
 *
 * Two module parameters are used to control the encapsulation, if both are
 * turned of the module works as a simple serial device with NO
 * encapsulation.
 *
 * See linux/drivers/usb/serial/safe_serial.c for a host class driver
 * implementation of this.
 *
 */


#include <linux/config.h>
#include <linux/module.h>

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-module.h"


MODULE_AUTHOR ("sl@lineo.com, tbr@lineo.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION ("USB Device Serial Function");

USBD_MODULE_INFO ("serial_fd 0.1-beta");

#ifndef MODULE
#undef GET_USE_COUNT
#define GET_USE_COUNT(foo) 1
#endif

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

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-debug.h"
#include "../usbd-inline.h"
#include "../usbd-arch.h"

#include "crc10.h"

#include "serproto.h"

#if 0
#define MIN(a, b) ({            \
        typeof(a) _a = (a);     \
        typeof(b) _b = (b);     \
        _a < _b ? _a : _b;      \
})
#endif


#define MAX_INTERFACES 1

#define MTU                             1500+100


#if !defined (CONFIG_USBD_VENDORID) && !defined(CONFIG_USBD_SERIAL_VENDORID)
#error No Vendor ID
#endif
#if !defined (CONFIG_USBD_PRODUCTID) && !defined(CONFIG_USBD_SERIAL_PRODUCTID)
#error No Product ID
#endif


#if CONFIG_USBD_SERIAL_VENDORID
#undef CONFIG_USBD_VENDORID
#define CONFIG_USBD_VENDORID CONFIG_USBD_SERIAL_VENDORID
#endif

#if CONFIG_USBD_SERIAL_PRODUCTID
#undef CONFIG_USBD_PRODUCTID
#define CONFIG_USBD_PRODUCTID CONFIG_USBD_SERIAL_PRODUCTID
#endif

#ifndef CONFIG_USBD_SERIAL_NUMBER_STR
#define CONFIG_USBD_SERIAL_NUMBER_STR           ""
#endif


#ifdef CONFIG_USBD_SELFPOWERED
#define BMATTRIBUTE BMATTRIBUTE_RESERVED | BMATTRIBUTE_SELF_POWERED
#define BMAXPOWER 0
#else
#define BMATTRIBUTE BMATTRIBUTE_RESERVED
#define BMAXPOWER CONFIG_USBD_MAXPOWER
#endif


/*
 * setup some default values for pktsizes and endpoint addresses.
 */

#ifndef CONFIG_USBD_SERIAL_OUT_PKTSIZE
#define CONFIG_USBD_SERIAL_OUT_PKTSIZE             64
#endif

#ifndef CONFIG_USBD_SERIAL_IN_PKTSIZE
#define CONFIG_USBD_SERIAL_IN_PKTSIZE              64
#endif

#ifndef CONFIG_USBD_SERIAL_INT_PKTSIZE
#define CONFIG_USBD_SERIAL_INT_PKTSIZE             16
#endif

#ifndef CONFIG_USBD_SERIAL_OUT_ENDPOINT
#define CONFIG_USBD_SERIAL_OUT_ENDPOINT            1
#endif

#ifndef CONFIG_USBD_SERIAL_IN_ENDPOINT
#define CONFIG_USBD_SERIAL_IN_ENDPOINT             2
#endif

#ifndef CONFIG_USBD_SERIAL_INT_ENDPOINT
#define CONFIG_USBD_SERIAL_INT_ENDPOINT                3
#endif


/*
 * check for architecture specific endpoint configurations
 */

#if     defined(ABS_OUT_ADDR)
#warning
#warning USING ABS ENDPOINT OUT ADDRESS
#undef CONFIG_USBD_SERIAL_OUT_ENDPOINT

#if     ABS_OUT_ADDR > 0
#define CONFIG_USBD_SERIAL_OUT_ENDPOINT            ABS_OUT_ADDR
#endif

#elif   defined(MAX_OUT_ADDR) && defined(CONFIG_USBD_SERIAL_OUT_ENDPOINT) && (CONFIG_USBD_SERIAL_OUT_ENDPOINT > MAX_OUT_ADDR)
#warning
#warning USING DEFAULT ENDPOINT OUT ADDRESS
#undef CONFIG_USBD_SERIAL_OUT_ENDPOINT
#define CONFIG_USBD_SERIAL_OUT_ENDPOINT            DFL_OUT_ADDR
#endif


#if     defined(ABS_IN_ADDR)
#warning
#warning USING ABS ENDPOINT IN ADDRESS
#undef CONFIG_USBD_SERIAL_IN_ENDPOINT

#if     ABS_IN_ADDR
#define CONFIG_USBD_SERIAL_IN_ENDPOINT             ABS_IN_ADDR
#endif

#elif   defined(MAX_IN_ADDR) && defined(CONFIG_USBD_SERIAL_IN_ENDPOINT) && (CONFIG_USBD_SERIAL_IN_ENDPOINT > MAX_IN_ADDR)
#warning
#warning USING DEFAULT ENDPOINT IN ADDRESS
#undef CONFIG_USBD_SERIAL_IN_ENDPOINT
#define CONFIG_USBD_SERIAL_IN_ENDPOINT             DFL_IN_ADDR
#endif


#if     defined(ABS_INT_ADDR)
#warning
#warning USING ABS ENDPOINT INT ADDRESS
#undef CONFIG_USBD_SERIAL_INT_ENDPOINT

#if     ABS_INT_ADDR
#define CONFIG_USBD_SERIAL_INT_ENDPOINT            ABS_INT_ADDR
#endif

#elif   defined(MAX_INT_ADDR) && defined(CONFIG_USBD_SERIAL_INT_ENDPOINT) && (CONFIG_USBD_SERIAL_INT_ENDPOINT > MAX_INT_ADDR)
#warning
#warning USING DEFAULT ENDPOINT INT ADDRESS
#undef CONFIG_USBD_SERIAL_INT_ENDPOINT
#define CONFIG_USBD_SERIAL_INT_ENDPOINT            DFL_INT_ADDR
#endif



#if     defined(MAX_OUT_PKTSIZE) && defined(CONFIG_USBD_SERIAL_OUT_PKTSIZE) && CONFIG_USBD_SERIAL_OUT_PKTSIZE > MAX_OUT_PKTSIZE
#warning
#warning OVERIDING ENDPOINT OUT PKTSIZE
#undef CONFIG_USBD_SERIAL_OUT_PKTSIZE
#define CONFIG_USBD_SERIAL_OUT_PKTSIZE             MAX_OUT_PKTSIZE
#endif

#if     defined(MAX_IN_PKTSIZE) && defined(CONFIG_USBD_SERIAL_IN_PKTSIZE) && CONFIG_USBD_SERIAL_IN_PKTSIZE > MAX_IN_PKTSIZE
#warning
#warning OVERIDING ENDPOINT IN PKTSIZE
#undef CONFIG_USBD_SERIAL_IN_PKTSIZE
#define CONFIG_USBD_SERIAL_IN_PKTSIZE              MAX_IN_PKTSIZE
#endif

#if     defined(MAX_INT_PKTSIZE) && defined(CONFIG_USBD_SERIAL_INT_PKTSIZE) && CONFIG_USBD_SERIAL_INT_PKTSIZE > MAX_INT_PKTSIZE
#warning
#warning OVERIDING ENDPOINT INT PKTSIZE
#undef CONFIG_USBD_SERIAL_INT_PKTSIZE
#define CONFIG_USBD_SERIAL_INT_PKTSIZE             MAX_INT_PKTSIZE
#endif




struct usb_serial_private {
	int interface;
	struct usb_device_instance *device;
	rwlock_t rwlock;
};

/* Module Parameters ************************************************************************* */

static char *dbg = NULL;
static u32 vendor_id;
static u32 product_id;
static u32 txqueue_urbs;
static u32 txqueue_bytes = 3032;

#ifndef CONFIG_USBD_SERIAL_SAFE_DEFAULT
#define CONFIG_USBD_SERIAL_SAFE_DEFAULT 0
#endif
#ifndef CONFIG_USBD_SERIAL_SAFE_PADDED
#define CONFIG_USBD_SERIAL_SAFE_PADDED 0
#endif

static int safe = CONFIG_USBD_SERIAL_SAFE_DEFAULT;
static int padded = CONFIG_USBD_SERIAL_SAFE_PADDED;

MODULE_PARM (dbg, "s");
MODULE_PARM (vendor_id, "i");
MODULE_PARM (product_id, "i");
MODULE_PARM (txqueue_urbs, "i");
MODULE_PARM (txqueue_bytes, "i");
MODULE_PARM (safe, "i");
MODULE_PARM (padded, "i");

MODULE_PARM_DESC (dbg, "USB Device Debug options");
MODULE_PARM_DESC (vendor_id, "USB Device Vendor ID");
MODULE_PARM_DESC (product_id, "USB Device Product ID");
MODULE_PARM_DESC (txqueue_urbs, "Maximum TX Queue Urbs");
MODULE_PARM_DESC (txqueue_bytes, "Maximum TX Queue Bytes");
MODULE_PARM_DESC (safe, "Safe Encapsulation");
MODULE_PARM_DESC (padded, "Safe Encapsulation Padding");



/* Debug switches (module parameter "dbg=...") *********************************************** */

extern int dbgflg_usbdfd_init;
int dbgflg_usbdfd_usbe;
int dbgflg_usbdfd_tx;
int dbgflg_usbdfd_rx;
int dbgflg_usbdfd_loopback;

static debug_option dbg_table[] = {
	{&dbgflg_usbdfd_init, NULL, "init", "initialization and termination"},
	{&dbgflg_usbdfd_usbe, NULL, "usbe", "USB events"},
	{&dbgflg_usbdfd_rx, NULL, "rx", "receive (from host)"},
	{&dbgflg_usbdfd_tx, NULL, "tx", "transmit (to host)"},
	{&dbgflg_usbdfd_loopback, NULL, "loop", "loopback mode if non-zero"},
	{NULL, NULL, "ser", "serial device (tty) handling"},
	{NULL, NULL, NULL, NULL}
};

#define dbg_init(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_init,lvl,fmt,##args)
#define dbg_usbe(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_usbe,lvl,fmt,##args)
#define dbg_rx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_rx,lvl,fmt,##args)
#define dbg_tx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_tx,lvl,fmt,##args)
#define dbg_loop(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_loopback,lvl,fmt,##args)

/* ******************************************************************************************* */

static int serial_created;

static struct usb_serial_private *serial_private_array[MAX_INTERFACES];

static rwlock_t serial_rwlock = RW_LOCK_UNLOCKED;	// lock for serproto device array access

static __inline__ struct usb_serial_private *get_serial_private (int interface)
{
	if (interface < 0 || interface >= MAX_INTERFACES) {
		return NULL;
	}
	return serial_private_array[interface];
}


/* usb-func.c ******************************************************************************** */


/* Communications Interface Class descriptions 
 */

static struct usb_endpoint_description serial_default[] = {
      {bEndpointAddress:CONFIG_USBD_SERIAL_OUT_ENDPOINT,
	      bmAttributes:BULK,
	      wMaxPacketSize:CONFIG_USBD_SERIAL_OUT_PKTSIZE,
	      bInterval:0,
	      direction:OUT,
      transferSize:CONFIG_USBD_SERIAL_OUT_PKTSIZE,},

      {bEndpointAddress:CONFIG_USBD_SERIAL_IN_ENDPOINT,
	      bmAttributes:BULK,
	      wMaxPacketSize:CONFIG_USBD_SERIAL_IN_PKTSIZE,
	      bInterval:0,
	      direction:IN,
      transferSize:CONFIG_USBD_SERIAL_IN_PKTSIZE,},

#if defined(CONFIG_USBD_SERIAL_INT_ENDPOINT) && (CONFIG_USBD_SERIAL_INT_ENDPOINT > 0)
      {bEndpointAddress:CONFIG_USBD_SERIAL_INT_ENDPOINT,
	      bmAttributes:INTERRUPT,
	      wMaxPacketSize:CONFIG_USBD_SERIAL_INT_PKTSIZE,
	      bInterval:0,
	      direction:IN,
      transferSize:CONFIG_USBD_SERIAL_INT_PKTSIZE,},
#endif

};


/* Data Interface Alternate description(s)
 */
static __devinitdata struct usb_alternate_description serial_data_alternate_descriptions[] = {
      {iInterface:"Simple Serial Data Interface - Bulk mode",
	      bAlternateSetting:0,
	      endpoints:sizeof (serial_default) /
	 sizeof (struct usb_endpoint_description),
      endpoint_list:serial_default,},
};

/* Interface description(s)
 */
static __devinitdata struct usb_interface_description serial_interfaces[] = {
      {iInterface:"Simple Serial Data Interface",
	      bInterfaceClass:LINEO_CLASS,
	      bInterfaceSubClass:LINEO_SUBCLASS_SAFESERIAL,
	      bInterfaceProtocol:LINEO_SAFESERIAL_CRC,
	      alternates:sizeof (serial_data_alternate_descriptions) /
	 sizeof (struct usb_alternate_description),
      alternate_list:serial_data_alternate_descriptions,},
};


#ifdef CONFIG_USBD_SERIAL_CDC
/*
 * CDC ACM Configuration
 */


/* Communication Interface Class descriptions
 */
static struct usb_class_description cdc_comm_class_descriptions[] = {
	{ CS_INTERFACE, USB_ST_HEADER,  0, { header: { bcdCDC: CLASS_BCD_VERSION, } }},
	{ CS_INTERFACE, USB_ST_UF,      1, { union_function: { bMasterInterface: 0, bSlaveInterface: { 1 }, }}},
	{ CS_INTERFACE, USB_ST_CMF,     0, { call_management: { bmCapabilities: 0, bDataInterface: 1, }}},
	{ CS_INTERFACE, USB_ST_ACMF,    0, { abstract_control: { bmCapabilities: 0, }}},
};


/* Data Interface Alternate 1 endpoints
 */
static __devinitdata struct usb_endpoint_description serial_alt_1_endpoints[] = {
      {bEndpointAddress:CONFIG_USBD_SERIAL_OUT_ENDPOINT,
	      bmAttributes:BULK,
	      wMaxPacketSize:CONFIG_USBD_SERIAL_OUT_PKTSIZE,
	      bInterval:0,
	      direction:OUT,
      transferSize:CONFIG_USBD_SERIAL_OUT_PKTSIZE,},

      {bEndpointAddress:CONFIG_USBD_SERIAL_IN_ENDPOINT,
	      bmAttributes:BULK,
	      wMaxPacketSize:CONFIG_USBD_SERIAL_IN_PKTSIZE,
	      bInterval:0,
	      direction:IN,
      transferSize:CONFIG_USBD_SERIAL_OUT_PKTSIZE,},

#if defined(CONFIG_USBD_SERIAL_INT_ENDPOINT) && (CONFIG_USBD_SERIAL_INT_ENDPOINT > 0)
      {bEndpointAddress:CONFIG_USBD_SERIAL_INT_ENDPOINT,
	      bmAttributes:INTERRUPT,
	      wMaxPacketSize:CONFIG_USBD_SERIAL_INT_PKTSIZE,
	      bInterval:0,
	      direction:IN,
      transferSize:CONFIG_USBD_SERIAL_INT_PKTSIZE,},
#endif
};


/* Data Interface Alternate description(s)
 */
static __devinitdata struct usb_alternate_description cdc_comm_alternate_descriptions[] = {
      {iInterface:"CDC ACM Comm Interface",
	      bAlternateSetting:0,
	      classes:sizeof (cdc_comm_class_descriptions) /
	 sizeof (struct usb_class_description),
      class_list:cdc_comm_class_descriptions,},
};

static __devinitdata struct usb_alternate_description cdc_data_alternate_descriptions[] = {
      {iInterface:"CDC ACM Data Interface - Disabled mode",
#ifdef CONFIG_ARCH_LUBBOCK
      bAlternateSetting:1,},
#else
      bAlternateSetting:0,},
#endif
      {iInterface:"CDC ACM Data Interface - Bulk mode",

#ifdef CONFIG_ARCH_LUBBOCK
	      bAlternateSetting:0,
#else
	      bAlternateSetting:1,
#endif
	      endpoints:sizeof (serial_alt_1_endpoints) /
	 sizeof (struct usb_endpoint_description),
      endpoint_list:serial_alt_1_endpoints,},
};


/* Interface description(s)
 */
static __devinitdata struct usb_interface_description cdc_interfaces[] = {
      {iInterface:"CDC ACM Communication Interface",
	      bInterfaceClass:COMMUNICATIONS_INTERFACE_CLASS,
	      bInterfaceSubClass:COMMUNICATIONS_ACM_SUBCLASS,
	      bInterfaceProtocol:COMMUNICATIONS_NO_PROTOCOL,
	      alternates:sizeof (cdc_comm_alternate_descriptions) /
	 sizeof (struct usb_alternate_description),
      alternate_list:cdc_comm_alternate_descriptions,},

      {iInterface:"CDC ACM Data Interface",
	      bInterfaceClass:DATA_INTERFACE_CLASS,
	      bInterfaceSubClass:COMMUNICATIONS_NO_SUBCLASS,
	      bInterfaceProtocol:COMMUNICATIONS_NO_PROTOCOL,
	      alternates:sizeof (cdc_data_alternate_descriptions) /
	 sizeof (struct usb_alternate_description),
      alternate_list:cdc_data_alternate_descriptions,},
};

#endif				/* CONFIG_USBD_SERIAL_CDC */

/* Configuration description(s)
 */
struct __devinitdata usb_configuration_description serial_description[] = {
#ifdef CONFIG_USBD_SERIAL_CDC
      {iConfiguration:"CDC 1.1 ACM Configuration",
	      bmAttributes:BMATTRIBUTE,
	      bMaxPower:BMAXPOWER,
	      interfaces:sizeof (cdc_interfaces) /
	 sizeof (struct usb_interface_description),
      interface_list:cdc_interfaces,},
#endif

      {iConfiguration:"USB Simple Serial Configuration",
	      bmAttributes:BMATTRIBUTE,
	      bMaxPower:BMAXPOWER,
	      interfaces:sizeof (serial_interfaces) /
	 sizeof (struct usb_interface_description),
      interface_list:serial_interfaces,},
};

/* Device Description
 */
struct __devinitdata usb_device_description serial_device_description = {
	bDeviceClass:COMMUNICATIONS_DEVICE_CLASS,
	bDeviceSubClass:0,	// XXX
	bDeviceProtocol:0,	// XXX
	idVendor:CONFIG_USBD_VENDORID,
	idProduct:CONFIG_USBD_PRODUCTID,
	iManufacturer:CONFIG_USBD_MANUFACTURER,
	iProduct:CONFIG_USBD_PRODUCT_NAME,
	iSerialNumber:CONFIG_USBD_SERIAL_NUMBER_STR,
};



/* Transmit Function - called by serproto ****************************************************** */

static int serial_xmit_data (int interface, unsigned char *data, int data_length)
{
	int port = 0;		// XXX compound device
	struct usb_serial_private *serial_private;
	struct usb_device_instance *device;
	struct urb *urb;
	int packet_length = data_length;

	dbg_tx (2, "data: %p data_length: %d", data, data_length);

	if ((serial_private = get_serial_private (interface)) == NULL) {
		dbg_tx (0, "cannot recover serial private");
		return -EINVAL;
	}

	if ((device = serial_private->device) == NULL) {
		dbg_tx (0, "cannot recover serial private device");
		return -EINVAL;
	}
	// XXX Check if we are busy

	if ((urb = usbd_alloc_urb (serial_private->device,
				   (serial_private->device->function_instance_array + port),
				   CONFIG_USBD_SERIAL_IN_ENDPOINT | IN, 0)) == NULL) {
		dbg_tx (0, "failed to alloc urb");
		return -EINVAL;
	}
#ifdef CONFIG_USBD_SERIAL_SAFE
	if (safe) {
		__u16 fcs;

		dbg_tx (1, "safe mode: padded: %d data_length: %d packet_length: %d", padded,
			data_length, packet_length);

		// extend length to pad if required
		if (padded) {
			//packet_length = MAX(packet_length, CONFIG_USBD_SERIAL_IN_PKTSIZE - 2);
			//packet_length = MIN(MAX(packet_length, packet_length), CONFIG_USBD_SERIAL_IN_PKTSIZE - 2);

			packet_length = CONFIG_USBD_SERIAL_IN_PKTSIZE - 2;
			//packet_length = 2;
			//packet_length++;
		}
		// set length and a null byte 
		data[packet_length] = data_length << 2;
		data[packet_length + 1] = 0;

		// compute CRC across data, padding, data_length and null byte
		fcs = fcs_compute10 (data, packet_length + 2, CRC10_INITFCS);

		// OR CRC into last two bytes
		data[packet_length] |= fcs >> 8;
		data[packet_length + 1] = fcs & 0xff;
		packet_length += 2;
		dbg_tx (1, "safe mode: data_length: %d packet_length: %d", data_length,
			packet_length);
	}
#endif

	urb->buffer = data;
	urb->actual_length = packet_length;


	//dbgPRINTmem(dbgflg_usbdfd_tx,3,urb->buffer,urb->actual_length);
	dbgPRINTmem (dbgflg_usbdfd_tx, 3, urb->buffer, CONFIG_USBD_SERIAL_IN_PKTSIZE + 4);

	// push it down into the usb-device layer
	return usbd_send_urb (urb);
}

/* serial_urb_sent - called to indicate URB transmit finished
 * @urb: pointer to struct urb
 * @rc: result
 */
int serial_urb_sent (struct urb *urb, int rc)
{
	int port = 0;		// XXX compound device
	struct usb_device_instance *device = urb->device;
	struct usb_serial_private *serial_private =
	    (device->function_instance_array + port)->privdata;

	dbg_tx (2, "%s safe: %d length: %d", urb->device->name, safe, urb->actual_length);
	serproto_done (serial_private->interface,
		       urb->buffer,
		       safe ? (urb->buffer[urb->actual_length - 2] >> 2) : urb->actual_length, 0);
	usbd_dealloc_urb (urb);
	return 0;
}


/* USB Device Functions ************************************************************************ */

/* serial_event - process a device event
 *
 */
void serial_event (struct usb_device_instance *device, usb_device_event_t event, int data)
{
	int port = 0;		// XXX compound device
	struct usb_function_instance *function;
	unsigned int flags;

	if ((function = device->function_instance_array + port) == NULL) {
		dbg_usbe (1, "no function");
		return;
	}

	dbg_usbe (3, "");
        switch (event) {
        case DEVICE_UNKNOWN:
        case DEVICE_INIT: 
        case DEVICE_CREATE:     
        case DEVICE_HUB_CONFIGURED:
        case DEVICE_RESET:
        case DEVICE_ADDRESS_ASSIGNED:
        case DEVICE_CONFIGURED:
        case DEVICE_DE_CONFIGURED:
        case DEVICE_SET_INTERFACE:
        case DEVICE_SET_FEATURE:
        case DEVICE_CLEAR_FEATURE:
        case DEVICE_BUS_INACTIVE:
        case DEVICE_BUS_ACTIVITY:
        case DEVICE_POWER_INTERRUPTION:
        case DEVICE_HUB_RESET:
        case DEVICE_DESTROY:
        case DEVICE_FUNCTION_PRIVATE:
                dbg_usbe (1,"%s data: %d", usbd_device_events[event], data);
                break;
        default:
                dbg_usbe (1,"%s", usbd_device_events[DEVICE_UNKNOWN]);
                break;
        }
	switch (event) {

	case DEVICE_UNKNOWN:
	case DEVICE_INIT:
		break;

	case DEVICE_CREATE:
		{
			int i;
			int interface;
			struct usb_serial_private *serial_private;

			// There is no way to indicate error, so make this unconditional
			// and undo it in the DESTROY event unconditionally as well.
			// It the responsibility of the USBD core and the bus interface
			// to see that there is a matching DESTROY for every CREATE.
			// XXX XXX MOD_INC_USE_COUNT;  // Before any sleepable fns such as kmalloc
			// XXX XXX dbg_init(0,"CREATE sc=%d uc=%d",serial_created,GET_USE_COUNT(THIS_MODULE));

			// sanity checks
			if (serial_created >= MAX_INTERFACES) {
				dbg_usbe (1,
					  "---> CREATE %s serial_created >= MAX_INTERFACES %d %d",
					  device->name, serial_created, MAX_INTERFACES);
				dbg_init (0, "CREATE Z1 sc=%d uc=%d", serial_created,
					  GET_USE_COUNT (THIS_MODULE));
				return;
			}

			write_lock_irqsave (&serial_rwlock, flags);
			for (i = 0; i < MAX_INTERFACES; i++) {
				if (serial_private_array[i] == NULL) {
					break;
				}
			}
			if (i >= MAX_INTERFACES) {
				write_unlock_irqrestore (&serial_rwlock, flags);
				dbg_usbe (1, "---> CREATE %s no free interfaces %d %d",
					  device->name, i, MAX_INTERFACES);
				dbg_init (0, "CREATE Z2 sc=%d uc=%d", serial_created,
					  GET_USE_COUNT (THIS_MODULE));
				return;
			}
			serial_created++;

			// allocate private data
			if ((serial_private =
			     kmalloc (sizeof (struct usb_serial_private), GFP_ATOMIC)) == NULL) {
				serial_created--;
				//MOD_DEC_USE_COUNT;
				//dbg_init(0,"CREATE X1 sc=%d uc=%d",serial_created,GET_USE_COUNT(THIS_MODULE));
				write_unlock_irqrestore (&serial_rwlock, flags);
				dbg_usbe (1, "---> CREATE malloc failed %s", device->name);
				return;
			}

			serial_private->rwlock = RW_LOCK_UNLOCKED;
			serial_private_array[i] = serial_private;
			write_unlock_irqrestore (&serial_rwlock, flags);

			dbg_usbe (1, "---> calling serproto_create(%s,-,%u,%u,%u,%u,%u)", USBD_TTY_NAME,
				  CONFIG_USBD_SERIAL_IN_PKTSIZE, txqueue_urbs, txqueue_bytes, safe,
				  safe ? 2 : 0);

			if ((interface = serproto_create (USBD_TTY_NAME, serial_xmit_data,
							  (safe
							   ? (CONFIG_USBD_SERIAL_IN_PKTSIZE -
							      2) : CONFIG_USBD_SERIAL_IN_PKTSIZE),
							  txqueue_urbs, txqueue_bytes, safe,
							  safe ? 2 : 0)) < 0) {
				// lock and modify device array
				write_lock_irqsave (&serial_rwlock, flags);
				kfree (serial_private);
				serial_created--;
				//MOD_DEC_USE_COUNT;
				//dbg_init(0,"CREATE X2 sc=%d uc=%d",serial_created,GET_USE_COUNT(THIS_MODULE));
				write_unlock_irqrestore (&serial_rwlock, flags);
				dbg_usbe (1, "---> serproto_create FAILED");
				return;
			}
			// lock and modify device array
			write_lock_irqsave (&serial_rwlock, flags);
			serial_private->interface = interface;
			serial_private->device = device;
			write_unlock_irqrestore (&serial_rwlock, flags);

			function->privdata = serial_private;

			dbg_usbe (1, "---> START %s privdata assigned: %p interface: %d",
				  device->name, serial_private, interface);

			return;
		}
		break;

	case DEVICE_HUB_CONFIGURED:
		break;
	case DEVICE_RESET:
		break;
	case DEVICE_ADDRESS_ASSIGNED:
		break;
	case DEVICE_CONFIGURED:
		{
			struct usb_serial_private *serial_private;
			int interface;
			if ((serial_private =
			     (device->function_instance_array + port)->privdata) == NULL) {
				return;
			}
			interface = serial_private->interface;
			if (interface < 0 || interface >= MAX_INTERFACES) {
				return;
			}
			serproto_control (interface, SERPROTO_CONNECT);
		}
		break;
	case DEVICE_SET_INTERFACE:
#ifdef CONFIG_USBD_SERIAL_CDC
#endif
		break;
	case DEVICE_SET_FEATURE:
		break;
	case DEVICE_CLEAR_FEATURE:
		break;
	case DEVICE_DE_CONFIGURED:
		{
			struct usb_serial_private *serial_private;
			int interface;
			if ((serial_private =
			     (device->function_instance_array + port)->privdata) == NULL) {
				return;
			}
			interface = serial_private->interface;
			if (interface < 0 || interface >= MAX_INTERFACES) {
				return;
			}
			serproto_control (interface, SERPROTO_DISCONNECT);
		}
		break;
	case DEVICE_BUS_INACTIVE:
		break;
	case DEVICE_BUS_ACTIVITY:
		break;
	case DEVICE_POWER_INTERRUPTION:
		break;
	case DEVICE_HUB_RESET:
		break;

	case DEVICE_DESTROY:
		{
			struct usb_serial_private *serial_private;
			int interface;

			if ((serial_private =
			     (device->function_instance_array + port)->privdata) == NULL) {
				dbg_usbe (1, "---> DESTROY %s serial_private null", device->name);
				// XXX XXX MOD_DEC_USE_COUNT;
				// XXX XXX dbg_init(0,"DESTROY Z1 sc=%d uc=%d",serial_created,GET_USE_COUNT(THIS_MODULE));
				return;
			}
			dbg_usbe (1, "---> DESTROY %s serial_private %p", device->name,
				  serial_private);
			interface = serial_private->interface;

			if (interface < 0 || interface >= MAX_INTERFACES) {
				// XXX XXX MOD_DEC_USE_COUNT;
				// XXX XXX dbg_init(0,"DESTROY Z2 sc=%d uc=%d",serial_created,GET_USE_COUNT(THIS_MODULE));
				return;
			}

			if (serial_private_array[interface] != serial_private) {
				// XXX XXX MOD_DEC_USE_COUNT;
				// XXX XXX dbg_init(0,"DESTROY Z3 sc=%d uc=%d",serial_created,GET_USE_COUNT(THIS_MODULE));
				return;
			}

			write_lock_irqsave (&serial_rwlock, flags);
			serial_private_array[interface] = NULL;
			kfree (serial_private);
			serial_created--;
			write_unlock_irqrestore (&serial_rwlock, flags);

			serproto_destroy (interface);
			// XXX XXX MOD_DEC_USE_COUNT;
			// XXX XXX dbg_init(0,"DESTROY sc=%d uc=%d",serial_created,GET_USE_COUNT(THIS_MODULE));

			dbg_usbe (1, "---> STOP %s", device->name);
			return;
		}
		break;

	case DEVICE_FUNCTION_PRIVATE:
		break;
	}
}



/* serial_recv_urb - called to indicate URB has been received
 * @urb - pointer to struct urb
 *
 * Return non-zero if we failed and urb is still valid (not disposed)
 */
int serial_recv_urb (struct urb *urb)
{
	int port = 0;		// XXX compound device
	struct usb_device_instance *device = urb->device;
	struct usb_serial_private *serial_private =
	    (device->function_instance_array + port)->privdata;
	int interface = serial_private->interface;

	dbg_rx (2, "length=%d", urb->actual_length);
	dbgPRINTmem (dbgflg_usbdfd_rx, 3, urb->buffer, urb->actual_length);

#ifdef CONFIG_USBD_SERIAL_SAFE
	if (safe) {
		__u16 fcs;

		if ((fcs = fcs_compute10 (urb->buffer, urb->actual_length, CRC10_INITFCS))) {
			dbg_rx (0, "CRC check failed");
			//return -EINVAL;
		} else {
			// recover the length from the trailer
			urb->actual_length = (urb->buffer[urb->actual_length - 2] >> 2);
		}
	}
	dbg_rx (2, "revised length=%d", urb->actual_length);
#endif
#if 0
	if (0 != dbgflg_usbdfd_loopback) {
		if (usbd_send_urb (urb)) {
			//XXX verify not freed in usbd_send_urb()
			//urb->buffer = NULL;
			//urb->actual_length = 0;
			//usbd_dealloc_urb(urb);
		}
		return 0;
	}
#endif

	// push the data up
	if (serproto_recv (interface, urb->buffer, urb->actual_length)) {
		return (1);
	}
	// free urb
	usbd_recycle_urb (urb);

	return (0);
}


struct usb_function_operations function_ops = {
	event:serial_event,
	recv_urb:serial_recv_urb,
	urb_sent:serial_urb_sent
};

struct usb_function_driver function_driver = {
	name:"function prototype",
	ops:&function_ops,
	device_description:&serial_device_description,
	configurations:sizeof (serial_description) / sizeof (struct usb_configuration_description),
	configuration_description:serial_description,
	this_module:THIS_MODULE,
};


/*
 * serial_modinit - module init
 *
 */
static int __init serial_modinit (void)
{
	debug_option *op = find_debug_option (dbg_table, "ser");

	printk (KERN_INFO "%s (%s)\n", __usbd_module_info, dbg);

#ifdef CONFIG_USBD_SERIAL_SAFE
	printk (KERN_INFO "vendor_id: %04x product_id: %04x safe: %d padded: %d\n", vendor_id,
		product_id, safe, padded);
#else
	printk (KERN_INFO "vendor_id: %04x product_id: %04x\n", vendor_id, product_id);
	if (safe || padded) {
		printk (KERN_ERR "serial_fd: not compiled for safe mode\n");
		return -EINVAL;
	}
#endif
	printk (KERN_INFO "dbg: %s\n", dbg);

	if (NULL != op) {
		op->sub_table = serproto_get_dbg_table ();
	}
	if (0 != scan_debug_options ("serial_fd", dbg_table, dbg)) {
		return (-EINVAL);
	}

	if (vendor_id) {
		serial_device_description.idVendor = vendor_id;
	}
	if (product_id) {
		serial_device_description.idProduct = product_id;
	}
	// Initialize the function registration code.
	//
	//if (usbd_strings_init()) {
	//    return -EINVAL;
	//}
	// initialize the serproto library
	//
	if (serproto_modinit (USBD_TTY_NAME, MAX_INTERFACES)) {
		return -EINVAL;
	}
	// register us with the usb device support layer
	//
	if (usbd_register_function (&function_driver)) {
		serproto_modexit ();
		return -EINVAL;
	}
	dbg_loop (1, "LOOPBACK mode");

	// return
	return 0;
}


/* serial_modexit - module cleanup
 */
static void __exit serial_modexit (void)
{
	// de-register us with the usb device support layer
	// 
	usbd_deregister_function (&function_driver);

	// tell the serproto library to exit
	//
	serproto_modexit ();

}

module_init (serial_modinit);
module_exit (serial_modexit);
