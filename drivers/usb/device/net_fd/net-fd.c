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

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-module.h"

MODULE_AUTHOR ("sl@lineo.com, tbr@lineo.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION ("USB Device Network Function");

USBD_MODULE_INFO ("net_fd 0.1");

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
#include <asm/system.h>

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-inline.h"
#include "../usbd-arch.h"

#include "crc32.h"
#include "../hotplug.h"

#include "netproto.h"

#define MCCI_ENABLE_CRC 0x03


#if !defined (CONFIG_USBD_VENDORID) && !defined(CONFIG_USBD_NET_VENDORID)
#error No Vendor ID
#endif
#if !defined (CONFIG_USBD_PRODUCTID) && !defined(CONFIG_USBD_NET_PRODUCTID)
#error No Product ID
#endif

#if !defined(CONFIG_USBD_NET_ALWAYSUP)
#define CONFIG_USBD_NET_ALWAYSUP 0;
#endif


#if defined(CONFIG_USBD_NET_VENDORID) && (CONFIG_USBD_NET_VENDORID > 0)
#undef CONFIG_USBD_VENDORID
#define CONFIG_USBD_VENDORID                    CONFIG_USBD_NET_VENDORID
#endif

#if defined(CONFIG_USBD_NET_PRODUCTID) && (CONFIG_USBD_NET_PRODUCTID > 0)
#undef CONFIG_USBD_PRODUCTID
#define CONFIG_USBD_PRODUCTID                   CONFIG_USBD_NET_PRODUCTID
#endif

#ifndef CONFIG_USBD_NET_LOCAL_OUI
#define CONFIG_USBD_NET_LOCAL_OUI               0x400001
	//#warning Setting default MAC LOCAL_OUI
#endif

#ifndef CONFIG_USBD_NET_REMOTE_OUI
#define CONFIG_USBD_NET_REMOTE_OUI              0x400002
	//#warning Setting default MAC REMOTE_OUI
#endif

#ifndef CONFIG_USBD_NET_LOCAL_MACADDR
#define CONFIG_USBD_NET_LOCAL_MACADDR           "400001000001"
	//#warning Setting default Local MAC Address
#endif

#ifndef CONFIG_USBD_NET_REMOTE_MACADDR
#define CONFIG_USBD_NET_REMOTE_MACADDR          "400002000001"
	//#warning Setting default Remote MAC Address
#endif

#ifndef CONFIG_USBD_MAXPOWER
#define CONFIG_USBD_MAXPOWER                    0
#endif

#ifndef CONFIG_USBD_MANUFACTURER
#define CONFIG_USBD_MANUFACTURER                "Lineo"
#endif

#define MAXTRANSFER                                     1514

#ifndef CONFIG_USBD_VENDORID
#error "CONFIG_USBD_VENDORID not defined"
#endif

#ifndef CONFIG_USBD_PRODUCTID
#error "CONFIG_USBD_PRODUCTID not defined"
#endif

#ifndef CONFIG_USBD_NET_IFNAME
#define CONFIG_USBD_NET_IFNAME                  "usbd"
#endif

#ifndef CONFIG_USBD_PRODUCT_NAME
#ifndef CONFIG_USBD_NET_CDC
#define CONFIG_USBD_PRODUCT_NAME "CDC Network Driver"
#else
#define CONFIG_USBD_PRODUCT_NAME "Linux Network Driver"
#endif
#endif

#ifndef CONFIG_USBD_SERIAL_NUMBER_STR
#define CONFIG_USBD_SERIAL_NUMBER_STR           ""
#endif

/*
 * USB 2.0 spec does not mention it, but MaxPower is expected to be at least one 
 * and is tested for in USB configuration tests.
 */
#ifdef CONFIG_USBD_SELFPOWERED
#define BMATTRIBUTE BMATTRIBUTE_RESERVED | BMATTRIBUTE_SELF_POWERED
#define BMAXPOWER                               1
#else
#define BMATTRIBUTE BMATTRIBUTE_RESERVED
#define BMAXPOWER                               CONFIG_USBD_MAXPOWER
#endif



/*
 * setup some default values for pktsizes and endpoint addresses.
 */

#ifndef CONFIG_USBD_NET_OUT_PKTSIZE
#define CONFIG_USBD_NET_OUT_PKTSIZE             64
#endif

#ifndef CONFIG_USBD_NET_IN_PKTSIZE
#define CONFIG_USBD_NET_IN_PKTSIZE              64
#endif

#ifndef CONFIG_USBD_NET_INT_PKTSIZE
#define CONFIG_USBD_NET_INT_PKTSIZE             16
#endif

#ifndef CONFIG_USBD_NET_OUT_ENDPOINT
#define CONFIG_USBD_NET_OUT_ENDPOINT            1
#endif

#ifndef CONFIG_USBD_NET_IN_ENDPOINT
#define CONFIG_USBD_NET_IN_ENDPOINT             2
#endif

#ifndef CONFIG_USBD_NET_INT_ENDPOINT
#define CONFIG_USBD_NET_INT_ENDPOINT                3
#endif


/*
 * check for architecture specific endpoint configurations
 */

#if     defined(ABS_OUT_ADDR)
        //#warning
        //#warning USING ABS ENDPOINT OUT ADDRESS
        #undef CONFIG_USBD_NET_OUT_ENDPOINT

        #if     ABS_OUT_ADDR > 0
                #define CONFIG_USBD_NET_OUT_ENDPOINT            ABS_OUT_ADDR
        #endif

#elif   defined(MAX_OUT_ADDR) && defined(CONFIG_USBD_NET_OUT_ENDPOINT) && (CONFIG_USBD_NET_OUT_ENDPOINT > MAX_OUT_ADDR)
                //#warning
                //#warning USING DEFAULT ENDPOINT OUT ADDRESS
        #undef CONFIG_USBD_NET_OUT_ENDPOINT
        #define CONFIG_USBD_NET_OUT_ENDPOINT            DFL_OUT_ADDR

#endif /* elif */


#if     defined(ABS_IN_ADDR)
	//#warning
	//#warning USING ABS ENDPOINT IN ADDRESS
        #undef CONFIG_USBD_NET_IN_ENDPOINT

        #if     ABS_IN_ADDR > 0
                #define CONFIG_USBD_NET_IN_ENDPOINT             ABS_IN_ADDR
        #endif

#elif   defined(MAX_IN_ADDR) && defined(CONFIG_USBD_NET_IN_ENDPOINT) && (CONFIG_USBD_NET_IN_ENDPOINT > MAX_IN_ADDR)
                //#warning
                //#warning USING DEFAULT ENDPOINT IN ADDRESS
        #undef CONFIG_USBD_NET_IN_ENDPOINT
        #define CONFIG_USBD_NET_IN_ENDPOINT             DFL_IN_ADDR

#endif /* elif */


#if     defined(ABS_INT_ADDR)
	//#warning
	//#warning USING ABS ENDPOINT INT ADDRESS
        #undef CONFIG_USBD_NET_INT_ENDPOINT

        #if     ABS_INT_ADDR
                #define CONFIG_USBD_NET_INT_ENDPOINT            ABS_INT_ADDR
        #endif

#elif   defined(MAX_INT_ADDR) && defined(CONFIG_USBD_NET_INT_ENDPOINT) && (CONFIG_USBD_NET_INT_ENDPOINT > MAX_INT_ADDR)
	//#warning
	//#warning USING DEFAULT ENDPOINT INT ADDRESS
        #undef CONFIG_USBD_NET_INT_ENDPOINT
        #define CONFIG_USBD_NET_INT_ENDPOINT            DFL_INT_ADDR

#endif /* elif */



#if     defined(MAX_OUT_PKTSIZE) && defined(CONFIG_USBD_NET_OUT_PKTSIZE) && CONFIG_USBD_NET_OUT_PKTSIZE > MAX_OUT_PKTSIZE
	//#warning
	//#warning OVERIDING ENDPOINT OUT PKTSIZE 
        #undef CONFIG_USBD_NET_OUT_PKTSIZE
        #define CONFIG_USBD_NET_OUT_PKTSIZE             MAX_OUT_PKTSIZE
#endif

#if     defined(MAX_IN_PKTSIZE) && defined(CONFIG_USBD_NET_IN_PKTSIZE) && CONFIG_USBD_NET_IN_PKTSIZE > MAX_IN_PKTSIZE
	//#warning
	//#warning OVERIDING ENDPOINT IN PKTSIZE 
        #undef CONFIG_USBD_NET_IN_PKTSIZE
        #define CONFIG_USBD_NET_IN_PKTSIZE              MAX_IN_PKTSIZE
#endif

#if     defined(MAX_INT_PKTSIZE) && defined(CONFIG_USBD_NET_INT_PKTSIZE) && CONFIG_USBD_NET_INT_PKTSIZE > MAX_INT_PKTSIZE
	//#warning
	//#warning OVERIDING ENDPOINT INT PKTSIZE
        #undef CONFIG_USBD_NET_INT_PKTSIZE
        #define CONFIG_USBD_NET_INT_PKTSIZE             MAX_INT_PKTSIZE
#endif



/* Module Parameters ************************************************************************* */

#define MAX_INTERFACES 1

static char *if_name = CONFIG_USBD_NET_IFNAME;
#if defined(CONFIG_USBD_NET_CDC) || defined(CONFIG_USBD_NET_MDLM)
static char *remote_mac_address;
#endif
static char *local_mac_address;

static char *dbg = NULL;

#if defined(CONFIG_USBD_NET_CDC) || defined(CONFIG_USBD_NET_MDLM)
static char remote_mac_address_buffer[14];
#endif
static u32 vendor_id;
static u32 product_id;
static int alwaysup = CONFIG_USBD_NET_ALWAYSUP;
static int out_pkt_sz = CONFIG_USBD_NET_OUT_PKTSIZE;
static int in_pkt_sz = CONFIG_USBD_NET_IN_PKTSIZE;


MODULE_PARM (if_name, "s");
#if defined(CONFIG_USBD_NET_CDC) || defined(CONFIG_USBD_NET_MDLM)
MODULE_PARM (remote_mac_address, "s");
#endif
MODULE_PARM (local_mac_address, "s");
MODULE_PARM (vendor_id, "i");
MODULE_PARM (product_id, "i");
MODULE_PARM (alwaysup, "i");
MODULE_PARM (out_pkt_sz, "i");
MODULE_PARM (in_pkt_sz, "i");
MODULE_PARM (dbg, "s");

MODULE_PARM_DESC (if_name, "Network Interface name prefix");
#if defined(CONFIG_USBD_NET_CDC) || defined(CONFIG_USBD_NET_MDLM)
MODULE_PARM_DESC (remote_mac_address, "Remote MAC");
#endif
MODULE_PARM_DESC (local_mac_address, "Local MAC");
MODULE_PARM_DESC (vendor_id, "vendor id");
MODULE_PARM_DESC (product_id, "product id");
MODULE_PARM_DESC (alwaysup, "always up");
MODULE_PARM_DESC (dbg, "dbg string");

/* Debug switches (module parameter "dbg=...") *********************************************** */

extern int dbgflg_usbdfd_init;
int dbgflg_usbdfd_usbe;
int dbgflg_usbdfd_rx;
int dbgflg_usbdfd_tx;
int dbgflg_usbdfd_ep0;

static debug_option dbg_table[] = {
	{&dbgflg_usbdfd_init, NULL, "init", "initialization and termination"},
	{&dbgflg_usbdfd_ep0, NULL, "ep0", "End Point 0 (setup) packet handling"},
	{&dbgflg_usbdfd_rx, NULL, "rx", "USB RX (host->device) handling"},
	{&dbgflg_usbdfd_tx, NULL, "tx", "USB TX (device->host) handling"},
	{&dbgflg_usbdfd_usbe, NULL, "usbe", "USB events"},
	{NULL, NULL, "net", "network device handling"},
	{NULL, NULL, NULL, NULL},
};

#define dbg_init(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_init,lvl,fmt,##args)
#define dbg_ep0(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_ep0,lvl,fmt,##args)
#define dbg_rx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_rx,lvl,fmt,##args)
#define dbg_tx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_tx,lvl,fmt,##args)
#define dbg_usbe(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_usbe,lvl,fmt,##args)

#define NET_INUSE       0x01
#define NET_ATTACHED    0x02

struct usb_net_private {
	int flags;
	int interface;
	struct usb_device_instance *device;
	int crc;
	unsigned int maxtransfer;
	char name[IFNAMSIZ];
	int index;
#ifdef CONFIG_ARCH_SA1100
	int first;
#endif
};

static struct usb_net_private net_private_array[MAX_INTERFACES];

static rwlock_t net_rwlock = RW_LOCK_UNLOCKED;	// lock for netproto device array access

#define RXECHO
#define TXECHO

typedef enum net_device_event {

	NET_UNKOWN,		// net  - unknown
	NET_SET_CRC,		// net  - vendor command

} net_device_event_t;



static __devinitdata unsigned char default_dev_addr[ETH_ALEN] = {
	0x40, 0x00, 0x00, 0x00, 0x00, 0x01
};


/* proc file system ********************************************************************** */
static void net_device_check_condition(usb_device_event_t event);



/* USB Configuration Description ************************************************************* */

#if !defined(CONFIG_USBD_NET_SAFE) && !defined(CONFIG_USBD_NET_CDC) && !defined(CONFIG_USBD_NET_MDLM)
        #error One of CONFIG_USBD_NET_{SAFE,CDC,MDLM} must be set
#endif

#if defined(CONFIG_USBD_NET_MDLM) && defined(CONFIG_USBD_NET_SAFE) 
        #error Only one of CONFIG_USBD_NET_SAFE or CONFIG_USBD_NET_MDLM can be set
#endif

#if defined(CONFIG_USBD_NET_MDLM) && defined(CONFIG_USBD_NET_CDC) 
        #error One of CONFIG_USBD_NET_CDC or CONFIG_USBD_NET_MDLM can be set
#endif


/* USB Safe Configuration ******************************************************************** */
#ifdef CONFIG_USBD_NET_SAFE

/*
 * Simple Ethernet Configuration
 */


/* Communication Interface Class descriptions
 */

static __devinitdata struct usb_endpoint_description net_default[] = {
      {bEndpointAddress:CONFIG_USBD_NET_OUT_ENDPOINT,
	      bmAttributes:BULK,
	      wMaxPacketSize:CONFIG_USBD_NET_OUT_PKTSIZE,
	      bInterval:0,
	      direction:OUT,
              transferSize:MAXTRANSFER + 4,},

      {bEndpointAddress:CONFIG_USBD_NET_IN_ENDPOINT,
	      bmAttributes:BULK,
	      wMaxPacketSize:CONFIG_USBD_NET_IN_PKTSIZE,
	      bInterval:0,
	      direction:IN,
              transferSize:MAXTRANSFER + 4,},

#if defined(CONFIG_USBD_NET_INT_ENDPOINT) && (CONFIG_USBD_NET_INT_ENDPOINT > 0)
      {bEndpointAddress:CONFIG_USBD_NET_INT_ENDPOINT,
	      bmAttributes:INTERRUPT,
	      wMaxPacketSize:CONFIG_USBD_NET_INT_PKTSIZE,
	      bInterval:10,
	      direction:IN,
              transferSize:CONFIG_USBD_NET_INT_PKTSIZE,},
#endif

};

/* Data Interface Alternate description(s)
 */
static __devinitdata struct usb_alternate_description net_data_alternate_descriptions[] = {
	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iInterface:"",
                #else
                        iInterface:"Simple Network Data Interface - Bulk mode",
                #endif
                bAlternateSetting:0,
                endpoints:sizeof (net_default) / sizeof (struct usb_endpoint_description),
                endpoint_list:net_default,},
};

/* Interface description(s)
 */
static __devinitdata struct usb_interface_description net_interfaces[] = {
	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iInterface:"",
                #else
                        iInterface:"Simple Network Data Interface",
                #endif
                bInterfaceClass:LINEO_CLASS,
                bInterfaceSubClass:LINEO_SUBCLASS_SAFENET,
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        bInterfaceProtocol:LINEO_SAFENET_CRC_PADDED,
                #else
                        bInterfaceProtocol:LINEO_SAFENET_CRC,
                #endif
                alternates:sizeof (net_data_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:net_data_alternate_descriptions,},
};
#endif				/* CONFIG_USBD_NET_SAFE */

/* USB CDC Configuration ********************************************************************* */
#ifdef CONFIG_USBD_NET_CDC

/*
 * CDC Ethernet Configuration
 */

/* Communication Interface Class descriptions
 */
static struct usb_class_description cdc_comm_class_descriptions[] = {
	{ CS_INTERFACE, USB_ST_HEADER,  0, { header: { bcdCDC: CLASS_BCD_VERSION, } }},

	{ CS_INTERFACE, USB_ST_ENF,     0,
	      {ethernet_networking:
		      {iMACAddress:NULL,
			      bmEthernetStatistics: 0, wMaxSegmentSize:1514,
              wNumberMCFilters: 0, bNumberPowerFilters:0,}}},

	{ CS_INTERFACE, USB_ST_UF,      1, { union_function: { bMasterInterface: 0, bSlaveInterface: { 1 }, }}},
};

/* Data Interface Alternate 1 endpoints
 */
static __devinitdata struct usb_endpoint_description net_alt_1_endpoints[] = {
      {bEndpointAddress:CONFIG_USBD_NET_OUT_ENDPOINT,
	      bmAttributes:BULK,
	      wMaxPacketSize:CONFIG_USBD_NET_OUT_PKTSIZE,
	      bInterval:0,
	      direction:OUT,
              transferSize:MAXTRANSFER + 2,},

      {bEndpointAddress:CONFIG_USBD_NET_IN_ENDPOINT,
	      bmAttributes:BULK,
	      wMaxPacketSize:CONFIG_USBD_NET_IN_PKTSIZE,
	      bInterval:0,
	      direction:IN,
              transferSize:MAXTRANSFER + 2,},

#if defined(CONFIG_USBD_NET_INT_ENDPOINT) && (CONFIG_USBD_NET_INT_ENDPOINT > 0)
      {bEndpointAddress:CONFIG_USBD_NET_INT_ENDPOINT,
	      bmAttributes:INTERRUPT,
	      wMaxPacketSize:CONFIG_USBD_NET_INT_PKTSIZE,
	      bInterval:10,
	      direction:IN,
              transferSize:CONFIG_USBD_NET_INT_PKTSIZE,},
#endif
};


/* Data Interface Alternate description(s)
 */
static __devinitdata struct usb_alternate_description cdc_comm_alternate_descriptions[] = {
	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iInterface:"",
                #else
                        iInterface:"CDC Network Comm Interface",
                #endif
                bAlternateSetting:0,
                classes:sizeof (cdc_comm_class_descriptions) / sizeof (struct usb_class_description),
                class_list:cdc_comm_class_descriptions,},
};

static __devinitdata struct usb_alternate_description cdc_data_alternate_descriptions[] = {
	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iInterface:"Bulk Disabled",
                #else
                        iInterface:"CDC Network Data Interface - Disabled mode",
                #endif
#ifdef CONFIG_ARCH_LUBBOCK
                bAlternateSetting:1,},
#else
                bAlternateSetting:0,},
#endif

	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iInterface:"Data Enabled",
                #else
                        iInterface:"CDC Network Data Interface - Bulk mode",
                #endif
#ifdef CONFIG_ARCH_LUBBOCK
                bAlternateSetting:0,
#else
                bAlternateSetting:1,
#endif
                endpoints:sizeof (net_alt_1_endpoints) / sizeof (struct usb_endpoint_description),
                endpoint_list:net_alt_1_endpoints,},
};

/* Interface description(s)
 */
static __devinitdata struct usb_interface_description cdc_interfaces[] = {
	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iInterface:"",
                #else
                        iInterface:"CDC Network Communication Interface",
                #endif
                bInterfaceClass:COMMUNICATIONS_INTERFACE_CLASS,
                bInterfaceSubClass:COMMUNICATIONS_ENCM_SUBCLASS,
                bInterfaceProtocol:COMMUNICATIONS_NO_PROTOCOL,
                alternates:sizeof (cdc_comm_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:cdc_comm_alternate_descriptions,},

	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iInterface:"",
                #else
                        iInterface:"CDC Network Data Interface",
                #endif
                bInterfaceClass:DATA_INTERFACE_CLASS,
                bInterfaceSubClass:COMMUNICATIONS_NO_SUBCLASS,
                bInterfaceProtocol:COMMUNICATIONS_NO_PROTOCOL,
                alternates:sizeof (cdc_data_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:cdc_data_alternate_descriptions,},
};


#endif				/* CONFIG_USBD_NET_CDC */

/* USB MDLM Configuration ******************************************************************** */
#ifdef CONFIG_USBD_NET_MDLM

/*
 * MDML Ethernet Configuration
 */

/* Communication Interface Class descriptions
 */
static struct usb_class_description mdlm_comm_class_descriptions[] = {
{ CS_INTERFACE, USB_ST_HEADER,  0, { header: { bcdCDC: CLASS_BCD_VERSION, } }},
	
{ CS_INTERFACE, USB_ST_MDLM, 0, { mobile_direct: {

                                bcdVersion:0x0100,
                                bGUID:{
                                            0x5d, 0x34, 0xcf, 0x66, 0x11, 0x18, 0x11, 0xd6,
                                            0xa2, 0x1a, 0x00, 0x01, 0x02, 0xca, 0x9a, 0x7f}}}
	 },

      { CS_INTERFACE, USB_ST_MDLMD, 2, {mobile_direct_detail:{
                                                      // XXX FIXME
                                bGuidDescriptorType:0x00,
                                bDetailData:{0x0, 0x3}}}
      },

      //{ CS_INTERFACE,  USB_ST_MDLMD, 1, { mobile_direct_detail: {
      //    bGuidDescriptorType: 0x01,
	//    bDetailData: { 0x02 } } }
	//},

	{ CS_INTERFACE, USB_ST_ENF, 0,
	      {ethernet_networking:
		      {iMACAddress:"402233445566",
                                bmEthernetStatistics: 0, wMaxSegmentSize:1514,
                                wNumberMCFilters: 0, bNumberPowerFilters:0,}}
	 },

	//{ CS_INTERFACE,  USB_ST_UF,      1, { union_function: { bMasterInterface: 0, bSlaveInterface: { 0 }, }}},
};


/* Data Interface Alternate 1 endpoints
 */
static __devinitdata struct usb_endpoint_description mdlm_alt_1_endpoints[] = {
      {bEndpointAddress:CONFIG_USBD_NET_OUT_ENDPOINT,
	      bmAttributes:BULK,
	      wMaxPacketSize:CONFIG_USBD_NET_OUT_PKTSIZE,
	      bInterval:0,
	      direction:OUT,
              transferSize:MAXTRANSFER + 2,},

      {bEndpointAddress:CONFIG_USBD_NET_IN_ENDPOINT,
	      bmAttributes:BULK,
	      wMaxPacketSize:CONFIG_USBD_NET_IN_PKTSIZE,
	      bInterval:0,
	      direction:IN,
              transferSize:MAXTRANSFER + 2,},

#if defined(CONFIG_USBD_NET_INT_ENDPOINT) && (CONFIG_USBD_NET_INT_ENDPOINT > 0)
      {bEndpointAddress:CONFIG_USBD_NET_INT_ENDPOINT,
	      bmAttributes:INTERRUPT,
	      wMaxPacketSize:CONFIG_USBD_NET_INT_PKTSIZE,
	      bInterval:10,
	      direction:IN,
              transferSize:CONFIG_USBD_NET_INT_PKTSIZE,},
#endif
};


/* Data Interface Alternate description(s)
 */
static __devinitdata struct usb_alternate_description mdlm_alternate_descriptions[] = {
	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iInterface:"",
                #else
                        iInterface:"MDLM Network Communication Interface",
                #endif
                bAlternateSetting:0,
                classes:sizeof (mdlm_comm_class_descriptions) / sizeof (struct usb_class_description),
                class_list:mdlm_comm_class_descriptions,
                endpoints:sizeof (mdlm_alt_1_endpoints) / sizeof (struct usb_endpoint_description),
                endpoint_list:mdlm_alt_1_endpoints,},
};

/* Interface description(s)
 */
static __devinitdata struct usb_interface_description mdlm_interfaces[] = {
	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iInterface:"",
                #else
                        iInterface:"MDLM Network Interface",
                #endif
                bInterfaceClass:COMMUNICATIONS_INTERFACE_CLASS,
                bInterfaceSubClass:COMMUNICATIONS_MDLM_SUBCLASS,
                bInterfaceProtocol:COMMUNICATIONS_NO_PROTOCOL,
                alternates:sizeof (mdlm_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:mdlm_alternate_descriptions,},
};


#endif				/* CONFIG_USBD_NET_CDC */

/* USB Configuration ************************************************************************* */

/* Configuration description(s)
 */
struct __devinitdata usb_configuration_description net_description[] = {
#ifdef CONFIG_USBD_NET_CDC
	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iConfiguration:"",
                #else
                        iConfiguration:"CDC 1.1 Configuration",
                #endif
                bmAttributes:BMATTRIBUTE,
                bMaxPower:BMAXPOWER,
                interfaces:sizeof (cdc_interfaces) / sizeof (struct usb_interface_description),
                interface_list:cdc_interfaces,},
#endif				/* CONFIG_USBD_NET_CDC */

#ifdef CONFIG_USBD_NET_MDLM
	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iConfiguration:"",
                #else
	                iConfiguration:"MDLM Network Configuration",
                #endif
	        bmAttributes:BMATTRIBUTE,
	        bMaxPower:BMAXPOWER,
	        interfaces:sizeof (mdlm_interfaces) / sizeof (struct usb_interface_description),
                interface_list:mdlm_interfaces,},
#endif				/* CONFIG_USBD_NET_SAFE */

#ifdef CONFIG_USBD_NET_SAFE
	{
                #if defined(CONFIG_USBD_NET_NO_STRINGS)
                        iConfiguration:"",
                #else
                        iConfiguration:"USB Simple Ethernet Configuration",
                #endif
                bmAttributes:BMATTRIBUTE,
                bMaxPower:BMAXPOWER,
                interfaces:sizeof (net_interfaces) / sizeof (struct usb_interface_description),
                interface_list:net_interfaces,},
#endif				/* CONFIG_USBD_NET_SAFE */

};

/* Device Description
 */
struct __devinitdata usb_device_description net_device_description = {
	bDeviceClass:COMMUNICATIONS_DEVICE_CLASS,
        bDeviceSubClass:COMMUNICATIONS_ENCM_SUBCLASS,
	bDeviceProtocol:0,	// XXX
	idVendor:CONFIG_USBD_VENDORID,
	idProduct:CONFIG_USBD_PRODUCTID,
	iManufacturer:CONFIG_USBD_MANUFACTURER,
	iProduct:CONFIG_USBD_PRODUCT_NAME,
	iSerialNumber:CONFIG_USBD_SERIAL_NUMBER_STR,
};


#undef NET_DESTRUCTOR
#ifdef NET_DESTRUCTOR
#warning DESTRUCTOR
void net_dev_kfree_skb_any (struct sk_buff *skb)
{
	printk (KERN_DEBUG
		"net_dev_kfree_skb: skb: %p head: %p data: %p tail: %p end: %p len: %d\n", skb,
		skb->head, skb->data, skb->tail, skb->end, skb->len);
}
#endif

/* *
 * net_dev_alloc_skb - allocate an skb 
 * @len:
 *
 * Allocate an skb. 
 */
static __inline__ struct sk_buff *net_dev_alloc_skb (int len)
{
	struct sk_buff *skb;
	if ((skb = netproto_dev_alloc_skb (len))) {
#ifdef NET_DESTRUCTOR
		skb->destructor = net_dev_kfree_skb_any;
#endif
	}
	return skb;
}


static int net_xmit_skb (int, struct sk_buff *);
static int net_set_addr (int, void *, int);
static int net_tx_timeout (int);

/* Intialize and destroy network interfaces ************************************************** */

/* *
 * net_create - create an interface
 * @device: usb device instance
 *
 */
void net_create (struct usb_device_instance *device, struct usb_function_instance *function)
{
	int i;
	struct usb_net_private *net_private;

	dbg_init (1, "---> privdata: %p", function->privdata);
        // lock and find an empty slot
        {			// get module lock, search for empty device slot, if successful allocate and save
                unsigned long flags;
                write_lock_irqsave (&net_rwlock, flags);

                // check if we already have an interface
                if ((net_private = function->privdata) && (net_private->flags & NET_INUSE)) {
                        write_unlock_irqrestore (&net_rwlock, flags);
                        return;
                }
                // serial number munge

                for (i = 0; i < MAX_INTERFACES; i++) {
                        dbg_init (1, "i: %d net_private_array[i].device: %p\n", i, net_private_array[i].device);
                        if (!net_private_array[i].flags) {
                                break;
                        }
                }
                if (i >= MAX_INTERFACES) {
                        dbg_init (1, "%s i >= MAX_INTERFACES %d %d", device->name, i, MAX_INTERFACES);
                        write_unlock_irqrestore (&net_rwlock, flags);
			return;
		}

		net_private = &net_private_array[i];
		net_private->flags |= NET_INUSE;
		net_private->index = i;

		write_unlock_irqrestore (&net_rwlock, flags);
	}

	function->privdata = net_private;

	dbg_init (1, "function->privdata set to: %p\n", function->privdata);

	net_private->device = device;
#ifdef CONFIG_ARCH_SA1100
	net_private->first = 1;
#endif

	MOD_INC_USE_COUNT;



	// should check for buffer overflow...
	if (strlen (if_name) < (IFNAMSIZ - 3)) {
		sprintf (net_private->name, "%s%d", if_name, net_private->index);
	} else {
		dbg_init (0, "if_name too long or too short\n");
		net_private->name[0] = '\0';
	}

	// create network interface
	if ((net_private->interface = netproto_create (net_private->name, net_xmit_skb, net_set_addr, 
                                        net_tx_timeout, default_dev_addr, ETH_ALEN, 1500, 1, 4000, 0)) < 0) 
        {	// lock and modify device array
		dbg_init (0, "FAILED\n");
		return;
	}

	net_private->flags |= NET_ATTACHED;

#if 0
#ifdef CONFIG_USBD_NET_CDC
	switch (device->configuration) {
	case 1:
		net_private->crc = 1;
	case 0:
		net_private->maxtransfer = MAXTRANSFER + 4 + in_pkt_sz;
		break;
	}
#else				/* CONFIG_USBD_NET_CDC */
        device->interface = 0;
        device->alternate = 0;

	net_private->crc = 1;
	net_private->maxtransfer = MAXTRANSFER + 4 + in_pkt_sz;
#endif				/* CONFIG_USBD_NET_CDC */
#else
	net_private->crc = 1;
	net_private->maxtransfer = MAXTRANSFER + 4 + in_pkt_sz;
#endif

	net_private->device = device;
#if 0
	netproto_on (net_private->interface);
#endif
}

/* *
 * net_destroy - destroy an interface
 * @device: usb device instance
 *
 */
void net_destroy (struct usb_device_instance *device)
{
	struct usb_net_private *net_private;
	struct usb_function_instance *function;
	int port = 0;		// XXX compound device

	dbg_init (1, "-      -       -       ");

	if (!(function = device->function_instance_array + port)) {
		return;
	}

	dbg_init (1, "destroying");

	if (!(net_private = function->privdata)) {
		dbg_init (1, "%s net_private null", device->name);
		return;
	}
	if (net_private->flags & NET_ATTACHED) {
		if (netproto_destroy (net_private->interface)) {
			dbg_init (1, "error destroying %d", net_private->index);
		}
	}

	{			// get module lock and delete from device array
		unsigned int flags;
		write_lock_irqsave (&net_rwlock, flags);
		net_private->device = NULL;
		net_private->flags = 0;
		function->privdata = NULL;
		write_unlock_irqrestore (&net_rwlock, flags);
	}

	netproto_off (net_private->interface);

	MOD_DEC_USE_COUNT;
	return;
}


/* Called when a USB Device is created or destroyed  ***************************************** */


// XXX this should get passed a device structure, not bus
static void net_function_init (struct usb_bus_instance *bus,
			       struct usb_device_instance *device,
			       struct usb_function_driver *function_driver)
{
#if defined(CONFIG_USBD_NET_MDLM) || defined(CONFIG_USBD_NET_CDC)
	int i;
#endif

	struct usb_function_instance *function;

	__u32 serial;		// trim for use in MAC addr

	int port = 0;		// XXX compound device

	//dbg_init(1, "-------------------------------------------------------------------------------"); 

	if (!(function = device->function_instance_array + port)) {
		return;
	}

	serial = bus->serial_number & 0x00ffffff;

#if defined(CONFIG_USBD_NET_CDC) || defined(CONFIG_USBD_NET_MDLM)
	if (remote_mac_address && (strlen (remote_mac_address) == 12)) {
		memcpy (remote_mac_address_buffer, remote_mac_address, 12);
	} else if (serial) {
		sprintf (remote_mac_address_buffer, "%06X%06X", CONFIG_USBD_NET_REMOTE_OUI, serial);
	} else {
		if (strlen (CONFIG_USBD_NET_REMOTE_MACADDR)) {
			memcpy (remote_mac_address_buffer, CONFIG_USBD_NET_REMOTE_MACADDR, 12);
		} else {
			memcpy (remote_mac_address_buffer, "400002000001", 12);
		}
	}
	remote_mac_address_buffer[12] = '\0';

	// sanity check
	if (strlen (remote_mac_address_buffer) != 12) {
		dbg_init (0, "mac_address has wrong length: %s %d", remote_mac_address_buffer,
			  strlen (remote_mac_address_buffer));
		//return -EINVAL;
		return;
	}
	dbg_init (1, "remote mac_address: %s", remote_mac_address_buffer);

#if defined(CONFIG_USBD_NET_CDC)
	// set the mac address in the descriptor
	for (i = 0;
	     i < sizeof (cdc_comm_class_descriptions) / sizeof (struct usb_class_description);
	     i++) {
		struct usb_class_description *class_description = cdc_comm_class_descriptions + i;
		if (class_description->bDescriptorSubtype == USB_ST_ENF) {
			class_description->description.ethernet_networking.iMACAddress =
			    remote_mac_address_buffer;
			break;
		}
	}
#endif
#if defined(CONFIG_USBD_NET_MDLM)
	// set the mac address in the descriptor
	for (i = 0;
	     i < sizeof (mdlm_comm_class_descriptions) / sizeof (struct usb_class_description);
	     i++) {
		struct usb_class_description *class_description = mdlm_comm_class_descriptions + i;
		if (class_description->bDescriptorSubtype == USB_ST_ENF) {
			class_description->description.ethernet_networking.iMACAddress =
			    remote_mac_address_buffer;
			break;
		}
	}
#endif
#endif				/* defined(CONFIG_USBD_NET_CDC) || defined(CONFIG_USBD_NET_MDLM) */

#if defined(CONFIG_USBD_MAC_AS_SERIAL_NUMBER)
	if (bus->serial_number_str) {
		kfree (bus->serial_number_str);
		bus->serial_number_str = NULL;
	}
	bus->serial_number_str = strdup (remote_mac_address_buffer);
	//net_device_description.iSerialNumber = remote_mac_address_buffer;
	dbg_init (1, "setting iSerialNumber to remote mac_address: %s", remote_mac_address_buffer);
#endif
        default_dev_addr[ETH_ALEN - 1] = 1;
}

static void net_function_exit (struct usb_device_instance *device)
{
	dbg_init (1, "CLOSING **************************");
	net_destroy (device);
}

/* Called to handle USB Events  ************************************************************** */

/**
 * net_event - process a device event
 * @device: usb device 
 * @event: the event that happened
 *
 * Called by the usb device core layer to respond to various USB events.
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
void net_event (struct usb_device_instance *device, usb_device_event_t event, int data)
{
	int port = 0;
	struct usb_function_instance *function;
	struct usb_net_private *net_private;

	if (!(function = device->function_instance_array + port)) {
		dbg_usbe (1, "function NULL");
		return;
	}

        dbg_usbe (1,"%s data: %d privdata: %p", USBD_DEVICE_EVENTS(event), data, function->privdata);

	dbg_usbe (7, "%p %s %d previous_state: %d device_state: %d",
		  device, device->name, event, device->device_previous_state, device->device_state);


	switch (event) {

	case DEVICE_CREATE:	// a bus interface driver has created a usb device
		if (alwaysup) {
			net_create (device, function);
		}
		break;

	case DEVICE_RESET:
		net_device_check_condition(DEVICE_RESET);
		if (!alwaysup) {
			net_destroy (device);
		}
		break;

	case DEVICE_CONFIGURED:	// the host has found a driver that matches
		net_device_check_condition(DEVICE_CONFIGURED);

#if defined(CONFIG_USBD_NET_CDC)
		break;
#else
                device->interface = 0;
                device->alternate = 0;
#endif

	case DEVICE_SET_INTERFACE:	// the host driver has selected an interface
		if (!alwaysup) {
			net_create (device, function);
		}
		break;

	case DEVICE_BUS_INACTIVE:	// suspend
		net_device_check_condition(DEVICE_BUS_INACTIVE);
                // XXX
                if ((net_private = function->privdata)) {
                        //netproto_off (net_private->interface);
                }
		break;

	case DEVICE_BUS_ACTIVITY:	// resume
		net_device_check_condition(DEVICE_BUS_ACTIVITY);
                // XXX
                if ((net_private = function->privdata)) {
                        //netproto_on (net_private->interface);
                }
		break;

	case DEVICE_DESTROY:	// the bus interface driver is unloading
		net_destroy (device);
		break;

	case DEVICE_FUNCTION_PRIVATE:
		switch (data) {
		case NET_SET_CRC:
			break;
		}
		break;

        default:
		break;
	}
}

/**
 * net_recv_setup - called with a control URB 
 * @urb - pointer to struct urb
 *
 * Check if this is a setup packet, process the device request, put results
 * back into the urb and return zero or non-zero to indicate success (DATA)
 * or failure (STALL).
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
int net_recv_setup (struct urb *urb)
{
	struct usb_device_request *request;
	struct usb_device_instance *device = urb->device;
	int port = 0;
	struct usb_net_private *net_private = (device->function_instance_array + port)->privdata;

	request = &urb->device_request;

	// handle USB Standard Request (c.f. USB Spec table 9-2)
	if ((request->bmRequestType & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_VENDOR) {
		dbg_ep0 (1, "not vendor request: %x %x", request->bmRequestType,
			 request->bmRequestType & USB_REQ_TYPE_MASK);
		return 0;	// XXX
	}
	// handle all requests that return data (direction bit set on bm RequestType)
	if ((request->bmRequestType & USB_REQ_DIRECTION_MASK)) {
		dbg_ep0 (1, "Device-to-Host");
	} else {
		dbg_ep0 (1, "Host-to-Device");
		switch (request->bRequest) {
		case MCCI_ENABLE_CRC:
			dbg_ep0 (1, "Enabling CRC");
			net_private->crc = 1;
			usbd_device_event_irq (device, DEVICE_FUNCTION_PRIVATE, NET_SET_CRC);
			break;
		}
	}

	return 0;
}

/**
 * net_recv_urb - called with a received URB 
 * @urb - pointer to struct urb
 *
 * Return non-zero if we failed and urb is still valid (not disposed)
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
int net_recv_urb (struct urb *urb)
{
	int port = 0;		// XXX compound device
	struct usb_device_instance *device;
	struct usb_function_instance *function;
	struct usb_net_private *net_private;
	unsigned int length;

	struct sk_buff *skb = NULL;
	int mtu;

	if (!urb || !(device = urb->device) ||
	    !(function = device->function_instance_array + port) ||
	    !(net_private = function->privdata)) {
		return -EINVAL;
	}

	if (!(net_private->flags & NET_INUSE)) {
		netproto_rx_missed_error (net_private->interface);
		return -EINVAL;
	}


	dbg_rx (5, "urb: %p len: %d maxtransfer: %d\n", urb, urb->actual_length,
		net_private->maxtransfer);

	if (urb->status != RECV_OK) {
		dbg_rx (0, "!RECV_OK urb->actual_length: %d maxtransfer: %d", urb->actual_length,
			net_private->maxtransfer);
		netproto_rx_fifo_error (net_private->interface);
		return -EINVAL;
	}

	length = urb->actual_length;

	if (length > net_private->maxtransfer) {
		dbg_rx (0, "TOO LARGE urb->actual_length: %d maxtransfer: %d", urb->actual_length,
			net_private->maxtransfer);
		netproto_rx_over_error (net_private->interface);
		return -EINVAL;
	}

	if (!(skb = net_dev_alloc_skb (length))) {
		dbg_rx (0, "cannot malloc skb");
		netproto_rx_missed_error (net_private->interface);
		return -EINVAL;
	}

	dbgPRINTmem (dbgflg_usbdfd_rx, 4, urb->buffer, length);

	if (net_private->crc) {
		__u32 fcs;

		// do we need to check for extra null byte
		if ((length % out_pkt_sz) == 1) {

			// fcs and copy length minus one byte first, if CRC32 is ok, then assume that we have an extra byte
			if (((fcs =
			      fcs_memcpy32 (skb_put (skb, length - 1), urb->buffer, length - 1,
					    CRC32_INITFCS)) != CRC32_GOODFCS)) {

				// CRC32 check failed, fcs and copy last byte, if ok then continue otherwise fail
				if ((fcs_memcpy32
				     (skb_put (skb, 1), urb->buffer + length - 1, 1,
				      fcs) != CRC32_GOODFCS)) {
					dbg_rx (1, "CRC32 failed on extra byte len: %d fcs: %08x",
						length, fcs);
					dbgPRINTmem (dbgflg_usbdfd_rx, 1, urb->buffer, length);
					dev_kfree_skb_any (skb);
					netproto_rx_crc_error (net_private->interface);
					return -EINVAL;
				}
			}
		} else {
			if (((fcs =
			      fcs_memcpy32 (skb_put (skb, length), urb->buffer, length,
					    CRC32_INITFCS)) != CRC32_GOODFCS)) {
				dbg_rx (1, "CRC32 failed len: %d fcs: %08x", length, fcs);
				dbgPRINTmem (dbgflg_usbdfd_rx, 4, urb->buffer, length);
				dev_kfree_skb_any (skb);
				netproto_rx_crc_error (net_private->interface);
				return -EINVAL;
			}
		}
		skb_trim (skb, skb->len - 4);
	}

	else {
		memcpy (skb_put (skb, length), urb->buffer, length);
	}

	if ((skb->len > net_private->maxtransfer) || (skb->tail > skb->end)) {
		dbg_rx (0, "ERROR skb: %p head: %p data: %p tail: %p end: %p len: %d",
			skb, skb->head, skb->data, skb->tail, skb->end, skb->len);
		dev_kfree_skb_any (skb);
		netproto_rx_length_error (net_private->interface);
		return -EINVAL;
	}
	// XXX should not be needed
	mtu = netproto_mtu (net_private->interface);
	if (skb->len > (mtu + 14)) {
		skb_trim (skb, (netproto_mtu (net_private->interface) + 14));
	}

	usbd_recycle_urb (urb);

#ifdef RXECHO
	dbg_rx (3, "skb: %p head: %p data: %p tail: %p end: %p len: %d",
		skb, skb->head, skb->data, skb->tail, skb->end, skb->len);
	dbgPRINTmem (dbgflg_usbdfd_rx, 3, skb->data, skb->len);
#endif


	// pass it up, free skb if non zero
	if (netproto_recv (net_private->interface, skb)) {
		dev_kfree_skb_any (skb);
		//netproto_rx_dropped(net_private->interface);
	}
	return 0;
}


/**
 * net_urb_sent - called to indicate URB transmit finished
 * @urb: pointer to struct urb
 * @rc: result
 *
 * The usb device core layer will use this to let us know when an URB has
 * been finished with.
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
int net_urb_sent (struct urb *urb, int rc)
{
	int port = 0;		// XXX compound device
	struct usb_device_instance *device;
	struct usb_function_instance *function;
	struct usb_net_private *net_private;
	int interface;
	struct sk_buff *skb;

	if (!urb || !(device = urb->device) ||
	    !(function = device->function_instance_array + port) ||
	    !(net_private = function->privdata)) {
		return -EINVAL;
	}
	if (!(net_private->flags & NET_INUSE)) {
		// The interface is being/has been shutdown.
		// XXX Check this logic for mem leaks, but the
		// shutdown should have taken care of it, so
		// a leak is not likely.
		return 0;
	}
	interface = net_private->interface;

	// retrieve skb pointer and unlink from urb pointers
	skb = (struct sk_buff *) urb->privdata;

	// if no CRC then we are pointing at skb->buffer, so don't attempt to free urb->buffer
	if (!net_private->crc) {
		urb->buffer = NULL;
		urb->actual_length = 0;
	}
	urb->privdata = NULL;
	usbd_dealloc_urb (urb);

	// tell netproto we are done with the skb, it will test for NULL
	netproto_done (interface, skb, rc != SEND_FINISHED_OK);
	return 0;
}


/* *
 * net_xmit_skb - called to transmit an skb
 * @interface: which interface
 * @skb: skb to send
 *
 */
static int net_xmit_skb (int interface, struct sk_buff *skb)
{
	int port = 0;		// XXX compound device
	struct usb_net_private *net_private;
	struct urb *urb;

	//dbg_tx(5,"skb: %p", skb);

	// XXX XXX
	// return -EINVAL;

	net_private = &net_private_array[interface];

	if (!net_private->device) {
		dbg_tx (0, "net_private NULL");
		return -EUNATCH;
	}
#if 0
#ifdef CONFIG_ARCH_SA1100
	// compensate for SA-1110 DMA setup problems by sending three dummy urbs before
	// any real traffic, these will be dropped at the far end
	if (net_private->first) {
		int i;
		dbg_tx (1, "sending three init URBS");
		for (i = 0; i < 3; i++) {
			if ((urb =
			     usbd_alloc_urb (net_private->device,
					     (net_private->device->function_instance_array + port),
					     CONFIG_USBD_NET_IN_ENDPOINT | IN, 200))) {
				memset (urb->buffer, i, 200);
				urb->actual_length = 100;
				usbd_send_urb (urb);
			}
		}
		net_private->first = 0;
	}
#endif
#endif

	if (!
	    (urb =
	     usbd_alloc_urb (net_private->device,
			     (net_private->device->function_instance_array + port),
			     CONFIG_USBD_NET_IN_ENDPOINT | IN, skb->len + 5 + in_pkt_sz))) {
		dbg_tx (0, "urb alloc failed len: %d", skb->len);
		return -ENOMEM;
	}

	if (net_private->crc) {
		u32 send_fcs;
		send_fcs = fcs_memcpy32 (urb->buffer, skb->data, skb->len, CRC32_INITFCS);
		urb->actual_length = skb->len;

		// check if we need to pre-pend a NULL byte before CRC
		if ((urb->actual_length % in_pkt_sz) == (in_pkt_sz - 4)) {
			send_fcs = fcs_pad32 (urb->buffer + urb->actual_length, 1, send_fcs);
			urb->actual_length++;
			dbg_tx (2, "do_fcs32: adding NULL byte before CRC");
		}
		// if transfer is less than packetsize guarantee that is at lest two packets
		else if (skb->len < (in_pkt_sz - 3)) {
			dbg_tx (2, "do_fcs32: len: %d padding; %d", skb->len,
				(in_pkt_sz - skb->len - 3));
			send_fcs =
			    fcs_pad32 (urb->buffer + urb->actual_length, (in_pkt_sz - skb->len - 3),
				       send_fcs);
			urb->actual_length = in_pkt_sz - 3;
		}
#if defined(CONFIG_USBD_NET_PADDED)
//#warning LINKUP Padding defined
		// Linkup - minimum size of last packet
		dbg_tx (2, "do_fcs32: checking actual: %d pktsize: %d remainder: %d",
			urb->actual_length, in_pkt_sz, (urb->actual_length + 4) % in_pkt_sz);

		while (((urb->actual_length + 4) % in_pkt_sz) <= (in_pkt_sz - 2)) {
			send_fcs = fcs_pad32 (urb->buffer + urb->actual_length, 1, send_fcs);
			urb->actual_length++;
			//dbg_tx(3, "do_fcs32: adding NULL byte(s) before CRC");
		}
#endif

		send_fcs = ~send_fcs;
		urb->buffer[urb->actual_length++] = send_fcs & 0xff;
		urb->buffer[urb->actual_length++] = (send_fcs >> 8) & 0xff;
		urb->buffer[urb->actual_length++] = (send_fcs >> 16) & 0xff;
		urb->buffer[urb->actual_length++] = (send_fcs >> 24) & 0xff;
	}

	else {
		urb->buffer = skb->data;
		urb->actual_length = skb->len;
	}

#ifdef CONFIG_USBD_NO_ZLP_SUPPORT
//#warning -
//#warning NO ZLP SUPPORT DEFINED
	// check if we need to append a NULL byte
	if (!(urb->actual_length % in_pkt_sz)) {
		dbg_tx (3, "adding NULL byte after CRC");
		urb->buffer[urb->actual_length++] = 0;
	}
#endif

	// save skb for netproto_done
	urb->privdata = (void *) skb;

	dbg_tx (4, "skb: %p head: %p data: %p tail: %p len: %d endpoint: %x",
		skb, skb->head, skb->data, skb->tail, skb->len, CONFIG_USBD_NET_IN_ENDPOINT);
	dbg_tx (4, "urb: %p buffer: %p acutal_length: %d", urb, urb->buffer, urb->actual_length);
	dbgPRINTmem (dbgflg_usbdfd_tx, 4, urb->buffer, urb->actual_length);

	if (usbd_send_urb (urb)) {
		dbg_tx (1, "usbd_send_urb failed");
		urb->privdata = NULL;
		usbd_dealloc_urb (urb);
		return -ECOMM;
	}
	return 0;
}

static int net_set_addr (int dev, void *addr, int len)
{
	return 0;
}

/** 
 * net_tx_timeout - called by netproto if transmit times out
 * @dev: which device
 *
 * Called to cancel an in progress transmit. Note that this routine is advisory,
 * it just sends a message to the lower layer which may or may not actually do
 * anything about it.
 */
static int net_tx_timeout (int interface)
{
	struct usb_net_private *net_private;

	dbg_tx (1, "if=%d", interface);

	net_private = &net_private_array[interface];

	if (!net_private->device) {
		return -EINVAL;
	}

	return usbd_cancel_urb (net_private->device, NULL);
}

struct usb_function_operations function_ops = {
	event:net_event,
	recv_urb:net_recv_urb,
	recv_setup:net_recv_setup,
	urb_sent:net_urb_sent,
#if 0
	alloc_urb_data:net_alloc_urb_data,
	dealloc_urb_data:net_dealloc_urb_data
#endif
	    function_init:net_function_init,
	function_exit:net_function_exit,
};

struct usb_function_driver function_driver = {
	name:"Generic Network",
	ops:&function_ops,
	device_description:&net_device_description,
	configurations:sizeof (net_description) / sizeof (struct usb_configuration_description),
	configuration_description:net_description,
	this_module:THIS_MODULE,
};



/* proc file system ********************************************************************** */
int net_device_condition = NET_DEVICE_CONDITION_UNKNOWN;
static int net_device_received_command = 0;
static int net_device_bus_active = 0;
static struct timer_list net_device_fail_check_timer;

static void net_device_fail_check(unsigned long data)
{
	if (net_device_condition == NET_DEVICE_CONDITION_RESET) {
		net_device_condition = NET_DEVICE_CONDITION_FAIL;
	}
}

static void net_device_check_condition(usb_device_event_t event)
{

	switch (event) {
	case DEVICE_RESET:
		del_timer(&net_device_fail_check_timer);

		net_device_condition = NET_DEVICE_CONDITION_RESET;
		net_device_received_command = 0;
		net_device_bus_active = 1;

		net_device_fail_check_timer.function = net_device_fail_check;
		net_device_fail_check_timer.expires = jiffies + 30 * 100;
		add_timer(&net_device_fail_check_timer);
		break;

	case DEVICE_CONFIGURED:
		net_device_received_command = 1;
		break;

	case DEVICE_BUS_INACTIVE:
		net_device_bus_active = 0;
		break;

	case DEVICE_BUS_ACTIVITY:
		net_device_bus_active = 1;
		break;

	default:
		break;
	}
}

/* *
 * net_device_proc_read - implement proc file system read.
 * @file: xx
 * @buf:  xx
 * @count:  xx
 * @pos:  xx
 *
 * Standard proc file system read function.
 *
 * We let upper layers iterate for us, *pos will indicate which device to return * statistics for.
 */
static ssize_t
net_device_proc_read_condition(struct file *file, char *buf, size_t count, loff_t * pos)
{
	int len = 0;
	char *p;

	if (*pos > 0)
		return 0;

	switch (net_device_condition) {
	case NET_DEVICE_CONDITION_UNKNOWN:
	default:
		p = "Unknown\n";
		break;
	case NET_DEVICE_CONDITION_RESET:
		p = "Enumeration Start\n";
		break;
	case NET_DEVICE_CONDITION_OK:
		if ( !net_device_bus_active ) {
			p = "Unknown -- bus inactive\n";
		} else {
			p = "Enumeration OK\n";
		}
		break;
	case NET_DEVICE_CONDITION_FAIL:
		if ( !net_device_bus_active ) {
			p = "Unknown -- bus inactive\n";
		} else
		if ( !net_device_received_command ) {
			p = "Unknown -- command wasn't received from host\n";
			// USB cable not connected
		} else {
			p = "Enumeration Fail\n";
		}
		break;
	}

	len = strlen(p);
	*pos += len;
	if (len > count) {
		len = -EINVAL;
	}
	else if (len > 0 && copy_to_user(buf, p, len)) {
		len = -EFAULT;
	}
	return len;
}

/* *
 * usbd_monitor_proc_write - implement proc file system write.
 * @file
 * @buf
 * @count
 * @pos
 *
 * Proc file system write function, used to signal monitor actions complete.
 * (Hotplug script (or whatever) writes to the file to signal the completion
 * of the script.)  An ugly hack.
 */
static ssize_t
net_device_proc_write_condition(struct file *file, const char *buf, size_t count, loff_t * pos)
{
	if (net_device_condition != NET_DEVICE_CONDITION_RESET)
		net_device_condition = NET_DEVICE_CONDITION_UNKNOWN;
	return count;
}

static struct file_operations net_device_proc_operations_condition = {
	read: net_device_proc_read_condition,
	write:net_device_proc_write_condition,
};



/* Module init and exit ********************************************************************** */

unsigned char hexdigit (char c)
{
	return isxdigit (c) ? (isdigit (c) ? (c - '0') : (c - 'A' + 10))
	    : 0;
}

/*
 * net_modinit - module init
 *
 */
static int __init net_modinit (void)
{
	char local_mac_address_buffer[13];
	int i;

	debug_option *op = find_debug_option (dbg_table, "net");

	printk (KERN_INFO "%s (dbg=\"%s\",alwaysup=%d,OUT=%d,IN=%d)\n",
		__usbd_module_info, dbg ? dbg : "", alwaysup, out_pkt_sz, in_pkt_sz);

	printk (KERN_INFO "vendorID: %x productID: %x\n", CONFIG_USBD_VENDORID,
		CONFIG_USBD_PRODUCTID);

	if (NULL != op) {
		op->sub_table = netproto_get_dbg_table ();
	}
	if (0 != scan_debug_options ("net_fd", dbg_table, dbg)) {
		return (-EINVAL);
	}

	memset (net_private_array, 0, sizeof (net_private_array));

	// verify pkt sizes not too small
	if (out_pkt_sz < 3 || in_pkt_sz < 3) {
		dbg_init (0, "Rx pkt size %d or Tx pkt size %d too small", out_pkt_sz, in_pkt_sz);
		return (-EINVAL);
	}
	// Check for non-default packet sizes
	if (out_pkt_sz != CONFIG_USBD_NET_OUT_PKTSIZE) {
#ifdef CONFIG_USBD_NET_SAFE
		net_default[0].wMaxPacketSize = out_pkt_sz;
#endif				/* CONFIG_USBD_NET_SAFE */
#ifdef CONFIG_USBD_NET_CDC
		net_alt_1_endpoints[0].wMaxPacketSize = out_pkt_sz;
#endif
	}

	if (in_pkt_sz != CONFIG_USBD_NET_IN_PKTSIZE) {
#ifdef CONFIG_USBD_NET_SAFE
		net_default[1].wMaxPacketSize = in_pkt_sz;
#endif				/* CONFIG_USBD_NET_SAFE */
#ifdef CONFIG_USBD_NET_CDC
		net_alt_1_endpoints[1].wMaxPacketSize = in_pkt_sz;
#endif
	}

	dbg_init (1, "Rx pkt size %d, Tx pkt size %d", out_pkt_sz, in_pkt_sz);

	if (vendor_id) {
		net_device_description.idVendor = vendor_id;
	}
	if (product_id) {
		net_device_description.idProduct = product_id;
	}
	// initialize the netproto library
	if (netproto_modinit (if_name, MAX_INTERFACES)) {
		dbg_init (0, "netproto_modinit failed");
		return -EINVAL;
	}

	if (local_mac_address && (strlen (local_mac_address) == 12)) {
		dbg_init (1, "using local_mac_address: %s", local_mac_address);
		memcpy (local_mac_address_buffer, local_mac_address, 12);
	} else {
		dbg_init (1, "using LOCAL_MACADDR: %s", CONFIG_USBD_NET_LOCAL_MACADDR);
		memcpy (local_mac_address_buffer, CONFIG_USBD_NET_LOCAL_MACADDR, 12);
	}

	local_mac_address_buffer[12] = '\0';

	dbg_init (1, "local_mac_address_buffer: %s", local_mac_address_buffer);
	dbg_init (1, "LOCAL_MACADDR: %s", CONFIG_USBD_NET_LOCAL_MACADDR);
	if (local_mac_address) {
		dbg_init (1, "localmac_address: %s len: %d", local_mac_address,
			  strlen (local_mac_address));
	}
	// save the mac address for the network device
	for (i = 0; i < ETH_ALEN; i++) {
		default_dev_addr[i] =
		    hexdigit (local_mac_address_buffer[i * 2]) << 4 |
		    hexdigit (local_mac_address_buffer[i * 2 + 1]);
		//dbg_init(0, "local_mac_address[%d]: %02x %02x %d %d %02x", i, 
		//        local_mac_address_buffer[i*2], local_mac_address_buffer[i*2+1], 
		//        hexdigit(local_mac_address_buffer[i*2]), hexdigit(local_mac_address_buffer[i*2+1]),
		//        default_dev_addr[i]);
	}

	// create network interface
	//net_create(0);

	// register us with the usb device support layer
	if (usbd_register_function (&function_driver)) {
		dbg_init (0, "usbd_register_function failed");
		netproto_modexit ();
		return -EINVAL;
	}

	{
		struct proc_dir_entry *p;
		if ((p = create_proc_entry("usb-condition", 0, 0)) == NULL) {
			netproto_modexit();
			return -ENOMEM;
		}
		p->proc_fops = &net_device_proc_operations_condition;

		init_timer(&net_device_fail_check_timer);
	}

	return 0;
}

/*
 * function_exit - module cleanup
 *
 */
static void __exit net_modexit (void)
{

	dbg_init (1, "exiting");

	// de-register us with the usb device support layer
	usbd_deregister_function (&function_driver);

	// destroy network interface
	//net_destroy(0);

	// tell the netproto library to exit
	netproto_modexit ();

	remove_proc_entry("usb-condition", NULL);
	del_timer(&net_device_fail_check_timer);

}

module_init (net_modinit);
module_exit (net_modexit);
