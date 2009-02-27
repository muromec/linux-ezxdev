/*
 * linux/drivers/usbd/pst_fd/pst-fd.c - Motorola PST USB function driver
 *
 * Copyright (C) 2003, 2005 - Motorola
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 *  2003-JUL-01 - Motorola PST USB Function Driver is Create by Levis. (Levis@motorola.com)
 *
 *  2005-JUL-28 - Add PST-BULK Driver By Liu Lin <linliu@motorola.com>
 *
 */


#include <linux/config.h>
#include <linux/module.h>

#include <usbd-export.h>
#include <usbd-build.h>

MODULE_AUTHOR ("Levis@Motorola.com");
MODULE_DESCRIPTION ("PST USB Function Driver");


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/skbuff.h>
#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/atmdev.h>
#include <linux/pkt_sched.h>
#include <asm/system.h>
#include <linux/delay.h>
#include <linux/sched.h>//LIN use to sleep_on_timeout
//#include <asm/arch-pxa/pxa-regs.h>
#include <linux/autoconf.h>

#include <usbd-chap9.h>
#include <usbd-mem.h>
#include <usbd.h>
#include <usbd-func.h>
#include <usbd-bus.h>

USBD_MODULE_INFO ("pst_usb 2.0-beta");

#include "pst-usb.h"
#include "ptfproto.h"

#define VERSION "0.90"
#define VERSION25 1
//LIN change code by the swith  move to pst-usb.h
// #define PST_BULKINTF 1 
// #define PST_DUOINTF 1


#define USBC_MINOR0 17
#define USBC_MINOR1 18
#define USBC_MINOR2 19


extern int motusbd_status;  //added by Jordan to Support CFG11 bplog
#define PST_DEBUG 0 
//#define PST_DUMP_DEBUG 1
//#define TSTCMD_DEBUG
#if PST_DEBUG
#define PRINTKD(fmt, args...) printk( fmt , ## args)
#else
#define PRINTKD(fmt, args...)
#endif

#ifndef CONFIG_USBD_MAXPOWER
#define CONFIG_USBD_MAXPOWER                    0
#endif

#ifndef CONFIG_USBD_MANUFACTURER
#define CONFIG_USBD_MANUFACTURER                "Lineo"
#endif

#undef BMAXPOWER
#define BMAXPOWER                               0x0a

/*
 * setup some default values for pktsizes and endpoint addresses.
 */
#if PST_BULKINTF 
#define TCMDBULKREADTIMEOUT			(HZ/16)
#define TEST_CMD_PKTSIZE_FS             	64
#define TEST_CMD_PKTSIZE_HS             	512
#else
#define TEST_CMD_PKTSIZE             	8
#endif

#define DATA_LOG_IN_PKTSIZE          	64
#define PTF_OUT_PKTSIZE               	64
#define PTF_IN_PKTSIZE           	64
#define AP_DATA_LOG_OUT_PKTSIZE   	64
#define AP_DATA_LOG_IN_PKTSIZE       	64
#if 0
/* XXX There is no way to get a specific endpoint, only an interface number.
       The host SW will extract the endpoint number from the descriptors. */
#define TEST_CMD_ENDPOINT            	0
#define DATA_LOG_IN_ENDPOINT       	11
#define PTF_OUT_ENDPOINT       		2//7
#define PTF_IN_ENDPOINT          	1//6
#define AP_DATA_LOG_OUT_ENDPOINT 	7//12
#define AP_DTAT_LOG_IN_ENDPOINT    	6//11
#endif

/* Module Parameters ***/
interface_t intf6,intf8;
static int usbi0_ref_count=0, usbi1_ref_count=0, usbi2_ref_count=0;
#if PST_DUOINTF
#define PST_MAX_BUF_CNT 16384
int intf8_read_cnt;

#define TCMD_BULKPORT 1
#define TCMD_CONTROLPORT 2
int g_tcmd_bulkorcontrol;
#endif
/* All of the globals like polling_times6, including the wait queues, should really
   be _inside_ the interface_t structure.  This would make it possible to pass
   around one pointer and avoid all the checking to see if this is 6 or 8.
   The code would be both faster and smaller. */
static int polling_times6=0,polling_times8=0;
static int specific_times6=0,specific_times8=0;
static int polling_ready6, polling_ready8;
static int polling_len, bplog = 0; 
static struct urb *urb_polling;
static struct usb_function_instance * pst_function;
int cable_connected = 0;

static void send_ep0_urb_to_host( void * data, int len );

#if PST_BULKINTF
//LIN static void send_epx_urb_to_host( void * data, int len,int epidx );
static int pxa_usb_send(char *buf, int len, __u8 ENDPOINT);
#endif

void polling_message( interface_t *interface, int polling_data_len );
void polling_message_return( interface_t *interface, int polling_data_len );

#ifdef CONFIG_ARCH_EZX  //w20535 support cfg11
int pst_function_enable( struct usb_function_instance* function);
void pst_function_disable( struct usb_function_instance *function);
#endif
//a17400:add ptf

DECLARE_WAIT_QUEUE_HEAD(wq_read6);
DECLARE_WAIT_QUEUE_HEAD(wq_read8);
DECLARE_WAIT_QUEUE_HEAD(wq_write6d);

struct usb_ptf_private {
        int device_index;	//it's device index in device array.
        int out_ep_addr;		/* hold OUT endpoint address for recv, IN ep address will be desinated in xxx_xmit_data */
        struct usb_function_instance *function;
};

static struct usb_ptf_private *ptf_private_array[PTF_MAX_DEV_NUMBER];	//yes, keep this number same as device number

static int ptf_ndx = -1,
           ap_log_ndx = -1;



/* USB Configuration Description by Tom************************************************************* */
/* USB Interface and Endpoint Configuration ********************** */

// Endpoint indexes in pst_endpoint_requests[] and the endpoint map.
//save space for complicance with config11
//#define BULK_OUT        0x00
//#define BULK_IN         0x01
//#define INT_IN          0x02

#define NDX_OF_DATALOG_IN_EP      0x03
#define NDX_OF_AP_LOG_IN_EP       0x04
#define NDX_OF_AP_LOG_OUT_EP      0x05
#define NDX_OF_PTF_IN_EP          0x06
#define NDX_OF_PTF_OUT_EP         0x07
#if PST_BULKINTF
#define NDX_OF_TESTCMD_IN_EP          0x08
#define NDX_OF_TESTCMD_OUT_EP         0x09
#define ENDPOINTS                 0x0A
#else
#define ENDPOINTS                 0x08
#endif



static __u8 datalog_endpoint[] = {
        0x07,                              // bLength
        USB_DT_ENDPOINT,                   // bDescriptorType   // 0x5
        /*DATA_LOG_IN_ENDPOINT | IN*/0x00, // bEndpointAddress - filled in later
        BULK,                              // bmAttributes
        0x00, 0x00,                        // wMaxPacketSize - filled in later
        0x00,                              // bInterval
};
static  struct usb_endpoint_descriptor  *datalog_endpoints[] = { (struct usb_endpoint_descriptor *) &datalog_endpoint };
u8 datalog_indexes[] = { NDX_OF_DATALOG_IN_EP, };

/*a17400: add ptf and AP logger endpoints according to new excel*/

static __u8 ap_log_endpoint_in[] = {
        /*endpoint in*/
        0x07,                                 // bLength
        USB_DT_ENDPOINT,                      // bDescriptorType   // 0x5
        /*AP_DTAT_LOG_IN_ENDPOINT | IN*/0x00, // bEndpointAddress - filled in later
        BULK,                                 // bmAttributes
        0x00, 0x00,                           // wMaxPacketSize - filled in later
        0x00,                                 // bInterval
};
static __u8 ap_log_endpoint_out[] = {
        /*endpoint out*/
        0x07,                                   // bLength
        USB_DT_ENDPOINT,                        // bDescriptorType   // 0x5
        /*AP_DATA_LOG_OUT_ENDPOINT | OUT*/0x00, // bEndpointAddress - filled in later
        BULK,                                   // bmAttributes
        0x00, 0x00,                             // wMaxPacketSize - filled in later
        0x00,                                   // bInterval
};
static struct usb_endpoint_descriptor *ap_log_endpoints[] = { 
        (struct usb_endpoint_descriptor *) &ap_log_endpoint_in,
        (struct usb_endpoint_descriptor *) &ap_log_endpoint_out };
u8 ap_log_indexes[] = { NDX_OF_AP_LOG_IN_EP, NDX_OF_AP_LOG_OUT_EP, };

static __u8 ptf_endpoint_in[] = {
        /*endpoint in*/
        0x07,                         // bLength
        USB_DT_ENDPOINT,              // bDescriptorType   // 0x5
        /*PTF_IN_ENDPOINT | IN*/0x00, // bEndpointAddress - filled in later
        BULK,                         // bmAttributes
        0x00, 0x00,                   // wMaxPacketSize - filled in later
        0x00,                         // bInterval
};

static __u8 ptf_endpoint_out[] = {                   
        /*endpoint out*/
        0x07,                           // bLength
        USB_DT_ENDPOINT,                // bDescriptorType   // 0x5
        /*PTF_OUT_ENDPOINT | OUT*/0x00, // bEndpointAddress - filled in later
        BULK,                           // bmAttributes
        0x00, 0x00,                     // wMaxPacketSize - filled in later
        0x00,                           // bInterval
};

static struct usb_endpoint_descriptor *ptf_endpoints[] = {
        (struct usb_endpoint_descriptor *) &ptf_endpoint_in,
        (struct usb_endpoint_descriptor *) &ptf_endpoint_out };
u8 ptf_indexes[] = { NDX_OF_PTF_IN_EP, NDX_OF_PTF_OUT_EP, };

#if PST_BULKINTF
//LIN endpoint for testcmd
static __u8 testcmd_endpoint_in[] = {
        /*endpoint in*/
        0x07,                                 // bLength
        USB_DT_ENDPOINT,                      // bDescriptorType   // 0x5
        0x00, // bEndpointAddress - filled in later
        BULK,                                 // bmAttributes
        0x00, 0x00,                           // wMaxPacketSize - filled in later
        0x00,                                 // bInterval
};
static __u8 testcmd_endpoint_out[] = {
        /*endpoint out*/
        0x07,                                   // bLength
        USB_DT_ENDPOINT,                        // bDescriptorType   // 0x5
        0x00, // bEndpointAddress - filled in later
        BULK,                                   // bmAttributes
        0x00, 0x00,                             // wMaxPacketSize - filled in later
        0x00,                                   // bInterval
};
static struct usb_endpoint_descriptor *testcmd_endpoints[] = { 
        (struct usb_endpoint_descriptor *) &testcmd_endpoint_in,
        (struct usb_endpoint_descriptor *) &testcmd_endpoint_out };
u8 testcmd_indexes[] = { NDX_OF_TESTCMD_IN_EP, NDX_OF_TESTCMD_OUT_EP, };
#endif

/*
 * Data Interface Alternate descriptor and description (one endpoint)
 */

static struct usb_alternate_description dummy_alternate_descriptions[] = {
        { iInterface:"Dummy", },
};

#define PST_DATA_CLASS         0xff
#define PST_DATA_SUBCLASS      0x02
#define PST_DATA_PROTOCOL      0xff

// w20146 add, patch from Tom in 20040223
static __u8 pseudo_alternate_descriptor0[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x00,              // bInterfaceNumber
	0x00,              // bAlternateSetting
	0x00,              // bNumEndpoints
	PST_DATA_CLASS,    // bInterfaceClass
	PST_DATA_SUBCLASS, // bInterfaceSubClass
	PST_DATA_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};

static struct usb_alternate_description pseudo_alternate_descriptions0[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor0,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor1[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x01,              // bInterfaceNumber
	0x00,              // bAlternateSetting
	0x00,              // bNumEndpoints
	PST_DATA_CLASS,    // bInterfaceClass
	PST_DATA_SUBCLASS, // bInterfaceSubClass
	PST_DATA_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};

static struct usb_alternate_description pseudo_alternate_descriptions1[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor1,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor2[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x02,              // bInterfaceNumber
	0x00,              // bAlternateSetting
	0x00,              // bNumEndpoints
	PST_DATA_CLASS,    // bInterfaceClass
	PST_DATA_SUBCLASS, // bInterfaceSubClass
	PST_DATA_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};

struct usb_alternate_description pseudo_alternate_descriptions2[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor2,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor3[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x03,              // bInterfaceNumber
	0x00,              // bAlternateSetting
	0x00,              // bNumEndpoints
	PST_DATA_CLASS,    // bInterfaceClass
	PST_DATA_SUBCLASS, // bInterfaceSubClass
	PST_DATA_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};

struct usb_alternate_description pseudo_alternate_descriptions3[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor3,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor4[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x04,              // bInterfaceNumber
	0x00,              // bAlternateSetting
	0x00,              // bNumEndpoints
	PST_DATA_CLASS,    // bInterfaceClass
	PST_DATA_SUBCLASS, // bInterfaceSubClass
	PST_DATA_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};

struct usb_alternate_description pseudo_alternate_descriptions4[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor4,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor5[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x05,              // bInterfaceNumber
	0x00,              // bAlternateSetting
	0x00,              // bNumEndpoints
	PST_DATA_CLASS,    // bInterfaceClass
	PST_DATA_SUBCLASS, // bInterfaceSubClass
	PST_DATA_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};

struct usb_alternate_description pseudo_alternate_descriptions5[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor5,
		endpoints:0,
	},
};

// -- w20146 -end


static __u8 datalog_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09,              // bLength
        USB_DT_INTERFACE,  // bDescriptorType   // 0x04
        0x06,              // bInterfaceNumber
        0x00,              // bAlternateSetting
        0x00,              // bNumEndpoints
        PST_DATA_CLASS,    // bInterfaceClass
        PST_DATA_SUBCLASS, // bInterfaceSubClass
        PST_DATA_PROTOCOL, // bInterfaceProtocol
        0x00,              // iInterface
};

struct usb_alternate_description datalog_alternate_descriptions[] = {
        {
	iInterface:"Motorola MCU Data Logger",
	interface_descriptor: (struct usb_interface_descriptor *)&datalog_alternate_descriptor,
	endpoints:sizeof (datalog_endpoints) / sizeof (struct usb_endpoint_descriptor *),
	endpoint_list:datalog_endpoints,
	endpoint_indexes:datalog_indexes,
        },
};


// w20146 add - 20040223
static __u8 pseudo_alternate_descriptor7[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x07,              // bInterfaceNumber
	0x00,              // bAlternateSetting
	0x00,              // bNumEndpoints
	PST_DATA_CLASS,    // bInterfaceClass
	PST_DATA_SUBCLASS, // bInterfaceSubClass
	PST_DATA_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};

struct usb_alternate_description pseudo_alternate_descriptions7[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor7,
		endpoints:0,
	},
};

/*
 * Comm Interface Alternate descriptor and description (no endpoints)
 */

#define PST_COMM_CLASS         0xff
#define PST_COMM_SUBCLASS      0x03
#define PST_COMM_PROTOCOL      0xff

static __u8 testcmd_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x08,              // bInterfaceNumber
	0x00,              // bAlternateSetting
#if PST_BULKINTF
  sizeof (testcmd_endpoints) / sizeof(struct usb_endpoint_descriptor *),// bNumEndpoints
#else
	0x00,              // bNumEndpoints
#endif
	PST_COMM_CLASS,    // bInterfaceClass
	PST_COMM_SUBCLASS, // bInterfaceSubClass
	PST_COMM_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};
#if PST_BULKINTF
struct usb_alternate_description testcmd_alternate_descriptions[] = {
        {
	iInterface:"Motorola Test Command",
        interface_descriptor: (struct usb_interface_descriptor *)&testcmd_alternate_descriptor,
	endpoints:sizeof (testcmd_endpoints) / sizeof (struct usb_endpoint_descriptor *),
	endpoint_list:testcmd_endpoints,
	endpoint_indexes:testcmd_indexes,
        },
};
#else
struct usb_alternate_description testcmd_alternate_descriptions[] = {
	{
iInterface:"Motorola Test Command ",
interface_descriptor: (struct usb_interface_descriptor *)&testcmd_alternate_descriptor,
	},
};
#endif

static __u8 pseudo_alternate_descriptor9[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x09,              // bInterfaceNumber
	0x00,              // bAlternateSetting
	0x00,              // bNumEndpoints
	PST_DATA_CLASS,    // bInterfaceClass
	PST_DATA_SUBCLASS, // bInterfaceSubClass
	PST_DATA_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};

struct usb_alternate_description pseudo_alternate_descriptions9[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor9,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptorA[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x0A,              // bInterfaceNumber
	0x00,              // bAlternateSetting
	0x00,              // bNumEndpoints
	PST_DATA_CLASS,    // bInterfaceClass
	PST_DATA_SUBCLASS, // bInterfaceSubClass
	PST_DATA_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};

struct usb_alternate_description pseudo_alternate_descriptionsA[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptorA,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptorB[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x0B,              // bInterfaceNumber
	0x00,              // bAlternateSetting
	0x00,              // bNumEndpoints
	PST_DATA_CLASS,    // bInterfaceClass
	PST_DATA_SUBCLASS, // bInterfaceSubClass
	PST_DATA_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};

struct usb_alternate_description pseudo_alternate_descriptionsB[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptorB,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptorC[sizeof(struct usb_interface_descriptor)] = {
	0x09,              // bLength
	USB_DT_INTERFACE,  // bDescriptorType   // 0x04
	0x0C,              // bInterfaceNumber
	0x00,              // bAlternateSetting
	0x00,              // bNumEndpoints
	PST_DATA_CLASS,    // bInterfaceClass
	PST_DATA_SUBCLASS, // bInterfaceSubClass
	PST_DATA_PROTOCOL, // bInterfaceProtocol
	0x00,              // iInterface
};

struct usb_alternate_description pseudo_alternate_descriptionsC[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptorC,
		endpoints:0,
	},
};

// -- w20146 -end



/*a17400 add ptf and ap logger alternate description(or)s*/

static __u8 ap_log_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09,              // bLength
        USB_DT_INTERFACE,  // bDescriptorType   // 0x04
        0x0d,              // bInterfaceNumber
        0x00,              // bAlternateSetting
        sizeof (ap_log_endpoints) / sizeof(struct usb_endpoint_descriptor *),
        // bNumEndpoints
        0xff,    // bInterfaceClass
        0x35, // bInterfaceSubClass
        0xff, // bInterfaceProtocol
        0x0c,              // iInterface
};


struct usb_alternate_description ap_log_alternate_descriptions[] = {
        {
	iInterface:"Motorola AP Data Logger",
	interface_descriptor: (struct usb_interface_descriptor *)&ap_log_alternate_descriptor,
	endpoints:sizeof (ap_log_endpoints) / sizeof (struct usb_endpoint_descriptor *),
	endpoint_list:ap_log_endpoints,
	endpoint_indexes:ap_log_indexes,
        },
};


static __u8 ptf_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09,              // bLength
        USB_DT_INTERFACE,  // bDescriptorType   // 0x04
        0x0e,              // bInterfaceNumber
        0x00,              // bAlternateSetting
        sizeof (ptf_endpoints) / sizeof(struct usb_endpoint_descriptor *),
        // bNumEndpoints
        0xff,    // bInterfaceClass
        0x36, // bInterfaceSubClass
        0xff, // bInterfaceProtocol
        0x0c,              // iInterface
};


struct usb_alternate_description ptf_alternate_descriptions[] = {
        {
	iInterface:"Motorola EzX PTF",
	interface_descriptor: (struct usb_interface_descriptor *)&ptf_alternate_descriptor,
	endpoints:sizeof (ptf_endpoints) / sizeof (struct usb_endpoint_descriptor *),
	endpoint_list:ptf_endpoints,
	endpoint_indexes:ptf_indexes,
        },
};


#if 0		// w20146 - 040223
/*
 * Comm Interface Alternate descriptor and description (no endpoints)
 */

#define PST_COMM_CLASS         0xff
#define PST_COMM_SUBCLASS      0x03
#define PST_COMM_PROTOCOL      0xff

static __u8 testcmd_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09,              // bLength
        USB_DT_INTERFACE,  // bDescriptorType   // 0x04
        0x08,              // bInterfaceNumber
        0x00,              // bAlternateSetting
        0x00,              // bNumEndpoints
        PST_COMM_CLASS,    // bInterfaceClass
        PST_COMM_SUBCLASS, // bInterfaceSubClass
        PST_COMM_PROTOCOL, // bInterfaceProtocol
        0x00,              // iInterface
};

static struct usb_alternate_description testcmd_alternate_descriptions[] = {
        {
	iInterface:"Motorola Test Command",
	interface_descriptor: (struct usb_interface_descriptor *)&testcmd_alternate_descriptor,
        },              
};
#endif		// w20146

/* Interface Description(s) */

static struct usb_interface_description testcmd_datalog_interfaces[] = {
#if 0	// w20146 - 040223
	{ alternate_list: dummy_alternate_descriptions, }, /* 0x0 */
        { alternate_list: dummy_alternate_descriptions, }, /* 0x1 */
        { alternate_list: dummy_alternate_descriptions, }, /* 0x2 */
        { alternate_list: dummy_alternate_descriptions, }, /* 0x3 */
        { alternate_list: dummy_alternate_descriptions, }, /* 0x4 */
        { alternate_list: dummy_alternate_descriptions, }, /* 0x5 */
#else
	{ alternates:1, alternate_list: pseudo_alternate_descriptions0, }, /* 0x0 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions1, }, /* 0x1 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions2, }, /* 0x2 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions3, }, /* 0x3 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions4, }, /* 0x4 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions5, }, /* 0x5 */
#endif
        {                                                  // 0x6
	alternates:sizeof(datalog_alternate_descriptions) / sizeof(struct usb_alternate_description),
	alternate_list:datalog_alternate_descriptions,
        },
#if 0		// w20146 - 040223
	{ alternate_list: dummy_alternate_descriptions, }, /* 0x7 */
#else
	{ alternates:1, alternate_list: pseudo_alternate_descriptions7, }, /* 0x7 */
#endif
        {                                                  // 0x8
	alternates:sizeof(testcmd_alternate_descriptions) / sizeof(struct usb_alternate_description),
	alternate_list:testcmd_alternate_descriptions,
        },
#if 0	
        { alternate_list: dummy_alternate_descriptions, }, /* 0x9 */
        { alternate_list: dummy_alternate_descriptions, }, /* 0xa */
        { alternate_list: dummy_alternate_descriptions, }, /* 0xb */
        { alternate_list: dummy_alternate_descriptions, }, /* 0xc */
#else
	{ alternates:1, alternate_list: pseudo_alternate_descriptions9, }, /* 0x9 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptionsA, }, /* 0xa */
	{ alternates:1, alternate_list: pseudo_alternate_descriptionsB, }, /* 0xb */
	{ alternates:1, alternate_list: pseudo_alternate_descriptionsC, }, /* 0xc */
#endif
        {                                                  // 0xd
	alternates:sizeof(ap_log_alternate_descriptions) / sizeof(struct usb_alternate_description),
	alternate_list:ap_log_alternate_descriptions,
        },
        {                                                  // 0xe
	alternates:sizeof(ptf_alternate_descriptions) / sizeof(struct usb_alternate_description),
	alternate_list:ptf_alternate_descriptions,
        },
};

/***USB Interface and Endpoint Configuration**********************************************/

/***USB Configuration ***/

/* Configuration description(s) */
static __u8 pst_configuration_descriptor[sizeof(struct usb_configuration_descriptor)] = {
        0x09,          // bLength
        USB_DT_CONFIG, // bDescriptorType   // 0x2
        0x00, 0x00,    // wTotalLength
        sizeof (testcmd_datalog_interfaces) / sizeof (struct usb_interface_description),
        // bNumInterfaces
        0x01,          // bConfigurationValue
        0x00,          // iConfiguration
        BMATTRIBUTE,   // bmAttributes   ??????? 0xC0
        BMAXPOWER,     // bMaxPower

};

struct usb_configuration_description pst_description[] = {
        {
	iConfiguration:"Motorola PTF AP BP Configuration",
	configuration_descriptor: (struct usb_configuration_descriptor *)pst_configuration_descriptor,
	bNumInterfaces:sizeof(testcmd_datalog_interfaces) / sizeof(struct usb_interface_description),
	interface_list:testcmd_datalog_interfaces,
        },
};
/********************************************************************/

/* Device Description */
/***PST USB Device Descriptor***/
#if 0
static __u8 pst_device_descriptor[sizeof(struct usb_device_descriptor)] = {
        0x12,          // bLength
        USB_DT_DEVICE, // bDescriptorType
        0x00, 0x00,    // bcdUSB,              set in usbd_function_init()
        0x00,          // bDeviceClass         ?????
        0x00,          // bDeviceSubClass      ?????
        0x00,          // bDeviceProtocol      ?????
        0x00,          // bMaxPacketSize0,     set in usbd_function_init()
        0x00, 0x00,    // idVendor,            filled from below in usbd_function_init()
        0x00, 0x00,    // idProduct,           filled from below in usbd_function_init()
        0x00, 0x00,    // bcdDevice,           filled from below in usbd_function_init()
        0x01,          // iManufacturer,       filled from below in usbd_function_init()
        0x02,          // iProduct,            filled from below in usbd_function_init()
        0x00,          // iSerialNumber,       filled from below in usbd_function_init()
        0x01,          // bNumConfigurations
};
#endif

static struct usb_device_descriptor pst_device_descriptor = {
	bLength: sizeof(struct usb_device_descriptor),
	bDescriptorType: USB_DT_DEVICE,
	bcdUSB: __constant_cpu_to_le16(USB_BCD_VERSION),
	bDeviceClass: 0x00,
	bDeviceSubClass: 0x00,
	bDeviceProtocol: 0x00,
	bMaxPacketSize0: 0x00,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_PST_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_PST_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_PST_BCDDEVICE),
};

static struct usb_endpoint_request pst_endpoint_requests[ENDPOINTS+1] = {
	 //we reserved the first three for compliance with config11
        { 1, 1, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, 64, 512, },
        { 1, 1, 0, USB_DIR_IN | USB_ENDPOINT_BULK, 64 , 512, },
        { 1, 0, 0, USB_DIR_IN | USB_ENDPOINT_INTERRUPT, 16, 64, },
	 //end here
        { 1, 1, 0, USB_DIR_IN | USB_ENDPOINT_BULK, DATA_LOG_IN_PKTSIZE, DATA_LOG_IN_PKTSIZE, },  // DATALOG_IN
        { 1, 1, 0, USB_DIR_IN | USB_ENDPOINT_BULK, AP_DATA_LOG_IN_PKTSIZE, AP_DATA_LOG_IN_PKTSIZE,  },  // AP_LOG_IN
        { 1, 1, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, AP_DATA_LOG_OUT_PKTSIZE, AP_DATA_LOG_OUT_PKTSIZE, }, // AP_LOG_OUT
        { 1, 1, 0, USB_DIR_IN | USB_ENDPOINT_BULK, PTF_IN_PKTSIZE, PTF_IN_PKTSIZE, },  // PTF_IN
        { 1, 1, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, PTF_OUT_PKTSIZE, PTF_OUT_PKTSIZE, }, // PTF_OUT
#if PST_BULKINTF
        { 1, 1, 0, USB_DIR_IN | USB_ENDPOINT_BULK, TEST_CMD_PKTSIZE_FS, TEST_CMD_PKTSIZE_HS, },  // TESTCMD_IN
        { 1, 1, 0, USB_DIR_OUT | USB_ENDPOINT_BULK,TEST_CMD_PKTSIZE_FS, TEST_CMD_PKTSIZE_HS, }, // TESTCMD_OUT
#endif        
        { 0, },
};



struct usb_device_description pst_device_description = {
	device_descriptor: &pst_device_descriptor,
	iManufacturer:"Motorola Inc.",
	iProduct:"Motorola Phone",
//	iSerialNumber:"0",
	endpointsRequested: ENDPOINTS,
	requestedEndpoints: pst_endpoint_requests,
};


int pst_urb_sent (struct urb *urb, int rc);
#if PST_BULKINTF
static int callback4bulktcmd( struct urb *urb, int rc );
static int normal_write2epx( const char *buffer, int stCount,int epidx);

#endif
/***USB Configuration End**************************************************/

/* *******************PTF Tool Functions******************************************************************* */

/*get_ptf_private (int device_index) --get a private instance from private array according it's device numer.
device_index:	the device num the returned private should include

return null, no such private

 */

//static __inline__ struct usb_ptf_private *get_ptf_private (int device_index)
static  struct usb_ptf_private *get_ptf_private (int device_index)
{
        int i;

        RETURN_NULL_IF (device_index < 0 || device_index >= PTF_MAX_DEV_NUMBER);
        
        //return ptf_private_array[interface]; we don'e follow net-fd and serial-fd, it is not clear.
        
        for (i = 0; i < PTF_MAX_DEV_NUMBER; i++)

                if(ptf_private_array[i])       //a17400: fixed, null pointer should be check

                        if(ptf_private_array[i]->device_index==device_index)

                                return ptf_private_array[i]; /* it is better... */
                
        return NULL;
}

#if 0
// TBR: no longer needed
/* get private data by look up its OUT endpoints */
static  struct usb_ptf_private *get_ptf_private_by_out_ep (int out_endpoint)
{
        int i=0;
        RETURN_NULL_IF (out_endpoint < 0 || out_endpoint >15);

        for (i = 0; i < PTF_MAX_DEV_NUMBER; i++)

                if(ptf_private_array[i])

                        if(ptf_private_array[i]->out_ep_addr==out_endpoint)

                                return ptf_private_array[i];
                
        return NULL;

}
#endif

//ptf_private_create():create a private and put it in private array
/* out_ep: out endpoint
   xmit_data: xmit_data callback
   tx_size: max tx_size
   return <0 error
   return index in ptf_private_array (>=0) success

 */
int ptf_private_create(struct usb_function_instance *function, int out_ep, int (*xmit_data) (int, unsigned char *, int), 
                int tx_size)
{
        struct usb_ptf_private * ptf_private;
        int device_index,i;

        for (i = 0; i < PTF_MAX_DEV_NUMBER; i++) {
                if (ptf_private_array[i] == NULL) {
                        break;
                }
        }

        if (i >= PTF_MAX_DEV_NUMBER) {
                printk (KERN_WARNING "ptf private array full\n");
                return -1;
        }

        // allocate private data
        if ((ptf_private =
                                kmalloc (sizeof (struct usb_ptf_private), GFP_ATOMIC)) == NULL) {//??? GFP_ATOMIC is ok?
                printk (KERN_WARNING "can't allocate mem for ptf private \n");
                return -1;
        }
        if (0>(device_index = ptfproto_create ("", xmit_data, tx_size) )){//???max tx size?
                kfree (ptf_private);
                printk (KERN_WARNING "can't apply new device in ptf device array\n");
                return-1;
        }
        // lock and modify device array
        ptf_private->device_index= device_index;
        ptf_private->function = function;
        ptf_private->out_ep_addr = usbd_endpoint_bEndpointAddress(function, out_ep, usbd_high_speed(function));
        ptf_private_array[i]=ptf_private;
        return(i);

}
//ptf_private_destroy() destory all private in private array and proto devices

void ptf_private_destroy(void){
        int i;
        for(i=0;i < PTF_MAX_DEV_NUMBER; i++) {
                int dev_index;
                struct usb_ptf_private *ptf_private=ptf_private_array[i];
                if (ptf_private){
                        dev_index=ptf_private->device_index;

                        //destory private in private array
                        ptf_private_array[i] = NULL;	//we have made sure dev_index is equal to private's index in private array

                        kfree (ptf_private);	//in fact, free ptf_private_array[dev_index] 

                        //destory device in device array
                        ptfproto_destory (dev_index);
                }

        }
}


//ap_log_xmit_data():xmit data via ap IN endpoint

static int ap_log_xmit_data (int dev_index, unsigned char *data, int data_length)
{
        struct usb_ptf_private *ptf_private;
        struct urb *urb;
        int packet_length = data_length;
        struct usb_function_instance *function;

        RETURN_EINVAL_IF(!(function = pst_function));

        //printk("ptf_xmit_data() begining:\nUDCCS11=%x UICR1=%x\n",UDCCS11,UICR1);
        if ((ptf_private = get_ptf_private (dev_index)) == NULL) {
                printk (KERN_WARNING "Can't recover private\n");
                return -EINVAL;
        }

        if (!(urb = usbd_alloc_urb (function, NDX_OF_AP_LOG_IN_EP, 0, pst_urb_sent ))) {
                printk (KERN_WARNING  "ap_xmit_data: failed to alloc urb\n");
                return -EINVAL;
        }

        urb->buffer = data;
        urb->actual_length = packet_length;

        // push it down into the usb-device layer
        return usbd_send_urb (urb);	//??? finally bi_send_urb(), return 0 when finished

}

//ptf_xmit_data(): xmit data via ptf IN endpoint

static int ptf_xmit_data (int dev_index, unsigned char *data, int data_length)
{
        struct usb_ptf_private *ptf_private;
        struct urb *urb;
        int packet_length = data_length;
        struct usb_function_instance *function;

        RETURN_EINVAL_IF(!(function = pst_function));

        //printk("ptf_xmit_data() begining:\nUDCCS11=%x UICR1=%x\n",UDCCS11,UICR1);
        if ((ptf_private = get_ptf_private (dev_index)) == NULL) {
                printk (KERN_WARNING "Can't recover private\n");
                return -EINVAL;
        }

        if (!(urb=usbd_alloc_urb (function, NDX_OF_PTF_IN_EP, 0, pst_urb_sent ))) {
                printk (KERN_WARNING  "ptf_xmit_data: failed to alloc urb\n");
                return -EINVAL;
        }

        urb->buffer = data;
        urb->actual_length = packet_length;

        return usbd_send_urb (urb);	//??? finally bi_send_urb(), return 0 when finished

}

//int ptf_urb_sent (struct urb *urb) ???


/* ptf_recv_urb_irq - called to indicate URB has been received (in irq state)
 * @urb - pointer to struct urb
 *
 * Return non-zero if we failed and urb is still valid (not disposed)
 */

int ptf_recv_urb_irq (struct urb *urb, int rc)
{
        struct usb_function_instance *function;
        // XXX TODO FIXME
        //*TBR* I have no idea if this will work, but it should at least compile...
        struct usb_ptf_private **func_priv;
        struct usb_ptf_private *ptf_private;
        int dev_index;

	if (rc != RECV_OK) {
        	urb->privdata = NULL;
		// let caller deallocate it
		return(-1);
	}

	if (NULL == (function = pst_function) ||
           (func_priv = usbd_function_get_privdata(function)) != ptf_private_array) { 
		return(-1);
	}

        /* yes, it is our function driver and it is our turn! */
	/* XXX This whole mechanism should be replaced by putting different
               callback routines into the urbs when they are created, or by
               putting the ptf_private pointer into the urb->privdata.
               This would allow removing the include of usbd-bus.h for the endpoint info */

        //ptf_private = get_ptf_private_by_out_ep(urb->endpoint->endpoint_address);
	ptf_private = urb->privdata;

        if(NULL==ptf_private){
                printk("no suiable private for this OUT endpoint");
                return -EINVAL;
        }

        //get bus array index from private

        dev_index = ptf_private->device_index;

        if (ptfproto_recv (dev_index, urb->buffer, urb->actual_length)) 
                return (-1);

        // recycle urb
	// QQQQQ zap buffer?
        usbd_start_recv (urb);
        return 0;
}


/*
 * Motorola PST interfaces device driver
 *
 * USB interface 6c --- Data Command
 * Major: 10, Minor 17
 *
 *   % mknod /dev/usbi6c c 10 17
 *
 *    endpoints: ep0
 *      read and write:
 *
 * USB interface 6d --- Data Logger
 * Major: 10, Minor 19
 *
 *   % mknod /dev/usbi6d c 10 19
 *
 *    endpoints: ep1
 *      write:  write log data to host by ep1 bulkin transfer mode.
 *
 * USB interface 8 --- Test Command
 * Major: 10, Minor 18
 *
 *   % mknod /dev/usbi8 c 10 18
 *
 *    endpoints: ep0
 *      write: write messages or command results to host by ep0 control transfer mode.
 *      read: read request->wValue to user space.
 *
 *
 * dump all data in the message list for debug.
 */
#ifdef PST_DUMP_DEBUG
static void dump_msg_list( struct list_head *list )
{
        struct list_head *pos;
        msg_node_t *node;
        int i;

        printk( KERN_DEBUG "dump_msg_list\n");
        list_for_each( pos, list ) {
                node = list_entry ( pos, msg_node_t, list);
                if (!node) { 
                        printk( KERN_DEBUG "a NULL node!");
                } else {
                        for ( i=0; i<node->size; i++ ) {
                                printk( KERN_DEBUG "0x%2.2x ", *(node->body+i));
                        }
                        printk( KERN_DEBUG "\n");
                }
        }
}
#else
static void dump_msg_list( struct list_head *list ) 
{
}
#endif 

static int
init_interface (interface_t * intf)
{
        intf->enable = 0;
        intf->list_len = 0;
        INIT_LIST_HEAD (&intf->in_msg_list);
        INIT_LIST_HEAD (&intf->out_msg_list);

        return 0;
}

static int
usbc_interface6c_open (struct inode *pInode, struct file *pFile)
{  
        if (init_interface (&intf6) < 0) 
                return -1;
        PRINTKD("\nintf8 is opened.\n");
        if (usbi0_ref_count == 0)
                intf6.enable = 1;

        usbi0_ref_count++;
        MOD_INC_USE_COUNT;

        return 0;
}

static ssize_t
usbc_interface6c_read (struct file *pFile, char *pUserBuffer,
                size_t stCount, loff_t * pPos)
{
        int ret;
        struct list_head* ptr=NULL;
        msg_node_t *node;

        while (list_empty (&intf6.out_msg_list))
        {
                if (pFile->f_flags & O_NONBLOCK) 
                {
                        printk ( KERN_INFO "no data in interface6 msg list now!\n");
                        return -EAGAIN;
                }
                PRINTKD("\"%s\" reading: going to sleep\n", current->comm);
                interruptible_sleep_on(&wq_read6);
                if (signal_pending(current))
                        return -ERESTARTSYS;
        }

        ptr = intf6.out_msg_list.next;
        node = list_entry (ptr, msg_node_t, list);
        ret = node->size;
        if (stCount <= ret){
                ret = stCount;
        }
        if ( copy_to_user (pUserBuffer, node->body, ret) ) 
        {
                printk ( KERN_INFO " copy_to_user() has some problems! node->size=%d\n", node->size);
                return 0;
        }

        list_del (ptr);
        kfree (node->body);
        node->body = NULL;
        kfree (node);
        return ret;
}

/*
   In this function, we need some critical section tests.
 */
static ssize_t
usbc_interface6c_write (struct file *pFile, const char *pUserBuffer,
                size_t stCount, loff_t * pPos)
{
        u8 *data;
        msg_node_t *node;

        if (stCount == 0)
                return 0;

        data = (u8 *) kmalloc (stCount, GFP_KERNEL);
        if (!data)
        {
                printk( KERN_INFO " kmalloc(%d) out of memory!\n", stCount);
                return -EFAULT;
        }

        if ( copy_from_user (data, pUserBuffer, stCount) )
        {
                kfree(data);
                printk( KERN_INFO "copy_from_user() failed.\n" );
                return -EFAULT;
        }

        node = (msg_node_t *) kmalloc (sizeof (msg_node_t), GFP_KERNEL);
        if (!node)
        {
                printk( KERN_INFO "kmalloc(%d) out of memory!\n", sizeof (msg_node_t));
                kfree(data);
                return -EFAULT;
        }

        node->size = stCount;		/* Default, tCount==node->size */
        node->body = data;

        list_add_tail (&node->list, &intf6.in_msg_list);
        intf6.list_len++;

        while(1){
                if(specific_times6)
                {
                        specific_times6--;
                        if(polling_times6)
                        {
                                polling_message(&intf6, stCount);
                                polling_times6--;
                        }
                        else
                                polling_ready6++;
                        break;
                }
        }

        *pPos = stCount;
        return stCount;
}

static int
usbc_interface6c_release (struct inode *pInode, struct file *pFile)
{
        dump_msg_list( &intf6.in_msg_list );

        usbi0_ref_count--;

        if (!usbi0_ref_count)
                intf6.enable = 0;
        MOD_DEC_USE_COUNT;

        return 0;
}

static int
usbc_interface8_open (struct inode *pInode, struct file *pFile)
{
        if (init_interface (&intf8) < 0)
                return -1;
        PRINTKD("\nintf8 is opened.\n");
        if ( usbi1_ref_count==0 ) 
                intf8.enable = 1;
	int intf8_read_cnt=0;

        usbi1_ref_count++;
        MOD_INC_USE_COUNT;

        return 0;
}


#if PST_DUOINTF
static ssize_t
usbc_interface8_read (struct file *pFile, char *pUserBuffer, size_t stCount, loff_t * pPos)
{
        __u16 ret;
        struct list_head* ptr=NULL;
        msg_node_t *node;
	int readed_cnt=0;
	char* pbufferStart = pUserBuffer;
	int  rcvlen;

        while (list_empty (&intf8.out_msg_list))
        {
                if (pFile->f_flags & O_NONBLOCK) 
                {
                        printk ( KERN_INFO "no data in interface8 msg list now!\n");
                        return -EAGAIN;
                }
                PRINTKD("\"%s\" reading: going to sleep\n", current->comm);
                interruptible_sleep_on(&wq_read8);
                if (signal_pending(current))                  
                        return -ERESTARTSYS;
        }
		
Lread_again:	//LIN bulk needed		
	
        ptr = intf8.out_msg_list.next;
        node = list_entry (ptr, msg_node_t, list);
        ret = node->size;
//LIN PRINTKD("\nintf8_read2 buf[%x %x %x %x] stCount%d\ ret%d \n",pUserBuffer[0],pUserBuffer[1],pUserBuffer[2],pUserBuffer[3],stCount,ret);
        
	if (stCount < ret){
		//LIN WARNING!!!!! 
		//LIN should keep remainder for next read op, but test command application doesn't think so
		printk(KERN_INFO "\n Cut Data %d from %d \n ",stCount,ret);
                ret = stCount;
        }

        if ( copy_to_user (pUserBuffer, node->body, ret) ) 
        {
                kfree (node->body);
                printk ( KERN_INFO " copy_to_user() has some problems! node->size=%d\n", node->size);
                return 0;
        }

	readed_cnt+=ret; //LIN if host send data above a usb packet size, need read again, finish flag is short packet

        list_del (ptr);
        kfree (node->body);
        node->body = NULL;
	kfree (node);

	if(g_tcmd_bulkorcontrol == TCMD_BULKPORT){
//LIN for flow control		
			intf8_read_cnt-=usbd_high_speed(pst_function)?TEST_CMD_PKTSIZE_HS:TEST_CMD_PKTSIZE_FS;
			
		if(ret >= usbd_high_speed(pst_function)?TEST_CMD_PKTSIZE_HS:TEST_CMD_PKTSIZE_FS){
			PRINTKD(" max packet, continue\n");
			pUserBuffer+=ret;
			if(!list_empty (&intf8.out_msg_list)){
				goto Lread_again;
			}
			else{
				//LIN If End Packet is a max size Packet, I don't know whether the transfer is end
				//LIN so following Dan's saying, cal the length of the transfer
				copy_from_user((char*)(&rcvlen),&(pbufferStart[8]),4);
				rcvlen = swab32(rcvlen);
				PRINTKD("\n---- USB Count %d n %d \n",rcvlen,readed_cnt);
				if(readed_cnt < rcvlen+12){//LIN wait a while for follow data. 12 is P2K05 TLCMD header length
					printk("\n abnormal wait -liulin\n");
					interruptible_sleep_on_timeout(&wq_read8,TCMDBULKREADTIMEOUT); 
				}

				if(!list_empty (&intf8.out_msg_list)){
					goto Lread_again;
				}else{
					PRINTKD("\n multiple of packet size No more data\n");
				}

			}
		}
	}
	else{
	//LIN else is a control interface read, return 
	}

	PRINTKD("\nusb_read: ret=%d jiffies=%d\n", readed_cnt,jiffies);
	return readed_cnt;
}

#else
 #if PST_BULKINTF

static ssize_t
usbc_interface8_read (struct file *pFile, char *pUserBuffer,
                size_t stCount, loff_t * pPos)
{
     __u16 ret;
        struct list_head* ptr=NULL;
        msg_node_t *node;
	int readed_cnt=0;
	char* pbufferStart = pUserBuffer;
	int  rcvlen;
	
//LIN maybe speed change	int MaxPktSize= usbd_high_speed(pst_function)?TEST_CMD_PKTSIZE_HS:TEST_CMD_PKTSIZE_FS;

        while (list_empty (&intf8.out_msg_list))
        {
                if (pFile->f_flags & O_NONBLOCK) 
                {
                        printk ( KERN_INFO "no data in interface8 msg list now!\n");
                        return -EAGAIN;
                }
                PRINTKD("\"%s\" reading: going to sleep\n", current->comm);
                interruptible_sleep_on(&wq_read8);
                if (signal_pending(current))                  
                        return -ERESTARTSYS;
        }
		
Lread_again:		
	
        ptr = intf8.out_msg_list.next;
        node = list_entry (ptr, msg_node_t, list);
        ret = node->size;
PRINTKD("\nintf8_read2 buf[%x %x %x %x] stCount%d\ ret%d \n",pUserBuffer[0],pUserBuffer[1],pUserBuffer[2],pUserBuffer[3],stCount,ret,g_tcmd_bulkorcontrol);

        if (stCount <= ret){
                //LIN WARNING!!!!!
                //LIN should keep remainder for next read op, but test command application doesn't think so
                printk(KERN_INFO "\n Cut Data %d from %d \n ",stCount,ret);
                ret = stCount;
        }

        if ( copy_to_user (pUserBuffer, node->body, ret) ) 
        {
                kfree (node->body);
                printk ( KERN_INFO " copy_to_user() has some problems! node->size=%d\n", node->size);
                return 0;
        }

	readed_cnt+=ret; //LIN if host send data above a usb packet size, need read again, finish flag is short packet

        list_del (ptr);
        kfree (node->body);
        node->body = NULL;
	 kfree (node);


	if(g_tcmd_bulkorcontrol == TCMD_BULKPORT){
		
					intf8_read_cnt-=usbd_high_speed(pst_function)?TEST_CMD_PKTSIZE_HS:TEST_CMD_PKTSIZE_FS;
					
		if(ret >= usbd_high_speed(pst_function)?TEST_CMD_PKTSIZE_HS:TEST_CMD_PKTSIZE_FS){
			PRINTKD(" max packet, continue\n");
			pUserBuffer+=ret;
			if(!list_empty (&intf8.out_msg_list)){
				goto Lread_again;
			}
			else{
				copy_from_user((char*)(&rcvlen),&(pbufferStart[8]),4);
				rcvlen = swab32(rcvlen);
				PRINTKD("\n---- USB Count %d n %d \n",rcvlen,readed_cnt);

				if(readed_cnt < rcvlen+12){//LIN wait a while for follow data. 12 is P2K05 TLCMD header length
					interruptible_sleep_on_timeout(&wq_read8,TCMDBULKREADTIMEOUT); 
				}

				if(!list_empty (&intf8.out_msg_list)){
					goto Lread_again;
				}else{
					PRINTKD("\n multiple of packet size No more data\n");
				}

			}
		}
	}
	else{
	//LIN else is a control interface read, return 
	}
	
	PRINTKD("\nusb_read: ret=%d rcvlen %d jiffies=%d\n", readed_cnt,jiffies,rcvlen);
	return readed_cnt;
}

 #else

static ssize_t
usbc_interface8_read (struct file *pFile, char *pUserBuffer,
                size_t stCount, loff_t * pPos)
{
        __u16 ret;
        struct list_head* ptr=NULL;
        msg_node_t *node;

        PRINTKD("\nThis is intf8_read1.\n");
        //printk("\nThis is intf8_read1.\n");

        while (list_empty (&intf8.out_msg_list))
        {
                if (pFile->f_flags & O_NONBLOCK) 
                {
                        printk ( KERN_INFO "no data in interface8 msg list now!\n");
                        return -EAGAIN;
                }
                PRINTKD("\"%s\" reading: going to sleep\n", current->comm);
                interruptible_sleep_on(&wq_read8);
                if (signal_pending(current))                  
                        return -ERESTARTSYS;
        }

        //printk("\nThis is intf8_read2.\n");
        ptr = intf8.out_msg_list.next;
        node = list_entry (ptr, msg_node_t, list);
        ret = node->size;
        if (stCount <= ret){
                ret = stCount;
        }

        //printk("\nThis is intf8_read3.\n");
        if ( copy_to_user (pUserBuffer, node->body, ret) ) 
        {
                kfree (node->body);
                printk ( KERN_INFO " copy_to_user() has some problems! node->size=%d\n", node->size);
                return 0;
        }

        PRINTKD("\nThis is intf8_read2.\n");
        //printk("\nThis is intf8_read4.\n");
        list_del (ptr);
        kfree (node->body);
        node->body = NULL;
        kfree (node);

        //printk("\nusb_read: ret=%d\n", ret);
        return ret;
}
 #endif
#endif






#if PST_DUOINTF

static ssize_t
usbc_interface8_write_bulk (struct file *pFile, const char *pUserBuffer,
			size_t stCount, loff_t * pPos)
{


	 unsigned char *pkbuf;
        int last_packet_len = 32, count1 = 0;

        if (stCount == 0)
                return 0;
		
//LD	if(stCount > PST_MAX_BUF_CNT)return 0;//LIN flow control
	
        PRINTKD("\nThis is intf8_write1[%x %x %x %x] count=%d jiffies=%d\n",pUserBuffer[0],pUserBuffer[1],pUserBuffer[2],pUserBuffer[3],stCount,jiffies);

#if 0	
	//LIN when last max packet transfered, pc driver need to add one dummy packet.
       if(stCount% (usbd_high_speed(pst_function)?TEST_CMD_PKTSIZE_HS:TEST_CMD_PKTSIZE_FS))
       { 
		PRINTKD("\n NO64\n");
		normal_write2epx(pUserBuffer, stCount,NDX_OF_TESTCMD_IN_EP);
       }else {
		PRINTKD("\n the Last Max Packet \n");
	        count = normal_write2epx(pUserBuffer,      stCount+1        ,NDX_OF_TESTCMD_IN_EP);
		/*LIN if two line below, PC driver will response first packet 
		count = normal_write2epx(pUserBuffer,         stCount-1        ,NDX_OF_TESTCMD_IN_EP);
		count1 = normal_write2epx(pUserBuffer+stCount-1,         1        ,NDX_OF_TESTCMD_IN_EP);
		*/
		return (count+count1);
        }
#endif
       if(stCount% (usbd_high_speed(pst_function)?TEST_CMD_PKTSIZE_HS:TEST_CMD_PKTSIZE_FS))//LIN MAXPACKET
       {
                PRINTKD("\n NO64\n");
                normal_write2epx(pUserBuffer, stCount, NDX_OF_TESTCMD_IN_EP);
       }else {
                PRINTKD("\n 64 \n");
                normal_write2epx(pUserBuffer, stCount, NDX_OF_TESTCMD_IN_EP);
                //LIN follow Dan says,sent 2 more 00 00 to completed the transfer
                if( pxa_usb_send((char*)&count1,     2,       NDX_OF_TESTCMD_IN_EP)<0 ){
                         printk ( KERN_INFO "liulin pxa_usb_send() failed!\n");
                }
        }
		
        *pPos = stCount;
        return stCount;
}

static ssize_t
usbc_interface8_write_control (struct file *pFile, const char *pUserBuffer,
                size_t stCount, loff_t * pPos)
{
        msg_node_t *node;

        if (stCount == 0)
                return 0;

        PRINTKD("\nstCount=%d\n", stCount);
        PRINTKD("\nThis is intf8_write1.\n");

        node = (msg_node_t *) kmalloc (sizeof (msg_node_t), GFP_KERNEL);
        if (!node)
        {
                printk( KERN_INFO "kmalloc(%d) out of memory!\n", sizeof (msg_node_t));
                return -EFAULT;
        }

        node->body = (__u8 *) kmalloc (stCount, GFP_KERNEL);
        if (!(node->body))
        {
                printk( KERN_INFO " kmalloc(%d) out of memory!\n", stCount);
		kfree(node);
                return -EFAULT;
        }
        PRINTKD("\nThis is intf8_write2.\n");
        if ( copy_from_user (node->body, pUserBuffer, stCount) )
        {
                kfree(node->body);
		kfree(node);
                printk( KERN_INFO "copy_from_user() failed.\n" );
                return -EFAULT;
        }

        PRINTKD("\nThis is intf8_write3.\n");
        node->size = stCount;		/* Default, tCount==node->size */
        list_add_tail (&node->list, &intf8.in_msg_list);
        intf8.list_len++;

        while(1){
                if(specific_times8)
                {
                        specific_times8--;
                        if(polling_times8)
                        {
                                polling_message(&intf8, stCount);
                                polling_times8--;
                        }
                        else
                                polling_ready8++;
                        break;
                }
        }

        *pPos = stCount;
        return stCount;
}

static ssize_t
usbc_interface8_write (struct file *pFile, const char *pUserBuffer,
			size_t stCount, loff_t * pPos)
{
	PRINTKD("intf8 write port=%d",g_tcmd_bulkorcontrol);
	if(g_tcmd_bulkorcontrol==TCMD_BULKPORT){
		return usbc_interface8_write_bulk(pFile,pUserBuffer,stCount,pPos);
	}
	else if(g_tcmd_bulkorcontrol==TCMD_CONTROLPORT){
		return usbc_interface8_write_control(pFile,pUserBuffer,stCount,pPos);
	}
	else{
		printk("\n\n liulin ERROR WRITE FIRST!!!!\n\n");
		return 0;
	}
		
}
#else

 #if PST_BULKINTF
static ssize_t
usbc_interface8_write (struct file *pFile, const char *pUserBuffer,
			size_t stCount, loff_t * pPos)
{

//LIN       msg_node_t *node;

	 unsigned char *pkbuf;
        int last_packet_len = 1, count = 0, count1 = 0;

        if (stCount == 0)
                return 0;
		
//LD	if(stCount > PST_MAX_BUF_CNT)return 0;//LIN flow control
	
        PRINTKD("\nThis is intf8_write1[%x %x %x %x] count=%d\n",pUserBuffer[0],pUserBuffer[1],pUserBuffer[2],pUserBuffer[3],stCount);

       if(stCount% (usbd_high_speed(pst_function)?TEST_CMD_PKTSIZE_HS:TEST_CMD_PKTSIZE_FS))//LIN MAXPACKET
       {
                PRINTKD("\n no64\n");
                normal_write2epx(pUserBuffer, stCount,NDX_OF_TESTCMD_IN_EP);
       }else {
                PRINTKD("\n 64 \n");
                //LIN count = normal_write2epx(pUserBuffer,         stCount+1        ,NDX_OF_TESTCMD_IN_EP);
		count = normal_write2epx(pUserBuffer,         stCount        ,NDX_OF_TESTCMD_IN_EP);
		//LIN follow Dan says,sent 2 more 00 00 to completed the transfer 
	      	if( pxa_usb_send(&count1, 2,NDX_OF_TESTCMD_IN_EP)<0 ){
			 printk ( KERN_INFO "liulin pxa_usb_send() failed!\n");
		}
        }

        *pPos = stCount;
        return stCount;
}

 #else

static ssize_t
usbc_interface8_write (struct file *pFile, const char *pUserBuffer,
                size_t stCount, loff_t * pPos)
{
        msg_node_t *node;

        if (stCount == 0)
                return 0;

        PRINTKD("\nstCount=%d\n", stCount);
        PRINTKD("\nThis is intf8_write1.\n");

        node = (msg_node_t *) kmalloc (sizeof (msg_node_t), GFP_KERNEL);
        if (!node)
        {
                printk( KERN_INFO "kmalloc(%d) out of memory!\n", sizeof (msg_node_t));
                return -EFAULT;
        }

        node->body = (__u8 *) kmalloc (stCount, GFP_KERNEL);
        if (!(node->body))
        {
                printk( KERN_INFO " kmalloc(%d) out of memory!\n", stCount);
                return -EFAULT;
        }
        PRINTKD("\nThis is intf8_write2.\n");
        if ( copy_from_user (node->body, pUserBuffer, stCount) )
        {
                kfree(node->body);
                printk( KERN_INFO "copy_from_user() failed.\n" );
                return -EFAULT;
        }

        PRINTKD("\nThis is intf8_write3.\n");
        node->size = stCount;		/* Default, tCount==node->size */
        list_add_tail (&node->list, &intf8.in_msg_list);
        intf8.list_len++;

        while(1){
                if(specific_times8)
                {
                        specific_times8--;
                        if(polling_times8)
                        {
                                polling_message(&intf8, stCount);
                                polling_times8--;
                        }
                        else
                                polling_ready8++;
                        break;
                }
        }

        *pPos = stCount;
        return stCount;
}
 #endif
#endif

static int
usbc_interface8_release (struct inode *pInode, struct file *pFile)
{
        dump_msg_list( &intf8.in_msg_list );

        usbi1_ref_count--;
        if (!usbi1_ref_count)
                intf8.enable = 0;

        MOD_DEC_USE_COUNT;

        return 0;
}

/* *
 * pxa_usb_send - called to transmit an urb
 * @buf: buffer need to be transfered
 * @len: buffer length
 *
 */
static int pxa_usb_send(char *buf, int len, __u8 ENDPOINT)
{	
        struct urb *urb;
        struct usb_function_instance *function;

        RETURN_EINVAL_IF(!(function = pst_function));

        //printk("\npxa_usb_send: ENDPOINT=%d\n", ENDPOINT);

        if (!(urb = usbd_alloc_urb (function, ENDPOINT, len, pst_urb_sent ))) {
                printk("\nurb alloc failed len: %d\n", len);
                return -ENOMEM;
        }

//        printk("\nw20535:pxa_usb_send alloc urb successfully!\n");

        urb->actual_length = len;
        memcpy(urb->buffer, buf, len);//copy data to urb.
        if (usbd_send_urb (urb)) {
                kfree(urb->buffer);
                kfree(urb);
                printk("\nusbd_send_urb failed\n");
                return -ECOMM;
        }

//       printk("\nw20535:pxa_usb_send: send urb successfully!\n");
        return 0;
}

#define BP_LOG_MAX_SIZE		2048
static int
usbc_interface6d_open (struct inode *pInode, struct file *pFile)
{
        usbi2_ref_count++;
        MOD_INC_USE_COUNT;
        return 0;
}

static int normal_write( const char *buffer, int stCount)
{
        u8 *data;

        data = (u8 *) kmalloc (stCount, GFP_KERNEL);
        if (!data)
        {
                printk ( KERN_INFO "kmalloc(%d) out of memory!\n", stCount);
                return -EFAULT;
        }
        if ( copy_from_user (data, buffer, stCount) )
        {
                kfree(data);
                printk ( KERN_INFO "copy_from_user() failed. stCount=%d\n", stCount);
                return 0;
        }
/*+++++++++++++++++++++++++++++++ added by Jordan to support CFG11 bplog ++++++++++++++++++++===*/
	if(motusbd_status ==  0x2){   
		PRINTKD("w20535:cfg11:normal_write end03\n");
        	if ( pxa_usb_send(data, stCount, 0x3)<0 )
	        {
        	        printk ( KERN_INFO "pxa_usb_send() failed!\n");
	                return 0;
        	}
	}else {
        	if ( pxa_usb_send(data, stCount, /* DATA_LOG_IN_ENDPOINT */ NDX_OF_DATALOG_IN_EP)<0 )
	        {
        	        printk ( KERN_INFO "pxa_usb_send() failed!\n");
	                return 0;
        	}
	}
/*+++++++++++++++++++++++++++++end by Jordan to support CFG11 bplog*/
        kfree(data);
        udelay(500);
        //printk("\nBP_Data_write: write data successfully.\n");
        return stCount;
}
#if PST_BULKINTF
static int normal_write2epx( const char *buffer, int stCount,int epidx)
{
        u8 *data;

        data = (u8 *) kmalloc (stCount, GFP_KERNEL);
        if (!data)
        {
                printk ( KERN_INFO "kmalloc(%d) out of memory!\n", stCount);
                return -EFAULT;
        }
        if ( copy_from_user (data, buffer, stCount) )
        {
                kfree(data);
                printk ( KERN_INFO "copy_from_user() failed. stCount=%d\n", stCount);
                return 0;
        }

      	if ( pxa_usb_send(data, stCount, epidx)<0 )
       {
      	        printk ( KERN_INFO "pxa_usb_send() failed!\n");
                return 0;
      	}
        kfree(data);
        return stCount;
}
#endif

static ssize_t
usbc_interface6d_write (struct file *pFile, const char *pUserBuffer,
                size_t stCount, loff_t * pPos)
{	
        int last_packet_len = 1, count = 0, count1 = 0;
        if( !cable_connected )
        {
                return -2;
        }
        if(stCount <= 0 || stCount > BP_LOG_MAX_SIZE )
        {
                return 0;
        }

        //printk("\nhere is write_6d_1!\n");	
        if(stCount%64)
        {
                return normal_write(pUserBuffer, stCount);
        }else {
                //printk("\nhere is write_6d_3!\n");	
                stCount--;
                count = normal_write(pUserBuffer, stCount);
                count1 = normal_write(pUserBuffer+last_packet_len, last_packet_len);
                return (count+count1);
        }
}

static int
usbc_interface6d_release (struct inode *pInode, struct file *pFile)
{
        usbi2_ref_count--;
        MOD_DEC_USE_COUNT;
        return 0;
}

/* Intialize and destroy pst-dev interfaces ************************************************** */

/* device fileoperation structures */
static struct file_operations usbc_fop0s = {
	owner:THIS_MODULE,
	open:usbc_interface6c_open,
	read:usbc_interface6c_read,
	write:usbc_interface6c_write,
	poll:NULL,
	ioctl:NULL,
	release:usbc_interface6c_release,
};

static struct file_operations usbc_fop1s = {
	owner:THIS_MODULE,
	open:usbc_interface8_open,
	read:usbc_interface8_read,
	write:usbc_interface8_write,
	poll:NULL,
	ioctl:NULL,
	release:usbc_interface8_release,
};

static struct file_operations usbc_fop2s = {
	owner:THIS_MODULE,
	open:usbc_interface6d_open,
	read:NULL,
	write:usbc_interface6d_write,
	poll:NULL,
	ioctl:NULL,
	release:usbc_interface6d_release,
};

static struct miscdevice usbc_misc_device0 = {
        USBC_MINOR0, "datacmd", &usbc_fop0s
};

static struct miscdevice usbc_misc_device1 = {
        USBC_MINOR1, "testcmd", &usbc_fop1s
};

static struct miscdevice usbc_misc_device2 = {
        USBC_MINOR2, "datalog", &usbc_fop2s
};

/* *
 * pst_dev_create - create an interface
 *
 */
int pst_dev_create (struct usb_function_instance *function)
{
        int rc;
	/* register bus driver */
        rc = misc_register (&usbc_misc_device0);
        PRINTKD("\nRegister device0(data cmd)\n");
        if (rc != 0)
        {
                printk (KERN_INFO "Couldn't register misc device0(Data Cmd).\n");
                return -EBUSY;
        }

        rc = misc_register (&usbc_misc_device1);
        PRINTKD("\nRegister device1(test cmd)\n");
        if (rc != 0)
        {
                printk (KERN_INFO "Couldn't register misc device1(Test Cmd).\n");
                goto dev1_register_error;
        }

        rc = misc_register (&usbc_misc_device2);
        PRINTKD("\nRegister device2(data log)\n");
        if (rc != 0)
        {
                printk (KERN_INFO "Couldn't register misc device1(Data Log).\n");
                goto dev2_register_error;
        }
        printk (KERN_INFO "USB Function Character Driver Interface"
                        " -%s, (C) 2002, Motorola Corporation.\n", VERSION);

        return 0;

dev2_register_error:
        misc_deregister (&usbc_misc_device1);
dev1_register_error:
        misc_deregister (&usbc_misc_device0);
        return -EBUSY;

}

/* *
 * pst_dev_destroy - destroy an interface
 *
 */
void pst_dev_destroy (void)
{
        pst_function = NULL;
        misc_deregister (&usbc_misc_device0);
        misc_deregister (&usbc_misc_device1);
        misc_deregister (&usbc_misc_device2);
}

static void start_recv(struct usb_function_instance *function)
{
	struct urb *urb;
	int recv_len;
	/* we need to allocate URBs with enough room for the endpoints
           transfersize plus one rounded up to the next packetsize */
	// AP_LOG
	if (ap_log_ndx < 0 || ptf_ndx < 0) {
		return;
	}
        recv_len = AP_DATA_LOG_OUT_PKTSIZE; /* XXX TBR: This should be the max tranfersize,
                                                   not pktsize, but I don't know what the
					           AP_LOG transfer size should be. MCEL - FIX */
        recv_len = ((recv_len + AP_DATA_LOG_OUT_PKTSIZE) / AP_DATA_LOG_OUT_PKTSIZE) *
                    AP_DATA_LOG_OUT_PKTSIZE;
	// Yes, this really is ptf_recv_urb_irq(), even for ap_log endpoint.
	RETURN_IF(!(urb = usbd_alloc_urb(function,NDX_OF_AP_LOG_OUT_EP,recv_len,ptf_recv_urb_irq)));
	urb->privdata = ptf_private_array[ap_log_ndx];
        PRINTKD("\nalloc urb aplog ep=0x%x\n",urb->endpoint->bEndpointAddress);
	RETURN_IF(usbd_start_recv(urb));
	// PTF
        recv_len = PTF_OUT_PKTSIZE; /* XXX TBR: This should be the max tranfersize,
                                                not pktsize, but I don't know what the
			                        PTF transfer size should be. MCEL - FIX */
        recv_len = ((recv_len + PTF_OUT_PKTSIZE) / PTF_OUT_PKTSIZE) * PTF_OUT_PKTSIZE;
	RETURN_IF(!(urb = usbd_alloc_urb(function,NDX_OF_PTF_OUT_EP,recv_len,ptf_recv_urb_irq)));
	urb->privdata = ptf_private_array[ptf_ndx];
	PRINTKD("\nalloc urb ptf ep=0x%x\n",urb->endpoint->bEndpointAddress);
	RETURN_IF(usbd_start_recv(urb));

	
#if PST_BULKINTF
	//LIN-050824 penging a urb in ep of testcmd
	recv_len =usbd_high_speed(pst_function)?TEST_CMD_PKTSIZE_HS:TEST_CMD_PKTSIZE_FS;
	recv_len *=2;
	urb = usbd_alloc_urb(function,NDX_OF_TESTCMD_OUT_EP,recv_len,callback4bulktcmd);
	PRINTKD("\nlen:%d  reqlen:%d  buflen:%d pkts:%d\n",recv_len,urb->request_length,urb->buffer_length,urb->endpoint->wMaxPacketSize);

	RETURN_IF(!urb);
	urb->privdata = &intf8;
	PRINTKD("\nalloc urb intf8 ep=0x%x\n",urb->endpoint->bEndpointAddress);
	RETURN_IF(!usbd_start_recv(urb));
	
	// Second stage of receive didn't start successfully.
	usbd_dealloc_urb(urb);
			
#endif
	
}

/* Called when a USB Device is created or destroyed  ***************************************** */

/* Called to handle USB Events  ************************************************************** */

/**
 * pst_event_irq - process a bus event
 * @bus: usb bus 
 * @event: the event that happened
 *
 * Called by the usb device core layer to respond to various USB events.
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */

void pst_event_irq (struct usb_function_instance *function, usb_device_event_t event, int data)
{
        int ret;
        //int i;
        //int device_index,dev_index;//interface
        //struct usb_ptf_private * func_priv;

        switch (event) {
        case DEVICE_CONFIGURED:
                cable_connected = 1;
                ptf_cable_connected=1;

		/* Old API used to auto-start OUT endpoints.  Now we need
                   to allocate and start our own urbs, with appropriate
                   callbacks. */
		start_recv(function);

                wake_up_interruptible(&ptf_write_queue);	
                PRINTKD("\nPST USB Device is configured.\n");
                break;	

        case DEVICE_CREATE:	// a bus interface driver has created a usb device
                PRINTKD("\nPST Misc Device is created.\n");
                if (0 > pst_dev_create (function)) {
                        PRINTKD("Create PST Device Failed!\n");
                        return;
                }

                usbd_function_set_privdata(function,ptf_private_array);
		
		if (!ptf_func_created){

                //enter ptf and ap log part
		ptf_ndx = ap_log_ndx = -1;
                ptf_ndx = ptf_private_create(function,NDX_OF_PTF_OUT_EP,ptf_xmit_data,PTF_OUT_PKTSIZE);
                if ( 0 > ptf_ndx){
                        printk("can not create private for ptf\n");
                        return; 
                }

                ap_log_ndx = ptf_private_create(function, NDX_OF_AP_LOG_OUT_EP, ap_log_xmit_data, AP_DATA_LOG_OUT_PKTSIZE);
                if ( 0 > ap_log_ndx){
                        printk("can not create private for ap log\n");
                        return; 
                }
			
		ptf_func_created=1;
		} /* end if (!ptf_func_created) */
		/* whatever first create or not, here ptf init should be ok... */
		
		enable_ptf_func_driver();
                break;

        case DEVICE_HUB_RESET:
                cable_connected = 0;
                ptf_cable_connected=0;
                break;

        case DEVICE_DESTROY:	// the bus interface driver is unloading
                PRINTKD("\nPST Misc Device is destroyed.\n");
                pst_dev_destroy ();

                //enter ptf and ap logger part

                if (ptf_private_array == usbd_function_get_privdata(function)) {
		  
		  ptfproto_send_fasync_to_all_devices(SIGIO);
#if 0	  /*a17400: when supporting usb switching, all private and proto dev structures will be keeped staticaly.*/
		  ptf_private_destroy();
#endif		  
		  //clear flag to indicates invalid bottom structure
		//ptf_func_created=0;
		  disable_ptf_func_driver();
                }
	
                break;

        default:
                break;
        }
        return;
}

// void specific_message( interface_t *interface, struct usb_device_request *req )
int specific_message( struct urb *urb, int rc )
{
	/* This is called only when *req has direction DATA_OUT,
	   so there is no immediate reply needed. */
	interface_t *interface = urb->privdata;
        msg_node_t *node;

	if (rc != RECV_OK) {
		// URB not received properly.
		goto bail;
	}

        node = (msg_node_t*)kmalloc( sizeof(msg_node_t), GFP_ATOMIC );
        if ( !node ) {
                printk( "specific_messages: (%d) out of memory!\n", sizeof(msg_node_t) );
                goto bail;
        }

        node->size = urb->actual_length;
        node->body = (__u8*)kmalloc( sizeof(__u8)*(node->size), GFP_ATOMIC );
        if ( !(node->body) ) {
                printk( "specific_messages: (%d) out of memory!\n", sizeof(__u8)*(node->size) );
                kfree(node);
                goto bail;
        }
        memcpy( node->body, urb->buffer, node->size );
        list_add_tail( &node->list, &interface->out_msg_list);

        // if(req->wIndex == 6
        if (interface == &intf6) 
        {
		PRINTKD( "interface6 " );
                specific_times6++;
                wake_up_interruptible(&wq_read6);
        }else if (interface == &intf8) 
        {
		PRINTKD( "interface8 " );
                specific_times8++;
#if PST_DUOINTF
		  g_tcmd_bulkorcontrol = TCMD_CONTROLPORT;
		PRINTKD("\n   g-control %d 0x%x\n",g_tcmd_bulkorcontrol,&g_tcmd_bulkorcontrol);
#endif
                wake_up_interruptible(&wq_read8);
        }
  
	PRINTKD( "specific_messages: return successfully!\n" );	 
bail:
	usbd_dealloc_urb(urb);
        return(0);
}
#if PST_BULKINTF
static int callback4bulktcmd( struct urb *urb, int rc )
{
	/* This is called only when *req has direction DATA_OUT,
	   so there is no immediate reply needed. */
	interface_t *interface = urb->privdata;
        msg_node_t *node;

	if (rc != RECV_OK) {
		// URB not received properly.
		goto bail;
	}
	

	if(!interface->enable) goto bail;//LIN interface have not been open, but pc sent data to it, ignore it.051114

//LIN flow control
	intf8_read_cnt+=usbd_high_speed(pst_function)?TEST_CMD_PKTSIZE_HS:TEST_CMD_PKTSIZE_FS;
	if(intf8_read_cnt>PST_MAX_BUF_CNT)
	{
		printk("\n No more buffer! \n");
		goto bail;
	}

        node = (msg_node_t*)kmalloc( sizeof(msg_node_t), GFP_ATOMIC );
        if ( !node ) {
                printk( "callback4bulktcmd: (%d) out of memory!\n", sizeof(msg_node_t) );
                goto bail;
        }

        node->size = urb->actual_length;
        node->body = (__u8*)kmalloc( sizeof(__u8)*(node->size), GFP_ATOMIC );
        if ( !(node->body) ) {
                printk( "callback4bulktcmd: (%d) out of memory!\n", sizeof(__u8)*(node->size) );
                kfree(node);
                goto bail;
        }

        memcpy( node->body, urb->buffer, node->size );
        list_add_tail( &node->list, &interface->out_msg_list);
#if PST_DUOINTF
	PRINTKD("\n  g-bulk %d 0x%x\n",g_tcmd_bulkorcontrol,&g_tcmd_bulkorcontrol);
	g_tcmd_bulkorcontrol = TCMD_BULKPORT;
#endif
        wake_up_interruptible(&wq_read8);
 
	 RETURN_ZERO_IF(!usbd_start_recv (urb));//LIN  a cycle finished!

bail:
	usbd_dealloc_urb(urb);
        return(0);
}
#endif


/* 
 * invoked by process_interface8_request and process_interface6_request
 *
 * process the polling request for interface
 *
 */

void polling_return( struct usb_device_request *req )
{
        polling_len = req->wLength;

        PRINTKD("\npolling_return..., polling_times6=%d polling_ready8=%d specific_times6=%d\n", 
                        polling_times6,polling_ready8,specific_times6);
        PRINTKD("\npolling_return..., polling_times8=%d polling_ready8=%d specific_times8=%d\n", 
                        polling_times8,polling_ready8,specific_times8);

        if(req->wIndex == 6) {
                if( (specific_times6 == 0) || (polling_ready6 > 0) ) {
                        polling_message_return( &intf6, polling_len );
                        if(polling_ready6 > 0) 
                                polling_ready6--;
                }
                else 
                        polling_times6++;
        }
        else if(req->wIndex == 8) {

                if( (specific_times8 == 0) || (polling_ready8 > 0) ) {

                        polling_message_return( &intf8, polling_len );

                        if(polling_ready8 > 0)
                                polling_ready8--;
                }
                else
                        polling_times8++;
        }
        return;
}
void polling_message( interface_t *interface, int polling_data_len )
{
        polling_data_t pd;
        struct list_head *ptr=NULL;
        msg_node_t* entry;
        int i;
        struct urb *urb;
        struct usb_function_instance *function;

        RETURN_IF(!(function = pst_function));


        memset( (void*)&pd, 0, sizeof(polling_data_t) );
        PRINTKD("\nThis is polling1\n");

        if ( interface->list_len>3 ) {
                pd.status = 0x02;
                pd.numMessages = 3;
        } 
        else {
                pd.status = 0x00;
                pd.numMessages = interface->list_len;
        }

        ptr = interface->in_msg_list.next;
        for ( i=0; i<pd.numMessages; i++ ) {
                entry = list_entry( ptr, msg_node_t, list );
                pd.msgsiz[i] = make_word_b(entry->size);
                ptr = ptr->next;
        }

        //need to build a urb to send to host.
        PRINTKD("\nThis is polling2\n");


        /* write back to HOST */
        send_ep0_urb_to_host( &pd, sizeof(__u8)*2+sizeof(__u16)*pd.numMessages );

        PRINTKD("polling messages: send out response now!\n");
        return;
}

void polling_message_return( interface_t *interface, int polling_data_len )
{
        //struct pst_private *pst = &pst_private;
        struct usb_function_instance *function;
        struct urb *urb;

        polling_data_t pd;
        struct list_head *ptr=NULL;
        msg_node_t* entry;
        int i,len;

        RETURN_IF(!(function = pst_function));
        memset( (void*)&pd, 0, sizeof(polling_data_t) );
        PRINTKD("\nThis is polling_message_return1\n");

        if ( interface->list_len>3 ) {
                pd.status = 0x02;
                pd.numMessages = 3;
        } else {
                pd.status = 0x00;
                pd.numMessages = interface->list_len;
        }

        ptr = interface->in_msg_list.next;
        for ( i=0; i<pd.numMessages; i++ ) {
                entry = list_entry( ptr, msg_node_t, list );
                pd.msgsiz[i] = make_word_b(entry->size);
                ptr = ptr->next;
        }

        len = sizeof(__u8)*2+sizeof(__u16)*pd.numMessages;
        RETURN_IF(!(urb = usbd_alloc_urb_ep0(function, len, NULL)));

        /* write back to HOST */
        urb->actual_length = len;
        memcpy(urb->buffer, &pd, len);//data

        RETURN_IF(!usbd_send_urb(urb));
        usbd_dealloc_urb(urb);

        PRINTKD("polling messages return: send out response now!\n");
        return;
}
/* 
 * invoked by process_interface6_request and process_interface8_request
 *
 * process the queue request for interface
 * The reqest has direction DATA_IN, so we need to send a reply.
 *
 */
void queue_message( interface_t *interface, struct usb_device_request *req)
{

        struct list_head* ptr=NULL,*tmp;
        msg_node_t* entry=NULL;
        queued_data_t *data=NULL;
        int numReq,i,numMsgBytes=0,numQueuedBytes=0,data_size=0;
        msg_t* msg_array;

        numReq = MIN( interface->list_len, req->wValue );
        PRINTKD("\nThe request wanted to be returned to host is %d.\n", numReq );

        ptr = interface->in_msg_list.next;
        for (i=0; i<numReq; i++ ) {
                entry = list_entry( ptr, msg_node_t, list );
                numMsgBytes += entry->size;
                ptr = ptr->next;
        }

        for (i=0; i<(interface->list_len-numReq); i++ ) {
                entry = list_entry( ptr, msg_node_t, list );
                numQueuedBytes += entry->size;
                ptr = ptr->next;
        }

        PRINTKD("\nThis is queue1\n");
        data_size = sizeof(queued_data_t)+sizeof(__u16)*numReq+numMsgBytes;
        PRINTKD("\ndata_size= %d\n", data_size);
        data = (queued_data_t*) kmalloc( data_size, GFP_ATOMIC );

        if ( !data ) {
                printk( "\nqueue_message: (%d) out of memory!\n", data_size );
                return;
        }
        data->numMsgs = numReq;
        data->numQueuedMsgs = interface->list_len - numReq;
        data->numQueuedBytes = make_word_b(numQueuedBytes);
        msg_array = (msg_t*) (data+1);
        PRINTKD("\ndata= %x\n", data);
        PRINTKD("\nmsg_array= %x\n", &msg_array);

        ptr = interface->in_msg_list.next;
        for ( i=0; i<numReq; i++ ) {
                entry = list_entry(ptr,  msg_node_t, list );
                msg_array->size = make_word_b(entry->size);
                PRINTKD("\nentry->size= %d\n", entry->size);
                PRINTKD("\nentry->body= %x\n", entry->body);
                PRINTKD("\n((char*)(msg_array))+2= %x\n", ((char*)msg_array)+2);
                memcpy(((char*)msg_array)+2, entry->body, entry->size );
                msg_array = (msg_t*)(((char*)msg_array)+2+entry->size);

                tmp = ptr;
                ptr = ptr->next;

                interface->list_len--;
                list_del( tmp );
                kfree( entry->body );
                kfree( entry );
                PRINTKD("\nThis is queue4\n");
        }

	// Send reply via ep0.	
	send_ep0_urb_to_host(data,data_size);

       kfree( data );
       PRINTKD("queue_messages: send out response now.\n");
       return;
}

/* 
 * process_interface0_request 
 * process the request for interface6
 */
/* NOTE: process_interface0_request() is called in interrupt context, so we can't use wait queue here. */
int process_interface6_request( struct usb_device_request *req )
{
        struct usb_function_instance *function;
        struct urb *urb;
	int len_reqd;

        RETURN_EINVAL_IF(!(function = pst_function));

        switch ( req->bRequest ) {
        case 0: /* Polling */
                if ( DATA_DIRECTION(req->bmRequestType) == DATA_IN ) {
                        polling_return( req );
                } else {
                        PRINTKD( "process_interface6_request() Invalid polling direction.\n" );
                }
                break;
        case 1: /* Queued Messages */
                if ( DATA_DIRECTION(req->bmRequestType) == DATA_IN ) {
                        queue_message( &intf6, req );
                } else {
                        PRINTKD( "process_interface6_request() Invalid queue direction.\n" );
                }
                break;
        case 2: /* Interface specific request */
                if ( DATA_DIRECTION(req->bmRequestType) == DATA_OUT ) {
			/* New API requires a two-stage receive for control write.
			   First stage is just the device request portion.
                           Device must allocate and start an urb to receive the second stage.
			   Use the urb received callback to maintain context (in this case,
			   specific_message( &intf6, ...)). */
			urb = usbd_alloc_urb_ep0(function, le16_to_cpu(req->wLength), specific_message);
			RETURN_EINVAL_IF(!urb);
			urb->privdata = &intf6;
			RETURN_ZERO_IF(!usbd_start_recv(urb));
			// Second stage of receive didn't start successfully.
			usbd_dealloc_urb(urb);
			return(-EINVAL); //_____ check w SL re Q2
                } 
                break;
        case 3: /* Security - TBD */
                PRINTKD( "TBD\n" );
                break;
        default:
                break;
        }
        return(0);
}

/* 
 * NOTE: process_interface8_request() is called in interrupt context, so we can't use wait queue here.
 *                             
 */
int process_interface8_request( struct usb_device_request *req )
{
        struct usb_function_instance *function;
        struct urb *urb;
	int len_reqd;

        RETURN_EINVAL_IF(!(function = pst_function));

        switch ( req->bRequest ) {
        case 0: /* Polling */
                PRINTKD("This is Polling case.\n");
                if ( DATA_DIRECTION(req->bmRequestType) == DATA_IN ) {
                        polling_return(req);
                } else {
                        PRINTKD( "process_interface8_request() Invalid polling direction.\n" );
                }
                break;

        case 1: /* Queued Messages */
                PRINTKD("This is Queued Messages case.\n");
                if ( DATA_DIRECTION(req->bmRequestType) == DATA_IN ) {
                        queue_message( &intf8, req );
                } else {
                        PRINTKD( "process_interface8_request() Invalid queued direction.\n" );
                }
                break;

        case 2: /* Interface specific request */
                PRINTKD("This is Test Command case.\n");
                if ( DATA_DIRECTION(req->bmRequestType) == DATA_OUT ) {
			/* New API requires a two-stage receive for control write.
			   First stage is just the device request portion.
                           Device must allocate and start an urb to receive the second stage.
			   Use the urb received callback to maintain context (in this case,
			   specific_message( &intf8, ...)). */
			urb = usbd_alloc_urb_ep0(function, le16_to_cpu(req->wLength), specific_message);
			RETURN_EINVAL_IF(!urb);
			urb->privdata = &intf8;
			RETURN_ZERO_IF(!usbd_start_recv(urb));
			// Second stage of receive didn't start successfully.
			usbd_dealloc_urb(urb);
			return(-EINVAL); // _____ check w SL re Q2
                } else {
                        PRINTKD( "process_interface8_request() Invalid interface command direction.\n" );
                }
                break;

        case 3: /* Security - TBD */
                PRINTKD( "Security command - TBD.\n" );
                break;

        default:
                break;
        }
        return(0);
}

/* We will process some interface specific request here. */
int ep0_process_vendor_request( struct usb_device_request *req )
{
        int enable,count,rc = 0;
        __u8 num_cfg = USB_NUM_OF_CFG_SETS;

#if PST_DEBUG
        {
                unsigned char * pdb = (unsigned char *) &req;
                PRINTKD( "VENDOR REQUEST:%2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X ",
                                pdb[0], pdb[1], pdb[2], pdb[3], pdb[4], pdb[5], pdb[6], pdb[7]
                       );
        }
#endif
        /* 
         * Do we need to do any thing else? We only throw it to upper driver and let 
         * them to decide what to do 
         */ 
        enable = 1;
        count = 100;
        if ( (req->bmRequestType&0x0F) == 0x00 ) 
        {
                switch ( req->bRequest ) {
                case 0:
                        //This branch will Report Number of configurations.
                        PRINTKD( "DEVICE 0\n");
                        if (((req->bmRequestType & 0x80) != 0)
                                        && (req->wValue == 0)
                                        && (req->wIndex == 0)
                                        && (req->wLength == 2))
                        {
                                send_ep0_urb_to_host( &num_cfg, 1 );
                        }
                        break;

                case 1:
                        //This branch will Change to alternate configuration set.
                        PRINTKD( "DEVICE 1\n");
                        if (((req->bmRequestType & 0x80) == 0)
                                        && (req->wIndex <= USB_NUM_OF_CFG_SETS)
                                        && (req->wValue == 0)
                                        && (req->wLength == 0) )
                        {
                                send_ep0_urb_to_host( NULL, 0 );
                                /*	do{
                                        count--;
                                        udelay(1);
                                        }while(count);*/
                                //	usb_reset_for_cfg8(enable);

                        }
                        break;

                case 2:
                        //This branch will List descriptors for alternate configuration set
                        PRINTKD( "DEVICE 2\n");
                        break;

                case 3:
                        //This branch will Quick-change to alternate configuration set
                        PRINTKD( "DEVICE 3\n");
                        if (((req->bmRequestType & 0x80) == 0)
                                        && (req->wIndex <= USB_NUM_OF_CFG_SETS)
                                        && (req->wValue == 0)
                                        && (req->wLength == 0) )
                        {
                                send_ep0_urb_to_host( NULL, 0 );;
                        }
                        break;

                default:
                        printk( "unknown device.\n" );
                        break;
                }
        }
        else if ( (req->bmRequestType&0x0F) == 0x01 ) 
        {
                switch ( req->wIndex ) {
                case 6:
                        PRINTKD( "INTERFACE 6\n");
                        if ( intf6.enable ) 
                                rc = process_interface6_request( req );
                        else /* This branch is not only for test. It is necessary. */
                                PRINTKD("interface6 is not enabled.");

                        break;
                case 8:
                        PRINTKD( "INTERFACE 8\n");
                        if ( intf8.enable ) 
                                rc = process_interface8_request( req );
                        else /* This branch is not only for test. It is necessary. */
                                PRINTKD("interface8 is not enabled.");

                        break;
                default:
                        PRINTKD( "unknown interface.\n" );
                        break;
                }

                PRINTKD("process vendor command OK!\n");
        }
        return rc;
}

/**
 * pst_recv_setup_irq - called with a control URB 
 *
 * Check if this is a setup packet, process the device request, put results
 * back into the urb and return zero or non-zero to indicate success (DATA)
 * or failure (STALL).
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
int pst_recv_setup_irq (struct usb_device_request *request)
{
        // handle USB Vendor Request (c.f. USB Spec table 9-2)
        if ((request->bmRequestType & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_VENDOR)
                return (ep0_process_vendor_request( request ));

        return 0;		
}

/*
 * send_urb()
 * data == data to send
 * urb == USB Data structure
 * len == bytes we actually have
 *
 */
static void send_ep0_urb_to_host( void * data, int len )
{
        //struct pst_private *pst = &pst_private;
        struct usb_function_instance *function;
        struct urb *urb;

        PRINTKD( "send_ep0_urb_to_host: bytes actual=%d\n", len);
        RETURN_IF(!(function = pst_function));

        RETURN_IF(!(urb = usbd_alloc_urb_ep0(function, len, NULL)));
        urb->actual_length = len;
	if (NULL != data)
        	memcpy(urb->buffer, data, len);
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Jordan : ZLP handle*/	
	if(0 == len || !(len % urb->bus->driver->maxpacketsize)) {
		urb->flags = USBD_URB_SENDZLP;
	}
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++End */
        PRINTKD("send out urb now! %p %p len=%d\n", urb, urb->buffer, urb->actual_length );
        RETURN_IF(!usbd_send_urb(urb));
        usbd_dealloc_urb(urb);
        return;
}

#if PST_BULKINTF

#if 0
/*
 * send_urb()
 * data == data to send
 * urb == USB Data structure
 * len == bytes we actually have
 *
 */
static void send_epx_urb_to_host( void * data, int len,int epidx )
{
        //struct pst_private *pst = &pst_private;
        struct usb_function_instance *function;
        struct urb *urb;

        PRINTKD( "send_epx_urb_to_host: bytes actual=%d epx=%d\n", len,epidx);
        RETURN_IF(!(function = pst_function));

        RETURN_IF(!(urb = usbd_alloc_urb(function,epidx,len,NULL)));
        urb->actual_length = len;
	if (NULL != data)
        	memcpy(urb->buffer, data, len);
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Jordan : ZLP handle*/	
	if(0 == len){
		urb->flags = USBD_URB_SENDZLP;
	}
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++End */
        PRINTKD("send out urb now! %p [%x %x %x %x] len=%d\n", urb, urb->buffer[0],urb->buffer[1], urb->buffer[2], urb->buffer[3],  urb->actual_length );
        RETURN_IF(!usbd_send_urb(urb));
        usbd_dealloc_urb(urb);
        return;
}
#endif

#endif

/**
 * pst_urb_sent - called to indicate URB transmit finished
 * @urb: pointer to struct urb
 * @rc: result
 *
 * The usb device core layer will use this to let us know when an URB has
 * been finished with.
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
int pst_urb_sent (struct urb *urb, int rc)
{
        urb->privdata = NULL;
        usbd_dealloc_urb (urb);

//        printk("\nw20535: pst_urb_sent: bplog=%d!!!!!\n", bplog);
        return 0;
}
#ifdef CONFIG_ARCH_EZX   //w20535
int pst_function_enable( struct usb_function_instance* function)
#else
static int pst_function_enable( struct usb_function_instance* function)
#endif
{
        printk(KERN_INFO"INIT**************************\n");

        pst_function = function;
        return 0;
}
#ifdef CONFIG_ARCH_EZX  //w20535
void pst_function_disable( struct usb_function_instance *function )
#else
static void pst_function_disable( struct usb_function_instance *function )
#endif
{
        printk(KERN_INFO"CLOSING************************\n");
        pst_function = NULL;
}

/***for PST***/
static struct usb_function_operations pst_function_ops = {
        event_irq: pst_event_irq, //recv_urb:pst_recv_urb,
        recv_setup_irq: pst_recv_setup_irq,
        function_enable: pst_function_enable,
        function_disable: pst_function_disable,
};

static struct usb_function_driver function_driver = {
        name:"pst",
        fops:&pst_function_ops,
        device_description:&pst_device_description,
        bNumConfigurations:sizeof (pst_description) / sizeof (struct usb_configuration_description),
        configuration_description:pst_description,
        idVendor: __constant_cpu_to_le16(CONFIG_USBD_PST_VENDORID),
        idProduct: __constant_cpu_to_le16(CONFIG_USBD_PST_PRODUCTID),
        bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_PST_BCDDEVICE),
};

/* Module init and exit *******************************************************************/
/*
 * pst_modinit - module init
 *
 */
static int pst_modinit (void)
{
        printk (KERN_INFO"init pst_mod\n");
        /*init ptf char device array */
        if ( 0 > ptfproto_modinit ("", PTF_MAX_DEV_NUMBER)) {
		printk(KERN_INFO "pst_modinit: ptfproto_modinit failed!\n");
                return -EINVAL;
	}
        // register us with the usb device support layer
        if (usbd_register_function (&function_driver)) {
                PRINTKD ("usbd_register_function failed\n");
                ptfproto_modexit();
                return -EINVAL;
        }

        return 0;
}

/*
 * function_exit - module cleanup
 *
 */
static void pst_modexit (void)
{
        PRINTKD ("exit pst_mod\n");
        // de-register us with the usb device support layer
        usbd_deregister_function (&function_driver);
        ptfproto_modexit();
}

module_init (pst_modinit);
module_exit (pst_modexit);
//EXPORT_SYMBOL(pst_modinit);
//EXPORT_SYMBOL(pst_modexit);
