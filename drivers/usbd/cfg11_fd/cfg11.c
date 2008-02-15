/*
 * usbd/cfg11_fd/cfg11.c
 *
 *
 *    	Author: 	Jordan Wang [w20535@motorola.com]
 *	Created: 	March 30, 2004
 *      Copyright:      2004-2005  Motorola
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
 * Note: this function driver is composite function which consists of 
 *	 acm interface, test command interface and MCU data logger interface. 
 *
 * 2005-07-26 Anthony Gao [w20101@motorola.com] 
 * modification -- extended config11 to contain five interfaces: 
 * modem, AP log, BP log, test command and PTF interface
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <usbd-export.h>
#include <usbd-build.h>

MODULE_AUTHOR("w20535@motorola.com");
MODULE_DESCRIPTION("Motorola cfg11 Function");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,17)
MODULE_LICENSE("GPL");
#endif


#define MCEL 1
#define CONFIG_USBD_ACM_DATALOG 1
#define PST_FD_AVAILABLE
#define USBD_CFG11_VENDORID	0x22b8
#define USBD_CFG11_PRODUCTID	0x6004	
#define USBD_CFG11_BCDDEVICE	0x0001


#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/smp_lock.h>
#include <linux/slab.h>

#include "../usbd-chap9.h"
#include "../usbd-mem.h"
#include "../usbd.h"
#include "../usbd-func.h"

#include "../usbd-bus.h"		// UDC
#include "../pst_fd/pst-usb.h"		//e12707 bulk testcmd support

USBD_MODULE_INFO("cfg11_fd 1.0-beta");


#define MAX_QUEUED_BYTES        256
#define MAX_QUEUED_URBS         10	// 64???

#define MAX_RECV_URBS           2      // Max for receiving data
#define RECV_RING_SIZE          (MAX_RECV_URBS+1)

/*
#ifndef CONFIG_USBD_MAXPOWER
#define CONFIG_USBD_MAXPOWER                    0
#endif
#undef BMAXPOWER
#define BMAXPOWER                               0x0a
*/

//Interface index for cfg11_fd
//commented out but kept it here for clarify what interfaces we are using.
//for real definition, please refer to acm.c and pst-fd.c
#define COMM_INTF       0x00
#define DATA_INTF       0x01
#define DATALOG_INTF    0x06
#define TESTCMD_INTF    0x08
#define AP_LOG_INTF    0x0d
#define PTF_INTF    0x0e

/*from pst-fd.c*/
extern int pst_dev_create(struct usb_function_instance *function);
extern void pst_dev_destroy(void);
extern void ep0_process_vendor_request(struct usb_device_request *req);
extern int pst_urb_sent (struct urb *, int );
extern int pst_function_enable(struct usb_function_instance *);
extern int pst_function_disable(struct usb_function_instance *);

extern struct usb_alternate_description datalog_alternate_descriptions[];
extern struct usb_alternate_description testcmd_alternate_descriptions[];
extern struct usb_alternate_description ap_log_alternate_descriptions[];
extern struct usb_alternate_description ptf_alternate_descriptions[];
extern struct usb_alternate_description pseudo_alternate_descriptions2[];
extern struct usb_alternate_description pseudo_alternate_descriptions3[];
extern struct usb_alternate_description pseudo_alternate_descriptions4[];
extern struct usb_alternate_description pseudo_alternate_descriptions5[];
extern struct usb_alternate_description pseudo_alternate_descriptions7[];
extern struct usb_alternate_description pseudo_alternate_descriptions9[];
extern struct usb_alternate_description pseudo_alternate_descriptionsA[];
extern struct usb_alternate_description pseudo_alternate_descriptionsB[];
extern struct usb_alternate_description pseudo_alternate_descriptionsC[];

extern void pst_event_irq (struct usb_function_instance *function, usb_device_event_t event, int data);

/* plan to put following into acm.h */
extern struct usb_alternate_description cdc_comm_alternate_descriptions[];
extern struct usb_alternate_description cdc_data_alternate_descriptions[];
extern void acm_event_irq (struct usb_function_instance *function, usb_device_event_t event, int data);
extern int acm_function_enable (struct usb_function_instance *function);
extern void acm_function_disable (struct usb_function_instance *function);
extern int acm_recv_setup_irq (struct usb_device_request *request);

/* Module Parameters */
static u32 vendor_id;
static u32 product_id;
static u32 max_queued_urbs = MAX_QUEUED_URBS;
static u32 max_queued_bytes = MAX_QUEUED_BYTES;

MODULE_PARM (vendor_id, "i");
MODULE_PARM (product_id, "i");
MODULE_PARM (max_queued_urbs, "i");
MODULE_PARM (max_queued_bytes, "i");

MODULE_PARM_DESC (vendor_id, "Device Vendor ID");
MODULE_PARM_DESC (product_id, "Device Product ID");
MODULE_PARM_DESC (max_queued_urbs, "Maximum TX Queued Urbs");
MODULE_PARM_DESC (max_queued_bytes, "Maximum TX Queued Bytes");

struct cfg11_private{
	struct usb_function_instance *function;
	int usb_driver_registered;
};
static struct cfg11_private cfg11_private;

/*
 * cfg11 Configuration
 *
 * Endpoint, Class, Interface, Configuration and Device Descriptors/Descriptions
 *
 */
//Endpoint indexes in cfg11_endpoint_requests[] and the endpoint map.
//commented out but kept it here for clarify what endpoints we are using.
//for real definition, please refer to acm.c and pst-fd.c
//#define CFG11_BULK_OUT        0x00// the first three is for acm
//#define CFG11_BULK_IN         0x01
//#define CFG11_INT_IN         0x02
//#define CFG11_DATALOG_IN_EP      0x03
//#define CFG11_AP_LOG_IN_EP       0x04
//#define CFG11_AP_LOG_OUT_EP      0x05
//#define CFG11_PTF_IN_EP          0x06
//#define CFG11_PTF_OUT_EP         0x07

//e12707 pst bulk support
#if PST_BULKINTF
#define CFG11_ENDPOINTS                 0x0A
#else
#define CFG11_ENDPOINTS                 0x08
#endif

/*interfaces*/
static struct usb_interface_description cfg11_interfaces[] = {
      { alternates:1,//sizeof (cdc_comm_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:cdc_comm_alternate_descriptions,
      },

      { alternates:1,//sizeof (cdc_data_alternate_descriptions) / sizeof (struct usb_alternate_description),
                alternate_list:cdc_data_alternate_descriptions,
      },

	{ alternates:1, alternate_list: pseudo_alternate_descriptions2, }, /* 0x2 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions3, }, /* 0x3 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions4, }, /* 0x4 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions5, }, /* 0x5 */

       { alternates:1, alternate_list:datalog_alternate_descriptions, },

	{ alternates:1, alternate_list: pseudo_alternate_descriptions7, }, /* 0x7 */

       { alternates:1, alternate_list:testcmd_alternate_descriptions, },

	{ alternates:1, alternate_list: pseudo_alternate_descriptions9, }, /* 0x9 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptionsA, }, /* 0xa */
	{ alternates:1, alternate_list: pseudo_alternate_descriptionsB, }, /* 0xb */
	{ alternates:1, alternate_list: pseudo_alternate_descriptionsC, }, /* 0xc */

       { alternates:1, alternate_list:ap_log_alternate_descriptions, }, /* 0xd */
       { alternates:1, alternate_list:ptf_alternate_descriptions, },/* 0xe */
};


/*Configuration Descriptor and Description*/
static __u8 cfg11_configuration_descriptor[sizeof(struct usb_configuration_descriptor)] = {
        0x09, USB_DT_CONFIG, 0x00, 0x00, sizeof (cfg11_interfaces) / sizeof (struct usb_interface_description),
        0x01, 0x00, BMATTRIBUTE, BMAXPOWER, };

struct usb_configuration_description cfg11_description[] = {
      { iConfiguration: "Motorola CDC Configuration",	// EZX
              configuration_descriptor: (struct usb_configuration_descriptor *)cfg11_configuration_descriptor,
	      bNumInterfaces:sizeof (cfg11_interfaces) / sizeof (struct usb_interface_description),
              interface_list:cfg11_interfaces,}, };

/*Device Descriptor*/
static struct usb_device_descriptor cfg11_device_descriptor = {
	bLength: sizeof(struct usb_device_descriptor),
	bDescriptorType: USB_DT_DEVICE,
	bcdUSB: __constant_cpu_to_le16(USB_BCD_VERSION),
	bDeviceClass: COMMUNICATIONS_DEVICE_CLASS,
	bDeviceSubClass: 0x02,	
	bDeviceProtocol: 0x00,
	bMaxPacketSize0: 0x00,
	idVendor: __constant_cpu_to_le16(USBD_CFG11_VENDORID),
	idProduct: __constant_cpu_to_le16(USBD_CFG11_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(USBD_CFG11_BCDDEVICE),
};

/*endpoint request map*/
static struct usb_endpoint_request cfg11_endpoint_requests[CFG11_ENDPOINTS+1] = {
        { 1, DATA_INTF, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, 64, 512, },
        { 1, DATA_INTF, 0, USB_DIR_IN | USB_ENDPOINT_BULK, 64, 512, },
        { 1, COMM_INTF, 0, USB_DIR_IN | USB_ENDPOINT_INTERRUPT, 16, 64, },

        { 1, DATALOG_INTF, 0, USB_DIR_IN | USB_ENDPOINT_BULK, 64, 64, },  // DATALOG_IN
        { 1, AP_LOG_INTF, 0, USB_DIR_IN | USB_ENDPOINT_BULK, 64, 64,  },  // AP_LOG_IN
        { 1, AP_LOG_INTF, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, 64, 64, }, // AP_LOG_OUT
        { 1, PTF_INTF, 0, USB_DIR_IN | USB_ENDPOINT_BULK, 64, 64, },  // PTF_IN
        { 1, PTF_INTF, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, 64, 64, }, // PTF_OUT

#if PST_BULKINTF
        { 1, TESTCMD_INTF, 0, USB_DIR_IN | USB_ENDPOINT_BULK, 64, 512, },  // TCMD
        { 1, TESTCMD_INTF, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, 64, 512, }, // TCMD
#endif
        { 0, },
};

struct usb_device_description cfg11_device_description = {
        device_descriptor: &cfg11_device_descriptor,
		iManufacturer: "Belcarra", //CONFIG_USBD_ACM_MANUFACTURER,
		iProduct: "Belcarra",
		  // "Motorola CDC Configuration", 
		  // //CONFIG_USBD_ACM_PRODUCT_NAME,
#if !defined(CONFIG_USBD_NO_SERIAL_NUMBER) && defined(CONFIG_USBD_SERIAL_NUMBER_STR)
	iSerialNumber: CONFIG_USBD_SERIAL_NUMBER_STR, 
#endif
        endpointsRequested: CFG11_ENDPOINTS,
        requestedEndpoints: cfg11_endpoint_requests,
};

void cfg11_event_irq (struct usb_function_instance *function, usb_device_event_t event, int data)
{
	pst_event_irq(function, event, data);
	acm_event_irq(function, event, data);
}

/*
 * cfg11_recv_setup_irq--called to indicate urb has been received
 */
 int cfg11_recv_setup_irq (struct usb_device_request *request)
{
     // verify that this is a usb class request per cdc-acm specification or a vendor request.
     if (!(request->bmRequestType & (USB_REQ_TYPE_CLASS | USB_REQ_TYPE_VENDOR))) {
//		TRACE_MSG("--> 0");
		return(0);
	}
     
    // determine the request direction and process accordingly
	switch (request->bmRequestType & (USB_REQ_DIRECTION_MASK | USB_REQ_TYPE_MASK)) {
	case USB_REQ_HOST2DEVICE | USB_REQ_TYPE_VENDOR: 	
        case USB_REQ_DEVICE2HOST | USB_REQ_TYPE_VENDOR: 	
        	ep0_process_vendor_request(request);
		return 0;
        default: break;
	}

    return acm_recv_setup_irq(request);
}
static int cfg11_function_enable(struct usb_function_instance *function)
{
	pst_function_enable(function);
//	TRACE_MSG("calling pst_function_enable ret!");

	return acm_function_enable(function);
}

static void cfg11_function_disable (struct usb_function_instance *function)
{
	  pst_function_disable(function);
//	  TRACE_MSG("calling pst_function_disable ret!");

	  return acm_function_disable(function);
}
static struct usb_function_operations cfg11_function_ops = {
	event_irq: cfg11_event_irq,
	recv_setup_irq: cfg11_recv_setup_irq,
        function_enable: cfg11_function_enable,
        function_disable: cfg11_function_disable,
};
static struct usb_function_driver cfg11_function_driver = {
	name: "cfg11",
	fops:&cfg11_function_ops,
	device_description:&cfg11_device_description,
	bNumConfigurations:sizeof (cfg11_description) / sizeof (struct usb_configuration_description),
	configuration_description:cfg11_description,
	idVendor: __constant_cpu_to_le16(USBD_CFG11_VENDORID),
	idProduct: __constant_cpu_to_le16(USBD_CFG11_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(USBD_CFG11_BCDDEVICE),
};

/*USB Module init/exit*/
/*
 *cfg11_modinit ---- module init
 */
 static int cfg11_modinit(void)
{

    THROW_IF (usbd_register_function (&cfg11_function_driver), error);
    cfg11_private.usb_driver_registered++;
    
    CATCH(error) {
          printk(KERN_ERR"%s: ERROR\n", __FUNCTION__);

          if (cfg11_private.usb_driver_registered) {
			usbd_deregister_function (&cfg11_function_driver);
            cfg11_private.usb_driver_registered = 0;
          }
//		  TRACE_MSG("--> -EINVAL");
          return -EINVAL;
        }
    return 0;
}
static void cfg11_modexit(void)
{
//	TRACE_MSG("cfg11 exit entered");
	if (cfg11_private.usb_driver_registered) {
		usbd_deregister_function (&cfg11_function_driver);
		cfg11_private.usb_driver_registered = 0;
	}
}
module_init (cfg11_modinit);
module_exit (cfg11_modexit);
