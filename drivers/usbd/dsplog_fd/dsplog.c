/*
 * linux/drivers/usbd/dsplog_fd/dsplog.c - Motorola DSP LOG USB function driver
 *
 *  Copyright (C) 2004 - Motorola
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
 * Motorola DSP LOG USB Function Driver is Created by: 
 *     Sun Bonnie (a5035c@motorola.com)
 *     Liu Weijie (a19553@motorola.com)
 *     02/25/2004
 */

#include <linux/config.h>
#include <linux/module.h>
#include "../usbd-build.h"
#include "../usbd-export.h"

MODULE_AUTHOR ("a5035c@Motorola.com");
MODULE_DESCRIPTION ("DSP Log Function Driver");

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
#include <linux/autoconf.h>

#include "../usbd-chap9.h"
#include "../usbd-mem.h"
#include "../usbd.h"
#include "../usbd-func.h"
#include "../pst_fd/pst-usb.h"

USBD_MODULE_INFO ("dsplog_usb 1.0");
#define VERSION "1.0"

#define DSP_DEBUG 1
#ifdef DSP_DEBUG
#define PRINTKD(fmt, args...) printk( fmt , ## args)
#else
#define PRINTKD(fmt, args...)
#endif

/*
 * setup some default values for pktsizes and endpoint addresses.
 */
#define USBC_MINOR_DSPLOG               20

#define TEST_CMD_PKTSIZE             	8
#define DATA_LOG_IN_PKTSIZE          	64
#define DSP_LOG_IN_PKTSIZE               64

#define DSP_LOG_MAX_SIZE  2048



static struct usb_function_instance * dsplog_function=NULL;

extern int cable_connected;
extern int pst_dev_create (struct usb_function_instance *function);
extern void pst_dev_destroy (void);
extern void  pst_recv_setup_irq(struct usb_device_request *request);
extern int  pst_function_enable(struct usb_function_instance* function);
extern void pst_function_disable(struct usb_function_instance *function);
extern struct usb_alternate_description datalog_alternate_descriptions[];
extern struct usb_alternate_description testcmd_alternate_descriptions[];

/*Local Function list*/
static int usbc_dsplog_open (struct inode *pInode, struct file *PFILE);
static ssize_t usbc_dsplog_write ( struct file *pFile, const char *pUserBuffer,
	                           size_t stCount, loff_t * pPos);
static int usbc_dsplog_release (struct inode *pInode, struct file *pFile);

static int dsplog_dev_create (struct usb_function_instance *function);
static void dsplog_dev_destroy (void);

static void dsplog_event_irq(struct usb_function_instance *function, usb_device_event_t event, int data);
static int dsplog_recv_setup_irq  (struct usb_device_request *request);
static int dsplog_urb_sent (struct urb *urb, int rc);
static int dsplog_function_enable( struct usb_function_instance *function );
static void dsplog_function_disable( struct usb_function_instance *function );




/* USB Interface and Endpoint Configuration Config 03 include three interface:
 *  Test Command
 *  Data Log (MCU)
 *  DSP  Log
 *  */

//new discription base on new usbd core 
// Endpoint indexes in pst_endpoint_requests[] and the endpoint map.

#define NDX_OF_DSP_LOG_IN_EP       0x01 //NDX_OF_DATALOG_IN_EP has been defined with 0x00 in PST
#define ENDPOINTS                  0x02
#define PST_DATA_CLASS 0xff
#define PST_DATA_SUBCLASS 0x02
#define PST_DATA_PROTOCOL 0xff

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

static struct usb_alternate_description pseudo_alternate_descriptions2[] = {
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

static struct usb_alternate_description pseudo_alternate_descriptions3[] = {
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

static struct usb_alternate_description pseudo_alternate_descriptions4[] = {
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

static struct usb_alternate_description pseudo_alternate_descriptions5[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor5,
		endpoints:0,
	},
};

// -- w20146 -end


/*
 *  DATA Log Interface(6) has been defined in PST  
 */

/*DSP Log Interface defince*/
static __u8 dsplog_endpoint[] = {
	0x07,                      /* bLength            */
	USB_DT_ENDPOINT,           /* bDescriptorType 0x5*/
	0x00,                      /* bEndpointAddress - filled in later*/
	BULK,                      /* bmAttributes       */
	0x00, 0x00,                /* wMaxPacketSize     */   
	0x00,                      /* bInterval          */
};


static  struct usb_endpoint_descriptor  *dsplog_endpoints[] = { (struct usb_endpoint_descriptor *) &dsplog_endpoint };
u8 dsplog_indexes[] = { NDX_OF_DSP_LOG_IN_EP, };


static __u8 dsplog_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
        0x09,              /* bLength               */
        USB_DT_INTERFACE,  /* bDescriptorType  0x04 */
        0x07,              /* bInterfaceNumber      */
        0x00,              /* bAlternateSetting     */
        0x00,              /* bNumEndpoints*/
        0xff,              /* bInterfaceClass       */
        0x02,              /* bInterfaceSubClass    */
        0xff,              /* bInterfaceProtocol*/
        0x00,              /* iInterface*/
};

static struct usb_alternate_description dsplog_alternate_descriptions[] = {
    {
 	iInterface:"Motorola DSP Logger",
	interface_descriptor: (struct usb_interface_descriptor *)&dsplog_alternate_descriptor,
	endpoints:sizeof (dsplog_endpoints) / sizeof (struct usb_endpoint_descriptor *),
	endpoint_list:dsplog_endpoints,
	endpoint_indexes:dsplog_indexes,
    },
};

/*
 * TestCmd Interface(8) Alternate descriptor and description (no endpoints) has been defined in PST.
 */


static struct usb_interface_description testcmd_data_dsp_log_interfaces[] = {

	{ alternates:1, alternate_list: pseudo_alternate_descriptions0, }, /* 0x0 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions1, }, /* 0x1 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions2, }, /* 0x2 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions3, }, /* 0x3 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions4, }, /* 0x4 */
	{ alternates:1, alternate_list: pseudo_alternate_descriptions5, }, /* 0x5 */

        {                                                  // 0x6
	  alternates:1,/*sizeof(datalog_alternate_descriptions) / sizeof(struct usb_alternate_description),*/
	  alternate_list:datalog_alternate_descriptions,
        },
        {                                                 //0x07
          alternates:1,/*sizeof (dsplog_alternate_descriptions) / sizeof (struct usb_alternate_description),*/
          alternate_list:dsplog_alternate_descriptions,
        },
        {                                                  // 0x8
	  alternates:1,/*sizeof(testcmd_alternate_descriptions) / sizeof(struct usb_alternate_description),*/
          alternate_list:testcmd_alternate_descriptions,
        },
	
};


/* Device Configuration description(s) */
static __u8 dsplog_configuration_descriptor[sizeof(struct usb_configuration_descriptor)] = {
        0x09,          /* bLength              */
        USB_DT_CONFIG, /* bDescriptorType  0x2 */
        0x00, 0x00,    /* wTotalLength*/
        sizeof (testcmd_data_dsp_log_interfaces) / sizeof (struct usb_interface_description),
                       /* bNumInterfaces       */
        0x01,          /* bConfigurationValue  */
        0x00,          /* iConfiguration       */
        0xc0,          /* bmAttributes  0xC0   */
        0x0a,          /* bMaxPower 0x0a*2 MA  */

};

/* 
 * Configuration description
 *  Attention:
 *  iConfiguration: Must same with PST configuration description, Otherwise RADIOCOM and RTA Tool
 *                 cann't detect the interface!!!
 * */
static struct usb_configuration_description dsplog_config_description[] = {
 {
  iConfiguration:"Motorola PTF AP BP Configuration", /*Must same with PST description!*/
  configuration_descriptor: (struct usb_configuration_descriptor *)dsplog_configuration_descriptor,
  bNumInterfaces:sizeof (testcmd_data_dsp_log_interfaces) / sizeof (struct usb_interface_description),
  interface_list:testcmd_data_dsp_log_interfaces,
 },
};

/*
 * DSPLog Device Description
 */
static struct usb_device_descriptor dsplog_device_descriptor = {
	bLength: sizeof(struct usb_device_descriptor),
	bDescriptorType: USB_DT_DEVICE,
	bcdUSB: __constant_cpu_to_le16(USB_BCD_VERSION),
	bDeviceClass: 0x00,
	bDeviceSubClass: 0x00,
	bDeviceProtocol: 0x00,
	bMaxPacketSize0: 0x00,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_DSP_LOG_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_DSP_LOG_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_DSP_LOG_BCDDEVICE),
};

static struct usb_endpoint_request dsplog_endpoint_requests[ENDPOINTS+1] = {
        { 1, 1, 0, USB_DIR_IN | USB_ENDPOINT_BULK, DATA_LOG_IN_PKTSIZE, DATA_LOG_IN_PKTSIZE, },  // DATALOG_IN
        { 1, 1, 0, USB_DIR_IN | USB_ENDPOINT_BULK, DSP_LOG_IN_PKTSIZE, DSP_LOG_IN_PKTSIZE,  },  // DSP_LOG_IN
        { 0, },
};



struct usb_device_description dsplog_device_description = {
	device_descriptor: &dsplog_device_descriptor,
	iManufacturer:"Motorola Inc.",
	iProduct:"Motorola Phone",
	iSerialNumber:"0",
	endpointsRequested: ENDPOINTS,
	requestedEndpoints: dsplog_endpoint_requests,
};


/* Test command, data command, data log device define in pst-fd.c*/
/* DSP Misc char device fileoperation structures */
static struct file_operations usbc_fop_dsplog = {
	owner:THIS_MODULE,
	open:usbc_dsplog_open,
	read:NULL,
	write:usbc_dsplog_write,
	poll:NULL,
	ioctl:NULL,
	release:usbc_dsplog_release,
};

static struct miscdevice usbc_misc_device_dsplog = {
	USBC_MINOR_DSPLOG, "dsplog", &usbc_fop_dsplog
};

static int usbc_dsplog_open (struct inode *pInode, struct file *pFile){	
	MOD_INC_USE_COUNT;
	return 0;
}

/*
 * Motorola DSP datalogger interfaces device driver
 *  USB interface 7 --- DSP Logger
 *  endpoints:ep8
 *  write :write dsp log data to host by ep8 bulk in transfer mode.
 */
static int normal_send( const char *buffer, int stCount)
{
        struct urb *urb;	
	if(!buffer){
	    printk(KERN_INFO "DSPLog: user buffer pointer is null\n");
	    return -EFAULT;
	}
	if (!(urb = usbd_alloc_urb (dsplog_function, NDX_OF_DSP_LOG_IN_EP, stCount, dsplog_urb_sent))) {
	       printk("\nDSPLog:urb alloc failed len: %d\n", stCount);
	       return -ENOMEM;
	}
	urb->actual_length = stCount;
        memcpy(urb->buffer, buffer, stCount);
        if (usbd_send_urb (urb)) {
               kfree(urb->buffer);
               kfree(urb);
	       urb=NULL;
               printk("\nDSPLog:usbd_send_urb failed\n");
               return -ECOMM;
        }
	udelay(500);
	PRINTKD("DSPLOG:Send URB Ok in normal_send:%d\n",stCount);
	return stCount;
}
static ssize_t usbc_dsplog_write (struct file *pFile, const char *pUserBuffer, size_t stCount, loff_t * pPos){
		
	int last_packet_len = 1, count = 0, count1 = 0;
	
	if( !cable_connected )
	{
	    return -2;
	}
	if(stCount <= 0 || stCount > DSP_LOG_MAX_SIZE )
	{
	    return 0;
	}
        /*We do so because the low layer driver has problem. it will send zero packet when x%64=0*/
	if(stCount%64){
	    return normal_send(pUserBuffer, stCount);
	}else {
            stCount--;
	    count = normal_send(pUserBuffer, stCount);
	    count1=normal_send(pUserBuffer+stCount,last_packet_len);
	    return (count+count1);
	}
}

/*  usbc_dsplog_release()
 *  Called when the dsplog char device close
 *  */
static int usbc_dsplog_release (struct inode *pInode, struct file *pFile){
	MOD_DEC_USE_COUNT;
	return 0;
}
 /* dsplog_dev_create - create an interface
 *  It inclued: test cmd device, data cmd device, data log device and dsp log device.
 *  dsp log char device create in this function, other devices are created in pst_dev_create().
 */
static int dsplog_dev_create (struct usb_function_instance *function){
	int rc;
        PRINTKD("DSPLOG:called pst_dev_create:0x%x\n",(unsigned int)dsplog_function);	
        rc = pst_dev_create(function);
        if (rc !=0){
             printk (KERN_INFO "DSP LOG:Couldn't register misc (Test cmd,Data Cmd,Data log).\n");
	     return -EBUSY;	
        }
        /* register DSP Log device driver */
	rc = misc_register (&usbc_misc_device_dsplog);
	PRINTKD("\nRegister DSP LOG Device\n");
	if (rc != 0)
	{
		printk (KERN_INFO "DSPLog:Couldn't register misc DSP LOG\n");
		pst_dev_destroy();
		return -EBUSY;
	}
	printk(KERN_INFO "USB Function Character DSP Log Driver Interface"
		" -%s, (C) 2004, Motorola Corporation.\n", VERSION);
	return 0;
}

/* dsplog_dev_destroy() 
 * pst_dev_destroy - destroy an interface
 *
 */
static void dsplog_dev_destroy (void)
{
     //  dsplog_function = NULL;
	misc_deregister(&usbc_misc_device_dsplog);
	pst_dev_destroy();
}

/*
 * dsplog_event - process a device event
 * @device: usb device 
 * @event: the event that happened
 *
 * Called by the usb device core layer to respond to various USB events.
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 */
static void dsplog_event_irq (struct usb_function_instance *function, usb_device_event_t event, int data){
	int ret;
	PRINTKD("\nDSPLOG: Receive IRPT Event\n");	
	switch (event) {
	case DEVICE_CONFIGURED:
		cable_connected = 1;
		PRINTKD("\nDSP LOG USB Device is configured.\n");
		break;	

	case DEVICE_CREATE:	
		PRINTKD("\nDSP LOG Misc Device is created.\n");
		ret = dsplog_dev_create (function);
		if(ret !=0){
		    PRINTKD("Create DSP LOG Device Failed!\n");
		    return;
		}
		break;
	case DEVICE_HUB_RESET:
		cable_connected = 0;
		break;

	case DEVICE_DESTROY:	
		PRINTKD("\nDSP Log Misc Device is destroyed.\n");
		dsplog_dev_destroy ();
		break;
        default:
		break;
	}
	return;
}
/**
 * dsplog_recv_setup_irq - called with a control URB 
 * @urb - pointer to struct urb
 *
 * Check if this is a setup packet, process the device request, put results
 * back into the urb and return zero or non-zero to indicate success (DATA)
 * or failure (STALL).
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 */
static int dsplog_recv_setup_irq (struct usb_device_request *request){
	PRINTKD("\nDSPLOG: Receive setup event and PST will handle it!\n");
	pst_recv_setup_irq(request);
	return 0;		
}


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
static int dsplog_urb_sent (struct urb *urb, int rc){
        PRINTKD("\nDSPLOG:urb sent ok.\n");
	urb->privdata = NULL;
	usbd_dealloc_urb (urb);
	return 0;
}


static int dsplog_function_enable( struct usb_function_instance* function)
{
        printk(KERN_INFO"DSPLog:INIT**************************\n");
        pst_function_enable(function);
        dsplog_function = function;
        return 0;
}

static void dsplog_function_disable( struct usb_function_instance *function )
{
        printk(KERN_INFO"DSPLog:CLOSING************************\n");
	pst_function_disable(function);
        dsplog_function = NULL;
}

/*USB Device Function driver define */
static struct usb_function_operations dsplog_function_ops = {
	event_irq:dsplog_event_irq,
	recv_setup_irq:dsplog_recv_setup_irq,
	function_enable:dsplog_function_enable,
	function_disable:dsplog_function_disable,
};

struct usb_function_driver dsplog_function_driver = {
	name:"dsplog",
	fops:&dsplog_function_ops,
	device_description:&dsplog_device_description,
	bNumConfigurations:sizeof (dsplog_config_description) / sizeof (struct usb_configuration_description),
	configuration_description:dsplog_config_description,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_DSP_LOG_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_DSP_LOG_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_DSP_LOG_BCDDEVICE),
};

/*
 * dsplog_modinit - module init
 *
 */
int dsplog_modinit (void){

	PRINTKD ("DSPLOG:init dsplog_mod\n");
        if (usbd_register_function (&dsplog_function_driver)) {
		PRINTKD ("usbd_register_function failed\n");
		return -EINVAL;
	}
	return 0;
}
/*
 *  dsplog_modexit  module cleanup
 *
 */
void dsplog_modexit (void){
       PRINTKD ("DSPLOG:exit dsplog_mod\n");
       usbd_deregister_function (&dsplog_function_driver);
}

module_init(dsplog_modinit);
module_exit(dsplog_modexit);
