/*
 *  /drivers/usbd/nm_fd/net_monitor.c - Motorola Netork Monitor USB function driver
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
 *  Motorola Network Monitor USB Function Driver is Created by:
 *     Liu Weijie (a19553@motorola.com)
 *     sun bonnie (a5035c@motorola.com)
 *     02/25/2004
 */

#include <linux/config.h>
#include <linux/module.h>

#include <usbd-export.h>
#include <usbd-build.h>

MODULE_AUTHOR ("a19553@motorola.com");

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION ("Motorola Network Monitor Function");

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

#include <usbd-chap9.h>
#include <usbd-mem.h>
#include <usbd.h>
#include <usbd-func.h>
#include "../pst_fd/pst-usb.h"


USBD_MODULE_INFO ("Mot Network Monitor 1.0");

#undef VERSION
#define VERSION "1.0"

#define DEBUG_NM 1

#ifdef DEBUG_NM
#define PRINTK(s...)  printk(s)
#else
#define PRINTK(s...)
#endif

#define ACM_COMM_INTF   0
#define ACM_DATA_INTF   1
#define NM_INTF         12

#define INDEX_OF_ACM_BULK_OUT        0x00
#define INDEX_OF_ACM_BULK_IN         0x01
#define INDEX_OF_ACM_INT_IN          0x04
#define INDEX_OF_NM_OUT_EP           0x02
#define INDEX_OF_NM_IN_EP            0x03   
#define ENDPOINTS                    0x05

#define USBC_NM_MINOR                16
#define NM_LOG_MAX_SIZE              4096
#define NM_LOG_MAX_REV_CMD           20
#define NM_OUT_PACKET_SIZE           128

DECLARE_WAIT_QUEUE_HEAD(wq_nm_read);

//extern struct usb_alternate_description cdc_comm_alternate_descriptions[];
//extern struct usb_alternate_description cdc_data_alternate_descriptions[];

extern void acm_event_irq(struct usb_function_instance *device, usb_device_event_t event,int data);
extern int acm_recv_setup_irq (struct usb_device_request *request);
extern int acm_function_enable (struct usb_function_instance *function);
extern void acm_function_disable (struct usb_function_instance *function);
#ifdef DEBUG_NM
extern int get_acm_interrupt_endpoint(void);
#endif


/* Local Function List */
static int nm_open(struct inode *inode, struct file *file);
static int nm_release(struct inode *inode, struct file *file);
static ssize_t nm_read (struct file *pFile, char *pUserBuffer, size_t stCount, loff_t * pPos);
static ssize_t nm_write(struct file *pFile, const char *pUserBuffer, size_t stCount, loff_t *pPos);
static int nm_dev_create (struct usb_function_instance *function);
static void nm_dev_destroy(void);

static void nm_event_irq (struct usb_function_instance *function, usb_device_event_t event, int data);
static int nm_recv_urb (struct urb *urb,int rc);
static int nm_recv_setup_irq (struct usb_device_request *request);
static int nm_urb_sent (struct urb *urb, int rc);
static int nm_function_enable ( struct usb_function_instance *function);
static void nm_function_disable (struct usb_function_instance *function); 

/*Local Var Define*/
static struct usb_function_instance * nm_function=NULL;
static int nm_cable_connected = 0;
static int usbc_nm_ref_count = 0;
static interface_t nm_msg; /*The Network Monitor Receive message buffer list, the type define in pst-usb.h.*/

/*
 *  Network Monitor Configuration
 * 
 *  Endpoint, Class, Interface, Configuration and Device descriptors/descriptions
 *   
 */

static __u8 acm_comm_1[] = { 0x07, USB_DT_ENDPOINT, IN,  INTERRUPT, 0, 0x00, 0x0a, };
static struct usb_endpoint_descriptor *acm_comm_endpoints[] = { (struct usb_endpoint_descriptor *) acm_comm_1 };

static u8 acm_comm_indexes[] = { INDEX_OF_ACM_INT_IN, };

static __u8 cdc_class_1[] = { 0x05, CS_INTERFACE, USB_ST_HEADER, 0x01, 0x01, /* CLASS_BDC_VERSION, CLASS_BDC_VERSION */ };
static __u8 cdc_class_2[] = { 0x05, CS_INTERFACE, USB_ST_CMF, 0x03, 0x01, /* bMasterInterface: 0, bSlaveInterface: 1 */ };
static __u8 cdc_class_3[] = { 0x05, CS_INTERFACE, USB_ST_UF, 0x00, 0x01, /* bMasterInterface: 0, bSlaveInterface: 1 */ };
static __u8 cdc_class_4[] = { 0x04, CS_INTERFACE, USB_ST_ACMF, 0x02, };

static struct usb_generic_class_descriptor *cdc_comm_class_descriptors[] =
        { (struct usb_generic_class_descriptor *) cdc_class_1,
          (struct usb_generic_class_descriptor *) cdc_class_2,
          (struct usb_generic_class_descriptor *) cdc_class_3,
          (struct usb_generic_class_descriptor *) cdc_class_4, };


static __u8 cdc_comm_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
         0x09, USB_DT_INTERFACE, ACM_COMM_INTF, 0x00, 0x00, // bInterfaceNumber, bAlternateSetting, bNumEndpoints
         COMMUNICATIONS_INTERFACE_CLASS, COMMUNICATIONS_ACM_SUBCLASS, 0x01, 0x00, };


static struct usb_alternate_description cdc_comm_alternate_descriptions[] = {
              {  iInterface: CONFIG_USBD_ACM_COMM_INTF,
                 interface_descriptor: (struct usb_interface_descriptor *)&cdc_comm_alternate_descriptor,
                 classes:sizeof (cdc_comm_class_descriptors) / sizeof (struct usb_generic_class_descriptor *),
                 class_list: cdc_comm_class_descriptors,
                 endpoint_list: acm_comm_endpoints,
                 endpoints:sizeof (acm_comm_endpoints) / sizeof(struct usb_endpoint_descriptor *),
                 endpoint_indexes: acm_comm_indexes,
              }, };

static __u8 acm_alt_1[] = { 0x07, USB_DT_ENDPOINT, OUT, BULK,      0, 0x00, 0x00, };
static __u8 acm_alt_2[] = { 0x07, USB_DT_ENDPOINT, IN,  BULK,     0,  0x00, 0x00, };
static struct usb_endpoint_descriptor *acm_alt_endpoints[] = {
                (struct usb_endpoint_descriptor *) acm_alt_1,
                (struct usb_endpoint_descriptor *) acm_alt_2, };
static u8 acm_alt_indexes[] = { INDEX_OF_ACM_BULK_OUT, INDEX_OF_ACM_BULK_IN, };

static __u8 cdc_data_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
                0x09, USB_DT_INTERFACE, ACM_DATA_INTF, 0x00, 0x00, // bInterfaceNumber, bAlternateSetting, bNumEndpoints
                DATA_INTERFACE_CLASS, COMMUNICATIONS_NO_SUBCLASS, COMMUNICATIONS_NO_PROTOCOL, 0x00, };

static struct usb_alternate_description cdc_data_alternate_descriptions[] = {
              {    iInterface: CONFIG_USBD_ACM_DATA_INTF,
                   interface_descriptor: (struct usb_interface_descriptor *)&cdc_data_alternate_descriptor,
                   endpoint_list: acm_alt_endpoints,
                   endpoints:sizeof (acm_alt_endpoints) / sizeof(struct usb_endpoint_descriptor *),
                   endpoint_indexes: acm_alt_indexes,
              }, };


static __u8 pseudo_alternate_descriptor2[sizeof(struct usb_interface_descriptor)] = {
	0x09,              /* bLength */
	USB_DT_INTERFACE,  /* bDescriptorType 0x04*/
	0x02,              /* bInterfaceNumber */
	0x00,              /* bAlternateSetting */
	0x00,              /* bNumEndpoints */
	0xFF,              /* bInterfaceClass */
	0x08,              /* bInterfaceSubClass */
	0x00,              /* bInterfaceProtocol */
	0x00,              /* iInterface */
};

static struct usb_alternate_description pseudo_alternate_descriptions2[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor2,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor3[sizeof(struct usb_interface_descriptor)] = {
	0x09,              /* bLength */
	USB_DT_INTERFACE,  /* bDescriptorType 0x04*/
	0x03,              /* bInterfaceNumber */
	0x00,              /* bAlternateSetting */
	0x00,              /* bNumEndpoints */
	0xFF,              /* bInterfaceClass */
	0x08,              /* bInterfaceSubClass */
	0x00,              /* bInterfaceProtocol */
	0x00,              /* iInterface */
};

static struct usb_alternate_description pseudo_alternate_descriptions3[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor3,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor4[sizeof(struct usb_interface_descriptor)] = {
	0x09,              /* bLength */
	USB_DT_INTERFACE,  /* bDescriptorType 0x04*/
	0x04,              /* bInterfaceNumber */
	0x00,              /* bAlternateSetting */
	0x00,              /* bNumEndpoints */
	0xFF,              /* bInterfaceClass */
	0x08,              /* bInterfaceSubClass */
	0x00,              /* bInterfaceProtocol */
	0x00,              /* iInterface */
};

static struct usb_alternate_description pseudo_alternate_descriptions4[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor4,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor5[sizeof(struct usb_interface_descriptor)] = {
	0x09,              /* bLength */
	USB_DT_INTERFACE,  /* bDescriptorType 0x04*/
	0x05,              /* bInterfaceNumber */
	0x00,              /* bAlternateSetting */
	0x00,              /* bNumEndpoints */
	0xFF,              /* bInterfaceClass */
	0x08,              /* bInterfaceSubClass */
	0x00,              /* bInterfaceProtocol */
	0x00,              /* iInterface */
};

static struct usb_alternate_description pseudo_alternate_descriptions5[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor5,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor6[sizeof(struct usb_interface_descriptor)] = {
	0x09,              /* bLength */
	USB_DT_INTERFACE,  /* bDescriptorType 0x04*/
	0x06,              /* bInterfaceNumber */
	0x00,              /* bAlternateSetting */
	0x00,              /* bNumEndpoints */
	0xFF,              /* bInterfaceClass */
	0x08,              /* bInterfaceSubClass */
	0x00,              /* bInterfaceProtocol */
	0x00,              /* iInterface */
};

static struct usb_alternate_description pseudo_alternate_descriptions6[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor6,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor7[sizeof(struct usb_interface_descriptor)] = {
	0x09,              /* bLength */
	USB_DT_INTERFACE,  /* bDescriptorType 0x04*/
	0x07,              /* bInterfaceNumber */
	0x00,              /* bAlternateSetting */
	0x00,              /* bNumEndpoints */
	0xFF,              /* bInterfaceClass */
	0x08,              /* bInterfaceSubClass */
	0x00,              /* bInterfaceProtocol */
	0x00,              /* iInterface */
};

static struct usb_alternate_description pseudo_alternate_descriptions7[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor7,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor8[sizeof(struct usb_interface_descriptor)] = {
	0x09,              /* bLength */
	USB_DT_INTERFACE,  /* bDescriptorType 0x04*/
	0x08,              /* bInterfaceNumber */
	0x00,              /* bAlternateSetting */
	0x00,              /* bNumEndpoints */
	0xFF,              /* bInterfaceClass */
	0x08,              /* bInterfaceSubClass */
	0x00,              /* bInterfaceProtocol */
	0x00,              /* iInterface */
};

static struct usb_alternate_description pseudo_alternate_descriptions8[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor8,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor9[sizeof(struct usb_interface_descriptor)] = {
	0x09,              /* bLength */
	USB_DT_INTERFACE,  /* bDescriptorType 0x04*/
	0x09,              /* bInterfaceNumber */
	0x00,              /* bAlternateSetting */
	0x00,              /* bNumEndpoints */
	0xFF,              /* bInterfaceClass */
	0x08,              /* bInterfaceSubClass */
	0x00,              /* bInterfaceProtocol */
	0x00,              /* iInterface */
};

static struct usb_alternate_description pseudo_alternate_descriptions9[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor9,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor10[sizeof(struct usb_interface_descriptor)] = {
	0x09,              /* bLength */
	USB_DT_INTERFACE,  /* bDescriptorType 0x04*/
	0x0a,              /* bInterfaceNumber */
	0x00,              /* bAlternateSetting */
	0x00,              /* bNumEndpoints */
	0xFF,              /* bInterfaceClass */
	0x08,              /* bInterfaceSubClass */
	0x00,              /* bInterfaceProtocol */
	0x00,              /* iInterface */
};

static struct usb_alternate_description pseudo_alternate_descriptions10[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor10,
		endpoints:0,
	},
};

static __u8 pseudo_alternate_descriptor11[sizeof(struct usb_interface_descriptor)] = {
	0x09,              /* bLength */
	USB_DT_INTERFACE,  /* bDescriptorType 0x04*/
	0x0b,              /* bInterfaceNumber */
	0x00,              /* bAlternateSetting */
	0x00,              /* bNumEndpoints */
	0xFF,              /* bInterfaceClass */
	0x08,              /* bInterfaceSubClass */
	0x00,              /* bInterfaceProtocol */
	0x00,              /* iInterface */
};

static struct usb_alternate_description pseudo_alternate_descriptions11[] = {
	{
		iInterface:"Pseudo",
		interface_descriptor: (struct usb_interface_descriptor *)&pseudo_alternate_descriptor11,
		endpoints:0,
	},
};


static __u8 nm_endpoint_in[] = {
	0x07,                      /* bLength             */
	USB_DT_ENDPOINT,           /* bDescriptorType 0x5 */
	IN,     /* bEndpointAddress*///address will be alloced orderly by the lower layer accoding to endpoint index and endponits_requst array
	BULK,                      /* bmAttributes        */
	0x00, 0x00,                /* wMaxPacketSize      */
	0x00                       /* bInterval           */
};

static __u8 nm_endpoint_out[] = {
	0x07,                      /* bLength             */
	USB_DT_ENDPOINT,           /* bDescriptorType 0x5 */
    OUT,                       /* bEndpointAddress    */
	BULK,                      /* bmAttributes        */
	0x00, 0x00,                /* wMaxPacketSize 64   */
	0x00                       /* bInterval           */   
};


static  struct usb_endpoint_descriptor  *nm_endpoints[] = { 
       (struct usb_endpoint_descriptor *) &nm_endpoint_out,
       (struct usb_endpoint_descriptor *) &nm_endpoint_in  
};
static u8 nmlog_indexes[] = {INDEX_OF_NM_OUT_EP , INDEX_OF_NM_IN_EP};

static __u8 nm_alternate_descriptor[sizeof(struct usb_interface_descriptor)] = {
   0x09,                          /* bLength*/
   USB_DT_INTERFACE,              /* bDescriptorType 0x04*/
   NM_INTF,                       /* bInterfaceNumber   */
   0x00,                          /* bAlternateSetting  */
   sizeof (nm_endpoints) / sizeof(struct usb_endpoint_descriptor *), /* bNumEndpoints*/
   0xFF,                          /* bInterfaceClass    */
   0x08,                          /* bInterfaceSubClass */
   0,                             /* bInterfaceProtocol */
   0x1A                           /* iInterface         */
};

static struct usb_alternate_description nm_alternate_descriptions[] = {
  {  
     iInterface:"Motorola Network Monitor",
     interface_descriptor:(struct usb_interface_descriptor *)&nm_alternate_descriptor,
     endpoint_list: nm_endpoints,
     endpoints:sizeof (nm_endpoints) / sizeof (struct usb_endpoint_descriptor *),
     endpoint_list: nm_endpoints,
     endpoint_indexes: nmlog_indexes
  }              
};

static struct usb_interface_description nm_interfaces[] = {
  {      
    alternates:1,/*sizeof(cdc_comm_alternate_descriptions)/sizeof(struct usb_alternate_description),*/
    alternate_list: cdc_comm_alternate_descriptions
  },
  { 
    alternates:1,/*sizeof (cdc_data_alternate_descriptions) / sizeof (struct usb_alternate_description),*/
    alternate_list: cdc_data_alternate_descriptions,
  },

  { alternates:1, alternate_list: pseudo_alternate_descriptions2, }, /* 0x2 */
  { alternates:1, alternate_list: pseudo_alternate_descriptions3, }, /* 0x3 */
  { alternates:1, alternate_list: pseudo_alternate_descriptions4, }, /* 0x4 */
  { alternates:1, alternate_list: pseudo_alternate_descriptions5, }, /* 0x5 */
  { alternates:1, alternate_list: pseudo_alternate_descriptions6, }, /* 0x6 */
  { alternates:1, alternate_list: pseudo_alternate_descriptions7, }, /* 0x7 */
  { alternates:1, alternate_list: pseudo_alternate_descriptions8, }, /* 0x8 */
  { alternates:1, alternate_list: pseudo_alternate_descriptions9, }, /* 0x9 */
  { alternates:1, alternate_list: pseudo_alternate_descriptions10, }, /* 0xa */
  { alternates:1, alternate_list: pseudo_alternate_descriptions11, }, /* 0xb */

  { 
     alternates:sizeof (nm_alternate_descriptions) / sizeof (struct usb_alternate_description),
     alternate_list: nm_alternate_descriptions,
  } 
};


static __u8 nm_configuration_descriptor[sizeof(struct usb_configuration_descriptor)] = {
  0x09,          /* bLength              */
  USB_DT_CONFIG, /* bDescriptorType  0x2 */
  0x00,0x00,     /* wTotalLength*/
  sizeof(nm_interfaces)/sizeof(struct usb_interface_description), /* bNumInterfaces  */
  0x01,          /* bConfigurationValue  */
  0x19,          /* iConfiguration       */
  0xC0,          /* bmAttributes  0xC0   */
  0x0A           /* bMaxPower 0x0a*2 MA  */
};

struct usb_configuration_description nm_configuration_description[] = {
  {
     iConfiguration: "Motorola Network Monitor Configuration",
     configuration_descriptor: (struct usb_configuration_descriptor *)nm_configuration_descriptor,
     bNumInterfaces: sizeof (nm_interfaces) / sizeof (struct usb_interface_description),
     interface_list: nm_interfaces
     /*interface_no_renumber: 1*/
  }
};

static struct usb_endpoint_request nm_endpoint_requests[ENDPOINTS+1] = {
  { 1, 1, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, 64, 512 }, /*ACM Data OUT*/
  { 1, 1, 0, USB_DIR_IN  | USB_ENDPOINT_BULK, 64, 512 }, /*ACM Data IN*/
  { 1, 12, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, 64, 64 }, /*NM Out Endpoint*/
  { 1, 12, 0, USB_DIR_IN  | USB_ENDPOINT_BULK, 64, 64 }, /*NM in Endpoint*/
  { 1, 0, 0, USB_DIR_IN  | USB_ENDPOINT_INTERRUPT, 16, 64 }, /*ACM control IN*/
  { 0, }
};

static struct usb_device_descriptor nm_device_descriptor = {
  bLength: sizeof(struct usb_device_descriptor), 
  bDescriptorType:USB_DT_DEVICE,   
  bcdUSB: __constant_cpu_to_le16(USB_BCD_VERSION), 
  bDeviceClass: 0xFF,            
  bDeviceSubClass:0x00,          
  bDeviceProtocol:0x00,          
  bMaxPacketSize0:0x00,          
  idVendor: __constant_cpu_to_le16(CONFIG_USBD_NM_VENDORID),
  idProduct: __constant_cpu_to_le16(CONFIG_USBD_NM_PRODUCTID),
  bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_NM_BCDDEVICE)
};

struct usb_device_description nm_device_description = {
    device_descriptor: &nm_device_descriptor,
    iManufacturer:"Belcarra",// iManufacturer:"Motorola Inc."
    iProduct:"Belcarra",//iProduct:"Motorola Phone (EZX)"
    iSerialNumber:0x00,
    endpointsRequested: ENDPOINTS,
    requestedEndpoints: nm_endpoint_requests
};

struct usb_function_operations function_ops_nm = {
  event_irq: nm_event_irq,
  recv_setup_irq: nm_recv_setup_irq,
  function_enable: nm_function_enable,
  function_disable: nm_function_disable
};

struct usb_function_driver nm_function_driver = {
  name:"NM",//must match with the define in  usbd_funcmame_table[] in usbd.c
  fops:&function_ops_nm,
  device_description:&nm_device_description,
  bNumConfigurations:sizeof(nm_configuration_description)/sizeof(struct usb_configuration_description),
  configuration_description: nm_configuration_description,
  idVendor: __constant_cpu_to_le16(CONFIG_USBD_NM_VENDORID),
  idProduct: __constant_cpu_to_le16(CONFIG_USBD_NM_PRODUCTID),
  bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_NM_BCDDEVICE)
};

static struct file_operations usbc_nm_fops = {
  owner: THIS_MODULE,
  open: nm_open,
  read: nm_read,
  write: nm_write,
  poll: NULL,
  ioctl: NULL,
  release: nm_release
};

static struct miscdevice usbc_misc_nm_device = {
    USBC_NM_MINOR, "nm", &usbc_nm_fops
};
/*
 * init_nm_msg_list()
 * Init Netowrk Monditor Receive Message list queue.
 * */
static int init_nm_msg_list (interface_t * msg_list){
    /*The caller must assure the msg_list isn't null.*/
    msg_list->enable = 0;
    msg_list->list_len = 0;
    INIT_LIST_HEAD (&msg_list->out_msg_list);
    return 0;
}
/*
 * dump_nm_msg_list()
 * destroy Netowrk Monditor Receive Message list queue.
 * */
static int dump_nm_msg_list(interface_t * msg_list){
    struct list_head *ptr;
    msg_node_t *node;

    PRINTK("\nNetMonitor: Dump msg list Totall:%d \n",msg_list->list_len);
    while (!list_empty (&msg_list->out_msg_list)){
       ptr = msg_list->out_msg_list.next;
       node = list_entry ( ptr, msg_node_t, list);
       if (node) {
          kfree (node->body);
	  node->body = NULL;
	  kfree (node); 
       }
       list_del (ptr);
    }
    msg_list->list_len = 0;
    PRINTK("\nNetMonitor: Dump msg finished!\n");
    return 0;
}
/*
 * nm_open()
 * Open Network Monitor char device.
 * */
static int nm_open(struct inode *inode, struct file *file){
    if (usbc_nm_ref_count == 0){
	init_nm_msg_list(&nm_msg);
	nm_msg.enable = 1;
    }
    usbc_nm_ref_count++;
    MOD_INC_USE_COUNT;
    PRINTK("\nNetMonitor:NM Device Opened successfully!\n");
    return 0;
}
/*
 * int nm_release()
 * Release Network Monitor char device.
 *
 * */
static int nm_release(struct inode *inode, struct file *file){
    usbc_nm_ref_count--;
    if (!usbc_nm_ref_count){
       dump_nm_msg_list(&nm_msg);
       nm_msg.enable = 0;	
    }
    MOD_DEC_USE_COUNT;
    PRINTK("NetMonitor:NM Device Closed successfully!\n");
    return 0;
}
/*
 * nm_read()
 *  Network monitor char device standard read fucntion.
 * Read data from Network Monitor recieve Message queue and return data to user.
 *
 * */
static ssize_t nm_read (struct file *pFile, char *pUserBuffer, size_t stCount, loff_t * pPos){
    int ret;
    struct list_head* ptr=NULL;
    msg_node_t *node;
    
    if (!pUserBuffer){
    	printk ( KERN_INFO "NetMonitor:user pbuffer pointer is null!\n");
    	return -EFAULT;
    }
    while (list_empty (&nm_msg.out_msg_list)) {
    	if (pFile->f_flags & O_NONBLOCK) {
    	   printk ( KERN_INFO "No data in Network Monitor msg list now!\n");
    	   return -EAGAIN;
    	}
    	PRINTK("NetMonitor:\"%s\" reading: going to sleep\n", current->comm);
    	interruptible_sleep_on(&wq_nm_read);
    	if (signal_pending(current))
    	   return -ERESTARTSYS;
    }
    
    ptr = nm_msg.out_msg_list.next;
    node = list_entry (ptr, msg_node_t, list);
    ret = node->size;
    if (stCount <= ret){
    	ret = stCount;
    }
    if ( copy_to_user (pUserBuffer, node->body, ret) ){
    	printk ( KERN_INFO " copy_to_user() has some problems! node->size=%d\n", node->size);
    	return -EFAULT;
    }
    list_del (ptr);
    nm_msg.list_len--;

    kfree(node->body);
    node->body = NULL;
    kfree (node);
    return ret;
}
/*
 * nm_normal_write()
 * Send out the data via low layer usb driver.
 * */
static int nm_normal_write( const char *buffer, int stCount)
{
    struct usb_function_instance *function = nm_function;
    struct urb *urb;
    
    if( !nm_cable_connected )
    {
        return -2;
    }
    if (!function){
    	printk ( KERN_INFO "NetMonitor:nm_function is Null!\n");
    	return -EFAULT;
    }
    if (!buffer){
    	printk ( KERN_INFO "NetMonitor:user buffer pointer is null!\n");
    	return -EFAULT;
    }
    if (!(urb = usbd_alloc_urb (function, INDEX_OF_NM_IN_EP, stCount,nm_urb_sent))) {
    	printk("\nurb alloc failed len: %d\n", stCount);
    	return -ENOMEM;
    }
    urb->actual_length = stCount;
    if ( copy_from_user (urb->buffer, buffer, stCount) ){
    	urb->privdata = NULL;
       	usbd_dealloc_urb (urb);
        printk ( KERN_INFO "copy_from_user() failed. stCount=%d\n", stCount);
    	return 0;        
    }
    if (usbd_send_urb(urb)) {
        urb->privdata = NULL;
       	usbd_dealloc_urb (urb);	
       	printk("\nusbd_send_urb failed\n");
    	return -ECOMM;
    }
    udelay(500);
    return stCount;
}
/*
 * nm_write()
 * Network monitor char device standard write fucntion.
 * Send out the data via low layer usb driver.
 * */
static ssize_t nm_write(struct file *pFile, const char *pUserBuffer, size_t stCount, loff_t *pPos){
    int count =0;

    if( !nm_cable_connected )
    {
       return -2;
    }
    if(stCount <= 0 || stCount > NM_LOG_MAX_SIZE )
    {
        return 0;
    }
    count = nm_normal_write(pUserBuffer, stCount);
    if ( count>0 )
       *pPos = count;
    return count;	
}
/*
 * int nm_dev_create(...)
 * Create NM device
 * */
static int nm_dev_create (struct usb_function_instance *function){
   int rc;
   /* register device driver */
   rc = misc_register (&usbc_misc_nm_device);
   PRINTK("\nNetmonitor:Register NetMonitor USB Device\n");
   if (rc != 0)
   {
       printk (KERN_INFO "Couldn't register misc device(nm).\n");
       return -EBUSY;
   }
   PRINTK (KERN_INFO "NetMonitor USB Function Character Driver Interface"
	        " -%s, (C) 2004, Motorola Corporation.\n", VERSION);
   return 0;
}
static void nm_dev_destroy(void){
   misc_deregister (&usbc_misc_nm_device);
}

/**
 * nm_event_irq - process a device event
 * @device: usb device 
 * @event: the event that happened
 *
 * Called by the usb device core layer to respond to various USB events.
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
static void nm_event_irq(struct usb_function_instance *function,usb_device_event_t event,int data){
    struct urb *urb;	
    switch (event) {
        case DEVICE_CREATE:
	     PRINTK("Netmonitor:NM Device Create!\n");
             if(nm_dev_create(function) != 0) 
                PRINTK("Netmonitor: Create NM USB Device failed!\n");
             return;
             break;
        case DEVICE_DESTROY:
	     PRINTK("NetMonitor:Device Destoried");
             nm_dev_destroy();
	     return;
             break;   
        case DEVICE_CONFIGURED:
             nm_cable_connected = 1;
	         PRINTK("Netmonitor: NM Cable connect!\n");
             urb = usbd_alloc_urb(function,INDEX_OF_NM_OUT_EP, NM_OUT_PACKET_SIZE,nm_recv_urb);
	         if (!urb)
	         return;
             if (usbd_start_recv(urb)) 
                  usbd_dealloc_urb(urb);
             PRINTK("Netmonitor: NM Device Configured!\n");
	     
             break;
             
        case DEVICE_RESET:
        case DEVICE_HUB_RESET://add by bonnie
             PRINTK("NetMonitor:Device hub reset and remove the cable connect!\n");
             nm_cable_connected = 0;
             break;
        
        case DEVICE_DE_CONFIGURED:
             PRINTK("NetMonitor:Cable Removed!\n");
             nm_cable_connected = 0;
             break;
    
        default:
             break;
    }
    /*Let ACM continuce handle the Message.*/
    acm_event_irq (function, event, data);
}
/**
 * nm_recv_urb - called with a received URB 
 * @urb - pointer to struct urb
 *
 * Return non-zero if we failed and urb is still valid (not disposed)
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
static int nm_recv_urb (struct urb *urb,int rc){
    
    int count = urb->actual_length;
    u8 *data;
    msg_node_t *node;
    struct list_head* ptr=NULL;
    if (RECV_OK != rc) {
	   printk("NetMonitor: rc !=RECV_OK");
	   return (1);
    }
    if ( nm_msg.enable == 0){
        printk("Netmonitor: nm_msg not init!\n");
        usbd_start_recv(urb); /*Reject the received urb*/
        return 0;   
    }
    /* If the recieve packets are more than NM_LOG_MAX_REV_CMD, destroy the oldest node.*/
    if (nm_msg.list_len > NM_LOG_MAX_REV_CMD){
        ptr = nm_msg.out_msg_list.next;
        node = list_entry (ptr, msg_node_t, list);	       
        list_del (ptr);
        nm_msg.list_len--;
        kfree(node->body);
        node->body = NULL;
        kfree (node);    
        node = NULL;
        PRINTK("\nNetMonitor:Delete the oldest data!\n");
    }
    data = (u8 *) kmalloc (count, GFP_ATOMIC);
    if (!data)
    {
        printk( KERN_INFO " kmalloc(%d) out of memory!\n", count);
        return -ENOMEM;
    }
    memcpy(data, urb->buffer, count); 
    node = (msg_node_t *) kmalloc (sizeof (msg_node_t), GFP_ATOMIC);
    if (!node)
    {
 	printk( KERN_INFO "kmalloc(%d) out of memory!\n", sizeof (msg_node_t));
 	kfree(data);
 	data = NULL;
 	return -ENOMEM;
    }
    node->size = count;
    node->body = data; 
    list_add_tail (&node->list, &nm_msg.out_msg_list);
    nm_msg.list_len++;
    
    wake_up_interruptible(&wq_nm_read);
    
    if (usbd_start_recv (urb)){
    	printk("\nNetMonitor:usbd_start_recv() error happen!\n");
       	usbd_dealloc_urb(urb);
	return (-EINVAL);
    }
    
#ifdef DEBUG_NM
    int n =0; 
    for (n=0;n<count; n++){
       PRINTK("n:%d 0x%x\n",n,*(data+n));
    }
#endif	    
    
    return 0;
}

/**
 * nm_recv_setup_irq - called with a control URB 
 * @urb - pointer to struct urb
 *
 * Check if this is a setup packet, process the device request, put results
 * back into the urb and return zero or non-zero to indicate success (DATA)
 * or failure (STALL).
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
static int nm_recv_setup_irq (struct usb_device_request *request){
    /*Net Monitor don't need handle the setup message.*/
    return acm_recv_setup_irq(request);
}
/**
 * nm_urb_sent - called to indicate URB transmit finished
 * @urb: pointer to struct urb
 * @rc: result
 *
 * The usb device core layer will use this to let us know when an URB has
 * been finished with.
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
static int nm_urb_sent(struct urb *urb, int rc)
{
    urb->privdata = NULL;
    usbd_dealloc_urb (urb);
    PRINTK("NetMonitor: URB Sent!\n");
    return 0;
}
/*
 * int nm_function_enable()
 * Network Moniotr do nothing, call acm acm_function_init();
*/
static int nm_function_enable ( struct usb_function_instance *function){
    
    PRINTK("NetMonitor:nm_function_enable !\n");
    nm_function = function;//add by bonnie
    acm_function_enable(function); 
    return 0;
}  
/*
*  void nm_function_exit()
*  Do nothing, call acm_function_exit();
*/                            
static void nm_function_disable (struct usb_function_instance *function){
    PRINTK("NetMonitor:nm_function_exit !\n");
    nm_function=NULL;
    acm_function_disable (function);
}      

/*
 *nm_modinit - module init
 * Register NM function driver.
 */
int nm_modinit (void)
{
    PRINTK("NetMonitor: nm_modinit!\n");

    if (usbd_register_function (&nm_function_driver)) {
	PRINTK("NetMonitor:usbd_register_function failed\n");
	return -EINVAL;
    }
    return 0;
}

/*
 * function_exit - module cleanup
 * Deregister function driver.
 */
void nm_modexit (void)
{
    PRINTK ("NetMonitor: nm_modexit!\n");
    if (!usbc_nm_ref_count){
       dump_nm_msg_list(&nm_msg);
       nm_msg.enable = 0;
    }
    usbd_deregister_function (&nm_function_driver);
}
module_init(nm_modinit);
module_exit(nm_modexit);
