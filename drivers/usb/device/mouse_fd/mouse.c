/*
 * mouse_fd/mouse.c
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *      Bruce Balden <balden@lineo.com>
 *
 * Copyright (C) 2002 Toshiba Corporation
 *
 * Changes Copyright (c) 2003 MontaVista Software, Inc.
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
 * Usage:
 *	move pointer to right(10) and bottom(20)
 *		echo "0 10 20" > /proc/driver/mouse_fd
 *	move pointer to left(10) and top(20)
 *		echo "0 246 236" > /proc/driver/mouse_fd
 *	press button 1
 *		echo "1 0 00" > /proc/driver/mouse_fd
 *	press button 2
 *		echo "2 0 00" > /proc/driver/mouse_fd
 *	press button 3
 *		echo "4 0 00" > /proc/driver/mouse_fd
 *	release buttons
 *		echo "0 0 00" > /proc/driver/mouse_fd
 */

#include <linux/config.h>
#include <linux/module.h>

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-module.h"


MODULE_AUTHOR("sl@lineo.com, tbr@lineo.com, TOSHIBA Corporation");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("USB Device Mouse Function");

USBD_MODULE_INFO("mouse_fd 0.1-beta");

#ifndef MODULE
#undef GET_USE_COUNT
#define GET_USE_COUNT(foo) 1
#endif

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <linux/netdevice.h>
#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/proc_fs.h>

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-debug.h"
#include "../usbd-inline.h"
#include "../usbd-arch.h"

#define MOUSE_PACKET_SIZE 3
#define MOUSE_PROC_NAME	"driver/mouse_fd"

#if !defined (CONFIG_USBD_VENDORID) && !defined(CONFIG_USBD_MOUSE_VENDORID)
        #error No Vendor ID
#endif
#if !defined (CONFIG_USBD_PRODUCTID) && !defined(CONFIG_USBD_MOUSE_PRODUCTID)
        #error No Product ID
#endif


#if CONFIG_USBD_MOUSE_VENDORID
        #undef CONFIG_USBD_VENDORID
        #define CONFIG_USBD_VENDORID CONFIG_USBD_MOUSE_VENDORID
#endif

#if CONFIG_USBD_MOUSE_PRODUCTID
        #undef CONFIG_USBD_PRODUCTID
        #define CONFIG_USBD_PRODUCTID CONFIG_USBD_MOUSE_PRODUCTID
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

#ifndef CONFIG_USBD_MOUSE_INT_PKTSIZE
        #define CONFIG_USBD_MOUSE_INT_PKTSIZE             16
#endif

#ifndef CONFIG_USBD_MOUSE_INT_ENDPOINT
    #define CONFIG_USBD_MOUSE_INT_ENDPOINT                3
#endif


/*
 * check for architecture specific endpoint configurations
 */

#if     defined(ABS_INT_ADDR) 
        #warning
        #warning USING ABS ENDPOINT INT ADDRESS
        #undef CONFIG_USBD_MOUSE_INT_ENDPOINT 

        #if     ABS_INT_ADDR
                #define CONFIG_USBD_MOUSE_INT_ENDPOINT            ABS_INT_ADDR
        #endif

#elif   defined(MAX_INT_ADDR) && defined(CONFIG_USBD_MOUSE_INT_ENDPOINT) && (CONFIG_USBD_MOUSE_INT_ENDPOINT > MAX_INT_ADDR)
        #warning
        #warning USING DEFAULT ENDPOINT INT ADDRESS
        #undef CONFIG_USBD_MOUSE_INT_ENDPOINT 
        #define CONFIG_USBD_MOUSE_INT_ENDPOINT            DFL_INT_ADDR
#endif

#if     defined(MAX_INT_PKTSIZE) && defined(CONFIG_USBD_MOUSE_INT_PKTSIZE) && CONFIG_USBD_MOUSE_INT_PKTSIZE > MAX_INT_PKTSIZE
        #warning
        #warning OVERIDING ENDPOINT INT PKTSIZE
        #undef CONFIG_USBD_MOUSE_INT_PKTSIZE
        #define CONFIG_USBD_MOUSE_INT_PKTSIZE             MAX_INT_PKTSIZE
#endif

struct usb_mouse_private {
    struct usb_device_instance *device;
    spinlock_t lock;
    __u8 data[MOUSE_PACKET_SIZE];
    int data_valid;
    int duration;
};

/* Module Parameters ************************************************************************* */

static char *dbg = NULL;
static u32 vendor_id;
static u32 product_id;

MODULE_PARM(dbg, "s");
MODULE_PARM(vendor_id, "i");
MODULE_PARM(product_id, "i");

MODULE_PARM_DESC(dbg, "USB Device Debug options");
MODULE_PARM_DESC(vendor_id, "USB Device Vendor ID");
MODULE_PARM_DESC(product_id, "USB Device Product ID");



/* Debug switches (module parameter "dbg=...") *********************************************** */

extern int dbgflg_usbdfd_init;
int      dbgflg_usbdfd_ep0;
int      dbgflg_usbdfd_usbe;
int      dbgflg_usbdfd_tx;

static debug_option dbg_table[] = {
    {&dbgflg_usbdfd_init,NULL,"init","initialization and termination"},
    {&dbgflg_usbdfd_ep0,NULL,"ep0","End Point 0 (setup) packet handling"},
    {&dbgflg_usbdfd_usbe,NULL,"usbe","USB events"},
    {&dbgflg_usbdfd_tx,NULL,"tx","transmit (to host)"},
    {NULL,NULL,NULL,NULL}
};

#define dbg_init(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_init,lvl,fmt,##args)
#define dbg_ep0(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_ep0,lvl,fmt,##args)
#define dbg_usbe(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_usbe,lvl,fmt,##args)
#define dbg_tx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_tx,lvl,fmt,##args)

/* ******************************************************************************************* */

/* HID Class descriptions 
 */

static struct usb_endpoint_description mouse_default[] = {
    { bEndpointAddress: CONFIG_USBD_MOUSE_INT_ENDPOINT,
        bmAttributes: INTERRUPT,
        wMaxPacketSize: CONFIG_USBD_MOUSE_INT_PKTSIZE,
        bInterval: 0x0a, /* 10ms */
        direction: IN,
        transferSize: MOUSE_PACKET_SIZE, },

};

/* HID Class descriptions
 */
/* c.f. HID E.10 Report Descriptor (Mouse) */
static __u8 mouse_report_desc[] = {
    0x05, 0x01,			// USAGE_PAGE (Generic Desktop)
    0x09, 0x02,			// USAGE (Mouse)
    0xa1, 0x01,			// COLLECTION (Application)
    0x09, 0x01,			//   USAGE (Pointer)
    0xa1, 0x00,			//   COLLECTION (Physical)
    0x05, 0x09,			//     USAGE_PAGE (Button)
    0x19, 0x01,			//     USAGE_MINIMUM (Button 1)
    0x29, 0x03,			//     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,			//     LOGICAL_MINIMUM (0)
    0x25, 0x01,			//     LOGICAL_MAXIMUM (1)
    0x95, 0x03,			//     REPORT_COUNT (3)
    0x75, 0x01,			//     REPORT_SIZE (1)
    0x81, 0x02,			//     INPUT (Data,Variable,Absolute)
    0x95, 0x01,			//     REPORT_COUNT (1)
    0x75, 0x05,			//     REPORT_SIZE (5)
    0x81, 0x01,			//     INPUT (Cnstant)
    0x05, 0x01,			//     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,			//     USAGE (X)
    0x09, 0x31,			//     USAGE (Y)
    0x15, 0x81,			//     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,			//     LOGICAL_MAXIMUM (127)
    0x75, 0x08,			//     REPORT_SIZE (8)
    0x95, 0x02,			//     REPORT_COUNT (2)
    0x81, 0x06,			//     INPUT (Data,Variable,Relative)
    0xc0,			//   END_COLLECTION
    0xc0			// END_COLLECTION
};

static struct usb_class_description mouse_class[] = {
    { USB_DT_HID, 0, 0, { hid: {
	bcdCDC: CLASS_HID_BCD_VERSION,
	bCountryCode: 0,
	bDescriptorType: USB_DT_REPORT,
	wDescriptorLength: sizeof(mouse_report_desc),
	reportDescriptor: mouse_report_desc } }}, 
};

/* Data Interface Alternate description(s)
 */
static __devinitdata struct usb_alternate_description mouse_data_alternate_descriptions[] = {
    {   iInterface: "Simple Mouse Data Interface - Int mode", 
        bAlternateSetting: 0,
	classes: sizeof(mouse_class)/sizeof(struct usb_class_description),
	class_list: mouse_class,
        endpoints: sizeof(mouse_default)/sizeof(struct usb_endpoint_description),
        endpoint_list: mouse_default, },
};

/* Interface description(s)
 */
static __devinitdata struct usb_interface_description mouse_interfaces[] = {
    {   iInterface: "Simple Mouse Data Interface", 
        bInterfaceClass: USB_CLASS_HID,
        bInterfaceSubClass: 0x01, /* Keyboard/Mouse */
        bInterfaceProtocol: 0x02, /* Mouse:2 */
        alternates: sizeof(mouse_data_alternate_descriptions)/sizeof(struct usb_alternate_description),
        alternate_list: mouse_data_alternate_descriptions, },
};


/* Configuration description(s)
 */
struct __devinitdata usb_configuration_description mouse_description[] = {
    {   iConfiguration: "USB Simple Mouse Configuration", 
        bmAttributes: BMATTRIBUTE,
        bMaxPower: BMAXPOWER,
        interfaces: sizeof(mouse_interfaces)/sizeof(struct usb_interface_description),
        interface_list: mouse_interfaces, },
};

/* Device Description
 */
struct __devinitdata usb_device_description mouse_device_description = {
    bDeviceClass:       0, 
    bDeviceSubClass:    0,
    bDeviceProtocol:    0,
    idVendor:           CONFIG_USBD_VENDORID,
    idProduct:          CONFIG_USBD_PRODUCTID,
    iManufacturer:      CONFIG_USBD_MANUFACTURER,
    iProduct:           CONFIG_USBD_PRODUCT_NAME,
    iSerialNumber:      CONFIG_USBD_SERIAL_NUMBER_STR,
};

static int mouse_send(struct usb_mouse_private *mouse_private)
{
    int port = 0; // XXX compound device
    struct urb *urb;
    unsigned long flags;

    spin_lock_irqsave(&mouse_private->lock, flags);
    if (!mouse_private->data_valid) {
	spin_unlock_irqrestore(&mouse_private->lock, flags);
	return 0;
    }
    if ((urb = usbd_alloc_urb(mouse_private->device,
			      (mouse_private->device->function_instance_array+port),
			      CONFIG_USBD_MOUSE_INT_ENDPOINT | IN, MOUSE_PACKET_SIZE))==NULL)
    {
	dbg_tx(0, "failed to alloc urb");
	spin_unlock_irqrestore(&mouse_private->lock, flags);
	return -EINVAL;
    }
    memcpy(urb->buffer, mouse_private->data, MOUSE_PACKET_SIZE);
    urb->actual_length = MOUSE_PACKET_SIZE;
    mouse_private->data_valid = 0;
    spin_unlock_irqrestore(&mouse_private->lock, flags);
    dbgPRINTmem(dbgflg_usbdfd_tx,3,urb->buffer,MOUSE_PACKET_SIZE);

    // push it down into the usb-device layer
    return usbd_send_urb(urb);
}

/* mouse_urb_sent - called to indicate URB transmit finished
 * @urb: pointer to struct urb
 * @rc: result
 */
int mouse_urb_sent (struct urb *urb, int status)
{
    int port = 0; // XXX compound device
    struct usb_device_instance *device = urb->device;
    struct usb_mouse_private *mouse_private = (device->function_instance_array+port)->privdata;

    dbg_tx(2,"%s length: %d status : %x",urb->device->name, urb->actual_length, status);
    usbd_dealloc_urb(urb);

    /* send next urb */
    return mouse_send(mouse_private);
}

/**
 * mouse_recv_setup - called with a control URB 
 * @urb - pointer to struct urb
 *
 * Check if this is a setup packet, process the device request, put results
 * back into the urb and return zero or non-zero to indicate success (DATA)
 * or failure (STALL).
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
int mouse_recv_setup (struct urb * urb)
{
    struct usb_device_request *request;
    struct usb_device_instance *device = urb->device;
    int port = 0;
    struct usb_mouse_private *mouse_private = (device->function_instance_array+port)->privdata;

    request = &urb->device_request;

    // handle HID Class-Specific Request (c.f. HID 7.2)
    if ((request->bmRequestType&USB_REQ_TYPE_MASK)!=USB_REQ_TYPE_CLASS) {
        dbg_ep0(1, "not class request: %x %x", request->bmRequestType, request->bmRequestType&USB_REQ_TYPE_MASK);
        return 0; // XXX
    }

    if ((request->bmRequestType&USB_REQ_DIRECTION_MASK)) {
        dbg_ep0(1, "Device-to-Host");
        switch (request->bRequest) {
        case USB_REQ_SET_IDLE:
	    mouse_private->duration = le16_to_cpu (request->wValue) >> 8;
	    break;
        case USB_REQ_SET_REPORT:
	    /* ignore */
	    break;
	}
    }
    else {
        dbg_ep0(1, "Host-to-Device");
        switch (request->bRequest) {
        case USB_REQ_GET_IDLE:
	    /* FIXME */
            break;
        case USB_REQ_GET_REPORT:
	    /* FIXME */
	    break;
        }
    }
    return 0;
}

/* USB Device Functions ************************************************************************ */

/* proc interface */
static int mouse_write_proc(struct file *file, const char *buffer,
			    unsigned long count, void *data)
{
    char str[128];
    int val[MOUSE_PACKET_SIZE];
    unsigned long flags;
    struct usb_mouse_private *mouse_private = data;

    memset(str, 0, sizeof(str));
    if (copy_from_user(str, buffer, min_t(unsigned long, count, sizeof(str))))
	return -EFAULT;
    if (sscanf(str, "%d %d %d", &val[0], &val[1], &val[2]) != 3)
	return -EINVAL;

    spin_lock_irqsave(&mouse_private->lock, flags);
    if (mouse_private->device->device_state == STATE_CONFIGURED &&
	!mouse_private->data_valid) {
	int i;
	for (i = 0; i < MOUSE_PACKET_SIZE; i++)
	    mouse_private->data[i] = val[i];
	mouse_private->data_valid = 1;
	mouse_send(mouse_private);
    }
    spin_unlock_irqrestore(&mouse_private->lock, flags);
    return count;
}

/* mouse_event - process a device event
 *
 */
void mouse_event(struct usb_device_instance *device, usb_device_event_t event, int data)
{
    int port = 0; // XXX compound device
    struct usb_function_instance *function;

    dbg_usbe(5,"%d",event);

    if ((function = device->function_instance_array+port)==NULL){
        dbg_usbe(1,"no function");
        return;
    }

    dbg_usbe(3,"---> %s %d", device->name, event);
    switch (event) {

    case DEVICE_UNKNOWN:
    case DEVICE_INIT:
        dbg_usbe(1,"---> INIT %s %d", device->name, event);
        break;

    case DEVICE_CREATE:
        dbg_usbe(1,"---> CREATE %s %d", device->name, event);
        {
            struct usb_mouse_private *mouse_private;
	    struct proc_dir_entry *pentry;

            // There is no way to indicate error, so make this unconditional
            // and undo it in the DESTROY event unconditionally as well.
            // It the responsibility of the USBD core and the bus interface
            // to see that there is a matching DESTROY for every CREATE.

            if ((mouse_private = kmalloc(sizeof(struct usb_mouse_private),GFP_ATOMIC))==NULL) {
                dbg_usbe(1,"---> CREATE malloc failed %s",device->name);
                return;
            }

            mouse_private->device = device;
	    mouse_private->duration = 0;	/* infinite */
	    mouse_private->data_valid = 0;
	    spin_lock_init(&mouse_private->lock);
	    pentry = create_proc_entry(MOUSE_PROC_NAME, 0200, NULL);
	    if (pentry) {
		pentry->write_proc = mouse_write_proc;
		pentry->data = mouse_private;
	    }

            function->privdata = mouse_private;

            dbg_usbe(1,"---> START %s privdata assigned: %p", device->name, mouse_private); 
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
        dbg_usbe(1,"---> CONFIGURED %s %d", device->name, event);
        break;
    case DEVICE_SET_INTERFACE:
        break;
    case DEVICE_SET_FEATURE:
        break;
    case DEVICE_CLEAR_FEATURE:
        break;
    case DEVICE_DE_CONFIGURED:
        dbg_usbe(1,"---> DECONFIGURED %s %d", device->name, event);
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
        dbg_usbe(1, "---> DESTROY %s %d", device->name, event);
        {
            struct usb_mouse_private *mouse_private;

            if ((mouse_private = (device->function_instance_array+port)->privdata) == NULL) {
                dbg_usbe(1, "---> DESTROY %s mouse_private null", device->name);
                return;
            }
            dbg_usbe(1, "---> DESTROY %s mouse_private %p", device->name, mouse_private);

            kfree(mouse_private);
	    remove_proc_entry(MOUSE_PROC_NAME, NULL);

            dbg_usbe(1,"---> STOP %s",device->name); 
            return;
        }
        break;

    case DEVICE_FUNCTION_PRIVATE:
        dbg_usbe(3, "%s", usbd_device_events[DEVICE_FUNCTION_PRIVATE]);
        break;

    }
}

static void mouse_function_exit(struct usb_device_instance *device)
{
    dbg_init(1, "CLOSING **************************");
    /* DEVICE_DESTROY will not called when bi driver unloaded.  Call here. */
    mouse_event(device, DEVICE_DESTROY, 0);
}

struct usb_function_operations function_ops = {
    event: mouse_event,
    urb_sent: mouse_urb_sent,
    recv_setup:mouse_recv_setup,
    function_exit: mouse_function_exit,
};

struct usb_function_driver function_driver = {
    name: "usbd mouse",
    ops: &function_ops,
    device_description: &mouse_device_description,
    configurations: sizeof(mouse_description)/sizeof(struct usb_configuration_description),
    configuration_description: mouse_description,
    this_module: THIS_MODULE,
};


/*
 * mouse_modinit - module init
 *
 */
static int __init mouse_modinit(void)
{
    printk(KERN_INFO "%s (dbg=\"%s\")\n", __usbd_module_info, dbg?dbg:"");

    if (vendor_id) {
        mouse_device_description.idVendor = vendor_id;
    }
    if (product_id) {
        mouse_device_description.idProduct = product_id;
    }
    printk(KERN_INFO "vendor_id: %04x product_id: %04x\n",
	   mouse_device_description.idVendor,
	   mouse_device_description.idProduct);

    if (0 != scan_debug_options("mouse_fd",dbg_table,dbg)) {
        return(-EINVAL);
    }


    // register us with the usb device support layer
    //
    if (usbd_register_function(&function_driver)) {
        return -EINVAL;
    }

    // return
    return 0;
}


/* mouse_modexit - module cleanup
 */
static void __exit mouse_modexit(void)
{
    // de-register us with the usb device support layer
    // 
    usbd_deregister_function(&function_driver);
}

module_init(mouse_modinit);
module_exit(mouse_modexit);

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
