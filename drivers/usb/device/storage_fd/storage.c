/*
 * storage_fd/storage.c
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
 * Changes copyright (C) 2003 MontaVista Software, Inc.
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
 * This module implements USB Mass Storage Class
 * (SubClass: ATAPI (0x05), Transport: Bulk-Only)
 *
 * Usage:
 */

#include <linux/config.h>
#include <linux/module.h>

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-module.h"


MODULE_AUTHOR("sl@lineo.com, tbr@lineo.com, TOSHIBA Corporation");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("USB Device Mass Storage Function");

#define STORAGE_MOD_NAME	"storage_fd"
USBD_MODULE_INFO(STORAGE_MOD_NAME " 0.1-beta");

#ifndef MODULE
#undef GET_USE_COUNT
#define GET_USE_COUNT(foo) 1
#endif

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#include <linux/netdevice.h>
#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <scsi/scsi.h>

#define __KERNEL_SYSCALLS__
static int errno;
#include <asm/unistd.h>

#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../usbd-debug.h"
#include "../usbd-inline.h"
#include "../usbd-arch.h"

#define MAX_STORAGES	1
#define STORAGE_DEFAULT_FILENAME	"/dev/ram0"
#define STORAGE_BLOCK_SIZE	512
#define STORAGE_PROC_NAME	"driver/" STORAGE_MOD_NAME

#define USB_REQ_BO_MASS_STORAGE_RESET	0xFF
#define USB_REQ_GET_MAX_LUN		0xFE

/* The Peripheral Device Type is returned by the INQUIRY command.
 * Peripheral Device Type 0x00 is for a direct-access device.
 */
#define PERIPHERAL_DEVICE_TYPE	0x00

#if !defined (CONFIG_USBD_VENDORID) && !defined(CONFIG_USBD_STORAGE_VENDORID)
        #error No Vendor ID
#endif
#if !defined (CONFIG_USBD_PRODUCTID) && !defined(CONFIG_USBD_STORAGE_PRODUCTID)
        #error No Product ID
#endif


#if CONFIG_USBD_STORAGE_VENDORID
        #undef CONFIG_USBD_VENDORID
        #define CONFIG_USBD_VENDORID CONFIG_USBD_STORAGE_VENDORID
#endif

#if CONFIG_USBD_STORAGE_PRODUCTID
        #undef CONFIG_USBD_PRODUCTID
        #define CONFIG_USBD_PRODUCTID CONFIG_USBD_STORAGE_PRODUCTID
#endif

#ifndef CONFIG_USBD_SERIAL_NUMBER_STR
        #define CONFIG_USBD_SERIAL_NUMBER_STR	"000000000000"
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

#ifndef CONFIG_USBD_STORAGE_OUT_PKTSIZE
        #define CONFIG_USBD_STORAGE_OUT_PKTSIZE             64
#endif

#ifndef CONFIG_USBD_STORAGE_IN_PKTSIZE
        #define CONFIG_USBD_STORAGE_IN_PKTSIZE              64
#endif

#ifndef CONFIG_USBD_STORAGE_OUT_ENDPOINT
    #define CONFIG_USBD_STORAGE_OUT_ENDPOINT                1
#endif

#ifndef CONFIG_USBD_STORAGE_IN_ENDPOINT
    #define CONFIG_USBD_STORAGE_IN_ENDPOINT                 2
#endif


/*
 * check for architecture specific endpoint configurations
 */

#if     defined(ABS_OUT_ADDR) 
        #warning
        #warning USING ABS ENDPOINT OUT ADDRESS
        #undef CONFIG_USBD_STORAGE_OUT_ENDPOINT 

        #if     ABS_OUT_ADDR
                #define CONFIG_USBD_STORAGE_OUT_ENDPOINT        ABS_OUT_ADDR
        #endif

#if     defined(ABS_IN_ADDR) 
        #warning
        #warning USING ABS ENDPOINT IN ADDRESS
        #undef CONFIG_USBD_STORAGE_IN_ENDPOINT 

        #if     ABS_IN_ADDR
                #define CONFIG_USBD_STORAGE_IN_ENDPOINT         ABS_IN_ADDR
        #endif

#elif   defined(MAX_OUT_ADDR) && defined(CONFIG_USBD_STORAGE_OUT_ENDPOINT) && (CONFIG_USBD_STORAGE_OUT_ENDPOINT > MAX_OUT_ADDR)
        #warning
        #warning USING DEFAULT ENDPOINT OUT ADDRESS
        #undef CONFIG_USBD_STORAGE_OUT_ENDPOINT 
        #define CONFIG_USBD_STORAGE_OUT_ENDPOINT            DFL_OUT_ADDR
#endif

#elif   defined(MAX_IN_ADDR) && defined(CONFIG_USBD_STORAGE_IN_ENDPOINT) && (CONFIG_USBD_STORAGE_IN_ENDPOINT > MAX_IN_ADDR)
        #warning
        #warning USING DEFAULT ENDPOINT IN ADDRESS
        #undef CONFIG_USBD_STORAGE_IN_ENDPOINT 
        #define CONFIG_USBD_STORAGE_IN_ENDPOINT             DFL_IN_ADDR
#endif

#if     defined(MAX_OUT_PKTSIZE) && defined(CONFIG_USBD_STORAGE_OUT_PKTSIZE) && CONFIG_USBD_STORAGE_OUT_PKTSIZE > MAX_OUT_PKTSIZE
        #warning
        #warning OVERIDING ENDPOINT OUT PKTSIZE
        #undef CONFIG_USBD_STORAGE_OUT_PKTSIZE
        #define CONFIG_USBD_STORAGE_OUT_PKTSIZE             MAX_OUT_PKTSIZE
#endif

#if     defined(MAX_IN_PKTSIZE) && defined(CONFIG_USBD_STORAGE_IN_PKTSIZE) && CONFIG_USBD_STORAGE_IN_PKTSIZE > MAX_IN_PKTSIZE
        #warning
        #warning OVERIDING ENDPOINT IN PKTSIZE
        #undef CONFIG_USBD_STORAGE_IN_PKTSIZE
        #define CONFIG_USBD_STORAGE_IN_PKTSIZE              MAX_IN_PKTSIZE
#endif

enum usb_storage_device_state {
    STATE_INVALID,
    STATE_IDLE,	/* Device intends to receive command */
    STATE_DN,	/* Device intends to transfer no data */
    STATE_DI,	/* Device intends to send data to the host */
    STATE_DO,	/* Device intends to receive data from the host */
};

#define CSW_STAT_GOOD	0x00
#define CSW_STAT_FAILED	0x01
#define CSW_STAT_PERR	0x02	/* Phase Error */

/* Command Block Wrapper */
#define MAX_CBLENGTH	16
struct CBW {
    u32	dSignature;
    u32 dTag;
    u32 dDataTransferLength;
    u8	bmFlags;
    u8	bLUN;
    u8	bCBLength;
    u8	CB[MAX_CBLENGTH];
} __attribute__((packed));
#define CBW_SIGNATURE	0x43425355

/* Command Status Wrapper */
struct CSW {
    u32	dSignature;
    u32	dTag;
    u32	dDataResidue;
    u8	bStatus;
} __attribute__((packed));
#define CSW_SIGNATURE	0x53425355

/* The READ_FORMAT_CAPACITIES command isn't defined in scsi.h */
#ifndef READ_FORMAT_CAPACITIES
#define READ_FORMAT_CAPACITIES 0x23
#endif

struct usb_storage_threaddata {
    int busy;
    struct CBW cbw;	/* cpu endian */
    struct CSW csw;	/* cpu endian */
    u8 *data_buf;
    /* NOTE: data_len CAN exceed cbw.dDataTransferLength */
    unsigned int data_len;
    unsigned int data_alloclen;

    /* sense data */
    struct {
	u8 key;
	u32 info;
	u32 cmdinfo;
	u8 code;
    } sense;

    /* real storage */
    struct file  *real_fd;
    char filename[128];
    unsigned int num_blocks;

    /* statistics */
    struct {
	/* transmit statistics */
	unsigned int read_blocks;
	unsigned int write_blocks;
	/* mass storage class-specific requests */
	unsigned int mass_storage_reset;
	unsigned int get_max_lun;
	/* command statictics */
	unsigned int format_unit;
	unsigned int inquiry;
	unsigned int mode_select_10;
	unsigned int mode_sense_10;
	unsigned int allow_medium_removal;
	unsigned int read_10;
	unsigned int read_12;
	unsigned int read_capacity;
	unsigned int read_format_capacities;
	unsigned int request_sense;
	unsigned int seek_10;
	unsigned int start_stop;
	unsigned int test_unit_ready;
	unsigned int verify;
	unsigned int write_10;
	unsigned int write_12;
	unsigned int write_verify;
	unsigned int unsupported;
    } stat;
};

struct usb_storage_private {
    struct usb_device_instance *device;
    enum usb_storage_device_state devstate;

    struct usb_storage_threaddata tdata;
};

static spinlock_t storage_lock;
static struct usb_storage_private storage_private;	// one and only storage

static DECLARE_MUTEX_LOCKED(storage_sem_start);
static DECLARE_MUTEX_LOCKED(storage_sem_work);
static int storage_thread_terminating;

static void storage_thread_poke(void)
{
    up(&storage_sem_work);
}

/* SCSI related definitions */
/*
 *	Sense codes
 */
 
#define SENCODE_NO_SENSE                        0x00
#define SENCODE_END_OF_DATA                     0x00
#define SENCODE_BECOMING_READY                  0x04
#define SENCODE_INIT_CMD_REQUIRED               0x04
#define SENCODE_PARAM_LIST_LENGTH_ERROR         0x1A
#define SENCODE_INVALID_COMMAND                 0x20
#define SENCODE_LBA_OUT_OF_RANGE                0x21
#define SENCODE_INVALID_CDB_FIELD               0x24
#define SENCODE_LUN_NOT_SUPPORTED               0x25
#define SENCODE_INVALID_PARAM_FIELD             0x26
#define SENCODE_PARAM_NOT_SUPPORTED             0x26
#define SENCODE_PARAM_VALUE_INVALID             0x26
#define SENCODE_RESET_OCCURRED                  0x29
#define SENCODE_LUN_NOT_SELF_CONFIGURED_YET     0x3E
#define SENCODE_INQUIRY_DATA_CHANGED            0x3F
#define SENCODE_SAVING_PARAMS_NOT_SUPPORTED     0x39
#define SENCODE_DIAGNOSTIC_FAILURE              0x40
#define SENCODE_INTERNAL_TARGET_FAILURE         0x44
#define SENCODE_INVALID_MESSAGE_ERROR           0x49
#define SENCODE_LUN_FAILED_SELF_CONFIG          0x4c
#define SENCODE_OVERLAPPED_COMMAND              0x4E

/* Module Parameters ************************************************************************* */

static char *dbg = NULL;
static u32 vendor_id;
static u32 product_id;
static char *serial_number = NULL;
static char *filename = NULL;

MODULE_PARM(dbg, "s");
MODULE_PARM(vendor_id, "i");
MODULE_PARM(product_id, "i");
MODULE_PARM(serial_number, "s");
MODULE_PARM(filename, "s");

MODULE_PARM_DESC(dbg, "USB Device Debug options");
MODULE_PARM_DESC(vendor_id, "USB Device Vendor ID");
MODULE_PARM_DESC(product_id, "USB Device Product ID");
MODULE_PARM_DESC(serial_number, "USB Device Serial Number");
MODULE_PARM_DESC(filename, "USB Device Storage Filename");



/* Debug switches (module parameter "dbg=...") *********************************************** */

extern int dbgflg_usbdfd_init;
int      dbgflg_usbdfd_ep0;
int      dbgflg_usbdfd_usbe;
int      dbgflg_usbdfd_tx;
int      dbgflg_usbdfd_rx;

static debug_option dbg_table[] = {
    {&dbgflg_usbdfd_init,NULL,"init","initialization and termination"},
    {&dbgflg_usbdfd_ep0,NULL,"ep0","End Point 0 (setup) packet handling"},
    {&dbgflg_usbdfd_usbe,NULL,"usbe","USB events"},
    {&dbgflg_usbdfd_tx,NULL,"tx","transmit (to host)"},
    {&dbgflg_usbdfd_rx,NULL,"rx","receive (from host)"},
    {NULL,NULL,NULL,NULL}
};

#define dbg_init(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_init,lvl,fmt,##args)
#define dbg_ep0(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_ep0,lvl,fmt,##args)
#define dbg_usbe(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_usbe,lvl,fmt,##args)
#define dbg_tx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_tx,lvl,fmt,##args)
#define dbg_rx(lvl,fmt,args...) dbgPRINT(dbgflg_usbdfd_rx,lvl,fmt,##args)

/* ******************************************************************************************* */

/* Mass Storage Class descriptions 
 */

#define STORAGE_TRANSFER_SIZE	STORAGE_BLOCK_SIZE	/* XXX max 0xfff. why? (see usbd-bi.c) */
static struct usb_endpoint_description storage_default[] = {
    { bEndpointAddress: CONFIG_USBD_STORAGE_OUT_ENDPOINT,
        bmAttributes: BULK,
        wMaxPacketSize: CONFIG_USBD_STORAGE_OUT_PKTSIZE,
        bInterval: 0,
        direction: OUT,
        transferSize: STORAGE_TRANSFER_SIZE, },

    { bEndpointAddress: CONFIG_USBD_STORAGE_IN_ENDPOINT,
        bmAttributes: BULK,
        wMaxPacketSize: CONFIG_USBD_STORAGE_IN_PKTSIZE,
        bInterval: 0,
        direction: IN,
        transferSize: STORAGE_TRANSFER_SIZE, },

};

/* Data Interface Alternate description(s)
 */
static __initdata struct usb_alternate_description storage_data_alternate_descriptions[] = {
    {   iInterface: "Simple Mass Storage Data Interface - Bulk-Only", 
        bAlternateSetting: 0,
	classes: 0,
	class_list: NULL,
        endpoints: sizeof(storage_default)/sizeof(struct usb_endpoint_description),
        endpoint_list: storage_default, },
};

/* Interface description(s)
 */
static __initdata struct usb_interface_description storage_interfaces[] = {
    {   iInterface: "Simple Mass Storage Data Interface", 
        bInterfaceClass: USB_CLASS_MASS_STORAGE,
	bInterfaceSubClass: 0x05, /*removable*/
        bInterfaceProtocol: 0x50, /* Bulk-Only Transport */
        alternates: sizeof(storage_data_alternate_descriptions)/sizeof(struct usb_alternate_description),
        alternate_list: storage_data_alternate_descriptions, },
};


/* Configuration description(s)
 */
struct __initdata usb_configuration_description storage_description[] = {
    {   iConfiguration: "USB Simple Mass Storage Configuration", 
        bmAttributes: BMATTRIBUTE,
        bMaxPower: BMAXPOWER,
        interfaces: sizeof(storage_interfaces)/sizeof(struct usb_interface_description),
        interface_list: storage_interfaces, },
};

/* Device Description
 */
struct __initdata usb_device_description storage_device_description = {
    bDeviceClass:       0, 
    bDeviceSubClass:    0,
    bDeviceProtocol:    0,
    idVendor:           CONFIG_USBD_VENDORID,
    idProduct:          CONFIG_USBD_PRODUCTID,
    iManufacturer:      CONFIG_USBD_MANUFACTURER,
    iProduct:           CONFIG_USBD_PRODUCT_NAME,
    iSerialNumber:      CONFIG_USBD_SERIAL_NUMBER_STR,
};

static int storage_receive_CBW(struct usb_storage_private *private,
			       u8 *buf, unsigned int len)
{
    struct CBW *cbwp = (struct CBW *)buf;
    struct usb_storage_threaddata *tdata = &private->tdata;

    if (len != sizeof(struct CBW)) {
	printk(KERN_ERR STORAGE_MOD_NAME ": bad CBW length (%d)\n", len);
	return -1;
    }
    if (le32_to_cpu(cbwp->dSignature) != CBW_SIGNATURE) {
	printk(KERN_ERR STORAGE_MOD_NAME ": bad CBW signature (%x)\n",
	       le32_to_cpu(cbwp->dSignature));
	return -1;
    }
    if (cbwp->bLUN != 0) {
	printk(KERN_ERR STORAGE_MOD_NAME ": bad CBW LUN (%x)\n", cbwp->bLUN);
	return -1;
    }

    tdata->cbw = *cbwp;
    tdata->cbw.dSignature = le32_to_cpu(tdata->cbw.dSignature);
    tdata->cbw.dTag = le32_to_cpu(tdata->cbw.dTag);
    tdata->cbw.dDataTransferLength = le32_to_cpu(tdata->cbw.dDataTransferLength);
    tdata->csw.dSignature = CSW_SIGNATURE;
    tdata->csw.dTag = tdata->cbw.dTag;
    tdata->csw.dDataResidue = tdata->cbw.dDataTransferLength;
    tdata->csw.bStatus = CSW_STAT_GOOD;

    return len - sizeof(struct CBW);
}

static int storage_send_CSW(struct usb_storage_private *private)
{
    struct urb *urb;
    int port = 0; // XXX compound device
    struct usb_storage_threaddata *tdata = &private->tdata;

    urb = usbd_alloc_urb(private->device,
			 private->device->function_instance_array+port,
			 CONFIG_USBD_STORAGE_IN_ENDPOINT | IN, sizeof (struct CSW));
    if (!urb) {
	printk(KERN_ERR STORAGE_MOD_NAME ": failed to alloc CSW urb\n");
	return -EINVAL;
    }


    tdata->csw.dSignature = cpu_to_le32(tdata->csw.dSignature);
    tdata->csw.dTag = cpu_to_le32(tdata->csw.dTag);
    tdata->csw.dDataResidue = cpu_to_le32(tdata->csw.dDataResidue);
    memcpy(urb->buffer, &tdata->csw, sizeof(struct CSW));
    urb->actual_length = sizeof(struct CSW);
    dbg_rx(3,"CSW (length %d)", urb->actual_length);
    dbgPRINTmem(dbgflg_usbdfd_tx,3,urb->buffer,min(urb->actual_length, 32u));
    if (usbd_send_urb(urb))
	return -EINVAL;
    return 0;
}

/* storage_urb_sent - called to indicate URB transmit finished
 * @urb: pointer to struct urb
 * @rc: result
 */
int storage_urb_sent (struct urb *urb, int status)
{
    dbg_tx(2,"%s length: %d status : %x",urb->device->name, urb->actual_length, status);
    usbd_dealloc_urb(urb);

    return 0;
}

/* storage_recv_urb - called to indicate URB has been received
 * @urb - pointer to struct urb
 *
 * Return non-zero if we failed and urb is still valid (not disposed)
 * NOTE: This function is called in bottom half context.
 */
int storage_recv_urb (struct urb *urb)
{
    int port = 0; // XXX compound device
    struct usb_device_instance *device = urb->device;
    struct usb_storage_private *private = (device->function_instance_array+port)->privdata;
    struct usb_storage_threaddata *tdata = &private->tdata;

    if (urb->status != RECV_OK)
	return 1;
    dbg_rx(2,"length=%d",urb->actual_length);
    dbgPRINTmem(dbgflg_usbdfd_rx,3,urb->buffer,min(urb->actual_length, 32u));

    // push the data up
    spin_lock(&storage_lock);
    if (private->tdata.busy) {
	printk(KERN_ERR STORAGE_MOD_NAME ": receive in busy state (%d)\n",
	       private->devstate);
	private->devstate = STATE_INVALID;
	spin_unlock(&storage_lock);
	usbd_recycle_urb(urb);	// free urb
	return 0;
    }
    switch (private->devstate) {
    case STATE_IDLE:
	tdata->data_len = 0;
	if (storage_receive_CBW(private, urb->buffer, urb->actual_length) < 0) {
	    private->devstate = STATE_INVALID;
	    break;
	}

	if (tdata->cbw.dDataTransferLength == 0) {
	    private->devstate = STATE_DN;
	    private->tdata.busy = 1;
	    storage_thread_poke();
	    break;
	}
	if (tdata->data_alloclen < tdata->cbw.dDataTransferLength) {
	    /* expand data buffer */
	    if (tdata->data_buf)
		kfree(tdata->data_buf);
	    tdata->data_buf = kmalloc(tdata->cbw.dDataTransferLength, GFP_ATOMIC);
	    if (!tdata->data_buf) {
		printk(KERN_ERR STORAGE_MOD_NAME ": failed to expand buffer (%d)\n",
		       tdata->cbw.dDataTransferLength);
		tdata->data_alloclen = 0;
		private->devstate = STATE_INVALID;
		break;
	    }
	    tdata->data_alloclen = tdata->cbw.dDataTransferLength;
	}
	if (tdata->cbw.bmFlags & 0x80) {	/* IN */
	    private->devstate = STATE_DI;
	    private->tdata.busy = 1;
	    storage_thread_poke();
	} else {	/* OUT */
	    private->devstate = STATE_DO;
	}
	break;
    case STATE_DO:
	if (tdata->data_len + urb->actual_length > tdata->cbw.dDataTransferLength) {
	    printk(KERN_ERR STORAGE_MOD_NAME ": bad DATA length (%d + %d > %d)\n",
		   tdata->data_len, urb->actual_length,
		   tdata->cbw.dDataTransferLength);
	    private->devstate = STATE_INVALID;
	    break;
	}
	memcpy(tdata->data_buf + tdata->data_len,
	       urb->buffer, urb->actual_length);
	tdata->data_len += urb->actual_length;
	if (tdata->data_len >= tdata->cbw.dDataTransferLength) {
	    /* all data received */
	    private->devstate = STATE_DN;
	    private->tdata.busy = 1;
	    storage_thread_poke();
	}
	break;
    case STATE_DI:
    default:
	printk(KERN_ERR STORAGE_MOD_NAME ": receive in bad state (%d)\n",
	       private->devstate);
	private->devstate = STATE_INVALID;
    }
    spin_unlock(&storage_lock);

    usbd_recycle_urb(urb);	// free urb
    return(0);
}

/**
 * storage_recv_setup - called with a control URB 
 * @urb - pointer to struct urb
 *
 * Check if this is a setup packet, process the device request, put results
 * back into the urb and return zero or non-zero to indicate success (DATA)
 * or failure (STALL).
 *
 * This routine IS called at interrupt time. Please use the usual precautions.
 *
 */
int storage_recv_setup (struct urb *urb)
{
    struct usb_device_request *request;
    struct usb_device_instance *device = urb->device;
    int port = 0;
    struct usb_storage_private *private = (device->function_instance_array+port)->privdata;
    struct usb_storage_threaddata *tdata = &private->tdata;

    request = &urb->device_request;

    // handle Mass Storage Class-Specific Request
    // c.f. USB Mass Storage Class Bulk-Only Transport 3.1, 3.2
    if ((request->bmRequestType & (USB_REQ_TYPE_MASK|USB_REQ_RECIPIENT_MASK)) !=
	 (USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE)) {
        dbg_ep0(1, "not class/interface request: %x", request->bmRequestType);
        return 0; // XXX
    }

    if ((request->bmRequestType&USB_REQ_DIRECTION_MASK)) {
        dbg_ep0(1, "Device-to-Host");
        switch (request->bRequest) {
        case USB_REQ_GET_MAX_LUN:
	    tdata->stat.get_max_lun++;
	    urb->actual_length = 1;
	    urb->buffer[0] = 0;	// only one LUN
	    break;
	default:
	    dbg_ep0(1, "Unknown request: %x", request->bRequest);
	}
    }
    else {
        dbg_ep0(1, "Host-to-Device");
        switch (request->bRequest) {
        case USB_REQ_BO_MASS_STORAGE_RESET:
	    tdata->stat.mass_storage_reset++;
	    /* do job in event handler */
	    usbd_device_event(private->device, DEVICE_FUNCTION_PRIVATE, 0);
            break;
	default:
	    dbg_ep0(1, "Unknown request: %x", request->bRequest);
        }
    }
    return 0;
}

/* USB Device Functions ************************************************************************ */

/* proc interface */
static int storage_read_proc(char *page, char **start, off_t off,
			     int count, int *eof, void *data)
{
    char *p = page;
    struct usb_storage_private *private = &storage_private;

    spin_lock_bh(&storage_lock);	/* prevent event handler */
    if (private->device) {
	struct usb_storage_threaddata *tdata = &private->tdata;
	p += sprintf(p, "storage:\t\t\"%s\" %d blocks\n",
		     tdata->filename, tdata->num_blocks);
	p += sprintf(p, "state:\t\t\t%d\n", private->devstate);
	p += sprintf(p, "transfer statistics:\n");
	p += sprintf(p, " read_blocks\t\t%u\n", tdata->stat.read_blocks);
	p += sprintf(p, " write_blocks\t\t%u\n", tdata->stat.write_blocks);
	p += sprintf(p, "mass storage class-specific request statistics:\n");
	p += sprintf(p, " mass_storage_reset\t%u\n", 
		tdata->stat.mass_storage_reset);
	p += sprintf(p, " get_max_lun\t\t%u\n", tdata->stat.get_max_lun);
	p += sprintf(p, "command statistics:\n");
	p += sprintf(p, " format_unit\t\t%u\n", tdata->stat.format_unit);
	p += sprintf(p, " inquiry\t\t%u\n", tdata->stat.inquiry);
	p += sprintf(p, " mode_select_10\t\t%u\n", tdata->stat.mode_select_10);
	p += sprintf(p, " mode_sense_10\t\t%u\n", tdata->stat.mode_sense_10);
	p += sprintf(p, " allow_medium_removal\t%u\n", 
		tdata->stat.allow_medium_removal);
	p += sprintf(p, " read_10\t\t%u\n", tdata->stat.read_10);
	p += sprintf(p, " read_12\t\t%u\n", tdata->stat.read_12);
	p += sprintf(p, " read_capacity\t\t%u\n", tdata->stat.read_capacity);
	p += sprintf(p, " read_format_capacities\t%u\n", 
		tdata->stat.read_format_capacities);
	p += sprintf(p, " request_sense\t\t%u\n", tdata->stat.request_sense);
	p += sprintf(p, " seek_10\t\t%u\n", tdata->stat.seek_10);
	p += sprintf(p, " start_stop\t\t%u\n", tdata->stat.start_stop);
	p += sprintf(p, " test_unit_ready\t%u\n", tdata->stat.test_unit_ready);
	p += sprintf(p, " verify\t\t\t%u\n", tdata->stat.verify);
	p += sprintf(p, " write_10\t\t%u\n", tdata->stat.write_10);
	p += sprintf(p, " write_12\t\t%u\n", tdata->stat.write_12);
	p += sprintf(p, " write_verify\t\t%u\n", tdata->stat.write_verify);
	p += sprintf(p, " unsupported\t\t%u\n", tdata->stat.unsupported);
    }
    spin_unlock_bh(&storage_lock);

    return p - page;
}

/* storage_event - process a device event
 *
 * NOTE: This function is called from keventd kernel thread.
 */
void storage_event(struct usb_device_instance *device, usb_device_event_t event, int data)
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
            struct usb_storage_private *private;

            // There is no way to indicate error, so make this unconditional
            // and undo it in the DESTROY event unconditionally as well.
            // It the responsibility of the USBD core and the bus interface
            // to see that there is a matching DESTROY for every CREATE.

	    spin_lock(&storage_lock);
	    if (storage_private.device) {
                dbg_usbe(1,"---> CREATE no free storage");
		spin_unlock(&storage_lock);
		return;
	    }
	    private = &storage_private;
	    private->device = device;
	    private->devstate = STATE_IDLE;
            function->privdata = private;
	    spin_unlock(&storage_lock);

            dbg_usbe(1,"---> START %s privdata assigned: %p", device->name, private); 
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
            struct usb_storage_private *private;
	    struct usb_storage_threaddata *tdata;

            if ((private = (device->function_instance_array+port)->privdata) == NULL) {
                dbg_usbe(1, "---> DESTROY %s private null", device->name);
                return;
            }
            dbg_usbe(1, "---> DESTROY %s private %p", device->name, private);
	    tdata = &private->tdata;

	    spin_lock(&storage_lock);
	    private->device = NULL;
	    private->devstate = STATE_INVALID;
	    if (tdata->busy) {
		/* free on DEVICE_FUNCTION_PRIVATE */
	    } else {
		if (tdata->data_buf)
		    kfree(tdata->data_buf);
		tdata->data_buf = NULL;
		tdata->data_len = tdata->data_alloclen = 0;
	    }
	    spin_unlock(&storage_lock);

            dbg_usbe(1,"---> STOP %s",device->name); 
            return;
        }
        break;

    case DEVICE_FUNCTION_PRIVATE:
        dbg_usbe(2,"---> FUNCTION_PRIVATE %s %d", device->name, event);
	{
            struct usb_storage_private *private;
	    struct usb_storage_threaddata *tdata;
            if ((private = (device->function_instance_array+port)->privdata) == NULL) {
                dbg_usbe(1, "---> PRIVATE %s private null", device->name);
                return;
            }
            dbg_usbe(2, "---> PRIVATE %s private %p", device->name, private);
	    tdata = &private->tdata;

	    spin_lock_bh(&storage_lock);
	    if (data == 0) {
		dbg_rx(1, "USB_REQ_BO_MASS_STORAGE_RESET received.");
		private->devstate = STATE_IDLE;
	    } else {
		/* kicked by storage_thread */
		struct urb *urb;

		if (!private->tdata.busy) {
		    /* command completed. send data and status */
		    switch (private->devstate) {
		    case STATE_DI:
			urb = usbd_alloc_urb(private->device,
					     private->device->function_instance_array+port,
					     CONFIG_USBD_STORAGE_IN_ENDPOINT | IN,
					     tdata->cbw.dDataTransferLength);
			if (!urb) {
			    printk(KERN_ERR STORAGE_MOD_NAME ": failed to alloc DATA urb\n");
			    private->devstate = STATE_INVALID;
			    break;
			}
			/* The USB Mass Storage Class Bulk-Only Transport 
			 * specification allows a client a couple of options 
			 * when it has less data to transmit to the host than 
			 * the host requested in the CBW.  The client can either 
			 * pad the data to the full requested length, or stall 
			 * the IN endpoint to notify the host of the condition.
			 * We choose to pad the data to the requested length.
			 */
			if (tdata->data_len < tdata->cbw.dDataTransferLength) {
			    dbg_tx(2, "fill data to pad up (%d < %d)",
				   tdata->data_len,
				   tdata->cbw.dDataTransferLength);
			    memset(tdata->data_buf + tdata->data_len, 0,
				   tdata->cbw.dDataTransferLength - tdata->data_len);
			}
			memcpy(urb->buffer, tdata->data_buf, 
			    tdata->cbw.dDataTransferLength);
			urb->actual_length = tdata->cbw.dDataTransferLength;
			tdata->csw.dDataResidue -= tdata->data_len;
			dbg_rx(3,"DATA (length %d)", tdata->data_len);
			dbgPRINTmem(dbgflg_usbdfd_tx,3,urb->buffer,min(tdata->data_len, 32u));
			if (usbd_send_urb(urb)) {
			    private->devstate = STATE_INVALID;
			    break;
			}
			/* FALLTHRU */
		    case STATE_DN:
			if (storage_send_CSW(private) < 0) {
			    private->devstate = STATE_INVALID;
			    break;
			}
			private->devstate = STATE_IDLE;
			break;
		    case STATE_IDLE:
			/* USB_REQ_BO_MASS_STORAGE_RESET received during
			   command processing */
			break;
		    default:
			private->devstate = STATE_INVALID;
			if (tdata->data_buf)
			    kfree(tdata->data_buf);
			tdata->data_buf = NULL;
			tdata->data_len = tdata->data_alloclen = 0;
		    }
		}
	    }
	    spin_unlock_bh(&storage_lock);
	}
        break;

    }
}


struct usb_function_operations function_ops = {
    event: storage_event,
    recv_urb: storage_recv_urb,
    urb_sent: storage_urb_sent,
    recv_setup: storage_recv_setup
};

struct usb_function_driver function_driver = {
    name: "usbd storage",
    ops: &function_ops,
    device_description: &storage_device_description,
    configurations: sizeof(storage_description)/sizeof(struct usb_configuration_description),
    configuration_description: storage_description,
    this_module: THIS_MODULE,
};


/*
 * RBC functions
 */

static void add_in_data(struct usb_storage_threaddata *tdata,
			void *buf, unsigned int len)
{
    if (tdata->data_len < tdata->cbw.dDataTransferLength) {
	memcpy(tdata->data_buf + tdata->data_len, buf,
	       min(tdata->cbw.dDataTransferLength - tdata->data_len, len));
    }
    tdata->data_len += 
	min(tdata->cbw.dDataTransferLength - tdata->data_len, len);
}

static int do_inquiry(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 data[4 + 4 + 8 + 16 + 4];
    unsigned int len = 4 + 4 + 8 + 16 + 4;
    u8 lun = CB[1] >> 5;

    if (lun != 0)
	goto invalid_lun;

    data[0] = PERIPHERAL_DEVICE_TYPE;
    data[1] = 0x80;		/* removable */
    data[2] = 0x00;
    data[3] = 0x01;
    data[4] = 4 + 8 + 16 + 4;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    memset(&data[8], ' ', 8 + 16 + 4);
    memcpy(&data[8], storage_device_description.iManufacturer,
	   min_t(int, 8, strlen(storage_device_description.iManufacturer)));
    memcpy(&data[16], storage_device_description.iProduct,
	   min_t(int, 16, strlen(storage_device_description.iProduct)));
    add_in_data(tdata, data, min(len, (unsigned int)CB[4]));
    return 0;

  invalid_lun:
    printk(KERN_WARNING STORAGE_MOD_NAME 
	": INQUIRY lun %d not supported\n", lun);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LUN_NOT_SUPPORTED;
    return -EINVAL;
}

static int do_mode_select_10(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 lun = CB[1] >> 5;

    if (lun != 0)
	goto invalid_lun;
    /* ignore the mode select command, nothing changeable */
    tdata->csw.dDataResidue = 0;
    return 0;

  invalid_lun:
    printk(KERN_WARNING STORAGE_MOD_NAME 
	": MODE_SELECT_10 lun %d not supported\n", lun);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LUN_NOT_SUPPORTED;
    return -EINVAL;
}

static int do_mode_sense_10(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 data[8 + 12 + 12 + 12 + 8];
    unsigned int len;
    int pc = CB[2] >> 6;	/* page control */
    int page = CB[2] & 0x3f;
    unsigned int alloc_len = be16_to_cpu(get_unaligned((u16 *)&CB[7]));
    u8 lun = CB[1] >> 5;

    if (lun != 0)
	goto invalid_lun;
    if (page != 0x01 && page != 0x08 && page != 0x1b && page != 0x1c 
	&& page != 0x3f)
    {
	goto invalid_cdb_field;
    }
    /* first page starts at offset 8 */
    len = 8;

    /* If page control 1 is requested, then we are to return a mask indicating 
     * which parameters are changeable rather than returning the actual 
     * parameter values.  A mask of all zeros indicates a parameter which is 
     * not changeable, and a mask of all ones indicates a paremeter which is 
     * changeable.  None of our parameters are changeable, so we will return 
     * zeros for all parameters when pc=1.
     */
    /* generate requested page(s) */
    if (page == 0x01 || page == 0x3f) {
	/* parameter page 0x01 (c.f. SFF-8070 9.7.5.1) */
	data[len + 0] = 0x01;
	data[len + 1] = 12 - 2;
	memset(&data[len +2], 0, 12 - 2);
	len += 12;
    }
    if (page == 0x08 || page == 0x3f) {
	/* parameter page 0x08 (c.f. SFF-8070 9.7.5.2) */
	data[len + 0] = 0x08;
	data[len + 1] = 12 - 2;
	memset(&data[len +2], 0, 12 - 2);
	len += 12;
    }
    if (page == 0x1b || page == 0x3f) {
	/* parameter page 0x1b (c.f. SFF-8070 9.7.5.3) */
	data[len + 0] = 0x1b;
	data[len + 1] = 12 - 2;
	memset(&data[len +2], 0, 12 - 2);
	if (pc != 1) {
	    data[len + 3] = 1;
	}
	len += 12;
    }
    if (page == 0x1c || page == 0x3f) {
	/* parameter page 0x1c (c.f. SFF-8070 9.7.5.4) */
	data[len + 0] = 0x1c;
	data[len + 1] = 8 - 2;
	memset(&data[len +2], 0, 8 - 2);
	len += 8;
    }
    /* mode parameter header (c.f. SPC-2 8.3, SFF-8070 9.7.5) */
    data[0] = 0;
    data[1] = len - 2;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    add_in_data(tdata, data, min(len, alloc_len));
    return 0;

  invalid_lun:
    printk(KERN_WARNING STORAGE_MOD_NAME 
	": MODE_SENSE_10 lun %d not supported\n", lun);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LUN_NOT_SUPPORTED;
    return -EINVAL;

  invalid_cdb_field:
    printk(KERN_WARNING STORAGE_MOD_NAME ": Invalid MODE_SENSE_10 CDB field\n");
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_INVALID_CDB_FIELD;
    return -EINVAL;
}

static int do_read(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    unsigned int lba, llen, len;
    loff_t  tmp_offset;
    int rlen;
    mm_segment_t fs;
    u8 lun = CB[1] >> 5;

    lba = be32_to_cpu(get_unaligned((u32 *)&CB[2]));
    if (CB[0] == READ_12)
	llen = be32_to_cpu(get_unaligned((u32 *)&CB[6]));	/* READ_12 */
    else
	llen = be16_to_cpu(get_unaligned((u16 *)&CB[7]));	/* READ_10 */
    len = llen * STORAGE_BLOCK_SIZE;

    if (lun != 0)
	goto invalid_lun;
    if (tdata->cbw.dDataTransferLength < len)
	goto phase_error;
    if (lba + llen > tdata->num_blocks)
	goto lba_out_of_range;
    if (  ( tmp_offset =  tdata->real_fd->f_op->llseek(tdata->real_fd, (off_t)lba * STORAGE_BLOCK_SIZE,
	      0 /* SEEK_SET */) )   == (off_t)-1) {
	printk (KERN_ERR STORAGE_MOD_NAME ": %s: lseek error on LBA %u \n",
	       tdata->filename, lba);
	goto medium_error;
    }
    fs = get_fs(); set_fs( KERNEL_DS );
    rlen=tdata->real_fd->f_op->read(tdata->real_fd, tdata->data_buf, len, &tmp_offset );
    set_fs( fs );
    if (rlen != len) {
	printk (KERN_ERR STORAGE_MOD_NAME ": %s: read error on LBA %u (read %d) \n",
	       tdata->filename, lba, rlen);
	goto medium_error;
    }
    tdata->data_len = len;
    tdata->stat.read_blocks += llen;
    return 0;

  invalid_lun:
    printk(KERN_WARNING STORAGE_MOD_NAME 
	": READ lun %d not supported\n", lun);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LUN_NOT_SUPPORTED;
    return -EINVAL;
  phase_error:
    printk(KERN_ERR STORAGE_MOD_NAME ": phase error on READ\n");
    tdata->csw.bStatus = CSW_STAT_PERR;
    return -EINVAL;
  lba_out_of_range:
    printk(KERN_WARNING STORAGE_MOD_NAME ": LBA out of range\n");
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LBA_OUT_OF_RANGE;
    return -EINVAL;
  medium_error:
    printk(KERN_WARNING STORAGE_MOD_NAME ": Medium error\n");
    tdata->sense.key = MEDIUM_ERROR;
    return -EINVAL;
}

static int do_readcapacity(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u32 data[2];
    u8 lun = CB[1] >> 5;

    if (lun != 0)
	goto invalid_lun;

    data[0] = cpu_to_be32(tdata->num_blocks - 1);
    data[1] = cpu_to_be32(STORAGE_BLOCK_SIZE);
    add_in_data(tdata, &data, sizeof(data));
    return 0;

  invalid_lun:
    printk(KERN_WARNING STORAGE_MOD_NAME 
	": READ_CAPACITY lun %d not supported\n", lun);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LUN_NOT_SUPPORTED;
    return -EINVAL;
}

static int do_readformatcapacities(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 data[4 + 8 + 8];
    unsigned int alloc_len = be16_to_cpu(get_unaligned((u16 *)&CB[7]));
    u8 lun = CB[1] >> 5;

    if (lun != 0)
	goto invalid_lun;

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 8 + 8;
    /* maximum capacity header */
    *(unsigned int *)(data + 4) = cpu_to_be32(tdata->num_blocks);
    *(unsigned int *)(data + 8) = cpu_to_be32(STORAGE_BLOCK_SIZE);
    data[8] = 0x01;	/* descriptor code: maximum formattable capacity */
    /* formattable capacity descriptor */
    *(unsigned int *)(data + 12) = cpu_to_be32(tdata->num_blocks);
    *(unsigned int *)(data + 16) = cpu_to_be32(STORAGE_BLOCK_SIZE);
    add_in_data(tdata, &data, min(sizeof(data), alloc_len));
    return 0;

  invalid_lun:
    printk(KERN_WARNING STORAGE_MOD_NAME 
	": READ_FORMAT_CAPACITIES lun %d not supported\n", lun);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LUN_NOT_SUPPORTED;
    return -EINVAL;
}

static int do_request_sense(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 data[18];
    unsigned int len = sizeof(data);
    u8 lun = CB[1] >> 5;

    if (lun != 0)
	goto invalid_lun;

    data[0] = 0x70;	/* current errors */
    data[1] = 0;
    data[2] = tdata->sense.key;
    put_unaligned(cpu_to_be32(tdata->sense.info), (u32 *)&data[3]);
    data[7] = len - 7;;
    put_unaligned(cpu_to_be32(tdata->sense.cmdinfo), (u32 *)&data[8]);
    data[12] = tdata->sense.code;
    data[13] = 0;
    data[14] = 0;
    data[15] = 0;
    data[16] = 0;
    data[17] = 0;
    add_in_data(tdata, data, min(len, (unsigned int)CB[4]));
    return 0;

  invalid_lun:
    printk(KERN_WARNING STORAGE_MOD_NAME 
	": REQUEST_SENSE lun %d not supported\n", lun);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LUN_NOT_SUPPORTED;
    return -EINVAL;
}

static int do_write(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    unsigned int lba, llen, len;
    loff_t  tmp_offset;
    int rlen;
    mm_segment_t fs;
    u8 lun = CB[1] >> 5;

    lba = be32_to_cpu(get_unaligned((u32 *)&CB[2]));
    if (CB[0] == WRITE_12)
	llen = be32_to_cpu(get_unaligned((u32 *)&CB[6]));	/* WRITE_12 */
    else	/* WRITE_10, WRITE_VERIFY */
	llen = be16_to_cpu(get_unaligned((u16 *)&CB[7]));
    len = llen * STORAGE_BLOCK_SIZE;

    if (lun != 0)
	goto invalid_lun;
    if (tdata->data_len < len)
	goto phase_error;
    if (lba + llen > tdata->num_blocks)
	goto lba_out_of_range;
    if (  ( tmp_offset = tdata->real_fd->f_op->llseek(tdata->real_fd, (off_t)lba * STORAGE_BLOCK_SIZE,
	      0 /* SEEK_SET */) ) == (off_t)-1) {
	printk (KERN_ERR STORAGE_MOD_NAME ": %s: lseek error on LBA %u (errno %d)\n",
	       tdata->filename, lba, errno);
	goto medium_error;
    }
    fs = get_fs(); set_fs( KERNEL_DS );
    rlen=tdata->real_fd->f_op->write(tdata->real_fd, tdata->data_buf, len, &tmp_offset);
    set_fs( fs );
    if (rlen != len) {
	printk (KERN_ERR STORAGE_MOD_NAME ": %s: write error on LBA %u (errno %d)\n",
	       tdata->filename, lba, errno);
	goto medium_error;
    }
    tdata->csw.dDataResidue -= len;
    tdata->stat.write_blocks += llen;
    return 0;

  invalid_lun:
    printk(KERN_WARNING STORAGE_MOD_NAME 
	": WRITE lun %d not supported\n", lun);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LUN_NOT_SUPPORTED;
    return -EINVAL;
  phase_error:
    printk(KERN_ERR STORAGE_MOD_NAME ": phase error on WRITE\n");
    tdata->csw.bStatus = CSW_STAT_PERR;
    return -EINVAL;
  lba_out_of_range:
    printk(KERN_WARNING STORAGE_MOD_NAME ": LBA out of range\n");
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LBA_OUT_OF_RANGE;
    return -EINVAL;
  medium_error:
    printk(KERN_WARNING STORAGE_MOD_NAME ": Medium error\n");
    tdata->sense.key = MEDIUM_ERROR;
    return -EINVAL;
}

/* process RBC Command Block */
static int storage_process_CB(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;

    if (CB[0] != REQUEST_SENSE) {
	/* initialize sense data */
	tdata->sense.key = NO_SENSE;
	tdata->sense.info = 0;
	tdata->sense.code = SENCODE_NO_SENSE;
	tdata->sense.cmdinfo = 0;
    }

    switch (CB[0]) {
    case FORMAT_UNIT:
	dbg_rx(2, "FORMAT_UNIT");
	tdata->stat.format_unit++;
	if (tdata->data_len)
	    goto phase_error;
	tdata->csw.dDataResidue = 0;
	/* do nothing */
	break;
    case INQUIRY:
	dbg_rx(2, "INQUIRY");
	tdata->stat.inquiry++;
	if (tdata->data_len)
	    goto phase_error;
	return do_inquiry(tdata);
    case MODE_SELECT_10:
	dbg_rx(2, "MODE_SELECT_10");
	tdata->stat.mode_select_10++;
	if (tdata->data_len)
	    goto phase_error;
	return do_mode_select_10(tdata);
    case MODE_SENSE_10:
	dbg_rx(2, "MODE_SENSE_10");
	tdata->stat.mode_sense_10++;
	if (tdata->data_len)
	    goto phase_error;
	return do_mode_sense_10(tdata);
    case ALLOW_MEDIUM_REMOVAL:
	dbg_rx(2, "ALLOW_MEDIUM_REMOVAL");
	tdata->stat.allow_medium_removal++;
	if (tdata->data_len)
	    goto phase_error;
	/* do nothing */
	break;
    case READ_10:
	dbg_rx(2, "READ_10");
	tdata->stat.read_10++;
	if (tdata->data_len)
	    goto phase_error;
	return do_read(tdata);
    case READ_12:
	dbg_rx(2, "READ_12");
	tdata->stat.read_12++;
	if (tdata->data_len)
	    goto phase_error;
	return do_read(tdata);
    case READ_CAPACITY:
	dbg_rx(2, "READ_CAPACITY");
	tdata->stat.read_capacity++;
	if (tdata->data_len)
	    goto phase_error;
	return do_readcapacity(tdata);
    case READ_FORMAT_CAPACITIES:
	dbg_rx(2, "READ_FORMAT_CAPACITIES");
	tdata->stat.read_format_capacities++;
	if (tdata->data_len)
	    goto phase_error;
	return do_readformatcapacities(tdata);
    case REQUEST_SENSE:
	dbg_rx(2, "REQUEST_SENSE");
	tdata->stat.request_sense++;
	if (tdata->data_len)
	    goto phase_error;
	return do_request_sense(tdata);
    case SEEK_10:
	dbg_rx(2, "SEEK_10");
	tdata->stat.seek_10++;
	if (tdata->data_len)
	    goto phase_error;
	/* do nothing */
	break;
    case START_STOP:
	dbg_rx(2, "START_STOP");
	tdata->stat.start_stop++;
	if (tdata->data_len)
	    goto phase_error;
	/* do nothing */
	break;
    case TEST_UNIT_READY:
	dbg_rx(2, "TEST_UNIT_READY");
	tdata->stat.test_unit_ready++;
	if (tdata->data_len)
	    goto phase_error;
	/* do nothing */
	break;
    case VERIFY:
	dbg_rx(2, "VERIFY");
	tdata->stat.verify++;
	if (tdata->data_len)
	    goto phase_error;
	/* do nothing */
	break;
    case WRITE_10:
	dbg_rx(2, "WRITE_10");
	tdata->stat.write_10++;
	return do_write(tdata);
    case WRITE_12:
	dbg_rx(2, "WRITE_12");
	tdata->stat.write_12++;
	return do_write(tdata);
    case WRITE_VERIFY:
	dbg_rx(2, "WRITE_VERIFY");
	tdata->stat.write_verify++;
	return do_write(tdata);
    default:
	printk(KERN_ERR STORAGE_MOD_NAME ": unknown RBC command (%02x)\n",
	       CB[0]);
	tdata->stat.unsupported++;
	goto unsupported;
    }

    return 0;

  phase_error:
    printk(KERN_ERR STORAGE_MOD_NAME ": phase error on RBC command %02x\n", CB[0]);
    tdata->csw.bStatus = CSW_STAT_PERR;
    return -EINVAL;
  unsupported:
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_INVALID_COMMAND;
    return -EINVAL;
}

static int storage_thread(void *data) 
{
    struct usb_storage_private *private;
    off_t off;

    daemonize ();
    reparent_to_init();
    spin_lock_irq(&current->sigmask_lock);
    sigemptyset(&current->blocked);
    recalc_sigpending(current);
    spin_unlock_irq(&current->sigmask_lock);

    strcpy (current->comm, STORAGE_MOD_NAME);

    private = &storage_private;
    memset(private, 0, sizeof(*private));
    if (!filename)
	filename = STORAGE_DEFAULT_FILENAME;
    strcpy(private->tdata.filename, filename);
    private->tdata.real_fd = filp_open (private->tdata.filename, O_RDWR, 0);
    if (IS_ERR(private->tdata.real_fd)) {
	printk(KERN_ERR STORAGE_MOD_NAME ": %s open failed (errno %d)\n",
	       private->tdata.filename, errno);
	storage_thread_terminating = 1;
	goto end;
    }
    if ((off = private->tdata.real_fd->f_op->llseek(private->tdata.real_fd, 0, 2 /* SEEK_END*/)) == (off_t)-1) {
	printk (KERN_ERR STORAGE_MOD_NAME ": %s lseek failed (errno %d)\n",
	       private->tdata.filename, errno);
	filp_close(private->tdata.real_fd,0);
	private->tdata.real_fd = NULL;
	storage_thread_terminating = 1;
	goto end;
    }
    private->tdata.num_blocks = off / STORAGE_BLOCK_SIZE;
    private->devstate = STATE_INVALID;
    printk(KERN_INFO STORAGE_MOD_NAME ": storage filename %s, %d blocks\n",
	   private->tdata.filename, private->tdata.num_blocks);

    // let startup continue
    up(&storage_sem_start);

    // process loop
    for (storage_thread_terminating = 0; !storage_thread_terminating;) {
        // wait for someone to tell us to do something
        down(&storage_sem_work);

	spin_lock_bh(&storage_lock);	/* prevent event handler */
	if (private->device && private->tdata.busy) {
	    spin_unlock_bh(&storage_lock);
	    /* this function CAN sleep */
	    if (storage_process_CB(&private->tdata) < 0) {
		if (private->tdata.csw.bStatus == CSW_STAT_GOOD)
		    private->tdata.csw.bStatus = CSW_STAT_FAILED;
	    }
	    spin_lock_bh(&storage_lock);
	    private->tdata.busy = 0;
	    /* kick event routine */
	    usbd_device_event(private->device, DEVICE_FUNCTION_PRIVATE, 1);
	}
	spin_unlock_bh(&storage_lock);
    }

    /* shutdown */
    if (private->tdata.data_buf)
	kfree(private->tdata.data_buf);
    filp_close(private->tdata.real_fd,0);

    // let the process stopping us know we are done and return
 end:
    up(&storage_sem_start);
    complete_and_exit (NULL, 0);
    return 0;
}

/**
 * storage_thread_kickoff - start command processing thread
 */
static int storage_thread_kickoff(void)
{
    storage_thread_terminating = 0;
    kernel_thread(&storage_thread, NULL, CLONE_FS | CLONE_FILES);
    down(&storage_sem_start);
    if (storage_thread_terminating)
	return -1;
    return 0;
}

/**
 * storage_thread_killoff - stop command processing thread
 */
static void storage_thread_killoff(void)
{
    if (!storage_thread_terminating) {
        storage_thread_terminating = 1;
        up(&storage_sem_work);
        down(&storage_sem_start);
    }
}

#define STORAGE_SERIAL_CHAR_MIN	12
static int __init storage_check_serial(const char *s)
{
    // c.f. USB Mass Storage Class Bulk-Only Transport 4.1.1 Serial Number
    if (strlen(s) < STORAGE_SERIAL_CHAR_MIN)
	return 0;
    while (*s) {
	// '0'-'9', 'A'-'F'
	if (!isxdigit(*s) || islower(*s))
	    return 0;
	s++;
    }
    return 1;
}

/*
 * storage_modinit - module init
 *
 */
static int __init storage_modinit(void)
{
    static char serial_str[STORAGE_SERIAL_CHAR_MIN + 1];
    int i, len;

    printk(KERN_INFO "%s (dbg=\"%s\")\n", __usbd_module_info, dbg?dbg:"");

    if (vendor_id) {
        storage_device_description.idVendor = vendor_id;
    }
    if (product_id) {
        storage_device_description.idProduct = product_id;
    }
    if (!serial_number)
	serial_number = storage_device_description.iSerialNumber;
    len = strlen(serial_number);
    while (len++ < STORAGE_SERIAL_CHAR_MIN)
	strcat(serial_str, "0");
    strcat(serial_str, serial_number);
    for (i = 0; serial_str[i]; i++) {
	if (islower(serial_str[i]))
	    serial_str[i] = toupper(serial_str[i]);
    }
    storage_device_description.iSerialNumber = serial_str;

    printk(KERN_INFO "vendor_id: %04x product_id: %04x, serial_number: %s\n",
	   storage_device_description.idVendor,
	   storage_device_description.idProduct,
	   storage_device_description.iSerialNumber);

    if (0 != scan_debug_options(STORAGE_MOD_NAME, dbg_table,dbg)) {
        return(-EINVAL);
    }

    if (!storage_check_serial(storage_device_description.iSerialNumber)) {
	printk(KERN_ERR "bad serial number (%s)\n",
	       storage_device_description.iSerialNumber);
	return -EINVAL;
    }

    spin_lock_init(&storage_lock);
    create_proc_read_entry(STORAGE_PROC_NAME, 0, NULL, storage_read_proc, 0);

    if (storage_thread_kickoff()) {
	return -ENODEV;
    }

    // register us with the usb device support layer
    //
    if (usbd_register_function(&function_driver)) {
        return -EINVAL;
    }

    // return
    return 0;
}


/* storage_modexit - module cleanup
 */
static void __exit storage_modexit(void)
{
    // de-register us with the usb device support layer
    // 
    usbd_deregister_function(&function_driver);
    remove_proc_entry(STORAGE_PROC_NAME, NULL);
    storage_thread_killoff();
}

module_init(storage_modinit);
module_exit(storage_modexit);

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
