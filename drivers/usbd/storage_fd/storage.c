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
 * Changes copyright (C) 2003 - Motorola
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
 * Date         Author        Reference    Comment
 * ==========   ===========   ===========  ==========================
 * 10/24/2003   Rob Coleman   LIBdd44877   Added the interface number
 *
 */

/*
 * This module implements USB Mass Storage Class
 * (SubClass: RBC, Transport: Bulk-Only)
 *
 * Usage:
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>


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
#include <linux/fs.h>
#include <linux/trace.h>
#include <linux/file.h>
#include <linux/types.h>
#include <linux/stat.h>
#include <linux/fcntl.h>
#include <scsi/scsi.h>

#ifdef CONFIG_USBD_ISP_BUS
#include <asm/io.h>
#endif

#define __KERNEL_SYSCALLS__
static int errno;
#include <asm/unistd.h>

#ifdef CONFIG_ARCH_EZX
#include <linux/ezxusbd.h>
#endif

#ifdef CONFIG_ITUNES_SIDEBAND
#include <linux/kmod.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#endif // CONFIG_ITUNES_SIDEBAND

#include "../usbd-export.h"
#include "../usbd-build.h"
#include "../usbd-chap9.h"
#include "../usbd-mem.h"
#include "../usbd.h"
#include "../usbd-func.h"
#include "../usbd-bus.h"
#include "../trace.h"
#ifdef CONFIG_USBD_ISP_BUS
#include "../../mmc/mmc_core.h"
#endif
MODULE_AUTHOR ("a18615@motorola.com");
MODULE_DESCRIPTION ("USB Device Mass Storage Function");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,17)
MODULE_LICENSE ("GPL");
#endif

#define STORAGE_MOD_NAME	"storage_fd"
USBD_MODULE_INFO (STORAGE_MOD_NAME " 1.0");

#define STORAGE_BLOCK_SIZE	512
#define STORAGE_PROC_NAME	STORAGE_MOD_NAME
#define STORAGE_MMC_NAME    "USBAction"

#define USB_REQ_BO_MASS_STORAGE_RESET	0xFF
#define USB_REQ_GET_MAX_LUN		0xFE

#if 0
/* RBC 6.1 states "RBC devices are identified by a PERIPHERAL DEVICE
 * TYPE 0Eh" */
#define PERIPHERAL_DEVICE_TYPE	0x0e
#else
/* XXX: Linux sd,scsi stack seems to expect this value... */
#define PERIPHERAL_DEVICE_TYPE	0x00
#endif

#if !defined(CONFIG_USBD_STORAGE_VENDORID)
#define CONFIG_USBD_STORAGE_VENDORID 0x22b8
#endif


#if  !defined(CONFIG_USBD_STORAGE_PRODUCTID)
#define CONFIG_USBD_STORAGE_PRODUCTID 0xffff
#endif

#if  !defined(CONFIG_USBD_STORAGE_BCDDEVICE)
#define CONFIG_USBD_STORAGE_BCDDEVICE 0x0001
#endif

#if  !defined(BULK_IN_PKTSIZE)
#define BULK_IN_PKTSIZE 64
#endif

#if  !defined(BULK_OUT_PKTSIZE)
#define BULK_OUT_PKTSIZE 64
#endif

#if  !defined(MAX_FAT_PARTITION)
#define MAX_FAT_PARTITION   3
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
 * Define kernel spaces
 */
#define SAVE_FS_DS(x)       \
do{                         \
    x = get_fs ();          \
    set_fs (KERNEL_DS);     \
}while(0)

#define RESTORE_FS_DS(x)    \
do{                         \
    set_fs(x);              \
}while(0)

/*
 * define debug print
 */
#if 0
#define dbgPRINT(fmt,args...)               \
do{                                         \
    printk("MAS: " fmt "\n", ##args);    \
}while (0)
#else
#define dbgPRINT(fmt,args...)               \
    do {}while (0)
#endif
/*
 * CSW states
 */
#define CSW_STAT_GOOD	0x00
#define CSW_STAT_FAILED	0x01
#define CSW_STAT_PERR	0x02	/* Phase Error */

/* SCSI related definitions */
/* private USB Mass Storage Mode Switch Command */
#define USB_SWITCH_FROM_UDISK   0xD6
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

enum usb_storage_device_state
{
    STATE_INVALID,
    STATE_IDLE,			/* Device intends to receive command */
    STATE_DN,			/* Device intends to transfer no data */
    STATE_DI,			/* Device intends to send data to the host */
    STATE_DO,			/* Device intends to receive data from the host */
};

/* Command Block Wrapper */
#define MAX_CBLENGTH	16
#define MAX_CBW_LEN     31
struct CBW
{
    u32 dSignature;
    u32 dTag;
    u32 dDataTransferLength;
    u8 bmFlags;
    u8 bLUN;
    u8 bCBLength;
    u8 CB[MAX_CBLENGTH];
} __attribute__ ((packed));
#define CBW_SIGNATURE	0x43425355

/* Command Status Wrapper */
#define MAX_CSW_LEN     13
struct CSW
{
    u32 dSignature;
    u32 dTag;
    u32 dDataResidue;
    u8 bStatus;
} __attribute__ ((packed));
#define CSW_SIGNATURE	0x53425355

static char *device_node[MAX_FAT_PARTITION + 1] =
{
    "/dev/tffsb",   //preload+user logical partition
    "/dev/mmca",    //mmc card a
    "/dev/mmcb",    //mmc card b
    ""              //NULL
};

#ifdef CONFIG_USBD_HIGH_SPEED
static struct usb_device_qualifier_descriptor storage_device_qualifier_descriptor = {
        bLength: sizeof(struct usb_device_qualifier_descriptor),
        bDescriptorType: USB_DT_DEVICE_QUALIFIER,
        bcdUSB: __constant_cpu_to_le16(USB_BCD_VERSION),
        bDeviceClass: 0x00,
        bDeviceSubClass: 0x00,
        bDeviceProtocol: 0x00,         
	bMaxPacketSize0: 0x00,
	bNumConfigurations: 0x00,
};
#endif
 

struct usb_storage_threaddata
{
    int busy;
    struct CBW cbw;		/* cpu endian */
    struct CSW csw;		/* cpu endian */
    u8 *data_buf;
    /* NOTE: data_len CAN exceed cbw.dDataTransferLength */
    unsigned int data_len;
    unsigned int data_alloclen;

    /* sense data */
    struct
    {
        u8 key;
        u32 info;
        u32 cmdinfo;
        u8 code;
    } sense;

    /* real storage */
    int logic_units;
    //struct file *real_fd[MAX_FAT_PARTITION];
    int real_fd[MAX_FAT_PARTITION];
    off_t num_blocks[MAX_FAT_PARTITION];

    /* statistics */
    struct
    {
        /* transmit statistics */
        unsigned int read_blocks;
        unsigned int write_blocks;
        /* command statictics */
        unsigned int inquiry;
        unsigned int mode_select;
        unsigned int mode_sense;
        unsigned int read_10;
        unsigned int read_capacity;
        unsigned int request_sense;
        unsigned int start_stop;
        unsigned int test_unit_ready;
        unsigned int verify;
        unsigned int write_10;
        unsigned int write_buffer;
        unsigned int unsupported;
    } stat;
} __attribute__ ((packed));


struct usb_storage_private
{
    struct usb_function_instance *function;
    enum usb_storage_device_state devstate;
    struct usb_storage_threaddata tdata;
	//struct tq_struct storage_bh;		// runs as bottom half, equivalent to interrupt time
};
static struct usb_storage_private storage_private;	// one and only storage

#ifdef CONFIG_ITUNES_SIDEBAND

typedef enum {FALSE, TRUE} BOOL;

static BOOL mas_working = FALSE;

char sideband_app_path[256] = "/usr/SYSqtapp/security/ssm/sideband";

#define VPD_ENABLE			0x1
#define SIDEBAND_APP_COMPLETE		0x1
#define MAX_SIDEBAND_PACKETSIZE		248

/* The iTunes sideband protocol request. */
typedef struct sideband_request
{
    __u8 operation;			/* The scsi command ID,  For sideband, The scsi command ID is INQUIRY(0x12) */
    __u8 VPD_enable;			/* If equal 1,it enable VPD */
    __u8 page_number;			/* The page number requested */
    __u8 reserved;			/* Reserved */
    __u8 buffer_size;			/* Buffer size for inquiry results(0xFF) */
    __u8 control;			/* Control(0x0) */
} sideband_request;

/* The iTunes sideband protocol response. */
typedef struct sideband_response
{
    __u8 device_type;			/* Device type (ignored) */
    __u8 page_number;			/* The page number responsed */
    __u8 reserved;			/* Reserved */
    __u8 data_size;			/* The number of bytes in this page excluding first 4 bytes of this header */
    __u8 data[MAX_SIDEBAND_PACKETSIZE];	/* the data of special page */
} sideband_response;

/* The iTunes sideband application response */
typedef struct sideband_app_request
{
    sideband_request srdata;		/* sideband protocol data */
    __u8 isUSBLost;			/* If USBCABLE_LOST, indicting the USB cable is plugged out or USB mode is lost. */
} sideband_app_request;

/* the iTunes sideband application response */
typedef struct sideband_app_response
{
    sideband_response srdata;		/* Device type (ignored) */
    __u8 complete;			/* the flag describes whether the capability file has been sent completely. 
    					 * if  SIDEBAND_APP_COMPLETE, the response data is the last VPD page*/
} sideband_app_response;


#define ITUNES_SIDEBAND_DEV_NAME	"sideband"
#define ITUNES_VPD_INQUIRY_TIMEOUT	(2*HZ)

#define VPD_PAGE_START		0xC0
#define VPD_PAGES		64	/* 0xC0 ~ 0xFF */

#define VPD_READBUFFER_SIZE	12	/* sizeof(sideband_app_request) */
#define VPD_WRITEBUFFER_SIZE	256	/* sizeof(sideband_app_response) */

#define VPD_BUF_EMPTY		0x00
#define VPD_BUF_REQ_PUT_ING	0x01
#define VPD_BUF_REQ_PUT_DONE	0x02
#define VPD_BUF_REQ_GET_ING	0x04
#define VPD_BUF_REQ_GET_DONE	0x08
#define VPD_BUF_RSP_PUT_ING	0x10
#define VPD_BUF_RSP_PUT_DONE	0x20
#define VPD_BUF_RSP_GET_ING	0x40
#define VPD_BUF_RSP_GET_DONE	0x80

typedef struct iTunes_sideband_dev
{
    struct list_head list;		/* list head for device list */
    int minor;				/* device minor number */
    int open_count;			/* have been opened for n times */
    struct fasync_struct *async_queue;	/* asynchronous readers */
//    sideband_response vpd[VPD_PAGES];	/* store all the pages? */
    atomic_t status;			/* buffer operation state */
    wait_queue_head_t req_wq, req_rq;	/* read and write queues */
    wait_queue_head_t rsp_wq, rsp_rq;	/* read and write queues */
    u8 read_buf[VPD_READBUFFER_SIZE];
    u8 write_buf[VPD_WRITEBUFFER_SIZE];
    u8 *read_p, *write_p;
    int read_rest, write_rest;
} iTunes_sideband_dev_t;

typedef struct iTunes_sideband_driver
{
    struct list_head devices;		/* head of devices list */
    u16 device_nr;			/* number of devices */
    spinlock_t lock;
} iTunes_sideband_driver_t;
    
struct iTunes_sideband_driver iTunes_sideband_driver;

static int iTunes_sideband_dev_create(void)
{
    struct iTunes_sideband_dev *dev = (struct iTunes_sideband_dev *)NULL;
    int ret = 0;
    
    dev = (struct iTunes_sideband_dev *)kmalloc(sizeof(struct iTunes_sideband_dev), GFP_KERNEL);
    if (NULL == dev)
    {
    	ret = -ENOMEM;
	goto out;
    }
    
    memset(dev, 0, sizeof(iTunes_sideband_dev_t));
    dev->minor = ITUNES_SIDEBAND_MINOR;
    init_waitqueue_head(&dev->req_wq);
    init_waitqueue_head(&dev->req_rq);
    init_waitqueue_head(&dev->rsp_wq);
    init_waitqueue_head(&dev->rsp_rq);
    atomic_set(&dev->status, VPD_BUF_EMPTY);
    dev->read_p = dev->read_buf;
    dev->read_rest = 0;
    dev->write_p = dev->write_buf;
    dev->write_rest = sizeof(sideband_app_response);
    
    spin_lock(&iTunes_sideband_driver.lock);
    list_add_tail(&dev->list, &iTunes_sideband_driver.devices);
    iTunes_sideband_driver.device_nr++;
    spin_unlock(&iTunes_sideband_driver.lock);
out:
    return ret;
}

static int iTunes_sideband_dev_destroy(void)
{
    struct iTunes_sideband_dev *dev  = (struct iTunes_sideband_dev *)NULL;
    int ret = 0;
    struct list_head *ptr;
    
    spin_lock(&iTunes_sideband_driver.lock);
    if (list_empty(&iTunes_sideband_driver.devices))
    {
        spin_unlock(&iTunes_sideband_driver.lock);
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t%s\r\n", __FILE__, __LINE__, __FUNCTION__, "No devices!");
	ret = -ENODEV;
	goto out;
    }
    list_for_each_prev(ptr, &iTunes_sideband_driver.devices)
    {
        dev = list_entry(ptr, struct iTunes_sideband_dev, list);
	ptr = dev->list.next;
	list_del(&dev->list);
	iTunes_sideband_driver.device_nr--;
	kfree(dev);
    }
    spin_unlock(&iTunes_sideband_driver.lock);
out:
    return ret;
}

static BOOL iTunes_sideband_read_ready(int status)
{
    if ((VPD_BUF_REQ_PUT_DONE | VPD_BUF_REQ_GET_ING) & status)
    {
        return TRUE;
    }
    else
    {
	return FALSE;
    }
}

static BOOL iTunes_sideband_write_ready(int status)
{
    if ((VPD_BUF_REQ_GET_DONE | VPD_BUF_RSP_PUT_ING) & status)
    {
    	return TRUE;
    }
    else
    {
    	return FALSE;
    }
}

static ssize_t iTunes_sideband_read(struct file *filep, char *buf, size_t count, loff_t *f_pos)
{
    struct iTunes_sideband_dev *dev  = (struct iTunes_sideband_dev *)NULL;
    sideband_app_request *req;
    int ret = 0, len = 0;
    dev = (struct iTunes_sideband_dev *)filep->private_data;
    if (dev)
    {
    	while(FALSE == iTunes_sideband_read_ready(atomic_read(&dev->status)))
	{
	    if (filep->f_flags & O_NONBLOCK)
	    { /* non-block */
	    	ret = -EAGAIN;
		goto out;
	    }
	    if (wait_event_interruptible(dev->req_rq, iTunes_sideband_read_ready(atomic_read(&dev->status))))
	    {
	        ret = -ERESTARTSYS;
		goto out;
	    }
	}
	atomic_set(&dev->status, VPD_BUF_REQ_GET_ING);
	if (count >= dev->read_rest)
	{ /* all of the rest */
	    len = dev->read_rest;
	}
	else
	{ /* part of the data */
	    len = count;
	}
	if (copy_to_user(buf, dev->read_p, len))
	{
	    ret = -EFAULT;
	    goto out;
	}
	dev->read_rest -= len;
	dev->read_p = (u8 *)((u8*)dev->read_p + len);
	if (0 == dev->read_rest)
	{
	    req = (sideband_app_request *)dev->read_buf;
	    if (req->isUSBLost == TRUE)
	    {
	    	atomic_set(&dev->status, VPD_BUF_EMPTY);
		wake_up_interruptible(&dev->req_wq);
	    }
	    else
	    {
	    	atomic_set(&dev->status, VPD_BUF_REQ_GET_DONE);
		wake_up_interruptible(&dev->rsp_wq);
	    }
	}
	ret = len;
    }
    else
    {
        ret = -ENODEV;
    }
out:
    return ret;
}

static ssize_t iTunes_sideband_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos)
{
    struct iTunes_sideband_dev *dev  = (struct iTunes_sideband_dev *)NULL;
    int ret = 0, len = 0;
    dev = (struct iTunes_sideband_dev *)filep->private_data;
    if (dev)
    {
    	while(FALSE == iTunes_sideband_write_ready(atomic_read(&dev->status)))
	{
	    if (filep->f_flags & O_NONBLOCK)
	    { /* non-block */
	    	ret = -EAGAIN;
		goto out;
	    }
	    if (wait_event_interruptible(dev->rsp_wq, iTunes_sideband_write_ready(atomic_read(&dev->status))))
	    {
	    	ret = -ERESTARTSYS;
		goto out;
	    }
	}
	atomic_set(&dev->status, VPD_BUF_RSP_PUT_ING);
	if (count >= dev->write_rest)
	{ /* all of the rest */
	    len = dev->write_rest;
	}
	else
	{ /* part of the data */
	    len = count;
	}
	if (copy_from_user(dev->write_p, buf, len))
	{
	    ret = -EFAULT;
	    goto out;
	}
	dev->write_rest -= len;
	dev->write_p = (u8 *)((u8*)dev->write_p + len);
	if (0 == dev->write_rest)
	{
	    atomic_set(&dev->status, VPD_BUF_RSP_PUT_DONE);
	    wake_up_interruptible(&dev->rsp_rq);
	}
	ret = len;
    }
    else
    {
    	ret = -ENODEV;
    }
out:
    return ret;
}

static unsigned int iTunes_sideband_poll(struct file *filep, poll_table *wait)
{
    struct iTunes_sideband_dev *dev  = (struct iTunes_sideband_dev *)NULL;
    u32 mask = 0;
    
    dev = (struct iTunes_sideband_dev *)filep->private_data;
    if (dev)
    {
        poll_wait(filep, &dev->req_wq,  wait);
	poll_wait(filep, &dev->req_rq,  wait);
	if (iTunes_sideband_read_ready(atomic_read(&dev->status)))
	{
	    mask |= POLLIN | POLLRDNORM;
	}
	if (iTunes_sideband_write_ready(atomic_read(&dev->status)))
	{
	    mask |= POLLOUT | POLLWRNORM;
	}
	return mask;
    }
    else
    {
        return -ENODEV;
    }
}

static int iTunes_sideband_fasync(int fd, struct file *filep, int mode)
{
    struct iTunes_sideband_dev *dev  = (struct iTunes_sideband_dev *)NULL;
    
    dev = (struct iTunes_sideband_dev *)filep->private_data;
    if (dev)
    {
    	return fasync_helper(fd, filep, mode, &dev->async_queue);
    }
    else
    {
    	return -ENODEV;
    }
}

static int iTunes_sideband_open(struct inode *inode, struct file *filep)
{
    struct iTunes_sideband_dev *dev  = (struct iTunes_sideband_dev *)NULL;
    struct list_head *ptr;
    int ret = 0;
    int minor = MINOR(inode->i_rdev);
    
    /* find minor */
    spin_lock(&iTunes_sideband_driver.lock);
    if (list_empty(&iTunes_sideband_driver.devices))
    {
        spin_unlock(&iTunes_sideband_driver.lock);
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t%s\r\n", __FILE__, __LINE__, __FUNCTION__, "No devices!");
	ret = -ENODEV;
	goto out;
    }
    list_for_each(ptr, &iTunes_sideband_driver.devices)
    {
    	dev = list_entry(ptr, struct iTunes_sideband_dev, list);
	if (dev->minor == minor)
		break;
    }
    spin_unlock(&iTunes_sideband_driver.lock);
    
    if (dev->minor != minor)
    {	
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\tDidn't find the minor: %d!\r\n", __FILE__, __LINE__, __FUNCTION__, minor);
	ret = -ENODEV;
	goto out;
    }
    
    if (dev->open_count)
    {
        ret = -EBUSY;
	goto out;
    }
    filep->private_data = (void *)dev;
    dev->open_count++;
out:
    return ret;
}

static int iTunes_sideband_release(struct inode *inode, struct file *filep)
{
    struct iTunes_sideband_dev *dev  = (struct iTunes_sideband_dev *)NULL;
    
    dev = (struct iTunes_sideband_dev *)filep->private_data;
    if (dev)
    {
        iTunes_sideband_fasync(-1, filep, 0);
	if (dev->open_count > 1)
	{
	    dev->open_count--;
	}
	else
	{
	    if (VPD_BUF_EMPTY != atomic_read(&dev->status))
	    {
	    	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t\"%s\" close device while device status is 0x%02X \r\n", \
			__FILE__, __LINE__, __FUNCTION__, current->comm, atomic_read(&dev->status));
	    }
	    filep->private_data = NULL;
	    dev->open_count = 0;
	}
	return 0;
    }
    else
    {
    	return -ENODEV;
    }
}

static struct file_operations iTunes_sideband_fops =
{
    .read =	iTunes_sideband_read,
    .write =	iTunes_sideband_write,
    .open =	iTunes_sideband_open,
    .release =	iTunes_sideband_release,
    .fasync =	iTunes_sideband_fasync,
    .poll =	iTunes_sideband_poll,
    .llseek =	no_llseek,
};

static struct miscdevice iTunes_sideband_miscdev =
{
    .minor =	ITUNES_SIDEBAND_MINOR,
    .name =	ITUNES_SIDEBAND_DEV_NAME,
    .fops =	&iTunes_sideband_fops,
};

static int run_sideband_app(void)
{
    int i, ret;
    char *argv[2], *envp[3];

    if (!sideband_app_path[0])
    {
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t%s\r\n", __FILE__, __LINE__, __FUNCTION__, "Sideband app path not provided!");	
	return -EINVAL;
    }
    if (in_interrupt())
    {
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t%s\r\n", __FILE__, __LINE__, __FUNCTION__, "in_interrupt, not allowed!");
	return -EIO;
    }
    if (!current->fs->root)
    {
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t%s\r\n", __FILE__, __LINE__, __FUNCTION__, "no FS !");
	return -EIO;
    }
    
    i = 0;
    argv[i++] = sideband_app_path;
    argv[i] = 0;
    
    i = 0;
    envp[i++] = "HOME=/";
    envp[i++] = "PATH=/sbin:/bin:/usr/sbin:/usr/bin";
    envp[i] = 0;
    
    ret = call_usermodehelper(argv[0], argv, envp);
    if (0 != ret)
    {
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t%s%d\r\n", __FILE__, __LINE__, __FUNCTION__, "Call sideband app failed: ", ret);
    }
    return ret;
}

#define __itunes_wait_event_interruptible_timeout(wq, condition, ret)	\
do {									\
    wait_queue_t __wait;						\
    init_waitqueue_entry(&__wait, current);				\
    									\
    add_wait_queue(&wq, &__wait);					\
    for (;;) {								\
        set_current_state(TASK_INTERRUPTIBLE);				\
	if (condition)							\
	    break;							\
	if (!signal_pending(current)) {					\
	    ret = schedule_timeout(ret);				\
	    if (!ret)							\
	    	break;							\
	    continue;							\
	}								\
	ret = -ERESTARTSYS;						\
	break;								\
    }									\
    current->state = TASK_RUNNING;					\
    remove_wait_queue(&wq, &__wait);					\
} while (0)
	
#define itunes_wait_event_interruptible_timeout(wq, condition, timeout)	\
({									\
    long __ret = timeout;						\
    if (!(condition))							\
        __itunes_wait_event_interruptible_timeout(wq, condition, __ret);\
    __ret;								\
})

static void add_in_data (struct usb_storage_threaddata *, void *, unsigned int);

static int do_vendor_specific_vpd(struct usb_storage_threaddata *tdata, int page, unsigned int alloclen)
{
    struct iTunes_sideband_dev *dev  = (struct iTunes_sideband_dev *)NULL;
    sideband_app_request *req;
    sideband_app_response *rsp;
    struct list_head *ptr;
    u8 data[256];
    unsigned int len = 0;
    int ret = 0;
    long tmp = 0;
    
    spin_lock(&iTunes_sideband_driver.lock);
    if (list_empty(&iTunes_sideband_driver.devices))
    {
    	spin_unlock(&iTunes_sideband_driver.lock);
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t%s\r\n", __FILE__, __LINE__, __FUNCTION__, "No devices!");
	ret = -ENODEV;
	goto out;
    }
    list_for_each(ptr, &iTunes_sideband_driver.devices)
    {
    	dev = list_entry(ptr, struct iTunes_sideband_dev, list);
	if (ITUNES_SIDEBAND_MINOR == dev->minor)
	{
	    break;
	}
    }
    spin_unlock(&iTunes_sideband_driver.lock);
    
    if (0 == dev->open_count)
    {
        if (run_sideband_app())
	{
	    dev = NULL;
	    ret = -ERESTARTSYS;
	    goto out;
	}
    }
    req = (sideband_app_request *)dev->read_buf;
    rsp = (sideband_app_response *)dev->write_buf;
    
    if (VPD_BUF_EMPTY != atomic_read(&dev->status))
    {
	tmp = itunes_wait_event_interruptible_timeout(dev->req_wq, (VPD_BUF_EMPTY == atomic_read(&dev->status)), ITUNES_VPD_INQUIRY_TIMEOUT);
	if (tmp < 0)
	{
	    ret = -ERESTARTSYS;
	    goto out;
	    
	}
    }
    if (VPD_BUF_EMPTY != atomic_read(&dev->status))
    { /* Just timeout */
	ret = -ERESTARTSYS;
	goto out;
    }
    
    atomic_set(&dev->status, VPD_BUF_REQ_PUT_ING);
    memset(req, 0, VPD_READBUFFER_SIZE);
    req->srdata.operation = INQUIRY;
    req->srdata.VPD_enable = VPD_ENABLE;
    req->srdata.page_number = page;
    req->srdata.reserved = 0;
    req->srdata.buffer_size = alloclen;
    req->srdata.control = 0;
    req->isUSBLost = !mas_working;
    dev->read_p = dev->read_buf;
    dev->read_rest = sizeof(sideband_app_request);
    atomic_set(&dev->status, VPD_BUF_REQ_PUT_DONE);
    
    wake_up_interruptible(&dev->req_rq);
    if (dev->async_queue)
    {
        kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
    }
    
    if (VPD_BUF_RSP_PUT_DONE != atomic_read(&dev->status))
    {
	tmp = itunes_wait_event_interruptible_timeout(dev->rsp_rq, (VPD_BUF_RSP_PUT_DONE == atomic_read(&dev->status)), ITUNES_VPD_INQUIRY_TIMEOUT);
	if (tmp < 0)
	{
	    ret = -ERESTARTSYS;
	    goto out;
	}
    }
    if (VPD_BUF_RSP_PUT_DONE != atomic_read(&dev->status))
    { /* Just timeout */
	ret = -ERESTARTSYS;
	goto out;
    }
    
    /* Waked up */
    atomic_set(&dev->status, VPD_BUF_RSP_GET_ING);
    len = rsp->srdata.data_size + 4;
    data[0] = rsp->srdata.device_type;
    data[1] = rsp->srdata.page_number;
    data[2] = rsp->srdata.reserved;
    data[3] = rsp->srdata.data_size;
    memcpy(&data[4], rsp->srdata.data, MAX_SIDEBAND_PACKETSIZE);
    add_in_data(tdata, data, min(len, alloclen));
    atomic_set(&dev->status, VPD_BUF_RSP_GET_DONE);
    
    memset(req, 0, VPD_READBUFFER_SIZE);
    dev->read_p = dev->read_buf;
    dev->read_rest = 0;
    memset(rsp, 0, VPD_WRITEBUFFER_SIZE);
    dev->write_p = dev->write_buf;
    dev->write_rest = sizeof(sideband_app_response);
out:
    if (dev)
    {
    	atomic_set(&dev->status, VPD_BUF_EMPTY);
	wake_up_interruptible(&dev->req_wq);
    }
    return ret;
}

static int iTunes_sideband_modinit(void)
{
    int ret = 0;
    spin_lock_init(&iTunes_sideband_driver.lock);
    INIT_LIST_HEAD(&iTunes_sideband_driver.devices);
    iTunes_sideband_driver.device_nr = 0;
    ret = iTunes_sideband_dev_create();
    if (ret < 0)
    {
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t%s%d\r\n", __FILE__, __LINE__, __FUNCTION__, "Create sideband devices failed: ", ret);
	return ret;
    }
    ret = misc_register(&iTunes_sideband_miscdev);
    if (ret < 0)
    {
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t%s%d\r\n", __FILE__, __LINE__, __FUNCTION__, "Register sideband device failed: ", ret);
	iTunes_sideband_dev_destroy();
	return ret;
    }
    return ret;
}

static void iTunes_sideband_modexit(void)
{
    int ret = 0;
    ret = misc_deregister(&iTunes_sideband_miscdev);
    if (ret < 0)
    {
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t%s%d\r\n", __FILE__, __LINE__, __FUNCTION__, "unregister sideband device failed: ", ret);
	/* no return */
    }
    iTunes_sideband_dev_destroy();
}

static void iTunes_sideband_mas_lost(unsigned long data)
{
    struct iTunes_sideband_dev *dev  = (struct iTunes_sideband_dev *)NULL;
    sideband_app_request *req;
    struct list_head *ptr;
    
    spin_lock(&iTunes_sideband_driver.lock);
    if (list_empty(&iTunes_sideband_driver.devices))
    {
    	spin_unlock(&iTunes_sideband_driver.lock);
	printk("[%s, line %04d], %s(): \r\t\t\t\t\t\t\t\t%s\r\n", __FILE__, __LINE__, __FUNCTION__, "No devices!");
	goto out;
    }
    list_for_each(ptr, &iTunes_sideband_driver.devices)
    {
    	dev = list_entry(ptr, struct iTunes_sideband_dev, list);
	if (ITUNES_SIDEBAND_MINOR == dev->minor)
	{
	    break;
	}
    }
    spin_unlock(&iTunes_sideband_driver.lock);
    
    if (0 == dev->open_count)
    {
        if (run_sideband_app())
	{
		goto out;
	}
    }
    req = (sideband_app_request *)dev->read_buf;
   
    /* It can happen at any time! */
    if (VPD_BUF_EMPTY != atomic_read(&dev->status))
    {
	if (itunes_wait_event_interruptible_timeout(dev->req_wq, (VPD_BUF_EMPTY == atomic_read(&dev->status)), ITUNES_VPD_INQUIRY_TIMEOUT) < 0)
	    goto out;
    }
    if (VPD_BUF_EMPTY != atomic_read(&dev->status))
	goto out;
    
    atomic_set(&dev->status, VPD_BUF_REQ_PUT_ING);
    memset(req, 0, VPD_READBUFFER_SIZE);
    req->isUSBLost = !mas_working;
    dev->read_p = dev->read_buf;
    dev->read_rest = sizeof(sideband_app_request);
    atomic_set(&dev->status, VPD_BUF_REQ_PUT_DONE);
    
    wake_up_interruptible(&dev->req_rq);
    if (dev->async_queue)
    {
        kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
    }
out:
    return;
}

DECLARE_TASKLET(iTunes_sideband_mas_lost_tasklet, iTunes_sideband_mas_lost, (unsigned long) 0);

#endif  // CONFIG_ITUNES_SIDEBAND

/* Mutex lock*/
static spinlock_t storage_lock;
static DECLARE_MUTEX(storage_sem);     
static DECLARE_MUTEX_LOCKED (storage_sem_start);
static DECLARE_MUTEX_LOCKED (storage_sem_work);
static int storage_thread_terminating;
static int function_disable_flag = -1;
static int do_inquiry (struct usb_storage_threaddata *tdata);
static int do_mode_select (struct usb_storage_threaddata *tdata);
static int do_mode_sense (struct usb_storage_threaddata *tdata);
static int do_read (struct usb_storage_threaddata *tdata);
static int do_readcapacity (struct usb_storage_threaddata *tdata);
static int do_request_sense (struct usb_storage_threaddata *tdata);
static int do_write (struct usb_storage_threaddata *tdata);
static int do_switch_cmd(struct usb_storage_threaddata *tdata);

extern int motusbd_load(int status);

#ifdef CONFIG_USBD_ISP_BUS
/* BULK IN urb */
static struct urb *g_in_urb = 0;
static int g_in_urb_flag = 0;

/* define DMA buffer size */
#define MIN_DMA_OUT_SIZE  512 
#define MAX_DMA_OUT_SIZE  (64 * 1024) 

#define MIN_DMA_IN_SIZE   512 
#define MAX_DMA_IN_SIZE   (64 * 1024) 

#define MIN_DMA_SIZE  512   /* min(MIN_DMA_IN_SIZE, MIN_DMA_OUT_SIZE) */
#define MAX_DMA_SIZE  (64 * 1024) /* max(MAX_DMA_IN_SIZE, MAX_DMA_OUT_SIZE */ 

#endif

//endpoints index
#define BULK_OUT    0
#define BULK_IN     1

/*process received urb, alloc 8 urbs for buffer*/
#ifdef CONFIG_USBD_ISP_BUS
#define STORAGE_URB_BUFFER_NUM  2
#else
#define STORAGE_URB_BUFFER_NUM  16
#endif

#ifdef CONFIG_USBD_ISP_BUS
extern struct list_head dma_write_buf_pool_head;
extern struct list_head dma_read_buf_pool_head;

static struct dma_buf_pool_s * g_write_dma_buf=NULL;
static struct dma_buf_pool_s * g_read_dma_buf=NULL;
extern struct dma_buf_pool_s dma_write_buf_pool[];
extern struct dma_buf_pool_s dma_read_buf_pool[];

#ifdef CONFIG_IPM_DEEPIDLE
extern int cam_ipm_hook(void);
extern void cam_ipm_unhook(void);
#endif
#endif                                                                                
struct storage_urb_buffer
{
    struct urb      *urb_buffer[STORAGE_URB_BUFFER_NUM];
#ifdef CONFIG_USBD_ISP_BUS
    unsigned char * buffer[STORAGE_URB_BUFFER_NUM];
#endif
    unsigned long   urb_index;
};
static struct storage_urb_buffer storage_urb_buf;
static u8 urb_max_num;
extern int storage_recv_urb (struct urb *urb, int rc);
extern int storage_send_urb (struct urb *urb, int rc);

static int urb_buffer_alloc(struct usb_function_instance *function)
{
    int i;
    struct urb *temp;

    down(&storage_sem);     //Mutex
    memset(&storage_urb_buf, 0, sizeof(struct storage_urb_buffer));
    for(i = 0; i < STORAGE_URB_BUFFER_NUM; i++)
    {
#ifdef CONFIG_USBD_ISP_BUS
        temp = usbd_alloc_urb (function, BULK_OUT, 0, storage_recv_urb);
        if(!temp)
            break;
	storage_urb_buf.buffer[i] = (unsigned char *)__get_free_pages(GFP_KERNEL, 4);
	temp->buffer = storage_urb_buf.buffer[i];
	if(temp->buffer == NULL)	{
        	usbd_dealloc_urb(temp);
		break;
	}
	temp->buffer_length = MAX_DMA_OUT_SIZE;
	temp->request_length = MAX_DMA_OUT_SIZE;
#else
        temp = usbd_alloc_urb (function, BULK_OUT, 64, storage_recv_urb);
        if(!temp)
            break;
#endif
        storage_urb_buf.urb_buffer[i] = temp;
        urb_max_num++;
    }
    if(i == 0)
    {
        dbgPRINT("alloc urb error line %d", __LINE__);
        up(&storage_sem);     //Mutex
        return -1;
    }

#ifdef CONFIG_USBD_ISP_BUS
    g_in_urb = usbd_alloc_urb (function, BULK_IN, 0, storage_send_urb);
    if (g_in_urb) {
        g_in_urb->buffer = (unsigned char *)__get_free_pages(GFP_KERNEL, 4);
	g_in_urb->dma_physaddr = (unsigned long)g_in_urb->buffer;
	if(!g_in_urb->buffer) {
            usbd_dealloc_urb(g_in_urb);
	    g_in_urb = 0;
        } else {
            g_in_urb->buffer_length  = MAX_DMA_IN_SIZE;
            g_in_urb->request_length = MAX_DMA_IN_SIZE;
	    g_in_urb->dma_mode = 1;
        }
    }
#endif
    up(&storage_sem);     //Mutex
    return 0;   //succ
}

static void urb_buffer_free(void)
{
    struct urb *temp;
    int i;

    down(&storage_sem);     //Mutex
    for(i = 0; i < STORAGE_URB_BUFFER_NUM; i++)
    {
        temp = storage_urb_buf.urb_buffer[i];
        if(temp)  {
#ifdef CONFIG_USBD_ISP_BUS
	    free_pages((unsigned long)storage_urb_buf.buffer[i], 4);
	    temp->buffer = 0;
#endif
            usbd_dealloc_urb(temp);
	}
    }
    urb_max_num = 0;
    memset(&storage_urb_buf, 0, sizeof(struct storage_urb_buffer));

#ifdef CONFIG_USBD_ISP_BUS
    if (g_in_urb) {
        if(g_in_urb->buffer)	{
	        free_pages((unsigned long)g_in_urb->buffer, 4);
		g_in_urb->buffer = 0;
	}
        usbd_dealloc_urb(g_in_urb);
        g_in_urb = 0;
    }
#endif

    up(&storage_sem);     //Mutex
}

static struct urb* get_urb_buffer(void)
{
    struct urb *temp;
    temp = storage_urb_buf.urb_buffer[storage_urb_buf.urb_index % urb_max_num];
#ifndef CONFIG_USBD_ISP_BUS    
    storage_urb_buf.urb_index++;
#endif
    if(!temp)
        return NULL;
#ifdef CONFIG_USBD_ISP_BUS
    temp->buffer = storage_urb_buf.buffer[storage_urb_buf.urb_index % urb_max_num];
    storage_urb_buf.urb_index++;

    temp->dma_mode = 0;
#endif

    return temp;    //succ
}

/*process all cards open, close*/
#define OPEN_CARD   1
#define CLOSE_CARD  2
static int storage_open_card_flag = 0;

static int
storage_open_card(struct usb_storage_private *private)
{
    struct usb_storage_threaddata *tdata;
    mm_segment_t fs;
    int i, j, fd;

    if(!private)
    {
        //storage_thread_terminating = 1;
        dbgPRINT (":  switch error line %d", __LINE__);
        return -1;
    }
    tdata = &(private->tdata);
    dbgPRINT (":  open file partition line %d", __LINE__);
    /* Init file description and blocks number*/
    for(i = 0; i < MAX_FAT_PARTITION; i++)
    {
        tdata->real_fd[i] = -1;
        tdata->num_blocks[i] = -1;
    }
    /* Open file*/
    i = j = 0;
    fd = -1;
    for(i = 0; i < MAX_FAT_PARTITION; i++)
    {
        SAVE_FS_DS(fs);
        fd = open(device_node[i], O_RDWR, 0);
        RESTORE_FS_DS(fs);
        if(fd >= 0)
        {
            dbgPRINT (":open file fd %d %s line %d", fd, device_node[i], __LINE__);
            off_t max_len = 0; 
            SAVE_FS_DS(fs);
            if((max_len = lseek(fd, 0, 2 /*SEEK_END*/)) != (off_t)-1)
            {
                dbgPRINT (":open file success max_len=%ld line %d", max_len, __LINE__);
                tdata->logic_units++;
                tdata->real_fd[j] = fd;
                tdata->num_blocks[j] = max_len / STORAGE_BLOCK_SIZE;
                j++;
            }
            RESTORE_FS_DS(fs);
        } else
        {
            dbgPRINT (":open file error fd %d %s line %d", fd, device_node[i], __LINE__);
        }
    }
    //Clear failured fd
    j = 0;
    for(i = 0; i < MAX_FAT_PARTITION; i++)
    {
        if(tdata->real_fd[i] >= 0)
        {
            if(tdata->num_blocks[i] == -1)
            {
                SAVE_FS_DS(fs);
                close(tdata->real_fd[i]);
                RESTORE_FS_DS(fs);
                tdata->real_fd[i] = -1;
                tdata->num_blocks[i] = -1;
                tdata->logic_units--;
            }
        } else
            j++;
    }
    if(j == MAX_FAT_PARTITION)  //All fat partion can not open
    {
        dbgPRINT (":file error line %d", __LINE__);
        //storage_thread_terminating = 1;
        return -1;
    }
    tdata->logic_units--;
    dbgPRINT (":open file success lun %d line%d", tdata->logic_units, __LINE__);
    return 0;
}

static int
storage_close_card(struct usb_storage_private *private)
{
    struct usb_storage_threaddata *tdata;
    mm_segment_t fs;
    int i;

    if(!private)
    {
        storage_thread_terminating = 1;
        dbgPRINT (":  switch error line %d", __LINE__);
        return -1;
    }
    tdata = &(private->tdata);
    dbgPRINT (":show down all file partition line %d", __LINE__);
    /*shut down all fat partion*/
    for(i = 0; i < MAX_FAT_PARTITION; i++)
    {
        if(tdata->real_fd[i] >= 0)
        {
            SAVE_FS_DS(fs);
            close(tdata->real_fd[i]);
            RESTORE_FS_DS(fs);
            tdata->real_fd[i] = -1;
            tdata->num_blocks[i] = -1;
        }
    }
    return 0;
}
#ifndef CONFIG_USBD_ISP_BUS
static int
storage_sync_card(int fd)
{
	struct file * file;
	struct dentry * dentry;
	struct inode * inode;
	int ret;

	ret = -EBADF;
	file = fget(fd);
	if (!file)
        return ret;
	dentry = file->f_dentry;
	inode = dentry->d_inode;
    fsync_dev(inode->i_rdev);
    destroy_buffers(inode->i_rdev);
    return 0;
}
#endif
/*SCSI switch command flag*/
static int scsi_switch_from_udisk = -1;
static int mpt_current_lun_num = 0;
/*proc unmount/mount process*/
static int mount_res = 0;
#define MAX_WRITE_LEN 10
static ssize_t
storage_umount_write (struct file *file, const char *buf, size_t count, loff_t * pos)
{
    char feedback[MAX_WRITE_LEN + 1];
    size_t n = count;
    size_t l;
    char c;

    MOD_INC_USE_COUNT;
    feedback[0] = 0;
    if (n > 0)
    {
        l = MIN (n, MAX_WRITE_LEN);
        if (copy_from_user (feedback, buf, l))
        {
            count = -EFAULT;
        } else
        {
            if (l > 0 && feedback[l - 1] == '\n')
            {
                l -= 1;
            }
            feedback[l] = 0;
            n -= l;
            // flush remainder, if any
            while (n > 0)
            {
                // Not too efficient, but it shouldn't matter
                if (copy_from_user (&c, buf + (count - n), 1))
                {
                    count = -EFAULT;
                    break;
                }
                n -= 1;
            }
        }
    }
    if (count > 0 && !strcmp (feedback, "Successed"))
    {
        //mount_res = 0;
        mount_res++;
    }
    else if (count > 0 && !strcmp (feedback, "Failed"))
    {
        //mount_res = 0;
        mount_res--;
    }
    MOD_DEC_USE_COUNT;

    return (count);
}

int cableplug = 1;
static ssize_t
storage_unmount_read (struct file *file, char *buf, size_t count, loff_t * pos)
{
    char *s1 = "plugged\n";
    char *s2 = "unplugged\n";
    //char *s3 = "maxcards\n";
    ssize_t len;
    char *s;

    MOD_INC_USE_COUNT;
    if (cableplug)
        s = s1;
    else
        s = s2;

    if (count < strlen (s) + 1)
        len = -EINVAL;
    else if (copy_to_user (buf, s, strlen (s) + 1))
        len = -EFAULT;
    else
    len = strlen (s) + 1;
    MOD_DEC_USE_COUNT;

    return len;
}

static struct file_operations storage_umount_functions = {
read:storage_unmount_read,
write:storage_umount_write,
};

/* Module Parameters ************************************************************************* */

/* ******************************************************************************************* */

/* Mass Storage Class descriptions 
 */
static __u8 storage_in[] = {
    0x07,				// bLength
    USB_DT_ENDPOINT,		// bDescriptorType   // 0x5
    CONFIG_USBD_STORAGE_IN_ENDPOINT | IN,	// bEndpointAddress
    BULK,				// bmAttributes
    BULK_IN_PKTSIZE, 0x00,	// wMaxPacketSize   ???????????? DATA_LOG_IN_PKTSIZE
    0x00,				// bInterval
};

static __u8 storage_out[] = {
    0x07,				// bLength
    USB_DT_ENDPOINT,		// bDescriptorType   // 0x5
    CONFIG_USBD_STORAGE_OUT_ENDPOINT | OUT,	// bEndpointAddress
    BULK,				// bmAttributes
    BULK_OUT_PKTSIZE, 0x00,	// wMaxPacketSize   ???????????? DATA_LOG_IN_PKTSIZE
    0x00,				// bInterval
};

static struct usb_endpoint_descriptor * storage_endpoints[] = {
    (struct usb_endpoint_descriptor *) storage_out,
    (struct usb_endpoint_descriptor *) storage_in,
};

/* Data Interface Alternate description(s)
 */
static __u8 storage_alternate_descriptor[sizeof (struct usb_interface_descriptor)] = {
    0x09,				// bLength
    USB_DT_INTERFACE,		// bDescriptorType   // 0x04
    0x00,				// bInterfaceNumber
    0x00,				// bAlternateSetting
    sizeof (storage_endpoints) / sizeof (__u8 *), // bNumEndpoints
    0x08,				// bInterfaceClass : MASS_STORAGE
    0x06,				// bInterfaceSubClass : SCSI
    0x50,				// bInterfaceProtocol : BULK_ONLY
    0x00,				// iInterface
};

u8 storage_alt_indexes[] = { BULK_OUT, BULK_IN, };
static struct usb_alternate_description storage_data_alternate_descriptions[] = {
    {
        interface_descriptor:(struct usb_interface_descriptor *) storage_alternate_descriptor,
        iInterface:"Mass Storage Data Interface",
        endpoints:sizeof (storage_endpoints) / sizeof (__u8 *),
        endpoint_list:storage_endpoints,
        endpoint_indexes:storage_alt_indexes,
    },
};

/* Interface description(s)
 */

static struct usb_interface_description storage_interfaces[] = {
    {
        alternates:sizeof (storage_data_alternate_descriptions) /
                   sizeof (struct usb_alternate_description),
        alternate_list:storage_data_alternate_descriptions,
    },
};

/* Configuration description(s)
 */
static __u8 storage_descriptor[sizeof (struct usb_configuration_descriptor)] = {
    0x09,				// bLength
    USB_DT_CONFIG,		// bDescriptorType   // 0x2
    0x00, 0x00,			// wTotalLength
    sizeof (storage_interfaces) / sizeof (struct usb_interface_description), // bNumInterfaces
    0x01,				// bConfigurationValue
    0x00,				// iConfiguration
    BMATTRIBUTE,		// bmAttributes   ??????? 0xC0
    BMAXPOWER,			// bMaxPower
};

struct usb_configuration_description storage_description[] = {
    {
        configuration_descriptor:(struct usb_configuration_descriptor *)storage_descriptor,
        iConfiguration:"USB Mass Storage Configuration",
        bNumInterfaces:sizeof (storage_interfaces) / sizeof (struct usb_interface_description),
        interface_list:storage_interfaces,
    },
};

/* Device Description
 */ 
static struct usb_device_descriptor storage_device_descriptor = {
	bLength: sizeof(struct usb_device_descriptor),
	bDescriptorType: USB_DT_DEVICE,
	bcdUSB: __constant_cpu_to_le16(USB_BCD_VERSION),
	bDeviceClass: 0x00,
	bDeviceSubClass: 0x00,
	bDeviceProtocol: 0x00,
	bMaxPacketSize0: 0x00,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_STORAGE_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_STORAGE_PRODUCTID),
	bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_STORAGE_BCDDEVICE),
};

//endpoint description
#define STORAGE_ENDPOINTS 2
static struct usb_endpoint_request storage_endpoint_requests[STORAGE_ENDPOINTS + 1] = {
#ifdef CONFIG_USBD_ISP_BUS
    { 1, 1, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, 0xFFFF, 0xFFFF, },
    { 1, 1, 0, USB_DIR_IN | USB_ENDPOINT_BULK, 0xFFFF, 0xFFFF, },
#else
    { 1, 1, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, 64, 512, },
    { 1, 1, 0, USB_DIR_IN | USB_ENDPOINT_BULK, 64, 512, },
#endif
    { 0, },
};

struct usb_device_description storage_device_description = {
    device_descriptor: &storage_device_descriptor,
#ifdef CONFIG_USBD_HIGH_SPEED
     device_qualifier_descriptor: &storage_device_qualifier_descriptor,
#endif /* CONFIG_USBD_HIGH_SPEED */
    iManufacturer:"Motorola Inc.",
    //iProduct:"Mass Storage Device",
#ifdef CONFIG_ARCH_EZX_E680
    iProduct:"E680 Phone",
#endif
#ifdef CONFIG_ARCH_EZX_A780
    iProduct:"A780 Phone",
#endif
#ifdef CONFIG_ARCH_EZX_BARBADOS
    iProduct:"Barbados Phone",
#endif
#ifdef CONFIG_ARCH_EZX_OAHU
    iProduct:"Oahu Phone",
#endif
#ifdef CONFIG_ARCH_EZX_HAINAN
    iProduct:"Phone",
#endif
#ifdef CONFIG_ARCH_EZX_SUMATRA
#ifdef CONFIG_ITUNES_SIDEBAND
    iProduct:"Motorola Phone USB Device(E6 iTunes)",
#else 
    iProduct:"Phone",
#endif
#endif
#ifdef CONFIG_ARCH_EZX_MARTINIQUE
    iProduct:"Martinique Phone",
#endif
    iSerialNumber:"000000000000",
    endpointsRequested: STORAGE_ENDPOINTS,
    requestedEndpoints: storage_endpoint_requests,
};

/*Send CSW to host*/
static int
storage_send_CSW (struct usb_storage_private *private)
{
    struct urb *urb;
    struct usb_storage_threaddata *tdata = &private->tdata;

    dbgPRINT("Entering send CSW line %d", __LINE__);
    urb = usbd_alloc_urb(private->function, BULK_IN, MAX_CSW_LEN, NULL);
    if (!urb)
    {
        dbgPRINT(": failed to alloc CSW urb line %d", __LINE__);
        return -EINVAL;
    }
    tdata->csw.dSignature = cpu_to_le32(tdata->csw.dSignature);
    tdata->csw.dTag = cpu_to_le32(tdata->csw.dTag);
    tdata->csw.dDataResidue = cpu_to_le32(tdata->csw.dDataResidue);
    memcpy(urb->buffer, &tdata->csw, sizeof(struct CSW));
    urb->actual_length = sizeof(struct CSW);
    dbgPRINT("CSW (length %d) line %d", urb->actual_length, __LINE__);
	//TRACE_MSG32("Thomas send csw urb %x", urb);
    if (usbd_send_urb(urb))
    {
        dbgPRINT("send CSW urb error line %d", __LINE__);
        usbd_dealloc_urb(urb);
        return -EINVAL;
    }
    return 0;
}

/*
 * process RBC Command Block 
 */
static int
storage_receive_CBW (struct usb_storage_private *private, u8 * buf, unsigned int len)
{
    struct CBW *cbwp = (struct CBW *) buf;
    struct usb_storage_threaddata *tdata = &private->tdata;

    if (len != sizeof (struct CBW))
    {
        dbgPRINT(": bad CBW length (%d) line %d", len, __LINE__);
        return -1;
    }

    if (le32_to_cpu (cbwp->dSignature) != CBW_SIGNATURE)
    {
        dbgPRINT(": bad CBW signature (%x) line %d", le32_to_cpu(cbwp->dSignature), __LINE__);
        return -1;
    }

    if (cbwp->bLUN > tdata->logic_units)
    {
        dbgPRINT(": bad CBW LUN (%x) line %d", cbwp->bLUN, __LINE__);
        return -1;
    }

    tdata->cbw = *cbwp;
    tdata->cbw.dSignature = le32_to_cpu (tdata->cbw.dSignature);
    tdata->cbw.dTag = le32_to_cpu (tdata->cbw.dTag);
    tdata->cbw.dDataTransferLength = le32_to_cpu (tdata->cbw.dDataTransferLength);
    tdata->csw.dSignature = CSW_SIGNATURE;
    tdata->csw.dTag = tdata->cbw.dTag;
    tdata->csw.dDataResidue = tdata->cbw.dDataTransferLength;
    tdata->csw.bStatus = CSW_STAT_GOOD;
    tdata->data_len = 0;
    return len - sizeof (struct CBW);
}

static int
storage_process_CB (struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 logic_units = tdata->cbw.bLUN;

    if (logic_units > tdata->logic_units)
        goto unsupported;

    if (CB[0] != REQUEST_SENSE)
    {
        /* initialize sense data */
        tdata->sense.key = NO_SENSE;
        tdata->sense.info = 0;
        tdata->sense.code = SENCODE_NO_SENSE;
        tdata->sense.cmdinfo = 0;
    }

    switch (CB[0])
    {
    case FORMAT_UNIT:
        dbgPRINT("FORMAT_UNIT line %d", __LINE__);
        goto unsupported;
    case INQUIRY:
        dbgPRINT("INQUIRY line %d", __LINE__);
        tdata->stat.inquiry++;
        if (tdata->data_len)
            goto phase_error;
        return do_inquiry (tdata);
    case MODE_SELECT:
        dbgPRINT ("MODE_SELECT line %d", __LINE__);
        tdata->stat.mode_select++;
        return do_mode_select (tdata);
    case MODE_SENSE:
        dbgPRINT ("MODE_SENSE line %d", __LINE__);
        tdata->stat.mode_sense++;
        if (tdata->data_len)
            goto phase_error;
        return do_mode_sense (tdata);
    case PERSISTENT_RESERVE_IN:
        dbgPRINT ("PERSISTENT_RESERVE_IN line %d", __LINE__);
        goto unsupported;
    case PERSISTENT_RESERVE_OUT:
        dbgPRINT ("PERSISTENT_RESERVE_OUT line %d", __LINE__);
        goto unsupported;
    case ALLOW_MEDIUM_REMOVAL:
        dbgPRINT ("ALLOW_MEDIUM_REMOVAL line %d", __LINE__);
        goto unsupported;
    case READ_10:
        dbgPRINT ("READ_10 line %d", __LINE__);
        tdata->stat.read_10++;
        if (tdata->data_len)
            goto phase_error;
        return do_read (tdata);
    case READ_CAPACITY:
        dbgPRINT ("READ_CAPACITY line %d", __LINE__);
        tdata->stat.read_capacity++;
        if (tdata->data_len)
            goto phase_error;
        return do_readcapacity (tdata);
    case RELEASE:
        dbgPRINT ("RELEASE line %d", __LINE__);
        goto unsupported;
    case REQUEST_SENSE:
        dbgPRINT ("REQUEST_SENSE line %d", __LINE__);
        tdata->stat.request_sense++;
        if (tdata->data_len)
            goto phase_error;
        return do_request_sense (tdata);
    case RESERVE:
        dbgPRINT ("RESERVE line %d", __LINE__);
        goto unsupported;
    case START_STOP:
        dbgPRINT ("START_STOP line %d", __LINE__);
        tdata->stat.start_stop++;
        if (tdata->data_len)
            goto phase_error;
      /* do nothing */
      break;
    case SYNCHRONIZE_CACHE:
        dbgPRINT ("SYNCHRONIZE_CACHE line %d", __LINE__);
        goto unsupported;
    case TEST_UNIT_READY:
        dbgPRINT ("TEST_UNIT_READY line %d", __LINE__);
        tdata->stat.test_unit_ready++;
        if (tdata->data_len)
            goto phase_error;
      /* do nothing */
      break;
    case VERIFY:
        dbgPRINT ("VERIFY line %d", __LINE__);
        tdata->stat.verify++;
        if (tdata->data_len)
            goto phase_error;
        /* do nothing */
        break;
    case WRITE_10:
        dbgPRINT ("WRITE_10 line %d", __LINE__);
        tdata->stat.write_10++;
        return do_write (tdata);
    case WRITE_BUFFER:
        dbgPRINT ("WRITE_BUFFER line %d", __LINE__);
        tdata->stat.write_buffer++;
        if (tdata->data_len)
            goto phase_error;
        /* do nothing */
        break;
    default:
        dbgPRINT(": unknown RBC command (%02x) line %d", CB[0], __LINE__);
        goto unsupported;
    }
    return 0;

phase_error:
    dbgPRINT(": phase error on RBC command %02x line %d", CB[0], __LINE__);
    tdata->csw.bStatus = CSW_STAT_PERR;
    return -EINVAL;
unsupported:
    tdata->stat.unsupported++;
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_INVALID_COMMAND;
    return -EINVAL;
}

#ifdef CONFIG_USBD_ISP_BUS
int storage_send_urb (struct urb *urb, int rc)
{
	g_in_urb_flag = 0;
	if (g_read_dma_buf != NULL)
	{
		mas_release_dma_buf(&dma_read_buf_pool_head, g_read_dma_buf);
		g_read_dma_buf = NULL;
		g_in_urb->buffer = (unsigned char *)g_in_urb->dma_physaddr;
	}
	return 0;
}


int release_read_buffer (struct urb *urb, int rc)
{
	if (g_read_dma_buf != NULL)
	{
		mas_release_dma_buf(&dma_read_buf_pool_head, g_read_dma_buf);
		g_read_dma_buf = NULL;
		urb->buffer = NULL;
	}
	return 1;
}
#endif

/* storage_recv_urb - called to indicate URB has been received
 * @urb - pointer to struct urb
 *
 * Return non-zero if we failed and urb is still valid (not disposed)
 * NOTE: This function is called in bottom half context.
 */
int
storage_recv_urb (struct urb *urb, int rc)
{
    struct usb_function_instance *function; 
    struct usb_storage_private *private;
    struct usb_storage_threaddata *tdata;
    struct urb *new_urb;

#ifdef CONFIG_USBD_ISP_BUS
    int current_length = MIN_DMA_OUT_SIZE;
#endif

    switch(rc)
    {
    case RECV_OK:
    case RECV_CANCELLED:
        break;
    default:
        printk("Enter recv urb error rc %d line %d", rc, __LINE__);
        return 1;
    }
    RETURN_ZERO_IF(!urb);

    dbgPRINT("Enter recv urb %p len %u line %d", urb, urb->actual_length, __LINE__);

    function = urb->function_instance;
    RETURN_ZERO_IF(!function);
    private = (struct usb_storage_private *)usbd_function_get_privdata(function);
    RETURN_ZERO_IF(!private);
    RETURN_ZERO_IF(!private->function)
    tdata = &(private->tdata);

#ifdef CONFIG_USBD_ISP_BUS
    //alloc urb for next CBW commands
    new_urb = get_urb_buffer();
    if (!new_urb)
    {
        spin_lock (&storage_lock);
        private->devstate = STATE_INVALID;
        //TRACE_MSG32("error alloc next urb %x", new_urb);
        //STALL bulk in pipe by USB Mass Storage Class 每 Bulk Only Transport 6.6.2
        //set high speed 0
        usbd_endpoint_halted(function,
                usbd_endpoint_bEndpointAddress(function, BULK_IN, 0));
        printk("never recv urb line %d", __LINE__);
        spin_unlock(&storage_lock);
        return 0;
    }
#endif
    //TRACE_MSG32("Thomas recv urb %x", urb);
    //TRACE_MSG32("Thomas recv urb actual %x", urb->actual_length);
    //TRACE_MSG32("Thomas recv urb request len %x", urb->request_length);
    // push the data up
    if (tdata->busy)
    {
        printk("receive in busy state (%d) line %d", private->devstate, __LINE__);
        printk("Enter urb %p line %d", urb, __LINE__);
        goto recv_urb;
    }

    dbgPRINT("current devstate %d line %d", private->devstate, __LINE__);
    switch (private->devstate)
    {
    case STATE_DO:
        //TRACE_MSG32("Thomas recv do urb %x", urb);
        //TRACE_MSG32("Thomas recv do urb actual %x", urb->actual_length);
        //TRACE_MSG32("Thomas recv data len %x", tdata->data_len);
        //TRACE_MSG32("Thomas recv trans len %x", tdata->cbw.dDataTransferLength);
        spin_lock (&storage_lock);
        if (tdata->data_len + urb->actual_length > tdata->cbw.dDataTransferLength)
        {
            printk(": bad DATA length (%d + %d > %d)\n",
                    tdata->data_len, urb->actual_length,
                    tdata->cbw.dDataTransferLength);
            private->devstate = STATE_INVALID;
            spin_unlock(&storage_lock);
            break;
        }
#ifndef CONFIG_USBD_ISP_BUS
        memcpy(tdata->data_buf + tdata->data_len, urb->buffer, urb->actual_length);
        tdata->data_len += urb->actual_length;
#else
        tdata->data_len += urb->actual_length;
        if (tdata->data_len < tdata->cbw.dDataTransferLength)
        	new_urb->buffer = tdata->data_buf + tdata->data_len;
#endif
        spin_unlock(&storage_lock);
        dbgPRINT("data len %d line %d", tdata->data_len, __LINE__);
        dbgPRINT("urb %p line %d", urb, __LINE__);
        if (tdata->data_len >= tdata->cbw.dDataTransferLength)
        {
            //TRACE_MSG32("Thomas recv all do urb %x", urb);
            //TRACE_MSG32("Thomas recv all do data len %x", tdata->data_len);
            spin_lock (&storage_lock);
            /* all data received */
            tdata->busy = 1;        /*enter thread, and process command bolck*/
            spin_unlock(&storage_lock);
            dbgPRINT("data len %d line %d", tdata->data_len, __LINE__);
            dbgPRINT("busy = %d line %d", tdata->busy, __LINE__);
            dbgPRINT("data_buf = %p line %d", tdata->data_buf, __LINE__);
            up(&storage_sem_work);      //call running thread
        }

#ifdef CONFIG_USBD_ISP_BUS
	current_length = tdata->cbw.dDataTransferLength - (tdata->data_len);
	if(current_length > MAX_DMA_OUT_SIZE) 
		current_length = MAX_DMA_OUT_SIZE;
#endif

        break;
    case STATE_IDLE:
        // Handle CBW command
        //TRACE_MSG32("Thomas recv idle urb %x", urb);
        //TRACE_MSG32("Thomas recv idle urb actual %x", urb->actual_length);
        //TRACE_MSG32("Thomas recv idle urb request len %x", urb->request_length);
        spin_lock (&storage_lock);
        if (storage_receive_CBW (private, urb->buffer, urb->actual_length) < 0)
        {
            private->devstate = STATE_INVALID;
            /*
             *STALL bulk in pipe by USB Mass Storage Class 
             *每 Bulk Only Transport 6.6.1. Set high speed 0.
             */
            //usbd_endpoint_halted(function,
                    //usbd_endpoint_bEndpointAddress(function, BULK_IN, 0));
            printk("set endpoint halted line %d, urb %x, buf %x, len %x\r\n", __LINE__, (unsigned int)urb, (unsigned int)urb->buffer, urb->actual_length);
            spin_unlock(&storage_lock);
            break;
        }
        spin_unlock(&storage_lock);

        dbgPRINT("current cbw %d line %d", tdata->cbw.CB[0], __LINE__);
        dbgPRINT("current cbw length %d line %d", 
                tdata->cbw.dDataTransferLength, __LINE__);
        //If Host expects no data transfers, send csw to host by 5.1
        if (tdata->cbw.dDataTransferLength == 0)
        {
            u8 *CB = tdata->cbw.CB;
            spin_lock (&storage_lock);
            private->devstate = STATE_DN;
            tdata->busy = 1;
            dbgPRINT("busy = %d line %d", tdata->busy, __LINE__);
            if (CB[0] == USB_SWITCH_FROM_UDISK) /*recv SCSI switch command*/
                do_switch_cmd(tdata);

            spin_unlock(&storage_lock);
            up(&storage_sem_work);      //call running thread
            break;
        } else  //CBW is valid, handle SCSI commands
        {
            spin_lock (&storage_lock);
#ifndef CONFIG_USBD_ISP_BUS
            if (tdata->data_buf)
                kfree(tdata->data_buf);
#endif
            tdata->data_buf = NULL;
            tdata->data_len = tdata->data_alloclen = 0;

#ifndef CONFIG_USBD_ISP_BUS
            tdata->data_buf = kmalloc (tdata->cbw.dDataTransferLength, GFP_ATOMIC);
	    dbgPRINT("data_buf %p line %d", tdata->data_buf, __LINE__);
            spin_unlock(&storage_lock);
            if (!tdata->data_buf)   //Internal error, send csw and Stall all pipe 6.6.2
            {
                spin_lock (&storage_lock);
                private->devstate = STATE_INVALID;
                tdata->csw.bStatus = CSW_STAT_PERR;
                tdata->data_alloclen = 0;
                printk(": devstate: (%d) line %d", private->devstate, __LINE__);
                if(storage_send_CSW(private) < 0)
                {
                    private->devstate = STATE_INVALID;
                    //set high speed 0
                    usbd_endpoint_halted(function,
                            usbd_endpoint_bEndpointAddress(function, BULK_IN, 0));
                    printk("set endpoint halted line %d", __LINE__);
                }
                spin_unlock(&storage_lock);
                break;
            }
#else
            struct dma_buf_pool_s * dma_buf;
            dma_buf = mas_get_dma_buf(&dma_write_buf_pool_head);

            spin_unlock(&storage_lock);
            if (dma_buf == NULL )
            {
                spin_lock (&storage_lock);
                private->devstate = STATE_INVALID;
                tdata->csw.bStatus = CSW_STAT_PERR;
                tdata->data_alloclen = 0;
                printk(": devstate: (%d) line %d", private->devstate, __LINE__);
                if(storage_send_CSW(private) < 0)
                {
                    private->devstate = STATE_INVALID;
                    //set high speed 0
                    usbd_endpoint_halted(function,
                            usbd_endpoint_bEndpointAddress(function, BULK_IN, 0));
                    printk("set endpoint halted line %d", __LINE__);
                }
                spin_unlock(&storage_lock);
                break;
            }
            tdata->data_buf = dma_buf->dma_buf_pool_ptr;

	    g_write_dma_buf = dma_buf;

            dbgPRINT("data_buf %p line %d", tdata->data_buf, __LINE__);
#endif
            spin_lock (&storage_lock);
            tdata->data_alloclen = tdata->cbw.dDataTransferLength;
            spin_unlock(&storage_lock);

            if (tdata->cbw.bmFlags & USB_DIR_IN)    /* IN */
            {
                spin_lock (&storage_lock);
                private->devstate = STATE_DI;
                tdata->busy = 1;        /*enter thread, and process command bolck*/
                dbgPRINT("busy = %d line %d", tdata->busy, __LINE__);
                spin_unlock(&storage_lock);
                up(&storage_sem_work);      //call running thread
            } else  /* OUT */
            {
                spin_lock (&storage_lock);
                private->devstate = STATE_DO;
                spin_unlock(&storage_lock);
#ifdef CONFIG_USBD_ISP_BUS
                new_urb->buffer = tdata->data_buf;
		current_length = tdata->cbw.dDataTransferLength;
		if(current_length > MAX_DMA_OUT_SIZE) 
			current_length = MAX_DMA_OUT_SIZE;
#endif
            }
        }
        break;
    default:
        //TRACE_MSG32("invalid state %d", private->devstate);
        printk("receive in invalid state (%d) line %d",
                private->devstate, __LINE__);
        break;
    }
recv_urb:
#ifndef CONFIG_USBD_ISP_BUS
    //alloc urb for next CBW commands
    new_urb = get_urb_buffer();
    if (!new_urb)
    {
        spin_lock (&storage_lock);
        private->devstate = STATE_INVALID;
        //TRACE_MSG32("error alloc next urb %x", new_urb);
        //STALL bulk in pipe by USB Mass Storage Class 每 Bulk Only Transport 6.6.2
        //set high speed 0
        usbd_endpoint_halted(function,
                usbd_endpoint_bEndpointAddress(function, BULK_IN, 0));
        dbgPRINT("never recv urb line %d", __LINE__);
        spin_unlock(&storage_lock);
        return 0;
    }
#endif
    //TRACE_MSG32("Thomas next cbw urb %x", new_urb);
    //TRACE_MSG32("Thomas next cbw urb request len %d", 
            //new_urb->request_length);
    //TRACE_MSG32("Thomas next cbw urb buffer len %d", 
                        //new_urb->buffer_length); 
#ifdef CONFIG_USBD_ISP_BUS
    new_urb->request_length = current_length;
    if(current_length > MIN_DMA_OUT_SIZE) 
	new_urb->dma_mode = 1;
#endif

    usbd_start_recv (new_urb);
    return 0;
}

//Process host send usb disk reset event
static void storage_recv_RESET(struct usb_function_instance *function)
{
    struct usb_storage_private *private;
    struct usb_storage_threaddata *tdata;

    if(!function)
        return;
    private = (struct usb_storage_private *)usbd_function_get_privdata(function);
    if(!private)
    {
        dbgPRINT (": no private line %d", __LINE__);
        return;
    }


    dbgPRINT (": recv usb disk reset line %d", __LINE__);
    tdata = &private->tdata;
    tdata->busy = 0;

#ifndef CONFIG_USBD_ISP_BUS
    if (tdata->data_buf)
        kfree (tdata->data_buf);
#else
    if (g_write_dma_buf != NULL)
    { 
	mas_release_dma_buf(&dma_write_buf_pool_head, g_write_dma_buf);
	g_write_dma_buf = NULL;
    }
#endif
    tdata->data_buf = NULL;

    tdata->data_len = tdata->data_alloclen = 0;
    private->devstate = STATE_IDLE; //set idle states
    dbgPRINT(": init devstate: (%d) line %d", private->devstate, __LINE__);

    storage_open_card_flag = CLOSE_CARD; //close all cards
    up(&storage_sem_work);      //call running thread
    storage_open_card_flag = OPEN_CARD;  //open all cards
    up(&storage_sem_work);      //call running thread
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
int
storage_recv_setup (struct usb_device_request *request)
{
    struct urb *urb;
    struct usb_function_instance *function; 
    struct usb_storage_private *private = &storage_private;

    RETURN_EINVAL_IF(!request);
    RETURN_EINVAL_IF(!private);
    function = private->function;
    RETURN_EINVAL_IF(!function);

    down(&storage_sem);     //Mutex
    // handle Mass Storage Class-Specific Request
    // c.f. USB Mass Storage Class Bulk-Only Transport 3.1, 3.2
    if ((request->bmRequestType & (USB_REQ_TYPE_MASK | USB_REQ_RECIPIENT_MASK)) !=
                                (USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE))
    {
        dbgPRINT("not class/interface request: %x line %d", request->bmRequestType, __LINE__);
        up(&storage_sem);     //Mutex
        return 0;
    }

    if ((request->bmRequestType & USB_REQ_DIRECTION_MASK))
    {
        dbgPRINT ("Device-to-Host line %d", __LINE__);
        switch (request->bRequest)
        {
        case USB_REQ_GET_MAX_LUN:
            RETURN_EINVAL_IF(!(urb = usbd_alloc_urb_ep0(function, 1, NULL)));   //One byte
            dbgPRINT ("bRequest %d line %d", request->bRequest, __LINE__);
            dbgPRINT ("partition num %d line %d",
                    private->tdata.logic_units, __LINE__);
            urb->actual_length = 1;
            if(scsi_switch_from_udisk != -1)
            {
                scsi_switch_from_udisk = -1;                //Clear scsi command flag
                urb->buffer[0] = mpt_current_lun_num;       // mpt Logic units number
            }else
            {
                urb->buffer[0] = private->tdata.logic_units;	// Normal Logic units number
            }

            if(usbd_send_urb(urb))
            {
                dbgPRINT ("send urb error line %d", __LINE__);
                usbd_dealloc_urb(urb);
            }
#ifndef CONFIG_USBD_ISP_BUS
            //Alloc usr for CBW command
            urb = get_urb_buffer(); //get urb from buffer
            RETURN_EINVAL_IF(!urb);
            dbgPRINT("recv urb success %p line %d", urb, __LINE__);
            usbd_start_recv (urb);
#endif
            break;
        default:
            dbgPRINT("Unknown request: %x line %d", request->bRequest, __LINE__);
        }
    }
    else
    {
        dbgPRINT("Host-to-Device, line %d", __LINE__);
        switch (request->bRequest)
        {
        case USB_REQ_BO_MASS_STORAGE_RESET:
            storage_recv_RESET(function);
            break;
        default:
            dbgPRINT("Unknown request: %x line %d", request->bRequest, __LINE__);
        }
    }

    up(&storage_sem);     //Mutex
    return 0;
}

/*
 * USB Device Functions
 */
/* proc interface */
static int
storage_read_proc (char *page, char **start, off_t off, int count, int *eof, void *data)
{
    char *p = page;
    struct usb_storage_private *private = &storage_private;

    spin_lock_bh(&storage_lock);	/* prevent event handler */
    if (private->function)
    {
        struct usb_storage_threaddata *tdata = &private->tdata;
        p += sprintf (p, "state: %d\n", private->devstate);
        p += sprintf (p, "transfer statistics:\n");
        p += sprintf (p, "read_blocks\t%u\n", tdata->stat.read_blocks);
        p += sprintf (p, "write_blocks\t%u\n", tdata->stat.write_blocks);
        p += sprintf (p, "command statistics:\n");
        p += sprintf (p, "inquiry\t%u\n", tdata->stat.inquiry);
        p += sprintf (p, "mode_select\t%u\n", tdata->stat.mode_select);
        p += sprintf (p, "mode_sense\t%u\n", tdata->stat.mode_sense);
        p += sprintf (p, "read_10\t%u\n", tdata->stat.read_10);
        p += sprintf (p, "read_capacity\t%u\n", tdata->stat.read_capacity);
        p += sprintf (p, "request_sense\t%u\n", tdata->stat.request_sense);
        p += sprintf (p, "start_stop\t%u\n", tdata->stat.start_stop);
        p += sprintf (p, "test_unit_ready\t%u\n", tdata->stat.test_unit_ready);
        p += sprintf (p, "verify\t%u\n", tdata->stat.verify);
        p += sprintf (p, "write_10\t%u\n", tdata->stat.write_10);
        p += sprintf (p, "write_buffer\t%u\n", tdata->stat.write_buffer);
        p += sprintf (p, "unsupported\t%u\n", tdata->stat.unsupported);
    }
    spin_unlock_bh(&storage_lock);
    return p - page;
}

/*
 * U disk thread, handle CBW and send CSW.
 */
static int
storage_thread (void *data)
{
    struct usb_storage_private *private = NULL;
    struct usb_storage_threaddata *tdata = NULL;
    struct usb_function_instance *function = NULL; 
    struct urb *urb = NULL;

    /* set thread is daemon*/
    daemonize();
    reparent_to_init();
    spin_lock_irq(&current->sigmask_lock);
    sigemptyset(&current->blocked);
    recalc_sigpending(current);
    spin_unlock_irq(&current->sigmask_lock);

    private = &storage_private;
    if(!private)
    {
        storage_thread_terminating = 1;
        dbgPRINT (":  switch error line %d", __LINE__);
        goto end;
    }
    tdata = &(private->tdata);

    up(&storage_sem_start);
    /*
     * Loop process command block
     */
    while(!storage_thread_terminating)
    {
        down(&storage_sem_work); // wait for someone to tell us to do something

        if(storage_open_card_flag == OPEN_CARD) //first open related cards
        {
            storage_open_card_flag = 0; //close all cards
            if(storage_open_card(private))
            {
                continue;
            }
        }
        if(storage_open_card_flag == CLOSE_CARD) //close related cards
        {
            storage_open_card_flag = 0; //close all cards
            storage_close_card(private);
        }

        dbgPRINT("loop line %d", __LINE__);
        if(tdata->busy)
        {
            spin_lock_bh(&storage_lock);	/* prevent event handler */
            if(private->devstate == STATE_DN)
            {
                goto send_CSW;
            }
            spin_unlock_bh(&storage_lock);
            dbgPRINT("data_buf %p line %d", tdata->data_buf, __LINE__);

            //Handle SCSI commands
            if (storage_process_CB(tdata) < 0)
            {   //Internal error, send csw and Stall all pipe
                dbgPRINT("process CB error line %d", __LINE__);
                private->tdata.csw.bStatus = CSW_STAT_FAILED;
                //goto send_CSW;
            }
            spin_lock_bh(&storage_lock);	/* prevent event handler */
            if(private->devstate == STATE_DI)
            {
#ifdef CONFIG_USBD_ISP_BUS
		if((g_in_urb) && (!g_in_urb_flag) && (tdata->cbw.dDataTransferLength > MIN_DMA_IN_SIZE))   {
			g_in_urb_flag = 1;
			urb = g_in_urb;
		} else   {
	    		if (tdata->cbw.CB[0] == READ_10)
	                	urb = usbd_alloc_urb (private->function, BULK_IN, 0, release_read_buffer);
			else
	                	urb = usbd_alloc_urb (private->function, BULK_IN, tdata->cbw.dDataTransferLength, NULL);
		}
#else
                urb = usbd_alloc_urb (private->function, BULK_IN,
        	                tdata->cbw.dDataTransferLength, NULL);
#endif
                if (!urb)
                {
                    private->devstate = STATE_INVALID;
                    //set high speed 0
		    printk("\n MASS STORAGE NO MEMORY! tell UI? \n");//LIN warning

		/* e12707 DEC 05*/ function = private->function;

                    usbd_endpoint_halted(function, 
                            usbd_endpoint_bEndpointAddress(function, BULK_IN, 0));
		

                    printk("set endpoint halted line %d", __LINE__);
                    //continue;
                    goto process_continue;
                }
#ifdef CONFIG_USBD_ISP_BUS		
	    	if (tdata->cbw.CB[0] == READ_10)
		{
            		tdata->data_buf = g_read_dma_buf->dma_buf_pool_ptr;
			urb->buffer = tdata->data_buf;
			mas_release_dma_buf(&dma_write_buf_pool_head, g_write_dma_buf);
	    		g_write_dma_buf = NULL;
		}
		else
                	memcpy(urb->buffer, tdata->data_buf, tdata->data_len);
#else
                memcpy(urb->buffer, tdata->data_buf, tdata->data_len);

#endif
                urb->actual_length = tdata->data_len;
                tdata->csw.dDataResidue -= urb->actual_length;
                if (usbd_send_urb(urb))
                {
                    private->devstate = STATE_INVALID;

                    printk("\n MASS STORAGE NO MEMORY!  tell UI? \n");//LIN warning

                /* e12707 DEC 05*/ function = private->function;

                    usbd_endpoint_halted(function,
                            usbd_endpoint_bEndpointAddress(function, BULK_IN, 0));
                

                    printk("set endpoint halted line %d", __LINE__);
                    //continue;
#ifdef CONFIG_USBD_ISP_BUS
                    if(urb != g_in_urb) 
#endif
			usbd_dealloc_urb(urb);
                    goto process_continue;
                }
            }

send_CSW:   //Send CSW
            dbgPRINT("send CSW line %d", __LINE__);
            if(function_disable_flag == 1)
            {
                goto process_continue;
            }
            if(storage_send_CSW(private) < 0)
            {
                private->devstate = STATE_INVALID;
                //STALL bulk in pipe by USB Mass Storage Class 每 Bulk Only Transport 6.6.1
                //set high speed 0
                function = private->function;
                usbd_endpoint_halted(function,
                        usbd_endpoint_bEndpointAddress(function, BULK_IN, 0));
                printk("set endpoint halted line %d", __LINE__);
                //continue;
                goto process_continue;
            }
process_continue:

#ifdef CONFIG_USBD_ISP_BUS

  	if (g_write_dma_buf != NULL)
	{
	    if ((tdata->cbw.CB[0] != READ_10) && (tdata->cbw.CB[0] != WRITE_10))
		mas_release_dma_buf(&dma_write_buf_pool_head, g_write_dma_buf);
	    g_write_dma_buf = NULL;
	}

#endif		
            //Set u disk state is IDLE
            private->devstate = STATE_IDLE;
            tdata->busy = 0;
            dbgPRINT("busy = %d line %d", tdata->busy, __LINE__);
            spin_unlock_bh(&storage_lock);

            /*private switch scsi command*/
            if(scsi_switch_from_udisk != -1)
            {
                mpt_current_lun_num = tdata->logic_units;   //save current logic unit for mpt switch
                //need to switch to other mode.
                motusbd_load(scsi_switch_from_udisk);
            }
        }
    }   /*loop process until storage_thread_terminating = 1*/

    /*shut down all fat partion*/
    storage_close_card(private);
end: //let the process stopping us know we are done and return
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
    {
        dbgPRINT(":  thread error line %d", __LINE__);
        return -EINVAL;
    }
    return 0;
}

/**
 * storage_thread_killoff - stop command processing thread
 */
static void storage_thread_killoff(void)
{
    if (!storage_thread_terminating)
    {
        storage_thread_terminating = 1;
        up(&storage_sem_work);      //call running thread
        down(&storage_sem_start);
    }
}

/* 
 * functon enable - init private data when function enable
 */
static int 
storage_function_enable(struct usb_function_instance *function)
{
    struct usb_storage_private *private;

    if(!function)
        return -EINVAL;

    down(&storage_sem);     //Mutex
    /*memset private data*/
    memset(&storage_private, 0, sizeof(struct usb_storage_private));

    dbgPRINT (": enter function enable line %d", __LINE__);
    private = &storage_private;
    private->function = function;
    private->devstate = STATE_IDLE; //set idle states
    dbgPRINT(":  init devstate: (%d) line %d", private->devstate, __LINE__);

    /*Set private data into function instance*/
    usbd_function_set_privdata(function, (void *)private);
    up(&storage_sem);     //Mutex    

    dbgPRINT (": leave function enable line %d", __LINE__);
    return 0;
}

/* 
 * functon disable - init private data when function enable
 */
static void
storage_function_disable(struct usb_function_instance *function)
{
    //struct usb_storage_private *private;
    //struct usb_storage_threaddata *tdata;

    function_disable_flag = 1;
    urb_buffer_free();          //free urb buffer
#ifdef CONFIG_USBD_ISP_BUS
#ifdef CONFIG_IPM_DEEPIDLE
    cam_ipm_unhook();
#endif
#endif
#if 0
    if(!function)
        return -EINVAL;
    dbgPRINT (": stop thread line %d", __LINE__);

    down(&storage_sem);     //Mutex
    //Get function private 
    private = (struct usb_storage_private *)usbd_function_get_privdata(function);
    if(!private)
    {
        dbgPRINT (":  no private line %d", __LINE__);
        return -EINVAL;
    }
    private->function = NULL;
    dbgPRINT ("---> DESTROY private %x, line %d", private, __LINE__);
    private->devstate = STATE_INVALID;
    dbgPRINT(":  devstate: (%d) line %d", private->devstate, __LINE__);

    tdata = &private->tdata;
    if (tdata->data_buf)
        kfree (tdata->data_buf);
    tdata->data_buf = NULL;
    tdata->data_len = tdata->data_alloclen = 0;
    tdata->logic_units = 0;
    memset(private, 0, sizeof(struct usb_storage_private));
    up(&storage_sem);     //Mutex
#endif

    return ;
}

/* storage_event - process a device event
 *
 * NOTE: This function is called from keventd kernel thread.
 */
void
storage_event (struct usb_function_instance *function, usb_device_event_t event, int data)
{
    struct urb *new_urb;
    dbgPRINT ("--->event %d line %d", event, __LINE__);
    if (!function)
    {
        dbgPRINT ("no function event %d line %d", event, __LINE__);
        return;
    }

    switch (event)
    {
    case DEVICE_CREATE:
#ifdef CONFIG_USBD_ISP_BUS
#ifdef CONFIG_IPM_DEEPIDLE
	cam_ipm_hook();
#endif
#endif

        function_disable_flag = -1;
        urb_buffer_free();          //free urb buffer
        urb_buffer_alloc(function);  //alloc urb buffer
        storage_open_card_flag = CLOSE_CARD; //close all cards
        up(&storage_sem_work);      //call running thread
        storage_open_card_flag = OPEN_CARD;  //open all cards
        up(&storage_sem_work);      //call running thread
        break;
    case DEVICE_CONFIGURED:
#ifdef CONFIG_ITUNES_SIDEBAND
    	mas_working = TRUE;
#endif
        storage_private.devstate = STATE_IDLE;
        new_urb = get_urb_buffer();
        usbd_start_recv (new_urb);
        break;
    case DEVICE_RESET:
#ifdef CONFIG_ITUNES_SIDEBAND
	mas_working = FALSE;
	storage_private.devstate = STATE_INVALID;
#endif
        break;
    case DEVICE_DE_CONFIGURED:
#ifdef CONFIG_ITUNES_SIDEBAND
    	mas_working = FALSE;
#endif
        dbgPRINT ("---> DECONFIGURED %d line %d", event, __LINE__);
        break;
    case DEVICE_HUB_RESET:
#ifdef CONFIG_ITUNES_SIDEBAND
    	mas_working = FALSE;
        storage_private.devstate = STATE_INVALID;
	if(in_interrupt())
	{
		tasklet_schedule(&iTunes_sideband_mas_lost_tasklet);
	}
	else
	{
		iTunes_sideband_mas_lost(0);
	}
#endif
        break;
    default:
        break;
    }
}

/*
 * usb storage function ops
 */
struct usb_function_operations storage_function_ops = {
    function_enable:storage_function_enable,
	function_disable:storage_function_disable,
    event_irq:storage_event,
    recv_setup_irq:storage_recv_setup
};

/*
 * usb storage function driver
 */
struct usb_function_driver storage_function_driver = {
    name:"storage",
    fops:&storage_function_ops,
    device_description:&storage_device_description,
    bNumConfigurations:sizeof (storage_description) /
                       sizeof (struct usb_configuration_description),
    configuration_description:storage_description,
	idVendor: __constant_cpu_to_le16(CONFIG_USBD_STORAGE_VENDORID),
	idProduct: __constant_cpu_to_le16(CONFIG_USBD_STORAGE_PRODUCTID),
    bcdDevice: __constant_cpu_to_le16(CONFIG_USBD_STORAGE_BCDDEVICE),
};


/*
 * RBC functions
 */
static void
add_in_data (struct usb_storage_threaddata *tdata, void *buf, unsigned int len)
{
    if (tdata->data_len < tdata->cbw.dDataTransferLength)
    {
        dbgPRINT(": where data_buf %p line %d", tdata->data_buf, __LINE__);
        dbgPRINT(": where data_len %d line %d", tdata->data_len, __LINE__);
        dbgPRINT(": where cbw_len %d line %d", 
                tdata->cbw.dDataTransferLength, __LINE__);
        dbgPRINT(": where len %d line %d", len, __LINE__);
        memcpy (tdata->data_buf + tdata->data_len, buf,
                min (tdata->cbw.dDataTransferLength - tdata->data_len, len));
    }
    tdata->data_len += len;
}

static int
do_vpd (struct usb_storage_threaddata *tdata, int page, unsigned int alloclen)
{
  u8 data[256];
  unsigned int len = 4;

  switch (page)
    {
    case 0x00:			/* Supported vital product data pages */
      /* TODO: This response is inaccurate after add vendor specific pages -> a6510c */
      data[4 + 0] = 0x00;	/* page list */
      data[4 + 1] = 0x80;
      data[4 + 2] = 0x83;
      len += 3;
      break;
    case 0x80:			/* Unit Serial Number */
      strcpy ((char *) &data[4], storage_device_description.iSerialNumber);
      len += strlen (storage_device_description.iSerialNumber);
      break;
    case 0x83:			/* Vital Product Data Device Identification */
      sprintf (&data[4 + 4], "%s %s %s",
	       storage_device_description.iManufacturer,
	       storage_device_description.iProduct,
	       storage_device_description.iSerialNumber);
      data[4 + 0] = 0x02;	/* ASCII */
      data[4 + 1] = 0x01;
      data[4 + 3] = strlen ((char *) &data[4 + 4]);
      len += 4 + data[3];
      break;
    default:
#ifdef CONFIG_ITUNES_SIDEBAND
      if((page >= 0xC0) && (page <= 0xFF))
      {
          return do_vendor_specific_vpd(tdata, page, alloclen);
      }
#endif
      return -EINVAL;
    }
  data[0] = PERIPHERAL_DEVICE_TYPE;
  data[1] = page;		/* page code */
  data[2] = 0;
  data[3] = len - 4;
  add_in_data (tdata, data, min (len, alloclen));
  return 0;
}

static int
do_inquiry (struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 data[4 + 4 + 8 + 16 + 4];
    unsigned int len = 4 + 4 + 8 + 16 + 4;

    if (CB[1] & 0x02)		/* CMDDT */
        goto invalid_cdb_field;
    if (CB[1] & 0x01)
    {				/* EVPD */
        if (do_vpd (tdata, CB[2], CB[4]) < 0)
            goto invalid_cdb_field;
        return 0;
    }
    if (CB[2])
        goto invalid_cdb_field;

    data[0] = PERIPHERAL_DEVICE_TYPE;
    data[1] = 0;			/* not removal */
    data[2] = 0x04;		/* SPC-2 */
    data[3] = 0x02;
    data[4] = 4 + 8 + 16 + 4;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    memset (&data[8], ' ', 8 + 16 + 4);
    memcpy (&data[8], storage_device_description.iManufacturer,
    min_t (int, 8, strlen (storage_device_description.iManufacturer)));
    memcpy (&data[16], storage_device_description.iProduct,
    min_t (int, 16, strlen (storage_device_description.iProduct)));
    add_in_data (tdata, data, min (len, (unsigned int) CB[4]));
    return 0;

invalid_cdb_field:
    dbgPRINT(": Invalid CDB field line %d", __LINE__);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_INVALID_CDB_FIELD;
    return -EINVAL;
}

static int
do_mode_select (struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    if (tdata->data_len < CB[4])
    {
        dbgPRINT(": phase error on MODE_SELECT line %d", __LINE__);
        tdata->csw.bStatus = CSW_STAT_PERR;
        return -EINVAL;
    }
    /* nothing changeable */
    tdata->csw.dDataResidue -= CB[4];
    return 0;
}

static int
do_mode_sense (struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 logic_units = tdata->cbw.bLUN;
    u8 data[4 + 13];
    unsigned int len = 4 + 13;
    int page = CB[2] & 0x3f;

    if ((CB[2] & 0xc0) == 0x40)	/* PC=01b: changable values */
        goto invalid_cdb_field;
    if (page != 0x06 && page != 0x3f)	/* page 6 or all */
        goto invalid_cdb_field;
    /* mode parameter header (c.f. SPC-2 8.3, RBC 5.8.2) */
    data[0] = len - 1;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    /* parameter page 6 (c.f. RBC 5.8.3) */
    data[4] = 0x80 | 0x06;
    data[5] = 13 - 2;
    data[6] = 0;
    put_unaligned (cpu_to_be16 (STORAGE_BLOCK_SIZE), (u16 *) & data[7]);
    data[9] = 0;
    put_unaligned (cpu_to_be32 (tdata->num_blocks[logic_units]), (u32 *) & data[10]);
    data[14] = 0;
    data[15] = 0x03;		/* FORMATD,LOCKD */
    data[16] = 0;
    add_in_data (tdata, data, min (len, (unsigned int) CB[4]));
    return 0;

invalid_cdb_field:
    dbgPRINT(": Invalid CDB field line %d", __LINE__);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_INVALID_CDB_FIELD;
    return -EINVAL;
}

static int
do_read (struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 logic_units = tdata->cbw.bLUN;
    unsigned int lba = be32_to_cpu (get_unaligned ((u32 *) & CB[2]));
    unsigned int llen = be16_to_cpu (get_unaligned ((u16 *) & CB[7]));
    unsigned int len = llen * STORAGE_BLOCK_SIZE;

#ifndef CONFIG_USBD_ISP_BUS
    unsigned int i;
    mm_segment_t fs;
#endif
    //off_t max_len = 0; 

    dbgPRINT ("Enter do_read line %d", __LINE__);
    if (tdata->cbw.dDataTransferLength < len)
    {
        dbgPRINT ("length error line %d", __LINE__);
        goto phase_error;
    }
    if (lba + llen > tdata->num_blocks[logic_units])
    {
        dbgPRINT ("lba error line %d", __LINE__);
        goto lba_out_of_range;
    }
#ifndef CONFIG_USBD_ISP_BUS
    SAVE_FS_DS(fs);
    if (lseek (tdata->real_fd[logic_units], (off_t) lba * STORAGE_BLOCK_SIZE,
                0 /* SEEK_SET */ ) == (off_t) - 1)
    //max_len = tdata->real_fd[logic_units]->f_op->llseek( tdata->real_fd[logic_units],
            //(off_t) lba * STORAGE_BLOCK_SIZE, 0);
    //if(max_len == (off_t)-1)
    {
        dbgPRINT ("lseek error line %d", __LINE__);
        goto medium_error;
    }

    i = read (tdata->real_fd[logic_units], tdata->data_buf, len);
    //i = tdata->real_fd[logic_units]->f_op->read(tdata->real_fd[logic_units],
            //tdata->data_buf, len, &(tdata->real_fd[logic_units]->f_pos));
    if (i != len)
    {
        dbgPRINT ("read error.(len = %d, ret = %d) line %d", len, i, __LINE__);
        goto medium_error;
    }
#else
	mmc_block_read_request(1, lba, llen, &g_read_dma_buf);
#endif
    tdata->data_len = len;
    tdata->stat.read_blocks += llen;
#ifndef CONFIG_USBD_ISP_BUS
    RESTORE_FS_DS(fs);
#endif
    return 0;

phase_error:
    dbgPRINT(": phase error on READ line %d", __LINE__);
    tdata->csw.bStatus = CSW_STAT_PERR;
    return -EINVAL;
lba_out_of_range:
    dbgPRINT(": LBA out of range line %d", __LINE__);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LBA_OUT_OF_RANGE;
    return -EINVAL;
#ifndef CONFIG_USBD_ISP_BUS
medium_error:
    dbgPRINT(": Medium error line %d", __LINE__);
    tdata->sense.key = MEDIUM_ERROR;
    RESTORE_FS_DS(fs);
#endif
    return -EINVAL;
}

static int
do_readcapacity (struct usb_storage_threaddata *tdata)
{
    u8 logic_units = tdata->cbw.bLUN;
    u32 data[2];
    dbgPRINT(": num_block %ld line %d", tdata->num_blocks[logic_units], __LINE__);
    data[0] = cpu_to_be32 (tdata->num_blocks[logic_units] - 1);
    data[1] = cpu_to_be32 (STORAGE_BLOCK_SIZE);
    add_in_data (tdata, &data, sizeof (data));
#ifdef CONFIG_USBD_ISP_BUS
    mmc_set_max_size(tdata->num_blocks[logic_units] - 1);
#endif
    return 0;
}

static int
do_request_sense (struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 data[13];
    unsigned int len = sizeof (data);
    data[0] = 0x70;		/* current errors */
    data[1] = 0;
    data[2] = tdata->sense.key;
    put_unaligned (cpu_to_be32 (tdata->sense.info), (u32 *) & data[3]);
    data[7] = len - 7;;
    put_unaligned (cpu_to_be32 (tdata->sense.cmdinfo), (u32 *) & data[8]);
    data[12] = tdata->sense.code;
    add_in_data (tdata, data, min (len, (unsigned int) CB[4]));
    return 0;
}

static int
do_write (struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    u8 logic_units = tdata->cbw.bLUN;
    unsigned int lba = be32_to_cpu (get_unaligned ((u32 *) & CB[2]));
    unsigned int llen = be16_to_cpu (get_unaligned ((u16 *) & CB[7]));
    unsigned int len = llen * STORAGE_BLOCK_SIZE;
#ifndef CONFIG_USBD_ISP_BUS
    mm_segment_t fs;
#endif
    if (tdata->data_len < len)
        goto phase_error;
    if (lba + llen > tdata->num_blocks[logic_units])
        goto lba_out_of_range;
#ifndef CONFIG_USBD_ISP_BUS
    SAVE_FS_DS(fs);
    if (lseek (tdata->real_fd[logic_units], (off_t) lba * STORAGE_BLOCK_SIZE,
                0 /* SEEK_SET */ ) == (off_t) - 1)
    //max_len = tdata->real_fd[logic_units]->f_op->llseek( tdata->real_fd[logic_units],
            //(off_t) lba * STORAGE_BLOCK_SIZE, 0);
    //if(max_len == (off_t)-1)
    {
        goto medium_error;
    }
    if (write (tdata->real_fd[logic_units], tdata->data_buf, len) != len)
    //if (tdata->real_fd[logic_units]->f_op->write(tdata->real_fd[logic_units],
                //tdata->data_buf, len, &(tdata->real_fd[logic_units]->f_pos)))
    {
        goto medium_error;
    }
#else
    if ((lba + llen) >= tdata->num_blocks[logic_units])
	goto medium_error;
    mmc_block_write_request(2, lba, llen, g_write_dma_buf);
#endif
    tdata->csw.dDataResidue -= len;
    tdata->stat.write_blocks += llen;
#ifndef CONFIG_USBD_ISP_BUS
    storage_sync_card(tdata->real_fd[logic_units]);  //sync data and buffer to disk
    RESTORE_FS_DS(fs);
#endif
    return 0;

phase_error:
    dbgPRINT(": phase error on WRITE line %d", __LINE__);
    tdata->csw.bStatus = CSW_STAT_PERR;
    return -EINVAL;
lba_out_of_range:
    dbgPRINT(": LBA out of range line %d", __LINE__);
    tdata->sense.key = ILLEGAL_REQUEST;
    tdata->sense.code = SENCODE_LBA_OUT_OF_RANGE;
    return -EINVAL;
medium_error:
    dbgPRINT(": Medium error line %d", __LINE__);
    tdata->sense.key = MEDIUM_ERROR;
#ifndef CONFIG_USBD_ISP_BUS
    RESTORE_FS_DS(fs);
#endif
    return -EINVAL;
}


static int
do_switch_cmd(struct usb_storage_threaddata *tdata)
{
    u8 *CB = tdata->cbw.CB;
    scsi_switch_from_udisk = CB[10];    //target mode
    return 0;
}
/*
 * RBC functions end
 */

/*
 * storage_modinit - module init
 */
int storage_modinit (void)
{
    struct proc_dir_entry *pw;
#ifdef CONFIG_USBD_ISP_BUS
    init_dma_buf_pool(dma_write_buf_pool,&dma_write_buf_pool_head, MAX_BUF_POOL_SIZE);
    init_dma_buf_pool(dma_read_buf_pool,&dma_read_buf_pool_head, MAX_BUF_POOL_SIZE);
#endif
    dbgPRINT(": Entering storage init. line %d", __LINE__);
    spin_lock_init(&storage_lock);
    /* register /proc/storage_fd*/
    create_proc_read_entry (STORAGE_PROC_NAME, 0, NULL, storage_read_proc, 0);
    /* register /proc/usbAction for mmc card*/
    if ((pw = create_proc_entry (STORAGE_MMC_NAME, 0666, 0)) == NULL)
    {
        return -ENOMEM;
    }
    pw->proc_fops = &storage_umount_functions;

    /* register storage function driver*/
    if (usbd_register_function (&storage_function_driver))
    {
        return -EINVAL;
    }
    dbgPRINT("Leaving storage init. line %d", __LINE__);

    if(storage_thread_kickoff())    //Start thread
    {
        dbgPRINT (": thread error line %d", __LINE__);
        return -EINVAL;
    }
#ifdef CONFIG_ITUNES_SIDEBAND
    return iTunes_sideband_modinit();
#endif
    return 0;
}

/*
 * storage_modexit - module cleanup
 */
void
storage_modexit (void)
{
#ifdef CONFIG_ITUNES_SIDEBAND
    iTunes_sideband_modexit();
#endif
    // de-register us with the usb device support layer
    //
    usbd_deregister_function (&storage_function_driver);
    remove_proc_entry (STORAGE_PROC_NAME, NULL);
    remove_proc_entry (STORAGE_MMC_NAME, NULL);
    storage_thread_killoff();   //stop thread.
#ifdef CONFIG_USBD_ISP_BUS
    release_dma_buf_pool(dma_write_buf_pool,&dma_write_buf_pool_head, MAX_BUF_POOL_SIZE);
    release_dma_buf_pool(dma_read_buf_pool,&dma_read_buf_pool_head, MAX_BUF_POOL_SIZE);
#endif
}

module_init(storage_modinit);
module_exit(storage_modexit);

//EXPORT_SYMBOL (storage_modinit);
//EXPORT_SYMBOL (storage_modexit);

