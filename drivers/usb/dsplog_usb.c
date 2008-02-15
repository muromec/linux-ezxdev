/*
 *  /drivers/usb/dsplog_usb.c
 *   
 *  This file just implements a USB DSP Log driver
 * 
 *  Copyright (C) 2005 - Motorola
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/usb.h>
#include <linux/poll.h>

#include <linux/major.h>
#include <linux/dsplog_usb.h>

#ifdef CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
#endif

/*Macro defined for this driver*/
#define DRIVER_VERSION      ""
#define DRIVER_AUTHOR       ""
#define DRIVER_DESC         "USB DSP LOG Driver"

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");


/* USB device product and vendor ID */
#define MOTO_DSPLOG_VID		0x22b8
#define MOTO_DSPLOG_PID		0x3006

/* the macro for detect the endpoint type */
#define IS_EP_BULK(ep)     ((ep).bmAttributes == USB_ENDPOINT_XFER_BULK ? 1 : 0)
#define IS_EP_BULK_IN(ep)  (IS_EP_BULK(ep) && ((ep).bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
#define IS_EP_BULK_OUT(ep) (IS_EP_BULK(ep) && ((ep).bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT)

/* extern values */
extern int usb_host_resumed;
extern void wmmx_ohci_resume_call(void);

int dsplog_active = 0;  //this flag is used by ipcusb.c
/* global values defined */
static struct fasync_struct     *dl_fasync;
static DECLARE_MUTEX_LOCKED(dl_callback_wait); 

/* DSPLOG USB interface number */
#define DSPLOG_IF_INDEX    2

/* Debug macro define */
//#define DL_DEBUG		

#ifdef  DL_DEBUG
#define dl_dbg(format, arg...) printk(__FILE__ ": " format "\n" , ## arg)
#else
#define dl_dbg(format, arg...) do {} while (0)
#endif

#ifdef CONFIG_DEVFS_FS
/* The External Parameter for Register The USB Device */
extern devfs_handle_t usb_devfs_handle;
#else
#define DSPLOG_MAJOR    MISC_MAJOR
#endif

#define MAX_READ_SIZE           2048
#define WAKE_UP_BP_UDELAY	125
 
unsigned char gBuffer[MAX_READ_SIZE];

/* Internal File-System entry fucntion declaration */
static int dsplog_fasync(int fd, struct file *filp, int mode);
static int dsplog_open(struct inode * inode, struct file * file);
static int dsplog_release(struct inode * inode, struct file * filp);
static ssize_t dsplog_read(struct file * filp, char * buf, size_t count, loff_t * l);
static unsigned int dsplog_poll(struct file * filp, struct poll_table_struct * wait);
static int dsplog_ioctl(struct inode * inode, struct file * filpi, unsigned int cmd, unsigned long arg);

/* Internal USB entry function declaration */
static void *usb_dl_probe(struct usb_device *usbdev, unsigned int ifnum, const struct usb_device_id *id);
static void usb_dl_disconnect(struct usb_device *usbdev, void *ptr);

/* file operation struction */
static struct file_operations dsplog_fops = {
    owner:          THIS_MODULE,
    open:           dsplog_open,
    ioctl:          dsplog_ioctl,
    release:        dsplog_release,
    read:           dsplog_read,
    poll:           dsplog_poll,
    fasync:         dsplog_fasync,
};

/*file device number*/
static struct miscdevice usbdl_misc_device = {
    DSPLOG_MINOR, "usbdl", &dsplog_fops
};

/* DSPLOG data structure */
struct dl_usb_data
{
	struct usb_device 	 *dl_dev;   /* USB device handle */	
	struct urb 	         dl_urb;	/* urbs */
	char                     *ibuf;
	char                     dl_ep;	    /* Endpoint assignments */
#ifdef CONFIG_DEVFS_FS
	devfs_handle_t           devfs;	    /* devfs device */
#else
	int                      devfs;
#endif
	int                      isopen;
	int                      probe_flag;
	int                      dataNum;
	int                      bufSize;
} dl_usb;

/* USB host stack entry fucntion for this driver */
static struct usb_driver usb_dl_driver = {
	name:		DL_DRV_NAME,
	probe:		usb_dl_probe,
	disconnect:	usb_dl_disconnect,
	fops:		&dsplog_fops,
	minor:          DSPLOG_MINOR,
	id_table:	NULL,
};

/******************************************************************************
 * Function Name: comx_get_dsr
 *
 * Input:           pInode    :  file node
 * Value Returned:  int       : -1 = false, other = true
 *
 * Description: This function is used to check whether the file node is correct.
 *
 * Modification History:
 *  
 *****************************************************************************/
static int checkDevice(struct inode *pInode)
{
    int minor;
    kdev_t dev = pInode->i_rdev;

#ifdef CONFIG_DEVFS_FS    
    if( MAJOR(dev) != USB_MAJOR)     {
        dl_dbg("checkDevice bad major = %d\n",MAJOR(dev) );
        return -1;
    }
    minor = MINOR(dev);
    if ( minor !=  DSPLOG_MINOR ) {
        printk("checkDevice bad minor = %d\n",minor );
        return -1;
    }
    return minor;
#else
    if( MAJOR(dev) != DSPLOG_MAJOR)     {
        dl_dbg("checkDevice bad major = %d\n",MAJOR(dev) );
        return -1;
    }

    minor = MINOR(dev);
    if ( minor !=  DSPLOG_MINOR ) {
        printk("checkDevice bad minor = %d\n",minor );
        return -1;
    }
    return minor;
#endif
}

/******************************************************************************
 * Function Name: dsplog_ioctl
 *
 * Input:           pInode    :  file node
 *                  file      :  file pointer
 * Value Returned:  int       : -EBUSY = this node has been opened, 
 *                            : 0 = true
 *
 * Description: This function is used to open this driver.
 *
 * Modification History:
 *  
 *****************************************************************************/
static int dsplog_ioctl(struct inode * inode, struct file * filpi, unsigned int cmd, unsigned long arg)
{
    switch(cmd) {
        case DSPLOG_ENABLED:
            if( !dsplog_active ) {
                printk("\r\n dsplog: enabled!");
                dsplog_active = 1;
            }
	    break;
	case DSPLOG_DISABLED:
            if( dsplog_active )  {
                printk("\r\n dsplog: disabled!");
                dsplog_active = 0;
            }
	    break;
    }
    return 0;
}

/******************************************************************************
 * Function Name: dsplog_open
 *
 * Input:           pInode    :  file node
 *                  file      :  file pointer
 * Value Returned:  int       : -EBUSY = this node has been opened, 
 *                            : 0 = true
 *
 * Description: This function is used to open this driver.
 *
 * Modification History:
 *  
 *****************************************************************************/
static int
dsplog_open(struct inode * inode, struct file * file)
{
    dl_dbg("open_dsplog");
    if( dl_usb.isopen ) {
	dl_dbg("dsplog open error");
	return -EBUSY;
    }
    
    MOD_INC_USE_COUNT;
    dl_usb.isopen = 1;

    dsplog_active = 1;

    return 0;
}

/******************************************************************************
 * Function Name: dsplog_release
 *
 * Input:           pInode    :  file node
 *                  filp      :  file pointer
 * Value Returned:  int       :  always 0 
 *
 * Description: This function is used to close the opened driver.
 *
 * Modification History:
 *  
 *****************************************************************************/
static int
dsplog_release(struct inode * inode, struct file * filp)
{
    dl_dbg("dsp_release\n");
    if( !dl_usb.isopen ) {
    	// not opened, directly return.
    	return 0;
    }
	
    dsplog_fasync(-1, filp, 0);
    dl_usb.isopen = 0;
    dsplog_active = 0;
    MOD_DEC_USE_COUNT;

    return 0;
}

/******************************************************************************
 * Function Name: dsplog_read
 *
 * Input:           filp      :  file pointer
 *                  buf       :  the buffer for save the read data
 *                  count     :  the read buffer size
 *                  l         :  
 * Value Returned:  int       :  the real data number in the read buffer
 *
 * Description: This function is used to read data.
 *
 * Modification History:
 *  
 *****************************************************************************/
static ssize_t dsplog_read(struct file * filp, char * buf, size_t count, loff_t * l)
{       
    int minor,result = count;
    int retval;
    int time = 0;
 
    minor = checkDevice( filp->f_dentry->d_inode );
    if ( minor == - 1)  {
        return -ENODEV;
    }
    /* not enabled */
    if( !dsplog_active )
	return 0;

    /* detect whether resume is need */
    if( !usb_host_resumed) {
        wmmx_ohci_resume_call();
        usb_host_resumed = 1;
    }

    /* detect whether BP is ready */
    set_GPIO_mode(GPIO_IN | GPIO_BP_RDY);
    if(!GPIO_is_high(GPIO_BP_RDY))
    {
	if(GPIO_is_high(GPIO_AP_RDY ))
	{
		GPCR(GPIO_AP_RDY ) = GPIO_bit(GPIO_AP_RDY );
		udelay(WAKE_UP_BP_UDELAY);
		GPSR(GPIO_AP_RDY ) = GPIO_bit(GPIO_AP_RDY );
	} else {
		GPSR(GPIO_AP_RDY ) = GPIO_bit(GPIO_AP_RDY );
		udelay(WAKE_UP_BP_UDELAY);
		GPCR(GPIO_AP_RDY ) = GPIO_bit(GPIO_AP_RDY );
	}
	time = jiffies;
	while(!GPIO_is_high(GPIO_BP_RDY) && (jiffies < (time+HZ))) ;
	if(!GPIO_is_high(GPIO_BP_RDY))
	{
		dl_dbg("%s: Wakeup BP timeout! BP is still in sleep state!\n", __FUNCTION__);		
	}				
    }

    dl_usb.dl_urb.dev = dl_usb.dl_dev;
    dl_usb.dl_urb.actual_length = 0;
    dl_usb.dataNum = 0;
    if( (retval = usb_submit_urb(&dl_usb.dl_urb) ) ) {
       	// submit URB error, return 0.
       	printk("submit urb is error, retval = %d",retval);
       	return -1;
    }
    down(&dl_callback_wait);

    if(dl_usb.dataNum < count)    {
        result = dl_usb.dataNum;
    }
    dl_usb.dataNum = 0; 
    copy_to_user(buf, (char*)&gBuffer[0], result );
    
    return result;
}

/******************************************************************************
 * Function Name: dsplog_fasync
 *
 * Input:           fd        :  
 *                  filp      :  
 *                  mode      :  
 * Value Returned:  int       :  the status value
 *
 * Description: This function is used to for fasync entry.
 *
 * Modification History:
 *  
 *****************************************************************************/
static int dsplog_fasync(int fd, struct file *filp, int mode)
{
    int minor = checkDevice( filp->f_dentry->d_inode);

    if ( minor == - 1)     {
        dl_dbg("smk_ts_fasyn:bad device minor");
        return -ENODEV;
    }
    return( fasync_helper(fd, filp, mode, &dl_fasync) );
}

/******************************************************************************
 * Function Name: dsplog_fasync
 *
 * Input:           filp      :  
 *                  wait      :  
 * Value Returned:  int       :  the status value
 *
 * Description: This function is used to for poll entry.
 *
 * Modification History:
 *  
 *****************************************************************************/
static unsigned int dsplog_poll(struct file * filp, struct poll_table_struct * wait)
{
    int minor;

    minor = checkDevice( filp->f_dentry->d_inode);
    if ( minor == - 1)  {
	dl_dbg("check device major/minor number error");
        return -ENODEV;
    }
    down(&dl_callback_wait);   
    if( dl_usb.dataNum <= 0 )  {
        return 0;
    }
    return (POLLIN | POLLRDNORM);
}

/******************************************************************************
 * Function Name:  usb_dl_callback
 *
 * Input:           urb       :  the current urb pointer
 * Value Returned:  void
 *
 * Description: This function is used as the call back function for DSPLOG bulk IN.
 *
 * Modification History:
 *  
 *****************************************************************************/
static void usb_dl_callback(struct urb *urb)
{
 	dl_dbg("usb_dl_callback, %d",urb->actual_length);
	dl_usb.dataNum = urb->actual_length;
        up(&dl_callback_wait);
}

/******************************************************************************
 * Function Name:  usb_dl_probe
 *
 * Input:           usbdev      :  the pointer for USB device structure
 *                  ifnum       :  interface number
 *                  id          :
 * Value Returned:  void *      :  NULL = error.
 *
 * Description: This function is used as the probe function of DSPLOG interface.
 *
 * Modification History:
 *  
 *****************************************************************************/

static void *usb_dl_probe(struct usb_device *usbdev, unsigned int ifnum,
			   const struct usb_device_id *id)
{	
	struct usb_interface_descriptor *dl_interface;
	struct usb_endpoint_descriptor *dl_endpoint;		

	dl_dbg("Beginning usb_dl_probe\n");
	
	if( dl_usb.probe_flag ) {
		dl_dbg("dsplog has been probed");
		return NULL;
	}

	dl_dbg("usb_dl_probe: vendor id 0x%x, device id 0x%x", usbdev->descriptor.idVendor, usbdev->descriptor.idProduct);
	
	if ((usbdev->descriptor.idVendor != MOTO_DSPLOG_VID) || (usbdev->descriptor.idProduct != MOTO_DSPLOG_PID) ||
	    (ifnum != DSPLOG_IF_INDEX) ) {
		dl_dbg("ifnum error ifnum = %d,idvendor=%x, idProduct=%x ",ifnum,usbdev->descriptor.idVendor,usbdev->descriptor.idProduct);
 		return NULL;
	}

        // Continue detect some other description.
	if (usbdev->descriptor.bNumConfigurations != 1)  {
		dl_dbg("usb_dl_probe: configuration number detected.");
		return NULL;
	}

	if (usbdev->config[0].bNumInterfaces != 3)  	{
		dl_dbg("usb_dl_probe: interface number detected error");
		return NULL;
	}
	
	dl_interface = usbdev->config[0].interface[ifnum].altsetting;
	dl_endpoint =  dl_interface[0].endpoint;

	dl_dbg("usb_dl_probe: Number of Endpoints:%d", (int) dl_interface->bNumEndpoints);	

	if (dl_interface->bNumEndpoints != 1)  {
		dl_dbg("usb_dl_probe: Only one endpoints is supported.");
		return NULL;
	}

	dl_dbg("usb_dl_probe: endpoint[0] is:%x", (&dl_endpoint[0])->bEndpointAddress);

	if (!IS_EP_BULK_IN(dl_endpoint[0])) 	{			
		info("usb_dl_probe: Undetected IN endpoint");
		return NULL;	/* Shouldn't ever get here unless we have something weird */
	}

	dl_usb.ibuf       = gBuffer;
	dl_usb.dl_ep      = (&dl_endpoint[0])->bEndpointAddress;
	dl_usb.probe_flag = 1;	
	dl_usb.dl_dev     = usbdev;	
	dl_usb.bufSize    = MAX_READ_SIZE;
	dl_usb.dataNum    = 0;
	dl_usb.isopen     = 0;

	/*Build a read urb and send a IN token first time*/
	FILL_BULK_URB(&(dl_usb.dl_urb), usbdev, usb_rcvbulkpipe(usbdev, dl_usb.dl_ep),
		dl_usb.ibuf, MAX_READ_SIZE, usb_dl_callback, 0);

	// register device
#ifdef CONFIG_DEVFS_FS
	dl_usb.devfs = devfs_register(usb_devfs_handle, DL_DRV_NAME,
				    DEVFS_FL_DEFAULT, USB_MAJOR, DSPLOG_MINOR,
				    S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP |
				    S_IWGRP | S_IROTH | S_IWOTH, &dsplog_fops, NULL);
				    
	if (dl_usb.devfs == NULL)  {
		dl_dbg("dsplog : device node registration failed");
		return NULL;
	}
#else
	misc_register (&usbdl_misc_device);
#endif
	
	return &dl_usb;
}


/******************************************************************************
 * Function Name:  usb_dl_disconnect
 *
 * Input:           usbdev      :  the pointer for USB device structure
 *                  ptr         :  DSPLOG usb structure
 *                  id          :
 * Value Returned:  void *      :  NULL = error.
 *
 * Description: This function is used as the disconnect for DSPLOG interface.
 *
 * Modification History:
 *  
 *****************************************************************************/
static void usb_dl_disconnect(struct usb_device *usbdev, void *ptr)
{
	int retval;
	struct dl_usb_data *dl = (struct dl_usb_data *) ptr;

	if( !dl_usb.probe_flag ) {
		dl_dbg("Call disconnect, but dsplog is not probed");
		return;
	}	

	retval = usb_unlink_urb (&dl->dl_urb);
	if (retval != -EINPROGRESS && retval != 0) 	{
		dl_dbg("disconnect: unlink urb error, %d", retval);
	}

        usb_driver_release_interface(&usb_dl_driver,
        &dl->dl_dev->actconfig->interface[DSPLOG_IF_INDEX]);
		
	dl_dbg("usb_dl_disconnect");
	dl_usb.probe_flag = 0;
	dl_usb.dataNum = 0;

        // wake up the blocked read
	up(&dl_callback_wait); 
        if(dl_fasync)  {
                 kill_fasync(&dl_fasync, SIGIO, POLL_IN);
        }

#ifdef CONFIG_DEVFS_FS
	devfs_unregister(dl_usb.devfs);
#else
	misc_deregister (&usbdl_misc_device);
#endif
}

/******************************************************************************
 * Function Name:  usb_ldl_init
 *
 * Input:           void       :
 * Value Returned:  int        :  -1 = False;   0 = True
 *
 * Description: This function is used as the drive initialize function.
 *
 * Modification History:
 *  
 *****************************************************************************/
static int __init usb_ldl_init(void)
{
	int result;

	dl_dbg ("init usb_dl_init");

        dl_fasync = NULL;
	memset((void *)&dl_usb, 0, sizeof(struct dl_usb_data));
	result = usb_register(&usb_dl_driver);
	if (result < 0) {
		dl_dbg("usb dl driver could not be registered");
		return -1;
	}

	return 0;
}

/******************************************************************************
 * Function Name:  usb_ldl_exit
 *
 * Input:           void       :
 * Value Returned:  void        :  -1 = False;   0 = True
 *
 * Description: This function is used as the drive exit function.
 *
 * Modification History:
 *  
 *****************************************************************************/
static void __exit usb_ldl_exit(void)
{
	dl_dbg ("usb_dl_exit");
	usb_deregister(&usb_dl_driver);
}

/* the module entry declaration of this driver */
module_init(usb_ldl_init);
module_exit(usb_ldl_exit);

