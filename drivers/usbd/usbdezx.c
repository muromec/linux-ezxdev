/*
 *  Copyright (C) 2003 Motorola
 *
 *  usbd/usbdezx.c - USB Device And EMUD Device Support
 *
 *  Usb setup support for Motorola Ezx Development Platform.
 *  
 *  Author:	Zhuang Xiaofan
 *  Created:	Nov 25, 2003
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
#include <linux/kernel.h>
#include <linux/version.h>

#ifdef CONFIG_ARCH_EZX

#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <asm/atomic.h>
#include <asm/system.h>
#include <linux/tqueue.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/ezxusbd.h>
#include <linux/interrupt.h>
#include <asm/hardware.h>

#ifdef CONFIG_MOT_POWER_IC
#include <linux/power_ic.h>
#endif

#define USB_FUNC_ACM	1
#define USB_FUNC_PST	2
#define USB_FUNC_NET	3

static int usb_flag = USB_FUNC_ACM;
int usbcable_status = 0;		// status of USB cable, depends on USB enumeration
int emucable_status = 0;		// status of EMU cable, such as USB/charger/carkit, depends on PCAP 4V INT

// re-enumeration mechanism when failing to requesting 500mA
// It is done with schedule_task to keventd thread
//  since the interface to control udc connection (pullup resistor) can't be called in interrupt context.
static struct tq_struct udccontrol_tq;
int udccontrol_flag = 0;
static void udcctrl_tq_handler(void *data);
int udccontrol_context_safe(int control_flag);

extern void pcap_switch_on_usb(void);
extern void pcap_switch_off_usb(void);
extern int motusbd_nfs_flag;
extern int motusbd_load(int status);

#ifdef CONFIG_ARCH_EZX // added by Jordan 
extern void udc_clock_enable(void);
extern void udc_clock_disable(void);
#endif

// usbd_in_nfs_mode:
// return whether usbd is in NFS mode
//	1:	Yes
//	0:	No
int usbd_in_nfs_mode(void)
{
	if(motusbd_nfs_flag)
	{
		printk (KERN_INFO "%s: NFS mode workaround, disable EMU VBUS interrupt\n", __FUNCTION__);
		// NFS fix for EMU VBUS 2.0V interrupt
		power_ic_event_mask(POWER_IC_EVENT_EOC_VBUSDET);	//EMU_INT_VBUS);
		//power_ic_event_mask(POWER_IC_EVENT_EOC_ID);	//EMU_INT_ID);	// => no need
	}

	return motusbd_nfs_flag;
}

// usbd_set_gpio_function
// This function sets the GPIOs for USB client
// param enable: 0 to disable USB function and set the GPIOs to power-saving state, anything else set USB function
void usbd_set_gpio_function(int enable)
{
	static int usbd_gpio_function_status = 1;	// default status is enable since phone will disable is first

	printk (KERN_INFO "%s: enable %d, current status %d\n", __FUNCTION__, enable, usbd_gpio_function_status);
	if(enable)
	{
		if(usbd_gpio_function_status == 0)
		{
			set_GPIO_mode(GPIO34_TXENB_MD);
			set_GPIO_mode(GPIO35_XRXD_MD);
			set_GPIO_mode(GPIO36_VMOUT_MD);
			set_GPIO_mode(GPIO39_VPOUT_MD);
			set_GPIO_mode(GPIO40_VPIN_MD);
			set_GPIO_mode(GPIO53_VMIN_MD);
			usbd_gpio_function_status = 1;
		}
	}
	else
	{
		if(usbd_gpio_function_status)
		{
			// Need to set PGSR?
			set_GPIO_mode(GPIO34_TXENB | GPIO_OUT);
			set_GPIO(GPIO34_TXENB);
			set_GPIO_mode(GPIO35_XRXD  | GPIO_IN);
			set_GPIO_mode(GPIO36_VMOUT | GPIO_IN);
			set_GPIO_mode(GPIO39_VPOUT | GPIO_IN);
			set_GPIO_mode(GPIO40_VPIN  | GPIO_IN);
			set_GPIO_mode(GPIO53_VMIN  | GPIO_IN);
			usbd_gpio_function_status = 0;
		}
	}
	return ;
}

static int __init usbd_func_switch(char *line)
{
	//printk(KERN_INFO"%s: line %s\n", __FUNCTION__, line);
	if (strncmp (line, "acm", 3) == 0) usb_flag=USB_FUNC_ACM;
	if (strncmp (line, "pst", 3) == 0) usb_flag=USB_FUNC_PST;
	if (strncmp (line, "net", 3) == 0) usb_flag=USB_FUNC_NET;
	return 1;
}
__setup("usbd=", usbd_func_switch);


void pulldown_usb(void)
{
	if((usb_flag == USB_FUNC_ACM) || (usb_flag == USB_FUNC_PST)) {
		//printk(KERN_INFO"\n!!!%s: jiff %d\n", __FUNCTION__, jiffies);

		// TODO:
		// set GPIOs to power-save mode

#ifdef CONFIG_MOT_POWER_IC
		if(!in_interrupt())
		{
			power_ic_periph_set_usb_pull_up(0);
		}

		// set GPIOs to power-save mode
		usbd_set_gpio_function(0);
#else
		pcap_switch_off_usb();
#endif
	}
}

void pullup_usb(void)
{
	if((usb_flag == USB_FUNC_ACM) || (usb_flag == USB_FUNC_PST)) {
		//printk(KERN_INFO"\n!!!%s: jiff %d\n", __FUNCTION__, jiffies);
		
		// TODO:
		// re-config low-level path of USBD, including GPIOs
		usbd_set_gpio_function(1);		
		UP2OCR = 0x02000000;		// select single end port

#ifdef CONFIG_MOT_POWER_IC
		if(!in_interrupt())
		{
			power_ic_periph_set_usb_pull_up(1);
		}
#else
		pcap_switch_on_usb();
#endif
	}
}

// Interface function to connect/disconnect UDC
// It is context safe API call, which means it can be called in interrrupt context
int udccontrol_context_safe(int control_flag)
{
	//printk(KERN_INFO"\n!!!%s: jiff %d\n", __FUNCTION__, (int)jiffies);

	// it is based keventd's task queue
	*((int *)(udccontrol_tq.data)) = control_flag;
	schedule_task(&udccontrol_tq);
	return 0;
}

extern void udc_connect (void);
extern void udc_disconnect (void);

static void udcctrl_tq_handler(void *data)
{
	int status = *((int *)data);
	//printk(KERN_INFO"\n!!!%s: status %d jiff %d\n", __FUNCTION__, status, (int)jiffies);
	if(status)
	{
		udc_connect();
	}
	else
	{
		udc_disconnect();
	}
	return ;
}


static int __init motusbd_func_modinit(void)
{
#ifdef CONFIG_ARCH_EZXBASE
	printk (KERN_INFO "%s: Barbados usbd GPIO init\n", __FUNCTION__);
	
	// set default USBD GPIO status to disable status
	usbd_set_gpio_function(0);

	// In init time, to GPIO_USB_READY (GPIO 99), set the function and clear it
	set_GPIO_mode(GPIO_USB_READY | GPIO_OUT);
	clr_GPIO(GPIO_USB_READY);

	// UP2OCR must be inited when system is initliazed due to USBH sleep/wakeup design.
	// USBD will set it when enabling usbd stack
	UP2OCR = 0x02000000;		// select single end port
	
	udccontrol_tq.routine = udcctrl_tq_handler;
	udccontrol_tq.data = &udccontrol_flag;
	
	pulldown_usb();
#endif
	
	//printk (KERN_INFO "%s: usb_flag = %d\n", __FUNCTION__, usb_flag);
	switch (usb_flag) {
	case USB_FUNC_NET:
		motusbd_nfs_flag = 1;
		usbd_set_gpio_function(1);
		motusbd_load(USB_STATUS_NET);
		break;
	case USB_FUNC_PST:
		usbd_set_gpio_function(1);
		motusbd_load(USB_STATUS_PST);
		break;
	case USB_FUNC_ACM:
		// fall through
	default:
#ifdef CONFIG_MOT_POWER_IC	//CONFIG_ARCH_EZXBASE
		// init usb to NULL to let app to do the switching logic
		motusbd_load(USB_STATUS_NONE);
#else
		motusbd_load(USB_STATUS_MODEM);
#endif
		break;
	}
	return 0;
}

static void __exit motusbd_func_modexit (void)
{
	// unload usbd func?
	return ;
}

module_init (motusbd_func_modinit);
module_exit (motusbd_func_modexit);

//EXPORT_SYMBOL(pullup_usb);
//EXPORT_SYMBOL(pulldown_usb);


#if	1 //def CONFIG_ARCH_EZXBASE
static int motusbd_dev_open_flag;
static DECLARE_WAIT_QUEUE_HEAD(motusbd_dev_wait);
static spinlock_t motusbd_dev_lock = SPIN_LOCK_UNLOCKED;

#define MOTUSBD_DEV_BUFF_SIZE	256
static unsigned int motusbd_dev_buf[MOTUSBD_DEV_BUFF_SIZE];		// 256 * 4 = 1K
static int motusbd_dev_buf_read = 0;
static int motusbd_dev_buf_write = 0;

#define MOTUSBD_DEV_BUF_EMPTY() (motusbd_dev_buf_read == motusbd_dev_buf_write)
#define MOTUSBD_DEV_FULL()  ( ((motusbd_dev_buf_write+1)%MOTUSBD_DEV_BUFF_SIZE) == motusbd_dev_buf_read)
#define MOTUSBD_DEV_INC(x)  ((x) = (((x)+1) % MOTUSBD_DEV_BUFF_SIZE))

static int motusbd_dev_open(struct inode *inode, struct file *file) 
{
	spin_lock(motusbd_dev_lock);
	if(motusbd_dev_open_flag)
		return -EBUSY;
	
	motusbd_dev_open_flag = 1;
	spin_unlock(motusbd_dev_lock);
	//printk (KERN_INFO "%s: return\n", __FUNCTION__);
	return 0;
}

static int motusbd_dev_free(struct inode *inode, struct file *file)
{
	motusbd_dev_open_flag = 0;
	//printk (KERN_INFO "%s: return\n", __FUNCTION__);
	return 0;
}

static ssize_t motusbd_dev_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	ssize_t retval = 0;
	unsigned long data;
	
	if (count < sizeof(unsigned long))
		return -EINVAL;
	
	if (MOTUSBD_DEV_BUF_EMPTY())
	{ 
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		retval = wait_event_interruptible(motusbd_dev_wait, !MOTUSBD_DEV_BUF_EMPTY());
		if (retval)
			return retval;
	}

	spin_lock(motusbd_dev_lock);
	data = motusbd_dev_buf[motusbd_dev_buf_read];
	MOTUSBD_DEV_INC (motusbd_dev_buf_read);
	//printk (KERN_INFO "%s: data is 0x%x\n", __FUNCTION__, data);
	spin_unlock(motusbd_dev_lock);
	
	retval = put_user (data, (unsigned long *)buf);
	
	return retval;
}

static unsigned int motusbd_dev_poll(struct file *file, poll_table *wait)
{
	//printk (KERN_INFO "%s: 1 read %d, write %d\n", __FUNCTION__, motusbd_dev_buf_read, motusbd_dev_buf_write);
	poll_wait (file, &motusbd_dev_wait, wait);
	//printk (KERN_INFO "%s: 2 read %d, write %d\n", __FUNCTION__, motusbd_dev_buf_read, motusbd_dev_buf_write);
	if(!MOTUSBD_DEV_BUF_EMPTY())
		return POLLIN | POLLRDNORM;
	return 0;
}

static int motusbd_dev_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
// 
// e12707 051025 Add USB Mode Switch Fuction
// 
	int argi;

	switch (cmd)
	{
		case  MOTUSBD_IOCTL_SET_USBMODE: //EMUDSOCK_CMDTYPE_REQ_USBMODE

			get_user(argi,(int*)arg);

			if((argi>=USB_STATUS_NONE)&&(argi<=USB_STATUS_NM))//the Last USB status
			{
				motusbd_load(argi);
				return 0;
			}
			else
			{
				printk("\n None of the usb mode:%d\n ",argi);
				return  -EFAULT;
			}
		break;

		case  MOTUSBD_IOCTL_GET_USBMODE:
			{
				extern int motusbd_status;

				put_user(motusbd_status,(int*)arg);

				return 0;
			}
		break;

		default:
			printk("\n None of the usb command:%d\n ",cmd);
			return  -EFAULT;
	}
	
}

int queue_motusbd_event(MOTUSBD_MSG_T event)
{
	//printk (KERN_INFO "%s: event 0x%x\n", __FUNCTION__, event);
	if (MOTUSBD_DEV_FULL())
		return -ENOMEM;
	
	spin_lock(motusbd_dev_lock);
	motusbd_dev_buf[motusbd_dev_buf_write] = event;
	MOTUSBD_DEV_INC (motusbd_dev_buf_write);
	spin_unlock(motusbd_dev_lock);
	
	wake_up_interruptible(&motusbd_dev_wait);
	return 0;
}

static struct file_operations motusbd_dev_fops =
{
	.owner =    THIS_MODULE,
	.ioctl =    motusbd_dev_ioctl,
	.open =     motusbd_dev_open,
	.release =  motusbd_dev_free,
	.read =     motusbd_dev_read,
	.poll =     motusbd_dev_poll,
};

int __init motusbd_dev_modinit(void)
{
	int ret;
	printk (KERN_INFO "%s: \n", __FUNCTION__);
	// register_misc???
	ret = register_chrdev(MOTUSBD_DEV_MAJOR_NUM, MOTUSBD_DEV_NAME, &motusbd_dev_fops);
	if (ret < 0)
	{
		printk (KERN_INFO "%s: register_chrdev error, ret %d\n", __FUNCTION__, ret);
		return ret;
	}
	motusbd_dev_open_flag = 0;
	motusbd_dev_buf_read = 0;
	motusbd_dev_buf_write = 0;
	return 0;
}

void __exit motusbd_dev_modexit(void)
{
	unregister_chrdev(MOTUSBD_DEV_MAJOR_NUM, MOTUSBD_DEV_NAME);
}

module_init (motusbd_dev_modinit);
module_exit (motusbd_dev_modexit);
#endif	// CONFIG_ARCH_EZXBASE

#endif	// CONFIG_ARCH_EZX
