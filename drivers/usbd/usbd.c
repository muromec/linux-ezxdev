/*
 * usbd/usbd.c - USB Device Core Layer
 *
 *      Copyright (c) 2004 Belcarra
 *
 * Adapted from earlier work:
 *      Copyright (c) 2002, 2003 Belcarra
 *      Copyright (c) 2000, 2001, 2002 Lineo
 *      Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@belcarra.com>, 
 *      Tom Rushworth <tbr@belcarra.com>, 
 *      Bruce Balden <balden@belcarra.com>
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
 * Copyright (C) 2004 - Motorola
 *
 * 2004-Dec-06 - Modified for EZXBASE By Zhao Liang <w20146@motorola.com>
 *
 */

/*
 * Notes...
 *
 * 1. The order of loading must be:
 *
 *        usbdcore
 *        usbdprocfs (optional)
 *        usbd-function(s)
 *        usbd-bus interface
 *
 * 2. Loading usb-function modules allows them to register with the usb-device
 * layer. This makes their usb configuration descriptors available. Currently
 * function modules cannot be loaded after a bus module has been loaded.
 *
 * 3. Loading a usb-bus module causes it to register with the usb-device
 * layer and then enable their upstream port. This in turn will cause the
 * upstream host to enumerate this device. Note that bus modules can create
 * multiple devices, one per physical interface.
 *
 * 4. The usbdcore layer provides a default configuration for endpoint 0 for
 * the device. 
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include "usbd-export.h"
#include "usbd-build.h"

MODULE_AUTHOR ("sl@lineo.com, tbr@lineo.com");
MODULE_DESCRIPTION ("USB Device Core Support");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,17)
MODULE_LICENSE("GPL");
#endif



#include <linux/init.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>

#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>

#include "usbd-chap9.h"
#include "usbd-mem.h"
#include "usbd.h"
#include "usbd-func.h"
#include "usbd-bus.h"
#include "usbd-admin.h"

#ifdef CONFIG_ARCH_EZX
#include <linux/ezxusbd.h>
extern int usbcable_status;
#endif

// The design in EZXBASE is that usb client stack is launched dynamically with the cable operation,
//  and phone never goes to sleep if usb cable is there.
// For usb client, there is no need for usb client to do pm_on / pm_off, thus to save time.
// So, undef CONFIG_PM only in the file to save memory
#undef CONFIG_PM	
#if defined(CONFIG_PM)
#include <linux/pm.h>
#endif

#if defined(CONFIG_PM)
//#define DEBUG_USBC_PM		//USBC PM DEBUG INFO added by Jordan  2004/04/27
#endif


USBD_MODULE_INFO ("usbdcore 2.0-beta");

#define MAX_INTERFACES 2

#ifdef CONFIG_ARCH_EZX
// Smaller setting will save memory.
// But PST has many interfaces (>15), so 20 is not enough
int usbd_maxstrings = 24;
#else
int usbd_maxstrings = 20;
#endif

/* Global variables ************************************************************************** */


extern int usbd_switch_init(void);

/* List support functions ******************************************************************** */

LIST_HEAD (usbd_function_drivers);	// list of all registered function modules

struct usb_bus_instance *usbd_bus_instance;

/* Support Functions ************************************************************************* */


#ifdef MALLOC_TEST
int usbd_mallocs;
#endif

/* Switch Functions ************************************************************************** */
/*
 * USBD Switch will make direct calls to the appropriate modinit/modexit and
 * bus interface bi_modinit/bi_modexit functions to load and unload as appropriate.
 *
 *
 * The following entry points are exported:
 *
 *      int usbd_enable(void)
 *      int usbd_disable(char *name)
 *      int usbd_disconnect(char *name)
 *      int usbd_connect(char *name)
 *      int usbd_pm_on(char *name)
 *      int usbd_pm_off(char *name)
 *
 *      int usbd_unload(void)
 *      int usbd_load(char *name)
 *      int usbd_unplug(char *name)
 *      int usbd_replug(char *name)
 *
 *
 * If CONFIG_PROCFS is defined then the following commands can be given to /proc/usbd-switch:
 *
 *      enable
 *      enable name
 *      disable
 *      disconnect
 *      connect
 *
 *      serial_number
 *      debug
 *      nodebug
 *
 *
 * There are four different styles of operations that can be used:
 *
 *      1. switch - change or configure the device function by sending any
 *      one of a number of commands to the switch interface [1] of
 *      usbd-switch:
 *
 *              - echo "serial_number xxxxx" > /proc/usbd-switch
 *              - echo "enable network_fd" > /proc/usbd-switch
 *              - echo "disconnect" > /proc/usbd-switch
 *              - echo "connect" > /proc/usbd-switch
 *              - echo "disable" > /proc/usbd-switch
 *      
 *      serial_number - set the serial number string
 *      enable function - set current function and enable function and bus interface
 *	enable          - enable current function and bus interface
 *      disable         - disable bus interface and current function
 *      connect         - enable current function and bus interface and connect pullup resistor
 *      disconnect      - unload bus interface, current function and reset current function,
 *	                  and disconnect pullup resistor
 *
 *      2. simple power management
 *
 *              - EXPORT_SYMBOL(usbd_enable);
 *              - EXPORT_SYMBOL(usbd_disable);
 *              - EXPORT_SYMBOL(usbd_disconnect);
 *              - EXPORT_SYMBOL(usbd_connect);
 *              - EXPORT_SYMBOL(usbd_pm_on);
 *              - EXPORT_SYMBOL(usbd_pm_off);
 *
 *         Deprecated:
 *
 *              - EXPORT_SYMBOL(usbd_load);
 *              - EXPORT_SYMBOL(usbd_unload);
 *              - EXPORT_SYMBOL(usbd_unplug);
 *              - EXPORT_SYMBOL(usbd_replug);
 *
 *      usbd_enable() - load function and bus interface
 *      usbd_disable() - unload bus interface and function
 *
 *      usbd_connect() - connect
 *      usbd_disconnect() - disconnect
 *
 *      usbd_pm_on() - connect
 *      usbd_pm_off() - disconnect
 *
 *      usbd_load() - load function and bus interface
 *      usbd_unload() - unload bus interface and function
 *      usbd_unplug() - disconnect
 *      usbd_replug() - connect
 *
 *      3. official power management CONFIG_PM [not tested yet]
 *
 *      Perform unplug and plug functions appropriate for CONFIG_PM actions.
 *
 *
 * N.B.
 *
 *      [1] The switch interface is only available when the USB device stack
 *      and usbd-switch are compiled into the kernel.  It is not available
 *      when the USB device stack consists of loadable modules.
 *
 *      [2] Cable detection requires implementation of architecture specific
 *      code that knows about the appropriate interrupt, GPIO pins etc.
 *
 */


extern struct list_head usbd_function_drivers;


/* Switch Functions ************************************************************************** */

/*
 * This is a table of function drivers that are compiled into the kernel.
 */

#define MAX_FUNC_NAME_LEN  64
char current_function[MAX_FUNC_NAME_LEN+1];
int switch_debug = 0;

char *usbd_load_arg;

int switch_exiting;
struct tq_struct enable_bh;
struct tq_struct disable_bh;



/* ******************************************************************************************* */

static DECLARE_MUTEX(usbd_sem);         // protect against multiple administrative actions overlapping

extern struct usb_bus_instance *usbd_bus_instance;

char *usbd_admin_names[] = {
        "enable", "disable", "connect", "disconnect", "pm_off", "pm_on", "serial_number", 
};

char *usbd_admin_aliases[] = {
        "load", "unload", "replug", "unplug", "pm_off", "pm_on", "serial_number_str", 
};


/* usbd_admin - 
 * 
 */
static int usbd_admin(usbd_admin_t usbd_admin_ndx, char *arg)
{

        //printk(KERN_INFO"%s: %s current: %s arg: %s\n", __FUNCTION__, 
        //                usbd_admin_names[usbd_admin_ndx], 
        //                current_function[0] ? current_function : "(EMPTY)",
        //                (arg && arg[0]) ? arg : "(EMPTY)");

        RETURN_EINVAL_IF(in_interrupt() || down_interruptible(&usbd_sem));

        /*
         * 
         */
	if (usbd_admin_ndx == usbd_admin_enable && (NULL == arg || 0 == *arg)) {
		// s/enable/enable current_function/
		arg = current_function;
	}

        THROW_IF(!usbd_bus_instance, error);

        THROW_IF (!usbd_bus_instance->admin[usbd_admin_ndx] || 
                        usbd_bus_instance->admin[usbd_admin_ndx](usbd_bus_instance, arg), error);

	if (usbd_admin_ndx == usbd_admin_enable && NULL != arg && arg != current_function) {
		// update current_function
		strncpy(current_function,arg,MAX_FUNC_NAME_LEN);
	}

        up(&usbd_sem);
        return 0;

        CATCH(error) {
                printk(KERN_INFO"%s: %d error\n", __FUNCTION__, usbd_admin_ndx);

                up(&usbd_sem);
                return -ENODEV;
        }
}


/* usbd_enable - 
 */
int usbd_enable(char *arg)
{
        //printk(KERN_INFO"%s:\n", __FUNCTION__);
        RETURN_ZERO_IF(switch_exiting);
        return usbd_admin(usbd_admin_enable, arg);
}

void usbd_enable_bh(void *arg)
{
        usbd_admin(usbd_admin_enable, (char *)arg);
}

int usbd_enable_irq(char *arg)
{
        RETURN_ZERO_IF(enable_bh.sync || switch_exiting);
        schedule_task(&enable_bh);
        return 0;
}


/* usbd_disable - 
 */
int usbd_disable(char *arg)
{
        RETURN_ZERO_IF(switch_exiting);
        return usbd_admin(usbd_admin_disable, arg);
}

void usbd_disable_bh(void *arg)
{
        usbd_admin(usbd_admin_disable, (char *)arg);
}

int usbd_disable_irq(char *arg)
{
        RETURN_ZERO_IF(disable_bh.sync || switch_exiting);
        schedule_task(&disable_bh);
        return 0;
}

/* usbd_load - 
 */
int usbd_load(char *arg)
{
        if (arg && strlen(arg)) {
                if ( usbd_load_arg)
                        lkfree(usbd_load_arg);
                usbd_load_arg = lstrdup(arg);
        }
        return usbd_admin(usbd_admin_enable, arg);
}

/* usbd_unload - 
 */
int usbd_unload(char *arg)
{
        return usbd_admin(usbd_admin_disable, arg);
}


/* usbd_connect - 
 */
int usbd_connect(char *arg)
{
        return usbd_admin(usbd_admin_connect, arg);
}

/* usbd_disconnect - 
 */
int usbd_disconnect(char *arg)
{
        return usbd_admin(usbd_admin_disconnect, arg);
}


/* usbd_replug - 
 */
int usbd_replug(char *arg)
{
        return usbd_admin(usbd_admin_connect, arg);
}

/* usbd_unplug - 
 */
int usbd_unplug(char *arg)
{
        return usbd_admin(usbd_admin_disconnect, arg);
}



/* usbd_pm_off - 
 */
int usbd_pm_off(char *arg)
{
        return usbd_admin(usbd_admin_pm_off, arg);
}

/* usbd_pm_on - 
 */
int usbd_pm_on(char *arg)
{
        return usbd_admin(usbd_admin_pm_on, arg);
}




/* ******************************************************************************************* */
#ifdef CONFIG_USBD_PROCFS

static int usbd_admin_procfs(char *arg)
//int usbd_admin_procfs(char *arg)
{
        usbd_admin_t admin;
        RETURN_EINVAL_IF(!arg || !strlen(arg));
        
        for (admin = usbd_admin_enable; admin <= usbd_admin_serial_number; admin++) {
                char *cp = usbd_admin_names[admin];
                int len = strlen(cp);
                if (!strncmp(arg, cp, len)) {
                        cp = arg + len;
                        while (cp && ((*cp == ' ') || (*cp == '\t'))) cp++;
                        return usbd_admin(admin, cp);
                }
        }

        for (admin = usbd_admin_enable; admin <= usbd_admin_serial_number; admin++) {
                char *cp = usbd_admin_aliases[admin];
                int len = strlen(cp);
                if (!strncmp(arg, cp, len)) {
                        cp = arg + len;
                        while (cp && ((*cp == ' ') || (*cp == '\t'))) cp++;
                        return usbd_admin(admin, cp);
                }
        }
        
        return -EINVAL;
}

/* usbd_proc_switch_write - handle a user switch request
 */
static ssize_t usbd_proc_switch_write (struct file *file, const char *buf, size_t count, loff_t * pos)
{

        char command[MAX_FUNC_NAME_LEN+1]; 
        size_t n = count;
        size_t l;
        char c;

        command[0] = 0;
        if (n > 0) {
                l = MIN(n,MAX_FUNC_NAME_LEN);
                if (copy_from_user (command, buf, l)) {
                        count = -EFAULT;
                } 
                else {
                        if (l > 0 && command[l-1] == '\n') {
                                l -= 1;
                        }
                        command[l] = 0;
                        n -= l;
                        // flush remainder, if any
                        while (n > 0) {
                                // Not too efficient, but it shouldn't matter
                                if (copy_from_user (&c, buf + (count - n), 1)) {
                                        count = -EFAULT;
                                        break;
                                }
                                n -= 1;
                        }
                }
        }

        if (0 >= count) {
                printk(KERN_INFO"%s: count <= 0 %d\n", __FUNCTION__, count);
                return count;
        }

        //printk(KERN_INFO"%s: %s\n", __FUNCTION__, command);
        if (!usbd_admin_procfs(command)) {
                // do nothing
                //printk(KERN_INFO"%s: back\n", __FUNCTION__);
        }


        /*
         * debug - increment switch_debug
         */
        else if (!strncmp("debug", command, 5)) {
                switch_debug++;
                if (switch_debug)
                        printk(KERN_INFO"%s: debug: %d\n", __FUNCTION__, switch_debug);
        }
        /*
         * nodebug - reset switch_debug
         */
        else if (!strncmp("nodebug", command, 7)) {
                switch_debug = 0;
                if (switch_debug)
                        printk(KERN_INFO"%s: no debug\n", __FUNCTION__);
        }

        return count;
}

/* *
 * usbd_proc_switch_read - implement proc file system read.
 * @file
 * @buf
 * @count
 * @pos
 *
 * Standard proc file system read function.
 *
 */
static ssize_t usbd_proc_switch_read (struct file *file, char *buf, size_t count, loff_t * pos)
{
	unsigned long page;
	int len;
	//int index;
	char *bp,*ep;
	struct list_head *lhd;

	if ((*pos)++ > 0) {
		/* This is the second time, we finished on the first time,
		   indicate we have no more data by returning 0. */
		return(0);
	}

	// get a page, max 4095 bytes of data...
	if (!(page = get_free_page (GFP_KERNEL))) {
		return -ENOMEM;
	}

	bp = (char *) page;
	ep = bp + 4095;

	bp += sprintf(bp, "USB cable ");
	if (NULL != usbd_bus_instance &&
	    NULL != usbd_bus_instance->driver &&
	    NULL != usbd_bus_instance->driver->bops &&
	    NULL != usbd_bus_instance->driver->bops->bus_attached &&
	    usbd_bus_instance->driver->bops->bus_attached(usbd_bus_instance)) {
		bp += sprintf(bp, "at");
	} else {
		bp += sprintf(bp, "de");
	}
	bp += sprintf(bp, "tached");
	bp += sprintf(bp, ", USB pullup resistor ");
	if (NULL != usbd_bus_instance &&
	    NULL != usbd_bus_instance->driver &&
	    NULL != usbd_bus_instance->driver->bops &&
	    NULL != usbd_bus_instance->driver->bops->bus_connected &&
	    !usbd_bus_instance->driver->bops->bus_connected(usbd_bus_instance)) {
		bp += sprintf(bp, "dis");
	}
	bp += sprintf(bp, "connected");
	bp += sprintf(bp, "\nUSB Device function drivers:\n");
	list_for_each (lhd, &usbd_function_drivers) {
		struct usb_function_driver *fd = list_entry(lhd, struct usb_function_driver, drivers);
		char enabled = '-';
		if (usbd_bus_instance && usbd_bus_instance->function_instance &&
		    usbd_bus_instance->function_instance->function_driver == fd) {
			enabled = '+';
		}
		if ((ep - bp) < 64) {
       			printk(KERN_INFO"%s: Too many usbd functions!\n", __FUNCTION__);
			break;
		}
		bp += sprintf(bp," %c [%s]\n",enabled,fd->name);
	}
	len = bp - (char *) page;

        if (len > 0 && copy_to_user (buf, (char *) page, len)) {
                //printk(KERN_INFO"%s: EFAULT\n", __FUNCTION__);
                len = -EFAULT;
        } else {
                //printk(KERN_INFO"%s: OK\n", __FUNCTION__);
        }
        free_page (page);
        return len;
}

static struct file_operations usbd_proc_switch_functions = {
                write:usbd_proc_switch_write,
                read:usbd_proc_switch_read,
};


#ifdef CONFIG_ARCH_EZX

//#define MOTUSBD_MAX_FUNCTION	10

int motusbd_status = USB_STATUS_NONE;		// USB_STATUS_MODEM;
extern __u32 usblan_ip_addr;

static char *motusbd_func_table[] = {
	USB_STATUS_STRING_MODEM,	// USB_STATUS_MODEM
	USB_STATUS_STRING_NET,		// USB_STATUS_NET
	USB_STATUS_STRING_CFG11,	// USB_STATUS_CFG11
	USB_STATUS_STRING_PST,		// USB_STATUS_PST
	USB_STATUS_STRING_STORAGE,	// USB_STATUS_STORAGE
	USB_STATUS_STRING_DSPLOG,
	USB_STATUS_STRING_NM,
	NULL,
};

static char *usbd_funcname_table[] = {
	"acm-CDC",
	"network-BLAN",
	"cfg11",		// ???
	"pst",
	"storage",		// ???
	"dsplog",
	"NM",          /*Motorola Network Monitor*/
	NULL,
};

extern int motusbd_nfs_flag;

// Interface function:
int motusbd_load(int status)
{
	int ret;
	char *arg = NULL;

	// use sema to protect motusb_status?
	
	// NFS mode: no switching!
	if((motusbd_nfs_flag) && (motusbd_status != USB_STATUS_NONE))
	{
		printk(KERN_ERR "%s: NFS mode, no switching: %d\n", __FUNCTION__, status);
		return 0;
	}
		
	if(status == motusbd_status)
		return 0;
	
	if(status != USB_STATUS_NONE)
	{
		arg = usbd_funcname_table[status];
		//printk(KERN_INFO "%s: load new driver %s\n", __FUNCTION__, arg);
		ret = usbd_load(arg);
	}
	else
	{
		//printk(KERN_INFO "%s: shutdown usb!\n", __FUNCTION__);
		ret = usbd_unload(arg);
		usbcable_status = 0;
	}
	//printk(KERN_INFO "\n !!! %s: pid %d, motusbd_status: new %d, old %d\n", __FUNCTION__, current->pid, status, motusbd_status);
	motusbd_status = status;
	return ret;
}


static ssize_t motusbd_proc_write (struct file *file, const char *buf, size_t count, loff_t * pos)
{
	char command[MAX_FUNC_NAME_LEN+1];
	size_t n = count;
	size_t l;
	char c;

	MOD_INC_USE_COUNT;
	command[0] = 0;
	if (n > 0) {
		l = MIN(n,MAX_FUNC_NAME_LEN);
		if (copy_from_user (command, buf, l))
			count = -EFAULT;
		else
		{
			if (l > 0 && command[l-1] == '\n') 
				l--;

			command[l] = 0;
			n -= l;
			while (n > 0) {
				if (copy_from_user (&c, buf + (count - n), 1)) {
					count = -EFAULT;
					break;
				}
				n--;
			}
		}
	}

	if(count > 0)
	{
		l = 0;
		do
		{
			if(!strcmp(command, motusbd_func_table[l]))
				break;
			l++;
		}while(motusbd_func_table[l] != NULL);

		if(motusbd_func_table[l] != NULL)
			motusbd_load(l);
		else
		{
			if(!strcmp(command, USB_STATUS_STRING_NONE))
				motusbd_load(USB_STATUS_NONE);
		}
	}

	MOD_DEC_USE_COUNT;
	return (count);
}

static ssize_t motusbd_proc_read(struct file *file, char *buf, size_t count, loff_t * pos)
{
	unsigned long page;
	int len = 0;
	int index;

	index = (*pos)++;
	//if(index > 3)
	//	return 0;

	MOD_INC_USE_COUNT;
	if (!(page = get_free_page (GFP_KERNEL))) {
		MOD_DEC_USE_COUNT;
		return -ENOMEM;
	}

	if(index == 1)
	{
		__u32 nh=0;
		unsigned char *cp;
		
		nh = htonl(usblan_ip_addr);		// ?????????????????
		cp = (unsigned char*) &nh;
		len += sprintf ((char *)page + len, "IP=%d.%d.%d.%d\n", cp[0], cp[1], cp[2], cp[3]);
	}
	else if(index == 2)
	{
		int status;
		if (NULL != usbd_bus_instance && NULL != usbd_bus_instance->driver &&
			NULL != usbd_bus_instance->driver->bops && NULL != usbd_bus_instance->driver->bops->bus_attached)
		{
			//if(0)
			//	status = usbd_bus_instance->driver->bops->bus_attached(usbd_bus_instance);
			//else
				status = usbcable_status;

			//printk(KERN_INFO "%s: usbcable_status %d\n", __FUNCTION__, usbcable_status);
			len += sprintf ((char *)page + len, "CABLE=%d\n", status);
		}
		else
		{
			len += sprintf ((char *)page + len, "CABLE=0\n");
		}
	}
	else if(index == 3)
	{
		if(usbcable_status && (NULL != usbd_bus_instance) && (NULL != usbd_bus_instance->function_instance)
			&& (NULL != usbd_bus_instance->function_instance->function_driver))
		{
			struct usb_configuration_descriptor *cfg_desc;
			cfg_desc = (&usbd_bus_instance->function_instance->function_driver->configuration_instance_array[0])->configuration_descriptor;
			if(cfg_desc->bMaxPower == POWER_CURRENT_500MA_SETTING)
				len += sprintf ((char *)page + len, "CHARGING=500\n");
			else if(cfg_desc->bMaxPower == POWER_CURRENT_100MA_SETTING)
				len += sprintf ((char *)page + len, "CHARGING=100\n");
			else
				len += sprintf ((char *)page + len, "CHARGING=0\n");
		}
		else
		{
			len += sprintf ((char *)page + len, "CHARGING=0\n");
		}
	}
	else if(index == 0)
	{
		switch(motusbd_status)
		{
			case USB_STATUS_MODEM:
				len += sprintf ((char *) page + len, USB_STATUS_STRING_MODEM);
				len += sprintf ((char *) page + len, "\n");
				break;
				
			case USB_STATUS_NET:
				len += sprintf ((char *) page + len, USB_STATUS_STRING_NET);
				len += sprintf ((char *) page + len, "\n");
				break;

			case USB_STATUS_CFG11:
				len += sprintf ((char *) page + len, USB_STATUS_STRING_CFG11);
				len += sprintf ((char *) page + len, "\n");
				break;

			case USB_STATUS_PST:
				len += sprintf ((char *) page + len, USB_STATUS_STRING_PST);
				len += sprintf ((char *) page + len, "\n");
				break;

			case USB_STATUS_NONE:
				len += sprintf ((char *) page + len, USB_STATUS_STRING_NONE);
				len += sprintf ((char *) page + len, "\n");
				break;
				
			case USB_STATUS_STORAGE:
				len += sprintf ((char *) page + len, USB_STATUS_STRING_STORAGE);
				len += sprintf ((char *) page + len, "\n");
				break;
        		
			case USB_STATUS_DSPLOG:
	                        len += sprintf ((char *) page + len, USB_STATUS_STRING_DSPLOG);
				len += sprintf ((char *) page + len, "\n");
				break;
                   
		        case USB_STATUS_NM:
			       len += sprintf ((char *) page + len, USB_STATUS_STRING_NM);
			       len += sprintf ((char *) page + len, "\n");
                               break;
			default:
				printk(KERN_ERR "%s: ERROR unknown motusbd_status %d\n", __FUNCTION__, motusbd_status);
				break;
		}
	}
	else
	{
		free_page(page);
		MOD_DEC_USE_COUNT;
		return 0;
	}

	if(len > count)
		len = -EINVAL;
	else if(len > 0 && copy_to_user (buf, (char *)page, len))
		len = -EFAULT;

	free_page(page);
	MOD_DEC_USE_COUNT;
	return len;
}


static struct file_operations motusbd_proc_ops = {
	write:	motusbd_proc_write,
	read:	motusbd_proc_read,
};

#endif		// end of CONFIG_ARCH_EZX
#endif		// end of CONFIG_USBD_PROCFS


/* ******************************************************************************************* */

#if defined(CONFIG_PM) 
/* switch_pm_event
 *
 * Handle power management event
 */
static int switch_pm_event (struct pm_dev *dev, pm_request_t rqst, void *unused)
{
	#ifdef DEBUG_USBC_PM
	printk(KERN_INFO"switch_pm_event:request: %d pm_dev: %p\n", rqst, dev);	
	#endif
	switch (rqst) {
	case PM_SUSPEND:
		#ifdef DEBUG_USBC_PM
		printk(KERN_INFO"switch_pm_event:USBC PM_SUSPEND\n");
		bulverde_udc_regs();
		#endif
		usbd_pm_off(usbd_load_arg);
		break;
	case PM_RESUME:
		#ifdef DEBUG_USBC_PM
		printk(KERN_INFO"switch_pm_event:USBC PM_RESUME.\n");
		#endif
                usbd_pm_on(usbd_load_arg);
		#ifdef DEBUG_USBC_PM
		bulverde_udc_regs();
		#endif
                break;
	}
	return 0;
}
#endif

/* ******************************************************************************************* */



/* ******************************************************************************************* */

#if defined(CONFIG_PM) 
struct pm_dev *switch_pm_info;
#endif

/* usbd_switch_init 
 *
 */
int usbd_switch_init(void)
{

#ifdef CONFIG_USBD_PROCFS
        struct proc_dir_entry *p;

        // create proc filesystem entries
        //printk(KERN_INFO"%s: creating /proc/usbd-switch\n", __FUNCTION__);
#ifdef CONFIG_ARCH_EZX
        if ((p = create_proc_entry ("usbd-switch_bmc", 0444, 0)) == NULL) {
#else
	if ((p = create_proc_entry ("usbd-switch", 0666, 0)) == NULL) {
#endif
                printk(KERN_ERR"%s: creating /proc/usbd-switch failed\n", __FUNCTION__);
                return -ENOMEM;
	}
        p->proc_fops = &usbd_proc_switch_functions;

#ifdef CONFIG_ARCH_EZX
	//printk(KERN_INFO"%s: creating %s\n", __FUNCTION__, MOTUSBD_PROCFILE);
	if ((p = create_proc_entry ("motusbd", 0666, 0)) == NULL)
		return -ENOMEM;
	p->proc_fops = &motusbd_proc_ops;
	if ((p = create_proc_entry ("usbd-switch", 0666, 0)) == NULL)
		return -ENOMEM;
	p->proc_fops = &motusbd_proc_ops;
#endif	// CONFIG_ARCH_EZX
	
#endif	// end of CONFIG_USBD_PROCFS
	
#ifdef CONFIG_PM
	switch_pm_info = NULL;

	if (!(switch_pm_info = pm_register (PM_USB_DEV, PM_SYS_UNKNOWN, switch_pm_event))) 
                return -ENOMEM;
#ifdef DEBUG_USBC_PM
	//printk(KERN_INFO"switch_pm_info has been registered.\n");
#endif
        switch_pm_info->state = 0;
#endif
        enable_bh.routine = usbd_enable_bh;
        enable_bh.data = NULL;
        disable_bh.routine = usbd_disable_bh;
        disable_bh.data = NULL;
	return 0;
}

void usbd_switch_exit (void)
{
        // This is *only* used for drivers compiled and used as a module.
#ifdef CONFIG_PM
        if (switch_pm_info)
                pm_unregister_all(switch_pm_event);
	switch_pm_info = NULL;
#endif
#ifdef CONFIG_USBD_PROCFS
        remove_proc_entry("usbd-switch", NULL);
#endif
        switch_exiting = 1;
        while (enable_bh.sync || disable_bh.sync) {
                printk(KERN_INFO"%s: waiting for bh\n", __FUNCTION__);
                schedule_timeout(10 * HZ);
        }
}



/* Module init ******************************************************************************* */

int usbd_switch_init(void);
void usbd_switch_exit (void);

static int usbd_device_init (void)
{
        //printk (KERN_INFO "%s: %s %s\n", __FUNCTION__, __usbd_module_info, USBD_EXPORT_TAG);

        usbd_switch_init();

        return 0;
}

static void usbd_device_exit (void)
{
        //printk (KERN_INFO "%s: %s exiting\n", __FUNCTION__, __usbd_module_info);

        usbd_switch_exit();

        if ( usbd_load_arg)
                lkfree(usbd_load_arg);

#ifdef MALLOC_TEST
        if (usbd_mallocs) printk(KERN_INFO"%s: usbd_mallocs non-zero! %d\n", __FUNCTION__, usbd_mallocs);
#endif
}

module_init (usbd_device_init);
module_exit (usbd_device_exit);

EXPORT_SYMBOL(usbd_function_drivers);
EXPORT_SYMBOL(usbd_maxstrings);

EXPORT_SYMBOL(usbd_enable);
EXPORT_SYMBOL(usbd_disable);
EXPORT_SYMBOL(usbd_enable_irq);
EXPORT_SYMBOL(usbd_disable_irq);

EXPORT_SYMBOL(usbd_connect);
EXPORT_SYMBOL(usbd_disconnect);
EXPORT_SYMBOL(usbd_pm_on);
EXPORT_SYMBOL(usbd_pm_off);

EXPORT_SYMBOL(usbd_load);
EXPORT_SYMBOL(usbd_unload);
EXPORT_SYMBOL(usbd_unplug);
EXPORT_SYMBOL(usbd_replug);


EXPORT_SYMBOL(usbd_bus_instance);
#ifdef MALLOC_TEST
EXPORT_SYMBOL(usbd_mallocs);
#endif


