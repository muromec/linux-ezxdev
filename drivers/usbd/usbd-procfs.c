/*
 * usbd/usbd-procfs.c - USB Device Core Layer
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
 * 2004-Dec-06 - Modified for EZXBASE debug  By Zhao Liang <w20146@motorola.com>
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

EXPORT_NO_SYMBOLS;

#include "usbd-export.h"
#include "usbd-build.h"

#ifdef MODULE
MODULE_AUTHOR ("sl@lineo.com, tbr@lineo.com");
MODULE_DESCRIPTION ("USB Device Core Support Procfs");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,17)
MODULE_LICENSE("GPL");
#endif

#endif

#include <linux/init.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/smp_lock.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/string.h>

#include "usbd-chap9.h"
#include "usbd-mem.h"
#include "usbd.h"
#include "usbd-func.h"
#include "usbd-bus.h"

USBD_MODULE_INFO ("usbdprocfs 2.0-beta");

#define MAX_INTERFACES 2

extern struct list_head usbd_function_drivers;
extern struct usb_bus_instance *usbd_bus_instance;


#ifdef CONFIG_USBD_PROCFS
/* Proc Filesystem *************************************************************************** */


/* *
 * dohex
 *
 */
static void dohexdigit (char *cp, unsigned char val)
{
	if (val < 0xa) {
		*cp = val + '0';
	} else if ((val >= 0x0a) && (val <= 0x0f)) {
		*cp = val - 0x0a + 'a';
	}
}

/* *
 * dohex
 *
 */
static void dohexval (char *cp, unsigned char val)
{
	dohexdigit (cp++, val >> 4);
	dohexdigit (cp++, val & 0xf);
}

/* *
 * dump_descriptor
 */
static int dump_descriptor (char *buf, char *sp)
{
        int num = *sp;
	int len = 0;

        //printk(KERN_INFO"%s: %p %d %d %d\n", __FUNCTION__, buf, *buf, buf[0], num);

	while (sp && num--) {
		dohexval (buf, *sp++);
		buf += 2;
		*buf++ = ' ';
		len += 3;
	}
	len++;
	*buf = '\n';
	return len;
}

/* dump_descriptors
 */
static int dump_config_descriptor(char *buf, char *sp)
{
        struct usb_configuration_descriptor *config = (struct usb_configuration_descriptor *) sp;

        int wTotalLength = le16_to_cpu(config->wTotalLength);
        int bConfigurationValue = config->bConfigurationValue;
        int interface;
        int class;
        int endpoint;
        int total;

        interface = class = endpoint = 0;

        for (total = 0; wTotalLength; ) {
                //printk(KERN_INFO"%s: wTotalLength: %d total: %d bLength: %d\n", 
                //                __FUNCTION__, wTotalLength, total, sp[0]);
                switch (sp[1]) {
                case USB_DESCRIPTOR_TYPE_CONFIGURATION:
                        interface = class = endpoint = 0;
                        total += sprintf(buf + total, "\nConfiguration descriptor [%d      ] ", 
                                        bConfigurationValue); 
                        break;
                case USB_DESCRIPTOR_TYPE_INTERFACE:
                        class = 0;
                        total += sprintf(buf + total, "\nInterface descriptor     [%d:%d:%d  ] ", 
                                        bConfigurationValue, ++interface, class); 
                        break;
                case USB_DESCRIPTOR_TYPE_ENDPOINT:
                        class = endpoint = 0;
                        total += sprintf(buf + total, "Endpint descriptor       [%d:%d:%d:%d] ", 
                                        bConfigurationValue, interface, class, ++endpoint); 
                        break;
                default:
                        endpoint = 0;
                        total += sprintf(buf + total, "Class descriptor         [%d:%d:%d  ] ", 
                                        bConfigurationValue, interface, ++class); 
                        break;
                }
                total += dump_descriptor(buf + total, sp);
                wTotalLength -= sp[0];
                sp += sp[0];
        }
        total += sprintf(buf + total, "\n"); 
        return total;
}

/* *
 * usbd_device_proc_read - implement proc file system read.
 * @file
 * @buf
 * @count
 * @pos
 *
 * Standard proc file system read function.
 *
 * We let upper layers iterate for us, *pos will indicate which device to return
 * statistics for.
 */
static ssize_t usbd_device_proc_read_functions (struct file *file, char *buf, size_t count, loff_t * pos)
{
	unsigned long page;
	int len = 0;
	int index;

        u8 config_descriptor[512];
        int config_size;

	//struct list_head *lhd;

	// get a page, max 4095 bytes of data...
	if (!(page = get_free_page (GFP_KERNEL))) {
		return -ENOMEM;
	}

	len = 0;
	index = (*pos)++;

	if (index == 0) {
		len += sprintf ((char *) page + len, "usb-device list\n");
        }

        //printk(KERN_INFO"%s: index: %d len: %d\n", __FUNCTION__, index, len);

        if (usbd_bus_instance && usbd_bus_instance->function_instance) {
                int configuration = index;
                struct usb_function_instance *function_instance = usbd_bus_instance->function_instance;
                struct usb_function_driver *function_driver = function_instance->function_driver;
                struct usb_configuration_instance *configuration_instance_array = function_driver->configuration_instance_array;
                if (configuration_instance_array) {

                        if (index == 0) {
                                len += sprintf ((char *) page + len, "\nDevice descriptor                  ");
                                len += dump_descriptor ((char *) page + len, (char *) function_driver->device_descriptor);
#ifdef CONFIG_USBD_HIGH_SPEED
                                len += sprintf ((char *) page + len, "\nDevice Qualifier descriptor        ");
                                len += dump_descriptor ((char *) page + len, 
                                                (char *) function_driver->device_qualifier_descriptor);
#endif
                        }
                        if (configuration < function_driver->bNumConfigurations) {

                                if ((config_size = usbd_get_descriptor(usbd_bus_instance, config_descriptor, 
                                                                sizeof(config_descriptor),
                                                                USB_DESCRIPTOR_TYPE_CONFIGURATION, 0)) > index) {
                                        len += dump_config_descriptor((char *)page + len, config_descriptor );
                                }
#ifdef CONFIG_USBD_HIGH_SPEED
                                if ((config_size = usbd_get_descriptor(usbd_bus_instance, config_descriptor, 
                                                                sizeof(config_descriptor),
                                                                USB_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION, index)) > 0) {
                                        len += dump_config_descriptor((char *)page + len, config_descriptor );
                                }

#endif
                        }
                        else if (configuration == function_driver->bNumConfigurations) {
                                int i;
                                int k;
                                struct usb_string_descriptor *string_descriptor;

                                //len += sprintf ((char *) page + len, "\n\n");

                                if ((string_descriptor = usbd_get_string (0)) != NULL) {
                                        len += sprintf ((char *) page + len, "String                   [%2d]      ", 0);

                                        for (k = 0; k < (string_descriptor->bLength / 2) - 1; k++) {
                                                len += sprintf ((char *) page + len, "%02x %02x ", 
                                                                string_descriptor->wData[k] >> 8, 
                                                                string_descriptor->wData[k] & 0xff);
                                                len++;
                                        }
                                        len += sprintf ((char *) page + len, "\n");
                                }

                                for (i = 1; i < usbd_maxstrings; i++) {

                                        if ((string_descriptor = usbd_get_string (i)) != NULL) {

                                                len += sprintf((char *)page+len, "String                   [%2d:%2d]   ", 
                                                                i, string_descriptor->bLength);

                                                // bLength = sizeof(struct usb_string_descriptor) + 2*strlen(str)-2;

                                                for (k = 0; k < (string_descriptor->bLength / 2) - 1; k++) {
                                                        *(char *) (page + len) = (char) string_descriptor->wData[k];
                                                        len++;
                                                }
                                                len += sprintf ((char *) page + len, "\n");
                                        }
                                }
                                len += sprintf((char *)page + len, "\n--\n"); 
                        }
                }
        }

        //printk(KERN_INFO"%s: len: %d count: %d\n", __FUNCTION__, len, count);

        if (len > count) {
                //printk(KERN_INFO"%s: len > count\n", __FUNCTION__);
                //printk(KERN_INFO"%s", page);
                len = -EINVAL;
        } 
        else if ((len > 0) && copy_to_user (buf, (char *) page, len)) {
                //printk(KERN_INFO"%s: EFAULT\n", __FUNCTION__);
                len = -EFAULT;
        }
        else {
                //printk(KERN_INFO"%s: OK\n", __FUNCTION__);
        }
        free_page (page);
        return len;
}

static struct file_operations usbd_device_proc_operations_functions = {
read:usbd_device_proc_read_functions,
};

#endif



/* Module init ******************************************************************************* */


static int usbd_procfs_init (void)
{
#ifdef CONFIG_USBD_PROCFS
        {
                struct proc_dir_entry *p;

		printk(KERN_INFO"%s: creating /proc/usb-functions\n", __FUNCTION__);
                // create proc filesystem entries
                if ((p = create_proc_entry ("usb-functions", 0, 0)) == NULL)
                        return -ENOMEM;
                p->proc_fops = &usbd_device_proc_operations_functions;
       }
#endif
        return 0;
}

static void usbd_procfs_exit (void)
{
#ifdef CONFIG_USBD_PROCFS
        // remove proc filesystem entry
        remove_proc_entry ("usb-functions", NULL);
#endif
}

module_init (usbd_procfs_init);
module_exit (usbd_procfs_exit);

