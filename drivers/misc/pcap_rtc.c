/*
 * linux/drivers/char/pcap_rtc.c
 *
 * Copyright (C) 2004-2005 Motorola
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 */

/*
 * Module Name: pcap_rtc.c
 *
 * General Description:
 *      PCAP RTC driver, used to get/set RTC TOD/TODA/DAY/DAYA values.
 *
 * Revision History:
 *
 * Author       Date            Number          Description of Changes
 * ------       ----            -----           ----------------------
 * e12051       12/14/2004      LIBbbxxx        initial vertion
 *
 */

/*
 * Include files
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>   /* PCAP_RTC_MINOR and misc_dev... */
#include <linux/errno.h>        /* ... */
#include <linux/time.h>         /* struct timeval */
#include <linux/types.h>        /* time_t */
#include <linux/fs.h>           /* file_operations */
#include <linux/power_ic.h>
#include <linux/pcap_rtc.h>

#include <asm/uaccess.h>        /* copy_to/from_user */
#include <asm/string.h>         /* memset */


/* constant defines */
#define DEBUG
#define PCAP_RTC_OPENED         0x01    /* means that /dev/pcap_rtc in use */

/* seconds since 1970 to 2000 */
/* #define NUMBER_OF_SECONDS       ((2000 - 1970) * 365 * 24 * 3600) */


/* extern */
extern int rtc_ioctl(unsigned int cmd, unsigned long arg);


/* local variables */
static unsigned long pcap_rtc_status = 0;       /* bitmapped status var */
static spinlock_t pcap_rtc_lock = SPIN_LOCK_UNLOCKED;


/* We use pcap_rtc_lock to protect againest concurrent opens. */
static int pcap_rtc_open(struct inode *inode, struct file *file)
{
        spin_lock(&pcap_rtc_lock);

        if (pcap_rtc_status & PCAP_RTC_OPENED) {
#ifdef DEBUG
                printk("PCAP_RTC: open twice.\n");
#endif
                spin_unlock(&pcap_rtc_lock);
                return -EBUSY;
        }

        pcap_rtc_status |= PCAP_RTC_OPENED;
        spin_unlock(&pcap_rtc_lock);

        return 0;
}

static int pcap_rtc_close(struct inode *inode, struct file *file)
{
        spin_lock(&pcap_rtc_lock);
        pcap_rtc_status &= ~PCAP_RTC_OPENED;
        spin_unlock(&pcap_rtc_lock);

        return 0;
}

static int pcap_rtc_ioctl(struct inode *inode, struct file *file,
                          unsigned int cmd, unsigned long arg)
{
        int ret = 0;

        switch (cmd) {
        case PCAP_RTC_IOC_GET_TIME:
                ret = rtc_ioctl(POWER_IC_IOCTL_GET_TIME, arg);
                break;
        case PCAP_RTC_IOC_SET_TIME:
                ret = rtc_ioctl(POWER_IC_IOCTL_SET_TIME, arg);
                break;
        case PCAP_RTC_IOC_GET_ALARM_TIME:
                ret = rtc_ioctl(POWER_IC_IOCTL_GET_ALARM_TIME, arg);
                break;
        case PCAP_RTC_IOC_SET_ALARM_TIME:
                ret = rtc_ioctl(POWER_IC_IOCTL_SET_ALARM_TIME, arg);
                break;
        default:
#ifdef DEBUG
                printk("0x%X unsupported ioctl command\n", (int)cmd);
#endif                
                ret = -ENOTTY;
                break;
        }

        return ret;
}

void update_xtime(void)
{
        struct timeval tv;

        power_ic_rtc_get_time(&tv);
        do_settimeofday(&tv);

        return;
}
EXPORT_SYMBOL(update_xtime);


static struct file_operations pcap_rtc_fops = {
        .owner=         THIS_MODULE,
        .ioctl=         pcap_rtc_ioctl,
        .open=          pcap_rtc_open,
        .release=       pcap_rtc_close,
};

static struct miscdevice pcap_rtc_miscdev = {
        .minor=         PCAP_RTC_MINOR,
        .name=          PCAP_RTC_NAME,
        .fops=          &pcap_rtc_fops,
};


static int __init pcap_rtc_init(void)
{
        int ret;

        ret = misc_register(&pcap_rtc_miscdev);

        return ret;
}

static void __exit pcap_rtc_exit(void)
{
        misc_deregister(&pcap_rtc_miscdev);

        return;
}


module_init(pcap_rtc_init);
module_exit(pcap_rtc_exit);

MODULE_AUTHOR("e12051@motorola.com");
/* MODULE_LICENCE("GPL"); */
