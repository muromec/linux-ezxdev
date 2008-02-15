/*
 * linux/drivers/char/omap_wdt.c
 *
 * Watchdog driver for the TI OMAP
 *
 * Author: MontaVista Software, Inc.
 *         <gdavis@mvista.com> or <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * History:
 *
 * 20030527: George G. Davis <gdavis@mvista.com>
 *	Initially based on linux-2.4.19-rmk7-pxa1/drivers/char/sa1100_wdt.c
 *	(c) Copyright 2000 Oleg Drokin <green@crimea.edu>
 *	Based on SoftDog driver by Alan Cox <alan@redhat.com>
 *
 * TODO:
 * 1. Need to disable watchdog when entering chip idle mode.
 */

#include <linux/module.h>
#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/smp_lock.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/bitops.h>
#include <asm/arch/ck.h>

#define TIMER_MARGIN	19	/* Default is 19 seconds */
#define TIMER_RATE	(12000000/14/256)

#if defined(CONFIG_ARCH_OMAP730)
#define OMAP1510_WATCHDOG_BASE OMAP730_WATCHDOG_BASE
#elif defined(CONFIG_ARCH_OMAP1610)
#define OMAP1510_WATCHDOG_BASE OMAP1610_WATCHDOG_BASE
#endif

static int timer_margin;	/* in seconds */
static int pre_margin;
static int omap_wdt_users;

static void
omap_wdt_ping(void)
{
	/* reload counter with (new) margin */
	pre_margin ^= 0xf; /* new LOAD_TIME must differ from previous */
	outw(pre_margin, OMAP1510_WATCHDOG_BASE + 4);
}

/*
 *	Allow only one person to hold it open
 */

static int
omap_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(1, (volatile unsigned long *) &omap_wdt_users))
		return -EBUSY;

	ck_enable(mpuwd_ck);
	outw(0x8000, OMAP1510_WATCHDOG_BASE + 8); /* set TIMER_MODE:WATCHDOG */
	timer_margin = TIMER_MARGIN;	/* default timeout */
	pre_margin = TIMER_RATE * timer_margin;
	return 0;

}

static int
omap_wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *      Shut off the timer.
	 *      Lock it in if it's a module and we defined ...NOWAYOUT
	 */
#ifndef CONFIG_WATCHDOG_NOWAYOUT
	outw(0xf5, OMAP1510_WATCHDOG_BASE + 8); /* timer mode */
	outw(0xa0, OMAP1510_WATCHDOG_BASE + 8); /* timer mode */
	ck_disable(mpuwd_ck);
#endif
	omap_wdt_users = 0;
	return 0;
}

static ssize_t
omap_wdt_write(struct file *file, const char *data, size_t len, loff_t * ppos)
{
	/*  Can't seek (pwrite) on this device  */
	if (ppos != &file->f_pos)
		return -ESPIPE;

	/* Refresh LOAD_TIME. */
	if (len) {
		omap_wdt_ping();
		return 1;
	}
	return 0;
}

static int
omap_wdt_ioctl(struct inode *inode, struct file *file,
	       unsigned int cmd, unsigned long arg)
{
	int new_margin;
	static struct watchdog_info ident = {
		.identity		= "OMAP Watchdog",
		.options		= WDIOF_SETTIMEOUT,
		.firmware_version	= 0,
	};

	switch (cmd) {
	default:
		return -ENOIOCTLCMD;
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info *) arg, &ident,
				    sizeof (ident));
	case WDIOC_GETSTATUS:
		return put_user(0, (int *) arg);
	case WDIOC_GETBOOTSTATUS:
		return put_user(inw(ARM_SYSST), (int *) arg);
	case WDIOC_KEEPALIVE:
		omap_wdt_ping();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, (int *) arg))
			return -EFAULT;
		if (new_margin < 1 || new_margin > 19)
			timer_margin = TIMER_MARGIN;	/* default timeout */
		else
			timer_margin = new_margin;
		pre_margin = TIMER_RATE * timer_margin;
		omap_wdt_ping();
		/* Fall */
	case WDIOC_GETTIMEOUT:
		return put_user(timer_margin, (int *) arg);
	}
}

static struct file_operations omap_wdt_fops = {
	.owner		= THIS_MODULE,
	.write		= omap_wdt_write,
	.ioctl		= omap_wdt_ioctl,
	.open		= omap_wdt_open,
	.release	= omap_wdt_release,
};

static struct miscdevice omap_wdt_miscdev = {
	.minor	= WATCHDOG_MINOR,
	.name	= "omap_wdt",
	.fops	= &omap_wdt_fops
};

static int __init
omap_wdt_init(void)
{
	int ret;

	ret = misc_register(&omap_wdt_miscdev);

	if (ret)
		return ret;

	/* Disable watchdog if it is already running */
	if (inw(OMAP1510_WATCHDOG_BASE + 8) & 0x8000) {
		/* Magic sequence required to disable watchdog */
		outw(0xf5, OMAP1510_WATCHDOG_BASE + 8); /* TIMER_MODE */
		outw(0xa0, OMAP1510_WATCHDOG_BASE + 8); /* TIMER_MODE */
	}

	if (timer_margin < 1 || timer_margin > 19)
		timer_margin = 19;

	printk(KERN_INFO "%s: TI OMAP Watchdog Timer: timer margin %d sec\n",
	       omap_wdt_miscdev.name, timer_margin);

	return 0;
}

static void __exit
omap_wdt_exit(void)
{
	misc_deregister(&omap_wdt_miscdev);
}

module_init(omap_wdt_init);
module_exit(omap_wdt_exit);

MODULE_AUTHOR("George G. Davis");
MODULE_LICENSE("GPL");
MODULE_PARM(timer_margin, "i");
EXPORT_NO_SYMBOLS;
