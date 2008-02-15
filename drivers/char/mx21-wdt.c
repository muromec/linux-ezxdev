/*
 *  dbmx21_wdt.c -- Dragonball MX21 Watch Dog Timer driver
 *
 */
/*
 *  Copyright 2002 Sony Corporation.
 *  Copyright (C) 2003 MontaVista Software Inc. <source@mvista.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  version 2 of the  License.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/smp_lock.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/arch/hardware.h>

#include <linux/pm.h>
#include <asm/arch/pll.h>

#if 1 /*CEE LDM*/
#include <linux/device.h>

extern void mx21_ldm_bus_register(struct device *device,
                          struct device_driver *driver);
extern void mx21_ldm_bus_unregister(struct device *device,
                          struct device_driver *driver);
static int
__ldm_suspend(struct device *dev, u32 state, u32 level);

static int
__ldm_resume(struct device *dev, u32 level);

static struct device_driver __driver_ldm = {
	.name = "mx21-wdt",
	.suspend = __ldm_suspend,
	.resume = __ldm_resume,
};

static struct device __device_ldm = {
	.name = "WDT",
	.bus_id = "wdt",
	.driver = &__driver_ldm,
	.power_state = DPM_POWER_ON,
};

#endif

#include "mx21-wdt.h"

static int debug = 0;
MODULE_PARM(debug, "i");
#define wdt_printk( level, args... ) if( (level)<=debug ) pr_debug( args )

#define ENTER() wdt_printk( 3, ">>> %s entered\n", __FUNCTION__ )
#define LEAVE() wdt_printk( 3, "<<< %s left out\n", __FUNCTION__ )
#define RETURN( something ) do { LEAVE(); return (something); } while( 0 )

static int wdt_is_open = 0;

/* -----------------------
 *  FUNCTION ROUTINES
 * -----------------------
 */

static void
wdt_disable(void)
{
	unsigned int out_data;
	ENTER();
	WDOG_WPR = WDT_DISABLE1;
	mdelay(500);
	out_data = WDOG_WPR;
	mdelay(500);
	wdt_printk(1, "disable: WDOG_WPR=0x%x\n", out_data);
	WDOG_WPR = WDT_DISABLE2;
	mdelay(500);
	WDOG_WPR = WDT_DISABLE3;
	mdelay(500);
	wdt_printk(1, "disable: WCR=0x%x\n", WDOG_WCR);
	mx_module_clk_close(IPG_MODULE_WDT);
	LEAVE();
}

static void
wdt_ping(void)
{
	ENTER();
	wdt_printk(1, "WCR=0x%x\n", WDOG_WCR);
	WDOG_WSR = 0x5555;
	wdt_printk(1, "WSR=0x%x\n", WDOG_WCR);
	WDOG_WSR = 0xaaaa;
	wdt_printk(1, "WSR=0x%x\n", WDOG_WCR);
	LEAVE();
}

static unsigned int
wdt_getsts(void)
{
	unsigned int sts;

	ENTER();
	sts = WDOG_WRSR;	/* Watchdog Status Reg Read */
	sts &= (WRSR_SFTW | WRSR_TOUT | WRSR_EXT | WRSR_PWR);
	RETURN(sts);
}

static void
wdt_setwdt(void)
{
	ENTER();
	mx_module_clk_open(IPG_MODULE_WDT);
	wdt_disable();
	mx_module_clk_open(IPG_MODULE_WDT);
	/* Watchdog Parameter Set & Enable      */
	/*   WatchdogParameter                  */
	/*     WatchTime    =60sec              */
	/*     Signal       =WDT_RST            */
	WDOG_WCR |= 15 << 9;
	WDOG_WCR |= WDE;
	LEAVE();
}

static int
wdt_open(struct inode *inode, struct file *file)
{
	ENTER();
	if (wdt_is_open) {
		RETURN(-EBUSY);
	}

	wdt_is_open = 1;
	wdt_setwdt();
	RETURN(0);
}

static int
wdt_close(struct inode *inode, struct file *file)
{
	ENTER();

	lock_kernel();
#ifndef CONFIG_WATCHDOG_NOWAYOUT
	wdt_printk(1,
		   "%s: disabling the watchdog. You may keep the device open\n",
		   __FUNCTION__);
	wdt_disable();
#endif
	wdt_is_open = 0;
	unlock_kernel();
	RETURN(0);
}

static ssize_t
wdt_read(struct file *file, char *buf, size_t count, loff_t * ptr)
{
	unsigned int status;

	ENTER();
	/*  Can't seek (pread) on this device  */
	if (ptr != &file->f_pos) {
		RETURN(-ESPIPE);
	}

	status = wdt_getsts();
	wdt_printk(1, "read: WCR=0x%x\n", WDOG_WCR);
	copy_to_user(buf, &status, sizeof (unsigned int));
	RETURN(sizeof (unsigned int));
}

static ssize_t
wdt_write(struct file *file, const char *buf, size_t count, loff_t * ppos)
{
	ENTER();
	/*  Can't seek (pwrite) on this device  */
	if (ppos != &file->f_pos) {
		RETURN(-ESPIPE);
	}

	if (count > 0) {
		wdt_ping();
		RETURN(1);
	}

	RETURN(0);
}

static int
wdt_ioctl(struct inode *inode, struct file *file,
	  unsigned int cmd, unsigned long arg)
{
	return -ENOTTY;
}

static struct file_operations wdt_fops = {
	owner:THIS_MODULE,
	open:wdt_open,		/* open  */
	release:wdt_close,	/* close */
	read:wdt_read,		/* read  */
	write:wdt_write,	/* write */
	ioctl:wdt_ioctl		/* ioctl */
};

static struct miscdevice wdt_miscdev = {
	WATCHDOG_MINOR,
	WDT_DEV_NAME,
	&wdt_fops
};

static int
handle_pm_event(struct pm_dev *dev, pm_request_t rq, void *data)
{
	switch (rq) {
	case PM_SUSPEND:
		lock_kernel();
#ifndef CONFIG_WATCHDOG_NOWAYOUT
		wdt_printk(1,
			   "%s: disabling the watchdog. You may keep the device open\n",
			   __FUNCTION__);
		wdt_disable();
#endif
		unlock_kernel();
		break;
	case PM_RESUME:
		wdt_printk(2, ("%s: resume\n", __FUNCTION__));
		if (wdt_is_open)
			wdt_setwdt();
		break;
	default:
		break;
	}
	return 0;
}

#if 1 /*CEE LDM*/
static int
__ldm_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		lock_kernel();
#ifndef CONFIG_WATCHDOG_NOWAYOUT
		wdt_printk(1,
			   "%s: disabling the watchdog. You may keep the device open\n",
			   __FUNCTION__);
		wdt_disable();
#endif
		unlock_kernel();
		break;
	}
	return 0;
}

static int
__ldm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		wdt_printk(2, ("%s: resume\n", __FUNCTION__));
		if (wdt_is_open)
			wdt_setwdt();
		break;
	}
	return 0;
}
#endif

static int __init
wdt_init(void)
{
	int ret;

	ENTER();

	ret = misc_register(&wdt_miscdev);

	if (ret < 0) {
		return ret;
	}
	pm_register(PM_SYS_DEV, PM_SYS_UNKNOWN, handle_pm_event);
#if 1 /*CEE LDM*/
	mx21_ldm_bus_register(&__device_ldm, &__driver_ldm);
#endif

	RETURN(0);
}

static void __exit
wdt_exit(void)
{
	ENTER();
#if 1 /*CEE LDM*/
	mx21_ldm_bus_unregister(&__device_ldm, &__driver_ldm);
#endif
	pm_unregister_all(handle_pm_event);
	misc_deregister(&wdt_miscdev);
	wdt_disable();
	LEAVE();
}

module_init(wdt_init);
module_exit(wdt_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MX21 Watchdog timer");
