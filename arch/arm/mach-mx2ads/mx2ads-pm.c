/* 
 * arch/arm/mach-mx2ads/mx2ads-pm.c
 *
 * Power management for the MX21ADS
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sysctl.h>
#include <linux/pm.h>
#include <linux/delay.h>

#include <asm/hardware.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/arch/irqs.h>

#define PREFIX "mx2ads-pm:"

/* number for the MX2ADS sleep sysctl()s
 *
 * This is picked at random out of thin air, hoping that it won't
 * clash with someone.  That's really ugly, but appears to be
 * "standard" practice (!?).  Oh well, with any luck we can throw
 * these away and replace them with sysfs parameters, in the
 * not-too-distant future...
 */

#define CTL_PM_MX2ADS 0x81c4

static int
suspend_handler(void)
{
	int retval;
	
	if ((retval = pm_send_all(PM_SUSPEND, (void *) 3))) {
		printk(KERN_ERR PREFIX "Device suspend failed. \n");
		return retval;
	}

	/* Stop PLLs */
	CRM_CSCR &= ~CSCR_MPEN;

	cpu_do_idle();

	if ((retval = pm_send_all(PM_RESUME, (void *) 0))) {
		printk(KERN_ERR PREFIX "Device resume failed. \n");
		return retval;
	}

	return 0;
}

/*
 * /proc interface for power managment
 * 
 */
static int
suspend_proc_handler(ctl_table * ctl, int write, struct file *filp,
		     void *buffer, size_t * lenp)
{
	return suspend_handler();
}

static struct ctl_table pm_table[] = {
	{
	 .ctl_name = CTL_PM_MX2ADS,
	 .procname = "sleep",
	 .mode = 0200,		/* write-only to trigger sleep */
	 .proc_handler = &suspend_proc_handler,
	 },
	{0}
};

/* Initialize power interface */
static int __init
mx2ads_pm_init(void)
{
	register_sysctl_table(pm_table, 1);
	return 0;
}

__initcall(mx2ads_pm_init);
