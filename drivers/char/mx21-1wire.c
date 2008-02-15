/*******************************************************************************************************

	mx21-1wire.c
	1-wire driver for MX2ADS

	Author: MontaVista Software, Inc. <source@mvista.com>
	Copyright (C) 2004 MontaVista Software, Inc.
	
	Based on the 1-wire driver from Motorola MX21 BSP ver 1.0.2
	Copyright (C) 2003 Motorola Semiconductors Inc

	This file is licensed under the terms of the GNU General Public License version 2.
	This program is licensed "as is" without any warranty of any kind, whether express or implied.
	
*******************************************************************************************************/
#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <asm/uaccess.h>

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/init.h>
#include <asm/bitops.h>
#include <asm/io.h>
#include <linux/errno.h>
#include <linux/tqueue.h>
#include <linux/wait.h>

#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <linux/pm.h>
#include <asm/arch/pll.h>

#if 1 /*CEE LDM*/
#include <linux/device.h>
#include <linux/dpm.h>

extern void mx21_ldm_bus_register(struct device *device,
                          struct device_driver *driver);
extern void mx21_ldm_bus_unregister(struct device *device,
                          struct device_driver *driver);
static int
__ldm_suspend(struct device *dev, u32 state, u32 level);

static int
__ldm_resume(struct device *dev, u32 level);

#ifdef CONFIG_DPM
static struct constraints mx2_1wire_constraints = {
	.count = 1,
	.param = {{DPM_MD_HCLK, 0, 0}},	/*to be initialized at module init time */
	.asserted = 1,
};
#endif

static struct device_driver __driver_ldm = {
	.name = "mx21-1wire",
	.devclass = NULL,
	.probe = NULL,
	.suspend = __ldm_suspend,
	.resume = __ldm_resume,
	.scale = NULL,
	.remove = NULL,
};

static struct device __device_ldm = {
	.name = "1-Wire",
	.bus_id = "1wire",
	.driver = &__driver_ldm,
	.power_state = DPM_POWER_ON,
#ifdef CONFIG_DPM
	.constraints = &mx2_1wire_constraints,
#endif
};

#endif


#include "mx21-1wire.h"

static int Owire_open(struct inode *inode, struct file *filp);
static int Owire_release(struct inode *inode, struct file *filp);
static ssize_t Owire_read(struct file *filp,
			  char *buf, size_t count, loff_t * l);
static ssize_t Owire_write(struct file *file,
			   const char *buffer, size_t count, loff_t * ppos);
static int Owire_ioctl(struct inode *inode,
		       struct file *filp, unsigned int cmd, unsigned long arg);
static unsigned int Owire_poll(struct file *filp,
			       struct poll_table_struct *wait);

static struct file_operations g_Owire_fops = {
	.open = Owire_open,
	.release = Owire_release,
	.write = Owire_write,
	.read = Owire_read,
	.poll = Owire_poll,
	.ioctl = Owire_ioctl,
	.fasync = NULL,
};

static int
Owire_open(struct inode *inode, struct file *filp)
{

	/* set AIPI1_PSR0 bit[9] and clear AIPI1_PSR1bit[9] to match 1-wire bus width 16bits */

	if (g_Owire_status & OWIRE_SUSPEND_STATUS) return -EPERM;
	if (g_Owire_status & OWIRE_OPEN_STATUS) return -EBUSY;

	AIPI_PSR0(1) |= 0x00200;
	AIPI_PSR1(1) &= ~0x00200;

	/* enable clk */
	mx_module_clk_open(IPG_MODULE_OWIRE);
	/* GPIO config */

	mx2_register_gpios(PORT_E, (1 << 16) , SECONDARY | NOINTERRUPT);

	/* reset the 1-wire module */
	OWIRE_RESET |= 1;
	OWIRE_RESET = 0;

	/* set the time devider to get the 1M clk from the main clock */
	OWIRE_TIME_DIV = 0x2B;

	OWIRE_CTRL |= 0x80;	/* control register bit 7 = 1; */
	/* check if PST is set */
	mdelay(1);
	/* check if RPP self cleared */
	mdelay(1);
	g_Owire_status |= OWIRE_OPEN_STATUS;

	mx_module_clk_close(IPG_MODULE_OWIRE);

	MOD_INC_USE_COUNT;
	return 0;
}

static int
Owire_release(struct inode *inode, struct file *filp)
{
	g_Owire_status &= ~OWIRE_OPEN_STATUS;
	/* disable clk */
	mx_module_clk_close(IPG_MODULE_OWIRE);
	MOD_DEC_USE_COUNT;
	return 0;
}

static ssize_t
Owire_read(struct file *filp, char *buf, size_t count, loff_t * l)
{
	int i, j;
	char tmp, value;
	char *pBuf;
	pBuf = buf;

	if (g_Owire_status & OWIRE_SUSPEND_STATUS) return -EPERM;

	/* enable clk */
	mx_module_clk_open(IPG_MODULE_OWIRE);
	for (i = 0; i < count; i++) {
		value = 0;
		for (j = 0; j < 8; j++) {
			/* control register bit 4 = 1 */
			OWIRE_CTRL |= 0x10;
			udelay(117);

			tmp = (OWIRE_CTRL & 0x08) ? 1 : 0;
			value |= tmp << j;
		}
		__copy_to_user(pBuf, &value, 1);
		pBuf++;

	}
	mx_module_clk_close(IPG_MODULE_OWIRE);
	return count;
}

static ssize_t
Owire_write(struct file *file, const char *buffer, size_t count, loff_t * ppos)
{
	int i, j;
	char value, tmp;
	if (g_Owire_status & OWIRE_SUSPEND_STATUS) return -EPERM;

	/* enable clk */
	mx_module_clk_open(IPG_MODULE_OWIRE);

	for (i = 0; i < count; i++) {
		__copy_from_user(&value, buffer, 1);
		for (j = 0; j < 8; j++) {
			tmp = value & 0x01;
			if (tmp == 0) {	/* control register bit 5 = 1 */
				OWIRE_CTRL |= 0x20;
				udelay(117);
			} else if (tmp == 1) {	/* control register bit 4 = 1 */
				OWIRE_CTRL |= 0x10;
				udelay(117);
			}
			value >>= 1;
		}
		buffer++;
	}
	mx_module_clk_close(IPG_MODULE_OWIRE);
	return count;
}

/*
 * Function Name: check_device
 *
 * Input: 		inode	:
 * Value Returned:	int	: Return status.If no error, return 0.
 *
 * Description: verify if the inode is a correct inode
 *
 */
static int
check_device(struct inode *pInode)
{
	int minor;
	kdev_t dev = pInode->i_rdev;

	if (MAJOR(dev) != g_Owire_major) {
		return -1;
	}
	minor = MINOR(dev);

	if (minor < 1)
		return minor;
	else {
		return -1;
	}
}

/*
 * Function Name: Owire_ioctl
 *
 * Input: 	inode	:
 * 			filp	:
 * 			cmd	: command for ioctl
 * 			arg	: parameter for command
 * Value Returned:	int	: Return status.If no error, return 0.
 *
 * Description: ioctl for this device driver
 *
 * 	
 */
static int
Owire_ioctl(struct inode *inode,
	    struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = -EIO;
	int minor;

	minor = check_device(inode);
	if (minor == -1) {
		return -ENODEV;
	}

	return ret;
}

/*
 * Function Name: Owire_poll
 *
 * Input: 		filp	:
 * 			wait	:
 * Value Returned:	int	: Return status.If no error, return 0.
 *
 * Description: support poll and select
 *
 */
static unsigned int
Owire_poll(struct file *filp, struct poll_table_struct *wait)
{
	return 0;
}

static void
mx2_Owire_suspend(void)
{
	g_Owire_status |= OWIRE_SUSPEND_STATUS;
}

static void
mx2_Owire_resume(void)
{
	g_Owire_status &= ~OWIRE_SUSPEND_STATUS;
}

static int
Owire_pm_handler(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	switch (rqst) {
	case PM_RESUME:
		mx2_Owire_resume();
		break;
	case PM_SUSPEND:
		mx2_Owire_suspend();
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
		mx2_Owire_suspend();
		break;
	}
	return 0;
}

static int
__ldm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		mx2_Owire_resume();
		break;
	}
	return 0;
}
#endif

#define MX2ADS_1WIRE_MIN_HCLK  88000

static int __init
Owire_init(void)
{
#if 1 /*CEE LDM*/
	if (MX2ADS_1WIRE_MIN_HCLK > mx_module_get_clk(HCLK) / 1000) {
		printk(KERN_ERR MODULE_NAME ": cannot initialize - HCLK too slow\n");
		return -EPERM;
	}
#endif

	/* register our character device */
	g_Owire_major = devfs_register_chrdev(0, MODULE_NAME, &g_Owire_fops);
	if (g_Owire_major < 0) {
		printk(KERN_ERR MODULE_NAME ": Unable to register characted device\n");
		return -ENODEV;
	}

	g_devfs_handle = devfs_register(NULL, MODULE_NAME, DEVFS_FL_DEFAULT,
					g_Owire_major, 0,
					S_IFCHR | S_IRUSR | S_IWUSR,
					&g_Owire_fops, NULL);

	/* Modified by Bill, May 9, 2004 */
	g_Owire_pm = pm_register(PM_SYS_DEV, PM_SYS_UNKNOWN, Owire_pm_handler);
#if 1 /*CEE LDM*/
#ifdef CONFIG_DPM
	mx2_1wire_constraints.param[0].min = MX2ADS_1WIRE_MIN_HCLK;	/*in KHz */
	mx2_1wire_constraints.param[0].max = mx_module_get_clk(MPLL) / 1000;	/* unlimited */
#endif
	mx21_ldm_bus_register(&__device_ldm, &__driver_ldm);
#endif
	g_Owire_status = 0;

	return 0;
}

static void __exit
Owire_cleanup(void)
{
	devfs_unregister_chrdev(g_Owire_major, MODULE_NAME);
	devfs_unregister(g_devfs_handle);
	pm_unregister(g_Owire_pm);
#if 1 /*CEE LDM*/
	mx21_ldm_bus_unregister(&__device_ldm, &__driver_ldm);
#endif
}

module_init(Owire_init);
module_exit(Owire_cleanup);
MODULE_LICENSE("GPL");
