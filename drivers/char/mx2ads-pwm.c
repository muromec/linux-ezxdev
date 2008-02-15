/*
 *  MX2 Pulse Width Modulator
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Copyright (C) 2002 Motorola Semiconductors HK Ltd
 *  Copyright (C) 2004 MontaVista Software Inc. <source@mvista.com>
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <asm/uaccess.h>	/* get_user,copy_to_user */

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/slab.h>

#include <linux/string.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/pm.h>
#include <linux/devfs_fs_kernel.h>

#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>

#include <asm/arch/gpio.h>
#include <asm/arch/pll.h>

#define PKMOD "PWM: "

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
	.name = "mx2ads-pwm",
	.suspend = __ldm_suspend,
	.resume = __ldm_resume,
};

static struct device __device_ldm = {
	.name = "PWM",
	.bus_id = "pwm",
	.driver = &__driver_ldm,
	.power_state = DPM_POWER_ON,
};

#endif

#include "mx2ads-pwm.h"


static struct timer_list pwm_timer;
static unsigned long pwm_timer_period;

/* variables */
static int pwm_mode, pwm_swap, pwm_sampling;
static char *gpWriteBuf;
static u8 *gpWriteBufPtr8;
static u16 *gpWriteBufPtr16;
static int pwm_write_cnt;
static int pwm_data_len;
static u16 pwm_sample_value;
static char *pwmp = "pwm";
static devfs_handle_t g_devfs_handle;

static char pwm_suspended, pwm_busy;
static DECLARE_MUTEX_LOCKED(pwm_wait);

MODULE_LICENSE("GPL");

static int gMajor = 0;

static struct file_operations ts_fops = {
	.open = pwm_open,
	.release = pwm_release,
	.read = pwm_read,
	.write = pwm_write,
	.ioctl = pwm_ioctl,
	.fasync = pwm_fasync,
};

/*
 * Function:
 *      pwm_delay
 * Synopsis:
 *      do the delay for period of clocks
 */
static void
pwm_delay(int clocks)
{
	while (--clocks > 0) {
		int ctr;

		for (ctr = 0; ctr < 100000; ctr++)
			continue;
	}
}

/*
 * Function:
 * 	pwm_soft_reset
 * Synopsis:
 * 	reset the PWM
 */
static void
pwm_soft_reset(void)
{
	PWM_PWMC |= PWM_EN;
	PWM_PWMC |= PWM_SWR;
	PWM_PWMC |= PWM_EN;
	pwm_delay(5);
}

/*
 * Function:
 *    pwm_release
 * Synopsis:
 *    release the filp
 */
static int
pwm_release(struct inode *inode, struct file *filp)
{
	MOD_DEC_USE_COUNT;
	pwm_busy = 0;
	return 0;
}

/*
 * Function:
 * 	pwm_open
 * Synopsis:
 *	open the device
 */
static int
pwm_open(struct inode *inode, struct file *filp)
{
	if (pwm_suspended) return -EPERM;
	if (pwm_busy) return -EBUSY;
	pwm_busy = 1;
	init_MUTEX_LOCKED(&pwm_wait);
	MOD_INC_USE_COUNT;
	return 0;
}

/*
 * Function:
 *	pwm_fasync
 */
static int
pwm_fasync(int fd, struct file *filp, int mode)
{
	return 0;
}

/*
 * Function:
 *	pwm_ioctl
 * Synopsis:
 *	handle IOCTL addressed to PWM device
 * Parameters:
 *      arg may be one of:
 *         PWM_IOC_SMODE     set PWM mode: tone/play
 *         PWM_IOC_SFREQ     set frequency of tone to play
 *         PWM_IOC_SDATALEN
 *         PWM_IOC_SSAMPLE
 *         PWM_IOC_SPERIOD
 *         PWM_IOC_STOP
 *         PWM_IOC_SWAPDATA
 */
static int
pwm_ioctl(struct inode *inode,
	  struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (pwm_suspended) return -EPERM;

	switch (cmd) {
	case PWM_IOC_SMODE:
		pwm_mode = arg;
		break;

	case PWM_IOC_SFREQ:
		pwm_sampling = arg;
		break;

	case PWM_IOC_SDATALEN:
		pwm_data_len = arg;
		break;

	case PWM_IOC_SSAMPLE:
		pwm_sample_value = arg;
		break;

	case PWM_IOC_SPERIOD:
		pwm_timer_period = arg / 12; /*good for tone mode only*/
		break;

	case PWM_IOC_STOP:
		/*all writes are blocking. No need to call this*/
		break;

	case PWM_IOC_SWAPDATA:
		pwm_swap = arg;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

/*
 * Function:
 *     pwm_read
 * Synopsis:
 *     handle the call of read
 */
static ssize_t
pwm_read(struct file * filp, char *buf, size_t count, loff_t * l)
{
	return (pwm_write_cnt == 0) ? 1 : 0;
}

/*
 * Function:
 * 	pwm_interrupt
 * Synopsis:
 *	IRQ handler
 */
static void
pwm_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	u16 tmp16;
	u32 status;

	/* clear status */
	status = PWM_PWMC;

	/* check end */
	if (pwm_write_cnt <= 0) {
		up(&pwm_wait);
		pwm_stop();
		return;
	}

	if (PWM_TONE_MODE != pwm_mode) {
		if (PWM_DATA_8BIT == pwm_data_len) {
			PWM_PWMS = (u32) (*gpWriteBufPtr8++);
			pwm_write_cnt--;
		} else /*if (pwm_data_len == PWM_DATA_16BIT)*/ {
			tmp16 = *gpWriteBufPtr8++;
			tmp16 |= (*gpWriteBufPtr8++ << 8);
			PWM_PWMS = (u32) tmp16;
			pwm_write_cnt -= 2;
		}
	}
}

static void
pwm_timer_func(unsigned long unused)
{
	u32 period;

	if (pwm_write_cnt <= 0) {
		up(&pwm_wait);
		pwm_stop();
		return;
	}

	/* 96M/128/2/period */
	period = *gpWriteBufPtr16++;

	if (period > 0) {
		PWM_PWMP = 96000000 / 128 / 12 / 2 / period;	/*96000000/128/2*11 */
	}

	pwm_timer.expires = jiffies + pwm_timer_period;
	add_timer(&pwm_timer);
	pwm_write_cnt--;
}


/*
 * Function:
 *	pwm_write
 * Synopsis:
 *	write to the device. Please write the sequence of tones to play
 */
static ssize_t
pwm_write(struct file *filp, const char *buf, size_t count, loff_t * f_pos)
{
	if (pwm_suspended) return -EPERM;

	if (NULL == (gpWriteBuf = kmalloc(count, GFP_KERNEL))) {
		return -ENOMEM;
	}
	gpWriteBufPtr8 = gpWriteBuf;

	if (copy_from_user(gpWriteBuf, buf, count)) {
		kfree(gpWriteBuf);
		gpWriteBuf = NULL;
		return -EFAULT;
	}

	mx_module_clk_open(IPG_MODULE_PWM);

		if (pwm_mode == PWM_TONE_MODE) {
			/* create periodic timer when tone mode */
			init_timer(&pwm_timer);
			pwm_timer.function = pwm_timer_func;
		}

	/*write data to PWM */
	pwm_write_cnt = count;

	pwm_soft_reset();

	PWM_PWMC &= ~PWM_EN;	/* disable PWM */

	PWM_PWMC &= 0xFFFF00F0;	/* sysclk; */

	/* check sampling rate */
	if (pwm_sampling == PWM_SAMPLING_8KHZ) {
		/* 96M / 23/2/256 = 8.152kHz */
		PWM_PWMC |= (1 << 8);
	} else /*if (arg == PWM_SAMPLING_16KHZ)*/ {
		PWM_PWMC |= (0 << 8);
	}

	PWM_PWMC &= ~0x3;	/* clksel = 0 (/2) */

	if (pwm_data_len == PWM_DATA_8BIT) {
		PWM_PWMP = 0xfe;	/* Period, 8bit */
	} else {	/* if(arg == PWM_DATA_16BIT) */
		PWM_PWMP = 0xfffe;	/* Period, 16bit */
	}

	if (pwm_swap & PWM_SWAP_HCTRL) {	/*Halfword FIFO data swapping */
		PWM_PWMC |= PWM_SWAP_HCTRL;
	} else {
		PWM_PWMC &= ~PWM_SWAP_HCTRL;
	}

	if (pwm_swap & PWM_SWAP_BCTRL) {	/*Byte FIFO data swapping */
		PWM_PWMC |= PWM_SWAP_BCTRL;
	} else {
		PWM_PWMC &= ~PWM_SWAP_BCTRL;
	}

	PWM_PWMC &= ~0x00000C0;	/* set REPEAT to zero */
	if (PWM_TONE_MODE != pwm_mode) {	/* pwm play mode */
		PWM_PWMC |= 0xf00;	/* set prescaler */
		PWM_PWMS = (u32) (*gpWriteBufPtr8++);
		pwm_write_cnt--;
		PWM_PWMC |= PWM_IRQ_EN;	/* enable interrupts */

	} else {
		PWM_PWMS = pwm_sample_value;
		PWM_PWMC |= 0x7f00;	/* set prescaler to 128 */
		gpWriteBufPtr16 = (u16 *) gpWriteBuf;

		pwm_write_cnt >>= 1;	/*input size is 8bit size, need /2 to get 16bit size */

		pwm_timer.expires = jiffies + pwm_timer_period;
		add_timer(&pwm_timer);
	}

	PWM_PWMC |= PWM_EN;	/* enable the PWM */

	down_interruptible(&pwm_wait);

	pwm_stop();

	return 0 /* ESUCCES */ ;
}

static void
pwm_stop(void)
{

	if (PWM_TONE_MODE == pwm_mode) {
		del_timer(&pwm_timer);
	}

	PWM_PWMC &= ~PWM_IRQ_EN;
	PWM_PWMC &= ~PWM_EN;

	pwm_write_cnt = 0;
	pwm_data_len = 0;
	pwm_sample_value = 0;
	PWM_PWMS = pwm_sample_value;

	if (gpWriteBuf) {
		kfree(gpWriteBuf);
		gpWriteBuf = NULL;
	}

	mx_module_clk_close(IPG_MODULE_PWM);
}

static void pwm_suspend(void)
{
	if (PWM_TONE_MODE == pwm_mode) {
		del_timer(&pwm_timer);
	}
	mx_module_clk_close(IPG_MODULE_PWM);
	pwm_suspended = 1;
}

static void pwm_resume(void)
{
	if (gpWriteBuf!= NULL) /*if still playing*/ {
		mx_module_clk_open(IPG_MODULE_PWM);
		if (PWM_TONE_MODE == pwm_mode) {
			add_timer(&pwm_timer);
		}
	}
	pwm_suspended = 0;
}

static int
handle_pm_event(struct pm_dev *dev, pm_request_t rq, void *data)
{
	switch (rq) {
	case PM_SUSPEND:
			pwm_suspend();
		break;
	case PM_RESUME:
			pwm_resume();
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
			pwm_suspend();
		break;
	}
	return 0;
}

static int
__ldm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
			pwm_resume();
		break;
	}
	return 0;
}
#endif


static int __init
pwm_init(void)
{
	int result;

	if ((result = request_irq(INT_PWM,
				  pwm_interrupt,
				  SA_INTERRUPT, "pwm", pwmp)) < 0) {
		printk(KERN_ERR PKMOD "error registering irq\n");
		return result;
	}

	result = mx2_register_gpio(PORT_E, 5, PRIMARY | NOINTERRUPT);
	if (result < 0) {
		printk(KERN_ERR PKMOD "error registering GPIO port E pin 5 \n");
		free_irq(INT_PWM, pwmp);
		return result;
	}


	gMajor = devfs_register_chrdev(0, MOD_NAME, &ts_fops);
	if (gMajor < 0) {
		printk(KERN_ERR PKMOD "unable to register character device\n");
		free_irq(INT_PWM, pwmp);
		mx2_unregister_gpio(PORT_E, 5);
		return -ENODEV;
	}

	g_devfs_handle = devfs_register(NULL, MOD_NAME, DEVFS_FL_DEFAULT,
					gMajor, 0,
					S_IFCHR | S_IRUSR | S_IWUSR,
					&ts_fops, NULL);

	/* register power manager */
	pm_register(PM_SYS_DEV, PM_SYS_UNKNOWN, handle_pm_event);
#if 1 /*CEE LDM*/
	mx21_ldm_bus_register(&__device_ldm, &__driver_ldm);
#endif

	mx_module_clk_close(IPG_MODULE_PWM);

	return 0;
}

static void __exit
pwm_cleanup(void)
{

	pm_unregister_all(handle_pm_event);
#if 1 /*CEE LDM*/
	mx21_ldm_bus_unregister(&__device_ldm, &__driver_ldm);
#endif

	mx2_unregister_gpio(PORT_E, 5);

	free_irq(INT_PWM, pwmp);

	devfs_unregister_chrdev(gMajor, MOD_NAME);
	devfs_unregister(g_devfs_handle);
}

module_init(pwm_init);
module_exit(pwm_cleanup);
