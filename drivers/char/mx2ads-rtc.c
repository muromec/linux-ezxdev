/*	Real Time Clock interface for Linux	
 *
 *	Copyright (C) 1996 Paul Gortmaker
 *      Copyright (C) 2003 MontaVista Software Inc. <source@mvista.com>
 *
 *	This driver allows use of the real time clock (built into
 *	nearly all computers) from user space. It exports the /dev/rtc
 *	interface supporting various ioctl() and also the
 *	/proc/driver/rtc pseudo-file for status information.
 *
 *	The ioctls can be used to set the interrupt behaviour and
 *	generation rate from the RTC via IRQ 8. Then the /dev/rtc
 *	interface can be used to make use of these timer interrupts,
 *	be they interval or alarm based.
 *
 *	The /dev/rtc interface will block on reads until an interrupt
 *	has been received. If a RTC interrupt has already happened,
 *	it will output an unsigned long and then block. The output value
 *	contains the interrupt status in the low byte and the number of
 *	interrupts since the last read in the remaining high bytes. The
 *	/dev/rtc interface can also be used with the select(2) call.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	Based on other minimal char device drivers, like Alan's
 *	watchdog, Ted's random, etc. etc.
 *
 *      2003/04/03 - dmitry pervushin <pervushin@rtsoft.msk.ru>
 *                   some modifications to correct work in 2.4.20
 *                   environment on mx1ads boards
 */
#define RTC_VERSION	"0.4.1"
#define RTC_IO_EXTENT	0x10
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/rtc.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>

#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/arch/pll.h>

#define RTC_AIE			0x0004
#define RTC_PIE			0xff80
#define RTC_UIE			0x0010
#define RTC_SIE			0x0001

/* RTC Control Register bits definition */
#define RTCCTL_SWR 		(1 << 0)
#define RTCCTL_XTL_SHIFT	5
#define RTCCTL_XTL(x)		(((x) & 0x3) << RTCCTL_XTL_SHIFT)
#define XTL_OSC_32768HZ_INPUT 	0x0
#define XTL_OSC_32000HZ_INPUT 	0x1
#define XTL_OSC_38400HZ_INPUT 	0x2
#define RTCCTL_EN 		(1 << 7)

#define TRY_SLEEPON

static int rtc_debug;
#define rtc_printk( dbg, params... ) if( (dbg)<=rtc_debug) pr_debug( params )

MODULE_PARM(rtc_debug, "i");

struct rtc_time rtc_timer;
static struct fasync_struct *rtc_async_queue;

#ifdef TRY_SLEEPON
wait_queue_head_t rtc_wait;
#else
static DECLARE_WAIT_QUEUE_HEAD(rtc_wait);
#endif

extern spinlock_t rtc_lock;

static struct timer_list rtc_irq_timer;

static loff_t rtc_llseek(struct file *file, loff_t offset, int origin);

static ssize_t rtc_read(struct file *file, char *buf,
			size_t count, loff_t * ppos);

static int rtc_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg);

static unsigned int rtc_poll(struct file *file, poll_table * wait);

static void get_rtc_time(struct rtc_time *rtc_tm);
static void get_rtc_alm_time(struct rtc_time *alm_tm);
static void rtc_dropped_irq(unsigned long data);

static void set_rtc_irq_bit(unsigned int bit);
static void mask_rtc_irq_bit(unsigned int bit);

void rtc_update_xtime_from_rtc(void);
void rtc_update_rtc_from_xtime(void);

static int rtc_read_proc(char *page, char **start, off_t off,
			 int count, int *eof, void *data);

/*
 *	Bits in rtc_status. (6 bits of room for future expansion)
 */

#define RTC_IS_OPEN		0x01	/* means /dev/rtc is in use     */
#define RTC_TIMER_ON		0x02	/* missed irq timer active      */

/*
 * rtc_status is never changed by rtc_interrupt, and ioctl/open/close is
 * protected by the big kernel lock. However, ioctl can still disable the timer
 * in rtc_status and then with del_timer after the interrupt has read
 * rtc_status but before mod_timer is called, which would then reenable the
 * timer (but you would need to have an awful timing before you'd trip on it)
 */
static unsigned long rtc_status = 0;	/* bitmapped status byte.       */
static unsigned long rtc_freq = 0;	/* Current periodic IRQ rate    */
static unsigned long rtc_irq_data = 0;	/* our output to the world      */
static unsigned long before_wk = 0;	/* the seconds value before set the stopwatch */

/*
 *	If this driver ever becomes modularised, it will be really nice
 *	to make the epoch retain its value across module reload...
 */

static unsigned long epoch = 2003;	/* year corresponding to 0x00   */

static const unsigned char days_in_mo[] =
    { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

unsigned
DAY_FORMAT(int *days, int *mon, int *year)
{
	unsigned d = 0;
	int m, y;

	if (!mon) {
		mon = &m;
		m = 0;
	}

	if (!year) {
		year = &y;
		y = 0;
	}

	rtc_printk(2, "Date before format %d/%d/%d\n", *days, *mon, *year);
	while (*year > 0) {
		int ndays;

		ndays = (*year + epoch % 4 == 0) ? 366 : 365;
		d += ndays;
		--*year;
		rtc_printk(2, "...year(%d), d = %d\n", *year, d);
	}

	while (--*mon > 0) {
		d += days_in_mo[*mon];
		rtc_printk(2, "...mon %d, d = %d\n", *mon, d);
	}

	rtc_printk(2, "...day, d = %d\n", d);
	rtc_printk(2, "Date formatted %d/%d/%d\n", d + *days, *mon, *year);
	return d + *days;
}

void
DAY_UNFORMAT(int *days, int *mon, int *year)
{
	int ok = 0;
	int ndays = 365;

	rtc_printk(2, "Date before unformat %d/%d/%d\n", *days, *mon, *year);

	if (*days > ndays) {
		++*year;
		(*days) -= ndays;
	}

	while (!ok && *days > 0) {
		if (*days > days_in_mo[*mon]) {
			*days -= days_in_mo[*mon];
			++*mon;
		} else {
			ok = 1;
		}
		if (*mon > 12) {
			*mon -= 12;
			++*year;
		}
	}

	rtc_printk(2, "Unformatted date is %d/%d/%d\n", *days, *mon, *year);
}

/*
 *	A very tiny interrupt handler. It runs with SA_INTERRUPT set,
 *	but there is possibility of conflicting with the set_rtc_mmss()
 *	call (the rtc irq and the timer irq can easily run at the same
 *	time in two different CPUs). So we need to serializes
 *	accesses to the chip with the rtc_lock spinlock that each
 *	architecture should implement in the timer code.
 *	(See ./arch/XXXX/kernel/time.c for the set_rtc_mmss() function.)
 */

static void
rtc_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	/*
	 *      Can be an alarm interrupt, update complete interrupt,
	 *      or a periodic interrupt. We store the status in the
	 *      low byte and the number of interrupts received since
	 *      the last read in the remainder of rtc_irq_data.
	 */
	static int freq_2HZ = 0;
	unsigned long isr;

	freq_2HZ = 0;
	spin_lock(&rtc_lock);
	rtc_irq_data += 0x10000;	/* 0x100 */
	rtc_irq_data &= ~0xffff;
	rtc_irq_data |= (RTC_RTCISR & 0xffff);

	rtc_printk(3, "%s: rtc_irq_data 0x%lx\n", __FUNCTION__, rtc_irq_data);

	/* acknowledge the interrupt */
	isr = RTC_RTCISR;
	rtc_printk(3, "%s: before setting RTCISR is 0x%lx\n", __FUNCTION__,
		   isr);

	RTC_RTCISR = RTC_RTCISR;

	isr = RTC_RTCISR;
	rtc_printk(3, "%s: after setting RTCISR is 0x%lx\n", __FUNCTION__, isr);

	if (rtc_status & RTC_TIMER_ON) {
		mod_timer(&rtc_irq_timer,
			  jiffies + HZ / rtc_freq + 2 * HZ / 100);
	}

	spin_unlock(&rtc_lock);

	/* Now do the rest of the actions */
	wake_up_interruptible(&rtc_wait);

	kill_fasync(&rtc_async_queue, SIGIO, POLL_IN);
}

/*
 *	Now all the various file operations that we export.
 */
static loff_t
rtc_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}

static int
rtc_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	  unsigned long arg)
{
	struct rtc_time wtime;

	rtc_printk(2, "%s: code %x\n", __FUNCTION__, cmd);

	switch (cmd) {
	case RTC_WIE_OFF:	/* mask stop watch int. enab. bit */
		mask_rtc_irq_bit(RTC_SIE);
		return 0;

	case RTC_WIE_ON:
		set_rtc_irq_bit(RTC_SIE);
		return 0;

	case RTC_AIE_OFF:	/* Mask alarm int. enab. bit    */
		mask_rtc_irq_bit(RTC_AIE);
		return 0;

	case RTC_AIE_ON:	/* Allow alarm interrupts.      */
		set_rtc_irq_bit(RTC_AIE);
		return 0;

	case RTC_PIE_OFF:	/* Mask periodic int. enab. bit */
		/* 
		 * Periodic interrupt rates are only accurate for the
		 * 32.768 kHz XTAL case, please refer to the i.MX2 Reference 
		 * Manual for actual interrupt rates for other RTC 
		 * XTAL frequencies
		 */
		mask_rtc_irq_bit(rtc_freq == 1 ? rtc_freq << 4 : rtc_freq << 6);
		if (rtc_status & RTC_TIMER_ON) {
			spin_lock_irq(&rtc_lock);
			rtc_status &= ~RTC_TIMER_ON;
			del_timer(&rtc_irq_timer);
			spin_unlock_irq(&rtc_lock);
		}
		return 0;

	case RTC_PIE_ON:	/* Allow periodic int in RTCISR [bit 7~bit 15] */
		/*
		 * We don't really want Joe User enabling more
		 * than 64Hz of interrupts on a multi-user machine.
		 * you can see linux\include\linux\capablities.h
		 */
		if ((rtc_freq > 64) && (!capable(CAP_SYS_RESOURCE))) {
			return -EACCES;
		}

		if (!(rtc_status & RTC_TIMER_ON)) {
			spin_lock_irq(&rtc_lock);
			rtc_irq_timer.expires =
			    jiffies + HZ / rtc_freq + 2 * HZ / 100;
			add_timer(&rtc_irq_timer);
			rtc_status |= RTC_TIMER_ON;
			spin_unlock_irq(&rtc_lock);
		}

		/* 
		 * Periodic interrupt rates are only accurate for the
		 * 32.768 kHz XTAL case, please refer to the i.MX2 Reference Manual for actual
		 * interrupt rates for other RTC XTAL frequencies
		 * 
		 * rtc_freq = 1HZ RTCISR[bit :4]
		 *
		 * For 32.768 kHz case:
		 * rtc_freq = 2HZ RTCISR[bit :7]
		 * rtc_freq = 4HZ RTCISR[bit :8]
		 * ...
		 * rtc_freq = 512HZ RTCISR[bit :15]
		 */
		set_rtc_irq_bit((rtc_freq ==
				 1) ? (rtc_freq << 4) : (rtc_freq << 6));
		return 0;

	case RTC_UIE_OFF:	/* Mask ints from RTC updates.  */
		mask_rtc_irq_bit(RTC_UIE);
		return 0;

	case RTC_UIE_ON:	/* Allow ints for RTC updates.  */
		set_rtc_irq_bit(RTC_UIE);
		return 0;
	case RTC_WKALM_SET:
		if (arg > 0x3f) {
			return -EFAULT;
		}
		spin_lock_irq(&rtc_lock);
		before_wk = RTC_SECOND & 0x3f;
		RTC_STPWCH = arg;
		spin_unlock_irq(&rtc_lock);
		return 0;

	case RTC_WKALM_RD:
		return put_user(RTC_STPWCH, (unsigned long *) arg);

	case RTC_ALM_READ:	/* Read the present alarm time */
		/*
		 * This returns a struct rtc_time. Reading >= 0xc0
		 * means "don't care" or "match all". Only the tm_hour,
		 * tm_min, and tm_sec values are filled in.
		 */
		memset(&wtime, 0, sizeof (struct rtc_time));
		get_rtc_alm_time(&wtime);
		break;

	case RTC_ALM_SET:	/* Store a time into the alarm */
		{
			/*
			 * This expects a struct rtc_time. Writing 0xff means
			 * "don't care" or "match all". Only the tm_hour,
			 * tm_min and tm_sec are used.
			 */
			unsigned int day, hrs, min, sec, mon, yrs;
			struct rtc_time alm_tm;

			if (copy_from_user(&alm_tm, (struct rtc_time *) arg,
					   sizeof (struct rtc_time)))
				return -EFAULT;

			day = alm_tm.tm_mday;
			hrs = alm_tm.tm_hour;
			min = alm_tm.tm_min;
			sec = alm_tm.tm_sec;
			yrs = alm_tm.tm_year + 1900;
			mon = alm_tm.tm_mon + 1;	/* tm_mon starts at zero */

			if (hrs >= 24)
				hrs = 0xff;

			if (min >= 60)
				min = 0xff;

			if (sec >= 60)
				sec = 0xff;

			if ((yrs -= epoch) > 255)
				yrs = 0;

			spin_lock_irq(&rtc_lock);

			RTC_DAYALARM = DAY_FORMAT(&day, &mon, &yrs);
			RTC_ALRM_HM = hrs << 8 | min;
			RTC_ALRM_SEC = sec;

			spin_unlock_irq(&rtc_lock);

			return 0;
		}
	case RTC_RD_TIME:	/* Read the time/date from RTC  */
		memset(&wtime, 0, sizeof (struct rtc_time));
		get_rtc_time(&wtime);
		break;

	case RTC_SET_TIME:	/* Set the RTC */
		{
			struct rtc_time rtc_tm;
			int mon, day, hrs, min, sec, leap_yr, yrs;

			if (copy_from_user(&rtc_tm, (struct rtc_time *) arg,
					   sizeof (struct rtc_time)))
				return -EFAULT;

			yrs = rtc_tm.tm_year + 1900;
			mon = rtc_tm.tm_mon + 1;	/* tm_mon starts at zero */
			day = rtc_tm.tm_mday;
			hrs = rtc_tm.tm_hour;
			min = rtc_tm.tm_min;
			sec = rtc_tm.tm_sec;

			rtc_printk(2,
				   "Setting time to (D/M/Y hh.mm.ss) %d/%02d/%04d %d.%02d.%02d\n",
				   day, mon, yrs, hrs, min, sec);

			if (yrs < 1970) {
				rtc_printk(1, "Invalid year\n");
				return -EINVAL;
			}

			leap_yr = ((!(yrs % 4) && (yrs % 100)) || !(yrs % 400));

			if ((mon > 12) || (day == 0)) {
				rtc_printk(1, "Invalud month\n");
				return -EINVAL;
			}

			if (day > (days_in_mo[mon] + ((mon == 2) && leap_yr))) {
				rtc_printk(1, "Invalud day\n");
				return -EINVAL;
			}

			if ((hrs >= 24) || (min >= 60) || (sec >= 60)) {
				rtc_printk(1, "Invalud time\n");
				return -EINVAL;
			}

			if ((yrs -= epoch) > 255) {	/* They are unsigned */
				rtc_printk(1, "Invalud year\n");
				return -EINVAL;
			}

			spin_lock_irq(&rtc_lock);

			RTC_DAYR = DAY_FORMAT(&day, &mon, &yrs);;
			RTC_HOURMIN = hrs << 8 | min;
			RTC_SECOND = sec;
			spin_unlock_irq(&rtc_lock);

			rtc_timer.tm_year = yrs;
			rtc_timer.tm_mon = mon;

			rtc_printk(3, "rtc_timer structure: %d/%d\n",
				   rtc_timer.tm_mon, rtc_timer.tm_year);

			return 0;
		}
	case RTC_IRQP_READ:	/* Read the periodic IRQ rate.  */
		return put_user(rtc_freq, (unsigned long *) arg);

	case RTC_IRQP_SET:	/* Set periodic IRQ rate to rtc_freq */
		{
			int tmp = 0;
			/*
			 * The range we can do is 1,2,4,8.. 512Hz.
			 */
			if (((arg < 2) || (arg > 512)) && arg != 1)
				return -EINVAL;

			while (arg > (1 << tmp))	/* get the power of arg */
				tmp++;

			/*
			 * Check that the input was really a power of 2.
			 */
			if (arg != (1 << tmp))
				return -EINVAL;

			spin_lock_irq(&rtc_lock);
			rtc_freq = arg;
			spin_unlock_irq(&rtc_lock);
			return 0;
		}

	case RTC_EPOCH_READ:	/* Read the epoch.      */
		{
			return put_user(epoch, (unsigned long *) arg);
		}
	case RTC_EPOCH_SET:	/* Set the epoch.       */
		{
			/*
			 * There were no RTC clocks before 1900.
			 */
			if (arg < 1900)
				return -EINVAL;

			if (!capable(CAP_SYS_TIME))
				return -EACCES;

			epoch = arg;
			return 0;
		}

	default:
		return -EINVAL;
	}

	return copy_to_user((void *) arg, &wtime, sizeof wtime) ? -EFAULT : 0;
}

/*
 *	We enforce only one user at a time here with the open/close.
 *	Also clear the previous interrupt data on an open, and clean
 *	up things on a close.
 */
/* We use rtc_lock to protect against concurrent opens. So the BKL is not
 * needed here. Or anywhere else in this driver. */
static int
rtc_open(struct inode *inode, struct file *file)
{
	spin_lock_irq(&rtc_lock);

	rtc_printk(2, "%s: enter\n", __FUNCTION__);
	if (rtc_status & RTC_IS_OPEN)
		goto out_busy;

	rtc_status |= RTC_IS_OPEN;

	rtc_irq_data = 0;
	spin_unlock_irq(&rtc_lock);
	rtc_printk(2, "%s: exit, OK\n", __FUNCTION__);

	return 0;

      out_busy:
	rtc_printk(2, "%s: exit, busy\n", __FUNCTION__);
	spin_unlock_irq(&rtc_lock);
	return -EBUSY;
}

static ssize_t
rtc_read(struct file *file, char *buf, size_t count, loff_t * ppos)
{
#ifdef TRY_SLEEPON
	unsigned long data;
	unsigned int j;
	ssize_t retval;

	if (count < sizeof (unsigned long))
		return -EINVAL;

	/* First make it right. The make it fast. Putting this whole
	 * block within the parentheses of a while would be too
	 * confusing. And no, xchg() is not the answer. */
	spin_lock_irq(&rtc_lock);
	data = rtc_irq_data;
	rtc_irq_data = 0;
	spin_unlock_irq(&rtc_lock);

	if (file->f_flags & O_NONBLOCK) {
		if (!data)
			return -EAGAIN;
		retval = put_user(data, (unsigned long *) buf);
		return retval;
	} else {
		while (1) {	/* ensure copy to user or block */
			spin_lock_irq(&rtc_lock);
			data = rtc_irq_data;
			rtc_irq_data = 0;
			spin_unlock_irq(&rtc_lock);
			if (data) {
				j = jiffies + before_wk * HZ;
				rtc_printk(2,
					   "%s: before_wk = 0x%lx\n",
					   __FUNCTION__, before_wk);
				before_wk = 0;
				while (jiffies < j)
					schedule();
				return put_user(data, (unsigned long *) buf);
			} else {
				interruptible_sleep_on(&rtc_wait);
				if (signal_pending(current))
					return -ERESTARTSYS;
			}
		}
	}

#else
	DECLARE_WAITQUEUE(wait, current);
	unsigned long data;
	ssize_t retval;

	if (count < sizeof (unsigned long))
		return -EINVAL;
	add_wait_queue(&rtc_wait, &wait);

	current->state = TASK_INTERRUPTIBLE;

	do {
		/* First make it right. The make it fast. Putting this whole
		 * block within the parentheses of a while would be too
		 * confusing. And no, xchg() is not the answer. */
		spin_lock_irq(&rtc_lock);
		data = rtc_irq_data;
		rtc_irq_data = 0;
		spin_unlock_irq(&rtc_lock);

		if (data != 0)
			break;

		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			goto out;
		}
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			goto out;
		}
		schedule();
	} while (1);

	retval = put_user(data, (unsigned long *) buf);
	if (!retval)
		retval = sizeof (unsigned long);
      out:
	current->state = TASK_RUNNING;
	remove_wait_queue(&rtc_wait, &wait);

	return retval;
#endif
}

static int
rtc_fasync(int fd, struct file *filp, int on)
{
	return fasync_helper(fd, filp, on, &rtc_async_queue);
}

static int
rtc_release(struct inode *inode, struct file *file)
{

	/*
	 * Turn off all interrupts once the device is no longer
	 * in use, and clear the data.
	 */

	spin_lock_irq(&rtc_lock);
	if (rtc_status & RTC_TIMER_ON) {
		rtc_status &= ~RTC_TIMER_ON;
		del_timer(&rtc_irq_timer);
	}
	spin_unlock_irq(&rtc_lock);

	if (file->f_flags & FASYNC) {
		rtc_fasync(-1, file, 0);
	}

	spin_lock_irq(&rtc_lock);
	rtc_irq_data = 0;
	spin_unlock_irq(&rtc_lock);

	/* No need for locking -- nobody else can do anything until this rmw is
	 * committed, and no timer is running. */
	rtc_status &= ~RTC_IS_OPEN;

	return 0;
}

/* Called without the kernel lock - fine */
static unsigned int
rtc_poll(struct file *file, poll_table * wait)
{
	unsigned long l;

	poll_wait(file, &rtc_wait, wait);

	spin_lock_irq(&rtc_lock);
	l = rtc_irq_data;
	spin_unlock_irq(&rtc_lock);

	return l ? POLLIN | POLLRDNORM : 0;
}

/*
 *	The various file operations we support.
 */

static struct file_operations rtc_fops = {
	.owner = THIS_MODULE,
	.llseek = rtc_llseek,
	.read = rtc_read,
	.poll = rtc_poll,
	.ioctl = rtc_ioctl,
	.open = rtc_open,
	.release = rtc_release,
	.fasync = rtc_fasync,
};

static struct miscdevice rtc_dev = {
	RTC_MINOR,
	"rtc",
	&rtc_fops
};

static int __init
rtc_init(void)
{
	/* Reset the module to its default state */

	RTC_RTCCTL |= RTCCTL_SWR; 

	udelay(10);

#if defined(OSC_32768HZ_INPUT)
	RTC_RTCCTL = (RTCCTL_EN | RTCCTL_XTL(XTL_OSC_32768HZ_INPUT));
#elif defined(OSC_32000HZ_INPUT)
	RTC_RTCCTL = (RTCCTL_EN | RTCCTL_XTL(XTL_OSC_32000HZ_INPUT));
#else
#error Unsupported MX21ADS Input Crystal Frequency	
#endif
	
	rtc_timer.tm_mon = 0;
	rtc_timer.tm_year = 0;

	mx_module_clk_open(IPG_MODULE_RTC);

	if (request_irq(INT_RTC_RX,
			rtc_interrupt, SA_SHIRQ | SA_INTERRUPT, "rtc", "rtc")) {
		/* Yeah right, seeing as irq 8 doesn't even hit the bus. */
		printk(KERN_ERR "rtc: IRQ %d is not free.\n", INT_RTC_RX);
		return -EIO;
	}

	misc_register(&rtc_dev);
	create_proc_read_entry("driver/rtc", 0, 0, rtc_read_proc, NULL);

	init_timer(&rtc_irq_timer);
	rtc_irq_timer.function = rtc_dropped_irq;
	spin_lock_irq(&rtc_lock);
	spin_unlock_irq(&rtc_lock);
	rtc_freq = 512;

#ifdef TRY_SLEEPON
	init_waitqueue_head(&rtc_wait);
#endif

	printk(KERN_INFO "Real Time Clock Driver\n");
	rtc_printk(1, "Debug level is %d\n", rtc_debug);

	return 0;
}

static void __exit
rtc_exit(void)
{
	remove_proc_entry("driver/rtc", NULL);
	misc_deregister(&rtc_dev);
	free_irq(INT_RTC_RX, "rtc");
	RTC_RTCCTL &= ~RTCCTL_EN;
	mx_module_clk_close(IPG_MODULE_RTC);

}

module_init(rtc_init);
module_exit(rtc_exit);

static void
rtc_dropped_irq(unsigned long data)
{
	unsigned long freq;

	spin_lock_irq(&rtc_lock);

	/* Just in case someone disabled the timer from behind our back... */
	if (rtc_status & RTC_TIMER_ON)
		mod_timer(&rtc_irq_timer,
			  jiffies + HZ / rtc_freq + 2 * HZ / 100);

	rtc_irq_data += ((rtc_freq / HZ) << 16);
	rtc_irq_data &= ~0xffff;

	rtc_irq_data |= (RTC_RTCISR & 0xffff);

	freq = rtc_freq;

	spin_unlock_irq(&rtc_lock);

	/* Now we have new data */
	wake_up_interruptible(&rtc_wait);

	kill_fasync(&rtc_async_queue, SIGIO, POLL_IN);

}

/*
 *	Info exported via "/proc/driver/rtc".
 */

static int
rtc_proc_output(char *buf)
{
	char *p;
	struct rtc_time tm;
	unsigned long freq;

	spin_lock_irq(&rtc_lock);
	freq = rtc_freq;
	spin_unlock_irq(&rtc_lock);

	p = buf;

	get_rtc_time(&tm);

	/*
	 * There is no way to tell if the luser has the RTC set for local
	 * time or for Universal Standard Time (GMT). Probably local though.
	 */
	p += sprintf(p,
		     "rtc_time\t: %02d:%02d:%02d\n"
		     "rtc_date\t: %04d-%02d-%02d\n"
		     "rtc_epoch\t: %04lu\n",
		     tm.tm_hour, tm.tm_min, tm.tm_sec,
		     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, epoch);

	get_rtc_alm_time(&tm);

	/*
	 * We implicitly assume 24hr mode here. Alarm values >= 0xc0 will
	 * match any value for that particular field. Values that are
	 * greater than a valid time, but less than 0xc0 shouldn't appear.
	 */
	p += sprintf(p, "alarm\t\t: ");
	if (tm.tm_hour <= 24)
		p += sprintf(p, "%02d:", tm.tm_hour);
	else
		p += sprintf(p, "**:");

	if (tm.tm_min <= 59)
		p += sprintf(p, "%02d:", tm.tm_min);
	else
		p += sprintf(p, "**:");

	if (tm.tm_sec <= 59)
		p += sprintf(p, "%02d\n", tm.tm_sec);
	else
		p += sprintf(p, "**\n");
	return p - buf;
}

static int
rtc_read_proc(char *page, char **start, off_t off,
	      int count, int *eof, void *data)
{
	int len = rtc_proc_output(page);
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

static void
get_rtc_time(struct rtc_time *rtc_tm)
{
	/*
	 * Only the values that we read from the RTC are set. We leave
	 * tm_wday, tm_yday and tm_isdst untouched. Even though the
	 * RTC has RTC_DAY_OF_WEEK, we ignore it, as it is only updated
	 * by the RTC when initially set to a non-zero value.
	 */
	spin_lock_irq(&rtc_lock);

	rtc_printk(3, "rtc_timer is %d/%d\n", rtc_timer.tm_mon,
		   rtc_timer.tm_year);
	rtc_tm->tm_sec = RTC_SECOND;
	rtc_tm->tm_min = RTC_HOURMIN & 0x3f;
	rtc_tm->tm_hour = (RTC_HOURMIN >> 8) & 0x1f;
	rtc_tm->tm_mday = RTC_DAYR;
	rtc_tm->tm_mon = rtc_timer.tm_mon;
	rtc_tm->tm_year = rtc_timer.tm_year;

	rtc_printk(1,
		   "%s: time is (D/M/Y hh:mm:ss) %02d/%02d/%04d %02d:%02d:%02d\n",
		   __FUNCTION__, rtc_tm->tm_mday, rtc_tm->tm_mon,
		   rtc_tm->tm_year, rtc_tm->tm_hour, rtc_tm->tm_min,
		   rtc_tm->tm_sec);

	DAY_UNFORMAT(&rtc_tm->tm_mday, &rtc_tm->tm_mon, &rtc_tm->tm_year);

	spin_unlock_irq(&rtc_lock);
/*
 * Account for differences between how the RTC uses the values
 * and how they are defined in a struct rtc_time;
 */
	if ((rtc_tm->tm_year += (epoch - 1900)) <= 69)
		rtc_tm->tm_year += 100;

	rtc_tm->tm_mon--;

	rtc_printk(1,
		   "%s: time is (D/M/Y hh:mm:ss) %02d/%02d/%04d %02d:%02d:%02d\n",
		   __FUNCTION__, rtc_tm->tm_mday, rtc_tm->tm_mon,
		   rtc_tm->tm_year, rtc_tm->tm_hour, rtc_tm->tm_min,
		   rtc_tm->tm_sec);
}

void
mx2ads_update_rtc(struct timeval *t)
{
	printk("Updating RTC\n");
}

void
rtc_update_xtime_from_rtc()
{
	struct rtc_time tm;
	struct timeval tv;

	tv.tv_usec = 0;
	get_rtc_time(&tm);
	tv.tv_sec = mktime(tm.tm_year, tm.tm_mon, tm.tm_mday,
			   tm.tm_hour, tm.tm_min, tm.tm_sec);
	do_settimeofday(&tv);

	rtc_printk(2, "%s: xtime has been set to %ld.%ld\n", __FUNCTION__,
		   tv.tv_sec, tv.tv_usec);
}

static void
get_rtc_alm_time(struct rtc_time *alm_tm)
{

	/*
	 * Only the values that we read from the RTC are set. That
	 * means only tm_hour, tm_min, and tm_sec.
	 */

	spin_lock_irq(&rtc_lock);
	alm_tm->tm_sec = RTC_ALRM_SEC;
	alm_tm->tm_min = RTC_ALRM_HM & 0x3f;
	alm_tm->tm_hour = (RTC_ALRM_HM >> 8) & 0x1f;
	alm_tm->tm_mday = RTC_DAYALARM;

	spin_unlock_irq(&rtc_lock);

}

/*
 * Used to disable/enable interrupts for any one of UIE, AIE, PIE.
 * Rumour has it that if you frob the interrupt enable/disable
 * bits in RTC_CONTROL, you should read RTC_INTR_FLAGS, to
 * ensure you actually start getting interrupts. Probably for
 * compatibility with older/broken chipset RTC implementations.
 * We also clear out any old irq data after an ioctl() that
 * meddles with the interrupt enable/disable bits.
 */

static void
mask_rtc_irq_bit(unsigned int bit)
{
	spin_lock_irq(&rtc_lock);

	RTC_RTCIENR &= ~bit;
	rtc_irq_data = 0;

	spin_unlock_irq(&rtc_lock);
}

static void
set_rtc_irq_bit(unsigned int bit)
{

	spin_lock_irq(&rtc_lock);

	RTC_RTCIENR |= bit;

	rtc_irq_data = 0;
	spin_unlock_irq(&rtc_lock);
}

EXPORT_SYMBOL(rtc_update_xtime_from_rtc);
MODULE_LICENSE("GPL");

