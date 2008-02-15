/*
 * rv5c387.c
 *
 * Device driver for the RICOH Co., Ltd., RV5C387 I2C Real Time Clock.
 *
 * Copyright (C) 2003 MontaVista Software, Inc.
 * Author George G. Davis <gdavis@mvista.com> or <source@mvista.com>
 *
 * Initially based on linux-2.4.19-rmk7/drivers/char/ds1307.c
 * Copyright (C) 2002 Intrinsyc Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/sysctl.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/rtc.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/irq.h>


#undef	DEBUG
#ifdef	DEBUG
#define DPRINTK(fmt, args...) \
	printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif


#define RV5C387_I2C_SLAVE_ADDR	0x32

#define PROC_RV5C387_NAME	"driver/rv5c387"

#define RV5C387_GETDATETIME	0
#define RV5C387_SETTIME		1
#define RV5C387_SETDATETIME	2
#define RV5C387_GETCTRL		3
#define RV5C387_SETCTRL		4

#if	defined(BCD_TO_BIN)
#undef	BCD_TO_BIN
#endif
#if	defined(BIN_TO_BCD)
#undef	BIN_TO_BCD
#endif
#define BCD_TO_BIN(val) (((val)&15) + ((val)>>4)*10)
#define BIN_TO_BCD(val) ((((val)/10)<<4) + (val)%10)

#define CTRL1_WALE	(1<<7)
#define CTRL1_DALE	(1<<6)
#define CTRL1_1224	(1<<5)
#define CTRL1_CLEN2	(1<<4)
#define CTRL1_TEST	(1<<3)

#define CTRL2_VDSL	(1<<7)
#define CTRL2_VDET	(1<<6)
#define CTRL2_SCRATCH	(1<<5)
#define CTRL2_XSTP	(1<<4)
#define CTRL2_CLEN1	(1<<3)
#define CTRL2_CTFG	(1<<2)
#define CTRL2_WAFG	(1<<1)
#define CTRL2_DAFG	(1<<0)

#define TWELVE_HOUR_MODE(n)	(!((n) & (CTRL1_1224 << 8)))
#define HOURS_PM(n)		(((n) >> 5) & 1)
#define HOURS_12(n)		BCD_TO_BIN((n) & 0x1F)
#define HOURS_24(n)		BCD_TO_BIN((n) & 0x3F)


typedef struct rtc_time_regs {
		u8 seconds;
		u8 minutes;
		u8 hours;
		u8 weekday;
		u8 day;
		u8 month;
		u8 year;
	} __attribute__((packed)) rtc_time_t;

typedef struct rtc_walarm_regs {
		u8 minutes;
		u8 hours;
		u8 weekday;
	} __attribute__((packed)) rtc_walarm_t;

typedef struct rtc_dalarm_regs {
		u8 minutes;
		u8 hours;
	} __attribute__((packed)) rtc_dalarm_t;

typedef struct rtc_registers {
		rtc_time_t	time;
		u8		oar;
		rtc_walarm_t	walarm;
		rtc_dalarm_t	dalarm;
		u8		rsrvd;
		u8		cntrl1;
		u8		cntrl2;
	} __attribute__((packed)) rtc_registers_t;


struct i2c_driver rv5c387_driver;
struct i2c_client *rv5c387_i2c_client = 0;

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] =
    { RV5C387_I2C_SLAVE_ADDR, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c		= normal_addr,
	.normal_i2c_range	= ignore,
	.probe			= ignore,
	.probe_range		= ignore,
	.ignore			= ignore,
	.ignore_range		= ignore,
	.force			= ignore,
};

static int rv5c387_rtc_ioctl(struct inode *, struct file *, unsigned int,
			     unsigned long);
static int rv5c387_rtc_open(struct inode *inode, struct file *file);
static int rv5c387_rtc_release(struct inode *inode, struct file *file);

static struct file_operations rtc_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= rv5c387_rtc_ioctl,
	.open		= rv5c387_rtc_open,
	.release	= rv5c387_rtc_release,
};

static struct miscdevice rv5c387_rtc_miscdev = {
	.minor	= RTC_MINOR,
	.name	= "rv5c387-rtc",
	.fops	= &rtc_fops
};

static int rv5c387_probe(struct i2c_adapter *adap);
static int rv5c387_detach(struct i2c_client *client);
static int rv5c387_command(struct i2c_client *client, unsigned int cmd,
			   void *arg);

struct i2c_driver rv5c387_driver = {
	.name		= "RV5C387",
	.id		= I2C_DRIVERID_RV5C387,
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= rv5c387_probe,
	.detach_client	= rv5c387_detach,
	.command	= rv5c387_command
};

static spinlock_t rv5c387_rtc_lock = SPIN_LOCK_UNLOCKED;

#define DAT(x) ((unsigned int)((x)->data)) /* copy of control register info */

/*
 *      A very tiny interrupt handler. It runs with SA_INTERRUPT set.
 */

static void rtc_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	DPRINTK("%s: irq\n", rv5c387_rtc_miscdev.name);
#if	0
        /*
         *      Can be an alarm interrupt, update complete interrupt,
         *      or a periodic interrupt. We store the status in the
         *      low byte and the number of interrupts received since
         *      the last read in the remainder of rtc_irq_data.
         */

        spin_lock (&rtc_lock);
        rtc_irq_data += 0x100;
        rtc_irq_data &= ~0xff;
        rtc_irq_data |= CMOS_READ(OMAP_RTC_STATUS_REG);

        if (rtc_irq_data & (1<<6))
                CMOS_WRITE((1<<6), OMAP_RTC_STATUS_REG);

        spin_unlock (&rtc_lock);

        /* Now do the rest of the actions */
        wake_up_interruptible(&rtc_wait);

        kill_fasync (&rtc_async_queue, SIGIO, POLL_IN);
#endif
}

static int
rv5c387_read(u8 *buf, int offset, int len)
{
	int ret;
	u32 flags;
	u8 ad[1] = { (offset & 0xf) << 4 };
	struct i2c_msg msgs[2] = {
		{rv5c387_i2c_client->addr, I2C_M_WR, 1, ad},
		{rv5c387_i2c_client->addr, I2C_M_RD, len, buf}
	};

	spin_lock_irqsave(&rv5c387_rtc_lock, flags);
	ret = i2c_transfer(rv5c387_i2c_client->adapter, msgs, 2);
	spin_unlock_irqrestore(&rv5c387_rtc_lock, flags);

	return ret == 2 ? len : ret;
}

static int
rv5c387_write(u8 *ibuf, int offset, int len)
{
	int ret;
	u32 flags;
	u8 buf[sizeof(rtc_registers_t) + 1];
	struct i2c_msg msgs[1] = {
		{rv5c387_i2c_client->addr, I2C_M_WR, len + 1, buf},
	};

 	buf[0] = (offset & 0xf) << 4;
	memcpy(&buf[1], ibuf, len);

	spin_lock_irqsave(&rv5c387_rtc_lock, flags);
	ret = i2c_transfer(rv5c387_i2c_client->adapter, msgs, 1);
	spin_unlock_irqrestore(&rv5c387_rtc_lock, flags);

	return ret == len + 1 ? len : ret;
}

#if	defined(DEBUG)
static void
rv5c387_dump(void)
{
	u8 buf[sizeof(rtc_registers_t)];
	int ret;

	ret = rv5c387_read(buf, 0, sizeof(rtc_registers_t));

	if (ret == sizeof(rtc_registers_t)) {
		int i;
		for (i = 0; i < sizeof(rtc_registers_t); i++) {
			printk("%02X ", buf[i]);
			if ((i % 8) == 7)
				printk("\n");
		}
		printk("\n");
	} else {
		printk("%s: rv5c387_read() returned %d.\n",
		       rv5c387_rtc_miscdev.name, ret);
	}
}
#endif

static void
rv5c387_enable_clock(int enable)
{
	u8 buf[2];
	int ret;
	u16 ctrl_info = DAT(rv5c387_i2c_client);

	if (enable) {
		ctrl_info &= ~(0x2008); /* Enable clock */
	} else {
		ctrl_info |= (0x2008); /* Disable clock */
		ctrl_info &= ~(0xc007); /* Mask interrupts */
	}

	ret = rv5c387_command(rv5c387_i2c_client, RV5C387_SETCTRL, &ctrl_info);
}

static int
rv5c387_attach(struct i2c_adapter *adap, int addr, unsigned short flags,
	       int kind)
{
	struct i2c_client *c;
	u8 buf[1], ad[1] = { (14 << 4) | 2 };
	struct i2c_msg msgs[2] = {
		{addr, I2C_M_WR, 1, ad},
		{addr, I2C_M_RD, 2, buf}
	};
	int ret;

	c = (struct i2c_client *) kmalloc(sizeof (*c), GFP_KERNEL);

	if (!c)
		return -ENOMEM;

	strcpy(c->name, "RV5C387");
	c->id = rv5c387_driver.id;
	c->flags = 0;
	c->addr = addr;
	c->adapter = adap;
	c->driver = &rv5c387_driver;
	c->data = NULL;

	spin_lock_irqsave(&rv5c387_rtc_lock, flags);
	ret = i2c_transfer(c->adapter, msgs, 2);
	spin_unlock_irqrestore(&rv5c387_rtc_lock, flags);

	if (ret != 2) {
		printk(KERN_ERR "%s: i2c_transfer() returned %d.\n",
		       rv5c387_rtc_miscdev.name, ret);
		kfree(c);
		return(ret);
	}

	/* Just override what we read and setup default operating mode */
	buf[0] = 0x20; /* no interrupts, 24hr mode, clock enabled; no test */
	buf[1] = 0x00;

	DAT(c) = (buf[0] << 8) | buf[1];

	DPRINTK("ctrl1/2: %04x\n", DAT(c));

	rv5c387_i2c_client = c;

	rv5c387_enable_clock(1);

	return i2c_attach_client(c);
}

static int
rv5c387_probe(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, rv5c387_attach);
}

static int
rv5c387_detach(struct i2c_client *client)
{
	rv5c387_enable_clock(0);
	i2c_detach_client(client);

	return 0;
}

static void
rv5c387_convert_to_time(struct rtc_time *dt, rtc_time_t *regs)
{
	DPRINTK("regs.year = %02x\n", regs->year);
	DPRINTK("regs.mon  = %02x\n", regs->month);
	DPRINTK("regs.mday = %02x\n", regs->day);
	DPRINTK("regs.hour = %02x\n", regs->hours);
	DPRINTK("regs.min  = %02x\n", regs->minutes);
	DPRINTK("regs.sec  = %02x\n", regs->seconds);

	dt->tm_sec = BCD_TO_BIN(regs->seconds);
	dt->tm_min = BCD_TO_BIN(regs->minutes);

	if (TWELVE_HOUR_MODE(DAT(rv5c387_i2c_client))) {
		dt->tm_hour = HOURS_12(regs->hours) % 12;
		if (HOURS_PM(regs->hours)) {	/* PM */
			dt->tm_hour += 12;
		}
	} else {		/* 24-hour-mode */
		dt->tm_hour = HOURS_24(regs->hours);
	}

	dt->tm_mday = BCD_TO_BIN(regs->day);
	/* dt->tm_mon is zero-based */
	dt->tm_mon = BCD_TO_BIN(regs->month & 0x1f) - 1;
	/* year is 1900 + dt->tm_year */
	dt->tm_year = BCD_TO_BIN(regs->year);

	if (regs->month & 0x80) {
		dt->tm_year += 100;
	}

	DPRINTK("tm_year = %d\n", dt->tm_year);
	DPRINTK("tm_mon  = %d\n", dt->tm_mon);
	DPRINTK("tm_mday = %d\n", dt->tm_mday);
	DPRINTK("tm_hour = %d\n", dt->tm_hour);
	DPRINTK("tm_min  = %d\n", dt->tm_min);
	DPRINTK("tm_sec  = %d\n", dt->tm_sec);
}

static int
rv5c387_get_datetime(struct i2c_client *client, struct rtc_time *dt)
{
	rtc_time_t regs;
	int ret = -EIO;

	memset(&regs, 0, sizeof (regs));

	ret = rv5c387_read((u8 *)&regs, 0, sizeof(regs));

	if (ret == sizeof(regs)) {
		rv5c387_convert_to_time(dt, &regs);
		ret = 0;
	} else {
		printk(KERN_ERR "%s: rv5c387_read() returned %d\n",
		       rv5c387_rtc_miscdev.name, ret);
	}

	return ret;
}

static int
rv5c387_set_datetime(struct i2c_client *client, struct rtc_time *dt,
		     int datetoo)
{
	int ret;
	int len = 3;
	rtc_time_t regs;

	memset(&regs, 0, sizeof(rtc_time_t));

	DPRINTK("tm_year = %d\n", dt->tm_year);
	DPRINTK("tm_mon  = %d\n", dt->tm_mon);
	DPRINTK("tm_mday = %d\n", dt->tm_mday);
	DPRINTK("tm_hour = %d\n", dt->tm_hour);
	DPRINTK("tm_min  = %d\n", dt->tm_min);
	DPRINTK("tm_sec  = %d\n", dt->tm_sec);

	regs.seconds = BIN_TO_BCD(dt->tm_sec);
	regs.minutes = BIN_TO_BCD(dt->tm_min);

	if (TWELVE_HOUR_MODE(DAT(rv5c387_i2c_client))) {
		regs.hours = BIN_TO_BCD(dt->tm_hour % 12);
		if (regs.hours == 0) {
			regs.hours = 12;
		}
		if (dt->tm_hour > 11) {
			regs.hours |= 0x20;
		}
	} else {
		regs.hours = BIN_TO_BCD(dt->tm_hour);
	}

	if (datetoo) {
		len = sizeof(rtc_time_t);

		/* we don't use day-of-week. */
		regs.day = BIN_TO_BCD(dt->tm_mday);
		regs.month = BIN_TO_BCD(dt->tm_mon + 1);
		regs.year = BIN_TO_BCD(dt->tm_year % 100);

		if (dt->tm_year / 100) {
			regs.month |= 0x80;
		}

		DPRINTK("regs.year = %02x\n", regs.year);
		DPRINTK("regs.mon  = %02x\n", regs.month);
		DPRINTK("regs.mday = %02x\n", regs.day);
	}

	DPRINTK("regs.hour = %02x\n", regs.hours);
	DPRINTK("regs.min  = %02x\n", regs.minutes);
	DPRINTK("regs.sec  = %02x\n", regs.seconds);

	ret = rv5c387_write((u8*)&regs, 0, len);

	if (ret == len) {
		ret = 0;
	} else {
		printk(KERN_ERR "%s: i2c_master_send() returned %d\n",
		       rv5c387_rtc_miscdev.name, ret);
	}

	return ret;
}

static int
rv5c387_get_ctrl(struct i2c_client *client, u16 *ctrl)
{
	*ctrl = DAT(client);

	return 0;
}

static int
rv5c387_set_ctrl(struct i2c_client *client, u16 *cinfo)
{
	u8 buf[2];
	int ret;

	buf[0] = (*cinfo) >> 8;
	buf[1] = (*cinfo) & 0xff;

	/* save the control reg info in the client data field so that get_ctrl
	 * function doesn't have to do an I2C transfer to get it.
	 */
	DAT(client) = *cinfo;

	ret = rv5c387_write(buf, 14, 2);

	return ret;
}

static int
get_rtc_alm_time(struct rtc_time *alm_tm)
{
	rtc_dalarm_t regs;
	int ret;

	ret = rv5c387_read((u8 *)&regs, offsetof(rtc_registers_t, dalarm),
			   sizeof(regs));

	if (ret == sizeof(regs)) {
		alm_tm->tm_min = BCD_TO_BIN(regs.minutes);
		alm_tm->tm_hour = BCD_TO_BIN(regs.hours);
	}

	return ret;
}

static int
set_rtc_alm_time(struct rtc_time *alm_tm)
{
	rtc_dalarm_t regs;

	regs.minutes = BIN_TO_BCD(alm_tm->tm_min);
	regs.hours = BIN_TO_BCD(alm_tm->tm_hour);

	return rv5c387_write((u8 *)&regs, offsetof(rtc_registers_t, dalarm),
			     sizeof(regs));
}

static int
rv5c387_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case RV5C387_GETDATETIME:
		return rv5c387_get_datetime(client, arg);

	case RV5C387_SETTIME:
		return rv5c387_set_datetime(client, arg, 0);

	case RV5C387_SETDATETIME:
		return rv5c387_set_datetime(client, arg, 1);

	case RV5C387_GETCTRL:
		return rv5c387_get_ctrl(client, arg);

	case RV5C387_SETCTRL:
		return rv5c387_set_ctrl(client, arg);

	default:
		return -EINVAL;
	}
}

static int
rv5c387_rtc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int
rv5c387_rtc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int
rv5c387_rtc_ioctl(struct inode *inode, struct file *file,
		  unsigned int cmd, unsigned long arg)
{
	u32 flags;
	struct rtc_time wtime;
	int status = 0;

	switch (cmd) {
	default:
	case RTC_PIE_ON:
	case RTC_PIE_OFF:
	case RTC_IRQP_READ:
	case RTC_IRQP_SET:
	case RTC_EPOCH_READ:
	case RTC_EPOCH_SET:
	case RTC_WKALM_SET:
	case RTC_WKALM_RD:
		return -EINVAL;
		break;

	case RTC_UIE_ON:
		return -EINVAL;
		break;

	case RTC_UIE_OFF:
		return -EINVAL;
		break;

	case RTC_AIE_ON:
		return -EINVAL;
		break;

	case RTC_AIE_OFF:
		return -EINVAL;
		break;

	case RTC_RD_TIME:
		memset(&wtime, 0, sizeof(wtime));
		rv5c387_command(rv5c387_i2c_client, RV5C387_GETDATETIME,
				&wtime);
		break;

	case RTC_SET_TIME:
		if (!capable(CAP_SYS_TIME)) {
			status = -EACCES;
			break;
		}

		if (copy_from_user(&wtime, (struct rtc_time *)arg,
				   sizeof (struct rtc_time))) {
			status = -EFAULT;
			break;
		}

		return rv5c387_command(rv5c387_i2c_client,
				       RV5C387_SETDATETIME, &wtime);
		break;

	case RTC_ALM_READ:	/* Read the present alarm time */
		/*
		 * This returns struct rtc_time. Only tm_hour,
		 * tm_min, and tm_sec values are filled in.
		 */
		memset(&wtime, 0, sizeof(wtime));
		get_rtc_alm_time(&wtime);
		break; 

	case RTC_ALM_SET:	/* Store a time into the alarm */
		/*
		 * This expects a struct rtc_time. But we only
		 * use tm_hour, tm_min and tm_sec.
		 */

		if (copy_from_user(&wtime, (struct rtc_time*)arg,
				   sizeof(struct rtc_time)))
			return -EFAULT;
		return set_rtc_alm_time(&wtime);
	}

	return copy_to_user((void *)arg, &wtime, sizeof(wtime)) ? -EFAULT : 0;
}

static char *
rv5c387_mon2str(unsigned int mon)
{
	char *mon2str[12] = {
		"Jan", "Feb", "Mar", "Apr", "May", "Jun",
		"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
	};
	if (mon > 11)
		return "error";
	else
		return mon2str[mon];
}

static int
rv5c387_rtc_proc_output(char *buf)
{
#define CHECK(ctrl,bit) ((ctrl & bit) ? "yes" : "no")
	rtc_registers_t regs;
	int ret;

	char *p = buf;

	ret = rv5c387_read((u8 *)&regs, 0, sizeof(rtc_registers_t));

	if (ret == sizeof(rtc_registers_t)) {
		int i;
		struct rtc_time dt;
		char text[9];

		p += sprintf(p, "RV5C387 I2C Real Time Clock)\n");

		rv5c387_convert_to_time(&dt, &regs.time);

		p += sprintf(p,
			     "Date/Time	     : %02d-%s-%04d %02d:%02d:%02d\n",
			     dt.tm_mday, rv5c387_mon2str(dt.tm_mon),
			     dt.tm_year + 1900, dt.tm_hour, dt.tm_min,
			     dt.tm_sec);

#if	0
		p += sprintf(p, "Clock halted	     : %s\n",
			     CHECK(ram[0], 0x80));
		p += sprintf(p, "24h mode	     : %s\n",
			     CHECK(ram[2], 0x40));
		p += sprintf(p, "Square wave enabled : %s\n",
			     CHECK(ram[7], 0x10));
		p += sprintf(p, "Freq		     : ");

		switch (ram[7] & 0x03) {
		case RATE_1HZ:
			p += sprintf(p, "1Hz\n");
			break;
		case RATE_4096HZ:
			p += sprintf(p, "4.096kHz\n");
			break;
		case RATE_8192HZ:
			p += sprintf(p, "8.192kHz\n");
			break;
		case RATE_32768HZ:
		default:
			p += sprintf(p, "32.768kHz\n");
			break;

		}
#endif
		p += sprintf(p, "\n");
	} else {
		p += sprintf(p, "Failed to read RTC!\n");
	}

	return p - buf;
}

static int
rv5c387_rtc_read_proc(char *page, char **start, off_t off,
		      int count, int *eof, void *data)
{
	int len = rv5c387_rtc_proc_output(page);

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

static __init int
rv5c387_init(void)
{
	int retval = 0;

	retval = i2c_add_driver(&rv5c387_driver);

	if (retval < 0) {
		return retval;
	}

	misc_register(&rv5c387_rtc_miscdev);
	create_proc_read_entry(PROC_RV5C387_NAME, 0, 0,
			       rv5c387_rtc_read_proc, NULL);
	printk(KERN_INFO "%s: RV5C387 RTC driver successfully loaded\n",
	       rv5c387_rtc_miscdev.name);

#if	0
	/* REVISIT: Innovator docs may be misleading. According to
	 * RV5C387A data sheet, INTB is the Alarm_W, INTC is the
	 * Alarm_D and INTA is the peridic interrupt, respectively.
	 */
	if (request_irq(INT_FPGA_RTC_A, rtc_interrupt, SA_INTERRUPT,
			"rtc", NULL)) {
		printk(KERN_ERR "rtc: Periodic IRQ%d is not free.\n",
		       INT_FPGA_RTC_A);
	}

	/* REVISIT: Enabling INT_FPGA_RTC_B results in continuous
	 * stream of interrupts.
	 */
	if (request_irq(INT_FPGA_RTC_B, rtc_interrupt, SA_INTERRUPT,
			"rtc", NULL)) {
		printk(KERN_ERR "rtc: Periodic IRQ%d is not free.\n",
		       INT_FPGA_RTC_B);
	}
#endif

	return retval;
}

static __exit void
rv5c387_exit(void)
{
#if	0
	free_irq (INT_FPGA_RTC_A, NULL);
	free_irq (INT_FPGA_RTC_B, NULL);
#endif

	remove_proc_entry(PROC_RV5C387_NAME, NULL);
	misc_deregister(&rv5c387_rtc_miscdev);
	i2c_del_driver(&rv5c387_driver);
}

module_init(rv5c387_init);
module_exit(rv5c387_exit);

MODULE_AUTHOR("George G. Davis");
MODULE_LICENSE("GPL");
