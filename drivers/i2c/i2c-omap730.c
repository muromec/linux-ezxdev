/*
 * linux/drivers/i2c/i2c-omap730.c
 *
 * TI OMAP730 I2C unified algorith+adapter driver.
 * Inspired by i2c-ibm_iic.c and i2c-omap1510.c.
 *
 * Copyright (C) 2003, 2004 MontaVista Software, Inc.
 *
 * ----------------------------------------------------------------------------
 * This file was highly leveraged from i2c-elektor.c, which was created
 * by Simon G. Vogl and Hans Berglund:
 *
 *
 * Copyright 1995-97 Simon G. Vogl
 *           1998-99 Hans Berglund
 *
 * With some changes from Kysti M�kki <kmalkki@cc.hut.fi> and even
 * Frodo Looijaard <frodol@dds.nl>
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 Modifications:
 ver. 1.1: Nov 2003, MontaVista Software
 - added DPM support
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <asm/arch/hardware.h>

#undef CONFIG_CEE
#define CONFIG_CEE

/* ----- global defines ----------------------------------------------- */
#define MODULE_NAME "OMAP730 I2C"
#define OMAP730_I2C_TIMEOUT HZ*5	/* timeout waiting for an I2C transaction */

#undef	I2C_OMAP730_DEBUG
#if	defined(I2C_OMAP730_DEBUG)
static int i2c_debug;
#define DEB0(format, arg...)	printk(KERN_DEBUG MODULE_NAME " DEBUG: " format "\n",  ## arg )
#define DEB1(format, arg...)	\
	if (i2c_debug>=1) {	\
		printk(KERN_DEBUG MODULE_NAME " DEBUG: " format "\n",  ## arg ); \
	}
#define DEB2(format, arg...)	\
	if (i2c_debug>=2) {	\
		printk(KERN_DEBUG MODULE_NAME " DEBUG: " format "\n",  ## arg ); \
	}
#define DEB3(format, arg...)	\
	if (i2c_debug>=3) {	\
		printk(KERN_DEBUG MODULE_NAME " DEBUG: " format "\n",  ## arg ); \
	}
#define DEB9(format, arg...)	\
	/* debug the protocol by showing transferred bits */	\
	if (i2c_debug>=9) {	\
		printk(KERN_DEBUG MODULE_NAME " DEBUG: " format "\n",  ## arg ); \
	}
#else
#define DEB0(fmt, args...)
#define DEB1(fmt, args...)
#define DEB2(fmt, args...)
#define DEB3(fmt, args...)
#define DEB9(fmt, args...)
#endif

#define err(format, arg...) printk(KERN_ERR MODULE_NAME " ERROR: " format "\n",  ## arg )
#define info(format, arg...) printk(KERN_INFO MODULE_NAME " INFO: " format "\n",  ## arg )
#define warn(format, arg...) printk(KERN_WARNING MODULE_NAME " WARNING: " format "\n",  ## arg )
#define emerg(format, arg...) printk(KERN_EMERG MODULE_NAME " EMERGENCY: " format "\n",  ## arg )

#define DEFAULT_OWN	1	/*default own I2C address */
#define MAX_MESSAGES	65536	/* max number of messages */

/* ----- global variables ---------------------------------------------	*/

static DECLARE_WAIT_QUEUE_HEAD(i2c_wait);

/* ----- module parameters --------------------------------------------	*/

/* Fast Mode: 400KHz, Standard Mode: 100KHz */
/*
 * Communication with the OV9640 camera is not reliable at
 * 100 KHz (probably because the I2c rise/fall times on the H2
 * board do not meet the SCCB maximum spec of 300 nsec). So we
 * have to run I2c at 50 KHz instead.
 *
 * Addendum: well, looks like we can get around the camera I2c problems
 * by introducing 100 usec delays between I2c transactions. With those
 * delays, camera comm at 100 KHz seems reliable, so the clock is now
 * bumped back up to 100 KHz.
 */
static int clock = 100; /* KHz */
static int own;
static int i2c_scan;		/* have a look at what's hanging 'round */

/* ----- Forward declarations -----------------------------------------	*/

void __exit omap_i2c_exit(void);
int __init omap_i2c_init(void);

static void omap_i2c_dec_use(struct i2c_adapter *adap);
static void omap_i2c_inc_use(struct i2c_adapter *adap);
static int omap_i2c_unregister(struct i2c_client *client);
static int omap_i2c_register(struct i2c_client *client);
static void omap_i2c_isr(int this_irq, void *dev_id, struct pt_regs *regs);
static u32 omap_i2c_func(struct i2c_adapter *adap);
static int omap_i2c_rxbytes(struct i2c_adapter *adap, struct i2c_msg *msg, int stop);
static int omap_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num);
static int omap_i2c_scan_bus(struct i2c_adapter *adap);
static int  omap_i2c_wait_for_pin(u16 * status);

#define ALLOW_SLEEP 1
#define NO_SLEEP 0

static int omap_i2c_wait_for_bb(char allow_sleep);
static int omap_i2c_reset(char allow_sleep);

static struct i2c_algorithm omap_i2c_algo = {
	.name = "OMAP730 I2C algorithm",
	.id = I2C_ALGO_EXP,
	.master_xfer = omap_i2c_xfer,
	.smbus_xfer = NULL,
	.slave_send = NULL,
	.slave_recv = NULL,
	.algo_control = NULL,
	.functionality = omap_i2c_func,
};
static struct i2c_adapter omap_i2c_adap = {
	.name = "OMAP730 I2C adapter",
	.id = I2C_ALGO_EXP,	/* REVISIT: register for id */
	.algo = &omap_i2c_algo,
	.algo_data = NULL,
	.inc_use = omap_i2c_inc_use,
	.dec_use = omap_i2c_dec_use,
	.client_register = omap_i2c_register,
	.client_unregister = omap_i2c_unregister,
};

#ifdef CONFIG_CEE	/* MVL-CEE */
#include <linux/device.h>

static int i2c_ldm_suspend(struct device *dev, u32 state, u32 level);
static int i2c_ldm_resume(struct device *dev, u32 level);

static struct device_driver i2c_driver_ldm = {
      name:"i2c-adap-omap730",
      probe:NULL,
      suspend:i2c_ldm_suspend,
      resume:i2c_ldm_resume,
      remove:NULL,
};

static struct device i2c_device_ldm = {
      name:"OMAP730 I2C Controller",
      bus_id:"I2C",
      driver:NULL,
      power_state:DPM_POWER_ON,
};

static void
i2c_ldm_driver_register(void)
{
	extern void mpu_public_driver_register(struct device_driver *driver);

	mpu_public_driver_register(&i2c_driver_ldm);
}

static void
i2c_ldm_device_register(void)
{
	extern void mpu_public_device_register(struct device *device);

	mpu_public_device_register(&i2c_device_ldm);
}
static void
i2c_ldm_driver_unregister(void)
{
	extern void mpu_public_driver_unregister(struct device_driver *driver);

	mpu_public_driver_unregister(&i2c_driver_ldm);
}

static void
i2c_ldm_device_unregister(void)
{
	extern void mpu_public_device_unregister(struct device *device);

	mpu_public_device_unregister(&i2c_device_ldm);
}

static int
i2c_ldm_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		DEB9("suspend");
		omap_i2c_wait_for_bb(NO_SLEEP);
		outw(0, I2C_CON);
		break;
	default:
		break;
	}
	return 0;
}

static int
i2c_ldm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		omap_i2c_reset(NO_SLEEP);
		DEB9("resume");
		break;
	default:
		break;
	}
	return 0;
}

#endif		/* MVL-CEE */

/* ----- Utility functions --------------------------------------------	*/

#define OMAP730_PSC 0

static int
omap_i2c_reset(char allow_sleep)
{
	unsigned long timeout;
	unsigned int iclk = 12000 / (OMAP730_PSC + 1);
	unsigned int n;
	u16 scl_low_hi;
	
	n = ((OMAP730_PSC == 0) ? 7 : ((OMAP730_PSC == 1) ? 6 : 5));
	
	outw(I2C_SYSC_SRST, I2C_SYSC);	/*soft reset */

	scl_low_hi = (iclk / (clock*2)) - n;
		
	/* Setup clock prescaler and SCL low/high times */
	outw(OMAP730_PSC, I2C_PSC);
	outw(scl_low_hi, I2C_SCLL);
	outw(scl_low_hi, I2C_SCLH);

	/* Set Own Address: */
	outw(own, I2C_OA);

	/* Take the I2C module out of reset: */
	outw(I2C_CON_EN, I2C_CON);
	timeout = jiffies + OMAP730_I2C_TIMEOUT;
	while (!(inw(I2C_SYSS) & I2C_SYSS_RDONE)) {
		if (time_after(jiffies, timeout)) {
			err("timeout waiting for I2C reset complete");
			return -EFAULT;
		}
		if (allow_sleep) schedule_timeout(1);
	}
	return 0;
}

/*
 * Waiting on Bus Busy
 */
static int
omap_i2c_wait_for_bb(char allow_sleep)
{
	unsigned long timeout;

	timeout = jiffies + HZ * 5;
	while (inw(I2C_STAT) & I2C_STAT_BB) {
		if (time_after(jiffies, timeout)) {
			DEB0("timeout waiting for bus ready");
			return -ETIMEDOUT;
		}
		if (allow_sleep) schedule_timeout(1);
	}

	return (0);
}

/*
 * After we issue a transaction on the OMAP730 I2C bus, this function
 * is called. It puts this process to sleep until we get an interrupt from
 * the controller telling us that the transaction we requested is complete.
 * pin = Pending Interrupt Not
 */
static int
omap_i2c_wait_for_pin(u16 * status)
{
	wait_queue_t __wait;

	/* Wait for interrupt. */
	init_waitqueue_entry(&__wait, current);
	add_wait_queue(&i2c_wait, &__wait);
	set_current_state(TASK_UNINTERRUPTIBLE);

	/* Enable interrupt. */
	outw((I2C_IE_XRDY_IE |
	      I2C_IE_RRDY_IE | I2C_IE_ARDY_IE | I2C_IE_NACK_IE |
	      I2C_IE_AL_IE), I2C_IE);

	schedule_timeout(OMAP730_I2C_TIMEOUT);
	current->state = TASK_RUNNING;
	remove_wait_queue(&i2c_wait, &__wait);

	if (!(inw(I2C_STAT) & (I2C_STAT_XRDY |
			       I2C_STAT_RRDY | I2C_STAT_ARDY |
			       I2C_STAT_NACK | I2C_STAT_AL))) {
		err("timeout waiting for event");
		return -ETIMEDOUT;
	}
	*status = inw(I2C_STAT);
	return 0;
}

/*
 * Sanity check for the adapter hardware - check the reaction of
 * the bus lines only if it seems to be idle.
 *
 * Scan the I2C bus for valid 7 bit addresses
 * (ie things that ACK on 1byte read)
 * if i2c_debug is off we print everything on one line.
 * if i2c_debug is on we do a newline per print so we don't
 * clash too much with printf's in the other functions.
 * TODO: check for 10-bit mode and never run as a slave.
 */
static int
omap_i2c_scan_bus(struct i2c_adapter *adap)
{
	int found = 0;
	int i;
	struct i2c_msg msg;
	char data[1];

	info("scanning for active I2C devices on the bus...");

	for (i = 1; i < 0x7f; i++) {
		if (inw(I2C_OA) == i) {
			continue;
		}

		msg.addr = i;
		msg.buf = data;
		msg.len = 0;
		msg.flags = I2C_M_RD;

		if (omap_i2c_xfer(adap, &msg, 1) == 0) {

			info("I2C device 0x%02x found", i);
			found++;
		}
	}

	if (!found)
		info("found nothing");

	return found;
}

/*
 * Prepare controller for a transaction and call omap_i2c_rxbytes
 * to do the work.
 */
static int
omap_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	int i;
	int status = 0;

	DEB1("msgs: %d", num);

	if ((num < 1) || (num > MAX_MESSAGES)) {
		return (status = -EINVAL);
	}

	/* Check for valid parameters in messages */
	for (i = 0; i < num; i++) {
		if (&msgs[i] == NULL) {
			return (status = -EINVAL);
		}

		if (msgs[i].buf == NULL) {
			return (status = -EINVAL);
		}
	}

	if ((status = omap_i2c_wait_for_bb(ALLOW_SLEEP)) < 0)
		return status;

	for (i = 0; i < num; i++) {
		DEB2("msg: %d, addr: 0x%04x, len: %d, flags: 0x%x",
		     i, msgs[i].addr, msgs[i].len, msgs[i].flags);

		status = omap_i2c_rxbytes(adap, &msgs[i], (i == (num - 1)));

		DEB2("status: %d", status);

		if (status != msgs[i].len)
			break;

	}

	if ((status >= 0) && (num > 1))
		status = num;

	DEB1("status: %d", status);

	return status;
}

/*
 * Low level master read/write transaction.
 */
static int
omap_i2c_rxbytes(struct i2c_adapter *adap, struct i2c_msg *msg, int stop)
{
	u16 data = 0;
	int count = 0;
	int tmp;
	u16 status;

	DEB2("addr: 0x%04x, len: %d, flags: 0x%x, stop: %d",
	     msg->addr, msg->len, msg->flags, stop);

	outw(msg->addr, I2C_SA);

	/* Sigh, seems we can't do zero length transactions. Thus, we
	 * can't probe for devices w/o actually sending/receiving at least
	 * a single byte. So we'll set count to 1 for the zero length
	 * transaction case and hope we don't cause grief for some
	 * arbitrary device due to random byte write/read during
	 * probes.
	 */
	outw((msg->len ? msg->len : 1), I2C_CNT);

	outw((I2C_CON_EN | I2C_CON_MST | I2C_CON_STT |
	      ((msg->flags & I2C_M_TEN) ? I2C_CON_XA : 0) |
	      ((msg->flags & I2C_M_RD) ? 0 : I2C_CON_TRX) |
	      (stop ? I2C_CON_STP : 0)), I2C_CON);

	for (;;) {
		if ((tmp = omap_i2c_wait_for_pin(&status)) < 0)
			return tmp;

		if (status & I2C_STAT_ARDY) {
			outw(I2C_STAT_ARDY, I2C_STAT);
			break;
		}
		if (status & I2C_STAT_NACK) {
			DEB0("NACK rcvd while sending addr %x data %x count %d, status %x", msg->addr, data, count, status);
			omap_i2c_reset(ALLOW_SLEEP);
			return -EREMOTEIO;
		}
		if (status & I2C_STAT_RRDY) {
			if (!(msg->flags & I2C_M_RD)) {
				DEB0("r/w hardware fault");
				omap_i2c_reset(ALLOW_SLEEP);
				return -EREMOTEIO;
			}

			/* REVISIT: Fix for BE mode */
			data = inw(I2C_DATA);
			if (count < msg->len) {
				msg->buf[count++] = (u8) data;
				if (count < msg->len) {
					msg->buf[count++] = (u8) (data >> 8);
				}
			}
			outw(I2C_STAT_RRDY, I2C_STAT);
			continue;
		}
		if (status & I2C_STAT_XRDY) {
			if ((msg->flags & I2C_M_RD)) {
				DEB0("r/w hardware fault");
				omap_i2c_reset(ALLOW_SLEEP);
				return -EREMOTEIO;
			}

			/* REVISIT: Fix for BE mode */
			if (count < msg->len) {
				data = msg->buf[count++];
				if (count < msg->len) {
					data |= msg->buf[count++] << 8;
				}
			}
			outw(data, I2C_DATA);
			outw(I2C_STAT_XRDY, I2C_STAT);
			continue;
		}
		if (status & I2C_STAT_AL) {
			DEB0("arbitration lost");
			omap_i2c_reset(ALLOW_SLEEP);
			return -EREMOTEIO;
		}
		if (status & I2C_STAT_ROVR) {
			DEB0("receive overrun");
			omap_i2c_reset(ALLOW_SLEEP);
			return -EREMOTEIO;
		}
		if (status & I2C_STAT_XUDF) {
			DEB0("transmit undeflow");
			omap_i2c_reset(ALLOW_SLEEP);
			return -EREMOTEIO;
		}

	}			/* for */

	DEB2("status: %d", count);
	if (count != msg->len) {
		DEB0("not all data sent/recvd");
		return -EIO;
	}

	return msg->len;
}

static u32
omap_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static void
omap_i2c_isr(int this_irq, void *dev_id, struct pt_regs *regs)
{
	DEB3("in interrupt handler");
	outw(0, I2C_IE);
	wake_up(&i2c_wait);
}

static int
omap_i2c_register(struct i2c_client *client)
{
	/* I2C client module ref counting seems rather broken.
	 * When a client is registered (attached) to an adapter, 
	 * you shouldn't be able to unload that adapter until
	 * after the client driver is unloaded. However, i2c-core
	 * doesn't appear to provide facilities to insure that
	 * an adapter is unloadable while a client driver is
	 * bound to it. In other words, it is possible to rmmod
	 * an adapter and then access a client driver which is
	 * still bound to it resulting in an oops. So this hack
	 * prevents unloading this adapter until all clients have
	 * been unregistered.
	 */
	MOD_INC_USE_COUNT;
	return 0;
}

static int
omap_i2c_unregister(struct i2c_client *client)
{
	MOD_DEC_USE_COUNT;
	return 0;
}

static void
omap_i2c_inc_use(struct i2c_adapter *adap)
{
	MOD_INC_USE_COUNT;
}

static void
omap_i2c_dec_use(struct i2c_adapter *adap)
{
	MOD_DEC_USE_COUNT;
}

/*initialize I2C*/
int __init
omap_i2c_init(void)
{
	int tmp;

	info("Driver ver. 1.1");
	DEB0("%s %s", __TIME__, __DATE__);

	if (clock > 400)
		clock = 400;	/*Fast mode */
	if (clock < 12)
		clock = 12;

	if ((own < 1) || (own > 0x7f))
		own = DEFAULT_OWN;

	tmp = (int) request_region(OMAP730_I2C_BASE, OMAP730_I2C_SIZE, MODULE_NAME);
	if (!tmp) {
		err("I2C is already in use");
		return -ENODEV;
	}

	/* add the I2C adapter/algorithm driver to the linux kernel */
	tmp = i2c_add_adapter(&omap_i2c_adap);
	if (tmp) {
		err("failed to add adapter");
		release_region(OMAP730_I2C_BASE, OMAP730_I2C_SIZE);
		return tmp;
	}

	tmp = request_irq(INT_I2C, omap_i2c_isr, 0, MODULE_NAME, MODULE_NAME);
	if (tmp) {
		err("failed to request i2c irq, using polled mode");
		i2c_del_adapter(&omap_i2c_adap);
		release_region(OMAP730_I2C_BASE, OMAP730_I2C_SIZE);
		return tmp;
	}


	omap_i2c_reset(ALLOW_SLEEP);

	if (i2c_scan)
		omap_i2c_scan_bus(&omap_i2c_adap);
#ifdef CONFIG_CEE	/* MVL-CEE */
	i2c_ldm_device_register();
	i2c_ldm_driver_register();
#endif			/* MVL-CEE */
	return 0;
}

void __exit
omap_i2c_exit(void)
{
#ifdef CONFIG_CEE	/* MVL-CEE */
	i2c_ldm_device_unregister();
	i2c_ldm_driver_unregister();
#endif			/* MVL-CEE */
	outw(0, I2C_CON);
	free_irq(INT_I2C, MODULE_NAME);
	i2c_del_adapter(&omap_i2c_adap);
	release_region(OMAP730_I2C_BASE, OMAP730_I2C_SIZE);
}

MODULE_AUTHOR("MontaVista Software, Inc.");
MODULE_DESCRIPTION("TI OMAP730 I2C bus adapter");
MODULE_LICENSE("GPL");

MODULE_PARM(clock, "i");
MODULE_PARM_DESC(clock,
		 "Set I2C clock in KHz: 100 (Standard Mode) or 400 (Fast Mode)");

MODULE_PARM(own, "i");

MODULE_PARM(i2c_scan, "i");
MODULE_PARM_DESC(i2c_scan, "Scan for active I2C clients on the bus");

#if	defined(I2C_OMAP730_DEBUG)
MODULE_PARM(i2c_debug, "i");
MODULE_PARM_DESC(i2c_debug,
		 "debug level - 0 off; 1 normal; 2,3 more verbose; "
		 "9 omap730-protocol");
#endif

module_init(omap_i2c_init);
module_exit(omap_i2c_exit);
