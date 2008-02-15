/*
 * linux/drivers/i2c/i2c-omap1510.c
 *
 * TI OMAP1510 I2C unified algorith+adapter driver (inspired by i2c-ibm_iic.c)
 *
 * Copyright (C) 2003 MontaVista Software, Inc.
 *
 * ----------------------------------------------------------------------------
 * This file was highly leveraged from i2c-elektor.c, which was created
 * by Simon G. Vogl and Hans Berglund:
 *
 *
 * Copyright 1995-97 Simon G. Vogl
 *           1998-99 Hans Berglund
 *
 * With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and even
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
#include "i2c-omap1510.h"

/* ----- global defines ----------------------------------------------- */

#undef	I2C_OMAP1510_DEBUG
#if	defined(I2C_OMAP1510_DEBUG)
#define DEB0(fmt, args...)	\
		printk(KERN_DEBUG "%s:%d: " fmt , __FUNCTION__ , __LINE__ , \
		       ## args);
#define DEB1(fmt, args...)	\
	if (i2c_debug>=1) {	\
		printk(KERN_DEBUG "%s:%d: " fmt , __FUNCTION__ , __LINE__ , \
		       ## args);	\
	}
#define DEB2(fmt, args...)	\
	if (i2c_debug>=2) {	\
		printk(KERN_DEBUG "%s:%d: " fmt , __FUNCTION__ , __LINE__ , \
		       ## args);	\
	}
#define DEB3(fmt, args...)	\
	if (i2c_debug>=3) {	\
		printk(KERN_DEBUG "%s:%d: " fmt , __FUNCTION__ , __LINE__ , \
		       ## args);	\
	}
#define DEB9(fmt, args...)	\
	/* debug the protocol by showing transferred bits */	\
	if (i2c_debug>=9) {	\
		printk(KERN_DEBUG "%s:%d: " fmt , __FUNCTION__ , __LINE__ , \
		       ## args);	\
	}
#else
#define DEB0(fmt, args...)
#define DEB1(fmt, args...)
#define DEB2(fmt, args...)
#define DEB3(fmt, args...)
#define DEB9(fmt, args...)
#endif

#define err(format, arg...) printk(KERN_ERR format , ## arg)
#define info(format, arg...) printk(KERN_INFO format , ## arg)
#define warn(format, arg...) printk(KERN_WARNING format , ## arg)
#define emerg(format, arg...) printk(KERN_EMERG format , ## arg)

#define DEF_TIMEOUT	16
#define DEFAULT_CLOCK	(100000)	/* default 100KHz */
#define DEFAULT_OWN	1
#define MAX_MESSAGES	65536		/* max number of messages */

/* ----- global variables ---------------------------------------------	*/

static struct gpi {
	int own;
	int clock;
	int irq;
	omap1510_i2c_p base;
} gpi;

static wait_queue_head_t i2c_wait;
static volatile int i2c_pending;
static int i2c_nacks;
static int i2c_arbls;

/* ----- module parameters --------------------------------------------	*/

static int base;
static int clock;
static int irq = -1; /* Polled mode */
static int own;
static int i2c_debug;
static int i2c_scan;	/* have a look at what's hanging 'round */

/* ----- Forward declarations -----------------------------------------	*/

static void omap1510_i2c_release(void);
static void omap1510_init(struct i2c_adapter *);
static int omap1510_i2c_wait_for_bb(struct i2c_adapter *);
static void omap1510_i2c_sleep(unsigned long);
static int omap1510_i2c_wait_for_pin(struct i2c_adapter *, u16 *, u16 *);
static int omap1510_i2c_scan_bus(struct i2c_adapter *);
static int omap1510_i2c_xfer(struct i2c_adapter *, struct i2c_msg[], int);
static int omap1510_i2c_rxbytes(struct i2c_adapter *, struct i2c_msg *, int);
static unsigned int omap1510_i2c_get_pclk(void);

#ifdef CONFIG_OMAP_INNOVATOR  /* linux-pm */
#include <linux/device.h>

static int i2c_ldm_suspend(struct device *dev, u32 state, u32 level);
static int i2c_ldm_resume(struct device *dev, u32 level);

static struct device_driver i2c_driver_ldm = {
       name:      "i2c-adap-omap1510",
       probe:     NULL,
       suspend:   i2c_ldm_suspend,
       resume:    i2c_ldm_resume,
       remove:    NULL,
};

static struct device i2c_device_ldm = {
       name: "OMAP1510 I2C Controller",
       bus_id: "I2C",
       driver: NULL,
       power_state: DPM_POWER_ON,
};

static void i2c_ldm_driver_register(void)
{
   extern void mpu_public_driver_register(struct device_driver *driver);

   mpu_public_driver_register(&i2c_driver_ldm);
}

static void i2c_ldm_device_register(void)
{
   extern void mpu_public_device_register(struct device *device);

   mpu_public_device_register(&i2c_device_ldm);
}
static void i2c_ldm_driver_unregister(void)
{
   extern void mpu_public_driver_unregister(struct device_driver *driver);

   mpu_public_driver_unregister(&i2c_driver_ldm);
}

static void i2c_ldm_device_unregister(void)
{
   extern void mpu_public_device_unregister(struct device *device);

   mpu_public_device_unregister(&i2c_device_ldm);
}

static int i2c_ldm_suspend(struct device *dev, u32 state, u32 level)
{
  	extern struct i2c_adapter omap1510_i2c_adap;
 
	switch(level) { 
	    case SUSPEND_POWER_DOWN: {
		omap1510_i2c_wait_for_bb(&omap1510_i2c_adap);
		outw(0, I2C_IE);
		outw(0, I2C_CON);
		break;
	    }
	    default:
		break;
	}
	return 0;
}

static int i2c_ldm_resume(struct device *dev, u32 level)
{
  	extern struct i2c_adapter omap1510_i2c_adap;

	switch(level) {
	    case RESUME_POWER_ON: {
		omap1510_init(&omap1510_i2c_adap);
		break;
	    }
	    default: {
		break;
  	    }
	}
	return 0;  
}

#endif /* linux-pm */


/* ----- Utility functions --------------------------------------------	*/

static unsigned int
omap1510_i2c_get_pclk(void)
{
	return 12000000UL;
}

static void
omap1510_init(struct i2c_adapter *adap)
{
	if (inw(I2C_CON) & I2C_CON_EN) {
		outw(0, I2C_CON);
		mdelay(10);
	}

	/* Setup clock prescaler to obtain approx 12MHz I2C module clock: */
	outw(omap1510_i2c_get_pclk()/12000000 - 1, I2C_PSC);

	/* Take the I2C module out of reset: */
	outw(I2C_CON_EN, I2C_CON);

	/* Make sure I2C module test mode is disabled: */
	outw(0, I2C_SYSTEST);

	/* Enable some interrupts. Interesting note: the I2C module does
	 * not seem to work unless "some" interrupts are enabled. I
	 * haven't determined which interrupts must be enabled in
	 * order for the module to function. But early polled mode
	 * driver tests reveal that the I2C module only works when
	 * interrupts are enabled. Otherwise, status register bits
	 * do not return expected events.
	 */
	outw((I2C_IE_XRDY_IE | I2C_IE_RRDY_IE | I2C_IE_ARDY_IE |
	      I2C_IE_NACK_IE | I2C_IE_AL_IE), I2C_IE);

	/* Setup clock control for standard 100kHz mode: */
	outw(omap1510_i2c_get_pclk()/clock/2 - 6, I2C_SCLL);
	outw(omap1510_i2c_get_pclk()/clock/2 - 6, I2C_SCLH);

	/* Set Own Address: */
	outw(own, I2C_OA);

	/* As per the Errata: "The setting of I2C_CNT = 0000h results in a
	 * byte count of 0 bytes instead of bytes 65536 as shown in the
	 * current TRM. It should be stated that the setting of 0000h is
	 * for 0 bytes and should not be used." Doh, 0 means 0 but don't
	 * try to perform 0 byte transfers. Darn, I thought I could use
	 * this for bus probing. Anyway, let's set it to 0 initially as
	 * the bus seems to be getting stuck in a funky state which doesn't
	 * seem to clear after performing a reinit.
	 */
	outw(0, I2C_CNT);
}

/*
 * Waiting on Bus Busy
 */
static int
omap1510_i2c_wait_for_bb(struct i2c_adapter *adap)
{
	int timeout = DEF_TIMEOUT;
	u16 status;
	u16 iv;

	while (((status = inw(I2C_STAT)) & I2C_STAT_BB) && timeout--) {
		iv = inw(I2C_IV);
		if (iv || (status & ~I2C_STAT_BB)) {
			DEB3("status: %04x, iv: %d, con: %04x\n",
			       status, iv, inw(I2C_CON));
		}
		schedule_timeout(1);
	}

	if (timeout <= 0) {
		DEB3("Timeout, I2C bus is busy\n");
	}

	return (timeout <= 0);
}

/*
 * Put this process to sleep for a period equal to timeout 
 */
static inline void
omap1510_i2c_sleep(unsigned long timeout)
{
	schedule_timeout(timeout);
}

/*
 * After we issue a transaction on the OMAP1510 I2C bus, this function
 * is called. It puts this process to sleep until we get an interrupt from
 * the controller telling us that the transaction we requested is complete.
 * pin = Pending Interrupt Not
 */
static int
omap1510_i2c_wait_for_pin(struct i2c_adapter *adap, u16 * status, u16 * iv)
{
	int retries = 3;
	int timeout = DEF_TIMEOUT;
	uint flags;

	do {
		/* Put the process to sleep. This process will be awakened
		 * when either the the I2C peripheral interrupts or the
		 * timeout expires. 
		 */
		if (irq == INT_I2C) {
			/* If irq is setup properly, wait for interrupt.
			 */
			local_irq_save(flags);
			if (i2c_pending == 0) {
				interruptible_sleep_on_timeout(&i2c_wait, 1);
			}
			i2c_pending = 0;
			*status = inw(I2C_STAT);
			*iv = inw(I2C_IV);
			local_irq_restore(flags);
		} else {
			/* Otherwise, use polled mode driver.
			 */
			udelay(1000000/clock * 10);
			*status = inw(I2C_STAT);
			*iv = inw(I2C_IV);
		}

		DEB3("con: %04x, cnt: %u, stat: %04x, iv: %u\n",
		     inw(I2C_CON), inw(I2C_CNT), *status, *iv);
	} while (!(*iv) &&
		 !(*status & (I2C_STAT_ROVR | I2C_STAT_XUDF |
			     I2C_STAT_RRDY | I2C_STAT_ARDY | I2C_STAT_NACK |
			     I2C_STAT_AL)) &&
		 (inw(I2C_CON) & (I2C_CON_MST)) &&
		  retries--);

	if (retries <= 0) {
		DEB2("Timeout\n");
	}

	return (retries <= 0) ? -1 : 0;
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
omap1510_i2c_scan_bus(struct i2c_adapter *adap)
{
	int found = 0;
	int i;
	struct i2c_msg msg;
	char data[1];

	printk(KERN_INFO "%s: scanning for active I2C devices on the bus.",
	       adap->name);

	if (i2c_debug)
		printk("\n");

	for (i = 1; i < 0x7f; i++) {
		if (inw(I2C_OA) == i) {
			continue;
		}

		msg.addr = i;
		msg.buf = data;
		msg.len = 0;
		msg.flags = I2C_M_RD;

		if (omap1510_i2c_xfer(adap, &msg, 1) == 0) {
			if (i2c_debug)
				printk(KERN_DEBUG "I2C Found 0x%02x\n", i);
			else
				printk(" 0x%02x", i);

			found++;
		}
	}

	if (i2c_debug) {
		if (!found)
			printk(KERN_DEBUG "I2C Found Nothing\n");
	} else {
		if (found)
			printk("\n");
		else
			printk(" Found nothing.\n");
	}

	return found;
}

/*
 * Prepare controller for a transaction and call omap1510_i2c_rxbytes
 * to do the work.
 */
static int
omap1510_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	int i;
	int status;

	DEB1("msgs: %d\n", num);

	if ((num < 1) || (num > MAX_MESSAGES)) {
		return (status = -EINVAL);
	}

	/* Check for valid parameters in messages */
	for (i=0; i < num; i++) {
		if (&msgs[i] == NULL) {
			return (status = -EINVAL);
		}

		if (msgs[i].buf == NULL){
			return (status = -EINVAL);
		}
	}

	if ((status = omap1510_i2c_wait_for_bb(adap))) {
		warn("%s: Timeout waiting for busy bus.\n", adap->name);
		if (!(inw(I2C_CON) & I2C_CON_MST)) {
			/* We're not the master but the bus is busy. */
			return (status = -EBUSY);
		}
		/* This shouldn't happen but we're already the master.
		 * So try to park the bus before starting transaction.
		 */
		err("%s is already active. Attempting to Reset module...\n",
		    adap->name);
		outw(inw(I2C_CON) | I2C_CON_STP, I2C_CON);
		if ((status = omap1510_i2c_wait_for_bb(adap))) {
			/* Eew, still busy, try resetting to unwedge... */
			omap1510_init(adap->algo_data);
			if (omap1510_i2c_wait_for_bb(adap)) {
				err("%s reset failed - I2C bus is wedged.\n",
				    adap->name);
				/* Just give up... */
				return (status = -EREMOTEIO);
			}
		}
	}

	for (i = 0; i < num; i++) {
		DEB2("msg: %d, addr: 0x%04x, len: %d, flags: 0x%x\n",
		     i, msgs[i].addr, msgs[i].len, msgs[i].flags);

		status = omap1510_i2c_rxbytes(adap->algo_data, &msgs[i],
					      (i == (num - 1)));

		DEB2("status: %d\n", status);

		if (status != msgs[i].len)
			break;

	}

	/* REVISIT: This is silly but fixup status to return expected results
	 */
	if ((status >= 0) && (num > 1))
		status = num;

	DEB1("status: %d\n", status);

	return status;
}

/*
 * Low level master read/write transaction.
 */
static int
omap1510_i2c_rxbytes(struct i2c_adapter *adap, struct i2c_msg *msg, int stop)
{
	uint flags;
	u16 status;
	u16 iv;
	u16 data = 0;
	int count = 0;

	DEB2("addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n",
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

	local_irq_save(flags);
	i2c_pending = 0;
	outw((I2C_CON_EN | I2C_CON_MST | I2C_CON_STT |
	      ((msg->flags & I2C_M_TEN) ? I2C_CON_XA : 0) |
	      ((msg->flags & I2C_M_RD) ? 0 : I2C_CON_TRX) |
	      (stop ? I2C_CON_STP : 0)), I2C_CON);
	local_irq_restore(flags);

	do {
		if (omap1510_i2c_wait_for_pin(adap, &status, &iv)) {
			if ((inw(I2C_CON) & I2C_CON_MST));
				omap1510_init(adap);
			return -EREMOTEIO;
		}

		if (status & I2C_STAT_NACK) {
			/* According to the Errata, when a NACK occurs, the
			 * system has to make several actions to recover:
			 * 1. read the INTCODE in the I2C_IV register to
			 *    release NACK in I2C_STAT
			 * 2. write to the STP bit in the I2C_CON register
			 *    to release I2C data line
			 *
			 * Do not poll the NACK and AL bits in I2C_CON (I'm
			 * pretty sure the errata means I2C_STAT here : )
			 * because an update could be missed. These bits
			 * require an interrupt process to be handled
			 * correctly (additionally, the INTCODE field in
			 * the I2C_IV register should be read before any
			 * other action is taken in the subroutine).
			 *
			 * For now, I assume step 1 is satisfied since we're
			 * here already. But REVISIT this assumption.
			 *
			 * NOTE: I've observed that XRDY interrupts can and do
			 * occur here shortly after a NACK occurs. This seems
			 * to be an artifact of the OMAP1510 I2C SM logic. The
			 * device does go offline when setting STP as per the
			 * errata recommendations. Anyway, just FYI on the
			 * wierd XRDY ints post NACK and be careful.
			 */
			i2c_nacks += 1;
			outw(inw(I2C_CON) | I2C_CON_STP, I2C_CON);
			/* Grrr, in spite of the Errata recommendations, we
			 * don't get an interrupt after the above. So rather
			 * than calling omap1510_i2c_wait_for_pin here, just
			 * wait via udelay for a few bit clocks and then call
			 * omap1510_i2c_wait_for_bb to make sure we're done.
			 */
			udelay(1000000/clock * 2);
			if (omap1510_i2c_wait_for_bb(adap)) {
				omap1510_init(adap);
			}
			return -EREMOTEIO;
		} else if (status & I2C_STAT_AL) {
			i2c_arbls += 1;
			return -EREMOTEIO;
		} else if (status & I2C_STAT_ARDY) {
			if (stop && (inw(I2C_CON) & I2C_CON_MST)) {
				outw(inw(I2C_CON) | I2C_CON_STP, I2C_CON);
				if (omap1510_i2c_wait_for_bb(adap)) {
					omap1510_init(adap);
				}
			}
			break;
		} else if (status & I2C_STAT_RRDY) {
			u16 rstatus = status;
	      		if (!(msg->flags & I2C_M_RD)) {
				err("%s r/w hardware fault.\n", adap->name);
				omap1510_init(adap);
				return -EREMOTEIO;
			}
			local_irq_save(flags);
			while (rstatus & I2C_STAT_RRDY) {
				/* REVISIT: Fix for BE mode */
				data = inw(I2C_DATA);
				if (count < msg->len) {
					msg->buf[count++] = (u8) data;
					if (count < msg->len) {
						msg->buf[count++] =
							(u8) (data >> 8);
					}
				} else if (msg->len != 0) {
					/* Er, h/w and/or s/w error? */
					err("%s rcv count mismatch.\n",
						adap->name);
				}
				rstatus = inw(I2C_CON);
			}
			i2c_pending = 0;
			local_irq_restore(flags);
		} else if (status & I2C_STAT_XUDF) {
	      		if ((msg->flags & I2C_M_RD)) {
				err("%s r/w hardware fault.\n", adap->name);
				omap1510_init(adap);
				return -EREMOTEIO;
			}
			local_irq_save(flags);
			/* REVISIT: Fix for BE mode */
			if (count < msg->len) {
				data = msg->buf[count++];
				if (count < msg->len) {
					data |= msg->buf[count++] << 8;
				}
			} else if (msg->len != 0) {
				/* Er, h/w and/or s/w error? */
				err("%s xmt count mismatch.\n",
					adap->name);
			}
			outw(data, I2C_DATA);
			i2c_pending = 0;
			local_irq_restore(flags);
		} else {
			DEB9("con: %04x, stat: %04x, iv: %d, cnt: %d\n",
			     inw(I2C_CON), status, iv, inw(I2C_CNT));
		}
	} while (inw(I2C_STAT) || (inw(I2C_CON) & I2C_CON_MST));

	if (i2c_debug >= 9) {
		if ((stop && (inw(I2C_CON) & I2C_CON_MST)) ||
		    (inw(I2C_STAT) && ~(I2C_STAT_BB)))
			DEB9("con: %04x, stat: %04x, iv: %d, cnt: %d\n",
			     inw(I2C_CON), inw(I2C_STAT), inw(I2C_IV),
			     inw(I2C_CNT));
	}

	DEB2("status: %d\n", count);

	/* Fixup return status for the silly 0 length probe case */
	return (count > 0) ? (msg->len ? count : 0) : count;
}

static u32
omap1510_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/* -----exported algorithm data: -------------------------------------	*/

static struct i2c_algorithm omap1510_i2c_algo = {
	.name = "OMAP1510 I2C algorithm",
	.id = I2C_ALGO_EXP,
	.master_xfer = omap1510_i2c_xfer,
	.smbus_xfer = NULL,
	.slave_send = NULL,
	.slave_recv = NULL,
	.algo_control = NULL,
	.functionality = omap1510_i2c_func,
};

/* 
 * registering functions to load algorithms at runtime 
 */
int
omap1510_i2c_add_bus(struct i2c_adapter *adap)
{
	short status;

	DEB2("hw routines for %s registered.\n", adap->name);

	/* register new adapter to i2c module... */

	adap->id |= omap1510_i2c_algo.id;
	adap->algo = &omap1510_i2c_algo;
	adap->timeout = 100;
	adap->retries = 3;
	adap->flags = 0;

	i2c_add_adapter(adap);

	omap1510_init(adap);

	if (i2c_scan) {
		if (omap1510_i2c_scan_bus(adap) < 0)
			return -ENODEV;
	}

	return 0;
}

int
omap1510_i2c_del_bus(struct i2c_adapter *adap)
{
	int res;

	if ((res = i2c_del_adapter(adap)) < 0)
		return res;

	DEB2("adapter unregistered: %s\n", adap->name);

	return 0;
}

/* ----- local functions ----------------------------------------------	*/

/*
 */
static void
omap1510_i2c_handler(int this_irq, void *dev_id, struct pt_regs *regs)
{

	i2c_pending = 1;

	DEB3("in interrupt handler\n");
	wake_up_interruptible(&i2c_wait);
}

/* Lock the region of memory where I/O registers exist.  Request our
 * interrupt line and register its associated handler.
 */
static int
i2c_hw_resrc_init(void)
{
	if (check_region(base, I2C_IOSIZE) < 0)
		return -ENODEV;

	request_region(base, I2C_IOSIZE, "OMAP1510 I2C");

	if (irq == INT_I2C)
		if (request_irq(irq, omap1510_i2c_handler,
				0, "OMAP1510 I2C", 0) < 0)
			return -ENODEV;

	if (*(volatile u32 *) (COMP_MODE_CTRL_0) == (u32) (0x0000eaef)) {
		/* This selects OMAP1510 I2C module standard mode
		 * when in OMAP1510 native mode:
		 */
		*(volatile u32 *) (MOD_CONF_CTRL_0) &=
		    ~(1 << CONF_MOD_I2C_SELECT_R);
	} else {
		/* This selects OMAP1510 I2C module standard mode
		 * when in OMAP1509 compatibility mode:
		 */
		*(volatile u32 *) (FUNC_MUX_CTRL_1) |= (1 << I2C_MODE_CTRL);
	}

	return 0;
}

static void
omap1510_i2c_release(void)
{
	outw(0, I2C_CON);
	if (irq == INT_I2C)
		free_irq(irq, 0);
	release_region(base, I2C_IOSIZE);
}

static int
omap1510_i2c_register(struct i2c_client *client)
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
omap1510_i2c_unregister(struct i2c_client *client)
{
	MOD_DEC_USE_COUNT;
	return 0;
}

static void
omap1510_i2c_inc_use(struct i2c_adapter *adap)
{
	MOD_INC_USE_COUNT;
}

static void
omap1510_i2c_dec_use(struct i2c_adapter *adap)
{
	MOD_DEC_USE_COUNT;
}

/* ------------------------------------------------------------------------
 * Encapsulate the above functions in the correct operations structure.
 * This is only done when more than one hardware adapter is supported.
 */
static struct i2c_algo_omap1510_data omap1510_i2c_data = {
	.data = NULL,
	.waitforpin = omap1510_i2c_wait_for_pin,
	.udelay = 80,
	.mdelay = 80,
	.timeout = 100,
};

static struct i2c_adapter omap1510_i2c_adap = {
	.name = "OMAP1510 I2C adapter",
	.id = I2C_ALGO_EXP,	/* REVISIT: register for id */
	.algo = &omap1510_i2c_algo,
	.algo_data = &omap1510_i2c_data,
	.inc_use = omap1510_i2c_inc_use,
	.dec_use = omap1510_i2c_dec_use,
	.client_register = omap1510_i2c_register,
	.client_unregister = omap1510_i2c_unregister,
};

/* Called when the module is loaded.  This function starts the
 * cascade of calls up through the heirarchy of i2c modules (i.e. up to the
 *  algorithm layer and into to the core layer)
 */
static int __init
omap1510_i2c_init(void)
{
	printk(KERN_INFO "Initializing %s.\n", omap1510_i2c_adap.name);

	base = I2C_BASE;

	if (irq == 0)
		irq = INT_I2C;

	/* Check lower and upper I2C SCL bounds based upon
	 * 12MHz I2C module clock source.
	 */
	if ((clock < 25000) || (clock > 400000))
		clock = DEFAULT_CLOCK;

	if ((own < 1) || (own > 0x7f))
		own = DEFAULT_OWN;

	omap1510_i2c_data.data = (void *) &gpi;

	init_waitqueue_head(&i2c_wait);

	if (i2c_hw_resrc_init() < 0)
		return -ENODEV;

	if (omap1510_i2c_add_bus(&omap1510_i2c_adap) < 0)
		return -ENODEV;

	printk(KERN_INFO "Found %s at %#x ", omap1510_i2c_adap.name, base);

	if (irq == INT_I2C)
		printk("using interrupt mode on irq %d\n", irq);
	else
		printk("using polled I/O mode.\n");


#ifdef CONFIG_OMAP_INNOVATOR /* linux-pm */
        i2c_ldm_device_register();
        i2c_ldm_driver_register();
#endif                      /* linux-pm */        
	return 0;
}

static void __exit
omap1510_i2c_exit(void)
{
#ifdef CONFIG_OMAP_INNOVATOR /* MVL-CEE */
        i2c_ldm_device_unregister();
        i2c_ldm_driver_unregister();
#endif                      /* MVL-CEE */ 
	omap1510_i2c_del_bus(&omap1510_i2c_adap);
	omap1510_i2c_release();
}

MODULE_AUTHOR("MontaVista Software, Inc.");
MODULE_DESCRIPTION("TI OMAP1510 I2C bus adapter");
MODULE_LICENSE("GPL");
MODULE_PARM(clock, "i");
#if	0
MODULE_PARM(irq, "i");
#endif
MODULE_PARM(own, "i");
MODULE_PARM(i2c_debug, "i");
MODULE_PARM(i2c_scan, "i");
MODULE_PARM_DESC(i2c_scan, "Scan for active I2C clients on the bus");
MODULE_PARM_DESC(i2c_debug,
		 "debug level - 0 off; 1 normal; 2,3 more verbose; "
		 "9 omap1510-protocol");

module_init(omap1510_i2c_init);
module_exit(omap1510_i2c_exit);
