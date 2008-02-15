/******************************************************************************

	i2c-dbmx2.c
	driver for on-chip I2C of Motorola DragonBall MX2
	supports i2c-dev interface

	Author: MontaVista Software, Inc. <source@mvista.com>
	Copyright (c) 2003-2004 MontaVista Software, Inc.

	Based on: i2c-aa-dbmx.c from Motorola Dragonball MX2 ADS BSP
	Copyright (C) 2002 Motorola Inc

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

*******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/fs.h>

#include <asm/irq.h>
#include <asm/arch/gpio.h>
#include <asm/io.h>
#include <asm/delay.h>

#include <linux/i2c.h>
#include <linux/i2c-id.h>

#include <asm/arch/pll.h>

#define MODULE_NAME "DBMX2 I2C"

#define PKMOD MODULE_NAME ": "

#define MX_I2C_TIMEOUT HZ	/*timeout for an I2C transfer */

#undef MX_I2C_WAIT_FOR_BUS
#define MX_I2C_WAIT_FOR_BUS	/*enable bus waiting code from old i2c-aa for debugging */
#undef MX_I2C_HIGH_SPEED	/*enable high-speed mode. Use it only if all devices support high-speed */
#undef MX_I2C_EXTRA_PULLUP	/*utilize extra pullup resistor for high speed mode */

static DECLARE_WAIT_QUEUE_HEAD(__i2c_wait);
static DECLARE_MUTEX(__i2c_lock);

/*initialization progress indicators*/
static char __initstate_adap;
static char __initstate_region;
static char __initstate_gpio;
#ifdef MX_I2C_EXTRA_PULLUP
static char __initstate_pullup;
#endif
static char __initstate_irq;

static unsigned int __ipg_clk_divider[] = {
	30, 32, 36, 42, 48, 52, 60, 72, 80, 88, 104, 128, 144, 160, 192, 240,
	288, 320, 384, 480, 576, 640, 768, 960, 1152, 1280, 1536, 1920, 2304,
	2560, 3072, 3840,
	22, 24, 26, 28, 32, 36, 40, 44, 48, 56, 64, 72, 80, 96, 112, 128,
	160, 192, 224, 256, 320, 384, 448, 512, 640, 768, 896, 1024, 1280, 1536,
	1792, 2048
};

#ifdef MX_I2C_WAIT_FOR_BUS
static int __wait_for_bus_busy(void);
static void __wait_for_bus_idle(void);
#endif

static void __i2c_interrupt(int irq, void *dev_id, struct pt_regs *reg);
static int __wait_for_end_of_tx(char verify_ack);

int __init i2c_dbmx2_init(void);
static void __init i2c_dbmx2_exit(void);

static void __i2c_inc_use(struct i2c_adapter *adap);
static void __i2c_dec_use(struct i2c_adapter *adap);
static int __i2c_master_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num);
static u32 __i2c_functionality(struct i2c_adapter *adap);

static struct i2c_algorithm __i2c_algorithm = {
	.name = MODULE_NAME " algo",
	.id = I2C_ALGO_OCP,
	.master_xfer = __i2c_master_xfer,
	.functionality = __i2c_functionality
};

static struct i2c_adapter __i2c_adapter = {
	.name = MODULE_NAME " adap",
	.id = I2C_ALGO_OCP | I2C_HW_OCP,
	.algo = &__i2c_algorithm,
	.inc_use = __i2c_inc_use,
	.dec_use = __i2c_dec_use
};

/*report type of I2C*/
static u32
__i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static void
__i2c_inc_use(struct i2c_adapter *adap)
{
	MOD_INC_USE_COUNT;
}

static void
__i2c_dec_use(struct i2c_adapter *adap)
{
	MOD_DEC_USE_COUNT;
}

/* Find best suited frequency divider (speed in Hz) */
static unsigned char
__calculate_ifdr(unsigned int speed)
{
	unsigned int ipg_clk = mx_module_get_clk(IPG_MODULE_I2C);
	unsigned int i, j;

	for (i = j = 0; i < sizeof (__ipg_clk_divider) / sizeof (unsigned int);
	     i++)
		if (abs(speed - (ipg_clk / __ipg_clk_divider[i])) <
		    abs(speed - (ipg_clk / __ipg_clk_divider[j])))
			j = i;

	return j;
}

#ifdef MX_I2C_WAIT_FOR_BUS
/* Waits for I2C bus to become busy (start signal detected).
   Returns 0 if arbitration OK, 1 if arbitration lost. */
static int
__wait_for_bus_busy(void)
{
	unsigned long int val;

	while (!((val = I2C_I2SR) & I2SR_BUSBUSY)) {
		if (val & I2SR_LOSTARB) {
			return 1;
		}
		schedule_timeout(1);
	}
	return 0;
}

/* Waits for I2C bus to become idle (stop signal detected) */
static void
__wait_for_bus_idle(void)
{
	while (I2C_I2SR & I2SR_BUSBUSY)
		schedule_timeout(1);
}
#endif

/* wait for end of transfer*/
static int
__wait_for_end_of_tx(char verify_ack)
{
	unsigned long tx_status;
	int ret = 0;
	wait_queue_t tmp_wait;

	tx_status = I2C_I2SR;
	if (tx_status & I2SR_INTPEND)
		goto end;

	init_waitqueue_entry(&tmp_wait, current);
	add_wait_queue(&__i2c_wait, &tmp_wait);
	set_current_state(TASK_UNINTERRUPTIBLE);

	I2C_I2CR |= I2CR_INTEN;	/*enable interrupt */

	schedule_timeout(MX_I2C_TIMEOUT);
	current->state = TASK_RUNNING;
	remove_wait_queue(&__i2c_wait, &tmp_wait);

	I2C_I2CR &= ~I2CR_INTEN;	/*disable interrupt */

	tx_status = I2C_I2SR;

	if (!(tx_status & I2SR_INTPEND))
		ret = -ETIMEDOUT;

      end:

	if ((tx_status & I2SR_ACKRCVD) && verify_ack && !ret)
		ret = -EPROTO;

	I2C_I2SR = tx_status & ~I2SR_INTPEND;

	return ret;		/*timeout happened */
}

/*send/receive data*/
int
__i2c_master_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num)
{
	int i, j, count = 0;
	int ret = -EIO;
	unsigned long int addr;

	if (num < 1) {
		return -EINVAL;
	}

	/* Check for valid parameters in messages */
	for (i = 0; i < num; i++) {
		if (&msgs[i] == NULL) {
			return -EINVAL;
		}

		if (msgs[i].buf == NULL) {
			return -EINVAL;
		}
	}

	down(&__i2c_lock);
	mx_module_clk_open(IPG_MODULE_I2C);

#ifdef MX_I2C_WAIT_FOR_BUS
      start:
#endif

	/*due to I2C hardware bugs we cannot turn off and on
	   master mode without full initialization */
	I2C_I2CR = 0;
	I2C_I2CR = I2CR_ENABLE;

#ifdef MX_I2C_HIGH_SPEED
#ifdef MX_I2C_EXTRA_PULLUP
	mx2_gpio_set_bit(PORT_D, 20, 1);	/*enable extra pullup */
#endif
	I2C_IFDR = __calculate_ifdr(400000);
#else
	I2C_IFDR = __calculate_ifdr(100000);
#endif
	udelay(10);

	I2C_I2CR |= I2CR_MASTER;	/*send start signal */

#ifdef MX_I2C_WAIT_FOR_BUS
	if (__wait_for_bus_busy()) {
		/*arbitration lost */
		I2C_I2CR &= ~I2CR_MASTER;	/*send stop signal */
		__wait_for_bus_idle();
		goto start;
	}
#endif

	for (i = 0; i < num; i++) {	/* deal with  message i */
		/*prepare address */
		addr = (msgs[i].addr << 1);
		if (msgs[i].flags & I2C_M_RD)
			addr |= 1;
		if (msgs[i].flags & I2C_M_REV_DIR_ADDR)
			addr ^= 1;

		I2C_I2CR &= ~I2CR_NOACK;	/*turn on ACK */
		/*transmit I2C address */
		I2C_I2CR |= I2CR_TRANSMIT;
		I2C_I2DR = addr;
		if ((ret = __wait_for_end_of_tx(1)) < 0)
			goto end_tx;

		if (msgs[i].len > 0) {
			if (msgs[i].flags & I2C_M_RD) {

				/*transmit data */

				I2C_I2CR &= ~I2CR_TRANSMIT;

				/*if second before last txfer in the last msg - disable ACK */
				if ((msgs[i].len == 1) && (i == num - 1))
					I2C_I2CR |= I2CR_NOACK;

				msgs[i].buf[0] = I2C_I2DR;	/*dummy read to start clock */

				if (msgs[i].len > 2)
					for (j = 0; j < msgs[i].len - 2; j++) {
						if ((ret =
						     __wait_for_end_of_tx(0)) <
						    0)
							goto end_tx;
						msgs[i].buf[j] = I2C_I2DR;
					}

				if ((ret = __wait_for_end_of_tx(0)) < 0)
					goto end_tx;

				/*second before last byte, unless length is 1 byte */
				if (msgs[i].len > 1) {
					if (i == num - 1)
						I2C_I2CR |= I2CR_NOACK;	/*unack in the last msg */
					msgs[i].buf[msgs[i].len - 2] = I2C_I2DR;

					if ((ret = __wait_for_end_of_tx(0)) < 0)
						goto end_tx;
				}

				if (i == num - 1) {	/*the last message done, stop the bus */
					I2C_I2CR &= ~I2CR_MASTER;	/*send stop signal */

#ifdef MX_I2C_WAIT_FOR_BUS
					__wait_for_bus_idle();
#endif
				} else
					I2C_I2CR |= I2CR_REPSTART;	/*not the last message, repeat start */

				/* read the last byte */
				msgs[i].buf[msgs[i].len - 1] = I2C_I2DR;

			} /* end of if(READ) */
			else {	/* write data */

				for (j = 0; j < msgs[i].len; j++) {
					I2C_I2DR = msgs[i].buf[j];
					if ((ret =
					     __wait_for_end_of_tx((j !=
								   msgs[i].len -
								   1) ? 1 : 0))
					    < 0)
						goto end_tx;
				}

				if (i == num - 1) {	/*the last message done, stop the bus */

					if (!(msgs[i].flags & I2C_M_NOSTART)) {
						/*I2C_M_NOSTART flag is privately used here as a workaround */
						/*I2C hardware bug workaround - dummy repeat start.
						   Without this code the I2C will not send out data againg */
						/*must be disabled for OV9640,
						   because OV9640 doesn't like repeat start.
						   In other cases must be enabled */
						I2C_I2CR |= I2CR_REPSTART;
						I2C_I2DR = 0;
						__wait_for_end_of_tx(0);
						/*end of hardware bug workaround */
					}

					I2C_I2CR &= ~I2CR_MASTER;	/*send stop signal */
#ifdef MX_I2C_WAIT_FOR_BUS
					__wait_for_bus_idle();
#endif
				} else
					I2C_I2CR |= I2CR_REPSTART;	/*not the last message, repeat start */

			}	/* end of else(WRITE) */
		} else {
			I2C_I2CR &= ~I2CR_MASTER;	/*send stop signal */
#ifdef MX_I2C_WAIT_FOR_BUS
			__wait_for_bus_idle();
#endif
		}
		count++;
	}			/* for */

	ret = count;

      end_tx:
	if (ret < 0) {		/*finished with an error */
		switch (ret) {
			case -EPROTO:
				printk(KERN_ERR PKMOD "ACK not received. Wrong I2C address?\n");
			break;
			case -ETIMEDOUT:
				printk(KERN_ERR PKMOD "I/O timeout\n");
			break;
			default:
				printk(KERN_ERR PKMOD "I/O error %d\n", ret);
		}
		I2C_I2CR &= ~I2CR_MASTER;	/*send stop signal */
#ifdef MX_I2C_WAIT_FOR_BUS
		__wait_for_bus_idle();
#endif
	}
	/* Clear the I2CR to disable I2C module */
#ifdef MX_I2C_HIGH_SPEED
#ifdef MX_I2C_EXTRA_PULLUP
	mx2_gpio_set_bit(PORT_D, 20, 0);	/*disable extra pullup */
#endif
#endif
	I2C_I2CR = 0;
	mx_module_clk_close(IPG_MODULE_I2C);
	up(&__i2c_lock);
	return ret;
}

/*interrupt handler*/
static void
__i2c_interrupt(int irq, void *dev_id, struct pt_regs *reg)
{
	I2C_I2CR &= ~I2CR_INTEN;	/*disable interrupt */
	wake_up(&__i2c_wait);
	return;
}

/*initialize I2C*/
int __init
i2c_dbmx2_init(void)
{
	int tmp;

	/*printk(KERN_INFO PKMOD "ver. 1.0 built on %s %s\n", __TIME__, __DATE__); */

	tmp = (int) request_region(I2C_BASE, I2C_IO_SIZE, "i2c");
	if (!tmp) {
		printk(KERN_ERR PKMOD "I2C is already in use\n");
		i2c_dbmx2_exit();
		return -1;
	}
	__initstate_region = 1;

/*
 * I2C pins:
 * I2C_SCL: PD18 PRIMARY BD w/HYS OD
 * I2C_SDA: PD17 PRIMARY BD       OD
 * PD20: enable additional pull-up resistors for high-speed mode
 */
#ifdef MX_I2C_EXTRA_PULLUP
	tmp = mx2_register_gpio(PORT_D, 20, GPIO | OUTPUT | NOINTERRUPT);
	if (tmp < 0) {
		printk(KERN_ERR PKMOD "PORT_D pin 20 is already in use\n");
		i2c_dbmx2_exit();
		return tmp;
	}
	__initstate_pullup = 1;
	mx2_gpio_set_bit(PORT_D, 20, 0);
#endif

	tmp =
	    mx2_register_gpios(PORT_D, ((1 << 17) | (1 << 18)),
			       PRIMARY | OUTPUT | NOINTERRUPT);
	if (tmp < 0) {
		printk(KERN_ERR PKMOD
		       "PORT_D pins 17 and 18 are already in use\n");
		i2c_dbmx2_exit();
		return tmp;
	}
	__initstate_gpio = 1;

	tmp = request_irq(INT_I2C, __i2c_interrupt, SA_INTERRUPT, "mx_i2c", "i2c_bus");
	if (tmp) {
		printk(KERN_ERR PKMOD "failed to request irq\n");
		i2c_dbmx2_exit();
		return tmp;
	}
	__initstate_irq = 1;

	/* add the I2C adapter/algorithm driver to the linux kernel */
	tmp = i2c_add_adapter(&__i2c_adapter);
	if (tmp) {
		printk(KERN_ERR PKMOD "failed to add adapter\n");
		i2c_dbmx2_exit();
		return tmp;
	}
	__initstate_adap = 1;

	return 0;
}

/*deinitialize*/
static void __init
i2c_dbmx2_exit(void)
{
	if (__initstate_adap)
		i2c_del_adapter(&__i2c_adapter);
	if (__initstate_irq)
		free_irq(INT_I2C, "i2c_bus");
	if (__initstate_gpio)
		mx2_unregister_gpios(PORT_D, ((1 << 17) | (1 << 18)));
#ifdef MX_I2C_EXTRA_PULLUP
	if (__initstate_pullup) {
		mx2_gpio_set_bit(PORT_D, 20, 0);
		mx2_unregister_gpio(PORT_D, 20);
	}
#endif
	if (__initstate_region) {
		release_region(I2C_BASE, I2C_IO_SIZE);
	}
}

#ifdef MODULE			/* initialized from i2c_core if not module */
MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION("I2C driver for Motorola DragonBall i.MX21");
MODULE_LICENSE("GPL");
module_init(i2c_dbmx2_init);
module_exit(i2c_dbmx2_exit);
#endif
