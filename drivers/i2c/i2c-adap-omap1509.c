/*
   -------------------------------------------------------------------------
   i2c-adap-omap1509.c i2c driver algorithms for OMAP support
   
   Steve Johnson, RidgeRun, Inc.

   Copyright 2002 RidgeRun, Inc.

   -------------------------------------------------------------------------
   This file was highly leveraged from i2c-adap-ite.c, which was created
   by Hai-Pao Fan.

   Hai-Pao Fan, MontaVista Software, Inc.
   hpfan@mvista.com or source@mvista.com

   Copyright 2001 MontaVista Software Inc.

   ----------------------------------------------------------------------------
   This file was highly leveraged from i2c-elektor.c, which was created
   by Simon G. Vogl and Hans Berglund:

 
     Copyright (C) 1995-97 Simon G. Vogl
                   1998-99 Hans Berglund

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.		     */
/* ------------------------------------------------------------------------- */

/* With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and even
   Frodo Looijaard <frodol@dds.nl> */

#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "i2c-algo-omap1509.h"
#include "i2c-omap1509.h"

#define DEFAULT_BASE          I2C_BASE
#define OMAP1509_I2C_IO_SIZE  0x24
#define DEFAULT_IRQ           INT_I2C
#define DEFAULT_CLOCK         0xd289	// default 100KHz 
#define DEFAULT_OWN           0         // We can't be a slave on 1509.

static int clock;
static int own;

static int i2c_debug;
static struct {
        int i2c_own;
        int i2c_clock;
        int i2c_irq;
        int i2c_base;
} gpi;
static wait_queue_head_t i2c_wait;
static int i2c_pending;

/* ----- global defines -----------------------------------------------	*/
#define DEB(x)	if (i2c_debug>=1) x
#define DEB2(x) if (i2c_debug>=2) x
#define DEB3(x) if (i2c_debug>=3) x
#define DEBE(x)	x	/* error messages 				*/


/* ----- local functions ----------------------------------------------	*/

static void i2c_omap1509_set(void *data, int ctl, u16 val)
{
        unsigned long j = jiffies + 10;

	DEB3(printk(" I2C write 0x%02x to 0x%x\n",(unsigned short)val, ctl&0xff));
	DEB3({while (jiffies < j) schedule();});
        outw(val,ctl);
}

static u16 i2c_omap1509_get(void *data, int ctl)
{
	u16 val;

	val = inw(ctl);
	DEB3(printk(" I2C read 0x%02x from 0x%x\n",(unsigned short)val, ctl&0xff));  
	return (val);
}

/* Return our slave address.  This is the address
 * put on the I2C bus when another master on the bus wants to address us
 * as a slave
 */
static int i2c_omap1509_getown(void *data)
{
	return (gpi.i2c_own);
}


static int i2c_omap1509_getclock(void *data)
{
	return (gpi.i2c_clock);
}


/* Put this process to sleep.  We will wake up when the
 * I2C controller interrupts.
 */
static void i2c_omap1509_waitforpin(void) 
{

        int timeout = 1;
        int flag;

        /* If interrupts are enabled (which they are), then put the process to
         * sleep.  This process will be awakened by two events -- either the
         * the I2C peripheral interrupts or the timeout expires. 
         * If interrupts are not enabled then delay for a reasonable amount 
         * of time and return.
         */
        if (gpi.i2c_irq > 0) {
                local_irq_save(flag);
                if (i2c_pending == 0) {
                        interruptible_sleep_on_timeout(&i2c_wait, timeout * HZ);
                }
                i2c_pending = 0;
                local_irq_restore(flag);
        } else {
                udelay(100);
        }
}


/*
  NOTE:
  In the 1509 Errata, it says you can't read the I2C status register
  immediately after detecting the interrupt.  So don't read the status
  here, let the upper layers handle the status.
*/
static void i2c_omap1509_handler(int this_irq, void *dev_id, struct pt_regs *regs) 
{
	
        i2c_pending = 1;

        DEB2(printk(__FUNCTION__ ": in interrupt handler\n"));
        wake_up_interruptible(&i2c_wait);
}


/* Lock the region of memory where I/O registers exist.  Request our
 * interrupt line and register its associated handler.
 */
static int i2c_hw_resrc_init(void)
{
  	if (check_region(gpi.i2c_base, OMAP1509_I2C_IO_SIZE) < 0 ) {
                return -ENODEV;
  	} else {
                request_region(gpi.i2c_base, OMAP1509_I2C_IO_SIZE, 
                               "i2c (i2c bus adapter)");
  	}
	if (gpi.i2c_irq > 0) {
                if (request_irq(gpi.i2c_irq, i2c_omap1509_handler, 0, "OMAP I2C", 0) < 0) {
                        gpi.i2c_irq = 0;
                } else
                        DEB3(printk("Enabled I2C IRQ %d\n", gpi.i2c_irq));
                enable_irq(gpi.i2c_irq);
	}
	return 0;
}


static void i2c_omap1509_release(void)
{
	if (gpi.i2c_irq > 0) {
		disable_irq(gpi.i2c_irq);
		free_irq(gpi.i2c_irq, 0);
	}
	release_region(gpi.i2c_base, 2);
}


static int i2c_omap1509_reg(struct i2c_client *client)
{
	return 0;
}


static int i2c_omap1509_unreg(struct i2c_client *client)
{
	return 0;
}


static void i2c_omap1509_inc_use(struct i2c_adapter *adap)
{
#ifdef MODULE
	MOD_INC_USE_COUNT;
#endif
}


static void i2c_omap1509_dec_use(struct i2c_adapter *adap)
{
#ifdef MODULE
	MOD_DEC_USE_COUNT;
#endif
}


/* ------------------------------------------------------------------------
 * Encapsulate the above functions in the correct operations structure.
 * This is only done when more than one hardware adapter is supported.
 */
static struct i2c_algo_omap1509_data i2c_omap1509_data = {
	NULL,
	i2c_omap1509_set,
	i2c_omap1509_get,
	i2c_omap1509_getown,
	i2c_omap1509_getclock,
	i2c_omap1509_waitforpin,
	80, 80, 100,		/*	waits, timeout */
};

static struct i2c_adapter i2c_omap1509_ops = {
	"OMAP1509 I2C adapter",
	I2C_ALGO_EXP,                   // need real number some day.
	NULL,
	&i2c_omap1509_data,
	i2c_omap1509_inc_use,
	i2c_omap1509_dec_use,
	i2c_omap1509_reg,
	i2c_omap1509_unreg,
};

/* Called when the module is loaded.  This function starts the
 * cascade of calls up through the heirarchy of i2c modules (i.e. up to the
 *  algorithm layer and into to the core layer)
 */
static int __init i2c_omap1509_init(void) 
{
	printk(KERN_INFO "Initialize OMAP I2C adapter module\n");
        gpi.i2c_base = DEFAULT_BASE;
        gpi.i2c_irq = DEFAULT_IRQ;

	if (clock == 0)
		gpi.i2c_clock = DEFAULT_CLOCK;
	else
		gpi.i2c_clock = clock;

	if (own == 0)
		gpi.i2c_own = DEFAULT_OWN;
	else
		gpi.i2c_own = own;

	i2c_omap1509_data.data = (void *)&gpi;

	init_waitqueue_head(&i2c_wait);

	if (i2c_hw_resrc_init() == 0) {
		if (i2c_omap1509_add_bus(&i2c_omap1509_ops) < 0)
			return -ENODEV;
	} else {
		return -ENODEV;
	}

	printk(KERN_INFO " found device at %#x irq %d.\n", 
               gpi.i2c_base, gpi.i2c_irq);
	return 0;
}


static void i2c_omap1509_exit(void)
{
	i2c_omap1509_del_bus(&i2c_omap1509_ops);
        i2c_omap1509_release();
}

EXPORT_NO_SYMBOLS;

/* If modules is NOT defined when this file is compiled, then the MODULE_*
 * macros will resolve to nothing
 */
MODULE_AUTHOR("RidgeRun, Inc.");
MODULE_DESCRIPTION("I2C-Bus adapter routines for OMAP1509 I2C bus adapter");
MODULE_LICENSE("GPL");

MODULE_PARM(clock, "i");
MODULE_PARM(own, "i");
MODULE_PARM(i2c_debug,"i");


/* Called when module is loaded or when kernel is intialized.
 * If MODULES is defined when this file is compiled, then this function will
 * resolve to init_module (the function called when insmod is invoked for a
 * module).  Otherwise, this function is called early in the boot, when the
 * kernel is intialized.  Check out /include/init.h to see how this works.
 */
module_init(i2c_omap1509_init);

/* Resolves to module_cleanup when MODULES is defined. */
module_exit(i2c_omap1509_exit); 
