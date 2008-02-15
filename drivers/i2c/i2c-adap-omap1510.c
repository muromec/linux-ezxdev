/*
 * i2c-omap1510.c
 *
 * Simple bit algo I2C test driver for the TI OMAP1510.
 *
 * Copyright (C) 2002 MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/stddef.h>
#include <linux/parport.h>
#include <linux/rtc.h>

#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/system.h>
#include "i2c-omap1510.h"

#undef	DEBUG
#if	defined(DEBUG)
#	define	DPRINTK(fmt, args...)	printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
#else
#	define	DPRINTK(fmt, args...)
#endif


static int iic_acquired;
static int iic_data_out;
static int iic_clock_out;

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CEE */
#include <linux/device.h>

static int i2c_ldm_suspend(struct device *dev, u32 state, u32 level);
static int i2c_ldm_resume(struct device *dev, u32 level);

static struct device_driver i2c_driver_ldm = {
       name:      "i2c-adap-omap1510",
       devclass:  NULL,
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
#endif /* MVL-CEE */


/*
 * Grab control of the I2C pins
 */
static inline void iic_acquirepins(void)
{
	I2C_SYSTEST |= SCL_O;
	I2C_SYSTEST &= ~SDA_O;
	iic_acquired = 1;
	iic_clock_out = 1;
	iic_data_out = 0;
}

/*
 * Release control of the I2C pins
 */
static inline void iic_releasepins(void)
{
	I2C_SYSTEST |= SCL_O | SDA_O;
	iic_acquired = 0;
}

/*
 * Initialize the interface
 */
static void iic_init(void)
{
  	// make sure we're using the "new" I2C interface
	outl(inl(MOD_CONF_CTRL_0) & ~(1<<16), MOD_CONF_CTRL_0);

	I2C_SYSTEST = ST_EN | FREE | (3 << TMODE_SHIFT);
	iic_releasepins();
}

#ifdef CONFIG_OMAP_INNOVATOR

static int i2c_ldm_suspend(struct device *dev, u32 state, u32 level)
{
 
  switch(level)
  { 
     case SUSPEND_POWER_DOWN:
       iic_releasepins();
       break;
  }      

  return 0;
}

static int i2c_ldm_resume(struct device *dev, u32 level)
{
   
  switch(level)
  {
     case RESUME_POWER_ON:
       iic_init();
       break;
  }

  return 0;  
}

#endif

/* --------- */

static void bit_omap1510_setscl(void *data, int state)
{
	if (!iic_acquired)
		iic_acquirepins();
	if (!iic_clock_out) {
		iic_clock_out = 1;
	}
	if (state) {
		I2C_SYSTEST |= SCL_O;
	} else {
		I2C_SYSTEST &= ~SCL_O;
	}
}

static void bit_omap1510_setsda(void *data, int state)
{
	if (!iic_acquired)
		iic_acquirepins();
	if (state) {
		I2C_SYSTEST |= SDA_O;
	} else {
		I2C_SYSTEST &= ~SDA_O;
	}
} 

static int bit_omap1510_getscl(void *data)
{
	if (!iic_acquired)
		iic_acquirepins();

	return (I2C_SYSTEST & SCL_I) ? 1 : 0;
}

static int bit_omap1510_getsda(void *data)
{
	if (!iic_acquired)
		iic_acquirepins();
	if (iic_data_out) {
		iic_data_out = 0;
	}
	return (I2C_SYSTEST & SDA_I) ? 1 : 0;
}

/* ---------- */

static int bit_omap1510_reg(struct i2c_client *client)
{
	return 0;
}

static int bit_omap1510_unreg(struct i2c_client *client)
{
	return 0;
}

static void bit_omap1510_inc_use(struct i2c_adapter *adap)
{
#ifdef MODULE
	MOD_INC_USE_COUNT;
#endif
	iic_acquirepins();
}

static void bit_omap1510_dec_use(struct i2c_adapter *adap)
{
	iic_releasepins();
#ifdef MODULE
	MOD_DEC_USE_COUNT;
#endif
}

static struct i2c_algo_bit_data bit_omap1510_data = {
	data:		NULL,
	setsda:		bit_omap1510_setsda,
	setscl:		bit_omap1510_setscl,
	getsda:		bit_omap1510_getsda,
	getscl:		bit_omap1510_getscl,
	udelay:		10,
	mdelay:		10,
	timeout:	100,
};

static struct i2c_adapter bit_omap1510_ops = {
	name:			"OMAP1510",
	id:			I2C_HW_B_IOC,
	algo:			NULL,
	algo_data:		&bit_omap1510_data,
	inc_use:		bit_omap1510_inc_use,
	dec_use:		bit_omap1510_dec_use,
	client_register:	bit_omap1510_reg,
	client_unregister:	bit_omap1510_unreg,
};

int __init i2c_omap1510_init(void)
{
	printk("i2c-omap1510.o: i2c OMAP1510 module\n");
	/* reset hardware to sane state */
	iic_init();
	if (i2c_bit_add_bus(&bit_omap1510_ops) < 0) {
		printk("i2c-omap1510: Unable to register with I2C\n");
		return -ENODEV;
	}

#ifdef CONFIG_OMAP_INNOVATOR /* MVL-CEE */
        i2c_ldm_device_register();
        i2c_ldm_driver_register();
#endif                      /* MVL-CEE */        

	return 0;
}

void __exit i2c_omap1510_exit(void)
{

#ifdef CONFIG_OMAP_INNOVATOR /* MVL-CEE */
        i2c_ldm_device_unregister();
        i2c_ldm_driver_unregister();
#endif                      /* MVL-CEE */ 
	i2c_bit_del_bus(&bit_omap1510_ops);
}


MODULE_AUTHOR("George G. Davis <davis_g@mvista.com>");
MODULE_DESCRIPTION("I2C-Bus bit algo test driver for TI OMAP1510");
MODULE_LICENSE("GPL");

module_init(i2c_omap1510_init);
module_exit(i2c_omap1510_exit);
