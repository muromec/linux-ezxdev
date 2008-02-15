/*
 *  Derived from:
 *   linux/drivers/l3/l3-bit-sa1100.c
 *
 *  Copyright (C) 2001 Russell King
 *  Copyright (C) 2001 Steve Johnson (stevej@ridgerun.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This is a L3 bus driver.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/i2c-algo-bit.h>
#include <linux/l3/algo-bit.h>

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>

#define NAME "l3-bit-omap1510-gpio"

struct bit_data {
	unsigned int	sda;
	unsigned int	scl;
	unsigned int	l3_mode;
};

/*
 * L3 bus GPIO pin definitions
 */

#define L3_DataPin	(1 << 8)
#define L3_ClockPin	(1 << 7)
#define L3_ModePin	(1 << 6)

   /* GPIO Alternate Function Reg.    */
#define GAFR (*(volatile u16 *) GPIO_INT_MASK_REG)  
   /* GPIO Pin output Set Reg.        */
#define GPSR (*(volatile u16 *) GPIO_DATA_OUTPUT_REG)
   /* GPIO Pin output Clear Reg.      */
#define GPCR GPSR
   /* GPIO Pin Direction Reg.         */
#define GPDR (*(volatile u16 *) GPIO_DIR_CONTROL_REG)
   /* GPIO Pin Level Reg.             */
#define GPLR (*(volatile u16 *) GPIO_DATA_INPUT_REG)
   /* GPIO Pin Control Reg.             */
#define GPCtlR (*(volatile u16 *) GPIO_PIN_CONTROL_REG)


static int getsda(void *data)
{
	struct bit_data *bits = data;

	return GPLR & bits->sda;
}

/*
 * iPAQs need the clock line driven hard high and low.
 */
static void l3_setscl(void *data, int state)
{
	struct bit_data *bits = data;
	unsigned long flags;

	local_irq_save(flags);
	if (state)
		GPSR |= bits->scl;
	else
		GPCR &= ~bits->scl;
	GPDR &= ~bits->scl;              // 0 for output
	local_irq_restore(flags);
}

static void l3_setsda(void *data, int state)
{
	struct bit_data *bits = data;

	if (state)
		GPSR |= bits->sda;
	else
		GPCR &= ~bits->sda;
}

static void l3_setdir(void *data, int in)
{
	struct bit_data *bits = data;
	unsigned long flags;

	local_irq_save(flags);
        // 0 for output
	if (in)
		GPDR |= bits->sda;
	else
		GPDR &= ~bits->sda;
	local_irq_restore(flags);
}

static void l3_setmode(void *data, int state)
{
	struct bit_data *bits = data;

	if (state)
		GPSR |= bits->l3_mode;
	else
		GPCR &= ~bits->l3_mode;
}

static struct l3_algo_bit_data l3_bit_data = {
	data:		NULL,
	setdat:		l3_setsda,
	setclk:		l3_setscl,
	setmode:	l3_setmode,
	setdir:		l3_setdir,
	getdat:		getsda,
	data_hold:	1,
	data_setup:	1,
	clock_high:	1,
	mode_hold:	1,
	mode_setup:	1,
};

static DECLARE_MUTEX(l3_lock);
#define LOCK		&l3_lock

static struct l3_adapter l3_adapter = {
	owner:		THIS_MODULE,
	name:		NAME,
	algo_data:	&l3_bit_data,
	lock:		LOCK,
};

static struct bit_data bit_data;

static int __init l3_init(struct bit_data *bits)
{
	l3_bit_data.data = bits;
	return l3_bit_add_bus(&l3_adapter);
}

static void __exit l3_exit(void)
{
	struct bit_data *bit = &bit_data;

        GPCR &= ~(bit->sda | bit->scl | bit->l3_mode);   // turn off

	l3_bit_del_bus(&l3_adapter);
}

static int __init bus_init(void)
{
	struct bit_data *bit = &bit_data;
	unsigned long flags;
	int ret;

        bit->sda     = L3_DataPin;
        bit->scl     = L3_ClockPin;
        bit->l3_mode = L3_ModePin;

	if (!bit->sda)
		return -ENODEV;

	/*
	 * Default level for L3 mode is low.
	 * We set SCL and SDA high (i2c idle state).
	 */
	local_irq_save(flags);
        GPDR |= (bit->sda | bit->scl | bit->l3_mode);   // input while changing state
        GAFR |= (bit->sda | bit->scl | bit->l3_mode);   // not ints 
        GPCtlR |= (bit->sda | bit->scl | bit->l3_mode); // MPU pin
        GPSR |= (bit->sda | bit->scl | bit->l3_mode);   // 1 output
        GPDR &= ~(bit->sda | bit->scl | bit->l3_mode);  // 0 for output
	local_irq_restore(flags);

        ret = l3_init(bit);

	return ret;
}

static void __exit bus_exit(void)
{
	l3_exit();
}

module_init(bus_init);
module_exit(bus_exit);
