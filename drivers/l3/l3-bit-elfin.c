/*
 * drivers/l3/l3-bit-elfin.c
 *
 * Copyright (C) 2004 Samsung Electronics Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * 2004-04-28 : Kwanghyun la <nala.la@samsung.com>
 *   - modified for sharing module device driver of samsung  arch
 *
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/l3/algo-bit.h>

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>

#define NAME "l3-bit-elfin-gpio"

struct bit_data {
	unsigned int	sda;
	unsigned int	scl;
	unsigned int	l3_mode;
};

static int getsda(void *data)
{
	struct bit_data *bits = data;

	return read_gpio_bit(bits->sda);
}

static DECLARE_MUTEX(l3_lock);
#define LOCK		&l3_lock

/*
 * iPAQs need the clock line driven hard high and low.
 */
static void l3_setscl(void *data, int state)
{
	struct bit_data *bits = data;
	unsigned long flags;

	local_irq_save(flags);
	if (state)
		write_gpio_bit(bits->scl, 1);
	else
		write_gpio_bit(bits->scl, 0);
	set_gpio_ctrl((bits->scl) | GPIO_PULLUP_DIS | GPIO_MODE_OUT);
	local_irq_restore(flags);
}

static void l3_setsda(void *data, int state)
{
	struct bit_data *bits = data;

	if (state)
		write_gpio_bit(bits->sda, 1);
	else
		write_gpio_bit(bits->sda, 0);
}

static void l3_setdir(void *data, int in)
{
	struct bit_data *bits = data;
	unsigned long flags;

	local_irq_save(flags);
	if (in)
		set_gpio_ctrl((bits->sda) | GPIO_PULLUP_DIS | GPIO_MODE_IN);
	else
		set_gpio_ctrl((bits->sda) | GPIO_PULLUP_DIS | GPIO_MODE_OUT);
	local_irq_restore(flags);
}

static void l3_setmode(void *data, int state)
{
	struct bit_data *bits = data;

	if (state)
		write_gpio_bit(bits->l3_mode, 1);
	else
		write_gpio_bit(bits->l3_mode, 0);
}

static struct l3_algo_bit_data l3_bit_data = {
	.data 		= NULL,
	.setdat		= l3_setsda,
	.setclk		= l3_setscl,
	.setmode	= l3_setmode,
	.setdir		= l3_setdir,
	.getdat		= getsda,
	.data_hold	= 1,
	.data_setup	= 1,
	.clock_high	= 1,
	.mode_hold	= 1,
	.mode_setup	= 1,
};

static struct l3_adapter l3_adapter = {
	.owner	= THIS_MODULE,
	.name	= NAME,
	.algo_data = &l3_bit_data,
	.lock	= LOCK,
};

static int inline l3_start(struct bit_data *bits)
{
	l3_bit_data.data = bits;
	return l3_bit_add_bus(&l3_adapter);
}

static void inline l3_end(void)
{
	l3_bit_del_bus(&l3_adapter);
}

static struct bit_data bit_data;

static int __init bus_init(void)
{
	struct bit_data *bit = &bit_data;
	unsigned long flags;
	int ret;

#if defined(CONFIG_BOARD_S3C24A0_SMDK)
	bit->sda     = GPIO_15;
	bit->scl     = GPIO_16;
	bit->l3_mode = GPIO_17;
#elif defined(CONFIG_BOARD_S3C2440_SMDK)
	bit->sda     = GPIO_B3;
	bit->scl     = GPIO_B4;
	bit->l3_mode = GPIO_B2;
#endif

	if (!bit->sda)
		return -ENODEV;

	/*
	 * Default level for L3 mode is low.
	 */

	local_irq_save(flags);

	/* L3 gpio interface set */
	write_gpio_bit(bit->l3_mode, 1);
	write_gpio_bit(bit->scl, 1);
	set_gpio_ctrl((bit->scl) | GPIO_PULLUP_DIS | GPIO_MODE_OUT);
	set_gpio_ctrl((bit->sda) | GPIO_PULLUP_DIS | GPIO_MODE_OUT);
	set_gpio_ctrl((bit->l3_mode) | GPIO_PULLUP_DIS | GPIO_MODE_OUT);

#ifdef CONFIG_BOARD_S3C2440_SMDK
	/* IIS gpio interface set */
        /* GPE 0: I2SLRCK */
        set_gpio_ctrl(GPIO_E0 | GPIO_PULLUP_EN | GPIO_MODE_I2SSDI);
        /* GPE 1: I2SSCLK */
        set_gpio_ctrl(GPIO_E1 | GPIO_PULLUP_EN | GPIO_MODE_I2SSCLK);
        /* GPE 2: CDCLK */
        set_gpio_ctrl(GPIO_E2 | GPIO_PULLUP_EN | GPIO_MODE_CDCLK);
        /* GPE 3: I2SSDI */
        set_gpio_ctrl(GPIO_E3 | GPIO_PULLUP_EN | GPIO_MODE_I2SSDI);
        /* GPE 4: I2SSDO */
        set_gpio_ctrl(GPIO_E4 | GPIO_PULLUP_EN | GPIO_MODE_I2SSDO);	
#if defined( CONFIG_SOUND_SMDK2440A_AC97) || defined (CONFIG_SOUND_SMDK2440A_AC97_MODULE)
	/* AC97 gpio interface set */
	printk(KERN_INFO "GPIO SET FOR AC97 \n");
        /* GPE 0: AC97 SYNC */
        set_gpio_ctrl(GPIO_E0 | GPIO_PULLUP_EN | GPIO_MODE_AC_SYNC);
        /* GPE 1: AC97 BIT CLOCK */
        set_gpio_ctrl(GPIO_E1 | GPIO_PULLUP_EN | GPIO_MODE_AC_BIT_CLK);
        /* GPE 2: AC97 nRESET */
        set_gpio_ctrl(GPIO_E2 | GPIO_PULLUP_EN | GPIO_MODE_AC_nRESET);
        /* GPE 3: AC97 SDATA IN */
        set_gpio_ctrl(GPIO_E3 | GPIO_PULLUP_EN | GPIO_MODE_AC_SDATA_IN);
        /* GPE 4: AC97 SDATA OUT */
        set_gpio_ctrl(GPIO_E4 | GPIO_PULLUP_EN | GPIO_MODE_AC_SDATA_OUT);	
#endif 
#endif

	local_irq_restore(flags);

	ret = l3_start(bit);
	if (ret)
		l3_end();

	printk("GPIO L3 bus interface for elfin, installed\n");

	return ret;
}

static void __exit bus_exit(void)
{
	l3_end();
	printk("GPIO L3 bus interface for elfin, uninstalled\n");
}

module_init(bus_init);
module_exit(bus_exit);
