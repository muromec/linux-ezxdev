/*
 *  linux/arch/arm/mach-pxa/generic.c
 *
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 * 
 * Code common to all PXA machines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Since this file should be linked before any other machine specific file,
 * the __initcall() here will be executed first.  This serves as default
 * initialization stuff for PXA machines which can be overriden later if
 * need be.
 */
/*
 * Copyright (C) 2004 Motorola Inc.
 *
 * 2004-Jan-02 Modify __initdata for BULVERDE ssp3 register, Jin Lihong
 *                               
 */
 
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>

#include <asm/hardware.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>

#include "generic.h"

#ifdef CONFIG_CPU_BULVERDE

/* Crystal clock : 13-MHZ*/
#define BASE_CLK	13000000

/*
 * Get the clock frequency as reflected by CCSR and the turbo flag.
 * We assume these values have been applied via a fcs.
 * If info is not 0 we also display the current settings.
 *
 * For more details, refer to Bulverde Manual, section 3.8.2.1
 */
unsigned int get_clk_frequency_khz( int info)
{
	unsigned long ccsr, turbo, b, ht;
	unsigned int l, L, m, M, n2, N, S, cccra;

	ccsr = CCSR;
	cccra = CCCR & (0x1 << 25);

	/* Read clkcfg register: it has turbo, b, half-turbo (and f) */
	asm( "mrc\tp14, 0, %0, c6, c0, 0" : "=r" (turbo) );
	b = (turbo & (0x1 << 3));
	ht = (turbo & (0x1 << 2));

	l  =  ccsr & 0x1f;
	n2 =  (ccsr>>7) & 0xF;
	if (l == 31) {
		/* The calculation from the Yellow Book is incorrect:
		   it says M=4 for L=21-30 (which is easy to calculate
		   by subtracting 1 and then dividing by 10, but not
		   with 31, so we'll do it manually */
		m = 1 << 2;
	}
	else {
		m  =  1<<( (l-1)/10 );
	}

	L = l * BASE_CLK;
	N = (n2 * L) / 2;
	S = (b) ? L : L/2;
	if (cccra == 0)
		M = L/m;
	else
		M = (b) ? L : L/2;

	if( info) {
		printk( KERN_INFO "Run Mode clock: %d.%02dMHz (*%d)\n",
			L / 1000000, (L % 1000000) / 10000, l );
		printk( KERN_INFO "Memory clock: %d.%02dMHz (/%d)\n",
			M / 1000000, (M % 1000000) / 10000, m );
		printk( KERN_INFO "Turbo Mode clock: %d.%02dMHz (*%d.%d, %sactive)\n",
			N / 1000000, (N % 1000000)/10000, n2 / 2, (n2 % 2)*5,
			(turbo & 1) ? "" : "in" );
		printk( KERN_INFO "System bus clock: %d.%02dMHz \n",
			S / 1000000, (S % 1000000) / 10000 );
	}

	return (turbo & 1) ? (N/1000) : (L/1000);
}

/*
 * Return the current mem clock frequency in units of 10kHz as
 * reflected by CCCR[A], B, and L
 */
unsigned int get_memclk_frequency_10khz(void)
{
	unsigned long ccsr, clkcfg, b;
	unsigned int l, L, m, M, cccra;

	cccra = CCCR & (0x1 << 25);

	/* Read clkcfg register to obtain b */
	asm( "mrc\tp14, 0, %0, c6, c0, 0" : "=r" (clkcfg) );
	b = (clkcfg & (0x1 << 3));

	ccsr = CCSR;
	l  =  ccsr & 0x1f;
	if (l == 31) {
		/* The calculation from the Yellow Book is incorrect:
		   it says M=4 for L=21-30 (which is easy to calculate
		   by subtracting 1 and then dividing by 10, but not
		   with 31, so we'll do it manually */
		m = 1 << 2;
	}
	else {
		m  =  1<<( (l-1)/10 );
	}

	L = l * BASE_CLK;
	if (cccra == 0)
		M = L/m;
	else
		M = (b) ? L : L/2;

	return (M / 10000);
}

#else

/*
 * Various clock factors driven by the CCCR register.
 */

/* Crystal Frequency to Memory Frequency Multiplier (L) */
static unsigned char L_clk_mult[32] = { 0, 27, 32, 36, 40, 45, 0, };

/* Memory Frequency to Run Mode Frequency Multiplier (M) */
static unsigned char M_clk_mult[4] = { 0, 1, 2, 4 };

/* Run Mode Frequency to Turbo Mode Frequency Multiplier (N) */
/* Note: we store the value N * 2 here. */
static unsigned char N2_clk_mult[8] = { 0, 0, 2, 3, 4, 0, 6, 0 };

/* Crystal clock */
#define BASE_CLK	3686400

/*
 * Get the clock frequency as reflected by CCCR and the turbo flag.
 * We assume these values have been applied via a fcs.
 * If info is not 0 we also display the current settings.
 */
unsigned int get_clk_frequency_khz( int info)
{
	unsigned long cccr, turbo;
	unsigned int l, L, m, M, n2, N;

	cccr = CCCR;
	asm( "mrc\tp14, 0, %0, c6, c0, 0" : "=r" (turbo) );

	l  =  L_clk_mult[(cccr >> 0) & 0x1f];
	m  =  M_clk_mult[(cccr >> 5) & 0x03];
	n2 = N2_clk_mult[(cccr >> 7) & 0x07];

	L = l * BASE_CLK;
	M = m * L;
	N = n2 * M / 2;

	if( info)
	{
		L += 5000;
		printk( KERN_INFO "Memory clock: %d.%02dMHz (*%d)\n",
			L / 1000000, (L % 1000000) / 10000, l );
		M += 5000;
		printk( KERN_INFO "Run Mode clock: %d.%02dMHz (*%d)\n",
			M / 1000000, (M % 1000000) / 10000, m );
		N += 5000;
		printk( KERN_INFO "Turbo Mode clock: %d.%02dMHz (*%d.%d, %sactive)\n",
			N / 1000000, (N % 1000000) / 10000, n2 / 2, (n2 % 2) * 5,
			(turbo & 1) ? "" : "in" );
	}

	return (turbo & 1) ? (N/1000) : (M/1000);
}

/*
 * Return the current mem clock frequency in units of 10kHz as
 * reflected by CCCR[A], B, and L
 */
unsigned int get_memclk_frequency_10khz(void)
{
	return L_clk_mult[(CCCR >> 0) & 0x1f] * BASE_CLK / 10000;
}

#endif

EXPORT_SYMBOL(get_clk_frequency_khz);
EXPORT_SYMBOL(get_memclk_frequency_10khz);

/*
 * Handy function to set GPIO alternate functions
 */

void set_GPIO_mode(int gpio_mode)
{
	long flags;
	int gpio = gpio_mode & GPIO_MD_MASK_NR;
	int fn = (gpio_mode & GPIO_MD_MASK_FN) >> 8;
	int gafr;

	local_irq_save(flags);
	if (gpio_mode & GPIO_MD_MASK_DIR)
		GPDR(gpio) |= GPIO_bit(gpio);
	else
		GPDR(gpio) &= ~GPIO_bit(gpio);
	gafr = GAFR(gpio) & ~(0x3 << (((gpio) & 0xf)*2));
	GAFR(gpio) = gafr |  (fn  << (((gpio) & 0xf)*2));
	local_irq_restore(flags);
}

EXPORT_SYMBOL(set_GPIO_mode);

/* 
 * Note that 0xfffe0000-0xffffffff is reserved for the vector table and
 * cache flush area.
 */
#ifdef CONFIG_CPU_BULVERDE
static struct map_desc standard_io_desc[] __initdata = {
 /* virtual     physical    length      domain     r  w  c  b */
  { 0xe6000000, 0x20000000, 0x01000000, DOMAIN_IO, 0, 1, 0, 0 }, /* PCMCIA0 IO */
  { 0xe7000000, 0x30000000, 0x01000000, DOMAIN_IO, 0, 1, 0, 0 }, /* PCMCIA1 IO */
  { 0xe8000000, 0x40000000, 0x02000000, DOMAIN_IO, 0, 1, 0, 0 }, /* Devs */
  { 0xea000000, 0x44000000, 0x00100000, DOMAIN_IO, 0, 1, 0, 0 }, /* LCD */
  { 0xec000000, 0x48000000, 0x00100000, DOMAIN_IO, 0, 1, 0, 0 }, /* Mem Ctl */
  { 0xee000000, 0x4C000000, 0x00100000, DOMAIN_IO, 0, 1, 0, 0 }, /* USB host */
  { 0xf8000000, 0x50000000, 0x00100000, DOMAIN_IO, 0, 1, 0, 0 }, /* Camera */
  { 0xfc000000, 0x58000000, 0x00100000, DOMAIN_IO, 0, 1, 0, 0 }, /* IMem ctl */
  { 0xfe000000, 0x5c000000, 0x00100000, DOMAIN_IO, 0, 1, 0, 1 }, /* IMem */
  { 0xff000000, 0x00000000, 0x00100000, DOMAIN_IO, 0, 1, 0, 0 }, /* UNCACHED_PHYS_0 */
  LAST_DESC
};
#else
static struct map_desc standard_io_desc[] __initdata = {
 /* virtual     physical    length      domain     r  w  c  b */
  { 0xf6000000, 0x20000000, 0x01000000, DOMAIN_IO, 0, 1, 0, 0 }, /* PCMCIA0 IO */
  { 0xf7000000, 0x30000000, 0x01000000, DOMAIN_IO, 0, 1, 0, 0 }, /* PCMCIA1 IO */
  { 0xf8000000, 0x40000000, 0x01800000, DOMAIN_IO, 0, 1, 0, 0 }, /* Devs */
  { 0xfa000000, 0x44000000, 0x00100000, DOMAIN_IO, 0, 1, 0, 0 }, /* LCD */
  { 0xfc000000, 0x48000000, 0x00100000, DOMAIN_IO, 0, 1, 0, 0 }, /* Mem Ctl */
  { 0xff000000, 0x00000000, 0x00100000, DOMAIN_IO, 0, 1, 0, 0 }, /* UNCACHED_PHYS_0 */
  LAST_DESC
};
#endif

void __init pxa_map_io(void)
{
	iotable_init(standard_io_desc);
	get_clk_frequency_khz( 1);
}
