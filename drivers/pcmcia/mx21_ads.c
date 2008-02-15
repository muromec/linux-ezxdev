/* 
 * drivers/pcmcia/mx21_ads.c
 *
 * MX2ADS specific PCMCIA implementation routines
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>.
 *
 * This file is based on mx21.c from Motorola Dragonball MX2 ADS BSP
 * Copyright 2002, 2003 Motorola, Inc. All Rights Reserved.
 * Copyright (C) 2000 John G Dorsey <john+@cs.cmu.edu>
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/delay.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/arch/gpio.h>

#include "mx21_generic.h"
#include "mx21.h"

void *dev_id;

static void
mx21ads_pcmcia_gpio_config(void)
{
	/* configure output pins */
	mx2_register_gpios(PORT_F, 0x0000003f, GPIO | OUTPUT | OCR_A | PULLUP);

	/* configure input pins */
	mx2_register_gpios(PORT_F, 0x00007fc0, GPIO | INPUT | ICONFA_IN | PULLUP);

	/* SPKOUT signal */
	mx2_register_gpios(PORT_E, (1 << 5), GPIO | OUTPUT | OCR_A | PULLUP);

	/* 
	 * For cs4, cs5 which control output voltage 
	 * set cs5,cs4 to B'' 0x400000 = 3.3v 0x200000 = 5v 
	 */
	mx2_register_gpios(PORT_F, (1 << 22), 
			GPIO | OUTPUT | OCR_DATA | INIT_DATA_1 | PULLUP);

	mx2_register_gpios(PORT_F, (1 << 21), 
			GPIO | OUTPUT | OCR_DATA | INIT_DATA_0 | PULLUP);

#ifdef CONFIG_MX2TO2
	mx2_register_gpios(PORT_F, (1 << 16), 
			GPIO | OUTPUT | OCR_DATA | INIT_DATA_0 | PULLUP);
#endif

	/* delay 1s for charging external card */

	mdelay(100);

	DEBUG(2, "mx21ads_pcmcia_gpio_config:GPIOF_GIUS_V=0x%x\n", GPIO_GIUS(5));
	DEBUG(2, "mx21ads_pcmcia_gpio_config:GPIOF_DDIR_V=0x%x\n", GPIO_DDIR(5));
	DEBUG(2, "mx21ads_pcmcia_gpio_config:GPIOF_OCR1_V=0x%x\n", GPIO_OCR1(5));
	DEBUG(2, "mx21ads_pcmcia_gpio_config:GPIOF_OCR2_V=0x%x\n", GPIO_OCR2(5));
	DEBUG(2, "mx21ads_pcmcia_gpio_config:GPIOF_ICONFA1_V=0x%x\n", GPIO_ICONFA1(5));
	DEBUG(2, "mx21ads_pcmcia_gpio_config:GPIOF_ICONFA2_V=0x%x\n", GPIO_ICONFA2(5));
	DEBUG(2, "mx21ads_pcmcia_gpio_config:GPIOF_PUEN_V=0x%x\n", GPIO_PUEN(5));

	DEBUG(2, "mx21ads_pcmcia_gpio_config:GPIOE_GIUS_V=0x%x\n", GPIO_GIUS(4));
	DEBUG(2, "mx21ads_pcmcia_gpio_config:GPIOE_DDIR_V=0x%x\n", GPIO_DDIR(4));
	DEBUG(2, "mx21ads_pcmcia_gpio_config:GPIOE_OCR1_V=0x%x\n", GPIO_OCR1(4));
	DEBUG(2, "mx21ads_pcmcia_gpio_config:GPIOE_PUEN_V=0x%x\n", GPIO_PUEN(4));

#ifdef CONFIG_MX2TO1 /* These bits were removed in TO-2 */
	pgcr_write(PGCR_ATASEL, 0);
	pgcr_write(PGCR_ATAMODE, 0);
#endif
	pgcr_write(PGCR_LPMEN, 0);
	pgcr_write(PGCR_SPKREN, 0);
	pgcr_write(PGCR_POE, 1);
	/* soft reset card */
	pgcr_write(PGCR_RESET, 1);

	mdelay(100);

	pgcr_write(PGCR_RESET, 0);

	mdelay(100);
}

static void
mx21ads_pcmcia_irpt_config(void)
{
	/* Enable irq */
	per_write(PER_ERRINTEN, 1);
	per_write(PER_PWRONEN, 0);
	per_write(PER_RDYRE, 0);
	per_write(PER_RDYFE, 1);
	per_write(PER_RDYHE, 0);
	per_write(PER_RDYLE, 0);
	per_write(PER_BVDE2, 0);
	per_write(PER_BVDE1, 0);
	per_write(PER_CDE2, 1);
	per_write(PER_CDE1, 1);
	per_write(PER_WPE, 0);
	per_write(PER_VSE2, 0);
	per_write(PER_VSE1, 0);
}

static void
mx21ads_pcmcia_controller_init(void)
{
	int i;
	for (i = 0; i < MX21_PCMCIA_MAX_WINDOW; i++) {
		pbr_write(i, PBR_PBA, 0);
		por_write(i, POR_PV, 0);
		pofr_write(i, POFR_POFA, 0);
	}
}

static int
mx21ads_pcmcia_init(struct pcmcia_init *init)
{
	int res;

	DEBUG(2, "%s(): entered CRM_CSCR=0x%x \n", __FUNCTION__, CRM_CSCR);

	mx21ads_pcmcia_gpio_config();
	mx21ads_pcmcia_irpt_config();
	mx21ads_pcmcia_controller_init();

	res = request_irq(INT_PCMCIA, init->handler, SA_SHIRQ | SA_INTERRUPT,
			  "pcmcia", init->handler);
	if (res) {
		printk(KERN_ERR "%s: request for PCMCIA failed\n",
		       __FUNCTION__);
		return -1;
	}
	dev_id = init->handler;

	return 1;
}

/*
 * Release all resources.
 */
static int
mx21ads_pcmcia_shutdown(void)
{
	DEBUG(2, "%s(): Freeing irq\n", __FUNCTION__);

	free_irq(INT_PCMCIA, dev_id);
	return 0;
}

static int
mx21ads_pcmcia_socket_state(struct pcmcia_state_array *state)
{
	if (state->size > 1)
		return -1;

	state->state[0].poweron = pipr_read(PIPR_POWERON) ? 1 : 0;
	state->state[0].ready = pipr_read(PIPR_RDY) ? 1 : 0;
	state->state[0].bvd2 = pipr_read(PIPR_BVD2) ? 1 : 0;
	state->state[0].bvd1 = pipr_read(PIPR_BVD1) ? 1 : 0;
	state->state[0].detect = pipr_read(PIPR_CD) ? 0 : 1;
	state->state[0].wrprot = pipr_read(PIPR_WP) ? 1 : 0;

	state->state[0].vs_3v = 1;
	state->state[0].vs_Xv = 0;

	return 1;
}

static int
mx21ads_pcmcia_get_irq_info(struct pcmcia_irq_info *info)
{
	if (info->sock > MX21_PCMCIA_MAX_SOCK)
		return -1;

	info->irq = INT_PCMCIA;
	return 0;
}

static int
mx21ads_pcmcia_configure_socket(const struct pcmcia_configure *configure)
{
	return 0;
}

/*
 * Enable card status IRQs on (re-)initialisation.  This can
 * be called at initialisation, power management event, or
 * pcmcia event.
 */
static int
mx21ads_pcmcia_socket_init(int sock)
{
	DEBUG(2, "%s(): for sock %d\n", __FUNCTION__, sock);

	return 0;
}

/*
 * Disable card status IRQs on suspend.
 */
static int
mx21ads_pcmcia_socket_suspend(int sock)
{
	DEBUG(2, "%s() for sock %d\n", __FUNCTION__, sock);

	return 0;
}

struct pcmcia_low_level mx21ads_pcmcia_ops = {
	.init = mx21ads_pcmcia_init,
	.shutdown = mx21ads_pcmcia_shutdown,
	.socket_state = mx21ads_pcmcia_socket_state,
	.get_irq_info = mx21ads_pcmcia_get_irq_info,
	.configure_socket = mx21ads_pcmcia_configure_socket,

	.socket_init = mx21ads_pcmcia_socket_init,
	.socket_suspend = mx21ads_pcmcia_socket_suspend,
};
