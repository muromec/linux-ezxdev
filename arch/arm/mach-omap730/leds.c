/*
 * linux/arch/arm/mach-omap730/leds.c
 *
 * OMAP LEDs dispatcher
 *
 * Copyright 2003 by Texas Instruments Incorporated
 * OMAP730 / P2-sample additions
 * Author: Jean Pihet
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#include <linux/init.h>

#include <asm/leds.h>
#include <asm/mach-types.h>

extern void perseus2_leds_event(led_event_t);

static int __init
omap730_leds_init(void)
{
	if (machine_is_omap_perseus2())
		leds_event = perseus2_leds_event;

	leds_event(led_start);
	return 0;
}

__initcall(omap730_leds_init);
