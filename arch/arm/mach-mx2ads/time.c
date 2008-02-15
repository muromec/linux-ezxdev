/*
 *  linux/arch/arm/mach-mx1ads/time.c
 *
 *  Copyright (C) 2000-2001 Deep Blue Solutions
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/config.h>
#include <linux/timex.h>

#include <asm/hardware.h>
#include <asm/io.h>

unsigned long mx2_clock_rate = 0;
unsigned long mx2_clock_divider = 0;
unsigned long mx2_cycles_per_jiffy = 0;

extern int (*set_rtc)(void);

/* FIXME-
 * When we have an external RTC part,
 * put the mapping in for that part.
 *
 * The internal RTC within the MX2 is not sufficient
 * for tracking time other than time of day, or
 * date over very short periods of time.
 *
 */

static int mx2ads_set_rtc(void)
{
	return 0;
}

static int mx2ads_rtc_init(void)
{
	xtime.tv_sec = 0;

	set_rtc = mx2ads_set_rtc;

	return 0;
}

EXPORT_SYMBOL(mx2_clock_rate);
EXPORT_SYMBOL(mx2_clock_divider);
EXPORT_SYMBOL(mx2_cycles_per_jiffy);

__initcall(mx2ads_rtc_init);

