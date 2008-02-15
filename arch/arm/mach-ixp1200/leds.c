/*
 * arch/arm/kernel/leds-ixp1200.c
 *
 * Copyright (C) 1998-1999 Russell King
 *
 * Mar-27-2000 Uday Naik    Created
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <asm/hardware.h>
#include <asm/leds.h>
#include <asm/spinlock.h>
#include <asm/system.h>
#include <asm/arch/ixp1200eb.h>

/* mask interrupts and set the led */

void ixp1200evb_leds_event(led_event_t ledevt)
{

  unsigned long flags;
  volatile unsigned int* ledreg = CSR_LED;

  save_flags_cli(flags);
  *ledreg = (unsigned int) ledevt;
  restore_flags(flags);

}

void (*leds_event)(led_event_t) = ixp1200evb_leds_event;

EXPORT_SYMBOL(leds_event);
