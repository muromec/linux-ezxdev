/*
 *  include/asm-arm/arch-omap730/keyboard.h
 *
 *  Initially based on linux/include/asm-arm/arch-sa1100/keyboard.h
 *  Copyright (c) 2004 MPC-Data Limited
 *    Dave Peverley <dpeverley@mpc-data.co.uk>
 *
 *  This file contains the OMAP architecture specific keyboard definitions
 */
#ifndef _OMAP730_KEYBOARD_H
#define _OMAP730_KEYBOARD_H

#include <linux/config.h>
#include <asm/mach-types.h>

#define kbd_disable_irq()	  do { } while(0)
#define kbd_enable_irq()	  do { } while(0)

#define kbd_setkeycode(sc,kc)     (-EINVAL)
#define kbd_getkeycode(sc)        (-EINVAL)

#define k_translate(sc, kcp, rm)  ({ *(kcp) = (sc); 1; })
#define k_unexpected_up(kc)       (0200)
#define kbd_leds(leds)            do {} while (0)
#define kbd_init_hw()             do {} while (0)
#define k_sysrq_xlate             ((unsigned char *)NULL)

#define k_sysrq_key		  13


#endif  /* _OMAP730_KEYBOARD_H */
