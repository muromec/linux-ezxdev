/*
 *  cerf_ucb1400gpio.h
 *
 *  UCB1400 GPIO control stuff for the cerf.
 *
 *  Copyright (C) 2002 Intrinsyc Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    Mar 2002: Initial version [FB]
 * 
 */
/* -- lcd -- */
extern void cerf_ucb1400gpio_lcd_enable( void);
extern void cerf_ucb1400gpio_lcd_disable( void);
extern void cerf_ucb1400gpio_lcd_contrast_step( int direction);

/* -- irda -- */
extern void cerf_ucb1400gpio_irda_enable( void);
extern void cerf_ucb1400gpio_irda_disable( void);

/* -- bt -- */
extern void cerf_ucb1400gpio_bt_enable( void);
extern void cerf_ucb1400gpio_bt_disable( void);

/* -- init -- */
extern int cerf_ucb1400gpio_init(void);
