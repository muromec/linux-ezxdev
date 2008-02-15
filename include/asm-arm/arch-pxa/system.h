/*
 * linux/include/asm-arm/arch-pxa/system.h
 *
 * Author:	Nicolas Pitre
 * Created:	Jun 15, 2001
 * Copyright:	MontaVista Software Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 ************************************ history *****************************
 *    author          date         CR          title
 *    weiqiang lin    2005-06-30  LIBgg39140   update pcap interface to spi mode
 */

#include "hardware.h"
#include "timex.h"
//#include "../../../drivers/misc/ssp_pcap.h"
#ifdef CONFIG_MOT_POWER_IC
#include <linux/power_ic.h>
#endif
static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode)
{
	if (mode == 's') {
		/* Jump into ROM at address 0 */
		cpu_reset(0);
	} else {
#ifdef CONFIG_ARCH_EZX
		extern void pcap_switch_off_usb(void);
		extern void pcap_switch_on_usb(void);

		*(unsigned long *)(phys_to_virt(BPSIG_ADDR)) = NO_FLAG;
#ifdef CONFIG_MOT_POWER_IC
		power_ic_periph_set_usb_pull_up(POWER_IC_PERIPH_OFF);
		power_ic_periph_set_flash_card_on(POWER_IC_PERIPH_OFF);
		mdelay(1000);//e12707 DEC 21 - for restart, disconnect time should be above 2 sec, otherwise unknow device appear on windows
		power_ic_periph_set_usb_pull_up(POWER_IC_PERIPH_ON);
		power_ic_periph_set_flash_card_on(POWER_IC_PERIPH_ON);
#endif
		if (!(GPLR(GPIO_BB_WDI) & GPIO_bit(GPIO_BB_WDI))) {
			*(unsigned long *)(phys_to_virt(BPSIG_ADDR)) = WDI_FLAG;
		}
		GPCR(GPIO_BB_RESET) = GPIO_bit(GPIO_BB_RESET);
#endif
		/* Initialize the watchdog and let it fire */
		OWER = OWER_WME;
		OSSR = OSSR_M3;
		OSMR3 = OSCR + CLOCK_TICK_RATE/100;	/* ... in 10 ms */
#ifdef CONFIG_ARCH_EZX
                MDREFR |= MDREFR_SLFRSH;
#endif
		while(1);
	}
}

