/*
 * linux/arch/arm/mach-omap730/leds-perseus2.c
 *
 * Copyright 2003 by Texas Instruments Incorporated
 * OMAP730 / P2-sample additions
 * Author: Jean Pihet
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/kernel_stat.h>
#include <linux/sched.h>

#include <asm/hardware.h>
#include <asm/leds.h>
#include <asm/system.h>
#include <asm/arch/perseus2.h>


/* LEDs definition on debug board (16 LEDs) */

#define PERSEUS2_FPGA_LED_CLAIMRELEASE   (1 << 15)
#define PERSEUS2_FPGA_LED_STARTSTOP      (1 << 14)
#define PERSEUS2_FPGA_LED_HALTED         (1 << 13)
#define PERSEUS2_FPGA_LED_IDLE           (1 << 12)
#define PERSEUS2_FPGA_LED_TIMER          (1 << 11)

/* cpu0 load-meter LEDs */

#define PERSEUS2_FPGA_LOAD_METER         (1 << 0)        // A bit of fun on our board ...
#define PERSEUS2_FPGA_LOAD_METER_SIZE    11
#define PERSEUS2_FPGA_LOAD_METER_MASK    ((1 << PERSEUS2_FPGA_LOAD_METER_SIZE) - 1)





void perseus2_leds_event(led_event_t evt)
{
	unsigned long flags;
	unsigned long hw_led_load_meter, activity;
	static unsigned long last_jiffies = 0, last_activity = 0;
	static unsigned long hw_led_state = 0;


	local_irq_save(flags);

	switch (evt) {
	case led_start:
		hw_led_state |= PERSEUS2_FPGA_LED_STARTSTOP;
		break;

	case led_stop:
		hw_led_state &= ~PERSEUS2_FPGA_LED_STARTSTOP;
		break;

	case led_claim:
		hw_led_state |= PERSEUS2_FPGA_LED_CLAIMRELEASE;
		break;

	case led_release:
		hw_led_state &= ~PERSEUS2_FPGA_LED_CLAIMRELEASE;
		break;

#ifdef CONFIG_LEDS_TIMER
	case led_timer:
		/*
		 * Toggle Timer LED
		 */
		if (hw_led_state & PERSEUS2_FPGA_LED_TIMER)
			hw_led_state &= ~PERSEUS2_FPGA_LED_TIMER;
		else
			hw_led_state |= PERSEUS2_FPGA_LED_TIMER;

#ifdef CONFIG_LEDS_CPU
		/*
	 	* Display Load Meter for CPU0
	 	*/
		// Calculate CPU0 load (in %)
		//  Method: /proc/stat alike
		activity = kstat.per_cpu_user[0] + kstat.per_cpu_nice[0] + kstat.per_cpu_system[0];
	        hw_led_load_meter = ((activity - last_activity) * 100) / (jiffies - last_jiffies);

		last_activity = activity;
		last_jiffies = jiffies;

		if (hw_led_load_meter > 98)	// 98: trick to burn the last LED as well
			hw_led_load_meter = 100;

		// Calculate the number of LEDs to burn (VU Meter) from percentage
		hw_led_load_meter = (1 << ((hw_led_load_meter * PERSEUS2_FPGA_LOAD_METER_SIZE) / 100)) - 1;
	
		// Update load LEDs
		hw_led_state &= ~(PERSEUS2_FPGA_LOAD_METER_MASK * PERSEUS2_FPGA_LOAD_METER);
		hw_led_state |= (hw_led_load_meter & PERSEUS2_FPGA_LOAD_METER_MASK) * PERSEUS2_FPGA_LOAD_METER;
		break;
#endif
#endif


#ifdef CONFIG_LEDS_CPU
	case led_idle_start:
		hw_led_state |= PERSEUS2_FPGA_LED_IDLE;
		break;

	case led_idle_end:
		hw_led_state &= ~PERSEUS2_FPGA_LED_IDLE;
		break;
#endif

	case led_halted:
		if (hw_led_state & PERSEUS2_FPGA_LED_HALTED)
			hw_led_state &= ~PERSEUS2_FPGA_LED_HALTED;
		else
			hw_led_state |= PERSEUS2_FPGA_LED_HALTED;
		break;

	/* Not Implemented */
	case led_green_on:
	case led_green_off:
	case led_amber_on:
	case led_amber_off:
	case led_red_on:
	case led_red_off:
	default:
		break;
	}


	/*
	 *  Actually burn the LEDs
	 */
	*((unsigned short *) (PERSEUS2_FPGA_LEDS)) = (unsigned short) (~hw_led_state & 0xFFFF);	
	
	local_irq_restore(flags);
}
