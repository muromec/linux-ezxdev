/*
 *  linux/include/asm-arm/arch-mx2ads/time.h
 *
 *  Copyright (C) 2004 MontaVista Software Inc.
 *  Copyright (C) 2004 Motorola Semiconductors HK Ltd
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef __ASM_ARCH_TIME_H__
#define __ASM_ARCH_TIME_H__

#include <linux/config.h>
#include <asm/system.h>
#include <asm/leds.h>
#include <linux/timex.h>
#include <asm/arch/pll.h>

/*
 * Returns number of ms since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 */
static unsigned long
mx2_gettimeoffset(void)
{
	unsigned long ticks;

	/*
	 * Get the current number of ticks.  Note that there is a race
	 * condition between us reading the timer and checking for
	 * an interrupt.  We get around this by ensuring that the
	 * counter has not reloaded between our two reads.
	 */
	ticks = GPT_TCN(GPT1);

	/*
	 * Interrupt pending?  If so, we've reloaded once already.
	 */
	if (GPT_TSTAT(GPT1) & 1)
		ticks += mx2_cycles_per_jiffy;

	/*
	 * Convert the ticks to usecs
	 */

	return TICKS2USECS(ticks);
}

/*
 * IRQ handler for the timer
 */
static void
mx2_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	if (GPT_TSTAT(GPT1))
#ifdef CONFIG_MX2TO1
		GPT_TSTAT(GPT1) = 0;	/* clear interrupt */
#else
		GPT_TSTAT(GPT1) = 3;	/* clear interrupt */
#endif
	do_timer(regs);
	do_profile(regs);
}


/*
 * Set up timer interrupt, and return the current time in seconds.
 */
static inline void
setup_timer(void)
{
	u32 divider1, divider2;

	/*set up PERCLK1 and PERCLK2. These clock are shared among several modules
	PERCLK1 is used by UART, GPT, PWM
	PERCLK2 is used by MMC/SD, SPI
	Initialization of the peripheral clocks
	is placed here to make sure PERCLK1 is initialized
	before it is used by GPT*/

#if defined(CONFIG_MX2TO1)
	divider2 = divider1 = mx_module_get_clk(FCLK);
#else /*TO2*/
	divider2 = divider1 = mx_module_get_clk(MPLL);
#endif
	divider1 = divider1/MX2_PRECLK1_FREQ + ((divider1%MX2_PRECLK1_FREQ > ((u32)MX2_PRECLK1_FREQ >> 1)) ? 1 : 0) - 1;
	divider2 = divider2/MX2_PRECLK2_FREQ + ((divider2%MX2_PRECLK2_FREQ > ((u32)MX2_PRECLK2_FREQ >> 1)) ? 1 : 0) - 1;

#if defined(CONFIG_MX2TO1)
	CRM_PCDR = (CRM_PCDR & ~PCDR_PERDIV1 & ~PCDR_PERDIV2) | (divider1 << PCDR_PERDIV1_SHIFT) | (divider2 << PCDR_PERDIV2_SHIFT);
#else /*TO2*/
	CRM_PCDR1 = (CRM_PCDR1 & ~PCDR1_PERDIV1 & ~PCDR1_PERDIV2) | (divider1 << PCDR1_PERDIV1_SHIFT) | (divider2 << PCDR1_PERDIV2_SHIFT);
#endif
	divider1 = mx_module_get_clk(PERCLK1);
	divider2 = divider1/CLOCK_TICK_RATE;  /*try to match*/
	mx2_clock_rate = divider1/divider2;
	mx2_clock_divider = divider2;


#if defined (CONFIG_PREEMPT_TIMES) || defined (CONFIG_ILATENCY) || defined (CONFIG_KFI)
	mx_module_clk_open(IPG_MODULE_GPT3);

	GPT_TCTL(GPT3) = 0;

        /* Set up a free-running timer */
	GPT_TPRER(GPT3) = mx2_clock_divider - 1;
	GPT_TCMP(GPT3) = 0xffffffff; /*clear the counter*/
	GPT_TCTL(GPT3) = (TIM_PERCLK1 | TIM_FRR | TIM_ENAB);
#endif

	mx2_cycles_per_jiffy = mx2_clock_rate/HZ;
	
	mx_module_clk_open(IPG_MODULE_GPT1);

	GPT_TCTL(GPT1) = 0;
	GPT_TPRER(GPT1) = mx2_clock_divider - 1;
	GPT_TCMP(GPT1) = mx2_cycles_per_jiffy - 1;
	GPT_TCTL(GPT1) = (TIM_PERCLK1 | TIM_INTEN | TIM_ENAB);

	timer_irq.handler = mx2_timer_interrupt;

	/* Make irqs happen for the system timer */
	setup_arm_irq(INT_GPT1, &timer_irq);
	gettimeoffset = mx2_gettimeoffset;

}

#endif
