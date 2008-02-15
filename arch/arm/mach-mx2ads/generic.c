/*
 * linux/arch/arm/mach-mx2ads/generic.c
 *
 * Code common to all i.MX21 machines.
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>
 *
 * Portions of this code were initially based on 
 * linux/arch/arm/mach-sa1100/generic.c and
 * linux/arch/arm/mach-pxa/cpu-pxa.c
 *
 * Copyright (c) Nicolas Pitre <nico@cam.org>
 * Copyright (C) 2002,2003 Intrinsyc Software
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>

#include <asm/hardware.h>
#include <asm/arch/pll.h>

#define MFN(x)	((x) & 0x3ff)
#define MFI(x)	(((x) & (0xf << 10)) >> 10)
#define MFD(x)	(((x) & (0x3ff << 16)) >> 16)
#define PD(x)	(((x) & (0xf << 26)) >> 26)

#if defined(OSC_32000HZ_INPUT)
#define XTAL_LOW_FREQ	32000
#elif defined(OSC_32768HZ_INPUT)
#define XTAL_LOW_FREQ	32768
#else
#define XTAL_LOW_FREQ	0
#endif

#ifdef CONFIG_MX21_TVOUT
#define XTAL_HIGH_FREQ	(FS453_MPLLIN_FREQ)/* When TVOUT is enabled the FS453
                                              ouput this clock to external High
                                              Freq clock */

#else
#define XTAL_HIGH_FREQ	(26000000)
#endif

/*
 * Return the current MPLL clock frequency in units of Hz
 */
unsigned int 
mx21_get_mpll_frequency(void)
{
	unsigned long long int dpll;
        u32 fref = (CRM_CSCR & CSCR_MCU_SEL) ? XTAL_HIGH_FREQ : 
			512 * XTAL_LOW_FREQ;	
	dpll =  (unsigned long long int)(2 * fref) * 
		((MFI(CRM_MPCTL0) * (MFD(CRM_MPCTL0) + 1) +  MFN(CRM_MPCTL0))) / 
		((MFD(CRM_MPCTL0) + 1) * (PD(CRM_MPCTL0) + 1));

	return (unsigned int) dpll;
}

/*
 * Return the current SPLL clock frequency in units of Hz
 */
unsigned int 
mx21_get_spll_frequency(void)
{
	unsigned long long int dpll;
	u32 fref = (CRM_CSCR & CSCR_SP_SEL) ? XTAL_HIGH_FREQ : 
		512 * XTAL_LOW_FREQ;	
	
	dpll =  (unsigned long long int)(2 * fref) * 
		((MFI(CRM_MPCTL0) * (MFD(CRM_MPCTL0) + 1) +  MFN(CRM_MPCTL0))) / 
		((MFD(CRM_MPCTL0) + 1) * (PD(CRM_MPCTL0) + 1));
	
	return (unsigned int) dpll;
}

/*
 * Get the clock frequency.
 * If info is not 0 we also display the current settings.
 */
unsigned int 
get_clk_frequency_khz(int info)
{
	unsigned int pll_freq = mx21_get_mpll_frequency();

	if (info) {
		unsigned int fclk = pll_freq / (((CRM_CSCR & CSCR_PRESC) >> CSCR_PRESC_SHIFT) + 1);
		unsigned int hclk = fclk / (((CRM_CSCR & CSCR_BCLKDIV) >> CSCR_BCLKDIV_SHIFT) + 1);

		printk(KERN_INFO "MPLL clock: %d.%06dMHz FCLK: %d.%06dMHz "
			"System bus clock (HCLK): %d.%06dMHz\n", 
			pll_freq / 1000000, pll_freq % 1000000, 
			fclk / 1000000, fclk % 1000000, 
			hclk / 1000000, hclk % 1000000) ;
	}
	
	return (pll_freq / (((CRM_CSCR & CSCR_PRESC) >> CSCR_PRESC_SHIFT) + 1)) / 1000;
}


EXPORT_SYMBOL(mx21_get_mpll_frequency);
EXPORT_SYMBOL(mx21_get_spll_frequency);
EXPORT_SYMBOL(get_clk_frequency_khz);
