/*
 *  File: arch/arm/mach-omap730/ck.c
 *
 *  Implements the clock interface for the omap730
 *
 *  Created 2001, Copyright (C) 2001 RidgeRun, Inc.  All rights reserved.
 *  Author: Gordon McNutt <gmcnutt@ridgerun.com>
 *
 *  Modified 2004, Copyright (C) 2004 MPC-Data Limited.  All rights reserved.
 *  Dave Peverley <dpeverley@mpc-data.co.uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/string.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/bitops.h>
#include <asm/arch/ck.h>
#include <asm/arch/timer.h>

#undef DEBUG

#ifdef DEBUG
#define DPRINT(args...) printk(args)
#else
#define DPRINT(args...) do {} while (0)
#endif


/*
 * A little detail ; In the OMAP730, the main clock CK_REF is supplied from an
 * external pin (CLKIN) at 13MHz, and then split via DPLL1 into three seperate 
 * clock sources :
 *   CK_GEN1, CK_GEN2, CK_GEN3 
 *
 * It should be noted that these three sources feed three independent clock
 * domains, MPU, DSP-MMU and TC respectively. 
 *
 * After the Clock generators, the three clock generators feed the clock
 * off to other subsystems within their domain. Various subsystems can 
 * have their rate further divided, idled or even disabled depending on
 * available features.
 */


static ck_info_t ck_info_table[] = {
	{
		.name = "clkin",
		.flags = 0,
		.parent = clkin,
	}, {
		.name = "ck_gen1",
		.flags = CK_RATEF | CK_IDLEF,
		.rreg = DPLL1_CTL_REG,
		.ireg = ARM_IDLECT1,
		/* Note - theres no rshift as this is not used. */
		.ishift = IDLDPLL_ARM,
		.parent = clkin,
	}, {
		.name = "ck_gen2",
		.flags = 0,
		.parent = ck_gen1,
	}, {
		.name = "ck_gen3",
		.flags = 0,
		.parent = ck_gen1,
	}, {
		.name = "tc_ck",
		.flags = CK_RATEF | CK_IDLEF,
		.parent = ck_gen3,
		.rreg = ARM_CKCTL,
		.ireg = ARM_IDLECT1,
		.rshift = TCDIV,
		.ishift = IDLIF_ARM
	}, {
		.name = "arm_ck",
		.flags = CK_RATEF,
		.parent = ck_gen1,
		.rreg = ARM_CKCTL,
		.rshift = ARMDIV,
	}, {
		.name = "mpuper_ck",
		.flags = CK_RATEF | CK_IDLEF | CK_ENABLEF,
		.parent = ck_gen1,
		.rreg = ARM_CKCTL,
		.ereg = ARM_IDLECT2,
		.ireg = ARM_IDLECT1,
		.rshift = PERDIV,
		.eshift = EN_PERCK,
		.ishift = IDLPER_ARM
	}, {
		.name = "mpuxor_ck",
		.flags = CK_ENABLEF | CK_IDLEF,
		.parent = clkin,
		.ireg = ARM_IDLECT1,
		.ereg = ARM_IDLECT2,
		.ishift = IDLXORP_ARM,
		.eshift = EN_XORPCK
	}, {
		.name = "mputim_ck",
		.flags = CK_IDLEF | CK_ENABLEF | CK_SELECTF,
		.parent = clkin,
		.ireg = ARM_IDLECT1,
		.ereg = ARM_IDLECT2,
		.sreg = ARM_CKCTL,
		.ishift = IDLTIM_ARM,
		.eshift = EN_TIMCK,
		.sshift = ARM_TIMXO
	}, {
		.name = "mpuwd_ck",
		.flags = CK_IDLEF | CK_ENABLEF,
		.parent = clkin,
		.ireg = ARM_IDLECT1,
		.ereg = ARM_IDLECT2,
		.ishift = IDLWDT_ARM,
		.eshift = EN_WDTCK,
	}, {
		.name = "dsp_ck",
		.flags = CK_RATEF,
		.parent = ck_gen2,
		.rreg = ARM_CKCTL,
		.rshift = DSPDIV,
	}, {
		.name = "dspmmu_ck",
		.flags = CK_RATEF,
		.parent = ck_gen2,
		.rreg = ARM_CKCTL,
		.rshift = DSPMMUDIV,
	}, {
		.name = "dma_ck",
		.flags = CK_RATEF | CK_IDLEF | CK_ENABLEF,
		.parent = ck_gen3,
		.rreg = ARM_CKCTL,
		.ireg = ARM_IDLECT1,
		.ereg = ARM_IDLECT2,
		.rshift = TCDIV,
		.ishift = IDLIF_ARM,
		.eshift = DMACK_REQ
	}, {
		.name = "api_ck",
		.flags = CK_RATEF | CK_ENABLEF,
		.parent = ck_gen3,
		.rreg = ARM_CKCTL,
		.ereg = ARM_IDLECT2,
		.rshift = TCDIV,
		.eshift = EN_APICK,
	}, {
		.name = "lcd_ck",
		.flags = CK_RATEF | CK_ENABLEF,
		.parent = ck_gen3,
		.rreg = ARM_CKCTL,
		.ereg = ARM_IDLECT2,
		.rshift = LCDDIV,
		.eshift = EN_LCDCK,
	},
};

static int ck_debug;
static volatile unsigned long ck_valid_table[CK_MAX_PLL_FREQ / 32 + 1];
static __u8 ck_lookup_table[CK_MAX_PLL_FREQ];

#ifdef CONFIG_INSTANT_ON
unsigned long instant_on_lpj = CLOCK_INSTANT_ON_LPJ;
#endif

/* __ck_set_input()
 *
 * For some clocks you have a choice of which "parent" clocks they are 
 * derived from. Use this to select a "parent". See the platform 
 * documentation for valid combinations. 
 */
static int
__ck_set_input(ck_t ck, ck_t input)
{
	volatile __u16 *reg;
	int shift;

	if (!CK_IN_RANGE(ck) || !CK_CAN_SWITCH(ck))
		return -EINVAL;

	reg = CK_SELECT_REG(ck);
	shift = CK_SELECT_SHIFT(ck);

	if (input == clkin) {
		*((volatile __u16 *) reg) &= ~(1 << shift);
		return 0;
	}
	else if (input == CK_PARENT(ck)) {
		*((volatile __u16 *) reg) |= (1 << shift);
		return 0;
	}
	return -EINVAL;
}

/* __ck_get_input()
 *
 * For some clocks you have a choice of which "parent" clocks they are 
 * derived from. Use this to find the "parent". See the platform 
 * documentation for valid combinations. 
 */
static int
__ck_get_input(ck_t ck, ck_t * input)
{
	if (!CK_IN_RANGE(ck))
		return -EINVAL;
	if (CK_CAN_SWITCH(ck)) {
		int shift;
		volatile __u16 *reg;

		reg = CK_SELECT_REG(ck);
		shift = CK_SELECT_SHIFT(ck);
		if (*reg & (1 << shift)) {
			*input = CK_PARENT(ck);
			return 0;
		}
	}
	*input = clkin;
	return 0;
}

/* __ck_set_pll_rate()
 *
 * This is the actual implementation to set the rate of the clocks 
 * that use DPLL1 as their source.
 * 
 * ( see __ck_set_rate() )
 * 
 */
static int
__ck_set_pll_rate(ck_t ck, int rate)
{
	volatile __u16 *pll;

	if ((rate < 0) || (rate > CK_MAX_PLL_FREQ))
		return -EINVAL;

	/* scan downward for the closest matching frequency */
	while (rate && !test_bit(rate, ck_valid_table))
		rate--;

	if (!rate) {
		printk(KERN_ERR "%s: couldn't find a matching rate\n",
			__FUNCTION__);
		return -EINVAL;
	}

	pll = (volatile __u16 *) CK_RATE_REG(ck);

	/* clear the rate bits */
	*pll &= ~(DPLL1_CTL_PLL_MULT_MSK << DPLL1_CTL_PLL_MULT_POS);

	/* 
	 * TODO : This is rubbish as it doesn't take into account the
	 *        divide value, only the mult.
	 */
	*pll |= (ck_lookup_table[rate - 1] << DPLL1_CTL_PLL_DIV_POS);

	return 0;
}

/* __ck_set_clkm_rate()
 *
 * This is the actual implementation to set the rate of the clkm clocks.
 * 
 * ( see __ck_set_rate() )
 */
static int
__ck_set_clkm_rate(ck_t ck, int rate)
{
	int shift, prate, div;
	volatile __u16 *reg;

	/* Now. We can only set this clock's value to a fraction of its
	   parent's value. The interface says I'll round down when necessary.
	   So first let's get the parent's current rate. */
	prate = ck_get_rate(CK_PARENT(ck));

	/* Now let's just start with the highest fraction and keep searching
	   down through available rates until we find one less than or equal
	   to the desired rate. */
	for (div = 0; div < 4; div++) {
		if (prate <= rate)
			break;
		prate = prate / 2;
	}

	/* Oops. Looks like the caller wants a rate lower than we can support.
	   Sorry, sucker. */
	if (div >= 4) {
		printk(KERN_ERR "%s: %d is too low\n", __FUNCTION__, rate);
		return -EINVAL;
	}

	/* One more detail: if this clock supports more than one parent, then
	   we're going to automatically switch over to the parent which runs
	   through the divisor. For omap this is not ambiguous because for all
	   such clocks one choice is always CLKIN (which doesn't run through
	   the divisor) and the other is whatever I encoded as CK_PARENT. Note
	   that I wait until we get this far because I don't want to switch
	   the input until we're sure this is going to work. */
	if (CK_CAN_SWITCH(ck))
		BUG_ON (__ck_set_input(ck, CK_PARENT(ck)) < 0);
	
	/* Now, at last, we can set the divisor. Clear the old rate bits and
	   set the new ones. */
	reg = (volatile __u16 *) CK_RATE_REG(ck);
	shift = CK_RATE_SHIFT(ck);
	*reg &= ~(ARM_CKCTL_DIV_MSK << shift);
	*reg |= (div << shift);

	/* And return the new (actual, after rounding down) rate. */
	return prate;
}

/* ck_set_rate()
 *
 * Use this to set a clock rate. If other clocks are derived from this one, 
 * their rates will all change too. If this is a derived clock and I can't 
 * change it to match your request unless I also change the parent clock, then
 * tough luck -- I won't change the parent automatically. I'll return an error
 * if I can't get the clock within 10% of what you want. Otherwise I'll return
 * the value I actually set it to. If I have to switch parents to get the rate
 * then I will do this automatically (since it only affects this clock and its
 * descendants). 
 */
int
ck_set_rate(ck_t ck, int rate)
{
	if (!CK_IN_RANGE(ck) || !CK_CAN_CHANGE_RATE(ck))
		return -EINVAL;
	return (ck == ck_gen1) ? __ck_set_pll_rate(ck, rate) :
		__ck_set_clkm_rate(ck, rate);
}

/* __ck_get_pll_rate()
 *
 * Gets the rate of the clock for clocks that use DPLL1 for their source.
 * 
 * ( see ck_get_rate() )
 */
static int
__ck_get_pll_rate(ck_t ck)
{
	int m, d;

	__u16 pll = *((volatile __u16 *) CK_RATE_REG(ck));

	m = (pll & (DPLL1_CTL_PLL_MULT_MSK << DPLL1_CTL_PLL_MULT_POS)) 
		>> DPLL1_CTL_PLL_MULT_POS;
	m = m ? m : 1;
	d = (pll & (DPLL1_CTL_PLL_DIV_MSK << DPLL1_CTL_PLL_DIV_POS)) 
		>> DPLL1_CTL_PLL_DIV_POS;
	d++;

	return ((CK_CLKIN * m) / d);
}

/* __ck_get_clkm_rate()
 *
 * Gets the rate of the clock for clocks that use CLKM for their source.
 * 
 * ( see ck_get_rate() )
 */
static int
__ck_get_clkm_rate(ck_t ck)
{
	static __u8 bits2div[] = {1, 2, 4, 8};
	int in, bits, reg, shift;

	reg = *(CK_RATE_REG(ck));
	shift = CK_RATE_SHIFT(ck);

	in = ck_get_rate(CK_PARENT(ck));
	bits = (reg & (ARM_CKCTL_DIV_MSK << shift)) >> shift;
	return (in / bits2div[bits]);
}

/* ck_get_rate()
 *
 * Get the current rate a clock is running at, or the parent if 
 * appropriate.
 */
int
ck_get_rate(ck_t ck)
{
	ck_t parent;

	if (!CK_IN_RANGE(ck))
		return -EINVAL;

	switch (ck) {

	case ck_gen1:
		return __ck_get_pll_rate(ck);

	case clkin:
		return CK_CLKIN;
		
	case mpuxor_ck:
	case ck_gen2:
	case ck_gen3:
		return ck_get_rate(CK_PARENT(ck));
		
	case arm_ck:
	case mpuper_ck:
	case dsp_ck:
	case dspmmu_ck:
	case lcd_ck:
	case tc_ck:
	case dma_ck:
	case api_ck:
		return __ck_get_clkm_rate(ck);

	case mputim_ck:
		__ck_get_input(ck, &parent);
		return ck_get_rate(parent);
		
	case mpuwd_ck:
#if 0
		/* Whilst this is correct, it evaluates to zero when CK_CLKIN 
		 * is 13MHz!
		 */
		return CK_CLKIN / 14;
#else
		/* 13MHz / 14 == 928KHz, so round up to 1MHz as its more 
		 * accurate than 0.
		 */
		return 1;    
#endif

	default:
		return -EINVAL;
	}
}

/* ck_enable()
 *
 * Enable/disable a clock. I'll return an error if the h/w doesn't support it.
 * If you disable a clock being used by an active device then you probably 
 * just screwed it. YOU are responsible for making sure this doesn't happen. 
 * Good luck. 
 */
int
ck_enable(ck_t ck)
{
	volatile __u16 *reg;
	int shift;

	if (!CK_IN_RANGE(ck))
		return -EINVAL;
	
	if (ck_debug)
		printk(KERN_DEBUG "%s: %s\n", __FUNCTION__, CK_NAME(ck));

	if (CK_CAN_DISABLE(ck)) {
		reg = CK_ENABLE_REG(ck);
		shift = CK_ENABLE_SHIFT(ck);
		*reg |= (1 << shift);
	}
	return 0;
}

/* ck_disable()
 *
 * Enable/disable a clock. I'll return an error if the h/w doesn't support it.
 * If you disable a clock being used by an active device then you probably 
 * just screwed it. YOU are responsible for making sure this doesn't happen. 
 * Good luck. 
 */
int
ck_disable(ck_t ck)
{
	volatile __u16 *reg;
	int shift;

	if (!CK_IN_RANGE(ck) || ck == clkin)
		return -EINVAL;
	
	if (ck_debug)
		printk(KERN_DEBUG "%s: %s\n", __FUNCTION__, CK_NAME(ck));

	if (CK_CAN_DISABLE(ck)) {
		reg = CK_ENABLE_REG(ck);
		shift = CK_ENABLE_SHIFT(ck);
		*reg &= ~(1 << shift);
	}
	return 0;
}

/* ck_valid_rate()
 *
 * Use this to get a bitmap of available rates for the clock. Caller allocates 
 * the buffer and passes in the length. Clock module fills up to len bytes of 
 * the buffer & passes back actual bytes used. 
 */
int ck_valid_rate(int rate)
{
	return test_bit(rate, ck_valid_table);
}

/* __ck_make_lookup_table()
 *
 * Creates a lookup table of rates and prescalers for DPLL multiplier
 * and divisor values as used by DPLL1.
 */
static void
__ck_make_lookup_table(void)
{
	__u8 m, d;

	memset((void *)ck_valid_table, 0, sizeof (ck_valid_table));

	for (m = 1; m <= DPLL1_CTL_PLL_MULT_MSK; m++)
		for (d = 0; d <= DPLL1_CTL_PLL_DIV_MSK; d++) {

			int rate = ((CK_CLKIN * m) / (d + 1));

			if (rate > CK_MAX_PLL_FREQ)
				continue;
			if (test_bit(rate, ck_valid_table))
				continue;
			set_bit(rate, ck_valid_table);
			ck_lookup_table[rate - 1] = (m << 2) | (d);
		}
}

/* init_ck()
 *
 * Initialises the clock control module. Responsible for setting up the 
 * DPLL, default prescalar values and enabling the initial clocks required.
 */
int __init
init_ck(void)
{
        printk("Initialising OMAP730 Clock Control.\n");

	__ck_make_lookup_table();

	/* We want to set syncronous scalable mode. This means that the MPU,
	 * GSM and TC  domains are synchronous and run at different clock 
	 * speeds. The clocks are all multiples of one another. 
	 *
	 * The divide down bits ARMDIV, DSPDIV and TCDIV in ARM_CKCTL define
	 * the prescalar value from the frequency of the DPLL.
	 *
	 * Note : We must make sure that DSPMMUDIV = DSPDIV * 2 in this mode.
	 */
	*ARM_SYSST = 0x1000;


	/* Waiting some time - if not, processor comes to incorrect state 
	 * when process next line while kernel XIP mode. I don't know why.
	 */
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	*ARM_CKCTL = OMAP_ARM_CKCTL;
	*DPLL1_CTL_REG = OMAP_DPLL_CTL_REG;

	/* 
	 * Turn off some other junk the bootloader might have turned
	 * on.
	 *
	 * Note : In the OMAP730 TRM, apparently various ARM_* registers
	 * have 'reserved' fields that should be kept at certain values. These
	 * are honored here so careful now...
	 *    ARM_IDLECT1:8  Always write 0
	 *    ARM_IDLECT2:9  Always write 0
	 *    ARM_RSTCT1:2   Always write 0
	 *    ARM_RSTCT1:1   Always write 0
	 */

		/* Input clock 13Mhz */
	*((volatile __u16 *)PCC_CTRL_REG) |= CLK_SWITCH_CMD;
	*((volatile __u16 *)ARM_IDLECT1) = 0x0400; /* Set WKUP_MODE */
	*((volatile __u16 *)ARM_IDLECT2) = 0x0000; /* Everything disabled */
	*((volatile __u16 *)ARM_RSTCT1)  = 0x0000; /* Global System Reset */

	/* 
	 * Only enable those clocks we will need, let the drivers
	 * enable other clocks as necessary.
	 */
	ck_enable(mpuper_ck);
	ck_enable(mpuxor_ck);
	ck_set_rate(mputim_ck, CK_CLKIN);
	ck_enable(mputim_ck);
	/* This timer is used to get machine cycles count, and compensate missed 
	 * timer interrupts (cf. time.h)
	 */
	start_mputimer1(0xffffffff);

	return 0;
}

EXPORT_SYMBOL(ck_get_rate);
EXPORT_SYMBOL(ck_set_rate);
EXPORT_SYMBOL(ck_enable);
EXPORT_SYMBOL(ck_disable);
