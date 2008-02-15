/*
 * arch/arm/mach-omap1610/ck.c
 *
 * Implements the clock interface for the omap1610
 * Copyright (C) 2004 MontaVista Software, Inc.
 *   <source@mvista.com>.
 *
 * Based on arch/arm/mach-omap1510/ck.c:
 * Created 2001, Copyright (C) 2001 RidgeRun, Inc.  All rights reserved.
 * Author: Gordon McNutt <gmcnutt@ridgerun.com>
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

static ck_info_t ck_info_table[] = {
	{
		.name = "clkin",
		.flags = 0,
		.parent = clkin,
	}, {
		.name = "ck_gen1",
		.flags = CK_RATEF | CK_IDLEF,
		.rreg = CK_DPLL1,
		.ireg = ARM_IDLECT1,
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
		.ireg = ARM_IDLECT1,
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
		.flags = CK_RATEF | CK_ENABLEF,
		.parent = ck_gen2,
		.rreg = ARM_CKCTL,
		.ereg = ARM_CKCTL,
		.rshift = DSPDIV,
		.eshift = EN_DSPCK,
	}, {
		.name = "dspmmu_ck",
		.flags = CK_RATEF | CK_ENABLEF,
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
		.flags = CK_RATEF | CK_IDLEF | CK_ENABLEF,
		.parent = ck_gen3,
		.rreg = ARM_CKCTL,
		.ireg = ARM_IDLECT1,
		.ereg = ARM_IDLECT2,
		.rshift = TCDIV,
		.ishift = IDLAPI_ARM,
		.eshift = EN_APICK,
	}, {
		.name = "lcd_ck",
		.flags = CK_RATEF | CK_ENABLEF,
		.parent = ck_gen3,
		.rreg = ARM_CKCTL,
		.ireg = ARM_IDLECT1,
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
	*pll &= ~(0x1f << 5);

	/* set the rate bits */
	*pll |= (ck_lookup_table[rate - 1] << 5);

	return 0;
}

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
	if (div == 4) {
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
	*reg &= ~(3 << shift);
	*reg |= (div << shift);

	/* And return the new (actual, after rounding down) rate. */
	return prate;
}

int
ck_set_rate(ck_t ck, int rate)
{
	if (!CK_IN_RANGE(ck) || !CK_CAN_CHANGE_RATE(ck))
		return -EINVAL;
	return (ck == ck_gen1) ? __ck_set_pll_rate(ck, rate) :
		__ck_set_clkm_rate(ck, rate);
}

static int
__ck_get_pll_rate(ck_t ck)
{
	int m, d;

	__u16 pll = *((volatile __u16 *) CK_RATE_REG(ck));

	m = (pll & (0x1f << 7)) >> 7;
	m = m ? m : 1;
	d = (pll & (3 << 5)) >> 5;
	d++;

	return ((CK_CLKIN * m) / d);
}

static int
__ck_get_clkm_rate(ck_t ck)
{
	static __u8 bits2div[] = {1, 2, 4, 8};
	int in, bits, reg, shift;

	reg = *(CK_RATE_REG(ck));
	shift = CK_RATE_SHIFT(ck);

	in = ck_get_rate(CK_PARENT(ck));
	bits = (reg & (3 << shift)) >> shift;
	return (in / bits2div[bits]);
}

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
		/* Note that this evaluates to zero when CK_CLKIN is 12MHz. */
		return CK_CLKIN / 14;
	default:
		return -EINVAL;
	}
}

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

int ck_valid_rate(int rate)
{
	return test_bit(rate, ck_valid_table);
}

static void
__ck_make_lookup_table(void)
{
	__u8 m, d;

	memset((void *)ck_valid_table, 0, sizeof (ck_valid_table));

	for (m = 1; m < 32; m++)
		for (d = 1; d < 5; d++) {

			int rate = ((CK_CLKIN * m) / (d));

			if (rate > CK_MAX_PLL_FREQ)
				continue;
			if (test_bit(rate, ck_valid_table))
				continue;
			set_bit(rate, ck_valid_table);
			ck_lookup_table[rate - 1] = (m << 2) | (d - 1);
		}
}

int __init
init_ck(void)
{
	__ck_make_lookup_table();

	/* We want to be in syncronous scalable mode */
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
	*DPLL_CTL_REG = OMAP_DPLL_CTL_REG;

	/* Turn off some other junk the bootloader might have turned on. */
	*ARM_CKCTL &= 0x0fff;	/* Turn off DSP, ARM_INTHCK, ARM_TIMXO. */
	*ARM_RSTCT1 = 0;	/* Put DSP/MPUI into reset until needed. */
	*ARM_RSTCT2 = 1;
	*ARM_IDLECT1 = 0x400;
	/* According to OMAP5910 Erratum SYS_DMA_1, bit DMACK_REQ (bit 8)
	 * of the ARM_IDLECT2 register must be set to zero.  The power-on 
	 * default value of this bit is one.
	 */
	if (inl(OMAP_LCD_CONTROL) & 1)	/* Test to see if LCD is enabled. */
		*ARM_IDLECT2 = 0x0008;	/* Leave LCD clock on if LCD is on. */
	else
		*ARM_IDLECT2 = 0x0000;	/* Turn LCD clock off also. */

	/* Only enable those clocks we will need, let the drivers
	 * enable other clocks as necessary.
	 */
	ck_enable(mpuper_ck);
	ck_enable(mpuxor_ck);
	ck_enable(mputim_ck);
	start_mputimer1(0xffffffff);

	return 0;
}

EXPORT_SYMBOL(ck_get_rate);
EXPORT_SYMBOL(ck_set_rate);
EXPORT_SYMBOL(ck_enable);
EXPORT_SYMBOL(ck_disable);
