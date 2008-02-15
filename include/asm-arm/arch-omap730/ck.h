/*
 * include/asm-arm/arch-omap730/ck.h
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *   <source@mvista.com>.
 *
 * This file is based on:
 *
 * NAME
 *        ck.h -- generic interface to clock control
 *     
 *  DESCRIPTION
 *        This interface will be implemented by arch-specific DPLL/CLKM/Clock
 *        Controller drivers. It's NOT a user interface -- if you want to
 *        provide a user interface then lay a proc or dev driver over it. The
 *        idea is that you can either do that or you can keep clock control
 *        entirely inside the kernel by having a clock policy module use this
 *        interface for control/sense.
 *
 *        This shows some possible architectures for supporting power management
 *        or whatever on top of the clock control interface:
 *
 *        +--------------------------+
 *        | pm app                   |
 *        +--------------------------+
 *        +-----------+ +------------+ +-----------+
 *        | proc "pm" | | /dev/clock | | pm module |
 *        +-----------+ +------------+ +-----------+
 *        /----------------------------------------\
 *        | ck.h ifc                               |
 *        \----------------------------------------/
 *        +----------------------------------------+
 *        | arch-specific clock driver(s)          |
 *        +----------------------------------------+
 *        
 *        Since this is a kernel-only interface it doesn't belong in the kernel
 *        includes -- only other modules should include this file.
 *
 *  AUTHOR
 *        gmcnutt@ridgerun.com
 *
 *  COPYRIGHT
 *        (C) 2001 RidgeRun, Inc. (http://www.ridgerun.com)
 *
 *
 * And also on:
 *
 *
 *       arch_ck.h -- enumerate the clocks for the omap1610 platform
 *    
 * AUTHOR
 *       gmcnutt@ridgerun.com
 *       Dave Peverley <dpeverley@mpc-data.co.uk>
 *
 * REFERENCES
 *       [1] Dsc21ArmPeripherals0.1.pdf Section 14
 *       [2] TMS320DSC21 Design Workshop Student Guide v3 p 5-14
 *
 * COPYRIGHT
 *       (C) 2001 RidgeRun, Inc. (http://www.ridgerun.com)
 *       (C) 2004 MPC-Data Limited (http://www.mpc-data.co.uk)
 *
 *   This program is free software; you can redistribute  it and/or modify it
 *   under  the terms of  the GNU General  Public License as published by the
 *   Free Software Foundation;  either version 2 of the  License, or (at your
 *   option) any later version.
 * 
 *   THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
 *   WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *   NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *   NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *   USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *   ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *   You should have received a copy of the  GNU General Public License along
 *   with this program; if not, write  to the Free Software Foundation, Inc.,
 *   675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_CK_H
#define __ASM_ARCH_CK_H

#include <linux/config.h>

/*
 * This enum is the bare minimum needed to support the kmod/ck.h interface.  
 */
typedef enum { 

        /* System clocks (fixed) */
        clkin = 0,

        /* DPLL1 */
	ck_gen1,   
	ck_gen2,  
	ck_gen3,

        /* Special:  TC needs to be checked before anything else,
	 * so I'll put it first so I can easily loop through.
	 */
        tc_ck,
        
        /* CLKM1 - MPU CLock Domain */
        arm_ck,      /* ARM_CK    : Straight division of DPLL1 */
	mpuper_ck,   /* ARMPER_CK : MPU External Peripheral Clock */
	mpuxor_ck,   /* ARMXOR_CK : Gated version of CK_REF for external periphs */
	mputim_ck,   /* ARMTIM_CK : OMAP3.2 MPU Internal OS Timers */
	mpuwd_ck,    /* ARMWDI_CK : MPU Watchdog Timer - CK_REF/14 */

        /* CLKM2 - DSP (GSM) Clock Domain */
	dsp_ck,      /*             Clock driving the DSP's MMU */
	dspmmu_ck,   /* DSPMMU_CK : Clock driving the DSP's MMU */

        /* CLKM3 - Traffic Controller Clock domain */
        /* tc_ck,       TC_CK : Traffic controller clock */ 
	dma_ck,      /* DMA_CK : System DMA Clock */
	api_ck,      /* API_CK : MPUI port clock */
	lcd_ck       /* LCD_CK : LCD clock */
} ck_t;

#define CK__MIN clkin
#define CK__MAX lcd_ck
#define CK_IN_RANGE(ck) (!((ck < CK__MIN) || (ck > CK__MAX)))

/* 
 * For different clock rates, we must set up some default 
 * states for DPLL1_CTL_REG. Note CLKIN == 13MHz so PLL_MULT
 * wont necesarrily give exact values as the defines indicate...
 * 
 * For all these values :
 *   LS_DISABLE=transparent     IAI=0
 *   IOB=StartLock              DPLL_TCLKOTU=DPLL1 
 *   PLL_DIV=CK_REF             PLL_ENABLE=Lock 
 *   BYPASS_DIV=CK_REF 
 *
 *   30MHz   -> PLL_MULT = 2 (d)     == 26 MHz
 *   60MHz   -> PLL_MULT = 5 (d)     == 65 MHz
 *   120MHz  -> PLL_MULT = 9 (d)     == 117 MHz
 *   168MHz  -> PLL_MULT = 13 (d)    == 169 MHz
 *   182MHz  -> PLL_MULT = 14 (d)    == 182 MHz
 *   192MHz  -> PLL_MULT = 15 (d)    == 195 MHz
 *   195MHz  -> PLL_MULT = 15 (d)    == 195 MHz
 *   201MHz  -> PLL_MULT = 31 (d),
 *              PLL_DIV  = 2 (d)     == 201.5 MHz
 */
#if defined(CONFIG_OMAP_ARM_120MHZ)
#define OMAP_ARM_CKCTL 0x6505
#define OMAP_DPLL_CTL_REG 0x2492
#define CK__MAX_RATE               117
#define CLOCK_INSTANT_ON_LPJ 288823 
#elif defined(CONFIG_OMAP_ARM_60MHZ)
#define OMAP_ARM_CKCTL 0x6505
#define OMAP_DPLL_CTL_REG 0x2292
#define CK__MAX_RATE               65
#define CLOCK_INSTANT_ON_LPJ 160457 
#elif defined(CONFIG_OMAP_ARM_30MHZ)
#define OMAP_ARM_CKCTL 0x6505
#define OMAP_DPLL_CTL_REG 0x2112
#define CK__MAX_RATE               26
#define CLOCK_INSTANT_ON_LPJ 64183 
#elif defined(CONFIG_OMAP_ARM_168MHZ)
#define OMAP_ARM_CKCTL 0x6505
#define OMAP_DPLL_CTL_REG 0x2692
#define CK__MAX_RATE               169
#define CLOCK_INSTANT_ON_LPJ 417189
#elif defined(CONFIG_OMAP_ARM_182MHZ)
#define OMAP_ARM_CKCTL 0x6505
#define OMAP_DPLL_CTL_REG 0x2712
#define CK__MAX_RATE               182
#define CLOCK_INSTANT_ON_LPJ 449280
#elif defined(CONFIG_OMAP_ARM_192MHZ)
#define OMAP_ARM_CKCTL 0x6505
#define OMAP_DPLL_CTL_REG 0x2792
#define CK__MAX_RATE               195
#define CLOCK_INSTANT_ON_LPJ 481371
#elif defined(CONFIG_OMAP_ARM_195MHZ)
#define OMAP_ARM_CKCTL 0x6505
#define OMAP_DPLL_CTL_REG 0x2792
#define CK__MAX_RATE               195
#define CLOCK_INSTANT_ON_LPJ 481371
#elif defined(CONFIG_OMAP_ARM_201MHZ)
#define OMAP_ARM_CKCTL 0x6505
#define OMAP_DPLL_CTL_REG 0x2FB2
#define CK__MAX_RATE               201
#define CLOCK_INSTANT_ON_LPJ 497417
#else
#error "OMAP MHZ not set, please run make xconfig"
#endif
#define CK_MAX_PLL_FREQ CK__MAX_RATE

/* Input clock frequency is 13MHz - this is used in the DPM code, so declare here */
#define CK_CLKIN                   13 

#define CK_DPLL_MASK 0x0fe0
/* This is used by CK and DSPC, so put it somewhere they can both see it. */
#define MPUI_STROBE_MAX_1509 24
#define MPUI_STROBE_MAX_1610 30
#define MPUI_STROBE_MAX_730  30

typedef struct {
	char *name;
	__u8 flags;
	ck_t parent;
	volatile __u16 *rreg;
	volatile __u16 *ereg;
	volatile __u16 *ireg;
	volatile __u16 *sreg;
	__s8 rshift;
	__s8 eshift;
	__s8 ishift;
	__s8 sshift;
} ck_info_t __attribute__ ((packed));

/* flags definitions */
#define CK_RATEF   (1<<0)
#define CK_IDLEF   (1<<1)
#define CK_ENABLEF (1<<2)
#define CK_SELECTF (1<<3)
#define CK_FLAGS(ck) ck_info_table[ck].flags
#define CK_CAN_CHANGE_RATE(cl) (CK_FLAGS(ck) & CK_RATEF)
#define CK_CAN_DISABLE(cl) (CK_FLAGS(ck) & CK_ENABLEF)
#define CK_CAN_IDLE(cl) (CK_FLAGS(ck) & CK_IDLEF)
#define CK_CAN_SWITCH(cl) (CK_FLAGS(ck) & CK_SELECTF)

#define CK_NAME(ck) ck_info_table[ck].name
#define CK_PARENT(ck) ck_info_table[ck].parent
#define CK_RATE_REG(ck) ck_info_table[ck].rreg
#define CK_ENABLE_REG(ck) ck_info_table[ck].ereg
#define CK_IDLE_REG(ck) ck_info_table[ck].ireg
#define CK_SELECT_REG(ck) ck_info_table[ck].sreg
#define CK_RATE_SHIFT(ck) ck_info_table[ck].rshift
#define CK_ENABLE_SHIFT(ck) ck_info_table[ck].eshift
#define CK_IDLE_SHIFT(ck) ck_info_table[ck].ishift
#define CK_SELECT_SHIFT(ck) ck_info_table[ck].sshift

/*  Clock initialization.  */
int init_ck(void);

/* Use this to set a clock rate. If other clocks are derived from this one, 
   their rates will all change too. If this is a derived clock and I can't 
   change it to match your request unless I also change the parent clock, then
   tough luck -- I won't change the parent automatically. I'll return an error
   if I can't get the clock within 10% of what you want. Otherwise I'll return
   the value I actually set it to. If I have to switch parents to get the rate
   then I will do this automatically (since it only affects this clock and its
   descendants). */
int ck_set_rate(ck_t ck, int val_in_mhz);
int ck_get_rate(ck_t ck);

/* Use this to check the validity of a given rate. */
int ck_valid_rate(int rate);

/* Enable/disable a clock. I'll return an error if the h/w doesn't support it.
   If you disable a clock being used by an active device then you probably 
   just screwed it. YOU are responsible for making sure this doesn't happen. 
   Good luck. */
int ck_enable(ck_t);
int ck_disable(ck_t);

#endif /* __ASM_ARCH_CK_H */
