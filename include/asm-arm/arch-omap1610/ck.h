/*
 * include/asm-arm/arch-omap1610/ck.h
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
 *
 * REFERENCES
 *       [1] Dsc21ArmPeripherals0.1.pdf Section 14
 *       [2] TMS320DSC21 Design Workshop Student Guide v3 p 5-14
 *
 * COPYRIGHT
 *       (C) 2001 RidgeRun, Inc. (http://www.ridgerun.com)
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

/* This enum is the bare minimum needed to support the kmod/ck.h interface. 
   I don't list entries for ck_gen2/ck_gen3 since these aren't really supported
   yet for OMAP. To use these we'd have to select one of the non-recommended
   clocking schemes.
*/
typedef enum { 

        /* system clocks (fixed) */
        clkin = 0,

        /* DPLL1 */
        ck_gen1, ck_gen2, ck_gen3,

        /*  
            Special:  TC needs to be checked before anything else,
            so I'll put it first so I can easily loop through.
        */
        tc_ck,
        
        /* CLKM1 */
        arm_ck, mpuper_ck, mpuxor_ck, mputim_ck, mpuwd_ck,

        /* CLKM2 */
        dsp_ck, dspmmu_ck, 

        /* CLKM3 */
        dma_ck, api_ck, lcd_ck
	
} ck_t;

#define CK__MIN clkin
#define CK__MAX lcd_ck
#define CK_IN_RANGE(ck) (!((ck < CK__MIN) || (ck > CK__MAX)))

#if defined(CONFIG_OMAP_ARM_120MHZ)
#define OMAP_ARM_CKCTL 0x110a
#define OMAP_DPLL_CTL_REG 0x2510
#define CK__MAX_RATE 120
#define CLOCK_INSTANT_ON_LPJ 296960 
#elif defined(CONFIG_OMAP_ARM_60MHZ)
#define OMAP_ARM_CKCTL 0x1005
#define OMAP_DPLL_CTL_REG 0x2290
#define CK__MAX_RATE 60
#define CLOCK_INSTANT_ON_LPJ 146944 
#elif defined(CONFIG_OMAP_ARM_30MHZ)
#define OMAP_ARM_CKCTL 0x1555
#define OMAP_DPLL_CTL_REG 0x2290
#define CK__MAX_RATE 30
#define CLOCK_INSTANT_ON_LPJ 72448 
#elif defined(CONFIG_OMAP_ARM_168MHZ)
#define OMAP_ARM_CKCTL 0x110f
#define OMAP_DPLL_CTL_REG 0x2710
#define CK__MAX_RATE 168
#define CLOCK_INSTANT_ON_LPJ 414720 
#elif defined(CONFIG_OMAP_ARM_192MHZ)
#define OMAP_ARM_CKCTL 0x110f
#define OMAP_DPLL_CTL_REG 0x2810
#define CK__MAX_RATE 192 
#define CLOCK_INSTANT_ON_LPJ 477182 
#else
#error "OMAP MHZ not set, please run make xconfig"
#endif
#define CK_MAX_PLL_FREQ CK__MAX_RATE

#define CK_CLKIN     12 /* MHz */

#define CK_DPLL_MASK 0x0fe0
/* This is used by CK and DSPC, so put it somewhere they can both see it. */
#define MPUI_STROBE_MAX_1509 24
#define MPUI_STROBE_MAX_1610 30

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
extern int init_ck(void);

/* Use this to set a clock rate. If other clocks are derived from this one, 
   their rates will all change too. If this is a derived clock and I can't 
   change it to match your request unless I also change the parent clock, then
   tough luck -- I won't change the parent automatically. I'll return an error
   if I can't get the clock within 10% of what you want. Otherwise I'll return
   the value I actually set it to. If I have to switch parents to get the rate
   then I will do this automatically (since it only affects this clock and its
   descendants). */
extern int ck_set_rate(ck_t, int);
extern int ck_get_rate(ck_t);

/* Use this to check the validity of a given rate. */
extern int ck_valid_rate(int);

/* Enable/disable a clock. I'll return an error if the h/w doesn't support it.
   If you disable a clock being used by an active device then you probably 
   just screwed it. YOU are responsible for making sure this doesn't happen. 
   Good luck. */
extern int ck_enable(ck_t);
extern int ck_disable(ck_t);

#endif /* __ASM_ARCH_CK_H */
