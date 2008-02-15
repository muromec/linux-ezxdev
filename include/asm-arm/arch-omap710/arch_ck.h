/*****************************************************************************
  NAME
        arch_ck.h -- enumerate the clocks for the omap710 platform
     
  AUTHOR
        gmcnutt@ridgerun.com

  REFERENCES
        [1] Dsc21ArmPeripherals0.1.pdf Section 14
        [2] TMS320DSC21 Design Workshop Student Guide v3 p 5-14

  COPYRIGHT
        (C) 2001 RidgeRun, Inc. (http://www.ridgerun.com)

   This program is free software; you can redistribute  it and/or modify it
   under  the terms of  the GNU General  Public License as published by the
   Free Software Foundation;  either version 2 of the  License, or (at your
   option) any later version.
 
   THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
   WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
   NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
   NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
   USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
   Please report all bugs and problems to the author or <support@dsplinux.net> 

   key: RRGPLCR (do not remove)
 *****************************************************************************/
#ifndef arch_ck_h
#define arch_ck_h

#include <linux/config.h>

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
        arm_ck, mpuper_ck, arm_gpio_ck, mpuxor_ck, mputim_ck, mpuwd_ck,

        /* CLKM2 */
        dsp_ck, dspmmu_ck, 

#if 0
        /* accessible only from the dsp... */
        dspper_ck, gpio_ck, dspxor_ck, dsptim_ck, dspwd_ck,
        uart_ck,
#endif
        /* CLKM3 */
        /* tc_ck, */ dma_ck, api_ck, hsab_ck, lbfree_ck, lb_ck, lcd_ck

} ck_t;

typedef enum {
        /* Reset the MPU */
        arm_rst,

        /* Reset the DSP */
        dsp_rst,

        /* Reset priority registers, EMIF config, and MPUI control logic */
        api_rst,

        /* Reset DSP, MPU, and Peripherals */
        sw_rst,
} reset_t;

#define CK__MIN clkin
#define CK__MAX lcd_ck
//#define CK__MAX_RATE (12 * 31)
#if defined(CONFIG_OMAP_ARM_120MHZ)
#define CK__MAX_RATE 120
#elif defined(CONFIG_OMAP_ARM_60MHZ)
#define CK__MAX_RATE 60
#elif defined(CONFIG_OMAP_ARM_30MHz)
#define CK__MAX_RATE 30
#endif

// This is used by CK and DSPC, so put it somewhere they can both see it.
#define MPUI_STROBE_MAX 24

#endif
