/*
 * include/asm-arm/arch-omap1610/time.h
 *
 * System timer definitions
 *
 *  Copyright (C) 2004 MontaVista Software, Inc.
 *    <source@mvista.com>
 * Copyright (C) 2000 RidgeRun, Inc.
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
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

#ifndef __ASM_ARCH_OMAP_TIME_H
#define __ASM_ARCH_OMAP_TIME_H

#include <linux/config.h>

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/arch/ck.h>
#include <asm/arch/timer.h>

#ifndef __instrument
#define __instrument
#define __noinstrument __attribute__ ((no_instrument_function))
#endif

/* After programming PTV with 0, the timer count rate is 6 MHz.
 * WARNING! this must be an even number, or machinecycles_to_usecs
 * below will break.
 */
#define MPUTICKS_PER_USEC 6

extern int mputimer_started[];
extern unsigned long (*gettimeoffset)(void);
extern unsigned long systimer_mark;
extern unsigned long omap1610_gettimeoffset(void);
#ifdef CONFIG_LEDS_TIMER
extern void do_leds(void);
#endif


extern inline void __noinstrument start_mputimer(int n,
						 unsigned long load_val)
{
	volatile mputimer_regs_t* timer = mputimer_base(n);

	mputimer_started[n] = 0;
	timer->cntl = MPUTIM_CLOCK_ENABLE;
	timer->load_tim = load_val;
	timer->cntl = (MPUTIM_CLOCK_ENABLE | MPUTIM_AR | MPUTIM_ST);
	mputimer_started[n] = 1;
}

extern inline unsigned long __noinstrument read_mputimer(int n)
{
	volatile mputimer_regs_t* timer = mputimer_base(n);
	return (mputimer_started[n] ? timer->read_tim : 0);
}

static void omap1610_timer_interrupt(int irq, 
                                     void *dev_id, 
                                     struct pt_regs *regs)
{
        unsigned long now, ilatency;

        /*
         * mark the time at which the timer interrupt ocurred using
         * timer1. We need to remove interrupt latency, which we can
         * retrieve from the current system timer2 counter. Both the
         * offset timer1 and the system timer2 are counting at 6MHz,
         * so we're ok.
         */
        now = 0 - read_mputimer1();
        ilatency = MPUTICKS_PER_USEC * 1000 * 10 - read_mputimer2();
        systimer_mark = now - ilatency;
        do_timer(regs);
#ifdef CONFIG_LEDS_TIMER
        do_leds();
#endif
        do_profile(regs);
}

#ifndef NO_SETUP_TIMER
extern inline void setup_timer(void)
{
        /* Since we don't call request_irq, we must init the structure. */
        gettimeoffset = omap1610_gettimeoffset;

        timer_irq.handler = omap1610_timer_interrupt;
        timer_irq.flags = SA_INTERRUPT;
        setup_arm_irq(INT_TIMER2, &timer_irq);
        start_mputimer2(MPUTICKS_PER_USEC * 1000 * 10 - 1);
}
#endif

#endif /*  __ASM_ARCH_OMAP_TIME_H */
