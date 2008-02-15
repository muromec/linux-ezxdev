/*
 * linux/include/asm-arm/arch-ti925/time.h
 *
 * BRIEF MODULE DESCRIPTION
 * timer definition
 *
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

#if !defined(__ASM_ARCH_TI925_EVM_TIME_H)
#define __ASM_ARCH_TI925_EVM_TIME_H

#include <linux/config.h>
#include <asm/system.h>
#include <asm/leds.h>

#define TIMER0_READ(r)    inl(IO_ADDRESS((r)+ TI925_TIMER0_BASE))
#define TIMER0_WRITE(r,v) outl((v),IO_ADDRESS(((r) + TI925_TIMER0_BASE)))

#define TIMER1_READ(r)    inl(IO_ADDRESS((r)+ TI925_TIMER1_BASE))
#define TIMER1_WRITE(r,v) outl((v),IO_ADDRESS(((r) + TI925_TIMER0_BASE)))

// Timer Registers
#define TIMER_TIM    0x00
#define TIMER_PRD    0x08
#define TIMER_TDDR   0x10
#define TIMER_TCR    0x14

// Timer Control Register definition
#define TIMER_TSS    (1<<0)
#define TIMER_TRB    (1<<1)
#define TIMER_INT    (1<<2)
#define TIMER_ARL    (1<<3)

/*
 * How long is the timer interval? 100 HZ, The CPU is 65Mhz/100HZ is the period
 */

#define TIMER_INTERVAL (65000000/100)

/*
 * IRQ handler for the timer
 */
static void ti925_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	do_leds();
	do_timer(regs);
	do_profile(regs);
	// restart the timer
	TIMER0_WRITE(TIMER_TCR,TIMER_INT);  // clear the timer int
	TIMER0_WRITE(TIMER_TCR,TIMER_TRB);  // reload the timer
	TIMER0_WRITE(TIMER_TCR,TIMER_TSS);  // start the timer

	// now we must clear the interrrupt, since its a level interrupt (i think)
	ack_level_irq(irq);
}

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
extern __inline__ void setup_timer(void)
{
        // since we don't call request_irq, we must init the structure
	timer_irq.handler = ti925_timer_interrupt;
	timer_irq.flags = SA_INTERRUPT;
	/*
	 * Initialise to a known state (all timers off)
	 */
	TIMER0_WRITE(TIMER_TCR, 0x0 );
	TIMER1_WRITE(TIMER_TCR, 0x0 );

	TIMER0_WRITE(TIMER_PRD,TIMER_INTERVAL); // ticks until its done
	TIMER0_WRITE(TIMER_TCR,TIMER_TSS); // start

	/* 
	 * Make irqs happen for the system timer
	 */
	setup_arm_irq(INT_TIN0, &timer_irq);
}
#endif
