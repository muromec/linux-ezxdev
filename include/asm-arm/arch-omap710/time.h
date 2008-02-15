/*
 * linux/include/asm-arm/arch-ti925/time.h
 *
 * BRIEF MODULE DESCRIPTION
 * 32kHz timer definition
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
#if !defined(__ASM_ARCH_OMAP710_TIME_H)
#define __ASM_ARCH_OMAP710_TIME_H

#include <linux/config.h>
#include <asm/system.h>
#include <asm/leds.h>
#include <asm/irq.h>

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#  define DBG(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args)
#  define ENTRY() DBG(":entry\n")
#else
#  define DBG(fmt, args...)
#  define ENTRY()
#endif

#define OMAP_32kHz_TIMER_BASE 0XFFFB9000

inline unsigned long  timer_read(int reg) {
        unsigned long val;
        val = (inw(IO_ADDRESS((reg) + OMAP_32kHz_TIMER_BASE)));
        DBG("TR: r:%d v:%x\n",reg,val);
        return val;
}
inline void timer_write(int reg,int val) {
        DBG("TW: r:%d v:%x\n",reg,val);
        outw( (val), (IO_ADDRESS( (reg) + OMAP_32kHz_TIMER_BASE)));
}

// Timer Registers
#define TIMER_CR     0x04
#define TIMER_TVR    0x00
#define TIMER_TCR    0x02

// Timer Control Register definition
#define TIMER_TSS    (1<<0)
#define TIMER_TRB    (1<<1)
#define TIMER_INT    (1<<2)
#define TIMER_ARL    (1<<3)

/*
 * How long is the timer interval? 100 HZ, right...
 * IRQ rate = (TVR + 1) / 32768 seconds 
 * TVR = 32768 * IRQ_RATE -1
 * IRQ_RATE =  1/100
 * TVR = 326
 */
#define TIMER_PERIOD 326 


#ifdef DEBUG
static volatile int timer_max_count;
#endif

/*
 * IRQ handler for the timer
 */
static inline void start_timer(void) {
#ifdef DEBUG

        // Test code that sets the timer as a 1-shot and makes sure
        //  it gets the time value written correctly.  timer_max_count
        //  is the max number of loops this had to go through.
        int count = 0;
        unsigned long val = 0;
        while (val != (TIMER_TSS | TIMER_TRB | TIMER_INT)) {
                timer_write(TIMER_CR,TIMER_TSS | TIMER_TRB | TIMER_INT); // start
                val = (inw(IO_ADDRESS((TIMER_CR) + OMAP_32kHz_TIMER_BASE)));
                count++;
        }
        if (count > timer_max_count) {
                timer_max_count = count;
        }

#else

        // Standard mode.  Just use the auto-reload timer.
	timer_write(TIMER_CR,TIMER_TSS | TIMER_TRB | TIMER_INT | TIMER_ARL); // start
#endif
}

static void omap_32kHz_timer_interrupt(int irq, 
        void *dev_id, 
        struct pt_regs *regs)
{
        ENTRY();
	do_leds();
	do_timer(regs);
	do_profile(regs);

#ifdef DEBUG
        start_timer();
#endif
}

extern __inline__ void setup_timer(void)
{
        // since we don't call request_irq, we must init the structure
        ENTRY();
        timer_irq.handler = omap_32kHz_timer_interrupt;
	timer_irq.flags = SA_INTERRUPT;
	timer_write(TIMER_CR, 0x0);
        timer_write(TIMER_TVR,TIMER_PERIOD);
	setup_arm_irq(INT_OS_32kHz_TIMER, &timer_irq);
        start_timer();
}

#endif
/* ---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
