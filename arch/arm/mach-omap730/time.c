/*
 *  linux/arch/arm/mach-omap730/time.c
 *
 * Copyright (C) 2000 RidgeRun, Inc. (http://www.ridgerun.com)
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
 * Copyright (C) 2004 Montavista Software, Inc.
 *         <source@mvista.com>.
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
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <asm/hardware.h>
#include <asm/arch/timer.h>
#define NO_SETUP_TIMER
#include <asm/arch/time.h>
#include <asm/arch/irqs.h>

/*
 * This marks the time of the last system timer interrupt
 * that was *processed by the ISR* (timer 2).
 */
unsigned long systimer_mark;

int mputimer_started[3] = {0,0,0};

void __noinstrument start_mputimer1(unsigned long load_val)
{
	start_mputimer(0, load_val);
}

void __noinstrument start_mputimer2(unsigned long load_val)
{
	start_mputimer(1, load_val);
}

void __noinstrument start_mputimer3(unsigned long load_val)
{
	start_mputimer(2, load_val);
}

unsigned long __noinstrument read_mputimer1(void)
{
	return read_mputimer(0);
}

unsigned long __noinstrument read_mputimer2(void)
{
	return read_mputimer(1);
}

unsigned long __noinstrument read_mputimer3(void)
{
	return read_mputimer(2);
}

unsigned long __noinstrument do_getmachinecycles(void)
{
	return 0 - read_mputimer(0);
}

unsigned long __noinstrument machinecycles_to_usecs(unsigned long mputicks)
{
	/* This calculation overflows for values of mputicks greater than 
	 * 2147483, which corresponds to about 330 milliseconds at a 
	 * tick rate of 6.5MHz.
	 */
	return (((mputicks*1000*2) / MPUTICKS_PER_MSEC) + 1) >> 1;
}

unsigned long omap730_gettimeoffset(void)
{
	/* return elapsed usecs since last system timer ISR */
	return machinecycles_to_usecs(do_getmachinecycles() - systimer_mark);
}
