/*
 * include/asm-arm/arch-pxa/preem_latency.h
 * Timing support for preempt-stats patch.
 *
 * Copyright 2003 MontaVista Software Inc.
 * Author: source@mvista.com
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 * WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 * USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "hardware.h"

#define readclock_init()
#define readclock(x)		do { (x) = OSCR; } while (0)
#define INTERRUPTS_ENABLED(x)   (!((x) & I_BIT)) /* Assuming x is PSR, check for I bit */
#ifdef CONFIG_CPU_BULVERDE
/* On the Bulverde, OSCR0 increments at the rising edge of 3.25 MHz clock */
#define clock_to_usecs(x)       (((x) * 4) / 13)
#define TICKS_PER_USEC          (3)
#else
#define clock_to_usecs(x)	((x) * 136 / 501) /* 501/136 ~= 3.6838, but we have 3.6864 */
#define TICKS_PER_USEC          (4)
#endif
