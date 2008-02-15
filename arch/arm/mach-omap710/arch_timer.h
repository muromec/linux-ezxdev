/*
 * File: arch_timer.h
 *
 * Arch-specific timer internals
 *
 * DESCRIPTION                                                                   
 *    This file attempts to encapsulate all the arch-specific details needed
 *    by the generic timer.c. All arch_timer.h headers need to provide the
 *    definitions in the 'interface' section. The stuff in the 'internals'
 *    section may vary.
 *
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: <author> <author-email> <date>
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
 * Please report all bugs/problems to the author or <support@dsplinux.net>
 *
 * key: RRGPLCR (do not remove)
 *
 */

#ifndef dsplinux_arch_timer_h
#define dsplinux_arch_timer_h

#ifndef __TIMER_INTERNAL__
#error Only timer.c should include this file
#endif

#include <asm/arch/irqs.h>

/* In case it isn't obvious, most of this crap is a bad idea. The timer table
   is ok, and the register access macros are borderline, but the masks and 
   feature macros are too much. Let this serve as a cautionary example to
   myself and others.
   --gmcnutt   
 */

/*****************************************************************************
                          I n t e r f a c e
 *****************************************************************************/
#define TIMER_N sizeof(__timer_table)/sizeof(timer_info_t)
#define TIMER_MAX_RATE 120	/* hack alert! this really depends on DPLL1 */
#define TIMER_MIN_RATE 12	/* hack alert! assumes CLKIN */
#define TIMER_SLOW_INPUT_CK clkin
#define TIMER_FAST_INPUT_CK ck_gen1
#define TIMER_MAX_DIV (1 << TIMER_MAX_PTV)
#define TIMER_MAX_PTV 8

#define TIMER_CK(timer) \
        (TIMER_IS_WDG(timer) ? clkin /* hack */: mputim_ck)
#define TIMER_COUNT_MAX(timer) \
        (TIMER_IS_WDG((timer)) ? 0xffff : 0xffffffff)
#define TIMER_IRQ(timer) __timer_table[(timer)].irq
#define TIMER_CNTL_REG(timer) __timer_table[(timer)].cntl
#define TIMER_LOAD_REG(timer) __timer_table[(timer)].load
#define TIMER_READ_REG(timer) __timer_table[(timer)].read
#define TIMER_MODE_REG(timer) __timer_table[(timer)].mode
#define TIMER_FEATS(timer) __timer_table[(timer)].feats

#define TIMER_CHECK(timer) (((timer) >= 0) && ((timer) < TIMER_N))
#define TIMER_START_MASK(timer) __timer_table[(timer)].start_mask
#define TIMER_PTV_MASK(timer) __timer_table[(timer)].ptv_mask
#define TIMER_STOP_MASK(timer) TIMER_START_MASK(timer)
#define TIMER_CAN_RST(timer) (__timer_table[(timer)].feats & TIMERF_RST)
#define TIMER_MODE_MASK(timer) __timer_table[(timer)].mode_mask
#define TIMER_IS_WDG(timer) TIMER_CAN_RST(timer)
/*****************************************************************************
                          I n t e r n a l s
 *****************************************************************************/
typedef struct {
	int irq;
	volatile __u32 *cntl;
	volatile __u32 *load;
	volatile __u32 *read;
	volatile __u32 *mode;	/* Only applies to watchdog timer */
	__u8 start_mask;
	__u16 ptv_mask;
	__u16 mode_mask;
	int feats;
} timer_info_t;

#define TIMER_STD_FEATS (TIMERF_PERIODIC|TIMERF_ONESHOT|TIMERF_INT)
#define TIMER_WDT_FEATS (TIMERF_PERIODIC|TIMERF_ONESHOT|TIMERF_INT|TIMERF_RST)
#define TIMER_STD_SMASK ((1 << 5) | 1)
#define TIMER_STD_PMASK (7 << 2)
#define TIMER_STD_MMASK (1 << 1)

/* Keep the watchdog timer first so that people scanning for it at runtime will
   find it immediately.
*/
timer_info_t __timer_table[] = {
	{
	      cntl:(volatile __u32 *) 0xfffec800,
	      load:(volatile __u32 *) 0xfffec804,
	      read:(volatile __u32 *) 0xfffec804,
					/* Yes, same as load */
	      mode:(volatile __u32 *) 0xfffec808,
	      feats:TIMER_WDT_FEATS,
	      start_mask:(1 << 7),
	      ptv_mask:(7 << 9),
	      mode_mask:(1 << 8),
      irq:INT_WD_TIMER},
	{
	      cntl:(volatile __u32 *) 0xfffec500,
	      load:(volatile __u32 *) 0xfffec504,
	      read:(volatile __u32 *) 0xfffec508,
	      feats:TIMER_STD_FEATS,
	      start_mask:TIMER_STD_SMASK,
	      ptv_mask:TIMER_STD_PMASK,
	      mode_mask:TIMER_STD_MMASK,
      irq:INT_TIMER1},
	{
	      cntl:(volatile __u32 *) 0xfffec600,
	      load:(volatile __u32 *) 0xfffec604,
	      read:(volatile __u32 *) 0xfffec608,
	      feats:TIMER_STD_FEATS,
	      start_mask:TIMER_STD_SMASK,
	      ptv_mask:TIMER_STD_PMASK,
	      mode_mask:TIMER_STD_MMASK,
      irq:INT_TIMER2},
	{
	      cntl:(volatile __u32 *) 0xfffec700,
	      load:(volatile __u32 *) 0xfffec704,
	      read:(volatile __u32 *) 0xfffec708,
	      feats:TIMER_STD_FEATS,
	      start_mask:TIMER_STD_SMASK,
	      ptv_mask:TIMER_STD_PMASK,
	      mode_mask:TIMER_STD_MMASK,
      irq:INT_TIMER3},
};

#endif
