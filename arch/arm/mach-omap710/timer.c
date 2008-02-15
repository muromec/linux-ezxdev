/*
 * File: timer.c
 *
 * A timer klib driver
 *
 * DESCRIPTION                                                                   
 *    "Kernel library for the timer hardware"
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

/*****************************************************************************
         I n c l u d e s                                                      
 *****************************************************************************/
#include <asm/arch/timer.h>
#include <asm/arch/ck.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <asm/errno.h>
#include <asm/page.h>		/* for BUG (a hack since BUG is meant for mem errors) */

#define __TIMER_INTERNAL__
#include "arch_timer.h"

/***************************************************************************** 
         P r i v a t e
 *****************************************************************************/

typedef struct {
	struct list_head list;
	int timer;
	timer_cb_t cb;
	void *data;
} timer_cb_info_t;

static struct list_head __timer_cb_table[TIMER_N];

static int __timer_requested_irq;
static int __timer_running;

static inline int
__timer_get_divisor(int timer)
{
	volatile __u32 *reg;
	int ptv, ret = 2;

	reg = TIMER_CNTL_REG(timer);

	if (TIMER_IS_WDG(timer))
		ptv = (*reg & (7 << 9)) >> 9;
	else
		ptv = (*reg & (7 << 2)) >> 2;

	while (ptv--)
		ret *= 2;

	return ret;
}

static void
__timer_isr(int irq, void *data, struct pt_regs *regs)
{
	timer_cb_info_t *ptr;

	list_for_each(((struct list_head *) ptr), (struct list_head *) data)
	    (ptr->cb) (ptr->timer, ptr->data);
}

/*****************************************************************************
         D r i v e r    I m p l                                               
 *****************************************************************************/
int
timer_get_num_timers(void)
{
	return TIMER_N;
}

int
timer_get_features(int timer, int *feats)
{
	if (!TIMER_CHECK(timer))
		return -EINVAL;
	return TIMER_FEATS(timer);
}

int
timer_get_irq_no(int timer)
{
	if (!TIMER_CHECK(timer))
		return -EINVAL;
	return TIMER_IRQ(timer);
}

int
timer_start(int timer)
{
	volatile __u32 *reg;

	if (!TIMER_CHECK(timer))
		return -EINVAL;

	// If no timer is running, we'll need to enable our clock
	if (!(__timer_running)) {
		ck_enable(TIMER_CK(timer));
	}
	reg = TIMER_CNTL_REG(timer);
	*reg |= (TIMER_START_MASK(timer));
	__timer_running |= (1 << timer);

	return 0;
}

int
timer_stop(int timer)
{
	volatile __u32 *reg;

	if (!TIMER_CHECK(timer))
		return -EINVAL;

	reg = TIMER_CNTL_REG(timer);
	*reg &= ~(TIMER_START_MASK(timer));

	// If no timers are running, turn off our clock.
	__timer_running &= ~(1 << timer);
	if (!(__timer_running)) {
		ck_disable(TIMER_CK(timer));
	}

	return 0;
}

int
timer_is_running(int timer)
{
	volatile __u32 *reg;

	if (!TIMER_CHECK(timer))
		return -EINVAL;

	reg = TIMER_CNTL_REG(timer);
	return (((*reg) & TIMER_START_MASK(timer)) == TIMER_START_MASK(timer));
}

int
timer_set_mode(int timer, timer_mode_t mode)
{
	volatile __u32 *reg;
	int ret = 0;

	if (!TIMER_CHECK(timer))
		return -EINVAL;

	reg = TIMER_CNTL_REG(timer);

	switch (mode) {
	case timer_mode_oneshot:
		*reg &= ~TIMER_MODE_MASK(timer);
		break;
	case timer_mode_periodic:
		*reg |= TIMER_MODE_MASK(timer);
		break;
	default:
		ret = -ENOSYS;
	}

	return ret;
}

int
timer_get_mode(int timer, timer_mode_t * mode)
{
	volatile __u32 *reg;

	if (!TIMER_CHECK(timer))
		return -EINVAL;

	if (!mode)
		return -EFAULT;

	reg = TIMER_CNTL_REG(timer);

	if (*reg & TIMER_MODE_MASK(timer))
		*mode = timer_mode_periodic;
	else
		*mode = timer_mode_oneshot;

	return 0;
}

int
timer_set_action(int timer, timer_action_t action)
{
	volatile __u32 *reg;

	if (!TIMER_CHECK(timer))
		return -EINVAL;

	if (TIMER_CAN_RST(timer)) {
		reg = TIMER_MODE_REG(timer);

		if (action == timer_action_reset) {
			*reg = (1 << 15);	/* activate reset mode */
		} else {
			*reg = 0xf5;	/* activate interrupt mode */
			*reg = 0xa0;
		}
	} else {
		/* handle non-watchdog timers */
		if (~(TIMER_CAN_RST(timer))) {
			if (action == timer_action_interrupt)
				return 0;
			else
				return -EINVAL;
		}
	}

	return 0;
}

int
timer_get_action(int timer, timer_action_t * action)
{
	volatile __u32 *reg;

	if (!TIMER_CHECK(timer))
		return -EINVAL;

	if (!action)
		return -EFAULT;

	if (TIMER_CAN_RST(timer)) {

		reg = TIMER_CNTL_REG(timer);

		if (*reg & (1 << 15))
			*action = timer_action_reset;
		else
			*action = timer_action_interrupt;

	} else {
		/* handle non-watchdog timers */
		if (~(TIMER_CAN_RST(timer))) {
			*action = timer_action_interrupt;
			return 0;
		}
	}
	return 0;
}
static int
__timer_set_wdg_period(int timer, unsigned long usecs)
{
	volatile __u32 *reg;
	int div, ptv, clock;

	if (!TIMER_CHECK(timer) || !usecs)
		return -EINVAL;

	/* First get the ticks per nsec of the current clock. */
	if ((clock = ck_get_rate(TIMER_CK(timer))) < 0)
		return -EIO;
	clock = (clock * 1000) / 14;

	ptv = 0;
	div = 2;

	/* If we need to increment the ptv... don't forget that our clock
	   units are in tick/nsec, so muliply the right hand by 1000 to
	   convert to usecs.
	 */
	while ((usecs / div) > (1000 * (TIMER_COUNT_MAX(timer) / clock))) {
		if (ptv == TIMER_MAX_PTV) {
			/* Well, this can happen if somebody tries to set the
			   watchdog timer to too high of a period. Since the
			   watchdog only has a 16 bit count register we can't
			   go quite as high as the normal timers with 32 bit
			   count registers.
			 */
			return -ERANGE;
		}
		div *= 2;
		ptv++;
	}

	reg = TIMER_CNTL_REG(timer);

	/* Set the PTV */
	if (TIMER_IS_WDG(timer)) {
		*reg &= ~(7 << 9);
		*reg |= (ptv << 9);
	} else {
		*reg &= ~(7 << 2);
		*reg |= (ptv << 2);
	}

	/* If Method #2 will give us an underflow... */
	if (usecs < div) {
		/* Method #1: since the max div is 256 we know that usecs is
		   less than 1000, so this will not overflow 
		 */
		usecs = usecs * (clock / div);
	} else {
		/* Method #2: since the max usecs is 0xffff (or we would have
		   detected an error when calculating the ptv above) we know
		   this will not overflow a 32-bit:
		 */
		usecs = (usecs / div) * clock;
	}

	/* ... and convert from ticks/nsecs to ticks/usecs */
	usecs /= 1000;

	/* Set the count */
	reg = TIMER_LOAD_REG(timer);
	*reg = usecs;

	return 0;
}

int
timer_set_period(int timer, unsigned long usecs)
{
	volatile __u32 *reg;
	int div, ptv, clock;

	if (!TIMER_CHECK(timer) || !usecs)
		return -EINVAL;

	if (TIMER_IS_WDG(timer))
		return __timer_set_wdg_period(timer, usecs);

	/* The count we must program to give us the desired period is given
	   by this formula:

	   count = period * clock / (2 ^ ptv)

	   If you manipulate this formula you can make a table like this, which
	   applies to the omap1510 h/w:

	   clock  range
	   -----  ------------------
	   12     167 ns - 10.6 days
	   15     133 ns - 8.4 days
	   30     66 ns - 4.24 days
	   60     33 ns - 2.12 days
	   120    17 ns - 1.06 days

	   Since we take about 50 nanoseconds per instruction on an omap1510
	   (based on 50 BogoMIPS) we might as well just use the slow clock for
	   everything. Incidentally, we could get a more accurate BogoMIPs 
	   calculation using the fast clock...

	   So now we just need to calculate the ptv.

	   ptv  range
	   ---  -----
	   0    167 ns - 715 sec (almost 2 hours)
	   2    334 ns - 1430 sec
	   etc...

	   So to prevent overflow we'll need to increment the ptv to meet this
	   constraint:

	   count <= MAX

	   Which we can rearrange until we can get something we can test for
	   without having to worry about overflow or integer round-to-zero:

	   period * clock / (2 ^ (ptv + 1)) <= MAX
	   period * clock / div <= MAX
	   period * clock <= MAX * div
	   period / div <= MAX / clock           
	 */

	/* First get the ticks per usec of the current clock. */
	if ((clock = ck_get_rate(TIMER_CK(timer))) < 0)
		return -EIO;

	ptv = 0;
	div = 2;

	/* If we need to increment the ptv... */
	while ((usecs / div) > (TIMER_COUNT_MAX(timer) / clock)) {
		if (ptv == TIMER_MAX_PTV) {
			/* Well, this can happen if somebody tries to set the
			   watchdog timer to too high of a period. Since the
			   watchdog only has a 16 bit count register we can't
			   go quite as high as the normal timers with 32 bit
			   count registers.
			 */
			return -ERANGE;
		}
		div *= 2;
		ptv++;
	}

	reg = TIMER_CNTL_REG(timer);

	/* Set the PTV */
	if (TIMER_IS_WDG(timer)) {
		*reg &= ~(7 << 9);
		*reg |= (ptv << 9);
	} else {
		*reg &= ~(7 << 2);
		*reg |= (ptv << 2);
	}

	/* If we need to avoid underflow... */
	if (usecs < div)
		usecs = (usecs * clock) / div;
	/* ...else avoid overflow... */
	else
		usecs = (usecs / div) * clock;

	/* Set the count */
	reg = TIMER_LOAD_REG(timer);
	*reg = usecs;

	return 0;
}

int
__timer_get_wdg_time(int timer, unsigned long *usecs, volatile __u32 * reg)
{
	/* Hack -- use a special routine for the wdt since it's clock is less
	   than one MHz (< 1 tick per usec). If we ask the clock driver what
	   the clock is it will return 0, which isn't much good for dividing
	   the count. So we hardcode knowledge that the wdt clock is CK_CLKIN
	   divided by 14. *** FIXME ***
	 */
	int clock, div;
	__u32 count;

	if (!usecs)
		return -EFAULT;

	/* Read the count. */
	count = *reg;

	/* Get the clock rate in units of ticks per nsecs. */
	if ((clock = ck_get_rate(TIMER_CK(timer))) < 0)
		return -EIO;
	clock = (clock * 1000) / 14;

	if (clock == 0)
		return -ERANGE;

	/* Get the clock divider. */
	div = __timer_get_divisor(timer);

	/* If we need to avoid underflow... */
	if (count < clock)
		*usecs = ((count * div) / clock);

	/* ...else avoid overflow... */
	else
		*usecs = ((count / clock) * div);

	/* ...and convert back to usecs */
	*usecs *= 1000;

	return 0;
}

int
__timer_get_time(int timer, unsigned long *usecs, volatile __u32 * reg)
{
	int clock, div;
	__u32 count;

	if (!usecs)
		return -EFAULT;

	/* The actual value WRT the units is given by the formula:
	   period = count * div / clock
	 */

	/* Read the count. */
	count = *reg;

	/* Get the clock rate in units of ticks per usecs. */
	if ((clock = ck_get_rate(TIMER_CK(timer))) < 0)
		return -EIO;

	if (clock == 0)
		return -ERANGE;

	/* Get the clock divider. */
	div = __timer_get_divisor(timer);

	/* If we need to avoid underflow... */
	if (count < clock)
		*usecs = ((count * div) / clock);

	/* ...else avoid overflow... */
	else
		*usecs = ((count / clock) * div);

	return 0;
}

int
timer_get_period(int timer, unsigned long *usecs)
{
	if (!TIMER_CHECK(timer))
		return -EINVAL;

	/* For the wdt the load register is write-only. */
	if (TIMER_IS_WDG(timer))
		return -ENOSYS;

	return __timer_get_time(timer, usecs, TIMER_LOAD_REG(timer));
}

int
timer_get_time_remaining(int timer, unsigned long *usecs)
{
	if (!TIMER_CHECK(timer))
		return -EINVAL;

	if (TIMER_IS_WDG(timer))
		return __timer_get_wdg_time(timer, usecs,
					    TIMER_READ_REG(timer));

	return __timer_get_time(timer, usecs, TIMER_READ_REG(timer));
}

int
timer_register_cb(int timer, timer_cb_t cb, void *data)
{
	timer_cb_info_t *info;
	struct list_head *head;
	int ret = 0, flags;

	if (!TIMER_CHECK(timer) || !cb)
		return -EINVAL;

	if (!(info = kmalloc(sizeof (timer_cb_info_t), GFP_KERNEL)))
		return -ENOMEM;

	/***************************************************/
	save_flags_cli(flags);

	head = &__timer_cb_table[timer];

	if (list_empty(head)) {
		if ((ret = request_irq(TIMER_IRQ(timer), &__timer_isr, 0,
				       "timer",
				       &__timer_cb_table[timer])) < 0) {
			restore_flags(flags);
			goto abort;
		}
		__timer_requested_irq |= (1 << timer);
	}

	list_add((struct list_head *) info, head);

	restore_flags(flags);
	/***************************************************/

	info->timer = timer;
	info->cb = cb;
	info->data = data;

	goto exit;

      abort:
	kfree(info);

      exit:
	return ret;
}

int
timer_unregister_cb(int timer, void *data)
{
	timer_cb_info_t *ptr;
	struct list_head *head;
	int flags;

	if (!TIMER_CHECK(timer))
		return -EINVAL;

	/***************************************************/
	save_flags_cli(flags);

	head = &__timer_cb_table[timer];

	list_for_each(((struct list_head *) ptr), head) {
		if (ptr->data != data)
			continue;
		list_del((struct list_head *) ptr);
		kfree(ptr);
		break;
	}

	restore_flags(flags);
	/***************************************************/

	return 0;
}

/*****************************************************************************
         M o d u l e                                                          
 *****************************************************************************/
static int __init
init_omap1510_timer(void)
{
	int timer;

	printk("DSPLinux Timer (c) 2001 RidgeRun, Inc.\n");

	for (timer = 0; timer < TIMER_N; timer++)
		INIT_LIST_HEAD(&__timer_cb_table[timer]);

	return 0;
}

static void __exit
exit_omap1510_timer(void)
{
	int i;

	for (i = 0; i < TIMER_N; i++) {
		timer_stop(i);
		// only free timers if you've requested one.  
		if (__timer_requested_irq & (1 << i)) {
			free_irq(TIMER_IRQ(i), &__timer_cb_table[i]);
		}
	}
	if (!(__timer_running)) {
		ck_disable(TIMER_CK(1));
	}
}

module_init(init_omap1510_timer);
module_exit(exit_omap1510_timer);
EXPORT_SYMBOL(timer_start);
EXPORT_SYMBOL(timer_stop);
EXPORT_SYMBOL(timer_get_mode);
EXPORT_SYMBOL(timer_set_mode);
EXPORT_SYMBOL(timer_get_period);
EXPORT_SYMBOL(timer_get_time_remaining);
EXPORT_SYMBOL(timer_get_action);
EXPORT_SYMBOL(timer_is_running);

EXPORT_SYMBOL(timer_get_num_timers);
EXPORT_SYMBOL(timer_set_period);
EXPORT_SYMBOL(timer_get_features);
EXPORT_SYMBOL(timer_register_cb);
EXPORT_SYMBOL(timer_unregister_cb);
EXPORT_SYMBOL(timer_set_action);
