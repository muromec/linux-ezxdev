/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (C) 2002, MontaVista Software <source@mvista.com>.
 *
 * Based on ibm405lp_dpm.c by Bishop Brock, Copyright (C) 2002,
 * International Business Machines Corporation.
 */

#include <linux/config.h>
#include <linux/dpm.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/string.h>

#include <asm/delay.h>
#include <asm/hardirq.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <asm/system.h>
#include <asm/uaccess.h>

#undef DEBUG
#ifdef DEBUG
#define DPRINT(args...) printk(args)
#else
#define DPRINT(args...) do {} while (0)
#endif

#ifdef CONFIG_DPM_DBGPIO
#define DBGPIO_INIT() dbgpio_init()
#define DBGPIO(n, v) dbgpio(n, v)
#else
#define DBGPIO_INIT()
#define DBGPIO(n, v)
#endif /* CONFIG_DPM_DBGPIO */

/* Global variables */

char *dpm_state_names[DPM_STATES] = DPM_STATE_NAMES;

extern dpm_fscaler compute_fscaler(struct dpm_md_opt *cur, struct dpm_md_opt *new);

static int 
dpm_fscale(struct dpm_opt *new, unsigned v, 
	   unsigned relock_wait, struct dpm_opt *relock, 
	   unsigned flags);

#ifdef CONFIG_DPM_UTIMER

static struct fscale_params {
	struct dpm_opt *new;	/* New operating point */
	unsigned v;		/* Current voltage */
	unsigned relock_wait;	/* Relock wait (if any) */
	struct dpm_opt *relock; /* Relock opt (if necessary) */
	int flags;		/* Flags */
} fscale_params;


static void
reenter_fscale(unsigned long arg)
{
	struct fscale_params *params = (struct fscale_params *)arg;
	dpm_fscale(params->new, params->v, 
		   params->relock_wait, params->relock,
		   params->flags);
}

static struct utimer_list set_opt_utimer = {
	function : NULL,
	data     : (unsigned long)&fscale_params,
	flags    : UT_NONE,
	t        : { 0, 0 }
};


static void
sched_fscale(struct dpm_opt *new, unsigned v, 
	     unsigned relock_wait, struct dpm_opt *relock, 
	     unsigned flags, unsigned uS)
{
	fscale_params.new = new;
	fscale_params.v = v;
	fscale_params.relock_wait = relock_wait;
	fscale_params.relock = relock;
	fscale_params.flags = flags;
	set_opt_utimer.function = reenter_fscale;
	set_opt_utimer.t.tv_sec = 0;
	set_opt_utimer.t.tv_nsec = uS * 1000;
	mb();
	if (add_utimer(&set_opt_utimer))
		BUG();
	return;
}

static void
sched_unlock(unsigned uS)
{
	typedef void  (*PFVUL)(unsigned long arg);

	set_opt_utimer.function = (PFVUL)dpm_unlock;
	set_opt_utimer.t.tv_sec = 0;
	set_opt_utimer.t.tv_nsec = uS * 1000;
	mb();
	if (add_utimer(&set_opt_utimer))
		BUG();
	return;
}
	
#endif /* CONFIG_DPM_UTIMER */

/* Set a new operating point.  This call can only be made while the dpm_sem is
   held. 

   The change is made in two phases.  In the first phase, any upward voltage
   scaling takes place.  This may require an asynchronous wait.  

   In the second phase, frequency scaling and downward voltage scaling take
   place. This phase is always entered at a voltage that will support any
   relock operating point and the new operating point. The second phase also
   handles any PLL relocking that may need to occur.  */

		
static struct dpm_opt temp_opt = { name : "[System Operating Point]" };
struct dpm_opt *dpm_relock_opt;

int
dpm_default_set_opt(struct dpm_opt *new, unsigned flags)
{
	unsigned relock_wait, fscale_wait, target_v;
	struct dpm_opt *cur, *relock;
	struct dpm_md_opt *md_cur, *md_new;

	/* Support for setting the operating point when DPM is not running, and
	   setting the first operating point. */

	if (!dpm_enabled || !dpm_active_opt) {
		DPRINT("dpm_default_set_opt: DPM not enabled or no active opt!\n");
		if (dpm_md_get_opt(&temp_opt)) {
			printk(KERN_ERR "dpm_default_set_opt: "
			      "DPM disabled and system "
			      "operating point is illegal!\n");
			return -EINVAL;
		}
		dpm_active_opt = &temp_opt;
	}
	cur = dpm_active_opt;

	DPRINT("dpm_default_set_opt: %s => %s\n", cur->name, new->name);

	md_cur = &cur->md_opt;
	md_new = &new->md_opt;

	relock_wait = dpm_relock_wait(md_cur, md_new);
	if (relock_wait == 0) {
		target_v = md_new->v;
		relock = NULL;
	} else {
		if (dpm_enabled)
			relock = dpm_active_policy->
				classes[DPM_RELOCK_STATE]->opt;
		else if (dpm_relock_opt)
			relock = dpm_relock_opt;
		else {
			printk(KERN_ERR "dpm_default_set_opt: "
			       "An opt. change requires relocking, but no "
			       "relock opt. is available. Aborting.\n");
			return -EINVAL;
		}
		target_v = max(md_new->v, relock->md_opt.v);
	}

	/* If the voltage is increasing we begin the voltage ramp here. */

	if (md_cur->v < target_v) {

		fscale_wait = dpm_bd_set_v_pre(md_cur->v, target_v, flags);

#ifndef CONFIG_DPM_UTIMER
		if (fscale_wait != 0)
			udelay(fscale_wait);
#else
		if (fscale_wait != 0) {
			if (flags & DPM_SYNC)
				udelay(fscale_wait);
			else {
				sched_fscale(new, target_v, relock_wait,
					     relock, flags, fscale_wait);
				return 0;
			}
		}
#endif
	} else {
		target_v = md_cur->v;
	}
	    
	return dpm_fscale(new, target_v, relock_wait, relock, flags);
}

EXPORT_SYMBOL(dpm_default_set_opt);
/* Potential BUG (if uS very big) - needs to be done right, with u64/u32
   division.  Should be part of standard kernel for PPC.  */

static void
udelay_from(u64 start, unsigned uS)
{
	u64 timeout;
	u32 ticks;

	/*
	u64 ticks;
	ticks = (tb_ticks_per_second * uS) / 1000000;
	*/

	ticks = ((tb_ticks_per_second / 1000000) * uS) +
		(((tb_ticks_per_second % 1000000) * uS) / 1000000) +
		2;
	timeout = start + ticks;

	while (dpm_md_time() < timeout);
}
	

static int
dpm_fscale(struct dpm_opt *new, unsigned v, 
	   unsigned relock_wait, struct dpm_opt *relock, unsigned flags)
{
	struct dpm_opt *cur = dpm_active_opt;
	struct dpm_md_opt *md_cur = &cur->md_opt;
	struct dpm_md_opt *md_new = &new->md_opt;
	dpm_fscaler fscaler;
	unsigned fscale_wait, s;

	/* HACK - No notifications for now.  This is where
	   frequency change notifications might take place, either
	   prior to or after the frequency changes, depending on the
	   needs of the device. */

	if (relock_wait != 0) {

		/* We relock using a modified version of the target operating
		   point.  Since we modify the relock point we copy it first.
		   To avoid getting "stuck" at a low-frequency operating point
		   we keep interrupts disabled during the relock.  So that the
		   time is not wasted we go ahead and compute the final
		   fscaler during the relock interval. */

		struct dpm_opt local_relock;
		struct dpm_md_opt *md_local;
		dpm_state_t local_state;
		u64 start;

		local_relock = *relock;
		md_local = &local_relock.md_opt;
		dpm_relock_setup(md_local, md_new);

		fscaler = compute_fscaler(md_cur, md_local);

		critical_save_and_cli(s);

		spin_lock(&dpm_policy_lock);
		fscaler(&md_local->regs);
		start = dpm_md_time();

		local_state = dpm_active_state;
		dpm_quick_enter_state(DPM_RELOCK_STATE);

#ifdef CONFIG_DPM_OPT_STATS
		dpm_update_stats(&relock->stats, &dpm_active_opt->stats);
#endif

		dpm_active_opt = relock;
		fscaler = compute_fscaler(md_local, md_new);
		udelay_from(start, relock_wait);
		dpm_quick_enter_state(local_state);

		spin_unlock(&dpm_policy_lock);

	} else {
		fscaler = compute_fscaler(md_cur, md_new);
		critical_save_and_cli(s);
	}

	fscaler(&md_new->regs);

#ifdef CONFIG_DPM_OPT_STATS
	dpm_update_stats(&new->stats, &dpm_active_opt->stats);
#endif
	dpm_active_opt = new;

	critical_restore_flags(s);

	/* Either finish an upward ramp or initiate downward scaling */

	fscale_wait = dpm_bd_set_v_post(v, md_new->v, flags);
	
#ifndef CONFIG_DPM_UTIMER
	if (fscale_wait != 0) 
		udelay(fscale_wait);
#else
	if (fscale_wait != 0) {
		if (flags & DPM_SYNC)
			udelay(rescale_wait);
		else if (flags & DPM_UNLOCK) {
			mb();
			sched_unlock(rescale_wait);
			return 0;
		}
	}
#endif

	mb();
	if (flags & DPM_UNLOCK)
		dpm_unlock();
	return 0;
}

