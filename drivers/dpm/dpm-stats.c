/*
 * drivers/dpm/policy.c  Dynamic Power Management Policies
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
 * Copyright (C) 2002, International Business Machines Corporation
 * All Rights Reserved
 *
 * Robert Paulsen
 * IBM Linux Technology Center
 * rpaulsen@us.ibm.com
 * August, 2002
 *
 */

/* TODO:

   Rethink init/enable/disable: It may be redundant and/or unsafe
   Fix initialization and stats
*/

#include <linux/dpm.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <asm/semaphore.h>
#include <asm/system.h>
#include <asm/uaccess.h>

// debug printout
// #define TRACE 1
#undef TRACE
#if defined(TRACE)
#define trace(args...) do { printk("TRACE: "); printk(args); } while(0)
#else
#define trace(args...) do {} while(0)
#endif

#define DPM_PP_SIZE (DPM_PP_NBR * sizeof(dpm_md_pp_t))

struct dpm_stats dpm_state_stats[DPM_STATES];

dpm_md_time_t
dpm_update_stats(struct dpm_stats *new, struct dpm_stats *old)
{
	dpm_md_time_t now = dpm_md_time();

	old->end_time = now;
	old->total_time += now - old->start_time;
	new->start_time = now;
	new->end_time = 0;
	new->count += 1;

	return now;
}

/*****************************************************************************
 * get a policy's statistics
 *****************************************************************************/

int
dpm_get_policy_stats(char *name, struct dpm_stats *stats)
{
	struct dpm_policy *policy;

	dpm_lock();

	/* find the named policy */
	list_find(policy, name, dpm_policies, struct dpm_policy);
	if (!policy) {
		dpm_unlock();
		return -ENOENT;
	}

	*stats = policy->stats;

	dpm_unlock();
	return 0;
}

/*****************************************************************************
 * get a class's statistics
 *****************************************************************************/

int
dpm_get_class_stats(char *name, struct dpm_stats *stats)
{
	struct dpm_class *cls;

	dpm_lock();

	/* find the named class */
	list_find(cls, name, dpm_classes, struct dpm_class);
	if (!cls) {
		dpm_unlock();
		return -ENOENT;
	}

	*stats = cls->stats;

	dpm_unlock();
	return 0;
}

/*****************************************************************************
 * get a operating point's statistics
 *****************************************************************************/

int
dpm_get_opt_stats(char *name, struct dpm_stats *stats)
{
	struct dpm_opt *opt;

	dpm_lock();

	/* find the named opt */
	list_find(opt, name, dpm_opts, struct dpm_opt);
	if (!opt) {
		dpm_unlock();
		return -ENOENT;
	}

	*stats = opt->stats;
	stats->total_time += dpm_md_time() - stats->start_time;
	dpm_unlock();
	return 0;
}

/*****************************************************************************
 * get statistics for all operating states
 *****************************************************************************/

int
dpm_get_os_stats(struct dpm_stats *stats)
{
	unsigned long flags;

	spin_lock_irqsave(&dpm_policy_lock, flags);
	memcpy(stats, dpm_state_stats, DPM_STATES * sizeof (struct dpm_stats));
	stats[dpm_active_state].total_time +=
		dpm_md_time() - stats[dpm_active_state].start_time;
	spin_unlock_irqrestore(&dpm_policy_lock, flags);
	return 0;
}

EXPORT_SYMBOL(dpm_get_os_stats);

void
dpm_init_stats(void) 
{
	memset(dpm_state_stats, 0, DPM_STATES * sizeof (struct dpm_stats));
#ifdef CONFIG_DPM_IDLE_STATS
	dpm_init_idle_stats();
#endif
}

#ifdef CONFIG_DPM_IDLE_STATS
void
dpm_init_idle_stats(void)
{
	memset((void *)&idle_lats, 0, sizeof(struct dpm_idle_lats));
}
#endif	

