/*
 * include/linux/dpm.h  DPM policy management
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

#ifndef __DPM_STATS_H__
#define __DPM_STATS_H__

/* Machine dependent timing/counts.  If the machine has not defined
   DPM_MD_STATS, we use a generic routine based on gettimeofday and 64-bit
   values.  Depending on how DPM is being used this may be really slow,
   though. */

#ifndef DPM_MD_STATS
typedef __u64 dpm_md_time_t;
typedef __u64 dpm_md_count_t;
#endif

/* statistics */
struct dpm_stats {
        dpm_md_count_t	count;
        dpm_md_time_t	total_time;
        dpm_md_time_t	start_time;
        dpm_md_time_t	end_time;
};

/* Kernel-only defines */

#ifdef __KERNEL__

#ifndef DPM_MD_STATS
extern inline dpm_md_time_t
dpm_md_time()
{
	struct timeval tv;
	u64 time;
	do_gettimeofday(&tv);
	time = ((u64)tv.tv_sec * 1000000) + tv.tv_usec;
	return time;
}
	
#define DPM_MD_HZ 1000000
#endif /* DPM_MD_STATS */

/*****************************************************************************
 * operating state statistics
 *****************************************************************************/

extern struct dpm_stats dpm_state_stats[DPM_STATES];

/* get statistics for a policy */
int dpm_get_policy_stats(char *name, struct dpm_stats *stats);

/* get statistics for a class */
int dpm_get_class_stats(char *name, struct dpm_stats *stats);

/* get statistics for an operating point */
int dpm_get_opt_stats(char *name, struct dpm_stats *stats);

/* get statistics for all operating states */
int dpm_get_os_stats(struct dpm_stats *stats);

/* update statistics structures */
dpm_md_time_t
dpm_update_stats(struct dpm_stats *new, struct dpm_stats *old);

/* initialize operating state statistics */
void dpm_init_stats(void);

/* initialize idle state statistics */
void dpm_init_idle_stats(void);

/*
 * idle statistics
 */
struct dpm_idle_lats {
	unsigned idles;
	unsigned idle_preemptions;
	unsigned quick_idles;
	unsigned full_idles;
	unsigned inefficient_idles;
	unsigned interrupted_idles;
	unsigned max_latency_to_idle;
	unsigned max_latency_to_idle_task;
	unsigned max_cs_to_idle;
};

extern struct dpm_idle_lats idle_lats;

#endif /*__KERNEL__*/
#endif /*__DPM_STATS_H__*/
