/*
 * drivers/dpm/proc.c  Dynamic Power Management /proc
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
 * Bishop Brock
 * IBM Research, Austin Center for Low-Power Computing
 * bcbrock@us.ibm.com
 * September, 2002
 *
 */

#include <linux/dpm.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <asm/semaphore.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>

/* for DPM event queue structures and functions */
#include <linux/device.h>

#define DEBUG
#ifdef DEBUG
#define DPRINT(args...) printk(KERN_CRIT args)
#else
#define DPRINT(args...) do {} while (0)
#endif

/****************************************************************************
 * /proc/driver/dpm interfaces
 *
 * NB: Some of these are borrowed from the 405LP, and may need to be made
 * machine independent.
 ****************************************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * /proc/driver/dpm/cmd (Write-Only)
 *
 * Writing a string to this file is equivalent to issuing a DPM command. 
 * Currently only one command per "write" is allowed, and there is a maximum on
 * the number of tokens that will be accepted (PAGE_SIZE / sizeof(char *)).
 * DPM can be initialized by a linewise copy of a configuration file to this
 * /proc file. 
 *
 * DPM Control
 * -----------
 *
 * init          : dpm_init()
 * enable        : dpm_enable()
 * disable       : dpm_disable()
 * terminate     : dpm_terminate()
 * 
 * Policy Control
 * --------------
 *
 * set_policy <policy>          : Set the policy by name
 * set_task_state <pid> <state> : Set the task state for a given pid, 0 = self
 *
 * Policy Creation
 * ---------------
 *
 * create_opt <name> <pp0> ... <ppn>
 *     Create a named operating point from DPM_PP_NBR paramaters.  All
 *     parameters must be  given. Parameter order and meaning are machine
 *     dependent. 
 *
 * create_class <name> <opt0> [ ... <optn> ]
 *     Create a named class from 1 or more named operating points.  All
 *     operating points must be defined before the call.
 *
 * create_policy <name> <class0> ... <classn>
 *     Create a named policy from DPM_STATES class names.  All classes must be
 *     defined before the call.  The order is machine dependent.
 *
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

static void
pwarn(char *command, int ntoks, char *requirement, int require)
{
	printk(KERN_WARNING "/proc/driver/dpm/cmd: "
	       "Command %s requires %s%d arguments - %d were given\n",
	       command, requirement, require - 1, ntoks - 1);
}


static int 
write_proc_dpm_cmd (struct file *file, const char *buffer,
		    unsigned long count, void *data)
{
	char *buf, *tok, **tokptrs;
	char *whitespace = " \t\r\n";
	int ret = 0, ntoks;

	if (current->uid != 0)
		return -EACCES;
	if (count == 0)
		return 0;
	if (!(buf = kmalloc(count + 1, GFP_KERNEL)))
		return -ENOMEM;
	if (copy_from_user(buf, buffer, count)) {
		ret = -EFAULT;
		goto out0;
	}

	buf[count] = '\0';

	if (!(tokptrs = (char **)__get_free_page(GFP_KERNEL))) {
		ret = -ENOMEM;
		goto out1;
	}

	ret = -EINVAL;
	ntoks = 0;
	do {
		buf = buf + strspn(buf, whitespace);
		tok = strsep(&buf, whitespace);
		if (*tok == '\0') {
			if (ntoks == 0) {
				ret = 0;
				goto out1;
			} else
				break;
		}
		if (ntoks == (PAGE_SIZE / sizeof(char **)))
			goto out1;
		tokptrs[ntoks++] = tok;
	} while(buf);

	if (ntoks == 1) {
		if (strcmp(tokptrs[0], "init") == 0) {
			ret = dpm_init();
		} else if (strcmp(tokptrs[0], "enable") == 0) {
			ret = dpm_enable();
		} else if (strcmp(tokptrs[0], "disable") == 0) {
			ret = dpm_disable();
		} else if (strcmp(tokptrs[0], "terminate") == 0) {
			ret = dpm_terminate();
		}
	} else if (ntoks == 2) {
		if (strcmp(tokptrs[0], "set_policy") == 0)
			ret = dpm_set_policy(tokptrs[1]);
		else if (strcmp(tokptrs[0], "set_state") == 0)
			ret = dpm_set_op_state(tokptrs[1]);
	} else {
		if (strcmp(tokptrs[0], "set_task_state") == 0) {
			if (ntoks != 3) 
				pwarn("set_task_state", ntoks, "", 3);
			else
				ret = dpm_set_task_state(simple_strtol(tokptrs[1],
								       NULL, 0),
							 simple_strtol(tokptrs[2],
								       NULL, 0));
		} else if (strcmp(tokptrs[0], "create_opt") == 0) {
			if (ntoks != DPM_PP_NBR + 2)
				pwarn("create_opt", ntoks, 
				      "", DPM_PP_NBR + 2);
			else {
				dpm_md_pp_t pp[DPM_PP_NBR];
				int i;
			
				for (i = 0; i < DPM_PP_NBR; i++)
					pp[i] = simple_strtol(tokptrs[i + 2],
							      NULL, 0);
				ret = dpm_create_opt(tokptrs[1], pp);
			}

		} else if (strcmp(tokptrs[0], "create_class") == 0) {
			if (ntoks < 3) 
				pwarn("create_class", ntoks, ">= ", 3);
			else 
				ret = dpm_create_class(tokptrs[1], &tokptrs[2],
						       ntoks - 2);

		} else if (strcmp(tokptrs[0], "create_policy") == 0) {
			if (ntoks != (DPM_STATES + 2)) 
				pwarn("create_policy", ntoks, "", 
				      DPM_STATES + 2);
			else 
				ret = dpm_create_policy(tokptrs[1],
							&tokptrs[2]);
		}
	}
out1:
	free_page((unsigned long)tokptrs);
out0:
	kfree(buf);
	if (ret == 0)
		return count;
	else 
		return ret;
}

static int 
write_proc_dpm_force (struct file *file, const char *buffer,
		    unsigned long count, void *data)
{
	char *buf, *tok, **tokptrs;
	char *whitespace = " \t\r\n";
	int ret = 0, ntoks;

	if (current->uid != 0)
		return -EACCES;
	if (count == 0)
		return 0;
	if (!(buf = kmalloc(count + 1, GFP_KERNEL)))
		return -ENOMEM;
	if (copy_from_user(buf, buffer, count)) {
		ret = -EFAULT;
		goto out0;
	}

	buf[count] = '\0';

	if (!(tokptrs = (char **)__get_free_page(GFP_KERNEL))) {
		ret = -ENOMEM;
		goto out1;
	}

	ret = -EINVAL;
	ntoks = 0;
	do {
		buf = buf + strspn(buf, whitespace);
		tok = strsep(&buf, whitespace);
		if (*tok == '\0') {
			if (ntoks == 0) {
				ret = 0;
				goto out1;
			} else
				break;
		}
		if (ntoks == (PAGE_SIZE / sizeof(char **)))
			goto out1;
		tokptrs[ntoks++] = tok;
	} while(buf);

	if (ntoks == 1) {
		if (strcmp(tokptrs[0], "0") == 0) {
			dpm_set_force(0);
		} else if (strcmp(tokptrs[0], "1") == 0) {
			dpm_set_force(1);
		}
	}
out1:
	free_page((unsigned long)tokptrs);
out0:
	kfree(buf);
	if (ret == 0)
		return count;
	else 
		return ret;
}

static int
read_proc_dpm_force(char *page, char **start, off_t offset, 
		    int count, int *eof, void *data)
{
	int len = 0;

	len = sprintf(page, "dpm_force is currently %d\n", dpm_get_force());

	*eof = 1;
	return len;
}

//++++
//extern inline int dpm_event_queue_empty(void);
//struct dpm_device_event_queue;
extern struct dpm_device_event_queue *dpm_event_queue;

static int
read_proc_dpm_device_events(char *page, char **start, off_t offset, 
			    int count, int *eof, void *data)
{
	int len = 0;
	char *event;
	DECLARE_WAITQUEUE(wait, current);

	if (offset)
		return 0;

	if (dpm_event_queue_empty()) {

		/* If the queue is empty, then let the read wait for
		   something to come in */
		add_wait_queue(&dpm_event_queue->proc_list, &wait);

repeat:
		set_current_state(TASK_INTERRUPTIBLE);
		if (dpm_event_queue_empty() && !signal_pending(current)) {
			schedule();
			goto repeat;
		}
		current->state = TASK_RUNNING;
		remove_wait_queue(&dpm_event_queue->proc_list, &wait);
	}
	while (!dpm_event_queue_empty()) {
		event = dpm_get_from_device_event_queue();
		
		/* If we got anything, print it to the proc interface */
		if (event && (strlen(event))) {
			if (len + strlen(event) + 1 < count)
				len += sprintf(page + len, "%s\n", event);
			
			/* The caller of dpm_get_from_device_event_queue is
			   responsible for freeing the returned string */
			kfree(event);
		}
	}

	*eof = 1;
	return len;
}

#ifdef CONFIG_DPM_STATS

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * /proc/driver/dpm/stats (Read-Only)
 *
 * Reading this file produces the following line for each defined operating
 * state:
 * 
 * state_name total_time count opt_name
 * 
 * Where:
 *
 * state_name = The operating state name.
 * total_time = The 64-bit number of dpm_md_time_t ticks spent in this 
 *              operating state.
 * count      = The 64-bit number of times this operating state was entered.
 * opt_name   = The name of the operating point currently assigned to this
 *              operating state.
 *
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

static int
sprintf_u64(char *buf, int fill, char *s, u64 ul)
{
	int len = 0;
	u32 u, l;
	
	u = (u32)((ul >> 32) & 0xffffffffU);
	l = (u32)(ul & 0xffffffffU);

	len += sprintf(buf + len, s);
	if (fill)
		len += sprintf(buf + len, "0x%08x%08x", u, l);
	else {
		if (u)
			len += sprintf(buf + len, "0x%x%x", u, l);
		else
			len += sprintf(buf + len, "0x%x", l);
	}
	return len;
}

static int
read_proc_dpm_stats(char *page, char **start, off_t offset, 
		    int count, int *eof, void *data)
{
	int i, len = 0;
	struct dpm_stats stats[DPM_STATES];

	if (!dpm_enabled) {
		len += sprintf(page + len, "DPM IS DISABLED\n");
		*eof = 1;
		return len;
	}

	dpm_get_os_stats(stats);

	for (i = 0; i < DPM_STATES; i++) {
		len += sprintf(page + len, "%20s", dpm_state_names[i]);
                len += sprintf_u64(page + len, 1, " ", 
				   (u64)stats[i].total_time);
		len += sprintf_u64(page + len, 1, " ", (u64)stats[i].count);
		len += sprintf(page + len, " %s\n", 
			       dpm_active_policy->classes[i]->opt->name);
	}

	*eof = 1;
	return len;
}

#ifdef CONFIG_DPM_OPT_STATS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * /proc/driver/dpm/opt_stats (Read-Only)
 *
 * Reading this file produces the following line for each defined operating
 * point:
 * 
 * name total_time count
 * 
 * Where:
 *
 * name       = The operating point name.
 * total_time = The 64-bit number of dpm_md_time_t ticks spent in this 
 *              operating state.
 * count      = The 64-bit number of times this operating point was entered.
 *
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

static int
read_proc_dpm_opt_stats(char *page, char **start, off_t offset, 
			int count, int *eof, void *data)
{
	int len = 0;
	struct dpm_opt *opt;
	struct list_head *p;
	dpm_md_time_t total_time;

	if (dpm_lock_interruptible())
		return -ERESTARTSYS;

	if (!dpm_enabled) {
		dpm_unlock();
		len += sprintf(page + len, "DPM IS DISABLED\n");
		*eof = 1;
		return len;
	}

	for (p = dpm_opts.next; p != &dpm_opts; p = p->next) {
		opt = list_entry(p, struct dpm_opt, list);
		len += sprintf(page + len, "%s", opt->name);
		total_time = opt->stats.total_time;
		if (opt == dpm_active_opt)
			total_time += dpm_md_time() - opt->stats.start_time;
		len += sprintf_u64(page + len, 0, " ", opt->stats.total_time);
		len += sprintf_u64(page + len, 0, " ", opt->stats.count);
		len += sprintf(page + len, "\n");
	}

	dpm_unlock();
	*eof = 1;
	return len;
}

#endif /* CONFIG_DPM_OPT_STATS */

#ifdef CONFIG_DPM_IDLE_STATS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * /proc/driver/dpm/idle_stats (Read/Write)
 *
 * Read to get a dump of statistics.  Write anything to clear all statistics.
 *
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

static int
sprintf_usec(char *s, u32 tb)
{
	unsigned nsec = (tb * 1000000) / (tb_ticks_per_second / 1000);

	return sprintf(s, " (%d.%03d uS)\n", nsec / 1000, nsec % 1000);
}

static int
read_proc_dpm_idle_stats(char *page, char **start, off_t offset,
		       int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf(page + len, 
		       "Total Idles               : %u\n", 
		       idle_lats.idles);
	len += sprintf(page + len, 
		       "Idle Preemptions          : %u\n", 
		       idle_lats.idle_preemptions);
	len += sprintf(page + len, 
		       "Quick Idles               : %u\n", 
		       idle_lats.quick_idles);
	len += sprintf(page + len, 
		       "Full Idles                : %u\n", 
		       idle_lats.full_idles);
	len += sprintf(page + len, 
		       "Inefficient Idles         : %u\n", 
		       idle_lats.inefficient_idles);
	len += sprintf(page + len, 
		       "Interrupted Idles         : %u\n", 
		       idle_lats.interrupted_idles);
	len += sprintf(page + len, 
		       "Max. Latency to Idle      : %u", 
		       idle_lats.max_latency_to_idle);
	len += sprintf_usec(page + len, idle_lats.max_latency_to_idle);
	len += sprintf(page + len, 
		       "Max. Latency to Idle-Task : %u", 
		       idle_lats.max_latency_to_idle_task);
	len += sprintf_usec(page + len, idle_lats.max_latency_to_idle_task);
	len += sprintf(page + len, 
		       "Max. Crit. Sect. to Idle  : %u", 
		       idle_lats.max_cs_to_idle);
	len += sprintf_usec(page + len, idle_lats.max_cs_to_idle);

	*eof = 1;
	return len;
}


static int 
write_proc_dpm_idle_stats (struct file *file, const char *buffer,
			 unsigned long count, void *data)
{
	unsigned long flags;

	spin_lock_irqsave(&dpm_policy_lock, flags);
	dpm_init_idle_stats();
	spin_unlock_irqrestore(&dpm_policy_lock, flags);
	return count;
}


#endif /* CONFIG_DPM_IDLE_STATS */
#endif /* CONFIG_DPM_STATS */

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * /proc/driver/dpm/state (Read-Only)
 *
 * Reading this file produces the following:
 * 
 * policy_name os os_name class_name os_opt_name opt_name hz
 * 
 * Where:
 *
 * policy_name = The name of the current policy
 * os          = The curret operating state index
 * os_name     = The current operating state name
 * class_name  = The name of the current operating point class
 * os_opt_name = The name of the implied operating point for the policy, class
 *               and state.
 * opt_name    = The name of the actual operating point; may be different if
 *               the operating state and operating point are out of sync.
 * hz          = The frequency of the statistics timer
 *
 * If DPM is disabled the line will appear as:
 *
 * N/A -1 N/A N/A N/A <hz>
 *
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

static int
read_proc_dpm_state(char *page, char **start, off_t offset, 
		    int count, int *eof, void *data)
{
	unsigned long flags;

	int len = 0;

	if (dpm_lock_interruptible())
		return -ERESTARTSYS;

	if (!dpm_enabled) {
		len += sprintf(page + len, "N/A -1 N/A N/A N/A N/A %d\n",
			       DPM_MD_HZ);
	} else {

		spin_lock_irqsave(&dpm_policy_lock, flags);
		len += sprintf(page + len,"%s %d %s %s %s %s %d\n",
			       dpm_active_policy->name, 
			       dpm_active_state,
			       dpm_state_names[dpm_active_state],
			       dpm_active_policy->
			       classes[dpm_active_state]->name, 
			       dpm_active_policy->
			       classes[dpm_active_state]->opt->name,
			       dpm_active_opt?dpm_active_opt->name:
			       "NO ACTIVE OPT",
			       DPM_MD_HZ);
		spin_unlock_irqrestore(&dpm_policy_lock, flags);
	}

	dpm_unlock();
	*eof = 1;
	return len;
}


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * /proc/driver/dpm/debug (Read-Only)
 *
 * Whatever it needs to be
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifdef DEBUG
static int
read_proc_dpm_debug(char *page, char **start, off_t offset, 
		    int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf(page + len, "No DEBUG info\n");
	*eof = 1;
	return len;
}
#endif /* DEBUG */

/****************************************************************************
 * /proc/driver/dpm init/cleanup
 ****************************************************************************/

static struct proc_dir_entry *proc_dpm;
static struct proc_dir_entry *proc_dpm_cmd;
static struct proc_dir_entry *proc_dpm_state;

static struct proc_dir_entry *proc_dpm_force;
static struct proc_dir_entry *proc_dpm_device_events;

#ifdef DPM_MD_PROC_INIT
static struct proc_dir_entry *proc_dpm_md;
#endif

#ifdef CONFIG_DPM_STATS
static struct proc_dir_entry *proc_dpm_stats;
#ifdef CONFIG_DPM_OPT_STATS
static struct proc_dir_entry *proc_dpm_opt_stats;
#endif
#ifdef CONFIG_DPM_IDLE_STATS
static struct proc_dir_entry *proc_dpm_idle_stats;
#endif
#endif

#ifdef DEBUG
static struct proc_dir_entry *proc_dpm_debug;
#endif

#ifdef CONFIG_DPM_TRACE
static struct proc_dir_entry *proc_dpm_trace;
#endif

void __init
dpm_proc_init(void)
{
	proc_dpm = proc_mkdir("driver/dpm", NULL);

	if (proc_dpm) {

		proc_dpm_cmd =
			create_proc_entry("cmd",
					  S_IWUSR,
					  proc_dpm);
		if (proc_dpm_cmd)
			proc_dpm_cmd->write_proc = write_proc_dpm_cmd;

		proc_dpm_force =
			create_proc_entry("force",
					  S_IWUSR,
					  proc_dpm);
		if (proc_dpm_force) {
			proc_dpm_force->write_proc = write_proc_dpm_force;
			proc_dpm_force->read_proc = read_proc_dpm_force;
		}

		proc_dpm_device_events =
			create_proc_read_entry("device-events",
					       S_IRUGO,
					       proc_dpm,
					       read_proc_dpm_device_events,
					       NULL);

		proc_dpm_state =
			create_proc_read_entry("state",
					       S_IRUGO,
					       proc_dpm,
					       read_proc_dpm_state, 
					       NULL); 
#ifdef CONFIG_DPM_STATS
		proc_dpm_stats =
			create_proc_read_entry("stats",
					       S_IRUGO,
					       proc_dpm,
					       read_proc_dpm_stats, 
					       NULL); 
#ifdef CONFIG_DPM_OPT_STATS
		proc_dpm_opt_stats =
			create_proc_read_entry("opt_stats",
					       S_IRUGO,
					       proc_dpm,
					       read_proc_dpm_opt_stats, 
					       NULL); 
#endif /* CONFIG_DPM_OPT_STATS  */

#ifdef CONFIG_DPM_IDLE_STATS
		proc_dpm_idle_stats =
			create_proc_read_entry("idle_stats",
					       S_IWUSR | S_IRUGO,
					       proc_dpm,
					       read_proc_dpm_idle_stats, 
					       NULL); 
		if (proc_dpm_idle_stats)
			proc_dpm_idle_stats->write_proc =
				write_proc_dpm_idle_stats;
#endif /* CONFIG_DPM_IDLE_STATS */

#endif /* CONFIG_DPM_STATS */

#ifdef DEBUG
		proc_dpm_debug =
			create_proc_read_entry("debug",
					       S_IRUGO,
					       proc_dpm,
					       read_proc_dpm_debug, 
					       NULL); 
#endif

#ifdef CONFIG_DPM_TRACE
		proc_dpm_trace =
			create_proc_read_entry("trace",
					       S_IWUSR | S_IRUGO,
					       proc_dpm,
					       read_proc_dpm_trace, 
					       NULL); 
		if (proc_dpm_trace)
			proc_dpm_trace->write_proc = write_proc_dpm_trace;
#endif

#ifdef DPM_MD_PROC_INIT
		proc_dpm_md = proc_mkdir("md", proc_dpm);
		DPM_MD_PROC_INIT(proc_dpm_md);
#endif
	} else {
	  printk(KERN_ERR "Attempt to create /proc/driver/dpm failed\n");

	}
}

#ifdef MODULE
void __exit
#else
void 
#endif
dpm_proc_cleanup(void)
{
	if (proc_dpm_cmd) {
		remove_proc_entry("cmd", proc_dpm);
		proc_dpm_cmd = NULL;
	}

	if (proc_dpm_state) {
		remove_proc_entry("state", proc_dpm);
		proc_dpm_state = NULL;
	}

#ifdef CONFIG_DPM_STATS
	if (proc_dpm_stats) {
		remove_proc_entry("stats", proc_dpm);
		proc_dpm_stats = NULL;
	}

#ifdef CONFIG_DPM_OPT_STATS
	if (proc_dpm_opt_stats) {
		remove_proc_entry("opt_stats", proc_dpm);
		proc_dpm_opt_stats = NULL;
	}
#endif /*CONFIG_DPM_OPT_STATS  */

#ifdef CONFIG_DPM_IDLE_STATS
	if (proc_dpm_idle_stats) {
		remove_proc_entry("idle_stats", proc_dpm);
		proc_dpm_idle_stats = NULL;
	}
#endif /* CONFIG_DPM_IDLE_STATS */
#endif /* CONFIG_DPM_STATS */

#ifdef DEBUG
	if (proc_dpm_debug) {
		remove_proc_entry("debug", proc_dpm);
		proc_dpm_debug = NULL;
	}
#endif

#ifdef CONFIG_DPM_TRACE
	if (proc_dpm_trace) {
		remove_proc_entry("trace", proc_dpm);
		proc_dpm_trace = NULL;
	}
#endif

#ifdef DPM_MD_PROC_CLEANUP
	DPM_MD_PROC_CLEANUP(proc_dpm_md);
	remove_proc_entry("md", proc_dpm);
#endif

	remove_proc_entry("driver/dpm", NULL);
}



/****************************************************************************
 * Machine-dependent /proc/driver/dpm/md entries
 ****************************************************************************/

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * /proc/driver/dpm/md/opts (Read-only)
 *
 * Reading this file will produce a dump of the current operating point, and a
 * listing of all of the defined operating points.
 *
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


static struct proc_dir_entry *proc_dpm_md_opts;
static struct proc_dir_entry *proc_dpm_md_cmd;

extern int read_proc_dpm_md_opts(char *page, char **start, off_t offset,
				 int count, int *eof, void *data);
extern int write_proc_dpm_md_cmd (struct file *file, const char *buffer,
				  unsigned long count, void *data);


void __init
dpm_generic_md_proc_init(struct proc_dir_entry *proc_dpm_md)
{
	proc_dpm_md_opts =
		create_proc_read_entry("opts",
				       S_IRUGO,
				       proc_dpm_md,
				       read_proc_dpm_md_opts, 
				       NULL); 

	proc_dpm_md_cmd =
		create_proc_entry("cmd",
				  S_IWUSR,
				  proc_dpm_md);
	if (proc_dpm_md_cmd)
		proc_dpm_md_cmd->write_proc = write_proc_dpm_md_cmd;
}


#ifdef MODULE
void __exit
#else
void
#endif
dpm_generic_md_proc_cleanup(struct proc_dir_entry *proc_dpm_md)
{
	if (proc_dpm_md_opts) {
		remove_proc_entry("opts", proc_dpm_md);
		proc_dpm_md_opts = NULL;
	}

	if (proc_dpm_md_cmd) {
		remove_proc_entry("cmd", proc_dpm_md);
		proc_dpm_md_cmd = NULL;
	}
}

/* sysfs DPM functions */


struct dpm_attribute {
        struct attribute        attr;
        ssize_t (*show)(char * buf, size_t count, loff_t off);
        ssize_t (*store)(const char * buf, size_t count, loff_t off);
};

#define DPM_ATTR(_name,_ds,_mode,_show,_store) \
struct dpm_attribute dpm_attr_##_ds = {            \
        .attr = {.name = __stringify(_name), .mode = _mode },   \
        .show   = _show,                                \
        .store  = _store,                               \
};

static ssize_t
dpm_force_show(char *buf, size_t count, loff_t offset)
{
	return offset ? 0 : sprintf(buf, "%d\n", dpm_get_force());
}


static ssize_t
dpm_force_store(const char *buf, size_t count, loff_t offset)
{
	int state;

	if (current->uid != 0)
		return -EACCES;
	if (count == 0)
		return 0;

	if (sscanf(buf, "%d", &state) != 1)
		return -EINVAL;

	dpm_set_force(state);
	return count;
}

static DPM_ATTR(force,force,S_IWUSR | S_IRUGO, dpm_force_show, 
		dpm_force_store);


static ssize_t
dpm_device_event_show(char *buf, size_t count, loff_t offset)
{
	int len = 0;
	char *event;
	DECLARE_WAITQUEUE(wait, current);

	if (offset)
		return 0;

	if (dpm_event_queue_empty()) {

		/* If the queue is empty, then let the read wait for
		   something to come in */
		add_wait_queue(&dpm_event_queue->proc_list, &wait);

repeat:
		set_current_state(TASK_INTERRUPTIBLE);
		if (dpm_event_queue_empty() && !signal_pending(current)) {
			schedule();
			goto repeat;
		}
		current->state = TASK_RUNNING;
		remove_wait_queue(&dpm_event_queue->proc_list, &wait);
	}
	while (!dpm_event_queue_empty()) {
		event = dpm_get_from_device_event_queue();
		
		/* If we got anything, print it */
		if (event && (strlen(event))) {
			if (len + strlen(event) + 1 < count)
				len += sprintf(buf + len, "%s\n", event);
			
			/* The caller of dpm_get_from_device_event_queue is
			   responsible for freeing the returned string */
			kfree(event);
		}
	}

	return len;
}

static DPM_ATTR(device-event,device_event,S_IWUSR | S_IRUGO, 
		dpm_device_event_show, NULL);

#define to_dpm_attr(_attr) container_of(_attr,struct dpm_attribute,attr)

static ssize_t
dpm_attr_show(struct kobject * kobj, struct attribute * attr,
              char * buf, size_t count, loff_t off)
{
        struct dpm_attribute * dpm_attr = to_dpm_attr(attr);
        ssize_t ret = 0;

        if (dpm_attr->show)
                ret = dpm_attr->show(buf,count,off);
        return ret;
}
                                                                                
static ssize_t
dpm_attr_store(struct kobject * kobj, struct attribute * attr,
               const char * buf, size_t count, loff_t off)
{
        struct dpm_attribute * dpm_attr = to_dpm_attr(attr);
        ssize_t ret = 0;
                                                                                
        if (dpm_attr->store)
                ret = dpm_attr->store(buf,count,off);
        return ret;
}


static struct attribute * dpm_default_attrs[] = {
        &dpm_attr_force.attr,
        &dpm_attr_device_event.attr,
        NULL,
};

static struct sysfs_ops dpm_sysfs_ops = {
        .show   = dpm_attr_show,
        .store  = dpm_attr_store,
};


static struct subsystem dpm_subsys = {
        .kobj           = {
                .name   = "dpm",
        },
        .sysfs_ops      = &dpm_sysfs_ops,
	.default_attrs  = dpm_default_attrs,
};

static struct kobject dpm_control = {
	.name = "control",
	.subsys = &dpm_subsys,
};

static int __init dpm_sysfs_init(void)
{
	int err;

        if ((err = subsystem_register(&dpm_subsys)) == 0) {
		err = kobject_register(&dpm_control);
	}

	return err;
}

int proc_pid_dpm(struct task_struct *task, char * buffer)
{
	int res;
	res = sprintf(buffer,"%d \n", task->dpm_state);
	return res;
}

__initcall(dpm_sysfs_init);

/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
