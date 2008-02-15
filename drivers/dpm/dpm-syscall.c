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

static void
display_opt(char *prefix, struct dpm_opt *opt)
{
	int i;
	printk("%s%sOPT %s is ", prefix, prefix, opt->name);
	for (i = 0; i < DPM_PP_NBR; ++i)
		printk("%04d ", opt->pp[i]);
	printk("\n");
}

/*****************************************************************************
 * display a class
 *****************************************************************************/
static void
display_class(char *prefix, struct dpm_class *cls)
{
	int i;
	printk("%sClass: %s\n", prefix, cls->name);
	printk("%s\tentry count: %lld\n", prefix, cls->stats.count);
	printk("%s\t total time: %lld\n", prefix, cls->stats.total_time);
	printk("%s\t start time: %lld\n", prefix, cls->stats.start_time);
	printk("%s\t   end time: %lld\n", prefix, cls->stats.end_time);

	if (cls->opt)
		printk("%s\tThe selected opt for this class is %s\n",
		       prefix, cls->opt->name);
	else
		printk("%s\tThis class has no selected opt\n", prefix);
	printk("%s\tThis class has %d operating point%s:\n", prefix, cls->nops,
	       cls->nops == 1 ? "" : "s");
	for (i = 0; i < cls->nops; ++i)
		display_opt(prefix, cls->ops[i]);
}

/*****************************************************************************
 * display a policy (only needed for testing/debug)
 * (left off static so it can be used by dpm-constr.c)
 *****************************************************************************/
void
display_policy(struct dpm_policy *policy)
{
	int i;
	printk("Policy: %s\n", policy->name);
	printk("\tentry count: %lld\n", policy->stats.count);
	printk("\t total time: %lld\n", policy->stats.total_time);
	printk("\t start time: %lld\n", policy->stats.start_time);
	printk("\t   end time: %lld\n", policy->stats.end_time);
	printk("\tThere are %d Op states. The last %d are task states\n",
	       DPM_STATES, DPM_TASK_STATES);
	for (i = 0; i < DPM_STATES; ++i)
		display_class("\t", policy->classes[i]);
}

/*****************************************************************************
 * Copy a name or names from user space and check for validity. Call free_name
 * or free_names to free when finished with the memory.
 *****************************************************************************/
static int
check_name(char *name)
{
	/* Check for whitespace in the name.  This is disallowed in DPM names
	   in order to simplify text-based processing of reports. Don't want to
	   use strsep() etc. as they mung the string. */

	for (; *name != '\0'; name++)
		if ((*name == ' ') || (*name == '\t') ||
		    (*name == '\r') || (*name == '\n'))
			return -EINVAL;
	return 0;
}

static void
free_name(char *name)
{
	kfree(name);
}

static int
u_get_name(char **name, const char *u_name)
{
	/* Valid for PPC, but why is strnlen_user() defined differently from 
	   the strnlen() library call?  This one includes the \0 at the end. */

	int len = strnlen_user(u_name, DPM_NAME_SIZE);
	int ret;

	if (len > DPM_NAME_SIZE) {
		trace("Name too long\n");
		return -EINVAL;
	}
	if (!(*name = (char *) kmalloc(len, GFP_KERNEL))) {
		trace("No memory\n");
		return -ENOMEM;
	}
	if (copy_from_user(*name, u_name, len)) {
		trace("u_get_name fault\n");
		ret = -EFAULT;
		goto free_name;
	}
	if (check_name(*name)) {
		trace("name doesn't check out\n");
		ret = -EINVAL;
		goto free_name;
	}
	return 0;
free_name:
	free_name(*name);
	return ret;
}

static void
free_names(char **names, int n)
{
	while (n)
		free_name(names[--n]);
	kfree(names);
}

static int
u_get_names(char ***names, char **u_names, int n)
{
	char **local_names;
	char *local_name;
	int i, ret;

	if (!(local_names = kmalloc(n * sizeof (char *), GFP_KERNEL)))
		return -ENOMEM;
	memset(local_names, 0, n * sizeof (char *));
	for (i = 0; i < n; i++) {
		ret = -EFAULT;
		if (copy_from_user(&local_name, &u_names[i], sizeof (char *)))
			goto free_names;
		if ((ret = u_get_name(&local_names[i], local_name)))
			goto free_names;
	}
	*names = local_names;
	return 0;
free_names:
	free_names(local_names, i);
	return ret;
}

static int
u_dpm_create_opt(const char *u_name, const dpm_md_pp_t * u_md_pp)
{
	int ret;
	char *name;
	dpm_md_pp_t *md_pp;

	if ((ret = u_get_name(&name, u_name)))
		return ret;
	ret = -ENOMEM;
	if (!(md_pp = kmalloc(DPM_PP_SIZE, GFP_KERNEL)))
		goto free_name;
	ret = -EFAULT;
	if (copy_from_user(md_pp, u_md_pp, DPM_PP_SIZE))
		goto free_all;
	ret = dpm_create_opt(name, md_pp);
free_all:
	kfree(md_pp);
free_name:
	free_name(name);
	return ret;
}

static int
u_dpm_create_class(const char *u_name, char **u_op_names, unsigned nops)
{
	int ret;
	char *name;
	char **op_names;

	if ((ret = u_get_name(&name, u_name)))
		return ret;
	if ((ret = u_get_names(&op_names, u_op_names, nops)))
		goto free_name;
	ret = dpm_create_class(name, op_names, nops);
	free_names(op_names, nops);
 free_name:
	free_name(name);
	return ret;
}


extern int
dpm_get_classes_in_policy(char *u_name, char **u_class_names);

static int
u_dpm_get_classes(const char *u_name, char **u_class_names)
{
	int ret, i = 0;
	char *name;
	char *class_names[DPM_STATES];

	if ((ret = u_get_name(&name, u_name)))
		return ret;
	
	if (dpm_get_classes_in_policy(name,class_names) !=0) {
		return -EEXIST;
	}

	while(u_class_names[i] != NULL && i < DPM_STATES) {
		if (0 != copy_to_user(u_class_names[i], class_names[i], 
					strlen(class_names[i]) + 1)) {
			return -EFAULT;
		}
		i++;
	}
	return 0;
}

extern int
dpm_get_all_policies(char **u_policy_names);

static int
u_dpm_get_all_policies(char *u_name, char **u_policy_names)
{
	int i = 0, count;
	char *policy_names[238];

	dpm_lock();
	count = dpm_get_all_policies(policy_names);	

	for(i = 0; i < count; i++) {
		if (0 != copy_to_user(u_policy_names[i], policy_names[i], 
					strlen(policy_names[i]) + 1)) {
			dpm_unlock();
			return -EFAULT;
		}
	}
	dpm_unlock();
	return 0;
}

static int
u_dpm_create_policy(const char *u_name, char **u_class_names)
{
	int ret;
	char *name;
	char **class_names;

	trace("u_dpm_create_policy");
	if ((ret = u_get_name(&name, u_name)))
		return ret;
	if ((ret = u_get_names(&class_names, u_class_names, DPM_STATES)))
		goto free_name;
	ret = dpm_create_policy(name, class_names);
	free_names(class_names, DPM_STATES);
 free_name:
	free_name(name);
	return ret;
}

static int
u_dpm_destroy_policy(const char *u_name)
{
	int ret;
	char *name;

	if ((ret = u_get_name(&name, u_name)))
		return ret;
	ret = dpm_destroy_policy(name);
	kfree(name);
	return ret;
}

static int
u_dpm_set_policy(const char *u_name)
{
	int ret;
	char *name;

	if ((ret = u_get_name(&name, u_name)))
		return ret;
	ret = dpm_set_policy(name);
	kfree(name);
	return ret;
}

static int
u_dpm_set_state(const char *u_name)
{
	int ret;
	char *name;

	if ((ret = u_get_name(&name, u_name)))
		return ret;
	ret = dpm_set_op_state(name);
	kfree(name);
	return ret;
}

/*****************************************************************************
 * get name of active policy
 *****************************************************************************/

int
u_dpm_get_policy(char *u_name)
{
	char *name;

	dpm_lock();
	if (!dpm_active_policy) {
		dpm_unlock();
		return -ENOENT;
	}

	name = dpm_active_policy->name;
	if (0 != copy_to_user(u_name, name, strlen(name) + 1)) {
		dpm_unlock();
		return -EFAULT;
	}

	dpm_unlock();
	return 0;
}

/*****************************************************************************
 * set a task state
 *****************************************************************************/

static int
u_dpm_set_task_state(pid_t pid, dpm_state_t task_state)
{
	return dpm_set_task_state(pid, task_state);
}

static int
u_dpm_get_task_state(pid_t pid, dpm_state_t * u_task_state)
{
	int ret;
	dpm_state_t task_state;

	if ((ret = dpm_get_task_state(pid, &task_state)))
		return ret;

	if (0 != copy_to_user(u_task_state, &task_state, sizeof (dpm_state_t))) {
		return -EFAULT;
	}
	return 0;
}

/*****************************************************************************
 * get a policy's statistics
 *****************************************************************************/

#ifdef CONFIG_DPM_STATS
static int
u_dpm_get_policy_stats(char *u_name, struct dpm_param *params)
{
	int ret;
	char *name;
	struct dpm_stats stats, *p;

	if ((ret = u_get_name(&name, u_name)))
		return ret;

	if ((ret = dpm_get_policy_stats(name, &stats)))
		goto out;

	if (copy_from_user(&p, &params->stats, sizeof (struct dpm_stats *)) ||
	    copy_to_user(p, &stats, sizeof (struct dpm_stats))) {
		ret = -EFAULT;
	}
out:
	free_name(name);
	return ret;
}

static int
u_dpm_get_class_stats(char *u_name, struct dpm_param *params)
{
	int ret;
	struct dpm_stats stats, *p;
	char *name;

	if ((ret = u_get_name(&name, u_name)))
		return ret;

	if ((ret = dpm_get_class_stats(name, &stats)))
		goto out;

	if (copy_from_user(&p, &params->stats, sizeof (struct dpm_stats *)) ||
	    copy_to_user(p, &stats, sizeof (struct dpm_stats))) {
		ret = -EFAULT;
	}
out:
	free_name(name);
	return ret;
}

/*****************************************************************************
 * get a operating point's statistics
 *****************************************************************************/

#ifdef CONFIG_DPM_OPT_STATS
static int
u_dpm_get_opt_stats(char *u_name, struct dpm_param *params)
{
	int ret;
	struct dpm_stats stats, *p;
	char *name;

	if ((ret = u_get_name(&name, u_name)))
		return ret;

	if ((ret = dpm_get_opt_stats(name, &stats)))
		goto out;

	if (copy_from_user(&p, &params->stats, sizeof (struct dpm_stats *)) ||
	    copy_to_user(p, &stats, sizeof (struct dpm_stats))) {
		ret = -EFAULT;
	}
out:
	free_name(name);
	return ret;
}
#endif /* CONFIG_DPM_OPT_STATS */

/*****************************************************************************
 * get statistics for all operating states
 *****************************************************************************/

static int
u_dpm_get_os_stats(struct dpm_param *params)
{
	int ret;
	struct dpm_stats stats[DPM_STATES], *p;

	if ((ret = dpm_get_os_stats(stats)))
		return ret;

	if (copy_from_user(&p, &params->stats, sizeof (struct dpm_stats *)) ||
	    copy_to_user(p, &stats, DPM_STATES * sizeof (struct dpm_stats))) {
		ret = -EFAULT;
	}
	return ret;
}
#endif
/*****************************************************************************
 * display a policy's info
 *****************************************************************************/
int
dpm_display_policy(char *name)
{
	struct dpm_policy *p;
	list_find(p, name, dpm_policies, struct dpm_policy);
	if (!p)
		printk("Policy \"%s\" not installed\n", name);
	else
		display_policy(p);
	return 0;
}

static int
u_dpm_display_policy(const char *u_name)
{
	int ret;
	char *name;

	if ((ret = u_get_name(&name, u_name)))
		return ret;
	ret = dpm_display_policy(name);
	kfree(name);
	return ret;
}

/*****************************************************************************
 * single system call to invoke any of the below DPM_* functions from user
 * space
 *****************************************************************************/
#define sys_dpm_param_check(params)\
do { if (!(params)) return -EINVAL; } while(0)

asmlinkage int
sys_dpm(int func, struct dpm_param *params)
{
	struct dpm_param p;

	if (current->uid != 0)	/* root-only access for now */
		return -EACCES;

	if (params) {
		if (0 != copy_from_user(&p, params, sizeof (struct dpm_param))) {
			return -EFAULT;
		}
	}
#ifdef CONFIG_X86
	/*
	 * x86 only has a stubbed out system call for use with the ADK.
 	 * If it ever becomes a real CEE product it will need full DPM
	 * support.
	 */
	return 0;
#endif

	switch (func) {
	case DPM_INIT:
		return dpm_init();
	case DPM_TERMINATE:
		return dpm_terminate();
	case DPM_DISABLE:
		return dpm_disable();
	case DPM_ENABLE:
		return dpm_enable();
	case DPM_CREATE_OPT:
		sys_dpm_param_check(params);
		return u_dpm_create_opt(p.name, p.pp);
	case DPM_CREATE_CLASS:
		sys_dpm_param_check(params);
		return u_dpm_create_class(p.name, p.m_names, p.m_count);
	case DPM_CREATE_POLICY:
		sys_dpm_param_check(params);
		return u_dpm_create_policy(p.name, p.m_names);
	case DPM_DESTROY_POLICY:
		sys_dpm_param_check(params);
		return u_dpm_destroy_policy(p.name);
	case DPM_SET_POLICY:
		sys_dpm_param_check(params);
		return u_dpm_set_policy(p.name);
	case DPM_GET_POLICY:
		sys_dpm_param_check(params);
		return u_dpm_get_policy(params->name);
	case DPM_GET_ALL_POLICIES:
		sys_dpm_param_check(params);
		return u_dpm_get_all_policies(p.name, p.m_names);
	case DPM_GET_CLASSES:
		sys_dpm_param_check(params);
		return u_dpm_get_classes(p.name, p.m_names);
	case DPM_SET_TASK_STATE:
		sys_dpm_param_check(params);
		return u_dpm_set_task_state(p.pid, p.task_state);
	case DPM_GET_TASK_STATE:
		sys_dpm_param_check(params);
		return u_dpm_get_task_state(p.pid, &params->task_state);
#ifdef CONFIG_DPM_STATS
		/* query statistics */
	case DPM_GET_POLICY_STATS:
		sys_dpm_param_check(params);
		return u_dpm_get_policy_stats(p.name, params);
	case DPM_GET_CLASS_STATS:
		sys_dpm_param_check(params);
		return u_dpm_get_class_stats(p.name, params);
	case DPM_GET_OS_STATS:
		sys_dpm_param_check(params);
		return u_dpm_get_os_stats(params);
#ifdef CONFIG_DPM_OPT_STATS
	case DPM_GET_OPT_STATS:
		sys_dpm_param_check(params);
		return u_dpm_get_opt_stats(p.name, params);
#endif /* CONFIG_DPM_OPT_STATS */
#endif /* CONFIG_DPM_STATS */
		/* for debug */
	case DPM_DISPLAY_POLICY:
		sys_dpm_param_check(params);
		u_dpm_display_policy(p.name);
		return 0;
	case DPM_SET_STATE:
		sys_dpm_param_check(params);
		return u_dpm_set_state(p.name);
	default:
		return -EBADRQC;
	}
}

EXPORT_SYMBOL(sys_dpm);
