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

#ifndef __DPM_H__
#define __DPM_H__

#include <linux/config.h>

#ifndef CONFIG_DPM

/* These 2 constants must always be defined for the benefit of the init task
   and system tasks, although they are otherwise ignored if DPM is not
   configured. The only DPM call that appears in system code is a call to
   dpm_set_os() in the context switch. */

#define DPM_NO_STATE   -1
#define DPM_TASK_STATE 0
#define dpm_set_os(task_state) do {} while (0);

#else

#include <asm/dpm.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/dpm-stats.h>

/* User mode defines */

/* Machine dependent timing/counts.  If the machine has not defined
   DPM_MD_STATS, we use a generic routine based on gettimeofday and 64-bit
   values.  Depending on how DPM is being used this may be really slow,
   though. */

/*
 * System call functions
 *
 * A single system call is used to access all functions. The system call takes
 * two parameters: a function code and a pointer to a block of parameters.
 */

/* function names for sys_dpm system call interface */
typedef enum {
	DPM_INIT,		/* initialize the DPM */
	DPM_TERMINATE,		/* terminate the DPM */
	DPM_DISABLE,		/* temporarily disable the DPM */
	DPM_ENABLE,		/* re-enable the disabled DPM */
	DPM_CREATE_OPT,		/* create an operating point */
	DPM_CREATE_CLASS,	/* create a class of operating points */
	DPM_CREATE_POLICY,	/* create a policy */
	DPM_DESTROY_POLICY,	/* destroy a policy */
	DPM_SET_POLICY,		/* set the active policy */
	DPM_GET_POLICY,		/* get the name of the active policy */
	DPM_GET_ALL_POLICIES,	/* get the names of all active policies */
	DPM_GET_CLASSES,	/* get the names of all active policy */
	DPM_SET_TASK_STATE,	/* set a task-specific operating state */
	DPM_GET_TASK_STATE,	/* get a task's task-specific oper state */
	/* get statistics */
	DPM_GET_POLICY_STATS,
	DPM_GET_CLASS_STATS,
	DPM_GET_OPT_STATS,
	DPM_GET_OS_STATS,
	/* debug */
	DPM_DISPLAY_POLICY,
	DPM_SET_STATE,		/* set a (non-scheduling-based) op state */
} dpm_func_t;

/* max size of DPM names */
enum {DPM_NAME_SIZE=256};

/* parameter block for sys_dpm system call interface */
struct dpm_param {
	char		*name;		/* name of entity. strlen(name) must
					   always be < DPM_NAME_SIZE. String
					   must be NULL-terminated. Used by
					   many DPM_*. Used to receive name
					   with DPM_GET_POLICY. */
	char		**m_names;	/* array of member names. Each member
					   name has same characteristics as
					   above entity name. Used by 
					   DPM_CREATE_CLASS and
					   DPM_CRETE_POLICY */
	unsigned	m_count;	/* length of m_names array when used
					   for operating points with
					   DPM_CREATE_CLASS. Not used for
					   class names with DPM_CREATE_POLICY
					   since a policy must always have
					   DPM_STATES classes. */
	dpm_md_pp_t	*pp;		/* array of machine dependent power
					   parameters. Used by DPM_CREATE_OP.
					   Length=DPM_PP_NBR */
	/* next two used by DPM_SET_TASK_STATE and DPM_GET_TASK_STATE. */
	pid_t		pid;		/* task's pid. 0 means "this" task. */
	dpm_state_t	task_state;	/* task operating state. Passed in to
					   DPM_SET_TASK_STATE. Written to by
					   DPM_GET_TASK_STATE. */
	/* Used to return values from DPM_GET_*_STATS. Note that it's a
	   pointer.  For DPM_GET_OS_STATS the array length must be DPM_STATES,
	   otherwise it's a single element. */
	struct dpm_stats     *stats;	        /* statistics */
};

/* single system call to invoke all functions. the "params" argument can be
 * NULL for DPM_INIT, DPM_TERMINATE, DPM_DISABLE, and DPM_ENABLE */
	
#if defined(__KERNEL__)
extern int sys_dpm(int func, struct dpm_param *params);
#else
#define sys_dpm(FUNC, PARAMS) \
	syscall(__NR_sys_dpm, FUNC, PARAMS)
#endif

/* Kernel-only defines */

#ifdef __KERNEL__

#include <linux/dpm-trace.h>
#include <linux/list.h>
#include <asm/semaphore.h>
#include <asm/atomic.h>

/*****************************************************************************
 * Search a list looking for a named entity.
 * A pointer to the found element is put in the variable named by the
 * "answer" argument (or it is set to zero if not found).
 * The structure's type name is given by the "element_type" argument.
 * The name being looked for is given by the "find_me" argument.
 * The name of the stand-alone list_head is given by the "list_name" argument.
 * Assumes the proper semaphore is held.
 * Assumes the structure's list_head is named "list".
 * Assumes the structure's name is in a field called "name"
 *****************************************************************************/
#define list_find(answer,find_me,list_name,element_type)        \
        do {                                                    \
                element_type            *elm;                   \
                struct list_head        *scan;                  \
                (answer)=0;                                     \
                for(scan=list_name.next;scan!=&list_name;       \
                                scan=scan->next) {              \
                        elm=list_entry(scan,element_type,list); \
                        if (strncmp((find_me),elm->name,        \
                                        DPM_NAME_SIZE)==0) {    \
                                (answer)=elm;                   \
                                break;                          \
                        }                                       \
                }                                               \
        } while(0)

/* internal representation of an operating point */
struct dpm_opt {
	char			*name;          /* name */
	struct list_head	list;		/* all installed op points */
	dpm_md_pp_t             pp[DPM_PP_NBR]; /* initialization params */
	struct dpm_md_opt	md_opt;         /* machine dependent part */
	int			constrained;	/* is this opt constrained? */
	struct dpm_stats        stats;          /* statistics */
};

/* internal representation of a class of op points (to be mapped to an
 * operating state */
struct dpm_class {
	char			*name;          /* name */
	struct list_head	list;		/* all installed classes */
	unsigned		nops;		/* nbr ops in this class */
	struct dpm_opt		**ops;		/* the ops in this class */
	struct dpm_opt		*opt;		/* the selected op point */
	struct dpm_stats        stats;          /* statistics */
};

/* internal representation of an installed power policy */
struct dpm_policy {
	char			*name;          /* name */
	struct list_head	list;		/* all installed policies */
	struct dpm_class	*classes[DPM_STATES];	/* the classes */
	struct dpm_stats        stats;          /* statistics */
};

/*
 * internal use utility functions for use by DPM
 */

/* DPM semaphore locking. To simplify future expansion, don't 'down' dpm_sem
   directly.  Also, dpm_sem must be 'up'ed only by dpm_unlock(). */

extern struct semaphore dpm_sem;

extern inline void
dpm_lock(void)
{
        down(&dpm_sem);
}

extern inline int
dpm_lock_interruptible(void)
{
        if (down_interruptible(&dpm_sem))
                return -ERESTARTSYS;
        return 0;
}

extern inline int
dpm_trylock(void)
{
        if (down_trylock(&dpm_sem))
                return -EBUSY;
        return 0;
}

void dpm_unlock(void);
void dpm_idle(void);

/* 
 * kernel's operating state interface
 */

/* set operating state */
void dpm_set_os(dpm_state_t state);

/*
 * device driver's constraint interface
 */

/* tbd */

/*
 * policy manager's interface
 */

extern char *dpm_state_names[DPM_STATES];

/* initialize/terminate the DPM */
int dpm_init(void);
int dpm_terminate(void);

/* (temporarily) disable the DPM */
int dpm_disable(void);

/* re-enable the DPM */
int dpm_enable(void);

/* suspend/resume DPM across a system shutdown */
int dpm_suspend(void);
void dpm_resume(void);

/* create operating point */
int _dpm_create_opt(struct dpm_opt **opt, const char *name, const dpm_md_pp_t *pp);
int dpm_create_opt(const char *name, const dpm_md_pp_t *pp);

/* create class of operating points */
int dpm_create_class(const char *name, char **op_names, unsigned nops);

/* create policy */
int dpm_create_policy(const char *name, char **class_names);

/* destroy policy */
int dpm_destroy_policy(const char *name);

/* activate a power policy */
int dpm_set_policy(const char *name);

/* get name of active power policy */
int dpm_get_policy(char *name);

/* set a task's task-specific operating state */
int dpm_set_task_state(pid_t pid, dpm_state_t state);

/* get  a task's task-specific operting state */
int dpm_get_task_state(pid_t pid, dpm_state_t *state);

/* set a raw operating state */
int dpm_set_op_state(const char *name);
int
dpm_default_set_opt(struct dpm_opt *opt, unsigned flags);

int dpm_get_force(void);
void dpm_set_force(int f);

/*
 * global data for power management system
 */

/* curently installed policies, classes and operating points */
extern struct list_head		dpm_policies;
extern struct list_head		dpm_classes;
extern struct list_head		dpm_opts;
extern struct semaphore		dpm_policy_sem;
extern spinlock_t		dpm_policy_lock;

/* the currently active policy */
extern struct dpm_policy	*dpm_active_policy;

/* the currently active operating state and operating point */
extern dpm_state_t		dpm_active_state;
extern struct dpm_opt		*dpm_active_opt;

/* is DPM initialized and enabled? */
extern int			dpm_initialized;
extern int			dpm_enabled;

/* If this operating point is defined, it will be used as the "relock"
   operating point for opt. changes requiring relocking while DPM is
   disabled. */

extern struct dpm_opt *dpm_relock_opt;

struct dpm_idle_parms {
	u32 flags;
	struct dpm_stats idle_stats;
	struct dpm_stats irq_stats;
	struct dpm_md_idle_parms md;	
};

u32 basic_idle(struct dpm_idle_parms *idle_parms);

dpm_fscaler
compute_fscaler(struct dpm_md_opt *cur, struct dpm_md_opt *new);

extern inline void
dpm_quick_enter_state(int new_state)
{
#ifdef CONFIG_DPM_STATS
	dpm_update_stats(&dpm_state_stats[new_state],
			 &dpm_state_stats[dpm_active_state]);
#endif

        dpm_active_state = new_state;
}

/*
 * The following functions are required to be defined in machine dependent
 * code:
 *
 * machine dependent code to set an operating point 
 * int dpm_md_set_opt(dpm_opt_t *opt, int flags);
 *
 * machine dependent code to initialize an operating point 
 * int dpm_md_init_opt(dpm_opt_t *opt);
 *
 * Machine dependent code to return a timestamp
 * dpm_md_time_t dpm_md_time(void);
 */

/* Flags for dpm_md_set_opt().  By default, dpm_md_set_op() is guaranteed not
   to block the caller, and will arrange to complete asynchronously if
   necessary. 

   DPM_SYNC    The operating point is guaranteed to be set when the call
               returns. The call may block.

   DPM_UNLOCK  The caller requires dpm_md_set_opt() to unlock the DPM system
               once the operating point is set.
*/

#define DPM_SYNC      0x01
#define DPM_UNLOCK    0x02

/*
 * Common machine-dependent and board-dependent function wrappers.
 */

extern struct dpm_md dpm_md;
extern struct dpm_bd dpm_bd;

static inline void
dpm_md_init(void)
{
        if (dpm_md.init)
                dpm_md.init();
}

static inline void
dpm_md_cleanup(void)
{
        if (dpm_md.cleanup)
                dpm_md.cleanup();
}


static inline void
dpm_md_idle_set_parms(struct dpm_md_idle_parms *idle_parms)
{
        if (dpm_md.idle_set_parms)
                dpm_md.idle_set_parms(idle_parms);
}


/* Machine-dependent operating point creating/query/setting */


static inline int
dpm_md_init_opt(struct dpm_opt *opt)
{
        if (dpm_md.init_opt)
                return dpm_md.init_opt(opt);
        return 0;
}

static inline int
dpm_md_set_opt(struct dpm_opt *opt, int flags)
{
        if (dpm_md.set_opt)
                return dpm_md.set_opt(opt, flags);
        return 0;
}

static inline int
dpm_md_get_opt(struct dpm_opt *opt)
{
        if (dpm_md.get_opt)
                return dpm_md.get_opt(opt);
        return 0;
}


static inline int
dpm_md_validate_opt(struct dpm_opt *opt)
{
        if (dpm_md.validate_opt)
                return dpm_md.validate_opt(opt);
	/* if there is no validate_opt function, assume the opt is
	   valid */
        return 1;
}


static inline int
dpm_bd_init(void)
{
        if (dpm_bd.init)
                return dpm_bd.init();
        return 0;
}

static inline int
dpm_bd_check_v(unsigned *v)
{
        if (dpm_bd.check_v)
                return dpm_bd.check_v(v);
        return 0;
}

static inline unsigned
dpm_bd_get_v(void)
{
        if (dpm_bd.get_v)
                return dpm_bd.get_v();
        return 0;
}

static inline int
dpm_bd_set_v_pre(unsigned cur_mv, unsigned target_mv, int flags)
{
        if (dpm_bd.set_v_pre)
                return dpm_bd.set_v_pre(cur_mv, target_mv, flags);
        return 0;
}

static inline int
dpm_bd_set_v_post(unsigned cur_mv, unsigned target_mv, int flags)
{
        if (dpm_bd.set_v_post)
                return dpm_bd.set_v_post(cur_mv, target_mv, flags);
        return 0;
}

#endif /* __KERNEL__ */

#endif /* CONFIG_DPM */
#endif /*__DPM_H__*/
