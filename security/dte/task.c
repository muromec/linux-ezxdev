/*
 * security/dte/task.c
 *
 * DTE security module functions.  These are inserted into the DTE
 * security plug in security/dte.c
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 * The functions which I don't redefine stay in security/dte/dte.c.
 *
 * author: Serge Hallyn  <hallyn@cs.wm.edu>
 */

#include "dte.h"
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/security.h>
#include <linux/sysctl.h>

extern int dte_initialized;
extern int dte_bitchmode;
extern struct security_operations *dte_secondary_ops;

/*
 * check whether current domain is required to switch domains
 * on execution of filename, if yes switch and return 0.
 * If error reading file, return < 0, else return > 0
 */
static int dte_auto_switch(char *type)
{
	int h;
	struct dte_task_sec *ts = current->security;
	struct dte_domain_t *curd = ts->dte_domain;
	struct dte_gateway *gw;

	/* no gateways, no auto switches: */
	if (!curd->num_gw)
		return 1;

	h = dte_hash(type, curd->num_gw);
	gw = &curd->gw[h];
	while (gw) {
		if (strcmp(gw->type, type)==0)
			break;
		gw = gw->hash_next;
	}
	if (gw) {
		if (ts->dte_back) {
			printk(KERN_NOTICE "auto switch overriding exec switch to %s.\n",
					curd->name);
			printk(KERN_NOTICE "switching domains: %s to %s on exec %s.\n",
					ts->dte_back->name, gw->domain->name, type);
			ts->dte_domain = gw->domain;
		} else {
			printk(KERN_NOTICE "switching domains: %s to %s on exec %s.\n",
					curd->name, gw->domain->name, type);
			ts->dte_back = ts->dte_domain;
			ts->dte_domain = gw->domain;
		}
		return 0;
	}
	return 1;
}

static int dte_check_x(struct dte_domain_t *d, char *type)
{
	struct dte_ta *ta;
	int h;

	h = dte_hash(type, d->num_ta);
	ta = &d->ta[h];
	while (ta && ta->type!=type)
		ta = ta->hash_next;

	if (ta && (ta->access&DTE_FX))
		return 0;		/* x granted */
	if (dte_bitchmode) {
		printk(KERN_NOTICE "Would be denied:  access from %s to %s as execute.\n",
				d->name, type);
		return 0;
	}
	return 1;			/* x denied */
}

static int dte_domain_has_ep(struct dte_domain_t *d, char *t)
{
	struct dte_ep *e;
	int h;

	h = dte_hash(t, d->num_ep);
	e = &d->ep[h];
	while (e->type && strcmp(e->type, t)!=0 && e->hash_next)
		e = e->hash_next;
	if (e->type && strcmp(e->type, t)==0)
		return 1;
	return 0;
}

int dte_binprm_alloc_security (struct linux_binprm *bprm)
{
	int rc = 0;
	bprm->security = NULL;
	if (dte_secondary_ops)
		rc = dte_secondary_ops->bprm_alloc_security(bprm);

	return rc;
}

void dte_binprm_free_security (struct linux_binprm *bprm)
{
	return;
}

int dte_binprm_set_security (struct linux_binprm *bprm)
{
	struct dte_inode_sec *s = bprm->file->f_dentry->d_inode->i_security;
	struct dte_task_sec *ts = current->security;
	int ret = 0;

	if (dte_secondary_ops)
		ret = dte_secondary_ops->bprm_set_security(bprm);
	if (ret)
		return ret;

	if (!s || !s->etype) {
		printk(KERN_NOTICE "dte_bprm_set_security: on exec of %s - no etype.\n",
				bprm->filename);
#if 0
		return -EACCES;
#else
		/* don't want to return -eaccess, as the file probably doesn't
		 * exist, and that could be misleading info for the user */
		return 0;
#endif
	}

	if (!dte_auto_switch(s->etype)) {
		printk(KERN_NOTICE "dte: auto switch to %s on exec %s in %s.\n",
				ts->dte_domain->name, bprm->filename, ts->dte_back->name);
	} else if (ts->dte_back &&
			!dte_domain_has_ep(ts->dte_domain, s->etype)) {
		printk(KERN_NOTICE "dte: type %s is not ep to domain %s.\n",
				s->etype, ts->dte_domain->name);
		ts->dte_domain = ts->dte_back;
		ts->dte_back = NULL;
		return -EACCES;
	}

	ret = dte_check_x(ts->dte_domain, s->etype);
	if (ret) {
		/* not allowed */
		printk(KERN_NOTICE "dte: domain %s may not execute type %s.\n",
				ts->dte_domain->name, s->etype);
		if (ts->dte_back) {
			ts->dte_domain = ts->dte_back;
			ts->dte_back = NULL;
		}
		return ret;
	}

#if 0
	if (ts->dte_back)
		printk(KERN_NOTICE "dte_bprm_set_sec: finalized trans from %s to %s.\n",
				ts->dte_back->name, ts->dte_domain->name);
#endif
	ts->dte_back = NULL;

	return 0;
}

int dte_task_alloc_security (struct task_struct *p)
{
	struct dte_task_sec *ps, *cs;

	ps = p->security = kmalloc(sizeof(struct dte_task_sec), GFP_KERNEL);
	cs = current->security;

	if (!ps)
		return -ENOMEM;

	if (cs)
		ps->dte_domain = cs->dte_domain;
	else
		ps->dte_domain = NULL;
	ps->dte_back   = NULL;
	return 0;
}

void dte_task_free_security (struct task_struct *p)
{
	if (p->security) {
		kfree(p->security);
		p->security = NULL;
	}
}

int dte_task_kill (struct task_struct *p, struct siginfo *info, int sig)
{
	int h;
	struct dte_sa *sa;
	struct dte_task_sec *sts,  /* sending task security */
				  *rts;  /* receiv task security */
	struct dte_domain_t *std,  /* sending task domain */
				  *rtd;  /* receiv task domain */

	if (!dte_initialized)
		return 0;

	/* following lsm documentation advice: */
	if (info && ((unsigned long)info==1 || SI_FROMKERNEL(info)))
		return 0;

	if (!p) {
		printk(KERN_ERR "dte_task_kill: sending signal to null.\n");
		return 0;
	}

#ifdef CONFIG_DTE_DEBUG
	printk(KERN_NOTICE "dte_task_kill: from %d to %d sig %d.\n",
			current->pid, p->pid, sig);
#endif

	sts = (struct dte_task_sec *)current->security;
	rts = (struct dte_task_sec *)p->security;
	if (!sts)
		panic("dte_kill: sending task has no security blob.\n");
	if (!rts)
		panic("dte_kill: recv task has no security blob.\n");
	std = sts->dte_domain;
	rtd = rts->dte_domain;
	if (std == rtd)
		return 0;  /* any signals allowed as far as we're concerned */
	if (!std || !rtd)
		BUG();
	if (!std->num_sa)
		goto signal_reject;  /* no signals at all, save some time */

	/* signals to anyone: */
	h  = dte_hash(std->name, std->num_sa);
	sa = &std->sa[h];
	while (sa) {
		if (sa->recv_domain==std)
			if (sa->signal==0 || sa->signal==sig)
				return 0;
		sa = sa->hash_next;
	}

	/* signals to requested receiving domain: */
	h  = dte_hash(rtd->name, std->num_sa);
	sa = &std->sa[h];
	while (sa) {
		if (sa->recv_domain==rtd)
			if (sa->signal==0 || sa->signal==sig)
				return 0;
		sa = sa->hash_next;
	}

signal_reject:
	printk(KERN_NOTICE "DTE: refusing signal %d from %d(%s) to %d(%s).\n",
			sig, current->pid, sts->dte_domain->name, p->pid,
			rts->dte_domain->name);
	return -EPERM;
}
