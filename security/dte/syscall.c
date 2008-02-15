/*
 * security/dte/syscall.c
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

extern int num_dte_domains;
extern struct dte_domain_t **dte_domains;

#define min(x,y) ( x<y ? x : y )
/*
 * for do_gettype: data is:
 *  1. (char *) filename
 *  2. (char *) buffer
 *  3. (int)    buflen
 */
struct dte_gt_struct {
	char *fnam;
	char *buf;
	int buflen;
};
static long dte_do_gettype(void *data)
{
	struct dte_gt_struct gs;
	char *fnam;
	long err;
	int len;
	struct nameidata nd;
	struct dte_inode_sec *s;

	err = -EFAULT;
	if (copy_from_user(&gs, data, sizeof(struct dte_gt_struct)))
		goto out;
	fnam = getname(gs.fnam);
	err = PTR_ERR(fnam);
	if (IS_ERR(fnam))
		goto out;
	err = 0;
	if (path_init(fnam,LOOKUP_POSITIVE|LOOKUP_FOLLOW,&nd))
		err = path_walk(fnam, &nd);
	putname(fnam);
	if (err)
		goto out;

	err = permission(nd.dentry->d_inode,MAY_READ);
	if (err)
		goto dput_and_out;

	err = -ENOENT;
	if (!nd.dentry->d_inode)
		goto dput_and_out;
	s = (struct dte_inode_sec *) nd.dentry->d_inode->i_security;
	if (!s)
		goto dput_and_out;
	if (!s->etype) {
		if (gs.buflen<10) {
			copy_to_user(gs.buf, "no etype.", gs.buflen);
			gs.buf[gs.buflen] = '\0';
		} else
			copy_to_user(gs.buf, "no etype.", 10);
		goto dput_and_out;
	}
	len = min(strlen(s->etype)+1, gs.buflen-1);
	if (copy_to_user(gs.buf, s->etype, len))
		err = -EFAULT;
	else {
		err = 0;
	/*	gs.buf[len] = '\0';*/  /* just to be SURE */
	}

dput_and_out:
	path_release(&nd);
out:
	return err;
}

struct dte_gd_struct {
	unsigned int pid;
	char *buf;
	int buflen;
};
static long dte_do_getdomain(void *data)
{
	struct dte_gd_struct gd;
	struct task_struct *p;
	struct dte_task_sec *ts;
	int err, len;

	err = -EFAULT;
	if (copy_from_user(&gd, data, sizeof(struct dte_gd_struct)))
		goto out;
	err = -1;
	p = find_task_by_pid(gd.pid);
	if (!p) goto out;
	ts = p->security;
	if (!ts || !ts->dte_domain) goto out;

	err = -EFAULT;
	len = min(strlen(ts->dte_domain->name)+1, gd.buflen-1);
	if (!copy_to_user(gd.buf, ts->dte_domain->name, len)) {
		err = 0;
/*		gd.buf[len] = '\0';*/
	}

out:
	return err;
}

static int dte_may_exec_to(struct dte_domain_t *a, struct dte_domain_t *b)
{
	struct dte_ea *ea;
	char *bn = b->name;
	int h;

	h = dte_hash(bn, a->num_ea);
	ea = &a->ea[h];
	while (ea->other_domain && strcmp(ea->other_domain->name, bn)!=0 &&
			ea->hash_next)
		ea = ea->hash_next;
	if (ea->other_domain && strcmp(ea->other_domain->name, bn)==0 &&
			dte_exec_access(ea->access))
		return 1;

	return 0;
}

static struct dte_domain_t *get_domain(char *name)
{
	struct dte_domain_t *d;
	int h;

	h = dte_hash(name,num_dte_domains);
	if (h<0 || h>num_dte_domains)
		return NULL;
	d = dte_domains[h];
	if (!d)
		return NULL;
	while (d->hash_next && strcmp(d->name,name))
		d = d->hash_next;
	if (strcmp(d->name,name))
		return NULL;
	return d;
}

/*
 * Modules apparently can't use errno, so I can't use unistd.h
 * directly.  Ergo, define my own execve.
 */
#define __NR_execve		 11
#define __syscall_return(type, res) \
do { \
	return (type) (res); \
} while (0)
#define _syscall3(type,name,type1,arg1,type2,arg2,type3,arg3) \
type name(type1 arg1,type2 arg2,type3 arg3) \
{ \
long __res; \
__asm__ volatile ("int $0x80" \
	: "=a" (__res) \
	: "0" (__NR_##name),"b" ((long)(arg1)),"c" ((long)(arg2)), \
		  "d" ((long)(arg3))); \
__syscall_return(type,__res); \
}

static inline _syscall3(int,execve,const char *,file,char **,argv,char **,envp)
/*
 * Note on dte_d_exec:
 * here is a major switch from my previous version.  I delay checking the
 * validity of the domain switch until well into the execve, in the
 * bprm_alloc_security, in fact.  In order to check it here, the current
 * domain needs to have the ability to access the file being executed so
 * that I can check the type of the file.  This should not be necessary, as
 * only the domain being switched to should need this permission.
 *
 * Note that this cannot work for auto domain switches.  There the source
 * domain must be able to reach the entry point, for us to even figure out
 * that there is a required auto switch.
 */
struct dte_exec_struct{
	char *fnam;
	char **argv;
	char **envp;
	char *domain;
};
static long dte_do_exec(void *data)
{
	struct dte_exec_struct es;
	char *fnam;
	long err;
	char *d;
	struct dte_task_sec *ts;
	struct dte_domain_t *dest;

	err = -EFAULT;
	if (copy_from_user(&es, data, sizeof(struct dte_exec_struct)))
		goto out;
	fnam = getname(es.fnam);
	err = PTR_ERR(fnam);
	if (IS_ERR(fnam))
		goto out;

	d = getname(es.domain);
	err = PTR_ERR(d);
	if (IS_ERR(d))
		goto out_putf;
	dest = get_domain(d);
	if (!dest) {
		printk(KERN_NOTICE "Error: no domain named %s.\n", d);
		putname(d);
		err = -EINVAL;
		goto out_putf;
	}
	putname(d);

	ts = current->security;
	if (!dte_may_exec_to(ts->dte_domain, dest)) {
		printk(KERN_NOTICE "dte: domain %s may not exec to domain %s.\n",
				ts->dte_domain->name, dest->name);
		err = -EACCES;
		goto out_putf;
	}

	ts->dte_back  = ts->dte_domain;
	ts->dte_domain = dest;

	printk(KERN_NOTICE "dte: requesting exec from %s to %s on %s.\n", 
			ts->dte_back->name, ts->dte_domain->name, fnam);
	putname(fnam);
	err = execve(es.fnam, es.argv, es.envp);

	/* 
	 * uh-oh - we had an error...
	 * reset the domain to original
	 */
	if (ts->dte_back) {
		ts->dte_domain = ts->dte_back;
		ts->dte_back = NULL;
	}
	goto out;

out_putf:
	putname(fnam);
out:
	return err;
}

/*asmlinkage*/
int dte_sys_security(unsigned int id, unsigned int call,
		unsigned long *args)
{
	switch(call) {
		case 1: return (int)dte_do_gettype(args);
		case 2: return (int)dte_do_getdomain(args);
		case 3: return (int)dte_do_exec(args);
		default: return -ENOSYS;
	}
}

