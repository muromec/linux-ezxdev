/*
 * security/dte/inode.c
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
extern int dte_debug;
extern int dte_bitchmode;

void dte_copy_ino_sec(struct inode *p, struct inode *c)
{
	struct dte_inode_sec *ps, *cs;

	ps = (struct dte_inode_sec *)p->i_security;
	cs = (struct dte_inode_sec *)c->i_security;
	if (!cs) {
		dte_inode_alloc_security(c);
		cs = (struct dte_inode_sec *)c->i_security;
	}
	cs->etype = ps->etype;
	cs->utype = ps->utype;
	cs->map   = ps->map;
	sema_init(&cs->s_sem, 1);
	cs->initialized = 1;
}

static inline void dte_real_postlookup (struct inode *ino, 
		struct dentry *d, int create)
{
	struct inode *di;  /* inode taken from child dentry */
	/*
	 * p is security field for parent inode
	 * c is security field for child inode
	 */
	struct dte_inode_sec *p, *c;
	struct dte_sb_sec *sb_sec; /* superblock security field */
	long long offset;
	unsigned char buf[2];
	int et, ret;
	mm_segment_t old_fs;

#ifdef CONFIG_DTE_DEBUG
	printk(KERN_NOTICE "dte_real_postlookup: called on %s.\n", d->d_iname);
#endif

	if (!dte_initialized) return;
	if (!d || !(di = d->d_inode)) {
		return;
	}
	c = (struct dte_inode_sec *) di->i_security;
	p = (struct dte_inode_sec *) ino->i_security;
#if 1
	if (!p)		/* this ought never happen */
		panic("no security object on parent inode %lu!\n",ino->i_ino);
	if (!c) {
		printk(KERN_NOTICE "no security object on child inode %s!\n",
				d->d_iname);
		dte_inode_alloc_security(di);
		c = di->i_security;
	}
#endif
	down(&c->s_sem);
	if (c->initialized) {
		up(&c->s_sem);
		return;
	}

	sb_sec = (struct dte_sb_sec *)di->i_sb->s_security;
	if (!sb_sec) {
		panic("dte_post_lookup: no s_security on inode's sb, %lu.\n", di->i_ino);
	}

	/* assign types using the hierarchical scheme */

#ifdef CONFIG_DTE_DEBUG
	printk(KERN_NOTICE "dte_real_postlookup: looking up %s.\n", d->d_iname);
#endif

	c->map = NULL;
	c->utype = c->etype = p->utype;
	c->initialized = 1;

	if (p->map && p->map->num_kids) {
		c->map = mapnode_getkid(p->map, d->d_name.name);
		if (c->map) {
			if (c->map->etype)
				c->etype = c->map->etype;
			if (c->map->utype)
				c->utype = c->map->utype;
		}
	}

	if (create || !sb_sec->fp_ready) {
		up(&c->s_sem);
		return;
	}

	/* 
	 * Read types from ea file
	 * We leave the mapnodes as are, but, if a valid type is found, we use
	 * that for both etype and utype
	 *
	 * An alternative would be to use it only for etype.  Not sure which is
	 * best.
	 */
	c->map = NULL;
	if (sb_sec->ntypes<128) {
		offset = sb_sec->offset+di->i_ino;
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		ret = sb_sec->fp->f_op->read(sb_sec->fp, buf, 1, &offset);
		set_fs(old_fs);
		if (ret<0)
			printk(KERN_NOTICE "dte_post_lookup: read(1) returned %d.\n", ret);
		et = (int)(*((unsigned char *)buf));
	} else {
		offset = sb_sec->offset+2*di->i_ino;
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		ret = sb_sec->fp->f_op->read(sb_sec->fp, buf, 2, &offset);
		set_fs(old_fs);
		if (ret<0)
			printk(KERN_NOTICE "dte_post_lookup: read(2) returned %d.\n", ret);
		et = (int)(*((unsigned short *)buf));
	}

	if (et!=sb_sec->ntypes) {
		c->utype = c->etype = sb_sec->type_conv[et];
		c->initialized = 1;
	}
	up(&c->s_sem);
}

void dte_post_lookup (struct inode *ino, struct dentry *d) {
	dte_real_postlookup(ino, d, 0);
}

void dte_d_instantiate (struct dentry *dentry, struct inode *inode) 
{
	struct dentry *parent_dentry;
	struct inode *parent_inode;
	/*
	 * p is security field for parent inode
	 * c is security field for child inode
	 */
	struct dte_inode_sec *p, *c;
	struct dte_sb_sec *sb_sec; /* superblock security field */
	long long offset;
	unsigned char buf[2];
	int et, ret;
	mm_segment_t old_fs;

#ifdef CONFIG_DTE_DEBUG
	printk(KERN_NOTICE "%s: called on %s.\n", __FUNCTION__, dentry->d_iname);
#endif

	if (!dte_initialized) return;
	if (!inode) 
		return;
	if (dentry == dentry->d_parent)
		return;

	parent_dentry = dentry->d_parent;
	if (!parent_dentry) {
		printk(KERN_NOTICE "%s: no parent dentry for dentry %s.\n", 
		       __FUNCTION__, dentry->d_iname);
		return;
	}
	parent_inode = parent_dentry->d_inode;
	if (!parent_inode) {
		printk(KERN_NOTICE "%s: no inode for parent dentry %s.\n", 
		       __FUNCTION__, parent_dentry->d_iname);
		return;
	}
	p = (struct dte_inode_sec *) parent_inode->i_security;
	if (!p || !p->initialized) {
		/* Unallocated or uninitialized security object on parent.
		   This can happen legitimately for pipe or socket inodes,
		   since DTE does not appear to label them.  Ignore for now. */
		return;
	}

	c = (struct dte_inode_sec *) inode->i_security;
	if (!c) {
		printk(KERN_NOTICE "%s: no security object on child inode %s!\n",
		       __FUNCTION__, dentry->d_iname);
		dte_inode_alloc_security(inode);
		c = inode->i_security;
	}
	down(&c->s_sem);
	if (c->initialized) {
		up(&c->s_sem);
		return;
	}

	/* assign types using the hierarchical scheme */

#ifdef CONFIG_DTE_DEBUG
	printk(KERN_NOTICE "%s: looking up %s.\n", __FUNCTION__, dentry->d_iname);
#endif

	c->map = NULL;
	c->utype = c->etype = p->utype;
	c->initialized = 1;

	if (p->map && p->map->num_kids) {
		c->map = mapnode_getkid(p->map, dentry->d_name.name);
		if (c->map) {
			if (c->map->etype)
				c->etype = c->map->etype;
			if (c->map->utype)
				c->utype = c->map->utype;
		}
	}

	sb_sec = (struct dte_sb_sec *)inode->i_sb->s_security;
	if (!sb_sec || !sb_sec->fp_ready) {
		up(&c->s_sem);
		return;
	}

	/* 
	 * Read types from ea file
	 * We leave the mapnodes as are, but, if a valid type is found, we use
	 * that for both etype and utype
	 *
	 * An alternative would be to use it only for etype.  Not sure which is
	 * best.
	 */
	c->map = NULL;
	if (sb_sec->ntypes<128) {
		offset = sb_sec->offset+inode->i_ino;
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		ret = sb_sec->fp->f_op->read(sb_sec->fp, buf, 1, &offset);
		set_fs(old_fs);
		if (ret<0)
			printk(KERN_NOTICE "%s: read(1) returned %d.\n", __FUNCTION__, ret);
		et = (int)(*((unsigned char *)buf));
	} else {
		offset = sb_sec->offset+2*inode->i_ino;
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		ret = sb_sec->fp->f_op->read(sb_sec->fp, buf, 2, &offset);
		set_fs(old_fs);
		if (ret<0)
			printk(KERN_NOTICE "%s: read(2) returned %d.\n", __FUNCTION__, ret);
		et = (int)(*((unsigned short *)buf));
	}

	if (et!=sb_sec->ntypes) {
		c->utype = c->etype = sb_sec->type_conv[et];
		c->initialized = 1;
	}
	up(&c->s_sem);
}

int dte_inode_alloc_security	(struct inode *inode)
{
	struct dte_inode_sec *s;
	
	s = inode->i_security = kmalloc(sizeof(struct dte_inode_sec), GFP_KERNEL);
	if (!s)
		panic("Out of memory, kmalloc failed.\n");
	s->map = NULL;
	s->etype = s->utype = NULL;
	s->initialized = 0;
	sema_init(&s->s_sem, 1);

	return 0;
}

void dte_inode_free_security	(struct inode *inode)
{
	if (inode->i_security)
		kfree(inode->i_security);
	inode->i_security = NULL;
}

/*
 * we'll optimize this later
 */
inline int find_type_conv(char **conv_array, int array_size, char *type)
{
	int i;

	for (i=0; i<array_size; i++) {
		if (strcmp(conv_array[i], type)==0)
			return i;
	}

	printk(KERN_ERR "dte: type not listed in superblock ea type list: %s.\n", type);
	printk(KERN_ERR "dte: returning type 0, which is %s.\n", conv_array[0]);
	return 0;
}

static void dte_inode_real_post_create (struct inode *inode, struct dentry *dentry)
{
	long long offset;
	unsigned char buf[2];
	int et, ret;
	mm_segment_t old_fs;
	unsigned short *usp;
	struct dte_inode_sec *isec;
	struct dte_sb_sec *sb_sec;

	if (!dte_initialized)
		return;
	dte_real_postlookup(inode, dentry, 1);

	sb_sec = (struct dte_sb_sec *)dentry->d_sb->s_security;

	if (sb_sec->fp_ready) {
		/* write ea */
		isec = (struct dte_inode_sec *)dentry->d_inode->i_security;
		et = find_type_conv(sb_sec->type_conv, sb_sec->ntypes, isec->etype);

		if (sb_sec->ntypes<128) {
			buf[0] = (unsigned char) et;
			offset = sb_sec->offset+dentry->d_inode->i_ino;
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			ret = sb_sec->fp->f_op->write(sb_sec->fp, buf, 1, &offset);
			set_fs(old_fs);
			if (ret<0)
				printk(KERN_NOTICE "dte_post_lookup: write(1) returned %d.\n", ret);
		} else {
			usp = (unsigned short *) buf;
			*usp = (unsigned short) et;
			offset = sb_sec->offset+2*dentry->d_inode->i_ino;
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			ret = sb_sec->fp->f_op->write(sb_sec->fp, buf, 2, &offset);
			set_fs(old_fs);
			if (ret<0)
				printk(KERN_NOTICE "dte_post_lookup: write(2) returned %d.\n", ret);
		}
	}
}

void dte_inode_post_create (struct inode *inode, struct dentry *dentry, int mask)
{
	dte_inode_real_post_create(inode, dentry);
}

void dte_inode_post_mknod (struct inode *inode, struct dentry *dentry,
				  int major, dev_t minor)
{
	dte_inode_real_post_create(inode, dentry);
}

void dte_inode_post_symlink (struct inode *inode, struct dentry *dentry,
				    const char *name)
{
	dte_inode_real_post_create(inode, dentry);
}

void dte_inode_post_mkdir (struct inode *inode, struct dentry *dentry,
				  int mask)
{
	dte_inode_real_post_create(inode, dentry);
}

#ifdef CONFIG_DTE_VERBOSE
#define DENY_ACCESS(str) { \
	printk(KERN_NOTICE "denied: %s to %s as %s\n", ts->dte_domain->name, \
			s->etype, str); \
	return -EACCES;}
#else
#define DENY_ACCESS(str) {return -EACCES;}
#endif

#define BITCH_ACCESS(str) { \
	printk(KERN_NOTICE "Would be denied: %s to %s as %s\n", ts->dte_domain->name, \
			s->etype, str); \
	return 0;}

static inline int dte_bitch_inode_permission (struct inode *inode, int mask)
{
	struct dte_inode_sec *s = inode->i_security;
	struct dte_task_sec *ts = current->security;
	struct dte_domain_t *d;
	struct dte_ta *ta;
	int h;

	if (!dte_initialized) return 0;  /* only during setup, particularly 
							dte.conf and dteeaf */
	if (!s || !s->etype) {
		printk(KERN_NOTICE "dte_inode_permission: inode has no i_security or et.\n");
		return 0;
	}
	if (!ts) {
		printk(KERN_NOTICE "dte_inode_permission: task has no task_security.\n");
		return 0;
	}
	d = ts->dte_domain;
	if (!d) {
		printk(KERN_NOTICE "dte_inode_permission: task has no dte_domain.\n");
		return 0;
	}
	h = dte_hash(s->etype, ts->dte_domain->num_ta);
	ta = &d->ta[h];
	while (ta && ta->type != s->etype)
		ta = ta->hash_next;
	if (!ta) {
		printk(KERN_NOTICE "dte_inode_permission: can't find type access.  Would deny!\n");
		return 0;
	}
	if (S_ISDIR(inode->i_mode)) {
		if ((mask&MAY_EXEC) && !(dte_descend_access(ta->access)))
			BITCH_ACCESS("dir x");
		if ((mask&MAY_WRITE) && !(dte_create_access(ta->access)))
			BITCH_ACCESS("dir w");
		if ((mask&MAY_READ) && !(dte_readdir_access(ta->access)))
			BITCH_ACCESS("dir r");
	} else {
		if ((mask&MAY_WRITE) && !(dte_fw_access(ta->access)))
			BITCH_ACCESS("file w");
		if ((mask&MAY_READ) && !(dte_fr_access(ta->access)))
			BITCH_ACCESS("file r");
	}
	return 0;
}

static inline int dte_real_inode_permission(struct inode *inode, int mask)
{
	struct dte_inode_sec *s = inode->i_security;
	struct dte_task_sec *ts = current->security;
	struct dte_domain_t *d;
	struct dte_ta *ta;
	int h;

	if (!dte_initialized) return 0;  /* only during setup, particularly 
							dte.conf and dteeaf */
	if (!s || !s->etype) {
		return 0;
	}
	if (!ts) {
		printk(KERN_NOTICE "dte_inode_permission: task has no task_security.\n");
		return 0;
	}
	d = ts->dte_domain;
	if (!d) {
		printk(KERN_NOTICE "dte_inode_permission: task has no dte_domain.\n");
		return 0;
	}
	h = dte_hash(s->etype, ts->dte_domain->num_ta);
	ta = &d->ta[h];
	while (ta && ta->type != s->etype)
		ta = ta->hash_next;
	if (!ta) {
		return -EACCES;
	}
	if (S_ISDIR(inode->i_mode)) {
		if ((mask&MAY_EXEC) && !(dte_descend_access(ta->access)))
			DENY_ACCESS("dir x");
		if ((mask&MAY_WRITE) && !(dte_create_access(ta->access)))
			DENY_ACCESS("dir w");
		if ((mask&MAY_READ) && !(dte_readdir_access(ta->access)))
			DENY_ACCESS("dir r");
	} else {
		if ((mask&MAY_WRITE) && !(dte_fw_access(ta->access)))
			DENY_ACCESS("file w");
		if ((mask&MAY_READ) && !(dte_fr_access(ta->access)))
			DENY_ACCESS("file r");
	}
	return 0;
}

int dte_inode_permission (struct inode *inode, int mask)
{
	if (dte_bitchmode)
		return dte_bitch_inode_permission(inode, mask);
	else
		return dte_real_inode_permission(inode, mask);
}
