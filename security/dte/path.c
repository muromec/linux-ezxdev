/*
 * security/dte/path.c
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

struct dte_map_node   *dte_root_mapnode;
int num_dte_map_nodes;
extern kmem_cache_t *dte_map_cache;   /* dte_map_node cache */

/*
* dte_d_path:
* This version of __d_path follows the dte rules for deciding a
* pathname.  It always uses the real root, and (when implemented, will)
* follows the pretend mounts.
 */
char * dte_d_path(struct dentry *dentry, struct vfsmount *vfsmnt,
		char *buffer, int buflen)
{
	char * end = buffer+buflen;
	char * retval;
	int namelen;
	struct dte_sb_sec *sb_sec;

	*--end = '\0';
	buflen--;
	if (!IS_ROOT(dentry) && list_empty(&dentry->d_hash)) {
		buflen -= 10;
		end -= 10;
		memcpy(end, " (deleted)", 10);
	}

	/* Get '/' right */
	retval = end-1;
	*retval = '/';

	if (!dentry)
		panic("dentry is NULL.\n");
	if (!vfsmnt)
		panic("vfsmnt is NULL.\n");

	for (;;) {
		struct dentry * parent;

		if (dentry == vfsmnt->mnt_root || IS_ROOT(dentry)) {
			/* Global root? */
			if (vfsmnt->mnt_parent == vfsmnt)
				goto global_root;

			sb_sec = (struct dte_sb_sec *)vfsmnt->mnt_sb->s_security;
			if (!sb_sec) {
				printk(KERN_ERR "at %s(maj %d/min %d): superblock has no dte_sec.\n",
						dentry->d_iname, MAJOR(vfsmnt->mnt_sb->s_dev),
						MINOR(vfsmnt->mnt_sb->s_dev));
				dentry = vfsmnt->mnt_mountpoint;
				vfsmnt = vfsmnt->mnt_parent;
			} else {
				dentry = sb_sec->mountpoint;
				vfsmnt = sb_sec->mnt_parent;
			}
			continue;
		}
		parent = dentry->d_parent;
		namelen = dentry->d_name.len;
		buflen -= namelen + 1;
		if (buflen < 0)
			break;
		end -= namelen;
		memcpy(end, dentry->d_name.name, namelen);
		*--end = '/';
		retval = end;
		dentry = parent;
	}
global_root:
	namelen = dentry->d_name.len;
	buflen -= namelen;
	if (buflen >= 0) {
		retval -= namelen-1;    /* hit the slash */
		memcpy(retval, dentry->d_name.name, namelen);
	}
	return retval;
}

#define map_strcmp(node, str, inlen) \
	(strncmp(node->name, str, inlen)!=0 || inlen!=node->namelen)

/*
 * called in read_policy.c:read_ta()
 */
struct dte_map_node *dte_find_map_node_create(char *path)
{
	int done, len;
	char *ptr, *ptre;  /* string pointer, end of string pointer */
	struct dte_map_node *c, *p;  /* child, parent map nodes */
	struct dte_map_node *tn, *tp; /* temp map node, and its hashprev */
	char *str;

	ptre=path;
	/* we insist on having full pathnames! */
	if (*ptre!='/') {
		printk(KERN_ERR "dte_find_map_node_create: sent pathname w/out leading /");
		return NULL;
	}

	if (!num_dte_map_nodes) {
		printk(KERN_ERR "dte_find_map_node_create: root map node not yet created!\n");
		return NULL;
	}

	done = 0;
	p    = dte_root_mapnode;
	while (!done) {
		ptr = ++ptre;
		while (*ptre!='/' && *ptre!='\0') ptre++;
		if (*ptre=='\0') done=1;
		len = ptre-ptr;
		if (len<=1) continue;
		if (!p->num_kids) {
			p->kids = kmalloc(sizeof(struct dte_map_node **),GFP_KERNEL);
			if (!p->kids) {
				printk(KERN_NOTICE "dte-map-create: out of memory.\n");
				return NULL;
			}
			p->kids[0] = NULL;
		}
		tn = tp = p->kids[0];
		if (!tn) {
			/* The parent map node has no children yet */
			c = kmem_cache_alloc(dte_map_cache, GFP_KERNEL);
			if (!c) {
				printk(KERN_ERR "dte_find_map_node_create: out of memory.\n");
				return NULL;
			}
			p->kids[0] = c;
			p->num_kids = 1;
			c->etype = c->utype = NULL;
			c->hash_next = NULL;
			c->kids = NULL;
			c->num_kids = 0;
			str = kmalloc(NAME_ALLOC_LEN(len), GFP_KERNEL);
			if (!str) {
				printk(KERN_ERR "dte_find_map_node_create: out of memory.\n");
				return NULL;
			}
			memcpy(str,ptr,len);
			str[len]=0;
			c->name = str;
			c->namelen  = len;
			p = c;
			continue;
		}
		while (tn) {
			if (map_strcmp(tn,ptr,len)==0) {
				/* this map node exists */
				p = tn;
				break;
			}
			tp = tn;
			tn = tn->hash_next;
		}
		if (tn)
			/* this piece existed and we've set the parentmap (p) */
			continue;
		/* this piece does not exist in the map node tree yet. */
		tp->hash_next = kmem_cache_alloc(dte_map_cache, GFP_KERNEL);
		tp = tp->hash_next;
		if (!tp) {
			printk(KERN_NOTICE "dte_find_map_node_create: out of memory.\n");
			return NULL;
		}
		tp->etype = tp->utype = NULL;
		tp->hash_next = NULL;
		tp->kids = NULL;
		tp->num_kids = 0;
		str = (char *) kmalloc(NAME_ALLOC_LEN(len), GFP_KERNEL);
		if (!str) {
			printk(KERN_NOTICE "dte_find_map_node_create: out of memory.\n");
			return NULL;
		}
		memcpy(str,ptr,len);
		str[len]=0;
		tp->name = str;
		tp->namelen  = len;
		/* can't yet create accurate hash: don't know num_kids for sure */
		p->num_kids++;
		p = tp;
	}
	return p;
}

/* convert the map_node tree to a hash table.  We keep the siblings
 * as a linked list while reading it in
 *
 * this requires at dte_read_policy:
 * mapnode->hash_next = NULL;
 */
int dte_c_maptohash(struct dte_map_node *p)
{
	struct dte_map_node *tmp;
	struct dte_map_node **tmparray;
	int i, lo;
	unsigned int h;

	if (!p->num_kids) return 0;

	/* first realloc the parent's child array from 1 to num_kids */
	tmparray = kmalloc((p->num_kids*sizeof(struct dte_map_node *)),
			GFP_KERNEL);
	if (!tmparray) {
		printk(KERN_NOTICE "dte_c_maptohash: out of mem.\n");
		return -ENOMEM;
	}
	tmparray[0] = p->kids[0];
	kfree(p->kids);
	tmp = tmparray[0];
	for (i=1; i<p->num_kids; i++) {
		tmp = tmp->hash_next;
		tmparray[i] = tmp;
	}

	p->kids = kmalloc((p->num_kids*sizeof(struct dte_map_node *)),
			GFP_KERNEL);
	if (!p->kids) {
		printk(KERN_NOTICE "dte_c_maptohash: out of mem.\n");
		return -ENOMEM;
	}
	memset(p->kids,0,p->num_kids*sizeof(struct dte_map_node *));

	lo = 0;
	for (i=0; i<p->num_kids; i++) {
		h = dte_hash(tmparray[i]->name,p->num_kids);
		tmp = p->kids[h];
		while (tmp && tmp->hash_next)
			tmp = tmp->hash_next;
		if (tmp) {
			while (p->kids[lo])
				lo++;
			p->kids[lo] = tmparray[i];
			tmp->hash_next = tmparray[i];
			tmp->hash_next->hash_next = NULL;
			lo++;
		} else {
			p->kids[h] = tmparray[i];
			p->kids[h]->hash_next = NULL;
		}
	}
	kfree(tmparray);

	for (i=0; i<p->num_kids; i++)
		dte_c_maptohash(p->kids[i]);

	return 1;
}

struct dte_map_node *mapnode_getkid(struct dte_map_node *m,
			const unsigned char *name)
{
	struct dte_map_node *tmp;
	unsigned int h;

	h = dte_hash(name, m->num_kids);
	tmp = m->kids[h];
	while (tmp && strcmp(name, tmp->name)!=0)
		tmp = tmp->hash_next;

	return tmp;
}
