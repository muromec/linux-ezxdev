/*
 * security/dte/mount.c
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
extern int num_dte_mount_r;
extern struct dte_mntr **dte_mount_r;
extern struct dte_map_node   *dte_root_mapnode;
extern int dte_bitchmode;
extern struct dte_domain_t *dte_init_domain;

static int xtoi(char *c)
{
	int ret = 0;

	while (*c==' ') c++;

	while (*c!='\n' && *c!='\0' && *c!=' ') {
		ret *= 16;
		switch(*c) {
			case 'a':
			case 'A': ret += 10; break;
			case 'b':
			case 'B': ret += 11; break;
			case 'c':
			case 'C': ret += 12; break;
			case 'd':
			case 'D': ret += 13; break;
			case 'e':
			case 'E': ret += 14; break;
			case 'f':
			case 'F': ret += 15; break;
			default: ret += ((*c)-'0');
		}
		c++;
	}
	return ret;
}

/* allocate and free the super-block's DTE security blob */
int dte_sb_alloc_security (struct super_block *sb)
{
	struct dte_sb_sec *sb_sec;

	/* temp test */
	printk(KERN_NOTICE "dte_sb_alloc_security: called on (%3d,%3d).\n",
			MAJOR(sb->s_dev), MINOR(sb->s_dev));
	if (sb->s_security)
		panic("dte_sb_alloc_security: already alloc'ed!\n");
	/* end temp test */
	sb_sec = sb->s_security = kmalloc(sizeof(struct dte_sb_sec), GFP_KERNEL);
	if (!sb_sec) {
		panic("dte_sb_alloc_security: out of memory.");
	}
	memset(sb_sec,0,sizeof(struct dte_sb_sec));
	sb_sec->fp = kmalloc(sizeof(struct file), GFP_KERNEL);
	sema_init(&sb_sec->s_sem, 1);
	return 0;
}

void dte_sb_free_security (struct super_block *sb)
{
	struct dte_sb_sec *sb_sec;

	printk(KERN_NOTICE "dte_sb_free_security: starting (%3d,%3d).\n",
			MAJOR(sb->s_dev), MINOR(sb->s_dev));
	sb_sec = (struct dte_sb_sec *)sb->s_security;
	sb_sec->initialized=0;
	kfree(sb_sec->type_conv);
	kfree(sb_sec);
	sb->s_security = NULL;
	printk(KERN_NOTICE "dte_sb_free_security: done (%3d,%3d).\n",
			MAJOR(sb->s_dev), MINOR(sb->s_dev));
}

/*
 * Look for the ea file
 * It will be in the root directory, named 'dteeaf'
 *
 * Note: mnt is null only when called from post_mountroot, in which case
 * sb->s_root is in fact root, and there's no fallback for deftype other
 * than the default utype (no parent inode)
 */
void dte_setup_eafile(struct super_block *sb, struct vfsmount *mnt)
{
	char *devname;
	struct dentry *dentry;
	char buf[1024], *bufp, *bufp2;
	struct dte_sb_sec *sb_sec;
	struct dte_inode_sec *p;
	long long offset;
	int i, err;
	struct file *fp;

	if (!dte_initialized)
		return;

	sb_sec = sb->s_security;
	if (!sb_sec) {
		printk(KERN_ERR "dte_setup_eafile: no s_security on the superblock! (%3d/%3d)\n",
				MAJOR(sb->s_dev), MINOR(sb->s_dev));
		dte_sb_alloc_security(sb);
		sb_sec = (struct dte_sb_sec *)sb->s_security;
	}
	if (sb_sec->initialized)
		return;

	if (mnt) {
		p = (struct dte_inode_sec *)
			mnt->mnt_mountpoint->d_inode->i_security;
		devname = mnt->mnt_devname;
		dentry = lookup_one_len("dteeaf", mnt->mnt_root, 6);
		err = PTR_ERR(dentry);
		if (IS_ERR(dentry) || !dentry->d_inode) {
			printk(KERN_NOTICE "dte_setup_eafile: error opening ea file for %s, %d.\n",
					devname, err);
			return;
		}
	} else {
		devname = "/dev/root";
		dentry = lookup_one_len("dteeaf", sb->s_root, 6);
		err = PTR_ERR(dentry);
		if (IS_ERR(dentry) || !dentry->d_inode) {
			printk(KERN_NOTICE "dte_setup_eafile: error opening ea file for %s, %d.\n",
					devname, err);
			return;
		}
	}

	err = open_private_file(sb_sec->fp, dentry, O_RDWR | O_SYNC);
	fp = sb_sec->fp;
	if (err)
	{
		printk(KERN_NOTICE "dte_setup_eafile: no dte ea file for %s, %d.\n",
				devname, err);
	} else if (!fp->f_op || !fp->f_op->read || !fp->f_op->write) {
		printk(KERN_NOTICE "dte_setup_eafile: no rw support for %s's ea file.\n",
				devname);
		dput(fp->f_dentry);
		close_private_file(fp);
	} else {
		/* read type table from the ea file */
		offset = 0;
		fp->f_op->read(fp, buf, 1024, &offset);
		sb_sec->ntypes = xtoi(buf);
		printk(KERN_NOTICE "dte_setup_eafile: there were %d types, buf %4s.\n",
				sb_sec->ntypes, buf);
		sb_sec->type_conv = kmalloc(sb_sec->ntypes*sizeof(char *), GFP_KERNEL);
		memset(sb_sec->type_conv, 0, sb_sec->ntypes*sizeof(char *));
		bufp2 = bufp = buf+4;
		for (i=0; i<sb_sec->ntypes; i++) {
			while (*bufp2!='\n') bufp2++;
			sb_sec->type_conv[i] = dte_get_type(bufp, bufp2);
			if (!sb_sec->type_conv[i]) {
				*bufp2 = '\0';
				panic("dev %s: can't find type number %d, %s.\n",
						devname, i, bufp);
			}
			bufp = ++bufp2;
		}
		sb_sec->offset = bufp2-buf;
		sb_sec->initialized = 1;
		sb_sec->fp_ready = 1;
	}
	printk(KERN_NOTICE "dte_setup_eafile: done\n");
}

/*
 * hierarchical_setup: hook up the superblock->true_parents.
 * called from dte_post_addmount, dte_post_mountroot, and
 * 		dte_walk_dcache_tree_full.
 * only wants to run on first mount of a device.
 */
void hierarchical_setup(struct vfsmount *mnt)
{
	struct dte_mntr *r;
	struct nameidata nd2;
	char path[500], devp[500];
	long retval=0;
	struct super_block *sb = mnt->mnt_sb;
	struct dte_sb_sec *sb_sec;

	if (!dte_initialized)
		return;
	sb_sec = (struct dte_sb_sec *)sb->s_security;
	if (atomic_read(&sb->s_active)>1 && sb_sec && sb_sec->mnt_parent)
		return;

	if (!sb_sec) {
		printk(KERN_ERR "hierarch_setup: no s_security on the superblock! (%3d/%3d)\n",
				MAJOR(sb->s_dev), MINOR(sb->s_dev));
		dte_sb_alloc_security(sb);
		sb_sec = (struct dte_sb_sec *)sb->s_security;
	}
	/* here's one of the saddest parts about doing this with lsm:
	 * I have to find the mntr a second time
	 */
	if (num_dte_mount_r) {
		sprintf(devp,"%3d,%3d",MAJOR(sb->s_dev),MINOR(sb->s_dev));
		/*
		* now look for a restriction on this device ...
		*/
		r = dte_mount_r[dte_hash(devp,num_dte_mount_r)];
		while (r && strcmp(devp,r->summ))
			r = r->hash_next;
	} else {
		/* no pretends, use the given parents */
		sb_sec->mnt_parent = mntget(mnt->mnt_parent);
		sb_sec->mountpoint = dget(mnt->mnt_mountpoint);
		return;
	}

	if (r && r->how=='p') {
		/*
		* got a pretend mount to hook up
		*/
		/* make sure we check against the real pathname, no bind trickery */
		printk(KERN_NOTICE "hooking up pretend for %s.\n",devp);
		if (path_init(r->path,
					LOOKUP_FOLLOW|LOOKUP_POSITIVE|LOOKUP_DIRECTORY, &nd2))
			retval = path_walk(r->path, &nd2);
		if (retval) {
			printk(KERN_NOTICE "dte_mount: path pretend %s does not exist!\n",
					r->path);
			/* so use the given parents */
			sb_sec->mnt_parent = mntget(mnt->mnt_parent);
			sb_sec->mountpoint = dget(mnt->mnt_mountpoint);
		} else {
			/* debug */
			dte_d_path(nd2.dentry,nd2.mnt,path,500);
			printk(KERN_NOTICE "dte_mount: mounting %s at %s.\n",devp,path);
			/* /debug */
			sb_sec->mnt_parent = mntget(nd2.mnt);
			sb_sec->mountpoint = dget(nd2.dentry);
			path_release(&nd2);
		}
	}
}

static void dte_walk_dcache_tree(struct dentry *parent)
{
	struct dentry *this_parent = parent;
	struct list_head *next;

repeat:
	next = this_parent->d_subdirs.next;
resume:
	while (next != &this_parent->d_subdirs) {
		struct list_head *tmp = next;
		struct dentry *dentry = list_entry(tmp, struct dentry, d_child);
		next = tmp->next;

		if (dentry->d_inode) {
			dte_inode_alloc_security(dentry->d_inode);
			dte_post_lookup(this_parent->d_inode, dentry);
		}
		if (!list_empty(&dentry->d_subdirs)) {
			this_parent = dentry;
			goto repeat;
		}
	}
	/*
	 * All done at this level ... ascend and resume the search.
	 */
	if (this_parent != parent) {
		next = this_parent->d_child.next;
		this_parent = this_parent->d_parent;
		goto resume;
	}
}

void dte_post_mountroot (void)
{
	struct super_block *sb;
	struct dte_inode_sec *s;
	struct dte_task_sec *task_sec;
	struct task_struct *p;
	mm_segment_t old_fs;

	if (read_dte_config()) {
		panic("Error reading DTE policy!\n");
	}
	if (!current->fs || !current->fs->rootmnt || !current->fs->rootmnt->mnt_sb) {
		printk(KERN_NOTICE "dte_post_mountroot: no sb\n");
		return;
	}
	sb = current->fs->rootmnt->mnt_sb;
	if (!sb->s_root || !sb->s_root->d_inode) {
		printk(KERN_NOTICE "dte_post_mountroot: dammit, no root d or inode.\n");
		return;
	}
	s = sb->s_root->d_inode->i_security;
	s->map = dte_root_mapnode;
	s->etype = dte_root_mapnode->etype;
	s->utype = dte_root_mapnode->utype;
	s->initialized = 1;

	if (dte_c_maptohash(dte_root_mapnode)!=1) {
		printk(KERN_EMERG "Uh-oh : Trouble hashing mapnodes.\n");
	}

	dte_walk_dcache_tree(sb->s_root);

	/*
	 * assign default label to running processes, of which there should
	 * be two : idle task and init
	 */
	for_each_task(p) {
		p->security = kmalloc(sizeof(struct dte_task_sec), GFP_KERNEL);
		task_sec = (struct dte_task_sec *)p->security;
		task_sec->dte_domain = dte_init_domain;
		task_sec->dte_back   = NULL;
	}

	/* setup gateways (for auto switches) from entry point data */
	if (dte_setup_gateways())
		panic("DTE: No memory for gateways.\n");
	if (dte_sort_mntrs())
		panic("DTE: NO memory for mount restrictions.\n");

	if (!dte_root_mapnode || !dte_root_mapnode->etype ||
			!dte_root_mapnode->utype)
		panic("Whoa: DTE: no root etype/utype set.  Stopping.\n");

#ifdef CONFIG_DTE_VERBOSE
	show_dte();
#endif

	printk(KERN_NOTICE "dte_post_mountroot: almost finished.\n");
	dte_initialized = 1;
	/* external attribute file setup */
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	dte_setup_eafile(sb, NULL);
	set_fs(old_fs);
	/* hierarchical type assignment setup */
	hierarchical_setup(current->fs->rootmnt);
	printk(KERN_NOTICE "dte_post_mountroot: finished.\n");
}

void dte_post_addmount (struct vfsmount *mnt, struct nameidata *nd)
{
	struct dte_inode_sec *p;
	struct dentry *mntroot = mnt->mnt_root;
	mm_segment_t old_fs;

	if (!dte_initialized)
		return ;

	printk(KERN_NOTICE "dte_post_addmount: Called on %s.\n",
			mnt->mnt_devname);
	if (!mnt->mnt_root) {
		printk(KERN_NOTICE "dte_post_addmount: no root dentry for dev %s.\n",
				mnt->mnt_devname);
		return;
	}

	/*
	 * check for external attributes file
	 */
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	dte_setup_eafile(mnt->mnt_sb, mnt);
	set_fs(old_fs);

	/*
	 * handle the 'true parent' hookups
	 */
	hierarchical_setup(mnt);

	/*
	 * lots of oops checks, but mainly just set the security and type info
	 * on the root inode
	 */
	if (!mntroot->d_inode) {
		printk(KERN_NOTICE "dte_post_addmount: no inode for root dentry for %s.\n",
				mnt->mnt_devname);
		return;
	}
	p = (struct dte_inode_sec *)mntroot->d_inode->i_security;
	if (!p) {
		/* this would only happen if dte was not yet initialized when
		 * the fs was first loaded */
		printk(KERN_NOTICE
				"dte_post_addmount: root dentry+inode, but no i_sec for %s.\n",
				mnt->mnt_devname);
		printk(KERN_NOTICE
				"dte_post_addmount: this is for inode %lu, dentry %s.\n",
				mntroot->d_inode->i_ino, mntroot->d_name.name);
		dte_copy_ino_sec(nd->dentry->d_inode, mntroot->d_inode);
		dte_walk_dcache_tree(mntroot);
		return;
	}

	if (p->etype)
		return;
	dte_copy_ino_sec(nd->dentry->d_inode, mntroot->d_inode);
}

int dte_mount (char * dev_name, struct nameidata *nd, char * type,
			unsigned long flags, void * data)
{
	return 0;
}

int dte_umount (struct vfsmount *mnt, int flags)
{
	struct super_block *sb = mnt->mnt_sb;
	struct dte_sb_sec *sb_sec = sb->s_security;

	printk(KERN_NOTICE "dte_umount: cleaning up ea fp and parents for %s (0).\n",
			mnt->mnt_devname);
	down(&sb_sec->s_sem);
	if (atomic_read(&sb->s_active) != 1) {
		up(&sb_sec->s_sem);
		return 0;
	}
	if (!sb_sec) {
		up(&sb_sec->s_sem);
		return 0;
	}
	if (!sb_sec->mountpoint) {
		up(&sb_sec->s_sem);
		return 0;
	}

	printk(KERN_NOTICE "dte_umount: cleaning up ea fp and parents for %s (1).\n",
			mnt->mnt_devname);
	if (sb_sec->fp_ready) {
		printk(KERN_NOTICE "dte_umount: dput'ing eafp.\n");
		dput(sb_sec->fp->f_dentry);
		sb_sec->fp_ready = 0;
		close_private_file(sb_sec->fp);
		printk(KERN_NOTICE "dte_umount: dput'ed eafp.\n");
	}
	if (sb_sec->fp) {
		kfree(sb_sec->fp);
		sb_sec->fp = NULL;
	}
	dput(sb_sec->mountpoint);
	mntput(sb_sec->mnt_parent);
	sb_sec->mountpoint = NULL;
	sb_sec->mnt_parent = NULL;
	printk(KERN_NOTICE "dte_umount: done with %s (2).\n",
			mnt->mnt_devname);
	up(&sb_sec->s_sem);
	return 0;
}

int dte_check_sb (struct vfsmount *mnt, struct nameidata *nd)
{
	struct super_block *sb = mnt->mnt_sb;
	struct dte_mntr *r;
	char path[500];
	char devp[500];
	int retval;

#ifdef CONFIG_DTE_VERBOSE
	printk(KERN_NOTICE "dte_check_sb: called (%3d,%3d).\n",
		MAJOR(sb->s_dev),MINOR(sb->s_dev));
#endif

	if (dte_initialized && num_dte_mount_r) {
		retval = -EPERM;

		sprintf(devp,"%3d,%3d",MAJOR(sb->s_dev),MINOR(sb->s_dev));

		/*
		* now look for a restriction on this device ...
		*/
		r = dte_mount_r[dte_hash(devp,num_dte_mount_r)];
		while (r && strcmp(devp,r->summ) && r->hash_next)
			r = r->hash_next;
		if (r && strcmp(devp, r->summ)==0) {
			dte_d_path(nd->dentry, nd->mnt, path, 500);
			if (r->how=='r' && strcmp(r->path, path)) {
				printk(KERN_NOTICE "dte_check_sb: dte forbids mounting %s except at %s.\n",
						r->summ, r->path);
/*				up(&nd.dentry->d_inode->i_zombie);*/
				if (dte_bitchmode) {
					printk(KERN_NOTICE "dte_check_sb: would deny.\n");
					return 0;
				}
				return retval;
			}
		}
	}
	return 0;
}
