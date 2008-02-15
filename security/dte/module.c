/*
 * security/dte/module.c
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
extern struct dte_map_node   *dte_root_mapnode;
extern struct dte_domain_t *default_domain;

/* a version of walk_dcache_tree which also crosses mounts */
/* it's called only at dte init, so it also sets up the superblocks */
void dte_walk_dcache_tree_full(struct vfsmount *pmnt, struct dentry *parent)
{
	struct dentry *this_parent = parent;
	struct vfsmount *mnt;
	struct list_head *next;
	mm_segment_t old_fs;

	/*
	 * check for external attributes file
	 * handle the 'true parent' hookups
	 */
	printk(KERN_NOTICE "dte_walk_dcache_tree_full: called on %s.\n",
			pmnt->mnt_devname);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	dte_setup_eafile(pmnt->mnt_sb, pmnt);
	set_fs(old_fs);
	hierarchical_setup(pmnt);
	printk(KERN_NOTICE "dte_walk_dcache_tree_full: did setup on %s.\n",
			pmnt->mnt_devname);

repeat:
	next = this_parent->d_subdirs.next;
resume:
	while (next != &this_parent->d_subdirs) {
		struct list_head *tmp = next;
		struct dentry *dentry = list_entry(tmp, struct dentry, d_child);
		next = tmp->next;

		if (dentry->d_inode) {
#ifdef CONFIG_DTE_DEBUG
			printk(KERN_NOTICE "dte_walk_dcache_tree_full: allocing sec on %s.\n",
					dentry->d_iname);
#endif
			dte_inode_alloc_security(dentry->d_inode);
#ifdef CONFIG_DTE_DEBUG
			printk(KERN_NOTICE "dte_walk_dcache_tree_full: starting lookup on %s.\n",
					dentry->d_iname);
#endif
			dte_post_lookup(this_parent->d_inode, dentry);
#ifdef CONFIG_DTE_DEBUG
			printk(KERN_NOTICE "dte_walk_dcache_tree_full: finished lookup on %s.\n",
					dentry->d_iname);
#endif
		}
		if (dentry->d_mounted) {
			mnt = lookup_mnt(pmnt, dentry);
			/* if it's a bind or we've already assigned this fs, skip */
			if (mnt && !mnt->mnt_root->d_inode->i_security &&
					mnt->mnt_root == mnt->mnt_sb->s_root) {
				printk(KERN_NOTICE "dte_walk_dcache_tree_full: setting root for %s.\n",
						mnt->mnt_devname);
				dte_copy_ino_sec(dentry->d_inode, mnt->mnt_root->d_inode);
				printk(KERN_NOTICE "dte_walk_dcache_tree_full: descending to %s.\n",
						mnt->mnt_devname);
				dte_walk_dcache_tree_full(mnt, mnt->mnt_root);
			} else if (mnt) {
				printk(KERN_NOTICE "walk_dcache_tree_full: not descending %s.\n",
						mnt->mnt_devname);
				if (mnt->mnt_root->d_inode->i_security) {
					printk(KERN_NOTICE "walk_dcache_tree_full: i_sec set for %s.\n",
							mnt->mnt_devname);
				}
				if (mnt->mnt_root != mnt->mnt_sb->s_root) {
					printk(KERN_NOTICE "walk_dcache_tree_full: s_root!=mnt_root,%s.\n",
							mnt->mnt_devname);
				}
			} else {
				printk(KERN_NOTICE "walk_dcache_tree_full: no mnt under %s,%s.\n",
						pmnt->mnt_devname, dentry->d_iname);
			}
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
/*	spin_unlock(&dcache_lock);*/
}

int setup_dte_module(void)
{
	struct vfsmount *root_mnt;
	struct super_block *root_sb;
	struct dte_sb_sec *root_sbsec;
	struct task_struct *taskp;
	struct dte_task_sec *task_sec;
	struct dte_inode_sec *root_ino_sec;

	if (read_dte_config()) {
		printk(KERN_ERR "DTE: Error reading config file!\n");
		return -1;
	}

	if (dte_setup_gateways()) {
		printk(KERN_ERR "DTE: Error setting up gateways.\n");
		free_dte_memory(4);
		return -ENOMEM;
	}

	if (dte_sort_mntrs()) {
		printk(KERN_ERR "DTE: Error sorting mount restrictions.\n");
		free_dte_memory(5);
		return -ENOMEM;
	}

	if (dte_c_maptohash(dte_root_mapnode)!=1) {
		printk(KERN_EMERG "Uh-oh : Trouble hashing mapnodes.\n");
	}

	/*
	* Might want to work on making the next step more intelligent:  Try to
	* assign the *appropriate* domains?
	*/
	lock_kernel();
	for_each_task(taskp) {
		taskp->security = kmalloc(sizeof(struct dte_task_sec), GFP_KERNEL);
		task_sec = (struct dte_task_sec *)taskp->security;
		task_sec->dte_domain = default_domain;
		task_sec->dte_back   = NULL;
	}

	root_mnt = current->fs->rootmnt;
	root_sb = root_mnt->mnt_sb;
	dte_sb_alloc_security(root_sb);
	root_sbsec = (struct dte_sb_sec *)root_sb->s_security;

	dte_inode_alloc_security(root_mnt->mnt_root->d_inode);
	root_ino_sec = (struct dte_inode_sec *)
		root_mnt->mnt_root->d_inode->i_security;
	root_ino_sec->map = dte_root_mapnode;
	root_ino_sec->etype = dte_root_mapnode->etype;
	root_ino_sec->utype = dte_root_mapnode->utype;
	root_ino_sec->initialized = 1;

	dte_initialized = 1;
	dte_walk_dcache_tree_full(root_mnt, root_sb->s_root);
	unlock_kernel();
	return 0;
}

