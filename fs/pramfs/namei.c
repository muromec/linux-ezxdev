/*
 * namei.c
 *
 * Inode operations for directories.
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc.
 * Copyright 2003 Sony Corporation
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 *
 * This software is being distributed under the terms of the GNU General Public
 * License version 2.  Some or all of the technology encompassed by this
 * software may be subject to one or more patents pending as of the date of
 * this notice.  No additional patent license will be required for GPL
 * implementations of the technology.  If you want to create a non-GPL
 * implementation of the technology encompassed by this software, please
 * contact legal@mvista.com for details including licensing terms and fees.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#include <linux/fs.h>
#include <linux/pram_fs.h>
#include <linux/pagemap.h>


/*
 * Couple of helper functions - make the code slightly cleaner.
 */

static inline void pram_inc_count(struct inode *inode)
{
	inode->i_nlink++;
	pram_write_inode(inode, 0);
}

static inline void pram_dec_count(struct inode *inode)
{
	if (inode->i_nlink) {
		inode->i_nlink--;
		pram_write_inode(inode, 0);
	}
}

static inline int pram_add_nondir(struct inode * dir,
				   struct dentry * dentry,
				   struct inode * inode)
{
	int err = pram_add_link(dentry, inode);
	if (!err) {
		d_instantiate(dentry, inode);
		return 0;
	}
	pram_dec_count(inode);
	iput(inode);
	return err;
}

/*
 * Methods themselves.
 */

static ino_t
pram_inode_by_name(struct inode * dir,
		   struct dentry * dentry)
{
	struct pram_inode * pi;
	ino_t ino;
	int namelen;

	pi = pram_get_inode(dir->i_sb, dir->i_ino);
	ino = pi->i_type.dir.head;
	
	while (ino) {
		pi = pram_get_inode(dir->i_sb, ino);

		if (pi->i_links_count) {
			namelen = strlen(pi->i_d.d_name);
		
			if (namelen == dentry->d_name.len &&
			    !memcmp(dentry->d_name.name,
				    pi->i_d.d_name, namelen))
				break;
		}
		
		ino = pi->i_d.d_next;
	}
	
	return ino;
}

static struct dentry *
pram_lookup(struct inode * dir, struct dentry *dentry)
{
	struct inode * inode = NULL;
	ino_t ino;

	if (dentry->d_name.len > PRAM_NAME_LEN)
		return ERR_PTR(-ENAMETOOLONG);

	ino = pram_inode_by_name(dir, dentry);
	if (ino) {
		struct pram_inode * pi = pram_get_inode(dir->i_sb, ino);
		inode = pram_fill_new_inode(dir->i_sb, pi);
		if (!inode)
			return ERR_PTR(-EACCES);
	}

	d_add(dentry, inode);
	return NULL;
}


/*
 * By the time this is called, we already have created
 * the directory cache entry for the new file, but it
 * is so far negative - it has no inode.
 *
 * If the create succeeds, we fill in the inode information
 * with d_instantiate(). 
 */
static int pram_create (struct inode * dir, struct dentry * dentry, int mode)
{
	struct inode * inode = pram_new_inode (dir, mode);
	int err = PTR_ERR(inode);
	if (!IS_ERR(inode)) {
		inode->i_op = &pram_file_inode_operations;
		inode->i_fop = &pram_file_operations;
		inode->i_mapping->a_ops = &pram_aops;
		err = pram_add_nondir(dir, dentry, inode);
	}
	return err;
}

static int pram_mknod (struct inode * dir, struct dentry *dentry, int mode,
			int rdev)
{
	struct inode * inode = pram_new_inode (dir, mode);
	int err = PTR_ERR(inode);
	if (!IS_ERR(inode)) {
		init_special_inode(inode, mode, rdev);
		err = pram_add_nondir(dir, dentry, inode);
	}
	return err;
}

static int pram_symlink (struct inode * dir,
			  struct dentry * dentry,
			  const char * symname)
{
	struct super_block * sb = dir->i_sb;
	int err = -ENAMETOOLONG;
	unsigned len = strlen(symname);
	struct inode * inode;

	if (len+1 > sb->s_blocksize)
		goto out;

	inode = pram_new_inode (dir, S_IFLNK | S_IRWXUGO);
	err = PTR_ERR(inode);
	if (IS_ERR(inode))
		goto out;

	inode->i_op = &pram_symlink_inode_operations;
	inode->i_mapping->a_ops = &pram_aops;
	err = pram_block_symlink(inode, symname, len);
	if (err)
		goto out_fail;

	inode->i_size = len;
	pram_write_inode(inode, 0);

	err = pram_add_nondir(dir, dentry, inode);
out:
	return err;

out_fail:
	pram_dec_count(inode);
	iput (inode);
	goto out;
}

static int pram_link (struct dentry * dest_dentry,
		       struct inode * dir,
		       struct dentry * dentry)
{
	pram_dbg ("hard links not supported\n");
	return -ENOSYS;
}

static int pram_unlink(struct inode * dir, struct dentry *dentry)
{
	struct inode * inode = dentry->d_inode;
	inode->i_ctime = dir->i_ctime;
	pram_dec_count(inode);

	return 0;
}

static int pram_mkdir(struct inode * dir, struct dentry * dentry, int mode)
{
	struct inode * inode;
	struct pram_inode * pi;
	unsigned long flags;
	int err = -EMLINK;

	if (dir->i_nlink >= PRAM_LINK_MAX)
		goto out;

	pram_inc_count(dir);
	
	inode = pram_new_inode (dir, S_IFDIR | mode);
	err = PTR_ERR(inode);
	if (IS_ERR(inode))
		goto out_dir;

	inode->i_op = &pram_dir_inode_operations;
	inode->i_fop = &pram_dir_operations;
	inode->i_mapping->a_ops = &pram_aops;

	pram_inc_count(inode);

	// make the new directory empty
	pi = pram_get_inode(dir->i_sb, inode->i_ino);
	pram_lock_inode(pi);
	pi->i_type.dir.head = pi->i_type.dir.tail = 0;
	pram_unlock_inode(pi);

	err = pram_add_link(dentry, inode);
	if (err)
		goto out_fail;

	d_instantiate(dentry, inode);
out:
	return err;

out_fail:
	pram_dec_count(inode);
	pram_dec_count(inode);
	iput(inode);
out_dir:
	pram_dec_count(dir);
	goto out;
}

static int pram_rmdir (struct inode * dir, struct dentry *dentry)
{
	struct inode * inode = dentry->d_inode;
	struct pram_inode * pi;
	int err = -ENOTEMPTY;
	
	if (!inode)
		return -ENOENT;

	pi = pram_get_inode(dir->i_sb, inode->i_ino);

	// directory to delete is empty?
	if (pi->i_type.dir.tail == 0) {
		inode->i_ctime = dir->i_ctime;
		inode->i_size = 0;
		inode->i_nlink = 0;
		pram_write_inode(inode, 0);
		pram_dec_count(dir);
		err = 0;
	} else {
		pram_dbg("dir not empty\n");
	}
	
	return err;
}

static int pram_rename (struct inode  * old_dir,
			struct dentry * old_dentry,
			struct inode  * new_dir,
			struct dentry * new_dentry)
{
	struct inode * old_inode = old_dentry->d_inode;
	struct inode * new_inode = new_dentry->d_inode;
	struct pram_inode * pi_new;
	int err = -ENOENT;

	if (new_inode) {
		err = -ENOTEMPTY;
		pi_new = pram_get_inode(new_dir->i_sb, new_inode->i_ino);
		if (S_ISDIR(old_inode->i_mode)) {
 			if (pi_new->i_type.dir.tail != 0)
				goto out;
			if (new_inode->i_nlink)
				new_inode->i_nlink--;
		}

		new_inode->i_ctime = CURRENT_TIME;
		pram_dec_count(new_inode);
	} else {
		if (S_ISDIR(old_inode->i_mode)) {
			err = -EMLINK;
			if (new_dir->i_nlink >= PRAM_LINK_MAX)
				goto out;
			pram_dec_count(old_dir);
			pram_inc_count(new_dir);
		}
	}

	/* unlink the inode from the old directory ... */
	if ((err = pram_remove_link(old_inode))) {
		goto out;
	}
	/* and link it into the new directory. */
	if ((err = pram_add_link(new_dentry, old_inode))) {
		goto out;
	}

	err = 0;
 out:
	return err;
}

struct inode_operations pram_dir_inode_operations = {
	create:		pram_create,
	lookup:		pram_lookup,
	link:		pram_link,
	unlink:		pram_unlink,
	symlink:	pram_symlink,
	mkdir:		pram_mkdir,
	rmdir:		pram_rmdir,
	mknod:		pram_mknod,
	rename:		pram_rename,
};
