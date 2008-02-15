/*
 * symlink.c
 *
 * Symlink methods.
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

int pram_block_symlink(struct inode *inode, const char *symname, int len)
{
	struct super_block * sb = inode->i_sb;
	pram_off_t block;
	char* blockp;
	unsigned long flags;
	int err;
	
	err = pram_alloc_blocks (inode, 0, 1);
	if (err)
		return err;

	block = pram_find_data_block(inode, 0);
	blockp = pram_get_block(sb, block);
	
	pram_lock_block(sb, blockp);
	memcpy(blockp, symname, len);
	blockp[len] = '\0';
	pram_unlock_block(sb, blockp);
	return 0;
}

static int pram_readlink(struct dentry *dentry, char *buffer, int buflen)
{
	struct inode * inode = dentry->d_inode;
	struct super_block * sb = inode->i_sb;
	pram_off_t block;
	char* blockp;
	
	block = pram_find_data_block(inode, 0);
	blockp = pram_get_block(sb, block);
	return vfs_readlink(dentry, buffer, buflen, blockp);
}

static int pram_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	struct inode * inode = dentry->d_inode;
	struct super_block * sb = inode->i_sb;
	pram_off_t block;
	char* blockp;

	block = pram_find_data_block(inode, 0);
	blockp = pram_get_block(sb, block);
	return vfs_follow_link(nd, blockp);
}

struct inode_operations pram_symlink_inode_operations = {
	readlink:	pram_readlink,
	follow_link:	pram_follow_link,
};
