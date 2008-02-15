/*
 * dir.c
 *
 * File operations for directories.
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
 *	Parent is locked.
 */
int pram_add_link (struct dentry * dentry, struct inode * inode)
{
	struct inode * dir = dentry->d_parent->d_inode;
	struct pram_inode * pidir, * pi, * pitail = NULL;
	unsigned long flags;
	pram_off_t tail_ino, prev_ino;
	const char *name = dentry->d_name.name;
	int namelen = dentry->d_name.len > PRAM_NAME_LEN ?
		PRAM_NAME_LEN : dentry->d_name.len;
	
	pidir = pram_get_inode(dir->i_sb, dir->i_ino);
	pi = pram_get_inode(dir->i_sb, inode->i_ino);
	
	dir->i_mtime = dir->i_ctime = CURRENT_TIME;

	tail_ino = pidir->i_type.dir.tail;
	if (tail_ino != 0) {
		pitail = pram_get_inode(dir->i_sb, tail_ino);
		pram_lock_inode(pitail);
		pitail->i_d.d_next = inode->i_ino;
		pram_unlock_inode(pitail);

		prev_ino = tail_ino;

		pram_lock_inode(pidir);
		pidir->i_type.dir.tail = inode->i_ino;
		pidir->i_mtime = dir->i_mtime;
		pidir->i_ctime = dir->i_ctime;
		pram_unlock_inode(pidir);
	} else {
		// the directory is empty
		prev_ino = 0;

		pram_lock_inode(pidir);
		pidir->i_type.dir.head = pidir->i_type.dir.tail = inode->i_ino;
		pidir->i_mtime = dir->i_mtime;
		pidir->i_ctime = dir->i_ctime;
		pram_unlock_inode(pidir);
	}


	pram_lock_inode(pi);
	pi->i_d.d_prev = prev_ino;
	pi->i_d.d_parent = dir->i_ino;
	memcpy(pi->i_d.d_name, name, namelen);
	pi->i_d.d_name[namelen] = '\0';
	pram_unlock_inode(pi);
	return 0;
}

int pram_remove_link(struct inode * inode)
{
	struct super_block * sb = inode->i_sb;
	struct pram_inode * prev = NULL;
	struct pram_inode * next = NULL;
	struct pram_inode * pidir, * pi;
	unsigned long flags;

	pi = pram_get_inode(sb, inode->i_ino);
	pidir = pram_get_inode(sb, pi->i_d.d_parent);
	if (!pidir)
		return -EACCES;
	
	if (inode->i_ino == pidir->i_type.dir.head) {
		// first inode in directory
		next = pram_get_inode(sb, pi->i_d.d_next);

		if (next) {
			pram_lock_inode(next);
			next->i_d.d_prev = 0;
			pram_unlock_inode(next);

			pram_lock_inode(pidir);
			pidir->i_type.dir.head = pi->i_d.d_next;
		} else {
			pram_lock_inode(pidir);
			pidir->i_type.dir.head = pidir->i_type.dir.tail = 0;
		}
		pram_unlock_inode(pidir);
	} else if (inode->i_ino == pidir->i_type.dir.tail) {
		// last inode in directory
		prev = pram_get_inode(sb, pi->i_d.d_prev);

		pram_lock_inode(prev);
		prev->i_d.d_next = 0;
		pram_unlock_inode(prev);

		pram_lock_inode(pidir);
		pidir->i_type.dir.tail = pi->i_d.d_prev;
		pram_unlock_inode(pidir);
	} else {
		// somewhere in the middle
		prev = pram_get_inode(sb, pi->i_d.d_prev);
		next = pram_get_inode(sb, pi->i_d.d_next);

		if (prev && next) {
			pram_lock_inode(prev);
			prev->i_d.d_next = pi->i_d.d_next;
			pram_unlock_inode(prev);
			
			pram_lock_inode(next);
			next->i_d.d_prev = pi->i_d.d_prev;
			pram_unlock_inode(next);
		}
	}
	
	pram_lock_inode(pi);
	pi->i_d.d_next = pi->i_d.d_prev = pi->i_d.d_parent = 0;
	pram_unlock_inode(pi);

	return 0;
}

#define S_SHIFT 12
static unsigned int dtype_by_mode[S_IFMT >> S_SHIFT] = {
        [S_IFREG >> S_SHIFT]    DT_REG,
        [S_IFDIR >> S_SHIFT]    DT_DIR,
        [S_IFCHR >> S_SHIFT]    DT_CHR,
        [S_IFBLK >> S_SHIFT]    DT_BLK,
        [S_IFIFO >> S_SHIFT]    DT_FIFO,
        [S_IFSOCK >> S_SHIFT]   DT_SOCK,
        [S_IFLNK >> S_SHIFT]    DT_LNK,
};
	
static int
pram_readdir (struct file * filp, void * dirent, filldir_t filldir)
{
	struct inode *inode = filp->f_dentry->d_inode;
	struct super_block * sb = inode->i_sb;
	struct pram_inode * pi;
	int namelen, ret=0;
	char *name;
	ino_t ino;

	if (filp->f_pos >> 32)
		return 0;

	pi = pram_get_inode(sb, inode->i_ino);

	switch ((unsigned long)filp->f_pos) {
	case 0:
		ret = filldir(dirent, ".", 1, 0, inode->i_ino, DT_DIR);
		filp->f_pos++;
		return ret;
	case 1:
		ret = filldir(dirent, "..", 2, 1, pi->i_d.d_parent, DT_DIR);
		ino = pi->i_type.dir.head;
		filp->f_pos = ino ? ino : 2;
		return ret;
	case 2:
		ino = pi->i_type.dir.head;
		if (ino) {
			filp->f_pos = ino;
			pi = pram_get_inode(sb, ino);
			break;
		} else {
			/* the directory is empty */
			filp->f_pos = 2;
			return 0;
		}
	case 3:
		return 0;
	default:
		ino = filp->f_pos;
		pi = pram_get_inode(sb, ino);
		break;
	}
	
	while (pi && !pi->i_links_count) {
		ino = filp->f_pos = pi->i_d.d_next;
		pi = pram_get_inode(sb, ino);
	}
	
	if (pi) {
		name = pi->i_d.d_name;
		namelen = strlen(name);
		
		ret = filldir(dirent, name, namelen,
			      filp->f_pos, ino,
			      dtype_by_mode[(pi->i_mode & S_IFMT)>>S_SHIFT]);
		filp->f_pos = pi->i_d.d_next ? pi->i_d.d_next : 3;
	} else
		filp->f_pos = 3;
	
	return ret;
}

struct file_operations pram_dir_operations = {
	read:		generic_read_dir,
	readdir:	pram_readdir,
	ioctl:		pram_ioctl,
	fsync:		pram_sync_file,
};
