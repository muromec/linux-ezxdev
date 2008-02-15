/*
 * file.c
 *
 * File operations for regular files.
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
#include <linux/sched.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

/*
 * Called when an inode is released. Note that this is different
 * from pram_open_file: open gets called at every open, but release
 * gets called only when /all/ the files are closed.
 */
static int pram_release_file (struct inode * inode, struct file * filp)
{
	return 0;
}

static int pram_direct_IO(int rw,
			   struct inode * inode,
			   char * buf,
			   loff_t offset,
			   int length)
{
	struct super_block * sb = inode->i_sb;
	int progress = 0, retval = 0;
	struct pram_inode * pi;
	void * tmp = NULL;
	unsigned long blocknr, blockoff, flags;
	int num_blocks, blocksize_mask, blocksize, blocksize_bits;

	blocksize_bits = inode->i_sb->s_blocksize_bits;
	blocksize = 1 << blocksize_bits;
	blocksize_mask = blocksize - 1;

	/* find starting block number to access */
	blocknr = offset >> blocksize_bits;
	/* find starting offset within starting block */
	blockoff = offset & blocksize_mask;
	/* find number of blocks to access */
	if ((blockoff + length) & blocksize_mask)
		num_blocks = (blockoff + length + blocksize) >> blocksize_bits;
	else
		num_blocks = (blockoff + length) >> blocksize_bits;
		
	if (rw == WRITE) {
		// prepare a temporary buffer to hold a user data block
		// for writing.
		tmp = kmalloc(blocksize, GFP_KERNEL);
		if (!tmp)
			return -ENOMEM;
		/* now allocate the data blocks we'll need */
		retval = pram_alloc_blocks(inode, blocknr, num_blocks);
		if (retval)
			goto fail;
	}
	
	pi = pram_get_inode(inode->i_sb, inode->i_ino);
	
	while (length) {
		int count;
		pram_off_t block = pram_find_data_block(inode, blocknr++);
		u8* bp = (u8*)pram_get_block(sb, block);
		if (!bp)
			goto fail;
		
		count = blockoff + length > blocksize ?
			blocksize - blockoff : length;
		
		if (rw == READ) {
			copy_to_user(buf, &bp[blockoff], count);
		} else {
			copy_from_user(tmp, buf, count);

			pram_lock_block(inode->i_sb, bp);
			memcpy(&bp[blockoff], tmp, count);
			pram_unlock_block(inode->i_sb, bp);
		}

		progress += count;
		buf += count;
		length -= count;
		blockoff = 0;
	}
	
	retval = progress;
 fail:
	if (tmp)
		kfree(tmp);
	return retval;
}


static ssize_t pram_file_direct_IO(int rw, struct file * filp,
				    char * buf, size_t count, loff_t offset)
{
	ssize_t retval;
	int iosize, progress;
	struct inode * inode = filp->f_dentry->d_inode;
	loff_t size = inode->i_size;

	if ((rw == READ) && (offset + count > size))
		count = size - offset;

	progress = retval = 0;
	while (count > 0) {
		iosize = count;

		retval = pram_direct_IO(rw, inode, buf,
					 offset+progress,
					 iosize);

		if (retval >= 0) {
			count -= retval;
			buf += retval;
			progress += retval;
		}

		if (retval != iosize)
			break;
	}

	if (progress)
		retval = progress;

	return retval;
}

static ssize_t pram_file_read(struct file * filp, char * buf,
			       size_t count, loff_t *ppos)
{
	loff_t pos = *ppos, size;
	struct inode *inode = filp->f_dentry->d_inode;
	ssize_t retval;

	if ((ssize_t) count < 0)
		return -EINVAL;

	retval = 0;
	if (!count)
		goto out; /* skip atime */
	size = inode->i_size;
	if (pos < size) {
		retval = pram_file_direct_IO(READ, filp, buf, count, pos);
		if (retval > 0)
			*ppos = pos + retval;
	}
	UPDATE_ATIME(filp->f_dentry->d_inode);
 out:
	return retval;
}


static ssize_t pram_file_write(struct file *file, const char *buf,
				size_t count, loff_t *ppos)
{
	struct inode	*inode = file->f_dentry->d_inode;
	unsigned long	limit = current->rlim[RLIMIT_FSIZE].rlim_cur;
	loff_t		pos;
	ssize_t		written;
	int		err;

	if ((ssize_t) count < 0)
		return -EINVAL;

	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	down(&inode->i_sem);

	pos = *ppos;
	err = -EINVAL;
	if (pos < 0)
		goto out;

	err = file->f_error;
	if (err) {
		file->f_error = 0;
		goto out;
	}

	written = 0;

	/* FIXME: this is for backwards compatibility with 2.4 */
	if (!S_ISBLK(inode->i_mode) && file->f_flags & O_APPEND)
		pos = inode->i_size;

	/*
	 * Check whether we've reached the file size limit.
	 */
	err = -EFBIG;
	
	if (!S_ISBLK(inode->i_mode) && limit != RLIM_INFINITY) {
		if (pos >= limit) {
			send_sig(SIGXFSZ, current, 0);
			goto out;
		}
		if (pos > 0xFFFFFFFFULL || count > limit - (u32)pos) {
			/* send_sig(SIGXFSZ, current, 0); */
			count = limit - (u32)pos;
		}
	}

	/*
	 *	LFS rule 
	 */
	if ( pos + count > MAX_NON_LFS && !(file->f_flags&O_LARGEFILE)) {
		if (pos >= MAX_NON_LFS) {
			send_sig(SIGXFSZ, current, 0);
			goto out;
		}
		if (count > MAX_NON_LFS - (u32)pos) {
			/* send_sig(SIGXFSZ, current, 0); */
			count = MAX_NON_LFS - (u32)pos;
		}
	}

	/*
	 *	Are we about to exceed the fs block limit ?
	 *
	 *	If we have written data it becomes a short write
	 *	If we have exceeded without writing data we send
	 *	a signal and give them an EFBIG.
	 *
	 *	Linus frestrict idea will clean these up nicely..
	 */
	 
	if (!S_ISBLK(inode->i_mode)) {
		if (pos >= inode->i_sb->s_maxbytes)
		{
			if (count || pos > inode->i_sb->s_maxbytes) {
				send_sig(SIGXFSZ, current, 0);
				err = -EFBIG;
				goto out;
			}
			/* zero-length writes at ->s_maxbytes are OK */
		}

		if (pos + count > inode->i_sb->s_maxbytes)
			count = inode->i_sb->s_maxbytes - pos;
	} else {
		if (is_read_only(inode->i_rdev)) {
			err = -EPERM;
			goto out;
		}
		if (pos >= inode->i_size) {
			if (count || pos > inode->i_size) {
				err = -ENOSPC;
				goto out;
			}
		}

		if (pos + count > inode->i_size)
			count = inode->i_size - pos;
	}

	err = 0;
	if (count == 0)
		goto out;

	remove_suid(inode);
	inode->i_ctime = inode->i_mtime = CURRENT_TIME;
	pram_write_inode(inode, 0);

	written = pram_file_direct_IO(WRITE, file, (char *)buf, count, pos);
	if (written > 0) {
		loff_t end = pos + written;
		if (end > inode->i_size && !S_ISBLK(inode->i_mode)) {
			inode->i_size = end;
			pram_write_inode(inode, 0);
		}
		*ppos = end;
	}

	err = written ? written : 0;
out:
	up(&inode->i_sem);
	return err;
}


int pram_file_mmap(struct file * file, struct vm_area_struct * vma)
{
	if (vma->vm_flags & VM_EXEC)
		return -ENOEXEC;
	return generic_file_mmap(file, vma);
}

struct file_operations pram_file_operations = {
	llseek:		generic_file_llseek, // ok
	read:		pram_file_read,
	write:		pram_file_write,
	ioctl:		pram_ioctl,
	mmap:		pram_file_mmap,
	open:		generic_file_open,   // ok
	release:	pram_release_file,
	fsync:		pram_sync_file,
};

struct inode_operations pram_file_inode_operations = {
	truncate:	pram_truncate,
};
