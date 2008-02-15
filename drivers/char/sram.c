/*
 * linux/drivers/char/sram.c
 *
 * Bulverde Internal Memory character device driver
 *
 * Created:	  Sep 05, 2003
 * Copyright:	  MontaVista Software Inc.
 * Last modified: Fri Jul 16 2004
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/config.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/init.h>
#include <linux/rwsem.h>
#include <linux/devfs_fs_kernel.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/pgalloc.h>

/* Default size, in bytes */
#define SRAM_SIZE_DEFAULT	1024

/* ioctl type */
#define SRAM_IOCTL_TYPE	'S'

/* Read size */
#define SRAM_IOCTL_SIZE_GET	_IOR(SRAM_IOCTL_TYPE, 0, unsigned long)
/* Write size. All data will be lost */
#define SRAM_IOCTL_SIZE_SET	_IOW(SRAM_IOCTL_TYPE, 1, unsigned long)

extern unsigned long sram_alloc(unsigned long);
extern void sram_free(unsigned long);

typedef struct sram_data {
	unsigned long start;
	unsigned long size;
} sram_data;

static unsigned long sram_size_default = SRAM_SIZE_DEFAULT;

/*
 * This funcion reads the SRAM memory.
 * The f_pos points to the SRAM memory location. 
 */
static ssize_t 
sram_read(struct file * file, char * buf,
	  size_t count, loff_t *ppos) 
{
	unsigned long p = *ppos;
	sram_data *data = (sram_data *)file->private_data;

	if (p >= data->size)
		return -EINVAL;

	if (count > data->size - p)
		count = data->size - p;

	if (copy_to_user(buf, (char *)data->start + p, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static ssize_t 
sram_write(struct file * file, const char * buf, 
	   size_t count, loff_t *ppos) 
{
	unsigned long p = *ppos;
	sram_data *data = (sram_data *)file->private_data;

	if (p >= data->size)
		return -EINVAL;

	if (count > data->size - p)
		count = data->size - p;

	if (copy_from_user((char *)data->start + p, buf, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static int 
sram_mmap(struct file * file, struct vm_area_struct * vma) 
{
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	sram_data *data = (sram_data *)file->private_data;

	if (((file->f_flags & O_WRONLY) != 0) ||
	    ((file->f_flags & O_RDWR) != 0))
		vma->vm_page_prot = (pgprot_t)PAGE_SHARED;
	else
		vma->vm_page_prot = (pgprot_t)PAGE_READONLY;
	
	/* Do not cache SRAM memory if O_SYNC flag is set */
	if ((file->f_flags & O_SYNC) != 0)
		pgprot_val(vma->vm_page_prot) &= ~L_PTE_CACHEABLE;

	/* Don't try to swap out physical pages.. */
	vma->vm_flags |= VM_RESERVED | VM_LOCKED | VM_IO;

	return remap_page_range(vma->vm_start, io_v2p(data->start),
				vma->vm_end - vma->vm_start, vma->vm_page_prot);
}


static loff_t 
sram_lseek(struct file * file, loff_t offset, int orig) 
{
	sram_data *data = (sram_data *)file->private_data;

	switch (orig) {
	case 0:
		file->f_pos = offset;
		break;
	case 1:
		file->f_pos += offset;
		break;
	case 2:
		file->f_pos = data->size + offset;
		break;
	default:
		return -EINVAL;
	}
	
	if (file->f_pos < 0) {
		file->f_pos = 0;
		return -EINVAL;
	} else if (file->f_pos > data->size) {
		file->f_pos = data->size;
		return -EINVAL;
	}
	return file->f_pos;
}

static int
sram_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
	   unsigned long arg)
{
	sram_data *data = (sram_data *)file->private_data;
	int newsize;

	if (_IOC_TYPE(cmd) != SRAM_IOCTL_TYPE)
		return -EINVAL;

	switch (cmd)
	{
		case SRAM_IOCTL_SIZE_GET:
			if (copy_to_user((void *)arg, &data->size, 
					 _IOC_SIZE(cmd)))
				return -EFAULT;
			break;

		case SRAM_IOCTL_SIZE_SET:
			if (copy_from_user(&newsize, (void *)arg, 
					   _IOC_SIZE(cmd)))
				return -EFAULT;
			if (newsize == data->size)
				return 0;
			data->size = newsize;
			if (data->start != 0)
				sram_free(data->start);
			data->start = sram_alloc(data->size);
			if (data->start == 0)
				return -ENOMEM;
			break;

		default:
			return -EINVAL;
	}
	return 0;
}

static int 
sram_open(struct inode *inode, struct file *file) 
{
	sram_data *data = kmalloc(sizeof(sram_data), GFP_KERNEL);

	if (!data)
		return -ENOMEM;
	file->private_data = data;
	data->size = sram_size_default;
	data->start = sram_alloc(data->size);
	return data->start ? 0 : -EINVAL;
}

static int 
sram_release(struct inode * inode, struct file * file) 
{
	sram_data *data = (sram_data *)file->private_data;
	if (data) {
		sram_free(data->start);
		kfree(data);
	}
	return 0;
}

static struct file_operations sram_fops = {
	llseek:		sram_lseek,
	read:		sram_read,
	write:		sram_write,
	mmap:		sram_mmap,
	ioctl:		sram_ioctl,
	open:		sram_open,
	release:	sram_release,
};

static struct miscdevice sram_misc = {
	minor : MISC_DYNAMIC_MINOR,
	name  : "misc/sram",
	fops  : &sram_fops,
};

static int __init sram_chr_init(void) {
	int ret = misc_register(&sram_misc);
	
	if (ret)
		printk(KERN_ERR "Cannot register SRAM device /dev/%s\n",
		       sram_misc.name);
	return ret;
}

static void __exit sram_chr_exit(void) {
	misc_deregister(&sram_misc);
}

module_init(sram_chr_init)
module_exit(sram_chr_exit)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("MontaVista Software Inc.");
MODULE_PARM(sram_size_default, "l");
MODULE_PARM_DESC(sram_size_default, "size of SRAM memory to be allocated");
