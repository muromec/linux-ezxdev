/*
    spi-dev.c - spi-bus driver, char device interface  

    Copyright (C) 1995-97 Simon G. Vogl
    Copyright (C) 1998-99 Frodo Looijaard <frodol@dds.nl>
    Copyright (C) 2002 Compaq Computer Corporation

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

/* Adapated from i2c-dev module by Jamey Hicks <jamey.hicks@compaq.com> */

/* Note that this is a complete rewrite of Simon Vogl's i2c-dev module.
   But I have used so much of his original code and ideas that it seems
   only fair to recognize him as co-author -- Frodo */

/* The devfs code is contributed by Philipp Matthias Hahn 
   <pmhahn@titan.lahn.de> */

/* $Id: spi-dev.c,v 1.1 2002/10/03 05:21:14 gdavis Exp $ */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/version.h>
#if LINUX_KERNEL_VERSION >= KERNEL_VERSION(2,4,0)
#include <linux/smp_lock.h>
#endif /* LINUX_KERNEL_VERSION >= KERNEL_VERSION(2,4,0) */
#ifdef CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
#endif


/* If you want debugging uncomment: */
/* #define DEBUG */

#include <linux/init.h>
#include <asm/uaccess.h>

#include <linux/spi/spi.h>

/* struct file_operations changed too often in the 2.1 series for nice code */

#if LINUX_KERNEL_VERSION < KERNEL_VERSION(2,4,9)
static loff_t spidev_lseek (struct file *file, loff_t offset, int origin);
#endif
static ssize_t spidev_read (struct file *file, char *buf, size_t count, 
                            loff_t *offset);
static ssize_t spidev_write (struct file *file, const char *buf, size_t count, 
                             loff_t *offset);

static int spidev_open (struct inode *inode, struct file *file);

static int spidev_release (struct inode *inode, struct file *file);

static int spidev_attach_adapter(struct spi_adapter *adap);
static int spidev_detach_client(struct spi_client *client);
static int spidev_command(struct spi_client *client, unsigned int cmd,
                           void *arg);

static int __init spi_dev_init(void);
static void spidev_cleanup(void);

static struct file_operations spidev_fops = {
#if LINUX_KERNEL_VERSION >= KERNEL_VERSION(2,4,0)
	owner:		THIS_MODULE,
#endif /* LINUX_KERNEL_VERSION >= KERNEL_VERSION(2,4,0) */
#if LINUX_KERNEL_VERSION < KERNEL_VERSION(2,4,9)
	llseek:		spidev_lseek,
#else
	llseek:		no_llseek,
#endif
	read:		spidev_read,
	write:		spidev_write,
	open:		spidev_open,
	release:	spidev_release,
};

#define SPIDEV_ADAPS_MAX SPI_ADAP_MAX
static struct spi_adapter *spidev_adaps[SPIDEV_ADAPS_MAX];
#ifdef CONFIG_DEVFS_FS
static devfs_handle_t devfs_spi[SPIDEV_ADAPS_MAX];
static devfs_handle_t devfs_handle = NULL;
#endif

static struct spi_driver spidev_driver = {
	name:		"spi-dev dummy driver",
	id:		SPI_DRIVERID_SPIDEV,
	flags:		SPI_DF_DUMMY,
	attach_adapter:	spidev_attach_adapter,
	detach_client:	spidev_detach_client,
	command:	spidev_command,
/*	inc_use:	NULL,
	dec_use:	NULL, */
};

static struct spi_client spidev_client_template = {
	name:		"SPI /dev entry",
	id:		1,
	flags:		0,
	addr:		-1,
/*	adapter:	NULL, */
	driver:		&spidev_driver,
/*	data:		NULL */
};

static int spidev_initialized;

#if LINUX_KERNEL_VERSION < KERNEL_VERSION(2,4,9)
/* Note that the lseek function is called llseek in 2.1 kernels. But things
   are complicated enough as is. */
loff_t spidev_lseek (struct file *file, loff_t offset, int origin)
{
#ifdef DEBUG
	struct inode *inode = file->f_dentry->d_inode;
	printk("spi-dev.o: spi-%d lseek to %ld bytes relative to %d.\n",
	       MINOR(inode->i_rdev),(long) offset,origin);
#endif /* DEBUG */
	return -ESPIPE;
}
#endif

static ssize_t spidev_read (struct file *file, char *buf, size_t count,
                            loff_t *offset)
{
	char *tmp;
	int ret;

#ifdef DEBUG
	struct inode *inode = file->f_dentry->d_inode;
#endif /* DEBUG */

	struct spi_client *client = (struct spi_client *)file->private_data;

	/* copy user space data to kernel space. */
	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;

#ifdef DEBUG
	printk("spi-dev.o: spi-%d reading %d bytes.\n",MINOR(inode->i_rdev),
	       count);
#endif

	ret = spi_read(client,tmp,count);
	if (ret >= 0)
		ret = copy_to_user(buf,tmp,count)?-EFAULT:ret;
	kfree(tmp);
	return ret;
}

static ssize_t spidev_write (struct file *file, const char *buf, size_t count,
                             loff_t *offset)
{
	int ret;
	char *tmp;
	struct spi_client *client = (struct spi_client *)file->private_data;

#ifdef DEBUG
	struct inode *inode = file->f_dentry->d_inode;
#endif /* DEBUG */

	/* copy user space data to kernel space. */
	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
	if (copy_from_user(tmp,buf,count)) {
		kfree(tmp);
		return -EFAULT;
	}

#ifdef DEBUG
	printk("spi-dev.o: spi-%d writing %d bytes.\n",MINOR(inode->i_rdev),
	       count);
#endif
	ret = spi_write(client,tmp,count);
	kfree(tmp);
	return ret;
}

int spidev_open (struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	struct spi_client *client;

	if ((minor >= SPIDEV_ADAPS_MAX) || ! (spidev_adaps[minor])) {
#ifdef DEBUG
		printk("spi-dev.o: Trying to open unattached adapter spi-%d\n",
		       minor);
#endif
		return -ENODEV;
	}

	/* Note that we here allocate a client for later use, but we will *not*
	   register this client! Yes, this is safe. No, it is not very clean. */
	if(! (client = kmalloc(sizeof(struct spi_client),GFP_KERNEL)))
		return -ENOMEM;
	memcpy(client,&spidev_client_template,sizeof(struct spi_client));
	client->adapter = spidev_adaps[minor];
	file->private_data = client;

	if (spidev_adaps[minor]->inc_use)
		spidev_adaps[minor]->inc_use(spidev_adaps[minor]);
#if LINUX_KERNEL_VERSION < KERNEL_VERSION(2,4,0)
	MOD_INC_USE_COUNT;
#endif /* LINUX_KERNEL_VERSION < KERNEL_VERSION(2,4,0) */

#ifdef DEBUG
	printk("spi-dev.o: opened spi-%d\n",minor);
#endif
	return 0;
}

static int spidev_release (struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	kfree(file->private_data);
	file->private_data=NULL;
#ifdef DEBUG
	printk("spi-dev.o: Closed: spi-%d\n", minor);
#endif
#if LINUX_KERNEL_VERSION < KERNEL_VERSION(2,4,0)
	MOD_DEC_USE_COUNT;
#else /* LINUX_KERNEL_VERSION >= KERNEL_VERSION(2,4,0) */
	lock_kernel();
#endif /* LINUX_KERNEL_VERSION < KERNEL_VERSION(2,4,0) */
	if (spidev_adaps[minor]->dec_use)
		spidev_adaps[minor]->dec_use(spidev_adaps[minor]);
#if LINUX_KERNEL_VERSION >= KERNEL_VERSION(2,4,0)
	unlock_kernel();
#endif /* LINUX_KERNEL_VERSION >= KERNEL_VERSION(2,4,0) */
	return 0;
}

int spidev_attach_adapter(struct spi_adapter *adap)
{
	int i;
	char name[8];

	if ((i = spi_adapter_id(adap)) < 0) {
		printk("spi-dev.o: Unknown adapter ?!?\n");
		return -ENODEV;
	}
	if (i >= SPIDEV_ADAPS_MAX) {
		printk("spi-dev.o: Adapter number too large?!? (%d)\n",i);
		return -ENODEV;
	}

	sprintf (name, "%d", i);
	if (! spidev_adaps[i]) {
		spidev_adaps[i] = adap;
#ifdef CONFIG_DEVFS_FS
		devfs_spi[i] = devfs_register (devfs_handle, name,
			DEVFS_FL_DEFAULT, SPI_MAJOR, i,
			S_IFCHR | S_IRUSR | S_IWUSR,
			&spidev_fops, NULL);
#endif
		printk("spi-dev.o: Registered '%s' as minor %d\n",adap->name,i);
	} else {
		/* This is actually a detach_adapter call! */
#ifdef CONFIG_DEVFS_FS
		devfs_unregister(devfs_spi[i]);
#endif
		spidev_adaps[i] = NULL;
#ifdef DEBUG
		printk("spi-dev.o: Adapter unregistered: %s\n",adap->name);
#endif
	}

	return 0;
}

int spidev_detach_client(struct spi_client *client)
{
	return 0;
}

static int spidev_command(struct spi_client *client, unsigned int cmd,
                           void *arg)
{
	return -1;
}

static int __init spi_dev_init(void)
{
	int res;

	printk(KERN_DEBUG "spi-dev.o: spi /dev entries driver module\n");

	spidev_initialized = 0;
#ifdef CONFIG_DEVFS_FS
	if (devfs_register_chrdev(SPI_MAJOR, "spi", &spidev_fops)) {
#else
	if (register_chrdev(SPI_MAJOR,"spi",&spidev_fops)) {
#endif
		printk("spi-dev.o: unable to get major %d for spi bus\n",
		       SPI_MAJOR);
		return -EIO;
	}
#ifdef CONFIG_DEVFS_FS
	devfs_handle = devfs_mk_dir(NULL, "spi", NULL);
#endif
	spidev_initialized ++;

	if ((res = spi_add_driver(&spidev_driver))) {
		printk("spi-dev.o: Driver registration failed, module not inserted.\n");
		spidev_cleanup();
		return res;
	}
	spidev_initialized ++;
	return 0;
}

static void spidev_cleanup(void)
{
	int res;

	if (spidev_initialized >= 2) {
		if ((res = spi_del_driver(&spidev_driver))) {
			printk("spi-dev.o: Driver deregistration failed, "
			       "module not removed.\n");
			return;
		}
		spidev_initialized --;
	}

	if (spidev_initialized >= 1) {
#ifdef CONFIG_DEVFS_FS
		devfs_unregister(devfs_handle);
		if ((res = devfs_unregister_chrdev(SPI_MAJOR, "spi"))) {
#else
		if ((res = unregister_chrdev(SPI_MAJOR,"spi"))) {
#endif
			printk("spi-dev.o: unable to release major %d for spi bus\n",
			       SPI_MAJOR);
			return;
		}
		spidev_initialized --;
	}
}

EXPORT_NO_SYMBOLS;

MODULE_AUTHOR("Jamey Hicks <jamey.hicks@compaq.com> Frodo Looijaard <frodol@dds.nl> and Simon G. Vogl <simon@tk.uni-linz.ac.at>");
MODULE_DESCRIPTION("SPI /dev entries driver");
MODULE_LICENSE("GPL");

module_init(spi_dev_init);
module_exit(spidev_cleanup);

