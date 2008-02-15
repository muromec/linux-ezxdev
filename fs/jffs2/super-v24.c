/*
 * JFFS2 -- Journalling Flash File System, Version 2.
 *
 * Copyright (C) 2001-2003 Red Hat, Inc.
 * Copyright (c) 2005 Motorola Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Created by David Woodhouse <dwmw2@infradead.org>
 * Added summary patch by Jiang Jun <a21079@motorola.com> 2005/04/13
 * Modified by Ru Yi <e5537c@motorola.com>  2005/10/11
 *     Mount record interface added on umount time
 *
 * For licensing information, see the file 'LICENCE' in this directory.
 *
 * $Id: super-v24.c,v 1.83 2005/02/09 09:23:54 pavlov Exp $
 *
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/jffs2.h>
#include <linux/pagemap.h>
#include <linux/mtd/mtd.h>
#include "compr.h"
#include "nodelist.h"
#include "summary.h"

#ifdef CONFIG_MOUNT_RECORD
#include <linux/mnt_record.h>
#endif

#ifndef MTD_BLOCK_MAJOR
#define MTD_BLOCK_MAJOR 31
#endif

static void jffs2_put_super (struct super_block *);

static struct super_operations jffs2_super_operations =
{
	.read_inode =	jffs2_read_inode,
	.put_super =	jffs2_put_super,
	.write_super =	jffs2_write_super,
	.statfs =	jffs2_statfs,
	.remount_fs =	jffs2_remount_fs,
	.clear_inode =	jffs2_clear_inode,
	.dirty_inode =	jffs2_dirty_inode,
};


static struct super_block *jffs2_read_super(struct super_block *sb, void *data, int silent)
{
	struct jffs2_sb_info *c;
	int ret;

	D1(printk(KERN_DEBUG "jffs2: read_super for device %s\n", kdevname(sb->s_dev)));

	if (major(sb->s_dev) != MTD_BLOCK_MAJOR) {
		if (!silent)
			printk(KERN_DEBUG "jffs2: attempt to mount non-MTD device %s\n", kdevname(sb->s_dev));
		return NULL;
	}

	c = JFFS2_SB_INFO(sb);
	memset(c, 0, sizeof(*c));

	sb->s_op = &jffs2_super_operations;

	c->mtd = get_mtd_device(NULL, minor(sb->s_dev));
	if (!c->mtd) {
		D1(printk(KERN_DEBUG "jffs2: MTD device #%u doesn't appear to exist\n", minor(sb->s_dev)));
		return NULL;
	}
#ifdef CONFIG_MOUNT_RECORD
	printk("mnt_record_enter begin\n");
	c->jffs2_need_checking = mnt_record_enter(c->mtd, sb->s_dev);
	printk("mnt_record_enter end (check %d)\n", c->jffs2_need_checking);
#endif

	ret = jffs2_do_fill_super(sb, data, silent);
	if (ret) {
		put_mtd_device(c->mtd);
		return NULL;
	}
#ifdef CONFIG_JFFS2_SUMMARY
        jffs2_sum_init(c);
#endif
	return sb;
}

static void jffs2_put_super (struct super_block *sb)
{
	struct jffs2_sb_info *c = JFFS2_SB_INFO(sb);

	D2(printk(KERN_DEBUG "jffs2: jffs2_put_super()\n"));


	if (!(sb->s_flags & MS_RDONLY))
		jffs2_stop_garbage_collect_thread(c);
	down(&c->alloc_sem);
	jffs2_flush_wbuf_pad(c);
	up(&c->alloc_sem);
#ifdef CONFIG_JFFS2_SUMMARY
        jffs2_sum_clean_all_info(c);
        jffs2_sum_exit(c);
#endif
	jffs2_free_ino_caches(c);
	jffs2_free_raw_node_refs(c);
	if (c->mtd->flags & MTD_NO_VIRTBLOCKS)
		vfree(c->blocks);
	else
		kfree(c->blocks);
	jffs2_flash_cleanup(c);
	kfree(c->inocache_list);
	if (c->mtd->sync)
		c->mtd->sync(c->mtd);
	put_mtd_device(c->mtd);
	
#ifdef CONFIG_MOUNT_RECORD
	printk("mnt_record_leave begin\n");
	mnt_record_leave(sb->s_dev);
	printk("mnt_record_leave end\n");
#endif
	D1(printk(KERN_DEBUG "jffs2_put_super returning\n"));
}

static DECLARE_FSTYPE_DEV(jffs2_fs_type, "jffs2", jffs2_read_super);

static int __init init_jffs2_fs(void)
{
	int ret;

	printk(KERN_INFO "JFFS2 version 2.2."
#ifdef CONFIG_JFFS2_FS_WRITEBUFFER
	       " (NAND)"
#endif
#ifdef CONFIG_JFFS2_SUMMARY
	       " (SUMMARY)"
#endif
	       " (C) 2001-2003 Red Hat, Inc.\n");

#ifdef JFFS2_OUT_OF_KERNEL
	/* sanity checks. Could we do these at compile time? */
	if (sizeof(struct jffs2_sb_info) > sizeof (((struct super_block *)NULL)->u)) {
		printk(KERN_ERR "JFFS2 error: struct jffs2_sb_info (%d bytes) doesn't fit in the super_block union (%d bytes)\n", 
		       sizeof(struct jffs2_sb_info), sizeof (((struct super_block *)NULL)->u));
		return -EIO;
	}

	if (sizeof(struct jffs2_inode_info) > sizeof (((struct inode *)NULL)->u)) {
		printk(KERN_ERR "JFFS2 error: struct jffs2_inode_info (%d bytes) doesn't fit in the inode union (%d bytes)\n", 
		       sizeof(struct jffs2_inode_info), sizeof (((struct inode *)NULL)->u));
		return -EIO;
	}
#endif
	ret = jffs2_compressors_init();
	if (ret) {
		printk(KERN_ERR "JFFS2 error: Failed to initialise compressors\n");
		goto out;
	}
	ret = jffs2_create_slab_caches();
	if (ret) {
		printk(KERN_ERR "JFFS2 error: Failed to initialise slab caches\n");
		goto out_compressors;
	}
	ret = register_filesystem(&jffs2_fs_type);
	if (ret) {
		printk(KERN_ERR "JFFS2 error: Failed to register filesystem\n");
		goto out_slab;
	}
	return 0;

 out_slab:
	jffs2_destroy_slab_caches();
 out_compressors:
	jffs2_compressors_exit();
 out:
	return ret;
}

static void __exit exit_jffs2_fs(void)
{
	jffs2_destroy_slab_caches();
	jffs2_compressors_exit();
	unregister_filesystem(&jffs2_fs_type);
}

module_init(init_jffs2_fs);
module_exit(exit_jffs2_fs);

MODULE_DESCRIPTION("The Journalling Flash File System, v2");
MODULE_AUTHOR("Red Hat, Inc.");
MODULE_LICENSE("GPL"); // Actually dual-licensed, but it doesn't matter for 
		       // the sake of this tag. It's Free Software.
