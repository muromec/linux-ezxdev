/*
 * pram_fs.h
 *
 * Definitions for the PRAMFS filesystem.
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
#ifndef _LINUX_PRAM_FS_H
#define _LINUX_PRAM_FS_H

#include <linux/types.h>

#ifdef __KERNEL__
#include <linux/sched.h>
#endif

/*
 * The PRAM filesystem constants/structures
 */

/*
 * Define PRAMFS_DEBUG to produce debug messages
 */
#undef PRAMFS_DEBUG

/*
 * The PRAM filesystem version
 */
#define PRAMFS_DATE		"2003/06/15"
#define PRAMFS_VERSION		"0.1b"

/*
 * Debug code
 */
#ifdef __KERNEL__
#define PFX "pramfs"
#ifdef PRAMFS_DEBUG
#define pram_dbg(format, arg...) \
    printk(KERN_DEBUG PFX ": " format , ## arg)
#else
#define pram_dbg(format, arg...) do {} while (0)
#endif
#define pram_err(format, arg...) \
    printk(KERN_ERR PFX ": " format , ## arg)
#define pram_info(format, arg...) \
    printk(KERN_INFO PFX ": " format , ## arg)
#define pram_warn(format, arg...) \
    printk(KERN_WARNING PFX ": " format , ## arg)
#endif

/*
 * The PRAM file system magic number
 */
#define PRAM_SUPER_MAGIC	0xEFFA

/*
 * Maximal count of links to a file
 */
#define PRAM_LINK_MAX		32000

typedef unsigned long pram_off_t;

#define PRAM_MIN_BLOCK_SIZE 512
#define	PRAM_MAX_BLOCK_SIZE 4096
#define PRAM_DEF_BLOCK_SIZE 2048

#define PRAM_INODE_SIZE 128 // must be power of two
#define PRAM_INODE_BITS   7

/*
 * Structure of a directory entry in PRAMFS.
 * Offsets are to the inode that holds the referenced dentry.
 */
struct pram_dentry {
	pram_off_t d_next;     /* next dentry in this directory */
	pram_off_t d_prev;     /* previous dentry in this directory */
	pram_off_t d_parent;   /* parent directory */
	char       d_name[0];
};


/*
 * Structure of an inode in PRAMFS
 */
struct pram_inode {
	__u32   i_sum;          /* checksum of this inode */
	__u32	i_uid;		/* Owner Uid */
	__u32	i_gid;		/* Group Id */
	__u16	i_mode;		/* File mode */
	__u16	i_links_count;	/* Links count */
	__u32	i_blocks;	/* Blocks count */
	__u32	i_size;		/* Size of data in bytes */
	__u32	i_atime;	/* Access time */
	__u32	i_ctime;	/* Creation time */
	__u32	i_mtime;	/* Modification time */
	__u32	i_dtime;	/* Deletion Time */

	union {
		struct {
			/*
			 * ptr to row block of 2D block pointer array,
			 * file block #'s 0 to (blocksize/4)^2 - 1.
			 */
			pram_off_t row_block;
		} reg;   // regular file or symlink inode
		struct {
			pram_off_t head; /* first entry in this directory */
			pram_off_t tail; /* last entry in this directory */
		} dir;
		struct {
			__u32 rdev; /* major/minor # */
		} dev;   // device inode
	} i_type;

	struct pram_dentry i_d;
};

#define PRAM_NAME_LEN \
    (PRAM_INODE_SIZE - offsetof(struct pram_inode, i_d.d_name) - 1)


/*
 * Behaviour when detecting errors
 */
#define PRAM_ERRORS_CONTINUE		1	/* Continue execution */
#define PRAM_ERRORS_RO			2	/* Remount fs read-only */
#define PRAM_ERRORS_PANIC		3	/* Panic */
#define PRAM_ERRORS_DEFAULT		PRAM_ERRORS_CONTINUE

#define PRAM_SB_SIZE 128 // must be power of two
#define PRAM_SB_BITS   7

/*
 * Structure of the super block in PRAMFS
 */
struct pram_super_block {
	__u32   s_size;         /* total size of fs in bytes */
	__u32   s_blocksize;    /* blocksize in bytes */
	__u32   s_features;     /* feature flags */
	__u32	s_inodes_count;	/* total inodes count (used or free) */
	__u32	s_free_inodes_count;/* free inodes count */
	__u32   s_free_inode_hint;  /* start hint for locating free inodes */
	__u32	s_blocks_count;	/* total data blocks count (used or free) */
	__u32	s_free_blocks_count;/* free data blocks count */
	__u32	s_free_blocknr_hint;/* free data blocks count */
	pram_off_t s_bitmap_start; /* data block in-use bitmap location */
	__u32   s_bitmap_blocks;/* size of bitmap in number of blocks */
	__u32	s_mtime;	/* Mount time */
	__u32	s_wtime;	/* Write time */
	__u32	s_rev_level;	/* Revision level */
	__u16	s_magic;	/* Magic signature */
	__u16	s_state;	/* File system state */
	__u16	s_errors;	/* Behaviour when detecting errors */
	char	s_volume_name[16]; /* volume name */
	__u32   s_sum;          /* checksum of this sb, including padding */
};

/* The root inode follows immediately after the super block */
#define PRAM_ROOT_INO PRAM_SB_SIZE

#ifdef __KERNEL__

/* Function Prototypes */

/* balloc.c */
extern void pram_init_bitmap(struct super_block * sb);
extern void pram_free_block (struct super_block * sb, int blocknr);
extern int pram_new_block (struct super_block * sb, int* blocknr, int zero);
extern unsigned long pram_count_free_blocks (struct super_block * sb);

/* dir.c */
extern int pram_add_link (struct dentry * dentry, struct inode *inode);
extern int pram_remove_link(struct inode * inode);

/* fsync.c */
extern int pram_sync_file (struct file *, struct dentry *, int);
extern int pram_fsync_inode (struct inode *, int);

/* inode.c */
extern int pram_alloc_blocks(struct inode * inode, int file_blocknr, int num);
extern pram_off_t pram_find_data_block(struct inode * inode,
					 int file_blocknr);
extern struct inode * pram_fill_new_inode(struct super_block *sb,
					   struct pram_inode * raw_inode);
extern void pram_put_inode (struct inode * inode);
extern void pram_delete_inode (struct inode * inode);
extern struct inode * pram_new_inode (const struct inode * dir, int mode);
extern void pram_read_inode (struct inode * inode);
extern void pram_truncate (struct inode * inode);
extern void pram_write_inode (struct inode * inode, int wait);
extern void pram_dirty_inode(struct inode * inode);

/* ioctl.c */
extern int pram_ioctl (struct inode *, struct file *, unsigned int,
			unsigned long);

/* super.c */
extern struct super_block * find_pramfs_super(void);
extern struct super_block * pram_read_super (struct super_block * sb,
					      void * data,
					      int silent);
extern int pram_statfs (struct super_block * sb, struct statfs * buf);
extern int pram_remount (struct super_block * sb, int * flags, char * data);

/* symlink.c */
extern int pram_block_symlink(struct inode *inode,
			       const char *symname, int len);

/* wprotect.c */
#ifndef CONFIG_PRAMFS_NOWP
extern void pram_writeable (void * vaddr, unsigned long size, int rw);
#endif

/* Inline functions start here */

static inline u32 pram_calc_checksum(u32* buf, int n)
{
	u32 sum = 0;
	while (n--)
		sum += *buf++;
	return sum;
}

// If this is part of a read-modify-write of the super block,
// pram_lock_super() before calling!
static inline struct pram_super_block *
pram_get_super(struct super_block * sb)
{
	return (struct pram_super_block *)sb->u.pram_sb.virt_addr;
}

// pram_lock_super() before calling!
static inline void pram_sync_super(struct pram_super_block * ps)
{
	ps->s_wtime = CURRENT_TIME;
	ps->s_sum = 0;
	ps->s_sum = - pram_calc_checksum((u32*)ps, PRAM_SB_SIZE>>2);
}

// pram_lock_inode() before calling!
static inline void pram_sync_inode(struct pram_inode * pi)
{
	//pi->i_mtime = CURRENT_TIME;
	pi->i_sum = 0;
	pi->i_sum = - pram_calc_checksum((u32*)pi, PRAM_INODE_SIZE>>2);
}

#ifndef CONFIG_PRAMFS_NOWP
#define pram_lock_range(p, len) {\
	spin_lock_irqsave(&init_mm.page_table_lock, flags);\
	pram_writeable((p), (len), 1);\
}

#define pram_unlock_range(p, len) {\
	pram_writeable((p), (len), 0);\
	spin_unlock_irqrestore(&init_mm.page_table_lock, flags);\
}
#else
#define pram_lock_range(p, len) do {} while (0)
#define pram_unlock_range(p, len) do {} while (0)
#endif

// write protection for super block
#define pram_lock_super(ps) \
        pram_lock_range((ps), PRAM_SB_SIZE)
#define pram_unlock_super(ps) {\
	pram_sync_super(ps);\
        pram_unlock_range((ps), PRAM_SB_SIZE);\
}

// write protection for inode metadata
#define pram_lock_inode(pi) \
        pram_lock_range((pi), PRAM_INODE_SIZE)
#define pram_unlock_inode(pi) {\
	pram_sync_inode(pi);\
        pram_unlock_range((pi), PRAM_SB_SIZE);\
}

// write protection for a data block
#define pram_lock_block(sb, bp) \
        pram_lock_range((bp), (sb)->s_blocksize)
#define pram_unlock_block(sb, bp) \
        pram_unlock_range((bp), (sb)->s_blocksize)

static inline void *
pram_get_bitmap(struct super_block * sb)
{
	struct pram_super_block * ps = pram_get_super(sb);
	return ((void*)ps + ps->s_bitmap_start);
}

// If this is part of a read-modify-write of the inode metadata,
// pram_lock_inode() before calling!
static inline struct pram_inode *
pram_get_inode(struct super_block * sb, pram_off_t ino)
{
	struct pram_super_block * ps = pram_get_super(sb);
	return ino ? (struct pram_inode *)((void*)ps + ino) : NULL;
}

static inline ino_t
pram_get_inodenr(struct super_block * sb, struct pram_inode * pi)
{
	struct pram_super_block * ps = pram_get_super(sb);
	return (ino_t)((unsigned long)pi - (unsigned long)ps);
}

static inline pram_off_t
pram_get_block_off(struct super_block * sb, unsigned long blocknr)
{
	struct pram_super_block * ps = pram_get_super(sb);
	return (pram_off_t)(ps->s_bitmap_start +
			     (blocknr << sb->s_blocksize_bits));
}

static inline unsigned long
pram_get_blocknr(struct super_block * sb, pram_off_t block)
{
	struct pram_super_block * ps = pram_get_super(sb);
	return (block - ps->s_bitmap_start) >> sb->s_blocksize_bits;
}

// If this is part of a read-modify-write of the block,
// pram_lock_block() before calling!
static inline void *
pram_get_block(struct super_block * sb, pram_off_t block)
{
	struct pram_super_block * ps = pram_get_super(sb);
	return block ? ((void*)ps + block) : NULL;
}


/*
 * Inodes and files operations
 */

/* dir.c */
extern struct file_operations pram_dir_operations;

/* file.c */
extern struct inode_operations pram_file_inode_operations;
extern struct file_operations pram_file_operations;

/* inode.c */
extern struct address_space_operations pram_aops;

/* namei.c */
extern struct inode_operations pram_dir_inode_operations;

/* symlink.c */
extern struct inode_operations pram_symlink_inode_operations;

#endif	/* __KERNEL__ */

#endif	/* _LINUX_PRAM_FS_H */
