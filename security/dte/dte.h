#ifndef __DTE_H
#define __DTE_H

/*
 *  Domain and Type Enforcement Security plug
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 * author: Serge Hallyn  <hallyn@cs.wm.edu>
 */

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/mount.h>

/* type access */
#define DTE_READ    1
#define DTE_WRITE   2
#define DTE_EXECUTE 4
#define DTE_APPEND  8
#define DTE_READDIR 16
#define DTE_CREATE  32
#define DTE_DESCEND 64

/* same thing */
#define DTE_FR   1
#define DTE_FW   2
#define DTE_FX   4
#define DTE_FA   8
#define DTE_DR  16
#define DTE_DW  32
#define DTE_DX  64

#define DTE_ACCESS_GRANTED 0
#define DTE_FR_DENIED      1
#define DTE_FW_DENIED      2
#define DTE_FX_DENIED      3
#define DTE_FA_DENIED      4
#define DTE_DR_DENIED      5
#define DTE_DW_DENIED      6
#define DTE_DX_DENIED      7
#define DTE_ERR_NOHASH     8
#define DTE_ERR_NOENT      8

/* domain access */
#define DTE_EXEC 1
#define DTE_AUTO 2

#define dte_fr_access(x) (x & DTE_READ)
#define dte_fw_access(x) (x & DTE_WRITE)
#define dte_fx_access(x) (x & DTE_EXECUTE)
#define dte_create_access(x)  (x & DTE_CREATE)
#define dte_dw_access(x)      (x & DTE_CREATE)
#define dte_descend_access(x) (x & DTE_DESCEND)
#define dte_dx_access(x)      (x & DTE_DESCEND)
#define dte_readdir_access(x) (x & DTE_READDIR)
#define dte_dr_access(x)      (x & DTE_READDIR)

#define dte_exec_access(x) (x & DTE_EXEC)
#define dte_auto_access(x) (x & DTE_AUTO)

#define NAME_ALLOC_LEN(len)   ((len+16) & ~15)

/*
 * SECTION
 * mapnodes:
 * mirror the dentry tree as far down as type assignment rules go.
 * Every inode whose (first-loaded) name is subject to a type
 * assignment rule will point to a mapnode.
 */
struct dte_map_node {
	char *name;
	int namelen;
	char *etype, *utype;
	int num_kids;
	struct dte_map_node **kids;
	struct dte_map_node *hash_next;
};

/*
 * SECTION
 * mount restrictions/pretend stuff
 */

#define DTE_MOUNT_PRETEND  1
#define DTE_MOUNT_RESTRICT 2
struct dte_mntr {
	unsigned char maj, min;
	char summ[8];  /* for hashing, convenience: "maj,min" */
	char *path;
	char how;
	struct dte_mntr *hash_next;
};

/*
 * SECTION
 * dte-lsm security labels
 */
struct dte_inode_sec {
	char *etype;
	char *utype;
	struct dte_map_node *map;
	char initialized;
	struct semaphore s_sem;
};

struct dte_task_sec {
	struct dte_domain_t *dte_domain;
	struct dte_domain_t *dte_back;
};

struct dte_sb_sec {
	char initialized;
	struct semaphore s_sem;

	/* hierarchical type assignment */
	/* this may become a list, to deal with cloned fs trees */
	struct vfsmount *mnt_parent;
	struct dentry *mountpoint;

	/* type assignment through external attributes */
	/* use a inode->type file... */
	int ntypes;
	char **type_conv;   /* type conversion array */
	struct file *fp;     /* the file storing ino->type */
	char fp_ready;
	int offset;  /* offset into file where inodes start */
};

/* entry points into domains */
/* this used to be done by filenames.  However we no longer have access at
 * run-time to filenames.  So we determine entry points by typename.  Ugh.
 */
struct dte_ep {
	char   *type;
	struct dte_ep *hash_next;
};

/*
 * the next two are hashed by domain name
 */
struct dte_ea {
	struct dte_domain_t *other_domain;  /* points into dte_domain_cache */
	unsigned char access;
	struct dte_ea *hash_next;
};

/*
 * since domains may send any signals to themselves, we use the
 * domain being defined as recv_domain for ->0 definitions.  That
 * is, if d->sa[j]->recv_domain==d, then d->sa[j]->sig may be sent
 * to anyone
 */
struct dte_sa {
	struct dte_domain_t *recv_domain;
	int signal;       /* 0 means all signals */
	struct dte_sa *hash_next;
};

struct dte_gateway {
	char *type;  /* dte type of files which may be used to enter a domain */
	struct dte_domain_t *domain;
	struct dte_gateway *hash_next;
};

/*
 * hashed by type name
 */
struct dte_ta {
	char *type;  /* points into dte_type_cache */
	unsigned char access;
	struct dte_ta *hash_next;
};

struct dte_domain_t {
	char *name;
	int namelen;
	int num_ep;
	struct dte_ep *ep;
	int num_ta;
	struct dte_ta *ta;
	int num_ea;
	struct dte_ea *ea;
	int num_sa;
	struct dte_sa *sa;
	int num_gw;             /* number of gateways */
	struct dte_gateway *gw; /* gateways into other domains */
	struct dte_domain_t *hash_next;
};

/*
 * SECTION
 * defs/structs to aid in reading policy file
 */

/* states reached during the reading of config file */
#define DTE_STATE_TYPES 1
#define DTE_STATE_DOMAINS 2
#define DTE_STATE_DEFS 3
#define DTE_STATE_SPECD 4
#define DTE_STATE_TA 5  /* type assigns */
#define DTE_STATE_DONE 30

struct dte_fdata {
	struct file *fin;
	int numbufs;  /* how many buffers have been read? */
	int state;
	char buffer[4096];
	int buflen;   /* how much was actually read into buffer */
	int eof;      /* have we reached end of file? */
	char *blin, *elin;  /* start and end of current line */
	char *mark;   /* after buffer read, points to prev eob */
	int len;      /* length of current line */
	mm_segment_t *fs;
};

/* from read_policy.c: */
unsigned int dte_hash(const char *s, int n);
unsigned int dte_hash_c(char *c, char *ce, int n);
int read_dte_config(void);
int dte_setup_gateways(void);
int dte_sort_mntrs(void);
void free_dte_memory(int depth);
#ifdef CONFIG_DTE_VERBOSE
void show_dte(void);
#endif
char *dte_get_type(char *c, char *ce);

/* from inode.c: */
void dte_copy_ino_sec(struct inode *p, struct inode *c);
void dte_post_lookup (struct inode *ino, struct dentry *d);
int dte_inode_alloc_security	(struct inode *inode);
void dte_inode_free_security	(struct inode *inode);
void dte_inode_post_create (struct inode *inode, struct dentry *dentry,
			int mask);
void dte_inode_post_mknod (struct inode *inode, struct dentry *dentry,
			int major, dev_t minor);
void dte_inode_post_symlink (struct inode *inode, struct dentry *dentry,
				    const char *name);
void dte_inode_post_mkdir (struct inode *inode, struct dentry *dentry,
				  int mask);
int dte_inode_permission (struct inode *inode, int mask);
int find_type_conv(char **conv_array, int array_size, char *type);

/* from mount.c: */
void hierarchical_setup(struct vfsmount *mnt);
void dte_post_mountroot (void);
void dte_post_addmount (struct vfsmount *mnt, struct nameidata *nd);
int dte_sb_alloc_security (struct super_block *sb);
void dte_sb_free_security (struct super_block *sb);
void dte_setup_eafile(struct super_block *sb, struct vfsmount *mnt);
int dte_mount (char * dev_name, struct nameidata *nd, char * type,
			unsigned long flags, void * data);
int dte_check_sb (struct vfsmount *mnt, struct nameidata *nd);
int dte_mount (char * dev_name, struct nameidata *nd, char * type,
			unsigned long flags, void * data);
int dte_umount (struct vfsmount *mnt, int flags);
int dte_check_sb (struct vfsmount *mnt, struct nameidata *nd);

/* from path.c: */
struct dte_map_node *dte_find_map_node_create(char *path);
int dte_c_maptohash(struct dte_map_node *p);
char * dte_d_path(struct dentry *dentry, struct vfsmount *vfsmnt,
			char *buffer, int buflen);
struct dte_map_node *mapnode_getkid(struct dte_map_node *m,
			const unsigned char *name);

/* from syscall.c: */
int dte_sys_security(unsigned int id, unsigned int call,
		unsigned long *args);

/* from task.c: */
int dte_binprm_alloc_security (struct linux_binprm *bprm);
void dte_binprm_free_security (struct linux_binprm *bprm);
int dte_binprm_set_security (struct linux_binprm *bprm);
int dte_task_alloc_security (struct task_struct *p);
void dte_task_free_security (struct task_struct *p);
int dte_task_kill (struct task_struct *p, struct siginfo *info, int sig);
#endif
