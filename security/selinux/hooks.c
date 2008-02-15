/*
 *  NSA Security-Enhanced Linux (SELinux) security module
 *
 *  This file contains the SELinux hook function implementations.
 *
 *  Authors:  Stephen Smalley, <sds@epoch.ncsc.mil>
 *            Chris Vance, <cvance@nai.com>
 *            Wayne Salamon, <wsalamon@nai.com>
 *
 *  Copyright (C) 2001,2002 Networks Associates Technology, Inc.
 * 
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 * [Jul 2002, Rogelio M. Serrano Jr.] Added type transitions for pts in devfs.
 */ 

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/security.h>
#include <linux/capability.h>
#include <linux/unistd.h>
#include <linux/flask/avc.h>
#include <linux/flask/nsid.h>
#include <linux/flask/psid.h>
#include <linux/flask/syscalls.h>
#include <linux/flask/syscalls_proto.h>
#include <asm/flask/syscallaccess.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/spinlock.h>
#include <linux/file.h>
#include <linux/ext2_fs.h>
#include <linux/proc_fs.h>
#include <linux/kd.h>
#include <net/icmp.h>
#include <net/ip.h>		/* for sysctl_local_port_range[] */
#include <net/tcp.h>		/* struct or_callable used in sock_rcv_skb */
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/ioctls.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/netfilter.h>	/* for network interface checks */
#include <linux/netfilter_ipv4.h>
#include <linux/netdevice.h>	/* for network interface checks */
#include <linux/netlink.h>
#include <linux/tcp.h>
#include <linux/brlock.h>
#include <linux/quota.h>
#include <linux/un.h>		/* for Unix socket types */
#include <net/af_unix.h>	/* for Unix socket types */
#include "selinux_plug.h"
#include "extsocket.h"
#include <linux/flask/selopt.h>

#ifndef _SELINUX_KERNEL_PATCH_
#warning "SELinux:  The separate SELinux kernel patch has not been applied.  Using SELinux without this patch is not recommended."
#endif

#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
int selinux_enforcing = 0;

static int __init enforcing_setup(char *str)
{
	selinux_enforcing = simple_strtol(str,NULL,0);
	return 1;
}
__setup("enforcing=", enforcing_setup);
#endif

/* Original (dummy) security module. */
static struct security_operations *original_ops = NULL;

/* Minimal support for a secondary security module,
   just to allow the use of the dummy or capability modules. 
   The owlsm module can alternatively be used as a secondary 
   module as long as CONFIG_OWLSM_FD is not enabled. */
static struct security_operations *secondary_ops = NULL;

/* Lists of security blobs created by this module. 
   Used to deallocate all security blobs and clear security 
   fields when the module exits. */
static LIST_HEAD(task_security_head);
static LIST_HEAD(sock_security_head);
static LIST_HEAD(inode_security_head);
static LIST_HEAD(file_security_head);
static LIST_HEAD(msg_security_head);
static LIST_HEAD(ipc_security_head);
static LIST_HEAD(superblock_security_head);
static LIST_HEAD(skb_security_head);
static LIST_HEAD(netdev_security_head);

/* Allocate and free functions for each kind of security blob. */

static spinlock_t task_alloc_lock = SPIN_LOCK_UNLOCKED;

static int task_alloc_security(struct task_struct *task)
{
	struct task_security_struct *tsec, *new_tsec;

	new_tsec = kmalloc(sizeof(struct task_security_struct), GFP_KERNEL);
	if (!new_tsec) 
		return -ENOMEM; 

	spin_lock(&task_alloc_lock);
	tsec = task->security;
	if (tsec && tsec->magic == SELINUX_MAGIC) {
		spin_unlock(&task_alloc_lock);
		kfree(new_tsec);
		return 0;
	}
	tsec = new_tsec;

	memset(tsec, 0, sizeof(struct task_security_struct));
	tsec->magic = SELINUX_MAGIC;
	tsec->task = task;
	list_add(&tsec->list, &task_security_head);
	tsec->osid = tsec->sid = SECINITSID_UNLABELED;
	task->security = tsec;

	spin_unlock(&task_alloc_lock);
	return 0;
}

static void task_free_security(struct task_struct *task)
{
	struct task_security_struct *tsec = task->security;

	if (!tsec || tsec->magic != SELINUX_MAGIC)
		return; 

	task->security = NULL;
	spin_lock(&task_alloc_lock);
	list_del(&tsec->list);
	spin_unlock(&task_alloc_lock);
	kfree(tsec);
}

#ifdef CONFIG_SECURITY_NETWORK

/* 
 * Functions used to allocate/free sock security structures.
 */
static spinlock_t sock_alloc_lock = SPIN_LOCK_UNLOCKED;

static int sock_alloc_security(struct sock *sk, unsigned int gfp_mask)
{
	struct sock_security_struct *sksec, *new_sksec;
	unsigned long flags;

	new_sksec = kmalloc(sizeof(struct sock_security_struct), gfp_mask);
	if (!new_sksec)
		return -ENOMEM;

	spin_lock_irqsave(&sock_alloc_lock, flags);
	sksec = sk->security;
	if (sksec && sksec->magic == SELINUX_MAGIC) {
		spin_unlock_irqrestore(&sock_alloc_lock,flags);
		kfree(new_sksec);
		return 0;
	}
	sksec = new_sksec;

	memset(sksec, 0, sizeof(struct sock_security_struct));
	sksec->magic = SELINUX_MAGIC;
	sksec->sk = sk;
	list_add(&sksec->list, &sock_security_head);
	sksec->sid = sksec->peer_sid = SECINITSID_UNLABELED;
	sk->security = sksec;

	spin_unlock_irqrestore(&sock_alloc_lock, flags);
	return 0;
}

static void sock_free_security(struct sock *sk)
{
	struct sock_security_struct *sksec;
	unsigned long flags;

	sksec = sk->security;
	if (!sksec || sksec->magic != SELINUX_MAGIC)
		return;

	sk->security = NULL;
	spin_lock_irqsave(&sock_alloc_lock, flags);
	list_del(&sksec->list);
	spin_unlock_irqrestore(&sock_alloc_lock, flags);
	kfree(sksec);
}

#endif

static spinlock_t inode_alloc_lock = SPIN_LOCK_UNLOCKED;

static int inode_alloc_security(struct inode *inode)
{
	struct task_security_struct *tsec = current->security;
	struct inode_security_struct *isec, *new_isec;

	new_isec = kmalloc(sizeof(struct inode_security_struct), GFP_KERNEL);
	if (!new_isec) 
		return -ENOMEM;

	spin_lock(&inode_alloc_lock);
	isec = inode->i_security;
	if (isec && isec->magic ==  SELINUX_MAGIC) {
		spin_unlock(&inode_alloc_lock);
		kfree(new_isec);
		return 0;
	}
	isec = new_isec;

	memset(isec, 0, sizeof(struct inode_security_struct));
	init_MUTEX(&isec->sem);
	isec->magic = SELINUX_MAGIC;
	isec->inode = inode;
	list_add(&isec->list, &inode_security_head);
	isec->sid = SECINITSID_UNLABELED;
	isec->sclass = SECCLASS_FILE;
	if (tsec && tsec->magic == SELINUX_MAGIC)
		isec->task_sid = tsec->sid;
	else
		isec->task_sid = SECINITSID_UNLABELED;
	extsocket_init(isec);
	inode->i_security = isec;

	spin_unlock(&inode_alloc_lock);
	return 0;
}

static void inode_free_security(struct inode *inode)
{
	struct inode_security_struct *isec = inode->i_security;

	if (!isec || isec->magic != SELINUX_MAGIC) 
		return; 

	inode->i_security = NULL;
	spin_lock(&inode_alloc_lock);
	list_del(&isec->list);
	spin_unlock(&inode_alloc_lock);
	kfree(isec);
}

static spinlock_t file_alloc_lock = SPIN_LOCK_UNLOCKED;

static int file_alloc_security(struct file *file)
{
	struct task_security_struct *tsec = current->security;
	struct file_security_struct *fsec, *new_fsec;

	new_fsec = kmalloc(sizeof(struct file_security_struct), GFP_ATOMIC);
	if (!new_fsec) 
		return -ENOMEM;

	spin_lock(&file_alloc_lock);
	fsec = file->f_security;
	if (fsec && fsec->magic == SELINUX_MAGIC) {
		spin_unlock(&file_alloc_lock);
		kfree(new_fsec);
		return 0;
	}
	fsec = new_fsec;

	memset(fsec, 0, sizeof(struct file_security_struct));
	fsec->magic = SELINUX_MAGIC;
	fsec->file = file;
	list_add(&fsec->list, &file_security_head);
	if (tsec && tsec->magic == SELINUX_MAGIC) {
		fsec->sid = tsec->sid;
		fsec->fown_sid = tsec->sid;
	} else {
		fsec->sid = SECINITSID_UNLABELED;
		fsec->fown_sid = SECINITSID_UNLABELED;
	}
	file->f_security = fsec;

	spin_unlock(&file_alloc_lock);
	return 0;
}

static void file_free_security(struct file *file)
{
	struct file_security_struct *fsec = file->f_security;

	if (!fsec || fsec->magic != SELINUX_MAGIC) 
		return; 

	file->f_security = NULL;
	spin_lock(&file_alloc_lock);
	list_del(&fsec->list);
	spin_unlock(&file_alloc_lock);
	kfree(fsec);
}

static spinlock_t sb_alloc_lock = SPIN_LOCK_UNLOCKED;

static int superblock_alloc_security(struct super_block *sb)
{
	struct superblock_security_struct *sbsec, *new_sbsec;

	new_sbsec = kmalloc(sizeof(struct superblock_security_struct), GFP_KERNEL);
	if (!new_sbsec) 
		return -ENOMEM;

	spin_lock(&sb_alloc_lock);
	sbsec = sb->s_security;
	if (sbsec && sbsec->magic == SELINUX_MAGIC) {
		spin_unlock(&sb_alloc_lock);
		kfree(new_sbsec);
		return 0;
	}
	sbsec = new_sbsec;

	memset(sbsec, 0, sizeof(struct superblock_security_struct));
	init_MUTEX(&sbsec->sem);
	sbsec->magic = SELINUX_MAGIC;
	sbsec->sb = sb;
	list_add(&sbsec->list, &superblock_security_head);
	sbsec->sid = SECINITSID_UNLABELED;
	sb->s_security = sbsec;

	spin_unlock(&sb_alloc_lock);
	return 0;
}

static void superblock_free_security(struct super_block *sb)
{
	struct superblock_security_struct *sbsec = sb->s_security;

	if (!sbsec || sbsec->magic != SELINUX_MAGIC) 
		return; 

	if (sbsec->uses_psids)
		psid_release(sb);

	sb->s_security = NULL;
	spin_lock(&sb_alloc_lock);
	list_del(&sbsec->list);
	spin_unlock(&sb_alloc_lock);
	kfree(sbsec);
}

#ifdef CONFIG_SECURITY_NETWORK

static spinlock_t skb_alloc_lock = SPIN_LOCK_UNLOCKED;

static int skb_alloc_security(struct sk_buff *skb, int gfp_mask)
{
	struct skb_security_struct *ssec, *new_ssec;
	unsigned long flags;

	new_ssec = kmalloc(sizeof(struct skb_security_struct), gfp_mask);
	if (!new_ssec)
		return -ENOMEM;

	spin_lock_irqsave(&skb_alloc_lock,flags);
	ssec = skb->lsm_security;
	if (ssec && ssec->magic == SELINUX_MAGIC) {
		spin_unlock_irqrestore(&skb_alloc_lock,flags);
		kfree(new_ssec);
		return 0;
	}
	ssec = new_ssec;
	
	memset(ssec, 0, sizeof(struct skb_security_struct));
	atomic_set(&ssec->use, 1);
	ssec->magic = SELINUX_MAGIC;
	ssec->skb = skb;
	ssec->ssid = ssec->msid = ssec->dsid = SECINITSID_UNLABELED;
	list_add(&ssec->list, &skb_security_head); 
	skb->lsm_security = ssec;

	spin_unlock_irqrestore(&skb_alloc_lock,flags);
	return 0;
}

static void skb_free_security(struct sk_buff *skb)
{
	struct skb_security_struct *ssec = skb->lsm_security;
	unsigned long flags;

	if (!ssec || ssec->magic != SELINUX_MAGIC)
		return;

	skb->lsm_security = NULL;
	       
	if (atomic_dec_and_test(&ssec->use)) {
		spin_lock_irqsave(&skb_alloc_lock,flags);
		list_del(&ssec->list); 
		spin_unlock_irqrestore(&skb_alloc_lock,flags);
		kfree(ssec);
	}
}

static spinlock_t netdev_alloc_lock = SPIN_LOCK_UNLOCKED;

static int netdev_alloc_security(struct net_device *dev)
{
	struct netdev_security_struct *nsec, *new_nsec;
	unsigned long flags;

	new_nsec = kmalloc(sizeof(struct netdev_security_struct), SAFE_ALLOC);
	if (!new_nsec) 
		return -ENOMEM;

	spin_lock_irqsave(&netdev_alloc_lock,flags);
	nsec = dev->security;
	if (nsec && nsec->magic == SELINUX_MAGIC) {
		spin_unlock_irqrestore(&netdev_alloc_lock,flags);
		kfree(new_nsec);
		return 0;
	}
	nsec = new_nsec;

	memset(nsec, 0, sizeof(struct netdev_security_struct));
	nsec->magic = SELINUX_MAGIC;
	nsec->dev = dev;
	list_add(&nsec->list, &netdev_security_head);
	nsec->sid = nsec->default_msg_sid = SECINITSID_UNLABELED;
	dev->security = nsec;

	spin_unlock_irqrestore(&netdev_alloc_lock,flags);
	return 0;
}

static void netdev_free_security(struct net_device *dev)
{
	struct netdev_security_struct *nsec = dev->security;
	unsigned long flags;

	if (!nsec || nsec->magic != SELINUX_MAGIC)
		return; 

	dev->security = NULL;
	spin_lock_irqsave(&netdev_alloc_lock,flags);
	list_del(&nsec->list);
	spin_unlock_irqrestore(&netdev_alloc_lock,flags);
	kfree(nsec);
}

#endif

/* The security server must be initialized before
   any labeling or access decisions can be provided. */
extern int ss_initialized;

/* The file system's label must be initialized prior to use.
   If the file system is persistent, then its persistent label mapping 
   must be initialized before labels for files in it can be obtained. */

static char *labeling_behaviors[5] = {
	"uses PSIDs",
	"uses transition SIDs",
	"uses task SIDs",
	"uses genfs_contexts",
	"not configured for labeling"
};

static int inode_doinit_with_dentry(struct inode *inode, struct dentry *opt_dentry);

static inline int inode_doinit(struct inode *inode) 
{
	return inode_doinit_with_dentry(inode, NULL);
}

static int superblock_doinit(struct super_block *sb) 
{
	struct superblock_security_struct *sbsec = sb->s_security;
	int rc, behavior;

	if (!sbsec) {
		rc = superblock_alloc_security(sb);
		if (rc)
			return rc;
		sbsec = sb->s_security;
	}

	if (sbsec->initialized) {
		/* do_kern_mount can be called multiple times on the same
		   superblock.  We only need to initialize its security 
		   state once. */
		return 0;
	}

	if (!ss_initialized) {
		/* Defer initialization until selinux_post_mountroot,
		   after the initial policy is loaded and the security
		   server is ready to handle calls. */
		return 0;
	}

	/* Determine the labeling behavior to use for this filesystem type. */
	rc = security_fs_use(sb->s_type->name, &behavior, &sbsec->sid);
	if (rc) {
		printk("%s:  security_fs_use(%s) returned %d\n", __FUNCTION__, sb->s_type->name, rc);
		behavior = SECURITY_FS_USE_NONE;
	}

	switch (behavior) {
	case SECURITY_FS_USE_PSID:
		/* PSIDs only work for persistent file systems with
		   unique and persistent inode numbers. */
		rc = psid_init(sb);
		if (rc) {
			printk("%s: psid_init returned "
			       "%d... (filesystem=%s, pid=%d)\n", 
			       __FUNCTION__,
			       -rc, sb->s_root->d_name.name, current->pid);
			return rc; 
		}
		sbsec->uses_psids = 1;
		break;
	case SECURITY_FS_USE_TRANS:
		/* Transition SIDs are used for pseudo filesystems like 
		   devpts and tmpfs where you want the SID to be derived
		   from the SID of the creating process and the SID of
		   the filesystem. */
		sbsec->uses_trans = 1;
		break;
	case SECURITY_FS_USE_TASK:
		/* Task SIDs are used for pseudo filesystems like pipefs
		   and sockfs where you want the objects to be labeled
		   with the SID of the creating process. */
		sbsec->uses_task = 1;
		break;
	case SECURITY_FS_USE_GENFS:
		/* genfs_contexts handles everything else, like devfs,
		   usbdevfs, driverfs, and portions of proc. */
		sbsec->uses_genfs = 1;
		break;
	case SECURITY_FS_USE_NONE:
		/* No labeling support configured for this filesystem type.
		   Don't appear to require labeling for binfmt_misc, bdev, 
		   or rootfs. */
		break;
	default:
		printk("%s:  security_fs_use(%s) returned unrecognized behavior %d\n", __FUNCTION__, sb->s_type->name, behavior);
		behavior = SECURITY_FS_USE_NONE;
		break;
	}
	
	/* proc also requires specialized processing for 
	   the /proc/PID entries. */
	if (strcmp(sb->s_type->name, "proc") == 0) 
		sbsec->proc = 1;

	sbsec->initialized = 1;

	printk(KERN_INFO "SELinux: initialized (dev %s, type %s), %s\n",
	       kdevname(to_kdev_t(sb->s_dev)), sb->s_type->name,
	       labeling_behaviors[behavior-1]);
	
	/* Initialize the root inode. */
	rc = inode_doinit_with_dentry(sb->s_root->d_inode, sb->s_root);
	if (rc)
		return rc;

	return 0;
}

/*
 * Set the SID on a /proc/PID entry.
 * The /proc/PID entries are labeled with the SID of the owning process.
 * Most entries simply inherit their SID from the parent directory SID.
 */
static void procfs_set_sid(struct inode *inode, struct dentry *dentry)
{
	struct task_security_struct *tsec;
	struct inode_security_struct *isec, *pisec;
	struct dentry *parent;
	unsigned int pid, c;
	struct task_struct *task;
	const char *name;
	int len;

	isec = inode->i_security;
#ifndef _SELINUX_KERNEL_PATCH_
	if (!isec)
		return;
#endif

	name = dentry->d_name.name;
	len = dentry->d_name.len;

	parent = dentry->d_parent;
	if (!parent || parent == dentry) 
		return;
	pisec = parent->d_inode->i_security;
#ifndef _SELINUX_KERNEL_PATCH_
	if (!pisec)
		return;
#endif

	if (pisec->inherit) {
		/* Inherit the SID from the parent by default. */
		isec->sid = pisec->sid;
		/* Propagate this property to all descendants. */
		isec->inherit = 1;
	}

	if (parent->d_parent != parent) {
		/* This entry is not in the top-level /proc directory.
		   Simply return, inheriting from the parent. */
		return;
	}

	/* For /proc/PID entries, use the SID of the owning process. 
	   Derived from fs/proc/base.c:proc_pid_lookup. */
#define MAX_MULBY10	((~0U-9)/10)
	pid = 0;
	while (len-- > 0) {
		c = *name - '0';
		name++;
		if (c > 9)
			return;
		if (pid >= MAX_MULBY10)
			return;
		pid *= 10;
		pid += c;
		if (!pid)
			return;
	}

	read_lock(&tasklist_lock);
	task = find_task_by_pid(pid);
	if (task)
		get_task_struct(task);
	read_unlock(&tasklist_lock);
	if (!task)
		return;

	tsec = task->security;
	isec->sid = tsec->sid;
	isec->inherit = 1;
	free_task_struct(task);

	return;
}

extern char * avc_d_path(struct dentry *dentry, 
			 char *buffer, int buflen);

/* The inode's security attributes must be initialized before first use. */
static int inode_doinit_with_dentry(struct inode *inode, struct dentry *opt_dentry) 
{
	struct superblock_security_struct *sbsec = NULL; 
	struct inode_security_struct *isec = inode->i_security;
	security_id_t sid, devfs_pts_sid;
	char *buffer, *path;
	struct dentry *dentry;
	int rc;

	if (!isec) {
		rc = inode_alloc_security(inode);
		if (rc)
			return rc;
		isec = inode->i_security;
	}

	down(&isec->sem);
	if (isec->initialized) {
		/* Already initialized. */
		up(&isec->sem);
		return 0;
	}

	if (inode->i_sock) {
		struct socket *sock = &inode->u.socket_i;
		if (sock->sk) {
			isec->sclass = socket_type_to_security_class(sock->sk->family, sock->sk->type);
		} else {
			/* else TCP control message? */
			isec->sclass = SECCLASS_SOCKET;
		}
	} else {
		isec->sclass = inode_mode_to_security_class(inode->i_mode);
	}

	if (inode->i_sb) {
		sbsec = inode->i_sb->s_security;
		if (!sbsec || !sbsec->initialized) {
			/* Defer initialization until selinux_post_mountroot,
			   after the initial policy is loaded and the security
			   server is ready to handle calls. */
			up(&isec->sem);
			return 0;
		}
	} else {
		isec->initialized = 1;
		up(&isec->sem);
		return 0;
	}
		
	if (sbsec->uses_psids) {
		rc = psid_to_sid(inode, &isec->sid);
		if (rc) {
			printk("%s:  psid_to_sid returned %d for dev=%s ino=%ld\n", __FUNCTION__, -rc, kdevname(to_kdev_t(inode->i_dev)), inode->i_ino);
			up(&isec->sem);
			return rc;
		}
	} else if (sbsec->uses_task) {
		isec->sid = isec->task_sid;
	} else if (sbsec->uses_trans) {
		/* Default to the fs SID. */
		isec->sid = sbsec->sid;

		/* Try to obtain a transition SID. */
		rc = security_transition_sid(isec->task_sid, 
					     sbsec->sid,
					     isec->sclass,
					     &sid);
		if (!rc)
			isec->sid = sid;
	} else if (sbsec->uses_genfs) {
		/* Default to the fs SID. */
		isec->sid = sbsec->sid;
		
		/* Obtain a SID based on the fstype, path, and class. */
		if (opt_dentry) 
			dentry = dget(opt_dentry);
		else
			dentry = d_find_alias(inode);
		if (dentry) {
			buffer = (char*)__get_free_page(GFP_KERNEL);
			if (buffer) {
				path = avc_d_path(dentry, buffer,
						  PAGE_SIZE);
				if (path) {
					if (!strcmp(inode->i_sb->s_type->name,
						    "devfs") && 
					    !memcmp(path, "/pts/", 5)) {
						rc = security_genfs_sid("devfs", "/pts", SECCLASS_DIR, &devfs_pts_sid);
						if (!rc) 
						  rc = security_transition_sid(
							  isec->task_sid, 
							  devfs_pts_sid, 
							  isec->sclass,
							  &sid);
					} else {
						rc = security_genfs_sid(inode->i_sb->s_type->name,
									path, 
									isec->sclass,
									&sid);
					}
					if (!rc)
						isec->sid = sid;
				}
				free_page((unsigned long)buffer);
			}
				
			if (sbsec->proc) {
				/* Handle /proc/PID. */
				procfs_set_sid(inode, dentry);
			}
				
			dput(dentry);
		}
	} else {
		/* Default to the fs SID. */
		isec->sid = sbsec->sid;
	}

	isec->initialized = 1;
	up(&isec->sem);
	return 0;
}

#ifdef CONFIG_SECURITY_NETWORK

/* The network interface security attributes must be initialized before 
 * first use. */
int netdev_precondition(struct net_device *dev) 
{
	struct netdev_security_struct *nsec = dev->security;
	int rc;

	if (nsec && nsec->magic == SELINUX_MAGIC) 
		return 1;

	rc = netdev_alloc_security(dev);
	if (rc)
		return rc;

	/* Set the SID for the device and the default message SID */
	nsec = dev->security;

	rc = security_netif_sid(dev->name, &nsec->sid,
				&nsec->default_msg_sid);
	if (rc) 
		return rc;

	return 1;
}

#endif

/* Convert a Linux signal to an access vector. */
static inline access_vector_t signal_to_av(int sig) 
{
	access_vector_t perm = 0;

	switch (sig) {
	case SIGCHLD:
		/* Commonly granted from child to parent. */
		perm = PROCESS__SIGCHLD;
		break;
	case SIGKILL:
		/* Cannot be caught or ignored */
		perm = PROCESS__SIGKILL;
		break;
	case SIGSTOP:
		/* Cannot be caught or ignored */
		perm = PROCESS__SIGSTOP;
		break;
	default:
		/* All other signals. */
		perm = PROCESS__SIGNAL;
		break;
	}

	return perm;
}

/* Check permission betweeen a pair of tasks, e.g. signal checks,
   fork check, ptrace check, etc. */
int task_has_perm(struct task_struct *tsk1,
		  struct task_struct *tsk2,
		  access_vector_t perms)
{
	struct task_security_struct *tsec1, *tsec2;

	tsec1 = tsk1->security;
	tsec2 = tsk2->security;
	return avc_has_perm_ref(tsec1->sid, tsec2->sid, 
				SECCLASS_PROCESS, perms, &tsec2->avcr);
}

/* Check whether a task is allowed to use a capability. */
int task_has_capability(struct task_struct *tsk,
			int cap)
{
	struct task_security_struct *tsec;
	avc_audit_data_t ad;

	tsec = tsk->security;

	AVC_AUDIT_DATA_INIT(&ad,CAP);
	ad.u.cap = cap;
	return avc_has_perm_audit(tsec->sid, tsec->sid, 
				  SECCLASS_CAPABILITY, CAP_TO_MASK(cap), &ad);
}

/* Check whether a task is allowed to use a system operation. */
int task_has_system(struct task_struct *tsk,
		    access_vector_t perms)
{
	struct task_security_struct *tsec;

	tsec = tsk->security;

	return avc_has_perm(tsec->sid, SECINITSID_KERNEL, 
			    SECCLASS_SYSTEM, perms);
}

/* Check whether a task is allowed to use a security operation. */
int task_has_security(struct task_struct *tsk,
		      access_vector_t perms)
{
	struct task_security_struct *tsec;

	tsec = tsk->security;

	return avc_has_perm(tsec->sid, SECINITSID_SECURITY, 
			    SECCLASS_SECURITY, perms);
}

/* Check whether a task has a particular permission to an inode.
   The 'aeref' parameter is optional and allows other AVC
   entry references to be passed (e.g. the one in the struct file).
   The 'adp' parameter is optional and allows other audit
   data to be passed (e.g. the dentry). */
int inode_has_perm(struct task_struct *tsk,
		   struct inode *inode,
		   access_vector_t perms,
		   avc_entry_ref_t *aeref,
		   avc_audit_data_t *adp)
{
	struct task_security_struct *tsec;
	struct inode_security_struct *isec;
	avc_audit_data_t ad;

	tsec = tsk->security;
	isec = inode->i_security;

#ifndef _SELINUX_KERNEL_PATCH_
	if (!isec || !isec->initialized)
		return 0;
#endif

	if (!adp) {
		adp = &ad;
		AVC_AUDIT_DATA_INIT(&ad, FS);
		ad.u.fs.inode = inode;
	}

	return avc_has_perm_ref_audit(tsec->sid, isec->sid, isec->sclass, 
				      perms, aeref ? aeref : &isec->avcr, adp);
}

/* Same as inode_has_perm, but pass explicit audit data containing 
   the dentry to help the auditing code to more easily generate the 
   pathname if needed. */
static inline int dentry_has_perm(struct task_struct *tsk,
				  struct dentry *dentry, 
				  access_vector_t av,
				  avc_entry_ref_t *aeref) 
{
	struct inode *inode = dentry->d_inode;
	avc_audit_data_t ad;
	AVC_AUDIT_DATA_INIT(&ad,FS);
	ad.u.fs.dentry = dentry;
	return inode_has_perm(tsk, inode, av, aeref, &ad);
}

/* Check whether a task can use an open file descriptor to
   access an inode in a given way.  Check access to the
   descriptor itself, and then use dentry_has_perm to
   check a particular permission to the file.
   Access to the descriptor is implicitly granted if it
   has the same SID as the process.  If av is zero, then
   access to the file is not checked, e.g. for cases
   where only the descriptor is affected like seek. */
static inline int file_has_perm(struct task_struct *tsk,
				struct file *file, 
				access_vector_t av)
{
	struct task_security_struct *tsec;
	struct file_security_struct *fsec;
	struct dentry *dentry = file->f_dentry;
	avc_audit_data_t ad;
	int rc;

	tsec = tsk->security;
	fsec = file->f_security;

	if (tsec->sid != fsec->sid) {
		AVC_AUDIT_DATA_INIT(&ad, FS);
		ad.u.fs.dentry = file->f_dentry;

		rc = avc_has_perm_ref_audit(tsec->sid, fsec->sid, 
					    SECCLASS_FD,
					    FD__USE,
					    &fsec->avcr, &ad);
		if (rc)
			return rc;
	}

	/* av is zero if only checking access to the descriptor. */
	if (av)
		return dentry_has_perm(tsk, dentry, av, &fsec->inode_avcr);
	
	return 0;
}

/* Check whether a task can create a file. */
static int may_create(struct inode *dir, 
		      struct dentry *dentry, 
		      security_class_t tclass)
{
	struct task_security_struct *tsec;
	struct inode_security_struct *dsec;
	struct superblock_security_struct *sbsec;
	security_id_t newsid;
	avc_audit_data_t ad;
	int rc;

	tsec = current->security;
	dsec = dir->i_security;

#ifndef _SELINUX_KERNEL_PATCH_
	if (!dsec)
		return 0;
#endif

	AVC_AUDIT_DATA_INIT(&ad, FS);
	ad.u.fs.dentry = dentry;

	rc = avc_has_perm_ref_audit(tsec->sid, dsec->sid, SECCLASS_DIR, 
				    DIR__ADD_NAME | DIR__SEARCH, 
				    &dsec->avcr, &ad);
	if (rc)
		return rc;

	if (tsec->in_sid[0]) {
		newsid = tsec->in_sid[0];
	} else {
		rc = security_transition_sid(tsec->sid, dsec->sid, tclass, 
					     &newsid);
		if (rc)
			return rc;
	}

	rc = avc_has_perm_audit(tsec->sid, newsid, tclass, FILE__CREATE, &ad);
	if (rc)
		return rc;

	if (dir->i_sb) {
		sbsec = dir->i_sb->s_security;
#ifndef _SELINUX_KERNEL_PATCH_
		if (!sbsec)
			return 0;
#endif

		rc = avc_has_perm_audit(newsid, sbsec->sid, 
					SECCLASS_FILESYSTEM, 
					FILESYSTEM__ASSOCIATE, &ad);
		if (rc)
			return rc;
	}

	return 0;
}

#define MAY_LINK   0
#define MAY_UNLINK 1
#define MAY_RMDIR  2

/* Check whether a task can link, unlink, or rmdir a file/directory. */
static int may_link(struct inode *dir, 
		    struct dentry *dentry,
		    int kind)
				 
{
	struct task_security_struct *tsec;
	struct inode_security_struct *dsec, *isec;
	avc_audit_data_t ad;
	access_vector_t av;
	int rc;

	tsec = current->security;
	dsec = dir->i_security;
	isec = dentry->d_inode->i_security;

#ifndef _SELINUX_KERNEL_PATCH_
	if (!dsec || !isec)
		return 0;
#endif

	AVC_AUDIT_DATA_INIT(&ad, FS);
	ad.u.fs.dentry = dentry;

	av = DIR__SEARCH;
	av |= (kind ? DIR__REMOVE_NAME : DIR__ADD_NAME);
	rc = avc_has_perm_ref_audit(tsec->sid, dsec->sid, SECCLASS_DIR, 
				    av, &dsec->avcr, &ad);
	if (rc)
		return rc;

	switch (kind) {
	case MAY_LINK:
		av = FILE__LINK;
		break;
	case MAY_UNLINK:
		av = FILE__UNLINK;
		break;
	case MAY_RMDIR:
		av = DIR__RMDIR;
		break;
	default:
		printk("may_link:  unrecognized kind %d\n", kind);
		return 0;
	}

	rc = avc_has_perm_ref_audit(tsec->sid, isec->sid, isec->sclass, 
				    av, &isec->avcr, &ad);
	return rc;
}

static inline int may_rename(struct inode *old_dir, 
			     struct dentry *old_dentry, 
			     struct inode *new_dir, 
			     struct dentry *new_dentry)
{
	struct task_security_struct *tsec;
	struct inode_security_struct *old_dsec, *new_dsec, *old_isec, *new_isec;
	avc_audit_data_t ad;
	access_vector_t av;
	int old_is_dir, new_is_dir;
	int rc;

	tsec = current->security;
	old_dsec = old_dir->i_security;
	old_isec = old_dentry->d_inode->i_security;
	old_is_dir = S_ISDIR(old_dentry->d_inode->i_mode);
	new_dsec = new_dir->i_security;

#ifndef _SELINUX_KERNEL_PATCH_
	if (!old_dsec || !old_isec || !new_dsec)
		return 0;
#endif

	AVC_AUDIT_DATA_INIT(&ad, FS);

	ad.u.fs.dentry = old_dentry;
	rc = avc_has_perm_ref_audit(tsec->sid, old_dsec->sid, SECCLASS_DIR, 
				    DIR__REMOVE_NAME | DIR__SEARCH, 
				    &old_dsec->avcr, &ad);
	if (rc)
		return rc;
	rc = avc_has_perm_ref_audit(tsec->sid, old_isec->sid, 
				    old_isec->sclass, 
				    FILE__RENAME, 
				    &old_isec->avcr, &ad);
	if (rc)
		return rc;
	if (old_is_dir && new_dir != old_dir) {
		rc = avc_has_perm_ref_audit(tsec->sid, old_isec->sid, 
					    old_isec->sclass, 
					    DIR__REPARENT, 
					    &old_isec->avcr, &ad);
		if (rc)
			return rc;
	}

	ad.u.fs.dentry = new_dentry;
	av = DIR__ADD_NAME | DIR__SEARCH;
	if (new_dentry->d_inode)
		av |= DIR__REMOVE_NAME;
	rc = avc_has_perm_ref_audit(tsec->sid, new_dsec->sid, SECCLASS_DIR, 
				    av,&new_dsec->avcr, &ad);
	if (rc)
		return rc;
	if (new_dentry->d_inode) {
		new_isec = new_dentry->d_inode->i_security;
#ifndef _SELINUX_KERNEL_PATCH_
		if (!new_isec) 
			return 0;
#endif
		new_is_dir = S_ISDIR(new_dentry->d_inode->i_mode);
		rc = avc_has_perm_ref_audit(tsec->sid, new_isec->sid, 
					    new_isec->sclass,
					    (new_is_dir ? DIR__RMDIR : FILE__UNLINK),
					    &new_isec->avcr, &ad);
		if (rc)
			return rc;
	}

	return 0;
}

/* Check whether a task can perform a filesystem operation. */
int superblock_has_perm(struct task_struct *tsk,
			struct super_block *sb,
			access_vector_t perms,
			avc_audit_data_t *ad)
{
	struct task_security_struct *tsec;
	struct superblock_security_struct *sbsec;

	tsec = tsk->security;
	sbsec = sb->s_security;
#ifndef _SELINUX_KERNEL_PATCH_
	if (!sbsec)
		return 0;
#endif
	return avc_has_perm_audit(tsec->sid, sbsec->sid, SECCLASS_FILESYSTEM, 
				  perms, ad);
}

/* Convert a Linux mode and permission mask to an access vector. */
static inline access_vector_t file_mask_to_av(int mode, int mask) 
{
	access_vector_t av = 0;

	if ((mode & S_IFMT) != S_IFDIR) {
		if (mask & MAY_EXEC) 
			av |= FILE__EXECUTE;
		if (mask & MAY_READ) 
			av |= FILE__READ;

		if (mask & MAY_APPEND)
			av |= FILE__APPEND;
		else if (mask & MAY_WRITE) 
			av |= FILE__WRITE;

	} else {
		if (mask & MAY_EXEC) 
			av |= DIR__SEARCH;
		if (mask & MAY_WRITE) 
			av |= DIR__WRITE;
		if (mask & MAY_READ) 
			av |= DIR__READ;
	}

	return av;
}

/* Convert a Linux file to an access vector. */
static inline access_vector_t file_to_av(struct file *file)
{
	access_vector_t av = 0;

	if (file->f_mode & FMODE_READ) 
		av |= FILE__READ;
	if (file->f_mode & FMODE_WRITE) {
		if (file->f_flags & O_APPEND)
			av |= FILE__APPEND;
		else
			av |= FILE__WRITE;
	}

	return av;
}

/* Set an inode's SID, where the inode may or may not already
   have a security structure. */
int inode_security_set_sid(struct inode *inode, security_id_t sid)
{
	struct inode_security_struct *isec = inode->i_security;
	int rc;

	if (!isec) {
		rc = inode_alloc_security(inode);
		if (rc)
			return rc;
		isec = inode->i_security;
	}
	down(&isec->sem);
	isec->sclass = inode_mode_to_security_class(inode->i_mode);
	isec->sid = sid;
	isec->initialized = 1;
	up(&isec->sem);
	return 0;
}

/* Set the security attributes on a newly created file. */
static int post_create(struct inode *dir, 
		       struct dentry *dentry)
{

	struct task_security_struct *tsec;
	struct inode *inode;
	struct inode_security_struct *dsec;
	struct superblock_security_struct *sbsec;
	security_id_t newsid;
	int rc;

	tsec = current->security;
	dsec = dir->i_security;

#ifndef _SELINUX_KERNEL_PATCH_
	if (!dsec)
		return 0;
#endif

	inode = dentry->d_inode;
	if (!inode) {
		/* Some file system types (e.g. NFS) may not instantiate
		   a dentry for all create operations (e.g. symlink),
		   so we have to check to see if the inode is non-NULL. */
		printk("post_create:  no inode, dir (dev=%s, ino=%ld)\n", kdevname(to_kdev_t(dir->i_dev)), dir->i_ino);
		return 0;
	}

	/* XXX:  Need a way to propagate SID from may_create to post_create. */
	if (tsec->in_sid[0]) {
		newsid = tsec->in_sid[0];
	} else {
		rc = security_transition_sid(tsec->sid, dsec->sid, 
					     inode_mode_to_security_class(inode->i_mode), 
					     &newsid);
		if (rc) {
			printk("post_create:  security_transition_sid failed, rc=%d (dev=%s ino=%ld)\n", -rc, kdevname(to_kdev_t(inode->i_dev)), inode->i_ino);
			return rc;
		}
	}
	
	rc = inode_security_set_sid(inode, newsid);
	if (rc) {
		printk("post_create:  inode_security_set_sid failed, rc=%d (dev=%s ino=%ld)\n", -rc, kdevname(to_kdev_t(inode->i_dev)), inode->i_ino);
		return rc;
	}

	if (dir->i_sb) {
		sbsec = dir->i_sb->s_security;
		if (sbsec && sbsec->uses_psids) {
			rc = sid_to_psid(dentry->d_inode, newsid);
			if (rc) {
				printk("post_create:  sid_to_psid failed, rc=%d (dev=%s ino=%ld)\n", -rc, kdevname(to_kdev_t(inode->i_dev)), inode->i_ino);
				return rc;
			}
		}
	}

	return 0;
}	


/* Hook functions begin here. */

/* assorted security operations  (mostly syscall interposition) */

static int selinux_ptrace(struct task_struct *parent, struct task_struct *child)
{
	int rc;

	rc = secondary_ops->ptrace(parent,child);
	if (rc)
		return rc;

	return task_has_perm(parent, child, PROCESS__PTRACE);
}

static int selinux_capget(struct task_struct *target, kernel_cap_t *effective, kernel_cap_t *inheritable, kernel_cap_t *permitted)	
{
	int error;

	error = task_has_perm(current, target, PROCESS__GETCAP);
	if (error)
		return error;

	return secondary_ops->capget(target, effective, inheritable, permitted);
}

static int selinux_capset_check(struct task_struct *target, kernel_cap_t *effective, kernel_cap_t *inheritable, kernel_cap_t *permitted)	
{
	int error;

	error = task_has_perm(current, target, PROCESS__SETCAP);
	if (error)
		return error;

	return secondary_ops->capset_check(target, effective, inheritable, permitted);
}

static void selinux_capset_set(struct task_struct *target, kernel_cap_t *effective, kernel_cap_t *inheritable, kernel_cap_t *permitted) 
{
	int error;

	error = task_has_perm(current, target, PROCESS__SETCAP);
	if (error)
		return;

	return secondary_ops->capset_set(target, effective, inheritable, permitted);
}

static int selinux_capable(struct task_struct *tsk, int cap)
{
	int rc;

	rc = secondary_ops->capable(tsk, cap);
	if (rc)
		return rc;

	return task_has_capability(tsk,cap);
}

static int selinux_sysctl(ctl_table *table, int op)
{
	int error = 0;
	access_vector_t av;
	struct task_security_struct *tsec;
	security_id_t tsid;
	int rc;
#ifdef CONFIG_PROC_FS
	int buflen;
	char *buffer, *path, *end;
	struct proc_dir_entry *de;
#endif

	tsec = current->security;
	
	/* Default to the well-defined sysctl SID. */
	tsid = SECINITSID_SYSCTL;

#ifdef CONFIG_PROC_FS
	/* If procfs is enabled, use the proc_dir_entry to
	   construct a pathname for a lookup in genfs_contexts. */
	buffer = (char*)__get_free_page(GFP_KERNEL);
	if (buffer) {
		buflen = PAGE_SIZE;
		end = buffer+buflen;
		*--end = '\0';
		buflen--;
		path = end-1;
		*path = '/';
		de = table->de;
		while (de && de != de->parent) {
			buflen -= de->namelen + 1;
			if (buflen < 0)
				break;
			end -= de->namelen;
			memcpy(end, de->name, de->namelen);
			*--end = '/';
			path = end;
			de = de->parent;
		}
		rc = security_genfs_sid("proc", 
					path, SECCLASS_FILE, &tsid);
		if (rc) {
			/* Fall back to the sysctl SID if this fails. */
			tsid = SECINITSID_SYSCTL;
		}

		free_page((unsigned long)buffer);
	}
#endif	
	
	/* The op values are "defined" in sysctl.c, thereby creating
	 * a bad coupling between this module and sysctl.c */
	if(op == 001) {
		error = avc_has_perm(tsec->sid, tsid, 
					SECCLASS_DIR, DIR__SEARCH);
	} else {
		av = 0;
		if (op & 004)
			av |= FILE__READ;
		if (op & 002)
			av |= FILE__WRITE;
		if (av)
			error = avc_has_perm(tsec->sid, tsid,
                                             SECCLASS_FILE, av);
        }

	return error;
}

static int selinux_sys_security(unsigned int magic, unsigned int call, unsigned long *args) 
{
	/* This hook function is not used, because the SELinux
	   module replaces the generic security syscall with its
	   own call in order to have the registers on the stack
	   available for execve_secure. */
	return -ENOSYS;
}

static int selinux_swapon(struct swap_info_struct *swap)
{
	return dentry_has_perm(current, swap->swap_file, FILE__SWAPON, NULL);
}

static int selinux_swapoff(struct swap_info_struct *swap)
{
	if (swap)
		return dentry_has_perm(current, swap->swap_file, FILE__SWAPON, NULL);
	return 0;
}

static int selinux_nfsservctl(int cmd, struct nfsctl_arg *arg)
{
	return task_has_system(current, SYSTEM__NFSD_CONTROL);
}

static int selinux_quotactl(int cmds, int type, int id, struct super_block *sb)
{
	int rc = 0;

	if (!sb)
		return 0;

	switch (cmds) {
		case Q_QUOTAON:
		case Q_QUOTAOFF:
		case Q_SETQUOTA:
		case Q_SETUSE:
		case Q_SETQLIM:
		case Q_SYNC:
		case Q_RSQUASH:
			rc = superblock_has_perm(current, 
						 sb, 
						 FILESYSTEM__QUOTAMOD, NULL);
			break;
		case Q_GETQUOTA:
		case Q_GETSTATS:
			rc = superblock_has_perm(current, 
						 sb, 
						 FILESYSTEM__QUOTAGET, NULL);
			break;
		default:
			rc = 0;  /* let the kernel handle invalid cmds */
			break;
	}
	return rc;
}

static int selinux_quota_on(struct file *f)
{
	return file_has_perm(current, f, FILE__QUOTAON);;
}

static int selinux_bdflush(int func, long data)
{
	return task_has_system(current, SYSTEM__BDFLUSH);
}

static int selinux_syslog(int type)
{
	int rc;

	rc = secondary_ops->syslog(type);
	if (rc)
		return rc;

	switch (type) {
		case 3:         /* Read last kernel messages */
			rc = task_has_system(current, SYSTEM__SYSLOG_READ);
			break;
		case 6:         /* Disable logging to console */
		case 7:         /* Enable logging to console */
		case 8:		/* Set level of messages printed to console */
			rc = task_has_system(current, SYSTEM__SYSLOG_CONSOLE);
			break;
		case 0:         /* Close log */
		case 1:         /* Open log */
		case 2:         /* Read from log */
		case 4:         /* Read/clear last kernel messages */
		case 5:         /* Clear ring buffer */
		default:
			rc = task_has_system(current, SYSTEM__SYSLOG_MOD);
			break;
	}
	return rc;
}

static int selinux_netlink_send(struct sk_buff *skb)  
{
	if (capable(CAP_NET_ADMIN))
		cap_raise (NETLINK_CB (skb).eff_cap, CAP_NET_ADMIN);
	else
		NETLINK_CB(skb).eff_cap = 0;
	return 0;
}

static int selinux_netlink_recv(struct sk_buff *skb)  
{
	if (!cap_raised(NETLINK_CB(skb).eff_cap, CAP_NET_ADMIN)) 
		return -EPERM;
	return 0;
}

/* binprm security operations */

static int selinux_bprm_alloc_security(struct linux_binprm *bprm)
{
	int rc;

	/* Make sure that the secondary module doesn't use the 
	   bprm->security field, since we do not yet support chaining
	   of multiple security structures on the field.  Neither 
	   the dummy nor the capability module use the field.  The owlsm
	   module uses the field if CONFIG_OWLSM_FD is enabled. */
	rc = secondary_ops->bprm_alloc_security(bprm);
	if (rc)
		return rc;
	if (bprm->security) {
		printk(KERN_WARNING "%s: no support yet for chaining on the security field by secondary modules.\n", __FUNCTION__);
		/* Release the secondary module's security object. */
		secondary_ops->bprm_free_security(bprm);
		/* Unregister the secondary module to prevent problems 
		   with subsequent binprm hooks. This will revert to the
		   original (dummy) module for the secondary operations. */
		rc = security_ops->unregister_security("unknown", secondary_ops);
		if (rc)
			return rc;
		printk(KERN_WARNING "%s: Unregistered the secondary security module.\n", __FUNCTION__);
	}
	bprm->security = NULL;
	return 0;
}

static int selinux_bprm_set_security(struct linux_binprm *bprm)
{
	struct task_security_struct *tsec;
	struct inode *inode = bprm->file->f_dentry->d_inode;
	struct inode_security_struct *isec;
	security_id_t newsid;
	avc_audit_data_t ad;
	int rc;

	rc = secondary_ops->bprm_set_security(bprm);
	if (rc)
		return rc;

	if (bprm->sh_bang || bprm->security)
		/* The security field should already be set properly. */
		return 0;

	tsec = current->security;
	isec = inode->i_security;

	/* Default to the current task SID. */
	bprm->security = (void *)tsec->sid;

#ifndef _SELINUX_KERNEL_PATCH_
	if (!isec)
		return 0;
#endif

	if (tsec->in_sid[0]) {
		newsid = tsec->in_sid[0];
	} else {
		/* Check for a default transition on this program. */
		rc = security_transition_sid(tsec->sid, isec->sid, SECCLASS_PROCESS, 
					     &newsid);
		if (rc)
			return rc;
	}

	AVC_AUDIT_DATA_INIT(&ad, FS);
	ad.u.fs.dentry = bprm->file->f_dentry;

	if (bprm->file->f_vfsmnt->mnt_flags & MNT_NOSUID)
		newsid = tsec->sid;

        if (tsec->sid == newsid) {
		rc = avc_has_perm_ref_audit(tsec->sid, isec->sid, 
					    SECCLASS_FILE, FILE__EXECUTE_NO_TRANS, 
					    &isec->avcr, &ad);
		if (rc)
			return rc;
	} else {
		/* Check permissions for the transition. */
		rc = avc_has_perm_audit(tsec->sid, newsid, 
					SECCLASS_PROCESS, PROCESS__TRANSITION, 
					&ad);
		if (rc)
			return rc;

		rc = avc_has_perm_ref_audit(newsid, isec->sid, 
					    SECCLASS_FILE, FILE__ENTRYPOINT, 
					    &isec->avcr, &ad);
		if (rc)
			return rc;

		/* Set the security field to the new SID. */
		bprm->security = (void*) newsid;
	}

	return 0;
}

static int selinux_bprm_check_security (struct linux_binprm *bprm)
{
	return 0;
}

static int selinux_bprm_secureexec (struct linux_binprm *bprm)
{       
        struct task_security_struct *tsec = current->security;
        
        /* Note that we must include the legacy uid/gid test below
           to retain it, as the new userland will simply use the
           value passed by AT_SECURE to decide whether to enable
           secure mode. */
        return (tsec->sid != tsec->osid ||
                current->euid != current->uid ||
                current->egid != current->gid);
}       

static void selinux_bprm_free_security(struct linux_binprm *bprm)
{
	/* Nothing to do - not dynamically allocated. */
	return;
}

/* Derived from fs/exec.c:flush_old_files. */
static inline void flush_unauthorized_files(struct files_struct * files)
{
	avc_audit_data_t ad;
	struct file *file;
	long j = -1;

	AVC_AUDIT_DATA_INIT(&ad,FS);

	read_lock(&files->file_lock);
	for (;;) {
		unsigned long set, i;

		j++;
		i = j * __NFDBITS;
		if (i >= files->max_fds || i >= files->max_fdset)
			break;
		set = files->open_fds->fds_bits[j];
		if (!set)
			continue;
		read_unlock(&files->file_lock);
		for ( ; set ; i++,set >>= 1) {
			if (set & 1) {
				file = fget(i);
				if (!file)
					continue;
				if (file_has_perm(current,
						  file, 
						  file_to_av(file))) 
					sys_close(i);
				fput(file);
			}
		}
		read_lock(&files->file_lock);

	}
	read_unlock(&files->file_lock);
}

static void selinux_bprm_compute_creds(struct linux_binprm *bprm)
{
	struct task_security_struct *tsec, *psec;
	security_id_t sid;
	int rc;

	secondary_ops->bprm_compute_creds(bprm);

	tsec = current->security;

	sid = (security_id_t)bprm->security;
	if (!sid)
		sid = tsec->sid;

	tsec->osid = tsec->sid;
	if (tsec->sid != sid) {
		/* Check for shared state.  If not ok, leave SID
		   unchanged and kill. */
		if ((atomic_read(&current->fs->count) > 1 ||
		     atomic_read(&current->files->count) > 1 ||
		     atomic_read(&current->sig->count) > 1)) {
			rc = avc_has_perm(tsec->sid, sid, 
					  SECCLASS_PROCESS, PROCESS__SHARE);
			if (rc) {
				force_sig(SIGKILL, current);
				return;
			}
		}

		/* Check for ptracing, and update the task SID if ok.
		   Otherwise, leave SID unchanged and kill. */
		task_lock(current);
		if (current->ptrace & PT_PTRACED) {
			psec = current->p_pptr->security;
			rc = avc_has_perm(psec->sid, sid, 
					  SECCLASS_PROCESS, PROCESS__PTRACE);
			if (rc) {
				task_unlock(current);
				force_sig(SIGKILL, current);
				return;
			}
		}
		tsec->sid = sid;
		task_unlock(current);

		/* Close files for which the new task SID is not authorized. */
		flush_unauthorized_files(current->files);

		/* Wake up parent if it is waiting so that it can recheck
		   wait permission to the new task SID. */
		wake_up_interruptible(&current->p_pptr->wait_chldexit);
	}
}

/* superblock security operations */

static int selinux_sb_alloc_security(struct super_block *sb)
{
	return superblock_alloc_security(sb);
}

static void selinux_sb_free_security(struct super_block *sb)
{
	superblock_free_security(sb);
}

static int selinux_sb_kern_mount(struct super_block *sb)
{
	avc_audit_data_t ad;
	int rc;

	rc = superblock_doinit(sb);
	if (rc)
		return rc;

	AVC_AUDIT_DATA_INIT(&ad,FS);
	ad.u.fs.dentry = sb->s_root;
	return superblock_has_perm(current, sb, FILESYSTEM__MOUNT, &ad);
}

static int selinux_sb_statfs(struct super_block *sb)
{
	struct task_security_struct *tsec;
	struct superblock_security_struct *sbsec;

	/* This hook function would simply call superblock_has_perm,
	   but it also needs to save the sb SID to support
	   statfs_secure, so we just inline superblock_has_perm. */

	tsec = current->security;
	sbsec = sb->s_security;
#ifndef _SELINUX_KERNEL_PATCH_
	if (!sbsec)
		return 0;
#endif
	tsec->out_sid[0] = sbsec->sid;

	return avc_has_perm(tsec->sid, sbsec->sid, SECCLASS_FILESYSTEM, 
			    FILESYSTEM__GETATTR);
}

static int selinux_mount(char * dev_name, 
                         struct nameidata *nd, 
                         char * type, 
                         unsigned long flags, 
                         void * data)	
{
	if (flags & MS_REMOUNT)
		return superblock_has_perm(current, nd->mnt->mnt_sb, FILESYSTEM__REMOUNT, NULL);
	else
		return dentry_has_perm(current, nd->dentry, FILE__MOUNTON, NULL);
}

static int selinux_check_sb(struct vfsmount *mnt, struct nameidata *nd)
{
	/* Handled by sb_kern_mount. */
	return 0;
}

static int selinux_umount(struct vfsmount *mnt, int flags)
{
	return superblock_has_perm(current,mnt->mnt_sb, FILESYSTEM__UNMOUNT,NULL);
}

static void selinux_umount_close(struct vfsmount *mnt)
{
	struct super_block *sb = mnt->mnt_sb;
	struct superblock_security_struct *sbsec;

	sbsec = sb->s_security;

	if (sbsec && sbsec->uses_psids)
		psid_release(sb);
	return;
}

static void selinux_umount_busy	(struct vfsmount *mnt)
{
	struct super_block *sb = mnt->mnt_sb;
	struct superblock_security_struct *sbsec;

	sbsec = sb->s_security;

	if (sbsec && sbsec->uses_psids)
		psid_init(sb);

	return;
}

static void selinux_post_remount(struct vfsmount *mnt, unsigned long flags, void *data)
{
	struct super_block *sb = mnt->mnt_sb;
	struct superblock_security_struct *sbsec;

	sbsec = sb->s_security;

	if (sbsec && sbsec->uses_psids)
		psid_remount(sb);
}

static void selinux_post_mountroot	(void)
{
	struct list_head *obj;

	printk(KERN_INFO "SELinux:  Completing initialization.\n");

	/* Load the security policy. */
	if (sys_security_load_policy(NULL, 0)) {
#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
		if (selinux_enforcing) 
			panic("SELinux:  Unable to load the policy.\n");
		else 
			printk("SELinux:  Unable to load the policy.\n");
#else
		panic("SELinux:  Unable to load the policy.\n");
#endif
		return;
	}

	/* Label any objects created prior to the policy load. */
	list_for_each(obj, &superblock_security_head) {
		struct superblock_security_struct *sbsec = list_entry(obj, struct superblock_security_struct, list);
		superblock_doinit(sbsec->sb);
	}
	list_for_each(obj, &inode_security_head) {
		struct inode_security_struct *isec = list_entry(obj, struct inode_security_struct, list);
		inode_doinit(isec->inode);
	}
}

static void selinux_post_addmount(struct vfsmount *mnt, struct nameidata *nd)
{
	/* Handled by sb_kern_mount. */
	return;
}

static int selinux_pivotroot (struct nameidata *old_nd, struct nameidata *new_nd)
{
	return 0;
}

static void selinux_post_pivotroot (struct nameidata *old_nd, struct nameidata *new_nd)
{
	selinux_post_mountroot();
	return;
}

/* inode security operations */

static int selinux_inode_alloc_security(struct inode *inode)
{
	return inode_alloc_security(inode);
}

static void selinux_inode_free_security(struct inode *inode)
{
	inode_free_security(inode);
}

static int selinux_inode_create(struct inode *dir, struct dentry *dentry, int mask)
{
	return may_create(dir, dentry, SECCLASS_FILE);
}

static void selinux_inode_post_create(struct inode *dir, struct dentry *dentry, int mask)
{
	post_create(dir, dentry);
}

static int selinux_inode_link(struct dentry *old_dentry, struct inode *dir, struct dentry *new_dentry)
{
	int rc;

	rc = secondary_ops->inode_link(old_dentry,dir,new_dentry);
	if (rc)
		return rc;
	return may_link(dir, old_dentry, MAY_LINK);
}

static void selinux_inode_post_link(struct dentry *old_dentry, struct inode *inode, struct dentry *new_dentry)
{
	return;
}

static int selinux_inode_unlink(struct inode *dir, struct dentry *dentry)
{
	return may_link(dir, dentry, MAY_UNLINK);
}

static int selinux_inode_symlink(struct inode *dir, struct dentry *dentry, const char *name)
{
	return may_create(dir, dentry, SECCLASS_LNK_FILE);
}

static void selinux_inode_post_symlink(struct inode *dir, struct dentry *dentry, const char *name)
{
	post_create(dir, dentry);
}

static int selinux_inode_mkdir(struct inode *dir, struct dentry *dentry, int mask)
{
	return may_create(dir, dentry, SECCLASS_DIR);
}

static void selinux_inode_post_mkdir(struct inode *dir, struct dentry *dentry, int mask)
{
	post_create(dir, dentry);
}

static int selinux_inode_rmdir(struct inode *dir, struct dentry *dentry)
{
	return may_link(dir, dentry, MAY_RMDIR);
}

static int selinux_inode_mknod(struct inode *dir, struct dentry *dentry, int mode, dev_t dev)
{
	return may_create(dir, dentry, inode_mode_to_security_class(mode));
}

static void selinux_inode_post_mknod(struct inode *dir, struct dentry *dentry, int mode, dev_t dev)
{
	post_create(dir, dentry);
}

static int selinux_inode_rename(struct inode *old_inode, struct dentry *old_dentry, struct inode *new_inode, struct dentry *new_dentry)
{
	return may_rename(old_inode, old_dentry, new_inode, new_dentry);
}

static void selinux_inode_post_rename(struct inode *old_inode, struct dentry *old_dentry, struct inode *new_inode, struct dentry *new_dentry)
{
	return;
}

static int selinux_inode_readlink(struct dentry *dentry)
{
	return dentry_has_perm(current, dentry, FILE__READ, NULL);
}

static int selinux_inode_follow_link(struct dentry *dentry, struct nameidata *nameidata)
{
	int rc;

	rc = secondary_ops->inode_follow_link(dentry,nameidata);
	if (rc)
		return rc;
	return dentry_has_perm(current, dentry, FILE__READ, NULL);
}

static int selinux_inode_permission(struct inode *inode, int mask)
{
	if (!mask) {
		/* No permission to check.  Existence test. */
		return 0;
	}

	return inode_has_perm(current, inode, 
			       file_mask_to_av(inode->i_mode, mask), NULL, NULL);
}

static int selinux_inode_revalidate(struct dentry *inode)
{
	/* Unused until we deal with NFS. */
	return 0;
}

static int selinux_inode_setattr(struct dentry *dentry, struct iattr *iattr)
{
	if (iattr->ia_valid & (ATTR_MODE | ATTR_UID | ATTR_GID | 
			       ATTR_ATIME_SET | ATTR_MTIME_SET))
		return dentry_has_perm(current, dentry, FILE__SETATTR, NULL);

	return dentry_has_perm(current, dentry, FILE__WRITE, NULL);
}

static int selinux_inode_stat(struct inode *inode)
{
	struct task_security_struct *tsec;
	struct inode_security_struct *isec;

	tsec = current->security;
	isec = inode->i_security;
#ifndef _SELINUX_KERNEL_PATCH_
	if (!isec)
		return 0;
#endif
	tsec->out_sid[0] = isec->sid;

	return inode_has_perm(current, inode, FILE__GETATTR, NULL, NULL);
}

static void selinux_inode_delete(struct inode *inode)
{
	struct super_block *sb = inode->i_sb; 
	if (sb) {
		struct superblock_security_struct *sbsec = sb->s_security;

		if (!sbsec || sbsec->magic != SELINUX_MAGIC || 
		    !sbsec->initialized)
			return;

		if (sbsec->uses_psids)
			clear_psid(inode);
	}
}

/* For now, simply use the existing [gs]etattr permissions between
   the current process and the target file for the *xattr operations.
   We will likely define new permissions later to distinguish these operations.
   If SELinux is changed to use extended attributes for file security contexts,
   then we will need to apply similar checking to setxattr as in 
   syscalls.c:chsid_common when 'name' corresponds to the SELinux security
   context, and we will need to impose stronger restrictions on removexattr. */

static int selinux_inode_setxattr (struct dentry *dentry, char *name, void *value, size_t size, int flags)
{
	return dentry_has_perm(current, dentry, FILE__SETATTR, NULL);
}

static int selinux_inode_getxattr (struct dentry *dentry, char *name)
{
	return dentry_has_perm(current, dentry, FILE__GETATTR, NULL);
}

static int selinux_inode_listxattr (struct dentry *dentry)
{
	return dentry_has_perm(current, dentry, FILE__GETATTR, NULL);
}

static int selinux_inode_removexattr (struct dentry *dentry, char *name)
{
	return dentry_has_perm(current, dentry, FILE__SETATTR, NULL);
}

/* file security operations */

static int selinux_file_permission(struct file *file, int mask)
{
	struct inode *inode = file->f_dentry->d_inode;

	if (!mask) {
		/* No permission to check.  Existence test. */
		return 0;
	}

	/* file_mask_to_av won't add FILE__WRITE if MAY_APPEND is set */
	if ((file->f_flags & O_APPEND) && (mask & MAY_WRITE))
		mask |= MAY_APPEND; 

	return file_has_perm(current, file, 
			     file_mask_to_av(inode->i_mode, mask));
}

static int selinux_file_alloc_security(struct file *file)
{
	return file_alloc_security(file);
}

static void selinux_file_free_security(struct file *file)
{
	file_free_security(file);
}

static int selinux_file_ioctl(struct file *file, unsigned int cmd, 
			      unsigned long arg)
{
	int error = 0;

	switch (cmd) {
		case FIONREAD:
		/* fall through */
		case FIBMAP:
		/* fall through */
		case FIGETBSZ:
		/* fall through */
		case EXT2_IOC_GETFLAGS:
		/* fall through */
		case EXT2_IOC_GETVERSION:
			error = file_has_perm(current, file, FILE__GETATTR);
			break;

		case EXT2_IOC_SETFLAGS:
		/* fall through */
		case EXT2_IOC_SETVERSION:
			error = file_has_perm(current, file, FILE__SETATTR);
			break;

		/* sys_ioctl() checks */
		case FIONBIO:
		/* fall through */
		case FIOASYNC:
			error = file_has_perm(current, file, 0);
			break;

	        case KDSKBENT:
	        case KDSKBSENT:
		  	if (!capable(CAP_SYS_TTY_CONFIG))
				error = -EPERM;
			break;

		/* default case assumes that the command will go
		 * to the file's ioctl() function.
		 */
		default:
			error = file_has_perm(current, file, FILE__IOCTL);
			
	}
	return error;
}

static int selinux_file_mmap(struct file *file, unsigned long prot,
			     unsigned long flags)
{
	access_vector_t av;

	if (file) {
		/* read access is always possible with a mapping */
		av = FILE__READ;

		/* write access only matters if the mapping is shared */
		if ((flags & MAP_TYPE) == MAP_SHARED && (prot & PROT_WRITE)) 
			av |= FILE__WRITE;

		if (prot & PROT_EXEC) 
			av |= FILE__EXECUTE;

		return file_has_perm(current, file, av);
	}
	return 0;
}

static int selinux_file_mprotect(struct vm_area_struct *vma, 
				 unsigned long prot)
{
	return selinux_file_mmap(vma->vm_file, prot, vma->vm_flags);
}

static int selinux_file_lock(struct file *file, unsigned int cmd)
{
	return file_has_perm(current, file, FILE__LOCK);
}

static int selinux_file_fcntl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	int err = 0;

	switch (cmd) {
	        case F_SETFL: 
			if (!file->f_dentry || !file->f_dentry->d_inode) {
				err = -EINVAL;
				break;
			}

			if ((file->f_flags & O_APPEND) && !(arg & O_APPEND)) {
				err = file_has_perm(current, file,FILE__WRITE);
				break;
			}
			/* fall through */
	        case F_SETOWN:
	        case F_SETSIG:
	        case F_GETFL:
	        case F_GETOWN:
	        case F_GETSIG:
			/* Just check FD__USE permission */
			err = file_has_perm(current, file, 0);
			break;
		case F_GETLK:
		case F_SETLK:
	        case F_SETLKW: 
	        case F_GETLK64:
		case F_SETLK64:
	        case F_SETLKW64: 
			if (!file->f_dentry || !file->f_dentry->d_inode) {
				err = -EINVAL;
				break;
			}
			err = file_has_perm(current, file, FILE__LOCK);
			break;
	}

	return err;
}

static int selinux_file_set_fowner(struct file *file)
{
	struct task_security_struct *tsec;
	struct file_security_struct *fsec;

	tsec = current->security;
	fsec = file->f_security;
	fsec->fown_sid = tsec->sid;

	return 0;
}

static int selinux_file_send_sigiotask(struct task_struct *tsk, 
				       struct fown_struct *fown, 
				       int fd, int reason)
{
        struct file *file;
	access_vector_t perm;
	struct task_security_struct *tsec;
	struct file_security_struct *fsec;

	/* struct fown_struct is never outside the context of a struct file */
        file = (struct file *)((long)fown - offsetof(struct file,f_owner));

	tsec = tsk->security;
	fsec = file->f_security;

	if (!fown->signum) 
		perm = signal_to_av(SIGIO); /* as per send_sigio_to_task */
	else
		perm = signal_to_av(fown->signum);

	return avc_has_perm(fsec->fown_sid, tsec->sid, 
			    SECCLASS_PROCESS, perm);
}

static int selinux_file_receive(struct file *file)
{
	return file_has_perm(current, file, file_to_av(file));
}

/* task security operations */

static int selinux_task_create(unsigned long clone_flags)
{
	return task_has_perm(current, current, PROCESS__FORK);
}

static int selinux_task_alloc_security(struct task_struct *tsk)
{
	struct task_security_struct *tsec1, *tsec2;
	int rc;

	tsec1 = current->security;

	rc = task_alloc_security(tsk);
	if (rc)
		return rc;
	tsec2 = tsk->security;

	tsec2->osid = tsec1->osid;
	tsec2->sid = tsec1->sid;

	return 0;
}

static void selinux_task_free_security(struct task_struct *tsk)
{
	task_free_security(tsk);
}

static int selinux_task_setuid(uid_t id0, uid_t id1, uid_t id2, int flags)
{
	/* Since setuid only affects the current process, and
	   since the SELinux controls are not based on the Linux
	   identity attributes, SELinux does not need to control
	   this operation.  However, SELinux does control the use 
	   of the CAP_SETUID and CAP_SETGID capabilities using the
	   capable hook. */
	return 0;
}

static int selinux_task_post_setuid(uid_t id0, uid_t id1, uid_t id2, int flags)
{
	return secondary_ops->task_post_setuid(id0,id1,id2,flags);
}

static int selinux_task_setgid(gid_t id0, gid_t id1, gid_t id2, int flags)
{
	/* See the comment for setuid above. */
	return 0;
}

static int selinux_task_setpgid(struct task_struct *p, pid_t pgid)
{
	return task_has_perm(current, p, PROCESS__SETPGID);
}

static int selinux_task_getpgid(struct task_struct *p)
{
	return task_has_perm(current, p, PROCESS__GETPGID);
}

static int selinux_task_getsid(struct task_struct *p)
{
	return task_has_perm(current, p, PROCESS__GETSESSION);
}

static int selinux_task_setgroups(int gidsetsize, gid_t *grouplist)
{
	/* See the comment for setuid above. */
	return 0;
}

static int selinux_task_setnice(struct task_struct *p, int nice)
{
	return task_has_perm(current,p, PROCESS__SETSCHED);
}

static int selinux_task_setrlimit(unsigned int resource, struct rlimit *new_rlim)
{
	/* SELinux does not currently provide a process 
	   resource limit policy based on security contexts.
	   It does control the use of the CAP_SYS_RESOURCE capability
	   using the capable hook. */
	return 0;
}

static int selinux_task_setscheduler(struct task_struct *p, int policy, struct sched_param *lp)
{
	avc_audit_data_t ad;
	struct task_security_struct *tsec1, *tsec2;

	tsec1 = current->security;
	tsec2 = p->security;

	/* No auditing from the setscheduler hook, since the runqueue lock
	   is held and the system will deadlock if we try to log an audit
	   message. */
	AVC_AUDIT_DATA_INIT(&ad, DONTAUDIT);
	return avc_has_perm_ref_audit(tsec1->sid, tsec2->sid, 
				      SECCLASS_PROCESS, PROCESS__SETSCHED, &tsec2->avcr, &ad);
}

static int selinux_task_getscheduler(struct task_struct *p)
{
	return task_has_perm(current, p, PROCESS__GETSCHED);
}

static int selinux_task_kill(struct task_struct *p, struct siginfo *info, int sig)
{
	access_vector_t perm;

	if (info && ((unsigned long)info == 1 || SI_FROMKERNEL(info)))
		return 0;

	if (!sig) 
		perm = PROCESS__SIGNULL; /* null signal; existence test */
	else 
		perm = signal_to_av(sig);
	
	return task_has_perm(current, p, perm);
}

static int selinux_task_prctl(int option, 
			      unsigned long arg2, 
			      unsigned long arg3,
			      unsigned long arg4, 
			      unsigned long arg5)
{
	/* The current prctl operations do not appear to require 
	   any SELinux controls since they merely observe or modify 
	   the state of the current process. */
	return 0;
}

static int selinux_task_wait(struct task_struct *p)
{
	access_vector_t perm;

	perm = signal_to_av(p->exit_signal);

	return task_has_perm(p, current, perm);
}

static void selinux_task_kmod_set_label(void)
{
  	struct task_security_struct *tsec;

	secondary_ops->task_kmod_set_label();

	tsec = current->security;
	tsec->osid = tsec->sid;
	tsec->sid = SECINITSID_KERNEL;
	return;
}

static void selinux_task_reparent_to_init(struct task_struct *p)
{
  	struct task_security_struct *tsec;

	secondary_ops->task_reparent_to_init(p);

	tsec = p->security;
	tsec->osid = tsec->sid;
	tsec->sid = SECINITSID_KERNEL; 
	return;
}

#ifdef CONFIG_SECURITY_NETWORK

static void skb_copy_security(struct skb_security_struct *new,
                              struct skb_security_struct *old)
{
	if (!old || old->magic != SELINUX_MAGIC)
		return;
	if (!new || new->magic != SELINUX_MAGIC)
		return;
	  
	new->serial    = old->serial;
	new->ssid      = old->ssid;
	new->msid      = old->msid;
	new->dsid      = old->dsid;
	new->opts      = old->opts;
	new->mapped    = old->mapped;
}

#ifdef CONFIG_NETFILTER

/*
 * Helper function for the selinux_ip_preroute_last hook and
 * selinux_ip_input_last hook.  
 */
static unsigned int selinux_ip_input_helper(struct sk_buff *skb, 
					    struct skb_security_struct *ssec,
					    struct net_device *dev) 
{
	struct iphdr *iph = skb->nh.iph;
	struct netdev_security_struct *nsec;
	access_vector_t netif_perm, node_perm;
	security_id_t node_sid;
	avc_audit_data_t ad;
	int err;

	err = netdev_precondition(dev);
	if (err == 0) 
		return NF_ACCEPT;
	if (err < 0) 
		return NF_DROP;
	nsec = dev->security;

	/* If the skb is unlabeled at this point, then initially assign 
	   the default message SID for the receiving network interface. 
	   This may be later overridden due to an explicit label option. */
	if (ssec->ssid == SECINITSID_UNLABELED ) 
		ssec->ssid = nsec->default_msg_sid;
	if (ssec->msid == SECINITSID_UNLABELED)
		ssec->msid = ssec->ssid;

	err = security_node_sid(PF_INET, &iph->saddr, sizeof(iph->saddr),
				&node_sid);
	if (err) 
		return NF_DROP;

	switch (iph->protocol) {
	case IPPROTO_UDP:
		netif_perm = NETIF__UDP_RECV;
		node_perm =   NODE__UDP_RECV;
		break;
	case IPPROTO_TCP:
		netif_perm = NETIF__TCP_RECV;
		node_perm =   NODE__TCP_RECV;
		break;
	default:
		netif_perm = NETIF__RAWIP_RECV;
		node_perm =   NODE__RAWIP_RECV;
		break;
	}

	AVC_AUDIT_DATA_INIT(&ad,NET);
	ad.u.net.netif = dev->name;
	ad.u.net.skb = skb;

	err = avc_has_perm_ref_audit(ssec->msid, nsec->sid, SECCLASS_NETIF, 
				     netif_perm, &nsec->avcr, &ad);
	if (err)
		return NF_DROP;

	err =  avc_has_perm_audit(ssec->msid, node_sid, SECCLASS_NODE, 
				  node_perm, &ad);
	if (err)
		return NF_DROP;

	return NF_ACCEPT;
}

static unsigned int selinux_ip_preroute_last(unsigned int hooknum, 
					     struct sk_buff **pskb,
					     const struct net_device *in,
					     const struct net_device *out,
					     int (*okfn)(struct sk_buff *))
{
	struct sk_buff *skb = *pskb;
	struct skb_security_struct *ssec;

	ssec = skb->lsm_security;

	return selinux_ip_input_helper(skb, ssec, (struct net_device*)in);
}

static unsigned int selinux_ip_input_first(unsigned int hooknum, 
					   struct sk_buff **pskb,
					   const struct net_device *in, 
					   const struct net_device *out,
					   int (*okfn)(struct sk_buff *))
{
	return nsid_ip_map_input(hooknum, pskb, in, out, okfn);
}

static unsigned int selinux_ip_input_last(unsigned int hooknum, 
					  struct sk_buff **pskb,
					  const struct net_device *in, 
					  const struct net_device *out,
					  int (*okfn)(struct sk_buff *))
{
	struct sk_buff *skb = *pskb;
	struct skb_security_struct *ssec;

	ssec = skb->lsm_security;

	if ( (!ssec->opts) || (selopt_get(ssec, SELOPT_BYPASS)) ) {
		/* Packet did not have a CIPSO label, or is bypassed.
		   Checking handled by selinux_ip_preroute_last. */
		return NF_ACCEPT;
	}

	/* Packet had a CIPSO label, which was mapped during
	   selinux_ip_input_first.  Check against the mapped label. */
	return selinux_ip_input_helper(skb, ssec, (struct net_device*)in);
}

static void selinux_skb_set_owner_w(struct sk_buff *skb, struct sock *sk);

/*
 * Add labels here, so packet filter can see labeled packets in the output
 * chain.  Note that IP header needs to be validated on the output hook.
 */
static unsigned int selinux_ip_output_first(unsigned int hooknum, 
					    struct sk_buff **pskb,
					    const struct net_device *in,
					    const struct net_device *out,
					    int (*okfn)(struct sk_buff *))
{
	struct sk_buff *skb = *pskb;
	struct skb_security_struct *ssec;

	ssec = skb->lsm_security;

	if (ssec->ssid == SECINITSID_UNLABELED) {
		const struct iphdr *ip = skb->nh.iph;

		if (!skb->sk && ip->protocol == IPPROTO_IGMP) {
			ssec->ssid = ssec->msid = SECINITSID_IGMP_PACKET;
		} else if (!skb->sk && ip->protocol == IPPROTO_ICMP) {
			ssec->ssid = ssec->msid = SECINITSID_ICMP_SOCKET;
		} else if (skb->sk) {
			selinux_skb_set_owner_w(skb, skb->sk);
		}
	}

	if (ssec->invalid) {
		printk("%s: dropping invalid skb\n", __FUNCTION__);
		return NF_DROP;
	}

	return nsid_ip_label_output(hooknum, pskb, in, out, okfn);
}

static unsigned int selinux_ip_postroute_last(unsigned int hooknum, 
					      struct sk_buff **pskb,
					      const struct net_device *in,
					      const struct net_device *out,
					      int (*okfn)(struct sk_buff *))
{
	struct sk_buff *skb = *pskb;
	struct net_device *dev = (struct net_device*)out;
	struct netdev_security_struct *nsec;
	struct skb_security_struct *ssec;
	avc_audit_data_t ad;
	struct iphdr *iph = skb->nh.iph;
	access_vector_t netif_perm;
	access_vector_t node_perm;
	security_id_t node_sid;	
	int err;

	err = security_node_sid(PF_INET, &iph->daddr, sizeof(iph->daddr),
				&node_sid);
	if (err) {
		return NF_DROP;
	}

	err = netdev_precondition(dev);
	if (err == 0)
		return NF_ACCEPT;
	if (err < 0) {
		return NF_DROP;
	}
	nsec = dev->security;

	ssec = skb->lsm_security;

	if (ssec->ssid == SECINITSID_UNLABELED) {
		const struct iphdr *ip = skb->nh.iph;

		if (!skb->sk && ip->protocol == IPPROTO_IGMP) {
			ssec->ssid = ssec->msid = SECINITSID_IGMP_PACKET;
		} else if (!skb->sk && ip->protocol == IPPROTO_ICMP) {
			ssec->ssid = ssec->msid = SECINITSID_ICMP_SOCKET;
		}
	}

	switch (iph->protocol) {
	case IPPROTO_UDP:
		netif_perm = NETIF__UDP_SEND;
		node_perm =   NODE__UDP_SEND;
		break;
	case IPPROTO_TCP:
		netif_perm = NETIF__TCP_SEND;
		node_perm =   NODE__TCP_SEND;
		break;
	default:
		netif_perm = NETIF__RAWIP_SEND;
		node_perm =   NODE__RAWIP_SEND;
		break;
	}

	AVC_AUDIT_DATA_INIT(&ad,NET);
	ad.u.net.netif = dev->name;
	ad.u.net.skb = skb;

	err = avc_has_perm_ref_audit(ssec->msid, nsec->sid, SECCLASS_NETIF, 
				     netif_perm, &nsec->avcr, &ad);
	if (err)
		return NF_DROP;

	err =  avc_has_perm_audit(ssec->msid, node_sid, SECCLASS_NODE, 
				  node_perm, &ad);
	if (err)
		return NF_DROP;

	return NF_ACCEPT;
}

#endif	/* CONFIG_NETFILTER */

static void selinux_ip_fragment(struct sk_buff *newskb, 
				const struct sk_buff *oldskb)
{
	skb_copy_security(newskb->lsm_security, oldskb->lsm_security);
	return;
}

static int selinux_ip_defragment(struct sk_buff *skb) 
{
	return nsid_ip_defragment(skb);
}

static void selinux_ip_decapsulate(struct sk_buff *skb) 
{
	return;
}

static void selinux_ip_encapsulate(struct sk_buff *skb)
{
	return;
}

static int selinux_ip_decode_options(struct sk_buff *skb,
                                     const char *optptr, unsigned char **pp_ptr)
{
	return nsid_ip_decode_options(skb, optptr, pp_ptr);
}

static void selinux_netdev_unregister(struct net_device *dev)
{
	netdev_free_security(dev);
}

/* socket security operations */
static int socket_has_perm(struct task_struct *task, struct socket *sock,
			   access_vector_t perms)
{
	struct inode_security_struct *isec;
	struct task_security_struct *tsec;
	avc_audit_data_t ad;
	int err;

	tsec = task->security;
	isec = SOCK_INODE(sock)->i_security;
	
	AVC_AUDIT_DATA_INIT(&ad,NET);
	ad.u.net.sk = sock->sk;
	err = avc_has_perm_ref_audit(tsec->sid, isec->sid, isec->sclass,
				     perms, &isec->avcr, &ad);

	return err;
}

static int selinux_socket_create(int family, int type, int protocol)
{
	int err;
	struct task_security_struct *tsec;
	security_id_t tsid;

	tsec = current->security;

	tsid = extsocket_create(tsec);

	err = avc_has_perm(tsec->sid, tsid,
			   socket_type_to_security_class(family, type),
			   SOCKET__CREATE);

	return err;
}

static void selinux_socket_post_create(struct socket *sock, int family, 
				      int type, int protocol)
{
	int err;
	struct inode_security_struct *isec;
	struct task_security_struct *tsec;

	err = inode_doinit(SOCK_INODE(sock));
	if (err < 0)
		return;
	isec = SOCK_INODE(sock)->i_security;

	tsec = current->security;

	isec->sclass = socket_type_to_security_class(family, type);

	isec->sid = extsocket_create(tsec);

	/* Set the SID for the sock linked to the socket */
	if(sock->sk) {
		struct sock_security_struct *sksec;
		sksec = sock->sk->security;
		sksec->sid = isec->sid;
	}
	return;
}

static int selinux_socket_bind(struct socket *sock, struct sockaddr *address,
			       int addrlen)
{
	int err;

	err = socket_has_perm(current, sock, SOCKET__BIND);
	if (err)
		return err;

	/*
	 * If PF_INET, check name_bind permission for the port.
	 */
	if (sock->sk->family == PF_INET) {
		struct inode_security_struct *isec;
		struct task_security_struct *tsec;
		avc_audit_data_t ad;
		struct sockaddr_in *addr = (struct sockaddr_in *)address;
		unsigned short snum = ntohs(addr->sin_port);
		struct sock *sk = sock->sk;
		security_id_t sid;

		tsec = current->security;
		isec = SOCK_INODE(sock)->i_security;

		if (snum&&(snum < max(PROT_SOCK,ip_local_port_range_0) ||
			   snum > ip_local_port_range_1)) {
			err = security_port_sid(sk->family, sk->type,
						sk->protocol, snum, &sid);
			if (err)
				return err;
			AVC_AUDIT_DATA_INIT(&ad,NET);
			ad.u.net.port = snum;
			err = avc_has_perm_audit(isec->sid, sid,
						 isec->sclass,
						 SOCKET__NAME_BIND, &ad);
			if (err)
				return err;
		}
	}

	return 0;
}

static int selinux_socket_connect(struct socket *sock, 
				  struct sockaddr *address,
				  int addrlen)
{
	int err;
	struct sock *sk = sock->sk;
	avc_audit_data_t ad;
	struct task_security_struct *tsec;
	struct inode_security_struct *isec;

	isec = SOCK_INODE(sock)->i_security;

	tsec = current->security;

	AVC_AUDIT_DATA_INIT(&ad, NET);
	ad.u.net.sk = sk;
	err = avc_has_perm_ref_audit(tsec->sid, isec->sid, isec->sclass,
				     SOCKET__CONNECT, &isec->avcr, &ad);
	if (err) 
		return err;

	return extsocket_connect(sock, address, addrlen, tsec, isec, &ad);
}

static int selinux_socket_listen(struct socket *sock, int backlog)
{
	int err;
	struct task_security_struct *tsec;
	struct inode_security_struct *isec;
	avc_audit_data_t ad;

	tsec = current->security;

	isec = SOCK_INODE(sock)->i_security;

	AVC_AUDIT_DATA_INIT(&ad, NET);
	ad.u.net.sk = sock->sk;

	err = avc_has_perm_ref_audit(tsec->sid, isec->sid, isec->sclass,
				     SOCKET__LISTEN, &isec->avcr, &ad);
	if (err)
		return err;

	return extsocket_listen(sock, tsec, isec, &ad);
}

static int selinux_socket_accept(struct socket *sock, struct socket *newsock)
{
	int err;
	struct task_security_struct *tsec;
	struct inode_security_struct *isec;
	struct inode_security_struct *newisec;
	avc_audit_data_t ad;

	tsec = current->security;

	isec = SOCK_INODE(sock)->i_security;

	AVC_AUDIT_DATA_INIT(&ad, NET);
	ad.u.net.sk = sock->sk;

	err = avc_has_perm_ref_audit(tsec->sid, isec->sid, isec->sclass,
				     SOCKET__ACCEPT, &isec->avcr, &ad);
	if (err)
		return err;

	err = inode_doinit(SOCK_INODE(newsock));
	if (err < 0)
		return err;
	newisec = SOCK_INODE(newsock)->i_security;

	newisec->sclass = isec->sclass;
	newisec->sid = isec->sid;

	extsocket_accept(isec, newisec);

	return 0;
}

static void selinux_socket_post_accept(struct socket *sock, 
				      struct socket *newsock)
{
	extsocket_post_accept(sock, newsock);
}

static int selinux_socket_sendmsg(struct socket *sock, struct msghdr *msg, 
 				  int size)
{
	struct task_security_struct *tsec;
	struct inode_security_struct *isec;
	avc_audit_data_t ad;
	struct sock *sk;
	int err;

	isec = SOCK_INODE(sock)->i_security;

	tsec = current->security;

	sk = sock->sk;

	AVC_AUDIT_DATA_INIT(&ad, NET);
	ad.u.net.sk = sk;
	err = avc_has_perm_ref_audit(tsec->sid, isec->sid, isec->sclass,
				     SOCKET__WRITE, &isec->avcr, &ad);
	if (err)
		return err;

	err = extsocket_sendmsg(sock, msg, tsec, isec, &ad);
	if (err)
		return err;

	nsid_sock_sendmsg(sock->sk);
	return 0;
}

static int selinux_socket_recvmsg(struct socket *sock, struct msghdr *msg, 
				  int size, int flags)
{
	struct inode_security_struct *isec;
	struct task_security_struct *tsec;
	avc_audit_data_t ad;
	int err;

	isec = SOCK_INODE(sock)->i_security;
	tsec = current->security;

	AVC_AUDIT_DATA_INIT(&ad,NET);
	ad.u.net.sk = sock->sk;
	err = avc_has_perm_ref_audit(tsec->sid, isec->sid, isec->sclass,
				     SOCKET__READ, &isec->avcr, &ad);
	if (err)
		return err;

	extsocket_recvmsg(sock, tsec, isec);

	return 0;
}

static int selinux_socket_getsockname(struct socket *sock)
{
	struct inode_security_struct *isec;
	struct task_security_struct *tsec;
	avc_audit_data_t ad;
	int err;

	tsec = current->security;
	isec = SOCK_INODE(sock)->i_security;

	AVC_AUDIT_DATA_INIT(&ad,NET);
	ad.u.net.sk = sock->sk;
	err = avc_has_perm_ref_audit(tsec->sid, isec->sid, isec->sclass,
				     SOCKET__GETATTR, &isec->avcr, &ad);
	if (err)
		return err;

	extsocket_getsockname(tsec, isec);
	return 0;
}

static int selinux_socket_getpeername(struct socket *sock)
{
	struct inode_security_struct *isec;
	struct task_security_struct *tsec;
	avc_audit_data_t ad;
	int err;

	tsec = current->security;
	isec = SOCK_INODE(sock)->i_security;
	
	AVC_AUDIT_DATA_INIT(&ad,NET);
	ad.u.net.sk = sock->sk;
	err = avc_has_perm_ref_audit(tsec->sid, isec->sid, isec->sclass,
				     SOCKET__GETATTR, &isec->avcr, &ad);
	if (err)
		return err;

	extsocket_getpeername(tsec, isec);
	return 0;
}

static int selinux_socket_setsockopt(struct socket *sock,int level,int optname)
{
	return socket_has_perm(current, sock, SOCKET__SETOPT);
}

static int selinux_socket_getsockopt(struct socket *sock, int level, 
				     int optname)
{
	return socket_has_perm(current, sock, SOCKET__GETOPT);
}

static int selinux_socket_shutdown(struct socket *sock, int how)
{
	return socket_has_perm(current, sock, SOCKET__SHUTDOWN);
}

#if SELINUX_NET_DEBUG
static void hexdump(u8 *x, int len)
{
	int i;

	printk("[ ");
	for (i = 0; i < len; i++)
		printk("%x ", x[i]);
	printk("]\n");
}

static void debug_skbuff(struct sock *sk, struct sk_buff *skb)
{
	struct iphdr *ip = skb->nh.iph;
	struct tcphdr *th = NULL;
	unsigned char *data = skb->data;
	unsigned int len = skb->len-skb->data_len;

	printk("debug_skbuff::\n");
	nf_dump_skb(PF_INET, skb); 

	printk("skb:: len=%d, data_len=%d\n", skb->len, skb->data_len);
	if (ip->protocol == IPPROTO_TCP) {
		th = skb->h.th;
		printk("tcp:: seq=%u, ack_seq=%u flags: %s%s%s%s%s%s%s%s\n",
		       th->seq, th->ack_seq,
		       th->fin?" FIN":"", th->syn?" SYN":"", th->rst?" RST":"",
		       th->psh?" PSH":"", th->ack?" ACK":"", th->urg?" URG":"",
		       th->ece?" ECE":"", th->cwr?" CWR":"");
	}

	if (!sk) {
		printk("debug_skbuff: null skb->sk\n");
	} else {
		printk("sock:: state=%d family=%d reuse=%d pair=%p\n",
		       sk->state, sk->family, sk->reuse, sk->pair);
	}

	if (data) {
		printk("data (len=%d):: ", len);
		hexdump((u8 *)data, len);
	}
	
	return;
}
#endif /* SELINUX_NET_DEBUG */

static int selinux_socket_sock_alloc_security(struct sock *sk, int gfp_mask)
{
	sk->security = NULL;
	return sock_alloc_security(sk, gfp_mask);
}

static void selinux_socket_sock_free_security(struct sock *sk)
{
	sock_free_security(sk);
}

static int selinux_socket_sock_rcv_skb(struct sock *sk, struct sk_buff *skb)
{
	avc_audit_data_t ad;
	struct socket *sock;
	struct net_device *dev;
	struct netdev_security_struct *nsec;
	struct inode_security_struct *isec;
	struct skb_security_struct *ssec;
	int err;

	if (sk->state == TCP_TIME_WAIT) {
		/* The sk is actually a tcp_tw_bucket, so 
		   there is no sk->socket to use. */
		/* debug_skbuff(sk, skb); */
		return 0;
	}

	sock = sk->socket;
	if (!sock) {
		/*
		 * TCP control messages don't always have a socket.
		 * TBD/cvance - should we still perform a recvfrom check?
		 */
		return 0;
	}

	if (!SOCK_INODE(sock)) {
		return 0;
	}

	isec = SOCK_INODE(sock)->i_security;
	if (!isec) 
		return 0;

	dev = skb->dev;
	if (!dev) {
		return 0;
	}
	err = netdev_precondition(dev);
	if (err <= 0) 
		return err;
	nsec = dev->security;
	
	ssec = skb->lsm_security;

	/* If the skb is unlabeled at this point, then assign 
	   the default message SID for the receiving network interface.
	   Ordinarily, this should be handled by the ip_preroute_last hook,
	   but it is possible that the kernel was configured w/o the 
	   IP hooks. Display a warning the first few times this occurs. */
	if( ssec->ssid == SECINITSID_UNLABELED ) {
		ssec->ssid = nsec->default_msg_sid;
		ssec->msid = ssec->ssid;
	}

	AVC_AUDIT_DATA_INIT(&ad,NET);
	ad.u.net.netif = skb->dev->name;
	ad.u.net.skb = skb;

	err = avc_has_perm_ref_audit(isec->sid, ssec->ssid,
				     isec->sclass, SOCKET__RECVFROM, 
				     &isec->avcr, &ad);
	if (err)
		return err;

	if (isec->sclass == SECCLASS_TCP_SOCKET) {
		struct tcphdr *th = skb->h.th;

		switch (sk->state) {
		case TCP_LISTEN:
			if (th->syn) {
				err = avc_has_perm_ref_audit(isec->sid,
						      ssec->ssid,
						      isec->sclass, 
						      TCP_SOCKET__ACCEPTFROM, 
						      &isec->avcr, &ad);
				if (err)
					return err;
			}
			break;
		case TCP_SYN_SENT:
			/*
			 * if we see th->ack, check connectto,
			 * if th->syn (but not th->ack), check connectto
			 * 
			 * Kernel Comment: "We see SYN without ACK. It is
			 * attempt of simultaneous connect with crossed SYNs.
			 * Particularly, it can be connect to self."
			 */
			if (!th->rst && (th->ack || th->syn)) {
				err = avc_has_perm_ref_audit(isec->sid, 
						      ssec->ssid,
						      isec->sclass, 
						      TCP_SOCKET__CONNECTTO, 
						      &isec->avcr, &ad);
				if (err)
					return err;
			}
			break;
		}
	}

	return extsocket_sock_rcv_skb(sk, skb, isec, ssec, &ad);
}

static int selinux_open_request_alloc_security(struct open_request *req)
{
	req->security = NULL;
	return extsocket_open_request_alloc_security(req);
}

static void selinux_open_request_free_security(struct open_request *req)
{
	extsocket_open_request_free_security(req);
}

static void selinux_tcp_connection_request(struct sock *sk, 
					   struct sk_buff *skb, 
					   struct open_request *req)
{
	extsocket_tcp_connection_request(sk, skb, req);
}

static void selinux_tcp_synack(struct sock *sk, struct sk_buff *skb,
			       struct open_request *req)
{
	extsocket_tcp_synack(sk, skb, req);
}

static void selinux_tcp_create_openreq_child(struct sock *sk, 
					     struct sock *newsk,
					     struct sk_buff *skb,
					     struct open_request *req)
{
	struct inode_security_struct *isec = NULL;
	struct sock_security_struct *newsksec;

	newsksec = newsk->security;

	/* 
	 * If the existing (listening) sock has a socket structure, then
	 * use it to obtain the security SID of the new sock. Otherwise,
	 * use the default TCP socket SID, but this case shouldn't happen.
	 */
	if (sk->socket) {
		isec = SOCK_INODE(sk->socket)->i_security;
		newsksec->sid = isec->sid;
	} else {
		newsksec->sid = SECINITSID_TCP_SOCKET;
	}

	extsocket_tcp_create_openreq_child(newsksec, isec, skb, req);
}

static int selinux_socket_unix_stream_connect(struct socket *sock, 
					      struct socket *other,
					      struct sock *newsk)
{
	struct inode_security_struct *isec;
	struct inode_security_struct *other_isec;
	avc_audit_data_t ad;
	int err;

	if (sock == NULL) {
		printk("socket_unix_stream_connect:: sock is NULL!\n");
		return 0;
	}
	if (other == NULL) {
		printk("socket_unix_stream_connect:: other is NULL!\n");
		return 0;
	}
	if (newsk == NULL) {
		printk("socket_unix_stream_connect:: newsk is NULL!\n");
		return 0;
	}

	isec = SOCK_INODE(sock)->i_security;
	other_isec = SOCK_INODE(other)->i_security;
	
	AVC_AUDIT_DATA_INIT(&ad,NET);
	ad.u.net.sk = other->sk;

	err = avc_has_perm_ref_audit(isec->sid, other_isec->sid,
				     isec->sclass,
				     UNIX_STREAM_SOCKET__CONNECTTO,
				     &other_isec->avcr, &ad);
	if (err)
		return err;

	return extsocket_unix_stream_connect(isec, other_isec, newsk, &ad);
}

static int selinux_socket_unix_may_send(struct socket *sock, 
					struct socket *other)
{
	struct inode_security_struct *isec;
	struct inode_security_struct *other_isec;
	avc_audit_data_t ad;
	int err;

	isec = SOCK_INODE(sock)->i_security;
	other_isec = SOCK_INODE(other)->i_security;

	AVC_AUDIT_DATA_INIT(&ad,NET);
	ad.u.net.sk = other->sk;

	err = avc_has_perm_ref_audit(isec->sid, other_isec->sid,
				     isec->sclass,
				     SOCKET__SENDTO,
				     &other_isec->avcr, &ad);
	if (err)
		return err;

	return extsocket_unix_may_send(isec, other_isec, &ad);
}

#endif

static spinlock_t ipc_alloc_lock = SPIN_LOCK_UNLOCKED;

static int ipc_alloc_security(struct task_struct *task, 
			      struct kern_ipc_perm *perm,
			      security_class_t sclass)
{
	struct task_security_struct *tsec = task->security;
	struct ipc_security_struct *isec, *new_isec;

	new_isec = kmalloc(sizeof(struct ipc_security_struct), GFP_KERNEL);
	if (!new_isec) 
		return -ENOMEM;

	spin_lock(&ipc_alloc_lock);
	isec = perm->security;
	if (isec && isec->magic == SELINUX_MAGIC) {
		spin_unlock(&ipc_alloc_lock);
		kfree(new_isec);
		return 0;
	}
	isec = new_isec;

	memset(isec, 0, sizeof(struct ipc_security_struct));
	isec->magic = SELINUX_MAGIC;
	isec->sclass = sclass;
	isec->ipc_perm = perm;
	list_add(&isec->list, &ipc_security_head);
	perm->security = isec;

	if (tsec) {
		if (tsec->in_sid[0])
			isec->sid = tsec->in_sid[0];
		else
			isec->sid = tsec->sid;
	} else {
		isec->sid = SECINITSID_UNLABELED;
	}

	spin_unlock(&ipc_alloc_lock);
	return 0;
}

static void ipc_free_security(struct kern_ipc_perm *perm)
{
	struct ipc_security_struct *isec = perm->security;
	if (!isec || isec->magic != SELINUX_MAGIC)
		return;

	perm->security = NULL;
	spin_lock(&ipc_alloc_lock);
	list_del(&isec->list);
	spin_unlock(&ipc_alloc_lock);
	kfree(isec);
}

static spinlock_t msg_alloc_lock = SPIN_LOCK_UNLOCKED;

static int msg_msg_alloc_security(struct msg_msg *msg)
{
	struct msg_security_struct *msec, *new_msec;

	new_msec = kmalloc(sizeof(struct msg_security_struct), GFP_KERNEL);
	if (!new_msec) 
		return -ENOMEM;

	spin_lock(&msg_alloc_lock);
	msec = msg->security;
	if (msec && msec->magic == SELINUX_MAGIC) {
		spin_unlock(&msg_alloc_lock);
		kfree(new_msec);
		return 0;
	}
	msec = new_msec;

	memset(msec, 0, sizeof(struct msg_security_struct));
	msec->magic = SELINUX_MAGIC;
	msec->msg = msg;
	list_add(&msec->list, &msg_security_head);
	msec->sid = SECINITSID_UNLABELED;
	msg->security = msec;

	spin_unlock(&msg_alloc_lock);
	return 0;
}

static void msg_msg_free_security(struct msg_msg *msg)
{
	struct msg_security_struct *msec = msg->security;
	if (!msec || msec->magic != SELINUX_MAGIC)
		return;

	msg->security = NULL;
	spin_lock(&msg_alloc_lock);
	list_del(&msec->list);
	spin_unlock(&msg_alloc_lock);
	kfree(msec);
}

static int ipc_has_perm(struct kern_ipc_perm *ipc_perms, 
			security_class_t sclass, access_vector_t perms)
{
	struct task_security_struct *tsec;
	struct ipc_security_struct *isec;
	avc_audit_data_t ad;

	tsec = current->security;
	isec = ipc_perms->security;

	AVC_AUDIT_DATA_INIT(&ad, IPC);
	ad.u.ipc_id = ipc_perms->key;

	return avc_has_perm_ref_audit(tsec->sid, isec->sid, sclass,
				      perms, &isec->avcr, &ad);
}

static void ipc_savesid(struct kern_ipc_perm *ipc_perms, 
			security_class_t sclass)
{
	struct task_security_struct *tsec;
	struct ipc_security_struct *isec;

	tsec = current->security;
	isec = ipc_perms->security;
	
	tsec->out_sid[0] = isec->sid;

	return;
}

static int selinux_msg_msg_alloc_security(struct msg_msg *msg)
{
	return msg_msg_alloc_security(msg);
}

static void selinux_msg_msg_free_security(struct msg_msg *msg)
{
	return msg_msg_free_security(msg);
}

/* message queue security operations */
static int selinux_msg_queue_alloc_security(struct msg_queue *msq)
{
	struct task_security_struct *tsec;
	struct ipc_security_struct *isec;
	avc_audit_data_t ad;
	int rc;

	rc = ipc_alloc_security(current, &msq->q_perm, SECCLASS_MSGQ);
	if (rc)
		return rc;

	tsec = current->security;
	isec = msq->q_perm.security;

	AVC_AUDIT_DATA_INIT(&ad, IPC);
 	ad.u.ipc_id = msq->q_perm.key;

	rc = avc_has_perm_ref_audit(tsec->sid, isec->sid, SECCLASS_MSGQ,
				    MSGQ__CREATE, &isec->avcr, &ad);
	if (rc) {
		ipc_free_security(&msq->q_perm);
		return rc;
	}
	return 0;
}

static void selinux_msg_queue_free_security(struct msg_queue *msq)
{
	ipc_free_security(&msq->q_perm);
}

static int selinux_msg_queue_associate(struct msg_queue *msq, int msqflg)
{
	struct task_security_struct *tsec;
	struct ipc_security_struct *isec;
	avc_audit_data_t ad;

	tsec = current->security;
	isec = msq->q_perm.security;

	/*
	 * If using msgget_secure and a sid was specified, but the key
	 * already has a different sid.
	 */
	if (tsec->in_sid[0] && tsec->in_sid[0] != isec->sid) {
		return -EACCES;
	}

	AVC_AUDIT_DATA_INIT(&ad, IPC);
	ad.u.ipc_id = msq->q_perm.key;

	return avc_has_perm_ref_audit(tsec->sid, isec->sid, SECCLASS_MSGQ,
				      MSGQ__ASSOCIATE, &isec->avcr, &ad);
}

static int selinux_msg_queue_msgctl(struct msg_queue *msq, int cmd)
{
	int err;
	int perms;

	switch(cmd) {
	case IPC_INFO:
	case MSG_INFO:
		/* No specific object, just general system-wide information. */
		return task_has_system(current, SYSTEM__IPC_INFO);
	case IPC_STAT:
	case MSG_STAT:
		perms = MSGQ__GETATTR | MSGQ__ASSOCIATE;
		break;
	case IPC_SET:
		perms = MSGQ__SETATTR;
		break;
	case IPC_RMID:
		perms = MSGQ__DESTROY;
		break;
	default:
		return 0;
	}

	err = ipc_has_perm(&msq->q_perm, SECCLASS_MSGQ, perms);
	if (!err && cmd == IPC_STAT)
		ipc_savesid(&msq->q_perm, SECCLASS_MSGQ);

	return err;
}

static int selinux_msg_queue_msgsnd(struct msg_queue *msq, struct msg_msg *msg,
				    int msqflg)
{
	struct task_security_struct *tsec;
	struct ipc_security_struct *isec;
	struct msg_security_struct *msec;
	avc_audit_data_t ad;
	int rc;

	tsec = current->security;
	isec = msq->q_perm.security;
	msec = msg->security;

	/*
	 * First time through, need to assign label to the message
	 */
	if (msec->sid == SECINITSID_UNLABELED) {
		/*
		 * Compute new sid based on current process and
		 * message queue this message will be stored in
		 */
		if (tsec->in_sid[0]) {
			msec->sid = tsec->in_sid[0];
		} else {
			rc = security_transition_sid(tsec->sid,
						      isec->sid,
						      SECCLASS_MSG,
						      &msec->sid);
			if (rc)
				return rc;
		}
	}

	AVC_AUDIT_DATA_INIT(&ad, IPC);
	ad.u.ipc_id = msq->q_perm.key;

	/* Can this process write to the queue? */
	rc = avc_has_perm_ref_audit(tsec->sid, isec->sid, SECCLASS_MSGQ,
				      MSGQ__WRITE, &isec->avcr, &ad);
	if (!rc)
		/* Can this process send the message */
		rc = avc_has_perm_ref_audit(tsec->sid, msec->sid,
					     SECCLASS_MSG, MSG__SEND,
					     &msec->avcr, &ad);
	if (!rc)
		/* Can the message be put in the queue? */
		rc = avc_has_perm_ref_audit(msec->sid, isec->sid,
					     SECCLASS_MSGQ, MSGQ__ENQUEUE,
					     &isec->avcr, &ad);

	return rc;
}

static int selinux_msg_queue_msgrcv(struct msg_queue *msq, struct msg_msg *msg,
				    struct task_struct *target, 
				    long type, int mode)
{
	struct task_security_struct *tsec;
	struct ipc_security_struct *isec;
	struct msg_security_struct *msec;
	avc_audit_data_t ad;
	int rc;

	tsec = target->security;
	isec = msq->q_perm.security;
	msec = msg->security;

	tsec->out_sid[0] = SECSID_NULL;

	AVC_AUDIT_DATA_INIT(&ad, IPC);
 	ad.u.ipc_id = msq->q_perm.key; 
	
	if (tsec->in_sid[0] && tsec->in_sid[0] != msec->sid) {
		return -EACCES;
	}

	rc = avc_has_perm_ref_audit(tsec->sid, isec->sid, 
				    SECCLASS_MSGQ, MSGQ__READ, 
				    &isec->avcr, &ad);
	if (!rc)
		rc = avc_has_perm_ref_audit(tsec->sid, msec->sid, 
					    SECCLASS_MSG, MSG__RECEIVE, 
					    &msec->avcr, &ad);
	if (!rc)
		tsec->out_sid[0] = msec->sid;

	return rc;
}

/* Shared Memory security operations */
static int selinux_shm_alloc_security(struct shmid_kernel *shp)
{
	struct task_security_struct *tsec;
	struct ipc_security_struct *isec;
	avc_audit_data_t ad;
	int rc;

	rc = ipc_alloc_security(current, &shp->shm_perm, SECCLASS_SHM);
	if (rc)
		return rc;

	tsec = current->security;
	isec = shp->shm_perm.security;

	AVC_AUDIT_DATA_INIT(&ad, IPC);
 	ad.u.ipc_id = shp->shm_perm.key; 

	rc = avc_has_perm_ref_audit(tsec->sid, isec->sid, SECCLASS_SHM,
				    SHM__CREATE, &isec->avcr, &ad);
	if (rc) {
		ipc_free_security(&shp->shm_perm);
		return rc;
	}
	return 0;
}

static void selinux_shm_free_security(struct shmid_kernel *shp)
{
	ipc_free_security(&shp->shm_perm);
}

static int selinux_shm_associate(struct shmid_kernel *shp, int shmflg)
{
	struct task_security_struct *tsec;
	struct ipc_security_struct *isec;
	avc_audit_data_t ad;

	tsec = current->security;
	isec = shp->shm_perm.security;

	/*
	 * If using msgget_secure and a sid was specified, but the key
	 * already has a different sid.
	 */
	if (tsec->in_sid[0] && tsec->in_sid[0] != isec->sid) {
		return -EACCES;
	}

	AVC_AUDIT_DATA_INIT(&ad, IPC);
	ad.u.ipc_id = shp->shm_perm.key;

	return avc_has_perm_ref_audit(tsec->sid, isec->sid, SECCLASS_SHM,
				      SHM__ASSOCIATE, &isec->avcr, &ad);
}

/* Note, at this point, shp is locked down */
static int selinux_shm_shmctl(struct shmid_kernel *shp, int cmd)
{
	int perms;
	int err;

	switch(cmd) {
	case IPC_INFO:
	case SHM_INFO:
		/* No specific object, just general system-wide information. */
		return task_has_system(current, SYSTEM__IPC_INFO);
	case IPC_STAT:
	case SHM_STAT:
		perms = SHM__GETATTR | SHM__ASSOCIATE;
		break;
	case IPC_SET:
		perms = SHM__SETATTR;
		break;
	case SHM_LOCK:
	case SHM_UNLOCK:
		perms = SHM__LOCK;
		break;
	case IPC_RMID:
		perms = SHM__DESTROY;
		break;
	default:
		return 0;
	}

	err = ipc_has_perm(&shp->shm_perm, SECCLASS_SHM, perms);
	if (!err && cmd == IPC_STAT)
		ipc_savesid(&shp->shm_perm, SECCLASS_SHM);

	return err;
}

static int selinux_shm_shmat(struct shmid_kernel *shp, 
			     char *shmaddr, int shmflg)
{
	access_vector_t perms;

	if (shmflg & SHM_RDONLY)
		perms = SHM__READ;
	else
		perms = SHM__READ | SHM__WRITE;

	return ipc_has_perm(&shp->shm_perm, SECCLASS_SHM, perms);
}

/* Semaphore security operations */
static int selinux_sem_alloc_security(struct sem_array *sma)
{
	struct task_security_struct *tsec;
	struct ipc_security_struct *isec;
	avc_audit_data_t ad;
	int rc;

	rc = ipc_alloc_security(current, &sma->sem_perm, SECCLASS_SEM);
	if (rc)
		return rc;

	tsec = current->security;
	isec = sma->sem_perm.security;

	AVC_AUDIT_DATA_INIT(&ad, IPC);
 	ad.u.ipc_id = sma->sem_perm.key; 

	rc = avc_has_perm_ref_audit(tsec->sid, isec->sid, SECCLASS_SEM,
				    SEM__CREATE, &isec->avcr, &ad);
	if (rc) {
		ipc_free_security(&sma->sem_perm);
		return rc;
	}
	return 0;
}

static void selinux_sem_free_security(struct sem_array *sma)
{
	ipc_free_security(&sma->sem_perm);
}

static int selinux_sem_associate(struct sem_array *sma, int semflg)
{
	struct task_security_struct *tsec;
	struct ipc_security_struct *isec;
	avc_audit_data_t ad;

	tsec = current->security;
	isec = sma->sem_perm.security;

	/*
	 * If using semget_secure and a sid was specified, but the key
	 * already has a different sid.
	 */
	if (tsec->in_sid[0] && tsec->in_sid[0] != isec->sid) {
		return -EACCES;
	}

	AVC_AUDIT_DATA_INIT(&ad, IPC);
	ad.u.ipc_id = sma->sem_perm.key;

	return avc_has_perm_ref_audit(tsec->sid, isec->sid, SECCLASS_SEM,
				      SEM__ASSOCIATE, &isec->avcr, &ad);
}

/* Note, at this point, sma is locked down */
static int selinux_sem_semctl(struct sem_array *sma, int cmd)
{
	int err;
	access_vector_t perms;

	switch(cmd) {
	case IPC_INFO:
	case SEM_INFO:
		/* No specific object, just general system-wide information. */
		return task_has_system(current, SYSTEM__IPC_INFO);
	case GETPID:
	case GETNCNT:
	case GETZCNT:
		perms = SEM__GETATTR;
		break;
	case GETVAL:
	case GETALL:
		perms = SEM__READ;
		break;
	case SETVAL:
	case SETALL:
		perms = SEM__WRITE;
		break;
	case IPC_RMID:
		perms = SEM__DESTROY;
		break;
	case IPC_SET:
		perms = SEM__SETATTR;
		break;
	case IPC_STAT:
	case SEM_STAT:
		perms = SEM__GETATTR | SEM__ASSOCIATE;
		break;
	default:
		return 0;
	}

	err = ipc_has_perm(&sma->sem_perm, SECCLASS_SEM, perms);
	if (!err && cmd == IPC_STAT)
		ipc_savesid(&sma->sem_perm, SECCLASS_SEM);

	return err;
}

static int selinux_sem_semop(struct sem_array *sma, 
			     struct sembuf *sops, unsigned nsops, int alter)
{
	access_vector_t perms;

	if (alter)
		perms = SEM__READ | SEM__WRITE;
	else
		perms = SEM__READ;

	return ipc_has_perm(&sma->sem_perm, SECCLASS_SEM, perms);
}

static int selinux_ipc_permission(struct kern_ipc_perm *ipcp, short flag)
{
	struct ipc_security_struct *isec = ipcp->security;
	security_class_t sclass = SECCLASS_IPC;
	access_vector_t av = 0;

	if (isec && isec->magic == SELINUX_MAGIC)
		sclass = isec->sclass;

	av = 0;
	if (flag & S_IRUGO)
		av |= IPC__UNIX_READ;
	if (flag & S_IWUGO)
		av |= IPC__UNIX_WRITE;

	if (av == 0)
		return 0;

	return ipc_has_perm(ipcp, sclass, av);
}

#ifdef CONFIG_SECURITY_NETWORK

static int selinux_skb_alloc_security(struct sk_buff *skb, int gfp_mask)
{
	return skb_alloc_security(skb, gfp_mask);
}

static int selinux_skb_clone(struct sk_buff *newskb, 
			      const struct sk_buff *oldskb) 
{
	struct skb_security_struct *ssec = oldskb->lsm_security;

	if (!ssec || ssec->magic != SELINUX_MAGIC)
		return -ENOMEM;

	atomic_inc(&ssec->use);	
	newskb->lsm_security = ssec;
	
	return 0;
}

static void selinux_skb_copy(struct sk_buff *newskb, 
			     const struct sk_buff *oldskb)
{
	skb_copy_security(newskb->lsm_security, oldskb->lsm_security);
	return;
}

/*
 * Copy security attributes from sending socket to skb, if appropriate.
 */
static void selinux_skb_set_owner_w(struct sk_buff *skb, struct sock *sk)
{
	struct skb_security_struct *ssec;
	struct inode *inode;
	struct inode_security_struct *isec;
	struct sock_security_struct *sksec;

	ssec = skb->lsm_security;

	sksec = sk->security;

	if (!sk->socket) {
		if (sk->family == PF_INET && sk->type == SOCK_STREAM) {
			/* TCP socket that has no associated user socket. */
			ssec->ssid = sksec->sid;
			ssec->msid = ssec->ssid;
		} else if (sk->family == PF_INET) {
			printk( "%s:  no userspace socket (family %d, type %d, protocol %d)\n", __FUNCTION__, sk->family, sk->type, sk->protocol);
		}
		return;
	}

	inode = SOCK_INODE(sk->socket);
	if (!inode) {
		printk("%s:  no inode for socket (family %d, type %d, protocol %d)\n", __FUNCTION__, sk->family, sk->type, sk->protocol);
		return;
	}

	isec = inode->i_security;
	if (!isec) {
		/* Handle kernel-created sockets, such as the 
		   TCP reset socket and the ICMP socket. */
		if (sk->family == PF_INET && sk->protocol == IPPROTO_TCP)
			ssec->ssid = SECINITSID_TCP_SOCKET;
		else if (sk->family == PF_INET && sk->protocol == IPPROTO_ICMP)
			ssec->ssid = SECINITSID_ICMP_SOCKET;
		ssec->msid = ssec->ssid;
		return;
	}

	/* Initialize SIDs from the sending socket. */
	ssec->ssid = isec->sid;
	ssec->msid = ssec->ssid;

	extsocket_skb_set_owner_w(skb, sk, ssec, isec);
	return;
}

static void selinux_skb_recv_datagram(struct sk_buff *skb, struct sock *sk,
				      unsigned flags)
{
	extsocket_skb_recv_datagram(skb, sk);
}

static void selinux_skb_free_security(struct sk_buff *skb)
{
	skb_free_security(skb);
}

#endif

/* module stacking operations */
int selinux_register_security (const char *name, struct security_operations *ops)
{
	if (secondary_ops != original_ops) {
		printk(KERN_INFO "%s:  There is already a secondary security module registered.\n", __FUNCTION__);
		return -EINVAL;
 	}
	
	secondary_ops = ops;

	printk(KERN_INFO "%s:  Registering secondary module %s\n",
	       __FUNCTION__,
	       name);

	return 0;
}

int selinux_unregister_security (const char *name, struct security_operations *ops)
{
	if (ops != secondary_ops) {
		printk (KERN_INFO "%s:  trying to unregister a security module that is not registered.\n", __FUNCTION__);
		return -EINVAL;
	}

	secondary_ops = original_ops;

	return 0;
}

static void selinux_d_instantiate (struct dentry *dentry, struct inode *inode)
{
	if (inode)
		inode_doinit_with_dentry(inode, dentry);
}

struct security_operations selinux_ops = {
	ptrace:				selinux_ptrace,
	capget:			        selinux_capget,
	capset_check:		        selinux_capset_check,
	capset_set:		        selinux_capset_set,	
	sysctl:				selinux_sysctl,
	capable:	                selinux_capable,
	sys_security:			selinux_sys_security,
	swapon:				selinux_swapon,
	swapoff:			selinux_swapoff,
	nfsservctl:			selinux_nfsservctl,
	quotactl:			selinux_quotactl,
	quota_on:			selinux_quota_on,
	bdflush:			selinux_bdflush,
	syslog:				selinux_syslog,

	netlink_send:			selinux_netlink_send,
        netlink_recv:			selinux_netlink_recv,

	bprm_alloc_security:		selinux_bprm_alloc_security,
	bprm_free_security:		selinux_bprm_free_security,
	bprm_compute_creds:		selinux_bprm_compute_creds,
	bprm_set_security:		selinux_bprm_set_security,
	bprm_check_security:		selinux_bprm_check_security,
	bprm_secureexec:		selinux_bprm_secureexec,

	sb_alloc_security:		selinux_sb_alloc_security,
	sb_free_security:		selinux_sb_free_security,
	sb_kern_mount:		        selinux_sb_kern_mount,
	sb_statfs:			selinux_sb_statfs,
	sb_mount:			selinux_mount,
	sb_check_sb:			selinux_check_sb,
	sb_umount:			selinux_umount,
	sb_umount_close:		selinux_umount_close,
	sb_umount_busy:			selinux_umount_busy,
	sb_post_remount:		selinux_post_remount,
	sb_post_mountroot:		selinux_post_mountroot,
	sb_post_addmount:		selinux_post_addmount,
	sb_pivotroot:			selinux_pivotroot,
	sb_post_pivotroot:		selinux_post_pivotroot,

	inode_alloc_security:		selinux_inode_alloc_security,
	inode_free_security:		selinux_inode_free_security,
	inode_create:			selinux_inode_create,
	inode_post_create:		selinux_inode_post_create,
	inode_link:			selinux_inode_link,
	inode_post_link:		selinux_inode_post_link,
	inode_unlink:			selinux_inode_unlink,
	inode_symlink:			selinux_inode_symlink,
	inode_post_symlink:		selinux_inode_post_symlink,
	inode_mkdir:			selinux_inode_mkdir,
	inode_post_mkdir:		selinux_inode_post_mkdir,
	inode_rmdir:			selinux_inode_rmdir,
	inode_mknod:			selinux_inode_mknod,
	inode_post_mknod:		selinux_inode_post_mknod,
	inode_rename:			selinux_inode_rename,
	inode_post_rename:		selinux_inode_post_rename,
	inode_readlink:			selinux_inode_readlink,
	inode_follow_link:		selinux_inode_follow_link,
	inode_permission:		selinux_inode_permission,
	inode_revalidate:		selinux_inode_revalidate,
	inode_setattr:			selinux_inode_setattr,
	inode_stat:			selinux_inode_stat,
	inode_delete:			selinux_inode_delete,
	inode_setxattr:			selinux_inode_setxattr,
	inode_getxattr:			selinux_inode_getxattr,
	inode_listxattr:		selinux_inode_listxattr,
	inode_removexattr:		selinux_inode_removexattr,
	
	file_permission:		selinux_file_permission,
	file_alloc_security:		selinux_file_alloc_security,
	file_free_security:		selinux_file_free_security,
	file_ioctl:			selinux_file_ioctl,
	file_mmap:			selinux_file_mmap,
	file_mprotect:			selinux_file_mprotect,
	file_lock:			selinux_file_lock,
	file_fcntl:			selinux_file_fcntl,
	file_set_fowner:		selinux_file_set_fowner,
	file_send_sigiotask:		selinux_file_send_sigiotask,
	file_receive:			selinux_file_receive,
	
	task_create:			selinux_task_create,
	task_alloc_security:		selinux_task_alloc_security,
	task_free_security:		selinux_task_free_security,
	task_setuid:			selinux_task_setuid,
	task_post_setuid:		selinux_task_post_setuid,
	task_setgid:			selinux_task_setgid,
	task_setpgid:			selinux_task_setpgid,
	task_getpgid:			selinux_task_getpgid,
	task_getsid:		        selinux_task_getsid,
	task_setgroups:			selinux_task_setgroups,
	task_setnice:			selinux_task_setnice,
	task_setrlimit:			selinux_task_setrlimit,
	task_setscheduler:		selinux_task_setscheduler,
	task_getscheduler:		selinux_task_getscheduler,
	task_kill:			selinux_task_kill,
	task_wait:			selinux_task_wait,
	task_prctl:			selinux_task_prctl,
	task_kmod_set_label:		selinux_task_kmod_set_label,
	task_reparent_to_init:		selinux_task_reparent_to_init,

	ipc_permission:			selinux_ipc_permission,
	
	msg_msg_alloc_security:		selinux_msg_msg_alloc_security,
	msg_msg_free_security:		selinux_msg_msg_free_security,
	
	msg_queue_alloc_security:	selinux_msg_queue_alloc_security,
	msg_queue_free_security:	selinux_msg_queue_free_security,
	msg_queue_associate:		selinux_msg_queue_associate,
	msg_queue_msgctl:		selinux_msg_queue_msgctl,
	msg_queue_msgsnd:		selinux_msg_queue_msgsnd,
	msg_queue_msgrcv:		selinux_msg_queue_msgrcv,
	
	shm_alloc_security:		selinux_shm_alloc_security,
	shm_free_security:		selinux_shm_free_security,
	shm_associate:			selinux_shm_associate,
	shm_shmctl:			selinux_shm_shmctl,
	shm_shmat:			selinux_shm_shmat,
	
	sem_alloc_security: 		selinux_sem_alloc_security,
	sem_free_security:  		selinux_sem_free_security,
	sem_associate:			selinux_sem_associate,
	sem_semctl:			selinux_sem_semctl,
	sem_semop:			selinux_sem_semop,
	
	register_security:		&selinux_register_security,
	unregister_security:		&selinux_unregister_security,
	
	d_instantiate:                  selinux_d_instantiate,

#ifdef CONFIG_SECURITY_NETWORK
        unix_stream_connect:		selinux_socket_unix_stream_connect,
	unix_may_send:			selinux_socket_unix_may_send,

	socket_create:			selinux_socket_create,
	socket_post_create:		selinux_socket_post_create,
	socket_bind:			selinux_socket_bind,
	socket_connect:			selinux_socket_connect,
	socket_listen:			selinux_socket_listen,
	socket_accept:			selinux_socket_accept,
	socket_post_accept:		selinux_socket_post_accept,
	socket_sendmsg:			selinux_socket_sendmsg,
	socket_recvmsg:			selinux_socket_recvmsg,
	socket_getsockname:		selinux_socket_getsockname,
	socket_getpeername:		selinux_socket_getpeername,
	socket_getsockopt:		selinux_socket_getsockopt,
	socket_setsockopt:		selinux_socket_setsockopt,
	socket_shutdown:		selinux_socket_shutdown,
	socket_sock_alloc_security:	selinux_socket_sock_alloc_security,
	socket_sock_free_security:	selinux_socket_sock_free_security,
	socket_sock_rcv_skb:		selinux_socket_sock_rcv_skb,
	open_request_alloc_security:	selinux_open_request_alloc_security,
	open_request_free_security:	selinux_open_request_free_security,
	tcp_connection_request:		selinux_tcp_connection_request,
	tcp_synack:			selinux_tcp_synack,
	tcp_create_openreq_child:	selinux_tcp_create_openreq_child,

	skb_alloc_security:		selinux_skb_alloc_security,
	skb_clone:			selinux_skb_clone,
	skb_copy:			selinux_skb_copy,
	skb_set_owner_w:		selinux_skb_set_owner_w,
	skb_recv_datagram:		selinux_skb_recv_datagram,
	skb_free_security:		selinux_skb_free_security,
	
	ip_fragment:			selinux_ip_fragment,
	ip_defragment:			selinux_ip_defragment,
	ip_encapsulate:			selinux_ip_encapsulate,
	ip_decapsulate:			selinux_ip_decapsulate,
	ip_decode_options:		selinux_ip_decode_options,
	
	netdev_unregister:		selinux_netdev_unregister,
#endif
};

extern long sys_security_selinux(struct pt_regs regs);

__init int selinux_init(void)
{
	struct task_security_struct *tsec;
#ifndef _SELINUX_KERNEL_PATCH_
	struct task_struct *tsk;
#endif

	printk(KERN_INFO "SELinux:  Initializing.\n");

#ifdef _SELINUX_KERNEL_PATCH_
	/* Set the security state for the initial task. */
	if (task_alloc_security(current))
		panic("SELinux:  Failed to initialize initial task.\n");
	tsec = current->security;
	tsec->osid = tsec->sid = SECINITSID_KERNEL;
#else
	printk(KERN_WARNING "SELinux:  The separate SELinux kernel patch was not applied.  Using SELinux without this patch is not recommended.\n");

	/* Set the security state for any existing tasks. */
	read_lock(&tasklist_lock);
	for_each_task(tsk) {
		if (task_alloc_security(tsk))
			panic("SELinux:  Failed to initialize initial task.\n");
		tsec = tsk->security;
		tsec->osid = tsec->sid = SECINITSID_KERNEL;
	}
	read_unlock(&tasklist_lock);
#endif
	
	avc_init();

	if (nsid_init())
		panic("SELinux: failed to initialize NSID API\n");

	original_ops = secondary_ops = security_ops;
	if (!secondary_ops) 
		panic ("SELinux: No initial security operations\n");
	if (register_security (&selinux_ops))
		panic("SELinux: Unable to register with kernel.\n");

	SELINUX_SYSCALL_SET(__NR_security, sys_security_selinux);

#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
	if (selinux_enforcing) {
		printk(KERN_INFO "SELinux:  Starting in enforcing mode\n");
	} else {
		printk(KERN_INFO "SELinux:  Starting in permissive mode\n");
	}
#endif
	return 0;
}

/* If the separate SELinux kernel patch has been applied, then 
   selinux_init is explicitly called from init/main.c:start_kernel() 
   immediately after the security_scaffolding_startup() call to
   initialize the LSM framework. If not, then it will be called with
   the other initcalls, and there will be superblocks and inodes that 
   will lack a properly allocated and initialized security field. */
#ifndef _SELINUX_KERNEL_PATCH_
__initcall(selinux_init);
#endif

#if defined(CONFIG_SECURITY_NETWORK) && defined(CONFIG_NETFILTER)

#define NF_IP_PRI_SELINUX_FIRST	(NF_IP_PRI_CONNTRACK + 5)
#define NF_IP_PRI_SELINUX_LAST	-NF_IP_PRI_SELINUX_FIRST

static struct nf_hook_ops selinux_ip_ops[] = {
	{ { NULL, NULL }, selinux_ip_preroute_last,
	PF_INET, NF_IP_PRE_ROUTING, NF_IP_PRI_SELINUX_LAST },

	{ { NULL, NULL }, selinux_ip_input_first,
	PF_INET, NF_IP_LOCAL_IN, NF_IP_PRI_SELINUX_FIRST },

	{ { NULL, NULL }, selinux_ip_input_last,
	PF_INET, NF_IP_LOCAL_IN, NF_IP_PRI_SELINUX_LAST },

	{ { NULL, NULL }, selinux_ip_output_first,
	PF_INET, NF_IP_LOCAL_OUT, NF_IP_PRI_SELINUX_FIRST },

	{ { NULL, NULL }, selinux_ip_postroute_last,
	PF_INET, NF_IP_POST_ROUTING, NF_IP_PRI_SELINUX_LAST }
};

static int __init selinux_nf_ip_init(void)
{
	int i;

	/* Hook registration never returns error (for now) */
	for (i = 0; i < sizeof(selinux_ip_ops)/sizeof(struct nf_hook_ops); i++)
		nf_register_hook(&selinux_ip_ops[i]);

	printk(KERN_INFO "SELinux:  Registered NetFilter hooks\n");

	return 0;
}

__initcall(selinux_nf_ip_init);

#endif	/* CONFIG_NETFILTER */

