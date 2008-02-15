/*================================================================================
                                                                               
Module Name:  ac_hooks.c

General Description: implement all necessary hooks in Access Control.

==================================================================================
 
   Copyright (C) 2005 - Motorola
 
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.
 
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
 
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  
Revision History:
                            Modification     Tracking
Author (core ID)                Date          Number      Description of Changes
------------------------    ------------    ----------   -------------------------
Wang Yang (w20619)           01/31/2005     LIBff54376   Init version     

==================================================================================*/
/*================================================================================
                                 INCLUDE FILES
================================================================================*/

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
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/spinlock.h>
#include <linux/file.h>
#include <linux/ext2_fs.h>
#include <linux/proc_fs.h>
#include <linux/kd.h>
#include <linux/dcache.h>
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
#include <linux/list.h>
#include <linux/quota.h>
#include <linux/un.h>		/* for Unix socket types */
#include <net/af_unix.h>	/* for Unix socket types */

#include <linux/mount.h>
#include <linux/namespace.h>
#include <linux/sched.h>

#include "ac_policy.h"
#include "ac_plug.h"

/* The security server must be initialized before access decisions can be provided. */
//extern int ac_initialized; it should be declared in load policy

static LIST_HEAD(task_security_head);
static LIST_HEAD(sock_security_head);
static LIST_HEAD(inode_security_head);
static LIST_HEAD(file_security_head);
//static LIST_HEAD(superblock_security_head);

int ac_enforcing = 0;

/* Original (dummy) security module. */
static struct security_operations *original_ops = NULL;
static struct security_operations *secondary_ops = NULL;

static spinlock_t task_alloc_lock = SPIN_LOCK_UNLOCKED;

extern void *sys_call_table[];

#define AC_SYSCALL_SET(x,y) sys_call_table[x] = y

static struct vfsmount *find_vfsmount(struct dentry *d) {
	struct dentry *root = dget(d->d_sb->s_root);
	struct namespace *namespace = current->namespace;
	struct list_head *head;
	struct vfsmount *mnt = NULL;

	down_read(&namespace->sem);
	list_for_each(head, &namespace->list) {
		mnt = list_entry(head, struct vfsmount, mnt_list);
		if (mnt->mnt_root == root) {
			break;	/* this is the problem spot */
		}
	}
	up_read(&namespace->sem);
	dput(root);
	return mnt;
}
/*
static char *inode_get_path(struct inode *inode)
{
    char *buffer, *path;
    struct dentry *dentry;
    struct vfsmount *mnt;

    path = NULL;   
    dentry = d_find_alias(inode);
    if (dentry) {
	mnt = find_vfsmount(dentry);
        buffer = (char*)__get_free_page(GFP_KERNEL);
        if (buffer) 
	{
	    path = d_path(dentry,mnt,buffer,PAGE_SIZE);
	    path = ac_d_path(dentry,buffer,PAGE_SIZE);
	    free_page((unsigned long)buffer);
	}
	
	dput(dentry);
    }
    return path;
}
*/
static int task_alloc_security(struct task_struct *task)
{
	struct task_security_struct *tsec, *new_tsec;

	new_tsec = kmalloc(sizeof(struct task_security_struct), GFP_KERNEL);
	if (!new_tsec) 
		return -ENOMEM; 

	spin_lock(&task_alloc_lock);
	//if has been allocated before, reture
	tsec = task->security;
	if (tsec && tsec->magic == AC_POLICY_MAGIC) {
		spin_unlock(&task_alloc_lock);
		kfree(new_tsec);
		return 0;
	}
	tsec = new_tsec;

	memset(tsec, 0, sizeof(struct task_security_struct));
	tsec->magic = AC_POLICY_MAGIC;
	tsec->task = task;
	list_add(&tsec->list, &task_security_head);
	tsec->pno = AC_DEFAULT_UNLABELED;
	task->security = tsec;

	spin_unlock(&task_alloc_lock);
	return 0;
}

static void task_free_security(struct task_struct *task)
{
	struct task_security_struct *tsec = task->security;

	if (!tsec || tsec->magic != AC_POLICY_MAGIC)
		return; 

	task->security = NULL;
	spin_lock(&task_alloc_lock);
	list_del(&tsec->list);
	spin_unlock(&task_alloc_lock);
	kfree(tsec);
}

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
	if (sksec && sksec->magic == AC_POLICY_MAGIC) {
		spin_unlock_irqrestore(&sock_alloc_lock,flags);
		kfree(new_sksec);
		return 0;
	}
	sksec = new_sksec;

	memset(sksec, 0, sizeof(struct sock_security_struct));
	sksec->magic = AC_POLICY_MAGIC;
	sksec->sk = sk;
	list_add(&sksec->list, &sock_security_head);
	sksec->rno = AC_DEFAULT_UNLABELED;
	sk->security = sksec;

	spin_unlock_irqrestore(&sock_alloc_lock, flags);
	return 0;
}

static void sock_free_security(struct sock *sk)
{
	struct sock_security_struct *sksec;
	unsigned long flags;

	sksec = sk->security;
	if (!sksec || sksec->magic != AC_POLICY_MAGIC)
		return;

	sk->security = NULL;
	spin_lock_irqsave(&sock_alloc_lock, flags);
	list_del(&sksec->list);
	spin_unlock_irqrestore(&sock_alloc_lock, flags);
	kfree(sksec);
}

static spinlock_t inode_alloc_lock = SPIN_LOCK_UNLOCKED;

static int inode_alloc_security(struct inode *inode)
{
	struct inode_security_struct *isec, *new_isec;

	new_isec = kmalloc(sizeof(struct inode_security_struct), GFP_KERNEL);
	if (!new_isec) 
		return -ENOMEM;

	spin_lock(&inode_alloc_lock);
	isec = inode->i_security;
	if (isec && isec->magic ==  AC_POLICY_MAGIC) {
		spin_unlock(&inode_alloc_lock);
		kfree(new_isec);
		return 0;
	}
	isec = new_isec;

	memset(isec, 0, sizeof(struct inode_security_struct));
	init_MUTEX(&isec->sem);
	isec->magic = AC_POLICY_MAGIC;
	isec->inode = inode;
	list_add(&isec->list, &inode_security_head);
	isec->rno = AC_DEFAULT_UNLABELED;
	//isec->sclass = SECCLASS_FILE;
	
	inode->i_security = isec;

	spin_unlock(&inode_alloc_lock);
	return 0;
}

static void inode_free_security(struct inode *inode)
{
	struct inode_security_struct *isec = inode->i_security;

	if (!isec || isec->magic != AC_POLICY_MAGIC) 
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
	struct file_security_struct *fsec, *new_fsec;

	new_fsec = kmalloc(sizeof(struct file_security_struct), GFP_ATOMIC);
	if (!new_fsec) 
		return -ENOMEM;

	spin_lock(&file_alloc_lock);
	fsec = file->f_security;
	if (fsec && fsec->magic == AC_POLICY_MAGIC) {
		spin_unlock(&file_alloc_lock);
		kfree(new_fsec);
		return 0;
	}
	fsec = new_fsec;

	memset(fsec, 0, sizeof(struct file_security_struct));
	fsec->magic = AC_POLICY_MAGIC;
	fsec->file = file;
	list_add(&fsec->list, &file_security_head);
	fsec->rno = AC_DEFAULT_UNLABELED;
	file->f_security = fsec;

	spin_unlock(&file_alloc_lock);
	return 0;
}

static void file_free_security(struct file *file)
{
	struct file_security_struct *fsec = file->f_security;

	if (!fsec || fsec->magic != AC_POLICY_MAGIC) 
		return; 

	file->f_security = NULL;
	spin_lock(&file_alloc_lock);
	list_del(&fsec->list);
	spin_unlock(&file_alloc_lock);
	kfree(fsec);
}

/*
static spinlock_t sb_alloc_lock = SPIN_LOCK_UNLOCKED;

static int superblock_alloc_security(struct super_block *sb)
{
	struct superblock_security_struct *sbsec, *new_sbsec;

	new_sbsec = kmalloc(sizeof(struct superblock_security_struct), GFP_KERNEL);
	if (!new_sbsec) 
		return -ENOMEM;

	spin_lock(&sb_alloc_lock);
	sbsec = sb->s_security;
	if (sbsec && sbsec->magic == AC_POLICY_MAGIC) {
		spin_unlock(&sb_alloc_lock);
		kfree(new_sbsec);
		return 0;
	}
	sbsec = new_sbsec;

	memset(sbsec, 0, sizeof(struct superblock_security_struct));
	init_MUTEX(&sbsec->sem);
	sbsec->magic = AC_POLICY_MAGIC;
	sbsec->sb = sb;
	list_add(&sbsec->list, &superblock_security_head);
	sbsec->rno = AC_DEFAULT_UNLABELED;
	sb->s_security = sbsec;

	spin_unlock(&sb_alloc_lock);
	return 0;
}

static void superblock_free_security(struct super_block *sb)
{
	struct superblock_security_struct *sbsec = sb->s_security;

	if (!sbsec || sbsec->magic != AC_POLICY_MAGIC) 
		return; 

	if (sbsec->uses_psids)
		psid_release(sb);

	sb->s_security = NULL;
	spin_lock(&sb_alloc_lock);
	list_del(&sbsec->list);
	spin_unlock(&sb_alloc_lock);
	kfree(sbsec);
}
*/

static inline int inode_mode_to_security_class(umode_t mode) 
{
	switch (mode & S_IFMT) {
	case S_IFDIR:
		return AC_CLASS_DIR;
	case S_IFSOCK:
	case S_IFLNK:
	case S_IFREG:		
	case S_IFBLK:
	case S_IFCHR:
	case S_IFIFO:
	default:
	  return AC_CLASS_FILE;
	}
}

static inline int socket_type_to_security_class(int family,int type)
{
	switch (family) {
	case PF_UNIX:
		/*switch (type) {
		case SOCK_STREAM:
			return AC_CLASS_UNIX_STREAM_SOCKET;
		case SOCK_DGRAM:
			return AC_CLASS_UNIX_DGRAM_SOCKET;
		}*/
	case PF_INET:
	case PF_INET6:
		/*switch (type) {
		case SOCK_STREAM:
			return AC_CLASS_TCP_SOCKET;
		case SOCK_DGRAM:
			return AC_CLASS_UDP_SOCKET;
		case SOCK_RAW:
			return AC_CLASS_RAWIP_SOCKET;
		}*/
	case PF_NETLINK:
		//return AC_CLASS_NETLINK_SOCKET;
	case PF_PACKET:
		//return AC_CLASS_PACKET_SOCKET;
	case PF_KEY:
		//return AC_CLASS_KEY_SOCKET;
        default:
	     return AC_CLASS_SOCKET;
	}
}

/*
 * Copied from fs/dcache.c:d_path and hacked up to
 * avoid need for vfsmnt, root, and rootmnt parameters.
 */
char * ac_d_path(struct dentry *dentry, 
		  char *buffer, int buflen)
{
	char * end = buffer+buflen;
	char * retval;
	int namelen;

	spin_lock(&dcache_lock);
	*--end = '\0';
	buflen--;
	if (!IS_ROOT(dentry) && list_empty(&dentry->d_hash)) {
		buflen -= 10;
		end -= 10;
		memcpy(end, " (deleted)", 10);
	}

	/* Get '/' right */
	retval = end-1;
	*retval = '/';

	for (;;) {
		struct dentry * parent;

		if (IS_ROOT(dentry)) {
			goto global_root;
		}
		parent = dentry->d_parent;
		namelen = dentry->d_name.len;
		if (!namelen)
			goto skip;
		buflen -= namelen + 1;
		if (buflen < 0)
			break;
		end -= namelen;
		memcpy(end, dentry->d_name.name, namelen);
		*--end = '/';
		retval = end;
skip:
		dentry = parent;
		if (!dentry)
			break;
	}
	spin_unlock(&dcache_lock);
	return retval;
global_root:
	namelen = dentry->d_name.len;
	buflen -= namelen;
	if (buflen >= 0) {
		retval -= namelen-1;	/* hit the slash */
		memcpy(retval, dentry->d_name.name, namelen);
	}
	spin_unlock(&dcache_lock);
	return retval;
}

/* Convert a Linux mode and permission mask to an access vector. */
static inline int file_mask_to_av(int mode, int mask) 
{
	int av = 0;

	if ((mode & S_IFMT) != S_IFDIR) {
		if (mask & MAY_EXEC) 
			av |= AC_FILE_EXECUTE;
		if (mask & MAY_READ) 
			av |= AC_FILE_READ;
		if (mask & MAY_WRITE) 
			av |= AC_FILE_WRITE;

	} else {
		if (mask & MAY_EXEC) 
			av |= AC_DIR_SEARCH;
		if (mask & MAY_WRITE) 
			av |= AC_DIR_WRITE;
		if (mask & MAY_READ) 
			av |= AC_DIR_READ;
	}

	return av;
}

/* Convert a Linux file to an access vector. */
static inline int file_to_av(struct file *file)
{
	int av = 0;

	if (file->f_mode & FMODE_READ) 
		av |= AC_FILE_READ;
	if (file->f_mode & FMODE_WRITE) {
		av |= AC_FILE_WRITE;
	}

	return av;
}

static int inode_doinit_with_dentry(struct inode *inode, struct dentry *opt_dentry);

static inline int inode_doinit(struct inode *inode) 
{
	return inode_doinit_with_dentry(inode, NULL);
}

static int superblock_doinit(struct super_block *sb) 
{
	int rc;

	//if (!ac_initialized) {
		/* Defer initialization until selinux_post_mountroot,
		   after the initial policy is loaded and the security
		   server is ready to handle calls. */
	//	return 0;
	//}
    
	/* Initialize the root inode. */
	rc = inode_doinit_with_dentry(sb->s_root->d_inode, sb->s_root);
	if (rc)
		return rc;

	return 0;
}

/* The inode's security attributes must be initialized before first use. */
static int inode_doinit_with_dentry(struct inode *inode, struct dentry *opt_dentry) 
{
	struct inode_security_struct *isec = inode->i_security;
	struct task_security_struct *tsec;
	char *buffer, *path, *fullpath = NULL;
	struct dentry *dentry;
	struct vfsmount *mnt = NULL;
    char *delimiter;
	int rc = 0;
    int len = 0;
    unsigned long ver;    

//	if (!ac_policy_init)
//	    return 0;

        tsec = current->security;
  
	if (!isec) {
		rc = inode_alloc_security(inode);
		if (rc)
			return rc;
		isec = inode->i_security;
	}

	if (!ac_policy_init)
	    return 0;

	down(&isec->sem);
	
	if(isec->initialized)
	{
		/* Already initialized. */
		up(&isec->sem);
		return 0;
	}
	
	if (inode->i_sock) {
		struct socket *sock = &inode->u.socket_i;
		if (sock->sk) {
			isec->class_no = socket_type_to_security_class(sock->sk->family, sock->sk->type);
		} 
		else {
			isec->class_no = AC_CLASS_SOCKET;
		}
	} 
	else {
		isec->class_no = inode_mode_to_security_class(inode->i_mode);
	}
	
	if (inode->i_sb == NULL) 
	{
                isec->initialized = 1;
		up(&isec->sem);
		return 0;
	}
	
	if (isec->class_no == AC_CLASS_SOCKET)
	{
		rc = ac_socket_getsid(tsec->pno, isec->class_no);
		if(rc >= 0)
		  isec->rno = rc;
	}
	else/* Obtain a SID based on the fstype, path, and class. */
	{
	  if (opt_dentry) 
		  dentry = dget(opt_dentry);
	  else
		  dentry = d_find_alias(inode);
	  if (dentry) {
		  if (!d_mountpoint(dentry))
		      mnt = find_vfsmount(dentry); 
		  buffer = (char*)__get_free_page(GFP_KERNEL);
		  if (buffer) 
		  {
			  if (d_mountpoint(dentry))
			      path = ac_d_path(dentry,buffer,PAGE_SIZE);
			  else
			      path = d_path(dentry,mnt, buffer, PAGE_SIZE);
			  if (path) 
			  {
                  ver = ac_get_version();
                  if (ver == AC_POLICY_VERSION_1)
                  {
                      rc = ac_object_getdefsid(path, AC_DEFAULT_TYPE_OBJECT);
                  }
                  else if (ver == AC_POLICY_VERSION_2)
                  {
                      delimiter = strstr(path, " (deleted)");
                      if (delimiter != NULL)
                      {
                          len = delimiter - path;
                          fullpath = (char *)kmalloc((len + 2), GFP_KERNEL);
                          if (!fullpath)
                          {
                              free_page((unsigned long)buffer);
                              dput(dentry);
                              up(&isec->sem);
                              return -ENOMEM;
                          }
    
	                      memset(fullpath, 0, (len + 2));
                          strncpy(fullpath, path, len);
                  
                          if ((isec->class_no == AC_CLASS_DIR) && (fullpath[len-1] != '/'))
                          {
                             strcat(fullpath, "/");
                          }
				          rc = ac_object_getdefsid(fullpath, AC_DEFAULT_TYPE_OBJECT);
                      }
                      else
                      {
                          if (isec->class_no == AC_CLASS_DIR)
                          {
                              fullpath = (char *)kmalloc((strlen(path) + 2), GFP_KERNEL);
                              if (!fullpath)
                              {
                                  free_page((unsigned long)buffer);
                                  dput(dentry);
                                  up(&isec->sem);
                                  return -ENOMEM;
                              }
    
	                          memset(fullpath, 0, (strlen(path)+2));
                              strcpy(fullpath, path);
                              strcat(fullpath, "/");
				              rc = ac_object_getdefsid(fullpath, AC_DEFAULT_TYPE_OBJECT);
                          }
                          else
                          {
				              rc = ac_object_getdefsid(path, AC_DEFAULT_TYPE_OBJECT);
                          }
                      }
                      if (fullpath)
                          kfree(fullpath);
                  }//end version2
                  else
                      rc = 0;
			  }//end path
			  if (rc >= 0)
		              isec->rno = rc;
			  free_page((unsigned long)buffer);
		  }
				
		  dput(dentry);
		}
	} 
	isec->initialized = 1;
	up(&isec->sem);
	return 0;
}

/* Set an inode's SID, where the inode may or may not already
   have a security structure. */
int inode_security_set_sid(struct inode *inode, int sid)
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
	isec->class_no = inode_mode_to_security_class(inode->i_mode);
	isec->rno = sid;
	isec->initialized = 1;
	up(&isec->sem);
	return 0;
}

/* Check permission of a task e.g. security checks,
   fork check, ptrace check, etc. */
int task_has_perm(struct task_struct *tsk, int class_no, int perms)
{
	struct task_security_struct *tsec;

	tsec = tsk->security;
	return ac_task_has_perm(tsec->pno, class_no, perms);
}

/* Check whether a task is allowed to use a capability. */
int task_has_capability(struct task_struct *tsk, int cap)
{
	return task_has_perm(tsk, AC_CLASS_CAPABILITY, CAP_TO_MASK(cap));
}

/* Check whether a task is allowed to use a system operation. */
int task_has_system(struct task_struct *tsk, int perms)
{
	return task_has_perm(tsk, AC_CLASS_SYSTEM, perms);
}

/* Check whether a task is allowed to use a security operation. */
int task_has_security(struct task_struct *tsk, int perms)
{
	return task_has_perm(tsk, AC_CLASS_SECURITY, perms);
}
/* Check whether a task has a particular permission to an inode. */
int inode_has_perm(struct task_struct *tsk,
		   struct inode *inode,
		   int perms)
{
	struct task_security_struct *tsec;
	struct inode_security_struct *isec;

	//inode_doinit_with_dentry(inode,NULL);
	
	tsec = tsk->security;
	isec = inode->i_security;
	
	if (!isec || !isec->initialized)
		return 0;

	return ac_check_ac_perm(tsec->pno, isec->rno, isec->class_no, perms);
}

/* Same as inode_has_perm, but pass explicit audit data containing 
   the dentry to help the auditing code to more easily generate the 
   pathname if needed. */
static inline int dentry_has_perm(struct task_struct *tsk,
				  struct dentry *dentry, 
				  int av) 
{
	struct inode *inode = dentry->d_inode;
        struct inode_security_struct *isec = inode->i_security;

	if (!isec || !isec->initialized)
	{
           inode_doinit_with_dentry(inode,dentry);
        }

	return inode_has_perm(tsk, inode, av);
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
				int av)
{
	struct task_security_struct *tsec;
	struct file_security_struct *fsec;
	struct dentry *dentry = file->f_dentry;

	tsec = tsk->security;
	fsec = file->f_security;

	/* av is zero if only checking access to the descriptor. */
	if (av)
	{
		return dentry_has_perm(tsk, dentry, av);
	}
	return 0;
}

/* Check whether a task can create a file. */
static int may_create(struct inode *dir, 
		      struct dentry *dentry, 
		      security_class_t tclass)
{
	struct task_security_struct *tsec;
	struct inode_security_struct *dsec;

	tsec = current->security;
	dsec = dir->i_security;

        if (!dsec)
		return 0;

	return ac_check_ac_perm(tsec->pno, dsec->rno, AC_CLASS_DIR, AC_DIR_SEARCH);
}

/* Check whether a task can perform a filesystem operation. */
int superblock_has_perm(struct task_struct *tsk,
			struct super_block *sb,
			access_vector_t perms)
{
	return 0;
}

/* assorted security operations  (mostly syscall interposition) */

static int ac_ptrace(struct task_struct *parent, struct task_struct *child)
{
	int rc;
    struct task_security_struct *ptsec, *tsec;

	rc = secondary_ops->ptrace(parent,child);
	if (rc)
            return rc;
    
    ptsec = parent->security;
    tsec = child->security;
    if((ptsec != NULL) && (tsec != NULL))
    {
        if(ptsec->pno == tsec->pno)
            return 0;
    }

	return task_has_perm(parent, AC_CLASS_PROCESS, AC_PROCESS_PTRACE);
}

static int ac_capget(struct task_struct *target, kernel_cap_t *effective, kernel_cap_t *inheritable, kernel_cap_t *permitted)	
{
	/* FIXME: add AC check first */

	/* standard set */
	return secondary_ops->capget(target,effective,inheritable,permitted);
}

static int ac_capset_check(struct task_struct *target, kernel_cap_t *effective, kernel_cap_t *inheritable, kernel_cap_t *permitted)	
{
	/* FIXME: add AC check first */

	/* standard set */
	 return secondary_ops->capset_check(target,effective,inheritable,permitted);
}

static void ac_capset_set(struct task_struct *target, kernel_cap_t *effective, kernel_cap_t *inheritable, kernel_cap_t *permitted) 
{
	/* FIXME: add AC check first */

	/* standard set */
	secondary_ops->capset_set(target,effective,inheritable,permitted);
	return;
}

static int ac_capable(struct task_struct *tsk, int cap)
{
	int rc;

	rc = secondary_ops->capable(tsk,cap);
	if (rc)
            return rc;
	/* FIXME: add AC check here */
	return 0;
}

static int ac_sysctl(ctl_table *table, int op)
{
	return 0;
}
	
static int ac_sys_security(unsigned int magic, unsigned int call, unsigned long *args) 
{
	/* This hook function is not used, because the Access Control
	   module replaces the generic security syscall with its
	   own call in order to have the registers on the stack
	   available for execve_secure. */
	return -ENOSYS;
}

static int ac_swapon(struct swap_info_struct *swap)
{
	return 0;
}

static int ac_swapoff(struct swap_info_struct *swap)
{
	return 0;
}

static int ac_nfsservctl(int cmd, struct nfsctl_arg *arg)
{
	return 0;
}

static int ac_quotactl(int cmds, int type, int id, struct super_block *sb)
{
	return 0;
}
	
static int ac_quota_on(struct file *f)
{
	return 0;
}

static int ac_bdflush(int func, long data)
{
	return 0;
}

static int ac_syslog(int type)
{
	int rc;

	rc = secondary_ops->syslog(type);
	if (rc)
	    return rc;

	/* FIXME: Add AC check here */
	return 0;
}

static int ac_netlink_send(struct sk_buff *skb)  
{
	return 0;
}

static int ac_netlink_recv(struct sk_buff *skb)  
{
	return 0;
}

/* binprm security operations */
static int ac_bprm_alloc_security(struct linux_binprm *bprm)
{
	int rc;
	rc = secondary_ops->bprm_alloc_security(bprm);
	if (rc)
	    return rc;

	bprm->security = NULL;
	return 0;
}

static int ac_bprm_set_security(struct linux_binprm *bprm)
{
	struct task_security_struct *tsec;
	int newsid;
	int rc;
	rc = secondary_ops->bprm_set_security(bprm);
	if (rc)
	    return rc;
	if (bprm->sh_bang || bprm->security)
		/* The security field should already be set properly. */
		return 0;

	tsec = current->security;
		
	/* Default to the current task pno. */
	bprm->security = (void *)tsec->pno;

	if (tsec->in_sid)
		newsid = tsec->in_sid;
	else
	{
	    char *buffer, *path;
            
	    path = NULL;   
            buffer = (char*)__get_free_page(GFP_KERNEL);
	    if (buffer) 
	    {
	        path = d_path(bprm->file->f_dentry,bprm->file->f_vfsmnt,buffer,PAGE_SIZE);
	        free_page((unsigned long)buffer);
	    }
	    
            if (!path)
		path = bprm->filename;
	    
	    rc = ac_object_getdefsid(path, AC_DEFAULT_TYPE_PROCESS);
	    if (rc < 0)
	        return rc;
			
	    /* Check for a default transition on this program. */
 	    rc = ac_process_getsid(tsec->pno, rc);

	    if (rc >= 0)
	        newsid = rc;
	    else		
		return rc;

	/* Check ptrace permission between the parent and 
	   the new SID for this process if this process is 
	    being traced. */
	/*	task_lock(current);
		if (current->ptrace & PT_PTRACED) {
			psec = current->p_pptr->security;
			rc = avc_has_perm(psec->sid, newsid, 
					  SECCLASS_PROCESS, PROCESS__PTRACE);
			if (rc) {
				task_unlock(current);
				return rc;
			}
		}
		task_unlock(current);
*/
		/* Set the security field to the new SID. */
	}
        bprm->security = (void*) newsid;
	

	return 0;
}

static int ac_binprm_check_security (struct linux_binprm *bprm)
{
	return 0;
}

static void ac_bprm_free_security(struct linux_binprm *bprm)
{
	/* Nothing to do - not dynamically allocated. */
	return;
}

/* Derived from fs/exec.c:flush_old_files. */
static inline void flush_unauthorized_files(struct files_struct * files)
{
	struct file *file;
	long j = -1;

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
				if (file_has_perm(current, file, file_to_av(file))) 
					sys_close(i);
				fput(file);
			}
		}
		read_lock(&files->file_lock);

	}
	read_unlock(&files->file_lock);
}

static void ac_bprm_compute_creds(struct linux_binprm *bprm)
{
        struct task_security_struct *tsec;
	int sid;

        secondary_ops->bprm_compute_creds(bprm);

	tsec = current->security;

	sid = (int)bprm->security;
	if (!sid)
		sid = tsec->pno;

	if (tsec->pno != sid) {
		task_lock(current);
		current->mm->dumpable = 0;
		
		tsec->pno = sid;
		task_unlock(current);

		flush_unauthorized_files(current->files);

		/* need to force wait permission check if parent is waiting */
		wake_up_interruptible(&current->p_pptr->wait_chldexit);
	}

	tsec->in_sid = 0;
}

static int ac_sb_alloc_security(struct super_block *sb)
{
	return 0;//superblock_alloc_security(sb);;
}

static void ac_sb_free_security(struct super_block *sb)
{
	return;//superblock_free_security(sb);
}

static int ac_sb_kern_mount(struct super_block *sb)
{
        int rc;

        rc = superblock_doinit(sb);
	if (rc)
		return rc;

	return task_has_perm(current, AC_CLASS_FILESYSTEM, AC_FILESYSTEM_MOUNT);
}

static int ac_sb_statfs(struct super_block *sb)
{
	return 0;
}

static int ac_mount(char * dev_name, 
                         struct nameidata *nd, 
                         char * type, 
                         unsigned long flags, 
                         void * data)	
{
	if (flags & MS_REMOUNT)
		//return superblock_has_perm(current, nd->mnt->mnt_sb, AC_FILESYSTEM_REMOUNT);
		return task_has_perm(current, AC_CLASS_FILESYSTEM, AC_FILESYSTEM_REMOUNT);
	else
		//return dentry_has_perm(current, nd->dentry, AC_FILESYSTEM_MOUNT);
		return task_has_perm(current, AC_CLASS_FILESYSTEM, AC_FILESYSTEM_MOUNT);
}

static int ac_check_sb(struct vfsmount *mnt, struct nameidata *nd)
{
	/* Handled by sb_kern_mount. */
	return 0;
}

static int ac_umount(struct vfsmount *mnt, int flags)
{
	//return superblock_has_perm(current,mnt->mnt_sb,AC_FILESYSTEM_UNMOUNT);
	return task_has_perm(current, AC_CLASS_FILESYSTEM, AC_FILESYSTEM_UNMOUNT);
}

static void ac_umount_close(struct vfsmount *mnt)
{
	return ;
}

static void ac_umount_busy	(struct vfsmount *mnt)
{
	return;
}

static void ac_post_remount(struct vfsmount *mnt, unsigned long flags, void *data)
{
	return;
}

static void ac_post_mountroot	(void)
{
	struct list_head *obj;

	printk(KERN_INFO "AC:  Completing initialization.\n");

	/* Load the security policy. */
	if (ac_load_policy()) 
		return;
	
	/* Label any objects created prior to the policy load. */
	list_for_each(obj, &inode_security_head) {
		struct inode_security_struct *isec = list_entry(obj, struct inode_security_struct, list);
		inode_doinit(isec->inode);
	}
}

static void ac_post_addmount(struct vfsmount *mnt, struct nameidata *nd)
{
        struct inode_security_struct *mntp_isec;
        struct inode_security_struct *root_isec;	
	
	/* Handled by sb_kern_mount. */
	if (mnt->mnt_mountpoint == nd->dentry)
        {
	    root_isec = mnt->mnt_root->d_inode->i_security;
	    mntp_isec = mnt->mnt_mountpoint->d_inode->i_security;
	    mntp_isec->initialized = 0;
	    inode_doinit(mnt->mnt_mountpoint->d_inode);
	    root_isec->rno = mntp_isec->rno;
	}
	return;
}

static int ac_pivotroot (struct nameidata *old_nd, struct nameidata *new_nd)
{
	return 0;
}

static void ac_post_pivotroot (struct nameidata *old_nd, struct nameidata *new_nd)
{
	ac_post_mountroot();
	return;
}

/* inode security operations */

static int ac_inode_alloc_security(struct inode *inode)
{
	return inode_alloc_security(inode);
}

static void ac_inode_free_security(struct inode *inode)
{
	inode_free_security(inode);
}

static int ac_inode_create(struct inode *dir, struct dentry *dentry, int mask)
{
	return may_create(dir, dentry, AC_CLASS_FILE);
}

static void ac_inode_post_create(struct inode *dir, struct dentry *dentry, int mask)
{
	return; //post_create(dir, dentry);
}

static int ac_inode_link(struct dentry *old_dentry, struct inode *dir, struct dentry *new_dentry)
{
	int rc;
	rc = secondary_ops->inode_link(old_dentry,dir,new_dentry);
	if (rc)
	    return rc;
	return 0;
}

static void ac_inode_post_link(struct dentry *old_dentry, struct inode *inode, struct dentry *new_dentry)
{
	return;
}

static int ac_inode_unlink(struct inode *dir, struct dentry *dentry)
{
	return 0;
}

static int ac_inode_symlink(struct inode *dir, struct dentry *dentry, const char *name)
{
	return 0;
}

static void ac_inode_post_symlink(struct inode *dir, struct dentry *dentry, const char *name)
{
	return;
}

static int ac_inode_mkdir(struct inode *dir, struct dentry *dentry, int mask)
{
	return may_create(dir, dentry, AC_CLASS_DIR);
}

static void ac_inode_post_mkdir(struct inode *dir, struct dentry *dentry, int mask)
{
	return;
}

static int ac_inode_rmdir(struct inode *dir, struct dentry *dentry)
{
	return 0;
}

static int ac_inode_mknod(struct inode *dir, struct dentry *dentry, int mode, dev_t dev)
{
	return may_create(dir, dentry, inode_mode_to_security_class(mode));
}

static void ac_inode_post_mknod(struct inode *dir, struct dentry *dentry, int mode, dev_t dev)
{
	return;
}

static int ac_inode_rename(struct inode *old_inode, struct dentry *old_dentry, struct inode *new_inode, struct dentry *new_dentry)
{
	return 0;
}

static void ac_inode_post_rename(struct inode *old_inode, struct dentry *old_dentry, struct inode *new_inode, struct dentry *new_dentry)
{
	return;
}

static int ac_inode_readlink(struct dentry *dentry)
{
	return 0;
}

static int ac_inode_follow_link(struct dentry *dentry, struct nameidata *nameidata)
{
	int rc;
	rc = secondary_ops->inode_follow_link(dentry,nameidata);
	if (rc)
	   return rc;
	return 0;
}

static int ac_inode_permission(struct inode *inode, int mask)
{
	if (!mask) {
		/* No permission to check.  Existence test. */
		return 0;
	}

	return inode_has_perm(current, inode, file_mask_to_av(inode->i_mode, mask));
}

static int ac_inode_revalidate(struct dentry *inode)
{
	/* Unused until we deal with NFS. */
	return 0;
}

static int ac_inode_setattr(struct dentry *dentry, struct iattr *iattr)
{
	return 0;
}

static int ac_inode_stat(struct inode *inode)
{
	return 0;
}

static void ac_inode_delete(struct inode *inode)
{
	return;
}

/* For now, simply use the existing [gs]etattr permissions between
   the current process and the target file for the *xattr operations.
   We will likely define new permissions later to distinguish these operations.
   If SELinux is changed to use extended attributes for file security contexts,
   then we will need to apply similar checking to setxattr as in 
   syscalls.c:chsid_common when 'name' corresponds to the SELinux security
   context, and we will need to impose stronger restrictions on removexattr. */

static int ac_inode_setxattr (struct dentry *dentry, char *name, void *value, size_t size, int flags)
{
	return 0;
}

static int ac_inode_getxattr (struct dentry *dentry, char *name)
{
	return 0;
}

static int ac_inode_listxattr (struct dentry *dentry)
{
	return 0;
}

static int ac_inode_removexattr (struct dentry *dentry, char *name)
{
	return 0;
}

/* file security operations */
static int ac_file_permission(struct file *file, int mask)
{
        struct inode *inode = file->f_dentry->d_inode;

	if (!mask) {
		/* No permission to check.  Existence test. */
		return 0;
	}

	return file_has_perm(current, file, 
			     file_mask_to_av(inode->i_mode, mask));
}

static int ac_file_alloc_security(struct file *file)
{
	return file_alloc_security(file);
}

static void ac_file_free_security(struct file *file)
{
	file_free_security(file);
}

static int ac_file_ioctl(struct file *file, unsigned int cmd, 
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
			//error = file_has_perm(current, file, FILE__GETATTR);
			//break;

		case EXT2_IOC_SETFLAGS:
		/* fall through */
		case EXT2_IOC_SETVERSION:
			//error = file_has_perm(current, file, FILE__SETATTR);
			//break;

		/* sys_ioctl() checks */
		case FIONBIO:
		/* fall through */
		case FIOASYNC:
			//error = file_has_perm(current, file, 0);
			//break;

	  case KDSKBENT:
	  case KDSKBSENT:
		//if (!capable(CAP_SYS_TTY_CONFIG))
		//  error = -EPERM;
		//break;

		/* default case assumes that the command will go
		 * to the file's ioctl() function.
		 */
		default:
			error = file_has_perm(current, file, AC_FILE_IOCTL);
			
	}
	return error;
}

static int ac_file_mmap(struct file *file, unsigned long prot,
			     unsigned long flags)
{
	return 0;
}

static int ac_file_mprotect(struct vm_area_struct *vma, 
				 unsigned long prot)
{
	return 0;
}

static int ac_file_lock(struct file *file, unsigned int cmd)
{
	return 0;
}

static int ac_file_fcntl(struct file *file, unsigned int cmd,
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
				err = file_has_perm(current, file, AC_FILE_WRITE);
				break;
			}
			/* fall through */
	   case F_SETOWN:
	   case F_SETSIG:
	   case F_GETFL:
	   case F_GETOWN:
	   case F_GETSIG:
			/* Just check FD__USE permission */
			//err = file_has_perm(current, file, 0);
			//break;
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
			err = file_has_perm(current, file, AC_FILE_LOCK);
			break;
	}
	return err;
}

static int ac_file_set_fowner(struct file *file)
{
	return 0;
}

static int ac_file_send_sigiotask(struct task_struct *tsk, 
				       struct fown_struct *fown, 
				       int fd, int reason)
{
   return 0;
}

static int ac_file_receive(struct file *file)
{
	return 0;
}

/* task security operations */
static int ac_task_create(unsigned long clone_flags)
{
	return 0; //task_has_perm(current, current, PROCESS__FORK);
}

static int ac_task_alloc_security(struct task_struct *tsk)
{
	struct task_security_struct *tsec1, *tsec2;
	int rc;

	tsec1 = current->security;

	rc = task_alloc_security(tsk);
	if (rc)
		return rc;
	tsec2 = tsk->security;

	tsec2->pno = tsec1->pno;

	return 0;
}

static void ac_task_free_security(struct task_struct *tsk)
{
	task_free_security(tsk);
}

static int ac_task_setuid(uid_t id0, uid_t id1, uid_t id2, int flags)
{
	/* Since setuid only affects the current process, and
	   since the SELinux controls are not based on the Linux
	   identity attributes, SELinux does not need to control
	   this operation.  However, SELinux does control the use 
	   of the CAP_SETUID and CAP_SETGID capabilities using the
	   capable hook. */
	return 0;
}

static int ac_task_post_setuid(uid_t id0, uid_t id1, uid_t id2, int flags)
{
	return secondary_ops->task_post_setuid(id0,id1,id2,flags);
}

static int ac_task_setgid(gid_t id0, gid_t id1, gid_t id2, int flags)
{
	/* See the comment for setuid above. */
	return 0;
}

static int ac_task_setpgid(struct task_struct *p, pid_t pgid)
{
	return 0;
}

static int ac_task_getpgid(struct task_struct *p)
{
	return 0;
}

static int ac_task_getsid(struct task_struct *p)
{
	return 0;
}

static int ac_task_setgroups(int gidsetsize, gid_t *grouplist)
{
	/* See the comment for setuid above. */
	return 0;
}

static int ac_task_setnice(struct task_struct *p, int nice)
{
	return 0;
}

static int ac_task_setrlimit(unsigned int resource, struct rlimit *new_rlim)
{
	/* SELinux does not currently provide a process 
	   resource limit policy based on security contexts.
	   It does control the use of the CAP_SYS_RESOURCE capability
	   using the capable hook. */
	return 0;
}

static int ac_task_setscheduler(struct task_struct *p, int policy, struct sched_param *lp)
{
	return 0;
}

static int ac_task_getscheduler(struct task_struct *p)
{
	return 0;
}

static int ac_signal_switch(int sig)
{
    switch (sig)
    {
	case SIGKILL:
	case SIGTERM:
	    return AC_PROCESS_SIGKILL;
	default:
            return 0;
    }
}

static int ac_task_kill(struct task_struct *p, struct siginfo *info, int sig)
{
	struct task_security_struct *tsec1 = p->security;
	struct task_security_struct *tsec2 = current->security;
	int ac_sig;
	
	if (info && (((unsigned long)info == 1) || (SI_FROMKERNEL(info))))
	    return 0;

	ac_sig = ac_signal_switch(sig);
	if (!ac_sig)
	    return 0;

        return ac_check_ac_perm(tsec2->pno,tsec1->pno,AC_CLASS_PROCESS,ac_sig); 
}

static int ac_task_prctl(int option, 
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

static int ac_task_wait(struct task_struct *p)
{
	return 0;
}

static void ac_task_kmod_set_label(void)
{
	struct task_security_struct *tsec;

	secondary_ops->task_kmod_set_label();

	tsec = current->security;
	tsec->pno = AC_CLASS_KERNEL;
        return;
}

static void ac_task_reparent_to_init(struct task_struct *p)
{
        struct task_security_struct *tsec;

        secondary_ops->task_reparent_to_init(p);

	tsec = p->security;
	tsec->pno = AC_CLASS_KERNEL;
	return;
}

#ifdef CONFIG_SECURITY_NETWORK

#ifdef CONFIG_SECURITY_NETFILTER

/*
 * Helper function for the ac_ip_preroute_last hook and
 * ac_ip_input_last hook.  
 */
static unsigned int ac_ip_input_helper(struct sk_buff *skb, 
					    struct skb_security_struct *ssec,
					    struct net_device *dev) 
{
	return 0;
}

static unsigned int ac_ip_preroute_last(unsigned int hooknum, 
					     struct sk_buff **pskb,
					     const struct net_device *in,
					     const struct net_device *out,
					     int (*okfn)(struct sk_buff *))
{
	return 0;
}

static unsigned int ac_ip_input_first(unsigned int hooknum, 
					   struct sk_buff **pskb,
					   const struct net_device *in, 
					   const struct net_device *out,
					   int (*okfn)(struct sk_buff *))
{
	return 0;
}

static unsigned int ac_ip_input_last(unsigned int hooknum, 
					  struct sk_buff **pskb,
					  const struct net_device *in, 
					  const struct net_device *out,
					  int (*okfn)(struct sk_buff *))
{
	return 0;
}

static void ac_skb_set_owner_w(struct sk_buff *skb, struct sock *sk)
{
	return;
}
/*
 * Add labels here, so packet filter can see labeled packets in the output
 * chain.  Note that IP header needs to be validated on the output hook.
 */
static unsigned int ac_ip_output_first(unsigned int hooknum, 
					    struct sk_buff **pskb,
					    const struct net_device *in,
					    const struct net_device *out,
					    int (*okfn)(struct sk_buff *))
{
	return 0;
}

static unsigned int ac_ip_postroute_last(unsigned int hooknum, 
					      struct sk_buff **pskb,
					      const struct net_device *in,
					      const struct net_device *out,
					      int (*okfn)(struct sk_buff *))
{
	return 0;
}

#endif	/* CONFIG_NETFILTER */

static void ac_ip_fragment(struct sk_buff *newskb, 
				const struct sk_buff *oldskb)
{
	return;
}

static int ac_ip_defragment(struct sk_buff *skb) 
{
	return 0;
}

static void ac_ip_decapsulate(struct sk_buff *skb) 
{
	return;
}

static void ac_ip_encapsulate(struct sk_buff *skb)
{
	return;
}

static int ac_ip_decode_options(struct sk_buff *skb,
                                     const char *optptr, unsigned char **pp_ptr)
{
	return 0;
}

static void ac_netdev_unregister(struct net_device *dev)
{
	return;
}

/* socket security operations */
/*
static int socket_has_perm(struct task_struct *task, struct socket *sock, int perms)
{
	struct inode_security_struct *isec;
	struct task_security_struct *tsec;
	int err;

	tsec = task->security;
	isec = SOCK_INODE(sock)->i_security;
	
	err = ac_check_ac_perm(tsec->pno, isec->rno, isec->class_no, perms);

	return err;
}*/

static int ac_socket_create(int family, int type, int protocol)
{
	return 0;
}

static void ac_socket_post_create(struct socket *sock, int family, 
				      int type, int protocol)
{
	int err;
	struct inode_security_struct *isec;

	err = inode_doinit(SOCK_INODE(sock));
	if (err < 0)
		return;
	isec = SOCK_INODE(sock)->i_security;

	/* Set the SID for the sock linked to the socket */
	if(sock->sk) {
		struct sock_security_struct *sksec;
		sksec = sock->sk->security;
		sksec->rno = isec->rno;
	}
	return;
}

static int ac_socket_bind(struct socket *sock, struct sockaddr *address,
			       int addrlen)
{
	return 0;
}

static int ac_socket_connect(struct socket *sock, 
				  struct sockaddr *address,
				  int addrlen)
{
	return 0;
}

static int ac_socket_listen(struct socket *sock, int backlog)
{
	return  0;
}

static int ac_socket_accept(struct socket *sock, struct socket *newsock)
{
        return 0;
}

static void ac_socket_post_accept(struct socket *sock, struct socket *newsock)
{
        return;
}

static int ac_socket_sendmsg(struct socket *sock, struct msghdr *msg, 
 				  int size)
{
	return 0;
}

static int ac_socket_recvmsg(struct socket *sock, struct msghdr *msg, 
				  int size, int flags)
{
	return 0;
}

static int ac_socket_getsockname(struct socket *sock)
{
	return 0;
}

static int ac_socket_getpeername(struct socket *sock)
{
  return 0;
}

static int ac_socket_setsockopt(struct socket *sock,int level,int optname)
{
	return 0;
}

static int ac_socket_getsockopt(struct socket *sock, int level, 
				     int optname)
{
	return 0;
}

static int ac_socket_shutdown(struct socket *sock, int how)
{
	return 0;
}

static int ac_socket_sock_alloc_security(struct sock *sk, int gfp_mask)
{
	sk->security = NULL;
	return sock_alloc_security(sk, gfp_mask);
}

static void ac_socket_sock_free_security(struct sock *sk)
{
	sock_free_security(sk);
}

static int ac_socket_sock_rcv_skb(struct sock *sk, struct sk_buff *skb)
{
	return 0;
}

static int ac_open_request_alloc_security(struct open_request *req)
{
	return 0;
}

static void ac_open_request_free_security(struct open_request *req)
{
	return;
}

static void ac_tcp_connection_request(struct sock *sk, 
					   struct sk_buff *skb, 
					   struct open_request *req)
{
        return;
}

static void ac_tcp_synack(struct sock *sk, struct sk_buff *skb,
			       struct open_request *req)
{
        return;
}

static void ac_tcp_create_openreq_child(struct sock *sk, 
					     struct sock *newsk,
					     struct sk_buff *skb,
					     struct open_request *req)
{
	return;
}

static int ac_socket_unix_stream_connect(struct socket *sock, 
					      struct socket *other,
					      struct sock *newsk)
{
       struct task_security_struct *isec;
       struct inode_security_struct *other_isec;

       if(sock == NULL)
       {
           printk("socket_unix_stream_connect: sock is NULL!\n");
	   return 0;
       }
       
       if(other == NULL)
       {
           printk("socket_unix_stream_connect: other is NULL!\n");
	   return 0;
       }
       if(newsk == NULL)
       {
           printk("socket_unix_stream_connect: newsk is NULL!\n");
	   return 0;
       }
       other_isec = SOCK_INODE(other)->i_security;
       if(!other_isec)
       {
           printk("w20619: other_isec is NULL!\n");
           return 0;
       }
       printf("w20619: server sid = %d\n",other_isec->rno); 

       isec = current->security;
       if(!isec)
       {
	   printk("w20619: the security of client task not initialized\n");
           return 0;
       }
       return  ac_check_ac_perm(isec->pno, other_isec->rno, AC_CLASS_SOCKET, AC_SOCKET_CONNECT);
}

static int ac_socket_unix_may_send(struct socket *sock, 
					struct socket *other)
{
       struct task_security_struct *isec;
       struct inode_security_struct *other_isec;

       if(sock == NULL)
           return 0;

       if(other == NULL)
           return 0;
       
	other_isec = SOCK_INODE(other)->i_security;
       if(!other_isec)
           return 0;

       isec = current->security;
       if(!isec)
           return 0;
       
       return  ac_check_ac_perm(isec->pno, other_isec->rno, AC_CLASS_SOCKET, AC_SOCKET_CONNECT);
}

#endif
/*
static int ipc_alloc_security(struct task_struct *task, 
			      struct kern_ipc_perm *perm,
			      security_class_t sclass)
{
	return 0;
}

static void ipc_free_security(struct kern_ipc_perm *perm)
{
        return;
}

static int msg_msg_alloc_security(struct msg_msg *msg)
{
        return 0;
}

static void msg_msg_free_security(struct msg_msg *msg)
{
        return;
}

static int ipc_has_perm(struct kern_ipc_perm *ipc_perms, 
			security_class_t sclass, access_vector_t perms)
{
        return 0;
}

static void ipc_savesid(struct kern_ipc_perm *ipc_perms, 
			security_class_t sclass)
{
        return;
}
*/
static int ac_msg_msg_alloc_security(struct msg_msg *msg)
{
	return 0;
}

static void ac_msg_msg_free_security(struct msg_msg *msg)
{
	return;
}

/* message queue security operations */
static int ac_msg_queue_alloc_security(struct msg_queue *msq)
{
	return 0;
}

static void ac_msg_queue_free_security(struct msg_queue *msq)
{
	return;
}

static int ac_msg_queue_associate(struct msg_queue *msq, int msqflg)
{
	return 0;
}

static int ac_msg_queue_msgctl(struct msg_queue *msq, int cmd)
{
	return 0;
}

static int ac_msg_queue_msgsnd(struct msg_queue *msq, struct msg_msg *msg,
				    int msqflg)
{
	return 0;
}

static int ac_msg_queue_msgrcv(struct msg_queue *msq, struct msg_msg *msg,
				    struct task_struct *target, 
				    long type, int mode)
{
	return 0;
}

/* Shared Memory security operations */
static int ac_shm_alloc_security(struct shmid_kernel *shp)
{
	return 0;
}

static void ac_shm_free_security(struct shmid_kernel *shp)
{
	return;
}

static int ac_shm_associate(struct shmid_kernel *shp, int shmflg)
{
        return 0;
}

/* Note, at this point, shp is locked down */
static int ac_shm_shmctl(struct shmid_kernel *shp, int cmd)
{
	return 0;
}

static int ac_shm_shmat(struct shmid_kernel *shp, 
			     char *shmaddr, int shmflg)
{
	return 0;
}

/* Semaphore security operations */
static int ac_sem_alloc_security(struct sem_array *sma)
{
	return 0;
}

static void ac_sem_free_security(struct sem_array *sma)
{
	return;
}

static int ac_sem_associate(struct sem_array *sma, int semflg)
{
	return 0;
}

/* Note, at this point, sma is locked down */
static int ac_sem_semctl(struct sem_array *sma, int cmd)
{
	return 0;
}

static int ac_sem_semop(struct sem_array *sma, 
			     struct sembuf *sops, unsigned nsops, int alter)
{
        return 0;
}

static int ac_ipc_permission(struct kern_ipc_perm *ipcp, short flag)
{
  return 0;
}

#ifdef CONFIG_SECURITY_NETWORK

static int ac_skb_alloc_security(struct sk_buff *skb, int gfp_mask)
{
	return 0;
}

static int ac_skb_clone(struct sk_buff *newskb, 
			      const struct sk_buff *oldskb) 
{
	return 0;
}

static void ac_skb_copy(struct sk_buff *newskb, 
			     const struct sk_buff *oldskb)
{
        return;
}

/*
 * Copy security attributes from sending socket to skb, if appropriate.
 */
static void ac_skb_set_owner_w(struct sk_buff *skb, struct sock *sk)
{
       return;
}

static void ac_skb_recv_datagram(struct sk_buff *skb, struct sock *sk,
				      unsigned flags)
{
       return;
}

static void ac_skb_free_security(struct sk_buff *skb)
{
       return;
}

#endif

/* module stacking operations */
int ac_register_security (const char *name, struct security_operations *ops)
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

int ac_unregister_security (const char *name, struct security_operations *ops)
{
	if (ops != secondary_ops) {
		printk (KERN_INFO "%s:  trying to unregister a security module that is not registered.\n", __FUNCTION__);
		return -EINVAL;
	}

	secondary_ops = original_ops;

	return 0;
}

static void ac_d_instantiate (struct dentry *dentry, struct inode *inode)
{
	if (inode)
		inode_doinit_with_dentry(inode, dentry);
}

struct security_operations ac_ops = {
	ptrace:				ac_ptrace,
	capget:			        ac_capget,
	capset_check:		        ac_capset_check,
	capset_set:		        ac_capset_set,	
	sysctl:				ac_sysctl,
	capable:	                ac_capable,
	sys_security:			ac_sys_security,
	swapon:				ac_swapon,
	swapoff:			ac_swapoff,
	nfsservctl:			ac_nfsservctl,
	quotactl:			ac_quotactl,
	quota_on:			ac_quota_on,
	bdflush:			ac_bdflush,
	syslog:				ac_syslog,
					
	netlink_send:			ac_netlink_send,
        netlink_recv:			ac_netlink_recv,
	
        bprm_alloc_security:		ac_bprm_alloc_security,
	bprm_free_security:		ac_bprm_free_security,
	bprm_compute_creds:		ac_bprm_compute_creds,
	bprm_set_security:		ac_bprm_set_security,
	bprm_check_security:		ac_binprm_check_security,

	sb_alloc_security:		ac_sb_alloc_security,
	sb_free_security:		ac_sb_free_security,
	sb_kern_mount:		        ac_sb_kern_mount,
	sb_statfs:			ac_sb_statfs,
	sb_mount:			ac_mount,
	sb_check_sb:			ac_check_sb,
	sb_umount:			ac_umount,
	sb_umount_close:		ac_umount_close,
	sb_umount_busy:			ac_umount_busy,
	sb_post_remount:		ac_post_remount,
	sb_post_mountroot:		ac_post_mountroot,
	sb_post_addmount:		ac_post_addmount,
	sb_pivotroot:			ac_pivotroot,
	sb_post_pivotroot:		ac_post_pivotroot,
 
	inode_alloc_security:		ac_inode_alloc_security,
	inode_free_security:		ac_inode_free_security,
	inode_create:			ac_inode_create,
	inode_post_create:		ac_inode_post_create,
	inode_link:			ac_inode_link,
	inode_post_link:		ac_inode_post_link,
	inode_unlink:			ac_inode_unlink,
	inode_symlink:			ac_inode_symlink,
	inode_post_symlink:		ac_inode_post_symlink,
	inode_mkdir:			ac_inode_mkdir,
	inode_post_mkdir:		ac_inode_post_mkdir,
	inode_rmdir:			ac_inode_rmdir,
	inode_mknod:			ac_inode_mknod,
	inode_post_mknod:		ac_inode_post_mknod,
	inode_rename:			ac_inode_rename,
	inode_post_rename:		ac_inode_post_rename,
	inode_readlink:			ac_inode_readlink,
	inode_follow_link:		ac_inode_follow_link,
	inode_permission:		ac_inode_permission,
	inode_revalidate:		ac_inode_revalidate,
	inode_setattr:			ac_inode_setattr,
	inode_stat:			ac_inode_stat,
	inode_delete:			ac_inode_delete,
	inode_setxattr:			ac_inode_setxattr,
	inode_getxattr:			ac_inode_getxattr,
	inode_listxattr:		ac_inode_listxattr,
	inode_removexattr:		ac_inode_removexattr,
	
	file_permission:		ac_file_permission,
	file_alloc_security:		ac_file_alloc_security,
	file_free_security:		ac_file_free_security,
	file_ioctl:			ac_file_ioctl,
	file_mmap:			ac_file_mmap,
	file_mprotect:			ac_file_mprotect,
	file_lock:			ac_file_lock,
	file_fcntl:			ac_file_fcntl,
	file_set_fowner:		ac_file_set_fowner,
	file_send_sigiotask:		ac_file_send_sigiotask,
	file_receive:			ac_file_receive,

	task_create:			ac_task_create,
	task_alloc_security:		ac_task_alloc_security,
	task_free_security:		ac_task_free_security,
	task_setuid:			ac_task_setuid,
	task_post_setuid:		ac_task_post_setuid,
	task_setgid:			ac_task_setgid,
	task_setpgid:			ac_task_setpgid,
	task_getpgid:			ac_task_getpgid,
	task_getsid:		        ac_task_getsid,
	task_setgroups:			ac_task_setgroups,
	task_setnice:			ac_task_setnice,
	task_setrlimit:			ac_task_setrlimit,
	task_setscheduler:		ac_task_setscheduler,
	task_getscheduler:		ac_task_getscheduler,
	task_kill:			ac_task_kill,
	task_wait:			ac_task_wait,
	task_prctl:			ac_task_prctl,
	task_kmod_set_label:		ac_task_kmod_set_label,
	task_reparent_to_init:		ac_task_reparent_to_init,
 
	ipc_permission:			ac_ipc_permission,
	
	msg_msg_alloc_security:		ac_msg_msg_alloc_security,
	msg_msg_free_security:		ac_msg_msg_free_security,
	
	msg_queue_alloc_security:	ac_msg_queue_alloc_security,
	msg_queue_free_security:	ac_msg_queue_free_security,
	msg_queue_associate:		ac_msg_queue_associate,
	msg_queue_msgctl:		ac_msg_queue_msgctl,
	msg_queue_msgsnd:		ac_msg_queue_msgsnd,
	msg_queue_msgrcv:		ac_msg_queue_msgrcv,
	
	shm_alloc_security:		ac_shm_alloc_security,
	shm_free_security:		ac_shm_free_security,
	shm_associate:			ac_shm_associate,
	shm_shmctl:			ac_shm_shmctl,
	shm_shmat:			ac_shm_shmat,
	
	sem_alloc_security: 		ac_sem_alloc_security,
	sem_free_security:  		ac_sem_free_security,
	sem_associate:			ac_sem_associate,
	sem_semctl:			ac_sem_semctl,
	sem_semop:			ac_sem_semop,

	register_security:		&ac_register_security,
	unregister_security:		&ac_unregister_security,

	d_instantiate:                  ac_d_instantiate,
 
#ifdef CONFIG_SECURITY_NETWORK
        unix_stream_connect:		ac_socket_unix_stream_connect,
	unix_may_send:			ac_socket_unix_may_send,
 
	socket_create:			ac_socket_create,
	socket_post_create:		ac_socket_post_create,
	socket_bind:			ac_socket_bind,
	socket_connect:			ac_socket_connect,
	socket_listen:			ac_socket_listen,
	socket_accept:			ac_socket_accept,
	socket_post_accept:		ac_socket_post_accept,
	socket_sendmsg:			ac_socket_sendmsg,
	socket_recvmsg:			ac_socket_recvmsg,
	socket_getsockname:		ac_socket_getsockname,
	socket_getpeername:		ac_socket_getpeername,
	socket_getsockopt:		ac_socket_getsockopt,
	socket_setsockopt:		ac_socket_setsockopt,
	socket_shutdown:		ac_socket_shutdown,
	socket_sock_alloc_security:	ac_socket_sock_alloc_security,
	socket_sock_free_security:	ac_socket_sock_free_security,
	socket_sock_rcv_skb:		ac_socket_sock_rcv_skb,

	open_request_alloc_security:	ac_open_request_alloc_security,
	open_request_free_security:	ac_open_request_free_security,
	tcp_connection_request:		ac_tcp_connection_request,
	tcp_synack:			ac_tcp_synack,
	tcp_create_openreq_child:	ac_tcp_create_openreq_child,

	skb_alloc_security:		ac_skb_alloc_security,
	skb_clone:			ac_skb_clone,
	skb_copy:			ac_skb_copy,
	skb_set_owner_w:		ac_skb_set_owner_w,
	skb_recv_datagram:		ac_skb_recv_datagram,
	skb_free_security:		ac_skb_free_security,
	
	ip_fragment:			ac_ip_fragment,
	ip_defragment:			ac_ip_defragment,
	ip_encapsulate:			ac_ip_encapsulate,
	ip_decapsulate:			ac_ip_decapsulate,
	ip_decode_options:		ac_ip_decode_options,
	
	netdev_unregister:		ac_netdev_unregister,
#endif
};

extern long sys_security_ac(struct pt_regs regs);
__init int ac_init(void)
{
	struct task_security_struct *tsec;
	struct task_struct *tsk;

	printk(KERN_INFO "Access Control:  Initializing.\n");

	/* Set the security state for the initial task. */
	if (task_alloc_security(current))
		panic("Access Control:  Failed to initialize initial task.\n");
	tsec = current->security;
	tsec->pno = AC_CLASS_KERNEL;

	/* Set the security state for any existing tasks. */
	read_lock(&tasklist_lock);
	for_each_task(tsk) {
		if (task_alloc_security(tsk))
			panic("Access Control:  Failed to initialize initial task.\n");
		tsec = tsk->security;
		tsec->pno = AC_CLASS_KERNEL;
	}
	read_unlock(&tasklist_lock);

	original_ops = secondary_ops = security_ops;
	if (!secondary_ops) 
		panic ("Access Control: No initial security operations\n");
	if (register_security (&ac_ops))
		panic("Access Control: Unable to register with kernel.\n");

	AC_SYSCALL_SET(__NR_security - __NR_SYSCALL_BASE, sys_security_ac);

#if CONFIG_SECURITY_AC_DEVELOP
	if (ac_enforcing) {
		printk(KERN_INFO "Access Control:  Starting in enforcing mode\n");
	} else {
		printk(KERN_INFO "Access Control:  Starting in permissive mode\n");
	}
#endif
	return 0;
}

/* If the separate Access Control kernel patch has been applied, then 
   ac_init is explicitly called from init/main.c:start_kernel() 
   immediately after the security_scaffolding_startup() call to
   initialize the LSM framework. If not, then it will be called with
   the other initcalls, and there will be superblocks and inodes that 
   will lack a properly allocated and initialized security field. */
__initcall(ac_init);
