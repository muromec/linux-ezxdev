/*
 * This is NOT Openwall, however, it is some of the Openwall kernel patch
 * ported to LSM.  For more information regarding the Openwall project
 * can be found at http://www.openwall.com/
 *
 * Copyright (C) 2001 Emily Ratliff <ratliff@austin.ibm.com>
 * Copyright (C) 2001 Nick Bellinger, Esquire <nickb@attheoffice.org>
 * Copyright (C) 2001 Chris Wright <chris@wirex.com>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/security.h>
#include <linux/stat.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <linux/ctype.h>
#include <linux/file.h>

#include "owlsm.h"

/* flag to keep track of how we were registered */
static int secondary;

static int owlsm_sethostname (char *hostname)
{
	return 0;
}

static int owlsm_setdomainname (char *domainname)
{
	return 0;
}

static int owlsm_reboot (unsigned int cmd)
{
	return 0;
}

static int owlsm_ioperm (unsigned long from, unsigned long num, int turn_on)
{
	return 0;
}

static int owlsm_iopl (unsigned int old, unsigned int level)
{
	return 0;
}

static int owlsm_ptrace (struct task_struct *parent, struct task_struct *child) 
{ 
	return 0; 
}

static int owlsm_capget (struct task_struct *target, kernel_cap_t * effective,
                        kernel_cap_t * inheritable, kernel_cap_t * permitted)
{
	return 0;
}

static int owlsm_capset_check (struct task_struct *target,
                             kernel_cap_t * effective,
                             kernel_cap_t * inheritable,
                             kernel_cap_t * permitted)
{
	return 0;
}

static void owlsm_capset_set (struct task_struct *target,
                            kernel_cap_t * effective,
                            kernel_cap_t * inheritable,
                            kernel_cap_t * permitted)
{
	return;
}

static int owlsm_acct (struct file *file) 
{
	return 0;
}

static int owlsm_capable (struct task_struct *tsk, int cap)
{
	/* from dummy.c */
	if (cap_is_fs_cap (cap) ? tsk->fsuid == 0 : tsk->euid == 0)
		/* capability granted */
		return 0;

	/* capability denied */
	return -EPERM;
}

static int owlsm_sysctl (ctl_table * table, int op) 
{
	return 0;
}

static int owlsm_sys_security (unsigned int id, unsigned int call,
                             unsigned long *args)
{
        return -ENOSYS;
}

static int owlsm_swapon (struct swap_info_struct *swap)
{
        return 0;
}

static int owlsm_swapoff (struct swap_info_struct *swap)
{
        return 0;
}

static int owlsm_nfsservctl (int cmd, struct nfsctl_arg *arg)
{
        return 0;
}

static int owlsm_quotactl (int cmds, int type, int id, struct super_block *sb)
{
        return 0;
}

static int owlsm_quota_on (struct file *f)
{
        return 0;
}

static int owlsm_bdflush (int func, long data)
{
        return 0;
}

static int owlsm_syslog (int type)
{
        return 0;
}

static int owlsm_settime (struct timeval *tv, struct timezone *tz)
{
        return 0;
}

static int owlsm_netlink_send (struct sk_buff *skb)
{
	/* from dummy.c */
	if (current->euid == 0)
		cap_raise (NETLINK_CB (skb).eff_cap, CAP_NET_ADMIN);
	else
		NETLINK_CB (skb).eff_cap = 0;
	return 0;
}

static int owlsm_netlink_recv (struct sk_buff *skb)
{
	/* from dummy.c */
	if (!cap_raised (NETLINK_CB (skb).eff_cap, CAP_NET_ADMIN))
		return -EPERM;
	return 0;
}

static int owlsm_binprm_alloc_security(struct linux_binprm *bprm)
{
	int exec_return, fd_return;
	
	exec_return = do_owlsm_exec_rlimit(bprm);
	fd_return = do_owlsm_sfd_alloc(bprm);

	return (exec_return || fd_return) ? -EPERM : 0;
}

static void owlsm_binprm_free_security (struct linux_binprm *bprm)
{
	return do_owlsm_sfd_free(bprm);
}

static void owlsm_binprm_compute_creds (struct linux_binprm *bprm) 
{
	return do_owlsm_sfd_compute(bprm);
}

static int owlsm_binprm_set_security (struct linux_binprm *bprm) 
{
	return do_owlsm_sfd_set(bprm);
}

static int owlsm_binprm_check_security (struct linux_binprm *bprm)
{
	return 0;
}

static int owlsm_sb_alloc_security (struct super_block *sb)
{
	return 0;
}

static void owlsm_sb_free_security (struct super_block *sb)
{
	return;
}

static int owlsm_sb_kern_mount (struct super_block *sb)
{
	return 0;
}

static int owlsm_sb_statfs (struct super_block *sb) 
{
	return 0;
}

static int owlsm_sb_mount (char *devname, struct nameidata *nd, char *type, 
			  unsigned long flags, void *data) 
{
	return 0;
}

static int owlsm_sb_check_sb (struct vfsmount *mnt, struct nameidata *nd)
{
	return 0;
}

static int owlsm_sb_umount (struct vfsmount *mnt, int flags) 
{
	return 0;
}

static void owlsm_sb_umount_close (struct vfsmount *mnt)
{
	return;
}

static void owlsm_sb_umount_busy (struct vfsmount *mnt) 
{
	return;
}
static void owlsm_sb_post_remount (struct vfsmount *mnt, unsigned long flags, 
				  void *data)
{
	return;
}

static void owlsm_sb_post_mountroot (void)
{
	return;
}

static void owlsm_sb_post_addmount (struct vfsmount *mnt, struct nameidata *nd)
{
	return;
}

static int owlsm_pivotroot (struct nameidata *old_nd, struct nameidata *new_nd)
{
	return 0;
}

static void owlsm_post_pivotroot (struct nameidata *old_nd, struct nameidata *new_nd)
{
	return;
}

static int owlsm_inode_alloc_security (struct inode *inode)
{
	return 0;
}

static void owlsm_inode_free_security (struct inode *inode)
{
	return;
}

static int owlsm_inode_create (struct inode *inode, struct dentry *dentry, 
			      int mask) 
{
	return 0;
}

static void owlsm_inode_post_create (struct inode *inode, struct dentry *dentry,
				    int mask) 
{
	return;
}

static int owlsm_inode_link (struct dentry *old_dentry, struct inode *inode, 
			    struct dentry *new_dentry) 
{
	return do_owlsm_link(old_dentry, inode, new_dentry);
}

static void owlsm_inode_post_link (struct dentry *old_dentry, 
				  struct inode *inode, 
				  struct dentry *new_dentry) 
{
	return;
}

static int owlsm_inode_unlink (struct inode *inode, struct dentry *dentry)
{
	return 0;
}

static int owlsm_inode_symlink (struct inode *inode, struct dentry *dentry, 
			       const char *name)
{
	return 0;
}

static void owlsm_inode_post_symlink (struct inode *inode, 
				      struct dentry *dentry, const char *name)
{
	return;
}

static int owlsm_inode_mkdir (struct inode *inode, struct dentry *dentry, 
			     int mask) 
{
	return 0;
}

static void owlsm_inode_post_mkdir (struct inode *inode, struct dentry *dentry, 
				   int mask) 
{
	return;
}

static int owlsm_inode_rmdir (struct inode *inode, struct dentry *dentry) 
{
	return 0;
}

static int owlsm_inode_mknod (struct inode *inode, struct dentry *dentry, 
			     int major, dev_t minor) 
{
	return 0;
}

static void owlsm_inode_post_mknod (struct inode *inode, struct dentry *dentry, 
				   int major, dev_t minor)
{
	return;
}

static int owlsm_inode_rename (struct inode *old_inode, 
			      struct dentry *old_dentry, 
			      struct inode *new_inode, 
			      struct dentry *new_dentry) 
{
	return 0;
}

static void owlsm_inode_post_rename (struct inode *old_inode, 
				    struct dentry *old_dentry, 
				    struct inode *new_inode, 
				    struct dentry *new_dentry) 
{
	return;
}

static int owlsm_inode_readlink (struct dentry *dentry)
{
	return 0;
}

static int owlsm_inode_follow_link (struct dentry *dentry, 
				   struct nameidata *nameidata) 
{
	return do_owlsm_follow_link(dentry, nameidata);
}

static int owlsm_inode_permission (struct inode *inode, int mask)
{
	return 0;
}

static int owlsm_inode_revalidate (struct dentry *inode)
{
	return 0;
}

static int owlsm_inode_setattr (struct dentry *dentry, struct iattr *iattr)
{
	return 0;
}

static int owlsm_inode_stat (struct inode *inode)
{
	return 0;
}

static void owlsm_delete (struct inode *ino) 
{
	return;
}

static int owlsm_file_permission (struct file *file, int mask)
{
	return 0;
}

static int owlsm_file_alloc_security (struct file *file)	
{
	return 0;
}

static void owlsm_file_free_security (struct file *file)	
{
	return;
}

static int owlsm_file_ioctl (struct file *file, unsigned int command , 
			    unsigned long arg)
{
	return 0;
}

static int owlsm_file_mmap (struct file *file, unsigned long prot, 
			   unsigned long flags)	
{
	return 0;
}

static int owlsm_file_mprotect (struct vm_area_struct *vma, unsigned long prot)
{
	return 0;
}

static int owlsm_file_lock (struct file *file, unsigned int cmd)
{
	return 0;
}

static int owlsm_file_fcntl (struct file *file, unsigned int cmd, 
			     unsigned long arg)
{
	return 0;
}

static int owlsm_file_set_fowner (struct file *file)
{
	return 0;
}

static int owlsm_file_send_sigiotask (struct task_struct *tsk, 
				      struct fown_struct *fown, 
				      int fd, int reason) 
{ 
	return 0;
}

static int owlsm_file_receive (struct file *file)
{
	return 0;
}

static int owlsm_task_create (unsigned long clone_flags)
{
	return 0;
}

static int owlsm_task_alloc_security (struct task_struct *p)
{
	return 0;
}

static void owlsm_task_free_security (struct task_struct *p)
{
	return;
}

static int owlsm_task_setuid (uid_t id0, uid_t id1, uid_t id2, int flags)
{ 
	return 0; 
}

static int owlsm_task_post_setuid(uid_t old_ruid, uid_t old_euid, 
				 uid_t old_suid, int flags)
{
	return 0;
}

static int owlsm_task_setgid (gid_t id0, gid_t id1, gid_t id2, int flags) 
{
	return 0;
}

static int owlsm_task_setpgid (struct task_struct *p, pid_t pgid) 
{
	return 0;
}

static int owlsm_task_getpgid (struct task_struct *p) 
{
	return 0;
}

static int owlsm_task_getsid (struct task_struct *p) 
{
	return 0;
}

static int owlsm_task_setgroups (int gidsetsize, gid_t *grouplist)
{
	return 0;
}

static int owlsm_task_setnice (struct task_struct *p, int nice)
{
	return 0;
}

static int owlsm_task_setrlimit (unsigned int resource, struct rlimit *new_rlim)
{
	return 0;
}

static int owlsm_task_setscheduler (struct task_struct *p, int policy,
				   struct sched_param *lp)
{
	return 0;
}

static int owlsm_task_getscheduler (struct task_struct *p) 
{ 
	return 0; 
}

static int owlsm_task_wait (struct task_struct *p) 
{
	return 0;
}

static int owlsm_task_kill (struct task_struct *p, struct siginfo *info, int sig)
{
	return 0;
}

static int owlsm_task_prctl (int option, unsigned long arg2, unsigned long arg3,
			     unsigned long arg4, unsigned long arg5)
{
	return 0;
}

static void owlsm_task_kmod_set_label (void) 
{ 
	return; 
}

static void owlsm_task_reparent_to_init (struct task_struct *p) 
{ 
	p->euid = p->fsuid = 0;
	return; 
}

static int owlsm_decode_options (struct sk_buff *skb, const char *optptr,
				unsigned char **pp_ptr)
{
	/* from dummy.c */
	if (!skb && !capable (CAP_NET_RAW)) {
		(const unsigned char *) *pp_ptr = optptr;
		return -EPERM;
	}
	return 0;
}

static int owlsm_module_create_module (const char *name_user, size_t size)
{
	return 0;
}

static int owlsm_module_init_module (struct module *mod)
{
	return 0;
}

static int owlsm_module_delete_module (const struct module *mod)	
{
	return 0;
}

static int owlsm_register (const char *name, struct security_operations *ops)	
{
	return -EINVAL;
}

static int owlsm_unregister (const char *name, struct security_operations *ops)
{
	return -EINVAL;
}

static struct security_operations owlsm_ops = {
	sethostname:			owlsm_sethostname,
	setdomainname:			owlsm_setdomainname,
	reboot:				owlsm_reboot,
	ioperm:				owlsm_ioperm,
	iopl:				owlsm_iopl,
	ptrace:				owlsm_ptrace,
	capget:				owlsm_capget,
	capset_check:			owlsm_capset_check,
	capset_set:			owlsm_capset_set,
	acct:				owlsm_acct,
	sysctl:				owlsm_sysctl,
	capable:			owlsm_capable,
	sys_security:			owlsm_sys_security,
	swapon:				owlsm_swapon,
	swapoff:			owlsm_swapoff,
	nfsservctl:			owlsm_nfsservctl,
	quotactl:			owlsm_quotactl,
	quota_on:			owlsm_quota_on,
	bdflush:			owlsm_bdflush,
	syslog:				owlsm_syslog,
	settime:                        owlsm_settime,

	netlink_send:			owlsm_netlink_send,
	netlink_recv:			owlsm_netlink_recv,
	
	bprm_alloc_security:		owlsm_binprm_alloc_security,
	bprm_free_security:		owlsm_binprm_free_security,
	bprm_compute_creds:		owlsm_binprm_compute_creds,
	bprm_set_security:		owlsm_binprm_set_security,
	bprm_check_security:		owlsm_binprm_check_security,
	
	sb_alloc_security:		owlsm_sb_alloc_security,
	sb_free_security:		owlsm_sb_free_security,
	sb_kern_mount:		        owlsm_sb_kern_mount,
	sb_statfs:		        owlsm_sb_statfs,
	sb_mount:			owlsm_sb_mount,
	sb_check_sb:			owlsm_sb_check_sb,
	sb_umount:			owlsm_sb_umount,
	sb_umount_close:		owlsm_sb_umount_close,
	sb_umount_busy:			owlsm_sb_umount_busy,
	sb_post_remount:		owlsm_sb_post_remount,
	sb_post_mountroot:		owlsm_sb_post_mountroot,
	sb_post_addmount:		owlsm_sb_post_addmount,
	sb_pivotroot:			owlsm_pivotroot,
	sb_post_pivotroot:		owlsm_post_pivotroot,
	
	inode_alloc_security:		owlsm_inode_alloc_security,
	inode_free_security:		owlsm_inode_free_security,
	inode_create:			owlsm_inode_create,
	inode_post_create:		owlsm_inode_post_create,
	inode_link:			owlsm_inode_link,
	inode_post_link:		owlsm_inode_post_link,
	inode_unlink:			owlsm_inode_unlink,
	inode_symlink:			owlsm_inode_symlink,
	inode_post_symlink:		owlsm_inode_post_symlink,
	inode_mkdir:			owlsm_inode_mkdir,
	inode_post_mkdir:		owlsm_inode_post_mkdir,
	inode_rmdir:			owlsm_inode_rmdir,
	inode_mknod:			owlsm_inode_mknod,
	inode_post_mknod:		owlsm_inode_post_mknod,
	inode_rename:			owlsm_inode_rename,
	inode_post_rename:		owlsm_inode_post_rename,
	inode_readlink:			owlsm_inode_readlink,
	inode_follow_link:		owlsm_inode_follow_link,
	inode_permission:		owlsm_inode_permission,
	inode_revalidate:		owlsm_inode_revalidate,
	inode_setattr:			owlsm_inode_setattr,
	inode_stat:			owlsm_inode_stat,
	inode_delete:			owlsm_delete,
	
	file_permission:		owlsm_file_permission,
	file_alloc_security:		owlsm_file_alloc_security,
	file_free_security:		owlsm_file_free_security,
	file_ioctl:			owlsm_file_ioctl,
	file_mmap:			owlsm_file_mmap,
	file_mprotect:			owlsm_file_mprotect,
	file_lock:			owlsm_file_lock,
	file_fcntl:			owlsm_file_fcntl,
	file_set_fowner:		owlsm_file_set_fowner,
	file_send_sigiotask:		owlsm_file_send_sigiotask,
	file_receive:			owlsm_file_receive,
	
	task_create:			owlsm_task_create,
	task_alloc_security:		owlsm_task_alloc_security,
	task_free_security:		owlsm_task_free_security,
	task_setuid:			owlsm_task_setuid,
	task_post_setuid:		owlsm_task_post_setuid,
	task_setgid:			owlsm_task_setgid,
	task_setpgid:			owlsm_task_setpgid,
	task_getpgid:			owlsm_task_getpgid,
	task_getsid:			owlsm_task_getsid,
	task_setgroups:			owlsm_task_setgroups,
	task_setnice:			owlsm_task_setnice,
	task_setrlimit:			owlsm_task_setrlimit,
	task_setscheduler:		owlsm_task_setscheduler,
	task_getscheduler:		owlsm_task_getscheduler,
	task_wait:			owlsm_task_wait,
	task_kill:			owlsm_task_kill,
	task_prctl:			owlsm_task_prctl,
	task_kmod_set_label:		owlsm_task_kmod_set_label,
	task_reparent_to_init:		owlsm_task_reparent_to_init,

	ip_decode_options:		owlsm_decode_options,
	
	module_create:			owlsm_module_create_module,
	module_initialize:		owlsm_module_init_module,
	module_delete:			owlsm_module_delete_module,
	
	register_security:		owlsm_register,
	unregister_security:		owlsm_unregister,
};

#if defined(CONFIG_SECURITY_owlsm_MODULE)
#define MY_NAME THIS_MODULE->name
#else
#define MY_NAME "owlsm"
#endif

static int __init owlsm_init (void)
{
	/* register ourselves with the security framework */
	if (register_security (&owlsm_ops)) {
		printk (KERN_INFO 
			"Failure registering owlsm module with the kernel\n");
		/* try registering with primary module */
		if (mod_reg_security (MY_NAME, &owlsm_ops)) {
			printk (KERN_INFO "Failure registering owlsm module "
				"with primary security module.\n");
			return -EINVAL;
		}
		secondary = 1;
	}
	printk(KERN_INFO "owlsm LSM initialized\n");
	return 0;
}

static void __exit owlsm_exit (void)
{
	/* remove ourselves from the security framework */
	if (secondary) {
		if (mod_unreg_security (MY_NAME, &owlsm_ops))
			printk (KERN_INFO "Failure unregistering owlsm module "
				"with primary module.\n");
		return;
	}
 
	if (unregister_security (&owlsm_ops)) {
		printk (KERN_INFO
			"Failure unregistering owlsm module with the kernel\n");
	}
}

module_init (owlsm_init);
module_exit (owlsm_exit);

MODULE_DESCRIPTION("LSM implementation of the Openwall kernel patch");
MODULE_LICENSE("GPL");
