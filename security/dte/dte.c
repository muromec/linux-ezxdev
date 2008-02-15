/*
 *  Domain and Type Enforcement Security plug
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This essentially a copy of the capability_plug.h file, with a few
 *	mods.  The dte functions to plug in here are in dte-funcs.c
 *
 * author: Serge Hallyn  <hallyn@cs.wm.edu>
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/security.h>
#include <linux/sysctl.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>


extern int dte_initialized;
struct security_operations *dte_secondary_ops;

/*
 * Prototypes for functions defined in dte-funcs.c
 */

extern int dte_sys_security(unsigned int id, unsigned int call,
		unsigned long *args);
extern void dte_post_mountroot (void);
extern void dte_post_addmount (struct vfsmount *mnt, struct nameidata *nd);
extern int dte_binprm_alloc_security (struct linux_binprm *bprm);
extern void dte_binprm_free_security (struct linux_binprm *bprm);
extern int dte_binprm_set_security (struct linux_binprm *bprm);
extern int dte_inode_alloc_security	(struct inode *inode);
extern void dte_inode_free_security	(struct inode *inode);
extern void dte_inode_post_create (struct inode *inode, struct dentry *dentry, int mask);
extern int dte_inode_permission (struct inode *inode, int mask);
extern int dte_task_alloc_security (struct task_struct *p);
extern void dte_task_free_security (struct task_struct *p);
extern int dte_sb_alloc_security (struct super_block *sb);
extern void dte_sb_free_security (struct super_block *sb);
extern int dte_mount (char * dev_name, struct nameidata *nd, char * type,
			unsigned long flags, void * data);
extern int dte_umount (struct vfsmount *mnt, int flags);
extern int dte_check_sb (struct vfsmount *mnt, struct nameidata *nd);
extern void dte_inode_post_mknod (struct inode *inode, struct dentry *dentry,
				  int major, dev_t minor);
extern void dte_inode_post_symlink (struct inode *inode, struct dentry *dentry,
				    const char *name);
extern void dte_inode_post_mkdir (struct inode *inode, struct dentry *dentry,
				  int mask);
extern int dte_task_kill (struct task_struct *p, struct siginfo *info, int sig);
extern void dte_d_instantiate (struct dentry *dentry, struct inode *inode);

/* flag to keep track of how we were registered */
/*static int secondary;*/

/*
 * Stub functions for the default security function pointers in case no
 * security model is loaded */
static int dte_sethostname (char *hostname)
{
	return 0;
}

static int dte_setdomainname (char *domainname)
{
	return 0;
}

static int dte_reboot (unsigned int cmd)
{
	return 0;
}

static int dte_ioperm (unsigned long from, unsigned long num, int turn_on)
{
	return 0;
}

static int dte_iopl (unsigned int old, unsigned int level)
{
	return 0;
}

/*
 * not sure about this one
 * For now, I will assume that if there is a dte_secondary_ops, then it
 * will do the right thing for capable().  This is true as of now,
 * since capability.c does the *right* thing, and owlsm.c does what
 * we used to do.
 * If no dte_secondary_ops, do same thing dummy.c did.
 */
static int dte_capable (struct task_struct *tsk, int cap)
{
	int rc;

	if (dte_secondary_ops) {
		rc = dte_secondary_ops->capable(tsk, cap);
		return rc;
	}
	if (cap_is_fs_cap (cap) ? tsk->fsuid == 0 : tsk->euid == 0)
		/* capability granted */
		return 0;

	/* capability denied */
	return -EPERM;
}

static int dte_swapon (struct swap_info_struct *swap)
{
	return 0;
}

static int dte_swapoff (struct swap_info_struct *swap)
{
	return 0;
}

static int dte_nfsservctl (int cmd, struct nfsctl_arg *arg)
{
	return 0;
}

static int dte_quotactl (int cmds, int type, int id, struct super_block *sb)
{
	return 0;
}

static int dte_quota_on (struct file *f)
{
	return 0;
}

static int dte_bdflush (int func, long data)
{
	return 0;
}

static int dte_syslog (int type)
{
	if (dte_secondary_ops) 
		return dte_secondary_ops->syslog(type);
	return 0;
}

static int dte_settime (struct timeval *tv, struct timezone *tz)
{
        return 0;
}

static int dte_netlink_send (struct sk_buff *skb)
{
	NETLINK_CB (skb).eff_cap = current->cap_effective;
	return 0;
}

static int dte_netlink_recv (struct sk_buff *skb)
{
	if (!cap_raised (NETLINK_CB (skb).eff_cap, CAP_NET_ADMIN))
		return -EPERM;
	return 0;
}

static int dte_ptrace (struct task_struct *parent, struct task_struct *child)
{
	int rc = 0;

	if (dte_secondary_ops)
		rc = dte_secondary_ops->ptrace(parent, child);

	return rc;
}

static int dte_capget (struct task_struct *target, kernel_cap_t * effective,
		       kernel_cap_t * inheritable, kernel_cap_t * permitted)
{
	int rc = 0;

	if (dte_secondary_ops)
		rc = dte_secondary_ops->capget(target, effective, inheritable, permitted);

	return rc;
}

static int dte_capset_check (struct task_struct *target,
			     kernel_cap_t * effective,
			     kernel_cap_t * inheritable,
			     kernel_cap_t * permitted)
{
	int rc = 0;

	if (dte_secondary_ops)
		rc = dte_secondary_ops->capset_check(target, effective,
				inheritable, permitted);

	return rc;
}

static void dte_capset_set (struct task_struct *target,
			    kernel_cap_t * effective,
			    kernel_cap_t * inheritable,
			    kernel_cap_t * permitted)
{
	if (dte_secondary_ops)
		dte_secondary_ops->capset_set(target, effective,
				inheritable, permitted);
}

static int dte_acct (struct file *file)
{
	return 0;
}

static int dte_sysctl (ctl_table * table, int op)
{
	return 0;
}

/* Copied from fs/exec.c */
static inline int must_not_trace_exec (struct task_struct *p)
{
	return (p->ptrace & PT_PTRACED)
	    && dte_capable (p->p_pptr, CAP_SYS_PTRACE);
}

static void dte_binprm_compute_creds (struct linux_binprm *bprm)
{
	if (dte_secondary_ops)
		dte_secondary_ops->bprm_compute_creds(bprm);
}

static int dte_binprm_check_security (struct linux_binprm *bprm)
{
	return 0;
}

static int dte_sb_kern_mount (struct super_block *sb)
{
	return 0;
}

static int dte_sb_statfs (struct super_block *sb)
{
	return 0;
}

static void dte_umount_close (struct vfsmount *mnt)
{
	return;
}

static void dte_umount_busy (struct vfsmount *mnt)
{
	return;
}

static void dte_post_remount (struct vfsmount *mnt, unsigned long flags,
			      void *data)
{
	return;
}

static int dte_pivotroot (struct nameidata *old_nd, struct nameidata *new_nd)
{
	return 0;
}

static void dte_post_pivotroot (struct nameidata *old_nd, struct nameidata *new_nd)
{
	return;
}

static int dte_inode_create (struct inode *inode, struct dentry *dentry,
			     int mask)
{
	return 0;
}

static int dte_inode_link (struct dentry *old_dentry, struct inode *inode,
			   struct dentry *new_dentry)
{
	return 0;
}

static void dte_inode_post_link (struct dentry *old_dentry, struct inode *inode,
				 struct dentry *new_dentry)
{
	return;
}

static int dte_inode_unlink (struct inode *inode, struct dentry *dentry)
{
	return 0;
}

static int dte_inode_symlink (struct inode *inode, struct dentry *dentry,
			      const char *name)
{
	return 0;
}

static int dte_inode_mkdir (struct inode *inode, struct dentry *dentry,
			    int mask)
{
	return 0;
}

static int dte_inode_rmdir (struct inode *inode, struct dentry *dentry)
{
	return 0;
}

static int dte_inode_mknod (struct inode *inode, struct dentry *dentry,
			    int major, dev_t minor)
{
	return 0;
}

static int dte_inode_rename (struct inode *old_inode, struct dentry *old_dentry,
			     struct inode *new_inode, struct dentry *new_dentry)
{
	return 0;
}

static void dte_inode_post_rename (struct inode *old_inode,
				   struct dentry *old_dentry,
				   struct inode *new_inode,
				   struct dentry *new_dentry)
{
	return;
}

static int dte_inode_readlink (struct dentry *dentry)
{
	return 0;
}

static int dte_inode_follow_link (struct dentry *dentry,
				  struct nameidata *nameidata)
{
	int rc = 0;

	if (dte_secondary_ops)
		rc = dte_secondary_ops->inode_follow_link(dentry, nameidata);

	return rc;
}

static int dte_inode_revalidate (struct dentry *inode)
{
	return 0;
}

static int dte_inode_setattr (struct dentry *dentry, struct iattr *iattr)
{
	return 0;
}

static int dte_inode_stat (struct inode *inode)
{
	return 0;
}

static void dte_delete (struct inode *ino)
{
	return;
}

static int dte_file_permission (struct file *file, int mask)
{
	return 0;
}

static int dte_file_alloc_security (struct file *file)
{
	return 0;
}

static void dte_file_free_security (struct file *file)
{
	return;
}

static int dte_file_ioctl (struct file *file, unsigned int command,
			   unsigned long arg)
{
	return 0;
}

static int dte_file_mmap (struct file *file, unsigned long prot,
			  unsigned long flags)
{
	return 0;
}

static int dte_file_mprotect (struct vm_area_struct *vma, unsigned long prot)
{
	return 0;
}

static int dte_file_lock (struct file *file, unsigned int cmd)
{
	return 0;
}

static int dte_file_fcntl (struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	return 0;
}

static int dte_file_set_fowner (struct file *file)
{
	return 0;
}

static int dte_file_send_sigiotask (struct task_struct *tsk,
				    struct fown_struct *fown, int fd,
				    int reason)
{
	return 0;
}

static int dte_file_receive (struct file *file)
{
	return 0;
}

static int dte_task_create (unsigned long clone_flags)
{
	return 0;
}

static int dte_task_setuid (uid_t id0, uid_t id1, uid_t id2, int flags)
{
	return 0;
}

/* moved from kernel/sys.c. */
/* 
 * dte_emulate_setxuid() fixes the effective / permitted capabilities of
 * a process after a call to setuid, setreuid, or setresuid.
 *
 *  1) When set*uiding _from_ one of {r,e,s}uid == 0 _to_ all of
 *  {r,e,s}uid != 0, the permitted and effective capabilities are
 *  cleared.
 *
 *  2) When set*uiding _from_ euid == 0 _to_ euid != 0, the effective
 *  capabilities of the process are cleared.
 *
 *  3) When set*uiding _from_ euid != 0 _to_ euid == 0, the effective
 *  capabilities are set to the permitted capabilities.
 *
 *  fsuid is handled elsewhere. fsuid == 0 and {r,e,s}uid!= 0 should 
 *  never happen.
 *
 *  -astor 
 *
 * cevans - New behaviour, Oct '99
 * A process may, via prctl(), elect to keep its capabilities when it
 * calls setuid() and switches away from uid==0. Both permitted and
 * effective sets will be retained.
 * Without this change, it was impossible for a daemon to drop only some
 * of its privilege. The call to setuid(!=0) would drop all privileges!
 * Keeping uid 0 is not an option because uid 0 owns too many vital
 * files..
 * Thanks to Olaf Kirch and Peter Benie for spotting this.
 */
static inline void dte_emulate_setxuid (int old_ruid, int old_euid,
					int old_suid)
{
	if ((old_ruid == 0 || old_euid == 0 || old_suid == 0) &&
	    (current->uid != 0 && current->euid != 0 && current->suid != 0) &&
	    !current->keep_capabilities) {
		cap_clear (current->cap_permitted);
		cap_clear (current->cap_effective);
	}
	if (old_euid == 0 && current->euid != 0) {
		cap_clear (current->cap_effective);
	}
	if (old_euid != 0 && current->euid == 0) {
		current->cap_effective = current->cap_permitted;
	}
}

static int dte_task_post_setuid (uid_t old_ruid, uid_t old_euid, uid_t old_suid,
				 int flags)
{
	int rc = 0;
	if (dte_secondary_ops)
		return dte_secondary_ops->task_post_setuid(old_ruid, old_euid,
				old_suid, flags);
	return rc;
}

static int dte_task_setgid (gid_t id0, gid_t id1, gid_t id2, int flags)
{
	return 0;
}

static int dte_task_setpgid (struct task_struct *p, pid_t pgid)
{
	return 0;
}

static int dte_task_getpgid (struct task_struct *p)
{
	return 0;
}

static int dte_task_getsid (struct task_struct *p)
{
	return 0;
}

static int dte_task_setgroups (int gidsetsize, gid_t * grouplist)
{
	return 0;
}

static int dte_task_setnice (struct task_struct *p, int nice)
{
	return 0;
}

static int dte_task_setrlimit (unsigned int resource, struct rlimit *new_rlim)
{
	return 0;
}

static int dte_task_setscheduler (struct task_struct *p, int policy,
				  struct sched_param *lp)
{
	return 0;
}

static int dte_task_getscheduler (struct task_struct *p)
{
	return 0;
}

static int dte_task_wait (struct task_struct *p)
{
	return 0;
}

static int dte_task_prctl (int option, unsigned long arg2, unsigned long arg3,
			   unsigned long arg4, unsigned long arg5)
{
	return 0;
}

static void dte_task_kmod_set_label (void)
{
	if (dte_secondary_ops)
		dte_secondary_ops->task_kmod_set_label();
}

static void dte_task_reparent_to_init (struct task_struct *p)
{
	if (dte_secondary_ops)
		dte_secondary_ops->task_reparent_to_init(p);
}

static int dte_ip_decode_options (struct sk_buff *skb, const char *optptr,
				  unsigned char **pp_ptr)
{
	if (!skb && !capable (CAP_NET_RAW)) {
		(const unsigned char *) *pp_ptr = optptr;
		return -EPERM;
	}
	return 0;
}

static int dte_module_create_module (const char *name_user, size_t size)
{
	return 0;
}

static int dte_module_init_module (struct module *mod_user)
{
	return 0;
}

static int dte_module_delete_module (const struct module *mod)
{
	return 0;
}

static int dte_register (const char *name, struct security_operations *ops)
{
	int rc;

	if (dte_secondary_ops) {
		rc = dte_secondary_ops->register_security(name, ops);
		printk("DTE: registering +tertiary module %s returned %d.\n",
				name, rc);
		return rc;
	}
	if (strcmp(name,"capability")==0 ||
			strcmp(name, "owlsm")==0) {
		dte_secondary_ops = ops;
		printk(KERN_NOTICE "DTE: Registering security module %s.\n", name);
		return 0;
	}
	printk(KERN_NOTICE 
	"Only capability and openwall modules may be registered with DTE.\n");
	printk(KERN_NOTICE "DTE: module %s may not be loaded.\n", name);
	return -EINVAL;
}

static int dte_unregister (const char *name, struct security_operations *ops)
{
	if (!dte_secondary_ops) {
		printk(KERN_NOTICE "DTE: no secondary module %s.\n", name);
		return -EINVAL;
	} else if (ops == dte_secondary_ops) {
		dte_secondary_ops = NULL;
		printk(KERN_NOTICE "DTE: unregistering module %s.\n", name);
		return 0;
	} else
		return dte_secondary_ops->unregister_security(name, ops);
}

struct security_operations dte_security_ops = {
	sethostname:			dte_sethostname,
	setdomainname:			dte_setdomainname,
	reboot:				dte_reboot,
	ioperm:				dte_ioperm,
	iopl:				dte_iopl,
	ptrace:				dte_ptrace,
	capget:				dte_capget,
	capset_check:			dte_capset_check,
	capset_set:			dte_capset_set,
	acct:				dte_acct,
	sysctl:				dte_sysctl,
	capable:			dte_capable,
	sys_security:			dte_sys_security,
	swapon:				dte_swapon,
	swapoff:			dte_swapoff,
	nfsservctl:			dte_nfsservctl,
	quotactl:			dte_quotactl,
	quota_on:			dte_quota_on,
	bdflush:			dte_bdflush,
	syslog:				dte_syslog,
	settime:                        dte_settime,

	netlink_send:			dte_netlink_send,
	netlink_recv:			dte_netlink_recv,

	bprm_alloc_security:		dte_binprm_alloc_security,
	bprm_free_security:		dte_binprm_free_security,
	bprm_compute_creds:		dte_binprm_compute_creds,
	bprm_set_security:		dte_binprm_set_security,
	bprm_check_security:		dte_binprm_check_security,
	
	sb_alloc_security:		dte_sb_alloc_security,
	sb_free_security:		dte_sb_free_security,
	sb_kern_mount:                  dte_sb_kern_mount,
	sb_statfs:			dte_sb_statfs,
	sb_mount:			dte_mount,
	sb_check_sb:			dte_check_sb,
	sb_umount:			dte_umount,
	sb_umount_close:		dte_umount_close,
	sb_umount_busy:			dte_umount_busy,
	sb_post_remount:		dte_post_remount,
	sb_post_mountroot:		dte_post_mountroot,
	sb_post_addmount:		dte_post_addmount,
	sb_pivotroot:			dte_pivotroot,
	sb_post_pivotroot:		dte_post_pivotroot,
	
	inode_alloc_security:		dte_inode_alloc_security,
	inode_free_security:		dte_inode_free_security,
	inode_create:			dte_inode_create,
	inode_post_create:		dte_inode_post_create,
	inode_link:			dte_inode_link,
	inode_post_link:		dte_inode_post_link,
	inode_unlink:			dte_inode_unlink,
	inode_symlink:			dte_inode_symlink,
	inode_post_symlink:		dte_inode_post_symlink,
	inode_mkdir:			dte_inode_mkdir,
	inode_post_mkdir:		dte_inode_post_mkdir,
	inode_rmdir:			dte_inode_rmdir,
	inode_mknod:			dte_inode_mknod,
	inode_post_mknod:		dte_inode_post_mknod,
	inode_rename:			dte_inode_rename,
	inode_post_rename:		dte_inode_post_rename,
	inode_readlink:			dte_inode_readlink,
	inode_follow_link:		dte_inode_follow_link,
	inode_permission:		dte_inode_permission,
	inode_revalidate:		dte_inode_revalidate,
	inode_setattr:			dte_inode_setattr,
	inode_stat:			dte_inode_stat,
	inode_delete:			dte_delete,
	
	file_permission:		dte_file_permission,
	file_alloc_security:		dte_file_alloc_security,
	file_free_security:		dte_file_free_security,
	file_ioctl:			dte_file_ioctl,
	file_mmap:			dte_file_mmap,
	file_mprotect:			dte_file_mprotect,
	file_lock:			dte_file_lock,
	file_fcntl:			dte_file_fcntl,
	file_set_fowner:		dte_file_set_fowner,
	file_send_sigiotask:		dte_file_send_sigiotask,
	file_receive:			dte_file_receive,
	
	task_create:			dte_task_create,
	task_alloc_security:		dte_task_alloc_security,
	task_free_security:		dte_task_free_security,
	task_setuid:			dte_task_setuid,
	task_post_setuid:		dte_task_post_setuid,
	task_setgid:			dte_task_setgid,
	task_setpgid:			dte_task_setpgid,
	task_getpgid:			dte_task_getpgid,
	task_getsid:			dte_task_getsid,
	task_setgroups:			dte_task_setgroups,
	task_setnice:			dte_task_setnice,
	task_setrlimit:			dte_task_setrlimit,
	task_setscheduler:		dte_task_setscheduler,
	task_getscheduler:		dte_task_getscheduler,
	task_wait:			dte_task_wait,
	task_kill:			dte_task_kill,
	task_prctl:			dte_task_prctl,
	task_kmod_set_label:		dte_task_kmod_set_label,
	task_reparent_to_init:		dte_task_reparent_to_init,

	ip_decode_options:		dte_ip_decode_options,
	
	module_create:			dte_module_create_module,
	module_initialize:		dte_module_init_module,
	module_delete:			dte_module_delete_module,
	
	register_security:		dte_register,
	unregister_security:		dte_unregister,

	d_instantiate:                  dte_d_instantiate,
};

extern int setup_dte_module(void);

static int __init dte_plug_init (void)
{
	/* register ourselves with the security framework */
	if (register_security (&dte_security_ops)) {
		printk (KERN_INFO "Failure registering DTE with the kernel\n");
		return -EINVAL;
	}
	dte_initialized = 0;
	dte_secondary_ops = NULL;
#ifdef MODULE
	printk(KERN_NOTICE "Setting up DTE...\n");
	if (setup_dte_module()) {
		if (unregister_security (&dte_security_ops)) {
			panic("Failure unregistering DTE with the kernel\n");
		} else {
			printk(KERN_ERR "Unregistered DTE functions!\n");
			return -EINVAL;
		}
	}
	printk(KERN_NOTICE "Finished setting up DTE.\n");
#endif
	printk(KERN_INFO "Domain and Type Enforcement Plug initialized\n");
	return 0;
}

static void __exit dte_plug_exit (void)
{
	printk(KERN_ERR "Attempted removal of DTE: not allowed!\n");
}

module_init (dte_plug_init);
module_exit (dte_plug_exit);

MODULE_AUTHOR("Serge Hallyn");
MODULE_DESCRIPTION("Linux DTE Security Module");
MODULE_LICENSE("GPL");
