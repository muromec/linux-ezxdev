/*
 *  NSA Security-Enhanced Linux (SELinux) security module
 *
 *  This file contains the architecture-independent code for the
 *  SELinux new system call implementations.
 *
 *  Authors:  Stephen Smalley, <sds@epoch.ncsc.mil>
 *            Chris Vance, <cvance@nai.com>
 *
 *  Copyright (C) 2001,2002 Networks Associates Technology, Inc.
 * 
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 */ 

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/security.h>
#include <linux/capability.h>
#include <linux/unistd.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/spinlock.h>
#include <linux/file.h>
#include <linux/socket.h>
#include <asm/uaccess.h>
#include <asm/ipc.h>			/* MSGGET, etc needed for sys_ipc() */
#include <linux/flask/avc.h>
#include <linux/flask/psid.h>
#include <linux/flask/syscalls.h>
#include <linux/flask/syscalls_proto.h>
#include <asm/flask/syscallaccess.h>
#include "selinux_plug.h"

long sys_getsecsid(void) 
{
	struct task_security_struct *tsec;
	extern int ss_initialized;

	if (!ss_initialized)
		return -ENOSYS;

	tsec = current->security;
	return tsec->sid;
}

long sys_getosecsid(void) 
{
	struct task_security_struct *tsec;

	tsec = current->security;
	return tsec->osid;
}

long sys_lstat_stat_secure(int follow_link,
			   const char *pathname, 
			   struct stat *buf,
			   security_id_t *out_sid)
{
	long (*stat_f)(const char * filename, struct stat * statbuf);
	struct task_security_struct *tsec;
	int rc;

	if (follow_link) 
		stat_f = SELINUX_SYSCALL_GET(__NR_stat);
	else
		stat_f = SELINUX_SYSCALL_GET(__NR_lstat);

	tsec = current->security;

	rc = stat_f(pathname, buf);
	if (rc)
		return rc;

	if (out_sid) {
		if (copy_to_user(out_sid, &tsec->out_sid[0], sizeof(security_id_t)))
			return -EFAULT;
	}
	
	return 0;
}

long sys_fstat_secure(unsigned int fd,
		      struct stat *buf,
		      security_id_t *out_sid)
{
	long (*fstat_f)(unsigned int fd, struct stat * statbuf);
	struct task_security_struct *tsec;
	int rc;

	fstat_f = SELINUX_SYSCALL_GET(__NR_fstat);

	tsec = current->security;
	rc = fstat_f(fd, buf);
	if (rc)
		return rc;

	if (out_sid) {
		if (copy_to_user(out_sid, &tsec->out_sid[0], sizeof(security_id_t)))
			return -EFAULT;
	}
	
	return 0;
}

/* Only one of the dentry and inode arguments needs to 
   be specified. The other parameter should be NULL.
   Use dentry if one is available to the caller.  Otherwise, 
   use inode.  */
static int chsid_common(struct dentry *dentry, 
			struct inode * inode,  
			security_id_t newsid)
{
	struct task_security_struct *tsec;
	struct inode_security_struct *isec;
	struct superblock_security_struct *sbsec;
	avc_audit_data_t ad;
	int rc = 0;

	if (dentry)
		inode = dentry->d_inode;

	if (!inode) {
		printk("chsid_common: NULL inode\n");
		return -ENOENT;
	}

	if (IS_RDONLY(inode))
		return -EROFS;

	tsec = current->security;
	isec = inode->i_security;

	AVC_AUDIT_DATA_INIT(&ad,FS);
	if (dentry)
		ad.u.fs.dentry = dentry;
	else
		ad.u.fs.inode = inode;

	down(&isec->sem);

	rc = avc_has_perm_ref_audit(tsec->sid, isec->sid, isec->sclass, 
				    FILE__RELABELFROM,
				    &isec->avcr, &ad);
	if (rc)
		goto out;

	rc = avc_has_perm_audit(tsec->sid, newsid, isec->sclass,
				FILE__RELABELTO, &ad);
	if (rc)
		goto out;

	if (inode->i_sb) {
		sbsec = inode->i_sb->s_security;
		if (!sbsec) {
			isec->sid = newsid;
			goto out;
		}

		rc = avc_has_perm_audit(newsid,
					sbsec->sid,
					SECCLASS_FILESYSTEM,
					FILESYSTEM__ASSOCIATE,
					&ad);
		if (rc)
			goto out;

		if (sbsec->uses_psids) {
			rc = sid_to_psid(inode, newsid);
			if (rc) 
				goto out;
		}
	}

	isec->sid = newsid;
out:

	up(&isec->sem);
	return rc;
}

long sys_lchsid_chsid(int follow_link,
		      const char *filename, 
		      security_id_t sid)
{
	struct nameidata nd;
	int error;

	if (follow_link) 
		error = user_path_walk(filename, &nd);
	else
		error = user_path_walk_link(filename, &nd);
	if (!error) {
		error = chsid_common(nd.dentry, NULL, sid);
		path_release(&nd);
	}
	return error;
}

long sys_fchsid(unsigned int fd, security_id_t sid)
{
	struct file * file;
	int error = -EBADF;

	file = fget(fd);
	if (file) {
		error = chsid_common(file->f_dentry, NULL, sid);
		fput(file);
	}
	return error;
}

long sys_ichsid(const char *filename, 
		unsigned long ino,
		security_id_t sid)
{
	struct nameidata nd;
	struct super_block *sb;
	struct inode *inode;
	int error;

	error = task_has_system(current, SYSTEM__ICHSID);
	if (error)
		return error;

	if (!ino)
		return -EINVAL;

	error = user_path_walk(filename, &nd);
	if (!error) {
		error = -ENODEV;
		sb = nd.dentry->d_inode->i_sb;
		if (sb) {
			error = -ENOENT;
			inode = iget(sb, ino);
			if (inode) {
				if (!is_bad_inode(inode)) 
					error = chsid_common(NULL, inode, sid);
				iput(inode);
			}
		}
		path_release(&nd);
	}
	return error;
}

long sys_open_secure(const char * filename, int flags, int mode,
		     security_id_t sid)
{
	long (*open_f)(const char * filename, int flags, int mode);
	struct task_security_struct *tsec;
	int rc;

	open_f = SELINUX_SYSCALL_GET(__NR_open);

	tsec = current->security;
	tsec->in_sid[0] = sid;
	rc = open_f(filename, flags, mode);
	tsec->in_sid[0] = 0;
	return rc;
}

long sys_mkdir_secure(const char * pathname, int mode,
		      security_id_t sid)
{
	long (*mkdir_f)(const char * pathname, int mode);
	struct task_security_struct *tsec;
	int rc;

	mkdir_f = SELINUX_SYSCALL_GET(__NR_mkdir);

	tsec = current->security;
	tsec->in_sid[0] = sid;
	rc = mkdir_f(pathname, mode);
	tsec->in_sid[0] = 0;
	return rc;
}

long sys_mknod_secure(const char * filename, int mode, dev_t dev,
		      security_id_t sid)
{
	long (*mknod_f)(const char * filename, int mode, dev_t dev);
	struct task_security_struct *tsec;
	int rc;

	mknod_f = SELINUX_SYSCALL_GET(__NR_mknod);

	tsec = current->security;
	tsec->in_sid[0] = sid;
	rc = mknod_f(filename,mode,dev);
	tsec->in_sid[0] = 0;
	return rc;
}

long sys_symlink_secure(const char * oldname, const char * newname,
			security_id_t sid)
{
	long (*symlink_f)(const char * oldname, const char * newname);
	struct task_security_struct *tsec;
	int rc;

	symlink_f = SELINUX_SYSCALL_GET(__NR_symlink);

	tsec = current->security;
	tsec->in_sid[0] = sid;
	rc = symlink_f(oldname, newname);
	tsec->in_sid[0] = 0;
	return rc;
}

long sys_statfs_secure(const char *pathname, 
		       struct statfs *buf,
		       security_id_t *out_sid)
{
	long (*statfs_f)(const char * path, struct statfs * buf);
	struct task_security_struct *tsec;
	int rc;

	statfs_f = SELINUX_SYSCALL_GET(__NR_statfs);

	tsec = current->security;
	rc = statfs_f(pathname, buf);
	if (rc)
		return rc;

	if (out_sid) {
		if (copy_to_user(out_sid, &tsec->out_sid[0], sizeof(security_id_t)))
			return -EFAULT;
	}
	
	return 0;
}

long sys_fstatfs_secure(unsigned int fd,
			struct statfs *buf,
			security_id_t *out_sid)
{
	long (*fstatfs_f)(unsigned int fd, struct statfs * buf);
	struct task_security_struct *tsec;
	int rc;

	fstatfs_f = SELINUX_SYSCALL_GET(__NR_fstatfs);

	tsec = current->security;
	rc = fstatfs_f(fd, buf);
	if (rc)
		return rc;

	if (out_sid) {
		if (copy_to_user(out_sid, &tsec->out_sid[0], sizeof(security_id_t)))
			return -EFAULT;
	}
	
	return 0;
}

int vfs_chsidfs(struct super_block *sb, 
		security_id_t fs_sid, 
		security_id_t f_sid)
{
	struct task_security_struct *tsec;
	struct superblock_security_struct *sbsec;
	avc_audit_data_t ad;
	int rc = 0;

	if (!sb) 
		return -ENODEV;

	tsec = current->security;
	sbsec = sb->s_security;

	AVC_AUDIT_DATA_INIT(&ad,FS);
	ad.u.fs.dentry = sb->s_root;

	down(&sbsec->sem);

	rc = avc_has_perm_audit(tsec->sid, sbsec->sid, SECCLASS_FILESYSTEM,
				FILESYSTEM__RELABELFROM, &ad);
	if (rc)
		goto out;

	rc = avc_has_perm_audit(sbsec->sid, fs_sid, SECCLASS_FILESYSTEM,
				FILESYSTEM__TRANSITION, &ad);
	if (rc)
		goto out;

	rc = avc_has_perm_audit(tsec->sid, fs_sid, SECCLASS_FILESYSTEM,
				FILESYSTEM__RELABELTO, &ad);
	if (rc)
		goto out;

	rc = avc_has_perm_audit(f_sid, fs_sid, SECCLASS_FILESYSTEM,
				FILESYSTEM__ASSOCIATE,
				&ad);
	if (rc)
		goto out;

	if (sbsec->uses_psids) {
		rc = psid_chsidfs(sb, fs_sid, f_sid);
		if (rc) 
			goto out;
	}

	sbsec->sid = fs_sid;

 out:
	up(&sbsec->sem);
	return rc;
}

long sys_chsidfs(const char * path, 
		 security_id_t fs_sid,
		 security_id_t f_sid)
{
	struct nameidata nd;
	int error;

	error = user_path_walk(path, &nd);
	if (!error) {
		error = vfs_chsidfs(nd.dentry->d_inode->i_sb, fs_sid, f_sid);
		path_release(&nd);
	}
	return error;
}

long sys_fchsidfs(unsigned int fd, 
		  security_id_t fs_sid,
		  security_id_t f_sid)
{
	struct file * file;
	int error;

	error = -EBADF;
	file = fget(fd);
	if (!file)
		goto out;
	error = vfs_chsidfs(file->f_dentry->d_inode->i_sb, fs_sid, f_sid);
	fput(file);
out:
	return error;
}

/*
 * 8 new IPC syscalls.
 */
int sys_semsid(int semid, security_id_t *out_sid)
{
	long (*ipc_f)(uint call, int first, int second, 
		      int third, void *ptr, long fifth);
	struct task_security_struct *tsec;
	mm_segment_t old_fs;
	struct semid_ds buf;
	union semun arg;
	int err;

	tsec = current->security;

	ipc_f = SELINUX_SYSCALL_GET(__NR_ipc);

	arg.buf = &buf;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	err = ipc_f(SEMCTL, semid, 0, IPC_STAT, &arg, 0);
	set_fs(old_fs);

	if (err)
		return err;
	
        if (out_sid) {
                if (copy_to_user(out_sid, &tsec->out_sid[0],
                                 sizeof(security_id_t)))
                        return -EFAULT;
        }

	return 0;
}

int sys_shmsid(int shmid, security_id_t *out_sid)
{
	long (*ipc_f)(uint call, int first, int second, 
		      int third, void *ptr, long fifth);
	struct task_security_struct *tsec;
	mm_segment_t old_fs;
	struct shmid_ds buf;
	int err;

	tsec = current->security;

	ipc_f = SELINUX_SYSCALL_GET(__NR_ipc);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	err = ipc_f(SHMCTL, shmid, IPC_STAT, 0, &buf, 0);
	set_fs(old_fs);

	if (err)
		return err;
	
        if (out_sid) {
                if (copy_to_user(out_sid, &tsec->out_sid[0],
                                 sizeof(security_id_t)))
                        return -EFAULT;
        }

	return 0;
}

int sys_msgsid(int msqid, security_id_t *out_sid)
{
	long (*ipc_f)(uint call, int first, int second, 
		      int third, void *ptr, long fifth);
	struct task_security_struct *tsec;
	mm_segment_t old_fs;
	struct msqid_ds buf;
	int err;

	tsec = current->security;

	ipc_f = SELINUX_SYSCALL_GET(__NR_ipc);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	err = ipc_f(MSGCTL, msqid, IPC_STAT, 0, &buf, 0);
	set_fs(old_fs);

	if (err)
		return err;
	
        if (out_sid) {
                if (copy_to_user(out_sid, &tsec->out_sid[0],
                                 sizeof(security_id_t)))
                        return -EFAULT;
        }

	return 0;
}

int sys_msgget_secure(key_t key, int msgflag, security_id_t sid)
{
	long (*ipc_f)(uint call, int first, int second, 
		      int third, void *ptr, long fifth);
	struct task_security_struct *tsec;
	int rc;

	ipc_f = SELINUX_SYSCALL_GET(__NR_ipc);

	tsec = current->security;
	tsec->in_sid[0] = sid;
	rc = ipc_f(MSGGET, key, msgflag, 0, 0, 0);
	tsec->in_sid[0] = 0;

	return rc;
}

int sys_semget_secure(key_t key, int nsems, int semflag, security_id_t sid)
{
	long (*ipc_f)(uint call, int first, int second, 
		      int third, void *ptr, long fifth);
	struct task_security_struct *tsec;
	int rc;

	ipc_f = SELINUX_SYSCALL_GET(__NR_ipc);

	tsec = current->security;
	tsec->in_sid[0] = sid;
	rc = ipc_f(SEMGET, key, nsems, semflag, 0, 0);
	tsec->in_sid[0] = 0;

	return rc;
}


int sys_shmget_secure (key_t key, int size, int flag, security_id_t sid)
{
	long (*ipc_f)(uint call, int first, int second, 
		      int third, void *ptr, long fifth);
	struct task_security_struct *tsec;
	int rc;

	ipc_f = SELINUX_SYSCALL_GET(__NR_ipc);

	tsec = current->security;
	tsec->in_sid[0] = sid;
	rc = ipc_f(SHMGET, key, size, flag, 0, 0);
	tsec->in_sid[0] = 0;

	return rc;
}


int sys_msgsnd_secure(int msqid, void *msgp, size_t msgsz, int msgflg,
		      security_id_t sid)
{
	long (*ipc_f)(uint call, int first, int second, 
		      int third, void *ptr, long fifth);
	struct task_security_struct *tsec;
	int rc;

	ipc_f = SELINUX_SYSCALL_GET(__NR_ipc);

	tsec = current->security;
	tsec->in_sid[0] = sid;
	rc = ipc_f(MSGSND, msqid, msgsz, msgflg, msgp, 0);
	tsec->in_sid[0] = 0;

	return rc;
}


int sys_msgrcv_secure(int msqid, void *msgp, size_t msgsz, long msgtyp,
		      int msgflg, security_id_t *out_sid)
{
	long (*ipc_f)(uint call, int first, int second, 
		      int third, void *ptr, long fifth);
	struct task_security_struct *tsec;
	uint call;
	int rc;

	if (!out_sid)
		return -EFAULT;

	tsec = current->security;

	ipc_f = SELINUX_SYSCALL_GET(__NR_ipc);

	if (copy_from_user(&tsec->in_sid[0], out_sid, sizeof(security_id_t)))
		return -EFAULT;

	/*
	 * Don't use default version (0), since sys_ipc will then
	 * perform a copy_from_user on an expected struct ipc_kludge
	 * pointer parameter.  Otherwise, note that the order of the
	 * parameters is different.  
	 */
	call = IPCCALL(1,MSGRCV);

	rc = ipc_f(call, msqid, msgsz, msgflg, msgp, msgtyp );
	tsec->in_sid[0] = 0;

	if (copy_to_user(out_sid, &tsec->out_sid[0], sizeof(security_id_t)))
		return -EFAULT;

	return rc;
}

#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
int sys_socket_secure(unsigned long *args, security_id_t sid)
{
	long (*socketcall_f)(int call, unsigned long *args);
	struct task_security_struct *tsec;
	int rc;

	socketcall_f = SELINUX_SYSCALL_GET(__NR_socketcall);

	tsec = current->security;
	tsec->in_sid[0] = sid;
	rc = socketcall_f(SYS_SOCKET, args);
	tsec->in_sid[0] = 0;

	return rc;
}
#else
int sys_socket_secure(unsigned long *args, security_id_t sid)
{
	return -ENOSYS;
}
#endif

#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
int sys_accept_secure(unsigned long *args, security_id_t *out_sid)
{
	long (*socketcall_f)(int call, unsigned long *args);
	struct task_security_struct *tsec;
	int rc;

	if (!out_sid)
		return -EFAULT;

	tsec = current->security;

	socketcall_f = SELINUX_SYSCALL_GET(__NR_socketcall);
	rc = socketcall_f(SYS_ACCEPT, args);

	if (rc < 0)
		return rc;

	if (out_sid) {
		if (copy_to_user(out_sid, &tsec->out_sid[0], 
				 sizeof(security_id_t)))
			return -EFAULT;
	}

	return rc;
}
#else
int sys_accept_secure(unsigned long *args, security_id_t *out_sid)
{
	return -ENOSYS;
}
#endif

#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
int sys_connect_secure(unsigned long *args, security_id_t sid)
{
	long (*socketcall_f)(int call, unsigned long *args);
	struct task_security_struct *tsec;
	int rc;

	socketcall_f = SELINUX_SYSCALL_GET(__NR_socketcall);

	tsec = current->security;
	tsec->in_sid[0] = sid;
	rc = socketcall_f(SYS_CONNECT, args);
	tsec->in_sid[0] = 0;

	return rc;
}
#else
int sys_connect_secure(unsigned long *args, security_id_t sid)
{
	return -ENOSYS;
}
#endif


#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
int sys_listen_secure(unsigned long *args, security_id_t sid, int useclient)
{
	long (*socketcall_f)(int call, unsigned long *args);
	struct task_security_struct *tsec;
	int rc;

	socketcall_f = SELINUX_SYSCALL_GET(__NR_socketcall);

	tsec = current->security;
	tsec->in_sid[0] = sid;
	tsec->in_sid[1] = useclient;
	rc = socketcall_f(SYS_LISTEN, args);
	tsec->in_sid[0] = 0;
	tsec->in_sid[1] = 0;

	return rc;
}
#else
int sys_listen_secure(unsigned long *args, security_id_t sid, int useclient)
{
	return -ENOSYS;
}
#endif

#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
int sys_getpeername_secure(unsigned long *args, security_id_t *out_sid)
{
	long (*socketcall_f)(int call, unsigned long *args);
	struct task_security_struct *tsec;
	int rc;

	if (!out_sid)
		return -EFAULT;

	tsec = current->security;

	socketcall_f = SELINUX_SYSCALL_GET(__NR_socketcall);
	rc = socketcall_f(SYS_GETPEERNAME, args);

	if (rc)
		return rc;

	if (out_sid) {
		if (copy_to_user(out_sid, &tsec->out_sid[0], 
				 sizeof(security_id_t)))
			return -EFAULT;
	}

	return rc;
}
#else
int sys_getpeername_secure(unsigned long *args, security_id_t *out_sid)
{
	return -ENOSYS;
}
#endif

#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
int sys_getsockname_secure(unsigned long *args, security_id_t *out_sid)
{
	long (*socketcall_f)(int call, unsigned long *args);
	struct task_security_struct *tsec;
	int rc;

	if (!out_sid)
		return -EFAULT;

	tsec = current->security;

	socketcall_f = SELINUX_SYSCALL_GET(__NR_socketcall);
	rc = socketcall_f(SYS_GETSOCKNAME, args);

	if (rc)
		return rc;

	if (out_sid) {
		if (copy_to_user(out_sid, &tsec->out_sid[0], 
				 sizeof(security_id_t)))
			return -EFAULT;
	}

	return rc;
}
#else
int sys_getsockname_secure(unsigned long *args, security_id_t *out_sid)
{
	return -ENOSYS;
}
#endif

#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
int sys_recvfrom_secure(unsigned long *args, security_id_t *sso_sidp, 
			security_id_t *msg_sidp)
{
	long (*socketcall_f)(int call, unsigned long *args);
	struct task_security_struct *tsec;
	int rc;

	if (!sso_sidp || !msg_sidp)
		return -EFAULT;

	tsec = current->security;

	socketcall_f = SELINUX_SYSCALL_GET(__NR_socketcall);
	rc = socketcall_f(SYS_RECVFROM, args);

	if (rc < 0)
		return rc;

	if (sso_sidp) {
		if (copy_to_user(sso_sidp, &tsec->out_sid[0], 
				 sizeof(security_id_t)))
			return -EFAULT;
	}

	if (msg_sidp) {
		if (copy_to_user(msg_sidp, &tsec->out_sid[1], 
				 sizeof(security_id_t)))
			return -EFAULT;
	}

	return rc;
}
#else
int sys_recvfrom_secure(unsigned long *args, security_id_t *sso_sidp, 
			security_id_t *msg_sidp)
{
	return -ENOSYS;
}
#endif

#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
int sys_recvmsg_secure(unsigned long *args, security_id_t *sso_sidp, 
		       security_id_t *msg_sidp)
{
	long (*socketcall_f)(int call, unsigned long *args);
	struct task_security_struct *tsec;
	int rc;

	if (!sso_sidp || !msg_sidp)
		return -EFAULT;

	tsec = current->security;

	socketcall_f = SELINUX_SYSCALL_GET(__NR_socketcall);
	rc = socketcall_f(SYS_RECVMSG, args);

	if (rc < 0)
		return rc;

	if (sso_sidp) {
		if (copy_to_user(sso_sidp, &tsec->out_sid[0], 
				 sizeof(security_id_t)))
			return -EFAULT;
	}

	if (msg_sidp) {
		if (copy_to_user(msg_sidp, &tsec->out_sid[1], 
				 sizeof(security_id_t)))
			return -EFAULT;
	}

	return rc;
}
#else
int sys_recvmsg_secure(unsigned long *args, security_id_t *sso_sidp, 
		       security_id_t *msg_sidp)
{
	return -ENOSYS;
}
#endif

#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
int sys_sendmsg_secure(unsigned long *args, security_id_t dso_sid, 
		       security_id_t msg_sid)
{
	long (*socketcall_f)(int call, unsigned long *args);
	struct task_security_struct *tsec;
	int rc;

	socketcall_f = SELINUX_SYSCALL_GET(__NR_socketcall);

	tsec = current->security;
	tsec->in_sid[0] = dso_sid;
	tsec->in_sid[1] = msg_sid;
	rc = socketcall_f(SYS_SENDMSG, args);
	tsec->in_sid[0] = 0;
	tsec->in_sid[1] = 0;

	return rc;
}
#else
int sys_sendmsg_secure(unsigned long *args, security_id_t dso_sid, 
		       security_id_t msg_sid)
{
	return -ENOSYS;
}
#endif

#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
int sys_sendto_secure(unsigned long *args, security_id_t dso_sid, 
		      security_id_t msg_sid)
{
	long (*socketcall_f)(int call, unsigned long *args);
	struct task_security_struct *tsec;
	int rc;

	socketcall_f = SELINUX_SYSCALL_GET(__NR_socketcall);

	tsec = current->security;
	tsec->in_sid[0] = dso_sid;
	tsec->in_sid[1] = msg_sid;
	rc = socketcall_f(SYS_SENDTO, args);
	tsec->in_sid[0] = 0;
	tsec->in_sid[1] = 0;

	return rc;
}
#else
int sys_sendto_secure(unsigned long *args, security_id_t dso_sid, 
		      security_id_t msg_sid)
{
	return -ENOSYS;
}
#endif

long sys_execve_secure(const char *path,
		       char **argv,
		       char **envp,
		       security_id_t sid,
		       struct pt_regs *regp)
{
	int error;
	char * filename;
	struct task_security_struct *tsec;

	tsec = current->security;

	filename = getname(path);
	error = PTR_ERR(filename);
	if (IS_ERR(filename))
		goto out;
	tsec->in_sid[0] = sid;
	error = do_execve(filename, argv, envp, regp);
	tsec->in_sid[0] = 0;
	if (error == 0)
		current->ptrace &= ~PT_DTRACE;
	putname(filename);
out:
	return error;
}

#if !defined(__alpha__) && !defined(__ia64__) && !defined(__mips64) && !defined(__x86_64__) && !defined(CONFIG_ARCH_S390X)

long sys_lstat64_stat64_secure(int follow_link,
			       const char *pathname, 
			       struct stat64 *buf,
			       security_id_t *out_sid)
{
	long (*stat64_f)(const char * filename, struct stat64 * statbuf);
	struct task_security_struct *tsec;
	int rc;

	if (follow_link) 
		stat64_f = SELINUX_SYSCALL_GET(__NR_stat64);
	else
		stat64_f = SELINUX_SYSCALL_GET(__NR_lstat64);

	tsec = current->security;

	rc = stat64_f(pathname, buf);
	if (rc)
		return rc;

	if (out_sid) {
		if (copy_to_user(out_sid, &tsec->out_sid[0], sizeof(security_id_t)))
			return -EFAULT;
	}
	
	return 0;
}

long sys_fstat64_secure(unsigned int fd,
			struct stat64 *buf,
			security_id_t *out_sid)
{
	long (*fstat64_f)(unsigned int fd, struct stat64 * statbuf);
	struct task_security_struct *tsec;
	int rc;

	fstat64_f = SELINUX_SYSCALL_GET(__NR_fstat64);

	tsec = current->security;
	rc = fstat64_f(fd, buf);
	if (rc)
		return rc;

	if (out_sid) {
		if (copy_to_user(out_sid, &tsec->out_sid[0], sizeof(security_id_t)))
			return -EFAULT;
	}
	
	return 0;
}

#endif

/* Argument list sizes for sys_security_selinux_worker */
#define AL(x) ((x) * sizeof(unsigned long))
static unsigned char nargs[SELINUXCALL_NUM+1]= {
	AL(0), /* null */
	AL(2), /* compute_av */
	AL(0), /* unused */
	AL(4), /* transition_sid */
	AL(4), /* member_sid */
	AL(3), /* sid_to_context */
	AL(3), /* context_to_sid */
	AL(2), /* load_policy */
	AL(4), /* change_sid */
	AL(2), /* get_sids */
	AL(5), /* get_user_sids */
	AL(0), /* avc_toggle */
	AL(0), /* getsid */
	AL(0), /* getosid */
	AL(3), /* lstat */
	AL(2), /* lchsid */
	AL(3), /* stat */
	AL(2), /* chsid */
	AL(3), /* fstat */
	AL(2), /* fchsid */
	AL(4), /* open */
	AL(3), /* mkdir */
	AL(4), /* mknod */
	AL(3), /* symlink */
	AL(3), /* statfs */
	AL(3), /* fstatfs */
	AL(3), /* chsidfs */
	AL(3), /* fchsidfs */
	AL(2), /* shmsid */
	AL(2), /* semsid */
	AL(2), /* msgsid */
	AL(4), /* shmget */
	AL(4), /* semget */
	AL(3), /* msgget */
	AL(5), /* msgsnd */
	AL(6), /* msgrcv */
	AL(4), /* execve */
	AL(3), /* stat64 */
	AL(3), /* lstat64 */
	AL(3), /* fstat64 */
	AL(0), /* avc_enforcing */
	AL(4), /* socket */
	AL(4), /* accept */
	AL(4), /* connect */
	AL(4), /* listen */
	AL(4), /* getpeername */
	AL(4), /* getsockname */
	AL(8), /* recvfrom */
	AL(5), /* recvmsg */
	AL(5), /* sendmsg */
	AL(8), /* sendto */
	AL(0), /* mls */
	AL(3)  /* ichsid */
};
#undef AL

long sys_security_selinux_worker(unsigned int magic,
				 unsigned int call, 
				 unsigned long* args, 
				 struct pt_regs* regp)
{
	unsigned long a[8];
	unsigned long a0,a1;
	int err = -EINVAL;

	if (magic != SELINUX_MAGIC)
		return -ENOSYS;

	if(call<1||call>SELINUXCALL_NUM)
		return -EINVAL;

	if (nargs[call] > (sizeof a))
		return -EFAULT;

	/* copy_from_user should be SMP safe. */
	if (copy_from_user(a, args, nargs[call]))
		return -EFAULT;
		
	a0=a[0];
	a1=a[1];
	
	switch(call) {
	case SELINUXCALL_COMPUTE_AV:
		err = sys_security_compute_av((struct security_query *)a0,
					      (struct security_response*)a1);
		break;
	case SELINUXCALL_TRANSITION_SID:
		err = sys_security_transition_sid(a0,a1,a[2],(security_id_t*)a[3]);
		break;
	case SELINUXCALL_MEMBER_SID:
		err = sys_security_member_sid(a0,a1,a[2],(security_id_t*)a[3]);
		break;
	case SELINUXCALL_SID_TO_CONTEXT:
		err = sys_security_sid_to_context(a0,
						  (security_context_t)a1,
						  (__u32 *)a[2]);
		break;
	case SELINUXCALL_CONTEXT_TO_SID:
		err = sys_security_context_to_sid((security_context_t)a0,
						  a1,
						  (security_id_t *)a[2]);
		break;
	case SELINUXCALL_LOAD_POLICY:
		err = sys_security_load_policy((char *) a0, a1);
		break;
	case SELINUXCALL_CHANGE_SID:
		err = sys_security_change_sid(a0,a1,a[2],
					      (security_id_t*)a[3]);
		break;
	case SELINUXCALL_GET_SIDS:
		err = sys_security_get_sids((security_id_t*)a0,
					    (__u32*)a1);
		break;
	case SELINUXCALL_GET_USER_SIDS:
		err = sys_security_get_user_sids(a0,
						 (char *) a1,
						 a[2],
						 (security_id_t*)a[3],
						 (__u32*)a[4]);
		break;
	case SELINUXCALL_AVC_TOGGLE:
		err = sys_avc_toggle();
		break;
	case SELINUXCALL_GETOSECSID:
		err = sys_getosecsid();
		break;
	case SELINUXCALL_GETSECSID:
		err = sys_getsecsid();
		break;
	case SELINUXCALL_LSTAT:
		err = sys_lstat_stat_secure(0,
					    (const char *)a0, 
					    (struct stat*)a1, 
					    (security_id_t*)a[2]);
		break;
	case SELINUXCALL_STAT:
		err = sys_lstat_stat_secure(1,
					    (const char *)a0, 
					    (struct stat*)a1, 
					    (security_id_t*)a[2]);
		break;
	case SELINUXCALL_FSTAT:
		err = sys_fstat_secure(a0, 
				       (struct stat*)a1, 
				       (security_id_t*)a[2]);
		break;
	case SELINUXCALL_LCHSID:
		err = sys_lchsid_chsid(0, (const char*)a0, a1);
		break;
	case SELINUXCALL_CHSID:
		err = sys_lchsid_chsid(1, (const char*)a0, a1);
		break;
	case SELINUXCALL_FCHSID:
		err = sys_fchsid(a0, a1);
		break;
	case SELINUXCALL_OPEN:
		err = sys_open_secure((const char *)a0,a1,a[2],a[3]);
		break;
	case SELINUXCALL_MKDIR:
		err = sys_mkdir_secure((const char *)a0,a1,a[2]);
		break;
	case SELINUXCALL_MKNOD:
		err = sys_mknod_secure((const char *)a0,a1,a[2],a[3]);
		break;
	case SELINUXCALL_SYMLINK:
		err = sys_symlink_secure((const char *)a0,(const char *)a1,a[2]);
		break;
	case SELINUXCALL_STATFS:
		err = sys_statfs_secure((const char *)a0,
					(struct statfs *)a1,
					(security_id_t *)a[2]);
		break;
	case SELINUXCALL_FSTATFS:
		err = sys_fstatfs_secure(a0,
					(struct statfs *)a1,
					(security_id_t *)a[2]);
	case SELINUXCALL_CHSIDFS:
		err = sys_chsidfs((const char *)a0,a1,a[2]);
		break;
	case SELINUXCALL_FCHSIDFS:
		err = sys_fchsidfs(a0,a1,a[2]);
		break;
	case SELINUXCALL_MSGGET:
		err = sys_msgget_secure(a0, a1, a[2]);
		break;
	case SELINUXCALL_SEMGET:
		err = sys_semget_secure(a0, a1, a[2], a[3]);
		break;
	case SELINUXCALL_SHMGET:
		err = sys_shmget_secure(a0, a1, a[2], a[3]);
		break;
	case SELINUXCALL_MSGSND:
		err = sys_msgsnd_secure(a0, (void *)a1, a[2], a[3], a[4]);
		break;
	case SELINUXCALL_MSGRCV:
		err = sys_msgrcv_secure(a0, (void *)a1, a[2], a[3], a[4], 
					(security_id_t *)a[5]);
		break;
	case SELINUXCALL_SEMSID:
		err = sys_semsid(a0, (security_id_t *)a1);
		break;
	case SELINUXCALL_SHMSID:
		err = sys_shmsid(a0, (security_id_t *)a1);
		break;
	case SELINUXCALL_MSGSID:
		err = sys_msgsid(a0, (security_id_t *)a1);
		break;
	case SELINUXCALL_EXECVE:
		err = sys_execve_secure((char *)a0, (char**)a1,
					(char**)a[2], a[3], regp);
		break;
#if !defined(__alpha__) && !defined(__ia64__) && !defined(__mips64) && !defined(__x86_64__) && !defined(CONFIG_ARCH_S390X)
	case SELINUXCALL_LSTAT64:
		err = sys_lstat64_stat64_secure(0,
					    (const char *)a0, 
					    (struct stat64*)a1, 
					    (security_id_t*)a[2]);
		break;
	case SELINUXCALL_STAT64:
		err = sys_lstat64_stat64_secure(1,
					    (const char *)a0, 
					    (struct stat64*)a1, 
					    (security_id_t*)a[2]);
		break;
	case SELINUXCALL_FSTAT64:
		err = sys_fstat64_secure(a0, 
				       (struct stat64*)a1, 
				       (security_id_t*)a[2]);
		break;
#endif
	case SELINUXCALL_AVC_ENFORCING:
		err = sys_avc_enforcing();
		break;
	case SELINUXCALL_SOCKET:
		err = sys_socket_secure(args, a[3]);
		break;
	case SELINUXCALL_ACCEPT:
		err = sys_accept_secure(args, (security_id_t *)a[3]);
		break;
	case SELINUXCALL_CONNECT:
		err = sys_connect_secure(args, a[3]);
		break;
	case SELINUXCALL_LISTEN:
		err = sys_listen_secure(args, a[2], a[3]);
		break;
	case SELINUXCALL_GETPEERNAME:
		err = sys_getpeername_secure(args, (security_id_t *)a[3]);
		break;
	case SELINUXCALL_GETSOCKNAME:
		err = sys_getsockname_secure(args, (security_id_t *)a[3]);
		break;
	case SELINUXCALL_RECVFROM:
		err = sys_recvfrom_secure(args, (security_id_t *)a[6],
					  (security_id_t *)a[7]);
		break;
	case SELINUXCALL_RECVMSG:
		err = sys_recvmsg_secure(args, (security_id_t *)a[3],
					 (security_id_t *)a[4]);
		break;
	case SELINUXCALL_SENDMSG:
		err = sys_sendmsg_secure(args, a[3], a[4]);
		break;
	case SELINUXCALL_SENDTO:
		err = sys_sendto_secure(args, a[6], a[7]);
		break;
	case SELINUXCALL_MLS:
		err = sys_security_mls();
		break;
	case SELINUXCALL_ICHSID:
		err = sys_ichsid((const char*)a0, a1, a[2]);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}
