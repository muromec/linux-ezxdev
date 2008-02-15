/*
 *  NSA Security-Enhanced Linux (SELinux) security module
 *
 *  This file contains the SELinux security data structures for kernel objects.
 *
 *  Author(s):  Stephen Smalley, <sds@epoch.ncsc.mil>
 *              Chris Vance, <cvance@nai.com>
 *              Wayne Salamon, <wsalamon@nai.com>
 *
 *  Copyright (C) 2001,2002 Networks Associates Technology, Inc.
 * 
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 */ 
#ifndef __SELINUX_PLUG_H
#define __SELINUX_PLUG_H

#include <linux/flask/flask.h>
#include <linux/flask/avc.h>
#include <linux/flask/psid.h>
#include <linux/flask/syscalls.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/in.h>

struct task_security_struct {
        unsigned long magic;           /* magic number for this module */
	struct task_struct *task;      /* back pointer to task object */
	struct list_head list;         /* list of task_security_struct */
	security_id_t osid;            /* SID prior to last execve */
	security_id_t sid;             /* current SID */
	security_id_t in_sid[2];       /* input SIDs */
	security_id_t out_sid[2];      /* output SIDs */
        avc_entry_ref_t avcr;          /* reference to process permissions */
};

struct sock_security_struct {
	unsigned long magic;           /* magic number for this module */
	struct sock *sk;               /* back pointer to sock object */
	struct list_head list;         /* list of sock_security_struct */
	security_id_t sid;             /* SID of the sock */
	security_id_t peer_sid;        /* SID of the network peer */
};
	
#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
struct open_request_security_struct {
	unsigned long magic;           /* magic number for this module */
	struct open_request *req;      /* back pointer to open request object */
	struct list_head list;         /* list of open_request_security_struct*/
	security_id_t newconn_sid;     /* SID of the new connection */
};
#endif

struct inode_security_struct {
	unsigned long magic;           /* magic number for this module */
        struct inode *inode;           /* back pointer to inode object */
	struct list_head list;         /* list of inode_security_struct */
	security_id_t task_sid;        /* SID of creating task */
	security_id_t sid;             /* SID of this object */
	security_class_t sclass;       /* security class of this object */
	avc_entry_ref_t avcr;          /* reference to object permissions */
	unsigned char initialized;     /* initialization flag */
	struct semaphore sem;          
	unsigned char inherit;         /* inherit SID from parent entry */
#ifdef CONFIG_SECURITY_SELINUX_EXTSOCKET
	security_id_t msid;            /* SID of message on the socket */
	security_id_t dsid;            /* SID of desired destination socket */
	security_id_t peer_sid;        /* SID of the peer socket */
	security_id_t newconn_sid;     /* SID to use for new connections */
	int useclient;                 /* Use client SID for connections */
	access_vector_t conn_perm;     /* connection permission */
#endif
};

struct file_security_struct {
	unsigned long magic;            /* magic number for this module */
	struct file *file;              /* back pointer to file object */
	struct list_head list;          /* list of file_security_struct */
	security_id_t sid;              /* SID of open file description */
	security_id_t fown_sid;         /* SID of file owner (for SIGIO) */
	avc_entry_ref_t avcr;           /* reference to fd permissions */
	avc_entry_ref_t inode_avcr;     /* reference to object permissions */
};

struct superblock_security_struct {
	unsigned long magic;            /* magic number for this module */
	struct super_block *sb;         /* back pointer to sb object */
	struct list_head list;          /* list of superblock_security_struct */
	security_id_t sid;              /* SID of file system */
	struct psidtab *psidtab;        /* persistent SID mapping */
	unsigned char uses_psids;       /* uses persistent SID flag */
	unsigned char initialized;      /* initialization flag */
	unsigned char uses_task;        /* use creating task SID for inodes */
	unsigned char uses_genfs;       /* use security_genfs_sid for inodes */
	unsigned char proc;             /* call procfs_set_sid */
        unsigned char uses_trans;       /* call security_transition_sid */
	struct semaphore sem;          
};

struct msg_security_struct {
        unsigned long magic;		/* magic number for this module */
	struct msg_msg *msg;		/* back pointer */
	struct list_head list;		/* list of msg_security_struct */
	security_id_t sid;              /* SID of message */
        avc_entry_ref_t avcr;		/* reference to permissions */
};

struct ipc_security_struct {
        unsigned long magic;		/* magic number for this module */
	struct kern_ipc_perm *ipc_perm; /* back pointer */
	security_class_t sclass;	/* security class of this object */
	struct list_head list;		/* list of ipc_security_struct */
	security_id_t sid;              /* SID of IPC resource */
        avc_entry_ref_t avcr;		/* reference to permissions */
};

struct skb_security_struct {
	unsigned long magic;            /* magic number for this module */
	struct sk_buff *skb;            /* back pointer */
	struct list_head list;          /* list of skb_security_struct */
	__u8 opts;                      /* Bitmap of current options */
	__u8 mapped;			/* Bitmap of mapped SIDs */
	__u8 invalid;			/* Security state invalidated */
	atomic_t use;                   /* reference count */
	__u32 serial;                   /* Policy ID used to label datagram */
	security_id_t ssid;             /* Source SID */
	security_id_t msid;             /* Message SID  */
	security_id_t dsid;             /* Destination SID */
	void *data;			/* Implementation specific data */
};

struct netdev_security_struct {
        unsigned long magic;		/* magic number for this module */
	struct net_device *dev;		/* back pointer to network device */
	struct list_head list;		/* list of netdev_security_struct */
	security_id_t sid;		/* SID of the network device    */
	security_id_t default_msg_sid;	/* Default SID for received messages */
        avc_entry_ref_t avcr;		/* reference to permissions */
};

static inline security_class_t inode_mode_to_security_class(umode_t mode) 
{
	switch (mode & S_IFMT) {
	case S_IFSOCK:
		return SECCLASS_SOCK_FILE;
	case S_IFLNK:
		return SECCLASS_LNK_FILE;
	case S_IFREG:
		return SECCLASS_FILE;
	case S_IFBLK:
		return SECCLASS_BLK_FILE;
	case S_IFDIR:
		return SECCLASS_DIR;
	case S_IFCHR:
		return SECCLASS_CHR_FILE;
	case S_IFIFO:
		return SECCLASS_FIFO_FILE;
		
	}

	return SECCLASS_FILE;
}

static inline security_class_t socket_type_to_security_class(int family, 
							     int type)
{
	switch (family) {
	case PF_UNIX:
		switch (type) {
		case SOCK_STREAM:
			return SECCLASS_UNIX_STREAM_SOCKET;
		case SOCK_DGRAM:
			return SECCLASS_UNIX_DGRAM_SOCKET;
		}
	case PF_INET:
	case PF_INET6:
		switch (type) {
		case SOCK_STREAM:
			return SECCLASS_TCP_SOCKET;
		case SOCK_DGRAM:
			return SECCLASS_UDP_SOCKET;
		case SOCK_RAW:
			return SECCLASS_RAWIP_SOCKET;
		}
	case PF_NETLINK:
		return SECCLASS_NETLINK_SOCKET;
	case PF_PACKET:
		return SECCLASS_PACKET_SOCKET;
	case PF_KEY:
		return SECCLASS_KEY_SOCKET;
	}

	return SECCLASS_SOCKET;
}

extern int inode_security_set_sid(struct inode *inode, security_id_t sid);

extern int task_has_system(struct task_struct *tsk, access_vector_t perms);

extern int task_has_security(struct task_struct *tsk, access_vector_t perms);

/* Range of port numbers used to automatically bind.
   Need to determine whether we should perform a name_bind 
   permission check between the socket and the port number. */
#define ip_local_port_range_0 sysctl_local_port_range[0]
#define ip_local_port_range_1 sysctl_local_port_range[1]

#endif /* __SELINUX_PLUG_H */
