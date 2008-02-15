/* -*- linux-c -*- */

/* Author : Stephen Smalley, <sds@epoch.ncsc.mil> */

#ifndef _LINUX_FLASK_SYSCALLS_H_
#define _LINUX_FLASK_SYSCALLS_H_

#include <linux/flask/flask_types.h>

#define SELINUX_MAGIC 0xf97cff8c 

/* Call values for sys_security. */
#define SELINUXCALL_COMPUTE_AV     1
#define SELINUXCALL_NOTIFY_PERM    2
#define SELINUXCALL_TRANSITION_SID 3
#define SELINUXCALL_MEMBER_SID     4
#define SELINUXCALL_SID_TO_CONTEXT 5
#define SELINUXCALL_CONTEXT_TO_SID 6
#define SELINUXCALL_LOAD_POLICY    7
#define SELINUXCALL_CHANGE_SID     8
#define SELINUXCALL_GET_SIDS       9
#define SELINUXCALL_GET_USER_SIDS 10
#define SELINUXCALL_AVC_TOGGLE    11
#define SELINUXCALL_GETSECSID     12
#define SELINUXCALL_GETOSECSID    13
#define SELINUXCALL_LSTAT         14
#define SELINUXCALL_LCHSID        15
#define SELINUXCALL_STAT          16
#define SELINUXCALL_CHSID         17
#define SELINUXCALL_FSTAT         18
#define SELINUXCALL_FCHSID        19
#define SELINUXCALL_OPEN          20
#define SELINUXCALL_MKDIR         21
#define SELINUXCALL_MKNOD         22
#define SELINUXCALL_SYMLINK       23
#define SELINUXCALL_STATFS        24
#define SELINUXCALL_FSTATFS       25
#define SELINUXCALL_CHSIDFS       26
#define SELINUXCALL_FCHSIDFS      27

#define SELINUXCALL_SHMSID        28
#define SELINUXCALL_SEMSID        29
#define SELINUXCALL_MSGSID        30
#define SELINUXCALL_SHMGET        31
#define SELINUXCALL_SEMGET        32
#define SELINUXCALL_MSGGET        33
#define SELINUXCALL_MSGSND        34
#define SELINUXCALL_MSGRCV        35

#define SELINUXCALL_EXECVE        36

#define SELINUXCALL_STAT64        37
#define SELINUXCALL_LSTAT64       38
#define SELINUXCALL_FSTAT64       39

#define SELINUXCALL_AVC_ENFORCING 40

#define SELINUXCALL_SOCKET        41
#define SELINUXCALL_ACCEPT        42
#define SELINUXCALL_CONNECT       43
#define SELINUXCALL_LISTEN        44
#define SELINUXCALL_GETPEERNAME   45
#define SELINUXCALL_GETSOCKNAME   46
#define SELINUXCALL_RECVFROM      47
#define SELINUXCALL_RECVMSG       48
#define SELINUXCALL_SENDMSG       49
#define SELINUXCALL_SENDTO        50

#define SELINUXCALL_MLS           51

#define SELINUXCALL_ICHSID        52

#define SELINUXCALL_NUM           52

/* Structure definitions for compute_av call */
struct security_query {
	security_id_t ssid;
	security_id_t tsid;
	security_class_t tclass;
	access_vector_t requested;
};

struct security_response {
	access_vector_t allowed;
	access_vector_t decided;
	access_vector_t auditallow;
	access_vector_t auditdeny;
	access_vector_t notify;
	__u32 seqno;
};

#endif

