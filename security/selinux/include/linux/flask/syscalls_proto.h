/* -*- linux-c -*- */

#ifndef _LINUX_FLASK_SYSCALLS_PROTO_H_
#define _LINUX_FLASK_SYSCALLS_PROTO_H_

long sys_getsecsid(void);
long sys_getosecsid(void);

long sys_lstat_stat_secure(int follow_link,
			   const char *pathname, 
			   struct stat *buf,
			   security_id_t *out_sid);
long sys_fstat_secure(unsigned int fd,
		      struct stat *buf,
		      security_id_t *out_sid);
long sys_lchsid_chsid(int follow_link,
		      const char *filename, 
		      security_id_t sid);
long sys_fchsid(unsigned int fd, security_id_t sid);
long sys_ichsid(const char *filename, 
		unsigned long ino,
		security_id_t sid);
long sys_open_secure(const char * filename, int flags, int mode,
		     security_id_t sid);
long sys_mkdir_secure(const char * pathname, int mode,
		      security_id_t sid);
long sys_mknod_secure(const char * filename, int mode, dev_t dev,
		      security_id_t sid);
long sys_symlink_secure(const char * oldname, const char * newname,
			security_id_t sid);
long sys_statfs_secure(const char *pathname, 
		       struct statfs *buf,
		       security_id_t *out_sid);
long sys_fstatfs_secure(unsigned int fd,
			struct statfs *buf,
			security_id_t *out_sid);
long sys_chsidfs(const char * path, 
		 security_id_t fs_sid,
		 security_id_t f_sid);
long sys_fchsidfs(unsigned int fd, 
		  security_id_t fs_sid,
		  security_id_t f_sid);

int sys_semsid(int semid, security_id_t *out_sid);
int sys_shmsid(int shmid, security_id_t *out_sid);
int sys_msgsid(int msqid, security_id_t *out_sid);
int sys_msgget_secure(key_t key, int msgflag, security_id_t sid);
int sys_semget_secure(key_t key, int nsems, int semflag, security_id_t sid);
int sys_shmget_secure (key_t key, int size, int flag, security_id_t sid);
int sys_msgsnd_secure(int msqid, void *msgp, size_t msgsz, int msgflg,
		      security_id_t sid);
int sys_msgrcv_secure(int msqid, void *msgp, size_t msgsz, long msgtyp,
		      int msgflg, security_id_t *out_sid);

int sys_socket_secure(unsigned long *args, security_id_t sid);
int sys_accept_secure(unsigned long *args, security_id_t *out_sid);
int sys_connect_secure(unsigned long *args, security_id_t sid);
int sys_listen_secure(unsigned long *args, security_id_t sid, int useclient);
int sys_getpeername_secure(unsigned long *args, security_id_t *out_sid);
int sys_getsockname_secure(unsigned long *args, security_id_t *out_sid);
int sys_recvfrom_secure(unsigned long *args, security_id_t *sso_sidp, 
			security_id_t *msg_sidp);
int sys_recvmsg_secure(unsigned long *args, security_id_t *sso_sidp, 
		       security_id_t *msg_sidp);
int sys_sendmsg_secure(unsigned long *args, security_id_t dso_sid, 
		       security_id_t msg_sid);
int sys_sendto_secure(unsigned long *args, security_id_t dso_sid, 
		      security_id_t msg_sid);

extern long sys_security_compute_av(struct security_query *query, 
				    struct security_response *response);

extern long sys_security_sid_to_context(security_id_t sid,	
					security_context_t scontext,	
					__u32 *scontext_len);

extern long sys_security_context_to_sid(security_context_t scontext,	
					__u32 scontext_len,	
					security_id_t * out_sid);

extern long sys_security_transition_sid(security_id_t ssid,	
					security_id_t tsid,	
					security_class_t tclass, 
					security_id_t * out_sid );

extern long sys_security_change_sid(security_id_t ssid,	
				    security_id_t tsid,	
				    security_class_t tclass, 
				    security_id_t * out_sid );

extern long sys_security_member_sid(security_id_t ssid,	
				    security_id_t tsid,	
				    security_class_t tclass,	
				    security_id_t * out_sid);
extern long sys_security_load_policy(char *path,
				     __u32 pathlen);

extern long sys_security_get_sids(security_id_t * sids,
				  __u32 *nel);

extern long sys_security_get_user_sids(security_id_t fromsid,
				       char *username, 
				       __u32 namelen,
				       security_id_t * sids,
				       __u32 *nel);
extern long sys_security_mls(void);

extern long sys_avc_toggle(void);
extern long sys_avc_enforcing(void);

#endif
