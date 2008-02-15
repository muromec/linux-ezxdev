
/* -*- linux-c -*- */

/*
 * Author : Stephen Smalley, <sds@epoch.ncsc.mil> 
 */

#ifndef _LINUX_SECURITY_H_
#define _LINUX_SECURITY_H_

/*
 * Security server interface.
 */

#include <linux/flask/flask.h>

/* Initialize the security server */
int security_init(void); 

/*
 * Compute access vectors based on a SID pair for
 * the permissions in a particular class.
 */
int security_compute_av(
	security_id_t ssid,			/* IN */
	security_id_t tsid,			/* IN */
	security_class_t tclass,		/* IN */
	access_vector_t requested,		/* IN */
	access_vector_t *allowed,		/* OUT */
	access_vector_t *decided,		/* OUT */
	access_vector_t *auditallow,		/* OUT */
	access_vector_t *auditdeny,		/* OUT */
	__u32 *seqno);			/* OUT */

/*
 * Compute a SID to use for labeling a new object in the 
 * class `tclass' based on a SID pair.  
 */
int security_transition_sid(
	security_id_t ssid,			/* IN */
	security_id_t tsid,			/* IN */
	security_class_t tclass,		/* IN */
	security_id_t *out_sid);	        /* OUT */

/*
 * Compute a SID to use when selecting a member of a 
 * polyinstantiated object of class `tclass' based on 
 * a SID pair.
 */
int security_member_sid(
	security_id_t ssid,			/* IN */
	security_id_t tsid,			/* IN */
	security_class_t tclass,		/* IN */
	security_id_t *out_sid);	        /* OUT */

/*
 * Compute a SID to use for relabeling an object in the 
 * class `tclass' based on a SID pair.  
 */
int security_change_sid(
	security_id_t ssid,			/* IN */
	security_id_t tsid,			/* IN */
	security_class_t tclass,		/* IN */
	security_id_t *out_sid);	        /* OUT */

/*
 * Write the security context string representation of 
 * the context associated with `sid' into a dynamically
 * allocated string of the correct size.  Set `*scontext'
 * to point to this string and set `*scontext_len' to
 * the length of the string.
 */
int security_sid_to_context(
	security_id_t  sid,			/* IN */
	security_context_t *scontext,		/* OUT */
	__u32   *scontext_len);			/* OUT */

/*
 * Return a SID associated with the security context that
 * has the string representation specified by `scontext'.
 */
int security_context_to_sid(
	security_context_t scontext,		/* IN */
	__u32   scontext_len,			/* IN */
	security_id_t *out_sid);		/* OUT */

/*
 * Return the SIDs to use for an unlabeled file system
 * that is being mounted from the device with the
 * the kdevname `name'.  The `fs_sid' SID is returned for 
 * the file system and the `file_sid' SID is returned
 * for all files within that file system.
 */
int security_fs_sid(
	char *dev,				/* IN */
	security_id_t *fs_sid,			/* OUT  */
	security_id_t *file_sid);		/* OUT */

/*
 * Return the SID of the port specified by
 * `domain', `type', `protocol', and `port'.
 */
int security_port_sid(
	__u16 domain,
	__u16 type,
	__u8 protocol,
	__u16 port,
	security_id_t *out_sid);

/*
 * Return the SIDs to use for a network interface
 * with the name `name'.  The `if_sid' SID is returned for 
 * the interface and the `msg_sid' SID is returned as
 * the default SID for messages received on the
 * interface.
 */
int security_netif_sid(
	char *name,
	security_id_t *if_sid,
	security_id_t *msg_sid);

/*
 * Return the SID of the node specified by the address
 * `addr' where `addrlen' is the length of the address
 * in bytes and `domain' is the communications domain or
 * address family in which the address should be interpreted.
 */
int security_node_sid(
	__u16 domain,
	void *addr,
	__u32 addrlen,
	security_id_t *out_sid);

/*
 * Return a value indicating how to handle labeling for the
 * the specified filesystem type, and optionally return a SID
 * for the filesystem object (overridden by PSID).  
 */
#define SECURITY_FS_USE_PSID  1 /* use persistent label mapping */
#define SECURITY_FS_USE_TRANS 2 /* use transition SIDs, e.g. devpts/tmpfs */
#define SECURITY_FS_USE_TASK  3 /* use task SIDs, e.g. pipefs/sockfs */
#define SECURITY_FS_USE_GENFS 4 /* use the genfs support */
#define SECURITY_FS_USE_NONE  5 /* no labeling support */
int security_fs_use(
	const char *fstype,                     /* IN */
	unsigned int *behavior,                 /* OUT */
	security_id_t *sid);			/* OUT  */

/*
 * Return the SID to use for a file in a filesystem
 * that cannot support a persistent label mapping or use another
 * fixed labeling behavior like transition SIDs or task SIDs.
 */
int security_genfs_sid(
	const char *fstype,                     /* IN */
	char *name,				/* IN */
	security_class_t sclass,                /* IN */
	security_id_t *sid);			/* OUT  */

#endif /* _LINUX_SECURITY_H_ */

