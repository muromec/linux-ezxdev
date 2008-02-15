
/* -*- linux-c -*- */

/*
 * Author : Stephen Smalley, <sds@epoch.ncsc.mil> 
 */

#ifndef _LINUX_AVC_SS_H_
#define _LINUX_AVC_SS_H_

/* 
 * Access vector cache interface for the security server
 */

#include <linux/flask/flask.h>

/*
 * Any of the SID parameters may be wildcarded,
 * in which case the operation is applied to all
 * matching entries in the AVC.
 */

/* Grant previously denied permissions */
int avc_ss_grant(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms,		/* IN */
	__u32 seqno);			/* IN */

/*
 * Try to revoke previously granted permissions, but
 * only if they are not retained as migrated permissions.
 * Return the subset of permissions that are retained.
 */
int avc_ss_try_revoke(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms,		/* IN */
	__u32 seqno,			/* IN */
	access_vector_t *out_retained);	/* OUT */

/*
 * Revoke previously granted permissions, even if
 * they are retained as migrated permissions.
 */
int avc_ss_revoke(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms,		/* IN */
	__u32 seqno);			/* IN */

/* 
 * Flush the cache and revalidate all migrated permissions.
 */
int avc_ss_reset(__u32 seqno);


/* Enable or disable auditing of granted permissions */
int avc_ss_set_auditallow(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms,		/* IN */
	__u32 seqno,			/* IN */
	__u32 enable);

/* Enable or disable auditing of denied permissions */
int avc_ss_set_auditdeny(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms,		/* IN */
	__u32 seqno,			/* IN */
	__u32 enable);

#endif /* _LINUX_AVC_SS_H_ */

