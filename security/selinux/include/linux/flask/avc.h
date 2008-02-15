
/* -*- linux-c -*- */

/*
 * Author : Stephen Smalley, <sds@epoch.ncsc.mil> 
 */

#ifndef _LINUX_AVC_H_
#define _LINUX_AVC_H_

/*
 * Access vector cache interface for object managers
 */

#include <linux/flask/flask.h>
#include <linux/flask/av_permissions.h>
#include <linux/flask/security.h>
#include <linux/stddef.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/spinlock.h>
#include <asm/system.h>

#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
extern int selinux_enforcing;
#endif


/*
 * An entry in the AVC.
 */
typedef struct avc_entry {
	security_id_t   ssid;
	security_id_t   tsid;
	security_class_t tclass;
	access_vector_t allowed;       
	access_vector_t decided; 
        access_vector_t auditallow;
	access_vector_t auditdeny;
	int		used;	 /* used recently */
} avc_entry_t;


/*
 * A reference to an AVC entry.
 */
typedef struct avc_entry_ref {  
	avc_entry_t *ae;	
} avc_entry_ref_t;

#define AVC_ENTRY_REF_NULL { NULL }

/* Initialize an AVC entry reference before first use. */
#define AVC_ENTRY_REF_INIT(h) { (h)->ae = NULL; }

#define AVC_ENTRY_REF_CPY(dst,src) (dst)->ae = (src)->ae


struct dentry;
struct inode;
struct sock;
struct sk_buff;

typedef struct avc_audit_data {
	char    type;
#define AVC_AUDIT_DATA_FS   1	
#define AVC_AUDIT_DATA_NET  2
#define AVC_AUDIT_DATA_CAP  3
#define AVC_AUDIT_DATA_IPC  4
#define AVC_AUDIT_DATA_DONTAUDIT 5 /* never audit this permission check */
	char dontcalldcache; /* no dcache calls to generate exe or path. */
	union 	{
		struct {
			struct dentry *dentry;
			struct inode *inode;
		} fs;
		struct {
			char *netif;
			struct sk_buff *skb;
			struct sock *sk;
			__u16 port;
			__u32 daddr;
		} net;
		int cap;
		int ipc_id;
	} u;
} avc_audit_data_t;

/* Initialize an AVC audit data structure. */
#define AVC_AUDIT_DATA_INIT(_d,_t) \
        { memset((_d), 0, sizeof(struct avc_audit_data)); (_d)->type = AVC_AUDIT_DATA_##_t; }


extern spinlock_t avc_lock;


/* 
 * AVC statistics
 */
#define AVC_ENTRY_LOOKUPS        0
#define AVC_ENTRY_HITS	         1
#define AVC_ENTRY_MISSES         2
#define AVC_ENTRY_DISCARDS       3
#define AVC_CAV_LOOKUPS          4
#define AVC_CAV_HITS             5
#define AVC_CAV_PROBES           6
#define AVC_CAV_MISSES           7
#define AVC_NSTATS               8
extern unsigned avc_cache_stats[AVC_NSTATS];
void avc_dump_stats(char *tag);

#ifdef AVC_CACHE_STATS
#define avc_cache_stats_incr(x) avc_cache_stats[(x)]++
#define avc_cache_stats_add(x,y) avc_cache_stats[(x)] += (y)
#else
#define avc_cache_stats_incr(x) 
#define avc_cache_stats_add(x,y) 
#endif


/*
 * AVC display support
 */
void avc_dump_av(
	security_class_t tclass,	/* IN */
	access_vector_t av);		/* IN */

void avc_dump_query(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass);	/* IN */

void avc_dump_cache(char *tag);


/*
 * AVC operations
 */

/* Initialize the AVC */
void avc_init(void);

/*
 * Look up an AVC entry that is valid for the 
 * `requested' permissions between the SID pair
 * (`ssid', `tsid'), interpreting the permissions
 * based on `tclass'.  If a valid AVC entry exists,
 * then this function updates `aeref' to refer to the
 * entry and returns 0. Otherwise, this function
 * returns -ENOENT.
 */
int avc_lookup(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t requested,	/* IN */
	avc_entry_ref_t *aeref);	/* OUT */

/*
 * Insert an AVC entry for the SID pair
 * (`ssid', `tsid') and class `tclass'. 
 * The access vectors and the sequence number are
 * normally provided by the security server in 
 * response to a security_compute_av call.  If the 
 * sequence number `seqno' is not less than the latest
 * revocation notification, then the function copies
 * the access vectors into a cache entry, updates 
 * `aeref' to refer to the entry, and returns 0.
 * Otherwise, this function returns -EAGAIN.
 */
int avc_insert(security_id_t ssid,		/* IN */
	       security_id_t tsid,		/* IN */
	       security_class_t tclass,		/* IN */
	       struct avc_entry *ae,		/* IN */
	       __u32 seqno,			/* IN */
	       avc_entry_ref_t *out_aeref);	/* OUT */


/* Audit the checking of permissions */
#define AVC_AUDITALLOW 0
#define AVC_AUDITDENY 1
void avc_audit(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms,		/* IN */
	struct avc_entry *ae,		/* IN */
	__u32 denied,			/* IN */
	avc_audit_data_t *auditdata);   /* IN */

/*
 * Check permissions using an AVC entry ref.
 *
 * If the ref is null or the underlying AVC entry has been invalidated
 * or the underlying AVC entry does not contain all the requested 
 * decisions, then this code falls through to avc_lookup.  In
 * this case, the AVC entry ref will be updated appropriately.
 */
static int avc_has_perm_ref_audit(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t requested, 	/* IN */
	avc_entry_ref_t *aeref,		/* IN */
	avc_audit_data_t *auditdata)	/* IN */
{
	struct avc_entry *ae;
	int             rc;
	unsigned long	flags;
	struct avc_entry entry;
	__u32 seqno;
	access_vector_t denied;

	spin_lock_irqsave(&avc_lock, flags);
	avc_cache_stats_incr(AVC_ENTRY_LOOKUPS);
	ae = aeref->ae;
	if (ae) {
		if (ae->ssid == ssid &&
		    ae->tsid == tsid &&
		    ae->tclass == tclass &&
		    ((ae->decided & requested) == requested)) {
			avc_cache_stats_incr(AVC_ENTRY_HITS);
			ae->used = 1;
		} else {
			avc_cache_stats_incr(AVC_ENTRY_DISCARDS);
			ae = 0;
		}
	}

	if (!ae) {
		avc_cache_stats_incr(AVC_ENTRY_MISSES);
		rc = avc_lookup(ssid, tsid, tclass, requested, aeref);
		if (rc) {
			spin_unlock_irqrestore(&avc_lock,flags);
			rc = security_compute_av(ssid,tsid,tclass,requested,
						 &entry.allowed, 
						 &entry.decided,
						 &entry.auditallow,
						 &entry.auditdeny,
						 &seqno);
			if (rc)
				return rc;
			spin_lock_irqsave(&avc_lock, flags);
			rc = avc_insert(ssid,tsid,tclass,&entry,seqno,aeref);
			if (rc) {
				spin_unlock_irqrestore(&avc_lock,flags);
				return rc;
			}
		}
		ae = aeref->ae;
	}

	denied = requested & ~(ae->allowed);

	if (!requested || denied) {
		if (!requested || (denied & ae->auditdeny))
			avc_audit(ssid, tsid, tclass, denied, ae,
				  AVC_AUDITDENY, auditdata);
#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
		if (selinux_enforcing) {
			spin_unlock_irqrestore(&avc_lock,flags);
			return -EACCES;
		} else {
			ae->allowed |= requested; 
			spin_unlock_irqrestore(&avc_lock,flags);
			return 0;
		}
#else
		spin_unlock_irqrestore(&avc_lock,flags);
		return -EACCES;
#endif
	}

	if (requested & ae->auditallow)
		avc_audit(ssid, tsid, tclass, requested, ae, 
			  AVC_AUDITALLOW, auditdata);

	spin_unlock_irqrestore(&avc_lock,flags);
	return 0;    
}

#define avc_has_perm_ref(ssid,tsid,tclass,requested,aeref) \
        avc_has_perm_ref_audit(ssid,tsid,tclass,requested,aeref,0)


/* Check permissions */
static inline int avc_has_perm_audit(
	security_id_t ssid,	   /* IN */
	security_id_t tsid,	   /* IN */
	security_class_t tclass,   /* IN */
	access_vector_t requested, /* IN */
	avc_audit_data_t *auditdata) /* IN */
{
	avc_entry_ref_t ref;
	AVC_ENTRY_REF_INIT(&ref);
	return avc_has_perm_ref_audit(ssid,tsid,tclass,requested,&ref,auditdata);
}

#define avc_has_perm(ssid,tsid,tclass,requested) \
        avc_has_perm_audit(ssid,tsid,tclass,requested,0)

#define AVC_CALLBACK_GRANT 	1
#define AVC_CALLBACK_TRY_REVOKE 2
#define AVC_CALLBACK_REVOKE     4
#define AVC_CALLBACK_RESET      8
#define AVC_CALLBACK_AUDITALLOW_ENABLE  16
#define AVC_CALLBACK_AUDITALLOW_DISABLE 32
#define AVC_CALLBACK_AUDITDENY_ENABLE   64
#define AVC_CALLBACK_AUDITDENY_DISABLE 128

/*
 * Register a callback for events in the set `events'
 * related to the SID pair (`ssid', `tsid') and
 * and the permissions `perms', interpreting
 * `perms' based on `tclass'.
 */
int avc_add_callback(int (*callback)(__u32 event, 
				     security_id_t ssid,
				     security_id_t tsid,
				     security_class_t tclass,
				     access_vector_t perms,
				     access_vector_t *out_retained),
		     __u32 events,
		     security_id_t ssid,
		     security_id_t tsid,
		     security_class_t tclass,
		     access_vector_t perms);

extern long sys_avc_toggle(void);

#endif /* _LINUX_AVC_H_ */

