
/* -*- linux-c -*- */

/*
 * Author:  Stephen Smalley, <sds@epoch.ncsc.mil>
 */

/*
 * Implementation of the kernel access vector cache (AVC).
 */

#include <linux/types.h>
#include <linux/flask/avc.h>
#include <linux/flask/avc_ss.h>
#include <linux/flask/class_to_string.h>
#include <linux/flask/common_perm_to_string.h>
#include <linux/flask/av_inherit.h>
#include <linux/flask/av_perm_to_string.h>
#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/dcache.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/un.h>
#include <net/af_unix.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/tcp.h>
#include "selinux_plug.h"

spinlock_t avc_lock = SPIN_LOCK_UNLOCKED;

typedef struct avc_node {
	struct avc_entry ae;
	struct avc_node *next;
}               avc_node_t;

static struct avc_node *avc_node_freelist = NULL;

#define AVC_CACHE_SLOTS 512 
#define AVC_CACHE_MAXNODES 410 

typedef struct {
	avc_node_t     *slots[AVC_CACHE_SLOTS];
	__u32    lru_hint;	/* LRU hint for reclaim scan */
	__u32   activeNodes;
	__u32	latest_notif;	/* latest revocation notification */
}               avc_cache_t;


static avc_cache_t avc_cache;

#define AVC_HASH(ssid,tsid,tclass) \
((ssid ^ (tsid<<2) ^ (tclass<<4)) & (AVC_CACHE_SLOTS - 1))

static char *avc_audit_buffer = NULL;

unsigned        avc_cache_stats[AVC_NSTATS];

/* 
 * Display the cache statistics 
 */
void avc_dump_stats(char *tag)
{
	printk("%s avc:  entry:  %d lookups == %d hits + %d misses (%d discards)\n",
	       tag,
	       avc_cache_stats[AVC_ENTRY_LOOKUPS],
	       avc_cache_stats[AVC_ENTRY_HITS],
	       avc_cache_stats[AVC_ENTRY_MISSES],
	       avc_cache_stats[AVC_ENTRY_DISCARDS]);

	printk("%s avc:  cav:  %d lookups == %d hits + %d misses\n",
	       tag,
	       avc_cache_stats[AVC_CAV_LOOKUPS],
	       avc_cache_stats[AVC_CAV_HITS],
	       avc_cache_stats[AVC_CAV_MISSES]);

	printk("%s avc:  cav:  %d/%d probe/hit ratio\n",
	       tag,
	       avc_cache_stats[AVC_CAV_PROBES],
	       avc_cache_stats[AVC_CAV_HITS]);
}


/* 
 * Display an access vector in human-readable form.
 */
void avc_dump_av(
	security_class_t tclass,
	access_vector_t av)
{
	char          **common_pts = 0;
	access_vector_t common_base = 0;
	int             i, i2, perm;


	if (av == 0) {
		printk(" null");
		return;
	}

	for (i = 0; i < AV_INHERIT_SIZE; i++) {
		if (av_inherit[i].tclass == tclass) {
			common_pts = av_inherit[i].common_pts;
			common_base = av_inherit[i].common_base;
			break;
		}
	}

	printk(" {");
	i = 0;
	perm = 1;
	while (perm < common_base) {
		if (perm & av)
			printk(" %s", common_pts[i]);
		i++;
		perm <<= 1;
	}

	while (i < sizeof(access_vector_t) * 8) {
		if (perm & av) {
			for (i2 = 0; i2 < AV_PERM_TO_STRING_SIZE; i2++) {
				if ((av_perm_to_string[i2].tclass == tclass) &&
				    (av_perm_to_string[i2].value == perm))
					break;
			}
			if (i2 < AV_PERM_TO_STRING_SIZE)
				printk(" %s", av_perm_to_string[i2].name);
		}
		i++;
		perm <<= 1;
	}

	printk(" }");
}


/* 
 * Display a SID pair and a class in human-readable form.
 */
void avc_dump_query(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass)	/* IN */
{
	int rc;
	security_context_t scontext;
	__u32 scontext_len;
	
 	rc = security_sid_to_context(ssid, &scontext, &scontext_len);
	if (rc)
		printk("ssid=%d", ssid);		
	else {
		printk("scontext=%s", scontext);		
		kfree(scontext);
	}

	rc = security_sid_to_context(tsid, &scontext, &scontext_len);
	if (rc)
		printk(" tsid=%d", tsid);
	else {
		printk(" tcontext=%s", scontext);
		kfree(scontext);
	}
	printk(" tclass=%s", class_to_string[tclass]);
}


/*
 * Initialize the cache.
 */
void avc_init(void)
{
	avc_node_t	*new;
	int             i;

	for (i = 0; i < AVC_NSTATS; i++)
		avc_cache_stats[i] = 0;

	for (i = 0; i < AVC_CACHE_SLOTS; i++)
		avc_cache.slots[i] = 0;
	avc_cache.lru_hint = 0;
	avc_cache.activeNodes = 0;
	avc_cache.latest_notif = 0;

	for (i = 0; i < AVC_CACHE_MAXNODES; i++) {
		new = (avc_node_t *) kmalloc(sizeof(avc_node_t),
					     GFP_ATOMIC);
		if (!new) {
			printk("avc:  only able to allocate %d entries\n", i);
			break;
		}
		memset(new, 0, sizeof(avc_node_t));
		new->next = avc_node_freelist;
		avc_node_freelist = new;
	}

	avc_audit_buffer = (char *)__get_free_page(GFP_ATOMIC);
	if (!avc_audit_buffer)
		panic("AVC:  unable to allocate audit buffer\n");
}

#if 0
static void avc_hash_eval(char *tag)
{
	int             i, chain_len, max_chain_len, slots_used;
	avc_node_t     *node;
	unsigned long	flags;

	spin_lock_irqsave(&avc_lock,flags);

	slots_used = 0;
	max_chain_len = 0;
	for (i = 0; i < AVC_CACHE_SLOTS; i++) {
		node = avc_cache.slots[i];
		if (node) {
			slots_used++;
			chain_len = 0;
			while (node) {
				chain_len++;
				node = node->next;
			}
			if (chain_len > max_chain_len)
				max_chain_len = chain_len;
		}
	}

	spin_unlock_irqrestore(&avc_lock,flags);

	printk("\n%s avc:  %d entries and %d/%d buckets used, longest chain length %d\n",
	       tag, avc_cache.activeNodes, slots_used, AVC_CACHE_SLOTS, max_chain_len);
}
#else
#define avc_hash_eval(t)
#endif

/* 
 * Display the contents of the cache in human-readable form.
 */
void avc_dump_cache(char *tag)
{
	int             i, chain_len, max_chain_len, slots_used;
	avc_node_t     *node;

	avc_dump_stats(tag);

	slots_used = 0;
	max_chain_len = 0;
	for (i = 0; i < AVC_CACHE_SLOTS; i++) {
		node = avc_cache.slots[i];
		if (node) {
			printk("\n%s avc:  slot %d:\n", tag, i);
			slots_used++;
			chain_len = 0;
			while (node) {
				avc_dump_query(node->ae.ssid, node->ae.tsid, node->ae.tclass);
				printk(" allowed");
				avc_dump_av(node->ae.tclass, node->ae.allowed);
				printk("\n");

				chain_len++;
				node = node->next;
			}

			if (chain_len > max_chain_len)
				max_chain_len = chain_len;
		}
	}

	printk("\n%s avc:  %d entries and %d/%d buckets used, longest chain length %d\n",
	       tag, avc_cache.activeNodes, slots_used, AVC_CACHE_SLOTS, max_chain_len);

	printk("%s avc:  latest_notif=%d\n", tag, avc_cache.latest_notif);
}


/*
 * Reclaim a node from the cache for use.
 */
static inline avc_node_t *avc_reclaim_node(void)
{
	avc_node_t     *prev, *cur;
	int             hvalue, try;

	hvalue = avc_cache.lru_hint;
	for (try = 0; try < 2; try++) {
		do {
			prev = NULL;
			cur = avc_cache.slots[hvalue];
			while (cur) {
				if (!cur->ae.used)
					goto found;

				cur->ae.used = 0;

				prev = cur;
				cur = cur->next;
			}
			hvalue = (hvalue + 1) & (AVC_CACHE_SLOTS - 1);
		} while (hvalue != avc_cache.lru_hint);
	}

	panic("avc_reclaim_node");

found:
	avc_cache.lru_hint = hvalue;

	if (prev == NULL)
		avc_cache.slots[hvalue] = cur->next;
	else
		prev->next = cur->next;

	return cur;
}


/*
 * Claim a node for use for a particular 
 * SID pair and class.
 */
static inline avc_node_t *avc_claim_node(
	security_id_t ssid,
	security_id_t tsid,
	security_class_t tclass)
{
	avc_node_t     *new;
	int             hvalue;


	hvalue = AVC_HASH(ssid, tsid, tclass);
	if (avc_node_freelist) {
		new = avc_node_freelist;
		avc_node_freelist = avc_node_freelist->next;
		avc_cache.activeNodes++;
	} else {
		new = avc_reclaim_node();
		if (!new)
			return NULL;
	}

	new->ae.used = 1;
	new->ae.ssid = ssid;
	new->ae.tsid = tsid;
	new->ae.tclass = tclass;
	new->next = avc_cache.slots[hvalue];
	avc_cache.slots[hvalue] = new;

	return new;
}


/*
 * Search for a node that has the specified
 * SID pair and class.
 */
static inline avc_node_t *avc_search_node(
	security_id_t ssid,
	security_id_t tsid,
	security_class_t tclass,
	int *probes)
{
	avc_node_t     *cur;
	int             hvalue;
	int             tprobes = 1;


	hvalue = AVC_HASH(ssid, tsid, tclass);
	cur = avc_cache.slots[hvalue];
	while (cur != NULL &&
	       (ssid != cur->ae.ssid ||
		tclass != cur->ae.tclass ||
		tsid != cur->ae.tsid)) {
		tprobes++;
		cur = cur->next;
	}

	if (cur == NULL) {
		/* cache miss */
		return NULL;
	}

	/* cache hit */
	if (probes)
		*probes = tprobes;

	cur->ae.used = 1;

	return cur;
}


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
	avc_entry_ref_t *aeref)		/* OUT */
{
	avc_node_t     *node;
	int             probes;

	avc_cache_stats_incr(AVC_CAV_LOOKUPS);
	node = avc_search_node(ssid, tsid, tclass,&probes);

	if (node && ((node->ae.decided & requested) == requested)) {
		avc_cache_stats_incr(AVC_CAV_HITS);
		avc_cache_stats_add(AVC_CAV_PROBES,probes);
		aeref->ae = &node->ae;
		return 0;
	} 

	avc_cache_stats_incr(AVC_CAV_MISSES);
	return -ENOENT;
}


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
	       avc_entry_ref_t *aeref)		/* OUT */
{
	avc_node_t     *node;

	if (seqno < avc_cache.latest_notif) {
		printk("avc:  seqno %d < latest_notif %d\n", seqno,
		       avc_cache.latest_notif);
		return -EAGAIN;
	}

	node = avc_claim_node(ssid, tsid, tclass);
	if (!node) {
		return -ENOMEM;
	}
	
	node->ae.allowed = ae->allowed;
	node->ae.decided = ae->decided;
	node->ae.auditallow = ae->auditallow;
	node->ae.auditdeny = ae->auditdeny;
	aeref->ae = &node->ae;
	return 0;
}

#define print_ipv4_addr(_addr,_port,_name1,_name2) { \
	if ((_addr)) \
		printk(" %s=%d.%d.%d.%d", (_name1), \
		       NIPQUAD((_addr))); \
	if ((_port)) \
		printk(" %s=%d", (_name2), ntohs((_port))); \
	}


/*
 * Copied from fs/dcache.c:d_path and hacked up to
 * avoid need for vfsmnt, root, and rootmnt parameters.
 */
char * avc_d_path(struct dentry *dentry, 
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

/*
 * Copied from net/core/utils.c:net_ratelimit and modified for
 * use by the AVC audit facility.
 */

int avc_msg_cost = 5*HZ;
int avc_msg_burst = 10*5*HZ;

/* 
 * This enforces a rate limit: not more than one kernel message
 * every 5secs to make a denial-of-service attack impossible.
 */ 
int avc_ratelimit(void)
{
	static spinlock_t ratelimit_lock = SPIN_LOCK_UNLOCKED;
	static unsigned long toks = 10*5*HZ;
	static unsigned long last_msg; 
	static int missed;
	unsigned long flags;
	unsigned long now = jiffies;

	spin_lock_irqsave(&ratelimit_lock, flags);
	toks += now - last_msg;
	last_msg = now;
	if (toks > avc_msg_burst)
		toks = avc_msg_burst;
	if (toks >= avc_msg_cost) {
		int lost = missed;
		missed = 0;
		toks -= avc_msg_cost;
		spin_unlock_irqrestore(&ratelimit_lock, flags);
		if (lost)
			printk(KERN_WARNING "AVC: %d messages suppressed.\n", lost);
		return 1;
	}
	missed++;
	spin_unlock_irqrestore(&ratelimit_lock, flags);
	return 0;
}


#ifdef CONFIG_SECURITY_SELINUX_DEVELOP

static inline int check_avc_ratelimit(void)  
{
	if (selinux_enforcing)
		return avc_ratelimit();
	else {
		/* If permissive, then never suppress messages. */
		return 1;
	}
}

#else

static inline int check_avc_ratelimit(void)  
{
	return avc_ratelimit();
}

#endif



/*
 * Audit the granting or denial of permissions.
 */
void avc_audit(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t audited,	/* IN */
	struct avc_entry *ae,		/* IN */
	__u32 denied,			/* IN */
	avc_audit_data_t *a)		/* IN */
{
	char *p;

	if (a && a->type == AVC_AUDIT_DATA_DONTAUDIT)
		return;

	if (!check_avc_ratelimit())
		return;

	printk("\navc:  %s ", denied ? "denied" : "granted");
	avc_dump_av(tclass,audited);
	printk(" for ");
	if (current && current->pid) {
		printk(" pid=%d", current->pid);
		if ((!a || !a->dontcalldcache) && current->mm) {
			struct vm_area_struct *vma = current->mm->mmap;

			while (vma) {
				if ((vma->vm_flags & VM_EXECUTABLE) && 
				    vma->vm_file) {
					p = d_path(vma->vm_file->f_dentry, 
						   vma->vm_file->f_vfsmnt, 
						   avc_audit_buffer,
						   PAGE_SIZE);
					printk(" exe=%s", p);
					break;
				}
				vma = vma->vm_next;
			}
		} else {
			printk(" comm=%s", current->comm);
		}
	}
	if (a) {
		switch (a->type) {
		case AVC_AUDIT_DATA_IPC:
			printk(" key=%d", a->u.ipc_id);
			break;
		case AVC_AUDIT_DATA_CAP:
			printk(" capability=%d", a->u.cap);
			break;
		case AVC_AUDIT_DATA_FS:
			if (a->u.fs.dentry) {
				struct inode *inode = a->u.fs.dentry->d_inode;
				
				if (!a->dontcalldcache) {
					p = avc_d_path(a->u.fs.dentry, 
						       avc_audit_buffer,
						       PAGE_SIZE);
					if (p)
						printk(" path=%s", p);
				}

				if (inode) {
					printk(" dev=%s ino=%ld", 
					       kdevname(to_kdev_t(inode->i_dev)),
					       inode->i_ino);
				}
			}

			if (a->u.fs.inode) {
				struct inode *inode = a->u.fs.inode;
				struct dentry *dentry;

				if (!a->dontcalldcache) {
					dentry = d_find_alias(inode);
					if (dentry) {
						p = avc_d_path(dentry, 
							       avc_audit_buffer,
							       PAGE_SIZE);
						if (p)
							printk(" path=%s", p);
						dput(dentry);
					}
				}

				printk(" dev=%s ino=%ld", 
				       kdevname(to_kdev_t(inode->i_dev)),inode->i_ino);
			}
			break;
		case AVC_AUDIT_DATA_NET:
			if (a->u.net.sk) {
				struct sock *sk = a->u.net.sk;

				switch (sk->family) {
				case AF_INET:
					print_ipv4_addr(sk->rcv_saddr,
							sk->sport,
							"laddr", "lport");
					print_ipv4_addr(sk->daddr,
							sk->dport,
							"faddr", "fport");
					break;
				case AF_UNIX: 
					if (sk->protinfo.af_unix.dentry) {
						p = d_path(sk->protinfo.af_unix.dentry, 
							   sk->protinfo.af_unix.mnt,
							   avc_audit_buffer,
							   PAGE_SIZE);
						printk(" path=%s", p);
					} else if (sk->protinfo.af_unix.addr) {
						p = avc_audit_buffer;
						memcpy(p,
						       sk->protinfo.af_unix.addr->name->sun_path,
						       sk->protinfo.af_unix.addr->len-sizeof(short));
						if (*p == 0) {
							*p = '@';
							p += sk->protinfo.af_unix.addr->len-sizeof(short);
							*p = 0;
						}
						printk(" path=%s", 
						       avc_audit_buffer);
					}
					break;
				}
			}
			if (a->u.net.daddr) {
				printk(" daddr=%d.%d.%d.%d", 
				       NIPQUAD(a->u.net.daddr));
				if (a->u.net.port)
					printk(" dest=%d", a->u.net.port);
			} else if (a->u.net.port)
				printk(" port=%d", a->u.net.port);
			if (a->u.net.skb) {
				struct sk_buff *skb = a->u.net.skb;

				if ((skb->protocol == __constant_htons(ETH_P_IP)) &&
				     skb->nh.iph) {
					__u16 source = 0, dest = 0;
					__u8  protocol = skb->nh.iph->protocol;


					if (protocol == IPPROTO_TCP && 
					    skb->h.th) {
						source = skb->h.th->source;
						dest = skb->h.th->dest;
					} 
					if (protocol == IPPROTO_UDP && 
					    skb->h.uh) {
						source = skb->h.uh->source;
						dest = skb->h.uh->dest;
					}

					print_ipv4_addr(skb->nh.iph->saddr,
							source,
							"saddr", "source");
					print_ipv4_addr(skb->nh.iph->daddr,
							dest,
							"daddr", "dest");
				}
			}
			if (a->u.net.netif)
				printk(" netif=%s", a->u.net.netif);
			break;
		}
	}
	printk(" ");
	avc_dump_query(ssid, tsid, tclass);
	printk("\n");
}


typedef struct avc_callback_node 
{
	int (*callback)(__u32 event, 
			security_id_t ssid,
			security_id_t tsid,
			security_class_t tclass,
			access_vector_t perms,
			access_vector_t *out_retained);
	__u32 events;
	security_id_t ssid;
	security_id_t tsid;
	security_class_t tclass;
	access_vector_t perms;
	struct avc_callback_node *next;
} avc_callback_node_t;

static avc_callback_node_t *avc_callbacks = NULL;


/*
 * Register a callback for events in the set `events'
 * related to the SID pair (`ssid', `tsid') and
 * and the permissions `perms', interpreting
 * `perms' based on `tclass'.
 */
int avc_add_callback(
	int (*callback)(__u32 event, 
			security_id_t ssid,
			security_id_t tsid,
			security_class_t tclass,
			access_vector_t perms,
			access_vector_t *out_retained),
	__u32 events,
	security_id_t ssid,
	security_id_t tsid,
	security_class_t tclass,
	access_vector_t perms)
{
    avc_callback_node_t *c;

    c = (avc_callback_node_t *) kmalloc(sizeof(avc_callback_node_t),
					GFP_ATOMIC);
    if (!c)
	return -ENOMEM;

    c->callback = callback;
    c->events = events;
    c->ssid = ssid;
    c->tsid = tsid;
    c->perms = perms;
    c->next = avc_callbacks;
    avc_callbacks = c;
    return 0;
}


#define AVC_SIDCMP(x,y) \
((x) == (y) || (x) == SECSID_WILD || (y) == SECSID_WILD)


/*
 * Update the cache entry `node' based on the 
 * event `event' and permissions `perms'.
 */
static inline void avc_update_node(
	__u32 event,
	avc_node_t *node,
	access_vector_t perms)
{
	switch (event) { 
	case AVC_CALLBACK_GRANT: 
		node->ae.allowed |= perms; 
		break; 
	case AVC_CALLBACK_TRY_REVOKE: 
	case AVC_CALLBACK_REVOKE: 
		node->ae.allowed &= ~perms; 
		break; 
	case AVC_CALLBACK_AUDITALLOW_ENABLE: 
		node->ae.auditallow |= perms; 
		break; 
	case AVC_CALLBACK_AUDITALLOW_DISABLE: 
		node->ae.auditallow &= ~perms; 
		break; 
	case AVC_CALLBACK_AUDITDENY_ENABLE: 
		node->ae.auditdeny |= perms; 
		break; 
	case AVC_CALLBACK_AUDITDENY_DISABLE: 
		node->ae.auditdeny &= ~perms; 
		break; 
	}
}


/*
 * Update any cache entries that match the 
 * SID pair (`ssid', `tsid') and class `tclass'
 * based on the event `event' and permissions
 * `perms'.
 */
static int avc_update_cache(
	__u32 event,	 		/* IN */
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms)		/* IN */
{
	avc_node_t     *node;	
	int i;
	unsigned long flags;

	spin_lock_irqsave(&avc_lock,flags);

	if (ssid == SECSID_WILD || tsid == SECSID_WILD) {
		/* apply to all matching nodes */
		for (i = 0; i < AVC_CACHE_SLOTS; i++) {
			for (node = avc_cache.slots[i]; node; 
			     node = node->next) {
				if (AVC_SIDCMP(ssid, node->ae.ssid) && 
				    AVC_SIDCMP(tsid, node->ae.tsid) &&
				    tclass == node->ae.tclass) {
					avc_update_node(event,node,perms);
				}
			}
		}
	} else {
		/* apply to one node */
		node = avc_search_node(ssid, tsid, tclass, 0);		
		if (node) {
			avc_update_node(event,node,perms);
		}
	}

	spin_unlock_irqrestore(&avc_lock,flags);

	return 0;
}

/*
 * Update the cache state and invoke any 
 * registered callbacks that match the 
 * SID pair (`ssid', `tsid') and class `tclass'
 * based on the event `event' and permissions
 * `perms'.  Increase the latest revocation 
 * notification sequence number if appropriate.
 */
static int avc_control(
	__u32 event,	 		/* IN */
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms,		/* IN */
	__u32 seqno,			/* IN */
	access_vector_t *out_retained)	/* OUT */
{
	avc_callback_node_t *c;
	access_vector_t tretained = 0, cretained = 0;
	int rc;
	unsigned long flags;

	/*
	 * try_revoke only removes permissions from the cache
	 * state if they are not retained by the object manager.
	 * Hence, try_revoke must wait until after the callbacks have
	 * been invoked to update the cache state.
	 */
	if (event != AVC_CALLBACK_TRY_REVOKE)
		avc_update_cache(event,ssid,tsid,tclass,perms);

	for (c = avc_callbacks; c; c = c->next)
	{
		if ((c->events & event) &&
		    AVC_SIDCMP(c->ssid, ssid) && 
		    AVC_SIDCMP(c->tsid, tsid) &&
		    c->tclass == tclass &&
		    (c->perms & perms)) {
			cretained = 0;
			rc = c->callback(event, ssid, tsid, tclass,
					 (c->perms & perms),	
					 &cretained);
			if (rc)
				return rc;
			tretained |= cretained;			
		}
	}

	if (event == AVC_CALLBACK_TRY_REVOKE) {
		/* revoke any unretained permissions */
		perms &= ~tretained;
		avc_update_cache(event,ssid,tsid,tclass,perms);
		*out_retained = tretained;
	}

	spin_lock_irqsave(&avc_lock,flags);
	if (seqno > avc_cache.latest_notif)
		avc_cache.latest_notif = seqno;
	spin_unlock_irqrestore(&avc_lock,flags);
	
	return 0;
}


/* Grant previously denied permissions */
int avc_ss_grant(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms,		/* IN */
	__u32 seqno)			/* IN */
{
	return avc_control(AVC_CALLBACK_GRANT,
			   ssid, tsid, tclass, perms, seqno, 0);
}


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
	access_vector_t *out_retained)	/* OUT */
{
	return avc_control(AVC_CALLBACK_TRY_REVOKE,
			   ssid, tsid, tclass, perms, seqno, out_retained);
}


/*
 * Revoke previously granted permissions, even if
 * they are retained as migrated permissions.
 */
int avc_ss_revoke(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms,		/* IN */
	__u32 seqno)			/* IN */
{
	return avc_control(AVC_CALLBACK_REVOKE,
			   ssid, tsid, tclass, perms, seqno, 0);
}


/* 
 * Flush the cache and revalidate all migrated permissions.
 */
int avc_ss_reset(__u32 seqno)
{
	avc_callback_node_t *c;
	int rc;
	avc_node_t     *node, *tmp;
	int             i;
	unsigned long flags;

	avc_hash_eval("reset");

	spin_lock_irqsave(&avc_lock,flags);

	for (i = 0; i < AVC_CACHE_SLOTS; i++) {
		node = avc_cache.slots[i];
		while (node) {
			tmp = node;
			node = node->next;
			tmp->ae.ssid = tmp->ae.tsid = SECSID_NULL;
			tmp->ae.tclass = SECCLASS_NULL;
			tmp->ae.allowed = tmp->ae.decided = 0;
			tmp->ae.auditallow = tmp->ae.auditdeny = 0;
			tmp->ae.used = 0;
			tmp->next = avc_node_freelist;
			avc_node_freelist = tmp;
			avc_cache.activeNodes--;
		}
		avc_cache.slots[i] = 0;
	}
	avc_cache.lru_hint = 0;

	spin_unlock_irqrestore(&avc_lock,flags);

	for (i = 0; i < AVC_NSTATS; i++)
		avc_cache_stats[i] = 0;

	for (c = avc_callbacks; c; c = c->next) {
		if (c->events & AVC_CALLBACK_RESET) {
			rc = c->callback(AVC_CALLBACK_RESET, 
					 0, 0, 0, 0, 0);
			if (rc)
				return rc;
		}
	}

	spin_lock_irqsave(&avc_lock,flags);
	if (seqno > avc_cache.latest_notif)
		avc_cache.latest_notif = seqno;
	spin_unlock_irqrestore(&avc_lock,flags);	

	return 0;
}


/* Enable or disable auditing of granted permissions */
int avc_ss_set_auditallow(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms,		/* IN */
	__u32 seqno,			/* IN */
	__u32 enable)
{
	if (enable)
		return avc_control(AVC_CALLBACK_AUDITALLOW_ENABLE,
				   ssid, tsid, tclass, perms, seqno, 0);
	else
		return avc_control(AVC_CALLBACK_AUDITALLOW_DISABLE,
				   ssid, tsid, tclass, perms, seqno, 0);
}


/* Enable or disable auditing of denied permissions */
int avc_ss_set_auditdeny(
	security_id_t ssid,		/* IN */
	security_id_t tsid,		/* IN */
	security_class_t tclass,	/* IN */
	access_vector_t perms,		/* IN */
	__u32 seqno,			/* IN */
	__u32 enable)
{
	if (enable)
		return avc_control(AVC_CALLBACK_AUDITDENY_ENABLE,
				   ssid, tsid, tclass, perms, seqno, 0);
	else
		return avc_control(AVC_CALLBACK_AUDITDENY_DISABLE,
				   ssid, tsid, tclass, perms, seqno, 0);
}


/*
 * Toggle the AVC between being permissive and 
 * enforcing permissions.  
 */
#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
long sys_avc_toggle(void) 
{
	int error;

	error = task_has_system(current, SYSTEM__AVC_TOGGLE);
	if (error)
		return error;

	selinux_enforcing = !selinux_enforcing;
	if (selinux_enforcing) 
		avc_ss_reset(avc_cache.latest_notif);

	return !selinux_enforcing;
}

long sys_avc_enforcing(void) 
{
	return selinux_enforcing;
}
#else
long sys_avc_toggle(void) 
{
	return 0;
}

long sys_avc_enforcing(void) 
{
	return 1;
}
#endif


