/* -*- linux-c -*- */
/*
 * Peer mapping cache.
 *
 * This structure contains a list of peers within our security perimeter
 * with which we exchange labeled packets.  Each entry contains the address
 * of the peer, a remote policy serial number, and a list of local to remote
 * SID mappings.  If any SCMP message is received from a peer with a different
 * policy serial number, the entire map for that peer is invalidated.
 *
 * The selopt_cache_lock protects the list containing the peer entries, as well
 * as preventing peers from being destroyed while their refcounts are being
 * incremented and decremented.  The only exception to this is when the entry
 * is created as it is not yet linked into the list.
 *
 * Copyright (c) 2001-2002 James Morris <jmorris@intercode.com.au>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 */
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netfilter.h>
#include <linux/ip.h>
#include <linux/netlink.h>
#include <linux/flask/flask_types.h>
#include <linux/flask/flnetlink.h>
#include "selinux_plug.h"
#include <linux/flask/selopt.h>
#include "cache.h"
#include "queue.h"

#define SELOPT_CACHE_PEER_MAX	1024	/* Max. total peers. */
#define SELOPT_CACHE_MAP_MAX	2048	/* Max. maps per peer. */

static LIST_HEAD(selopt_cache_list);
static rwlock_t selopt_cache_lock = RW_LOCK_UNLOCKED;
volatile __u32 global_map_total;
volatile __u32 peer_total;

/*
 * Increment peer reference count.
 */
inline void selopt_cache_peer_get(struct selopt_cache_peer *peer)
{
	atomic_inc(&peer->use);
}
  
/*
 * Decrement peer reference count.
 */      
inline void selopt_cache_peer_put(struct selopt_cache_peer *peer)
{
	atomic_dec(&peer->use);
}

/*
 * Find a peer in the cache by address.
 */
static inline struct selopt_cache_peer *__selopt_cache_peer_find(__u32 addr)
{
	struct list_head *i;
	
	for (i = selopt_cache_list.next; i != &selopt_cache_list; i = i->next) {
		struct selopt_cache_peer *peer = (struct selopt_cache_peer *)i;
		
		if (addr == peer->addr)
			return peer;
	}
	return NULL;
}

/*
 * Add a peer entry to the cache, possibly returning an existing
 * entry via old instead if someone raced us here.
 */
static inline int __selopt_cache_peer_append(struct selopt_cache_peer *new,
                                             struct selopt_cache_peer **old)
{
	struct selopt_cache_peer *peer;
	
	peer = __selopt_cache_peer_find(new->addr);
	if (peer) {
		if (old)
			*old = peer;
		return -EEXIST;
	}
	
	if (peer_total >= SELOPT_CACHE_PEER_MAX) {
		if (net_ratelimit())
			printk(KERN_WARNING "SELinux: clamping cache "
			       "at %u entries\n", SELOPT_CACHE_PEER_MAX);
		return -ENOSPC;
	}
	
	list_add_tail(&new->list, &selopt_cache_list);
	peer_total++;
	
	return 0;
}

/*
 * Lookup the SID map for a peer, given the remote SID.
 */
static inline struct selopt_cache_map *
__selopt_cache_map_lookup(struct selopt_cache_peer *peer, security_id_t rsid)
{
	struct list_head *i;
	struct selopt_cache_map *map = peer->map;

	if (rsid == SECSID_NULL)
		return NULL;
		
	for (i = map->list.next; i != &map->list; i = i->next) {
		struct selopt_cache_map *entry = (struct selopt_cache_map *)i;
		
		if (entry->rsid == rsid)
			return entry;
	}
	
	return NULL;
}

/*
 * Return an equivalent local SID given a remote SID, for the specified peer.
 */
static inline security_id_t
__selopt_cache_map_sid(struct selopt_cache_peer *peer, security_id_t rsid)
{
	struct selopt_cache_map *map;

	map = __selopt_cache_map_lookup(peer, rsid);
	if (map == NULL)
		return SECSID_NULL;
		
	return map->lsid;
}

/*
 * Create a new map entry for a peer given a remote SID.
 * TBD: try and move the kmalloc out of the spinlock.
 */
static inline struct selopt_cache_map *
__selopt_cache_map_create(struct selopt_cache_peer *peer,
                          security_id_t rsid, int *err)
{
	struct selopt_cache_map *map;

	if (peer->map_total >= SELOPT_CACHE_MAP_MAX) {
		if (net_ratelimit())
			printk(KERN_WARNING "SELinux: clamping maps "
			       "at %u entries for peer %u.%u.%u.%u\n",
			       SELOPT_CACHE_MAP_MAX, NIPQUAD(peer->addr));
		*err = -ENOSPC;
		return NULL;
	}

	map = kmalloc(sizeof(struct selopt_cache_map), GFP_ATOMIC);
	if (map == NULL) {
		printk(KERN_WARNING "%s: out of memory\n", __FUNCTION__);
		*err = -ENOMEM;
		return NULL;
	}
	
	memset(map, 0, sizeof(struct selopt_cache_map));
	
	map->rsid = rsid;
	map->lsid = SECSID_NULL;
	
	list_add_tail(&map->list, &peer->map->list);
	
	peer->map_total++;
	global_map_total++;
	
	*err = 0;
	return map;
}

/*
 * Flush all map entries for a peer.
 */
static inline void __selopt_cache_map_flush(struct selopt_cache_peer *peer)
{
	struct selopt_cache_map *map = peer->map;
	
	while (map->list.next != &map->list) {
		struct list_head *i = map->list.next;
		
		list_del(i);
		kfree(i);
		peer->map_total--;
		global_map_total--;
	}
}

/*
 * Destroy a peer entry and it's mappings.
 */
static inline void __selopt_cache_peer_destroy(struct selopt_cache_peer *peer)
{
	write_lock_bh(&peer->map_lock);
	
	if (peer->map) {
		__selopt_cache_map_flush(peer);
		kfree(peer->map);
		peer->map = NULL;
	}
	
	write_unlock_bh(&peer->map_lock);
	
	list_del(&peer->list);
}

/*
 * Peer destructor, protected by refcount.
 */
static void selopt_cache_peer_destroy(struct selopt_cache_peer *peer)
{
	write_lock_bh(&selopt_cache_lock);
	
	if (atomic_dec_and_test(&peer->use)) {
		__selopt_cache_peer_destroy(peer);
		kfree(peer);
	}
	
	write_unlock_bh(&selopt_cache_lock);
}

/*
 * XXX: reschedule for refcounts
 */
void selopt_cache_flush(void)
{
	write_lock_bh(&selopt_cache_lock);
	
	while (selopt_cache_list.next != &selopt_cache_list) {
		struct selopt_cache_peer *peer =
			(struct selopt_cache_peer *)selopt_cache_list.next;
		
		if (atomic_read(&peer->use) != 1)
			printk(KERN_ERR "BUG: Invalid refcount for peer!\n");
	
		__selopt_cache_peer_destroy(peer);
		kfree(peer);
	}	
	
	write_unlock_bh(&selopt_cache_lock);
}

/*
 * Fill an skb with an flnetlink peer message.  Called during a netlink dump.
 */
static int selopt_cache_fill_map(struct sk_buff *skb,
                                 struct selopt_cache_peer *peer,
                                 struct selopt_cache_map *map,
                                 int type, u32 pid, u32 seq)
{
	__u16 datalen;
	struct nlmsghdr *nlh;
	unsigned char *b = skb->tail;
	struct flmsg_map_res *msg;

	datalen = sizeof(struct flmsg_map_res) + sizeof(struct flmsg_attr_map);

	nlh = NLMSG_PUT(skb, pid, seq, type, datalen);
	
	if (pid)
		nlh->nlmsg_flags |= NLM_F_MULTI;

	msg = NLMSG_DATA(nlh);
	
	msg->base.count = 1;
	msg->base.peer = peer->addr;
	msg->base.serial = peer->addr;
	msg->map[0].lsid = map->lsid;
	msg->map[0].rsid = map->rsid;
	
	nlh->nlmsg_len = skb->tail - b;
	return skb->len;

nlmsg_failure:
	skb_trim(skb, b - skb->data);
	return -1;
}

/*
 * Netlink callback for dumping the entire cache to userspace.  Callback args[0]
 * is the peer index and args[1] is the map index.
 */
int selopt_cache_dump(struct sk_buff *skb, struct netlink_callback *cb)
{
	int peer_idx, map_idx;
	int s_peer_idx = cb->args[0];
	int s_map_idx = cb->args[1];
	struct list_head *i, *j;

	read_lock_bh(&selopt_cache_lock);

	for (i = selopt_cache_list.next, peer_idx = map_idx = 0;
	     i != &selopt_cache_list; i = i->next, peer_idx++) {
	     	struct selopt_cache_map *map;
		struct selopt_cache_peer *peer = (struct selopt_cache_peer *)i;
		
		if (peer_idx < s_peer_idx)
			continue;
		
		/*
		 * Found the current peer, start dumping it's map.
		 */
		map = peer->map;
		
		read_lock_bh(&peer->map_lock);
		
		for (j = map->list.next, map_idx = 0;
		     j != &map->list; j = j->next, map_idx++) {
			struct selopt_cache_map *entry = (struct selopt_cache_map *)j;
			
			if (map_idx < s_map_idx)
				continue;
				
			if (selopt_cache_fill_map(skb, peer, entry, FLMSG_CACHE_GET,
		    	    NETLINK_CB(cb->skb).pid, cb->nlh->nlmsg_seq) <= 0) {
				read_unlock_bh(&peer->map_lock);
				goto escape;
			}
		}
		
		read_unlock_bh(&peer->map_lock);
	}

escape:
	read_unlock_bh(&selopt_cache_lock);
	
	cb->args[0] = peer_idx;
	cb->args[1] = map_idx;
	
	return skb->len;
}

/*
 * Find or create a peer entry in the cache.
 *
 * Returns the specified peer entry with refcount incremented, sets rc to
 * SELOPT_CACHE_NEW if the entry was created by this request.
 *
 * Caller must decrement refcount after it is finished using the entry
 * returned from here.
 */
struct selopt_cache_peer *selopt_cache_peer_lookup(__u32 addr, int *err)
{
	int rc;
	struct selopt_cache_peer *new, *old;
	
	read_lock_bh(&selopt_cache_lock);
	
	old = __selopt_cache_peer_find(addr);
	if (old)
		selopt_cache_peer_get(old);
		
	read_unlock_bh(&selopt_cache_lock);
	
	if (old)
		return old;
	
	/*
	 * Slow path, create new entry and add it to the cache.
	 */
	new = kmalloc(sizeof(struct selopt_cache_peer), SAFE_ALLOC);
	if (new == NULL) {
		*err = -ENOMEM;
		return NULL;
	}
	
	memset(new, 0, sizeof(struct selopt_cache_peer));
	
	new->map = kmalloc(sizeof(struct flmsg_attr_map), SAFE_ALLOC);
	if (new->map == NULL) {
		kfree(new);
		*err = -ENOMEM;
		return NULL;
	}
	
	memset(new->map, 0, sizeof(struct flmsg_attr_map));
	INIT_LIST_HEAD(&new->map->list);
	
	INIT_LIST_HEAD(&new->list);
	atomic_set(&new->use, 1);
	new->map_lock = RW_LOCK_UNLOCKED;
	new->addr = addr;
	
	write_lock_bh(&selopt_cache_lock);
	
	rc = __selopt_cache_peer_append(new, &old);
	
	if (rc >= 0) {
		*err = SELOPT_CACHE_NEW;
		selopt_cache_peer_get(new);
		write_unlock_bh(&selopt_cache_lock);
		return new;
	}

	/*
	 * If someone raced us to this, return their entry and destroy ours.
	 */
	if (rc == -EEXIST) {
		selopt_cache_peer_get(old);
		write_unlock_bh(&selopt_cache_lock);
		selopt_cache_peer_destroy(new);
		return old;
	}
	
	write_unlock_bh(&selopt_cache_lock);
	selopt_cache_peer_destroy(new);
	*err = rc;
	return NULL;
}

/*
 * Attempt to map a SID from the cache.  If it's not there, the caller
 * can schedule a remote lookup via SCMP.
 */
static int selopt_cache_map_sid(struct selopt_cache_peer *peer,
                                security_id_t rsid)
{
	security_id_t lsid;
	
	read_lock_bh(&peer->map_lock);
	lsid = __selopt_cache_map_sid(peer, rsid);
	read_unlock_bh(&peer->map_lock);
	
	return lsid;
}

/*
 * Map SIDs from cache, caller can determine which SIDs need SCMP lookups.
 */
void selopt_cache_map_sids(struct selopt_cache_peer *peer,
                          struct sk_buff *skb, int status)
{
	struct skb_security_struct *ssec = skb->lsm_security;

	if (status != SELOPT_CACHE_NEW) {
	
		if (selopt_get(ssec, SELOPT_SSID)) {
			security_id_t lsid;
			
			lsid = selopt_cache_map_sid(peer, ssec->ssid);
			if (lsid != SECSID_NULL) {
				selopt_map(ssec, SELOPT_SSID);
				ssec->ssid = lsid;
			}
		}
		
		if (selopt_get(ssec, SELOPT_MSID)) {
			security_id_t lsid;
			
			lsid = selopt_cache_map_sid(peer, ssec->msid);
			if (lsid != SECSID_NULL) {
				selopt_map(ssec, SELOPT_MSID);
				ssec->msid = lsid;
			}
		} else {
			/* msid defaults to ssid. */
			ssec->msid = ssec->ssid;
		}
		
		if (selopt_get(ssec, SELOPT_DSID)) {
			security_id_t lsid;
			
			lsid = selopt_cache_map_sid(peer, ssec->dsid);
			if (lsid != SECSID_NULL) {
				selopt_map(ssec, SELOPT_DSID);
				ssec->dsid = lsid;
			}
		}
	}
	
	return;
}

/*
 * Update and possibly create a new peer map entry.
 */
int selopt_cache_map_update(struct selopt_cache_peer *peer,
                            security_id_t rsid, security_id_t lsid)
{
	int err = 0;
	struct selopt_cache_map *map;

	write_lock_bh(&peer->map_lock);
	
	map = __selopt_cache_map_lookup(peer, rsid);
	
	if (map == NULL) {
		map = __selopt_cache_map_create(peer, rsid, &err);
		if (map == NULL) {
			write_unlock_bh(&peer->map_lock);
			return err;
		}
	}

	if (map->lsid != lsid)
		map->lsid = lsid;

	write_unlock_bh(&peer->map_lock);
	
	return err;
}

/*
 * Handle mapping response, add to cache then run packet queue evictor.
 */
int selopt_cache_map_response(struct flmsg_map_res *res)
{
	int i, err;
	struct selopt_cache_peer *peer;

	/*
	 * Get the cache entry for this peer.
	 */
	peer = selopt_cache_peer_lookup(res->base.peer, &err);
	if (peer == NULL)
		return err;

	/*
	 * Update the mappings from the map response.
	 */
	for (i = 0; i < res->base.count; i++) {
		int rc;
		struct flmsg_attr_map map = res->map[i];
		
		rc = selopt_cache_map_update(peer, map.rsid, map.lsid);
		if (rc) {
			selopt_cache_peer_put(peer);
			return rc;
		}
	}

	selopt_queue_evict(peer);
	selopt_cache_peer_put(peer);
	
	return 0;
}

/*
 * Build and broadcast a flnetlink map request message.
 */
int selopt_cache_request_mappings(struct sk_buff *skb)
{
	__u16 datalen, nsids;
	unsigned char *oldtail;
	struct nlmsghdr *nlh;
	struct sk_buff *nskb;
	struct iphdr *iph = skb->nh.iph;
	struct skb_security_struct *ssec = skb->lsm_security;
	struct flmsg_map_req *req;
int i;
	
	SDBG(ip_warn(iph, "ssid=%u msid=%u dsid=%u",
	             ssec->ssid, ssec->msid, ssec->dsid));
	
	nsids = selopt_maps_needed(ssec);
	datalen = sizeof(struct flmsg_map_req) + nsids * sizeof(security_id_t);
	
	nskb = alloc_skb(NLMSG_SPACE(datalen), SAFE_ALLOC);
	if (nskb == NULL) {
		printk(KERN_WARNING "%s: out of memory\n", __FUNCTION__);
		return -ENOMEM;
	}
	
	oldtail = nskb->tail;
	
	nlh = NLMSG_PUT(nskb, 0, 0, FLMSG_CACHE_MAP_REQ, datalen);
	req = NLMSG_DATA(nlh);
	
	memset(req, 0, datalen);
	
	req->base.count = nsids;
	req->base.peer = iph->saddr;
	req->base.serial = ssec->serial;
	
	i = 0;
	if (selopt_map_needed(ssec, SELOPT_SSID)) {
		req->sid[i] = ssec->ssid;
		i++;
	}
	
	if (selopt_map_needed(ssec, SELOPT_MSID)) {
		req->sid[i] = ssec->msid;
		i++;
	}
		
	if (selopt_map_needed(ssec, SELOPT_DSID)) {
		req->sid[i] = ssec->dsid;
	}
	
	nlh->nlmsg_len = nskb->tail - oldtail;
	netlink_broadcast(flnl, nskb, 0, FLN_G_CACHE, SAFE_ALLOC);
	        
	return 0;

nlmsg_failure:
	kfree_skb(nskb);
	return -ENOBUFS;
}

__init int selopt_cache_init(void)
{
	return 0;
}

void selopt_cache_exit(void)
{
	selopt_cache_flush();
	return;
}
