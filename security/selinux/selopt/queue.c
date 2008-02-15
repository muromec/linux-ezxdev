/* -*- linux-c -*- */
/*
 * Packet queue, for deferred SCMP lookups.
 *
 * Copyright (c) 2001-2002 James Morris <jmorris@intercode.com.au>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
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
#include <linux/ip.h>
#include <linux/netfilter.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>
#include <linux/sysctl.h>
#include <linux/netlink.h>
#include <linux/in.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/ip.h>
#include <linux/flask/flask_types.h>
#include "selinux_plug.h"
#include <linux/flask/selopt.h>
#include "queue.h"

#define SELOPT_QUEUE_MAX_ID		3000
#define SELOPT_QUEUE_MAX_NAME		"selopt_queue_max"
#define SELOPT_QUEUE_MAX_DEFAULT	1024

struct selopt_queue_entry {
	struct list_head list;
	struct sk_buff *skb;
	struct nf_info *info;
	void *data;
};

typedef int (*queue_matchfn)(struct selopt_queue_entry *entry,
                             unsigned long data);

static LIST_HEAD(selopt_queue_list);
static rwlock_t selopt_queue_lock = RW_LOCK_UNLOCKED;
static int selopt_queue_total = 0;
static int sysctl_selopt_queue_max = SELOPT_QUEUE_MAX_DEFAULT;

/*
 * Append an entry to the queue.
 */
static int inline __selopt_queue_append(struct selopt_queue_entry *entry)
{
	if (selopt_queue_total >= sysctl_selopt_queue_max) {
		if (net_ratelimit())
			printk(KERN_WARNING "SELinux: clamping queue "
			       "at %d entries\n", sysctl_selopt_queue_max);
		return -ENOSPC;
	}
	
	list_add_tail(&entry->list, &selopt_queue_list);
	selopt_queue_total++;

	return 0;
}

/*
 * Find and return a queued packet matched by matchfn, or the first entry if
 * matchfn is NULL.
 */
static struct selopt_queue_entry *
__selopt_queue_find(queue_matchfn matchfn, unsigned long data)
{
	struct list_head *i;
	
	for (i = selopt_queue_list.next; i != &selopt_queue_list; i = i->next) {
		struct selopt_queue_entry *entry = (struct selopt_queue_entry *)i;
		
		if (!matchfn || matchfn(entry, data))
			return entry;
	}
	return NULL;
}

/*
 * Dequeue entry with verdict.
 */
static void __selopt_queue_dequeue(struct selopt_queue_entry *entry, int verdict)
{
	list_del(&entry->list);
	selopt_queue_total--;
	nf_reinject(entry->skb, entry->info, verdict);
	kfree(entry);
}

/*
 * Flush entire queue with verdict.
 */
static inline void __selopt_queue_flush(int verdict)
{
	struct selopt_queue_entry *entry;
	
	while ((entry = __selopt_queue_find(NULL, 0)) != NULL)
		__selopt_queue_dequeue(entry, verdict);

	return;
}

/*
 * Add a new packet to the queue.
 */
static int selopt_queue_enqueue(struct sk_buff *skb,
                                struct nf_info *info, void *data)
{
	int rc;
	struct selopt_queue_entry *entry;
	
	entry = kmalloc(sizeof(struct selopt_queue_entry), SAFE_ALLOC);
	if (entry == NULL) {
		printk(KERN_WARNING "%s: out of memory\n", __FUNCTION__);
		return NF_DROP;
	}
	
	memset(entry, 0, sizeof(struct selopt_queue_entry));
	
	entry->skb = skb;
	entry->info = info;
	entry->data = data;

	write_lock_bh(&selopt_queue_lock);
	rc = __selopt_queue_append(entry);
	write_unlock_bh(&selopt_queue_lock);
	
	if (rc) {
		kfree(entry);
		return rc;
	}
	
	return 0;
}

/*
 * Netfilter queue handler.  Packets arrive here if a Netfilter hook has
 * returned NF_QUEUE, and must be returned to the stack via nf_reinject().
 */
static int selopt_queue_handler(struct sk_buff *skb,
                                struct nf_info *info, void *data)
{
	return selopt_queue_enqueue(skb, info, data);
}



static inline int
match_saddr(struct selopt_queue_entry *entry, unsigned long addr)
{
	return (entry->skb->nh.iph->saddr == addr);
}

static inline int
match_dev(struct selopt_queue_entry *entry, unsigned long ifindex)
{
	if (entry->info->indev)
		if (entry->info->indev->ifindex == ifindex)
			return 1;
			
	if (entry->info->outdev)
		if (entry->info->outdev->ifindex == ifindex)
			return 1;
			
	return 0;
}

/*
 * Attempt to evict queue entries for a peer, typically called after
 * a map response has been received.
 */
void selopt_queue_evict(struct selopt_cache_peer *peer)
{
	struct list_head *i;
	
	write_lock_bh(&selopt_queue_lock);

	for (i = selopt_queue_list.next; i != &selopt_queue_list; ) {
		struct selopt_queue_entry *entry = (struct selopt_queue_entry *)i;
		struct sk_buff *skb = entry->skb;
	
		i = i->next;
		
		if (peer->addr == skb->nh.iph->saddr) {
			struct skb_security_struct *ssec = skb->lsm_security;

			selopt_cache_map_sids(peer, skb, 0);

			if (selopt_mapped_all(ssec))
				__selopt_queue_dequeue(entry, NF_ACCEPT);
		}		
	}
	
	write_unlock_bh(&selopt_queue_lock);
	return;
}

/*
 * Flush all entries from the queue with given Netfilter verdict.
 */
void selopt_queue_flush(int verdict)
{
	write_lock_bh(&selopt_queue_lock);
	__selopt_queue_flush(verdict);
	write_unlock_bh(&selopt_queue_lock);
	return;
}

/*
 * Flush entries associated with the device.
 */
static void selopt_queue_dev_flush(struct net_device *dev, int verdict)
{
	struct selopt_queue_entry *entry;

	write_lock_bh(&selopt_queue_lock);
	
	while ((entry = __selopt_queue_find(match_dev, dev->ifindex)) != NULL)
		__selopt_queue_dequeue(entry, verdict);
	
	write_unlock_bh(&selopt_queue_lock);
}

/*
 * The device notifier is used to flush queue entries referencing a particular
 * device when it goes down.
 */
static int selopt_queue_receive_event(struct notifier_block *this,
                                      unsigned long event, void *ptr)
{
	struct net_device *dev = ptr;
	
	if (event == NETDEV_DOWN)
		selopt_queue_dev_flush(dev, NF_DROP);

	return NOTIFY_DONE;
}

static struct notifier_block selopt_queue_notifier = {
	selopt_queue_receive_event,
	NULL,
	0
};

/*
 * Fill an skb with a queue entry.  Called during a netlink dump.
 */
static int selopt_queue_fill_entry(struct sk_buff *skb,
                                   struct selopt_queue_entry *entry,
                                   int type, u32 pid, u32 seq)
{
	struct flmsg_queue_entry *r;
	struct nlmsghdr *nlh;
	unsigned char *b = skb->tail;
	struct sk_buff *eskb = entry->skb;
	struct iphdr *iph = eskb->nh.iph;
	struct skb_security_struct *ssec = eskb->lsm_security;
	
	        
	nlh = NLMSG_PUT(skb, pid, seq, type, sizeof(*r));
	
	if (pid)
		nlh->nlmsg_flags |= NLM_F_MULTI;

	r = NLMSG_DATA(nlh);
	memset(r, 0, sizeof(*r));
	
	r->base.serial = ssec->serial;
	r->base.peer   = iph->saddr;
	r->ttl         = 123;
	r->proto       = iph->protocol;
	
	/*
	 * Add port information, if possible.
	 */
	if (!(ntohs(iph->frag_off) & IP_OFFSET)) {
		void *protoh = (__u32 *)iph + iph->ihl;
		unsigned int datalen = eskb->len - iph->ihl * 4;
		
		switch (iph->protocol) {
		case IPPROTO_TCP: {
			struct tcphdr *tcph = protoh;
			
			if (datalen < sizeof (*tcph))
				break;
				
			r->sport = tcph->source;
			r->dport = tcph->dest;
			break;
		}
		
		case IPPROTO_UDP: {
			struct tcphdr *udph = protoh;
			
			if (datalen < sizeof (*udph))
				break;
				
			r->sport = udph->source;
			r->dport = udph->dest;
			break;
		}
		
		default:
			break;
			
		}
	}
	
	/*
	 * Add SID value and mapping status for each SID present.
	 */
	if (selopt_get(ssec, SELOPT_SSID)) {
		r->ssid_attr.sid = ssec->ssid;
		r->ssid_attr.mapped = selopt_mapped(ssec, SELOPT_SSID);
	}
	
	if (selopt_get(ssec, SELOPT_MSID)) {
		r->msid_attr.sid = ssec->msid;
		r->msid_attr.mapped = selopt_mapped(ssec, SELOPT_MSID);
	}
	
	if (selopt_get(ssec, SELOPT_DSID)) {
		r->dsid_attr.sid = ssec->dsid;
		r->dsid_attr.mapped = selopt_mapped(ssec, SELOPT_DSID);
	}
	
	nlh->nlmsg_len = skb->tail - b;
	return skb->len;

nlmsg_failure:
	skb_trim(skb, b - skb->data);
	return -1;
}

/*
 * Netlink callback for dumping the entire queue to userspace.
 */
int selopt_queue_dump(struct sk_buff *skb, struct netlink_callback *cb)
{
	int idx;
	int s_idx = cb->args[0];
	struct list_head *i;

	read_lock_bh(&selopt_queue_lock);
	
	for (i = selopt_queue_list.next, idx = 0;
	     i != &selopt_queue_list; i = i->next, idx++) {
		struct selopt_queue_entry *entry =
			(struct selopt_queue_entry *)i;
		
		if (idx < s_idx)
			continue;
			
		if (selopt_queue_fill_entry(skb, entry, FLMSG_QUEUE_GET,
		    NETLINK_CB(cb->skb).pid, cb->nlh->nlmsg_seq) <= 0)
			break;
	}
	
	read_unlock_bh(&selopt_queue_lock);
	
	cb->args[0] = idx;
	
	return skb->len;
}

/*
 * Sysctl stuff.
 */
static struct ctl_table_header *selopt_sysctl_header;

static ctl_table selopt_table[] = {
	{ SELOPT_QUEUE_MAX_ID, SELOPT_QUEUE_MAX_NAME, &sysctl_selopt_queue_max,
	  sizeof(sysctl_selopt_queue_max), 0644,  NULL, proc_dointvec },
 	{ 0 }
};

static ctl_table selopt_dir_table[] = {
	{NET_IPV4, "ipv4", NULL, 0, 0555, selopt_table, 0, 0, 0, 0, 0},
	{ 0 }
};

static ctl_table selopt_root_table[] = {
	{CTL_NET, "net", NULL, 0, 0555, selopt_dir_table, 0, 0, 0, 0, 0},
	{ 0 }
};


__init int selopt_queue_init(void)
{
	int rc;

	selopt_sysctl_header = register_sysctl_table(selopt_root_table, 0);

#ifdef CONFIG_SYSCTL
	/* This error condition is only valid if sysctl support is enabled. */
	if (selopt_sysctl_header == NULL) {
		printk(KERN_WARNING "SELinux: register_sysctl_table failed\n");
		return -ENOMEM;
	}
#endif

	register_netdevice_notifier(&selopt_queue_notifier);
	
	rc = nf_register_queue_handler(PF_INET, selopt_queue_handler, NULL);
	if (rc) {
		printk(KERN_WARNING "SELinux: selopt queue reg. failed\n");
		unregister_netdevice_notifier(&selopt_queue_notifier);
		unregister_sysctl_table(selopt_sysctl_header);
		return rc;
	}
	
	return 0;
}

void selopt_queue_exit(void)
{
	nf_unregister_queue_handler(PF_INET);
	unregister_netdevice_notifier(&selopt_queue_notifier);
	selopt_queue_flush(NF_DROP);
	unregister_sysctl_table(selopt_sysctl_header);
	return;
}
