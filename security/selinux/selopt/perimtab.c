/* -*- linux-c -*- */
/*
 * Perimeter Table
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
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <linux/flask/flask_types.h>
#include <linux/flask/flnetlink.h>

#include "perimtab.h"

#define SELOPT_PERIMTAB_MAX 1024

struct selopt_perimtab_entry {
	struct list_head list;
	__u32 addr;
	__u32 mask;
};

static LIST_HEAD(selopt_perimtab_list);
static rwlock_t selopt_perimtab_lock = RW_LOCK_UNLOCKED;
static volatile __u32 selopt_perimtab_total;

static inline struct selopt_perimtab_entry *__selopt_perimtab_find(__u32 addr,
                                                                   __u32 mask)
{
	struct list_head *i;
	
	for (i = selopt_perimtab_list.next;
	     i != &selopt_perimtab_list; i = i->next) {
		struct selopt_perimtab_entry *entry =
			(struct selopt_perimtab_entry *)i;
		
		if (addr == entry->addr && mask == entry->mask)
			return entry;
	}
	
	return NULL;
}

static inline int __selopt_perimtab_append(struct selopt_perimtab_entry *entry)
{
	if (selopt_perimtab_total >= SELOPT_PERIMTAB_MAX) {
		if (net_ratelimit())
			printk(KERN_WARNING "SELinux: clamping perimeter table "
			       "at %u entries\n", SELOPT_PERIMTAB_MAX);
		return -ENOSPC;
	}
	
	if (__selopt_perimtab_find(entry->addr, entry->mask))
		return -EEXIST;

	list_add_tail(&entry->list, &selopt_perimtab_list);
	selopt_perimtab_total++;
	
	return 0;
}

static inline int __selopt_perimtab_match(__u32 addr)
{
	struct list_head *i;

	for (i = selopt_perimtab_list.next;
	     i != &selopt_perimtab_list; i = i->next) {
		struct selopt_perimtab_entry *entry =
			(struct selopt_perimtab_entry *)i;
	
		if (entry->addr == (addr & entry->mask))
			return 1;
	}
	return 0;
}

static inline int __selopt_perimtab_delete(__u32 addr, __u32 mask)
{
	struct selopt_perimtab_entry *entry;
	
	entry = __selopt_perimtab_find(addr, mask);
	if (entry == NULL)
		return -ENOENT;
	
	list_del(&entry->list);
	kfree(entry);
	selopt_perimtab_total--;
	
	return 0;
}

static inline void __selopt_perimtab_flush(void)
{
	while (selopt_perimtab_list.next != &selopt_perimtab_list) {
		struct list_head *i = selopt_perimtab_list.next;
		
		list_del(i);
		kfree(i);
		selopt_perimtab_total--;
	}
}

/*
 * Notify userspace that the perimeter table is being flushed.
 */
static void selopt_perimtab_flush_notify(void)
{
	unsigned char *oldtail;
	struct nlmsghdr *nlh;
	struct sk_buff *skb;
	
	skb = alloc_skb(NLMSG_SPACE(0), SAFE_ALLOC);
	if (skb == NULL) {
		printk(KERN_WARNING "%s: out of memory\n", __FUNCTION__);
		return;
	}
	
	oldtail = skb->tail;
	nlh = NLMSG_PUT(skb, 0, 0, FLMSG_PERIM_FLUSH, NLMSG_SPACE(0));
	nlh->nlmsg_len = skb->tail - oldtail;
	netlink_broadcast(flnl, skb, 0, FLN_G_PERIM, GFP_KERNEL);

	return;

nlmsg_failure:
	kfree_skb(skb);
	printk(KERN_WARNING "%s: netlink message failure\n", __FUNCTION__);
}


/*
 * Flush all entries from the table.
 */
void selopt_perimtab_flush(void)
{
	selopt_perimtab_flush_notify();
	
	write_lock_bh(&selopt_perimtab_lock);
	__selopt_perimtab_flush();
	write_unlock_bh(&selopt_perimtab_lock);
	
	return;
}

/*
 * Add a new entry to the table.
 */
int selopt_perimtab_append(__u32 addr, __u32 mask)
{
	int rc;
	struct selopt_perimtab_entry *entry;

	entry = kmalloc(sizeof(struct selopt_perimtab_entry), GFP_KERNEL);
	if (entry == NULL)
		return -ENOMEM;
	
	entry->addr = addr;
	entry->mask = mask;

	write_lock_bh(&selopt_perimtab_lock);
	rc = __selopt_perimtab_append(entry);
	write_unlock_bh(&selopt_perimtab_lock);
	
	if (rc)
		kfree(entry);

	return rc;
}

/*
 * Match an address in the table.
 */
int selopt_perimtab_match(__u32 addr)
{
	int rc;
	
	read_lock_bh(&selopt_perimtab_lock);
	rc = __selopt_perimtab_match(addr);
	read_unlock_bh(&selopt_perimtab_lock);
	return rc;
}

/*
 * Delete an entry from the table.
 */
int selopt_perimtab_delete(__u32 addr, __u32 mask)
{
	int rc;

	write_lock_bh(&selopt_perimtab_lock);
	rc = __selopt_perimtab_delete(addr, mask);
	write_unlock_bh(&selopt_perimtab_lock);
	return rc;
}


/*
 * Fill an skb with a perimtab entry.  Called during a netlink dump.
 */
static int selopt_perimtab_fill_entry(struct sk_buff *skb,
                                      struct selopt_perimtab_entry *entry,
                                      int type, u32 pid, u32 seq)
{
	struct flmsg_perim_entry *r;
	struct nlmsghdr *nlh;
	unsigned char *b = skb->tail;

	nlh = NLMSG_PUT(skb, pid, seq, type, sizeof(struct flmsg_perim_entry));
	
	if (pid)
		nlh->nlmsg_flags |= NLM_F_MULTI;

	r = NLMSG_DATA(nlh);
	
	r->entry.addr = entry->addr;
	r->entry.mask = entry->mask;
	
	nlh->nlmsg_len = skb->tail - b;
	return skb->len;

nlmsg_failure:
	skb_trim(skb, b - skb->data);
	return -1;
}

/*
 * Netlink callback for dumping the entire table to userspace.
 */
int selopt_perimtab_dump(struct sk_buff *skb, struct netlink_callback *cb)
{
	int idx;
	int s_idx = cb->args[0];
	struct list_head *i;

	read_lock_bh(&selopt_perimtab_lock);
	
	for (i = selopt_perimtab_list.next, idx = 0;
	     i != &selopt_perimtab_list; i = i->next, idx++) {
		struct selopt_perimtab_entry *entry =
			(struct selopt_perimtab_entry *)i;
		
		if (idx < s_idx)
			continue;
			
		if (selopt_perimtab_fill_entry(skb, entry, FLMSG_PERIM_GET,
		    NETLINK_CB(cb->skb).pid, cb->nlh->nlmsg_seq) <= 0)
			break;
	}
	
	read_unlock_bh(&selopt_perimtab_lock);
	
	cb->args[0] = idx;
	
	return skb->len;
}

__init int selopt_perimtab_init(void)
{
	return 0;
}

void selopt_perimtab_exit(void)
{
	selopt_perimtab_flush();
}
