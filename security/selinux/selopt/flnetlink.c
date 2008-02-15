/* -*- linux-c -*- */
/*
 * Flask Netlink User/Kernel Interface
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
#include <linux/socket.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <net/sock.h>
#include <linux/netfilter.h>
#include <linux/flask/flask_types.h>
#include <linux/flask/flnetlink.h>
#include "selinux_plug.h"
#include <linux/flask/selopt.h>
#include "perimtab.h"
#include "cache.h"
#include "queue.h"

static DECLARE_MUTEX(flnl_sem);
struct sock *flnl = NULL;

/*
 * Parse a (non-dump) flnetlink message from userspace.
 */
static inline int flnetlink_parse_msg(unsigned char *msg, unsigned char type,
                                      __u32 seq, unsigned int len)
{
	int rc = 0;

	switch (type) {
	case FLMSG_PERIM_ADD:
	{
		struct flmsg_perim_entry *flmsg;
		
		if (len != sizeof(struct flmsg_perim_entry)) {
			rc = -EINVAL;
			break;
		}
		
		flmsg = (struct flmsg_perim_entry *)msg;
		rc = selopt_perimtab_append(flmsg->entry.addr, flmsg->entry.mask);
		break;
	}
	
	case FLMSG_PERIM_GET:
		rc = -ENOSYS;
		break;

	case FLMSG_PERIM_DEL:
	{
		struct flmsg_perim_entry *flmsg;
		
		if (len != sizeof(struct flmsg_perim_entry)) {
			rc = -EINVAL;
			break;
		}
		
		flmsg = (struct flmsg_perim_entry *)msg;
		rc = selopt_perimtab_delete(flmsg->entry.addr, flmsg->entry.mask);
		break;
	}
	
	case FLMSG_PERIM_FLUSH:
		selopt_perimtab_flush();
		break;
		
	case FLMSG_CACHE_MAP_RES:
	{
		struct flmsg_map_res *flmsg = (struct flmsg_map_res *)msg;
		
		if (len != sizeof(struct flmsg_map_res) +
		    flmsg->base.count * sizeof(struct flmsg_attr_map)) {
			rc = -EINVAL;
			break;
		}

		rc = selopt_cache_map_response(flmsg);
		break;
	}
	
	case FLMSG_CACHE_FLUSH:
		selopt_cache_flush();
		break;
		

	case FLMSG_QUEUE_GET:
		rc = -ENOSYS;
		break;
		
	case FLMSG_QUEUE_FLUSH:
		selopt_queue_flush(NF_DROP);
		break;
		
	default:
		rc = -EINVAL;
		break;
	};
	
	return rc;
}

/*
 * Called when a netlink dump completes.
 */
static int flnetlink_done(struct netlink_callback *cb)
{
	return 0;
}

/*
 * Process one flnetlink message, brodcast any changes made.
 */
static inline int flnetlink_rcv_msg(struct sk_buff *skb,
                                    struct nlmsghdr *nlh, int *errp)
{
	int type, rc;
	
	if (!(nlh->nlmsg_flags&NLM_F_REQUEST))
		return 0;

	type = nlh->nlmsg_type;
	
	if (type < FLMSG_BASE)
		return 0;
	
	if (type >= FLMSG_MAX) {
		*errp = -EINVAL;
		return -1;
	}

	if (security_ops->netlink_recv(skb)) {
		*errp = -EPERM;
		return -1;
	}

	/*
	 * Netlink dump starter.
	 */
	if (nlh->nlmsg_flags&NLM_F_DUMP) {
		__u32 rlen;
		fln_dumpfn dumpfn = NULL;
		
		switch (type) {
		case FLMSG_PERIM_GET:
			dumpfn = selopt_perimtab_dump;
			break;
			
		case FLMSG_CACHE_GET:
			dumpfn = selopt_cache_dump;
			break;
		
		case FLMSG_QUEUE_GET:
			dumpfn = selopt_queue_dump;
			break;
			
		default:
			*errp = -EINVAL;
			return -1;
		}
		
		*errp = netlink_dump_start(flnl, skb, nlh,
		                           dumpfn, flnetlink_done);
		if (*errp != 0)
			return -1;
		
		rlen = NLMSG_ALIGN(nlh->nlmsg_len);
		
		if (rlen > skb->len)
			rlen = skb->len;
			
		skb_pull(skb, rlen);
		return -1;
	}

	/*
	 * Non-dump requests.
	 */
	rc = flnetlink_parse_msg(NLMSG_DATA(nlh), type,
	                         nlh->nlmsg_seq, skb->len - NLMSG_LENGTH(0));
	if (rc) {
		*errp = rc;
		return -1;
	}

	switch (type) {
	case FLMSG_PERIM_ADD:
	case FLMSG_PERIM_DEL:
	case FLMSG_PERIM_FLUSH:
		atomic_inc(&skb->users);
		netlink_broadcast(flnl, skb, 0, FLN_G_PERIM, GFP_KERNEL);
		break;
	
	case FLMSG_CACHE_FLUSH:
		atomic_inc(&skb->users);
		netlink_broadcast(flnl, skb, 0, FLN_G_CACHE, GFP_KERNEL);
		break;
	
	case FLMSG_QUEUE_FLUSH:
		atomic_inc(&skb->users);
		netlink_broadcast(flnl, skb, 0, FLN_G_QUEUE, GFP_KERNEL);
		break;
		
	default:
		break;
	}
	
	return 0;
}

/*
 * Process one skb from userspace, possibly containing multiple
 * flnetlink messages.
 */
static inline int flnetlink_rcv_skb(struct sk_buff *skb)
{
	int err;
	struct nlmsghdr *nlh;
	
	while (skb->len >= NLMSG_SPACE(0)) {
		u32 rlen;
		
		nlh = (struct nlmsghdr *)skb->data;
		if (nlh->nlmsg_len < sizeof(*nlh) || skb->len < nlh->nlmsg_len)
			return 0;
		
		rlen = NLMSG_ALIGN(nlh->nlmsg_len);
		if (rlen > skb->len)
			rlen = skb->len;
			
		if (flnetlink_rcv_msg(skb, nlh, &err)) {
			if (err == 0)
				return -1;
			netlink_ack(skb, nlh, err);
		
		} else if (nlh->nlmsg_flags&NLM_F_ACK)
			netlink_ack(skb, nlh, 0);
		
		skb_pull(skb, rlen);
	}
	
	return 0;
}

/*
 * Socket receive routine.
 */
static void flnetlink_rcv(struct sock *sk, int len)
{
	do {
		struct sk_buff *skb;
		
		if (down_trylock(&flnl_sem))
			return;

		while ((skb = skb_dequeue(&sk->receive_queue)) != NULL) {
			if (flnetlink_rcv_skb(skb)) {
				if(skb->len)
					skb_queue_head(&sk->receive_queue, skb);
				else
					kfree_skb(skb);
				break;
			}
			kfree_skb(skb);
		}
		up(&flnl_sem);
		
	} while (flnl && flnl->receive_queue.qlen);

	return;
}

__init int flnetlink_init(void)
{
	flnl = netlink_kernel_create(NETLINK_FLASK, flnetlink_rcv);
	if (flnl == NULL) {
		printk(KERN_WARNING "SELinux: flnetlink init failed\n");
		return -EADDRNOTAVAIL; 
	}
	
	return 0;
}

void flnetlink_exit(void)
{
	sock_release(flnl->socket);
	down(&flnl_sem);
	up(&flnl_sem);
}
