/* -*- linux-c -*- */
/*
 * Network SID API
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
#include <linux/module.h>
#include <linux/socket.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/netfilter.h>
#include <linux/flask/nsid.h>
#include <linux/smp_lock.h>
#include <linux/brlock.h>

#if defined(CONFIG_SECURITY_SELINUX_NSID)

static spinlock_t nsid_ops_lock = SPIN_LOCK_UNLOCKED;
static struct nsid_operations *nsid_ops;

static unsigned int dummy_ip_label_output(unsigned int hooknum,
                                          struct sk_buff **pskb,
                                          const struct net_device *in,
                                          const struct net_device *out,
                                          int (*okfn)(struct sk_buff *))
{
	return NF_ACCEPT;
}

static unsigned int dummy_ip_map_input(unsigned int hooknum,
                                       struct sk_buff **pskb,
                                       const struct net_device *in,
                                       const struct net_device *out,
                                       int (*okfn)(struct sk_buff *))
{
	return NF_ACCEPT;
}


static int dummy_ip_decode_options(struct sk_buff *skb,
                                   const char *optptr, unsigned char **pp_ptr)
{
	if (!skb && !capable(CAP_NET_RAW)) {
		(const unsigned char *)*pp_ptr = optptr;
		return -EPERM;
	}
	
	return 0;
}

static int dummy_ip_defragment(struct sk_buff *skb)
{
	return 0;
}

static void dummy_sock_sendmsg(struct sock *sk)
{
	return;
}

static struct nsid_operations dummy_ops =
{
	ip_label_output:	dummy_ip_label_output,
	ip_map_input:		dummy_ip_map_input,
	ip_decode_options:	dummy_ip_decode_options,
	ip_defragment:		dummy_ip_defragment,
	sock_sendmsg:		dummy_sock_sendmsg,
};

unsigned int nsid_ip_label_output(unsigned int hooknum,
                                  struct sk_buff **pskb,
                                  const struct net_device *in,
                                  const struct net_device *out,
                                  int (*okfn)(struct sk_buff *))
{
	return nsid_ops->ip_label_output(hooknum, pskb, in, out, okfn);
}

unsigned int nsid_ip_map_input(unsigned int hooknum,
                               struct sk_buff **pskb,
                               const struct net_device *in,
                               const struct net_device *out,
                               int (*okfn)(struct sk_buff *))
{
	return nsid_ops->ip_map_input(hooknum, pskb, in, out, okfn);
}


int nsid_ip_decode_options(struct sk_buff *skb,
                           const char *optptr, unsigned char **pp_ptr)
{
	return nsid_ops->ip_decode_options(skb, optptr, pp_ptr);
}

int nsid_ip_defragment(struct sk_buff *skb)
{
	return nsid_ops->ip_defragment(skb);
}

void nsid_sock_sendmsg(struct sock *sk)
{
	nsid_ops->sock_sendmsg(sk);
}

int nsid_register_ops(struct nsid_operations *ops)
{
	spin_lock_bh(&nsid_ops_lock);
	
	if (nsid_ops && nsid_ops != &dummy_ops) {
		spin_unlock_bh(&nsid_ops_lock);
		printk(KERN_WARNING "nsid ops already registered\n");
		return -EBUSY;
	}
	
	nsid_ops = ops;
	spin_unlock_bh(&nsid_ops_lock);
	
	return 0;
}

int nsid_unregister_ops(struct nsid_operations *ops)
{
	spin_lock_bh(&nsid_ops_lock);
	
	if (nsid_ops != ops) {
		spin_unlock_bh(&nsid_ops_lock);
		printk(KERN_WARNING "failed to unregister other nsid ops\n");
		return -EINVAL;
	}
	
	/* Synchronize with net_rx_action() and Netfilter hook iteration */
	br_write_lock_bh(BR_NETPROTO_LOCK);
	nsid_ops = &dummy_ops;
	br_write_unlock_bh(BR_NETPROTO_LOCK);
	
	spin_unlock_bh(&nsid_ops_lock);
	
	return 0;
}

__init int nsid_init(void)
{
	int rc;
	
	rc = nsid_register_ops(&dummy_ops);
	if (rc)
		return rc;
		
	printk(KERN_INFO "SELinux: NSID API initialized\n");
	
	return 0;
}

EXPORT_SYMBOL(nsid_ip_label_output);
EXPORT_SYMBOL(nsid_ip_map_input);
EXPORT_SYMBOL(nsid_ip_decode_options);
EXPORT_SYMBOL(nsid_ip_defragment);
EXPORT_SYMBOL(nsid_sock_sendmsg);
EXPORT_SYMBOL(nsid_register_ops);
EXPORT_SYMBOL(nsid_unregister_ops);

#endif	/* CONFIG_SECURITY_SELINUX_NSID */
