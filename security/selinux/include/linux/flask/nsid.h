/* -*- linux-c -*- */
/*
 * Network SID API.
 *
 * Copyright (c) 2001-2002 James Morris <jmorris@intercode.com.au>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 */
#ifndef _LINUX_FLASK_NSID_H_
#define _LINUX_FLASK_NSID_H_

#if defined(CONFIG_SECURITY_SELINUX_NSID)

struct nsid_operations {
	unsigned int (*ip_label_output)(unsigned int hooknum,
	                                struct sk_buff **pskb,
	                                const struct net_device *in,
	                                const struct net_device *out,
	                                int (*okfn)(struct sk_buff *));
	                                
	unsigned int (*ip_map_input)(unsigned int hooknum,
	                             struct sk_buff **pskb,
	                             const struct net_device *in,
	                             const struct net_device *out,
	                             int (*okfn)(struct sk_buff *));

	int (*ip_decode_options)(struct sk_buff *skb,
	                         const char *optptr,
	                         unsigned char **pp_ptr);
	                         
	int (*ip_defragment)(struct sk_buff *skb);

	void (*sock_sendmsg)(struct sock *sk);
};

unsigned int nsid_ip_label_output(unsigned int hooknum,
                                  struct sk_buff **pskb,
                                  const struct net_device *in,
                                  const struct net_device *out,
                                  int (*okfn)(struct sk_buff *));

unsigned int nsid_ip_map_input(unsigned int hooknum,
                               struct sk_buff **pskb,
                               const struct net_device *in,
                               const struct net_device *out,
                               int (*okfn)(struct sk_buff *));

int nsid_ip_decode_options(struct sk_buff *skb,
                           const char *optptr, unsigned char **pp_ptr);

int nsid_ip_defragment(struct sk_buff *skb);

void nsid_sock_sendmsg(struct sock *sk);

int nsid_register_ops(struct nsid_operations *ops);
int nsid_unregister_ops(struct nsid_operations *ops);

int nsid_init(void) __init;

#else

#include <linux/netfilter.h>

static inline unsigned int nsid_ip_label_output(unsigned int hooknum,
                                                struct sk_buff **pskb,
                                                const struct net_device *in,
                                                const struct net_device *out,
                                                int (*okfn)(struct sk_buff *))
{ return NF_ACCEPT; }

static inline unsigned int nsid_ip_map_input(unsigned int hooknum,
                                             struct sk_buff **pskb,
                                             const struct net_device *in,
                                             const struct net_device *out,
                                             int (*okfn)(struct sk_buff *))
{ return NF_ACCEPT; }


static inline int nsid_ip_decode_options(struct sk_buff *skb,
                                         const char *optptr,
                                         unsigned char **pp_ptr)
{
	if (!skb && !capable(CAP_NET_RAW)) {
		(const unsigned char *)*pp_ptr = optptr;
		return -EPERM;
	}
	return 0;
}

static inline int nsid_ip_defragment(struct sk_buff *skb)
{ return 0; }

static inline void nsid_sock_sendmsg(struct sock *sk)
{ return; }

static inline __init int nsid_init(void)
{ return 0; }

#endif	/* defined CONFIG_SECURITY_SELINUX_NSID */
      
#endif /* _LINUX_FLASK_NSID_H_ */

