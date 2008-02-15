/* -*- linux-c -*- */
/*
 * SELinux Labeled IP Networking via CIPSO/FIPS-188 Options
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
#include <net/ip.h>
#include <net/tcp.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/tcp.h>
#include <linux/netfilter.h>
#include <linux/flask/nsid.h>
#include <linux/flask/flask_types.h>
#include <linux/flask/flnetlink.h>
#include "selinux_plug.h"
#include <linux/flask/selopt.h>
#include "perimtab.h"
#include "cache.h"
#include "queue.h"

/*
 * Map remote SIDS to local SIDS via the peer cache.  If we get
 * a cache miss or invalidation caused by policy serial number change,
 * the packet is queued during the SCMP lookup, then reinjected.
 */
static unsigned int selopt_map_sids(unsigned int hooknum,
                                    struct sk_buff **pskb,
                                    const struct net_device *in,
                                    const struct net_device *out,
                                    int (*okfn)(struct sk_buff *))
{
	int rc = 0;
	struct sk_buff *skb = *pskb;
	struct iphdr *iph = skb->nh.iph;
	struct skb_security_struct *ssec = skb->lsm_security;
	struct selopt_cache_peer *peer;

	SDBG(ip_warn(iph, "serial=%u ssid=%u msid=%u dsid=%u bypass=%u "
	             "opts=%x", ssec->serial, ssec->ssid, ssec->msid,
	             ssec->dsid, selopt_get(ssec, SELOPT_BYPASS), ssec->opts));

	/* Implicit mapping for local packets */
	if (inet_addr_type(iph->saddr) == RTN_LOCAL) {
		selopt_map_all(ssec);
		return NF_ACCEPT;
	}	

	/* If this packet is a SCMP bypass packet, make sure it's destined for 
	 * the SCMP port in this machine and label the packet accordingly.
	 */
	if (selopt_get(ssec, SELOPT_BYPASS)) {
		struct udphdr *udph = (void *)iph + iph->ihl * 4;
		if (iph->protocol != IPPROTO_UDP) {
			ip_warn(iph, "non-UDP bypass packet\n");
			return NF_DROP;	
		} 
		if (udph->dest != __constant_htons(SCMP_PORT)) {
			ip_warn(iph, "bypass packet destined for non-SCMP port %u\n", ntohs(udph->dest));
			return NF_DROP;	
		}
		ssec->ssid = ssec->msid = SECINITSID_SCMP_PACKET;
		return NF_ACCEPT;
	}

	peer = selopt_cache_peer_lookup(iph->saddr, &rc);
	if (peer == NULL)
		return NF_DROP;

	selopt_cache_map_sids(peer, skb, rc);
	selopt_cache_peer_put(peer);

	if (selopt_mapped_all(ssec)) {

		SDBG(ip_warn(iph, "mapped: serial=%u ssid=%u msid=%u dsid=%u",
	                     ssec->serial, ssec->ssid, ssec->msid, ssec->dsid));

		return NF_ACCEPT;
	}
	
	/*
	 * Slow path, send map request and queue packet.
	 */
	rc = selopt_cache_request_mappings(skb);
	if (rc)
		return NF_DROP;

	return NF_QUEUE;
}


/*
 * Return true if this packet needs to bypass normal labeling.  Currently, this
 * means unfragmented SCMP packets, and may later include ISAKMP packets.
 */
static int selopt_bypass_output(struct sk_buff *skb)
{
	struct iphdr *iph = skb->nh.iph;
	
	if (iph->protocol == IPPROTO_UDP) {
		struct udphdr *udph;
		__u8 ihlen = iph->ihl * 4;
	
		if (iph->frag_off & __constant_htons(IP_MF|IP_OFFSET))
			return 0;
			
		if ((skb->len - ihlen) < sizeof(struct udphdr))
			return 0;
		
		udph = (void *)iph + ihlen;
		if (udph->dest == __constant_htons(SCMP_PORT)
		    || udph->source == __constant_htons(SCMP_PORT))
			return 1;
	}
	return 0;
}

/* 
 * This is critical for TCP to operate with our CIPSO options.  The effective
 * MSS is the maximum size of segments which are transmitted, and it must
 * take the size of the options into account
 */
static void selopt_adjust_effective_mss(struct sock *sk)
{
	struct tcp_opt *tp = &sk->tp_pinfo.af_tcp;

	if (tp->ext_header_len < SELOPT_ALIGN(SELOPT_MAX_TCP_LEN)) {
		tp->ext_header_len = SELOPT_ALIGN(SELOPT_MAX_TCP_LEN);
		tcp_sync_mss(sk, tp->pmtu_cookie);
	}
	return;
}

/*
 * Returns true if the address is within our security perimeter.
 */
static int selopt_within_perimeter(__u32 addr)
{
	return selopt_perimtab_match(addr);
}

/*
 * If not a bypass label, always include serial and ssid.  The msid and
 * dsid tags are only included if they have been explicitly specified via
 * the extended socket API.
 */
static inline __u8 selopt_calculate_optlen(__u8 protocol,
                                           struct skb_security_struct *ssec)
{
	__u8 len = SELOPT_BASE_LEN;

	if (selopt_get(ssec, SELOPT_BYPASS))
		len += SELOPT_BYPASS_LEN;
	else {
		len += SELOPT_SERIAL_LEN + SELOPT_SSID_LEN;
		
		if (selopt_get(ssec, SELOPT_MSID)) {
			if (protocol != IPPROTO_TCP)
				len += SELOPT_MSID_LEN;
		}
		
		if (selopt_get(ssec, SELOPT_DSID))
			len += SELOPT_DSID_LEN;
	}
	return len;
}


/*
 * Fill optptr buffer with IP options.  Note that optlen is not 32-bit
 * aligned here.
 */
static void selopt_options_fill(struct sk_buff *skb,
                                unsigned char *optptr, __u8 optlen)
{
	__u8 i;
	struct skb_security_struct *ssec = skb->lsm_security;
	struct iphdr *iph = skb->nh.iph;
	
	memset(optptr, IPOPT_NOOP, SELOPT_ALIGN(optlen));
	
	optptr[0] = IPOPT_CIPSO;
	optptr[1] = optlen;
	
	*(__u32 *)&optptr[2] = __constant_htonl(SELOPT_DOI);
	
	i = SELOPT_BASE_LEN;
		
	if (selopt_get(ssec, SELOPT_BYPASS)) {
		optptr[i] = SELOPT_BYPASS;
		optptr[i+1] = SELOPT_BYPASS_LEN;
		i += SELOPT_BYPASS_LEN;
		goto out_tag;
	}
	
	optptr[i] = SELOPT_SERIAL;
	optptr[i+1] = SELOPT_SERIAL_LEN;
	*(__u32 *)&optptr[i+2] = htonl(ssec->serial);
	i += SELOPT_SERIAL_LEN;
	
	optptr[i] = SELOPT_SSID;
	optptr[i+1] = SELOPT_SSID_LEN;
	*(__u32 *)&optptr[i+2] = htonl(ssec->ssid);
	i += SELOPT_SSID_LEN;
	
	if (selopt_get(ssec, SELOPT_MSID)) {
		if (iph->protocol != IPPROTO_TCP) {
			optptr[i] = SELOPT_MSID;
			optptr[i+1] = SELOPT_MSID_LEN;
			*(__u32 *)&optptr[i+2] = htonl(ssec->msid);
			i += SELOPT_MSID_LEN;
		}
	}
	
	if (selopt_get(ssec, SELOPT_DSID)) {
		optptr[i] = SELOPT_DSID;
		optptr[i+1] = SELOPT_DSID_LEN;
		*(__u32 *)&optptr[i+2] = htonl(ssec->dsid);
		i += SELOPT_DSID_LEN;
	}
	
out_tag:
	optptr[6] = SELOPT_FREEFORM;
	optptr[7] = i - SELOPT_BASE_LEN + 2;
	return;
}

/*
 * Add labels to outgoing packets by inserting CIPSO/FIPS-188 options.
 * Note that packets on the output hook need to be length validated.
 */
static unsigned int selopt_ip_label_output(unsigned int hooknum,
                                           struct sk_buff **pskb,
                                           const struct net_device *in,
                                           const struct net_device *out,
                                           int (*okfn)(struct sk_buff *))
{
	__u8 optlen, ihlen, *optptr;
	__u16 datalen, totlen;
	struct sk_buff *skb = *pskb;
	struct skb_security_struct *ssec = skb->lsm_security;
	struct iphdr *iph = skb->nh.iph;
	
	totlen = skb->len;
	if (totlen < sizeof(struct iphdr))
		return NF_DROP;
	
	if (totlen != ntohs(iph->tot_len))
		return NF_DROP;
	
	if (!selopt_within_perimeter(iph->daddr))
		return NF_ACCEPT;
	
	ihlen = iph->ihl * 4;
	if (ihlen != sizeof(struct iphdr)) {
		ip_warn(iph, "unable to label packet with ihlen %u", ihlen);
		return NF_DROP;
	}

	if (selopt_bypass_output(skb))
		selopt_set(ssec, SELOPT_BYPASS);
		
	/* Temporary hack until policy interleave implemented. */
	selopt_set(ssec, SELOPT_SERIAL);
	ssec->serial = iph->saddr;
	
	/* SSID is always set */
	selopt_set(ssec, SELOPT_SSID);
	
	/* Check the other SIDs */
	if ((ssec->msid != SECINITSID_UNLABELED) && (ssec->msid != ssec->ssid))
		selopt_set(ssec, SELOPT_MSID);

	if (ssec->dsid != SECINITSID_UNLABELED)
		selopt_set(ssec, SELOPT_DSID);

	/* Packets with local destination do not need CIPSO labels. */
	if (inet_addr_type(iph->daddr) == RTN_LOCAL)
		return NF_ACCEPT;

	totlen = skb->len;
	datalen = totlen - ihlen;
	
	optlen = selopt_calculate_optlen(iph->protocol, ssec);
	totlen += SELOPT_ALIGN(optlen);

	if (totlen > 0xFFFF) {
		ip_warn(iph, "oversize packet %u\n", totlen);
		return NF_DROP;
	}
	
	if (skb_tailroom(skb) < SELOPT_ALIGN(optlen)
	    || skb_cloned(skb) || skb_shared(skb)) {
		struct sk_buff *newskb;
		struct skb_security_struct *new_ssec;

		newskb = skb_copy_expand(skb, skb_headroom(skb),
		                         SELOPT_ALIGN(optlen), SAFE_ALLOC);
		if (!newskb) {
			ip_warn(iph, "out of memory expanding skb\n");
			return NF_DROP;
		}
		
		if (skb->sk)
			skb_set_owner_w(newskb, skb->sk);

		/* Copy the MSID and DSID from the old SKB security structure
		 * because skb_set_owner_w() hooks have special processing to
		 * handle extended socket calls.
		 */
		new_ssec = newskb->lsm_security;
		new_ssec->msid = ssec->msid;
		new_ssec->dsid = ssec->dsid;		
			
		kfree_skb(skb);
		
		*pskb = skb = newskb;
		iph = skb->nh.iph;
	}
	
	skb_put(skb, SELOPT_ALIGN(optlen));
	optptr = (__u8 *)iph + sizeof(struct iphdr);
	memmove(optptr + SELOPT_ALIGN(optlen), optptr, datalen);
	skb->h.raw += SELOPT_ALIGN(optlen);
	
	selopt_options_fill(skb, optptr, optlen);
	
	iph->ihl = (ihlen + SELOPT_ALIGN(optlen)) >> 2;
	iph->tot_len = htons(totlen);
	iph->check = 0;
	ip_send_check(iph);
	
	return NF_ACCEPT;
}

/*
 * Map remote SIDs on incoming packets to local SIDs.
 */
static unsigned int selopt_ip_map_input(unsigned int hooknum,
                                        struct sk_buff **pskb,
                                        const struct net_device *in,
                                        const struct net_device *out,
                                        int (*okfn)(struct sk_buff *))
{
	struct sk_buff *skb = *pskb;
	struct skb_security_struct *ssec = skb->lsm_security;
	struct iphdr *iph = skb->nh.iph;
	
	if (selopt_within_perimeter(iph->saddr)) {
		
		if (!ssec->opts) {
			/*
			 * CIPSO 5.1.2: Respond with a type 12 code 1 ICMP
			 * message with a pointer value of 134. (todo).
			 */
			ip_warn(iph, "unlabeled packet from within perimeter\n");
			return NF_DROP;
		}
		
		return selopt_map_sids(hooknum, pskb, in, out, okfn);
	}
	
	if (ssec->opts) {
		ip_warn(iph, "labeled packet from outside perimeter\n");
		return NF_DROP;
	}

	return NF_ACCEPT;
}

static inline int selopt_param(struct skb_security_struct *ssec,
                              __u8 paramtype, __u8 optlen, int i,
                              __u8 reqlen, const char *optptr,
                              unsigned char **pp_ptr)
{
	__u8 paramlen;
	
	if (selopt_get(ssec, paramtype)) {
		(const unsigned char *)*pp_ptr = &optptr[i];
		return -EINVAL;
	}
	
	paramlen = optptr[i + 1];
	if (paramlen != reqlen || paramlen > optlen - i) {
		(const unsigned char *)*pp_ptr = &optptr[i] + 1;
		return -EINVAL;
	}

	selopt_set(ssec, paramtype);
	return paramlen;
}
	
static inline int selopt_parse_null_param(struct skb_security_struct *ssec,
                                          __u8 paramtype, __u8 optlen, int i,
                                          const char *optptr,
                                          unsigned char **pp_ptr)
{

	return selopt_param(ssec, paramtype, optlen, i,
	                    SELOPT_NULL_LEN, optptr, pp_ptr);
}

static inline int selopt_parse_u32_param(struct skb_security_struct *ssec,
                                         __u8 paramtype, __u8 optlen, int i,
                                         const char *optptr,
                                         unsigned char **pp_ptr, __u32 *param)
{
	int rc;
	
	rc = selopt_param(ssec, paramtype, optlen, i,
	                 SELOPT_U32_LEN, optptr, pp_ptr);
	if (rc)
		*param = ntohl(*(__u32 *)&optptr[i + 2]);
	return rc;
}

/*
 * Decode IP options on incoming packets into labels.
 *
 * Note: we're guaranteed to have a minimum of 2 bytes at optptr, with
 * optptr[1] being a valid length of data to access.
 */
static int selopt_ip_decode_options(struct sk_buff *skb,
                                    const char *optptr,
                                    unsigned char **pp_ptr)
{
	__u8 opttype = optptr[0];
	__u8 optlen = optptr[1];
	__u8 i, tagtype, taglen;
	__u32 doi;
	struct iphdr *iph;
	struct skb_security_struct *ssec;
	
	/*
	 * No user can set security options on outgoing packets when labeling
	 * is enabled.
	 */
	if (!skb)
		return -EPERM;
	
	/*
	 * Received packet is not being delivered locally, so we don't care
	 * about its security labels.
	 */
	iph = skb->nh.iph;
	if (inet_addr_type(iph->daddr) != RTN_LOCAL)
		return 0;

	ssec = skb->lsm_security;
	
	/* This is the only type of security option which we accept */
	if (opttype != IPOPT_CIPSO) {
		(const unsigned char *)*pp_ptr = optptr;
		return -EPERM;
	}

	if (ssec->opts) {
		if (inet_addr_type(iph->saddr) != RTN_LOCAL) {
			ip_warn(iph, "packet already labeled %#x\n", ssec->opts)
			return 0;
		}
	}
	
	if (optlen < SELOPT_MIN_LEN) {
		(const unsigned char *)*pp_ptr = optptr;
		return -EINVAL;
	}

	doi = *(__u32 *)&optptr[2];
	if (doi != __constant_htonl(SELOPT_DOI)) {
		(const unsigned char *)*pp_ptr = optptr + 3;
		return -EINVAL;
	}

	tagtype = optptr[6];
	if (tagtype != SELOPT_FREEFORM) {
		(const unsigned char *)*pp_ptr = optptr + 7;
		return -EINVAL;
	}

	taglen = optptr[7];
	if (taglen != (optlen - 6)) {
		(const unsigned char *)*pp_ptr = optptr + 8;
		return -EINVAL;
	}

	for (i = 8; i < optlen; ) {
		__u8 paramtype = optptr[i];
		int rc = 0;
		
		switch (paramtype) {
		case SELOPT_BYPASS:
			rc = selopt_parse_null_param(ssec, paramtype, optlen,
			                             i, optptr, pp_ptr);
			break;

		case SELOPT_SERIAL:
			rc = selopt_parse_u32_param(ssec, paramtype, optlen,
			                            i, optptr, pp_ptr,
			                            &ssec->serial);
			break;

		case SELOPT_SSID:
			rc = selopt_parse_u32_param(ssec, paramtype, optlen,
			                            i, optptr, pp_ptr,
			                            &ssec->ssid);
			break;

		case SELOPT_MSID:
			rc = selopt_parse_u32_param(ssec, paramtype, optlen,
			                            i, optptr, pp_ptr,
			                            &ssec->msid);
			break;

		case SELOPT_DSID:
			rc = selopt_parse_u32_param(ssec, paramtype, optlen,
			                            i, optptr, pp_ptr,
			                            &ssec->dsid);
			break;
	
		default:
			(const unsigned char *)*pp_ptr = &optptr[i] + 1;
			return -EINVAL;
		}
		
		if (rc < 0)
			return rc;
		
		i += rc;
	}

	if (i != optlen) {
		(const unsigned char *)*pp_ptr = &optptr[2];
		return -EINVAL;
	}

	return 0;
}

/*
 * Validate labels on incoming fragments as they are about to be added
 * to a reassembly queue.
 */
static int selopt_ip_defragment(struct sk_buff *skb)
{
	struct iphdr *iph = skb->nh.iph;
	struct skb_security_struct *ssec = skb->lsm_security;
	
	if (ssec->opts)
		ip_warn(iph, "labeled fragments yet supported\n");
	return 0;
}

/*
 * Adjust effective MSS for outgoing TCP data segments.
 */
static void selopt_sock_sendmsg(struct sock *sk)
{
	if (sk->family == AF_INET && sk->type == SOCK_STREAM)
		if (selopt_within_perimeter(sk->daddr))
			selopt_adjust_effective_mss(sk);
	return;
}

static struct nsid_operations selopt_ops =
{
	ip_label_output:	selopt_ip_label_output,
	ip_map_input:		selopt_ip_map_input,
	ip_decode_options:	selopt_ip_decode_options,
	ip_defragment:		selopt_ip_defragment,
	sock_sendmsg:		selopt_sock_sendmsg,
};

__init int selopt_init(void)
{
	int rc;
	
	rc = flnetlink_init();
	if (rc)
		return rc;
	
	rc = selopt_perimtab_init();
	if (rc) {
		flnetlink_exit();
		return rc;
	}
	
	rc = selopt_cache_init();
	if (rc) {
		selopt_perimtab_exit();
		flnetlink_exit();
		return rc;
	}

	rc = selopt_queue_init();
	if (rc) {
		selopt_cache_exit();
		selopt_perimtab_exit();
		flnetlink_exit();
		return rc;
	}

	rc = nsid_register_ops(&selopt_ops);
	if (rc) {
		selopt_queue_exit();
		selopt_cache_exit();
		selopt_perimtab_exit();
		flnetlink_exit();
		return rc;
	}
	
	printk(KERN_INFO "SELinux: CIPSO/FIPS-188 IP labeling initialized\n");
	
	return 0;
}

__exit void selopt_exit(void)
{
	nsid_unregister_ops(&selopt_ops);
	selopt_queue_exit();
	selopt_cache_exit();
	selopt_perimtab_exit();
	flnetlink_exit();
	
	return;
}

module_init(selopt_init);
module_exit(selopt_exit);

MODULE_DESCRIPTION("SELinux CIPSO/FIPS-188 IP Labeling");
MODULE_LICENSE("GPL");


