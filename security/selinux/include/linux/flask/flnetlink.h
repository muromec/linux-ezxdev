/* -*- linux-c -*- */
/*
 * Flask Netlink Interface
 *
 * Copyright (c) 2001-2002 James Morris <jmorris@intercode.com.au>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 */
#ifndef _LINUX_FLASK_FLNETLINK_H_
#define _LINUX_FLASK_FLNETLINK_H_

/* This probably needs to be in <linux/netlink.h> */
#define NETLINK_FLASK		6

/* Message types. */
#define FLMSG_BASE		0x10
enum {
	FLMSG_PERIM_ADD = FLMSG_BASE,
	FLMSG_PERIM_GET,
	FLMSG_PERIM_DEL,
	FLMSG_PERIM_FLUSH,
	
	FLMSG_CACHE_ADD,
	FLMSG_CACHE_GET,
	FLMSG_CACHE_DEL,
	FLMSG_CACHE_FLUSH,
	
	FLMSG_CACHE_MAP_ADD,
	FLMSG_CACHE_MAP_GET,
	FLMSG_CACHE_MAP_DEL,
	FLMSG_CACHE_MAP_REQ,
	FLMSG_CACHE_MAP_RES,
	FLMSG_CACHE_MAP_FLUSH,
	
	FLMSG_QUEUE_GET,
	FLMSG_QUEUE_FLUSH,
	
	FLMSG_MAX
};

/* Multicast groups. */
#define FLN_G_NONE		0x00000000
#define FLN_G_PERIM		0x00000001
#define FLN_G_CACHE		0x00000002
#define FLN_G_QUEUE		0x00000004
#define FLN_G_ALL		0xffffffff

struct flmsg_base {
	u_int32_t serial;
	u_int32_t peer;
	u_int16_t count;
};

struct flmsg_attr_perim {
	u_int32_t addr;
	u_int32_t mask;
};

struct flmsg_attr_map {
	security_id_t lsid;
	security_id_t rsid;
};

struct flmsg_attr_queue {
	security_id_t sid;
	u_int8_t mapped;
};

struct flmsg_perim_entry {
	struct flmsg_base base;
	struct flmsg_attr_perim entry;
};

struct flmsg_map_req {
	struct flmsg_base base;
	security_id_t sid[0];
};

struct flmsg_map_res {
	struct flmsg_base base;
	struct flmsg_attr_map map[0];
};

struct flmsg_queue_entry {
	struct flmsg_base base;
	u_int32_t ttl;
	u_int8_t proto;
	u_int16_t sport;
	u_int16_t dport;
	struct flmsg_attr_queue ssid_attr;
	struct flmsg_attr_queue msid_attr;
	struct flmsg_attr_queue dsid_attr;
};

#ifdef __KERNEL__

extern struct sock *flnl;

typedef int (*fln_dumpfn)(struct sk_buff * skb, struct netlink_callback *cb);

int flnetlink_init(void) __init;
void flnetlink_exit(void);

#endif	/* __KERNEL__ */
                                                                          
#endif	/* _LINUX_FLASK_FLNETLINK_H_ */
