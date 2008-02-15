/* -*- linux-c -*- */
/*
 * Peer NSID mapping cache.
 *
 * Copyright (c) 2001-2002 James Morris <jmorris@intercode.com.au>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 */
#ifndef _SELOPT_CACHE_H_
#define _SELOPT_CACHE_H_

#include <linux/flask/flnetlink.h>

#define SELOPT_CACHE_NEW	1

struct selopt_cache_map {
	struct list_head list;
	security_id_t lsid;
	security_id_t rsid;
};

struct selopt_cache_peer {
	struct list_head list;
	rwlock_t map_lock;
	struct selopt_cache_map *map;
	__u32 map_total;
	__u32 addr;
	__u32 serial;
	atomic_t use;
};

struct selopt_cache_peer *selopt_cache_peer_lookup(__u32 addr, int *err);

inline void selopt_cache_peer_get(struct selopt_cache_peer *peer);

inline void selopt_cache_peer_put(struct selopt_cache_peer *peer);

void selopt_cache_map_sids(struct selopt_cache_peer *peer,
                           struct sk_buff *skb, int status);

int selopt_cache_request_mappings(struct sk_buff *skb);

int selopt_cache_map_response(struct flmsg_map_res *res);

int selopt_cache_dump(struct sk_buff *skb, struct netlink_callback *cb);

void selopt_cache_flush(void);

int selopt_cache_init(void) __init;
void selopt_cache_exit(void);

#endif	/* _SELOPT_CACHE_H_ */
