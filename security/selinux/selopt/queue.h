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
 *
 */
#ifndef _SELOPT_QUEUE_H_
#define _SELOPT_QUEUE_H_

#include "cache.h"

void selopt_queue_evict(struct selopt_cache_peer *peer);
void selopt_queue_flush(int verdict);
int selopt_queue_dump(struct sk_buff *skb, struct netlink_callback *cb);

int selopt_queue_init(void) __init;
void selopt_queue_exit(void);

#endif	/* _SELOPT_QUEUE_H_ */
