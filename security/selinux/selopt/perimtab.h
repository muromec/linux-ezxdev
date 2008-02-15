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
#ifndef _SELOPT_PERIMTAB_H_
#define _SELOPT_PERIMTAB_H_

int selopt_perimtab_append(__u32 addr, __u32 mask);
int selopt_perimtab_match(__u32 addr);
int selopt_perimtab_delete(__u32 addr, __u32 mask);
void selopt_perimtab_flush(void);

int selopt_perimtab_init(void) __init;
void selopt_perimtab_exit(void);

#include <linux/skbuff.h>
#include <linux/netlink.h>

int selopt_perimtab_dump(struct sk_buff *skb, struct netlink_callback *cb);

#endif	/* _SELOPT_PERIMTAB_H_ */
