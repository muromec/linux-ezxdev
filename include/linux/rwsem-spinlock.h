/*
 * Copyright 2005 Motorola, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 */

/*
 * Revision History:
 *                    Modification     Tracking
 * Author                 Date          Number     Description of Changes
 * ----------------   ------------    ----------   -------------------------
 * Jiang Lili(w14359)  05/08/2005     LIBff43246   basic linux OS change for Barbados p1 board
 *
 */

/* rwsem-spinlock.h: fallback C implementation
 *
 * Copyright (c) 2001   David Howells (dhowells@redhat.com).
 * - Derived partially from ideas by Andrea Arcangeli <andrea@suse.de>
 * - Derived also from comments by Linus
 *
 * Trylock by Brian Watson (Brian.J.Watson@compaq.com).
 */

#ifndef _LINUX_RWSEM_SPINLOCK_H
#define _LINUX_RWSEM_SPINLOCK_H

#ifndef _LINUX_RWSEM_H
#error please dont include linux/rwsem-spinlock.h directly, use linux/rwsem.h instead
#endif

#include <linux/spinlock.h>
#include <linux/list.h>

#ifdef __KERNEL__

#include <linux/types.h>

struct rwsem_waiter;

/*
 * the rw-semaphore definition
 * - if activity is 0 then there are no active readers or writers
 * - if activity is +ve then that is the number of active readers
 * - if activity is -1 then there is one active writer
 * - if wait_list is not empty, then there are processes waiting for the semaphore
 */
struct rw_semaphore {
	__s32			activity;
	spinlock_t		wait_lock;
	struct list_head	wait_list;
#if RWSEM_DEBUG
	int			debug;
#endif
};

/*
 * initialisation
 */
#if RWSEM_DEBUG
#define __RWSEM_DEBUG_INIT      , 0
#else
#define __RWSEM_DEBUG_INIT	/* */
#endif

#define __RWSEM_INITIALIZER(name) \
{ 0, SPIN_LOCK_UNLOCKED, LIST_HEAD_INIT((name).wait_list) __RWSEM_DEBUG_INIT }

#define DECLARE_RWSEM(name) \
	struct rw_semaphore name = __RWSEM_INITIALIZER(name)

extern void FASTCALL(init_rwsem(struct rw_semaphore *sem));
extern void FASTCALL(__down_read(struct rw_semaphore *sem));
extern void FASTCALL(__try_down_read(struct rw_semaphore *sem, 
        unsigned int *result));
extern int FASTCALL(__down_read_trylock(struct rw_semaphore *sem));
extern void FASTCALL(__down_write(struct rw_semaphore *sem));
extern int FASTCALL(__down_write_trylock(struct rw_semaphore *sem));
extern void FASTCALL(__up_read(struct rw_semaphore *sem));
extern void FASTCALL(__up_write(struct rw_semaphore *sem));

#endif /* __KERNEL__ */
#endif /* _LINUX_RWSEM_SPINLOCK_H */
