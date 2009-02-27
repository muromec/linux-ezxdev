/*
 * Header for MultiMediaCard (MMC)
 *
 * Copyright 2002 Hewlett-Packard Company
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * HEWLETT-PACKARD COMPANY MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
 * AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
 * FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 * Many thanks to Alessandro Rubini and Jonathan Corbet!
 *
 * Based strongly on code by:
 *
 * Author: Yong-iL Joh <tolkien@mizi.com>
 * Date  : $Date: 2003/09/19 16:54:27 $ 
 *
 * Author:  Andrew Christian
 *          15 May 2002
 */
/*
 * Copyright (C) 2005 Motorola Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*
 * modified by w20598, for EZX platform
 */


#ifndef MMC_MMC_MEDIA_H
#define MMC_MMC_MEDIA_H

#include <linux/interrupt.h>
#include <linux/list.h>

#include <linux/mmc/mmc_protocol.h>

/* Set an upper bound for how many cards we'll support */
/* This is used only for static array initialization */
#define MMC_MAX_SLOTS   2

#define MMC_SLOT_FLAG_INSERT  (1<<0)
#define MMC_SLOT_FLAG_EJECT   (1<<1)
#define MMC_SLOT_FLAG_LOCKED  (1<<2)
#define MMC_SLOT_FLAG_LOCK_FAILED (1<<3)

struct mmc_media_driver;

struct mmc_slot {
	int             id;     /* Card index */
	/* Card specific information */
	struct mmc_cid  cid;
	struct mmc_csd  csd;

	enum card_state state;  /* empty, ident, ready, whatever */
	int             flags;  /* Ejected, inserted */
        int             sd;     /* MMC or SD card */
        int             rca;    /* RCA */
        u32             scr;    /* SCR 63:32*/        
	/* Assigned media driver */
	struct mmc_media_driver *media_driver;
	struct mmc_dev 		*dev;
};

#define MMC_IO_UNKNOWN	(-1)
#define MMC_IO_READ	(1)
#define MMC_IO_WRITE	(2)
#define MMC_IO_LOCK	(3)

struct mmc_io_request {
	int            id;         /* Card index     */
	int            cmd;        /* READ or WRITE  */
	unsigned long  sector;     /* Start address  */
	unsigned long  nr_sectors; /* Length of read */
	unsigned long  block_len;  /* Size of sector (sanity check) */
	char          *buffer;     /* Data buffer    */
	void	      *done_data;  /* completion */
	void	      (*done)(struct mmc_io_request*); /* completion function */
};

/* Media driver (e.g., Flash card, I/O card...) */
struct mmc_media_driver {
	struct list_head   node;
	char              *name;
	void (*load)(struct mmc_slot *);
	void (*unload)(struct mmc_slot *);
	int  (*probe)(struct mmc_slot *);
	void (*io_request_done)(struct mmc_io_request *, int result);
};

struct mmc_media_module {
	int (*init)(void);
	void (*cleanup)(void);
};

/* Calls made by the media driver */
extern int  mmc_register_media_driver( struct mmc_media_driver * );
extern void mmc_unregister_media_driver( struct mmc_media_driver * );
extern int  mmc_handle_io_request( struct mmc_io_request * );

#endif  /* MMC_MMC_MEDIA_H */

