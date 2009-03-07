#ifndef __MODEM_PPP_CHANNEL_H__
#define __MODEM_PPP_CHANNEL_H__
/*   
 * modem_ppp_channel.h 
 *
 * Copyright 1999 Paul Mackerras.
 * Copyright (C) 2004, 2005 Motorola
 *
 * Definitions for the interface between the generic PPP code
 * and a PPP channel.
 *
 * A PPP channel provides a way for the generic PPP code to send
 * and receive packets over some sort of communications medium.
 * Packets are stored in sk_buffs and have the 2-byte PPP protocol
 * number at the start, but not the address and control bytes.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA. 
 * 
 * Change History:
 * 2004-Nov-19  -- change funtion naming style to keep consistency with 
 *                 modem_ppp_generic.c and modem_ppp_async. used for EzX 
 *                 modem bridge only.
 * 2005-April-4 -- Clear kloc warning
 */


#include <linux/list.h>
#include <linux/skbuff.h>
#include <linux/poll.h>
#include <linux/ppp_defs.h>

struct modem_ppp_channel;

struct modem_ppp_channel_ops {
	/* Send a packet (or multilink fragment) on this channel.
	   Returns 1 if it was accepted, 0 if not. */
	int	(*start_xmit)(struct modem_ppp_channel *, struct sk_buff *);
	/* Handle an ioctl call that has come in via /dev/ppp. */
	int	(*ioctl)(struct modem_ppp_channel *, unsigned int, unsigned long);
    /* handle flow control request from the other end */
	int	(*flowctl)(struct modem_ppp_channel *, int);

};

struct modem_ppp_channel {
	void		*private;	/* channel private data */
	struct modem_ppp_channel_ops *ops;	/* operations for this channel */
	int		mtu;		/* max transmit packet size */
	int		hdrlen;		/* amount of headroom channel needs */
	void		*ppp;		/* opaque to channel */
	/* the following are not used at present */
	int		speed;		/* transfer rate (bytes/second) */
	int		latency;	/* overhead time in milliseconds */
};

#ifdef __KERNEL__
/* Called by the channel when it can send some more data. */
extern void modem_ppp_output_wakeup(struct modem_ppp_channel *);

/* Called by the channel to process a received PPP packet.
   The packet should have just the 2-byte PPP protocol header. */
extern void modem_ppp_input(struct modem_ppp_channel *, struct sk_buff *);

/* Called by the channel when an input error occurs, indicating
   that we may have missed a packet. */
extern void modem_ppp_input_error(struct modem_ppp_channel *, int code);

/* Attach a channel to a given PPP unit. */
extern int modem_ppp_register_channel(struct modem_ppp_channel *);

/* Detach a channel from its PPP unit (e.g. on hangup). */
extern void modem_ppp_unregister_channel(struct modem_ppp_channel *);

/* Get the channel number for a channel */
extern int modem_ppp_channel_index(struct modem_ppp_channel *);

/* Get the unit number associated with a channel, or -1 if none */
extern int modem_ppp_unit_number(struct modem_ppp_channel *);

extern int modem_ppp_idle_time( struct modem_ppp_channel *, struct ppp_idle * );

/*
 * SMP locking notes:
 * The channel code must ensure that when it calls ppp_unregister_channel,
 * nothing is executing in any of the procedures above, for that
 * channel.  The generic layer will ensure that nothing is executing
 * in the start_xmit and ioctl routines for the channel by the time
 * that ppp_unregister_channel returns.
 */

#endif /* __KERNEL__ */
#endif
