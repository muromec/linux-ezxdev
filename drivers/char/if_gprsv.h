/*
 * if_gprsv.h
 *
 * GPRS Interface for Linux.
 *      This module implemented a tty line discipline and a virtual network 
 *      device. The macro NR_LDISCS defined in tty.h should be changed to 17, and in 
 *      termios.h and a new line discipline is defined(#define N_GPRS 16).
 *
 * Copyright (C) 2002, 2005 - Motorola
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
 *     December 2002, created.
 *     March 2005, add support for modem bridge.
 */

#ifndef _IF_GPRSV_H_
#define _IF_GPRSV_H_

/* #define N_GPRS 15 */
/* #define ENABLE_DNS_BROADCAST */


/*
 * socket Ioctl definitions.
 */
struct gprsv_stats{
    unsigned long gprsv_ibytes;	/* bytes received */
    unsigned long gprsv_obytes;	/* bytes sent */
};
struct ifgprsvstatsreq {
    struct ifreq	 b;
    struct gprsv_stats stats;			/* statistic information */
};

/*
 * The following structure records the time in seconds since
 * the last NP packet was sent or received.
 */
struct gprsv_idle {
    time_t xmit_idle;		/* time since last NP packet sent */
    time_t recv_idle;		/* time since last NP packet received */
};
struct ifgprsvidlesreq {
	struct ifreq	 b;
	struct gprsv_idle idle;			/* statistic information */
};

#define ifr__name       b.ifr_ifrn.ifrn_name
#define stats_ptr       b.ifr_ifru.ifru_data

#define SIOCGGPRSVSTATS     (SIOCDEVPRIVATE + 0)
#define SIOCGGPRSIDLETIME   (SIOCDEVPRIVATE + 1)

#define GPRSVIOCNEWUNIT       _IOWR('t', 160, int)   /*create GPRS unit*/
#define GPRSVIOCNEWBRDEVICE   _IOWR('t', 161, int)   /*create GPRS bridge device*/
#define GPRSVIOCBINDPEER      _IOWR('t', 162, int)   /*bind with another bridge device*/

#endif
