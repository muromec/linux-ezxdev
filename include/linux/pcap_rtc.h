#ifndef __PCAP_RTC_H__
#define __PCAP_RTC_H__
/*
 * linux/drivers/char/pcap_rtc.c
 *
 * Copyright (C) 2004-2005 Motorola Inc.
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
 * Originally written by Yin Kangkai (e12051@motorola.com)
 */

#include <linux/ioctl.h>

#define PCAP_RTC_NAME   "pcap_rtc"

/* pcap_rtc get time (seconds since 2000) */
#define PCAP_RTC_IOC_GET_TIME           _IOR('p', 1, struct timeval *)
/* pcap_rtc set time (seconds since 2000) */
#define PCAP_RTC_IOC_SET_TIME           _IOW('p', 1, struct timeval *)
/* pcap_rtc get alarm time (seconds since 2000) */
#define PCAP_RTC_IOC_GET_ALARM_TIME     _IOR('p', 2, struct timeval *)
/* pcap_rtc set alarm time (seconds since 2000) */
#define PCAP_RTC_IOC_SET_ALARM_TIME     _IOW('p', 2, struct timeval *)

#endif  /* __PCAP_RTC_H__ */
