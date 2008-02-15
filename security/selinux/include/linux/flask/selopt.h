/* -*- linux-c -*- */
/*
 * SELinux CIPSO/FIPS-188 IP Labeling
 *
 * Copyright (c) 2001-2002 James Morris <jmorris@intercode.com.au>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 */
#ifndef _LINUX_FLASK_SELOPT_H_
#define _LINUX_FLASK_SELOPT_H_

#define SELOPT_DOI		0x10001000	/* Made up */
#define SELOPT_FREEFORM		7		/* FIPS-188, type 7 tag */
#define SELOPT_BASE_LEN		8		/* type+len+doi+tagtype+taglen*/
#define SELOPT_NULL_LEN		2		/* Length of null parameter */
#define SELOPT_U32_LEN		6		/* Length of 32-bit parameter */
#define SELOPT_BYPASS		1		/* Bypass label */
#define SELOPT_BYPASS_LEN	SELOPT_NULL_LEN	/* Bypass label length */
#define SELOPT_SERIAL		2		/* Policy serial label */
#define SELOPT_SERIAL_LEN	SELOPT_U32_LEN	/* Policy serial length */
#define SELOPT_SSID		3		/* Source SID label */
#define SELOPT_SSID_LEN		SELOPT_U32_LEN	/* Source SID length */
#define SELOPT_MSID		4		/* Message SID label */
#define SELOPT_MSID_LEN		SELOPT_U32_LEN	/* Message SID length */
#define SELOPT_DSID		5		/* Destination SID label */
#define SELOPT_DSID_LEN		SELOPT_U32_LEN	/* Destination SID length */

#define SCMP_PORT		40000		/* IANA unassigned */

#ifdef __KERNEL__

#undef DEBUG_LABELING

#define SELOPT_ALIGN(val)		((val + 3) & ~3)
#define SELOPT_MIN_LEN			SELOPT_BASE_LEN + SELOPT_BYPASS_LEN
#define SELOPT_STD_LEN			(SELOPT_BASE_LEN \
                                         + SELOPT_SERIAL_LEN + SELOPT_SSID_LEN)
#define SELOPT_MAX_TCP_LEN		(SELOPT_BASE_LEN + SELOPT_SERIAL_LEN \
                                         + SELOPT_SSID_LEN + SELOPT_DSID_LEN)

static inline int selopt_get(struct skb_security_struct *ssec, __u8 val)
{
	return ssec->opts & (1 << val);
}

static inline void selopt_set(struct skb_security_struct *ssec, __u8 val)
{
	ssec->opts |= (1 << val);
}

static inline void selopt_map(struct skb_security_struct *ssec, __u8 val)
{
	ssec->mapped |= (1 << val);
}

static inline void selopt_map_all(struct skb_security_struct *ssec)
{
	ssec->mapped = ssec->opts & ~(1 << SELOPT_SERIAL);
}

static inline int selopt_mapped(struct skb_security_struct *ssec, __u8 val)
{
	return ssec->mapped & (1 << val);
}

static inline int selopt_mapped_all(struct skb_security_struct *ssec)
{
	return ssec->mapped == (ssec->opts & ~(1 << SELOPT_SERIAL));
}

static inline int selopt_map_needed(struct skb_security_struct *ssec, __u8 val)
{
	return selopt_get(ssec, val) && !selopt_mapped(ssec, val);
}

static inline int selopt_maps_needed(struct skb_security_struct *ssec)
{
	int i = 0;
	
	if (selopt_map_needed(ssec, SELOPT_SSID)) i++;
	if (selopt_map_needed(ssec, SELOPT_MSID)) i++;
	if (selopt_map_needed(ssec, SELOPT_DSID)) i++;
	
	return i;
}

#define ip_warn(iph, fmt, args...)                                     \
                                                                       \
       do { printk(KERN_WARNING "%s: " fmt, __FUNCTION__ , ## args);     \
            printk(" [%u.%u.%u.%u->%u.%u.%u.%u]\n",                    \
            NIPQUAD(iph->saddr), NIPQUAD(iph->daddr));                 \
       } while (0);


#ifdef DEBUG_LABELING
#define SDBG(x)		do { x; } while (0);
#else
#define SDBG(x)		do { } while (0);
#endif	/* DEBUG_LABELING */

#endif	/* __KERNEL__ */

#endif /* _LINUX_FLASK_SELOPT_H_ */

