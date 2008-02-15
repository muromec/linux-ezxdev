/*
 * ptf_fd/ptfproto.h
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *      Bruce Balden <balden@lineo.com>
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
 *
 * Copyright (C) 2003 - Motorola
 *
 *--------Motorola EZX- history
 *26-02-2003    created by a17400 try to finish char device driver
 *
 *
 *
 */

#ifndef PTFPROTO_H
#define PTFPROTO_H 1

#define PTF_MAX_DEV_NUMBER	2	//define maximum device number
extern int ptfproto_modinit (char *, int);
extern int ptf_open (struct inode *, struct file *);
extern int ptf_release (struct inode *, struct file *);
extern int ptf_fasync(int fd, struct file *filp, int mode);
extern int ptf_read (struct file *, char *, size_t, loff_t *);
extern int ptf_write (struct file *, const char *, size_t,loff_t *);
extern int ptfproto_create(char *name,int (*xmit_data) (int, unsigned char *, int), int tx_size);
extern unsigned int ptf_poll (struct file *, struct poll_table_struct * );
extern int ptfproto_destory(int dev_num);
extern int ptfproto_recv(int , unsigned char * , size_t );
extern void ptfproto_modexit (void);
extern void ptf_dbg_print (char *);

extern void enable_ptf_func_driver(void);
extern void disable_ptf_func_driver(void);
extern int check_ptf_func_driver(void);
/* a17400: comment to flags:
ptf_cable_connected-- indicate run time if usb cable is plugged to device from host.
ptf_func_created-- indicate char driver and usb function driver for PTF is initialized, only once in run time...
 */
extern int ptf_cable_connected;
extern int ptf_func_created;
extern wait_queue_head_t ptf_write_queue;
extern void ptfproto_send_fasync_to_all_devices(int sig);
//extern int serproto_create (char *, int (*xmit_data) (int, unsigned char *, int), int, int, int, int, int);
//extern int ptfproto_done (int, void *, int, int);
//extern int ptfproto_recv (int, unsigned char *, int);
//extern int serproto_control (int, int);
//#define SERPROTO_DISCONNECT 0
//#define SERPROTO_CONNECT    1
//extern int serproto_destroy (int);


#endif
