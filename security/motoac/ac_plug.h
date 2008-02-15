#ifndef AC_PLUG_H
#define AC_PLUG_H
/*================================================================================
                                                                               
                      Header Name: ac_plug.h

General Description: security structure definitions for different classes
 
==================================================================================
 
   Copyright (C) 2005 - Motorola
 
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.
 
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
 
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   
Revision History:
                            Modification     Tracking
Author(core ID)                 Date          Number      Description of Changes
------------------------    ------------    ----------   -----------------------------------
Wang Yang (w20619)           01/28/2005     LIBff54376   Init version   

==================================================================================*/

#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/dcache.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/un.h>
#include <net/af_unix.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/tcp.h>

#define AC_DEFAULT_POLICY 1

/*
 * Security structure definitions
 */
struct task_security_struct {
    unsigned long magic;           /* magic number for this module */
	struct task_struct *task;      /* back pointer to task object */
	struct list_head list;         /* list of task_security_struct */
	int in_sid;
	int pno;                       /* process type no*/
};

struct sock_security_struct {
	unsigned long magic;           /* magic number for this module */
	struct sock *sk;               /* back pointer to sock object */
	struct list_head list;         /* list of sock_security_struct */
	int    rno;                    /* resource type no for this socket */
};

struct inode_security_struct {
	unsigned long magic;           /* magic number for this module */
    struct inode *inode;           /* back pointer to inode object */
    struct list_head list;         /* list of inode_security_struct */
	int rno;                       /* resource type no of this object */
	int class_no;                  /* class type no of this object */
	unsigned char initialized;     /* initialization flag */
	struct semaphore sem;          
};

struct file_security_struct {
	unsigned long magic;            /* magic number for this module */
	struct file *file;              /* back pointer to file object */
	struct list_head list;          /* list of file_security_struct */
	int rno;                        /* resource type no for the file */
};

struct skb_security_struct {
	int sno;
};

extern int ac_policy_init;
extern int ac_policy_enable;
extern int inode_security_set_sid(struct inode *inode, int sid);
#endif
