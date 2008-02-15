/*
 * ioctl.c
 *
 * Ioctl method for directory and regular files.
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc.
 * Copyright 2003 Sony Corporation
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 *
 * This software is being distributed under the terms of the GNU General Public
 * License version 2.  Some or all of the technology encompassed by this
 * software may be subject to one or more patents pending as of the date of
 * this notice.  No additional patent license will be required for GPL
 * implementations of the technology.  If you want to create a non-GPL
 * implementation of the technology encompassed by this software, please
 * contact legal@mvista.com for details including licensing terms and fees.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#include <linux/fs.h>
#include <linux/pram_fs.h>
#include <linux/sched.h>
#include <asm/uaccess.h>


int pram_ioctl (struct inode * inode, struct file * filp, unsigned int cmd,
		unsigned long arg)
{
	// FIXME: need any special ioctl's?
	return 0;
}
