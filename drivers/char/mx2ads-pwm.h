/*
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
 *
 * Copyright (C) 2002 Motorola Semiconductors HK Ltd
 * Copyright (C) 2003 MontaVista Software Inc. <source@mvista.com>
 *
 */
#ifndef PWM_H
#define PWM_H

#include <linux/timer.h>
#include <linux/sched.h>

#include "mx2ads-pwm_ioctl.h"

#define MOD_NAME	"pwm"
#define DEV_IRQ_NAME		MOD_NAME
#define DEV_IRQ_ID		MOD_NAME
#define AITC_PWM_INT		34

#define MAX_ID			0x14

#define	PWM_FIFO_AV		0x20
#define	PWM_IRQ_AV		0x80
#define	PWM_EN		1<<4
#define	PWM_IRQ_EN		1<<6
#define	PWM_SWR			1<<16

/* functions and interface */
static int pwm_open(struct inode *inode, struct file *filp);
static int pwm_release(struct inode *inode, struct file *filp);
static ssize_t pwm_read(struct file *, char *, size_t, loff_t * l);
static int pwm_ioctl(struct inode *inode,
		     struct file *filp, unsigned int cmd, unsigned long arg);
static ssize_t pwm_write(struct file *filp,
			 const char *buf, size_t count, loff_t * f_pos);
static int pwm_fasync(int fd, struct file *filp, int mode);

static void pwm_timer_func(unsigned long dummy);

static void pwm_stop(void);

#endif				/*PWM_H */
