/*
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
 * Copyright 2003 Motorola, Inc. All Rights Reserved.
 *
 *	 1wire.h
 *	 1wire driver header file for Mx21.
*/

#ifndef _FS_1WIRE_INC_H__
#define _FS_1WIRE_INC_H__

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>

#define MODULE_NAME "Owire"
//#define DBMX_DEBUG 1
#ifdef DBMX_DEBUG
#define TRACE(fmt, args...) \
	{ \
		pr_debug("\n %s:%d:%s:",__FILE__, __LINE__,__FUNCTION__); \
		pr_debug(fmt, ## args);\
	}
#else
#define TRACE(fmt, args...)
#endif

static devfs_handle_t g_devfs_handle;
int g_Owire_major = 0;

struct pm_dev *g_Owire_pm;

struct apmc_user *g_Owire_apmc;

static int g_Owire_status;
#define OWIRE_OPEN_STATUS		0x0001
#define OWIRE_SUSPEND_STATUS	0x0002

#endif
