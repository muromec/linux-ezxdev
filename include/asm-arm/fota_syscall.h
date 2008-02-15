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
 *
 *  linux/arch/arm/mach-ezx/fota_syscall.h
 *
 *  Support for the Motorola Ezx Development Platform for Barbados product.
 *
 *  Author:     Jordan Wang
 *  		    Motorola BJDC 
 *  Created:    Jan 31, 2005
 *
 */
#ifndef __ASM_SYSCALL_H__
#define __ASM_SYSCALL_H__

#include <linux/config.h>

#include <linux/errno.h>
#include <linux/types.h>
#include <linux/unistd.h>

#define __NR_sys_fota_req (__NR_SYSCALL_BASE+ 226)

#endif

