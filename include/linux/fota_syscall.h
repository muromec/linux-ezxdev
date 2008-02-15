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
 */
/*
 *  include/linux/fota_syscall.h
 *
 *  Support for the Motorola Ezx Development Platform for Barbados product.
 *
 *  Author:     Jordan Wang
 *  		    Motorola BJDC 
 *  Created:    Jan 31, 2005
 */
#ifndef __FOTA_SYSCALL_H__
#define __FOTA_SYSCALL_H__

#include <asm/fota_syscall.h>
#include <linux/config.h>

typedef enum
{
	FODA_REQ_REBOOT_AP,
	FODA_REQ_REBOOT_BP,
	FODA_REQ_CONNECT_OPT1_OPT2,
	FODA_REQ_DISCONNECT_OPT1_OPT2
}fota_req_t;

extern int sys_fota_req(fota_req_t params);
#if 0
#if defined(__KERNEL__)
#define sys_fota_req(PARAMS) \
	syscall(__NR_sys_fota_req, PARAMS)
#else
extern int sys_fota_req(fota_req_t params);
#endif
#endif
#endif /* FOTA_SYSCALL_H*/
