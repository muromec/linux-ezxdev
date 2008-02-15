#ifndef __POWER_MANAGEMENT_H__
#define __POWER_MANAGEMENT_H__
/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/module/power_management.h
 * 
 * Description - This is the header of internal definitions the power IC power management interface.
 *
 * Copyright (C) 2005 Motorola, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation.
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
 * 2005-Jun-09 - Design of interfaces to control power-related functions.
 */



/*!
 * @file power_management.h
 *
 * @ingroup poweric_power_management
 *
 * @brief This is the header of internal definitions the power IC power management interface.
 *
 */

#include <linux/power_ic.h>

int pmm_ioctl(unsigned int cmd, unsigned int arg);


#endif /* __POWER_MANAGEMENT_H__ */
