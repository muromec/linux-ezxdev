#ifndef __TOUCHSCREEN_H__
#define __TOUCHSCREEN_H__
/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/module/touchscreen.h
 * 
 * Description - header of internal definitions the power IC touchscreen interface.
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
 * 2005-Apr-27 - Design of the touchscreen module
 */



/*!
 * @file touchscreen.h
 *
 * @ingroup poweric_touchscreen
 *
 * @brief This is the header of internal definitions the power IC touchscreen interface.
 *
 */

int touchscreen_ioctl(unsigned int cmd, unsigned long arg);
void touchscreen_init(void);


#endif /* __TOUCHSCREEN_H__ */
