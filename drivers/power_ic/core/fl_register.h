/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2004 Motorola, Inc. All Rights Reserved.
 */

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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 * Motorola 2004-Dec-06 - DDesign the Low Level interface to Funlight IC.
 */


#ifndef __FL_REGISTER_H__
#define __FL_REGISTER_H__  

/*!
 * @file fl_register.h
 * @brief This file contains prototypes of Funlight driver available in kernel space.
 *
 * @ingroup poweric_core
 */


void fl_initialize(void (*reg_init_fcn)(void));
int fl_reg_read (int reg, unsigned int *reg_value);
int fl_reg_write (int reg, unsigned int reg_value);

#endif  /* __FL_REGISTER_H__ */
