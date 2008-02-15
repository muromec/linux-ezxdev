/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2005-2006 Motorola, Inc. All Rights Reserved.
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
 * Motorola 2005-Feb-21 - Redesign of the register functions for Atlas driver. */

/*!
 * @file atlas_register.c
 *
 * @brief This file contains register function of ATLAS driver. 
 *
 * @ingroup poweric_core
 */

#include "/vobs/jem/hardhat/linux-2.6.x/drivers/mxc/atlas/core/atlas_spi_inter.h"
#include "os_independent.h"

/*!
 * @brief Reads a ATLAS register
 *
 * This function implements a read of a given ATLAS register.  
 *
 * @param reg          ATLAS register number
 * @param value_ptr    Pointer to which the read data should be stored
 *
 * @return 0 if successful
 */

int atlas_reg_read (int reg, unsigned int *value_ptr)
{ 
    return (spi_read_reg (reg, value_ptr));
}

/*!
 * @brief Writes a ATLAS register
 *
 * This function implements a write to a given ATLAS register.  
 *
 * @param reg          ATLAS register number
 * @param value        Value to write to the register
 *
 * @return 0 if successful
 */

int atlas_reg_write (int reg, unsigned int value)
{
    return (spi_write_reg (reg, &value));
}
