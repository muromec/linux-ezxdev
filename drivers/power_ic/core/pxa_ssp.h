/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/core/pxa_ssp.h
 * 
 * Description - header file for the low level SPI driver for Intel Bulverde.
 *
 * Copyright 2005 Motorola, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published 
 * by the Free Software Foundation.
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
 * 2005-Jun-16 - Design of low level SPI interface for Intel Bulverde.
 */


#ifndef __PXA_SSP_H
#define __PXA_SSP_H

/*!
 * @ingroup spi_management
 */

extern void pxa_ssp_initialize(void);
extern bool pxa_ssp_tx_in_progress(void);
extern int pxa_ssp_start_tx(SPI_QUEUE_NODE_T *node_p);
                           
#endif /* __PXA_SSP_H */
