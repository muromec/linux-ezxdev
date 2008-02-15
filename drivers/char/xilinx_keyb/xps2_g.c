/* $Id: xps2_g.c,v 1.3 2002/07/17 20:58:58 ryserp Exp $ */
/*****************************************************************************
*
*     Author: Xilinx, Inc.
*     
*     
*     This program is free software; you can redistribute it and/or modify it
*     under the terms of the GNU General Public License as published by the
*     Free Software Foundation; either version 2 of the License, or (at your
*     option) any later version.
*     
*     
*     XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
*     COURTESY TO YOU. BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
*     ONE POSSIBLE IMPLEMENTATION OF THIS FEATURE, APPLICATION OR STANDARD,
*     XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION IS FREE
*     FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE FOR
*     OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
*     XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
*     THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO ANY
*     WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE FROM
*     CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND
*     FITNESS FOR A PARTICULAR PURPOSE.
*     
*     
*     Xilinx products are not intended for use in life support appliances,
*     devices, or systems. Use in such applications is expressly prohibited.
*     
*     
*     (c) Copyright 2002 Xilinx Inc.
*     All rights reserved.
*     
*     
*     You should have received a copy of the GNU General Public License along
*     with this program; if not, write to the Free Software Foundation, Inc.,
*     675 Mass Ave, Cambridge, MA 02139, USA.
*
*****************************************************************************/
/****************************************************************************/
/**
*
* @file xps2_g.c
*
* This file contains a configuration table that specifies the configuration of
* PS/2 devices in the system.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a ch   06/18/02 First release
* </pre>
*
******************************************************************************/
/***************************** Include Files ********************************/

#include "xps2.h"
#include "xparameters.h"

/************************** Constant Definitions ****************************/

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/

/*
 * The configuration table for PS/2 devices in the table. Each
 * device should have an entry in this table.
 */
XPs2_Config XPs2_ConfigTable[] = {
	{
	 XPAR_PS2_0_DEVICE_ID,
	 XPAR_PS2_0_BASEADDR}
	,

	{
	 XPAR_PS2_1_DEVICE_ID,
	 XPAR_PS2_1_BASEADDR}
};

/************************** Function Prototypes *****************************/
