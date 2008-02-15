/* $Id: xemac_g.c,v 1.4 2002/06/26 20:12:10 linnj Exp $ */
/******************************************************************************
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
******************************************************************************/
/*****************************************************************************/
/**
*
* @file xemac_g.c
*
* This file contains a configuration table that specifies the configuration
* of EMAC devices in the system.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a rpm  07/31/01 First release
* 1.00b rpm  02/20/02 Repartitioned files and functions
* </pre>
*
******************************************************************************/

/***************************** Include Files *********************************/
#include "xemac.h"
#include "xparameters.h"

/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/

/************************** Variable Prototypes ******************************/

/**
 * This table contains configuration information for each EMAC device
 * in the system.
 */
XEmac_Config XEmac_ConfigTable[XPAR_XEMAC_NUM_INSTANCES] = {
	{
	 XPAR_EMAC_0_DEVICE_ID,	/* Unique ID  of device */
	 XPAR_EMAC_0_BASEADDR,	/* Base address of device */
	 XPAR_EMAC_0_ERR_COUNT_EXIST,	/* Does device have counters? */
	 XPAR_EMAC_0_DMA_PRESENT,	/* Does device have scatter-gather DMA? */
	 XPAR_EMAC_0_MII_EXIST	/* Does device support MII? */
	 }
};
