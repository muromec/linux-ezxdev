/*
 * arch/arm/mach-pxa/mainstone-dpm.c
 * 
 * Mainstone-specific DPM support
 *
 * Author: <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 * 2005-May-11  copy from arch/arm/mach-pxa/mainstone_dpm.c and porting to EZX platform,  Zhuang Xiaofan
 * 2005-Nov-11  add Montavista 3.1 patch,  Zhuang Xiaofan
 *
 */

#include <linux/config.h>
#include <linux/dpm.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <asm/hardirq.h>
#include <asm/page.h>
#include <asm/uaccess.h>

#include "bulverde_voltage.h"

/****************************************************************************
 * Initialization/Exit
 ****************************************************************************/

int dpm_mainstone_bd_init(void)
{
	return 0;
}

void dpm_mainstone_bd_exit(void)
{
	return;
}
void dpm_bulverde_board_setup(void)
{
	dpm_bd.init = dpm_mainstone_bd_init;
	dpm_bd.exit = dpm_mainstone_bd_exit;
	dpm_bd.set_v_pre = 0;
	dpm_bd.set_v_post = 0;
}
