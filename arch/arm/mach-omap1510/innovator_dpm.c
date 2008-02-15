/*
 * arch/arm/mach-omap1510/innovator-dpm.c  Innovator-specific DPM support
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (C) 2002 MontaVista Software <source@mvista.com>.
 *
 * Based on arch/ppc/platforms/ibm405lp_dpm.c by Bishop Brock.
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

/****************************************************************************
 * Initialization/Exit
 ****************************************************************************/

int dpm_innovator_bd_init(void)
{
	return 0;
}

void dpm_innovator_bd_exit(void)
{
}

void dpm_omap1510_board_setup(void)
{
	dpm_bd.init = dpm_innovator_bd_init;
	dpm_bd.exit = dpm_innovator_bd_exit;
	dpm_bd.check_v = NULL;
	dpm_bd.set_v_pre = NULL;
	dpm_bd.set_v_post = NULL;
}
