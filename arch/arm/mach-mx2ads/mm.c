/*
 *  linux/arch/arm/mach-MX2ads/mm.c
 *
 *  Copyright (C) 1999,2000 Arm Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright (C) 2002 Motorola Semiconductors HK Ltd
 *    - add MX2ADS specific definitions
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
#include <linux/config.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/init.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>

#include <asm/mach/map.h>

static struct map_desc mx2ads_io_desc[] __initdata = {
	{MX2ADS_PER_IOBASE, MX2ADS_PER_BASE, MX2ADS_PER_SIZE, DOMAIN_IO, 0, 1, 0, 0},
	{MX2_IO_IOBASE, MX2_IO_BASE, MX2_IO_SIZE, DOMAIN_IO, 0, 1, 0, 0},
	{MX2_CSI_IOBASE, MX2_CSI_BASE, MX2_CSI_SIZE, DOMAIN_IO, 0, 1, 0, 0},
	{MX2_PCMCIA_IO_IOBASE, MX2_PCMCIA_IO_BASE, MX2_PCMCIA_IO_SIZE, DOMAIN_IO, 0, 1, 0, 0},
	{MX2_NFC_IOBASE, MX2_NFC_BASE, MX2_NFC_SIZE, DOMAIN_IO, 0, 1, 0, 0},
	{MX2_PCMCIA_IOBASE, MX2_PCMCIA_BASE, MX2_PCMCIA_SIZE, DOMAIN_IO, 0, 1, 0, 0},
	LAST_DESC
};

void __init
mx2ads_map_io(void)
{
	iotable_init(mx2ads_io_desc);
}
