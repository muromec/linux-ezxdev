/*
 *  linux/arch/arm/mach-omap710/mm.c
 *
 * BRIEF MODULE DESCRIPTION
 *   maps in the hardware correctly
 *
 * Copyright (C) 2000 RidgeRun, Inc. (http://www.ridgerun.com)
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/init.h>
#include <linux/config.h>

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/arch/sizes.h>
#include <asm/mach/map.h>
#include <asm/arch/hardware.h>

/*
 * Map the various IO regions onto 0xExxx.xxxx.  For
 * example, SC_BASE (0x1100.0000) gets mapped
 * to 0xF110.0000
 */

#if	0
#define MAP_DESC(base,start,size) { base,start,size,DOMAIN_IO,0,1,0,0 }
#endif

static struct map_desc io_desc[] __initdata = {
#if	1
/* virtual		physical		length			domain		r  w  c  b */
	{IO_BASE, IO_START, IO_SIZE, DOMAIN_IO, 1, 1, 0, 0},
	{nCS1_BASE, nCS1_START, nCS1_SIZE, DOMAIN_IO, 1, 1, 0, 0},
	{FPGA_BASE, FPGA_START, FPGA_SIZE, DOMAIN_IO, 1, 1, 0, 0},
	{OMAP710_GPIO_BASE, OMAP710_GPIO_START, OMAP710_GPIO_SIZE, DOMAIN_IO, 1,
	 1, 0, 0},
	{EPLD_BASE, EPLD_START, EPLD_SIZE, DOMAIN_IO, 1, 1, 0, 0},
	{OMAP710_SRAM_BASE, OMAP710_SRAM_START, OMAP710_SRAM_SIZE, DOMAIN_IO, 1,
	 1, 0, 0},
#else
	MAP_DESC(IO_BASE, IO_START, IO_SIZE),
	MAP_DESC(nCS1_BASE, nCS1_START, nCS1_SIZE),
	MAP_DESC(FPGA_BASE, FPGA_START, FPGA_SIZE),
	MAP_DESC(OMAP710_GPIO_BASE, OMAP710_GPIO_START, OMAP710_GPIO_SIZE),
	MAP_DESC(EPLD_BASE, EPLD_START, EPLD_SIZE),
	MAP_DESC(OMAP710_SRAM_BASE, OMAP710_SRAM_START, OMAP710_SRAM_SIZE),
#endif
	LAST_DESC
};

extern void board_init(void);
extern void fpga_init(void);

void __init
omap710_map_io(void)
{
	iotable_init(io_desc);
//    board_init();
//    fpga_init();
}
