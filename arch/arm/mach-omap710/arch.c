/*
 *  linux/arch/arm/mach-omap710/arch.c
 *
 * BRIEF MODULE DESCRIPTION
 *   sets up the mach_type definition
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/io.h>

extern void omap710_map_io(void);

void
board_init(void)
{
	// Changing Perseus IO pins to LCD
	// See section 4.4 of SWPU012A_OMAP710TRM_bmrk.pdf for details.
//	outw(0, PERSEUS_IO_CONF6);
//	outw(0, PERSEUS_IO_CONF5);
//	outw((inw(PERSEUS_IO_CONF2) & ~0x3C), PERSEUS_IO_CONF2);

	// Changing Perseus IO pins from MPU_EXT_NIRQ to GPIO72
//	outw((inw(PERSEUS_IO_CONF7) | 0xC000), PERSEUS_IO_CONF7);

	// Force a write to the Ethernet chip to enable access verify in 68k mode.
//	outw( 0x0000, FPGA_ETHR_BASE + 0xC);

	/*
	   Set TIPB to buffer writes to all 4 TIPB MPU strobes.
	 */
//	outw(0x3, MPU_PUBLIC_TIPB_CNTL_REG);
//	outw(0x3, MPU_PRIVATE_TIPB_CNTL_REG);

	init_ck();
}

void __init
omap710_fixup(struct machine_desc *desc, struct param_struct *params,
	      char **cmdline, struct meminfo *mi)
{
}

extern void irq_init_irq(void);

MACHINE_START(TI925, "ARM-OMAP710")
	MAINTAINER("MontaVista Software, Inc.")
	BOOT_MEM(0x10000000, 0xe0000000, 0xe0000000)
	FIXUP(omap710_fixup)
	MAPIO(omap710_map_io)
	INITIRQ(irq_init_irq)
MACHINE_END
