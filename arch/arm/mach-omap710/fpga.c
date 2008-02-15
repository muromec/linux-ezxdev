/*
 *  linux/arch/arm/mach-omap710/fpga.c
 *
 * BRIEF MODULE DESCRIPTION
 *   OMAP710 fpga supprt
 *
 * Copyright (C) 2001 RidgeRun, Inc.
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

// REVISIT: gdl

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/hardware.h>

inline void
fpga_write(unsigned char val, int reg)
{
	outb(val, reg);
}

inline unsigned long
fpga_read(int reg)
{
	return inb(reg);
}

inline void
epld_write(int reg, unsigned long val)
{
	outb(val, EPLD_BASE + 0xffe + reg);
}

inline unsigned long
epld_read(int reg)
{
	return inb(EPLD_BASE + 0xffe + reg);
}

int
fpga_fpga_rev(void)
{
	int rev = fpga_read(FPGA_REV_LOW) + (fpga_read(FPGA_REV_HIGH) << 8);
	return rev;
}

#if 0				// 710 doesn't have a board rev field.
int
fpga_board_rev(void)
{
	return fpga_read(FPGA_BOARD_REV);
}
#endif
void
fpga_init(void)
{

	return;			// These numbers are OK for 1510, but wrong for 710.
#if 0
	printk("FPGA REV %d BOARD REV %d\n", fpga_fpga_rev(), fpga_board_rev());
	fpga_write(0x7, 0x2);
	fpga_write(0x62, 0x5);
	fpga_write(0xc, 0xc);

	//fpga_write(epld_write(0xff,0x0);
	epld_write(0x1, 0xff);
	epld_write(0x0, 0xff);
#endif
}

/* Read the EMIFS_CONFIG register to determine if we booted from CS0
 * (internal boot ROM) or from CS3 (external flash bank).  The memory map
 * is different depending on the boot device.  The boot device is assigned
 * address 0x00000000, and the other chip select is assigned address
 * 0x0C000000.
 *
 * Returns 1 if the boot device is CS0 (internal ROM), 0 otherwise.
 */
int
omap_boot_from_cs0(void)
{
	volatile u32 * v = (volatile u32 *)(OMAP_EMIFS_CONFIG_REG);

	return ((*v & OMAP_EMIFS_CONFIG_BM) == 0);
}

/* Provide a generic routine to manipulate the Write Protect (WP) signal for
 * EMIFS flash banks.  Only one WP signal is provided for all of the flash
 * banks, so we need a common routine to control it.
 *
 */
void
omap_set_vpp(int vpp)
{
	volatile u32 * v = (volatile u32 *)(OMAP_EMIFS_CONFIG_REG);
	static spinlock_t vpp_spin = SPIN_LOCK_UNLOCKED;
	static int vpp_counter;

	spin_lock_irq(&vpp_spin);

	if (vpp) {
		vpp_counter += 1;
		*v |= OMAP_EMIFS_CONFIG_WP;
	} else {
		if (vpp_counter > 0)
			vpp_counter -= 1;

		if (vpp_counter == 0)
			*v &= ~OMAP_EMIFS_CONFIG_WP;
	}

	spin_unlock_irq(&vpp_spin);
}


EXPORT_SYMBOL(omap_set_vpp);
EXPORT_SYMBOL(omap_boot_from_cs0);
EXPORT_SYMBOL(fpga_read);
EXPORT_SYMBOL(fpga_write);

  /*---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
