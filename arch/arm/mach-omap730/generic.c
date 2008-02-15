/*
 * linux/arch/arm/mach-omap730/generic.c
 *
 * Code common to all OMAP machines.
 *
 * Modifications for the OMAP730 :
 * Copyright (c) 2004, MPC-Data Limited (http://www.mpc-data.co.uk)
 * Dave Peverley <dpeverley@mpc-data.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <asm/hardware.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>
#include <asm/arch/ck.h>
#include <asm/io.h>
#include <asm/arch/omap730_config.h>


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


/*
 * Common OMAP I/O mapping
 *
 * The machine specific code may provide the extra mapping besides the
 * default mapping provided here.
 */

#define MAP_DESC(base,start,size,domain,r,w,c,b) { base,start,size,domain,r,w,c,b }
static struct map_desc standard_io_desc[] __initdata = {
	MAP_DESC(IO_BASE,
		 IO_START,
		 IO_SIZE,
		 DOMAIN_IO,
		 0,1,0,0),
	MAP_DESC(OMAP730_SRAM_BASE,
		 OMAP730_SRAM_START,
		 OMAP730_SRAM_SIZE,
		 DOMAIN_IO,
		 0,1,0,0),
	LAST_DESC
};

void __init omap730_map_io(void)
{
	iotable_init(standard_io_desc);

	/* Hold GSM Reset until needed. */
	*((volatile __u16 *)M_CTL) &= ~1;

	/* Must init clocks early to assure that timer interrupt works
	 */
	init_ck();
}

EXPORT_SYMBOL(omap_set_vpp);
EXPORT_SYMBOL(omap_boot_from_cs0);
