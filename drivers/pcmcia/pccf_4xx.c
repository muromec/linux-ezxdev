/* Device driver for the IBM 4xx PCMCIA/CF+ Interface Macro (PCCF)
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
 * Copyright (C) 2001, International Business Machines Corporation
 * All Rights Reserved.
 * Copyright (C) 2002 MontaVista Software, Inc. <source@mvista.com>
 *
 *  Eric Van Hensbegren <bergevan@us.ibm.com>
 *  IBM Austin Research Lab
 *
 *  Based on original code by Magnus Damm for mpc8xx series driver
 *
 *  May, 2001     - Original version for Pecan 
 *  August, 2001  - Modified by Benjamin Li (IBM Austin) for 405LP
 *  October, 2001 - Modified/Documented by Bishop Brock (IBM Research Austin)
 *                  bcbrock@us.ibm.com
 *  August, 2002  - Conversion to standalone in-kernel driver by Todd
 *                  Poynor (MontaVista Software)
 ****************************************************************************

 Module Parameters
 =================

 pwr5by3=[0|1]

   0 - [ Default ] Cards are powered by the Vcc voltage they require, either
       3.3 V or 5 V.  If the power rail requested is not available, the card
       insertion will fail.

   1 - Cards that request 5 V will be powered by 3.3 V instead.  See below for
       further details.

 ignore_wrprot=[0|1]

   0 - [ Default ] Drivers that request write-protected regions will fail
       unless the configuration can actually write protect the region.  

   1 - If this module can satisfy a request to write protect a region it will,
       otherwise it will simply print a warning and carry on.

  pc_debug=<n>

   A standard parameter for PCMCIA modules.  If modules are compiled with
   -DPCMCIA_DEBUG=<m>, then debugging prints are enabled and <m> is the default
   debugging level.  The level can be changed by the pc_debug=<n> parameter
   when the module is loaded. Debug level 0 suppresses all messages.

 Introduction
 ============

 The 4xx PCCF (PCMCIA/CF+) macro is an "internal" EBC peripheral that sits
 between the EBC controller and the chip IOs.  When enabled, PCCF takes control
 of 4 EBC chip selects and other EBC signals, and does a straightforward
 mapping of EBC transactions to PCMCIA/CF+ transactions. The type of PCMCIA/CF+
 transaction (Attribute Memory, Common Memory, I/O, or Macro Control) is
 determined by a simple address decode of the high-order bits of the physical
 address.  PCCF was first implemented as an external EBC peripheral in an FPGA
 on the IBM Austin Research Pecan board.

 PCCF is not compatible with the ExCA standard.  PCCF supports only a single
 memory window, and this driver does not attempt to partition the single
 physical window into multiple virtual windows.  Although the PCCF macro
 supports only a single IO area as well, this driver gives the illusion that 2
 IO windows exist.  This is possible because IO windows don't involve address
 translation. On the other hand, the implementation can not support different
 IO window-specific parameters on the two IO windows.

 As implemented in the 405LP, PCCF is not strictly compatible with the PCMCIA
 specification, as it only provides 23 of the 26 address bits called for in the
 specification.  Thus 405LP/PCCF may not support some archaic PCMCIA SRAM
 cards, but will support modern PCMCIA cards such as IDE disks and network
 adapters, and fully support CF+ cards (but see discussion on power below).

 PCCF supports a single PCMCIA/CF slot.

 EBC Setup and Memory Map
 =========================

 The EBC interface to PCCF can be configured in numerous ways.  Also, the
 ability of PCCF to generate valid PCMCIA/CF transactions critically depends on
 the proper setup of the access parameters (EBC0_BnAP) register(s) for the EBC
 bank(s) controlling PCCF.  EBC setup is currently handled by this driver for
 all supported configurations.  This driver maps the PCCF macro in a single EBC
 bank at a configuration-dependent address, and the access parameters provide a
 minimum 300 ns cycle time for all memory and IO accesses.  The EBC access
 parameters *must* include sample-on-ready to allow slow devices to control the
 timing of their transactions, and the EBC0_CFG register will specify at least
 a 12 us timeout (for PCMCIA specification compatibility).
 In order to guarantee correct operation for all cards, the board
 design must correctly bring in the PCMCIA WAIT# signal.

 The PCCF macro maps a 32 MB physical address space as indicated below.
 Offsets are relative to the physical base address.

 0x00000000 - 0x007fffff : 8 MB PCCF control register space
 0x00800000 - 0x00ffffff : 8 MB Attribute memory space
 0x01000000 - 0x017fffff : 8 MB Common memory space
 0x01800000 - 0x018fffff : 8 MB I/O space

 Endianess
 =========

 PCMCIA is a little-endian specification.  The PCCF controller byte-swaps all
 16-bit accesses.  This means that 16-bit accesses will need to use
 byte-swapping IO, as normally occurs in 4xx kernels.  The PCCF control
 registers are big-endian, however.  That's why the accesses of those registers
 use in_be16 and out_be16, which are non-swapping operations.

 A note on IDE: IDE control and status operations are 8-bit operations, so
 endianess is not an issue.  IDE sector reads/writes do not use swapping IO on
 PowerPC, *except* for the IDE identification sector, which is swapped so that
 it can be interpreted.  This allows removable drives to be created on e.g., an
 IA32 laptop and read with this controller and vice versa.

 Legacy IO and Address Mapping
 =============================

 Requests for memory windows are given a physical address, which the caller
 will (redundantly) ioremap().  Requests for I/O space windows are not
 modified.  For legacy IO to work, the _IO_BASE symbols is set to reference
 isa_io_base, which holds the virtual address of our IO area mapping.

 Power Considerations
 ====================

 Vcc
 ---

 The PCCF macro control register includes bits for both 3.3 V and 5 V, Vcc
 control. On the 405LP, however, only the 3.3 V control is brought out of the
 chip, as 405LP I/O are not 5 V compliant. 

 To reduce I/O count the 405LP also does not connect the VS2# signal.  VS2# is
 pulled high internally to the PCCF macro.

 A 405LP-based system designed for 5 V PCMCIA support will require external
 voltage-conversion buffers and an alternate means for selecting the 5 V Vcc
 level, e.g., a GPIO pin or CPLD (as on Beech) . Similarly, a 405LP-based
 system providing X.X V operation will need a way to signal VS2#.

 The Pecan FPGA implementation of PCCF supports both 3.3 V and 5 V operation,
 and both VS1# and VS2#.

 If the current configuration does not support 5V operation, then inserting a 5
 V card into the slot will fail. Vcc = 3.3 V is always assumed to be present.
 Some "5 V" PCMCIA cards will operate correctly at Vcc = 3.3 V, however.  To
 support this mode of operation, the driver can be loaded with the parameter

 pwr5by3=1

 which will cause all 5 V cards to be powered by 3.3 V (even if the
 configuration normally supports 5 V operation).  Beware that this may cause
 equipment damage (unlikely on a well-designed board) or loss of data (perhaps
 more likely).

 Vpp
 ---

 PCCF does not directly support Vpp power selection.  The default assumption is
 that the external power supply is configured according to the PCMCIA standard,
 and will supply Vpp = Vcc when power is applied to the socket.  If a board is
 constructed such that Vpp power selection is possible, the routine
 set_Vpp() et. al. will need to be changed accordingly.

 Limitations
 ===========

 The controller does not generate interrupts for memory-card status change
 events such as battery voltage changes or changes on the READY/BUSY line.

 The macro and card IRQ lines are fixed; The controller can not route the card
 interrupt to another IRQ.  This may have implications for legacy device
 drivers.

 The PCCF macro includes a single control register for all of the memory and IO
 registers.  It is not possible to set different access parameters (i.e.,
 8-bit, 16-bit, auto-size) for different areas.  The access parameters are
 reset every time a new memory or I/O window is mapped.  This may cause some
 device drivers to fail when using this controller.

 Furthermore, this module assumes that the highest-level device driver knows
 whether it is accessing 8- or 16-bit IO registers, and it is up to the driver
 to do inb(), inw() etc. as appropriate for the IO area being accessed.  The
 PCCF macro can not split 16-bit bus transactions into a pair of 8-bit
 transactions as some other controllers can.

 There is currently no support for optimizing memory and IO access speeds.  [
 It's not clear this is ever used; In any event, don't do this until we have
 our variable-speed mechanism in place. ]

 There is currently no support for write-protecting windows.  This can be added
 but will require BIOS-like intervention to reprogram the EBC banks as
 protection comes on and off line.  Single EBC-bank configurations like Pecan
 will be difficult and painful to write-protect correctly.

****************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/string.h>

#include <asm/io.h>
#include <asm/bitops.h>
#include <asm/segment.h>
#include <asm/system.h>
#include <asm/irq.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/kmod.h>

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/ss.h>
#include <pcmcia/bus_ops.h>

#include <asm/byteorder.h>
#include <asm/pccf_4xx.h>

MODULE_AUTHOR("Eric Van Hensbergen <bergevan@us.ibm.com>");
MODULE_DESCRIPTION("IBM PCCF 4xx socket driver");
MODULE_PARM(pwr5by3, "i");
MODULE_PARM_DESC(pwr5by3, "Experimental: Power 5 V cards with 3.3 V");
MODULE_PARM(ignore_wrprot, "i");
MODULE_PARM_DESC(ignore_wrprot, "Experimental: Ignore write protect requests");

static int pwr5by3 = 0;
static int ignore_wrprot = 0;

static const char *version =
    "pccf_4xx.c $Revision: 1.3 $ $Date: 2001/10/26 02:22:41 $ (Eric Van Hensbergen)";

#define PCMCIA_DEBUG 0

#ifdef PCMCIA_DEBUG
static int pc_debug = PCMCIA_DEBUG;
MODULE_PARM(pc_debug, "i");
MODULE_PARM_DESC(pc_debug,
		 "Set pc_debug to a non-0 value to enable debug msgs.");
#define DEBUG(n, args...) if (pc_debug>(n)) printk(KERN_DEBUG args)
#else
#define DEBUG(n, args...)
#endif

/* Configurations */

#define VCC5V_SUPPORT 1

/* Minimum bus cycle time (in ns) */

#define MIN_SPEED 300

#define PCMCIA_ATTRIB_OFFSET	0x00800000
#define PCMCIA_ATTRIB_PADDR	(PCCF_4XX_PADDR + PCMCIA_ATTRIB_OFFSET)

#define PCMCIA_MEM_WIN_NO 		1
#define PCMCIA_IO_WIN_NO 		2

#define PCMCIA_CTRL(offset)\
 ((volatile unsigned short *)((u8 *) pccf_4xx_macro_vaddr + (offset)))

/* PCCF control registers are big-endian */

#define GET_PCMCIA_CTRL(offset)        (in_be16(PCMCIA_CTRL(offset)))
#define SET_PCMCIA_CTRL(value, offset) (out_be16(PCMCIA_CTRL(offset), value))

#define PCMCIA_CSR GET_PCMCIA_CTRL(0x00)	/* Card Status Register */
#define PCMCIA_CCR GET_PCMCIA_CTRL(0x02)	/* Card Control Register */
#define PCMCIA_ISR GET_PCMCIA_CTRL(0x04)	/* Interrupt Status Register */
#define PCMCIA_ICR GET_PCMCIA_CTRL(0x06)	/* Interrupt Control Register */
#define PCMCIA_MRR GET_PCMCIA_CTRL(0x0e)	/* Macro Reset Register */

#define SET_PCMCIA_CSR(value) 		SET_PCMCIA_CTRL(value, 0x00)
#define SET_PCMCIA_CCR(value) 		SET_PCMCIA_CTRL(value, 0x02)
#define SET_PCMCIA_ISR(value) 		SET_PCMCIA_CTRL(value, 0x04)
#define SET_PCMCIA_ICR(value) 		SET_PCMCIA_CTRL(value, 0x06)
#define SET_PCMCIA_MRR(value) 		SET_PCMCIA_CTRL(value, 0x0e)

#ifndef get_field16
#define get_field16(reg, hi, lo)\
((reg >> (15 - (lo))) & ~(-1 << ((lo) - (hi) + 1)))

#define make_field16(val, hi, lo)\
(((val) & ~(-1 << ((lo) - (hi) + 1))) << (15 - (lo)))
#endif				/* get_field16 */

#ifndef get_field32
#define get_field32(reg, hi, lo)\
((reg >> (31 - (lo))) & ~(-1 << ((lo) - (hi) + 1)))

#define make_field32(val, hi, lo)\
(((val) & ~(-1 << ((lo) - (hi) + 1))) << (31 - (lo)))

#define set_field32(word, hi, lo, val)\
(((word) & ~(make_field32(0xffffffff, hi, lo))) | make_field32(val, hi, lo))
#endif				/* get_field32 */

#define PCMCIA_CSR_DETECT(csr)		(get_field16(csr, 2, 2))
#define PCMCIA_CSR_VS1(csr)		(get_field16(csr, 5, 5))
#define PCMCIA_CSR_VS2(csr)		(get_field16(csr, 4, 4))
#define PCMCIA_CSR_IO(csr)		(get_field16(csr, 8, 8))
#define PCMCIA_CSR_PWR(csr)		(get_field16(csr, 14, 15))
#define PCMCIA_CSR_READY(csr)		(get_field16(csr, 12, 12)==0)

#define PCMCIA_CCR_MODE(ccr)   		get_field16(ccr, 0, 1)
#define PCMCIA_CCR_3V(ccr)     		get_field16(ccr, 6, 6)
#define PCMCIA_CCR_5V(ccr)     		get_field16(ccr, 7, 7)
#define PCMCIA_CCR_816(ccr)    		get_field16(ccr, 12, 12)
#define PCMCIA_CCR_IOIS16(ccr) 		get_field16(ccr, 13, 13)

#define PCMCIA_ISR_STSCHG(isr) 		get_field16(isr, 6, 6)
#define PCMCIA_ISR_DETECT(isr) 		get_field16(isr, 7, 7)

#define PCMCIA_ICR_STSCHG(icr) 		get_field16(icr, 4, 4)
#define PCMCIA_ICR_DETECT(icr) 		get_field16(icr, 5, 5)

/*  These are values used to represent voltage levels  */
#define VOLT00                          00
#define VOLT33				33
#define VOLT50				50

/****************************************************************************
  PCCF control routines
****************************************************************************/

/* Pecan and Beech Pass 1 use the PCCF macro for power control (although Beech
   Pass 1 doesn't support +5V.

   Beech Pass 2 uses an FPGA register to control the 5V power supply.  When
   moving between power modes on Beech, we're always careful to transit via a
   state that carefully powers-down the socket (order is important).  The
   ground state on the Beech 2 socket power is PCCF 3V bit == 0, FPGA 5V bit =
   0.  Setting exactly one of these to 1 enables the proper voltage on the
   socket. */

static void
pcmcia_0v(void)
{				/* Remove power */
	unsigned short ccr = PCMCIA_CCR;
	unsigned long flags;

	save_and_cli(flags);
#if defined(CONFIG_BEECH)
	writeb(readb(beech_fpga_reg_4) & ~FPGA_REG_4_PCMCIA_5V,
	       beech_fpga_reg_4);
#elif defined(CONFIG_ARCTIC2)
	writeb(0, ARCTIC2_FPGA_PCCF_POWER);
#endif
	restore_flags(flags);

	ccr &= ~0x0200;	/* -+3V */
	ccr &= ~0x0100;	/* -+5V */
	SET_PCMCIA_CCR(ccr);
}

static void
pcmcia_3v(void)
{				/* Supply 3 V */
	unsigned short ccr = PCMCIA_CCR;

	pcmcia_0v();

	ccr |= 0x0200;		/* ++3V */
	ccr &= ~0x0100;		/* -+5V */
	SET_PCMCIA_CCR(ccr);
}

static void
pcmcia_5v(void)
{				/* Supply 5 V */
	unsigned short ccr = PCMCIA_CCR;
	unsigned long flags;

	pcmcia_0v();

	ccr &= ~0x0200;		/* -+3V */
	ccr |= 0x0100;		/* ++5V */
	SET_PCMCIA_CCR(ccr);

	save_and_cli(flags);
#if defined(CONFIG_BEECH)
	writeb(readb(beech_fpga_reg_4) | FPGA_REG_4_PCMCIA_5V,
	       beech_fpga_reg_4);
#elif defined(CONFIG_ARCTIC2)
	writeb(ARCTIC2_FPGA_PCCF_POWER_5V, ARCTIC2_FPGA_PCCF_POWER);
#endif
	restore_flags(flags);
}

static void
pcmcia_reset(void)
{				/* Assert card reset for 10 us */
	unsigned short ccr = PCMCIA_CCR;

	ccr |= 0x0080;		/* Reset */
	SET_PCMCIA_CCR(ccr);
	udelay(10);
	ccr = PCMCIA_CCR;
	ccr &= ~0x0080;
	SET_PCMCIA_CCR(ccr);
}

static void
pcmcia_mrr(void)
{				/* Reset the PCCF macro */
	SET_PCMCIA_MRR(0xffff);
}

static void
pcmcia_mem(void)
{				/* Set memory mode */
	unsigned short ccr = PCMCIA_CCR;

	ccr |= 0xc000;
	SET_PCMCIA_CCR(ccr);
}

static void
pcmcia_io(void)
{				/* Set memory and IO mode */
	unsigned short ccr = PCMCIA_CCR;

	ccr |= 0x4000;
	ccr &= ~0x8000;
	SET_PCMCIA_CCR(ccr);
}

static void
pcmcia_clr_cdi(void)
{				/* Clear the card-detect interrupt */
	unsigned short icr = PCMCIA_ICR;

	icr |= 0x0100;
	SET_PCMCIA_ICR(icr);
	icr = PCMCIA_ICR;
	icr &= ~0x0100;
	SET_PCMCIA_ICR(icr);
}

static void
pcmcia_unmask_cdi(void)
{				/* Unmask the card-detect interrupt */
	unsigned short icr = PCMCIA_ICR;

	icr &= ~0x0400;
	SET_PCMCIA_ICR(icr);
}

static void
pcmcia_unmask_sti(void)
{				/* Unmask the status-change interrupt */
	unsigned short icr = PCMCIA_ICR;

	icr &= ~0x0800;
	SET_PCMCIA_ICR(icr);
}

static void
pcmcia_mask_cdi(void)
{				/* Mask the card-detect interrupt */
	unsigned short icr = PCMCIA_ICR;

	icr |= 0x0400;
	SET_PCMCIA_ICR(icr);
}

static void
pcmcia_mask_sti(void)
{				/* Mask the status-change interrupt */
	unsigned short icr = PCMCIA_ICR;

	icr |= 0x0800;
	SET_PCMCIA_ICR(icr);
}

static void
pcmcia_16bit(void)
{				/* Set 16-bit mode */
	unsigned short ccr = PCMCIA_CCR;

	ccr &= ~0x000c;
	SET_PCMCIA_CCR(ccr);
}

static void
pcmcia_autosz(void)
{				/* Set IOIS16# mode */
	unsigned short ccr = PCMCIA_CCR;

	ccr &= ~0x000c;
	ccr |= 0x0004;
	SET_PCMCIA_CCR(ccr);
}

static void
pcmcia_8bit(void)
{				/* Set 8-bit mode */
	unsigned short ccr = PCMCIA_CCR;

	ccr &= ~0x000c;
	ccr |= 0x0008;
	SET_PCMCIA_CCR(ccr);
}

/* This structure records the macro interrupt number and its handler and
   private data, a saved copy of the socket state, and memory and IO window
   structures. */

typedef struct socket_info_t {
	u_char intr;
	void (*handler) (void *info, u_int events);
	void *info;
	socket_state_t state;
	struct pccard_mem_map mem_win[PCMCIA_MEM_WIN_NO];
	struct pccard_io_map io_win[PCMCIA_IO_WIN_NO];
} socket_info_t;

/* Our socket(s) support(s) PCMCIA/CF+ cards (not cardbus) with static memory
   maps, no ISA IRQs, and a single fixed socket interrupt. 

   The PCCF controller does not place any restrictions on memory window sizes.
   However, the card services driver seems to use the map_size a lot during
   numerous ioremap()ing and iounmap()ing operations.  To be safe we are
   setting the map_size parameter to PAGE_SIZE, which is (similar to) what
   other drivers do.

*/

static socket_cap_t capabilities = {
	SS_CAP_PCCARD | SS_CAP_STATIC_MAP,	/* PCMCIA, static map */
	0,			/* No ISA interrupts */
	PAGE_SIZE,		/* Window size granularity */
	0,			/* io_offset */
	PCCF_4XX_CARD_IRQ,		/* Single fixed socket interrupt */
	NULL,			/* No CardBus support */
	NULL			/* No virtual bus operations */
};

/* Socket parameters and data */

#define TOTAL_NUM_OF_SOCKETS		1
static int sockets = TOTAL_NUM_OF_SOCKETS;
static socket_info_t socket[TOTAL_NUM_OF_SOCKETS];

/****************************************************************************
  Configuration-specific code
****************************************************************************/

/* Voltage control.  See comments at the beginning of the file.

   The 'poweroff' situation is called out separately because we also use the
   occasion of a powerdown to reset some state.  This is the default state of
   the socket - status change interrupt masked, the card in memory mode and
   16-bit mode.

   These routines will probably be slightly different for each
   chip/board. set_voltage() is called with the requested Vcc and Vpp values,
   which may be modified. In the event of an error, power is removed from the
   socket. */

static int
set_Vcc(int *Vcc)
{
	switch (*Vcc) {
	case VOLT00:
	case VOLT33:
		break;
	case VOLT50:
		if (pwr5by3) {
			printk(KERN_WARNING "pccf_4xx: Warning: Powering 5 V "
			       "card with 3.3 V.  Good Luck!\n");
			*Vcc = VOLT33;
		} else if (!VCC5V_SUPPORT) {
			printk(KERN_ERR "pccf_4xx: The card requested "
			       "Vcc = 5 V, which is not supported by this\n"
			       "platform.  You can try reloading the module "
			       " with the parameter pwr5by3=1,\n"
			       "which will cause all 5 V cards to be powered "
			       " by 3.3 V.  Beware that this may cause\n"
			       "equipment damage or loss of data.\n");
			return -EINVAL;
		}
		break;
	default:
		printk(KERN_ERR "pccf_4xx: Illegal Vcc voltage = %d\n", *Vcc);
		return -EINVAL;
		break;
	}

	switch (*Vcc) {
	case VOLT00:
		pcmcia_0v();
		break;
	case VOLT33:
		pcmcia_3v();
		break;
	default:
		pcmcia_5v();
		break;
	}
	return 0;
}

/* We assume that if Vpp request is <= Vcc, then Vcc = Vpp is OK.  If we're in
   pwr5by3 mode then Vcc 3.3, Vpp = 5 is OK, too. The thing that can't be
   supported (on PECAN) are old memory cards that require 12 V for flash
   update.  */

static int
set_Vpp(int *Vcc, int *Vpp)
{
	if (*Vcc != *Vpp) {
		if (*Vpp > *Vcc) {
			if (!(pwr5by3 && (*Vcc = VOLT33) & (*Vpp = VOLT50))) {
				printk(KERN_ERR "pccf_4xx: Can't supply"
				       " Vpp = %d with Vcc = %d\n", *Vpp, *Vcc);
				return -EINVAL;
			}
		}
		DEBUG(0, "pccf_4xx: Setting Vpp = Vcc = "
		      "%d.%d\n", *Vcc / 10, *Vcc % 10);
		*Vpp = *Vcc;
	}
	return 0;
}

static void
poweroff(void)
{
	int Vcc = 0, Vpp = 0;

	set_Vcc(&Vcc);
	set_Vpp(&Vcc, &Vpp);
	pcmcia_mask_sti();
	pcmcia_mem();
	pcmcia_16bit();
}

static int
set_voltage(int *Vcc, int *Vpp)
{
	if ((*Vcc == 0) && (*Vpp == 0))
		poweroff();
	if (set_Vcc(Vcc))
		goto error_exit;
	if (set_Vpp(Vcc, Vpp))
		goto error_exit;
	return 0;

      error_exit:
	poweroff();
	return -EINVAL;
}

/* Write protection is currently unimplemented */

static int
write_protect_io(unsigned int sock, u_char map)
{
	if (ignore_wrprot) {
		printk(KERN_WARNING "pccf_4xx: Request to write protect IO "
		       "map %d on socket %d will be ignored.\n", map, sock);
		return 0;
	} else {
		printk(KERN_ERR "pccf_4xx: Request to write protect IO "
		       "map %d on socket %d can not be satisfied. "
		       "Aborting.\n You can try reloading the module "
		       "with the paramater ignore_wrprot=1 which "
		       "will aviod this error, but might allow "
		       "critical data on the card to be overwritten.\n",
		       map, sock);
		return -EINVAL;
	}
}

static int
write_unprotect_io(unsigned int sock, u_char map)
{
	return 0;
}

static int
write_protect_mem(unsigned int sock, u_char map)
{
	if (ignore_wrprot) {
		printk(KERN_WARNING "pccf_4xx: Request to write protect "
		       "memory map %d on socket %d will be ignored.\n",
		       map, sock);
		return 0;
	} else {
		printk(KERN_ERR "pccf_4xx: Request to write protect memory "
		       "map %d on socket %d can not be satisfied. "
		       "Aborting.\n You can try reloading the module "
		       "with the paramater ignore_wrprot=1 which "
		       "will aviod this error, but might allow "
		       "critical data on the card to be overwritten.\n",
		       map, sock);
		return -EINVAL;
	}
}

static int
write_unprotect_mem(unsigned int sock, u_char map)
{
	return 0;
}

/****************************************************************************
   PCMCIA Driver
****************************************************************************/

/* This is the handler for the PCCF macro interrupt.  The only interrupts
   signalled by PCCF are the card detection and status change. This driver can
   only clear the card-detect interrupt - the card driver must take care of
   status change interrupts. The status-change interrupt bit is only asserted
   by PCCF when in IO mode. 

   Card detect interrupts always power off the socket and return the socket to
   an initial state. If a card-detect interrupt occurs any status change
   interrupt is bogus and is ignored -- the card ain*t there or has just been
   inserted. */

static u_int pending_events[TOTAL_NUM_OF_SOCKETS];
static spinlock_t pending_event_lock = SPIN_LOCK_UNLOCKED;

static void
pccf_4xx_task(void *dummy)
{
	u_int events;
	int i;

	for (i = 0; i < TOTAL_NUM_OF_SOCKETS; i++) {
		spin_lock_irq(&pending_event_lock);
		events = pending_events[i];
		pending_events[i] = 0;
		spin_unlock_irq(&pending_event_lock);

		if (events && socket[i].handler)
			socket[i].handler(socket[i].info, events);
	}
}

static struct tq_struct pccf_4xx_tqentry = {
	routine:pccf_4xx_task
};

static void
pccf_4xx_interrupt(int irq, void *dev, struct pt_regs *regs)
{
	unsigned int events;
	unsigned short isr = PCMCIA_ISR;

	DEBUG(4, "pccf_4xx: interrupt(%d)\n"
	      "csr = 0x%04x, ccr = 0x%04x, isr = 0x%04x icr = 0x%04x\n",
	      irq, PCMCIA_CSR, PCMCIA_CCR, PCMCIA_ISR, PCMCIA_ICR);

	if (PCMCIA_ISR_DETECT(isr)) {
		pcmcia_clr_cdi();
		poweroff();
		events = SS_DETECT;
	} else if (PCMCIA_ISR_STSCHG(isr))
		events = SS_STSCHG;
	else
		events = 0;

	DEBUG(2, "pccf_4xx: socket 0 event 0x%02x\n", events);

	if (events) {
		spin_lock(&pending_event_lock);
		pending_events[0] |= events;
		spin_unlock(&pending_event_lock);
		schedule_task(&pccf_4xx_tqentry);
	}
}

/* This routine registers a callback to be called when the socket driver
   receives card status change events. */

static int
pccf_4xx_register_callback(unsigned int sock,
			   void (*handler) (void *, unsigned int), void *info)
{
	DEBUG(4, "pccf_4xx: register_callback(%d) = 0x%p, 0x%p\n",
	      sock, handler, info);

	if (sock >= TOTAL_NUM_OF_SOCKETS)
		return -EINVAL;

	socket[sock].handler = handler;
	socket[sock].info = info;
	if (handler == NULL) {
		MOD_DEC_USE_COUNT;
	} else {
		MOD_INC_USE_COUNT;
	}
	return 0;
}

/* The capabilities are copied to the caller. */

static int
pccf_4xx_inquire_socket(unsigned int sock, socket_cap_t * cap)
{
	if (sock >= TOTAL_NUM_OF_SOCKETS) {
		DEBUG(4, "pccf_4xx: inquire_socket(%d) failed\n", sock);
		return -EINVAL;
	}

	DEBUG(0, "pccf_4xx: inquire_socket(%d)\n", sock);

	*cap = capabilities;
	return 0;
}

/* Return the socket status. The PCCF controller does not support all of the
   status signals, e.g., the BVD signals. The SS_STSCHG flag is only set if an
   unmasked STSCHG interrupt is pending. */

static int
pccf_4xx_get_status(unsigned int sock, u_int * value)
{
	unsigned short csr = PCMCIA_CSR;
	unsigned short isr = PCMCIA_ISR;
	*value = 0;

	if (sock >= TOTAL_NUM_OF_SOCKETS) {
		DEBUG(4, "pccf_4xx: get_status(%d) failed\n", sock);
		return -EINVAL;
	}

	if (PCMCIA_CSR_DETECT(csr)) {
		*value |= SS_DETECT;
	}
	if (PCMCIA_CSR_IO(csr)) {
		if (PCMCIA_ISR_STSCHG(isr))
			*value |= SS_STSCHG;
	} else {
		if (PCMCIA_CSR_READY(csr)) {
			*value |= SS_READY;
		}
	}

	if (PCMCIA_CSR_PWR(csr)) {
		*value |= SS_POWERON;
	}

	switch ((PCMCIA_CSR_VS1(csr) << 1) | PCMCIA_CSR_VS2(csr)) {
	case 3:		/* 5V card */
		break;
	case 1:
		*value |= SS_3VCARD;
		break;
	default:
		*value |= SS_XVCARD;
		break;
	};

	DEBUG(1, "pccf_4xx: GetStatus(%d) = %#4.4x\n", sock, *value);
	return 0;
}

/* Copy the socket state structure. */

static int
pccf_4xx_get_socket(unsigned int sock, socket_state_t * state)
{
	if (sock >= TOTAL_NUM_OF_SOCKETS) {
		DEBUG(4, "pccf_4xx: get_socket(%d) failed\n", sock);
		return -EINVAL;
	}

	*state = socket[sock].state;	/* copy the whole structure */

	DEBUG(3, "GetSocket(%d) = flags %#3.3x, Vcc %d, Vpp %d, "
	      "io_irq %d, csc_mask %#2.2x\n", sock, state->flags,
	      state->Vcc, state->Vpp, state->io_irq, state->csc_mask);
	return 0;
}

/* Set the socket parameters, and apply power to the slot. 

   If the io_irq error is triggered it would indicate that the card driver has
   its own idea of which interrupt it wants to use, and that its idea is
   wrong. Again, this controller does not generate interrupts for memory card
   status changes. */

static int
pccf_4xx_set_socket(unsigned int sock, socket_state_t * state)
{
	int Vcc = state->Vcc;
	int Vpp = state->Vpp;

	if (sock >= TOTAL_NUM_OF_SOCKETS) {
		DEBUG(4, "pccf_4xx: set_socket(%d) failed\n", sock);
		return -EINVAL;
	}

	DEBUG(1, "pccf_4xx: SetSocket(%d, flags %#3.3x, Vcc %d, Vpp %d, "
	      "io_irq %d, csc_mask %#2.2x)\n", sock, state->flags,
	      state->Vcc, state->Vpp, state->io_irq, state->csc_mask);

	/* Apply/remove power */

	set_voltage(&Vcc, &Vpp);

	/* Decode csc_mask requests. */

	if (state->csc_mask & SS_DETECT)
		pcmcia_unmask_cdi();
	else
		pcmcia_mask_cdi();	/* This would be extremely bizarre */

	if (state->csc_mask & SS_STSCHG)
		pcmcia_unmask_sti();
	else
		pcmcia_mask_sti();

	if (state->csc_mask & (SS_READY | SS_BATDEAD | SS_BATWARN))
		printk(KERN_WARNING "pccf_4xx: Warning: The PCMCIA controller "
		       "is unable to automatically detect one or more memory "
		       "card status changes requested by the device driver. "
		       "The installed device may not function correctly with "
		       "this driver in this system, and data may be lost "
		       "due to battery failure.\n");

	/* Perform flag actions */

	if (state->flags & SS_RESET) {
		if (Vcc == VOLT00)
			printk(KERN_WARNING "pccf_4xx: Request to reset "
			       " unpowered slot ignored\n");
		else
			pcmcia_reset();
	}

	if (state->flags & SS_IOCARD) {
		pcmcia_io();
		if (state->io_irq && (state->io_irq != PCCF_4XX_CARD_IRQ)) {
			printk(KERN_CRIT "pccf_4xx: io_irq requested as %d, "
			       "should be %d. Aborting.\n",
			       state->io_irq, PCCF_4XX_CARD_IRQ);
			return -EINVAL;
		}
	} else {
		pcmcia_mem();
	}

	if (state->flags & SS_SPKR_ENA)
		printk(KERN_WARNING "pccf_4xx: The request to enable the "
		       " speaker pin of the installed card will be "
		       "ignored.\n");

	/* SS_OUTPUT_ENA -- Nothing to do here in any supported
	   configuration
	   SS_PWR_AUTO   -- Not supported by the controller */

	/* Copy the state to the socket */

	socket[sock].state = *state;

	return 0;
}

/* Copy the IO map back to the caller */

static int
pccf_4xx_get_io_map(unsigned int sock, struct pccard_io_map *io)
{
	u_char map;

	if (sock >= TOTAL_NUM_OF_SOCKETS) {
		DEBUG(4, "pccf_4xx: get_io_map(%d) failed\n", sock);
		return -EINVAL;
	}

	map = io->map;
	if (io->map >= PCMCIA_IO_WIN_NO) {
		printk(KERN_ERR "pccf_4xx: Driver requested IO map %d "
		       "which is not supported by this controller\n", map);
		return -EINVAL;
	}

	*io = socket[sock].io_win[io->map];

	DEBUG(1, "pccf_4xx: GetIOMap(%d, %d) = %#2.2x, %d ns, "
	      "%#4.4x-%#4.4x\n", sock, map, io->flags, io->speed,
	      io->start, io->stop);
	return 0;
}

/* Set up an IO map. This driver uses 2 virtual windows, although there is only
   one physical address space on the PCCF macro.

   There's nothing really to "turn on" here.  We presume the EBC is always set
   up and ready to go. */

static int
pccf_4xx_set_io_map(unsigned int sock, struct pccard_io_map *io)
{
	socket_info_t *s = &socket[sock];
	u_char map;
	int error;

	if (sock >= TOTAL_NUM_OF_SOCKETS) {
		DEBUG(4, "pccf_4xx: set_io_map(%d) failed\n", sock);
		return -EINVAL;
	}

	DEBUG(1, "pccf_4xx: SetIOMap(%d, %d, %#2.2x, %d ns, "
	      "%#4.4x-%#4.4x)\n", sock, io->map, io->flags,
	      io->speed, io->start, io->stop);

	map = io->map;

	if ((io->map >= PCMCIA_IO_WIN_NO) ||
	    (io->start >= PCCF_4XX_IO_WINSIZE) ||
	    (io->stop >= PCCF_4XX_IO_WINSIZE) || (io->stop < io->start)) {
		printk(KERN_ERR "pccf_4xx: set_io_map: Illegal "
		       "IO map: map = %d, start = 0x%08x, stop = 0x%08x\n",
		       io->map, io->start, io->stop);
		return -EINVAL;
	}

	if (io->flags & MAP_ACTIVE) {

		if (io->flags & MAP_WRPROT) {
			if ((error = write_protect_io(sock, io->map)))
				return error;
		} else {
			if ((error = write_unprotect_io(sock, io->map)))
				return error;
		}

		if (io->flags & MAP_16BIT) {
			pcmcia_16bit();
		} else if (io->flags & MAP_AUTOSZ) {
			pcmcia_autosz();
		} else {
			pcmcia_8bit();
		}

		/* MAP_0WS      ignored
		   MAP_USE_WAIT ignored (assumed)
		   MAP_PREFECTH ignored (could be implemented with some
		   MMU gyrations)

		   speed        ignored */
	}

	/* Copy the struct and modify the flags to be accurate based on our
	   limitations. */

	s->io_win[io->map] = *io;
	s->io_win[io->map].flags &=
	    (MAP_ACTIVE | MAP_WRPROT | MAP_AUTOSZ | MAP_16BIT);

	return 0;
}

/* Copy the memory map back to the caller */

static int
pccf_4xx_get_mem_map(unsigned int sock, struct pccard_mem_map *mem)
{
	if (sock >= TOTAL_NUM_OF_SOCKETS) {
		DEBUG(4, "pccf_4xx: get_mem_map(%d) failed\n", sock);
		return -EINVAL;
	}

	if (mem->map >= PCMCIA_MEM_WIN_NO) {
		printk(KERN_ERR "pccf_4xx: Driver requested memory map %d "
		       "which is not supported by this controller\n", mem->map);
		return -EINVAL;
	}

	*mem = socket[sock].mem_win[mem->map];	/* copy the struct */

	DEBUG(1, "pccf_4xx: GetMemMap(%d, %d) = %#2.2x, %d ns, %#5.5lx-%#5."
	      "5lx, %#5.5x\n", sock, mem->map, mem->flags, mem->speed,
	      mem->sys_start, mem->sys_stop, mem->card_start);

	return 0;
}

/* Set the memory map. */

static int
pccf_4xx_set_mem_map(unsigned int sock, struct pccard_mem_map *mem)
{
	socket_info_t *s = &socket[sock];
	int error;
	if (sock >= TOTAL_NUM_OF_SOCKETS) {
		DEBUG(4, "pccf_4xx: set_mem_map(%d) failed\n", sock);
		return -EINVAL;
	}

	if ((mem->map >= PCMCIA_MEM_WIN_NO) ||
	    (mem->sys_start > mem->sys_stop) ||
	    ((mem->sys_stop - mem->sys_start) >= PCCF_4XX_MEM_WINSIZE) ||
	    ((mem->card_start + (mem->sys_stop - mem->sys_start)) >=
	     PCCF_4XX_MEM_WINSIZE)) {
		DEBUG(4, "pccf_4xx: set_mem_map: Illegal "
		      "memory map: map = %d, start = 0x%08lx, stop = 0x%08lx "
		      "card_start = 0x%08x\n",
		      mem->map, mem->sys_start, mem->sys_stop, mem->card_start);
		return -EINVAL;
	}

	DEBUG(1, "pccf_4xx: SetMemMap[entry]"
	      "(%d, %d, %#2.2x, %d ns, %#8.8lx-%#8.8"
	      "lx, %#5.5x)\n", sock, mem->map, mem->flags, mem->speed,
	      mem->sys_start, mem->sys_stop, mem->card_start);

	if (mem->flags & MAP_ACTIVE) {
		mem->sys_stop -= mem->sys_start;
		if (mem->flags & MAP_ATTRIB)
			mem->sys_start = PCMCIA_ATTRIB_PADDR + mem->card_start;
		else
			mem->sys_start = PCCF_4XX_MEM_PADDR + mem->card_start;
		mem->sys_stop += mem->sys_start;
	}

	DEBUG(2, "pccf_4xx: SetMemMap[exit]"
	      "(%d, %d, %#2.2x, %d ns, %#8.8lx-%#8.8"
	      "lx, %#5.5x)\n", sock, mem->map, mem->flags, mem->speed,
	      mem->sys_start, mem->sys_stop, mem->card_start);

	if (mem->flags & MAP_ACTIVE) {

		if (mem->flags & MAP_WRPROT) {
			if ((error = write_protect_mem(sock, mem->map)))
				return error;
		} else {
			if ((error = write_unprotect_mem(sock, mem->map)))
				return error;
		}

		if (mem->flags & MAP_16BIT) {
			pcmcia_16bit();
		} else if (mem->flags & MAP_AUTOSZ) {
			pcmcia_autosz();
		} else {
			pcmcia_8bit();
		}

		/* MAP_0WS      ignored
		   MAP_USE_WAIT ignored (assumed) */

		if (mem->speed > MIN_SPEED) {
			printk(KERN_ERR "pccf_4xx: Memory speed %d ns is not "
			       "supported.\n", mem->speed);
			return -EINVAL;
		}
	}

	/* Copy the struct and modify the flags to be accurate based on our
	   limitations. */

	s->mem_win[mem->map] = *mem;
	s->mem_win[mem->map].flags &=
	    (MAP_ACTIVE | MAP_WRPROT | MAP_AUTOSZ | MAP_16BIT);

	return 0;
}

#if 1 /* linux-pm */
#include <linux/device.h>

static int pccf_4xx_dpm_suspend(struct device *dev, u32 state, u32 level);
static int pccf_4xx_dpm_resume(struct device *dev, u32 level);

static struct device_driver pccf_4xx_driver_ldm = {
       name:      "pccf_4xx",
       devclass:  NULL,
       probe:     NULL,
       suspend:   pccf_4xx_dpm_suspend,
       resume:    pccf_4xx_dpm_resume,
       remove:    NULL,
};

static struct device pccf_4xx_device_ldm = {
       name: "PCMCIA/CF+",
       bus_id: "pcmcia",
       driver: NULL,
       power_state: DPM_POWER_ON,
};

static void pccf_4xx_ldm_driver_register(void)
{
   extern void plb_driver_register(struct device_driver *driver);

   plb_driver_register(&pccf_4xx_driver_ldm);
}

static void pccf_4xx_ldm_device_register(void)
{
   extern void plb_device_register(struct device *device);

   plb_device_register(&pccf_4xx_device_ldm);
}
static void pccf_4xx_ldm_driver_unregister(void)
{
   extern void plb_driver_unregister(struct device_driver *driver);

   plb_driver_unregister(&pccf_4xx_driver_ldm);
}

static void pccf_4xx_ldm_device_unregister(void)
{
   extern void plb_device_unregister(struct device *device);

   plb_device_unregister(&pccf_4xx_device_ldm);
}

static u16 hw_regs[3];
enum { CCR_reg, ICR_reg, MRR_reg };

static int pccf_4xx_dpm_suspend(struct device *dev, u32 state, u32 level)
{

  switch(level)
  { 
     case SUSPEND_POWER_DOWN:

            pcmcia_dpm_suspend_card(0);

            hw_regs[CCR_reg] = PCMCIA_CCR;  
            hw_regs[ICR_reg] = PCMCIA_ICR;
            hw_regs[MRR_reg] = PCMCIA_MRR;             

    break;
  }
  
  return 0;
}

static int pccf_4xx_dpm_resume(struct device *dev, u32 level)
{

  switch(level)
  {
     case RESUME_POWER_ON:

          pcmcia_mrr();		/* Reset the PCCF Macro */
          poweroff();		/* Make sure socket is powered down */

          SET_PCMCIA_CCR(hw_regs[CCR_reg]);  
	   SET_PCMCIA_ICR(hw_regs[ICR_reg]);
          SET_PCMCIA_MRR(hw_regs[MRR_reg]);

          pcmcia_dpm_resume_card(0);

     break;
  }
  return 0;
}

#endif /* linux-pm */

static int
pccf_4xx_sock_init(unsigned int s)
{
	DEBUG(3, "pccf_4xx: sock_init(%d)\n", s);

	pccf_4xx_set_socket(s, &dead_socket);
	return 0;

}

static int
pccf_4xx_suspend(unsigned int s)
{
	return (pccf_4xx_set_socket(s, &dead_socket));
}
static void
pccf_4xx_proc_setup(unsigned int sock, struct proc_dir_entry *base)
{
}

/* Service table and service dispatching */

static struct pccard_operations pccf_4xx_services = {
	&pccf_4xx_sock_init,
	&pccf_4xx_suspend,
	&pccf_4xx_register_callback,
	&pccf_4xx_inquire_socket,
	&pccf_4xx_get_status,
	&pccf_4xx_get_socket,
	&pccf_4xx_set_socket,
	&pccf_4xx_get_io_map,
	&pccf_4xx_set_io_map,
	&pccf_4xx_get_mem_map,
	&pccf_4xx_set_mem_map,
	&pccf_4xx_proc_setup
};

/* The initialization/exit functions. */

static int __init
init_pccf_4xx(void)
{
	servinfo_t serv;
	int error;

	printk(KERN_INFO "%s\n", version);

	DEBUG(0, "pccf_4xx: Physical base address is 0x%08lx\n", PCCF_4XX_PADDR);
	DEBUG(0, "pccf_4xx: Virtual base addresses: macro=0x%p, mem=0x%lx"
	      " io=0x%lx\n", pccf_4xx_macro_vaddr, _ISA_MEM_BASE, _IO_BASE);

	CardServices(GetCardServicesInfo, &serv);
	if (serv.Revision != CS_RELEASE_CODE) {
		printk(KERN_NOTICE
		       "pccf_4xx: Card services version does not match!\n"
		       "Card Services is 0x%04x, CS_RELEASE_CODE is 0x%04x\n",
		       serv.Revision, CS_RELEASE_CODE);
		return -EBUSY;
	}

	pcmcia_mrr();		/* Reset the PCCF Macro */
	poweroff();		/* Make sure socket is powered down */
	pcmcia_mask_cdi();	/* Mask card-detect interrupt */
	pcmcia_mask_sti();	/* Mask status change interrupt */
	pcmcia_clr_cdi();	/* Clear card detect status */

	/* Note that the PCMCIA macro interrupts are not shared. */

	socket[0].intr = PCCF_4XX_MACRO_IRQ;

	if ((error = request_irq(PCCF_4XX_MACRO_IRQ, pccf_4xx_interrupt, 0, "pcmcia",
				 &socket[0])) != 0) {
		printk(KERN_ERR "pccf_4xx: IRQ %d not available!\n", PCCF_4XX_MACRO_IRQ);
		return error;
	}

	if ((error = register_ss_entry(sockets, &pccf_4xx_services)) != 0) {
		printk(KERN_ERR "pccf_4xx: register_ss_entry() failed!\n");
		goto error_ss;
	}

#if 1 /* linux-pm */
	pccf_4xx_ldm_device_register();
        pccf_4xx_ldm_driver_register();
#endif /* linux-pm */	

	return 0;

      error_ss:
	free_irq(socket[0].intr, &socket[0]);
	return error;
}

static void __exit
exit_pccf_4xx(void)
{
	poweroff();
	pcmcia_mrr();

	request_module("pcmcia_core");

	unregister_ss_entry(&pccf_4xx_services);

	free_irq(socket[0].intr, &socket[0]);

#if 1 /* linux-pm */
        pccf_4xx_ldm_device_unregister();
        pccf_4xx_ldm_driver_unregister();
#endif /* linux-pm */	
}

module_init(init_pccf_4xx);
module_exit(exit_pccf_4xx);
