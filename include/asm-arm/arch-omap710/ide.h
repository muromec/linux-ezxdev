/*
 *  linux/include/asm-omap710/ide.h
 *
 *  Copyright (C) 1998 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Modifications:
 *   29-07-1998 RMK     Major re-work of IDE architecture specific code
 *   15-03-2002 Steve Kranz
 *                      Modified for use on the Texas Instrument's omap710
 *                      EVM for the CompactFlash slot. this mostly involved adding the
 *                      do_board_specific_setup() routine. (skranz@ridgerun.com)
 */
#include <asm/irq.h>
#include <asm/arch/hardware.h>

#define CFLASH_REG_BASE nCS1_BASE+0x1000 /* virtual address, not physical */
#define CFLASH_CONFIG   nCS1_BASE+0x0a00 /* virtual address, not physical */
#define CF_IRQ_NUM      INT_CFIREQ /* interrupt number associated with the CF IREQ pin */

static void clear_pending_CF_ints(void)
{
  // Write to the level 2 interrupt controller ITR reg.
  *((unsigned long *)0xfffecb00) = 0xfffffff3;
}

static int do_board_specific_setup(void)
{
  #define HWCONFIG_REG_BASE  0xfffe1000
  #define EMIF_REG_BASE      0xfffecc00
  #define CF_CONTROLLER_BASE 0x0fffbe000

  unsigned short reg_val;
  unsigned long volatile i;
  
  // On the omap710 the card's registers are accessible
  // to the driver only after device specific setup has been
  // performed. After this logic is performed the registers
  // located at address CFLASH_REG_BASE will correspond to
  // the inserted Compact Flash card.
  reg_val = *((unsigned short *)(HWCONFIG_REG_BASE+0x1e));
  reg_val = reg_val & 0xffab; // clear a few bits.
  *((unsigned short *)(HWCONFIG_REG_BASE+0x1e)) = reg_val; /* PERSEUS_IO_CONF0 ; power-on default = 0x3fff */
#if 0
  *((unsigned short *)(HWCONFIG_REG_BASE+0x24)) = 0xc000; /* PERSEUS_IO_CONF3 ; power-on default = 0xc000 */
  *((unsigned short *)(HWCONFIG_REG_BASE+0x26)) = 0x0000; /* PERSEUS_IO_CONF4 ; power-on default = 0x0000 */
  *((unsigned short *)(HWCONFIG_REG_BASE+0x30)) = 0xfcfe; /* PERSEUS_IO_CONF9 ; power-on default = 0xfcfe */
#endif

  udelay(100);
  // for (i=0;i<0xf;i++) {/*pause*/}
  // Set up the EMIF to access flash in asyncronous mode on CS1
  *((unsigned int *)(EMIF_REG_BASE+0x14)) = 0x0000fff9; /* MIF_Slow_nCS1 */
  *((unsigned int *)(EMIF_REG_BASE+0x0c)) = 0x0000001d; /* write protect turned off. */
  *((unsigned int *)(EMIF_REG_BASE+0x40)) = 0x00020002; /* CONFIG_DYNAMIC_WAITSTATE_ON_CS1_POS */

  // Configure the board's CF controller to put compact flash in CS1 mem space.
  // Recall that CS1 memory space starts at 0x04000000 (omap710);
  *((unsigned short *)(CF_CONTROLLER_BASE+0x2)) = 0x000d;

  // Reset the board's CF controller.
  *((unsigned short *)(CF_CONTROLLER_BASE+0x4)) = 0x0001;
  *((unsigned short *)(CF_CONTROLLER_BASE+0x4)) = 0x0000;
  udelay(100);
  //for (i=0;i<0xf;i++) {/*pause*/}

  // Check the board's CF Controller status
  reg_val = *((unsigned short *)(CF_CONTROLLER_BASE+0x0));
  if (reg_val & 0x0001) {
    printk("No Card detected in CompactFlash slot\n");
    printk("Reminder: The CF slot requires that JP36 jumper be in 1-2 position.\n");
    return 0;
  }

  {
    // Note: IRQ processing.
    // Put the CF card in I/O mode as apposed to Memory Map mode.
    // In I/O mode the IREQ CF pin is active low, and in MemoryMap mode
    // the pin is the opposite. The boards CF controller expects
    // to see it only one way. In other respects the registers of
    // the CF card are treated the same with the exception that the
    // Mem Mode has registers sitting at PHY address 0x04000000 while
    // the I/O Mode places the same registers at 0x04001000 instead.
    // No changes are required in the ide driver between these two
    // modes -- rather it only effects the IRQ processing of the CF
    // card. Skranz, Mar, 2002
    unsigned char reg;
    reg = *((unsigned char *)CFLASH_CONFIG);
    reg = reg | 0x01;
    *((unsigned char *)CFLASH_CONFIG) = reg; /* See Note above. */
  }

  clear_pending_CF_ints();
  printk("Card detected in CompactFlash slot.\n");
  return 1;
}

/*
 * Set up a hw structure for a specified data port, control port and IRQ.
 * This should follow whatever the default interface uses.
 */
static __inline__ void
ide_init_hwif_ports(hw_regs_t *hw, int data_port, int ctrl_port, int *irq)
{
        ide_ioreg_t reg = (ide_ioreg_t) data_port;
        int i;

        for (i = IDE_DATA_OFFSET; i <= IDE_STATUS_OFFSET; i++) {
                hw->io_ports[i] = reg;
                reg += 1;
        }
        hw->io_ports[IDE_CONTROL_OFFSET] = (ide_ioreg_t) ctrl_port;
        if (irq)
                *irq = 0;
}

/*
 * This registers the standard ports for this architecture with the IDE
 * driver.
 */
static __inline__ void
ide_init_default_hwifs(void)
{
        hw_regs_t hw;

        memset(&hw, 0, sizeof(hw));

        if (!do_board_specific_setup()) return;
        
        ide_init_hwif_ports(&hw, CFLASH_REG_BASE, CFLASH_REG_BASE+IDE_CONTROL_OFFSET, NULL);
#if 1 /* SKranz, Mar 2002. */
        // This is the omap710 way. We'll define the irq number
        // explicitly here so as to prevent the ide driver from
        // trying to probe for it at runtime since that probe logic
        // somehow set the stage for IRQ problems with the ethernet
        // chipset. 
        hw.irq = CF_IRQ_NUM;
#else
        // The recommended way. This will cause the ide driver
        // to probe at run-time to determine the irq associated
        // with the device.
        hw.irq = 0;
#endif        
        ide_register_hw(&hw, NULL);
}
