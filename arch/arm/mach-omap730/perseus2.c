/*
 * linux/arch/arm/mach-omap730/perseus2.c
 *
 * This file contains Perseus2-specific code.
 *
 * Copyright (c) 2004, MPC-Data Limited (http://www.mpc-data.co.uk)
 * Dave Peverley <dpeverley at mpc-data.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <asm/serial.h>
#include <asm/string.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/arch/hardware.h>
#include <asm/setup.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/arch/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/arch/ck.h>
#include <asm/arch/irqs.h>
#include <asm/arch/omap730_config.h>
#include <asm/arch/perseus2.h>

static void
perseus2_init_irq(void)
{
	omap730_init_irq();
}

static void __init
fixup_perseus2(struct machine_desc *desc, struct param_struct *params,
	        char **cmdline, struct meminfo *mi)
{
	struct tag *t = (struct tag *)params;

	setup_initrd(__phys_to_virt(0x10800000), 4 * 1024 * 1024);
	if (t->hdr.tag != ATAG_CORE) {
		t->hdr.tag = ATAG_CORE;
		t->hdr.size = tag_size(tag_core);
		t->u.core.flags = 0;
		t->u.core.pagesize = PAGE_SIZE;
		t->u.core.rootdev = RAMDISK_MAJOR << 8 | 0;
		t = tag_next(t);

		t->hdr.tag = ATAG_MEM;
		t->hdr.size = tag_size(tag_mem32);
		t->u.mem.start = PHYS_OFFSET;
		t->u.mem.size  = 32 * 1024 * 1024;
		t = tag_next(t);

#ifdef	CONFIG_BLK_DEV_RAM
		t->hdr.tag = ATAG_RAMDISK;
		t->hdr.size = tag_size(tag_ramdisk);
		t->u.ramdisk.flags = 1;
		t->u.ramdisk.size = CONFIG_BLK_DEV_RAM_SIZE;
		t->u.ramdisk.start = 0;
		t = tag_next(t);
#endif

		t->hdr.tag = ATAG_NONE;
		t->hdr.size = 0;
	}

	return;
}

static void __init
perseus2_serial_init(void)
{
#ifdef	CONFIG_SERIAL
	struct serial_struct s;

	memset(&s, 0, sizeof(s));

	s.line = 0;	/* ttyS0 */
	s.type = PORT_16750;
	s.xmit_fifo_size = 64;
	s.baud_base = BASE_BAUD;
	s.iomem_base = (u8*)OMAP730_UART1_BASE;
	s.iomem_reg_shift = 0;
	s.io_type = SERIAL_IO_MEM;
	s.irq = INT_UART1;
	s.flags = STD_COM_FLAGS;
	if (early_serial_setup(&s) != 0) {
		panic("Could not register ttyS0!");
	}
#endif
}

/* In a Rev. 5 P2, we have eight different versions of the physical memory map 
 * depending on whether we booted from external flash on CS3 or internal boot 
 * ROM on CS0 (switch S401-9), whether NOR flash is enabled or disabled 
 * (switch S401-7), and whether NAND flash is enabled or disabled 
 * (switch S401-8).
 *
 *	P2 Rev. 5 Switch settings
 *	S401-7 ON  --> NOR flash enabled
 *	       OFF --> NOR flash disabled
 *	S401-8 ON  --> NAND flash enabled
 *	       OFF --> NAND flash disabled
 *	S401-9 ON  --> boot from external flash on CS3
 *	       OFF --> boot from internal ROM on CS0
 *
 *	S401-9 OFF, boot from internal ROM on CS0
 *		S401-7	S401-8	CS0		CS2		CS3
 *
 *		OFF	OFF	ROM		---		---
 *				0x00000000
 *
 *		OFF	ON	ROM		---		NAND
 *				0x00000000			0x0C000000
 *
 *		ON	OFF	ROM		---		NOR
 *				0x00000000			0x0C000000
 *
 *		ON	ON	ROM		NAND		NOR
 *				0x00000000	0x08000000	0x0C000000
 *
 *	S401-9 ON, boot from external flash on CS3
 *		S401-7	S401-8	CS0		CS2		CS3
 *
 *		OFF	OFF	ROM		---		---  **invalid**
 *				0x0C000000
 *
 *		OFF	ON	ROM		---		NAND **invalid**
 *				0x0C000000			0x00000000
 *
 *		ON	OFF	ROM		---		NOR
 *				0x0C000000			0x00000000
 *
 *		ON	ON	ROM		NAND		NOR
 *				0x0C000000	0x08000000	0x00000000
 */

#define MAP_DESC(base,start,size,domain,r,w,c,b) { base,start,size,domain,r,w,c,b }
static struct map_desc perseus2_common_io_desc[] __initdata = {
	MAP_DESC(PERSEUS2_FPGA_BASE,
		 PERSEUS2_FPGA_START,
		 PERSEUS2_FPGA_SIZE,
		 DOMAIN_IO,
		 0,1,0,0),
	LAST_DESC
};


static void __init perseus2_map_io(void)
{
	omap730_map_io();
	iotable_init(perseus2_common_io_desc);

	perseus2_serial_init();

	/*
	 *  CSx timings
	 */

	/* CS0 timings setup : Flash */
	*((volatile __u32 *) FLASH_CFG_0) = 0x0000fff3;
	*((volatile __u32 *) FLASH_ACFG_0_1) = 0x00000088;
	/* CS1 timings setup : Ethernet */
	*((volatile __u32 *) FLASH_CFG_1) = 0x0000fff3;
	*((volatile __u32 *) FLASH_ACFG_1_1) = 0x00000000;

	/* 
	 * OMAP730 MUX configuration 
	 *
	 * I REALLY wanted to put these in generic.c but as they all depend
	 * on what pins go where on the PCB, its not the right place to put 
	 * it all :-(
	 */

	/* Configure IRQ lines */

	OMAP730_PIN_MUX(D_MPU_NIRQ, MPU_EXT_NIRQ);      /* SMC Ethernet IRQ Line. */
	
	/* Configure GPIO lines */

	OMAP730_PIN_MUX(D_TPU_TSPEN1, GPIO_8);          /* MMC Card Detect line */
	OMAP730_PIN_MUX(D_RFEN,       GPIO_19);         /* Reset line for LCD, USB, BT. */
	OMAP730_PIN_MUX(D_MCLK_OUT,   GPIO_35);         /* USB Charge current select line */
	OMAP730_PIN_MUX(D_CRESET,     GPIO_36);         /* Headset detect line */
	OMAP730_PIN_MUX(D_EAC_CDI,    GPIO_67);         /* MMC Write Protect Line */	
	OMAP730_PIN_MUX(D_DDR,        GPIO_72_73_74);   /* Charge state LED line, 
							 * Battery/AC Charge enable line, 
							 * NAND Flash RnB line */
	OMAP730_PIN_MUX(D_SPI1_SEN2, GPIO_134);         /* LCD backlight enable line */
	OMAP730_PIN_MUX(D_CLK13MREQ, GPIO_145);         /* OTG XCVR INT line */

	/* Configure UART1 out of the P2 DB9, and UART2 (IrDA only) out of 
	 * the Debug board DB9.
	 *
	 * Set DIP7 on the debug board to ON to route the MPU_RX_IR line to the RXD
	 * pin on the IRDA module, otherwise the IRDA module will not receive.
	 *
	 * Note : This works fine for UART1, but UART2 can't be used in 
	 * modem mode as when in modem mode, data travels via a different
	 * MUX MPU_UART_RX2 / MUX MPU_UART_TX2 which can only be muxed to
	 * the same destination as UART1, effectively replacing it.
 	 */

	OMAP730_PIN_MUX(D_UART_TX_RX,   MPU_UART1);
	OMAP730_PIN_MUX(D_UART_IRDA_RX, MPU_UART_RX_IR2);
	OMAP730_PIN_MUX(D_UART_IRDA_TX, MPU_UART_TX_IR2);
	OMAP730_PIN_MUX(D_UART_IRDA_SD, MPU_UART_SD2);

	/* Configure KBR5:11, KBR0, KBR1, KBR2:0, KBR3, KBR4, KBC0, KBC1, 
	 * KBC2, KBC3, KBC4:0 for the keypad driver */

	OMAP730_PIN_MUX(D_SMC_IO, KBR_5);
	OMAP730_PIN_MUX(D_KB0, KBR_0);
	OMAP730_PIN_MUX(D_KB1, KBR_1);
	OMAP730_PIN_MUX(D_KB2, KBR_2);
	OMAP730_PIN_MUX(D_KB3, KBR_3);
	OMAP730_PIN_MUX(D_KB4, KBR_4);
	OMAP730_PIN_MUX(D_KB5, KBC_0);
	OMAP730_PIN_MUX(D_KB6, KBC_1);
	OMAP730_PIN_MUX(D_KB7, KBC_2);
	OMAP730_PIN_MUX(D_KB8, KBC_3);
	OMAP730_PIN_MUX(D_KB9, KBC_4);

        /* Configure I2C_SCK, I2C_SDA for the I2C driver. */

	OMAP730_PIN_MUX(D_I2C_SCK, MPU_I2C_SCK);
	OMAP730_PIN_MUX(D_I2C_SDA, MPU_I2C_SDA);

        /* Configure SDMC(SDMC_CLK, SDMC_CMD, SDMC_DAT0, SDMC_DAT1) misc pins 
	 * and DAT2, DAT3 for the MMC driver. */

	OMAP730_PIN_MUX(D_SDMC,      SDMC_PINS);
	OMAP730_PIN_MUX(D_SDMC_DAT2, SDMC_DAT_2);
	OMAP730_PIN_MUX(D_SDMC_DAT3, SDMC_DAT_3);

	/* Configure nCS1, SDO, SCLK for the uWire driver */

	OMAP730_PIN_MUX(D_SMC_CLK, MPU_UW_SCLK);
	OMAP730_PIN_MUX(D_SMC_RST, MPU_UW_SDO);
	OMAP730_PIN_MUX(D_SMC_CD,  MPU_UW_nSCS1);

	/* NOTE : We don't MUX SDI as it shares a muxed pin with KBR_5 (see 
	 *        above) on the P2. This may need to be done on other OMAP730
	 *        based devices if required.
	 */	

	/*  Configure pins for the LCD Driver */

	OMAP730_PIN_MUX(D_LCD_PXL_15_12, LCD_PIXEL);
	OMAP730_PIN_MUX(D_LCD_PXL_10,    LCD_PIXEL);          
	OMAP730_PIN_MUX(D_LCD_PXL_11,    LCD_PIXEL);          
	OMAP730_PIN_MUX(D_LCD_PXL_9_2,   LCD_PIXEL);
	OMAP730_PIN_MUX(D_LCD_UWIRE,     LCD_MISC);
	OMAP730_PIN_MUX(D_LCD_VSYNC,     LCD_VSYNC_AC);

	/* Configure pins for the USB Driver using an external OTG 
	 * Transceiver in bidirectional mode 
	 */

	OMAP730_CONFIGURE(USB_TRANSCEIVER_SEL, EXT_XCVR);
	OMAP730_PIN_MUX(D_DM,    OTG);
	OMAP730_PIN_MUX(D_PU_EN, USB_RCV);
	OMAP730_PIN_MUX(D_VBUSI, USB_TXEN);

	// TODO : Not sure if this should be elsewhere, but the 1301 is
	//        hardwired to low speed...
	OMAP730_CONFIGURE(USB_TRANSCEIVER_SPEED, LOW);
	
	
	/* 
	 * TODO : The TRM says that for this USB config, we *may* want to set
	 *        CONF9 to move MPU_EXT_NIRQ to mode 1 : USB_VBUSI, but this
	 *        would break ethernet... :-/
	 */


	 /* Configure pins for the camera */

	 OMAP730_PIN_MUX(D_CAM_LCLK,  CAM_LCLK);
	 OMAP730_PIN_MUX(D_CAM_EXCLK, CAM_EXCLK);
	 OMAP730_PIN_MUX(D_CAM_HS,    CAM_HS);
	 OMAP730_PIN_MUX(D_CAM_VS,    CAM_VS);
	 OMAP730_PIN_MUX(D_CAM_RSTZ,  CAM_RSTZ);
	 OMAP730_PIN_MUX(D_CAM_DAT0,  CAM_DATA_0);
	 OMAP730_PIN_MUX(D_CAM_DAT1,  CAM_DATA_1);
	 OMAP730_PIN_MUX(D_CAM_DAT2,  CAM_DATA_2);
	 OMAP730_PIN_MUX(D_CAM_DAT3,  CAM_DATA_3);
	 OMAP730_PIN_MUX(D_CAM_DAT4,  CAM_DATA_4);
	 OMAP730_PIN_MUX(D_CAM_DAT5,  CAM_DATA_5);
	 OMAP730_PIN_MUX(D_CAM_DAT6,  CAM_DATA_6);
	 OMAP730_PIN_MUX(D_CAM_DAT7,  CAM_DATA_7);
	 OMAP730_PIN_PE(D_CAM_LCLK,  PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_EXCLK, PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_HS,    PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_VS,    PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_RSTZ,  PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_DAT0,  PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_DAT1,  PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_DAT2,  PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_DAT3,  PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_DAT4,  PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_DAT5,  PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_DAT6,  PE_ENABLE);
	 OMAP730_PIN_PE(D_CAM_DAT7,  PE_ENABLE);

	/* 
	 * GPIO Configuration - set direction and default state.
	 */

        GPIO_SETUP(GPIO_BACKLIGHT_ENABLE);
	GPIO_SETUP(GPIO_MMC_WP);
	GPIO_SETUP(GPIO_MMC_CD);
	GPIO_SETUP(GPIO_LCD_USB_BT_RESET);
	GPIO_SETUP(GPIO_OTG_XCVR_INT);
	GPIO_SETUP(GPIO_NAND_RnB);
	GPIO_SETUP(GPIO_HEADSET_DETECT);
	GPIO_SETUP(GPIO_CHARGE_ENABLE);
	GPIO_SETUP(GPIO_USB_CHARGE_CURR);
	GPIO_SETUP(GPIO_CHARGE_STATE_LED);
	GPIO_SETUP(GPIO_ENABLE_MMC_VDD);

	/*
	 *  DPLL clock enable requests.
	 */

	OMAP730_CONFIGURE(UART1_DPLL_REQ, ACTIVE);
	OMAP730_CONFIGURE(UART3_DPLL_REQ, ACTIVE);

	/* 
	 * LCD/USB/BT reset: low pulse 
	 * This used to live in the LCD driver but we dont want to reset a 
	 * possibly already initialised BT or USB driver by reseting all 
	 * there! 
	 *
	 * TODO : Is there any BT/USB setup we must should do prior to reset 
	 *        that should be here?
	 *
	 */

	set_p2_reset_lcd_usb_bt(LCD_USB_BT_RESET_RELEASE);
	mdelay(1);
	set_p2_reset_lcd_usb_bt(LCD_USB_BT_RESET_HOLD);
	mdelay(1);
	set_p2_reset_lcd_usb_bt(LCD_USB_BT_RESET_RELEASE);
	mdelay(5);

	return;
}

MACHINE_START(OMAP_PERSEUS2, "TI-Perseus2/OMAP730")
	MAINTAINER("MontaVista Software, Inc.")
	BOOT_MEM(0x10000000, 0xe0000000, 0xe0000000)
	BOOT_PARAMS(0x10000100)
	FIXUP(fixup_perseus2)
	MAPIO(perseus2_map_io)
	INITIRQ(perseus2_init_irq)
MACHINE_END
