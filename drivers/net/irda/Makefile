# File: drivers/irda/Makefile
#
# Makefile for the Linux IrDA infrared port device drivers.
#
# 9 Aug 2000, Christoph Hellwig <hch@infradead.org>
# Rewritten to use lists instead of if-statements.
#

O_TARGET := irda.o

export-objs	= irport.o


obj-$(CONFIG_IRTTY_SIR)		+= irtty.o
obj-$(CONFIG_IRPORT_SIR)	+= 		irport.o
obj-$(CONFIG_USB_IRDA)		+= irda-usb.o
obj-$(CONFIG_NSC_FIR)		+= nsc-ircc.o
obj-$(CONFIG_WINBOND_FIR)	+= w83977af_ir.o
obj-$(CONFIG_SA1100_FIR)	+= sa1100_ir.o
obj-$(CONFIG_PXA_FIR)	        += pxa_ir.o
obj-$(CONFIG_PXA_FICP)	        += pxaficp_ir.o
obj-$(CONFIG_TOSHIBA_FIR)	+= toshoboe.o
obj-$(CONFIG_SMC_IRCC_FIR)	+= smc-ircc.o	irport.o
obj-$(CONFIG_ALI_FIR)		+= ali-ircc.o
obj-$(CONFIG_VLSI_FIR)		+= vlsi_ir.o
obj-$(CONFIG_ESI_DONGLE)	+= esi.o
obj-$(CONFIG_TEKRAM_DONGLE)	+= tekram.o
obj-$(CONFIG_ACTISYS_DONGLE)	+= actisys.o
obj-$(CONFIG_GIRBIL_DONGLE)	+= girbil.o
obj-$(CONFIG_LITELINK_DONGLE)	+= litelink.o
obj-$(CONFIG_OLD_BELKIN_DONGLE)	+= old_belkin.o
obj-$(CONFIG_EP7211_IR)		+= ep7211_ir.o
obj-$(CONFIG_AU1000_FIR)	+= au1k_ir.o
obj-$(CONFIG_MCP2120_DONGLE)	+= mcp2120.o
obj-$(CONFIG_ACT200L_DONGLE)	+= act200l.o
obj-$(CONFIG_MA600_DONGLE)	+= ma600.o
obj-$(CONFIG_OMAP_SIR)          += omap-sir.o
obj-$(CONFIG_OMAP1610_IR)       += omap1610-ir.o 
obj-$(CONFIG_OMAP730_IR)	+= omap730-ir.o 
obj-$(CONFIG_MX2_IR)	        += mx21_ir.o

include $(TOPDIR)/Rules.make
