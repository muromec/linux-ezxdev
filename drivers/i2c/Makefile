#
# Makefile for the kernel i2c bus driver.
#
#
# Copyright (C) 2003 Motorola Inc.
# History:
#   Jay Jia(w20091),   Motorola  Nov 25,2003,          add i2c slave drivers
# 
O_TARGET := i2c.o

export-objs	:= i2c-core.o i2c-algo-bit.o i2c-algo-pcf.o i2c-algo-pxa.o \
		   i2c-proc.o i2c-algo-8xx.o i2c-algo-ibm_ocp.o  e680_camera.o \
		   i2c-ov9640.o i2c-algo-ite.o i2c-adap-ibm_ocp.o a780_camera.o

mod-subdirs     := xilinx_iic

obj-$(CONFIG_I2C)		+= i2c-core.o
obj-$(CONFIG_I2C_CHARDEV)	+= i2c-dev.o
obj-$(CONFIG_I2C_ALGOBIT)	+= i2c-algo-bit.o
obj-$(CONFIG_I2C_BITLP)		+= i2c-philips-par.o
obj-$(CONFIG_I2C_BITELV)	+= i2c-elv.o
obj-$(CONFIG_I2C_BITVELLE)	+= i2c-velleman.o
obj-$(CONFIG_I2C_FRODO)		+= i2c-frodo.o
obj-$(CONFIG_I2C_ALGOPCF)	+= i2c-algo-pcf.o
obj-$(CONFIG_I2C_PCFISA)	+= i2c-elektor.o
obj-$(CONFIG_I2C_PROC)		+= i2c-proc.o
obj-$(CONFIG_I2C_ALGO8XX)	+= i2c-algo-8xx.o i2c-rpx.o
obj-$(CONFIG_I2C_IBM_OCP)	+= i2c-algo-ibm_ocp.o i2c-adap-ibm_ocp.o
obj-$(CONFIG_I2C_ITE_ALGO)	+= i2c-algo-ite.o
obj-$(CONFIG_I2C_ITE_ADAP)	+= i2c-adap-ite.o
obj-$(CONFIG_I2C_ALGO8260)	+= i2c-algo-8260.o
obj-$(CONFIG_I2C_KEYWEST)	+= i2c-keywest.o
subdir-$(CONFIG_I2C_XILINX)	+= xilinx_iic
obj-$(CONFIG_I2C_XILINX)	+= xilinx_iic/xilinx_iic.o
obj-$(CONFIG_I2C_X220)		+= i2c-x220.o

obj-$(CONFIG_I2C_PFS168)	+= i2c-pfs168.o
obj-$(CONFIG_I2C_M41T11)	+= m41t11.o
obj-$(CONFIG_I2C_OMAP1510)      += i2c-omap1510.o
obj-$(CONFIG_I2C_OMAP1610)      += i2c-omap1610.o
obj-$(CONFIG_I2C_OMAP730)       += i2c-omap730.o

# PXA adapters
obj-$(CONFIG_I2C_PXA_ALGO)	+= i2c-algo-pxa.o
obj-$(CONFIG_I2C_PXA_ADAP)	+= i2c-adap-pxa.o
obj-$(CONFIG_I2C_ADCM2700)     += e680_camera.o
obj-$(CONFIG_I2C_A780_CAMERA)	+= a780_camera.o

obj-$(CONFIG_I2C_ADCM2650)	+= i2c-adcm2650.o

obj-$(CONFIG_I2C_DBMX2)		+= i2c-dbmx2.o

# This is needed for automatic patch generation: sensors code starts here
obj-$(CONFIG_I2C_ALI1535)		+= i2c-ali1535.o
obj-$(CONFIG_I2C_ALI15X3)		+= i2c-ali15x3.o
obj-$(CONFIG_I2C_AMD756)		+= i2c-amd756.o
obj-$(CONFIG_I2C_HYDRA)			+= i2c-hydra.o
obj-$(CONFIG_I2C_I801)			+= i2c-i801.o
obj-$(CONFIG_I2C_I810)			+= i2c-i810.o
obj-$(CONFIG_I2C_ISA)			+= i2c-isa.o
obj-$(CONFIG_I2C_PIIX4)			+= i2c-piix4.o
obj-$(CONFIG_I2C_SIS5595)		+= i2c-sis5595.o
obj-$(CONFIG_I2C_TSUNAMI)		+= i2c-tsunami.o
obj-$(CONFIG_I2C_VIA)			+= i2c-via.o
obj-$(CONFIG_I2C_VIAPRO)		+= i2c-viapro.o
obj-$(CONFIG_I2C_VOODOO3)		+= i2c-voodoo3.o
obj-$(CONFIG_ARCH_EZX_E680)		+= boomer.o
# This is needed for automatic patch generation: sensors code ends here

include $(TOPDIR)/Rules.make

