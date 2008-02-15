#
# Makefile for the Intel PXA250/210 PCMCIA driver
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).

O_TARGET := pxa_cs.o
list-multi := pxa_cs_mod.o

pxa_cs_mod-objs-y				:= pxa.o
pxa_cs_mod-objs-$(CONFIG_ARCH_LUBBOCK)		+= lubbock.o
pxa_cs_mod-objs-$(CONFIG_ARCH_MAINSTONE)	+= mainstone.o
pxa_cs_mod-objs-$(CONFIG_ARCH_PXA_IDP)		+= pxa_idp.o
pxa_cs_mod-objs-$(CONFIG_ARCH_PXA_CERF)		+= ../sa1100_cerf.o
pxa_cs_mod-objs					:= $(pxa_cs_mod-objs-y)

obj-$(CONFIG_PCMCIA_PXA)			+= pxa_cs_mod.o

include $(TOPDIR)/Rules.make

pxa_cs_mod.o: $(pxa_cs_mod-objs)
	$(LD) -r -o $@ $(pxa_cs_mod-objs)
