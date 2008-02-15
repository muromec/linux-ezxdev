#  Copyright (C) 2005 - Motorola
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License version 2 as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#

#
# Makefile for the power IC drivers.
#

ifeq ($(CONFIG_CPU_BULVERDE),y)
    O_TARGET := power_ic_core.o

    export-objs := \
        external.o \
        event.o 

    obj-$(CONFIG_MOT_POWER_IC) := \
        core.o \
        event.o \
        external.o \
        thread.o

    obj-$(CONFIG_MOT_POWER_IC_PCAP2) += \
        fl_register.o \
        eoc_register.o \
        pcap_register.o \
        pxa_ssp.o \
        spi_main.o \
        spi_user.o

    obj-$(CONFIG_MOT_POWER_IC_ATLAS) += \
        atlas_register.o

    include $(TOPDIR)/Rules.make
endif

ifeq ($(CONFIG_ARCH_SCMA11),y)    
    obj-$(CONFIG_MOT_POWER_IC) := \
        core.o \
        external.o  \
        event.o \
        thread.o

    obj-$(CONFIG_MOT_POWER_IC_ATLAS) += \
        atlas_register.o
endif

fastdep:
