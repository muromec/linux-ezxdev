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
    O_TARGET     := power_ic.o

    subdir-$(CONFIG_MOT_POWER_IC)       += core module

    obj-$(CONFIG_MOT_POWER_IC)       += core/power_ic_core.o module/power_ic_module.o
    
    include $(TOPDIR)/Rules.make
endif

ifeq ($(CONFIG_ARCH_SCMA11),y)
    obj-y       := core/ module/
endif

fastdep:
