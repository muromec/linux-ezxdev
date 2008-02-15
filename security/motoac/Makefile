#==================================================================================
                                                                               
#General Description: Makefile for the Motorola Access Control Module.
 
#==================================================================================
 
#   Copyright (C) 2005 - Motorola
 
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License version 2 as
#   published by the Free Software Foundation.
 
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
 
#  You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software
#   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  
#Revision History:
#   2005-Jan - Motorola - created by Zhu Zhifu for Security Access Control module
#==================================================================================

O_TARGET := vmlinux-obj.o

export-objs     := ac_policy.o ac_hooks.o ac_syscalls.o

obj-$(CONFIG_SECURITY_MOTOAC)	+= motoac.o

motoac-objs	:= ac_policy.o ac_hooks.o ac_syscalls.o

include $(TOPDIR)/Rules.make

# Link rules for multi-part drivers.

#%.o:%.c 
#	$(CC) $(CFLAGS) -o $@ -c $< 	

motoac.o: $(motoac-objs)
	$(LD) -r -o $@ $(motoac-objs)
