# Makefile for the Linux sound card driver
#
# Copyright (C) 2005 Motorola Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# modified by w20076, for EZX platform
#

# All of the (potential) objects that export symbols.
# This list comes from 'grep -l EXPORT_SYMBOL *.[hc]'.

export-objs	:=  pcap_audio.o ezx-common.o product_dep.o ezx-dsp.o ezx-audio.o \
		    ezx-dsp16.o ezx-nssp.o ezx-phone.o ezx-vib.o fmradio.o

obj-$(CONFIG_SOUND_EZX_BASE) += pcap_audio.o ezx-common.o product_dep.o ezx-dsp.o ezx-audio.o \
				ezx-dsp16.o ezx-nssp.o ezx-phone.o ezx-vib.o
obj-$(CONFIG_SOUND_EZX_FMRADIO) += fmradio.o

O_TARGET	:= ezxbase_snd_drvs.o

include $(TOPDIR)/Rules.make

