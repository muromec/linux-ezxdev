# $Id: Rules.make,v 1.1 2003/01/09 21:35:46 ahennessy Exp $

# Replace mtdchar.o with mtdchar-compat.o

obj-y		:= $(subst mtdchar.o,mtdchar-compat.o,$(obj-y))
obj-m		:= $(subst mtdchar.o,mtdchar-compat.o,$(obj-m))

# Makefile fragments for older kernels.

multi-y		:= $(filter $(list-multi), $(obj-y))
multi-m		:= $(filter $(list-multi), $(obj-m))
int-y		:= $(sort $(foreach m, $(multi-y), $($(basename $(m))-objs)))
int-m		:= $(sort $(foreach m, $(multi-m), $($(basename $(m))-objs)))

# Files that are both resident and modular: remove from modular.

obj-m		:= $(filter-out $(obj-y), $(obj-m))
int-m		:= $(filter-out $(int-y), $(int-m))

# Take multi-part drivers out of obj-y and put components in.

obj-y		:= $(filter-out $(list-multi), $(obj-y)) $(int-y)

# Translate to Rules.make lists.

O_OBJS		:= $(filter-out $(export-objs), $(obj-y))
OX_OBJS		:= $(filter     $(export-objs), $(obj-y))
M_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-m)))
MX_OBJS		:= $(sort $(filter     $(export-objs), $(obj-m)))

SUB_DIRS	:= $(subdir-y)
MOD_SUB_DIRS	:= $(subdir-m)

TOPDIR := $(OLDTOPDIR)
include $(TOPDIR)/Rules.make
