#
# Makefile for the memory technology device drivers.
#


export-objs	:= mtdcore.o mtdpart.o redboot.o cmdlinepart.o afs.o \
		   mtdconcat.o mtd_blkdevs.o

O_TARGET	:= mtdlink.o

list-multi	:= nftl.o inftl.o mtd_blkdevs.o

mod-subdirs	:=
subdir-y	:= chips maps devices nand
subdir-m	:= $(subdir-y)

# Core functionality.
mtd-y				:= mtdcore.o
mtd-$(CONFIG_MTD_PARTITIONS)	+= mtdpart.o
obj-$(CONFIG_MTD)		+= $(mtd-y)

obj-$(CONFIG_MTD_CONCAT)	+= mtdconcat.o
obj-$(CONFIG_MTD_REDBOOT_PARTS) += redboot.o
obj-$(CONFIG_MTD_CMDLINE_PARTS) += cmdlinepart.o
obj-$(CONFIG_MTD_AFS_PARTS)	+= afs.o

# 'Users' - code which presents functionality to userspace.
obj-$(CONFIG_MTD_CHAR)		+= mtdchar.o
obj-$(CONFIG_MTD_BLOCK)		+= mtdblock.o mtd_blkdevs.o
obj-$(CONFIG_MTD_BLOCK_RO)	+= mtdblock_ro.o mtd_blkdevs.o
obj-$(CONFIG_FTL)		+= ftl.o mtd_blkdevs.o
obj-$(CONFIG_NFTL)		+= nftl.o mtd_blkdevs.o
obj-$(CONFIG_INFTL)		+= inftl.o mtd_blkdevs.o

nftl-objs		:= nftlcore.o nftlmount.o
inftl-objs		:= inftlcore.o inftlmount.o

obj-y		+= chips/chipslink.o maps/mapslink.o \
		   devices/devlink.o nand/nandlink.o

include $(TOPDIR)/Rules.make

nftl.o: $(nftl-objs)
	$(LD) -r -o $@ $(nftl-objs)

inftl.o: $(inftl-objs)
	$(LD) -r -o $@ $(inftl-objs)

