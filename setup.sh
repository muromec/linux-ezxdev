#! /bin/sh
# 
#================================================================================
#                                                                               
# Module Name:  setup.sh
#
# General Description: create link file for arm/pxa platform
#                      setup configuration  for dedicated product
#
#================================================================================
#
#  Copyright (C) 2003, 2005 Motorola Inc.
# 
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
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
# Revision History:
#                            Modification     Tracking
# Author                 Date          Number     Description of Changes
# ----------------   ------------    ----------   -------------------------------
# Zhou Jingze        11/22/2003                    Created   
# Ru Yi              09/28/2005                    Added paniclog  
# Ru Yi              12/20/2005                    Modified panic cmdline value

cd ./include
if [ -h ./asm ]; then rm -rf ./asm ; fi
ln -s asm-arm asm

cd ./asm-arm
if [ -h ./proc ]; then rm -rf ./proc  ; fi
if [ -h ./arch ]; then rm -rf ./arch  ; fi
ln -s proc-armv proc
ln -s arch-pxa arch
cd ..
cd ..

case "$1" in
    PRODUCT=E680)
	make ezx-680_config
	;;
    PRODUCT=E680I)
	make ezx-680i_config
	;;
    PRODUCT=A780)
        make ezx-780_config
	;;
    PRODUCT=SUMATRA)
        make ezx-sumatra_config
	;;
    PRODUCT=SUMATRAI)
        make ezx-sumatrai_config
	;;
    PRODUCT=HAINAN)
        make ezx-hainan_config
	;;
    PRODUCT=BARBADOS)
        make ezx-barbados_config
    ;;
    PRODUCT=MARTINIQUE)
        make ezx-martinique_config
    ;;

    ?)
        echo "usage:  ./setup.sh [PRODUCT=E680|PRODUCT=E680I|PRODUCT=A780|?]"
        echo "param:  PRODUCT=E680	(E680)"
        echo "        PRODUCT=E680I	(E680I)"
        echo "        PRODUCT=A780	(A780)"
        echo "        PRODUCT=SUMATRA	(SUMATRA)"
        echo "        PRODUCT=SUMATRAI	(SUMATRAI)"
        echo "        PRODUCT=HAINAN	(HAINAN)"
        echo "        PRODUCT=BARBADOS  (Barbados)"
        echo "        PRODUCT=MARTINIQUE  (Martinique)"
    	echo "        ?		(print this help)"
        exit 1
        ;;
    *)
        echo "usage:  ./setup.sh [PRODUCT=E680|PRODUCT=E680I|PRODUCT=A780|PRODUCT=SUMATRA|PRODUCT=SUMATRAI|PRODUCT=HAINAN|PRODUCT=BARBADOS|PRODUCT=MARTINIQUE| ?]"
	exit 1
	;;
esac

make oldconfig

case "$2" in
    LCD=18BPP)
        sed -e 's/CONFIG_FB_PXA_16BPP=.$*/# CONFIG_FB_PXA_16BPP is not set/' .config > .tmpconfig1
	sed -e 's/# CONFIG_FB_PXA_18BPP is not set.*$/CONFIG_FB_PXA_18BPP=y/' .tmpconfig1 > .tmpconfig
	mv -f .tmpconfig .config

	cd include/linux
	sed -e 's/#define CONFIG_FB_PXA_16BPP 1.*$/#undef CONFIG_FB_PXA_16BPP/' autoconf.h > .tmp1
        sed -e 's/#undef  CONFIG_FB_PXA_18BPP.*$/#define CONFIG_FB_PXA_18BPP 1/' .tmp1 > .tmp
        mv -f .tmp autoconf.h
	cd ../..
	;;

    console=2)
	sed -e "s/CONFIG_CMDLINE=.*$/CONFIG_CMDLINE=\"root=3e00 rootfstype=cramfs paniclog=on ip=off console=ttyS2,115200n8\"/" .config > .tmpconfig
	mv -f .tmpconfig .config
	cd include/linux
	sed -e "s/CONFIG_CMDLINE.*$/CONFIG_CMDLINE \"root=3e00 rootfstype=cramfs paniclog=on ip=off console=ttyS2,115200n8\"/" autoconf.h > .tmp
	mv -f .tmp autoconf.h
	cd ../..
	;;

    console=nfs)
	sed -e "s/CONFIG_CMDLINE=.*$/CONFIG_CMDLINE=\"root=\/dev\/nfs nfsroot=192.168.1.1:\/download\/root paniclog=on ip=192.168.1.2:192.168.1.1:::usbb0: usbd=net\"/" .config > .tmpconfig
	mv -f .tmpconfig .config
	cd include/linux
	sed -e "s/CONFIG_CMDLINE.*$/CONFIG_CMDLINE \"root=\/dev\/nfs nfsroot=192.168.1.1:\/download\/root paniclog=on ip=192.168.1.2:192.168.1.1:::usbb0: usbd=net\"/" autoconf.h > .tmp
	mv -f .tmp autoconf.h
	cd ../..
	;;

    console=nfs_con2)
	sed -e "s/CONFIG_CMDLINE=.*$/CONFIG_CMDLINE=\"root=\/dev\/nfs nfsroot=192.168.1.1:\/download\/root paniclog=on ip=192.168.1.2:192.168.1.1:::usbb0: usbd=net console=ttyS2,115200n8\"/" .config > .tmpconfig
	mv -f .tmpconfig .config
	cd include/linux
	sed -e "s/CONFIG_CMDLINE.*$/CONFIG_CMDLINE \"root=\/dev\/nfs nfsroot=192.168.1.1:\/download\/root paniclog=on ip=192.168.1.2:192.168.1.1:::usbb0: usbd=net console=ttyS2,115200n8\"/" autoconf.h > .tmp
	mv -f .tmp autoconf.h
	cd ../..
	;;

    *)
	;;
esac

#make oldconfig
#sed -e 's/CONFIG_CMDLINE=.*$/CONFIG_CMDLINE="root=3e00 rw ip=off CONSOLE=\/dev\/null"/' .config > .tmpconfig
#mv -f .tmpconfig .config
#cd include/linux
#sed -e 's/CONFIG_CMDLINE.*$/CONFIG_CMDLINE "root=3e00 rw ip=off CONSOLE=\/dev\/null"/' autoconf.h > .tmp
#mv -f .tmp autoconf.h
#cd ../..

make dep 

