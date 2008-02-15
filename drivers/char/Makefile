#
# Makefile for the kernel character device drivers.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definitions are now inherited from the
# parent makes..
#

#
# Copyright (C) 2005 Motorola Inc.
#
# modified by w12051, for EZX platform
#

#
# This file contains the font map for the default (hardware) font
#
FONTMAPFILE = cp437.uni

O_TARGET := char.o

obj-y	 += mem.o tty_io.o n_tty.o tty_ioctl.o raw.o pty.o misc.o random.o \
            gpio_test.o

# All of the (potential) objects that export symbols.
# This list comes from 'grep -l EXPORT_SYMBOL *.[hc]'.

export-objs     :=	busmouse.o console.o keyboard.o sysrq.o \
			misc.o pty.o random.o selection.o serial.o \
			sonypi.o tty_io.o tty_ioctl.o generic_serial.o \
			au1000_gpio.o hp_psaux.o nvram.o kfi.o \
			ibm_ocp_gpio.o lcd.o h3600_backpaq_fpga.o \
			pfs168_spi.o rw4mouse.o ite_gpio.o mk712.o led.o \
			shm_vio.o shm_vio4.o shmse_keyb.o mx2ads-rtc.o \
                        led_pxa_e680.o

mod-subdirs	:=	joystick ftape drm drm-4.0 pcmcia

list-multi	:=	

KEYMAP   =defkeymap.o
KEYBD    =pc_keyb.o
CONSOLE  =console.o
SERIAL   =serial.o

ifeq ($(ARCH),s390)
  KEYMAP   =
  KEYBD    =
  CONSOLE  =
  SERIAL   =
endif

ifeq ($(ARCH),mips)
  ifneq ($(CONFIG_PC_KEYB),y)
    KEYBD    =
  endif
endif

ifeq ($(ARCH),s390x)
  KEYMAP   =
  KEYBD    =
  CONSOLE  =
  SERIAL   =
endif

ifeq ($(ARCH),xtensa)
  ifneq ($(CONFIG_PC_KEYB),y)
    KEYBD    =
  endif
endif

ifeq ($(ARCH),m68k)
   ifdef CONFIG_AMIGA
      KEYBD = amikeyb.o
   else
      ifndef CONFIG_MAC
	 KEYBD =
      endif
   endif
   SERIAL   =
endif

ifdef CONFIG_Q40
  KEYBD += q40_keyb.o
  SERIAL = serial.o
endif

ifdef CONFIG_APOLLO
  KEYBD += dn_keyb.o
endif

ifeq ($(ARCH),parisc)
   ifdef CONFIG_GSC_PS2
      KEYBD   = hp_psaux.o hp_keyb.o
   else
      KEYBD   =
   endif
   ifdef CONFIG_PDC_CONSOLE
      CONSOLE += pdc_console.o
   endif
endif

ifeq ($(ARCH),arm)
  KEYMAP     :=
  KEYBD      :=
  ifeq ($(CONFIG_PC_KEYMAP),y)
    KEYMAP   := defkeymap.o
  endif
  ifeq ($(CONFIG_PC_KEYB),y)
    KEYBD    += pc_keyb.o
  endif
  ifeq ($(CONFIG_KMI_KEYB),y)
    KEYBD    += amba_kmi_keyb.o
  endif
  ifeq ($(CONFIG_SA1111),y)
    KEYBD    += sa1111_keyb.o
  endif
  ifeq ($(CONFIG_ARCH_EDB7211),y)
    KEYBD    += edb7211_keyb.o
  endif
  ifeq ($(CONFIG_ARCH_AUTCPU12),y)
    KEYMAP   := defkeymap.o
    KEYBD    += clps711x_keyb.o
  endif
  ifeq ($(CONFIG_SA1100_GRAPHICSCLIENT),y)
    KEYMAP    = gckeymap.o
    KEYBD    += gc_keyb.o
  endif
  ifeq ($(CONFIG_SA1100_CERF_CPLD),y)
    KEYBD    += cerf_keyb.o
  endif
  ifeq ($(CONFIG_ARCH_FORTUNET),y)
    KEYMAP   := defkeymap.o
  endif
  ifeq ($(CONFIG_SA1100_H3XXX),y)
    KEYBD    += h3600_keyboard.o
  endif
  ifeq ($(CONFIG_ARCH_OMAP1510),y)
    list-multi	+= keypad.o
    keypad_objs	:= omap1510_keypad.o
    obj-$(CONFIG_OMAP1510_KEYPAD)	+= keypad.o
    list-multi	+= juno_hid.o
    juno_objs 	:= omap1510_hid.o omap1510_ps2.o omap1510_spi.o
    obj-$(CONFIG_OMAP1510_HID)	+= juno_hid.o
    obj-$(CONFIG_OMAP1510_TOUCHSCREEN)	+= omap1510_tsdev.o
  endif
  ifeq ($(CONFIG_ARCH_OMAP710),y)
    list-multi	+= keypad.o
    keypad_objs	:= omap710_keypad.o
    obj-$(CONFIG_OMAP710_KEYPAD)	+= keypad.o
  endif
endif

ifeq ($(ARCH),sh)
  ifneq ($(CONFIG_PC_KEYB),y)
  KEYMAP   =
  KEYBD    =
  CONSOLE  =
  endif
  ifeq ($(CONFIG_SH_7751_SOLUTION_ENGINE),y)
  KEYMAP   = defkeymap.o
  KEYBD    = pc_keyb.o
  CONSOLE  = console.o
  endif
  ifeq ($(CONFIG_SH_MOBILE_SOLUTION_ENGINE),y)
  KEYMAP   = defkeymap.o
  KEYBD    = scan_keyb.o shmse_keyb.o
  CONSOLE  = console.o
  endif
  ifeq ($(CONFIG_SH_HP600),y)
  KEYMAP   = defkeymap.o
  KEYBD    = scan_keyb.o hp600_keyb.o
  CONSOLE  = console.o
  endif
  ifeq ($(CONFIG_SH_DMIDA),y)
  # DMIDA does not connect the HD64465 PS/2 keyboard port
  # but we allow for USB keyboards to be plugged in.
  KEYMAP   = defkeymap.o
  KEYBD    = # hd64465_keyb.o pc_keyb.o
  CONSOLE  = console.o
  endif
  ifeq ($(CONFIG_SH_EC3104),y)
  KEYMAP   = defkeymap.o
  KEYBD    = ec3104_keyb.o
  CONSOLE  = console.o
  endif
  ifeq ($(CONFIG_SH_DREAMCAST),y)
  KEYMAP   = defkeymap.o
  KEYBD    = dc_keyb.o
  CONSOLE  = console.o
  endif
endif

ifeq ($(CONFIG_DECSTATION),y)
  KEYMAP   =
  KEYBD    =
  SERIAL   = decserial.o
endif

ifeq ($(CONFIG_BAGET_MIPS),y)
  KEYBD    =
  SERIAL   =
endif

ifeq ($(CONFIG_MIPS_LXPB20K),y)
  SERIAL   =
endif

ifeq ($(CONFIG_NINO),y)
  SERIAL   =
endif

ifeq ($(CONFIG_VRC4173),y)
  ifeq ($(CONFIG_VRC4173_PS2U),y)
    KEYBD    = vrc4173-ps2u.o
  else
    KEYBD    =
  endif
endif

ifneq ($(CONFIG_SUN_SERIAL),)
  SERIAL   =
endif

ifeq ($(CONFIG_QTRONIX_KEYBOARD),y)
  KEYBD    = qtronix.o
  KEYMAP   = qtronixmap.o
endif

ifeq ($(CONFIG_DUMMY_KEYB),y)
  KEYBD = dummy_keyb.o
endif

obj-$(CONFIG_VT) += vt.o vc_screen.o consolemap.o consolemap_deftbl.o $(CONSOLE) selection.o
obj-$(CONFIG_SERIAL) += $(SERIAL)
obj-$(CONFIG_SERIAL_HCDP) += hcdp_serial.o
obj-$(CONFIG_TS_AU1000_ADS7846) += au1000_ts.o
obj-$(CONFIG_TOUCHSCREEN_ANAKIN) += anakin_ts.o
obj-$(CONFIG_VRC4173) += vrc4173-bcu.o
obj-$(CONFIG_VRC4173_PIU) += vrc4173-piu.o

ifndef CONFIG_SUN_KEYBOARD
  obj-$(CONFIG_VT) += keyboard.o $(KEYMAP) $(KEYBD)
else
  obj-$(CONFIG_PCI) += keyboard.o $(KEYMAP)
endif

obj-$(CONFIG_HIL) += hp_keyb.o
obj-$(CONFIG_MAGIC_SYSRQ) += sysrq.o
obj-$(CONFIG_ATARI_DSP56K) += dsp56k.o
obj-$(CONFIG_ROCKETPORT) += rocket.o
obj-$(CONFIG_MOXA_SMARTIO) += mxser.o
obj-$(CONFIG_MOXA_INTELLIO) += moxa.o
obj-$(CONFIG_DIGI) += pcxx.o
obj-$(CONFIG_DIGIEPCA) += epca.o
obj-$(CONFIG_CYCLADES) += cyclades.o
obj-$(CONFIG_STALLION) += stallion.o
obj-$(CONFIG_ISTALLION) += istallion.o
obj-$(CONFIG_SIBYTE_SB1250_DUART) += sb1250_duart.o
obj-$(CONFIG_COMPUTONE) += ip2.o ip2main.o
obj-$(CONFIG_RISCOM8) += riscom8.o
obj-$(CONFIG_ISI) += isicom.o
obj-$(CONFIG_ESPSERIAL) += esp.o
obj-$(CONFIG_SYNCLINK) += synclink.o
obj-$(CONFIG_SYNCLINKMP) += synclinkmp.o
obj-$(CONFIG_N_HDLC) += n_hdlc.o
obj-$(CONFIG_SPECIALIX) += specialix.o
obj-$(CONFIG_AMIGA_BUILTIN_SERIAL) += amiserial.o
obj-$(CONFIG_A2232) += ser_a2232.o generic_serial.o
obj-$(CONFIG_SX) += sx.o generic_serial.o
obj-$(CONFIG_RIO) += rio/rio.o generic_serial.o
obj-$(CONFIG_SH_SCI) += sh-sci.o generic_serial.o
obj-$(CONFIG_SERIAL167) += serial167.o
obj-$(CONFIG_MVME147_SCC) += generic_serial.o vme_scc.o
obj-$(CONFIG_MVME162_SCC) += generic_serial.o vme_scc.o
obj-$(CONFIG_BVME6000_SCC) += generic_serial.o vme_scc.o
obj-$(CONFIG_HVC_CONSOLE) += hvc_console.o
obj-$(CONFIG_SERIAL_TX3912) += generic_serial.o serial_tx3912.o
obj-$(CONFIG_TX4925_SIO) += generic_serial.o tx4925_sio.o
obj-$(CONFIG_TX4927_SIO) += generic_serial.o tx4927_sio.o
obj-$(CONFIG_TXX927_SERIAL) += serial_txx927.o
obj-$(CONFIG_GT64XXX_MPSC) += gt64xxx_mpsc.o

subdir-$(CONFIG_RIO) += rio
subdir-$(CONFIG_INPUT) += joystick

obj-$(CONFIG_ATIXL_BUSMOUSE) += atixlmouse.o
obj-$(CONFIG_LOGIBUSMOUSE) += logibusmouse.o
obj-$(CONFIG_PRINTER) += lp.o

ifeq ($(CONFIG_INPUT),y)
obj-y += joystick/js.o
endif

obj-$(CONFIG_MAPLE_KEYBOARD) += maple_keyb.o
obj-$(CONFIG_MAPLE_MOUSE) += maplemouse.o

obj-$(CONFIG_BUSMOUSE) += busmouse.o
obj-$(CONFIG_DTLK) += dtlk.o
obj-$(CONFIG_R3964) += n_r3964.o
obj-$(CONFIG_APPLICOM) += applicom.o
obj-$(CONFIG_MK712_MOUSE) += mk712.o
obj-$(CONFIG_SONYPI) += sonypi.o
obj-$(CONFIG_MS_BUSMOUSE) += msbusmouse.o
obj-$(CONFIG_82C710_MOUSE) += qpmouse.o
obj-$(CONFIG_AMIGAMOUSE) += amigamouse.o
obj-$(CONFIG_ATARIMOUSE) += atarimouse.o
obj-$(CONFIG_ADBMOUSE) += adbmouse.o
obj-$(CONFIG_PC110_PAD) += pc110pad.o
obj-$(CONFIG_MK712_MOUSE) += mk712.o
obj-$(CONFIG_RTC) += rtc.o
obj-$(CONFIG_EFI_RTC) += efirtc.o
obj-$(CONFIG_MIPS_RTC) += mips_rtc.o
obj-$(CONFIG_SA1100_RTC) += sa1100-rtc.o
obj-$(CONFIG_PXA_RTC) += sa1100-rtc.o
obj-$(CONFIG_X1226_RTC) += x1226-rtc.o
obj-$(CONFIG_OMAP_RTC) += omap-rtc.o
obj-$(CONFIG_RV5C387_RTC) += rv5c387-rtc.o
obj-$(CONFIG_MX2ADS_RTC) += mx2ads-rtc.o
ifeq ($(CONFIG_ALL_PPC),)
  obj-$(CONFIG_NVRAM) += nvram.o
endif
obj-$(CONFIG_TOSHIBA) += toshiba.o
obj-$(CONFIG_I8K) += i8k.o
obj-$(CONFIG_DS1620) += ds1620.o
obj-$(CONFIG_INTEL_RNG) += i810_rng.o
obj-$(CONFIG_AMD_RNG) += amd768_rng.o
obj-$(CONFIG_AMD_PM768) += amd76x_pm.o
obj-$(CONFIG_BRIQ_PANEL) += briq_panel.o

obj-$(CONFIG_ITE_GPIO) += ite_gpio.o
obj-$(CONFIG_AU1000_GPIO) += au1000_gpio.o
obj-$(CONFIG_AU1000_USB_TTY) += au1000_usbtty.o
obj-$(CONFIG_AU1000_USB_RAW) += au1000_usbraw.o
obj-$(CONFIG_COBALT_LCD) += lcd.o
obj-$(CONFIG_IBM_OCP_GPIO) += ibm_ocp_gpio.o
obj-$(CONFIG_PPC405_GPIO) += ppc405_gpio.o
obj-$(CONFIG_IBMTS_SEMTECH) += ibmts_semtech.o
obj-$(CONFIG_RWMOUSE) += rw4mouse.o

obj-$(CONFIG_QIC02_TAPE) += tpqic02.o

subdir-$(CONFIG_FTAPE) += ftape
subdir-$(CONFIG_DRM_OLD) += drm-4.0
subdir-$(CONFIG_DRM_NEW) += drm
subdir-$(CONFIG_PCMCIA) += pcmcia
subdir-$(CONFIG_AGP) += agp

ifeq ($(CONFIG_FTAPE),y)
obj-y       += ftape/ftape.o
endif

obj-$(CONFIG_H8) += h8.o
obj-$(CONFIG_PPDEV) += ppdev.o
obj-$(CONFIG_DZ) += dz.o
obj-$(CONFIG_NWBUTTON) += nwbutton.o
obj-$(CONFIG_NWFLASH) += nwflash.o
obj-$(CONFIG_SH_MOBILE_VIO) += shm_vio.o
obj-$(CONFIG_SH_MOBILE_VIO4) += shm_vio4.o

obj-$(CONFIG_ATI_XILLEON_FP) += xilleon_fp.o
obj-$(CONFIG_ATI_XILLEON_IR) += xilleon_ir.o

# Only one watchdog can succeed. We probe the hardware watchdog
# drivers first, then the softdog driver.  This means if your hardware
# watchdog dies or is 'borrowed' for some reason the software watchdog
# still gives you some cover.

obj-$(CONFIG_PCWATCHDOG) += pcwd.o
obj-$(CONFIG_ACQUIRE_WDT) += acquirewdt.o
obj-$(CONFIG_ADVANTECH_WDT) += advantechwdt.o
obj-$(CONFIG_IB700_WDT) += ib700wdt.o
obj-$(CONFIG_MIXCOMWD) += mixcomwd.o
obj-$(CONFIG_60XX_WDT) += sbc60xxwdt.o
obj-$(CONFIG_W83877F_WDT) += w83877f_wdt.o
obj-$(CONFIG_SC520_WDT) += sc520_wdt.o
obj-$(CONFIG_WDT) += wdt.o
obj-$(CONFIG_WDTPCI) += wdt_pci.o
obj-$(CONFIG_21285_WATCHDOG) += wdt285.o
obj-$(CONFIG_977_WATCHDOG) += wdt977.o
obj-$(CONFIG_I810_TCO) += i810-tco.o
obj-$(CONFIG_MACHZ_WDT) += machzwd.o
obj-$(CONFIG_SH_WDT) += shwdt.o
obj-$(CONFIG_EUROTECH_WDT) += eurotechwdt.o
obj-$(CONFIG_ALIM7101_WDT) += alim7101_wdt.o
obj-$(CONFIG_INDYDOG) += indydog.o
obj-$(CONFIG_SA1100_WATCHDOG) += sa1100_wdt.o
obj-$(CONFIG_SC1200_WDT) += sc1200wdt.o
obj-$(CONFIG_SOFT_WATCHDOG) += softdog.o
obj-$(CONFIG_PPC405_WDT)  += ppc405_wdt.o
obj-$(CONFIG_ARCTIC2_MISC) += arctic2_misc.o
obj-$(CONFIG_PXA_WATCHDOG) += sa1100_wdt.o
obj-$(CONFIG_VR41XX_WDT) += vr41xxwdt.o
obj-$(CONFIG_WAFER_WDT) += wafer5823wdt.o
obj-$(CONFIG_OMAP_WATCHDOG) += omap_wdt.o

obj-$(CONFIG_CPCI735_WATCHDOG) += wdt_cpci735.o
obj-$(CONFIG_DBMX21_WDT) += mx21-wdt.o

obj-$(CONFIG_LINUX_LED) += led.o
obj-$(CONFIG_CPCI735_LED) += led_cpci735.o
obj-$(CONFIG_PXA_E680_LED) += led_pxa_e680.o

obj-$(CONFIG_MX2ADS_SLCD) += mx2-slcdc.o
obj-$(CONFIG_MX2ADS_OWIRE) += mx21-1wire.o

mod-subdirs     +=	xilinx_gpio
subdir-$(CONFIG_XILINX_GPIO) += xilinx_gpio
obj-$(CONFIG_XILINX_GPIO) += xilinx_gpio/xilinx_gpio.o
ifeq ($(CONFIG_VIRTEX_II_PRO),y)
  ifeq ($(CONFIG_VT),y)
    ifeq ($(CONFIG_PC_KEYBOARD),y)
      subdir-$(CONFIG_VT) += xilinx_keyb
      obj-$(CONFIG_VT) += xilinx_keyb/xilinx_keyb.o
    endif
  endif
endif

subdir-$(CONFIG_MWAVE) += mwave
ifeq ($(CONFIG_MWAVE),y)
  obj-y += mwave/mwave.o
endif

obj-$(CONFIG_NEWTONKBD) += newtonkbd.o
obj-$(CONFIG_TOUCHSCREEN_H3600) += h3600_ts.o
obj-$(CONFIG_H3600_STOWAWAY) += h3600_stowaway.o
obj-$(CONFIG_H3800_MICROKBD) += h3600_microkbd.o
obj-$(CONFIG_H3600_BACKPAQ_FPGA) += h3600_backpaq_fpga.o
obj-$(CONFIG_H3600_BACKPAQ_ACCEL) += h3600_backpaq_accel.o
obj-$(CONFIG_H3600_BACKPAQ_GASGAUGE) += h3600_backpaq_gasgauge.o
obj-$(CONFIG_SA1100_LIRC) += lirc_sir.o
obj-$(CONFIG_SA1100_JORNADA56X) += jornada56x_ts.o

obj-$(CONFIG_PFS168_SPI) += pfs168_spi.o
obj-$(CONFIG_PFS168_SPI) += pfs168_spi_mux.o
obj-$(CONFIG_PFS168_MISC) += pfs168_misc.o
obj-$(CONFIG_KFI) += kfi.o
obj-$(CONFIG_KFI_STATIC_RUN) += kfistatic.o

ifeq ($(CONFIG_ARCH_ACORN),y)
mod-subdirs	+= ../acorn/char
subdir-y	+= ../acorn/char
obj-y		+= ../acorn/char/acorn-char.o
endif

obj-$(CONFIG_OMAP1510_HWERROR) += omap1510_hwerror.o
obj-$(CONFIG_INNOVATOR_KEYPAD) += innovator-keypad.o
obj-$(CONFIG_INNOVATOR_PS2) += innovator_ps2.o
obj-$(CONFIG_OMAP730_KEYPAD) += omap730-keypad.o

obj-$(CONFIG_MAINSTONE_KEYPAD) += mstone_keypad.o
obj-$(CONFIG_MX2ADS_KEYPAD) += mx21-keyp.o

obj-$(CONFIG_MX2_TOUCHSCREEN) += mx2ads-spi.o
obj-$(CONFIG_MX2ADS_PWM) += mx2ads-pwm.o

obj-$(CONFIG_BULVERDE_SRAM_DEV) += sram.o

include $(TOPDIR)/Rules.make

fastdep:

conmakehash: conmakehash.c
	$(HOSTCC) $(HOSTCFLAGS) -o conmakehash conmakehash.c

consolemap_deftbl.c: $(FONTMAPFILE) conmakehash
	./conmakehash $(FONTMAPFILE) > consolemap_deftbl.c

consolemap_deftbl.o: consolemap_deftbl.c $(TOPDIR)/include/linux/types.h

.DELETE_ON_ERROR:

defkeymap.c: defkeymap.map
ifneq ($(shell uname), Linux)
	touch $@
else
	set -e ; loadkeys --mktable $< | sed -e 's/^static *//' > $@
endif

qtronixmap.c: qtronixmap.map
ifneq ($(shell uname), Linux)
	touch $@
else
	set -e ; loadkeys --mktable $< | sed -e 's/^static *//' > $@
endif

kfistatic.c: kfistatic.conf
	$(TOPDIR)/scripts/mkkfirun.pl $< > $@

# OMAP HID support:
juno_hid.o: $(juno_objs)
	$(LD) -r -o $@ $(juno_objs)

keypad.c: keypad.o

keypad.o: $(keypad_objs)
	$(LD) -r -o $@ $(keypad_objs)
