/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2005 - Motorola
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 * Motorola 2005-Jun-16 - Rewrote the software for PCAP
 * Motorola 2005-Nov-26 - Finalized the software.
 *
 */

#ifndef __EMU_ATLAS_H__
#define __EMU_ATLAS_H__

#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
#include <asm/arch/pxa-regs.h>
#else
#include <asm/arch/gpio.h>
#endif

#include <asm/io.h>
#include "emu.h"

/*!
 * @file emu_atlas.h
 *
 * @brief This file contains defines, macros and prototypes needed specifically
 * for the EMU One Chip/ATLAS hardware combination.
 *
 * @ingroup poweric_emu
 */

#define EMU_INT_ID          POWER_IC_EVENT_ATLAS_IDI
#define EMU_SENSE_ID_FLOAT  POWER_IC_EVENT_ATLAS_ID_FLOAT
#define EMU_SENSE_ID_GND    POWER_IC_EVENT_ATLAS_ID_GROUND
#define EMU_INT_VBUS        POWER_IC_EVENT_ATLAS_USBI
#define EMU_INT_SE1         POWER_IC_EVENT_ATLAS_SE1I
#define EMU_INT_VBUS_OV     POWER_IC_EVENT_ATLAS_CHGOVI
#define EMU_INT_CHRG_CURR   POWER_IC_EVENT_ATLAS_CHGCURRI

/* Define the bit masks for the headset send/end key work around */
#define FSENB_MASK     0x00000001
#define USBPU_MASK     0x00000004
#define USBXCVREN_MASK 0x00001000

/* 10 bit A2D thresholds for VBUS */
#define EMU_VBUS_5VOLTS     0x074
#define EMU_VBUS_4_5VOLTS   0x061

/* 10 bit A2D ranges for readings on the ID line */
#define EMU_ID_COUNTS_OPEN_MAX          0x3FF
#define EMU_ID_COUNTS_OPEN_MIN          0x3D9
#define EMU_ID_COUNTS_440K_MAX          0x3D8
#define EMU_ID_COUNTS_440K_MIN          0x1CD
#define EMU_ID_COUNTS_200K_MAX          0x1CC
#define EMU_ID_COUNTS_200K_MIN          0x0F8
#define EMU_ID_COUNTS_100K_MAX          0x0F7
#define EMU_ID_COUNTS_100K_MIN          0x07E
#define EMU_ID_COUNTS_10K_MAX           0x07A
#define EMU_ID_COUNTS_10K_MIN           0x013
#define EMU_ID_COUNTS_1K_MAX            0x02B
#define EMU_ID_COUNTS_1K_MIN            0x000

#define EMU_ID_COUNTS__NUM      6

typedef int EMU_ID_COUNTS_T;

/* EMU One Chip interrupt macros */
#define EMU_SUBSCRIBE_VBUS_INTERRUPT(handler) \
     power_ic_event_subscribe(EMU_INT_VBUS, (handler))

#define EMU_SUBSCRIBE_ID_INTERRUPT(handler) \
     power_ic_event_subscribe(EMU_INT_ID, (handler))

#define EMU_SUBSCRIBE_SE1_INTERRUPT(handler) \
     power_ic_event_subscribe(EMU_INT_SE1, (handler))

#define EMU_SUBSCRIBE_CHRGCURR_INTERRUPT(handler) \
     power_ic_event_subscribe(EMU_INT_CHRG_CURR, (handler))

/* EMU One Chip interrupt sense macros */
#define EMU_GET_VBUS_SENSE() \
({\
     int value;\
     \
     if (0 > power_ic_get_reg_value(POWER_IC_REG_ATLAS_INT_SENSE_0, 16, &value, 3))\
     {\
        value = 0;\
     }\
     value;\
}) 
     
/* EMU ATLAS CHARGER 0 macros */
#define EMU_SET_VBUS_5K_PULL_DOWN(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_ATLAS_CHARGER_0, 19, (on_off))

#define EMU_SET_REVERSE_MODE(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_ATLAS_CHARGER_0, 13, (on_off))


#define EMU_GET_REVERSE_MODE() \
({\
     int value;\
     \
     if ( 0 > power_ic_get_reg_value(POWER_IC_REG_ATLAS_CHARGER_0, 13, &value, 1))\
     {\
        value = -1;\
     }\
     value;\
}) 

/* EMU One Chip Conn Control macros */
#define EMU_SET_ID_PULL_UP_CONTROL(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_ATLAS_USB_0, 22, (on_off))

#define EMU_SET_ID_PULL_DOWN(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_ATLAS_USB_0, 20, (on_off))

#define EMU_SET_EMU_CONN_MODE(conn_mode) \
     power_ic_set_reg_value(POWER_IC_REG_ATLAS_USB_0, 14, (conn_mode), 3)

#define EMU_SET_XCVR_ENABLE(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_ATLAS_USB_0, 12, (on_off))

#define EMU_SET_VBUS_70K_PULL_DOWN_B(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_ATLAS_USB_0, 6, (on_off))

#define EMU_SET_DPLUS_150K_PULL_UP(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_ATLAS_USB_0, 5, (on_off))
     
#define EMU_SET_USB_SUSPEND(on_off) \
    power_ic_set_reg_bit(POWER_IC_REG_ATLAS_USB_0, 1, (on_off))

#define EMU_SET_VUSB_INPUT_SOURCE(input_source) \
    power_ic_set_reg_value(POWER_IC_REG_ATLAS_CHARGE_USB_1, 0, (input_source), 2)

#define EMU_SET_VUSB_OUTPUT_VOLTAGE(voltage) \
    power_ic_set_reg_bit(POWER_IC_REG_ATLAS_CHARGE_USB_1, 2, (voltage))

#define EMU_SET_VUSB_REGULATOR_STATE(on_off) \
    power_ic_set_reg_bit(POWER_IC_REG_ATLAS_CHARGE_USB_1, 3, (on_off))

#define EMU_SET_TRANSCEIVER_STATE(on_off) \
    power_ic_set_reg_bit(POWER_IC_REG_ATLAS_USB_0, 12, (on_off))
    
#define EMU_SET_HEADSET_PULL_UP(pullup) \
    power_ic_set_reg_bit(POWER_IC_REG_ATLAS_CHARGE_USB_1, 8, (pullup))

#define EMU_SET_HS_SEND_END_REGS(set) \  
   power_ic_set_reg_mask(POWER_IC_REG_ATLAS_USB_0, \
                         FSENB_MASK|USBPU_MASK|USBXCVREN_MASK,          \
                         (set)? FSENB_MASK|USBPU_MASK|USBXCVREN_MASK : 0)
        
#endif /* __EMU_ATLAS_H__ */
