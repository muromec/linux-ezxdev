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
 * Motorola 2005-Feb-28 - Rewrote the software for PCAP.
 * Motorola 2005-Jun-19 - Added the EMU 1 Interrupt sense macro
 * Motorola 2005-Nov-22 - Finalize the software.
 *
 */

#ifndef __EMU_EOC_H__
#define __EMU_EOC_H__

#include <asm/arch/pxa-regs.h>
#include <asm/io.h>
#include "emu.h"

/*!
 * @file emu_eoc.h
 *
 * @brief This file contains defines, macros and prototypes needed specifically
 * for the EMU One Chip/PCAP 2 hardware combination.
 *
 * @ingroup poweric_emu
 */

#define EMU_INT_ID         POWER_IC_EVENT_EOC_ID
#define EMU_SENSE_ID_FLOAT POWER_IC_EVENT_EOC_ID_FLOAT
#define EMU_SENSE_ID_GND   POWER_IC_EVENT_EOC_ID_GROUND
#define EMU_INT_VBUS       POWER_IC_EVENT_EOC_VBUSDET
#define EMU_INT_SE1        POWER_IC_EVENT_EOC_SE1_DET
#define EMU_INT_VBUS_OV    POWER_IC_EVENT_EOC_VBUSOV
#define EMU_INT_CHRG_CURR  POWER_IC_EVENT_EOC_CHRG_CURR

/* Define the bit masks for the headset send/end key work around */
#define FSENB_MASK     0x00000001
#define USBPU_MASK     0x00000004
#define USBXCVREN_MASK 0x00001000

/* 10 bit A2D thresholds for VBUS */
#define EMU_VBUS_5VOLTS   0x180
#define EMU_VBUS_4_5VOLTS 0x100

/* 10 bit A2D ranges for readings on the ID line */
#define EMU_ID_COUNTS_OPEN_MAX 0x3FF
#define EMU_ID_COUNTS_OPEN_MIN 0x3D6
#define EMU_ID_COUNTS_440K_MAX 0x3D5
#define EMU_ID_COUNTS_440K_MIN 0x269
#define EMU_ID_COUNTS_200K_MAX 0x268
#define EMU_ID_COUNTS_200K_MIN 0x123
#define EMU_ID_COUNTS_100K_MAX 0x122
#define EMU_ID_COUNTS_100K_MIN 0x07F
#define EMU_ID_COUNTS_10K_MAX  0x07E
#define EMU_ID_COUNTS_10K_MIN  0x020
#define EMU_ID_COUNTS_1K_MAX   0x01F
#define EMU_ID_COUNTS_1K_MIN   0x006

#define EMU_ID_COUNTS__NUM     6

/* define GPIO for stereo emu headset */
#define GPIO99_ST_HST  99

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
     if (0 > power_ic_get_reg_value(POWER_IC_REG_EOC_INT_SENSE, 1, &value, 3))\
     {\
        value = 0;\
     }\
     value;\
}) 
     
/* EMU One Chip Power Control 0 macros */
#define EMU_SET_VBUS_5K_PULL_DOWN(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_EOC_POWER_CONTROL_0, 19, (on_off))

#define EMU_SET_REVERSE_MODE(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_EOC_POWER_CONTROL_0, 13, (on_off))


#define EMU_GET_REVERSE_MODE() \
({\
     int value;\
     \
     if ( 0 > power_ic_get_reg_value(POWER_IC_REG_EOC_POWER_CONTROL_0, 13, &value, 1))\
     {\
        value = -1;\
     }\
     value;\
}) 


/* EMU One Chip Conn Control macros */
#define EMU_SET_ID_PULL_UP_CONTROL(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL, 22, (on_off))

#define EMU_SET_ID_PULL_DOWN(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL, 20, (on_off))

#define EMU_SET_EMU_CONN_MODE(conn_mode) \
     power_ic_set_reg_value(POWER_IC_REG_EOC_CONN_CONTROL, 14, (conn_mode), 3)

#define EMU_SET_XCVR_ENABLE(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL, 12, (on_off))

#define EMU_SET_VBUS_70K_PULL_DOWN_B(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL, 6, (on_off))

#define EMU_SET_DPLUS_150K_PULL_UP(on_off) \
     power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL, 5, (on_off))
     
#define EMU_SET_USB_SUSPEND(on_off) \
    power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL, 1, (on_off))

#define EMU_SET_VUSB_INPUT_SOURCE(input_source) \
    power_ic_set_reg_value(POWER_IC_REG_EOC_POWER_CONTROL_1, 0, (input_source), 2)

#define EMU_SET_VUSB_OUTPUT_VOLTAGE(voltage) \
    power_ic_set_reg_bit(POWER_IC_REG_EOC_POWER_CONTROL_1, 2, (voltage))

#define EMU_SET_VUSB_REGULATOR_STATE(on_off) \
    power_ic_set_reg_bit(POWER_IC_REG_EOC_POWER_CONTROL_1, 3, (on_off))

#define EMU_SET_TRANSCEIVER_STATE(on_off) \
    power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL, 12, (on_off))

#define EMU_CONFIG_HEADSET_MODE() \
    (set_GPIO_mode(GPIO99_ST_HST | GPIO_OUT))

#define EMU_SET_HEADSET_PULL_UP(pullup) \
({ \
    if(pullup == ENABLE) \
    { \
        set_GPIO(GPIO99_ST_HST); \
    } \
    else \
    { \
        clr_GPIO(GPIO99_ST_HST); \
    }  \
})  

#define EMU_SET_HS_SEND_END_REGS(set)\
   power_ic_set_reg_mask(POWER_IC_REG_EOC_CONN_CONTROL,\
                         FSENB_MASK|USBPU_MASK|USBXCVREN_MASK,          \
                         (set ? FSENB_MASK|USBPU_MASK|USBXCVREN_MASK : 0))
#endif /* __EMU_H__ */
