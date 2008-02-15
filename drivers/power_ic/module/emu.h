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
 * Motorola 2005-Jun-16 - Added Multiple GPIO macros.
 * Motorola 2005-Dec-06 - Added Argon support
 *
 */

#ifndef __EMU_H__
#define __EMU_H__

/*!
 * @file emu.h
 *
 * @brief This file contains defines, macros and prototypes for the EMU
 * detection state machine.
 *
 * @ingroup poweric_emu
 */ 

#include <linux/init.h>
#include <linux/moto_accy.h>
#include <linux/power_ic.h>

#ifdef CONFIG_MOT_POWER_IC_PCAP2
#include "emu_eoc.h"
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#include "emu_atlas.h"
#endif

typedef enum
{
    FALSE = 0,
    DISABLE = FALSE,
    UNMASK = DISABLE,
    TRUE,
    ENABLE = TRUE,
    MASK = ENABLE
} BOOL;

#define EMU_A2D_ID   POWER_IC_ATOD_CHANNEL_CHARGER_ID
#define EMU_A2D_VBUS POWER_IC_ATOD_CHANNEL_MOBPORTB

/* defines used by polling_interval */
/* 1mS = (HZ/1000) */
#define EMU_POLL_UNPOWERED_SIHF_DELAY (5000 * HZ) / 1000
#define EMU_POLL_SPD_REMOVAL_DELAY    ( 500 * HZ) / 1000
#define EMU_POLL_REV_MODE_DELAY       ( 200 * HZ) / 1000
#define EMU_POLL_DFLT_DEB_DELAY       ( 100 * HZ) / 1000
#define EMU_POLL_SETTLE_DELAY         (  50 * HZ) / 1000
#define EMU_POLL_HEADSET_DELAY        (  20 * HZ) / 1000
#define EMU_POLL_FAST_POLL_DELAY      (  10 * HZ) / 1000
#define EMU_POLL_MIN_DELAY            2 /*the smallest possible delay in jiffies */
#define EMU_POLL_WAIT_EVENT          -1
#define EMU_POLL_CONTINUE            -2

/* debounce counts */
#define EMU_POLL_DEB_DONE_CNT           3
#define EMU_POLL_DEB_UNPOWERED_SIHF_CNT 50

/* GPIO macros */
#ifdef CONFIG_CPU_BULVERDE
#define EMU_GET_D_PLUS() \
    ((GPLR(GPIO40_FFDTR) & GPIO_bit(GPIO40_FFDTR)) ? \
      EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED)

#define EMU_GET_D_MINUS() \
    ((GPLR(GPIO53_MMCCLK) & GPIO_bit(GPIO53_MMCCLK)) ? \
      EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED)

#elif defined(CONFIG_ARCH_SCMA11)
#define EMU_GET_D_PLUS() \
({\
    ((gpio_get_data(GPIO_AP_C_PORT, 16)) ? \
        EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED); \
})

#define EMU_GET_D_MINUS() \
({\
    ((gpio_get_data(GPIO_AP_C_PORT, 17)) ? \
      EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED); \
})
#endif

/* Defines all available EMU detection states */
typedef enum
{
    EMU_STATE_DISCONNECTED,                     /*0*/
    
    EMU_STATE_CONNECTING__DEBOUNCE_DEV_TYPE,    /*1*/
    EMU_STATE_CONNECTING__SPD_POLL,             /*2*/
    EMU_STATE_CONNECTING__UNPOWERED_SIHF,       /*3*/
    EMU_STATE_CONNECTING__PPD_VALIDATE,         /*4*/
    EMU_STATE_CONNECTING__PRE_PPD_IDENTIFY,     /*5*/
    EMU_STATE_CONNECTING__PPD_IDENTIFY,         /*6*/

    EMU_STATE_CONNECTED__POLL_SPD_REMOVAL,      /*7*/
    EMU_STATE_CONNECTED__FACTORY,               /*8*/
    EMU_STATE_CONNECTED__INVALID = EMU_STATE_CONNECTED__FACTORY,
    EMU_STATE_CONNECTED__NOT_SUPPORTED = EMU_STATE_CONNECTED__FACTORY,
    EMU_STATE_CONNECTED__HEADSET,               /*9*/
    
    EMU_STATE_DISCONNECTING,                    /*10*/

    EMU_STATE__NUM_STATES
} EMU_STATE_T;

/* Defines all possible EMU device types */
typedef enum
{
    EMU_DEV_TYPE_NONE,         /* no device connected*/
    EMU_DEV_TYPE_SPD,
    EMU_DEV_TYPE_PPD,
    EMU_DEV_TYPE_NOT_SUPPORTED,
    EMU_DEV_TYPE_INVALID,

    EMU_DEV_TYPE__NUM
} EMU_DEV_TYPE_T;

/* Defines the possible EMU bus signals */
typedef enum
{
    EMU_BUS_SIGNAL_DP_DM,
    EMU_BUS_SIGNAL_DPLUS,
    EMU_BUS_SIGNAL_DMINUS,
    EMU_BUS_SIGNAL_SE1,
    EMU_BUS_SIGNAL_VBUS,
    EMU_BUS_SIGNAL_ID,
    EMU_BUS_SIGNAL_ID_FLOAT,
    EMU_BUS_SIGNAL_ID_GROUND,

    EMU_BUS_SIGNAL__NUM
} EMU_BUS_SIGNAL_T;

/* Defines the possible EMU bus signal states */
typedef enum
{
    EMU_BUS_SIGNAL_STATE_DISABLED = FALSE,
    EMU_BUS_SIGNAL_STATE_ENABLED = TRUE,
    EMU_BUS_SIGNAL_STATE_UNKNOWN,          /* just used initially */
    EMU_BUS_SIGNAL_STATE_ID_FLOATING,
    EMU_BUS_SIGNAL_STATE_ID_GROUNDED, 
    EMU_BUS_SIGNAL_STATE_ID_RESISTOR,
    EMU_BUS_SIGNAL_STATE_ID_FACTORY,
    EMU_BUS_SIGNAL_STATE_DP_DM_00,
    EMU_BUS_SIGNAL_STATE_DP_DM_01,
    EMU_BUS_SIGNAL_STATE_DP_DM_10,
    EMU_BUS_SIGNAL_STATE_DP_DM_11,
    EMU_BUS_SIGNAL_STATE_VBUS0V8, /* 0.8V <= VBUS < 2.0V */
    EMU_BUS_SIGNAL_STATE_VBUS2V0, /* 2.0V <= VBUS < 4.4V */
    EMU_BUS_SIGNAL_STATE_VBUS4V4, /* 4.4V <= VBUS < 7.0V */

    EMU_BUS_SIGNAL_STATE__NUM
} EMU_BUS_SIGNAL_STATE_T;

/* Defines the possible ID resistor values */
typedef enum
{
    EMU_ID_RESISTOR_OPEN = 0,
    EMU_ID_RESISTOR_FACTORY,
    EMU_ID_RESISTOR_GROUND,
    EMU_ID_RESISTOR_440K,
    EMU_ID_RESISTOR_200K,
    EMU_ID_RESISTOR_100K,
    EMU_ID_RESISTOR_10K,
    EMU_ID_RESISTOR_1K,

    /* This is used to break this enum into SE1 resistor devices
       and non SE1 resistor devices */
    EMU_ID_RESISTOR_SE1,
                            
    EMU_ID_RESISTOR_OPEN_SE1 = EMU_ID_RESISTOR_SE1,
    EMU_ID_RESISTOR_FACTORY_SE1,
    EMU_ID_RESISTOR_GROUND_SE1,
    EMU_ID_RESISTOR_440K_SE1,
    EMU_ID_RESISTOR_200K_SE1,
    EMU_ID_RESISTOR_100K_SE1,
    EMU_ID_RESISTOR_10K_SE1,
    EMU_ID_RESISTOR_1K_SE1,
    
    EMU_ID_RESISTOR__NUM
} EMU_ID_RESISTOR_T;

/* Defines the possible CONN modes, this table is tied to
   hardware */
typedef enum
{
    EMU_CONN_MODE_USB,             /*000*/
    EMU_CONN_MODE_UART1,           /*001*/
    EMU_CONN_MODE_UART2,           /*010*/
    EMU_CONN_MODE_RESERVED,        /*011*/
    EMU_CONN_MODE_MONO_AUDIO,      /*100*/
    EMU_CONN_MODE_STEREO_AUDIO,    /*101*/
    EMU_CONN_MODE_LOOPBACK_RIGHT,  /*110*/
    EMU_CONN_MODE_LOOPBACK_LEFT,   /*111*/

    EMU_CONN_MODE__NUM
} EMU_CONN_MODE_T;

typedef enum
{
    EMU_VREG_IN_VINBUS,
    EMU_VREG_IN_VBUS,
    EMU_VREG_IN_BPLUS,
    EMU_VREG_IN_VBUS2
} EMU_VREG_IN_T;

typedef enum
{
    EMU_VREG_OUT_2_775V,
    EMU_VREG_OUT_3_3V
} EMU_VREG_OUT_T;

/* Defines the needed transceiver states */
typedef enum
{
    EMU_XCVR_OFF,
    EMU_XCVR_PPD_DETECT,
    EMU_XCVR_SPD_DETECT,
    EMU_XCVR_PPD,
    EMU_XCVR_PPD_AUDIO,
    EMU_XCVR_SPD,
    EMU_XCVR_SPD_AUDIO,
    EMU_XCVR_USB_HOST,
    EMU_XCVR_FACTORY_MODE,
    
    EMU_XCVR__NUM
} EMU_XCVR_T;


/************************** DISCONNECTED STATE FUNTIONS **************************/
void disconnected_enter(EMU_STATE_T prev_state, int *polling_interval);
EMU_STATE_T disconnected_handler(int *polling_interval);

/************************** CONNECTING STATE FUNTIONS ****************************/
void debounce_dev_type_enter(EMU_STATE_T prev_state, int *polling_interval);
EMU_STATE_T debounce_dev_type_handler(int *polling_interval);
void spd_delay_enter(EMU_STATE_T prev_state, int *polling_interval);
EMU_STATE_T spd_delay_handler(int *polling_interval);
void unpowered_sihf_enter(EMU_STATE_T prev_state,int *polling_interval);
EMU_STATE_T unpowered_sihf_handler(int *polling_interval);
void ppd_validate_enter(EMU_STATE_T prev_state, int *polling_interval);
EMU_STATE_T ppd_validate_handler(int *polling_interval);
void pre_ppd_identify_enter(EMU_STATE_T prev_state, int *polling_interval);
EMU_STATE_T pre_ppd_identify_handler(int *polling_interval);
void ppd_identify_enter(EMU_STATE_T prev_state, int *polling_interval);
EMU_STATE_T ppd_identify_handler(int *polling_interval);

/************************** CONNECTED STATE FUNTIONS *****************************/
void device_config_enter(EMU_STATE_T prev_state, int *polling_interval);
EMU_STATE_T poll_spd_removal_handler(int *polling_interval);
EMU_STATE_T factory_handler(int *polling_interval);
EMU_STATE_T headset_handler(int *polling_interval);

/************************** DISCONNECTING STATE FUNTIONS *************************/
void disconnecting_enter(EMU_STATE_T prev_state, int *polling_interval);
EMU_STATE_T disconnecting_handler(int *polling_interval);
void disconnecting_exit(EMU_STATE_T next_state, int *polling_interval);


/************************** STATE MACHINE FUNTIONS *******************************/
void __init emu_init(void);
     
/************************** UTILITY FUNTIONS *************************************/
BOOL is_device_type_debounced(EMU_DEV_TYPE_T *device_type);
BOOL is_bus_state_debounced(EMU_BUS_SIGNAL_T signal,
                            EMU_BUS_SIGNAL_STATE_T *bus_state,
                            int counter_limit);
EMU_ID_RESISTOR_T get_id_res_value(void);
EMU_DEV_TYPE_T get_device_type(void);
EMU_BUS_SIGNAL_STATE_T get_bus_state(EMU_BUS_SIGNAL_T signal);
void emu_util_set_emu_conn_mode(MOTO_ACCY_HEADSET_MODE_T mode);
void emu_default_register_settings(int *polling_interval);

/**************************** XCVR FUNCTIONS *************************************/
void emu_configure_usb_xcvr(EMU_XCVR_T state);

/************************** GLOBAL VARIABLES *************************************/
/* Holds the current state */
extern EMU_STATE_T emu_state;

/* Holds the previous state */
extern EMU_STATE_T emu_prev_state;

/* Holds what we believe the currently connected device is */
extern MOTO_ACCY_TYPE_T emu_current_device;

/* Holds the current device type */
extern EMU_DEV_TYPE_T emu_current_device_type;

/* Used to keep track of the send/end key position */
extern EMU_BUS_SIGNAL_STATE_T emu_id_state;

/* Used to keep track of state of VBUS */
extern EMU_BUS_SIGNAL_STATE_T emu_vbus_det_state;

/* Used to keep track of the number of times a device
   type or bus state has been debounced */
extern int emu_debounce_counter;
/*Used to keep track of the conn mode update status.*/
extern BOOL emu_audio_update;
/*Used to keep track of the conn mode*/
extern MOTO_ACCY_HEADSET_MODE_T emu_audio_mode;
#endif /* __EMU_H__ */
