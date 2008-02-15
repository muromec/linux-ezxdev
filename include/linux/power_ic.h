/*
 * Copyright 2004-2005 Motorola, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 */
/*
 * Revision History:
 *                    Modification     Tracking
 * Author                 Date          Number     Description of Changes
 * ----------------   ------------    ----------   -------------------------
 * Michelle(w38024)    09/07/2005     LIBgg81373    Missing defines for EMU accessories in power_ic.h
 * Golec Chris(wlcg05) 11/22/2005     LIBhh36712    update PCAP2 init value for product HAINAN and Martinique.
 * Michelle(w38024)    11/28/2005     LIBhh42187    Port TCMD ioctl calls for SCM-A11
 *
 */


/*!
 * @mainpage Power IC driver
 *
 * @section intro Introduction
 *
 * @subsection intro_purpose Purpose
 *
 * The purpose of this documentation is to document the design and programming interface 
 * of the power IC device driver for various Linux platforms.
 *
 * @subsection intro_target Target Audience
 *
 * This document is intended to be used by software developers.
 *
 * @subsection intro_problems Problem Reporting Instructions
 *
 * Problems or corrections to this document must be reported using DDTS.
 *
 * @section design Design Information
 *
 * The documentation is divided up on a per-module basis.  Click on the
 * Modules link above for the detailed design information.
 *
 * @section api API Information
 *
 * For a description of the API for the driver, see the documentation for the
 * following files:
 *
 * - lights_backlight.h - Backlight control interface.
 * - lights_funlights.h - Interface for funlight control.
 * - moto_accy.h - Accessory-related interface.
 * - power_ic.h - Interface for the main functionality provided by the driver.
 */

/*!
 * @defgroup poweric_core Power IC core
 *
 * This module makes up the core of the low-level power IC driver.  It implements
 * support for the reading and writing of registers and manages the interrupts from
 * the power IC (or power ICs in the case of PCAP plus EMU One Chip).  The module
 * also provides the main interface to the remainder of the power IC modules (e.g.,
 * RTC, A/D converter, etc.).
 */
 
/*!
 * @defgroup poweric_accy Power IC accessory driver
 *
 * This module provides the interface for accessory operations through /dev/accy.
 */  
 
/*!
 * @defgroup poweric_atod Power IC AtoD converter driver
 *
 * This module provides the interface to the power IC for AtoD conversions.
 */ 
 
/*!
 * @defgroup poweric_audio Power IC audio driver
 *
 * This module controls the audio functions of the power IC.
 */  
 
/*!
 * @defgroup poweric_charger Power IC charger driver
 *
 * This module provides the interface to control various charging functions.
 */ 

/*!
 * @defgroup poweric_debounce Power IC debounce
 *
 * This module debounces signals from the power IC, primarily keypresses.
 */
 
/*!
 * @defgroup poweric_debug Power IC debug
 *
 * This is a group of source code that is not compiled by default, but is
 * included with the driver for developer builds to aid debugging.
 */ 
 
/*!
 * @defgroup poweric_emu Power IC EMU driver
 *
 * This module handles the detection of all devices attached to the EMU bus.
 */  
 
/*!
 * @defgroup poweric_lights Power IC lighting driver
 *
 * This module controls all of the lights (keypad, display, fun coloured stuff, etc)
 * on a phone.
 */   

/*!
 * @defgroup poweric_periph Power IC peripheral driver
 *
 * This module makes up the interface to the power IC for peripherals.  This includes
 * things like the vibrator, Bluetooth, and the flash card.
 */
 
/*!
 * @defgroup poweric_power_management Power IC power management driver
 *
 * This module provides interfaces for managing various power-related functions.
 */ 

/*!
 * @defgroup poweric_rtc Power IC RTC driver
 *
 * This module makes up the interface to the power IC for the real-time clock.
 */

/*!
 * @defgroup poweric_tcmd_ioctl Power IC TCMD ioctl support
 *
 * This module will contain all power-related ioctls that are required for TCMD 
 * support unless the requirements can be met with ones that already exist for normal
 * operation. 
 */ 
 
 /*!
 * @defgroup poweric_touchscreen Power IC Touchscreen driver
 *
 * This module makes up the interface to the power IC for touchscreen.
 */
 
#ifndef __POWER_IC_H__
#define __POWER_IC_H__

/*!
 * @file power_ic.h
 *
 * @ingroup poweric_core
 *
 * @brief Contains power IC driver interface information (types, enums, macros, functions, etc.)
 *
 * This file contains the following information:
 * - User-space interface
 *   - @ref ioctl_core        "Core ioctl commands"
 *   - @ref ioctl_atod        "AtoD converter ioctl commands" 
 *   - @ref ioctl_audio       "Audio ioctl commands"
 *   - @ref ioctl_charger     "Charger ioctl commands" 
 *   - @ref ioctl_lights      "Lighting ioctl commands" 
 *   - @ref ioctl_periph      "Peripheral ioctl commands"
 *   - @ref ioctl_pwr_mgmt    "Power management ioctl commands"
 *   - @ref ioctl_rtc         "RTC ioctl commands"
 *   - @ref ioctl_tcmd        "Test Command ioctl commands"
 *   - @ref ioctl_touchscreen "Touchscreen ioctl commands"
 *   - @ref ioctl_types       "Types used in ioctl calls"
 *
 * - Kernel-Space Interface:
 *   - @ref kernel_reg_macros        "Register Macros"
 *   - @ref kernel_types             "Types"
 *   - @ref kernel_funcs_atod        "AtoD functions"
 *   - @ref kernel_funcs_audio       "Audio functions"
 *   - @ref kernel_funcs_charger     "Charger functions"
 *   - @ref kernel_funcs_event       "Event functions"
 *   - @ref kernel_funcs_periph      "Peripheral functions"
 *   - @ref kernel_funcs_pwr_mgmt    "Power Management functions"
 *   - @ref kernel_funcs_reg         "Register access functions"
 *   - @ref kernel_funcs_rtc         "RTC functions"
 *   - @ref kernel_funcs_touchscreen "Touchscreen functions"
 */

#include <linux/ioctl.h>

/* Including the kernel version of the time header from user-space is causing some 
 * headaches. Until someone comes up with a better idea, use the kernel version for
 * kernel builds and the system's version for user-space. We only really care about
 * timeval anway...
 */
#ifdef __KERNEL__
#include <linux/time.h>
#else
#include <sys/time.h>
#include <sys/types.h>
#endif

#include <linux/lights_backlight.h>
#include <linux/lights_funlights.h>

#include <stdbool.h>

/*******************************************************************************************
 * Universal constants
 *
 *    These are constants that are universal to use of the power IC driver from both kernel
 *    and user space.. 
 ******************************************************************************************/
/*! The major number of the power IC driver. */
#define POWER_IC_MAJOR_NUM 220

/*! The name of the device in /dev. */
#define POWER_IC_DEV_NAME "power_ic"

/*! The number of individual AtoD readings returned for the AtoD raw conversion request. */
#define POWER_IC_ATOD_NUM_RAW_RESULTS 8

/******************************************************************************************
* NOTE
* 
*    #ifdefs are not recognized outside of the kernel, so they cannot be used in this 
*    global header.
* 
*    This separation of enums for the PCAP/EOC/FL/ATLAS registers is a temporary solution
*    because there are individuals outside of the kernel directly accessing these
*    registers by name.  DO NOT USE THESE REGISTERS DIRECTLY!!!!  
*
* BE WARNED:
*
*    We need to be able to support multiple platforms and if you directly access these 
*    registers, your stuff will break.
******************************************************************************************/

/*! Enumeration of all registers in the power IC(s) */
enum
{
    POWER_IC_REG_PCAP_FIRST_REG = 0,

    POWER_IC_REG_PCAP_ISR = POWER_IC_REG_PCAP_FIRST_REG, /*!< Interrupt status regiter. */
    POWER_IC_REG_PCAP_IMR,           /*!< Interrupt mask register. */
    POWER_IC_REG_PCAP_PSTAT,         /*!< Processor status register. */
    POWER_IC_REG_PCAP_INT_SEL,       /*!< Interrupt select register. */
    POWER_IC_REG_PCAP_SWCTRL,        /*!< Switching regulator control register. */
    POWER_IC_REG_PCAP_VREG1,         /*!< Regulator bank 1 control register. */
    POWER_IC_REG_PCAP_VREG2,         /*!< Regulator bank 2 control register. */
    POWER_IC_REG_PCAP_AUX_VREG,      /*!< Auxiliary regulator control register. */
    POWER_IC_REG_PCAP_BATT_DAC,      /*!< Battery control register. */
    POWER_IC_REG_PCAP_ADC1,          /*!< AtoD control register. */
    POWER_IC_REG_PCAP_ADC2,          /*!< AtoD result register. */
    POWER_IC_REG_PCAP_AUD_CODEC,     /*!< Audio codec control register. */
    POWER_IC_REG_PCAP_RX_AUD_AMPS,   /*!< Rx audio amplifiers control register. */
    POWER_IC_REG_PCAP_ST_DAC,        /*!< Stereo DAC control register. */
    POWER_IC_REG_PCAP_RTC_TOD,       /*!< Real Time Clock time of day register. */
    POWER_IC_REG_PCAP_RTC_TODA,      /*!< RTC time of day alarm register. */
    POWER_IC_REG_PCAP_RTC_DAY,       /*!< RTC day register. */
    POWER_IC_REG_PCAP_RTC_DAYA,      /*!< RTC day alarm register. */
    POWER_IC_REG_PCAP_MTRTMR,        /*!< AtoD monitor timer register. */
    POWER_IC_REG_PCAP_PWRCTRL,       /*!< Power control register. */
    POWER_IC_REG_PCAP_BUSCTRL,       /*!< Connectivity register. */
    POWER_IC_REG_PCAP_PERIPH,        /*!< Peripheral control register. */
    POWER_IC_REG_PCAP_AUX_VREG_MASK, /*!< Auxiliary regulator mask register. */
    POWER_IC_REG_PCAP_VENDOR_REV,
    POWER_IC_REG_PCAP_LOWPWR_CTRL,   /*!< Regulator low power control register. */
    POWER_IC_REG_PCAP_PERIPH_MASK,   /*!< Peripheral control register. */
    POWER_IC_REG_PCAP_TX_AUD_AMPS,   /*!< Tx audio amplifiers control register. */
    POWER_IC_REG_PCAP_GP,            /*!< General purpose register. */
    POWER_IC_REG_PCAP_TEST1,
    POWER_IC_REG_PCAP_TEST2,
    POWER_IC_REG_PCAP_VENDOR_TEST1,
    POWER_IC_REG_PCAP_VENDOR_TEST2,

    POWER_IC_REG_PCAP_LAST_REG = POWER_IC_REG_PCAP_VENDOR_TEST2,

    /* End of PCAP registers.  Start of EMU One Chip registers. */

    POWER_IC_REG_EOC_FIRST_REG,

    POWER_IC_REG_EOC_INT_STATUS = POWER_IC_REG_EOC_FIRST_REG,
    POWER_IC_REG_EOC_INT_MASK,
    POWER_IC_REG_EOC_INT_SENSE,
    POWER_IC_REG_EOC_POWER_CONTROL_0,
    POWER_IC_REG_EOC_POWER_CONTROL_1,
    POWER_IC_REG_EOC_CONN_CONTROL,

    POWER_IC_REG_EOC_LAST_REG = POWER_IC_REG_EOC_CONN_CONTROL,

    /* End of EMU One Chip Registers. Start of Funlight registers. */

    POWER_IC_REG_FL_FIRST_REG,

    POWER_IC_REG_FL_INPUT1  = POWER_IC_REG_FL_FIRST_REG,  /* LED0-7 Input Register */
    POWER_IC_REG_FL_REGISTER1,      /*!< Register 1...empty */
    POWER_IC_REG_FL_PSC0,           /*!< Frequency Prescaler 0 */
    POWER_IC_REG_FL_PWM0,           /*!< PWM Register 0 */
    POWER_IC_REG_FL_PSC1,           /*!< Frequency Prescaler 1 */
    POWER_IC_REG_FL_PWM1,           /*!< PWM Register 1 */
    POWER_IC_REG_FL_LS0,            /*!< LED0-3 Selector */
    POWER_IC_REG_FL_LS1,            /*!< LED4-7 Selector */
    
    POWER_IC_REG_FL_LAST_REG = POWER_IC_REG_FL_LS1,

    /* End of Funlight Registers */
    
    POWER_IC_REG_NUM_REGS_PCAP
}; 

enum
{
    /* Beginning of Atlas Registers */
    POWER_IC_REG_ATLAS_FIRST_REG,
    
    POWER_IC_REG_ATLAS_INT_STAT_0 = POWER_IC_REG_ATLAS_FIRST_REG,  /*!< Interrupt Status 0 */
    POWER_IC_REG_ATLAS_INT_MASK_0,            /*!< Interrupt Mask 0 */
    POWER_IC_REG_ATLAS_INT_SENSE_0,           /*!< Interrupt Sense 0 */
    POWER_IC_REG_ATLAS_INT_STAT_1,            /*!< Interrupt Status 1 */
    POWER_IC_REG_ATLAS_INT_MASK_1,            /*!< Interrupt Mask 1 */
    POWER_IC_REG_ATLAS_INT_SENSE_1,           /*!< Interrupt Sense 1 */
    POWER_IC_REG_ATLAS_PWRUP_SENSE,           /*!< Power Up Mode Sense */
    POWER_IC_REG_ATLAS_REVISION,              /*!< Revision */
    POWER_IC_REG_ATLAS_SEMAPHORE,             /*!< Semaphore */
    POWER_IC_REG_ATLAS_ARB_PERIPH_AUDIO,      /*!< Arbitration Peripheral Audio */
    POWER_IC_REG_ATLAS_ARB_SWITCHERS,         /*!< Arbitration Switchers */
    POWER_IC_REG_ATLAS_ARB_REG_0,             /*!< Arbitration Regulators 0 */
    POWER_IC_REG_ATLAS_ARB_REG_1,             /*!< Arbitration Regulators 1 */
    POWER_IC_REG_ATLAS_PWR_CONTROL_0,         /*!< Power Control 0 */
    POWER_IC_REG_ATLAS_PWR_CONTROL_1,         /*!< Power Control 1 */
    POWER_IC_REG_ATLAS_PWR_CONTROL_2,         /*!< Power Control 2 */
    POWER_IC_REG_ATLAS_REGEN_ASSIGN,          /*!< Regen Assignment */
    POWER_IC_REG_ATLAS_CONTROL_SPARE,         /*!< Control Spare */
    POWER_IC_REG_ATLAS_MEMORY_A,              /*!< Memory A */
    POWER_IC_REG_ATLAS_MEMORY_B,              /*!< Memory B */
    POWER_IC_REG_ATLAS_RTC_TIME,              /*!< RTC Time */
    POWER_IC_REG_ATLAS_RTC_ALARM,             /*!< RTC Alarm */
    POWER_IC_REG_ATLAS_RTC_DAY,               /*!< RTC Day */
    POWER_IC_REG_ATLAS_RTC_DAY_ALARM,         /*!< RTC Day Alarm */
    POWER_IC_REG_ATLAS_SWITCHERS_0,           /*!< Switchers 0 */
    POWER_IC_REG_ATLAS_SWITCHERS_1,           /*!< Switchers 1 */
    POWER_IC_REG_ATLAS_SWITCHERS_2,           /*!< Switchers 2 */
    POWER_IC_REG_ATLAS_SWITCHERS_3,           /*!< Switchers 3 */ 
    POWER_IC_REG_ATLAS_SWITCHERS_4,           /*!< Switchers 4 */
    POWER_IC_REG_ATLAS_SWITCHERS_5,           /*!< Switchers 5 */
    POWER_IC_REG_ATLAS_REG_SET_0,             /*!< Regulator Setting 0 */
    POWER_IC_REG_ATLAS_REG_SET_1,             /*!< Regulator Setting 1 */
    POWER_IC_REG_ATLAS_REG_MODE_0,            /*!< Regulator Mode 0 */
    POWER_IC_REG_ATLAS_REG_MODE_1,            /*!< Regulator Mode 1 */
    POWER_IC_REG_ATLAS_PWR_MISC,              /*!< Power Miscellaneous */
    POWER_IC_REG_ATLAS_PWR_SPARE,             /*!< Power Spare */
    POWER_IC_REG_ATLAS_AUDIO_RX_0,            /*!< Audio Rx 0 */
    POWER_IC_REG_ATLAS_AUDIO_RX_1,            /*!< Audio Rx 1 */
    POWER_IC_REG_ATLAS_AUDIO_TX,              /*!< Audio Tx */
    POWER_IC_REG_ATLAS_SSI_NETWORK,           /*!< SSI Network */
    POWER_IC_REG_ATLAS_AUDIO_CODEC,           /*!< Audio Codec */
    POWER_IC_REG_ATLAS_AUDIO_STEREO_DAC,      /*!< Audio Stereo DAC */
    POWER_IC_REG_ATLAS_AUDIO_SPARE,           /*!< Audio Spare */
    POWER_IC_REG_ATLAS_ADC_0,                 /*!< ADC 0 */
    POWER_IC_REG_ATLAS_ADC_1,                 /*!< ADC 1 */
    POWER_IC_REG_ATLAS_ADC_2,                 /*!< ADC 2 */
    POWER_IC_REG_ATLAS_ADC_3,                 /*!< ADC 3 */
    POWER_IC_REG_ATLAS_ADC_4,                 /*!< ADC 4 */
    POWER_IC_REG_ATLAS_CHARGER_0,             /*!< Charger 0 */
    POWER_IC_REG_ATLAS_USB_0,                 /*!< USB 0 */
    POWER_IC_REG_ATLAS_CHARGE_USB_1,          /*!< USB 1 */
    POWER_IC_REG_ATLAS_LED_CONTROL_0,         /*!< LED Control 0 */
    POWER_IC_REG_ATLAS_LED_CONTROL_1,         /*!< LED Control 1 */
    POWER_IC_REG_ATLAS_LED_CONTROL_2,         /*!< LED Control 2 */
    POWER_IC_REG_ATLAS_LED_CONTROL_3,         /*!< LED Control 3 */
    POWER_IC_REG_ATLAS_LED_CONTROL_4,         /*!< LED Control 4 */
    POWER_IC_REG_ATLAS_LED_CONTROL_5,         /*!< LED Control 5 */
    POWER_IC_REG_ATLAS_SPARE,                 /*!< Spare */
    POWER_IC_REG_ATLAS_TRIM_0,                /*!< Trim 0 */
    POWER_IC_REG_ATLAS_TRIM_1,                /*!< Trim 1 */
    POWER_IC_REG_ATLAS_TEST_0,                /*!< Test 0 */
    POWER_IC_REG_ATLAS_TEST_1,                /*!< Test 1*/
    POWER_IC_REG_ATLAS_TEST_2,                /*!< Test 2 */
    POWER_IC_REG_ATLAS_TEST_3,                /*!< Test 3 */
    
    POWER_IC_REG_ATLAS_LAST_REG = POWER_IC_REG_ATLAS_TEST_3,
    
    /* End of Atlas Registers */
   
    POWER_IC_REG_NUM_REGS_ATLAS
};

/*! Type for all registers. Only values in register enumeration are valid. */
typedef int POWER_IC_REGISTER_T;

/*! Enumeration of possible power IC events */
enum
{
    POWER_IC_EVENT_PCAP_FIRST = 0, 

    POWER_IC_EVENT_PCAP_ADCDONEI = POWER_IC_EVENT_PCAP_FIRST, /*!< AtoD conversion complete interrupt. */
    POWER_IC_EVENT_PCAP_TSI,          /*!< Touchscreen interrupt. */
    POWER_IC_EVENT_PCAP_1HZI,         /*!< 1Hz interrupt. */
    POWER_IC_EVENT_PCAP_WHI,          /*!< AtoD above WHIGH threshold interrupt. */
    POWER_IC_EVENT_PCAP_WLI,          /*!< AtoD below WLOW threshold interrupt. */
    POWER_IC_EVENT_PCAP_TODAI,        /*!< Time of day alarm interrupt. */
    POWER_IC_EVENT_PCAP_USB4VI,       /*!< USBDET 4V interrupt. */
    POWER_IC_EVENT_PCAP_ONOFFI,       /*!< On/Off button interrupt. */
    POWER_IC_EVENT_PCAP_ONOFF2I,      /*!< On/Off button 2 interrupt. */
    POWER_IC_EVENT_PCAP_USB1VI,       /*!< USB 1V interrupt. */
    POWER_IC_EVENT_PCAP_MOBPORTBI,    /*!< MOBPORTB interrupt. */
    POWER_IC_EVENT_PCAP_MB2I,         /*!< Mic Bias current interrupt. */
    POWER_IC_EVENT_PCAP_A1I,          /*!< A1 interrupt. */
    POWER_IC_EVENT_PCAP_STI,          /*!< Stereo headset detect interrupt. */
    POWER_IC_EVENT_PCAP_PCI,          /*!< Power cut interrupt. */
    POWER_IC_EVENT_PCAP_WARMI,        /*!< Warm start interrupt. */
    POWER_IC_EVENT_PCAP_EOLI,         /*!< Battery end of life interrupt. */
    POWER_IC_EVENT_PCAP_CLKI,         /*!< CLK_STAT interrupt. */
    POWER_IC_EVENT_PCAP_UNUSED,
    POWER_IC_EVENT_PCAP_BATTFBI,      /*!< Battery feedback interrupt. */
    POWER_IC_EVENT_PCAP_ADCDONE2I,    /*!< AtoD conversion complete interrupt. */
    POWER_IC_EVENT_PCAP_SOFT_RESETI,  /*!< BATT_DET_IN interrupt. */
    POWER_IC_EVENT_PCAP_MNEXBI,       /*!< Seamless comparator interrupt. */

    POWER_IC_EVENT_PCAP_LAST = POWER_IC_EVENT_PCAP_MNEXBI,

    POWER_IC_EVENT_EOC_FIRST,

    POWER_IC_EVENT_EOC_VBUS_3V4 = POWER_IC_EVENT_EOC_FIRST, /*!< VBUS > 3.4 volts */
    POWER_IC_EVENT_EOC_VBUSDET,      /*!< VBUS voltage changed */
    POWER_IC_EVENT_EOC_VBUSOV,       /*!< VBUS overvoltage */
    POWER_IC_EVENT_EOC_RVRS_CHRG,    /*!< Reverse charge */
    POWER_IC_EVENT_EOC_ID,           /*!< ID transition (ID float bit) */
    POWER_IC_EVENT_EOC_ID_FLOAT = POWER_IC_EVENT_EOC_ID,
    POWER_IC_EVENT_EOC_ID_GROUND,    /*!< ID ground sense bit (unused interrupt status/mask) */
    POWER_IC_EVENT_EOC_SE1_DET,      /*!< single-ended 1 detect */
    POWER_IC_EVENT_EOC_CC_CV,        /*!< charger mode transition */
    POWER_IC_EVENT_EOC_CHRG_CURR,    /*!< charger current below 20mA */
    POWER_IC_EVENT_EOC_RVRS_MODE,    /*!< Reverse current limit exceeded */
    POWER_IC_EVENT_EOC_CK_DET,       /*!< Carkit detect */
    POWER_IC_EVENT_EOC_BATTPON,      /*!< Battery Voltage > or < 3.4 volts */

    POWER_IC_EVENT_EOC_LAST = POWER_IC_EVENT_EOC_BATTPON,
    
    POWER_IC_EVENT_NUM_EVENTS_PCAP
};

enum
{
    POWER_IC_EVENT_ATLAS_FIRST_REG,
    
    /* Interrupt Status 0 Register */
    POWER_IC_EVENT_ATLAS_ADCDONEI = POWER_IC_EVENT_ATLAS_FIRST_REG, /*!< AtoD Conversion complete interrupt */
    POWER_IC_EVENT_ATLAS_ADCBISDONEI,    /*!< ADCBIS complete interrupt */
    POWER_IC_EVENT_ATLAS_TSI,            /*!< Touchscreen interrupt */
    POWER_IC_EVENT_ATLAS_WHIGHI,         /*!< ADC reading above high limit interrupt */
    POWER_IC_EVENT_ATLAS_WLOWI,          /*!< ADC reading below low limit interrupt */
    POWER_IC_EVENT_ATLAS_RESERVED1,      /*!< For future use */
    POWER_IC_EVENT_ATLAS_CHGDETI,        /*!< Charger Attached interrupt */
    POWER_IC_EVENT_ATLAS_CHGOVI,         /*!< Charger overvoltage detection interrupt */
    POWER_IC_EVENT_ATLAS_CHGREVI,        /*!< Charger path reverse current interrupt */
    POWER_IC_EVENT_ATLAS_CHGSHORTI,      /*!< Charger path short current interrupt */
    POWER_IC_EVENT_ATLAS_CCCVI,          /*!< BP regulator V or I interrupt */
    POWER_IC_EVENT_ATLAS_CHGCURRI,       /*!< Charge current below threshold warning interrupt */
    POWER_IC_EVENT_ATLAS_BPONI,          /*!< BP turn on threshold interrupt */
    POWER_IC_EVENT_ATLAS_LOBATLI,        /*!< Low battery low threshold warning interrupt */
    POWER_IC_EVENT_ATLAS_LOBATHI,        /*!< Low battery high threshold warning interrupt */
    POWER_IC_EVENT_ATLAS_RESERVED2,      /*!< For future use */
    POWER_IC_EVENT_ATLAS_USBI,           /*!< USB VBUS detect interrupt */
    POWER_IC_EVENT_ATLAS_USB2V0S,        /*!< USB2V0 sense bit, unused in mask interrupt */
    POWER_IC_EVENT_ATLAS_USB0V8S,        /*!< USB0V8 sense bit, unused in mask interrupt */
    POWER_IC_EVENT_ATLAS_IDI,            /*!< USB ID detect interrupt */
    POWER_IC_EVENT_ATLAS_ID_FLOAT = POWER_IC_EVENT_ATLAS_IDI,
    POWER_IC_EVENT_ATLAS_ID_GROUND,      /*!< USB ID Ground sense bit (unused interrupt status/mask) */
    POWER_IC_EVENT_ATLAS_SE1I,           /*!< Single ended 1 detect interrupt */
    POWER_IC_EVENT_ATLAS_CKDETI,         /*!< Carkit detect interrupt */
    POWER_IC_EVENT_ATLAS_RESERVED3,      /*!< For future use */
    
    POWER_IC_EVENT_ATLAS_FIRST_REG_LAST = POWER_IC_EVENT_ATLAS_RESERVED3,
    
    POWER_IC_EVENT_ATLAS_SECOND_REG,
    
    /* Interrupt Status 1 Register */
    POWER_IC_EVENT_ATLAS_1HZI = POWER_IC_EVENT_ATLAS_SECOND_REG,           /*!< 1 Hz timetick */
    POWER_IC_EVENT_ATLAS_TODAI,          /*!< Time of day alarm interrupt */
    POWER_IC_EVENT_ATLAS_RESERVED4,      /*!< For future use */
    POWER_IC_EVENT_ATLAS_ONOFD1I,        /*!< ON1B event interrupt */
    POWER_IC_EVENT_ATLAS_ONOFD2I,        /*!< ON2B event interrupt */
    POWER_IC_EVENT_ATLAS_ONOFD3I,        /*!< ON3B event interrupt */
    POWER_IC_EVENT_ATLAS_SYSRSTI,        /*!< System reset interrupt */
    POWER_IC_EVENT_ATLAS_RTCRSTI,        /*!< RTC reset event interrupt */
    POWER_IC_EVENT_ATLAS_PCI,            /*!< Power cut event interrupt */
    POWER_IC_EVENT_ATLAS_WARMI,          /*!< Warm start event interrupt */
    POWER_IC_EVENT_ATLAS_MEMHLDI,        /*!< Memory hold event interrupt */
    POWER_IC_EVENT_ATLAS_PWRRDYI,        /*!< Power Gate and DVS power ready interrupt */
    POWER_IC_EVENT_ATLAS_THWARNLI,       /*!< Thermal warning low threshold interrupt */
    POWER_IC_EVENT_ATLAS_THWARNHI,       /*!< Thermal warning high threshold interrupt */
    POWER_IC_EVENT_ATLAS_CLKI,           /*!< Clock source change interrupt */
    POWER_IC_EVENT_ATLAS_SEMAFI,         /*!< Semaphore */
    POWER_IC_EVENT_ATLAS_RESERVED5,      /*!< For future use */
    POWER_IC_EVENT_ATLAS_MC2BI,          /*!< Microphone bias 2 detect interrupt */
    POWER_IC_EVENT_ATLAS_HSDETI,         /*!< Headset attach interrupt */
    POWER_IC_EVENT_ATLAS_HSLI,           /*!< Stereo headset detect interrupt */
    POWER_IC_EVENT_ATLAS_ALSPTHI,        /*!< Thermal shutdown Alsp interrupt */
    POWER_IC_EVENT_ATLAS_AHSSHORTI,      /*!< Short circuit on Ahs outputs interrupt */
    POWER_IC_EVENT_ATLAS_RESERVED6,      /*!< For future use */
    POWER_IC_EVENT_ATLAS_RESERVED7,      /*!< For future use */
    
    POWER_IC_EVENT_ATLAS_SECOND_REG_LAST = POWER_IC_EVENT_ATLAS_RESERVED7,

    POWER_IC_EVENT_NUM_EVENTS_ATLAS
};

typedef int  POWER_IC_EVENT_T;

/*! Defines the possible settings for the state of a power IC peripheral */
typedef enum
{
    POWER_IC_PERIPH_OFF,
    POWER_IC_PERIPH_ON
} POWER_IC_PERIPH_ONOFF_T;

/* For audio interface */
/* Output path bit mask */
#define  POWER_IC_NO_SPEAKER_MASK              0x00
#define  POWER_IC_HEADSET_SPEAKER_MASK         0x01  
#define  POWER_IC_HANDSET_SPEAKER_MASK         0x02
#define  POWER_IC_ALERT_SPEAKER_MASK           0x04
#define  POWER_IC_ST_HEADSET_SPEAKER_MASK      0x08
#define  POWER_IC_BUS_SPEAKER_MASK             0x10
#define  POWER_IC_ST_HANDSET_SPEAKER_MASK      0x20
#define  POWER_IC_EMU_HEADSET_SPEAKER_MASK     0x40
#define  POWER_IC_EMU_ST_HEADSET_SPEAKER_MASK  0x80
#define  POWER_IC_ALL_SPEAKER_MASK             0xFF
#define  POWER_IC_NUM_OF_OUT_PATH              0x09

/* Input path enum */
typedef enum
{
    POWER_IC_NO_MIC,
    POWER_IC_HEADSET_MIC,
    POWER_IC_HANDSET_MIC,
    POWER_IC_EXTERNAL_MIC,
    POWER_IC_NUM_OF_IN_PATH
} POWER_IC_AUD_IN_PATH_T;

/*Sample rate for the ST Dac*/
typedef enum
{
    POWER_IC_ST_DAC_SR_8000,
    POWER_IC_ST_DAC_SR_11025,
    POWER_IC_ST_DAC_SR_12000,
    POWER_IC_ST_DAC_SR_16000,
    POWER_IC_ST_DAC_SR_22050,
    POWER_IC_ST_DAC_SR_24000,
    POWER_IC_ST_DAC_SR_32000,
    POWER_IC_ST_DAC_SR_44100,
    POWER_IC_ST_DAC_SR_48000
} POWER_IC_ST_DAC_SR_T;

/* Sample rate for the codec */
typedef enum
{
    POWER_IC_CODEC_SR_8000,
    POWER_IC_CODEC_SR_16000
} POWER_IC_CODEC_SR_T;

/*! For the AtoD interface, these are the individual channels that can be requested. */
typedef enum
{
    POWER_IC_ATOD_CHANNEL_AD6,
    POWER_IC_ATOD_CHANNEL_BATT,
    POWER_IC_ATOD_CHANNEL_BATT_CURR,
    POWER_IC_ATOD_CHANNEL_BPLUS,
    POWER_IC_ATOD_CHANNEL_CHARGER_ID,
    POWER_IC_ATOD_CHANNEL_CHRG_CURR,
    POWER_IC_ATOD_CHANNEL_COIN_CELL,
    POWER_IC_ATOD_CHANNEL_MOBPORTB,
    POWER_IC_ATOD_CHANNEL_TEMPERATURE,
    
    POWER_IC_ATOD_NUM_CHANNELS
    
} POWER_IC_ATOD_CHANNEL_T;

/*! The timing requested for the battery/current conversion. */
typedef enum
{
    POWER_IC_ATOD_TIMING_IMMEDIATE,
    POWER_IC_ATOD_TIMING_IN_BURST,
    POWER_IC_ATOD_TIMING_OUT_OF_BURST
} POWER_IC_ATOD_TIMING_T;

/*! Indication of whether a hardware-timed conversion completed or timed out. */
typedef enum
{
    POWER_IC_ATOD_CONVERSION_TIMEOUT,
    POWER_IC_ATOD_CONVERSION_COMPLETE
} POWER_IC_ATOD_CONVERSION_TIMEOUT_T;

/*! The direction in which current will be measured for the battery/current conversion. */
typedef enum
{
    POWER_IC_ATOD_CURR_POLARITY_DISCHARGE,
    POWER_IC_ATOD_CURR_POLARITY_CHARGE,
} POWER_IC_ATOD_CURR_POLARITY_T;

/*! Power-up reasons */
typedef enum
{
    POWER_IC_POWER_UP_REASON_NONE,
    POWER_IC_POWER_UP_REASON_FIRST_POWER_KEY_LONG,    /*!< indicates sense bit still active */
    POWER_IC_POWER_UP_REASON_FIRST_POWER_KEY_SHORT,   /*!< indicates only status bit active */
    POWER_IC_POWER_UP_REASON_SECOND_POWER_KEY_LONG,   /*!< indicates sense bit still active */
    POWER_IC_POWER_UP_REASON_SECOND_POWER_KEY_SHORT,  /*!< indicates only status bit active */
    POWER_IC_POWER_UP_REASON_THIRD_POWER_KEY_LONG,    /*!< indicates sense bit still active - ATLAS */
    POWER_IC_POWER_UP_REASON_THIRD_POWER_KEY_SHORT,   /*!< indicates only status bit active - ATLAS */
    POWER_IC_POWER_UP_REASON_CHARGER,
    POWER_IC_POWER_UP_REASON_POWER_CUT,
    POWER_IC_POWER_UP_REASON_ALARM,
} POWER_IC_POWER_UP_REASON_T;



/*! Power paths that can be selected. */
typedef enum
{
    POWER_IC_CHARGER_POWER_DUAL_PATH,                 /*!< Dual-path mode under hardware control. */
    POWER_IC_CHARGER_POWER_CURRENT_SHARE,             /*!< Current-share under hardware control. */
    POWER_IC_CHARGER_POWER_DUAL_PATH_SW_OVERRIDE,     /*!< Dual-path mode forced under software control. */
    POWER_IC_CHARGER_POWER_CURRENT_SHARE_SW_OVERRIDE  /*!< Current-share forced under software control. */
} POWER_IC_CHARGER_POWER_PATH_T;

/*! SIM Voltages that can be selected. */
typedef enum
{
    POWER_IC_SIM_VOLT_18,        /*!< indicates a 1.8 voltage setting for SIM */
    POWER_IC_SIM_VOLT_30         /*!< indicates a 3.0 voltage setting for SIM */
} POWER_IC_SIM_VOLTAGE_T;


#ifndef DOXYGEN_SHOULD_SKIP_THIS /* This stuff just clutters the documentation. Don't bother. */
/*!
 * @name ioctl() command ranges
 *
 * These are the ranges of ioctl commands for the power IC driver. These cannot be used
 * directly from user-space, but are used to construct the request parameters used in ioctl() 
 * calls to the driver. 
 */

/* @{ */
 
/* Base of all the ioctl() commands that will be handled by the driver core. */
#define POWER_IC_IOC_CMD_CORE_BASE          0x00
/* Last of the range of ioctl() commands reserved for the driver core. */
#define POWER_IC_IOC_CMD_CORE_LAST_CMD      (POWER_IC_IOC_CMD_CORE_BASE + 0x0F)

/* Base of all the ioctl() commands that will be handled by the peripheral module. */
#define POWER_IC_IOC_CMD_PERIPH_BASE        (POWER_IC_IOC_CMD_CORE_LAST_CMD + 1)
/* This is the last of the range of ioctl() commands reserved for the peripheral interface. */
#define POWER_IC_IO_CMD_PERIPH_LAST_CMD     (POWER_IC_IOC_CMD_PERIPH_BASE + 0x0F)

/* Base for the ioctl() commands for the RTC */
#define POWER_IC_IOC_RTC_BASE               (POWER_IC_IO_CMD_PERIPH_LAST_CMD + 1)
/* Last ioctl() command reserved for the RTC. */
#define POWER_IC_IOC_RTC_LAST_CMD           (POWER_IC_IOC_RTC_BASE + 0x0F)

/*! Base for the ioctl() commands for AtoD requests */
#define POWER_IC_IOC_ATOD_BASE              (POWER_IC_IOC_RTC_LAST_CMD + 1)
/*! Last ioctl() command reserved for AtoD. */
#define POWER_IC_IOC_ATOD_LAST_CMD          (POWER_IC_IOC_ATOD_BASE + 0x0F)

/*! Base for the ioctl() commands for the power management module. */
#define POWER_IC_IOC_PMM_BASE               (POWER_IC_IOC_ATOD_LAST_CMD + 1)
/*! Last ioctl() command reserved for power management. */
#define POWER_IC_IOC_PMM_LAST_CMD           (POWER_IC_IOC_PMM_BASE + 0x0F)

/* Base of all the ioctl() commands that will be handled by the audio module. */
#define POWER_IC_IOC_CMD_AUDIO_BASE         (POWER_IC_IOC_PMM_LAST_CMD + 1)
/* Last ioctl() command reserved for the audio interface. */
#define POWER_IC_IOC_CMD_AUDIO_LAST_CMD     (POWER_IC_IOC_CMD_AUDIO_BASE + 0x1F)

/* This is the base of all the ioctl() commands that will be handled by the lights module. */
#define POWER_IC_IOC_LIGHTS_BASE            (POWER_IC_IOC_CMD_AUDIO_LAST_CMD + 1)
/* Last ioctl() command reserved for the lights interface. */
#define POWER_IC_IOC_LIGHTS_LAST_CMD        (POWER_IC_IOC_LIGHTS_BASE + 0x0F)

/* This is the base of all the ioctl() commands that will be handled by the charger module. */
#define POWER_IC_IOC_CMD_CHARGER_BASE       (POWER_IC_IOC_LIGHTS_LAST_CMD + 1)
/* Last ioctl() command reserved for the charger interface. */
#define POWER_IC_IOC_CHARGER_LAST_CMD       (POWER_IC_IOC_CMD_CHARGER_BASE + 0x0F)

/* Base of all the ioctl() commands that will be handled by the touchscreen module. */
#define POWER_IC_IOC_CMD_TOUCHSCREEN_BASE   (POWER_IC_IOC_CHARGER_LAST_CMD + 1)
/* Last ioctl() command reserved for the touchscreen interface. */
#define POWER_IC_IOC_TOUCHSCREEN_LAST_CMD   (POWER_IC_IOC_CMD_TOUCHSCREEN_BASE + 0x0F)

/* Base of all the ioctl() commands that will be handled by the tcmd_ioctl module. */
#define POWER_IC_IOC_CMD_TCMD_BASE          (POWER_IC_IOC_TOUCHSCREEN_LAST_CMD + 1)
/* Last ioctl() command reserved for the tcmd_ioctl module. */
#define POWER_IC_IOC_CMD_TCMD_LAST_CMD      (POWER_IC_IOC_CMD_TCMD_BASE + 0x0F)

/* @} End of ioctl range constants. -------------------------------------------------------------- */

/* Old IOC_CMD numbers that we have removed. These were clearly commented that they should
 * not be used in calls to ioctl(), but they got used anyway. These are here only to allow
 * the build to complete so we don't hold too much up right now. 
 *
 * In case you missed it, DO NOT USE THESE. EVER. 
 */
#define POWER_IC_IOC_LIGHTS_BACKLIGHTS_SET           (POWER_IC_IOC_LIGHTS_BASE + 0x00)
#define POWER_IC_IOC_LIGHTS_FL_SET_CONTROL           (POWER_IC_IOC_LIGHTS_BASE + 0x01)
#define POWER_IC_IOC_LIGHTS_FL_UPDATE                (POWER_IC_IOC_LIGHTS_BASE + 0x02)

#endif /* Doxygen skips over this... */

/*******************************************************************************************
 * Driver ioctls
 *
 * All of the ioctl commands supported by the driver are in this section.
 *
 * Note: when adding new ioctl commands, each should be fully documented in the format used
 * for the existing commands. This will be the only documentation available for each ioctl,
 * so it is in our best interest to provide useful and consistent information here.  
 *
 * Typically, each ioctl command should have the following:
 *
 * A brief, one-sentence description prefixed with doxygen's brief tag.
 *
 * A paragraph with a more detailed description of the operation of the command.
 *
 * A paragraph describing inputs to the command.
 *
 * A paragraph describing outputs from the command.
 *
 * optionally, one or more notes about the command (prefxed by Doxygen's note tag).
 ******************************************************************************************/

/*!
 * @anchor ioctl_core
 * @name Core ioctl() commands
 *
 * These are the commands that can be passed to ioctl() to request low-level operations on the
 * power IC driver. In general. <b>the basic register access commands should not be used unless 
 * absolutely necessary</b>, as the register enumerations are different between the PCAP and 
 * Atlas platforms. If you write code against the PCAP register set, you <b>will</b> have to rewrite
 * that code for Atlas. Instead, the abstracted interfaces should be used to read and write 
 * values to registers as these are supported without any name changes between the two platforms.
 */

/* @{ */

/*!
 * @brief Reads a register.
 *
 * This command reads the entire contents of a single specified register and passes back its
 * contents. 
 *
 * The register is specified in the reg field of the passed POWER_IC_REG_ACCESS_T.
 *
 * The value read is returned in the value field of the structure.
 */
#define POWER_IC_IOCTL_READ_REG \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CORE_BASE + 0x00), POWER_IC_REG_ACCESS_T *)
        
/*!
 * @brief Writes a register.
 *
 * This command overwrites the entire contents of a single specified register. 
 * 
 * The register is specified in the reg field of the passed POWER_IC_REG_ACCESS_T and the 
 * new value to be written is passed in the value field of the structure.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 */
#define POWER_IC_IOCTL_WRITE_REG \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CORE_BASE + 0x01), POWER_IC_REG_ACCESS_T *)   

/*!
 * @brief Reads a subset of bits from a register.
 *
 * This command reads a contiguous set of bits from a single specified register. The
 * driver reads from the register specified and does the necessary masking and shifting so
 * that only the specified bits are passed back, shifted so that the first bit read is bit
 * zero of the value passed back.
 * 
 * The register is specified in the reg field of the passed POWER_IC_REG_ACCESS_T, the 
 * index of the first (least significant) bit is specified in the index field, and the 
 * number of bits including the first bit is specified in the num_bits field.
 *
 * The value read starts from bit zero of the returned value, which is passed back in the
 * value field of the structure. 
 */
#define POWER_IC_IOCTL_READ_REG_BITS \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CORE_BASE + 0x02), POWER_IC_REG_ACCESS_T *)
        
/*!
 * @brief Writes a subset of bits from a register.
 *
 * This command overwrites a contiguous set of bits from a single specified register. The 
 * driver takes the value and shifts it to the correct position in the register, overwriting
 * the subset of bits specified. The remaining bits are not changed.
 *
 * The register is specified in the reg field of the passed POWER_IC_REG_ACCESS_T, the index
 * of the first (least significant) bit is specified in the index field, the number of bits
 * (including the first bit) is specified in the num_bits field and the value to be shifted
 * and written is specified in the value field.
 * 
 * The command has no output other than the returned error code for the ioctl() call.
 */
#define POWER_IC_IOCTL_WRITE_REG_BITS \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CORE_BASE + 0x03), POWER_IC_REG_ACCESS_T *)
/*!
 * @brief Writes a subset of bits from a register.
 *
 * This command overwrites a possibly non-contiguous set of bits from a single specified 
 * register. This is similar to POWER_IC_IOCTL_WRITE_REG_BITS above, but instead the value 
 * specified must be bit-exact compared to the register to be written (i.e. the bits you 
 * want to set/clear in the register must be set the same way in the value passed in to 
 * the command.
 *
 * The register is specified in the reg field of the passed POWER_IC_REG_ACCESS_T
 * and the bitmask describing which register bits to be changed is specified by the 
 * num_bits field (each bit set will result in that bit in the register being overwritten) 
 * and the bit-exact value to be written is passed in the value field of the structure.
 * 
 * This command has no output other than the returned error code for the ioctl() call.
 */
#define POWER_IC_IOCTL_WRITE_REG_MASK \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CORE_BASE + 0x04), POWER_IC_REG_ACCESS_T *) 
        
/*!
 * @brief Gets the powerup reason.
 *
 * This command retrieves the reason the phone powered up from the driver. When the driver 
 * starts, it checks for possible reasons for powering up and remembers the reason, which
 * is retrieved later via this command.
 *
 * The command has no inputs.
 * 
 * The command passes back the powerup reason via the passed pointer to a 
 * POWER_IC_POWER_UP_REASON_T.
 */
#define POWER_IC_IOCTL_GET_POWER_UP_REASON \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CORE_BASE + 0x05), POWER_IC_POWER_UP_REASON_T *) 


/*!
 * @brief Gets hardware information.
 *
 * This command fetches information about the hardware's type and any revision information 
 * that might be available for the hardware.
 *
 * The command has no inputs.
 * 
 * The command passes back the power IC hardware type and revision information in the
 * POWER_IC_HARDWARE_T structure, which is passed by pointer.
 *
 * @note Under most circumstances, a caller does not need to know whether the power IC is a
 * PCAP or Atlas or something else. This is only intended for the few cases where more
 * information is needed about the hardware, e.g. to interpret the results of an AtoD conversion
 * or similar. <b>Please do not make unnecessary use of this command.</b>
 */
#define POWER_IC_IOCTL_GET_HARDWARE_INFO \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CORE_BASE + 0x06), POWER_IC_HARDWARE_T *) 

/* Add new core ioctls above this line... **********************************************************/
        
/*!
 * @brief Displays timing information for debugging purposes.
 *
 * Ordinarily, this command does nothing. If called, an error will be returned. However, this
 * command is a placeholder for developers that want to measure timing information within the
 * driver and need a means to get that timing information during a quiet period. Developers
 * wishing to use this should fill out the placeholder in the core ioctl handler to print
 * information to the console.
 *
 * The command takes a single bool, which is used by the timing module's output functions
 * to determine whether to show all information (including all times recorded) or just to
 * display summaries.
 * 
 * This command has no output other than the returned error code for the ioctl() call.
 */
#define POWER_IC_IOCTL_TIMING_SHOW_INFO \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CORE_BASE + 0x0E), bool)
/*!
 * @brief Resets debugging timing information.
 *
 * Ordinarily, this command does nothing. If called, an error will be returned. However, this
 * command is a placeholder for developers that want to measure timing information within the
 * driver and need a means to get that timing information during a quiet period. Developers
 * wishing to use this should fill out the placeholder in the core ioctl handler to reset the
 * timers used.
 *
 * This command takes no inputs.
 * 
 * The command has no output other than the returned error code for the ioctl() call.
 */
#define POWER_IC_IOCTL_TIMING_RESET \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CORE_BASE + 0x0F), void)
/* @} End of core ioctls.  ------------------------------------------------------------------------*/

/*!
 * @anchor ioctl_periph
 * @name Peripheral ioctl() commands
 *
 * These are the commands that can be used through ioctl() to perform various peripheral
 * operations. The caller does not need to know which platform these commands are being 
 * performed on - the driver handles the differences between PCAP and Atlas.
 */

/* @{ */

/*!
 * @brief Controls power to Bluetooth.
 *
 * This command controls the regulator that powers the Bluetooth hardware, allowing the
 * caller to turn the hardware on and off.
 *
 * The command takes a single POWER_IC_PERIPH_ONOFF_T, indicating whether Bluetooth should be
 * turned on or off.
 * 
 * This command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_PERIPH_SET_BLUETOOTH_ON \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_PERIPH_BASE + 0x00), POWER_IC_PERIPH_ONOFF_T)

/*!
 * @brief Controls power to media card.
 *
 * This command controls the regulator that powers a removable media card, allowing the
 * caller to turn the hardware on and off.
 *
 * The command takes a single POWER_IC_PERIPH_ONOFF_T, indicating whether the media should be
 * turned on or off.
 * 
 * The command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_PERIPH_SET_FLASH_CARD_ON \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_PERIPH_BASE + 0x01), POWER_IC_PERIPH_ONOFF_T)        

/*!
 * @brief Sets the vibrator level.
 *
 * This command sets the level at which the vibrator should spin when it is turned on.
 *
 * This command takes a single int, indicating the level at which the vibrator should turn.
 * For both PCAP and Atlas, the available range of levels is 0..3, with higher levels resulting
 * in the vibrator spinning faster. 
 * 
 * This command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_PERIPH_SET_VIBRATOR_LEVEL \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_PERIPH_BASE + 0x02), int)
        
/*!
 * @brief Turns the vibrator on and off.
 *
 * This command is the on/off switch for the vibrator. When on, the vibrator will spin at the
 * level set by POWER_IC_IOCTL_PERIPH_SET_VIBRATOR_LEVEL.
 *
 * This command takes a single POWER_IC_PERIPH_ONOFF_T, indicating whether the vibrator should be
 * turned on or off.
 * 
 * This command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_PERIPH_SET_VIBRATOR_ON \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_PERIPH_BASE + 0x03), POWER_IC_PERIPH_ONOFF_T)
        
/*!
 * @brief Controls WLAN power.
 *
 * This command controls power to the wireless LAN hardware. This can only set both supplies
 * (the WLAN core and the RF hardware) on or off. If the control side of the LAN hardware must
 * be turned on without powering the RF hardware (e.g. for airplane mode) then the
 * POWER_IC_IOCTL_PERIPH_SET_WLAN_LOW_POWER_STATE_ON should be used instead.
 *
 * The command takes a single POWER_IC_PERIPH_ONOFF_T, indicating whether the WLAN hardware
 * should be turned on or off.
 * 
 * The command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_PERIPH_SET_WLAN_ON \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_PERIPH_BASE + 0x04),      POWER_IC_PERIPH_ONOFF_T)
        
/*!
 * @brief Sets WLAN low power mode.
 *
 * This command turns on power only to the WLAN control logic, not the RF hardware. This is
 * intended for placing the WLAN hardware in low-power mode where the RF hardware is not
 * needed, or to power up the control hardware without the RF hardware being turned on.
 *
 * This command takes no inputs.
 * 
 * This command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_PERIPH_SET_WLAN_LOW_POWER_STATE_ON \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_PERIPH_BASE + 0x05), void)
        
/*!
 * @brief Controls power to Camera.
 *
 * This command controls the regulator that powers the Camera hardware, allowing the
 * caller to turn the hardware on and off.
 *
 * The command takes a single POWER_IC_PERIPH_ONOFF_T, indicating whether the Camera should be
 * turned on or off.
 * 
 * This command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_PERIPH_SET_CAMERA_ON \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_PERIPH_BASE + 0x06), POWER_IC_PERIPH_ONOFF_T)        
/* @} End of power management ioctls.  ------------------------------------------------------------*/


/*!
 * @anchor ioctl_rtc
 * @name Real-time clock (RTC) ioctl() commands
 *
 * These are the commands that can be used through ioctl() to perform various operations on 
 * the real-time clock. The caller does not need to know which platform these commands are being 
 * performed on - the driver handles the differences between PCAP and Atlas.
 */

/* @{ */

/*!
 * @brief Gets the current RTC time.
 *
 * This command reads the current time from the hardware.
 *
 * This command takes no inputs.
 * 
 * The command passes back the current time in the passed timeval structure. The time
 * is expressed in the unix-standard number of seconds since January, 1 1970 00:00:00 UTC.
 */ 
#define POWER_IC_IOCTL_GET_TIME \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_RTC_BASE + 0x00), struct timeval *) 

/*!
 * @brief Sets the current RTC time.
 *
 * This command writes the passed time to the hardware RTC.
 *
 * The command takes the current time in the passed timeval structure. The time should be
 * expressed in the unix-standard number of seconds since January, 1 1970 00:00:00 UTC.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_SET_TIME \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_RTC_BASE + 0x01), struct timeval *) 

/*!
 * @brief Gets the currently set RTC alarm.
 *
 * This command reads the current programmed alarm from the hardware.
 *
 * This command takes no inputs.
 * 
 * The command passes back the alarm time in the passed timeval structure. The time
 * is expressed in the unix-standard number of seconds since January, 1 1970 00:00:00 UTC.
 */ 
#define POWER_IC_IOCTL_GET_ALARM_TIME \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_RTC_BASE + 0x02), struct timeval *)
        
/*!
 * @brief Programs the RTC alarm.
 *
 * This command writes the passed alarm time to the hardware RTC.
 *
 * The command takes the alarm time in the passed timeval structure. The time should be
 * expressed in the unix-standard number of seconds since January, 1 1970 00:00:00 UTC.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_SET_ALARM_TIME \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_RTC_BASE + 0x03), struct timeval *) 
/* @} End of RTC ioctls.  -------------------------------------------------------------------------*/

/*!
 * @anchor ioctl_atod
 * @name Analogue to Digital converter (AtoD) ioctl() commands
 *
 * These are the commands that can be used through ioctl() to perform various AtoD 
 * conversions. The caller does not need to know which platform these commands are 
 * being performed on in order to perform a conversion, but may need to know to
 * interpret the measurements correctly.
 */

/* @{ */

/*!
 * @brief Converts a single channel.
 *
 * This command performs a conversion on a single AtoD channel. The results of the 
 * conversion are averaged and phasing is applied to the result (if available for
 * the converted channel).
 *
 * The command takes the channel to be converted in the channel field of the passed
 * POWER_IC_ATOD_REQUEST_SINGLE_CHANNEL_T structure.
 *
 * The averaged (and possibly phased) AtoD measurement is returned in the result field of the 
 * structure.
 */ 
#define POWER_IC_IOCTL_ATOD_SINGLE_CHANNEL \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_ATOD_BASE + 0x00), POWER_IC_ATOD_REQUEST_SINGLE_CHANNEL_T *) 
  
/*!
 * @brief Converts battery voltage and current.
 *
 * This command performs a conversion of the battery voltage and current. The results of the 
 * conversion are averaged and phasing is applied to the result (if available).
 *
 * The command takes the expected direction of current flow in the polarity field of
 * the passed POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T structure (this is only relevant for PCAP
 * - this is due to a PCAP hardware limitation).
 *
 * The averaged (and possibly phased) AtoD measurements for the battery voltage and current are
 * passed back in the batt_result and curr_result fields of the structure.  The current value passed 
 * back has been converted to milliamps.
 */       
#define POWER_IC_IOCTL_ATOD_BATT_AND_CURR \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_ATOD_BASE + 0x01), POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) 

/*!
 * @brief Converts set of channels.
 *
 * This command performs a conversion of a set of AtoD channels, taking one sample for each
 * channel converted. The measurements are phased where phasing is available.
 *
 * The command takes no inputs.
 *
 * The results for all of the converted channels are passed back to the caller in the
 * POWER_IC_ATOD_RESULT_GENERAL_CONVERSION_T structure passed by pointer to the ioctl() call.
 */      
#define POWER_IC_IOCTL_ATOD_GENERAL \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_ATOD_BASE + 0x02), POWER_IC_ATOD_RESULT_GENERAL_CONVERSION_T *) 

/*!
 * @brief Converts touchscreen position.
 *
 * This command performs a conversion for the x and y positions on the touchscreen. No averaging
 * nor phasing is done by the command - x and y are both single unphased AtoD measurements.
 *
 * The command takes no inputs.
 *
 * The results for the position are passed back to the caller in the x and y fields of the
 * POWER_IC_ATOD_RESULT_TOUCHSCREEN_T structure passed by pointer to the ioctl() call.
 */      
#define POWER_IC_IOCTL_ATOD_TOUCHSCREEN_POSITION \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_ATOD_BASE + 0x03), POWER_IC_ATOD_RESULT_TOUCHSCREEN_T *) 

/*!
 * @brief Converts touchscreen pressure.
 *
 * This command performs a conversion for the touch pressure on the touchscreen. No averaging
 * nor phasing is done by the command - the pressure is a single unphased AtoD measurement.
 *
 * The command takes no inputs.
 *
 * The result for the pressure is passed back to the caller in the pressure field of the
 * POWER_IC_ATOD_RESULT_TOUCHSCREEN_T structure passed by pointer to the ioctl() call.
 */  
#define POWER_IC_IOCTL_ATOD_TOUCHSCREEN_PRESSURE \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_ATOD_BASE + 0x04), POWER_IC_ATOD_RESULT_TOUCHSCREEN_T *) 

/*!
 * @brief Enables a hardware-timed conversion.
 *
 * This command is different from the other conversions in that no conversion is performed
 * within the duration of the command. This command sets the hardware to perform a conversion
 * of the battery voltage and current that is triggered relative to a transmit burst. Once 
 * set, this command returns and the hardware waits for the transmit burst to occur. If
 * no transmit occurs, the conversion will never be performed. The conversion results can
 * be polled by using the read() system call on the power IC driver.
 *
 * The command takes the expected direction of current flow in the polarity field of
 * the passed POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T structure (this is used only for PCAP and
 * ignored for Atlas - this is due to a PCAP hardware limitation), and the timing of the
 * conversion (in/out of burst) is specified in the timing field of the structure.
 *
 * The command has no output other than the returned error code for the ioctl() call. The
 * status of the conversion and the results are retrieved via the read() system call on the
 * power IC device.
 */       
#define POWER_IC_IOCTL_ATOD_BEGIN_CONVERSION \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_ATOD_BASE + 0x05), POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *)         

/*!
 * @brief Cancels a hardware-timed conversion.
 *
 * This command disables a hardware-timed conversion previously set up by the 
 * POWER_IC_IOCTL_ATOD_BEGIN_CONVERSION command. If no conversion has been set up
 * or a conversion has already been completed, the command will return an error.
 *
 * The command takes no inputs.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 */     
#define POWER_IC_IOCTL_ATOD_CANCEL_CONVERSION \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_ATOD_BASE + 0x06), int)

/*!
 * @brief Programs the AtoD phasing.
 *
 * This command programs the phasing applied to various AtoD conversion results. The
 * phasing applied is an array of 12 bytes in the same format as is used for all
 * products' AtoD phasing.
 *
 * The command takes a pointer to an array of phasing bytes.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 *
 * @note The driver will reject all phasing that is deemed to be unprogrammed. If 
 * any slope byte is seen as 0x00 or 0xFF, the phasing will be rejected and the 
 * driver will continue to use the previously programmed values.
 */  
#define POWER_IC_IOCTL_ATOD_SET_PHASING \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_ATOD_BASE + 0x07), unsigned char *)               


/*!
 * @brief Converts battery voltage and current for Phasing.
 *
 * This command performs a conversion of the battery voltage and current. The results of the 
 * conversion are averaged and phasing is applied to the result (if available).
 *
 * The command takes the expected direction of current flow in the polarity field of
 * the passed POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T structure (this is only relevant for PCAP
 * - this is due to a PCAP hardware limitation).
 *
 * The averaged (and possibly phased) AtoD measurements for the battery voltage and current are
 * passed back in the batt_result and curr_result fields of the structure.  The current value passed 
 * back is in DAC values.
 */       
#define POWER_IC_IOCTL_ATOD_BATT_AND_CURR_PHASED \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_ATOD_BASE + 0x09), POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *)


/*!
 * @brief Converts a single channel.
 *
 * This command performs a conversion on a single AtoD channel. However, unlike the
 * POWER_IC_IOCTL_ATOD_SINGLE_CHANNEL the results are not averaged nor are they phased.
 * Instead, all of the samples will be passed back to the caller.
 *
 * The command takes the channel to be converted in the channel field of the passed
 * POWER_IC_ATOD_REQUEST_RAW_T structure.
 *
 * The command passes back all of the results in the results[] field of the passed structure
 * and number of samples taken (varies between Atlas/PCAP) will be recorded in the num_results
 * field.
 *
 * @note This is really only intended for testing purposes. Since the results are unphased and
 * not averaged, making decisions on the results of this conversion is probably not a good idea.
 */ 
#define POWER_IC_IOCTL_ATOD_RAW \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_ATOD_BASE + 0x0F), POWER_IC_ATOD_REQUEST_RAW_T *)
/* @} End of AtoD ioctls ------------------------------------------------------------------------- */

/*!
 * @anchor ioctl_pwr_mgmt
 *
 * @name Power management ioctl() commands
 *
 * These are the commands that can be used through ioctl() to perform various power management
 * operations on the hardware.
 */

/* @{ */

/*!
 * @brief Sets the AP core voltage.
 *
 * This command programs the regulator that provides the supply to the core of the AP. 
 * The command will ensure that the voltage requested is safe. For safe settings, the driver
 * will set the regulator to the next highest voltage supported in hardware (i.e. you may not
 * get exactly the voltage requested, but the supply is guaranteed to be at least that voltage).
 *
 * The command takes an int, which is the requested core voltage in millivolts.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 *
 * @note After setting the core voltage, this command will delay for an additional short period
 * of time before returning. This delay (in the order of hundreds of microseconds) is to allow
 * the regualtor to settle at the new level before any other operations are performed (such as
 * increasing the core frequency, etc.)
 */ 
#define POWER_IC_IOCTL_PMM_SET_CORE_VOLTAGE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_PMM_BASE + 0x00), int )	

/* @} End of power management ioctls.  ------------------------------------------------------------*/

/*!
 * @name Audio ioctl() commands
 * @anchor ioctl_audio
 *
 * These are the commands that can be used through ioctl() to perform various audio-related
 * operations. The caller does not need to know which platform these commands are 
 * being performed on - the driver handles the differences between PCAP and Atlas.
 *
 * @todo The audio ioctl commands need to be documented!
 */

/* @{ */
#define POWER_IC_IOCTL_AUDIO_WRITE_OUTPATH \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x00), unsigned int)
#define POWER_IC_IOCTL_AUDIO_READ_OUTPATH \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x01), unsigned int *)
#define POWER_IC_IOCTL_AUDIO_WRITE_INPATH \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x02), POWER_IC_AUD_IN_PATH_T)
#define POWER_IC_IOCTL_AUDIO_READ_INPATH \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x03), POWER_IC_AUD_IN_PATH_T *)
#define POWER_IC_IOCTL_AUDIO_WRITE_IGAIN \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x04), int)
#define POWER_IC_IOCTL_AUDIO_READ_INGAIN \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x05), int *)
#define POWER_IC_IOCTL_AUDIO_WRITE_OGAIN \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x06), int)
#define POWER_IC_IOCTL_AUDIO_READ_OGAIN \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x07), int *)
#define POWER_IC_IOCTL_AUDIO_CODEC_EN \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x08), bool)
#define POWER_IC_IOCTL_AUDIO_ST_DAC_EN \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x09), bool)
#define POWER_IC_IOCTL_AUDIO_SET_CODEC_SM \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x0A), bool)
#define POWER_IC_IOCTL_AUDIO_SET_ST_DAC_SM \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x0B), bool)
#define POWER_IC_IOCTL_AUDIO_SET_ST_DAC_SR \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x0C), POWER_IC_ST_DAC_SR_T)
#define POWER_IC_IOCTL_AUDIO_SET_CODEC_SR \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x0D), POWER_IC_CODEC_SR_T)
#define POWER_IC_IOCTL_AUDIO_CONFIG_CODEC_LP \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x0E), bool)
#define POWER_IC_IOCTL_AUDIO_SET_OUTPUT_HP \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x0F), bool)
#define POWER_IC_IOCTL_AUDIO_SET_INPUT_HP \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x10), bool)
#define POWER_IC_IOCTL_AUDIO_DITHER_EN \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x11), bool)
#define POWER_IC_IOCTL_AUDIO_SET_ST_DAC_NET_MODE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x12), int)
#define POWER_IC_IOCTL_AUDIO_SET_ST_DAC_NUM_TS \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x13), int)
#define POWER_IC_IOCTL_AUDIO_POWER_OFF \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x14), void)
#define POWER_IC_IOCTL_AUDIO_ST_DAC_RESET \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x15), void)
#define POWER_IC_IOCTL_AUDIO_CODEC_RESET \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_AUDIO_BASE + 0x16), void)
/* @} End of Audio ioctls.  -----------------------------------------------------------------------*/

/*!
 * @name Lighting ioctl() commands
 * @anchor ioctl_lights
 *
 * These are the commands that can be used through ioctl() to perform various operations. 
 * on the phone's backlights/funlights. The caller does not need to know which platform these
 * commands are being performed on - the driver handles the differences between PCAP and Atlas.
 *
 * @todo The lighting ioctl commands need to be documented!
 */

/* @{ */
#define POWER_IC_IOCTL_LIGHTS_BACKLIGHTS_SET \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_LIGHTS_BASE + 0x00), LIGHTS_BACKLIGHT_SET_T *)
#define POWER_IC_IOCTL_LIGHTS_FL_SET_CONTROL \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_LIGHTS_BASE + 0x01), LIGHTS_FL_SET_T *)
#define POWER_IC_IOCTL_LIGHTS_FL_UPDATE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_LIGHTS_BASE + 0x02), LIGHTS_FL_UPDATE_T *)
/* @} End of lighting ioctls.  -------------------------------------------------------------------- */

/*!
 * @anchor ioctl_charger
 * @name Charger ioctl() commands
 *
 * These are the commands that can be used through ioctl() to control the charging hardware.
 */

/* @{ */

/*!
 * @brief Sets the charge voltage.
 *
 * This command programs the maximum voltage that the battery will be charged up to. 
 * While it is expected that the main 4.2V level used will never change, there are 
 * potentially some differences in the other levels used depending on the hardware type
 * and revision.
 *
 * The command takes an int, which is the requested setting for VCHRG as per the hardware
 * specification.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_CHARGER_SET_CHARGE_VOLTAGE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CHARGER_BASE + 0x00), int)
        
/*!
 * @brief Sets the main charge current.
 *
 * This command programs the maximum current that will be fed into the battery while
 * charging. This is only an upper limit - the charging hardware is free to charge
 * at a limit lower than that set depending on the battery voltage and how much current
 * an attached charger can supply.
 *
 * The command takes an int, which is the requested setting for ICHRG as per the hardware
 * specification. For the most part this translates into 100's of milliamps of current, 
 * but there are discontinuities at the top end of the range.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 *
 * @note The main charge current and trickle charge current settings are mutually exclusive.
 * Setting this control will result in the trickle charge current being set to zero.
 */ 
#define POWER_IC_IOCTL_CHARGER_SET_CHARGE_CURRENT \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CHARGER_BASE + 0x01), int)

/*!
 * @brief Sets the trickle charge current.
 *
 * This command programs the maximum trickle current that will be fed into the battery 
 * while charging. This is only an upper limit - the charging hardware is free to charge
 * at a limit lower than that set depending on the battery voltage and how much current
 * an attached charger can supply.
 *
 * The command takes an int, which is the requested setting for ICHRG_TR as per the hardware
 * specification. This should translate to roughly 12 mA of current per count.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 *
 * @note The trickle charge current and main charge current settings are mutually exclusive.
 * Setting this control will result in the main charge current being set to zero.
 */ 
#define POWER_IC_IOCTL_CHARGER_SET_TRICKLE_CURRENT \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CHARGER_BASE + 0x02), int)

/*!
 * @brief Sets the power path.
 *
 * This command sets the path that supplies power to the phone. The hardware can be set
 * to either supply current from the battery (current share) or from the attached charger
 * (dual-path).
 *
 * The command takes an POWER_IC_CHARGER_POWER_PATH_T, indicates whether the hardware should
 * be set in current-share or dual-path mode.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 *
 * @note Setting current-share mode when no battery is attached will result in an instant
 * powerdown.
 */ 
#define POWER_IC_IOCTL_CHARGER_SET_POWER_PATH \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CHARGER_BASE + 0x03), POWER_IC_CHARGER_POWER_PATH_T) 
        
/*!
 * @brief Reads overvoltage state.
 *
 * This function reads the charger overvoltage sense bit from the hardware and returns it
 * to the caller.
 *
 * The command expects a pointer to an int, where the command will store the read state of the
 * overvoltage detect hardware. This will be zero if no overvoltage condition existsm and 
 * greater than zero if overvoltage has been detected.
 *
 * This command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_CHARGER_GET_OVERVOLTAGE \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CHARGER_BASE + 0x04), int *) 
        
/*!
 * @brief Resets the overvoltage hardware.
 *
 * This function is used to reset the overvoltage hardware after a problem is detected.
 * once an overvoltage condition occurs, charging will be disabled in hardware until it is
 * reset.
 *
 * This commend takes no inputs.
 *
 * This command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_CHARGER_RESET_OVERVOLTAGE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_CHARGER_BASE + 0x05), void)         
/* @} End of charger control ioctls.  -------------------------------------------------------------*/

/*!
 * @anchor ioctl_touchscreen
 * @name Touchscreen ioctl() commands
 *
 * These are the commands that can be used through ioctl() to control touchscreen operation.
 * The caller does not need to know which platform these commands are being performed on - 
 * the driver handles the differences between PCAP and Atlas.
 */

/* @{ */

/*!
 * @brief Enables the touchscreen.
 *
 * This command turns the touchscreen on. When on, the hardware will be set in a
 * mode that generates an interrupt when the screen is pressed, and the driver thread
 * will handle presses, drags and releases. It is assumed that any product that has a 
 * flip over the touchscreen will disable the touchscreen when the flip is closed
 * to save current and will turn the touchscreen on when the flip is opened.
 *
 * The command takes no inputs.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_TS_ENABLE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TOUCHSCREEN_BASE + 0x00), void)

/*!
 * @brief Disables the touchscreen.
 *
 * This command turns the touchscreen off. When disabled, the touchscreen hardware will
 * be turned off and no touchscreen processing will be done. It is assumed that any 
 * product that has a flip over the touchscreen will disable the touchscreen  when the
 * flip is closed to save current and will turn the touchscreen on when the flip is opened.
 *
 * The command takes no inputs.
 *
 * The command has no output other than the returned error code for the ioctl() call.
 */ 
#define POWER_IC_IOCTL_TS_DISABLE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TOUCHSCREEN_BASE + 0x01), void)
        
/*!
 * @brief Sets the touchscreen poll interval.
 *
 * This command sets the poll interval at which position updates will be performed when
 * the touchscreen is pressed and held.
 *
 * The command takes an int, which is the requested interval in milliseconds.
 *
 * The command has no output, but the returned value through ioctl() indicate the period
 * used for the poll interval.
 *
 * @note Due to timing constraints in the kernel, the actual poll interval used will likely
 * not be exactly that set. The lowest interval allowed will correspond to 1 jiffy within the
 * kernel, which is typically in the order of 10ms in length.
 */ 
#define POWER_IC_IOCTL_TS_SET_INTERVAL \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TOUCHSCREEN_BASE + 0x02), int)
/* @} End of touchscreen ioctls.  -----------------------------------------------------------------*/

/*!
 * @name Test ioctl() commands
 * @anchor ioctl_tcmd
 *
 * These are the commands that are not normally used as part of the phone's normal 
 * operation, but are intended for use in test commands.
 *
 * @todo The tcmd ioctl commands need to be better documented.
 */
/* @{ */

/*! Configures the transceiver to the requested state, if an out of range state is passed
 *  the off state is used.  */
#define POWER_IC_IOCTL_CMD_TCMD_EMU_TRANS_STATE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x00), EMU_XCVR_T)
/*! Sets the MONO adder to the requested state, if an invalid state is requested nothing 
 * is done to hardware and -EFAULT is returned to user. */
#define POWER_IC_IOCTL_CMD_TCMD_MONO_ADDER_STATE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x01), POWER_IC_TCMD_MONO_ADDER_T)
/*! Returns the current value of the IDFLOATS bit */
#define POWER_IC_IOCTL_CMD_TCMD_IDFLOATS_READ \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x02), unsigned int *)
/*! Returns the current value of the IDGNDS bit */
#define POWER_IC_IOCTL_CMD_TCMD_IDGNDS_READ \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x03), unsigned int *)
/*! Returns the current value of the headset detect bit */
#define POWER_IC_IOCTL_CMD_TCMD_A1SNS_READ \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x04), unsigned int *)
/*! Returns the current value of the mic bias MB2SNS bit */	
#define POWER_IC_IOCTL_CMD_TCMD_MB2SNS_READ \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x05), unsigned int *)
/*! Passing 1 sets the reverse mode to on passing 0 sets it to off */
#define POWER_IC_IOCTL_CMD_TCMD_REVERSE_MODE_STATE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x06), int)
/*! Returns the current value of the bits that determine the source of the VUSB input */
#define POWER_IC_IOCTL_CMD_TCMD_VUSBIN_READ \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x07), unsigned int *)
/*! Set the bits that determine the source of the VUSB input */
#define POWER_IC_IOCTL_CMD_TCMD_VUSBIN_STATE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x08), int)
/*! Returns 1 if the external 32KHz oscillator is present and 0 if it is not */
#define POWER_IC_IOCTL_CMD_TCMD_CLKSTAT_READ \
        _IOR(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x09), unsigned int *)
/*! Passing 1 Enables the charging of the coincell passing 0 sets it to no charge */
#define POWER_IC_IOCTL_CMD_TCMD_COINCHEN_STATE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x0A), int)
/*! Sets the CONN mode to the requested state, if an invalid state is requested nothing 
 * is done to hardware and -EFAULT is returned to user. */
#define POWER_IC_IOCTL_CMD_TCMD_EMU_CONN_STATE \
        _IOW(POWER_IC_MAJOR_NUM, (POWER_IC_IOC_CMD_TCMD_BASE + 0x0B), EMU_CONN_MODE_T)
/* @} End of tcmd ioctls  -------------------------------------------------------------------------*/

/*!
 * @anchor ioctl_types
 * @name ioctl() types
 *
 * These are the types of data passed in various ioctl() requests.
 */

/* @{ */

/*!
 * @brief Type of data used for power IC register access.
 *
 * This structure is passed as the final parameter to ioctl() when using the commands:
 * POWER_IC_IOCTL_READ_REG, POWER_IC_IOCTL_WRITE_REG, POWER_IC_IOCTL_READ_REG_BITS,
 * POWER_IC_IOCTL_WRITE_REG_BITS, or POWER_IC_IOCTL_WRITE_REG_MASK.
 */
typedef struct
{
    POWER_IC_REGISTER_T reg; /*!< Register to be read/written. */
    int index;               /*!< Index of first bit for read/write bits operations. */
    int num_bits;            /*!< Number of bits to be written for r/w bits, or bitmask for write mask. */
    int value;               /*!< Value written for writes, value read put here for reads. */
} POWER_IC_REG_ACCESS_T;

/*!
 * @brief Chipsets reported by POWER_IC_IOCTL_GET_HARDWARE_INFO.
 *
 * This is the set of chipsets that can be reported in the POWER_IC_HARDWARE_T retrieved by the
 * POWER_IC_IOCTL_GET_HARDWARE_INFO command. Under normal circumstances, POWER_IC_CHIPSET_UNKNOWN
 * should never be reported.
 */
typedef enum
{
    POWER_IC_CHIPSET_PCAP2_EOC,
    POWER_IC_CHIPSET_ATLAS,
    
    /* Add new chipsets above this line. */
    POWER_IC_CHIPSET_UNKNOWN = 0xFF
} POWER_IC_CHIPSET_T;

/*!
 * @brief Type returned by POWER_IC_IOCTL_GET_HARDWARE_INFO.
 *
 * This is all of the information retrieved by the POWER_IC_IOCTL_GET_HARDWARE_INFO command.
 * For the two sets of revision history, revision1 will represent the main power IC (e.g.
 * PCAP or Atlas) and revision2 will represent other hardware alongside it (e.g. EMU one-chip
 * for our PCAP/EMU one-chip combination).
 *
 * @note For some hardware types, revision information is not available for the hardware. In
 * these cases, the corresponding revision will be set to POWER_IC_HARDWARE_NO_REV_INFO.
 */
typedef struct
{
    POWER_IC_CHIPSET_T chipset;  /*!< Indicates type of hardware. */
    int revision1;               /*!< Revision info for primary IC. */
    int revision2;               /*!< Revision info for secondary IC. */
} POWER_IC_HARDWARE_T;

/*! Revision info in POWER_IC_HARDWARE_T will be set to this if no information is available. */
#define POWER_IC_HARDWARE_NO_REV_INFO    0xFFFFFFFF

/* The following types are used in the AtoD interface. */

/*!
 * @brief General AtoD conversion results.
 *
 * This type is the set of AtoD conversions returned from a general conversion request.
 * this is passed by address as the 3rd parameter to a POWER_IC_IOCTL_ATOD_GENERAL 
 * ioctl() request.
 */
typedef struct
{
    int coin_cell;
    int battery;
    int bplus;
    int mobportb;
    int charger_id;
    int temperature;
} POWER_IC_ATOD_RESULT_GENERAL_CONVERSION_T;

/*!
 * @brief Touchscreen position  AtoD conversion results.
 *
 * This type holds the results of the touchscreen position/pressure AtoD measurement. This is
 * passed by address as the 3rd parameter to a POWER_IC_IOCTL_ATOD_TOUCHSCREEN
 * ioctl() request.
 */
typedef struct
{
    int x;
    int y;
    int pressure;
} POWER_IC_ATOD_RESULT_TOUCHSCREEN_T;

/*!
 * @brief Structure passed to ioctl() for a single channel request.
 *
 * This type is passed by pointer as the 3rd parameter of a 
 * POWER_IC_IOCTL_ATOD_SINGLE_CHANNEL ioctl() request.
 */
typedef struct
{
    POWER_IC_ATOD_CHANNEL_T channel; /*!< The channel that should be converted. */
    int                     result;  /*!< The result of the conversion, */
} POWER_IC_ATOD_REQUEST_SINGLE_CHANNEL_T;

/*!
 * @brief Structure passed to ioctl() for a battery/current request.
 *
 * This type is passed by pointer as the 3rd parameter of a 
 * POWER_IC_IOCTL_ATOD_BATT_AND_CURR ioctl() request.
 */
typedef struct
{
    POWER_IC_ATOD_TIMING_T timing;          /*!< The timing for the conversion. */
    int timeout_secs;                       /*!< If a non-immediate conversion, timeout after this number of seconds.*/
    POWER_IC_ATOD_CURR_POLARITY_T polarity; /*!< The direction in which current should be sampled. Only 
                                             *   relevant for PCAP. */
    int batt_result;                        /*!< The battery result from the conversion. */
    int curr_result;                        /*!< The current result from the conversion. */
} POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T;

/*!
 * @brief Structure passed to ioctl() for a raw conversion request.
 *
 * This type is passed by pointer as the 3rd parameter of a 
 * POWER_IC_IOCTL_ATOD_RAW ioctl() request.
 */
typedef struct
{
    POWER_IC_ATOD_CHANNEL_T channel; /*!< The channel that should be converted. */
    int                     results[POWER_IC_ATOD_NUM_RAW_RESULTS]; /*!< Results of conversion. */
    int                     num_results; /*!< Number of the results being passed back. */
} POWER_IC_ATOD_REQUEST_RAW_T;

/*! Defines the possible modes of the MONO adder, this table is tied to the
 * hardware, this case supports PCAP2 and Atlas.  The Mono adder can be used
 * to sum the left and right channels of the stereo DAC or signals supplied
 * to the left and right PGA inputs. The Mono adder can then attenuate the
 * summed signals by 0dB, 3dB or 6dB and an identical monophonic signal
 * to the output amplifiers.  
 *
 */
 
typedef enum
{
    POWER_IC_TCMD_MONO_ADDER_STEREO,        /*!< 00 - Right PGA and Left PGA Separated (Stereo) */
    POWER_IC_TCMD_MONO_ADDER_R_L,           /*!< 01 - Right PGA + Left PGA */
    POWER_IC_TCMD_MONO_ADDER_R_L_3DB_LOSS,  /*!< 10 - (Right PGA + Left PGA) -3 dB */
    POWER_IC_TCMD_MONO_ADDER_R_L_6DB_LOSS,  /*!< 11 - (Right PGA + Left PGA) -6 dB */

    POWER_IC_TCMD_MONO_ADDER_NUM
} POWER_IC_TCMD_MONO_ADDER_T;

/* @} End of ioctl types.  ------------------------------------------------------------------------*/

#ifdef __KERNEL__
/*!
 * @anchor kernel
 * @section kernel Kernel-space Interface
 *
 * The following macros, types and functions are only available from within the kernel. These
 * are inaccessible from user space.
 */
/*!
 * @anchor kernel_reg_macros
 * @name Register Macros
 *
 * The following macros are used to identify to which device a register belongs.
 */

/* @{ */

#ifdef CONFIG_MOT_POWER_IC_PCAP2
/*! @brief Returns 1 if given register is a PCAP register */
#define POWER_IC_REGISTER_IS_PCAP(reg) \
     (((reg) >= POWER_IC_REG_PCAP_FIRST_REG) && ((reg) <= POWER_IC_REG_PCAP_LAST_REG))

/*! @brief Returns 1 if given register is an EMU One Chip register */
#define POWER_IC_REGISTER_IS_EOC(reg) \
     (((reg) >= POWER_IC_REG_EOC_FIRST_REG) && ((reg) <= POWER_IC_REG_EOC_LAST_REG))

/*! @brief Returns 1 if given register is a Funlight register */
#define POWER_IC_REGISTER_IS_FL(reg) \
     (((reg) >= POWER_IC_REG_FL_FIRST_REG) && ((reg) <= POWER_IC_REG_FL_LAST_REG))
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
/*! @brief Returns 1 if given register is an Atlas register */
#define POWER_IC_REGISTER_IS_ATLAS(reg) \
     (((reg) >= POWER_IC_REG_ATLAS_FIRST_REG) && ((reg) <= POWER_IC_REG_ATLAS_LAST_REG))
#endif /* ATLAS */

/* @} End of register macros.  --------------------------------------------------------------------*/

/*!
 * @anchor kernel_types
 * @name Kernel-space types
 *
 * These types are only for use by the kernel-space interfaces.
 */

/* @{ */

/*! States to determine if Touchscreen should be Disabled, in Standby, Position,
 *  or Pressure Mode. 
 */
typedef enum
{
    POWER_IC_TS_MODE_DISABLED,
    POWER_IC_TS_MODE_LOW_PWR_MODE,
    POWER_IC_TS_MODE_FULL_PWR_MODE
} POWER_IC_TS_MODES_T;

/*! States to determine if the touchscreen has been pressed, the position, or if 
 *  it was released. 
 */
typedef enum
{
    POWER_IC_TS_PRESSED,
    POWER_IC_TS_POSITION,
    POWER_IC_TS_RELEASE,
    
    POWER_IC_TS_NUM
} POWER_IC_TS_CALLBACK_T;

/*!
 * The type of the callback functions used for events.  Function takes the
 * event number as its parameter and returns 0 if the event is NOT handled
 * by the function (and the next callback in the chain should be called) or
 * non-zero if the event is handled.
 */
typedef int (*POWER_IC_EVENT_CALLBACK_T)(POWER_IC_EVENT_T);

/*!
 * The type of callback functions that can be used for Touchscreen can either
 * take one or two parameters.  When the touchscreen callback function for 
 * position or pressure is called, both the x and y values will need to be 
 * passed.  When calling the release callback function, it is not necessary
 * to pass any parameters to the function.  These typedefs will be used
 * to cast the call to the callback in order to pass the correct number of 
 * parameters.
 */
typedef void (*POWER_IC_TS_PRESS_POSITION_T)(int x, int y);
typedef void (*POWER_IC_TS_RELEASE_T)(void);

/* @} End of kernel types  ------------------------------------------------------------------------*/

/*!
 * @anchor kernel_funcs_reg
 * @name Kernel-space register access functions
 *
 * These functions are exported by the driver to allow other kernel-space code to
 * have register-level access to the power IC.  For more information, see the
 * documentation for the external.c file.
 */

/* @{ */

int power_ic_read_reg(POWER_IC_REGISTER_T reg, unsigned int *reg_value);
int power_ic_write_reg(POWER_IC_REGISTER_T reg, unsigned int *reg_value);
int power_ic_write_reg_value(POWER_IC_REGISTER_T reg, unsigned int reg_value);
int power_ic_set_reg_value(POWER_IC_REGISTER_T reg, int index, int value, int nb_bits);
int power_ic_get_reg_value(POWER_IC_REGISTER_T reg, int index, int *value, int nb_bits);
int power_ic_set_reg_mask(POWER_IC_REGISTER_T reg, int mask, int value);
int power_ic_set_reg_bit(POWER_IC_REGISTER_T reg, int index, int value);

/* @} End of kernel register access functions -----------------------------------------------------*/

/*!
 * @anchor kernel_funcs_event
 * @name Kernel-space event handling functions
 *
 * These functions are exported by the driver to allow other kernel-space code to
 * subscribe to and manage power IC events.  For more information, see the
 * documentation for the event.c file.
 */

/* @{ */

int power_ic_event_subscribe (POWER_IC_EVENT_T event, POWER_IC_EVENT_CALLBACK_T callback);
int power_ic_event_unsubscribe (POWER_IC_EVENT_T event, POWER_IC_EVENT_CALLBACK_T callback);
int power_ic_event_unmask (POWER_IC_EVENT_T event);
int power_ic_event_mask (POWER_IC_EVENT_T event);
int power_ic_event_clear (POWER_IC_EVENT_T event);
int power_ic_event_sense_read (POWER_IC_EVENT_T event);

/* @} End of kernel event functions ---------------------------------------------------------------*/

/*!
 * @anchor kernel_funcs_periph
 * @name Kernel-space peripheral functions
 *
 * These functions are exported by the driver to allow other kernel-space code to
 * control power IC-related peripherals.  For more information, see the documentation
 * for the peripheral.c file.
 */

/* @{ */

int power_ic_periph_set_bluetooth_on(int on);
int power_ic_periph_set_flash_card_on(int on);
int power_ic_periph_set_vibrator_level(int level);
int power_ic_periph_set_vibrator_on(int on);
int power_ic_periph_is_usb_cable_connected (void);
int power_ic_periph_is_usb_pull_up_enabled (void);
void power_ic_periph_set_usb_pull_up (int on);
int power_ic_periph_set_wlan_on(int on);
int power_ic_periph_set_wlan_low_power_state_on(void);
int power_ic_periph_set_sim_voltage(unsigned char sim_card_num, POWER_IC_SIM_VOLTAGE_T volt);
int power_ic_periph_set_camera_on(int on);

/* @} End of kernel peripheral functions --------------------------------------------------------- */

/*!
 * @anchor kernel_funcs_rtc
 * @name Kernel-space RTC functions
 *
 * These functions are exported by the driver to allow other kernel-space code to
 * access power IC real-time clock information.  For more information, see the
 * documentation for the rtc.c file.
 */

/* @{ */

int power_ic_get_num_power_cuts(int * data);
int power_ic_rtc_set_time(struct timeval * power_ic_time);
int power_ic_rtc_get_time(struct timeval * power_ic_time);
int power_ic_rtc_set_time_alarm(struct timeval * power_ic_time);
int power_ic_rtc_get_time_alarm(struct timeval * power_ic_time);

/* @} End of kernel rtc functions ---------------------------------------------------------------- */


/*!
 * @anchor kernel_funcs_atod
 * @name Kernel-space AtoD functions
 *
 * These functions are exported by the driver to allow other kernel-space code to
 * perform AtoD conversions.  For more information, see the documentation for the 
 * atod.c file.
 */

/* @{ */
int power_ic_atod_single_channel(POWER_IC_ATOD_CHANNEL_T channel, int * result);
int power_ic_atod_general_conversion(POWER_IC_ATOD_RESULT_GENERAL_CONVERSION_T * result);
int power_ic_atod_current_and_batt_conversion(POWER_IC_ATOD_TIMING_T timing,
                                              int timeout_secs,
                                              POWER_IC_ATOD_CURR_POLARITY_T polarity,
                                              int * batt_result, int * curr_result);
int power_ic_atod_touchscreen_position_conversion(int * x, int * y);
int power_ic_atod_touchscreen_pressure_conversion(int * p);
int power_ic_atod_raw_conversion(POWER_IC_ATOD_CHANNEL_T channel, int * samples, int * length);
int power_ic_atod_set_touchscreen_mode(POWER_IC_TS_MODES_T mode);
/* @} End of kernel atod functions --------------------------------------------------------------- */

/*!
 * @anchor kernel_funcs_pwr_mgmt
 * @name Kernel-space power management functions
 *
 * These functions are exported by the driver to allow other kernel-space code to
 * perform power management functions. For more information, see the documentation
 * for the power_management.c file.
 */
 
/* @{ */
#ifdef CONFIG_MOT_POWER_IC_PCAP2
int power_ic_set_core_voltage(int millivolts);
#endif
/* @} End of power management functions.  ---------------------------------------------------------*/

/*!
 * @anchor kernel_funcs_audio
 * @name Kernel-space audio functions
 *
 * These functions are exported by the driver to allow other kernel-space code to
 * perform audio registers read/write.  For more information, see the documentation for the 
 * audio.c file.
 */

/* @{ */
int power_ic_audio_power_off(void);
int power_ic_audio_write_output_path(unsigned int out_type);
int power_ic_audio_read_output_path(void);
int power_ic_audio_write_input_path(POWER_IC_AUD_IN_PATH_T in_type);
POWER_IC_AUD_IN_PATH_T power_ic_audio_read_input_path(void);
int power_ic_audio_write_igain(int igain);
int power_ic_audio_read_igain(int *igain);
int power_ic_audio_write_ogain(int ogain);
int power_ic_audio_read_ogain(int* ogain);
int power_ic_audio_codec_en(bool val);
int power_ic_audio_st_dac_en(bool val);
int power_ic_audio_set_codec_sm(bool val);
int power_ic_audio_set_st_dac_sm(bool val);
int power_ic_audio_set_st_dac_sr(POWER_IC_ST_DAC_SR_T val);
int power_ic_audio_set_codec_sr(POWER_IC_CODEC_SR_T val);
int power_ic_audio_config_loopback(bool val);
int power_ic_audio_set_output_hp(bool val);
int power_ic_audio_set_input_hp(bool val);
int power_ic_audio_dither_en(bool val);
int power_ic_audio_set_st_dac_net_mode(int val);
int power_ic_audio_set_st_dac_num_ts(int val);
int power_ic_audio_st_dac_reset(void);
int power_ic_audio_codec_reset(void);
void power_ic_audio_conn_mode_set(unsigned int out_type, POWER_IC_AUD_IN_PATH_T in_type);

/* @} End of kernel audio functions -------------------------------------------------------------- */

/*!
 * @anchor kernel_funcs_charger
 * @name Kernel-space charger control functions
 *
 * These functions are exported by the driver to allow other kernel-space code to control
 * various charging capabilities.  For more information, see the documentation for the 
 * charger.c file.
 */

/* @{ */
int power_ic_charger_set_charge_voltage(int charge_voltage);
int power_ic_charger_set_charge_current(int charge_current);
int power_ic_charger_set_trickle_current(int charge_current);
int power_ic_charger_set_power_path(POWER_IC_CHARGER_POWER_PATH_T path);
int power_ic_charger_get_overvoltage(int * overvoltage);
int power_ic_charger_reset_overvoltage(void);
/* @} End of kernel charging functions ----------------------------------------------------------- */

/*!
 * @anchor kernel_funcs_touchscreen
 * @name Kernel-space touchscreen functions
 *
 * These functions are exported by the driver to allow other kernel-space code to
 * control various touchscreen capabilities.  For more information, see the documentation
 * for the touchscreen.c file.
 */

/* @{ */
void * power_ic_touchscreen_register_callback(POWER_IC_TS_CALLBACK_T callback, void * reg_ts_fcn);
int power_ic_touchscreen_set_repeat_interval(int period);
void power_ic_touchscreen_enable(bool on);
/* @} End of kernel touchscreen functions -------------------------------------------------------- */

#endif /* __KERNEL__ */

#endif /* __POWER_IC_H__ */
