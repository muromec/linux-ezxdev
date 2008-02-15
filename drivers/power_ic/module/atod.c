/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2004-2005 - Motorola
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
 * Motorola 2004-Dec-17 - Rewrote software for a less generic interface.
 * Motorola 2005-Feb-28 - Add more robust routines to work around 
 *                        hardware issues.
 * Motorola 2005-May-12 - Added phasing capability.
 *
 */

/*!
 * @file atod.c
 *
 * @ingroup poweric_atod
 * 
 * @brief Power IC AtoD converter management interface.
 *
 * This file contains all of the functions and internal data structures needed
 * to implement the power IC AtoD interface. This includes:
 *
 *    - The low-level functions required to ensure exclusive hardware access. 
 *    - The public functions that can be used to request AtoD conversions from kernel-space.
 *    - The ioctl() handler called from the driver core to deal with requests from user-space.
 *
 * The main responsibilitiy of the interface is to ensure that simultaneous requests are 
 * handled appropriately. The biggest problem in managing these requests is that while most
 * requests by necessity must be blocked while a conversion is in progress, one type of request
 * (we're calling this a "postponable" request) must cede the hardware to the others if necessary
 * and be retried later.
 *
 * The request that can be postponed is the request for measurements of the battery and current.
 * In certain situations, this conversion must be timed relative to a transmit burst to either 
 * guarantee that the measurement was taken outside or inside the burst. The measurement inside
 * the burst is usually required in order to monitor the battery when close to the shutoff level.
 * Each time the transmitter is keyed, its load causes a significant sag in the battery voltage
 * which can result in the hardware shutting off the phone. This is guarded against by forcing
 * the hardware to take measurements of the battery during the transmit burst so that the phone 
 * can be shut down gracefully if necessary.
 *
 * If a GSM phone were guaranteed to transmit during every frame (4.6ms), then this might not be 
 * a problem. However, because of DTx (Discontinuous Transmission), this is not guaranteed. When
 * the user is not talking and DTx is enabled, the phone does not transmit each frame but instead
 * leaves the transmitter off, periodically updating background noise information instead. In the
 * worst case, this results in a series of bursts for a noise speech encoder frame approximately 
 * every half a second. This would mean that in the very worst case, the converter would be 
 * set up and the conversion would not occur until the next burst that was something like 500ms
 * later.
 *
 * Given a conversion is typically on the order of hundreds of microseconds in duration, a 
 * potential delay of hundreds of milliseconds is unacceptable, especially where the touchscreen
 * is involved - a half-second reaction time in performing the touchscreen position conversion
 * before any processing of the results is even done by the caller cannot be allowed.
 *
 * Ignoring the mechanism of the conversion request (function call in kernel-space vs. 
 * ioctl() from user-space), in general the two types of conversion are executed as follows.
 *
 * Exclusive conversion:
 * 
 *    - Obtain the hardware lock for hardware access (sleep until obtained).
 *    - Program the hardware for the specific conversion.
 *    - Start the conversion immediately.
 *    - While the conversion has not yet completed:
 *        - poll the hardware for completion.
 *    - Retrieve the results of the conversion from the hardware.
 *    - If a conversion was postponed, restart it.
 *    - Give up the hardware lock so that other conversions may occur.
 *    - Process the samples and return as necessary.
 *
 * This type of conversion will obtain a lock and will hold it for the duration of the conversion.
 * This lock must be obtained by a conversion request before the hardware can be manipulated, so
 * the thread that holds the lock can be assured that the entire process will be completed without
 * interference.
 *
 * There is no delay added to the polling loop other than that incurred in reading the state of
 * the ASC bit. This has been seen to take in the region of 50us and the total time spent waiting
 * in the loop is only about 150us, so there didn't appear to be much reason to add a udelay()
 * or equivalent.
 *
 * Initially a call to schedule() was planned to be executed within the body of the loop, but 
 * since the total polling time is so short, this was removed.
 *
 * The other type of conversion (postponable) will work alongside this.
 *
 * Postponable conversion:
 *
 *    - Obtain the postponable conversion lock (this will ensure only one postponable conversion 
 *      runs at a time), sleeping until obtained.
 *    - Obtain the hardware lock for hardware access (sleep until obtained).
 *    - Program the hardware for a hardware-timed converison
 *    - Give up the hardware lock so that an exclusive conversion may start.
 *    - Wait for the conversion to be completed (or for the timeout to expire).
 *    - Copy the results of the conversion.
 *    - Process the samples and return as necessary.
 *    - Give up the postponable conversion lock so that another postponable converison may occur.
 *
 * By giving up the hardware lock before waiting for the conversion to complete, this allows
 * another exclusive conversion to grab the hardware and interrupt this conversion. Because the
 * hardware lock is not held for the duration of this conversion, a second lock is required to
 * ensure that only one postponable conversion request can proceed at any given time.
 *
 * This requires that the code be written so that both conversions cooperate with each other.
 * In particular, a process that interrupts an postponable conversion must restart that
 * conversion once its exclusive conversion is done.
 *
 * The non-blocking version of a postponable conversion does not cause the calling process to
 * sleep while the conversion is in progress, but instead sets up the hardware for the conversion
 * and returns. The calling process can then later check for the conversion's completion by
 * poll()ing the device.
 * 
 * The end result of all this should be that all exclusive conversions should complete safely, 
 * and that they cannot be delayed in the case where another postponable conversion is waiting.
 *
 * IMPORTANT: the kernel-space functions assume that they are called from a thread as they will
 * cause that thread to sleep. To avoid kernel panics, these functions will check their context
 * and will fail if currently in an interrupt context.
 *
 * @todo The touchscreen interface may need additional work. At this time, only position is
 *       converted - we have no information on whether pressure or any other modes are required.
 */

#include <asm/delay.h>
#include <asm/errno.h>
#include <asm/semaphore.h>
#include <asm/uaccess.h>
#include <linux/completion.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/power_ic.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/types.h>
#include <stdbool.h>

#include "../core/os_independent.h"

#include "atod.h"

#ifndef DOXYGEN_SHOULD_SKIP_THIS
EXPORT_SYMBOL(power_ic_atod_single_channel);
EXPORT_SYMBOL(power_ic_atod_general_conversion);
EXPORT_SYMBOL(power_ic_atod_current_and_batt_conversion);
EXPORT_SYMBOL(power_ic_atod_touchscreen_position_conversion);
EXPORT_SYMBOL(power_ic_atod_raw_conversion);
#endif

/******************************************************************************
* Local constants
******************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
#ifdef CONFIG_MOT_POWER_IC_PCAP2
/* Bits in the batt control register. */
#define EXT_ISENSE_MASK        0x0000200

/* Bits in ADC1... */
#define ADEN_MASK              0x0000001
#define RAND_MASK              0x0000002
#define AD_SEL1_MASK           0x0000004
#define AD_SEL1_SHIFT          2
#define AD_SEL2_MASK           0x0000008
#define ADA1_MASK              0x0000070
#define ADA1_SHIFT             4
#define ADA2_MASK              0x0000380
#define ADA2_SHIFT             7
#define ATO_MASK               0x0003C00
#define ATO_SHIFT              10
#define ATOX_MASK              0x0004000
#define MTR1_MASK              0x0008000
#define MTR2_MASK              0x0010000
#define TS_M_MASK              0x00E0000
#define TS_M_SHIFT             17
#define TS_REF_LOW_PWR_MASK    0x0100000
#define TS_REFENB_MASK         0x0200000
#define BATT_I_POLARITY_MASK   0x0400000
#define BATT_I_ADC_MASK        0x0800000
#define TS_PRI_ADJ_MASK        0x1000000

/* Bits in ADC2. */
#define ADD1_MASK              0x00003FF
#define ADD1_SHIFT             0
#define ADD2_MASK              0x00FFC00
#define ADD2_SHIFT             10
#define ADINC1_MASK            0x0100000
#define ADINC2_MASK            0x0200000
#define ASC_MASK               0x0400000
#define ASC_SHIFT              22
#define ADTRIG_ONESHOT_MASK    0x0800000

/* Supported touchscreen modes. */
#define TOUCHSCREEN_POSITION   1
#define TOUCHSCREEN_PRESSURE   2
#define TOUCHSCREEN_STANDBY    5

/* The number of channels sampled in a single bank. */
#define SAMPLES_PER_BANK       7

/* The delay programmed for in-burst and out of burst conversions, in cycles of the
 * the  ADC_CLK/256 which for PCAP is 4.3MHz +/- 10%  or 15.12 to 18.48 KHz */
#define DELAY_IN_BURST      6
#define DELAY_OUT_OF_BURST  0

/* Used for register definitions to cut down on the amount of duplicate code */
#define GET_RESULTS_ADC           POWER_IC_REG_PCAP_ADC2
#define DISABLE_CONVERT_REG       POWER_IC_REG_PCAP_ADC1
#define ARM_ONE_SHOT_REG          POWER_IC_REG_PCAP_ADC2
#define START_CONVERT_REG_ASC     POWER_IC_REG_PCAP_ADC2
#define START_CONVERT_REG_ATO     POWER_IC_REG_PCAP_ADC1
#define TOUCHSCREEN_STANDBY_REG   POWER_IC_REG_PCAP_ADC1
#define TS_LOW_PWR_MOD_REG        POWER_IC_REG_PCAP_ADC1

#define EVENT_ADCDONEI            POWER_IC_EVENT_PCAP_ADCDONEI
#define EVENT_TSI                 POWER_IC_EVENT_PCAP_TSI    

/* An current AtoD is converted to millamps using these. According to hardware team,
 * current in mA = (atod - 178) * 3.165 mA/count. AtoD measurements below the min current
 * will be zeroed as this is the offset that is subtracted from the AtoD before scaling. */
#define CONVERT_MA_MIN_CURRENT      178
#define CONVERT_MA_MULT             3165
#define CONVERT_MA_DIV              1000  

#define ATOD_NUM_RAW_RESULTS  7

#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
/* Bits in ADC0... */
#define TS_M_MASK              0x0007000
#define TS_M_SHIFT             12
#define TS_REF_LOW_PWR_MASK    0x0000800
#define TS_REFENB_MASK         0x0000400
#define ADINC1_MASK            0x0010000
#define ADINC2_MASK            0x0020000
#define BATT_I_ADC_MASK        0x0000004

/* Bits in ADC1... */
#define ADEN_MASK              0x0000001
#define RAND_MASK              0x0000002
#define AD_SEL1_MASK           0x0000008
#define AD_SEL1_SHIFT          3
#define ADA1_MASK              0x00000E0
#define ADA1_SHIFT             5
#define ADA2_MASK              0x0000700
#define ADA2_SHIFT             8
#define ATO_MASK               0x007F800
#define ATO_SHIFT              11
#define ATOX_MASK              0x0080000
#define ASC_MASK               0x0100000
#define ASC_SHIFT              20
#define ADTRIG_ONESHOT_MASK    0x0400000

/* Bits in ADC2. */
#define ADD1_MASK              0x0000FFC
#define ADD1_SHIFT             2
#define ADD2_MASK              0x0FFC000
#define ADD2_SHIFT             14

/* The number of channels sampled in a single bank. */
#define SAMPLES_PER_BANK       8

/* Supported touchscreen modes. */
#define TOUCHSCREEN_POSITION   3
#define TOUCHSCREEN_PRESSURE   2
#define TOUCHSCREEN_STANDBY    5

/* The delay programmed for in-burst and out of burst conversions, in cycles of the
 * 16.384kHz ATO clock. For now, these are the same timings as used in the past. */
#define DELAY_IN_BURST      4
#define DELAY_OUT_OF_BURST  0

/* Used for register definitions to cut down on the amount of duplicate code */
#define GET_RESULTS_ADC           POWER_IC_REG_ATLAS_ADC_2
#define DISABLE_CONVERT_REG       POWER_IC_REG_ATLAS_ADC_1
#define ARM_ONE_SHOT_REG          POWER_IC_REG_ATLAS_ADC_1
#define START_CONVERT_REG_ASC     POWER_IC_REG_ATLAS_ADC_1
#define START_CONVERT_REG_ATO     POWER_IC_REG_ATLAS_ADC_1
#define TOUCHSCREEN_STANDBY_REG   POWER_IC_REG_ATLAS_ADC_0
#define TS_LOW_PWR_MOD_REG        POWER_IC_REG_ATLAS_ADC_0

#define EVENT_ADCDONEI            POWER_IC_EVENT_ATLAS_ADCDONEI
#define EVENT_TSI                 POWER_IC_EVENT_ATLAS_TSI     

/* An current AtoD is converted to millamps using these. According to hardware team,
 * current in mA = (atod - 238) * 5.6 mA/count. AtoD measurements below the min current
 * will be zeroed as this is the offset that is subtracted from the AtoD before scaling. */
#define CONVERT_MA_MIN_CURRENT      238
#define CONVERT_MA_MULT             5600
#define CONVERT_MA_DIV              1000

#define ATOD_NUM_RAW_RESULTS  8

#endif /* ATLAS / PCAP2 */

/* This is the offset used in determining whether any samples taken should be filtered.
 * if a sample is more than this many counts below the highest sample in the set, it
 * is dropped from the average. */
#define HACK_SAMPLE_FILTER_OFFSET       20

/* If we drop more than this number of samples, we'll retry the conversion. */
#define HACK_TOO_MANY_SAMPLES_FILTERED  3

#endif /* Doxygen skips over this */

/* These are various constants used for phasing. */
#define PHASING_ZERO_OFFSET     0x00   /*<! Default zero offset correction. */
#define PHASING_ZERO_SLOPE      0x80   /*<! Default zero slope correction. */
#define PHASING_MIN_RESULT      0x0000 /*<! Minimum 10-bit result allowed after phasing. */
#define PHASING_MAX_RESULT      0x03FF /*<! Maximum 10-bit result allowed after phasing. */
#define PHASING_OFFSET_MULTIPLY 4      /*<! 8-bit offset multiplied to be applied to 10-bit AtoD. */
#define PHASING_SLOPE_DIVIDE    7      /*<! Shift for division after slope multipler. */
#define PHASING_BAD_SLOPE_MAX   0xFF
#define PHASING_BAD_SLOPE_MIN   0x00
#define PHASING_NOT_AVAILABLE   -1


/******************************************************************************
* Local type definitions
******************************************************************************/

/*! The channels in bank 1 that are involved in a touchscreen conversion. */
typedef enum
{
    /* Lower channels not used for touchscreen. */
    
    TOUCHSCREEN_CONV_CHANNEL_X = 3, /*!< X position. */
    TOUCHSCREEN_CONV_CHANNEL_PRESSURE = TOUCHSCREEN_CONV_CHANNEL_X, /*!< Only in pressure mode. */
    TOUCHSCREEN_CONV_CHANNEL_Y = 5  /*!< Y position. */
} TOUCHSCREEN_CONV_CHANNEL_T;

/*! Used internally to indicate which type of conversion has been requested. */
typedef enum
{
    POWER_IC_ATOD_TYPE_SINGLE_CHANNEL, /*!< An immediate conversion of a single channel. */
    POWER_IC_ATOD_TYPE_BATT_AND_CURR,  /*!< Battery and current conversion. */
    POWER_IC_ATOD_TYPE_GENERAL,        /*!< Typical set of conversions - temperature, supply, etc. */
    POWER_IC_ATOD_TYPE_TOUCHSCREEN_POSITION,  /*!< Touchscreen conversion. */
    POWER_IC_ATOD_TYPE_TOUCHSCREEN_PRESSURE,  /*!< Touchscreen pressure conversion. */
    POWER_IC_ATOD_TYPE_RAW             /*!< Intended only for testing the hardware. */
} POWER_IC_ATOD_TYPE_T;

/*! The structure used to specify what conversion exclusive_conversion() should set up. */
typedef struct
{
    POWER_IC_ATOD_TYPE_T     type;    /*!< The type of the conversion. */
    POWER_IC_ATOD_CHANNEL_T  channel; /*!< Only needed for the single-channel conversion. */    
    POWER_IC_ATOD_TIMING_T   timing;  /*!< Ignored for single-channel conversion. */
    POWER_IC_ATOD_CURR_POLARITY_T  curr_polarity; /*!< Polarity for batt/current conversion. */
    int                      timeout; /*!< Timeout for postponable conversion, in seconds. */
} POWER_IC_ATOD_REQUEST_T;

/* Pick a bank of channels... */
typedef enum
{
    POWER_IC_ATOD_BANK_0,
    POWER_IC_ATOD_BANK_1
} POWER_IC_ATOD_BANK_T;

/* The phasing data is stored in NVM in the following byte order. It is important that this 
 * order be kept the same for both Linux and legacy products.*/
typedef enum
{
    ATOD_PHASING_BATT_CURR_OFFSET,
    ATOD_PHASING_BATT_CURR_SLOPE,
    ATOD_PHASING_UNUSED1_OFFSET,
    ATOD_PHASING_UNUSED1_SLOPE,
    ATOD_PHASING_BATTPLUS_OFFSET,
    ATOD_PHASING_BATTPLUS_SLOPE,
    ATOD_PHASING_BPLUS_OFFSET,
    ATOD_PHASING_BPLUS_SLOPE,
    ATOD_PHASING_UNUSED2_OFFSET,
    ATOD_PHASING_UNUSED2_SLOPE,
    ATOD_PHASING_MOBPORTB_OFFSET,
    ATOD_PHASING_MOBPORTB_SLOPE,
    
    ATOD_PHASING_NUM_VALUES

} ATOD_PHASING_VALUES_T;

#ifdef CONFIG_MOT_POWER_IC_PCAP2
typedef enum
{
    ATOD_CHANNEL_PCAP_COIN_CELL,
    ATOD_CHANNEL_PCAP_BATT,
    ATOD_CHANNEL_PCAP_BPLUS,
    ATOD_CHANNEL_PCAP_MOBPORTB,
    ATOD_CHANNEL_PCAP_TEMPERATURE,
    ATOD_CHANNEL_PCAP_CHARGER_ID,
    ATOD_CHANNEL_PCAP_AD6,  
    ATOD_CHANNEL_PCAP_AD7,
    ATOD_CHANNEL_PCAP_AD8,
    ATOD_CHANNEL_PCAP_AD9,
    ATOD_CHANNEL_PCAP_TOUCHSCREEN_X1,
    ATOD_CHANNEL_PCAP_TOUCHSCREEN_X2,
    ATOD_CHANNEL_PCAP_TOUCHSCREEN_Y1,
    ATOD_CHANNEL_PCAP_TOUCHSCREEN_Y2, 
    
    ATOD_CHANNEL_NUM_VALUES
} ATOD_CHANNEL_T;
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
typedef enum
{
    ATOD_CHANNEL_ATLAS_BATT,
    ATOD_CHANNEL_ATLAS_BATT_CURR,
    ATOD_CHANNEL_ATLAS_BPLUS,
    ATOD_CHANNEL_ATLAS_MOBPORTB,
    ATOD_CHANNEL_ATLAS_CHRG_CURR,
    ATOD_CHANNEL_ATLAS_TEMPERATURE,
    ATOD_CHANNEL_ATLAS_COIN_CELL,
    ATOD_CHANNEL_ATLAS_CHARGER_ID,
    ATOD_CHANNEL_ATLAS_AD8,  
    ATOD_CHANNEL_ATLAS_AD9,
    ATOD_CHANNEL_ATLAS_AD10,
    ATOD_CHANNEL_ATLAS_AD11,
    ATOD_CHANNEL_ATLAS_TOUCHSCREEN_X1,
    ATOD_CHANNEL_ATLAS_TOUCHSCREEN_X2,
    ATOD_CHANNEL_ATLAS_TOUCHSCREEN_Y1,
    ATOD_CHANNEL_ATLAS_TOUCHSCREEN_Y2, 
    
    ATOD_CHANNEL_NUM_VALUES
} ATOD_CHANNEL_T;
#endif

/******************************************************************************
* Local variables
******************************************************************************/
/*! This mutex must be grabbed before altering any converter hardware setting. */
static DECLARE_MUTEX(hardware_mutex);

/*! This mutex ensures that only one postponable conversion is being handled at any time. */
static DECLARE_MUTEX(postponable_mutex);

#ifndef DOXYGEN_SHOULD_SKIP_THIS /* Doxygen does a really bad job of documenting this stuff... */
/*! Completion for the postponable conversion - uses a queue to allow a timeout. */
DECLARE_WAIT_QUEUE_HEAD(postponable_complete_queue);
#endif

/*! This flag will be set while a conversion that can be postponed is in progress, This flag
 *  must only be modified when the converter lock is held. */
static bool postponable_conversion_started = false;

/*! This variable is used to remember the current polarity originally requested for a
 *  postponable conversion. */
static POWER_IC_ATOD_CURR_POLARITY_T postponable_conversion_polarity = POWER_IC_ATOD_CURR_POLARITY_DISCHARGE;

/*! Remembers the AtoD timing originally requested for a postponable conversion. */
static POWER_IC_ATOD_TIMING_T postponable_conversion_timing = POWER_IC_ATOD_TIMING_IN_BURST;

/*!  This flag will be set when a postponable conversion is complete.*/
static bool postponable_conversion_event_flag = false;

/*! Holds the results from a postponable conversion. The results are fetched when the interrupt
 *  occurs so that they are not potentially overwritten by a following exclusive conversion. */
static int postponable_results[SAMPLES_PER_BANK];

/*! Indicates whether an error occurred when fetching the results of a postponable conversion. */
static int postponable_results_error = 0;
 
/*! Phasing data is stored here. */
static unsigned char phasing_values[ATOD_PHASING_NUM_VALUES] =
{
    /* Discharge current. */
    PHASING_ZERO_OFFSET,
    PHASING_ZERO_SLOPE,
    /* Unused. */
    PHASING_ZERO_OFFSET,
    PHASING_ZERO_SLOPE,
    /* Batt+. */
    PHASING_ZERO_OFFSET,
    PHASING_ZERO_SLOPE,
    /* B+. */
    PHASING_ZERO_OFFSET,
    PHASING_ZERO_SLOPE,
    /* Unused. */
    PHASING_ZERO_OFFSET,
    PHASING_ZERO_SLOPE,
    /* MobportB. */
    PHASING_ZERO_OFFSET,
    PHASING_ZERO_SLOPE,
};

/*! Lookup table pairing AtoD channels with phasing data. Used to indicate whether 
 *  phasing is available for each channel. Channels that do not have phasing available
 *  will be marked with ATOD_PHASING_NUM_VALUES. Those that have phasing available will
 *  will be marked with the index of the phasing offset for the channel. Note that 
 *  battery current will be a special case handled separately for PCAP. */
#ifdef CONFIG_MOT_POWER_IC_PCAP2
static const int phasing_channel_lookup[ATOD_CHANNEL_NUM_VALUES] = 
{
    ATOD_PHASING_NUM_VALUES,      /* Coin cell */
    ATOD_PHASING_BATTPLUS_OFFSET,
    ATOD_PHASING_BPLUS_OFFSET,
    ATOD_PHASING_MOBPORTB_OFFSET,
    ATOD_PHASING_NUM_VALUES,      /* Temperature */
    ATOD_PHASING_NUM_VALUES,      /* Charger ID */
    ATOD_PHASING_NUM_VALUES,
    ATOD_PHASING_NUM_VALUES,
    ATOD_PHASING_NUM_VALUES,
    ATOD_PHASING_NUM_VALUES,      /* Light sensor? */
    ATOD_PHASING_NUM_VALUES,      /* Touchscreen X1 */
    ATOD_PHASING_NUM_VALUES,      /* Touchscreen X2 */
    ATOD_PHASING_NUM_VALUES,      /* Touchscreen Y1 */
    ATOD_PHASING_NUM_VALUES       /* Touchscreen Y2 */
};
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
static const int phasing_channel_lookup[ATOD_CHANNEL_NUM_VALUES] = 
{
    ATOD_PHASING_BATTPLUS_OFFSET,
    ATOD_PHASING_BATT_CURR_OFFSET,
    ATOD_PHASING_NUM_VALUES,      /* Application supply */
    ATOD_PHASING_MOBPORTB_OFFSET,
    ATOD_PHASING_NUM_VALUES,      /* Charger current */
    ATOD_PHASING_NUM_VALUES,      /* Temperature */
    ATOD_PHASING_NUM_VALUES,      /* Coin Cell */
    ATOD_PHASING_NUM_VALUES,      /* Atlas Die Temperature */
    ATOD_PHASING_NUM_VALUES,
    ATOD_PHASING_NUM_VALUES,
    ATOD_PHASING_NUM_VALUES,
    ATOD_PHASING_NUM_VALUES,
    ATOD_PHASING_NUM_VALUES,      /* Touchscreen X1 */
    ATOD_PHASING_NUM_VALUES,      /* Touchscreen X2 */
    ATOD_PHASING_NUM_VALUES,      /* Touchscreen Y1 */
    ATOD_PHASING_NUM_VALUES       /* Touchscreen Y2 */
};
#endif

/*! Lookup table pairing AtoD channels with the actual channel for either PCAP or Atlas. 
 *  The values in this table match the order of the enum in power_ic.h which is NOT
 *  hardware dependent.
 */
#ifdef CONFIG_MOT_POWER_IC_PCAP2
static const int atod_channel_lookup[ATOD_CHANNEL_NUM_VALUES] = 
{
    ATOD_CHANNEL_PCAP_AD6,
    ATOD_CHANNEL_PCAP_BATT,
    ATOD_CHANNEL_NUM_VALUES,
    ATOD_CHANNEL_PCAP_BPLUS,
    ATOD_CHANNEL_PCAP_CHARGER_ID,
    ATOD_CHANNEL_NUM_VALUES,
    ATOD_CHANNEL_PCAP_COIN_CELL,
    ATOD_CHANNEL_PCAP_MOBPORTB,
    ATOD_CHANNEL_PCAP_TEMPERATURE,
    ATOD_CHANNEL_NUM_VALUES,  
    ATOD_CHANNEL_NUM_VALUES,
    ATOD_CHANNEL_NUM_VALUES,
    ATOD_CHANNEL_NUM_VALUES,
    ATOD_CHANNEL_NUM_VALUES,
};
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
static const int atod_channel_lookup[ATOD_CHANNEL_NUM_VALUES] = 
{
    ATOD_CHANNEL_NUM_VALUES,
    ATOD_CHANNEL_ATLAS_BATT,
    ATOD_CHANNEL_ATLAS_BATT_CURR,
    ATOD_CHANNEL_ATLAS_BPLUS,
    ATOD_CHANNEL_ATLAS_CHARGER_ID,
    ATOD_CHANNEL_ATLAS_CHRG_CURR,
    ATOD_CHANNEL_ATLAS_COIN_CELL,
    ATOD_CHANNEL_ATLAS_MOBPORTB,
    ATOD_CHANNEL_ATLAS_TEMPERATURE,
    ATOD_CHANNEL_NUM_VALUES,  
    ATOD_CHANNEL_NUM_VALUES,
    ATOD_CHANNEL_NUM_VALUES,
    ATOD_CHANNEL_NUM_VALUES,
    ATOD_CHANNEL_NUM_VALUES,
    ATOD_CHANNEL_NUM_VALUES,
    ATOD_CHANNEL_NUM_VALUES 
};
#endif

/******************************************************************************
* Local function prototypes
******************************************************************************/
static int phasing_available(POWER_IC_ATOD_CHANNEL_T channel);
static int set_phasing_values(unsigned char * values);
static int phase_atod(int offset, int slope, int atod);
static int phase_atod_10(int offset, int slope, int atod);
static int phase_channel(POWER_IC_ATOD_CHANNEL_T channel, int atod);
static int disable_converter(void);
static int arm_one_shot(void);
static int start_conversion(POWER_IC_ATOD_TIMING_T type);
static int set_single_channel_conversion(POWER_IC_ATOD_CHANNEL_T channel);
static int set_bank_conversion(POWER_IC_ATOD_BANK_T bank);
static int set_batt_current_conversion(POWER_IC_ATOD_CURR_POLARITY_T curr_polarity);
static int set_touchscreen_position_conversion(void);
static int set_touchscreen_pressure_conversion(void);
static int set_up_hardware_exclusive_conversion(POWER_IC_ATOD_REQUEST_T * request);
static int get_results(int * results);
static int convert_to_milliamps(int sample);
static int exclusive_conversion(POWER_IC_ATOD_REQUEST_T * request, int * samples);
static int non_blocking_conversion(POWER_IC_ATOD_REQUEST_T * request);

static int atod_complete_event_handler(POWER_IC_EVENT_T unused);

static int HACK_sample_filter(int * samples, int * average);

static int power_ic_atod_nonblock_begin_conversion(POWER_IC_ATOD_TIMING_T timing,
                                            POWER_IC_ATOD_CURR_POLARITY_T polarity);
static int power_ic_atod_nonblock_cancel_conversion(void);
static int atod_channel_available(POWER_IC_ATOD_CHANNEL_T channel);
/******************************************************************************
* Local functions
******************************************************************************/

/*!
 * @brief Indicates if phasing is available.
 *
 * This function indicates whether phasing is available for a channel and the 
 * location of the phasing parameters in the phasing table.
 *
 * @param        channel   The AtoD channel to check for phasing.
 *
 * @return returns PHASING_NOT_AVAILABLE if no phasing is available, or the 
 * index in the phasing table of the phasing offset if phasing exists.
 */
static int phasing_available(POWER_IC_ATOD_CHANNEL_T channel)
{
    if(phasing_channel_lookup[channel] != ATOD_PHASING_NUM_VALUES)
    {
        return phasing_channel_lookup[channel];
    }
    
    return PHASING_NOT_AVAILABLE;
}

/*!
 * @brief Sets the table of phasing values.
 *
 * This function takes a set of phasing values and, after some basic checking
 * for validity, overwrites the driver's table of values.
 *
 * @param        values    The new phasing values to be set.
 *
 * @return returns 0 if the new values are accepted and copied successfully.
 */
static int set_phasing_values(unsigned char * values)
{
    int i;
    
    tracemsg(_k_d("   Setting new AtoD phasing values."));
    
    for(i = 1; i< ATOD_PHASING_NUM_VALUES; i+=2)
    {
        tracemsg(_k_d("   values[%d..%d]: 0x%X, 0x%X"), i-1, i, values[i-1], values[i]);
        
        /* Realistically, only the slope can be used to indicate a problem in the passed
         * phasing values. Although a slope correction of 0x00 or 0xFF is technically allowed,
         * in reality these shouldn't ever be seen in actual phasing data (e.g. a slope of 
         * zero results in a flat response from the phased AtoD, which is useless). 
         * We therefore start from the first slope in the phasing values (index 1) and 
         * skip over 2 vaules at a time to the next slope.*/
        if((values[i] == PHASING_BAD_SLOPE_MAX) || (values[i] == PHASING_BAD_SLOPE_MIN))
        {
            tracemsg(_k_d("   Error: Bad slope value seen. Phasing not set."));
            return -EINVAL;
        }        
    }

    /* If a signal caused us to continue without getting the mutex, then quit with an error. */
    if(down_interruptible(&hardware_mutex) != 0)
    {
        tracemsg(_k_d("   process received signal while waiting for hardware mutex. Exiting."));
        return -EINTR;
    }
    
    memcpy(phasing_values, values, ATOD_PHASING_NUM_VALUES);
    up(&hardware_mutex);
    
    return 0;
}

/*!
 * @brief Phases a single AtoD measurement.
 *
 * This function uses the supplied phasing parameters to correct slight 
 * imperfections in the response of the AtoD converter.
 *
 * @param        offset   The offset that should be applied from phasing. Note
 *                        that this is a signed value.
 * @param        slope    The multiplier for correcting the slope of the AtoD response.
 * @param        atod     The AtoD measurement to be phased.
 *                        
 * @return returns 0 if successful.
 *
 * @note Where possible, phase_channel() should be used instead of this function. This
 * function presumes that the caller knows how the phasing data is stored and whether
 * phasing is available for the AtoD channel, whereas phase_channel() does all that for
 * the caller based on the passed channel.
 */
static int phase_atod(int offset, int slope, int atod)
{
    int phased_atod = atod;
    
    /* First, correct the slope if the phasing indicates an adjustment is needed. */
    if(slope != PHASING_ZERO_SLOPE)
    {
        phased_atod = phased_atod * slope;
        phased_atod = phased_atod >> PHASING_SLOPE_DIVIDE;
    }
    
    /* Next, correct any fixed offset in the response. The offset is just a signed
     * 8-bit value that should be added to the AtoD measurement. */
    phased_atod += (offset * PHASING_OFFSET_MULTIPLY);
    
    /* Finally, restrict the result to the valid range of AtoD measurements. */
    if(phased_atod > PHASING_MAX_RESULT)
    {
        phased_atod = PHASING_MAX_RESULT;
    }
    else if(phased_atod < PHASING_MIN_RESULT)
    {
        phased_atod = PHASING_MIN_RESULT;
    }
    
    return phased_atod;
}

/*!
 * @brief Phases a single AtoD measurement (for use with unscaled offsets).
 *
 * This function uses the supplied phasing parameters to correct slight 
 * imperfections in the response of the AtoD converter.
 *
 * @param        offset   The offset that should be applied from phasing. Note
 *                        that this is a signed value and is the raw value to be 
 *                        applied to the 10-bit DAC reading.
 * @param        slope    The multiplier for correcting the slope of the AtoD response.
 * @param        atod     The AtoD measurement to be phased.
 *                        
 * @return returns 0 if successful.
 *
 * @note Where possible, phase_channel() should be used instead of this function. This
 * function presumes that the caller knows how the phasing data is stored and whether
 * phasing is available for the AtoD channel, whereas phase_channel() does all that for
 * the caller based on the passed channel.
 */
static int phase_atod_10(int offset, int slope, int atod)
{
    int phased_atod = atod;
    
    /* First, correct the slope if the phasing indicates an adjustment is needed. */
    if(slope != PHASING_ZERO_SLOPE)
    {
        phased_atod = phased_atod * slope;
        phased_atod = phased_atod >> PHASING_SLOPE_DIVIDE;
    }
    
    /* Next, correct any fixed offset in the response. The offset is just a signed
     * 8-bit value that should be added to the AtoD measurement.  This is for the 
     * special cases when the offset is with respect to the raw 10-bit DAC and has 
     * not been scaled for 8-bit. */
    phased_atod += offset;
    
    /* Finally, restrict the result to the valid range of AtoD measurements. */
    if(phased_atod > PHASING_MAX_RESULT)
    {
        phased_atod = PHASING_MAX_RESULT;
    }
    else if(phased_atod < PHASING_MIN_RESULT)
    {
        phased_atod = PHASING_MIN_RESULT;
    }
    
    return phased_atod;
}

/*!
 * @brief Phases an AtoD measurement from a given channel.
 *
 * This function phases a channel's AtoD measurement based on whether phasing is available
 * and handles the stored phasing parameters so that the caller doesn't need to care.
 *
 * @param        channel  The channel from which the AtoD measurement was taken.
 * @param        atod     The AtoD measurement to be phased.
 *                        
 * @return returns 0 if successful.
 *
 * @note Where possible, phase_channel() should be used in preference to phase_atod().
 * This function hides the knowledge of all phasing parameters and operations from the
 * caller and is much nicer to use than phase_atod(). The main exception is current
 * measurements - they are not a discrete channel in themselves, so they don't conform
 * to the way the rest of the phasing is set up.
 */
static int phase_channel(POWER_IC_ATOD_CHANNEL_T channel, int atod)
{
    int phased_atod = atod;
    int phasing_offset;
    signed char signed_offset;
    int atod_channel = atod_channel_available(channel);
    
    if(atod_channel == ATOD_CHANNEL_NUM_VALUES)
    {
        tracemsg("AtoD channel is not a valid channel on this Power IC.");
        return -EINVAL;
    }
    
    phasing_offset = phasing_available(atod_channel);
    
    /* We only need to phase the measurement if there is phasing available for this
     * channel. */
    if(phasing_offset != PHASING_NOT_AVAILABLE)
    {
        /* The offset is actually a signed value. */
        signed_offset = (signed char) phasing_values[phasing_offset];
        
        phased_atod = phase_atod(signed_offset, phasing_values[phasing_offset + 1], atod);
    }
    
    return phased_atod;
}

/*!
 * @brief Disables the converter.
 *
 * @pre The caller of this function must hold the hardware lock prior to 
 * using this function.
 *
 * @return returns 0 if successful.
 */
static int disable_converter(void)
{
    return(power_ic_set_reg_mask(DISABLE_CONVERT_REG, ADEN_MASK, 0));
}

/*!
 * @brief Arms the one-shot trigger for a conversion.
 *
 * This function arms the one-shot hardware trigger for a conversion. The one-shot 
 * trigger ensures that after a conversion completes, no further conversions will be 
 * performed on subsequent transmit bursts, ensuring that the software can retrieve the
 * results of the conversion This is only needed for a hardware-timed conversion.
 *
 * @pre The caller of this function must hold the hardware lock prior to 
 * using this function.
 *
 * @return 0 if successful.
 */
static int arm_one_shot(void)
{
    int error;
    
    /* One-shot is armed by clearing and then setting the ADTRIG_ONESHOT bit. */
    error = power_ic_set_reg_mask(ARM_ONE_SHOT_REG, ADTRIG_ONESHOT_MASK, 0);
    
    if(error != 0)
    {
        return error;
    }

    return(power_ic_set_reg_mask(ARM_ONE_SHOT_REG, ADTRIG_ONESHOT_MASK, ADTRIG_ONESHOT_MASK));
}

/*!
 * @brief Starts an AtoD conversion.
 * 
 * This function triggers an AtoD conversion with the currently programmed settings.
 * The function will either start a conversion immediately or set up a hardware-timed
 * conversion taken either inside or outside the transmit burst depending on the input.
 *
 * @pre The caller of this function must hold the hardware lock prior to 
 * using this function.
 *
 * @param        timing    Timing of conversion (immediate, in/out of burst)
 *
 * @return 0 if successful.
 */
static int start_conversion(POWER_IC_ATOD_TIMING_T timing)
{
    int mask;
    int setup;
    int delay;
    int error;
    
    if(timing == POWER_IC_ATOD_TIMING_IMMEDIATE)
    {
        /* An immediate conversion is triggered by setting the ASC bit. */
        mask = ASC_MASK;
        setup = ASC_MASK;
        
        return(power_ic_set_reg_mask(START_CONVERT_REG_ASC, mask, setup));
    }
    else /* This is a hardware-timed conversion. */
    {
        if(timing == POWER_IC_ATOD_TIMING_IN_BURST)
        {
            delay = DELAY_IN_BURST;
        }
        else
        {
            delay = DELAY_OUT_OF_BURST;
        }
    
        /* The timing for a hardware-controlled conversion is handled by setting ATO
         * and ATOX. In this case, ATOX will be left low so that there is no additional
         * delay between each of the conversions. */
        mask = ATO_MASK | ATOX_MASK;
        setup = (delay << ATO_SHIFT) & ATO_MASK;
        
        tracemsg(_k_d("    hardware-timed conversion - setting delay: mask 0x%X, data 0x%X."), 
                 mask, setup); 

        error = power_ic_set_reg_mask(START_CONVERT_REG_ATO, mask, setup);
        
        if(error != 0)
        {
            return error;
        }
        
        /* Allow the conversion to begin when the hardware is ready. */
        return(arm_one_shot());
    }
}

/*!
 * @brief Sets up conversion of a single AtoD channel.
 *
 * This function programs the hardware to convert a single channel. The converter 
 * will be enabled if this succeeds and will then be ready to perform the conversion.
 *
 * @pre The caller of this function must hold the hardware lock prior to 
 * using this function.
 *
 * @param        channel   The channel to be sampled.
 *
 * @return 0 if successful.
 */
static int set_single_channel_conversion(POWER_IC_ATOD_CHANNEL_T channel)
{
    int setup;
    int atod_channel = atod_channel_available(channel);
    
    if(atod_channel == ATOD_CHANNEL_NUM_VALUES)
    {
        tracemsg("AtoD channel is not a valid channel on this Power IC.");
        return -EINVAL;
    }
    
    /* Pick the bank and channel. */
    if(atod_channel < SAMPLES_PER_BANK)
    {
        setup = ADEN_MASK | RAND_MASK | ((atod_channel << ADA1_SHIFT) & ADA1_MASK);
    }
    else
    {
        setup = ADEN_MASK | RAND_MASK | AD_SEL1_MASK | 
                (((atod_channel - SAMPLES_PER_BANK) << ADA1_SHIFT) & ADA1_MASK);
    }
    
#ifdef CONFIG_MOT_POWER_IC_PCAP2    
    return(power_ic_set_reg_mask(POWER_IC_REG_PCAP_ADC1,(ADEN_MASK | RAND_MASK | AD_SEL1_MASK 
        | ADA1_MASK | BATT_I_ADC_MASK), setup));
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    int error;
    
    error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_ADC_0, BATT_I_ADC_MASK, 0);
    error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_ADC_1,(ADEN_MASK | RAND_MASK | AD_SEL1_MASK 
        | ADA1_MASK), setup);

    return error; 
#endif
}

/*!
 * @brief Sets up a conversion of all the AtoD channels on either bank.
 *
 * This function programs the hardware to convert either bank of 7 channels.  The 
 * converter will be enabled if this succeeds and will then be ready to perform 
 * the conversion.
 *
 * @pre The caller of this function must hold the hardware lock prior to 
 * using this function.
 *
 * @param        bank      The bank of channels to be sampled.
 *
 * @return 0 if successful.
 */
static int set_bank_conversion(POWER_IC_ATOD_BANK_T bank)
{
    int setup= ADEN_MASK | ((bank << AD_SEL1_SHIFT) & AD_SEL1_MASK);
  
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    return(power_ic_set_reg_mask(POWER_IC_REG_PCAP_ADC1,(ADEN_MASK | RAND_MASK | AD_SEL1_MASK 
        | BATT_I_ADC_MASK), setup));
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    int error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_ADC_0, BATT_I_ADC_MASK, 0);
    error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_ADC_1, (ADEN_MASK | RAND_MASK 
        | AD_SEL1_MASK), setup);
           
    return error;
#endif
}

/*!
 * @brief Sets up a conversion of both battery voltage and current.
 * 
 * This function programs the hardware to perform a battery/current conversion. The hardware
 * provides a mode where the current drawn from the battery can be sampled at the same time
 * as an AtoD reading of the battery. The charger software can then use both pieces of 
 * information to better deal with sags in battery voltage.
 *
 * @pre The caller of this function must hold the hardware lock prior to 
 * using this function.
 *
 * @param        curr_polarity  Selects in which direction current will be measured.
 *
 * @note         curr_polarity is ignored for Atlas
 *
 * @return returns 0 if successful.
 */
static int set_batt_current_conversion(POWER_IC_ATOD_CURR_POLARITY_T curr_polarity)
{
    int channel = atod_channel_available(POWER_IC_ATOD_CHANNEL_BATT);
    
    if(channel == ATOD_CHANNEL_NUM_VALUES)
    {
        tracemsg("AtoD channel is not a valid channel on this Power IC.");
        return -EINVAL;
    }
    
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    /* A battery & current conversion is enabled by setting the BATT_I_ADC bit.
     * RAND is set to take 3 pairs of voltage and current. */
    int setup = ADEN_MASK | RAND_MASK | (channel << ADA1_SHIFT) 
          | BATT_I_ADC_MASK;

    if(curr_polarity == POWER_IC_ATOD_CURR_POLARITY_DISCHARGE)
    {
        setup |= BATT_I_POLARITY_MASK;
    }
 
    return(power_ic_set_reg_mask(POWER_IC_REG_PCAP_ADC1,(ADEN_MASK | RAND_MASK | AD_SEL1_MASK 
        | ADA1_MASK | BATT_I_POLARITY_MASK | BATT_I_ADC_MASK), setup));
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    /* A battery & current conversion is enabled by setting the BATT_I_ADC bit.
     * RAND is set to take 4 pairs of voltage and current. */
    int setup = ADEN_MASK | RAND_MASK | (channel << ADA1_SHIFT);
    int error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_ADC_0, BATT_I_ADC_MASK, BATT_I_ADC_MASK);
    error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_ADC_1,(ADEN_MASK | RAND_MASK 
        | AD_SEL1_MASK | ADA1_MASK), setup);
    
    return error;
#endif    
}

/*!
 * @brief Sets up a conversion of the touchscreen position.
 * 
 * This function programs the hardware to perform a touchscreen position conversion. 
 * Both the X and Y positions will be set up to be converted, and the reference source
 * for the touchscreen will be turned up to full power.
 *
 * @pre The caller of this function must hold the hardware lock prior to 
 * using this function.
 *
 * @return 0 if successful.
 */
static int set_touchscreen_position_conversion(void)
{
#ifdef CONFIG_MOT_POWER_IC_PCAP2 
    return(power_ic_set_reg_mask(POWER_IC_REG_PCAP_ADC1,(ADEN_MASK | RAND_MASK | AD_SEL1_MASK 
        | BATT_I_ADC_MASK | TS_M_MASK ),(ADEN_MASK | ((POWER_IC_ATOD_BANK_1 << AD_SEL1_SHIFT) 
        & AD_SEL1_MASK) | (TOUCHSCREEN_POSITION << TS_M_SHIFT))));

#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    int error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_ADC_0,(BATT_I_ADC_MASK | TS_M_MASK), 
        (TOUCHSCREEN_POSITION << TS_M_SHIFT));
    error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_ADC_1,(ADEN_MASK | RAND_MASK 
        | AD_SEL1_MASK),(ADEN_MASK | ((POWER_IC_ATOD_BANK_1 << AD_SEL1_SHIFT) & AD_SEL1_MASK)));
    return error;
#endif
}

/*!
 * @brief Sets up a conversion of the touchscreen pressure.
 * 
 * This function programs the hardware to perform a touchscreen pressure conversion. 
 * The pressure will be set up to be converted, and the reference source
 * for the touchscreen will be turned up to full power.
 *
 * @pre The caller of this function must hold the hardware lock prior to 
 * using this function.
 *
 * @return 0 if successful.
 */
static int set_touchscreen_pressure_conversion(void)
{
#ifdef CONFIG_MOT_POWER_IC_PCAP2 
    return(power_ic_set_reg_mask(POWER_IC_REG_PCAP_ADC1,(ADEN_MASK | RAND_MASK | AD_SEL1_MASK 
        | BATT_I_ADC_MASK | TS_M_MASK),(ADEN_MASK | ((POWER_IC_ATOD_BANK_1 << AD_SEL1_SHIFT) 
        & AD_SEL1_MASK) | (TOUCHSCREEN_PRESSURE << TS_M_SHIFT))));

#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    int error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_ADC_0,(BATT_I_ADC_MASK | TS_M_MASK), 
        (TOUCHSCREEN_PRESSURE << TS_M_SHIFT));
    error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_ADC_1,(ADEN_MASK | RAND_MASK 
        | AD_SEL1_MASK),(ADEN_MASK | ((POWER_IC_ATOD_BANK_1 << AD_SEL1_SHIFT) & AD_SEL1_MASK)));
    return error;
#endif
}

/*!
 * @brief Sets the hardware for a specified type of conversion.
 * 
 * This function prepares the hardware to perform the specified conversion. The conversion
 * will not be triggered, but the hardware will be set up so the specified conversion will
 * be performed once the converter is triggered.
 *
 * @pre The caller of this function must hold the hardware lock prior to using
 * this function. Although this function does not directly alter the hardware's
 * settings, the functions it calls do.
 *
 * @param        request   Pointer to structure that describes the request.
 *
 * @return 0 if successful.
 */
static int set_up_hardware_exclusive_conversion(POWER_IC_ATOD_REQUEST_T * request)
{
    switch(request->type)
    {
        case POWER_IC_ATOD_TYPE_SINGLE_CHANNEL:
        case POWER_IC_ATOD_TYPE_RAW: /* Fallthrough - only results of raw conversion differ. */
            return (set_single_channel_conversion(request->channel));
            
        case POWER_IC_ATOD_TYPE_BATT_AND_CURR:
            return(set_batt_current_conversion(request->curr_polarity));
            
        case POWER_IC_ATOD_TYPE_GENERAL:
            return(set_bank_conversion(POWER_IC_ATOD_BANK_0));
            
        case POWER_IC_ATOD_TYPE_TOUCHSCREEN_POSITION:
            return(set_touchscreen_position_conversion());
            
        case POWER_IC_ATOD_TYPE_TOUCHSCREEN_PRESSURE:
            return(set_touchscreen_pressure_conversion());

        default:
            return -EINVAL;
    }     
}

/*!
 * @brief Fetches the results of a previously completed conversion.
 *
 * This function reads the conversion results from the hardware. The results are not 
 * interpreted in any way - the function just passes back the results with no
 * knowledge of their meaning.
 *
 * @pre The caller of this function must hold the hardware lock prior to 
 * using this function.
 *
 * @note Ideally this function would use PCAP's auto-increment functionality to
 * cycle through the results without having to select a result each time. This
 * appears to be broken, however.
 *
 * @param        results   Pointer to array that will store the results. The array must
 *                         a minimum of PCAP_SAMPLES_PER_BANK elements in size.
 *
 * @return 0 if successful.
 */
static int get_results(int * results)
{
    int setup;
    int mask = ADA1_MASK | ADA2_MASK;
    int error;
    int i, j;
    int result;
    
    /* To save time and SPI traffic, fetch the results two at a time. */
    for (i = 0; i < SAMPLES_PER_BANK; i+=2)
    {
        j = i+1;
        
#ifdef CONFIG_MOT_POWER_IC_PCAP2       
        /* Restrict the second result to one less than the maximum 3-bit result address, 
         * as we're not sure what PCAP will do in that case. The result does not exist, 
         * but it's not well documented in the spec and we don't trust this not to mess 
         * PCAP up more than it already is. */
        setup = (i << ADA1_SHIFT) | ((j % (SAMPLES_PER_BANK - 1)) << ADA2_SHIFT);

        error = power_ic_set_reg_mask(POWER_IC_REG_PCAP_ADC1, mask, setup);
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)    
        setup = (i << ADA1_SHIFT) | (j  << ADA2_SHIFT);
 
        error = power_ic_set_reg_mask(POWER_IC_REG_ATLAS_ADC_1, mask, setup);

#endif

        if(error != 0)
        {
            tracemsg(_k_d("   error %d while selecting result."), error);
            return error;
        }
        
        /* Get the results. This retrieves two channels at once. */
        error = power_ic_read_reg(GET_RESULTS_ADC, &result);
        
        if(error != 0)
        {
            tracemsg(_k_d("   error %d while fetching result %d."), error, i);
            return error;
        }

        results[i] = (result & ADD1_MASK) >> ADD1_SHIFT;

        /* Three is only one result in the last set for PCAP, but for SCM-A11, there are two, 
           and the SAMPLES_PER_BANK is increased by 1, so this still applies. */
        if(j < SAMPLES_PER_BANK)
        {
            results[j] = (result & ADD2_MASK) >> ADD2_SHIFT;
        }
    }
    
    return 0;
}

/*!
 * @brief Converts a current AtoD measurement to milliamps.
 *
 * @param        sample    AtoD sample to be converted.
 *
 * @return       current in milliamps.
 */
static int convert_to_milliamps(int sample)
{
    int curr = sample;
    /* Current AtoD measurement has an offset - if the sample is above the offset,
     * then a non-zero current can be calculated. */
    if (sample > CONVERT_MA_MIN_CURRENT)
    {
        curr -= CONVERT_MA_MIN_CURRENT;
        curr = (curr * CONVERT_MA_MULT);
        curr = (curr / CONVERT_MA_DIV);
    }
    else
    {
        curr = 0;
    }
    
    return curr;
}

#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
extern int spi_int_event_unmask (POWER_IC_EVENT_T event);
extern inline int get_spi_a2d_block(void);
extern inline int get_spi_a2d_non_block(void);
extern inline int get_spi_a2d_non_block_preemptable(void);
extern int power_ic_event_unmask (POWER_IC_EVENT_T event);
extern int spi_int_set_reg_mask(POWER_IC_REGISTER_T reg, int mask, int value);
extern int spi_int_set_reg_bit(POWER_IC_REGISTER_T reg, int index, int value);
extern int spi_int_reg_read (int reg, u32 *value_ptr);

extern int spi_a2d_req( int (*setup_p)(void*), 
                 void * setup_para_p,
                 int (*finish_p)(void *), 
                 void * finish_para_p,
                 int flagBlock
                 );


typedef void (*A2D_TOUCHSCREEN_PRESSURE_CALLBACK)(u32 pressure);
typedef void (*A2D_TOUCHSCREEN_POSITION_CALLBACK)(u32 x, u32 y);
typedef int (*SPI_A2D_FINISH)(void *);


/*!
 * @brief Setup and start an a2d single conversion.
 *
 * @param        parameter_p    AtoD conversion channel.
 *
 * @return       0 upon success<BR>
 *               -EINVAL if a bad arguement is used.<BR>
 */
int spi_a2d_single_conversion_setup(void * parameter_p)
{
        POWER_IC_ATOD_CHANNEL_T channel=*(POWER_IC_ATOD_CHANNEL_T *)parameter_p;

        int setup;
        int atod_channel = atod_channel_available(channel);
    
        if(atod_channel == ATOD_CHANNEL_NUM_VALUES)
        {
                tracemsg("AtoD channel is not a valid channel on this Power IC.");
                return -EINVAL;
        }
    
        /* Pick the bank and channel. */
        if(atod_channel < SAMPLES_PER_BANK)
        {
                setup= ADEN_MASK | RAND_MASK | ((atod_channel << ADA1_SHIFT) & ADA1_MASK);
        }
        else
        {
                setup= ADEN_MASK | RAND_MASK | AD_SEL1_MASK | 
                (((atod_channel - SAMPLES_PER_BANK) << ADA1_SHIFT) & ADA1_MASK);
        }
    
        int error;
        error = spi_int_event_unmask(EVENT_ADCDONEI);
        error |= spi_int_set_reg_mask(
                           POWER_IC_REG_PCAP_ADC1,
                         ( ADEN_MASK | RAND_MASK | AD_SEL1_MASK | 
                           ADA1_MASK | BATT_I_ADC_MASK ),
                           setup);
        /* start convertion! */
        error |= spi_int_set_reg_mask(START_CONVERT_REG_ASC, ASC_MASK,ASC_MASK);
        return error;
}

/*!
 * @brief Setup and start an a2d touch screen position conversion.
 *
 * @param        unused    unused parameter
 *
 * @return       0 upon success<BR>
 *               -EINVAL if a bad arguement is used.<BR>
 */
int spi_a2d_touchscreen_position_conversion_setup(void * unused)
{
         int error;
         error = spi_int_event_unmask(EVENT_ADCDONEI);

         error |= spi_int_set_reg_mask(
                         POWER_IC_REG_PCAP_ADC1,
                         (ADEN_MASK | RAND_MASK | AD_SEL1_MASK |
                         BATT_I_ADC_MASK | TS_M_MASK ),
                         (ADEN_MASK | ((POWER_IC_ATOD_BANK_1 << AD_SEL1_SHIFT) &
                         AD_SEL1_MASK) | (TOUCHSCREEN_POSITION << TS_M_SHIFT)));

        /* start convertion! */
        error |= spi_int_set_reg_mask(START_CONVERT_REG_ASC, ASC_MASK,ASC_MASK);
        return error;
}

/*!
 * @brief Setup and start an a2d touch screen ppressure conversion.
 *
 * @param        unused    unused parameter
 *
 * @return       0 upon success<BR>
 *               -EINVAL if a bad arguement is used.<BR>
 */
int spi_a2d_touchscreen_pressure_conversion_setup(void * unused)
{
         int error;
         error = spi_int_event_unmask(EVENT_ADCDONEI);

         error |= spi_int_set_reg_mask(
                         POWER_IC_REG_PCAP_ADC1,
                         (ADEN_MASK | RAND_MASK | AD_SEL1_MASK |
                         BATT_I_ADC_MASK | TS_M_MASK),
                         (ADEN_MASK | ((POWER_IC_ATOD_BANK_1 << AD_SEL1_SHIFT) &
                         AD_SEL1_MASK) | (TOUCHSCREEN_PRESSURE << TS_M_SHIFT)));

        /* start convertion! */
        error |= spi_int_set_reg_mask(START_CONVERT_REG_ASC, ASC_MASK,ASC_MASK);
        return error;
}

/*!
 * @brief Setup and start an a2d general conversion.
 *
 * @param        unused    unused parameter
 *
 * @return       0 upon success<BR>
 *               -EINVAL if a bad arguement is used.<BR>
 */
int spi_a2d_general_conversion_setup(void * unused)
{
         int setup;
         setup = ADEN_MASK | ((POWER_IC_ATOD_BANK_0 << AD_SEL1_SHIFT) & AD_SEL1_MASK);
         int error;
         error = spi_int_event_unmask(EVENT_ADCDONEI);
 
         error |= spi_int_set_reg_mask(
                         POWER_IC_REG_PCAP_ADC1,
                         (ADEN_MASK | RAND_MASK | AD_SEL1_MASK | BATT_I_ADC_MASK),
                         setup);

        /* start convertion! */
        error |= spi_int_set_reg_mask(START_CONVERT_REG_ASC, ASC_MASK,ASC_MASK);
        return error;
}

int timing_start_conversion(POWER_IC_ATOD_TIMING_T timing);
/*!
 * @brief Setup and start an a2d current and battery conversion.
 *
 * @param        timing_p    Conversion timing. NULL, POWER_IC_ATOD_TIMING_IN_BURST, POWER_IC_ATOD_TIMING_OUT_OF_BURST
 *
 * @return       0 upon success<BR>
 *               -EINVAL if a bad arguement is used.<BR>
 */
int spi_a2d_current_and_batt_conversion_setup(void * timing_p)
{
        int channel = atod_channel_available(POWER_IC_ATOD_CHANNEL_BATT);
        if(channel == ATOD_CHANNEL_NUM_VALUES)
        {
                tracemsg("AtoD channel is not a valid channel on this Power IC.");
                return -EINVAL;
        }

        /* A battery & current conversion is enabled by setting the BATT_I_ADC bit.
         * RAND is set to take 3 pairs of voltage and current. */
        int setup = ADEN_MASK | RAND_MASK | (channel << ADA1_SHIFT) 
                    | BATT_I_ADC_MASK;

        /*
        if(curr_polarity == POWER_IC_ATOD_CURR_POLARITY_DISCHARGE)
        {
            setup |= BATT_I_POLARITY_MASK;
        }
        */

        int error;
        error = spi_int_event_unmask(EVENT_ADCDONEI);
 
        error |= spi_int_set_reg_mask(
                        POWER_IC_REG_PCAP_ADC1,
                        ( ADEN_MASK | RAND_MASK | AD_SEL1_MASK 
                        | ADA1_MASK | BATT_I_POLARITY_MASK | BATT_I_ADC_MASK), 
                        setup);

        if(timing_p!=NULL)
        {
                return error| timing_start_conversion(
                                  *(POWER_IC_ATOD_TIMING_T*)timing_p);
        }else{
                return error| spi_int_set_reg_mask(
                                  START_CONVERT_REG_ASC, ASC_MASK, ASC_MASK);
        }
}

/*!
 * @brief Setup and start an a2d current and battery discharge conversion.
 *
 * @param        timing_p    Conversion timing. NULL, POWER_IC_ATOD_TIMING_IN_BURST, POWER_IC_ATOD_TIMING_OUT_OF_BURST
 *
 * @return       0 upon success<BR>
 *               -EINVAL if a bad arguement is used.<BR>
 */
int spi_a2d_current_and_batt_conversion_setup_discharge(void * timing_p)
{
        int channel = atod_channel_available(POWER_IC_ATOD_CHANNEL_BATT);
        if(channel == ATOD_CHANNEL_NUM_VALUES)
        {
                tracemsg("AtoD channel is not a valid channel on this Power IC.");
                return -EINVAL;
        }
    
        /* A battery & current conversion is enabled by setting the BATT_I_ADC bit.
         * RAND is set to take 3 pairs of voltage and current. */
        int setup = ADEN_MASK | RAND_MASK | (channel << ADA1_SHIFT) 
                        | BATT_I_ADC_MASK;

        /*
        if(curr_polarity == POWER_IC_ATOD_CURR_POLARITY_DISCHARGE)
        {
        */
                setup |= BATT_I_POLARITY_MASK;
        /*
        }
        */

        int error;
        error = spi_int_event_unmask(EVENT_ADCDONEI);
 
        error |= spi_int_set_reg_mask(
                        POWER_IC_REG_PCAP_ADC1,
                        ( ADEN_MASK | RAND_MASK | AD_SEL1_MASK 
                        | ADA1_MASK | BATT_I_POLARITY_MASK | BATT_I_ADC_MASK), 
                        setup);

        if(timing_p!=NULL)
        {
                return error| timing_start_conversion(
                                  *(POWER_IC_ATOD_TIMING_T*)timing_p);
        }else{
                return error| spi_int_set_reg_mask(
                                  START_CONVERT_REG_ASC, ASC_MASK, ASC_MASK);
        }
}

/*!
 * @brief Stop the AtoD converter.
 *
 * This function stop the AtoD converter. 
 * Disable the converter and clear the ASC bit.
 *
 * @pre The caller of this function must get SPI bus permission first.
 *
 */
void spi_int_stop_atod_converter(void)
{
        /* stop the a2d converter */
        spi_int_set_reg_mask(POWER_IC_REG_PCAP_ADC1, ADEN_MASK, 0);
        /*
         * In order to avoid AtoD Converter restart new conversions
         * we need to clear ASC bit before read results.
         */
        spi_int_set_reg_mask(START_CONVERT_REG_ASC, ASC_MASK, 0);
}

/*!
 * @brief Fetches the results of a previously completed conversion, even in interrupt context.
 *
 * This function reads the conversion results from the hardware. The results are not 
 * interpreted in any way - the function just passes back the results with no
 * knowledge of their meaning.
 *
 * @pre The caller of this function must hold the hardware lock prior to 
 * using this function.
 *
 * @note Ideally this function would use PCAP's auto-increment functionality to
 * cycle through the results without having to select a result each time. This
 * appears to be broken, however.
 *
 * @param        results   Pointer to array that will store the results. The array must
 *                         a minimum of PCAP_SAMPLES_PER_BANK elements in size.
 *
 * @return 0 if successful.
 */

int spi_int_get_results(int * results)
{
        int setup;
        int mask = ADA1_MASK | ADA2_MASK | ADEN_MASK ;
        int error;
        int i, j;
        int result;

        spi_int_stop_atod_converter();

        /* To save time and SPI traffic, fetch the results two at a time. */
        for (i = 0; i < SAMPLES_PER_BANK; i+=2)
        {
                j = i+1;
                /* 
                 * Restrict the second result to one less than the maximum 3-bit
                 * result address, as we're not sure what PCAP will do in that 
                 * case. The result does not exist, but it's not well documented
                 * in the spec and we don't trust this not to mess PCAP up more 
                 * than it already is. 
                 */
 
                setup = (i << ADA1_SHIFT) 
                        | ((j % (SAMPLES_PER_BANK - 1)) << ADA2_SHIFT)
                        | ADEN_MASK;

                error = spi_int_set_reg_mask(POWER_IC_REG_PCAP_ADC1, mask, setup);

                /* Get the results. This retrieves two channels at once. */
                error = spi_int_reg_read(GET_RESULTS_ADC, &result);
        
                if(error != 0)
                {
                        tracemsg(_k_d("   error %d while fetching result %d."), error, i);
                        return error;
                }

                results[i] = (result & ADD1_MASK) >> ADD1_SHIFT;

                /* 
                 * Three is only one result in the last set for PCAP, but for 
                 * SCM-A11, there are two, and the SAMPLES_PER_BANK is increased
                 * by 1, so this still applies. 
                 */
                if(j < SAMPLES_PER_BANK)
                {
                        results[j] = (result & ADD2_MASK) >> ADD2_SHIFT;
                }
        }
    
        return 0;
}

/*!
 * @brief Performs a conversion immediately that cannot be interrupted by other conversions.
 *
 * This function performs a conversion that cannot be interrupted by any other
 * conversion. The conversion is performed immediately. If the converter is waiting
 * for a hardware-timed conversion to occur, then this will steal the hardware and 
 * perform this conversion before returning control to the other process to retry the
 * hardware-timed conversion.
 *
 * @pre This function will cause the calling process to sleep if another conversion
 * is in progress, and as such cannot be used in an interrupt context.
 *
 * @param        request   Pointer to structure that describes the request.
 * @param        samples   Pointer to an array where the raw samples should be stored.
 *                         The array must a minimum of PCAP_SAMPLES_PER_BANK elements in size.
 *
 * @return 0 if successful.
 */
static int exclusive_conversion(POWER_IC_ATOD_REQUEST_T * request, int* samples)
{
        switch(request->type)
        {
                case POWER_IC_ATOD_TYPE_SINGLE_CHANNEL:
                case POWER_IC_ATOD_TYPE_RAW: 
                /* Fallthrough - only results of raw conversion differ. */
                {
                        spi_a2d_req(spi_a2d_single_conversion_setup,
                                    &(request->channel),
                                    (SPI_A2D_FINISH)spi_int_get_results,
                                    (void*)samples, get_spi_a2d_block());
                        return 0;
                }
            
                case POWER_IC_ATOD_TYPE_BATT_AND_CURR:
                {
                        if(request->curr_polarity == POWER_IC_ATOD_CURR_POLARITY_DISCHARGE)
                        {
                                spi_a2d_req(spi_a2d_current_and_batt_conversion_setup_discharge,
                                        NULL,
                                        (SPI_A2D_FINISH)spi_int_get_results,
                                        (void*)samples, get_spi_a2d_block());
                        }
                        else
                        {
                                spi_a2d_req(spi_a2d_current_and_batt_conversion_setup,
                                        NULL,
                                        (SPI_A2D_FINISH)spi_int_get_results,
                                        (void*)samples, get_spi_a2d_block());
                        }
 
                        return 0;
                }
            
                case POWER_IC_ATOD_TYPE_GENERAL:
                {
                        spi_a2d_req(spi_a2d_general_conversion_setup,
                                NULL,
                                (SPI_A2D_FINISH)spi_int_get_results,
                                (void*) samples, get_spi_a2d_block() );
                        return 0;
                }
                case POWER_IC_ATOD_TYPE_TOUCHSCREEN_POSITION:
                {
                        spi_a2d_req(spi_a2d_touchscreen_position_conversion_setup,
                                NULL,
                                (SPI_A2D_FINISH)spi_int_get_results,
                                (void*)samples, get_spi_a2d_block());
                        return 0;
                }
            
                case POWER_IC_ATOD_TYPE_TOUCHSCREEN_PRESSURE:
                {
                        spi_a2d_req(spi_a2d_touchscreen_pressure_conversion_setup,
                                NULL,
                                (SPI_A2D_FINISH)spi_int_get_results,
                                (void*)samples, get_spi_a2d_block());
 
                        return 0;
                }

                default:
                        return -EINVAL;
        }     
}

/*!
 * @brief Callback function for touch screen pressure conversion finished.
 *
 * @param        callback_p     application call back function.
 *
 * @return       0 upon success<BR>
 */
int spi_a2d_touchscreen_pressure_conversion_finish(void * callback_p)
{
        u32 pressure=0;
        int samples[SAMPLES_PER_BANK];

        int error = spi_int_get_results(samples);
        if(error==0)
        {
                if(callback_p != NULL)
                {
                        pressure = samples[TOUCHSCREEN_CONV_CHANNEL_PRESSURE];
                        (*((A2D_TOUCHSCREEN_PRESSURE_CALLBACK)callback_p))(pressure);
                }
                else
                {
                        printk("spi_a2d_touchscreen_pressure_conversion_finish, but call back function missing!\n");
                }
        }
        else
        {
                printk("spi_a2d_touchscreen_pressure_conversion_finish error!\n");
        }
        return error;
}

/*!
 * @brief  Touch screen pressure conversion request, even in interrupt context.
 *
 * @param        finish_callback     application call back function.
 *
 * @return       0 upon success<BR>
 */
void spi_a2d_touchscreen_pressure_conversion(void * finish_callback)
{
        spi_a2d_req(spi_a2d_touchscreen_pressure_conversion_setup,
                NULL,
                spi_a2d_touchscreen_pressure_conversion_finish,
                finish_callback, get_spi_a2d_non_block());
}

/*!
 * @brief Callback function for touch screen position conversion finished.
 *
 * @param        callback_p     application call back function.
 *
 * @return       0 upon success<BR>
 */
int spi_a2d_touchscreen_position_conversion_finish(void * callback_p)
{
        u32 x=0;
        u32 y=0;
        int samples[SAMPLES_PER_BANK];

        int error = spi_int_get_results(samples);
        if(error==0)
        {
                /* According to ezx-ts.c it seems the x and y changed!!!! */
                x = samples[TOUCHSCREEN_CONV_CHANNEL_X];
                y = samples[TOUCHSCREEN_CONV_CHANNEL_Y];
                
                if(callback_p != NULL)
                {
                        (*((A2D_TOUCHSCREEN_POSITION_CALLBACK)callback_p))(x, y);
                }
                else
                {
                        printk("spi_a2d_touchscreen_position_conversion_finish, but call back function missing!\n");
                }
        }
        else
        {
                printk("spi_a2d_touchscreen_position_conversion_finish error!\n");
        }

        /* 
         * Touch screen need to enter standby mode. And wait for next conversion
         * request. Because we will perform postion conversion after 
         * position conversion.
         */
        error |= spi_int_set_reg_mask(
                        POWER_IC_REG_PCAP_ADC1,
                        TS_M_MASK,
                        (TOUCHSCREEN_STANDBY << TS_M_SHIFT) );

        return error;
}

/*!
 * @brief  Touch screen position conversion request, even in interrupt context.
 *
 * @param        finish_callback     application call back function.
 *
 * @return       0 upon success<BR>
 */
void spi_a2d_touchscreen_position_conversion(void * finish_callback)
{
  spi_a2d_req(spi_a2d_touchscreen_position_conversion_setup,
              NULL,
              spi_a2d_touchscreen_position_conversion_finish,
              finish_callback, get_spi_a2d_non_block());
}

/*!
 * @brief Sets the touchscreen to be disabled, in low power mode, or full power mode, even in interrupt context.
 *
 * This function programs the hardware to put the touchscreen source back in low-power 
 * mode if in standy, in full-power mode if in position or pressure mode, and completely
 * disabled. 
 *
 * @param  mode   Mode in which Touchscreen should be set.
 *
 * @return 0 if successful.
 */
int spi_a2d_set_touchscreen_mode(POWER_IC_TS_MODES_T mode)
{
    int error;
    
    switch(mode)
    {
        case POWER_IC_TS_MODE_DISABLED:
            /* Disable Hardware -- 1 to disable*/
            error = spi_int_set_reg_bit (TS_LOW_PWR_MOD_REG, TS_REFENB_MASK, 1);
            break;
        
        case POWER_IC_TS_MODE_LOW_PWR_MODE:
            error = spi_int_set_reg_mask(TOUCHSCREEN_STANDBY_REG,(TS_M_MASK | TS_REFENB_MASK
                | TS_REF_LOW_PWR_MASK), ((TOUCHSCREEN_STANDBY << TS_M_SHIFT) | TS_REF_LOW_PWR_MASK));
            break;
            
        case POWER_IC_TS_MODE_FULL_PWR_MODE:
            error = spi_int_set_reg_mask(TS_LOW_PWR_MOD_REG, (TS_REF_LOW_PWR_MASK 
                | TS_REFENB_MASK), 0);
            break;
            
        default:
            return -EINVAL;
            break;
        
    }
    
    return (error);
}

#else


/*!
 * @brief Performs a conversion immediately that cannot be interrupted by other conversions.
 *
 * This function performs a conversion that cannot be interrupted by any other
 * conversion. The conversion is performed immediately. If the converter is waiting
 * for a hardware-timed conversion to occur, then this will steal the hardware and 
 * perform this conversion before returning control to the other process to retry the
 * hardware-timed conversion.
 *
 * @pre This function tries to claim the hardware lock, The caller cannot already
 * hold this lock or deadlock will result.
 *
 * @pre This function will cause the calling process to sleep if another conversion
 * is in progress, and as such cannot be used in an interrupt context.
 *
 * @param        request   Pointer to structure that describes the request.
 * @param        samples   Pointer to an array where the raw samples should be stored.
 *                         The array must a minimum of PCAP_SAMPLES_PER_BANK elements in size.
 *
 * @return 0 if successful.
 */
static int exclusive_conversion(POWER_IC_ATOD_REQUEST_T * request, int * samples)
{
    int error;
    int conversion_in_progress = true;

    /* If a signal caused us to continue without getting the mutex, then quit with an error. */
    if(down_interruptible(&hardware_mutex) != 0)
    {
        tracemsg(_k_d("   process received signal while waiting for hardware mutex. Exiting."));
        return -EINTR;
    }
    
    /* Check to see if we have stolen the converter while it was waiting for a hardware-timed
     * conversion to take place. If so, then the interrupt must be masked for now. Once this
     * conversion is complete, the other conversion will be restarted. */
    if(postponable_conversion_started)
    {
        tracemsg(_k_d("AtoD - stole converter from postponable conversion."));
        error = power_ic_event_mask(EVENT_ADCDONEI);
    
        if(error != 0)
        {
            tracemsg(_k_d("AtoD - error %d masking AtoD interrupt."), error);
            goto quit_due_to_error;
        }
    }
    
    /* Set up the hardware then start the conversion */
    error = set_up_hardware_exclusive_conversion(request);
    
    if(error != 0)
    {
        tracemsg(_k_d("AtoD - error %d setting up exclusive conversion."), error);
        goto quit_due_to_error;
    }
    
    /* All exclusive conversions are performed immediately. */
    error = start_conversion(POWER_IC_ATOD_TIMING_IMMEDIATE);
    
    if(error != 0)
    {
        tracemsg(_k_d("AtoD - error %d starting exclusive conversion."), error);
        goto quit_due_to_error;
    }

    /* Wait for the conversion to complete. */
    while(conversion_in_progress)
    {
        /* Figure out whether the conversion has completed. ASC clears once the conversion 
         * is complete. This read has been seen to take about 50us, so a further delay probably
         * isn't necessary. */
        power_ic_get_reg_value(START_CONVERT_REG_ASC, ASC_SHIFT, &conversion_in_progress, 1);
    }

    /* The conversion is complete - get the results. */
    error = get_results(samples);
    
    if(error != 0)
    {
        tracemsg(_k_d("AtoD - error %d getting results of exclusive conversion."), error);
        goto quit_due_to_error;
    }

    /* Disable the converter. If it's needed to restart the postponable conversion, it'll
     * be turned back on at that point. */
    error = disable_converter();
            
    if(error != 0)
    {
        tracemsg(_k_d("AtoD - error %d turning off converter."), error);
        goto quit_due_to_error;
    }
    
    /* Finally, if we stole the converter from a postponable conversion that was in progress,
     * restart that conversion. */
    if((postponable_conversion_started) && (!postponable_conversion_event_flag))
    {
        /* Set the converter up to the the postponable conversion's batt/current conversion. */
        error = set_batt_current_conversion(postponable_conversion_polarity);
        
        if(error != 0)
        {
            tracemsg(_k_d("AtoD - error %d setting batt/current conversion."), error);
            goto quit_due_to_error;
        }
        
        /* Unmask the conversion complete interrupt so that it's ready for the hardware-
         * timed conversion*/
        error = power_ic_event_clear(EVENT_ADCDONEI);
        error |= power_ic_event_unmask(EVENT_ADCDONEI);
        
        if(error != 0)
        {
            tracemsg(_k_d("AtoD - error %d unmasking AtoD interrupt."), error);
            goto quit_due_to_error;            
        }
        
        /* Restart the postponable conversion. */
        error = start_conversion(postponable_conversion_timing);
    }
    
quit_due_to_error:        
    /* Done - release the converter. */
    up(&hardware_mutex);

    return error;
}

#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */

#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
/*!
 * @brief Start a hardware time a2d conversion.
 *
 * @param        timing     hardware timing
 *
 * @return       0 upon success<BR>
 */
int timing_start_conversion(POWER_IC_ATOD_TIMING_T timing)
{
        u32 delay = 0x0;
        u32 error = 0x0;
        if(timing == POWER_IC_ATOD_TIMING_IN_BURST)
        {
            delay = DELAY_IN_BURST;
        }
        else
        {
            delay = DELAY_OUT_OF_BURST;
        }

        /* 
         * The timing for a hardware-controlled conversion is handled by 
         * setting ATO and ATOX. In this case, ATOX will be left low so that 
         * there is no additional delay between each of the conversions.
         */
        error |= spi_int_set_reg_mask(
                      START_CONVERT_REG_ATO, 
                      ATO_MASK | ATOX_MASK ,  
                      (delay << ATO_SHIFT) & ATO_MASK );

        /*
         * One-shot is armed by clearing and then setting 
         * the ADTRIG_ONESHOT bit. 
         */
        error |= spi_int_set_reg_mask(
                      ARM_ONE_SHOT_REG, 
                      ADTRIG_ONESHOT_MASK | ASC_MASK, 0);
    
        return error|(spi_int_set_reg_mask(
                      ARM_ONE_SHOT_REG, 
                      ADTRIG_ONESHOT_MASK, ADTRIG_ONESHOT_MASK));

}

void spi_int_non_blocking_conversion(void);

/* record the last postponable conversion results are acceptable or not */
static int flag_last_postponable = true;

/*
 * If PCAP ok, the min battery votage should more than 2.7V.
 */
#define MIN_ACCEPTABLE_BATT         0x158

/*
 * Generally the distance between two sample values should be less than 0.6V.
 */
#define MAX_ACCEPTABLE_DISTANCE      0x90

/*!
 * @brief Check the non blocking conversion results.
 *  
 * In this function we need to check the results before we wake up the 
 * atod read queue. If the results are not acceptable, we need to resetup
 * this non blocking conversino again!
 *
 * @return       true the results acceptable <BR>
 *               false the results are not acceptable <BR>
 */
static int check_postponable_results(void)
{
/*
 * the normal case sample values:
 * **************** atod_read Batt+=0x92;current=452mAx;
 * **************** 0=0x24C;2=0x24B;4=0x24b;6=0x24C;
 * **************** 1=0x13D;3=0x141;5=0x146;
 * the bad sample values:
 * **************** atod_read Batt+=0x2B;current=0mAx;
 * **************** 0=0x20D;2=0x0;4=0x0;6=0x272;
 * **************** 1=0x0;3=0x0;5=0x0;
 *
 * We only need to handle the 4 batt sample datas (0, 2, 4, 6), 
 * and the 3 current sample datas are not important.
 */

/*
 * 1. The bad sample values will not come continuous. (experience data)
 *    If current sample is invalid, the next values are acceptable.
 * 2. If the PCAP work ok, the battery votage >= 2.7V.
 *    The equation below is calculated to fit the desired input range to 0.4V to 2.3V.
 *    VBATSENSE=((V_BATTERY) - 1.2632) * 0.5429               +/-5%
 *
 *    ( 2.7  - 1.2632 ) * 0.5429 = 0.78003872   
 *    ( 0.78003872 / 2.3 ) * 255 = 86.482553739130434782608695652174 = 0x56
 *    0x56 * 4 = 0x158
 *
 * 3. Generally the distance between two sample values should be less than 0.6V.
 *    And calculated it to fit the desired input range to 0.4V to 2.3V is 0x90.
 *    ( (0.6 * 0.5429 ) / 2.3 ) * 255 = 0x24
 *    0x24 * 4  = 0x90
 *    
 */

        /* get the max value of the four data */
        int max_batt = 0x0;
        int average_batt = 0x0;
        int i;
        for(i=0;i<SAMPLES_PER_BANK;i=i+2)
        {
                if(postponable_results[i]>max_batt)
                {
                        max_batt = postponable_results[i];
                }
        }    
        /* if the distance more than 0x100 drop */
        for(i=0;i<SAMPLES_PER_BANK;i=i+2)
        {
                if((max_batt - postponable_results[i])>MAX_ACCEPTABLE_DISTANCE)
                {
                        flag_last_postponable = false;
                        return false;
                }
        }       
        
        for(i=0;i<SAMPLES_PER_BANK;i=i+2)
        {
                average_batt += postponable_results[i];
        }       
        average_batt = average_batt / ((SAMPLES_PER_BANK+1)/2);
        if(average_batt < MIN_ACCEPTABLE_BATT)
        {
                if(flag_last_postponable == false)
                {
                        flag_last_postponable = true;
                        return true;
                }
                else
                {
                        flag_last_postponable = false;
                        return false;
                }
        }
        flag_last_postponable = true;
        return true;
}

/*!
 * @brief Callback function for non blocking conversion finished.
 *  
 * In this function we need to check the results before we wake up the 
 * atod read queue. If the results are not acceptable, we need to resetup
 * this non blocking conversino again!
 *
 * @param        callback_p     useless
 *
 * @return       0 upon success<BR>
 */
int non_blocking_conversion_finish(void * callback_p)
{
        /* 
         * Get the results of the conversion before they are possibly 
         * overwritten by an exclusive conversion. 
         */
        spi_int_get_results(postponable_results);

        if(check_postponable_results()==false)
        {
                spi_int_non_blocking_conversion();
                return 0;
        }

        /* 
         * results are acceptable , this will be checked in atod_read function
         */
        postponable_results_error = 0x0;

        /* 
         * The conversion complete interrupt is only used for non-immediate
         * conversions. Since this interrupt has occurred, we want to wake the 
         * conversion that's waiting on completion.
         */
        postponable_conversion_event_flag = true;
        wake_up(&postponable_complete_queue);

        return postponable_results_error;
}

/*!
 * @brief Starts a conversion that may be interrupted by any exclusive conversion.
 * 
 * This function starts a hardware-timed conversion that may be interrupted by any 
 * exclusive conversion while waiting for the hardware to trigger the conversion. This
 * is done for hardware-timed conversions as there is the potential for a long delay 
 * while the hardware waits for the next transmit burst.
 *
 * This function does not block the caller until the conversion completes. This 
 * is intended for use by the task that contains the battery charger so it does 
 * not block the entire task's operation.
 *
 * This function implement with the SPI BUS interruptable design.
 *
 * @return       non-zero if an error occurs..
 */
void spi_int_non_blocking_conversion(void)
{
        if(postponable_conversion_polarity == POWER_IC_ATOD_CURR_POLARITY_DISCHARGE)
        {
                spi_a2d_req(spi_a2d_current_and_batt_conversion_setup_discharge,
                                &postponable_conversion_timing,
                                (SPI_A2D_FINISH)non_blocking_conversion_finish,
                                NULL, get_spi_a2d_non_block_preemptable());
        }
        else
        {
                spi_a2d_req(spi_a2d_current_and_batt_conversion_setup,
                                &postponable_conversion_timing,
                                (SPI_A2D_FINISH)non_blocking_conversion_finish,
                                NULL, get_spi_a2d_non_block_preemptable());
        }
}

/*
 * Error code: there is a non blocking conversion processing!
 */
#define NON_BLOCKING_CONVERSION_REENTER_ERROR        0x99

/*!
 * @brief Starts a conversion that may be interrupted by any exclusive conversion.
 * 
 * This function starts a hardware-timed conversion that may be interrupted by any 
 * exclusive conversion while waiting for the hardware to trigger the conversion. This
 * is done for hardware-timed conversions as there is the potential for a long delay 
 * while the hardware waits for the next transmit burst.
 *
 * This function does not block the caller until the conversion completes. This 
 * is intended for use by the task that contains the battery charger so it does 
 * not block the entire task's operation.
 *
 * @param        request   Pointer to structure that describes the request.
 *
 * @return       non-zero if an error occurs..
 */

int non_blocking_conversion(POWER_IC_ATOD_REQUEST_T * request)
{
    /* If a signal caused us to continue without getting the mutex, then quit with an error. */
    if(down_interruptible(&postponable_mutex) != 0)
    {
        tracemsg(_k_d("   process received signal while waiting for postponable mutex. Exiting."));
        return -EINTR;
    }
    
    /* If there is already a non blocking conversion setup, just return */    
    if (postponable_conversion_started == true)
    {
        up(&postponable_mutex);
        return NON_BLOCKING_CONVERSION_REENTER_ERROR;
    }
    
    /* Remember that we started a new conversion. */
    postponable_conversion_started = true;
    postponable_conversion_event_flag = false;

    postponable_conversion_timing = request->timing;
    /* add a new variable to record the current polarity */
    postponable_conversion_polarity = request->curr_polarity;

    spi_int_non_blocking_conversion();

    up(&postponable_mutex);

    return 0;
}
 
extern void spi_a2d_interrupt_handler(void);

/*
 * atod conversion complete handler is spi_a2d_interrupt_handler
 * in file power_ic/core/spi_main.c
 */
int atod_complete_event_handler(POWER_IC_EVENT_T unused)
{
  //tracemsg(_k_d("AtoD conversion complete event."));
  spi_a2d_interrupt_handler();
  return 1;
}
#else


/*!
 * @brief Starts a conversion that may be interrupted by any exclusive conversion.
 * 
 * This function starts a hardware-timed conversion that may be interrupted by any 
 * exclusive conversion while waiting for the hardware to trigger the conversion. This
 * is done for hardware-timed conversions as there is the potential for a long delay 
 * while the hardware waits for the next transmit burst.
 *
 * This function does not block the caller until the conversion completes. This 
 * is intended for use by the task that contains the battery charger so it does 
 * not block the entire task's operation.
 *
 * @param        request   Pointer to structure that describes the request.
 *
 * @return       non-zero if an error occurs..
 */
static int non_blocking_conversion(POWER_IC_ATOD_REQUEST_T * request)
{
    int error = 0;
    
    tracemsg(_k_d("AtoD - starting non-blocking conversion."));
    
    /* If a signal caused us to continue without getting the mutex, then quit with an error. */
    if(down_interruptible(&postponable_mutex) != 0)
    {
        tracemsg(_k_d("   process received signal while waiting for postponable mutex. Exiting."));
        return -EINTR;
    }
    
    /* If a signal caused us to continue without getting the mutex, then quit with an error. */
    if(down_interruptible(&hardware_mutex) != 0)
    {
        tracemsg(_k_d("   process received signal while waiting for hardware mutex. Exiting."));
        return -EINTR;
    }
    
    /* Remember that we started a new conversion. */
    postponable_conversion_started = true;
    postponable_conversion_event_flag = false;
        
    /* Set up hardware for battery/current conversion. */
    postponable_conversion_polarity = request->curr_polarity;
    error = set_batt_current_conversion(request->curr_polarity);
        
    if(error != 0)
    {
        tracemsg(_k_d("AtoD - error %d setting batt/current conversion."), error);
        goto quit_due_to_error;
    }
        
    /* Unmask the conversion complete interrupt. */
    error = power_ic_event_clear(EVENT_ADCDONEI);
    error |= power_ic_event_unmask(EVENT_ADCDONEI);
    
    if(error != 0)
    {
        tracemsg(_k_d("AtoD - error %d unmasking AtoD interrupt."), error);
        goto quit_due_to_error;
    }
        
    /* Set up timing for this conversion. */
    error = start_conversion(request->timing);
        
    if(error != 0)
    {
        tracemsg(_k_d("AtoD - error %d starting postponable conversion."), error);
        goto quit_due_to_error;
    }              

    /* Allow other conversion access to the hardware (possibly postponing this conversion)
     * then exit. It is up to the caller to check back later for a completed conversion. */
    up(&hardware_mutex);
    up(&postponable_mutex);
    tracemsg(_k_d("AtoD - non-blocking conversion ready."));

    return 0;

quit_due_to_error:
    /* Make sure that the converter is available after an error occurs. Mask the
     * interrupt and release the lockes. */
    power_ic_event_mask(EVENT_ADCDONEI);
    up(&hardware_mutex);
    up(&postponable_mutex);
    
    return error;
}

/*!
 * @brief Conversion complete event handler.
 *
 * This function is the handler for a conversion complete event from the power IC. It 
 * isn't called directly, but is registered with the driver core as a callback for when
 * a conversion complete event occurs.
 *
 * This handler is only used for hardware-timed (postponable) conversions - all other
 * conversions are done through polling to eliminate the overhead of context switching.
 *
 *
 * @param        unused     The event type (not used).
 *
 * @return 1 to indicate the event was handled.
 */
static int atod_complete_event_handler(POWER_IC_EVENT_T unused)
{
    tracemsg(_k_d("AtoD conversion complete event."));
   
    /* The conversion complete interrupt is only used for non-immediate conversions.
     * Since this interrupt has occurred, we want to wake the conversion that's waiting
     * on completion. */
    postponable_conversion_event_flag = true;
    wake_up(&postponable_complete_queue);
    
    /* Get the results of the conversion before they are possibly overwritten by an
     * exclusive conversion. */
    postponable_results_error = get_results(postponable_results);
    
    /* We're done with the converter. */
    disable_converter();
    
    /* This should pretty much handle all conversion complete events... */
    return 1;
}
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */

/*!
 * @brief filters bad results from samples to be averaged.
 *
 * This function takes a set of samples (presumably all from the same channel) and
 * calculates an average of the samples, discarding those that look bad so they aren't
 * used in the average. This function only filters what look like abnormally low samples.
 *
 * This function is currently only intended for the mobport channel. Currently we're seeing
 * varying results in the set of samples in this channel - most of the samples are OK, but
 * randomly one or two of the samples will be at half the level of all the others in the set.
 * We don't know why this is happening just yet, so this function is to be used to hack 
 * around the problem for now.
 *
 * @param        samples   Points to a set of PCAP_SAMPLES_PER_BANK samples.
 * @param        average   The suggested average for this set of samples.
 *
 * @return the number of samples discarded. For example, 0 means all the samples were good,
 * 1 shows that one of the samples was discarded and not used in the average, etc.
 */
static int HACK_sample_filter(int * samples, int * average)
{
    int highest_sample = 0;
    int bad_sample_threshold;
    int filtered_average = 0;
    int num_samples = 0;
    int i;

    /* First, find out the highest of the samples. */
    for(i=0; i<SAMPLES_PER_BANK; i++)
    {
        if(samples[i] > highest_sample)
        {
            highest_sample = samples[i];
        }
    }
    
    /* This will be our threshold for rejecting samples. */
    bad_sample_threshold = highest_sample - HACK_SAMPLE_FILTER_OFFSET;
    
    /* Now, walk through the set of samples, ignoring the samples that are too many counts
     * below the highest one in the set. */
    for(i=0; i<SAMPLES_PER_BANK; i++)
    {
        /* If sample is above the threshold, then use it for averaging. */
        if(samples[i] >= bad_sample_threshold)
        {
            filtered_average += samples[i];
            num_samples++;
        }
    }
    
    /* If any samples were good, then go ahead and average them. */
    if(num_samples > 0)
    {
        *average = filtered_average / num_samples;
    }
    
    /* Figure out how many samples were disacarded. */
    return(SAMPLES_PER_BANK - num_samples);
}

/*!
 * @brief Indicates if the atod channel is available
 *
 * This function indicates whether the channel passed is available on this power IC.
 *
 * @param        channel  The channel to look up
 *                        
 * @return returns the hardware channel.
 */
static int atod_channel_available(POWER_IC_ATOD_CHANNEL_T channel)
{
    return atod_channel_lookup[channel];
}

/*******************************************************************************************
 * Global functions to be called only from driver
 ******************************************************************************************/

/*!
 * @brief ioctl() handler for the AtoD interface.
 *
 * This function is the ioctl() interface handler for all AtoD operations. It is not called
 * directly through an ioctl() call on the power IC device, but is executed from the core ioctl
 * handler for all ioctl requests in the range for AtoD operations.
 *
 * @param        cmd       ioctl() request received.
 * @param        arg       typically points to structure giving further details of conversion.
 *
 * @return 0 if successful.
 */
int atod_ioctl(unsigned int cmd, unsigned long arg)
{
    /* This is scratch space where the data passed by the caller will be stored teemporarily.
     * Since there are several possible structures that can be passed with an AtoD ioctl()
     * request, it is wasteful to have one of each on the stack when only one will be used in
     * any given call. */
    int temp[10];
    int error;
    
    /* Handle the request. */
    switch(cmd)
    {
        case POWER_IC_IOCTL_ATOD_SINGLE_CHANNEL:
            /* Get the data accompanying the request. from user space. */
            if(copy_from_user((void *)temp, (void *)arg, 
                             sizeof(POWER_IC_ATOD_REQUEST_SINGLE_CHANNEL_T)) != 0)
            {
                tracemsg(_k_d("   error copying single channel data from user space."));
                return -EFAULT;
            }
            
            /* Convert the channel requested. */
            if((error = power_ic_atod_single_channel( 
                             ((POWER_IC_ATOD_REQUEST_SINGLE_CHANNEL_T *) temp)->channel, 
                            &((POWER_IC_ATOD_REQUEST_SINGLE_CHANNEL_T *) temp)->result)) != 0)
            {
                return error;
            }
                                          
            /* Return the conversion result back to the user. */
            if(put_user( ((POWER_IC_ATOD_REQUEST_SINGLE_CHANNEL_T *) temp)->result,
                         &((POWER_IC_ATOD_REQUEST_SINGLE_CHANNEL_T *) arg)->result) != 0)
            {
                tracemsg(_k_d("   error copying conversion result to user space."));
                return -EFAULT;
            }
            
            break;

        case POWER_IC_IOCTL_ATOD_BATT_AND_CURR_PHASED:
        case POWER_IC_IOCTL_ATOD_BATT_AND_CURR:
	
            /* Get the data accompanying the request.from user space. */
            if(copy_from_user((void *)temp, (void *)arg, 
                  sizeof(POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T)) != 0)
            {
                tracemsg(_k_d("   error copying batt/curr data from user space."));
                return -EFAULT;
            }
            
            /* Convert both the battery and current. */
            if((error = power_ic_atod_current_and_batt_conversion( 
                              ((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->timing,
                              ((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->timeout_secs,
                              ((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->polarity, 
                             &((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->batt_result,
                             &((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->curr_result)) != 0)
            {
                return error;
            }
            
	    /* Since the POWER_IC_IOCTL_ATOD_BATT_AND_CURR will be used during normal operation.  The follow if will be used 
	     * to convert the curr_result from DACs to milliamps since this an easier number to work with.  However; the 
	     * POWER_IC_IOCTL_ATOD_BATT_AND_CURR_PHASED will be used primarily for phasing it should remain it DACs.  
	     */
	     
	    if (cmd == POWER_IC_IOCTL_ATOD_BATT_AND_CURR)
	    { 
                ((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->curr_result =
                    convert_to_milliamps(((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->curr_result);
             
                /* Report all charging currents as negative. Convention is that positive currents are
                 * drawn from battery. */
                if(((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->polarity == POWER_IC_ATOD_CURR_POLARITY_CHARGE)
                {
                     ((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->curr_result = 
		         -((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->curr_result;
                }    		    
            }
	    
            /* Return the conversion result back to the user. */
            if(copy_to_user((void *) arg, (void *) temp, 
                   sizeof(POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T)) != 0)
            {
                tracemsg(_k_d("   error copying conversion results to user space."));
                return -EFAULT;
            }    
            
            break;
            
        case POWER_IC_IOCTL_ATOD_GENERAL:
            /* No input data accompanies this request from user-space. Just perform the 
             * conversion. */
            if((error = power_ic_atod_general_conversion(
                                     (POWER_IC_ATOD_RESULT_GENERAL_CONVERSION_T *) temp)) != 0)
            {
                tracemsg(_k_d("   error copying general data from user space."));
                return error;
            }
                                     
            /* Return the conversion results back to the user. */
            if(copy_to_user((void *)arg, (void *) temp, 
                   sizeof(POWER_IC_ATOD_RESULT_GENERAL_CONVERSION_T)) != 0)
            {
                tracemsg(_k_d("   error copying conversion results to user space."));
                return -EFAULT;
            }        

            break;
            
        case POWER_IC_IOCTL_ATOD_TOUCHSCREEN_POSITION:
            /* No input data accompanies this request from user-space. Just perform the 
             * conversion. */
            tracemsg(_k_d("=> Touchscreen position AtoD conversion"));
        
            if((error = power_ic_atod_touchscreen_position_conversion(
                              &((POWER_IC_ATOD_RESULT_TOUCHSCREEN_T *) temp)->x, 
                              &((POWER_IC_ATOD_RESULT_TOUCHSCREEN_T *) temp)->y)) != 0)
            {
                return error;
            }
                              
            /* Return the conversion result back to the user. */
            if(copy_to_user( (void *) arg, (void *) temp, 
                   sizeof(POWER_IC_ATOD_RESULT_TOUCHSCREEN_T)) != 0)
            {
                tracemsg(_k_d("   error copying conversion results to user space."));
                return -EFAULT;
            }     
            break;
        
        case POWER_IC_IOCTL_ATOD_TOUCHSCREEN_PRESSURE:
            /* No input data accompanies this request from user-space. Just perform the 
             * conversion. */
            tracemsg(_k_d("=> Touchscreen pressure AtoD conversion"));
        
            if((error = power_ic_atod_touchscreen_pressure_conversion(
                              &((POWER_IC_ATOD_RESULT_TOUCHSCREEN_T *) temp)->pressure)) != 0)
            {
                return error;
            }
                              
            /* Return the conversion result back to the user. */
            if(copy_to_user( (void *) arg, (void *) temp, 
                   sizeof(POWER_IC_ATOD_RESULT_TOUCHSCREEN_T)) != 0)
            {
                tracemsg(_k_d("   error copying conversion results to user space."));
                return -EFAULT;
            }     
            break;
                
        case POWER_IC_IOCTL_ATOD_RAW:
            /* Get the data accompanying the request. from user space. */
            if(copy_from_user((void *)temp, (void *)arg, 
                             sizeof(POWER_IC_ATOD_REQUEST_RAW_T)) != 0)
            {
                tracemsg(_k_d("   error copying raw request data from user space."));
                return -EFAULT;
            }
        
            tracemsg(_k_d("=> Raw conversion on channel %d"),
                             ((POWER_IC_ATOD_REQUEST_RAW_T *) temp)->channel);
            
            /* Convert the channel requested. */
            if((error = power_ic_atod_raw_conversion(
                             ((POWER_IC_ATOD_REQUEST_RAW_T *) temp)->channel, 
                             ((POWER_IC_ATOD_REQUEST_RAW_T *) temp)->results,
                             &((POWER_IC_ATOD_REQUEST_RAW_T *) temp)->num_results)) != 0)
            {
                return error;
            }
                                          
            /* Return the conversion results back to the user. */
            if(copy_to_user((void *) arg, (void *) temp, 
                   sizeof(POWER_IC_ATOD_REQUEST_RAW_T)) != 0)
            {
                tracemsg(_k_d("   error copying conversion results to user space."));
                return -EFAULT;
            }
            
            break;
            
        case POWER_IC_IOCTL_ATOD_BEGIN_CONVERSION:
            /* Get the data accompanying the request.from user space. */
            if(copy_from_user((void *)temp, (void *)arg, 
                  sizeof(POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T)) != 0)
            {
                tracemsg(_k_d("   error copying begin conversion data from user space"));
                return -EFAULT;
            }
            
            tracemsg(_k_d("=> Non-blocking convert batt and current: timing %d, polarity %d"), 
                             ((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->timing, 
                             ((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->polarity);  
                             
                             
            /* Kick off the conversion. This will not wait for completion. */
            if((error = power_ic_atod_nonblock_begin_conversion( 
                              ((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->timing,
                              ((POWER_IC_ATOD_REQUEST_BATT_AND_CURR_T *) temp)->polarity)) != 0)
            {
                return error;
            }
            
            break;
            
        case  POWER_IC_IOCTL_ATOD_CANCEL_CONVERSION:
            tracemsg(_k_d("=> Cancelling non-blocking AtoD conversion."));
            
            /* This interface doesn't use any parameters. Just try to cancel the conversion. */
            return(power_ic_atod_nonblock_cancel_conversion());
            break;
            
        case POWER_IC_IOCTL_ATOD_SET_PHASING:
            tracemsg(_k_d("=> Setting AtoD phasing values."));
            
            /* Get the phasing values passed in from user space. */
            if(copy_from_user((void *)temp, (void *)arg, 
                  ATOD_PHASING_NUM_VALUES) != 0)
            {
                tracemsg(_k_d("   error copying phasing data from user space."));
                return -EFAULT;
            }
            
            return set_phasing_values((unsigned char *)temp);
            break;
            
        default: /* This shouldn't be able to happen, but just in case... */
            tracemsg(_k_d("=> 0x%X unsupported AtoD ioctl command"), (int) cmd);
            return -ENOTTY;
            break;
  
        }

    return 0;
}

/*!
 * @brief the poll() handler for the AtoD interface.
 *
 * This function will add the postponable AtoD wait queue to the poll table to allow
 * for polling for conversion complete to occur. If the conversion completes, the 
 * function returns and indicates that the results are ready.
 
 * @param        file        file pointer
 * @param        wait        poll table for this poll()
 *
 * @return 0 if no data to read, (POLLIN|POLLRDNORM) if results available
 */

unsigned int atod_poll(struct file *file, poll_table *wait)
{
    unsigned int retval = 0;

    /* Add our wait queue to the poll table */
    poll_wait (file, &postponable_complete_queue, wait);

    /* If there are changes, indicate that there is data available to read */
    if (postponable_conversion_event_flag)
    {
        /* If an error somehow occurred getting the results, report it. */
        if(postponable_results_error)
        {
            retval = (POLLERR | POLLRDNORM);
        }
        else /* Everything's OK - the data is ready to be read. */
        {
            retval = (POLLIN | POLLRDNORM);
        }
    }
    return retval;
}


/*!
 * @brief the read() handler for the AtoD interface.
 *
 * This function implements the read() system call for the AtoD interface. The 
 * function returns up to one conversion result per call as a a pair of signed
 * ints. The first int is the averaged battery measurement and the second is the
 * averaged current in milliamps.
 *
 * If no events are pending, the function will return immediately with a return
 * value of 0, indicating that no data was copied into the buffer.
 *
 * @param        file        file pointer
 * @param        buf         buffer pointer to copy the data to
 * @param        count       size of buffer
 * @param        ppos        read position (ignored)
 *
 * @return 0 if no data is available, 8 if results are copied, < 0 on error
 */

ssize_t atod_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    ssize_t retval = 0;
    int results[2];
    int i;
    int average = 0;
    
    /* If there isn't space in the buffer for the results, return a failure. Do not 
     * consider the process complete if this happens as the results are still available. */
    if(count < sizeof(results))
    {
        return -EFBIG;
    }
    
    /* Only get the results if they're available. */
    if(postponable_conversion_event_flag)
    {
        if(!postponable_results_error)
        {        
            /* Average the battery voltage. */
            for(i=0; i < (SAMPLES_PER_BANK - 1); i+=2)
            {
                average += postponable_results[i];
            }
            results[0] = phase_channel(POWER_IC_ATOD_CHANNEL_BATT, average / (SAMPLES_PER_BANK/2));
            
            /* Average all of the current samples taken. */
            average = 0;
            for(i=1; i<SAMPLES_PER_BANK; i+=2)
            {
                average += postponable_results[i];
            }
            results[1] = convert_to_milliamps(phase_atod_10((signed char)phasing_values[ATOD_PHASING_BATT_CURR_OFFSET], 
                                              phasing_values[ATOD_PHASING_BATT_CURR_SLOPE], 
                                              (average / (SAMPLES_PER_BANK/2))));
            
            /* Copy the results back to user-space. */
            if( !(copy_to_user((int *)buf, (void *)results, sizeof(results))) )
            {
                /* Data was copied successfully. */
                retval = sizeof(results);
            }
            else
            {
                return -EFAULT;
            }
        }
        else
        {
            /* There was an error getting the results after the conversion. Set up
             * to return with an error. */
            retval = postponable_results_error;
        }

        if(down_interruptible(&postponable_mutex) != 0)
        {
            tracemsg(_k_d("   process received signal while waiting for postponable mutex. Exiting."));
            return -EINTR;
        }
        
        /* The results have been read, so the postponable conversions can be reset. */
        postponable_conversion_started = false;
        postponable_conversion_event_flag = false;
        up(&postponable_mutex);
        tracemsg(_k_d("AtoD - process %d released postponable conversion lock."), current->pid);        
    }

    return retval;
}

/*!
 * @brief Initialisation function for the AtoD interface.
 *
 * This function initialises the AtoD interface. It should be called only once from the 
 * driver init routine.
 *
 * @return 0 if successful.
 */
int __init atod_init(void)
{
    tracemsg(_k_d("Initialising AtoD interface."));
    
    /* Register our handler for AtoD complete event. */
    power_ic_event_subscribe(EVENT_ADCDONEI, &atod_complete_event_handler);
     
    return 0;
}


/*!
 * @brief Converts the battery voltage and current.
 *
 * This function begins a conversion of the battery and current channels, but does not 
 * sleep or otherwise wait for the conversion to complete. The caller is therefore not
 * blocked and is free to do other stuff while the conversion proceeds. It is then the
 * caller's responsibility to then poll for the conversion's completion.
 *
 *
 * As with the blocking version of the battery and current conversion, the hardware 
 * ensures that the current measured is that at the same point in time that the voltage
 * is sampled, the idea being that a sag in battery voltage due to current drawn can be
 * accounted for.
 *
 * If the timing specified is immediate, then an error will be returned. This interface
 * is only intended for taking conversions that are triggered by the hardware.
 *
 * @param        timing       Indicates the timing needed for the conversion.
 * @param        polarity     The direction in which current should be measured.
 *
 * @return non-zero if an error occurs.
 */
static int power_ic_atod_nonblock_begin_conversion(POWER_IC_ATOD_TIMING_T timing,
                                            POWER_IC_ATOD_CURR_POLARITY_T polarity)
{
    int error = 0;
    
    POWER_IC_ATOD_REQUEST_T request = 
    {
        .type = POWER_IC_ATOD_TYPE_BATT_AND_CURR,
        .timing = timing,
        .curr_polarity = polarity,
    };
    
    if(timing == POWER_IC_ATOD_TIMING_IMMEDIATE)
    {
        /* This interface isn't supposed to be used for immediate conversions. This timing
         * is invalid. */
        return -EINVAL; 
    }
    else
    {
        /* Start the conversion as requested. This will return without waiting for the
         * conversion to complete. */
        error = non_blocking_conversion(&request);
    }
    
    return error;
}

/*!
 * @brief Cancels an outstanding non-blocking conversion.
 *
 * This function cancels non-blocking conversion that was previously started with 
 * power_ic_atod_nonblock_begin_conversion() or through the equivalent ioctl() call.
 * This will disable the interrupt, clear the necessary flags and give up the semaphore
 * so that any other postponable conversions may take place.
 *
 * @pre This function will release the semaphore used to ensure that only one postponable
 * conversion is in progress at a given time. If this function is called while a blocking
 * postponable conversion is in progress, its semaphore will be released and things will
 * go wrong if another conversion is waiting for the semaphore.
 *
 * @return non-zero if an error occurs.
 */
static int power_ic_atod_nonblock_cancel_conversion(void)
{
    int error = 0;
    
    /* If no conversion has been started, then calling this is wrong. */
    if(!postponable_conversion_started)
    {
        tracemsg(_k_d("   Error: no non-blocking conversion in progress. Unable to cancel."));
        return -EINVAL;
    }

    /* If a signal caused us to continue without getting the mutex, then quit with an error. */
    if(down_interruptible(&postponable_mutex) != 0)
    {
        tracemsg(_k_d("   process received signal while waiting for postponable mutex. Exiting."));
        return -EINTR;
    }
  
    error = power_ic_event_mask(EVENT_ADCDONEI);
    
    if(error != 0)
    {
        tracemsg(_k_d("AtoD - error %d disabling AtoD interrupt."), error);
    }
    
    /* Clear the flags involved with a postponable conversion. */
    postponable_conversion_started = false;
    postponable_conversion_event_flag = false;
    
    /* Finally, give up the postponable lock. This will allow other postponable conversions
     * to proceed. */
    up(&postponable_mutex);
    tracemsg(_k_d("AtoD - process %d released postponable conversion lock."), current->pid);
    
    return error;
}


/*******************************************************************************************
 * External interface functions
 ******************************************************************************************/
 
/*!
 * @brief Converts a single AtoD channel.
 *
 * Performs a conversion of a single specified channel that cannot be interrupted 
 * by any other conversion, and returns the average for the conversion results.
 *
 * @pre This function will cause the calling process to sleep while waiting for the 
 * conversion to be completed. Therefore, it cannot be used from an interrupt context.
 *
 * @param        channel   The channel that should be converted..
 * @param        result    Points to where the average of the samples should be stored.
 *
 * @return 0 if successful.
 */
int power_ic_atod_single_channel(POWER_IC_ATOD_CHANNEL_T channel, int * result)
{
    int average;
    int error;
    int i;
    int samples[SAMPLES_PER_BANK];
    int filtered_samples;
    int filtered_average;
    
    POWER_IC_ATOD_REQUEST_T request = 
    {
        .type = POWER_IC_ATOD_TYPE_SINGLE_CHANNEL,
        .channel = channel,
    };
    
    /* Don't allow the conversion to proceed if in interrupt contect. */
    if(in_interrupt())
    {
        tracemsg("single-channel AtoD conversion attempted in interrupt context. Exiting.");
        return -EPERM;
    }

HACK_retry_conversion:
    /* Perform the conversion. This will sleep. */
    error = exclusive_conversion(&request, samples);
    
    if(error != 0)
    {
        *result = 0;
        return error;
    }
    
    average = 0;
    /* For a single channel conversion, the average will be returned. */
    for(i=0; i<SAMPLES_PER_BANK; i++)
    {
        average += samples[i];
    }
    
    average = average / SAMPLES_PER_BANK;
    
    /* If this is the channel for mobportb, do filtering on the results just to make sure. */
    if(channel == POWER_IC_ATOD_CHANNEL_MOBPORTB)
    {
        filtered_samples = HACK_sample_filter(samples, &filtered_average);
        
        /* If any samples were filtered out, use the filtered average instead. */
        if(filtered_samples > 0)
        {
            tracemsg(_k_d("HACK: %d samples filtered - average 0x%X (would have been 0x%X)"),
                     filtered_samples, filtered_average, average);
                     
            /* If too many samples were dropped, then redo the conversion. Jumping around like
             * this sucks, but hopefully this can be removed once we get to the bottom of this. */
            if(filtered_samples > HACK_TOO_MANY_SAMPLES_FILTERED)
            {
                tracemsg(_k_d("HACK: %d samples filtered is too many. Retrying conversion."),
                         filtered_samples);
                goto HACK_retry_conversion;
            }
	    
	    *result = phase_channel(channel, filtered_average);
        }
	else
        {
            *result = phase_channel(channel, average);
        }
    }
    else
    {
        *result = phase_channel(channel, average);
    }
    
    return 0;
}

/*!
 * @brief Converts a general set of AtoD channels.
 * 
 * This function performs a conversion of the general set of channels, including battery, 
 * temperature, etc. Each individual result is returned - the results are not averaged.
 * This conversion cannot be iinterrupted by any other conversion.
 *
 * @pre This function will cause the calling process to sleep while waiting for the 
 * conversion to be completed, and so cannot be used in an interrupt context.
 *
 * @param        result    Points to structure where conversion results should be stored.
 *
 * @return 0 if successful.
 */
int power_ic_atod_general_conversion(POWER_IC_ATOD_RESULT_GENERAL_CONVERSION_T * result)
{
    int error;
    int samples[SAMPLES_PER_BANK];
    
    POWER_IC_ATOD_REQUEST_T request = 
    {
        .type = POWER_IC_ATOD_TYPE_GENERAL,
    };

    /* Don't allow the conversion to proceed if in interrupt contect. */
    if(in_interrupt())
    {
        tracemsg("general AtoD conversion attempted in interrupt context. Exiting.");
        return -EPERM;
    }
    
    /* Perform the conversion. This will sleep. */
    error = exclusive_conversion(&request, samples);
    
    if(error != 0)
    {
        return error;
    }
    
    /* Copy the results individually so no assumptions made about channel ordering. */
    result->coin_cell   = phase_channel(POWER_IC_ATOD_CHANNEL_COIN_CELL,   
                              samples[atod_channel_available(POWER_IC_ATOD_CHANNEL_COIN_CELL)]);
    result->battery     = phase_channel(POWER_IC_ATOD_CHANNEL_BATT,        
                              samples[atod_channel_available(POWER_IC_ATOD_CHANNEL_BATT)]);
    result->bplus       = phase_channel(POWER_IC_ATOD_CHANNEL_BPLUS,       
                              samples[atod_channel_available(POWER_IC_ATOD_CHANNEL_BPLUS)]);
    result->mobportb    = phase_channel(POWER_IC_ATOD_CHANNEL_MOBPORTB,    
                              samples[atod_channel_available(POWER_IC_ATOD_CHANNEL_MOBPORTB)]);
    result->charger_id  = phase_channel(POWER_IC_ATOD_CHANNEL_CHARGER_ID,  
                              samples[atod_channel_available(POWER_IC_ATOD_CHANNEL_CHARGER_ID)]);
    result->temperature = phase_channel(POWER_IC_ATOD_CHANNEL_TEMPERATURE, 
                              samples[atod_channel_available(POWER_IC_ATOD_CHANNEL_TEMPERATURE)]);
    
    return 0;
}

/*!
 * @brief Converts the battery voltage and current.
 *
 * This function performs a conversion of the battery and current channels. The hardware 
 * ensures that the current measured is that at the same point in time that the voltage
 * is sampled, the idea being that a sag in battery voltage due to current drawn can be
 * accounted for.
 *
 * This function can only be used to start immediate conversions.
 *
 * @pre This function will cause the calling process to sleep while waiting for the 
 * conversion to be completed, and so cannot be used in an interrupt context.
 *
 * @param        timing       Indicates the timing needed for the conversion.
 * @param        timeout_secs This request will time out if no conversion takes place. Set to zero
 *                            if no timeout is required.
 * @param        polarity     The direction in which current should be measured.
 * @param        batt_result  Where the battery sample should be stored.
 * @param        curr_result  Where the current sample should be stored.
 *
 * @return POWER_IC_ATOD_CONVERSION_TIMEOUT if conversion timed out, 
 * POWER_IC_ATOD_CONVERSION_COMPLETE if conversion was successfully completed.
 */
int power_ic_atod_current_and_batt_conversion(POWER_IC_ATOD_TIMING_T timing,
                                              int timeout_secs,
                                              POWER_IC_ATOD_CURR_POLARITY_T polarity,
                                              int * batt_result, int * curr_result)
{
    int average = 0;
    int error;
    int i;
    int samples[SAMPLES_PER_BANK];
    
    POWER_IC_ATOD_REQUEST_T request = 
    {
        .type = POWER_IC_ATOD_TYPE_BATT_AND_CURR,
        .timing = timing,
        .curr_polarity = polarity,
        .timeout = timeout_secs,
    };
    
    /* Don't allow the conversion to proceed if in interrupt contect. */
    if(in_interrupt())
    {
        tracemsg("batt/current AtoD conversion attempted in interrupt context. Exiting.");
        return -EPERM;
    }
    
    if(timing == POWER_IC_ATOD_TIMING_IMMEDIATE)
    {
        /* Perform the conversion immediately. This will sleep. */
        error = exclusive_conversion(&request, samples);
    }
    else
    {
         /* The non-blocking interface must be used for non-immediate conversions. */
        return -EINVAL;
    }
    
    /* Average the battery voltage. */
    for(i=0; i < (SAMPLES_PER_BANK - 1); i+=2)
    {
        average += samples[i];
    }
    *batt_result = phase_channel(POWER_IC_ATOD_CHANNEL_BATT, average / (SAMPLES_PER_BANK/2));
        
    /* Average all of the current samples taken. */
    average = 0;
    for(i=1; i<SAMPLES_PER_BANK; i+=2)
    {
        average += samples[i];
    }
    average = average / (SAMPLES_PER_BANK/2);
    *curr_result = phase_atod_10((signed char)phasing_values[ATOD_PHASING_BATT_CURR_OFFSET], 
                         phasing_values[ATOD_PHASING_BATT_CURR_SLOPE], average);
   

    
    return error;
}

/*!
 * @brief Performs a conversion of the touchscreen position.
 *
 * This function performs a conversion of the touchscreen x and y positions. This 
 * conversion cannot be interrupted by any other conversion.
 *
 * @param        x         Converted x position will be stored here.
 * @param        y         Converted y position will be stored here.
 *
 * @return 0 if successful.
 */
int power_ic_atod_touchscreen_position_conversion(int * x, int * y)
{
    int error;
    int samples[SAMPLES_PER_BANK];
    
    POWER_IC_ATOD_REQUEST_T request = 
    {
        .type = POWER_IC_ATOD_TYPE_TOUCHSCREEN_POSITION,
    };
    
    /* Don't allow the conversion to proceed if in interrupt contect. */
    if(in_interrupt())
    {
        tracemsg("touchscreen position AtoD conversion attempted in interrupt context. Exiting.");
        return -EPERM;
    }

    /* Perform the conversion. This will sleep. */
    error = exclusive_conversion(&request, samples);
    
    if(error != 0)
    {
        return error;
    }
    
    /* Copy the results individually so no assumptions made about channel ordering. For the
     * touchscreen, no averaging is possible. */
    *x = samples[TOUCHSCREEN_CONV_CHANNEL_X];
    *y = samples[TOUCHSCREEN_CONV_CHANNEL_Y];
    
    return 0;
}

/*!
 * @brief Performs a conversion of the touchscreen pressure.
 *
 * This function performs a conversion of the touchscreen pressure. This 
 * conversion cannot be interrupted by any other conversion.
 *
 * @param        p         Converted pressure will be stored here.
 *
 * @return 0 if successful.
 */
int power_ic_atod_touchscreen_pressure_conversion(int * p)
{
    int error;
    int samples[SAMPLES_PER_BANK];
    
    POWER_IC_ATOD_REQUEST_T request = 
    {
        .type = POWER_IC_ATOD_TYPE_TOUCHSCREEN_PRESSURE,
    };
    
    /* Don't allow the conversion to proceed if in interrupt contect. */
    if(in_interrupt())
    {
        tracemsg("touchscreen pressure AtoD conversion attempted in interrupt context. Exiting.");
        return -EPERM;
    }

    /* Perform the conversion. This will sleep. */
    error = exclusive_conversion(&request, samples);
    
    if(error != 0)
    {
        return error;
    }
    
    /* Copy the results individually so no assumptions made about channel ordering. For the
     * touchscreen, no averaging is possible. */
    *p = samples[TOUCHSCREEN_CONV_CHANNEL_PRESSURE];
    
    return 0;
}

/*!
 * @brief Performs a conversion for testing the converter hardware.
 *
 * This function performs a set of conversions on a single channel similar to the 
 * single channel request, but instead returns all of the individual measurements 
 * unphased and in the same format as read from the hardware. This conversion 
 * will be performed immediately if the converter is not in use, and cannot be 
 * interrupted by any other conversion.
 *
 * @pre This function will cause the calling process to sleep while waiting for the 
 * conversion to be completed, and so cannot be used in an interrupt context.
 *
 * @param        channel   This channel will be converted.
 * @param        samples   POWER_IC_ATOD_NUM_RAW_RESULTS samples will be stored here.
 * @param        length    Number of samples being stored.
 *
 * @return 0 if successful.
 */
int power_ic_atod_raw_conversion(POWER_IC_ATOD_CHANNEL_T channel, int * samples, int * length)
{
    POWER_IC_ATOD_REQUEST_T request = 
    {
        .type = POWER_IC_ATOD_TYPE_RAW,
        .channel = channel,
    };
    
    *length = ATOD_NUM_RAW_RESULTS;

    /* Don't allow the conversion to proceed if in interrupt contect. */
    if(in_interrupt())
    {
        tracemsg("raw AtoD conversion attempted in interrupt context. Exiting.");
        return -EPERM;
    }

    /* Perform the conversion. This will sleep. The results will be copied straight to the
     * caller's array with no post-processing. */
    return(exclusive_conversion(&request, samples));
}

/*!
 * @brief Sets the touchscreen to be disabled, in low power mode, or full power mode.
 *
 * This function programs the hardware to put the touchscreen source back in low-power 
 * mode if in standy, in full-power mode if in position or pressure mode, and completely
 * disabled. 
 *
 * @param  mode   Mode in which Touchscreen should be set.
 *
 * @return 0 if successful.
 */
int power_ic_atod_set_touchscreen_mode(POWER_IC_TS_MODES_T mode)
{
    int error;
    
    switch(mode)
    {
        case POWER_IC_TS_MODE_DISABLED:
            /* Disable Hardware -- 1 to disable*/
            error = power_ic_set_reg_bit (TS_LOW_PWR_MOD_REG, TS_REFENB_MASK, 1);
            break;
        
        case POWER_IC_TS_MODE_LOW_PWR_MODE:
            error = power_ic_set_reg_mask(TOUCHSCREEN_STANDBY_REG,(TS_M_MASK | TS_REFENB_MASK
                | TS_REF_LOW_PWR_MASK), ((TOUCHSCREEN_STANDBY << TS_M_SHIFT) | TS_REF_LOW_PWR_MASK));
            break;
            
        case POWER_IC_TS_MODE_FULL_PWR_MODE:
            error = power_ic_set_reg_mask(TS_LOW_PWR_MOD_REG, (TS_REF_LOW_PWR_MASK 
                | TS_REFENB_MASK), 0);
            break;
            
        default:
            return -EINVAL;
            break;
        
    }
    
    return (error);
}
