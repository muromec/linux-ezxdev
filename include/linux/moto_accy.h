/*
 * Copyright (C) 2004-2005 Motorola, Inc.
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
 * Created by Sierra Team of Motorola
 */

#ifndef __MOTO_ACCY_H__
#define __MOTO_ACCY_H__

/*!
 * @file moto_accy.h
 *
 * @brief This file contains defines, macros and prototypes for the accessory
 * interface.
 *
 * @ingroup poweric_accy
 */

/******************************************************************************
* Include files
******************************************************************************/

#include <linux/bitops.h>

/******************************************************************************
* Macros and Constants
******************************************************************************/

/* device driver node information */
#define MOTO_ACCY_DRIVER_MAJOR_NUM 221
#define MOTO_ACCY_DRIVER_DEV_NAME "accy"


/*! Flag indicating connected accy versus removed in reply from read() */
#define MOTO_ACCY_CONNECTED_FLAG 0x80000000


/*! Macro for setting a bit in a bitmask of accessories */
#define ACCY_BITMASK_SET(mask,accy) ((mask) |= (1 << (accy)))

/*! Macro for setting all bits in a bitmask of accessories */
#define ACCY_BITMASK_SET_ALL(mask) ((mask) = (MOTO_ACCY_MASK_T)(0xFFFFFFFF))

/*! Macro for clearing a bit in a bitmask of accessories */
#define ACCY_BITMASK_CLEAR(mask,accy) ((mask) &= ~(1 << (accy)))

/*! Macro for clearing all bits in a bitmask of accessories */
#define ACCY_BITMASK_CLEAR_ALL(mask) ((mask) = (MOTO_ACCY_MASK_T)(0x00000000))

/*! Macro for checking a bit in a bitmask of accessories */
#define ACCY_BITMASK_ISSET(mask,accy) (((mask) & (1 << (accy))) != 0)

/*! Macro for checking for any bits in a bitmask of accessories */
#define ACCY_BITMASK_ISEMPTY(mask) ((mask) == 0)

/*!
 * Macro for finding the first bit set in a bitmask of accessories.  Note
 * that you MUST guarantee that the mask is not empty before using this
 * macro.
 *
 * @note This should probably use ffs() instead of generic_ffs(), but that
 * causes build errors for unknown reasons.
 */
#define ACCY_BITMASK_FFS(mask) (generic_ffs((mask)) - 1)

/******************************************************************************
* Types
******************************************************************************/

/*! Enumeration holding all of the possible accessories that are supported */
typedef enum
{
    /*! Indicates that no accessory is connected */
    MOTO_ACCY_TYPE_NONE,                /*0*/

    /*! Indicates that an invalid or broken accessory is connected */
    MOTO_ACCY_TYPE_INVALID,             /*1*/

    /*! Indicates that a unsupported (ie. OTG, Smart) accessory is connected */
    MOTO_ACCY_TYPE_NOT_SUPPORTED,       /*2*/

    /*! Charger-type accessories */
    MOTO_ACCY_TYPE_CHARGER_MID,         /*3*/
    MOTO_ACCY_TYPE_CHARGER_MID_MPX,     /*4*/
    MOTO_ACCY_TYPE_CHARGER_FAST,        /*5*/
    MOTO_ACCY_TYPE_CHARGER_FAST_MPX,    /*6*/
    MOTO_ACCY_TYPE_CHARGER_FAST_3G,     /*7*/

    /*! Carkit-type accessories */
    MOTO_ACCY_TYPE_CARKIT_MID,          /*8*/
    MOTO_ACCY_TYPE_CARKIT_FAST,         /*9*/
    MOTO_ACCY_TYPE_CARKIT_SMART,        /*10*/

    /*! Data cable-type accessories */
    MOTO_ACCY_TYPE_CABLE_USB,           /*11*/
    MOTO_ACCY_TYPE_CABLE_REGRESSION,    /*12*/
    MOTO_ACCY_TYPE_CABLE_FACTORY,       /*13*/

    /*! Headset-type accessories */
    MOTO_ACCY_TYPE_HEADSET_MONO,        /*14*/
    MOTO_ACCY_TYPE_HEADSET_STEREO,      /*15*/
    MOTO_ACCY_TYPE_HEADSET_EMU_MONO,    /*16*/
    MOTO_ACCY_TYPE_HEADSET_EMU_STEREO,  /*17*/

    /*
     * NOTE: if number of accessories grows larger than the number of bits in
     * an unsigned long int, then the macros for accessing accessory masks need to change.
     */
    MOTO_ACCY_TYPE_3MM5_HEADSET_STEREO,		/*18*/
    MOTO_ACCY_TYPE_3MM5_HEADSET_STEREO_MIC,	/*19*/
    /* Insert new accessories before this */
    MOTO_ACCY_TYPE__NUM_DEVICES         /*20*/
} MOTO_ACCY_TYPE_T;

/*! Type holding a bitmask of accessories */
typedef unsigned long int MOTO_ACCY_MASK_T;

/*! Structure holding device state information */
typedef struct
{
    MOTO_ACCY_TYPE_T device; /*!< Device to which the following state information applies */
    int state; /*!< device state (could be connected/disconnected or muted/unmuted) */
} MOTO_ACCY_DEVICE_STATE_T;

/*Defines the emu headset mode*/
typedef enum
{
    MOTO_ACCY_HEADSET_MODE_NONE,
    MOTO_ACCY_HEADSET_MODE_MONO,
    MOTO_ACCY_HEADSET_MODE_STEREO
}MOTO_ACCY_HEADSET_MODE_T;

/******************************************************************************
* ioctl Definitions
******************************************************************************/

/* ioctl numbers */
#define MOTO_ACCY_IOC_GET_INTERESTED_MASK    0x00
#define MOTO_ACCY_IOC_SET_INTERESTED_MASK    0x01
#define MOTO_ACCY_IOC_GET_DEVICE_STATE       0x02
#define MOTO_ACCY_IOC_SET_CHARGER_LOAD_LINE  0x03
#define MOTO_ACCY_IOC_SET_MUTE_STATE         0x04
#define MOTO_ACCY_IOC_GET_ALL_DEVICES        0x05

/*! 
 * @name ioctl requests
 * 
 * The following definitions are the requests that a user-space process can make
 * of the driver via calls to ioctl().
 */

/* @{ */

/*!
 * @brief ioctl for getting the mask of interesting accessories (accessories about which
 * events will be reported).
 *
 * The mask contains one bit per accessory with accessory * number 0 (MOTO_ACCY_TYPE_NONE) 
 * being represented in the least significant bit. If a bit is set in the mask, it 
 * indicates that events will be reported for that accessory.  If a bit is clear, 
 * no events will be reported for the accessory.  The final parameter to ioctl() 
 * should be a pointer to a MOTO_ACCY_MASK_T that will be filled in with the mask.
 */
#define MOTO_ACCY_IOCTL_GET_INTERESTED_MASK \
    _IOR(MOTO_ACCY_DRIVER_MAJOR_NUM, MOTO_ACCY_IOC_GET_INTERESTED_MASK, MOTO_ACCY_MASK_T *)

/*!
 * @brief ioctl for setting the mask of interesting accessories (accessories about which
 * events will be reported).
 *
 * The mask contains one bit per accessory with accessory number 0 (MOTO_ACCY_TYPE_NONE) 
 * being represented in the least significant bit. If a bit is set in the mask, it 
 * indicates that events will be reported for that accessory.  If a bit is clear, 
 * no events will be reported for the accessory.  The final parameter to ioctl() 
 * should be a pointer to a MOTO_ACCY_MASK_T that will be used to set the mask of 
 * interesting accessories.
 */
#define MOTO_ACCY_IOCTL_SET_INTERESTED_MASK \
    _IOW(MOTO_ACCY_DRIVER_MAJOR_NUM, MOTO_ACCY_IOC_SET_INTERESTED_MASK, MOTO_ACCY_MASK_T *)

/*!
 * @brief ioctl for getting the state of a device (connected or not connected).  
 *
 * The final parameter to ioctl() should be a pointer to a MOTO_ACCY_DEVICE_STATE_T.  
 * When calling ioctl(), the device field of the MOTO_ACCY_DEVICE_STATE_T structure should 
 * be set to the device for which the state information is requested.  When ioctl() returns
 * with a successful return value, the state field of the provided MOTO_ACCY_DEVICE_STATE_T
 * structure will be filled in with the state of the device.  A state of 0 indicates
 * that the accessory is not connected and a state of 1 indicates that the accessory
 * is connected.
 */
#define MOTO_ACCY_IOCTL_GET_DEVICE_STATE \
    _IOR(MOTO_ACCY_DRIVER_MAJOR_NUM, MOTO_ACCY_IOC_GET_DEVICE_STATE, MOTO_ACCY_DEVICE_STATE_T *)

/*!
 * @brief ioctl for controlling the load line of the currently connected charger accessory.
 *
 * The final parameter is a pointer to an unsigned long integer representing the requested
 * state of the charger load line.  A value of 0 indicates the low-power load line (default
 * for all chargers).  A value of 1 indicates the high-power load line.  The ioctl() will
 * only return a successful indication when a 3G fast charger is connected since that is
 * the only charger accessory that supports an adjustable load line.
 */
#define MOTO_ACCY_IOCTL_SET_CHARGER_LOAD_LINE \
    _IOW(MOTO_ACCY_DRIVER_MAJOR_NUM, MOTO_ACCY_IOC_SET_CHARGER_LOAD_LINE, unsigned long *)

/*!
 * @brief ioctl for setting the mute state of a connected accessory.
 *
 * The final parameter to ioctl() is a pointer to a MOTO_ACCY_DEVICE_STATE_T structure.  
 * When calling ioctl(), the device field of the structure should be set to the device
 * for which the mute state is requested to be changed.  The state field should be set 
 * to the requested mute state with 0 indicating unmuted and 1 indicating muted. 
 * ioctl() will return a successful indication only if the indicated accessory is 
 * currently connected and supports a mute operation.
 */
#define MOTO_ACCY_IOCTL_SET_MUTE_STATE \
    _IOW(MOTO_ACCY_DRIVER_MAJOR_NUM, MOTO_ACCY_IOC_SET_MUTE_STATE, MOTO_ACCY_DEVICE_STATE_T *)
    
/*!
 * @brief ioctl for getting the state of all devices.
 *
 * The final parameter to ioctl() is a point to a MOTO_ACCY_MASK_T. iotcl() will return
 * a bitmask of all accessories as defined for the MOTO_ACCY_MASK type. A bit set in
 * a given postiion means that device is attached. For each device in the mask, a bit set
 * indicates that device is attached.
 */
#define MOTO_ACCY_IOCTL_GET_ALL_DEVICES \
    _IOR(MOTO_ACCY_DRIVER_MAJOR_NUM, MOTO_ACCY_IOC_GET_ALL_DEVICES, MOTO_ACCY_MASK_T *)
/* @} */

/******************************************************************************
* Function Prototypes
******************************************************************************/

#ifdef __KERNEL__

/*!
 * @name Kernel-space accessory interface functions
 *
 * These functions are exported by the driver to allow other kernel-space code to control
 * various accessory operations. For more information, see the documentation for the 
 * accy.c file.
 */
/* @{ */
void moto_accy_init (void);
void moto_accy_destroy (void);
void moto_accy_notify_insert (MOTO_ACCY_TYPE_T accy);
void moto_accy_notify_remove (MOTO_ACCY_TYPE_T accy);
MOTO_ACCY_MASK_T moto_accy_get_all_devices(void);
void moto_accy_set_accessory_power(MOTO_ACCY_TYPE_T device, int on_off);
/* @} */

#endif /* __KERNEL__ */

#endif /* __MOTO_ACCY_H__ */
