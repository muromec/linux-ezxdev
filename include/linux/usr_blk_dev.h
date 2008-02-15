#ifndef __USR_BLK_DRV_H__
#define __USR_BLK_DRV_H__
/*
 * Copyright 2005 Motorola, Inc. All Rights Reserved.
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
/*!
 * @defgroup usr_blk_dev User Block Device
 */
/*!
 * @ingroup usr_blk_dev
 *
 * @file usr_blk_dev.h
 *
 * @brief External interfaces to the user block device.
 *
 * This module is intended to process block device commands and pass the
 * read and write information to user space for processing.  It has be implemented
 * in order to meet the legal obligations put forth by the SD association, by allowing
 * specific proprietary code to remain out of the kernel.
 *
 */
#include <asm/ioctl.h>

/*******************************************************************************************
 * Universal constants
 *
 *    These are constants that are universal to the user block device from both kernel
 *    driver and the user space process.
 ******************************************************************************************/

#define USR_BLK_DEV_MAJOR_NUM    243   /*!< @brief Major number of the user mode block driver. */

#define USR_BLK_DEV_NUM_DEVICES  1     /*!< @brief The number of devices which are supported by the driver. */

#define USR_BLK_DEV_CID_SZ       16    /*!< @brief The number of bytes read from user space by the
                                            USR_BLK_DEV_CMD_ID_READ_CID command. */
#define USR_BLK_DEV_VERSION      0x01  /*!< @brief The version of the kernel driver.  See
                                            USR_BLK_DEV_CMD_T for more details. */

/*
 * Define the ioctls which are needed by the legacy applications.  These are set up
 * to match the existing numbers within linux/include/mmc_protocol.h.  The exception
 * to this is IOCMMCGETCARDCID which reads the CID and returns it in the same format
 * as found in USR_BLK_DEV_CMD_T.
 */
#ifndef IOCMMCGETCARDCID
/*!
 * @brief Get the CID from a media card.
 *
 * Sends the CID which was read from a media card at power up and places it
 * in the location pointed to by parameter.  USR_BLK_DEV_CID_SZ bytes will
 * be copied to the pointer location, as such it must be at least that big.
 */
# define IOCMMCGETCARDCID    _IOR(USR_BLK_DEV_MAJOR_NUM, 0x0f02, char *)
#endif

#ifndef IOCMMCGETCARDSTATUS
/*!
 * @brief Get the the status of the device.
 *
 * Copies the status of the device into the location pointed to by the
 * parameter passed in.  The status flags are as follows:
 *   0x01 - When set the device is write protected.
 *   0x02 - When set the device is locked.
 *   0x04 - When set it indicates the last attempt to lock the device failed.
 *   0x08 - When set it indicates locking is not supported.
 *   0x10 - When set the card has changed since the last time this bit was
 *          cleared.  To clear the bit set it before using this ioctl.  This
 *          bit will be set at power up if a card is inserted.
 */
# define IOCMMCGETCARDSTATUS _IOWR(USR_BLK_DEV_MAJOR_NUM, 0x0f05, unsigned long *)

/*! @brief When set indicates the card is write protected. */
# define IOCMMC_STATUS_WRITE_PROTECTED    0x01

/*! @brief When set indicates the card is locked. */
# define IOCMMC_STATUS_LOCKED             0x02

/*! @brief When set indicates the last attempt to lock the card failed. */
# define IOCMMC_STATUS_LOCK_FAILED        0x04

/*! @brief When set indicates locking/unlocking is not supported. */
# define IOCMMC_STATUS_LOCK_NOT_SUPPORTED 0x08

/*!
 * @brief When set indicates the media has changed since the last time the bit was cleared.
 *
 * When set indicates the media has changed since the last time the bit was cleared.  To clear
 * the bit set it in the parameter to ioctl before calling ioctl.  For example:
 *
 *   val = IOCMMC_STATUS_MEDIA_CHANGED
 *   ioctl(fh, IOCMMCGETCARDSTATUS, &val);
 */
# define IOCMMC_STATUS_MEDIA_CHANGED      0x10
#endif


#ifndef IOCMMCGETCARDTYPE
/*!
 * @brief Gets the type of card which is inserted.
 *
 * Copies the type of device attached into the location pointed to by the
 * parameter passed in.  The card type will be one of the following:
 *
 *   0 - MMC
 *   1 - SDIO
 */
# define IOCMMCGETCARDTYPE   _IOR(USR_BLK_DEV_MAJOR_NUM, 0x0f06, unsigned long *)

# define IOCMMC_TYPE_MMC     0  /*!< @brief The media is an MMC card. */

# define IOCMMC_TYPE_SD      1  /*!< @brief The media is an SD card. */
#endif

#ifndef IOCMMCGETSIZE
/*!
 * @brief Gets the size of the card in bytes
 *
 * Copies the size of the card in bytes to the location pointed to by the
 * parameter passed in.  This is for legacy applications only, any new
 * applications should use BLKGETSIZE which returns the size in sectors.
 */
# define IOCMMCGETSIZE       _IOR(USR_BLK_DEV_MAJOR_NUM, 0x0f09, unsigned long *)
#endif

/*!
 * @brief Tracks the state of removable media.
 *
 * The following is used to track the state of removable media.  For
 * non removable media the only valid states are attached and validated.
 *
 * @note The code relies on the fact that removed is first and all states for
 *       which communication with the card can be done are after error.
 */
typedef enum
{
    /*! The media is not connected. */
    USR_BLK_DEV_MEDIA_STATE_REMOVED,
    /*! The media is attached, but an error was detected during initialization or validation. */
    USR_BLK_DEV_MEDIA_STATE_ERROR,
    /*! The media has been detected as attached, but has not yet been initialized. */
    USR_BLK_DEV_MEDIA_STATE_ATTACHED,
    /*! The media is attached and initialized, but the kernel has not validated it. */
    USR_BLK_DEV_MEDIA_STATE_INITIALIZED,
    /*! The media is attached, initialized, validated and ready for use. */
    USR_BLK_DEV_MEDIA_STATE_VALIDATED,
} USR_BLK_DEV_MEDIA_STATE_T;

/*!
 * @brief Commands which can be sent to the user mode driver.
 *
 * This enumeration defines the commands which can be sent by the usr_blk_dev to
 * user space.  The commands can be read from the device in the proc directory.
 * When a command is written from the kernel block device driver it is written
 * as USR_BLK_DEV_CMD_T, thus it would make sense to read the data into a 
 * variable of the same type in the user space process.  The user space driver
 * can then act on the command and write the return back to the proc entry.
 * The value which is written to the proc entry by the user space process must
 * must be of the type USR_BLK_DEV_CMD_T as well.
 *
 * @note The code in the driver as well as user space requires this enumeration
 * to be in the following order.  If new commands are added they must be added
 * before the END identifier and the command tables in both the user space process
 * and the kernel driver must be updated.
 */
typedef enum
{
    /*! No active command. */
    USR_BLK_DEV_CMD_ID_NONE,
    /*!
     * Initialize the part and return the necessary data in
     * cmd_data_from_usr.init.  The user space process is responsible for
     * reading the block sizes, as well as the part size.  See
     * USR_BLK_DEV_INIT_DATA_T for more details.
     */
    USR_BLK_DEV_CMD_ID_INIT,
    /*!
     * The user space driver must perform any actions needed to handle the
     * removal of the device.  In this case the kernel driver detected the
     * removal of the device.  This command does not have any data.
     */
    USR_BLK_DEV_CMD_ID_REMOVE,
    /*!
     * Read data and place it in a kernel buffer.  A pointer to the data is
     * passed to the user space process in cmd_data_to_usr.rw.  The pointer
     * will be to a buffer in kernel space.  The data read must be placed in
     * the buffer.  The status of the read is sent back to the kernel driver in
     * cmd_data_from_usr.rw_success.  See USR_BLK_DEV_CMD_T and
     * USR_BLK_DEV_RW_DATA_T for more details.
     */
    USR_BLK_DEV_CMD_ID_READ,
    /*!
     * Write data from a kernel buffer.  A pointer to the data is passed to the
     * user space process in cmd_data_to_usr.rw.  The pointer will be to a
     * buffer in kernel space.  The data written must be taken from the buffer.
     * The status of the write is sent back to the kernel driver in
     * cmd_data_from_usr.rw_success.  See USR_BLK_DEV_CMD_T and
     * USR_BLK_DEV_RW_DATA_T for more details.
     */
    USR_BLK_DEV_CMD_ID_WRITE,
    /*!
     * Check if the media has changed.  The status of the change is passed back
     * to the kernel driver in cmd_data_from_usr.changed.  See USR_BLK_DEV_CMD_T
     * for more details.
     */
    USR_BLK_DEV_CMD_ID_MEDIA_CHANGE,
    /*!
     * Revalidate the new media.  Most of the block necessary operations for
     * revalidation are done within the kernel driver.  The user space process
     * may not have to do anything for revalidation, since USR_BLK_DEV_CMD_ID_INIT
     * will be sent if a device insertion is detected. This command does not have
     * any data.
     */
    USR_BLK_DEV_CMD_ID_REVALIDATE,
    /*!
     * Read the CID if the device has a CID register.  Currently this reads 16
     * bytes of data from the register and passes it back to the kernel driver.
     * Not all devices contain a CID, it is generally on MMC and SD media cards.
     * See USR_BLK_DEV_CMD_T for more details.
     */
    USR_BLK_DEV_CMD_ID_READ_CID,
    /*!
     * When this command is received by the user process it may simply ignore
     * the command or perform any operations needed to close the device.  The
     * kernel driver is being unloaded.  This command does not have any data.
     */
    USR_BLK_DEV_CMD_ID_EXIT,
    /*!
     * The last entry in the enumeration.  Any new commands must be added before
     * this entry.
     */
    USR_BLK_DEV_CMD_ID_END
} USR_BLK_DEV_CMD_ID_T;

/*!
 * @brief Return data from the user space process to the kernel driver for the
 *        init command (USR_BLK_DEV_CMD_ID_INIT).
 *
 * This structure contains the data which the kernel driver needs to process
 * block data requests.
 */
typedef struct
{
    /*! true when the kernel and user space versions match, false if they do not. */
    bool version_match;
    /*!
     * The minimum operating voltage in millivolts.  For example 1.9v would
     * be 1900.
     */
    uint16_t min_oper_volt;
    /*! The maximum operating voltage in millivolts. */
    uint16_t max_oper_volt;
    /*< The read block size in bytes. */
    uint16_t read_block_len;
    /*! The write block size in bytes. */
    uint16_t write_block_len;
    /*! The size of the device in sectors. */
    uint32_t device_size;
    /*!
     * true if the device has changed since the last time the command
     * USR_BLK_DEV_CMD_ID_MEDIA_CHANGE was run.  To reset this flag
     * USR_BLK_DEV_CMD_ID_MEDIA_CHANGE must be sent to the user mode
     * driver.  It is included here to simplify the kernel driver.
     */
    bool media_changed;
} USR_BLK_DEV_INIT_DATA_T;

/*!
 * @brief Data sent to the user space process with a read or write command
 *        (USR_BLK_DEV_CMD_ID_READ, USR_BLK_DEV_CMD_ID_WRITE).
 */
typedef struct
{
    uint32_t sector;
    uint32_t num_sectors;
    void *buf;
} USR_BLK_DEV_RW_DATA_T;

/*!
 * @brief Structure used to communicate commands between the kernel driver
 *        and the user space process.
 */
typedef struct
{
    /*! The command to execute. */
    USR_BLK_DEV_CMD_ID_T id;
    /*! The device number for the command to be executed on. */
    uint8_t device_num;
    /*!
     * Tracks the attached status of the media.  Please see USR_BLK_DEV_MEDIA_STATE_T
     * for more details on the exact states.  The primary purpose of this is to
     * inform the user driver when a device has been removed in the middle of a
     * command.  The user driver must poll this value any time it is in a loop
     * waiting for a response from the device.
     */
    USR_BLK_DEV_MEDIA_STATE_T media_state;
    union
    {
        /*!
         * The version of the kernel driver is transfered to the user space
         * process during the init command.  This can be checked against
         * USR_BLK_DEV_VERSION in the user space driver to determine if
         * communication with the kernel driver is possible.
         */
        unsigned int version;
        /*! Data transfered to the user space process during a read. */
        USR_BLK_DEV_RW_DATA_T rw;
    } cmd_data_to_usr;
    union
    {
        /*!
         * Data filled in by the user space process as a response to the init
         * command.
         */
        USR_BLK_DEV_INIT_DATA_T init;
        /*! Data filled in by the user space driver during the check_media command. */
        bool media_changed;
        /*!
         * Data filled in by the user space driver during a read or write command.
         * true when the command was a success, false when an error was encountered.
         */
        bool rw_success;
        /*!
         * Data filled in by the user space driver during the read CID command.
         */
        uint8_t cid[USR_BLK_DEV_CID_SZ];
    } cmd_data_from_usr;
} USR_BLK_DEV_CMD_T;

#endif
