#ifndef __SPI_USER_H__
#define __SPI_USER_H__
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
 * @file spi_user.h
 *
 * @ingroup spi_user
 *
 * @brief Contains user mode SPI interface information (types, enums, macros, functions, etc.)
 */

#include <linux/ioctl.h>
#include <stdbool.h>

/*! @brief Maximum number of vectors that can be included in a single SPI transfer */
#define MOTO_SPI_MAX_VECTORS 5

/*! @brief The major number of the user mode SPI driver. */
#define MOTO_SPI_DRIVER_MAJOR_NUM 223
/*! The name of the user mode SPI driver. */
#define MOTO_SPI_DRIVER_DEV_NAME  "spi_user"

/*!
 * @brief Get the SPI parameters and place in the pointer to MOTO_SPI_PARAMETER_T.
 *
 * Read the SPI parameters from the kernel SPI device driver and copy the data
 * to the user provided pointer.  If the chip select entry contains
 * MOTO_SPI_CS__END then the SPI parameters have not yet been initialized.
 * In this case the other entries are not valid.
 *
 * @return -EFAULT upon an error copying the data to user space.<BR>
 *         0 upon success.
 * 
 * @note See MOTO_SPI_PARAMETER_T for more information on the data copied.
 */
#define MOTO_SPI_IOCTL_GET_SPI_PARAMS \
    _IOR(MOTO_SPI_DRIVER_MAJOR_NUM, 0x00, MOTO_SPI_PARAMETER_T *)

/*!
 * @brief Set the SPI parameters provided by a pointer to MOTO_SPI_PARAMETER_T.
 *
 * Sets the kernel SPI parameters provided by the pointer to
 * MOTO_SPI_PARAMETER_T.  The chip select entry must contain a value less
 * than MOTO_SPI_CS__END (see MOTO_SPI_CS_T) or the data will not be set.
 *
 * @return -EFAULT upon an error copying the data to user space.<BR>
 *         -EINVAL for invalid chip selects.<BR>
 *         0 upon success.
 * 
 * @note See MOTO_SPI_PARAMETER_T for more information on the data copied.
 */
#define MOTO_SPI_IOCTL_SET_SPI_PARAMS \
    _IOW(MOTO_SPI_DRIVER_MAJOR_NUM, 0x01, MOTO_SPI_PARAMETER_T *)

/*!
 * @brief Sets the SPI filler character to the 8 bit value provided.
 *
 * Sets the SPI filler character to the 8 bit value provided.  The filler
 * character will be transmitted in place of NULL buffers which are provided
 * to MOTO_SPI_IOCTL_TRANSCEIVE.  This is provided as a convenience to users
 * of MOTO_SPI_IOCTL_TRANSCEIVE.
 *
 * @return Always returns 0.
 */
#define MOTO_SPI_IOCTL_SET_FILLER \
    _IOW(MOTO_SPI_DRIVER_MAJOR_NUM, 0x02, unsigned char)

/*!
 * @brief Sends and receives data pairs over the SPI.
 *
 * Transceives data provided by the caller over the SPI.  Read and write are
 * not used, since every transmission over the SPI must send and receive data.
 * The data is passed in MOTO_SPI_TRANSCEIVE_T which contains a count and
 * a set of vectors.  Up to MOTO_SPI_MAX_VECTORS can be sent on one call to
 * MOTO_SPI_IOCTL_TRANSCEIVE.
 *   The data can be transfered to/from either kernel memory or user memory.
 * In the case of kernel memory the memory must be DMA accessible and no copy
 * to or from user space will happen.  This is provided for cases when a user
 * application must copy data directly into kernel structures.  (For example if
 * the user space app is used as a helper to the user block device
 * (usr_blk_dev).  If the data is from user space it will be copied into some
 * scratch buffers for transmission by the kernel SPI driver.
 *   If the tx pointer (tx_data) provided in the vectors is null, then the
 * filler character will be transmitted in place of the buffer.  (See
 * MOTO_SPI_IOCTL_SET_FILLER for more details on the filler character.)  If
 * the receive pointer (rx_data) is NULL, then the receive data will be copied
 * into a scratch buffer and discarded.
 *
 * @return -ENOTCONN if the parameters have yet to be setup.<BR>
 *         -EFAULT if an error happens when copying to or from user space.<BR>
 *         -ENOMEM if no memory is available for a transmit or receive buffer.<BR>
 *         0 upon success.
 *
 * @note It is possible to receive -ENOMEM as an error when the receive pointer
 *       (rx_data) is NULL, since the driver may need to allocate memory for
 *       the data which will be returned from the kernel SPI driver.
 */
#define MOTO_SPI_IOCTL_TRANSCEIVE                                       \
    _IOW(MOTO_SPI_DRIVER_MAJOR_NUM, 0x03, MOTO_SPI_TRANSCEIVE_T *)

/*!
 * @brief Locks or unlocks the chip select.
 *
 * Locks the chip select if the data passed in is true.  If the data is false
 * the chip select will be unlocked.  If the chip select is currently locked
 * and the parameter is unlock (false) the chip select will be unlocked before
 * the ioctl call returns.  If the chip select is not locked and the request
 * is to lock the chip select (true) it will not be locked until data is
 * sent using MOTO_SPI_IOCTL_TRANSCEIVE.
 *
 * @return -ENOTCONN if the parameters have yet to be setup.<BR>
 *         -EFAULT if the kernel SPI driver is not able to fulfill the request.<BR>
 *         0 upon success
 */
#define MOTO_SPI_IOCTL_LOCK \
    _IOW(MOTO_SPI_DRIVER_MAJOR_NUM, 0x04, bool)

/*! @brief The list of chip selects which can be accessed from user space. */
typedef enum
{
    /*! @brief The chip select is the trans flash. */
    MOTO_SPI_CS_TFLASH,
    /*! @brief The chip select will not be activated during the transfer. */
    MOTO_SPI_CS_DUMMY,
    /*!
     * @brief The last entry in the enumeration, it does not correspond to any
     *        chip select.  All new values must be placed above this entry.
     */
    MOTO_SPI_CS__END
} MOTO_SPI_CS_T;

/*!
 * @brief Holds the user selectable parameters for SPI transmissions.
 *
 * This contains the subset of the kernel SPI driver parameters which are
 * allowed to be set by user space applications.  This is done for both
 * security reasons and to keep the kernel SPI driver header file private
 * to the kernel.
 */
typedef struct
{
    /*! @brief The chip select which the data will be send out on. */
    MOTO_SPI_CS_T cs;
    /*!
     * @brief The chip select is locked when true; not locked when false.
     * 
     * When true the data which is being transmitted is part of a series of
     * transmits which must be sent out without the chip select being toggled.
     * Once the data starts to transmit no transmits to other chip selects,
     * regardless of priority, will be allowed.  To clear this mode the last
     * call in the chain to spi_transceive or spi_transceivev must have this
     * set to false or a call to spi_release_cs_lock() must be executed.  If
     * this is not done all SPI traffic will be halted and the queue will overflow.
     */
    unsigned int lock_cs:1;
    /*!
     * @brief false when the SPI clock needs to idle low.  true when the clock
     *        idles high.
     */
    unsigned int clk_idle_state:1;

    /*!
     * @brief Selects the phase of the MISO and MOSI lines with respect to the clock.
     *
     * true when the MISO and MOSI lines are in phase with the clock:
     *
     * \verbatim
     * 
     *      --+   +---+   +---+
     * Clk    |   |   |   |   |
     *        +---+   +---+   +-
     * 
     *        +-------+       +-
     * Data   |       |       |
     *      --+       +-------+ 
     *
     * \endverbatim
     *
     * false when they are out of phase by a half a clock cycle.
     *
     * \verbatim
     * 
     *      ------+   +---+   +--
     * Clk        |   |   |   |
     *            +---+   +---+
     *
     *        +-------+       +-
     * Data   |       |       |
     *      --+       +-------+
     *
     * \endverbatim
     *
     */
    unsigned int clk_phase:1;

    /*!
     * @brief Speed in HZ for the transceive.
     *
     * The speed in HZ at which the transfer must be completed.  The speed will be
     * set to closest speed the hardware can achieve, but will never be faster than
     * the rate requested.
     */
    uint32_t speed;
} MOTO_SPI_PARAMETER_T;

/*! @brief Structure holding information about a single vector for a SPI transfer */
typedef struct
{
    /*! @brief Size of the data to transfer in bytes. */
    size_t size;

    /*!
     * @brief Pointer to data to transmit.
     *
     * The pointer to the data to transmit.  The data can be in either user
     * space or kernel space, depending on the value of tx_data_is_kernel_space.
     * If NULL is specified the filler character will be transmitted.  See
     * MOTO_SPI_IOCTL_SET_FILLER for more information on the filler byte.
     */
    void *tx_data;
    /*!
     * @brief Pointer to where received data should be placed
     *
     * Pointer to the location in which the received data will be placed. This
     * can be in either kernel space or user space based on the value
     * of rx_data_is_kernel_space.  If the pointer is NULL, then the received
     * data will be placed in a scratch buffer and discarded.
     */
    void *rx_data;

    /*! @brief 1 if tx_data pointer is a kernel-space pointer, 0 for user space. */
    bool tx_data_is_kernel_space;
    /*! @brief 1 if rx_data pointer is a kernel-space pointer, 0 for user space. */
    bool rx_data_is_kernel_space;
} MOTO_SPI_VECTOR_T;

/*! @brief Structure holding information about the SPI transfer */
typedef struct
{
    /* @brief The number of vectors which are to be transceived. */
    unsigned int count;
    /* @brief The vector pairs to be transceived.  See MOTO_SPI_VECTOR_T. */
    MOTO_SPI_VECTOR_T vectors[MOTO_SPI_MAX_VECTORS];
} MOTO_SPI_TRANSCEIVE_T;

#ifdef __KERNEL__
extern void __init moto_spi_init (void);
extern void __exit moto_spi_destroy (void);
#endif
#endif
