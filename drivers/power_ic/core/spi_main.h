/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/core/spi_main.h
 *
 * Description - Defines the external interface to the spi drivers.
 *
 * Copyright (C) 2005 - Motorola
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Motorola 2005-Jun-16 - File Creation.
 *
 */
#ifndef __SPI_MAIN_H__
#define __SPI_MAIN_H__

#include <linux/types.h>

/*!
 * @ingroup spi_management
 *
 * @file spi_main.h
 *
 * @brief Defines the external interface to the spi drivers.
 *
 * The SPI drivers are used to queue up date to be sent out the SPI over multiple
 * spi interfaces and or chip selects.  This must be the only interface to the SPI
 * hardware which is controlled by the spi_management system.
 */

/*!
 * @brief Maximum number of vectors which can be specified in spi_transceivev().
 *
 * In order to save execution time by not having a linked list of vectors, the code
 * is set up to support a static number of vectors within a single transaction.  This
 * define is used to specify this number.  It should be large enough to support all
 * users, but no larger in order to save memory.
 */
#define SPI_IOVEC_MAX  5

/*!
 * @brief All of the possible SPI ports which data can be sent out on.
 */
typedef enum
{
    SPI_PORT_MAIN, /*!< Send data out in the one and only SPI port. */
    SPI_PORT__END  /*!< Must be the last entry in the enumeration.  */
} SPI_PORT_T;

/*!
 * @brief All of the devices which can send data out the bus.
 *
 * They are listed in terms of decreasing priority with the highest priority device at
 * the start of the enumeration.
 */
typedef enum
{
    SPI_CS_PCAP,   /*!< Send the data to the PCAP. */
    SPI_CS_TFLASH, /*!< Send the data to the trans flash. */
    SPI_CS_DUMMY,  /*!< Send the data out the SPI, but do not activate a chip select.
                        For support of the trans flash init sequence.  The code
                        in pxa_ssp_initialize() relies on this being second to last. */
    SPI_CS__END    /*!< Must be the last entry in the enumeration. */
} SPI_CS_T;

/*!
 * @brief Holds data which is allowed to change between SPI transactions.
 *
 * Includes the following:
 *   - Flag to control blocking when the transmit is in process
 *   - Flags to control the clock polarity and phase
 *   - The chip select to be used for the transmission
 *   - The speed in Hz for the transmission
 *
 * This structure must remain accessable to the driver code after the
 * spi_transceivev() and spi_transceive() functions return, if the no wait flag
 * is set.
 * 
 */
typedef struct
{
    /*! The chip select which the data will be send out on. */
    SPI_CS_T cs:8;
    
    /*!
     * false when the low level code (spi_transceive() or spi_transceivev()) will block.
     * true when requesting a transceive which should not block.  In the non blocking case
     * the user must use spi_check_complete() to get* the data some time in the future.
     */
    unsigned int nonblocking:1;

    /*!
     * When true the data which is being transmitted is part of a series of transmits which
     * must be sent out without the chip select being toggled.  Once the data starts to transmit
     * no transmits to other chip selects, regardless of priority, will be allowed.  To clear
     * this mode the last call in the chain to spi_transceive or spi_transceivev must have this
     * set to false or a call to spi_release_cs_lock() must be executed.  If this is not done all
     * SPI traffic will be halted and the queue will overflow.
     */
    unsigned int lock_cs:1;
    
    /*!
     * false when the SPI clock needs to idle low.  true when the clock
     * idles high.
     */
    unsigned int clk_idle_state:1;

    /*!
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
     * The speed in HZ at which the transfer must be completed.  The speed will be
     * set to cloest speed the hardware can acheive, but will never be faster than
     * the rate requested.
     */
    uint32_t speed;
    /*!
     * Pointer to a kernel function to call when the transmit is completed.  This may
     * be called from an interrupt context, or may be called from the same context from
     * which the transfer was requested.  This will depend on how long it will take to
     * transfer the data.  Quick transfers will wait for the transfer to complete, so they
     * will call this function from the same context.  Transfers which take a while to run
     * will call the function from an interrupt context.  In general it is a good idea to
     * set nonblocking when using this callback.
     */
    void (*xfer_complete_p)(int sequence, void * data_p);
    /*!
     * A pointer which will be supplied to the transfer complete callback (xfer_complete_p)
     * when it is called.
     */
    void *xfer_complete_param_p;
      
} SPI_DATA_PARAMETER_T;

/*!
 * @brief Used as an index into the queue of messages to be sent.
 *
 * It is defined as a typedef in order to allow a simple way to change the size of
 * the queue while still keeping the memory usage low.
 */
typedef uint8_t SPI_QUEUE_INDEX_T;

/*!
 * @brief Structure used to specify start addresses for reads and writes to the spi.
 * 
 * This is used when a user wants to send multiple blocks in the same transmission.
 * This structure is also used for all internal transmits.  This is modeled after
 * readv and writev.  Both the tx_base and rx_base must have valid addresses when
 * calling spi_transceivev().  In this case they can both be the same pointer, if
 * desired.
 */
typedef struct
{
    void *tx_base;   /*!< Base address of the buffer to transmit. */
    void *rx_base;   /*!< Base address of the buffer which will be received. */
    size_t iov_len;  /*!< Length of both the transmit and receive buffers. */
} SPI_IOVEC_T;

/*!
 * @brief Holds a single entry in the queue of SPI messages to be sent.
 *
 * Included in each queue node are
 *   - previous and next indices
 *   - transmit complete flash
 *   - sequence number
 *   - parameter pointer see SPI_DATA_PARAMETER_T for more information
 *   - port number
 *   - number of vectors
 *   - vector array
 *   - low level data pointer 
 */
typedef struct
{
    /*!
     * The queue is implemented as an array with previous and next pointers which are
     * indexes into array.  This is done so that kmalloc does not need to be run
     * for each node added.  It is assumed the array will never be larger than 256
     * entries.
     */
    SPI_QUEUE_INDEX_T next_i;
    SPI_QUEUE_INDEX_T prev_i;
    
    /*!
     * The number of entries in the vector array.  (This is not grouped with the vector
     * array, since it is a uint8, and some non aligned space is available here.)
     */
    uint8_t count;
    
    /*! The port the data must be transmitted or received on. */
    SPI_PORT_T port:7;

    /*! true once the transmit has been completed. */
    unsigned int tx_complete:1;

    /*!
     * Sequence number assigned to all nodes.  This is used when the user does not wish
     * to wait for the transmit to complete before the a transceive or receive completes.
     * In this case the sequence number can be used to fetch the data in the future.
     * (This is an int as opposed to an unsigned int, so that an error value can be
     * returned from functions as well as the sequence number.)
     */
    int sequence;

    /*!
     * Set to 0 for success of transmit or a negative error number if the transmit failed.
     * This will be initialized to a success and changed in the low level hardware driver
     * if an error is encountered.
     */
    int error;

    /*! The user controlled parameters for the transfer. */
    SPI_DATA_PARAMETER_T params;

    /*!
     * An array of vectors which are used to specify the data to be transmitted and
     * received.  This is modeled after readv and writev to allow maximum flexability
     * and keep the number of memory copies to a minimum.
     */
    SPI_IOVEC_T vectors[SPI_IOVEC_MAX];

    /*!
     * For use in the the low level driver if it needs to track any specific data for
     * the mode which is being sent.
     */
    void *low_level_data_p;
} SPI_QUEUE_NODE_T;

/*!
 * @brief Holds the base information for a SPI queue.
 *
 * This includes the head of the queue, a lock for the queue, and information pretaining
 * to a priority chip select if it is set.  See spi_lock_transfer_priority() and
 * spi_restore_transfer_pririty() for more information on priority chip selects.
 */
typedef struct
{
    SPI_QUEUE_INDEX_T head_i;      /*!< The head of the queue is where items are inserted.  The tail is
                                         not tracked, since it will be the previous node to the head. */
    spinlock_t lock;               /*!< Used to protect the head from being changed during inserts and
                                        removals from the queue. */
    SPI_CS_T locked_cs;            /*!< When not equal to SPI_CS__END it contains the chip select ID which
                                        is exclusively being sent out. */
} SPI_QUEUE_T;

extern int spi_transceivev(SPI_DATA_PARAMETER_T *params_p, SPI_IOVEC_T *vectors, size_t count);
extern int spi_transceive(SPI_DATA_PARAMETER_T *params_p, size_t length, void *tx_p, void *rx_p);
extern int spi_check_complete(SPI_DATA_PARAMETER_T *params_p, int sequence);
extern SPI_QUEUE_NODE_T *spi_tx_completed(SPI_QUEUE_NODE_T *node_p);
extern int spi_release_cs_lock(SPI_DATA_PARAMETER_T *params_p);
extern void spi_initialize(void);

#endif /* __SPI_MAIN_H__ */
