/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/core/spi_main.c
 *
 * Description - This is the main file for SPI data flow management.
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
 * Motorola 2005-Jun-16 - Rewrote most functions from previous
 *                        Motorola File located at: 
 *                        /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/misc
 * Motorola 2005-Sep-13 - Minor changes to finalize software
 *
 */

/*!
 * @defgroup spi_management SPI data flow management
 */

/*!
 * @file spi_main.c
 *
 * @ingroup spi_management
 *
 * @brief This is the main file for SPI data flow management.
 *
 * <B>Overview:</B><BR>
 *   Provides the external interface by which data can be sent out one of the
 *   SPI buses.  This module implements the queues which are used for transferring
 *   data and is hardware independent.  It requires the external interfaces defined
 *   in below under "Low Level Driver Requirements"  to be provided externally.
 *
 *   The system works by first placing the requested data into the transfer (tx) queue.
 *   A request to transfer is then made of the low level driver.  It will start the
 *   transmission if a previous transmission is not already running.  Once it detects
 *   the completion of a transmission it will call transfer complete function
 *   (spi_tx_completed()) within this module.  This will move the data from the
 *   transmit queue to the received queue (rx) and handle waking up the transceive
 *   routine if it was blocking.
 *
 *   If the user does not wish to wait for the transfer to complete, they must set
 *   the nonblocking flag in the parameter structure.  In this case, the user must poll
 *   for the transfer to complete using spi_check_complete() or the received queue
 *   will fill up.
 *
 *   In all cases the user provided buffer will get the received data copied into it.
 *   This will be the case even if the user does not wish to receive the data.  In this
 *   case, the user still must block on the completion or poll for the completion with
 *   spi_check_complete().  The receive and transmit buffers must be DMA accessible for
 *   systems which support DMA at the low level.  See the low lever driver for details.
 *
 * <B>Low Level Driver Requirements:</B><BR>
 *   The low level driver module must be able to send the current message and then
 *   call spi_tx_completed() when the transmit was completed.  Then based upon the
 *   return value the next transmission will be started or the transmit shut down.
 *   A new transmit must be started if the return value is non NULL.
 *
 *   A pointer is provided in the queue for use by the low level driver if necessary.
 *   See low_level_data_p in SPI_QUEUE_NODE_T for more details.
 */

#include <linux/config.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/limits.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spi_user.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <stdbool.h>
#include "os_independent.h"
#include "spi_main.h"
#include "pxa_ssp.h"


#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT

#include <asm/arch/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <linux/power_ic.h>
#include <linux/delay.h>

extern int spi_int_set_reg_mask(POWER_IC_REGISTER_T reg, int mask, int value);
extern int spi_int_transceive(u32 *tx_p, u32 *rx_p);
extern void spi_int_stop_atod_converter(void);
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */


/******************************************************************************
* Local constants and macros
******************************************************************************/

/*!
 * @brief The number of queue entries which should be defined when the system
 * initializes.
 *
 * This should be as close to the number of nodes which will be used at run time.
 * The list will resize if more are needed, but this will take execution time.
 */
#define SPI_NUM_INIT_QUEUE_ENTRIES 10

/*!
 * @brief The maximum number of queue entries allowed in the transaction queue.
 *
 * Once this is reached the queue will not be expanded in size.  This should never
 * happen during normal running.  It is setup to find potential problems during
 * development.
 */
#define SPI_MAX_NUM_QUEUE_ENTRIES (SPI_NUM_INIT_QUEUE_ENTRIES*10)

/*!
 * @brief Value used for the index of unused nodes in the queue or for the head if no
 * entries are in the queue.
 */
#define SPI_QUEUE_NODE_UNUSED   ((SPI_QUEUE_INDEX_T)(-1))

/*!
 * @brief Used to reserve space in the queue for a new node which has not yet been placed
 * into one of the queues.
 */
#define SPI_QUEUE_NODE_RESERVED (SPI_QUEUE_NODE_UNUSED-1)

/*!
 * @brief Macro to return the SPI port on which a specific chip select is assigned to.
 */
#define spi_cs_to_port(cs)  ((cs) < SPI_CS__END) ? cs_to_port[(cs)]: SPI_PORT__END;

#if (defined POWER_IC_SPI_MAIN_DEBUG_LEVEL) && (POWER_IC_SPI_MAIN_DEBUG_LEVEL > 1)
# define DEBUG_MSG_LEVEL2 tracemsg
#else
# define DEBUG_MSG_LEVEL2(fmt, args...)
#endif
#if (defined POWER_IC_SPI_MAIN_DEBUG_LEVEL)
# define DEBUG_MSG_LEVEL1 tracemsg
#else
# define DEBUG_MSG_LEVEL1(fmt, args...)
#endif

/******************************************************************************
* Local variables
******************************************************************************/
/*!
 * @brief Holds the function pointers for the low level driver.
 *
 * Each low level driver is required to provide the interface functions in
 * this structure.  A pointer to the functions will be placed in #low_level_drv_tb
 * for use by this driver.
 *
 * @note It is possible to have different low level drivers for each queue.
 */
static const struct
{
    /*!
     * Called to initialize the low level driver.
     */
    void (*spi_initialize)(void);
    /*!
     * Starts a transmit if none is currently active and returns.  It must not
     * not block to wait for the transmit to complete.  It may delay for a short
     * period of time while waiting for short transmits to complete.
     */
    int (*spi_start_tx)(SPI_QUEUE_NODE_T *node_p);
    /*!
     * Called to determine if a transmit is currently in progress.  If one is not
     * and a new node has been added to the queue spi_start_tx will be called to
     * initiate a transmit.  It must return true when a transmit is in progress and
     * false the low level driver is idle.
     */
    bool (*spi_tx_in_progress)(void);
} low_level_drv_tb[SPI_PORT__END] =
{
    /*! Define the functions for use with the main SPI port (SSP on Bulverde). */
    {
        pxa_ssp_initialize,
        pxa_ssp_start_tx,
        pxa_ssp_tx_in_progress
    }
};

/*!
 * @brief Table used to translate between chip select and port.
 *
 * Currently only one port is available for use on Bulverde.
 */
static const SPI_PORT_T cs_to_port[SPI_CS__END] =
{
    SPI_PORT_MAIN,  /* SPI_CS_PCAP   */
    SPI_PORT_MAIN   /* SPI_CS_TFLASH */
};

/*!
 * @brief The SPI transfer queue contains new items which have not yet been
 * sent out the SPI bus.
 *
 * They remain in the queue until the transfer has been completed.  Once
 * they have been transfered they are placed in the received queue.
 */
static SPI_QUEUE_T spi_tx_queue[SPI_PORT__END];

/*!
 * @brief The SPI receive queue contains the data which was received during a transmit.
 *
 * A node is only moved from the received queue after the data received has been placed
 * in the data portion of the node, and the transfer has been completed.
 */
static SPI_QUEUE_T spi_rx_queue[SPI_PORT__END];

#ifndef DOXYGEN_SHOULD_SKIP_THIS 
/*!
 * @brief Wait queue used to wait for a transfer to complete.
 *
 * Only one wait queue is used for for all transmissions, so it is necessary to check
 * to be sure the the transmit which was requested is the one which completed.
 */
DECLARE_WAIT_QUEUE_HEAD(rx_complete_wait_queue);
#endif

/*!
 * @brief Memory array which holds the queue entries.
 *
 * Elements in this array can be assigned to the Tx or Rx queues.
 */
static SPI_QUEUE_NODE_T *queue_data_p = NULL;

/*!
 * @brief The number of entries in the queue_data_p array.
 *
 * This is tracked to simplify growing the queue.
 */
static SPI_QUEUE_INDEX_T num_queue_entries;

/*!
 * @brief Used to protect writes to the data array which is used for the SPI queue.
 *
 * This is only used when new nodes are created or deleted.
 */
static spinlock_t queue_lock = SPIN_LOCK_UNLOCKED;

/******************************************************************************
* Local functions
******************************************************************************/

/*!
 * @brief Inserts an existing node into a SPI transaction queue.
 *
 * Places the node in the queue based on the prioity of the device to which
 * the data will be sent.  This is done based on the entry port in the params_p
 * entry in the node.  The smaller the port the higher priority the data is
 * considered.
 *
 * @param     queue_p  Pointer to either the transmit or receive queue.
 * @param     node_i   Index of an existing node to insert onto the queue.
 *
 * @return    0 upon success<BR>
 *            -EINVAL if a bad arguement is used.<BR>
 */
static int spi_queue_node_insert(SPI_QUEUE_T *queue_p, SPI_QUEUE_INDEX_T node_i)
{
    SPI_QUEUE_INDEX_T i;

    DEBUG_MSG_LEVEL2(_k_d("spi_queue_node_insert node = %d"), node_i);
    if ((queue_p == NULL) || (node_i >= num_queue_entries))
    {
        return (-EINVAL);
    }
    
    spin_lock(queue_p->lock);
    i = queue_p->head_i;
    if (i == SPI_QUEUE_NODE_UNUSED)
    {
        queue_p->head_i = node_i;
        queue_data_p[node_i].next_i = node_i;
        queue_data_p[node_i].prev_i = node_i;
    }
    else
    {
        /*
         * Find the correct location to insert the node, based on priority.
         */
        do
        {
            if (queue_data_p[node_i].params.cs < queue_data_p[i].params.cs)
            {
                break;
            }
            i = queue_data_p[i].next_i;
        }
        while (i != queue_p->head_i);
        /*
         * Add the node to the list in the correct location.
         */
        i = queue_data_p[i].prev_i;
        queue_data_p[node_i].prev_i = i;
        queue_data_p[node_i].next_i = queue_data_p[i].next_i;
        queue_data_p[queue_data_p[i].next_i].prev_i = node_i;
        queue_data_p[i].next_i = node_i;
        /*
         * If inserting in place of the head node, reset the head to the
         * node with the highest priority.
         */
        if ((i == queue_p->head_i) && (queue_data_p[node_i].params.cs < queue_data_p[i].params.cs))
        {
            queue_p->head_i = node_i;
        }
    }
    spin_unlock(queue_p->lock);
    return 0;
}

/*!
 * @brief Creates a new node for a SPI transaction queue.
 *
 * Creates a new node by searching the array of pre allocated nodes and
 * finding a node which is currently not used.  If no nodes are available the
 * array is resized and a node is assigned from the new array elements. The
 * function is safe to be called from an interrupt or bottom half context.
 *
 * @param     spi_index_p Pointer to the index to the spi node which will be created.  
 * @param     params_p    Contains the SPI bus specific parameters for the transaction. These
 *                        include items such as the chip select and the transfer speed.
 * @param     vectors     Array of blocks to be transmitted and received.
 * @param     count       The number of entries in the vectors array.
 * 
 * @return    This function returns 0 if successful.<BR>
 *            -EINVAL - Bad data contained in params_p.<BR>
 *            -ENOMEM - Unable to allocate memory for the new node.<BR>
 */
static int spi_queue_node_create
(
   SPI_QUEUE_INDEX_T *spi_index_p,
   SPI_DATA_PARAMETER_T *params_p,
   SPI_IOVEC_T *vectors,
   size_t count
)
{
    static int sequence = 0;
    static spinlock_t sequence_lock = SPIN_LOCK_UNLOCKED;
    SPI_PORT_T port;
    SPI_QUEUE_INDEX_T i;
    SPI_QUEUE_NODE_T *new_queue_p;
    
    *spi_index_p = SPI_QUEUE_NODE_UNUSED;

    port = spi_cs_to_port(params_p->cs);
    /*
     * Range check the necessary elements and flag an error if out of range.
     * (The port check will check the chip select as well, since spi_cs_to_port
     * will use SPI_PORT__END if invalid cs numbers.)
     */
    if ((port >= SPI_PORT__END) || (vectors == NULL) || (count > SPI_IOVEC_MAX))
    {
        return -EINVAL;
    }

    /*
     * Find an available location for the new data.
     */
    spin_lock(queue_lock);
    for (i = 0; i < num_queue_entries; i++)
    {
        if (queue_data_p[i].next_i == SPI_QUEUE_NODE_UNUSED)
        {
            queue_data_p[i].next_i = SPI_QUEUE_NODE_RESERVED;
            break;
        }
    }
    spin_unlock(queue_lock);
    if (i >= num_queue_entries)
    {
        /*
         * No free element was found.  Allocate a larger buffer from which to select
         * nodes.
         */
        if (num_queue_entries >= SPI_MAX_NUM_QUEUE_ENTRIES)
        {
            tracemsg(_k_w("spi_queue_node_create: max number of queue entries reached."));
            return -ENOMEM;
        }
        /* At this point i is set to num_queue_entries from the loop above. */
        num_queue_entries += SPI_NUM_INIT_QUEUE_ENTRIES;
        /* Use GFP_ATOMIC in case this is called from an interrupt context. */
        new_queue_p = (SPI_QUEUE_NODE_T *)kmalloc(sizeof(SPI_QUEUE_NODE_T)*num_queue_entries, GFP_ATOMIC);
        if (new_queue_p == NULL)
        {
            tracemsg(_k_w("spi_queue_node_create: out of memory."));
            return -ENOMEM;
        }
        /* Copy the old queue into the new queue. */
        memcpy(new_queue_p, queue_data_p, sizeof(SPI_QUEUE_NODE_T)*i);
        kfree(queue_data_p);
        queue_data_p = new_queue_p;
        DEBUG_MSG_LEVEL2(_k_d("spi_queue_node_create: expanded queue size from %d entries to %d entries."), i, num_queue_entries);
    }
    /* Copy the existing data into the new node. */
    memcpy(&queue_data_p[i].params, params_p, sizeof(queue_data_p[i].params));
    queue_data_p[i].tx_complete = false;
    queue_data_p[i].port = port;
    queue_data_p[i].low_level_data_p = NULL;
    /* The vectors are copied from the user supplied data in case they were on the stack.  In this
       case they may not be valid by the time they need to be used. */
    memcpy(queue_data_p[i].vectors, vectors, sizeof(SPI_IOVEC_T)*count);
    queue_data_p[i].count = count;
    queue_data_p[i].error = 0;
    
    /*
     * Assign the next available sequence number to the node.  No check is made for
     * existing sequence numbers in the queue, since the transfer will in general happen
     * in order and none will ever be queue enough to end up with a duplicate.
     */
    spin_lock(sequence_lock);
    queue_data_p[i].sequence = sequence++;
    if (sequence < 0)
    {
        sequence = 0;
    }
    spin_unlock(sequence_lock);
    *spi_index_p = i;
    return 0;
}

/*!
 * @brief Removes the node passed in from the queue.
 *
 * Removes the node passed in from the queue passed in.  The node is not free to
 * be reused until spi_queue_node_destroy() is called.
 *
 * @param  queue_p Pointer to either the transmit or receive queue
 * @param  node_i  Index to the node to remove.
 *
 * @return 0 upon success<BR>
 *         -EINVAL - Null pointer in queue_p or an invalid index in node_i.<BR>
 */
static int spi_queue_node_remove(SPI_QUEUE_T *queue_p, SPI_QUEUE_INDEX_T node_i)
{
    SPI_QUEUE_INDEX_T next_i;

    DEBUG_MSG_LEVEL2(_k_d("spi_queue_node_remove node = %d"), node_i);
    if ((queue_p == NULL) || (node_i >= num_queue_entries))
    {
        return -EINVAL;
    }
    spin_lock(queue_p->lock);
    if (node_i == queue_p->head_i)
    {
        /* Check and see if this is the only item in the list. */
        if (node_i == queue_data_p[node_i].next_i)
        {
            queue_p->head_i = SPI_QUEUE_NODE_UNUSED;
        }
        else
        {
            next_i = queue_data_p[node_i].next_i;
            queue_p->head_i = next_i;
            queue_data_p[next_i].prev_i = queue_data_p[node_i].prev_i;
            queue_data_p[queue_data_p[node_i].prev_i].next_i = next_i;
        }
    }
    else
    {
        next_i = queue_data_p[node_i].next_i;
        queue_data_p[next_i].prev_i = queue_data_p[node_i].prev_i;
        queue_data_p[queue_data_p[node_i].prev_i].next_i = next_i;
    }
    spin_unlock(queue_p->lock);
    return 0;
}

/*!
 * @brief Destroys the SPI transaction node passed in.
 *
 * @param   node_i Index of the node which will be released for reuse.
 */
static void spi_queue_node_destroy(SPI_QUEUE_INDEX_T node_i)
{
    DEBUG_MSG_LEVEL2(_k_d("spi_queue_node_destroy node = %d"), node_i);
    spin_lock(queue_lock);
    queue_data_p[node_i].next_i = SPI_QUEUE_NODE_UNUSED;
    spin_unlock(queue_lock);
}

/*!
 * @brief Find a transmit queue node with the sequence number passed in.
 *
 * @param  queue_p  Pointer to either the transmit or receive queue
 * @param  sequence The sequence number to locate in the queue.
 *
 * @return An index to the node with the sequence number or
 *         SPI_QUEUE_NODE_UNUSED if it is not found.
 */
static SPI_QUEUE_INDEX_T spi_queue_find_sequence_node(SPI_QUEUE_T *queue_p, int sequence)
{
    SPI_QUEUE_INDEX_T i;
    SPI_QUEUE_INDEX_T end_i;

    DEBUG_MSG_LEVEL2(_k_d("spi_queue_find_sequence_node sequence = %d"), sequence);
    if ((queue_p == NULL) || (queue_p->head_i == SPI_QUEUE_NODE_UNUSED))
    {
        return SPI_QUEUE_NODE_UNUSED;
    }

    /*
     * Need to protect the search from a new node being entered on the list
     * during the search.
     */
    spin_lock(queue_p->lock);
    
    /*
     * Search the list from the head, since this is called out to spi_tx_complete
     * which may be running in an interrupt.  In this case It will be looking for
     * the last node sent which will be toward the head of the list.
     */
    i = queue_p->head_i;
    end_i = i;
    do
    {
        if (queue_data_p[i].sequence == sequence)
        {
            spin_unlock(queue_p->lock);
            return i;
        }
        i = queue_data_p[i].next_i;
    } while (i != end_i);
    spin_unlock(queue_p->lock);
    return SPI_QUEUE_NODE_UNUSED;
}

/*!
 * @brief Finds the node which needs to be transfered next.
 *
 * When the chip select is locked the oldest node with the locked chip
 * select is returned.  If the chip select is not locked then the head of
 * the queue is returned.
 *
 * @param    port The qspi port for which the search must be done.
 *
 * @return   A pointer to the node to transmit or NULL if none.
 *
 * @note No range checking is done on port. It is assumed this will be done
 *       by the caller.
 */
static SPI_QUEUE_NODE_T *spi_queue_find_next_tx(SPI_PORT_T port)
{
    SPI_QUEUE_INDEX_T i;
    SPI_QUEUE_INDEX_T end_i;
    SPI_CS_T locked_cs;


    DEBUG_MSG_LEVEL2(_k_d("spi_queue_find_next_tx"));
    /* Do nothing if the queue is empty. */
    if (spi_tx_queue[port].head_i != SPI_QUEUE_NODE_UNUSED)
    {
        /*
         * If no lock is set up, then simply return the head of the queue.
         */
        locked_cs = spi_tx_queue[port].locked_cs;
        if (locked_cs == SPI_CS__END)
        {
            DEBUG_MSG_LEVEL2(_k_d("spi_queue_find_next_tx - Returning head of list %d (lock - %d)."),
                             spi_tx_queue[port].head_i, locked_cs);
            return &queue_data_p[spi_tx_queue[port].head_i];
        }
        else
        {
            DEBUG_MSG_LEVEL2(_k_d("spi_queue_find_next_tx - Locked searching for the next node with the lock CS (%d)."),
                             locked_cs);
            /*
             * Find the oldest node with the chip select which is locked.
             */
            spin_lock(spi_tx_queue[port].lock);
            i = spi_tx_queue[port].head_i;
            end_i = i;
            do
            {
                if (queue_data_p[i].params.cs == locked_cs)
                {
                    spin_unlock(spi_tx_queue[port].lock);
                    return &queue_data_p[i];
                }
                i = queue_data_p[i].next_i;
            }
            while (i != end_i);
            spin_unlock(spi_tx_queue[port].lock);
        }
    }
    return NULL;
}

/*
 * @brief Sets up the lock status based on the node about to be transmitted
 *
 * Sets the queues lock chip select based on the node which is passed in.  Once
 * a lock is set up only nodes for the locked chip select can be sent.  The selection
 * of these nodes is done in spi_queue_find_next_tx().
 *
 * @param node_p Pointer to the node which will be transmitted next.
 *
 * @return Pointer to the node which will be transmitted next (node_p).
 */
static SPI_QUEUE_NODE_T *spi_setup_next_tx(SPI_QUEUE_NODE_T *node_p)
{
    SPI_PORT_T port;
    
    DEBUG_MSG_LEVEL2(_k_d("spi_setup_next_tx"));
    if (node_p != NULL)
    {
        DEBUG_MSG_LEVEL2(_k_d("spi_setup_next_tx: Sequence %d"), node_p->sequence);
        port = node_p->port;
        /* Set the chip select lock if the node is part of a chain of nodes. */
        if (node_p->params.lock_cs)
        {
            spi_tx_queue[port].locked_cs = node_p->params.cs;
        }
        else
        {
            spi_tx_queue[port].locked_cs = SPI_CS__END;
        }
    }
    return node_p;
}

/******************************************************************************
* Global functions
******************************************************************************/
/*!
 * @brief Transfers the tail of the transmit queue to the head of the receive queue.
 *
 * This function must be called by the low level driver when a transfer of a node
 * is completed.  It will transfer the head node in the transfer queue to the
 * received queue.  It then wakes up the process which is waiting for the data
 * to be received.  If the node was a null node (used for unlocking a chip select),
 * the node is deleted from the transmit queue, since no result will be expected.
 *
 * @param     node_p - Pointer to the transmit node for which the data was just
 *            sent.
 *
 * @return    A pointer to the next node to transmit or NULL if no more data
 *            is in the transmit queue.
 */
SPI_QUEUE_NODE_T *spi_tx_completed(SPI_QUEUE_NODE_T *node_p)
{
    SPI_QUEUE_INDEX_T i;
    SPI_PORT_T port;

    DEBUG_MSG_LEVEL2(_k_d("spi_tx_completed: sequence %d node_p %p"), node_p->sequence, node_p);
    port = node_p->port;
    /*
     * Move the node from the transfer to the received queue.
     */
    i = spi_queue_find_sequence_node(&spi_tx_queue[port], node_p->sequence);
    if (i != SPI_QUEUE_NODE_UNUSED)
    {
        (void)spi_queue_node_remove(&spi_tx_queue[port], i);
        /*
         * If this was an empty node, it was used to unlock the chip select.  As a result
         * no data needs to be returned.  Simply destroy the node to free the memory.
         */
        if (queue_data_p[i].count == 0)
        {
            spi_queue_node_destroy(i); 
        }
        else
        {
            spi_queue_node_insert(&spi_rx_queue[port], i);

            /* Indicate the transmit has been completed. */
            queue_data_p[i].tx_complete = true;

            /* Call the transfer complete function, if one was supplied. */
            if (node_p->params.xfer_complete_p)
            {
                (*node_p->params.xfer_complete_p)(node_p->sequence, node_p->params.xfer_complete_param_p);
            }
            /* If the blocking until the data is transfered, wake up the user thread. */
            if (!node_p->params.nonblocking)
            {
                /* Wake up the process which requested the data be sent. */
                wake_up(&rx_complete_wait_queue);
            }
        }
        return spi_setup_next_tx(spi_queue_find_next_tx(port));
    }
    return NULL;
}

/*!
 * @brief Send and receive multiple blocks (vectors) of data over the SPI
 *
 * Sends and receives data over the SPI.  The data passed in is placed in a queue and
 * will be transmitted as soon as possible.  The function may block until all of
 * the data requested has been received.  It will block if the nonblocking parameter in
 * params_p is false.  If it is true the function will not block and the data received
 * will need to be retrieved using spi_check_complete().
 *
 * IMPORTANT:
 *   -# None of the data pointers can be NULL.  The low level drivers all assume that the
 *      same amount of data which will be transmitted will be received.  As a result they
 *      need to have a location to place the receive data.  On the same note, they must
 *      transmit something to receive something, so the transmit pointers must be valid as well.
 *   -# If the nonblocking flag is set within params_p then the data pointed to by params_p must
 *      remain valid until spi_check_complete() indicates the transmission is complete.
 *
 * @param     params_p  Contains the SPI bus specific parameters for the transaction. These
 *                      include items such as the chip select and the transfer speed.
 * @param     vectors   Pointer to an array of vectors (blocks) of data to send over the
 *                      spi.  The tx and rx pointers cannot be NULL, they can point to the
 *                      same memory location or different locations.  In both cases they
 *                      must be DMA accessible.
 * @param     count     The number of vectors (blocks) in the vectors array.
 *
 * @return    A sequence number upon success (will be 0 or greater).  The sequence number
 *            is only valid if a no wait transmit was requested.<BR>
 *            -EINVAL  - Null pointer passed in or bad data contained in params_p.<BR>
 *            -ENOMEM  - Unable to allocate memory for the new node.<BR>
 */
int spi_transceivev(SPI_DATA_PARAMETER_T *params_p, SPI_IOVEC_T *vectors, size_t count)
{
    SPI_QUEUE_INDEX_T node_i;
    SPI_QUEUE_NODE_T *node_p;
    SPI_PORT_T port;
    int sequence;
    size_t i;
    int error;

    DEBUG_MSG_LEVEL1(_k_d("spi_transceivev"));
    if ((params_p == NULL) || (vectors == NULL))
    {
        tracemsg(_k_w("spi_transceivev: Parameter pointer or vectors are null."));
        return -EINVAL;
    }

    /* Return an error if any of the user specified pointers are null. */
    for (i = 0; i < count; i++)
    {
        if ((vectors[i].rx_base == NULL) || (vectors[i].tx_base == NULL))
        {
            tracemsg(_k_w("spi_transceivev: Vector base address for tx or rx null at vector %d."), i);
            return -EINVAL;
        }
    }
    /* Get the port number for the transaction and do not continue if it is invalid. */
    port = spi_cs_to_port(params_p->cs);
    if (port >= SPI_PORT__END)
    {
        tracemsg(_k_w("spi_transceivev: Invalid port number %d."), port);
        return -EINVAL;
    }
    
    /* Place the data to be sent and received into the queue. */
    error = spi_queue_node_create(&node_i, params_p, vectors, count);
    if (error != 0)
    {
        tracemsg(_k_w("spi_transceivev: Unable to create queue node."));
        return error;
    }
    
    error = spi_queue_node_insert(&spi_tx_queue[port], node_i);
    if (error != 0)
    {
        tracemsg(_k_w("spi_transceivev: Unable to insert queue node."));
        return error;
    }

    /*
     * Store the sequence number, in case the node is destroyed by the callback before
     * the start tx function returns.  This can only happen when the nonblock parameter
     * is true.
     */
    sequence = queue_data_p[node_i].sequence;
    
    /*
     * Start the transmit if nothing is currently being transmitted by the low level driver. 
     * No need to validate the range of port since it was done in spi_queue_node_create.
     * The transmit will be started when the last higher priority one is completed or
     * when spi_restore_transfer_priority is called.
     */
    if (!(low_level_drv_tb[port].spi_tx_in_progress()))
    {
        node_p = spi_setup_next_tx(spi_queue_find_next_tx(port));
        error = 1;  /* In case no node needs to be sent due to a lock, force blocking. */
        if (node_p != NULL)
        {
            error = low_level_drv_tb[port].spi_start_tx(node_p);
        }
    }
    /*
     * If the user requested non-blocking and an error is not encountered
     * return the sequence number, even if the transfer is already completed.
     * This is done to keep the users of this function from having to check for
     * the case of the transfer already completed.  This way they can always
     * use spi_check_complete().
     */
    if ((error >= 0) && (params_p->nonblocking))
    {
        DEBUG_MSG_LEVEL1(_k_d("spi_transceivev: Returning sequence number: %d"), sequence);
        return sequence;
    }

    /*
     * If the return value indicates the user must wait for the transmit
     * wait for the transmit to complete.
     */
    if (error == 1)
    {
        /*
         * This does not use interruptable, since SPI transfers will complete quickly
         * and if interrupted orphan data will be left in the received queue if the
         * user does not call spi_check_complete().
         */
        wait_event(rx_complete_wait_queue, (queue_data_p[node_i].tx_complete != false));
        error = queue_data_p[node_i].error;
        DEBUG_MSG_LEVEL1(_k_d("spi_transceivev: Done waiting error is %d"), error);
    }
    
    /* Remove the node from the receive queue. */
    (void)spi_queue_node_remove(&spi_rx_queue[port], node_i);
    spi_queue_node_destroy(node_i);
    return error;
}

/*!
 * @brief Send and receive data over the SPI
 *
 * Sends and receives data over the SPI.  The data passed in is placed in a queue and
 * will be transmitted as soon as possible.  The function may block until all of
 * the data requested has been received.  It will block if the nonblocking parameter in
 * params_p is false.  It is is true the function will not block and the data received
 * will need to be retrieved using spi_fetch_recieve.
 *
 * IMPORTANT:
 *   None of the data pointers can be NULL.  The low level drivers all assume that the
 *   same amount of data which will be transmitted will be received.  As a result they
 *   need to have a location to place the receive data.  On the same note, they must
 *   transmit something to receive something, so the transmit pointers must be valid as well.
 *
 * @param     params_p Contains the SPI bus specific parameters for the transaction. These
 *                     include items such as the chip select and the transfer speed.
 * @param     length   The number of bytes which need to be transfered.
 * @param     tx_p     Pointer to the data to be transmitted (must be DMA accessible).  This
 *                     pointer cannot be NULL or an error will be returned.
 * @param     rx_p     Pointer to the data to be received (must be DMA accessible).  This
 *                     pointer cannot be NULL or an error will be returned.
 *
 * @return    A sequence number upon success (will be 0 or greater).  The sequence number
 *            is only valid if a no wait transmit was requested. <BR>
 *            -ENODATA - Internal error.<BR>
 *            -EINVAL  - Null pointer passed in or bad data contained in params_p.<BR>
 *            -ENOMEM  - Unable to allocate memory for the new node.<BR>
 */
int spi_transceive(SPI_DATA_PARAMETER_T *params_p, size_t length, void *tx_p, void *rx_p)
{
    SPI_IOVEC_T spi_trans_vectors;

    spi_trans_vectors.tx_base = tx_p;
    spi_trans_vectors.rx_base = rx_p;
    spi_trans_vectors.iov_len = length;
    return spi_transceivev(params_p, &spi_trans_vectors, 1);
}

/*!
 * @brief Check to see if the transmit and receive are completed.
 *
 * Searches the received queue for the sequence number which is provided.  If
 * it is found the user is informed and the node is destroyed.  The user must
 * call this if they have set up the nonblocking parameter of the receive queue will
 * fill up.  No data is copied, since the user specified a receive pointer to
 * which the data will already be copied.
 *
 * @param   params_p  Contains the SPI bus specific parameters for the transaction. These
 *                    include items such as the chip select and the transfer speed.
 * @param   sequence  The sequence number from the receive or transceive for
 *                    which the data must be received.
 *
 * @return  0 upon successfully finding the node and copying data to data_p.<BR>
 *          1 if the transcation is still in progress (node is in the transmit queue).<BR>
 *          -ENODATA - Internal error.<BR>
 *          -EINVAL  - Bad data contained in params_p or the node is not in either
 *                     queue.<BR>
 */
int spi_check_complete(SPI_DATA_PARAMETER_T *params_p, int sequence)
{
    SPI_QUEUE_INDEX_T node_i;
    SPI_PORT_T port;

    DEBUG_MSG_LEVEL1(_k_d("spi_check_complete = %d"), sequence);
    if (params_p == NULL)
    {
        return -EINVAL;
    }
    
    /* Get the port number for the transaction and do not continue if it is invalid. */
    port = spi_cs_to_port(params_p->cs);
    if (port >= SPI_PORT__END)
    {
        return -EINVAL;
    }
    
    node_i = spi_queue_find_sequence_node(&spi_rx_queue[port], sequence);
    if (node_i == SPI_QUEUE_NODE_UNUSED)
    {
        /* Return an error if the node is not in the transmit queue. */
        if (spi_queue_find_sequence_node(&spi_tx_queue[port], sequence) == SPI_QUEUE_NODE_UNUSED)
        {
            return -EINVAL;
        }
        return 1;
    }
    if (spi_queue_node_remove(&spi_rx_queue[port], node_i) != 0)
    {
        return -ENODATA;
    }
    spi_queue_node_destroy(node_i);
    return 0;
}

/*!
 * @brief Releases a chip select lock
 *
 * Releases a chip select lock if it is currently active.  The lock is released by
 * placing a node with no data within the transmit queue.  When the node is executed
 * no data will be sent, but the chip select lock will be released, and the chip select
 * will be disabled.
 *
 * @param   params_p  Contains the SPI bus specific parameters for the transaction. These
 *                    include items such as the chip select and the transfer speed.
 *
 * @return 0 upon success
 */
int spi_release_cs_lock(SPI_DATA_PARAMETER_T *params_p)
{
    SPI_PORT_T port;
    static const SPI_IOVEC_T spi_null_trans_vectors =
        {
            .tx_base = NULL,
            .rx_base = NULL,
            .iov_len = 0
        };

    if (params_p == NULL)
    {
        return -EINVAL;
    }

    /* Get the port number for the transaction and do not continue if it is invalid. */
    port = spi_cs_to_port(params_p->cs);
    if (port >= SPI_PORT__END)
    {
        return -EINVAL;
    }
    
    if (spi_tx_queue[port].locked_cs != SPI_CS__END)
    {
        return spi_transceivev(params_p, (SPI_IOVEC_T *)&spi_null_trans_vectors, 0);
    }
    return 0;
}
#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
void __init spi_int_memory_pool_init(void);
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */

/*!
 * @brief Initialize the SPI queues at power up.
 */
void __init spi_initialize(void)
{
    unsigned int i;

    DEBUG_MSG_LEVEL1(_k_d("spi_initialize"));

#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
    spi_int_memory_pool_init();
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */

    for (i = 0; i < SPI_PORT__END; i++)
    {
        spi_tx_queue[i].head_i = SPI_QUEUE_NODE_UNUSED;
        spi_tx_queue[i].locked_cs = SPI_CS__END;
        spi_rx_queue[i].head_i = SPI_QUEUE_NODE_UNUSED;
        spi_rx_queue[i].locked_cs = SPI_CS__END;
        low_level_drv_tb[i].spi_initialize();
    }
    num_queue_entries = SPI_NUM_INIT_QUEUE_ENTRIES;
    queue_data_p = (SPI_QUEUE_NODE_T *)kmalloc(sizeof(SPI_QUEUE_NODE_T)*SPI_NUM_INIT_QUEUE_ENTRIES, GFP_KERNEL);
    for (i = 0; i < SPI_NUM_INIT_QUEUE_ENTRIES; i++)
    {
        queue_data_p[i].next_i = SPI_QUEUE_NODE_UNUSED;
    }
    /* Initialize the user space interface. */
    moto_spi_init();
}

#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
/*----------------------New SPI Driver begin---------------------------*/
/*Low is for a read and high is for a write.*/
#define PCAP_RW_MSK  0x80000000

/*Used to mask off the dead bit for the PCAP.*/
#define PCAP_DEAD_MSK 0x02000000

/*Used to mask off the register number for the PCAP.*/
#define PCAP_REG_MSK 0x7C000000

/*The bit position for the start of the PCAP register number.*/
#define PCAP_REG_SFT 26


/*
 * spi lock for SPI bus
 */
spinlock_t spi_bus_int_lock = SPIN_LOCK_UNLOCKED;

/*!
 * @brief Read PCAP register value.
 *
 * In order to process SPI bus in interrupt context, we need to call
 * spi_int_transceive function
 *
 * @param     reg        The PCAP register index.
 * @param     value_ptr  The receive data
 *
 * @return    0 upon success<BR>
 */
int spi_int_reg_read (int reg, u32 *value_ptr)
{
        int error;

        if (reg > POWER_IC_REG_PCAP_LAST_REG-POWER_IC_REG_PCAP_FIRST_REG)
        {
                return -EINVAL;
        }
        /* Set up the address and the read write bit to a 0. */
        *value_ptr = (reg<<PCAP_REG_SFT)&PCAP_REG_MSK;

        /* Put the bit in the correct order for the transfer to the pcap. */
        /* *value_ptr = __cpu_to_be32(*value_ptr); */
        error = spi_int_transceive(value_ptr, value_ptr);
        /* *value_ptr = __be32_to_cpu(*value_ptr); */
        return error;
}

/*!
 * @brief Write PCAP register.
 *
 * In order to process SPI bus in interrupt context, we need to call
 * spi_int_transceive function
 *
 * @param     reg        The PCAP register index.
 * @param     value      The receive data
 *
 * @return    0 upon success<BR>
 */
int spi_int_reg_write (int reg, u32 value)
{
        if (reg > POWER_IC_REG_PCAP_LAST_REG-POWER_IC_REG_PCAP_FIRST_REG)
        {
                 return -EINVAL;
        }

        value &= ~(PCAP_RW_MSK|PCAP_REG_MSK|PCAP_DEAD_MSK);
        value |= PCAP_RW_MSK|((reg<<PCAP_REG_SFT)&PCAP_REG_MSK);
        /* value = __cpu_to_be32(value); */
        return spi_int_transceive(&value, &value);
}
/*----------------------New SPI Driver  end ---------------------------*/

/*----------------------Begin Add SPI_AQ and SPI_RQ--------------------*/
/*
 * macro for flagBlock, the different type of a2d conversion
 */
#define SPI_A2D_BLOCK                   0x0
#define SPI_A2D_NON_BLOCK               0x1
#define SPI_A2D_NON_BLOCK_PREEMPTABLE   0x2
inline int get_spi_a2d_block(void)
{
        return SPI_A2D_BLOCK;
}
inline int get_spi_a2d_non_block(void)
{
        return SPI_A2D_NON_BLOCK;
}
inline int get_spi_a2d_non_block_preemptable(void)
{
        return SPI_A2D_NON_BLOCK_PREEMPTABLE;
}

/* wait queue for the SPI_NQ */
DECLARE_WAIT_QUEUE_HEAD(enter_dma_wait_queue);
/* flag for wait queue of SPI_NQ */
static int flag_enter_dma_wait_queue = false;

/*
 * use interrupt lock to lock this spi_bus_status
 */
#define SPI_STATUS_IDLE               0x0
#define SPI_STATUS_DMA_MODE           0x1
#define SPI_STATUS_INTERRUPT_MODE     0x2

struct SPI_AQ_NODE_T
{
        struct list_head node; /* krenerl double link list implementation */

        int (*setup_conversion_p)(void *);    /* conversion setup function */
        void * setup_conversion_parameter_p;  /* setup function parameter */
        int (*finish_conversion_p)(void *);   /* conversion finish function */
        void * finish_conversion_parameter_p; /* finish function parameter */
        int flagBlock;                        /* Block flag */
        int flagComplete;                     /* Complete flag */
        u32 start_jiffies;                    /* conversion start jiffies */
};


/* the flag which indicate A2D convertion processing or not */
static struct SPI_AQ_NODE_T * p_cur_aq_node = NULL;
/* general a2d conversion no more than 1 scond */
#define A2D_JIFFIES_LIMIT                (HZ)      

/* dobule linked queue head for SPI_AQ */
LIST_HEAD(SPI_AQ);
/* use wait queue for SPI_AR block mode */
DECLARE_WAIT_QUEUE_HEAD(SPI_AQ_COMPLETE_QUEUE);

/* SPI bus status */
int spi_bus_status = SPI_STATUS_IDLE;

/*!
 * @brief Clear ADCDONEI interrupt.
 *
 * In order to handle missing ADCDONEI interrupt case, we need to clear 
 * ADCDONEI interrupt if it really happen.
 *
 * @return    true  if ADCDONEI interrupt happen and clear <BR>
 *            false if ADCDONEI interrupt didn't happen <BR>
 */
int clear_ADCDONEI(void)
{
    if( GPLR(GPIO1_RST)&GPIO_bit(GPIO1_RST) )
    {
        spi_int_reg_write (POWER_IC_REG_PCAP_ISR, 
           1 << (POWER_IC_EVENT_PCAP_ADCDONEI - POWER_IC_EVENT_PCAP_FIRST) );
        return true;
    }
    return false;
}

/*!
 * @brief release the SPI bus, it should be called with interrupt disabled.
 *
 * Release the SPI bus. In case of read/write conflication, this function should
 * be called with interrupt disabled. 
 *
 */
inline void release_spi_bus(void)
{
        if(spi_bus_status == SPI_STATUS_INTERRUPT_MODE)
        {
                 flag_enter_dma_wait_queue = true;
                 /* high priority than DMA_MODE */
                 /* so we need to wake up the enter_dma_wait_queue */
                 wake_up(&enter_dma_wait_queue);
        }
        spi_bus_status = SPI_STATUS_IDLE;
}

/*!
 * @brief Modify SPI bus status, get/release SPI bus control
 *
 * In order to handle the SPI bus contention we need to manage the SPI control
 * permission. Each SPI operations should get the control before it process.
 *
 * @param     in_status        The request SPI bus status to be changed to 
 *
 * @return    true upon success<BR>
 */
int modify_spi_bus_status(int in_status)
{
        int ret = false;
        u32 spi_bus_int_lock_flag;
        spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);

        if(in_status == SPI_STATUS_IDLE)
        {/* the spi bus is idle now, so direct change mode */
                release_spi_bus();
                ret = true;
        }
        else
        {/* the spi bus is not idle, we need to check */
         /* spi bus status can only change from busy to idle */
                if(spi_bus_status == SPI_STATUS_IDLE)
                {
                        spi_bus_status = in_status;
                        ret = true;

                        /* protection for a2d conversion */
                        if( (p_cur_aq_node !=NULL)&&
                            (p_cur_aq_node->flagBlock!=SPI_A2D_NON_BLOCK_PREEMPTABLE))
                        {
                                if(jiffies >= p_cur_aq_node->start_jiffies)
                                {
                                        if ((jiffies - p_cur_aq_node->start_jiffies) > A2D_JIFFIES_LIMIT)
                                        {
                                                list_add(&p_cur_aq_node->node, &SPI_AQ);
                                                p_cur_aq_node = NULL;
                                                printk("a2d time over! retry again!\n");
                                                if(clear_ADCDONEI()){
                                                    printk("clear ADCDONEI ok!\n");
                                                }

                                        }
                                }
                                else
                                {
                                        if(jiffies > A2D_JIFFIES_LIMIT)
                                        {/* jiffies loop back  */
                                                list_add(&p_cur_aq_node->node, &SPI_AQ);
                                                p_cur_aq_node = NULL;
                                                printk("a2d time over! retry again 2!\n");
                                                if(clear_ADCDONEI()){
                                                    printk("clear ADCDONEI ok!\n");
                                                }
                                        }
                                }
                        }/* (p_cur_aq_node !=NULL) */

                }
        }

        spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
        return ret;
}

/*!
 * @brief Enter SPI bus idle status
 */
void enter_idle(void)
{
  modify_spi_bus_status(SPI_STATUS_IDLE);
}

/*!
 * @brief Enter SPI bus DMA transfer status
 *
 * All SPI bus operations from pxa_ssp.c regards as DMA operations.
 */
void enter_dma(void)
{
        while(modify_spi_bus_status(SPI_STATUS_DMA_MODE)==false)
        { 
                /* because SPI_AQ and SPI_RQ has high priority */
                /* so we need to wait for them */
                /* enter_dma wait... */
                wait_event(enter_dma_wait_queue, flag_enter_dma_wait_queue);
                /* enter_dma wake up... */
                flag_enter_dma_wait_queue = false;
        }
}

/* 
 * double linked queue head for SPI_RQ 
 * only support block mode 
 */
LIST_HEAD(SPI_RQ);

struct SPI_RQ_NODE_T
{
  struct list_head node;
  void (*int_bottom_half_p)(void);
};

int spi_int_schedule(void);
struct SPI_AQ_NODE_T * kmalloc_spi_aq(void);
struct SPI_RQ_NODE_T * kmalloc_spi_rq(void);
void kfree_spi_aq(struct SPI_AQ_NODE_T * p_old);
void kfree_spi_rq(struct SPI_RQ_NODE_T * p_old);

/*!
 * @brief free memory of SPI_AQ node 
 *
 * if this SPI_AQ node is a nonblock a2d request free this node directly
 * if it is a block a2d request, this node should be free in spi_a2d_req
 *
 * @param     SPI_AQ_NODE_T  Pointer to a SPI_AQ node
 *
 */
void a2d_queue_delete(struct SPI_AQ_NODE_T * temp_pp)
{
      temp_pp-> flagComplete = true;
      /*
        temp_pp-> setup_conversion_p = NULL;
        temp_pp-> setup_conversion_parameter_p = NULL;
        temp_pp-> finish_conversion_p = NULL;
        temp_pp-> finish_conversion_parameter_p = NULL;
      */

      if(temp_pp->flagBlock == SPI_A2D_BLOCK)
      {
        /* delay kfree operation in spi_a2d_req */
        wake_up(&SPI_AQ_COMPLETE_QUEUE);
      }
      else
      {
        kfree_spi_aq(temp_pp);
      }
}

/*!
 * @brief atod conversion request (SPI_AR)
 *
 * atod conversion request, add this request to SPI_AQ 
 * if it is a block request, block the request until this conversion finished.
 *
 * @param     setup_p  Special atod conversion setup function pointer. 
 * @param     setup_para_p Atod conversion setup function parameter.
 * @param     finish_p  Special atod conversion finish function pointer. 
 * @param     finish_para_p Atod conversion finish function parameter.
 *
 * @return    true  upon success<BR>
 *            false if alloc memory fail!.<BR>
 */
int spi_a2d_req( int (*setup_p)(void*), 
                 void * setup_para_p,
                 int (*finish_p)(void *), 
                 void * finish_para_p,
                 int flagBlock
                 )
{
        struct SPI_AQ_NODE_T * pNew;
        pNew = kmalloc_spi_aq();

        if(pNew == NULL)
        {
                printk("SPI BUS: Warning, spi_a2d_req request node fail!\n");
                return false;
        }

        pNew -> setup_conversion_p = setup_p;
        pNew -> setup_conversion_parameter_p = setup_para_p;
        pNew -> finish_conversion_p = finish_p;
        pNew -> finish_conversion_parameter_p = finish_para_p;
        pNew -> flagBlock = flagBlock;
        pNew -> flagComplete = false;
        pNew -> start_jiffies = 0x0;

        u32 spi_bus_int_lock_flag;  
        spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);

        if(flagBlock == SPI_A2D_NON_BLOCK)
        {/* higher priority, add it to list head */
                list_add(&pNew->node, &SPI_AQ);
        }
        else
        {
                list_add_tail(&pNew->node, &SPI_AQ);
        }

        spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);

        spi_int_schedule();

        if(flagBlock == SPI_A2D_BLOCK)
        {/* need to block until finish */
                wait_event(SPI_AQ_COMPLETE_QUEUE, (pNew->flagComplete!=false));
                kfree_spi_aq(pNew);
                return true;
        }
        else
        {/* nonBlock direct return */
                return true;
        }
}

/*!
 * @brief spi bus operation request from interrupt context (SPI_RQ)
 *
 * spi bus operation request, 
 * if spi bus is idle now, run this request function directly 
 * if spi bus is busy now, add this request to SPI_RQ 
 *
 * @param     ptr_procedure  Pointer to spi bus operation function.
 *
 * @return    true  success, run directly or queued. <BR>
 *            false failed to allocate memory.<BR>
 */
int spi_int_req( void (*ptr_procedure)(void) )
{
        if(!modify_spi_bus_status(SPI_STATUS_INTERRUPT_MODE))
        {
                /* can not hold the spi bus so enqueue */
                struct SPI_RQ_NODE_T * pNew;
                pNew = kmalloc_spi_rq();

                if(pNew == NULL)
                {
                        printk("SPI BUS: Warning, spi_int_req request node fail!\n");
                        return false;
                }
                pNew -> int_bottom_half_p = ptr_procedure; 
 
                u32 spi_bus_int_lock_flag;
                spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);
                list_add_tail(&pNew->node, &SPI_RQ);
                spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
                return true;
        }
        /* get spi bus control now! direct call the procedure! */
        ptr_procedure();

        modify_spi_bus_status(SPI_STATUS_IDLE);

        /* because there are no operations now */
        /* so we need to call spi_int_schedule to  */
        /* run the new operation(s) add by ptr_procedure; */
        spi_int_schedule();
 
        return true;
}

/*!
 * @brief Set the value of a range of bits in a power IC register
 *
 * This function is used to set a range of bits in a power IC register.  The
 * function is implemented as a call to spi_int_set_reg_mask() by converting
 * the input parameters index and nb_bits into a bitmask.
 * This function has an assumption that it has hold spi bus read/write 
 * permissition. (modify_spi_bus_status)
 *
 * @param        reg        register number
 * @param        index      starting bit index (0 = least-significant bit)
 * @param        value      new value
 * @param        nb_bits    number of bits to set
 *
 * @return 0 if successful
 */ 
int spi_int_set_reg_value(POWER_IC_REGISTER_T reg, int index, int value, int nb_bits) 
{
        return spi_int_set_reg_mask (reg, (((1 << nb_bits) - 1) << index), value << index);
}

/*!
 * @brief Set the value of a single bit in a power IC register
 *
 * This function is used to set an individual bit in a power IC register.  The
 * function is implemented simply as a call to spi_int_set_reg_value() with
 * the number of bits parameter set to 1.
 * This function has an assumption that it has hold spi bus read/write 
 * permissition. (modify_spi_bus_status)
 *
 * @param        reg        register number
 * @param        index      bit index to set (0 = least-significant bit)
 * @param        value      new bit value
 *
 * @return 0 if successful
 */ 
int spi_int_set_reg_bit(POWER_IC_REGISTER_T reg, int index, int value) 
{
        return spi_int_set_reg_value (reg, index, value ? 1 : 0, 1);
}

/*!
 * @brief Run available spi a2d conversion requests and spi interrupt requests.
 *
 * This function is used to set an individual bit in a power IC register.  The
 * function is implemented simply as a call to spi_int_set_reg_value() with
 * the number of bits parameter set to 1.
 * This function has an assumption that it has hold spi bus read/write 
 * permissition. (modify_spi_bus_status)
 *
 * @return false if can not get spi bus read/write permission.a
 *         true  if available spi requests have been handled.
 */ 
int spi_int_schedule(void)
{
        struct SPI_AQ_NODE_T * p_temp_aq_node = NULL;
        struct list_head *iterator;

        /* apply the SPI bus control */
        if(!modify_spi_bus_status(SPI_STATUS_INTERRUPT_MODE))
        {
                return false;
        }

/*
 * We should loop for all available nodes, and we have a assumption that 
 * there are only several operations now.
 * all nodes of SPI_RQ and the first node of SPI_AQ
 * if A2D conversion is processing, the first node is p_cur_aq_node
 * not the first node of SPI_AQ
 * if the type of first node is PREEMPTABLE, we will preempty it.
 */
        u32 spi_bus_int_lock_flag;
spi_int_schedule_loop:
        spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);
        if(list_empty(&SPI_RQ))
        {/* SPI_RQ is empty! */
                if(list_empty(&SPI_AQ))
                {/* no more A2D conversion request now, return */
                        goto spi_int_schedule_exit;
                }

                if(p_cur_aq_node!=NULL)
                {/* A2D conversion is processing --- p_cur_aq_node != NULL */

                        /*
                         * if current a2d conversion is a preemptable one
                         * because the ATO counter does not count down while
                         * the conversion processing, we need to check the 
                         * POWER_IC_REG_PCAP_ISR register to make sure the 
                         * conversion does not finish. And mask it.
                         */
                        if (p_cur_aq_node->flagBlock==
                                      SPI_A2D_NON_BLOCK_PREEMPTABLE)
                        { 
                                /*
                                 * get the next available a2d node, 
                                 * not a preemptable one
                                 */
                                list_for_each(iterator, &SPI_AQ) {
                                        p_temp_aq_node = list_entry(iterator, 
                                                struct SPI_AQ_NODE_T, node);
                                        if(p_temp_aq_node->flagBlock!=
                                                SPI_A2D_NON_BLOCK_PREEMPTABLE)
                                        {
                                                break;
                                        }else{
                                                p_temp_aq_node = NULL;
                                        }
                                }
                                if(p_temp_aq_node !=NULL)
                                {/* we find a available node */
                                        spi_int_stop_atod_converter();
                                        clear_ADCDONEI();
                                        list_add_tail(&p_cur_aq_node->node, &SPI_AQ);
                                        p_cur_aq_node = p_temp_aq_node;
                                }
                                else
                                {
/* if we want to drop the later preemptable node, just uncomment this
                                        printk("more than 1 preemptable node, please checked\n");
                                        p_temp_aq_node = list_entry((SPI_AQ.next),
                                                struct SPI_AQ_NODE_T,
                                                node);    
                                        if(p_temp_aq_node->flagBlock==SPI_A2D_NON_BLOCK_PREEMPTABLE)
                                        {
                                                list_del(&(p_temp_aq_node->node));
                                                a2d_queue_delete(p_temp_aq_node);
                                                printk("drop the reenter one!\n");
                                        }
*/
                                        goto spi_int_schedule_exit;
                                }
                       }
                       else
                       {
                                goto spi_int_schedule_exit;
                       }
                }
                else
                {
                         p_cur_aq_node = list_entry((SPI_AQ.next),
                                 struct SPI_AQ_NODE_T, node);
                }
                list_del(&(p_cur_aq_node->node));

                spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
    
                /*
                 * we need to check this before setup_conversion_p call
                 * because while the finish interrupt come
                 * the global pointer p_cur_aq_node may changed before we check!
                 */
                if(p_cur_aq_node->setup_conversion_p)
                {
                        p_cur_aq_node -> start_jiffies = jiffies;

                        /* unmask the  POWER_IC_EVENT_PCAP_ADCDONEI */
                        spi_int_set_reg_mask(0x1, 0x1, 0x0);

                        if(0!=(p_cur_aq_node -> setup_conversion_p(
                            p_cur_aq_node -> setup_conversion_parameter_p)))
                        {/* error occur in calling setup_conversion_p! */
                                printk("SPI_BUS: error occur while setup a2d conversion, please check!\n");
                                struct SPI_AQ_NODE_T * temp_pp = p_cur_aq_node;
                                p_cur_aq_node = NULL;
                                a2d_queue_delete(temp_pp);
                        }
                 }
                 /* perform the next operation */
                 goto spi_int_schedule_loop;
         }

         struct SPI_RQ_NODE_T * tempP;
         tempP = list_entry((SPI_RQ.next), struct SPI_RQ_NODE_T, node);
         list_del(&(tempP->node));
         spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
         tempP -> int_bottom_half_p(); 

         kfree_spi_rq(tempP);

         /* perform the next operation */
         goto spi_int_schedule_loop;

spi_int_schedule_exit:
         release_spi_bus();
         spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
         return true;
}

/*!
 * @brief handle a2d conversion finish interrupts.
 *
 * This function has an assumption that it has hold spi bus read/write 
 * permissition. (modify_spi_bus_status)
 * All spi a2d conversions have been seperate into 2 phase:
 *    setup conversion and start conversion phase,
 *    finish conversion and get results phase,
 */ 
void spi_a2d_interrupt_handler(void)
{
         if(p_cur_aq_node)
         {/* No queue operation need! */
                if(p_cur_aq_node->finish_conversion_p != NULL)
                {
                        p_cur_aq_node->finish_conversion_p(
                            p_cur_aq_node->finish_conversion_parameter_p);
                }

                spi_int_stop_atod_converter();

                struct SPI_AQ_NODE_T * temp_pp = p_cur_aq_node;
                p_cur_aq_node = NULL;
                a2d_queue_delete(temp_pp);
        }
        else
        {
                printk("SPI BUS: Warning, no atod finish handler found!\n");
        }
}

/*-----------------------End  Add SPI_AQ and SPI_RQ-------------------- */

/*----------------Memory Pool for SPI_AQ and SPI_RQ begin--------------*/
/*
 * 1. We reserve nodes at the beginning.
 * 2. We doesn't really return memory to system.
 * 3. If the memory pool is empty, NULL will be return.
 * 4. Kmalloc_spi_aq should take place of  all kmalloc for SPI_AQ_NODE_T.
 * 5. Kfree_spi_aq should take place of all kmalloc for SPI_AQ_NODE_T.
 * 6. Kmalloc_spi_rq should take place of  all kmalloc for SPI_RQ_NODE_T.
 * 7. Kfree_spi_rq should take place of all kmalloc for SPI_RQ_NODE_T.
 * 8. We use two double-link queues to simulate the stack structures.
 *     SPI_AQ_STACK  and SPI_RQ_STACK
 * 9. The stack limit will be 20 for each queue now.
 */

/* dobule linked queue head for SPI_AQ_STACK */
LIST_HEAD(SPI_AQ_STACK);

/* dobule linked queue head for SPI_RQ_STACK */
LIST_HEAD(SPI_RQ_STACK);

/* generally 3 is enough */
#define SPI_RQ_STACK_LIMIT      20
/* generally 2 is enough */
#define SPI_AQ_STACK_LIMIT      20

/*!
 * @brief malloc memory for SPI_AQ
 *
 * Malloc memory for SPI_AQ from SPI_AQ_STACK.
 *
 * @return    memory pointer  upon success.<BR>
 *            NULL  if no node avaliable.<BR>
 *
 */
struct SPI_AQ_NODE_T * kmalloc_spi_aq(void)
{
        u32 spi_bus_int_lock_flag;
        spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);

        /* we check the reserve stack for memory nodes */
        if(list_empty(&SPI_AQ_STACK))
        {
                spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
                return NULL;
        }

        struct SPI_AQ_NODE_T * p_new;
        p_new = list_entry((SPI_AQ_STACK.next), struct SPI_AQ_NODE_T, node);
        list_del(&(p_new->node));

        spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);

        return p_new;
}

/*!
 * @brief malloc memory for SPI_RQ
 *
 * Malloc memory for SPI_RQ from SPI_RQ_STACK.
 *
 * @return    memory pointer  upon success.<BR>
 *            NULL  if no node avaliable.<BR>
 *
 */
struct SPI_RQ_NODE_T * kmalloc_spi_rq(void)
{
        u32 spi_bus_int_lock_flag;
        spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);

        /* we check the reserve stack for memory nodes */
        if(list_empty(&SPI_RQ_STACK))
        {
                spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
                return NULL;
        }

        struct SPI_RQ_NODE_T * p_new;
        p_new = list_entry((SPI_RQ_STACK.next), struct SPI_RQ_NODE_T, node);
        list_del(&(p_new->node));

        spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);

        return p_new;
}

/*!
 * @brief free memory for SPI_AQ
 *
 * Free memory for SPI_AQ from SPI_AQ_STACK.
 *
 * @return    memory pointer  upon success.<BR>
 *            NULL  if no node avaliable.<BR>
 *
 */
void kfree_spi_aq(struct SPI_AQ_NODE_T * p_old)
{
        struct list_head *iterator;
        int stack_counter = 0x0;

        if(p_old == NULL) return;

        u32 spi_bus_int_lock_flag;
        spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);

        list_for_each(iterator, &SPI_AQ_STACK) {
                stack_counter ++;
        }

        if(stack_counter >= SPI_AQ_STACK_LIMIT)
        {
                spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
                kfree(p_old);
        }else{
                list_add(&p_old->node, &SPI_AQ_STACK);
                spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
        }
}

/*!
 * @brief free memory for SPI_RQ
 *
 * Free memory for SPI_RQ from SPI_RQ_STACK.
 *
 * @return    memory pointer  upon success.<BR>
 *            NULL  if no node avaliable.<BR>
 *
 */
void kfree_spi_rq(struct SPI_RQ_NODE_T * p_old)
{
        struct list_head *iterator;
        int stack_counter = 0x0;

        if(p_old == NULL) return;

        u32 spi_bus_int_lock_flag;
        spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);

        list_for_each(iterator, &SPI_RQ_STACK) {
                stack_counter ++;
        }

        if(stack_counter >= SPI_RQ_STACK_LIMIT)
        {
                spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
                kfree(p_old);
        }else{
                list_add(&p_old->node, &SPI_RQ_STACK);
                spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
        }
}

/*!
 * @brief Reserve memory for SPI bus AtoD conversion request.
 *
 * This function is used to reserve memory for SPI bus AtoD conversion request.
 *
 * @param        count      the initialization nodes count.
 *
 */ 
void __init spi_int_init_aq(int count) 
{
        struct SPI_AQ_NODE_T * p_temp = NULL;
        u32 spi_bus_int_lock_flag;
        spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);
        
        while(count >= 0)
        {
                p_temp = NULL;
                while(p_temp == NULL)
                {
                        p_temp = (struct SPI_AQ_NODE_T *)kmalloc( sizeof(struct SPI_AQ_NODE_T), GFP_ATOMIC);
                }
                list_add(&p_temp->node, &SPI_AQ_STACK);
                count--;
        }

        spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
}

/*!
 * @brief Reserve memory for SPI bus interrupt handle request.
 *
 * This function is used to reserve memory for SPI bus interrupt handle request.
 *
 * @param        count      the initialization nodes count.
 *
 */ 
void __init spi_int_init_rq(int count) 
{
        struct SPI_RQ_NODE_T * p_temp = NULL;
        u32 spi_bus_int_lock_flag;
        spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);
        
        while(count >= 0)
        {
                p_temp = NULL;
                while(p_temp == NULL)
                {
                        p_temp = (struct SPI_RQ_NODE_T *)kmalloc( sizeof(struct SPI_RQ_NODE_T), GFP_ATOMIC);
                }
                list_add(&p_temp->node, &SPI_RQ_STACK);
                count--;
        }

        spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
}

/*!
 * @brief Reserve memory for SPI bus batch operations.
 *
 * This function is used to reserve memory for SPI bus AtoD conversion request 
 * and interrupt handle request.
 *
 */ 
void __init spi_int_memory_pool_init(void)
{
        spi_int_init_aq(SPI_AQ_STACK_LIMIT); 
        spi_int_init_rq(SPI_RQ_STACK_LIMIT); 
}
/*----------------Memory Pool for SPI_AQ and SPI_RQ end----------------*/

#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */
