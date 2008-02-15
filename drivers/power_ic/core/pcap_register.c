/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/core/pcap_register.c
 *
 * Description - This file contains register function of PCAP driver.
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
 * Motorola 2005-Feb-28 - Full rewrite from Atlas to PCAP registers
 * Motorola 2005-Jun-16 - Rewrote most functions to be more efficient
 *                        with more of the logic contained in this
 *                        file.
 * Motorola 2205-Jul-21 - Added a Table for spi settings in an 
 *                        interrupt context. 
 *
 */

/*!
 * @file pcap_register.c
 *
 * @brief This file contains register function of PCAP driver.
 *
 * The format of a message to the pcap in bits is as follows:<BR>
 * <BR>
 * wrrrrrxd dddddddd dddddddd dddddddd<BR>
 * <BR>
 * <B>Where:</B><BR>
 *    w     - Is the read write bit with write being active high.<BR>
 *    rrrrr - Is the register address.<BR>
 *    x     - Is a dead bit which can be low or high. The code makes it low.<BR>
 *    d...d - Is the data to be written when w is high or the data read when
 *            w is low.<BR>
 *
 * @ingroup poweric_core
 */

#include <linux/errno.h>
#include <linux/power_ic.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <asm/byteorder.h>
#include "spi_main.h"

/*!
 * @brief Used to mask off the read write bit for the PCAP.
 *
 * Low is for a read and high is for a write.
 */
#define PCAP_RW_MSK  0x80000000

/*!
 * @brief Used to mask off the register number for the PCAP.
 */
#define PCAP_REG_MSK 0x7C000000

/*!
 * @brief The bit position for the start of the PCAP register number.
 */
#define PCAP_REG_SFT 26

/*!
 * @brief Used to mask off the dead bit for the PCAP.
 */
#define PCAP_DEAD_MSK 0x02000000

static void pcap_reg_tx_complete(int sequence, void *unused);

/*
 * @brief Table used to control the SPI parameters for a non interrupt context.
 * See SPI_DATA_PARAMETERS_T as well as section 3.5.2 of the PCAP spec
 * for more details.
 */
static const SPI_DATA_PARAMETER_T pcap_normal_spi_params =
    {
        .cs = SPI_CS_PCAP,            /* Use the PCAP chip select. */
        .nonblocking = false,         /* Wait for the transfers to complete before returning. */
        .lock_cs = false,             /* Do not extend the chip select over multiple transfers. */
        .clk_idle_state = false,      /* Clock idles low for the PCAP. */
        .clk_phase = false,           /* Clock phase for the PCAP.  */
        .speed = 13000000,            /* Transfer at 13 MHz, max speed is 20 MHz. */
        .xfer_complete_p = NULL,      /* No need for a callback since the transfer will complete quickly. */
        .xfer_complete_param_p = NULL /* No data for the callback needed, since their is not callback. */
    };

/*
 * @brief Table used to control the SPI parameters for an interrupt context.
 * See SPI_DATA_PARAMETERS_T as well as section 3.5.2 of the PCAP spec
 * for more details.
 */
static const SPI_DATA_PARAMETER_T pcap_irq_spi_params =
    {
        .cs = SPI_CS_PCAP,                       /* Use the PCAP chip select. */
        .nonblocking = true,                     /* Do not Wait for the transfers to complete before returning. */
        .lock_cs = false,                        /* Do not extend the chip select over multiple transfers. */
        .clk_idle_state = false,                 /* Clock idles low for the PCAP. */
        .clk_phase = false,                      /* Clock phase for the PCAP.  */
        .speed = 13000000,                       /* Transfer at 13 MHz, max speed is 20 MHz. */
        .xfer_complete_p = pcap_reg_tx_complete, /* Callback to dispose of the response.  */
        .xfer_complete_param_p = NULL            /* No data for the callback needed. */
    };

/*!
 * @brief Function called once a register write completed
 *
 * This function is called once the transfer of data to the PCAP is
 * completed.  It may actually be called before pcap_reg_write returns.
 * It is reponsble for fulfilling the SPI requirement of calling
 * spi_check_complete() in order to clean up the memory in the spi queue.
 *
 * @param sequence     The sequence number of the transfer to clean up
 * @param unused       Unused optional data
 *
 */
static void pcap_reg_tx_complete(int sequence, void *unused)
{
    /* Toss out the response. */
    spi_check_complete((SPI_DATA_PARAMETER_T *)&pcap_irq_spi_params, sequence);
}

/*!
 * @brief Reads a PCAP register
 *
 * This function implements a read of a given PCAP register.  Currently, all
 * PCAP register access goes through the legacy SSP_PCAP functions.  The
 * function spi_transceive() is used to read a register.
 *
 * @param reg          PCAP register number
 * @param value_ptr    Pointer to which the read data should be stored
 *
 * @return 0 if successful<BR>
 *         -EINVAL upon an error<BR>
 *         See also spi_transceive() return values<BR>
 *
 * @note The memory which is pointed to by value_ptr must be DMA accessible on
 *       most systems.  This is a requirement of spi_transceive().
 */
int pcap_reg_read (int reg, unsigned int *value_ptr)
{
    int error;
    
    if (reg > POWER_IC_REG_PCAP_LAST_REG-POWER_IC_REG_PCAP_FIRST_REG)
    {
        return -EINVAL;
    }
    /* Set up the address and the read write bit to a 0. */
    *value_ptr = (reg<<PCAP_REG_SFT)&PCAP_REG_MSK;
    /* Put the bit in the correct order for the transfer to the pcap. */
    *value_ptr = __cpu_to_be32(*value_ptr);
    error = spi_transceive((SPI_DATA_PARAMETER_T *)&pcap_normal_spi_params, sizeof(unsigned int), value_ptr, value_ptr);
    *value_ptr = __be32_to_cpu(*value_ptr);
    return error;
}

/*!
 * @brief Writes a PCAP register
 *
 * This function implements a write to a given PCAP register.  Currently, all
 * PCAP register access goes through the legacy SSP_PCAP functions.  The
 * function spi_transceive() is used to write a register.
 *
 * @param reg          PCAP register number
 * @param value        Value to write to the register
 *
 * @return 0 if successful<BR>
 *         -EINVAL upon an error<BR>
 *         See also spi_transceive() return values<BR>
 *
 * @note In an interrupt context the parameter value must be preserved until
 *       after pcap_reg_tx_complete() is called.  This currently works since
 *       the transfer is very fast.  If the SPI clock is made slower, this
 *       may no longer be the case and this function will need to be updated.
 */

int pcap_reg_write (int reg, unsigned int value)
{
    SPI_DATA_PARAMETER_T *spi_params_p;
    int sequence;
    
    if (reg > POWER_IC_REG_PCAP_LAST_REG-POWER_IC_REG_PCAP_FIRST_REG)
    {
        return -EINVAL;
    }

    /* If in an interrupt context do not block and use a callback to remove
       the qspi node. */
    spi_params_p = (SPI_DATA_PARAMETER_T *)&pcap_normal_spi_params;
    if (in_interrupt())
    {
        spi_params_p = (SPI_DATA_PARAMETER_T *)&pcap_irq_spi_params;
    }
    value &= ~(PCAP_RW_MSK|PCAP_REG_MSK|PCAP_DEAD_MSK);
    value |= PCAP_RW_MSK|((reg<<PCAP_REG_SFT)&PCAP_REG_MSK);
    value = __cpu_to_be32(value);
    sequence = spi_transceive(spi_params_p, sizeof(value), &value, &value);
    
    /* Convert a postive sequence value to a vaild return value. */
    if (sequence > 0)
    {
        sequence = 0;
    }
    return sequence;
}
