/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/core/pxa_ssp.c
 * 
 * Description -  This is the low level SPI driver for Intel Bulverde.
 *
 * Copyright (C) 2005-2006 Motorola, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published 
 * by the Free Software Foundation.
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
 * 2005-Apr-04 - Design of low level SPI interface for Intel Bulverde.
 *
 * 2006-Jan-10 - Finalize the interface for Intel Bulverde
 */
/*!
 * @file pxa_ssp.c
 *
 * @ingroup spi_management
 *
 * @brief This is the low level SPI driver for Intel Bulverde.
 *
 * <B>Overview:</B><BR>
 *   Implements the interface needed to transmit and receive SPI messages
 *   over SSP0.  The code uses DMA to transfer data to and from the SSP FIFO.
 *   Two DMA channels are used for moving the data for transmit and receive.
 *   Transmit complete is handled by using the DMA transfer complete interrupt
 *   on the receive DMA channel for large or slow transfers.  Small quick transfers
 *   do not use the interrupt, but simply poll for the DMA transfer complete.
 *   This is done to save interrupt overhead for small transfers.
 *
 *   Once a transmit is completed, the module will inform spi_main of the completed
 *   transmit by calling spi_tx_completed.  If this routine does not return
 *   NULL it passes back a pointer to the next node in the queue which must be
 *   transmitted.
 *
 *   spi_main calls pxa_ssp_start_tx() to start a transmit.  It will call
 *   pxa_ssp_start_tx() for every entry in the queue, so pxa_ssp_start_tx() must
 *   ignore requests while currently sending data, since the next entry will be
 *   started when the current transmit completes.
 *
 *   This driver must also be able to handle being passed a node with no data to
 *   transmit.  This is done when a chip select lock needs to be released and
 *   no further data needs to be transmitted.
 *
 *   spi_main also calls pxa_ssp_initialize() at initialization time.
 */
#include <linux/config.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <asm/arch/pxa-regs.h>
#include <asm/dma.h>
#include <asm/hardware.h>
#include <asm/memory.h>
#include <asm/proc/cache.h>
#include <stdbool.h>
#include "spi_main.h"
#include "pxa_ssp.h"
#include "os_independent.h"

/******************************************************************************
* Local constants and macros
******************************************************************************/

/*
 * SSP bit defines which are not included in pxa-regs.h.  These are only here until the changes
 * are integrated int pxa-regs.h.
 */
#ifndef NSSCR0_TIM
# define NSSCR0_TIM        (1<<23)  /*!< @brief When set the TX FIFO under run interrupt is enabled. */
#endif
#ifndef NSSCR0_RIM
# define NSSCR0_RIM        (1<<22)  /*!< @brief When set the RX FIFO overrun interrupt is enabled. */
#endif
#ifndef NSSCR0_EDSS
# define NSSCR0_EDSS       (1<<20)  /*!< @brief Set when more than 16 bit data transfers are desired.  See
                                         #NSSCR0_DSS_MSK for more details. */
#endif
#ifndef NSSCR0_SCR_SFT
# define NSSCR0_SCR_SFT    8        /*!< @brief The bit position (shift value) for the Serial Clock Rate */
#endif
#ifndef NSSCR0_SCR_MSK
# define NSSCR0_SCR_MSK    (0x0fff<<NSSCR0_SCR_SFT) /*!< @brief Mask to set the Serial Clock Rate register
                                                         which contains an integer divider for the clock. */
#endif
#ifndef NSSCR0_DSS_MSK
# define NSSCR0_DSS_MSK    (0xf<<0) /*!< @brief Used to mask off the bit transfer size.
                                         The bit transfer size is in bits when #NSSCR0_EDSS is 0 and in bits+17
                                         when #NSSCR0_EDSS is 1. */
#endif
#ifndef NSSCR1_TRAIL
# define NSSCR1_TRAIL      (1<<22)  /*!< @brief 0 if trailing bytes are handled by CPU or 1 if handled by DMA. */
#endif
#ifndef NSSCR1_RFT_SFT
# define NSSCR1_RFT_SFT    10       /*!< @brief The bit position of the receive FIFO threshold. */
#endif
#ifndef NSSCR1_TFT_SFT
# define NSSCR1_TFT_SFT    6        /*!< @brief The bit position of the transmit FIFO threshold. */
#endif
#ifndef SSTO
# define SSTO              __REG(0x41000028)  /*! @brief SSP timeout register. */
#endif
#ifndef NSSTO_MSK
# define NSSTO_MSK         0x00ffffff  /*!< @brief Mask for the SSTO value. */
#endif
#ifndef NSSSR_TFL_SFT
# define NSSSR_TFL_SFT     8        /*!< @brief Bit position of the TFL bits in the SSSR. */
#endif
#ifndef NSSRS_RFL_SFT
# define NSSSR_RFL_SFT     12       /*!< @brief Bit position of the RFL bits in the SSSR. */
#endif 
#ifndef DALGN
# define DALGN             __REG(0x400000a0) /*!< @brief Alignment register for DMA. */
#endif

/*! @brief Retrieves the value of the TFL and shifts it info position. */
#define GET_SSSR_TFL      ((SSSR&NSSSR_TFL_MASK)>>NSSSR_TFL_SFT)
/*! @brief Retrieves the value of the RFL and shifts it info position. */
#define GET_SSSR_RFL      ((SSSR&NSSSR_RFL_MASK)>>NSSSR_RFL_SFT)

#define SSP_MAX_CLK       13000000 /*!< @brief The maximum clock rate for the SSP. */
#define SSP_FIFO_SIZE     16       /*!< @brief The size of the transmit and receive SSP FIFOs. */

/*!
 * @brief The clock rate of the PXA Peripheral Clock in HZ.
 */
#define PXA_PERIPHERAL_CLK 312000000

/*!
 * @brief The number of micro seconds to wait for a transmit to complete.
 *
 * If a transmit will take equal to or more than this value an interrupt will be used
 * to handle the completion of the transmit.  Values less than this this will simply
 * poll for the completion.
 *
 */
#define PXA_POLL_RESULT_US 500

/*!
 * @brief Macro used to print the important registers for debugging.
 */
#ifdef POWER_IC_PXA_SSP_DEBUG
# define DEBUG_MSG tracemsg
# define DEBUG_PRINT_REGS(s) tracemsg(_k_d(s "cken: %08x sscr0: %08x sscr1: %08x sssr:%08x\nddadr.rx: %08x dcsr.rx:%08x dcmd.rx:%08x dtadr.rx:%08x dsadr.rx:%08x\nddadr.tx: %08x dcsr.tx:%08x dcmd.tx:%08x dtadr.tx:%08x dsadr.tx:%08x"), \
                                      CKEN, SSCR0, SSCR1, SSSR, \
                                      DDADR(pxa_ssp_xfr_info.rx_dma_chan), DCSR(pxa_ssp_xfr_info.rx_dma_chan), \
                                      DCMD(pxa_ssp_xfr_info.rx_dma_chan),   \
                                      DTADR(pxa_ssp_xfr_info.rx_dma_chan), DSADR(pxa_ssp_xfr_info.rx_dma_chan), \
                                      DDADR(pxa_ssp_xfr_info.tx_dma_chan), DCSR(pxa_ssp_xfr_info.tx_dma_chan), \
                                      DCMD(pxa_ssp_xfr_info.tx_dma_chan),   \
                                      DTADR(pxa_ssp_xfr_info.tx_dma_chan), DSADR(pxa_ssp_xfr_info.tx_dma_chan))
#else
# define DEBUG_MSG(fmt, args...)
# define DEBUG_PRINT_REGS(s)
#endif

/******************************************************************************
* Local type definitions
******************************************************************************/

/*!
 * @brief Contains information pertaining to the current transfer which is in progress.
 *
 * This includes the dma channels along with an in progress flag.
 */
typedef struct
{
    int tx_dma_chan;           /*!< Transmit DMA channel */
    int rx_dma_chan;           /*!< Receive DMA channel */
    bool in_progress;          /*!< true if a transfer is in progress, false if not. */
    SPI_QUEUE_NODE_T *node_p;  /*!< Pointer to the SPI queue node which is being transmitted.  This is set up
                                    in pxa_ssp_initiate_tx and referenced out of the interrupts.  A parameter
                                    to the interrupt is not used, since the DMA channel will not be re-requested
                                    between interrupt driver transfers for execution speed reasons. */
} PXA_SSP_XFR_INFO_T;

/******************************************************************************
* Local variables
******************************************************************************/

/*!
 * @brief Holds the pointers to the dma descriptor chains as well as the number
 * of descriptors which can be used.
 */
static struct
{
    pxa_dma_desc rx[SPI_IOVEC_MAX];  /*!< Receive descriptor array. */
    pxa_dma_desc tx[SPI_IOVEC_MAX];  /*!< Transmit descriptor array. */
} pxa_ssp_dma_descriptors __attribute__ ((aligned(16)));

/*!
 * @brief Contains information pertaining to the current transfer which is in progress.
 *
 * This includes the dma channels along with an in progress flag.  See PXA_SSP_XFR_INFO_T
 * for more details.
 */
static PXA_SSP_XFR_INFO_T pxa_ssp_xfr_info;

/*!
 * true if the chip select is currently locked by the higher level driver, false if
 * it is not locked.
 */
static bool cs_locked;

/*!
 * @brief Table which contains the SPI chip select data.  This table is indexed by
 * SPI_CS_T.
 */
static const struct
{
    int16_t gpio;
    bool active_high;
} pxa_ssp_cs_info[SPI_CS__END] =
    {
        { GPIO_SPI_CE,  true  },  /* SPI_CS_PCAP   */
        { GPIO20_DREQ0, false },  /* SPI_CS_TFLASH */
        { -1,           false }   /* SPI_CS_DUMMY  */
    };

/******************************************************************************
* Local functions
******************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
static void pxa_ssp_dma_rx_irq(int channel, void *p, struct pt_regs *regs);
static void pxa_ssp_dma_tx_irq(int channel, void *p, struct pt_regs *regs);
#endif

/*!
 * @brief Prints the contents of the transmit of receive vectors.
 *
 * This function is included for debugging purposes only.  It is not included
 * in non debug builds.
 *
 * @param vec_p Pointer to the vector array to print from.
 * @param count The number of vectors in the vector array.
 * @param tx    true if the tx entries are to be printed, false for rx.
 */
#ifdef POWER_IC_PXA_SSP_DEBUG_SPI
static void pxa_ssp_print_vectors(SPI_QUEUE_NODE_T *node_p, unsigned int count, bool tx)
{
    static const char hex[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                               '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
    SPI_IOVEC_T *vec_p;
    unsigned int i;
    unsigned int j;
    unsigned int k;
    u8 *base_p;
    char tmp[61];
    char *tmp_p;

    if (node_p->params.cs != POWER_IC_PXA_SSP_DEBUG_SPI)
    {
        return;
    }
    vec_p = node_p->vectors;
    if (tx)
    {
        tracemsg(_k_d("Transmitting sequence %d over SPI cs %d:"), node_p->sequence, node_p->params.cs);
    }
    else
    {
        tracemsg(_k_d("Received sequence %d over SPI cs %d:"), node_p->sequence, node_p->params.cs);
    }
    for (i = 0; i < count; i++)
    {
        base_p = (u8 *)vec_p[i].rx_base;
        if (tx)
        {
            base_p = (u8 *)vec_p[i].tx_base;
        }
        tmp_p = tmp;
        k = 0;
        for (j = 0; j < vec_p[i].iov_len; j++)
        {
            if ((j != 0 ) && (j%20 == 0))
            {
                *tmp_p = 0x00;
                tracemsg(_k_d("%d: %s"), i, tmp);
                tmp_p = tmp;
            }
            *tmp_p++ = hex[(*base_p)>>4];
            *tmp_p++ = hex[(*base_p++)&0x0f];
            *tmp_p++ = ' ';
        }
        *tmp_p = 0x00;
        tracemsg(_k_d("%d: %s"), i, tmp);
    }
}
#else
# define pxa_ssp_print_vectors(vec_p, count, tx)
#endif

/*!
 * @brief Sets up the transmit and receive descriptor chains for DMA
 *
 * Given a list of memory blocks, sets up the DMA descriptor chains for
 * both transmit and receive DMA.  See the PXA DMA spec for more information
 * on descriptor chains.
 *
 * @param node_p  Pointer to the SPI transaction node for which the
 *                transmit must be initiated.  This must be valid,
 *                no checking is done before it is used.
 *
 * @note The DMA descriptor memory must be in physical memory, and must
 *       not be cached by the ARM core.  If it is cached and the cache is
 *       not flushed to physical memory, the DMA will not work, as it reads
 *       the physical memory.
 *
 * @return 0 upon success.<BR>
 *         -EINVAL if no vectors are specified in the node or not enough
 *                 space in pxa_ssp_dma_descriptors for all of the vectors.<BR>
 *         -EFBIG  Size of one of the vectors is larger than can be handled
 *                 by the hardware.<BR>
 */
static int pxa_ssp_setup_dma_desc(SPI_QUEUE_NODE_T *node_p)
{
    unsigned int total_size = 0;
    size_t i;

    if (node_p->count == 0)
    {
        tracemsg(_k_w("pxa_ssp_setup_dma_desc: No vectors specified."));
        return -EINVAL;
    }
    if (node_p->count >= sizeof(pxa_ssp_dma_descriptors.rx)/sizeof(pxa_ssp_dma_descriptors.rx[0]))
    {
        tracemsg(_k_w("pxa_ssp_setup_dma_desc: Not enough descriptors available."));
        return -EINVAL;
    }
    for (i=0; i<node_p->count; i++)
    {
        if (node_p->vectors[i].iov_len > (1<<13))
        {
            tracemsg(_k_w("pxa_ssp_setup_dma_desc: Vector is larger than hardware DMA can handle."));
            return -EFBIG;
        }
        /*
         * Set up for receive as follows:
         *   No interrupt on complete (handled by stop interrupt in dcsr),
         *   increment only the target address,
         *   source flow control,
         *   burst size of 8 bytes,
         *   1 byte for the width,
         *   set the receive size
         */
        pxa_ssp_dma_descriptors.rx[i].ddadr = __pa(&pxa_ssp_dma_descriptors.rx[i+1]);
        pxa_ssp_dma_descriptors.rx[i].dsadr = __PREG(SSDR); /* Set the source address to the transmit FIFO. */
        pxa_ssp_dma_descriptors.rx[i].dtadr = __pa(node_p->vectors[i].rx_base); /* Set the source data physical address. */
        pxa_ssp_dma_descriptors.rx[i].dcmd = (DCMD_INCTRGADDR | DCMD_FLOWSRC | DCMD_BURST8 |
                                              DCMD_WIDTH1 | node_p->vectors[i].iov_len);

        /*
         * Set up for transmit as follows:
         *   No interrupt upon transmit complete,
         *   increment only the source address,
         *   target flow control,
         *   burst size of 8 bytes,
         *   1 byte for the width,
         *   set the transmit size
         */
        pxa_ssp_dma_descriptors.tx[i].ddadr = __pa(&pxa_ssp_dma_descriptors.tx[i+1]);
        pxa_ssp_dma_descriptors.tx[i].dtadr = __PREG(SSDR); /* Set the destination address to the transmit FIFO. */;
        pxa_ssp_dma_descriptors.tx[i].dsadr = __pa(node_p->vectors[i].tx_base); /* Set the source data physical address. */
        pxa_ssp_dma_descriptors.tx[i].dcmd = (DCMD_INCSRCADDR | DCMD_FLOWTRG | DCMD_BURST8 |
                                              DCMD_WIDTH1 | node_p->vectors[i].iov_len);
        DEBUG_MSG(_k_d("Descriptor %d: rx.ddadr:%08x rx.dsadr:%08x rx.dtadr:%08x rx.dcmd:%08x tx.ddadr:%08x tx.dsadr:%08x tx.dtadr:%08x tx.dcmd:%08x"), i,
                  pxa_ssp_dma_descriptors.rx[i].ddadr, pxa_ssp_dma_descriptors.rx[i].dsadr,
                  pxa_ssp_dma_descriptors.rx[i].dtadr, pxa_ssp_dma_descriptors.rx[i].dcmd,
                  pxa_ssp_dma_descriptors.tx[i].ddadr, pxa_ssp_dma_descriptors.tx[i].dsadr,
                  pxa_ssp_dma_descriptors.tx[i].dtadr, pxa_ssp_dma_descriptors.tx[i].dcmd);
        /* Clean and invalidate the transmit and receive buffers in case they are in the processor cache. */
        flush_dcache_range((int)node_p->vectors[i].rx_base, (int)node_p->vectors[i].rx_base+node_p->vectors[i].iov_len);
        flush_dcache_range((int)node_p->vectors[i].tx_base, (int)node_p->vectors[i].tx_base+node_p->vectors[i].iov_len);

        total_size +=  node_p->vectors[i].iov_len;
    }
    /* Reset the last descriptor to be an end of chain. */
    pxa_ssp_dma_descriptors.rx[i-1].ddadr = DDADR_STOP;
    pxa_ssp_dma_descriptors.tx[i-1].ddadr = DDADR_STOP;
    
    /* Clean and invalidate the descriptors in case they are in the processor cache. */
    flush_dcache_range((int)&pxa_ssp_dma_descriptors,
                       (int)&pxa_ssp_dma_descriptors+sizeof(pxa_ssp_dma_descriptors));
    DDADR(pxa_ssp_xfr_info.rx_dma_chan) = __pa(pxa_ssp_dma_descriptors.rx);
    DDADR(pxa_ssp_xfr_info.tx_dma_chan) = __pa(pxa_ssp_dma_descriptors.tx);
    return total_size;
}

/*!
 * @brief Send and receive all data requested without DMA.
 *
 * Transmits and receives all of the data within the vectors for the node
 * pointer which is passed in.  The data is sent in a loop and no interrupts
 * or DMA is used.
 *
 * @param node_p  Pointer to the SPI transaction node for which the
 *                transmit must be initiated.  This must be valid,
 *                no checking is done before it is used.
 *
 * @pre It is assumed that the SSP will be enabled and the communication
 * parameters set up.
 */
void pxa_ssp_manual_tx_rx(SPI_QUEUE_NODE_T *node_p)
{
    unsigned int i;
    unsigned int j;
    unsigned int sent;
    uint8_t *tx_p;
    uint8_t *rx_p;

    /* Loop through all of the vectors. */
    for (i=0; i<node_p->count; i++)
    {
        rx_p = node_p->vectors[i].rx_base;
        tx_p = node_p->vectors[i].tx_base;
        j = node_p->vectors[i].iov_len;
        /*
         * Prime the queue with up to half of the bytes which can be sent (8).
         */
        sent = 8;
        if (j < sent)
        {
            sent = j;
        }
        j -= sent;
        while (sent--)
        {
            SSDR = *tx_p++;
        }
        /*
         * Send the remaining bytes.
         */
        while (j > 0)
        {
            /*
             * Add 8 more bytes to the queue, since 8 are read below.  This
             * is done so that the Tx queue is being loaded while data is
             * still being sent.  This maximizes the speed at which the data
             * is transfered.
             */
            sent = 8;
            if (j < sent)
            {
                sent = j;
            }
            j -= sent;
            /*
             * No need to top test here since j will have to be greater than
             * zero before this code can execute.
             */
            do
            {
                SSDR = *tx_p++;
            } while (--sent);
            /*
             * If not done transmitting wait for at least 8 bytes to be received
             * and then read them in.  Only the last transmit will have less than
             * 8 bytes to receive.  In this case j will be 0, so the loop below
             * will not execute.  The data will be read in later.
             */
            if (j == 0)
            {
                break;
            }
            while ((GET_SSSR_RFL < 7) || !(SSSR&NSSSR_RNE));
            sent = 8;
            do
            {
                *rx_p++ = (uint8_t)SSDR;
            } while (--sent);
        }
        /*
         * Wait for the transfer to complete. 
         */
        while ((GET_SSSR_TFL != 0x0) || (SSSR&NSSSR_BSY));
        /*
         * Read the remaining bytes.
         */
        do
        {
            *rx_p++ = (uint8_t)SSDR;
        } while (SSSR&NSSSR_RNE);
    }
}

/*!
 * @brief Sets the chip select to the requested level.
 *
 * Enables or disables the chip select which will be used for the
 * transfer. The polarity of the chip select is based on values in
 * the table pxa_ssp_cs_info.
 *
 * @param cs      The chip select which must be activated or deactivated.
 * @param enable  true when the chip select must be activated.
 *                false when the chip select must be deactivated.
 *
 * @return 0 upon success<BR>
 *         -EINVAL when the chip select is out of range of the table<BR>
 */
static int pxa_ssp_set_cs(SPI_CS_T cs, bool enable)
{
    int16_t gpio;
    
    DEBUG_MSG(_k_d("pxa_ssp_set_cs %d %d"), cs, enable);
    if (cs >= SPI_CS__END)
    {
        DEBUG_MSG(_k_d("pxa_ssp_set_cs - Invalid CS number"));
        return -EINVAL;
    }
    /* Only update the chip select if it is not locked. */
    if (!cs_locked)
    {
        gpio = pxa_ssp_cs_info[cs].gpio;
        /*
         * Only change the state of the GPIO if it is valid.  This is done to
         * support the dummy chip select.
         */
        if (gpio >= 0)
        {
            if (pxa_ssp_cs_info[cs].active_high == enable)
            {
                GPSR(gpio) = GPIO_bit(gpio);
            }
            else
            {
                GPCR(gpio) = GPIO_bit(gpio);
            }
        }
    }
    return 0;
}

#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
/* recore current lock chip */
static SPI_CS_T cur_lock_cs = -1;

#define  DATA_SIZE_32Bit                  0x10000F
#define  START_Transceive                 0x80

extern void enter_idle(void);
extern void enter_dma(void);
extern int spi_int_schedule(void);
static int pxa_ssp_rx_complete(SPI_QUEUE_NODE_T *node_p);
extern spinlock_t spi_bus_int_lock;
#endif  /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */



/*!
 * @brief Sets up SSP and DMA then starts the transfer.
 *
 * This function sets up and starts the SSP transfer.  It can be called
 * from an interrupt to set up a transfer of the next node.  The routine
 * will determine if it is best to simply wait for the transmit to complete
 * or if an interrupt is best to handle the completion.  If the transmit will
 * take less than PXA_POLL_RESULT_US, the routine will indicate waiting is
 * best, but if greater the DMA transmit complete interrupt will be enabled
 * to handle the completion of the receive.
 *
 * @param node_p    Pointer to the SPI transaction node for which the
 *                  transmit must be initiated.  This must be valid,
 *                  no checking is done before it is used.
 * @param no_wait   true when the calling routine cannot wait for the
 *                  transmit to complete.  In general this should be true
 *                  if called from an interrupt context.
 *
 * @return 0 upon success and no need to wait for the transmit to complete,
 *           since the DMA receive complete interrupt has been enabled.<BR>
 *         1 upon success, but the calling routine must wait for the
 *           transmit to complete, since the  DMA receive complete
 *           interrupt was not enabled.<BR>
 *         -EINVAL data contained within the node pointer is incorrect.<BR>
 *
 * @pre node_p must be a valid pointer to a valid transaction queue node.
 */
int pxa_ssp_initiate_tx(SPI_QUEUE_NODE_T *node_p, bool no_wait)
{
 #ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
    enter_dma();

    if( (node_p->params.cs == SPI_CS_PCAP)
      &&(node_p->count == 1)
      &&(node_p->vectors[0].iov_len ==4)  )
    {
     
      u32 tempVal;
      u32 sendVal;
      u32 receiveVal;
      
      u32 spi_bus_int_lock_flag;
      spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);

      /*
       * Prepare for transfer
       *
       * Disable the SSP and it's clock until a transmit is ready to go out.
       * Set the 32 bit data size
       */
      CKEN &= ~CKEN23_SSP1;
      SSCR0 = 0|DATA_SIZE_32Bit;

      SSCR1 = 0;
      if (node_p->params.clk_idle_state)
      {
        SSCR1 |= NSSCR1_SPO;
      }
      if (node_p->params.clk_phase)
      {
        SSCR1 |= NSSCR1_SPH;
      }

      /* Enable the SSP clock. */
      CKEN |= CKEN23_SSP1;

      /* Now that the clock polarity has changed enable the chip select. */
      pxa_ssp_set_cs(node_p->params.cs, true);

      /* Enable the SSP port operations */
      SSCR0 |= START_Transceive;

      /* maybe it need to wait for the clock enable! */

      /* Empty the receive FIFO first! */
      while((( (SSSR) & (0x1 << 3) )!=0))
      {
        tempVal = (SSDR);
      }

      /* send    32bit data */
      sendVal = __be32_to_cpu(*(u32 *)node_p->vectors[0].tx_base);
      SSDR = __be32_to_cpu(*(u32 *)node_p->vectors[0].tx_base); 
      /*
       * Wait until receive data ready!
       * This 32bit operations will finish no more than 1000 us.
       */
      int udelay_count = 0;
      while((((SSSR) & (0x1 << 3))==0) && (udelay_count< 1000))
      {
        udelay(1);
        udelay_count ++;
      }
      /* Transfer ok, Receive data ready! */
      *(u32 *)(node_p->vectors[0].rx_base) = __cpu_to_be32(SSDR);
      receiveVal = __be32_to_cpu(*(u32 *)node_p->vectors[0].rx_base);

      /* Receive all trailling data */
      while(((SSSR) & (0x1 << 3))!=0)
      {
        tempVal = (SSDR);
      }

      /* 
       * We doesn't need to care the chip select here.
       * It will be handle in function pxa_ssp_rx_complete.
       */
      /* Disable the Chip Enable GPIO */
      /* pxa_ssp_set_cs(node_p->params.cs, false); */

      /* 
       * We doesn't need to stop spi here, because it will stop 
       * in function pxa_ssp_rx_complete while there is no more SPI operations 
       */
      /* Disable the SSP port operations */
      /* SSCR0 &= ~START_Transceive; */

      /* Stop the spi */
      /* CKEN &= ~CKEN23_SSP1; */ 
      
      spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);
 
      (void)pxa_ssp_rx_complete(node_p);

      return 0;
    }
#endif  /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */

    unsigned long irq_flags;
    unsigned int timeout;
    unsigned int total_to_transmit;
    bool wait_for_rx;
    int error;

    DEBUG_MSG(_k_d("pxa_ssp_initiate_tx %d node_p %p rx_dma_chan %d tx_dma_chan %d"),
              no_wait, node_p, pxa_ssp_xfr_info.rx_dma_chan, pxa_ssp_xfr_info.tx_dma_chan);
    pxa_ssp_print_vectors(node_p, node_p->count, true);

    /* Set up a pointer to the data for when the interrupt fires.  */
    pxa_ssp_xfr_info.node_p = node_p;
    
    /* Enable the SSP clock. */
    CKEN |= CKEN23_SSP1;

    /*
     * Set up the variable communication parameters for this transfer.  The SSP is disabled
     * by this write.  This is done in order to force a DMA trigger after the DMA is set up
     * in the case that the SSP was already activated when this function is called.
     */
    timeout = (NSSCR0_SCR_MSK>>NSSCR0_SCR_SFT)+1;
    if (node_p->params.speed != 0)
    {
        timeout = (SSP_MAX_CLK-1)/node_p->params.speed;
    }
    if (timeout > NSSCR0_SCR_MSK>>NSSCR0_SCR_SFT)
    {
        tracemsg(_k_w("pxa_ssp_initiate_tx - Unable to achieve %d Hz setting to %d Hz."),
                 node_p->params.speed, SSP_MAX_CLK/(NSSCR0_SCR_MSK>>NSSCR0_SCR_SFT));
        timeout = NSSCR0_SCR_MSK>>NSSCR0_SCR_SFT;
    }
    SSCR0 = ((timeout<<NSSCR0_SCR_SFT) | NSSCR0_TIM | NSSCR0_RIM | NSSCR0_DSS_8bit);
    
    /*
     * Set the thresholds from which to trigger a DMA request.  The value in the register
     * is one less than the value desired (0 in the register is 1 byte).
     */
    SSCR1 = (NSSCR1_TRAIL | (7 << NSSCR1_RFT_SFT) | (7 << NSSCR1_TFT_SFT));
                         
    if (node_p->params.clk_idle_state)
    {
        SSCR1 |= NSSCR1_SPO;
    }
    if (node_p->params.clk_phase)
    {
        SSCR1 |= NSSCR1_SPH;
    }

    /*
     * This will set the TFS flag which will allow the DMA transfer to start as soon as
     * the RUN flag is set.  It would make more sense to enable this after setting RUN, but
     * in order for the clock polity to change before setting the chip select this bit must
     * be set.  It is done before the descriptor chain is set up to allow for the necessary
     * SSP start up time.
     */
    SSCR0 |= NSSCR0_SSE;

    /*
     * Set up a descriptor chain for both transmit and receive for all entries in the vectors.
     * Before this can be done clear any pending interrupts and clear the RUN bit.
     */
    DCSR(pxa_ssp_xfr_info.rx_dma_chan) = DCSR_ENRINTR|DCSR_ENDINTR|DCSR_STARTINTR|DCSR_BUSERR;
    DCSR(pxa_ssp_xfr_info.tx_dma_chan) = DCSR_ENRINTR|DCSR_ENDINTR|DCSR_STARTINTR|DCSR_BUSERR;
    total_to_transmit = pxa_ssp_setup_dma_desc(node_p);
    
    /*
     * Determine if it is more efficient to simply wait for the receive.  The wait will happen
     * if the transmit will complete in less than PXA_POLL_RESULT_US.
     * Time for transmit =
     *     (1/(SSP_MAX_CLK)/timeout)*total_to_transmit*8) < PXA_POLL_RESULT_US/1000000us/S
     *  => ((timeout/SSP_MAX_CLK)*total_to_transmit*8)*1000000 < PXA_POLL_RESULT_US
     *  => (timeout*total_to_transmit*8000000)/SSP_MAX_CLK < PXA_POLL_RESUT_US
     *  => (timeout*total_to_transmit*8000000 < PXA_POLL_RESULT_US*SSP_MAX_CLK
     *  => timeout*total_to_transmit < (PXA_POLL_RESULT_US*SSP_MAX_CLK)/8000000
     * Adjust for u32 math
     *  => timeout*total_to_transmit < (PXA_POLL_RESULT_US*(SPP_MAX_CLK/1000000))/8
     * example with values
     *  => timeout*total_to_transmit < (500*13)/8
     * But a timeout is 0 based so one must be added in the calculation.
     */
    wait_for_rx = ((!no_wait) && ((timeout+1)*total_to_transmit) < (PXA_POLL_RESULT_US*(SSP_MAX_CLK/1000000))/8);
    DEBUG_MSG(_k_d("wait_for_rx = %d based on %d < %d"),
              wait_for_rx, (timeout+1)*total_to_transmit, (PXA_POLL_RESULT_US*(SSP_MAX_CLK/1000000))/8);

    /* Allow alignment on any addresses for the transfer. */
    DALGN |= (1<<pxa_ssp_xfr_info.rx_dma_chan)|(1<<pxa_ssp_xfr_info.tx_dma_chan);

    /*
     * Now that the clock polarity has changed enable the chip select.
     */
    error = pxa_ssp_set_cs(node_p->params.cs, true);
    if (error < 0)
    {
        SSCR0 = 0;
        return (error);
    }

    /*
     * If waiting for the receive, simply send the data manually without DMA.  This is done since
     * it will be faster than waiting on the DMA in heavy use cases.
     */
    if (wait_for_rx)
    {
        pxa_ssp_manual_tx_rx(node_p);
        return 1;
    }

    /* Disable interrupts until all flags have been set. */
    local_irq_save(irq_flags);
    
    /* Transfer with a descriptor chain, the transfer will start immediately since SSSR[TFS] is set. */
    DCSR(pxa_ssp_xfr_info.rx_dma_chan) = DCSR_RUN;
    DCSR(pxa_ssp_xfr_info.tx_dma_chan) = DCSR_RUN;
    
    /* Enable the receive complete interrupt now that the channels are running. */
    DCSR(pxa_ssp_xfr_info.rx_dma_chan) |= DCSR_STOPIRQEN;

    /*
     * Start the transmit DMA by enabling the SSP which will result in the TFS flag being
     * set in the SSSR which will trigger the transmit DMA.
     */
    SSCR1 |= NSSCR1_TSRE | NSSCR1_RSRE;

    /*
     * Convert timeout to an actual clock rate in MHz so that the trailing byte timeout can
     * be calculated.  Then calculate the time in peripheral clock cycles for one byte
     * to be transmitted.  This plus 2 bit times will become the timeout.   If the byte
     * size is changed or additional inter byte delays are added this will need to change.
     */
    timeout = (PXA_PERIPHERAL_CLK/SSP_MAX_CLK)*(timeout+1)*(8+2);
    
    /* Set the receive idle timeout to catch trailing bytes. */
    SSTO = timeout&NSSTO_MSK;
    
    /* Re-enable interrupts. */
    local_irq_restore(irq_flags);
    DEBUG_PRINT_REGS("pxa_ssp_initiate_tx: Transmit started.\n");
    return 0;
}       

/*!
 * @brief Handles operations when a receive is completed
 *
 * Handles the disabling of the chip select upon the completion of a receive
 * operation.  If more data is pending the transmit will be started.  If
 * nothing is pending, the SSP and DMA will be shutdown.
 *
 * @param node_p Pointer to the transaction queue node for which the
 *               receive was just completed.
 *
 * @return 0 upon success.<BR>
 *         -EINVAL when the chip select is out of range of the table.<BR>
 */
static int pxa_ssp_rx_complete(SPI_QUEUE_NODE_T *node_p)
{
    SPI_QUEUE_NODE_T *next_tx_p;
    
    DEBUG_MSG(_k_d("pxa_ssp_rx_complete: node_p %p"), node_p);
    pxa_ssp_print_vectors(node_p, node_p->count, false);

    /*
     * Loop through any 0 length nodes if any are found.  The 0 length nodes are
     * used to reset the chip select if it is locked.
     */
    do
    {
        /* Disable the chip select if not transferring a chain messages. */
        if (!node_p->params.lock_cs)
        {
            (void)pxa_ssp_set_cs(node_p->params.cs, false);
#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
            cur_lock_cs = -1;
        }
        else
        {
            cur_lock_cs = node_p->params.cs;
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */
        }

        /* Indicate the transceive of the node has been completed. */
        next_tx_p = spi_tx_completed(node_p);
    } while ((next_tx_p != NULL) && (next_tx_p->count == 0));

#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
    enter_idle();
    spi_int_schedule();
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */
    
    /* If another node is ready for transmission, start to send it. */
    if (next_tx_p != NULL)
    {
        (void)pxa_ssp_initiate_tx(next_tx_p, true);
    }
    else
    {
        pxa_ssp_xfr_info.in_progress = 0;
        /* Reset the alignment to be on 64 bit alignment. */
        DALGN &= ~((1<<pxa_ssp_xfr_info.rx_dma_chan)|(1<<pxa_ssp_xfr_info.tx_dma_chan));
        /* The SSP RX and TX no longer need to be mapped to these DMA channels.  */
        DRCMRRXSSDR = 0;
        DRCMRTXSSDR = 0;
        /* No more to transmit, free the dma channels. */
        pxa_free_dma(pxa_ssp_xfr_info.tx_dma_chan);
        pxa_free_dma(pxa_ssp_xfr_info.rx_dma_chan);
        /* Shut down the SSP to save current. */
        SSCR0 = 0;
        SSCR1 = 0; /* Clear the DMA flags. */
        CKEN &= ~CKEN23_SSP1;
    }
    return 0;
}

/*!
 * @brief DMA receive complete interrupt.
 *
 * When called a transceive has been completed.  If another transfer is in
 * the queue it will be started.
 *
 * @param channel The DMA channel number for which the interrupt is for.  
 * @param p       Not used in this function
 * @param regs    Not used in this function
 */
static void pxa_ssp_dma_rx_irq(int channel, void *p, struct pt_regs *regs)
{
    DEBUG_PRINT_REGS("pxa_ssp_dma_rx_irq");
    /* Do nothing if this interrupt is not for a receive complete. */
    if (channel != pxa_ssp_xfr_info.rx_dma_chan)
    {
        return;
    }
    (void)pxa_ssp_rx_complete(pxa_ssp_xfr_info.node_p);
    
    /* Clear the RX complete interrupt. */
    DCSR(pxa_ssp_xfr_info.rx_dma_chan) |= DCSR_ENRINTR|DCSR_ENDINTR|DCSR_STARTINTR|DCSR_BUSERR;
}

/*!
 * @brief DMA transmit complete interrupt.
 *
 * The transmit complete interrupt will be called when the last of the data
 * has been copied into the transmit FIFO.  Nothing is done by this interrupt
 * since the transmission complete is handled by the rx dma interrupt.  The
 * API for requesting a dma channel requires an interrupt, so this is it.
 *
 * @param channel The DMA channel number for which the interrupt is for.  
 * @param p       Not used in this function
 * @param regs    Not used in this function
 */
static void pxa_ssp_dma_tx_irq(int channel, void *p, struct pt_regs *regs)
{
    DEBUG_PRINT_REGS("pxa_ssp_dma_tx_irq");
    /*
     * Abort the transfer, since this must be some sort of error (bus error),
     *  since the other interrupts are never enabled.
     */
    if (channel == pxa_ssp_xfr_info.tx_dma_chan)
    {
        /* Indicate an error occurred. */
        pxa_ssp_xfr_info.node_p->error = -EIO;
        /*
         * If the receive interrupt is enabled, the code is not polling for the
         * receive, so it must be aborted.  If the transmit did not happen, the
         * receive will not either.
         */
        if (DCSR(pxa_ssp_xfr_info.rx_dma_chan) & DCSR_STOPIRQEN)
        {
            /* Disable the receive interrupt, since it will not happen. */
            DCSR(pxa_ssp_xfr_info.rx_dma_chan) &= ~DCSR_STOPIRQEN;
            /* Abort the transfer. */
            (void)pxa_ssp_rx_complete(pxa_ssp_xfr_info.node_p);
        }
        /* Clear the TX complete interrupt. */
        DCSR(pxa_ssp_xfr_info.tx_dma_chan) |= DCSR_ENRINTR|DCSR_ENDINTR|DCSR_STARTINTR|DCSR_BUSERR;
    }
}

/******************************************************************************
* Global functions
******************************************************************************/

/*!
 * @brief Starts a SPI transfer on the Bulverde SSP.
 *
 * Externally called function to start a transmission of a node.  It must only
 * start a SPI transmission, if one is not already in progress.  If one is in progress
 * nothing needs to be done, since the next node on the queue will be transmitted
 * when the current transmit completes.
 *
 * @param node_p Pointer to the node in the transmit queue which must be sent
 *
 * @return 0 upon success and the data has been transmitted.<BR>
 *         1 upon success, but the transmit has yet to complete.<BR>
 *         -ENODEV if DMA channels could not be allocated to send the data.<BR>
 *         -EINVAL the node pointer or the data contained within the node is incorrect.<BR>
 */
int pxa_ssp_start_tx(SPI_QUEUE_NODE_T *node_p)
{
    int error;
    
    DEBUG_MSG(_k_d("pxa_ssp_start_tx"));
    if (node_p == NULL)
    {
        tracemsg(_k_w("pxa_ssp_start_tx: Null node pointer."));
        return -EINVAL;
    }

    /*
     * If a transmit is in progress, return without sending the node.
     */
    if (pxa_ssp_xfr_info.in_progress)
    {
        DEBUG_MSG(_k_d("pxa_ssp_start_tx - Write in progress, will transmit when completed."));
        return 1;
    }

    /*
     * Request a transmit and a receive DMA channel and update the DMA Channel map
     * registers with the channel numbers.
     */
    pxa_ssp_xfr_info.in_progress = true;
    pxa_ssp_xfr_info.rx_dma_chan = pxa_request_dma("SSP1 rx", DMA_PRIO_MEDIUM, pxa_ssp_dma_rx_irq, (void *)node_p);
    if (pxa_ssp_xfr_info.rx_dma_chan < 0)
    {
        tracemsg(_k_w("pxa_ssp_start_tx: Error requesting RX DMA channel."));
        return pxa_ssp_xfr_info.rx_dma_chan;
    }
    pxa_ssp_xfr_info.tx_dma_chan = pxa_request_dma("SSP1 tx", DMA_PRIO_MEDIUM, pxa_ssp_dma_tx_irq, (void *)node_p);
    if (pxa_ssp_xfr_info.tx_dma_chan < 0)
    {
        tracemsg(_k_w("pxa_ssp_start_tx: Error requesting TX DMA channel."));
        pxa_free_dma(pxa_ssp_xfr_info.rx_dma_chan);
        return pxa_ssp_xfr_info.tx_dma_chan;
    }
        
    /* Set up the DMA channels for use with the SSP. */
    DRCMRRXSSDR = DRCMR_MAPVLD | pxa_ssp_xfr_info.rx_dma_chan;
    DRCMRTXSSDR = DRCMR_MAPVLD | pxa_ssp_xfr_info.tx_dma_chan;

    /*
     * If this is an empty node, simply update the chip select and check for more nodes
     * to transmit.  This must be done after the DMA channels are setup in case another node
     * follows this node.  In that case the transmit of the node which follows the chip select
     * release node must have it's transmit started.
     */
    if (node_p->count == 0)
    {
        return pxa_ssp_rx_complete(node_p);
    }
    
    error = pxa_ssp_initiate_tx(node_p, false);
    if (error < 0)
    {
        DRCMRRXSSDR = 0;
        DRCMRTXSSDR = 0;
        pxa_free_dma(pxa_ssp_xfr_info.rx_dma_chan);
        pxa_free_dma(pxa_ssp_xfr_info.tx_dma_chan);
        tracemsg(_k_w("pxa_ssp_start_tx: Unable to initiate transfer error %d"), error);
        return error;
    }
    if (error)
    {
        (void)pxa_ssp_rx_complete(node_p);
    }

#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
    if(error==0)
    {
      return 0;
    }
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */

    /* Indicate the caller must wait for the transfer to complete. */     
    return 1;
}

/*!
 * @brief Returns true if a transfer is in progress
 *
 * Returns true when a transfer is in progress and false when the driver is
 * idle.  
 *
 * @return true if a transfer is in progress
 *         false if the driver is idle
 */
bool pxa_ssp_tx_in_progress(void)
{
    return (pxa_ssp_xfr_info.in_progress);
}

/*!
 * @brief Initialize the ssp ports used in SPI mode.
 *
 * This routine initializes the SSP hardware as well as the GPIO needed for SPI
 * transmits over the SSP.  The chip select ports will all be disabled by this
 * init routine.
 */
void __init pxa_ssp_initialize(void)
{
    SPI_CS_T cs;
    int16_t gpio;
    

    DEBUG_MSG(_k_d("pxa_ssp_initialize"));

    /* Start off with an unlocked chip select. */
    cs_locked = false;
    
    /* Set the GPIO muxes for CLK, MOSI and MISO. */
    set_GPIO_mode(GPIO_SPI_CLK|GPIO_ALT_FN_3_OUT);
    set_GPIO_mode(GPIO_SPI_MOSI|GPIO_ALT_FN_2_OUT);
    set_GPIO_mode(GPIO_SPI_MISO|GPIO_ALT_FN_1_IN);

    /* Set up all but the dummy chip select. */
    for (cs=0; cs < SPI_CS_DUMMY; cs++)
    {
        gpio = pxa_ssp_cs_info[cs].gpio;
        set_GPIO_mode(gpio|GPIO_OUT);
        pxa_ssp_set_cs(cs, false);
        /* The PGSR will be loaded with the inactive states for the GPIO to be 
           used since these settings are used during sleep and deep-sleep mode.
           The active states can found in the pxa_ssp_cs_info Table.  */  
        if (pxa_ssp_cs_info[cs].active_high)
        {
            PGSR(gpio) &= ~GPIO_bit(gpio);
        }
        else
        {
            PGSR(gpio) |= GPIO_bit(gpio);
        }
    }
    
    /*
     * The SPI_MOSI line has to be high during sleep and deep-sleep mode
     * to avoid current consumption through a 47k pull-up.
     */
    PGSR(GPIO_SPI_MOSI) |= GPIO_bit(GPIO_SPI_MOSI);
    
    /*
     * Disable the SSP and it's clock until a transmit is ready to go out.
     */
    SSCR0 = 0;
    CKEN &= ~CKEN23_SSP1;
}

#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
/*!
 * @brief Sets the chip select to the requested level. 
 *
 * This function ignore variable cs_locked.
 * Enables or disables the chip select which will be used for the
 * transfer. The polarity of the chip select is based on values in
 * the table pxa_ssp_cs_info.
 *
 * @param cs      The chip select which must be activated or deactivated.
 * @param enable  true when the chip select must be activated.
 *                false when the chip select must be deactivated.
 *
 * @return 0 upon success<BR>
 *         -EINVAL when the chip select is out of range of the table<BR>
 */
static int spi_int_set_cs(SPI_CS_T cs, bool enable)
{
    int16_t gpio;
    
    DEBUG_MSG(_k_d("spi_int_set_cs %d %d"), cs, enable);
    if (cs >= SPI_CS__END)
    {
        DEBUG_MSG(_k_d("spi_int_set_cs - Invalid CS number"));
        return -EINVAL;
    }

    gpio = pxa_ssp_cs_info[cs].gpio;
    /*
     * Only change the state of the GPIO if it is valid.  This is done to
     * support the dummy chip select.
     */
    if (gpio >= 0)
    {
        if (pxa_ssp_cs_info[cs].active_high == enable)
        {
            GPSR(gpio) = GPIO_bit(gpio);
        }
        else
        {
            GPCR(gpio) = GPIO_bit(gpio);
        }
    }

    return 0;
}


/*!
 * @brief transfer 32 bit data on SPI bus, even in interrupt context.
 *
 * In order to process SPI bus in interrupt context, we need to transfer data 
 * on SPI bus without DMA. And this function only support PCAP operations.
 *
 * @param     tx_p  32 bits data to send.
 * @param     rx_p  32 bits data to receive.
 *
 * @return    0 upon success<BR>
 */
int spi_int_transceive(u32 *tx_p, u32 *rx_p)
{
        u32 spi_bus_int_lock_flag;
        spin_lock_irqsave(&spi_bus_int_lock, spi_bus_int_lock_flag);

        u32 tempVal;

        /*
         * Prepare for transfer
         * Disable the SSP and it's clock until a transmit is ready to go out.
         * Set the 32 bit data size
         */
        CKEN &= ~CKEN23_SSP1;
        SSCR0 = 0|DATA_SIZE_32Bit;

        SSCR1 = 0;
        /*
         * Because we only support PCAP operations, so SSCR1=0 is OK.
         * No more check for clk_idle_state and clk_phase
         */
        /*
        if (node_p->params.clk_idle_state)
        {
          SSCR1 |= NSSCR1_SPO;
        }
        if (node_p->params.clk_phase)
        {
          SSCR1 |= NSSCR1_SPH;
        }
        */

        /* Enable the SSP clock. */
        CKEN |= CKEN23_SSP1;

        /*
         * Now that the clock polarity has changed enable the chip select.
         * pxa_ssp_set_cs(node_p->params.cs, true);
         */
        if(cur_lock_cs != SPI_CS_PCAP)
        {
                spi_int_set_cs(cur_lock_cs, false);
                spi_int_set_cs(SPI_CS_PCAP, true);
        }

        /* Enable the SSP port operations */
        SSCR0 |= START_Transceive;

        /* Empty the receive FIFO first! */
        while((( (SSSR) & (0x1 << 3) )!=0))
        {
                tempVal = (SSDR);
        }

        /* send    32bit data */
        SSDR = *tx_p;
        /* Wait until receive data ready! */
        /* This 32bit operations will finish no more than 1000 us. */
        int udelay_count = 0;
        while((((SSSR) & (0x1 << 3))==0) && (udelay_count< 1000))
        {
                udelay(1);
                udelay_count ++;
        }

        /* receive 32 bit data */
        * rx_p = SSDR;

        /* Receive all trailling data ok! */
        while(((SSSR) & (0x1 << 3))!=0)
        {
                tempVal = (SSDR);
        }

        /* Disable the Chip Enable GPIO */
        /* pxa_ssp_set_cs(node_p->params.cs, false); */
        if(cur_lock_cs != SPI_CS_PCAP)
        {
                spi_int_set_cs(cur_lock_cs, true);
                spi_int_set_cs(SPI_CS_PCAP, false);
        }

        /* Disable the SSP port operations */
        SSCR0 &= ~START_Transceive;
        /* Stop the spi */
        CKEN &= ~CKEN23_SSP1;

        spin_unlock_irqrestore(&spi_bus_int_lock, spi_bus_int_lock_flag);

        return 0;
}
#endif /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */
