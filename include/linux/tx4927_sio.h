/*
 * linux/include/linux/tx4927_sio.h
 *
 * tx4927 Serial IO driver (using generic serial)
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * Copyright 2001-2002 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef TX4927_SIO_H
#define TX4927_SIO_H

#include <linux/init.h>
#include <linux/config.h>
#include <linux/tty.h>
#include <linux/major.h>
#include <linux/ptrace.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/serial.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/delay.h>
#include <asm/wbflush.h>
#include <linux/types.h>
#include <linux/serialP.h>
#include <linux/generic_serial.h>
#include <asm/tx4927/tx4927.h>


/******************************************************************************/
/* BEG: H/W Register Locations/Definitions                                    */
/******************************************************************************/


#define TX4927_SIO_BASE_SIO0 ((tx4927_sio_hw_st*)(TX4927_BASE|TX4927_SIO0_BASE))
#define TX4927_SIO_BASE_SIO1 ((tx4927_sio_hw_st*)(TX4927_BASE|TX4927_SIO1_BASE))


#define TX4927_SIO_SILCR                      0x00
#define TX4927_SIO_SILCR_RWUB                 BM_15_15
#define TX4927_SIO_SILCR_TWUB                 BM_14_14
#define TX4927_SIO_SILCR_UODE                 BM_13_13
#define TX4927_SIO_SIDICR_R_07_12             BM_07_12
#define TX4927_SIO_SILCR_SCS_MASK             BM_05_06
#define TX4927_SIO_SILCR_UEPS                 BM_04_04
#define TX4927_SIO_SILCR_UPEN                 BM_03_03
#define TX4927_SIO_SILCR_USBL                 BM_02_02
#define TX4927_SIO_SILCR_UMODE_MASK           BM_00_01
#define TX4927_SIO_SILCR_MULTI                BM_01_01
#define TX4927_SIO_SILCR_UMODE_DATA           BM_00_00


#define TX4927_SIO_SIDICR                     0x04
#define TX4927_SIO_SIDICR_TDE                 BM_15_15
#define TX4927_SIO_SIDICR_RDE                 BM_14_14
#define TX4927_SIO_SIDICR_TIE                 BM_13_13
#define TX4927_SIO_SIDICR_RIE                 BM_12_12
#define TX4927_SIO_SIDICR_SPIE                BM_11_11
#define TX4927_SIO_SIDICR_CTSAC_MASK          BM_09_10
#define TX4927_SIO_SIDICR_CTSAC_NONE          0
#define TX4927_SIO_SIDICR_CTSAC_RISE          BM_09_09
#define TX4927_SIO_SIDICR_CTSAC_FALL          BM_10_10
#define TX4927_SIO_SIDICR_CTSAC_BOTH          BM_09_10
#define TX4927_SIO_SIDICR_R_06_08             BM_06_08
#define TX4927_SIO_SIDICR_STIE                BM_00_05
#define TX4927_SIO_SIDICR_STIE_OERS           BM_05_05
#define TX4927_SIO_SIDICR_STIE_CTSS           BM_04_04
#define TX4927_SIO_SIDICR_STIE_RBRKD          BM_03_03
#define TX4927_SIO_SIDICR_STIE_TRDY           BM_02_02
#define TX4927_SIO_SIDICR_STIE_TXALS          BM_01_01
#define TX4927_SIO_SIDICR_STIE_UBRKD          BM_00_00


#define TX4927_SIO_SIDISR                     0x08
#define TX4927_SIO_SIDISR_UBRK                BM_15_15
#define TX4927_SIO_SIDISR_UVALID              BM_14_14
#define TX4927_SIO_SIDISR_UFER                BM_13_13
#define TX4927_SIO_SIDISR_UPER                BM_12_12
#define TX4927_SIO_SIDISR_UOER                BM_11_11
#define TX4927_SIO_SIDISR_ERI                 BM_10_10
#define TX4927_SIO_SIDISR_TOUT                BM_09_09
#define TX4927_SIO_SIDISR_TDIS                BM_08_08
#define TX4927_SIO_SIDISR_RDIS                BM_07_07
#define TX4927_SIO_SIDISR_STIS                BM_06_06
#define TX4927_SIO_SIDISR_R_05_05             BM_05_05
#define TX4927_SIO_SIDISR_RFDN                BM_00_04


#define TX4927_SIO_SISCISR                    0x0c
#define TX4927_SIO_SISCISR_CTSS               BM_04_04
#define TX4927_SIO_SISCISR_TRDY               BM_02_02
#define TX4927_SIO_SISCISR_TXALS              BM_01_01


#define TX4927_SIO_SIFCR                      0x10
#define TX4927_SIO_SIFCR_SWRST                BM_15_15
#define TX4927_SIO_SIFCR_RDIL                 BM_07_08
#define TX4927_SIO_SIFCR_TDIL                 BM_03_04
#define TX4927_SIO_SIFCR_TFRST                BM_02_02
#define TX4927_SIO_SIFCR_RFRST                BM_01_01
#define TX4927_SIO_SIFCR_FRSTE                BM_00_00


#define TX4927_SIO_SIFLCR                     0x14
#define TX4927_SIO_SIFLCR_RCS                 BM_12_12
#define TX4927_SIO_SIFLCR_TES                 BM_11_11
#define TX4927_SIO_SIFLCR_RTSSC               BM_09_09
#define TX4927_SIO_SIFLCR_RSDE                BM_08_08
#define TX4927_SIO_SIFLCR_TSDE                BM_07_07
#define TX4927_SIO_SIFLCR_TBRK                BM_01_01



#define TX4927_SIO_SIBGR                      0x18
#define TX4927_SIO_SIBGR_BCLK_MASK            BM_08_09
#define TX4927_SIO_SIBGR_BCLK_T0              0
#define TX4927_SIO_SIBGR_BCLK_T2              BM_08_08
#define TX4927_SIO_SIBGR_BCLK_T4              BM_09_09
#define TX4927_SIO_SIBGR_BCLK_T6              BM_08_09
#define TX4927_SIO_SIBGR_BRD_MASK             BM_00_07
#define TX4927_SIO_SIBGR_BRD_000110           222
#define TX4927_SIO_SIBGR_BRD_000150           163
#define TX4927_SIO_SIBGR_BRD_000300            81
#define TX4927_SIO_SIBGR_BRD_000600           163
#define TX4927_SIO_SIBGR_BRD_001200            81
#define TX4927_SIO_SIBGR_BRD_002400           163
#define TX4927_SIO_SIBGR_BRD_004800            81
#define TX4927_SIO_SIBGR_BRD_009600           163
#define TX4927_SIO_SIBGR_BRD_014400           109
#define TX4927_SIO_SIBGR_BRD_019200            81
#define TX4927_SIO_SIBGR_BRD_028800            54
#define TX4927_SIO_SIBGR_BRD_038400            41
#define TX4927_SIO_SIBGR_BRD_057600            27
#define TX4927_SIO_SIBGR_BRD_076800            20
#define TX4927_SIO_SIBGR_BRD_115200            14

#define TX4927_SIO_BAUD_MASK   (TX4927_SIO_SIBGR_BCLK_MASK|TX4927_SIO_SIBGR_BRD_MASK)
#define TX4927_SIO_BAUD_000110 (TX4927_SIO_SIBGR_BCLK_T6|TX4927_SIO_SIBGR_BRD_000110)
#define TX4927_SIO_BAUD_000150 (TX4927_SIO_SIBGR_BCLK_T6|TX4927_SIO_SIBGR_BRD_000150)
#define TX4927_SIO_BAUD_000300 (TX4927_SIO_SIBGR_BCLK_T6|TX4927_SIO_SIBGR_BRD_000300)
#define TX4927_SIO_BAUD_000600 (TX4927_SIO_SIBGR_BCLK_T4|TX4927_SIO_SIBGR_BRD_000600)
#define TX4927_SIO_BAUD_001200 (TX4927_SIO_SIBGR_BCLK_T4|TX4927_SIO_SIBGR_BRD_001200)
#define TX4927_SIO_BAUD_002400 (TX4927_SIO_SIBGR_BCLK_T2|TX4927_SIO_SIBGR_BRD_002400)
#define TX4927_SIO_BAUD_004800 (TX4927_SIO_SIBGR_BCLK_T2|TX4927_SIO_SIBGR_BRD_004800)
#define TX4927_SIO_BAUD_009600 (TX4927_SIO_SIBGR_BCLK_T0|TX4927_SIO_SIBGR_BRD_009600)
#define TX4927_SIO_BAUD_014400 (TX4927_SIO_SIBGR_BCLK_T0|TX4927_SIO_SIBGR_BRD_014400)
#define TX4927_SIO_BAUD_019200 (TX4927_SIO_SIBGR_BCLK_T0|TX4927_SIO_SIBGR_BRD_019200)
#define TX4927_SIO_BAUD_028800 (TX4927_SIO_SIBGR_BCLK_T0|TX4927_SIO_SIBGR_BRD_028800)
#define TX4927_SIO_BAUD_038400 (TX4927_SIO_SIBGR_BCLK_T0|TX4927_SIO_SIBGR_BRD_038400)
#define TX4927_SIO_BAUD_057600 (TX4927_SIO_SIBGR_BCLK_T0|TX4927_SIO_SIBGR_BRD_057600)
#define TX4927_SIO_BAUD_076800 (TX4927_SIO_SIBGR_BCLK_T0|TX4927_SIO_SIBGR_BRD_076800)
#define TX4927_SIO_BAUD_115200 (TX4927_SIO_SIBGR_BCLK_T0|TX4927_SIO_SIBGR_BRD_115200)

#define TX4927_SIO_SITFIFO                    0x1c
#define TX4927_SIO_SITFIFO_CHAR               BM_00_07


#define TX4927_SIO_SIRFIFO                    0x20
#define TX4927_SIO_SIRFIFO_CHAR               BM_00_07

#define TX4927_SIO_RD(b,o)   TX4927_RD( ((u32)(b)) + ((u32)(o))      )
#define TX4927_SIO_WR(b,o,v) TX4927_WR( ((u32)(b)) + ((u32)(o)), (v) )

/******************************************************************************/
/* END: H/W Register Locations/Definitions                                    */
/******************************************************************************/






struct tx4927_sio_hw_st
{
  vu32 silcr;
  vu32 sidicr; 
  vu32 sidisr; 
  vu32 siscisr;
  vu32 sifcr;
  vu32 siflcr; 
  vu32 sibgr;
  vu32 sitfifo;
  vu32 sirfifo;
};
typedef struct tx4927_sio_hw_st tx4927_sio_hw_st;


struct tx4927_sio_sw_st
{
  struct gs_port          gs;                 /* Must be first field! */
  tx4927_sio_hw_st*       base;
  unsigned int            size;
  unsigned long           irq;
  unsigned int            tx_fifo_max;
  unsigned int            rx_fifo_max;
  int                     flags;
  struct wait_queue       *shutdown_wait;
  struct async_icount     icount;             /* 4 IRQ counters */
  int                     read_status_mask;
  int                     ignore_status_mask;
  int                     x_char;             /* XON/XOFF */
  char*                   irq_name;
};
typedef struct tx4927_sio_sw_st tx4927_sio_sw_st;


void
tx4927_sio_bits_set_clr( tx4927_sio_hw_st* base, u32 offset, u32 set, u32 clr );

u32
tx4927_sio_rd( tx4927_sio_hw_st* base, int offset );

u32
tx4927_sio_rx_fifo_ready( tx4927_sio_hw_st* base );

u32
tx4927_sio_rx_fifo_rd( tx4927_sio_hw_st* base );

u32
tx4927_sio_rx_fifo_ready_rd( tx4927_sio_hw_st* base );

void
tx4927_sio_wr( tx4927_sio_hw_st* base, int offset, u32 value );

void
tx4927_sio_tx_fifo_wait( tx4927_sio_hw_st* base );

void
tx4927_sio_tx_fifo_flush( tx4927_sio_hw_st* base );

void
tx4927_sio_tx_fifo_wait_wr_flush( tx4927_sio_hw_st* base, u8 ch );


#endif
