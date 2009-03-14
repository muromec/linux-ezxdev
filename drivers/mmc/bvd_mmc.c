/*
 * drivers/mmc/bvd_mmc.c
 * Low-level MMC/SD functions for the Intel Bulverde MMC/SD on-chip controller
 * Mostly based on: drivers/mmc/omap_mmc.c
 *
 * Copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * References to technical documents used in comments:
 * [1] - Intel(R) Bulverde Processor (B-Stepping) Developer's Manual
 */
/*
 * Copyright 2004 Motorola, Inc. All Rights Reserved.
 * Revision History:
                    Modification    
 Changed by            Date             Description of Changes
----------------   ------------      -------------------------
Zhu Zhifu           04/05/2004         change for write protection
zhou qiong          06/08/2004         change for pm callback
Jiang Lili          08/24/2005         change for add card detect when call pm resume
Jiang Lili          10/25/2005         change for sleep power consume isue
*/

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/pm.h>
#include <linux/mmc/mmc_ll.h>
#include <linux/pci.h>

#include <asm/irq.h>        
#include <asm/unaligned.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mainstone.h>
#include <asm/arch/ezx.h>
#include <asm/dma.h>

#ifdef CONFIG_ARCH_EZX_HAINAN
extern u8 ezx_mmc_get_slot_state(void);
#endif 

#define MMC_CLK_OFF_TIMEOUT 1000

#define MMC_COMMAND_TIMEOUT HZ

/* Device-specific data */
typedef struct bvd_mmc_data {
	struct mmc_request *	request;
	int			sd;
	int			dma_rx_ch;
	int			dma_tx_ch;
	u32			event_mask;
	int                     use_4bit;
	struct timer_list	timeout;
} bvd_mmc_data_t;

#define MMC_EVENT_RESPONSE	0x01	/* Command response */
#define MMC_EVENT_DATA_DONE	0x02	/* Data Transfer done */
#define MMC_EVENT_RX_DMA_DONE	0x04	/* Rx DMA done */
#define MMC_EVENT_TX_DMA_DONE	0x08	/* Tx DMA done */
#define MMC_EVENT_PROG_DONE	0x10	/* Programming is done */
#define MMC_EVENT_CLK_OFF	0x80	/* Clock is off */

static bvd_mmc_data_t bvd_mmc_data;
extern void ezx_detect_handler(unsigned long data);

static int mmc_crc_error = 0;

#ifdef CONFIG_ARCH_EZX_E680
/* dont call hotplug funciton if phone resumes from sleeping */ 
static void e680_reconfig_gpio()
{
    set_GPIO_mode(GPIO_MMC_CLK | GPIO_ALT_FN_2_OUT);
    set_GPIO_mode(GPIO_MMC_CMD | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
    set_GPIO_mode(GPIO_MMC_DATA0  | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
    set_GPIO_mode(GPIO_MMC_DATA1 | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
    set_GPIO_mode(GPIO_MMC_DATA2 | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
    set_GPIO_mode(GPIO_MMC_DATA3 | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
}
#endif

/* Stop the MMC clock and wait while it is happens */
static inline int bvd_mmc_stop_clock_and_wait(void)
{
	int timeout;
	
	DEBUG(2, "stop MMC clock\n");
	MMC_STRPCL = MMC_STRPCL_STOP_CLK;

	for (timeout = 0; !(MMC_I_REG & MMC_I_CLK_IS_OFF); timeout++) {
		if (timeout > MMC_CLK_OFF_TIMEOUT) {
			DEBUG(3, "Timeout on stop clock waiting\n");
			return MMC_ERROR_TIMEOUT;
		}
		udelay(1);
	}
	DEBUG(2, "clock off time is %d microsec\n", timeout);
        return MMC_NO_ERROR;
}

/* Stop the clock */
static inline void bvd_mmc_stop_clock(void)
{
	MMC_STRPCL = MMC_STRPCL_STOP_CLK;
}

/* Start the MMC clock */
static inline int bvd_mmc_start_clock(void)
{
	DEBUG(2, "start MMC clock\n");
	MMC_STRPCL = MMC_STRPCL_STRT_CLK;
	return MMC_NO_ERROR;
}

/* Select the MMC clock frequency */
int bvd_mmc_set_clock(u32 rate)
{
	DEBUG(2, "set clock to %u Hz\n", rate);
	bvd_mmc_stop_clock_and_wait();
	if (rate < 304000)
		return MMC_ERROR_OUT_OF_RANGE;

	/* It seems, MMC controller can not operate correctly at highest
	   clock frequency */
	MMC_CLKRT = (rate >= 19500000 ? MMC_CLKRT_FREQ_19_5MHZ :
		     rate >= 9750000  ? MMC_CLKRT_FREQ_9_75MHZ :
		     rate >= 4880000  ? MMC_CLKRT_FREQ_4_88MHZ :
		     rate >= 2440000  ? MMC_CLKRT_FREQ_2_44MHZ : 
		     rate >= 1220000  ? MMC_CLKRT_FREQ_1_22MHZ :
		     rate >= 609000   ? MMC_CLKRT_FREQ_609KHZ :
		     MMC_CLKRT_FREQ_304KHZ);

	return MMC_NO_ERROR;
}

/* Initialize the MMC controller to up the slot, assuming the card is in slot */
void bvd_mmc_slot_up(void) 
{
	DEBUG(2, "Init MMC h/w\n");
	
	/* Turn on core clock signal for the MMC controller ([2], 3.8.2.2) */
	CKEN |= CKEN12_MMC;
	DEBUG(3, " ...core MMC clock OK\n");
	
	/* Configure MMCLK bus clock output signal
	   ([2], 15.3, 24.4.2) */
	set_GPIO_mode(GPIO_MMC_CLK | GPIO_ALT_FN_2_OUT);
	DEBUG(3, " ...MMCLK signal OK\n");

	/* Configure MMCMD command/response bidirectional signal
	   ([2], 15.3, 24.4.2) */
	set_GPIO_mode(GPIO_MMC_CMD | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
	DEBUG(3, " ...MMCMD signal OK\n");

	/* Configure MMDAT[0123] data bidirectional signals
	   ([2], 15.3, 24.4.2) */
	set_GPIO_mode(GPIO_MMC_DATA0  | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
	// for dat3 used to detect card insert and remove on A780, 
	// so we only use 1 bit transfer mode ---zq
#ifndef CONFIG_ARCH_EZX_A780
    //from Barbados P3, we will use 4 bit mode --jll
 	set_GPIO_mode(GPIO_MMC_DATA1 | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
	set_GPIO_mode(GPIO_MMC_DATA2 | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
	set_GPIO_mode(GPIO_MMC_DATA3 | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
#endif 	
	DEBUG(3, " ...MMDATx signals OK\n");

	/* One of Intel's hardware hackers recommend me
	   to wait 1ms here. Ok, it's not so complex :-) */
	mdelay(1);
	
	/*
	 * Ok, looks like basic h/w initialized, let's talk with MMC itself
	 */
	
        /* Stop the MMC clock before 1st command ([2], 15.6) */
	bvd_mmc_stop_clock_and_wait();

	enable_irq(IRQ_MMC);

	DEBUG(2, "MMC h/w initialized\n");
}

/* Shut down the slot */
void bvd_mmc_slot_down(void)
{
	DEBUG(2, "down MMC h/w\n");

	/* Turn off core clock signal for the MMC controller ([2], 3.8.2.2) */
	CKEN &= ~CKEN12_MMC;
}

/* Halt Tx and Rx DMA channels */
static inline void bvd_mmc_dma_halt(struct bvd_mmc_data *mmc)
{
	DCSR(mmc->dma_tx_ch) = DCSR_ENDINTR | DCSR_STARTINTR | DCSR_BUSERR;
	DCSR(mmc->dma_rx_ch) = DCSR_ENDINTR | DCSR_STARTINTR | DCSR_BUSERR;
}



/* Establish set of events we are waiting for command completion */
static void bvd_mmc_wait_for(u32 event_set)
{
	unsigned long flags;
	local_irq_save(flags);
	bvd_mmc_data.event_mask = event_set;
	MMC_I_MASK = MMC_I_ALL &
		~((event_set & MMC_EVENT_DATA_DONE ? MMC_I_DATA_TRAN_DONE : 0) |
		  (event_set & MMC_EVENT_RESPONSE ? MMC_I_END_CMD_RES : 0) |
		  (event_set & MMC_EVENT_PROG_DONE ? MMC_I_PRG_DONE : 0) |
		  (event_set == MMC_EVENT_CLK_OFF ? MMC_I_CLK_IS_OFF : 0) |
		  (event_set != 0 ? 
		   MMC_I_RES_ERR | MMC_I_DAT_ERR | MMC_I_TINT :
		   0));
	local_irq_restore(flags);
}

/* Complete the request processing */
static inline void bvd_mmc_request_complete(struct bvd_mmc_data *mmc, 
					    enum mmc_result_t result)
{
	struct mmc_request *req = mmc->request;
	unsigned int flags;

	local_irq_save(flags);
	if (req != NULL) {
		bvd_mmc_dma_halt(mmc);
		del_timer(&mmc->timeout);
		req->result = result;
		mmc_cmd_complete(req);
		mmc->request = NULL;
		bvd_mmc_wait_for(0);
	}
	local_irq_restore(flags);
	if (result != MMC_NO_ERROR)
		bvd_mmc_stop_clock_and_wait();
}


/* Trigger the event(s). Complete commands if all expected events are
   occured */
static void bvd_mmc_event(u32 event)
{
	unsigned long flags;
	u32 events;
	struct bvd_mmc_data *mmc = &bvd_mmc_data;
	local_irq_save(flags);
	events = mmc->event_mask & ~event;
	bvd_mmc_wait_for(events);
	local_irq_restore(flags);

	if (events == MMC_EVENT_CLK_OFF)
		bvd_mmc_stop_clock();
	else if (events == 0)
	{
		if (mmc_crc_error)
		    bvd_mmc_request_complete(mmc, MMC_ERROR_CRC);
		else
		    bvd_mmc_request_complete(mmc, MMC_NO_ERROR);
	}
}

static void bvd_mmc_dma_rx_start(struct bvd_mmc_data *mmc);

/* Handle DMA data receive completion */
static void bvd_mmc_dma_rx_callback(int channel, void *data, 
				      struct pt_regs *regs)
{
	struct bvd_mmc_data *mmc = data;
	struct mmc_request *request = mmc->request;

	DEBUG(3, "DMA RX Callback: DCSR 0x%08x MMC_STAT 0x%08x "
		 "I_REG 0x%08x I_MASK 0x%08x nob %d\n",
		DCSR(channel), MMC_STAT, MMC_I_REG, MMC_I_MASK,
		request->nob);

	request->buffer += request->block_len;
	if (DCSR(channel) & DCSR_BUSERR)  {
		printk(KERN_DEBUG "bvd_mmc: MMC rx dma bus error.\n");
	}
	DCSR(channel) = DCSR_STARTINTR | DCSR_ENDINTR | DCSR_BUSERR;
	if (--request->nob > 0) {
		bvd_mmc_dma_rx_start(mmc);
	} else {
		bvd_mmc_event(MMC_EVENT_RX_DMA_DONE);
	}
}

/* Prepare DMA to start data transfer from the MMC card */
static void bvd_mmc_dma_rx_start(struct bvd_mmc_data *mmc)
{
	struct mmc_request *request = mmc->request;
	int channel = mmc->dma_rx_ch;
 	dma_addr_t dma_addr = virt_to_bus(request->buffer);

	DEBUG(3, "MMC DMA Rx start: chan %d buf 0x%08x phys 0x%08x "
		 "blklen %d nob%d\n",
		 channel, (u32)request->buffer, (u32)dma_addr,
		 request->block_len, request->nob);

	consistent_sync(request->buffer, request->block_len, 
			PCI_DMA_FROMDEVICE);

	DCSR(channel) = DCSR_NODESC;
	DSADR(channel) = __PREG(MMC_RXFIFO);
	DTADR(channel) = dma_addr;
	DCMD(channel) = DCMD_INCTRGADDR | DCMD_FLOWSRC | DCMD_WIDTH1 |
			DCMD_BURST32 | DCMD_ENDIRQEN |
			(request->block_len & DCMD_LENGTH);
	DRCMRRXMMC = (channel & DRCMR_CHLNUM) | DRCMR_MAPVLD;
	DCSR(channel) |= DCSR_RUN;
}

static void bvd_mmc_dma_tx_start(struct bvd_mmc_data *mmc);

/* Handle transmit DMA competion */
static void bvd_mmc_dma_tx_callback(int channel, void *data, 
				    struct pt_regs *regs)
{
	struct bvd_mmc_data *mmc = data;
	struct mmc_request *request = mmc->request;
	
	DEBUG(3, "DMA TX Callback\n");
	request->buffer += request->block_len;
	if (DCSR(channel) & DCSR_BUSERR)  {
		printk(KERN_DEBUG "bvd_mmc: MMC tx dma bus error.\n");
	}
	DCSR(channel) = DCSR_STARTINTR | DCSR_ENDINTR | DCSR_BUSERR;
	if (--request->nob > 0)
		bvd_mmc_dma_tx_start(mmc);
	else
		bvd_mmc_event(MMC_EVENT_TX_DMA_DONE);
}

/* Prepare DMA to start data transfer to the MMC card */
static void bvd_mmc_dma_tx_start(struct bvd_mmc_data *mmc)
{
	struct mmc_request *request = mmc->request;
	int channel = mmc->dma_tx_ch;
	dma_addr_t dma_addr = virt_to_bus(request->buffer);

	DEBUG(3, "MMC DMA Tx start: chan %d buf 0x%08x phys 0x%08x "
		 "blklen %d nob%d\n",
		 channel, (u32)request->buffer, (u32)dma_addr,
		 request->block_len, request->nob);

	consistent_sync(request->buffer, request->block_len, 
			PCI_DMA_TODEVICE);

	DCSR(channel) = DCSR_NODESC;
	DTADR(channel) = __PREG(MMC_TXFIFO);
	DSADR(channel) = dma_addr;
	DCMD(channel) = DCMD_INCSRCADDR | DCMD_FLOWTRG |  DCMD_WIDTH1 | 
			DCMD_BURST32 | DCMD_ENDIRQEN |
			(request->block_len & DCMD_LENGTH);
	DRCMRTXMMC = (channel & DRCMR_CHLNUM) | DRCMR_MAPVLD;
	DCSR(channel) |= DCSR_RUN;
}


/* Prepare MMC controller for card command execution */
static int bvd_mmc_exec_command(struct mmc_request *request)
{
	u32 cmdat = 0;
	u32 nob = 1;

	/* use 4-bit bus width when possible */
	if (bvd_mmc_data.use_4bit) 
		cmdat |= MMC_CMDAT_SD_4DAT;

	switch (request->cmd) {
	/* MMC core extra command */
	case MMC_CIM_RESET:
		cmdat |= MMC_CMDAT_INIT;
		break;

	/* bc - broadcast - no response */
	case MMC_GO_IDLE_STATE:
	case MMC_SET_DSR:
		break;

	/* bcr - broadcast with response */
	case MMC_SEND_OP_COND:
	case MMC_ALL_SEND_CID:
	case MMC_GO_IRQ_STATE:
		break;

	/* adtc - addressed with data transfer */
	case MMC_READ_DAT_UNTIL_STOP:
	case MMC_READ_SINGLE_BLOCK:
	case MMC_READ_MULTIPLE_BLOCK:
		cmdat |= MMC_CMDAT_DATA_EN |
			 MMC_CMDAT_RD | MMC_CMDAT_DMA_EN;
		break;

	case SEND_SCR:
		cmdat |= MMC_CMDAT_DATA_EN | MMC_CMDAT_RD;
		break;

	case MMC_WRITE_DAT_UNTIL_STOP:
	case MMC_WRITE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:
	case MMC_PROGRAM_CID:
	case MMC_PROGRAM_CSD:
	case MMC_SEND_WRITE_PROT:
	case MMC_GEN_CMD:
		cmdat |= MMC_CMDAT_DATA_EN | MMC_CMDAT_WR | MMC_CMDAT_DMA_EN;
		break;

	case MMC_LOCK_UNLOCK:
		cmdat |= MMC_CMDAT_DATA_EN | MMC_CMDAT_WR;
		break;

	case MMC_STOP_TRANSMISSION:
		cmdat |= MMC_CMDAT_STOP_TRAN;
		break;

	/* ac - no data transfer */
	default: 
		break;
	}  
        
	switch (request->cmd) {
	case MMC_READ_MULTIPLE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:
		nob = request->nob;
		break;
	default:
		nob = 1;
	}

	switch (request->rtype) {
	case RESPONSE_NONE:
		cmdat |= MMC_CMDAT_RES_NORESP;
		break;

	case RESPONSE_R1B:
		cmdat |= MMC_CMDAT_BUSY;
		/*FALLTHRU*/
	case RESPONSE_R1:
	case RESPONSE_R4:
	case RESPONSE_R5:
	case RESPONSE_R6:
		cmdat |= MMC_CMDAT_RES_RESP;
		break;

	case RESPONSE_R3:
		cmdat |= MMC_CMDAT_RES_R3;
		break;

	case RESPONSE_R2_CID:
	case RESPONSE_R2_CSD:
		cmdat |= MMC_CMDAT_RES_R2;
		break;

	default:
		break;
	}

	/* Set command index */
	if (request->cmd == MMC_CIM_RESET) {
		MMC_CMD = MMC_GO_IDLE_STATE & MMC_CMD_MASK;
	} else {
		MMC_CMD = request->cmd & MMC_CMD_MASK;
	}
        
        /* Set argument */    
	MMC_ARGL = request->arg & MMC_ARGL_MASK;
	MMC_ARGH = (request->arg >> 16) & MMC_ARGH_MASK;

	if (request->cmd == SEND_SCR) {
		MMC_BLKLEN = 8;
		MMC_NOB = 1;
	} else {
		MMC_BLKLEN = request->block_len & MMC_BLKLEN_MASK;
		MMC_NOB = nob & MMC_NOB_MASK;
	}
	MMC_RDTO = ~0 & MMC_RDTO_MASK;
	MMC_RESTO = ~0 & MMC_RESTO_MASK;

	MMC_CMDAT = cmdat;

        /* Send command */
	DEBUG(3, ": Send cmd %d cmdat: %x arg: %d resp %d\n", request->cmd,
	      cmdat, request->arg, request->rtype);

	/* Setup DMA if necessary */
	switch (request->cmd) {
	case MMC_READ_SINGLE_BLOCK:
	case MMC_READ_MULTIPLE_BLOCK:
		bvd_mmc_wait_for(MMC_EVENT_RESPONSE |
				 MMC_EVENT_DATA_DONE |
				 MMC_EVENT_RX_DMA_DONE |
				 MMC_EVENT_CLK_OFF);
		bvd_mmc_dma_rx_start(&bvd_mmc_data);
		break;

	case MMC_WRITE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:
	case MMC_PROGRAM_CID:
	case MMC_PROGRAM_CSD:
	case MMC_SEND_WRITE_PROT:
	case MMC_GEN_CMD:
		bvd_mmc_wait_for(MMC_EVENT_RESPONSE |
				 MMC_EVENT_DATA_DONE |
				 MMC_EVENT_TX_DMA_DONE |
				 MMC_EVENT_PROG_DONE |
				 MMC_EVENT_CLK_OFF);
		bvd_mmc_dma_tx_start(&bvd_mmc_data);
		break;

	case MMC_LOCK_UNLOCK:
		bvd_mmc_wait_for(MMC_EVENT_RESPONSE |
				 MMC_EVENT_DATA_DONE |
				 MMC_EVENT_PROG_DONE |
				 MMC_EVENT_CLK_OFF);
		break;

	case MMC_WRITE_DAT_UNTIL_STOP:
	case MMC_READ_DAT_UNTIL_STOP:
		printk(KERN_ERR "Unsupported MMC data transfer command: %d\n",
		       request->cmd);
		bvd_mmc_wait_for(0);
		break;

	case SEND_SCR:
		bvd_mmc_wait_for(MMC_EVENT_RESPONSE | 
				 MMC_EVENT_DATA_DONE |
				 MMC_EVENT_CLK_OFF);
		break;

	case MMC_STOP_TRANSMISSION:
		bvd_mmc_wait_for(MMC_EVENT_RESPONSE |
		                 MMC_EVENT_DATA_DONE |
		                 MMC_EVENT_PROG_DONE |
		                 MMC_EVENT_CLK_OFF);
		break;

	default:
		bvd_mmc_wait_for(MMC_EVENT_RESPONSE |
				 MMC_EVENT_CLK_OFF);
		break;
	}

	bvd_mmc_data.timeout.expires = jiffies + MMC_COMMAND_TIMEOUT;
	add_timer(&bvd_mmc_data.timeout);
	
	barrier();
	bvd_mmc_start_clock();
	return MMC_NO_ERROR;
}

/* Start the command execution */
void bvd_mmc_send_command(struct mmc_request *request)
{
	int retval;

	DEBUG(2, "send command [%d %x]\n", request->cmd, request->arg);

	/* Sanity check */
	if (bvd_mmc_data.request != NULL) {
		printk(KERN_ERR "MMC command sent while another command "
				" in processing\n");
		request->result = MMC_ERROR_DRIVER_FAILURE;
		mmc_cmd_complete(request);
	}

        /* CRC error check. add by w20598 */
	if (mmc_crc_error)
	{
	    /* prevent continuous CRC error interrupt after an CRC error happens */
	    /* this will make for IRQ23(Memory to Memeory DMA) diable */
	    disable_irq(IRQ_MMC); 
	    mmc_crc_error = 0;
            bvd_mmc_request_complete(&bvd_mmc_data, MMC_ERROR_CRC);
	}
	
	/* Save current request for the future processing */
 	bvd_mmc_data.request = request;
	/* Indicate we have no result yet */
	request->result = MMC_NO_RESPONSE;

	if (request->cmd == MMC_CIM_RESET) {
		/* On reset, 1-bit bus width */
		bvd_mmc_data.use_4bit = 0;

		/* On reset, drop MMC clock down */
		retval = bvd_mmc_set_clock(MMC_CLOCK_SLOW);
		if (retval) {
			bvd_mmc_request_complete(&bvd_mmc_data, retval);
			return;
		}
	}

        switch(request->cmd) {
          case SD_SEND_IF_COND:
          case SD_SEND_OP_COND:
                  DEBUG(3, "Have an SD card\n");
                  bvd_mmc_data.sd = 1;
                  break;

          case MMC_SEND_OP_COND:
                  DEBUG(3, "Have an MMC card\n");
                  bvd_mmc_data.sd = 0;
                  /* always use 1bit for MMC */
                  bvd_mmc_data.use_4bit = 0;
                  break;
        }

	if (request->cmd == SET_BUS_WIDTH) {
		if (request->arg == 0x2) {
			DEBUG(2, "Use 4-bit bus width\n");
			bvd_mmc_data.use_4bit = 1;
		}
		else {
			DEBUG(2, "Use 1-bit bus width\n");
			bvd_mmc_data.use_4bit = 0;
		}
	}
	retval = bvd_mmc_exec_command(request);
	if (retval) {
		bvd_mmc_request_complete(&bvd_mmc_data, retval);
	}
}

/* Fetch response data from the response FIFO */
static void bvd_mmc_fetch_response(u8 *buf, int len)
{
	while (len-- > 0) {
		u16 rdata = MMC_RES;
		*buf++ = (rdata >> 8) & 0xff;
		if (len-- > 0)
			*buf++ = rdata & 0xff;
	}
}

/* Obtain response to the command and store it to response buffer */
static void bvd_mmc_get_response(struct mmc_request *request)
{
	u8 *buf;

	if (!request) {
		DEBUG(1, "Oops - fetch response for zero request ?\n");
		return;
	}
	buf = request->response;

	DEBUG(2, "fetch response for request %d, cmd %d\n", request->rtype, request->cmd);

	request->result = MMC_NO_ERROR;
	switch (request->rtype) {
	case RESPONSE_R1: case RESPONSE_R1B: case RESPONSE_R6:
	case RESPONSE_R3: case RESPONSE_R4: case RESPONSE_R5:
		bvd_mmc_fetch_response(buf, 5);
		DEBUG(3, "request %d, response [%02x %02x %02x %02x %02x]\n",
		      request->rtype, buf[0], buf[1], buf[2], buf[3], buf[4]);
		break;

	case RESPONSE_R2_CID: case RESPONSE_R2_CSD:
		bvd_mmc_fetch_response(buf, 17);
		DEBUG(3, "request %d, response [", request->rtype);
#if CONFIG_MMC_DEBUG_VERBOSE > 2
		if (g_mmc_debug >= 3) {
			int n;
			for (n = 0; n < 17; n++)
				printk("%02x ", buf[n]);
			printk("]\n");
		}
#endif
		break;
		
	case RESPONSE_NONE:
		bvd_mmc_fetch_response(buf, 0);
		DEBUG(3, "No response\n");
		break;
		
	default:
		bvd_mmc_fetch_response(buf, 0);
		DEBUG(3, "unhandled response type for request %d\n", request->rtype);
		break;
	}
}

/* SD card returns SCR register as data. MMC core expect it in the
   response buffer, after normal response. */
static void bvd_mmc_get_extra_response(struct mmc_request *request)
{
	u8 *buf = request->response;
	if (request->cmd == SEND_SCR) {
		int i;
		DEBUG(3, "extra response: [");
		for (i = 0; i < 8; i+=4) 
			*((u32*)(buf + i + 5)) = MMC_RXFIFO;
#if CONFIG_MMC_DEBUG_VERBOSE > 2
		for (i = 0; i < 8; i++) {
			if (g_mmc_debug >= 3)
				printk(" %02x", buf[i + 5]);
		}
		printk(" ]\n");
#endif
	}
}

/* Program I/O mode data transfer */
static void bvd_mmc_tx(struct mmc_request *request)
{
	int i, count;
	
	DEBUG(2, "\n");

	if (!request->cnt) return;

	count = (request->cnt < 32) ? request->cnt : 32;
	for (i=0; i < count; i+=4)
		MMC_TXFIFO = *((u32*)(request->buffer+i));

	if (count < 32) 
		MMC_PRTBUF = 0x1;

	request->buffer += count;
	request->cnt -= count;
}

/* Card service IRQ handler */
static void bvd_mmc_int(int irq, void *dev, struct pt_regs *regs)
{
	int status, irqtype;
	bvd_mmc_data_t *mmc = (bvd_mmc_data_t *)dev;

	status = MMC_STAT;
	irqtype = MMC_I_REG;

	DEBUG(3, "MMC service IRQ [MMC_STAT %x, MMC_I_REG %x, MMC_I_MASK %x], CD %p, request %p\n",
	      status, irqtype, MMC_I_MASK, mmc, mmc->request);

	/* Checking for data or response timeout */
	if ((status & (MMC_STAT_TIME_OUT_READ | MMC_STAT_TIME_OUT_RES)) ||
	    (irqtype & MMC_I_TINT)) {
		DEBUG(1, "MMC/SD timeout, MMC_STAT 0x%x\n", status);
		bvd_mmc_request_complete(mmc, MMC_ERROR_TIMEOUT);
		return;
	}

	/* Checking for CRC error */
	if ((status & (MMC_STAT_CRC_RD_ERR | MMC_STAT_CRC_WR_ERR | 
		       MMC_STAT_RES_CRC_ERR)) ||
	    (irqtype & (MMC_I_RES_ERR | MMC_I_DAT_ERR))) {
		DEBUG(1, "MMC/CD CRC error, MMC_STAT 0x%x\n", status);

		if (!mmc_crc_error)
		{

		switch (mmc->request->cmd) {
			/* FIXME: workaround for sighting #55580 */
		case MMC_ALL_SEND_CID:
		case MMC_SEND_CSD:
		case MMC_SEND_CID:
			/* ignore CRC error for these commands */
			break;

			/* FIXME: workaround for sighting #59136 */
		case MMC_SEND_STATUS:
			if ((MMC_CLKRT & 0x7) == MMC_CLKRT_FREQ_19_5MHZ) {
				struct mmc_request *req = mmc->request;

				bvd_mmc_set_clock(9750000);

				del_timer(&mmc->timeout);
				bvd_mmc_wait_for(0);
				mmc->request = NULL;

				bvd_mmc_send_command(req);

				return;
			}
                  
			/* FIXME: multi-block CRC issue */
		case MMC_READ_SINGLE_BLOCK:
		case MMC_READ_MULTIPLE_BLOCK:
		    mmc_crc_error = 1;
 		    bvd_mmc_wait_for(MMC_EVENT_RX_DMA_DONE);
		    return;

		case MMC_WRITE_BLOCK:
		case MMC_WRITE_MULTIPLE_BLOCK:
		    mmc_crc_error = 1;
                    bvd_mmc_wait_for(MMC_EVENT_TX_DMA_DONE);
                    return;

			/* else fall-through */
		default:
			bvd_mmc_request_complete(mmc, MMC_ERROR_CRC);
			return;
		}

		} /* if (! mmc_crc_error) */
	}

	/* If the command and response sequence has completed, get the 
	   response */
	if ((mmc->event_mask & MMC_EVENT_RESPONSE) && 
	    (irqtype & MMC_I_END_CMD_RES)) {
		bvd_mmc_get_response(mmc->request);
		bvd_mmc_event(MMC_EVENT_RESPONSE);
	}

	/* Transfer data in program I/O mode */
	if ((mmc->event_mask & MMC_EVENT_DATA_DONE) &&
	    (irqtype & MMC_I_TXFIFO_WR_REQ)) {
		bvd_mmc_tx(mmc->request);
	}

	/* Handle data transfer done event */	
	if ((mmc->event_mask & MMC_EVENT_DATA_DONE) && 
	    (irqtype & MMC_I_DATA_TRAN_DONE)) {
		bvd_mmc_get_extra_response(mmc->request);
		bvd_mmc_event(MMC_EVENT_DATA_DONE);
	}
	
	/* Handle programming done event */
	if ((mmc->event_mask & MMC_EVENT_PROG_DONE) &&
	    (irqtype & MMC_I_PRG_DONE)) {
		bvd_mmc_event(MMC_EVENT_PROG_DONE);
	}

	/* Handle clock off event */	
	if ((mmc->event_mask == MMC_EVENT_CLK_OFF) && 
	    (MMC_I_REG & MMC_I_CLK_IS_OFF)) {
		bvd_mmc_event(MMC_EVENT_CLK_OFF);
	}
}

/* Timeout handler */
static void bvd_mmc_timeout(unsigned long data)
{
	struct bvd_mmc_data *mmc = (struct bvd_mmc_data *)data;
	printk(KERN_ERR "MMC timeout: waiting events 0x%02x\n", 
	       mmc->event_mask);

	if (!mmc_crc_error)
            bvd_mmc_request_complete(mmc, MMC_ERROR_TIMEOUT);
}

#ifdef CONFIG_PM
/*
 * Standard PM functions, if CONFIG_PM is used
 */

/* Suspend the MMC slot */
static int bvd_mmc_suspend(void)
{
	DEBUG(1, "suspend mmc slot\n");
	/*set GPIO output high to save power*/
	CKEN &= ~CKEN12_MMC;
	set_GPIO(GPIO_MMC_CLK);
	set_GPIO_mode(GPIO_MMC_CLK | GPIO_OUT);
	set_GPIO(GPIO_MMC_CMD);
	set_GPIO_mode(GPIO_MMC_CMD | GPIO_OUT);
	set_GPIO(GPIO_MMC_DATA0);
	set_GPIO_mode(GPIO_MMC_DATA0 | GPIO_OUT);
	set_GPIO(GPIO_MMC_DATA1);
	set_GPIO_mode(GPIO_MMC_DATA1 | GPIO_OUT);
	set_GPIO(GPIO_MMC_DATA2);
	set_GPIO_mode(GPIO_MMC_DATA2 | GPIO_OUT);
#ifdef CONFIG_ARCH_EZXBASE
	set_GPIO(GPIO_MMC_DATA3);	
	set_GPIO_mode(GPIO_MMC_DATA3 | GPIO_IN);
#endif	
	return 0;
}

/* Resume the MMC slot */
static void bvd_mmc_resume(void)
{
	DEBUG(1, "resume mmc slot\n");
	
#ifdef CONFIG_ARCH_EZXBASE		
#ifndef CONFIG_ARCH_EZX_HAINAN
    if( GPLR(GPIO_MMC_DETECT) & GPIO_bit(GPIO_MMC_DETECT) )
#else
    if(ezx_mmc_get_slot_state())
#endif
#else
	if( GPLR(GPIO_MMC_DATA3) & GPIO_bit(GPIO_MMC_DATA3) )
#endif	
	{
		CKEN |= CKEN12_MMC;
		set_GPIO_mode(GPIO_MMC_CLK | GPIO_ALT_FN_2_OUT);
		set_GPIO_mode(GPIO_MMC_CMD | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
		set_GPIO_mode(GPIO_MMC_DATA0  | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
#ifdef CONFIG_ARCH_EZXBASE		
		set_GPIO_mode(GPIO_MMC_DATA1  | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
		set_GPIO_mode(GPIO_MMC_DATA2  | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
		set_GPIO_mode(GPIO_MMC_DATA3  | GPIO_ALT_FN_1_IN | GPIO_ALT_FN_1_OUT);
#endif	
	}	
}

static int bvd_mmc_pm_callback(struct pm_dev *pm_dev,
			       pm_request_t req, void *data)
{
	switch(req) {
	case PM_SUSPEND:
		bvd_mmc_suspend();
		break;
		
	case PM_RESUME:
#ifdef CONFIG_ARCH_EZX_E680
		e680_reconfig_gpio();
#else 
		bvd_mmc_resume();
#endif
		break;
		
	default:
		printk(KERN_ERR "MMC/SD: invalid PM request %d\n", req);
		break;
	}
	return 0;
}
#endif /* CONFIG_PM */



/* Initialize the slot and prepare software layer to operate with it */
int bvd_mmc_slot_init(void)
{
	int retval;
	int channel;


	/* Request basic card servicing IRQ */
	retval = request_irq(IRQ_MMC, bvd_mmc_int,
			     SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "MMC/SD", &bvd_mmc_data);
	if (retval) {
		printk(KERN_ERR "MMC/SD: can't request MMC/SD IRQ\n");
		return retval;
	}
	
	/* Request MMC Rx DMA channel */
	channel = pxa_request_dma("MMC Rx", DMA_PRIO_MEDIUM,
				  bvd_mmc_dma_rx_callback,
				  &bvd_mmc_data);
	if (channel < 0) {
		printk(KERN_ERR "pxa_request_dma failed for MMC Rx\n");
		goto err1;
	}
	bvd_mmc_data.dma_rx_ch = channel;

	/* Request MMC Tx DMA channel */
	channel = pxa_request_dma("MMC Tx", DMA_PRIO_MEDIUM,
				  bvd_mmc_dma_tx_callback,
				  &bvd_mmc_data);
	if (channel < 0) {
		printk(KERN_ERR "pxa_request_dma failed for MMC Tx\n");
		goto err2;
	}
	bvd_mmc_data.dma_tx_ch = channel;
	
	/* Use 1-bit by default */
	bvd_mmc_data.use_4bit = 0;

	init_timer(&bvd_mmc_data.timeout);
	bvd_mmc_data.timeout.function = bvd_mmc_timeout;
	bvd_mmc_data.timeout.data = (unsigned long)&bvd_mmc_data;
	
#ifdef CONFIG_PM
	/* Register MMC slot as as power-managed device */
        pm_register(PM_UNKNOWN_DEV, PM_SYS_UNKNOWN, bvd_mmc_pm_callback);
#endif

	DEBUG(2, "MMC initialized with code %d\n", retval);
	return retval;


err2:	pxa_free_dma(bvd_mmc_data.dma_rx_ch);
err1:	free_irq(IRQ_MMC, &bvd_mmc_data);

	return -1;
}

/* Shut down the slot and relax software about MMC slot */
void bvd_mmc_slot_cleanup(void)
{
        long flags;

	DEBUG(2, "Bye, MMC\n");

	local_irq_save(flags); 
	/* Shut down the slot */
	bvd_mmc_slot_down();
	
	/* Free DMA channels */
	pxa_free_dma(bvd_mmc_data.dma_rx_ch);
	pxa_free_dma(bvd_mmc_data.dma_tx_ch);

	/* Free both IRQs */
	free_irq(IRQ_MMC, &bvd_mmc_data);

        local_irq_restore(flags);
}

MODULE_LICENSE("GPL");
