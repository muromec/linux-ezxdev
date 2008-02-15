/*
 * drivers/mmc/omap_mmc.c
 *
 * Low-level MMC functions for the OMAP 1510/1610 MMC/SD controller
 * SD support is not present
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>
 *
 * 2003-2004 (c) Monta Vista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */

#undef CONFIG_CEE
#define CONFIG_CEE

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/mmc/mmc_ll.h>
#include <linux/pm.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/arch/irq.h>
#include <asm/unaligned.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#if defined(CONFIG_ARCH_OMAP730)
#include <asm/arch/omap730_config.h>
#include <asm/arch/perseus2.h>
#endif


struct response_info {
	int length;
	u16 cdc_flags;
};

enum omap_request_type {
	RT_NO_RESPONSE,
	RT_RESPONSE_ONLY,
	RT_READ,
	RT_WRITE
};

struct omap_mmc_data {
	struct timer_list sd_detect_timer;
	u32 clock;		/* Current clock frequency */
	struct mmc_request *request;	/* Current request */
	enum omap_request_type type;
	dma_regs_t *tx_dma_regs;
	dma_regs_t *rx_dma_regs;

};

static struct omap_mmc_data g_omap_mmc_data;
#ifdef CONFIG_ARCH_OMAP1510
#define OMAP_MMC_OCR 0x00ffc000
#define OMAP_MMC_INT INT_MMC
#define OMAP_MMC_CD  INT_FPGA_CD
#define OMAP_MMC_CSDP ((1 << 10) | (1 << 9) | 1)
#define OMAP_MMC_TX_CCR (1 << 13) | (1 << 5) | (1 << 6)
#define OMAP_MMC_RX_CCR (1 << 15) | (1 << 5) | (1 << 6)
#define OMAP_MMC_CEN 32
#define OMAP_MMC_CFN_DIV 64
#define OMAP_MMC_AF_LEVEL (1 << 7)
static u8 card_state = 1 << 4;
#define OMAP_MMC_CON_CLKD_MSK 0x00FF
#endif
#ifdef CONFIG_ARCH_OMAP1610
static u8 card_state = 1 << 1;
#define OMAP_MMC_OCR 0x00ff8000
#define OMAP_MMC_INT INT_MMC_SDIO1
#define OMAP_MMC_CD  INT_MPUIO_1
#define OMAP_MMC_CSDP ((1 << 10) | (1 << 9) | (2 << 7) | (1 << 6) | 1)
#define OMAP_MMC_TX_CCR (1 << 12) | (1 << 10) | (1 << 5) | (1 << 6)
#define OMAP_MMC_RX_CCR (1 << 14) | (1 << 10) | (1 << 5) | (1 << 6)
#define OMAP_MMC_CEN 32
#define OMAP_MMC_CFN_DIV 64
#define OMAP_MMC_CEN_TX 8
#define OMAP_MMC_CFN_DIV_TX 16
#define OMAP_MMC_AF_LEVEL ((1 << 7) | 7)
#define OMAP_MMC_CON_CLKD_MSK 0x03FF
#endif
#ifdef CONFIG_ARCH_OMAP730
#ifdef CONFIG_MACH_OMAP_PERSEUS2
static u8 card_state = 1 << 0;
#define OMAP_MMC_OCR 0x00ff8000
#define OMAP_MMC_INT INT_MMC_SDIO1
#define OMAP_MMC_CD  INT_P2_MMC_CD
#define OMAP_MMC_CSDP ((1 << 10) | (1 << 9) | (2 << 7) | (1 << 6) | 1)
#define OMAP_MMC_TX_CCR (1 << 12) | (1 << 10) | (1 << 5) | (1 << 6)
#define OMAP_MMC_RX_CCR (1 << 14) | (1 << 10) | (1 << 5) | (1 << 6)
#define OMAP_MMC_CEN 32
#define OMAP_MMC_CFN_DIV 64
#define OMAP_MMC_CEN_TX 8
#define OMAP_MMC_CFN_DIV_TX 16
#define OMAP_MMC_AF_LEVEL (MMC_BUF_TXDE | (7 << MMC_BUF_AEL_POS))
#define OMAP_MMC_CON_CLKD_MSK MMC_CON_CLKD_MSK
#define OMAP_MMC_CMD_TIMEOUT MMC_STAT_CMD_TIMEOUT
#define OMAP_MMC_DATA_TIMEOUT MMC_STAT_DATA_TIMEOUT
#define OMAP_MMC_CMD_CRC MMC_STAT_CMD_CRC
#define OMAP_MMC_DATA_CRC MMC_STAT_DATA_CRC
#define OMAP_MMC_END_OF_CMD MMC_STAT_END_OF_CMD
#else
#error ERROR -- unsupported OMAP730 target
#endif
#endif

#define OMAP_MMC_MASTER_CLOCK 48000000

static __inline__ void
mmc_delay(void)
{
	udelay(1);
}

#ifdef CONFIG_CEE		/* MVL-CEE */
#include <linux/device.h>

static int omap_mmc_dpm_suspend(struct device *dev, u32 state, u32 level);
static int omap_mmc_dpm_resume(struct device *dev, u32 level);

static struct device_driver omap_mmc_driver_ldm = {
	name:"omap-mmc",
	devclass:NULL,
	probe:NULL,
	suspend:omap_mmc_dpm_suspend,
	resume:omap_mmc_dpm_resume,
	remove:NULL,
};

static struct device omap_mmc_device_ldm = {
	name:"OMAP1510/1610/730 MMC/SD",
	bus_id:"MMC_SD",
	driver:NULL,
	power_state:DPM_POWER_ON,
};

static void
omap_mmc_ldm_driver_register(void)
{
	extern void mpu_public_driver_register(struct device_driver *driver);

	mpu_public_driver_register(&omap_mmc_driver_ldm);
}

static void
omap_mmc_ldm_device_register(void)
{
	extern void mpu_public_device_register(struct device *device);

	mpu_public_device_register(&omap_mmc_device_ldm);
}

static void
omap_mmc_ldm_driver_unregister(void)
{
	extern void mpu_public_driver_unregister(struct device_driver *driver);

	mpu_public_driver_unregister(&omap_mmc_driver_ldm);
}

static void
omap_mmc_ldm_device_unregister(void)
{
	extern void mpu_public_device_unregister(struct device *device);

	mpu_public_device_unregister(&omap_mmc_device_ldm);
}

#endif				/* MVL-CEE */

/**************************************************************************
 *   Clock routines
 **************************************************************************/

static int
omap_mmc_stop_clock(void)
{
	/* Clear 7:0 bits of the MMC_CON to disable clock */

	outw(inw(OMAP_MMC_CON) & ~OMAP_MMC_CON_CLKD_MSK, OMAP_MMC_CON);
	return MMC_NO_ERROR;

}

static int
omap_mmc_set_clock(u32 rate)
{
	int retval;
	DEBUG(3, ": Set rate: %d \n", rate);

	u32 master = OMAP_MMC_MASTER_CLOCK;	/* Default master clock */

	u16 divisor = master / rate;

	retval = omap_mmc_stop_clock();
	if (retval)
		return retval;

	outw((inw(OMAP_MMC_CON) & ~OMAP_MMC_CON_CLKD_MSK) | divisor, 
		OMAP_MMC_CON);
	mmc_delay();

	/* FIX-ME: Time out values */

	outw(0x90, OMAP_MMC_CTO);
	mmc_delay();

	outw(0xffff, OMAP_MMC_DTO);
	mmc_delay();

	g_omap_mmc_data.clock = rate;

	return MMC_NO_ERROR;
}

static void
omap_mmc_set_transfer(u16 block_len, u16 nob)
{
	outw(block_len - 1, OMAP_MMC_BLEN);	/* Only 10:0 bits are valid */
	outw(nob - 1, OMAP_MMC_NBLK);	/* Only 10:0 bits are valid */
	DEBUG(3, "Set transfer: %x \n", inw(OMAP_MMC_BLEN));
}

static void omap_mmc_start_tx_dma_transfer(int size);
static void omap_mmc_start_rx_dma_transfer(int size);
static void omap_mmc_handle_int(struct omap_mmc_data *sd, u16 status);

static void
omap_mmc_dma_reset(int direction, dma_regs_t * regs)
{
	int status;
	/*  Reset to default values */
	regs->csdp = 0;
	regs->ccr = direction;
	regs->cicr = 0x3;
	status = regs->csr;	// read status to clear it.
	regs->cssa_l = 0;
	regs->cssa_u = 0;
	regs->cdsa_l = 0;
	regs->cdsa_u = 0;
	regs->cen = 0;
	regs->cfn = 0;
#ifdef CONFIG_ARCH_OMAP1510
	regs->cfi = 0;
	regs->cei = 0;
#endif

}

static void
omap_mmc_rx_dma_callback(void *data, int size)
{
	g_omap_mmc_data.request->buffer += g_omap_mmc_data.request->block_len;
	g_omap_mmc_data.request->nob--;
	omap_clear_dma(g_omap_mmc_data.rx_dma_regs);
	omap_mmc_handle_int(&g_omap_mmc_data, inw(OMAP_MMC_STAT));
	DEBUG(3, ": RX Transfer finished\n");
}

static void
omap_mmc_tx_dma_callback(void *data, int size)
{
	g_omap_mmc_data.request->buffer += g_omap_mmc_data.request->block_len;
	while (!(inw(OMAP_MMC_STAT) & (1 << 4))) ;	/* Card exit busy state? */
	g_omap_mmc_data.request->nob--;
	omap_clear_dma(g_omap_mmc_data.tx_dma_regs);
	omap_mmc_handle_int(&g_omap_mmc_data, inw(OMAP_MMC_STAT));

	DEBUG(3, ": TX Transfer finished. Status: %d\n", inw(OMAP_MMC_STAT));
}

static void
omap_mmc_start_tx_dma_transfer(int size)
{
	dma_regs_t *regs = g_omap_mmc_data.tx_dma_regs;
	dma_addr_t dma_addr = virt_to_bus(g_omap_mmc_data.request->buffer);
	omap_mmc_dma_reset(eMMCTx, regs);
	consistent_sync(g_omap_mmc_data.request->buffer,
			g_omap_mmc_data.request->block_len, PCI_DMA_TODEVICE);

	/* Configure DMA */
	regs->csdp = OMAP_MMC_CSDP;
	regs->ccr |= OMAP_MMC_TX_CCR;
	regs->cicr = (1 << 5) | (1 << 1) | 1;
	regs->cdsa_l = OMAP_MMC_DATA & 0xffff;
	regs->cdsa_u = OMAP_MMC_DATA >> 16;
	regs->cssa_l = dma_addr & 0xffff;
	regs->cssa_u = dma_addr >> 16;

#ifdef CONFIG_ARCH_OMAP1510
	regs->cen = OMAP_MMC_CEN;	/* 32 words per frame */
	regs->cfn = size / OMAP_MMC_CFN_DIV;
	regs->cfi = 0;
	regs->cei = 1;
#endif

#if defined(CONFIG_ARCH_OMAP1610) || defined(CONFIG_ARCH_OMAP730)
	regs->cen = OMAP_MMC_CEN_TX;
	regs->cfn = size / OMAP_MMC_CFN_DIV_TX;
	regs->ccr2 = 0;
	regs->lch_ctrl = 2;
#endif

	/* Start DMA */

	regs->ccr |= (1 << 7);

}

static void
omap_mmc_start_rx_dma_transfer(int size)
{
	dma_regs_t *regs = g_omap_mmc_data.rx_dma_regs;
	dma_addr_t dma_addr = virt_to_bus(g_omap_mmc_data.request->buffer);
	consistent_sync(g_omap_mmc_data.request->buffer,
			g_omap_mmc_data.request->block_len, PCI_DMA_TODEVICE);
	omap_mmc_dma_reset(eMMCRx, regs);
	/* Configure DMA */
	regs->csdp = (1 << 3) | (1 << 2) | 1;

	//regs->csdp = OMAP_MMC_CSDP;
	regs->ccr |= OMAP_MMC_RX_CCR;
	regs->cicr = (1 << 5) | (1 << 1) | 1;
	regs->cssa_l = OMAP_MMC_DATA & 0xffff;
	regs->cssa_u = OMAP_MMC_DATA >> 16;
	regs->cdsa_l = dma_addr & 0xffff;
	regs->cdsa_u = dma_addr >> 16;
	regs->cen = OMAP_MMC_CEN;	/* 32 words per frame */
	regs->cfn = size / OMAP_MMC_CFN_DIV;

#ifdef CONFIG_ARCH_OMAP1510
	regs->cfi = 0;
	regs->cei = 1;
#endif

#if defined(CONFIG_ARCH_OMAP1610) || defined(CONFIG_ARCH_OMAP730)
	regs->ccr2 = 0;
	regs->lch_ctrl = 2;
#endif

	/* Start DMA */

	regs->ccr |= (1 << 7);

}

static int
omap_mmc_exec_command(struct mmc_request *request)
{

	u16 cmd_mask = 0;

	disable_irq(OMAP_MMC_INT);
	outw(0xffff, OMAP_MMC_STAT);	/* Clear status bits */

	if ((request->cmd == MMC_SEND_OP_COND) ||
	    (request->cmd == MMC_ALL_SEND_CID) ||
	    (request->cmd == MMC_SET_RELATIVE_ADDR) ||
	    (request->cmd == MMC_GO_IRQ_STATE)) {
		/* Set Open Drain Mode for MMC commands 1,2,3,40 */
		cmd_mask |= (1 << 6);
	}

	/* Set response type */

	switch (request->rtype) {
	case RESPONSE_NONE:
		break;
	case RESPONSE_R1:
		cmd_mask |= 1 << 8;
		break;
	case RESPONSE_R1B:
		cmd_mask |= 1 << 8 | 1 << 11;
		break;
	case RESPONSE_R2_CID:
	case RESPONSE_R2_CSD:
		cmd_mask |= 1 << 9;
		break;
	case RESPONSE_R3:
		cmd_mask |= 1 << 8 | 1 << 9;
		break;
	case RESPONSE_R4:
		cmd_mask |= 1 << 10;
		break;
	case RESPONSE_R5:
		cmd_mask |= 1 << 10 | 1 << 8;
		break;
	}

	/* Set command type */
	switch (request->cmd) {

		/* MMC core extra command */
	case MMC_CIM_RESET:
		cmd_mask |= 1 << 7;	/* Initialization sequence send prior to command */
		break;
		/* bc - broadcast - no response */
	case MMC_GO_IDLE_STATE:
	case MMC_SET_DSR:
		break;
		/* bcr - broadcast with response */
	case MMC_SEND_OP_COND:
	case MMC_ALL_SEND_CID:
	case MMC_GO_IRQ_STATE:
		cmd_mask |= 1 << 12;
		break;
		/* adtc - addressed with data transfer */
	case MMC_READ_DAT_UNTIL_STOP:
	case MMC_READ_SINGLE_BLOCK:
	case MMC_READ_MULTIPLE_BLOCK:
		cmd_mask |= 1 << 15 | 1 << 12 | 1 << 13;
		break;
	case MMC_WRITE_DAT_UNTIL_STOP:
	case MMC_WRITE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:
	case MMC_PROGRAM_CID:
	case MMC_PROGRAM_CSD:
	case MMC_SEND_WRITE_PROT:
	case MMC_LOCK_UNLOCK:
	case MMC_GEN_CMD:
		cmd_mask |= 1 << 12 | 1 << 13;
		break;
		/* ac - no data transfer */
	default:
		cmd_mask |= 1 << 13;
	}

	/* Set command index */
	if (cmd_mask & (1 << 7)) {
		/* Use this command to reset card */
		cmd_mask |= MMC_GO_IDLE_STATE;
	} else {
		cmd_mask |= request->cmd;
	}

	/* Set argument */

	outw(request->arg >> 16, OMAP_MMC_ARGH);
	outw(request->arg & 0xffff, OMAP_MMC_ARGL);

	switch (request->cmd) {
	case MMC_READ_SINGLE_BLOCK:
	case MMC_READ_MULTIPLE_BLOCK:
		omap_mmc_set_transfer(request->block_len, request->nob);
		omap_mmc_start_rx_dma_transfer(request->block_len);
		outw((0x1F << 8) | (1 << 15), OMAP_MMC_BUF);	/* Set AF_level to 32 words */
		g_omap_mmc_data.type = RT_READ;
		outw((1 << 7 | 1 << 8), OMAP_MMC_IE);
		break;

	case MMC_WRITE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:
		omap_mmc_set_transfer(request->block_len, request->nob);
		omap_mmc_start_tx_dma_transfer(request->block_len);
		g_omap_mmc_data.type = RT_WRITE;

		outw(OMAP_MMC_AF_LEVEL, OMAP_MMC_BUF);	/* Set AE_level to 1 word */
		outw((1 << 7 | 1 << 8), OMAP_MMC_IE);

		break;
	default:
		outw((1 | 1 << 7 | 1 << 8 | 1 << 14), OMAP_MMC_IE);
		g_omap_mmc_data.type = RT_RESPONSE_ONLY;
		break;
	}

	/* Send command */

	DEBUG(3, ": Send cmd: %x \n Arg: %d\n", cmd_mask, request->arg);

	outw(cmd_mask, OMAP_MMC_CMD);

	enable_irq(OMAP_MMC_INT);

	return MMC_NO_ERROR;
}

static void
omap_mmc_send_command(struct mmc_request *request)
{
	int retval;

	/* TODO: Grab a lock???? */
	g_omap_mmc_data.request = request;
	request->result = MMC_NO_RESPONSE;	/* Flag to indicate don't have a result yet */

	if (request->cmd == MMC_CIM_RESET) {

		/* Reset OMAP MMC hardware */

		retval = omap_mmc_set_clock(MMC_CLOCK_SLOW);
		if (retval) {

			/* If any error occured -> exit with error code */

			request->result = retval;
			mmc_cmd_complete(request);
			return;
		}
	}

	retval = omap_mmc_exec_command(request);

	if (retval) {

		/* If any error occured -> exit with error code */

		request->result = retval;
		mmc_cmd_complete(request);
	}
}

/**************************************************************************/
/* TODO: Fix MMC core for correct response processing */

static void
omap_mmc_get_response(struct mmc_request *request)
{
	u8 *buf = request->response;
	u16 data;

	request->result = MMC_NO_ERROR;	/* Mark this as having a request result of some kind */
	DEBUG(3, ": Get response \n");
	switch (request->rtype) {
	case RESPONSE_R1:
	case RESPONSE_R1B:
		data = inw(OMAP_MMC_RSP7);	/* Get response in little endian format */
		DEBUG(3, ": Response data: %x", data);
		*(buf) = request->cmd;	/* for mmc core */

		*(buf + 1) = data >> 8;
		*(buf + 2) = data & 0xff;

		data = inw(OMAP_MMC_RSP6);
		DEBUG(3, ": %x \n", data);
		*(buf + 3) = data >> 8;
		*(buf + 4) = data & 0xff;
		break;

	case RESPONSE_R3:
	case RESPONSE_R4:
	case RESPONSE_R5:
		data = inw(OMAP_MMC_RSP7);	/* Get response in little endian format */
		DEBUG(3, ": Response data: %x", data);
		*(buf) = 0x3f;	/* for mmc core */
		*(buf + 1) = data >> 8;
		*(buf + 2) = data & 0xff;
		data = inw(OMAP_MMC_RSP6);
		DEBUG(3, " %x \n", data);
		*(buf + 3) = data >> 8;
		*(buf + 4) = data & 0xff;
		break;

	case RESPONSE_R2_CID:
	case RESPONSE_R2_CSD:
		*(buf) = 0x3f;	/* for mmc core */
		data = inw(OMAP_MMC_RSP7);
		*(buf + 1) = data >> 8;
		*(buf + 2) = data & 0xff;
		data = inw(OMAP_MMC_RSP6);
		*(buf + 3) = data >> 8;
		*(buf + 4) = data & 0xff;
		data = inw(OMAP_MMC_RSP5);
		*(buf + 5) = data >> 8;
		*(buf + 6) = data & 0xff;
		data = inw(OMAP_MMC_RSP4);
		*(buf + 7) = data >> 8;
		*(buf + 8) = data & 0xff;
		data = inw(OMAP_MMC_RSP3);
		*(buf + 9) = data >> 8;
		*(buf + 10) = data & 0xff;
		data = inw(OMAP_MMC_RSP2);
		*(buf + 11) = data >> 8;
		*(buf + 12) = data & 0xff;
		data = inw(OMAP_MMC_RSP1);
		*(buf + 13) = data >> 8;
		*(buf + 14) = data & 0xff;
		data = inw(OMAP_MMC_RSP0);
		*(buf + 15) = data >> 8;
		*(buf + 16) = data & 0xff;

	case RESPONSE_NONE:
	default:
		break;
	}
}

static void
omap_mmc_handle_int(struct omap_mmc_data *sd, u16 status)
{
	int retval = MMC_NO_ERROR;
	int flags;

	local_irq_save(flags);

	DEBUG(3, ": Interrupt. Status: %d\n", status);

	/* Data or Command time-out? */

	if (status & (OMAP_MMC_CMD_TIMEOUT | OMAP_MMC_DATA_TIMEOUT)) {
		DEBUG(0, ": MMC/SD status: %d \n", inw(OMAP_MMC_STAT));
		retval = MMC_ERROR_TIMEOUT;
		goto terminate_int;
	}

	/* CRC error? */

	if ((status & (OMAP_MMC_CMD_CRC | OMAP_MMC_DATA_CRC)) &&
	    !(inw(OMAP_MMC_CON) & (1 << 15))) {
		DEBUG(0, ": MMC/SD status: %d \n", inw(OMAP_MMC_STAT));
		retval = MMC_ERROR_CRC;
		goto terminate_int;
	}

	if (((status & OMAP_MMC_END_OF_CMD)
	     && (sd->request->result == MMC_NO_RESPONSE))
	    || (status & 1 << 14))
		omap_mmc_get_response(sd->request);

	switch (g_omap_mmc_data.type) {
	case RT_NO_RESPONSE:
		break;

	case RT_WRITE:
		if (sd->request->nob) {
			/* Start another transfer */
			outw(0xffff, OMAP_MMC_STAT);
			omap_mmc_start_tx_dma_transfer(g_omap_mmc_data.request->
						       block_len);
			return;
		}
		break;

	case RT_READ:
		if (sd->request->nob) {
			/* Start another transfer */
			outw(0xffff, OMAP_MMC_STAT);
			omap_mmc_start_rx_dma_transfer(g_omap_mmc_data.request->
						       block_len);
			return;
		}
		break;

	case RT_RESPONSE_ONLY:
		if (sd->request->result < 0) {
			printk(KERN_INFO
			       "MMC/SD: Illegal interrupt - command hasn't finished\n");
			retval = MMC_ERROR_TIMEOUT;
		}
		break;

	}

      terminate_int:

	outw(0, OMAP_MMC_IE);	/* Clear all interrupts */

	sd->request->result = retval;
	mmc_cmd_complete(sd->request);
	local_irq_restore(flags);
}

/* Handle IRQ */

static void
omap_mmc_int(int irq, void *dev_id, struct pt_regs *regs)
{
	struct omap_mmc_data *sd = (struct omap_mmc_data *) dev_id;
	u16 status = inw(OMAP_MMC_STAT);

	omap_mmc_handle_int(sd, status);

}

/* Detect Card */
static void
omap_mmc_fix_sd_detect(unsigned long nr)
{
#ifdef CONFIG_ARCH_OMAP1510
	card_state = inb(INNOVATOR_FPGA_INFO) & (1 << 4);

	if (card_state == 0) {
		outw((1 << 11), OMAP_MMC_CON);
		mmc_insert(0);
	} else {
		mmc_eject(0);
	}
#endif

#ifdef CONFIG_ARCH_OMAP1610
	u8 state;

	state = inb(MPUIO_INPUT_LATCH_REG) & (1 << 1);
	if (card_state == state) {
		if (card_state == 0) {
			outw((1 << 11), OMAP_MMC_CON);
			mmc_insert(0);

			omap1610_tune_irq(OMAP_MMC_CD, IFL_HIGHLEVEL);
		} else {
			mmc_eject(0);

			omap1610_tune_irq(OMAP_MMC_CD, IFL_LOWLEVEL);
		}
	}
#endif

#ifdef CONFIG_ARCH_OMAP730
	u8 state;

	state = get_p2_mmc_cd_state() & 1;
	if(card_state == state) {
		if ( card_state == 0 ) { 
			/* Slot now has a card */
			outw(MMC_CON_POWER_UP, OMAP_MMC_CON); 
			mmc_insert(0);

			omap730_tune_irq(OMAP_MMC_CD, IFL_HIGHLEVEL);
		} else {
			/* Slot is now empty */
			mmc_eject(0);

			omap730_tune_irq(OMAP_MMC_CD, IFL_LOWLEVEL);	    
		}
	}
#endif
}

static void
omap_mmc_sd_detect_int(int irq, void *dev_id, struct pt_regs *regs)
{
	struct omap_mmc_data *sd = (struct omap_mmc_data *) dev_id;

#ifdef CONFIG_ARCH_OMAP1510
	u8 state;

	state = inb(INNOVATOR_FPGA_INFO) & (1 << 4);
	if (card_state != state)
		mod_timer(&sd->sd_detect_timer,
			  jiffies + ((card_state ? 50 : 500) * HZ) / 1000);
#endif

#ifdef CONFIG_ARCH_OMAP1610
	card_state = inb(MPUIO_INPUT_LATCH_REG) & (1 << 1);
	mod_timer(&sd->sd_detect_timer,
		  jiffies + ((card_state ? 50 : 500) * HZ) / 1000);
#endif

#ifdef CONFIG_ARCH_OMAP730
	card_state = get_p2_mmc_cd_state() & 1;
	mod_timer(&sd->sd_detect_timer,
		  jiffies + ((card_state ? 50 : 500) * HZ) / 1000);
#endif
}

static int
omap_mmc_slot_is_empty(int slot)
{

#ifdef CONFIG_ARCH_OMAP1510
	card_state = inb(INNOVATOR_FPGA_INFO) & (1 << 4);
#endif

#ifdef CONFIG_ARCH_OMAP1610
	card_state = inb(MPUIO_INPUT_LATCH_REG) & (1 << 1);
#endif

#ifdef CONFIG_ARCH_OMAP730
	card_state = get_p2_mmc_cd_state();
#endif

	return card_state != 0;
}

/**************************************************************************
 *
 *                       Hardware initialization                                             
 *
 *************************************************************************/
static void
omap_mmc_slot_up(void)
{

	/* OMAP MMC slot hardware init */

#ifdef CONFIG_ARCH_OMAP1510
	outb(inb(OMAP1510P1_FPGA_POWER) | (1 << 3), OMAP1510P1_FPGA_POWER);
#endif

#if defined(CONFIG_ARCH_OMAP1510) || defined(CONFIG_ARCH_OMAP1610)
	/* Enable MMC/SD clock and disable HI-Z on the MMC.DAT2 pin */

	outl(inl(MOD_CONF_CTRL_0) | (1 << 23) | (1 << 21), MOD_CONF_CTRL_0);

	/* Set up correct pin multiplexing (OMAP1510 mode) */

	/* Configure MMC.DAT3 pin */

	outl(inl(FUNC_MUX_CTRL_D) & ~((1 << 14) | (1 << 13) | (1 << 12)),
	     FUNC_MUX_CTRL_D);

	/* Configure MMC.CLK pin */

	outl(inl(FUNC_MUX_CTRL_A) & ~((1 << 23) | (1 << 22) | (1 << 21)),
	     FUNC_MUX_CTRL_A);

	/* Configure MMC_DAT0_SPI.DI pin */

	outl(inl(FUNC_MUX_CTRL_B) & ~((1 << 2) | (1 << 1) | (1 << 0)),
	     FUNC_MUX_CTRL_B);

	/* Configure MMC.DAT2 pin */

	outl(inl(FUNC_MUX_CTRL_A) & ~((1 << 20) | (1 << 19) | (1 << 18)),
	     FUNC_MUX_CTRL_A);

	/* Configure MMC.DAT1 pin */

	outl(inl(FUNC_MUX_CTRL_A) & ~((1 << 26) | (1 << 25) | (1 << 24)),
	     FUNC_MUX_CTRL_A);

	/* Configure MMC.CMD_SPI.DO pin  */

	outl(inl(FUNC_MUX_CTRL_A) & ~((1 << 29) | (1 << 28) | (1 << 27)),
	     FUNC_MUX_CTRL_A);
#endif

#ifdef CONFIG_ARCH_OMAP730
	/* Pin MUX is set up for the OMAP730/P2 in perseus2.c */
	set_p2_v_sdmmc_state(V_SDMMC_ENABLED);
	set_p2_mmc_wp_state(MMC_WP_DISABLED);

	/* Start up the 48MHz MMC Clock */
	OMAP730_CONFIGURE(PCONF_MMC_DPLL_REQ, ACTIVE);

	*((volatile u32 *)ECO_SPARE1) =  ECO_SPARE1_INITVALUE;
#endif

	/* Configure MMC_CON register */
	outw((1 << 11), OMAP_MMC_CON);

	/* Clear interrupts and status */
	outw(0, OMAP_MMC_IE);	/* Clear all interrupts */
	outw(0xffff, OMAP_MMC_STAT);	/* Clear status bits */
}

static void
omap_mmc_slot_down(void)
{

	disable_irq(OMAP_MMC_CD);
	disable_irq(OMAP_MMC_INT);

	/* Power Off MMC through FPGA */

#ifdef CONFIG_ARCH_OMAP1510
	outb(inb(OMAP1510P1_FPGA_POWER) & ~(1 << 3), OMAP1510P1_FPGA_POWER);
#endif

#if defined(CONFIG_ARCH_OMAP1510) || defined(CONFIG_ARCH_OMAP1610)
	/* Disable MMC/SD clock and enable HI-Z on the MMC.DAT2 pin */

	outl(inl(MOD_CONF_CTRL_0) & ~((1 << 23) | (1 << 21)), MOD_CONF_CTRL_0);
#endif

#ifdef CONFIG_ARCH_OMAP730
	OMAP730_CONFIGURE(PCONF_MMC_DPLL_REQ, INACTIVE);
#endif

	outw(0, OMAP_MMC_CON);

	del_timer_sync(&g_omap_mmc_data.sd_detect_timer);

#ifdef CONFIG_ARCH_OMAP730
	set_p2_v_sdmmc_state(V_SDMMC_DISABLED);
#endif
}

/* Standard PM functions */

static int
omap_mmc_suspend(void)
{
	omap_mmc_stop_clock();
	omap_mmc_slot_down();
	return 0;
}

static void
omap_mmc_resume(void)
{
	omap_mmc_slot_up();
	omap_mmc_set_clock(g_omap_mmc_data.clock);
	/* Set up timers */

	g_omap_mmc_data.sd_detect_timer.function = omap_mmc_fix_sd_detect;
	g_omap_mmc_data.sd_detect_timer.data = (unsigned long) &g_omap_mmc_data;
	init_timer(&g_omap_mmc_data.sd_detect_timer);

	enable_irq(OMAP_MMC_INT);
	enable_irq(OMAP_MMC_CD);	/* Enable card detect IRQ */
}

#ifdef CONFIG_PM

static int
omap_mmc_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{

	switch (req) {
	case PM_SUSPEND:
		mmc_eject(0);
		omap_mmc_suspend();
		break;

	case PM_RESUME:
		omap_mmc_resume();
		omap_mmc_fix_sd_detect(0);
		break;
	}

	return 0;

}

#endif

#ifdef CONFIG_CEE		/* MVL-CEE */

static int
omap_mmc_dpm_suspend(struct device *dev, u32 state, u32 level)
{

	switch (level) {

	case SUSPEND_POWER_DOWN:

		/* Turn off power to MMC/SD */
		omap_mmc_suspend();
		break;
	}

	return 0;

}

static int
omap_mmc_dpm_resume(struct device *dev, u32 level)
{

	switch (level) {
	case RESUME_POWER_ON:

		/* Turn on power to MMC/SD */
		omap_mmc_resume();
		mmc_eject(0);
		omap_mmc_fix_sd_detect(0);
		break;
	}

	return 0;
}

#endif

static int
omap_mmc_slot_init(void)
{
	int retval;

	/* Set up timers */

	g_omap_mmc_data.sd_detect_timer.function = omap_mmc_fix_sd_detect;
	g_omap_mmc_data.sd_detect_timer.data = (unsigned long) &g_omap_mmc_data;
	init_timer(&g_omap_mmc_data.sd_detect_timer);

	/* Basic service interrupt */

	retval = request_irq(OMAP_MMC_INT, omap_mmc_int,
			     SA_INTERRUPT, "omap_mmc_int", &g_omap_mmc_data);
	if (retval) {
		printk(KERN_CRIT "MMC/SD: unable to grab MMC IRQ\n");
		return retval;
	}

	disable_irq(OMAP_MMC_INT);

	/* Card Detect interrupt */

	retval = request_irq(OMAP_MMC_CD, omap_mmc_sd_detect_int,
			     SA_INTERRUPT, "omap_mmc_cd", &g_omap_mmc_data);
#ifdef CONFIG_ARCH_OMAP1610
	if (omap_mmc_slot_is_empty(0) == 0)
		omap1610_tune_irq(OMAP_MMC_CD, IFL_HIGHLEVEL);
#endif
#ifdef CONFIG_ARCH_OMAP730
	if (omap_mmc_slot_is_empty(0) == 0)
		omap730_tune_irq(OMAP_MMC_CD, IFL_HIGHLEVEL);
#endif

	if (retval) {
		printk(KERN_CRIT "MMC/SD: unable to grab MMC_CD_IRQ\n");
		goto err1;
	}

	disable_irq(OMAP_MMC_CD);

	if (omap_request_dma
	    (eMMCRx, "MMC/SD Rx DMA", (dma_callback_t) omap_mmc_rx_dma_callback,
	     &g_omap_mmc_data, &(g_omap_mmc_data.rx_dma_regs))) {
		printk(KERN_ERR "Failed to request MMC Rx DMA \n");
		goto err2;
	}
	if (omap_request_dma
	    (eMMCTx, "MMC/SD Tx DMA", (dma_callback_t) omap_mmc_tx_dma_callback,
	     &g_omap_mmc_data, &(g_omap_mmc_data.tx_dma_regs))) {
		printk(KERN_ERR "Failed to request MMC Tx DMA \n");
		goto err3;
	}
#ifdef CONFIG_PM
	pm_register(PM_UNKNOWN_DEV, PM_SYS_UNKNOWN, omap_mmc_pm_callback);
#endif
	omap_mmc_slot_up();

	enable_irq(OMAP_MMC_CD);	/* Enable IRQ to detect card */
	goto noerr;

      err3:omap_free_dma(g_omap_mmc_data.tx_dma_regs);
      err2:omap_free_dma(g_omap_mmc_data.rx_dma_regs);
      err1:free_irq(OMAP_MMC_INT, &g_omap_mmc_data);
      noerr:return retval;

}

static void
omap_mmc_slot_cleanup(void)
{
	long flags;

	local_irq_save(flags);

	omap_mmc_slot_down();

	mmc_eject(0);
	omap_free_dma(g_omap_mmc_data.tx_dma_regs);
	omap_free_dma(g_omap_mmc_data.rx_dma_regs);

	free_irq(OMAP_MMC_CD, &g_omap_mmc_data);
	free_irq(OMAP_MMC_INT, &g_omap_mmc_data);

	local_irq_restore(flags);
}

/***********************************************************/

static struct mmc_slot_driver dops = {
	.owner = THIS_MODULE,
	.name = "OMAP1510/1610/730 MMC",
	.ocr = OMAP_MMC_OCR,
	.flags = MMC_SDFLAG_MMC_MODE,

	.init = omap_mmc_slot_init,
	.cleanup = omap_mmc_slot_cleanup,
	.is_empty = omap_mmc_slot_is_empty,
	.send_cmd = omap_mmc_send_command,
	.set_clock = omap_mmc_set_clock,
};

static int __init
omap_mmc_init(void)
{
	int retval;

#ifdef CONFIG_CEE		/* MVL-CCE */
	omap_mmc_ldm_device_register();
	omap_mmc_ldm_driver_register();
#endif				/* MVL-CCE */

	retval = mmc_register_slot_driver(&dops, 1);
	if (retval < 0)
		printk(KERN_INFO "MMC/SD: unable to register slot\n");

	return retval;
}

static void __exit
omap_mmc_cleanup(void)
{
#ifdef CONFIG_CEE		/* MVL-CCE */
	omap_mmc_ldm_device_unregister();
	omap_mmc_ldm_driver_unregister();
#endif				/* MVL-CCE */

	mmc_unregister_slot_driver(&dops);
}

module_init(omap_mmc_init);
module_exit(omap_mmc_cleanup);

MODULE_AUTHOR("Alexey Lugovskoy");
MODULE_DESCRIPTION("OMAP1510/1610/730 Innovator MMC/SD");
MODULE_LICENSE("GPL");
EXPORT_NO_SYMBOLS;
