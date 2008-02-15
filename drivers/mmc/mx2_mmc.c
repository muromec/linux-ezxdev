/*
 * BRIEF MODULE DESCRIPTION
 * Low-level MMC functions for the MX2 MMC controller
 * SD support is not present
 *
 * Based on: mx1_mmc.c
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */
#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/unaligned.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <linux/mmc/mmc_ll.h>
#include <asm/arch/gpio.h>

#include <linux/pm.h>
#include <asm/arch/pll.h>

#if 1 /*CEE LDM*/
#include <linux/device.h>
#include <linux/dpm.h>

extern void mx21_ldm_bus_register(struct device *device,
                          struct device_driver *driver);
extern void mx21_ldm_bus_unregister(struct device *device,
                          struct device_driver *driver);
static int
__ldm_suspend(struct device *dev, u32 state, u32 level);

static int
__ldm_resume(struct device *dev, u32 level);

#ifdef CONFIG_DPM
static struct constraints mx2mmc_constraints = {
	.count = 1,
	.param = {{DPM_MD_HCLK, 0, 0}},	/*to be initialized at module init time */
	.asserted = 1,
};
#endif

static struct device_driver __driver_ldm = {
	.name = "mx2_mmc",
	.devclass = NULL,
	.probe = NULL,
	.suspend = __ldm_suspend,
	.resume = __ldm_resume,
	.scale = NULL,
	.remove = NULL,
};

static struct device __device_ldm = {
	.name = "MX2_MMC",
	.bus_id = "mx2_mmc",
	.driver = &__driver_ldm,
	.power_state = DPM_POWER_ON,
#ifdef CONFIG_DPM
	.constraints = &mx2mmc_constraints,
#endif
};

#endif

#include "mx2_mmcsd_def.h"


/* MMC Port Selection */
#define MMC_BASE SDHC_1

enum mx2_request_type {
	RT_NO_RESPONSE,
	RT_RESPONSE_ONLY,
	RT_READ,
	RT_WRITE
};

struct mx2_mmc_data {
	u32 clock;		/* Current clock frequency */
	struct mmc_request *request;	/* Current request */
	enum mx2_request_type type;
};

static struct mx2_mmc_data g_mx2_mmc_data;
static u16 card_state;
static u8 bit_width_flag = 0;	/* 1 bit bus width */
static int rx_counter = 0;
static int tx_counter = 0;
static char mx2_mmc_inserted;
void mmcsd_socket1_irq_handler(int irq, void *dev_id, struct pt_regs *regs);

static __inline__ void
mmc_delay(void)
{
	udelay(1);
}

/**************************************************************************
 *   Clock routines
 **************************************************************************/

u32
mx2_mmc_softreset(void)
{
	/* System Reset */
	SDHC_STR_STP_CLK(MMC_BASE) = 0x8;

	SDHC_STR_STP_CLK(MMC_BASE) = 0x9;
	SDHC_STR_STP_CLK(MMC_BASE) = 0x1;
	SDHC_STR_STP_CLK(MMC_BASE) = 0x1;
	SDHC_STR_STP_CLK(MMC_BASE) = 0x1;
	SDHC_STR_STP_CLK(MMC_BASE) = 0x1;
	SDHC_STR_STP_CLK(MMC_BASE) = 0x1;
	SDHC_STR_STP_CLK(MMC_BASE) = 0x1;
	SDHC_STR_STP_CLK(MMC_BASE) = 0x1;
	SDHC_STR_STP_CLK(MMC_BASE) = 0x1;

	/* Set MMC Response Time-Out Register */
	SDHC_RESPONSE_TO(MMC_BASE) = 0xFF;

	/* Set Block length register */
	SDHC_BLK_LEN(MMC_BASE) = 512;

	/* Set MMC Number of Blocks Register */
	SDHC_NOB(MMC_BASE) = 1;

	return 0;

}

static int
mx2_mmc_stop_clock(void)
{
	/* Clear bit 1 of STR_STP_CLK to disable clock */

	do {
		SDHC_STR_STP_CLK(MMC_BASE) &= ~MMCSD_CLOCK_MASK;
		SDHC_STR_STP_CLK(MMC_BASE) |= 0x01;
	}
	while (SDHC_STATUS(MMC_BASE) & 0x100);

	return 0;

}

u32
mx2_clock_run(void)
{
	return (SDHC_STATUS(MMC_BASE)) & 0x100;
}

static int
mx2_mmc_start_clock(void)
{

	u8 count = 0;
	do {
		if (count++ == 0) {
			/* do this every 256 loops */
			SDHC_STR_STP_CLK(MMC_BASE) &= ~MMCSD_CLOCK_MASK;
			SDHC_STR_STP_CLK(MMC_BASE) |= 0x2;
		}
	}
	while (!mx2_clock_run());

	return 0;

}

static int
mx2_mmc_set_clock(u32 rate)
{
	u8 clk_presc, clk_divider;
	u8 divisor;

	DEBUG(3, ": Set rate: %d \n", rate);

	if (rate == 0) {
		/* Disable MMC_SD_CLK */
		mx2_mmc_stop_clock();
		return MMC_NO_ERROR;
	}

	divisor = mx_module_get_clk(PERCLK2) / rate;

	for (clk_presc = 0; clk_presc <= 10; clk_presc++) {
		if ((clk_divider = divisor / (1 << clk_presc)) <= 8)
			break;	/* clk_divider=divisor/2**clk_presc */
	}

	mx2_mmc_stop_clock();

	clk_divider = clk_divider & 0xf;
	clk_presc = clk_presc & 0xfff;

	if (rate == MMC_CLOCK_SLOW)
		SDHC_CLK_RATE(MMC_BASE) = (8 << 4) | 0xf;
	else
		SDHC_CLK_RATE(MMC_BASE) = (clk_presc << 4) | 2;

	mmc_delay();

	/* Time out values */

	mx2_mmc_start_clock();

	SDHC_RESPONSE_TO(MMC_BASE) = 0xFFFF;
	SDHC_READ_TO(MMC_BASE) = 0x2DB4;

	g_mx2_mmc_data.clock = rate;

	return MMC_NO_ERROR;
}

static void
mx2_mmc_set_transfer(u16 block_len, u16 nob)
{
	SDHC_BLK_LEN(MMC_BASE) = block_len;
	SDHC_NOB(MMC_BASE) = nob;
}

static void
mx2_mmc_rx(void)
{
u16 *buf;
u32 status;
	int i = 0;

	{
		buf = (u16 *)((u32)g_mx2_mmc_data.request->buffer + rx_counter);
		status = SDHC_STATUS(MMC_BASE) & 0xFFFF;
		while (1) {

			*buf++ = (u16) SDHC_BUFFER_ACCESS(MMC_BASE);
			if (SDHC_STATUS(MMC_BASE) & MX2STAT_CRC_READ_ERR) {
				DEBUG(3, ": CRC_ERR %04x \n",
				      SDHC_STATUS(MMC_BASE));
			}
			if (SDHC_STATUS(MMC_BASE) & MX2STAT_TIME_OUT_READ) {
				DEBUG(3, ": TIMEOUT %04x \n",
				      SDHC_STATUS(MMC_BASE));
			}

			status = SDHC_STATUS(MMC_BASE) & 0xFFFF;
			i++;
			if (bit_width_flag) {
				if (i == 32)
					break;
			} else {
				if (i == 8)
					break;
			}

		}
	}
	rx_counter += i * 2;

	if (rx_counter >= g_mx2_mmc_data.request->block_len) {

		DEBUG(3, ": RX Transfer finished\n");

		g_mx2_mmc_data.request->nob--;

	}

}

static void
mx2_mmc_tx(void)
{
u16 *buf;
int i = 0;

	buf = (u16 *)((u32)g_mx2_mmc_data.request->buffer + tx_counter);
	if (tx_counter <
	    g_mx2_mmc_data.request->block_len * g_mx2_mmc_data.request->nob) {
		while (1) {
			(u16) SDHC_BUFFER_ACCESS(MMC_BASE) = *buf++;
			i++;
			if (bit_width_flag) {
				if (i == 32)
					break;
			} else {
				if (i == 8)
					break;
			}
		}
		tx_counter += i * 2;
	}
	if (tx_counter >= g_mx2_mmc_data.request->block_len) {
		SDHC_INT_MASK(MMC_BASE) = MX2_WRITE_OP_DONE_MASK;

	}

}

static int
mx2_mmc_exec_command(struct mmc_request *request)
{
	u16 dat_control = 0;
	u16 int_mask = 0;

	SDHC_INT_MASK(MMC_BASE) = MX2_AUTO_CARD_DETECT_MASK & 0x7f;

	int_mask = 0x7b;

	mx2_mmc_stop_clock();	/* Clear status bits */
	switch (request->cmd) {
	case MMC_READ_SINGLE_BLOCK:
		mx2_mmc_set_transfer(request->block_len, request->nob);
		break;
	case MMC_WRITE_BLOCK:
		mx2_mmc_set_transfer(request->block_len, request->nob);
		break;
	}

	dat_control = 0;

	/* Set response type */

	switch (request->rtype) {
	case RESPONSE_NONE:
		break;
	case RESPONSE_R1:
	case RESPONSE_R1B:
		dat_control |= MMCSDB_R1;
		break;
	case RESPONSE_R2_CID:
	case RESPONSE_R2_CSD:
		dat_control |= MMCSDB_R2;
		break;
	case RESPONSE_R3:
		dat_control |= MMCSDB_R3;
		break;
	case RESPONSE_R4:
		dat_control |= MMCSDB_R4;
		break;
	case RESPONSE_R5:
		dat_control |= MMCSDB_R5;
		break;
	}

	if (bit_width_flag) {
		dat_control |= MMCSD_BUS_4BIT;
	}

	/* Set command type */
	switch (request->cmd) {

	case MMC_GO_IDLE_STATE:
		dat_control |= MMCSDB_INIT;
		break;
	case MMC_READ_SINGLE_BLOCK:
	case MMC_READ_MULTIPLE_BLOCK:
		dat_control |= MMCSDB_DATEN;
		break;

	case MMC_WRITE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:

		dat_control |= MMCSDB_DATEN | MMCSDB_WRRD;
		break;
	case MMC_SELECT_CARD:
		if (request->arg != 0)
			dat_control |= MMCSDB_BSY | MMCSDB_R1;
		else
			dat_control |= MMCSDB_BSY;
		break;
	case MMC_LOCK_UNLOCK:
		dat_control |= MMCSDB_DATEN | MMCSDB_WRRD;
		break;
	}

	/* Send command */

	DEBUG(3, ": Send cmd : %d  Arg: %x\n", request->cmd, request->arg);
	SDHC_CMD(MMC_BASE) = request->cmd;

	/* Set argument */
	SDHC_ARGH(MMC_BASE) = (request->arg) >> 16;
	SDHC_ARGL(MMC_BASE) = (request->arg) & 0xffff;
	SDHC_CMD_DAT_CONT(MMC_BASE) = dat_control;

	if (request->cmd == MMC_READ_SINGLE_BLOCK
	    || request->cmd == MMC_READ_MULTIPLE_BLOCK) {
		g_mx2_mmc_data.type = RT_READ;
		rx_counter = 0;
	} else if (request->cmd == MMC_WRITE_BLOCK
		   || request->cmd == MMC_WRITE_MULTIPLE_BLOCK) {
		g_mx2_mmc_data.type = RT_WRITE;
		tx_counter = 0;
	} else if (request->rtype == RESPONSE_NONE) {
		g_mx2_mmc_data.type = RT_NO_RESPONSE;
	} else {
		g_mx2_mmc_data.type = RT_RESPONSE_ONLY;
	}

	SDHC_INT_MASK(MMC_BASE) = int_mask;
	mx2_mmc_start_clock();

	return MMC_NO_ERROR;
}

static void
mx2_mmc_send_command(struct mmc_request *request)
{
	int retval;

	g_mx2_mmc_data.request = request;
	request->result = MMC_NO_RESPONSE;	/* Flag to indicate don't have a result yet */

	if (request->cmd == MMC_CIM_RESET) {

		/* Reset MX2 MMC hardware */
		mx2_mmc_softreset();
		retval = mx2_mmc_set_clock(MMC_CLOCK_SLOW);
		if (retval) {
			/* If any error occured -> exit with error code */

			request->result = retval;
			mmc_cmd_complete(request);
			return;
		}
		request->result = MMC_NO_ERROR;
		request->rtype = RESPONSE_NONE;
		mmc_cmd_complete(request);
		return;
	}

	retval = mx2_mmc_exec_command(request);

	if (retval) {

		/* If any error occured -> exit with error code */

		request->result = retval;
		mmc_cmd_complete(request);
	}

}

/**************************************************************************/
/* TODO: Fix MMC core for correct response processing */

static void
mx2_mmc_get_response(struct mmc_request *request)
{
	int i;
	u16 rs;
	u32 temp;

	request->result = MMC_NO_ERROR;	/* Mark this as having a request result of some kind */
	DEBUG(3, ": Get response \n");
	switch (request->rtype) {
	case RESPONSE_R1:
	case RESPONSE_R1B:

		request->response[0] = request->cmd;

		temp = SDHC_RES_FIFO(MMC_BASE);
		rs = (u16) temp;
		request->response[1] = (u8) (rs & 0x00FF);

		temp = SDHC_RES_FIFO(MMC_BASE);
		rs = (u16) temp;
		request->response[2] = (u8) (rs >> 8);
		if (request->cmd == 3 && request->response[1] == 0
		    && request->response[2] == 0x40) {
			request->response[2] &=
			    request->response[2] & ~(1 << 6);
		}		/* mmc device does not clear this bit after APP_CMD command for some mmc cards */
		request->response[3] = (u8) (rs & 0x00FF);

		temp = SDHC_RES_FIFO(MMC_BASE);
		rs = (u16) temp;
		request->response[4] = (u8) (rs >> 8);
		break;

	case RESPONSE_R3:
	case RESPONSE_R4:
	case RESPONSE_R5:

		request->response[0] = 0x3f;

		temp = SDHC_RES_FIFO(MMC_BASE);
		rs = (u16) temp;
		request->response[1] = (u8) (rs & 0x00FF);

		temp = SDHC_RES_FIFO(MMC_BASE);
		rs = (u16) temp;
		request->response[2] = (u8) (rs >> 8);
		request->response[3] = (u8) (rs & 0x00FF);

		temp = SDHC_RES_FIFO(MMC_BASE);
		rs = (u16) temp;
		request->response[4] = (u8) (rs >> 8);

		break;

	case RESPONSE_R2_CID:
	case RESPONSE_R2_CSD:
		request->response[0] = 0x3f;	/* for mmc core */

		for (i = 0; i < 8; i++) {

			temp = SDHC_RES_FIFO(MMC_BASE);
			rs = (u16) temp;
			request->response[2 * i + 1] = (u8) (rs >> 8);
			request->response[2 * i + 2] = (u8) (rs & 0x00FF);

		}

	case RESPONSE_NONE:
	default:
		break;
	}

}

static void
mx2_mmc_handle_int(struct mx2_mmc_data *sd)
{
	int retval = MMC_NO_ERROR;
	u16 status = SDHC_STATUS(MMC_BASE);

	mx2_mmc_stop_clock();	/* Clear status bits */
	SDHC_INT_MASK(MMC_BASE) = MX2_AUTO_CARD_DETECT_MASK & 0x7f;	/* mask and clear all interrupts */

	/* Data or Command time-out? */

	if (status & (MX2STAT_TIME_OUT_RESP | MX2STAT_TIME_OUT_READ)) {
		DEBUG(1, " TIMEOUT: MMC status: %x \n", status);
		retval = MMC_ERROR_TIMEOUT;
		sd->request->result = MMC_ERROR_TIMEOUT;
		mx2_mmc_get_response(sd->request);
		goto terminate_int;
	}

	/* CRC error? */

	if ((status & (MX2STAT_RESP_CRC_ERR /* | MX2STAT_CRC_READ_ERR */ ))) {
		DEBUG(1, " MMCSD_RESP_CRC_ERR: MMC status: %x \n", status);
		retval = MMC_ERROR_CRC;
		goto terminate_int;
	}

	if (((status & MX2STAT_END_CMD_RESP)
	     || (status & MX2STAT_DATA_TRANS_DONE)
	     || (status & MX2STAT_WRITE_OP_DONE))
	    && (sd->request->result == MMC_NO_RESPONSE)) {

		mx2_mmc_get_response(sd->request);
	}

	switch (g_mx2_mmc_data.type) {
	case RT_NO_RESPONSE:
		break;

	case RT_WRITE:
		if (sd->request->nob) {
			/* Start another transfer */
		}
		break;

	case RT_READ:
		if (sd->request->nob) {
			/* Start another transfer */
		}
		break;

	case RT_RESPONSE_ONLY:
		if (sd->request->result < 0) {
			printk(KERN_INFO
			       "MMC: Illegal interrupt - command hasn't finished\n");
			retval = MMC_ERROR_TIMEOUT;
		}
		break;

	}

      terminate_int:
	sd->request->result = retval;
	mmc_cmd_complete(sd->request);
}

/* Handle IRQ */

static void
mx2_mmc_int(int irq, void *dev_id, struct pt_regs *regs)
{
	struct mx2_mmc_data *sd = (struct mx2_mmc_data *) dev_id;
	u32 status = SDHC_STATUS(MMC_BASE) & 0xFFFF;

	DEBUG(4, " 0x%x\n", SDHC_STATUS(MMC_BASE));
	if ((sd->request->cmd == MMC_READ_SINGLE_BLOCK
	     || sd->request->cmd == MMC_READ_MULTIPLE_BLOCK)) {
		if (status & MX2STAT_APPL_BUFF_FF) {
			mx2_mmc_rx();
		} else if ((status & MX2STAT_DATA_TRANS_DONE)
			   || ((status & MX2STAT_END_CMD_RESP)
			       && rx_counter >=
			       g_mx2_mmc_data.request->block_len)) {
			mx2_mmc_handle_int(sd);
		}
	} else
	    if ((sd->request->cmd == MMC_WRITE_BLOCK
		 || sd->request->cmd == MMC_WRITE_MULTIPLE_BLOCK)) {
		if (status & MX2STAT_WRITE_OP_DONE) {
			mx2_mmc_handle_int(sd);
		} else if (status & MX2STAT_DATA_TRANS_DONE) {
		} else if (status & MX2STAT_APPL_BUFF_FE) {
			mx2_mmc_tx();
		}
	} else {
		mx2_mmc_handle_int(sd);
	}
}

static int
mx2_mmc_slot_is_empty(int slot)
{
	card_state = SDHC_STATUS(MMC_BASE) & MX2STAT_CARD_PRESENCE;
	return card_state == 0;
}

/**************************************************************************
 *
 *                       Hardware initialization
 *
 *************************************************************************/
static int
mx2_mmc_slot_up(void)
{

	int retval;

	/* MX2 MMC slot hardware init */
	mx_module_clk_open(IPG_MODULE_SDHC1);

	/* Perclock 2 */

	/* GPIO */

	retval = mx2_register_gpios(PORT_E, (1 << 22) |	/* MMC: SD_CMD */
				    (1 << 23) |	/* MMC: SD_CLK */
				    (1 << 21) |	/* MMC: SD_DAT */
				    (1 << 20) |	/* MMC: SD_DAT */
				    (1 << 19) |	/* MMC: SD_DAT */
				    (1 << 18),	/* MMC: SD_DAT */
				    PRIMARY | NOINTERRUPT);

	if (retval < 0)
		goto error;
	return retval;

      error:
	mx2_unregister_gpios(PORT_E,
			     (1 << 18) | (1 << 19) | (1 << 20) | (1 << 21) | (1
									      <<
									      22)
			     | (1 << 23));
	return retval;
}

static void
mx2_mmc_slot_down(void)
{

	disable_irq(INT_SDHC1);

	/* Power Off MMC through FPGA */

	/* Disable MMC clock and enable HI-Z on the MMC.DAT2 pin */

}

/* Standard PM functions */

static int
mx2_mmc_suspend(void)
{
	mx2_gpio_mask_intr(PORT_D, 25);
	if (mx2_mmc_inserted) {
		mmc_eject(0);
		/*set it to be negative level trigger detect card withdraw */
		mx2_gpio_config_intr(PORT_D, 25, NEGATIVE_LEVEL, mmcsd_socket1_irq_handler);
		mx2_mmc_inserted = 0;
	}
	mx2_mmc_slot_down(); /*disable MMC IRQ*/
	mx2_mmc_stop_clock();
	mx_module_clk_close(IPG_MODULE_SDHC1);
	return 0;
}

static void
mx2_mmc_resume(void)
{
	mx_module_clk_open(IPG_MODULE_SDHC1);
	mx2_mmc_set_clock(g_mx2_mmc_data.clock);
	enable_irq(INT_SDHC1);
	mx2_gpio_unmask_intr(PORT_D, 25);
}

#ifdef CONFIG_PM

static int
mx2_mmc_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{

	switch (req) {
	case PM_SUSPEND:
		mx2_mmc_suspend();
		break;

	case PM_RESUME:
		mx2_mmc_resume();
		break;
	}

	return 0;

}

#endif
#if 1 /*CEE LDM*/
static int
__ldm_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		mx2_mmc_suspend();
		break;
	}
	return 0;
}

static int
__ldm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		mx2_mmc_resume();
		break;
	}
	return 0;
}
#endif


void
mmcsd_socket1_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	u32 trigger;

	trigger = mx2_gpio_get_intr_config(PORT_D, 25);
	if (mx2_gpio_intr_status_bit(PORT_D, 25)) {

		if (trigger == 2)	/* it is positive level now */
		{
			if (mx2_mmc_inserted) {
				mmc_eject(0);
				/*set it to be negative level trigger detect card withdraw */
				mx2_gpio_config_intr(PORT_D, 25, NEGATIVE_LEVEL, mmcsd_socket1_irq_handler);
				mx2_mmc_inserted = 0;
			}
		}
		if (trigger == 3)	/* it is negative level now */
		{
			if (!mx2_mmc_inserted) {
				mmc_insert(0);
				/*set it to be positive level trigger detect card insert */
				mx2_gpio_config_intr(PORT_D, 25, POSITIVE_LEVEL, mmcsd_socket1_irq_handler);
				mx2_mmc_inserted = 1;
			}
		}
		mx2_gpio_clear_intr(PORT_D, 25);
	}
}

#define MX2ADS_MMC_MIN_HCLK  33000

static int
mx2_mmc_slot_init(void)
{
	int retval;
	long flags;

#if 1 /*CEE LDM*/
	if (MX2ADS_MMC_MIN_HCLK > mx_module_get_clk(HCLK) / 1000) {
		printk(KERN_ERR "cannot initialize MMC - HCLK too slow\n");
		return -EPERM;
	}
#endif

	/* Basic service interrupt */

	local_irq_save(flags);

	mx2_mmc_slot_up();
	mx2_mmc_softreset();

	/* check hardware revision */
	if ((SDHC_REV_NO(MMC_BASE) & 0xFFFF) != 0x0400) {
		printk(KERN_ERR "MMC Hardware revision = 0x%x",
		       SDHC_REV_NO(MMC_BASE));
	}

	mx2_register_gpio(PORT_D, 25, GPIO | INPUT | TRISTATE);
	mx2_gpio_unmask_intr(PORT_D, 25);	/* enable */
	mx2_gpio_config_intr(PORT_D, 25, NEGATIVE_LEVEL,
			     mmcsd_socket1_irq_handler);
	mx2_mmc_inserted = 0;

	SDHC_INT_MASK(MMC_BASE) = MX2_AUTO_CARD_DETECT_MASK & 0x7f;	/* Clear all interrupts */

	retval = request_irq(INT_SDHC1, mx2_mmc_int,
			     SA_INTERRUPT, "mx2_mmc_int", &g_mx2_mmc_data);

	if (retval) {
		printk(KERN_CRIT "MMC: unable to grab MMC IRQ\n");
		return retval;
	}
#ifdef CONFIG_PM
	pm_register(PM_UNKNOWN_DEV, PM_SYS_UNKNOWN, mx2_mmc_pm_callback);
#endif
#if 1 /*CEE LDM*/
#ifdef CONFIG_DPM
	mx2mmc_constraints.param[0].min = MX2ADS_MMC_MIN_HCLK;	/*in KHz */
	mx2mmc_constraints.param[0].max = mx_module_get_clk(MPLL) / 1000;	/* unlimited */
#endif
	mx21_ldm_bus_register(&__device_ldm, &__driver_ldm);
#endif

	local_irq_restore(flags);

	return retval;
}

static void
mx2_mmc_slot_cleanup(void)
{
	long flags;

	local_irq_save(flags);

	mx2_mmc_slot_down();

	free_irq(INT_SDHC1, &g_mx2_mmc_data);
	mx_module_clk_close(IPG_MODULE_SDHC1);

	local_irq_restore(flags);

	mx2_unregister_gpio(PORT_D, 25);
#ifdef CONFIG_PM
	pm_unregister_all(mx2_mmc_pm_callback);
#endif
#if 1 /*CEE LDM*/
	mx21_ldm_bus_unregister(&__device_ldm, &__driver_ldm);
#endif
}

/***********************************************************/

static struct mmc_slot_driver dops = {
	.owner = THIS_MODULE,
	.name = "MX2 MMC",
	.ocr = 0x00ffc000,
	.flags = MMC_SDFLAG_MMC_MODE,

	.init = mx2_mmc_slot_init,
	.cleanup = mx2_mmc_slot_cleanup,
	.is_empty = mx2_mmc_slot_is_empty,
	.send_cmd = mx2_mmc_send_command,
	.set_clock = mx2_mmc_set_clock
};

int __init
mx2_mmc_init(void)
{
	int retval;

	retval = mmc_register_slot_driver(&dops, 1);
	if (retval < 0)
		printk(KERN_INFO "MMC: unable to register slot\n");

	return retval;
}

void __exit
mx2_mmc_cleanup(void)
{
	mmc_unregister_slot_driver(&dops);
	mx2_unregister_gpios(PORT_E,
			     (1 << 18) | (1 << 19) | (1 << 20) | (1 << 21) | (1
									      <<
									      22)
			     | (1 << 23));
}

module_init(mx2_mmc_init);
module_exit(mx2_mmc_cleanup);

MODULE_DESCRIPTION("MX2 MMC");
MODULE_LICENSE("GPL");
EXPORT_NO_SYMBOLS;
