/*
 *  linux/arch/arm/mach-mx2ads/dma.c
 *
 *  Copyright (C) 1995-2000 Russell King
 *  Copyright (C) 2003 MontaVista Software Inc <source@mvista.com>,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * MX2-specific DMA handling
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>

#include <asm/delay.h>
#include <asm/dma.h>
#include <asm/mach/dma.h>
#include <asm/arch/dma.h>

#include <asm/arch/hardware.h>
#include <asm/arch/pll.h>

#define PKMOD "MX2 DMA: "

static dma_t *mx2_dma_chan;	/* keep address of the upper-layer dma structure */
static dma_callback_t dma_callback[MAX_DMA_CHANNELS];	/* pointers to DMA IRQ callback functions for every channel */
static mx2dma2dset_t dma_2d_lock[MAX_DMA_2D_REG_SETS];

/* handle dma irq requests */
static void
dma_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	dmach_t channel;
	int status = 0;

	channel = irq - 32;

	if (DMA_DISR & (1 << channel))
		status |= DMA_DONE;

	if (DMA_DBTOSR & (1 << channel))
		status |= DMA_BURST_TIMEOUT;

	if (DMA_DSESR & (1 << channel))
		status |= DMA_TRANSFER_ERROR;

	if (DMA_DBOSR & (1 << channel))
		status |= DMA_BUFFER_OVERFLOW;

	if (DMA_DRTOSR & (1 << channel))
		status |= DMA_REQUEST_TIMEOUT;

	dma_callback[channel] (channel, dev_id, status);	/* irq is never requested if no callback specified */

	DMA_DISR = 1 << channel;
	DMA_DBTOSR = 1 << channel;
	DMA_DRTOSR = 1 << channel;
	DMA_DSESR = 1 << channel;
	DMA_DBOSR = 1 << channel;
}

/* must be called after request_dma(), and when dma is not enabled */
int
mx2_request_dma_irq(dmach_t channel, dma_callback_t callback, void *dev_id)
{
	dma_t *dma;

	if (channel >= MAX_DMA_CHANNELS)
		return -EINVAL;
	dma = mx2_dma_chan + channel;

	if (!callback)
		return -EINVAL;

	if (!dma->lock) {
		return -EINVAL;
	}
	if (dma->active != 0)
		return -EBUSY;

	int err = 0;

	dma->dma_irq = channel + 32;
	err = request_irq(dma->dma_irq, dma_irq_handler, SA_INTERRUPT,
			  dma->device_id, dev_id);

	if (err) {
		return err;
	}

	dma_callback[channel] = callback;

	DMA_DIMR &= ~(1 << channel);	/* unmask interrupt */

	return 0;
}

/* must be called before free_dma(), and when dma is not enabled */
void
mx2_free_dma_irq(dmach_t channel, void *dev_id)
{
	dma_t *dma;

	if (channel >= MAX_DMA_CHANNELS)
		return;

	dma = mx2_dma_chan + channel;

	if (!dma->lock) {
		return;
	}

	if (dma->active != 0)
		return;

	if (dma_callback[channel]) {
		free_irq(dma->dma_irq, dev_id);
		dma_callback[channel]=0;
		DMA_DIMR |= (1 << channel);	/* mask interrupt */
	} else
		printk(KERN_ERR PKMOD
		       "trying to free free DMA IRQ for channel %d\n", channel);

}

/*request a 2d register set
  set: register set number, 0 or 1
  returns: 0 - success, < 0 - failure
 */
int
mx2_request_2d_dma(mx2dma2dset_t set)
{
	if (set >= MAX_DMA_2D_REG_SETS)
		return -EINVAL;
	if (xchg(&dma_2d_lock[set], 1) != 0)
		return -EBUSY;
	return 0;
}

/*free a requested 2d register set
  set: register set number, 0 or 1
 */
void
mx2_free_2d_dma(mx2dma2dset_t set)
{
	if (set >= MAX_DMA_2D_REG_SETS)
		return;
	if (xchg(&dma_2d_lock[set], 0) == 0)
		printk(KERN_ERR PKMOD
		       "trying to free free 2D register set %d\n", set);
}

/*
 * STANDARD DMA FUNCTION PROCESSING BELOW
 */

int
mx2_request_dma(dmach_t channel, dma_t * dma)
{
	DMA_DCR |= 0x1;	/* enable DMA module */
	return 0;
}

void
mx2_free_dma(dmach_t channel, dma_t * dma)
{
	dma_t *tmp_dma;
	int i;

	if (dma_callback[channel]) {
		printk(KERN_ERR PKMOD
		       "trying to free DMA channel %d with non-free IRQ\n",
		       channel);
		return;
	}

	for (i = 0, tmp_dma = mx2_dma_chan; i < MAX_DMA_CHANNELS;
	     i++, tmp_dma++) {
		if (tmp_dma->lock)
			return;
	}

	DMA_DCR &= ~0x1;	/* none of the channels are in-use */
}

void
mx2_enable_dma(dmach_t channel, dma_t * dma)
{
	switch (dma->dma_mode & DMA_MODE_MASK) {
	case DMA_MODE_READ:
		DMA_DAR(channel) = virt_to_bus(dma->buf.address);
		break;
	case DMA_MODE_WRITE:
		DMA_SAR(channel) = virt_to_bus(dma->buf.address);
		break;
	default:
		/*case DMA_MODE_CASCADE: */
		printk(KERN_ERR PKMOD
		       "unsupported dma mode for channel %d. Channel not enabled\n",
		       channel);
		return;
	}

	DMA_CNTR(channel) = dma->buf.length;

	DMA_DISR = 1 << channel;
	DMA_DBTOSR = 1 << channel;
	DMA_DRTOSR = 1 << channel;
	DMA_DSESR = 1 << channel;
	DMA_DBOSR = 1 << channel;
	DMA_CCR(channel) &= ~DMA_CTL_CEN;
	DMA_CCR(channel) |= DMA_CTL_CEN;
}

void
mx2_disable_dma(dmach_t channel, dma_t * dma)
{
	DMA_CCR(channel) &= ~DMA_CTL_CEN;
}

int
mx2_get_dma_residue(dmach_t channel, dma_t * dma)
{
	return (DMA_CNTR(channel) - DMA_CCNR(channel));
}

static struct dma_ops mx2_dops = {
	request:mx2_request_dma,
	free:mx2_free_dma,
	enable:mx2_enable_dma,
	disable:mx2_disable_dma,
	residue:mx2_get_dma_residue,
	setspeed:NULL,
	type:"DBMX2 on-chip DMA",
};

void
arch_dma_init(dma_t * dma_chan)
{
	dma_t *dma;
	int i;

	mx_module_clk_open(HCLK_MODULE_DMA);
	mx_module_clk_open(IPG_MODULE_DMA);
	mx2_dma_chan = dma_chan;	/* save address locally */

	/* initialize dma channel data structures */
	for (i = 0, dma = dma_chan; i < MAX_DMA_CHANNELS; i++, dma++) {
		dma->dma_base = DMA_CH_BASE(i);
		dma->dma_irq = i + 32;	/* Dma channel interrupt number */
		dma->d_ops = &mx2_dops;
	}
	DMA_DCR = 0x2;	/*reset DMA */
	udelay(1);
	mx2_conf_dma_burst_timeout(0, 0);	/*disable dma burst timeout */
}

EXPORT_SYMBOL(mx2_request_dma_irq);
EXPORT_SYMBOL(mx2_free_dma_irq);
EXPORT_SYMBOL(mx2_request_2d_dma);
EXPORT_SYMBOL(mx2_free_2d_dma);
