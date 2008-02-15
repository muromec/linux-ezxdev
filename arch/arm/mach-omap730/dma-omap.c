/*
 * linux/arch/arm/mach-omap730/dma-omap.c
 *
 * Derived from arch/arm/kernel/dma-sa1100.c
 *
 * Support functions for the OMAP730 internal DMA channels.
 *
 * Copyright (C) 2000 Nicolas Pitre
 * Copyright (C) 2001 RidgeRun, Inc.
 *
 * Modified for the OMAP730 by Dave Peverley <dpeverley at mpc-data.co.uk>
 * Copyright (c) 2004 MPC-Data Limited (http://www.mpc-data.co.uk)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/errno.h>

#ifdef CONFIG_OMAP730_PM
#include <linux/pm.h>
#include <linux/delay.h>
#endif

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/io.h>

/*
 * The TEST code is an attempt to make auto_init work with Power management.
 * It adds some placeholders so I can see state, and it checks to see if 
 * AI is in use before it stops a current buffer.  It looks ugly,
 * but I'm leaving it in for now.
 */
#undef TEST
#undef DEBUG

#ifdef DEBUG
#define DPRINTK( arg... )  printk( ##arg )
#define inline
#else
#define DPRINTK( x... )
#endif

static dma_regs_t *dma_base = (dma_regs_t *) OMAP_DMA_BASE;

typedef struct {
	const char *device_id;	/* device name */
	dma_callback_t callback;	/* to call when DMA completes */
	void *data;		/* ... with private data ptr */
	dma_device_t device;	/* this channel device, 0  if unused */
	dma_direction_t dir;	/* copy to the device or from the device? */
	u16 status;		/* A read clears status, so we need to keep it around */
	u8 in_use;		/* Does someone own the channel?  */
} omap_dma_t;

static omap_dma_t dma_chan[MAX_OMAP_DMA_CHANNELS];
static spinlock_t dma_list_lock = SPIN_LOCK_UNLOCKED;

/* -------------------------------------------------------------------------- */

/* The following DMA helper functions were backported from the 2.6 kernel.
 * The revised OMAP DMA API in the 2.6 kernel is incompatible with the DMA API 
 * in the 2.4. kernel because the prototypes of some existing functions were 
 * changed.  This incomplete backport of parts of the 2.6 API is to make it 
 * easier to port OMAP drivers from the 2.6 kernel to the 2.4 kernel.
 */

void omap_set_dma_transfer_params(int lch, int data_type, int elem_count,
	int frame_count, int frame_sync, dma_device_t sync, int priority,
	int auto_init, int repeat, int end_prog, int omap31_comp_disable,
	int interrupt_enabled)
{
	u16 w;
	int ch = lch;

	w = readw(OMAP_DMA_CSDP_REG(ch));
	w &= ~0x03;
	w |= data_type & 0x03;
	writew(w, OMAP_DMA_CSDP_REG(ch));

	w = readw(OMAP_DMA_CCR_REG(ch));
	w &= ~0x0f7f;
	w |= (sync & 0x1f);
	w |= (frame_sync & 0x1) << 5;
	w |= (priority & 0x1) << 6;
	w |= (auto_init & 0x1) << 8;
	w |= (repeat & 0x1) << 9;
	w |= (omap31_comp_disable & 0x1) << 10;
	w |= (end_prog & 0x1) << 11;
	writew(w, OMAP_DMA_CCR_REG(ch));

	writew((sync == eDMANotSync) ? OMAP_DMA_LCH_CTRL_2D:OMAP_DMA_LCH_CTRL_P, 
		OMAP_DMA_LCH_CTRL_REG(ch));
	
	if (elem_count > 65535)
	{
		printk(KERN_WARNING "Max DMA element count is 65535 !\n");
		elem_count %= 65536;
	}
	
	writew(elem_count, OMAP_DMA_CEN_REG(ch));

	if (frame_count > 65535)
	{
		printk(KERN_WARNING "Max DMA frame count is 65535 !\n");
		frame_count %= 65536;
	}

	writew(frame_count, OMAP_DMA_CFN_REG(ch));

	/* Enable channel interrupts.
	 * This is omap_enable_dma_irq() from the 2.6 kernel.
	 */
	w = readw(OMAP_DMA_CICR_REG(lch));
	w |= interrupt_enabled;
	writew(w, OMAP_DMA_CICR_REG(lch));
}

void omap_set_dma_src_params(int lch, int src_port, int src_amode, 
	int src_start, int src_packing, int src_burst)
{
	u16 w;
 
	w = readw(OMAP_DMA_CCR_REG(lch));
	w &= ~(0x03 << 12);
	w |= (src_amode & 0x03) << 12;
	writew(w, OMAP_DMA_CCR_REG(lch));	

	w = readw(OMAP_DMA_CSDP_REG(lch));
	w &= ~0x01FC;
	w |= (src_port & 0x0F) << 2;
	w |= (src_packing & 0x01) << 6;
	w |= (src_burst & 0x03) << 7;
	writew(w, OMAP_DMA_CSDP_REG(lch));

	writew(src_start & 0xFFFF, OMAP_DMA_CSSA_L_REG(lch));
	writew((src_start >> 16) & 0xFFFF, OMAP_DMA_CSSA_U_REG(lch));
}

void omap_set_dma_dest_params(int lch, int dest_port, int dest_amode, 
	int dest_start, int dest_packing, int dest_burst)
{
	u16 w;

	w = readw(OMAP_DMA_CCR_REG(lch));
	w &= ~(0x03 << 14);
	w |= (dest_amode & 0x03) << 14;
	writew(w, OMAP_DMA_CCR_REG(lch));

	w = readw(OMAP_DMA_CSDP_REG(lch));
	w &= ~0xFE00;
	w |= (dest_port & 0x0F) << 9;
	w |= (dest_packing & 0x01) << 13;
	w |= (dest_burst & 0x03) << 14;
	writew(w, OMAP_DMA_CSDP_REG(lch));

	writew(dest_start & 0xFFFF, OMAP_DMA_CDSA_L_REG(lch));
	writew((dest_start >> 16) & 0xFFFF, OMAP_DMA_CDSA_U_REG(lch));
}

void omap_set_dma_params(int lch, dma_channel_params params)
{
	omap_set_dma_transfer_params(lch, params.data_type, 
		params.elem_count, params.frame_count, 
		params.frame_sync, params.sync, 
		params.priority, params.auto_init, 
		params.repeat, params.end_prog, 
		params.omap31_comp_disable, 
		params.ie);
 
	omap_set_dma_src_params(lch, params.src_port, params.src_amode, 
		params.src_start, params.src_packing, params.src_burst);
	omap_set_dma_dest_params(lch, params.dest_port, params.dest_amode, 
		params.dest_start, params.dest_packing, params.dest_burst);
}

/* -------------------------------------------------------------------------- */

static const int dma_irq[]=
{
	INT_DMA_CH0, INT_DMA_CH1, INT_DMA_CH2, INT_DMA_CH3, INT_DMA_CH4, INT_DMA_CH5, INT_DMA_CH6, INT_DMA_CH7,
	INT_DMA_CH8, INT_DMA_CH9, INT_DMA_CH10,INT_DMA_CH11,INT_DMA_CH12,INT_DMA_CH13,INT_DMA_CH14,INT_DMA_CH15,
};

static void
dma_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	dma_regs_t *dma_regs = dev_id;
	omap_dma_t *dma =
	    dma_chan +
	    (((u_int) dma_regs - OMAP_DMA_BASE) / sizeof (dma_regs_t));
	int status;

	status = dma_regs->csr;	/* A read clears all dma status bits */
	dma->status |= status;

	DPRINTK("%s: status = 0x%04x, dma->status = 0x%04x\n",
		__FUNCTION__, status, dma->status);
	if (dma->status & (DCSR_DROP | DCSR_TOUT)) {
		printk(KERN_DEBUG "DMA %x on \"%s\" caused an error %x\n", irq,
		       dma->device_id, dma->status);
		if (dma->callback)
			dma->callback(dma->data);
		dma->status = 0;
		return;
	}
	if (dma->callback) {
		dma->callback(dma->data);
	}
	if ((dma->status & ~(DCSR_DROP | DCSR_TOUT)) == 0) {
		DPRINTK("DMA ended with no status\n");
	}
#ifdef DEBUG
	if ((dma->status & DCSR_BLOCK) == 0) {
		DPRINTK("DMA status %x\n", dma->status);
	}
#endif
	dma->status = 0;	/* reset status after callback has had a chance 
				 * to process it
				 */
}

/*
 * omap_dma_autoidle_set - Enable or disable the auto idle bit in the DMA global control
 * register.
 *
 * Params:
 *  set = 0 clears the bit disabling the auto idle function.  DMA clocks always on.
 *  set = !0 sets the bit enabling the auto idle function.
 */

void
omap_dma_autoidle_set(int set)
{
	if (set) {
		*((volatile u16 *)OMAP_DMA_GCR) |= GCR_CLK_AUTOGATING_ON;
	} else {
		*((volatile u16 *)OMAP_DMA_GCR) &= ~(GCR_CLK_AUTOGATING_ON);
	}
}

/**
 *	omap_request_dma - allocate one of the OMAP's DMA chanels
 *	@device: The OMAP peripheral targeted by this request
 *	@device_id: An ascii name for the claiming device
 *	@callback: Function to be called when the DMA completes
 *	@data: A cookie passed back to the callback function
 *	@dma_regs: Pointer to the location of the allocated channel's identifier
 *
 * 	This function will search for a free DMA channel and returns the
 * 	address of the hardware registers for that channel as the channel
 * 	identifier. This identifier is written to the location pointed by
 * 	@dma_regs. The list of possible values for @device are listed into
 * 	linux/include/asm-arm/arch-omap730/dma.h as a dma_device_t enum.
 *
 * 	Note that reading from a port and writing to the same port are
 * 	actually considered as two different streams requiring separate
 * 	DMA registrations.
 *
 * 	The @callback function is called from interrupt context when one
 * 	of the two possible DMA buffers in flight has terminated. That
 * 	function has to be small and efficient while postponing more complex
 * 	processing to a lower priority execution context.
 *
 * 	If no channels are available, or if the desired @device is already in
 * 	use by another DMA channel, then an error code is returned.  This
 * 	function must be called before any other DMA calls.
 **/

int
omap_request_dma(dma_device_t device, const char *device_id,
		 dma_callback_t callback, void *data, dma_regs_t ** dma_regs)
{
	omap_dma_t *dma = NULL;
	dma_regs_t *regs;
	int i, err, status;

	*dma_regs = NULL;

	err = 0;
	spin_lock(&dma_list_lock);
		for (i = (device == eDMANotSync)? 0:1 ; i < MAX_OMAP_DMA_CHANNELS; i++) {
		if (dma_chan[i].in_use) {
			if ((dma_chan[i].device == device) &&
			    (device != eDMANotSync)) {
				err = -EBUSY;
				break;
			}
		} else if (!dma) {
			dma = &dma_chan[i];
		}
	}
	if (!err) {
		if (dma) {
			dma->device = device;
		} else {
			err = -ENOSR;
		}
	}
	spin_unlock(&dma_list_lock);
	if (err)
		return err;

	i = dma - dma_chan;
	regs = &dma_base[i];

	err = request_irq(dma_irq[i], dma_irq_handler, SA_INTERRUPT, device_id, regs);

	if (err) {
		printk(KERN_ERR
		       "%s: unable to request IRQ %d for DMA channel\n",
		       device_id, dma_irq[i]);
		return err;
	}

	*dma_regs = regs;
	dma->device_id = device_id;
	dma->callback = callback;
	dma->data = data;
	dma->in_use = 1;

	/*  Reset to default values */
	regs->csdp = 0;
	regs->ccr = device | (1 << 10); /* TODO : Is this correct? 
					 * (See TRM pos. typo)
					 */
	regs->cicr = (DCICR_TOUT_IE | DCICR_DROP_IE);
	status = regs->csr;	/* Read status to clear it. */
	regs->cssa_l = 0;
	regs->cssa_u = 0;
	regs->cdsa_l = 0;
	regs->cdsa_u = 0;
	regs->cen = 0;
	regs->cfn = 0;
	regs->csfi = 0;
	regs->csei = 0;
	regs->cdfi = 0;
	regs->cdei = 0;
	regs->ccr2 = 0;
	regs->clink_ctrl = 0;
	regs->lch_ctrl = (device != eDMANotSync)?LCH_TYPE_P:LCH_TYPE_2D;

	DPRINTK("requested dma device %x\n", device);
	return 0;
}


int
omap_dma_get_status(dma_regs_t * regs)
{
	int i;

	for (i = 0; i < MAX_OMAP_DMA_CHANNELS; i++)
		if (regs == &dma_base[i])
			return dma_chan[i].status;
	printk(KERN_ERR "%s: bad DMA identifier\n", __FUNCTION__);
	return -EINVAL;
}

/**
 * 	omap_free_dma - free a OMAP DMA channel
 * 	@regs: identifier for the channel to free
 *
 * 	This clears all activities on a given DMA channel and releases it
 * 	for future requests.  The @regs identifier is provided by a
 * 	successful call to omap_request_dma().
 **/

void
omap_free_dma(dma_regs_t * regs)
{
	int i;

	for (i = 0; i < MAX_OMAP_DMA_CHANNELS; i++)
		if (regs == &dma_base[i]) {
			if (!dma_chan[i].in_use)
				printk(KERN_ERR "%s: Trying to free free DMA\n", __FUNCTION__);
			else {
				omap_clear_dma(regs);
				free_irq(dma_irq[i], regs);
				dma_chan[i].device = 0;
				dma_chan[i].in_use = 0;
			}
			return;
		}
	printk(KERN_ERR "%s: bad DMA identifier\n", __FUNCTION__);
}

/**
 * 	omap_start_dma - submit a data buffer for DMA
 * 	@regs: identifier for the channel to use
 * 	@dma_ptr: buffer physical (or bus) start address
 * 	@size: buffer size
 *
 * 	This function hands the given data buffer to the hardware for DMA
 * 	access. If another buffer is already in flight then this buffer
 * 	will be queued so the DMA engine will switch to it automatically
 * 	when the previous one is done.  The DMA engine is actually toggling
 * 	between two buffers so at most 2 successful calls can be made before
 * 	one of them terminates and the callback function is called.
 *
 * 	The @regs identifier is provided by a successful call to
 * 	omap_request_dma().
 *
 * 	The @size must not be larger than %MAX_DMA_SIZE.  If a given buffer
 * 	is larger than that then it's the caller's responsibility to split
 * 	it into smaller chunks and submit them separately. If this is the
 * 	case then a @size of %CUT_DMA_SIZE is recommended to avoid ending
 * 	up with too small chunks. The callback function can be used to chain
 * 	submissions of buffer chunks.
 *
 *      If the @size parameter is 0, then this assumes the user has set
 *      up the appropriate registers that calculate DMA size, and this
 *      doesn't try to reset them.  
 *
 *      If you want to DMA more than 64K, this will try to calculate registers
 *      for you.  If you send in an odd size, this may take some time for the 
 *      calculations, which are done with interrupts off.  So try to limit
 *      the DMA size or send in a regs->pfn value that evenly divides the
 *      size you send in.
 *
 * 	Error return values:
 * 	%-EOVERFLOW:	Given buffer size is too big.
 * 	%-EBUSY:	Both DMA buffers are already in use.
 * 	%-EAGAIN:	Both buffers were busy but one of them just completed
 * 			but the interrupt handler has to execute first.
 *      %-EINVAL:       Can't make a valid dma request with given size parameter
 *
 * 	This function returs 0 on success.
 **/

int
omap_start_dma(dma_regs_t * regs, dma_addr_t dma_ptr, u_int size)
{
	unsigned long flags;
	int ret = 0;
	int channel = (((u_int) regs - OMAP_DMA_BASE) / sizeof (dma_regs_t));
	omap_dma_t *dma = &dma_chan[channel];

	if (size > MAX_DMA_SIZE)
		return -EOVERFLOW;

	local_irq_save(flags);

	DPRINTK("%s: chan = %d, dma_ptr =  0x%08x, size = %d\n",
		__FUNCTION__, channel, dma_ptr, size);
	DPRINTK("%s: csdp = 0x%04x, ccr = 0x%04x, cicr = 0x%04x, cpc = 0x%04x\n",
		__FUNCTION__, regs->csdp, regs->ccr, regs->cicr, regs->cpc);

	/*
	   NOTE:

	   Auto initialization conflicts with power management.  To keep track
	   of two buffers and be able to stop and restart them correctly
	   would require a pretty significant rewrite.  But if I turn
	   off AI, sound playback is affected.

	   So rather than affecting sound playback, note that if we go
	   into power saving while an audio buffer is playing, we will
	   probably not recover correctly.

	   #ifdef CONFIG_OMAP730_PM
	   if (auto_init) {
	   	local_irq_restore(flags);
	   	return -EBUSY;
	   }
	   #endif        
	 */

	// Setup simple transfer.
	if (dma->dir == eDmaOut) {
		regs->cssa_l = dma_ptr;
		regs->cssa_u = dma_ptr >> 16;
	} else {
		regs->cdsa_l = dma_ptr;
		regs->cdsa_u = dma_ptr >> 16;
	}

	// size is in bytes.  Check csdp to see how many elements to transfer:
	if ((regs->csdp & DCSDP_DATA_TYPE_MASK) == DATA_TYPE_S16) {	// 16-bit transfers
		size >>= 1;
	} else if ((regs->csdp & DCSDP_DATA_TYPE_MASK) == DATA_TYPE_S32) {	// 32-bit transfers
		size >>= 2;
	}

	/*
	   If size = 0, that means the user has set up the c*n and c*i
	   registers, so don't mess with anything.

	   Otherwise, now that we know how many bytes are being sent, 
	   let's see how they're organized.  Default case
	   is a simple contiguous buffer with CEN=size
	   and CFN=1.
	   But have to check other cases, too.
	 */
	if (size) {
		if ((regs->cfn == 0) || (regs->cfn == 1)) {
			// Probably default values set by this routine.  Try using 1
			//  for cfn, cen is 16-bit max.
			if (size < 0x10000) {
				regs->cen = size;
				regs->cfn = 1;
			} else {
				int divider = 2;
				while (!(((size / divider) < 0x10000) &&
					 ((size % divider) == 0))) {
					divider++;
					if (divider > 0xffff) {
						// Can't get a valid pair.
						local_irq_restore(flags);
						return -EINVAL;
					}
				}
				regs->cen = (size / divider);
				regs->cfn = divider;
			}
		} else {
			// Assume cfn was set by user, use set value if possible
			if ((size % regs->cfn) == 0) {
				regs->cen = (size / regs->cfn);
			} else {	// Numbers don't match, try to find matching ones
				int divider = 1;
				while (!(((size / divider) < 0x10000) &&
					 ((size % divider) == 0))) {
					divider++;
					if (divider > 0xffff) {
						// Can't get a valid pair.
						local_irq_restore(flags);
						return -EINVAL;
					}
				}
				regs->cen = (size / divider);
				regs->cfn = divider;
			}
		}
	}
	
	/* DMA channel is not running, so we don't set the REPEAT bit. */
	regs->ccr = (regs->ccr | DCCR_EN) & ~(DCCR_REPEAT | DCCR_AUTO_INIT);

	DPRINTK("started transfer %x, ccr = 0x%04x, cfn = 0x%04x, cen=0x%04x\n", 
		dma_ptr, regs->ccr, regs->cfn, regs->cen);
	wmb();		// make sure previous writes are done before we enable.

	local_irq_restore(flags);
	return ret;
}

/*
 *     omap_clear_dma - clear DMA pointers
 *     @regs: identifier for the channel to use
 *
 *     This clears any DMA state so the DMA engine is ready to restart
 *     with new buffers through omap_start_dma(). Any buffers in flight
 *     are discarded.
 *
 *     The @regs identifier is provided by a successful call to
 *     omap_request_dma().
 */

void
omap_clear_dma(dma_regs_t * regs)
{
	unsigned long flags;
	int status;

	local_irq_save(flags);

	regs->ccr &= ~(DCCR_REPEAT | DCCR_AUTO_INIT | DCCR_END_PROG | DCCR_EN);
	status = regs->csr;     /* read csr to clear pending interrupts */

	local_irq_restore(flags);
}

/**
 * 	omap_get_dma_pos - return current DMA position
 * 	@regs: identifier for the channel to use
 *
 * 	This function returns the current physical (or bus) address for the
 * 	given DMA channel.  If the channel is running i.e. not in a stopped
 * 	state then the caller must disable interrupts prior calling this
 * 	function and process the returned value before re-enabling them to
 * 	prevent races with the completion interrupt handler and the callback
 * 	function. The validation of the returned value is the caller's
 * 	responsibility as well 
 *
 * 	The @regs identifier is provided by a successful call to
 * 	omap_request_dma().
 **/

dma_addr_t
omap_get_dma_pos(dma_regs_t * regs)
{

	return (dma_addr_t) (regs->cssa_l | (regs->cssa_u << 16));

}

/**
 * 	omap_reset_dma - reset a DMA channel
 * 	@regs: identifier for the channel to use
 *
 * 	This function resets and reconfigure the given DMA channel. This is
 * 	particularly useful after a sleep/wakeup event.
 *
 * 	The @regs identifier is provided by a successful call to
 * 	omap_request_dma().
 **/

void
omap_reset_dma(dma_regs_t * regs)
{
	int i;

	for (i = 0; i < MAX_OMAP_DMA_CHANNELS; i++)
		if (regs == &dma_base[i])
			break;
	if (i >= MAX_OMAP_DMA_CHANNELS) {
		printk(KERN_ERR "%s: bad DMA identifier\n", __FUNCTION__);
		return;
	}

	omap_clear_dma(regs);

}

#ifdef CONFIG_OMAP730_PM
/* Drivers should call this from their PM callback function */

/* Bitfields so we can keep track of which
   channels have been modified by PM. */
static int channel_auto;

#ifdef TEST
static int channel_external_auto;
static int sleep_regs[100];
static int wake_regs[100];
static int sleep_request;
static int wake_request;
#endif

/* Returns 1 if DMA is busy, 0 otherwise */
static int
dma_channel_moving_data(dmach_t channel)
{
	dma_regs_t *regs = &dma_base[channel];

	/* Check to see if enabled. */
	if (regs->ccr & DCCR_EN)
		return 1;
	return 0;
}

/* Returns 1 if DMA is busy, 0 otherwise.
   Called from PM code to check if we can idle. */
int
omap_dma_busy(void)
{
	dmach_t channel;
	for (channel = 0; channel < MAX_OMAP_DMA_CHANNELS; channel++)
		if (dma_chan[channel].in_use &&
		    dma_channel_moving_data(channel))
			return 1;
	return 0;
}

#ifdef TEST
/* Returns 1 if turned off AI, 0 otherwise */
static int
dma_disable_auto_init(dmach_t channel)
{
	dma_regs_t *regs = &dma_base[channel];

	if (regs->ccr & DCCR_AUTO_INIT) {
		/* Turn off auto init if enabled. */
		regs->ccr &= ~(DCCR_AUTO_INIT | DCCR_END_PROG | DCCR_REPEAT);
		return 1;
	}
	return 0;
}
#else
static inline int
dma_disable_auto_init(dmach_t channel)
{
	return 0;
}
#endif

/*
  Local Power management.
*/

static int
dma_pm_callback(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	dmach_t channel;
	int channel_enabled = 0;
	int try_count = 0;

	DPRINTK("%s: %d\n", __FUNCTION__, rqst);

	if (rqst == PM_SUSPEND) {
		for (channel = 0; channel < MAX_OMAP_DMA_CHANNELS; channel++) {
			if (dma_chan[channel].in_use) {
				if (dma_channel_moving_data(channel)) {
					channel_auto |=
					    (dma_disable_auto_init(channel) <<
					     channel);
					channel_enabled |= (1 << channel);
				}
			}
		}

		while (channel_enabled && (try_count++ < 50)) {
			udelay(100);
			DPRINTK(KERN_INFO "%s: channel_enabled %x\n",
				__FUNCTION__, channel_enabled);
			for (channel = 0; channel < MAX_OMAP_DMA_CHANNELS;
			     channel++) {
				if (channel_enabled & (1 << channel)) {
					if (!dma_channel_moving_data(channel)) {
						channel_enabled &=
						    ~(1 << channel);
					}
				}
			}
		}
		if (channel_enabled) {
			printk(KERN_INFO
			       "Can't suspend, DMA channel(s) busy, bitmap: 0x%x\n",
			       channel_enabled);
			return 1;	/* tell suspend we can't stop now. */
		}
	} else if (rqst == PM_RESUME) {
		for (channel = 0; channel < MAX_OMAP_DMA_CHANNELS; channel++) {
			if ((dma_chan[channel].in_use) &&
			    (channel_auto & (1 << channel))) {
				dma_regs_t *regs = &dma_base[channel];
				regs->ccr |= DCCR_EN;
				channel_auto &= ~(1 << channel);
				DPRINTK(KERN_INFO "re-enable channel %d\n",
					channel);
			}
		}
	}
	return 0;
}
#endif /* CONFIG_OMAP730_PM */

int
omap_dma_setup(dma_device_t device, dma_direction_t direction)
{
	int i;

	for (i = 0; i < MAX_OMAP_DMA_CHANNELS; i++)
		if (device == dma_chan[i].device)
			break;
	if (i >= MAX_OMAP_DMA_CHANNELS || !dma_chan[i].in_use) {
		printk(KERN_ERR "%s: bad DMA identifier\n", __FUNCTION__);
		return -EINVAL;
	}

	dma_chan[i].dir = direction;

	return 0;
}


static int __init
omap_init_dma(void)
{
	int channel;
	int status;
	dma_regs_t *regs;

	/* Enable OMAP23.2 GDMA */
	outw(GSCR_OMAP_3_1_MAPPING_DISABLE, OMAP_DMA_GSCR);

	for (channel = 0; channel < MAX_OMAP_DMA_CHANNELS; channel++) {
		regs = &dma_base[channel];
		status = regs->csr;
	}

#ifdef CONFIG_OMAP730_PM
	pm_register(PM_SYS_DEV, PM_SYS_ARCH_SPECIFIC, dma_pm_callback);
#endif
	return 0;
}

__initcall(omap_init_dma);

EXPORT_SYMBOL(omap_request_dma);
EXPORT_SYMBOL(omap_dma_get_status);
EXPORT_SYMBOL(omap_free_dma);
EXPORT_SYMBOL(omap_dma_setup);
EXPORT_SYMBOL(omap_clear_dma);
EXPORT_SYMBOL(omap_start_dma);
EXPORT_SYMBOL(omap_get_dma_pos);
EXPORT_SYMBOL(omap_reset_dma);
#ifdef CONFIG_OMAP730_PM
EXPORT_SYMBOL(omap_dma_busy);
#endif
EXPORT_SYMBOL(omap_dma_autoidle_set);
EXPORT_SYMBOL(omap_set_dma_transfer_params);
EXPORT_SYMBOL(omap_set_dma_src_params);
EXPORT_SYMBOL(omap_set_dma_dest_params);
EXPORT_SYMBOL(omap_set_dma_params);
