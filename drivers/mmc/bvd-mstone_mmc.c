/*
 * drivers/mmc/bvd-mstone_mmc.c
 * Mainstone (bulverde-based) board-level MMC driver
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
 * [1] - Intel(R) HCDDBVA0 Development Platform for Intel(R) PCA User's Guide
 * [2] - Intel(R) Bulverde Processor (B-Stepping) Developer's Manual
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/mmc/mmc_ll.h>

#include <asm/irq.h>        
#include <asm/unaligned.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mainstone.h>

/* Generic Bulverde MMC driver externals */
extern void bvd_mmc_slot_up(void);
extern void bvd_mmc_slot_down(void);
extern int  bvd_mmc_slot_init(void);
extern void bvd_mmc_slot_cleanup(void);
extern void bvd_mmc_send_command(struct mmc_request *request);
extern int  bvd_mmc_set_clock(u32 rate);

static struct timer_list mstone_detection;

enum {
	MMC_CARD_REMOVED,
	MMC_CARD_INSERTED
} slot_state = MMC_CARD_REMOVED;

/* Initialize the h/w to up the slot, assuming the card is in slot */
static void mstone_mmc_slot_up(void) 
{
	MOD_INC_USE_COUNT;
	DEBUG(2, "Init MMC h/w\n");

	/* Turn on power for the MMC controller ([1], 3.2.2.5) */
	MST_MSCWR1 |= MST_MSCWR1_MMC_ON;
	DEBUG(3, " ...MMC power OK\n");

	/* Make sure SD/Memory Stick multiplexer's signals
	   are routed to MMC controller ([1], 3.2.2.5) */
	MST_MSCWR1 &= ~MST_MSCWR1_MS_SEL;
	DEBUG(3, " ...multiplexer OK\n");

	bvd_mmc_slot_up();

	slot_state = MMC_CARD_INSERTED;
	DEBUG(2, "MMC h/w initialized\n");
}

/* Shut down the slot h/w */
static void mstone_mmc_slot_down(void)
{
	DEBUG(2, "down MMC h/w\n");

	/* Disable the Bulverde MMC controller */
	bvd_mmc_slot_down();

	/* Turn off power for MMC controller ([1], 3.2.2.5) */
	MST_MSCWR1 &= ~MST_MSCWR1_MMC_ON;
	slot_state = MMC_CARD_REMOVED;

	MOD_DEC_USE_COUNT;
}

/* Returns true if MMC slot is empty */
static int mstone_mmc_slot_is_empty(int slot)
{
	/* Check MSCRD for MMC card detection ([1], 3.2.2.7) */
	return (MST_MSCRD & MST_MSCRD_nMMC_CD);
}

/* Called when MMC card is inserted into / removed from slot */
static void mstone_detect_handler(unsigned long data)
{
	int empty;
	DEBUG(2, "card detect handler\n");
	empty = mstone_mmc_slot_is_empty(0);
	if (slot_state == MMC_CARD_INSERTED && empty) {
		DEBUG(3, "no card in slot\n");
		mstone_mmc_slot_down();
		mmc_eject(0);
	} else if (slot_state == MMC_CARD_REMOVED && !empty) {
		DEBUG(3, "found the card in slot\n");
		mstone_mmc_slot_up();
		mmc_insert(0);
	}
}

static void mstone_detect_int(int irq, void *dev, struct pt_regs *regs)
{
	DEBUG(2, "card detect IRQ\n");
	mod_timer(&mstone_detection, jiffies + HZ);
}


/* Initialize the slot and prepare software layer to operate with it */
static int mstone_mmc_slot_init(void)
{
	int retval;

	retval = bvd_mmc_slot_init();
	if (retval)
		return retval;
	
        /* Request card detect interrupt */
	retval = request_irq(MAINSTONE_MMC_IRQ, mstone_detect_int, 
			     SA_INTERRUPT | SA_SAMPLE_RANDOM,
			     "MMC card detect", (void *)1);
	if (retval) {
		printk(KERN_ERR "MMC/SD: can't request MMC card detect IRQ\n");
		bvd_mmc_slot_cleanup();
		return -1;
	}

	/* Making slot usable if the card present */
	if (!mstone_mmc_slot_is_empty(0)) {
		mstone_mmc_slot_up();
	}

	DEBUG(2, "MMC initialized\n");
	return 0;
}

/* Shut down the slot and relax software about MMC slot */
static void mstone_mmc_slot_cleanup(void)
{
        long flags;

        local_irq_save(flags); 

	/* Shut down the slot */
	mstone_mmc_slot_down();
	bvd_mmc_slot_cleanup();

	/* Free card detect IRQ */
	free_irq(MAINSTONE_MMC_IRQ, (void *)1);

        local_irq_restore(flags);
}

#ifdef CONFIG_DPM
#include <linux/device.h>

static int mstone_mmc_suspend(struct device *dev, u32 state, u32 level);
static int mstone_mmc_resume(struct device *dev, u32 level);

static struct device_driver mstone_mmc_driver_ldm = {
	name:          "mst-mmc",
	devclass:      NULL,
	probe:         NULL,
	suspend:       mstone_mmc_suspend,
	resume:        mstone_mmc_resume,
	scale:         NULL,
	remove:        NULL,
};

static struct device mstone_mmc_device_ldm = {
	name:         "MSTONE MMC",
	bus_id:       "mstone-mmc",
	driver:       NULL,
	power_state:  DPM_POWER_ON,
};

static void
mstone_mmc_ldm_register(void)
{
#ifdef CONFIG_ARCH_MAINSTONE
	extern void pxaopb_driver_register(struct device_driver *driver);
	extern void pxaopb_device_register(struct device *device);
	
	pxaopb_driver_register(&mstone_mmc_driver_ldm);
	pxaopb_device_register(&mstone_mmc_device_ldm);
#endif
}

static void
mstone_mmc_ldm_unregister(void)
{
#ifdef CONFIG_ARCH_MAINSTONE
	extern void pxaopb_driver_unregister(struct device_driver *driver);
	extern void pxaopb_device_unregister(struct device *device);
	
	pxaopb_device_unregister(&mstone_mmc_device_ldm);
	pxaopb_driver_unregister(&mstone_mmc_driver_ldm);
#endif
}

static int
mstone_mmc_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		enable_irq(MAINSTONE_MMC_IRQ);
		/* Making slot usable if the card present */
		if (!mstone_mmc_slot_is_empty(0)) {
			mstone_mmc_slot_up();
		}
		break;
	}
	return 0;
}

static int
mstone_mmc_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		mstone_mmc_slot_down();
		disable_irq(MAINSTONE_MMC_IRQ);
		break;
	}
	return 0;
}

#endif /* CONFIG_DPM */

static struct mmc_slot_driver mstone_dops = {
	owner:		THIS_MODULE,
	name:		"HCDDBBVA0 (Mainstone) MMC/SD",
	ocr:		1 << 19,  /* Mainstone voltage is 3.15V */
	flags:		MMC_SDFLAG_MMC_MODE | MMC_SDFLAG_SD_MODE,
	init:		mstone_mmc_slot_init,
	cleanup:	mstone_mmc_slot_cleanup,
	is_empty:	mstone_mmc_slot_is_empty,
	send_cmd:	bvd_mmc_send_command,
	set_clock:	bvd_mmc_set_clock,
};

int __init mstone_mmc_init(void)
{
	int retval;

	init_timer(&mstone_detection);
	mstone_detection.function = mstone_detect_handler;
	
#ifdef CONFIG_DPM
	mstone_mmc_ldm_register();
#endif /* CONFIG_DPM */ 

	/* The MMC clock is stopped at this point */
	retval = mmc_register_slot_driver(&mstone_dops, 1);
	if (retval < 0)
		printk(KERN_INFO "MMC/SD: unable to register slot "
		       "driver, error %d\n", retval);

	DEBUG(3, "slot driver registered\n");
	return retval;
}

void __exit mstone_mmc_exit(void)
{
	mmc_unregister_slot_driver(&mstone_dops);
	
#ifdef CONFIG_DPM
	mstone_mmc_ldm_unregister();
#endif /* CONFIG_DPM */

	DEBUG(3, "slot driver unregistered\n");
}

module_init(mstone_mmc_init);
module_exit(mstone_mmc_exit);

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION("Intel Mainstone MMC/SD driver");
MODULE_LICENSE("GPL");
