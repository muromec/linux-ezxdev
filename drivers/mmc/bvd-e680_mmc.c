/*
 * drivers/mmc/bvd-e680_mmc.c
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
/*
 * Copyright 2004-2005 Motorola, Inc. All Rights Reserved.
 * Revision History:
                    Modification    
 Changed by            Date             Description of Changes
----------------   ------------      -------------------------
Zhu Zhifu           09/24/2004         import to E680 HW
jiang Lili          05/11/2005         Change for sumatra HW
*/
/*
 * E680 MMC/SD driver spec
 *
 * o Card slot 
 *
 *   There are two MMC/SD card slots on board. One is for Tri-Flash, the other
 *   is for SD card. 1 is assigned to SD card slot(external), 0 is assigned to
 *   Tri-Flash slot(internal).
 * 
 * o Card detection
 *
 *   Tri-Flash slot is assumed to be available always. SD card slot detection
 *   is through GPIO11(SD_nCD). 
 *
 * o Card selection 
 *
 *   For chip selection, E680 assigns
 *   - 0 to select SD card slot(external)  
 *   - 1 to select Tri-Flash slot(internal)
 *
 * o Mode switching
 *   
 *   In E680 MMC/SD driver, we need to use MMC/SD mode when SD slot is empty
 *   for higher performance as Motoroal required. We switch to SPI mode when 
 *   SD slot is occupied, and won't switch back to MMC/SD mode.
 *
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
#include <linux/power_ic.h>
#include "../misc/ssp_pcap.h"

/* Generic Bulverde MMC driver externals */
extern void bvd_mmc_slot_up(void);
extern void bvd_mmc_slot_down(void);
extern int  bvd_mmc_slot_init(void);
extern void bvd_mmc_slot_cleanup(void);
extern void bvd_mmc_send_command(struct mmc_request *request);
extern int  bvd_mmc_set_clock(u32 rate);

static struct timer_list e680_detection;

typedef enum {
	MMC_CARD_REMOVED,
	MMC_CARD_INSERTED
}slot_state_t;


extern int mmc_slot_enable;

extern int first_have_card;

slot_state_t slot_state[2];

#define E680_SLOT_TRIFLASH	0
#define E680_SLOT_SD		1

/* Initialize the h/w to up the slot, assuming the card is in slot */
static void e680_mmc_slot_up(int slot) 
{
	int i, flag;

	MOD_INC_USE_COUNT;

	/* TODO : Turn on power for the MMC controller */

	/* Enable Bulverde MMC/SD controller */
	flag = 0;
	//for (i=0; i < 2; i++) {
	for (i=0;i < 1; i++) {
		if (slot_state[i] == MMC_CARD_INSERTED) {
			flag = 1;
			break;
		}
	}

	if (!flag) {
		DEBUG(2, "Init MMC h/w\n");
		bvd_mmc_slot_up();
		DEBUG(2, "MMC h/w initialized\n");
	}

	slot_state[slot] = MMC_CARD_INSERTED;
}

/* Shut down the slot h/w */
static void e680_mmc_slot_down(int slot)
{
	int i, flag;

        slot_state[slot] = MMC_CARD_REMOVED;
 	 
	/* Disable Bulverde MMC/SD controller */
	flag = 0;
	//for (i=0; i < 2; i++) {
	for (i=0; i < 1; i++) {
		if (slot_state[i] == MMC_CARD_INSERTED) {
			flag = 1;
			break;
		}
	}

	if (!flag) {
		DEBUG(2, "down MMC h/w\n");

		/* Disable the Bulverde MMC controller */
		bvd_mmc_slot_down();
	}

	/* TODO : Turn off power for MMC controller ([1], 3.2.2.5) */
	MOD_DEC_USE_COUNT;
}

/* Returns true if MMC slot is empty */
static int e680_mmc_slot_is_empty(int slot)
{
	int empty;
   
    // P1 use a XOR on MMC detect and radio detect, SD insert is high.
    int ret;
   
    ret = GPLR0 & GPIO_bit(GPIO_MMC_DETECT);
    if (ret )
    {
       DEBUG(2, "~~~~~~SD is inserted\n");
       return 0;
    }
    else
    {
       DEBUG(2, "~~~~~~SD is empty\n");

       return 1;
    }
//for P0
//	return GPLR0 & GPIO_bit(GPIO_MMC_DETECT);

#if 0
	if (slot == E680_SLOT_TRIFLASH ) 
		empty= 0;
	else 
		empty= GPLR0 & GPIO_bit(11);

	return empty;
#endif
}

/* Return true if MMC slot is write-protected */
static int e680_mmc_slot_is_wp(int slot)
{
//Sumatra P1 is GPIO21, P0 is GPIO 80
        return (GPLR2 & GPIO_bit(GPIO_MMC_WP));

#if 0
	if (slot == E680_SLOT_TRIFLASH) 
 		return 0;
	else
		return (GPLR3 & GPIO_bit(96));
#endif
}
 
/* Called when MMC card is inserted into / removed from slot */
static void e680_detect_handler(unsigned long data)
{
	int empty;
	DEBUG(2, "card detect handler\n");
        empty = e680_mmc_slot_is_empty(0);
	if (slot_state[0] == MMC_CARD_INSERTED && empty) {
	        DEBUG(3, "no card in slot\n");
		mmc_slot_enable = 0;
	        e680_mmc_slot_down(0);
	        mmc_eject(0);
	} else if (slot_state[0] == MMC_CARD_REMOVED && !empty) {
	        DEBUG(3, "found the card in slot\n");
	        e680_mmc_slot_up(0);
	        mmc_insert(0);
	} 
		
#if 0
	empty = e680_mmc_slot_is_empty(E680_SLOT_SD);
	if (slot_state[E680_SLOT_SD] == MMC_CARD_INSERTED && empty) {
		DEBUG(3, "no card in slot\n");
		e680_mmc_slot_down(E680_SLOT_SD);
		mmc_eject(E680_SLOT_SD);
	} else if (slot_state[E680_SLOT_SD] == MMC_CARD_REMOVED && !empty) {
		DEBUG(3, "found the card in slot\n");
		e680_mmc_slot_up(E680_SLOT_SD);
		mmc_insert(E680_SLOT_SD);
	}
#endif
}

static void e680_detect_int(int irq, void *dev, struct pt_regs *regs)
{
	DEBUG(2, "card detect IRQ\n");

	mod_timer(&e680_detection, jiffies + HZ);
}


/* Initialize the slot and prepare software layer to operate with it */
static int e680_mmc_slot_init(void)
{
	int retval;

	retval = bvd_mmc_slot_init();
	if (retval)
		return retval;
	
	/* GPIO IRQ detection: GPIO 11 (SD_nCD) */
	set_GPIO_IRQ_edge( GPIO_MMC_DETECT, GPIO_FALLING_EDGE | GPIO_RISING_EDGE);

        /* Request card detect interrupt */
	retval = request_irq(IRQ_GPIO(GPIO_MMC_DETECT), e680_detect_int,
			SA_INTERRUPT | SA_SAMPLE_RANDOM,
			"MMC/SD card detect", (void*)1);

	if (retval) {
		printk(KERN_ERR "MMC/SD: can't request MMC card detect IRQ\n");
		bvd_mmc_slot_cleanup();
		return -1;
	}

	/* set wp GPIO(GPIO96 or GPIO107) as input */
//	GPDR3 &= 0x1ffffff & ( ~(0x1ffffff & (1 << (GPIO_MMC_WP - 96)))); 
        set_GPIO_mode(GPIO_MMC_WP | GPIO_IN);


        /* Making slot usable if the card present */
	if (!e680_mmc_slot_is_empty(0)) {
		first_have_card = 1;
	        e680_mmc_slot_up(0);
        }
	
#if 0	
	/* Making Tri-Flash usable always */
	e680_mmc_slot_up(E680_SLOT_TRIFLASH);

	/* Making slot usable if the card present */
	if (!e680_mmc_slot_is_empty(E680_SLOT_SD)) {
		e680_mmc_slot_up(E680_SLOT_SD);
	}
#endif
	DEBUG(2, "MMC initialized\n");
	return 0;
}

/* Shut down the slot and relax software about MMC slot */
static void e680_mmc_slot_cleanup(void)
{
        long flags;

        local_irq_save(flags); 

	/* Shut down the slot */
#if 0
	e680_mmc_slot_down(E680_SLOT_TRIFLASH);
	e680_mmc_slot_down(E680_SLOT_SD);
#endif
	e680_mmc_slot_down(0);
	bvd_mmc_slot_cleanup();

	/* Free card detect IRQ */
	free_irq(IRQ_GPIO(GPIO_MMC_DETECT), (void *)1);
        local_irq_restore(flags);
}

static struct mmc_slot_driver e680_dops = {
	owner:		THIS_MODULE,
	name:		"Motorola E680 MMC/SD",
	ocr:		1 << 19,  /* Mainstone voltage is 3.15V */
	flags:		MMC_SDFLAG_MMC_MODE | MMC_SDFLAG_SD_MODE | MMC_SDFLAG_SPI_MODE,
	init:		e680_mmc_slot_init,
	cleanup:	e680_mmc_slot_cleanup,
	is_empty:	e680_mmc_slot_is_empty,
	is_wp:		e680_mmc_slot_is_wp,
	send_cmd:	bvd_mmc_send_command,
	set_clock:	bvd_mmc_set_clock,
};

int __init e680_mmc_init(void)
{
	int retval;
#ifdef CONFIG_ARCH_EZX_E680
	SSP_PCAP_MMCSD_poweroff();
	mdelay(100);
	SSP_PCAP_MMCSD_poweron();
#else
	/* For Sumatra P0 use VAUX3, for Sumatra P1 and later use VAUX2 */
	
	/* For VAUX2, temp code, waiting for power_ic module new API */
  /*  SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX2_EN);
	mdelay(100);	
	SSP_PCAP_bit_clean(SSP_PCAP_PRI_BIT_AUX_VREG_VAUX2_0);
	SSP_PCAP_bit_set(SSP_PCAP_PRI_BIT_AUX_VREG_VAUX2_1);
    SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX2_EN);*/
        // By power ic module API:
        power_ic_periph_set_flash_card_on(POWER_IC_PERIPH_OFF);
        mdelay(10);
        power_ic_periph_set_flash_card_on(POWER_IC_PERIPH_ON);

#endif


	init_timer(&e680_detection);
	e680_detection.function = e680_detect_handler;

	/* The MMC clock is stopped at this point */
	//retval = mmc_register_slot_driver(&e680_dops, 2);
	retval = mmc_register_slot_driver(&e680_dops, 1);
	if (retval < 0)
		printk(KERN_INFO "MMC/SD: unable to register slot "
		       "driver, error %d\n", retval);

	DEBUG(3, "slot driver registered\n");
	return retval;
}

void __exit e680_mmc_exit(void)
{
	mmc_unregister_slot_driver(&e680_dops);
#ifdef CONFIG_ARCH_EZX_E680
	SSP_PCAP_MMCSD_poweroff();
#else	
// By power ic module API:
	power_ic_periph_set_flash_card_on(POWER_IC_PERIPH_OFF);
	
	/* For VAUX2 */
	//SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX2_EN);
#endif	
	
	DEBUG(3, "slot driver unregistered\n");
}

module_init(e680_mmc_init);
module_exit(e680_mmc_exit);

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION("Intel Mainstone MMC/SD driver");
MODULE_LICENSE("GPL");
