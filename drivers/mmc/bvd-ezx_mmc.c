/*
 * drivers/mmc/bvd-ezx_mmc.c
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
 * Copyright 2003-2005 Motorola, Inc. All Rights Reserved.
 * Revision History:
                    Modification    
 Changed by            Date             Description of Changes
----------------   ------------      -------------------------
Zhou Qiong          12/22/2003         modify for Motorola EZX platform
jiang Lili          04/12/2005         modify for mmc card detect and 4 bit mode
jiang Lili          05/11/2005         remove warning
Liu Anyu            11/15/2005         fix card detect GPIO issue
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
#include <asm/arch/ezx.h>
#include <asm/arch/hardware.h>

#include "../misc/ssp_pcap.h"
#include <linux/power_ic.h>

#define VAP_MMC		0x00000B00
#define VAP_MMC_MASK	0xFFFFF0FF

/* Generic Bulverde MMC driver externals */
extern void bvd_mmc_slot_up(void);
extern void bvd_mmc_slot_down(void);
extern int  bvd_mmc_slot_init(void);
extern void bvd_mmc_slot_cleanup(void);
extern void bvd_mmc_send_command(struct mmc_request *request);
extern int  bvd_mmc_set_clock(u32 rate);

static struct timer_list ezx_detection;

extern int first_have_card;
extern int mmc_slot_enable;

enum {
	MMC_CARD_REMOVED,
	MMC_CARD_INSERTED
} slot_state = MMC_CARD_REMOVED;

u8 ezx_mmc_get_slot_state(void)
{
    if (slot_state == MMC_CARD_REMOVED)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}   

EXPORT_SYMBOL(ezx_mmc_get_slot_state);
/* Initialize the h/w to up the slot, assuming the card is in slot */
static void ezx_mmc_slot_up(void) 
{
	MOD_INC_USE_COUNT;
	DEBUG(2, "Init MMC h/w\n");
	
	bvd_mmc_slot_up();

	slot_state = MMC_CARD_INSERTED;
	DEBUG(2, "MMC h/w initialized\n");
}

/* Shut down the slot h/w */
static void ezx_mmc_slot_down(void)
{
	DEBUG(2, "down MMC h/w\n");
	
	/* Disable the Bulverde MMC controller */
	bvd_mmc_slot_down();
	
	slot_state = MMC_CARD_REMOVED;
	MOD_DEC_USE_COUNT;
}

/* Returns true if MMC slot is empty */
static int ezx_mmc_slot_is_empty(int slot)
{
	unsigned long	gpioval;
	
#ifdef CONFIG_ARCH_EZXBASE
    gpioval = GPLR(GPIO_MMC_DETECT) & GPIO_bit(GPIO_MMC_DETECT);
    if (gpioval)
    {
    	DEBUG(2, "check if triflash exist %lx is not empty.\n", gpioval);
    	return 0;
    }
    else
    {
    	DEBUG(2, "check if triflash exist %lx is empty.\n", gpioval);
    	return 1;
    }
#else	
	/* Check DAT3 for triflash card detection */
	gpioval = GPLR(GPIO_MMC_DATA3) & GPIO_bit(GPIO_MMC_DATA3);
	DEBUG(2, "check if triflash exist %lx\n", gpioval);
	
	/* if triflash DAT3 is high, there is triflash card    */
	if( GPLR(GPIO_MMC_DATA3) & GPIO_bit(GPIO_MMC_DATA3) )
              	return 0;
	else
		return 1;
#endif		
}

/* Called when MMC card is inserted into / removed from slot */
void ezx_detect_handler(unsigned long data)
{
        int empty;
        DEBUG(2, "card detect handler\n");

        empty = ezx_mmc_slot_is_empty(0);
        if (slot_state == MMC_CARD_INSERTED && empty) {
                DEBUG(2, "no card in slot\n");
                mmc_slot_enable = 0;
                ezx_mmc_slot_down();
                mmc_eject(0);
         } else if (slot_state == MMC_CARD_REMOVED && !empty) {
                DEBUG(2, "found the card in slot\n");
                ezx_mmc_slot_up();
                mmc_insert(0);
         }
}

static void ezx_detect_int(int irq, void *dev, struct pt_regs *regs)
{
        DEBUG(0, ":card detect IRQ\n");
        mod_timer(&ezx_detection, jiffies + HZ);
}

/* Initialize the slot and prepare software layer to operate with it */
static int ezx_mmc_slot_init(void)
{
	int retval;

	retval = bvd_mmc_slot_init();
	if (retval)
		return retval;
#ifdef CONFIG_ARCH_EZXBASE
 set_GPIO_mode(GPIO_MMC_DETECT | GPIO_IN);

#ifndef CONFIG_ARCH_EZX_HAINAN	
	/* GPIO IRQ detection: GPIO 11 (SD_nCD) */
	set_GPIO_IRQ_edge( GPIO_MMC_DETECT, GPIO_FALLING_EDGE | GPIO_RISING_EDGE);

        /* Request card detect interrupt */
	retval = request_irq(IRQ_GPIO(GPIO_MMC_DETECT), ezx_detect_int,
			             SA_INTERRUPT | SA_SAMPLE_RANDOM, "MMC/SD card detect", (void*)1);
         if(retval)
         {
                printk(KERN_ERR "MMC/SD: can't request MMC card detect IRQ\n");
                 bvd_mmc_slot_cleanup();
                 return -1;
         }

#endif
#else
	
	set_GPIO_mode(GPIO_MMC_DATA3 | GPIO_IN);
        set_GPIO_IRQ_edge(GPIO_MMC_DATA3, GPIO_FALLING_EDGE | GPIO_RISING_EDGE);

	if (GPLR(GPIO_MMC_DATA3) & 0x8000)
        {
            DEBUG(0,":GPIO 111 defaut is high\n");
        }else
            DEBUG(0,":GPIO 111 default is low\n");
         
	retval = request_irq(IRQ_GPIO(GPIO_MMC_DATA3), ezx_detect_int,
                         SA_INTERRUPT | SA_SAMPLE_RANDOM, "MMC card detect", (void *)1);
	 if(retval)
	 {		
	 	printk(KERN_ERR "MMC/SD: can't request MMC card detect IRQ\n");
	         bvd_mmc_slot_cleanup();
        	 return -1;
         }
#endif	
	/* Making slot usable if the card present */
#ifndef CONFIG_ARCH_EZX_HAINAN
	if (!ezx_mmc_slot_is_empty(0)) 
	{
		first_have_card = 1;
		ezx_mmc_slot_up();
	}
#else
       if (!ezx_mmc_slot_is_empty(0)) 
       {//has card
           first_have_card = 1;
           ezx_mmc_slot_up();
           mmc_insert(0);
       }
       else //no card
       {
           mmc_slot_enable = 0;
           ezx_mmc_slot_down();
       }
#endif

	DEBUG(2, "MMC initialized\n");
	return 0;
}

/* Shut down the slot and relax software about MMC slot */
static void ezx_mmc_slot_cleanup(void)
{
        long flags;

        local_irq_save(flags); 

	/* Shut down the slot */
	ezx_mmc_slot_down();
	bvd_mmc_slot_cleanup();
	
#ifdef CONFIG_ARCH_EZXBASE
    free_irq(IRQ_GPIO(GPIO_MMC_DETECT),(void *)1);
#else
	free_irq(IRQ_GPIO(GPIO_MMC_DATA3),(void *)1);
#endif	
        local_irq_restore(flags);
}

static struct mmc_slot_driver ezx_dops = {
	owner:		THIS_MODULE,
	name:		"HCDDBBVA0 (Ezx) MMC/SD",
	ocr:		1 << 15,  /* triflash voltage is 2.775V */
	flags:		MMC_SDFLAG_MMC_MODE | MMC_SDFLAG_SD_MODE,
	init:		ezx_mmc_slot_init,
	cleanup:	ezx_mmc_slot_cleanup,
	is_empty:	ezx_mmc_slot_is_empty,
	send_cmd:	bvd_mmc_send_command,
	set_clock:	bvd_mmc_set_clock,
};

int __init ezx_mmc_init(void)
{
	int retval;
	
#ifdef CONFIG_ARCH_EZX_A780
	unsigned long ssp_pcap_register_val;
	
	/* close VAP_MMC (PCAP_VAUX3) */
	SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX3_EN);
	SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX3_EN);

	mdelay(10);

	/* open VAP_MMC (PCAP VAUX3) */
	SSP_PCAP_read_data_from_PCAP(SSP_PCAP_ADJ_AUX_VREG_REGISTER, &ssp_pcap_register_val);	
	DEBUG(2, "\nSSP_PCAP_ADJ_AUX_VREG_REGISTER=0x%x\n", ssp_pcap_register_val);
	
	ssp_pcap_register_val = (ssp_pcap_register_val & VAP_MMC_MASK) | VAP_MMC;
	SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_AUX_VREG_REGISTER, ssp_pcap_register_val);
	SSP_PCAP_write_data_to_PCAP(SSP_PCAP_ADJ_AUX_VREG_REGISTER, ssp_pcap_register_val);
	SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX3_EN);
	SSP_PCAP_bit_set(SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX3_EN);
	
	SSP_PCAP_read_data_from_PCAP(SSP_PCAP_ADJ_AUX_VREG_REGISTER, &ssp_pcap_register_val);	
	DEBUG(2, "SSP_PCAP_ADJ_AUX_VREG_REGISTER=0x%x\n", ssp_pcap_register_val);
#else
//	power_ic_periph_set_flash_card_on(POWER_IC_PERIPH_OFF);
//	mdelay(10);			//PCAP restart done in arch_reset() system.h
	power_ic_periph_set_flash_card_on(POWER_IC_PERIPH_ON);
	udelay(1000);
#endif
	
	 init_timer(&ezx_detection);
         ezx_detection.function = ezx_detect_handler;
		
	PGSR(GPIO_MMC_CLK)  |=  GPIO_bit(GPIO_MMC_CLK);
        PGSR(GPIO_MMC_CMD)  |=  GPIO_bit(GPIO_MMC_CMD);
        PGSR(GPIO_MMC_DATA0)  |=  GPIO_bit(GPIO_MMC_DATA0);
        PGSR(GPIO_MMC_DATA1)  |=  GPIO_bit(GPIO_MMC_DATA1);
        PGSR(GPIO_MMC_DATA2)  |=  GPIO_bit(GPIO_MMC_DATA2);
        PGSR(GPIO_MMC_DATA3)  |=  GPIO_bit(GPIO_MMC_DATA3);	

	/* The MMC clock is stopped at this point */
	retval = mmc_register_slot_driver(&ezx_dops, 1);
	if (retval < 0)
		printk(KERN_INFO "MMC/SD: unable to register slot "
		       "driver, error %d\n", retval);

	DEBUG(3, "slot driver registered\n");
	return retval;
}

void __exit ezx_mmc_exit(void)
{
	mmc_unregister_slot_driver(&ezx_dops);
	
	/* close VAP_MMC (PCAP_VAUX3) */
#ifdef CONFIG_ARCH_EZX_A780 	
	SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX3_EN);
	SSP_PCAP_bit_clean(SSP_PCAP_ADJ_BIT_AUX_VREG_VAUX3_EN);
#else	
	power_ic_periph_set_flash_card_on(POWER_IC_PERIPH_OFF);
#endif

	DEBUG(3, "slot driver unregistered\n");
}

module_init(ezx_mmc_init);
module_exit(ezx_mmc_exit);

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION("Motorola A780 triflash/MMC/SD driver");
MODULE_LICENSE("GPL");
