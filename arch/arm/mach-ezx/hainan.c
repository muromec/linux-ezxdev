/*
 *  linux/arch/arm/mach-ezx/hainan.c
 *
 * Copyright (C) 2004-2005 Motorola 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * 2004-Dec-4 Created for deal with Motorola Hainan  matchine, Wang Jordan
 */

 
#include <linux/init.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/bitops.h>

#include <asm/types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/arch/irq.h>

#include "generic.h"
#include <linux/tty.h>
#include <linux/apm_bios.h>


#define FIRST_STEP              1
#define LAST_STEP               1 
#define BP_RDY_TIMEOUT          0x000c0000 

extern unsigned long sleep_jiffy;
extern int periodic_wake;

extern int ipc_is_active(void);
extern void usb_send_readurb(void);
extern void pm_do_poweroff(void);
extern void ssp_pcap_handshake_init(void);

/* check power down condition */
inline void check_power_off(void)
{
	if ((!GPIO_is_high(GPIO_BP_RDY))) {
                  pm_do_poweroff();
        }
}

static int step = FIRST_STEP;
void handshake(void)
{
	 if (step == 1) {
		 if (GPIO_is_high(GPIO_BP_RDY)) {
			 step ++;

		     {
			     set_GPIO(GPIO_AP_RDY);
			     set_GPIO_mode(GPIO_AP_RDY | GPIO_OUT);
		     }
		}
	}
}
int handshake_pass(void)
{
        return (step > LAST_STEP);
}

#ifdef CONFIG_APM
#define IRQ_EFFECTIVE(x)	(((x) != IRQ_OST0) && ((x) != XSCALE_PMU_IRQ))
#define EFF_IRQ 		~(1 << IRQ_OST0 | 1 << XSCALE_PMU_IRQ)
#define EFF_IRQ2	        0x2

int irq_pending(void)
{
        return ((ICPR & ICMR & EFF_IRQ) || (ICPR2 & ICMR2 & EFF_IRQ2));
}

int pm_handle_irq(int irq)
{

        extern unsigned long sleep_to;
        extern int can_sleep;
        static unsigned long tmp_jiffy;         /* for temporary store of jiffies */

        /*
         * if the timeout is 0 or ICL is active, return.
         */
        if ((sleep_to == 0) || ipc_is_active()) {
                tmp_jiffy = jiffies;
                return irq;
        }

        if (IRQ_EFFECTIVE(irq)) {
                tmp_jiffy = jiffies;
                can_sleep = 0;

        } else if (jiffies > tmp_jiffy + sleep_to) {

                /*
                 * I think this is enough to prevent from reentering here
                 * due to jiffies will be stoped
                 */
                tmp_jiffy = jiffies;
                can_sleep = 1;

		if ((jiffies >= sleep_jiffy) && (!periodic_wake)) {
			sleep_jiffy = jiffies + 20;
                	apm_event_notify(APM_EVENT_PROFILER, EVENT_PROF_SLEEP, 0);
		}
        }

        return irq;
}

void bp_wdi_intr(int irq, void *dev_id, struct pt_regs *regs)
{
	printk(KERN_EMERG "BP paniced, system will reboot now!!!\n");
        apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_BBWDI, 0);
}

static struct irqaction bp_wdi_irq = {
	        name:           "BP wdi",
	        handler:        bp_wdi_intr
};

#endif
extern int in_bprdy_interrupt(void);
extern void set_in_bprdy_interrupt(void);
extern void clear_in_bprdy_interrupt(void);
static void bp_rdy_intr(int irq, void *dev_id, struct pt_regs *regs)
{
        if(in_bprdy_interrupt())
        {
//                printk("in_bprdy_interrupt[%d]\n", in_bprdy_interrupt());
                return;
        }
	
	set_in_bprdy_interrupt();

        if (handshake_pass()) {
		usb_send_readurb();
       	}else{
		handshake();
                if(in_bprdy_interrupt())
                    clear_in_bprdy_interrupt();

	}
}
static struct irqaction bp_rdy_irq = {
        name:           "BP rdy",
        handler:        bp_rdy_intr
};

static void __init ezx_init_gpio_irq(void)
{
#ifdef CONFIG_APM
	set_GPIO_IRQ_edge(GPIO_BB_WDI, GPIO_FALLING_EDGE);
	setup_arm_irq(IRQ_GPIO(GPIO_BB_WDI), &bp_wdi_irq);
#endif
        set_GPIO_IRQ_edge(GPIO_BP_RDY, GPIO_RISING_EDGE);
        setup_arm_irq(IRQ_GPIO(GPIO_BP_RDY), &bp_rdy_irq);

}

static void __init barbados_init_irq(void)
{
	
	pxa_init_irq();

	/* init ezx specfic gpio irq */
	ezx_init_gpio_irq();

	handshake();
}

static void __init
fixup_barbados(struct machine_desc *desc, struct param_struct *params,
		char **cmdline, struct meminfo *mi)
{
	ORIG_VIDEO_COLS = 30;
	ORIG_VIDEO_LINES = 80;
}

static void __init barbados_map_io(void)
{
	pxa_map_io();
#ifdef CONFIG_LOGO_SHOWED_BY_MBM
        /*
        * Don't clear CKEN16_LCD,CKEN0_PWM0CKEN16_LCD  and CKEN0_PWM0
        * setup by MBM for showing logo. Otherwise, LCD can't display normally when control
        * reaches to kernel from MBM.
        */
        CKEN = CKEN & (CKEN9_OSTIMER|CKEN22_MEMC|CKEN16_LCD|CKEN0_PWM0|CKEN20_IM);
#else
        CKEN = CKEN9_OSTIMER | CKEN22_MEMC;
#endif


        /*  set BB_RESET PIN out put high */
        set_GPIO_mode(GPIO_BB_RESET|GPIO_OUT);
        set_GPIO(GPIO_BB_RESET);

        set_GPIO_mode(GPIO42_BTRXD_MD);
        set_GPIO_mode(GPIO43_BTTXD_MD);
        set_GPIO_mode(GPIO44_BTCTS_MD);
        set_GPIO_mode(GPIO45_BTRTS_MD);

	set_GPIO_mode(GPIO46_STRXD_MD);
	set_GPIO_mode(GPIO47_STTXD_MD);

	OSCC = 0x20 | OSCC_OON;		// OSD = ~3ms

        /* setup sleep mode values */
        PWER  = PWER_RTC_TIMER|(1<<GPIO_TFLASH_INT)|(1<<GPIO_BT_HOSTWAKE)|(1<<GPIO_EMU_INT)|
                                (1<<GPIO_BP_RDY)|(1<<GPIO_PCAP_SEC_INT)|(1<<GPIO_BB_WDI)|(1<<GPIO_FLIP_PIN);

        PRER  = (1<<GPIO_BT_HOSTWAKE)|(1<<GPIO_TFLASH_INT)|(1<<GPIO_EMU_INT)|
                 (1<<GPIO_BP_RDY)|(1<<GPIO_PCAP_SEC_INT)|(1<<GPIO_FLIP_PIN);

        PFER  = (1<< GPIO_BT_HOSTWAKE)|(1<<GPIO_TFLASH_INT)|(1<< GPIO_BB_WDI)|(1<<GPIO_FLIP_PIN);
        // keypad wakeup (PKWR,PGSR3) should be in keypad.c

        PGSR0 = (GPIO_bit(GPIO_CAM_RST)) |  (GPIO_bit(GPIO_WDI_AP));

        PGSR1 = (GPIO_bit(GPIO_BT_WAKEUP))|(GPIO_bit(GPIO_SYS_RESTART))|(GPIO_bit(GPIO_CAM_EN))|(GPIO_bit(GPIO_BT_RESET))|(GPIO_bit(GPIO_CLI_RESETB));

        PGSR2 = (GPIO_bit(GPIO_LCD_OFF)) | ( GPIO_bit(GPIO_LCD_CM));

	PGSR3 = (GPIO_bit(GPIO_BB_RESET)) | ( GPIO_bit(GPIO_AP_RDY)) /*e12707 DEC 20 don't care GPIO99 - | (GPIO_bit(GPIO_USB_READY)) */;

        PCFR  = PCFR_DC_EN | PCFR_OPDE;
        PSLR  = 0x05900f00;
}
MACHINE_START(EZX, "Motorola A1200 HAINAN")
	MAINTAINER("Motorola Inc.")
	BOOT_MEM(0xa0000000, 0x40000000, io_p2v(0x40000000))
	BOOT_PARAMS(0xa0000100)
	FIXUP(fixup_barbados)
	MAPIO(barbados_map_io)
	INITIRQ(barbados_init_irq)
MACHINE_END

