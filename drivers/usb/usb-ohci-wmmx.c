/*
 * linux/drivers/usb/usb-ohci-wmmx.c
 * BRIEF MODULE DESCRIPTION
 *	Intel Bulverde on-chip OHCI controller (non-pci) configuration.
 * 
 * Copyright 2005 Motorola
 * Copyright 2004 Motorola
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
 * History: 
 *  2004-May-01            Support usb host driver in EZX platform         [q16136@motorola.com]  
 *  2005-Jul-01            Redesign usb host resume and support single ended mode [w20535@motorola.com]
 */

/*
 *  linux/drivers/usb/usb-ohci-wmmx.c
 *
 *  The outline of this code was taken from Brad Parkers <brad@heeltoe.com>
 *  original OHCI driver modifications, and reworked into a cleaner form
 *  by Russell King <rmk@arm.linux.org.uk>.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/pci.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/pci.h>
#include <linux/pm.h>
#include <linux/device.h>
#include "usb-ohci.h"

#undef DEBUG
//#define DEBUG
#undef DUMP_BUL_OHCI

#undef TRANSCEIVER_MODE
#define UP2OCR		__REG(0x40600020)
#define UP3OCR		__REG(0x40600024)

#define PMM_NPS_MODE           1
#define PMM_GLOBAL_MODE        2
#define PMM_PERPORT_MODE       3
#define PMM_MIXED_MODE         4

int __devinit
hc_add_ohci(struct pci_dev *dev, int irq, void *membase, unsigned long flags,
	    ohci_t **ohci, const char *name, const char *slot_name);
extern void hc_remove_ohci(ohci_t *ohci);

static ohci_t *wmmx_ohci;
/* bogus pci_dev structure */
static struct pci_dev bogus_pcidev;


int usb_host_suspended = 0;

#if 0
static struct pm_dev *bvd_pm_dev;
#endif

#ifdef CONFIG_DPM
#include <linux/device.h>

static int usb_suspend(struct device * dev, u32 state, u32 level);
static int usb_resume(struct device * dev, u32 level);

static struct device_driver usb_driver_ldm = {
	name:      	"usb-ohci-wmmx",
	devclass:  	NULL,
	probe:     	NULL,
	suspend:   	usb_suspend,
	resume:    	usb_resume,
	scale:	  	NULL,
	remove:    	NULL,
};

static struct device usb_device_ldm = {
	name:		"USB OHCI Host Controller",
	bus_id:		"usb-ohci",
	driver:		NULL,
	power_state:	DPM_POWER_ON,
};

static void usb_ldm_register(void)
{
	extern void pxasys_driver_register(struct device_driver *driver);
	extern void pxasys_device_register(struct device *device);

	pxasys_driver_register(&usb_driver_ldm);
	pxasys_device_register(&usb_device_ldm);
}

static void usb_ldm_unregister(void)
{
	extern void pxasys_driver_unregister(struct device_driver *driver);
	extern void pxasys_device_unregister(struct device *device);

	pxasys_driver_unregister(&usb_driver_ldm);
	pxasys_device_unregister(&usb_device_ldm);
}

/* forward declarations for resume */
static int wmmx_ohci_select_pmm(int mode, int ports);
static void wmmx_ohci_configure(void);
static void board_usb_configure(void);

static int usb_resume(struct device * dev, u32 level)
{
	extern int ohci_resume(ohci_t *);
	int ret = 0;
	void *ohci_addr;

	switch (level) {
	case RESUME_POWER_ON:
		/* enable the clock */

		/* complete re-init */
		wmmx_ohci_configure();
		
		ohci_addr = ioremap(UHC_BASE_PHYS, 32*1024);
		
		/*
		 * Fill in some fields of the bogus pci_dev.
		 */
		memset(&bogus_pcidev, 0, sizeof(struct pci_dev));
		strcpy(bogus_pcidev.name, "BULVERDE OHCI");
		strcpy(bogus_pcidev.slot_name, "builtin");
		bogus_pcidev.resource[0].name = "OHCI Operational Registers";
		
		/*
		 * Initialise the generic OHCI driver.
		 */
		ret = hc_add_ohci(&bogus_pcidev, IRQ_USBH1, ohci_addr, 0,
				  &wmmx_ohci, "usb-ohci", "wmmx");
		
		/* Select Power Management Mode */
		wmmx_ohci_select_pmm(PMM_GLOBAL_MODE, 0);

		break;
	}

	return ret;
}

static int usb_suspend(struct device * dev, u32 state, u32 level)
{
	extern int ohci_suspend(ohci_t *, u32);
	int ret = 0;
	
	switch (level) {
	case SUSPEND_POWER_DOWN:
		/* complete exit */
		/* hc_remove_ohci() will release OHCI register VM mapping */
		hc_remove_ohci(wmmx_ohci);
		if (1) {
		/* per the yellow book, 20.7.4.4 */
		/* 1 */
		CKEN |= CKEN10_USBHOST;
		UHCHR |= UHCHR_FHR;
		udelay(11);
		UHCHR &= ~UHCHR_FHR;

		/* 2 */
		UHCCOMS |= 1;
		udelay(11);

		/* 3 */
		UHCRHS |= 1;

		/* 4 */
		UHCHR &= ~UHCHR_SSE;
		}
		/*
		 * Stop the USB clock.
		 */
		CKEN &= ~CKEN10_USBHOST;
		break;
	}

	return ret;
}

#endif /* CONFIG_DPM */

static void board_usb_configure(void)
{
#ifdef CONFIG_ARCH_EZX
#ifdef TRANSCEIVER_MODE
	/* setup Port3's GPIO pin. */
	set_GPIO_mode(GPIO_ALT_FN_3_OUT | GPIO30_USB_P3_2); /* GPIO30 - USB_P3_2/ICL_TXENB */
	set_GPIO_mode(GPIO_ALT_FN_3_OUT | GPIO31_USB_P3_6); /* GPIO31 - USB_P3_6/ICL_VPOUT */
	set_GPIO_mode(GPIO_ALT_FN_2_IN | GPIO90_USB_P3_5); /* GPIO90 - USB_P3_5/ICL_VPIN */
	set_GPIO_mode(GPIO_ALT_FN_2_IN | GPIO91_USB_P3_1); /* GPIO91 - USB_P3_1/ICL_XRXD */
	set_GPIO_mode(GPIO_ALT_FN_1_OUT | GPIO56_USB_P3_4); /* GPIO56 - USB_P3_4/ICL_VMOUT */
	set_GPIO_mode(GPIO_ALT_FN_3_IN | GPIO113_USB_P3_3);/* GPIO113 - USB_P3_3/ICL_VMIN */
	UP3OCR = 0x00000000;
	UHCHR = UHCHR & ~(UHCHR_SSEP2 | UHCHR_SSEP3 | UHCHR_SSE);
#else
	set_GPIO_mode(GPIO_ALT_FN_3_OUT | GPIO30_USB_P3_2); /* GPIO30 - USB_P3_2/ICL_TXENB */
	set_GPIO_mode(GPIO_ALT_FN_3_OUT | GPIO31_USB_P3_6); /* GPIO31 - USB_P3_6/ICL_VPOUT */
	set_GPIO_mode(GPIO_ALT_FN_2_IN | GPIO90_USB_P3_5); /* GPIO90 - USB_P3_5/ICL_VPIN */
	set_GPIO_mode(GPIO_ALT_FN_2_IN | GPIO91_USB_P3_1); /* GPIO91 - USB_P3_1/ICL_XRXD */
	set_GPIO_mode(GPIO_ALT_FN_1_OUT | GPIO56_USB_P3_4); /* GPIO56 - USB_P3_4/ICL_VMOUT */
	set_GPIO_mode( GPIO_OUT| GPIO113_USB_P3_3);
	GPCR(GPIO113_USB_P3_3) = GPIO_bit(GPIO113_USB_P3_3);
	UP3OCR = 0x00000003;

	UHCHR = UHCHR & ~(UHCHR_SSEP2 | UHCHR_SSEP3 | UHCHR_SSE);
#endif
#endif
}


static void wmmx_ohci_configure(void)
{
	/* start bulverde's USB host clock. */
	CKEN |= CKEN10_USBHOST;

	UHCHR |= UHCHR_FHR;
	udelay(11);
	UHCHR &= ~UHCHR_FHR;

	/* reset USB Bus Interface */
	UHCHR |= UHCHR_FSBIR;
	while (UHCHR & UHCHR_FSBIR);

	board_usb_configure();

#if defined(DEBUG)
	printk("USB host VERSION %08x\n", UHCREV);
	printk("USB Reset %08x\n", UHCHR);
#endif
}

#define USB_OHCI_OP_PHYS_BASE ((unsigned int)&UHCREV)

static int wmmx_ohci_select_pmm(int mode, int ports)
{
	switch ( mode ) {
	case PMM_NPS_MODE: /* this mode is only for development */
		UHCRHDA |= RH_A_NPS;
		break; 
	case PMM_GLOBAL_MODE:
		UHCRHDA &= ~(RH_A_NPS | RH_A_PSM);
		break;
	case PMM_PERPORT_MODE:
		UHCRHDA &= ~RH_A_NPS | RH_A_PSM;
		/* Should I use one port*/
		{
			int i;
			for (i = 0; i < ports; i++)
				UHCRHDB |= (unsigned int)(1u << (i + 17));
		}
		break;
	case PMM_MIXED_MODE:
		printk(KERN_INFO"Currently, do not support this mode %d\n", mode);
		break;
	default:
		printk(KERN_INFO"Invalid mode %d\n", mode );
		return -1;
	}

	return 0;
}

#ifdef CONFIG_PM
static u32 usbh_reg[50];
static u32 usbhreg = 0x4c000000;
extern int usb_host_resumed;
static int wmmx_ohci_suspend( ohci_t *ohci )
{
#ifdef CONFIG_NO_USB_SUSPEND_RESUME
	printk("%s: NO_USB_SUSPEND_RESUME!\n", __FUNCTION__);
	return 1;
#else 
	int i;

	if(usb_host_suspended)
		return  0;

	usbhreg = 0x4c000000;
	for(i=0; i<28; i++)
	{
		//printk("usbhreg=0x%x\n", usbhreg);
		usbh_reg[i] = __REG(usbhreg);
		//printk("usbh_reg[%d]=0x%x\n", i, usbh_reg[i]);
		usbhreg +=0x4;
	}
	usbh_reg[40] = UP2OCR;
	usbh_reg[41] = UP3OCR;
#ifdef TRANSCEIVER_MODE 
	if(GPIO_is_high(GPIO30_USB_P3_2))
	{
		PGSR0 |= GPIO_bit(GPIO30_USB_P3_2);
	}
	if(GPIO_is_high(GPIO31_USB_P3_6))
	{
		PGSR0 |= GPIO_bit(GPIO31_USB_P3_6);
	}
	if(GPIO_is_high(GPIO56_USB_P3_4))
	{
		PGSR1 |= GPIO_bit(GPIO56_USB_P3_4);
	}else 
	{
		PGSR1 &= ~(GPIO_bit(GPIO56_USB_P3_4));
	}
#else
	if(GPIO_is_high(GPIO90_USB_P3_5))
	{
		PGSR2 |= GPIO_bit(GPIO90_USB_P3_5);
	}
	if(GPIO_is_high(GPIO91_USB_P3_1))
	{
		PGSR2 |= GPIO_bit(GPIO91_USB_P3_1);
	}
	if(GPIO_is_high(GPIO113_USB_P3_3))
	{
		PGSR3 |= GPIO_bit(GPIO113_USB_P3_3);
	}else
	{
		PGSR3 &= ~(GPIO_bit(GPIO113_USB_P3_3));
	}
#endif	
	//printk("Frame # =0x%x\n", __REG(0x4c00003c));
	CKEN &= ~CKEN10_USBHOST;
	CKEN &= ~CKEN11_USB;
	usb_host_resumed = 0;
	usb_host_suspended = 1;
	return 0;
#endif
}

void wmmx_ohci_suspend_call(void)
{ 
	wmmx_ohci_suspend( wmmx_ohci );
}
	
void clear_BLE(void)
{
	wmmx_ohci->hc_control &= ~(OHCI_CTRL_BLE);
}
void set_BLE(void)
{
	wmmx_ohci->hc_control |= OHCI_CTRL_BLE;
}
int usbh_finished_resume = 1;
int wmmx_host_gpio_resume(void)
{
#ifdef TRANSCEIVER_MODE
	set_GPIO_mode(GPIO_ALT_FN_2_IN | 91); /* GPIO91 - USB_P3_1/ICL_XRXD */
	
	__REG(0x40e0000c) |= 0x40000000; //GPDR0
	__REG(0x40e00000) |= 0x40000000; //GPLR
	__REG(0x40e00058) |= 0x30000000; //GAFRU_U
	set_GPIO_mode(GPIO_ALT_FN_3_IN | 113);/* GPIO113 - USB_P3_3/ICL_VMIN */
	set_GPIO_mode(GPIO_ALT_FN_1_OUT | 56); /* GPIO56 - USB_P3_4/ICL_VMOUT */
	set_GPIO_mode(GPIO_ALT_FN_2_IN | 90); /* GPIO90 - USB_P3_5/ICL_VPIN */
	set_GPIO_mode(GPIO_ALT_FN_3_OUT | 31); /* GPIO31 - USB_P3_6/ICL_VPOUT */
#else
        set_GPIO_mode(GPIO_ALT_FN_3_OUT | GPIO30_USB_P3_2); /*GPIO30 - USB_P3_2/ICL_TXENB */
//        set_GPIO_mode(GPIO_ALT_FN_3_IN | GPIO113_USB_P3_3);/* GPIO113 - USB_P3_3/ICL_VMIN */
	set_GPIO_mode( GPIO_OUT| GPIO113_USB_P3_3);
	GPCR(GPIO113_USB_P3_3) = GPIO_bit(GPIO113_USB_P3_3);

        set_GPIO_mode(GPIO_ALT_FN_1_OUT | GPIO56_USB_P3_4); /* GPIO56 - USB_P3_4/ICL_VMOUT */
        set_GPIO_mode(GPIO_ALT_FN_2_IN | GPIO90_USB_P3_5); /* GPIO90 - USB_P3_5/ICL_VPIN */
        set_GPIO_mode(GPIO_ALT_FN_3_OUT | GPIO31_USB_P3_6); /* GPIO31 - USB_P3_6/ICL_VPOUT */
        set_GPIO_mode(GPIO_ALT_FN_2_IN | GPIO91_USB_P3_1);  /*GPIO91 - USB_P3_1/ICL_XRXD*/
#endif
}
extern int host_port_suspended;
static int wmmx_ohci_resume(ohci_t *ohci)
{
#ifdef CONFIG_NO_USB_SUSPEND_RESUME
	printk("%s: NO_USB_SUSPEND_RESUME!\n", __FUNCTION__);
	return 1;
#else 
	unsigned int uPortNumber;

	usbh_finished_resume = 0;
#if defined(DEBUG)
	unsigned int j;
	printk("PGSR0=0x%x\n", PGSR0);	
	printk("PGSR1=0x%x\n", PGSR1);	
	printk("PGSR2=0x%x\n", PGSR2);	
	printk("PGSR3=0x%x\n", PGSR3);	

	/* Print out inital Values. */
	printk("OHCI REGISTERS AFTER SUSPEND \n");	
	usbhreg = 0x4c000000;
	for(j=0; j<28; j++)	
	{
		printk("usbhreg[0x%x]=0x%x\n", usbhreg, __REG(usbhreg));
		usbhreg +=0x4;
	}
      printk("\ngpcr0[%x]gpcr1[%x]gpcr2[%x]gpcr3[%x]\n", GPCR0, GPCR1, GPCR2, GPCR3);
      printk("\ngpsr0[%x]gpcr1[%x]gpsr2[%x]gpsr3[%x]\n", GPSR0, GPSR1, GPSR2, GPSR3);
      printk("\ngpdr0[%x]gpdr1[%x]gpdr2[%x]gpdr3[%x]\n", GPDR0, GPDR1, GPDR2, GPDR3);
#endif	

	
	UHCHR = 2;
	udelay(11);
	UHCHR = 6;
	udelay(11);
	UHCHR = 4;
	udelay(11);

	/* start bulverde's USB host clock. */
	CKEN |= CKEN10_USBHOST;

	/* Restore the HCCA Register */
	__REG(0x4c000018) = usbh_reg[6];
	
	/* Restore the (HUCCHED) UHC Control Head Endpoint Descriptor Register*/
	__REG(0x4c000020) = usbh_reg[8];

	/* Restore the (HUCCHED) UHC Bulk Head Endpoint Descriptor Register*/
	__REG(0x4c000028) = usbh_reg[10];
	
	/* Restore the UHC Frame Interval(UHCFMI) Register - Mask off reserved bits, set FIT */
	__REG(0x4c000034) = (usbh_reg[13] & ~(0xc000));

	/* Restore the UHC Periodic Start(UHCPERS) Register */
	__REG(0x4c000040) = usbh_reg[16] & 0x3fff;
	
	/* Restore the UHC Low-Speed Threshold(UHCLST) Register */
	__REG(0x4c000044) = usbh_reg[17] & 0xfff;
	
	/* Restore the Root Hub Descriptor A(UHCRHDA) Register */
	__REG(0x4c000048) = usbh_reg[18];
	
	/* Restore the Root Hub Descriptor B(UHCRHDA) Register */
	__REG(0x4c00004c) = usbh_reg[19];
	
	/* Now ebable the list processing */
	/* Restore the (UCHCON) UHC Host Control Register */
	__REG(0x4c000004) = usbh_reg[1];
	
	__REG(0x4c000010) = usbh_reg[4];

	UP2OCR = usbh_reg[40];
	UP3OCR = usbh_reg[41];


	wmmx_host_gpio_resume();

	/* Now turn on Port Power for all ports that had it before */
	/* Restore the (UCHCON) UHC Host Control Register */
	//printk("ENABLING POWER!\n");
	for(uPortNumber=0; uPortNumber<3; uPortNumber++)
	{
		//printk("Port %d\n", uPortNumber);
		if((usbh_reg[21+uPortNumber] & 0x100) == 0x100)
		{
			__REG(0x4c000054+(uPortNumber*4)) = 0x100;
		} 
	}
	printk("POWER ENABLED!\n");
	mdelay(2);

	/* Enable & Suspend the port */
	__REG(0x4c00005c) = 0x6;
	mdelay(1);

	/* Resume Port */
	__REG(0x4c00005c) = 0x8;

	while((__REG(0x4c00005c) & (1<<18)) == 0)
	{
		if( !(UHCRHPS3 & 0x2))
			break;
		mdelay(1);
	}
	
	/* Clear the status change bits */
	__REG(0x4c000054) = (1<<16);
	__REG(0x4c00005c) = (1<<18) | (1<<16);
	for(uPortNumber=0; uPortNumber<3; uPortNumber++)
	{
		printk("Port %d After=0x%x\n", uPortNumber, __REG(0x4c000054+(uPortNumber*4)));
	}

#if defined(DEBUG)
	printk("OHCI REGISTERS After Restore \n");
	usbhreg = 0x4c000000;
	for(j=0; j<28; j++)
	{
		printk("usbhreg[0x%x]=0x%x\n", usbhreg, __REG(usbhreg));
		usbhreg +=0x4;
	}
#endif
	printk("\n-----------------\n END OF RESUME SEQUENCE\n");

	if( !(UHCRHPS3 & 0x2))
	{
		printk("\nwmmx_ohci_resume:port disbled.\n");
		usbh_finished_resume = 1;	
		return -1;
	}

        UHCRHPS3 = 0x4;
        printk("\nsuspend-port[0x%x]\n", UHCRHPS3);
        while(!(UHCRHPS3 & 0x4)) mdelay(1);
	host_port_suspended = 1;

	usbh_finished_resume = 1;	
	usb_host_suspended = 0;
	return 0;
#endif
}

void wmmx_ohci_resume_call(void)
{ 
	wmmx_ohci_resume( wmmx_ohci );
}

#if 0
static int pm_suspend = 0;
static int bvd_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	int ret = 0;
 	ohci_t *ohci = (ohci_t*)pm_dev->data;
	printk(": bvd pm callback %d RTSR=%08x ICPR=%08x\n", req, RTSR, ICPR);
	switch (req) {
	case PM_SUSPEND:
		if(pm_suspend)
			break;
		if(ret = wmmx_ohci_suspend(ohci))
			printk(KERN_ERR"wmmx_ohci_suspend() error!\n");	
		else
			pm_suspend = 1;		
		break;
	case PM_RESUME:
		if(!pm_suspend)
			break;
		if(ret = wmmx_ohci_resume(ohci))
			printk(KERN_ERR"wmmx_ohci_resume() error!\n");	
		else
			pm_suspend = 0;	
		break;
	default:
		printk(KERN_ERR"Invalid pm request %d!\n", req);
		break;
        }
        return ret;
}
#endif		
#endif 

static int __init wmmx_ohci_init(void)
{
	int ret;
	void *ohci_addr;

	wmmx_ohci_configure();

	ohci_addr = ioremap(UHC_BASE_PHYS, 32*1024);

	printk("ochi_addr = 0x%p\n", ohci_addr);	
	/*
	 * Fill in some fields of the bogus pci_dev.
	 */
	memset(&bogus_pcidev, 0, sizeof(struct pci_dev));
	strcpy(bogus_pcidev.name, "BULVERDE OHCI");
	strcpy(bogus_pcidev.slot_name, "builtin");
	bogus_pcidev.resource[0].name = "OHCI Operational Registers";

	/*
	 * Initialise the generic OHCI driver.
	 */
	ret = hc_add_ohci(&bogus_pcidev, IRQ_USBH1, ohci_addr, 0, &wmmx_ohci,
			  "usb-ohci", "wmmx");

	/* Select Power Management Mode */
//	wmmx_ohci_select_pmm(PMM_GLOBAL_MODE, 0); //each port is powered individually
	
	wmmx_ohci_select_pmm(PMM_NPS_MODE, 0); //NoPowerSwitch.ports're always powered when the UHC is powered on. 

#if 0
#ifdef CONFIG_PM
	bvd_pm_dev = pm_register(PM_SYS_DEV, PM_SYS_UNKNOWN, bvd_pm_callback);
	if(bvd_pm_dev)
	{
		bvd_pm_dev->data = (void*)wmmx_ohci;
	}	
#endif 
#endif 

#ifdef CONFIG_DPM
	usb_ldm_register();
#endif

	usb_host_suspended = 0;
	return ret;
}

static void __exit wmmx_ohci_exit(void)
{
#ifdef CONFIG_DPM
	usb_ldm_unregister();
#endif

#if 0	
#ifdef CONFIG_PM
	pm_unregister(bvd_pm_dev);	
#endif
#endif 
	/* hc_remove_ohci() will release OHCI register VM mapping */
	hc_remove_ohci(wmmx_ohci);

	/*
	 * Stop the USB clock.
	 */
	CKEN &= ~CKEN10_USBHOST;
}

module_init(wmmx_ohci_init);
module_exit(wmmx_ohci_exit);

