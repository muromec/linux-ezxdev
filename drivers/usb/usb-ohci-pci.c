#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>  /* for in_interrupt() */
#undef DEBUG
#include <linux/usb.h>

#include "usb-ohci.h"

#ifdef CONFIG_PMAC_PBOOK
#include <asm/machdep.h>
#include <asm/pmac_feature.h>
#include <asm/pci-bridge.h>
#ifndef CONFIG_PM
#define CONFIG_PM
#endif
#endif


/*-------------------------------------------------------------------------*/

/* Increment the module usage count, start the control thread and
 * return success. */

static struct pci_driver ohci_pci_driver;

static int __devinit
hc_found_ohci (struct pci_dev *dev, int irq,
	void *mem_base, const struct pci_device_id *id)
{
	ohci_t * ohci;
	char buf[8], *bufp = buf;
	int ret;

#ifndef __sparc__
	sprintf(buf, "%d", irq);
#else
	bufp = __irq_itoa(irq);
#endif
	printk(KERN_INFO __FILE__ ": USB OHCI at membase 0x%lx, IRQ %s\n",
		(unsigned long)	mem_base, bufp);
	printk(KERN_INFO __FILE__ ": usb-%s, %s\n", dev->slot_name, dev->name);
    
	ohci = hc_alloc_ohci (dev, mem_base);
	if (!ohci) {
		return -ENOMEM;
	}
	if ((ret = ohci_mem_init (ohci)) < 0) {
		hc_release_ohci (ohci);
		return ret;
	}
	ohci->flags = id->driver_data;

	/* Check for NSC87560. We have to look at the bridge (fn1) to identify
	   the USB (fn2). This quirk might apply to more or even all NSC stuff
	   I don't know.. */
	   
	if(dev->vendor == PCI_VENDOR_ID_NS)
	{
		struct pci_dev *fn1  = pci_find_slot(dev->bus->number, PCI_DEVFN(PCI_SLOT(dev->devfn), 1));
		if(fn1 && fn1->vendor == PCI_VENDOR_ID_NS && fn1->device == PCI_DEVICE_ID_NS_87560_LIO)
			ohci->flags |= OHCI_QUIRK_SUCKYIO;
		
	}
	
	if (ohci->flags & OHCI_QUIRK_SUCKYIO)
		printk (KERN_INFO __FILE__ ": Using NSC SuperIO setup\n");
	if (ohci->flags & OHCI_QUIRK_AMD756)
		printk (KERN_INFO __FILE__ ": AMD756 erratum 4 workaround\n");
	if (hc_reset (ohci) < 0) {
		hc_release_ohci (ohci);
		return -ENODEV;
	}

	/* FIXME this is a second HC reset; why?? */
	writel (ohci->hc_control = OHCI_USB_RESET, &ohci->regs->control);
	wait_ms (10);

	usb_register_bus (ohci->bus);
	
	if (request_irq (irq, hc_interrupt, SA_SHIRQ,
			ohci_pci_driver.name, ohci) != 0) {
		err ("request interrupt %s failed", bufp);
		hc_release_ohci (ohci);
		return -EBUSY;
	}
	ohci->irq = irq;     

	if (hc_start (ohci) < 0) {
		err ("can't start usb-%s", dev->slot_name);
		hc_release_ohci (ohci);
		return -EBUSY;
	}

#ifdef	DEBUG
	ohci_dump (ohci, 1);
#endif
	return 0;
}

/*-------------------------------------------------------------------------*/

/* configured so that an OHCI device is always provided */
/* always called with process context; sleeping is OK */

static int __devinit
ohci_pci_probe (struct pci_dev *dev, const struct pci_device_id *id)
{
	unsigned long mem_resource, mem_len;
	void *mem_base;
	int status;

	if (pci_enable_device(dev) < 0)
		return -ENODEV;

        if (!dev->irq) {
        	err("found OHCI device with no IRQ assigned. check BIOS settings!");
		pci_disable_device (dev);
   	        return -ENODEV;
        }
	
	/* we read its hardware registers as memory */
	mem_resource = pci_resource_start(dev, 0);
	mem_len = pci_resource_len(dev, 0);
	if (!request_mem_region (mem_resource, mem_len, ohci_pci_driver.name)) {
		dbg ("controller already in use");
		pci_disable_device (dev);
		return -EBUSY;
	}

	mem_base = ioremap_nocache (mem_resource, mem_len);
	if (!mem_base) {
		err("Error mapping OHCI memory");
		release_mem_region (mem_resource, mem_len);
		pci_disable_device (dev);
		return -EFAULT;
	}

	/* controller writes into our memory */
	pci_set_master (dev);
	
	status = hc_found_ohci (dev, dev->irq, mem_base, id);
	if (status < 0) {
		iounmap (mem_base);
		release_mem_region (mem_resource, mem_len);
		pci_disable_device (dev);
	}
	return status;
} 

/*-------------------------------------------------------------------------*/

/* may be called from interrupt context [interface spec] */
/* may be called without controller present */
/* may be called with controller, bus, and devices active */

static void __devexit
ohci_pci_remove (struct pci_dev *dev)
{
	ohci_t		*ohci = (ohci_t *) pci_get_drvdata(dev);

	dbg ("remove %s controller usb-%s%s%s",
		hcfs2string (ohci->hc_control & OHCI_CTRL_HCFS),
		dev->slot_name,
		ohci->disabled ? " (disabled)" : "",
		in_interrupt () ? " in interrupt" : ""
		);

	hc_remove_ohci(ohci);

	release_mem_region (pci_resource_start (dev, 0), pci_resource_len (dev, 0));
	pci_disable_device (dev);
}


#ifdef	CONFIG_PM

/*-------------------------------------------------------------------------*/

static int
ohci_pci_suspend (struct pci_dev *dev, u32 state)
{
	ohci_t			*ohci = (ohci_t *) pci_get_drvdata(dev);
	extern int ohci_suspend(ohci_t *, u32);

	return ohci_suspend(ohci, state);
}

/*-------------------------------------------------------------------------*/

static int
ohci_pci_resume (struct pci_dev *dev)
{
	ohci_t		*ohci = (ohci_t *) pci_get_drvdata(dev);
	extern int ohci_resume(ohci_t *);

	return ohci_resume(ohci);
}

#endif	/* CONFIG_PM */


/*-------------------------------------------------------------------------*/

static const struct pci_device_id __devinitdata ohci_pci_ids [] = { {

	/*
	 * AMD-756 [Viper] USB has a serious erratum when used with
	 * lowspeed devices like mice.
	 */
	vendor:		0x1022,
	device:		0x740c,
	subvendor:	PCI_ANY_ID,
	subdevice:	PCI_ANY_ID,

	driver_data:	OHCI_QUIRK_AMD756,

} , {

	/* handle any USB OHCI controller */
	class: 		((PCI_CLASS_SERIAL_USB << 8) | 0x10),
	class_mask: 	~0,

	/* no matter who makes it */
	vendor:		PCI_ANY_ID,
	device:		PCI_ANY_ID,
	subvendor:	PCI_ANY_ID,
	subdevice:	PCI_ANY_ID,

	}, { /* end: all zeroes */ }
};

MODULE_DEVICE_TABLE (pci, ohci_pci_ids);

static struct pci_driver ohci_pci_driver = {
	name:		"usb-ohci",
	id_table:	&ohci_pci_ids [0],

	probe:		ohci_pci_probe,
	remove:		__devexit_p(ohci_pci_remove),

#ifdef	CONFIG_PM
	suspend:	ohci_pci_suspend,
	resume:		ohci_pci_resume,
#endif	/* PM */
};

 
/*-------------------------------------------------------------------------*/

static int __init ohci_hcd_init (void) 
{
	return pci_module_init (&ohci_pci_driver);
}

/*-------------------------------------------------------------------------*/

static void __exit ohci_hcd_cleanup (void) 
{	
	pci_unregister_driver (&ohci_pci_driver);
}

module_init (ohci_hcd_init);
module_exit (ohci_hcd_cleanup);

