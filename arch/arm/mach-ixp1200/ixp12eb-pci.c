
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/ioport.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach/pci.h>
#include <asm/hardware.h>

extern void ixp1200_pci_init(struct arm_pci_sysdata *);
extern void ixp1200_setup_resources(struct resource **);

static u8 __init ixp12eb_swizzle(struct pci_dev *dev, u8 *pin)
{
	return PCI_SLOT(dev->devfn);
}

static int __init ixp12eb_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	if (dev->vendor == PCI_VENDOR_ID_INTEL &&
		dev->device == PCI_DEVICE_ID_INTEL_IXP1200)
		return 0;

	return IXP1200_IRQ_PCI;
}

struct hw_pci ixp12eb_pci __initdata = {
	init: ixp1200_pci_init,
	setup_resources: ixp1200_setup_resources,
	swizzle: ixp12eb_swizzle,
	map_irq: ixp12eb_map_irq,
};

