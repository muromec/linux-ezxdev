/******************************************************************************

	linux/drivers/usb/usb-ohci-omap1510.c
	USB OHCI Support for OMAP1510/1610/730

	Author: MontaVista Software, Inc. <source@mvista.com>
	Copyright (c) 2003 MontaVista Software, Inc.

	The outline of this code was taken from Brad Parkers <brad@heeltoe.com>
	original OHCI driver modifications, and reworked into a cleaner form
	by Russell King <rmk@arm.linux.org.uk>.

	Support for the OMAP730 included by Simon Dunn <sdunn at mpc-data.co.uk>
	Copyright (c) 2004 MPC-Data Limited (http://www.mpc-data.co.uk)
	April 2004.

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

*******************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pci.h>		/* for pci_pool_* prototypes */
#include <linux/usb.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "usb-ohci.h"

#if !defined(CONFIG_ARCH_OMAP730)
#define OMAP1510_LB_OFFSET (0x30000000UL)

#ifdef CONFIG_ARCH_OMAP1610
#define OMAP1610_SET_FUNC_MUX_CTRL(mode,reg,bit) outl((inl(reg)&~(0x7<<bit))|(mode<<bit),reg)
#define OMAP1610_CONFIRM_MUX_SETUP() outl(0xeaef,COMP_MODE_CTRL_0)
#define open_drain(bit,reg) outl(inl(reg)|(1<<bit),reg)
#endif
#endif

extern int __devinit hc_add_ohci(struct pci_dev *dev, int irq, void *membase,
				 unsigned long flags, ohci_t ** ohci,
				 const char *name, const char *slot_name);
extern void hc_remove_ohci(ohci_t * ohci);

static ohci_t *omap1510_ohci;

/* bogus pci_dev structure */
static struct pci_dev bogus_pcidev;

static int __devinit omap1510_ohci_configure(void);
static void omap1510_ohci_release(void);

#if defined(CONFIG_OMAP_H2) || defined(CONFIG_MACH_OMAP_PERSEUS2)
#include <linux/i2c.h>

#if defined(CONFIG_ARCH_OMAP730)
#define OTG_BASE	OMAP730_USB_OTG_BASE
#define OTG_IOSIZE	OMAP730_USB_OTG_SIZE
#endif

#if defined(CONFIG_MACH_OMAP_PERSEUS2)
#define ISP1301_I2C_ADDR 0x2C
#else
#define ISP1301_I2C_ADDR 0x2D
#endif

#define ISP1301_I2C_MODE_CONTROL_1 0x4
#define ISP1301_I2C_MODE_CONTROL_2 0x12
#define ISP1301_I2C_OTG_CONTROL_1 0x6
#define ISP1301_I2C_OTG_CONTROL_2 0x10
#define ISP1301_I2C_INTERRUPT_SOURCE 0x8
#define ISP1301_I2C_INTERRUPT_LATCH 0xA
#define ISP1301_I2C_INTERRUPT_FALLING 0xC
#define ISP1301_I2C_INTERRUPT_RISING 0xE
#define ISP1301_I2C_REG_CLEAR_ADDR 1

struct i2c_driver isp1301_driver;
struct i2c_client *isp1301_i2c_client;

static unsigned short normal_i2c[] = { ISP1301_I2C_ADDR, I2C_CLIENT_END };
static unsigned short dummy_i2c_addrlist[] = { I2C_CLIENT_END };
static struct i2c_client_address_data addr_data = {
	.normal_i2c = normal_i2c,
	.normal_i2c_range = dummy_i2c_addrlist,
	.probe = dummy_i2c_addrlist,
	.probe_range = dummy_i2c_addrlist,
	.ignore = dummy_i2c_addrlist,
	.ignore_range = dummy_i2c_addrlist,
	.force = dummy_i2c_addrlist
};

static int initstate_region;
static int initstate_i2c;

static int isp1301_probe(struct i2c_adapter *adap);
static int isp1301_detach(struct i2c_client *client);
static int isp1301_command(struct i2c_client *client, unsigned int cmd,
			   void *arg);

struct i2c_driver isp1301_driver = {
	.name = "OMAP+USB",
	.id = I2C_DRIVERID_EXP0,	/* Fake Id */
	.flags = I2C_DF_NOTIFY,
	.attach_adapter = isp1301_probe,
	.detach_client = isp1301_detach,
	.command = isp1301_command
};

static int
isp1301_attach(struct i2c_adapter *adap, int addr, unsigned short flags,
	       int kind)
{
	struct i2c_client *c;

	c = (struct i2c_client *) kmalloc(sizeof (*c), GFP_KERNEL);

	if (!c)
		return -ENOMEM;

	strcpy(c->name, "OMAP+USB");
	c->id = isp1301_driver.id;
	c->flags = 0;
	c->addr = addr;
	c->adapter = adap;
	c->driver = &isp1301_driver;
	c->data = NULL;

	isp1301_i2c_client = c;

	return i2c_attach_client(c);
}

static int
isp1301_probe(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, isp1301_attach);
}

static int
isp1301_detach(struct i2c_client *client)
{
	i2c_detach_client(client);
	kfree(isp1301_i2c_client);
	return 0;
}

/* No commands defined */
static int
isp1301_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	return 0;
}

static void
i2c_write(u8 buf, u8 subaddr)
{
	char tmpbuf[2];

	tmpbuf[0] = subaddr;	/*register number */
	tmpbuf[1] = buf;	/*register data */
	i2c_master_send(isp1301_i2c_client, &tmpbuf[0], 2);
}

static void
isp1301_configure(void)
{
	i2c_write(6, ISP1301_I2C_MODE_CONTROL_1);
	i2c_write(~6, ISP1301_I2C_MODE_CONTROL_1 | ISP1301_I2C_REG_CLEAR_ADDR);
	i2c_write(4, ISP1301_I2C_MODE_CONTROL_2);
	i2c_write(~4, ISP1301_I2C_MODE_CONTROL_2 | ISP1301_I2C_REG_CLEAR_ADDR);
	i2c_write(0x2C, ISP1301_I2C_OTG_CONTROL_1);
	i2c_write(~0x2C, ISP1301_I2C_OTG_CONTROL_1 | ISP1301_I2C_REG_CLEAR_ADDR);
	i2c_write(0xFF,
		  ISP1301_I2C_INTERRUPT_LATCH | ISP1301_I2C_REG_CLEAR_ADDR);
	i2c_write(0xFF,
		  ISP1301_I2C_INTERRUPT_FALLING | ISP1301_I2C_REG_CLEAR_ADDR);
	i2c_write(0xFF,
		  ISP1301_I2C_INTERRUPT_RISING | ISP1301_I2C_REG_CLEAR_ADDR);
}

static void
isp1301_vbus_off(void)
{
	i2c_write(0xC, ISP1301_I2C_OTG_CONTROL_1);
	i2c_write(~0xC, ISP1301_I2C_OTG_CONTROL_1 | ISP1301_I2C_REG_CLEAR_ADDR);
}
#endif

#undef CONFIG_CEE
#define CONFIG_CEE

#if	defined(CONFIG_CEE)	/* MVL-CEE */
#include <linux/device.h>

static int omap1510_ohci_suspend(struct device *dev, u32 state, u32 level);
static int omap1510_ohci_resume(struct device *dev, u32 level);
static int omap1510_ohci_scale(struct bus_op_point *op, u32 level);

static int omap1510_ohci_pid;
static DECLARE_COMPLETION(kusbdpmd);
static int omap1510_usbpmd_req;
static wait_queue_head_t kusbdpmd_wait;

static struct device_driver omap1510_ohci_driver_ldm = {
	.name = "omap-ohci",
	.devclass = NULL,
	.probe = NULL,
	.suspend = omap1510_ohci_suspend,
	.resume = omap1510_ohci_resume,
	.scale = omap1510_ohci_scale,
	.remove = NULL,
};

static struct device omap1510_ohci_device_ldm = {
	.name = "OMAP1510/1610/730 OHCI",
	.bus_id = "OHCI",
	.driver = NULL,
	.power_state = DPM_POWER_ON,
};

static int
omap1510_ohci_thread(void *arg)
{
	int ret;
	struct task_struct *tsk = current;

	/*
	 * We don't want /any/ signals, not even SIGKILL
	 */
	sigfillset(&tsk->blocked);
	sigemptyset(&tsk->pending.signal);
	recalc_sigpending(tsk);
	daemonize();
	reparent_to_init();
	strcpy(tsk->comm, "kusbdpmd");
	tsk->tty = NULL;

	while (1) {
		int req;

		do {
			req = xchg(&omap1510_usbpmd_req, 0);
			if (req == 0) {
				sigemptyset(&tsk->pending.signal);
				interruptible_sleep_on(&kusbdpmd_wait);
			}
		} while (req == 0);

		if (req == SIGTERM) {
			break;
		} else if (req == SIGHUP) {
			ret = omap1510_ohci_configure();
			complete(&kusbdpmd);
		} else {
			printk(KERN_DEBUG "%s: Unknown Request: %d\n",
			       __FUNCTION__, req);
		}
	}

	complete_and_exit(&kusbdpmd, 0);
}

static void
omap1510_ohci_ldm_driver_register(void)
{
	extern void mpu_public_driver_register(struct device_driver *driver);
	mpu_public_driver_register(&omap1510_ohci_driver_ldm);
}

static void
omap1510_ohci_ldm_device_register(void)
{
	extern void mpu_public_device_register(struct device *device);
	mpu_public_device_register(&omap1510_ohci_device_ldm);
}

static void
omap1510_ohci_ldm_driver_unregister(void)
{
	extern void mpu_public_driver_unregister(struct device_driver *driver);
	mpu_public_driver_unregister(&omap1510_ohci_driver_ldm);
}

static void
omap1510_ohci_ldm_device_unregister(void)
{
	extern void mpu_public_device_unregister(struct device *device);
	mpu_public_device_unregister(&omap1510_ohci_device_ldm);
}

static int
omap1510_ohci_scale(struct bus_op_point *op, u32 level)
{
	/* REVISIT */
	return 0;
}

static int
omap1510_ohci_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		omap1510_ohci_release();
		break;
	}

	return 0;
}

static int
omap1510_ohci_resume(struct device *dev, u32 level)
{
	int ret = 0;

	switch (level) {
	case RESUME_POWER_ON:

		/* Wake up thread for reinit. */
		init_completion(&kusbdpmd);
		omap1510_usbpmd_req = SIGHUP;
		wake_up(&kusbdpmd_wait);
		break;
	}

	return ret;
}
#endif				/* MVL-CEE */

#if defined(CONFIG_ARCH_OMAP730)
static int __devinit 
omap1510_ohci_configure(void)
{
	int ret;

	isp1301_configure();

	/* set up pinout */
	/* USB pin group 3 configuration */

	/* set up USB Host without OTG function. */

	/* ARM_RSTCT2.0 = 1(PER_EN) ARMPER_nRST pin to high-level output voltage */
	*((volatile __u16 *)ARM_RSTCT2) |= ARM_RSTCT2_PER_EN;

	/* ARM_IDLECT3.0 = 1 (EN_OCPI_CK) Enables the L3 OCPI clock. */
	/* ARM_IDLECT3.1 = 0 (IDLOCPI_ARM) make L3 OCPI clock idle when MPU is idle */
	*((volatile __u16 *)ARM_IDLECT3) &= ~(IDLOCPI_ARM);
	*((volatile __u16 *)ARM_IDLECT3) |= EN_OCPI_CK;

	// Enable everything in OCPI. Seems like everything must be enabled
	*((volatile __u32 *)OCPI_PROT) &= ~(OCPI_PROT_MASK);
	*((volatile __u32 *)OCPI_SEC) &= ~(OCPI_SEC_MASK);

	/* SOFT_REQ_REG.8 = 1 (SOFT_USB_OTG_DPLL_REQ) Software request active for USB OTG for ULPD_PLL */
	/* SOFT_REQ_REG.4 = 0 (USB_REQ_EN) USB Client HW DPLL request disable */
	/* SOFT_REQ_REG.3 = 1 (SOFT_USB_REQ) Software system clock request active for USB host */
	/* SOFT_REQ_REG.0 = 1 (SOFT_DPLL_REQ) Software ULPD_PLL clock request active */
	*((volatile __u16 *)SOFT_REQ_REG) &= ~(USB_REQ_EN);
	*((volatile __u16 *)SOFT_REQ_REG) |= ( SOFT_DPLL_REQ | SOFT_USB_REQ | SOFT_USB_OTG_DPLL_REQ );

	/* Enable USB host system clock. */
	*((volatile __u16 *)SOFT_DISABLE_REQ_REG) &= ~(DIS_USB_HOST_DPLL_REQ); 


	/* USB pin group 0, USB port 0 transceiver mode */
	/* 3 pin DAT/SE0 mode bidirectional */
	*((volatile __u32 *)OTG_SYSCON_1) = (THREE_PIN_BI << USB0_TRX_MODE_POS);
	
	*((volatile __u32 *)OTG_SYSCON_2) = (	USBx_SYNCHRO |	/* USBx_SYNCHRO = 1 */
						UHOST_EN |	/* UHOST_EN = 1 */
						HMC_MODE_1	/* HMC_MODE = 1 */
					    );

	/* OTG_IRQ_EN = 0 */
	*((volatile __u16 *)OTG_IRQ_EN) = 0;

	/*
	 * Fill in some fields of the bogus pci_dev.
	 */
	memset(&bogus_pcidev, 0, sizeof (struct pci_dev));
	strcpy(bogus_pcidev.name, "OMAP1510 OHCI");
	strcpy(bogus_pcidev.slot_name, "builtin");
	bogus_pcidev.resource[0].name = "OHCI Operational Registers";
	bogus_pcidev.resource[0].start = USB_HOST_BASE;
	bogus_pcidev.resource[0].end = USB_HOST_BASE + USB_HOST_SIZE;
	bogus_pcidev.resource[0].flags = 0;
	bogus_pcidev.irq = INT_USB_HHC1;

	/*
	 * Initialise the generic OHCI driver.
	 */
	ret = hc_add_ohci(&bogus_pcidev, bogus_pcidev.irq,
			  (void *) bogus_pcidev.resource[0].start, 0,
			  &omap1510_ohci, "usb-ohci", "omap1510");
	return ret;
}
#else

static int __devinit
omap1510_ohci_configure(void)
{
#if defined(CONFIG_OMAP_H2)
	isp1301_configure();
#endif

#ifndef CONFIG_ARCH_OMAP1610
	/* TO DO:  make a proper header file for all of these registers. */
#ifdef CONFIG_OMAP_INNOVATOR
	volatile unsigned char *fpga_usb_host_ctrl =
	    (unsigned char *) 0xe800020c;
#endif
	volatile unsigned short *apll_ctrl_reg = (unsigned short *) 0xfffe084c;
	volatile unsigned short *dpll_ctrl_reg = (unsigned short *) 0xfffe083c;
	volatile unsigned short *soft_req_reg = (unsigned short *) 0xfffe0834;
	volatile unsigned short *clock_ctrl_reg = (unsigned short *) 0xfffe0830;
	volatile unsigned long *mod_conf_ctrl_0 = (unsigned long *) 0xfffe1080;

	volatile unsigned long *lb_clock_div = (unsigned long *) 0xfffec10c;
	volatile unsigned short *lb_mmu_cntl_reg =
	    (unsigned short *) 0xfffec208;
	volatile unsigned short *lb_mmu_lock_reg =
	    (unsigned short *) 0xfffec224;
	volatile unsigned short *lb_mmu_ld_tlb_reg =
	    (unsigned short *) 0xfffec228;
	volatile unsigned short *lb_mmu_cam_h_reg =
	    (unsigned short *) 0xfffec22c;
	volatile unsigned short *lb_mmu_cam_l_reg =
	    (unsigned short *) 0xfffec230;
	volatile unsigned short *lb_mmu_ram_h_reg =
	    (unsigned short *) 0xfffec234;
	volatile unsigned short *lb_mmu_ram_l_reg =
	    (unsigned short *) 0xfffec238;

	int tlb;
	unsigned long lbaddr, physaddr;
#endif

#ifndef CONFIG_ARCH_OMAP1610

#define APLL_CTRL_REG_APLL_NDPLL_SWITCH		0x0001
#define DPLL_CTRL_REG_PLL_ENABLE		0x0010
#define DPLL_CTRL_REG_LOCK			0x0001
#define SOFT_REQ_REG_DPLL_REQ			0x0001
#define CLOCK_CTRL_REG_USB_MCLK_EN		0x0010
#define MOD_CONF_CTRL_0_USB_HOST_HHC_UHOST_EN	0x00000200

	*apll_ctrl_reg &= ~APLL_CTRL_REG_APLL_NDPLL_SWITCH;
	*dpll_ctrl_reg |= DPLL_CTRL_REG_PLL_ENABLE;
	*soft_req_reg |= SOFT_REQ_REG_DPLL_REQ;
	while (!(*dpll_ctrl_reg & DPLL_CTRL_REG_LOCK)) ;
	*clock_ctrl_reg |= CLOCK_CTRL_REG_USB_MCLK_EN;
	*ARM_IDLECT2 |= (1 << EN_LBFREECK) | (1 << EN_LBCK);
	*mod_conf_ctrl_0 |= MOD_CONF_CTRL_0_USB_HOST_HHC_UHOST_EN;

#ifdef CONFIG_OMAP_INNOVATOR
	*fpga_usb_host_ctrl |= 0x20;
#endif

	*lb_clock_div = (*lb_clock_div & 0xfffffff8) | 0x4;
	*lb_mmu_cntl_reg = 0x3;
	udelay(200);
	for (tlb = 0; tlb < 32; tlb++) {
		lbaddr = tlb * 0x00100000 + OMAP1510_LB_OFFSET;
		physaddr = tlb * 0x00100000 + PHYS_OFFSET;
		*lb_mmu_cam_h_reg = (lbaddr & 0x0fffffff) >> 22;
		*lb_mmu_cam_l_reg = ((lbaddr & 0x003ffc00) >> 6) | 0xc;
		*lb_mmu_ram_h_reg = physaddr >> 16;
		*lb_mmu_ram_l_reg = (physaddr & 0x0000fc00) | 0x300;
		*lb_mmu_lock_reg = tlb << 4;
		*lb_mmu_ld_tlb_reg = 0x1;
	}
	*lb_mmu_cntl_reg = 0x7;
	udelay(200);
#else
#define MOD_CONF_CTRL_0_USB_HOST_HHC_UHOST_EN	0x00000200

	/*set up pinout */
	/*USB pin group 1 configuration */
	OMAP1610_SET_FUNC_MUX_CTRL(1, FUNC_MUX_CTRL_9, 24);	/*USB1.TXD W14 */
	open_drain(4, PULL_DWN_CTRL_2);
	OMAP1610_SET_FUNC_MUX_CTRL(2, FUNC_MUX_CTRL_9, 3);	/*USB1.TXEN W16 */
	open_drain(29, PULL_DWN_CTRL_1);
#ifndef CONFIG_OMAP_H2
	OMAP1610_SET_FUNC_MUX_CTRL(2, FUNC_MUX_CTRL_A, 3);	/*USB1.VP AA17 */
	open_drain(7, PULL_DWN_CTRL_2);
#endif
	OMAP1610_SET_FUNC_MUX_CTRL(4, FUNC_MUX_CTRL_9, 0);	/*USB1.SEO W13 */
	open_drain(28, PULL_DWN_CTRL_1);
	OMAP1610_SET_FUNC_MUX_CTRL(1, FUNC_MUX_CTRL_A, 9);	/*USB1.RCV V15 */
	open_drain(9, PULL_DWN_CTRL_2);
#ifndef CONFIG_OMAP_H2
	OMAP1610_SET_FUNC_MUX_CTRL(2, FUNC_MUX_CTRL_A, 6);	/*USB1.VM P14 */
	open_drain(8, PULL_DWN_CTRL_2);
#endif
#ifdef CONFIG_OMAP_H2
	OMAP1610_SET_FUNC_MUX_CTRL(5, FUNC_MUX_CTRL_A, 12);	/*USB1.SPEED R13 *//*connected on OMAP1610 H2, not INNOVATOR */
	open_drain(10, PULL_DWN_CTRL_2);
#endif

#ifndef CONFIG_OMAP_H2
	/*USB pin group 2 configuration */
	OMAP1610_SET_FUNC_MUX_CTRL(1, FUNC_MUX_CTRL_B, 9);	/*USB2.TXEN W9 */
	open_drain(19, PULL_DWN_CTRL_2);
	OMAP1610_SET_FUNC_MUX_CTRL(1, FUNC_MUX_CTRL_B, 6);	/*USB2.VP AA9 */
	open_drain(18, PULL_DWN_CTRL_2);
	OMAP1610_SET_FUNC_MUX_CTRL(1, FUNC_MUX_CTRL_B, 3);	/*USB2.SUSP Y10 */
	open_drain(17, PULL_DWN_CTRL_2);
	OMAP1610_SET_FUNC_MUX_CTRL(1, FUNC_MUX_CTRL_C, 21);	/*USB2.RCV Y5 */
	open_drain(1, PULL_DWN_CTRL_3);
	OMAP1610_SET_FUNC_MUX_CTRL(1, FUNC_MUX_CTRL_C, 18);	/*USB2.VM R9 */
	open_drain(0, PULL_DWN_CTRL_3);
	OMAP1610_SET_FUNC_MUX_CTRL(2, FUNC_MUX_CTRL_C, 27);	/*USB2.TXD V6 */
	open_drain(3, PULL_DWN_CTRL_3);
	OMAP1610_SET_FUNC_MUX_CTRL(2, FUNC_MUX_CTRL_C, 24);	/*USB2.SE0 W5 */
	open_drain(2, PULL_DWN_CTRL_3);
#endif
	OMAP1610_CONFIRM_MUX_SETUP();

#ifdef CONFIG_OMAP_H2
	outl((inl(USB_TRANSCEIVER_CTRL) & ~(1 << 7)), USB_TRANSCEIVER_CTRL);
#else
	outl((inl(USB_TRANSCEIVER_CTRL) | (1 << 7)), USB_TRANSCEIVER_CTRL);
	outl((inl(USB_TRANSCEIVER_CTRL) | (1 << 8)), USB_TRANSCEIVER_CTRL);
#endif

	/*set up USB Host without OTG function.
	   OTG registers still must be configured */

	//RESET_CONTROL.0 = 1 (CONF_OCP_RESET_R) //page oct03.448
	//RESET_CONTROL.2 = 1 (CONF_ARMIO_RESET_R)
	outl(inl(RESET_CONTROL) | 5, RESET_CONTROL);

	//ARM_RSTCT2.0 = 1(PER_EN)
	outl(inl(ARM_RSTCT2) | 1, ARM_RSTCT2);

	//ARM_IDLECT3.0 = 1 (EN_OCPI_CK)
	//ARM_IDLECT3.1 = 0 (IDLOCPI_ARM) //make OCPI idle when MPU is idle
	outl(inl(ARM_IDLECT3) | 1, ARM_IDLECT3);
	outl(inl(ARM_IDLECT3) & ~(1 << 1), ARM_IDLECT3);

	// Enable everything in OCPI. Seems like everything must be enabled
	outl(inl(OCPI_PROT) & ~0xff, OCPI_PROT);
	outl(inl(OCPI_SEC) & ~0x7f, OCPI_SEC);

	//CLOCK_CTRL_REG.5 = 0 (DIS_USB_PVCI_CLK)
	//CLOCK_CTRL_REG.4 = 1 (USB_MCLK_EN)
	outl((inl(CLOCK_CTRL_REG) & ~(1 << 5)), CLOCK_CTRL_REG);
	outl((inl(CLOCK_CTRL_REG) | (1 << 4)), CLOCK_CTRL_REG);

	//MOD_CONF_CTRL_0.9 = 1 (CONF_MOD_USB_HOST_HHC_UHOST_EN_R) //enable 48 and 12 MHz clocks
	//MOD_CONF_CTRL_0.11 = 0 (CONF_MOD_USB_HOST_UART_SELECT_R) //enable uart on USB port 1
	outl((inl(MOD_CONF_CTRL_0) | (1 << 9)), MOD_CONF_CTRL_0);
	outl((inl(MOD_CONF_CTRL_0) & ~(1 << 11)), MOD_CONF_CTRL_0);

	//SOFT_REQ_REG.8 = 1 (SOFT_USB_OTG_DPLL_REQ)
	//SOFT_REQ_REG.4 = x (USB_REQ_EN) //USB Client HW DPLL req
	//SOFT_REQ_REG.3 = 1 (SOFT_USB_REQ)
	//SOFT_REQ_REG.0 = 1 (SOFT_DPLL_REQ) //ULPD_PLL clock req
	outl((inl(SOFT_REQ_REG) | (1 << 0) | (1 << 3) | (1 << 8)),
	     SOFT_REQ_REG);

	//SOFT_DISABLE_REQ_REG.6 = 0 (DIS_USB_HOST_DPLL_REQ) //USB Host clock HW
	//SOFT_DISABLE_REQ_REG.3 = 0 (DIS_PERIPH_REQ)
	outl((inl(SOFT_DISABLE_REQ_REG) & ~((1 << 6) | (1 << 3))),
	     SOFT_DISABLE_REQ_REG);

	//? OTG_SYSCON_1.1 = 1 (SOFT_RST)
	//? OTG_SYSCON_1.2 == 1 ? (RESET_DONE)

	//OTG_SYSCON_1.15 = 0 (OTG_IDLE_EN)
	//OTG_SYSCON_1.13 = 0 (DEV_IDLE_EN) //user choice
	//OTG_SYSCON_1.1 = 0 (SOFT_RST)
	//OTG_SYSCON_1.26-24 = 3(USB2_TRXMODE) //user choice
	//OTG_SYSCON_1.22-20 = 3(USB1_TRXMODE) //user choice
	//OTG_SYSCON_1.18-16 = x (USB0_TRXMODE) //user choice
#ifdef CONFIG_OMAP_H2
	outl((2 << 20), OTG_SYSCON_1);
#else
	outl((inl(OTG_SYSCON_1) & (7 << 16)), OTG_SYSCON_1);
	outl((inl(OTG_SYSCON_1) | (3 << 20) | (3 << 24)), OTG_SYSCON_1);
#endif

	//OTG_SYSCON_2.31 = 0 (OTG_EN)
	//OTG_SYSCON_2.30 = 1 (USBx_SYNCHRO)
	//OTG_SYSCON_2.29 = 0 (OTG_MST16)
	//OTG_SYSCON_2.28 = 0 (SRP_GPDATA)
	//OTG_SYSCON_2.27 = 0 (SRP_GPDVBUS)
	//OTG_SYSCON_2.26-24 = 0 (SRP_GPUVBUS)
	//OTG_SYSCON_2.22-20 = 0 (A_WAIT_RISE)
	//OTG_SYSCON_2.18-16 = 4 (B_ASE0_BRST)
	//OTG_SYSCON_2.14 = 0 (SRP_DPW)
	//OTG_SYSCON_2.13 = 0 (SRP_DATA)
	//OTG_SYSCON_2.12 = 0 (SRP_VBUS)
	//OTG_SYSCON_2.10 = 0 (OTG_PADEN)
	//OTG_SYSCON_2.9 = 0 (HMC_PADEN)
	//OTG_SYSCON_2.8 = 1 (UHOST_EN) //user choice
	//OTG_SYSCON_2.7 = 0 (HMC_TTLSPEED) //user choice
	//OTG_SYSCON_2.6 = 0 (HMC_TTLATTACH) //user choice
	//OTG_SYSCON_2.5-0 = 4 (HMC_MODE) //user choice
	outl((1 << 30) | (4 << 16) | (1 << 8) | 4, OTG_SYSCON_2);

	//OTG_CTRL.18 = x (BSESSVLD) //1 = host attached (VBUS present)
	//other bits must be 0
	outl((inl(OTG_CTRL) & (1 << 18)), OTG_CTRL);

	//OTG_IRQ_EN = 0
	outl(0, OTG_IRQ_EN);

#endif
	/*
	 * Fill in some fields of the bogus pci_dev.
	 */
	memset(&bogus_pcidev, 0, sizeof (struct pci_dev));
	strcpy(bogus_pcidev.name, "OMAP1510 OHCI");
	strcpy(bogus_pcidev.slot_name, "builtin");
	bogus_pcidev.resource[0].name = "OHCI Operational Registers";
	bogus_pcidev.resource[0].start = 0xfffba000;
	bogus_pcidev.resource[0].end = 0xfffba000 + 4096;	/* REVISIT */
	bogus_pcidev.resource[0].flags = 0;
	bogus_pcidev.irq = IH2_BASE + 6;

	/*
	 * Initialise the generic OHCI driver.
	 */
	return hc_add_ohci(&bogus_pcidev, bogus_pcidev.irq,
			   (void *) bogus_pcidev.resource[0].start, 0,
			   &omap1510_ohci, "usb-ohci", "omap1510");
}
#endif /* defined(CONFIG_ARCH_OMAP730) */

static void
omap1510_ohci_release(void)
{
#if !defined(CONFIG_ARCH_OMAP730)
	/* REVISIT: Need to properly shut off clocks here. */
	volatile unsigned long *mod_conf_ctrl_0 = (unsigned long *) 0xfffe1080;
#endif

	hc_remove_ohci(omap1510_ohci);

#if !defined(CONFIG_ARCH_OMAP730)
	*mod_conf_ctrl_0 &= ~MOD_CONF_CTRL_0_USB_HOST_HHC_UHOST_EN;
#endif

#if defined(CONFIG_ARCH_OMAP1610) || defined(CONFIG_ARCH_OMAP730)
	outl(inl(OTG_SYSCON_2) & ~(1 << 8), OTG_SYSCON_2);
#endif
#if defined(CONFIG_OMAP_H2) || defined(CONFIG_MACH_OMAP_PERSEUS2)
	isp1301_vbus_off();
#endif
}

#if defined(CONFIG_OMAP_H2) || defined(CONFIG_MACH_OMAP_PERSEUS2)
/* Release IO region */
static void
omap_h2_release_io(void)
{
	if (initstate_region) {
		release_region(OTG_BASE, OTG_IOSIZE);
		initstate_region = 0;
	}
	if (initstate_i2c) {
		i2c_del_driver(&isp1301_driver);
		initstate_i2c = 0;
	}
}
#endif

static int __init
omap1510_ohci_init(void)
{
#if defined(CONFIG_OMAP_H2) || defined(CONFIG_MACH_OMAP_PERSEUS2)
	int tmp =
	    (int) request_region(OTG_BASE, OTG_IOSIZE, "OMAP H2 USB OHCI");
	if (!tmp) {
		err("OTG is already in use");
		omap_h2_release_io();
		return -ENODEV;
	}
	initstate_region = 1;

	tmp = i2c_add_driver(&isp1301_driver);
	if (tmp < 0) {		// didn't attach
		err("failed to connect I2C to ISP1301 USB Transceiver");
		omap_h2_release_io();
		return tmp;
	}
	initstate_i2c = 1;
#endif

#if	defined(CONFIG_CEE)	/* MVL-CEE */
	init_waitqueue_head(&kusbdpmd_wait);
	init_completion(&kusbdpmd);
	omap1510_usbpmd_req = SIGHUP;
	if ((omap1510_ohci_pid = kernel_thread(omap1510_ohci_thread,
					       (void *) NULL,
					       CLONE_FS | CLONE_FILES |
					       CLONE_SIGHAND)) <= 0) {
#if defined(CONFIG_OMAP_H2) || defined(CONFIG_MACH_OMAP_PERSEUS2)
		omap_h2_release_io();
#endif
		return omap1510_ohci_pid;
	}
	wait_for_completion(&kusbdpmd);

	omap1510_ohci_ldm_driver_register();
	omap1510_ohci_ldm_device_register();
	return 0;
#else				/* MVL-CEE */
	return omap1510_ohci_configure();
#endif				/* MVL-CEE */
}

static void __exit
omap1510_ohci_exit(void)
{
#if	defined(CONFIG_CEE)	/* MVL-CEE */
	omap1510_ohci_ldm_device_unregister();
	omap1510_ohci_ldm_driver_unregister();

	/* Kill the thread */
	init_completion(&kusbdpmd);
	omap1510_usbpmd_req = SIGTERM;
	wake_up(&kusbdpmd_wait);
	wait_for_completion(&kusbdpmd);
#endif				/* MVL-CEE */
	omap1510_ohci_release();
#if defined(CONFIG_OMAP_H2) || defined(CONFIG_MACH_OMAP_PERSEUS2)
	omap_h2_release_io();
#endif
}

module_init(omap1510_ohci_init);
module_exit(omap1510_ohci_exit);

MODULE_LICENSE("GPL");
