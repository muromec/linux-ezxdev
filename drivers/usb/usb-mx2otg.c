/******************************************************************************

	usb-mx2otg.c
	driver for Motorola Dragonball MX2 USB OTG
	This module configures GPIO multiplexing, and contols role switching
	between USB host and USB device.
	Designed to work with usb-mx2hcd and device/bi/mx2.

	Copyright (c) 2004 MontaVista Software, Inc. <source@mvista.com>

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

********************************************************************************/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/completion.h>

#include <asm/irq.h>
#include <asm/system.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/arch/gpio.h>
#include <asm/proc/cache.h>
#include <asm/io.h>
#include <asm/arch/pll.h>


#ifdef CONFIG_PM
/* power management stuff */
#include <linux/pm.h>
struct pm_dev *mx2otg_pm_dev;
#endif
#if 1 /*CEE LDM*/
static char mx2otg_ldm_inited;

#include <linux/device.h>

extern void mx21_ldm_bus_register(struct device *device,
                          struct device_driver *driver);
extern void mx21_ldm_bus_unregister(struct device *device,
                          struct device_driver *driver);
static int
__ldm_suspend(struct device *dev, u32 state, u32 level);

static int
__ldm_resume(struct device *dev, u32 level);

static struct device_driver __driver_ldm = {
	.name = "usb-mx2otg",
	.suspend = __ldm_suspend,
	.resume = __ldm_resume,
};

static struct device __device_ldm = {
	.name = "USB OTG",
	.bus_id = "usb_otg",
	.driver = &__driver_ldm,
	.power_state = DPM_POWER_ON,
};

#endif

#include <linux/usb.h>
#include "usb-mx2otg.h"

static int
 mx2otg_i2cRegRead(u8 reg);
static int
 mx2otg_i2cRegWrite(u8 reg, u8 data);

static char mx2otg_dma_irq_inited;
static char mx2otg_gpioB_inited;
static char mx2otg_gpioC_inited;
static char mx2otg_gpioC8_inited;
static char mx2otg_region_inited;
static char mx2otg_thread_inited;
static char mx2otg_thread_terminating;
static char mx2otg_device_state;
#define MX2OTG_B_DEVICE_STATE 2
#define MX2OTG_A_DEVICE_STATE 1

static struct mx2otg_host_descr_st *mx2otg_host_struct;
static struct mx2otg_device_descr_st *mx2otg_device_struct;

DECLARE_MUTEX_LOCKED(mx2otg_mutex_id);
DECLARE_COMPLETION(mx2otg_thread_completion);
DECLARE_MUTEX(mx2otg_device_lock);

static int mx2otg_thread(void *data);
static void mx2otg_disable_a_device(void);
static void mx2otg_disable_b_device(void);


#ifdef CONFIG_PM
static int
mx2otg_pm_callback(struct pm_dev *pm_dev, pm_request_t request, void *unused)
{
	switch (request) {

	case PM_SUSPEND:
		dbg("power management suspend");
		mx2otg_disable_a_device();
		mx2otg_disable_b_device();
		mx_module_clk_close(HCLK_MODULE_USBOTG);
		mx_module_clk_close(IPG_MODULE_USBOTG);
		break;

	case PM_RESUME:
		dbg("power management resume");
		mx_module_clk_open(HCLK_MODULE_USBOTG);
		mx_module_clk_open(IPG_MODULE_USBOTG);
		up(&mx2otg_mutex_id);
		break;
	}
	return 0;
}
#endif
#if 1 /*CEE LDM*/
static int
__ldm_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		dbg("suspend");
		mx2otg_disable_a_device();
		mx2otg_disable_b_device();
		mx_module_clk_close(HCLK_MODULE_USBOTG);
		mx_module_clk_close(IPG_MODULE_USBOTG);
		break;
	}
	return 0;
}

static int
__ldm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		dbg("resume");
		mx_module_clk_open(HCLK_MODULE_USBOTG);
		mx_module_clk_open(IPG_MODULE_USBOTG);
		up(&mx2otg_mutex_id);
		break;
	}
	return 0;
}
#endif


#define MX2HCD_DATAMEM_SIZE 4096

/* describes a reserved block in OTG DATA memory*/
struct mx2otg_mem_descr_st {
	u16 addr;
	unsigned long size;
	struct mx2otg_mem_descr_st *next;
};

/* points to the list of reserved blocks in OTG DATA memory */
static struct mx2otg_mem_descr_st *data_mem_list;

/* allocates "size" (normaly equal to maxpacket) of bytes in OTG DATA memory.
returns a relative starting address of the block requested or -ENOMEM. */
int
mx2otg_alloc_DATA_mem(u16 size)
{
	u16 addr;
	struct mx2otg_mem_descr_st **p, *tmp, *area;
	size += (~size + 1) & 0x3;	/* size must be word-aligned */
	if (size > MX2HCD_DATAMEM_SIZE)
		return -ENOMEM;

	area =
	    (struct mx2otg_mem_descr_st *) kmalloc(sizeof (*area), GFP_ATOMIC);
	if (!area)
		return -ENOMEM;

	addr = 0;

	for (p = &data_mem_list; (tmp = *p); p = &tmp->next) {
		if ((size + addr) < addr)
			goto out;
		if (size + addr <= tmp->addr)
			break;
		addr = tmp->size + tmp->addr;
		if (addr > MX2HCD_DATAMEM_SIZE - size)
			goto out;
	}
	area->addr = addr;
	area->size = size;
	area->next = *p;
	*p = area;

	return addr;

      out:

	kfree(area);
	return -ENOMEM;

}

/* frees previously allocated OTG DATA memory
descr_addr - realtive starting address in the OTG DATA memory */
void
mx2otg_free_DATA_mem(u16 descr_addr)
{

	struct mx2otg_mem_descr_st **p, *tmp;

	for (p = &data_mem_list; (tmp = *p); p = &tmp->next) {
		if (tmp->addr == descr_addr) {
			*p = tmp->next;
			kfree(tmp);
			return;
		}
	}
	err("Trying to free nonexistent OTG DATA memory area, offset %x",
	    descr_addr);
}

int
mx2otg_register_host(struct mx2otg_host_descr_st *mx2otg_host_descr)
{
	if (!mx2otg_host_struct) {
		mx2otg_host_struct = mx2otg_host_descr;
		up(&mx2otg_mutex_id);
		return 0;
	} else
		return -EBUSY;
}

void
mx2otg_unregister_host(struct mx2otg_host_descr_st *mx2otg_host_descr)
{
	if (mx2otg_host_struct == mx2otg_host_descr) {
		mx2otg_disable_a_device();	/*disable A device if it was enabled */

		mx2otg_host_struct = 0;
	} else
		err("freeing non-existent host descriptor");
}

int
mx2otg_register_device(struct mx2otg_device_descr_st *mx2otg_device_descr)
{
	if (!mx2otg_device_struct) {
		mx2otg_device_struct = mx2otg_device_descr;
		up(&mx2otg_mutex_id);
		return 0;
	} else
		return -EBUSY;

}

void
mx2otg_unregister_device(struct mx2otg_device_descr_st *mx2otg_device_descr)
{
	if (mx2otg_device_struct == mx2otg_device_descr) {
		mx2otg_disable_b_device();	/*disable B device if it was enabled */
		mx2otg_device_struct = 0;
	} else
		err("freeing non-existent device descriptor");
}

/*-------------------------------------------------------------------------*/

/* Pinout:
  ***USB Transceiver:
  Pin:        Exp Con: CPU Con: CPU: GPIO: Mode:
  USBG_SCL    2.B8     PY1.42   B12  PC6   Primary
  USBG_SDA    2.B9     PY1.43   G12  PC5   Primary
  USBG_FS     2.B6     PY1.40   A13  PC8   GPIO-A-out (USBG_TRX_INT)
  USBG_OE_B   2.B5     PY1.36   H13  PC9   Primary
  USBG_TXDP   2.B3     PY1.39   G13  PC11  Primary
  USBG_TXDM   2.B4     PY1.41   B13  PC10  Primary
  USBG_RXDP   2.B1     PY1.35   D14  PC13  Primary
  USBG_RXDM   2.B2     PY1.37   C13  PC12  Primary

  RESET_OUT_B 2.C11    PY1.80   V15  PE17  Primary  - configured in the bootloader?
  USBG_ON_B   2.B7     PY1.52   D13  PC7   Primary

  ***MAX3355EEUD:
  USB_PWR     2.C14    PY1.54   B11  PB23  Primary
  SSI1_TXD    1.C10    PY2.38   C15  PC22  GPIO     - ?
  SSI2_RXD    2.A15    PY2.35   A17  PC25  GPIO     - ?

  Note: USBG stands for USB OTG. There are also USB Host1 and 2, not connected on MX2ADS.
  Seems like USB OTG is Port 1. USB Host 1 and 2 must be disconnected in GPIO multiplexing.
*/

#define MX2HOST_PORT_B_MASK (1<<23)
#define MX2HOST_PORT_C_MASK ((1<<5)|(1<<6)|(1<<7)|(1<<9)|(1<<10)|(1<<11)|(1<<12)|(1<<13))

/*
58 INT_USBCTRL Control Interrupt and I2C interrupt
57 INT_USBHNP  HNP Interrupt
56 INT_USBFUNC Function Interrupt
55 INT_USBHOST Host interrupt
54 INT_USBDMA  DMA interrupt
53 INT_USBWKUP Wakeup Interrupt
*/

#define OTG_TXCVR_DEV_WRITE_ADDR 0x2D
#define OTG_TXCVR_DEV_READ_ADDR (0x2D | (1<<7))

static int __init mx2ads_usbotg_init(void);
static void __init mx2ads_usbotg_exit(void);

static void
mx2otg_dma_interrupt(int this_irq, void *dev_id, struct pt_regs *regs)
{
	dbg("dma interrupt");
	if (mx2otg_host_struct) {
		if (mx2otg_host_struct->dma_err)
			mx2otg_host_struct->dma_err();
	}
	if (mx2otg_device_struct) {
		if (mx2otg_device_struct->dma_err)
			mx2otg_device_struct->dma_err();
	}
	OTG_DMA_DINT_STAT = 1;

}

static int
mx2otg_enable_b_device(void)
{
	int ret = 0;
	down(&mx2otg_device_lock);
	if ((mx2otg_device_state != MX2OTG_B_DEVICE_STATE)
	    && (mx2otg_device_struct)) {
		dbg("enable B device");
		OTG_CORE_HNP_CSTAT |= (1 << 22);	/* enable Function Mode */
		mx2otg_i2cRegWrite(OTG_TXCVR_CTRL_REG1_CLR, 0x4);	/* disable D+ pull-down */
		mx2otg_i2cRegWrite(OTG_TXCVR_CTRL_REG1_SET, 0x1);	/* enable D+ pull-up */
		mx2otg_device_state = MX2OTG_B_DEVICE_STATE;
		if (mx2otg_device_struct->enable) {
			ret = mx2otg_device_struct->enable();
		}
	}
	up(&mx2otg_device_lock);
	return ret;
}

static void
mx2otg_disable_b_device(void)
{
	down(&mx2otg_device_lock);
	if (mx2otg_device_state != MX2OTG_B_DEVICE_STATE) {
		up(&mx2otg_device_lock);
		return;
	}
	dbg("disable B device");

	mx2otg_i2cRegWrite(OTG_TXCVR_CTRL_REG1_CLR, 0x1);	/* disable D+ pull-up */
	mx2otg_i2cRegWrite(OTG_TXCVR_CTRL_REG1_SET, 0x4);	/* enable D+ pull-down */

	if ((mx2otg_device_struct)
	    && (mx2otg_device_struct->disable)) {
		mx2otg_device_struct->disable();
	}

	OTG_CORE_HNP_CSTAT &= ~(1 << 22);	/* disable Function Mode */
	mx2otg_device_state = 0;
	up(&mx2otg_device_lock);
}

static int
mx2otg_enable_a_device(void)
{
	int ret = 0;
	down(&mx2otg_device_lock);
	if ((mx2otg_device_state != MX2OTG_A_DEVICE_STATE)
	    && (mx2otg_host_struct)) {
		dbg("enable A device");

		OTG_CORE_HNP_CSTAT |= (1 << 21);	/* enable Host Mode */
		mx2otg_i2cRegWrite(OTG_TXCVR_CTRL_REG1_SET, 0x20);	/* turn on Vbus */
		mx2otg_device_state = MX2OTG_A_DEVICE_STATE;

		if (mx2otg_host_struct->enable) {
			ret = mx2otg_host_struct->enable();
		}
	}
	up(&mx2otg_device_lock);
	return ret;
}

static void
mx2otg_disable_a_device(void)
{
	down(&mx2otg_device_lock);
	if (mx2otg_device_state != MX2OTG_A_DEVICE_STATE) {
		up(&mx2otg_device_lock);
		return;
	}
	dbg("disable A device");
	if ((mx2otg_host_struct)
	    && (mx2otg_host_struct->disable))
		mx2otg_host_struct->disable();

	mx2otg_i2cRegWrite(OTG_TXCVR_CTRL_REG1_CLR, 0x20);	/* turn off Vbus */
	OTG_CORE_HNP_CSTAT &= ~(1 << 21);	/* disable Host Mode */
	mx2otg_device_state = 0;
	up(&mx2otg_device_lock);
}

#define ISP1301_ID_GROUNDED  (1<<3)

static void
mx2otg_process_id_pin(void)
{
	int intsrc;
	mx2otg_i2cRegWrite(OTG_TXCVR_INT_LAT_REG_CLR, 0xff);
	intsrc = mx2otg_i2cRegRead(OTG_TXCVR_INT_SRC_REG);

	if (intsrc < 0) {
		err("failed to read I2C txceiver's int src register");
		return;
	}

	if (intsrc & ISP1301_ID_GROUNDED) {	/* is A device */
		dbg("id pin grounded - A dev");

		mx2otg_disable_b_device();	/* disable B device if it was enabled */
		if (mx2otg_enable_a_device() < 0)
			err("error enabling USB host");

	} else {		/*is B device */
		dbg("id pin floating - B dev");
		mx2otg_disable_a_device();	/* disable A device if it was enabled */
		if (mx2otg_enable_b_device() < 0)
			err("error enabling USB device");
	}
}

static int
mx2otg_thread(void *data)
{
	daemonize();
	reparent_to_init();
	strcpy(current->comm, "mx2otg_id");

	for (;;) {
		down(&mx2otg_mutex_id);
		if (mx2otg_thread_terminating == 1)
			break;
		mx2otg_process_id_pin();
	}
	complete_and_exit(&mx2otg_thread_completion, 0);
	return 0;
}

static void
mx2otg_id_pin_interrupt(int this_irq, void *dev_id, struct pt_regs *regs)
{

	if (mx2_gpio_intr_status_bit(PORT_C, 8)) {
		mx2_gpio_clear_intr(PORT_C, 8);
		up(&mx2otg_mutex_id);
	}
}

static int
mx2otg_waitI2Cbusy(void)
{
	unsigned long timeout;
	int cnt = 0;

	timeout = jiffies + HZ;

#if 0
	/* poll for i2c ready interrupt */
	while (!(OTG_I2C_MASTER_INT_REG & 0x02)) {
		if (time_after(jiffies, timeout)) {
			err("I2C busy waiting timeout");
			return -ETIMEDOUT;
		}
		schedule_timeout(1);
	}
	/* clear the RWREADY bit after it set */
	if ((OTG_I2C_MASTER_INT_REG & 0x2)) {
		OTG_I2C_MASTER_INT_REG = 0x2;
	}
#else
	while (OTG_I2C_OP_CTRL_REG & 0x80) {	/* waiting for bus to become free */
		if (time_after(jiffies, timeout)) {
			err("I2C busy waiting timeout");
			return -ETIMEDOUT;
		}
		cnt++;
		if (!in_interrupt())
			schedule_timeout(1);

	}
#endif
	dbg("wait count %d", cnt);
	return 0;
}

static int
mx2otg_i2cRegWrite(u8 reg, u8 data)
{
	OTG_I2C_TXCVR_REG(reg) = data;
	OTG_I2C_SEQ_RD_STARTAD = reg;	/* set sequential read start address */
	OTG_I2C_SEQ_OP_REG = 1;	/* set number of sequential operations */
	OTG_I2C_OTG_XCVR_DEVAD = OTG_TXCVR_DEV_WRITE_ADDR;	/* set transceiver's I2C device address, start operation */
	return mx2otg_waitI2Cbusy();
}

static int
mx2otg_i2cRegRead(u8 reg)
{
	OTG_I2C_SEQ_RD_STARTAD = reg;	/* set sequential read start address */
	OTG_I2C_SEQ_OP_REG = 1;	/* set number of sequential operations */
	OTG_I2C_OTG_XCVR_DEVAD = OTG_TXCVR_DEV_READ_ADDR;	/* set transceiver's I2C device address, start operation */
	if (mx2otg_waitI2Cbusy() < 0)
		return -ETIMEDOUT;
	return OTG_I2C_TXCVR_REG(reg);
}

static int __init
mx2otg_reset(void)
{
	unsigned long timeout;
	OTG_CORE_RST_CTRL = (1 << 15) | 0x3F;

	timeout = jiffies + HZ;
	while (OTG_CORE_RST_CTRL) {
		if (time_after(jiffies, timeout)) {
			return -ETIMEDOUT;
		}
		schedule_timeout(1);
	}
	return 0;
}

static int __init
mx2otg_clk_en(void)
{
	unsigned long timeout;
	OTG_CORE_CLK_CTRL = 0x7;

	timeout = jiffies + HZ;

	while (!((OTG_CORE_CLK_CTRL & 0x7) == 0x7)) {
		if (time_after(jiffies, timeout)) {
			return -ETIMEDOUT;
		}
		schedule_timeout(1);
	}
	return 0;
}

static inline int __init
mx2ads_usbotg_setup_clock(void)
{
#if 0
#define MX2OTG_48MHZ 48000000

u32 tmpclock = mx_module_get_clk(SPLL);

	tmpclock = tmpclock/MX2OTG_48MHZ + ((tmpclock%MX2OTG_48MHZ > ((u32)MX2OTG_48MHZ >> 1)) ? 1 : 0);

	if (!tmpclock) {
		err("SPLL clock too slow");
		return -1;
	}
	tmpclock--;

	if (tmpclock > ((u32)CSCR_USB_DIV>>CSCR_USB_DIV_SHIFT)) {
		err("SPLL clock too fast");
		return -1;
	}

	CRM_CSCR = (CRM_CSCR & ~CSCR_USB_DIV) | (tmpclock << CSCR_USB_DIV_SHIFT);
#else
	/*I'm using a fixed value (5=div by 6) here because it works in cases of SPLL 234.88(TV-Out attached)
	and 266MHz (TV-Out not attached).
	The name of the clock it generates (CLK48M) suggests it should be 48MHz,
	but in practice it is rather far from 48MHz in our cases.
	Moreover, attempts to approximate clock to this value may result in USB not working: in case of 234.88MHz
	the above algorithm tries to divide the SPLL clock into 5, not 6, and the USB doesn't
	communicate with attached devices or host.
	After all, I don't know what are the real requirements to setting USB_DIV field in the CSCR register.
	Let's fix it, if necessary, later, when more use cases or more description is avaliable*/
	CRM_CSCR = (CRM_CSCR & ~CSCR_USB_DIV) | (5 << CSCR_USB_DIV_SHIFT);
#endif

	return 0;
}

/* module init */
static int __init
mx2ads_usbotg_init(void)
{
	long tmp;

	dbg("MX2 OTG Driver ver. 1.0 built on %s %s", __TIME__, __DATE__);

	/* check revision */
	dbg("OTG_CORE_HWMODE: %x. 16 MSb's indicate revision",
	    (int) OTG_CORE_HWMODE);

	/* request SSI register region */

	if (!(request_region(OTG_BASE, OTG_IO_SIZE, "mx2ads - otg host"))) {
		err("OTG is already in use");
		mx2ads_usbotg_exit();
		return -EBUSY;
	}
	mx2otg_region_inited = 1;
	
	if (mx2ads_usbotg_setup_clock() < 0) {
		mx2ads_usbotg_exit();
		return -EFAULT;
	}


	/* enable USB OTG Clk and USB OTG H Clk */
	mx_module_clk_open(HCLK_MODULE_USBOTG);
	mx_module_clk_open(IPG_MODULE_USBOTG);

	/* configure GPIO for USBOTG uses */
	tmp =
	    mx2_register_gpios(PORT_B, MX2HOST_PORT_B_MASK, PRIMARY | TRISTATE);
	if (tmp < 0) {
		mx2ads_usbotg_exit();
		err("GPIO Pins needed are not available (B)");
		return -EBUSY;
	}
	mx2otg_gpioB_inited = 1;

	tmp =
	    mx2_register_gpios(PORT_C, MX2HOST_PORT_C_MASK, PRIMARY | TRISTATE);
	if (tmp < 0) {
		mx2ads_usbotg_exit();
		err("GPIO Pins needed are not available (C)");
		return -EBUSY;
	}
	mx2otg_gpioC_inited = 1;

	tmp = mx2_register_gpio(PORT_C, 8, GPIO | INPUT | PULLUP);
	if (tmp < 0) {
		mx2ads_usbotg_exit();
		err("A GPIO Pin needed is not available (C-8)");
		return -EBUSY;
	}
	mx2otg_gpioC8_inited = 1;
	mx2_gpio_mask_intr(PORT_C, 8);
	mx2_gpio_config_intr(PORT_C, 8, NEGATIVE_EDGE,
			     &mx2otg_id_pin_interrupt);

	tmp = request_irq(INT_USBDMA, mx2otg_dma_interrupt, 0, "OTG", NULL);
	if (tmp) {
		mx2ads_usbotg_exit();
		err("failed to request DMA irq\n");
		return -EFAULT;
	}
	mx2otg_dma_irq_inited = 1;

	/*init thread */
	mx2otg_thread_terminating = 0;
	tmp =
	    kernel_thread(&mx2otg_thread, NULL,
			  CLONE_FS | CLONE_FILES | CLONE_SIGHAND);
	if (tmp < 0) {
		mx2ads_usbotg_exit();
		err("could not start thread");
		return tmp;
	}
	mx2otg_thread_inited = 1;

	/*##############CONFIGURING OTG CORE################# */

	/* reset the whole OTG */
	if ((tmp = mx2otg_reset()) < 0) {
		mx2ads_usbotg_exit();
		err("Timeout waiting for reset to complete");
		return tmp;
	}

	/*enable clocks */
	if ((tmp = mx2otg_clk_en()) < 0) {
		mx2ads_usbotg_exit();
		err("Timeout waiting for clock enable to complete");
		return tmp;
	}

	/*OTG_CORE_HWMODE current settings:
	   Debounce - no information on applicability in the ref manual
	   bit 14 = 0 - enable. Enabled by HW default
	   OTG transceiver configuration:
	   bits 7:6 = 00 - differential transmit/receive (TxDp, TxDm, RxDp, RxDm, RxD)
	   Host transceiver configuration (do we care ?):
	   bits 5:4 = 00 - same as for the OTG
	   OTG Module Configuration:
	   bits 0:1 = 11 - enable Host, Device and Software HNP. Host only mode seems to be not-functional
	 */

	OTG_CORE_HWMODE = (OTG_CORE_HWMODE & 0xFFFFFF3C) | 3;

#ifdef CONFIG_MX2TO1
	OTG_SYS_CTRL = 0;	/*disable bypass, disable wakeup interrupts, unmask power pins */
#endif

	/*##############CONFIGURING OTG HNP################# */

	OTG_CORE_HNP_CSTAT = 0;	/*disable host and function. They'll be enabled later */

	/*##############CONFIGURING OTG TRANSCEIVER(I2C)################# */
	OTG_I2C_SCLK_TO_SCL_HPER = 0xF0;	/* configure clock as 100kHz */
	OTG_I2C_MASTER_INT_REG &= 0x0f;	/* disable all interrupts */
	OTG_I2C_MASTER_INT_REG &= 0xf0;	/* clear all interrupts */

	OTG_I2C_OP_CTRL_REG = 0x01;	/* enable I2C output, software controlled mode */

	mx2otg_waitI2Cbusy();
	/*pull-down dp and dm, turn off VBUS */
	mx2otg_i2cRegWrite(OTG_TXCVR_MODE_REG1_CLR, 0xFF);
	mx2otg_i2cRegWrite(OTG_TXCVR_MODE_REG2_CLR, 0xFF);
	mx2otg_i2cRegWrite(OTG_TXCVR_CTRL_REG1_SET, 0xC);
	mx2otg_i2cRegWrite(OTG_TXCVR_CTRL_REG1_CLR, ~0xC);

	mx2otg_i2cRegWrite(OTG_TXCVR_INT_FALSE_REG_CLR, 0xFF);
	mx2otg_i2cRegWrite(OTG_TXCVR_INT_TRUE_REG_SET, ISP1301_ID_GROUNDED);	/*enable ID pin change interrupts */
	mx2otg_i2cRegWrite(OTG_TXCVR_INT_TRUE_REG_CLR, ~ISP1301_ID_GROUNDED);
	mx2otg_i2cRegWrite(OTG_TXCVR_INT_FALSE_REG_SET, ISP1301_ID_GROUNDED);	/*enable ID pin change interrupts */
	mx2otg_i2cRegWrite(OTG_TXCVR_INT_FALSE_REG_CLR, ~ISP1301_ID_GROUNDED);
	mx2otg_i2cRegWrite(OTG_TXCVR_INT_LAT_REG_CLR, 0xFF);

	mx2_gpio_unmask_intr(PORT_C, 8);

	/*configure USB DMA */
	OTG_DMA_MISC_CTRL = 3 | (1 << 3);	/* select round robin mode, also filter on ETD completion code */
	OTG_DMA_DINT_STAT = 1;	/*clear error int status */
	OTG_DMA_DINT_STEN = 1;	/*interrupt enable */

#if 1 /*CEE LDM*/
	mx21_ldm_bus_register(&__device_ldm, &__driver_ldm);
	mx2otg_ldm_inited = 1;
#endif
#ifdef CONFIG_PM
	mx2otg_pm_dev =
	    pm_register(PM_USB_DEV, PM_SYS_UNKNOWN, mx2otg_pm_callback);
	if (!mx2otg_pm_dev)
		warn("failed to initialize power management");
#endif

	return 0;
}

/* module exit*/
static void __init
mx2ads_usbotg_exit(void)
{

#if 1 /*CEE LDM*/
	if (mx2otg_ldm_inited)
		mx21_ldm_bus_unregister(&__device_ldm, &__driver_ldm);
#endif
#ifdef CONFIG_PM
	if (mx2otg_pm_dev)
		pm_unregister(mx2otg_pm_dev);
#endif

	if (mx2otg_thread_inited) {
		mx2otg_thread_terminating = 1;
		up(&mx2otg_mutex_id);
		wait_for_completion(&mx2otg_thread_completion);
	}

	if (mx2otg_dma_irq_inited) {
		free_irq(INT_USBDMA, NULL);
	}

	if (mx2otg_gpioB_inited) {
		mx2_unregister_gpios(PORT_B, MX2HOST_PORT_B_MASK);
	}

	if (mx2otg_gpioC8_inited) {
		mx2_gpio_mask_intr(PORT_C, 8);
		mx2_unregister_gpio(PORT_C, 8);
	}

	if (mx2otg_gpioC_inited) {
		mx2_unregister_gpios(PORT_C, MX2HOST_PORT_C_MASK);
	}

	if (mx2otg_region_inited) {
		release_region(OTG_BASE, OTG_IO_SIZE);
		mx_module_clk_close(HCLK_MODULE_USBOTG);
		mx_module_clk_close(IPG_MODULE_USBOTG);
	}

}

module_init(mx2ads_usbotg_init);
module_exit(mx2ads_usbotg_exit);

MODULE_AUTHOR("MontaVista Software Inc");
MODULE_DESCRIPTION("Motorola i.MX21 USB OTG");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(mx2otg_alloc_DATA_mem);
EXPORT_SYMBOL(mx2otg_free_DATA_mem);
EXPORT_SYMBOL(mx2otg_register_host);
EXPORT_SYMBOL(mx2otg_unregister_host);
EXPORT_SYMBOL(mx2otg_register_device);
EXPORT_SYMBOL(mx2otg_unregister_device);
