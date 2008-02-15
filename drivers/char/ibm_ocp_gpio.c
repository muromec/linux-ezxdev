/*
 * FILE NAME ibm_ocp_gpio.c
 *
 * BRIEF MODULE DESCRIPTION
 *  API for IBM PowerPC 4xx GPIO device.
 *  Driver for IBM PowerPC 4xx GPIO device.
 *
 *  Armin Kuster akuster@pacbell.net
 *  Sept, 2001
 *
 *  Orignial driver
 *  Author: MontaVista Software, Inc.  <source@mvista.com>
 *          Frank Rowand <frank_rowand@mvista.com>
 *          Debbie Chu   <debbie_chu@mvista.com>
 *
 * Copyright 2000,2001,2002 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE	LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *	TODO: devfs
 *
 *	Version: 02/01/12 - Armin
 *			 converted to ocp and using ioremap
 *
 *	1.2 02/21/01 - Armin
 *		minor compiler warning fixes
 *
 *	1.3 02/22/01 - Armin
 *		added apm
 *
 *	1.4 05/07/02 - Armin/David Mueller
 *		coverted to core_ocp[];
 *
 *	1.5 05/25/02 - Armin
 *	 name change from *_driver to *_dev
 *
 *	1.6 06/04/02 - Matt Porter
 *	ioremap paddr. Comment as 4xx generic driver.
 *	Fix header to be userland safe and locate in
 *	an accessible area.  Add ioctl to configure
 *	multiplexed GPIO pins.
 *
 *	1.7 07/25/02 - Armin
 *	added CPM to enable/disable in init/exit
 *
 */

#define VUFX "07.25.02"

#include <linux/module.h>
#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/pm.h>
#include <linux/ibm_ocp_gpio.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/ocp.h>

struct miscdevice ibm_gpio_miscdev;
static struct gpio_regs *gpiop;

#ifdef CONFIG_PM
static struct pm_dev *pm_gpio;

static int
gpio_save_state(u32 state)
{
	return 0;
}

static int
gpio_suspend(u32 state)
{
	mtdcr(DCRN_CPMFR, mfdcr(DCRN_CPMFR) | state);
	return 0;
}

static int
gpio_resume(u32 state)
{
	mtdcr(DCRN_CPMFR, mfdcr(DCRN_CPMFR) & ~state);
	return 0;
}
#endif

int
ibm_gpio_config(__u32 device, __u32 mask, __u32 data)
{
	u32 cfg_reg;

	if (device != 0)
		return -ENXIO;
#ifdef CONFIG_PM
	pm_access(pm_gpio);
#endif

#ifdef CONFIG_40x
	/*
	 * PPC405 uses CPC0_CR0 to select multiplexed GPIO pins.
	 */
	cfg_reg = mfdcr(DCRN_CHCR0);
	cfg_reg = (cfg_reg & ~mask) | (data & mask);
	mtdcr(DCRN_CHCR0, cfg_reg);
#elif CONFIG_440
	/*
	 * PPC440 uses CPC0_GPIO to select multiplexed GPIO pins.
	 */
	cfg_reg = mfdcr(DCRN_CPC0_GPIO);
	cfg_reg = (cfg_reg & ~mask) | (data & mask);
	mtdcr(DCRN_CPC0_GPIO, cfg_reg);
#else
#error This driver is only supported on PPC40x and PPC440 CPUs
#endif

	return 0;
}

int
ibm_gpio_tristate(__u32 device, __u32 mask, __u32 data)
{
	if (device != 0)
		return -ENXIO;
#ifdef CONFIG_PM
	pm_access(pm_gpio);
#endif
	gpiop->tcr = (gpiop->tcr & ~mask) | (data & mask);
	return 0;
}

int
ibm_gpio_open_drain(__u32 device, __u32 mask, __u32 data)
{
	if (device != 0)
		return -ENXIO;
#ifdef CONFIG_PM
	pm_access(pm_gpio);
#endif
	gpiop->odr = (gpiop->odr & ~mask) | (data & mask);

	return 0;
}

int
ibm_gpio_in(__u32 device, __u32 mask, volatile __u32 * data)
{
	if (device != 0)
		return -ENXIO;
#ifdef CONFIG_PM
	pm_access(pm_gpio);
#endif
	gpiop->tcr = gpiop->tcr & ~mask;
	eieio();

	/*
	   ** If the previous state was OUT, and gpiop->ir is read once, then the
	   ** data that was being OUTput will be read.  One way to get the right
	   ** data is to read gpiop->ir twice.
	 */

	*data = gpiop->ir;
	*data = gpiop->ir & mask;
	eieio();
	return 0;
}

int
ibm_gpio_out(__u32 device, __u32 mask, __u32 data)
{
	if (device != 0)
		return -ENXIO;
#ifdef CONFIG_PM
	pm_access(pm_gpio);
#endif
	gpiop->or = (gpiop->or & ~mask) | (data & mask);
	eieio();
	gpiop->tcr = gpiop->tcr | mask;
	eieio();
	return 0;
}

static int
ibm_gpio_open(struct inode *inode, struct file *file)
{
	MOD_INC_USE_COUNT;

	return 0;
}

static int
ibm_gpio_release(struct inode *inode, struct file *file)
{
	MOD_DEC_USE_COUNT;

	return 0;
}

static int
ibm_gpio_ioctl(struct inode *inode, struct file *file,
	       unsigned int cmd, unsigned long arg)
{
	static struct ibm_gpio_ioctl_data ioctl_data;
	int status;

	switch (cmd) {
	case IBMGPIO_IN:
		if (copy_from_user(&ioctl_data,
				   (struct ibm_gpio_ioctl_data *) arg,
				   sizeof (ioctl_data))) {
			return -EFAULT;
		}

		status = ibm_gpio_in(ioctl_data.device,
				     ioctl_data.mask, &ioctl_data.data);
		if (status != 0)
			return status;

		if (copy_to_user((struct ibm_gpio_ioctl_data *) arg,
				 &ioctl_data, sizeof (ioctl_data))) {
			return -EFAULT;
		}

		break;

	case IBMGPIO_OUT:
		if (copy_from_user(&ioctl_data,
				   (struct ibm_gpio_ioctl_data *) arg,
				   sizeof (ioctl_data))) {
			return -EFAULT;
		}

		return ibm_gpio_out(ioctl_data.device,
				    ioctl_data.mask, ioctl_data.data);

		break;

	case IBMGPIO_OPEN_DRAIN:
		if (copy_from_user(&ioctl_data,
				   (struct ibm_gpio_ioctl_data *) arg,
				   sizeof (ioctl_data))) {
			return -EFAULT;
		}

		return ibm_gpio_open_drain(ioctl_data.device,
					   ioctl_data.mask, ioctl_data.data);

		break;

	case IBMGPIO_TRISTATE:
		if (copy_from_user(&ioctl_data,
				   (struct ibm_gpio_ioctl_data *) arg,
				   sizeof (ioctl_data)))
			return -EFAULT;

		return ibm_gpio_tristate(ioctl_data.device,
					 ioctl_data.mask, ioctl_data.data);

		break;

	case IBMGPIO_CFG:
		if (copy_from_user(&ioctl_data,
				   (struct ibm_gpio_ioctl_data *) arg,
				   sizeof (ioctl_data)))
			return -EFAULT;

		return ibm_gpio_config(ioctl_data.device,
				ioctl_data.mask, ioctl_data.data);

		break;

	default:
		return -ENOIOCTLCMD;

	}
	return 0;
}

static struct file_operations ibm_gpio_fops = {
	owner:THIS_MODULE,
	ioctl:ibm_gpio_ioctl,
	open:ibm_gpio_open,
	release:ibm_gpio_release,
};

static int __init
ibm_gpio_init(void)
{
	int curr_gpio = 0;
	struct ocp_dev *gpio_dev;

	printk("IBM gpio driver version %s\n", VUFX);
	while (curr_gpio != -ENXIO) {
		if (!(gpio_dev = ocp_alloc_dev(0)))
			return -ENOMEM;

		gpio_dev->type = GPIO;
		if ((curr_gpio = ocp_register(gpio_dev)) == -ENXIO) {
			ocp_free_dev(gpio_dev);
			break;
		} else {
			ibm_gpio_miscdev.minor = 185;	/*GPIO_MINOR; */
			ibm_gpio_miscdev.name = gpio_dev->name;
			ibm_gpio_miscdev.fops = &ibm_gpio_fops;
			misc_register(&ibm_gpio_miscdev);	/*ibm_gpio_miscdev); */

			gpiop = (struct gpio_regs *) ioremap(gpio_dev->paddr,
					sizeof (struct gpio_regs));
			mtdcr(DCRN_CPMFR, mfdcr(DCRN_CPMFR)
					& ~ocp_get_pm(GPIO, curr_gpio));
			printk("GPIO #%d at 0x%lx\n", curr_gpio,
			       (unsigned long) gpiop);

		}
	}

	return (curr_gpio == -ENXIO) ? 0 : curr_gpio;
}

static void __exit
ibm_gpio_exit(void)
{
	int i;
	struct ocp_dev *gpio_dev;

	for (i = 0; i < ocp_get_max(GPIO); i++) {
		gpio_dev = ocp_get_dev(GPIO, i);
		misc_deregister(&ibm_gpio_miscdev);
		mtdcr(DCRN_CPMFR, mfdcr(DCRN_CPMFR) | ocp_get_pm(GPIO, i));
		ocp_unregister(gpio_dev);
	}
}

module_init(ibm_gpio_init);
module_exit(ibm_gpio_exit);

EXPORT_SYMBOL(ibm_gpio_tristate);
EXPORT_SYMBOL(ibm_gpio_open_drain);
EXPORT_SYMBOL(ibm_gpio_in);
EXPORT_SYMBOL(ibm_gpio_out);

MODULE_LICENSE("GPL");
