/*
 *  arch/arm/mach-mx2ads/gpio.c
 *
 *  The Motorola MX2 GPIO driver/framework
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Copyright 2002 Sony Corporation.
 *  Copyright (C) 2003 MontaVista Software Inc. <source@mvista.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  version 2 of the  License.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
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
 */

#include <linux/module.h>
#include <linux/version.h>

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/proc_fs.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/errno.h>

#include <asm/arch/irqs.h>
#include <asm/arch/hardware.h>

#include <asm/arch/gpio.h>

#define ESUCCESS 0
static mx2_gpio_t gpio[6];

static gpio_callback_t gpio_callback[6 * 32];	/* pointers to GPIO IRQ callback functions for every bit of every port */

/*
 *  debugging code. mx2_gpio_debuglevel means the maximum level of
 *  messages will be sent to console:
 *      0 - only critical errors;
 *      1 - severe errors;
 *      2 - warnings;
 *      3 - all the debugging messages
 *  Looks reasonable to set it to non-zero if you suspect problems with
 *  GPIO modules/GPIO itself
 *
 *  MX2_GPIO_RANGE allows you to verify the range of passed parameter,
 *  something like assert(), but only range-checking
 */

#define MX2GPIO_DEBUG 0

static int debug =
#ifdef MX2GPIO_DEBUG
    MX2GPIO_DEBUG;
#else
    0;
#endif

int
mx2_gpio_setdebug(int new_debuglevel)
{
#ifdef MX2GPIO_DEBUG
	int old_debuglevel = debug;

	debug = new_debuglevel;
	printk("%s: debug level set to %d instead of %d\n",
	       __FUNCTION__, new_debuglevel, old_debuglevel);
	return old_debuglevel;
#else
	return -1;
#endif
}

#ifdef MX2GPIO_DEBUG
#define mx2_debug( level, params ) if( (level) <= debug ) printk params;
#else
#define mx2_debug( level, paramlist )
#endif

#define MX2_GPIO_RANGE( val, v1, v2 )                           \
    if( (val)>v2 || (val)<v1 ) {                                \
	mx2_debug( 1, ( KERN_ERR"%s: %s must be in range 0x%X..0x%X\n", \
	     __FUNCTION__, #val, v1, v2 ));                     \
	return -EINVAL;                                         \
    }

/*
 * Function:
 *   mx2_register_gpio
 * Synopsis:
 *   register CPU-GPIO use.
 * Parmaters
 *     port:   GPIO port ( PORT_A/PORT_B/PORT_C/...PORT_F )
 *     bitnum: Specify bit to use in GPIO port
 *     mode:   Any valid combination of following:
 *              = GPIO / PRIMARY / SECONDARY
 *                 select GPIO or CPU function (error if not specified)
 *                 sets bit at GIUS and GPR
 *              - INPUT / OUTPUT :
 *                 sets bit at DDIR (error if not specified when GPIO)
 *              - OCR_A / OCR_B / OCR_C / OCR_DATA :
 *                 sets bit at OCRn (default OCR_DATA)
 *              - ICONFA_IN / ICONFA_INT / ICONFA_0 / ICONFA_1
 *                 sets bit at ICONFAn (default ICONFA_IN)
 *              - ICONFB_IN / ICONFB_INT / ICONFB_0 / ICONFB_1
 *                 sets bit at ICONFAn (default ICONFA_IN)
 *              - PULLUP / TRISTATE / NOINTERRUPT
 *                 sets bit at PUEN (default PULLUP)
 *              - INIT_DATA_0 / INIT_DATA_1
 *                 set initial data for DR when output.
 *     name:   String to appear at /proc/gpio
 *
 *   Returns:
 *     status of operation ( 0 on success )
 */
int
mx2_register_gpio(unsigned int port, unsigned int bitnum,
		  unsigned int mode /* , const char *name */ )
{
	unsigned int val;
	unsigned long flags;

	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	MX2_GPIO_RANGE(bitnum, 0, 31);

	mx2_debug(3, ("%s: Configuring PORT %c, bit %d\n",
		      __FUNCTION__, port - PORT_A + 'A', bitnum));
	/* check number of bits set for essesntials */
	if (hweight32(mode & (GPIO | PRIMARY | SECONDARY)) != 1) {
		mx2_debug(1,
			  ("%s: only one of GPIO/PRIMARY/SECONDARY should be set\n",
			   __FUNCTION__));
		return -EINVAL;
	}

	if (mode & GPIO) {
		if (hweight32(mode & (INPUT | OUTPUT)) != 1) {
			mx2_debug(1,
				  ("%s: only one of INPUT/OUTPUT should be set\n",
				   __FUNCTION__));
			return -EINVAL;
		}
	}

	local_irq_save(flags);

	/* check if used */
	if (gpio[port].used & (1 << bitnum)) {
		mx2_debug(1, ("%s: GPIO (port=%d, bit=%d) is reserved\n",
			      __FUNCTION__, port, bitnum));
	} else {
		gpio[port].used |= (1 << bitnum);
		gpio[port].bit[bitnum].mode = mode;
	}

	if (mode & INPUT) {
		mx2_debug(3,
			  ("%s: INPUT mode is configuring, bit = %d\n",
			   __FUNCTION__, bitnum));
		/* set ICONF A */
		if (bitnum < 16) {
			val = GPIO_ICONFA1(port);
			val &= ~(0x3 << (bitnum * 2));
			switch (mode & (3 << ICONFA_BIT)) {
			case ICONFA_1:
				val |= ICONF_1 << (bitnum * 2);
				break;
			case ICONFA_0:
				val |= ICONF_0 << (bitnum * 2);
				break;
			case ICONFA_INT:
				val |= ICONF_ISR << (bitnum * 2);
				break;
			case ICONFA_IN:
			default:
				val |= ICONF_GPIO_IN << (bitnum * 2);
				break;
			}
			GPIO_ICONFA1(port)=val;
		} else {
			val = GPIO_ICONFA2(port);
			val &= ~(0x3 << ((bitnum - 16) * 2));
			switch (mode & (3 << ICONFA_BIT)) {
			case ICONFA_1:
				val |= ICONF_1 << ((bitnum - 16) * 2);
				break;
			case ICONFA_0:
				val |= ICONF_0 << ((bitnum - 16) * 2);
				break;
			case ICONFA_INT:
				val |= ICONF_ISR << ((bitnum - 16) * 2);
				break;
			case ICONFA_IN:
			default:
				val |= ICONF_GPIO_IN << ((bitnum - 16) * 2);
				break;
			}
			GPIO_ICONFA2(port) = val;
		}

		/* set ICONF B */
		if (bitnum < 16) {
			val = GPIO_ICONFB1(port);
			val &= ~(0x3 << (bitnum * 2));
			switch (mode & (3 << ICONFB_BIT)) {
			case ICONFB_1:
				val |= ICONF_1 << (bitnum * 2);
				break;
			case ICONFB_0:
				val |= ICONF_0 << (bitnum * 2);
				break;
			case ICONFB_INT:
				val |= ICONF_ISR << (bitnum * 2);
				break;
			case ICONFB_IN:
			default:
				val |= ICONF_GPIO_IN << (bitnum * 2);
				break;
			}
			GPIO_ICONFB1(port) = val;;
		} else {
			val = GPIO_ICONFB2(port);
			val &= ~(0x3 << ((bitnum - 16) * 2));
			switch (mode & (3 << ICONFB_BIT)) {
			case ICONFB_1:
				val |= ICONF_1 << ((bitnum - 16) * 2);
				break;
			case ICONFB_0:
				val |= ICONF_0 << ((bitnum - 16) * 2);
				break;
			case ICONFB_INT:
				val |= ICONF_ISR << ((bitnum - 16) * 2);
				break;
			case ICONFB_IN:
			default:
				val |= ICONF_GPIO_IN << ((bitnum - 16) * 2);
				break;
			}
			GPIO_ICONFB2(port)=val;
		}

		/* set DDIR */
		val = GPIO_DDIR(port);
		val &= ~(1 << bitnum);
		GPIO_DDIR(port)=val;

	} else if (mode & OUTPUT) {

		mx2_debug(3,
			  ("%s: OUTPUT mode is configuring, bit = %d\n",
			   __FUNCTION__, bitnum));

		/* set initial data output value */
		val = GPIO_DR(port);
		if (mode & INIT_DATA_1) {
			val |= 1 << bitnum;
		} else {
			val &= ~(1 << bitnum);
		}
		GPIO_DR(port)=val;

		/* set OCR */
		if (bitnum < 16) {
			val = GPIO_OCR1(port);
			val &= ~(0x3 << (bitnum * 2));
			switch (mode & (3 << OCR_BIT)) {
			case OCR_DATA:
				val |= OCR_DATA_REG << (bitnum * 2);
				break;
			case OCR_C:
				val |= OCR_C_IN << (bitnum * 2);
				break;
			case OCR_B:
				val |= OCR_B_IN << (bitnum * 2);
				break;
			case OCR_A:
			default:
				val |= OCR_A_IN << (bitnum * 2);
				break;
			}
			GPIO_OCR1(port)=val;
		} else {
			val = GPIO_OCR2(port);
			val &= ~(0x3 << ((bitnum - 16) * 2));
			switch (mode & (3 << OCR_BIT)) {
			case OCR_DATA:
				val |= OCR_DATA_REG << ((bitnum - 16) * 2);
				break;
			case OCR_C:
				val |= OCR_C_IN << ((bitnum - 16) * 2);
				break;
			case OCR_B:
				val |= OCR_B_IN << ((bitnum - 16) * 2);
				break;
			case OCR_A:
			default:
				val |= OCR_A_IN << ((bitnum - 16) * 2);
				break;
			}
			GPIO_OCR2(port)=val;
		}

		/* set DDIR */
		mx2_debug(3,
			  ("%s: DDIR/setting bit %d\n", __FUNCTION__, bitnum));
		val = GPIO_DDIR(port);
		val |= 1 << bitnum;
		GPIO_DDIR(port)=val;

	}

	if (!(mode & NOINTERRUPT)) {
		/* set PUEN */
		val = GPIO_PUEN(port);
		if (mode & TRISTATE) {
			mx2_debug(3,
				  ("%s: PUEN/ clearing bit %d\n", __FUNCTION__,
				   bitnum));
			val &= ~(1 << bitnum);
		} else {
			mx2_debug(3,
				  ("%s: PUEN/ setting bit %d\n", __FUNCTION__,
				   bitnum));
			val |= 1 << bitnum;
		}
		GPIO_PUEN(port)=val;
	}

	/* set GIUS */
	if (mode & GPIO) {	/* use as GPIO function */
		/* set GIUS */
		val = GPIO_GIUS(port);
		mx2_debug(3,
			  ("%s: GIUS/ setting bit %d\n", __FUNCTION__, bitnum));
		GPIO_GIUS(port) = val | (1 << bitnum);
	} else {		/* use as CPU function */

		/* unset GIUS */
		val = GPIO_GIUS(port);
		mx2_debug(3,
			  ("%s: GIUS/ clearing bit %d\n", __FUNCTION__,
			   bitnum));
		val &= ~(1 << bitnum);
		GPIO_GIUS(port) = val;

		/* set GPR for PRIMARY or SECONDARY function */
		val = GPIO_GPR(port);
		if (mode & PRIMARY) {
			mx2_debug(3,
				  ("%s: GPR/ clearing bit %d\n", __FUNCTION__,
				   bitnum));
			val &= ~(1 << bitnum);
		} else {
			mx2_debug(3,
				  ("%s: GPR/ setting bit %d\n", __FUNCTION__,
				   bitnum));
			val |= 1 << bitnum;
		}
		GPIO_GPR(port) = val;
	}

	local_irq_restore(flags);

	return ESUCCESS;
}

/*
 * Clears registered gpio.
 */
int
mx2_unregister_gpio(unsigned int port, unsigned int bitnum)
{
	unsigned long flags;

	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	MX2_GPIO_RANGE(bitnum, 0, 31);
	local_irq_save(flags);

	/* cleanup */
	if (gpio[port].used & (1 << bitnum)) {
		gpio[port].used &= ~(1 << bitnum);
		gpio[port].bit[bitnum].mode = 0;
	} else {
		mx2_debug(2,
			  ("%s: warning - port 0x%x, bit %d is already unregistered\n",
			   __FUNCTION__, port, bitnum));
	}

	local_irq_restore(flags);
	return ESUCCESS;
}

/*
 *  Register several CPU-GPIO use.
 *
 *  Applies same 'mode' and 'name' for specified bitmask.
 *    !! can only be used bits at the same port !!
 *
 *   Returns:
 *     Upon success, returns 0.
 *     If failed, negative value is returned.
 *
 */
int
mx2_register_gpios(unsigned int port, unsigned long bitmask, unsigned int mode)
{
	int bit, retval = 0;
	unsigned long done = 0;

	mx2_debug(3, ("%s: bitmask = %08X\n", __FUNCTION__, (int)bitmask));
	for (bit = 0; bit < 32; bit++) {
		if (bitmask & (1 << bit)) {
			if (0 != (retval = mx2_register_gpio(port, bit, mode))) {
				goto error;
			} else {
				set_bit(bit, &done);
			}
		}
	}
	return ESUCCESS;

      error:
	mx2_unregister_gpios(port, done);
	return retval;
}

/*
 *  Unregister several CPU-GPIO use.
 */
int
mx2_unregister_gpios(unsigned int port, unsigned long bitmask)
{
	int bit;

	for (bit = 0; bit < 32; bit++) {
		if (bitmask & (1 << bit)) {
			mx2_unregister_gpio(port, bit);
		}
	}
	return ESUCCESS;
}

/*
 * Gets value of read-only GPIO SSR at specified bitnum of port
 * Returns: 1 or 0
 */
unsigned
mx2_gpio_get_bit(int port, int bitnum)
{
	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	return ((GPIO_SSR(port) & (1 << bitnum)) ? 1 : 0);
}

/*
 * Gets value of read-only GPIO SSR at specified port
 */
unsigned int
mx2_gpio_get(int port)
{
	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	return (unsigned int) (GPIO_SSR(port));
}

/*
 * Sets value of GPIO DATA Register (DR)  at specified bitnum of port.
 * Requires:
 *  - 0 or non-zero (=1) as value
 */
int
mx2_gpio_set_bit(int port, int bitnum, int value)
{
	unsigned int val;
	unsigned long flags;

	MX2_GPIO_RANGE(port, PORT_A, PORT_F);

	local_irq_save(flags);
	val = GPIO_DR(port);
	val = (val & ~(1 << bitnum)) | ((value ? 1 : 0) << bitnum);
	GPIO_DR(port)=val;
	local_irq_restore(flags);

	return ESUCCESS;
}

/*
 * Configure interrupt setting, for bitnum of port configured as input.
 * Requires:
 *  - intrconf can be one of
 *     POSITIVE_EDGE
 *     NEGATIVE_EDGE
 *     POSITIVE_LEVEL
 *     NEGATIVE_LEVEL
 */
int
mx2_gpio_config_intr(int port, int bitnum, int intrconf,
		     gpio_callback_t callback)
{
	unsigned int val;
	unsigned long flags;

	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	MX2_GPIO_RANGE(bitnum, 0, 31);

	if (bitnum < 16) {
		local_irq_save(flags);
		val = GPIO_ICR1(port);
		mx2_debug(3, ("%s: %08X + GPIO_ICR1 ( %08X ) setting ~%02X\n",
			      __FUNCTION__, (int)GPIO_BASE(port), (int)GPIO_ICR1(port), val));
		val =
		    (val & ~(0x3 << (bitnum * 2))) | (intrconf << (bitnum * 2));
		GPIO_ICR1(port)=val;
		local_irq_restore(flags);
	} else {
		local_irq_save(flags);
		val = GPIO_ICR2(port);
		val = ((val & ~(0x3 << ((bitnum - 16) * 2))) |
		       (intrconf << ((bitnum - 16) * 2)));
		mx2_debug(3, ("%s: %08X + GPIO_ICR2 ( %08X ) setting ~%02X\n",
			      __FUNCTION__, (int)GPIO_BASE(port), (int)GPIO_ICR2(port), val));
		GPIO_ICR2(port)=val;
		local_irq_restore(flags);
	}
	gpio_callback[port * 32 + bitnum] = callback;
	return ESUCCESS;
}

/*
 * Gets interrupt config parameter
 */
int
mx2_gpio_get_intr_config(int port, int bitnum)
{
	unsigned int val;
	unsigned long flags;

	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	MX2_GPIO_RANGE(bitnum, 0, 31);

	if (bitnum < 16) {
		local_irq_save(flags);
		val = ((GPIO_ICR1(port)) >> (bitnum * 2)) & 0x3;
		local_irq_restore(flags);
	} else {
		local_irq_save(flags);
		val = ((GPIO_ICR2(port)) >> ((bitnum-16) * 2)) & 0x3;
		local_irq_restore(flags);
	}
	mx2_debug(3, ("%s: val = %08X\n", __FUNCTION__, val));
	return val;
}

/*
 * Gets interrupt status of specified bitnum of port
 * Returns: 1 or 0
 */
int
mx2_gpio_intr_status_bit(int port, int bitnum)
{
	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	MX2_GPIO_RANGE(bitnum, 0, 31);

	return (((GPIO_ISR(port)) & (1 << bitnum)) ? 1 : 0);
}

int
mx2_gpio_intr_mask_bit(int port, int bitnum)
{
	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	MX2_GPIO_RANGE(bitnum, 0, 31);

	return (((GPIO_IMR(port)) & (1 << bitnum)) ? 1 : 0);
}

/*
 * Gets interrupt status of specified port
 */
unsigned int
mx2_gpio_intr_status(int port)
{
	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	return (unsigned int) (GPIO_ISR(port));
}

/*
 * Clear specified GPIO interrupt
 */
int
mx2_gpio_clear_intr(int port, int bitnum)
{
	unsigned int v;
	unsigned long flags;

	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	MX2_GPIO_RANGE(bitnum, 0, 31);

	local_irq_save(flags);
	v = GPIO_ISR(port);
	mx2_debug(2, ("%s: status = 0x%08X\n", __FUNCTION__, v));
	GPIO_ISR(port) =  v | (1 << bitnum);
	local_irq_restore(flags);

	return ESUCCESS;
}

/*
 * Mask specified GPIO interrupt
 */
int
mx2_gpio_mask_intr(int port, int bitnum)
{
	unsigned long flags;

	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	MX2_GPIO_RANGE(bitnum, 0, 31);

	local_irq_save(flags);
	GPIO_IMR(port) = GPIO_IMR(port) & ~(1 << bitnum);
	local_irq_restore(flags);

	return ESUCCESS;
}

/*
 * Unmask specified GPIO interrupt to activate
 */
int
mx2_gpio_unmask_intr(int port, int bitnum)
{
	unsigned long flags;

	MX2_GPIO_RANGE(port, PORT_A, PORT_F);
	MX2_GPIO_RANGE(bitnum, 0, 31);

	local_irq_save(flags);
	GPIO_IMR(port) = GPIO_IMR(port) | (1 << bitnum);
	local_irq_restore(flags);

	return ESUCCESS;
}

static void
gpio_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	int port, bit;

	for (port = 0; port < 6; port++) {
		for (bit = 0; bit < 32; bit++) {
			if (mx2_gpio_intr_status_bit(port, bit) &
			    mx2_gpio_intr_mask_bit(port, bit)) {
				if (gpio_callback[port * 32 + bit]) {
					gpio_callback[port * 32 + bit] (irq,
									dev_id,
									regs);
				}
				break;
			}
		}
	}
}


/*
 * MX2 GPIO Intitializer/termination function
 *
 */
int __init
mx2_gpio_init(void)
{
	unsigned int err;

	mx2_debug(1, ("GPIO framework on MX2 DragonBall, compiled %s %s\n",
		      __DATE__, __TIME__));
	err = request_irq(INT_GPIO, gpio_irq_handler, SA_INTERRUPT | SA_SHIRQ,
			  "gpio", "gpio");


	return ESUCCESS;
}

module_init(mx2_gpio_init)

MODULE_LICENSE("GPL");

EXPORT_SYMBOL(mx2_gpio_setdebug);
EXPORT_SYMBOL(mx2_register_gpio);
EXPORT_SYMBOL(mx2_unregister_gpio);
EXPORT_SYMBOL(mx2_register_gpios);
EXPORT_SYMBOL(mx2_unregister_gpios);

EXPORT_SYMBOL(mx2_gpio_get_bit);
EXPORT_SYMBOL(mx2_gpio_get);
EXPORT_SYMBOL(mx2_gpio_set_bit);
EXPORT_SYMBOL(mx2_gpio_config_intr);
EXPORT_SYMBOL(mx2_gpio_intr_status_bit);
EXPORT_SYMBOL(mx2_gpio_intr_status);
EXPORT_SYMBOL(mx2_gpio_clear_intr);
EXPORT_SYMBOL(mx2_gpio_mask_intr);
EXPORT_SYMBOL(mx2_gpio_unmask_intr);

EXPORT_SYMBOL(mx2_gpio_get_intr_config);
