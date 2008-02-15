/*
 * File: pwt.c
 *
 *   Driver for Pulse Width Tone (PWT) control.
 *
 * Created 2001, Copyright (C) 2001 RidgeRun, Inc.  All rights reserved.
 * Author: Steve Johnson, stevej@ridgerun.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
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
 * Please report all bugs/problems to the author or <support@dsplinux.net>
 *
 * key: RRGPLCR (do not remove)
 *
 */

/*
 * Use versioning if needed
 */
#include <linux/config.h>	/* retrieve the CONFIG_* macros */
#ifdef CONFIG_MODVERSIONS
#   undef MODVERSIONS		/* it might be defined */
#   define MODVERSIONS
#endif

#ifdef MODVERSIONS
#  include <linux/modversions.h>
#endif

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/devfs_fs_kernel.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/arch/pwt_ioctl.h>

MODULE_AUTHOR("Copyright 2001 RidgeRun, Inc.");
MODULE_DESCRIPTION("PWT driver");

#undef PWT_DEBUG

#ifdef PWT_DEBUG
#define DBG(format, arg...) printk(KERN_DEBUG __FUNCTION__ ": " format "\n" , ##arg)
#define static
#else
#define DBG(x...)
#endif

#define PWT_BASE_ADDRESS 0xfffb6000

#define FRC (PWT_BASE_ADDRESS + 0x0)
#define VRC (PWT_BASE_ADDRESS + 0x4)
#define GCR (PWT_BASE_ADDRESS + 0x8)

typedef struct {
	devfs_handle_t device;
} driver_struct_t;

static driver_struct_t pwt;

/*
  Table of available frequencies.
*/

#define NUM_OCTAVES 4
#define NUM_FREQS   12
static const u16 freq[NUM_OCTAVES][NUM_FREQS] = {
	{5274, 4978, 4699, 4435, 4186, 3951, 3729, 3520, 3322, 3136, 2960,
	 2794},
	{2637, 2489, 2349, 2217, 2093, 1976, 1865, 1760, 1661, 1568, 1480,
	 1397},
	{1319, 1245, 1175, 1109, 1047, 988, 932, 880, 831, 784, 740, 698},
	{659, 622, 587, 554, 523, 494, 466, 440, 415, 392, 370, 349}
};

static inline void
orb(unsigned char val, unsigned long reg)
{
	outb((inb(reg) | val), reg);
}

static inline void
andb(unsigned char val, unsigned long reg)
{
	outb((inb(reg) & val), reg);
}

static int
pwt_open(struct inode *inode, struct file *filep)
{
	filep->private_data = 0;
	filep->f_pos = 0;

	MOD_INC_USE_COUNT;
	return 0;
}

static int
pwt_release(struct inode *inode, struct file *filep)
{
	MOD_DEC_USE_COUNT;
	return 0;
}

static int
pwt_ioctl(struct inode *inode, struct file *filep, unsigned int cmd,
	  unsigned long arg)
{
	int err = 0;
	u16 value;
	u8 octave = 0;
	u8 note = 0;
	int count;

	DBG("cmd - %x, arg - %ld", cmd, arg);

	/* Range check the IOCTL cmd */
	if ((_IOC_TYPE(cmd) != PWT_MAGIC_NUMBER) ||
	    (_IOC_NR(cmd) > PWT_IOC_MAXNR)) {
		DBG("Bad ioctl value");
		return -EINVAL;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE, (void *) arg, _IOC_SIZE(cmd));
		if (err) {
			DBG("access_ok(VERIFY_WRITE) error");
		}
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void *) arg, _IOC_SIZE(cmd));
		if (err) {
			DBG("access_ok(VERIFY_READ) error");
		}
	}
	if (err)
		return -EFAULT;

	switch (cmd) {
	case PWT_ENABLE:
		orb(1, VRC);
		break;
	case PWT_DISABLE:
		andb(~1, VRC);
		break;
	case PWT_SET_FREQUENCY:

		copy_from_user(&value, (const void *) arg, sizeof (value));
		// Simple note/octave setting
		if (value < 0x30) {
			outb(value, FRC);
		} else {
			// Snap to limits 
			if (value > freq[0][0])
				value = freq[0][0];	// max
			if (value < freq[3][11])
				value = freq[3][11];	// min

			// Group into buckets
			if (value < ((freq[3][0] + freq[2][11]) / 2)) {
				octave = 3;
			} else if (value < ((freq[2][0] + freq[1][11]) / 2)) {
				octave = 2;
			} else if (value < ((freq[1][0] + freq[0][11]) / 2)) {
				octave = 1;
			}

			note = PWT_f;	// fall through for lowest value in octave
			for (count = 0; count < NUM_FREQS; count++) {
				if (value > freq[octave][count]) {
					if (count == 0) {
						note = count;
					} else {	// split the difference between freqs
						if (value >
						    ((freq[octave][count] +
						      freq[octave][count -
								   1]) / 2)) {
							note = count - 1;
						} else {
							note = count;
						}
					}
					break;

				}
			}

			outb(FREQ(note, octave), FRC);
		}
		break;
	case PWT_GET_FREQUENCY:
		value = inb(FRC);
		copy_to_user((void *) arg, &value, sizeof (value));
		break;

		// Volume is set/read as percentages of max
	case PWT_SET_VOLUME:
		copy_from_user(&value, (const void *) arg, sizeof (value));
		value = (value - 1) * 64 / 100;
		value = (((value & 0x3f) << 1) | (inb(VRC) & 0x1));
		outb(value, VRC);
		break;
	case PWT_GET_VOLUME:
		value = (inb(VRC) >> 1);
		value = (value + 1) * 100 / 64;
		copy_to_user((void *) arg, &value, sizeof (value));
		break;
	case PWT_SET_FRC:
		copy_from_user(&value, (const void *) arg, sizeof (value));
		if (value > 0x2f)
			return -EINVAL;
		outb(value, FRC);
		break;
	case PWT_GET_FRC:
		value = inb(FRC);
		copy_to_user((void *) arg, &value, sizeof (value));
		break;
	case PWT_SET_VRC:
		copy_from_user(&value, (const void *) arg, sizeof (value));
		if (value > 0x7f)
			return -EINVAL;
		outb(value, VRC);
		break;
	case PWT_GET_VRC:
		value = inb(VRC);
		copy_to_user((void *) arg, &value, sizeof (value));
		break;
	case PWT_SET_GCR:
		copy_from_user(&value, (const void *) arg, sizeof (value));
		if (value > 0x3)
			return -EINVAL;
		outb(value, GCR);
		break;
	case PWT_GET_GCR:
		value = inb(GCR);
		copy_to_user((void *) arg, &value, sizeof (value));
		break;
	default:
		return -EINVAL;
		break;
	}

	return 0;
}

struct file_operations pwt_fops = {
	ioctl:pwt_ioctl,
	open:pwt_open,
	release:pwt_release,
};

int
omap1510_pwt_init(void)
{
	/*
	   Basic initialization of device nodes.
	 */

	printk(KERN_INFO "DSPLinux PWT Control (c) 2001 RidgeRun, Inc\n");

	pwt.device = devfs_register(NULL,	// dir
				    "pwt",	// name
				    DEVFS_FL_DEFAULT,	//flags
				    122,	// major
				    0,	// minor
				    S_IFCHR | S_IRUGO | S_IWUGO,	// mode
				    &pwt_fops,	// ops
				    NULL);	// info (filp->private)
	if (pwt.device < 0) {
		printk(KERN_ERR "pwt_control: can't get major number\n");
		return -EIO;
	}

	orb(1, GCR);		// Enable PWT clock

	return 0;
}

void
omap1510_pwt_exit(void)
{
	devfs_unregister(pwt.device);
}

module_init(omap1510_pwt_init);
module_exit(omap1510_pwt_exit);
