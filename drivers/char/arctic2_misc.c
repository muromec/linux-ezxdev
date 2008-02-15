/* IBM Arctic-2 miscellaneous device support
 *
 * Char driver for /dev/arcmsc, which handles the on-case buttons,
 * frontlight control, programmable LED and dock/undock detection (so
 * far)
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (C) 2002 K. Iwasaki, Ken Inoue, David Gibson IBM Corporation.
 *
 * Ken Inoue
 * IBM Thomas J. Watson Research Center
 * <keninoue@us.ibm.com>
 *
 * David Gibson
 * IBM OzLabs, Canberra, Australia.
 * <arctic@gibson.dropbear.id.au>
 */
#include <linux/module.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/signal.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <asm/types.h>
#include <linux/types.h>
#include <linux/wrapper.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/i2c.h>

#include <asm/time.h>
#include <asm/ppc4xx_pic.h>
#include <asm/ocp.h>
#include <platforms/arctic2.h>

#include <linux/arctic2_misc.h>

#define ARCTIC2_MISC_MINOR	117

#define ARCTIC_FPGA_INT		27
#define ARCTIC_FPGA_INT_MASK	(0x80000000 >> ARCTIC_FPGA_INT)

#define ARCTIC2_FPGA_IRQS	(ARCTIC2_FPGA_IRQ_PWR | ARCTIC2_FPGA_IRQ_MIC \
				| ARCTIC2_FPGA_IRQ_BTN)

static struct i2c_adapter *arctic_i2c;

#define DBG(x)

/* Simple wrappers for i2c_transfer() - the ones in the I2C layer
 * (i2c_master_send() etc.) impose a stupid structure on the
 * driver. */
static int i2c_write(u16 addr, const char *buf, int len)
{
	struct i2c_msg msg = {
		.addr 	= addr,
		.flags 	= 0,
		.len	= len,
		.buf	= (char *)buf,
	};

	/* Our driver might be initialized before the IIC driver is,
	 * so we lazily find the I2C adapter */
	if (! arctic_i2c) {
		struct ocp_dev *ocp_iic;

		ocp_iic = ocp_get_dev(IIC, 0);
		if (!ocp_iic)
			return -EBUSY;

		arctic_i2c = (struct i2c_adapter *)ocp_iic->ocpdev;
	}

	return i2c_transfer(arctic_i2c, &msg, 1);
}

static int i2c_writeread(u16 addr, const char *wbuf, size_t wlen,
			 char *rbuf, size_t rlen)
{
	struct i2c_msg msg[2] = {
		{ .addr		= addr,
		  .flags	= 0,
		  .len		= wlen,
		  .buf		= (char *)wbuf,
		},
		{ .addr		= addr,
		  .flags	= I2C_M_RD,
		  .len		= rlen,
		  .buf		= rbuf
		},
	};

	/* Our driver might be initialized before the IIC driver is,
	 * so we lazily find the I2C adapter */
	if (! arctic_i2c) {
		struct ocp_dev *ocp_iic;

		ocp_iic = ocp_get_dev(IIC, 0);
		if (!ocp_iic)
			return -EBUSY;

		arctic_i2c = (struct i2c_adapter *)ocp_iic->ocpdev;
	}

	return i2c_transfer(arctic_i2c, msg, 2);
}

/************************************************************************/
/* Button handling code							*/
/************************************************************************/

#define ARCTIC_BUTTON_BUF_SIZE	256
#define BTN_DELAY  		(HZ/10) /* poll 10 times per second */

struct arctic2_button_state {
	struct fasync_struct *fasync;
	wait_queue_head_t wq;

	/* Last known values (used for polling) */
	u8 btn, mway;

	struct timer_list timer;

	spinlock_t qlock;
	unsigned long qhead;
	unsigned char buf[ARCTIC_BUTTON_BUF_SIZE];
};

static struct arctic2_button_state btn_state;

static inline int queue_empty(loff_t off)
{
	return (off >= btn_state.qhead);
}

static void put_in_queue(u8 data)
{
	unsigned long flags;
	unsigned long pos;

	spin_lock_irqsave(&btn_state.qlock, flags);

	pos = btn_state.qhead % ARCTIC_BUTTON_BUF_SIZE;

	btn_state.buf[pos] = data;
	btn_state.qhead++;

	kill_fasync(&btn_state.fasync, SIGIO, POLL_IN);
	wake_up_interruptible(&btn_state.wq);
		
	spin_unlock_irqrestore(&btn_state.qlock, flags);
}

static int get_from_queue(loff_t *tail, unsigned char *data)
{
	unsigned long flags;
	int ret = 1;

	spin_lock_irqsave(&btn_state.qlock, flags);

	/* If the reader didn't keep up, tough, they lose events */
	if (*tail + ARCTIC_BUTTON_BUF_SIZE < btn_state.qhead)
		*tail = btn_state.qhead - ARCTIC_BUTTON_BUF_SIZE;

	if (*tail < btn_state.qhead) { /* non-empty */
		*data = btn_state.buf[*tail % ARCTIC_BUTTON_BUF_SIZE];
		(*tail)++;
	} else {
		ret = 0;
	}
		
	spin_unlock_irqrestore(&btn_state.qlock, flags);

	return ret;
}

#ifndef CONFIG_405LP_PM_BUTTON
static void do_powerbutton(void)
{
	/* Power off button pressed */
	int i;
	
	/* FIXME: we can't turn off while the power button is still
	 * depressed - the hardware will just switch us on agan.  It
	 * would be nice to wait for the release to do the powerdown,
	 * but apparently jitter makes this difficult.  Instead we
	 * just put a 3s delay between blanking the display and
	 * turning off the unit, and hope the user releases the button
	 * in the meantime */
	
	/* Change display active area to just 1 pixel */ 
	mtdcr(DCRN_LCD0_CFGADDR, DCRN_LCD0_ADSR);
	mtdcr(DCRN_LCD0_CFGDATA, 0x00010001);
	/* Then wait for at least 1 screen refresh cycle */ 
	for (i = 0; i < 1000; i++) {
		udelay(1000); 
	} /* 1s should be enough... */ 
	
	/* Disable LCD */  
	mtdcr(DCRN_LCD0_CFGADDR, DCRN_LCD0_DER); 
	mtdcr(DCRN_LCD0_CFGDATA, 0); 
	udelay(100);
	/* Power off LCD */
	arctic2_set_lcdpower(0);
	
	/* FIXME: do we need this stuff...? */
	asm volatile("sync");
	
	for (i = 0; i < 2000; i++) {
		udelay(1000); /* 1 ms */ 
	} /* 2 sec delay */
	
	asm volatile("sync");
	
	arctic2_poweroff(); /* never returns */
}
#endif /* CONFIG_405LP_PM_BUTTON */

static void poll_buttons(unsigned long data __attribute__((unused)))
{
	u8 btn, mway;

	btn = readb(ARCTIC2_FPGA_BUTTONS) & 0x3F;
	mway = readb(ARCTIC2_FPGA_MULTIWAY) & 0x1F;

	if (btn || mway) {
		/* At least one button is pressed */
#ifndef CONFIG_405LP_PM_BUTTON 
		if ((btn & ~btn_state.btn) & ARCTIC2_FPGA_BTN_PWR)
			do_powerbutton(); /* never returns */
#endif /* CONFIG_405LP_PM_BUTTON */
		
		if ((btn & ~btn_state.btn) & 0x01)
			put_in_queue(0x03);
		else if ((btn & ~btn_state.btn) & 0x02)
			put_in_queue(0x01);
		else if ((btn & ~btn_state.btn) & 0x04)
			put_in_queue(0x02);
		else if ((btn & ~btn_state.btn) & 0x08)
			put_in_queue(0x04);
		else if ((btn & ~btn_state.btn) & ARCTIC2_FPGA_BTN_MIC)
			put_in_queue(0x05);

		if ((mway & ~btn_state.mway)
		    & ARCTIC2_FPGA_MULTIWAY_PUSH) {
			/* We prioritize push, so it gets handled in
			 * preference to any jiggling of the lever
			 * that happens at the same time */
			put_in_queue(0x06);
		} else {
			/* We translate the odd 45deg rotated switches
			 * into the cardinal directions.  Have to
			 * handle the masking of the old value a bit
			 * differently for that. */
			if ((mway == ARCTIC2_FPGA_MULTIWAY_N)
			    && !(btn_state.mway == ARCTIC2_FPGA_MULTIWAY_N))
				put_in_queue(0x07);
			if ((mway == ARCTIC2_FPGA_MULTIWAY_E)
			    && !(btn_state.mway == ARCTIC2_FPGA_MULTIWAY_E))
				put_in_queue(0x08);
			if ((mway == ARCTIC2_FPGA_MULTIWAY_S)
			    && !(btn_state.mway == ARCTIC2_FPGA_MULTIWAY_S))
				put_in_queue(0x09);
			if ((mway == ARCTIC2_FPGA_MULTIWAY_W)
			    && !(btn_state.mway == ARCTIC2_FPGA_MULTIWAY_W))
				put_in_queue(0x0a);
		}
		
		/* While a button is pressed, the FPGA continues to
		 * assert the interrupt line, so the only way to look
		 * for more presses (or releases) is to poll :-( */
		writeb(0, ARCTIC2_FPGA_IRQ_ENABLE);
		btn_state.timer.expires = jiffies + BTN_DELAY;
		add_timer(&btn_state.timer);
	} else {
		/* Nothing pressed any more, re-enable the interrupt
		 * and stop polling */
		writeb(ARCTIC2_FPGA_IRQS, ARCTIC2_FPGA_IRQ_ENABLE);
	}

	btn_state.btn = btn;
	btn_state.mway = btn;
}

static void arctic_button_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	poll_buttons(0);
	/* TODO: TCPA (assume shared interrupt)  */
}

static int arctic_button_fasync(int fd, struct file *filp, int on)
{
	int ret;

	ret = fasync_helper(fd, filp, on, &btn_state.fasync);
	if (ret < 0)
		return ret;
	return 0;
}

static ssize_t arctic_button_read(struct file *filp, char *buf, size_t count,
				  loff_t *pos)
{
	ssize_t bytes_read = 0;
	int ret;

	if (queue_empty(filp->f_pos)) {	/* wait for an event */
		if (filp->f_flags & O_NONBLOCK)
			return -EWOULDBLOCK;

		ret = wait_event_interruptible(btn_state.wq,
					       !queue_empty(filp->f_pos));
		if (ret)
			return ret;
	}

	while (bytes_read < count) {
		u8 c;

		if (! get_from_queue(&filp->f_pos, &c))
			break;

		if (put_user(c, buf + bytes_read))
			return -EFAULT;

		bytes_read++;
	}

	return bytes_read;
}

static unsigned int arctic_button_poll(struct file *filp, poll_table *wait)
{
	poll_wait(filp, &btn_state.wq, wait);
	if (!queue_empty(filp->f_pos))
		return POLLIN | POLLRDNORM;
	return 0;
}

/************************************************************************/
/* Frontlight handling code						*/
/************************************************************************/

#define	FRONTLIGHT_I2C_ADDR	0x28

static spinlock_t arctic2_fl_lock = SPIN_LOCK_UNLOCKED;
static u8 arctic2_frontlight_status = 0x20; /* Maximum brightness */

static int set_frontlight(u8 val)
{
	int ret;

	spin_lock_irq(&arctic2_fl_lock);

	if ( (val > 3) && (arctic2_frontlight_status < 4) ) {
		/* Turn on the LCD */
		mtdcri(DCRN_LCD0, DER, 0);
		udelay(1000); /* 1ms delay */ 
	}

	ret = i2c_write(FRONTLIGHT_I2C_ADDR, &val, 1);
	if (ret >= 0)
		ret = 0;

	if ( (val > 3) && (arctic2_frontlight_status < 4) ) {
		udelay(1000); /* 1ms delay */ 
		mtdcri(DCRN_LCD0, DER, 1);
	}

	arctic2_frontlight_status = val;

	spin_unlock_irq(&arctic2_fl_lock);

	return ret;
}

static ssize_t arctic2_frontlight_write(struct file *filp, const char *buf,
					size_t count, loff_t *ppos)
{
	u8 data;
	int err;

	if (count == 0)
		return 0; 

	if (get_user(data, buf))
		return -EFAULT;

	/* clamp */
	if (data > 0x1f)
		data =  0x1f; 

	err = set_frontlight(data);
	if (err)
		return err;

	return 1; /* always process 1 byte at a time */
}

/************************************************************************/
/* Programmable LED control						*/
/************************************************************************/

#define LED_CTRL_ADDR           0x0D
#define LED_RED_ADDR            0x2D
#define LED_GREEN_ADDR          0x4D
#define LED_BLUE_ADDR           0x6D

static void write_led_reg(u8 high, u8 low, u8 addr)
{
        writeb(high, ARCTIC2_FPGA_LED_DATA_HI);
        writeb(low, ARCTIC2_FPGA_LED_DATA_LOW);
        writeb(addr, ARCTIC2_FPGA_LED_ADDR);
        writeb(0x01, ARCTIC2_FPGA_LED_CTRL);
        /* FIXME: should we wait for the sending bit to clear? */
}

/************************************************************************/
/* Battery fuel guage							*/
/************************************************************************/

#define FUELGAUGE_I2C_ADDR	0x48

static int read_battery_voltage(void)
{
	int ret;
	u8 zero = 0x00; /* command byte to write to the I2C bus */
	u16 buf[4];

	memset(buf, 0, sizeof(buf));

	/* The ADS7823 makes and returns 4 samples */
	ret = i2c_writeread(FUELGAUGE_I2C_ADDR, &zero, 1,
			    (char *)buf, sizeof(buf));
	if (ret < 0)
		return ret;

	printk(KERN_DEBUG "arctic2_misc: (%d) fuelguage returned: %hx/%hx/%hx/%hx\n",
	       ret,
	       be16_to_cpu(buf[0]), be16_to_cpu(buf[1]),
	       be16_to_cpu(buf[2]), be16_to_cpu(buf[3]));
	
	return be16_to_cpu(buf[0]);
}

/************************************************************************/
/* Common device code							*/
/************************************************************************/

static int arctic2_misc_open(struct inode *inode, struct file *filp)
{
	filp->f_pos = 0;
	return 0;
}

static int arctic2_misc_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static int arctic_misc_ioctl(struct inode *inode, struct file *filp,
			  unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	/* So far all ioctl()s require write permission to the file */
	if (! (filp->f_mode & FMODE_WRITE))
		return -EPERM;

	switch (cmd) {
		/* frontlight ioctl()s */
	case 0: /* backwards compatibility with old versions */
	case ARCTIC2_MISC_FLSTATUS:
		ret = mfdcri(DCRN_LCD0, DER) << 8;
		ret |= arctic2_frontlight_status;
		break;

		/* power ioctls()s */
	case 1: /* backwards compatibility with old versions */
	case ARCTIC2_MISC_POWEROFF:
		arctic2_poweroff();
		break;

	case ARCTIC2_MISC_PWRSTATUS: {
		int voltage = read_battery_voltage();
		u32 status;

		status = voltage & 0xfff;

		if (put_user(status, (u32 *)arg))
			ret = -EFAULT;

		break;
	}

		/* multicolor LED ioctl()s */
        case ARCTIC2_MISC_LEDSTANDBY: {
		int tmp;
		u8 reg;

                if (get_user(tmp, (int *)arg))
                        return -EFAULT;
		
                if ( (tmp != 0) && (tmp != 1) )
                        return -EINVAL;
		
                reg = tmp << 1;
                writeb(reg, ARCTIC2_FPGA_LED_CTRL);
                break;
	}

        case ARCTIC2_MISC_LEDCTRL: {
		u32 tmp;

                if (get_user(tmp, (u32 *)arg)) {
			ret = -EFAULT;
			break;
		}

                if (tmp & ~0xff) {
			ret = -EINVAL; /* overkill? */
			break;
		}

                write_led_reg((tmp >> 8) & 0xff, tmp & 0xff, LED_CTRL_ADDR);
                break;
	}

        case ARCTIC2_MISC_LEDTIMING: {
		u32 tmp;
		u8 on, off, color;

                if (get_user(tmp, (u32 *)arg)) {
                        ret = -EFAULT;
			break;
		}

                if (tmp & ~0xff007f7f) {
			ret = -EINVAL; /* overkill? */
			break;
		}

                color = (tmp & ARCTIC2_LED_TIMING_COLOR) >> 24;
                on = (tmp & ARCTIC2_LED_TIMING_ON) >> 8;
                off = (tmp & ARCTIC2_LED_TIMING_OFF);

                if ( (color > 2) || (on > 127) || (off > 127) || (on >= off) )
                        return -EINVAL; /* overkill? */

                write_led_reg(on, off, LED_CTRL_ADDR + ((color+1) << 5));
                break;
	}

	default:
		ret = -ENOIOCTLCMD;
	}

	return ret;
}

static struct file_operations arctic_misc_fops = {
	.open		= arctic2_misc_open,
	.release	= arctic2_misc_close,
	.read		= arctic_button_read,
	.write		= arctic2_frontlight_write,
	.poll		= arctic_button_poll,
	.ioctl		= arctic_misc_ioctl,
	.fasync		= arctic_button_fasync,
};

static struct miscdevice arctic_button_miscdev = {
	ARCTIC2_MISC_MINOR, "arcmsc", &arctic_misc_fops
};

MODULE_AUTHOR("K.Iwasaki, K. Inoue, David Gibson");
MODULE_DESCRIPTION("Arctic-2 miscellaneous device driver");

int __init init_arctic2_misc(void)
{
	int irq_requested = 0;
	int err;

	/* we need to setup all the same registers bios should have */
	/* already setup for us because there is a chance that the */
	/* user could have a different bios that isn't just right... */

	/* set IRQ to level-triggered, high */
	mtdcr(DCRN_UIC0_TR, mfdcr(DCRN_UIC0_TR) & ~ARCTIC_FPGA_INT_MASK);
	mtdcr(DCRN_UIC0_PR, mfdcr(DCRN_UIC0_PR) | ARCTIC_FPGA_INT_MASK);
	mtdcr(DCRN_UIC0_SR, ARCTIC_FPGA_INT_MASK);

	memset(&btn_state, 0, sizeof(btn_state));
	/* don't forget to init the waitqueue or else we oops on read */
	init_waitqueue_head(&btn_state.wq);
	spin_lock_init(&btn_state.qlock);

	init_timer(&btn_state.timer);
	btn_state.timer.function = poll_buttons;

	err = request_irq(ARCTIC_FPGA_INT, arctic_button_irq, SA_SHIRQ,
			  "arctic2_misc", NULL);
	if (err) {
		printk(KERN_ERR "arctic2_misc: Couldn't register IRQ (%d)\n",
		       err);
		goto fail;
	}
	irq_requested = 1;

	writeb(ARCTIC2_FPGA_IRQS, ARCTIC2_FPGA_IRQ_ENABLE);

	/* register under major 10, minor specified in arctic_button_miscdev */
	err = misc_register(&arctic_button_miscdev);
	if (err) {
		printk(KERN_ERR "arctic2_misc: Couldn't register misc dev (%d)\n",
		       err);
		goto fail;
	}

	return 0;
 fail:
	misc_deregister(&arctic_button_miscdev); /* fails safely if
						  * not registered */
	if (irq_requested)
		free_irq(ARCTIC_FPGA_INT, NULL);
	return err;
}

void cleanup_arctic2_misc(void)
{
	misc_deregister(&arctic_button_miscdev);
	free_irq(ARCTIC_FPGA_INT, NULL);
}

module_init(init_arctic2_misc);
module_exit(cleanup_arctic2_misc);
