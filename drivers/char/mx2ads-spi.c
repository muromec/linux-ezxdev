/******************************************************************************
	mx2ads-spi.c
	driver for Motorola  MX21 ASP - Touchscreen support

	Copyright (C) 2003 Motorola Semiconductors HK Ltd

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
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/init.h>
#include <asm/bitops.h>
#include <asm/io.h>
#include <linux/errno.h>
#include <linux/tqueue.h>
#include <linux/wait.h>

#include <linux/pm.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pll.h>

#if 1 /*CEE LDM*/
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
	.name = "mx2ads-spi",
	.suspend = __ldm_suspend,
	.resume = __ldm_resume,
};

static struct device __device_ldm = {
	.name = "Touchscreen",
	.bus_id = "spi_ts",
	.driver = &__driver_ldm,
	.power_state = DPM_POWER_ON,
};

#endif

#include "mx2ads-spi.h"

#define MODULE_NAME "touchscreen"

static int debug = 0;
MODULE_PARM(debug, "i");
#define spi_printk( level, args... ) if( (level) <= debug ) pr_debug( args )

#define ENTER() spi_printk( 4, "%s: entered\n", __FUNCTION__ )
#define LEAVE() spi_printk( 4, "%s: leaving\n", __FUNCTION__ )

static void digi_isr(int irq, void *dev_id, struct pt_regs *regs);

static int digi_open(struct inode *inode, struct file *filp);
static int digi_release(struct inode *inode, struct file *filp);
static int digi_fasync(int fd, struct file *filp, int mode);
static int digi_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
		      unsigned long arg);
static ssize_t digi_read(struct file *filp, char *buf, size_t count,
			 loff_t * l);
static unsigned int digi_poll(struct file *filp,
			      struct poll_table_struct *wait);

static int check_device(struct inode *pInode);
static ts_event_t *get_data(void);
static int get_block(ts_event_t * *block, int size);
static void digi_sam_callback(unsigned long data);

static void add_x_y(unsigned x, unsigned y, unsigned flag);
static int init_buf(void);
static void spi_flush_fifo(void);

static int spi_tx_fifo_empty(void);
static int spi_rx_fifo_data_ready(void);
static unsigned int spi_exchange_data(unsigned int dataTx);
static void mx2ts_suspend(void);
static void mx2ts_resume(void);

static int g_digi_major = 0;
static devfs_handle_t g_devfs_handle;
static wait_queue_head_t digi_wait;
struct fasync_struct *ts_fasync;
static u16 rptr, wptr;
static struct timer_list pen_timer;
u8 pen_timer_status;
spinlock_t pen_lock;

struct pm_dev *g_digi_pm;
static int mx2ts_inuse, mx2ts_suspended;

struct file_operations g_digi_fops = {
	.open = digi_open,
	.release = digi_release,
	.read = digi_read,
	.poll = digi_poll,
	.ioctl = digi_ioctl,
	.fasync = digi_fasync,
};

#define NODATA()	(rptr==wptr)
/**
*Misc functions for touch pannel
*/
static void
touch_pan_enable(void)
{
	/*set cspi1_ss0 to be low effective */
	/*cspi1_ss0 --pd28, it connect with pen_cs_b */
	mx2_gpio_set_bit(PORT_D, 28, 0);
}

static void
touch_pan_disable(void)
{
	/*set the cspi1_ss0 to be high */
	mx2_gpio_set_bit(PORT_D, 28, 1);
}

static void
touch_pan_set_inter(void)
{
	/*pe10, uart3_cts pin connect to penIrq */
	mx2_register_gpios(PORT_E, (1 << 10), GPIO | INPUT);
	mx2_gpio_unmask_intr(PORT_E, 10);
	mx2_gpio_set_bit(PORT_E, 10, 0);
	mx2_gpio_config_intr(PORT_E, 10, POSITIVE_EDGE, digi_isr);

}

static void
touch_pan_enable_inter(void)
{
	mx2_gpio_unmask_intr(PORT_E, 10);
}

static void
touch_pan_disable_inter(void)
{
	mx2_gpio_mask_intr(PORT_E, 10);

}

static void
touch_pan_clear_inter(void)
{
	mx2_gpio_clear_intr(PORT_E, 10);
}
#if 0
static void
touch_pan_set_pos_inter(void)
{
	mx2_gpio_config_intr(PORT_E, 10, POSITIVE_EDGE, digi_isr);
}
#endif

/*here set it to be Negative level sensitive */
static void
touch_pan_set_neg_inter(void)
{
	mx2_gpio_config_intr(PORT_E, 10, NEGATIVE_EDGE, digi_isr);
}

#if 0
/*here check if it is positive level sensitive */
static u8
touch_pan_check_int_pol(void)
{
	if (mx2_gpio_get_intr_config(PORT_E, 10) == 1)
		return 0;
	else
		return 1;
}
#endif

void
touch_pan_init(void)
{				/*chip select for ADC */
	/*cspi1_ss0 --pd28, it connect with pen_cs_b */
	mx2_register_gpios(PORT_D, (1 << 28), GPIO | OUTPUT | INIT_DATA_1);
}

/*read data from Touch Pannel */
void
touch_pan_read_dev(u32 * x, u32 * y)
{
	u32 x_upper, x_lower, y_upper, y_lower;

	touch_pan_disable_inter();
	touch_pan_enable();
	x_upper = spi_exchange_data(0xD0);	/* this is dummy data */
	x_upper = spi_exchange_data(0x00);
	x_lower = spi_exchange_data(0x90);
	y_upper = spi_exchange_data(0x00);
	y_lower = spi_exchange_data(0x00);

	*x = (((x_upper << 5) & 0xFFE0) | ((x_lower >> 3) & 0x1F));
	*y = (((y_upper << 5) & 0xFFE0) | ((y_lower >> 3) & 0x1F));
	touch_pan_disable();
	spi_printk(3, "x=%d ,y=%d \n", *x, *y);
	mx2_gpio_clear_intr(PORT_E, 10);

}

void
touch_pan_read_data(u32 * x, u32 * y)
{
#define NUM_OF_SAMPLE  (4)
#define MAX_POS_DIFF   (4)
	u32 xPos[NUM_OF_SAMPLE], yPos[NUM_OF_SAMPLE];

	touch_pan_read_dev(&xPos[0], &yPos[0]);
	if ((xPos[0] < 100) || (xPos[0] > 5000) ||
	    (yPos[0] < 100) || (yPos[0] > 5000)) {
		*x = TPNL_PEN_UPVALUE;
		*y = TPNL_PEN_UPVALUE;
		return;
	}
	touch_pan_read_dev(&xPos[1], &yPos[1]);
	if ((xPos[1] < 100) || (xPos[1] > 5000) ||
	    (yPos[1] < 100) || (yPos[1] > 5000)) {
		*x = TPNL_PEN_UPVALUE;
		*y = TPNL_PEN_UPVALUE;
		return;
	}
	touch_pan_read_dev(&xPos[2], &yPos[2]);
	if ((xPos[2] < 100) || (xPos[2] > 5000) ||
	    (yPos[2] < 100) || (yPos[2] > 5000)) {
		*x = TPNL_PEN_UPVALUE;
		*y = TPNL_PEN_UPVALUE;
		return;
	}
	touch_pan_read_dev(&xPos[3], &yPos[3]);
	if ((xPos[3] < 100) || (xPos[3] > 5000) ||
	    (yPos[3] < 100) || (yPos[3] > 5000)) {
		*x = TPNL_PEN_UPVALUE;
		*y = TPNL_PEN_UPVALUE;
		return;
	}

	*x = (xPos[0] + xPos[1] + xPos[2] + xPos[3]) >> 2;
	*y = (yPos[0] + yPos[1] + yPos[2] + yPos[3]) >> 2;

	return;
}

/**
 * Function Name: digi_isr
 *
 * Input: filp	:
 * 		  wait	:
 * Value Returned:
 *
 * Description: support touch pannel interrupt,since MX1 and MX21 adopted different ADC chip,in MX1,AD7843 will
 *				generate interrupt according to level trigger configuration, so can generate one interrupt to
 *				detect pen down, then reconfigure it to generate pen up interrupt.But in MX21, the AD7873 will
 *				have the pen irq line reassert after each pen sample is taken. It will deassert when you take a
 *				sample. When the pen is down, we disable the interrupt and take a sample every 20ms. Before we
 *				take the sample, we check if the pen irq line is deasserted. If it is deasserted, then this means
 *				the pen is up.
 */
static void
digi_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	/*judge if it is the pen interrupt */

	touch_pan_clear_inter();
	touch_pan_disable_inter();
	pen_timer.expires = jiffies + HZ / 50 + 2 * HZ / 100;
	add_timer(&pen_timer);
	pen_timer_status = 1;
	return;
}

/******************************************************************************
 * Function Name: digi_sam_callback
 *
 * Input: 		:
 * * Value Returned:
 *
 * Description: spi sampling timer call back
 *
 *****************************************************************************/
static void
digi_sam_callback(unsigned long data)
{
	u32 newX, newY;
	static u32 lastX, lastY;
	u32 maxDelay;
	maxDelay = 0;
	newX = TPNL_PEN_UPVALUE;

	if (pen_timer_status != 0)
		mod_timer(&pen_timer, jiffies + HZ / 50 + 2 * HZ / 100);
	if (mx2_gpio_get_bit(PORT_E, 10) != 0) {	/*judge if is pen down */
		if (pen_timer_status != 0) {	/*in use */
			/*get Pen up */
			add_x_y(lastX, lastY, PENUP);
			/* if there is a pen up, shall tell apps to handle the event, and it shall be data there */
			if (!NODATA()) {
				wake_up_interruptible(&digi_wait);
				if (ts_fasync)
					kill_fasync(&ts_fasync, SIGIO, POLL_IN);
			}

			/*release timer */
			spin_lock_irq(&pen_lock);
			pen_timer_status = 0;
			del_timer(&pen_timer);
			spin_unlock_irq(&pen_lock);

			touch_pan_set_neg_inter();
			touch_pan_clear_inter();
			touch_pan_enable_inter();
		}
		return;
	}
	if (mx2_gpio_get_bit(PORT_E, 10) == 0) {	/*positive,it is pen down */
		/*read data with MAX delay from ADC */
		while ((newX == TPNL_PEN_UPVALUE) && (maxDelay <= 100)) {
			touch_pan_read_data(&newX, &newY);
			maxDelay++;
		}
		if (newX == TPNL_PEN_UPVALUE) {
			return;
		}
		/*add element into buffer */
		add_x_y(newX, newY, PENDOWN);
		lastX=newX;
		lastY=newY;

		wake_up_interruptible(&digi_wait);

		if (ts_fasync)
			kill_fasync(&ts_fasync, SIGIO, POLL_IN);
		return;
	} else {
		spi_printk(3, " error to call me \n");
		return;
	}

}

/************************************************
Misc functions for SPI
*************************************************/
void
spi_init(void)
{
	int datarate;
	int cs;
	datarate = 0x8;		//1024
	cs = 0x0;		//ss0
	/*cspi1_mosi--pd31 output */
	/*cspi1_miso--pd30 input */
	/*cspi1_sclk--pd29 output */
	/*cspi1_ss0 --pd28,output it connect with pen_cs_b */

	mx2_register_gpios(PORT_D, (1 << 28) | (1 << 29) | (1 << 31),
			   PRIMARY | OUTPUT);
	mx2_register_gpios(PORT_D, (1 << 30), PRIMARY | INPUT);

#ifdef CONFIG_MX2TO1
	/*reset spi1 */
	CSPI_RESETREG(1) = 0x00000001;
	udelay(200);		//wait
	/* use 32.768kHz */
	CSPI_PERIODREG(1) = 0x00008000;
	/*bit19,18 CS[1:0], bit 17~14 datarate[3:0] */
	CSPI_CONTROLREG(1) |= (cs << 18);
	CSPI_CONTROLREG(1) |= (datarate << 14);
#else
	// newly added bits in TO2 [23:21]
	CSPI_CONTROLREG(1) &= ~0xffc000;
	//bit20,19 CS[1:0], bit 18~14 datarate[4:0]
	CSPI_CONTROLREG(1) |= (cs<<19);
	datarate=18;
	CSPI_CONTROLREG(1) |= (datarate<<14);
#endif
	CSPI_CONTROLREG(1) |= 0x00000c07;	/*ss low active, 8 bit transfer,master mode */

}

#if 0
/*To polling when Spi transfer is not finished */
static void
spi_poll_done(void)
{
	/*here should poll the IRQ bit in IRQ reg to see */
	while (!(CSPI_INTREG(1) & SPI_TE_INT_BIT)) ;	/*transfer FIFO is empty */
	/* Reset SPI IRQ bit */
	CSPI_INTREG(1) &= SPI_TE_INT_BIT;
	/*wait until XCH bit is clear, hence exchange is complete */
	while (!(CSPI_CONTROLREG(1) & SPI_XCH_BIT)) ;
}

static void
spi_tx_data(u16 data)
{
	/* ensure data will not be transmitted while writing to SPI */

	CSPI_CONTROLREG(1) &= SPI_XCH_MASK;
	CSPI_CONTROLREG(1) |= SPI_EN_BIT;
	CSPI_TXDATAREG(1) = (u32) data;	/*transfer data */
	CSPI_CONTROLREG(1) |= SPI_XCH_BIT;
	spi_poll_done();
	CSPI_CONTROLREG(1) &= SPI_XCH_MASK;
}
#endif

static int
spi_tx_fifo_empty(void)
{
	return (CSPI_INTREG(1) & SPI_TE_INT_BIT);
}

static int
spi_rx_fifo_data_ready(void)
{
	return (CSPI_INTREG(1) & SPI_RR_INT_BIT);
}

static unsigned int
spi_exchange_data(unsigned int dataTx)
{
	while (!spi_tx_fifo_empty()) ;

	CSPI_TXDATAREG(1) = dataTx;	/*transfer data */
	CSPI_CONTROLREG(1) |= SPI_XCH_BIT;	/* exchange data */

	while (!spi_rx_fifo_data_ready()) ;

	return CSPI_RXDATAREG(1);
}

/******************************************************************************
 * Function Name: check_device
 *
 * Input: 		inode	:
 * Value Returned:	int	: Return status.If no error, return 0.
 *
 * Description: verify if the inode is a correct inode
 *
 *****************************************************************************/
static int
check_device(struct inode *pInode)
{
	int minor;
	kdev_t dev = pInode->i_rdev;

	if (MAJOR(dev) != g_digi_major) {
		return -1;
	}
	minor = MINOR(dev);

	if (minor < MAX_ID)
		return minor;
	else {
		return -1;
	}
}

/**************************************************
Misc functions for buffer
***************************************************/
/* local buffer for store data
 * a new feature is read block, assume the buffer is a continuous buffer
 * so, if there is enough data, read data from current position to
 * buffer end, then next time, from buffer head to data end. -- the buffer
 * is a loop buffer.
 */
ts_event_t *buffer;
#define BUFLEN		250
#define BUFSIZE 	(BUFLEN*sizeof(ts_event_t))
#define NEXTI(i)	{i=(i==BUFLEN-1)?0:i+1;}
#define GETNEXTI(i)	((i==BUFLEN-1)?0:i+1)
#define BLKEND(i)	((i)==0)

#define DSIZE()		((wptr<rptr)?(BUFLEN-rptr):(wptr-rptr))

void
spi_flush_fifo(void)
{
	u32 i, j;
	for (i = 0; i < 8; i++) {
		if (CSPI_INTREG(1) & SPI_RR_INT_BIT)
			j = CSPI_RXDATAREG(1);
	}
}

/******************************************************************************
 * Function Name: init_buf
 *
 * Input: 		void	:
 * Value Returned:	int	: Return status.If no error, return 0.
 *
 * Description: init the loop buffer for store data read out from spi FIFO
 * 	and shall be copied to user
 *
 *****************************************************************************/
static int
init_buf(void)
{
	buffer = (ts_event_t *) vmalloc(BUFSIZE);
	if (!buffer) {
		return -1;
	}

	rptr = wptr = 0;
	return 1;
}

/******************************************************************************
 * Function Name: add_x_y
 *
 * Input: 		i	:
 * Value Returned:	void	:
 *
 * Description: add pen data to buffer
 *
 *****************************************************************************/
static void
add_x_y(unsigned x, unsigned y, unsigned flag)
{

	if (GETNEXTI(wptr) == rptr)
		return;
	buffer[wptr].x = x;
	buffer[wptr].y = y;

	buffer[wptr].pressure = flag;

	NEXTI(wptr);		/* goto next wptr */
}

/******************************************************************************
 * Function Name: get_block
 *
 * Input: 		block:
				size:
 * Value Returned:	int	: count of data size
 *
 * Description: read a block of 'touch' data from buffer, read count shall less
 *	than size,but shall be as more as possible. the data shall not really copy
 *  to upper layer,untill copy_to_user is invoked.
 *****************************************************************************/
/*assume continuous buffer */
static int
get_block(ts_event_t * *block, int size)
{
	int cnt, rd;
	unsigned long flags;
	ts_event_t *p;
	if (NODATA())
		return 0;

	cnt = 0;

	save_flags(flags);
	cli();
	/*critical section */
	/*get the actual size of data need read */
	if (DSIZE() * sizeof (ts_event_t) >= size) {
		rd = size / sizeof (ts_event_t);
	} else {
		rd = DSIZE();
	}

	*block = p = get_data();
	cnt++;

	while (p && (cnt < rd)) {
		if (rptr == 0)
			break;
		p = get_data();
		cnt++;
	}

	restore_flags(flags);
	return (cnt) * sizeof (ts_event_t);

}

/******************************************************************************
 * Function Name: get_data
 *
 * Input: 		void	:
 * Value Returned:	ts_event_t	: a 'touch' event data format
 *
 * Description: read a 'touch' event from data buffer
 *
 *****************************************************************************/
/* no really read out the data, so no copy */
static ts_event_t *
get_data(void)
{
	ts_event_t *data;

	if (NODATA())
		return NULL;

	data = &(buffer[rptr]);

	spi_printk(3, "*** Read - x: %d, y: %d, rptr: 0x%04x, wptr: 0x%04x\n",
		   data->x, data->y, (int) rptr, (int) wptr);

	NEXTI(rptr);
	return data;
}

/******************************************************************************
 * Function Name: digi_release
 *
 * Input: 		inode	:
 * 			filp	:
 * Value Returned:	int	: Return status.If no error, return 0.
 *
 * Description: release resource when close the inode
 *
 *****************************************************************************/
int
digi_release(struct inode *inode, struct file *filp)
{

	digi_fasync(-1, filp, 0);
	if (!mx2ts_suspended) {
		mx2ts_suspend();
		mx2ts_suspended = 0; /*to reuse the  mx2ts_suspend function*/
	}

	MOD_DEC_USE_COUNT;
	mx2ts_inuse--;

	return 0;
}

/******************************************************************************
 * Function Name: digi_open
 *
 * Input: 		inode	:
 * 			filp	:
 * Value Returned:	int	: Return status.If no error, return 0.
 *
 * Description: allocate resource when open the inode
 *
 *****************************************************************************/
int
digi_open(struct inode *inode, struct file *filp)
{
	if (mx2ts_suspended) return -EPERM;
	if (mx2ts_inuse) return -EBUSY;
	mx2ts_inuse++;

	MOD_INC_USE_COUNT;
        mx2ts_suspended = 1; /*to reuse the  mx2ts_resume function*/
	mx2ts_resume();
	return 0;
}

/******************************************************************************
 * Function Name: digi_fasync
 *
 * Input: 		fd	:
 * 			filp	:
 * 			mode	:
 * Value Returned:	int	: Return status.If no error, return 0.
 *
 * Description: provide fasync functionality for select system call
 *
 *****************************************************************************/
static int
digi_fasync(int fd, struct file *filp, int mode)
{
	if (mx2ts_suspended) return -EPERM;
	/* TODO TODO put this data into file private data */
	int minor = check_device(filp->f_dentry->d_inode);
	if (minor == -1) {
		return -ENODEV;
	}
	return (fasync_helper(fd, filp, mode, &ts_fasync));
}

/******************************************************************************
 * Function Name: digi_ioctl
 *
 * Input: 		inode	:
 * 			filp	:
 * 			cmd	: command for ioctl
 * 			arg	: parameter for command
 * Value Returned:	int	: Return status.If no error, return 0.
 *
 * Description: ioctl for this device driver
 *
 *****************************************************************************/
int
digi_ioctl(struct inode *inode,
	   struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = -EIO;
	int minor;
	if (mx2ts_suspended) return -EPERM;

	minor = check_device(inode);
	if (minor == -1) {
		spi_printk(3, "bad minor\n");
		return -ENODEV;
	}

	spi_printk(3, "minor=%08x cmd=%d\n", minor, cmd);

	return ret;
}

/******************************************************************************
 * Function Name: spi_poll
 *
 * Input: 		filp	:
 * 			wait	:
 * Value Returned:	int	: Return status.If no error, return 0.
 *
 * Description: support poll and select
 *
 *****************************************************************************/
unsigned int
digi_poll(struct file *filp, struct poll_table_struct *wait)
{
	int minor;
	if (mx2ts_suspended) return -EPERM;

	minor = check_device(filp->f_dentry->d_inode);
	if (minor == -1)
		return -ENODEV;

	poll_wait(filp, &digi_wait, wait);

	return (NODATA())? 0 : (POLLIN | POLLRDNORM);
}

/******************************************************************************
 * Function Name: digi_read
 *
 * Input: 		filp	: the file
 * 			buf	: data buffer
 * 			count	: number of chars to be readed
 * 			l	: offset of file
 * Value Returned:	int	: Return status.If no error, return 0.
 *
 * Description: read device driver
 *
 *****************************************************************************/
ssize_t
digi_read(struct file * filp, char *buf, size_t count, loff_t * l)
{
	int nonBlocking = filp->f_flags & O_NONBLOCK;
	int minor;
	ts_event_t *ev;
	int cnt = 0;

	if (mx2ts_suspended) return -EPERM;

	minor = check_device(filp->f_dentry->d_inode);

	if (minor == -1)
		return -ENODEV;

	if (nonBlocking) {
		if (!NODATA()) {
			/* returns length to be copied otherwise errno -Exxx */

			cnt = get_block(&ev, count);
			if (cnt > count) {
				printk(KERN_ERR "Error read spi buffer\n");
				return -EINVAL;
			}

			__copy_to_user(buf, (char *) ev, cnt);
			return cnt;
		} else
			return -EINVAL;
	} else {

		/* check , when woken, there is a complete event to read */
		while (1) {
			if (!NODATA()) {

				cnt = get_block(&ev, count);
				if (cnt > count) {
					printk(KERN_ERR
					       "Error read spi buffer\n");
					return -EINVAL;
				}
				/* returns length to be copied otherwise errno -Exxx */

				__copy_to_user(buf, (char *) ev, cnt);
				return cnt;
			} else {
				interruptible_sleep_on(&digi_wait);
				if (signal_pending(current))
					return -ERESTARTSYS;
			}
		}
	}
}

static void
mx2ts_suspend(void)
{

	if (mx2ts_inuse && !mx2ts_suspended) {
		/*disable the ADC interrupt */
		/*disable_pen_interrupt(); */

		touch_pan_disable_inter();
		mx_module_clk_close(IPG_MODULE_CSPI1);
	}
	mx2ts_suspended = 1;

}

static void
mx2ts_resume(void)
{
	if (mx2ts_inuse && mx2ts_suspended) {
		mx_module_clk_open(IPG_MODULE_CSPI1);

		spi_flush_fifo();
		/*enable ADC interrupt */
		touch_pan_disable_inter();
		touch_pan_set_neg_inter();	/*set default polarity */
		touch_pan_clear_inter();
		touch_pan_enable_inter();

		pen_timer_status = 0;
	}
	mx2ts_suspended = 0;
}


/**
* power management callback function, will disable the clk from PCCR0 register
*/
int
digi_pm_handler(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	switch (rqst) {
	case PM_RESUME:
		mx2ts_resume();
		break;
	case PM_SUSPEND:
		mx2ts_suspend();
		break;
	default:
		break;
	}
	return 0;
}
#if 1 /*CEE LDM*/
static int
__ldm_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		mx2ts_suspend();
		break;
	}
	return 0;
}

static int
__ldm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		mx2ts_resume();
		break;
	}
	return 0;
}
#endif

/*******************************************************
*digitizer init function
*Parameters:None
*Return	
*	0	indicates SUCCESS
* 	-1	indicates FAILURE
*Description: in this function should initialize the SPI device
as well as the external touch pannel hardware
********************************************************/
signed int __init
digi_init(void)
{
	int i;
	spi_printk(3, "Pen Driver 0.4.0\n");
	spi_printk(3, "Motorola SPS-Suzhou\n");
	init_timer(&pen_timer);
	pen_timer.function = digi_sam_callback;

	g_digi_major = devfs_register_chrdev(0, MODULE_NAME, &g_digi_fops);
	if (g_digi_major < 0) {
		printk("%s driver: Unable to register driver\n", MODULE_NAME);
		return -ENODEV;
	}

	g_devfs_handle = devfs_register(NULL, MODULE_NAME, DEVFS_FL_DEFAULT,
					g_digi_major, 0,
					S_IFCHR | S_IRUSR | S_IWUSR,
					&g_digi_fops, NULL);

	/*init SPI buffer */
	if (-1 == init_buf()) {
		printk("digi_init: cannot init spi buffer, exit\n");

		devfs_unregister_chrdev(g_digi_major, MODULE_NAME);
		devfs_unregister(g_devfs_handle);
		return -1;
	}

	init_waitqueue_head(&digi_wait);

	mx_module_clk_open(IPG_MODULE_CSPI1);

	/*first init touch pannel */
	touch_pan_set_inter();	/*set pen IRQ */
	touch_pan_disable_inter();	/*mask pen interrupt */
	/*init pen touch pannel in GPIO,disabled */
	/*init SPI module */
	spi_init();
	touch_pan_init();

	touch_pan_enable();	/*enable touch pannel */

	spi_exchange_data(0x90);
	spi_exchange_data(0x00);
	spi_exchange_data(0xD0);
	spi_exchange_data(0x00);
	spi_exchange_data(0x00);
	touch_pan_disable();

	/* Delay for A/D Converter to be stable */
	for (i = 0; i < 50000; i++) ;
	mx_module_clk_close(IPG_MODULE_CSPI1);

	g_digi_pm = pm_register(PM_SYS_DEV, PM_SYS_VGA, digi_pm_handler);
#if 1 /*CEE LDM*/
	mx21_ldm_bus_register(&__device_ldm, &__driver_ldm);
#endif
	return 0;
}

void __exit
digi_cleanup(void)
{
	/*Do some cleanup work */

	if (g_digi_major > 0) {
		devfs_unregister_chrdev(g_digi_major, MODULE_NAME);
		devfs_unregister(g_devfs_handle);
	}
	pm_unregister(g_digi_pm);
#if 1 /*CEE LDM*/
	mx21_ldm_bus_unregister(&__device_ldm, &__driver_ldm);
#endif
	mx_module_clk_close(IPG_MODULE_CSPI1);
}

module_init(digi_init);
module_exit(digi_cleanup);

MODULE_LICENSE("GPL");
