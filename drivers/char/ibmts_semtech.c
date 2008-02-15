/* IBM Arctic-2 Semtech touch panel controller driver for /dev/ibmts interface
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
 *  Copyright (C) 2002 IBM Corporation.
 *
 * Ken Inoue 
 * IBM Thomas J. Watson Research Center
 * <keninoue@us.ibm.com>
 *
 * David Gibson
 * IBM OzLabs, Canberra, Australia.
 * <arctic@gibson.dropbear.id.au>
 *
 * John F. Davis
 * IBM RTP
 * <johndavi@us.ibm.com>
 */

#include <linux/module.h>      /* Get my module mojo */
#include <linux/types.h>       /* int16_t ... */
#include <linux/poll.h>        /* polling macros */
#include <linux/interrupt.h>   /* Tasklets */
#include <linux/sched.h>       /* wait_on, etc */
#include <linux/wait.h>        /* wait queues */
#include <linux/init.h>        /* module_init() macro */
#include <linux/spinlock.h>    /* mutex structures/code */
#include <linux/fs.h>          /* struct file, struct inode */
#include <linux/vmalloc.h>
#include <linux/serial_reg.h>
#include <linux/proc_fs.h>
#include <asm/ibm4xx.h>
#include <asm/io.h>

#define IBMTS_DEVICE_NAME	"ibmts"
#define IBMTS_MAJOR		254

#define IBMTS_SAMPLES 128
#define IBMTS_TIMEOUT_INTERVAL 10

#define PEN_RELEASE 0
#define PEN_GLIDE   2
#define PEN_PRESS   1

#define DEFAULT_SAMPLE_MASK 0xFFFFFFFE  /* can be configured via proc file at runtime. */
static int sample_mask = DEFAULT_SAMPLE_MASK;

struct ts_event {
	short pressure;     /* Pressure of stylus  - 0 for release, positive otherwise */
	int x;              /* X coordinate stylus - -1 for PEN_RELEASE */
	int y;              /* Y coordinate stylus - -1 for PEN_RELEASE */
	int millisecs; 		/* A timestamp */
	unsigned int flags; /* Not used for now. */
};

typedef struct ibmts_semtech_event {
	struct ibmts_semtech_event *pNext;
	struct ts_event event;
} ibmts_semtech_event;

// HACK: FIXME: put the following in a dynamically allocated struct  

static int IsOpenForRead = 0;

static unsigned char pen_down = 0;

static wait_queue_head_t ibmts_semtech_proc_list;

static spinlock_t ibmts_semtech_lock = SPIN_LOCK_UNLOCKED;

#define SEMTECH_FSM_RESET 0
#define SEMTECH_FSM_DATA_READY 4

#define SEMTECH_SYNC_FLAG 0x80
#define SEMTECH_ID 0x81 

static unsigned char semtech_FSM = SEMTECH_FSM_RESET;
static unsigned char semtech_data_index = 0; 
static unsigned char semtech_data[4];
static unsigned char semtech_FSM_next[] = { 
	(SEMTECH_FSM_RESET + 1), (SEMTECH_FSM_RESET + 2),
	(SEMTECH_FSM_RESET + 3), SEMTECH_FSM_DATA_READY
}; 

static ibmts_semtech_event events[IBMTS_SAMPLES];

static ibmts_semtech_event *ibmts_semtech_rdp = NULL;
static ibmts_semtech_event *ibmts_semtech_wrp = NULL;

static unsigned long uart_base;

static void ibmts_semtech_handle_read(unsigned char data);
static void ibmts_uart_init(void);

#ifdef CONFIG_ARCTIC2  /* linux-pm */
#include <linux/device.h>

static int ibmts_suspend(struct device * dev, u32 state, u32 level);
static int ibmts_resume(struct device * dev, u32 level);
static int ibmts_scale(struct bus_op_point * op, u32 level);

static struct device_driver ibmts_driver_ldm = {
       name:      	"ibmts_semtech",
       devclass:  	NULL,
       probe:     	NULL,
       suspend:   	ibmts_suspend,
       resume:    	ibmts_resume,
       scale:	  	ibmts_scale,
       remove:    	NULL,
};

static struct device ibmts_device_ldm = {
       name:		"IBM Touch panel(Semtech) ",
       bus_id:		"ibmts",
       driver: 		NULL,
       power_state:	DPM_POWER_OFF,
};



static void ibmts_ldm_register(void)
{
	extern void opb_driver_register(struct device_driver *driver);
	extern void opb_device_register(struct device *device);

	opb_driver_register(&ibmts_driver_ldm);
	opb_device_register(&ibmts_device_ldm);
}

static void ibmts_ldm_unregister(void)
{
	extern void opb_driver_unregister(struct device_driver *driver);
	extern void opb_device_unregister(struct device *device);

	opb_driver_unregister(&ibmts_driver_ldm);
	opb_device_unregister(&ibmts_device_ldm);
}

static int ibmts_scale(struct bus_op_point * op, u32 level)
{
	/* linux-pm-TODO */

	return 0;
}

enum {
	Ulcr, Umcr, Uscr, Uier, Udll, Udlm
};

static int ibmts_suspend(struct device * dev, u32 state, u32 level)
{

  switch(level)
  { 
     case SUSPEND_POWER_DOWN:
       break;
  }
  
  return 0;
}

static int ibmts_resume(struct device * dev, u32 level)
{

  switch(level)
  {
     case RESUME_POWER_ON:
	     ibmts_uart_init();
	     break;
  }

  return 0;
}
#endif /* linux-pm */

/* UART driver for ibmts_semtech */

/*
 * Read bytes, passing them to the semtech protocol handler.
 */

static void ibmts_uart_rx(unsigned char *status)
{
	unsigned char ch;

	do {
		ch = readb(uart_base + UART_RX);

		if (! (*status & (UART_LSR_FE | UART_LSR_PE | UART_LSR_OE)))
			ibmts_semtech_handle_read(ch);

		*status = readb(uart_base + UART_LSR);
	} while (*status & UART_LSR_DR);

	return;
}

/*
 * Serial interrupt handler.
 */

static void ibmts_uart_interrupt(int irq, void *info, struct pt_regs *regs)
{
	unsigned char iir, status;
	int max_loops = 50;

	iir = readb(uart_base + UART_IIR);

	do {
		status = readb(uart_base + UART_LSR);

		if (status & UART_LSR_DR)
			ibmts_uart_rx(&status);

		iir = readb(uart_base + UART_IIR);
	} while (((iir & UART_IIR_NO_INT) == 0) && (--max_loops != 0));

	return;
}

/*
 * Initialize the UART: 19200 baud, FIFO.
 */

static void ibmts_uart_init(void)
{
	unsigned long flags;
	unsigned char scratch;
	unsigned char save_lcr, mcr;
	unsigned char cval, quot;

	uart_base = UART1_IO_BASE;

	save_flags(flags); cli();

	mcr = readb(uart_base + UART_MCR);
	save_lcr = readb(uart_base + UART_LCR);
	writeb(0xBF, uart_base + UART_LCR); /* set up for StarTech test */
	writeb(0, uart_base + UART_EFR);	/* EFR is the same as FCR */
	writeb(0, uart_base + UART_LCR);
	writeb(UART_FCR_ENABLE_FIFO, uart_base + UART_FCR);
	scratch = readb(uart_base + UART_IIR) >> 6;

	/*
	 * Reset the UART.
	 */

	writeb(save_lcr, uart_base + UART_LCR);
	writeb(mcr, uart_base + UART_MCR);
	writeb(UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR |
	       UART_FCR_CLEAR_XMIT, uart_base + UART_FCR);
	writeb(0, uart_base + UART_FCR);
	scratch = readb(uart_base + UART_RX);

	request_irq(UART1_INT, ibmts_uart_interrupt, 0,
		    "Touchscreen", NULL);

	/*
	 * Clear the FIFO buffers and disable them
	 * (they will be reenabled in a moment)
	 */
	writeb(UART_FCR_ENABLE_FIFO, uart_base + UART_FCR);
	writeb(UART_FCR_ENABLE_FIFO |
	       UART_FCR_CLEAR_RCVR |
	       UART_FCR_CLEAR_XMIT, uart_base + UART_FCR);
	writeb(0, uart_base + UART_FCR);

	/*
	 * Clear the interrupt registers.
	 */
	scratch = readb(uart_base + UART_LSR);
	scratch = readb(uart_base + UART_RX);
	scratch = readb(uart_base + UART_IIR);
	scratch = readb(uart_base + UART_MSR);

	/*
	 * Now, initialize the UART 
	 */
	writeb(UART_LCR_WLEN8, uart_base + UART_LCR);	/* reset DLAB */
	mcr = UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2;
	writeb(mcr, uart_base + UART_MCR);
	
	/*
	 * Enable receive interrupts.
	 */

	writeb(UART_IER_RLSI | UART_IER_RDI, uart_base + UART_IER);

	/*
	 * And clear the interrupt registers again for luck.
	 */

	scratch = readb(uart_base + UART_LSR);
	scratch = readb(uart_base + UART_RX);
	scratch = readb(uart_base + UART_IIR);
	scratch = readb(uart_base + UART_MSR);

	/*
	 * Set the UART divisor registers to match the baud rate of
	 * 19200.  Values below were precalculated (ok, copied from gdb
	 * output of debugging real serial driver).
	 */

	cval = 0x03;
	quot = 0x24;

	writeb(cval | UART_LCR_DLAB, uart_base + UART_LCR); /* set DLAB */
	writeb(quot & 0xff, uart_base + UART_DLL);      /* LS of divisor */
	writeb(quot >> 8, uart_base + UART_DLM);	/* MS of divisor */
	writeb(cval, uart_base + UART_LCR);		/* reset DLAB */
	writeb(UART_FCR_ENABLE_FIFO, uart_base + UART_FCR);
	writeb(UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_8,
	       uart_base + UART_FCR);

	restore_flags(flags);
	return;
}


/*
 * End UART driver
 */

static int queue_empty(void)
{
	return (ibmts_semtech_rdp == ibmts_semtech_wrp);
}

struct ts_event *ibmts_semtech_get_event(void)
{
        unsigned long flags;
        ibmts_semtech_event *result;

        spin_lock_irqsave(&ibmts_semtech_lock, flags);

	if ( (ibmts_semtech_rdp == NULL) || queue_empty() )
		return NULL;

	result = ibmts_semtech_rdp;
        ibmts_semtech_rdp = ibmts_semtech_rdp->pNext;

        spin_unlock_irqrestore(&ibmts_semtech_lock, flags);
        return &(result->event);
};

static void ibmts_semtech_put_event(struct ts_event *pEvent)
{
	if ( ibmts_semtech_rdp == NULL)
                return;

	if (! IsOpenForRead)
		return; 

	ibmts_semtech_wrp->event.pressure = pEvent->pressure;
	ibmts_semtech_wrp->event.x = pEvent->x;
	ibmts_semtech_wrp->event.y = pEvent->y;
	ibmts_semtech_wrp->event.millisecs = pEvent->millisecs;
	ibmts_semtech_wrp->event.flags = pEvent->flags;

	ibmts_semtech_wrp = ibmts_semtech_wrp->pNext;
	if (ibmts_semtech_wrp == ibmts_semtech_rdp) /* Wrap */
		ibmts_semtech_rdp = ibmts_semtech_rdp->pNext; /* Write over */ 
	else
		if (waitqueue_active(&ibmts_semtech_proc_list))
			wake_up_interruptible(&ibmts_semtech_proc_list);
}

static int ibmts_semtech_open(struct inode *inode, struct file *file)
{
  	int i;

  	printk("ibmts: %d.%d\n",
	       inode->i_rdev >> 8, inode->i_rdev & 0xFF);
        if (file->f_flags & O_NONBLOCK)
		printk("ibmts: Non-blocked mode\n");
	else
		printk("ibmts: Blocked mode\n");

	if ((file-> f_mode & FMODE_READ)) {
  		if (IsOpenForRead)
			return -EBUSY;
      		IsOpenForRead++;
   	}


	/* Initialize the structs. */
	ibmts_semtech_rdp = &(events[0]);
	ibmts_semtech_wrp = ibmts_semtech_rdp;
	for (i=0; i<IBMTS_SAMPLES; i++)
		events[i].pNext = &(events[i+1]);

	events[IBMTS_SAMPLES-1].pNext = &(events[0]);
	pen_down = 0;
	semtech_FSM = SEMTECH_FSM_RESET; 
	semtech_data_index = 0;

	MOD_INC_USE_COUNT;

	return 0;
}


static int ibmts_semtech_release(struct inode *inode, struct file *file)
{
	if ((file-> f_mode & FMODE_READ))
  		IsOpenForRead--;

	MOD_DEC_USE_COUNT;

  	return 0;
}

static ssize_t ibmts_semtech_read(struct file *pFile, char *buffer,
				  size_t count, loff_t *ppos)
{
        ssize_t bytes_read = 0;
        struct ts_event *pWork;

        while (queue_empty()) { /* wait for an event */
                if (pFile->f_flags & O_NONBLOCK)
                        return -EWOULDBLOCK;

		if (wait_event_interruptible(ibmts_semtech_proc_list, ! queue_empty()))
			return -ERESTARTSYS; 
        }

        while ( (bytes_read < count)  && (! queue_empty()) ) {
                pWork = ibmts_semtech_get_event();
		if (pWork) {
	                if (copy_to_user (buffer + bytes_read, (void *) pWork,
					  sizeof(struct ts_event))) {
        	                bytes_read = -EFAULT;
                	        break;
	                }
        	        bytes_read += sizeof(struct ts_event);
			if (count - bytes_read < sizeof(struct ts_event)) break;
		}
        }

        if (bytes_read > 0)
                return bytes_read;

        if (signal_pending(current))
                return -ERESTARTSYS;

        return bytes_read;
}

static ssize_t ibmts_semtech_write(struct file *file, const char *buffer,
				   size_t length, loff_t *ppos)
{
	return -EINVAL; 
}

static int ibmts_semtech_ioctl(struct inode * dev_inode, struct file *filep, 
			       unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

/*
  Poll for input - return code indicating if there is input to read.
 */
 
static unsigned int
ibmts_semtech_poll (struct file* filp, poll_table * wait)
{
        poll_wait(filp, &ibmts_semtech_proc_list, wait);
        return queue_empty() ? 0 : (POLLIN | POLLRDNORM);
}

/* Module Declarations ***************************** */

static struct file_operations ibmts_semtech_fops = {
	.owner		= THIS_MODULE,
	.open		= ibmts_semtech_open,
  	.read		= ibmts_semtech_read,
  	.write		= ibmts_semtech_write,
	.ioctl		= ibmts_semtech_ioctl,
	.poll		= ibmts_semtech_poll,
  	.release	= ibmts_semtech_release
};

static void semtech_parse_data(unsigned char *pData)
{
    struct ts_event event; 
    unsigned int dx, dy; 
    static int x=0; /* last x value */
    static int y=0; /* last y value */
    static int id_printed = 0;

    if (pData[0] == SEMTECH_ID) {
	    unsigned char class = pData[1] & 0x0f;
	    unsigned char family = ((pData[1] & 0x40) >> 2) | 
		    (pData[2] & 0x0f);
	    unsigned char sub = (pData[2] & 0x70) >> 4;
	    unsigned int version = ((pData[1] & 0x30) << 3) | pData[3];

	    if (! id_printed++)
		    printk("Semtech class 0x%x family 0x%x sub-product 0x%x version 0x%x.\n",
			   class, family, sub, version);
	    return;
    }

/* #define IBMTS_SEMTECH_XYSWAP */

#if defined(IBMTS_SEMTECH_XYSWAP)
        dx = (((unsigned int)pData[1] & 0x70) << 3) | ((unsigned int)pData[3] & 0x7f);
        dy = (((unsigned int)pData[1] & 0x07) << 7) | ((unsigned int)pData[2] & 0x7f);
#else
        dy = (((unsigned int)pData[1] & 0x70) << 3) | ((unsigned int)pData[3] & 0x7f);
        dx = (((unsigned int)pData[1] & 0x07) << 7) | ((unsigned int)pData[2] & 0x7f);
#endif

    if (! pen_down) { /* Pen up until now */
        if (( pData[0] & 0x1f)) { /* Pen still up */
                return;
        } else {
            event.x = dx;
            event.y = dy;
            event.pressure = 500;
            event.millisecs = jiffies*10;
            pen_down = 1;
        }
    } else { /* Pen was down */
        if (pData[0] & 0x07f) { /* Pen up */
            event.x = -1;
            event.y = -1;
            event.pressure = 0;
            event.millisecs = jiffies*10;
            pen_down = 0;
        } else {    /* Pen Down */
            /* The pen is down and gliding, however omit 
             * values that are not of significant difference. 
             */
            dx &= sample_mask;
            dy &= sample_mask;
            if ( (dx != x) || (dy != y) ) {
                /* create events for the glide */
                event.x = dx;
                event.y = dy;
                event.pressure = 500;
                event.millisecs = jiffies * 10;
            } else {
                /* not a significan difference to warrant adding to the 
                 * event queue. 
                 */
                return;
            }
        }   /* Pen Down End */
    }

    /* save old values */
    x = dx;
    y = dy;

    ibmts_semtech_put_event(&event); /* Wakeup handled in put_event() */
}


static void ibmts_semtech_handle_read(unsigned char data)
{
	/* Between packets & sync flag on, or In packet & sync flag off */
	if ( ( (semtech_FSM == SEMTECH_FSM_RESET) && (data & SEMTECH_SYNC_FLAG) ) ||   
	     ( (semtech_FSM != SEMTECH_FSM_RESET) && !(data & SEMTECH_SYNC_FLAG) ) ) {
		/* Record and proceed to next stage   */
		semtech_data[semtech_data_index++] = data; 
		semtech_FSM = semtech_FSM_next[semtech_FSM]; 
	} else { /* Out of sync */
		semtech_data_index = 0;
		
		if (data & SEMTECH_SYNC_FLAG) {
			semtech_data[semtech_data_index++] = data; 
			semtech_FSM = semtech_FSM_next[semtech_FSM]; 
		} else {
			semtech_FSM = SEMTECH_FSM_RESET;
		}
	}

	/* Full packet received, process it now */
	if (semtech_FSM == SEMTECH_FSM_DATA_READY) {
		semtech_parse_data(semtech_data); 
		semtech_FSM = SEMTECH_FSM_RESET; 
		semtech_data_index = 0;
	}
	
}

/* proc fs entries for the driver. */
static struct proc_dir_entry *proc_semtech_ts;
static struct proc_dir_entry *proc_semtech_ts_mask;

static int read_proc_mask(char *page, char **start, off_t offset, int count, int *eof, void *data) {

    int len = 0;
    
    len += sprintf(page+len, "%x\n",sample_mask);
    *eof = 1;
    return len;

}   

#define isdigit(c) (c >= '0' && c <= '9')
__inline static int atoi( char *s) {
    int i = 0;
    while (isdigit(*s))
      i = i*10 + *(s++) - '0';
    return i;
}
static int write_proc_mask(struct file *file, const char *buffer, unsigned long count, void *data) {

    char *mybuf;

    if (current->uid != 0)
        return -EACCES;

    if (count == 0)
        return 0;

    if (!(mybuf = kmalloc(count+1, GFP_KERNEL)))
        return -ENOMEM;

    if(copy_from_user(mybuf,buffer,count)) {
        kfree(mybuf);
        return -EFAULT;
    }


    mybuf[count] = '\0';    /* remove the trailing newline. */

    /* Convert the mask to decimal and update the mask.
     * I agree this code should accept hex input as well.
     */
    sample_mask = atoi(mybuf);

    kfree(mybuf);
    return count;
                                          
}
void proc_init(void) {

    proc_semtech_ts = proc_mkdir("driver/semtech_ts",NULL);

    if (proc_semtech_ts == NULL) {
        printk(KERN_ERR "Attempt to create proc entry for semtech touchscreen failed.\n");
        return;
    }

//  proc_semtech_ts_mask = create_proc_entry("mask",S_IWUSR,proc_semtech_ts);
//  if (proc_semtech_ts == NULL) {
//      printk(KERN_ERR "Attempt to create proc entry for sensitivity mask failed.\n");
//      return;
//  } else {
//      proc_semtech_ts_mask->write_proc = write_proc_mask;
//  }

    proc_semtech_ts_mask = create_proc_read_entry("mask",S_IRUGO|S_IWUSR,proc_semtech_ts,read_proc_mask,NULL);
    if (proc_semtech_ts == NULL) {
        printk(KERN_ERR "Attempt to create proc entry for sensitivity mask failed.\n");
        return;
    } else {
        proc_semtech_ts_mask->write_proc = write_proc_mask;
                              
    }
}
void proc_cleanup(void) {


    if (proc_semtech_ts_mask) {
        remove_proc_entry("mask",proc_semtech_ts);
        proc_semtech_ts_mask = NULL;
    }
    if (proc_semtech_ts) {
        remove_proc_entry("driver/semtech_ts",NULL);
        proc_semtech_ts = NULL;
    }
}




MODULE_AUTHOR("Ken Inoue");
MODULE_DESCRIPTION("IBM Arctic-2 Semtech touch panel controller driver\n");

int __init init_ibmts_semtech_module(void)
{
	int err;

	init_waitqueue_head(&ibmts_semtech_proc_list);

  	err = register_chrdev(IBMTS_MAJOR, IBMTS_DEVICE_NAME, &ibmts_semtech_fops);

  	if (err < 0) {
    		printk("ibmts_semtech: register_chrdev failed with %d\n",  err);
	    	return err;
	}

  	printk("ibmts_semtech: v0.03 e-8 device major %d \n", IBMTS_MAJOR);
	ibmts_uart_init();

#ifdef CONFIG_PROC_FS
    /* Initialize the proc filesystem. */
    proc_init();
#endif

#ifdef CONFIG_ARCTIC2 /* linux-pm */
    ibmts_ldm_register();
#endif                /* linux-pm */
  	return 0;
}

void cleanup_ibmts_semtech_module(void)
{
  	int ret;

  	ret = unregister_chrdev(IBMTS_MAJOR, IBMTS_DEVICE_NAME);
	if (ret < 0)
		printk("Error : unregister_chrdev: %d\n", ret);

#ifdef CONFIG_PROC_FS
    /* clean up the proc filesystem. */
    proc_cleanup();
#endif

#ifdef CONFIG_ARCTIC2 /* linux-pm */
    ibmts_ldm_unregister();
#endif                /* linux-pm */

}

module_init(init_ibmts_semtech_module);
module_exit(cleanup_ibmts_semtech_module);
