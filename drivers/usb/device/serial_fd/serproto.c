/*
 * serial_fd/serproto.h
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *      Bruce Balden <balden@lineo.com>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */


#include <linux/config.h>
#include <linux/module.h>
#ifndef MODULE
#undef GET_USE_COUNT
#define GET_USE_COUNT(foo) 1
#endif

#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/termios.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/devfs_fs_kernel.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <asm/segment.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <asm/system.h>


#include "serproto.h"


//#define        SERIAL_TTY_MAJOR         222
#define        SERIAL_TTY_MAJOR         188	// re-use USB host tty dev number
#define        SERIAL_TTY_MINORS         1

static int serial_refcount;
static struct tty_struct *serial_tty[SERIAL_TTY_MINORS];
static struct termios *serial_termios[SERIAL_TTY_MINORS];
static struct termios *serial_termios_locked[SERIAL_TTY_MINORS];
extern int usb;


#define        MIN(a,b) ((a>b) ? b : a)
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

struct serproto_dev {
	int number;		// serial device number (index in serproto_device_array)
	int opencnt;		// number of opens
	int connected;		// TRUE if USB has connected.
	unsigned int clocal;

	struct tq_struct write_wakeup_task;	// task queue for line discipline waking up

	struct tty_struct *tty;	// serial tty structure

	struct tty_driver tty_driver;

	int (*xmit_data) (int, unsigned char *, int);	// callback to send data

	int tx_size;		// maximum transmit size

	rwlock_t rwlock;	// lock changing this structure
	//static DECLARE_MUTEX_LOCKED(busy);// semaphore to wait if busy

	unsigned int max_queue_entries;	// maximum queue entries
	unsigned int max_queue_bytes;	// maximum queued data

	unsigned int queued_entries;	// current queued entries
	unsigned int queued_bytes;	// current queued data

	int blocked;
	int trailer;

};

int serproto_devices;		// maximum number of interaces 

static struct serproto_dev **serproto_device_array;	// pointer to active interaces

static rwlock_t serproto_rwlock = RW_LOCK_UNLOCKED;	// lock for changing global structures

/* Debug switches ****************************************************************************** */

static int dbgflg_init = 0;
static int dbgflg_oc = 0;
static int dbgflg_rx = 0;
static int dbgflg_tx = 0;
static int dbgflg_mgmt = 0;
static int dbgflg_loopback = 0;

static debug_option dbg_table[] = {
	{&dbgflg_init, NULL, "init", "initialization/termination handling"},
	{&dbgflg_oc, NULL, "opcl", "open/close handling"},
	{&dbgflg_rx, NULL, "rx", "receive (from host)"},
	{&dbgflg_tx, NULL, "tx", "transmit (to host)"},
	{&dbgflg_mgmt, NULL, "mgmt", "ioctl, termios, etc."},
	{&dbgflg_loopback, NULL, "loop", "enable loopback if non-zero"},
	{NULL, NULL, NULL, NULL}
};

#define dbg_init(lvl,fmt,args...) dbgPRINT(dbgflg_init,lvl,fmt,##args)
#define dbg_oc(lvl,fmt,args...) dbgPRINT(dbgflg_oc,lvl,fmt,##args)
#define dbg_rx(lvl,fmt,args...) dbgPRINT(dbgflg_rx,lvl,fmt,##args)
#define dbg_tx(lvl,fmt,args...) dbgPRINT(dbgflg_tx,lvl,fmt,##args)
#define dbg_mgmt(lvl,fmt,args...) dbgPRINT(dbgflg_mgmt,lvl,fmt,##args)
#define dbg_loop(lvl,fmt,args...) dbgPRINT(dbgflg_loopback,lvl,fmt,##args)

debug_option *serproto_get_dbg_table (void)
{
	return (dbg_table);
}

/* Serial Driver Support Functions ************************************************************* */


/* *
 * serial_open - open serial device
 * @tty: tty device
 * @filp: file structure
 *
 * Called to open serial device.
 */
static int serial_open (struct tty_struct *tty, struct file *filp)
{
	unsigned long flags;
	int n = 0, rc = 0;
	struct serproto_dev *device = NULL;

	dbg_oc (3, "tty #%p file #%p", tty, filp);

	if (NULL == tty || 0 > (n = MINOR (tty->device) - tty->driver.minor_start) ||
	    n >= serproto_devices || NULL == (device = serproto_device_array[n])) {
		dbg_oc (1, "FAIL ENODEV");
		return -ENODEV;
	}

	MOD_INC_USE_COUNT;
	dbg_init (1, "OPEN uc=%d", GET_USE_COUNT (THIS_MODULE));
	write_lock_irqsave (&device->rwlock, flags);

	if (1 == ++device->opencnt) {
		// First open
		tty->driver_data = device;
		device->tty = tty;
		tty->low_latency = 1;


		/* force low_latency on so that our tty_push actually forces the data through, 
		 * otherwise it is scheduled, and with high data rates (like with OHCI) data
		 * can get lost. 
		 * */
		tty->low_latency = 1;

	} else if (tty->driver_data != device || device->tty != tty) {
		// Second or later open, different tty/device combo
		rc = -EBUSY;
	}
	// XXX Should extract info from somewhere to see if receive is OK
	write_unlock_irqrestore (&device->rwlock, flags);

	if (0 != rc) {
		if (-EBUSY == rc) {
			dbg_oc (1, "2nd, conflict: old dev #%p new #%p, old tty #%p new #%p",
				tty->driver_data, device, device->tty, tty);
		}
		MOD_DEC_USE_COUNT;
		dbg_init (0, "OPEN rc=%d uc=%d", rc, GET_USE_COUNT (THIS_MODULE));
	}
	dbg_oc (3, "->%d n=%d", rc, n);
	return (rc);
}

static void serial_close (struct tty_struct *tty, struct file *filp)
{
	unsigned long flags;
	struct serproto_dev *device;
	int uc;

	uc = GET_USE_COUNT (THIS_MODULE);
	dbg_oc (3, "tty #%p file #%p uc=%d", tty, filp, uc);

	if ((device = tty->driver_data) != NULL) {
		write_lock_irqsave (&device->rwlock, flags);
		if (0 >= --device->opencnt) {
			// Last (or extra) close
			dbg_oc (1, "Last: old tty #%p new #%p oc=%d",
				device->tty, tty, device->opencnt);
			tty->driver_data = NULL;
			device->tty = NULL;
			device->opencnt = 0;
		}
		write_unlock_irqrestore (&device->rwlock, flags);
	} else {
		dbg_oc (1, "not presently connected");
	}
	if (uc > 0) {
		// Should really check that uc hasn't changed since start of fn...
		MOD_DEC_USE_COUNT;
		dbg_init (1, "CLOSE uc=%d", GET_USE_COUNT (THIS_MODULE));
	}
	dbg_oc (3, "OK");
	return;
}

static void serial_flush (struct tty_struct *tty)
{
	dbg_mgmt (1, "tty#%p", tty);
}

static int serial_write (struct tty_struct *tty, int from_user, const unsigned char *buf, int count)
{
	// Return the number of bytes (out of count) that get written,
	// or negative for error.
	unsigned long flags;
	struct serproto_dev *device;
	int cnt = count;
	int size;
	const unsigned char *currpos = buf;

	unsigned char *buffer;

	dbg_tx (3, "count=%d", count);

	if ((device = tty->driver_data) == NULL) {
		dbg_tx (1, "not presently connected -> FAIL");
		return -EINVAL;
	}
	dbgPRINTmem (dbgflg_tx, 4, buf, count);

	// loop on data
	while (cnt > 0) {

		int length;

		// send at most tx_size bytes
		size = MIN (device->tx_size, cnt);

		write_lock_irqsave (&device->rwlock, flags);
		// Make sure we can send.
		if (!device->connected ||
		    (device->max_queue_entries > 0
		     && (device->queued_entries >= device->max_queue_entries))
		    || (device->queued_bytes >= device->max_queue_bytes)) {
			// Can't write any more,
			// return the number that we did manage to send.
			write_unlock_irqrestore (&device->rwlock, flags);
			dbg_tx (2, "->%d/%d", (count - cnt), count);
			return (count - cnt);
		}
		size = MIN ((device->max_queue_bytes - device->queued_bytes), size);

		// allocate a buffer

		length = (device->blocked ? device->tx_size : size) + 1 + device->trailer;
		dbg_tx (1, "------> blocked: %d tx_size: %d size: %d trailer: %d, length: %d",
			device->blocked, device->tx_size, size, device->trailer, length);

		if ((buffer = kmalloc (length, GFP_KERNEL)) == NULL) {
			write_unlock_irqrestore (&device->rwlock, flags);
			dbg_tx (2, "->ENOMEM");
			return -ENOMEM;
		}
		memset (buffer, '\0', length);


		// copy data
		if (from_user) {
			copy_from_user ((void *) buffer, currpos, size);
		} else {
			memcpy ((void *) buffer, currpos, size);
		}

		currpos += size;
		cnt -= size;

		device->xmit_data (device->number, buffer, size);

		device->queued_entries++;
		device->queued_bytes += size;
		write_unlock_irqrestore (&device->rwlock, flags);
	}

	// Everything went out.
	dbg_tx (5, "->%d (all)", count);

	return count;
}


static int serial_write_room (struct tty_struct *tty)
{
	/* Return the amount of room for writing. */
	unsigned long flags;
	struct serproto_dev *device;
	int n = 0;

	dbg_tx (7, "entered");

	if ((device = tty->driver_data) == NULL) {
		dbg_tx (1, "not presently connected -> FAIL");
		return (-EINVAL);
	}
	read_lock_irqsave (&device->rwlock, flags);
	if (device->connected &&
	    (device->queued_bytes < device->max_queue_bytes) &&
	    (device->max_queue_entries == 0
	     || device->queued_entries < device->max_queue_entries)) {
#if 0
		if (device->tx_size < (n = device->max_queue_bytes - device->queued_bytes)) {
			n = device->tx_size;
		}
#else
		n = device->max_queue_bytes - device->queued_bytes;
#endif
	}
	read_unlock_irqrestore (&device->rwlock, flags);
	// Shouldn't really access these outside the lock, but only the dbg msg can go wrong.
	dbg_tx (6, "c:%c b=%u/%u e=%u/%u -> %d", (device->connected ? 'T' : 'F'),
		device->queued_bytes, device->max_queue_bytes,
		device->queued_entries, device->max_queue_entries, n);
	return (n);
}


static int
serial_ioctl (struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc, type;

	type = _IOC_TYPE (cmd);
	dbg_mgmt (2, "type#%02x cmd#%08x arg#%08lx", type, cmd, arg);

	switch (cmd) {
	case /* TCSETATTR */ 3:
		rc = 0;
		break;
	default:
		rc = -ENOIOCTLCMD;
	}

	return (rc);
}


static void serial_set_termios (struct tty_struct *tty, struct termios *old)
{
	struct serproto_dev *device = tty->driver_data;
	struct termios *tio = tty->termios;

	device->clocal = tio->c_cflag & CLOCAL;
	dbg_mgmt (2, "clocal->%c", (device->clocal ? 'T' : 'F'));

	return;
}


static void serial_throttle (struct tty_struct *tty)
{
	dbg_mgmt (1, "entered");

	return;
}

static void serial_unthrottle (struct tty_struct *tty)
{
	dbg_mgmt (1, "entered");

	return;
}

static int serial_chars_in_buffer (struct tty_struct *tty)
{
	struct serproto_dev *device;
	int n;

	//dbg_rx(4,"entered");

	if ((device = tty->driver_data) == NULL) {
		dbg_rx (1, "not presently connected -> FAIL");
		return (-EINVAL);
	}
	read_lock (&device->rwlock);
	n = device->queued_bytes;
	read_unlock (&device->rwlock);
	//dbg_rx(4,"->%d",n);
	return (n);
}



static struct tty_driver serial_tty_driver = {
	magic:TTY_DRIVER_MAGIC,
	driver_name:"usbd-serial",
	name:USBD_TTY_NAME,
	major:SERIAL_TTY_MAJOR,
	minor_start:1,
	num:SERIAL_TTY_MINORS,
	type:TTY_DRIVER_TYPE_SERIAL,
	subtype:SERIAL_TYPE_NORMAL,
	flags:TTY_DRIVER_REAL_RAW | TTY_DRIVER_NO_DEVFS,

	refcount:&serial_refcount,
	table:serial_tty,
	termios:serial_termios,
	termios_locked:serial_termios_locked,

	open:serial_open,
	close:serial_close,
	flush_buffer:serial_flush,
	write:serial_write,
	write_room:serial_write_room,
	ioctl:serial_ioctl,
	set_termios:serial_set_termios,
	throttle:serial_throttle,
	unthrottle:serial_unthrottle,
	chars_in_buffer:serial_chars_in_buffer,
};


/* Library Interface Functions ***************************************************************** */

/**
 * serproto_modinit - initialize the serproto library
 * @name: name 
 * @num: number of interfaces to allow
 *
 */
int serproto_modinit (char *name, int num)
{
	dbg_init (1, "%s[%d]", name, num);

	rwlock_init (&serproto_rwlock);
	serproto_devices = num;

	if (!(serproto_device_array = kmalloc (sizeof (struct serproto_dev *) * num, GFP_KERNEL))) {
		dbg_init (0, "kmalloc failed");
		return -EINVAL;
	}
	memset (serproto_device_array, 0, sizeof (struct serproto_dev *) * num);

	dbg_loop (1, "LOOPBACK mode");

	return 0;
}

static void wakeup_writers (void *private)
{
	struct serproto_dev *device = (struct serproto_dev *) private;
	struct tty_struct *tty;

	if (NULL != device && NULL != (tty = device->tty)) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && NULL != tty->ldisc.write_wakeup) {
			(tty->ldisc.write_wakeup) (tty);
		}
		wake_up_interruptible (&tty->write_wait);
	}
	MOD_DEC_USE_COUNT;
}

/**
 * serproto_create - create a serial interface
 * @name: name
 * @xmit_data: callback to transmit data
 * @max_queue_entries: maximum number of outstanding requests to allow
 * @max_queue_bytes: maximum number of bytes to allow
 *
 * Create a serial interface, providing an xmit data function. 
 *
 * A returned value greater or equal to zero indicates success. This value can be 
 * used with subsequent function calls to indicate the created serial interface.
 */

int serproto_create (char *name,
		     int (*xmit_data) (int, unsigned char *, int),
		     int tx_size,
		     int max_queue_entries, int max_queue_bytes, int blocked, int trailer)
{
	struct serproto_dev *device;
	int i;

	dbg_init (1, "array: %p name: %p[%s] xmit: %p", serproto_device_array, name, name,
		  xmit_data);

	if (!serproto_device_array) {
		dbg_init (1, "invalid args -> FAIL");
		return (-EINVAL);
	}

	/* Note: the tty layer assumes that if *_write_room() returns n > 0, that
	   n more bytes can be queued up in UPTO n CALLS TO *_write().  It does not
	   re-verify the available room.  This means that if we are limiting the
	   number of queued entries, and we indicate that 64 _bytes_ can be queued,
	   there may be upto 64 *_write(1) calls, which is probably going to overflow
	   the queue.  It would also seem that the tty layer does not check the
	   return value of the *_write() calls, so that calls which fail due to
	   insufficient queue entries are simply lost.  For this reason, setting
	   max_queue_entries to 0 allows unlimitied queueing. */
	if (!serproto_device_array || !name || !strlen (name) || !xmit_data ||
	    max_queue_entries < 0 || max_queue_bytes <= 0) {
		dbg_init (1, "invalid args -> FAIL");
		return (-EINVAL);
	}

	{
		unsigned long flags;
		write_lock_irqsave (&serproto_rwlock, flags);
		for (i = 0; i < serproto_devices; i++) {
			if (!(device = serproto_device_array[i])) {
				break;
			}
		}
		if (i == serproto_devices) {
			write_unlock_irqrestore (&serproto_rwlock, flags);
			dbg_init (1, "name: %s cannot find empty serproto_device slot", name);
			return (-ENOMEM);
		}

		if (!(device = kmalloc (sizeof (struct serproto_dev), GFP_ATOMIC))) {
			write_unlock_irqrestore (&serproto_rwlock, flags);
			dbg_init (1, "name: %s kmalloc failed", name);
			return (-ENOMEM);
		}

		memset (device, 0, sizeof (struct serproto_dev));
		rwlock_init (&device->rwlock);

		device->xmit_data = xmit_data;

		device->tx_size = MIN (256, tx_size);	// maximum 256 bytes
		device->max_queue_entries = max_queue_entries;
		device->max_queue_bytes = max_queue_bytes;

		device->write_wakeup_task.routine = wakeup_writers;
		device->write_wakeup_task.data = device;

		device->blocked = blocked;
		device->trailer = trailer;

		memcpy (&device->tty_driver, &serial_tty_driver, sizeof (struct tty_driver));
		device->tty_driver.init_termios = tty_std_termios;
		device->tty_driver.init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
		// Turn off echo fight with standard UNIX USB Host serial drivers.
		device->tty_driver.init_termios.c_lflag &= ~(ECHO | ICANON);
		/* We don't need \n -> \r\n output conversion */
                device->tty_driver.init_termios.c_oflag &= ~ONLCR;
		serproto_device_array[i] = device;
		write_unlock_irqrestore (&serproto_rwlock, flags);
	}

	if (tty_register_driver (&device->tty_driver)) {
		unsigned long flags;
		write_lock_irqsave (&serproto_rwlock, flags);
		serproto_device_array[i] = NULL;
		kfree (device);
		write_unlock_irqrestore (&serproto_rwlock, flags);
		printk (KERN_ERR "%s(%s) - failed to register tty driver\n",__FUNCTION__,name);
		dbg_init (1, "try registration failed");
		return (-EINVAL);
	}
	tty_register_devfs (&device->tty_driver, DEVFS_FL_DEFAULT, device->tty_driver.minor_start);
	dbg_init (2, "name: %s -> %d", name, i);
	return (i);
}

/**
 * serproto_done - transmit done
 * @interface: device interface
 * @data: data buffer
 * @len: data length
 * @rc: non-zero indicates failure
 *
 */
int serproto_done (int interface, void *data, int len, int rc)
{
	struct serproto_dev *device;
	unsigned long flags;

	dbg_tx (4, "interface: %d", interface);

	// Must be done with interrupts off, since sync may
	// be set by another (different) call to queue_task()

	write_lock_irqsave (&device->rwlock, flags);

	if ((device = serproto_device_array[interface]) == NULL) {
		write_unlock_irqrestore (&device->rwlock, flags);
		dbg_tx (1, "no device -> FAIL");
		return (-EINVAL);
	}

	device->queued_entries--;
	device->queued_bytes -= len;

	if (!device->write_wakeup_task.sync) {
		// Queue a wakeup for anyone waiting to write,
		// but lock the module in place first.
		MOD_INC_USE_COUNT;
		queue_task (&device->write_wakeup_task, &tq_immediate);
		mark_bh (IMMEDIATE_BH);
	}
	write_unlock_irqrestore (&device->rwlock, flags);

	return 0;
}


/**
 * serproto_recv - receive data
 * @data: data buffer
 * @length: length of valid data
 */
int serproto_recv (int interface, unsigned char *data, int length)
{
	struct serproto_dev *device;
	struct tty_struct *tty;
#if 0
	unsigned long flags;
#endif
	int i;

	dbg_rx (4, "length=%d", length);
	dbg_rx (5, "interface: %d data: %p serproto_device_array: %p",
		interface, data, serproto_device_array);
	dbg_rx (5, "device: %p", serproto_device_array[interface]);

	if ((device = serproto_device_array[interface]) == NULL) {
		dbg_rx (1, "no device -> FAIL");
		return (-EINVAL);
	}
#if 0
	if (0 != dbgflg_loopback) {
		// serproto_done() will free data when transmission complete
		device->xmit_data (device->number, data, length);
		write_lock_irqsave (&device->rwlock, flags);
		device->queued_entries++;
		device->queued_bytes += length;
		write_unlock_irqrestore (&device->rwlock, flags);
		dbg_rx (3, "loopback->0");
		return (0);
	}
#endif

	if (device->opencnt <= 0 || (tty = device->tty) == NULL) {
		dbg_rx (1, "no tty -> FAIL");
		//kfree(data);
		return (-EINVAL);
	}
	// XXX bail if not open for reading....
	for (i = 0; i < length; ++i) {
		tty_insert_flip_char (tty, data[i], 0);
	}
	tty_flip_buffer_push (tty);
	//kfree(data);
	dbg_rx (4, "->0");
	return 0;
}


/**
 * serproto_control - control serial interface
 * @interface: serial interface
 * @operation: operation
 *
 * Control a serial interface, 
 */
int serproto_control (int interface, int operation)
{
	struct serproto_dev *device;
	unsigned long flags;
	dbg_mgmt (1, "interface: %d op=%d", interface, operation);
	if ((device = serproto_device_array[interface]) == NULL) {
		dbg_mgmt (1, "no device");
		return (-ENODEV);
	}
	switch (operation) {
	case SERPROTO_CONNECT:
		write_lock_irqsave (&device->rwlock, flags);
		device->connected = TRUE;
		/* Wake up anybody who might have blocked waiting
		   for the connection. */
		// Must be done with interrupts off, since sync may
		// be set by another (different) call to queue_task()
		if (!device->write_wakeup_task.sync) {
			// Queue a wakeup for anyone waiting to write,
			// but lock the module in place first.
			MOD_INC_USE_COUNT;
			queue_task (&device->write_wakeup_task, &tq_immediate);
			mark_bh (IMMEDIATE_BH);
		}
		write_unlock_irqrestore (&device->rwlock, flags);
		break;
	case SERPROTO_DISCONNECT:
		device->connected = FALSE;
		/* Send a HANGUP to anybody involved with this device. */
		if (device->tty && !device->clocal) {
			dbg_mgmt (1, "calling hangup");
			tty_hangup (device->tty);
		}

	}

	return 0;
}


/**
 * serproto_destroy - destroy a serial interface
 * @interface: serial interface
 *
 * Call to tear down a previously created serial interface
 */

int serproto_destroy (int interface)
{
	struct serproto_dev *device;
	unsigned int flags;

	dbg_init (1, "interface: %d", interface);

	if ((device = serproto_device_array[interface]) == NULL) {
		dbg_init (1, "no device");
		return (0);
	}
	// grab device lock and delete device entry from serproto device array
	write_lock_irqsave (&serproto_rwlock, flags);
	device = serproto_device_array[interface];
	serproto_device_array[interface] = NULL;
	// Let write_wakeup_task know this device is toast.
	device->tty = NULL;
	write_unlock_irqrestore (&serproto_rwlock, flags);

	// remove serial interace
	tty_unregister_driver (&device->tty_driver);
	tty_unregister_devfs (&device->tty_driver, device->tty_driver.minor_start);
	// XXXX make sure write_wakeup_task is not scheduled!!!
	kfree (device);

	dbg_init (2, "->0");
	return 0;
}


/** 
 * serproto_modexit - unload library
 *
 */
void serproto_modexit (void)
{
	int i;
	int devices;
	struct serproto_dev **device_array;
	unsigned int flags;

	dbg_init (1, "entered");

	if (0 == serproto_devices) {
		// nothing to do
		return;
	}

	/* This should never be called from
	   an interrupt context, so irq save/restore
	   shouldn't be required, but it isn't going
	   to be called often enough for the extra
	   overhead to hurt either. */
	write_lock_irqsave (&serproto_rwlock, flags);
	devices = serproto_devices;
	serproto_devices = 0;
	write_unlock_irqrestore (&serproto_rwlock, flags);

	for (i = 0; i < devices; i++) {
		serproto_destroy (i);
	}

	write_lock_irqsave (&serproto_rwlock, flags);
	device_array = serproto_device_array;
	serproto_device_array = NULL;
	write_unlock_irqrestore (&serproto_rwlock, flags);
	kfree (serproto_device_array);
}
