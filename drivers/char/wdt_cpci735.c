/*
 * Watchdog Timer for the Force CPCI735
 *
 * Corey Minyard (cminyard@mvista.com)
 */


#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/spinlock.h>

#include <asm/uaccess.h>
#include <asm/io.h>


/************************************************************************/
/* The following are defines for the CPCI735 FPGA2.  If something else
   ever uses that FPGA's registers, this will need to be split out. */

#define CPCI735_FPGA2_RESET_STAT_REG	0x03
#define CPCI735_FPGA2_WDOG_RETRIG_REG	0x05
#define CPCI735_FPGA2_WDOG_CONTROL_REG	0x06
#define CPCI735_FPGA2_VERSION_REG	0x08

#define CPCI735_FPGA2_RESET_STAT_CPCI_RESET	0x01
#define CPCI735_FPGA2_RESET_STAT_PB_RESET	0x02
#define CPCI735_FPGA2_RESET_STAT_GP_RESET	0x04
#define CPCI735_FPGA2_RESET_STAT_RTB_RESET	0x08
#define CPCI735_FPGA2_RESET_STAT_SW_RESET	0x20
#define CPCI735_FPGA2_RESET_STAT_WD_RESET	0x40
#define CPCI735_FPGA2_RESET_STAT_PWRON_RESET	0x80

/* Timeouts (in milliseconds) for the watchdog timeout for each setting
   of the timer.  The index into the array is the setting. */
static unsigned int cpci735_timeouts[] =
{
    37,
    75,
    150,
    300,
    600,
    1200,
    2400,
    4800,
    9600,
    19200,
    38300,
    76700,
    153400,
    306800,
    613600,
    1227200
};

/* Settings for the FUNCT bits of the watchdog control register */
#define CPCI735_FPGA2_WDOG_DISABLE	0x00
#define CPCI735_FPGA2_WDOG_RESET	0x10
#define CPCI735_FPGA2_WDOG_NMI		0x40
#define CPCI735_FPGA2_WDOG_NMI_RESET	0x50
#define CPCI735_FPGA2_WDOG_MASK		0x70

/* Settings for the RESET_MODE bit of the watchdog control register. */
#define DEFAULT_WDOG_AFTER_RESET	0x00
#define KEEP_WDOG_AFTER_RESET		0x80

/* State of the reset status register at startup. */
static unsigned char reset_status_reg;

static spinlock_t fpga2_lock = SPIN_LOCK_UNLOCKED;

static unsigned short fpga2_index_port = 0x102;
static unsigned short fpga2_data_port = 0x103;

static unsigned char cpci735_read_fpga2(int offset)
{
	unsigned char rv;

	spin_lock(&fpga2_lock);
	outb(offset, fpga2_index_port);
	rv = inb(fpga2_data_port);
	spin_unlock(&fpga2_lock);
	return rv;
}

static void cpci735_write_fpga2(int offset, unsigned char value)
{
	spin_lock(&fpga2_lock);
	outb(offset, fpga2_index_port);
	outb(value, fpga2_data_port);
	spin_unlock(&fpga2_lock);
}

/************************************************************************/



#define VERSION "1.0"

/* Default the timeout to 1.2 seconds. */
static int cpci735_default_timeout = 0x05;

/* Is someone using the watchdog (only one user is allowed)? */
static int cpci735_wdog_open = 0;


static void cpci735_heartbeat(void)
{
	/* You can write any value, we write 0xad */
	cpci735_write_fpga2(CPCI735_FPGA2_WDOG_RETRIG_REG, 0xad);
}

static int cpci735_set_timeout(unsigned long arg)
{
	int i;

	for (i=0; i<16; i++)
	{
		if (arg <= cpci735_timeouts[i])
		{
			/* Write the values for the watchdog. */
			cpci735_write_fpga2(CPCI735_FPGA2_WDOG_CONTROL_REG,
					    (CPCI735_FPGA2_WDOG_RESET
					     | DEFAULT_WDOG_AFTER_RESET
					     | i));
			return 0;
		}
	}

	return -EINVAL;
}

static struct watchdog_info ident=
{
    WDIOF_CARDRESET,
    1,
    "CPCI735"
};

static int cpci735_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int i;
	int val;
	unsigned char reg;

	switch(cmd) {
	case WDIOC_GETSUPPORT:
		i = copy_to_user((void*)arg, &ident, sizeof(ident));
		return i ? -EFAULT : 0;

	case WDIOC_SETTIMEOUT:
		i = copy_from_user(&val, (void *) arg, sizeof(int));
		if (i)
			return -EFAULT;
		/* Val comes in as seconds, convert to milliseconds. */
		return cpci735_set_timeout(val * 1000);

	case WDIOC_KEEPALIVE:
		cpci735_heartbeat();
		return 0;

	case WDIOC_SETOPTIONS:
		i = copy_from_user(&val, (void *) arg, sizeof(int));
		if (i)
			return -EFAULT;
		if (val & WDIOS_DISABLECARD)
		{
			/* Disable the watchdog timer, but don't modify the
			   timing parameters. */
			reg = cpci735_read_fpga2
				(CPCI735_FPGA2_WDOG_CONTROL_REG);
			reg &= ~CPCI735_FPGA2_WDOG_MASK;
			reg |= CPCI735_FPGA2_WDOG_DISABLE;
			cpci735_write_fpga2(CPCI735_FPGA2_WDOG_CONTROL_REG,
					    reg);
		}

		if (val & WDIOS_ENABLECARD)
		{
			/* Enable the watchdog timer, but don't modify the
			   timing parameters. */
			reg = cpci735_read_fpga2
				(CPCI735_FPGA2_WDOG_CONTROL_REG);
			reg &= ~CPCI735_FPGA2_WDOG_MASK;
			reg |= CPCI735_FPGA2_WDOG_RESET;
			cpci735_write_fpga2(CPCI735_FPGA2_WDOG_CONTROL_REG,
					    reg);
		}
		return 0;

	case WDIOC_GETBOOTSTATUS:
		val = 0;
		if (reset_status_reg & CPCI735_FPGA2_RESET_STAT_WD_RESET) {
			val |= WDIOF_CARDRESET;
		}
		return copy_to_user((void *) arg, &val, sizeof(val));

	case WDIOC_GETSTATUS:
		val = 0;
		return copy_to_user((void *) arg, &val, sizeof(val));

	case WDIOC_GETTEMP:
	default:
		return -ENOIOCTLCMD;
	}
}

static ssize_t cpci735_write(struct file *file,
			     const char  *buf,
			     size_t      len,
			     loff_t      *ppos)
{
	/*  Can't seek (pwrite) on this device  */
	if (ppos != &file->f_pos)
		return -ESPIPE;

	if (len)
	{
		cpci735_heartbeat();
		return 1;
	}
	return 0;
}

static ssize_t cpci735_read(struct file *file,
			    char        *buf,
			    size_t      count,
			    loff_t      *ppos)
{
	/*  Can't seek (pread) on this device  */
	if (ppos != &file->f_pos)
		return -ESPIPE;

	/* Also can't read it. */
	return -EINVAL;
}

static int cpci735_open(struct inode *ino, struct file *filep)
{
        switch (MINOR(ino->i_rdev))
        {
                case WATCHDOG_MINOR:
                    if (cpci735_wdog_open)
                        return -EBUSY;

                    MOD_INC_USE_COUNT;
                    cpci735_wdog_open = 1;

		    /* Initialize to the default timeout. */
		    cpci735_write_fpga2(CPCI735_FPGA2_WDOG_CONTROL_REG,
					(CPCI735_FPGA2_WDOG_RESET
					 | DEFAULT_WDOG_AFTER_RESET
					 | cpci735_default_timeout));
                    return(0);

                default:
                    return (-ENODEV);
        }
}

static int cpci735_close(struct inode *ino, struct file *filep)
{
	if (MINOR(ino->i_rdev)==WATCHDOG_MINOR)
	{
#ifndef CONFIG_WATCHDOG_NOWAYOUT	
		cpci735_write_fpga2(CPCI735_FPGA2_WDOG_CONTROL_REG,
				    CPCI735_FPGA2_WDOG_DISABLE);
#endif		
	        cpci735_wdog_open = 0;
		MOD_DEC_USE_COUNT;
	}
	return 0;
}

static struct file_operations cpci735_wdog_fops = {
	owner:		THIS_MODULE,
	read:		cpci735_read,
	write:		cpci735_write,
	ioctl:		cpci735_ioctl,
	open:		cpci735_open,
	release:	cpci735_close,
};

static struct miscdevice cpci735_wdog_miscdev = {
	WATCHDOG_MINOR,
	"watchdog",
	&cpci735_wdog_fops
};

static int __init cpci735_wdog_init(void)
{
	unsigned char vera, verb;

printk("A\n");
	vera = cpci735_read_fpga2(CPCI735_FPGA2_VERSION_REG);
	verb = (vera >> 4) & 0xf;
	vera &= 0xf;

	printk("cpci735 watchdog: v%s Corey Minyard (minyard@mvista.com)\n",
	       VERSION);
	printk("  FPGA2 version %d.%d\n", vera, verb);

	reset_status_reg = cpci735_read_fpga2(CPCI735_FPGA2_RESET_STAT_REG);
	cpci735_write_fpga2(CPCI735_FPGA2_RESET_STAT_REG, 0);

	misc_register(&cpci735_wdog_miscdev);

	return 0;
}

static void __exit cpci735_wdog_exit(void)
{
	/*  Disable the timer. */
	cpci735_write_fpga2(CPCI735_FPGA2_WDOG_CONTROL_REG,
			    CPCI735_FPGA2_WDOG_DISABLE);

	misc_deregister(&cpci735_wdog_miscdev);
}

module_init(cpci735_wdog_init);
module_exit(cpci735_wdog_exit);

MODULE_LICENSE("GPL");
