/*
 * bios-less APM driver for ARM Linux 
 *  Jamey Hicks <jamey@crl.dec.com>
 *  adapted from the APM BIOS driver for Linux by Stephen Rothwell (sfr@linuxcare.com)
 *
 * APM 1.2 Reference:
 *   Intel Corporation, Microsoft Corporation. Advanced Power Management
 *   (APM) BIOS Interface Specification, Revision 1.2, February 1996.
 *
 * [This document is available from Microsoft at:
 *    http://www.microsoft.com/hwdev/busbios/amp_12.htm]
 */
/*
 * Copyright (C) 2002-2005 Motorola Inc.
 *
 *  2002-Nov-11  Porting to EzX platform, Zhuang Xiaofan
 *
 *  2003-May-12  Make apm devie only  be accessed by one user, Zhuang Xiaofan
 *
 *  2004-Feb-12  Porting wakeup check routine to E680/A780 (Bulverde), Zhuang Xiaofan
 *
 *  2005-Jun-30  Update pcap interface to spi mode, Lin Weiiqng
 *	
*/

#include <linux/config.h>
#include <linux/module.h>

#include <linux/poll.h>
#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/timer.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/apm_bios.h>
#include <linux/power_ic.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/kernel.h>
#include <linux/smp_lock.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/hardware.h>

#include <asm/arch/irqs.h>
#include <linux/interrupt.h>
#include <asm/mach/irq.h>



#if defined(CONFIG_APM_DISPLAY_BLANK) && defined(CONFIG_VT)
extern int (*console_blank_hook)(int);
#endif

struct apm_bios_info apm_bios_info = {
        /* this driver simulates APM version 1.2 */
        version: 0x102,
        flags: APM_32_BIT_SUPPORT
};

/*
 * The apm_bios device is one of the misc char devices.
 * This is its minor number.
 */
#define	APM_MINOR_DEV	134
#define EZX_BARBADOS_POWEROFF
//#define EZX_BARBADOS_POWEROFF_DEBUG_TRIGGER
/*
 * See Documentation/Config.help for the configuration options.
 *
 * Various options can be changed at boot time as follows:
 * (We allow underscores for compatibility with the modules code)
 *	apm=on/off			enable/disable APM
 *	    [no-]debug			log some debugging messages
 *	    [no-]power[-_]off		power off on shutdown
 */

/*
 * Need to poll the APM BIOS every second
 */
#define APM_CHECK_TIMEOUT	(HZ)

/*
 * Ignore suspend events for this amount of time after a resume
 */
#define DEFAULT_BOUNCE_INTERVAL		(3 * HZ)

/*
 * Timeout to wait for BP signal when power off
 */
#define OSCR_TICKS_1MS         	3250
#define POWER_OFF_TIMEOUT	(5000 * OSCR_TICKS_1MS)

/*
 * The magic number in apm_user
 */
#define APM_BIOS_MAGIC		0x4101

/*
 * Local variables
 */
//static int			suspends_pending;
//static int			standbys_pending;
//static int			ignore_normal_resume;

#ifdef CONFIG_APM_RTC_IS_GMT
#	define	clock_cmos_diff	0
#	define	got_clock_diff	1
#else
//static long			clock_cmos_diff;
//static int			got_clock_diff;
#endif
static int			debug;
static int			apm_disabled;
#ifdef CONFIG_SMP
static int			power_off;
#else
static int			power_off = 1;
#endif
static int			exit_kapmd;
static int			kapmd_running;

/* 
 * timeout setting to profile sleep condition
 */
unsigned long sleep_to = 0;
int can_sleep = 0;
unsigned long sleep_jiffy = 0;
int periodic_wake = 0;
static int user_off_available = 0;
extern int usb_host_suspended;

extern int pm_do_suspend(unsigned int);
extern int pm_do_useroff(void);
extern int irq_pending(void);

extern void ssp_pcap_intoSleepCallBack(void);
extern void ssp_pcap_wakeUpCallBack(void);
extern void ezx_i2c_pm_suspend(void);
extern void ezx_i2c_pm_resume(void);
extern void wmmx_ohci_suspend_call(void);
extern void wmmx_ohci_resume_call(void);

extern int ipc_is_active(void);

static DECLARE_WAIT_QUEUE_HEAD(apm_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(apm_suspend_waitqueue);
static struct apm_user *	user_list = NULL;

static char driver_version[] = "1.20";	/* no spaces */

typedef struct lookup_t {
	int	key;
	char *	msg;
} lookup_t;

static const lookup_t error_table[] = {
/* N/A	{ APM_SUCCESS,		"Operation succeeded" }, */
	{ APM_DISABLED,		"Power management disabled" },
	{ APM_CONNECTED,	"Real mode interface already connected" },
	{ APM_NOT_CONNECTED,	"Interface not connected" },
	{ APM_16_CONNECTED,	"16 bit interface already connected" },
/* N/A	{ APM_16_UNSUPPORTED,	"16 bit interface not supported" }, */
	{ APM_32_CONNECTED,	"32 bit interface already connected" },
	{ APM_32_UNSUPPORTED,	"32 bit interface not supported" },
	{ APM_BAD_DEVICE,	"Unrecognized device ID" },
	{ APM_BAD_PARAM,	"Parameter out of range" },
	{ APM_NOT_ENGAGED,	"Interface not engaged" },
	{ APM_BAD_FUNCTION,     "Function not supported" },
	{ APM_RESUME_DISABLED,	"Resume timer disabled" },
	{ APM_BAD_STATE,	"Unable to enter requested state" },
/* N/A	{ APM_NO_EVENTS,	"No events pending" }, */
	{ APM_NO_ERROR,		"BIOS did not set a return code" },
	{ APM_NOT_PRESENT,	"No APM present" }
};
#define ERROR_COUNT	(sizeof(error_table)/sizeof(lookup_t))

static int apm_get_power_status(u_char *ac_line_status,
                                u_char *battery_status,
                                u_char *battery_flag,
                                u_char *battery_percentage,
                                u_short *battery_life)
{
        //platform_apm_get_power_status(ac_line_status, battery_status, battery_flag, battery_percentage, battery_life);
	return APM_SUCCESS;
}

static int queue_empty(struct apm_user *as)
{
	return as->event_head == as->event_tail;
}

static apm_event_t get_queued_event(struct apm_user *as)
{
	as->event_tail = (as->event_tail + 1) % APM_MAX_EVENTS;
	return as->events[as->event_tail];
}

static int check_apm_user(struct apm_user *as, const char *func)
{
	if ((as == NULL) || (as->magic != APM_BIOS_MAGIC)) {
		printk(KERN_ERR "apm: %s passed bad filp\n", func);
		return 1;
	}
	return 0;
}

static void queue_apm_event(apm_event_t event, struct apm_user *sender)
{
	struct apm_user *	as;

	if (user_list == NULL)
		return;
	for (as = user_list; as != NULL; as = as->next) {
		if (as == sender)
			continue;
		as->event_head = (as->event_head + 1) % APM_MAX_EVENTS;
		if (as->event_head == as->event_tail) {
			static int notified;

			if (notified++ == 0)
			    printk(KERN_ERR "apm: an event queue overflowed\n");
			as->event_tail = (as->event_tail + 1) % APM_MAX_EVENTS;
		}
		as->events[as->event_head] = event;
	}
	wake_up_interruptible(&apm_waitqueue);
}

apm_event_t __inline APM_Events(unsigned short type, unsigned short kind, 
		unsigned int info)
{
	apm_event_t event;
	event.type = type;
	event.kind = kind;
	event.info = info;
	return event;
}		

void apm_event_notify(short type, short kind, int info)
{
	apm_event_t event;
	event = APM_Events(type, kind, info);
	queue_apm_event(event, NULL);
}
EXPORT_SYMBOL(apm_event_notify);

/*
This array maps PEDR register bit -----> irq number,
	IRQ_XXX: corresponding irq num,
	-1:	 bit reserved, 
	-2:	 bit mux, not used 
*/
static int PEDR_to_IRQ[] = {
		IRQ_GPIO(0), IRQ_GPIO(1), -1, IRQ_GPIO(3), 
		IRQ_GPIO(4), -1, -1, -1,
		-1, IRQ_GPIO(9), IRQ_GPIO(10), IRQ_GPIO(11), 
		IRQ_GPIO(12), IRQ_GPIO(13), IRQ_GPIO(14), IRQ_GPIO(15),
		-1, -2, -1, -1,
		-2, -1, -1, -1,
		IRQ_GPIO(35), IRQ_MSL, IRQ_USB, IRQ_USBH1,
		IRQ_USBH2, -1, IRQ_PWRI2C, IRQ_RTCAlrm
};
#define PEDR_bitmap	0xdf12fe1b	/* PEDR effective bits bitmap*/

extern void set_ipc_is_active(void);
/* this function is called after exiting sleep */
static void check_wakeup(void)
{
	int i;
	struct irqaction *action;
	struct irqdesc *desc;
	unsigned long flags;
	unsigned long source = PEDR & PEDR_bitmap;

	printk(KERN_INFO "wakeup: PKSR=0x%x, PEDR=0x%x\n", PKSR, PEDR);
	if (source == 0)
		return;

	local_irq_save(flags);
	PEDR = source;

	/* check whether it is waked by periodic timer */
	if ((source == PEDR_EDRTC) && (RTSR & (RTSR_SWAL1 | RTSR_PIAL)))
		periodic_wake = 1;
	else
		periodic_wake = 0;

	/*check whether system is waked up by gpio0[bp_rdy]*/
	/* prevent the case that system enter sleep again before host resume finished.*/
	if( source == PEDR_GPIO0)
		set_ipc_is_active();	

	/* prevent interrupts loss */
	for (i = 0; i < 32; i++) {
		if ((source & (1 << i)) && (PEDR_to_IRQ[i] >= 0)) {
			desc = irq_desc + PEDR_to_IRQ[i];
			action = desc->action;

			if (action && desc->enabled) {
				action->handler(PEDR_to_IRQ[i], action->dev_id, NULL);
			}
		}
	}
			
	local_irq_restore(flags);
}

static inline void clear_dpcsr()
{
#define DPCSR __REG(0x400000a4)
#define DPCSR_BRG_SPLIT (1 << 31)
#define DPCSR_BRG_BUSY (1 << 0)

	unsigned long cnt = 0;
	do {
		if (!(DPCSR & DPCSR_BRG_BUSY)) {
        		DPCSR = 0;
			return;
		}
	} while (++cnt < 0x80000000);
	printk("Can not clear the dma split of DPCSR\n");
}

static int pm_go_sleep(int arg)
{
       	unsigned long flags;

	sleep_jiffy = 0;

#ifdef CONFIG_NO_USB_SUSPEND_RESUME
	return 0;
#endif
	if ((can_sleep && (!ipc_is_active()) && (!periodic_wake)) || arg) {
                local_irq_save(flags);
                if (pm_send_all(PM_SUSPEND, (void *)3) == 0) {
                        ezx_i2c_pm_suspend();
                        wmmx_ohci_suspend_call();
			if (!irq_pending()) {
                		if(usb_host_suspended)
		                     UP3OCR = 0x3;
                        	pm_do_suspend(CPUMODE_SLEEP);
				clear_dpcsr();
			} 
                        APM_DPRINTK("resume\n");
                        ezx_i2c_pm_resume();
                        pm_send_all(PM_RESUME, (void *)0);
                        check_wakeup();
                } 
                local_irq_restore(flags);
        }
        return 0;
}

/*
 * It will be called after all periodic jobs done to put CPU into sleep directly.
 * Only when CPU is waked by periodic timer and no other interrupts occurred,
 * the sleep sequence would be executed.
 */
int periodic_jobs_done(void)
{
	if (periodic_wake) {
		periodic_wake = 0;
	 	pm_go_sleep(0);
	}
	return 0;
}


void pm_do_poweroff(void)
{
	unsigned long start, flags;

	PWER &= ~0x8000ffff;
	local_irq_save(flags);
	printk("Poweroff, Begin to wait BP_RDY signal.\n");
	pm_send_all(PM_SUSPEND, (void *)3);

	start = OSCR;
	*(unsigned long *)(phys_to_virt(BPSIG_ADDR)) = NO_FLAG;
	do {
		if( !(GPLR(GPIO_BP_RDY) & GPIO_bit(GPIO_BP_RDY))){
			printk(KERN_DEBUG"got BP_RDY signal.\n");
			
			GPDR(GPIO_WDI_AP) |= GPIO_bit(GPIO_WDI_AP);
			GPCR(GPIO_WDI_AP) = GPIO_bit(GPIO_WDI_AP);
			while(1);
		}

		if ((OSCR - start) >= POWER_OFF_TIMEOUT) {
			printk(KERN_DEBUG "timeout when power down\n");
			mdelay(1);
			GPDR(GPIO_WDI_AP) |= GPIO_bit(GPIO_WDI_AP);
			GPCR(GPIO_WDI_AP) = GPIO_bit(GPIO_WDI_AP);
			while(1);
		}
		
	} while(1);
					
	pm_do_useroff();
	printk(KERN_DEBUG "resume from useroff\n");
	pm_send_all(PM_SUSPEND, (void *)0);
	local_irq_restore(flags);	
}

static ssize_t do_read(struct file *fp, char *buf, size_t count, loff_t *ppos)
{
	struct apm_user *	as;
	int			i;
	apm_event_t		event;
	DECLARE_WAITQUEUE(wait, current);

	as = fp->private_data;
	if (check_apm_user(as, "read"))
		return -EIO;
	if (count < sizeof(apm_event_t))
		return -EINVAL;
	if (queue_empty(as)) {
		if (fp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		add_wait_queue(&apm_waitqueue, &wait);

repeat:
		set_current_state(TASK_INTERRUPTIBLE);
		if (queue_empty(as) && !signal_pending(current)) {
			schedule();
			goto repeat;
		}
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&apm_waitqueue, &wait);
	}
	i = count;
	while ((i >= sizeof(event)) && !queue_empty(as)) {
		event = get_queued_event(as);

		if (copy_to_user(buf, &event, sizeof(event))) {
			if (i < count)
				break;
			return -EFAULT;
		}
		buf += sizeof(event);
		i -= sizeof(event);
	}
	if (i < count)
		return count - i;
	if (signal_pending(current))
		return -ERESTARTSYS;
	return 0;
}

static unsigned int do_poll(struct file *fp, poll_table * wait)
{
	struct apm_user * as;

	as = fp->private_data;
	if (check_apm_user(as, "poll"))
		return 0;
	poll_wait(fp, &apm_waitqueue, wait);
	if (!queue_empty(as))
		return POLLIN | POLLRDNORM;
	return 0;
}


void  (*pipm_start_pmu)(void) = NULL;
EXPORT_SYMBOL(pipm_start_pmu);

static int do_ioctl(struct inode * inode, struct file *filp,
		    u_int cmd, u_long arg)
{
	struct apm_user *	as;
	struct pm_dev *		pm;
	struct ipm_config	conf;

	as = filp->private_data;
	if (check_apm_user(as, "ioctl"))
		return -EIO;

	memset(&conf, 0, sizeof(conf));

	switch (cmd) {
        case APM_IOC_SUSPEND:
		pm_do_suspend(CPUMODE_SLEEP);
		break;
        case APM_IOC_SET_WAKEUP:
		if ((pm = pm_find((pm_dev_t)arg,NULL)) == NULL)
			return -EINVAL;
		pm_send(pm,PM_SET_WAKEUP,NULL);
		break;
	case APM_IOC_SLEEP:
		pm_go_sleep(arg);
		break;
	case APM_IOC_SET_SPROF_WIN:
		sleep_to = arg * HZ;
		APM_DPRINTK("do_ioctl: sleep timeout %ld\n", arg);
		break;
	case APM_IOC_WAKEUP_ENABLE:
		PWER |= arg;
		APM_DPRINTK("do_ioctl: enable wakeup source:PWER=0x%x\n", PWER);
		break;
	case APM_IOC_WAKEUP_DISABLE:
		PWER &= ~arg;
		APM_DPRINTK("do_ioctl: disable wakeup source:PWER=0x%x\n", PWER);
		break;
	case APM_IOC_POWEROFF:
		APM_DPRINTK("do_ioctl: do power off\n");
		/* here all device should response ok */
		pm_send_all(PM_SUSPEND, (void *)3);
		pm_do_poweroff();
		pm_send_all(PM_RESUME, (void *)0);
		break;
	case APM_IOC_RESET_BP:
		APM_DPRINTK("do_ioctl: reset bp\n");
		GPCR(GPIO_BB_RESET) = GPIO_bit(GPIO_BB_RESET);
		mdelay(1);
		GPSR(GPIO_BB_RESET) = GPIO_bit(GPIO_BB_RESET);
		break;
	case APM_IOC_USEROFF_ENABLE:
		APM_DPRINTK("do_ioctl: useroff support enable\n");
		user_off_available = (int)arg;
		break;
	case APM_IOC_NOTIFY_BP:
                break;
        case APM_IOC_REFLASH:
                cpu_proc_fin();
                *(unsigned long *)(phys_to_virt(FLAG_ADDR)) = REFLASH_FLAG;

//		power_ic_periph_set_usb_pull_up(0);
//LIN           mdelay(1000);
//LIN let GPIO control the connectivity
#include <linux/power_ic.h>
		set_GPIO_mode(GPIO_USB_READY|GPIO_OUT);
		clr_GPIO(GPIO_USB_READY);
		power_ic_periph_set_usb_pull_up(0);
		mdelay(10);
		power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL,19,1);//LIN set USB_CNTRL(bit19) to disable emu control
		mdelay(1000);
//LIN		power_ic_periph_set_usb_pull_up(1);

                /* Initialize the watchdog and let it fire */
                OWER = OWER_WME;
                OSSR = OSSR_M3;
		OSMR3 = OSCR + CLOCK_TICK_RATE/100;	/* ... in 10 ms */
                MDREFR |= MDREFR_SLFRSH;
		while(1);
                break;
        case APM_IOC_PASSTHRU:
                cpu_proc_fin();
                *(unsigned long *)(phys_to_virt(FLAG_ADDR)) = PASS_THRU_FLAG;

		power_ic_periph_set_usb_pull_up(0);
                mdelay(1000);
		power_ic_periph_set_usb_pull_up(1);

                /* Initialize the watchdog and let it fire */
                OWER = OWER_WME;
                OSSR = OSSR_M3;
		OSMR3 = OSCR + CLOCK_TICK_RATE/100;	/* ... in 10 ms */
                MDREFR |= MDREFR_SLFRSH;
		while(1);
                break;
	case APM_IOC_SET_IPROF_WIN:
		/* set profile window size */
		break;
	case APM_IOC_STARTPMU:
		if( pipm_start_pmu !=NULL )
			pipm_start_pmu();
		break;
	case APM_IOC_GET_IPM_CONFIG:
		get_ipm_config(&conf);
		return (copy_to_user((void *)arg, &conf,sizeof(conf)))? -EFAULT:0;
		break;
	case APM_IOC_SET_IPM_CONFIG:
		if(copy_from_user(&conf,(void *)arg,sizeof(conf)))
			return -EFAULT;
		
		return set_ipm_config(&conf);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int do_release(struct inode * inode, struct file * filp)
{
	struct apm_user *	as;

	as = filp->private_data;
	if (check_apm_user(as, "release"))
		return 0;
	filp->private_data = NULL;
	lock_kernel();

	/*
	 * by zxf, 11/11/2002
	 * free memory malloced when open correctly
	 */
	if (user_list == as)
		user_list = as->next;
	else {
		struct apm_user *	as1;

		for (as1 = user_list;
		     (as1 != NULL) && (as1->next != as);
		     as1 = as1->next)
			;
		if (as1 == NULL)
			printk(KERN_ERR "apm: filp not in user list\n");
		else
			as1->next = as->next;
	}

	unlock_kernel();
	APM_DPRINTK("do_release: free mem at 0x%x\n", (unsigned int)as);
	kfree(as);
	return 0;
}

static int do_open(struct inode * inode, struct file * filp)
{
	struct apm_user *as;
	if (user_list != NULL)
		return -EBUSY;

	as = (struct apm_user *)kmalloc(sizeof(*as), GFP_KERNEL);
	APM_DPRINTK("do_open: malloc mem at 0x%x\n", (unsigned int)as);
	if (as == NULL) {
		printk(KERN_ERR "apm: cannot allocate struct of size %d bytes\n",
		       sizeof(*as));
		return -ENOMEM;
	}
	as->magic = APM_BIOS_MAGIC;
	as->event_tail = as->event_head = 0;
	as->suspends_pending = as->standbys_pending = 0;
	as->suspends_read = as->standbys_read = 0;
	/*
	 * XXX - this is a tiny bit broken, when we consider BSD
         * process accounting. If the device is opened by root, we
	 * instantly flag that we used superuser privs. Who knows,
	 * we might close the device immediately without doing a
	 * privileged operation -- cevans
	 */
	as->suser = capable(CAP_SYS_ADMIN);
	as->next = user_list;
	user_list = as;
	filp->private_data = as;

	return 0;
}

static int apm_get_info(char *buf, char **start, off_t fpos, int length)
{
	char *		p;
	unsigned short	dx;
	unsigned short	error;
	unsigned char   ac_line_status = 0xff;
	unsigned char   battery_status = 0xff;
	unsigned char   battery_flag   = 0xff;
        unsigned char   percentage     = 0xff;
	int             time_units     = -1;
	char            *units         = "?";

	p = buf;

	if ((smp_num_cpus == 1) &&
	    !(error = apm_get_power_status(&ac_line_status,
                                           &battery_status, &battery_flag, &percentage, &dx))) {
		if (apm_bios_info.version > 0x100) {
			if (dx != 0xffff) {
				units = (dx & 0x8000) ? "min" : "sec";
				time_units = dx & 0x7fff;
			}
		}
	}
	/* Arguments, with symbols from linux/apm_bios.h.  Information is
	   from the Get Power Status (0x0a) call unless otherwise noted.

	   0) Linux driver version (this will change if format changes)
	   1) APM BIOS Version.  Usually 1.0, 1.1 or 1.2.
	   2) APM flags from APM Installation Check (0x00):
	      bit 0: APM_16_BIT_SUPPORT
	      bit 1: APM_32_BIT_SUPPORT
	      bit 2: APM_IDLE_SLOWS_CLOCK
	      bit 3: APM_BIOS_DISABLED
	      bit 4: APM_BIOS_DISENGAGED
	   3) AC line status
	      0x00: Off-line
	      0x01: On-line
	      0x02: On backup power (BIOS >= 1.1 only)
	      0xff: Unknown
	   4) Battery status
	      0x00: High
	      0x01: Low
	      0x02: Critical
	      0x03: Charging
	      0x04: Selected battery not present (BIOS >= 1.2 only)
	      0xff: Unknown
	   5) Battery flag
	      bit 0: High
	      bit 1: Low
	      bit 2: Critical
	      bit 3: Charging
	      bit 7: No system battery
	      0xff: Unknown
	   6) Remaining battery life (percentage of charge):
	      0-100: valid
	      -1: Unknown
	   7) Remaining battery life (time units):
	      Number of remaining minutes or seconds
	      -1: Unknown
	   8) min = minutes; sec = seconds */

	p += sprintf(p, "%s %d.%d 0x%02x 0x%02x 0x%02x 0x%02x %d%% %d %s\n",
		     driver_version,
		     (apm_bios_info.version >> 8) & 0xff,
		     apm_bios_info.version & 0xff,
		     apm_bios_info.flags,
		     ac_line_status,
		     battery_status,
		     battery_flag,
		     percentage,
		     time_units,
		     units);

	return p - buf;
}

#ifndef MODULE
static int __init apm_setup(char *str)
{
	int	invert;

	while ((str != NULL) && (*str != '\0')) {
		if (strncmp(str, "off", 3) == 0)
			apm_disabled = 1;
		if (strncmp(str, "on", 2) == 0)
			apm_disabled = 0;
		invert = (strncmp(str, "no-", 3) == 0);
		if (invert)
			str += 3;
		if (strncmp(str, "debug", 5) == 0)
			debug = !invert;
		if ((strncmp(str, "power-off", 9) == 0) ||
		    (strncmp(str, "power_off", 9) == 0))
			power_off = !invert;
		str = strchr(str, ',');
		if (str != NULL)
			str += strspn(str, ", \t");
	}
	return 1;
}

__setup("apm=", apm_setup);
#endif


static struct file_operations apm_bios_fops = {
	owner:		THIS_MODULE,
	read:		do_read,
	poll:		do_poll,
	ioctl:		do_ioctl,
	open:		do_open,
	release:	do_release,
};
static struct miscdevice apm_device = {
	APM_MINOR_DEV,
	"apm_bios",
	&apm_bios_fops
};

#define APM_INIT_ERROR_RETURN	return -1

/*
 * Just start the APM thread. We do NOT want to do APM BIOS
 * calls from anything but the APM thread, if for no other reason
 * than the fact that we don't trust the APM BIOS. This way,
 * most common APM BIOS problems that lead to protection errors
 * etc will have at least some level of being contained...
 *
 * In short, if something bad happens, at least we have a choice
 * of just killing the apm thread..
 */
static int __init apm_init(void)
{
	if (apm_bios_info.version == 0) {
		printk(KERN_INFO "apm: BIOS not found.\n");
		APM_INIT_ERROR_RETURN;
	}
	printk(KERN_INFO
		"apm: BIOS version %d.%d Flags 0x%02x (Driver version %s)\n",
		((apm_bios_info.version >> 8) & 0xff),
		(apm_bios_info.version & 0xff),
		apm_bios_info.flags,
		driver_version);

	if (apm_disabled) {
		printk(KERN_NOTICE "apm: disabled on user request.\n");
		APM_INIT_ERROR_RETURN;
	}

	if (PM_IS_ACTIVE()) {
		printk(KERN_NOTICE "apm: overridden by ACPI.\n");
		APM_INIT_ERROR_RETURN;
	}
	pm_active = 1;

	create_proc_info_entry("apm", 0, NULL, apm_get_info);

	misc_register(&apm_device);
	pm_power_off = pm_do_poweroff;

	clear_dpcsr();
	return 0;
}

static void __exit apm_exit(void)
{
	misc_deregister(&apm_device);
	remove_proc_entry("apm", NULL);
	if (power_off)
		pm_power_off = NULL;
	exit_kapmd = 1;
	while (kapmd_running)
		schedule();
	pm_active = 0;
}

module_init(apm_init);
module_exit(apm_exit);

MODULE_AUTHOR("Jamey Hicks, pulling bits from original by Stephen Rothwell");
MODULE_DESCRIPTION("A minimal emulation of APM");
MODULE_PARM(debug, "i");
MODULE_PARM_DESC(debug, "Enable debug mode");
MODULE_PARM(power_off, "i");
MODULE_PARM_DESC(power_off, "Enable power off");

EXPORT_NO_SYMBOLS;

