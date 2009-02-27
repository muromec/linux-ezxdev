/*
 *	Real Time Clock interface for Linux on StrongARM SA1100
 *	and XScale PXA270/250/210.
 *
 *	Copyright (c) 2000 Nils Faerber
 *
 *	Based on rtc.c by Paul Gortmaker
 *	Date/time conversion routines taken from arch/arm/kernel/time.c
 *			by Linus Torvalds and Russel King
 *		and the GNU C Library
 *	( ... I love the GPL ... just take what you need! ;)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *        1.05      2005-03-18         Yin Kangkai <e12051@motorola.com>
 *        - add handling of RTC Periodic
 *
 *        1.04      2005-01-18         Yin Kangkai <e12051@motorola.com>
 *        - add handling of RTC Stopwatch
 * 
 *	1.03	2004-01-12	Zhuang Xiaofan <w19962@motorola.com>
 *	- add queue apm event
 *
 *      1.02    2002-07-15      Andrew Christian <andrew.christian@hp.com>
 *      - added pm_ routines to shut off extraneous interrupts while asleep
 *
 *	1.01	2002-07-09	Nils Faerber <nils@kernelconcepts.de>
 *	- fixed rtc_poll() so that select() now works
 *
 *	1.00	2001-06-08	Nicolas Pitre <nico@cam.org>
 *	- added periodic timer capability using OSMR1
 *	- flag compatibility with other RTC chips
 *	- permission checks for ioctls
 *	- major cleanup, partial rewrite
 *
 *	0.03	2001-03-07	CIH <cih@coventive.com>
 *	- Modify the bug setups RTC clock.
 *
 *	0.02	2001-02-27	Nils Faerber <nils@kernelconcepts.de>
 *	- removed mktime(), added alarm irq clear
 *
 *	0.01	2000-10-01	Nils Faerber <nils@kernelconcepts.de>
 *	- initial release
 */
/*
 * Copyright (C) 2004-2005 Motorola Inc.
 *
 * modified by e12051, for EZX platform
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/rtc.h>
#include <linux/pm.h>
#include <linux/apm_bios.h>
#include <linux/delay.h>

#include <asm/bitops.h>
#include <asm/hardware.h>
#include <asm/irq.h>

#define	DRIVER_VERSION		"1.05"
/* #define DEBUG */
/* #define DEBUG_TMPDEV */

/*
 * Due to the Stopwatch bug, it can not get 100HZ resolution.
 * To fix this, we use Periodic Timer (1K HZ resolution) when
 * the user request interval is small than 65s. While when the
 * user request interval > 65s, we assume that the user can 
 * tolerate the Stopwatch 1HZ relolution and uses Stopwatch Timer.
 */

#define TIMER_FREQ		CLOCK_TICK_RATE

#define RTC_DEF_DIVIDER		32768 - 1
#define RTC_DEF_TRIM		0

/* Those are the bits from a classic RTC we want to mimic */
#define RTC_IRQF		0x80	/* any of the following 3 is active */
#define RTC_PF			0x40
#define RTC_AF			0x20
#define RTC_UF			0x10


/* request status */
#define APP_SLEEP_READY                 (1 << 0)
#define REQUEST_IN_USE                  (1 << 1)
#define APP_ALARM_READY                 (1 << 2)

/* Periodic RTCPICR max */
#define RTCPICR_MAX                     ((1 << 16) - 1)

/* SWAR max */
#define SWAR_MAX  (\
                  99 |\
                  59 << HUNDREDTHS_BITS |\
                  59 << (HUNDREDTHS_BITS + SECONDS_BITS) |\
                  31 << (HUNDREDTHS_BITS + SECONDS_BITS + MINUTES_BITS)\
                  )

/* RTC Stopwatch register related micros */
#define HUNDREDTHS_BITS                 7
#define SECONDS_BITS                    6
#define MINUTES_BITS                    6
#define HOURS_BITS                      5
#define HUNDREDTHS_MASK                 0x00007f
#define SECONDS_MASK                    0x001f80
#define MINUTES_MASK                    0x07e000
#define HOURS_MASK                      0xf80000
#define HUNDREDTHS(x)   ((x) & HUNDREDTHS_MASK)
#define SECONDS(x)      (((x) & SECONDS_MASK) >> HUNDREDTHS_BITS)
#define MINUTES(x)      (((x) & MINUTES_MASK) >> (SECONDS_BITS + HUNDREDTHS_BITS))
#define HOURS(x)        (((x) & HOURS_MASK) >> (MINUTES_BITS + SECONDS_BITS + HUNDREDTHS_BITS))

/* the app's alarm request */
struct rtc_sw_request {
        int status;                     /* status of this request & app */
        int type;                       /* strict or fuzzy timer */
        struct task_struct *process;    /* the process */
        
        /* Both store in hundredths. */
        unsigned long interval;         /* alarm interval */
        unsigned long remain;           /* remain value of interval */

        struct rtc_sw_request *prev;    /* prev request */
        struct rtc_sw_request *next;    /* next request */
};

/* INTERRUPT SOURCE */
typedef enum {
        INT_STOPWATCH = 0,
        INT_PERIODIC
} INT_SOURCE_E;

/* CALLER SOURCE */
typedef enum {
        CALLER_ISR = 0,
        CALLER_IOCTL
} CALL_SOURCE_E;

/* timer type */
enum {
        RTC_SW_STRICT = 0,
        RTC_SW_FUZZ
};

/* defined in arm/mach-ezx/apm.c */
extern void apm_event_notify(short type, short kind, int info);
extern int periodic_jobs_done(void);

static struct rtc_sw_request *rtc_sw_request_head = NULL;

static unsigned long rtc_status;
static unsigned long rtc_irq_data;
static unsigned long rtc_freq = 1024;
#ifdef DEBUG
static unsigned long sw_oscr_new = 0, sw_oscr_old = 0;
#endif

static struct fasync_struct *rtc_async_queue;
static DECLARE_WAIT_QUEUE_HEAD(rtc_wait);
static DECLARE_WAIT_QUEUE_HEAD(rtc_sw_wait);

extern spinlock_t rtc_lock;
static spinlock_t request_lock;

static const unsigned char days_in_mo[] =
	{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

#define is_leap(year) \
	((year) % 4 == 0 && ((year) % 100 != 0 || (year) % 400 == 0))


#if 1 /* MVL-CEE */
#ifdef CONFIG_DPM
#include <linux/device.h>

static int pxartc_suspend(struct device *dev, u32 state, u32 level);
static int pxartc_resume(struct device *dev, u32 level);

static struct device_driver pxartc_driver_ldm = {
	name:          "pxa-rtc",
	devclass:      NULL,
	probe:         NULL,
	suspend:       pxartc_suspend,
	resume:        pxartc_resume,
	remove:        NULL,
	constraints:   NULL,
};

static struct device pxartc_device_ldm = {
	name:         "PXA Real Time Clock",
	bus_id:       "pxartc",
	driver:       NULL,
	power_state:  DPM_POWER_ON,
};

static void
pxartc_ldm_register(void)
{
	extern void pxaopb_driver_register(struct device_driver *driver);
	extern void pxaopb_device_register(struct device *device);
	
	pxaopb_driver_register(&pxartc_driver_ldm);
	pxaopb_device_register(&pxartc_device_ldm);
}

static void
pxartc_ldm_unregister(void)
{
	extern void pxaopb_driver_unregister(struct device_driver *driver);
	extern void pxaopb_device_unregister(struct device *device);
	
	pxaopb_device_unregister(&pxartc_device_ldm);
	pxaopb_driver_unregister(&pxartc_driver_ldm);
}

static int
pxartc_resume(struct device *dev, u32 level)
{
	unsigned int rtsr = RTSR;

	switch (level) {
	case RESUME_POWER_ON:
		/* +++: the PM resume below checks if the RTC is
		   suspended before resuming--does that imply that
		   there is a problem if we just try to resume here
		   even if we're not suspended? */
		RTSR = 0;
		RTSR = RTSR_HZ;
		RTSR = rtsr & (RTSR_HZE|RTSR_ALE);
		enable_irq(IRQ_OST1);
		enable_irq(IRQ_RTCAlrm);
		enable_irq(IRQ_RTC1Hz);
		break;
	}
	
	return 0;
}

static int
pxartc_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		/* +++: is there a need to indicate that the RTC is
		   suspended? */
		disable_irq(IRQ_OST1);
		disable_irq(IRQ_RTCAlrm);
		disable_irq(IRQ_RTC1Hz);
		break;
	}
	
	return 0;
}
#endif
#endif /* MVL-CEE */

/* Return interval in 1/100 sec */
static inline unsigned long 
sw_oscr_interval(unsigned long begin, unsigned long end)
{
        /* OSCR0 uses 3.25MHz clock */
        return ((end - begin) / (CLOCK_TICK_RATE / 100));
}

/*
 * Converts seconds since 1970-01-01 00:00:00 to Gregorian date.
 */

static void decodetime (unsigned long t, struct rtc_time *tval)
{
	long days, month, year, rem;

	days = t / 86400;
	rem = t % 86400;
	tval->tm_hour = rem / 3600;
	rem %= 3600;
	tval->tm_min = rem / 60;
	tval->tm_sec = rem % 60;
	tval->tm_wday = (4 + days) % 7;

#define LEAPS_THRU_END_OF(y) ((y)/4 - (y)/100 + (y)/400)

	year = 1970 + days / 365;
	days -= ((year - 1970) * 365
			+ LEAPS_THRU_END_OF (year - 1)
			- LEAPS_THRU_END_OF (1970 - 1));
	if (days < 0) {
		year -= 1;
		days += 365 + is_leap(year);
	}
	tval->tm_year = year - 1900;
	tval->tm_yday = days + 1;

	month = 0;
	if (days >= 31) {
		days -= 31;
		month++;
		if (days >= (28 + is_leap(year))) {
			days -= (28 + is_leap(year));
			month++;
			while (days >= days_in_mo[month]) {
				days -= days_in_mo[month];
				month++;
			}
		}
	}
	tval->tm_mon = month;
	tval->tm_mday = days + 1;
}


/* 
 * convert from rtc_sw_time to hundredths.
 * PREDICTION: rtc_sw_time checked.
 * 
 * returns 0 on success, -1 on error.
 */
static int
convert_to_hundredths(struct rtc_sw_time *rtime, unsigned long *hundredths)
{
        if (rtime == NULL)
                return -1;

        *hundredths = (rtime->hours * 360000) +
                (rtime->minutes * 6000) +
                (rtime->seconds * 100) +
                (rtime->hundredths);
        return 0;
}


/* 
 * convert from hundredths to rtc_sw_time 
 *
 * returns 0 on success, -1 on error.
 */
static int 
convert_from_hundredths(unsigned long hundredths, struct rtc_sw_time *rtime)
{
        if (rtime == NULL)
                return -1;

        rtime->hours = hundredths / 360000;
        hundredths %= (3600 * 100);

        rtime->minutes = hundredths / 6000;
        hundredths %= (60 * 100);

        rtime->seconds = hundredths / 100;
        hundredths %= 100;

        rtime->hundredths = hundredths;

        return 0;
}


/*
 * The app alarm request related functions
 * The request is a double linked list.
 */

/* 
 * create one request struct
 */
static struct rtc_sw_request * request_create(void)
{
        struct rtc_sw_request *req;
        
        req = (struct rtc_sw_request *)
                kmalloc(sizeof(struct rtc_sw_request), GFP_KERNEL);
        if (req == NULL) {
                printk(KERN_WARNING "request malloc failed.\n");
                return NULL;
        }

        return req;
}

/* 
 * free the request 
 */
static void request_free(struct rtc_sw_request *req)
{
        if (req == NULL) {
                printk(KERN_WARNING "try to free a NULL request.\n");
                return;
        }

        kfree(req);
        return;
}

/* 
 * init the request struct
 */
static inline void request_init(struct rtc_sw_request *request)
{
        if (request == NULL) {
                printk("<%s>: try to init NULL req.\n", __FUNCTION__);
                return;
        }
        memset(request, 0, sizeof(struct rtc_sw_request));

        return;
}

/*
 * Add request into the request list.
 * Just intert into the head.
 */
static void request_add(struct rtc_sw_request *req)
{
        req->prev = NULL;
        req->next = NULL;
        
        if (rtc_sw_request_head == NULL) {
                rtc_sw_request_head = req;
        } else {
                rtc_sw_request_head->prev = req;
                req->next = rtc_sw_request_head;
                rtc_sw_request_head = req;
        }

        return;
}

/*
 * Del request from the list.
 * We have two methods:
 * request_del_time: del request from the list according to the time 
 *                   (hundredths).
 * request_del_process: del request from the list according to the process.
 */

/*
 * detach_from_list: an inline routing used by request_del_time and
 * request_del_process. 
 * NOTE!!: we assume that the caller has grabbed the lock.
 */
static void inline detach_from_list(struct rtc_sw_request *req)
{
        if (req == NULL) {
                printk(KERN_WARNING "Trying to del NULL request.\n");
                return;
        }
        
        /* I'm the only one... */
        if ((req->prev == NULL) && (req->next == NULL)) {
                rtc_sw_request_head = NULL;
                return;
        }

        /*
         * At least two request nodes.
         */
        
        /* I'm in the first place... */
        if ((req->prev == NULL) && (req->next != NULL)) {
                rtc_sw_request_head = req->next;
                req->next->prev = NULL;
        }

        /* I'm in the last place... */
        if ((req->prev != NULL) && (req->next == NULL)) {
                req->prev->next = NULL;
        }

        /* I'm in the middle of the list */
        if ((req->prev != NULL) && (req->next != NULL)) {
                req->prev->next = req->next;
                req->next->prev = req->prev;
        }

        req->next = NULL;
        req->prev = NULL;

        return;
}
            
/*
 * request_del_time:
 * Detach the request from the list according to
 * the given time (hundredths), and return the request.
 * If no request found, return NULL.
 *
 * NOTE!!: we assume that the caller has grabbed the lock.
 */
static struct rtc_sw_request * request_del_time(unsigned long time)
{
        struct rtc_sw_request * req;

        if (time == 0)
                return NULL;

        req = rtc_sw_request_head;
        if (req == NULL) {
                return NULL;
        }

        while (req != NULL) {
                if ((req->process == current) && (req->interval == time)) {
#ifdef DEBUG
                        printk("del request<interval:%ld>.\n", time);
#endif
                        detach_from_list(req);
                        goto found;
                }
                req = req->next;
        }

 found:
        return req;
}

/*
 * request_del_process:
 * Detach the request from the list and return the request.
 * If no request found, return NULL.
 *
 * NOTE!!: we assume that the caller has grabbed the lock.
 */
static struct rtc_sw_request * request_del_process(struct task_struct *p)
{
        struct rtc_sw_request *req = NULL;
        
        if (p == NULL)
                return NULL;

        req = rtc_sw_request_head;
        if (req == NULL) {
                return NULL;
        }
        
        while (req != NULL) {
                if (req->process == p) {
#ifdef DEBUG
                        printk("del request<process>.\n");
#endif
                        detach_from_list(req);
                        goto found;
                }
                req = req->next;
        }

 found:
        return req;
}


/*
 * This function update one request.
 * Minus the hundredths that has past.
 * NOTE!!: we are holding the request lock.
 */
static void update_one_request(struct rtc_sw_request *req, unsigned long hun)
{
#ifdef DEBUG
        printk("<%s>: hundredths: %ld\n", __FUNCTION__, hun);
#endif
        if (req->remain <= hun) {
                /* wake up the process */
                wake_up_process(req->process);
                req->status &= ~APP_SLEEP_READY;
                req->status &= ~REQUEST_IN_USE;
                req->status |= APP_ALARM_READY;
                req->remain = req->interval;
        } else {
                req->remain -= hun;
        }

        return;
}
                

/*
 * This function update each of the request in the list,
 * decrease the hundredths that has past.
 * This routing is called by ioctl RTC_SW_SETTIME and the rtc_sw_interrupt.
 * NOTE!!: we are holding the request lock.
 */
static void update_request_list(unsigned long hundredths)
{
        struct rtc_sw_request *p = NULL;

#ifdef DEBUG
        printk("<%s>: hundredths: %ld\n",__FUNCTION__, hundredths);
#endif
        if (hundredths == 0)
                return;
        
        p = rtc_sw_request_head;
        while (p != NULL) {
                update_one_request(p, hundredths);
                p = p->next;
        }

        return;
}

/*
 * Get the request whose remain is the lowest.
 * return NULL if no request.
 *
 * NOTE: the caller is holding the lock.
 */
static struct rtc_sw_request * get_lowest_request(void)
{
        struct rtc_sw_request *req = NULL;
        struct rtc_sw_request *select = NULL;

        req = rtc_sw_request_head;
        
        /* get the initial select */
        if (req != NULL) {
                select = req;
                req = req->next;
        }

        /* walk through the list */
        while (req != NULL) {
                if (req->remain < select->remain) {
                        select = req;
                }
                req = req->next;
        }

        return select;
}
                
        
/* 
 * rtc_sw_time check 
 */
static int rtc_sw_time_check(struct rtc_sw_time *rtime)
{
        int hr = rtime->hours;
        int mt = rtime->minutes;
        int sd = rtime->seconds;
        int hs = rtime->hundredths;
        
        if ((hr < 0 || hr > 23) ||
            (mt < 0 || mt > 59) ||
            (sd < 0 || sd > 59) ||
            (hs < 0 || hs > 99))
                return -1;

        /* we don't support interval <= 1 */
        if ((hr == 0) && (mt == 0) && (sd == 0) && (hs <= 1))
                return -1;
        
        return 0;
}

/*
 * Get RTC Stopwatch SWCR and transfer it into hundredths
 */
static unsigned long get_rtc_sw_reg(void)
{
        struct rtc_sw_time rt;
        unsigned long value;
        unsigned long swcr = SWCR;
        
        rt.hours = HOURS(swcr);
        rt.minutes = MINUTES(swcr);
        rt.seconds = SECONDS(swcr);
        rt.hundredths = HUNDREDTHS(swcr);

        if (convert_to_hundredths(&rt, &value) != 0)
                return 0;

        return value;
}


/*
 * Get RTC Periodic corresponding register and transfer it into hundredths
 */
static unsigned long get_rtc_pi_reg(CALL_SOURCE_E caller)
{
        unsigned long value = 0;

#ifdef DEBUG
        printk("RTCPICR: %ld, PIAR: %ld\n", RTCPICR, PIAR);
#endif
        if (caller == CALLER_ISR)
                value = PIAR;
        else 
                value = RTCPICR;

        /* convert it to hundredths */
        if ((value % 10) >= 5)
                return (value / 10 + 1);
        else
                return (value / 10);
}


/*
 * Set value to the RTC Stopwatch SWAR1.
 *
 * Input: value (hundredths)
 * Output: set SWAR1 and
 *      return -1 if failed.
 */
static int set_rtc_sw_reg(unsigned long value)
{
        struct rtc_sw_time rt;
        unsigned long swar = 0;
        
        if (value == 0)
                return -1;
        if (convert_from_hundredths(value, &rt) != 0)
                return -1;

        swar |= (rt.hundredths |
                 rt.seconds << HUNDREDTHS_BITS |
                 rt.minutes << (HUNDREDTHS_BITS + SECONDS_BITS) |
                 rt.hours << (HUNDREDTHS_BITS + SECONDS_BITS + MINUTES_BITS));

#ifdef DEBUG
        printk("<%s>: set SWAR1: %08x\n",__FUNCTION__, swar);
#endif
        SWAR1 = swar;
#ifdef DEBUG
        printk("after set SWAR1, SWCR: 0x%x\n", SWCR);
        sw_oscr_old = OSCR;
#endif

        /**
         * From Intel spec:
         * 21.4.6.2
         * RTC Recoery from Standby, Sleep, and Deep Sleep Modes
         * 
         * If RTSR[SWCE] is set to one and a match between SWCR and SWAR2 
         * occurs. Wake-up of the Bulverde processor occurs *REGARDLESS* of 
         * the state of the RTSR[SWALE2] bit.
         * ...
         * So I just set SWAR2 to a LARGE value here to prevent SWAR2 from 
         * waking up the Bulverde.
         */
        SWAR2 = SWAR_MAX;

        return 0;
}

/*
 * Set value to the RTC Periodic PIAR.
 *
 * Input: value (hundredths)
 * Output: set PIAR and
 *      return -1 if failed.
 */
static int set_rtc_pi_reg(unsigned long value)
{
        if (value == 0)
                return -1;

#ifdef DEBUG
        printk("<%s>: set PIAR: %08x\n",__FUNCTION__, value * 10);
#endif
        PIAR = value * 10;

        /* seems that the RTCPICR doesn't reset to 0 when you set PIAR, 
         * so I do this by hand. 
         */
        udelay(1);
        RTCPICR = 0;

#ifdef DEBUG
        printk("<%s>: after set PIAR, RTCPICR: %08x\n", __FUNCTION__, RTCPICR);
        sw_oscr_old = OSCR;
#endif

        return 0;
}


/*
 * Get time slice past.
 * return the time past (hundredths).
 *
 * NOTE: we assume the caller has grabed the lock.
 */
static unsigned long get_slice_past(INT_SOURCE_E int_src, CALL_SOURCE_E caller)
{
        unsigned long past = 0;

        if (rtc_sw_request_head == NULL)
                return 0;
        /* 
         * If not NULL, we should get register's value and
         * update the request
         */
        if (int_src == INT_STOPWATCH)
                past = get_rtc_sw_reg();
        else
                past = get_rtc_pi_reg(caller);

        return past;
}


/*
 * When the app exit normally or abnormally, this function must be called
 * since we should delete the request nodes the app have applied.
 * 
 * Called when a app exit.
 */
static void rtc_sw_app_exit(void)
{
        struct rtc_sw_request *req = NULL;
        
        spin_lock_irq(&request_lock);

        /* remove all the request that belongs to this process */
        while ((req = request_del_process(current)) != NULL) {
                spin_unlock_irq(&request_lock);
                request_free(req);
                spin_lock_irq(&request_lock);
        }
        
        spin_unlock_irq(&request_lock);

        return;
}


/*
 * The interrupt handler.
 * Currently, we handle the RTC Timer Alarm (RTSR_AL), HZ (RTSR_HZ),
 * Stopwatch Alarm 1 (RTSR_SWAL1) and Periodic interrupt.
 * 
 * comments by zq: if 1HZ interrupt enabled (HZE=1), it always enabled until clear it by rtc_ioctl.
 *                 the rtc_interrupt handler will not clear HZE bit.
 */
static void rtc_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
        unsigned long slice;
        unsigned int rtsr= RTSR;
        int ret;

#ifdef DEBUG
        sw_oscr_new = OSCR;
        printk("<ISR>RTSR: %08x, SWCR: %08x, RTCPICR: %08x, oscr past: %lu\n", 
                RTSR, SWCR, RTCPICR, sw_oscr_interval(sw_oscr_old, sw_oscr_new));
#endif
        /* RTC Periodic or Stopwatch interrupt detected */
        if ((rtsr & RTSR_PIAL) || (rtsr & RTSR_SWAL1)) {
                struct rtc_sw_request *req;
                
                /* clear interrupt source, and stop counting */
                if (rtsr & RTSR_PIAL) {
                        RTSR |= RTSR_PIAL;
                        RTSR &= ~RTSR_PICE;
                } else {
                        RTSR |= RTSR_SWAL1;
                        RTSR &= ~RTSR_SWCE;
                }

                /* grab request lock, be careful */
                spin_lock(&request_lock);

                /* update the list */
                if (rtsr & RTSR_PIAL)
                        slice = get_slice_past(INT_PERIODIC, CALLER_ISR);
                else
                        slice = get_slice_past(INT_STOPWATCH, CALLER_ISR);

                if (slice == 0) {
#ifdef DEBUG
                        printk(KERN_WARNING "rtc interrupt, past slice 0.\n");
#endif
                }
                update_request_list(slice);

                /* select the request whose remain is the smallest. */
                req = get_lowest_request();
                if (req == NULL) {
#ifdef DEBUG
                        printk(KERN_WARNING "No request in the list.\n");
#endif
                        RTSR &= ~(RTSR_PICE | RTSR_PIALE | 
                                  RTSR_SWCE | RTSR_SWALE1);
                } else {
                        if ((req->remain * 10) > RTCPICR_MAX) {
                                ret = set_rtc_sw_reg(req->remain);
                                udelay(1);
                                RTSR |= (RTSR_SWCE | RTSR_SWALE1);
                        } else {
                                ret = set_rtc_pi_reg(req->remain);
                                udelay(1);
                                RTSR |= (RTSR_PICE | RTSR_PIALE);
                        }

                        req->status |= REQUEST_IN_USE;
                }

                spin_unlock(&request_lock);
        }

        /* RTC HZ or ALARM detected */
        if ((rtsr & RTSR_AL) || (rtsr & RTSR_HZ)) {
                /* clear interrupt sources */
                RTSR |= (RTSR_AL | RTSR_HZ);

                /* clear alarm interrupt if it has occurred */
                if (rtsr & RTSR_AL)
                        RTSR &= ~RTSR_ALE;
		
                /* update irq data & counter */
                if (rtsr & RTSR_AL) {
		        printk("RTC ALARM detected, rtsr=0x%x, RTSR=0x%x\n", rtsr, RTSR);
                        rtc_irq_data |= (RTC_AF|RTC_IRQF);
                        apm_event_notify(APM_EVENT_DEVICE, EVENT_DEV_RTC, 0);
                }
                if (rtsr & RTSR_HZ) {
		        printk("RTC 1HZ detected, rtsr=0x%x, RTSR=0x%x\n", rtsr, RTSR);
                        rtc_irq_data |= (RTC_UF | RTC_IRQF);
                }
                
                rtc_irq_data += 0x100;

                /* wake up waiting process */
                wake_up_interruptible(&rtc_wait);
                kill_fasync (&rtc_async_queue, SIGIO, POLL_IN);
        }

        return;

}

static void timer1_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	/*
	 * If we match for the first time, the periodic interrupt flag won't
	 * be set.  If it is, then we did wrap around (very unlikely but
	 * still possible) and compute the amount of missed periods.
	 * The match reg is updated only when the data is actually retrieved
	 * to avoid unnecessary interrupts.
	 */
	OSSR = OSSR_M1;	/* clear match on timer1 */
	if (rtc_irq_data & RTC_PF) {
		rtc_irq_data += (rtc_freq * ((1<<30)/(TIMER_FREQ>>2))) << 8;
	} else {
		rtc_irq_data += (0x100|RTC_PF|RTC_IRQF);
	}

	wake_up_interruptible(&rtc_wait);
	kill_fasync (&rtc_async_queue, SIGIO, POLL_IN);
}

static int rtc_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit (1, &rtc_status))
		return -EBUSY;
	rtc_irq_data = 0;
	return 0;
}

/* 
 * rtc stopwatch open function
 * NOTE: we must support multi open 
 */
static int rtc_sw_open(struct inode *inode, struct file *file)
{
        /*
         * Nothing special here.
         * We do accept multiple open at the same time.
         */
#ifdef DEBUG
        printk("RTC Stopwatch opened.\n");
#endif
        return 0;
}

static int rtc_release(struct inode *inode, struct file *file)
{
	spin_lock_irq (&rtc_lock);
	RTSR |= (RTSR_AL|RTSR_HZ);
	OIER &= ~OIER_E1;
	OSSR = OSSR_M1;
	spin_unlock_irq (&rtc_lock);
	rtc_status = 0;
	return 0;
}

/* 
 * rtc stopwatch release function 
 *
 * When the user close the fd, we delete all the request
 * the user applied.
 */
static int rtc_sw_release(struct inode *inode, struct file *file)
{
        rtc_sw_app_exit();
        
        return 0;
}

static int rtc_fasync (int fd, struct file *filp, int on)
{
	return fasync_helper (fd, filp, on, &rtc_async_queue);
}

static unsigned int rtc_poll(struct file *file, poll_table *wait)
{
	poll_wait (file, &rtc_wait, wait);
	return (rtc_irq_data) ? (POLLIN | POLLRDNORM) : 0;
}

/* rtc stopwatch poll */
static unsigned int rtc_sw_poll(struct file *file, poll_table *wait)
{
        struct rtc_sw_request * req = NULL;
        int data = 0;

        
        /* wait for wake_up_process */
        poll_wait (file, &rtc_sw_wait, wait);

        /* check the APP_ALARM_READY bit */
        spin_lock_irq(&request_lock);

        req = rtc_sw_request_head;
        while (req != NULL) {
                if ((req->process == current) &&
                    (req->status & APP_ALARM_READY)) {
                        /* clear the ALARM ready bit */
                        req->status &= ~APP_ALARM_READY;
                        data = 1;
                        
                        /* break the while */
                        break; 
                }
                req = req->next;
        }

        spin_unlock_irq(&request_lock);

#ifdef DEBUG
        printk("<%s>: data: %d\n",__FUNCTION__, data);
#endif
        
        return (data) ? (POLLIN | POLLRDNORM) : 0;
}

static loff_t rtc_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}

/* rtc stopwatch llseek */
static loff_t rtc_sw_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}

ssize_t rtc_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long data;
	ssize_t retval;

	if (count < sizeof(unsigned long))
		return -EINVAL;

	add_wait_queue(&rtc_wait, &wait);

	for (;;) {
                set_current_state(TASK_INTERRUPTIBLE);
                
		spin_lock_irq (&rtc_lock);
		data = rtc_irq_data;
		if (data != 0) {
			rtc_irq_data = 0;
			break;
		}
		spin_unlock_irq (&rtc_lock);

		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			goto out;
		}

		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			goto out;
		}

		schedule();
	}

	if (data & RTC_PF) {
		/* interpolate missed periods and set match for the next one */
		unsigned long period = TIMER_FREQ/rtc_freq;
		unsigned long oscr = OSCR;
		unsigned long osmr1 = OSMR1;
		unsigned long missed = (oscr - osmr1)/period;
		data += missed << 8;
		OSSR = OSSR_M1;	/* clear match on timer 1 */
		OSMR1 = osmr1 + (missed + 1)*period;
		/* ensure we didn't miss another match in the mean time */
		while( (signed long)((osmr1 = OSMR1) - OSCR) <= 0 ) {
			data += 0x100;
			OSSR = OSSR_M1;	/* clear match on timer 1 */
			OSMR1 = osmr1 + period;
		}
	}
	spin_unlock_irq (&rtc_lock);

	data -= 0x100;	/* the first IRQ wasn't actually missed */

	retval = put_user(data, (unsigned long *)buf);
	if (!retval)
		retval = sizeof(unsigned long);

out:
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&rtc_wait, &wait);
	return retval;
}

/* rtc stopwatch read */
ssize_t rtc_sw_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
        struct rtc_sw_request *req = NULL;
        unsigned long data = 0;
        int ret = 0;
        
        /* check the APP_ALARM_READY bit */
        spin_lock_irq(&request_lock);
        add_wait_queue(&rtc_sw_wait, &wait);
        
 repeat:
        set_current_state(TASK_INTERRUPTIBLE);

        req = rtc_sw_request_head;
        while (req != NULL) {
                if ((req->process == current) &&
                    (req->status & APP_ALARM_READY)) {
                        /* clear the ALARM ready bit */
                        req->status &= ~APP_ALARM_READY;
                        data = 1;
                        
                        /* break the while */
                        break; 
                }
                req = req->next;
        }

        if (data != 1) {
                /* FIX me: repeatly check this flag is unnecessary */
                if (file->f_flags & O_NONBLOCK) {
                        ret = -EAGAIN;
                        spin_unlock_irq(&request_lock);
                        goto out;
                }

                if (!signal_pending(current)) {
                        spin_unlock_irq(&request_lock);
                        schedule();
                        spin_lock_irq(&request_lock);
                        goto repeat;
                } else {
                        ret = -ERESTARTSYS;
                        spin_unlock_irq(&request_lock);
                        goto out;
                }
        }

        spin_unlock_irq(&request_lock);
        
	ret = put_user(data, (unsigned long *)buf);
	if (!ret)
		ret = sizeof(unsigned long);

 out:
        set_current_state(TASK_RUNNING);
        remove_wait_queue(&rtc_sw_wait, &wait);
        
        return ret;
}

static int rtc_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	struct rtc_time tm;

	switch (cmd) {
	case RTC_AIE_OFF:
		spin_lock_irq(&rtc_lock);
		RTSR &= ~RTSR_ALE;
		rtc_irq_data = 0;
		spin_unlock_irq(&rtc_lock);
		return 0;
	case RTC_AIE_ON:
		spin_lock_irq(&rtc_lock);
		RTSR |= RTSR_ALE;
		rtc_irq_data = 0;
		spin_unlock_irq(&rtc_lock);
		return 0;
	case RTC_UIE_OFF:
		spin_lock_irq(&rtc_lock);
		RTSR &= ~RTSR_HZE;
		rtc_irq_data = 0;
		spin_unlock_irq(&rtc_lock);
		return 0;
	case RTC_UIE_ON:
		spin_lock_irq(&rtc_lock);
		RTSR |= RTSR_HZE;
		rtc_irq_data = 0;
		spin_unlock_irq(&rtc_lock);
		return 0;
	case RTC_PIE_OFF:
		spin_lock_irq(&rtc_lock);
		OIER &= ~OIER_E1;
		rtc_irq_data = 0;
		spin_unlock_irq(&rtc_lock);
		return 0;
	case RTC_PIE_ON:
		if ((rtc_freq > 64) && !capable(CAP_SYS_RESOURCE))
			return -EACCES;
		spin_lock_irq(&rtc_lock);
		OSMR1 = TIMER_FREQ/rtc_freq + OSCR;
		OIER |= OIER_E1;
		rtc_irq_data = 0;
		spin_unlock_irq(&rtc_lock);
		return 0;
	case RTC_ALM_READ:
		decodetime (RTAR, &tm);
		break;
	case RTC_ALM_SET:
                if (copy_from_user (&tm, (struct rtc_time*)arg, sizeof (tm)))
	                        return -EFAULT;
                tm.tm_year += 1900;
                if (tm.tm_year < 1970 || (unsigned)tm.tm_mon >= 12 ||
		                    tm.tm_mday < 1 || tm.tm_mday > (days_in_mo[tm.tm_mon] +
			                                    (tm.tm_mon == 1 && is_leap(tm.tm_year))) ||
		                    (unsigned)tm.tm_hour >= 24 ||
		                    (unsigned)tm.tm_min >= 60 ||
		                    (unsigned)tm.tm_sec >= 60)
                            return -EINVAL;
                RTAR = mktime ( tm.tm_year, tm.tm_mon + 1, tm.tm_mday,
		                                tm.tm_hour, tm.tm_min, tm.tm_sec);
                return 0;		
	case RTC_RD_TIME:
		decodetime (RCNR, &tm);
		break;
	case RTC_SET_TIME:
		if (!capable(CAP_SYS_TIME))
			return -EACCES;
		if (copy_from_user (&tm, (struct rtc_time*)arg, sizeof (tm)))
			return -EFAULT;
		tm.tm_year += 1900;
		if (tm.tm_year < 1970 || (unsigned)tm.tm_mon >= 12 ||
		    tm.tm_mday < 1 || tm.tm_mday > (days_in_mo[tm.tm_mon] +
				(tm.tm_mon == 1 && is_leap(tm.tm_year))) ||
		    (unsigned)tm.tm_hour >= 24 ||
		    (unsigned)tm.tm_min >= 60 ||
		    (unsigned)tm.tm_sec >= 60)
			return -EINVAL;
		RCNR = mktime (	tm.tm_year, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec);
		return 0;
	case RTC_IRQP_READ:
		return put_user(rtc_freq, (unsigned long *)arg);
	case RTC_IRQP_SET:
		if (arg < 1 || arg > TIMER_FREQ)
			        return -EINVAL;
		if ((arg > 64) && (!capable(CAP_SYS_RESOURCE)))
			        return -EACCES;
		rtc_freq = arg;
		return 0;
	case RTC_EPOCH_READ:
		return put_user (1970, (unsigned long *)arg);
	default:
		return -EINVAL;
	}
	return copy_to_user ((void *)arg, &tm, sizeof (tm)) ? -EFAULT : 0;
}

static inline struct rtc_sw_request * get_fuzz_req(void)
{
        struct rtc_sw_request *req = NULL;
        struct rtc_sw_request *fuzz = NULL;

        spin_lock_irq(&request_lock);
        req = rtc_sw_request_head;
        while (req != NULL) {
                if (req->type == RTC_SW_FUZZ) {
                        fuzz = req;
                        /* break the while */
                        break; 
                }
                req = req->next;
        }
        spin_unlock_irq(&request_lock);

        return fuzz;
}


/* rtc stopwatch ioctl */
static int rtc_sw_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
        struct rtc_sw_request *req = NULL;
        struct rtc_sw_request *fuzz = NULL;
        struct rtc_sw_time tm;
        unsigned long hundredths;
        unsigned long slice;
        int notify = 0;
        int ret;

#ifdef DEBUG
        printk("ioctl rcv cmd: %x\n", cmd);
#endif

        switch (cmd) {
        case RTC_SW_SETTIME:
#ifdef DEBUG
                printk("ioctl rcv cmd RTC_SW_SETTIME.\n");
#endif                
                if (copy_from_user(&tm, (struct rtc_sw_time *)arg,
                                   sizeof(struct rtc_sw_time)))
                        return -EFAULT;

                if (rtc_sw_time_check(&tm) != 0) {
                        printk(KERN_WARNING "rtc stopwatch time invalid.\n");
                        return -EINVAL;
                }
                if (convert_to_hundredths(&tm, &hundredths) != 0)
                        return -EINVAL;

                req = request_create();
                if (req == NULL) {
                        printk(KERN_WARNING "request create failed.\n");
                        return -ENOMEM;
                }

                request_init(req);
                req->status |= APP_SLEEP_READY;
                req->type = RTC_SW_STRICT;
                req->process = current;
                req->interval = hundredths;
                req->remain = req->interval;

                /* grab the lock, and be careful. */
                spin_lock_irq(&request_lock);
                
                spin_lock_irq(&rtc_lock);
                if ((RTSR & RTSR_SWCE) && (RTSR & RTSR_SWALE1)) {
                        RTSR &= ~RTSR_SWCE;
                        slice = get_slice_past(INT_STOPWATCH, CALLER_IOCTL);
                } else if ((RTSR & RTSR_PICE) && (RTSR & RTSR_PIALE)) {
                        RTSR &= ~RTSR_PICE;
                        slice = get_slice_past(INT_PERIODIC, CALLER_IOCTL);
                } else 
                        slice = 0;
                spin_unlock_irq(&rtc_lock);

                /*
                 * Before we insert the request to the list,
                 * updade the list.
                 */
                if (slice > 0)
                        update_request_list(slice);
                request_add(req);

                /* re-schedule :) */
                req = get_lowest_request();
                if (req == NULL) {
                        printk(KERN_WARNING "Can't be, no request in the list.\n");
                        ret = -EINVAL;
                } else {
                        /* operate on the hardware rtc RTSR */
                        spin_lock_irq(&rtc_lock);

                        if (req->remain > RTCPICR_MAX) {
                                ret = set_rtc_sw_reg(req->remain);
                                udelay(1);
                                RTSR |= (RTSR_SWALE1 | RTSR_SWCE);
                        } else {
                                ret = set_rtc_pi_reg(req->remain);
                                udelay(1);
                                RTSR |= (RTSR_PIALE | RTSR_PICE);
                        }

                        req->status |= REQUEST_IN_USE;
#ifdef DEBUG
                        printk("%s: RTSR=%08x\n",
                               __FUNCTION__, RTSR);
#endif                        
                        spin_unlock_irq(&rtc_lock);
                }
                
                spin_unlock_irq(&request_lock);

                return ret;
        case RTC_SW_SETTIME_FUZZ:
                /* 
                 * Use this type of request to sync with two or more
                 * applications. For example: 
                 *      app1 apply a 1 min request at the time of 10s;
                 *      app2 apply a 1 min request at the time of 30s;
                 * Without sync, the Bulverde should wake up at every
                 * 10s and 30s of one minute, consuming more power.
                 *
                 * We try to sync this two apps by align both request to
                 * 1 minute, i.e., both of this two request will start 
                 * right at next minute. With their interval Untouched.
                 */

                if (copy_from_user(&tm, (struct rtc_sw_time *)arg,
                                   sizeof(struct rtc_sw_time)))
                        return -EFAULT;

                if (rtc_sw_time_check(&tm) != 0) {
                        printk(KERN_WARNING "rtc stopwatch time invalid.\n");
                        return -EINVAL;
                }
                if (convert_to_hundredths(&tm, &hundredths) != 0)
                        return -EINVAL;

                req = request_create();
                if (req == NULL) {
                        printk(KERN_WARNING "request create failed.\n");
                        return -ENOMEM;
                }

                request_init(req);
                req->status |= APP_SLEEP_READY;
                req->type = RTC_SW_FUZZ;
                req->process = current;
                req->interval = hundredths;

                /* grab the lock, and be careful. */
                spin_lock_irq(&request_lock);
                
                spin_lock_irq(&rtc_lock);
                if ((RTSR & RTSR_SWCE) && (RTSR & RTSR_SWALE1)) {
                        RTSR &= ~RTSR_SWCE;
                        slice = get_slice_past(INT_STOPWATCH, CALLER_IOCTL);
                } else if ((RTSR & RTSR_PICE) && (RTSR & RTSR_PIALE)) {
                        RTSR &= ~RTSR_PICE;
                        slice = get_slice_past(INT_PERIODIC, CALLER_IOCTL);
                } else 
                        slice = 0;
                spin_unlock_irq(&rtc_lock);

                /*
                 * Before we insert the request to the list,
                 * updade the list.
                 */
                if (slice > 0)
                        update_request_list(slice);

                /* check if we already have fuzzy req in the list or not */
                fuzz = get_fuzz_req();
                if (fuzz != NULL) {
                        /* sync with the prev fuzzy req */
                        req->remain = fuzz->remain % req->interval;
                } else {
                        /* FIX me: how about xtime changed later? */
                        /* align to next minute */
#define XTIME_REMAIN (60 * 100 - \
			((xtime.tv_sec % 60) * 100 + xtime.tv_usec / 10000))
                        req->remain = XTIME_REMAIN % req->interval;
                }

                request_add(req);

                /* re-schedule :) */
                req = get_lowest_request();
                if (req == NULL) {
                        printk(KERN_WARNING "Can't be, no request in the list.\n");
                        ret = -EINVAL;
                } else {
                        /* operate on the hardware rtc RTSR */
                        spin_lock_irq(&rtc_lock);

                        if (req->remain > RTCPICR_MAX) {
                                ret = set_rtc_sw_reg(req->remain);
                                udelay(1);
                                RTSR |= (RTSR_SWALE1 | RTSR_SWCE);
                        } else {
                                ret = set_rtc_pi_reg(req->remain);
                                udelay(1);
                                RTSR |= (RTSR_PIALE | RTSR_PICE);
                        }

                        req->status |= REQUEST_IN_USE;
#ifdef DEBUG
                        printk("%s: RTSR=%08x\n",
                               __FUNCTION__, RTSR);
#endif                        
                        spin_unlock_irq(&rtc_lock);
                }
                
                spin_unlock_irq(&request_lock);

                return ret;
        case RTC_SW_DELTIME:
#ifdef DEBUG
                printk("ioctl rcv cmd RTC_SW_DELTIME.\n");
#endif                

                if (copy_from_user(&tm, (struct rtc_sw_time *)arg,
                                   sizeof(struct rtc_sw_time)))
                        return -EFAULT;

                if (rtc_sw_time_check(&tm) != 0) {
                        printk(KERN_WARNING "rtc stopwatch time invalid.\n");
                        return -EINVAL;
                }
                if (convert_to_hundredths(&tm, &hundredths) != 0)
                        return -EINVAL;
                
                /*
                 * Remove all the request that belongs to this process and
                 * matchs the given time.
                 */
                ret = -EINVAL;
                spin_lock_irq(&request_lock);
                while ((req = request_del_time(hundredths)) != NULL) {
                        spin_unlock_irq(&request_lock);
                        request_free(req);
                        spin_lock_irq(&request_lock);
                        ret = 0;
                }
                spin_unlock_irq(&request_lock);

                return ret;
        case RTC_SW_JOB_DONE:
#ifdef DEBUG
                printk("ioctl rcv cmd RTC_SW_JOB_DONE.\n");
#endif                
                
                spin_lock_irq(&request_lock);

                /* set the APP_SLEEP_READY status */
                req = rtc_sw_request_head;
                while (req != NULL) {
                        if (req->process == current) {
                                req->status |= APP_SLEEP_READY;
                        }
                        req = req->next;
                }

                /* may we send a sleep msg to DPM? */
                notify = 1;
                req = rtc_sw_request_head;

                while (req != NULL) {
                        if (!(req->status & APP_SLEEP_READY)) {
                                notify = 0;
                                break;
                        }
                        req = req->next;
                }

                if (notify == 1) {
                        /* notify DPM */
                        (void) periodic_jobs_done();
                }

                spin_unlock_irq(&request_lock);
                
                return 0;
        case RTC_SW_APP_EXIT:
#ifdef DEBUG
                printk("ioctl rcv cmd RTC_SW_APP_EXIT.\n");
#endif                
                rtc_sw_app_exit();

                return 0;
        default:
                return -EINVAL;
        }
        
}


static struct file_operations rtc_fops = {
	owner:		THIS_MODULE,
	llseek:		rtc_llseek,
	read:		rtc_read,
	poll:		rtc_poll,
	ioctl:		rtc_ioctl,
	open:		rtc_open,
	release:	rtc_release,
	fasync:		rtc_fasync,
};

/* rtc stopwatch fops */
static struct file_operations rtc_sw_fops = {
	owner:          THIS_MODULE,
	llseek:         rtc_sw_llseek,
	read:           rtc_sw_read,
	poll:           rtc_sw_poll,
	ioctl:          rtc_sw_ioctl,
	open:           rtc_sw_open,
	release:        rtc_sw_release,
};


static struct miscdevice sa1100rtc_miscdev = {
	RTC_MINOR,
	"rtc",
	&rtc_fops
};

static struct miscdevice sa1100rtc_sw_miscdev = {
	RTC_SW_MINOR,
	"rtc_sw",
	&rtc_sw_fops
};

static int rtc_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *p = page;
	int len;
	struct rtc_time tm;

	decodetime (RCNR, &tm);
	p += sprintf(p, "rtc_time\t: %02d:%02d:%02d\n"
			"rtc_date\t: %04d-%02d-%02d\n"
			"rtc_epoch\t: %04d\n",
			tm.tm_hour, tm.tm_min, tm.tm_sec,
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, 1970);
	decodetime (RTAR, &tm);
	p += sprintf(p, "alrm_time\t: %02d:%02d:%02d\n"
			"alrm_date\t: %04d-%02d-%02d\n",
			tm.tm_hour, tm.tm_min, tm.tm_sec,
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
	p += sprintf(p, "trim/divider\t: 0x%08x\n", RTTR);
	p += sprintf(p, "alarm_IRQ\t: %s\n", (RTSR & RTSR_ALE) ? "yes" : "no" );
	p += sprintf(p, "update_IRQ\t: %s\n", (RTSR & RTSR_HZE) ? "yes" : "no");
	p += sprintf(p, "periodic_IRQ\t: %s\n", (OIER & OIER_E1) ? "yes" : "no");
	p += sprintf(p, "periodic_freq\t: %ld\n", rtc_freq);

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

/* let's use the write interface for modifying RTTR */
static int rtc_write_proc(struct file *file, const char *buffer,
			  unsigned long count, void *data)
{
	if (count > 15)
		return -EINVAL;
	if (count > 0) {
		char lbuf[16];
		if (copy_from_user(lbuf, buffer, count))
			return -EFAULT;
		lbuf[count] = 0;
		RTTR = simple_strtoul(lbuf, NULL, 0);
	}
	return count;
}

static int rtc_setup_interrupts(void)
{
	int ret;

	ret = request_irq (IRQ_RTC1Hz, rtc_interrupt, SA_INTERRUPT, "rtc 1Hz", NULL);
	if (ret) {
		printk ("rtc: IRQ %d already in use.\n", IRQ_RTC1Hz);
		goto IRQ_RTC1Hz_failed;
	}
	ret = request_irq (IRQ_RTCAlrm, rtc_interrupt, SA_INTERRUPT, "rtc Alrm", NULL);
	if (ret) {
		printk("rtc: IRQ %d already in use.\n", IRQ_RTCAlrm);
		goto IRQ_RTCAlrm_failed;
	}
	ret = request_irq (IRQ_OST1, timer1_interrupt, SA_INTERRUPT, "rtc timer", NULL);
	if (ret) {
		printk("rtc: IRQ %d already in use.\n", IRQ_OST1);
		goto IRQ_OST1_failed;
	}

	return 0;

IRQ_OST1_failed:
	free_irq (IRQ_RTCAlrm, NULL);
IRQ_RTCAlrm_failed:
	free_irq (IRQ_RTC1Hz, NULL);
IRQ_RTC1Hz_failed:
	return ret;
}

static void rtc_free_interrupts(void)
{
	free_irq (IRQ_OST1, NULL);
	free_irq (IRQ_RTCAlrm, NULL);
	free_irq (IRQ_RTC1Hz, NULL);
}

static struct pm_dev *rtc_pm_dev;

static int rtc_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	static int suspended = 0;
         
#ifdef DEBUG
	printk(__FUNCTION__ ": pm callback %d RTSR=%08x ICPR=%08x\n", req, RTSR, ICPR );
#endif

	switch (req) {
	case PM_SUSPEND: /* Enter D1-D3 */
		disable_irq(IRQ_OST1);
		disable_irq(IRQ_RTCAlrm);
		disable_irq(IRQ_RTC1Hz);
		suspended = 1;
                break;
	case PM_RESUME:  /* Enter D0 */
		if ( suspended ) {
			enable_irq(IRQ_OST1);
			enable_irq(IRQ_RTCAlrm);
			enable_irq(IRQ_RTC1Hz);
			suspended = 0;
		}
		break;
        }
        return 0;
}

static int __init rtc_init(void)
{
	struct proc_dir_entry *entry;
	int ret;

	ret = misc_register (&sa1100rtc_miscdev);
        if (ret != 0) {
                printk("rtc_init can't register rtc: %d\n", ret);
                return ret;
        }
        ret = misc_register(&sa1100rtc_sw_miscdev);
        if (ret != 0) {
                printk("rtc_init can't register rtc_sw: %d\n", ret);
                return ret;
        }
	entry = create_proc_entry ("driver/rtc", 0, 0);
	if (entry) {
		entry->read_proc = rtc_read_proc;
		entry->write_proc = rtc_write_proc;
	}
	ret = rtc_setup_interrupts();
	if (ret)
		goto IRQ_failed;

	printk (KERN_INFO "SA1100 Real Time Clock driver v" DRIVER_VERSION "\n");

	/*
	 * According to the manual we should be able to let RTTR be zero
	 * and then a default diviser for a 32.768KHz clock is used.
	 * Apparently this doesn't work, at least for my SA1110 rev 5.
	 * If the clock divider is uninitialized then reset it to the
	 * default value to get the 1Hz clock.
	 */
	if (RTTR == 0) {
		RTTR = RTC_DEF_DIVIDER + (RTC_DEF_TRIM << 16);
		printk (KERN_WARNING "rtc: warning: initializing default clock divider/trim value\n");
		/*  The current RTC value probably doesn't make sense either */
		RCNR = 0;
	}

	rtc_pm_dev = pm_register(PM_UNKNOWN_DEV, PM_SYS_UNKNOWN, rtc_pm_callback);
#ifdef CONFIG_DPM
	pxartc_ldm_register();
#endif /* CONFIG_DPM */
       
	return 0;

IRQ_failed:
	remove_proc_entry ("driver/rtc", NULL);
	misc_deregister(&sa1100rtc_miscdev);
        misc_deregister(&sa1100rtc_sw_miscdev);
	return ret;
}

static void __exit rtc_exit(void)
{
#ifdef CONFIG_DPM
	pxartc_ldm_unregister();
#endif /* CONFIG_DPM */ 
	pm_unregister(rtc_pm_dev);
	rtc_free_interrupts();
	remove_proc_entry ("driver/rtc", NULL);
	misc_deregister(&sa1100rtc_miscdev);
        misc_deregister(&sa1100rtc_sw_miscdev);
}

module_init(rtc_init);
module_exit(rtc_exit);

MODULE_AUTHOR("Nils Faerber <nils@@kernelconcepts.de>");
MODULE_DESCRIPTION("SA1100/PXA Realtime Clock Driver (RTC)");
EXPORT_NO_SYMBOLS;

