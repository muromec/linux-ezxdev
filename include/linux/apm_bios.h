#ifndef _LINUX_APM_H
#define _LINUX_APM_H

/*
 * Include file for the interface to an APM BIOS
 * Copyright 1994-2001 Stephen Rothwell (sfr@canb.auug.org.au)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 * modified by w20535, for EZX platform
 */

/* #include <linux/pm-devices.h> */

#if !defined(CONFIG_ARCH_EZX)
typedef unsigned short	apm_event_t;
typedef unsigned short	apm_eventinfo_t;
#endif

#ifdef __KERNEL__

#define APM_40		0x40
#define APM_CS		(APM_40 + 8)
#define APM_CS_16	(APM_CS + 8)
#define APM_DS		(APM_CS_16 + 8)

struct apm_bios_info {
	unsigned short	version;
	unsigned short	cseg;
	unsigned long	offset;
	unsigned short	cseg_16;
	unsigned short	dseg;
	unsigned short	flags;
	unsigned short	cseg_len;
	unsigned short	cseg_16_len;
	unsigned short	dseg_len;
};

/* Results of APM Installation Check */
#define APM_16_BIT_SUPPORT	0x0001
#define APM_32_BIT_SUPPORT	0x0002
#define APM_IDLE_SLOWS_CLOCK	0x0004
#define APM_BIOS_DISABLED	  	0x0008
#define APM_BIOS_DISENGAGED	 0x0010

/*
 * Data for APM that is persistant across module unload/load
 */
struct apm_info {
	struct apm_bios_info	bios;
	unsigned short		connection_version;
	int			get_power_status_broken;
	int			get_power_status_swabinminutes;
	int			allow_ints;
	int			realmode_power_off;
	int			disabled;
};

/*
 * The APM function codes
 */
#define	APM_FUNC_INST_CHECK	0x5300
#define	APM_FUNC_REAL_CONN	0x5301
#define	APM_FUNC_16BIT_CONN	0x5302
#define	APM_FUNC_32BIT_CONN	0x5303
#define	APM_FUNC_DISCONN	0x5304
#define	APM_FUNC_IDLE		0x5305
#define	APM_FUNC_BUSY		0x5306
#define	APM_FUNC_SET_STATE	0x5307
#define	APM_FUNC_ENABLE_PM	0x5308
#define	APM_FUNC_RESTORE_BIOS	0x5309
#define	APM_FUNC_GET_STATUS	0x530a
#define	APM_FUNC_GET_EVENT	0x530b
#define	APM_FUNC_GET_STATE	0x530c
#define	APM_FUNC_ENABLE_DEV_PM	0x530d
#define	APM_FUNC_VERSION	0x530e
#define	APM_FUNC_ENGAGE_PM	0x530f
#define	APM_FUNC_GET_CAP	0x5310
#define	APM_FUNC_RESUME_TIMER	0x5311
#define	APM_FUNC_RESUME_ON_RING	0x5312
#define	APM_FUNC_TIMER		0x5313

/*
 * Function code for APM_FUNC_RESUME_TIMER
 */
#define	APM_FUNC_DISABLE_TIMER	0
#define	APM_FUNC_GET_TIMER	1
#define	APM_FUNC_SET_TIMER	2

/*
 * Function code for APM_FUNC_RESUME_ON_RING
 */
#define	APM_FUNC_DISABLE_RING	0
#define	APM_FUNC_ENABLE_RING	1
#define	APM_FUNC_GET_RING	2

/*
 * Function code for APM_FUNC_TIMER_STATUS
 */
#define	APM_FUNC_TIMER_DISABLE	0
#define	APM_FUNC_TIMER_ENABLE	1
#define	APM_FUNC_TIMER_GET	2

/*
 * in arch/i386/kernel/setup.c
 */
extern struct apm_info	apm_info;

#endif	/* __KERNEL__ */

/*
 * Power states
 */
#define APM_STATE_READY		0x0000
#define APM_STATE_STANDBY	0x0001
#define APM_STATE_SUSPEND	0x0002
#define APM_STATE_OFF		0x0003
#define APM_STATE_BUSY		0x0004
#define APM_STATE_REJECT	0x0005
#define APM_STATE_OEM_SYS	0x0020
#define APM_STATE_OEM_DEV	0x0040

#define APM_STATE_DISABLE	0x0000
#define APM_STATE_ENABLE	0x0001

#define APM_STATE_DISENGAGE	0x0000
#define APM_STATE_ENGAGE	0x0001

/*
 * Events (results of Get PM Event)
 */
#define APM_SYS_STANDBY		0x0001
#define APM_SYS_SUSPEND		0x0002
#define APM_NORMAL_RESUME	0x0003
#define APM_CRITICAL_RESUME	0x0004
#define APM_LOW_BATTERY		0x0005
#define APM_POWER_STATUS_CHANGE	0x0006
#define APM_UPDATE_TIME		0x0007
#define APM_CRITICAL_SUSPEND	0x0008
#define APM_USER_STANDBY	0x0009
#define APM_USER_SUSPEND	0x000a
#define APM_STANDBY_RESUME	0x000b
#define APM_CAPABILITY_CHANGE	0x000c

/*
 * Error codes
 */
#define APM_SUCCESS		0x00
#define APM_DISABLED		0x01
#define APM_CONNECTED		0x02
#define APM_NOT_CONNECTED	0x03
#define APM_16_CONNECTED	0x05
#define APM_16_UNSUPPORTED	0x06
#define APM_32_CONNECTED	0x07
#define APM_32_UNSUPPORTED	0x08
#define APM_BAD_DEVICE		0x09
#define APM_BAD_PARAM		0x0a
#define APM_NOT_ENGAGED		0x0b
#define APM_BAD_FUNCTION	0x0c
#define APM_RESUME_DISABLED	0x0d
#define APM_NO_ERROR		0x53
#define APM_BAD_STATE		0x60
#define APM_NO_EVENTS		0x80
#define APM_NOT_PRESENT		0x86

/*
 * APM Device IDs
#define APM_DEVICE_BIOS		0x0000
#define APM_DEVICE_ALL		0x0001
#define APM_DEVICE_DISPLAY	0x0100
#define APM_DEVICE_STORAGE	0x0200
#define APM_DEVICE_PARALLEL	0x0300
#define APM_DEVICE_SERIAL	0x0400
#define APM_DEVICE_NETWORK	0x0500
#define APM_DEVICE_PCMCIA	0x0600
#define APM_DEVICE_BATTERY	0x8000
#define APM_DEVICE_OEM		0xe000
#define APM_DEVICE_OLD_ALL	0xffff
#define APM_DEVICE_CLASS	0x00ff
#define APM_DEVICE_MASK		0xff00
 */
/*
 *  APM devices IDs for non-x86
 */
#define APM_DEVICE_ALL		PM_SYS_DEV
#define APM_DEVICE_DISPLAY	  PM_DISPLAY_DEV
#define APM_DEVICE_STORAGE	PM_STORAGE_DEV
#define APM_DEVICE_PARALLEL	PM_PARALLEL_DEV
#define APM_DEVICE_SERIAL	PM_SERIAL_DEV
#define APM_DEVICE_NETWORK	PM_NETWORK_DEV
#define APM_DEVICE_PCMCIA	PM_PCMCIA_DEV
#define APM_DEVICE_BATTERY	PM_BATTERY_DEV
#define APM_DEVICE_TPANEL	PM_TPANEL_DEV


#ifdef __KERNEL__
/*
 * This is the "All Devices" ID communicated to the BIOS
 */
#define APM_DEVICE_BALL		((apm_info.connection_version > 0x0100) ? \
				 APM_DEVICE_ALL : APM_DEVICE_OLD_ALL)
#endif

/*
 * Battery status
 */
#define APM_MAX_BATTERIES	2

/*
 * APM defined capability bit flags
 */
#define APM_CAP_GLOBAL_STANDBY		0x0001
#define APM_CAP_GLOBAL_SUSPEND		0x0002
#define APM_CAP_RESUME_STANDBY_TIMER	0x0004 /* Timer resume from standby */
#define APM_CAP_RESUME_SUSPEND_TIMER	0x0008 /* Timer resume from suspend */
#define APM_CAP_RESUME_STANDBY_RING	0x0010 /* Resume on Ring fr standby */
#define APM_CAP_RESUME_SUSPEND_RING	0x0020 /* Resume on Ring fr suspend */
#define APM_CAP_RESUME_STANDBY_PCMCIA	0x0040 /* Resume on PCMCIA Ring	*/
#define APM_CAP_RESUME_SUSPEND_PCMCIA	0x0080 /* Resume on PCMCIA Ring	*/

/*
 * ioctl operations
 */
#include <linux/ioctl.h>

#define APM_IOC_STANDBY		_IO('A', 1)
#define APM_IOC_SUSPEND		_IO('A', 2)
#define APM_IOC_SET_WAKEUP	_IO('A', 3)

#if	defined(CONFIG_ARCH_SA1100)
#define APM_AC_OFFLINE 0
#define APM_AC_ONLINE 1
#define APM_AC_BACKUP 2
#define APM_AC_UNKNOWN 0xFF

#define APM_BATTERY_STATUS_HIGH 0
#define APM_BATTERY_STATUS_LOW  1
#define APM_BATTERY_STATUS_CRITICAL 2
#define APM_BATTERY_STATUS_CHARGING 3
#define APM_BATTERY_STATUS_UNKNOWN 0xFF

#define APM_BATTERY_LIFE_UNKNOWN 0xFFFF
#define APM_BATTERY_LIFE_MINUTES 0x8000
#define APM_BATTERY_LIFE_VALUE_MASK 0x7FFF
#endif

#if defined(CONFIG_ARCH_EZX)
//#define APM_DEBUG

#ifdef APM_DEBUG
#define APM_DPRINTK(format, args...)	printk(format, ##args)
#else
#define APM_DPRINTK(format, args...)
#endif

struct  ipm_config {
	/*  Below  items must be set to set configurations. */
	unsigned int	core_freq;	/*  in khz. */
	unsigned int	core_vltg;	/*  in mV.  */
	unsigned int	turbo_ratio;	/*  specify the N value.	*/
	unsigned int	cpu_mode;
	unsigned int	fast_bus_mode;
	unsigned int	sys_bus_freq;
	unsigned int	mem_bus_freq;
	unsigned int	lcd_freq;
	unsigned int	enabled_device;
};

struct clk_regs {
	unsigned long cccr;
	unsigned long clkcfg;
	unsigned long mdrefr;
};

/*
 * kernel event definitions for our ezx platform
 */
typedef struct {
	unsigned short type;	/*	What type of IPM events.	*/
	unsigned short kind;	/*	What kind, or sub-type of events.*/
	unsigned int info;
} apm_event_t;

/* event types */
#define	APM_EVENT_PROFILER	0x0		/*	Profiler events.	*/
#define	APM_EVENT_PMU		0x1		/*	PMU events, may not need.	*/
#define	APM_EVENT_DEVICE	0x2		/*	Device event.	*/

/* event kinds */ 
#define	EVENT_PROF_IDLE		0x0		/* CPU utility */
#define	EVENT_PROF_PERF		0x1		/* bottleneck: mem or cpu */
#define EVENT_PROF_SLEEP	0x2		/* sleep condition */

#define EVENT_DEV_ACCS		0x0
#define EVENT_DEV_BT		0x1
#define EVENT_DEV_TS		0x2
#define EVENT_DEV_KEY		0x3
#define EVENT_DEV_RTC		0x4
#define EVENT_DEV_FLIP		0x5
#define EVENT_DEV_ICL		0x6
#define EVENT_DEV_BBWDI		0x7

/* event info */
#define PERF_CPU_BOUND		0x1
#define PERF_MEM_BOUND		0x2

#define DEV_OFF			0x0
#define DEV_ON			0x1

#define DEV_DETACH              0x0
#define DEV_ATTACH              0x1

/*
 * some new io control commands
 */
#define APM_IOC_GET_IPM_CONFIG		_IOR('A', 4, struct ipm_config)
#define APM_IOC_SET_IPM_CONFIG		_IOW('A', 5, struct ipm_config)
#define APM_IOC_SLEEP			_IOW('A', 6, int)
#define APM_IOC_SET_SPROF_WIN		_IOW('A', 7, int)
#define APM_IOC_WAKEUP_ENABLE		_IOW('A', 8, int)
#define APM_IOC_WAKEUP_DISABLE		_IOW('A', 9, int)
#define APM_IOC_POWEROFF		_IO('A', 10)
#define APM_IOC_RESET_BP		_IO('A', 11)
#define APM_IOC_USEROFF_ENABLE		_IOW('A', 12, int)
#define APM_IOC_NOTIFY_BP		_IOW('A', 13, int)
#define APM_IOC_REFLASH			_IO('A', 14)
#define APM_IOC_PASSTHRU		_IO('A', 15)
#define	APM_IOC_STARTPMU		_IOW('A', 16, int)
#define	APM_IOC_SET_IPROF_WIN		_IOW('A', 17, int)

/* parameters passed to APM_IOC_NOTIFY_BP in order to signal BP */
#define APM_NOTIFY_BP_QUIET		1
#define APM_NOTIFY_BP_UNHOLD		0


#define FCS_EOF			/* do FCS in EOF handler */
#ifdef FCS_EOF
#define WAIT_MORE_EOF		/* do FCS in next EOF if this EOF handler was delayed too long */
#endif
//#define FCS_WITH_MEM

extern void apm_event_notify(short type, short kind, int info);
extern int periodic_jobs_done(void);

extern int get_core_voltage(void);
extern int enter_13M_quickly(void);
extern void exit_13M_quickly(int mode);

extern int get_ipm_config(struct ipm_config  *ret_conf);
extern int set_ipm_config(struct ipm_config  *set_conf);

#endif /* CONFIG_ARCH_EZX */


/*
 * Maximum number of events stored
 */
#define APM_MAX_EVENTS		  20

/*
 * The per-file APM data
 */
struct apm_user {
	int		magic;
	struct apm_user *next;
	int		suser: 1;
	int		suspend_wait: 1;
	int		suspend_result;
	int		suspends_pending;
	int		standbys_pending;
	int		suspends_read;
	int		standbys_read;
	int		event_head;
	int		event_tail;
	apm_event_t	events[APM_MAX_EVENTS];
};

#endif	/* LINUX_APM_H */
