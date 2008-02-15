/*
 * Copyright (C) 2005 Motorola 
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * 2005-June-1  Created for FOTA system call, Wang Jordan
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <asm/semaphore.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/fota_syscall.h>

#include <asm/arch/ezx.h>
#include <asm/hardware.h>
#include <linux/delay.h>
#include <asm/arch-pxa/timex.h>



extern void *sys_call_table[];

/* until we get a permanent syscall number */
#ifdef __NR_SYSCALL_BASE
static int sys_fota_req_nr = __NR_sys_fota_req - __NR_SYSCALL_BASE;
#else
static int sys_fota_req_nr = __NR_sys_fota_req;
#endif

//#undef FOTA_REQ_TRACE
#define FOTA_REQ_TRACE

#if defined(FOTA_REQ_TRACE)
#define trace(args...) do { printk("FOTA_REQ_TRACE: "); printk(args); } while(0)
#define FOTA_SYSCALL_DEBUG 1
#else
#define trace(args...) do {} while(0)
#endif
 
static int foda_req_reboot_ap_handler(void);
static int foda_req_reboot_bp_handler(void);
static int foda_req_connect_opt1_opt2_handler(void);
static int foda_req_disconnect_opt1_opt2_handler(void);
/*
 *   0 : sucessful
 *   -1: failed.
 */
static int foda_req_reboot_ap_handler(void)
{
   // handler for reboot ap in fota mode
   /* Initialize the watchdog and let it fire */
	OWER = OWER_WME;
	OSSR = OSSR_M3;
	OSMR3 = OSCR + CLOCK_TICK_RATE/100;	/* ... in 10 ms */
//        MDREFR |= MDREFR_SLFRSH;   need to review this code with Xiaofan
	while(1);
	return 0;  // just to cheat compiler
}


/*
 *   0 : sucessful
 *   -1: failed.
 */
static int foda_req_reboot_bp_handler(void)
{
    //handle for reboot bp in normal mode and fota mode
	trace("foda_req_reboot bp.\n");
	GPCR(GPIO_BB_RESET) = GPIO_bit(GPIO_BB_RESET);
	mdelay(1);
	GPSR(GPIO_BB_RESET) = GPIO_bit(GPIO_BB_RESET);
	trace("reset bp done.\n");
	return 0;
}

/*
 *   0 : sucessful
 *   -1: failed.
 */
static int foda_req_connect_opt1_opt2_handler(void)
{
    //handle for shorting opt1 and opt2 to ensure that BP can run blob when rebooting.
    GPDR(GPIO_BB_FLASHMODE_EN) |= GPIO_bit(GPIO_BB_FLASHMODE_EN);
    GPSR(GPIO_BB_FLASHMODE_EN) = GPIO_bit(GPIO_BB_FLASHMODE_EN);
    trace("foda_req_connect_opt1_opt2 done.\n");
    return 0;
}

/*
 *   0 : sucessful
 *   -1: failed.
 */
static int foda_req_disconnect_opt1_opt2_handler(void)
{
	//handle for disconnecting opt1 and opt2 to ensure that BP can not run blob when rebooting
	GPDR(GPIO_BB_FLASHMODE_EN) |= GPIO_bit(GPIO_BB_FLASHMODE_EN);
	GPCR(GPIO_BB_FLASHMODE_EN) = GPIO_bit(GPIO_BB_FLASHMODE_EN);	
	trace("foda_req_disconnect_opt1_opt2 done.\n");
	return 0;
}
asmlinkage int sys_fota_req(fota_req_t fota_req)
{
#if 0
	if (current->uid != 0)
		return -EACCES;
	trace("fota_syscall has been called.\n");
	if(params)
	{
		if(0 != copy_from_user(&p, params, sizeof(int)))
			return -EFAULT;
	}
#endif
	#ifdef FOTA_SYSCALL_DEBUG
	printk("fota_syscall_para %d.\n", fota_req);
	#endif
	switch(fota_req) {
	case FODA_REQ_REBOOT_AP:
		return foda_req_reboot_ap_handler();
	case FODA_REQ_REBOOT_BP:
		 return foda_req_reboot_bp_handler();
	case FODA_REQ_CONNECT_OPT1_OPT2:
		 return foda_req_connect_opt1_opt2_handler();
	case FODA_REQ_DISCONNECT_OPT1_OPT2:
		 return foda_req_disconnect_opt1_opt2_handler();
	default:
		return -EBADRQC;
	}
}
/****************************************************************************
 * install fota syscall  support
 ****************************************************************************/
int fota_req_init_module(void)
{
	trace("fota_req_init_module enter.\n");
	/* ensure syscall slot is available */
	if (sys_call_table[sys_fota_req_nr] != sys_call_table[0])
		return 1;
	/* install our syscall */
	sys_call_table[sys_fota_req_nr] = sys_fota_req;
	trace("fota_req_init_module finished.\n");
	return 0;
}

void fota_req_exit_module(void)
{
	trace("fota_req_exit_module entered.\n");
	/* uninstall our syscall */
	sys_call_table[sys_fota_req_nr] = sys_call_table[0];
	trace("fota_req_exit_module finished.\n");
}

#ifdef MODULE
module_init(fota_req_init_module);
module_exit(fota_req_exit_module);
#else
__initcall(fota_req_init_module);
#endif
