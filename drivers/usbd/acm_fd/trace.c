/*
 * usbd/acm_fd/trace.c
 *
 *      Copyright (c) 2004 Belcarra
 *
 * Adapted from earlier work:
 *      Copyright (c) 2002, 2003 Belcarra
 *      Copyright (c) 2002 Lineo
 *
 * By: 
 *      Stuart Lynne <sl@belcarra.com>, 
 *      Tom Rushworth <tbr@belcarra.com>, 
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

/*
 * Copyright (C) 2004 - Motorola
 *
 * 2004-Dec-06 - Change for EZXBASE By Zhao Liang <w20146@motorola.com>
 *
 */



#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <linux/proc_fs.h>
#include <linux/vmalloc.h>

#include <asm/atomic.h>
#include <asm/io.h>

#include <linux/proc_fs.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,6)
#define USE_ADD_DEL_TIMER_FOR_USBADDR_CHECK 1
#include <linux/timer.h>
#else
#undef USE_ADD_DEL_TIMER_FOR_USBADDR_CHECK
#include <linux/tqueue.h>
#endif

#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/cache.h>


#if defined(CONFIG_ARCH_SA1100) || defined (CONFIG_ARCH_PXA)
#include <asm/dma.h>
#include <asm/mach/dma.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/types.h>
#endif

#if defined(CONFIG_MIPS_AU1000) || defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_PB1100) || defined(CONFIG_MIPS_DB1100)
#include <asm/au1000.h>
#include <asm/au1000_dma.h>
#include <asm/mipsregs.h>
#endif

#if defined(CONFIG_ARCH_SAMSUNG)
#include <asm/arch/timers.h>
#include <asm/arch/hardware.h>
#endif

#if defined(CONFIG_ARCH_MX1ADS)
#include "dbmx1_bi/dbmx1.h"
#endif

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>

#include <usbd-chap9.h>
#include <usbd-mem.h>
#include <usbd.h>
#include <trace.h>


int acm_trace_first;
int acm_trace_last_read;
int acm_trace_next;
int acm_trace_total;
acm_trace_t *acm_traces;

extern int acm_interrupts;


#if defined(CONFIG_USBD_ACM_TRACE) && defined(CONFIG_PROC_FS)

acm_trace_t *ACM_TRACE_NEXT(char *fn, acm_trace_types_t acm_trace_type)
{
        acm_trace_t *p;
	unsigned long flags;

	// Get the next trace slot - this needs to be atomic.
	local_irq_save (flags);
        p = acm_traces + acm_trace_next;
#if defined(TRACE_MAX_IS_2N)
	acm_trace_next = (acm_trace_next + 1) & TRACE_MASK;
        if (acm_trace_next == acm_trace_first) {
		// We have wraparound, bump acm_trace_first
		if (acm_trace_first == acm_trace_last_read) {
			// We have to bump last read too.
			acm_trace_last_read = (acm_trace_last_read + 1) & TRACE_MASK;
		}
		acm_trace_first = (acm_trace_first + 1) & TRACE_MASK;
	}
#else
        if (TRACE_MAX <= ++acm_trace_next)
		acm_trace_next = 0;

        if (acm_trace_next == acm_trace_first) {
		// We have wrap around, bump acm_trace_first
		if (acm_trace_first == acm_trace_last_read) {
			// We have to bump last read too.
        		if (TRACE_MAX <= ++acm_trace_last_read)
				acm_trace_last_read = 0;
		}
        	if (TRACE_MAX <= ++acm_trace_first)
			acm_trace_first = 0;
        }
#endif
	acm_trace_total++;
	// End of next trace slot.
	local_irq_restore (flags);

#if defined(CONFIG_ARCH_SA1100) || defined (CONFIG_ARCH_PXA)
        p->oscr = OSCR;
#elif defined(CONFIG_MIPS_AU1000) || defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_PB1100) || defined(CONFIG_MIPS_DB1100)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,19)
        p->cp0_count = __read_32bit_c0_register(CP0_COUNT);
#else
        p->cp0_count = read_c0_count();
#endif
#elif defined(CONFIG_ARCH_SAMSUNG)
        //p->jiffies = jiffies;
        //p->tcnt0 = *(volatile u32 *)TCNT0;
        p->tcnt1 = *(volatile u32 *)TCNT1;
#else
        p->jiffies = jiffies;
#endif
#if defined(CONFIG_MIPS_AU1000) || defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_PB1100) || defined(CONFIG_MIPS_DB1100)
        //p->sofs = au_readl(USBD_FRAMENUM);
#endif
#if defined(CONFIG_ARCH_MX1ADS)
        p->sofs = USBD_FRAME & 0x3ff;
#endif

        p->interrupts = acm_interrupts;
        p->acm_trace_type = acm_trace_type;
	p->function = fn;

        //printk(KERN_INFO"first: %d next: %d interrupts: %d oscr: %d\n", acm_trace_first, acm_trace_next, acm_interrupts, p->oscr);

        return p;
}

/* Proc Filesystem *************************************************************************** */
        
/* *    
 * acm_trace_proc_read - implement proc file system read.
 * @file        
 * @buf         
 * @count
 * @pos 
 *      
 * Standard proc file system read function.
 */         
static ssize_t acm_trace_proc_read (struct file *file, char *buf, size_t count, loff_t * pos)
{                                  
        unsigned long page;
        int len = 0;
        int index;
        int oindex;
        int previous;
	unsigned long flags;

        acm_trace_t px;
        acm_trace_t ox;
        acm_trace_t *o, *p;
	

        MOD_INC_USE_COUNT;

	/* Get the index of the entry to read and update last_read - this needs to be atomic. */
        p = &px;
	o = NULL;
        oindex = index = (*pos)++;
	local_irq_save (flags);
                         
#if defined(TRACE_MAX_IS_2N)
	index = (acm_trace_first + index) & TRACE_MASK;
#else
        index += acm_trace_first;
        if (index >= TRACE_MAX) {
                index -= TRACE_MAX;
        }
#endif
	// Are we at the end of the data?
        if (((acm_trace_first < acm_trace_next) &&
             (index >= acm_trace_first) && (index < acm_trace_next)) ||
            ((acm_trace_first > acm_trace_next) &&
             ((index < acm_trace_next) || (index >= acm_trace_first)))) {
		// Nope, there's data to show.
		memcpy(p,(acm_traces+index),sizeof(acm_trace_t));
		// Is there a previous event?
        	previous = (index) ? (index - 1) : (TRACE_MAX - 1);
		if (previous != acm_trace_next && acm_trace_total > 1) {
			// There is a valid previous event.
			o = &ox;
			memcpy(o,(acm_traces+previous),sizeof(acm_trace_t));
		}
		acm_trace_last_read = index;
	} else {
		index = -1;
	}
	local_irq_restore (flags);
	if (index < 0) {
		// End of data.
		return(0);
	}

        // get a page, max 4095 bytes of data...
        if (!(page = get_free_page (GFP_KERNEL))) {
                MOD_DEC_USE_COUNT;
                return -ENOMEM;
        }

        len = 0;

        if (oindex == 0) {
#if defined(CONFIG_ARCH_SAMSUNG)
                len += sprintf ((char *) page + len, " Index     Ints     Ticks [%d]\n", CONFIG_USBD_SMDK2500_BCLOCK );
#else
                len += sprintf ((char *) page + len, " Index     Ints     Ticks\n");
#endif
        }       

        //printk(KERN_INFO"first: %d next: %d index: %d %d prev: %d\n", acm_trace_first, acm_trace_next, oindex, index, previous);

#if defined(CONFIG_ARCH_SA1100) || defined (CONFIG_ARCH_PXA) || defined(CONFIG_MIPS_AU1000)  || defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_PB1100) || defined(CONFIG_MIPS_DB1100)
        u32 ticks = 0;
#elif defined(CONFIG_ARCH_SAMSUNG)
        u32 ticks = 0;
#else
        u64 jifs = 0;
#endif
        unsigned char *cp;
        unsigned int *ip;
        int skip = 0;

	/* If there is a previous trace event, we want to calculate how many
           ticks have elapsed siince it happened.  Unfortunately, determining
           if there _is_ a previous event isn't obvious, since we have to watch
           out for startup and wraparound. */
        if (o != NULL) {

#if defined(CONFIG_ARCH_SA1100) || defined (CONFIG_ARCH_PXA)
                /*
                 * oscr is 3.6864 Mhz free running counter, 
                 *
                 *      1/3.6864 = .2712
                 *      60/221   = .2714
                 *
                 */
                if (o->oscr) {
                        ticks = (p->oscr > o->oscr) ? (p->oscr - o->oscr) : (o->oscr - p->oscr) ;
                        ticks = (ticks * 60) / 221;
                }

#elif defined(CONFIG_MIPS_AU1000)  || defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_PB1100) || defined(CONFIG_MIPS_DB1100)
                /*
                 * cp0_count is incrementing timer at system clock
                 */
                if (o->cp0_count) {
                        ticks = (p->cp0_count > o->cp0_count) ? 
                                (p->cp0_count - o->cp0_count) : (o->cp0_count - p->cp0_count) ;
                        ticks = ticks / CONFIG_USBD_AU1X00_SCLOCK;
                }

#elif defined(CONFIG_ARCH_SAMSUNG)
                /*
                 * tcnt1 is a count-down timer running at the system bus clock
                 * The divisor must be set as a configuration value, typically 66 or 133.
                 */
                if (o->tcnt1) {
                        ticks = (p->tcnt1 < o->tcnt1) ?  (o->tcnt1 - p->tcnt1) : (p->tcnt1 - o->tcnt1) ;
                        ticks /= CONFIG_USBD_SMDK2500_BCLOCK;
                }
#else
                if (o->jiffies) {
                        jifs = p->jiffies - acm_traces[previous].jiffies;
                }
#endif

                if (o->interrupts != p->interrupts) {
                        skip++;
                }
        }
                
        //printk(KERN_INFO"index: %d interrupts: %d\n", index, p->interrupts);
        len += sprintf ((char *) page + len, "%s%6d %8d ", skip?"\n":"", index, p->interrupts);

#if defined(CONFIG_ARCH_SA1100) || defined (CONFIG_ARCH_PXA) || defined(CONFIG_MIPS_AU1000) || defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_PB1100) || defined(CONFIG_MIPS_DB1100)
        if (ticks > 1024*1024) {
                len += sprintf ((char *) page + len, "%8dM ", ticks>>20);
        }
        else {
                len += sprintf ((char *) page + len, "%8d  ", ticks);
        }
#elif defined(CONFIG_ARCH_SAMSUNG)
        //len += sprintf ((char *) page + len, "%8u ", p->jiffies);
        //len += sprintf ((char *) page + len, "%8u ", p->tcnt0);
        len += sprintf ((char *) page + len, "%8u ", p->tcnt1);
        if (ticks > 1024*1024) {
                len += sprintf ((char *) page + len, "%8dM ", ticks>>20);
        }
        else {
                len += sprintf ((char *) page + len, "%8d  ", ticks);
        }
#else
        if (jifs > 1024) {
                len += sprintf ((char *) page + len, "%4dK ", (int)jifs>>10);
        }
        else {
                len += sprintf ((char *) page + len, "%4d  ", (int)jifs);
        }
#endif
#if defined(CONFIG_ARCH_MX1ADS)
        len += sprintf ((char *) page + len, "%6d  ", (int)p->sofs);
#endif

        switch (p->acm_trace_type) {
        case acm_trace_msg_n:
                len += sprintf ((char *) page + len, " --                   %s: ",p->function);
                len += sprintf ((char *) page + len, p->trace.msg.msg);
                break;

        case acm_trace_msg32_n:
                len += sprintf ((char *) page + len, " --                   %s: ",p->function);
                len += sprintf ((char *) page + len, p->trace.msg32.msg, p->trace.msg32.val);
                break;

        case acm_trace_msg16_n:
                len += sprintf ((char *) page + len, " --                   %s: ",p->function);
                len += sprintf ((char *) page + len, p->trace.msg16.msg, p->trace.msg16.val0, p->trace.msg16.val1);
                break;

        case acm_trace_msg8_n:
                len += sprintf ((char *) page + len, " --                   %s: ",p->function);
                len += sprintf ((char *) page + len, p->trace.msg8.msg, 
                                p->trace.msg8.val0, p->trace.msg8.val1, p->trace.msg8.val2, p->trace.msg8.val3);
                break;

        case acm_trace_setup_n:
                cp = (unsigned char *)&p->trace.setup;
                len += sprintf ((char *) page + len, 
                                " --           %s: request [%02x %02x %02x %02x %02x %02x %02x %02x]", 
                                p->function, cp[0], cp[1], cp[2], cp[3], cp[4], cp[5], cp[6], cp[7]);
                break;
        }
        len += sprintf ((char *) page + len, "\n");

	// len == 0 is valid, it just means we've reached the end of the data.
        if (len > count) {
                len = -EINVAL;
        } 
        else if (len > 0 && copy_to_user (buf, (char *) page, len)) {
                len = -EFAULT;
        }
        free_page (page);
        MOD_DEC_USE_COUNT;
        return len;
}

/* *
 * acm_trace_proc_write - implement proc file system write.
 * @file
 * @buf
 * @count
 * @pos
 *
 * Proc file system write function.
 */
static ssize_t acm_trace_proc_write (struct file *file, const char *buf, size_t count, loff_t * pos)
{
#define MAX_TRACE_CMD_LEN  64
        char command[MAX_TRACE_CMD_LEN+1];
        size_t n = count;
	size_t l;
	char c;

        if (n > 0) {
                l = MIN(n,MAX_TRACE_CMD_LEN);
                if (copy_from_user (command, buf, l)) {
                        count = -EFAULT;
                } else {
                        // flush remainder, if any
                        n -= l;
                        while (n > 0) {
                                // Not too efficient, but it shouldn't matter
                                if (copy_from_user (&c, buf + (count - n), 1)) {
                                        count = -EFAULT;
                                        break;
                                }
                                n -= 1;
                        }
			// Terminate command[]
                        if (l > 0 && command[l-1] == '\n') {
                                l -= 1;
                        }
                        command[l] = 0;
                }
        }

        if (0 >= count) {
                printk(KERN_INFO"%s: count <= 0 %d\n", __FUNCTION__, count);
                return count;
        }

	if (!strncmp("flush", command, 5)) {
		/* Move pointers so that next read continues from last read point,
                   instead of all available messages - this needs to be atomic.  */
		unsigned long flags;

		local_irq_save (flags);
#if defined(TRACE_MAX_IS_2N)
		acm_trace_first = (acm_trace_last_read + 1) & TRACE_MASK;
#else
		if (TRACE_MAX <= (acm_trace_first = acm_trace_last_read + 1)) {
			acm_trace_first = 0;
		}
#endif
		local_irq_restore (flags);
	}
#ifdef CONFIG_ARCH_EZX
	else if (!strncmp("reinit", command, 6))
	{
		acm_trace_first = 0;
		acm_trace_last_read = acm_trace_last_read = TRACE_MAX - 1;
		acm_trace_next = 0;
		acm_trace_total = 0;
		memset(acm_traces, 0, sizeof(acm_trace_t) * TRACE_MAX);
		TRACE_MSG("reinit");
		printk(KERN_INFO"%s: reinit\n", __FUNCTION__);
	}
#endif

        return count;
}

static struct file_operations acm_trace_proc_operations_functions = {
        read:acm_trace_proc_read,
        write:acm_trace_proc_write,
};

#if defined(CONFIG_ARCH_SAMSUNG)
#endif

/**
 * udc_request_udc_io - request UDC io region
 *
 * Return non-zero if not successful.
 */
int acm_trace_init (char *name)
{
        printk(KERN_INFO"%s: creating /proc/%s with %u entries\n", __FUNCTION__,name,TRACE_MAX);
        if (!(acm_traces = vmalloc(sizeof(acm_trace_t) * TRACE_MAX))) {
                printk(KERN_ERR"%s: malloc failed %p %d\n", __FUNCTION__, acm_traces, sizeof(acm_trace_t) * TRACE_MAX);
                return -EINVAL;
        }
        memset(acm_traces, 0, sizeof(acm_trace_t) * TRACE_MAX);
	acm_trace_last_read = TRACE_MAX - 1;

        {
                struct proc_dir_entry *p;

                // create proc filesystem entries
#ifdef CONFIG_ARCH_EZX
		if ((p = create_proc_entry (name, 0666, 0)) == NULL) {
#else
		if ((p = create_proc_entry (name, 0, 0)) == NULL) {
#endif
                        printk(KERN_INFO"%s PROC FS failed\n",name);
                }
                else {
                        p->proc_fops = &acm_trace_proc_operations_functions;
                }
        }
#if defined(CONFIG_ARCH_SAMSUNG)
        *(volatile u32 *)TMOD |= 0x3 << 3;
#endif
        printk(KERN_INFO"%s: OK\n", __FUNCTION__);
	return 0;
}

/**
 * acm_trace_exit - remove procfs entry, free trace data space.
 */
void acm_trace_exit (char *name)
{
        {
                unsigned long flags;
                local_irq_save (flags);
                remove_proc_entry (name, NULL);
                if (acm_traces) {
                        acm_trace_t *p = acm_traces;
                        acm_traces = NULL;
                        vfree(p);
                }
                local_irq_restore (flags);
        }
}


#else
int acm_trace_init (char *name)
{
        return 0;
}

void acm_trace_exit (char *name)
{
	return;
}
#endif

/* End of FILE */

