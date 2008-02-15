/*
 * usbd/trace.c
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
 * Copyright (C) 2003 - Motorola
 *
 * 2003-Dec-06 - Modified for EZXBASE By Zhao Liang <w20146@motorola.com>
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>

//EXPORT_NO_SYMBOLS;

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

#if defined(CONFIG_SOC_AU1X00) || \
        defined(CONFIG_MIPS_AU1X00) || \
        defined(CONFIG_CPU_AU1X00) || \
        defined(CONFIG_MIPS_AU1000) || \
        defined(CONFIG_MIPS_PB1500) || \
        defined(CONFIG_MIPS_PB1100) 

#include <asm/au1000.h>
#include <asm/au1000_dma.h>
#include <asm/mipsregs.h>

#endif /* defined(..) */

#if defined(CONFIG_ARCH_SAMSUNG)
#include <asm/arch/timers.h>
#include <asm/arch/hardware.h>
#endif /* defined(CONFIG_ARCH_SAMSUNG) */

#if defined(CONFIG_ARCH_MX1ADS)
#include "dbmx1_bi/dbmx1.h"
#endif /* defined(CONFIG_ARCH_MX1ADS) */

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>

#include "usbd-chap9.h"
#include "usbd-mem.h"
#include "usbd.h"
#include "usbd-func.h"
#include "usbd-bus.h"

#include "trace.h"
#include "usbd-bi.h"


#ifdef CONFIG_ARCH_EZX
extern int trace_reinit_ezx(void);
#endif

int trace_first;
int trace_next;
trace_t *traces;

#if defined(CONFIG_USBD_BI_REGISTER_TRACE) && defined(CONFIG_PROC_FS)

trace_t *TRACE_NEXT(trace_types_t trace_type)
{
        trace_t *p;

        p = traces + trace_next;

        #if defined(CONFIG_ARCH_SA1100) || \
                        defined (CONFIG_ARCH_PXA)

                p->oscr = OSCR;

        #elif defined(CONFIG_SOC_AU1X00) || \
                defined(CONFIG_MIPS_AU1X00) || \
                        defined(CONFIG_CPU_AU1X00) || \
                        defined(CONFIG_MIPS_AU1000) || \
                        defined(CONFIG_MIPS_PB1500) || \
                        defined(CONFIG_MIPS_PB1100) 

        #if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,19)
                p->cp0_count = __read_32bit_c0_register(CP0_COUNT);
        #else
                p->cp0_count = read_c0_count();
        #endif

        #elif defined(CONFIG_ARCH_SAMSUNG)
                p->tcnt1 = *(volatile u32 *)TCNT1;
        #else
                p->jiffies = jiffies;
        #endif

        #if defined(CONFIG_SOC_AU1X00) || \
                defined(CONFIG_MIPS_AU1X00) || \
                        defined(CONFIG_CPU_AU1X00) || \
                        defined(CONFIG_MIPS_AU1000) || \
                        defined(CONFIG_MIPS_PB1500) || \
                        defined(CONFIG_MIPS_PB1100) 
                //p->sofs = au_readl(USBD_FRAMENUM);
        #endif

        #if defined(CONFIG_ARCH_MX1ADS)
                p->sofs = USBD_FRAME & 0x3ff;
        #endif

        p->interrupts = udc_interrupts;
        p->trace_type = trace_type;

        trace_next++;
        trace_next = (trace_next == TRACE_MAX) ? 0 : trace_next;

        if (trace_next == trace_first) {
                trace_first++;
                trace_first = (trace_first == TRACE_MAX) ? 0 : trace_first;
        }

        return p;
}

/* Proc Filesystem *************************************************************************** */
        
static __inline__ char *nullchk(char *s)
{
        return s ? s : "(NULL)";
}
/* *    
 * trace_proc_read - implement proc file system read.
 * @file        
 * @buf         
 * @count
 * @pos 
 *      
 * Standard proc file system read function.
 */         
static ssize_t trace_proc_read (struct file *file, char *buf, size_t count, loff_t * pos)
{                                  
        unsigned long page;
        int len = 0;
        int index;
        int oindex;
        int previous;

        #ifdef MODULE
                MOD_INC_USE_COUNT;
        #endif
        // get a page, max 4095 bytes of data...
        if (!(page = get_free_page (GFP_KERNEL))) {

                #ifdef MODULE
                        MOD_DEC_USE_COUNT;
                #endif
                return -ENOMEM;
        }

        len = 0;
        oindex = index = (*pos)++;

        if (index == 0) {
                #if defined(CONFIG_ARCH_SAMSUNG)
                        len += sprintf ((char *) page + len, " Index     Ints     Ticks [%d]\n", CONFIG_USBD_SMDK2500_BCLOCK );
                #else
                        len += sprintf ((char *) page + len, " Index     Ints     Ticks\n");
                #endif
        }       
                         
        index += trace_first;
        if (index >= TRACE_MAX) {
                index -= TRACE_MAX;
        }
        previous = (index) ? (index - 1) : (TRACE_MAX - 1);

        //printk(KERN_INFO"first: %d next: %d index: %d %d prev: %d\n", trace_first, trace_next, oindex, index, previous);

        if (
                        ((trace_first < trace_next) && (index >= trace_first) && (index < trace_next)) ||
                        ((trace_first > trace_next) && ((index < trace_next) || (index >= trace_first)))
           )
        {

                #if defined(CONFIG_SOC_AU1X00) || \
                                defined(CONFIG_MIPS_AU1X00) || \
                                        defined(CONFIG_CPU_AU1X00) || \
                                        defined(CONFIG_MIPS_AU1000) || \
                                        defined(CONFIG_MIPS_PB1500) || \
                                        defined(CONFIG_MIPS_PB1100) 

                        u32 ticks = 0;

                #elif defined(CONFIG_ARCH_SAMSUNG)
                        u32 ticks = 0;
                #elif defined(CONFIG_ARCH_SA1100) || defined (CONFIG_ARCH_PXA)
                        u32 ticks = 0;
                        u64 jifs = 0;
                #else
                        u64 jifs = 0;
                #endif

                trace_t *p = traces + index;
                unsigned char *cp;
                int skip = 0;

                if (oindex > 0) {
                        trace_t *o = traces + previous;

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

                        #elif defined(CONFIG_SOC_AU1X00) || \
                                        defined(CONFIG_MIPS_AU1X00) || \
                                        defined(CONFIG_CPU_AU1X00) || \
                                        defined(CONFIG_MIPS_AU1000) || \
                                        defined(CONFIG_MIPS_PB1500) || \
                                        defined(CONFIG_MIPS_PB1100) 
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
                                        jifs = p->jiffies - traces[previous].jiffies;
                                }
                        #endif

                        if (o->interrupts != p->interrupts) {
                                skip++;
                        }
                }
                
                //printk(KERN_INFO"index: %d interrupts: %d\n", index, p->interrupts);
                len += sprintf ((char *) page + len, "%s%6d %8d ", skip?"\n":"", index, p->interrupts);

                #if defined(CONFIG_SOC_AU1X00) || \
                               defined(CONFIG_MIPS_AU1X00) || \
                               defined(CONFIG_CPU_AU1X00) || \
                               defined(CONFIG_MIPS_AU1000) || \
                               defined(CONFIG_MIPS_PB1500) || \
                               defined(CONFIG_MIPS_PB1100) || \
                               defined(CONFIG_ARCH_SA1100) || \
                               defined(CONFIG_ARCH_PXA)

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
                                len += sprintf ((char *) page + len, "%4d ", (int)jifs>>20);
                        }
                        else {
                                len += sprintf ((char *) page + len, "%4d  ", (int)jifs);
                        }
                #endif
                #if defined(CONFIG_ARCH_MX1ADS)
                        len += sprintf ((char *) page + len, "%6d  ", (int)p->sofs);
                #endif

                switch (p->trace_type) {
                case trace_msg:
                        len += sprintf ((char *) page + len, " --                   ");
                        len += sprintf ((char *) page + len, p->trace.msg.msg);
                        break;

                case trace_w:
                        len += sprintf ((char *) page + len, " -->                  ");
                        len += sprintf ((char *) page + len, "[%8x] W %s", p->trace.msg32.val, nullchk(p->trace.msg32.msg));
                        break;

                case trace_r:
                        len += sprintf ((char *) page + len, "<--                   ");
                        len += sprintf ((char *) page + len, "[%8x] R %s", p->trace.msg32.val, nullchk(p->trace.msg32.msg));
                        break;

                case trace_msg32:
                        len += sprintf ((char *) page + len, " --                   ");
                        len += sprintf ((char *) page + len, p->trace.msg32.msg, p->trace.msg32.val);
                        break;

                case trace_msg16:
                        len += sprintf ((char *) page + len, " --                   ");
                        len += sprintf ((char *) page + len, p->trace.msg16.msg, p->trace.msg16.val0, p->trace.msg16.val1);
                        break;

                case trace_msg8:
                        len += sprintf ((char *) page + len, " --                   ");
                        len += sprintf ((char *) page + len, p->trace.msg8.msg, 
                                        p->trace.msg8.val0, p->trace.msg8.val1, p->trace.msg8.val2, p->trace.msg8.val3);
                        break;

                case trace_setup:
                        cp = (unsigned char *)&p->trace.setup;
                        len += sprintf ((char *) page + len, 
                                        " --           request [%02x %02x %02x %02x %02x %02x %02x %02x]", 
                                        cp[0], cp[1], cp[2], cp[3], cp[4], cp[5], cp[6], cp[7]);
                        break;

                case trace_recv:
                case trace_sent:
                        cp = (unsigned char *)&p->trace.sent;
                        len += sprintf ((char *) page + len, 
                                        "%s             %s [%02x %02x %02x %02x %02x %02x %02x %02x]", 
                                        ( p->trace_type == trace_recv)?"<-- ":" -->",
                                        ( p->trace_type == trace_recv)?"recv":"sent",
                                        cp[0], cp[1], cp[2], cp[3], cp[4], cp[5], cp[6], cp[7]);
                        break;
                }
                len += sprintf ((char *) page + len, "\n");
        }

        if ((len > count) || (len == 0)) 
                len = -EINVAL;
        
        else if (len > 0 && copy_to_user (buf, (char *) page, len)) 
                len = -EFAULT;
        
        free_page (page);
        MOD_DEC_USE_COUNT;
        return len;
}

/* *
 * trace_proc_write - implement proc file system write.
 * @file
 * @buf
 * @count
 * @pos
 *
 * Proc file system write function, used to signal monitor actions complete.
 * (Hotplug script (or whatever) writes to the file to signal the completion
 * of the script.)  An ugly hack.
 */
#define MAX_BITRACE_LEN	10

static ssize_t trace_proc_write (struct file *file, const char *buf, size_t count, loff_t * pos)
{
#if 0
	return count;
#else
#ifdef CONFIG_ARCH_EZX
	// w20146 - expand a new feature to re-init bitrace
	char command[MAX_BITRACE_LEN + 1];
	size_t n = count, l;
	char c;

	command[0] = 0;
	printk(KERN_DEBUG "%s: count=%u\n",__FUNCTION__,count);

	if(n > 0)
	{
		l = MIN(n, MAX_BITRACE_LEN);
		if (copy_from_user (command, buf, l)) 
		{
			count = -EFAULT;
		}
		else
		{
			if (l > 0 && command[l-1] == '\n')
				l -= 1;

			command[l] = 0;
			n -= l;
			// flush remainder, if any
			while (n > 0) {
				// Not too efficient, but it shouldn't matter
				if (copy_from_user (&c, buf + (count - n), 1)) 
				{
					count = -EFAULT;
					break;
				}
				n--;
			}
		}
	}
	printk(KERN_DEBUG "%s: command [%s]\n",__FUNCTION__,command);
	if (count > 0 && !strcmp (command, "reinit")) 
	{
		trace_reinit_ezx();
	}

	return count;
#endif	// end of CONFIG_ARCH_EZX
#endif
}

static struct file_operations trace_proc_operations_functions = {
        read:trace_proc_read,
        write:trace_proc_write,
};


#ifdef CONFIG_ARCH_EZX
int trace_reinit_ezx(void)
{
	// w20146 - 
	printk(KERN_INFO"%s:\n", __FUNCTION__);
	trace_first = trace_next = 0;
	memset(traces, 0, sizeof(trace_t) * TRACE_MAX);
	TRACE_MSG("reinit");
	return 0;
}
#endif

int trace_reinit (void)
{
        //printk(KERN_INFO"%s:\n", __FUNCTION__);
        //trace_first = trace_next = 0;
        return 0;
}

/**
 * udc_request_udc_io - request UDC io region
 *
 * Return non-zero if not successful.
 */
int trace_init (void)
{
        struct proc_dir_entry *p;
        trace_first = trace_next = 0;
        printk(KERN_INFO"%s:\n", __FUNCTION__);

        if (!(traces = vmalloc(sizeof(trace_t) * TRACE_MAX))) {
                printk(KERN_ERR"BITRACE malloc failed %p %d\n", traces, sizeof(trace_t) * TRACE_MAX);
                return -EINVAL;
        }

        memset(traces, 0, sizeof(trace_t) * TRACE_MAX);

        TRACE_MSG("init");
        TRACE_MSG("test");

        // create proc filesystem entries
        if ((p = create_proc_entry ("bitrace", 0666, 0)) == NULL) 	// 0
                printk(KERN_INFO"BITRACE PROC FS failed\n");
        else 
                p->proc_fops = &trace_proc_operations_functions;

        #if defined(CONFIG_ARCH_SAMSUNG)
                *(volatile u32 *)TMOD |= 0x3 << 3;
        #endif
	return 0;
}

/**
 * udc_release_io - release UDC io region
 */
void trace_exit (void)
{
        {
                unsigned long flags;
                local_irq_save (flags);
                remove_proc_entry ("bitrace", NULL);
                if (traces) {
                        trace_t *p = traces;
                        traces = 0;
                        vfree(p);
                }
                local_irq_restore (flags);
        }
}

#else
int trace_reinit (void)
{
        return 0;
}

int trace_init (void)
{
        return 0;
}

void trace_exit (void) 
{
	return;
}
#endif

/* End of FILE */


//EXPORT_SYMBOL(trace_first);
//EXPORT_SYMBOL(trace_next);
//EXPORT_SYMBOL(traces);

