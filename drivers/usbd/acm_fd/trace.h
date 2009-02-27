/*
 * usbd/acm_fd/trace.h
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
 * 2004-Dec-06 - Modified for EZXBASE debug  By Zhao Liang <w20146@motorola.com>
 *
 */

#if defined(CONFIG_ARCH_SAMSUNG)
#ifndef CONFIG_USBD_SMDK2500_BCLOCK 
#define CONFIG_USBD_SMDK2500_BCLOCK 66
#endif
#endif

/* Fast Trace Utility
   This set of definitions and code is meant to provide a _fast_ debugging facility
   (much faster than printk) so that time critical code can be debugged by looking
   at a trace of events provided by reading a file in the procfs without affecting
   the timing of events in the critical code.

   The mechanism used it to allocate a (large) ring buffer of relatively small structures
   that include location and high-res timestamp info, and up to 8 bytes of optional
   data.  Values are stored and timestamps are taken as the critical code runs, but
   data formatting and display are done during the procfs read, when more time is
   available :).

   Note that there is usually some machine dependent code involved in getting the
   high-res timestamp, and there may be other bits used just to keep the overall
   time impact as low as possible.

   varargs style macros were avoided because there seems to be no way to avoid
   a run-time check on the number of arguments if they are used, and the time penalty
   doesn't seem to be worth the gain in utility.
 */

typedef enum acm_trace_types {
        acm_trace_setup_n, acm_trace_msg_n, acm_trace_msg32_n, acm_trace_msg16_n, acm_trace_msg8_n
} acm_trace_types_t;

typedef struct acm_trace_msg {
        char    *msg;
} acm_trace_msg_t;

typedef struct acm_trace_msg32 {
        u32      val;
        char    *msg;
} acm_trace_msg32_t;

typedef struct acm_trace_msg16 {
        u16     val0;
        u16     val1;
        char    *msg;
} acm_trace_msg16_t;

typedef struct acm_trace_msg8 {
        u8      val0;
        u8      val1;
        u8      val2;
        u8      val3;
        char    *msg;
} acm_trace_msg8_t;


typedef struct trace {
        acm_trace_types_t        acm_trace_type;
        char    *function;
        u32     interrupts;
#if defined(CONFIG_ARCH_SA1100) || defined (CONFIG_ARCH_PXA)
        u32     oscr;
#elif defined(CONFIG_MIPS_AU1000) || defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_PB1100) || defined(CONFIG_MIPS_DB1100)
        u32     cp0_count;
#elif defined(CONFIG_ARCH_SAMSUNG)
        //u32     tcnt0;
        u32     tcnt1;
        //u64     jiffies;
#else
        u64     jiffies;
#endif
#if defined(CONFIG_MIPS_AU1000) || defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_PB1100) || defined(CONFIG_MIPS_DB1100) || defined(CONFIG_ARCH_MX1ADS)
        u64     sofs;
#endif
        union {
                acm_trace_msg_t        msg;
                acm_trace_msg8_t       msg8;
                acm_trace_msg16_t      msg16;
                acm_trace_msg32_t      msg32;

                struct usb_device_request       setup;

        } trace;

} acm_trace_t;

#define TRACE_MAX_IS_2N 1
#if defined(TRACE_MAX_IS_2N)
#define TRACE_MAX  0x00008000
#define TRACE_MASK 0x00007FFF
#else
#define TRACE_MAX       30000
#endif

extern int acm_trace_first;
extern int acm_trace_last_read;
extern int acm_trace_next;

extern acm_trace_t *acm_traces;

#ifdef CONFIG_USBD_ACM_TRACE

acm_trace_t *ACM_TRACE_NEXT(char *fn, acm_trace_types_t acm_trace_type);

#if 0
static __inline__ acm_trace_t *ACM_TRACE_NEXT(char *fn, acm_trace_types_t acm_trace_type)
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
	// End of next trace slot.
	local_irq_restore (flags);

#if defined(CONFIG_ARCH_SA1100) || defined (CONFIG_ARCH_PXA)
        p->oscr = OSCR;
#elif defined(CONFIG_MIPS_AU1000) || defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_PB1100) || defined(CONFIG_MIPS_DB1100)
        //p->cp0_count = __read_32bit_c0_register(CP0_COUNT);
        p->cp0_count = read_c0_count();
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

        p->interrupts = udc_interrupts;
        p->acm_trace_type = acm_trace_type;
	p->function = fn;

        acm_trace_next++;
        acm_trace_next = (acm_trace_next == TRACE_MAX) ? 0 : acm_trace_next;

        if (acm_trace_next == acm_trace_first) {
                acm_trace_first++;
                acm_trace_first = (acm_trace_first == TRACE_MAX) ? 0 : acm_trace_first;
        }
        //printk(KERN_INFO"first: %d next: %d interrupts: %d\n", acm_trace_first, acm_trace_next, udc_interrupts);

        return p;
}
#endif

static __inline__ void acm_trace_setup(char *fn, struct usb_device_request *setup)
{
        if (acm_traces) {
                acm_trace_t *p = ACM_TRACE_NEXT(fn,acm_trace_setup_n);
                memcpy(&p->trace.setup, setup, sizeof(struct usb_device_request));
        }
}
#define TRACE_SETUP(setup) acm_trace_setup(__FUNCTION__,setup)

static __inline__ void acm_trace_msg(char *fn, char *msg)
{
        if (acm_traces) {
                acm_trace_t *p = ACM_TRACE_NEXT(fn,acm_trace_msg_n);
                p->trace.msg.msg = msg;
        }
}
#define TRACE_MSG(msg) acm_trace_msg(__FUNCTION__,msg)


static __inline__ void acm_trace_msg_1xU32(char *fn, char *fmt, u32 val)
{
        if (acm_traces) {
                acm_trace_t *p = ACM_TRACE_NEXT(fn,acm_trace_msg32_n);
                p->trace.msg32.val = val;
                p->trace.msg32.msg = fmt;
        }
}
#define TRACE_MSG1(fmt,val) acm_trace_msg_1xU32(__FUNCTION__,fmt,(u32)val)

static __inline__ void acm_trace_msg_2xU16(char *fn, char *fmt, u16 val0, u16 val1)
{
        if (acm_traces) {
                acm_trace_t *p = ACM_TRACE_NEXT(fn,acm_trace_msg16_n);
                p->trace.msg16.val0 = val0;
                p->trace.msg16.val1 = val1;
                p->trace.msg16.msg = fmt;
        }
}
#define TRACE_MSG2(fmt,val0,val1) acm_trace_msg_2xU16(__FUNCTION__,fmt,(u16)val0,(u16)val1)

static __inline__ void acm_trace_msg_4xU8(char *fn, char *fmt, u8 val0, u8 val1, u8 val2, u8 val3)
{
        if (acm_traces) {
                acm_trace_t *p = ACM_TRACE_NEXT(fn,acm_trace_msg8_n);
                p->trace.msg8.val0 = val0;
                p->trace.msg8.val1 = val1;
                p->trace.msg8.val2 = val2;
                p->trace.msg8.val3 = val3;
                p->trace.msg8.msg = fmt;
        }
}
#define TRACE_MSG4(fmt,val0,val1,val2,val3) acm_trace_msg_4xU8(__FUNCTION__,fmt,(u8)val0,(u8)val1,(u8)val2,(u8)val3)

#else

#define TRACE_SETUP(setup)
#define TRACE_MSG(msg)
#define TRACE_MSG1(fmt,val)
#define TRACE_MSG2(fmt,val0,val1)
#define TRACE_MSG4(fmt,val0,val1,val2,val3)

#endif

int acm_trace_init (char *str);
void acm_trace_exit (char *str);

