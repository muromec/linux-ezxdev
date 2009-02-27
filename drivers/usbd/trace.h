/*
 * usbd/trace.h
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


typedef enum trace_int_types {
        trace_int_udc, trace_int_ep0, trace_int_in, trace_int_out, trace_int_int
} trace_int_types_t;

typedef struct trace_int {
        u32     last;
        u32     total;
        u32     samples;
} trace_int_t;


typedef enum trace_types {
        trace_setup, trace_msg, trace_msg32, trace_msg16, trace_msg8, trace_recv, trace_sent, trace_w, trace_r
} trace_types_t;


typedef struct trace_regs32 {
        u32     reg;
        char *  msg;
} trace_regs32_t;


typedef struct trace_msg {
        char    *msg;
} trace_msg_t;

typedef struct trace_msg32 {
        u32      val;
        char    *msg;
} trace_msg32_t;

typedef struct trace_msg16 {
        u16     val0;
        u16     val1;
        char    *msg;
} trace_msg16_t;

typedef struct trace_msg8 {
        u8      val0;
        u8      val1;
        u8      val2;
        u8      val3;
        char    *msg;
} trace_msg8_t;


typedef struct trace {
        trace_types_t        trace_type;
        u32     interrupts;
#if defined(CONFIG_ARCH_SA1100) || defined (CONFIG_ARCH_PXA)
        u32     oscr;

#elif defined(CONFIG_SOC_AU1X00) || defined(CONFIG_MIPS_AU1X00) || defined(CONFIG_CPU_AU1X00) || defined(CONFIG_MIPS_AU1000) || defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_PB1100) 

        u32     cp0_count;
#elif defined(CONFIG_ARCH_SAMSUNG)
        //u32     tcnt0;
        u32     tcnt1;
        //u64     jiffies;
#else
        u64     jiffies;
#endif
#if defined(CONFIG_SOC_AU1X00) || defined(CONFIG_MIPS_AU1X00) || defined(CONFIG_CPU_AU1X00) || defined(CONFIG_MIPS_AU1000) || defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_PB1100) || defined(CONFIG_ARCH_MX1ADS)
        u64     sofs;
#endif
        union {
                trace_msg_t        msg;
                trace_msg8_t       msg8;
                trace_msg16_t      msg16;
                trace_msg32_t      msg32;

                struct usb_device_request       setup;
                unsigned char      recv[8];
                unsigned char      sent[8];

        } trace;

} trace_t;


#define TRACE_MAX       30000		//10000		// w20146 - add it

extern int trace_first;
extern int trace_next;

extern trace_int_t *trace_ints;
extern trace_t *traces;

#ifdef CONFIG_USBD_BI_REGISTER_TRACE

trace_t *TRACE_NEXT(trace_types_t trace_type);

static __inline__ void TRACE_SETUP(struct usb_device_request *setup)
{
        if (traces) {
                trace_t *p = TRACE_NEXT(trace_setup);
                p->trace_type = trace_setup;
                memcpy(&p->trace.setup, setup, sizeof(struct usb_device_request));
        }
}

static __inline__ void TRACE_MSG(char *msg)
{
        if (traces) {
                trace_t *p = TRACE_NEXT(trace_msg);
                p->trace.msg.msg = msg;
        }
}

static __inline__ void TRACE_W(char *msg, u32 val)
{
        if (traces) {
                trace_t *p = TRACE_NEXT(trace_w);
                p->trace.msg32.val = val;
                p->trace.msg32.msg = msg;
        }
}

static __inline__ void TRACE_R(char *msg, u32 val)
{
        if (traces) {
                trace_t *p = TRACE_NEXT(trace_r);
                p->trace.msg32.val = val;
                p->trace.msg32.msg = msg;
        }
}

static __inline__ void TRACE_MSG32(char *msg, u32 val)
{
        if (traces) {
                trace_t *p = TRACE_NEXT(trace_msg32);
                p->trace.msg32.val = val;
                p->trace.msg32.msg = msg;
        }
}

static __inline__ void TRACE_MSG16(char *msg, u16 val0, u16 val1)
{
        if (traces) {
                trace_t *p = TRACE_NEXT(trace_msg16);
                p->trace.msg16.val0 = val0;
                p->trace.msg16.val1 = val1;
                p->trace.msg16.msg = msg;
        }
}

static __inline__ void TRACE_MSG8(char *msg, u8 val0, u8 val1, u8 val2, u8 val3)
{
        if (traces) {
                trace_t *p = TRACE_NEXT(trace_msg8);
                p->trace.msg8.val0 = val0;
                p->trace.msg8.val1 = val1;
                p->trace.msg8.val2 = val2;
                p->trace.msg8.val3 = val3;
                p->trace.msg8.msg = msg;
        }
}

static __inline__ void TRACE_RECV(unsigned char *cp)
{
        if (traces) {
                trace_t *p = TRACE_NEXT(trace_recv);
                memcpy(&p->trace.recv, cp, 8);
        }
}

static __inline__ void TRACE_RECVN(unsigned char *cp, int bytes)
{
        if (traces) {
                trace_t *p = TRACE_NEXT(trace_recv);
                memset(&p->trace.recv, 0, 8);
                memcpy(&p->trace.recv, cp, bytes);
        }
}

static __inline__ void TRACE_SENT(unsigned char *cp)
{
        if (traces) {
                trace_t *p = TRACE_NEXT(trace_sent);
                memcpy(&p->trace.sent, cp, 8);
        }
}

#else

static __inline__ void TRACE_SETUP(struct usb_device_request *setup)
{
}

static __inline__ void TRACE_IRQS(u32 cr, u32 sr)
{
}

static __inline__ void TRACE_RECV(unsigned char *cp)
{
}

static __inline__ void TRACE_SENT(unsigned char *cp)
{
}

static __inline__ void TRACE_W(char *msg, u32 val)
{
}

static __inline__ void TRACE_R(char *msg, u32 val)
{
}

static __inline__ void TRACE_MSG(char *msg)
{
}

static __inline__ void TRACE_MSG32(char *msg, u32 val)
{
}

static __inline__ void TRACE_MSG16(char *msg, u16 val0, u16 val1)
{
}

static __inline__ void TRACE_MSG8(char *msg, u8 val0, u8 val1, u8 val2, u8 vale)
{
}
static __inline__ void TRACE_RECVN(unsigned char *cp, int bytes)
{
}

#endif

int trace_init (void);
int trace_reinit (void);
void trace_exit (void);

