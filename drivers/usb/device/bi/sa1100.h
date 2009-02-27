/*
 * sa1100_bi/udc.h
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


#define EP0_PACKETSIZE          0x8

#define UDC_MAX_ENDPOINTS       3

#define UDC_NAME                "Intel SA-1110 USBD"

#define MAX_DEVICES             1

#define EP1_RX_LOGICAL          1
#define EP2_TX_LOGICAL          2

#define EP1_RX_PHYSICAL         0
#define EP2_TX_PHYSICAL         1

#if defined(CONFIG_SA1100_ASSABET) || defined(CONFIG_SA1100_BITSY) || defined(CONFIG_SA1100_H3XXX) || defined(CONFIG_SA1100_COLLIE)
#define CONFIG_SA1100_USBCABLE_GPIO 23
#define CONFIG_SA1100_USBCABLE_ACTIVE_HIGH 1
#undef  CONFIG_SA1100_CONNECT_GPIO
#undef  CONFIG_SA1100_CONNECT_ACTIVE_HIGH
#endif

#if defined(CONFIG_SA1110_CALYPSO)
#define CONFIG_SA1100_USBCABLE_GPIO 12
#undef  CONFIG_SA1100_USBCABLE_ACTIVE_HIGH
#define CONFIG_SA1100_CONNECT_GPIO 27
#define CONFIG_SA1100_CONNECT_ACTIVE_HIGH 1
#endif

int ep0_reset (void);

void ep0_int_hndlr (unsigned int);
void ep1_int_hndlr (unsigned int);
void ep2_int_hndlr (unsigned int, int);

int ep1_restart (void);

void ep1_reset (void);
void ep2_reset (void);

int ep0_enable (struct usb_device_instance *device, struct usb_endpoint_instance *endpoint);
void ep1_enable (struct usb_device_instance *, struct usb_endpoint_instance *, int);
int ep2_enable (struct usb_device_instance *device, struct usb_endpoint_instance *, int);

void ep0_disable (void);
void ep1_disable (void);
void ep2_disable (void);

void ep2_send (void);

extern unsigned int udc_interrupts;
extern unsigned int ep0_interrupts;
extern unsigned int tx_interrupts;
extern unsigned int rx_interrupts;
extern unsigned int udc_address_errors;
extern unsigned int udc_rpe_errors;
extern unsigned int udc_fcs_errors;
extern unsigned int udc_ep1_errors;
extern unsigned int udc_ep2_errors;
extern unsigned int udc_ep2_tpe;
extern unsigned int udc_ep2_tur;
extern unsigned int udc_ep2_sst;
extern unsigned int udc_ep2_fst;

#define SET_AND_TEST(s,t,c) for (c=20; ((s), (t)) && c--;udelay(0))

#define IOIOIO(reg,val,set,test,rc) do { SET_AND_TEST(set,test,rc); } while(0)

#define _DDAR(Nb) (0xB0000000 + (Nb)*DMASp)

// Ser0UDCCR
#define UDCCR  ((volatile unsigned int *)io_p2v(0x80000000))
#define UDCCR_INTS (UDCCR_EIM | UDCCR_RIM | UDCCR_TIM | UDCCR_SRM | UDCCR_REM)
#define UDCCR_ERR29 0x80	// enable ERRATA 29 fix for B5 stepping

#define UDCAR ((volatile unsigned int *)io_p2v(0x80000004))
#define UDCOMP ((volatile unsigned int *)io_p2v(0x80000008))
#define UDCIMP ((volatile unsigned int *)io_p2v(0x8000000C))

// Ser0UDCCS0
#define UDCCS0 ((volatile unsigned int *)io_p2v(0x80000010))

#define UDCD0  ((volatile unsigned int *)io_p2v(0x8000001c))
#define UDCDR  ((volatile unsigned int *)io_p2v(0x80000028))
#define UDCWC  ((volatile unsigned int *)io_p2v(0x80000020))
#define UDCSR  ((volatile unsigned int *)io_p2v(0x80000030))

// Ser0UDCCS1
#define UDCCS1  ((volatile unsigned int *)io_p2v(0x80000014))

// Ser0UDCCS2
#define UDCCS2  ((volatile unsigned int *)io_p2v(0x80000018))

#define UDCCR  ((volatile unsigned int *)io_p2v(0x80000000))
#define UDCAR ((volatile unsigned int *)io_p2v(0x80000004))

#define SA1100_IRQ(x)           (0 + (x))
#define IRQ_GPIO_11_27(x)       (32 + (x) - 11)
#define SA1100_GPIO_TO_IRQ(i)   (((i) < 11) ? SA1100_IRQ(i) : IRQ_GPIO_11_27(i))

