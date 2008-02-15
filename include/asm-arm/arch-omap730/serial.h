/*
 * include/asm-arm/arch-omap730/serial.h
 *
 * serial definitions
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *   <source@mvista.com>
 *
 * Modifications for the OMAP730 cpu :
 *
 * Copyright (c) 2004, MPC-Data Limited
 *   Dave Peverley <dpeverley@mpc-data.co.uk>
 *
 */

#ifndef __ASM_ARCH_SERIAL_H
#define __ASM_ARCH_SERIAL_H
#include <asm/arch/hardware.h>
#include <asm/irq.h>

#define BASE_BAUD (48000000/16)

#define RS_TABLE_SIZE	1

#define	STD_COM_FLAGS	(ASYNC_SKIP_TEST)

#define STD_SERIAL_PORT_DEFNS \
                { 0, BASE_BAUD, OMAP730_UART1_BASE, INT_UART1, 0 }

#define EXTRA_SERIAL_PORT_DEFNS

#endif /* __ASM_ARCH_SERIAL_H */
