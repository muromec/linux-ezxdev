/*
 * include/asm-arm/arch-omap/serial.h
 *
 * serial definitions
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *   <source@mvista.com>
 *
 */

#ifndef __ASM_ARCH_SERIAL_H
#define __ASM_ARCH_SERIAL_H
#include <asm/arch/hardware.h>
#include <asm/irq.h>

#define BASE_BAUD (48000000/16)

#if defined (CONFIG_OMAP1610_IR)
#define RS_TABLE_SIZE	2
#elif defined (CONFIG_OMAP1610_IR_MODULE)
#define RS_TABLE_SIZE	2
#else
#define RS_TABLE_SIZE   3
#endif

#define	STD_COM_FLAGS	(ASYNC_SKIP_TEST)

#define STD_SERIAL_PORT_DEFNS
#define EXTRA_SERIAL_PORT_DEFNS

#endif /* __ASM_ARCH_SERIAL_H */
