/*
 * linux/include/asm-arm/arch-omap/serial.h
 *
 * BRIEF MODULE DESCRIPTION
 * serial definitions
 *
 */

#ifndef __ASM_ARCH_SERIAL_H
#define __ASM_ARCH_SERIAL_H
#include <asm/arch/hardware.h>
#include <asm/irq.h>

#define BASE_BAUD (12000000/16)

#ifdef CONFIG_OMAP_SIR_MODULE
#define RS_TABLE_SIZE	2
#else
#define RS_TABLE_SIZE	3
#endif

#define	STD_COM_FLAGS	(ASYNC_SKIP_TEST)

#define STD_SERIAL_PORT_DEFNS
#define EXTRA_SERIAL_PORT_DEFNS

#endif
