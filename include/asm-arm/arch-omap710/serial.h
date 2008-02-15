/*
 * linux/include/asm-arm/arch-omap710/serial.h
 *
 * BRIEF MODULE DESCRIPTION
 * serial definitions
 *
 */

#ifndef __ASM_ARCH_SERIAL_H
#define __ASM_ARCH_SERIAL_H
#include <asm/arch/hardware.h>
#include <asm/irq.h>

#if	defined(CONFIG_SERIAL_8250)

#define BASE_BAUD (12000000/16)

#define RS_TABLE_SIZE	2

#define STD_COM_FLAGS (ASYNC_SKIP_TEST)

#define STD_SERIAL_PORT_DEFNS \
	{			\
		base_baud: BASE_BAUD,		\
		irq: INT_UART1,			\
		flags: STD_COM_FLAGS,		\
		iomem_base: (u8*)UART1_BASE,	\
		io_type: SERIAL_IO_MEM		\
	}, {					\
		base_baud: BASE_BAUD,		\
		irq: INT_UART3,			\
		flags: STD_COM_FLAGS,		\
		iomem_base: (u8*)UART3_BASE,	\
		io_type: SERIAL_IO_MEM		\
	}

#define EXTRA_SERIAL_PORT_DEFNS

#elif	defined(CONFIG_SERIAL)

#define BASE_BAUD (750000)
//#define BASE_BAUD (345600)

#define RS_TABLE_SIZE	5

#define	STD_COM_FLAGS	(ASYNC_SKIP_TEST)
// #undef	STD_COM_FLAGS	(ASYNC_SKIP_TEST)
// #define	STD_COM_FLAGS	(ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST)

#define STD_SERIAL_PORT_DEFNS \
	{	/* ttyS0: */				\
		type:		PORT_16750,		\
		xmit_fifo_size:	64,			\
		baud_base:	(12000000/16),		\
		iomem_base:	(u8*)UART_MODEM,	\
		iomem_reg_shift:	0,		\
		io_type:	SERIAL_IO_MEM,		\
		irq:		INT_UART_MODEM,		\
		flags:		STD_COM_FLAGS |		\
				ASYNC_SPD_OMAP		\
	}, {	/* ttyS1: */				\
		type:		PORT_16750,		\
		xmit_fifo_size:	64,			\
		baud_base:	(12000000/16),		\
		iomem_base:	(u8*)UART_IRDA,		\
		iomem_reg_shift:	0,		\
		io_type:	SERIAL_IO_MEM,		\
		irq:		INT_UART_IRDA,		\
		flags:		STD_COM_FLAGS |		\
				ASYNC_SPD_OMAP		\
	}, {	/* ttyS2: */				\
		type:		PORT_16750,		\
		xmit_fifo_size:	64,			\
		baud_base:	(12000000/16),		\
		iomem_base:	(u8*)UART_MODEM2,	\
		/* NOTE: Contrary to OMAP710 ref man, this works: */	\
		iomem_reg_shift:	0,		\
		io_type:	SERIAL_IO_MEM,		\
		irq:		INT_UART_MODEM2,	\
		/* NOTE: Contrary to OMAP710 ref man, this works: */	\
		flags:		STD_COM_FLAGS |		\
				ASYNC_SPD_OMAP		\
	}

#define EXTRA_SERIAL_PORT_DEFNS

#endif

#endif
