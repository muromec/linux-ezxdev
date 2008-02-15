/*
 * linux/include/asm-arm/arch-ti925/ti925_evm/serial.h
 *
 * BRIEF MODULE DESCRIPTION
 * serial definitions
 *
 * Copyright (C) 2000 RidgeRun, Inc.
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_TI925_EVM_SERIAL_H
#define __ASM_ARCH_TI925_EVM_SERIAL_H

#include <asm/arch/hardware.h>
#include <asm/irq.h>

#define BASE_BAUD (806400)

#define _SER_IRQ0	INT_UART1
#define _SER_IRQ1	INT_UART2

#define RS_TABLE_SIZE	2

#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST)


#define STD_SERIAL_PORT_DEFNS \
	{  \
	magic: 0, \
        baud_base: BASE_BAUD, \
	port: 0, \
	irq: _SER_IRQ1, \
	flags: STD_COM_FLAGS, \
	type: PORT_UNKNOWN, \
	iomem_base: (u8*)IO_ADDRESS(TI925_UART0_BASE), \
	iomem_reg_shift: 2, \
	io_type: SERIAL_IO_MEM \
	},	/* ttyS1 */ \
	{  \
	magic: 0, \
        baud_base: BASE_BAUD, \
	port: 0, \
	irq: _SER_IRQ0, \
	flags: STD_COM_FLAGS, \
	type: PORT_UNKNOWN, \
	iomem_base: (u8*)IO_ADDRESS(TI925_UART1_BASE), \
	iomem_reg_shift: 2, \
	io_type: SERIAL_IO_MEM \
	}	/* ttyS0 */ 

#define EXTRA_SERIAL_PORT_DEFNS

#endif
