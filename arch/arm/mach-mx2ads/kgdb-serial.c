/*
 * FILE NAME: arch/arm/mach-mx1ads/kgdb-serial.c
 *
 * BRIEF MODULE DESCRIPTION:
 *  Provides low level kgdb serial support hooks for MX1.
 *
 * Copyright 2001, 2003 MontaVista Software Inc. <source@<source@mvista.com>
 *
 * Copyright 1999 ARM Limited
 * Copyright (C) 2000 Deep Blue Solutions Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE	LIABLE FOR ANY   DIRECT, INDIRECT,
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

#include <linux/types.h>
#include <linux/serial.h>
#include <asm/ptrace.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/kgdb.h>
#include <asm/arch/pll.h>
#include <asm/arch/gpio.h>


typedef unsigned char U8;	/* unsigned 8 bit data  */
typedef unsigned short U16;	/* unsigned 16 bit data */
typedef unsigned int U32;	/* unsigned 32 bit data */


static  unsigned char port = 0;

void
kgdb_serial_init(void)
{
	unsigned long freq_10khz;

	switch (kgdb_ttyS) {
	case 0:
	mx_module_clk_open(IPG_MODULE_UART1);

	mx2_register_gpios(PORT_E,
				    (1 << 12) |
				    (1 << 13) |
				    (1 << 14) |
				    (1 << 15), PRIMARY | NOINTERRUPT);
	break;
	case 1:
	mx_module_clk_open(IPG_MODULE_UART2);

	mx2_register_gpios(PORT_E,
				    (1 << 3) |
				    (1 << 4) |
				    (1 << 6) |
				    (1 << 7), PRIMARY | NOINTERRUPT);
	break;
	case 2:
	mx_module_clk_open(IPG_MODULE_UART3);

	mx2_register_gpios(PORT_E,
				    (1 << 8) |
				    (1 << 9) |
				    (1 << 10) |
				    (1 << 11), PRIMARY | NOINTERRUPT);
	break;
	case 3:
	mx_module_clk_open(IPG_MODULE_UART4);

#ifdef CONFIG_MX2TO1
	mx2_register_gpios(PORT_B,
				    (1 << 28) |
				    (1 << 29) |
				    (1 << 30) |
				    (1 << 31), SECONDARY | NOINTERRUPT);


#else
	mx2_register_gpios(PORT_B,
				    (1 << 28), SECONDARY | NOINTERRUPT);
	mx2_register_gpios(PORT_B,
				    (1 << 31) |
				    (1 << 29), GPIO | INPUT);
	mx2_register_gpios(PORT_B,
				    (1 << 30), GPIO | OUTPUT);

	SYS_FMCR &= ~((1 << 25) | (1 << 24));
#endif
	break;
	default:
	return;
	}

	port = kgdb_ttyS;

#if defined(CONFIG_MX2TO1)
	freq_10khz = mx_module_get_clk(PERCLK) / 10000;
#else /*TO2*/
	freq_10khz = mx_module_get_clk(PERCLK1) / 10000;
#endif

	UART_UCR1(port)=0x1;
	UART_UCR2(port)=0x4026;
	UART_UCR3(port)=0x4;
	UART_UCR4(port)=0x8000;
	UART_UFCR(port)=(32 << UART_TXTL_BIT) | (1 << UART_RXTL_BIT) | UART_RFDIV_2;
	UART_UBIR(port)=((kgdb_baud * 16 * 2) / freq_10khz) - 1;
	UART_UBMR(port)=10000 - 1;
	return;
}

void
kgdb_serial_putchar(unsigned char ch)
{
	while (!(UART_USR2(port) & USR2_TXFE)) ;    /* wait until TXFE bit set */
	UART_UTXD(port)= (unsigned int) (ch & 0xff);
}

unsigned char
kgdb_serial_getchar(void)
{
	while (!(UART_USR2(port) & USR2_RDR)) ;	    /* wait until RDR bit set */
	return (unsigned char) UART_URXD(port);
}
