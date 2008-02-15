/*
 * FILE NAME: linux/arch/arm/mach-pxa/kgdb-serial.c
 *  
 * BRIEF MODULE DESCRIPTION:
 *  Provides low level kgdb serial support hooks for the DBPXA250 board
 *  
 * Author:	Nicolas Pitre
 * Copyright:	(C) 2002 MontaVista Software Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 * WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO  EVENT  SHALL   THE AUTHOR  BE	LIABLE FOR ANY   DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 * USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 * 2005-May-11  copy from arch/arm/mach-pxa/kgdb-serial.c and porting to EZX platform,  Zhuang Xiaofan
 *
 */


#include <asm/hardware.h>
#include <asm/kgdb.h>
#include <linux/serial_reg.h>

#define UART		FFUART
//#define UART		BTUART
//#define UART		STUART

static volatile unsigned long *port = (unsigned long *)&UART;

void kgdb_serial_init(void)
{
	switch ((long)port) {
	case (long)&FFUART: CKEN |= CKEN6_FFUART; break;
	case (long)&BTUART: CKEN |= CKEN7_BTUART; break;
	case (long)&STUART: CKEN |= CKEN5_STUART; break;
	}

	port[UART_LCR] = LCR_DLAB;
	port[UART_DLL] = 8;	/* base = 921600 bauds */
	port[UART_DLM] = 0;
	port[UART_LCR] = LCR_WLS1|LCR_WLS0;
	port[UART_MCR] = 0;
	port[UART_IER] = IER_UUE;
	port[UART_FCR] = FCR_ITL_16;

	return;
}

void kgdb_serial_putchar(unsigned char c)
{
	while (!(port[UART_LSR] & LSR_TDRQ));
	port[UART_TX] = c;
}

unsigned char kgdb_serial_getchar(void)
{
	while (!(port[UART_LSR] & UART_LSR_DR));
	return port[UART_RX];
}
