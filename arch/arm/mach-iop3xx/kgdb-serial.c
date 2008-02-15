/*
 * FILE NAME: arch/arm/mach-iop310/kgdb-serial.h
 *
 * BRIEF MODULE DESCRIPTION:
 *  Provides low level kgdb serial support hooks for the IQ80310 board
 *
 * TODO: This should be adapted to work on any 1655x UART
 *
 * Author: Deepak Saxena <dsaxena@mvista.com>
 *
 * Copyright 2001 MontaVista Software Inc.
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

#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/serial_reg.h>

#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>

#include <asm/kgdb.h>
#include <asm/serial.h>
#include <asm/hardware.h>
#include <asm/irq.h>


#if	defined(CONFIG_KGDB_UART0)
#	define  KGDB_UART 0x90810000
#	define  KGDB_UART_IDX 0
#elif	defined(CONFIG_KGDB_UART1)
#	define  KGDB_UART 0x90800000
#	define  KGDB_UART_IDX 1
#else
#	error "No kgdb serial port UART has been selected."
#endif

#if	defined(CONFIG_KGDB_9600BAUD)
#	define  BAUD_RATE   9600
#elif	defined(CONFIG_KGDB_19200BAUD)
#	define  BAUD_RATE   19200
#elif	defined(CONFIG_KGDB_38400BAUD)
#	define  BAUD_RATE   38400
#elif	defined(CONFIG_KGDB_57600BAUD)
#	define  BAUD_RATE   57600
#elif	defined(CONFIG_KGDB_115200BAUD)
#	define  BAUD_RATE   115200
#else
#	error "kgdb serial baud rate has not been specified."
#endif


static struct serial_state ports[RS_TABLE_SIZE] = {
	SERIAL_PORT_DFNS
};

static short index = 0;
static volatile unsigned char *serial_base = NULL;

void kgdb_serial_init(void)
{
	index = KGDB_UART_IDX;
	serial_base = (unsigned char *) __ioaddr(ports[index].port);

	serial_base[UART_LCR] = UART_LCR_DLAB | UART_LCR_WLEN8;
	serial_base[UART_DLL] = (ports[index].baud_base / BAUD_RATE) & 0xff;
	serial_base[UART_DLM] = (ports[index].baud_base / BAUD_RATE) >> 8;
	serial_base[UART_LCR] = UART_LCR_WLEN8;
	serial_base[UART_IER] = 0;
	serial_base[UART_MCR] = 0; // UART_MCR_DTR | UART_MCR_RTS;
	serial_base[UART_FCR] = 0;
	serial_base[UART_FCR] = 0xc0 | 0x00 | UART_FCR_ENABLE_FIFO;
	serial_base[UART_FCR] = 0xc0 | 0x06 | UART_FCR_ENABLE_FIFO;
	serial_base[UART_FCR] = 0xc0 | 0x00 | UART_FCR_ENABLE_FIFO;

	return;
}

void kgdb_serial_putchar(unsigned char ch)
{
	unsigned char status;

	do {
		status = serial_base[UART_LSR];
	} while ((status & (UART_LSR_TEMT|UART_LSR_THRE)) !=
			(UART_LSR_TEMT|UART_LSR_THRE));

	serial_base[UART_TX] = ch;
}

unsigned char kgdb_serial_getchar(void)
{
	unsigned char ch;

	while(!(serial_base[UART_LSR] & 0x1));

	ch = serial_base[UART_RX];

	return ch;
}
