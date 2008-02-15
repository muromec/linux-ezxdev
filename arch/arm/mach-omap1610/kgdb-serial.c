/*
 * FILE NAME: arch/arm/mach-omap/kgdb-serial.c
 *
 * BRIEF MODULE DESCRIPTION:
 *  Provides low level kgdb serial support hooks for the TI OMAP
 *
 * TODO: This should be adapted to work on any 1655x UART
 *
 * Author: George Davis <gdavis@mvista.com>
 *
 * Copyright 2002 MontaVista Software Inc.
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

static struct async_struct pinfo = {0};

static __inline__ unsigned int
serial_inp(struct async_struct *info, int offset)
{
	return readb((unsigned long) info->iomem_base +
			(offset<<info->iomem_reg_shift));
}

static __inline__ void
serial_outp(struct async_struct *info, int offset, int value)
{
	writeb(value, (unsigned long) info->iomem_base +
		(offset<<info->iomem_reg_shift));
}

void
kgdb_serial_init(void)
{
	extern int kgdb_ttyS;
	extern int kgdb_baud;
	extern struct serial_state rs_table[];
	struct serial_state *ser = &rs_table[kgdb_ttyS];
	unsigned short quot;

	pinfo.state = ser;
	pinfo.magic = SERIAL_MAGIC;
	pinfo.port = ser->port;
	pinfo.flags = ser->flags;
	pinfo.iomem_base = ser->iomem_base;
	pinfo.iomem_reg_shift = ser->iomem_reg_shift;

	serial_outp(&pinfo, UART_OMAP_MDR1, 0x07);	/* disable UART */
	serial_outp(&pinfo, UART_LCR, 0xBF);	/* select EFR */
	serial_outp(&pinfo, UART_EFR, UART_EFR_ECB);
	serial_outp(&pinfo, UART_LCR, UART_LCR_DLAB);	/* set DLAB */
	serial_outp(&pinfo, UART_DLL, 0x00);
	serial_outp(&pinfo, UART_DLM, 0x00);
	serial_outp(&pinfo, UART_LCR, 0x00);	/* reset DLAB */
	serial_outp(&pinfo, UART_OMAP_SCR, 0x00);
	serial_outp(&pinfo, UART_FCR, 0x00);
	serial_outp(&pinfo, UART_MCR, 0x40);	/* enable TCR/TLR */
	serial_outp(&pinfo, UART_OMAP_TCR, 0x0F);
	serial_outp(&pinfo, UART_OMAP_TLR, 0x00);
	serial_outp(&pinfo, UART_MCR, 0x00);
	serial_outp(&pinfo, UART_LCR, 0xBF);	/* select EFR */
	serial_outp(&pinfo, UART_EFR, 0x00);
	serial_outp(&pinfo, UART_LCR, 0x00);	/* reset DLAB */
	serial_outp(&pinfo, UART_OMAP_MDR1, 0x00);	/* enable UART */

	/*
	 * Clear the FIFO buffers and disable them
	 * (they will be reenabled later)
	 */
	serial_outp(&pinfo, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_outp(&pinfo, UART_FCR, (UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR |
			UART_FCR_CLEAR_XMIT));
	serial_outp(&pinfo, UART_FCR, 0);

	/*
	 * Clear the interrupt registers.
	 */
        (void) serial_inp(&pinfo, UART_LSR);
        (void) serial_inp(&pinfo, UART_RX);
        (void) serial_inp(&pinfo, UART_IIR);
        (void) serial_inp(&pinfo, UART_MSR);

	/*
	 * Now, initialize the UART
	 */
        serial_outp(&pinfo, UART_LCR, UART_LCR_WLEN8);    /* reset DLAB */

	quot = pinfo.state->baud_base / kgdb_baud;

	serial_outp(&pinfo, UART_LCR, UART_LCR_WLEN8);
	serial_outp(&pinfo, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_outp(&pinfo, UART_LCR, UART_LCR_WLEN8 | UART_LCR_DLAB);
	serial_outp(&pinfo, UART_DLL, quot & 0xff);
	serial_outp(&pinfo, UART_DLM, quot >> 8);
	serial_outp(&pinfo, UART_LCR, UART_LCR_WLEN8);
	serial_outp(&pinfo, UART_IER, 0);
	serial_outp(&pinfo, UART_MCR, 0);
}

void
kgdb_serial_putchar(unsigned char ch)
{
	unsigned char status;

	if (!pinfo.state)
		return;			/* need to init device first */

	do {
		status = serial_inp(&pinfo, UART_LSR);
	} while ((status & (UART_LSR_TEMT | UART_LSR_THRE)) !=
		 (UART_LSR_TEMT | UART_LSR_THRE));

	serial_outp(&pinfo, UART_TX, ch);
}

unsigned char
kgdb_serial_getchar(void)
{
	if (!pinfo.state)
		return 0;		/* need to init device first */

	while ((serial_inp(&pinfo, UART_LSR) & UART_LSR_DR) == 0)
		;

	return(serial_inp(&pinfo, UART_RX));
}
