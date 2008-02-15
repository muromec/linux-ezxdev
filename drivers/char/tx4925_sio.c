/*
 * linux/drivers/char/tx4925_sio.c
 *
 * tx4925 Serial IO driver (using generic serial)
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * Copyright 2001-2002 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#ifdef CONFIG_TX4925_SIO_CONSOLE
#include <linux/console.h>
#endif
#include <linux/tx4925_sio.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>
#include <asm/serial.h>

/******************************************************************************/
/* BEG: TODO                                                                  */
/******************************************************************************/

#define TX4925_SIO_INLINE inline

#define TX4925_SIO_CLOSE_WAIT  ( 30 * HZ )
#define TX4925_SIO_CLOSE_DELAY (  5 * HZ / 10 )

static void tx4925_sio_disable_tx_interrupts(void *ptr);
static void tx4925_sio_enable_tx_interrupts(void *ptr);
static void tx4925_sio_disable_rx_interrupts(void *ptr);
static void tx4925_sio_enable_rx_interrupts(void *ptr);
static int tx4925_sio_get_CD(void *ptr);
static void tx4925_sio_shutdown_port(void *ptr);
static int tx4925_sio_set_real_termios(void *ptr);
static int tx4925_sio_chars_in_buffer(void *ptr);
static void tx4925_sio_close(void *ptr);
static void tx4925_sio_hungup(void *ptr);
static int tx4925_sio_info_port_count(void *ptr);
static char *tx4925_sio_info_driver_name(void *ptr);
static char *tx4925_sio_info_driver_version(void *prt);
static int tx4925_sio_info_line(void *ptr, char *buf, int port, int max);

#define TX4925_SIO_DEBUG

#ifdef TX4925_SIO_DEBUG
#define TX4925_SIO_DEBUG_NONE       0x00000000

#define TX4925_SIO_DEBUG_INFO       ( 1 <<  0 )
#define TX4925_SIO_DEBUG_WARN       ( 1 <<  1 )
#define TX4925_SIO_DEBUG_EROR       ( 1 <<  2 )

#define TX4925_SIO_DEBUG_INIT       ( 1 <<  3 )
#define TX4925_SIO_DEBUG_OPEN       ( 1 <<  4 )
#define TX4925_SIO_DEBUG_CLOSE      ( 1 <<  5 )
#define TX4925_SIO_DEBUG_IRQ        ( 1 <<  6 )
#define TX4925_SIO_DEBUG_IRQ_RX     ( 1 <<  7 )
#define TX4925_SIO_DEBUG_IRQ_TX     ( 1 <<  8 )

#define TX4925_SIO_DEBUG_TERMIOS    ( 1 << 10 )
#define TX4925_SIO_DEBUG_IOCTL      ( 1 << 11 )
#define TX4925_SIO_DEBUG_THROTTLE   ( 1 << 12 )
#define TX4925_SIO_DEBUG_BREAK      ( 1 << 12 )

#define TX4925_SIO_DEBUG_CON_WAIT   ( 1 << 21 )
#define TX4925_SIO_DEBUG_CON_WR     ( 1 << 22 )
#define TX4925_SIO_DEBUG_CON_WR_CH  ( 1 << 23 )
#define TX4925_SIO_DEBUG_CON_DEV    ( 1 << 24 )
#define TX4925_SIO_DEBUG_CON_SETUP  ( 1 << 25 )
#define TX4925_SIO_DEBUG_CON_INIT   ( 1 << 26 )

#define TX4925_SIO_DEBUG_KDBG       ( 1 << 27 )
#define TX4925_SIO_DEBUG_KDBG_RD    ( 1 << 28 )
#define TX4925_SIO_DEBUG_KDBG_RD_CH ( 1 << 29 )
#define TX4925_SIO_DEBUG_KDBG_WR    ( 1 << 30 )
#define TX4925_SIO_DEBUG_KDBG_WR_CH ( 1 << 31 )

#define TX4925_SIO_DEBUG_ALL        0xffffffff
#endif

#ifdef TX4925_SIO_DEBUG
static const u32 tx4925_sio_debug_flag = (TX4925_SIO_DEBUG_NONE
//                                       | TX4925_SIO_DEBUG_INFO
					  | TX4925_SIO_DEBUG_WARN
					  | TX4925_SIO_DEBUG_EROR
//                                       | TX4925_SIO_DEBUG_INIT
//                                       | TX4925_SIO_DEBUG_OPEN
//                                       | TX4925_SIO_DEBUG_CLOSE
//                                       | TX4925_SIO_DEBUG_IRQ
//                                       | TX4925_SIO_DEBUG_IRQ_RX
//                                       | TX4925_SIO_DEBUG_IRQ_TX
//                                       | TX4925_SIO_DEBUG_TERMIOS
//                                       | TX4925_SIO_DEBUG_IOCTL
//                                       | TX4925_SIO_DEBUG_THROTTLE
//                                       | TX4925_SIO_DEBUG_BREAK
//                                       | TX4925_SIO_DEBUG_CON_WAIT
//                                       | TX4925_SIO_DEBUG_CON_WR
//                                       | TX4925_SIO_DEBUG_CON_WR_CH
//                                       | TX4925_SIO_DEBUG_CON_DEV
//                                       | TX4925_SIO_DEBUG_CON_SETUP
//                                       | TX4925_SIO_DEBUG_CON_INIT
//                                       | TX4925_SIO_DEBUG_KDBG
//                                       | TX4925_SIO_DEBUG_KDBG_RD
//                                       | TX4925_SIO_DEBUG_KDBG_RD_CH
//                                       | TX4925_SIO_DEBUG_KDBG_WR
//                                       | TX4925_SIO_DEBUG_KDBG_WR_CH
    );
#endif

#ifdef TX4925_SIO_DEBUG
#define TX4925_SIO_DPRINTK(flag,str...) \
        if ( tx4925_sio_debug_flag&flag ) \
        { \
           char tmp[100]; \
           sprintf( tmp, str ); \
           printk( "%s(%s:%u)::%s", __FUNCTION__, __FILE__, __LINE__, tmp ); \
        }
#else
#define TX4925_SIO_DPRINTK(flag,str...)
#endif

static struct real_driver tx4925_sio_real_driver = {
	disable_tx_interrupts:tx4925_sio_disable_tx_interrupts,
	enable_tx_interrupts:tx4925_sio_enable_tx_interrupts,
	disable_rx_interrupts:tx4925_sio_disable_rx_interrupts,
	enable_rx_interrupts:tx4925_sio_enable_rx_interrupts,
	get_CD:tx4925_sio_get_CD,
	shutdown_port:tx4925_sio_shutdown_port,
	set_real_termios:tx4925_sio_set_real_termios,
	chars_in_buffer:tx4925_sio_chars_in_buffer,
	close:tx4925_sio_close,
	hungup:tx4925_sio_hungup,
	read_proc_port_count:tx4925_sio_info_port_count,
	read_proc_driver_name:tx4925_sio_info_driver_name,
	read_proc_driver_version:tx4925_sio_info_driver_version,
	read_proc_line:tx4925_sio_info_line
};

#define TX4925_SIO_DRIVER_NAME           "tx4925_sio"
#define TX4925_SIO_PORT_CFG_TX_FIFO_MAX  tx_fifo_max:8
#define TX4925_SIO_PORT_CFG_RX_FIFO_MAX  rx_fifo_max:16
#define TX4925_SIO_PORT_CFG(cfg_base,cfg_irq,cfg_irq_name) \
        TX4925_SIO_PORT_CFG_TX_FIFO_MAX,      \
        TX4925_SIO_PORT_CFG_RX_FIFO_MAX,      \
        base:cfg_base, size:36, irq:cfg_irq, irq_name:TX4925_SIO_DRIVER_NAME"/"cfg_irq_name

static struct tx4925_sio_sw_st tx4925_sio_hw_table[] = {
	{TX4925_SIO_PORT_CFG(TX4925_SIO_BASE_SIO0, 36, "0")},
	{TX4925_SIO_PORT_CFG(TX4925_SIO_BASE_SIO1, 37, "1")}
};
#define TX4925_SIO_NUM_PORTS (sizeof(tx4925_sio_hw_table)/sizeof(struct serial_state))

#define TX4925_SIO_MINOR_START 64

static char *tx4925_sio_driver_name = "TX4925 SIO Driver";
static char *tx4925_sio_driver_version = "0.0";

static struct tty_driver tx4925_sio_driver;
static struct tty_driver tx4925_sio_callout;
static int tx4925_sio_refcount;
static int tx4925_sio_initialized = 0;

static struct tty_struct *tx4925_sio_table[TX4925_SIO_NUM_PORTS];
static struct termios *tx4925_sio_termios[TX4925_SIO_NUM_PORTS];
static struct termios *tx4925_sio_termios_locked[TX4925_SIO_NUM_PORTS];

/******************************************************************************/
/* END: TODO                                                                  */
/******************************************************************************/

/******************************************************************************/
/* BEG: Driver Debug Routines                                                 */
/******************************************************************************/

#ifdef TX4925_SIO_DEBUG
void
tx4925_dump_regs(tx4925_sio_hw_st * base)
{
	printk("base    = 0x%08x 0x%08x\n", (u32) & base, (u32) base);
	printk("silcr   = 0x%08x 0x%08x\n", (u32) & base->silcr,
	       (u32) base->silcr);
	printk("sidicr  = 0x%08x 0x%08x\n", (u32) & base->sidicr,
	       (u32) base->sidicr);
	printk("sidisr  = 0x%08x 0x%08x\n", (u32) & base->sidisr,
	       (u32) base->sidisr);
	printk("siscisr = 0x%08x 0x%08x\n", (u32) & base->siscisr,
	       (u32) base->siscisr);
	printk("sifcr   = 0x%08x 0x%08x\n", (u32) & base->sifcr,
	       (u32) base->sifcr);
	printk("siflcr  = 0x%08x 0x%08x\n", (u32) & base->siflcr,
	       (u32) base->siflcr);
	printk("sibgr   = 0x%08x 0x%08x\n", (u32) & base->sibgr,
	       (u32) base->sibgr);
	return;
}
#endif

#ifdef TX4925_SIO_DEBUG
void
tx4925_sio_dump_regs(unsigned int port_number)
{
	if (port_number < TX4925_SIO_NUM_PORTS) {
		tx4925_dump_regs((tx4925_sio_hw_st *)
				 tx4925_sio_hw_table[port_number].base);
	} else {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_EROR,
				   "Bad Port Number [%d] != [0j.%u]\n",
				   port_number, TX4925_SIO_NUM_PORTS - 1);
	}
	return;
}
#endif

#ifdef TX4925_SIO_DEBUG
void
tx4925_dump_regs_sio0(void)
{
	tx4925_sio_dump_regs(0);
}
#endif

#ifdef TX4925_SIO_DEBUG
void
tx4925_dump_regs_sio1(void)
{
	tx4925_sio_dump_regs(1);
}
#endif

/******************************************************************************/
/* END: Driver Debug Routines                                                 */
/******************************************************************************/

/******************************************************************************/
/* BEG: Misc                                                                  */
/******************************************************************************/

int
tx4925_sio_info_port_count(void *ptr)
{
	return (TX4925_SIO_NUM_PORTS);
}

char *
tx4925_sio_info_driver_name(void *ptr)
{
	return (tx4925_sio_driver_name);
}

char *
tx4925_sio_info_driver_version(void *ptr)
{
	return (tx4925_sio_driver_version);
}

int
tx4925_sio_info_line(void *ptr, char *buf, int port_number, int max)
{
	int len = 0;
	tx4925_sio_sw_st *sp;
	vu32 siflcr;
	vu32 siscisr;
	vu32 sibgr;
	int baud = -1;
	char status[100];
	char tmpbuf[100];
	int tmplen;

	sp = tx4925_sio_hw_table + port_number;

	GS_SPRINTF(tmpbuf, tmplen, max, buf, len,
		   "%d: uart:%s port:0x%08x irq:%ld", port_number,
		   TX4925_SIO_DRIVER_NAME, (u32) sp->base, sp->irq);

	sibgr = tx4925_sio_rd(sp->base, TX4925_SIO_SIBGR);
	sibgr &= (TX4925_SIO_SIBGR_BCLK_MASK | TX4925_SIO_SIBGR_BRD_MASK);
	switch (sibgr) {
	case TX4925_SIO_BAUD_000110:{
			baud = 110;
			break;
		}
	case TX4925_SIO_BAUD_000150:{
			baud = 150;
			break;
		}
	case TX4925_SIO_BAUD_000300:{
			baud = 300;
			break;
		}
	case TX4925_SIO_BAUD_000600:{
			baud = 600;
			break;
		}
	case TX4925_SIO_BAUD_001200:{
			baud = 1200;
			break;
		}
	case TX4925_SIO_BAUD_002400:{
			baud = 2400;
			break;
		}
	case TX4925_SIO_BAUD_004800:{
			baud = 4800;
			break;
		}
	case TX4925_SIO_BAUD_009600:{
			baud = 9600;
			break;
		}
	case TX4925_SIO_BAUD_014400:{
			baud = 14400;
			break;
		}
	case TX4925_SIO_BAUD_019200:{
			baud = 19200;
			break;
		}
	case TX4925_SIO_BAUD_028800:{
			baud = 28800;
			break;
		}
	case TX4925_SIO_BAUD_038400:{
			baud = 38400;
			break;
		}
	case TX4925_SIO_BAUD_057600:{
			baud = 57600;
			break;
		}
	case TX4925_SIO_BAUD_076800:{
			baud = 76800;
			break;
		}
	case TX4925_SIO_BAUD_115200:{
			baud = 115200;
			break;
		}
	}
	GS_SPRINTF(tmpbuf, tmplen, max, buf, len, " baud:%d", baud);

	GS_SPRINTF(tmpbuf, tmplen, max, buf, len, " tx:%d", sp->icount.tx);
	GS_SPRINTF(tmpbuf, tmplen, max, buf, len, " rx:%d", sp->icount.rx);

	status[0] = '\0';
	status[1] = '\0';
	siflcr = tx4925_sio_rd(sp->base, TX4925_SIO_SIFLCR);
	if ((siflcr & TX4925_SIO_SIFLCR_RTSSC) == 0) {
		strcat(status, "|RTS");
	}
	siscisr = tx4925_sio_rd(sp->base, TX4925_SIO_SISCISR);
	if ((siscisr & TX4925_SIO_SISCISR_CTSS) == 0) {
		strcat(status, "|CTSS");
	}
	GS_SPRINTF(tmpbuf, tmplen, max, buf, len, " %s", status + 1);

	if (sp->icount.frame) {
		GS_SPRINTF(tmpbuf, tmplen, max, buf, len, " fe:%d",
			   sp->icount.frame);
	}

	if (sp->icount.parity) {
		GS_SPRINTF(tmpbuf, tmplen, max, buf, len, " pe:%d",
			   sp->icount.parity);
	}

	if (sp->icount.brk) {
		GS_SPRINTF(tmpbuf, tmplen, max, buf, len, " brk:%d",
			   sp->icount.brk);
	}

	if (sp->icount.overrun) {
		GS_SPRINTF(tmpbuf, tmplen, max, buf, len, " oe:%d",
			   sp->icount.overrun);
	}

	GS_SPRINTF(tmpbuf, tmplen, max, buf, len, "\n");

	return (len);
}

void
tx4925_sio_bits_set_clr(tx4925_sio_hw_st * base, u32 offset, u32 set, u32 clr)
{
	u32 val;

	val = tx4925_sio_rd(base, offset);
	val |= (set);
	val &= (~clr);
	tx4925_sio_wr(base, offset, val);

	return;
}

static void
tx4925_sio_hungup(void *ptr)
{
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CLOSE, "-\n");

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CLOSE, "=hangup\n");

	MOD_DEC_USE_COUNT;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CLOSE, "+\n");

	return;
}

static void
tx4925_sio_reset_fifo(tx4925_sio_sw_st * sp)
{
	vu32 sifcr;
	tx4925_sio_hw_st *base = sp->base;

	/* Start FIFO reset */
	sifcr = tx4925_sio_rd(base, TX4925_SIO_SIFCR);
	sifcr |=
	    (TX4925_SIO_SIFCR_TFRST | TX4925_SIO_SIFCR_RFRST |
	     TX4925_SIO_SIFCR_FRSTE);
	tx4925_sio_wr(base, TX4925_SIO_SIFCR, sifcr);

	/* Stop FIFO reset */
	sifcr = tx4925_sio_rd(base, TX4925_SIO_SIFCR);
	sifcr &=
	    ~(TX4925_SIO_SIFCR_TFRST | TX4925_SIO_SIFCR_RFRST |
	      TX4925_SIO_SIFCR_FRSTE);
	tx4925_sio_wr(base, TX4925_SIO_SIFCR, sifcr);

	return;
}

/******************************************************************************/
/* END: Misc                                                                  */
/******************************************************************************/

/******************************************************************************/
/* BEG: Driver Read Routines                                                  */
/******************************************************************************/

/* Read a serial register */
u32
tx4925_sio_rd(tx4925_sio_hw_st * base, int offset)
{
	u32 val;

	val = TX4925_SIO_RD(base, offset);

	return (val);
}

/* return the number of bytes/block in rx fifo (0=empty) */
u32
tx4925_sio_rx_fifo_ready(tx4925_sio_hw_st * base)
{
	u32 ready;

	ready = tx4925_sio_rd(base, TX4925_SIO_SIDISR);
	ready &= TX4925_SIO_SIDISR_RFDN;

	return (ready);
}

/* non-blocking read of rx fifo */
u32
tx4925_sio_rx_fifo_rd(tx4925_sio_hw_st * base)
{
	u32 val;

	val = tx4925_sio_rd(base, TX4925_SIO_SIRFIFO);

	tx4925_sio_bits_set_clr(base, TX4925_SIO_SIDISR, 0,
				TX4925_SIO_SIDISR_RDIS);

	return (val);
}

/* blocking read of rx fifo */
u32
tx4925_sio_rx_fifo_ready_rd(tx4925_sio_hw_st * base)
{
	u32 val;

	while (1) {
		val = tx4925_sio_rx_fifo_ready(base);
		if (val) {
			break;
		}
	}

	val = tx4925_sio_rx_fifo_rd(base);

	return (val);
}

/******************************************************************************/
/* END: Driver Read Routines                                                  */
/******************************************************************************/

/******************************************************************************/
/* BEG: Driver Write Routines                                                 */
/******************************************************************************/

void
tx4925_sio_wr(tx4925_sio_hw_st * base, int offset, u32 value)
{
	TX4925_SIO_WR(base, offset, value);
	return;
}

u32
tx4925_sio_tx_fifo_empty(tx4925_sio_hw_st * base)
{
	u32 empty;

	empty = tx4925_sio_rd(base, TX4925_SIO_SISCISR);
	empty &= TX4925_SIO_SISCISR_TXALS;

	return (empty);
}

void
tx4925_sio_tx_fifo_wait(tx4925_sio_hw_st * base)
{
	u32 reg;

	while (1) {
		reg = tx4925_sio_rd(base, TX4925_SIO_SISCISR);
		if (reg & TX4925_SIO_SISCISR_TRDY) {
			break;
		}
	}

	return;
}

void
tx4925_sio_tx_fifo_flush(tx4925_sio_hw_st * base)
{
	u32 empty;

	while (1) {
		empty = tx4925_sio_tx_fifo_empty(base);
		if (empty) {
			break;
		}
	}

	return;
}

void
tx4925_sio_tx_fifo_wait_wr_flush(tx4925_sio_hw_st * base, u8 ch)
{
	tx4925_sio_tx_fifo_wait(base);
	tx4925_sio_wr(base, TX4925_SIO_SITFIFO, (u32) ch);
	tx4925_sio_tx_fifo_flush(base);
	return;
}

/******************************************************************************/
/* END: Driver Write Routines                                                 */
/******************************************************************************/

/******************************************************************************/
/* BEG: Generic Serial Interface Routines                                     */
/******************************************************************************/

static void
tx4925_sio_disable_tx_interrupts(void *ptr)
{
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) ptr;
	unsigned long flags;

	save_and_cli(flags);

	sp->gs.flags &= ~GS_TX_INTEN;

	tx4925_sio_bits_set_clr(sp->base, TX4925_SIO_SIDICR, 0,
				TX4925_SIO_SIDICR_TIE);
	tx4925_sio_bits_set_clr(sp->base, TX4925_SIO_SIDISR, 0,
				TX4925_SIO_SIDISR_TDIS);

	restore_flags(flags);

	return;
}

static void
tx4925_sio_enable_tx_interrupts(void *ptr)
{
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) ptr;
	unsigned long flags;

	save_and_cli(flags);

	tx4925_sio_bits_set_clr(sp->base, TX4925_SIO_SIDICR,
				TX4925_SIO_SIDICR_TIE, 0);

	restore_flags(flags);

	return;
}

static void
tx4925_sio_disable_rx_interrupts(void *ptr)
{
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) ptr;
	unsigned long flags;

	save_and_cli(flags);

	tx4925_sio_bits_set_clr(sp->base, TX4925_SIO_SIDICR, 0,
				TX4925_SIO_SIDICR_RIE);
	tx4925_sio_bits_set_clr(sp->base, TX4925_SIO_SIDISR, 0,
				TX4925_SIO_SIDISR_RDIS);

	restore_flags(flags);

	return;
}

static void
tx4925_sio_enable_rx_interrupts(void *ptr)
{
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) ptr;
	unsigned long flags;

	save_and_cli(flags);

	tx4925_sio_bits_set_clr(sp->base, TX4925_SIO_SIDICR,
				TX4925_SIO_SIDICR_RIE, 0);

	restore_flags(flags);

	return;
}

static int
tx4925_sio_get_CD(void *ptr)
{
	/* not sure what todo here */
	return (1);
}

static void
tx4925_sio_shutdown_port(void *ptr)
{
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) ptr;

	sp->gs.flags &= ~GS_ACTIVE;

	//tx4925_sio_wr( sp->base, TX4925_SIO_SIDICR, 0 );

	free_irq(sp->irq, sp);

	//tx4925_sio_reset_fifo( sp );

	return;
}

static int
tx4925_sio_set_real_termios(void *ptr)
{
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) ptr;
	u32 val_baud;
	u32 val_cntl;
	u32 org_baud;
	u32 org_cntl;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INFO, "-\n");
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "-\n");

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "=base:0x%08x\n",
			   (unsigned int) sp->base);
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "=baud:%d\n", sp->gs.baud);
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "=cntl:0x%08x\n",
			   (unsigned int) sp->gs.tty->termios->c_cflag);

	tx4925_sio_wr(sp->base, TX4925_SIO_SILCR, 0x20);	/* IMBUSCLK */
	tx4925_sio_wr(sp->base, TX4925_SIO_SIFLCR, 0x0);	/* disable flow control */

	org_baud = val_baud = tx4925_sio_rd(sp->base, TX4925_SIO_SIBGR);
	org_cntl = val_cntl = tx4925_sio_rd(sp->base, TX4925_SIO_SILCR);

#if 0
	printk("000110 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T6,
	       TX4925_SIO_SIBGR_BRD_000110, TX4925_SIO_BAUD_000110);
	printk("000150 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T6,
	       TX4925_SIO_SIBGR_BRD_000150, TX4925_SIO_BAUD_000150);
	printk("000300 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T6,
	       TX4925_SIO_SIBGR_BRD_000300, TX4925_SIO_BAUD_000300);
	printk("000600 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T4,
	       TX4925_SIO_SIBGR_BRD_000600, TX4925_SIO_BAUD_000600);
	printk("001200 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T4,
	       TX4925_SIO_SIBGR_BRD_001200, TX4925_SIO_BAUD_001200);
	printk("002400 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T2,
	       TX4925_SIO_SIBGR_BRD_002400, TX4925_SIO_BAUD_002400);
	printk("004800 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T2,
	       TX4925_SIO_SIBGR_BRD_004800, TX4925_SIO_BAUD_004800);
	printk("009600 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T6,
	       TX4925_SIO_SIBGR_BRD_009600, TX4925_SIO_BAUD_009600);
	printk("014400 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T6,
	       TX4925_SIO_SIBGR_BRD_014400, TX4925_SIO_BAUD_014400);
	printk("019200 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T6,
	       TX4925_SIO_SIBGR_BRD_019200, TX4925_SIO_BAUD_019200);
	printk("028800 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T6,
	       TX4925_SIO_SIBGR_BRD_028800, TX4925_SIO_BAUD_028800);
	printk("038400 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T6,
	       TX4925_SIO_SIBGR_BRD_038400, TX4925_SIO_BAUD_038400);
	printk("057600 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T6,
	       TX4925_SIO_SIBGR_BRD_057600, TX4925_SIO_BAUD_057600);
	printk("076800 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T6,
	       TX4925_SIO_SIBGR_BRD_076800, TX4925_SIO_BAUD_076800);
	printk("115200 0x%08x=c 0x%08x=b 0x%08x=r\n", TX4925_SIO_SIBGR_BCLK_T6,
	       TX4925_SIO_SIBGR_BRD_115200, TX4925_SIO_BAUD_115200);
#endif

	if (sp->gs.baud) {
		val_baud &= ~(TX4925_SIO_BAUD_MASK);
		switch (sp->gs.baud) {
		case 110:{
				val_baud |= TX4925_SIO_BAUD_000110;
				break;
			}
		case 150:{
				val_baud |= TX4925_SIO_BAUD_000150;
				break;
			}
		case 300:{
				val_baud |= TX4925_SIO_BAUD_000300;
				break;
			}
		case 600:{
				val_baud |= TX4925_SIO_BAUD_000600;
				break;
			}
		case 1200:{
				val_baud |= TX4925_SIO_BAUD_001200;
				break;
			}
		case 2400:{
				val_baud |= TX4925_SIO_BAUD_002400;
				break;
			}
		case 4800:{
				val_baud |= TX4925_SIO_BAUD_004800;
				break;
			}
		case 9600:{
				val_baud |= TX4925_SIO_BAUD_009600;
				break;
			}
		case 14400:{
				val_baud |= TX4925_SIO_BAUD_014400;
				break;
			}
		case 19200:{
				val_baud |= TX4925_SIO_BAUD_019200;
				break;
			}
		case 28800:{
				val_baud |= TX4925_SIO_BAUD_028800;
				break;
			}
		case 38400:{
				val_baud |= TX4925_SIO_BAUD_038400;
				break;
			}
		case 57600:{
				val_baud |= TX4925_SIO_BAUD_057600;
				break;
			}
		case 76800:{
				val_baud |= TX4925_SIO_BAUD_076800;
				break;
			}
		case 115200:{
				val_baud |= TX4925_SIO_BAUD_115200;
				break;
			}
		default:{
				val_baud = org_baud;
				break;
			}
		}
	}

	if (sp->gs.tty->termios->c_cflag & CSIZE) {
		if (sp->gs.tty->termios->c_cflag == CS7) {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "=D7\n");
			val_cntl |= TX4925_SIO_SILCR_UMODE_DATA;
		} else {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "=D8\n");
			val_cntl &= ~(TX4925_SIO_SILCR_UMODE_DATA);
		}
	}

	if (C_CSTOPB(sp->gs.tty)) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "=S2\n");
		val_cntl |= TX4925_SIO_SILCR_USBL;
	} else {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "=S1\n");
		val_cntl &= ~(TX4925_SIO_SILCR_USBL);
	}

	if (C_PARENB(sp->gs.tty)) {
		val_cntl |= TX4925_SIO_SILCR_UPEN;
		if (C_PARODD(sp->gs.tty)) {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "=po\n");
			val_cntl &= ~(TX4925_SIO_SILCR_UEPS);
		} else {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "=pe\n");
			val_cntl |= TX4925_SIO_SILCR_UEPS;
		}
	} else {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "=pn\n");
		val_cntl &= ~(TX4925_SIO_SILCR_UPEN);
	}

	if (org_baud != val_baud) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS,
				   "=UpdatingBaud:0x%08x (was=0x%08x)\n",
				   val_baud, org_baud);
		tx4925_sio_wr(sp->base, TX4925_SIO_SIBGR, val_baud);
	}

	if (org_cntl != val_cntl) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS,
				   "=UpdatingCntl:0x%08x (was=0x%08x)\n",
				   val_cntl, org_cntl);
		tx4925_sio_wr(sp->base, TX4925_SIO_SILCR, val_cntl);
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_TERMIOS, "+\n");
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INFO, "+\n");

	return (0);
}

static int
tx4925_sio_chars_in_buffer(void *ptr)
{
	int count;
	int is_empty;

	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) ptr;

	is_empty = tx4925_sio_tx_fifo_empty(sp->base);

	if (is_empty) {
		count = 0;
	} else {
		count = 1;
	}

	return (count);
}

static void
tx4925_sio_close(void *ptr)
{
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CLOSE, "-\n");

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CLOSE, "=close\n");

	MOD_DEC_USE_COUNT;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CLOSE, "+\n");

	return;
}

static void
tx4925_sio_errata_reset_delay(void)
{
	/* must do bogus read of another imb module after starting sio reset */
	/* 4 cycle delay requirement -- at 8 cycles this pic read will do */
	TX4925_RD(0xff1ff600);
	return;
}

static void
tx4925_sio_reset_chip(tx4925_sio_sw_st * sp)
{
	vu32 sifcr;
	tx4925_sio_hw_st *base = sp->base;

	/* Reset the chip */
	sifcr = tx4925_sio_rd(base, TX4925_SIO_SIFCR);
	sifcr |= (TX4925_SIO_SIFCR_SWRST);
	tx4925_sio_wr(base, TX4925_SIO_SIFCR, sifcr);
	tx4925_sio_errata_reset_delay();

	/* Reset the fifos */
	tx4925_sio_reset_fifo(sp);

	/* put things back the way we expect them */
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INFO, "-\n");
	tx4925_sio_set_real_termios(sp);
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INFO, "+\n");

	return;
}

/******************************************************************************/
/* END: Generic Serial Interface Routines                                     */
/******************************************************************************/

/******************************************************************************/
/* BEG: Driver Interrupt Handler Routines                                     */
/******************************************************************************/

static void
tx4925_sio_receive_chars(tx4925_sio_sw_st * sp)
{
	tx4925_sio_hw_st *base = sp->base;
	struct tty_struct *tty = sp->gs.tty;
	u32 status;
	u32 data;
	unsigned int i;
	unsigned char ch;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_RX, "-\n");

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_RX, "=(rx=%u,%u)\n",
			   sp->rx_fifo_max, (sp->rx_fifo_max << 1));

	/* only try twice the max fifo size */
	for (i = 0; i < (sp->rx_fifo_max << 1); i++) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_RX, "=LoopTop\n");

		status = tx4925_sio_rd(base, TX4925_SIO_SIDISR);
		if (status & TX4925_SIO_SIDISR_RFDN) {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_RX,
					   "=RFDN (count=%u)\n",
					   (status & TX4925_SIO_SIDISR_RFDN));
			data = tx4925_sio_rd(base, TX4925_SIO_SIRFIFO);
			if (tty->flip.count < TTY_FLIPBUF_SIZE) {
				TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_RX,
						   "=ReadChar\n");
				ch = data & 0x000000ff;
				TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_RX,
						   "=ReadChar=[0x%02x/%03d/%c]\n",
						   ch, ch, ch);
				*tty->flip.char_buf_ptr++ = ch;
				*tty->flip.flag_buf_ptr++ = 0;
				tty->flip.count++;
				sp->icount.rx++;
				continue;
			}
		}

		tx4925_sio_bits_set_clr(base, TX4925_SIO_SIDISR, 0,
					TX4925_SIO_SIDISR_RDIS);

		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_RX, "=LoopBreak\n");
		break;
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_RX, "=Push\n");
	tty_flip_buffer_push(tty);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_RX, "+\n");
	return;
}

static void
tx4925_sio_transmit_chars(tx4925_sio_sw_st * sp)
{
	tx4925_sio_hw_st *base = sp->base;
	struct tty_struct *tty = sp->gs.tty;
	u32 status;
	u32 data;
	unsigned int i;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX, "-\n");

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX, "=(tx=%u,%u)\n",
			   sp->tx_fifo_max, (sp->tx_fifo_max << 1));

	for (i = 0; i < (sp->tx_fifo_max << 1); i++) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX, "=LoopTop\n");

		status = tx4925_sio_rd(base, TX4925_SIO_SISCISR);
		if (status & TX4925_SIO_SISCISR_TRDY) {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX, "=TRDY\n");

			if (sp->x_char) {
				data = sp->x_char;
				TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX,
						   "=TX-xchar=[0x%02x/%03d/%c]\n",
						   data, data, data);
				tx4925_sio_wr(base, TX4925_SIO_SITFIFO,
					      (u32) data);
				sp->icount.tx++;
				sp->x_char = 0;
			} else if ((sp->gs.xmit_cnt <= 0) ||
				   (sp->gs.tty->stopped) ||
				   (sp->gs.tty->hw_stopped)) {
				TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX,
						   "=TX-done\n");
				break;
			} else {
				data = sp->gs.xmit_buf[sp->gs.xmit_tail++];
				TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX,
						   "=TX-char=[0x%02x/%03d/%c]\n",
						   data, data, data);
				tx4925_sio_wr(base, TX4925_SIO_SITFIFO,
					      (u32) data);
				sp->icount.tx++;
				sp->gs.xmit_tail &= SERIAL_XMIT_SIZE - 1;
				if (--sp->gs.xmit_cnt <= 0) {
					break;
				}
			}
		}
	}

	if ((sp->gs.xmit_cnt <= 0) ||
	    (sp->gs.tty->stopped) || (sp->gs.tty->hw_stopped)) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX,
				   "=TX-rs_disable_tx_interrupts\n");
		tx4925_sio_disable_tx_interrupts(sp);
	}

	if (sp->gs.xmit_cnt <= sp->gs.wakeup_chars) {
		if ((sp->gs.tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
		    (sp->gs.tty->ldisc.write_wakeup)) {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX,
					   "=TX-write_wakeup\n");
			(sp->gs.tty->ldisc.write_wakeup) (sp->gs.tty);
		}
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX,
				   "=TX-wake_up_interruptible\n");
		wake_up_interruptible(&sp->gs.tty->write_wait);
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX, "=Push\n");
	tty_flip_buffer_push(tty);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ_TX, "+\n");

	return;
}

static void
tx4925_sio_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned long flags;
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) dev_id;
	tx4925_sio_hw_st *base = sp->base;
	u32 status;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ, "-\n");

	save_and_cli(flags);

	if (!sp || !sp->gs.tty) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_EROR, "!sp or !tty\n");
		restore_flags(flags);
		return;
	}

	status = tx4925_sio_rd(sp->base, TX4925_SIO_SIDISR);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ, "=[status=0x%08x]\n", status);

	if (status & TX4925_SIO_SIDISR_TOUT) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ, "=Timeout\n");
		tx4925_sio_bits_set_clr(base, TX4925_SIO_SIDISR, 0,
					TX4925_SIO_SIDISR_TOUT);
		status = tx4925_sio_rd(base, TX4925_SIO_SIDISR);
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ, "=Timeout [0x%08x]\n",
				   status);
	}

	if (status & TX4925_SIO_SIDISR_UBRK) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ, "=Break\n");
		/* mvmcp todo ??? */
	}

	if (status & TX4925_SIO_SIDISR_UFER) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ, "=FrameError\n");
		/* mvmcp todo ??? */
	}

	if (status & TX4925_SIO_SIDISR_UPER) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ, "=ParityError\n");
		/* mvmcp todo ??? */
	}

	if (status & TX4925_SIO_SIDISR_UOER) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ, "=OverrunError\n");
		/* mvmcp todo ??? */
	}

	/* receive bytes */
	if (status & TX4925_SIO_SIDISR_RDIS) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ, "=Rx\n");
		tx4925_sio_receive_chars(sp);
	}

	tx4925_sio_transmit_chars(sp);

	restore_flags(flags);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IRQ, "+\n");

	return;
}

/******************************************************************************/
/* END: Driver Interrupt Handler Routines                                     */
/******************************************************************************/

/******************************************************************************/
/* BEG: Driver Interface Routines                                             */
/******************************************************************************/

static int
tx4925_sio_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd,
		 unsigned long arg)
{
	int rc = 0;
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) tty->driver_data;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IOCTL, "-\n");

	switch (cmd) {
	case TIOCMSET:
		{
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IOCTL,
					   "TIOCMSET [0x%08x]\n", cmd);
			rc = put_user(((tty->termios->
					c_cflag & CLOCAL) ? 1 : 0),
				      ((unsigned int *) arg));
			break;
		}

	case TIOCSSOFTCAR:
		{
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IOCTL,
					   "TIOCSSOFTCAR [0x%08x]\n", cmd);
			break;
		}

	case TIOCGSERIAL:
		{
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IOCTL,
					   "TIOCGSERIAL [0x%08x]\n", cmd);
			rc = verify_area(VERIFY_WRITE, (void *) arg,
					 sizeof (struct serial_struct));
			if (rc == 0) {
				gs_getserial(&sp->gs,
					     (struct serial_struct *) arg);
			}
			break;
		}

	case TIOCSSERIAL:
		{
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IOCTL,
					   "TIOCSSERIAL [0x%08x]\n", cmd);
			rc = verify_area(VERIFY_READ, (void *) arg,
					 sizeof (struct serial_struct));
			if (rc == 0) {
				rc = gs_setserial(&sp->gs,
						  (struct serial_struct *) arg);
			}
			break;
		}

	default:
		{
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IOCTL,
					   "default [0x%08x]\n", cmd);
			rc = -ENOIOCTLCMD;
			break;
		}
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_IOCTL, "+\n");

	return (rc);
}

static void
tx4925_sio_send_xchar(struct tty_struct *tty, char ch)
{
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) tty->driver_data;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INFO, "-\n");

	sp->x_char = ch;
	if (ch) {
		tx4925_sio_enable_tx_interrupts(tty);
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INFO, "+\n");

	return;
}

static void
tx4925_sio_throttle(struct tty_struct *tty)
{
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) tty->driver_data;
	unsigned long flags;
	vu32 siflcr;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_THROTTLE, "-\n");

	if (I_IXOFF(tty)) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_THROTTLE,
				   "=SendingStop(0x%02x)\n", STOP_CHAR(tty));
		tx4925_sio_send_xchar(tty, STOP_CHAR(tty));
	}

	if (tty->termios->c_cflag & CRTSCTS) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_THROTTLE, "=CRTSCTS\n");

		save_and_cli(flags);

		siflcr = tx4925_sio_rd(sp->base, TX4925_SIO_SIFLCR);
		siflcr |= (TX4925_SIO_SIFLCR_RTSSC | TX4925_SIO_SIFLCR_RSDE);
		tx4925_sio_wr(sp->base, TX4925_SIO_SIFLCR, siflcr);

		restore_flags(flags);
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_THROTTLE, "+\n");
	return;
}

static void
tx4925_sio_unthrottle(struct tty_struct *tty)
{
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) tty->driver_data;
	unsigned long flags;
	vu32 siflcr;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_THROTTLE, "-\n");

	if (I_IXOFF(tty)) {
		if (sp->x_char) {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_THROTTLE,
					   "=Clearing\n");
			sp->x_char = 0;
		} else {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_THROTTLE,
					   "=SendingStart(0x%02x)\n",
					   START_CHAR(tty));
			tx4925_sio_send_xchar(tty, START_CHAR(tty));
		}
	}

	if (tty->termios->c_cflag & CRTSCTS) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_THROTTLE, "=CRTSCTS\n");

		save_and_cli(flags);

		siflcr = tx4925_sio_rd(sp->base, TX4925_SIO_SIFLCR);
		siflcr &= ~(TX4925_SIO_SIFLCR_RTSSC | TX4925_SIO_SIFLCR_RSDE);
		tx4925_sio_wr(sp->base, TX4925_SIO_SIFLCR, siflcr);

		restore_flags(flags);
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_THROTTLE, "+\n");

	return;
}

static void
tx4925_sio_break(struct tty_struct *tty, int break_state)
{
	tx4925_sio_sw_st *sp = (tx4925_sio_sw_st *) tty->driver_data;
	unsigned long flags;
	vu32 siflcr;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_BREAK, "-\n");

	save_and_cli(flags);

	siflcr = tx4925_sio_rd(sp->base, TX4925_SIO_SIFLCR);

	if (break_state == -1) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_BREAK, "+BreakDisable\n");
		siflcr |= (TX4925_SIO_SIFLCR_TBRK);
	} else {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_BREAK, "+BreakEnable\n");
		siflcr &= ~(TX4925_SIO_SIFLCR_TBRK);
	}

	tx4925_sio_wr(sp->base, TX4925_SIO_SIFLCR, siflcr);

	restore_flags(flags);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_BREAK, "+\n");

	return;
}

int
tx4925_sio_read_proc(char *page, char **start, off_t off, int count, int *eof,
		     void *data)
{
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INFO, "-\n");
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INFO, "+\n");
	return (0);
}

static int
tx4925_sio_open(struct tty_struct *tty, struct file *filp)
{
	int retval;
	unsigned int port_number;
	struct tx4925_sio_sw_st *sp;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_OPEN, "-\n");

	if (tx4925_sio_initialized == 0) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_EROR, "!Not Init\n");
		return (-EIO);
	}

	port_number = MINOR(tty->device) - tty->driver.minor_start;
	if (port_number >= TX4925_SIO_NUM_PORTS) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_EROR,
				   "Bad Port Number [%d] != [0j.%u]\n",
				   port_number, TX4925_SIO_NUM_PORTS - 1);
		return (-ENODEV);
	}

	sp = tx4925_sio_hw_table + port_number;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_OPEN, "Port#%u=0x%08x\n",
			   port_number, (u32) sp->base);

	tty->driver_data = sp;
	sp->gs.tty = tty;
	sp->gs.count++;

	retval = gs_init_port(&sp->gs);
	if (retval) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_EROR,
				   "gs_init_port() failed\n");
		sp->gs.count--;
		return (retval);
	}

	sp->gs.flags |= GS_ACTIVE;

	if (sp->gs.count == 1) {
		MOD_INC_USE_COUNT;

		/* Clear the interrupt registers */
		tx4925_sio_wr(sp->base, TX4925_SIO_SIDICR, 0);

		/* Reset UART */
		tx4925_sio_reset_chip(sp);

		retval =
		    request_irq(sp->irq, tx4925_sio_interrupt, SA_INTERRUPT,
				sp->irq_name, sp);
		if (retval) {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_EROR,
					   "request_irq(%ld)=%d\n", sp->irq,
					   retval);
			MOD_DEC_USE_COUNT;
			sp->gs.count--;
			return (retval);
		}
	}

	retval = gs_block_til_ready(&sp->gs, filp);
	if (retval) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_EROR,
				   "gs_block_til_ready()=%d\n", retval);
		if (sp->gs.count == 1) {
			MOD_DEC_USE_COUNT;
			free_irq(sp->irq, sp->irq_name);
		}
		sp->gs.count--;
		return (retval);
	}

	/* Enable interrupts */
	tx4925_sio_enable_rx_interrupts(sp);
	tx4925_sio_enable_tx_interrupts(sp);

	if ((sp->gs.count == 1) && (sp->gs.flags & ASYNC_SPLIT_TERMIOS)) {
		if (tty->driver.subtype == SERIAL_TYPE_NORMAL) {
			*tty->termios = sp->gs.normal_termios;
		} else {
			*tty->termios = sp->gs.callout_termios;
		}
		tx4925_sio_set_real_termios(sp);
	}

	sp->gs.session = current->session;
	sp->gs.pgrp = current->pgrp;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_OPEN, "+\n");

	return (0);
}

/******************************************************************************/
/* END: Driver Interface Routines                                             */
/******************************************************************************/

/******************************************************************************/
/* BEG: Driver Init Functions                                                 */
/******************************************************************************/

void __init
tx4925_sio_init(void)
{
	int i;
	int r;
	struct tx4925_sio_sw_st *sp;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT, "-\n");

	tx4925_sio_initialized = 1;

	printk("%s version %s\n", tx4925_sio_info_driver_name(NULL),
	       tx4925_sio_info_driver_version(NULL));

#ifdef TX4925_SIO_DEBUG
	{
		printk("tx4925_sio_debug_flag=0x%08x\n", tx4925_sio_debug_flag);
	}
#endif

	memset(&tx4925_sio_driver, 0, sizeof (struct tty_driver));

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT, "=Setting Normal Driver\n");
	tx4925_sio_driver.magic = TTY_DRIVER_MAGIC;
	tx4925_sio_driver.driver_name = TX4925_SIO_DRIVER_NAME;
#if (LINUX_VERSION_CODE > 0x2032D && defined(CONFIG_DEVFS_FS))
	{
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT, "=Setting DEVFS\n");
		tx4925_sio_driver.name = "ttyS%d";
	}
#else
	{
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT,
				   "=Setting NO DEVFS\n");
		tx4925_sio_driver.name = "ttyS";
	}
#endif
	tx4925_sio_driver.major = TTY_MAJOR;
	tx4925_sio_driver.minor_start = TX4925_SIO_MINOR_START;
	tx4925_sio_driver.num = TX4925_SIO_NUM_PORTS;
	tx4925_sio_driver.type = TTY_DRIVER_TYPE_SERIAL;
	tx4925_sio_driver.subtype = SERIAL_TYPE_NORMAL;
	tx4925_sio_driver.init_termios = tty_std_termios;
	tx4925_sio_driver.init_termios.c_cflag =
	    (B38400 | CS8 | CREAD | HUPCL | CLOCAL);
	tx4925_sio_driver.flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_NO_DEVFS;
	tx4925_sio_driver.refcount = &tx4925_sio_refcount;
	tx4925_sio_driver.table = tx4925_sio_table;
	tx4925_sio_driver.termios = tx4925_sio_termios;
	tx4925_sio_driver.termios_locked = tx4925_sio_termios_locked;

	tx4925_sio_driver.open = tx4925_sio_open;
	tx4925_sio_driver.close = gs_close;
	tx4925_sio_driver.write = gs_write;
	tx4925_sio_driver.put_char = gs_put_char;
	tx4925_sio_driver.flush_chars = gs_flush_chars;
	tx4925_sio_driver.write_room = gs_write_room;
	tx4925_sio_driver.chars_in_buffer = gs_chars_in_buffer;
	tx4925_sio_driver.flush_buffer = gs_flush_buffer;
	tx4925_sio_driver.set_termios = gs_set_termios;
	tx4925_sio_driver.stop = gs_stop;
	tx4925_sio_driver.start = gs_start;
	tx4925_sio_driver.hangup = gs_hangup;
	tx4925_sio_driver.read_proc = gs_read_proc;
	tx4925_sio_driver.ioctl = tx4925_sio_ioctl;
	tx4925_sio_driver.send_xchar = tx4925_sio_send_xchar;
	tx4925_sio_driver.throttle = tx4925_sio_throttle;
	tx4925_sio_driver.unthrottle = tx4925_sio_unthrottle;
	tx4925_sio_driver.break_ctl = tx4925_sio_break;
	tx4925_sio_driver.local_data = tx4925_sio_hw_table;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT, "=Setting Callout Driver\n");
	tx4925_sio_callout = tx4925_sio_driver;
#if (LINUX_VERSION_CODE > 0x2032D && defined(CONFIG_DEVFS_FS))
	{
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT, "=Setting DEVFS\n");
		tx4925_sio_callout.name = "cua%d";
	}
#else
	{
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT,
				   "=Setting NO DEVFS\n");
		tx4925_sio_callout.name = "cua";
	}
#endif
	tx4925_sio_callout.major = TTYAUX_MAJOR;
	tx4925_sio_callout.subtype = SERIAL_TYPE_CALLOUT;
	tx4925_sio_callout.read_proc = NULL;
	tx4925_sio_callout.proc_entry = NULL;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT, "=Registering Drivers\n");

	if (tty_register_driver(&tx4925_sio_driver)) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_EROR,
				   "Couldn't register TX4925_SIO normal driver\n");
		tx4925_sio_initialized = 0;
	}

	if (tty_register_driver(&tx4925_sio_callout)) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_EROR,
				   "Couldn't register TX4925_SIO callout driver\n");
		tx4925_sio_initialized = 0;
	}

	for (i = 0; i < TX4925_SIO_NUM_PORTS; i++) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT,
				   "=Setting State [i=%u]\n", i);

		/* get the port */
		sp = tx4925_sio_hw_table + i;

		/* see if its free */
		r = check_mem_region((unsigned long) sp->base, sp->size);
		if (r) {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT,
					   "=check_mem_region() failed with %d\n",
					   r);
			continue;
		}

		/* base,irq,flags hard-code in definition */
		sp->gs.callout_termios = tty_std_termios;
		sp->gs.normal_termios = tty_std_termios;
		sp->gs.magic = SERIAL_MAGIC;
		sp->gs.close_delay = TX4925_SIO_CLOSE_DELAY;
		sp->gs.closing_wait = TX4925_SIO_CLOSE_WAIT;
		sp->gs.rd = &tx4925_sio_real_driver;

		tty_register_devfs(&tx4925_sio_driver, 0,
				   tx4925_sio_driver.minor_start + i);
		tty_register_devfs(&tx4925_sio_callout, 0,
				   tx4925_sio_callout.minor_start + i);

#ifdef NEW_WRITE_LOCKING
		{
			sp->gs.port_write_sem = MUTEX;
		}
#endif

#ifdef DECLARE_WAITQUEUE
		{
			init_waitqueue_head(&sp->gs.open_wait);
			init_waitqueue_head(&sp->gs.close_wait);
		}
#endif

		/* reserve it  */
		request_mem_region((unsigned long) sp->base, sp->size,
				   sp->irq_name);

		printk("ttyS%d at 0x%x (irq=%ld) is a %s\n", i, (u32) sp->base,
		       sp->irq, tx4925_sio_driver_name);
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT, "=Initialized=%d\n",
			   tx4925_sio_initialized);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_INIT, "+\n");
	return;
}

/******************************************************************************/
/* END: Driver Init Functions                                                 */
/******************************************************************************/

/******************************************************************************/
/* BEG: Modules                                                               */
/******************************************************************************/

#ifdef MODULE
int
init_module(void)
{
	return (tx4925_sio_init());
}
#endif

/******************************************************************************/
/* END: Modules                                                               */
/******************************************************************************/

/******************************************************************************/
/* BEG: Serial Console Routines                                               */
/******************************************************************************/

#ifdef CONFIG_TX4925_SIO_CONSOLE
static int
tx4925_sio_console_wait_key(struct console *co)
{
	u32 ch;
	u32 status;
	tx4925_sio_hw_st *tx4925_sio_console_base;
	u32 save_sidicr;

	tx4925_sio_console_base =
	    (tx4925_sio_hw_st *) tx4925_sio_hw_table[co->index].base;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_WAIT, "-[b=0x%08x,c=%d]\n",
			   (u32) tx4925_sio_console_base, co->index);

	save_sidicr = tx4925_sio_rd(tx4925_sio_console_base, TX4925_SIO_SIDICR);
	tx4925_sio_wr(tx4925_sio_console_base, TX4925_SIO_SIDICR, 0);

	while (1) {
		status =
		    tx4925_sio_rd(tx4925_sio_console_base, TX4925_SIO_SIDISR);
		if (status & TX4925_SIO_SIDISR_RFDN) {
			break;
		}
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_WAIT, "=Waiting\n");
	}

	ch = tx4925_sio_rd(tx4925_sio_console_base, TX4925_SIO_SIRFIFO);
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_WAIT, "=[ch=0x%02x/%03d/%c]\n",
			   ch, ch, ch);

	tx4925_sio_tx_fifo_flush(tx4925_sio_console_base);

	tx4925_sio_wr(tx4925_sio_console_base, TX4925_SIO_SIDICR, save_sidicr);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_WAIT, "+[b=0x%08x,c=%d]\n",
			   (u32) tx4925_sio_console_base, co->index);

	return (ch);
}
#endif

#ifdef CONFIG_TX4925_SIO_CONSOLE
static void
tx4925_sio_console_write(struct console *co, const char *s, unsigned count)
{
	unsigned int i;
	tx4925_sio_hw_st *tx4925_sio_console_base;
	u32 save_sidicr;

	tx4925_sio_console_base =
	    (tx4925_sio_hw_st *) tx4925_sio_hw_table[co->index].base;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_WR, "-[b=0x%08x,c=%d]\n",
			   (u32) tx4925_sio_console_base, co->index);

	save_sidicr = tx4925_sio_rd(tx4925_sio_console_base, TX4925_SIO_SIDICR);
	tx4925_sio_wr(tx4925_sio_console_base, TX4925_SIO_SIDICR, 0);

	for (i = 0; i < count; i++, s++) {
		if (*s == '\n') {
			tx4925_sio_tx_fifo_wait(tx4925_sio_console_base);
			tx4925_sio_wr(tx4925_sio_console_base,
				      TX4925_SIO_SITFIFO, (u32) '\r');
		}
		tx4925_sio_tx_fifo_wait(tx4925_sio_console_base);
		tx4925_sio_wr(tx4925_sio_console_base, TX4925_SIO_SITFIFO,
			      (u32) * s);
	}

	tx4925_sio_tx_fifo_flush(tx4925_sio_console_base);

	tx4925_sio_wr(tx4925_sio_console_base, TX4925_SIO_SIDICR, save_sidicr);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_WR, "+[b=0x%08x,c=%d]\n",
			   (u32) tx4925_sio_console_base, co->index);

	return;
}
#endif

#ifdef CONFIG_TX4925_SIO_CONSOLE
static kdev_t
tx4925_sio_console_device(struct console *co)
{
	kdev_t dev;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_DEV, "-[b=0x%08x,c=%d]\n", 0,
			   co->index);

	dev = MKDEV(TTY_MAJOR, TX4925_SIO_MINOR_START + co->index);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_DEV,
			   "=[dev(0x%02x,0x%02x+0x%02x)=0x%02x]\n", TTY_MAJOR,
			   TX4925_SIO_MINOR_START, co->index, dev);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_DEV, "+[b=0x%08x,c=%d]\n", 0,
			   co->index);

	return (dev);
}
#endif

#ifdef CONFIG_TX4925_SIO_CONSOLE
static int __init
tx4925_sio_console_setup(struct console *co, char *options)
{
	int retval = 0;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_SETUP, "-[b=0x%08x,c=%d]\n", 0,
			   co->index);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_SETUP, "=[options=%s]\n",
			   options);

	if ((co->index < 0) || (co->index >= TX4925_SIO_NUM_PORTS)) {
		retval = -1;
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_EROR,
				   "Bad Port Number [%d] != [0j.%u]\n",
				   co->index, TX4925_SIO_NUM_PORTS - 1);
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_SETUP, "+[b=0x%08x,c=%d]\n", 0,
			   co->index);

	return (retval);
}
#endif

#ifdef CONFIG_TX4925_SIO_CONSOLE
static struct console tx4925_sio_console_st = {
	name:"ttyS",
	wait_key:tx4925_sio_console_wait_key,
	write:tx4925_sio_console_write,
	device:tx4925_sio_console_device,
	setup:tx4925_sio_console_setup,
	flags:CON_PRINTBUFFER,
	index:-1,
};
#endif

#ifdef CONFIG_TX4925_SIO_CONSOLE
void __init
tx4925_sio_console_init(void)
{
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_INIT, "-\n");

	register_console(&tx4925_sio_console_st);

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_CON_INIT, "+\n");

	return;
}
#endif

/******************************************************************************/
/* END: Serial Console Routines                                               */
/******************************************************************************/

/******************************************************************************/
/* BEG: KDBG Routines                                                         */
/******************************************************************************/

#ifdef CONFIG_REMOTE_DEBUG
static tx4925_sio_hw_st *tx4925_sio_kdbg_base = 0;
#endif

#ifdef CONFIG_REMOTE_DEBUG
void
tx4925_sio_kdbg_init(unsigned int port_number)
{
	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG, "-[b=0x%08x]\n",
			   (u32) tx4925_sio_kdbg_base);

	if (port_number < TX4925_SIO_NUM_PORTS) {
		tx4925_sio_kdbg_base = tx4925_sio_hw_table[port_number].base;
	} else {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_EROR,
				   "Bad Port Number [%u] != [0j.%u]\n",
				   port_number, TX4925_SIO_NUM_PORTS - 1);
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG, "+[b=0x%08x]\n",
			   (u32) tx4925_sio_kdbg_base);

	return;
}
#endif

#ifdef CONFIG_REMOTE_DEBUG
u8
tx4925_sio_kdbg_rd(void)
{
	u32 status;
	u8 ch = 0;

	if (tx4925_sio_kdbg_base == 0) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG_RD,
				   "=Calling init()\n");
		tx4925_sio_kdbg_init(1);	/* MVMCP revist -- config item */
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG_RD, "-[b=0x%08x]\n",
			   (u32) tx4925_sio_kdbg_base);

	while (1) {
		status = tx4925_sio_rd(tx4925_sio_kdbg_base, TX4925_SIO_SIDISR);
		if (status & TX4925_SIO_SIDISR_RFDN) {
			ch = (u8) tx4925_sio_rd(tx4925_sio_kdbg_base,
						TX4925_SIO_SIRFIFO);

			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG_RD_CH,
					   "=found=%c\n", ch);

			break;
		}
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG_RD, "+[b=0x%08x]\n",
			   (u32) tx4925_sio_kdbg_base);

	return (ch);
}
#endif

#ifdef CONFIG_REMOTE_DEBUG
int
tx4925_sio_kdbg_wr(u8 ch)
{
	u32 status;

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG_WR, "-[b=0x%08x]\n",
			   (u32) tx4925_sio_kdbg_base);

	if (tx4925_sio_kdbg_base == 0) {
		TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG_WR,
				   "=Calling init()\n");
		tx4925_sio_kdbg_init(1);	/* MVMCP revist -- config item */
	}

	while (1) {
		status =
		    tx4925_sio_rd(tx4925_sio_kdbg_base, TX4925_SIO_SISCISR);
		if (status & TX4925_SIO_SISCISR_TRDY) {
			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG_WR_CH,
					   "=fifo has space\n");

			if (ch == '\n') {
				TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG_WR_CH,
						   "=forcing <CR>\n");
				tx4925_sio_kdbg_wr('\r');
			}

			TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG_WR_CH,
					   "=sending <%c>\n", ch);
			tx4925_sio_wr(tx4925_sio_kdbg_base, TX4925_SIO_SITFIFO,
				      (u32) ch);

			break;
		}
	}

	TX4925_SIO_DPRINTK(TX4925_SIO_DEBUG_KDBG_WR, "+[b=0x%08x]\n",
			   (u32) tx4925_sio_kdbg_base);

	return (1);
}
#endif

/******************************************************************************/
/* END: KDBG Routines                                                         */
/******************************************************************************/
