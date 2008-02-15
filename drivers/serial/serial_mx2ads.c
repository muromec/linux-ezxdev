/*
 *  linux/drivers/char/mx2ads.c
 *
 *  Driver for DragonBall serial ports
 *
 *  Copyright 2002 Sony Corporation.
 *  Copyright 2003, MontaVista Software, Inc  <source@mvista.com>
 *
 *  Original source based on linux/drivers/char/serial_sa1100.c
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/circ_buf.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/wait.h>
#include <linux/pm.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>
#include <asm/hardware.h>
#include <asm/pgalloc.h>

#include <linux/proc_fs.h>

#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/irq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/pll.h>

#if defined(CONFIG_SERIAL_MX2ADS_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/serial_core.h>

#if !defined(CONFIG_SERIAL_MX2ADS_UART1) && \
!defined(CONFIG_SERIAL_MX2ADS_UART2) && \
!defined(CONFIG_SERIAL_MX2ADS_UART3) && \
!defined(CONFIG_SERIAL_MX2ADS_UART4)
#error At least 1 MX2 UART port must be selected
#endif

#ifdef CONFIG_SERIAL_MX2ADS_UART1
#define MX2ADS_UART1 1
#else
#define MX2ADS_UART1 0
#endif

#ifdef CONFIG_SERIAL_MX2ADS_UART2
#define MX2ADS_UART2 1
#else
#define MX2ADS_UART2 0
#endif

#ifdef CONFIG_SERIAL_MX2ADS_UART3
#define MX2ADS_UART3 1
#else
#define MX2ADS_UART3 0
#endif

#ifdef CONFIG_SERIAL_MX2ADS_UART4
#define MX2ADS_UART4 1
#else
#define MX2ADS_UART4 0
#endif

#define NR_PORTS (MX2ADS_UART1 + MX2ADS_UART2 + MX2ADS_UART3 + MX2ADS_UART4)

/* We've been assigned a range on the "Low-density serial ports" major */
#define SERIAL_MX2ADS_MAJOR	        TTY_MAJOR	/* 204 */
#define CALLOUT_MX2ADS_MAJOR		205
#define MINOR_START		        64


/* default FIFO settings */
#define TX_LVL   16		/* generate int if tx fifo <= this level */
#define RX_LVL   1		/* generate int if rx fifo >= this level */

#define TX_MAX   16		/* put up to 16 chars at a time (fifo == 32 chars) */

#define DEBUG_NAME __FILE__

#undef UART_DEBUG

#ifdef UART_DEBUG
#define ENTRY() pr_debug( "%s entered\n", __FUNCTION__ )
#define EXIT() pr_debug(  "%s exited\n", __FUNCTION__ )
#else
#define ENTRY()
#define EXIT()
#endif


static int initstate_uart;
static int initstate_uart_irq[NR_PORTS];
#ifdef CONFIG_SERIAL_MX2ADS_UART1
static int initstate_region1;
static int initstate_gpio1;
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART2
static int initstate_region2;
static int initstate_gpio2;
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART3
static int initstate_region3;
static int initstate_gpio3;
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART4
static int initstate_region4;
static int initstate_gpio4;
#ifdef CONFIG_MX2TO2
static int initstate_gpio4_30;
static int initstate_gpio4_28;
#endif
#endif
static int irq_active[NR_PORTS];
struct uart_info * local_info[NR_PORTS];

/*
   Uart I/O functions
 */
static inline unsigned int
uart_in(struct uart_port *port, u_int offset)
{
	return __REG32_2(port->mapbase, offset);
}

static inline void
uart_out(struct uart_port *port, u_int offset, unsigned int value)
{
	__REG32_2(port->mapbase, offset) = value;
}

static inline void
uart_putc(struct uart_port *port, char c)
{
	__REG32_2(port->mapbase, UART_TXDATA) = (unsigned int) (c & 0xff);
}

static inline unsigned int
uart_getc(struct uart_port *port)
{
	unsigned int usr2;
	usr2 = uart_in(port, UART_SR2);
	if (!(usr2 & UART_RDR)) {
		return (0);
	} else {
		return __REG32_2(port->mapbase, UART_RXDATA);
	}
}

/* Test which uart */
static inline int
get_uart_id(struct uart_port *port)
{
	unsigned long membase = (unsigned long) port->membase;

	switch (membase) {
	case UART1_BASE:
		return 0;
	case UART2_BASE:
		return 1;
	case UART3_BASE:
		return 2;
	case UART4_BASE:
		return 3;
	default:
		return -1;
	}
}

static inline void
uart_init_timer(struct uart_port *port, struct timer_list *tl)
{
	init_timer(&tl[get_uart_id(port)]);
}

static inline int
uart_del_timer(struct uart_port *port, struct timer_list *tl)
{
	return del_timer(&tl[get_uart_id(port)]);
}

static inline void
uart_add_timer(struct uart_port *port, struct timer_list *tl,
	       void *func, unsigned long data, unsigned long expire)
{
	int id = get_uart_id(port);

	tl[id].function = func;
	tl[id].data = data;
	tl[id].expires = expire;
	add_timer(&tl[id]);
}

/*
 * This is the size of our serial port register set.
 */
#define UART_PORT_SIZE	SZ_4K

static struct tty_driver normal, callout;
static struct tty_struct *mx2ads_table[NR_PORTS];
static struct termios *mx2ads_termios[NR_PORTS],
    *mx2ads_termios_locked[NR_PORTS];
#ifdef SUPPORT_SYSRQ
static struct console mx2ads_console;
#endif

/*
 * interrupts disabled on entry
 */
static void
mx2ads_stop_tx(struct uart_port *port, u_int from_tty)
{
	unsigned int ucr1;

	ENTRY();
	ucr1 = uart_in(port, UART_CR1);
	uart_out(port, UART_CR1, ucr1 & ~(UART_TXEMPTYEN));
	EXIT();

}

/*
 * interrupts may not be disabled on entry
 */
static void
mx2ads_start_tx(struct uart_port *port, u_int nonempty, u_int from_tty)
{
	unsigned int ucr1;

	if (nonempty) {
		ucr1 = uart_in(port, UART_CR1);
		uart_out(port, UART_CR1, ucr1 | (UART_TXEMPTYEN));
	}
}

/*
 * Interrupts enabled
 */
static void
mx2ads_stop_rx(struct uart_port *port)
{
}

/*
 * enable RTS delta interrupts
 */
static void
mx2ads_enable_ms(struct uart_port *port)
{
	ENTRY();
	uart_out(port, UART_CR1, uart_in(port, UART_CR1) | UART_RTSDEN);
	EXIT();
}

/*
 * called within interrupt handler
 */
static inline int
mx2ads_rx_chars(struct uart_info *info)
{
	struct uart_port *port = info->port;
	struct tty_struct *tty = info->tty;
	unsigned int rx;

	if (!tty)
		return -ENODEV;

	/* Check to not overflow buffer */

	while ((rx = uart_getc(port)) & UART_CHARRDY) {

		if (tty->flip.count >= TTY_FLIPBUF_SIZE) {

			tty->flip.tqueue.routine((void *) tty);
			if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
				continue;
			}
		}

		*tty->flip.char_buf_ptr = rx & 0xff;
		*tty->flip.flag_buf_ptr = TTY_NORMAL;
		port->icount.rx++;

		if (rx & (UART_PRERR | UART_OVRRUN | UART_FRAMERR)) {
			/*
			 * For statistics only
			 */
			if (rx & UART_OVRRUN) {
				port->icount.overrun++;
			} else if (rx & UART_PRERR) {
				port->icount.parity++;
			} else if (rx & UART_FRAMERR) {
				port->icount.frame++;
			}

			/*
			 * Mask off conditions which should be ingored.
			 */
			rx &= port->read_status_mask;

			if (rx & UART_PRERR) {
				*tty->flip.flag_buf_ptr++ = TTY_PARITY;
			} else if (rx & UART_FRAMERR) {
				*tty->flip.flag_buf_ptr++ = TTY_FRAME;
			}
		}
		if ((rx & port->ignore_status_mask) == 0) {
			tty->flip.flag_buf_ptr++;
			tty->flip.char_buf_ptr++;
			tty->flip.count++;
		}
		if ((rx & UART_OVRRUN) && tty->flip.count < TTY_FLIPBUF_SIZE) {
			/*
			 * Overrun is special, since it's reported
			 * immediately, and doesn't affect the current
			 * character.
			 */
			*tty->flip.flag_buf_ptr = TTY_OVERRUN;
			tty->flip.flag_buf_ptr++;
			tty->flip.char_buf_ptr++;
			tty->flip.count++;
		}
	}

	tty->flip.tqueue.routine((void *) tty);

	return 0;
}

static void
mx2ads_tx_chars(struct uart_info *info)
{
	struct uart_port *port = info->port;
	int len;

	if (port->x_char) {
		uart_putc(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (info->xmit.head == info->xmit.tail
	    || info->tty->stopped || info->tty->hw_stopped) {
		mx2ads_stop_tx(info->port, 0);
		return;
	}

	/* TRDY is automatically cleared if FIFO level goes beyond threshold */
	len = CIRC_CNT(info->xmit.head, info->xmit.tail, UART_XMIT_SIZE);
	len = (len > TX_MAX) ? TX_MAX : len;
	while (len-- > 0) {
		uart_putc(port, info->xmit.buf[info->xmit.tail]);
		info->xmit.tail = (info->xmit.tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (info->xmit.head == info->xmit.tail)
			break;
	}

	if (CIRC_CNT(info->xmit.head,
		     info->xmit.tail, UART_XMIT_SIZE) < WAKEUP_CHARS)
		uart_event(info, EVT_WRITE_WAKEUP);

	if (info->xmit.head == info->xmit.tail)
		mx2ads_stop_tx(info->port, 0);
}

/*
   Only has 1 mode. Interrupt cause are TRDY or TXEMPTY.
 */
static void
mx2ads_tx_int(struct uart_info *info)
{
	mx2ads_tx_chars(info);
}

/*
 */
static void
mx2ads_rx_int(struct uart_info *info)
{
	struct uart_port *port = info->port;
	int id;

	ENTRY();
	id = get_uart_id(port);

	/* should be RRDY interrupt */
	mx2ads_rx_chars(info);

	/* add timer to set DREN */

	return;
}

static void
mx2ads_int(int irq, void *dev_id, struct pt_regs *regs)
{
	struct uart_port *port = dev_id;
	unsigned int usr2;
	int id;

	id = get_uart_id(port);

	/* clear AWAKE interrupt, may be needed while going to sleep */
	uart_out(port, UART_SR1, UART_AWAKE);

	if (!irq_active[id]) return;


	usr2 = uart_in(port, UART_SR2);

	if (usr2 & UART_RDR) {
		mx2ads_rx_int(local_info[id]);
	} else if (usr2 & UART_TXDC) {
		mx2ads_tx_int(local_info[id]);
	}
}


/*
 * Return TIOCSER_TEMT when transmitter is empty.
 */
static u_int
mx2ads_tx_empty(struct uart_port *port)
{
	return (uart_in(port, UART_SR2) & UART_TXFE) ? TIOCSER_TEMT : 0;
}

static u_int
mx2ads_get_mctrl(struct uart_port *port)
{
	unsigned int status = 0, ucr2, ucr3;
	ucr2 = uart_in(port, UART_CR2);
	ucr3 = uart_in(port, UART_CR3);

	ENTRY();		/* this function is called periodically */

	status |= (ucr2 & UART_CTSC) ? TIOCM_RTS : 0;
	status |= (ucr2 & UART_RTSS) ? 0 : TIOCM_CTS;
	status |= (ucr3 & UART_DCD) ? TIOCM_CAR : 0;	/* only in UART2 */
	status |= (ucr3 & UART_DTREN) ? TIOCM_DTR : 0;	/* only in UART2 */

	return status;
}

static void
mx2ads_set_mctrl(struct uart_port *port, u_int mctrl)
{
}

/*
 * Interrupts always disabled.
 */
static void
mx2ads_break_ctl(struct uart_port *port, int break_state)
{
	unsigned int ucr1;

	ENTRY();

	ucr1 = uart_in(port, UART_CR1);

	if (break_state == -1)
		ucr1 |= UART_SNDBRK;
	else
		ucr1 &= ~UART_SNDBRK;

	uart_out(port, UART_CR1, ucr1);

	EXIT();

}


static int
mx2ads_startup(struct uart_port *port, struct uart_info *info)
{
	unsigned long flags;
	int id;
	ENTRY();

	id = get_uart_id(port);

	local_info[id] = info; /*save for referencing in the local int handler*/

	/* initialise the old status of the modem signals */
	port->ops->set_mctrl(port, info->mctrl);
	local_irq_save(flags);

	/* disable asynchronous wake up interrupt */
	uart_out(port, UART_CR3, uart_in(port, UART_CR3) & ~UART_AWAKEN);

	uart_out(port, UART_CR1,
		 uart_in(port, UART_CR1) | UART_UARTEN | UART_RRDYEN);
	uart_out(port, UART_CR2,
		 uart_in(port, UART_CR2) | UART_RXEN | UART_TXEN);
	uart_out(port, UART_CR3, uart_in(port, UART_CR3));
	uart_out(port, UART_CR4, 0x8000 | UART_DREN);

	irq_active[id] = 1;

	local_irq_restore(flags);

	EXIT();

	return 0;
}

static void
mx2ads_shutdown(struct uart_port *port, struct uart_info *info)
{
	int id;
	unsigned long flags;

	ENTRY();

	id = get_uart_id(port);

	local_irq_save(flags);

	/* Put to reset value */
	uart_out(port, UART_CR1, 0x0);
	uart_out(port, UART_CR2, UART_IRTS | UART_WS);
	uart_out(port, UART_CR3, UART_RXDMUXSEL);
	uart_out(port, UART_CR4, 0x8000);

	uart_out(port, UART_SR1, UART_AWAKE);	/* clear first */

	/* enable asynchronous wake up interrupt */
	uart_out(port, UART_CR3, uart_in(port, UART_CR3) | UART_AWAKEN);

	irq_active[id] = 0;

	local_irq_restore(flags);

	EXIT();
}

static void
mx2ads_change_speed(struct uart_port *port, u_int cflag, u_int iflag,
		    u_int quot)
{
	unsigned int ucr1, ucr2, ucr3, ucr4, bir, bmr, fcr;
	unsigned long flags;
	unsigned long freq_10khz;

	ENTRY();

	/* disable tx and rx */
	uart_out(port, UART_CR1, uart_in(port, UART_CR1) & ~UART_UARTEN);
	uart_out(port, UART_CR2,
		 uart_in(port, UART_CR2) & ~(UART_TXEN | UART_RXEN));

#if defined(CONFIG_MX2TO1)
	freq_10khz = mx_module_get_clk(PERCLK) / 10000;
#else /*TO2*/
	freq_10khz = mx_module_get_clk(PERCLK1) / 10000;
#endif

	/* for uart options */
	ucr1 = uart_in(port, UART_CR1);
	ucr2 = uart_in(port, UART_CR2);

	/* for baud rate */
	ucr3 = uart_in(port, UART_CR3);
	ucr4 = uart_in(port, UART_CR4);
	bir = uart_in(port, UART_BIR);
	bmr = uart_in(port, UART_BMR);

	/* byte size */
	ucr2 &= ~(UART_WS);	/* clear bits */
	if ((cflag & CSIZE) == CS7)
		ucr2 |= UART_7BIT;
	else if ((cflag & CSIZE) == CS8)
		ucr2 |= UART_8BIT;
	else
		ucr2 |= UART_8BIT;

	/* parity */
	ucr2 &= ~(UART_PREN | UART_PROE);	/* clear bits */
	if (cflag & PARENB) {
		ucr2 |= UART_PREN;	/* parity enable */
		if (cflag & PARODD)
			ucr2 |= UART_PARITY_ODD;	/* odd */
		else
			ucr2 |= UART_PARITY_EVEN;	/* even */
	} else {
		ucr2 |= UART_NON_PARITY;	/* non parity */
	}

	/* stop bit */
	ucr2 &= ~(UART_STPB);	/* clear bits */
	if (cflag & CSTOPB) {
		ucr2 |= UART_STOPBIT_2;
	} else {
		ucr2 |= UART_STOPBIT_1;
	}

	/* flow control */
	if (cflag & CRTSCTS) {
		ucr2 &= ~UART_IRTS;	/* stop ingoring RTS  */

		ucr2 |= UART_CTSC;	/* enbale CTS */

	} else {
		ucr2 |= UART_IRTS;

		ucr2 &= ~UART_CTSC;

	}

	ucr3 |= 0x4;

	port->read_status_mask = UART_OVRRUN;
	if (iflag & IGNPAR)
		port->read_status_mask |= UART_FRAMERR | UART_PRERR;

	/*
	 * Characteres to ignore
	 */
	port->ignore_status_mask = 0;
	if (iflag & IGNPAR)
		port->ignore_status_mask |= UART_PRERR | UART_FRAMERR;
	if (iflag & IGNBRK && iflag & IGNPAR)
		port->ignore_status_mask |= UART_OVRRUN;

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_CHARRDY;


	fcr = (TX_LVL << UFCR_TXTL_BIT) | (RX_LVL << UFCR_RXTL_BIT) |
	UFCR_RFDIV_2;

#define MX2UART_RFDIV 2

	/*
	   e.g.
	   115200 baud: 115200*16/16MHz = 0.1152 = 1152/10000
	   BIR = 1152 - 1, BMR = 10000 - 1
	 */
	switch (cflag & CBAUD) {
	case B1200:
		/* 12/10000 = 3/2500 */
		bir = ((1200 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B1800:
		/* 18/10000 = 9/5000 */
		bir = ((1800 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B2400:
		/* 24/10000 = 3/1250 */
		bir = ((2400 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B4800:
		/* 48/10000 = 3/625 */
		bir = ((4800 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B9600:
		/* 96/10000 = 6/625 */
		bir = ((9600 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B19200:
		/* 192/10000 = 12/625 */
		bir = ((19200 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B38400:
		/* 384/10000 = 96/2500 = 24/625 */
		bir = ((38400 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B57600:
		/* 576/10000 = 36/625 */
		bir = ((57600 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B115200:
		/* 1152/10000 = 72/625 */
		bir = ((115200 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B230400:
		/* 2304/10000 = 144/625 */
		bir = ((230400 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B460800:
		/* 4608/10000 = 288/625 */
		bir = ((460800 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B500000:
		/* 5000/10000 = 1/2 */
		bir = ((500000 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B921600:
		/* 9216/10000 = 576/625 */
		bir = ((921600 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	case B1000000:
		/* 10000/10000 = 1/1 */
		bir = ((1000000 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 9999;
		break;
	default:
		printk(KERN_WARNING "%s: Unexpected baudrate = 0x%08x\n",
			 __FUNCTION__, cflag & CBAUD);
		bir = ((115200 * 16 * MX2UART_RFDIV) / freq_10khz) - 1;
		bmr = 10000 - 1;
	}

	/* disable interrupts */
	local_irq_save(flags);

	/* set values */
	uart_out(port, UART_CR1, ucr1 | UART_UARTEN);
	uart_out(port, UART_CR2, ucr2 | UART_TXEN | UART_RXEN);

	uart_out(port, UART_CR3, ucr3);
	uart_out(port, UART_CR4, ucr4);
	uart_out(port, UART_FCR, fcr);

	uart_out(port, UART_BIR, bir);
	uart_out(port, UART_BMR, bmr);

	/* reenable interrupts */
	local_irq_restore(flags);

}

static const char *
mx2ads_type(struct uart_port *port)
{
	ENTRY();
	return port->type == PORT_MX2ADS ? "mx2ads" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void
mx2ads_release_port(struct uart_port *port)
{
	ENTRY();
	release_mem_region(port->mapbase, UART_PORT_SIZE);
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int
mx2ads_request_port(struct uart_port *port)
{
	ENTRY();
	return request_mem_region(port->mapbase, UART_PORT_SIZE,
				  "serial_mx2") != NULL ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void
mx2ads_config_port(struct uart_port *port, int flags)
{
	ENTRY();
	if (flags & UART_CONFIG_TYPE && mx2ads_request_port(port) == 0)
		port->type = PORT_MX2ADS;
	EXIT();
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 */
static int
mx2ads_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;

	ENTRY();
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_MX2ADS)
		ret = -EINVAL;
	if (ser->irq < 0 || ser->irq >= NR_IRQS)
		ret = -EINVAL;
	EXIT();
	return ret;
}

static struct uart_ops mx2ads_pops = {
	.tx_empty = mx2ads_tx_empty,
	.set_mctrl = mx2ads_set_mctrl,
	.get_mctrl = mx2ads_get_mctrl,
	.stop_tx = mx2ads_stop_tx,
	.start_tx = mx2ads_start_tx,
	.stop_rx = mx2ads_stop_rx,
	.enable_ms = mx2ads_enable_ms,
	.break_ctl = mx2ads_break_ctl,
	.startup = mx2ads_startup,
	.shutdown = mx2ads_shutdown,
	.change_speed = mx2ads_change_speed,
	.type = mx2ads_type,

	.release_port = mx2ads_release_port,
	.request_port = mx2ads_request_port,
	.config_port = mx2ads_config_port,
	.verify_port = mx2ads_verify_port,
};

static struct uart_port mx2ads_ports[NR_PORTS] = {
#ifdef CONFIG_SERIAL_MX2ADS_UART1
	{
	 .membase = (unsigned long *) UART1_BASE,
	 .iotype = SERIAL_IO_MEM,
	 .mapbase = UART1_BASE,
	 .irq = INT_UART1,
	 .uartclk = 44 / 2 * 1000 * 1000,
	 .fifosize = 8,
	 .ops = &mx2ads_pops,
	 .flags = ASYNC_BOOT_AUTOCONF,
	 },
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART2
	{
	 .iotype = SERIAL_IO_MEM,
	 .membase = (unsigned long *) UART2_BASE,
	 .mapbase = UART2_BASE,
	 .irq = INT_UART2,
	 .uartclk = 44 / 2 * 1000 * 1000,
	 .fifosize = 32,
	 .ops = &mx2ads_pops,
	 .flags = ASYNC_BOOT_AUTOCONF,
	 },
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART3
	{
	 .iotype = SERIAL_IO_MEM,
	 .membase = (unsigned long *) UART3_BASE,
	 .mapbase = UART3_BASE,
	 .irq = INT_UART3,
	 .uartclk = 44 / 2 * 1000 * 1000,
	 .fifosize = 32,
	 .ops = &mx2ads_pops,
	 .flags = ASYNC_BOOT_AUTOCONF,
	 },
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART4
	{
	 .iotype = SERIAL_IO_MEM,
	 .membase = (unsigned long *) UART4_BASE,
	 .mapbase = UART4_BASE,
	 .irq = INT_UART4,
	 .uartclk = 44 / 2 * 1000 * 1000,
	 .fifosize = 32,
	 .ops = &mx2ads_pops,
	 .flags = ASYNC_BOOT_AUTOCONF,
	 },
#endif
};

#ifdef CONFIG_SERIAL_MX2ADS_CONSOLE

static inline void
wait_for_xmitr(struct uart_port *port)
{
	unsigned int status, tmout = 1000000;

	do {
		status = uart_in(port, UART_SR2);
		if (--tmout == 0)
			break;
	} while (!(status & 0x4000 /*UART_TRDY */ ));
}

static void
mx2ads_console_write(struct console *co, const char *s, u_int count)
{
	struct uart_port *port = mx2ads_ports + co->index;
	unsigned int ucr1, ucr2;
	u_int i;

	/* If port is disabled, do nothing */

	ucr1 = uart_in(port, UART_CR1);
	ucr2 = uart_in(port, UART_CR2);

	if (!((ucr1 & UART_UARTEN) && (ucr2 & (UART_TXEN | UART_RXEN))))
		return;

	/*
	 * Now, do each character
	 */
	for (i = 0; i < count; i++) {
		wait_for_xmitr(port);

		/*
		 *      Send the character out.
		 *      If a LF, also do CR...
		 */
		uart_putc(port, s[i]);
		if (s[i] == '\n') {
			wait_for_xmitr(port);
			uart_putc(port, '\r');
		}
	}

	/*
	 * Wait for the tranceiver to empty
	 */
	wait_for_xmitr(port);
}

static kdev_t
mx2ads_console_device(struct console *co)
{
	ENTRY();

	return MKDEV(SERIAL_MX2ADS_MAJOR, MINOR_START + co->index);
}

/*
 * If the port was already initialised (eg, by a boot loader), try to determine
 * the current setup.
 */
static void __init
mx2ads_console_get_options(struct uart_port *port, int *baud, int *parity,
			   int *bits)
{
	unsigned int ucr1, ucr2, ucr3, ucr4, bir, bmr;

	ENTRY();

	ucr1 = uart_in(port, UART_CR1);
	ucr2 = uart_in(port, UART_CR2);
	if ((ucr1 & UART_UARTEN) && (ucr2 & (UART_TXEN | UART_RXEN))) {
		/* this port is enabled */

		/* check parity */
		if (ucr2 & UART_PREN) {
			if ((ucr2 & UART_PROE) == UART_PARITY_ODD) {
				*parity = 'o';	/* odd */
			} else {
				*parity = 'e';	/* even */
			}
		} else {
			*parity = 'n';	/* non-parity */
		}

		/* check word size */
		if ((ucr2 & UART_WS) == UART_8BIT) {
			*bits = 8;
		} else {
			*bits = 7;
		}

		/* this code suppose UART_REF_CLOCK to be 16 Mhz */
		ucr3 = uart_in(port, UART_CR3);
		ucr4 = uart_in(port, UART_CR4);
		bir = uart_in(port, UART_BIR);
		bmr = uart_in(port, UART_BMR);
		*baud = CONFIG_SERIAL_MX2ADS_DEFAULT_BAUDRATE;
	}
}

static int __init
mx2ads_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = CONFIG_SERIAL_MX2ADS_DEFAULT_BAUDRATE;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	ENTRY();
	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	port = uart_get_console(mx2ads_ports, NR_PORTS, co);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		mx2ads_console_get_options(port, &baud, &parity, &bits);

	EXIT();

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console mx2ads_console = {
	.name = "ttyMX",
	.write = mx2ads_console_write,
	.device = mx2ads_console_device,
	.setup = mx2ads_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
};

void __init
mx2ads_console_init(void)
{
	ENTRY();
	register_console(&mx2ads_console);
	EXIT();
}

#define MX2ADS_CONSOLE	&mx2ads_console
#else
#define MX2ADS_CONSOLE	NULL
#endif				/* CONFIG_SERIAL_MX2ADS_CONSOLE */

static struct uart_driver mx2ads_reg = {
	.owner = THIS_MODULE,
	.normal_major = SERIAL_MX2ADS_MAJOR,
#ifdef CONFIG_DEVFS_FS
	.normal_name = "ttyMX%d",
	.callout_name = "cuamx%d",
#else
	.normal_name = "ttyMX",
	.callout_name = "cuamx",
#endif
	.normal_driver = &normal,
	.callout_major = CALLOUT_MX2ADS_MAJOR,
	.callout_driver = &callout,
	.table = mx2ads_table,
	.termios = mx2ads_termios,
	.termios_locked = mx2ads_termios_locked,
	.minor = MINOR_START,
	.nr = NR_PORTS,
	.port = mx2ads_ports,
	.cons = MX2ADS_CONSOLE,
};


static void  inline mx2ads_serial_reset(void)
{
	/* do reset */
#ifdef CONFIG_SERIAL_MX2ADS_UART1
	UART_UCR2(0) = 0;
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART2
	UART_UCR2(1) = 0;
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART3
	UART_UCR2(2) = 0;
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART4
	UART_UCR2(3) = 0;
#endif
}

static void __exit mx2ads_serial_exit(void);

static int __init
mx2ads_serial_init(void)
{
	int retval, i;

	ENTRY();

#ifdef CONFIG_SERIAL_MX2ADS_UART1
	if (!(request_region
	     (UART1_BASE, 0x1000, "uart1"))) {
		mx2ads_serial_exit();
		return -1;
	}
	initstate_region1 = 1;

	mx_module_clk_open(IPG_MODULE_UART1);

	retval = mx2_register_gpios(PORT_E,
				    (1 << 12) |
				    (1 << 13) |
				    (1 << 14) |
				    (1 << 15), PRIMARY | NOINTERRUPT);
	if (retval < 0) {
		printk(KERN_ERR DEBUG_NAME "Request GPIO E 12, 13, 14, 15 (UART1) failed\n");
		mx2ads_serial_exit();
		return retval;
	}
	initstate_gpio1 = 1;
#endif

#ifdef CONFIG_SERIAL_MX2ADS_UART2
	if (!(request_region
	     (UART2_BASE, 0x1000, "uart2"))) {
		mx2ads_serial_exit();
		return -1;
	}
	initstate_region2 = 1;

	mx_module_clk_open(IPG_MODULE_UART2);

	retval = mx2_register_gpios(PORT_E,
				    (1 << 3) |
				    (1 << 4) |
				    (1 << 6) |
				    (1 << 7), PRIMARY | NOINTERRUPT);
	if (retval < 0) {
		printk(KERN_ERR DEBUG_NAME "Request GPIO E 3, 4, 6, 7 (UART2) failed\n");
		mx2ads_serial_exit();
		return retval;
	}
	initstate_gpio2 = 1;
#endif

#ifdef CONFIG_SERIAL_MX2ADS_UART3
	if (!(request_region
	     (UART3_BASE, 0x1000, "uart3"))) {
		mx2ads_serial_exit();
		return -1;
	}
	initstate_region3 = 1;

	mx_module_clk_open(IPG_MODULE_UART3);

	retval = mx2_register_gpios(PORT_E,
				    (1 << 8) |
				    (1 << 9) |
				    (1 << 10) |
				    (1 << 11), PRIMARY | NOINTERRUPT);
	if (retval < 0) {
		printk(KERN_ERR DEBUG_NAME "Request GPIO E 8, 9, 10, 11 (UART3) failed\n");
		mx2ads_serial_exit();
		return retval;
	}
	initstate_gpio3 = 1;
#endif

#ifdef CONFIG_SERIAL_MX2ADS_UART4
	if (!(request_region
	     (UART4_BASE, 0x1000, "uart4"))) {
		mx2ads_serial_exit();
		return -1;
	}
	initstate_region4 = 1;

	mx_module_clk_open(IPG_MODULE_UART4);

#ifdef CONFIG_MX2TO1
	retval = mx2_register_gpios(PORT_B,
				    (1 << 28) |
				    (1 << 29) |
				    (1 << 30) |
				    (1 << 31), SECONDARY | NOINTERRUPT);

	if (retval < 0) {
		printk(KERN_ERR DEBUG_NAME "Request GPIO B 28, 29, 30, 31 (UART4) failed\n");
		mx2ads_serial_exit();
		return retval;
	}

	initstate_gpio4 = 1;

#else
	retval = mx2_register_gpios(PORT_B,
				    (1 << 28), SECONDARY | NOINTERRUPT);

	if (retval < 0) {
		printk(KERN_ERR DEBUG_NAME "Request GPIO B 28 (UART4) failed\n");
		mx2ads_serial_exit();
		return retval;
	}

	initstate_gpio4_28 = 1;

	retval = mx2_register_gpios(PORT_B,
				    (1 << 31) |
				    (1 << 29), GPIO | INPUT);

	if (retval < 0) {
		printk(KERN_ERR DEBUG_NAME "Request GPIO B 29, 31 (UART4) failed\n");
		mx2ads_serial_exit();
		return retval;
	}

	initstate_gpio4 = 1;

	retval = mx2_register_gpios(PORT_B,
				    (1 << 30), GPIO | OUTPUT);

	if (retval < 0) {
		printk(KERN_ERR DEBUG_NAME "Request GPIO B 30 (UART4) failed\n");
		mx2ads_serial_exit();
		return retval;
	}

	initstate_gpio4_30 = 1;


	SYS_FMCR &= ~((1 << 25) | (1 << 24));
#endif

#endif

	for (i = 0; i < NR_PORTS; i++) {
		irq_active[i] = 0;
		/*
		 * Allocate the IRQ
		 */
		retval = request_irq(mx2ads_ports[i].irq, mx2ads_int, 0, "uart", &mx2ads_ports[i]);
		if (retval < 0) {
			printk(KERN_ERR DEBUG_NAME "Request IRQ for %s%d failed\n",
			       "uart", i);
			mx2ads_serial_exit();
			return retval;
		}
		initstate_uart_irq[i] = 1;

	}

	retval = uart_register_driver(&mx2ads_reg);
	if (retval < 0) {
		printk(KERN_ERR DEBUG_NAME "Register UART driver failed\n");
		mx2ads_serial_exit();
		return retval;
	}
	initstate_uart = 1;

	EXIT();

	return 0;
}

static void __init
mx2ads_serial_exit(void)
{
	int i;
	ENTRY();

	if (initstate_uart) {

		mx2ads_serial_reset();
		uart_unregister_driver(&mx2ads_reg);
	}

	for (i = 0; i < NR_PORTS; i++) {
		if (initstate_uart_irq[i]) {
			free_irq(mx2ads_ports[i].irq, &mx2ads_ports[i]);
		}
	}

#ifdef CONFIG_SERIAL_MX2ADS_UART1
	if (initstate_gpio1)
		mx2_unregister_gpios(PORT_E,
				     (1 << 12) |
				     (1 << 13) |
				     (1 << 14) |
				     (1 << 15));
	if (initstate_region1) {
		mx_module_clk_close(IPG_MODULE_UART1);
		release_region(UART1_BASE, 0x1000);
	}
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART2
	if (initstate_gpio2)
		mx2_unregister_gpios(PORT_E,
				     (1 << 3) |
				     (1 << 4) |
				     (1 << 6) |
				     (1 << 7));
	if (initstate_region2) {
		mx_module_clk_close(IPG_MODULE_UART2);
		release_region(UART2_BASE, 0x1000);
	}
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART3
	if (initstate_gpio3)
		mx2_unregister_gpios(PORT_E,
				     (1 << 8) |
				     (1 << 9) |
				     (1 << 10) |
				     (1 << 11));
	if (initstate_region3) {
		mx_module_clk_close(IPG_MODULE_UART3);
		release_region(UART3_BASE, 0x1000);
	}
#endif
#ifdef CONFIG_SERIAL_MX2ADS_UART4
#ifdef CONFIG_MX2TO1
	if (initstate_gpio4)
		mx2_unregister_gpios(PORT_B,
				     (1 << 28) |
				     (1 << 29) |
				     (1 << 30) |
				     (1 << 31));
#else
	if (initstate_gpio4_30)
		mx2_unregister_gpios(PORT_B,
				     (1 << 30));

	if (initstate_gpio4)
		mx2_unregister_gpios(PORT_B,
				     (1 << 29) |
				     (1 << 31));
	if (initstate_gpio4_28)
		mx2_unregister_gpios(PORT_B,
				     (1 << 28));
#endif
	if (initstate_region4) {
		mx_module_clk_close(IPG_MODULE_UART4);
		release_region(UART4_BASE, 0x1000);
	}
#endif


}

module_init(mx2ads_serial_init);
module_exit(mx2ads_serial_exit);

EXPORT_NO_SYMBOLS;
MODULE_DESCRIPTION("DragonBall generic serial port driver");
MODULE_LICENSE("GPL");
