/*
 *  linux/drivers/char/serial_h3800.c
 *
 *  16550A driver for H3800 ASIC serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o, 
 *  and Russell King's serial_8250.c driver.  
 *
 *  Okay, it's basically a straight copy of Russell's 8250 driver with
 *  hooks so I can turn on and off subsystems.
 *  
 *  Copyright (C) 2002 Andrew Christian
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *
 * A note about mapbase / membase
 *
 *  mapbase is the physical address of the IO port.  Currently, we don't
 *  support this very well, and it may well be dropped from this driver
 *  in future.  HOWEVER, the uart_setup_port function in the serial_core
 *  requires either mapbase or iobase to be non-null
 *
 *  membase is an 'ioremapped' cookie.  This is compatible with the old
 *  serial.c driver, and is currently the preferred form.
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
#include <linux/serial_reg.h>
#include <linux/serialP.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/kmod.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>
#include <asm/hardware.h>
#include <asm/arch/serial_h3800.h>
#include <asm/arch/h3600_asic.h>

#define NR_PORTS                2      // The ASIC has two UARTs on it
#define SERIAL_SA1100_MAJOR	204
#define CALLOUT_SA1100_MAJOR	205
#define MINOR_START		60     // We made this up. Probably a bad idea 

#define UART_PORT_SIZE        0x24

static struct tty_driver normal, callout;
static struct tty_struct *serial_h3800_table[NR_PORTS];
static struct termios *serial_h3800_termios[NR_PORTS], *serial_h3800_termios_locked[NR_PORTS];
static struct serial_h3800_fns serial_h3800_fns;

#define port_acr	unused[0]	/* 8bit */
#define port_ier	unused[1]	/* 8bit */
#define port_rev	unused[2]	/* 8bit */
#define port_lcr	unused[3]	/* 8bit */

static __inline__ unsigned int serial_in(struct uart_port *port, int offset)
{
	unsigned int value;
	value = readb((unsigned long)port->membase + (offset << port->regshift));

	if (0)
		printk(__FUNCTION__ ": offset=%d (%lx) -> %x\n", offset, 
		       (unsigned long)port->membase + (offset << port->regshift),
		       value);

	return value;
}

static __inline__ void serial_out(struct uart_port *port, int offset, int value)
{
	writeb(value, (unsigned long)port->membase + (offset << port->regshift));

	if (0)
		printk(__FUNCTION__ ": offset=%d (%lx) <- %x\n", offset, 
		       (unsigned long)port->membase + (offset << port->regshift),
		       value);
}


/*
 * Evil...get rid of this....
 */
static void autoconfig(struct uart_port *port)
{
	unsigned char scratch;
	unsigned char save_lcr, save_mcr;
	unsigned long flags;

	if (0) printk(__FUNCTION__ "\n");
	if (!port->iobase && !port->membase)
		return;

	save_flags(flags); cli();

	save_mcr = serial_in(port, UART_MCR);
	save_lcr = serial_in(port, UART_LCR);

	if (0) printk(__FUNCTION__ " saved mcr=%x lcr=%x\n", save_mcr, save_lcr);
	serial_out(port, UART_LCR, 0xBF); // set up for StarTech test
	serial_out(port, UART_EFR, 0);	// EFR is the same as FCR 
	serial_out(port, UART_LCR, 0);
	serial_out(port, UART_FCR, UART_FCR_ENABLE_FIFO);
	scratch = serial_in(port, UART_IIR) >> 6;
	if ( scratch != 3 )
		printk("serial: Ugh!  Can't identify UART\n");

	serial_out(port, UART_LCR, save_lcr);
	port->fifosize = 16;

	// Reset the UART.
	serial_out(port, UART_MCR, save_mcr);
	serial_out(port, UART_FCR, (UART_FCR_ENABLE_FIFO |
				     UART_FCR_CLEAR_RCVR |
				     UART_FCR_CLEAR_XMIT));
	serial_out(port, UART_FCR, 0);
	(void)serial_in(port, UART_RX);
	serial_out(port, UART_IER, 0);
	
	restore_flags(flags);
}

static void serial_h3800_stop_tx(struct uart_port *port, u_int from_tty)
{
	if (port->port_ier & UART_IER_THRI) {
		port->port_ier &= ~UART_IER_THRI;
		serial_out(port, UART_IER, port->port_ier);
	}
}

static void serial_h3800_start_tx(struct uart_port *port, u_int nonempty, u_int from_tty)
{
	if (nonempty && !(port->port_ier & UART_IER_THRI)) {
		port->port_ier |= UART_IER_THRI;
		serial_out(port, UART_IER, port->port_ier);
	}
}

static void serial_h3800_stop_rx(struct uart_port *port)
{
	port->port_ier &= ~UART_IER_RLSI;
	port->read_status_mask &= ~UART_LSR_DR;
	serial_out(port, UART_IER, port->port_ier);
}

static void serial_h3800_enable_ms(struct uart_port *port)
{
	port->port_ier |= UART_IER_MSI;
	serial_out(port, UART_IER, port->port_ier);
}

static __inline__ void serial_h3800_rx_chars(struct uart_info *info, int *status, struct pt_regs *regs)
{
	struct tty_struct *tty = info->tty;
	struct uart_port *port = info->port;
	unsigned char ch;
	int max_count = 256;

	do {
		if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
			tty->flip.tqueue.routine((void *)tty);
			if (tty->flip.count >= TTY_FLIPBUF_SIZE)
				return; // if TTY_DONT_FLIP is set
		}
		ch = serial_in(port, UART_RX);
		*tty->flip.char_buf_ptr = ch;
		*tty->flip.flag_buf_ptr = TTY_NORMAL;
		port->icount.rx++;

		if (*status & (UART_LSR_BI | UART_LSR_PE |
			       UART_LSR_FE | UART_LSR_OE)) {
			/*
			 * For statistics only
			 */
			if (*status & UART_LSR_BI) {
				*status &= ~(UART_LSR_FE | UART_LSR_PE);
				port->icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				uart_handle_break(info, &serial_h3800_console);
			} else if (*status & UART_LSR_PE)
				port->icount.parity++;
			else if (*status & UART_LSR_FE)
				port->icount.frame++;
			if (*status & UART_LSR_OE)
				port->icount.overrun++;

			/*
			 * Mask off conditions which should be ingored.
			 */
			*status &= port->read_status_mask;

			if (*status & UART_LSR_BI) {
#ifdef SERIAL_DEBUG_INTR
				printk("handling break....");
#endif
				*tty->flip.flag_buf_ptr = TTY_BREAK;
			} else if (*status & UART_LSR_PE)
				*tty->flip.flag_buf_ptr = TTY_PARITY;
			else if (*status & UART_LSR_FE)
				*tty->flip.flag_buf_ptr = TTY_FRAME;
		}
		if (uart_handle_sysrq_char(info, ch, regs))
			goto ignore_char;
		if ((*status & port->ignore_status_mask) == 0) {
			tty->flip.flag_buf_ptr++;
			tty->flip.char_buf_ptr++;
			tty->flip.count++;
		}
		if ((*status & UART_LSR_OE) &&
		    tty->flip.count < TTY_FLIPBUF_SIZE) {
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
	ignore_char:
		*status = serial_in(port, UART_LSR);
	} while ((*status & UART_LSR_DR) && (max_count-- > 0));
	tty_flip_buffer_push(tty);
}

static __inline__ void serial_h3800_tx_chars(struct uart_info *info, int *intr_done)
{
	struct uart_port *port = info->port;
	int count;

	if (port->x_char) {
		serial_out(port, UART_TX, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		if (intr_done)
			*intr_done = 0;
		return;
	}
	if (info->xmit.head == info->xmit.tail
	    || info->tty->stopped
	    || info->tty->hw_stopped) {
		serial_h3800_stop_tx(port, 0);
		return;
	}

	count = port->fifosize;
	do {
		serial_out(port, UART_TX, info->xmit.buf[info->xmit.tail]);
		info->xmit.tail = (info->xmit.tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (info->xmit.head == info->xmit.tail)
			break;
	} while (--count > 0);

	if (CIRC_CNT(info->xmit.head, info->xmit.tail, UART_XMIT_SIZE) <
			WAKEUP_CHARS)
		uart_event(info, EVT_WRITE_WAKEUP);

#ifdef SERIAL_DEBUG_INTR
	printk("THRE...");
#endif
	if (intr_done)
		*intr_done = 0;

	if (info->xmit.head == info->xmit.tail)
		serial_h3800_stop_tx(info->port, 0);
}

static __inline__ void check_modem_status(struct uart_info *info)
{
	struct uart_port *port = info->port;
	int status;

	status = serial_in(port, UART_MSR);

	if (status & UART_MSR_ANY_DELTA) {
		if (status & UART_MSR_TERI)
			port->icount.rng++;
		if (status & UART_MSR_DDSR)
			port->icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change(info, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change(info, status & UART_MSR_CTS);

		wake_up_interruptible(&info->delta_msr_wait);
	}
}

#define RS_ISR_PASS_LIMIT 256

static void serial_h3800_int(int irq, void *dev_id, struct pt_regs *regs)
{
	struct uart_info *info = dev_id;
	int pass_counter = 0;

	if (0) printk("serial_h3800_int(%d) info=%p port=%p\n", irq, info, info->port);

	do {
		int status = serial_in(info->port, UART_LSR);
		if (status & UART_LSR_DR)
			serial_h3800_rx_chars(info, &status, regs);
		check_modem_status(info);
		if (status & UART_LSR_THRE)
			serial_h3800_tx_chars(info, 0);

		if (pass_counter++ > RS_ISR_PASS_LIMIT) {
			printk(__FUNCTION__ " overrun pass limit\n");
			break;
		}
	} while (!(serial_in(info->port, UART_IIR) & UART_IIR_NO_INT));
}

static u_int serial_h3800_tx_empty(struct uart_port *port)
{
	return serial_in(port, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
}

static u_int serial_h3800_get_mctrl(struct uart_port *port)
{
	unsigned long flags;
	unsigned char status;
	unsigned int ret;

	save_flags(flags); cli();
	status = serial_in(port, UART_MSR);
	restore_flags(flags);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial_h3800_set_mctrl(struct uart_port *port, u_int mctrl)
{
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	serial_out(port, UART_MCR, mcr);
}

static void serial_h3800_break_ctl(struct uart_port *port, int break_state)
{
	if (break_state == -1)
		port->port_lcr |= UART_LCR_SBC;
	else
		port->port_lcr &= ~UART_LCR_SBC;
	serial_out(port, UART_LCR, port->port_lcr);
}

static int serial_h3800_startup(struct uart_port *port, struct uart_info *info)
{
	int retval;

	if (0) printk(__FUNCTION__ " %p\n", port);

	/* Call any user-specified open routines */
	if ( serial_h3800_fns.open ) {
		retval = serial_h3800_fns.open(port, info);
		if ( retval )
			return retval;
	}

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reeanbled in change_speed())
	 */
	serial_out(port, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_out(port, UART_FCR, UART_FCR_ENABLE_FIFO |
		   UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_out(port, UART_FCR, 0);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_in(port, UART_LSR);
	(void) serial_in(port, UART_RX);
	(void) serial_in(port, UART_IIR);
	(void) serial_in(port, UART_MSR);

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(port->irq, serial_h3800_int, 0, "serial_h3800", info );
	if ( retval ) {
		if ( serial_h3800_fns.close )
			serial_h3800_fns.close(port, info);
		return retval;
	}

	/*
	 * Now, initialize the UART
	 */
	serial_out(port, UART_LCR, UART_LCR_WLEN8);

	/*
	 * Most PC uarts need OUT2 raised to enable interrupts.
	 */
	if (port->irq != 0)
		info->mctrl |= TIOCM_OUT2;
	serial_h3800_set_mctrl(port, info->mctrl);  // TODO: do we need this?

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via change_speed(), which will be occuring imminently
	 * anyway, so we don't enable them here.
	 */
	port->port_ier = UART_IER_RLSI | UART_IER_RDI;
	serial_out(port, UART_IER, port->port_ier);

	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void) serial_in(port, UART_LSR);
	(void) serial_in(port, UART_RX);
	(void) serial_in(port, UART_IIR);
	(void) serial_in(port, UART_MSR);

	return 0;
}

static void serial_h3800_shutdown(struct uart_port *port, struct uart_info *info)
{
	if (0) printk(__FUNCTION__ " %p\n", port);

	/*
	 * First, disable all intrs from the port.
	 */
	port->port_ier = 0;
	serial_out(port, UART_IER, 0);

	synchronize_irq();  /* TODO: What is this? */

	/*
	 * Free the IRQ
	 */
	free_irq(port->irq, info);
	info->mctrl &= ~TIOCM_OUT2;

	serial_h3800_set_mctrl(port, info->mctrl);  // TODO: do we need this?

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(port, UART_LCR, serial_in(port, UART_LCR) & ~UART_LCR_SBC);
	serial_out(port, UART_FCR, UART_FCR_ENABLE_FIFO |
				    UART_FCR_CLEAR_RCVR |
				    UART_FCR_CLEAR_XMIT);
	serial_out(port, UART_FCR, 0);

	/*
	 * Read data port to reset things
	 */
	(void) serial_in(port, UART_RX);

	if ( serial_h3800_fns.close )
		serial_h3800_fns.close(port, info);
}

static void serial_h3800_change_speed(struct uart_port *port, u_int cflag, u_int iflag, u_int quot)
{
	unsigned char cval, fcr = 0;
	unsigned long flags;

	if (0) printk(__FUNCTION__ " cflag=%08x iflag=%08x\n", cflag, iflag);

	switch (cflag & CSIZE) {
	case CS5:	cval = 0x00;	break;
	case CS6:	cval = 0x01;	break;
	case CS7:	cval = 0x02;	break;
	default:
	case CS8:	cval = 0x03;	break;
	}

	if (cflag & CSTOPB)
		cval |= 0x04;
	if (cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	if ((port->uartclk / quot) < (2400 * 16))
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
	else
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_8;

	port->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (iflag & IGNPAR)
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	port->ignore_status_mask = 0;
	if (iflag & IGNPAR)
		port->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (iflag & IGNBRK) {
		port->ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (iflag & IGNPAR)
			port->ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	port->port_ier &= ~UART_IER_MSI;
	if (port->flags & ASYNC_HARDPPS_CD || cflag & CRTSCTS ||
	    !(cflag & CLOCAL))
		port->port_ier |= UART_IER_MSI;

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	save_flags(flags); cli();
	serial_out(port, UART_IER, port->port_ier);

	serial_out(port, UART_LCR, cval | UART_LCR_DLAB);	/* set DLAB */
	serial_out(port, UART_DLL, quot & 0xff);	/* LS of divisor */
	serial_out(port, UART_DLM, quot >> 8);		/* MS of divisor */
	serial_out(port, UART_LCR, cval);		/* reset DLAB */
	port->port_lcr = cval;				/* Save LCR */
	if (fcr & UART_FCR_ENABLE_FIFO) {
		/* emulated UARTs (Lucent Venus 167x) need two steps */
		serial_out(port, UART_FCR, UART_FCR_ENABLE_FIFO);
	}
	serial_out(port, UART_FCR, fcr);	/* set fcr */
	restore_flags(flags);
}

static void serial_h3800_pm(struct uart_port *port, u_int state, u_int oldstate)
{
	if (0) printk(__FUNCTION__ " %p state=%d oldstate=%d\n", port, state, oldstate);
	if ( serial_h3800_fns.pm )
		serial_h3800_fns.pm( port, state, oldstate );
}

static const char * serial_h3800_type(struct uart_port *port)
{
	printk(__FUNCTION__ " %p\n", port);
	return port->type == PORT_H3800_ASIC ? "H3800_ASIC" : NULL;
}

static void serial_h3800_release_port(struct uart_port *port)
{
	printk(__FUNCTION__ " %p  mapbase=%lx\n", port, port->mapbase);
	release_mem_region(port->mapbase, UART_PORT_SIZE);
}

static int serial_h3800_request_port(struct uart_port *port)
{
	printk(__FUNCTION__ " %p  mapbase=%lx\n", port, port->mapbase);
	return request_mem_region(port->mapbase, UART_PORT_SIZE,
			"serial_h3800") != NULL ? 0 : -EBUSY;
}

static void serial_h3800_config_port(struct uart_port *port, int flags)
{
	printk(__FUNCTION__ " %p  flags=%x\n", port, flags);
	if (flags & UART_CONFIG_TYPE && serial_h3800_request_port(port) == 0)
		port->type = PORT_H3800_ASIC;

// TODO: Is this needed?
	autoconfig(port);
}

static int serial_h3800_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	printk(__FUNCTION__ " %p\n", port);
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_SA1100)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_MEM)
		ret = -EINVAL;
	if (port->uartclk / 16 != ser->baud_base)   /* TODO: If we allow other baud rates, fix this */
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops serial_h3800_pops = {
	tx_empty:	serial_h3800_tx_empty,
	set_mctrl:	serial_h3800_set_mctrl,
	get_mctrl:	serial_h3800_get_mctrl,
	stop_tx:	serial_h3800_stop_tx,
	start_tx:	serial_h3800_start_tx,
	stop_rx:	serial_h3800_stop_rx,
	enable_ms:	serial_h3800_enable_ms,
	break_ctl:	serial_h3800_break_ctl,
	startup:	serial_h3800_startup,
	shutdown:	serial_h3800_shutdown,
	change_speed:	serial_h3800_change_speed,
	pm:		serial_h3800_pm,
	type:		serial_h3800_type,
	release_port:	serial_h3800_release_port,
	request_port:	serial_h3800_request_port,
	config_port:	serial_h3800_config_port,
	verify_port:	serial_h3800_verify_port,
};

/* 
 * We create a static structure to hold our two possible
 * serial ports.  We initialize these to reasonable values,
 * but leave the port type as PORT_UNKNOWN so it won't be
 * be installed.  That will be handled later by uart_register_port
 */

static struct uart_port h3800_asic_ports[NR_PORTS];

static void __init serial_h3800_init_ports(void)
{
	static int first = 1;
	int i;

	if (!first)
		return;
	first = 0;

	for (i = 0; i < NR_PORTS; i++)
		h3800_asic_ports[i].ops = &serial_h3800_pops;

	h3800_asic_ports[0].mapbase = asic_v2p(H3800_ASIC2_UART_0_Base);
	h3800_asic_ports[1].mapbase = asic_v2p(H3800_ASIC2_UART_1_Base);
}

static struct uart_driver serial_h3800_reg = {
	owner:			THIS_MODULE,
#ifdef CONFIG_DEVFS_FS
	normal_name:		"ttySB%d",
	callout_name:		"cuab/%d",
#else
	normal_name:		"ttySB",
	callout_name:		"cuab",
#endif
	normal_major:		SERIAL_SA1100_MAJOR,
	callout_major:		CALLOUT_SA1100_MAJOR,
	normal_driver:		&normal,
	callout_driver:		&callout,
	table:			serial_h3800_table,
	termios:		serial_h3800_termios,
	termios_locked:		serial_h3800_termios_locked,
	minor:			MINOR_START,
	nr:			NR_PORTS,
	port:			h3800_asic_ports,
	cons:			NULL,
};

int register_serial_h3800(int index)
{
	struct uart_port port;

	printk(__FUNCTION__ " %d\n", index);

	if (index < 0 || index >= NR_PORTS) {
		printk(KERN_ERR __FUNCTION__ ": bad index number %d\n", index);
		return -EINVAL;
	}

	port.iobase   = 0;
	port.uartclk  = 3686400;   // Technically, we could also run off the 24 MHz clock
	port.fifosize = 16;
	port.regshift = 2;
	port.iotype   = SERIAL_IO_MEM;
	port.flags    = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST;

	switch (index) {
	case 0:
		port.membase = H3800_ASIC2_UART_0_Base;
		port.irq     = IRQ_H3800_UART_0;
		break;

	case 1:
		port.membase = H3800_ASIC2_UART_1_Base;
		port.irq     = IRQ_H3800_UART_1;
		break;
	}

	printk(" membase=%p irq=%d\n", port.membase, port.irq);
	return uart_register_port(&serial_h3800_reg, &port);
}

void unregister_serial_h3800(int line)
{
	printk(__FUNCTION__ " %d\n", line);
	uart_unregister_port(&serial_h3800_reg, line);
}

void register_serial_h3800_fns(struct serial_h3800_fns *fns)
{
	if ( fns )
		serial_h3800_fns = *fns;
	else
		memset(&serial_h3800_fns,0,sizeof(serial_h3800_fns));
}


static int __init serial_h3800_init(void)
{
	serial_h3800_init_ports();   // This may be a waste of time...
	return uart_register_driver(&serial_h3800_reg);
}

static void __exit serial_h3800_exit(void)
{
	uart_unregister_driver(&serial_h3800_reg);
}

module_init(serial_h3800_init);
module_exit(serial_h3800_exit);

EXPORT_SYMBOL(register_serial_h3800);
EXPORT_SYMBOL(register_serial_h3800_fns);
EXPORT_SYMBOL(unregister_serial_h3800);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrew Christian");
MODULE_DESCRIPTION("16550A serial driver for H3800 ASIC");

