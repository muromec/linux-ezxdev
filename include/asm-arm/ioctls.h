/*
 * Copyright (C) 2005 Motorola Inc.
 *
 *  modified by a14194, for Motorola Ezx Platform 
 */

#ifndef __ASM_ARM_IOCTLS_H
#define __ASM_ARM_IOCTLS_H

#include <asm/ioctl.h>

/* 0x54 is just a magic number to make these relatively unique ('T') */

#define TCGETS		0x5401
#define TCSETS		0x5402
#define TCSETSW		0x5403
#define TCSETSF		0x5404
#define TCGETA		0x5405
#define TCSETA		0x5406
#define TCSETAW		0x5407
#define TCSETAF		0x5408
#define TCSBRK		0x5409
#define TCXONC		0x540A
#define TCFLSH		0x540B
#define TIOCEXCL	0x540C
#define TIOCNXCL	0x540D
#define TIOCSCTTY	0x540E
#define TIOCGPGRP	0x540F
#define TIOCSPGRP	0x5410
#define TIOCOUTQ	0x5411
#define TIOCSTI		0x5412
#define TIOCGWINSZ	0x5413
#define TIOCSWINSZ	0x5414
#define TIOCMGET	0x5415
#define TIOCMBIS	0x5416
#define TIOCMBIC	0x5417
#define TIOCMSET	0x5418
#define TIOCGSOFTCAR	0x5419
#define TIOCSSOFTCAR	0x541A
#define FIONREAD	0x541B
#define TIOCINQ		FIONREAD
#define TIOCLINUX	0x541C
#define TIOCCONS	0x541D
#define TIOCGSERIAL	0x541E
#define TIOCSSERIAL	0x541F
#define TIOCPKT		0x5420
#define FIONBIO		0x5421
#define TIOCNOTTY	0x5422
#define TIOCSETD	0x5423
#define TIOCGETD	0x5424
#define TCSBRKP		0x5425	/* Needed for POSIX tcsendbreak() */
#define TIOCTTYGSTRUCT	0x5426  /* For debugging only */
#define TIOCSBRK	0x5427  /* BSD compatibility */
#define TIOCCBRK	0x5428  /* BSD compatibility */
#define TIOCGSID	0x5429  /* Return the session ID of FD */
#define TIOCGPTN	_IOR('T',0x30, unsigned int) /* Get Pty Number (of pty-mux device) */
#define TIOCSPTLCK	_IOW('T',0x31, int)  /* Lock/unlock Pty */

#define TIOCWAKEBTS			_IO('T',0x32) 		/* SET BTWAKE Pin */
#define TIOCWAKEBTC			_IO('T',0x33)	  	/* Clear BTWAKE Pin */
#define TIOCRESETBTS			_IO('T',0x34) 		/* SET BTRESET Pin */
#define TIOCRESETBTC			_IO('T',0x35)  		/* Clear BTRESET Pin */
#define TIOC_ENABLE_IFLOW		_IO('T',0x38)  		/* RTS_BTUART is controlled by hardware flow control and asserted */
#define TIOC_DISABLE_IFLOW_RTS_LOW	_IO('T',0x39)  		/* RTS_BTUART is controlled by software flow control and deasserted */
#define TIOC_HOST_WAKE_STATE		_IOR('T',0x40, int)  	/* The state of HOST_WAKE immediately*/
#define TIOC_HOST_WAKE_WATCH		_IOR('T',0x41, int)  	/* The state of HOST_WAKE blocked*/
#define TIOCRESETTF			_IO('T',0x42)	   	/* Reset Transmitter FIFO */
#define TIOCRESETRF			_IO('T',0x43)   	/* Reset Receiver FIFO */
#define TIOC_BT_VOLTAGE_ON		_IO('T', 0x44) 		/*BT Voltage on*/
#define TIOC_BT_VOLTAGE_OFF		_IO('T', 0x45)     	/* BT Voltage off*/
#define TIOCPKTMS			_IO('T', 0x46) 		/* Enable PTY modem signal */
#define TIOCSERSETLSR			_IO('T', 0x47)     	/* Set BT PTY LSR */
#define TIOCSPEEDCTL			_IO('T',0x48)		/* Control the speed with kernel timer */
#define TIOCMOUTQ			_IO('T',0x49)		/* Get exact PTY master OUTQ */

#define BT_UART_CTL_STOP 0
#define BT_UART_CTL_16_K 1
#define BT_UART_CTL_32_K 2
#define BT_UART_CTL_44_1_K 3
#define BT_UART_CTL_48_K 4


#define FIONCLEX	0x5450  /* these numbers need to be adjusted. */
#define FIOCLEX		0x5451
#define FIOASYNC	0x5452
#define TIOCSERCONFIG	0x5453
#define TIOCSERGWILD	0x5454
#define TIOCSERSWILD	0x5455
#define TIOCGLCKTRMIOS	0x5456
#define TIOCSLCKTRMIOS	0x5457
#define TIOCSERGSTRUCT	0x5458 /* For debugging only */
#define TIOCSERGETLSR   0x5459 /* Get line status register */
#define TIOCSERGETMULTI 0x545A /* Get multiport config  */
#define TIOCSERSETMULTI 0x545B /* Set multiport config */

#define TIOCMIWAIT	0x545C	/* wait for a change on serial input line(s) */
#define TIOCGICOUNT	0x545D	/* read serial port inline interrupt counts */
#define FIOQSIZE	0x545E

/* Used for packet mode */
#define TIOCPKT_DATA		 0
#define TIOCPKT_FLUSHREAD	 1
#define TIOCPKT_FLUSHWRITE	 2
#define TIOCPKT_STOP		 4
#define TIOCPKT_START		 8
#define TIOCPKT_NOSTOP		16
#define TIOCPKT_DOSTOP		32
#define TIOCPKT_MSC		64

#define TIOCSER_TEMT	0x01	/* Transmitter physically empty */

#endif
