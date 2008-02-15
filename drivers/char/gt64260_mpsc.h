/*
 *	Linux Monitor (LiMon) - Galileo serial port header file
 *
 *	  Copyright 2001 Xyterra Computing, Inc.
 *	  (See Xyterra.license)
 */

#ifndef __SERIAL_H__
#define	__SERIAL_H__

#include <asm/gt64260.h>
#include <asm/page.h>

/* Status bits in the SDMA header for UART mode */
#define STS_OWN		(1<<31)	/* Descriptor owner */
#define STS_AUTO	(1<<30)	/* Auto Mode */
#define STS_EI		(1<<23) /* Enable Interrupt */
#define STS_NS		(1<<20) /* [tx] data sent sans stop bit */
#define STS_ADDR	(1<<19) /* [tx] */
#define STS_PREAMBLE	(1<<18) /* [tx] */
#define STS_FIRST	(1<<17)	/* First in Frame */
#define STS_LAST	(1<<16) /* Last in Frame */
#define STS_ES		(1<<15)	/* Error Summery */
#define STS_C		(1<<14) /* [rx] */
#define STS_CT		(1<<13) /* [rx] */
#define STS_AM		(1<<12) /* [rx] address match */
#define STS_A		(1<<11) /* [rx] */
#define STS_MI		(1<<10) /* [rx] Max Idle expired */
#define STS_BR		(1<<9)	/* [rx] Break Received */
#define STS_OR		(1<<6)	/* [rx] Data Overrun */
#define STS_FR		(1<<3)	/* [rx] Framing Error */
#define STS_CTSL	(1<<1)	/* [tx] CTS Lost */
#define STS_CDL		(1<<1)	/* [rx] Carrier Detect Lost */
#define STS_PE		(1<<0)	/* [rx] Parity Error */

/* SDMA command register */
#define SDMA_ABORT_TX	(1<<31)
#define SDMA_DEMAND_TX	(1<<23)
#define SDMA_STOP_TX	(1<<16)
#define SDMA_ABORT_RX	(1<<15)
#define SDMA_ENABLE_RX	(1<<7)

/* Magic values for MPSC */
#define GALMPSC_CONNECT            0x1
#define GALMPSC_DISCONNECT         0x0

#define GALMPSC_UART               0x1

#define GALMPSC_STOP_BITS_1        0x0
#define GALMPSC_STOP_BITS_2        0x1

#define GALMPSC_CHAR_LENGTH_5	   0x0
#define GALMPSC_CHAR_LENGTH_6	   0x1
#define GALMPSC_CHAR_LENGTH_7      0x2
#define GALMPSC_CHAR_LENGTH_8      0x3

#define GALMPSC_PARITY_ODD         0x0
#define GALMPSC_PARITY_EVEN        0x2
#define GALMPSC_PARITY_MARK        0x3
#define GALMPSC_PARITY_SPACE       0x1
#define GALMPSC_PARITY_NONE        -1


#endif	/* __SERIAL_H__ */
