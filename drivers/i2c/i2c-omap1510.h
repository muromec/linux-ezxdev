/*
 * File: i2c-omap1510.h
 *
 * TI OMAP1510 I2C module register definitions
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
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
 *
 */

#ifndef I2C_OMAP1510_H
#define I2C_OMAP1510_H
#include <linux/i2c.h>

struct i2c_algo_omap1510_data {
	void *data;		/* private data for low-level routines  */
	void (*setomap1510) (void *data, int ctl, u16 val);
	 u16(*getomap1510) (void *data, int ctl);
	int (*getown) (void *data);
	int (*getclock) (void *data);
	void (*waitforpin) (void);

	/* local settings */
	int udelay;
	int mdelay;
	int timeout;
};

/* I2C Registers: */

#define I2C_BASE	(0xfffb3800UL)
#define I2C_IOSIZE	(0x40)
#define I2C_REV		(I2C_BASE + 0x00)
#define I2C_IE		(I2C_BASE + 0x04)
#define I2C_STAT	(I2C_BASE + 0x08)
#define I2C_IV		(I2C_BASE + 0x0c)
#define I2C_BUF		(I2C_BASE + 0x14)
#define I2C_CNT		(I2C_BASE + 0x18)
#define I2C_DATA	(I2C_BASE + 0x1c)
#define I2C_CON		(I2C_BASE + 0x24)
#define I2C_OA		(I2C_BASE + 0x28)
#define I2C_SA		(I2C_BASE + 0x2c)
#define I2C_PSC		(I2C_BASE + 0x30)
#define I2C_SCLL	(I2C_BASE + 0x34)
#define I2C_SCLH	(I2C_BASE + 0x38)
#define I2C_SYSTEST	(I2C_BASE + 0x3c)

struct omap1510_i2c_regs {
	volatile u16 rev;	/* 00 */
	volatile u16 pad0;
	volatile u16 ie;	/* 04 */
	volatile u16 pad1;
	volatile u16 stat;	/* 08 */
	volatile u16 pad2;
	volatile u16 iv;	/* 0c */
	volatile u16 pad3;
	volatile u32 rsrvd0;	/* 10 */
	volatile u16 buf;	/* 14 */
	volatile u16 pad4;
	volatile u16 cnt;	/* 18 */
	volatile u16 pad5;
	volatile u16 data;
	volatile u16 pad6;
	volatile u32 rsrvd1;	/* 20 */
	volatile u16 con;	/* 24 */
	volatile u16 pad7;
	volatile u16 oa;	/* 28 */
	volatile u16 pad8;
	volatile u16 sa;	/* 2c */
	volatile u16 pad9;
	volatile u16 psc;	/* 30 */
	volatile u16 pada;
	volatile u16 scll;	/* 34 */
	volatile u16 padb;
	volatile u16 sclh;	/* 38 */
	volatile u16 padc;
	volatile u16 systest;	/* 3c */
	volatile u16 padd;
};

typedef struct omap1510_i2c_regs omap1510_i2c_t;
typedef struct omap1510_i2c_regs * omap1510_i2c_p;

/* I2C Interrupt Enable Register (I2C_IE): */

#define I2C_IE_XRDY_IE	(1 << 4)	/* Transmit data ready interrupt enable */
#define I2C_IE_RRDY_IE	(1 << 3)	/* Receive data ready interrupt enable */
#define I2C_IE_ARDY_IE	(1 << 2)	/* Register access ready interrupt enable */
#define I2C_IE_NACK_IE	(1 << 1)	/* No acknowledgment interrupt enable */
#define I2C_IE_AL_IE	(1 << 0)	/* Arbitration lost interrupt enable */

/* I2C Status Register (I2C_STAT): */

#define I2C_STAT_SBD	(1 << 15)	/* Single byte data */
#define I2C_STAT_BB	(1 << 12)	/* Bus busy */
#define I2C_STAT_ROVR	(1 << 11)	/* Receive overrun */
#define I2C_STAT_XUDF	(1 << 10)	/* Transmit underflow */
#define I2C_STAT_AAS	(1 << 9)	/* Address as slave */
#define I2C_STAT_AD0	(1 << 8)	/* Address zero */
#define I2C_STAT_XRDY	(1 << 4)	/* Transmit data ready */
#define I2C_STAT_RRDY	(1 << 3)	/* Receive data ready */
#define I2C_STAT_ARDY	(1 << 2)	/* Register access ready */
#define I2C_STAT_NACK	(1 << 1)	/* No acknowledgment interrupt enable */
#define I2C_STAT_AL	(1 << 0)	/* Arbitration lost interrupt enable */

/* I2C Interrupt Vector Register (I2C_IV): */

/* I2C Interrupt Code Register (I2C_INTCODE): */

#define I2C_INTCODE_MASK	7
#define I2C_INTCODE_NONE	0
#define I2C_INTCODE_AL		1	/* Arbitration lost */
#define	I2C_INTCODE_NAK		2	/* No acknowledgement/general call */
#define I2C_INTCODE_ARDY	3	/* Register access ready */
#define I2C_INTCODE_RRDY	4	/* Rcv data ready */
#define I2C_INTCODE_XRDY	5	/* Xmit data ready */

/* I2C Buffer Configuration Register (I2C_BUF): */

#define I2C_BUF_RDMA_EN		(1 << 15)	/* Receive DMA channel enable */
#define I2C_BUF_XDMA_EN		(1 << 7)	/* Transmit DMA channel enable */

/* I2C Configuration Register (I2C_CON): */

#define I2C_CON_EN	(1 << 15)	/* I2C module enable */
#define I2C_CON_BE	(1 << 14)	/* Big endian mode */
#define I2C_CON_STB	(1 << 11)	/* Start byte mode (master mode only) */
#define I2C_CON_MST	(1 << 10)	/* Master/slave mode */
#define I2C_CON_TRX	(1 << 9)	/* Transmitter/receiver mode (master mode only) */
#define I2C_CON_XA	(1 << 8)	/* Expand address */
#define I2C_CON_RM	(1 << 2)	/* Repeat mode (master mode only) */
#define I2C_CON_STP	(1 << 1)	/* Stop condition (master mode only) */
#define I2C_CON_STT	(1 << 0)	/* Start condition (master mode only) */

/* I2C System Test Register (I2C_SYSTEST): */

#define I2C_SYSTEST_ST_EN	(1 << 15)	/* System test enable */
#define I2C_SYSTEST_FREE	(1 << 14)	/* Free running mode (on breakpoint) */
#define I2C_SYSTEST_TMODE_MASK	(3 << 12)	/* Test mode select */
#define I2C_SYSTEST_TMODE_SHIFT	(12)		/* Test mode select */
#define I2C_SYSTEST_SCL_I	(1 << 3)	/* SCL line sense input value */
#define I2C_SYSTEST_SCL_O	(1 << 2)	/* SCL line drive output value */
#define I2C_SYSTEST_SDA_I	(1 << 1)	/* SDA line sense input value */
#define I2C_SYSTEST_SDA_O	(1 << 0)	/* SDA line drive output value */
#endif /* I2C_OMAP1510_H */
