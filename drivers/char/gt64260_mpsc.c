/* UART driver for Galileo/Marvell GT-642x0 MPSC
 *
 * Based upon the work of:
 *	Chris Zankel <chris@mvista.com>
 *	Troy Benjegerdes <tbenjegerdes@mvista.com>
 *	Mark Greer <mgreer@mvista.com>
 *	Dave Wilhardt <dwilhardt@xyterra.com
 *	Dan Malek (arch/ppc/8xx_io/uart.c)
 *	??? (drivers/char/serial_21285.c)
 *
 * Maintained by: Rex Feany <rfeany@zumanetworks.com>
 *
 * In an ideal world, this driver will be used by both MIPS and PowerPC
 * that use the gt64240 and gt64260 bridge chips. I need to either get a
 * 64240 mips board or find someone to test this.. -- Troy
 * 
 */

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/slab.h>

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/circ_buf.h>

#ifdef CONFIG_GT64260_CONSOLE
# include <linux/console.h>
#endif
#ifdef CONFIG_USE_PPCBOOT
# include <asm/ppcboot.h>
#endif


#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <asm/poll.h>
#include <asm/cache.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm-ppc/gt64260.h>

#include "gt64260_mpsc.h"

/* Number of Tx descriptors */
#define NUM_TX_DESC	64

/* How much do we send in one tx */
#define TX_BUF_SIZE	64

/* How much per descriptor */
#define RX_BUF_SIZE	128

/* Number of Rx descriptors */
#define NUM_RX_DESC	32

/* Do explicit cache management, or allow snooping? */
#define SOFTWARE_CACHE_MGMT

/* Number of ports we handle. */
#define NR_PORTS 2

/* How long before a transmit timesout [ms] */
#define WAIT_FOR_TXD_TIME      (30*1000)

/* Maximum Idle Register value */
#define DEFAULT_MIR	40

/* Which port is console? */
#ifdef CONFIG_GT64260_CONSOLE
# if defined(CONFIG_GT64260_CONSOLE_1)
#  define CONFIG_SERIAL_CONSOLE_PORT	1
# else
#  define CONFIG_SERIAL_CONSOLE_PORT	0
# endif
#endif

/* I bet you can figure this one out */
#define SDMA_IRQ 36

#ifdef CONFIG_DEVFS_FS
# define GT_MPSC_DEVNAME "tts/%d"
# define GT_MPSC_AUXNAME "cua/%d"
#else
# define GT_MPSC_DEVNAME "ttyS"
# define GT_MPSC_AUXNAME "cua"
#endif

#define GT_MPSC_MAJOR		TTY_MAJOR
#define GT_MPSC_MINOR_START	64
#define GT_MPSC_AUXMAJOR	TTYAUX_MAJOR

/*
 * We can do explicit cache management, or let
 * the galileo snoop take care of it.
 */
#ifdef SOFTWARE_CACHE_MGMT
#  define FLUSH_DCACHE(st, en)		flush_dcache_range(st, en)
#  define INVALIDATE_DCACHE(st, en)	invalidate_dcache_range(st, en)
#  define CLEAN_DCACHE(st, en)		clean_dcache_range(st, en)
#  define WANT_SNOOP 0
#else
#  define FLUSH_DCACHE(st, en)
#  define INVALIDATE_DCACHE(st, en)
#  define CLEAN_DCACHE(st, en)
#  define WANT_SNOOP 1
#endif

/* States for tx descriptors */
#define TX_FREE 0
#define TX_WAITING 1
#define TX_ACTIVE 2

/* Gaps between ch0/ch1 registers */
#define GALMPSC_REG_GAP            0x1000
#define GALBRG_REG_GAP             0x0008
#define GALSDMA_REG_GAP            0x2000

#define MPSC_TX_ABORT             (1 << 7)
#define MPSC_RX_ABORT             (1 << 23)
#define MPSC_ENTER_HUNT           (1 << 31)


/* 
 * Transmit/Receive descriptors. 
 * 4 long word alignment
 */
typedef struct mpsc_rx_desc {
        u16   bufsize;
        u16   bytecnt;
        u32   cmd_sts;
        u32   next_desc_ptr;
        u32   buf_ptr;
} __attribute((aligned(256),packed)) rxd_t;

typedef struct mpsc_tx_desc {
        u16   bytecnt;
        u16   shadow;
        u32   cmd_sts;
        u32   next_desc_ptr;
        u32   buf_ptr;
} __attribute((aligned(256),packed)) txd_t;

struct mpsc_port {
	int			magic;
	int			flags;
	int			count;
	struct tty_struct 	*tty;
	int			line;
	long			session; /* Session of opening process */
	long			pgrp; /* pgrp of opening process */

	/* Receive */
	unsigned int rx_current;
	volatile rxd_t rx_ring[NUM_RX_DESC];
	volatile u8 rx_buffer[NUM_RX_DESC][RX_BUF_SIZE];

	/* How full rx buffers get */
	int rx_hiwater;

	/* What we have set in the hardware */
	struct termios termios;

	/* Request that no more transmits are done */
	int tx_stop;

	/* How much stuff is queued to go out? */
	atomic_t tx_len;

	/* Are we transmitting something now? */
	int tx_running;

	/* High-water mark for tx queue */
	int tx_hiwater;

	/* Current descriptor for gt_write */
	int tx_current;

	/* Current descriptor for transmit */
	int tx_active;

	/* Status of each descriptor */
	int tx_status[NUM_TX_DESC];

	/* Transmit descriptor ring & buffers */
	volatile txd_t tx_ring[NUM_TX_DESC];
	volatile u8 tx_buffer[NUM_TX_DESC][TX_BUF_SIZE];

	/* Statistics baby */
	struct async_icount icount;
};

/* 
 * GT64240A errata: cant read MPSC/BRG registers... 
 * so make mirrors in ram for read/modify write.
 */

#define GT_REG_WRITE_MIRROR_G(a,d) {mh.a ## _M = d; gt_write(a,d);}
#define GT_REG_READ_MIRROR_G(a) (mh.a ## _M)

#define GT_REG_WRITE_MIRROR(a,i,g,d) {mh.a ## _M[i] = d; gt_write(a + (i*g),d);}
#define GT_REG_READ_MIRROR(a,i,g) (mh.a ## _M[i])

struct _tag_mirror_hack {
	unsigned GT64260_BRG_0_BCR_M[2];	/* b200 */
	unsigned GT64260_MPSC_0_CHR_1_M[2];	/* 800c */
	unsigned GT64260_MPSC_0_CHR_2_M[2];	/* 8010 */
	unsigned GT64260_MPSC_0_MPCR_M[2];	/* 8008 */

	unsigned GT64260_MPSC_MRR_M;		/* b400 */
	unsigned GT64260_MPP_SERIAL_PORTS_MULTIPLEX_M; /* 0xf010 */
	unsigned GT64260_MPSC_RCRR_M;		/* b404 */
	unsigned GT64260_MPSC_TCRR_M;		/* b408 */
	unsigned GT64260_SDMA_INTR_MASK_M;	/* b880 */
};

static struct _tag_mirror_hack mh;

static struct tty_driver serial_driver;
static int gt_refcount;
static struct mpsc_port mpsc_ports[NR_PORTS];
static struct termios *gt_termios[NR_PORTS];
static struct termios *gt_termios_locked[NR_PORTS];
static struct tty_struct *gt_table[NR_PORTS];


/* used by gt_tty_write. no reason to have one per port (see comments in serial.c). */
static unsigned char *tmp_buf;
static DECLARE_MUTEX(tmp_buf_sem);

#ifdef CONFIG_GT64260_CONSOLE
extern struct console sercons;
#endif

#ifdef	CONFIG_USE_PPCBOOT
extern bd_t ppcboot_bd;
extern int ppcboot_bd_valid;
#endif

static void gt_tty_send_xchar(struct tty_struct *tty, char ch);

static char serial_version[] = "%W%";

/*
 * This is used to figure out the divisor speeds and the timeouts,
 * indexed by the termio value.
 */
static int baud_table[] = {
	0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800,
	9600, 19200, 38400, 57600, 115200, 230400, 460800, 0 };


#if 0
static void
putstr(char *s)
{
	char c;

	while ((c = *s++) != 0) {
		gt_tty_send_xchar(0, c);
		if (c == '\n')
			gt_tty_send_xchar(0, '\r');
	}
	gt_tty_send_xchar(0, '\n');
	gt_tty_send_xchar(0, '\r');
}

static void
dump_tx(struct mpsc_port *info, const char *tag)
{
	char str[1024];
	int i;

	sprintf(str, "** TX DUMP LINE %1.1d <%s>", info->line, tag);
	putstr(str);
	sprintf(str, "** %d tx, I am %c! %d bytes backlog! I am also %c!",
			NUM_TX_DESC,
			info->tx_running ? 'R' : 'W',
			atomic_read(&info->tx_len),
			info->tx_stop ? 'H' : 'N');
	putstr(str);
	
	sprintf(str, "** Ring Dump");
	putstr(str);
	sprintf(str, "### ptr      <   buf  > C A   F W A   Bytecnt Shadow");
	putstr(str);

	for (i = 0; i < NUM_TX_DESC; i++) {
		sprintf(str, "%3.3d %lx <%lx> %c %c   %c %c %c       %.4d     %.4d",
				i,
				(unsigned long) &info->tx_ring[i], 
				(unsigned long) info->tx_buffer[i], 
				i == info->tx_current ? 'X' : '-',
				i == info->tx_active ? 'X' : '-',
				info->tx_status[i] == TX_FREE ? 'X' : '-',
				info->tx_status[i] == TX_WAITING ? 'X' : '-',
				info->tx_status[i] == TX_ACTIVE ? 'X' : '-',
				info->tx_ring[i].bytecnt,
				info->tx_ring[i].shadow);
		putstr(str);
	}
}
#endif

/*************************************************************
 * Setup routines for MPSC, BRG, and SDMA. 
 * Stolen from the ppcboot MPSC driver.
 */

static void
galbrg_set_CDV(int channel, int value)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR(GT64260_BRG_0_BCR, channel, GALBRG_REG_GAP);
	temp &= 0xFFFF0000;
	temp |= (value & 0x0000FFFF);
	GT_REG_WRITE_MIRROR(GT64260_BRG_0_BCR,channel,GALBRG_REG_GAP, temp);
}

static void
galbrg_set_CUV(int channel, int value)
{
	gt_write(GT64260_BRG_0_BTR + (channel * GALBRG_REG_GAP), value);
}

static void
galbrg_set_clksrc(int channel, int value)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR(GT64260_BRG_0_BCR,channel, GALBRG_REG_GAP);
	temp &= 0xFF83FFFF;
	temp |= (value << 18); 
	GT_REG_WRITE_MIRROR(GT64260_BRG_0_BCR,channel, GALBRG_REG_GAP,temp);
}

static void
galbrg_enable(int channel)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR(GT64260_BRG_0_BCR, channel, GALBRG_REG_GAP);
	temp |= 0x00010000;
	GT_REG_WRITE_MIRROR(GT64260_BRG_0_BCR, channel, GALBRG_REG_GAP,temp);
}

static void
galbrg_disable(int channel)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR(GT64260_BRG_0_BCR, channel, GALBRG_REG_GAP);
	temp &= 0xFFFEFFFF;
	GT_REG_WRITE_MIRROR(GT64260_BRG_0_BCR, channel, GALBRG_REG_GAP,temp);
}

static void
galbrg_set_baudrate(int channel, int rate) 
{
#ifdef CONFIG_GT64260_BRG_EQ_BUS
	int clk = ppcboot_bd_valid?ppcboot_bd.bi_busfreq:100000000;
#else
	int clk = CONFIG_GT64260_BRG_CLK_RATE;
#endif
	int clock = (clk/(16*rate)) - 1;

	galbrg_disable(channel);
	galbrg_set_CDV(channel, clock);
	galbrg_enable(channel);
}

static void
galmpsc_connect(int channel, int connect)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR_G(GT64260_MPSC_MRR);
	if ((channel == 0) && connect) 
		temp &= ~0x00000007;
	else if ((channel == 1) && connect)
		temp &= ~(0x00000007 << 6);
	else if ((channel == 0) && !connect)
		temp |= 0x00000007;
	else
		temp |= (0x00000007 << 6);

	/* Just in case... */
	temp &= 0x3fffffff;
	GT_REG_WRITE_MIRROR_G(GT64260_MPSC_MRR, temp);
}

static void
galmpsc_route_serial(int channel, int connect)
{
	unsigned int temp;
	
	temp = gt_read(GT64260_MPP_SERIAL_PORTS_MULTIPLEX);

	if ((channel == 0) && connect) 
		temp |= 0x00000100;
	else if ((channel == 1) && connect)
		temp |= 0x00001000;
	else if ((channel == 0) && !connect)
		temp &= ~0x00000100;
	else
		temp &= ~0x00001000;

	gt_write(GT64260_MPP_SERIAL_PORTS_MULTIPLEX,temp);
}

static void
galmpsc_route_rx_clock(int channel, int brg)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR_G(GT64260_MPSC_RCRR);

	if (channel == 0) 
		temp |= brg;
	else
		temp |= (brg << 8);

	GT_REG_WRITE_MIRROR_G(GT64260_MPSC_RCRR,temp);
}

static void
galmpsc_route_tx_clock(int channel, int brg)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR_G(GT64260_MPSC_TCRR);

	if (channel == 0) 
		temp |= brg;
	else
		temp |= (brg << 8);

	GT_REG_WRITE_MIRROR_G(GT64260_MPSC_TCRR,temp);
}

static void
galmpsc_write_config_regs(int mpsc)
{
	/* Main config reg Low (Null modem, Enable Tx/Rx, UART mode) */
	gt_write(GT64260_MPSC_0_MMCRL + (mpsc*GALMPSC_REG_GAP), 0x000004c4);
		
	/* Main config reg High (32x Rx/Tx clock mode, width=8bits */
	gt_write(GT64260_MPSC_0_MMCRH +(mpsc*GALMPSC_REG_GAP), 0x024003f8);
		//        22 2222 1111
		//        54 3210 9876
		// 0000 0010 0000 0000
		//       1
		//       098 7654 3210
		// 0000 0011 1111 1000
}

static void
galmpsc_set_brkcnt(int mpsc, int value)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR(GT64260_MPSC_0_CHR_1,mpsc,GALMPSC_REG_GAP);
	temp &= 0x0000FFFF;
	temp |= (value << 16);
	GT_REG_WRITE_MIRROR(GT64260_MPSC_0_CHR_1,mpsc,GALMPSC_REG_GAP, temp);
}

static void
galmpsc_set_tcschar(int mpsc, int value)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR(GT64260_MPSC_0_CHR_1,mpsc,GALMPSC_REG_GAP);
	temp &= 0xFFFF0000;
	temp |= value;
	GT_REG_WRITE_MIRROR(GT64260_MPSC_0_CHR_1,mpsc,GALMPSC_REG_GAP, temp);
}


static void
galmpsc_set_char_length(int mpsc, int value)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR(GT64260_MPSC_0_MPCR,mpsc,GALMPSC_REG_GAP);
	temp &= 0xFFFFCFFF;
	temp |= (value << 12);
	GT_REG_WRITE_MIRROR(GT64260_MPSC_0_MPCR,mpsc,GALMPSC_REG_GAP, temp);
}

static void
galmpsc_set_stop_bit_length(int mpsc, int value)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR(GT64260_MPSC_0_MPCR,mpsc,GALMPSC_REG_GAP);
	temp |= (value << 14);
	GT_REG_WRITE_MIRROR(GT64260_MPSC_0_MPCR,mpsc,GALMPSC_REG_GAP,temp);
}

static void
galmpsc_set_parity(int mpsc, int value)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR(GT64260_MPSC_0_CHR_2,mpsc,GALMPSC_REG_GAP);

	if (value != GALMPSC_PARITY_NONE) {
		temp &= 0xFFF3FFF3;
		temp |= ((value << 18) | (value << 2));
		temp |= ((value << 17) | (value << 1));
	} else {
		temp &= 0xFFF1FFF1;
	}
	GT_REG_WRITE_MIRROR(GT64260_MPSC_0_CHR_2,mpsc,GALMPSC_REG_GAP, temp);
}

static void
galmpsc_set_snoop(int mpsc, int value)
{
	unsigned long reg = mpsc ? GT64260_MPSC_1_CNTL_LO : GT64260_MPSC_0_CNTL_LO;
	unsigned int temp;

	temp = gt_read(reg);
	if(value)
		temp |= (1<< 6) | (1<<14) | (1<<22) | (1<<30);
	else
		temp &= ~((1<< 6) | (1<<14) | (1<<22) | (1<<30));
	gt_write(reg, temp);
}

static void
galmpsc_enter_hunt(int mpsc)
{
	unsigned int temp;
	
	temp = GT_REG_READ_MIRROR(GT64260_MPSC_0_CHR_2,mpsc,GALMPSC_REG_GAP);
	temp |= 0x80000000;
	GT_REG_WRITE_MIRROR(GT64260_MPSC_0_CHR_2,mpsc,GALMPSC_REG_GAP, temp);

	/* Should Poll on Enter Hunt bit, but the register is write-only.
	   Errata suggests pausing 100 system cycles. */
	udelay(100);
}

static void
galmpsc_set_MIR(int mpsc, int value)
{
	gt_write(GT64260_MPSC_0_CHR_3+(mpsc*GALMPSC_REG_GAP), value);
}

static void
galmpsc_config_channel_regs(int mpsc)
{
	GT_REG_WRITE_MIRROR(GT64260_MPSC_0_CHR_1,mpsc,GALMPSC_REG_GAP, 0);
	GT_REG_WRITE_MIRROR(GT64260_MPSC_0_CHR_2,mpsc,GALMPSC_REG_GAP, 0);

	gt_write(GT64260_MPSC_0_CHR_4+(mpsc*GALMPSC_REG_GAP), 0);
	gt_write(GT64260_MPSC_0_CHR_5+(mpsc*GALMPSC_REG_GAP), 0);
	gt_write(GT64260_MPSC_0_CHR_6+(mpsc*GALMPSC_REG_GAP), 0);
	gt_write(GT64260_MPSC_0_CHR_7+(mpsc*GALMPSC_REG_GAP), 0);
	gt_write(GT64260_MPSC_0_CHR_8+(mpsc*GALMPSC_REG_GAP), 0);
	gt_write(GT64260_MPSC_0_CHR_9+(mpsc*GALMPSC_REG_GAP), 0);
	gt_write(GT64260_MPSC_0_CHR_10+(mpsc*GALMPSC_REG_GAP), 0);

	galmpsc_set_brkcnt(mpsc, 0x3);
	galmpsc_set_tcschar(mpsc, 0xab);
}

static void
galmpsc_freeze_tx(int mpsc, int onoff)
{
	unsigned int temp;

	temp = GT_REG_READ_MIRROR(GT64260_MPSC_0_MPCR,mpsc,GALMPSC_REG_GAP);
	if (onoff) temp |= (1<<9);
	else temp &= ~(1<<9);
	GT_REG_WRITE_MIRROR(GT64260_MPSC_0_MPCR,mpsc,GALMPSC_REG_GAP, temp);
}

static void
galsdma_set_RFT(int channel)
{
	unsigned int temp;

        temp = gt_read(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP));
	temp |= 0x00000001;
	gt_write(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP), temp);
}

static void
galsdma_set_SFM(int channel)
{
	unsigned int temp;

        temp = gt_read(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP));
	temp |= 0x00000002;
	gt_write(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP), temp);
}

static void
galsdma_set_rxbe(int channel)
{
	unsigned int temp;

        temp = gt_read(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP));
	temp &= ~0x00000040;
	gt_write(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP), temp);
}

static void
galsdma_set_txbe(int channel)
{
	unsigned int temp;

        temp = gt_read(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP));
	temp &= ~0x00000080;
	gt_write(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP), temp);
}

static void
galsdma_set_RC(int channel, unsigned int value)
{
	unsigned int temp;

	temp = gt_read(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP));
	temp &= ~0x0000003c;
	temp |= (value << 2);
	gt_write(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP), temp);
}

static unsigned int
galsdma_intr_mask(int channel, unsigned int value)
{
	unsigned int temp;
	unsigned int old;

	old = temp = GT_REG_READ_MIRROR_G(GT64260_SDMA_INTR_MASK);
	value &= 0xf;
	if (channel) value <<= 8;
	temp &= ~value;
	GT_REG_WRITE_MIRROR_G(GT64260_SDMA_INTR_MASK, temp);

	if (channel) old >>= 8;
	return old & 0xf;
}

static void
galsdma_intr_unmask(int channel, unsigned int value)
{
	unsigned temp;

	temp = GT_REG_READ_MIRROR_G(GT64260_SDMA_INTR_MASK);
	value &= 0xf;
	if (channel) value <<= 8;
	temp |= value;
	GT_REG_WRITE_MIRROR_G(GT64260_SDMA_INTR_MASK, temp);
}

static inline void
galsdma_intr_ack(void)
{
	gt_write(GT64260_SDMA_INTR_CAUSE, 0);
}

static inline void
galsdma_set_rx_ring(int chan, unsigned long val)
{
	gt_write(GT64260_SDMA_0_SCRDP+(chan*GALSDMA_REG_GAP), val);
}

static inline void
galsdma_set_tx_ring(int chan, unsigned long val)
{
	gt_write(GT64260_SDMA_0_SFTDP+(chan*GALSDMA_REG_GAP), val);
	gt_write(GT64260_SDMA_0_SCTDP+(chan*GALSDMA_REG_GAP), val);
}

static void
galsdma_set_burstsize(int channel, unsigned int value)
{
	unsigned int temp;

	temp = gt_read(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP));
	temp &= 0xFFFFCFFF;

	switch (value) {
		case 8: temp |= 0x3 << 12; break;
		case 4: temp |= 0x2 << 12; break;
		case 2: temp |= 0x1 << 12; break;
		case 1: temp |= 0x0 << 12; break;
	}
	gt_write(GT64260_SDMA_0_SDC+(channel*GALSDMA_REG_GAP), temp);
}

static void
galsdma_request(int chan, unsigned int val)
{
	unsigned int temp;

	temp = gt_read(GT64260_SDMA_0_SDCM+(chan*GALSDMA_REG_GAP));
	if (val)
		temp |= val;
	else
		temp = 0;
	gt_write(GT64260_SDMA_0_SDCM+(chan*GALSDMA_REG_GAP), temp);
}

static void
gt_shutdown(int chan)
{
	/* Abort any SDMA transfers */
	galsdma_request(chan, 0);
	galsdma_request(chan, SDMA_ABORT_TX|SDMA_ABORT_RX);

#if 0
	/* shut down the MPSC */
	gt_write(GT64260_MPSC_0_MMCRL+(chan*GALMPSC_REG_GAP), 0);
	gt_write(GT64260_MPSC_0_MMCRH+(chan*GALMPSC_REG_GAP), 0);
	GT_REG_WRITE_MIRROR(GT64260_MPSC_0_MPCR, chan, GALMPSC_REG_GAP,0);
	udelay(100000);
#endif

	/* clear the SDMA current and first TX and RX pointers */
	galsdma_set_tx_ring(chan, 0);
	galsdma_set_rx_ring(chan, 0);
	udelay(100);

	/* Disable interrupts */
	galsdma_intr_mask(chan, 0xf);
	galsdma_intr_ack();
        udelay(1000);
}

static void
galsdma_enable_rx(int chan)
{
	galmpsc_enter_hunt(chan);
	galsdma_request(chan, SDMA_ENABLE_RX);
}

static void
ll_mpsc_init(int chan)
{
	/* BRG config */
	galbrg_set_clksrc(chan, CONFIG_GT64260_CLKSRC);
	galbrg_set_CUV(chan, 0);
	galbrg_enable(chan);

	/* Set up clock routing */
	galmpsc_connect(chan, GALMPSC_CONNECT);
	galmpsc_route_serial(chan, GALMPSC_CONNECT);
	galmpsc_route_rx_clock(chan, chan);
	galmpsc_route_tx_clock(chan, chan);

	/* SDMA config */
	galsdma_set_burstsize(chan, L1_CACHE_BYTES/8);
	galsdma_set_txbe(chan);
	galsdma_set_rxbe(chan);
	galsdma_set_RC(chan, 0xf);
	galsdma_set_SFM(chan);
	galsdma_set_RFT(chan);

	/* MPSC config */
	gt_shutdown(chan); 
	galmpsc_write_config_regs(chan);
	galmpsc_config_channel_regs(chan);
	galmpsc_set_MIR(chan, DEFAULT_MIR);
	galmpsc_set_char_length(chan, GALMPSC_CHAR_LENGTH_8);       /* 8 */
	galmpsc_set_parity(chan, GALMPSC_PARITY_NONE);              /* N */
	galmpsc_set_stop_bit_length(chan, GALMPSC_STOP_BITS_1);     /* 1 */
	galmpsc_set_snoop(chan, WANT_SNOOP);
}

static void 
gt_set_cflag(int chan, int cflag)
{
	int baud;
	int arg;

	/* Character size */
	switch (cflag & CSIZE) {
		case CS5: arg = GALMPSC_CHAR_LENGTH_5; break;
		case CS6: arg = GALMPSC_CHAR_LENGTH_6; break;
		case CS7: arg = GALMPSC_CHAR_LENGTH_7; break;
		case CS8: arg = GALMPSC_CHAR_LENGTH_8; break;
		default: arg = GALMPSC_CHAR_LENGTH_8; break;
	}
	galmpsc_set_char_length(chan, arg);

	/* Stop bits */
	arg = GALMPSC_STOP_BITS_1;
	if (cflag & CSTOPB)
		arg = GALMPSC_STOP_BITS_2;
	galmpsc_set_stop_bit_length(chan, arg);

	/* Parity */
	arg = GALMPSC_PARITY_NONE;
	if (cflag & PARENB) {
		arg = GALMPSC_PARITY_EVEN;
		if (cflag & PARODD)
			arg = GALMPSC_PARITY_ODD;
	}
	galmpsc_set_parity(chan, arg);

	switch (cflag & CBAUD) {
		case B200:	baud = 200;	break;
		case B300:	baud = 300;	break;
		case B1200:	baud = 1200;	break;
		case B1800:	baud = 1800;	break;
		case B2400:	baud = 2400;	break;
		case B4800:	baud = 4800;	break;
		default:
		case B9600:	baud = 9600;	break;
		case B19200:	baud = 19200;	break;
		case B38400:	baud = 38400;	break;
		case B57600:	baud = 57600;	break;
		case B115200:	baud = 115200;	break;
	}
	galbrg_set_baudrate(chan, baud);
}

/* gt_tx_sync - Wait for current transmit to finish */
static void
gt_tx_sync(int chan)
{
	int i=0;

	/* ugh, the MVP seems to get stuck and needs to have transmit aborts for
	 * some unknown reason -- Troy */
	/* we probably should do something nicer than udelay..
	 * is schedule_timeout safe here? */

	while(gt_read(GT64260_SDMA_0_SDCM+(chan*GALSDMA_REG_GAP)) & SDMA_DEMAND_TX) {
		if (i++ > WAIT_FOR_TXD_TIME) {
		    /*
			galsdma_request(chan, SDMA_ABORT_TX);
			printk(KERN_ERR "chan %d: tx sync timeout\n", chan);
		    */
			/* _gt_putc('!'); */
			break;
		}
		udelay(1000);
	}
}

/**********************************************************
 * Serial Driver Proper
 */

/* restart_tx - start tx next buffer.
 * Called from interrupt and task context */
static void
restart_tx(struct mpsc_port *info)
{
        volatile txd_t *txd = &info->tx_ring[info->tx_active];

	if (!info->tx_stop && !info->tx_running && (info->tx_status[info->tx_active] == TX_WAITING)) {
		info->tx_running = 1;
		info->tx_status[info->tx_active] = TX_ACTIVE;

		CLEAN_DCACHE((u32)phys_to_virt((u32)txd->buf_ptr),
				(u32)(phys_to_virt((u32)txd->buf_ptr)+txd->bytecnt));
		CLEAN_DCACHE((u32) txd, (u32)(txd+1));

		galsdma_set_tx_ring(info->line, virt_to_phys((u32 *)txd));
		galsdma_request(info->line, SDMA_STOP_TX|SDMA_DEMAND_TX);
	}
}

/* Close this buffer and mark it to be sent out */
static void
next_tx(struct mpsc_port *info)
{
	volatile txd_t *txd = &info->tx_ring[info->tx_current];

	txd->cmd_sts = STS_EI | STS_OWN | STS_FIRST | STS_LAST;
	txd->shadow = txd->bytecnt;

	info->tx_status[info->tx_current] = TX_WAITING;
	info->tx_current = (info->tx_current + 1) % NUM_TX_DESC;

	CLEAN_DCACHE((u32) txd, (u32)(txd+1));
}

static void
tx_interrupt(struct mpsc_port *info)
{
	struct tty_struct *tty = info->tty;
	volatile txd_t *txd = &info->tx_ring[info->tx_active];

	/* Not expecting anything */
	if (!info->tx_running)
		return;

	INVALIDATE_DCACHE((u32) txd, (u32)(txd+1));
	if (txd->cmd_sts & STS_OWN) 
		return;

	local_bh_disable();
	if (info->tx_status[info->tx_active] != TX_ACTIVE)  
		panic("bad serial status");

	info->icount.tx += txd->bytecnt;
	atomic_sub(txd->bytecnt, &info->tx_len); txd->bytecnt = 0;
	info->tx_status[info->tx_active] = TX_FREE;
	info->tx_active = (info->tx_active + 1) % NUM_TX_DESC;
	info->tx_running = 0;

	/* Try and send some more */
	restart_tx(info);
	local_bh_enable();

	/* Wake up any writers */
	if (tty) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && tty->ldisc.write_wakeup)
			(tty->ldisc.write_wakeup)(tty);
		wake_up_interruptible(&tty->write_wait);
	}
}

static void
rx_interrupt(struct mpsc_port *info)
{
	volatile rxd_t *rxd = &info->rx_ring[info->rx_current];
	struct async_icount *icount = &info->icount;
	struct tty_struct *tty = info->tty;
	unsigned long temp;
	unsigned char *pp;

	INVALIDATE_DCACHE((u32)rxd, (u32)(rxd+1));

	/* fooyah! An interrupt without a ready descriptor looks
	 * suspiciously like an error interrupt. So, we handle it that 
	 * way. */
	if (rxd->cmd_sts & STS_OWN) {
#if 0
		/* galsdma_enable_rx(info->line);  HA! can't do this blindly */ 
		/* it randomly drops chars, and stuff if we do. SO, 
		 * this means if we hit an RX error (such as overun) we are
		 * toast. :( */
#endif
		return;
	}

	/* Try and read as many done descriptors as we can. */
	do {
		unsigned long sts = rxd->cmd_sts;
		unsigned int cnt = rxd->bytecnt;

		/* Clean up used descriptor */
		rxd->cmd_sts = STS_OWN | STS_EI | STS_LAST | STS_FIRST;
		rxd->bytecnt = 0;
		CLEAN_DCACHE((u32)rxd, (u32)(rxd+1));
		info->rx_current = (info->rx_current + 1) % NUM_RX_DESC;

		if (cnt > info->rx_hiwater)
			info->rx_hiwater = cnt;

		if (sts & STS_BR) {
			icount->brk++;
			tty_insert_flip_char(tty, 0, TTY_BREAK);
			tty_schedule_flip(tty);
			return;
		}
	
		/* if there is nowhere to put the data, discard it */
		if (tty == 0)
			continue;

		/* RX anomoly -- galileo bug! */
		if (!cnt) 
			continue;

		/* Make sure there is enough room to store the chars */
		/* which is more important? clearning out the rx ring or
		 * makeing every effort to not drop chars? */
                if (tty->flip.count+cnt >= TTY_FLIPBUF_SIZE) {
			tty->flip.tqueue.routine((void *) tty);
			if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
				icount->buf_overrun++;
				return; // if TTY_DONT_FLIP is set
			}
		}

		icount->rx += cnt;
		pp = phys_to_virt((u32) rxd->buf_ptr);
		INVALIDATE_DCACHE((u32)pp, (u32)(pp+RX_BUF_SIZE));

		if (tty->flip.count < TTY_FLIPBUF_SIZE) {
			int count = min_t(int, cnt, TTY_FLIPBUF_SIZE - tty->flip.count);

			memcpy(tty->flip.char_buf_ptr, pp, count);
			memset(tty->flip.flag_buf_ptr, TTY_NORMAL, count);

			tty->flip.count += count;
			tty->flip.char_buf_ptr += count;

			/* Allow for possible error flag below */
			tty->flip.flag_buf_ptr += count - 1; 

			if (sts & STS_PE) {
				icount->parity++;
				*tty->flip.flag_buf_ptr = TTY_PARITY;
			} else if (sts & STS_FR) {
				icount->frame++;
				*tty->flip.flag_buf_ptr = TTY_FRAME;
			} else if (sts & STS_OR) {
				*tty->flip.flag_buf_ptr = TTY_OVERRUN;
				icount->overrun++;
			}

			tty->flip.flag_buf_ptr++;
			tty_flip_buffer_push(tty);

			/* Oops.. lost some chars */
			if (count < cnt) {
				icount->buf_overrun++;
				return;
			}
		} else {
			icount->buf_overrun++;
			printk("gt64260_mpsc: buffer overrun\n");
		}

		/* Advance to next descriptor */
		rxd = &info->rx_ring[info->rx_current];
		INVALIDATE_DCACHE((u32)rxd, (u32)(rxd+1));
	} while (! (rxd->cmd_sts & STS_OWN));

	/* If RX is disabled here, start it back up again */
	temp = gt_read(GT64260_SDMA_0_SDCM+(info->line*GALSDMA_REG_GAP));
	if ((temp & SDMA_ENABLE_RX) == 0) 
		galsdma_enable_rx(info->line);
}

static void
gt_sdma_interrupt(int irq, void *dev_id, struct pt_regs *fp)
{
	int i;

	galsdma_intr_ack();
	for (i = 0; i < NR_PORTS; i++) {
		struct mpsc_port *info = mpsc_ports + i;

		if (! (info->flags & ASYNC_INITIALIZED))
			continue;

		rx_interrupt(info);
		tx_interrupt(info);
	}
}

static ssize_t
gt_sdma_write(struct mpsc_port *info, const char *data, size_t wlen)
{
	volatile txd_t *txd;
	unsigned long flags;
	int len = wlen;
	int max;

	do {
		u32 buffer;
		
		save_flags(flags); cli();
		if (info->tx_status[info->tx_current] != TX_FREE)
			break;
		txd = &info->tx_ring[info->tx_current];

		max = min_t(int, TX_BUF_SIZE - txd->bytecnt, len);

		buffer = (u32)&info->tx_buffer[info->tx_current][txd->bytecnt];
		memcpy((void *)buffer, data, max);

		/* flush this */
		CLEAN_DCACHE(buffer, buffer + max);
		
		txd->bytecnt += max;
		data += max;
		len -= max;

		if (txd->bytecnt >= TX_BUF_SIZE)
			next_tx(info);
		restore_flags(flags);
	} while (len > 0);

	atomic_add((wlen - len), &info->tx_len);
	if ((max = atomic_read(&info->tx_len)) > info->tx_hiwater)
		info->tx_hiwater = max;

	return wlen - len;
}

static void
setup_dma(struct mpsc_port *info)
{
	int i;
	volatile rxd_t *rxd = info->rx_ring;

	/* Initialize the TX descriptor */
	memset(info->tx_status, TX_FREE, NUM_TX_DESC);
	memset((void *) info->tx_ring, 0, sizeof(txd_t) * NUM_TX_DESC);
	for (i = 0; i < NUM_TX_DESC; i++) {
		info->tx_ring[i].buf_ptr = virt_to_phys((u32 *) info->tx_buffer[i]);
	}

	/* Initialize the RX descriptors */
	for (i = 0; i < NUM_RX_DESC; i++) {
		rxd[i].cmd_sts = STS_OWN | STS_EI | STS_LAST | STS_FIRST;
		rxd[i].buf_ptr = virt_to_phys((u32 *) info->rx_buffer[i]);
		rxd[i].bufsize = RX_BUF_SIZE;
		rxd[i].bytecnt = 0;
		rxd[i].next_desc_ptr = virt_to_phys((u32 *) &rxd[i+1]);
	}
	rxd[NUM_RX_DESC - 1].next_desc_ptr = virt_to_phys((u32 *) &rxd[0]);

	info->rx_current = 0;
	info->tx_current = 0;
	info->tx_active = 0;
	info->tx_running = 0;
	info->tx_stop = 0;
	info->tx_hiwater = 0;
	atomic_set(&info->tx_len, 0);

	CLEAN_DCACHE((u32)&rxd[0], (u32)&rxd[NUM_RX_DESC]);
	CLEAN_DCACHE((u32)info->rx_buffer[0], (u32)info->rx_buffer[NUM_RX_DESC]);

	udelay(10000); 
	galsdma_set_rx_ring(info->line, virt_to_phys((u32 *) &rxd[0]));
}

static int 
gt_startup(struct mpsc_port *info)
{
	unsigned long flags;

	/* return if already initialized */
	if (info->flags & ASYNC_INITIALIZED)
		return 0;

	save_flags(flags); cli();

#ifdef SERIAL_DEBUG_OPEN
	printk("starting up ttys%d ...", info->line);
#endif

	if (info->tty)
		clear_bit(TTY_IO_ERROR, &info->tty->flags);
	ll_mpsc_init(info->line);
	setup_dma(info);
			
	/* clear and enable interrupt */
	galsdma_intr_ack();
	galsdma_intr_unmask(info->line, 0xf);

	/* set flag that the uart is initialized */
	info->flags |= ASYNC_INITIALIZED;

	/* start receiver and go to 'hunt mode' */
	galsdma_enable_rx(info->line);

	restore_flags(flags);
	return 0;
}

/* Is called once before things start running */
static void
setup_once(void)
{
        /* Write-only register cache hack */
	memset(&mh, 0, sizeof(mh));
	mh.GT64260_MPSC_MRR_M=0x3fffffff;

	/* Setup the mpsc port state structure */
	memset(&mpsc_ports, 0, sizeof(struct mpsc_port) * NR_PORTS);
	mpsc_ports[0].line = 0;
	mpsc_ports[1].line = 1;
}


/***********************************************************
 * Interface to upper layers
 */

static int
gt_tty_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;
	struct async_icount cnow, icount;
	unsigned long flags;

	if ((cmd != TIOCGSERIAL) && (cmd != TIOCSSERIAL) &&
	    (cmd != TIOCSERCONFIG) && (cmd != TIOCSERGSTRUCT) && (cmd != TIOCMIWAIT) && (cmd != TIOCGICOUNT)) {
		if (tty->flags & (1 << TTY_IO_ERROR))
			return -EIO;
	}

	/* XXX Most of these are meaningless without modem control lines */
	switch (cmd) {
		case TIOCMGET:
		case TIOCMBIS:
		case TIOCMBIC:
		case TIOCMSET:
		case TIOCGSERIAL:
		case TIOCSSERIAL:
		case TIOCSERCONFIG:
		case TIOCSERGETLSR:
		case TIOCSERGSTRUCT:
		case TIOCMIWAIT:
			return 0;

		case TIOCGICOUNT:
			save_flags(flags); cli();
			cnow = info->icount;
			restore_flags(flags);

			icount.cts = cnow.cts;
			icount.dsr = cnow.dsr;
			icount.rng = cnow.rng;
			icount.dcd = cnow.dcd;
			icount.rx = cnow.rx;
			icount.tx = cnow.tx;
			icount.frame = cnow.frame;
			icount.overrun = cnow.overrun;
			icount.parity = cnow.parity;
			icount.brk = cnow.brk;
			icount.buf_overrun = cnow.buf_overrun;
			
			if (copy_to_user((void *)arg, &icount, sizeof(icount)))
				return -EFAULT;
			return 0;

		case TIOCSERGWILD:
		case TIOCSERSWILD:
			/* "setserial -W" is called in Debian boot */
			printk("TIOCSER?WILD ioctl obsolete, ignored.\n");
			return 0;

		default:
			return -ENOIOCTLCMD;
	}
	return 0;
}

static void
gt_tty_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;
	int orig_jiffies = jiffies;

	while (atomic_read(&info->tx_len)) {
		current->state = TASK_INTERRUPTIBLE;
		schedule_timeout(1);
		if (signal_pending(current)) 
			break;
		if (timeout && time_after(jiffies, orig_jiffies + timeout))
			break;
	}
	current->state = TASK_RUNNING;
}

/* called after lots of put_char's */
static void
gt_tty_flush_chars(struct tty_struct *tty)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;
	volatile txd_t *txd;
	unsigned long flags;

	save_flags(flags); cli();
	if (info->tx_status[info->tx_current] == TX_FREE) {
		txd = &info->tx_ring[info->tx_current];
		if (txd->bytecnt)
			next_tx(info);
	}
	restart_tx(info);
	restore_flags(flags);
}

static int
gt_tty_write(struct tty_struct *tty, int from_user, const unsigned char *buf, int wlen)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;
	int len = wlen;
	int ret = 0;
	int max;

	if (wlen <= 0)
		return 0;

	if (from_user) {
		down(&tmp_buf_sem);
		do {
			max = min_t(int, PAGE_SIZE, len);

			if (copy_from_user(tmp_buf, buf, max)) {
				if (!ret) 
					ret = -EFAULT;
				break;
			}
			max = gt_sdma_write(info, tmp_buf, max);
			len -= max;
			buf += max;
			ret += max;
		} while ((len > 0) && (max > 0));
		up(&tmp_buf_sem);
	} else {
		ret = gt_sdma_write(info, buf, wlen);
	}
	gt_tty_flush_chars(tty);

	return ret;
}

/* remove pending data */
static void
gt_tty_flush_buffer(struct tty_struct *tty)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;
	unsigned long flags;
	int saved = info->tx_stop;
	int i;

	if(info->line<0 || info->line>NR_PORTS) return;

	/* Wait for current TX to finish */
	info->tx_stop = 1;
	gt_tx_sync(info->line);
	while (info->tx_running){
		schedule_timeout(1);
	}

	save_flags(flags); cli();

	for (i = 0; i < NUM_TX_DESC ; i++) {
		info->tx_ring[i].bytecnt = 0;
		info->tx_ring[i].shadow = 0xfefe;
		info->tx_status[i] = TX_FREE;
	}

	info->tx_active = info->tx_current = 0;
	info->tx_running = 0;
	atomic_set(&info->tx_len, 0);

	restore_flags(flags);

	info->tx_stop = saved;
	wake_up_interruptible(&tty->write_wait);
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP))
	    && tty->ldisc.write_wakeup)
		(tty->ldisc.write_wakeup) (tty);

}

static void
gt_tty_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;
	gt_sdma_write(info, &ch, 1);
}

static int
gt_tty_write_room(struct tty_struct *tty)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;
	int cnt = 0;

	if (info->tx_status[info->tx_current] == TX_FREE) {
		cnt = info->tx_current < info->tx_active ?
			info->tx_active - info->tx_current :
			NUM_TX_DESC - info->tx_current + info->tx_active;

		/* we could be in the process of filling one up, 
		 * don't count it */
		cnt--; 
	}
	return TX_BUF_SIZE * cnt;
}

static int
gt_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;
	return atomic_read(&info->tx_len);
}

static void
gt_tty_send_xchar(struct tty_struct *tty, char ch)
{
	struct mpsc_port *info = &mpsc_ports[0];

	if (tty) info = (struct mpsc_port *)tty->driver_data;

	/* use TCS to send high priority chars */
	gt_write(GT64260_MPSC_0_CHR_1+(info->line*GALMPSC_REG_GAP), ch);
	mb();
	gt_write(GT64260_MPSC_0_CHR_2+(info->line*GALMPSC_REG_GAP), 0x200);
	mb();
	udelay(10000);
}

static void
gt_tty_break(struct tty_struct *tty, int break_state)
{
	/* Nope, sorry */
}

static void
gt_tty_hangup(struct tty_struct *tty)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;

	info->count = 0;
	gt_shutdown(info->line);

	info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CALLOUT_ACTIVE|ASYNC_INITIALIZED);
	info->tty = 0;
}

/* Stop tx */
static void
gt_tty_stop(struct tty_struct *tty)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;
	info->tx_stop = 1;
	galmpsc_freeze_tx(info->line, 1);
}

static void
gt_tty_start(struct tty_struct *tty)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;
	unsigned long flags;

	info->tx_stop = 0;
	galmpsc_freeze_tx(info->line, 0);

	save_flags(flags); cli();
	restart_tx(info);
	restore_flags(flags);
}

/* stop rx */
static void
gt_tty_throttle(struct tty_struct *tty)
{
	if (I_IXOFF(tty))
		gt_tty_send_xchar(tty, STOP_CHAR(tty));
}

/* restart rx */
static void
gt_tty_unthrottle(struct tty_struct *tty)
{
	if (I_IXOFF(tty)) 
		gt_tty_send_xchar(tty, START_CHAR(tty));
}

static void
gt_tty_set_termios(struct tty_struct *tty, struct termios *old_termios)
{
	struct mpsc_port *info = (struct mpsc_port *)tty->driver_data;

	if (tty->termios->c_cflag == info->termios.c_cflag)
		return;

	info->termios.c_cflag = tty->termios->c_cflag;
	gt_set_cflag(info->line, tty->termios->c_cflag);
}

static int
gt_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct mpsc_port *info;
	int retval;
	int line;

	MOD_INC_USE_COUNT;
	line = MINOR(tty->device) - tty->driver.minor_start;
	if (line < 0 || line >= NR_PORTS) {
		MOD_DEC_USE_COUNT;
		return -ENODEV;
	}

	info = &mpsc_ports[line];
	tty->driver_data = info;
	info->tty = tty;
	info->tty->low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;

	/* Setup temp buffer for userspace writes */
	down(&tmp_buf_sem);
	if (!tmp_buf)
		tmp_buf = (unsigned char *) get_zeroed_page(GFP_KERNEL);
	up(&tmp_buf_sem);
	if (!tmp_buf)
		return -ENOMEM;

	/* Start up serial port */
	if (++info->count == 1) {
		retval = gt_startup(info);
		if (retval) {
			printk("gt_tty_open statup <0\n");
			MOD_DEC_USE_COUNT;
			return retval;
		}
	}

#ifdef CONFIG_GT64260_CONSOLE
	if (sercons.cflag && sercons.index == line) {
		tty->termios->c_cflag = sercons.cflag;
		sercons.cflag = 0;
		gt_set_cflag(info->line, tty->termios->c_cflag);
	}
#endif

	info->session = current->session;
	info->pgrp = current->pgrp;

#ifdef SERIAL_DEBUG_OPEN
	printk("gt_tty_open ttys%d successful...", info->line);
#endif

	return 0;
}

static void
gt_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct mpsc_port *info = (struct mpsc_port *) tty->driver_data;
	unsigned long flags;

	if(info->line<0 || info->line>NR_PORTS) return;

	save_flags(flags); 
	cli();
	if (--info->count > 0) {
		restore_flags(flags);
		MOD_DEC_USE_COUNT;
		return;
	}
	info->flags |= ASYNC_CLOSING;
	restore_flags(flags);

	tty->closing = 1;
	gt_tx_sync(info->line);
	gt_shutdown(info->line);

	set_bit(TTY_IO_ERROR, &tty->flags);
	if (tty->driver.flush_buffer)
		tty->driver.flush_buffer(tty);
	if (tty->ldisc.flush_buffer)
		tty->ldisc.flush_buffer(tty);

	tty->closing = 0;
	info->tty = 0;
	info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CALLOUT_ACTIVE|
			           ASYNC_INITIALIZED|ASYNC_CLOSING);

	MOD_DEC_USE_COUNT;
}

static inline int 
line_info(char *buf, struct mpsc_port *info)
{
	struct async_icount *icount = &info->icount;
	int ret;

	ret = sprintf(buf, "port %d:\n", info->line);
	ret += sprintf(buf+ret, "   tx: total:%d running:%d qlen:%d hi:%d\n", 
							icount->tx,
							info->tx_running, 
							atomic_read(&info->tx_len),
							info->tx_hiwater);

	ret += sprintf(buf+ret, "   rx: total:%d fe:%d pe:%d brk:%d oe:%d roe:%d hi:%d\n",
							icount->rx,
							icount->frame,
							icount->parity,
							icount->brk,
							icount->overrun,
							icount->buf_overrun,
							info->rx_hiwater);
	return ret;
}

static int
gt_tty_read_proc(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	off_t begin = 0;
	int len;
	int i;

	len = sprintf(buf, "gt64260_mpsc:%s\n", serial_version);
	for (i = 0; i < NR_PORTS; i++) {
		len += line_info(buf+len, mpsc_ports+i);
		if (len+begin > off+count)
			goto done;
		if (len+begin < off) {
			begin += len;
			len = 0;
		}
	}
	*eof = 1;
done:
	if (off >= len+begin)
		return 0;
	*start = buf + (off-begin);
	return ((count < begin+len-off) ? count : begin+len-off);
}

/**********************************************************
 * Serial Driver Initilization
 */

int __init
gt_mpsc_init(void)
{
	int retval;

#ifndef CONFIG_GT64260_CONSOLE
	setup_once();
#endif

	serial_driver.magic = TTY_DRIVER_MAGIC;
	serial_driver.driver_name = "gt-serial";

	serial_driver.major = GT_MPSC_MAJOR;
	serial_driver.name = GT_MPSC_DEVNAME;
	serial_driver.minor_start = GT_MPSC_MINOR_START;
	serial_driver.name_base = 0;
	serial_driver.num = NR_PORTS;
	serial_driver.type = TTY_DRIVER_TYPE_SERIAL;
	serial_driver.subtype = SERIAL_TYPE_NORMAL;
	serial_driver.init_termios = tty_std_termios;
	serial_driver.init_termios.c_cflag = B38400 | CS8 | CREAD | HUPCL | CLOCAL;
	serial_driver.flags = TTY_DRIVER_REAL_RAW;
	serial_driver.refcount = &gt_refcount;
	serial_driver.table = gt_table;
	serial_driver.termios = gt_termios;
	serial_driver.termios_locked = gt_termios_locked;

	serial_driver.open = gt_tty_open;
	serial_driver.close = gt_tty_close;
	serial_driver.write = gt_tty_write;
	serial_driver.put_char = gt_tty_put_char;
	serial_driver.flush_chars = gt_tty_flush_chars;
	serial_driver.write_room = gt_tty_write_room;
	serial_driver.chars_in_buffer = gt_tty_chars_in_buffer;
	serial_driver.flush_buffer = gt_tty_flush_buffer;
	serial_driver.ioctl = gt_tty_ioctl;
	serial_driver.throttle = gt_tty_throttle;
	serial_driver.unthrottle = gt_tty_unthrottle;
	serial_driver.set_termios = gt_tty_set_termios;
	serial_driver.stop = gt_tty_stop;
	serial_driver.start = gt_tty_start;
	serial_driver.hangup = gt_tty_hangup;
	serial_driver.break_ctl = gt_tty_break;
	serial_driver.send_xchar = gt_tty_send_xchar;
	serial_driver.wait_until_sent = gt_tty_wait_until_sent;
	serial_driver.read_proc = gt_tty_read_proc;

	if (tty_register_driver(&serial_driver))
		panic("Couldn't register GT64260 MPSC serial driver\n");

	/* Setup IRQ handler */
	galsdma_intr_ack();
	galsdma_intr_mask(0, 0xf);
	galsdma_intr_mask(1, 0xf);

	retval = request_irq(SDMA_IRQ, gt_sdma_interrupt, 0, "GT64260 Serial DMA", 0);
	if (retval){
		printk(KERN_ERR "Could't get GT64260 SDMA IRQ");
		if ( ppc_md.progress ) ppc_md.progress("Could't get GT64260 SDMA IRQ", 0x0);	
	}

	if ( ppc_md.progress ) ppc_md.progress("gt_mpsc_init: exit", 0x0);
	return 0;
}

static void __exit
gt_mpsc_cleanup(void)
{
	unsigned long flags;
	int ret;

	save_flags(flags);
	cli();

	ret = tty_unregister_driver(&serial_driver);
	if (ret)
		printk(KERN_ERR "Unable to unregister GT64260 MPSC driver (%d)\n", ret);

	if (tmp_buf)
		free_page((u32) tmp_buf);

	free_irq(SDMA_IRQ, NULL);
	restore_flags(flags);
}

module_init(gt_mpsc_init);
module_exit(gt_mpsc_cleanup);

MODULE_DESCRIPTION("GT64260 MPSC UART driver");
MODULE_AUTHOR("Rex Feany <rfeany@zumanetworks.com>");
MODULE_LICENSE("GPL");

/***************************************************************************
 * Serial console support 
 */

#ifdef CONFIG_GT64260_CONSOLE

/* Buffer descriptors for serial console */
static char sercon_tx_buffer[PAGE_SIZE] __attribute__ ((aligned (PAGE_SIZE)));
static volatile txd_t sercon_txd;

static int __init
gt_console_setup(struct console *co, char *options)
{
	int baud = 38400;
	int bits = 8;
	int parity = 'n';
	int cflag = CREAD | HUPCL | CLOCAL;
	int bidx;

#ifdef	CONFIG_USE_PPCBOOT
	if (ppcboot_bd_valid)
		baud = ppcboot_bd.bi_baudrate;
#endif /* CONFIG_USE_PPCBOOT */

	if ( ppc_md.progress ) ppc_md.progress("gt_console_setup: enter", 0x0);

	/* Parse the 'options' string and configure console accordingly.  */
	if (options) {
		char *s;

		baud = simple_strtoul(options, NULL, 10);
		s = options;
		while (*s >= '0' && *s <= '9')
			s++;
		if (*s)
			parity = *s++;
		if (*s)
			bits = *s - '0';
	}

	/*
	 * construct cflags
	 */

	for (bidx = 0; bidx < (sizeof(baud_table) / sizeof(int)); bidx++)
		if (baud == baud_table[bidx])
			break;

	/* make sure we have a useful value */
	if (bidx == (sizeof(baud_table) / sizeof(int)))
		bidx = 13;	/* B9600 */
	cflag |= bidx;

	switch (bits) {
		case 7:
			cflag |= CS7;
			break;
		default:
			cflag |= CS8;
			break;
	}
	switch (parity) {
		case 'o':
		case 'O':
			cflag |= PARODD;
			break;
		case 'e':
		case 'E':
			cflag |= PARENB;
			break;
	}
	co->cflag = cflag;

	ll_mpsc_init(co->index);
	gt_set_cflag(co->index, cflag);

	co->flags |= CON_ENABLED;

	if ( ppc_md.progress ) ppc_md.progress("gt_console_setup: exit", 0x0);
	return 0;
}

/* copy_convert - newline to cr/lf conversion and copy */
static ssize_t
copy_convert(char *dst, ssize_t dlen, const char *src, ssize_t *pslen)
{
	int copied = 0;
	int slen = *pslen;

	while(dlen && slen) {
		dlen--; slen--; copied++;
		if ((*dst++ = *src++) == '\n') {
			if (dlen) {
				*dst++ = '\r';
				--dlen;
				++copied;
			} else {
				*pslen = slen;
				return --copied;
			}
		}
	}
	*pslen = slen;
	return copied;
}

static void
gt_console_write(struct console *co, const char *s, unsigned count)
{
	unsigned int mask;

	if(!count) return;

	if(co->index<0 || co->index>NR_PORTS) return;

	mask=galsdma_intr_mask(co->index, 0xf);

	gt_tx_sync(co->index);
	
	do {
		int did = copy_convert(sercon_tx_buffer, PAGE_SIZE, s, &count);
		s+=did;

		sercon_txd.cmd_sts = STS_LAST | STS_FIRST | STS_OWN;
		sercon_txd.shadow = did;
		sercon_txd.bytecnt = did;
		sercon_txd.buf_ptr = virt_to_phys(sercon_tx_buffer);
		sercon_txd.next_desc_ptr = 0;

		clean_dcache_range((u32)sercon_tx_buffer, (u32)(sercon_tx_buffer+did));
		clean_dcache_range((u32)&sercon_txd, (u32)(&sercon_txd+1));

		galsdma_set_tx_ring(co->index, virt_to_phys(&sercon_txd));
		galsdma_request(co->index, SDMA_STOP_TX|SDMA_DEMAND_TX);

		gt_tx_sync(co->index);
	} while (count > 0);

	galsdma_intr_unmask(co->index, mask);
}

#if 0
static int
gt_console_wait_key(struct console *co)
{
	return 0;
}
#endif

static void
gt_console_unblank(void)
{
}

static kdev_t
gt_console_device(struct console *co)
{
	return MKDEV(TTY_MAJOR, co->index + 64);
}

static struct console sercons = {
	name:		"ttyS",
	write:		gt_console_write,
	device:		gt_console_device,
/*	wait_key:	gt_console_wait_key, */
	unblank:	gt_console_unblank,
	setup:		gt_console_setup,
	flags:		CON_PRINTBUFFER,
	index:		CONFIG_SERIAL_CONSOLE_PORT,
};

int
gt64260_mpsc_console_init(void)
{
	if ( ppc_md.progress ) ppc_md.progress("gt64260_mpsc_console_init: enter", 0x0);
	setup_once();
	register_console(&sercons);
	if ( ppc_md.progress ) ppc_md.progress("gt64260_mpsc_console_init: exit", 0x0);

	return 0;
}
#endif /* CONFIG_GT64260_CONSOLE */


