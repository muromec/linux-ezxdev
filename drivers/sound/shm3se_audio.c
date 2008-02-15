/*
 * drivers/sound/shm3se_audio.c
 *
 * SH73180 SIU, SIOF and Audio CODEC(AK2440) sound device driver for Super-H
 *
 * Author: Takashi SHUDO
 *
 * 2004 (c) Takashi SHUDO. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/*
  Devices

  /dev/dsp	: SIOF (sampling rate 8KHz only)
  /dev/dsp1	: SIU (write(play) only)

  DMAC
	  Ch 2	: SIOF rx
	  Ch 3	: SIOF tx
	  Ch 4	: SIU tx
  
  Audio CODEC	: AK2440 (I2C control)
*/

/* ioctl SNDCTL_DSP_STEREO */
#define CHANNEL_MONO   0
#define CHANNEL_STEREO 1

#define I2C_WRITE 0xfffe	/* ioctl I2C data write */

/*
 * Configration
 */
#define DEFAULT_SIOF_CHANNEL	CHANNEL_MONO	/* SIOF(/dev/dsp)
						 * default monaural */
#define DEFAULT_SIU_RATE	44100	/* SIU(/dev/dsp1) default
					 * sampling rate 44.1KHz */
#define DEFAULT_SIU_CHANNEL	CHANNEL_STEREO	/* SIU(/dev/dsp1)
						 * default monaural */
#define INITPIN			/* if need PIN function initialize,
				 * define it. */
/*#define I2CRELEASE*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/ioport.h>

#include <linux/slab.h>

#include <linux/sound.h>
#include <linux/soundcard.h>

#include <linux/pm.h>
#include <linux/device.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/delay.h>

#include <asm/irq-sh73180.h>
#include <asm/ioctls.h>
#include <asm/sh_mobile3_pm.h>

#ifdef DEBUG
#  define DPRINTK(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

/*
 * DMAC Buffer size define
 */
#define SIOF_DMABUF_NUM	160	/* 20ms(8KHz) */
#define SIOF_DMABUF_SIZE	(sizeof(int) * SIOF_DMABUF_NUM)

#define SIU_DMABUF_NUM	2048
#define SIU_DMABUF_SIZE		(sizeof(int) * SIU_DMABUF_NUM)

#define MAX_DMA_BANK	8	/* number of DMAC buffer */

/* PFC and etc Registers */
#define	PCCR	(0xa4050104)	/* SIU0OSPD */
#define	PFCR	(0xa405010a)	/* SIU0OBT,SIU0OLR,SIU0ISLD,SIU0ISPD */
#define	PHCR	(0xa405010e)	/* SIU0OSLD */
#define	PKCR	(0xa4050112)
#define SCPCR	(0xa4050116)
#define	PRCR	(0xa4050160)
#define	PQCR	(0xa405011c)
#define	PSELA	(0xa4050140)
#define	HIZCRA	(0xa4050146)
#define	DRVCR	(0xa405014a)
#define	PSELD	(0xa40501a0)
#define	PSELE	(0xa4050158)

#define	MSTPCR0	(0xa4150030)
#define	MSTPCR2	(0xa4150038)

/****************************************************************************
 * I2C
 ****************************************************************************/
/* I2C Registers */
#define I2C_REG_BASE	0xa4470000
#define ICDR	(I2C_REG_BASE+0x0000)
#define ICCR	(I2C_REG_BASE+0x0004)
#define ICSR	(I2C_REG_BASE+0x0008)
#define ICIC	(I2C_REG_BASE+0x000c)
#define ICCL	(I2C_REG_BASE+0x0010)
#define ICCH	(I2C_REG_BASE+0x0014)

#define MAX_IIC_BUF	20

#define ICCL_VAL	0x96
#define ICCH_VAL	0x78

static unsigned char iic_data[MAX_IIC_BUF];
static unsigned char iic_size = 0;
static unsigned char iic_num = 0;
static unsigned char iic_end = 0;

static DECLARE_WAIT_QUEUE_HEAD(iicwq);

static inline void
i2c_restart(void)
{
	ctrl_outb(0x00, ICCR);	/* Stop */

	ctrl_outb(ICCL_VAL, ICCL);
	ctrl_outb(ICCH_VAL, ICCH);

	ctrl_outb(0x0c, ICIC);	/* ALE, TACKE Interrupt enable */
}

/* I2C interrupt handler */

/* Arbitration lost */
static void
i2c_ali_int_hdr(int irq, void *dev_id, struct pt_regs *regs)
{
	DPRINTK("ALI int\n");
}

/* TACK */
static void
i2c_tacki_int_hdr(int irq, void *dev_id, struct pt_regs *regs)
{
	DPRINTK("TACKI int\n");

	ctrl_outb(ctrl_inb(ICSR) & ~0x04, ICSR);

	i2c_restart();

	ctrl_outb(ctrl_inb(ICIC) | 0x01, ICIC);	/* data transmit
						 * enable interrupt
						 * enable */

	printk("I2C TACK occuer.\n");
	iic_end = 1;
	wake_up(&iicwq);
}

/* Wait */
static void
i2c_waiti_int_hdr(int irq, void *dev_id, struct pt_regs *regs)
{
	DPRINTK("WAITI int\n");
}

/* Data transmit enable */
static void
i2c_dtei_int_hdr(int irq, void *dev_id, struct pt_regs *regs)
{
	DPRINTK("DTE int\n");

	if ((iic_num + 1) < iic_size) {
		ctrl_outb(iic_data[iic_num], ICDR);

		DPRINTK("write = %02x\n", iic_data[iic_num]);

		iic_num++;
	} else {
		ctrl_outb(iic_data[iic_num], ICDR);
		ctrl_outb(0x90, ICCR);
		ctrl_outb(ctrl_inb(ICIC) & ~0x01, ICIC);

		DPRINTK("write last = %02x\n", iic_data[iic_num]);
		iic_end = 1;
		iic_num++;
		wake_up(&iicwq);
	}
#if 0
	iic_end = 1;
	wake_up(&iicwq);
#endif
}

static inline int
i2c_init(void)
{
	DPRINTK("init\n");

	ctrl_outw(ctrl_inw(SCPCR) & ~0x0f00, SCPCR);    /* PFC */

	ctrl_outb(ctrl_inb(ICCR) & ~0x80, ICCR);	/* Stop I2C */

	if (request_irq(IIC0_ALI_IRQ, i2c_ali_int_hdr, SA_INTERRUPT,
			"sh_i2c", NULL)) {
		printk(KERN_ERR "Cannot allocate IIC_ALI_IRQ\n");
		return -ENOMEM;
	}

	if (request_irq(IIC0_TACKI_IRQ, i2c_tacki_int_hdr, SA_INTERRUPT,
			"sh_i2c", NULL)) {
		printk(KERN_ERR "Cannot allocate IIC_TACKI_IRQ\n");
		free_irq(IIC0_ALI_IRQ, NULL);
		return -ENOMEM;
	}

	if (request_irq(IIC0_WAITI_IRQ, i2c_waiti_int_hdr, SA_INTERRUPT,
			"sh_i2c", NULL)) {
		printk(KERN_ERR "Cannot allocate IIC_WAITI_IRQ\n");
		free_irq(IIC0_ALI_IRQ, NULL);
		free_irq(IIC0_TACKI_IRQ, NULL);
		return -ENOMEM;
	}

	if (request_irq(IIC0_DTEI_IRQ, i2c_dtei_int_hdr, SA_INTERRUPT,
			"sh_i2c", NULL)) {
		printk(KERN_ERR "Cannot allocate IIC_DTEI_IRQ\n");
		free_irq(IIC0_ALI_IRQ, NULL);
		free_irq(IIC0_TACKI_IRQ, NULL);
		free_irq(IIC0_WAITI_IRQ, NULL);
		return -ENOMEM;
	}

	return 0;
}

static inline void
i2c_open(void)
{
	DPRINTK("open\n");

	/* Start */

	ctrl_outl(ctrl_inl(MSTPCR0) & ~0x00400003, MSTPCR0);	/* INTC,
								 * IIC1,
								 * IIC2
								 * Start */

	ctrl_outb(ctrl_inb(ICCR) | 0x80, ICCR);	/* Start I2C */

	ctrl_outb(0x00, ICIC);	/* Interrupt disable */

	/* Clock setting */
	ctrl_outb(ICCL_VAL, ICCL);
	ctrl_outb(ICCH_VAL, ICCH);

	/* Open */

	ctrl_outb(0x0c, ICIC);	/* ALE, TACKE Interrupt enable */
}

static inline void
i2c_exit(void)
{
	DPRINTK("release\n");

	ctrl_outb(0x00, ICIC);	/* Disable all I2C interrupt */

	free_irq(IIC0_ALI_IRQ, NULL);
	free_irq(IIC0_TACKI_IRQ, NULL);
	free_irq(IIC0_WAITI_IRQ, NULL);
	free_irq(IIC0_DTEI_IRQ, NULL);
}

static inline void
i2c_write(unsigned char *d, int size)
{
	int i;

	DPRINTK("size=%d\n", size);

	for (i = 0; i < size; i++) {
		iic_data[i] = d[i];
	}
	iic_num = 0;
	iic_size = size;

	iic_end = 0;

	i2c_restart();

	ctrl_outb(ctrl_inb(ICIC) | 0x01, ICIC);	/* data transmit
						 * enable interrupt
						 * enable */
	ctrl_outb(0x94, ICCR);	/* start condition */

	wait_event_interruptible(iicwq, iic_end != 0);

	DPRINTK("end\n");
}

/*
 * DMA buffer
 */
typedef struct {
	unsigned int *p_buff[MAX_DMA_BANK];
	int size[MAX_DMA_BANK];
	int wp;
	int rp;
} st_dmabuff;

static inline void
init_dmabuff(st_dmabuff * buff)
{
	buff->wp = 0;
	buff->rp = 0;
}

/* call before buffer read */
static inline unsigned int *
get_data_dmabank(st_dmabuff * buff, int *size)
{
	DPRINTK("R:%d\n", buff->rp);

	if (buff->rp != buff->wp) {
		*size = buff->size[buff->rp];
		return buff->p_buff[buff->rp];
	} else {
		return 0;	/* buffer empty */
	}
}

/* call after buffer read */
static inline void
free_dmabank(st_dmabuff * buff)
{
	int n, p;

	if (buff->rp != buff->wp) {
		p = buff->rp;
		n = buff->rp + 1;
		if (n >= MAX_DMA_BANK) {
			n = 0;
		}
		buff->rp = n;

		DPRINTK("F:%d\n", p);
	}
}

/* call before buffer write */
static inline unsigned int *
get_free_dmabank(st_dmabuff * buff, int size)
{
	int n = buff->wp + 1;
	if (n >= MAX_DMA_BANK) {
		n = 0;
	}

	if (n != buff->rp) {
		DPRINTK("G:%d\n", buff->wp);

		buff->size[buff->wp] = size;
		return buff->p_buff[buff->wp];
	} else {
		return 0;	/* buffer full */
	}
}

/* call after buffer write */
static inline void
write_dmabank(st_dmabuff * buff)
{
	int n = buff->wp + 1;
#ifdef DEBUG
	int p = buff->wp;
#endif
	if (n >= MAX_DMA_BANK) {
		n = 0;
	}

	if (n != buff->rp) {
		buff->wp = n;

		DPRINTK("W:%d\n", p);
	}
}

/* call after buffer write */
static inline void
write_size_dmabank(st_dmabuff * buff, int size)
{
	int n = buff->wp + 1;
#ifdef DEBUG
	int p = buff->wp;
#endif
	if (n >= MAX_DMA_BANK) {
		n = 0;
	}

	if (n != buff->rp) {
		buff->size[buff->wp] = size;
		buff->wp = n;

		DPRINTK("W:%d\n", p);
	}
}

/****************************************************************************
 * Driver work
 ****************************************************************************/

struct sh_siof_card {

	wait_queue_head_t wq;
	wait_queue_head_t rq;
	wait_queue_head_t syncq;

	int dev_audio;		/* soundcore stuff */

	unsigned long siof_iobase;

	unsigned int channel;

	st_dmabuff tx_buff;
	st_dmabuff rx_buff;

	unsigned int w_openCnt;
	unsigned int r_openCnt;
};

static struct sh_siof_card *siof_devs = NULL;

struct sh_siu_card {

	wait_queue_head_t wq;
	wait_queue_head_t syncq;

	int dev_audio;		/* soundcore stuff */

	unsigned long siu_iobase;

	unsigned int smp_rate;
	unsigned int channel;

	st_dmabuff tx_buff;

	unsigned int w_openCnt;
};

static struct sh_siu_card *siu_devs = NULL;

/****************************************************************************
 * DMA function
 ****************************************************************************/
/* DMA registers */
#define DMAC_BASE	(0xfe008020)
#define DMAC2_BASE	(DMAC_BASE+0x20)
#define DMAC3_BASE	(DMAC_BASE+0x30)
#define DMAC4_BASE	(DMAC_BASE+0x50)
#define SAR_OFF		0x00
#define DAR_OFF		0x04
#define DMATCR_OFF	0x08
#define CHCR_OFF	0x0c

#define DMAOR	0xfe008060
#define DMARS0	0xfe009000
#define DMARS1	0xfe009004
#define DMARS2	0xfe009008

/* SIOF rx */
#define SAR2	(DMAC2_BASE+SAR_OFF)
#define DAR2	(DMAC2_BASE+DAR_OFF)
#define DMATCR2	(DMAC2_BASE+DMATCR_OFF)
#define CHCR2   (DMAC2_BASE+CHCR_OFF)

/* SIOF tx */
#define SAR3	(DMAC3_BASE+SAR_OFF)
#define DAR3	(DMAC3_BASE+DAR_OFF)
#define	DMATCR3	(DMAC3_BASE+DMATCR_OFF)
#define CHCR3	(DMAC3_BASE+CHCR_OFF)

/* SIU tx */
#define SAR4	(DMAC4_BASE+SAR_OFF)
#define DAR4	(DMAC4_BASE+DAR_OFF)
#define	DMATCR4	(DMAC4_BASE+DMATCR_OFF)
#define CHCR4	(DMAC4_BASE+CHCR_OFF)

#define CHCR_DE	(1<<0)
#define CHCR_TE	(1<<1)

#define DMAAMASK	0x1fffffff

/*
 * DMA2 (siof_read)
 */
static int rdma_end;
static int read_cnt;
static int dma_read_cnt;

static inline void
rx_start_dma2(void *buf, unsigned int size)
{
	rdma_end = 0;

	ctrl_outl((ctrl_inl(CHCR2) & ~CHCR_DE), CHCR2);
	ctrl_outl((unsigned int) buf & DMAAMASK, DAR2);
	ctrl_outl(size, DMATCR2);
	ctrl_outl((ctrl_inl(CHCR2) & ~CHCR_TE), CHCR2);
	ctrl_outl((ctrl_inl(CHCR2) | CHCR_DE), CHCR2);

	DPRINTK("rx(Ch2) add=%08x size=%d DMA start\n", (int) buf, size);
}

static inline void
rx_stop_dma2(void)
{
	DPRINTK("DMA rx(Ch2) stop\n");

	ctrl_outl(ctrl_inl(CHCR2) & ~(CHCR_DE | CHCR_TE), CHCR2);
}

static void
dma2_receive_end_intr(int irq, void *dev_id, struct pt_regs *regs)
{
	struct sh_siof_card *card = siof_devs;
	unsigned int *p;
	int size;

	ctrl_outl(((ctrl_inl(CHCR2) & ~CHCR_DE) & ~CHCR_TE), CHCR2);
	/* DMA stop , disable DMATCR interrupt */

	write_dmabank(&card->rx_buff);

	if (dma_read_cnt < SIOF_DMABUF_SIZE) {
		size = dma_read_cnt;
	} else {
		size = SIOF_DMABUF_SIZE;
	}

	if (size > 0) {
		p = get_free_dmabank(&card->rx_buff, size);

		if (p != 0) {
			dma_read_cnt -= size;
			rx_start_dma2(p, size / sizeof (int));
		} else {
			DPRINTK("read buffer empty.\n");
		}
	}

	DPRINTK("wakeup.\n");

	rdma_end = 1;

	wake_up(&card->rq);
}

/*
 * DMA3 (siof_write)
 */
static inline void
tx_start_dma3(void *buf, unsigned int size)
{
	ctrl_outl((ctrl_inl(CHCR3) & ~CHCR_DE), CHCR3);
	ctrl_outl((unsigned int) buf & DMAAMASK, SAR3);
	ctrl_outl(size, DMATCR3);
	ctrl_outl((ctrl_inl(CHCR3) & ~CHCR_TE), CHCR3);
	ctrl_outl((ctrl_inl(CHCR3) | CHCR_DE), CHCR3);

	DPRINTK("tx(Ch3) add=%08x size=%d DMA start\n", (int) buf, size);
}

static inline void
tx_stop_dma3(void)
{
	DPRINTK("DMA tx(Ch3) stop\n");

	ctrl_outl(ctrl_inl(CHCR3) & ~(CHCR_DE | CHCR_TE), CHCR3);
}

static void
dma3_transfer_end_intr(int irq, void *dev_id, struct pt_regs *regs)
{
	struct sh_siof_card *card = siof_devs;
	int size;

	ctrl_outl(((ctrl_inl(CHCR3) & ~CHCR_DE) & ~CHCR_TE), CHCR3);
	/* DMA stop , disable DMATCR interrupt */

	free_dmabank(&card->tx_buff);

	unsigned int *p = get_data_dmabank(&card->tx_buff, &size);

	if (p == 0) {
		wake_up_interruptible(&card->syncq);
	} else {
		tx_start_dma3(p, size);
		DPRINTK("wakeup.\n");
		wake_up_interruptible(&card->wq);
	}

	wake_up_interruptible(&card->wq);
	DPRINTK("wake_up w\n");
}

/*
 * DMA4 (siu_write)
 */
static inline void
tx_start_dma4(void *buf, unsigned int size)
{
	ctrl_outl((ctrl_inl(CHCR4) & ~CHCR_DE), CHCR4);
	ctrl_outl((unsigned int) buf & DMAAMASK, SAR4);
	ctrl_outl(size, DMATCR4);
	ctrl_outl((ctrl_inl(CHCR4) & ~CHCR_TE), CHCR4);
	ctrl_outl((ctrl_inl(CHCR4) | CHCR_DE), CHCR4);

	DPRINTK("tx(Ch4) add=%08x size=%d DMA start\n", (int) buf, size);
}

static inline void
tx_stop_dma4(void)
{
	DPRINTK("DMA tx(Ch4) stop\n");

	ctrl_outl(ctrl_inl(CHCR4) & ~(CHCR_DE | CHCR_TE), CHCR4);
}

static void
dma4_transfer_end_intr(int irq, void *dev_id, struct pt_regs *regs)
{
	struct sh_siu_card *card = siu_devs;
	int size;

	ctrl_outl(((ctrl_inl(CHCR4) & ~CHCR_DE) & ~CHCR_TE), CHCR4);
	/* DMA stop , disable DMATCR interrupt */

	free_dmabank(&card->tx_buff);

	unsigned int *p = get_data_dmabank(&card->tx_buff, &size);

	if (p == 0) {
		wake_up_interruptible(&card->syncq);
	} else {
		tx_start_dma4(p, size);
		DPRINTK("wakeup.\n");
		wake_up_interruptible(&card->wq);
	}
}

/****************************************************************************
 * CODEC (AK2440)
 ****************************************************************************/
#define AUDIOCODEC_RES	(0xb1600000)
#define WCOMMAND	0x9e

#define AK_CLKCNT	0x50	/* MSTCLK_I=13MHz, PLL1(Master mode) */
				/* (Address:18h) */
#define SMPC_8000	(AK_CLKCNT | 0x00)	/* 8 KHz        */
#define SMPC_11025	(AK_CLKCNT | 0x01)	/* 11.025 KHz   */
#define SMPC_12000	(AK_CLKCNT | 0x02)	/* 12 KHz       */
#define SMPC_16000	(AK_CLKCNT | 0x03)	/* 16 KHz       */
#define SMPC_22050	(AK_CLKCNT | 0x04)	/* 22.05 KHz    */
#define SMPC_24000	(AK_CLKCNT | 0x05)	/* 24 KHz       */
#define SMPC_32000	(AK_CLKCNT | 0x06)	/* 32 KHz       */
#define SMPC_44100	(AK_CLKCNT | 0x07)	/* 44.1 KHz     */
#define SMPC_48000	(AK_CLKCNT | 0x08)	/* 48 KHz       */

static unsigned char ak_reset[] = {
	WCOMMAND, 0x01, 0xff
};

static unsigned char ak_06[] = {
	WCOMMAND, 0x06, 0x30, 0x24, 0x28
};

static unsigned char ak_0b[] = {
	WCOMMAND, 0x0b, 0x13, 0x00, 0x05
};

static unsigned char ak_15[] = {
	WCOMMAND, 0x15,
	0x00,			/* DAC Volume 0dB */
	0x0f, 0x0f,
	SMPC_44100,		/* 44.1KHz(Clock:13MHz) */
	0x04, 0x00, 0x0e, 0x00, 0x00, 0x01, 0x1c, 0x10, 0x10, 0x1c
};

static unsigned char ak_03[] = {
	WCOMMAND, 0x03, 0x0d, 0x35, 0x33
};

static unsigned char ak_10[] = {
	WCOMMAND, 0x10,
	0x00,			/* VR3(L) Volume */
	0x00			/* VR4(R) Volume */
};

static inline void
ak2440_init(void)
{
	DPRINTK("AK2440 initialize.\n");

	ctrl_outw(ctrl_inw(AUDIOCODEC_RES) | 0x0100, AUDIOCODEC_RES);

	i2c_open();

	i2c_write(ak_reset, sizeof (ak_reset));
	mdelay(1);
	i2c_write(ak_06, sizeof (ak_06));
	mdelay(1);
	i2c_write(ak_0b, sizeof (ak_0b));
	mdelay(1);
	i2c_write(ak_15, sizeof (ak_15));
	mdelay(1);
	i2c_write(ak_03, sizeof (ak_03));
	mdelay(1);
	i2c_write(ak_10, sizeof (ak_10));
	mdelay(1);

#ifdef CODECLOOPBACK
	{
		/* for DEBUG */
		static unsigned char ak_lp0[] = {
			WCOMMAND, 0x0a, 0x06, 0x13,
		};

		static unsigned char ak_lp1[] = {
			WCOMMAND, 0x05, 0x1f,
		};

		i2c_write(ak_lp0, sizeof (ak_lp0));	/* for DEBUG */
		mdelay(1);
		i2c_write(ak_lp1, sizeof (ak_lp1));	/* for DEBUG */
		mdelay(1);
	}
#endif
}

static inline void
ak2440_exit(void)
{
	DPRINTK("AK2440 stop.\n");

	ctrl_outw(ctrl_inw(AUDIOCODEC_RES) & ~0x0100, AUDIOCODEC_RES);
}

/*****************************************************************************
 * SIU
 ****************************************************************************/
/* SIU registers */
#define SCLKACR		0xa4150008

#define SIU_REG_BASE	0xa454c000
#define SIU_P_RAM	0xa4540000	/* PRAM         */
#define SIU_X_RAM	0xa4544000	/* XRAM         */
#define SIU_Y_RAM	0xa4546000	/* YRAM         */
#define SIU_FIFO_RAM	0xa4548000	/* FIFO RAM     */

#define IFCTL	(SIU_REG_BASE+0x0000)
#define SRCTL	(SIU_REG_BASE+0x0004)
#define SFORM	(SIU_REG_BASE+0x0008)
#define CKCTL	(SIU_REG_BASE+0x000c)
#define TRDAT	(SIU_REG_BASE+0x0010)
#define STFIFO	(SIU_REG_BASE+0x0014)
#define DPAK	(SIU_REG_BASE+0x001c)
#define CKREV	(SIU_REG_BASE+0x0020)
#define EVNTC	(SIU_REG_BASE+0x0028)
#define SBCTL	(SIU_REG_BASE+0x0040)
#define SBPSET	(SIU_REG_BASE+0x0044)
#define SBBUS	(SIU_REG_BASE+0x0048)
#define SBACTIV	(SIU_REG_BASE+0x0074)
#define DMAIA	(SIU_REG_BASE+0x0090)
#define DMAIB	(SIU_REG_BASE+0x0094)
#define DMAOA	(SIU_REG_BASE+0x0098)
#define DMAOB	(SIU_REG_BASE+0x009c)
#define DMAML	(SIU_REG_BASE+0x00a0)
#define BRGASEL	(SIU_REG_BASE+0x0100)
#define BRRA	(SIU_REG_BASE+0x0104)
#define BRGBSEL	(SIU_REG_BASE+0x0108)
#define BRRB	(SIU_REG_BASE+0x010C)

#define SIU_REG_SIZE	0x100

/* PRAM program array size */
#define PRAM0SIZE (0x0100/4)	/* =64  */
#define PRAM1SIZE ((0x2000-0x0100)/4)	/* =1984        */
#define XRAM0SIZE (0x0200/4)	/* =128 */
#define XRAM1SIZE (0x0100/4)	/* =64  */
#define XRAM2SIZE (0x0100/4)	/* =64  */
#define YRAM0SIZE (0x0040/4)	/* =16  */
#define YRAM1SIZE (0x0080/4)	/* =32  */
#define YRAM2SIZE (0x0040/4)	/* =16  */
#define YRAM3SIZE (0x0080/4)	/* =32  */
#define YRAM4SIZE (0x0080/4)	/* =32  */
#define YRAMFIRSIZE (0x0400/4)	/* =256 */
#define YRAMIIRSIZE (0x0200/4)	/* =128 */

/* PRAM offset=0x0000 */
static unsigned long siu_spb_pro_p0[PRAM0SIZE] = {
	0x40001a00,		//           ; MOVIH #H'0000, SSIZE
	0x62281a00,		//           ; MOVIL #H'1140, SSIZE
	0x40017800,		//           ; MOVIH #I010, LOOPADR
	0x60019800,		//           ; MOVIL #I020, LOOPADR
	0x41001d00,		//           ; MOVIH #H'0800, DVCTLA
	0x61001d00,		//           ; MOVIL #H'0800, DVCTLA
	0x41001e00,		//           ; MOVIH #H'0800, DVCTLB
	0x61001e00,		//           ; MOVIL #H'0800, DVCTLB
	0x0f7c0500,		//           ; MOVH.L        ZERO, #0, R5
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0d7c0500,		//           ; MOVH.L        ZERO, #0, @R5+
	0x1f800000,		//           ; HLT   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
};

/* PRAM offset=0x0100 */
static long siu_spb_pro_p1[PRAM1SIZE] = {
	0x66000600,		//           ; MOVIL #DA000, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a180800,		//           ; MOVH.L        @R6+, #0, X0
	0x0a180900,		//           ; MOVH.L        @R6+, #0, X1
	0x09181000,		//           ; MOVH.L        @R6, #0, D
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x66014600,		//           ; MOVIL #DA00A, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09180300,		//           ; MOVH.L        @R6, #0, R3
	0x09180000,		//           ; MOVH.L        @R6, #0, R0
	0x0fd40100,		//           ; MOVL.L        IOSTS, #0, R1
	0x14802000,		//           ; AND   R0, R1, R0
	0x2010bc00,		//           ; JMPT  #L037, #28
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0fd41b00,		//           ; MOVL.L        IOSTS, #0, HOSTR
	0x0f0c1900,		//           ; MOVH.L        R3, #0, FLAG
	0x2410a000,		//           ; JMPF  #L037, #0
	0x0f780c00,		//           ; MOVH.L        DVCTLB, #0, Y0
	0x0fb20d00,		//           ; MOVL.L        Y0, #16, Y1
	0x60000c00,		//           ; MOVIL #H'0000, Y0
	0x4060ba00,		//           ; MOVIH #H'0305, SSIZE
	0x78001a00,		//           ; MOVIL #H'C000, SSIZE
	0x0f0d1200,		//           ; MOVH.L        R3, #8, IOADR
	0x1748a000,		//           ; SET   IOADR, #5
	0x20102900,		//           ; JMPT  #L036, #9
	0x16c8a000,		//           ; RST   IOADR, #5
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0f0e1200,		//           ; MOVH.L        R3, #16, IOADR
	0x1748a000,		//           ; SET   IOADR, #5
	0x20102500,		//           ; JMPT  #L036, #5
	0x16c8a000,		//           ; RST   IOADR, #5
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0f4c0a00,		//           ; MOVH.L        IDAT, #0, A0
	0x0f0d1200,		//           ; MOVH.L        R3, #8, IOADR
	0x0faa0b00,		//           ; MOVL.L        A0, #16, A1
	0x60000a00,		//           ; MOVIL #H'0000, A0
	0x08280b00,		//           ; SWAP  A0, A1
	0xeb000000,		//           ; 
	0x08280b00,		//           ; SWAP  A0, A1
	0x0ffc0a00,		//           ; MOVL.L        ZERO, #0, A0
	0xe4148000,		//           ; 
	0xe4f08000,		//           ; 
	0xe5300000,		//           ; 
	0x200f0200,		//           ; JMPT  #L035, #2
	0xeb400000,		//           ; 
	0x0ffc0b00,		//           ; MOVL.L        ZERO, #0, A1
	0x00000000,		//           ; NOP   
	0xe41cc000,		//           ; 
	0xe4f8c000,		//           ; 
	0xe5380000,		//           ; 
	0x0fa80000,		//           ; MOVL.L        A0, #0, R0
	0x5fffe100,		//           ; MOVIH #H'FFFF, R1
	0x60000100,		//           ; MOVIL #H'0000, R1
	0x14802000,		//           ; AND   R0, R1, R0
	0x0f2e0200,		//           ; MOVH.L        A1, #16, R2
	0x200b5e00,		//           ; JMPT  #L030, #30
	0x40000200,		//           ; MOVIH #H'0000, R2
	0x15004000,		//           ; OR    R0, R2, R0
	0x0f001400,		//           ; MOVH.L        R0, #0, ODAT
	0x0f0d1200,		//           ; MOVH.L        R3, #8, IOADR
	0x17488000,		//           ; SET   IOADR, #4
	0x0f0e1200,		//           ; MOVH.L        R3, #16, IOADR
	0x17488000,		//           ; SET   IOADR, #4
	0x0f201900,		//           ; MOVH.L        X0, #0, FLAG
	0x20122700,		//           ; JMPT  #L038, #7
	0x6601c600,		//           ; MOVIL #DA00E, R6
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09180000,		//           ; MOVH.L        @R6, #0, R0
	0x0fd40100,		//           ; MOVL.L        IOSTS, #0, R1
	0x14802000,		//           ; AND   R0, R1, R0
	0x205d9c00,		//           ; JMPT  #L500, #28
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0f201900,		//           ; MOVH.L        X0, #0, FLAG
	0x245d8000,		//           ; JMPF  #L500, #0
	0x66010600,		//           ; MOVIL #DA008, R6
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x09181900,		//           ; MOVH.L        @R6, #0, FLAG
	0x2014a000,		//           ; JMPT  #L040, #0
	0x0f211200,		//           ; MOVH.L        X0, #8, IOADR
	0x66010600,		//           ; MOVIL #DA008, R6
	0x60002100,		//           ; MOVIL #H'0001, R1
	0x0f7c0000,		//           ; MOVH.L        ZERO, #0, R0
	0x0c040600,		//           ; MOVH.L        R1, #0, @R6
	0x00000000,		//           ; NOP   
	0x0f001400,		//           ; MOVH.L        R0, #0, ODAT
	0x00000000,		//           ; NOP   
	0x0f001400,		//           ; MOVH.L        R0, #0, ODAT
	0x00000000,		//           ; NOP   
	0x0f001400,		//           ; MOVH.L        R0, #0, ODAT
	0x00000000,		//           ; NOP   
	0x0f001400,		//           ; MOVH.L        R0, #0, ODAT
	0x0f201900,		//           ; MOVH.L        X0, #0, FLAG
	0x0f420000,		//           ; MOVH.L        D, #16, R0
	0x0f430100,		//           ; MOVH.L        D, #24, R1
	0x40000200,		//           ; MOVIH #H'0000, R2
	0x601fe200,		//           ; MOVIL #H'00FF, R2
	0x14804000,		//           ; AND   R0, R2, R0
	0x14844100,		//           ; AND   R1, R2, R1
	0x18002000,		//           ; CMP   R0, R1
	0x241abb00,		//           ; JMPF  #L120, #27
	0x0f420000,		//           ; MOVH.L        D, #16, R0
	0x40000200,		//           ; MOVIH #H'0000, R2
	0x601fe200,		//           ; MOVIL #H'00FF, R2
	0x14804000,		//           ; AND   R0, R2, R0
	0x600fe200,		//           ; MOVIL #H'007F, R2
	0x18004000,		//           ; CMP   R0, R2
	0x205c5c00,		//           ; JMPT  #L400, #28
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0f221200,		//           ; MOVH.L        X0, #16, IOADR
	0x1748a000,		//           ; SET   IOADR, #5
	0x205c4500,		//           ; JMPT  #L400, #5
	0x16c8a000,		//           ; RST   IOADR, #5
	0x0f240500,		//           ; MOVH.L        X1, #0, R5
	0x40601600,		//           ; MOVIH #H'0300, MODCTLX
	0x607fd600,		//           ; MOVIL #H'03FE, MODCTLX
	0x1769c000,		//           ; SET   SSIZE, #14
	0x0d4c0500,		//           ; MOVH.L        IDAT, #0, @R5+
	0x0f420000,		//           ; MOVH.L        D, #16, R0
	0x0f430100,		//           ; MOVH.L        D, #24, R1
	0x40000200,		//           ; MOVIH #H'0000, R2
	0x601fe200,		//           ; MOVIL #H'00FF, R2
	0x14804000,		//           ; AND   R0, R2, R0
	0x14844100,		//           ; AND   R1, R2, R1
	0x11002000,		//           ; ADDI  R0, #1, R0
	0x0f820000,		//           ; MOVL.L        R0, #16, R0
	0x0f870100,		//           ; MOVL.L        R1, #24, R1
	0x15002000,		//           ; OR    R0, R1, R0
	0x0f001000,		//           ; MOVH.L        R0, #0, D
	0x0f240000,		//           ; MOVH.L        X1, #0, R0
	0x0f140100,		//           ; MOVH.L        R5, #0, R1
	0x5fffe200,		//           ; MOVIH #H'FFFF, R2
	0x60000200,		//           ; MOVIL #H'0000, R2
	0x14804000,		//           ; AND   R0, R2, R0
	0x15002000,		//           ; OR    R0, R1, R0
	0x2008fe00,		//           ; JMPT  #L020, #30
	0x0f000900,		//           ; MOVH.L        R0, #0, X1
	0x0f240500,		//           ; MOVH.L        X1, #0, R5
	0x0f140200,		//           ; MOVH.L        R5, #0, R2
	0x0f211200,		//           ; MOVH.L        X0, #8, IOADR
	0x1748a000,		//           ; SET   IOADR, #5
	0x2015c900,		//           ; JMPT  #L060, #9
	0x16c8a000,		//           ; RST   IOADR, #5
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x201d6100,		//           ; JMPT  #L160, #1
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x2020c200,		//           ; JMPT  #L180, #2
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x2029e300,		//           ; JMPT  #L200, #3
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x20467e00,		//           ; JMPT  #L220, #30
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0f260500,		//           ; MOVH.L        X1, #16, R5
	0x0a941f00,		//           ; MOVL.L        @R5+, #0, ZERO
	0x09941f00,		//           ; MOVL.L        @R5, #0, ZERO
	0x09940000,		//           ; MOVL.L        @R5, #0, R0
	0x0f000a00,		//           ; MOVH.L        R0, #0, A0
	0x60000a00,		//           ; MOVIL #0000, A0
	0x0f820b00,		//           ; MOVL.L        R0, #16, A1
	0x0f240000,		//           ; MOVH.L        X1, #0, R0
	0x0f960100,		//           ; MOVL.L        R5, #16, R1
	0x40000200,		//           ; MOVIH #H'0000, R2
	0x7fffe200,		//           ; MOVIL #H'FFFF, R2
	0x14804000,		//           ; AND   R0, R2, R0
	0x15002000,		//           ; OR    R0, R1, R0
	0x0f000900,		//           ; MOVH.L        R0, #0, X1
	0x0f240500,		//           ; MOVH.L        X1, #0, R5
	0x0f420000,		//           ; MOVH.L        D, #16, R0
	0x0f430100,		//           ; MOVH.L        D, #24, R1
	0x40000200,		//           ; MOVIH #H'0000, R2
	0x601fe200,		//           ; MOVIL #H'00FF, R2
	0x14804000,		//           ; AND   R0, R2, R0
	0x14844100,		//           ; AND   R1, R2, R1
	0x12802000,		//           ; SUBI  R0, #1, R0
	0x0f820000,		//           ; MOVL.L        R0, #16, R0
	0x20467e00,		//           ; JMPT  #L220, #30
	0x0f870100,		//           ; MOVL.L        R1, #24, R1
	0x15002000,		//           ; OR    R0, R1, R0
	0x0f001000,		//           ; MOVH.L        R0, #0, D
	0x0f260500,		//           ; MOVH.L        X1, #16, R5
	0x0a941f00,		//           ; MOVL.L        @R5+, #0, ZERO
	0x09941f00,		//           ; MOVL.L        @R5, #0, ZERO
	0x09940000,		//           ; MOVL.L        @R5, #0, R0
	0x0f240100,		//           ; MOVH.L        X1, #0, R1
	0x0f960200,		//           ; MOVL.L        R5, #16, R2
	0x40000300,		//           ; MOVIH #H'0000, R3
	0x7fffe300,		//           ; MOVIL #H'FFFF, R3
	0x14846100,		//           ; AND   R1, R3, R1
	0x15044100,		//           ; OR    R1, R2, R1
	0x0f040900,		//           ; MOVH.L        R1, #0, X1
	0x66180600,		//           ; MOVIL #STACK0, R6
	0x0d200600,		//           ; MOVH.L        X0, #0, @R6+
	0x0d240600,		//           ; MOVH.L        X1, #0, @R6+
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x66044600,		//           ; MOVIL #SA002, R6
	0x09981f00,		//           ; MOVL.L        @R6, #0, ZERO
	0x09981f00,		//           ; MOVL.L        @R6, #0, ZERO
	0x09980100,		//           ; MOVL.L        @R6, #0, R1
	0x12842200,		//           ; SUBI  R1, #1, R2
	0x0f881600,		//           ; MOVL.L        R2, #0, MODCTLX
	0x66060600,		//           ; MOVIL #SA010, R6
	0x09981f00,		//           ; MOVL.L        @R6, #0, ZERO
	0x09981f00,		//           ; MOVL.L        @R6, #0, ZERO
	0x09980500,		//           ; MOVL.L        @R6, #0, R5
	0x0d000500,		//           ; MOVH.L        R0, #0, @R5+
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0c940600,		//           ; MOVL.L        R5, #0, @R6
	0x66040600,		//           ; MOVIL #SA000, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181a00,		//           ; MOVH.L        @R6+, #0, SSIZE
	0x09181600,		//           ; MOVH.L        @R6, #0, MODCTLX
	0x09180600,		//           ; MOVH.L        @R6, #0, R6
	0x0ffc0a00,		//           ; MOVL.L        ZERO, #0, A0
	0x0ffc0b00,		//           ; MOVL.L        ZERO, #0, A1
	0x40269800,		//           ; MOVIH #LPS1, LOOPADR
	0x6026d800,		//           ; MOVIL #LPE1, LOOPADR
	0x84480000,		//           ; 
	0x84da0000,		//           ; 
	0x84480000,		//           ; 
	0x84da0000,		//           ; 
	0x84488000,		//           ; 
	0x84da9400,		//           ; 
	0x84490000,		//           ; 
	0x84db1700,		//           ; 
	0x84490000,		//           ; 
	0x84db1700,		//           ; 
	0x82490000,		//           ; 
	0x82db1700,		//           ; 
	0x80010000,		//           ; 
	0x80011700,		//           ; 
	0xe4148000,		//           ; 
	0xe41cc000,		//           ; 
	0x66180600,		//           ; MOVIL #STACK0, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a180800,		//           ; MOVH.L        @R6+, #0, X0
	0x0a180900,		//           ; MOVH.L        @R6+, #0, X1
	0x0f420000,		//           ; MOVH.L        D, #16, R0
	0x0f430100,		//           ; MOVH.L        D, #24, R1
	0x40000200,		//           ; MOVIH #H'0000, R2
	0x601fe200,		//           ; MOVIL #H'00FF, R2
	0x14804000,		//           ; AND   R0, R2, R0
	0x14844100,		//           ; AND   R1, R2, R1
	0x12802000,		//           ; SUBI  R0, #1, R0
	0x0f820000,		//           ; MOVL.L        R0, #16, R0
	0x20467e00,		//           ; JMPT  #L220, #30
	0x0f870100,		//           ; MOVL.L        R1, #24, R1
	0x15002000,		//           ; OR    R0, R1, R0
	0x0f001000,		//           ; MOVH.L        R0, #0, D
	0x66180600,		//           ; MOVIL #STACK0, R6
	0x0d200600,		//           ; MOVH.L        X0, #0, @R6+
	0x0d240600,		//           ; MOVH.L        X1, #0, @R6+
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x66040600,		//           ; MOVIL #SA000, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x09181a00,		//           ; MOVH.L        @R6, #0, SSIZE
	0x09181600,		//           ; MOVH.L        @R6, #0, MODCTLX
	0x66050600,		//           ; MOVIL #SA008, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a180000,		//           ; MOVH.L        @R6+, #0, R0
	0x0a180100,		//           ; MOVH.L        @R6+, #0, R1
	0x09180200,		//           ; MOVH.L        @R6, #0, R2
	0x09180300,		//           ; MOVH.L        @R6, #0, R3
	0x09180500,		//           ; MOVH.L        @R6, #0, R5
	0x180c0000,		//           ; CMP   R3, R0
	0x20333b00,		//           ; JMPT  #LABEL, #27
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x08000800,		//           ; SWAP  R0, X0
	0x08040c00,		//           ; SWAP  R1, Y0
	0x08080d00,		//           ; SWAP  R2, Y1
	0x080c0a00,		//           ; SWAP  R3, A0
	0x0f420000,		//           ; MOVH.L        D, #16, R0
	0x0f430100,		//           ; MOVH.L        D, #24, R1
	0x40000200,		//           ; MOVIH #H'0000, R2
	0x601fe200,		//           ; MOVIL #H'00FF, R2
	0x14804000,		//           ; AND   R0, R2, R0
	0x14844100,		//           ; AND   R1, R2, R1
	0x12802000,		//           ; SUBI  R0, #1, R0
	0x0f820000,		//           ; MOVL.L        R0, #16, R0
	0x0f870100,		//           ; MOVL.L        R1, #24, R1
	0x15002000,		//           ; OR    R0, R1, R0
	0x0f001000,		//           ; MOVH.L        R0, #0, D
	0x40601600,		//           ; MOVIH #H'0300, MODCTLX
	0x607fd600,		//           ; MOVIL #H'03FE, MODCTLX
	0x0f260500,		//           ; MOVH.L        X1, #16, R5
	0x0a941f00,		//           ; MOVL.L        @R5+, #0, ZERO
	0x09941f00,		//           ; MOVL.L        @R5, #0, ZERO
	0x09940000,		//           ; MOVL.L        @R5, #0, R0
	0x0f240100,		//           ; MOVH.L        X1, #0, R1
	0x0f960200,		//           ; MOVL.L        R5, #16, R2
	0x40000300,		//           ; MOVIH #H'0000, R3
	0x7fffe300,		//           ; MOVIL #H'FFFF, R3
	0x14846100,		//           ; AND   R1, R3, R1
	0x15044100,		//           ; OR    R1, R2, R1
	0x0f040900,		//           ; MOVH.L        R1, #0, X1
	0x66044600,		//           ; MOVIL #SA002, R6
	0x09981f00,		//           ; MOVL.L        @R6, #0, ZERO
	0x09981f00,		//           ; MOVL.L        @R6, #0, ZERO
	0x09980100,		//           ; MOVL.L        @R6, #0, R1
	0x12842200,		//           ; SUBI  R1, #1, R2
	0x0f881600,		//           ; MOVL.L        R2, #0, MODCTLX
	0x66060600,		//           ; MOVIL #SA010, R6
	0x09981f00,		//           ; MOVL.L        @R6, #0, ZERO
	0x09981f00,		//           ; MOVL.L        @R6, #0, ZERO
	0x09980500,		//           ; MOVL.L        @R6, #0, R5
	0x0d000500,		//           ; MOVH.L        R0, #0, @R5+
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0c940600,		//           ; MOVL.L        R5, #0, @R6
	0x08000800,		//           ; SWAP  R0, X0
	0x08040c00,		//           ; SWAP  R1, Y0
	0x08080d00,		//           ; SWAP  R2, Y1
	0x080c0a00,		//           ; SWAP  R3, A0
	0x0fb01600,		//           ; MOVL.L        Y0, #0, MODCTLX
	0x202c3e00,		//           ; JMPT  #LABEL2, #30
	0x118c0300,		//           ; SUB   R3, R0, R3
	0x66184600,		//           ; MOVIL #STACK1, R6
	0x0c240600,		//           ; MOVH.L        X1, #0, @R6
	0x100c2100,		//           ; ADD   R3, R1, R1
	0x6605c600,		//           ; MOVIL #SA00E, R6
	0x0d840600,		//           ; MOVL.L        R1, #0, @R6+
	0x0d940600,		//           ; MOVL.L        R5, #0, @R6+
	0x0ffc0100,		//           ; MOVL.L        ZERO, #0, R1
	0x130c4300,		//           ; SHA   R3, R2, R3
	0x0f7c1200,		//           ; MOVH.L        ZERO, #0, IOADR
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x180c0000,		//           ; CMP   R3, R0
	0x20357b00,		//           ; JMPT  #LABEL3, #27
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x2033fe00,		//           ; JMPT  #LABEL4, #30
	0x118c0300,		//           ; SUB   R3, R0, R3
	0x11044100,		//           ; ADDI  R1, #2, R1
	0x00000000,		//           ; NOP   
	0x66048600,		//           ; MOVIL #SA004, R6
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09180600,		//           ; MOVH.L        @R6, #0, R6
	0x40000000,		//           ; MOVIH #H'0000, R0
	0x60004000,		//           ; MOVIL #H'0002, R0
	0x13004000,		//           ; SHA   R0, R2, R0
	0x10180600,		//           ; ADD   R6, R0, R6
	0x11982600,		//           ; SUB   R6, R1, R6
	0x0ffc0a00,		//           ; MOVL.L        ZERO, #0, A0
	0x0ffc0b00,		//           ; MOVL.L        ZERO, #0, A1
	0x4037d800,		//           ; MOVIH #LPS, LOOPADR
	0x60381800,		//           ; MOVIL #LPE, LOOPADR
	0x84480000,		//           ; 
	0x84da0000,		//           ; 
	0x84480000,		//           ; 
	0x84da0000,		//           ; 
	0x84488000,		//           ; 
	0x84da9400,		//           ; 
	0x84490000,		//           ; 
	0x84db1700,		//           ; 
	0x84490000,		//           ; 
	0x84db1700,		//           ; 
	0x84490000,		//           ; 
	0x84db1700,		//           ; 
	0x80010000,		//           ; 
	0x80011700,		//           ; 
	0xe4148000,		//           ; 
	0xe41cc000,		//           ; 
	0xe4f08000,		//           ; 
	0xe4f8c000,		//           ; 
	0xe5300000,		//           ; 
	0xe5380000,		//           ; 
	0x66064600,		//           ; MOVIL #SA012, R6
	0x0f2a0a00,		//           ; MOVH.L        A0, #16, A0
	0x0da80600,		//           ; MOVL.L        A0, #0, @R6+
	0x0f2e0b00,		//           ; MOVH.L        A1, #16, A1
	0x0dac0600,		//           ; MOVL.L        A1, #0, @R6+
	0x66060600,		//           ; MOVIL #SA010, R6
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09180500,		//           ; MOVH.L        @R6, #0, R5
	0x66040600,		//           ; MOVIL #SA000, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x09181a00,		//           ; MOVH.L        @R6, #0, SSIZE
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09180600,		//           ; MOVH.L        @R6, #0, R6
	0x40000000,		//           ; MOVIH #H'0000, R0
	0x60004000,		//           ; MOVIL #H'0002, R0
	0x13004000,		//           ; SHA   R0, R2, R0
	0x10180600,		//           ; ADD   R6, R0, R6
	0x11982600,		//           ; SUB   R6, R1, R6
	0x12984600,		//           ; SUBI  R6, #2, R6
	0x0ffc0a00,		//           ; MOVL.L        ZERO, #0, A0
	0x0ffc0b00,		//           ; MOVL.L        ZERO, #0, A1
	0x403d7800,		//           ; MOVIH #LPS2, LOOPADR
	0x603db800,		//           ; MOVIL #LPE2, LOOPADR
	0x84480000,		//           ; 
	0x84da0000,		//           ; 
	0x84480000,		//           ; 
	0x84da0000,		//           ; 
	0x84488000,		//           ; 
	0x84da9400,		//           ; 
	0x84490000,		//           ; 
	0x84db1700,		//           ; 
	0x84490000,		//           ; 
	0x84db1700,		//           ; 
	0x84490000,		//           ; 
	0x84db1700,		//           ; 
	0x80010000,		//           ; 
	0x80011700,		//           ; 
	0xe4148000,		//           ; 
	0xe41cc000,		//           ; 
	0xe4f08000,		//           ; 
	0xe4f8c000,		//           ; 
	0xe5300000,		//           ; 
	0xe5380000,		//           ; 
	0x66050600,		//           ; MOVIL #SA008, R6
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09180000,		//           ; MOVH.L        @R6, #0, R0
	0x40021a00,		//           ; MOVIH #H'0010, SSIZE
	0x68001a00,		//           ; MOVIL #H'4000, SSIZE
	0x0f8c0c00,		//           ; MOVL.L        R3, #0, Y0
	0x11806300,		//           ; SUB   R0, R3, R3
	0x0f8c0d00,		//           ; MOVL.L        R3, #0, Y1
	0x0f2a0800,		//           ; MOVH.L        A0, #16, X0
	0x0ffc0a00,		//           ; MOVL.L        ZERO, #0, A0
	0xe8000000,		//           ; 
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0xe4148000,		//           ; 
	0x0f2e0800,		//           ; MOVH.L        A1, #16, X0
	0x0ffc0b00,		//           ; MOVL.L        ZERO, #0, A1
	0xe8000000,		//           ; 
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0xe41cc000,		//           ; 
	0x66064600,		//           ; MOVIL #SA012, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a180800,		//           ; MOVH.L        @R6+, #0, X0
	0x0a180900,		//           ; MOVH.L        @R6+, #0, X1
	0x0a180c00,		//           ; MOVH.L        @R6+, #0, Y0
	0xe8400000,		//           ; 
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0xe4148000,		//           ; 
	0xe9400000,		//           ; 
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0xe41cc000,		//           ; 
	0x4064da00,		//           ; MOVIH #H'0326, SSIZE
	0x78001a00,		//           ; MOVIL #H'C000, SSIZE
	0x08280800,		//           ; SWAP  A0, X0
	0x0ffc0a00,		//           ; MOVL.L        ZERO, #0, A0
	0xe8000000,		//           ; 
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0xe4148000,		//           ; 
	0x082c0800,		//           ; SWAP  A1, X0
	0x0ffc0b00,		//           ; MOVL.L        ZERO, #0, A1
	0xe8000000,		//           ; 
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0xe41cc000,		//           ; 
	0x66180600,		//           ; MOVIL #STACK0, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a180800,		//           ; MOVH.L        @R6+, #0, X0
	0x0a180900,		//           ; MOVH.L        @R6+, #0, X1
	0x20467e00,		//           ; JMPT  #L220, #30
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x24480400,		//           ; JMPF  #L280, #4
	0x0f231200,		//           ; MOVH.L        X0, #24, IOADR
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0f4c0c00,		//           ; MOVH.L        IDAT, #0, Y0
	0x0fb20d00,		//           ; MOVL.L        Y0, #16, Y1
	0x0f320c00,		//           ; MOVH.L        Y0, #16, Y0
	0x0fb20c00,		//           ; MOVL.L        Y0, #16, Y0
	0xe4108000,		//           ; 
	0xe41ac000,		//           ; 
	0x2456c500,		//           ; JMPF  #L320, #5
	0x66180600,		//           ; MOVIL #STACK0, R6
	0x0d200600,		//           ; MOVH.L        X0, #0, @R6+
	0x0d240600,		//           ; MOVH.L        X1, #0, @R6+
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x660c0600,		//           ; MOVIL #GE000, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181a00,		//           ; MOVH.L        @R6+, #0, SSIZE
	0x0a180000,		//           ; MOVH.L        @R6+, #0, R0
	0x0a180300,		//           ; MOVH.L        @R6+, #0, R3
	0x404a9800,		//           ; MOVIH #LPS3, LOOPADR
	0x6052d800,		//           ; MOVIL #LPE3, LOOPADR
	0xe5300000,		//           ; 
	0xe5380000,		//           ; 
	0x0f29aa00,		//           ; MOVH.L        A0, #13, A0
	0x0f2dab00,		//           ; MOVH.L        A1, #13, A1
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x11004500,		//           ; ADDI  R0, #2, R5
	0x110c4600,		//           ; ADDI  R3, #2, R6
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x94500000,		//           ; 
	0x92500000,		//           ; 
	0x00000000,		//           ; NOP   
	0x92488000,		//           ; 
	0xe8000000,		//           ; 
	0xe4148000,		//           ; 
	0xe4148000,		//           ; 
	0x11000500,		//           ; ADDI  R0, #0, R5
	0x0ca80500,		//           ; MOVL.L        A0, #0, @R5
	0x0ffc0a00,		//           ; MOVL.L        ZERO, #0, A0
	0x94500000,		//           ; 
	0x94500000,		//           ; 
	0x92480000,		//           ; 
	0x92488000,		//           ; 
	0x92488000,		//           ; 
	0x80010000,		//           ; 
	0xe4148000,		//           ; 
	0xe4148000,		//           ; 
	0x11000500,		//           ; ADDI  R0, #0, R5
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0a940100,		//           ; MOVL.L        @R5+, #0, R1
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0a940200,		//           ; MOVL.L        @R5+, #0, R2
	0x0c880500,		//           ; MOVL.L        R2, #0, @R5
	0x11004500,		//           ; ADDI  R0, #2, R5
	0x0c840500,		//           ; MOVL.L        R1, #0, @R5
	0x11010500,		//           ; ADDI  R0, #8, R5
	0x110c4600,		//           ; ADDI  R3, #2, R6
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x94500000,		//           ; 
	0x92500000,		//           ; 
	0x00000000,		//           ; NOP   
	0x92488000,		//           ; 
	0xe8000000,		//           ; 
	0xe41cc000,		//           ; 
	0xe41cc000,		//           ; 
	0x1100c500,		//           ; ADDI  R0, #6, R5
	0x0cac0500,		//           ; MOVL.L        A1, #0, @R5
	0x0ffc0b00,		//           ; MOVL.L        ZERO, #0, A1
	0x94500000,		//           ; 
	0x94500000,		//           ; 
	0x92480000,		//           ; 
	0x92488000,		//           ; 
	0x92488000,		//           ; 
	0x80010300,		//           ; 
	0xe41cc000,		//           ; 
	0xe41cc000,		//           ; 
	0x1100c500,		//           ; ADDI  R0, #6, R5
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0a940100,		//           ; MOVL.L        @R5+, #0, R1
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0a940200,		//           ; MOVL.L        @R5+, #0, R2
	0x0c880500,		//           ; MOVL.L        R2, #0, @R5
	0x11010500,		//           ; ADDI  R0, #8, R5
	0x0c840500,		//           ; MOVL.L        R1, #0, @R5
	0x11018000,		//           ; ADDI  R0, #12, R0
	0x110d8300,		//           ; ADDI  R3, #12, R3
	0x660cc600,		//           ; MOVIL #GE003, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09180c00,		//           ; MOVH.L        @R6, #0, Y0
	0x09180800,		//           ; MOVH.L        @R6, #0, X0
	0x4020da00,		//           ; MOVIH #H'0106, SSIZE
	0x70201a00,		//           ; MOVIL #H'8100, SSIZE
	0x08280b00,		//           ; SWAP  A0, A1
	0xeb800000,		//           ; 
	0x08280b00,		//           ; SWAP  A0, A1
	0x0ffc0a00,		//           ; MOVL.L        ZERO, #0, A0
	0xe4148000,		//           ; 
	0xe4f08000,		//           ; 
	0xe5300000,		//           ; 
	0xeb800000,		//           ; 
	0x0ffc0b00,		//           ; MOVL.L        ZERO, #0, A1
	0x00000000,		//           ; NOP   
	0xe41cc000,		//           ; 
	0xe4f8c000,		//           ; 
	0xe5380000,		//           ; 
	0x24563400,		//           ; JMPF  #L310, #20
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0xe4200000,		//           ; 
	0x0c200600,		//           ; MOVH.L        X0, #0, @R6
	0x16e68000,		//           ; RST   FLAG, #20
	0x66180600,		//           ; MOVIL #STACK0, R6
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a181f00,		//           ; MOVH.L        @R6+, #0, ZERO
	0x0a180800,		//           ; MOVH.L        @R6+, #0, X0
	0x0a180900,		//           ; MOVH.L        @R6+, #0, X1
	0x0f740c00,		//           ; MOVH.L        DVCTLA, #0, Y0
	0x0fb20d00,		//           ; MOVL.L        Y0, #16, Y1
	0x0f320c00,		//           ; MOVH.L        Y0, #16, Y0
	0x0fb20c00,		//           ; MOVL.L        Y0, #16, Y0
	0xe4f08000,		//           ; 
	0xe4f8c000,		//           ; 
	0xe5300000,		//           ; 
	0xe5380000,		//           ; 
	0x4060ba00,		//           ; MOVIH #H'0305, SSIZE
	0x70001a00,		//           ; MOVIL #H'8000, SSIZE
	0x08280b00,		//           ; SWAP  A0, A1
	0xeb000000,		//           ; 
	0x08280b00,		//           ; SWAP  A0, A1
	0x0ffc0a00,		//           ; MOVL.L        ZERO, #0, A0
	0xe4148000,		//           ; 
	0xeb400000,		//           ; 
	0x0ffc0b00,		//           ; MOVL.L        ZERO, #0, A1
	0x00000000,		//           ; NOP   
	0xe41cc000,		//           ; 
	0xe4f08000,		//           ; 
	0xe4f8c000,		//           ; 
	0xe5300000,		//           ; 
	0xe5380000,		//           ; 
	0x0fa80000,		//           ; MOVL.L        A0, #0, R0
	0x5fffe100,		//           ; MOVIH #H'FFFF, R1
	0x60000100,		//           ; MOVIL #H'0000, R1
	0x14802000,		//           ; AND   R0, R1, R0
	0x0f2e0200,		//           ; MOVH.L        A1, #16, R2
	0x40000200,		//           ; MOVIH #H'0000, R2
	0x15004000,		//           ; OR    R0, R2, R0
	0x245b6600,		//           ; JMPF  #L380, #6
	0x66100600,		//           ; MOVIL #S000, R6
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09181f00,		//           ; MOVH.L        @R6, #0, ZERO
	0x09180100,		//           ; MOVH.L        @R6, #0, R1
	0x16002000,		//           ; XOR   R0, R1, R0
	0x0c000600,		//           ; MOVH.L        R0, #0, @R6
	0x0f211200,		//           ; MOVH.L        X0, #8, IOADR
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x2015de00,		//           ; JMPT  #L060, #30
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x0f001400,		//           ; MOVH.L        R0, #0, ODAT
	0x0f211200,		//           ; MOVH.L        X0, #8, IOADR
	0x17488000,		//           ; SET   IOADR, #4
	0x0f221200,		//           ; MOVH.L        X0, #16, IOADR
	0x17488000,		//           ; SET   IOADR, #4
	0x0f231200,		//           ; MOVH.L        X0, #24, IOADR
	0x17488000,		//           ; SET   IOADR, #4
	0x66000600,		//           ; MOVIL #DA000, R6
	0x0d200600,		//           ; MOVH.L        X0, #0, @R6+
	0x0d240600,		//           ; MOVH.L        X1, #0, @R6+
	0x0d400600,		//           ; MOVH.L        D, #0, @R6+
	0x0f201900,		//           ; MOVH.L        X0, #0, FLAG
	0x2008e700,		//           ; JMPT  #L020, #7
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x1f800000,		//           ; HLT   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
	0x00000000,		//           ; NOP   
};

/* YRAM offset=0x0000 */
const long siu_spb_pro_y0[YRAM0SIZE] = {
	0x000a0300,		// H'00         ydef[0]         passA=disable
	0x03000300,		// H'04         ydef[1]         
	0x08000000,		// H'08         ydef[2]         
	0x00000000,		// H'0c         ydef[3]         
	0x00000000,		// H'10         ydef[4]         
	0x00010900,		// H'14         ydef[5]         portB -> CPU            passB=disable
	0x00020000,		// H'18         ydef[6]         pathB 
	0x00080000,		// H'1c         ydef[7]         pathA 
};

/* YRAM offset=0x0040 */
const long siu_spb_pro_y1[YRAM1SIZE] = {
	0x2304501b,		// H'40         ydef[16]        SRC -> over=8, 31tap
	0x0000003f,		// H'44         ydef[17]        

	0x00000100,		// H'48         ydef[18]        
	0x00000000,		// H'4c         ydef[19]        

	0x0000ac44,		// H'50         ydef[20]        outfs
	0x0000ac44,		// H'54         ydef[21]        infs
	0x00000003,		// H'58         ydef[22]        over,   =log2(8)
	0x00000000,		// H'5c         ydef[23]        
	0x00000000,		// H'60         ydef[24]        
	0x00000000,		// H'64         ydef[25]        
	0x00000000,		// H'68         ydef[26]        
	0x00008312,		// H'6c         ydef[27]        =(1/outfs)*0x40000000
};

/* YRAM offset=0x00c0 */
static long siu_spb_pro_y2[YRAM2SIZE] = {
	0x03041003,		// band=3

	0x00000200,		// IIR data start address/2
	0x00000300,
	0x01000000,
	0x40000000,
};

/* YRAM offset=0x0100 */
static long siu_spb_pro_y3[YRAM3SIZE] = {
	0x00000000,
	0x00000000
};

/* YRAM offset=0x0180 */
static long siu_spb_pro_y4[YRAM4SIZE] = {
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000
};

/*----------------------------------------------------------------------------
  Sampling Rate Conversion Filter Coefficient
  Input Sampling Frequency = 8 kHz
  Output Sampling Frequency = 44.1 kHz
  Over-Sampling Rate = 8 times
  31 TAPs
*/
static unsigned long SIU_SRC_8000_44100[YRAMFIRSIZE] = {
	0x00000000, 0x000000E0, 0x000003E1, 0x0000091D, 0x000010E2, 0x00001A6A,
	0x000023B7, 0x000029AB, 0x00002872, 0x00001C2E,
	0x000001D6, 0xFFFFD839, 0xFFFFA0E0, 0xFFFF60CA, 0xFFFF209C, 0xFFFEEC41,
	0xFFFED1D1, 0xFFFEDFB9, 0xFFFF2242, 0xFFFFA0C9,
	0x00005AEF, 0x0001465E, 0x00024D82, 0x00034FDF, 0x00042432, 0x00049C9D,
	0x00048C9F, 0x0003D062, 0x00025483, 0x00001D34,
	0xFFFD4B7D, 0xFFFA1F50, 0xFFF6F576, 0xFFF440A1, 0xFFF27DBA, 0xFFF2240E,
	0xFFF392DE, 0xFFF6FE8B, 0xFFFC5FF1, 0x000368F0,
	0x000B80BD, 0x0013CA07, 0x001B3402, 0x002095EB, 0x0022D314, 0x0021040A,
	0x001A9FE0, 0x000FA003, 0x0000989C, 0xFFEEC03E,
	0xFFDBE2E1, 0xFFCA3EA9, 0xFFBC4B69, 0xFFB4712A, 0xFFB4B4DC, 0xFFBE64CF,
	0xFFD1D00D, 0xFFEE13ED, 0x001109CA, 0x00375C47,
	0x005CC6F1, 0x007C7E3E, 0x0091B814, 0x009847AC, 0x008D3B80, 0x006F68DE,
	0x003FD138, 0x0001CE79, 0xFFBAF99E, 0xFF72C55A,
	0xFF31CFEA, 0xFF00F8A9, 0xFEE84F66, 0xFEEDFAF9, 0xFF1538CE, 0xFF5D9932,
	0xFFC2974A, 0x003BA51B, 0x00BCB7D2, 0x0137520F,
	0x019BFA3E, 0x01DBFB62, 0x01EB42D2, 0x01C22325, 0x015EC3D5, 0x00C60719,
	0x0003BBD2, 0xFF2A0154, 0xFE4FD9B5, 0xFD8EFFB2,
	0xFD012C88, 0xFCBD1224, 0xFCD35A8C, 0xFD4C0770, 0xFE2489C9, 0xFF4EDC28,
	0x00B1D4EE, 0x022AC612, 0x03905DB3, 0x04B69487,
	0x057355F2, 0x05A3707C, 0x052F4CE6, 0x040EE405, 0x024C7767, 0x0005A694,
	0xFD6AA23C, 0xFABB68D4, 0xF8432E59, 0xF652414F,
	0xF536ECF2, 0xF535FAF2, 0xF6838915, 0xF93CEBD1, 0xFD643E32, 0x02DE291F,
	0x09722D80, 0x10CD8C58, 0x1888A217, 0x202E4E9A,
	0x2744CEE9, 0x2D57402A, 0x31FF0232, 0x34EC0D4F, 0x35EB8AEB, 0x34EC0D4D,
	0x31FF022A, 0x2D57402C, 0x2744CEE9, 0x202E4E98,
	0x1888A21B, 0x10CD8C58, 0x09722D80, 0x02DE291C, 0xFD643E35, 0xF93CEBD1,
	0xF6838915, 0xF535FAF3, 0xF536ECFB, 0xF6524159,
	0xF8432E61, 0xFABB68DA, 0xFD6AA243, 0x0005A694, 0x024C7764, 0x040EE400,
	0x052F4CDA, 0x05A3706E, 0x057355E5, 0x04B69481,
	0x03905DAE, 0x022AC60F, 0x00B1D4EE, 0xFF4EDC2A, 0xFE2489CE, 0xFD4C0778,
	0xFCD35A91, 0xFCBD122E, 0xFD012C96, 0xFD8EFFB5,
	0xFE4FD9BA, 0xFF2A0155, 0x0003BBD2, 0x00C60718, 0x015EC3CD, 0x01C22322,
	0x01EB42CA, 0x01DBFB5B, 0x019BFA39, 0x0137520A,
	0x00BCB7CE, 0x003BA51A, 0xFFC2974B, 0xFF5D9934, 0xFF1538D0, 0xFEEDFAFE,
	0xFEE84F6C, 0xFF00F8AF, 0xFF31CFEE, 0xFF72C55D,
	0xFFBAF99F, 0x0001CE79, 0x003FD137, 0x006F68DB, 0x008D3B7C, 0x009847AA,
	0x0091B810, 0x007C7E3B, 0x005CC6EE, 0x00375C45,
	0x001109C9, 0xFFEE13EE, 0xFFD1D00E, 0xFFBE64D0, 0xFFB4B4DE, 0xFFB4712C,
	0xFFBC4B6C, 0xFFCA3EAB, 0xFFDBE2E2, 0xFFEEC03E,
	0x0000989C, 0x000FA002, 0x001A9FDF, 0x00210409, 0x0022D313, 0x002095E9,
	0x001B3401, 0x0013CA06, 0x000B80BD, 0x000368F0,
	0xFFFC5FF1, 0xFFF6FE8B, 0xFFF392DF, 0xFFF2240E, 0xFFF27DBB, 0xFFF440A2,
	0xFFF6F576, 0xFFFA1F51, 0xFFFD4B7D, 0x00001D34,
	0x00025483, 0x0003D062, 0x00048C9F, 0x00049C9D, 0x00042432, 0x00034FDF,
	0x00024D82, 0x0001465E, 0x00005AEF, 0xFFFFA0C9,
	0xFFFF2242, 0xFFFEDFB9, 0xFFFED1D1, 0xFFFEEC41, 0xFFFF209C, 0xFFFF60CA,
	0xFFFFA0E0, 0xFFFFD839, 0x000001D6, 0x00001C2E,
	0x00002872, 0x000029AB, 0x000023B7, 0x00001A6A, 0x000010E2, 0x0000091D,
	0x000003E1, 0x000000E0, 0xFFFFFF0E, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

/*
  Sampling Rate Conversion Filter Coefficient
  Input Sampling Frequency = 11.025 kHz
  Output Sampling Frequency = 44.1 kHz
  Over-Sampling Rate = 8 times
  31 TAPs
*/
static unsigned long SIU_SRC_11025_44100[YRAMFIRSIZE] = {
	0x00000000, 0x000000F7, 0x000003FF, 0x0000093F, 0x00001100, 0x00001A74,
	0x00002396, 0x00002946, 0x000027B4, 0x00001B0B,
	0x00000055, 0xFFFFD677, 0xFFFF9F16, 0xFFFF5F49, 0xFFFF1FC8, 0xFFFEEC88,
	0xFFFED395, 0xFFFEE33C, 0xFFFF2794, 0xFFFFA7B4,
	0x000062E9, 0x00014E88, 0x000254B5, 0x000354C6, 0x00042570, 0x00049902,
	0x00048362, 0x0003C157, 0x00024049, 0x0000054C,
	0xFFFD3249, 0xFFFA07EA, 0xFFF6E369, 0xFFF43784, 0xFFF280AC, 0xFFF23533,
	0xFFF3B2E1, 0xFFF72C3B, 0xFFFC9812, 0x0003A648,
	0x000BBC6B, 0x0013FC25, 0x001B5485, 0x00209DAA, 0x0022BCE4, 0x0020CDBB,
	0x001A4B00, 0x000F3242, 0x00001BB0, 0xFFEE413D,
	0xFFDB7113, 0xFFC9E9E6, 0xFFBC2226, 0xFFB47E76, 0xFFB4FE83, 0xFFBEE9F0,
	0xFFD2885A, 0xFFEEEFC2, 0x0011F33B, 0x00383915,
	0x005D7B48, 0x007CEFF4, 0x0091D215, 0x0097FD20, 0x008C8A40, 0x006E5ADC,
	0x003E7C8B, 0x000053E8, 0xFFB981B7, 0xFF717C4B,
	0xFF30E085, 0xFF00871B, 0xFEE87446, 0xFEEEBF4D, 0xFF16939E, 0xFF5F6F19,
	0xFFC4BC32, 0x003DE022, 0x00BEC90F, 0x0138F996,
	0x019CFFAF, 0x01DC3520, 0x01EA9BF2, 0x01C09F54, 0x015C808D, 0x00C339DB,
	0x0000AD6A, 0xFF270684, 0xFE4D4A0E, 0xFD8D2B83,
	0xFD005323, 0xFCBD5992, 0xFCD4C9CD, 0xFD4E8419, 0xFE27D967, 0xFF52A97D,
	0x00B5B886, 0x022E5162, 0x03932787, 0x04B8454E,
	0x0573B281, 0x05A261E9, 0x052CE47A, 0x040B5AA9, 0x02482871, 0x00010673,
	0xFD6631F1, 0xFAB7A79B, 0xF8408B23, 0xF6510DBE,
	0xF537535C, 0xF537F8B7, 0xF686EDF9, 0xF9415E92, 0xFD6945E6, 0x02E33A5C,
	0x0976B9D1, 0x10D11247, 0x188ABBED, 0x202EBE19,
	0x274384AC, 0x2D545DB5, 0x31FAD676, 0x34E70C05, 0x35E63FAE, 0x34E70C03,
	0x31FAD66E, 0x2D545DB7, 0x274384AC, 0x202EBE18,
	0x188ABBF0, 0x10D11248, 0x0976B9D1, 0x02E33A59, 0xFD6945E9, 0xF9415E91,
	0xF686EDF9, 0xF537F8B7, 0xF5375366, 0xF6510DC8,
	0xF8408B2C, 0xFAB7A7A1, 0xFD6631F8, 0x00010673, 0x0248286E, 0x040B5AA4,
	0x052CE46E, 0x05A261DB, 0x0573B274, 0x04B84548,
	0x03932782, 0x022E515F, 0x00B5B885, 0xFF52A97F, 0xFE27D96C, 0xFD4E8421,
	0xFCD4C9D2, 0xFCBD599C, 0xFD005331, 0xFD8D2B87,
	0xFE4D4A14, 0xFF270686, 0x0000AD6A, 0x00C339DA, 0x015C8085, 0x01C09F51,
	0x01EA9BEA, 0x01DC3519, 0x019CFFAA, 0x0138F990,
	0x00BEC90B, 0x003DE021, 0xFFC4BC33, 0xFF5F6F1B, 0xFF1693A0, 0xFEEEBF51,
	0xFEE8744C, 0xFF008721, 0xFF30E089, 0xFF717C4E,
	0xFFB981B8, 0x000053E8, 0x003E7C8A, 0x006E5AD9, 0x008C8A3C, 0x0097FD1E,
	0x0091D211, 0x007CEFF1, 0x005D7B45, 0x00383913,
	0x0011F33A, 0xFFEEEFC3, 0xFFD2885B, 0xFFBEE9F2, 0xFFB4FE85, 0xFFB47E78,
	0xFFBC2229, 0xFFC9E9E8, 0xFFDB7114, 0xFFEE413D,
	0x00001BB0, 0x000F3241, 0x001A4B00, 0x0020CDBA, 0x0022BCE3, 0x00209DA9,
	0x001B5484, 0x0013FC24, 0x000BBC6A, 0x0003A647,
	0xFFFC9812, 0xFFF72C3C, 0xFFF3B2E2, 0xFFF23533, 0xFFF280AC, 0xFFF43784,
	0xFFF6E36A, 0xFFFA07EA, 0xFFFD3249, 0x0000054C,
	0x00024049, 0x0003C157, 0x00048361, 0x00049901, 0x00042570, 0x000354C6,
	0x000254B5, 0x00014E88, 0x000062E9, 0xFFFFA7B4,
	0xFFFF2794, 0xFFFEE33C, 0xFFFED395, 0xFFFEEC88, 0xFFFF1FC9, 0xFFFF5F49,
	0xFFFF9F16, 0xFFFFD677, 0x00000055, 0x00001B0B,
	0x000027B4, 0x00002946, 0x00002396, 0x00001A74, 0x00001100, 0x0000093F,
	0x000003FF, 0x000000F7, 0xFFFFFF23, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

/*
  Sampling Rate Conversion Filter Coefficient
  Input Sampling Frequency = 12 kHz
  Output Sampling Frequency = 44.1 kHz
  Over-Sampling Rate = 8 times
  31 TAPs
*/
static unsigned long SIU_SRC_12000_44100[YRAMFIRSIZE] = {
	0x00000000, 0x0000011D, 0x0000042D, 0x00000974, 0x0000112D, 0x00001A80,
	0x0000235D, 0x000028A2, 0x00002684, 0x0000193E,
	0xFFFFFDF6, 0xFFFFD3B5, 0xFFFF9C4C, 0xFFFF5CF9, 0xFFFF1E92, 0xFFFEED14,
	0xFFFED67B, 0xFFFEE8E2, 0xFFFF3010, 0xFFFFB2A7,
	0x00006F77, 0x00015B4E, 0x00025FE0, 0x00035C3A, 0x0004270C, 0x000492EF,
	0x0004746D, 0x0003A94A, 0x0002202C, 0xFFFFDF8F,
	0xFFFD0AB3, 0xFFF9E366, 0xFFF6C78F, 0xFFF429F0, 0xFFF2863D, 0xFFF25130,
	0xFFF3E63A, 0xFFF774F0, 0xFFFCF0E9, 0x000406EB,
	0x000C19F4, 0x00144A20, 0x001B8650, 0x0020A81B, 0x002297FF, 0x00207643,
	0x0019C3A4, 0x000E8433, 0xFFFF566F, 0xFFED7989,
	0xFFDABF10, 0xFFC9667B, 0xFFBBE3F3, 0xFFB496A6, 0xFFB575E5, 0xFFBFBEC1,
	0xFFD3AD21, 0xFFF04B85, 0x0013632E, 0x003993BE,
	0x005E94D3, 0x007D9F59, 0x0091F671, 0x009782AA, 0x008B6E1E, 0x006CAD49,
	0x003C60D8, 0xFFFDFE23, 0xFFB73202, 0xFF6F7851,
	0xFF2F6B94, 0xFEFFD9D3, 0xFEE8B4D7, 0xFEEFFB44, 0xFF18BC24, 0xFF625853,
	0xFFC82013, 0x004164A8, 0x00C2095A, 0x013B9106,
	0x019E95CC, 0x01DC88E9, 0x01E98D2F, 0x01BE34A1, 0x0158E92A, 0x00BECAD0,
	0xFFFBD9F5, 0xFF225482, 0xFE49437C, 0xFD8A4EAE,
	0xFCFF034F, 0xFCBDD1FC, 0xFCD714AF, 0xFD5276F8, 0xFE2D1765, 0xFF58ABA3,
	0x00BBDB68, 0x0233E6A7, 0x03978981, 0x04BAEA58,
	0x05743DE9, 0x05A0B041, 0x052911BC, 0x0405C139, 0x024158DB, 0xFFF9B8A9,
	0xFD5F317F, 0xFAB1BD13, 0xF83C6537, 0xF64F2CC3,
	0xF537F99E, 0xF53B2194, 0xF68C4CEB, 0xF9486677, 0xFD7137CC, 0x02EB3A41,
	0x097DE706, 0x10D6A0B4, 0x188E0B4A, 0x202F6C6D,
	0x274179DE, 0x2D4FCEC5, 0x31F44009, 0x34DF24B0, 0x35DDE3B3, 0x34DF24AF,
	0x31F44001, 0x2D4FCEC7, 0x274179DE, 0x202F6C6C,
	0x188E0B4E, 0x10D6A0B4, 0x097DE706, 0x02EB3A3E, 0xFD7137D0, 0xF9486677,
	0xF68C4CEB, 0xF53B2194, 0xF537F9A8, 0xF64F2CCD,
	0xF83C6540, 0xFAB1BD19, 0xFD5F3186, 0xFFF9B8A9, 0x024158D9, 0x0405C134,
	0x052911B1, 0x05A0B034, 0x05743DDC, 0x04BAEA52,
	0x0397897C, 0x0233E6A4, 0x00BBDB67, 0xFF58ABA5, 0xFE2D1769, 0xFD527700,
	0xFCD714B3, 0xFCBDD206, 0xFCFF035E, 0xFD8A4EB2,
	0xFE494382, 0xFF225483, 0xFFFBD9F5, 0x00BECACE, 0x0158E922, 0x01BE349D,
	0x01E98D27, 0x01DC88E2, 0x019E95C7, 0x013B9100,
	0x00C20956, 0x004164A7, 0xFFC82014, 0xFF625855, 0xFF18BC27, 0xFEEFFB49,
	0xFEE8B4DD, 0xFEFFD9D9, 0xFF2F6B97, 0xFF6F7855,
	0xFFB73204, 0xFFFDFE23, 0x003C60D7, 0x006CAD46, 0x008B6E1A, 0x009782A8,
	0x0091F66D, 0x007D9F55, 0x005E94D0, 0x003993BC,
	0x0013632E, 0xFFF04B86, 0xFFD3AD22, 0xFFBFBEC2, 0xFFB575E6, 0xFFB496A8,
	0xFFBBE3F5, 0xFFC9667C, 0xFFDABF11, 0xFFED798A,
	0xFFFF566F, 0x000E8432, 0x0019C3A4, 0x00207642, 0x002297FE, 0x0020A81A,
	0x001B864F, 0x00144A1F, 0x000C19F4, 0x000406EA,
	0xFFFCF0E9, 0xFFF774F0, 0xFFF3E63B, 0xFFF25131, 0xFFF2863D, 0xFFF429F1,
	0xFFF6C78F, 0xFFF9E367, 0xFFFD0AB3, 0xFFFFDF8F,
	0x0002202C, 0x0003A94A, 0x0004746D, 0x000492EE, 0x0004270C, 0x00035C3A,
	0x00025FE0, 0x00015B4E, 0x00006F77, 0xFFFFB2A7,
	0xFFFF3010, 0xFFFEE8E2, 0xFFFED67B, 0xFFFEED14, 0xFFFF1E92, 0xFFFF5CF9,
	0xFFFF9C4C, 0xFFFFD3B5, 0xFFFFFDF6, 0x0000193E,
	0x00002684, 0x000028A2, 0x0000235D, 0x00001A80, 0x0000112D, 0x00000974,
	0x0000042D, 0x0000011D, 0xFFFFFF43, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

/*
  Sampling Rate Conversion Filter Coefficient
  Input Sampling Frequency = 16 kHz
  Output Sampling Frequency = 44.1 kHz
  Over-Sampling Rate = 8 times
  31 TAPs
*/
static unsigned long SIU_SRC_16000_44100[YRAMFIRSIZE] = {
	0x00000000, 0x000000E0, 0x000003E1, 0x0000091D, 0x000010E2, 0x00001A6A,
	0x000023B7, 0x000029AB, 0x00002872, 0x00001C2E,
	0x000001D6, 0xFFFFD839, 0xFFFFA0E0, 0xFFFF60CA, 0xFFFF209C, 0xFFFEEC41,
	0xFFFED1D1, 0xFFFEDFB9, 0xFFFF2242, 0xFFFFA0C9,
	0x00005AEF, 0x0001465E, 0x00024D82, 0x00034FDF, 0x00042432, 0x00049C9D,
	0x00048C9F, 0x0003D062, 0x00025483, 0x00001D34,
	0xFFFD4B7D, 0xFFFA1F50, 0xFFF6F576, 0xFFF440A1, 0xFFF27DBA, 0xFFF2240E,
	0xFFF392DE, 0xFFF6FE8B, 0xFFFC5FF1, 0x000368F0,
	0x000B80BD, 0x0013CA07, 0x001B3402, 0x002095EB, 0x0022D314, 0x0021040A,
	0x001A9FE0, 0x000FA003, 0x0000989C, 0xFFEEC03E,
	0xFFDBE2E1, 0xFFCA3EA9, 0xFFBC4B69, 0xFFB4712A, 0xFFB4B4DC, 0xFFBE64CF,
	0xFFD1D00D, 0xFFEE13ED, 0x001109CA, 0x00375C47,
	0x005CC6F1, 0x007C7E3E, 0x0091B814, 0x009847AC, 0x008D3B80, 0x006F68DE,
	0x003FD138, 0x0001CE79, 0xFFBAF99E, 0xFF72C55A,
	0xFF31CFEA, 0xFF00F8A9, 0xFEE84F66, 0xFEEDFAF9, 0xFF1538CE, 0xFF5D9932,
	0xFFC2974A, 0x003BA51B, 0x00BCB7D2, 0x0137520F,
	0x019BFA3E, 0x01DBFB62, 0x01EB42D2, 0x01C22325, 0x015EC3D5, 0x00C60719,
	0x0003BBD2, 0xFF2A0154, 0xFE4FD9B5, 0xFD8EFFB2,
	0xFD012C88, 0xFCBD1224, 0xFCD35A8C, 0xFD4C0770, 0xFE2489C9, 0xFF4EDC28,
	0x00B1D4EE, 0x022AC612, 0x03905DB3, 0x04B69487,
	0x057355F2, 0x05A3707C, 0x052F4CE6, 0x040EE405, 0x024C7767, 0x0005A694,
	0xFD6AA23C, 0xFABB68D4, 0xF8432E59, 0xF652414F,
	0xF536ECF2, 0xF535FAF2, 0xF6838915, 0xF93CEBD1, 0xFD643E32, 0x02DE291F,
	0x09722D80, 0x10CD8C58, 0x1888A217, 0x202E4E9A,
	0x2744CEE9, 0x2D57402A, 0x31FF0232, 0x34EC0D4F, 0x35EB8AEB, 0x34EC0D4D,
	0x31FF022A, 0x2D57402C, 0x2744CEE9, 0x202E4E98,
	0x1888A21B, 0x10CD8C58, 0x09722D80, 0x02DE291C, 0xFD643E35, 0xF93CEBD1,
	0xF6838915, 0xF535FAF3, 0xF536ECFB, 0xF6524159,
	0xF8432E61, 0xFABB68DA, 0xFD6AA243, 0x0005A694, 0x024C7764, 0x040EE400,
	0x052F4CDA, 0x05A3706E, 0x057355E5, 0x04B69481,
	0x03905DAE, 0x022AC60F, 0x00B1D4EE, 0xFF4EDC2A, 0xFE2489CE, 0xFD4C0778,
	0xFCD35A91, 0xFCBD122E, 0xFD012C96, 0xFD8EFFB5,
	0xFE4FD9BA, 0xFF2A0155, 0x0003BBD2, 0x00C60718, 0x015EC3CD, 0x01C22322,
	0x01EB42CA, 0x01DBFB5B, 0x019BFA39, 0x0137520A,
	0x00BCB7CE, 0x003BA51A, 0xFFC2974B, 0xFF5D9934, 0xFF1538D0, 0xFEEDFAFE,
	0xFEE84F6C, 0xFF00F8AF, 0xFF31CFEE, 0xFF72C55D,
	0xFFBAF99F, 0x0001CE79, 0x003FD137, 0x006F68DB, 0x008D3B7C, 0x009847AA,
	0x0091B810, 0x007C7E3B, 0x005CC6EE, 0x00375C45,
	0x001109C9, 0xFFEE13EE, 0xFFD1D00E, 0xFFBE64D0, 0xFFB4B4DE, 0xFFB4712C,
	0xFFBC4B6C, 0xFFCA3EAB, 0xFFDBE2E2, 0xFFEEC03E,
	0x0000989C, 0x000FA002, 0x001A9FDF, 0x00210409, 0x0022D313, 0x002095E9,
	0x001B3401, 0x0013CA06, 0x000B80BD, 0x000368F0,
	0xFFFC5FF1, 0xFFF6FE8B, 0xFFF392DF, 0xFFF2240E, 0xFFF27DBB, 0xFFF440A2,
	0xFFF6F576, 0xFFFA1F51, 0xFFFD4B7D, 0x00001D34,
	0x00025483, 0x0003D062, 0x00048C9F, 0x00049C9D, 0x00042432, 0x00034FDF,
	0x00024D82, 0x0001465E, 0x00005AEF, 0xFFFFA0C9,
	0xFFFF2242, 0xFFFEDFB9, 0xFFFED1D1, 0xFFFEEC41, 0xFFFF209C, 0xFFFF60CA,
	0xFFFFA0E0, 0xFFFFD839, 0x000001D6, 0x00001C2E,
	0x00002872, 0x000029AB, 0x000023B7, 0x00001A6A, 0x000010E2, 0x0000091D,
	0x000003E1, 0x000000E0, 0xFFFFFF0E, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

/*
  Sampling Rate Conversion Filter Coefficient
  Input Sampling Frequency = 22.05 kHz
  Output Sampling Frequency = 44.1 kHz
  Over-Sampling Rate = 8 times
  31 TAPs
*/
static unsigned long SIU_SRC_22050_44100[YRAMFIRSIZE] = {
	0x00000000, 0x000000F7, 0x000003FF, 0x0000093F, 0x00001100, 0x00001A74,
	0x00002396, 0x00002946, 0x000027B4, 0x00001B0B,
	0x00000055, 0xFFFFD677, 0xFFFF9F16, 0xFFFF5F49, 0xFFFF1FC8, 0xFFFEEC88,
	0xFFFED395, 0xFFFEE33C, 0xFFFF2794, 0xFFFFA7B4,
	0x000062E9, 0x00014E88, 0x000254B5, 0x000354C6, 0x00042570, 0x00049902,
	0x00048362, 0x0003C157, 0x00024049, 0x0000054C,
	0xFFFD3249, 0xFFFA07EA, 0xFFF6E369, 0xFFF43784, 0xFFF280AC, 0xFFF23533,
	0xFFF3B2E1, 0xFFF72C3B, 0xFFFC9812, 0x0003A648,
	0x000BBC6B, 0x0013FC25, 0x001B5485, 0x00209DAA, 0x0022BCE4, 0x0020CDBB,
	0x001A4B00, 0x000F3242, 0x00001BB0, 0xFFEE413D,
	0xFFDB7113, 0xFFC9E9E6, 0xFFBC2226, 0xFFB47E76, 0xFFB4FE83, 0xFFBEE9F0,
	0xFFD2885A, 0xFFEEEFC2, 0x0011F33B, 0x00383915,
	0x005D7B48, 0x007CEFF4, 0x0091D215, 0x0097FD20, 0x008C8A40, 0x006E5ADC,
	0x003E7C8B, 0x000053E8, 0xFFB981B7, 0xFF717C4B,
	0xFF30E085, 0xFF00871B, 0xFEE87446, 0xFEEEBF4D, 0xFF16939E, 0xFF5F6F19,
	0xFFC4BC32, 0x003DE022, 0x00BEC90F, 0x0138F996,
	0x019CFFAF, 0x01DC3520, 0x01EA9BF2, 0x01C09F54, 0x015C808D, 0x00C339DB,
	0x0000AD6A, 0xFF270684, 0xFE4D4A0E, 0xFD8D2B83,
	0xFD005323, 0xFCBD5992, 0xFCD4C9CD, 0xFD4E8419, 0xFE27D967, 0xFF52A97D,
	0x00B5B886, 0x022E5162, 0x03932787, 0x04B8454E,
	0x0573B281, 0x05A261E9, 0x052CE47A, 0x040B5AA9, 0x02482871, 0x00010673,
	0xFD6631F1, 0xFAB7A79B, 0xF8408B23, 0xF6510DBE,
	0xF537535C, 0xF537F8B7, 0xF686EDF9, 0xF9415E92, 0xFD6945E6, 0x02E33A5C,
	0x0976B9D1, 0x10D11247, 0x188ABBED, 0x202EBE19,
	0x274384AC, 0x2D545DB5, 0x31FAD676, 0x34E70C05, 0x35E63FAE, 0x34E70C03,
	0x31FAD66E, 0x2D545DB7, 0x274384AC, 0x202EBE18,
	0x188ABBF0, 0x10D11248, 0x0976B9D1, 0x02E33A59, 0xFD6945E9, 0xF9415E91,
	0xF686EDF9, 0xF537F8B7, 0xF5375366, 0xF6510DC8,
	0xF8408B2C, 0xFAB7A7A1, 0xFD6631F8, 0x00010673, 0x0248286E, 0x040B5AA4,
	0x052CE46E, 0x05A261DB, 0x0573B274, 0x04B84548,
	0x03932782, 0x022E515F, 0x00B5B885, 0xFF52A97F, 0xFE27D96C, 0xFD4E8421,
	0xFCD4C9D2, 0xFCBD599C, 0xFD005331, 0xFD8D2B87,
	0xFE4D4A14, 0xFF270686, 0x0000AD6A, 0x00C339DA, 0x015C8085, 0x01C09F51,
	0x01EA9BEA, 0x01DC3519, 0x019CFFAA, 0x0138F990,
	0x00BEC90B, 0x003DE021, 0xFFC4BC33, 0xFF5F6F1B, 0xFF1693A0, 0xFEEEBF51,
	0xFEE8744C, 0xFF008721, 0xFF30E089, 0xFF717C4E,
	0xFFB981B8, 0x000053E8, 0x003E7C8A, 0x006E5AD9, 0x008C8A3C, 0x0097FD1E,
	0x0091D211, 0x007CEFF1, 0x005D7B45, 0x00383913,
	0x0011F33A, 0xFFEEEFC3, 0xFFD2885B, 0xFFBEE9F2, 0xFFB4FE85, 0xFFB47E78,
	0xFFBC2229, 0xFFC9E9E8, 0xFFDB7114, 0xFFEE413D,
	0x00001BB0, 0x000F3241, 0x001A4B00, 0x0020CDBA, 0x0022BCE3, 0x00209DA9,
	0x001B5484, 0x0013FC24, 0x000BBC6A, 0x0003A647,
	0xFFFC9812, 0xFFF72C3C, 0xFFF3B2E2, 0xFFF23533, 0xFFF280AC, 0xFFF43784,
	0xFFF6E36A, 0xFFFA07EA, 0xFFFD3249, 0x0000054C,
	0x00024049, 0x0003C157, 0x00048361, 0x00049901, 0x00042570, 0x000354C6,
	0x000254B5, 0x00014E88, 0x000062E9, 0xFFFFA7B4,
	0xFFFF2794, 0xFFFEE33C, 0xFFFED395, 0xFFFEEC88, 0xFFFF1FC9, 0xFFFF5F49,
	0xFFFF9F16, 0xFFFFD677, 0x00000055, 0x00001B0B,
	0x000027B4, 0x00002946, 0x00002396, 0x00001A74, 0x00001100, 0x0000093F,
	0x000003FF, 0x000000F7, 0xFFFFFF23, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

/*
  Sampling Rate Conversion Filter Coefficient
  Input Sampling Frequency = 24 kHz
  Output Sampling Frequency = 44.1 kHz
  Over-Sampling Rate = 8 times
  31 TAPs
*/
static unsigned long SIU_SRC_24000_44100[YRAMFIRSIZE] = {
	0x00000000, 0x000000E0, 0x000003E1, 0x0000091D, 0x000010E2, 0x00001A6A,
	0x000023B7, 0x000029AB, 0x00002872, 0x00001C2E,
	0x000001D6, 0xFFFFD839, 0xFFFFA0E0, 0xFFFF60CA, 0xFFFF209C, 0xFFFEEC41,
	0xFFFED1D1, 0xFFFEDFB9, 0xFFFF2242, 0xFFFFA0C9,
	0x00005AEF, 0x0001465E, 0x00024D82, 0x00034FDF, 0x00042432, 0x00049C9D,
	0x00048C9F, 0x0003D062, 0x00025483, 0x00001D34,
	0xFFFD4B7D, 0xFFFA1F50, 0xFFF6F576, 0xFFF440A1, 0xFFF27DBA, 0xFFF2240E,
	0xFFF392DE, 0xFFF6FE8B, 0xFFFC5FF1, 0x000368F0,
	0x000B80BD, 0x0013CA07, 0x001B3402, 0x002095EB, 0x0022D314, 0x0021040A,
	0x001A9FE0, 0x000FA003, 0x0000989C, 0xFFEEC03E,
	0xFFDBE2E1, 0xFFCA3EA9, 0xFFBC4B69, 0xFFB4712A, 0xFFB4B4DC, 0xFFBE64CF,
	0xFFD1D00D, 0xFFEE13ED, 0x001109CA, 0x00375C47,
	0x005CC6F1, 0x007C7E3E, 0x0091B814, 0x009847AC, 0x008D3B80, 0x006F68DE,
	0x003FD138, 0x0001CE79, 0xFFBAF99E, 0xFF72C55A,
	0xFF31CFEA, 0xFF00F8A9, 0xFEE84F66, 0xFEEDFAF9, 0xFF1538CE, 0xFF5D9932,
	0xFFC2974A, 0x003BA51B, 0x00BCB7D2, 0x0137520F,
	0x019BFA3E, 0x01DBFB62, 0x01EB42D2, 0x01C22325, 0x015EC3D5, 0x00C60719,
	0x0003BBD2, 0xFF2A0154, 0xFE4FD9B5, 0xFD8EFFB2,
	0xFD012C88, 0xFCBD1224, 0xFCD35A8C, 0xFD4C0770, 0xFE2489C9, 0xFF4EDC28,
	0x00B1D4EE, 0x022AC612, 0x03905DB3, 0x04B69487,
	0x057355F2, 0x05A3707C, 0x052F4CE6, 0x040EE405, 0x024C7767, 0x0005A694,
	0xFD6AA23C, 0xFABB68D4, 0xF8432E59, 0xF652414F,
	0xF536ECF2, 0xF535FAF2, 0xF6838915, 0xF93CEBD1, 0xFD643E32, 0x02DE291F,
	0x09722D80, 0x10CD8C58, 0x1888A217, 0x202E4E9A,
	0x2744CEE9, 0x2D57402A, 0x31FF0232, 0x34EC0D4F, 0x35EB8AEB, 0x34EC0D4D,
	0x31FF022A, 0x2D57402C, 0x2744CEE9, 0x202E4E98,
	0x1888A21B, 0x10CD8C58, 0x09722D80, 0x02DE291C, 0xFD643E35, 0xF93CEBD1,
	0xF6838915, 0xF535FAF3, 0xF536ECFB, 0xF6524159,
	0xF8432E61, 0xFABB68DA, 0xFD6AA243, 0x0005A694, 0x024C7764, 0x040EE400,
	0x052F4CDA, 0x05A3706E, 0x057355E5, 0x04B69481,
	0x03905DAE, 0x022AC60F, 0x00B1D4EE, 0xFF4EDC2A, 0xFE2489CE, 0xFD4C0778,
	0xFCD35A91, 0xFCBD122E, 0xFD012C96, 0xFD8EFFB5,
	0xFE4FD9BA, 0xFF2A0155, 0x0003BBD2, 0x00C60718, 0x015EC3CD, 0x01C22322,
	0x01EB42CA, 0x01DBFB5B, 0x019BFA39, 0x0137520A,
	0x00BCB7CE, 0x003BA51A, 0xFFC2974B, 0xFF5D9934, 0xFF1538D0, 0xFEEDFAFE,
	0xFEE84F6C, 0xFF00F8AF, 0xFF31CFEE, 0xFF72C55D,
	0xFFBAF99F, 0x0001CE79, 0x003FD137, 0x006F68DB, 0x008D3B7C, 0x009847AA,
	0x0091B810, 0x007C7E3B, 0x005CC6EE, 0x00375C45,
	0x001109C9, 0xFFEE13EE, 0xFFD1D00E, 0xFFBE64D0, 0xFFB4B4DE, 0xFFB4712C,
	0xFFBC4B6C, 0xFFCA3EAB, 0xFFDBE2E2, 0xFFEEC03E,
	0x0000989C, 0x000FA002, 0x001A9FDF, 0x00210409, 0x0022D313, 0x002095E9,
	0x001B3401, 0x0013CA06, 0x000B80BD, 0x000368F0,
	0xFFFC5FF1, 0xFFF6FE8B, 0xFFF392DF, 0xFFF2240E, 0xFFF27DBB, 0xFFF440A2,
	0xFFF6F576, 0xFFFA1F51, 0xFFFD4B7D, 0x00001D34,
	0x00025483, 0x0003D062, 0x00048C9F, 0x00049C9D, 0x00042432, 0x00034FDF,
	0x00024D82, 0x0001465E, 0x00005AEF, 0xFFFFA0C9,
	0xFFFF2242, 0xFFFEDFB9, 0xFFFED1D1, 0xFFFEEC41, 0xFFFF209C, 0xFFFF60CA,
	0xFFFFA0E0, 0xFFFFD839, 0x000001D6, 0x00001C2E,
	0x00002872, 0x000029AB, 0x000023B7, 0x00001A6A, 0x000010E2, 0x0000091D,
	0x000003E1, 0x000000E0, 0xFFFFFF0E, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

/*
  Sampling Rate Conversion Filter Coefficient
  Input Sampling Frequency = 32 kHz
  Output Sampling Frequency = 44.1 kHz
  Over-Sampling Rate = 8 times
  31 TAPs
*/
static unsigned long SIU_SRC_32000_44100[YRAMFIRSIZE] = {
	0x00000000, 0x000000E0, 0x000003E1, 0x0000091D, 0x000010E2, 0x00001A6A,
	0x000023B7, 0x000029AB, 0x00002872, 0x00001C2E,
	0x000001D6, 0xFFFFD839, 0xFFFFA0E0, 0xFFFF60CA, 0xFFFF209C, 0xFFFEEC41,
	0xFFFED1D1, 0xFFFEDFB9, 0xFFFF2242, 0xFFFFA0C9,
	0x00005AEF, 0x0001465E, 0x00024D82, 0x00034FDF, 0x00042432, 0x00049C9D,
	0x00048C9F, 0x0003D062, 0x00025483, 0x00001D34,
	0xFFFD4B7D, 0xFFFA1F50, 0xFFF6F576, 0xFFF440A1, 0xFFF27DBA, 0xFFF2240E,
	0xFFF392DE, 0xFFF6FE8B, 0xFFFC5FF1, 0x000368F0,
	0x000B80BD, 0x0013CA07, 0x001B3402, 0x002095EB, 0x0022D314, 0x0021040A,
	0x001A9FE0, 0x000FA003, 0x0000989C, 0xFFEEC03E,
	0xFFDBE2E1, 0xFFCA3EA9, 0xFFBC4B69, 0xFFB4712A, 0xFFB4B4DC, 0xFFBE64CF,
	0xFFD1D00D, 0xFFEE13ED, 0x001109CA, 0x00375C47,
	0x005CC6F1, 0x007C7E3E, 0x0091B814, 0x009847AC, 0x008D3B80, 0x006F68DE,
	0x003FD138, 0x0001CE79, 0xFFBAF99E, 0xFF72C55A,
	0xFF31CFEA, 0xFF00F8A9, 0xFEE84F66, 0xFEEDFAF9, 0xFF1538CE, 0xFF5D9932,
	0xFFC2974A, 0x003BA51B, 0x00BCB7D2, 0x0137520F,
	0x019BFA3E, 0x01DBFB62, 0x01EB42D2, 0x01C22325, 0x015EC3D5, 0x00C60719,
	0x0003BBD2, 0xFF2A0154, 0xFE4FD9B5, 0xFD8EFFB2,
	0xFD012C88, 0xFCBD1224, 0xFCD35A8C, 0xFD4C0770, 0xFE2489C9, 0xFF4EDC28,
	0x00B1D4EE, 0x022AC612, 0x03905DB3, 0x04B69487,
	0x057355F2, 0x05A3707C, 0x052F4CE6, 0x040EE405, 0x024C7767, 0x0005A694,
	0xFD6AA23C, 0xFABB68D4, 0xF8432E59, 0xF652414F,
	0xF536ECF2, 0xF535FAF2, 0xF6838915, 0xF93CEBD1, 0xFD643E32, 0x02DE291F,
	0x09722D80, 0x10CD8C58, 0x1888A217, 0x202E4E9A,
	0x2744CEE9, 0x2D57402A, 0x31FF0232, 0x34EC0D4F, 0x35EB8AEB, 0x34EC0D4D,
	0x31FF022A, 0x2D57402C, 0x2744CEE9, 0x202E4E98,
	0x1888A21B, 0x10CD8C58, 0x09722D80, 0x02DE291C, 0xFD643E35, 0xF93CEBD1,
	0xF6838915, 0xF535FAF3, 0xF536ECFB, 0xF6524159,
	0xF8432E61, 0xFABB68DA, 0xFD6AA243, 0x0005A694, 0x024C7764, 0x040EE400,
	0x052F4CDA, 0x05A3706E, 0x057355E5, 0x04B69481,
	0x03905DAE, 0x022AC60F, 0x00B1D4EE, 0xFF4EDC2A, 0xFE2489CE, 0xFD4C0778,
	0xFCD35A91, 0xFCBD122E, 0xFD012C96, 0xFD8EFFB5,
	0xFE4FD9BA, 0xFF2A0155, 0x0003BBD2, 0x00C60718, 0x015EC3CD, 0x01C22322,
	0x01EB42CA, 0x01DBFB5B, 0x019BFA39, 0x0137520A,
	0x00BCB7CE, 0x003BA51A, 0xFFC2974B, 0xFF5D9934, 0xFF1538D0, 0xFEEDFAFE,
	0xFEE84F6C, 0xFF00F8AF, 0xFF31CFEE, 0xFF72C55D,
	0xFFBAF99F, 0x0001CE79, 0x003FD137, 0x006F68DB, 0x008D3B7C, 0x009847AA,
	0x0091B810, 0x007C7E3B, 0x005CC6EE, 0x00375C45,
	0x001109C9, 0xFFEE13EE, 0xFFD1D00E, 0xFFBE64D0, 0xFFB4B4DE, 0xFFB4712C,
	0xFFBC4B6C, 0xFFCA3EAB, 0xFFDBE2E2, 0xFFEEC03E,
	0x0000989C, 0x000FA002, 0x001A9FDF, 0x00210409, 0x0022D313, 0x002095E9,
	0x001B3401, 0x0013CA06, 0x000B80BD, 0x000368F0,
	0xFFFC5FF1, 0xFFF6FE8B, 0xFFF392DF, 0xFFF2240E, 0xFFF27DBB, 0xFFF440A2,
	0xFFF6F576, 0xFFFA1F51, 0xFFFD4B7D, 0x00001D34,
	0x00025483, 0x0003D062, 0x00048C9F, 0x00049C9D, 0x00042432, 0x00034FDF,
	0x00024D82, 0x0001465E, 0x00005AEF, 0xFFFFA0C9,
	0xFFFF2242, 0xFFFEDFB9, 0xFFFED1D1, 0xFFFEEC41, 0xFFFF209C, 0xFFFF60CA,
	0xFFFFA0E0, 0xFFFFD839, 0x000001D6, 0x00001C2E,
	0x00002872, 0x000029AB, 0x000023B7, 0x00001A6A, 0x000010E2, 0x0000091D,
	0x000003E1, 0x000000E0, 0xFFFFFF0E, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

/*
  Sampling Rate Conversion Filter Coefficient
  Input Sampling Frequency = 44.1 kHz
  Output Sampling Frequency = 44.1 kHz
  Over-Sampling Rate = 8 times
  31 TAPs
*/
static unsigned long SIU_SRC_44100_44100[YRAMFIRSIZE] = {
	0x00000000, 0x000000D6, 0x000003D5, 0x0000090F, 0x000010D6, 0x00001A66,
	0x000023C4, 0x000029D3, 0x000028BE, 0x00001CA3,
	0x00000272, 0xFFFFD8EF, 0xFFFFA19B, 0xFFFF6168, 0xFFFF20F4, 0xFFFEEC28,
	0xFFFED11F, 0xFFFEDE52, 0xFFFF201F, 0xFFFF9E00,
	0x000057B6, 0x0001430E, 0x00024A93, 0x00034DDB, 0x000423A6, 0x00049E05,
	0x0004904D, 0x0003D66A, 0x00025CA6, 0x000026D9,
	0xFFFD55B0, 0xFFFA28D1, 0xFFF6FCD6, 0xFFF4446C, 0xFFF27CAB, 0xFFF21D43,
	0xFFF3860F, 0xFFF6EC2E, 0xFFFC4952, 0x00035027,
	0x000B688F, 0x0013B5A4, 0x001B26AD, 0x0020928A, 0x0022DBC5, 0x002119B8,
	0x001AC1F1, 0x000FCC34, 0x0000CB07, 0xFFEEF39D,
	0xFFDC110A, 0xFFCA6135, 0xFFBC5C7A, 0xFFB46C3F, 0xFFB49790, 0xFFBE2F71,
	0xFFD185EA, 0xFFEDBB4B, 0x0010AB7B, 0x003702E2,
	0x005C7DB6, 0x007C4FC5, 0x0091ACED, 0x0098651A, 0x008D8273, 0x006FD565,
	0x00405A79, 0x00026743, 0xFFBB9197, 0xFF734AAA,
	0xFF32313E, 0xFF012755, 0xFEE84166, 0xFEEDAC91, 0xFF14AD82, 0xFF5CDBFD,
	0xFFC1B9E9, 0x003ABE79, 0x00BBE1BF, 0x0136A662,
	0x019B8FCA, 0x01DBE30D, 0x01EB8529, 0x01C2BECA, 0x015FACFC, 0x00C7283D,
	0x0004F79C, 0xFF2B358D, 0xFE50E2FE, 0xFD8FBD8E,
	0xFD01854F, 0xFCBCF660, 0xFCD2C74A, 0xFD4B073F, 0xFE23342C, 0xFF4D537B,
	0x00B042EF, 0x02295764, 0x038F3CDE, 0x04B5E4F8,
	0x05732FA6, 0x05A3DCCD, 0x053044F2, 0x041050E7, 0x024E344D, 0x00078482,
	0xFD6C6D1D, 0xFABCED40, 0xF8443F76, 0xF652BE19,
	0xF536C43D, 0xF5352DBD, 0xF6822ACB, 0xF93B2071, 0xFD623689, 0x02DC1D77,
	0x09705762, 0x10CC2010, 0x1887C8BF, 0x202E215B,
	0x27455402, 0x2D586A18, 0x3200B116, 0x34EE1270, 0x35EDADE9, 0x34EE126F,
	0x3200B10E, 0x2D586A1A, 0x27455402, 0x202E215A,
	0x1887C8C3, 0x10CC2010, 0x09705762, 0x02DC1D74, 0xFD62368C, 0xF93B2071,
	0xF6822ACB, 0xF5352DBE, 0xF536C447, 0xF652BE23,
	0xF8443F7F, 0xFABCED46, 0xFD6C6D23, 0x00078482, 0x024E344A, 0x041050E2,
	0x053044E6, 0x05A3DCC0, 0x05732F99, 0x04B5E4F2,
	0x038F3CD8, 0x02295761, 0x00B042EE, 0xFF4D537D, 0xFE233431, 0xFD4B0747,
	0xFCD2C74F, 0xFCBCF66A, 0xFD01855E, 0xFD8FBD91,
	0xFE50E304, 0xFF2B358F, 0x0004F79C, 0x00C7283B, 0x015FACF4, 0x01C2BEC7,
	0x01EB8521, 0x01DBE306, 0x019B8FC5, 0x0136A65D,
	0x00BBE1BB, 0x003ABE78, 0xFFC1B9E9, 0xFF5CDC00, 0xFF14AD85, 0xFEEDAC95,
	0xFEE8416C, 0xFF01275B, 0xFF323141, 0xFF734AAD,
	0xFFBB9199, 0x00026743, 0x00405A77, 0x006FD562, 0x008D826F, 0x00986518,
	0x0091ACE9, 0x007C4FC2, 0x005C7DB3, 0x003702E0,
	0x0010AB7A, 0xFFEDBB4B, 0xFFD185EB, 0xFFBE2F72, 0xFFB49792, 0xFFB46C40,
	0xFFBC5C7C, 0xFFCA6137, 0xFFDC110B, 0xFFEEF39E,
	0x0000CB07, 0x000FCC33, 0x001AC1F1, 0x002119B7, 0x0022DBC4, 0x00209289,
	0x001B26AC, 0x0013B5A3, 0x000B688E, 0x00035027,
	0xFFFC4952, 0xFFF6EC2F, 0xFFF38610, 0xFFF21D43, 0xFFF27CAB, 0xFFF4446D,
	0xFFF6FCD6, 0xFFFA28D1, 0xFFFD55B0, 0x000026D9,
	0x00025CA6, 0x0003D66A, 0x0004904D, 0x00049E04, 0x000423A6, 0x00034DDA,
	0x00024A93, 0x0001430E, 0x000057B6, 0xFFFF9E00,
	0xFFFF201F, 0xFFFEDE52, 0xFFFED11F, 0xFFFEEC28, 0xFFFF20F4, 0xFFFF6168,
	0xFFFFA19B, 0xFFFFD8EF, 0x00000272, 0x00001CA3,
	0x000028BE, 0x000029D3, 0x000023C3, 0x00001A66, 0x000010D6, 0x0000090F,
	0x000003D5, 0x000000D6, 0xFFFFFF06, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

/*
  Sampling Rate Conversion Filter Coefficient
  Input Sampling Frequency = 48 kHz
  Output Sampling Frequency = 44.1 kHz
  Over-Sampling Rate = 8 times
  31 TAPs
*/
static unsigned long SIU_SRC_48000_44100[YRAMFIRSIZE] = {
	0x00000000, 0xFFFFFB33, 0xFFFFF80C, 0xFFFFF368, 0xFFFFEE04, 0xFFFFE96D,
	0xFFFFE7E2, 0xFFFFEC00, 0xFFFFF850, 0x00000EB4,
	0x00002FD6, 0x00005A8B, 0x00008B6F, 0x0000BCB2, 0x0000E640, 0x0000FE51,
	0x0000FA5F, 0x0000D095, 0x00007979, 0xFFFFF1C6,
	0xFFFF3C28, 0xFFFE629C, 0xFFFD7725, 0xFFFC9389, 0xFFFBD7F6, 0xFFFB685F,
	0xFFFB68AC, 0xFFFBF7FC, 0xFFFD2B55, 0xFFFF086D,
	0x00018122, 0x0004708E, 0x00079A3C, 0x000AAC41, 0x000D447B, 0x000EF8F5,
	0x000F630D, 0x000E2C8C, 0x000B1D6C, 0x000628C4,
	0xFFFF7716, 0xFFF76C43, 0xFFEEA79F, 0xFFE5FCFC, 0xFFDE6652, 0xFFD8EE47,
	0xFFD694F3, 0xFFD830F8, 0xFFDE4FF6, 0xFFE919D6,
	0xFFF83ABF, 0x000AD746, 0x001F8DE2, 0x00348796, 0x0047984C, 0x00566DC2,
	0x005EC9EF, 0x005EC43A, 0x00550B2E, 0x00411FAC,
	0x002381F8, 0xFFFDC9DF, 0xFFD2A43D, 0xFFA5B297, 0xFF7B4BFA, 0xFF5821EA,
	0xFF40CF3E, 0xFF395A22, 0xFF44B4E3, 0xFF644AEF,
	0xFF97A79B, 0xFFDC4358, 0x002D8181, 0x0084E4F9, 0x00DA7CCA, 0x01258457,
	0x015D2CBC, 0x01797F89, 0x01744690, 0x0149E1CD,
	0x00F9F3B1, 0x0087CD70, 0xFFFA8963, 0xFF5CC7BF, 0xFEBC09F5, 0xFE27B296,
	0xFDAFB914, 0xFD632A55, 0xFD4E956B, 0xFD7A8AF4,
	0xFDEA5549, 0xFE9B0FB5, 0xFF833C5F, 0x0092F02E, 0x01B49F46, 0x02CE86F2,
	0x03C4A2B8, 0x047B0C19, 0x04D8968B, 0x04C971EF,
	0x044196AF, 0x033EBF75, 0x01C9BAC0, 0xFFF6E7D7, 0xFDE5C389, 0xFBBF7AD3,
	0xF9B48FC9, 0xF7F9B15B, 0xF6C3FAAD, 0xF644DF63,
	0xF6A613AD, 0xF805C522, 0xFA73759B, 0xFDEDC27E, 0x02615026, 0x07A8FCC2,
	0x0D8F660C, 0x13D1B078, 0x1A23651C, 0x2033243B,
	0x25AFD93D, 0x2A4E0DD5, 0x2DCCF367, 0x2FFACB82, 0x30B84D03, 0x2FFACB85,
	0x2DCCF361, 0x2A4E0DCC, 0x25AFD93D, 0x20332442,
	0x1A23651F, 0x13D1B077, 0x0D8F660C, 0x07A8FCBA, 0x02615024, 0xFDEDC27F,
	0xFA73759B, 0xF805C521, 0xF6A613B5, 0xF644DF6E,
	0xF6C3FAB7, 0xF7F9B163, 0xF9B48FD8, 0xFBBF7ADD, 0xFDE5C38B, 0xFFF6E7D7,
	0x01C9BABC, 0x033EBF6D, 0x044196A5, 0x04C971E9,
	0x04D89685, 0x047B0C12, 0x03C4A2B3, 0x02CE86EB, 0x01B49F42, 0x0092F02D,
	0xFF833C60, 0xFE9B0FB9, 0xFDEA5553, 0xFD7A8AF8,
	0xFD4E9573, 0xFD632A58, 0xFDAFB920, 0xFE27B29A, 0xFEBC09FC, 0xFF5CC7C0,
	0xFFFA8963, 0x0087CD6E, 0x00F9F3AE, 0x0149E1C8,
	0x01744688, 0x01797F84, 0x015D2CB7, 0x01258452, 0x00DA7CC8, 0x0084E4F7,
	0x002D8180, 0xFFDC4359, 0xFF97A79D, 0xFF644AF3,
	0xFF44B4E6, 0xFF395A26, 0xFF40CF42, 0xFF5821EE, 0xFF7B4BFD, 0xFFA5B298,
	0xFFD2A43E, 0xFFFDC9DF, 0x002381F7, 0x00411FAA,
	0x00550B2C, 0x005EC437, 0x005EC9EE, 0x00566DC0, 0x0047984B, 0x00348794,
	0x001F8DE1, 0x000AD745, 0xFFF83AC0, 0xFFE919D7,
	0xFFDE4FF7, 0xFFD830F9, 0xFFD694F4, 0xFFD8EE48, 0xFFDE6653, 0xFFE5FCFD,
	0xFFEEA79F, 0xFFF76C43, 0xFFFF7716, 0x000628C3,
	0x000B1D6C, 0x000E2C8C, 0x000F630D, 0x000EF8F5, 0x000D447B, 0x000AAC40,
	0x00079A3B, 0x0004708E, 0x00018122, 0xFFFF086D,
	0xFFFD2B56, 0xFFFBF7FC, 0xFFFB68AD, 0xFFFB685F, 0xFFFBD7F7, 0xFFFC9389,
	0xFFFD7725, 0xFFFE629C, 0xFFFF3C28, 0xFFFFF1C6,
	0x00007979, 0x0000D095, 0x0000FA5F, 0x0000FE50, 0x0000E640, 0x0000BCB2,
	0x00008B6E, 0x00005A8B, 0x00002FD6, 0x00000EB4,
	0xFFFFF850, 0xFFFFEC00, 0xFFFFE7E2, 0xFFFFE96D, 0xFFFFEE04, 0xFFFFF369,
	0xFFFFF80C, 0xFFFFFB33, 0xFFFFFCF6, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

/*
 * SIU SPB work
 */
unsigned long ydef[YRAM0SIZE + YRAM1SIZE + YRAM2SIZE + YRAM3SIZE + YRAM4SIZE];
unsigned long fircoef[YRAMFIRSIZE];
unsigned long iircoef[YRAMIIRSIZE];
unsigned long stfifo = 0, trdat = 0;

static inline void
siu_samplingrateconvert(unsigned int infs, unsigned int outfs,
			unsigned int tap, unsigned int over,
			unsigned long *coef)
{
	unsigned int cnt;
	unsigned long *ptr;
	unsigned int tmp;

	tmp = 0x03045000;
	tmp |= (over << 26);
	tmp |= (tap - 4);
	ydef[16] = tmp;
	tmp = (tap * 2 + 1);
	ydef[17] = tmp;
	ydef[20] = outfs;
	ydef[21] = infs;
	ydef[27] = (0x40000000 / outfs);
	ptr = coef;
	for (cnt = 0; cnt < (tap * over + 1); cnt++) {
		fircoef[cnt] = (unsigned int) (*ptr++);
	}
}

static inline void
siu_spb_start(void)
{
	int cnt;
	unsigned long *add;
	unsigned long *ptr;

	/* SPB Program Load to PRAM */
	DPRINTK("Firm writing...\n");
	ptr = (unsigned long *) siu_spb_pro_p0;
	add = (unsigned long *) SIU_P_RAM;
	for (cnt = 0; cnt < PRAM0SIZE; cnt++) {
		*add++ = (*ptr++);
	}

	ptr = (unsigned long *) siu_spb_pro_p1;
	add = (unsigned long *) (SIU_P_RAM + 0x0100);
	for (cnt = 0; cnt < PRAM1SIZE; cnt++) {
		*add++ = (*ptr++);
	}

	/* XRAM */
	add = (unsigned long *) SIU_X_RAM;
	for (cnt = 0; cnt < (XRAM0SIZE + XRAM1SIZE + XRAM2SIZE); cnt++) {
		*add++ = 0x0;
	}

	/* YRAM */
	add = (unsigned long *) SIU_Y_RAM;
	for (cnt = 0;
	     cnt < (YRAM0SIZE + YRAM1SIZE + YRAM2SIZE + YRAM3SIZE + YRAM4SIZE);
	     cnt++) {
		*add++ = ydef[cnt];
	}

	/* YRAM FIR */
	add = (unsigned long *) (SIU_Y_RAM + 0x0200);
	for (cnt = 0; cnt < YRAMFIRSIZE; cnt++) {
		*add++ = fircoef[cnt];
	}

	/* YRAM IIR */
	add = (unsigned long *) (SIU_Y_RAM + 0x0600);
	for (cnt = 0; cnt < YRAMIIRSIZE; cnt++) {
		*add++ = iircoef[cnt];
	}

	ctrl_outl(stfifo, STFIFO);
	ctrl_outl(trdat, TRDAT);

	ctrl_outl(0x00000000, SBACTIV);	/* SPB software start */
	ctrl_outl(0xc0000000, SBCTL);	/* SPB program active */
	for (cnt = 0; cnt < 0x00010000; cnt++) {
		if (ctrl_inl(SBCTL) == 0x80000000)
			break;
	}
	if (cnt == 0x00010000) {
		DPRINTK("SPB Start error ???\n");
		return;
	}

	ctrl_outl(0x00400000, SBPSET);	/* SPB program start address */
	ctrl_outl(0xc0000000, SBACTIV);	/* SPB hardware start */

	DPRINTK("SPB Start.\n");

	return;
}

static inline void
siu_spb_stop(void)
{
	ctrl_outl(0x00000000, SBACTIV);
	ctrl_outl(0x00000000, SBCTL);	/* SPB program stop */

	DPRINTK("SPB Stop.\n");
}

static inline int
siu_set_rate(unsigned long rate)
{
	DPRINTK("sampling rate=%d(Hz)\n", (int) rate);

	siu_spb_stop();

	switch (rate) {
	case 8000:		/* 8 KHz */
		siu_samplingrateconvert(8000, 44100, 31, 8, SIU_SRC_8000_44100);
		break;

	case 11025:		/* 11.025 KHz */
		siu_samplingrateconvert(11025, 44100, 31, 8,
					SIU_SRC_11025_44100);
		break;

	case 12000:		/* 12 KHz */
		siu_samplingrateconvert(12000, 44100, 31, 8,
					SIU_SRC_12000_44100);
		break;

	case 16000:		/* 16 KHz */
		siu_samplingrateconvert(16000, 44100, 31, 8,
					SIU_SRC_16000_44100);
		break;

	case 22050:		/* 22.05 KHz */
		siu_samplingrateconvert(22050, 44100, 31, 8,
					SIU_SRC_22050_44100);
		break;

	case 24000:		/* 24 KHz */
		siu_samplingrateconvert(24000, 44100, 31, 8,
					SIU_SRC_24000_44100);
		break;

	case 32000:		/* 32 KHz */
		siu_samplingrateconvert(32000, 44100, 31, 8,
					SIU_SRC_32000_44100);
		break;

	case 44100:		/* 44.1 KHz */
		siu_samplingrateconvert(44100, 44100, 31, 8,
					SIU_SRC_44100_44100);
		break;

	case 48000:		/* 48 KHz */
		siu_samplingrateconvert(48000, 44100, 31, 8,
					SIU_SRC_48000_44100);
		break;

	default:
		siu_spb_start();
		return -1;
	}

	siu_spb_start();

	return 0;
}

void
siu_int_hdr(int irq, void *dev_id, struct pt_regs *regs)
{
	DPRINTK("siu INT\n");

	ctrl_outl(EVNTC, 0x00000011);
}

static int
siu_open(struct inode *inode, struct file *file)
{
	struct sh_siu_card *card = siu_devs;

	DPRINTK("open\n");

	if (file->f_mode & FMODE_WRITE) {
		if (card->w_openCnt > 0) {
			return -EBUSY;
		}
	}

	if (file->f_mode & FMODE_WRITE) {
		card->w_openCnt++;
	}

	MOD_INC_USE_COUNT;

	return 0;
}

static ssize_t
siu_write(struct file *file, const char *buf, size_t count, loff_t * ppos)
{
	struct sh_siu_card *card = siu_devs;
	int ct = (int) count;
	int dmatsz = 0;

	DPRINTK("size=%d\n", count);

	if (!access_ok(VERIFY_READ, buf, count)) {
		return -EFAULT;
	}

	while (ct > 0) {
		unsigned int *p = get_free_dmabank(&card->tx_buff,
						   SIU_DMABUF_SIZE);

		if (p == 0) {
			DPRINTK("sleep on.\n");
			interruptible_sleep_on(&card->wq);

			if (signal_pending(current)) {
				break;
			}
		} else {
			dmatsz = 0;
			while ((dmatsz < SIU_DMABUF_NUM) && (ct > 0)) {
				unsigned int ld, d0, d1, d2, d3;

				if (card->channel == CHANNEL_STEREO) {
					__get_user(d0, buf);
					__get_user(d1, buf + 1);
					__get_user(d2, buf + 2);
					__get_user(d3, buf + 3);
					ld = (d0 << 16) + (d1 << 24) +
					    (d2) + (d3 << 8);

					ct -= 4;
					buf += 4;
				} else {
					/* mono */
					__get_user(d0, buf);
					__get_user(d1, buf + 1);
					ld = (d0 << 16) + (d1 << 24);

					ct -= 2;
					buf += 2;
				}

				p[dmatsz] = ld;
				dmatsz++;
			}

			write_size_dmabank(&card->tx_buff, dmatsz);

			DPRINTK(" W:%08x,%d\n", (int) p, ct);
		}

		if (((ctrl_inl(CHCR4) & CHCR_DE) == 0)
		    && ((ctrl_inl(CHCR4) & CHCR_TE) == 0)) {
			int size;
			DPRINTK("DMA start\n");
			p = get_data_dmabank(&card->tx_buff, &size);
			if (p != 0) {
				tx_start_dma4(p, size);
			}
		}
	}

	return count;
}

static inline int
i2c_send_data(unsigned char *p)
{
	unsigned char data[256];
	volatile unsigned char size;
	int i;

	if (get_user(size, p)) {
		return -EFAULT;
	}
	for (i = 0; i < size; i++) {
		if (get_user(data[i], p + i + 1)) {
			return -EFAULT;
		}
	}
	i2c_write(data, size);
	mdelay(1);
	return 0;
}

static int
siu_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	  unsigned long arg)
{
	struct sh_siu_card *card = siu_devs;
	volatile int val = 0;

	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *) arg);

	case SNDCTL_DSP_RESET:
		if (file->f_mode & FMODE_WRITE) {
			tx_stop_dma4();
		}

		return 0;

	case SNDCTL_DSP_SYNC:
		if (file->f_mode & FMODE_WRITE) {
			if ((ctrl_inl(CHCR4) & CHCR_DE)) {
				interruptible_sleep_on(&card->syncq);
			}
		}
		return 0;

	case SNDCTL_DSP_SPEED:	/* set smaple rate */
		if (get_user(val, (int *) arg)) {
			return -EFAULT;
		}

		if (siu_set_rate(val) != 0) {
			DPRINTK("not support this sampling rate %d\n", val);
		} else {
			card->smp_rate = val;
		}
		return put_user(card->smp_rate, (int *) arg);

	case SNDCTL_DSP_STEREO:	/* set stereo or mono channel */
		if (get_user(val, (int *) arg)) {
			return -EFAULT;
		}

		if (file->f_mode & FMODE_WRITE) {
			tx_stop_dma4();
			if (val) {
				DPRINTK("Stereo.\n");
				card->channel = CHANNEL_STEREO;
				ctrl_outl(ctrl_inl(IFCTL) & ~0x00000040, IFCTL);
			} else {
				DPRINTK("Nomo.\n");
				card->channel = CHANNEL_MONO;
				ctrl_outl(ctrl_inl(IFCTL) | 0x00000040, IFCTL);
			}
		}
		return 0;

	case SNDCTL_DSP_GETBLKSIZE:
		val = (SIU_DMABUF_SIZE * MAX_DMA_BANK);
		return put_user(val, (int *) arg);

	case SNDCTL_DSP_GETFMTS:	/* Returns a mask of supported
					 * sample format */
		return put_user(AFMT_S16_LE, (int *) arg);

	case SNDCTL_DSP_SETFMT:	/* Select sample format */
		if (get_user(val, (int *) arg)) {
			return -EFAULT;
		}

		if (val != AFMT_QUERY) {
			if (file->f_mode & FMODE_WRITE) {
				tx_stop_dma4();
			}
		}

		return put_user(AFMT_S16_LE, (int *) arg);

	case SNDCTL_DSP_CHANNELS:
		if (get_user(val, (int *) arg)) {
			return -EFAULT;
		}

		if (val != 0) {
			if (file->f_mode & FMODE_WRITE) {
				tx_stop_dma4();
			}
		}
		return put_user((volatile int) 2, (int *) arg);

	case SNDCTL_DSP_POST:
		/* FIXME: the same as RESET ?? */
		return 0;

	case SNDCTL_DSP_SUBDIVIDE:
		return 0;

	case SNDCTL_DSP_SETFRAGMENT:
		return 0;

	case SNDCTL_DSP_NONBLOCK:
		file->f_flags |= O_NONBLOCK;
		return 0;

	case SNDCTL_DSP_GETCAPS:
		return 0;

	case SNDCTL_DSP_SETDUPLEX:
		return -EINVAL;

	case SOUND_PCM_READ_RATE:
		return put_user(card->smp_rate, (int *) arg);

	case SOUND_PCM_READ_CHANNELS:
		return put_user((card->channel & CHANNEL_STEREO) ? 2 : 1,
				(int *) arg);

	case SOUND_PCM_READ_BITS:
		return put_user(AFMT_S16_LE, (int *) arg);

	case SNDCTL_DSP_GETOSPACE:
		{
			audio_buf_info abinfo;
			
			if (!(file->f_mode & FMODE_WRITE))
				return -EINVAL;
			abinfo.fragsize = SIU_DMABUF_NUM;
			abinfo.bytes = SIU_DMABUF_SIZE;
			abinfo.fragstotal = MAX_DMA_BANK;
			abinfo.fragments = MAX_DMA_BANK;
			return copy_to_user((void *) arg, &abinfo,
					    sizeof (abinfo)) ? -EFAULT : 0;
		}

#ifndef	I2CRELEASE
	case I2C_WRITE:
		return i2c_send_data((unsigned char *) arg);
#endif

	case TCGETS:
		return -EINVAL;

	default:
		DPRINTK("unknown ioctl 0x%x\n", cmd);
		return -EINVAL;
	}

	DPRINTK("unimplemented ioctl 0x%x\n", cmd);

	return -EINVAL;
}

static int
siu_release(struct inode *inode, struct file *file)
{
	struct sh_siu_card *card = siu_devs;

	DPRINTK("release\n");

	if (file->f_mode & FMODE_WRITE) {
		tx_stop_dma4();
		init_dmabuff(&card->tx_buff);
		card->w_openCnt--;
		if (card->w_openCnt < 0) {
			card->w_openCnt = 0;
		}
	}

	MOD_DEC_USE_COUNT;

	return 0;
}

static struct file_operations sh_siu_fops = {
      write:siu_write,
      ioctl:siu_ioctl,
      open:siu_open,
      release:siu_release,
};

static inline void
init_sbp(void)
{
	/*
	 * SIU firmware write
	 */
	unsigned long cnt, n;

	ctrl_outl(ctrl_inl(MSTPCR2) & ~0x00000100, MSTPCR2);	/* SIU
								 * Operates */
	ctrl_outl(0x00000000, SRCTL);	/* SIU software reset */

	mdelay(1);

	for (cnt = 0, n = 0; cnt < YRAM0SIZE; cnt++) {
		ydef[cnt] = siu_spb_pro_y0[n++];
	}
	for (n = 0; cnt < (YRAM0SIZE + YRAM1SIZE); cnt++) {
		ydef[cnt] = siu_spb_pro_y1[n++];
	}
	for (n = 0; cnt < (YRAM0SIZE + YRAM1SIZE + YRAM2SIZE); cnt++) {
		ydef[cnt] = siu_spb_pro_y2[n++];
	}
	for (n = 0; cnt < (YRAM0SIZE + YRAM1SIZE + YRAM2SIZE + YRAM3SIZE);
	     cnt++) {
		ydef[cnt] = siu_spb_pro_y3[n++];
	}
	for (n = 0;
	     cnt < (YRAM0SIZE + YRAM1SIZE + YRAM2SIZE + YRAM3SIZE + YRAM4SIZE);
	     cnt++) {
		ydef[cnt] = siu_spb_pro_y4[n++];
	}
	for (cnt = 0; cnt < YRAMFIRSIZE; cnt++) {
		fircoef[cnt] = SIU_SRC_44100_44100[cnt];
	}
	for (cnt = 0; cnt < YRAMIIRSIZE; cnt++) {
		iircoef[cnt] = 0x00;
	}
	stfifo = 0x00;
	trdat = 0x00;

	/*
	 * SIU Initialize
	 */
	ctrl_outl(0x00000301, SRCTL);	/* portA, portB, SIU operate */

	ctrl_outl(0x00c00000, CKCTL);	/* portA=512fs, portB=64fs */
	ctrl_outl(0x00000000, BRGASEL);
	ctrl_outl(0x00000000, BRRA);	/* portA  44.1kHz */
	ctrl_outl(0x00000001, BRGBSEL);
	ctrl_outl(0x00000015, BRRB);	/* portB 44.1kHz(44.1kHz
					 * 512fs)/(64*2*(21+1))=8.018kHz
					 * 64fs */
	ctrl_outl(0x40660000, IFCTL);
#if DEFAULT_SIU_CHANNEL == CHANNEL_MONO
	ctrl_outl(ctrl_inl(IFCTL) | 0x00000040, IFCTL);
#endif
	ctrl_outl(0x0c0c0000, SFORM);	/* portA 32bt/fs, portB 32bt/fs */

	/* PORT A, B Setting */
	trdat |= 0x0f0f0000;
	stfifo |= 0x0f0a0f2a;

	ydef[0] = 0x000a0309;
	ydef[1] = siu_spb_pro_y0[1];
	ydef[2] = 0x10000000;
	ydef[3] = siu_spb_pro_y0[3];
	ydef[4] = siu_spb_pro_y0[4];
	ydef[5] = 0x00010901;
	ydef[6] = 0x00020000;
	ydef[7] = 0x00080000;

	/* PCM Data pack setting */
	ctrl_outl(ctrl_inl(DPAK) & ~0xc0000000, DPAK);

	/* Sampling rate setting & start SBP */
	siu_set_rate(DEFAULT_SIU_RATE);
}

static inline void
free_siu_buff(void)
{
	int i;
	struct sh_siu_card *card = siu_devs;

	DPRINTK("free siu buff.\n");

	for (i = 0; i < MAX_DMA_BANK; i++) {
		if (card->tx_buff.p_buff[i] != 0) {
			kfree(P1SEGADDR(card->tx_buff.p_buff[i]));
		}
	}
}

static inline void
siu_init_hw(void)
{
#ifdef INITPIN
	/*
	 * PFC setup
	 */

	ctrl_outw(ctrl_inw(PSELE) | 0x000f, PSELE);
	ctrl_outw(0x0000, PKCR);
	ctrl_outw(ctrl_inw(SCPCR) & ~0x3000, SCPCR);
	ctrl_outw(ctrl_inw(HIZCRA) & ~0x0020, HIZCRA);
	ctrl_outw(ctrl_inw(DRVCR) & ~0x0100, DRVCR);

	ctrl_outl(ctrl_inl(MSTPCR0) & ~0x00600000, MSTPCR0);	/* DMAC,
								 * INTC
								 * Operates */
	ctrl_outl(ctrl_inl(MSTPCR2) | 0x00000100, MSTPCR2);	/* Clock
								 * supply
								 * to
								 * SIU
								 * halted */
	ctrl_outl(0x000000a0, SCLKACR);	/* External input clock */
#endif				/* INITPIN */

	init_sbp();

	/* DMA registars */
	ctrl_outw(0x00b1, DMARS2);	/* DMA4=siu_tx */
	ctrl_outw(0x0301, DMAOR);	/* Round robin, All channel enable */
	ctrl_outl(0x00001814, CHCR4);
	ctrl_outl(DMAOA & DMAAMASK, DAR4);

	return;
}

static inline int
siu_init(void)
{
	int i;

	DPRINTK("init\n");

	struct sh_siu_card *card;
	unsigned int *tmp_buf;

	if ((card = kmalloc(sizeof (struct sh_siu_card), GFP_KERNEL)) == NULL) {
		return -ENOMEM;
	}

	memset(card, 0, sizeof (*card));
	card->siu_iobase = SIU_REG_BASE;
	card->channel = DEFAULT_SIU_CHANNEL;
	card->smp_rate = DEFAULT_SIU_RATE;
	siu_devs = card;

	for (i = 0; i < MAX_DMA_BANK; i++) {
		card->tx_buff.p_buff[i] = 0;
	}

	/* malloc buffer */
	for (i = 0; i < MAX_DMA_BANK; i++) {
		if ((tmp_buf = kmalloc(SIU_DMABUF_SIZE, GFP_KERNEL)) != NULL) {
			card->tx_buff.p_buff[i] = P2SEGADDR(tmp_buf);
		} else {
			printk("no memory(tx)\n");
			free_siu_buff();
			return -ENOMEM;
		}
	}
	init_dmabuff(&card->tx_buff);

	/* initialize queue */
	init_waitqueue_head(&card->wq);
	init_waitqueue_head(&card->syncq);

	/* Claim iospace */
	request_region(card->siu_iobase, SIU_REG_SIZE, "sh_siu");

	/* IRQ set */
	if (request_irq(SIU_IRQ, siu_int_hdr, SA_INTERRUPT, "sh_siu", card)) {
		printk(KERN_ERR "Cannot allocate SIU_IRQ\n");
		free_siu_buff();
		release_region(card->siu_iobase, SIU_REG_SIZE);
		kfree(card);
		return -ENODEV;
	}

	if (request_irq
	    (DMTE4_IRQ, dma4_transfer_end_intr, SA_INTERRUPT, "dmac4", card)) {
		printk(KERN_ERR "Cannot allocate DMTE4_IRQ\n");
		free_siu_buff();
		release_region(card->siu_iobase, SIU_REG_SIZE);
		free_irq(SIU_IRQ, card);
		kfree(card);
		return -ENODEV;
	}
	/* Register /dev/dsp1 */
	if ((card->dev_audio = register_sound_dsp(&sh_siu_fops, -1)) < 0) {
		printk(KERN_ERR "couldn't register DSP device!\n");
		free_siu_buff();
		release_region(card->siu_iobase, SIU_REG_SIZE);
		free_irq(SIU_IRQ, card);
		free_irq(DMTE4_IRQ, card);
		kfree(card);
		return -ENODEV;
	}

	siu_init_hw();

	return 0;
}

static inline void
siu_exit(void)
{
	int i;
	struct sh_siu_card *card = siu_devs;

	DPRINTK("exit\n");

	release_region(card->siu_iobase, SIU_REG_SIZE);
	free_irq(DMTE4_IRQ, card);
	free_irq(SIU_IRQ, card);

	unregister_sound_dsp(card->dev_audio);
	for (i = 0; i < MAX_DMA_BANK; i++) {
		kfree(P1SEGADDR(card->tx_buff.p_buff[i]));
	}
	kfree(card);
}

/*****************************************************************************
 * SIOF
 ****************************************************************************/
/* SIOF registers */
#define SIOF_IOBASE	0xa4410000
#define	SIMDR	(SIOF_IOBASE + 0x00)
#define	SISCR	(SIOF_IOBASE + 0x02)
#define SITDAR	(SIOF_IOBASE + 0x04)
#define	SIRDAR	(SIOF_IOBASE + 0x06)
#define	SICDAR	(SIOF_IOBASE + 0x08)
#define	SICTR	(SIOF_IOBASE + 0x0c)
#define	SIFCTR	(SIOF_IOBASE + 0x10)
#define	SISTR	(SIOF_IOBASE + 0x14)
#define	SIIER	(SIOF_IOBASE + 0x16)
#define	SITDR	(SIOF_IOBASE + 0x20)
#define	SIRDR	(SIOF_IOBASE + 0x24)
#define	SITCR	(SIOF_IOBASE + 0x28)
#define	SIRCR	(SIOF_IOBASE + 0x2c)
#define	SPICR	(SIOF_IOBASE + 0x30)

#define SIOF_REG_SIZE	45

#define	STR_TCRDY	0x4000
#define	STR_TFEMP	0x2000
#define	STR_TDREQ	0x1000
#define	STR_RCRDY	0x0400
#define	STR_RFFUL	0x0200
#define	STR_RDREQ	0x0100
#define	STR_SAERR	0x0020
#define	STR_FSERR	0x0010
#define	STR_TFOVF	0x0008
#define	STR_TFUDF	0x0004
#define	STR_RFUDF	0x0002
#define	STR_RFOVF	0x0001

#define	CTR_TERE	0x0300
#define	CTR_TXRXRST	0x0003
#define	CTR_FSE		0x4000
#define	CTR_MST		0xc000
#define	CTR_TXE		0x0200
#define	CTR_RXE		0x0100

#define IER_TDMAE	(1 << 15)
#define IER_TDREQE	(1 << 12)
#define IER_RDMAE	(1 << 11)
#define IER_RDREQE	(1 << 8)

#define STR_TFUDR	(1 << 2)
#define STR_RFOVR	(1 << 0)

#define SIOF_S_RATE	8000	/* SIOF Sampling rate 8KHz only */

static void
siof_er_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned short status;

	status = ctrl_inw(SISTR);
	ctrl_outw(status, SISTR);

	DPRINTK("status = %04x\n", status);
}

static int
siof_open(struct inode *inode, struct file *file)
{
	struct sh_siof_card *card = siof_devs;

	DPRINTK("open\n");

	if (file->f_mode & FMODE_READ) {
		if (card->r_openCnt > 0) {
			return -EBUSY;
		}
	}

	if (file->f_mode & FMODE_WRITE) {
		if (card->w_openCnt > 0) {
			return -EBUSY;
		}
	}

	if (file->f_mode & FMODE_READ) {
		card->r_openCnt++;
	}

	if (file->f_mode & FMODE_WRITE) {
		card->w_openCnt++;
	}

	MOD_INC_USE_COUNT;

	return 0;
}

static ssize_t
siof_read(struct file *file, char *buf, size_t count, loff_t * ppos)
{
	struct sh_siof_card *card = siof_devs;
	int sc = 0;

	if (card->channel == CHANNEL_STEREO) {
		read_cnt = count;
	} else {
		read_cnt = count * 2;
	}

	dma_read_cnt = read_cnt;

	DPRINTK("size=%d\n", count);

	if (!access_ok(VERIFY_WRITE, buf, count)) {
		return -EFAULT;
	}

	while (read_cnt > 0) {
		int size;
		unsigned int *p = get_data_dmabank(&card->rx_buff, &size);

		if (p == 0) {
			if (((ctrl_inl(CHCR2) & CHCR_DE) == 0)
			    && ((ctrl_inl(CHCR2) & CHCR_TE) == 0)) {
				DPRINTK("DMA start\n");
				if (read_cnt < SIOF_DMABUF_SIZE) {
					size = read_cnt;
				} else {
					size = SIOF_DMABUF_SIZE;
				}
				p = get_free_dmabank(&card->rx_buff, size);
				if (p != 0) {
					dma_read_cnt -= size;
					rx_start_dma2(p, size / sizeof (int));

					DPRINTK("SIOF RX start\n");
					ctrl_outw((ctrl_inw(SICTR) | CTR_RXE),
						  SICTR);
					/* enable SIOF receive */
					ctrl_outw((ctrl_inw(SIIER) |
						   IER_RDREQE), SIIER);
					/* enable
					 * receive
					 * interrupt */
				} else {
					DPRINTK("buffer empty.\n");
				}
			}
			if (file->f_flags & O_NONBLOCK) {
				DPRINTK("-EAGAIN.\n");
				return -EAGAIN;
			} else {
				DPRINTK("sleep on.\n");
				wait_event_interruptible(card->rq,
							 rdma_end != 0);
				if (signal_pending(current)) {
					break;
				}
			}
		} else {
			while (size > 0 && read_cnt > 0) {
				unsigned int ld = *p;

				if (card->channel == CHANNEL_STEREO) {
					__put_user(ld >> 16, buf);
					__put_user(ld >> 24, buf + 1);
					__put_user(ld >> 16, buf + 2);
					__put_user(ld >> 24, buf + 3);

					buf += 4;
					sc += 4;
				} else {
					__put_user(ld >> 16, buf);
					__put_user(ld >> 24, buf + 1);

					buf += 2;
					sc += 2;
				}

				size -= sizeof (int);
				read_cnt -= sizeof (int);
				p++;
			}

			free_dmabank(&card->rx_buff);

			DPRINTK(" R:%08x,%d\n", (int) p, read_cnt);

			if (file->f_flags & O_NONBLOCK) {
				break;
			}
		}

	}

	return sc;
}

static ssize_t
siof_write(struct file *file, const char *buf, size_t count, loff_t * ppos)
{
	struct sh_siof_card *card = siof_devs;
	int ct = (int) count;
	int dmatsz = 0;

	DPRINTK("size=%d\n", count);

	if (!access_ok(VERIFY_READ, buf, count)) {
		return -EFAULT;
	}

	while (ct > 0) {
		unsigned int *p = get_free_dmabank(&card->tx_buff,
						   SIOF_DMABUF_SIZE);

		if (p == 0) {
			if (file->f_flags & O_NONBLOCK) {
				break;
			} else {
				DPRINTK("sleep on.\n");
				interruptible_sleep_on(&card->wq);

				if (signal_pending(current)) {
					break;
				}
			}
		} else {
			dmatsz = 0;
			while ((dmatsz < SIOF_DMABUF_NUM) && (ct > 0)) {
				unsigned int ld, d0, d1, d2, d3;

				if (card->channel == CHANNEL_STEREO) {
					__get_user(d0, buf);
					__get_user(d1, buf + 1);
					__get_user(d2, buf + 2);
					__get_user(d3, buf + 3);
					ld = (d0 << 16) + (d1 << 24) +
					    (d2) + (d3 << 8);

					ct -= 4;
					buf += 4;
				} else {
					/* mono */
					__get_user(d0, buf);
					__get_user(d1, buf + 1);
					ld = (d0 << 16) + (d1 << 24);

					ct -= 2;
					buf += 2;
				}

				p[dmatsz] = ld;
				dmatsz++;
			}

			write_size_dmabank(&card->tx_buff, dmatsz);

			DPRINTK(" W:%08x,%d\n", (int) p, ct);
		}

		if (((ctrl_inl(CHCR3) & CHCR_DE) == 0)
		    && ((ctrl_inl(CHCR3) & CHCR_TE) == 0)) {
			int size;
			DPRINTK("DMA start\n");
			p = get_data_dmabank(&card->tx_buff, &size);
			if (p != 0) {
				tx_start_dma3(p, size);

				DPRINTK("SIOF TX start\n");
				ctrl_outw((ctrl_inw(SICTR) | CTR_TXE), SICTR);
				/* enable SIOF transmit */
				ctrl_outw((ctrl_inw(SIIER) | IER_TDREQE),
					  SIIER);
				/* enable transmit interrupt */
			}
		}
	}

	return count - ct;
}

static int
siof_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	struct sh_siof_card *card = siof_devs;
	volatile int val = 0;

	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *) arg);

	case SNDCTL_DSP_RESET:
		if (file->f_mode & FMODE_WRITE) {
			tx_stop_dma3();
		}

		if (file->f_mode & FMODE_READ) {
			rx_stop_dma2();
		}

		return 0;

	case SNDCTL_DSP_SYNC:
		if (file->f_mode & FMODE_WRITE) {
			if ((ctrl_inl(CHCR3) & CHCR_DE)) {
				interruptible_sleep_on(&card->syncq);
			}
		}
		return 0;

	case SNDCTL_DSP_SPEED:	/* set smaple rate */
		return put_user(SIOF_S_RATE, (int *) arg);

	case SNDCTL_DSP_STEREO:	/* set stereo or mono channel */
		if (get_user(val, (int *) arg)) {
			return -EFAULT;
		}

		if (file->f_mode & FMODE_WRITE) {
			tx_stop_dma3();
			if (val) {
				DPRINTK("Stereo.\n");
				card->channel = CHANNEL_STEREO;
			} else {
				DPRINTK("Nomo.\n");
				card->channel = CHANNEL_MONO;
			}
		}

		if (file->f_mode & FMODE_READ) {
			rx_stop_dma2();
			if (val) {
				DPRINTK("Stereo.\n");
				card->channel = CHANNEL_STEREO;
			} else {
				DPRINTK("Nomo.\n");
				card->channel = CHANNEL_MONO;
			}
		}
		return 0;

	case SNDCTL_DSP_GETBLKSIZE:
		val = (SIOF_DMABUF_SIZE * MAX_DMA_BANK);
		return put_user(val, (int *) arg);

	case SNDCTL_DSP_GETFMTS:	/* Returns a mask of supported
					 * sample format */
		return put_user(AFMT_S16_LE, (int *) arg);

	case SNDCTL_DSP_SETFMT:	/* Select sample format */
		if (get_user(val, (int *) arg)) {
			return -EFAULT;
		}

		if (val != AFMT_QUERY) {
			if (file->f_mode & FMODE_WRITE) {
				tx_stop_dma3();
			}
			if (file->f_mode & FMODE_READ) {
				rx_stop_dma2();
			}
		}

		return put_user(AFMT_S16_LE, (int *) arg);

	case SNDCTL_DSP_CHANNELS:
		if (get_user(val, (int *) arg)) {
			return -EFAULT;
		}

		if (val != 0) {
			if (file->f_mode & FMODE_WRITE) {
				tx_stop_dma3();
			}
			if (file->f_mode & FMODE_READ) {
				rx_stop_dma2();
			}
		}
		return put_user(2, (int *) arg);

	case SNDCTL_DSP_POST:
		/* FIXME: the same as RESET ?? */
		return 0;

	case SNDCTL_DSP_SUBDIVIDE:
		return 0;

	case SNDCTL_DSP_SETFRAGMENT:
		return 0;

	case SNDCTL_DSP_NONBLOCK:
		file->f_flags |= O_NONBLOCK;
		return 0;

	case SNDCTL_DSP_GETCAPS:
		return 0;

	case SNDCTL_DSP_SETDUPLEX:
		return -EINVAL;

	case SOUND_PCM_READ_RATE:
		return put_user(SIOF_S_RATE, (int *) arg);

	case SOUND_PCM_READ_CHANNELS:
		return put_user((card->channel & CHANNEL_STEREO) ? 2 : 1,
				(int *) arg);

	case SOUND_PCM_READ_BITS:
		return put_user(AFMT_S16_LE, (int *) arg);

#ifndef	I2CRELEASE
	case I2C_WRITE:
		return i2c_send_data((unsigned char *) arg);
#endif

	case TCGETS:
		return -EINVAL;

	case SNDCTL_DSP_GETOSPACE:
	case SNDCTL_DSP_GETISPACE:
		{
			audio_buf_info abinfo;

			abinfo.fragsize = SIOF_DMABUF_NUM;
			abinfo.bytes = SIOF_DMABUF_SIZE;
			abinfo.fragstotal = MAX_DMA_BANK;
			abinfo.fragments = MAX_DMA_BANK;
			return copy_to_user((void *) arg, &abinfo,
					    sizeof (abinfo)) ? -EFAULT : 0;
		}
		break;
		
	default:
		DPRINTK("unknown ioctl 0x%x\n", cmd);
		return -EINVAL;
	}

	DPRINTK("unimplemented ioctl 0x%x\n", cmd);

	return -EINVAL;
}

static int
siof_release(struct inode *inode, struct file *file)
{
	struct sh_siof_card *card = siof_devs;

	DPRINTK("release\n");

	if (file->f_mode & FMODE_READ) {
		rx_stop_dma2();
		read_cnt = 0;
		init_dmabuff(&card->rx_buff);
		card->r_openCnt--;
		if (card->r_openCnt < 0) {
			card->r_openCnt = 0;
		}
	}

	if (file->f_mode & FMODE_WRITE) {
		tx_stop_dma3();
		init_dmabuff(&card->tx_buff);
		card->w_openCnt--;
		if (card->w_openCnt < 0) {
			card->w_openCnt = 0;
		}
	}

	MOD_DEC_USE_COUNT;

	return 0;
}

static unsigned int
siof_poll(struct file *file, poll_table * wait)
{
	struct sh_siof_card *card = siof_devs;
	unsigned int size, mask;
	int n;

	DPRINTK("select\n");

	mask = 0;

	poll_wait(file, &(card->rq), wait);
	if (get_data_dmabank(&card->rx_buff, &size))
		mask |= POLLIN | POLLRDNORM;

	poll_wait(file, &(card->wq), wait);
	n = card->tx_buff.wp + 1;
	if (n >= MAX_DMA_BANK)
		n = 0;
	if (n != card->tx_buff.rp)
		mask |= POLLOUT | POLLWRNORM;

	return mask;
}

static struct file_operations sh_siof_fops = {
      read:siof_read,
      write:siof_write,
      ioctl:siof_ioctl,
      open:siof_open,
      poll:siof_poll,
      release:siof_release,
};

static inline void
free_siof_buff(void)
{
	int i;
	struct sh_siof_card *card = siof_devs;

	DPRINTK("free siof buff.\n");

	for (i = 0; i < MAX_DMA_BANK; i++) {
		if (card->rx_buff.p_buff[i] != 0) {
			kfree(P1SEGADDR(card->rx_buff.p_buff[i]));
		}
		if (card->tx_buff.p_buff[i] != 0) {
			kfree(P1SEGADDR(card->tx_buff.p_buff[i]));
		}
	}
}

static inline void
siof_init_hw(void)
{
#ifdef INITPIN
	/* PFC setup */
	ctrl_outl(ctrl_inl(MSTPCR0) & ~0x00600004, MSTPCR0);	/* SIOF,
								 * DMAC,
								 * INTC
								 * Operates */
	ctrl_outw((ctrl_inw(PSELA) & ~0xff00), PSELA);
	ctrl_outw(0x0000, PQCR);
	ctrl_outw((ctrl_inw(HIZCRA) & ~0x0008), HIZCRA);
#endif				/* INITPIN */

	/* Initialize SIOF register */
	ctrl_outw((ctrl_inw(SICTR) & ~(CTR_MST | CTR_TXE | CTR_RXE)), SICTR);
	ctrl_outw(0x0000, SIIER);
	ctrl_outw((ctrl_inw(SICTR) | CTR_TXRXRST), SICTR);

	ctrl_outw(0x8c00, SIMDR);	/* data 16bits, frame 32bits */
	ctrl_outw(0x8000, SITDAR);	/* 16bits mono (left only) */
	ctrl_outw(0x9901, SISCR);	/* 8KHz */
	ctrl_outw(0x8000, SIRDAR);	/* left channel receive enable */
	ctrl_outw(0x0000, SICDAR);
	ctrl_outw(0xa0a0, SIFCTR);
	ctrl_outw(IER_TDMAE | IER_RDMAE, SIIER);

	ctrl_outw(ctrl_inw(SICTR) & ~(CTR_RXE | CTR_TXE), SICTR);
	ctrl_outw(ctrl_inw(SICTR) | CTR_MST, SICTR);

	/* DMA registars */
	ctrl_outw(0x5152, DMARS1);	/* DMA3=tx, DMA2=rx */
	ctrl_outw(0x0301, DMAOR);	/* Round robin, All channel enable */
	ctrl_outl(0x00004814, CHCR2);
	ctrl_outl(SIRDR & DMAAMASK, SAR2);
	ctrl_outl(0x00001814, CHCR3);
	ctrl_outl(SITDR & DMAAMASK, DAR3);

	return;
}

static inline int
siof_init(void)
{
	int i;

	DPRINTK("init\n");

	struct sh_siof_card *card;
	unsigned int *tmp_buf;

	if ((card = kmalloc(sizeof (struct sh_siof_card), GFP_KERNEL)) == NULL) {
		return -ENOMEM;
	}

	memset(card, 0, sizeof (*card));
	card->siof_iobase = SIOF_IOBASE;
	card->channel = DEFAULT_SIOF_CHANNEL;
	siof_devs = card;

	for (i = 0; i < MAX_DMA_BANK; i++) {
		card->rx_buff.p_buff[i] = 0;
		card->tx_buff.p_buff[i] = 0;
	}

	/* malloc buffer */
	for (i = 0; i < MAX_DMA_BANK; i++) {
		if ((tmp_buf = kmalloc(SIOF_DMABUF_SIZE, GFP_KERNEL)) != NULL) {
			card->rx_buff.p_buff[i] = P2SEGADDR(tmp_buf);
		} else {
			printk("sh_siof:no memory(rx)\n");
			free_siof_buff();
			return -ENOMEM;
		}

		if ((tmp_buf = kmalloc(SIOF_DMABUF_SIZE, GFP_KERNEL)) != NULL) {
			card->tx_buff.p_buff[i] = P2SEGADDR(tmp_buf);
		} else {
			free_siof_buff();
			printk("sh_siof:no memory(tx)\n");
			return -ENOMEM;
		}
	}
	init_dmabuff(&card->rx_buff);
	init_dmabuff(&card->tx_buff);

	/* initialize queue */
	init_waitqueue_head(&card->wq);
	init_waitqueue_head(&card->rq);
	init_waitqueue_head(&card->syncq);

	/* Claim iospace */
	request_region(card->siof_iobase, SIOF_REG_SIZE, "sh_siof");

	if (request_irq(DMTE2_IRQ, dma2_receive_end_intr, SA_INTERRUPT, "dmac2",
			card)) {
		printk(KERN_ERR "Cannot allocate DMTE2_IRQ\n");
		free_siof_buff();
		release_region(card->siof_iobase, SIOF_REG_SIZE);
		kfree(card);
		return -ENODEV;
	}

	if (request_irq
	    (DMTE3_IRQ, dma3_transfer_end_intr, SA_INTERRUPT, "dmac3", card)) {
		printk(KERN_ERR "Cannot allocate DMTE3_IRQ\n");
		free_siof_buff();
		release_region(card->siof_iobase, SIOF_REG_SIZE);
		free_irq(DMTE2_IRQ, card);
		kfree(card);
		return -ENODEV;
	}

	if (request_irq(SIOF0_IRQ, siof_er_interrupt, SA_INTERRUPT,
			"sh_siof", card)) {
		printk(KERN_ERR "Cannot allocate SIOF_IRQ\n");
		free_siof_buff();
		release_region(card->siof_iobase, SIOF_REG_SIZE);
		free_irq(DMTE2_IRQ, card);
		free_irq(DMTE3_IRQ, card);
		kfree(card);
		return -ENODEV;
	}
	/* Register /dev/dsp */
	if ((card->dev_audio = register_sound_dsp(&sh_siof_fops, -1)) < 0) {
		printk(KERN_ERR "couldn't register DSP device!\n");
		free_siof_buff();
		release_region(card->siof_iobase, SIOF_REG_SIZE);
		free_irq(DMTE2_IRQ, card);
		free_irq(DMTE3_IRQ, card);
		free_irq(SIOF0_IRQ, card);
		kfree(card);
		return -ENODEV;
	}


	siof_init_hw();
	
	return 0;
}

static inline void
siof_exit(void)
{
	int i;
	struct sh_siof_card *card = siof_devs;

	DPRINTK("exit\n");

	release_region(card->siof_iobase, SIOF_REG_SIZE);
	free_irq(DMTE2_IRQ, card);
	free_irq(DMTE3_IRQ, card);
	free_irq(SIOF0_IRQ, card);

	unregister_sound_dsp(card->dev_audio);
	for (i = 0; i < MAX_DMA_BANK; i++) {
		kfree(P1SEGADDR(card->rx_buff.p_buff[i]));
		kfree(P1SEGADDR(card->tx_buff.p_buff[i]));
	}
	kfree(card);
}

#ifdef CONFIG_DPM
static void shmv3se_audio_ldm_register(void);
static void shmv3se_audio_ldm_unregister(void);
#endif
/****************************************************************************/

static int __init
shmv3se_audio_init(void)
{
	int rt;

	if ((rt = i2c_init()) != 0) {
		return rt;
	}

	ak2440_init();
#ifdef I2CRELEASE
	i2c_exit();
#endif

	if ((rt = siof_init()) != 0) {
		return rt;
	}

	if ((rt = siu_init()) != 0) {
		return rt;
	}

#ifdef CONFIG_DPM
 	shmv3se_audio_ldm_register();
#endif

	return 0;
}

static void __exit
shmv3se_audio_exit(void)
{
#ifdef CONFIG_DPM
	shmv3se_audio_ldm_unregister();
#endif
	siu_exit();
	siof_exit();
	ak2440_exit();
#ifndef I2CRELEASE
	i2c_exit();
#endif
}

#ifdef CONFIG_PM

#define RXRST	0x1
#define TXE	(1<<9)
#define RXE     (1<<8)
#define	SIU_STOP (1<<8)
#define SIOF_STOP (1<<2)
static int
shmv3se_suspend(struct device * dev,  u32 state, u32 level )
{
	switch (level) {
	case SUSPEND_POWER_DOWN:

		i2c_write(ak_reset, sizeof (ak_reset));
		mdelay(1);

		ctrl_outw((ctrl_inw(SICTR) & ~TXE), SICTR);	/* disable transmit */
		ctrl_outw((ctrl_inw(SICTR) & ~RXE), SICTR);	/* disable receive */
		ctrl_outl(ctrl_inl(SBCTL) & ~(1<<30), SBCTL);	/* pause */
		
		ctrl_outl(ctrl_inl(MSTPCR0)|SIOF_STOP,MSTPCR0);
		ctrl_outl(ctrl_inl(MSTPCR2)|SIU_STOP,MSTPCR2);
		
		/* Originally enabled in "siof_init()" */
		disable_irq(SIOF0_IRQ);
		disable_irq(DMTE2_IRQ);
		disable_irq(DMTE3_IRQ);
		
		/* Originally enabled in "siu_init()" */
		disable_irq(SIU_IRQ);
		disable_irq(DMTE4_IRQ);
		
		/* Originally enabled in "i2c_init()" */
		disable_irq(IIC0_ALI_IRQ);
		disable_irq(IIC0_TACKI_IRQ);
		disable_irq(IIC0_WAITI_IRQ);
		disable_irq(IIC0_DTEI_IRQ);
		
		break;
	}

	return 0;
}

static void
do_resume(void *x)
{
	/* originally done in "siof_init()" */
	enable_irq(SIOF0_IRQ);
	enable_irq(DMTE2_IRQ);
	enable_irq(DMTE3_IRQ);
	
	/* originally done in "siu_init()" */
	enable_irq(SIU_IRQ);
	enable_irq(DMTE4_IRQ);
	
	/* originally done in "i2c_init()" */
	enable_irq(IIC0_ALI_IRQ);
	enable_irq(IIC0_TACKI_IRQ);
	enable_irq(IIC0_WAITI_IRQ);
	enable_irq(IIC0_DTEI_IRQ);
#if 0  /* should identify which stanby mode */
	if ((shm3_stby_mode & (SWSTBY)) && !(shm3_stby_mode & (MLRST)))
		goto skip_point;
#endif
	ctrl_outl(ctrl_inl(MSTPCR0) & ~SIOF_STOP , MSTPCR0);
	ctrl_outl(ctrl_inl(MSTPCR2) & ~SIU_STOP , MSTPCR2);
	
	ctrl_outw(ctrl_inw(SCPCR) & ~0x0f00, SCPCR);    /* PFC from i2c_init() */
	ctrl_outb(ctrl_inb(ICCR) & ~0x80, ICCR);	/* Stop I2C */
	
	ak2440_init();
	
	siof_init_hw();

	siu_init_hw();

 skip_point:
	ctrl_outw(ctrl_inw(SICTR) |TXE, SICTR);		/* enable  transmit */
	ctrl_outw((ctrl_inw(SICTR) | RXRST), SICTR); /* reset RX operations*/
	ctrl_outw(ctrl_inw(SICTR) |RXE, SICTR);		/* enable receive */
	ctrl_outl(ctrl_inl(SBCTL) |(1<<30), SBCTL);	/* resume */

	/* should be implemented here */
	nmi_restore_dma(); 

	/* siof queue */
	if (waitqueue_active(&siof_devs->wq))
		wake_up_interruptible(&siof_devs->wq);
	if (waitqueue_active(&siof_devs->rq))
		wake_up_interruptible(&siof_devs->rq);
	if (waitqueue_active(&siof_devs->syncq))
		wake_up_interruptible(&siof_devs->syncq);
	
	/* siu queue */
	if (waitqueue_active(&siu_devs->wq))
		wake_up_interruptible(&siu_devs->wq);
	if (waitqueue_active(&siu_devs->syncq))
		wake_up_interruptible(&siu_devs->syncq);
	
	return;
}

static struct tq_struct resume_task = { routine: do_resume };

static int
shmv3se_resume(struct device * dev, u32 level )
{
	switch (level) {
	case RESUME_POWER_ON:
		/*
		 * Resume is getting called in an interrupt context on
		 * SH, and resume requires waiting on queues etc. to power
		 * up the camera. So we can't resume here. So we have to
		 * use a kernel thread for resume requests (PITA).
		 */
		resume_task.data = NULL;
		schedule_task(&resume_task);

		break;
	}

	return 0;
}
#endif

#ifdef CONFIG_DPM
static struct device_driver shmv3se_audio_driver_ldm = {
        name:           "shm3se_audio",
        devclass:       NULL,
        probe:          NULL,
        suspend:        shmv3se_suspend,
        resume:         shmv3se_resume,
        scale:          NULL,
        remove:         NULL,
};
 
static struct device shmv3se_audio_device_ldm = {
        name:           "SUPERH SIOF Sound Controller",
        bus_id:         "sound",
        driver:         NULL,
        power_state:    DPM_POWER_ON,
};

static void shmv3se_audio_ldm_register(void)
{
        extern void plb_driver_register(struct device_driver *driver);
        extern void plb_device_register(struct device *device);

        plb_driver_register(&shmv3se_audio_driver_ldm);
        plb_device_register(&shmv3se_audio_device_ldm);
}
						 
static void shmv3se_audio_ldm_unregister(void)
{
        extern void plb_driver_unregister(struct device_driver *driver);
        extern void plb_device_unregister(struct device *device);
				 
        plb_driver_unregister(&shmv3se_audio_driver_ldm);
        plb_device_unregister(&shmv3se_audio_device_ldm);
}
											
#endif /* CONFIG_DPM */

MODULE_LICENSE("GPL");

module_init(shmv3se_audio_init);
module_exit(shmv3se_audio_exit);
