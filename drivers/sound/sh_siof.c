/*
 *  drivers/sound/sh_siof.c
 *  Copyright (C) 2002,2003 Kuniki Nakamura
 *  SIOF sound device driver for Super-H
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/pm.h>
#include <linux/device.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "sh_siof.h"

#define DMABUF_SIZE  1024
#define DMABUF_MAXNUM  160

#define SIOF_8BIT   1
#define SIOF_16BIT  2
#define SIOF_MONO   1
#define SIOF_STEREO 2
#define S_RATE      44100

typedef struct dma_buf {
	unsigned int   *buf;
	unsigned int    size;
} DMA_BUF;

typedef struct dma_fifo {
	DMA_BUF         dma_buf[DMABUF_MAXNUM];
	unsigned int    w_p;
	unsigned int    r_p;
} DMA_FIFO;

struct siof_card {

	wait_queue_head_t wq;
	wait_queue_head_t rq;
	wait_queue_head_t syncq;
	unsigned int    mode;	/* fmt and channel */
	unsigned int    fmt;	/* fmt */
	unsigned int    channel;	/* channel */

	/* DMA stuff */
	DMA_FIFO        tx_dmabuf;
	DMA_FIFO        rx_dmabuf;

	/* soundcore stuff */
	int             dev_audio;

	/* hardware resourses */
	unsigned long   iobase;
	unsigned long   irqs;
	unsigned int    w_openCnt;
	unsigned int    r_openCnt;

#ifdef DEBUG_BUF
	unsigned int    tx_buferrCnt;
	unsigned int    rx_buferrCnt;
#endif

};

#ifdef DEBUG_BUF
unsigned int    tx_ovr = 0;
unsigned int    tx_udr = 0;
unsigned int    rx_ovr = 0;
unsigned int    rx_udr = 0;
#endif

static struct siof_card *devs = NULL;

static int siof_busy=0;

static int
siof_open(struct inode *inode, struct file *file)
{
	struct siof_card *card;

#ifdef DEBUG
	printk("siof_open start\n");
#endif

	card = devs;
	siof_busy=1;
	if (file->f_mode & FMODE_READ) {
		if (card->r_openCnt > 0)
		{
			return -EBUSY;
		}
	}
	if (file->f_mode & FMODE_WRITE)	{
		if (card->w_openCnt > 0)
		{
			return -EBUSY;
		}
	}

	card->fmt = SIOF_16BIT;
	card->channel = SIOF_STEREO;
	card->mode = M16B_STER;	/* 16bit stereo mode */

	ctrl_outw(ctrl_inw(SICTR) | SCKE | FSE, SICTR);
	ctrl_outb(0x00, PORT_PMDR);	/* reset negate */
	/* enable SCK and FSYNC */

	if (file->f_mode & FMODE_READ) {
		ctrl_outw(ctrl_inw(SICTR) | RXRST, SICTR);	/* reset receive module */
		card->rx_dmabuf.w_p = 0;
		card->rx_dmabuf.r_p = 0;
		card->r_openCnt++;
	}
	if (file->f_mode & FMODE_WRITE) {
		ctrl_outw(ctrl_inw(SICTR) | TXRST, SICTR);	/* reset Transmit module */
		card->tx_dmabuf.w_p = 0;
		card->tx_dmabuf.r_p = 0;
		card->w_openCnt++;
	}
#ifdef DEBUG
	printk("SICTR = 0x%x\n", ctrl_inw(SICTR));
#endif
	MOD_INC_USE_COUNT;
	return 0;
}

static void
rx_start_dma(void *buf, unsigned int size)
{
#ifdef DEBUG
	printk("start DMA \n");
	printk("buf = 0x%x size = %x\n", buf, size);
#endif
	ctrl_outl((ctrl_inl(CHCR2) & ~DE), CHCR2);
	ctrl_outl((unsigned int) buf, DAR2);	/* set DMA buffer address */
//      ctrl_outl(size/4 + ((size%4 > 0)? 1 : 0), DMATCR2); /* set DMA count */
	ctrl_outl(size, DMATCR2);	/* set DMA count */
	ctrl_outl((ctrl_inl(CHCR2) & ~TE), CHCR2);
	ctrl_outl((ctrl_inl(CHCR2) | DE), CHCR2);	/* DMA start */
}

static void
tx_start_dma(void *buf, unsigned int size)
{
#ifdef DEBUG
	printk("start DMA \n");
	printk("buf = 0x%x size = %x\n", buf, size);
#endif
	ctrl_outl((ctrl_inl(CHCR3) & ~DE), CHCR3);
	ctrl_outl((unsigned int) buf, SAR3);	/* set DMA buffer address */
//      ctrl_outl(size/4 + ((size%4 > 0)? 1 : 0), DMATCR3); /* set DMA count */
	ctrl_outl(size, DMATCR3);	/* set DMA count */
	ctrl_outl((ctrl_inl(CHCR3) & ~TE), CHCR3);
	ctrl_outl((ctrl_inl(CHCR3) | DE), CHCR3);	/* DMA start */
}

static inline void
rx_stop(void)
{
	ctrl_outl(ctrl_inl(CHCR2) & ~(DE | TE), CHCR2);
	/* DMA stop , disable DMATCR interrupt */
	ctrl_outw(ctrl_inw(SIIER) & ~(RDREQE | RFOVRE | RFUDRE), SIIER);
	/* disable dma transfer trigger , disable receive,underrn,overrun interrupt */
	ctrl_outw(ctrl_inw(SICTR) | RXRST, SICTR);	/* reset receive module */
}

static inline void
tx_stop(void)
{
#ifdef DEBUG
	printk("tx_stop\n");
#endif
	ctrl_outl(ctrl_inl(CHCR3) & ~(DE | TE), CHCR3);
	/* DMA stop , disable DMATCR interrupt */
	ctrl_outw(ctrl_inw(SIIER) & ~(TDREQE | TFUDRE | TFOVRE), SIIER);
	/* disable dma transfer trigger , disable tranmit,underrun,overrun interrupt */
	ctrl_outw(ctrl_inw(SICTR) | TXRST, SICTR);	/* reset Transmit module */
}

static inline unsigned int
buffer_full(DMA_FIFO * fifo)
{
	if (((fifo->w_p + 1) & (DMABUF_MAXNUM - 1)) == fifo->r_p) {
		return 1;
	}
	return 0;
}

static inline unsigned int
buffer_empty(DMA_FIFO * fifo)
{
	if (fifo->w_p == fifo->r_p) {
		return 1;
	}
	return 0;
}

static unsigned int
make_pcm_data(unsigned int mode, unsigned char *buffer, unsigned int *lchrch)
{
	unsigned int    ret = 0;

	switch (mode) {
	case M8B_MONO:
		__put_user(*lchrch >> 24, buffer);
		ret = 1;
		break;
	case M16B_MONO:
		__put_user(*lchrch >> 16, buffer);
		__put_user(*lchrch >> 24, buffer + 1);
		ret = 2;
		break;
	case M8B_STER:
		__put_user(*lchrch >> 24, buffer);
		__put_user(*lchrch >> 8, buffer);
		ret = 2;
		break;
	case M16B_STER:
		__put_user(*lchrch >> 16, buffer);
		__put_user(*lchrch >> 24, buffer);
		__put_user(*lchrch >> 0, buffer);
		__put_user(*lchrch >> 8, buffer);
		ret = 4;
		break;
	}

	return ret;
}

//static unsigned int make_siof_data(unsigned int mode, const char * buf, unsigned int * lchrch)
static unsigned int
make_siof_data(unsigned int mode, const char *cbuf, unsigned int *lchrch)
{
	unsigned int    ret = 0;
	const unsigned char *buf = cbuf;
	unsigned int    d0, d1, d2, d3;

	switch (mode) {
	case M8B_MONO:
		__get_user(d0, buf);
		*lchrch = d0 << 24;
		ret = 1;
		break;
	case M16B_MONO:
		__get_user(d0, buf);
		__get_user(d1, buf + 1);
		*lchrch = (d0 << 16) + (d1 << 24);
		ret = 2;
		break;
	case M8B_STER:
		__get_user(d0, buf);
		__get_user(d1, buf + 1);
		*lchrch = (d0 << 24) + (d1 << 8);
		ret = 2;
		break;
	case M16B_STER:
		__get_user(d0, buf);
		__get_user(d1, buf + 1);
		__get_user(d2, buf + 2);
		__get_user(d3, buf + 3);
		*lchrch = (d0 << 16) + (d1 << 24) + (d2) + (d3 << 8);
		ret = 4;
		break;
	}

	return ret;
}

static ssize_t
siof_read(struct file *file, char *buffer, size_t count, loff_t * ppos)
{
	struct siof_card *card = devs;
	DMA_BUF        *fifo;
	unsigned int    data_size;
	unsigned int    lchrch;
	int             ct = (int) count;
	unsigned long   flags;

#ifdef DEBUG
	printk("start siof_read \n");
	printk("ct = 0x%x\n", ct);
#endif
	if (!access_ok(VERIFY_WRITE, buffer, count))
		return -EFAULT;

	save_flags(flags);

	while (ct > 0) {
#ifdef DEBUG
		printk("start while(ct > 0) ct %x \n", ct);
		printk("dma count 0x%x ", ctrl_inl(DMATCR2));
		printk("DMAOR 0x%x ", ctrl_inw(DMAOR));
		printk("SAR2 0x%x ", ctrl_inl(SAR2));
		printk("DAR2 0x%x ", ctrl_inl(DAR2));
		printk("CHCR2 0x%x\n", ctrl_inl(CHCR2));
		printk("SIOF SISTR 0x%x ", ctrl_inw(SISTR));
		printk("SIIER 0x%x ", ctrl_inw(SIIER));
		printk("SICTR 0x%x \n", ctrl_inw(SICTR));
#endif
		
		if (!(ctrl_inl(CHCR2) & DE)) {
			if (!buffer_full(&card->rx_dmabuf)) {
				fifo = &card->rx_dmabuf.dma_buf[card->rx_dmabuf.w_p];
				fifo->size = 0;
				rx_start_dma(fifo->buf, DMABUF_SIZE);
				ctrl_outw((ctrl_inw(SICTR) | RXE), SICTR);	/* enable SIO receive */
				ctrl_outw(ctrl_inw(SISTR), SISTR);
				/* enable receive,overrun,underrun interrupt */
				ctrl_outw((ctrl_inw(SIIER) | RDREQE | RFOVRE | RFUDRE), SIIER);	
			}
		}

		cli();
		if (buffer_empty(&card->rx_dmabuf)) {
			if (file->f_flags & O_NONBLOCK)	{
				return -EAGAIN;
			}
#ifdef DEBUG
			printk("sleep on\n");
#endif
			interruptible_sleep_on(&card->rq);
			if (signal_pending(current)) {
				break;
			}
		}
		sti();

		fifo = &card->rx_dmabuf.dma_buf[card->rx_dmabuf.r_p];
		while (fifo->size < DMABUF_SIZE && ct > 0) {
			lchrch = fifo->buf[fifo->size++];
			data_size =
				make_pcm_data(card->mode,
					      (unsigned char *) buffer,
					      &lchrch);
			ct -= data_size;
			buffer += data_size;
		}
		if (fifo->size == DMABUF_SIZE) {
			card->rx_dmabuf.r_p =
				(++card->rx_dmabuf.r_p) & (DMABUF_MAXNUM - 1);
		}
	}

	restore_flags(flags);
#ifdef DEBUG
	printk("read end(return %x) ct %x\n", count - ct, ct);
#endif
	ct = (ct < 0) ? 0 : ct;
	return count - ct;
}

static ssize_t
siof_write(struct file *file, const char *buf, size_t count, loff_t * ppos)
{
	struct siof_card *card = devs;
	DMA_BUF        *fifo;
	unsigned int    lchrch;
	unsigned int    data_size;
	int             ct = (int) count;
	unsigned long   flags;

#ifdef DEBUG
	printk("start siof_write \n");
	printk("ct = 0x%x\n", ct);
#endif
	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	save_flags(flags);

	while (ct > 0) {
#ifdef DEBUG
		printk("start while(ct > 0) ct %x \n", ct);
		printk("dma count 0x%x ", ctrl_inl(DMATCR3));
		printk("DMAOR 0x%x ", ctrl_inw(DMAOR));
		printk("SAR3 0x%x ", ctrl_inl(SAR3));
		printk("DAR3 0x%x ", ctrl_inl(DAR3));
		printk("CHCR3 0x%x\n", ctrl_inl(CHCR3));
		printk("SIOF SISTR 0x%x ", ctrl_inw(SISTR));
		printk("SIIER 0x%x ", ctrl_inw(SIIER));
		printk("SICTR 0x%x \n", ctrl_inw(SICTR));
#endif

		cli();
		if (buffer_full(&card->tx_dmabuf)) {
			if (file->f_flags & O_NONBLOCK)
			{
				return -EAGAIN;
			}
#ifdef DEBUG
			printk("sleep on\n");
#endif
			interruptible_sleep_on(&card->wq);
			if (signal_pending(current))
			{
				break;
			}
		}
		sti();

		fifo = &card->tx_dmabuf.dma_buf[card->tx_dmabuf.w_p];
		fifo->size = 0;
		while (fifo->size < DMABUF_SIZE && ct > 0) {
			data_size =
				make_siof_data(card->mode, buf, &lchrch);
			fifo->buf[fifo->size++] = lchrch;
			ct -= data_size;
			buf += data_size;
		}
		card->tx_dmabuf.w_p =
			(++card->tx_dmabuf.w_p) & (DMABUF_MAXNUM - 1);
#ifdef DEBUG1
		printk("fifo->size %x, ", fifo->size);
		printk("tx_dma:read_p  %d, ", card->tx_dmabuf.r_p);
		printk("tx_dma:write_p %d\n", card->tx_dmabuf.w_p);
#endif

		cli();
		if (!(ctrl_inl(CHCR3) & DE)) {
			fifo = &card->tx_dmabuf.dma_buf[card->tx_dmabuf.r_p];
			tx_start_dma(fifo->buf, fifo->size);
			ctrl_outw((ctrl_inw(SICTR) | TXE), SICTR);	/* enable SIO transmit */
			ctrl_outw(ctrl_inw(SISTR), SISTR);
			/* enable transmit,underrun,overrun interrupt */
			ctrl_outw((ctrl_inw(SIIER) | TDREQE | TFOVRE | TFUDRE), SIIER);	
		}
		sti();
	}

	restore_flags(flags);
#ifdef DEBUG
	printk("write end(return %x) ct %x\n", count - ct, ct);
#endif
	ct = (ct < 0) ? 0 : ct;
	return count - ct;
}

static int
siof_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	struct siof_card *card = devs;
	int             val;

#ifdef DEBUG
	printk("sh_siof: siof_ioctl\n");
#endif

	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *) arg);

	case SNDCTL_DSP_RESET:
		if (file->f_mode & FMODE_WRITE) {
			tx_stop();
			card->tx_dmabuf.w_p = 0;
			card->tx_dmabuf.r_p = 0;
		}
		if (file->f_mode & FMODE_READ) {
			rx_stop();
			card->rx_dmabuf.w_p = 0;
			card->rx_dmabuf.r_p = 0;
		}
		return 0;

	case SNDCTL_DSP_SYNC:
		if (file->f_mode & FMODE_WRITE) {
			if ((ctrl_inl(CHCR3) & DE)) {
				interruptible_sleep_on(&card->syncq);
			}
		}
		return 0;

	case SNDCTL_DSP_SPEED:	/* set smaple rate */
		return put_user(S_RATE, (int *) arg);	/* sample rate is 44100hz */

	case SNDCTL_DSP_STEREO:	/* set stereo or mono channel */
		if (get_user(val, (int *) arg))
			return -EFAULT;
		if (val == 0)
			return -EINVAL;
		if (file->f_mode & FMODE_WRITE)	{
			tx_stop();
			card->channel = SIOF_STEREO;
		}
		if (file->f_mode & FMODE_READ) {
			rx_stop();
			card->channel = SIOF_STEREO;
		}
		return 0;

	case SNDCTL_DSP_GETBLKSIZE:
		val = (DMABUF_SIZE * card->fmt * card->channel);
		return put_user(val, (int *) arg);

	case SNDCTL_DSP_GETFMTS:	/* Returns a mask of supported sample format */
		return put_user(AFMT_S16_LE, (int *) arg);

	case SNDCTL_DSP_SETFMT:	/* Select sample format */
		if (get_user(val, (int *) arg))	{
			return -EFAULT;
		}
		if (val != AFMT_QUERY) {
			if (file->f_mode & FMODE_WRITE) {
				tx_stop();
			}
			if (file->f_mode & FMODE_READ) {
				rx_stop();
			}
		}
		return put_user(AFMT_S16_LE, (int *) arg);

	case SNDCTL_DSP_CHANNELS:
		if (get_user(val, (int *) arg))	{
			return -EFAULT;
		}
		if (val != 0) {
			if (file->f_mode & FMODE_WRITE)	{
				tx_stop();
			}
			if (file->f_mode & FMODE_READ) {
				rx_stop();
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
		return put_user(S_RATE, (int *) arg);

	case SOUND_PCM_READ_CHANNELS:
		return put_user((card->channel & SIOF_STEREO) ? 2 : 1,
				(int *) arg);

	case SOUND_PCM_READ_BITS:
		return put_user(AFMT_S16_LE, (int *) arg);

	default:
		printk("unknown ioctl 0x%x\n", cmd);
		return -EINVAL;
	}
	printk("unimplemented ioctl 0x%x\n", cmd);
	return -EINVAL;
}

static int
siof_release(struct inode *inode, struct file *file)
{
	struct siof_card *card = devs;

#ifdef DEBUG
	printk("DMATCR: %08x\n", ctrl_inl(DMATCR3));
#endif
	if (file->f_mode & FMODE_READ) {
		rx_stop();
		card->r_openCnt--;
		if (card->r_openCnt < 0) {
			card->r_openCnt = 0;
		}
	}
	if (file->f_mode & FMODE_WRITE)	{
		tx_stop();
		card->w_openCnt--;
		if (card->w_openCnt < 0) {
			card->w_openCnt = 0;
		}
	}
	MOD_DEC_USE_COUNT;
	if (!MOD_IN_USE) {
		ctrl_outb(0x10, PORT_PMDR);
		ctrl_outw(0x0000, SICTR);	/* disable SCK,FSYNC,receive and transmit */
	}
#ifdef DEBUG_BUF
	printk("tx_buferrCnt = %d rx_buferrCnt = %d\n", card->tx_buferrCnt,
	       card->rx_buferrCnt);
	printk("tx_udr = %d tx_ovr = %d rx_udr = %d rx_ovr = %d\n", tx_udr,
	       tx_ovr, rx_udr, rx_ovr);
	if (file->f_mode & FMODE_READ) {
		card->rx_buferrCnt = 0;
		rx_ovr = 0;
		rx_udr = 0;
	}
	if (file->f_mode & FMODE_WRITE) {
		card->tx_buferrCnt = 0;
		tx_ovr = 0;
		tx_udr = 0;
	}
#endif
	siof_busy=0;
	return 0;
}

static /*const */ struct file_operations sh_siof_fops = {
	read:    siof_read,
	write:   siof_write,
	ioctl:   siof_ioctl,
	open:    siof_open,
	release: siof_release,
};

static void
dma_receive_end_intr(int irq, void *dev_id, struct pt_regs *regs)
{
	struct siof_card *card = devs;
	DMA_BUF        *fifo;

	ctrl_outl(((ctrl_inl(CHCR2) & ~DE) & ~TE), CHCR2);	/* DMA stop , disable DMATCR interrupt */

#ifdef DEBUG
	printk("dma_receive_end_intr\n");
#endif

	card->rx_dmabuf.w_p = (++card->rx_dmabuf.w_p) & (DMABUF_MAXNUM - 1);
	if (!buffer_full(&card->rx_dmabuf)) {
		fifo = &card->rx_dmabuf.dma_buf[card->rx_dmabuf.w_p];
		fifo->size = 0;
		rx_start_dma(fifo->buf, DMABUF_SIZE);
	} else {
		/* disable dma receive trigger , disable receive overrun,underrun interrupt */
		ctrl_outw((((ctrl_inw(SIIER) & ~RDREQE) & ~RFOVRE) & ~RFUDRE), SIIER);	
		ctrl_outw((ctrl_inw(SICTR) & ~RXE), SICTR);	/* disable receive */
#ifdef DEBUG_BUF
		card->rx_buferrCnt++;
#endif
	}

	wake_up_interruptible(&card->rq);
#ifdef DEBUG
	printk("wake_up\n");
#endif
}

static void
dma_transfer_end_intr(int irq, void *dev_id, struct pt_regs *regs)
{
	struct siof_card *card = devs;
	DMA_BUF        *fifo;

	ctrl_outl(((ctrl_inl(CHCR3) & ~DE) & ~TE), CHCR3);	/* DMA stop , disable DMATCR interrupt */

#ifdef DEBUG
	printk("dma_transfer_end_intr\n");
#endif

	card->tx_dmabuf.r_p = (++card->tx_dmabuf.r_p) & (DMABUF_MAXNUM - 1);
	if (!buffer_empty(&card->tx_dmabuf)) {
		fifo = &card->tx_dmabuf.dma_buf[card->tx_dmabuf.r_p];
		tx_start_dma(fifo->buf, fifo->size);
	} else {
		/* disable dma transfer trigger , disable transmit underrun,overrun interrupt */
		ctrl_outw((((ctrl_inw(SIIER) & ~TDREQE) & ~TFUDRE) & ~TFOVRE), SIIER);	
		ctrl_outw((ctrl_inw(SICTR) & ~TXE), SICTR);	/* disable transmit */
		wake_up_interruptible(&card->syncq);
#ifdef DEBUG_BUF
		card->tx_buferrCnt++;
#endif
	}

	wake_up_interruptible(&card->wq);
#ifdef DEBUG
	printk("wake_up\n");
#endif

}

static void
siof_er_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned int   *data, i;
	unsigned short  status;

#ifdef DEBUG
	printk("siof_er_interrupt\n");
#endif
	status = ctrl_inw(SISTR);
	ctrl_outw(status, SISTR);
#ifdef DEBUG
	printk("status = %04x\n", status);
#endif

	if (status & RFOVR) {
		if (ctrl_inl(DMATCR2) > 0) {
			ctrl_outw((ctrl_inw(SICTR) & ~RXE), SICTR);	/* disable receive */
#if 1
			ctrl_outl((ctrl_inl(CHCR2) & ~DE), CHCR2);	/* DMA stop */
			data = (unsigned int *) ctrl_inl(DAR2);
			for (i = 0; i < 16; i++)
				*(data++) = ctrl_inl(SIRDR);
			rx_start_dma(data, ctrl_inl(DMATCR2) - 16);
#endif
			ctrl_outw((ctrl_inw(SICTR) | RXE), SICTR);	/* enable SIO receive */
		}
	}
	if (status & TFUDR) {
		if (ctrl_inl(DMATCR3) > 0) {
			ctrl_outw((ctrl_inw(SICTR) & ~TXE), SICTR);	/* disable transmit */
			ctrl_outl((ctrl_inl(CHCR3) & ~DE), CHCR3);	/* DMA stop */
			data = (unsigned int *) ctrl_inl(SAR3);
			for (i = 0; i < 16; i++)
				ctrl_outl(*(data++), SITDR);
			tx_start_dma(data, ctrl_inl(DMATCR3) - 16);
			ctrl_outw((ctrl_inw(SICTR) | TXE), SICTR);	/* enable SIO transmit */
		}
	}
#ifdef DEBUG_BUF
	if (status & TFUDR) {
		tx_udr++;
	}
	if (status & TFOVR) {
		tx_ovr++;
	}
	if (status & RFUDR) {
		rx_udr++;
	}
	if (status & RFOVR) {
		rx_ovr++;
	}
#endif
}

static int __init
siof_init(void)
{
	struct siof_card *card;
	unsigned int    i, *tmp_buf;

#ifdef DEBUG
	printk("siof_init start\n");
#endif

	if ((card = kmalloc(sizeof (struct siof_card), GFP_KERNEL)) == NULL)
		return -ENOMEM;
	memset(card, 0, sizeof (*card));

	for (i = 0; i < DMABUF_MAXNUM; i++) {
		if ((tmp_buf = kmalloc((sizeof (int) * DMABUF_SIZE), GFP_KERNEL)) != NULL) {
			card->rx_dmabuf.dma_buf[i].buf = P2SEGADDR(tmp_buf);
			card->rx_dmabuf.dma_buf[i].size = 0;
		} else {
			printk("sh_siof:no memory(rx)\n");
			return -ENOMEM;
		}
	}
	for (i = 0; i < DMABUF_MAXNUM; i++) {
		if ((tmp_buf = kmalloc((sizeof (int) * DMABUF_SIZE), GFP_KERNEL)) != NULL) {
			card->tx_dmabuf.dma_buf[i].buf = P2SEGADDR(tmp_buf);
			card->tx_dmabuf.dma_buf[i].size = 0;
		} else {
			printk("sh_siof:no memory(tx)\n");
			return -ENOMEM;
		}
	}

	disable_hlt();

	card->iobase = SIOF_IOBASE;

	devs = card;
	init_waitqueue_head(&card->wq);
	init_waitqueue_head(&card->rq);
	init_waitqueue_head(&card->syncq);
	ctrl_outw(ctrl_inw(PORT_PSELB) | 0xfe00, PORT_PSELB);	/* switch to SIOF */
	ctrl_outw(0x0500, PORT_PMCR);	/*  (VIO, MCLK, PTM5,4, SIOF) */
	ctrl_outb(0x10, PORT_PMDR);	/* PTM4=1 to reset codec */

	/* claim our iospace and irq */
	request_region(card->iobase, 45, "sh_siof");

	if (request_irq(DMTE2_IRQ, dma_receive_end_intr, SA_INTERRUPT, "dmac2", card)) {
		printk(KERN_ERR "dmac: Cannnot allocate irq\n");
		release_region(card->iobase, 45);
		kfree(card);
		return -ENODEV;
	}

	if (request_irq(DMTE3_IRQ, dma_transfer_end_intr, SA_INTERRUPT, "dmac3", card))	{
		printk(KERN_ERR "dmac: Cannnot allocate irq\n");
		free_irq(DMTE2_IRQ, card);
		release_region(card->iobase, 45);
		kfree(card);
		return -ENODEV;
	}

	if (request_irq(SIOF0_IRQ, siof_er_interrupt, SA_INTERRUPT, "siof_err", card)) {
		printk(KERN_ERR "siof: Cannot allocate irq\n");
		free_irq(DMTE3_IRQ, card);
		free_irq(DMTE2_IRQ, card);
		release_region(card->iobase, 45);
		kfree(card);
		return -ENODEV;
	}

	/* register /dev/dsp */
	if ((card->dev_audio = register_sound_dsp(&sh_siof_fops, -1)) < 0) {
		printk(KERN_ERR "sh_siof: couldn't register DSP device!\n");
		free_irq(SIOF0_IRQ, card);
		free_irq(DMTE3_IRQ, card);
		free_irq(DMTE2_IRQ, card);
		release_region(card->iobase, 45);
		kfree(card);
		return -ENODEV;
	}
	/* initialize siof register */
	ctrl_outw(0xdd00, SIMDR);
	ctrl_outw(0x0001, SISCR);
	ctrl_outw(0x8183, SITDAR);
	ctrl_outw(0x8082, SIRDAR);
	ctrl_outw(0xc080, SIFCTR);

	ctrl_outw(TDMAE | RDMAE, SIIER);

	/* DMA setting */
	/* channel3 siof transmit, channel2 siof receive */
	ctrl_outw(0x5152, DMARS1);

	ctrl_outw(0x0301, DMAOR);	/* DMAC Master enable */
	ctrl_outl(0x00004814, CHCR2);
	ctrl_outl(SIRDR, SAR2);
	ctrl_outl(0x00001814, CHCR3);
	ctrl_outl(SITDR, DAR3);
#ifdef DEBUG
	printk("SIMDR(0x%08x)  = 0x%x\n", SIMDR, ctrl_inw(SIMDR));
	printk("SISCR(0x%08x)  = 0x%x\n", SISCR, ctrl_inw(SISCR));
	printk("SITDAR(0x%08x) = 0x%x\n", SITDAR, ctrl_inw(SITDAR));
	printk("SIRDAR(0x%08x) = 0x%x\n", SIRDAR, ctrl_inw(SIRDAR));
	printk("SIFCTR(0x%08x) = 0x%x\n", SIFCTR, ctrl_inw(SIFCTR));
#endif
	siof_ldm_register();
	return 0;
}

void __exit
siof_exit(void)
{
	struct siof_card *card = devs;
	unsigned int    i;
	
	siof_ldm_unregister();

#ifdef DEBUG
	printk("siof_exit \n");
#endif
	ctrl_outw(0x0300, DMAOR);	/* DMAC Master disable */

	/* free hardware resources */
	release_region(card->iobase, 45);
	free_irq(DMTE2_IRQ, card);
	free_irq(DMTE3_IRQ, card);
	free_irq(SIOF0_IRQ, card);

	/* unregister audio devices */
	unregister_sound_dsp(card->dev_audio);
	for (i = 0; i < DMABUF_MAXNUM; i++) {
		kfree(P1SEGADDR(card->rx_dmabuf.dma_buf[i].buf));
	}
	for (i = 0; i < DMABUF_MAXNUM; i++) {
		kfree(P1SEGADDR(card->tx_dmabuf.dma_buf[i].buf));
	}
	kfree(card);
}

static int
siof_suspend(struct device * dev,  u32 state, u32 level )
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		if (siof_busy)   {
			ctrl_outw((ctrl_inw(SICTR) & ~TXE), SICTR);	/* disable transmit */
			ctrl_outw((ctrl_inw(SICTR) & ~RXE), SICTR);	/* disable receive */
		}
		ctrl_outb(ctrl_inb(STBCR3) | 0x40, STBCR3);     /* SIOF clock stop */
		break;
	}
	return(0);
}

static int
siof_resume(struct device * dev, u32 level )
{
	switch (level) {
	case RESUME_POWER_ON:
		ctrl_outb(ctrl_inb(STBCR3) & ~0x40, STBCR3);    /* SIOF clock start  */
		if (siof_busy) {
			ctrl_outw((ctrl_inw(SICTR) | TXE), SICTR);      /* enable SIO transmit */
			ctrl_outw((ctrl_inw(SICTR) | RXE), SICTR);      /* enable SIO receive */
	       }
		break;
	}
	return(0);
}

static struct device_driver siof_driver_ldm = {
	name:      	"siof",
	devclass:  	NULL,
	probe:     	NULL,
	suspend:   	siof_suspend,
	resume:    	siof_resume,
	scale:	  	NULL,
	remove:    	NULL,
};

static struct device siof_device_ldm = {
	name:		"SUPERH SIOF Sound Controller",
	bus_id:		"siof",
	driver: 	NULL,
	power_state:	DPM_POWER_ON,
};

static void siof_ldm_register(void)
{
#ifdef CONFIG_SUPERH
	extern void plb_driver_register(struct device_driver *driver);
	extern void plb_device_register(struct device *device);

	plb_driver_register(&siof_driver_ldm);
	plb_device_register(&siof_device_ldm);
#endif
}

static void siof_ldm_unregister(void)
{
#ifdef CONFIG_SUPERH
	extern void plb_driver_unregister(struct device_driver *driver);
	extern void plb_device_unregister(struct device *device);

	plb_driver_unregister(&siof_driver_ldm);
	plb_device_unregister(&siof_device_ldm);
#endif
}


MODULE_LICENSE("GPL");

module_init(siof_init);
module_exit(siof_exit);

EXPORT_SYMBOL(siof_suspend);
EXPORT_SYMBOL(siof_resume);
