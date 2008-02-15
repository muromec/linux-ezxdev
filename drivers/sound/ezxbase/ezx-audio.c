/*
 * Copyright (C) 2002-2005 Motorola Inc.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 *  History:
 *  zhouqiong          Jun 20,2002             created
 *  LiYong             Sep 23,2003             (1)Port from EZX
 *  Jin Lihong(w20076) Mar.15,2004,LIBdd86574  mixer bug fix
 *  Jin Lihong(w20076) Apr.13,2004,LIBdd96876  close dsp protection,and add 3d control interface for app
 *  Jin Lihong(w20076) Jun.15,2004,LIBee21625  boomer's power management
 *  Jin Lihong(w20076) Aug.11,2004,LIBff01482  DMA channel release bug fix
 *  Lv Yunguang(a6511c) Mar.10,2005            Change the debug log for barbados 
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/sound.h>
#include <linux/soundcard.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/dma.h>

#include "ezx-common.h"
#include "ezx-audio.h"


#define AUDIO_NBFRAGS_DEFAULT	8
#define AUDIO_FRAGSIZE_DEFAULT	8192

#define MAX_DMA_SIZE		4096
#define DMA_DESC_SIZE		sizeof(pxa_dma_desc)

#define AUDIO_ACTIVE(state)	((state)->rd_ref || (state)->wr_ref)


/*
 * This function frees all buffers
 */
static void audio_clear_buf(audio_stream_t *s)
{
	DECLARE_WAITQUEUE(wait, current);
	int frag;

	if (!s->buffers)
		return;

	/* Ensure DMA isn't running */
        AUDPRINTk2("ensure dma isn't running\n");
	set_current_state(TASK_UNINTERRUPTIBLE);
	add_wait_queue(&s->stop_wq, &wait);
	DCSR(s->dma_ch) = DCSR_STOPIRQEN;
	schedule();
	remove_wait_queue(&s->stop_wq, &wait);
        AUDPRINTk2("free dma buffers\n");
	/* free DMA buffers */
	for (frag = 0; frag < s->nbfrags; frag++) {
		audio_buf_t *b = &s->buffers[frag];
		if (!b->master)
			continue;
		consistent_free(b->data, b->master, b->dma_desc->dsadr);
	}
        AUDPRINTk2("free descriptor buffers\n");
	/* free descriptor ring */
	if (s->buffers->dma_desc)
		consistent_free(s->buffers->dma_desc, 
				s->nbfrags * s->descs_per_frag * DMA_DESC_SIZE,
				s->dma_desc_phys);

	/* free buffer structure array */
	kfree(s->buffers);
	s->buffers = NULL;
}

/*
 * This function allocates the DMA descriptor array and buffer data space
 * according to the current number of fragments and fragment size.
 */
static int audio_setup_buf(audio_stream_t *s)
{
	pxa_dma_desc *dma_desc;
	dma_addr_t dma_desc_phys;
	int nb_desc, frag, i, buf_size = 0;
	char *dma_buf = NULL;
	dma_addr_t dma_buf_phys = 0;

	if (s->buffers)
		return -EBUSY;
        AUDPRINTk2("audio setup buffer \n");
	/* Our buffer structure array */
	s->buffers = kmalloc(sizeof(audio_buf_t) * s->nbfrags, GFP_KERNEL);
	if (!s->buffers)
		goto err;
	memzero(s->buffers, sizeof(audio_buf_t) * s->nbfrags);

	/* 
	 * Our DMA descriptor array:
	 * for Each fragment we have one checkpoint descriptor plus one 
	 * descriptor per MAX_DMA_SIZE byte data blocks.
	 */
	AUDPRINTk2("malloc descriptor memory\n");
        nb_desc = (1 + (s->fragsize + MAX_DMA_SIZE - 1)/MAX_DMA_SIZE) * s->nbfrags;
	dma_desc = consistent_alloc(GFP_KERNEL,
				    nb_desc * DMA_DESC_SIZE,
				    &dma_desc_phys);
	if (!dma_desc)
		goto err;
	s->descs_per_frag = nb_desc / s->nbfrags;
	s->buffers->dma_desc = dma_desc;
	s->dma_desc_phys = dma_desc_phys;
	for (i = 0; i < nb_desc - 1; i++)
		dma_desc[i].ddadr = dma_desc_phys + (i + 1) * DMA_DESC_SIZE;
	dma_desc[i].ddadr = dma_desc_phys;
        AUDPRINTk2("malloc dma memory\n");
	/* Our actual DMA buffers */
	for (frag = 0; frag < s->nbfrags; frag++) {
		audio_buf_t *b = &s->buffers[frag];

		/*
		 * Let's allocate non-cached memory for DMA buffers.
		 * We try to allocate all memory at once.
		 * If this fails (a common reason is memory fragmentation),
		 * then we'll try allocating smaller buffers.
		 */
		if (!buf_size) {
			buf_size = (s->nbfrags - frag) * s->fragsize;
			do {
				dma_buf = consistent_alloc(GFP_KERNEL,
							   buf_size, 
							   &dma_buf_phys);
				if (!dma_buf)
					buf_size -= s->fragsize;
			} while (!dma_buf && buf_size);
			if (!dma_buf)
				goto err;
			b->master = buf_size;
			memzero(dma_buf, buf_size);
		}

		/* 
		 * Set up our checkpoint descriptor.  Since the count 
		 * is always zero, we'll abuse the dsadr and dtadr fields
		 * just in case this one is picked up by the hardware
		 * while processing SOUND_DSP_GETPTR.
		 */
		dma_desc->dsadr = dma_buf_phys;
		dma_desc->dtadr = dma_buf_phys;
		dma_desc->dcmd = DCMD_ENDIRQEN;
		if (s->output && !s->mapped)
			dma_desc->ddadr |= DDADR_STOP;
		b->dma_desc = dma_desc++;

		/* set up the actual data descriptors */
		for (i = 0; (i * MAX_DMA_SIZE) < s->fragsize; i++) {
			dma_desc[i].dsadr = (s->output) ?
				(dma_buf_phys + i*MAX_DMA_SIZE) : s->dev_addr;
			dma_desc[i].dtadr = (s->output) ?
				s->dev_addr : (dma_buf_phys + i*MAX_DMA_SIZE);
			dma_desc[i].dcmd = s->dcmd |
				((s->fragsize < MAX_DMA_SIZE) ?
					s->fragsize : MAX_DMA_SIZE);
		}
		dma_desc += i;

		/* handle buffer pointers */
		b->data = dma_buf;
		dma_buf += s->fragsize;
		dma_buf_phys += s->fragsize;
		buf_size -= s->fragsize;
	}

	s->usr_frag = s->dma_frag = 0;
	s->bytecount = 0;
	s->getptrCount = 0;
	s->fragcount = 0;
	sema_init(&s->sem, (s->output) ? s->nbfrags : 0);
	return 0;

err:
	printk("cotulla-audio: unable to allocate audio memory\n ");
	audio_clear_buf(s);
	return -ENOMEM;
}

/*
 * Our DMA interrupt handler
 */
static void audio_dma_irq(int ch, void *dev_id, struct pt_regs *regs)
{
	audio_stream_t *s = dev_id;
	u_int dcsr;

	dcsr = DCSR(ch);
        AUDPRINTk2("dcsr=0x%lx\n", dcsr);
	DCSR(ch) = dcsr & ~DCSR_STOPIRQEN;

	if (!s->buffers) {
		printk("SSP DMA: wow... received IRQ for channel %d but no buffer exists\n", ch);
		return;
	}

	if (dcsr & DCSR_BUSERR)
		printk("SSP DMA: bus error interrupt on channel %d\n", ch);

	if (dcsr & DCSR_ENDINTR) 
	{
		u_long cur_dma_desc;
		u_int cur_dma_frag;

		/* 
		 * Find out which DMA desc is current.  Note that DDADR
		 * points to the next desc, not the current one.
		 */
		cur_dma_desc = DDADR(ch) - s->dma_desc_phys - DMA_DESC_SIZE;
		/*
		 * Let the compiler nicely optimize constant divisors into
		 * multiplications for the common cases which is much faster.
		 * Common cases: x = 1 + (1 << y) for y = [0..3]
		 */
		switch (s->descs_per_frag) {
		case 2:  cur_dma_frag = cur_dma_desc / (2*DMA_DESC_SIZE); break;
		case 3:  cur_dma_frag = cur_dma_desc / (3*DMA_DESC_SIZE); break;
		case 5:  cur_dma_frag = cur_dma_desc / (5*DMA_DESC_SIZE); break;
		case 9:  cur_dma_frag = cur_dma_desc / (9*DMA_DESC_SIZE); break;
		default: cur_dma_frag =
			    cur_dma_desc / (s->descs_per_frag * DMA_DESC_SIZE);
		}
		
		/* Account for possible wrap back of cur_dma_desc above */
		if (cur_dma_frag >= s->nbfrags)
			cur_dma_frag = s->nbfrags - 1;
		
        	AUDPRINTk2("cur_dma_frag=0x%lx\n", cur_dma_frag);
                AUDPRINTk2("s->dma_frag=0x%lx\n", s->dma_frag);
        	while (s->dma_frag != cur_dma_frag) {
			if (!s->mapped) {
				/* 
				 * This fragment is done - set the checkpoint
				 * descriptor to STOP until it is gets
				 * processed by the read or write function.
				 */
				s->buffers[s->dma_frag].dma_desc->ddadr |= DDADR_STOP;
				up(&s->sem);
			}
			if (++s->dma_frag >= s->nbfrags)
				s->dma_frag = 0;

			/* Accounting */
			s->bytecount += s->fragsize;
			s->fragcount++;
		}

		/* ... and for polling processes */
		wake_up(&s->frag_wq);
	}

	if ((dcsr & DCSR_STOPIRQEN) && (dcsr & DCSR_STOPSTATE))
		wake_up(&s->stop_wq);
        AUDPRINTk2("audio dma irq complete\n");
}

/*
 * Validate and sets up buffer fragments, etc.
 */
static int audio_set_fragments(audio_stream_t *s, int val)
{
	if (s->mapped || DCSR(s->dma_ch) & DCSR_RUN)
		return -EBUSY;
	if (s->buffers)
		audio_clear_buf(s);
	AUDPRINTk2("audio set fragments\n");
        s->nbfrags = (val >> 16) & 0x7FFF;
	val &= 0xffff;
	if (val < 5)
		val = 5;
	if (val > 15)
		val = 15;
	s->fragsize = 1 << val;
	if (s->nbfrags < 2)
		s->nbfrags = 2;
	if (s->nbfrags * s->fragsize > 256 * 1024)
		s->nbfrags = 256 * 1024 / s->fragsize;
	if (audio_setup_buf(s))
		return -ENOMEM;
	return val|(s->nbfrags << 16);
}


/*
 * The fops functions
 */

static int audio_write(struct file *file, const char *buffer,
		       size_t count, loff_t *ppos)
{
	const char *buffer0 = buffer;
	audio_state_t *state = (audio_state_t *)file->private_data;
	audio_stream_t *s = state->output_stream;
	int chunksize, ret = 0;

	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (s->mapped)
		return -ENXIO;
	if (!s->buffers && audio_setup_buf(s))
		return -ENOMEM;
        
        AUDPRINTk2("enter audio write \n");
        AUDPRINTk2("buffer0=0x%lx\n", buffer0);
        AUDPRINTk2("count=%d\n", count);
	while (count > 0) {
		audio_buf_t *b = &s->buffers[s->usr_frag];

		/* Grab a fragment */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (down_trylock(&s->sem))
				break;
		} else {
			ret = -ERESTARTSYS;
			if (down_interruptible(&s->sem))
				break;
		}

		/* Feed the current buffer */
		chunksize = s->fragsize - b->offset;
		if (chunksize > count)
			chunksize = count;
		if (copy_from_user(b->data + b->offset, buffer, chunksize)) {
			up(&s->sem);
			return -EFAULT;
		}
		b->offset += chunksize;
		buffer += chunksize;
		
		count -= chunksize;
		if (b->offset < s->fragsize) {
			up(&s->sem);
			break;
		}

		/* 
		 * Activate DMA on current buffer.
		 * We unlock this fragment's checkpoint descriptor and
		 * kick DMA if it is idle.  Using checkpoint descriptors
		 * allows for control operations without the need for 
		 * stopping the DMA channel if it is already running.
		 */
		b->offset = 0;
                AUDPRINTk2("enable dma run\n");
		b->dma_desc->ddadr &= ~DDADR_STOP;
		if (DCSR(s->dma_ch) & DCSR_STOPSTATE) {
			DDADR(s->dma_ch) = b->dma_desc->ddadr;
			DCSR(s->dma_ch) = DCSR_RUN;
		}

		/* move the index to the next fragment */
		if (++s->usr_frag >= s->nbfrags)
			s->usr_frag = 0;
	}

	if ((buffer - buffer0))
		ret = buffer - buffer0;
        AUDPRINTk2("write bytes=0x%x \n", ret);
	return ret;
}


static int audio_read(struct file *file, char *buffer,
		      size_t count, loff_t *ppos)
{
	char *buffer0 = buffer;
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->input_stream;
	int chunksize, ret = 0;

	if (ppos != &file->f_pos)
		return -ESPIPE;
	if (s->mapped)
		return -ENXIO;
	if (!s->buffers && audio_setup_buf(s))
		return -ENOMEM;

        AUDPRINTk2("enter audio read \n");
        AUDPRINTk2("buffer0=0x%lx\n", buffer0);
        AUDPRINTk2("count=%d\n", count);

	while (count > 0) {
		audio_buf_t *b = &s->buffers[s->usr_frag];

		/* prime DMA */
		if (DCSR(s->dma_ch) & DCSR_STOPSTATE) {
			DDADR(s->dma_ch) = 
				s->buffers[s->dma_frag].dma_desc->ddadr;
			DCSR(s->dma_ch) = DCSR_RUN;
		}

		/* Wait for a buffer to become full */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (down_trylock(&s->sem))
				break;
		} else {
			ret = -ERESTARTSYS;
			if (down_interruptible(&s->sem))
				break;
		}

		/* Grab data from current buffer */
		chunksize = s->fragsize - b->offset;
		if (chunksize > count)
			chunksize = count;
		if (copy_to_user(buffer, b->data + b->offset, chunksize)) {
			up(&s->sem);
			return -EFAULT;
		}
		b->offset += chunksize;
		buffer += chunksize;
		count -= chunksize;
		if (b->offset < s->fragsize) {
			up(&s->sem);
			break;
		}

		/* 
		 * Make this buffer available for DMA again.
		 * We unlock this fragment's checkpoint descriptor and
		 * kick DMA if it is idle.  Using checkpoint descriptors
		 * allows for control operations without the need for 
		 * stopping the DMA channel if it is already running.
		 */
		b->offset = 0;
		b->dma_desc->ddadr &= ~DDADR_STOP;

		/* move the index to the next fragment */
		if (++s->usr_frag >= s->nbfrags)
			s->usr_frag = 0;
	}

	if ((buffer - buffer0))
		ret = buffer - buffer0;

        AUDPRINTk2("read bytes=0x%x \n", ret);
	return ret;
}


static int audio_sync(struct file *file)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->output_stream;
	audio_buf_t *b;
	pxa_dma_desc *final_desc;
	u_long dcmd_save = 0;
	DECLARE_WAITQUEUE(wait, current);

	if (!(file->f_mode & FMODE_WRITE) || !s->buffers || s->mapped)
		return 0;
        AUDPRINTk2("enter audio sync \n");
	/*
	 * Send current buffer if it contains data.  Be sure to send
	 * a full sample count.
	 */
	final_desc = NULL;
	b = &s->buffers[s->usr_frag];
	if (b->offset &= ~3) 
	{
		final_desc = &b->dma_desc[1 + b->offset/MAX_DMA_SIZE];
		b->offset &= (MAX_DMA_SIZE-1);
		dcmd_save = final_desc->dcmd;
		final_desc->dcmd = b->offset | s->dcmd | DCMD_ENDIRQEN; 
		final_desc->ddadr |= DDADR_STOP;
		b->offset = 0;
		b->dma_desc->ddadr &= ~DDADR_STOP;
		if (DCSR(s->dma_ch) & DCSR_STOPSTATE) {
			DDADR(s->dma_ch) = b->dma_desc->ddadr;
			DCSR(s->dma_ch) = DCSR_RUN;
		}
	}

	/* Wait for DMA to complete. */
	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&s->frag_wq, &wait);
	while ((DCSR(s->dma_ch) & DCSR_RUN) && !signal_pending(current)) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&s->frag_wq, &wait);

	/* Restore the descriptor chain. */
	if (final_desc) {
		final_desc->dcmd = dcmd_save;
		final_desc->ddadr &= ~DDADR_STOP;
		b->dma_desc->ddadr |= DDADR_STOP;
	}
	return 0;
}


static unsigned int audio_poll(struct file *file,
			       struct poll_table_struct *wait)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *is = state->input_stream;
	audio_stream_t *os = state->output_stream;
	unsigned int mask = 0;

        AUDPRINTk2("enter audio poll \n");
	if (file->f_mode & FMODE_READ) {
		/* Start audio input if not already active */
		if (!is->buffers && audio_setup_buf(is))
			return -ENOMEM;
		if (DCSR(is->dma_ch) & DCSR_STOPSTATE) {
			DDADR(is->dma_ch) = 
				is->buffers[is->dma_frag].dma_desc->ddadr;
			DCSR(is->dma_ch) = DCSR_RUN;
		}
		poll_wait(file, &is->frag_wq, wait);
	}

	if (file->f_mode & FMODE_WRITE) {
		if (!os->buffers && audio_setup_buf(os))
			return -ENOMEM;
		poll_wait(file, &os->frag_wq, wait);
	}

	if (file->f_mode & FMODE_READ)
		if (( is->mapped && (is->bytecount != is->getptrCount)) ||
		    (!is->mapped && atomic_read(&is->sem.count) > 0))
			mask |= POLLIN | POLLRDNORM;

	if (file->f_mode & FMODE_WRITE)
		if (( os->mapped && (os->bytecount != os->getptrCount)) ||
		    (!os->mapped && atomic_read(&os->sem.count) > 0))
			mask |= POLLOUT | POLLWRNORM;

	return mask;
}


static int audio_ioctl( struct inode *inode, struct file *file,
			uint cmd, ulong arg)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;
	long val;

	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *)arg);

	case SNDCTL_DSP_GETBLKSIZE:
                AUDPRINTk2("get block size\n");
		if (file->f_mode & FMODE_WRITE)
			return put_user(os->fragsize, (int *)arg);
		else
			return put_user(is->fragsize, (int *)arg);

	case SNDCTL_DSP_GETCAPS:
                AUDPRINTk2("get audio capability \n");
		val = DSP_CAP_REALTIME|DSP_CAP_TRIGGER|DSP_CAP_MMAP;
		if (is && os)
			val |= DSP_CAP_DUPLEX;
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETFRAGMENT:
                AUDPRINTk2("set fragment no\n");
		if (get_user(val, (long *) arg))
			return -EFAULT;

		if (file->f_mode & FMODE_READ) 
		{
			int ret = audio_set_fragments(is, val);
			if (ret < 0)
				return ret;
			ret = put_user(ret, (int *)arg);
			if (ret)
				return ret;
		}
		if (file->f_mode & FMODE_WRITE) 
		{
			int ret = audio_set_fragments(os, val);
			if (ret < 0)
				return ret;
			ret = put_user(ret, (int *)arg);
			if (ret)
				return ret;
		}
		return 0;

	case SNDCTL_DSP_SYNC:
		return audio_sync(file);

	case SNDCTL_DSP_SETDUPLEX:
		return 0;

	case SNDCTL_DSP_POST:
		return 0;

	case SNDCTL_DSP_GETTRIGGER:
                AUDPRINTk2("get trigger\n");
		val = 0;
		if (file->f_mode & FMODE_READ && DCSR(is->dma_ch) & DCSR_RUN)
			val |= PCM_ENABLE_INPUT;
		if (file->f_mode & FMODE_WRITE && DCSR(os->dma_ch) & DCSR_RUN)
			val |= PCM_ENABLE_OUTPUT;
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETTRIGGER:
                AUDPRINTk2("set trigger\n");
		if (get_user(val, (int *)arg))
			return -EFAULT;
		if (file->f_mode & FMODE_READ) 
		{
			if (val & PCM_ENABLE_INPUT) 
			{
				if (!is->buffers && audio_setup_buf(is))
					return -ENOMEM;
				if (!(DCSR(is->dma_ch) & DCSR_RUN)) 
				{
					audio_buf_t *b = &is->buffers[is->dma_frag];
					DDADR(is->dma_ch) = b->dma_desc->ddadr;
					DCSR(is->dma_ch) = DCSR_RUN;	
                                        AUDPRINTk2("start read dma channel\n");
				}
			} else{
				DCSR(is->dma_ch) = 0;
			}
		}
		if (file->f_mode & FMODE_WRITE) 
		{
			if (val & PCM_ENABLE_OUTPUT) 
			{
				if (!os->buffers && audio_setup_buf(os))
					return -ENOMEM;
				if (!(DCSR(os->dma_ch) & DCSR_RUN)) {
					audio_buf_t *b = &os->buffers[os->dma_frag];
					DDADR(os->dma_ch) = b->dma_desc->ddadr;
					DCSR(os->dma_ch) = DCSR_RUN;
                                        AUDPRINTk2("start write dma channel\n");
				}
			} else{
				DCSR(os->dma_ch) = 0;
			}
		}
		return 0;

	case SNDCTL_DSP_GETOSPACE:
	case SNDCTL_DSP_GETISPACE:
		{
			audio_buf_info inf = { 0, };
			audio_stream_t *s = (cmd == SNDCTL_DSP_GETOSPACE) ? os : is;
                        AUDPRINTk2("get input and output space size\n");
			if ((s == is && !(file->f_mode & FMODE_READ)) ||
				(s == os && !(file->f_mode & FMODE_WRITE)))
				return -EINVAL;
			if (!s->buffers && audio_setup_buf(s))
				return -ENOMEM;
			
			inf.bytes = atomic_read(&s->sem.count) * s->fragsize;
			inf.bytes -= s->buffers[s->usr_frag].offset;
			inf.fragments = inf.bytes / s->fragsize;
			inf.fragsize = s->fragsize;
			inf.fragstotal = s->nbfrags;

			return copy_to_user((void *)arg, &inf, sizeof(inf));
	    }

	case SNDCTL_DSP_GETOPTR:
	case SNDCTL_DSP_GETIPTR:
	    {
			count_info inf = { 0, };
			audio_stream_t *s = (cmd == SNDCTL_DSP_GETOPTR) ? os : is;
			dma_addr_t ptr;
			int bytecount, offset, flags;
                        AUDPRINTk2("get input and output pointer\n");
			if ((s == is && !(file->f_mode & FMODE_READ)) ||
				(s == os && !(file->f_mode & FMODE_WRITE)))
				return -EINVAL;

			if (DCSR(s->dma_ch) & DCSR_RUN) 
			{
				audio_buf_t *b;
				save_flags_cli(flags);
				ptr = (s->output) ? DSADR(s->dma_ch) : DTADR(s->dma_ch);
				b = &s->buffers[s->dma_frag];
				offset = ptr - b->dma_desc->dsadr;
				if (offset >= s->fragsize)
					offset = s->fragsize - 4;
			} else 
			{
				save_flags(flags);
				offset = 0;
			}
			inf.ptr = s->dma_frag * s->fragsize + offset;
			bytecount = s->bytecount + offset;
			s->getptrCount = s->bytecount;  /* so poll can tell if it changes */
			inf.blocks = s->fragcount;
			s->fragcount = 0;
			restore_flags(flags);
			if (bytecount < 0)
				bytecount = 0;
			inf.bytes = bytecount;
			return copy_to_user((void *)arg, &inf, sizeof(inf));
	    }

	case SNDCTL_DSP_NONBLOCK:
                AUDPRINTk2("set device none block mode\n");
		file->f_flags |= O_NONBLOCK;
		return 0;
	
	case SNDCTL_DSP_GETODELAY:
		{	
			int count = 0, offset;
			int i;
			int flags;
			audio_buf_t *b;
			dma_addr_t ptr;
                        AUDPRINTk2("get output delay\n ");	
			if(!(file->f_mode & FMODE_WRITE))		
				return -EINVAL;
			if(!os->buffers && audio_setup_buf(os))
				return -ENOMEM;
		
			save_flags_cli(flags);
			for(i = 0; i < os->nbfrags; i++){
				/* if contain data */
				if(atomic_read(&os->sem.count) <= 0){
					count += os->fragsize;
				} 
			}
			if (DCSR(os->dma_ch) & DCSR_RUN) 
			{
				ptr = DSADR(os->dma_ch);
				b = &os->buffers[os->dma_frag];
				offset = ptr - b->dma_desc->dsadr;
				if (offset >= os->fragsize)
					offset = os->fragsize - 4;
			} else 
			{
				offset = 0;
			}
			count -= offset;
			restore_flags(flags);
			return put_user(count,(int *)arg);
		}	

	case SNDCTL_DSP_RESET:
                AUDPRINTk2("reset audio data buffer\n");
		if (file->f_mode & FMODE_WRITE) 
			audio_clear_buf(os);
		if (file->f_mode & FMODE_READ)
			audio_clear_buf(is);
		return 0;

	default:
		return state->client_ioctl(inode, file, cmd, arg);
	}

	return 0;
}


static int audio_mmap(struct file *file, struct vm_area_struct *vma)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *s;
	unsigned long size, vma_addr;
	int i, ret;

	if (vma->vm_pgoff != 0)
		return -EINVAL;

	if (vma->vm_flags & VM_WRITE) {
		if (!state->wr_ref)
			return -EINVAL;
		s = state->output_stream;
	} else if (vma->vm_flags & VM_READ) {
		if (!state->rd_ref)
			return -EINVAL;
		s = state->input_stream;
	} else return -EINVAL;

	if (s->mapped)
		return -EINVAL;

        AUDPRINTk2("enter audio mmap \n");
	size = vma->vm_end - vma->vm_start;
	if (size != s->fragsize * s->nbfrags)
		return -EINVAL;
	if (!s->buffers && audio_setup_buf(s))
		return -ENOMEM;
	
	vma_addr = vma->vm_start;
	for (i = 0; i < s->nbfrags; i++) {
		audio_buf_t *buf = &s->buffers[i];
		if (!buf->master)
			continue;
		ret = remap_page_range(vma_addr, buf->dma_desc->dsadr,
				       buf->master, vma->vm_page_prot);
		if (ret)
			return ret;
		vma_addr += buf->master;
	}
	for (i = 0; i < s->nbfrags; i++)
		s->buffers[i].dma_desc->ddadr &= ~DDADR_STOP;
	
	s->mapped = 1;
	return 0;
}


static int audio_release(struct inode *inode, struct file *file)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *is = state->input_stream;
	audio_stream_t *os = state->output_stream;
	
	down(&state->sem);

        AUDPRINTk2("enter audio release \n");
	if (file->f_mode & FMODE_READ) 
	{
		audio_clear_buf(state->input_stream);
		*is->drcmr = 0;
		pxa_free_dma(is->dma_ch);
		state->rd_ref = 0;
	}

	if (file->f_mode & FMODE_WRITE) 
	{
		audio_sync(file);
		audio_clear_buf(state->output_stream);
		*os->drcmr = 0;
		pxa_free_dma(os->dma_ch);
		state->wr_ref = 0;
	}

	if (!AUDIO_ACTIVE(state)) {
	       if (state->hw_shutdown)
		       state->hw_shutdown();
#ifdef CONFIG_PM
	       pm_unregister(state->pm_dev);
#endif
	}

	up(&state->sem);
	return 0;
}

#ifdef CONFIG_PM
static int audio_dma_sleep(audio_stream_t *s)
{
	dmach_t	channel;

	channel = s->dma_ch;
	DCSR(channel)=DCSR_STARTINTR|DCSR_ENDINTR|DCSR_BUSERR;
        AUDPRINTk2("DINT=0x%lx\n", DINT);
        AUDPRINTk2("DCSR(%d)=0x%lx\n", channel, DCSR(channel));
        AUDPRINTk2("s->dcmd=0x%lx\n", s->dcmd);
        AUDPRINTk2("*s->drcmr=0x%lx\n", *s->drcmr);
        AUDPRINTk2("DDADR(%d)=0x%lx\n", channel, DDADR(channel)); 
	return 0;
}

static int audio_dma_wakeup(audio_stream_t *s)
{
	dmach_t	channel;

	channel = s->dma_ch;
	DCSR(channel)=DCSR_STARTINTR|DCSR_ENDINTR|DCSR_BUSERR;
	*s->drcmr = s->dma_ch | DRCMR_MAPVLD;
        AUDPRINTk2("DINT=0x%lx\n", DINT);
        AUDPRINTk2("DCSR(%d)=0x%lx\n", channel, DCSR(channel));
        AUDPRINTk2("s->dcmd=0x%lx\n", s->dcmd);
        AUDPRINTk2("*s->drcmr=0x%lx\n", *s->drcmr);
        AUDPRINTk2("DDADR(%d)=0x%lx\n", channel, DDADR(channel));
	return 0;
}

static int audio_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	audio_state_t *state = (audio_state_t *)pm_dev->data;

	switch (req) {
	case PM_SUSPEND: /* enter D1-D3 */
		if (state->output_stream)
			audio_dma_sleep(state->output_stream);
		if (state->input_stream)
			audio_dma_sleep(state->input_stream);
		if (AUDIO_ACTIVE(state) && state->hw_shutdown)
			if( strncmp(state->input_stream->name,"ezx va",6 ) ){
			        AUDPRINTk1("sleep, shutdown pcap audio hw.\n");
				state->hw_shutdown();
			}
			else{
			        AUDPRINTk1("during call, do not shutdown pcap audio hw.\n");
			}
		break;
	case PM_RESUME:  /* enter D0 */
		if (AUDIO_ACTIVE(state) && state->hw_init)
			if( strncmp(state->input_stream->name,"ezx va",6 ) ){
			        AUDPRINTk1("wake up, init pcap audio hw.\n");
				state->hw_init();
			}
			else{
			        AUDPRINTk1("during call, avoid init pcap audio hw twice.\n");
			}
		if (state->output_stream)
			audio_dma_wakeup(state->output_stream);
		if (state->input_stream)
			audio_dma_wakeup(state->input_stream);
		break;
	}
	return 0;
}
#endif


int cotulla_audio_attach(struct inode *inode, struct file *file,
			 audio_state_t *state)
{
	audio_stream_t *is = state->input_stream;
	audio_stream_t *os = state->output_stream;
	int err;

	down(&state->sem);

	/* access control */
	err = -ENODEV;
	if ((file->f_mode & FMODE_WRITE) && !os)
		goto out;
	if ((file->f_mode & FMODE_READ)	&& !is)
		goto out;

	err = -EBUSY;
	if ((file->f_mode & FMODE_WRITE) && state->wr_ref)
		goto out;
	if ((file->f_mode & FMODE_READ) && state->rd_ref)
		goto out;

        AUDPRINTk2("request dma channels \n");
	/* request DMA channels */ 
	if (file->f_mode & FMODE_WRITE)
	{
		err = pxa_request_dma(os->name, DMA_PRIO_LOW, audio_dma_irq, os);
		if (err < 0)
			goto out;
		os->dma_ch = err;
	}

	if (file->f_mode & FMODE_READ)
	{
		err = pxa_request_dma(is->name, DMA_PRIO_LOW, audio_dma_irq, is);
		if (err < 0){
			if(file->f_mode & FMODE_WRITE){
				*os->drcmr = 0;
				pxa_free_dma(os->dma_ch);
			}
			goto out;
		}
		is->dma_ch = err;
	}
	
	err = -ENODEV;
	/* now complete initialisation */
	if(!AUDIO_ACTIVE(state)){
		if(state->hw_init){
			err = state->hw_init();
			if(err<0){
				if(file->f_mode & FMODE_WRITE){
					*os->drcmr = 0;
					pxa_free_dma(os->dma_ch);
				}
				if(file->f_mode & FMODE_READ){
					*is->drcmr = 0;
					pxa_free_dma(is->dma_ch);
				}
				goto out;
			}
		}

#ifdef CONFIG_PM
		state->pm_dev = pm_register(PM_SYS_DEV, 0, audio_pm_callback);
		if (state->pm_dev)
			state->pm_dev->data = state;
#endif
	}
        AUDPRINTk2("init data structure \n");
	file->private_data	= state;
	file->f_op->release	= audio_release;
	file->f_op->write	= audio_write;
	file->f_op->read	= audio_read;
	file->f_op->mmap	= audio_mmap;
	file->f_op->poll	= audio_poll;
	file->f_op->ioctl	= audio_ioctl;
	file->f_op->llseek	= no_llseek;

	if ((file->f_mode & FMODE_WRITE)) 
	{
		state->wr_ref = 1;
		os->fragsize = AUDIO_FRAGSIZE_DEFAULT;
		os->nbfrags = AUDIO_NBFRAGS_DEFAULT;
		os->output = 1;
		os->mapped = 0;
		init_waitqueue_head(&os->frag_wq);
		init_waitqueue_head(&os->stop_wq);
		*os->drcmr = os->dma_ch | DRCMR_MAPVLD;
                AUDPRINTk2("*os->drcmr=0x%lx\n", *os->drcmr);
	}
	if (file->f_mode & FMODE_READ) 
	{
		state->rd_ref = 1;
		is->fragsize = AUDIO_FRAGSIZE_DEFAULT;
		is->nbfrags = AUDIO_NBFRAGS_DEFAULT;
		is->output = 0;
		is->mapped = 0;
		init_waitqueue_head(&is->frag_wq);
		init_waitqueue_head(&is->stop_wq);
		*is->drcmr = is->dma_ch | DRCMR_MAPVLD;
                AUDPRINTk2("*is->drcmr=0x%lx\n", *is->drcmr);
	}
	err = 0;
	up(&state->sem);
        AUDPRINTk2("audio device open success\n");
	return err;
out:
	up(&state->sem);
	printk("audio device is busy or there is no audio device\n");
	return err;
}

EXPORT_SYMBOL(cotulla_audio_attach);


