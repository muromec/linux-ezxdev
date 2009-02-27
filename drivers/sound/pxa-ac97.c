/*
 *  linux/drivers/sound/pxa-ac97.c -- AC97 interface for the Cotula chip
 *
 *  Author:	Nicolas Pitre
 *  Created:	Aug 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/ac97_codec.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/dma.h>

#include <asm/arch/mainstone.h>

#include "pxa-audio.h"

#if DEBUG
static unsigned int pxa_ac97_dbg = 1;
#else
#define pxa_ac97_dbg    0
#endif

static int suspended = 0; /* So as not to suspend more than once */

struct dma_register_s {
	int ddadr, drcmr, dcsr;
	int saved;
};

/* Save the registers for the input stream channel and/or the output
   stream channel */
static struct dma_register_s saved_dma_register_os;
static struct dma_register_s saved_dma_register_is;

/*
 * The codec register read operation requires 3 read cycles on PXA250 in order
 * to guarrantee that good read data can be returned.
 *            _             _              _            _
 *sync: _____| |_addr______| |_data1______| |__data2___| |__data_n__
 *SDONE:__            _              _              _______________
 *        |_addr1____| |__addr2_____| |__addr_n____|
 *       ^           
 *       First read begins
 *                   ^ SDONE usually goes true in the latter half of AC link frame
 *                     ^ Second read begins, but data from codec hasn't arrived yet!
 *                                  ^ second read ends, from 1 to 3 frames AFTER frame
 *                                    in which the address goes out!
 *                                    ^ Third read begins from one to 3 frames after the
 *                                      initial frame, data from codec guarranteed to be 
 *                                      available by this time.
 *                                                 ^ read cycle ends.
 * Note how reads can be pipelined, possibly useful for reading touch panel
 * control registers or rapid sampling of codec gpio lines. 
 */

static DECLARE_MUTEX(car_mutex);
static DECLARE_WAIT_QUEUE_HEAD(gsr_wq);
static volatile long gsr_bits;

#define wait_event_timeout(wq, condition, timeout)			\
({									\
	long __ret = timeout;						\
	if (!(condition)) { 						\
		struct task_struct *__tsk = current;			\
		DECLARE_WAITQUEUE(__wait, __tsk);			\
		add_wait_queue(&wq, &__wait);				\
		for (;;) {						\
			set_task_state(__tsk, TASK_UNINTERRUPTIBLE);	\
			if (condition)					\
				break;					\
			__ret = schedule_timeout(__ret);		\
			if (!__ret)					\
				break;					\
		}							\
		remove_wait_queue(&wq, &__wait);			\
		set_current_state(TASK_RUNNING);			\
	}								\
	__ret;								\
})

static u16 pxa_ac97_read(struct ac97_codec *codec, u8 reg)
{
	unsigned short val = -1;
	volatile u32 *reg_addr;

	down(&car_mutex);
	if (CAR & CAR_CAIP) {
		printk(KERN_CRIT"%s: CAR_CAIP already set\n", __FUNCTION__);
		goto out;
	}

	/* set up primary or secondary codec space */
	reg_addr = (codec->id & 1) ? &SAC_REG_BASE : &PAC_REG_BASE;
	reg_addr += (reg >> 1);

	/* start read access across the ac97 link */
	gsr_bits = 0;
	val = *reg_addr;
	if (reg == AC97_GPIO_STATUS)
		goto out;
	wait_event_timeout(gsr_wq, gsr_bits & GSR_SDONE, 1);
	if (!gsr_bits & GSR_SDONE) {
		printk(KERN_ERR "%s: read error (ac97_reg=%d GSR=%#lx)\n",
				__FUNCTION__, reg, gsr_bits);
		val = -1;
		goto out;
	}

	/* valid data now */
	gsr_bits = 0;
	val = *reg_addr;			
	/* but we've just started another cycle... */
	wait_event_timeout(gsr_wq, gsr_bits & GSR_SDONE, 1);

out:	up(&car_mutex);
	return val;
}

static void pxa_ac97_write(struct ac97_codec *codec, u8 reg, u16 val)
{
	volatile u32 *reg_addr;

	down(&car_mutex);

	if (CAR & CAR_CAIP) {
		printk(KERN_CRIT "%s: CAR_CAIP already set\n", __FUNCTION__);
		goto out;
	}

	/* set up primary or secondary codec space */
	reg_addr = (codec->id & 1) ? &SAC_REG_BASE : &PAC_REG_BASE;
	reg_addr += (reg >> 1);
	gsr_bits = 0;
	*reg_addr = val;
	wait_event_timeout(gsr_wq, gsr_bits & GSR_CDONE, 1);
	if (!gsr_bits & GSR_SDONE)
		printk(KERN_ERR "%s: write error (ac97_reg=%d GSR=%#lx)\n",
				__FUNCTION__, reg, gsr_bits);

out:	up(&car_mutex);
}

static void pxa_ac97_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	long status;

	status = GSR;
	if (status) {
		GSR = status;
		gsr_bits |= status;
		wake_up(&gsr_wq);

#ifdef CONFIG_CPU_BULVERDE
		/* Although we don't use those we still need to clear them
		   since they tend to spuriously trigger when MMC is used
		   (hardware bug? go figure)... */
		MISR = (1 << 3);  /* MISR_EOC */
		PISR = (1 << 3);  /* PISR_EOC */
		MCSR = (1 << 3);  /* MCSR_EOC */
#endif
	}
}

static struct ac97_codec pxa_ac97_codec = {
	codec_read:	pxa_ac97_read,
	codec_write:	pxa_ac97_write,
};

static void pxa_ac97_reset(void)
{
	/* First, try cold reset */
	GCR &=  GCR_COLD_RST;  /* clear everything but nCRST */
	GCR &= ~GCR_COLD_RST;  /* then assert nCRST */

	gsr_bits = 0;
#ifdef CONFIG_CPU_BULVERDE
	/* PXA27x Developers Manual section 13.5.2.2.1 */
	CKEN |= (1 << 31);
	udelay(5);
	CKEN &= ~(1 << 31);
	GCR = GCR_COLD_RST;
	udelay(50);
#else
	GCR = GCR_COLD_RST;
	GCR |= GCR_CDONE_IE|GCR_SDONE_IE;
	wait_event_timeout(gsr_wq, gsr_bits & (GSR_PCR | GSR_SCR), 1);
#endif

	if (!((GSR | gsr_bits) & (GSR_PCR | GSR_SCR))) {
		printk(KERN_INFO "%s: cold reset timeout (GSR=%#lx)\n",
				 __FUNCTION__, gsr_bits);

		/* let's try warm reset */
		gsr_bits = 0;
#ifdef CONFIG_CPU_BULVERDE
		/* warm reset broken on Bulverde,
		   so manually keep AC97 reset high */
		GPSR(113) = GPIO_bit(113);
		set_GPIO_mode(113 | GPIO_OUT); 
		udelay(10);
		GCR |= GCR_WARM_RST;
		set_GPIO_mode(113 | GPIO_ALT_FN_2_OUT);
		udelay(50);
#else
		GCR |= GCR_WARM_RST|GCR_PRIRDY_IEN|GCR_SECRDY_IEN;;
		wait_event_timeout(gsr_wq, gsr_bits & (GSR_PCR | GSR_SCR), 1);
#endif			

		if (!((GSR | gsr_bits) & (GSR_PCR | GSR_SCR)))
			printk(KERN_INFO "%s: warm reset timeout (GSR=%#lx)\n",
					 __FUNCTION__, gsr_bits);
	}

	GCR &= ~(GCR_PRIRDY_IEN|GCR_SECRDY_IEN);
	GCR |= GCR_SDONE_IE|GCR_CDONE_IE;
}

static DECLARE_MUTEX(pxa_ac97_mutex);
static int pxa_ac97_refcount;

int pxa_ac97_get(struct ac97_codec **codec)
{
	int ret;

	*codec = NULL;
	down(&pxa_ac97_mutex);

	if (!pxa_ac97_refcount) {
		ret = request_irq(IRQ_AC97, pxa_ac97_irq, 0, "AC97", NULL);
		if (ret)
			return ret;

		CKEN |= CKEN2_AC97; 

#ifdef CONFIG_ARCH_MAINSTONE
		/*
		 * On Mainstone, we enable the on board audio amp and
		 * route AC97_SYSCLK via GPIO45 to the audio daughter card
		 */
		MST_MSCWR2 &= ~MST_MSCWR2_AC97_SPKROFF;
		set_GPIO_mode(45 | GPIO_ALT_FN_1_OUT);
#endif

		set_GPIO_mode(GPIO31_SYNC_AC97_MD);
		set_GPIO_mode(GPIO30_SDATA_OUT_AC97_MD);
		set_GPIO_mode(GPIO28_BITCLK_AC97_MD);
		set_GPIO_mode(GPIO29_SDATA_IN_AC97_MD);

#ifdef CONFIG_CPU_BULVERDE
		/* Use GPIO 113 as AC97 Reset on Bulverde */
		set_GPIO_mode(113 | GPIO_ALT_FN_2_OUT);
#endif

		pxa_ac97_reset();
		ret = ac97_probe_codec(&pxa_ac97_codec);
		if (ret != 1) {
			free_irq(IRQ_AC97, NULL);
			GCR = GCR_ACLINK_OFF;
			CKEN &= ~CKEN2_AC97; 
			return ret;
		}

		// need little hack for UCB1400 (should be moved elsewhere)
		pxa_ac97_write(&pxa_ac97_codec,AC97_EXTENDED_STATUS,1);
		//pxa_ac97_write(&pxa_ac97_codec, 0x6a, 0x1ff7);
		//pxa_ac97_write(&pxa_ac97_codec, 0x6a, 0x0050);
		//pxa_ac97_write(&pxa_ac97_codec, 0x6c, 0x0030);
	}

	pxa_ac97_refcount++;
	up(&pxa_ac97_mutex);
	*codec = &pxa_ac97_codec;
	return 0;
}

void pxa_ac97_put(void)
{
	down(&pxa_ac97_mutex);
	if (pxa_ac97_refcount > 0) {
		pxa_ac97_refcount--;
		if (!pxa_ac97_refcount) {
#ifdef CONFIG_ARCH_MAINSTONE
			MST_MSCWR2 |= MST_MSCWR2_AC97_SPKROFF;
#endif
			/* 
			 * DOC CHANGE: D5. AC97: AC97 out channels are
			 * transmitting bad data
			 *
			 * The following steps initiate a power down:
			 * 1. Write 0x26 and initiate power-down
			 * 2. Wait for GSR[CDONE] indicating that the
			 *    command has completed.
			 * 3. Set GCR[ACOFF] to shut down the AC-link
			 * 4. Set CKEN[31] in the Clock Enable register
			 * 5. Wait for GSR[ACOFFD], indicating that the
			 *    AC-link shutdown is complete
			 * 6. Clear CKEN[31]
			 */
			/* 1 & 2 */
			pxa_ac97_write(&pxa_ac97_codec, AC97_POWER_CONTROL,
					    0x1000);
			/* 3 */
			GCR |= GCR_ACLINK_OFF;
			/* 4 */
			CKEN |= CKEN31_AC97CONF;
			/* 5 */
			while (!(GSR & GSR_ACOFFD));
			/* 6 */
			CKEN &= ~CKEN31_AC97CONF;
			/* now safe to turn off clock */
			CKEN &= ~CKEN2_AC97; 
			free_irq(IRQ_AC97, NULL);
		}
	} else {
		printk(KERN_CRIT "%s: pxa_ac97_refcount would go negative\n", __FUNCTION__);
	}
	up(&pxa_ac97_mutex);
}

EXPORT_SYMBOL(pxa_ac97_get);
EXPORT_SYMBOL(pxa_ac97_put);

/*
 * Audio Mixer stuff
 */

static audio_state_t ac97_audio_state;
static audio_stream_t ac97_audio_in;

static int mixer_ioctl( struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret;

	ret = pxa_ac97_codec.mixer_ioctl(&pxa_ac97_codec, cmd, arg);
	if (ret)
		return ret;

	/* We must snoop for some commands to provide our own extra processing */
	switch (cmd) {
	case SOUND_MIXER_WRITE_RECSRC:
		/*
		 * According to the PXA250 spec, mic-in should use different
		 * DRCMR and different AC97 FIFO.
		 * Unfortunately current UCB1400 versions (up to ver 2A) don't
		 * produce slot 6 for the audio input frame, therefore the PXA
		 * AC97 mic-in FIFO is always starved.
		 */
#if 0
		ret = get_user(val, (int *)arg);
		if (ret)
			return ret;
		pxa_audio_clear_buf(&ac97_audio_in);
		*ac97_audio_in.drcmr = 0;
		if (val & (1 << SOUND_MIXER_MIC)) {
			ac97_audio_in.dcmd = DCMD_RXMCDR;
			ac97_audio_in.drcmr = &DRCMRRXMCDR;
			ac97_audio_in.dev_addr = __PREG(MCDR);
		} else {
			ac97_audio_in.dcmd = DCMD_RXPCDR;
			ac97_audio_in.drcmr = &DRCMRRXPCDR;
			ac97_audio_in.dev_addr = __PREG(PCDR);
		}
		if (ac97_audio_state.rd_ref)
			*ac97_audio_in.drcmr =
				ac97_audio_in.dma_ch | DRCMR_MAPVLD;
#endif
		break;
	}
	return 0;
}

static struct file_operations mixer_fops = {
	ioctl:		mixer_ioctl,
	llseek:		no_llseek,
	owner:		THIS_MODULE
};

/*
 * AC97 codec ioctls
 */

static int codec_adc_rate = 48000;
static int codec_dac_rate = 48000;

static int ac97_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int ret;
	long val = 0;

	switch(cmd) {
	case SNDCTL_DSP_STEREO:
		ret = get_user(val, (int *) arg);
		if (ret)
			return ret;
		/* FIXME: do we support mono? */
		ret = (val == 0) ? -EINVAL : 1;
		return put_user(ret, (int *) arg);

	case SNDCTL_DSP_CHANNELS:
	case SOUND_PCM_READ_CHANNELS:
		/* FIXME: do we support mono? */
		return put_user(2, (long *) arg);

	case SNDCTL_DSP_SPEED:
		ret = get_user(val, (long *) arg);
		if (ret)
			return ret;
		if (file->f_mode & FMODE_READ)
			codec_adc_rate = ac97_set_adc_rate(&pxa_ac97_codec, val);
		if (file->f_mode & FMODE_WRITE)
			codec_dac_rate = ac97_set_dac_rate(&pxa_ac97_codec, val);
		/* fall through */

	case SOUND_PCM_READ_RATE:
		if (file->f_mode & FMODE_READ)
			val = codec_adc_rate;
		if (file->f_mode & FMODE_WRITE)
			val = codec_dac_rate;
		return put_user(val, (long *) arg);

	case SNDCTL_DSP_SETFMT:
	case SNDCTL_DSP_GETFMTS:
		/* FIXME: can we do other fmts? */
		return put_user(AFMT_S16_LE, (long *) arg);

	default:
		/* Maybe this is meant for the mixer (As per OSS Docs) */
		return mixer_ioctl(inode, file, cmd, arg);
	}
	return 0;
}


/*
 * Audio stuff
 */

static audio_stream_t ac97_audio_out = {
	name:			"AC97 audio out",
	dcmd:			DCMD_TXPCDR,
	drcmr:			&DRCMRTXPCDR,
	dev_addr:		__PREG(PCDR),
};

static audio_stream_t ac97_audio_in = {
	name:			"AC97 audio in",
	dcmd:			DCMD_RXPCDR,
	drcmr:			&DRCMRRXPCDR,
	dev_addr:		__PREG(PCDR),
};

static audio_state_t ac97_audio_state = {
	output_stream:		&ac97_audio_out,
	input_stream:		&ac97_audio_in,
	client_ioctl:		ac97_ioctl,
	sem:			__MUTEX_INITIALIZER(ac97_audio_state.sem),
	suspend_wq:             __WAIT_QUEUE_HEAD_INITIALIZER(ac97_audio_state.suspend_wq),
	stop_dma_for_suspend:   0
};

static int ac97_audio_open(struct inode *inode, struct file *file)
{
	return pxa_audio_attach(inode, file, &ac97_audio_state);
}

/*
 * Missing fields of this structure will be patched with the call
 * to pxa_audio_attach().
 */

static struct file_operations ac97_audio_fops = {
	open:		ac97_audio_open,
	owner:		THIS_MODULE
};


#ifdef CONFIG_DPM
#include <linux/device.h>

static int ac97_suspend(struct device *dev, u32 state, u32 level);
static int ac97_resume(struct device *dev, u32 level);

static struct device_driver ac97_driver_ldm = {
	name:          "ac97",
	devclass:      NULL,
	probe:         NULL,
	suspend:       ac97_suspend,
	resume:        ac97_resume,
	scale:         0,
	remove:        NULL,
};

static struct device ac97_device_ldm = {
	name:         "AC97 Sound Controller",
	bus_id:       "ac97",
	driver:       NULL,
	power_state:  DPM_POWER_ON,
	constraints:   NULL,
};

static void
ac97_ldm_register(void)
{
	extern void pxaopb_driver_register(struct device_driver *driver);
	extern void pxaopb_device_register(struct device *device);
	
	pxaopb_driver_register(&ac97_driver_ldm);
	pxaopb_device_register(&ac97_device_ldm);
}

static void
ac97_ldm_unregister(void)
{
	extern void pxaopb_driver_unregister(struct device_driver *driver);
	extern void pxaopb_device_unregister(struct device *device);
	
	pxaopb_device_unregister(&ac97_device_ldm);
	pxaopb_driver_unregister(&ac97_driver_ldm);
}

static int
ac97_resume(struct device *dev, u32 level)
{
	struct ac97_codec *dummy;
	audio_state_t *state = &ac97_audio_state;
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;
	/* wake_up will be set if there were saved registers for
	   either the input_stream's DMA channel or the
	   output_stream's DMA channel */
	int wake_up = 0;

	switch (level) {
	case RESUME_POWER_ON:
	case RESUME_RESTORE_STATE:
	case RESUME_ENABLE:

		if (!suspended) {
			return 0;
		}
		
		suspended = 0;

		/* Restore saved DMA registers */
		if (saved_dma_register_os.saved) {
			wake_up = 1;
			*os->drcmr = saved_dma_register_os.drcmr;
			DDADR(os->dma_ch) = saved_dma_register_os.ddadr;
			DCSR(os->dma_ch) = saved_dma_register_os.dcsr;
			saved_dma_register_os.saved = 0;
		}
		if (saved_dma_register_is.saved) {
			wake_up = 1;
			*is->drcmr = saved_dma_register_is.drcmr;
			DDADR(is->dma_ch) = saved_dma_register_is.ddadr;
			DCSR(is->dma_ch) = saved_dma_register_is.dcsr;
			saved_dma_register_is.saved = 0;
		}

		pxa_ac97_get(&dummy);
		ac97_restore_state(dummy);

		/* reset DAC and ADC rates */
		codec_dac_rate = ac97_set_dac_rate(dummy, codec_dac_rate);
		codec_adc_rate = ac97_set_adc_rate(dummy, codec_adc_rate);

		if (wake_up) {
			/* Clear the flag and kick the wait queue */
			state->stop_dma_for_suspend = 0;
			
			/* Wake up pxa-audio's audio_write/audio_read */
			wake_up_interruptible(&state->suspend_wq);
		}
		break;
	}
		
	return 0;
}

static int
ac97_suspend(struct device *dev, u32 state, u32 level)
{
	audio_state_t *audio_state = &ac97_audio_state;
	audio_stream_t *os = audio_state->output_stream;
	audio_stream_t *is = audio_state->input_stream;

	switch (level) {
	case SUSPEND_NOTIFY:
	case SUSPEND_SAVE_STATE:
		break;
	case SUSPEND_DISABLE:
	case SUSPEND_POWER_DOWN:

		if (suspended) {
			return 0;
		}
		suspended = 1;

		if ((os && os->dma_ch) || (is && is->dma_ch)) {
			/* Block pxa-audio.c's audio_write/audio_read */
			audio_state->stop_dma_for_suspend = 1;
		}

		/* Save registers for output stream's DMA channel */
		if (os && os->dma_ch) {
			/* If there is an outputstream and an attached
			   DMA channel on the output stream */

			/* Must save these registers for when we wake up */
			/* set saved flag so we know to restore these */
			saved_dma_register_os.saved = 1;
			saved_dma_register_os.dcsr = DCSR(os->dma_ch);
			saved_dma_register_os.drcmr = *os->drcmr;
			saved_dma_register_os.ddadr = DDADR(os->dma_ch);
		}

		/* Save registers for input stream's DMA channel */
		if (is && is->dma_ch) {
			/* If there is an outputstream and an attached
			   DMA channel on the output stream */

			/* Must save these registers for when we wake up */
			/* set saved flag so we know to restore these */
			saved_dma_register_is.saved = 1;
			saved_dma_register_is.dcsr = DCSR(is->dma_ch);
			saved_dma_register_is.drcmr = *is->drcmr;
			saved_dma_register_is.ddadr = DDADR(is->dma_ch);
		}
		
		pxa_ac97_put();
		break;
	}
	return 0;
}

#endif /* CONFIG_DPM */

static int __init pxa_ac97_init(void)
{
	int ret;
	struct ac97_codec *dummy;

	ret = pxa_ac97_get(&dummy);
	if (ret)
		return ret;

	ac97_audio_state.dev_dsp = register_sound_dsp(&ac97_audio_fops, -1);
	pxa_ac97_codec.dev_mixer = register_sound_mixer(&mixer_fops, -1);
#ifdef CONFIG_DPM
	ac97_ldm_register();
#endif /* CONFIG_DPM */

	return 0;
}

static void __exit pxa_ac97_exit(void)
{
#ifdef CONFIG_DPM
	ac97_ldm_unregister();
#endif /* CONFIG_DPM */
	unregister_sound_dsp(ac97_audio_state.dev_dsp);
	unregister_sound_mixer(pxa_ac97_codec.dev_mixer);
	pxa_ac97_put();
}


module_init(pxa_ac97_init);
module_exit(pxa_ac97_exit);

