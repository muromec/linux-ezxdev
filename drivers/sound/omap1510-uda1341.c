/*
 * Glue audio driver for the TI OMAP1510 & Philips UDA1341 codec.
 *
 * Copyright (c) 2000 Nicolas Pitre <nico@cam.org>
 * Copyright (C) 2001, Steve Johnson <stevej@ridgerun.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 *
 * This is the machine specific part of the TI OMAP1510 support.
 * This driver makes use of the UDA1341 and the omap1510-audio modules.
 *
 * History:
 *
 * 2000-05-21	Nicolas Pitre	Initial UDA1341 driver release.
 *
 * 2000-07-??	George France	Bitsy support.
 *
 * 2000-12-13	Deborah Wallach	Fixed power handling for iPAQ/h3600
 *
 * 2001-06-03	Nicolas Pitre	Made this file a separate module, based on
 * 				the former sa1100-uda1341.c driver.
 *
 * 2001-07-13	Nicolas Pitre	Fixes for all supported samplerates.
 *
 * 2001-10-31   Steve Johnson   Modified for OMAP1510.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/l3/l3.h>
#include <linux/l3/uda1341.h>

#include <asm/semaphore.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/arch/fpga.h>
#include <asm/arch/ck.h>

#include "omap-audio.h"


#undef DEBUG
#ifdef DEBUG
#define DPRINTK( x... )  printk( ##x )
#else
#define DPRINTK( x... )
#endif


#define AUDIO_NAME		"OMAP1510_UDA1341"

#define AUDIO_RATE_DEFAULT	44100

/*
  Local prototypes.
*/
static void omap1510_audio_init(void *dummy);
static void omap1510_audio_shutdown(void *dummy);
static int omap1510_audio_ioctl(struct inode *inode, struct file *file,
                                uint cmd, ulong arg);

static struct l3_client uda1341;

#if OLD_DMA
static audio_stream_t output_stream = {
        dma_ch: -1,
};
static audio_stream_t input_stream = {
        dma_ch: -1,
};
#else
static audio_stream_t output_stream;
static audio_stream_t input_stream;
#endif

static audio_state_t audio_state = {
	output_stream:	&output_stream,
	output_dma:	eAudioTx,
	output_id:	"UDA1341 out",
	input_stream:	&input_stream,
	input_dma:	eAudioRx,
	input_id:	"UDA1341 in",
//	need_tx_for_rx:	1,  TODO Try this once receive is working.
	need_tx_for_rx:	0,
	hw_init:	omap1510_audio_init,
	hw_shutdown:	omap1510_audio_shutdown,
	client_ioctl:	omap1510_audio_ioctl,
	sem:		__MUTEX_INITIALIZER(audio_state.sem),
};



static int
mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
printk(__FUNCTION__ "called\n");
	/*
	 * We only accept mixer (type 'M') ioctls.
	 */
	if (_IOC_TYPE(cmd) != 'M')
		return -EINVAL;

	return l3_command(&uda1341, cmd, (void *)arg);
}

static struct file_operations omap1510_mixer_fops = {
	ioctl:		mixer_ioctl,
	owner:		THIS_MODULE
};


/*
 * Audio interface
 */

static long audio_samplerate = AUDIO_RATE_DEFAULT;

static void omap1510_set_samplerate(long val)
{
	struct uda1341_cfg cfg;
        u8 fpga;

printk(__FUNCTION__ "called\n");
	/* We don't want to mess with clocks when frames are in flight */
        // TODO - could call omap1510_dma_flush_all, or could poll on
        // enable bit to wait for DMA writes to stop.

	/* wait for any frame to complete */
	udelay(125);

        /*
         * We have the following clock sources:
         * 12.288 MHz and 16.9344 MHz.
         * We have dividers in the FPGA (1, 2, 4), and in
         * the codec.
         *   clock   epld div  codec_div  freq
         *  12.288     1        256       48K
         *   "         4        384        8K
         *  16.9344    1        384       44.1K
         *   "         2        384       22.05K
         *   "         4        384       11.025K
         */

        if (val >= 48000)
                val = 48000;
        else if (val >= 44100)
                val = 44100;
        else if (val >= 22050)
                val = 22050;
        else if (val >= 11025)
                val = 11025;
        else 
                val = 8000;

        /* Set the external clock generator */
        switch (val) {
        case 48000:
        case 8000:
                /* 12.288 MHz */
                fpga = fpga_read(OMAP1510P1_FPGA_AUDIO);
                if (fpga & 0x4) {
                        fpga &= ~0x4;
                        fpga_write(fpga, OMAP1510P1_FPGA_AUDIO);
                }
                break;
        default:
                /* 16.3944 MHz */
                fpga = fpga_read(OMAP1510P1_FPGA_AUDIO);
                if ((fpga & 0x4) == 0) {
                        fpga |= 0x4;
                        fpga_write(fpga, OMAP1510P1_FPGA_AUDIO);
                }
                break;
        }

        /* Select the clock divisor */
        switch (val) {
#if 0
        case 
        case 
		cfg.fs = 512;
                break;
#endif
        case 48000:
		cfg.fs = 256;
                break;
        default:
		cfg.fs = 384;
                break;
        }

	cfg.format = FMT_I2S;
	l3_command(&uda1341, L3_UDA1341_CONFIGURE, &cfg);
	audio_samplerate = val;
}

static void omap1510_audio_init(void *dummy)
{
        dma_regs_t *dma_regs = output_stream.dma_regs;
        u8 fpga;

printk(__FUNCTION__ "called\n");
        /*                              
          Init FPGA.       
          Turn on Audio amplifier and set 12.288 MHz clock             
        */                           
        fpga = fpga_read(OMAP1510P1_FPGA_POWER);
        fpga &= 0xfe;
        fpga_write(fpga, OMAP1510P1_FPGA_POWER);
        fpga = 0xc;        
        fpga_write(fpga, OMAP1510P1_FPGA_AUDIO);              


#ifndef CONFIG_DSP_MCBSP1

        /*
          Now here's an ugly hack.  To use McBSP1, you need to enable
          a clock on the DSP.  So enable the MPUI, set the clock,
          and start the DSP.  

          An even uglier, evil hack.  If this is going to take the DSP
          out of reset, shove an idle loop at the reset vector
          and make it loop instead of crash.  You will still see
          a DSP watchdog timer go off.
        */

        {
                u16 tmp;
                u8 c55_start[] = { 0x7A, 0x00, 0x00, 0x0C, 0x4A, 0x7A, 0x20, 0x20, 0x20, 0x20 };

                tmp = inw(ARM_RSTCT1);
		// check if DSP is up
                if (!(tmp & (ARM_RSTCT1_DSP_RST|ARM_RSTCT1_DSP_EN))) {
                        if (!(tmp & ARM_RSTCT1_DSP_RST)) {   // MPUI in reset
                                tmp |= ARM_RSTCT1_DSP_RST;
                                outw(tmp, ARM_RSTCT1);
                                ck_enable(api_ck);
                        }
			// REVISIT: I'm not finding this in the OMAP1509 TRM:
                        tmp = inw(0xe1008008);
                        if (0 == (tmp & 0x2)) {  // DSP CLKM enable
                                tmp |= 0x2;
                                outw(tmp, 0xe1008008);
                        }
                        tmp = inw(0xe1008014);
                        if (0 == (tmp & 0x1)) {  // DSP PER_EN bit
                                tmp |= 0x1;
                                outw(tmp, 0xe1008014);
                        }
                        tmp = inw(ARM_CKCTL);   // Enable DSP
                        tmp |= 0x2000;
                        outw(tmp, ARM_CKCTL);
                        // Write C55 code at reset vector.
                        memcpy((OMAP_DSP_BASE + 0x4c000), 
                               &c55_start, 
                               sizeof(c55_start));
                        outw(0x5, MPUI_DSP_BOOT_CONFIG); // Set DSP boot mode
                        tmp = inw(ARM_RSTCT1);   // take DSP out of reset
                        tmp |= ARM_RSTCT1_DSP_RST|ARM_RSTCT1_DSP_EN;
                        outw(tmp, ARM_RSTCT1);                         
                } else {
			// DSP's up, just check the clock/per bits
                        tmp = inw(0xe1008008);
                        if (0 == (tmp & 0x2)) {  // DSP CLKM enable
                                tmp |= 0x2;
                                outw(tmp, 0xe1008008);
                        }
                        tmp = inw(0xe1008014);
                        if (0 == (tmp & 0x1)) {  // DSP PER_EN bit
                                tmp |= 0x1;
                                outw(tmp, 0xe1008014);
                        }
                }
        }          


        /*  
            While we're waiting for the UDA1341 to wake up, let's configure the
            DMA channel and MCBSP.
        */


        // Setup DMA channel to McBSP1 audio Tx.
        dma_regs->csdp = 0x0a01;
        dma_regs->ccr = 0x1000 | audio_state.output_dma;            // source auto increment, don't enable yet
        dma_regs->cicr = 0x0b;
        dma_regs->cdsa_l = ((OMAP1510_MCBSP1_BASE + 0x806) & 0xffff);     //McBSP1 DXR1
        dma_regs->cdsa_u = ((OMAP1510_MCBSP1_BASE + 0x806) >> 16);
        dma_regs->cfn = 0x1;
        omap_dma_setup(audio_state.output_dma, eDmaOut);

        // Initialize McBSP channel
        outw(0x0000, OMAP1510_MCBSP1_BASE + 0x80a);  // SPCR1
        outw(0x0000, OMAP1510_MCBSP1_BASE + 0x808);  // SPCR2
        outw(0x0f03, OMAP1510_MCBSP1_BASE + 0x824);  // PCR0
        outw(0x0040, OMAP1510_MCBSP1_BASE + 0x80e);  // RCR1
        outw(0x8045, OMAP1510_MCBSP1_BASE + 0x80c);  // RCR2
        outw(0x0040, OMAP1510_MCBSP1_BASE + 0x812);  // XCR1
        outw(0x8041, OMAP1510_MCBSP1_BASE + 0x810);  // XCR2
        outw(0x0f0b, OMAP1510_MCBSP1_BASE + 0x816);  // SRGR1
        outw(0x101f, OMAP1510_MCBSP1_BASE + 0x814);  // SRGR2
        outw(0x0001, OMAP1510_MCBSP1_BASE + 0x80a);  // SPCR1 enable
        outw(0x03f1, OMAP1510_MCBSP1_BASE + 0x808);  // SPCR2 enable

#endif

        /* Wait for the UDA1341 to wake up */
        mdelay(1);

        /* external clock configuration */
        omap1510_set_samplerate(audio_samplerate);

	/* Initialize the UDA1341 internal state */
	l3_open(&uda1341);

}

static void omap1510_audio_shutdown(void *dummy)
{
        u8 fpga;
printk(__FUNCTION__ "called\n");

#ifndef CONFIG_DSP_MCBSP1

        // Disable the McBSP channel
        outw(0x0000, OMAP1510_MCBSP1_BASE + 0x806);  // flush data
        outw(0x0000, OMAP1510_MCBSP1_BASE + 0x806);  // flush data
        outw(0x0000, OMAP1510_MCBSP1_BASE + 0x80a);  // disable SPCR1
        outw(0x0000, OMAP1510_MCBSP1_BASE + 0x808);  // disable SPCR2     

#endif

	/* disable the audio clocks/signals */
	l3_close(&uda1341);

        // Turn off clocks, but don't turn off audio amplifier
        // or you will hear a nasty noise when you turn it back
        // on.
        fpga = 0;
        fpga_write(fpga, OMAP1510P1_FPGA_AUDIO);
}

static int omap1510_audio_ioctl(struct inode *inode, struct file *file,
                                uint cmd, ulong arg)
{
	long val;
	int ret = 0;

printk(__FUNCTION__ "called\n");
	/*
	 * These are platform dependent ioctls which are not handled by the
	 * generic omap-audio module.
	 */
	switch (cmd) {
	case SNDCTL_DSP_STEREO:
		ret = get_user(val, (int *) arg);
		if (ret)
			return ret;
		/* the UDA1341 is stereo only */
		ret = (val == 0) ? -EINVAL : 1;
		return put_user(ret, (int *) arg);

	case SNDCTL_DSP_CHANNELS:
	case SOUND_PCM_READ_CHANNELS:
		/* the UDA1341 is stereo only */
		return put_user(2, (long *) arg);

	case SNDCTL_DSP_SPEED:
		ret = get_user(val, (long *) arg);
		if (ret) break;
		omap1510_set_samplerate(val);
		/* fall through */

	case SOUND_PCM_READ_RATE:
		return put_user(audio_samplerate, (long *) arg);

	case SNDCTL_DSP_SETFMT:
	case SNDCTL_DSP_GETFMTS:
		/* we can do 16-bit only */
		return put_user(AFMT_S16_LE, (long *) arg);

	default:
		/* Maybe this is meant for the mixer (As per OSS Docs) */
		return mixer_ioctl(inode, file, cmd, arg);
	}

	return ret;
}

static int omap1510_audio_open(struct inode *inode, struct file *file)
{
printk(__FUNCTION__ "called\n");
	return omap_audio_attach(inode, file, &audio_state);
}

/*
 * Missing fields of this structure will be patched with the call
 * to omap_audio_attach().
 */
static struct file_operations omap1510_audio_fops = {
	open:		omap1510_audio_open,
	owner:		THIS_MODULE
};


static int audio_dev_id, mixer_dev_id;

static int __init omap1510_uda1341_init(void)
{
	int ret;

printk(__FUNCTION__ "called\n");
	ret = l3_attach_client(&uda1341, "l3-bit-omap1510-gpio", "uda1341");
	if (ret)
		goto out;

	/* register devices */
	audio_dev_id = register_sound_dsp(&omap1510_audio_fops, -1);
	mixer_dev_id = register_sound_mixer(&omap1510_mixer_fops, -1);

	printk( KERN_INFO "OMAP1510 audio support initialized\n" );
	return 0;

out:
	return ret;
}

static void __exit omap1510_uda1341_exit(void)
{
printk(__FUNCTION__ "called\n");
	unregister_sound_dsp(audio_dev_id);
	unregister_sound_mixer(mixer_dev_id);
	l3_detach_client(&uda1341);
}

module_init(omap1510_uda1341_init);
module_exit(omap1510_uda1341_exit);

MODULE_AUTHOR("Nicolas Pitre, George France, Steve Johnson");
MODULE_DESCRIPTION("Glue audio driver for the TI OMAP1510 & Philips UDA1341 codec.");

EXPORT_NO_SYMBOLS;
