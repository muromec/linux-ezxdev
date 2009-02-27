/*
 * Glue audio driver for the TI OMAP1510 & TI AIC23 CODEC
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
 * 2002-05-23   Steve Johnson   Modified for OMAP1510.
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
#include <linux/i2c.h>

#include <asm/semaphore.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/arch/ck.h>

#include "omap-audio.h"
#include <asm/arch/mcbsp.h>
#include "aic23.h"

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DPRINTK( x... )  printk( ##x )
#else
#define DPRINTK( x... )
#endif

#define AUDIO_NAME		"OMAP1510_AIC23"

#define AUDIO_RATE_DEFAULT	44100

/* Define the macro MCBSP_I2S_MASTER to make the McBSP the I2S
 * master and the CODEC the I2S slave.  If this macro is not defined 
 * then the CODEC is the I2S master and the McBSP is the I2S slave.
 */
#define MCBSP_I2S_MASTER

/*
  Local prototypes.
*/
static void omap1510_audio_init(void *dummy);
static void omap1510_audio_shutdown(void *dummy);
static int omap1510_audio_ioctl(struct inode *inode, struct file *file,
                                uint cmd, ulong arg);

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CEE */
#include <linux/device.h>

static int omap1510_audio_suspend(struct device * dev, u32 state, u32 level);
static int omap1510_audio_resume(struct device * dev, u32 level);

static struct device_driver audio_driver_ldm = {
       name:      "omap1510-aic23",
       devclass:  NULL,
       probe:     NULL,
       suspend:   omap1510_audio_suspend,
       resume:    omap1510_audio_resume,
       remove:    NULL,
};

static struct device audio_device_ldm = {
       name: "OMAP1510 I2S Audio Codec",
       bus_id: "I2S_Audio",
       driver: NULL,
       power_state: DPM_POWER_ON,
};

static void audio_ldm_driver_register(void)
{
   extern void dsp_public_driver_register(struct device_driver *driver);

   dsp_public_driver_register(&audio_driver_ldm);
}

static void audio_ldm_device_register(void)
{
   extern void dsp_public_device_register(struct device *device);

   dsp_public_device_register(&audio_device_ldm);
}

static void audio_ldm_driver_unregister(void)
{
   extern void dsp_public_driver_unregister(struct device_driver *driver);

   dsp_public_driver_unregister(&audio_driver_ldm);
}

static void audio_ldm_device_unregister(void)
{
   extern void dsp_public_device_unregister(struct device *device);

   dsp_public_device_unregister(&audio_device_ldm);
}
#endif /* MVL-CEE */

/* Addresses to scan */
// static unsigned short normal_i2c[] = { DEV_AIC23_ID };
static unsigned short normal_i2c[] = { I2C_CLIENT_END };
// static unsigned short normal_i2c_range[] = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short normal_i2c_range[] = { 0x1b, I2C_CLIENT_END };
static unsigned short probe[2]        = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short probe_range[2]  = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short ignore[2]       = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short ignore_range[2] = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short force[2]        = { I2C_CLIENT_END, I2C_CLIENT_END };
static struct i2c_client_address_data addr_data = {
	normal_i2c, normal_i2c_range, 
	probe, probe_range, 
	ignore, ignore_range, 
	force
};

static int this_adap;
static struct i2c_driver driver;
static struct i2c_client aic23 = {
        "(unset)",		
        -1,
        0,
        0,
        NULL,
        &driver    
};

static int input_or_output;

static audio_stream_t output_stream = {
		id:		"AIC23 out",
		dma_dev:	eAudioTx,
};

static audio_stream_t input_stream = {
		id:		"AIC23 in",
		dma_dev:	eAudioRx,
};

static audio_state_t audio_state = {
	output_stream:	&output_stream,
	input_stream:	&input_stream,
//	need_tx_for_rx:	1,  TODO Try this once receive is working.
	need_tx_for_rx:	0,
	hw_init:	omap1510_audio_init,
	hw_shutdown:	omap1510_audio_shutdown,
	client_ioctl:	omap1510_audio_ioctl,
	sem:		__MUTEX_INITIALIZER(audio_state.sem),
};

#define REC_MASK (SOUND_MASK_LINE | SOUND_MASK_MIC)
#define DEV_MASK (REC_MASK | SOUND_MASK_VOLUME)

#define SET_VOLUME 1
#define SET_LINE   2

#define DEFAULT_VOLUME          93
#define DEFAULT_INPUT_VOLUME     0	/* 0 ==> mute line in */
					/* use input vol of 75 for 0dB gain */

#define OUTPUT_VOLUME_MIN 0x30
#define OUTPUT_VOLUME_MAX LHV_MAX
#define OUTPUT_VOLUME_RANGE (OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)
#define OUTPUT_VOLUME_MASK OUTPUT_VOLUME_MAX

#define INPUT_VOLUME_MIN 0x0
#define INPUT_VOLUME_MAX LIV_MAX
#define INPUT_VOLUME_RANGE (INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)
#define INPUT_VOLUME_MASK INPUT_VOLUME_MAX

struct {
        u8 volume;
        u16 volume_reg;
        u8 line;
        u8 mic;
        u16 input_volume_reg;
        int mod_cnt;
} aic23_local;

/*
  Simplified write routine.  I know the i2c message
  is just two bytes and won't be more.
*/
static void omap1510_write_i2c(u8 address, u16 data) 
{
        u8 buf[2];
        int ret = 0;
        int retries = 5;

	DPRINTK(__FUNCTION__ ": addr 0x%02x, data 0x%04x\n", address, data);
        buf[0] = ((address << 1) | ((data & 0x100) >> 8));
        buf[1] = (data & 0xff);
        while ((ret != sizeof(buf)) && (retries--)) {
                if (sizeof(buf) != (ret = i2c_master_send(&aic23, (const char *)buf, sizeof(buf)))) {
                        DPRINTK(__FUNCTION__ ": expected %d, received %d, address 0x%x\n", sizeof(buf), ret, address);
                        DPRINTK("retries - %d\n", retries);
                }
        }

}

static void aic23_update(int flag, int val)
{
        u8 volume;

        switch (flag) {
        case SET_VOLUME:
                aic23_local.volume_reg &= ~OUTPUT_VOLUME_MASK;
                // Convert 0 -> 100 volume to 0x30 -> 0x7f volume range
                volume = ((val * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MIN;
                aic23_local.volume_reg |= volume;
                omap1510_write_i2c(LEFT_CHANNEL_VOLUME_ADDR, aic23_local.volume_reg);
                omap1510_write_i2c(RIGHT_CHANNEL_VOLUME_ADDR, aic23_local.volume_reg);
                break;

        case SET_LINE:
                // [4:0] 12dB to -34.5dB in 1.5 steps 10111=0
                aic23_local.input_volume_reg &= ~INPUT_VOLUME_MASK;
                // Convert 0 -> 100 volume to 0x0 -> 0x1f volume range
                volume = ((val * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;
                aic23_local.input_volume_reg |= volume;
                omap1510_write_i2c(LEFT_LINE_VOLUME_ADDR, aic23_local.input_volume_reg);
                omap1510_write_i2c(RIGHT_LINE_VOLUME_ADDR, aic23_local.input_volume_reg);
                break;
        }

}

static int mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
        int val;
        int gain;
        int ret = 0;
        int nr = _IOC_NR(cmd);

	/*
	 * We only accept mixer (type 'M') ioctls.
	 */
	if (_IOC_TYPE(cmd) != 'M')
		return -EINVAL;

	DPRINTK(__FUNCTION__ " 0x%08x\n", cmd);

	if (cmd == SOUND_MIXER_INFO) {
		struct mixer_info mi;
                
		strncpy(mi.id, "AIC23", sizeof(mi.id));
		strncpy(mi.name, "TI AIC23", sizeof(mi.name));
		mi.modify_counter = aic23_local.mod_cnt;
		return copy_to_user((void *)arg, &mi, sizeof(mi));
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = get_user(val, (int *)arg);
		if (ret)
			goto out;

                // Ignore separate left/right channel for now,
                // even the codec does support it.
		gain = val & 255;

		switch (nr) {
		case SOUND_MIXER_VOLUME:
			aic23_local.volume = val;
			aic23_local.mod_cnt++;
			aic23_update(SET_VOLUME, gain);
			break;

		case SOUND_MIXER_LINE:
			aic23_local.line = val;
			aic23_local.mod_cnt++;
			aic23_update(SET_LINE, gain);
			break;

		case SOUND_MIXER_MIC:
			aic23_local.mic = val;
			aic23_local.mod_cnt++;
			aic23_update(SET_LINE, gain);
			break;

		case SOUND_MIXER_RECSRC:
			break;

		default:
			ret = -EINVAL;
		}
	}

	if (ret == 0 && _IOC_DIR(cmd) & _IOC_READ) {
		ret = 0;

		switch (nr) {
		case SOUND_MIXER_VOLUME:     val = aic23_local.volume;	break;
		case SOUND_MIXER_LINE:       val = aic23_local.line;	break;
		case SOUND_MIXER_MIC:        val = aic23_local.mic;	break;
		case SOUND_MIXER_RECSRC:     val = REC_MASK;	break;
		case SOUND_MIXER_RECMASK:    val = REC_MASK;	break;
		case SOUND_MIXER_DEVMASK:    val = DEV_MASK;	break;
		case SOUND_MIXER_CAPS:       val = 0;		break;
		case SOUND_MIXER_STEREODEVS: val = 0;		break;
		default:	val = 0;     ret = -EINVAL;	break;
		}

		if (ret == 0)
			ret = put_user(val, (int *)arg);
	}
 out:
	return ret;

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
        int clock_divider = 0;
        int sampling_rate = 0;
        int bosr = 0;
        u16 tmp;

	/* We don't want to mess with clocks when frames are in flight */
        // TODO - could call omap1510_dma_flush_all, or could poll on
        // enable bit to wait for DMA writes to stop.

	/* wait for any frame to complete */
	udelay(125);

	DPRINTK(__FUNCTION__ " %d\n", val);

/* #define CODEC_CLOCK 12000000 */
#define CODEC_CLOCK 16934400
#if (CODEC_CLOCK == 12000000)

#ifdef CONFIG_OMAP_INNOVATOR
	/* set the CODEC clock input source to 12.000MHz */
	fpga_write(fpga_read(OMAP1510P1_FPGA_POWER) & ~0x01, 
			OMAP1510P1_FPGA_POWER);
#endif

        /*
         * We have the following clock sources:
         * 12.000 MHz
         *
         *  Available sampling rates:
         *  96kHz, 88.2KHz, 48kHz, 44.1kHz, 32kHz, 8kHz,
         *  and half of those 24kHz, 16kHz, (4kHz)
         *  Won't bother supporting those in ().
         */
        if (val >= 96000)
                val = 96000;
        else if (val >= 88200)
                val = 88200;
        else if (val >= 48000)
                val = 48000;
        else if (val >= 44100)
                val = 44100;
        else if (val >= 32000)
                val = 32000;
        else if (val >= 24000)
                val = 24000;
        else if (val >= 16000)
                val = 16000;
        else 
                val = 8000;

        /* Set the clock divider */
        switch (val) {
        case 24000:
        case 16000:
                clock_divider = CLKIN_HALF;
                break;
        default:
                break;
        }

        /* Set the sampling rate bits */
        bosr=0;
        switch (val) {
        case 96000:
                sampling_rate = 0x07;
                break;
        case 88200:
                bosr = 1;
                sampling_rate = 0x0f;
                break;
        case 48000:
        case 24000:
                sampling_rate = 0;
                break;
        case 44100:
                bosr = 1;
                sampling_rate = 0x08;
                break;
        case 32000:
        case 16000:
                sampling_rate = 6;
                break;
        case 8000:
                sampling_rate = 3;
                break;
        }

        // Set sample rate at CODEC
        tmp = (clock_divider | (sampling_rate << SR_SHIFT) | (bosr << BOSR_SHIFT) | USB_CLK_ON);
        omap1510_write_i2c(SAMPLE_RATE_CONTROL_ADDR, tmp);
#else	/* if (CODEC_CLOCK == 12000000) */

#ifdef CONFIG_OMAP_INNOVATOR
	/* set the CODEC clock input source to 16.9344MHz */
	fpga_write(fpga_read(OMAP1510P1_FPGA_POWER) | 0x01, 
			OMAP1510P1_FPGA_POWER);
#endif
        /*
         * We have the following clock sources:
         * 16.9344 MHz
         *
         *  Available sampling rates:
         *  88.2KHz, 44.1kHz, 22.05kHz
         */
	if (val >= 88200)
                val = 88200;
        else if (val >= 44100)
                val = 44100;
	else if (val >= 22050)
		val = 22050;
	else 
		val =8018;

        /* Set the sampling rate bits */
        bosr=1;
        switch (val) {
        case 88200:
                sampling_rate = 0x0f;
                break;
        case 44100:
                sampling_rate = 0x08;
                break;
        case 22050:
                clock_divider = CLKIN_HALF;
                sampling_rate = 0x08;
                break;
	case 8018:
		sampling_rate = 0x0b;
		break;
        }

        // Set sample rate at CODEC
        tmp = (clock_divider | (sampling_rate << SR_SHIFT) | (bosr << BOSR_SHIFT));
        omap1510_write_i2c(SAMPLE_RATE_CONTROL_ADDR, tmp);
#endif	/* if (CODEC_CLOCK == 12000000) */

#ifdef MCBSP_I2S_MASTER
        /* 
           Set Sample Rate at McBSP

           Formula : 
           Codec System Clock = CODEC_CLOCK, or half if clock_divider = 1;
           clkgdv = ((Codec System Clock / (SampleRate * BitsPerSample * 2)) - 1);

           FWID = BitsPerSample - 1;
           FPER = (BitsPerSample * 2) - 1;
        */  
        {
                int clkgdv;

                if (clock_divider == CLKIN_HALF) {
                        clkgdv = ((CODEC_CLOCK / 2) / (val * 16 * 2)) - 1;
                } 
                else {
                        clkgdv = (CODEC_CLOCK / (val * 16 * 2)) - 1;
                }

                outw((FWID(15) | CLKGDV(clkgdv)), AUDIO_SRGR1);   
//              outw((FWID(15) | CLKGDV(7)), AUDIO_SRGR1);      // 48K
//              outw((FWID(15) | CLKGDV(0x2f)), AUDIO_SRGR1);   // 8K
                outw((FSGM | FPER(31)), AUDIO_SRGR2);
        }
#endif

	audio_samplerate = val;
}

static void omap1510_audio_init(void *dummy)
{
        dma_regs_t *dma_regs;
        u16 tmp;

        if (input_or_output == FMODE_WRITE) {
                dma_regs = output_stream.dma_regs;
        }
        else if (input_or_output == FMODE_READ) {
                dma_regs = input_stream.dma_regs;
        }
        else {
                return;
        }

        // Reset codec
        omap1510_write_i2c(RESET_CONTROL_ADDR, 0);

        /*
          Now here's an ugly hack.  To use McBSP1, you need to enable
          a clock on the DSP.  So enable the MPUI, set the clock,
          and start the DSP.  

          An even uglier, evil hack.  If this is going to take the DSP
          out of reset, shove an idle loop at the reset vector
          and make it loop instead of crash.  You will still see
          a DSP watchdog timer go off.

          I would prefer having a DSP program (probably the MP3 decoder) set
          up the DSP, but this allows an ARM-only MP3 decoder to work.  With
          this code, the DSP is never put into idle, so the OMAP chip cannot 
          go into any decent low-power mode.  Also, all of the DSP interface
          dependencies (such as MPUI_STROBE settings) would be taken care
          of.  Plus, we wouldn't have to worry about different boot vector
          addresses depending on the chip version.
        */

        {
                u32 boot_vector;
                u8 c55_start[] = { 0x7A, 0x00, 0x00, 0x0C, 0x4A, 0x7A, 0x20, 0x20, 0x20, 0x20 };

                tmp = inw(0xfffece10);	/* read ARM_RSTCT1 register */
                if (0 == (tmp & 0x6)) {      // check if DSP is up
			DPRINTK(__FUNCTION__ ": Bringing DSP out of reset.\n");
                        if (0 == (tmp & 0x4)) {   // MPUI in reset
                                tmp |= 0x4;
                                outw(tmp, 0xfffece10);
                                ck_enable(api_ck);
                        }
                        tmp = inw(0xe1008008);	/* read DSP_IDLECT2 register */
                        if (0 == (tmp & 0x2)) {  // DSP CLKM enable
                                tmp |= 0x2;
                                outw(tmp, 0xe1008008);
                        }
                        tmp = inw(0xe1008014);	/* read DSP_RSTCT2 register */
                        if (0 == (tmp & 0x1)) {  // DSP PER_EN bit
                                tmp |= 0x1;
                                outw(tmp, 0xe1008014);
                        }
                        tmp = inw(0xfffece00);	/* read ARM_CKCTL register */
                        tmp |= 0x2000;
                        outw(tmp, 0xfffece00);
                        // Write C55 code at reset vector.
                        if (cpu_type() == OMAP1509) {
                                boot_vector = 0x4c000;
                        }
                        else {
                                boot_vector = 0x10000;
                        }
                        memcpy((void *)(OMAP_DSP_BASE + boot_vector), 
                               &c55_start, 
                               sizeof(c55_start));
                        outw(0x5, 0xfffec918);   // Set DSP boot mode
                        tmp = inw(0xfffece10);   // take DSP out of reset
                        tmp |= 0x6;
                        outw(tmp, 0xfffece10);
                }     
                else {        // DSP's up, just check the clock/per bits
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
            Configure the DMA channel and MCBSP.
        */

        if (input_or_output == FMODE_WRITE) {
                // Setup DMA channel to McBSP1 audio Tx.
                dma_regs->csdp = 0x0a01;
                dma_regs->ccr = 0x1000 | audio_state.output_stream->dma_dev;            // source auto increment, don't enable yet
                dma_regs->cicr = 0x23;
                dma_regs->cdsa_l = ((OMAP1510_MCBSP1_BASE + 0x806) & 0xffff);     //McBSP1 DXR1
                dma_regs->cdsa_u = ((OMAP1510_MCBSP1_BASE + 0x806) >> 16);
                dma_regs->cfn = 0x1;
                omap_dma_setup(audio_state.output_stream->dma_dev, eDmaOut);
        }
        else {
                // Setup DMA channel to McBSP1 audio Rx.
                dma_regs->csdp = 0x0015;                                   
                dma_regs->ccr = 0x4000 | audio_state.input_stream->dma_dev;            // destn auto increment, don't enable yet
                dma_regs->cicr = 0x23;
                dma_regs->cssa_l = ((OMAP1510_MCBSP1_BASE + 0x802) & 0xffff);     //McBSP1 DRR1
                dma_regs->cssa_u = ((OMAP1510_MCBSP1_BASE + 0x802) >> 16);
                dma_regs->cfn = 0x1;
                omap_dma_setup(audio_state.input_stream->dma_dev, eDmaIn);
        }
        // Initialize McBSP channel
        // See TI application report, 
        //  SPRA595, "TMS320C6000 McBSP: I2S Interface" 
        //  for info on why these registers have these values.
        // Default to 16 bits/sample stereo.
        outw(0x0000, AUDIO_SPCR1);  // disable
        outw(0x0000, AUDIO_SPCR2);  // disable
#ifdef MCBSP_I2S_MASTER
	/* configure MCBSP to be the I2S master */
        outw((FSXM | FSRM | CLKXM | CLKRM | CLKXP | CLKRP), AUDIO_PCR0);

         // 16 bits per Receive/Transmit word, 2 word per frame, 16-bit stereo.
        outw((XFRLEN1(0) | XWDLEN1(2)), AUDIO_XCR1);
        outw((XPHASE | XFRLEN2(0) | XWDLEN2(2) | XDATDLY(1) | XFIG), AUDIO_XCR2);
        outw(RFRLEN1(0) | RWDLEN1(2), AUDIO_RCR1);   
        outw((RPHASE | RFRLEN2(0) | RWDLEN2(2) | RDATDLY(1) | RFIG), AUDIO_RCR2);
        outw((RINTM(3) | RRST), AUDIO_SPCR1);                // enable input
        outw((FRST | GRST | XRST | XINTM(3)), AUDIO_SPCR2);  // enable output
#else	/* ifdef MCBSP_I2S_MASTER */
	/* configure MCBSP to be the I2S slave */
        outw(CLKXP | CLKRP, AUDIO_PCR0);

         // 16 bits per Receive/Transmit word, 2 word per frame, 16-bit stereo.
        outw((XFRLEN1(0) | XWDLEN1(2)), AUDIO_XCR1);
        outw((XPHASE | XFRLEN2(0) | XWDLEN2(2) | XDATDLY(1) | XFIG), AUDIO_XCR2);
        outw(RFRLEN1(0) | RWDLEN1(2), AUDIO_RCR1);   
        outw((RPHASE | RFRLEN2(0) | RWDLEN2(2) | RDATDLY(1) | RFIG), AUDIO_RCR2);
        outw((RINTM(3) | RRST), AUDIO_SPCR1);	// enable input
        outw((XINTM(3) | XRST), AUDIO_SPCR2);	// enable output
#endif	/* ifdef MCBSP_I2S_MASTER */

	DPRINTK(__FUNCTION__ ", line %d: McBSP enabled\n", __LINE__);
	DPRINTK("McBSP1:  DRR2 =0x%04x, DRR1 =0x%04x, DXR2 =0x%04x, DXR1 =0x%04x\n",
		inw(AUDIO_DRR2), inw(AUDIO_DRR1), inw(AUDIO_DXR2), inw(AUDIO_DXR1));
	DPRINTK("McBSP1:  SPCR2=0x%04x, SPCR1=0x%04x, RCR2 =0x%04x, RCR1 = 0x%04x\n",
		inw(AUDIO_SPCR2), inw(AUDIO_SPCR1), inw(AUDIO_RCR2), inw(AUDIO_RCR1));
	DPRINTK("McBSP1:  XCR2 =0x%04x, XCR1 =0x%04x, SRGR2=0x%04x, SRGR1=0x%04x\n",
		inw(AUDIO_XCR2), inw(AUDIO_XCR1), inw(AUDIO_SRGR2), inw(AUDIO_SRGR1));
	DPRINTK("McBSP1:  MCR2 =0x%04x, MCR1 =0x%04x, RCERA=0x%04x, RCERB=0x%04x\n",
		inw(AUDIO_MCR2), inw(AUDIO_MCR1), inw(AUDIO_RCERA), inw(AUDIO_RCERB));
	DPRINTK("McBSP1:  XCERA=0x%04x, XCERB=0x%04x, PCR0 =0x%04x\n",
		inw(AUDIO_XCERA), inw(AUDIO_XCERB), inw(AUDIO_PCR0));

	/* Initialize the AIC23 internal state */
        /*  
            The AIC23 uses 9 bits for register control.  The
            extra bit gets placed in the LSB of the subregister
            address, and the address is shifted by one.
        */
            
	// the volume control is only for earphone not line out
	// line input volume can be controlled but not in following code
	// which pick the default value 0dB

	// Left line input volume control
	tmp = 0;	/* set LRS_ENABLED to simultaneously adjust left/right */
//        tmp |= LIV_DEFAULT;	// [4:0] 12dB to -34.5dB in 1.5 steps 10111=0
//        tmp |= LIV_MAX;	        // [4:0] 12dB to -34.5dB in 1.5 steps 10111=0
//        tmp |= LIM_MUTED;
        aic23_local.line = DEFAULT_INPUT_VOLUME;
        aic23_local.mic = DEFAULT_INPUT_VOLUME;
        aic23_local.input_volume_reg = tmp;
        aic23_update(SET_LINE, DEFAULT_INPUT_VOLUME);
//        omap1510_write_i2c(LEFT_LINE_VOLUME_ADDR, tmp);
//        omap1510_write_i2c(RIGHT_LINE_VOLUME_ADDR, tmp);

	// Left/Right headphone channel volume control
	// Zero-cross detect on
	tmp = LZC_ON;	/* set LRS_ENABLED to simultaneously adjust left/right */ 
//	tmp |= LHV_DEFAULT;	// Volume dB, base volume
//	tmp |= LHV_MAX; 	// Volume dB, base volume
        aic23_local.volume = DEFAULT_VOLUME;
        aic23_local.volume_reg = tmp;
        aic23_update(SET_VOLUME, DEFAULT_VOLUME);
//        omap1510_write_i2c(LEFT_CHANNEL_VOLUME_ADDR, tmp);
//        omap1510_write_i2c(RIGHT_CHANNEL_VOLUME_ADDR, tmp);

	// Analog audio path control
	// DAC selected
	tmp = DAC_SELECTED; 
//	tmp |= STE_ENABLED;
	tmp |= INSEL_MIC; // comment out for line input
//	tmp |= MICB_20DB;
//	tmp |= MICM_MUTED; 
        omap1510_write_i2c(ANALOG_AUDIO_CONTROL_ADDR, tmp);

	// Digital audio path control
	// De-emphasis control 44.1kHz
	tmp = DEEMP_44K;
        omap1510_write_i2c(DIGITAL_AUDIO_CONTROL_ADDR, tmp);

	// Power control, everything is on. TI code sets to 0x100, which is invalid in AIC23 spec?
	tmp = 0x0;
        omap1510_write_i2c(POWER_DOWN_CONTROL_ADDR, tmp);

	// Digital audio interface, master/slave mode, I2S, 16 bit
	tmp = FOR_I2S;
#ifndef MCBSP_I2S_MASTER
	tmp |= MS_MASTER;
#endif
        omap1510_write_i2c(DIGITAL_AUDIO_FORMAT_ADDR, tmp);

	// Digital interface
	tmp = ACT_ON;
        omap1510_write_i2c(DIGITAL_INTERFACE_ACT_ADDR, tmp);

        /* clock configuration */
        omap1510_set_samplerate(audio_samplerate);

}

static void omap1510_audio_shutdown(void *dummy)
{
        /* 
           Turn off codec after it is done.  
           Can't do it immediately, since it may still have
           buffered data.

           Wait 20ms (arbitrary value) and then turn it off.
        */
        
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(2);

        // Disable the McBSP channel
        outw(0x0000, AUDIO_DXR1);  // flush data
        outw(0x0000, AUDIO_DXR1);  // flush data
        outw(0x0000, AUDIO_SPCR1);  // disable SPCR1
        outw(0x0000, AUDIO_SPCR2);  // disable SPCR2     

        omap1510_write_i2c(RESET_CONTROL_ADDR, 0);
        omap1510_write_i2c(POWER_DOWN_CONTROL_ADDR, 0xff);
        
}

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CEE */

static int omap1510_audio_suspend(struct device *dev, u32 state, u32 level)
{
  extern void audio_ldm_suspend(void *data);

  switch(level)
  {
     case SUSPEND_POWER_DOWN: 

       /* Turn off power to omap1510_audio */
        
       audio_ldm_suspend(&audio_state);     
       break;
  }

  return 0;

}

static int omap1510_audio_resume(struct device *dev, u32 level)
{
  extern void audio_ldm_resume(void *data);

  switch(level)
  {
     case RESUME_POWER_ON:

       /* Turn on power to omap1510_audio */

       audio_ldm_resume(&audio_state);
       break;
  }

  return 0;
}

#endif


static int omap1510_audio_ioctl(struct inode *inode, struct file *file,
                                uint cmd, ulong arg)
{
	long val;
	int ret = 0;

	DPRINTK(__FUNCTION__ " 0x%08x\n", cmd);

	/*
	 * These are platform dependent ioctls which are not handled by the
	 * generic omap-audio module.
	 */
	switch (cmd) {
	case SNDCTL_DSP_STEREO:
		ret = get_user(val, (int *) arg);
		if (ret)
			return ret;
		/* the AIC23 is stereo only */
		ret = (val == 0) ? -EINVAL : 1;
		return put_user(ret, (int *) arg);

	case SNDCTL_DSP_CHANNELS:
	case SOUND_PCM_READ_CHANNELS:
		/* the AIC23 is stereo only */
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
        // Set a flag so we know how to initialize hardware.
        if (file->f_mode & FMODE_WRITE) {
                input_or_output = FMODE_WRITE;
        }
        else if (file->f_mode & FMODE_READ) {
                input_or_output = FMODE_READ;
        }
        else {
                input_or_output = 0;
        }
                
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


static int aic23_attach(struct i2c_adapter *adap, int addr,
			unsigned short flags, int kind)
{

        // This makes sure this is only attached once.
	if (this_adap > 0)
		return -1;
	this_adap++;
	
        aic23.adapter = adap;
        aic23.addr = addr;

        printk(KERN_INFO "%s: chip found @ 0x%x\n", __FUNCTION__, addr);

        i2c_attach_client(&aic23);

	return 0;
}

static int aic23_probe(struct i2c_adapter *adap)
{
	this_adap = 0;
//	if (adap->id == I2C_ALGO_EXP)
		return i2c_probe(adap, &addr_data, aic23_attach);
	return 0;
}

static int aic23_detach(struct i2c_client *client)
{
	i2c_detach_client(client);
	return 0;
}

/* ----------------------------------------------------------------------- */

static struct i2c_driver driver = {
        "i2c aic23 driver",
        I2C_DRIVERID_EXP0,           // Fake ID
        I2C_DF_NOTIFY,
        aic23_probe,
        aic23_detach,
};


static int audio_dev_id, mixer_dev_id;

static int __init omap1510_aic23_init(void)
{
	int ret;
        u8 fpga;
        
        /*
          Turn on chip select for CODEC (shared with touchscreen).  
          Don't turn it back off, in case touch screen needs it.
        */                           
        fpga = fpga_read(OMAP1510P1_FPGA_TOUCHSCREEN);
        fpga |= 0x4;
        fpga_write(fpga, OMAP1510P1_FPGA_TOUCHSCREEN);
        
        aic23_local.volume = DEFAULT_VOLUME;

	ret = i2c_add_driver(&driver);
	if ((ret) || (this_adap == 0)) {          // didn't attach
                printk(KERN_INFO
		       "OMAP1510 audio support failed to find AIC23.\n");
		goto out;
        }

	/* register devices */
	audio_dev_id = register_sound_dsp(&omap1510_audio_fops, -1);
	mixer_dev_id = register_sound_mixer(&omap1510_mixer_fops, -1);

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CCE */
        audio_ldm_device_register();
        audio_ldm_driver_register();
#endif /* MVL-CCE */

	printk(KERN_INFO "OMAP1510 audio support initialized\n");
	return 0;

out:
	return ret;
}

static void __exit omap1510_aic23_exit(void)
{

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CCE */
        audio_ldm_device_unregister();
        audio_ldm_driver_unregister();
#endif /* MVL-CCE */

	unregister_sound_dsp(audio_dev_id);
	unregister_sound_mixer(mixer_dev_id);
        i2c_del_driver(&driver);
}

module_init(omap1510_aic23_init);
module_exit(omap1510_aic23_exit);

MODULE_AUTHOR("Nicolas Pitre, George France, Steve Johnson");
MODULE_DESCRIPTION("Glue audio driver for the TI OMAP1510 & AIC23 codec.");
MODULE_LICENSE("GPL");

EXPORT_NO_SYMBOLS;
