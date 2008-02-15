/*
 * drivers/sound/omap1610-tsc2101.c *
 * Glue audio driver for the TI OMAP1610 & TI TSC2101 CODEC
 *          (based on omap1510-aic23.c)
 *
 * Copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */




#include <linux/module.h>
/* #include <linux/kernel.h> */
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>

#include <asm/semaphore.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/arch/ck.h>

#include "omap-audio.h"
#include <asm/arch/mcbsp.h>
#include <asm/hardware/tsc2101.h>

#undef DEBUG
/* #define DEBUG */
#ifdef DEBUG
#define DPRINTK( x... )  printk( ##x )
#define FN_IN printk(__FUNCTION__ " start\n")
#define FN_OUT(n) printk(__FUNCTION__ " end(%u)\n",n)
#else
#define DPRINTK( x... )
#define FN_IN
#define FN_OUT(n)
#endif

#define AUDIO_NAME		"OMAP1610_TSC2101"
#define AUDIO_RATE_DEFAULT	44100


/* Define the macro MCBSP_I2S_MASTER to make the McBSP the I2S
 * master and the CODEC the I2S slave.  If this macro is not defined 
 * then the CODEC is the I2S master and the McBSP is the I2S slave.
 */

/*
 *  Local prototypes.
 */
static void omap1610_audio_init(void *dummy);
static void omap1610_audio_shutdown(void *dummy);
static int omap1610_audio_ioctl(struct inode *inode, struct file *file,
                                uint cmd, ulong arg);

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CEE */
#include <linux/device.h>

static int omap1610_audio_suspend(struct device * dev, u32 state, u32 level);
static int omap1610_audio_resume(struct device * dev, u32 level);

static struct device_driver audio_driver_ldm = {
       name:      "omap1610-tsc2101",
       devclass:  NULL,
       probe:     NULL,
       suspend:   omap1610_audio_suspend,
       resume:    omap1610_audio_resume,
       remove:    NULL,
};

static struct device audio_device_ldm = {
       name: "OMAP1610/TSC2101 Audio Codec",
       bus_id: "I2S_Audio",
       driver: NULL,
       power_state: DPM_POWER_ON,
};

static void audio_ldm_driver_register(void)
{
   extern void dsp_public_driver_register(struct device_driver *driver);
   FN_IN;
   dsp_public_driver_register(&audio_driver_ldm);
   FN_OUT(0);
}

static void audio_ldm_device_register(void)
{
   extern void dsp_public_device_register(struct device *device);
   FN_IN;
   dsp_public_device_register(&audio_device_ldm);
   FN_OUT(0);
}

static void audio_ldm_driver_unregister(void)
{
   extern void dsp_public_driver_unregister(struct device_driver *driver);
   FN_IN;
   dsp_public_driver_unregister(&audio_driver_ldm);
   FN_OUT(0);
}

static void audio_ldm_device_unregister(void)
{
   extern void dsp_public_device_unregister(struct device *device);
   FN_IN;
   dsp_public_device_unregister(&audio_device_ldm);
   FN_OUT(0);
}
#endif /* MVL-CEE */


static int openMode=0;


static audio_stream_t output_stream = {
	.id = 		"TSC2101 out",
	.dma_dev =	eAudioTx,
};

static audio_stream_t input_stream = {
	.id = 		"TSC2101 in",
	.dma_dev =	eAudioRx,
};

static audio_state_t audio_state = {
	.output_stream =        &output_stream,
	.input_stream =         &input_stream,
	.need_tx_for_rx =       0,
	.hw_init =              omap1610_audio_init,
	.hw_shutdown =          omap1610_audio_shutdown,
	.client_ioctl =         omap1610_audio_ioctl,
	.sem = 	                __MUTEX_INITIALIZER(audio_state.sem),
};

#define PAGE2_AUDIO_CODEC_REGISTERS  2 << 11
#define LEAVE_CS 0x80

#define REC_MASK (SOUND_MASK_LINE | SOUND_MASK_MIC)
#define DEV_MASK (REC_MASK | SOUND_MASK_VOLUME)

#define SET_VOLUME 1
#define SET_LINE   2

#define DEFAULT_VOLUME          100



#define DEFAULT_INPUT_VOLUME     0	/* 0 ==> mute line in */
					/* use input vol of 75 for 0dB gain */

#define OUTPUT_VOLUME_MIN 0x7F
#define OUTPUT_VOLUME_MAX 0x00
#define OUTPUT_VOLUME_RANGE (OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)
#define OUTPUT_VOLUME_MASK OUTPUT_VOLUME_MIN

#define INPUT_VOLUME_MIN 0x0
#define INPUT_VOLUME_MAX LIV_MAX
#define INPUT_VOLUME_RANGE (INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)
#define INPUT_VOLUME_MASK INPUT_VOLUME_MAX

#define MCBSP_I2S_MASTER

struct {
        u8 volume;
        u16 volume_reg;
        u8 line;
        u8 mic;
        u16 input_volume_reg;
        int mod_cnt;
} tsc2101_local;

/*
  Simplified write to uWire routine.
*/

extern u16 omap1610_uwire_data_transfer(u8 cs, u16 data, u8 trans_size, u8 rec_size);

static void omap1610_tsc2101_write(u8 address, u16 data) 
{
	DPRINTK(__FUNCTION__ ": addr 0x%02x, data 0x%04x\n", address, data);

        /* CS = 1, 16 bit transfer, write-only */

	omap1610_uwire_data_transfer(LEAVE_CS | 1, (PAGE2_AUDIO_CODEC_REGISTERS | (address << 5)), 16, 0);
        omap1610_uwire_data_transfer(1, data, 16, 0);
	FN_OUT(0);
}

static u16 omap1610_tsc2101_read(u8 address) 
{
	DPRINTK(__FUNCTION__ ": addr 0x%02x\n", address);

        /* CS = 1, 16 bit transfer, write-only */

	omap1610_uwire_data_transfer(LEAVE_CS | 1, (0x8000 | PAGE2_AUDIO_CODEC_REGISTERS | (address << 5)), 16, 0);
	return omap1610_uwire_data_transfer(1, (PAGE2_AUDIO_CODEC_REGISTERS | (address << 5)), 0, 16);
}

static void tsc2101_update(int flag, int val)
{
        u16 volume;

	FN_IN;
        switch (flag) {
        case SET_VOLUME:
                /* Convert 0 -> 100 volume to 0x30 -> 0x7f volume range */
                volume = ((val * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MIN;
		omap1610_tsc2101_write(TSC2101_DAC_GAIN_CTRL,(volume<<8)|volume);

                break;

        case SET_LINE:
                break;
        }
	FN_OUT(0);
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
	FN_IN;
	if (_IOC_TYPE(cmd) != 'M')
		return -EINVAL;

	DPRINTK(__FUNCTION__ " 0x%08x\n", cmd);

	if (cmd == SOUND_MIXER_INFO) {
		struct mixer_info mi;
                
		strncpy(mi.id, "TSC2101", sizeof(mi.id));
		strncpy(mi.name, "TI TSC2101", sizeof(mi.name));
		mi.modify_counter = tsc2101_local.mod_cnt;
		FN_OUT(1);
		return copy_to_user((void *)arg, &mi, sizeof(mi));
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = get_user(val, (int *)arg);
		if (ret)
			goto out;

                /* Ignore separate left/right channel for now,
		   even the codec does support it. */
		gain = val & 255;

		switch (nr) {
		case SOUND_MIXER_VOLUME:
			tsc2101_local.volume = val;
			tsc2101_local.mod_cnt++;
			tsc2101_update(SET_VOLUME, gain);
			break;

		case SOUND_MIXER_LINE:
			tsc2101_local.line = val;
			tsc2101_local.mod_cnt++;
			tsc2101_update(SET_LINE, gain);
			break;

		case SOUND_MIXER_MIC:
			tsc2101_local.mic = val;
			tsc2101_local.mod_cnt++;
			tsc2101_update(SET_LINE, gain);
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
		case SOUND_MIXER_VOLUME:     val = tsc2101_local.volume;	break;
		case SOUND_MIXER_LINE:       val = tsc2101_local.line;	break;
		case SOUND_MIXER_MIC:        val = tsc2101_local.mic;	break;
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
	FN_OUT(0);
	return ret;

}

static struct file_operations omap1610_mixer_fops = {
	ioctl:		mixer_ioctl,
	owner:		THIS_MODULE
};


/*
 * Audio interface
 */

static long audio_samplerate = AUDIO_RATE_DEFAULT;


static void omap1610_set_samplerate(long val)
{
        int divisor = 0;
        int fs_44kHz = 0;

	/* We don't want to mess with clocks when frames are in flight */
        /* TODO - could call omap1510_dma_flush_all, or could poll on
	   enable bit to wait for DMA writes to stop. */

	/* wait for any frame to complete */
	udelay(125);

	DPRINTK(__FUNCTION__ " %d\n", val);

        /*
         * We have the following clock sources:
         * 12.000 MHz
         *
         *  Available sampling rates:
         *  48kHz, 44.1kHz, 32kHz, 29.4kHz, 24kHz,
         *  22 kHz, 16 kHz, 14.7kHz, 12kHz, 11kHz,
	 *  9.6kHz, 8.8kHz, 8kHz, (7.3kHz)
         *  Won't bother supporting those in ().
         */
        if (val >= 48000)
	  {
                val = 48000;
		fs_44kHz=0;
		divisor=0; /* division to 1 */
	  }
        else if (val >= 44100)
	  {
                val = 44100;
		fs_44kHz=1;
		divisor=0; /* division to 1 */
	  }
        else if (val >= 32000)
	  {
                val = 32000;
		fs_44kHz=0;
		divisor=1; /* division to 1.5 */
	  }
        else if (val >= 29400)
	  {
                val = 29400;
		fs_44kHz=1;
		divisor=1; /* division to 1.5 */
	  }
        else if (val >= 24000)
	  {
                val = 24000;
		fs_44kHz=0;
		divisor=2; /* division to 2 */
	  }
        else if (val >= 22050)
	  {
                val = 22050;
		fs_44kHz=1;
		divisor=2; /* division to 2 */
	  }
        else if (val >= 16000)
	  {
                val = 16000;
		fs_44kHz=0;
		divisor=3; /* division to 3 */
	  }
        else if (val >= 14700)
	  {
                val = 14700;
		fs_44kHz=1;
		divisor=3; /* division to 3 */
	  }
        else if (val >= 12000)
	  {
                val = 12000;
		fs_44kHz=0;
		divisor=4; /* division to 4 */
	  }
        else if (val >= 11025)
	  {
                val = 11025;
		fs_44kHz=1;
		divisor=4; /* division to 4 */
	  }
        else if (val >= 9600)
	  {
                val = 9600;
		fs_44kHz=0;
		divisor=5; /* division to 5 */
	  }
        else if (val >= 8820)
	  {
                val = 8820;
		fs_44kHz=1;
		divisor=5; /* division to 5 */
	  }
        else if( val >= 8000)
	  {
                val = 8000;
		fs_44kHz=0;
		divisor=7; /* division to 6 */
	  }
        else 
	  {
                val = 7350;
		fs_44kHz=1;
		divisor=7; /* division to 6 */
	  }

	omap1610_tsc2101_write(TSC2101_AUDIO_CTRL_3, (fs_44kHz<<13)|(1<<11));
	omap1610_tsc2101_write(TSC2101_AUDIO_CTRL_1, (divisor<<3)|divisor);
	
	if(fs_44kHz) {
	    /* pll enable, P, J */
	    omap1610_tsc2101_write(TSC2101_PLL_PROG_1, (1<<15)|(1<<8)|(7<<2));
	    /* D (NB: in decimal!) */ 
	    omap1610_tsc2101_write(TSC2101_PLL_PROG_2, 5264<<2);
	} else {
	    omap1610_tsc2101_write(TSC2101_PLL_PROG_1, (1<<15)|(1<<8)|(8<<2));
	    omap1610_tsc2101_write(TSC2101_PLL_PROG_2, 1920<<2);
	}

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
                int clkgdv=0;

                outw((FWID(15) | CLKGDV(clkgdv)), AUDIO_SRGR1);   
                outw((GSYNC | CLKSP | FSGM | FPER(31)), AUDIO_SRGR2);
        }
#endif

	audio_samplerate = val;
	FN_OUT(0);
}


static void omap1610_audio_init(void *dummy)
{

        u16 tmp;

	FN_IN;



        /* check that open mode is correct */
	if(openMode & FMODE_WRITE || openMode & FMODE_READ){
	}else{
	  printk("Incorrect open mode: %s:%d  %s\n",__FILE__,__LINE__,__FUNCTION__);
	  FN_OUT(1);
	  return;
	}

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
                if (0 == (tmp & 0x6)) {      /*  check if DSP is up */
			DPRINTK(__FUNCTION__ ": Bringing DSP out of reset.\n");
                        if (0 == (tmp & 0x4)) {   /*  MPUI in reset */
                                tmp |= 0x4;
                                outw(tmp, 0xfffece10);
                                ck_enable(api_ck);
                        }
                        tmp = inw(0xe1008008);	/* read DSP_IDLECT2 register */
                        if (6 != (tmp & 0x6)) {  /*  DSP CLKM enable */
                                tmp |= 0x6;
                                outw(tmp, 0xe1008008);
                        }
                        tmp = inw(0xe1008014);	/* read DSP_RSTCT2 register */
                        if (3 != (tmp & 0x3)) {  /*  DSP PER_EN bit */
                                tmp |= 0x3;
                                outw(tmp, 0xe1008014);
                        }
                        tmp = inw(0xfffece00);	/* read ARM_CKCTL register */
                        tmp |= 0x2000;
                        outw(tmp, 0xfffece00);
                        /*  Write C55 code at reset vector. */
			DPRINTK(__FUNCTION__ ": Bringing DSP out of reset - in progress.\n");
			boot_vector = 0x10000;
                        memcpy((void *)(OMAP_DSP_BASE + boot_vector), 
                               &c55_start, 
                               sizeof(c55_start));
                        outw(0x5, 0xfffec918);   /*  Set DSP boot mode */
                        tmp = inw(0xfffece10);   /*  take DSP out of reset */
                        tmp |= 0x6;
                        outw(tmp, 0xfffece10);
                }     
                else {        /*  DSP's up, just check the clock/per bits */
                        tmp = inw(0xe1008008);
                        if (0 == (tmp & 0x2)) {  /*  DSP CLKM enable */
                                tmp |= 0x2;
                                outw(tmp, 0xe1008008);
                        }
                        tmp = inw(0xe1008014);
                        if (0 == (tmp & 0x1)) {  /*  DSP PER_EN bit */
                                tmp |= 0x1;
                                outw(tmp, 0xe1008014);
                        }
                }
        }          

        /*  
            Configure the DMA channel and MCBSP.
        */
	DPRINTK(__FUNCTION__ ": Configure DMA output channel. output_stream.dma_regs: %p\n", output_stream.dma_regs);
	DPRINTK(__FUNCTION__ ": Configure DMA input channel. input_stream.dma_regs: %p\n", input_stream.dma_regs);



            if(openMode & FMODE_WRITE){
                /*  Setup DMA channel to McBSP1 audio Tx. */
                output_stream.dma_regs->csdp = 0x0a01;
                output_stream.dma_regs->ccr |= 0x1400;/* !!!!! source auto increment, don't enable yet */
                output_stream.dma_regs->cicr = 0x23;
                output_stream.dma_regs->cdsa_l = ((OMAP1610_MCBSP1_BASE + 0x806) & 0xffff);     /* McBSP1 DXR1 */
                output_stream.dma_regs->cdsa_u = ((OMAP1610_MCBSP1_BASE + 0x806) >> 16);
                output_stream.dma_regs->cfn = 0x1;
                omap_dma_setup(audio_state.output_stream->dma_dev, eDmaOut);
	    }/* if(openMode & FMODE_WRITE) */





	    if(openMode & FMODE_READ){
                /*  Setup DMA channel to McBSP1 audio Rx. */
                input_stream.dma_regs->csdp = 0x0015;                                   
                input_stream.dma_regs->ccr |= 0x4400;            /*  destn auto increment, don't enable yet */
                input_stream.dma_regs->cicr = 0x23;
                input_stream.dma_regs->cssa_l = ((OMAP1610_MCBSP1_BASE + 0x802) & 0xffff);     /* McBSP1 DRR1 */
                input_stream.dma_regs->cssa_u = ((OMAP1610_MCBSP1_BASE + 0x802) >> 16);
                input_stream.dma_regs->cfn = 0x1;
                omap_dma_setup(audio_state.input_stream->dma_dev, eDmaIn);
	    }/* if(openMode & FMODE_READ) */

        /* Initialize McBSP channel
	 * See TI application report, 
	 *  SPRA595, "TMS320C6000 McBSP: I2S Interface" 
	 * for info on why these registers have these values.
	 * Default to 16 bits/sample stereo.  !!!? does it correspond to AUDIO_CTRL_1 state?
	 */
	DPRINTK(__FUNCTION__ ": Configure McBSP channel\n");
        outw(0x0000, AUDIO_SPCR1);  /*  disable */
        outw(0x0000, AUDIO_SPCR2);  /*  disable */
#ifdef MCBSP_I2S_MASTER
        outw(( CLKXM | CLKRM | FSXP | FSRP | CLKXP | CLKRP), AUDIO_PCR0);

	/*  16 bits per Receive/Transmit word, 2 word per frame, 16-bit stereo. */
        outw((XFRLEN1(0) | XWDLEN1(2)), AUDIO_XCR1);

	/*COMPAND: MSB - First OK ( XCOMPAND(1) - LSB first)
	OMAP1610TRM.pdf: 16.13.5  16-160 XCOMPAND(2 or 3) - register aLaw/uLaw
	*/
        outw(( XPHASE | XFRLEN2(0) | XWDLEN2(2) | XDATDLY(1) | XFIG), AUDIO_XCR2);

        outw((RFRLEN1(0) | RWDLEN1(2)), AUDIO_RCR1);   
        outw((RPHASE | RFRLEN2(0) | RWDLEN2(2) | RDATDLY(1) /*| RFIG*/), AUDIO_RCR2);
        outw((RINTM(3) | RRST), AUDIO_SPCR1);                /*  enable input */
        outw((FREE | FRST | GRST | XRST | XINTM(3)), AUDIO_SPCR2);  /*  enable output */

	outw((FWID(15)), AUDIO_SRGR1);   
	outw((GSYNC | CLKSP | FSGM | FPER(31)), AUDIO_SRGR2);
#else	/* ifdef MCBSP_I2S_MASTER */
	/* configure MCBSP to be the I2S slave */
        outw(CLKXP | CLKRP, AUDIO_PCR0);

	/*  16 bits per Receive/Transmit word, 2 word per frame, 16-bit stereo. */
        outw((XFRLEN1(0) | XWDLEN1(2)), AUDIO_XCR1);
        outw((XPHASE | XFRLEN2(0) | XWDLEN2(2) | XDATDLY(1) | XFIG), AUDIO_XCR2);
        outw(RFRLEN1(0) | RWDLEN1(2), AUDIO_RCR1);   
        outw((RPHASE | RFRLEN2(0) | RWDLEN2(2) | RDATDLY(1) | RFIG), AUDIO_RCR2);
        outw((RINTM(3) | RRST), AUDIO_SPCR1);	/*  enable input */
        outw((XINTM(3) | XRST), AUDIO_SPCR2);	/*  enable output */
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

	omap1610_tsc2101_write(TSC2101_CODEC_POWER_CTRL,0);

        /* Mute Headset In
	   Set Headset In Volume 0dB
	   AGC for Headset In off */
	omap1610_tsc2101_write(TSC2101_HEADSET_GAIN_CTRL, 0x7700);

        /* unMute Handset In
	   Handset AGC off */
	omap1610_tsc2101_write(TSC2101_HANDSET_GAIN_CTRL, 0x7700);

        /* Cell In AGC off */
	omap1610_tsc2101_write(TSC2101_AGC_CTRL, 0x0000);

        /* Mute Cellphone In, Buzzer In */
	omap1610_tsc2101_write(TSC2101_BUZZER_GAIN_CTRL, 0xC57C);
  
        /* Mute Analog Sidetone
	  Select MIC_INHND input for handset mic
	  Cell Phone In not connected */
	omap1610_tsc2101_write(TSC2101_MIXER_PGA_CTRL, 0x8030);

        /* Keyclick disabled
	   Keyclick medium amplitude, 1kHz, 4 periods keyclick */
	omap1610_tsc2101_write(TSC2101_AUDIO_CTRL_2, 0x4410);

        /* ADC, DAC, Analog Sidetone, cellphone, buzzer softstepping enabled
	   1dB AGC hysteresis
	   MICes bias 2V */
	omap1610_tsc2101_write(TSC2101_AUDIO_CTRL_4, 0x4540);

        /* Set codec output volume, seam to be an artefact
	 *TODO: check if this may be removed from here
	 */	
	omap1610_tsc2101_write(TSC2101_DAC_GAIN_CTRL, 0x0000);	


        /* DAC left routed to SPK2, right to SPK1
	   SPK1/2 unmuted */
	omap1610_tsc2101_write(TSC2101_AUDIO_CTRL_5, 0x4080);

        /* SPK2/SPK1 routed to OUT8P/N 
	   OUT8P/N unmuted, Cap interface for Headset */
	omap1610_tsc2101_write(TSC2101_AUDIO_CTRL_6, 0xC048);

        /* Headset/Hook switch detect disabled */
	omap1610_tsc2101_write(TSC2101_AUDIO_CTRL_7, 0x0000);

	/* Initialize the AIC23 internal state */
        /*  
            The AIC23 uses 9 bits for register control.  The
            extra bit gets placed in the LSB of the subregister
            address, and the address is shifted by one.
        */

	/* The volume control is only for earphone not line out
	   line input volume can be controlled but not in following code
	   which pick the default value 0dB */
        tsc2101_local.line = DEFAULT_INPUT_VOLUME;
        tsc2101_local.mic = DEFAULT_INPUT_VOLUME;
        tsc2101_update(SET_LINE, DEFAULT_INPUT_VOLUME);

	/* Left/Right headphone channel volume control
	   Zero-cross detect on */
        tsc2101_update(SET_VOLUME, tsc2101_local.volume);

        /* clock configuration */
        omap1610_set_samplerate(audio_samplerate);
	FN_OUT(0);
}

static void omap1610_audio_shutdown(void *dummy)
{
        /* 
           Turn off codec after it is done.  
           Can't do it immediately, since it may still have
           buffered data.

           Wait 20ms (arbitrary value) and then turn it off.
        */
        
	FN_IN;
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(2);

        /*  Disable the McBSP channel */
        outw(0x0000, AUDIO_DXR1);  /* flush data */
        outw(0x0000, AUDIO_DXR1);  /* flush data */
        outw(0x0000, AUDIO_SPCR1);  /* disable SPCR1 */
        outw(0x0000, AUDIO_SPCR2);  /* disable SPCR2 */

	omap1610_tsc2101_write(TSC2101_CODEC_POWER_CTRL,0xE7FD);
	FN_OUT(0);
}

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CEE */

static int omap1610_audio_suspend(struct device *dev, u32 state, u32 level)
{
  extern void audio_ldm_suspend(void *data);

  FN_IN;
  switch(level)
  {
     case SUSPEND_POWER_DOWN: 

       /* Turn off power to omap1610_audio */
        
       audio_ldm_suspend(&audio_state);
       /* Power off codec. */
       omap1610_tsc2101_write(TSC2101_CODEC_POWER_CTRL, 0xE7FD);
       break;
  }

  FN_OUT(0);
  return 0;
}

static int omap1610_audio_resume(struct device *dev, u32 level)
{
  extern void audio_ldm_resume(void *data);

  FN_IN;
  switch(level)
  {
     case RESUME_POWER_ON:
	     /* Turn on power to omap1610_audio - power on codec */
	     omap1610_tsc2101_write(TSC2101_CODEC_POWER_CTRL,0x0);
	     audio_ldm_resume(&audio_state);
	     break;
  }

  FN_OUT(0);
  return 0;
}
#endif

static int omap1610_audio_ioctl(struct inode *inode, struct file *file,
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
		/* the TSC2101 is stereo only */
		ret = (val == 0) ? -EINVAL : 1;
		FN_OUT(1);
		return put_user(ret, (int *) arg);

	case SNDCTL_DSP_CHANNELS:
	case SOUND_PCM_READ_CHANNELS:
		/* the AIC23 is stereo only */
		FN_OUT(2);
		return put_user(2, (long *) arg);

	case SNDCTL_DSP_SPEED:
		ret = get_user(val, (long *) arg);
		if (ret) break;
		omap1610_set_samplerate(val);
		/* fall through */

	case SOUND_PCM_READ_RATE:
		FN_OUT(3);
		return put_user(audio_samplerate, (long *) arg);

	case SNDCTL_DSP_SETFMT:
	case SNDCTL_DSP_GETFMTS:
		/* we can do 16-bit only */
		FN_OUT(4);
		return put_user(AFMT_S16_LE, (long *) arg);

	default:
		/* Maybe this is meant for the mixer (As per OSS Docs) */
		FN_OUT(5);
		return mixer_ioctl(inode, file, cmd, arg);
	}

	FN_OUT(0);
	return ret;
}

static int omap1610_audio_open(struct inode *inode, struct file *file)
{
        /* Set a flag so we know how to initialize hardware. */
	FN_IN;

	openMode=file->f_mode;

/*
        if (file->f_mode & FMODE_WRITE) {
                input_or_output = FMODE_WRITE;
        }
        else if (file->f_mode & FMODE_READ) {
                input_or_output = FMODE_READ;
        }
        else {
                input_or_output = 0;
        }
*/
	FN_OUT(0);
	return omap_audio_attach(inode, file, &audio_state);
}

/*
 * Missing fields of this structure will be patched with the call
 * to omap_audio_attach().
 */
static struct file_operations omap1610_audio_fops = {
	open:		omap1610_audio_open,
	owner:		THIS_MODULE
};


static int audio_dev_id, mixer_dev_id;
extern void omap1610_uwire_cs1_configure_mode(u8 edge_rd, u8 edge_wr, u8 lvl, u8 frq, u8 chk);

static int __init omap1610_tsc2101_init(void)
{
	FN_IN;
	outl(inl(PU_PD_SEL_2)|(1<<22), PU_PD_SEL_2); /* Configure MCLK enable */

	outl(inl(FUNC_MUX_CTRL_7)|(2<<18), FUNC_MUX_CTRL_7); /* Configure N15 pin to be uWire CS1 */
	omap1610_uwire_cs1_configure_mode(1,1,0,2,0);

        tsc2101_local.volume = DEFAULT_VOLUME;

	/* register devices */
	audio_dev_id = register_sound_dsp(&omap1610_audio_fops, -1);
	mixer_dev_id = register_sound_mixer(&omap1610_mixer_fops, -1);

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CCE */
        audio_ldm_device_register();
        audio_ldm_driver_register();
#endif /* MVL-CCE */

	printk(KERN_INFO "OMAP1610 audio support initialized\n");
	FN_OUT(0);
	return 0;
}

static void __exit omap1610_tsc2101_exit(void)
{

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CCE */
        audio_ldm_device_unregister();
        audio_ldm_driver_unregister();
#endif /* MVL-CCE */

	unregister_sound_dsp(audio_dev_id);
	unregister_sound_mixer(mixer_dev_id);
}

module_init(omap1610_tsc2101_init);
module_exit(omap1610_tsc2101_exit);

MODULE_AUTHOR("MontaVista");
MODULE_DESCRIPTION("Glue audio driver for the TI OMAP1610 & TSC2101 codec.");
MODULE_LICENSE("GPL");

EXPORT_NO_SYMBOLS;
