/*
 *
 * Glue audio driver for the TI OMAP730 & TI TWL3016 (Syren) CODEC
 *          (based on omap1610-tsc2101.c)
 * Jean Pihet <j-pihet@ti.com>
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
#include <asm/hardware.h>
#include <asm/arch/ck.h>

#include "omap730-audio.h"
#include <linux/csmi.h>

#undef CONFIG_CEE
#define CONFIG_CEE

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DPRINTK( x... )  printk(KERN_WARNING x)
//#define DPRINTK( x... )  printk( ##x )
#define FN_IN printk("%s start\n", __FUNCTION__)
#define FN_OUT(n) printk("%s end(%d)\n", __FUNCTION__, n)
#else
#define DPRINTK( x... )
#define FN_IN
#define FN_OUT(n)
#endif

#define AUDIO_NAME		"OMAP730_SYREN"

#define AUDIO_RATE_DEFAULT	44100

#define REC_MASK ( SOUND_MASK_LINE | SOUND_MASK_MIC )
#define DEV_MASK ( REC_MASK | SOUND_MASK_PCM )

#define SET_LINE   1
#define SET_PCM    2

#define DEFAULT_VOLUME       	81	// 0 dB gain
#define DEFAULT_INPUT_VOLUME 	0	// mute inputs

#define OUTPUT_VOLUME_MIN 	0x00
#define OUTPUT_VOLUME_MAX 	0x7F
#define OUTPUT_VOLUME_RANGE 	(OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)
#define OUTPUT_VOLUME_MASK 	OUTPUT_VOLUME_MIN

#define INPUT_VOLUME_MIN 	0x00
#define INPUT_VOLUME_MAX 	0x7F
#define INPUT_VOLUME_RANGE 	(INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)
#define INPUT_VOLUME_MASK 	INPUT_VOLUME_MAX

#define	DMA_DEFAULT_VOLUME	0xE7	// 0dB gain
#define	MIXER_DEFAULT_VOLUME	0x67	// 0dB gain


/*
  Local prototypes.
*/
static void omap730_audio_init(void *dummy);
static void omap730_audio_shutdown(void *dummy);
static int omap730_audio_ioctl(struct inode *inode, struct file *file,
                                uint cmd, ulong arg);
static int omap730_getclock(void);
static void omap730_setclock(int on_off);


#ifdef CONFIG_CEE  /* MVL-CEE */
#include <linux/device.h>

static int omap730_audio_suspend(struct device * dev, u32 state, u32 level);
static int omap730_audio_resume(struct device * dev, u32 level);

static struct device_driver audio_driver_ldm = {
       name:      "omap730-syren",
       devclass:  NULL,
       probe:     NULL,
       suspend:   omap730_audio_suspend,
       resume:    omap730_audio_resume,
       remove:    NULL,
};

static struct device audio_device_ldm = {
       name: "OMAP730/TWL3016 (SYREN) Audio Codec",
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

static int input_or_output;

static audio_stream_t output_stream = {
		name:		"SYREN out",
};

static audio_stream_t input_stream = {
		name:		"SYREN in",
};

static audio_state_t audio_state = {
	output_stream:	&output_stream,
	input_stream:	&input_stream,
	hw_init:	omap730_audio_init,
	hw_shutdown:	omap730_audio_shutdown,
	client_ioctl:	omap730_audio_ioctl,
	sem:		__MUTEX_INITIALIZER(audio_state.sem),
};

struct {
        u8 volume;
        u8 line;
        u8 mic;
        int mod_cnt;
} eac_local;


static void eac_dump(void)
{
#ifdef DEBUG

	#define DBPRINT(name, addr)     printk("%s: %s(@0x%08x) = 0x%08x\n", __FUNCTION__, name, (__u32) addr, *((__u16 *) addr));

	DBPRINT("DPLL1_CTL_REG", DPLL1_CTL_REG);
	DBPRINT("ARM_SYSST", ARM_SYSST);
	DBPRINT("ARM_CKCTL", ARM_CKCTL);
	DBPRINT("PCC_CTRL_REG", PCC_CTRL_REG);
	DBPRINT("ARM_RSTCT2", ARM_RSTCT2);
	DBPRINT("ARM_IDLECT1", ARM_IDLECT1);
	DBPRINT("ARM_IDLECT2", ARM_IDLECT2);
	DBPRINT("ARM_IDLECT3", ARM_IDLECT3);
	DBPRINT("OMAP_DMA_GCR_REG", OMAP_DMA_GCR_REG);
	DBPRINT("PERSEUS2_MODE_1", PERSEUS2_MODE_1);
	DBPRINT("ULPD_SOFT_DISABLE_REQ_REG", SOFT_DISABLE_REQ_REG);
	DBPRINT("ULPD_CAM_CLK_CTRL", CAM_CLK_CTRL);
	DBPRINT("EAC_AGCTR", EAC_AGCTR);
	DBPRINT("EAC_AGCFR", EAC_AGCFR);
	DBPRINT("EAC_AGCFR2", EAC_AGCFR2);
	DBPRINT("EAC_CPTCTL", EAC_CPTCTL);
	DBPRINT("EAC_CPCFR1", EAC_CPCFR1);
	DBPRINT("EAC_CPCFR2", EAC_CPCFR2);
	DBPRINT("EAC_CPCFR3", EAC_CPCFR3);
	DBPRINT("EAC_CPCFR4", EAC_CPCFR4);
	DBPRINT("EAC_AMVCTR", EAC_AMVCTR);
	DBPRINT("EAC_AM1VCTR", EAC_AM1VCTR);
	DBPRINT("EAC_AM2VCTR", EAC_AM2VCTR);
	DBPRINT("EAC_AM3VCTR", EAC_AM3VCTR);
	DBPRINT("EAC_AMSCFR", EAC_AMSCFR);
	DBPRINT("EAC_MPCTR", EAC_MPCTR);
	DBPRINT("EAC_MPMCCFR", EAC_MPMCCFR);
	DBPRINT("EAC_BPCTR", EAC_BPCTR);
	DBPRINT("EAC_BPMCCFR", EAC_BPMCCFR);
	DBPRINT("SOFT_REQ_REG", SOFT_REQ_REG);
	DBPRINT("PCC_PERIPH_CLOCK_SOURCE_SEL", PCC_PERIPH_CLOCK_SOURCE_SEL);
#endif
}


static void omap730_eac_write(__u16* addr, __u16 data) 
{
	DPRINTK("%s: addr 0x%08x, data 0x%04x\n", __FUNCTION__, (int) addr, data);
	*((__u16 *) addr) = data;
}

static u16 omap730_eac_read(__u16* addr) 
{
	__u16 data;
	
	data = *((__u16 *) addr);
	DPRINTK("%s: addr 0x%08x, data 0x%04x\n", __FUNCTION__, (int) addr, data);

	return data;
}

static void eac_update(void)
{
        u16 volume, line;
	u16 mixer1_vol, mixer2_vol, mixer3_vol;
	u16 reg1_vol, reg2_vol, reg3_vol;
	int clock_enabled;

	FN_IN;

	// Enable the EAC clock if needed
	if (!(clock_enabled = omap730_getclock()))
		omap730_setclock(1);

	// PCM -> Mixer 2B & 3B
	// LINE & MIC -> Mixer 1A & 3A

       	// Convert % to Gain
  	volume = ((eac_local.volume * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MIN;
	line = ((eac_local.line * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;

	// Calc the register value (Mixer1 B & Mixer2 A to 0)
	mixer1_vol = (line << MIXER_x_A_GAIN_OFFSET) & MIXER_x_A_MASK;
	mixer2_vol = (volume << MIXER_x_B_GAIN_OFFSET) & MIXER_x_B_MASK;
	mixer3_vol = (line << MIXER_x_A_GAIN_OFFSET) & MIXER_x_A_MASK;
	mixer3_vol |= (volume << MIXER_x_B_GAIN_OFFSET) & MIXER_x_B_MASK;

	// Use some read/write/read algotithm to prevent strange EAC mixer registers access problem
	reg1_vol = omap730_eac_read((u16 *) EAC_AM1VCTR);
	reg2_vol = omap730_eac_read((u16 *) EAC_AM2VCTR);
	reg3_vol = omap730_eac_read((u16 *) EAC_AM3VCTR);
	// Apply volume to Mixer1
       	omap730_eac_write((u16 *) EAC_AM1VCTR, mixer1_vol);
	// Apply volume to Mixer2
	omap730_eac_write((u16 *) EAC_AM2VCTR, mixer2_vol);
	// Apply volume to Mixer3
       	omap730_eac_write((u16 *) EAC_AM3VCTR, mixer3_vol);
	mdelay(1);
	// Check the registers values
	reg1_vol = omap730_eac_read((u16 *) EAC_AM1VCTR);
	reg2_vol = omap730_eac_read((u16 *) EAC_AM2VCTR);
	reg3_vol = omap730_eac_read((u16 *) EAC_AM3VCTR);

	// Feedback on the actual mixer settings
	if (reg1_vol != mixer1_vol || reg2_vol != mixer2_vol || reg3_vol != mixer3_vol)
	{
		eac_local.volume = (100 * ((reg2_vol & MIXER_x_B_MASK) >> MIXER_x_B_GAIN_OFFSET) - OUTPUT_VOLUME_MIN) / OUTPUT_VOLUME_RANGE;
		eac_local.line = eac_local.mic = (100 * ((reg1_vol & MIXER_x_A_MASK) >> MIXER_x_A_GAIN_OFFSET) - INPUT_VOLUME_MIN) / INPUT_VOLUME_RANGE;
	}

	// Disable the EAC clock if it was disabled when entering the function
	if (!clock_enabled)
		omap730_setclock(0);

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

	DPRINTK("%s: IOC_NR=0x%08x, IOC_DIR=0x%08x\n", __FUNCTION__, _IOC_NR(cmd), _IOC_DIR(cmd));

	if (cmd == SOUND_MIXER_INFO) {
		struct mixer_info mi;
                
		strncpy(mi.id, "SYREN", sizeof(mi.id));
		strncpy(mi.name, "TI SYREN", sizeof(mi.name));
		mi.modify_counter = eac_local.mod_cnt;
		FN_OUT(1);
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
		case SOUND_MIXER_PCM:
			eac_local.volume = gain;
			eac_local.mod_cnt++;
			eac_update();
			break;

		case SOUND_MIXER_MIC:
		case SOUND_MIXER_LINE:
			eac_local.line = eac_local.mic = gain;
			eac_local.mod_cnt++;
			eac_update();
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
		case SOUND_MIXER_PCM:        val = eac_local.volume;	break;
		case SOUND_MIXER_LINE:       val = eac_local.line;	break;
		case SOUND_MIXER_MIC:        val = eac_local.mic;	break;
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

static struct file_operations omap730_mixer_fops = {
	ioctl:		mixer_ioctl,
	owner:		THIS_MODULE
};


/*
 * Audio interface
 */

static long audio_samplerate = AUDIO_RATE_DEFAULT;

static void omap730_set_samplerate(long val)
{
        int fsint = 0;


	FN_IN;
	
	/* We don't want to mess with clocks when frames are in flight */
        // TODO - could call dma_flush_all, or could poll on
        // enable bit to wait for DMA writes to stop.

	/* wait for any frame to complete */
	udelay(125);

	DPRINTK("%s %d\n", __FUNCTION__, (int) val);

	fsint = omap730_eac_read((u16 *) EAC_AGCFR) & ~EAC_AGCFR_FSINT_MASK;

        /*
         * We have the following clock sources:
         * 13.000 MHz
         *
         *  Available sampling rates:
         *  (48kHz,) 44.1kHz, 22 kHz, 11kHz, 8kHz
         */
        //if (val >= 48000)
	//  {
        //        val = 48000;
	//	fsint = 0x0004;
	//  }
        //else 
	if (val >= 44100)
	  {
                val = 44100;
		fsint |= EAC_AGCFR_FSINT_44KHZ;
	  }
        else if (val >= 22050)
	  {
                val = 22050;
		fsint |= EAC_AGCFR_FSINT_22KHZ;
	  }
        else if (val >= 11025)
	  {
                val = 11025;
		fsint |= EAC_AGCFR_FSINT_11KHZ;
	  }
        else
	  {
                val = 8000;
		fsint |= EAC_AGCFR_FSINT_8KHZ;
	  }

	omap730_eac_write((u16 *) EAC_AGCFR, fsint);
	audio_samplerate = val;
	FN_OUT(0);
}

static int omap730_getclock(void)
{
	return (omap730_eac_read((u16 *) EAC_AGCTR) & EAC_AGCTR_MCLK_EN);
}

/* Warning: init/deinit operation order is important to keep the codec in sync
	with the I2S frames.
   Init:   Enable Syren codec (GTI_SetupAudio(1))
           Enable EAC Codec Port & Clocks (omap730_setclock(1))
           Enable DMA
   Deinit: Disable DMA
           Disable EAC Codec Port & Clocks (omap730_setclock(0))
	   Disable Syren codec (GTI_SetupAudio(0))
*/
static void omap730_setclock(int on_off)
{
    u16 agctr_temp, soft_temp;
    u16 temp;
    
    agctr_temp = omap730_eac_read((u16 *) EAC_AGCTR) & ~EAC_AGCTR_RESERVED;
    soft_temp = omap730_eac_read((u16 *) SOFT_REQ_REG);

    if (on_off)
    {
	// Enable clock & disable low power
	agctr_temp |= EAC_AGCTR_MCLK_EN;
	agctr_temp &= ~EAC_AGCTR_EACPWD;
	soft_temp |= SOFT_REQ_REG_EAC12M_DPLL_REQ;
    	omap730_eac_write((u16 *) EAC_AGCTR, agctr_temp);
    	mdelay(1);
    	omap730_eac_write((u16 *) SOFT_REQ_REG, soft_temp);

	// Enable C-Port
	temp = omap730_eac_read((u16 *) EAC_CPTCTL);
	temp |= EAC_CPTCTL_CPEN;
	omap730_eac_write((u16 *) EAC_CPTCTL, temp);	
    }
    else 
    {
	// Disable C-Port
	temp = omap730_eac_read((u16 *) EAC_CPTCTL);
	temp &= ~EAC_CPTCTL_CPEN;
	omap730_eac_write((u16 *) EAC_CPTCTL, temp);	

	// Disable clock & enable low power
	agctr_temp &= ~EAC_AGCTR_MCLK_EN;
	agctr_temp |= EAC_AGCTR_EACPWD;
	soft_temp &= ~SOFT_REQ_REG_EAC12M_DPLL_REQ;
    	omap730_eac_write((u16 *) SOFT_REQ_REG, soft_temp);
    	omap730_eac_write((u16 *) EAC_AGCTR, agctr_temp);
    	mdelay(1);
    }

}

static void omap730_audio_init(void *dummy)
{
	FN_IN;

        /*  
            Configure the DMA channel and EAC
        */

	// DMA configuration already done
	// Setup the I2S codec through the ARM7.
	GTI_SetupAudio(1);
	// Enable EAC clocks
	omap730_setclock(1);

	// Setup K switches for Phone Call + play/record
	omap730_eac_write((u16 *) EAC_AMSCFR, EAC_AMSCFR_DEFAULT_SWITCHES);

        omap730_set_samplerate(audio_samplerate);

	eac_dump();

	FN_OUT(0);
}

static void omap730_audio_shutdown(void *dummy)
{
	u16 temp;

        /* 
           Turn off codec after it is done.  
           Can't do it immediately, since it may still have
           buffered data.

           Wait 20ms (arbitrary value) and then turn it off.
        */
        
	FN_IN;
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(2);

	// Audio Global Control Register 2
	temp = omap730_eac_read((u16 *) EAC_AGCTR);
	// DMA read and write operation disabled
	temp &= ~(EAC_AGCTR_DMAWEN | EAC_AGCTR_DMAREN);
	omap730_eac_write((u16 *) EAC_AGCTR, temp);
	
	// Shutdown the I2S codec through the ARM7.
	omap730_setclock(0);
	// Disable EAC clocks
	GTI_SetupAudio(0);

	FN_OUT(0);
}

#ifdef CONFIG_CEE  /* MVL-CEE */

static int omap730_audio_suspend(struct device *dev, u32 state, u32 level)
{
  extern void audio_ldm_suspend(void *data);

  FN_IN;
  switch(level)
  {
     case SUSPEND_POWER_DOWN: 

       /* Turn off power to omap730_audio */
        
       audio_ldm_suspend(&audio_state);      
       omap730_setclock(0);

       break;
  }

  FN_OUT(0);
  return 0;
}

static int omap730_audio_resume(struct device *dev, u32 level)
{
  extern void audio_ldm_resume(void *data);

  FN_IN;
  switch(level)
  {
     case RESUME_POWER_ON:

       /* Turn on power to omap730_audio */

       omap730_setclock(1);
       audio_ldm_resume(&audio_state);
       
       break;
  }

  FN_OUT(0);
  return 0;
}

#endif


static int omap730_audio_ioctl(struct inode *inode, struct file *file,
                                uint cmd, ulong arg)
{
	long val;
	int ret = 0;

	DPRINTK("%s 0x%08x\n", __FUNCTION__, cmd);

	/*
	 * These are platform dependent ioctls which are not handled by the
	 * generic omap-audio module.
	 */
	switch (cmd) {
	case SNDCTL_DSP_STEREO:
		ret = get_user(val, (int *) arg);
		if (ret)
			return ret;
		/* the SYREN is stereo only */
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
		omap730_set_samplerate(val);
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

static int omap730_audio_open(struct inode *inode, struct file *file)
{
        // Set a flag so we know how to initialize hardware.
	FN_IN;
        if (file->f_mode & FMODE_WRITE) {
                input_or_output = FMODE_WRITE;
        }
        else if (file->f_mode & FMODE_READ) {
                input_or_output = FMODE_READ;
        }
        else {
                input_or_output = 0;
        }
	FN_OUT(0);
	return omap_audio_attach(inode, file, &audio_state);
}

/*
 * Missing fields of this structure will be patched with the call
 * to omap_audio_attach().
 */
static struct file_operations omap730_audio_fops = {
	open:		omap730_audio_open,
	owner:		THIS_MODULE
};


static int audio_dev_id, mixer_dev_id;

static int __init omap730_syren_init(void)
{
	u16 temp;

	
	FN_IN;

	eac_dump();

	/*
	 * Pins multiplexing
	 */
	
	// D_SYREN_VOICE: SCLK, SDO, SDI, FSYNC
    	*((volatile __u32 *) PERSEUS2_IO_CONF2) &= 0xFFFFFF1F;

	// D_SYREN_SPI + PE: MCUDI, MCUDO, MCUEN
	*((volatile __u32 *) PERSEUS2_IO_CONF2) &= 0xFFFFFFF0;

	// D_CRESET: GPIO36 used for headset detection
	*((volatile __u32 *) PERSEUS2_IO_CONF3) &= 0xFFFFFF1F;  
	*((volatile __u32 *) PERSEUS2_IO_CONF3) |= 0x000000C0;

	// D_EAC pins: CSYNC, CSCLK, CDO
	*((volatile __u32 *) PERSEUS2_IO_CONF4) &= 0xFF1FFFFF;
	
	// D_SMC_PWR: TSPACT_9
    	*((volatile __u32 *) PERSEUS2_IO_CONF9) &= 0xF1FFFFFF;
    	*((volatile __u32 *) PERSEUS2_IO_CONF9) |= 0x04000000;

	// PE_GSM_NIRQ
    	*((volatile __u32 *) PERSEUS2_IO_CONF10) &= 0xFFFFFFFE;

	// MCBSP1_CLKS_SEL: Internal oscilator, from PCC (13/48 Mhz)		
	*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFEF;

	// INTERNAL_EAC_BT_AUSPI_SOURC: Pins BT AuSPI : SCLK, SDI, FSYNC
	*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFF3;
	
	// INTERNAL_GSM_VOICE_SOURCE: Internal EAC Modem AuSPI
	*((volatile __u32 *) PERSEUS2_MODE_1) &= 0xFFFFFFFC;
	*((volatile __u32 *) PERSEUS2_MODE_1) |= 0x00000001;

	/*
	 * UPLD Clocks
	 */

	*((volatile __u16 *) SOFT_REQ_REG) |= SOFT_REQ_REG_EAC12M_DPLL_REQ;
	*((volatile __u16 *) PCC_PERIPH_CLOCK_SOURCE_SEL) &= ~PCC_PERIPH_SOURCE_EAC_CLK_SOURCE;
	*((volatile __u16 *) CAM_CLK_CTRL) |= CAM_CLK_CTRL_SYSTEM_CLK_EN;
 
	/*
	 * GPIO pins setup to detect headset
	 */
	// TODO: GPIO36 for headset detection	
		

	/*
	 * EAC setup
	 */

	// Audio Global Control Register 2
	temp = omap730_eac_read((u16 *) EAC_AGCTR);
	// EAC in powerdown mode
	temp |= EAC_AGCTR_EACPWD;
	// Audio processing disabled	
	temp &= ~EAC_AGCTR_AUDEN;
	omap730_eac_write((u16 *) EAC_AGCTR, temp);
	
	// Audio Global Configuration Register
	temp = omap730_eac_read((u16 *) EAC_AGCFR) & EAC_AGCFR_RESERVED;
	// stereo, 16 bit audio file
	temp |= EAC_AGCFR_B8_16 | EAC_AGCFR_MN_ST;
	// clock setting
	temp |= EAC_AGCFR_AUD_CKSRC_12MHZ;
	omap730_eac_write((u16 *) EAC_AGCFR, temp);

	// EAC rev2 Intermediate sample frequency for DMA read and write operations	
	omap730_set_samplerate(AUDIO_RATE_DEFAULT);	

	// set clock on
	omap730_setclock(1);

	// Audio Mixer Switchs Configuration Register	
	omap730_eac_write((u16 *) EAC_AMSCFR, EAC_AMSCFR_DEFAULT_SWITCHES);

	// Set default volume
	// Default DMA volume
	omap730_eac_write((u16 *) EAC_AMVCTR, (DMA_DEFAULT_VOLUME << EAC_AMVCTR_RD_DMA_OFFSET) | (DMA_DEFAULT_VOLUME << EAC_AMVCTR_WR_DMA_OFFSET));
	// Line (GSM) & Mic input volume control
        eac_local.line = eac_local.mic = DEFAULT_INPUT_VOLUME;
	// MPU volume control
        eac_local.volume = DEFAULT_VOLUME;
        eac_update();
	// No sidetone
	temp = omap730_eac_read((u16 *) EAC_ASTCTR);
	temp &= ~EAC_ASTCTR_ATTEN;
	omap730_eac_write((u16 *) EAC_ASTCTR, temp);
	
	// Audio processing enable
	temp = omap730_eac_read((u16 *) EAC_AGCTR);
	temp |= EAC_AGCTR_AUDEN;
	omap730_eac_write((u16 *) EAC_AGCTR, temp);
	
	/*
	 * Codec port setup
	 */

	// CODEC Port Interface Control and Status Register
	temp = omap730_eac_read((u16 *) EAC_CPTCTL) & EAC_CPTCTL_RESERVED;
	// CODEC RESET release , clear RECEIVE DATA REGISTER FULL and TRANSMIT DATA REGISTER EMPTY
	temp |= EAC_CPTCTL_CRST | EAC_CPTCTL_TXE | EAC_CPTCTL_RXF;
	// C_PORT ENABLE Disabled to configure some registers
	temp &= ~EAC_CPTCTL_CPEN;
	omap730_eac_write((u16 *) EAC_CPTCTL, temp);	

	// Codec Port Configuration Register 1
	// Codec-Port interface mode: I2S mode, Number of time slots per audio frame: 2 time slots per frame
	omap730_eac_write((u16 *) EAC_CPCFR1, EAC_CPCFR1_MODE_I2S);	
	
	// CODEC PORT CONFIGURATION REGISTER 2
	omap730_eac_write((u16 *) EAC_CPCFR2, EAC_CPCFR2_I2S_20BITS);	
	
	// CODEC PORT INTERFACE CONFIGURATION REGISTER 3
	omap730_eac_write((u16 *) EAC_CPCFR3, EAC_CPCFR3_I2S_INPUT);	

	// CODECPORT INTERFACE CONFIGURATION REGISTER 4
	// DIVB Calc: (12000000/(2*16*44100))-1=7
	omap730_eac_write((u16 *) EAC_CPCFR4, EAC_CPCFR4_I2S_DIV7);	
	
	// CODEC Port Interface Control and Status Register
	temp = omap730_eac_read((u16 *) EAC_CPTCTL) & EAC_CPTCTL_RESERVED;
	// C_PORT ENABLE Enabled
	temp |= EAC_CPTCTL_CPEN;
	omap730_eac_write((u16 *) EAC_CPTCTL, temp);	

	/*
	 * Modem port setup
	 */

	// Modem Port Control Register
	omap730_eac_write((u16 *) EAC_MPCTR, EAC_MPCTR_DISABLEALL);	
	
	// Modem Port Main channel Configuration Register
	omap730_eac_write((u16 *) EAC_MPMCCFR, EAC_MPMCCFR_DEFAULT_MASTER_NOCOMP_16BITS);	
	
	// Modem Port Control Register
	temp = omap730_eac_read((u16 *) EAC_MPCTR);
	temp |= EAC_MPCTR_PRE_MC_16 | EAC_MPCTR_MC_EN;	// Prescaler and enable
	omap730_eac_write((u16 *) EAC_MPCTR, temp);	
	temp |= EAC_MPCTR_CKEN;				// Clocks running
	omap730_eac_write((u16 *) EAC_MPCTR, temp);	

	/*
	 * Bluetooth port setup
	 */

	// Bluetooth Port Control Register
	omap730_eac_write((u16 *) EAC_BPCTR, EAC_BPCTR_DISABLEALL);	
	
	// Bluetooth Port Main channel Configuration Register
	omap730_eac_write((u16 *) EAC_BPMCCFR, EAC_BPMCCFR_DEFAULT_SLAVE_NOCOMP_16BITS);

	// Modem Port Control Register
	temp = omap730_eac_read((u16 *) EAC_BPCTR);
	temp |= EAC_BPCTR_PRE_MC_16 | EAC_BPCTR_MC_EN;	// Prescaler and enable
	omap730_eac_write((u16 *) EAC_BPCTR, temp);	
	temp |= EAC_BPCTR_CKEN;				// Clocks running
	omap730_eac_write((u16 *) EAC_BPCTR, temp);	


	/*
	 * Driver init
	 */

	/* register devices */
	audio_dev_id = register_sound_dsp(&omap730_audio_fops, -1);
	mixer_dev_id = register_sound_mixer(&omap730_mixer_fops, -1);

#ifdef CONFIG_CEE  /* MVL-CCE */
        audio_ldm_device_register();
        audio_ldm_driver_register();
#endif /* MVL-CCE */

	printk(KERN_INFO "OMAP730 audio support initialized\n");

	eac_dump();

	FN_OUT(0);
	return 0;
}

static void __exit omap730_syren_exit(void)
{

#ifdef CONFIG_CEE  /* MVL-CCE */
        audio_ldm_device_unregister();
        audio_ldm_driver_unregister();
#endif /* MVL-CCE */

	unregister_sound_dsp(audio_dev_id);
	unregister_sound_mixer(mixer_dev_id);
}

module_init(omap730_syren_init);
module_exit(omap730_syren_exit);

MODULE_AUTHOR("Jean Pihet");
MODULE_DESCRIPTION("Glue audio driver for the TI OMAP730 & TI TWL3016 (Syren) CODEC");
MODULE_LICENSE("GPL");

EXPORT_NO_SYMBOLS;
