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
 *  History:
 *  zhouqiong          Jun 20,2002             created
 *  zhouqiong          Sep 19,2002             according code review meeting minutes.
 *  zhouqiong          Oct 30,2002             according new requirement for VA.ASSP interface split to
 *                                             /dev/dsp (support stereo playback) and /dev/dsp16 (support
 *                                             mono playback and record) this file is for stereo playback.
 *  zhouqiong          Nov 05,2002             according code review meeting minutes.
 *  zhouqiong          Jan 13,2003             (1) add audio panic return value
 *                                             (2) modify sample frequency to standard
 *  zhouqiong          Mar 03,2003             (1) open headset interrupt
 *                                             (2) change gain when headset is in
 *                                             (3) add ioctl to get headset status
 *  zhouqiong          Apr 17,2003             (1) according codec_dac_rate init pcap
 *  zhouqiong          Apr 18,2003             (1) change output gain according output path
 *  zhouqiong          Apr 24,2003             (1) no switch when headset insert and remove
 *  zhouqiong          May 21,2003             (1) modify loudspk gain max 0db, for audio-shaping
 *  LiYong             Sep 23,2003             (1)Port from EZX; (2)Modify the ASSP port inital
 *  Jia Tong(w19836)   Feb 04,2004,LIBdd67717  haptics feature added
 *  Jia Tong(w19836)   Feb 23,2004,LIBdd79841  haptics GPIO initialization change
 *  Li  Yong(w19946)   Feb 26,2004 LIBdd80614  Add DAI test
 *                                             Add control to switch PCAP CODEC mode from master to slave mode
 *  Jin Lihong(w20076) Mar.15,2004,LIBdd86574  mixer bug fix
 *  Jia Tong(w19836)   Mar 17,2004,LIBdd87621  GPIO change for haptics filter & boomer mute while setting haptics.
 *  Jin Lihong(w20076) Apr.13,2004,LIBdd96876  close dsp protection,and add 3d control interface for app
 *   Li  Yong(w19946)  Apr.23.2004.LIBee02702  Add EMU Carkit 
 *   Li  Yong(w19946)  May.23.2004.LIBee12065  Add the EMU audio test
 *  lin weiqiang       Jun.08,2004,LIBee14656  record noise bug fix.
 *  Jin Lihong(w20076) Jun.22,2004,LIBee24284  mixer power save
 *  Jin Lihong(w20076) Aug.11,2004,LIBff01482  audio pcap LOW_POWER bit initialize
 *  Lv Yunguang(a6511c) Mar.10,2005            Change the debug log for barbados 
 *
 */
 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/dma.h>

#include "ezx-audio.h"
#include "ezx-common.h"


static DECLARE_MUTEX(cotulla_assp_mutex);

static int codec_dac_rate = STEREO_CODEC_44K_RATE;	 /* default 44k sample rate */

#ifdef CONFIG_PM
static struct pm_dev *mixer_hw_pm_dev;
#endif


EXPORT_SYMBOL(mixer_ioctl);

static int assp_init(void);
static void assp_shutdown(void);

/*initialize hardware, assp controller and pcap register*/
static int assp_init(void)
{
	unsigned long flags;
	unsigned long audiostatus;
	unsigned long timeout;
	int ret;
	
	down(&cotulla_assp_mutex); 
        AUDPRINTk1(EZXOSS_DEBUG "setup assp controller register \n");
	local_irq_save(flags);
	CKEN |= CKEN4_ASSP;	                           /* need enable cken4  */

	set_GPIO_mode(GPIO_SYNC_IN_ASSP_MD);             /* Assp Frame sync */
	set_GPIO_mode(GPIO_SDATA_OUT_ASSP_MD);           /* Assp TX */
	set_GPIO_mode(GPIO_BITCLK_IN_ASSP_MD);           /* ASSP BitCLK  */
	set_GPIO_mode(GPIO_SDATA_IN_ASSP_MD);            /* ASsp RX */

	/* setup assp port */
	ASSCR0 = ASSCR0_FRF_PSP | ASSCR0_EDSS | ASSCR0_DSS_16bit;  /* PSP mode, 32bit */
	ASSCR1 = ASSCR1_EBCEI | ASSCR1_SCLKDIR | ASSCR1_SFRMDIR | ASSCR1_TFTH_4;
	ASSPSP = ASSPSP_SFRMWDTH_16 | ASSPSP_SCMODE;	
	ASSCR1 |= ASSCR1_TSRE;	                           /* enable transmit dma request */
	ASSCR0 |= ASSCR0_SSE;	                           /* enable assp controller */
	local_irq_restore(flags);

	AUDPRINTk1(EZXOSS_DEBUG "ASSCR0 = 0x%lx\n", ASSCR0 );
        AUDPRINTk1(EZXOSS_DEBUG "ASSCR1 = 0x%lx\n", ASSCR1 );
        AUDPRINTk1(EZXOSS_DEBUG "ASSPSP = 0x%lx\n", ASSPSP );

        AUDPRINTk1(EZXOSS_DEBUG "setup pcap audio register\n");

	poweron_mixer(DSP_DEVICE);
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ST_DAC_SW_INDEX, PCAP_BIT_SET_VALUE, ST_DAC_SW_NUM_BITS );

	/* (1) set bitclk(BCK=3, two_time_slot) pll clock(ST_CLK=0, 13M) NORMAL mode(DIG_AUD_FS =00) */
	power_ic_set_reg_value( PCAP_ST_DAC, PCAP_ST_DAC_INDEX, PCAP_BIT_CLEAN_VALUE, PCAP_ST_DAC_NUM_BITS );
        power_ic_set_reg_value( PCAP_ST_DAC,BCLK_INDEX, ST_BCLK_SLOT_2, BCLK_NUM_BITS );
	power_ic_set_reg_value( PCAP_ST_DAC,ST_CLK_INDEX, ST_CLK_PLL_CLK_IN_13M0, ST_CLK_NUM_BITS );
	power_ic_set_reg_value( PCAP_ST_DAC,DIG_AUD_FS_INDEX, DIGITAL_AUDIO_INTERFACE_NORMAL, DIG_AUD_FS_NUM_BITS ); 

	if( PHONENOTOPENED )
		pcap_use_ap_13m_clock();

	/* set stereo sample rate */
	switch(codec_dac_rate)
	{
	   	 case STEREO_CODEC_48K_RATE:	
			PCAP_CDC_SR_set(ST_SAMPLE_RATE_48K);
		      break;

	   	 case STEREO_CODEC_44K_RATE:    
			PCAP_CDC_SR_set(ST_SAMPLE_RATE_44K);
		      break;

	   	 case STEREO_CODEC_32K_RATE: 	
			PCAP_CDC_SR_set(ST_SAMPLE_RATE_32K);
		      break;

	   	 case STEREO_CODEC_24K_RATE: 	
			PCAP_CDC_SR_set(ST_SAMPLE_RATE_24K);
	                     break;

	   	 case STEREO_CODEC_22K_RATE:	
			PCAP_CDC_SR_set(ST_SAMPLE_RATE_22K);
		      break;

	   	 case STEREO_CODEC_16K_RATE:	
			PCAP_CDC_SR_set(ST_SAMPLE_RATE_16K);
		      break;

	   	 case STEREO_CODEC_12K_RATE: 	
			PCAP_CDC_SR_set(ST_SAMPLE_RATE_12K);
		      break;

	   	 case STEREO_CODEC_11K_RATE:	
			PCAP_CDC_SR_set(ST_SAMPLE_RATE_11K);
		      break;

	   	 case STEREO_CODEC_8K_RATE: 	
			PCAP_CDC_SR_set(ST_SAMPLE_RATE_8K);
		      break;
	   	 default:
			PCAP_CDC_SR_set(ST_SAMPLE_RATE_44K);
		      break;
	}
        AUDPRINTk1(EZXOSS_DEBUG "codec_dac_rate=%d\n", codec_dac_rate);
	power_ic_set_reg_value( PCAP_ST_DAC,ST_CLK_INV_INDEX, PCAP_BIT_CLEAN_VALUE, ST_CLK_INV_NUM_BITS );
	power_ic_set_reg_value( PCAP_ST_DAC,ST_FS_INV_INDEX, PCAP_BIT_SET_VALUE, ST_FS_INV_NUM_BITS );

	/* (3) reset digital filter(DF_RESET_ST_DAC=1) */
	power_ic_set_reg_value( PCAP_ST_DAC,DF_RESET_ST_DAC_INDEX, PCAP_BIT_SET_VALUE, DF_RESET_ST_DAC_NUM_BITS );

	/* (4)set  bitclk output(SMB_ST_DAC=0), audio IO=part1(DIG_AUD_IN_ST_DAC=1) */
	power_ic_set_reg_value( PCAP_ST_DAC,SMB_ST_DAC_INDEX, PCAP_BIT_CLEAN_VALUE, SMB_ST_DAC_NUM_BITS );
	power_ic_set_reg_value( PCAP_ST_DAC,DIG_AUD_IN_ST_DAC_INDEX, PCAP_BIT_SET_VALUE, DIG_AUD_IN_ST_DAC_NUM_BITS );

	/* (5) enable pcap clk(ST_CLK_EN=1),enable dac(ST_DAC_EN=1)  */
	power_ic_set_reg_value( PCAP_ST_DAC,ST_CLK_EN_INDEX, PCAP_BIT_SET_VALUE, ST_CLK_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_ST_DAC,ST_DAC_EN_INDEX, PCAP_BIT_SET_VALUE, ST_DAC_EN_NUM_BITS );
	mdelay(1);	/* specified enable time according spec */		

        AUDPRINTk1(EZXOSS_DEBUG "codec_output_path=%d\n", codec_output_path);
	set_audio_output(codec_output_path);

        print_pcap_audio_reg_vals();
	timeout = 0;
	/* check if ssp is ready for slave operation	*/
	while(((audiostatus = ASSSR) & ASSSR_CSS) !=0)	{
		if((timeout++) > 10000000)
			goto err;
	}

        AUDPRINTk1(EZXOSS_DEBUG " complete all hardware init \n");
	up(&cotulla_assp_mutex);
		
	return 0;
err:
	up(&cotulla_assp_mutex);
	printk(EZXOSS_DEBUG "audio panic: ssp don't ready for slave operation!!! ");
	return -ENODEV;	
}


static void assp_shutdown(void)
{
	down(&cotulla_assp_mutex);
        AUDPRINTk1(EZXOSS_DEBUG "close assp port\n");
	ASSCR0 = 0;
	ASSCR1 = 0;
	ASSPSP = 0;
	CKEN &= ~CKEN4_ASSP; 

	set_GPIO_mode(GPIO_ASSP_SCLK3 | GPIO_IN);             /* Assp Frame sync */
        set_GPIO_mode(GPIO_ASSP_TXD3  | GPIO_IN);
        set_GPIO_mode(GPIO_ASSP_RXD3  | GPIO_IN);           /* ASSP BitCLK  */
        set_GPIO_mode(GPIO_ASSP_SFRM3 | GPIO_IN);            /* ASsp RX */
	up(&cotulla_assp_mutex);

        AUDPRINTk1(EZXOSS_DEBUG "close pcap register\n");

	shutdown_mixer(DSP_DEVICE);
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS,ST_DAC_SW_INDEX, PCAP_BIT_CLEAN_VALUE, ST_DAC_SW_NUM_BITS );

	/* disable PCAP stereo DAC */
	power_ic_set_reg_value( PCAP_ST_DAC,ST_CLK_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ST_CLK_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_ST_DAC,ST_DAC_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ST_DAC_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_ST_DAC,SMB_ST_DAC_INDEX, PCAP_BIT_SET_VALUE, SMB_ST_DAC_NUM_BITS );

	if( audioonflag == 0 )
		pcap_use_bp_13m_clock();
}


/*
 * Audio Mixer stuff
 */
int mixer_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret=0;
	long val;
#ifdef EZX_OSS_DEBUG
	unsigned long ssp_pcap_register_val;
#endif

	switch(cmd) 
	{
		case SOUND_MIXER_READ_IGAIN:
			val = codec_input_gain;
                        AUDPRINTk1(EZXOSS_DEBUG "read input gain=%d\n", val);
			return put_user(val, (long *)arg);
			
		case SOUND_MIXER_WRITE_IGAIN:
			ret = get_user(val, (long *) arg);
			if (ret)
				return ret;
			
			if( (val<EZX_OSS_MIN_LOGICAL_GAIN) || (val>EZX_OSS_MAX_LOGICAL_GAIN) )
				ret = -EINVAL;
			else{
				codec_input_gain = val;
                                AUDPRINTk1(EZXOSS_DEBUG " write input gain=%d\n", codec_input_gain);
				set_input_gain_hw_reg();
			}
			
			return put_user(ret, (int *) arg);

		case SOUND_MIXER_READ_OGAIN:
			/* read pcap ogain register  */
			val = codec_output_gain;	
                        AUDPRINTk1(EZXOSS_DEBUG " read output gain=%d\n", val);
			return put_user(val, (long *)arg);
	
		case SOUND_MIXER_WRITE_VOLUME:
		case SOUND_MIXER_WRITE_OGAIN:
			ret = get_user(val, (long *) arg);
			if (ret)
				return ret;
                        AUDPRINTk1(EZXOSS_DEBUG " write output gain=%d\n", val);

			if((val >= EZX_OSS_MIN_LOGICAL_GAIN)&&(val <=EZX_OSS_MAX_LOGICAL_GAIN))
			{
				codec_output_gain = val;
				/* write pcap ogain register */
				set_output_gain_hw_reg();
			}
			else
			{
				ret = -EINVAL; 
                                AUDPRINTk1(EZXOSS_DEBUG "value is invalid\n");
			}
			return put_user(ret, (int *) arg);

		case SOUND_MIXER_READ_RECSRC:
                        AUDPRINTk1(EZXOSS_DEBUG "read input path\n");
			/* read pcap input status, 0-extmic, 1-A5, 2-A3 */
			val = codec_input_path;
			return put_user(val, (long *)arg);

		case SOUND_MIXER_WRITE_RECSRC:
			ret = get_user(val, (long *) arg);
			if (ret)
				return ret;
                        AUDPRINTk1(EZXOSS_DEBUG "force write input path=%d\n", val);
			/* close old input path */
                     (*mixer_close_input_path[codec_input_path])();
			/* open input path  */
			if( (val>INPUT_PATH_MAX) || (val<INPUT_PATH_MIN) )
				ret = -EINVAL;
			else
				set_audio_input(val);

#ifdef EZX_OSS_DEBUG
		power_ic_read_reg(PCAP_TX_AUD_AMPS, &ssp_pcap_register_val);
                printk(EZXOSS_DEBUG "pcap register 26 = 0x%lx\n", ssp_pcap_register_val);
#endif
			return put_user(ret, (int *) arg);

		case SOUND_MIXER_READ_OUTSRC:
                        AUDPRINTk1(EZXOSS_DEBUG " read output path\n");
			val = codec_output_path;
			return put_user(val, (long *)arg);

		case SOUND_MIXER_WRITE_OUTSRC:
			ret = get_user(val, (long *) arg);
			if (ret)
				return ret;
                        AUDPRINTk1(EZXOSS_DEBUG "force write output path=0x%03X\n", val);
			/* close old output path */
                     (*mixer_close_output_path[codec_output_path])();

			/* set pcap output register */
			if( val>OUTPUT_PATH_MAX )
				ret = -EINVAL;
			else
				set_audio_output(val);

#ifdef EZX_OSS_DEBUG
		power_ic_read_reg(PCAP_RX_AUD_AMPS, &ssp_pcap_register_val);
              	printk(EZXOSS_DEBUG "pcap register 12 = 0x%lx\n", ssp_pcap_register_val);
#endif
			return put_user(ret, (int *) arg);

		case SOUND_MIXER_WRITE_MUTE:	/* mute output path */
			/* 0-unmute, 1-mute */
			ret = get_user(val, (long *) arg);
			if (ret)
				return ret;
			if(val == 1)	/* if mute  */
			{
                                AUDPRINTk1(EZXOSS_DEBUG " mute PGA\n");
				audio_output_mute = true;
				power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_R_EN_INDEX, PCAP_BIT_CLEAN_VALUE, PGA_R_EN_NUM_BITS );
				power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_L_EN_INDEX, PCAP_BIT_CLEAN_VALUE, PGA_L_EN_NUM_BITS );
				(*mixer_close_output_path[codec_output_path])();
			}
			else if(val == 0)	/*  unmute  */
			{
                                AUDPRINTk1(EZXOSS_DEBUG "unmute PGA\n");
				audio_output_mute = false;
				if(MONODEVOPENED){
					power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_R_EN_INDEX, PCAP_BIT_SET_VALUE, PGA_R_EN_NUM_BITS );
					set_audio_output(codec_output_path);
				}
				if(STEREODEVOPENED){
					power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_R_EN_INDEX, PCAP_BIT_SET_VALUE, PGA_R_EN_NUM_BITS );
					power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_L_EN_INDEX, PCAP_BIT_SET_VALUE, PGA_L_EN_NUM_BITS );
					set_audio_output(codec_output_path);
				}
			}
			else
			{
				ret = -EINVAL;
			}
			return put_user(ret, (int *) arg);

		case SOUND_MIXER_WRITE_INPUTMUTE:	/* mute input path  for DAI test */

			/* 0-unmute, 1-mute */
			ret = get_user(val, (long *) arg);
			if (ret)
				return ret;
			if(val == 1)	/* if mute  */
			{
                                AUDPRINTk1(EZXOSS_DEBUG " mute input\n");
				audio_input_mute = true;
                    		(*mixer_close_input_path[codec_input_path])();
			}
			else if(val == 0)	/*  unmute  */
			{
                                AUDPRINTk1(EZXOSS_DEBUG "unmute input\n");
				audio_input_mute = false;
				set_audio_input(codec_input_path);
			}
			else
			{
				ret = -EINVAL;
			}
			return put_user(ret, (int *) arg);

		case SOUND_MIXER_WRITE_LOOPBACK:  /* set loopback mode for DAI test */
			/* 0-unloopback, 1-loopback */
			ret = get_user(val, (long *) arg);
			if (ret)
				return ret;
			if(val == 1)	
			{
                            AUDPRINTk1(EZXOSS_DEBUG "loopback\n");
			    power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_EN_INDEX, PCAP_BIT_SET_VALUE, CDC_EN_NUM_BITS );
			    power_ic_set_reg_value( PCAP_RX_AUD_AMPS, CD_BYP_INDEX, PCAP_BIT_SET_VALUE, CD_BYP_NUM_BITS );
					  
			    	
			}
			else if(val ==0)
			{
                            AUDPRINTk1(EZXOSS_DEBUG "unloopback\n");
			    power_ic_set_reg_value( PCAP_RX_AUD_AMPS, CD_BYP_INDEX, PCAP_BIT_CLEAN_VALUE, CD_BYP_NUM_BITS );
			}
			else
			{
			    ret = -EINVAL;
			}
			return put_user(ret, (int *) arg);

		case SOUND_MIXER_WRITE_AUDOHPF:	/* set audio output High Pass filter for test command */
			/* 0-disable filter, 1-enable filter */
			ret = get_user(val, (long *) arg);
			if (ret)
				return ret;
			if(val == 1)	
			{
                            AUDPRINTk1(EZXOSS_DEBUG "enable audio output High Pass filter\n");
			    power_ic_set_reg_value( PCAP_AUD_CODEC, AUDOHPF_INDEX, PCAP_BIT_SET_VALUE, AUDOHPF_NUM_BITS );
			    power_ic_set_reg_value( PCAP_AUD_CODEC, DF_RESET_INDEX, PCAP_BIT_SET_VALUE, DF_RESET_NUM_BITS );
			    	
			}
			else if(val ==0)
			{
                            AUDPRINTk1(EZXOSS_DEBUG "disable audio output High Pass filter\n");
			    power_ic_set_reg_value( PCAP_AUD_CODEC, AUDOHPF_INDEX, PCAP_BIT_CLEAN_VALUE, AUDOHPF_NUM_BITS );
			    power_ic_set_reg_value( PCAP_AUD_CODEC, DF_RESET_INDEX, PCAP_BIT_SET_VALUE, DF_RESET_NUM_BITS );
			}
			else
			{
			    ret = -EINVAL;
			}
			return put_user(ret, (int *) arg);

		case SOUND_MIXER_WRITE_AUDIHPF:	/* set audio input High Pass filter for test command */
			/* 0-disable filter, 1-enable filter */
			ret = get_user(val, (long *) arg);
			if (ret)
				return ret;
			if(val == 1)	
			{
                            AUDPRINTk1("enable audio input High Pass filter\n");
			    power_ic_set_reg_value( PCAP_AUD_CODEC, AUDIHPF_INDEX, PCAP_BIT_SET_VALUE, AUDIHPF_NUM_BITS );
			    power_ic_set_reg_value( PCAP_AUD_CODEC, DF_RESET_INDEX, PCAP_BIT_SET_VALUE, DF_RESET_NUM_BITS );
			    	
			}
			else if(val ==0)
			{
                            AUDPRINTk1("disable audio input High Pass filter\n");
			    power_ic_set_reg_value( PCAP_AUD_CODEC, AUDIHPF_INDEX, PCAP_BIT_CLEAN_VALUE, AUDIHPF_NUM_BITS );
			    power_ic_set_reg_value( PCAP_AUD_CODEC, DF_RESET_INDEX, PCAP_BIT_SET_VALUE, DF_RESET_NUM_BITS );
			}
			else
			{
			    ret = -EINVAL;
			}
			return put_user(ret, (int *) arg);
		
		case SOUND_MIXER_WRITE_CODEC_SLAVE:
			AUDPRINTk1(EZXOSS_DEBUG "user set codec slave.\n");
			power_ic_set_reg_value( PCAP_AUD_CODEC,SMB_INDEX, PCAP_BIT_SET_VALUE, SMB_NUM_BITS );
			power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_CLK_EN_INDEX, PCAP_BIT_CLEAN_VALUE,CDC_CLK_EN_NUM_BITS );
			power_ic_set_reg_value( PCAP_AUD_CODEC, DF_RESET_INDEX, PCAP_BIT_SET_VALUE, DF_RESET_NUM_BITS );
		       return put_user(ret, (int *) arg);

		case SOUND_MIXER_WRITE_HW_ATTENU:
			AUDPRINTk1(EZXOSS_DEBUG "user set hw attenu.\n");
			ret = get_user(val, (long *) arg);
			if(ret)
				return ret;
			
			if(val == HW_ATTENUATION_USED){
				use_hw_noise_attenuate();
				ret = HW_ATTENUATION_USED;
			}
			else if(val == HW_ATTENUATION_BYPASSED){
				bypass_hw_noise_attenuate();
				ret = HW_ATTENUATION_BYPASSED;
			}
			else
				ret = -EINVAL;
			
			return put_user(ret, (int *) arg);

		default:
			return -EINVAL;
	}
	
	return 0;
}


static struct file_operations mixer_fops = {
	ioctl:		mixer_ioctl,
	owner:		THIS_MODULE
};


/*
 * ASSP codec ioctls
 */

static int assp_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int ret;
	long val;
	int audiostatus, timeout;

	switch(cmd) {
	case SNDCTL_DSP_STEREO:
                AUDPRINTk1(EZXOSS_DEBUG " check if support stereo\n");
		ret = get_user(val, (int *) arg);
		if (ret)
			return ret;
		if(file->f_mode & FMODE_WRITE)
		{
		   if(val) /* write only support stereo mode */
		   	ret = 1;
		   else  /* not support mono mode */		
		   	ret = -EINVAL;
		}
		else
		{
		   	ret = -EINVAL;
		}
		return put_user(ret, (int *) arg);

	case SNDCTL_DSP_CHANNELS:
	case SOUND_PCM_READ_CHANNELS:
                AUDPRINTk1(EZXOSS_DEBUG " check if 2 channels \n");
		if(file->f_mode & FMODE_WRITE)
		{
			return put_user(2, (long *) arg);
		}
		else
		{
			ret = -EINVAL;
			return put_user(ret, (long *) arg);	
		}

	case SNDCTL_DSP_SPEED:
                AUDPRINTk1(EZXOSS_DEBUG " set sample frequency \n");
		ret = get_user(val, (long *) arg);
		if (ret)
			return ret;

		if(file->f_mode & FMODE_WRITE)
		{
		   down(&cotulla_assp_mutex);
		   ASSCR0 &= ~ASSCR0_SSE; 

		   power_ic_set_reg_value( PCAP_ST_DAC, ST_CLK_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ST_CLK_EN_NUM_BITS );
		   power_ic_set_reg_value( PCAP_ST_DAC, ST_DAC_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ST_DAC_EN_NUM_BITS );
		   switch(val)
		   {
		   	 case STEREO_CODEC_48K_RATE:	
			      ret=PCAP_CDC_SR_set(ST_SAMPLE_RATE_48K);
			      			         /* set pcap dac sample rate  */
			      codec_dac_rate = val;
			      break;

		   	 case STEREO_CODEC_44K_RATE: 	
			      ret = PCAP_CDC_SR_set(ST_SAMPLE_RATE_44K);
							 /* set pcap dac sample rate  */
			      codec_dac_rate = val;
			      break;

		   	 case STEREO_CODEC_32K_RATE: 	
			      ret = PCAP_CDC_SR_set(ST_SAMPLE_RATE_32K);
							 /* set pcap dac sample rate  */
			      codec_dac_rate = val;
			      break;

		   	 case STEREO_CODEC_24K_RATE: 	
			      ret = PCAP_CDC_SR_set(ST_SAMPLE_RATE_24K);
							 /* set pcap dac sample rate  */
			      codec_dac_rate = val;
			      break;

		   	 case STEREO_CODEC_22K_RATE:	
			      ret = PCAP_CDC_SR_set(ST_SAMPLE_RATE_22K); 
							 /* set pcap dac sample rate  */
			      codec_dac_rate = val;
			      break;

		   	 case STEREO_CODEC_16K_RATE:	
			      ret = PCAP_CDC_SR_set(ST_SAMPLE_RATE_16K);
							 /* set pcap dac sample rate  */
			      codec_dac_rate = val;
			      break;

		   	 case STEREO_CODEC_12K_RATE: 	
			      ret = PCAP_CDC_SR_set(ST_SAMPLE_RATE_12K);
							 /* set pcap dac sample rate  */
			      codec_dac_rate = val;
			      break;

		   	 case STEREO_CODEC_11K_RATE:	
			      ret = PCAP_CDC_SR_set(ST_SAMPLE_RATE_11K);
							 /* set pcap dac sample rate  */
			      codec_dac_rate = val;
			      break;

		   	 case STEREO_CODEC_8K_RATE: 	
			      ret = PCAP_CDC_SR_set(ST_SAMPLE_RATE_8K);
							 /* set pcap dac sample rate  */
			      codec_dac_rate = val;
			      break;
		   	 default:
			      ret = -EINVAL;
 			      break;	
		   }
		   /* reset digital filter(DF_RESET_ST_DAC=1)   */
		  power_ic_set_reg_value( PCAP_ST_DAC, DF_RESET_ST_DAC_INDEX, PCAP_BIT_SET_VALUE, DF_RESET_ST_DAC_NUM_BITS);
		  power_ic_set_reg_value( PCAP_ST_DAC, ST_CLK_EN_INDEX, PCAP_BIT_SET_VALUE, ST_CLK_EN_NUM_BITS);
		  power_ic_set_reg_value( PCAP_ST_DAC, ST_DAC_EN_INDEX, PCAP_BIT_SET_VALUE, ST_DAC_EN_NUM_BITS);
                        AUDPRINTk1(EZXOSS_DEBUG "DA sample freq = %d\n", codec_dac_rate);     
			ASSCR0 |= ASSCR0_SSE;	/* enable assp controller  */
			timeout = 0;
			/* check if ssp is ready for slave operation	*/
			while(((audiostatus = ASSSR) & ASSSR_CSS) !=0)
			{
				if((timeout++) > 10000000)
				{
					printk(EZXOSS_DEBUG "audio panic: can't be slave mode!!!");
					ret = -ENODEV;
					break;
				}
			}
		   up(&cotulla_assp_mutex);
		}
		else
		{
			ret = -EINVAL;
		}
		return put_user(codec_dac_rate, (long *) arg);

	case SOUND_PCM_READ_RATE:
		if (file->f_mode & FMODE_WRITE)
		{
                        AUDPRINTk1(EZXOSS_DEBUG "read DA sample freq\n");
			val = codec_dac_rate;
		}
		else
		{
			val = -EINVAL;
		}
		return put_user(val, (long *) arg);

	case SNDCTL_DSP_SETFMT:
	case SNDCTL_DSP_GETFMTS: 
		/* SUPPORT little endian signed 16 */
                AUDPRINTk1(EZXOSS_DEBUG "data format is AFMT_S16_LEd\n");
		return put_user(AFMT_S16_LE, (long *) arg); 
		
	default:
		return mixer_ioctl(inode, file, cmd, arg);
	}
	return 0;
}


/*
 * Audio stuff
 */
static audio_stream_t assp_audio_out = {
	name:			"assp audio out",
	dcmd:			DCMD_TXASSDR,
	drcmr:			&DRCMRTXASSDR,  /* ASSP dma map register */
	dev_addr:		__PREG(ASSDR),
};


static audio_stream_t assp_audio_in = {
	name:			"assp audio in",
	dcmd:			DCMD_RXASSDR,
	drcmr:			&DRCMRRXASSDR,  /* ASSP dma map register */
	dev_addr:		__PREG(ASSDR),
};


static audio_state_t assp_audio_state = {
	output_stream:		&assp_audio_out,
	input_stream:		&assp_audio_in,
	client_ioctl:		assp_ioctl,
	hw_init:		assp_init,
	hw_shutdown:		assp_shutdown,
	sem:			__MUTEX_INITIALIZER(assp_audio_state.sem),
};


static int assp_audio_open(struct inode *inode, struct file *file)
{
        AUDPRINTk1("stereo audio open \n");
	if( audioonflag & DSP_DEVICE ){
		AUDPRINTk1(EZXOSS_DEBUG "open dsp EBUSY because 0x%X device is using the sound hardware.\n",audioonflag );

		return -EBUSY;
	}

	return cotulla_audio_attach(inode, file, &assp_audio_state);
}


/*
 * Missing fields of this structure will be patched with the call
 * to cotulla_audio_attach().
 */
static struct file_operations assp_audio_fops = {
	open:		assp_audio_open,
	owner:		THIS_MODULE
};


static int mixer_dev_id;

static int __init cotulla_assp_init(void)
{
	assp_audio_state.dev_dsp = register_sound_dsp(&assp_audio_fops, -1);
	mixer_dev_id = register_sound_mixer(&mixer_fops, -1);

	set_GPIO_mode(GPIO_ASSP_SCLK3 | GPIO_IN);			/* Assp Frame sync */
        set_GPIO_mode(GPIO_ASSP_TXD3  | GPIO_IN);   
	set_GPIO_mode(GPIO_ASSP_RXD3  | GPIO_IN);			/* ASSP BitCLK  */
	set_GPIO_mode(GPIO_ASSP_SFRM3 | GPIO_IN);			/* ASsp RX */

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AUDIO_LOW_PWR_INDEX, PCAP_BIT_CLEAN_VALUE, AUDIO_LOW_PWR_NUM_BITS );
	pcap_audio_init();

#ifdef CONFIG_PM
	mixer_hw_pm_dev = pm_register(PM_UNKNOWN_DEV, 0, mixer_hw_pm_callback);
#endif

        AUDPRINTk1(EZXOSS_DEBUG "cotulla-assp-init ok\n");
	return 0;
}


static void __exit cotulla_assp_exit(void)
{
	unregister_sound_dsp(assp_audio_state.dev_dsp);
	unregister_sound_mixer(mixer_dev_id);

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AUDIO_LOW_PWR_INDEX, PCAP_BIT_SET_VALUE, AUDIO_LOW_PWR_NUM_BITS );
#ifdef CONFIG_PM
	pm_unregister(mixer_hw_pm_dev);
#endif

        AUDPRINTk1(EZXOSS_DEBUG "cotulla-assp-exit ok\n");
}


module_init(cotulla_assp_init);
module_exit(cotulla_assp_exit);


