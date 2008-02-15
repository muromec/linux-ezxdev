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
 *  zhouqiong          Jun 20,2002            created
 *  zhouqiong          Sep 19,2002            according code review meeting minutes.
 *  zhouqiong          Oct 30,2002            according new requirement for VA.ASSP interface split to
 *                                                /dev/dsp (support stereo playback) and /dev/dsp16 (support
 *                                                mono playback and record).this file is for mono playback and record
 *  zhouqiong          Nov 05,2002            according code review meeting minutes.
 *  zhouqiong          Mar 04,2003            (1) don't close headset interrupt;
 *                                            (2) when headset in, output gain decrease 6db
 *  zhouqiong          Apr 24,2003            no switch for headset insert and remove
 *  LiYong             Sep 23,2003            Port from EZX
 *  Jin Lihong(w20076) Mar.15,2004,LIBdd86574  mixer bug fix
 *  Jin Lihong(w20076) Apr.13,2004,LIBdd96876  close dsp protection,and add 3d control interface for app
 *  lin weiqiang       Jun.08,2004,LIBee14656  record noise bug fix.
 *  Lv Yunguang(a6511c) Mar.10,2005            Change the debug log for barbados
 * 
 */
 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/completion.h>
#include <linux/poll.h>
#include <linux/sound.h>
#include <linux/soundcard.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/dma.h>

#include "ezx-audio.h"
#include "ezx-common.h"


static DECLARE_MUTEX(cotulla_ssp2_mono_mutex);

static int ssp2_mono_init(void);
static void ssp2_mono_shutdown(void);


/*initialize hardware, ssp controller and pcap register*/
static int ssp2_mono_init(void)
{
	unsigned long flags;
	unsigned int audiostatus;
	unsigned long timeout;

	down(&cotulla_ssp2_mono_mutex);
		
        AUDPRINTk1(EZXOSS_DEBUG "setup ssp2 controller register \n");
	local_irq_save(flags);
	CKEN |= CKEN3_NSSP;	/* need enable cken4  */

	set_GPIO_mode(GPIO_BITCLK_IN_NSSP_MD);                  /* BitCLK */
	set_GPIO_mode(GPIO_SYNC_IN_NSSP_MD);                    /* FS */
	set_GPIO_mode(GPIO_SDATA_OUT_NSSP_MD);		            /* TXD ssp2 ALF out 2 */
	set_GPIO_mode(GPIO_SDATA_IN_NSSP_MD);	                /* RXD ssp2 ALF in 1*/

	NSSCR0 = NSSCR0_FRF_PSP | NSSCR0_DSS_16bit;             /* PSP mode, 16bit */
	NSSCR1 = NSSCR1_TTE | NSSCR1_EBCEI | NSSCR1_SCLKDIR | NSSCR1_SFRMDIR | NSSCR1_TFTH_4 | NSSCR1_RFTH_14;
	NSSPSP = NSSPSP_SFRMWDTH_1 | NSSPSP_STRTDLY_1 |  NSSPSP_SFRMP_HIGH | NSSPSP_SCMODE;	
	NSSCR1 |= NSSCR1_TSRE | NSSCR1_RSRE;	                /* enable dma request */
	NSSCR0 |= NSSCR0_SSE;	                                /* enable nssp controller */

	local_irq_restore(flags);

        AUDPRINTk1(EZXOSS_DEBUG "NSSCR0 = 0x%lx\n", NSSCR0 );
        AUDPRINTk1(EZXOSS_DEBUG "NSSCR1 = 0x%lx\n", NSSCR1 );
        AUDPRINTk1(EZXOSS_DEBUG "NSSPSP = 0x%lx\n", NSSPSP );
	
        AUDPRINTk1(EZXOSS_DEBUG "setup pcap audio register\n");
	set_pcap_telephone_codec();
	poweron_mixer(DSP16_DEVICE);
	set_audio_output(codec_output_path);
	set_audio_input(codec_input_path);
	set_input_gain_hw_reg();

	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, CDC_SW_INDEX, PCAP_BIT_SET_VALUE, CDC_SW_NUM_BITS );

	pcap_use_ap_13m_clock();

        print_pcap_audio_reg_vals();
	timeout = 0;
	/* check if ssp is ready for slave operation	*/
	while(((audiostatus = NSSSR) & NSSSR_CSS) !=0){
		if((timeout++) > 10000000)
			goto err;
	}
	
	up(&cotulla_ssp2_mono_mutex);

        AUDPRINTk1(EZXOSS_DEBUG " complete all hardware init \n");
	return 0;

err:
	up(&cotulla_ssp2_mono_mutex);
	printk(EZXOSS_DEBUG "audio panic2: ssp don't ready for slave operation!!! ");
	return -ENODEV;
}


static void ssp2_mono_shutdown(void)
{
	down(&cotulla_ssp2_mono_mutex); 

	/* clear SSP2 port */
        AUDPRINTk1(EZXOSS_DEBUG "close dsp16\n");
	NSSCR0 = 0;
	NSSCR1 = 0;
	NSSPSP = 0;
	CKEN &= ~CKEN3_NSSP;    /* SSP2 need control the CKEN3  */ 

	set_GPIO_mode(GPIO_NSSP_SCLK2 | GPIO_IN);                  /* BitCLK */
        set_GPIO_mode(GPIO_NSSP_SFRM2 | GPIO_IN);                  /* FS */
        set_GPIO_mode(GPIO_NSSP_TXD2  | GPIO_IN);
        set_GPIO_mode(GPIO_NSSP_RXD2  | GPIO_IN);

        AUDPRINTk1(EZXOSS_DEBUG "close pcap register\n");
	
	/* close pcap output path */
	shutdown_mixer(DSP16_DEVICE);
	/* disable PCAP mono codec */
	power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_CLK_EN_INDEX, PCAP_BIT_CLEAN_VALUE, CDC_CLK_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_EN_INDEX, PCAP_BIT_CLEAN_VALUE, CDC_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, SMB_INDEX, PCAP_BIT_SET_VALUE, SMB_NUM_BITS );
	/* set fsync, tx, bitclk are tri-stated */
	power_ic_set_reg_value( PCAP_AUD_CODEC, CD_TS_INDEX, PCAP_BIT_SET_VALUE, CD_TS_NUM_BITS );

	if( STEREODEVOPENED )
		pcap_use_ap_13m_clock();
	else
		pcap_use_bp_13m_clock();

        print_pcap_audio_reg_vals();
	up(&cotulla_ssp2_mono_mutex);
}


/*
 * SSP2 codec ioctls
 */
static int codec_adc_rate = PHONE_CODEC_DEFAULT_RATE;	/* default 8k sample rate */
static int codec_dac_rate = PHONE_CODEC_DEFAULT_RATE;	/* default 8k sample rate */

static int ssp2_mono_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int ret;
	long val=0;
	int audiostatus, timeout;

	switch(cmd) {
	case SNDCTL_DSP_STEREO:
                AUDPRINTk1(" check if support stereo\n");
		ret = get_user(val, (int *) arg);
		if (ret)
			return ret;
	
		if(val) 
			ret = -EINVAL;	/* not support stereo */
		else  
		   	ret = 1;
		return put_user(ret, (int *) arg);

	case SNDCTL_DSP_CHANNELS:
	case SOUND_PCM_READ_CHANNELS:
                AUDPRINTk1(" check if 2 channels \n");
		return put_user(1, (long *) arg);

	case SNDCTL_DSP_SPEED:
                AUDPRINTk1(" set sample frequency \n");
		ret = get_user(val, (long *) arg);
		if (ret)
			return ret;

	    down(&cotulla_ssp2_mono_mutex); 
	    NSSCR0 &= ~NSSCR0_SSE; 
	    power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_CLK_EN_INDEX, PCAP_BIT_CLEAN_VALUE, CDC_CLK_EN_NUM_BITS );
	    power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_EN_INDEX, PCAP_BIT_CLEAN_VALUE, CDC_EN_NUM_BITS );
	    switch(val)
		{
		   	 case PHONE_CODEC_16K_RATE: 
					ret = power_ic_set_reg_value( PCAP_AUD_CODEC, FS_8K_16K_INDEX, PCAP_BIT_SET_VALUE, FS_8K_16K_NUM_BITS );
					codec_adc_rate = val;	
					codec_dac_rate = val;	
					break;
		   	 case PHONE_CODEC_DEFAULT_RATE: 
					ret = power_ic_set_reg_value( PCAP_AUD_CODEC, FS_8K_16K_INDEX, PCAP_BIT_CLEAN_VALUE, FS_8K_16K_NUM_BITS );
					codec_adc_rate = val;	
					codec_dac_rate = val;	
					break;
		   	 default:
					ret = -EINVAL;
 					break;	
		}	
		/* reset digital filter(DF_RESET=1)  */
		power_ic_set_reg_value( PCAP_AUD_CODEC, DF_RESET_INDEX, PCAP_BIT_SET_VALUE, DF_RESET_NUM_BITS );
		power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_CLK_EN_INDEX, PCAP_BIT_SET_VALUE, CDC_CLK_EN_NUM_BITS );
		power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_EN_INDEX, PCAP_BIT_SET_VALUE, CDC_EN_NUM_BITS );
		
		NSSCR0 |= NSSCR0_SSE;	/* enable ssp2 controller  */
		timeout = 0;
		/* check if ssp is ready for slave operation	*/
		while(((audiostatus = NSSSR) & NSSSR_CSS) !=0)
		{
			if((timeout++) > 10000000)
			{
				printk("audio panic3: can't be slave mode!!!");
				ret = -ENODEV;
				break;
			}
		}
                AUDPRINTk1("AD sample freq = %d\n", codec_adc_rate);
                AUDPRINTk1("DA sample freq = %d\n", codec_dac_rate); 
	        up(&cotulla_ssp2_mono_mutex); 
		return put_user(codec_adc_rate, (long *) arg);

	case SOUND_PCM_READ_RATE:
		if (file->f_mode & FMODE_WRITE)
		{
                        AUDPRINTk1("read DA sample freq\n");
			val = codec_dac_rate;
		}
		if (file->f_mode & FMODE_READ)
		{
                        AUDPRINTk1("read AD sample freq\n");
			val = codec_adc_rate;
		}
		return put_user(val, (long *) arg);

	case SNDCTL_DSP_SETFMT:
	case SNDCTL_DSP_GETFMTS: 
		/* SUPPORT little endian signed 16 */
                AUDPRINTk1("data format is AFMT_S16_LEd\n");
		return put_user(AFMT_S16_LE, (long *) arg); 
		
	default:
		return mixer_ioctl(inode, file, cmd, arg);
	}
	return 0;
}


/*
 * Audio stuff
 */
static audio_stream_t ssp2_mono_audio_out = {
	name:			"ssp2 mono audio out",
	dcmd:			DCMD_TXNSSDR,
	drcmr:			&DRCMRTXNSSDR,  /* NSSP dma map register */
	dev_addr:		__PREG(NSSDR),
};

static audio_stream_t ssp2_mono_audio_in = {
	name:			"ssp2 mono audio in",
	dcmd:			DCMD_RXNSSDR,
	drcmr:			&DRCMRRXNSSDR,  /* SSP2 dma map register */
	dev_addr:		__PREG(NSSDR),
};

static audio_state_t ssp2_mono_audio_state = {
	output_stream:		&ssp2_mono_audio_out,
	input_stream:		&ssp2_mono_audio_in,
	client_ioctl:		ssp2_mono_ioctl,
	hw_init:		ssp2_mono_init,
	hw_shutdown:		ssp2_mono_shutdown,
	sem:			__MUTEX_INITIALIZER(ssp2_mono_audio_state.sem),
};

static int ssp2_mono_audio_open(struct inode *inode, struct file *file)
{
        AUDPRINTk1("mono open \n");
	if( audioonflag & (CALL_DEVICE|DSP16_DEVICE|BT_DEVICE) ){
		AUDPRINTk1(EZXOSS_DEBUG "open dsp16 EBUSY because 0x%X device is using the sound hardware.\n",audioonflag );

		return -EBUSY;
	}

	return cotulla_audio_attach(inode, file, &ssp2_mono_audio_state);
}

/*
 * Missing fields of this structure will be patched with the call
 * to cotulla_audio_attach().
 */

static struct file_operations ssp2_mono_audio_fops = {
	open:		ssp2_mono_audio_open,
	owner:		THIS_MODULE
};

static int __init cotulla_ssp2_mono_init(void)
{
	ssp2_mono_audio_state.dev_dsp = register_sound_dsp16(&ssp2_mono_audio_fops, -1);

        AUDPRINTk1("/dev/dsp16 init ok\n");
	return 0;
}

static void __exit cotulla_ssp2_mono_exit(void)
{
	unregister_sound_dsp16(ssp2_mono_audio_state.dev_dsp);
        AUDPRINTk1("/dev/dsp16 exit ok\n");
}

module_init(cotulla_ssp2_mono_init);
module_exit(cotulla_ssp2_mono_exit);



