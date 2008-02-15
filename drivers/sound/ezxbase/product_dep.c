
/*
 * Copyright (C) 2004 Motorola Inc.
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
 *  Description:  Motorola phone hw dependent audio functions implementation
 *
 *  History:
 *  Jin Lihong(w20076) Jan 13,2004,LIBdd68327  Created
 *  Jin Lihong(w20076) Mar.15,2004,LIBdd86574  mixer bug fix
 *  Jin Lihong(w20076) Apr.13,2004,LIBdd96876  close dsp protection,and add 3d control interface for app
 *  Jin Lihong(w20076) Jun.22,2004,LIBee24284  mixer power save
 *  Cheng Xuefeng(a2491c) Jun.24,2004,LIBdd95397  Add EMU PIHF carkit sound path
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


u32 gpio_hw_attenuate_status;
typedef enum{
	EMU_HS_NONE=0X0,
	EMU_HS_INPUT_OPEN=0X1,
	EMU_HS_OUTPUT_OPEN=0X2
} emu_hs_status_t;
static emu_hs_status_t emu_hs_status=EMU_HS_NONE; 

void close_input_carkit(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close input carkit. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, EXT_MIC_MUX_INDEX, PCAP_BIT_CLEAN_VALUE, EXT_MIC_MUX_NUM_BITS );
}


void close_input_handset(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close input handset. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON1_INDEX, PCAP_BIT_CLEAN_VALUE, MB_ON1_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A5_EN_INDEX, PCAP_BIT_CLEAN_VALUE, A5_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A5_MUX_INDEX, PCAP_BIT_CLEAN_VALUE, A5_MUX_NUM_BITS );
}


void close_input_headset(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close input headset. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON2_INDEX, PCAP_BIT_CLEAN_VALUE, MB_ON2_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A3_EN_INDEX, PCAP_BIT_CLEAN_VALUE, A3_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A3_MUX_INDEX, PCAP_BIT_CLEAN_VALUE, A3_MUX_NUM_BITS );
}


void close_input_emuhs(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close input emuhs. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, EXT_MIC_MUX_INDEX, PCAP_BIT_CLEAN_VALUE, EXT_MIC_MUX_NUM_BITS );
	if ( (0X0 == (EMU_HS_OUTPUT_OPEN & emu_hs_status)) ) {
		power_ic_audio_conn_mode_set(POWER_IC_NO_SPEAKER_MASK,POWER_IC_NO_MIC);
	}
	 emu_hs_status &= ~EMU_HS_INPUT_OPEN;
}


void open_input_carkit(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "open input carkit. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, EXT_MIC_MUX_INDEX, PCAP_BIT_SET_VALUE, EXT_MIC_MUX_NUM_BITS );
}


void open_input_handset(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "open input handset. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON1_INDEX, PCAP_BIT_SET_VALUE, MB_ON1_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A5_EN_INDEX, PCAP_BIT_CLEAN_VALUE, A5_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A5_MUX_INDEX, PCAP_BIT_SET_VALUE, A5_MUX_NUM_BITS );
}


void open_input_headset(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "open input headset. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON2_INDEX, PCAP_BIT_SET_VALUE, MB_ON2_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A3_EN_INDEX, PCAP_BIT_CLEAN_VALUE, A3_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A3_MUX_INDEX, PCAP_BIT_SET_VALUE, A3_MUX_NUM_BITS );
}


void open_input_emuhs(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "open input emuhs. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, EXT_MIC_MUX_INDEX, PCAP_BIT_SET_VALUE, EXT_MIC_MUX_NUM_BITS );
	power_ic_audio_conn_mode_set(POWER_IC_EMU_HEADSET_SPEAKER_MASK,POWER_IC_EXTERNAL_MIC);
	emu_hs_status |= EMU_HS_INPUT_OPEN;
}


void close_output_pcap_headset(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close output pcap headset. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_CLEAN_VALUE, AHS_CONFIG_NUM_BITS );
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ARIGHT_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ALEFT_EN_NUM_BITS );
}


void close_output_pcap_louderspeaker(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close output pcap louderspeaker. \n");

	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, A2_EN_INDEX, PCAP_BIT_CLEAN_VALUE, A2_EN_NUM_BITS );
}


void close_output_pcap_earpiece(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close output pcap earpiece. \n");

	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, A1CTRL_INDEX, PCAP_BIT_SET_VALUE, A1CTRL_NUM_BITS );
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, A1_EN_INDEX, PCAP_BIT_CLEAN_VALUE, A1_EN_NUM_BITS );
}


void close_output_pcap_carkit(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close output pcap carkit. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_CLEAN_VALUE, AHS_CONFIG_NUM_BITS );
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ALEFT_EN_NUM_BITS );

      power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL,20,0);        
}


void close_output_pcap_headjack(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close output pcap headjack. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_CLEAN_VALUE, AHS_CONFIG_NUM_BITS );
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ARIGHT_EN_NUM_BITS );
}


void close_output_pcap_emuhs_st_media(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close output stereo emuhs media mode \n");
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_CLEAN_VALUE, AHS_CONFIG_NUM_BITS );
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ARIGHT_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ALEFT_EN_NUM_BITS );

	if ( 0X0 == (EMU_HS_INPUT_OPEN & emu_hs_status) ) {
		power_ic_audio_conn_mode_set(POWER_IC_NO_SPEAKER_MASK,POWER_IC_NO_MIC);        
	}
	emu_hs_status &= ~EMU_HS_OUTPUT_OPEN;
}


void close_output_pcap_emuhs_st_call(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close output stereo emuhs call mode \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_CLEAN_VALUE, AHS_CONFIG_NUM_BITS );
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ALEFT_EN_NUM_BITS );

	if ( 0X0 == (EMU_HS_INPUT_OPEN & emu_hs_status) ) {
		power_ic_audio_conn_mode_set(POWER_IC_NO_SPEAKER_MASK,POWER_IC_NO_MIC);
        }
	emu_hs_status &= ~EMU_HS_OUTPUT_OPEN;
}


void close_output_pcap_emuhs_mono(void)
{
	AUDPRINTk1(EZXOSS_DEBUG "close output emuhs mono \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_CLEAN_VALUE, AHS_CONFIG_NUM_BITS );
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ALEFT_EN_NUM_BITS );
 
	if (  0X0 == (EMU_HS_INPUT_OPEN & emu_hs_status) ) {
		power_ic_audio_conn_mode_set(POWER_IC_NO_SPEAKER_MASK,POWER_IC_NO_MIC);
        }
	emu_hs_status &= ~EMU_HS_OUTPUT_OPEN;
}


int open_output_pcap_headset(void)
{
	int ret;

	AUDPRINTk1(EZXOSS_DEBUG "open output pcap headset. \n");

	ret = power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_SET_VALUE, AHS_CONFIG_NUM_BITS );
        ret = power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_SET_VALUE, ARIGHT_EN_NUM_BITS );	
	ret = power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_SET_VALUE, ALEFT_EN_NUM_BITS );

	return ret;
}


int  open_output_pcap_louderspeaker(void)
{
	int ret;
    
	AUDPRINTk1(EZXOSS_DEBUG "open output pcap louderspeaker. \n");

	ret = power_ic_set_reg_value( PCAP_RX_AUD_AMPS, A2_EN_INDEX, PCAP_BIT_SET_VALUE, A2_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A2_CONFIG_INDEX, PCAP_BIT_SET_VALUE, A2_CONFIG_NUM_BITS );
    
	return ret;
}


int open_output_pcap_earpiece(void)
{
	int ret;

	AUDPRINTk1(EZXOSS_DEBUG "open output pcap earpiece. \n");

	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, A1CTRL_INDEX, PCAP_BIT_CLEAN_VALUE, A1CTRL_NUM_BITS );
	ret = power_ic_set_reg_value( PCAP_RX_AUD_AMPS, A1_EN_INDEX, PCAP_BIT_SET_VALUE, A1_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A1_CONFIG_INDEX, PCAP_BIT_SET_VALUE, A1_CONFIG_NUM_BITS );

	return ret;
}


int  open_output_pcap_carkit(void)
{
	int ret;

	AUDPRINTk1(EZXOSS_DEBUG "open output pcap carkit. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_SET_VALUE, AHS_CONFIG_NUM_BITS );
	ret = power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_SET_VALUE, ALEFT_EN_NUM_BITS );

         power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL,20,1);        
         power_ic_set_reg_bit(POWER_IC_REG_EOC_POWER_CONTROL_1,0,1);        
         power_ic_set_reg_bit(POWER_IC_REG_EOC_POWER_CONTROL_1,1,0); 

         power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL,14,0);        
         power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL,15,0);        
         power_ic_set_reg_bit(POWER_IC_REG_EOC_CONN_CONTROL,16,1); 

	return ret;
}


int  open_output_pcap_headjack(void)
{
	int ret;
    
	AUDPRINTk1(EZXOSS_DEBUG "open output pcap headjack. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_SET_VALUE, AHS_CONFIG_NUM_BITS );
	ret = power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_SET_VALUE, ARIGHT_EN_NUM_BITS );

	return ret;
}


int open_output_pcap_emuhs_st_media(void)
{
	int ret;

	AUDPRINTk1(EZXOSS_DEBUG "open output stereo emuhs media mode. \n");

	ret = power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_SET_VALUE, AHS_CONFIG_NUM_BITS );
        ret = power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_SET_VALUE, ARIGHT_EN_NUM_BITS );	
	ret = power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_SET_VALUE, ALEFT_EN_NUM_BITS );
	
	power_ic_audio_conn_mode_set(POWER_IC_EMU_ST_HEADSET_SPEAKER_MASK,POWER_IC_NO_MIC);
	emu_hs_status |= EMU_HS_OUTPUT_OPEN;

	return ret;
}


int open_output_pcap_emuhs_st_call(void)
{
	int ret;

	AUDPRINTk1(EZXOSS_DEBUG "open output stereo emuhs call mode. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_SET_VALUE, AHS_CONFIG_NUM_BITS );
	ret = power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_SET_VALUE, ALEFT_EN_NUM_BITS );

	power_ic_audio_conn_mode_set(POWER_IC_EMU_HEADSET_SPEAKER_MASK,POWER_IC_EXTERNAL_MIC);
	emu_hs_status |= EMU_HS_OUTPUT_OPEN;
	return ret;
}


int open_output_pcap_emuhs_mono(void)
{
	int ret;

	AUDPRINTk1(EZXOSS_DEBUG "open output emuhs mono. \n");

	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_SET_VALUE, AHS_CONFIG_NUM_BITS );
	ret = power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_SET_VALUE, ALEFT_EN_NUM_BITS );

	power_ic_audio_conn_mode_set(POWER_IC_EMU_HEADSET_SPEAKER_MASK,POWER_IC_EXTERNAL_MIC);
	emu_hs_status |= EMU_HS_OUTPUT_OPEN;
	
	return ret;
}



void set_output_gain_hw_reg(void)
{
	if(STEREODEVOPENED && MONODEVOPENED){
		switch( codec_output_path ){
			case EARPIECE_OUT:
			case LOUDERSPEAKER_OUT:
			case DIA25_STEREO_HS_OUT:
			case CARKIT_OUT:
			case DIA25_MONO_HS_OUT:
			case STEREO_EMUHS_MEDIA_OUT:
			case MONO_EMUHS_OUT:
			case DIA35_HS_MIC_MEDIA_OUT:
			case DIA35_HS_MIC_CALL_OUT:
			case DIA35_HS_NOMIC_MEDIA_OUT:
			case DIA35_HS_NOMIC_CALL_OUT:
			case STEREO_EMUHS_CALL_OUT:
				PCAP_MONO_set(MONO_PGA_RL_6DB);
				break;

			default:
				PCAP_MONO_set(MONO_PGA_R_L_STEREO);	/* bt */
		}
	}
	else if(MONODEVOPENED){
		switch( codec_output_path ){
			case EARPIECE_OUT:
			case LOUDERSPEAKER_OUT:
				PCAP_MONO_set(MONO_PGA_R_L_STEREO);
				break;

			case DIA25_STEREO_HS_OUT:
			case CARKIT_OUT:
			case DIA25_MONO_HS_OUT:
			case STEREO_EMUHS_MEDIA_OUT:
			case MONO_EMUHS_OUT:
			case DIA35_HS_MIC_MEDIA_OUT:
			case DIA35_HS_MIC_CALL_OUT:
			case DIA35_HS_NOMIC_MEDIA_OUT:
			case DIA35_HS_NOMIC_CALL_OUT:
			case STEREO_EMUHS_CALL_OUT:
				PCAP_MONO_set(MONO_PGA_RL);
				break;

			default:
				PCAP_MONO_set(MONO_PGA_R_L_STEREO);
		}
	}
	else{
		switch( codec_output_path ){
			case DIA25_STEREO_HS_OUT:
			case DIA25_MONO_HS_OUT:
			case STEREO_EMUHS_MEDIA_OUT:
			case DIA35_HS_MIC_MEDIA_OUT:
			case DIA35_HS_MIC_CALL_OUT:
			case DIA35_HS_NOMIC_MEDIA_OUT:
			case DIA35_HS_NOMIC_CALL_OUT:
			case STEREO_EMUHS_CALL_OUT:
				PCAP_MONO_set(MONO_PGA_R_L_STEREO);
				break;

			case EARPIECE_OUT:
			case LOUDERSPEAKER_OUT:
			case CARKIT_OUT:
			case MONO_EMUHS_OUT:
				PCAP_MONO_set(MONO_PGA_RL_6DB);
				break;

			default:
				PCAP_MONO_set(MONO_PGA_R_L_STEREO);
		}
	}

	PCAP_AUDOG_set( PCAP_OUTPUT_GAIN_REG_VAL_FROM_LOGIC );

	AUDPRINTk1(EZXOSS_DEBUG "codec_output_gain=%d\n", codec_output_gain);
	AUDPRINTk1(EZXOSS_DEBUG "output gain=%d\n",PCAP_OUTPUT_GAIN_REG_VAL_FROM_LOGIC);
	print_pcap_audio_reg_vals();
}


void set_input_gain_hw_reg(void)
{
	PCAP_AUDIG_set( PCAP_INPUT_AUDIG_REG_VAL_FROM_LOGIC );
	AUDPRINTk1(EZXOSS_DEBUG "codec_input_gain=%d\n", codec_input_gain);
	AUDPRINTk1(EZXOSS_DEBUG "input gain reg val: %d\n", PCAP_INPUT_AUDIG_REG_VAL_FROM_LOGIC);
}


void poweron_mixer( audio_dev_type type )
{
	AUDPRINTk1(EZXOSS_DEBUG "No. 0x%X device wants to power on the mixer hardware.\n", type);
	AUDPRINTk1(EZXOSS_DEBUG "No. 0x%X device has already powered on the mixer hardware.\n", audioonflag);

	audioonflag |= type;
	if( audioonflag == type )
	{
		AUDPRINTk1(EZXOSS_DEBUG "No. 0x%X device is powering on the mixer hardware.\n", type);
		power_ic_set_reg_value( PCAP_TX_AUD_AMPS, V2_EN_2_INDEX, PCAP_BIT_SET_VALUE, V2_EN_2_NUM_BITS );
		power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_R_EN_INDEX, PCAP_BIT_SET_VALUE, PGA_R_EN_NUM_BITS );
		
	}
	if( STEREODEVOPENED )
	{
		power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_L_EN_INDEX, PCAP_BIT_SET_VALUE, PGA_L_EN_NUM_BITS );
	} else {
		power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_L_EN_INDEX, PCAP_BIT_CLEAN_VALUE, PGA_L_EN_NUM_BITS );
	}
}


void shutdown_mixer( audio_dev_type type )
{
	AUDPRINTk1(EZXOSS_DEBUG "No. 0x%X device wants to shut down the mixer hardware.\n", type);

	audioonflag &= ~type;

	if( STEREODEVOPENED )
	{
		power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_L_EN_INDEX, PCAP_BIT_SET_VALUE, PGA_L_EN_NUM_BITS );
	} else {
		power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_L_EN_INDEX, PCAP_BIT_CLEAN_VALUE, PGA_L_EN_NUM_BITS );
	}
	if( audioonflag == 0 )
	{
		AUDPRINTk1(EZXOSS_DEBUG "No. 0x%X device is shutting down the mixer hardware.\n", type);
		power_ic_set_reg_value( PCAP_RX_AUD_AMPS, PGA_R_EN_INDEX, PCAP_BIT_CLEAN_VALUE, PGA_R_EN_NUM_BITS );
		(*mixer_close_output_path[codec_output_path])();
		(*mixer_close_input_path[codec_input_path])();

		power_ic_set_reg_value( PCAP_TX_AUD_AMPS, V2_EN_2_INDEX, PCAP_BIT_CLEAN_VALUE, V2_EN_2_NUM_BITS );
	}
	
	AUDPRINTk1(EZXOSS_DEBUG "No. 0x%X device is still using the mixer hardware.\n", audioonflag);

	set_output_gain_hw_reg();
}


void use_hw_noise_attenuate(void)
{
	AUDPRINTk1( EZXOSS_DEBUG "set hw noise attenuation gpio low. \n");

/*
	set_GPIO_mode(GPIO_HW_ATTENUATE | GPIO_OUT);
	clr_GPIO(GPIO_HW_ATTENUATE);
	PGSR(GPIO_HW_ATTENUATE) &= ~GPIO_bit(GPIO_HW_ATTENUATE);
        gpio_hw_attenuate_status = 0;
*/
}


void bypass_hw_noise_attenuate(void)
{
	AUDPRINTk1( EZXOSS_DEBUG "set hw noise attenuation gpio high. \n");

/*
	set_GPIO_mode(GPIO_HW_ATTENUATE | GPIO_OUT);
	set_GPIO(GPIO_HW_ATTENUATE);
	PGSR(GPIO_HW_ATTENUATE) |= GPIO_bit(GPIO_HW_ATTENUATE);
        gpio_hw_attenuate_status = 1;
*/
}


void pcap_use_ap_13m_clock(void)
{
	OSCC |= 0x00000008;
	set_GPIO_mode(AP_13MHZ_OUTPUT_PIN | GPIO_ALT_FN_3_OUT);

	power_ic_set_reg_value( PCAP_AUD_CODEC, CLK_IN_SEL_INDEX, PCAP_BIT_SET_VALUE, CLK_IN_SEL_NUM_BITS );
	power_ic_set_reg_value( PCAP_ST_DAC, ST_DAC_CLK_IN_SEL_INDEX, PCAP_BIT_SET_VALUE, ST_DAC_CLK_IN_SEL_NUM_BITS );
}


void pcap_use_bp_13m_clock(void)
{
	OSCC &= ~0x00000008;
	set_GPIO_mode(AP_13MHZ_OUTPUT_PIN | GPIO_IN);

	power_ic_set_reg_value( PCAP_AUD_CODEC, CLK_IN_SEL_INDEX, PCAP_BIT_CLEAN_VALUE, CLK_IN_SEL_NUM_BITS );
	power_ic_set_reg_value( PCAP_ST_DAC, ST_DAC_CLK_IN_SEL_INDEX, PCAP_BIT_CLEAN_VALUE, ST_DAC_CLK_IN_SEL_NUM_BITS );
}


#ifdef CONFIG_PM
int mixer_hw_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	switch(req){
		case PM_SUSPEND:
			AUDPRINTk1(EZXOSS_DEBUG "before AP sleep.\n");
			if( (audioonflag & (CALL_DEVICE|FM_DEVICE) ) == 0  )
			{
//				power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AUDIO_LOW_PWR_INDEX, PCAP_BIT_SET_VALUE, AUDIO_LOW_PWR_NUM_BITS );
                     (*mixer_close_input_path[codec_input_path])();
                     (*mixer_close_output_path[codec_output_path])();
			}
			break;
		case PM_RESUME:
			AUDPRINTk1(EZXOSS_DEBUG "after AP sleep.\n");
//			power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AUDIO_LOW_PWR_INDEX, PCAP_BIT_CLEAN_VALUE, AUDIO_LOW_PWR_NUM_BITS );
			set_output_gain_hw_reg();

			if(gpio_hw_attenuate_status) {
				bypass_hw_noise_attenuate();
			} else {
				use_hw_noise_attenuate();
			}

			set_audio_output(codec_output_path);
			set_audio_input(codec_input_path);

			break;
	}
	return 0;
}
#endif


void pcap_audio_init(void)
{
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A1_CONFIG_INDEX, PCAP_BIT_SET_VALUE, A1_CONFIG_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_SET_VALUE, AHS_CONFIG_NUM_BITS );
}


/* for 3MM5 headset support ==begin==*/
                                                                                                                             
void  close_input_3mm5_with_mic_call_mode_headset(void)
{
        AUDPRINTk1(EZXOSS_DEBUG "close input MOTO 3.5mm headset that were in phone call mode. \n");


        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A3_EN_INDEX, PCAP_BIT_CLEAN_VALUE, A3_EN_NUM_BITS );
        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A3_MUX_INDEX, PCAP_BIT_CLEAN_VALUE, A3_MUX_NUM_BITS );
        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON2_INDEX, PCAP_BIT_CLEAN_VALUE, MB_ON2_NUM_BITS );
}
                                                                                                                             
void   open_input_3mm5_with_mic_call_mode_headset(void)
{
        AUDPRINTk1(EZXOSS_DEBUG "open input MOTO 3.5mm headset that were in phone call mode. \n");
                                                                                                                             
                                                                                                                             
        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON2_INDEX, PCAP_BIT_SET_VALUE, MB_ON2_NUM_BITS );
        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A3_EN_INDEX, PCAP_BIT_CLEAN_VALUE, A3_EN_NUM_BITS );
        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, A3_MUX_INDEX, PCAP_BIT_SET_VALUE, A3_MUX_NUM_BITS );
                                                                                                                             
}

void close_output_3mm5_with_mic_media_mode_headset(void)
{
        AUDPRINTk1(EZXOSS_DEBUG "close output MOTO 3.5mm headset that provide mic and were in media mode. \n");


        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_CLEAN_VALUE, AHS_CONFIG_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ARIGHT_EN_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ALEFT_EN_NUM_BITS );
        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON2_INDEX, PCAP_BIT_CLEAN_VALUE, MB_ON2_NUM_BITS );                                                                                                                     
}
                                                                                                                             
void close_output_3mm5_with_mic_call_mode_headset(void)
{
        AUDPRINTk1(EZXOSS_DEBUG "close output MOTO 3.5mm headset that provide mic and were in phone call mode. \n");


        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_CLEAN_VALUE, AHS_CONFIG_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ARIGHT_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ALEFT_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON2_INDEX, PCAP_BIT_CLEAN_VALUE, MB_ON2_NUM_BITS );
}
                                                                                                                             
void close_output_3mm5_without_mic_media_mode_headset(void)
{
        AUDPRINTk1(EZXOSS_DEBUG "close output standard 3.5mm headset that were in media mode. \n");
        

        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_CLEAN_VALUE, AHS_CONFIG_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ARIGHT_EN_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ALEFT_EN_NUM_BITS );
}

void close_output_3mm5_without_mic_call_mode_headset(void)
{
        AUDPRINTk1(EZXOSS_DEBUG "close output standard 3.5mm headset that were in phone call mode. \n");

                                                                                                                             
        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_CLEAN_VALUE, AHS_CONFIG_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ARIGHT_EN_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_CLEAN_VALUE, ALEFT_EN_NUM_BITS );
                                                                                                                             
}

int open_output_3mm5_with_mic_media_mode_headset(void)
{
        int ret = 0 ;
        AUDPRINTk1(EZXOSS_DEBUG "open output MOTO 3.5mm headset that provide mic and is in media mode. \n");
                                                                                                                             
                                                                                                                             
        ret = power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_SET_VALUE, AHS_CONFIG_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_SET_VALUE, ARIGHT_EN_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_SET_VALUE, ALEFT_EN_NUM_BITS );
        power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON2_INDEX, PCAP_BIT_SET_VALUE, MB_ON2_NUM_BITS );

	return ret ;
}


int open_output_3mm5_with_mic_call_mode_headset(void)
{
	int ret = 0 ;
        AUDPRINTk1(EZXOSS_DEBUG "open output MOTO 3.5mm headset that provide mic and is in phone call mode. \n");

                                                                                                                             
        ret = power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_SET_VALUE, AHS_CONFIG_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_SET_VALUE, ARIGHT_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_SET_VALUE, ALEFT_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON2_INDEX, PCAP_BIT_SET_VALUE, MB_ON2_NUM_BITS );	
                                                                                                                             
	return ret ;
}

int open_output_3mm5_without_mic_media_mode_headset(void)
{
        int ret = 0 ;
	                                                                                                         
        AUDPRINTk1(EZXOSS_DEBUG "open output standard 3.5mm headset that is in media mode. \n");
                                                                                                                             
                                                                                                                             
        ret = power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_SET_VALUE, AHS_CONFIG_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_SET_VALUE, ARIGHT_EN_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_SET_VALUE, ALEFT_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON2_INDEX, PCAP_BIT_CLEAN_VALUE, MB_ON2_NUM_BITS );
                                                                                                                             
	return ret ;
}


int open_output_3mm5_without_mic_call_mode_headset(void)
{
   	int ret = 0 ;
        AUDPRINTk1(EZXOSS_DEBUG "open output standard 3.5mm headset that is in phone call mode. \n");
                                                                                                                             
                                                                                                                             
        ret = power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AHS_CONFIG_INDEX, PCAP_BIT_SET_VALUE, AHS_CONFIG_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ARIGHT_EN_INDEX, PCAP_BIT_SET_VALUE, ARIGHT_EN_NUM_BITS );
        power_ic_set_reg_value( PCAP_RX_AUD_AMPS, ALEFT_EN_INDEX, PCAP_BIT_SET_VALUE, ALEFT_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_TX_AUD_AMPS, MB_ON2_INDEX, PCAP_BIT_CLEAN_VALUE, MB_ON2_NUM_BITS );
                                                                                                                             
	return ret ;
                                                                                                                             
}
                                                                                                                             
/* for 3MM5 headset support ==end==*/

