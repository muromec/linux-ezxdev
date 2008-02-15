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
 *
 *  History:
 *  Jin Lihong(w20076) Jan 13,2004,LIBdd68327  Created
 *  Jin Lihong(w20076) Mar.15,2004,LIBdd86574  mixer bug fix
 *  Jin Lihong(w20076) Apr.24,2004,LIBee03164  reduce music noise, add new pathes for haptics
 *  Cheng Xuefeng(a2491c) June.24,2004,LIBdd95397 Add sound path for EMU PIHF
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


int codec_output_gain = EZX_OSS_DEFAULT_OUTPUT_LOGICAL_GAIN;	/* A2,loudspeaker, gain level */
output_enum codec_output_path = LOUDERSPEAKER_OUT;
bool audio_output_mute = false;

int codec_input_gain = EZX_OSS_DEFAULT_AUDIG_LOGICAL_GAIN;	/* A5,mic,gain=+17db */
input_enum codec_input_path = HANDSET_INPUT;
bool audio_input_mute = false;

int audioonflag = 0;


void (*mixer_close_input_path[INPUT_TOTAL_TYPES])(void) =
{
    	close_input_carkit,
    	close_input_handset,
    	close_input_headset,
    	close_input_emuhs,
	close_input_3mm5_with_mic_call_mode_headset
};

void (*mixer_open_input_path[INPUT_TOTAL_TYPES])(void) =
{
    	open_input_carkit,
   	open_input_handset,
    	open_input_headset,
   	open_input_emuhs,
	open_input_3mm5_with_mic_call_mode_headset
};

void (*mixer_close_output_path[OUTPUT_TOTAL_TYPES])(void) =
{
  	close_output_pcap_headset,
    	close_output_pcap_louderspeaker,
    	close_output_pcap_earpiece,
    	close_output_pcap_carkit,
    	close_output_pcap_headjack,
    	close_output_pcap_emuhs_st_media,
    	close_output_pcap_emuhs_mono,
	close_output_3mm5_with_mic_media_mode_headset,
        close_output_3mm5_with_mic_call_mode_headset,
        close_output_3mm5_without_mic_media_mode_headset,
        close_output_3mm5_without_mic_call_mode_headset,
    	close_output_pcap_emuhs_st_call
};

int (*mixer_open_output_path[OUTPUT_TOTAL_TYPES])(void) =
{
    	open_output_pcap_headset,
    	open_output_pcap_louderspeaker,
    	open_output_pcap_earpiece,
    	open_output_pcap_carkit,
    	open_output_pcap_headjack,
    	open_output_pcap_emuhs_st_media,
    	open_output_pcap_emuhs_mono,
	open_output_3mm5_with_mic_media_mode_headset,
        open_output_3mm5_with_mic_call_mode_headset,
        open_output_3mm5_without_mic_media_mode_headset,
        open_output_3mm5_without_mic_call_mode_headset,	
    	open_output_pcap_emuhs_st_call
};


void set_audio_output(output_enum out)
{
	codec_output_path = out;

	if( audio_output_mute ){
		AUDPRINTk1(EZXOSS_DEBUG " output was muted, do not set path to %d\n", out);
		return;
	}

	if( audioonflag == 0 ){		/* for power save */
		AUDPRINTk1(EZXOSS_DEBUG " no device opened, do not set out path to %d\n", out);
		return;
	}

	(*mixer_open_output_path[out])();
	set_output_gain_hw_reg();
}


void set_audio_input(input_enum in)
{
	codec_input_path = in;

	if( audio_input_mute ){
		AUDPRINTk1(EZXOSS_DEBUG " input was muted, do not set path to %d\n", in);
		return;
	}

	if( audioonflag == 0 ){		/* for power save */
		AUDPRINTk1(EZXOSS_DEBUG " no device opened, do not set in path to %d\n", in);
		return;
	}

	(*mixer_open_input_path[in])();
}


void print_pcap_audio_reg_vals(void)
{
#ifdef EZX_OSS_DEBUG
	//unsigned long ssp_pcap_register_val;
        unsigned int ssp_pcap_register_val;
        power_ic_read_reg(PCAP_ST_DAC, &ssp_pcap_register_val);
        printk(EZXOSS_DEBUG "pcap register 13 = 0x%lx\n", ssp_pcap_register_val);
        power_ic_read_reg(PCAP_RX_AUD_AMPS, &ssp_pcap_register_val);
        printk(EZXOSS_DEBUG "pcap register 12 = 0x%lx\n", ssp_pcap_register_val);
        power_ic_read_reg(PCAP_AUD_CODEC, &ssp_pcap_register_val);
        printk(EZXOSS_DEBUG "pcap register 11 = 0x%lx\n", ssp_pcap_register_val);
        power_ic_read_reg(PCAP_TX_AUD_AMPS, &ssp_pcap_register_val);
        printk(EZXOSS_DEBUG "pcap register 26 = 0x%lx\n", ssp_pcap_register_val);
#endif
}





