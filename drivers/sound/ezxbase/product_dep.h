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
 *  Jin Lihong(w20076) Apr.13,2004,LIBdd96876  close dsp protection,and add 3d control interface for app
 *  Jin Lihong(w20076) Jun.22,2004,LIBee24284  mixer power save
 *  Cheng Xuefeng(a2491c) Jun.24,2004,LIBdd95397  Add EMU PIHF carkit sound path
 *
 */

#ifndef EZX_BARBADOS_H
#define EZX_BARBADOS_H

#include <linux/config.h>
#include <linux/pm.h>

#ifdef CONFIG_MOT_POWER_IC_BARREL_HEADSET_STEREO_3MM5
  #include <linux/moto_accy.h>
#else
  #define moto_3mm5_multi_mode_gpio()  {}
  #define moto_3mm5_phone_mode_gpio()  {}
  #define std_3mm5_multi_mode_gpio()   {}
  #define std_3mm5_phone_mode_gpio()   {}
  #define headset_3mm5_power_saving()  {}
#endif

//#define EZX_OSS_DEBUG          /* debug switch for all ezx oss src files excluding ezx-audio.c */
//#define EZX_OSS_AUDIO_DEBUG    /* debug switch for src file ezx-audio.c */

#ifdef EZX_OSS_DEBUG
#define AUDPRINTk1(fmt,args...) printk(fmt,##args) 
#else
#define AUDPRINTk1(fmt,args...) { }
#endif

#ifdef EZX_OSS_AUDIO_DEBUG
#define AUDPRINTk2(fmt,args...) printk(fmt,##args) 
#else 
#define AUDPRINTk2(fmt,args...) { }
#endif 



typedef enum{
	CALL_DEVICE	= 0x01,
	DSP_DEVICE	= 0x02,
	DSP16_DEVICE	= 0x04,
	BT_DEVICE	= 0x08,
	FM_DEVICE	= 0x10
}audio_dev_type;

typedef enum{
	HW_ATTENUATION_USED,
	HW_ATTENUATION_BYPASSED
}hw_noise_attenuation;


void close_input_carkit(void);
void close_input_handset(void);
void close_input_headset(void);
void close_input_emuhs(void);

void open_input_carkit(void);
void open_input_handset(void);
void open_input_headset(void);
void open_input_emuhs(void);

void close_output_pcap_headset(void);
void close_output_pcap_louderspeaker(void);
void close_output_pcap_earpiece(void);
void close_output_pcap_carkit(void);
void close_output_pcap_headjack(void);
void close_output_pcap_emuhs_st_media(void);
void close_output_pcap_emuhs_st_call(void);
void close_output_pcap_emuhs_mono(void);

int open_output_pcap_headset(void);
int open_output_pcap_louderspeaker(void);
int open_output_pcap_earpiece(void);
int open_output_pcap_carkit(void);
int open_output_pcap_headjack(void);
int open_output_pcap_emuhs_st_media(void);
int open_output_pcap_emuhs_st_call(void);
int open_output_pcap_emuhs_mono(void);

void set_output_gain_hw_reg(void);
void set_input_gain_hw_reg(void);

void poweron_mixer( audio_dev_type type );
void shutdown_mixer( audio_dev_type type );

void use_hw_noise_attenuate(void);
void bypass_hw_noise_attenuate(void);

void pcap_use_ap_13m_clock(void);
void pcap_use_bp_13m_clock(void);

void pcap_audio_init(void);
int mixer_hw_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data);

/* for 3MM5 headset support ==begin==*/
void  close_input_3mm5_with_mic_call_mode_headset(void);
void   open_input_3mm5_with_mic_call_mode_headset(void);
                                                                                                                             
void close_output_3mm5_with_mic_media_mode_headset(void);
void close_output_3mm5_with_mic_call_mode_headset(void);
void close_output_3mm5_without_mic_media_mode_headset(void);
void close_output_3mm5_without_mic_call_mode_headset(void);
                                                                                                                             
int open_output_3mm5_with_mic_media_mode_headset(void);
int open_output_3mm5_with_mic_call_mode_headset(void);
int open_output_3mm5_without_mic_media_mode_headset(void);
int open_output_3mm5_without_mic_call_mode_headset(void);
                                                                                                                             
/* for 3MM5 headset support ==end==*/


#endif


