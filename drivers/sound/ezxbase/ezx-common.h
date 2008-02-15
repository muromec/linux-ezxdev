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
 *  Jin Lihong(w20076) Jan 12,2004,LIBdd68327  Created
 *  Jin Lihong(w20076) Mar.15,2004,LIBdd86574  mixer bug fix
 *  Jin Lihong(w20076) Apr.24,2004,LIBee03164  reduce music noise, add new pathes for haptics
 *  Li  Yong(w19946)   Apr.22.2004.LIBee02702  Add EMU carKit
 *  Cheng Xuefeng(a2491c) Jun.15.2004,LIBdd95397 Support EMU PIHF feature
 *
 */

#ifndef EZX_COMMON_H
#define EZX_COMMON_H


#include <linux/soundcard.h>

#include "pcap_audio.h"
#include "product_dep.h"

#define	EZXOSS_EMERG	"Snd <0 EMERG>: "		/* system is unusable */
#define	EZXOSS_ALERT	"Snd <1 ALERT>: "		/* action must be taken immediately */
#define	EZXOSS_CRIT	"Snd <2 CRIT>: "		/* critical conditions */
#define	EZXOSS_ERR	"Snd <3 ERR>: "			/* error conditions */
#define	EZXOSS_WARNING	"Snd <4 WARNING>: "		/* warning conditions */
#define	EZXOSS_NOTICE	"Snd <5 NOTICE>: "		/* normal but significant condition */
#define	EZXOSS_INFO	"Snd <6 INFO>: "		/* informational */
#define	EZXOSS_DEBUG	"Snd <7 DEBUG>: "		/* debug-level messages */

#define MONODEVOPENED	( audioonflag & (CALL_DEVICE|DSP16_DEVICE|BT_DEVICE) )
#define PHONEDEVOPENED	( audioonflag & CALL_DEVICE )
#define PHONENOTOPENED	( !(PHONEDEVOPENED) )
#define STEREODEVOPENED	( audioonflag & (DSP_DEVICE|FM_DEVICE) )


#define EZX_OSS_MIN_LOGICAL_GAIN 0
#define EZX_OSS_MAX_LOGICAL_GAIN 100
#define EZX_OSS_DEFAULT_OUTPUT_LOGICAL_GAIN 74		/* +9db, same as a760 */
#define EZX_OSS_DEFAULT_AUDIG_LOGICAL_GAIN  55		/* +17db, same as a760 */
#define EZX_OSS_DEFAULT_MIC2IG_LOGICAL_GAIN 55		/* +17db, same as a760 */

#define PCAP_OUTPUT_GAIN_MIN_REG_VAL 0
#define PCAP_OUTPUT_GAIN_MAX_REG_VAL 15
#define PCAP_INPUT_GAIN_MIN_REG_VAL  0
#define PCAP_INPUT_GAIN_MAX_REG_VAL  31

#define PCAP_OUTPUT_GAIN_REG_VAL_FROM_LOGIC  (codec_output_gain*PCAP_OUTPUT_GAIN_MAX_REG_VAL/EZX_OSS_MAX_LOGICAL_GAIN)
#define PCAP_INPUT_AUDIG_REG_VAL_FROM_LOGIC  (codec_input_gain*PCAP_INPUT_GAIN_MAX_REG_VAL/EZX_OSS_MAX_LOGICAL_GAIN)
#define PCAP_INPUT_MIC2IG_REG_VAL_FROM_LOGIC (codec_input_gain*PCAP_INPUT_GAIN_MAX_REG_VAL/EZX_OSS_MAX_LOGICAL_GAIN)


#define INPUT_PATH_MIN CARKIT_INPUT
#define INPUT_PATH_MAX DIA35_HS_MIC_CALL_INPUT
#define INPUT_TOTAL_TYPES 5
#define OUTPUT_PATH_MAX STEREO_EMUHS_CALL_OUT
#define OUTPUT_TOTAL_TYPES 12

extern int codec_output_gain, codec_input_gain;
extern output_enum codec_output_path;
extern input_enum codec_input_path;
extern int audioonflag;
extern bool audio_output_mute, audio_input_mute;

extern void (*mixer_close_output_path[OUTPUT_TOTAL_TYPES])(void);
extern int  (*mixer_open_output_path[OUTPUT_TOTAL_TYPES])(void);
extern void (*mixer_close_input_path[INPUT_TOTAL_TYPES])(void);
extern void (*mixer_open_input_path[INPUT_TOTAL_TYPES])(void);
void set_audio_output(output_enum);
void set_audio_input(input_enum);

void print_pcap_audio_reg_vals(void);

#endif



