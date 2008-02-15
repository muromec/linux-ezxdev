/*
 *      ibmcsi3k.h : IBM PowerPC 405LP Codec Serial Interface (CSI) +
 *                      Si3000 voiceband codec driver
 *                      for the 405LP evaluation board
 *
 *	Based on various sound drivers in linux/drivers/sound, including but not limited to
 *	es1370.c and vwsnd.c, as well as an unfinished driver from IBM Austin Research Lab.
 *
 *      Copyright (C) 2002  IBM Corp.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __SI3000_INC__
#define __SI3000_INC__

#define SI3000_SW_FC	0x00010000	/* Request bit for secondary FSYNC */

#define SI3000_CR1	1		/* Control register 1		*/
#define SI3000_CR2	2		/* Control register 2		*/
#define SI3000_PLL1D	3		/* PLL1 Divider N1 (minus 1)	*/
#define SI3000_PLL1M	4		/* PLL1 Multiplier M1 (minus 1)	*/
#define SI3000_RXGC1	5		/* Rx Gain Control 1		*/
#define SI3000_ADC_VC	6		/* ADC Volume Control		*/
#define SI3000_DAC_VC	7		/* DAC Volume Control		*/
#define SI3000_SR	8		/* Status Report		*/
#define SI3000_AA	9		/* Analog Attenuation		*/

/* Macros for register access */
#define SI3000_REG_WRITE(a,b) 	( ( ((unsigned int)(a)) << 24) | \
				  ( ((unsigned int)(b)) << 16 ) )
#define SI3000_REG_READ(a)	( ( ((unsigned int)(a)) << 24 ) | 0x20000000 )

/* Si3000 register bit fields (byte format) */
#define SI3000_CR1_SR 	0x80 		/* Software Reset		*/
#define SI3000_CR1_SPD	0x10		/* Speaker Drive Power 	(1=on)	*/
#define SI3000_CR1_LPD	0x08		/* Line Driver Power 	(1=on)	*/
#define SI3000_CR1_HPD	0x04		/* Handset Drive Power 	(1=on)	*/
#define SI3000_CR1_MPD	0x02		/* Mic Bias Power 	(1=OFF)	*/
#define SI3000_CR1_CPD	0x01		/* Chip power down 	(1=OFF)	*/

#define SI3000_CR2_HPD	0x10		/* High Pass Filter (1=DISABLE)	*/
#define SI3000_CR2_PLL	0x08		/* PLL divide by 10 (1) / 5 (0)	*/
#define SI3000_CR2_DL1	0x04		/* Digital loopback 1		*/
#define SI3000_CR2_DL2	0x02		/* Digital loopback 2		*/
#define SI3000_CR2_AL	0x01		/* Analog loopback		*/

#define SI3000_RXGC1_LIG	0xC0	/* Line In Gain mask		*/
#define SI3000_RXGC1_LIG_OFFSET	6	

#define SI3000_RXGC1_LIG_20DB	0xC0	/* Line In Gain 20dB		*/
#define SI3000_RXGC1_LIG_10DB	0x80	/* Line In Gain 10dB		*/
#define SI3000_RXGC1_LIG_0DB	0x40	/* Line In Gain 0dB		*/
#define SI3000_RXGC1_LIM	0x20	/* Line In Mute (1=mute)	*/
#define SI3000_RXGC1_MIG	0x18	/* Mic In Gain mask		*/
#define SI3000_RXGC1_MIG_OFFSET 3
#define SI3000_RXGC1_MIG_30DB	0x18	/* Mic In Gain 30dB		*/
#define SI3000_RXGC1_MIG_20DB	0x10	/* Mic In Gain 20dB		*/
#define SI3000_RXGC1_MIG_10DB	0x08	/* Mic In Gain 10dB		*/
#define SI3000_RXGC1_MIG_0DB	0x00	/* Mic In Gain 0dB		*/
#define SI3000_RXGC1_MIM	0x04	/* Mic In Mute 	(1=mute)	*/
#define SI3000_RXGC1_HIM	0x02	/* Handset Input Mute (1=mute)	*/
#define SI3000_RXGC1_IIR	0x01	/* Handset Input Mute (1=mute)	*/

#define SI3000_ADC_VC_RXG	0x7C	/* RX PGA Gain Control		*/
#define SI3000_ADC_VC_RXG_OFFSET 2
#define SI3000_ADC_VC_RXGC(N) 	(N << SI3000_ADC_VC_RXG_OFFSET)
#define SI3000_ADC_VC_LOM	0x02	/* Line Out Mute (0=mute)	*/
#define SI3000_ADC_VC_LOM_OFFSET 1
#define SI3000_ADC_VC_HOM	0x01	/* Handset Out Mute (0=mute)	*/

#define SI3000_DAC_VC_TXG	0x7C	/* TX PGA Gain Control		*/
#define SI3000_DAC_VC_TXG_OFFSET 2
#define SI3000_DAC_VC_TXGC(N) 	(N << SI3000_DAC_VC_TXG_OFFSET)
#define SI3000_DAC_VC_SLM	0x02	/* SPKR_L Mute	(0=mute)	*/
#define SI3000_DAC_VC_SRM	0x01	/* SPKR_R Mute	(0=mute)	*/

#define SI3000_SR_SLSC		0x80	/* SPK_L short circuit	(1=short) */
#define SI3000_SR_SRSC		0x40	/* SPK_R short circuit	(1=short) */
#define SI3000_SR_LOSC		0x20	/* Line out short circuit	*/

#define SI3000_AA_LOT		0x0C	/* Line Out Attenuation		*/
#define SI3000_AA_LOT_OFFSET	2	

#define SI3000_AA_SOT		0x03	/* Speaker Out Attenuation	*/

#endif
