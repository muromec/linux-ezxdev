/*
 * tlv320aic23.h : Defines for Texas Instruments TLV320AIC23 CD quality codec
 *		   Used by ibmcsiti.c sound driver
 *
 *  Copyright (C) 2002, Ken Inoue and David Gibson, IBM Corporation
 */

#ifndef _TLV320AIC23_H_
#define _TLV320AIC23_H_

#define TLV320_LLI	0x00		/* Left Line In Channel Vol Ctl */
#define TLV320_RLI	0x01		/* Right Line In Channel  V.C.  */
#define TLV320_LCH	0x02		/* Left Channel Headphone V.C.  */
#define TLV320_RCH	0x03		/* Right Channel Headphone V.C. */
#define TLV320_AAP	0x04		/* Analog Audio Path Control	*/
#define TLV320_DAP	0x05		/* Digital Audio Path Control	*/
#define TLV320_PDC	0x06		/* Power Down Control		*/
#define TLV320_DAI	0x07		/* Digital Audio Interface Fmt  */
#define TLV320_SRC	0x08		/* Sample Rate Control		*/
#define TLV320_DIA	0x09		/* Digital Interface Activation */
#define TLV320_RR   	0x0F		/* Reset Register		*/

#define TLV320_REG_EXTENT	10	

/* Macros for register access */
#define TLV320_REG_WRITE(a,b) 	( ( ((u16)(a)) << 9) | \
				  ( ((u16)(b)) & 0x01ff ) )

/* TLV320AIC23 register bit fields (short format) */
#define TLV320_LLI_LRS	0x0100		/* L/R line simul. update (1=on)*/
#define TLV320_LLI_LIM 	0x0080		/* Left line input mute (1=mute)*/
#define TLV320_LLI_LIV	0x001F		/* Mask for left line in volume */
#define TLV320_LLI_LIV_POR 0x17		/* POR default (0dB)		*/

#define TLV320_RLI_RLS	0x0100		/* R/L line simul. update (1=on)*/
#define TLV320_RLI_RIM 	0x0080		/* Right line in mute (1=mute)  */
#define TLV320_RLI_RIV	0x001F		/* Mask for right line in vol   */
#define TLV320_RLI_RIV_POR 0x17		/* POR default (0dB)		*/

#define TLV320_LCH_LRS	0x0100		/* L/R headphone simul. update  */
#define TLV320_LCH_LZC	0x0080		/* Zero cross detect (1=on)	*/
#define TLV320_LCH_LHV	0x007F		/* Mask for left headphone vol  */
#define TLV320_LCH_LHV_POR 0x79		/* POR default (0dB)		*/

#define TLV320_RCH_RLS	0x0100		/* L/R headphone simul. update  */
#define TLV320_RCH_RZC	0x0080		/* Zero cross detect (1=on)	*/
#define TLV320_RCH_RHV	0x007F		/* Mask for left headphone vol  */
#define TLV320_RCH_RHV_POR 0x79		/* POR default (0dB)		*/

#define TLV320_AAP_STA	0x00C0		/* Mask for sidetone attenuation*/
#define TLV320_AAP_STE	0x0020		/* Sidetone enable (1=on)	*/
#define TLV320_AAP_DAC 	0x0010		/* DAC off/on			*/
#define	TLV320_AAP_BYP	0x0008		/* Bypass 			*/
#define TLV320_AAP_INSEL  0x0004	/* Input select for ADC		*/
					/* 	0 = Line, 1 = Mic	*/
#define TLV320_AAP_MICM	0x0002		/* Mic mute (1=mute)		*/
#define TLV320_AAP_MICB	0x0001		/* Mic boost (1=20dB)		*/ 

#define TLV320_DAP_DACM	0x0008		/* DAC mute (1=mute)		*/
#define	TLV320_DAP_DEEMP 0x0006		/* Deemphasis 00 disabled,	*/
					/* 01 32K, 10 44.1K, 11 48KHz	*/
#define TLV320_DAP_ADCHP 0x0001		/* ADC high-pass filter (1=on)	*/

#define TLV320_PDC_OFF	0x0080		/* Device power (0=ON, 1=OFF)	*/
#define TLV320_PDC_CLK	0x0040		/* Clock power			*/
#define TLV320_PDC_OSC	0x0020		/* Osc power			*/
#define TLV320_PDC_OUT	0x0010		/* Outputs power 		*/
#define TLV320_PDC_DAC	0x0008		/* DAC power			*/
#define TLV320_PDC_ADC	0x0004		/* ADC power			*/
#define TLV320_PDC_MIC	0x0002		/* Mic power			*/
#define TLV320_PDC_LINE	0x0001		/* Line in power		*/

#define TLV320_DAI_MS	0x0040		/* Master (1) / slave (0) mode	*/
#define TLV320_DAI_LRSWAP 0x0020	/* Left/Right swap (1=swap)	*/
#define TLV320_DAI_LRP	0x0010		/* Signal phase			*/
#define	TLV320_DAI_IWL	0x000C		/* Mask for input bit length    */
					/* (00 = 16 bit)		*/
#define TLV320_DAI_FOR	0x0003		/* Data formats			*/

#define TLV320_SRC_CLKOUT 0x0080	/* Clock output divider		*/
#define TLV320_SRC_CLKIN  0x0040	/* Clock input divider		*/
#define TLV320_SRC_SR	0x003C		/* Sampling rate control mask	*/
#define TLV320_SRC_BOSR	0x0002		/* Base oversampling rate	*/
					/*   USB mode: 0 = 250, 1=272fs */
					/* Normal mode: 0 = 256, 1=384fs*/
#define TLV320_SRC_USB	0x0001		/* USB mode (1) / Normal (0)	*/

#define TLV320_DIA_ACT	0x0001		/* Activate Interface (1=active)*/

#endif /* _TLV320AIC23_H_ */
