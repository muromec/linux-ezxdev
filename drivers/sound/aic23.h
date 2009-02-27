/*
 * File: aic23.h
 *
 * Defines for TI TLV320AIC23 CODEC
 *
 * Copyright (C) 2002 RidgeRun, Inc.
 * Author: Steve Johnson
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Please report all bugs/problems to the author or <support@dsplinux.net>
 *
 * key: RRGPLCR (do not remove)
 *
 */
#ifndef aic23_h
#define aic23_h

// Codec TLV320AIC23 
#define LEFT_LINE_VOLUME_ADDR		0x00 
#define RIGHT_LINE_VOLUME_ADDR		0x01
#define LEFT_CHANNEL_VOLUME_ADDR	0x02
#define RIGHT_CHANNEL_VOLUME_ADDR	0x03
#define ANALOG_AUDIO_CONTROL_ADDR	0x04
#define DIGITAL_AUDIO_CONTROL_ADDR	0x05
#define POWER_DOWN_CONTROL_ADDR		0x06
#define DIGITAL_AUDIO_FORMAT_ADDR	0x07
#define SAMPLE_RATE_CONTROL_ADDR	0x08
#define DIGITAL_INTERFACE_ACT_ADDR	0x09
#define RESET_CONTROL_ADDR		0x0F 

// Left line input volume control register
#define LRS_ENABLED			0x0100
#define LIM_MUTED			0x0080
#define LIV_CONTROL			0x001F
#define LIV_DEFAULT			0x0017
#define LIV_MAX				0x001f

// Left channel headphone volume control register
#define LZC_ON				0x0080
#define LHV_DEFAULT			0x0067
#define LHV_MAX				0x007f

// Analog audio path control register
#define STE_ENABLED			0x0020
#define DAC_SELECTED			0x0010
#define BYPASS_ON			0x0008
#define INSEL_MIC			0x0004
#define MICM_MUTED			0x0002
#define MICB_20DB			0x0001

// Digital audio path control register
#define DACM_MUTE			0x0008
#define DEEMP_32K			0x0002
#define DEEMP_44K			0x0004
#define DEEMP_48K			0x0006
#define ADCHP_ON			0x0001

// Power control down register 
#define DEVICE_POWER_OFF	  	0x0080
#define CLK_OFF				0x0040
#define OSC_OFF				0x0020
#define OUT_OFF				0x0010
#define DAC_OFF				0x0008
#define ADC_OFF				0x0004
#define MIC_OFF				0x0002
#define LINE_OFF			0x0001

// Digital audio interface register
#define MS_MASTER			0x0040
#define LRSWAP_ON			0x0020
#define LRP_ON				0x0010
#define IWL_16				0x0000
#define IWL_20				0x0004
#define IWL_24				0x0008
#define IWL_32				0x000C
#define FOR_I2S				0x0002
#define FOR_DSP				0x0003

// Sample rate control register
#define CLKOUT_HALF			0x0080
#define CLKIN_HALF			0x0040
#define BOSR_384fs			0x0002 //BOSR_272fs when in USB mode
#define USB_CLK_ON			0x0001
#define SR_MASK                         0xf
#define SR_SHIFT                        2
#define BOSR_SHIFT                      1

	// Digital interface register
#define ACT_ON				0x0001
      
//#define DEV_AIC23_ID                    (0x1A)    // 0x1b cs high  0x1A cs low
#define DEV_AIC23_ID                    (0x1A >> 1)    // 0x1b cs high  0x1A cs low

#endif
