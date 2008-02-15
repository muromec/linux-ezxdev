/*
 * Copyright (C) 2005 Motorola Inc.
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
 */
/*================================================================================
                                                                                                                             
Module Name:  pcap_audio.c
                                                                                                                             
General Description: Define some interfaces for audio driver using the PCAP audio functions.
                                                                                                                             
Revision History:
                            Modification     Tracking
Author (core ID)                Date          Number      Description of Changes
------------------------    ------------    ----------   -------------------------
Lv Yunguang (a6511c)        05/13/2005      LIBXXXXXXX     Created
                                                                                                                             
Portability: Depend on PCAP of Power-IC hardware and the power_ic.h, only for EZX platform.
                                                                                                                             
==================================================================================*/
/*================================================================================
                                 INCLUDE FILES
================================================================================*/
#include <linux/errno.h>
#include <linux/delay.h>

#include  "pcap_audio.h"
                                                                                                                             
/*================================================================================
                                LOCAL CONSTANTS
================================================================================*/
                                                                                                                             
                                                                                                                             
/*================================================================================
                   LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
================================================================================*/
                                                                                                                             
                                                                                                                             
/*================================================================================
                                  LOCAL MACROS
================================================================================*/
                                                                                                                             

                                                                                                                             
/*================================================================================
                            LOCAL FUNCTION PROTOTYPES
================================================================================*/
                                                                                                                             
                                                                                                                            
/*================================================================================
                                LOCAL VARIABLES
================================================================================*/
                                                                                                                             
                                                                                                                             
/*================================================================================
                                GLOBAL VARIABLES
================================================================================*/
                                                                                                                       
                                                                                                                             
/*================================================================================
                            LOCAL POINTER DECLARATIONS
================================================================================*/
                                                                                                                             
                                                                                                                             
/*================================================================================
                                LOCAL FUNCTIONS
================================================================================*/
                                                                                 
                                                                                                                             

                                                                                                                             
/*================================================================================
                               GLOBAL FUNCTIONS
================================================================================*/
                                                                                                                           
/*================================================================================
FUNCTION: PCAP_MONO_set
DESCRIPTION: Title
   Set the MONO type of RX AMP in PCAP.
ARGUMENTS PASSED:
   momoType
RETURN VALUE:
   int
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int PCAP_MONO_set(PCAP_MONO_TYPE monoType)
{
 	return power_ic_set_reg_value(PCAP_RX_AUD_AMPS, MONO_INDEX, monoType, MONO_NUM_BITS); 
}
                                                                                                         
/*================================================================================
FUNCTION: PCAP_AUDOG_set
DESCRIPTION: Title
   Set the gain value of ouput from RX AMP in PCAP.
ARGUMENTS PASSED:
    audioOutGain
RETURN VALUE:
   int
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int PCAP_AUDOG_set( int audioOutGain )
{
	if ( audioOutGain > PCAP_AUDOG_MAX_VALUE ){
	   return - EINVAL ;   
	}
    
   	return power_ic_set_reg_value( PCAP_RX_AUD_AMPS, AUDOG_INDEX, audioOutGain,  AUDOG_NUM_BITS );
}
                                                                                                        
/*================================================================================
FUNCTION: PCAP_AUDIG_set
DESCRIPTION: Title
   Set the gain value of input to TX AMP in PCAP.
ARGUMENTS PASSED:
    audioOutGain
RETURN VALUE:
   int
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int PCAP_AUDIG_set( int audioInGain )
{
        if ( audioInGain > PCAP_AUDIG_MAX_VALUE ){
           return - EINVAL ;
        }
                                                                                                                             
        return power_ic_set_reg_value( PCAP_TX_AUD_AMPS, AUDIG_INDEX, audioInGain,  AUDIG_NUM_BITS );
}

/*================================================================================
FUNCTION: PCAP_CDC_SR_set
DESCRIPTION: Title
   Set the sample rate of ST DAC in PCAP.
ARGUMENTS PASSED:
    PCAP_ST_SAMPLE_RATE_TYPE srType
RETURN VALUE:
   int
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
int PCAP_CDC_SR_set(PCAP_ST_SAMPLE_RATE_TYPE srType)
{
        return power_ic_set_reg_value( PCAP_ST_DAC, ST_DAC_SR_INDEX, srType,  ST_DAC_SR_NUM_BITS );
}

/*================================================================================
FUNCTION:t PCAP_vibrate_start_command
DESCRIPTION: Title
   Set the vibrator to start vibrate.
ARGUMENTS PASSED:
    void
RETURN VALUE:
   void
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
void PCAP_vibrate_start_command(void)
{
    power_ic_set_reg_value( PCAP_AUX_VREG, PCAP_VIBRATOR_ONOFF_INDEX, PCAP_BIT_SET_VALUE,  PCAP_VIBRATOR_ONOFF_NUM_BITS );
}

                       
/*================================================================================
FUNCTION:t PCAP_vibrate_stop_command
DESCRIPTION: Title
   Set the vibrator to stop vibrate.
ARGUMENTS PASSED:
    void
RETURN VALUE:
   void
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
void PCAP_vibrate_stop_command(void)
{
    power_ic_set_reg_value( PCAP_AUX_VREG, PCAP_VIBRATOR_ONOFF_INDEX, PCAP_BIT_CLEAN_VALUE,  PCAP_VIBRATOR_ONOFF_NUM_BITS );
}


/*================================================================================
FUNCTION:t PCAP_V_VIB_Level_set
DESCRIPTION: Title
   Set the vibrate level of vibrator in PCAP.
ARGUMENTS PASSED:
    PCAP_VibratorLevel_TYPE  VIBLevel
RETURN VALUE:
    int
PRE-CONDITIONS:
   None
POST-CONDITIONS:
   None
IMPORTANT NOTES:
   None
================================================================================*/
void PCAP_V_VIB_level_set(PCAP_VibratorLevel_TYPE VIBLevel)
{
      power_ic_set_reg_value( PCAP_AUX_VREG, PCAP_VIBRATOR_LEVEL_INDEX, PCAP_BIT_CLEAN_VALUE, PCAP_VIBRATOR_LEVEL_NUM_BITS );
}



void set_pcap_telephone_codec(void)
{
	unsigned long ssp_pcap_register_val;

	ssp_pcap_register_val = CDC_CLK_IN_13M0;
	power_ic_set_reg_value( PCAP_AUD_CODEC, PCAP_AUD_CODEC_INDEX, PCAP_BIT_CLEAN_VALUE, PCAP_AUD_CODEC_NUM_BITS );
        power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_CLK_INDEX, CDC_CLK_IN_13M0, CDC_CLK_NUM_BITS );

	power_ic_set_reg_value( PCAP_AUD_CODEC, SMB_INDEX, PCAP_BIT_CLEAN_VALUE, SMB_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, FS_8K_16K_INDEX, PCAP_BIT_CLEAN_VALUE, FS_8K_16K_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, DIG_AUD_IN_INDEX, PCAP_BIT_CLEAN_VALUE, DIG_AUD_IN_NUM_BITS );

	power_ic_set_reg_value( PCAP_AUD_CODEC, AUDIHPF_INDEX, PCAP_BIT_SET_VALUE, AUDIHPF_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, AUDOHPF_INDEX, PCAP_BIT_SET_VALUE, AUDOHPF_NUM_BITS );

	power_ic_set_reg_value( PCAP_AUD_CODEC, CLK_INV_INDEX, PCAP_BIT_CLEAN_VALUE, CLK_INV_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, FS_INV_INDEX, PCAP_BIT_CLEAN_VALUE, FS_INV_NUM_BITS );

	/*(3) reset digital filter(DF_RESET=1) */
	power_ic_set_reg_value( PCAP_AUD_CODEC, DF_RESET_INDEX, PCAP_BIT_SET_VALUE, DF_RESET_NUM_BITS );

	power_ic_set_reg_value( PCAP_AUD_CODEC, ADITH_INDEX, PCAP_BIT_CLEAN_VALUE, ADITH_NUM_BITS );
	/* (4) enable pcap clk(CDC_CLK_EN=1),enable CODEC(CDC_EN=1)   */
	power_ic_set_reg_value( PCAP_RX_AUD_AMPS, CD_BYP_INDEX, PCAP_BIT_CLEAN_VALUE, CD_BYP_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_CLK_EN_INDEX, PCAP_BIT_SET_VALUE, CDC_CLK_EN_NUM_BITS );
	power_ic_set_reg_value( PCAP_AUD_CODEC, CDC_EN_INDEX, PCAP_BIT_SET_VALUE, CDC_EN_NUM_BITS );
	mdelay(1);	/* specified enable time */
}




