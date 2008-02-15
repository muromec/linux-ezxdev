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

#ifndef PCAP_AUDIO_H
#define PCAP_AUDIO_H
/*================================================================================
                                                                                                                             
                      Header Name: pcap_audio.h
                                                                                                                             
General Description: Header file for PCAP registers definition which used in audio driver.

Revision History:
                            Modification     Tracking
Author(core ID)                 Date          Number      Description of Changes
------------------------    ------------    ----------   -----------------------------------
Lv Yunguang(a6511c)         May 10,2005     LIBXXXXXXX   Created
                                                                                                                             
==================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif
                                                                                                                             
/*===============================================================================
                                 INCLUDE FILES
================================================================================*/
#include <linux/power_ic.h>
//#include "../../misc/ssp_pcap.h"                                                                                                                             
/*================================================================================
                                   CONSTANTS
================================================================================*/

/* Define the PCAP registers for audio function in Power_IC */
#define PCAP_RX_AUD_AMPS     POWER_IC_REG_PCAP_RX_AUD_AMPS
#define PCAP_TX_AUD_AMPS     POWER_IC_REG_PCAP_TX_AUD_AMPS
#define PCAP_AUD_CODEC       POWER_IC_REG_PCAP_AUD_CODEC
#define PCAP_ST_DAC          POWER_IC_REG_PCAP_ST_DAC
#define PCAP_AUX_VREG        POWER_IC_REG_PCAP_AUX_VREG

/*RX_AUD_AMPS register definition*/
#define  PCAP_RX_AUD_AMPS_INDEX    0
#define  PCAP_RX_AUD_AMPS_NUM_BITS     24
#define             A1_EN_INDEX    0
#define             A1_EN_NUM_BITS     1
#define             A2_EN_INDEX    1
#define             A2_EN_NUM_BITS     1
#define       PGA_OUTL_EN_INDEX    2
#define       PGA_OUTL_EN_NUM_BITS     1
#define       A2_ADJ_LOCK_INDEX    3
#define       A2_ADJ_LOCK_NUM_BITS     1
#define             A4_EN_INDEX    4
#define             A4_EN_NUM_BITS     1
#define         ARIGHT_EN_INDEX    5
#define         ARIGHT_EN_NUM_BITS     1
#define          ALEFT_EN_INDEX    6
#define          ALEFT_EN_NUM_BITS     1
#define            CD_BYP_INDEX    7
#define            CD_BYP_NUM_BITS     1
#define            CDC_SW_INDEX    8
#define            CDC_SW_NUM_BITS     1
#define         ST_DAC_SW_INDEX    9
#define         ST_DAC_SW_NUM_BITS     1
#define         PGA_IN_SW_INDEX    10
#define         PGA_IN_SW_NUM_BITS     1
#define          PGA_R_EN_INDEX    11
#define          PGA_R_EN_NUM_BITS     1
#define          PGA_L_EN_INDEX    12
#define          PGA_L_EN_NUM_BITS     1
#define             AUDOG_INDEX    13
#define             AUDOG_NUM_BITS     4
#define            A1CTRL_INDEX    17
#define            A1CTRL_NUM_BITS     1
#define           A1ID_RX_INDEX    18
#define           A1ID_RX_NUM_BITS     1
#define              MONO_INDEX    19
#define              MONO_NUM_BITS     2
#define     AUDOG_PRI_ADJ_INDEX    21
#define     AUDOG_PRI_ADJ_NUM_BITS     1
#define      MONO_PRI_ADJ_INDEX    22
#define      MONO_PRI_ADJ_NUM_BITS     1
#define        RX_PRI_ADJ_INDEX    23
#define        RX_PRI_ADJ_NUM_BITS     2
                                                                                                                            
/*TX_AUD_AMPS register definition*/
#define  PCAP_TX_AUD_AMPS_INDEX    0
#define  PCAP_TX_AUD_AMPS_NUM_BITS     24
#define             AUDIG_INDEX    0
#define             AUDIG_NUM_BITS     5
#define             A3_EN_INDEX    5
#define             A3_EN_NUM_BITS     1
#define            A3_MUX_INDEX    6
#define            A3_MUX_NUM_BITS     1
#define             A5_EN_INDEX    7
#define             A5_EN_NUM_BITS     1
#define            A5_MUX_INDEX    8
#define            A5_MUX_NUM_BITS     1
#define       EXT_MIC_MUX_INDEX    9
#define       EXT_MIC_MUX_NUM_BITS     1
#define            MB_ON2_INDEX    10
#define            MB_ON2_NUM_BITS     1
#define            MB_ON1_INDEX    11
#define            MB_ON1_NUM_BITS     1
#define           A1ID_TX_INDEX    12
#define           A1ID_TX_NUM_BITS     1
#define         A1_CONFIG_INDEX    13
#define         A1_CONFIG_NUM_BITS     1
#define        AHS_CONFIG_INDEX    14
#define        AHS_CONFIG_NUM_BITS     1
#define         A2_CONFIG_INDEX    15
#define         A2_CONFIG_NUM_BITS     1
#define     AUDIO_LOW_PWR_INDEX    19
#define     AUDIO_LOW_PWR_NUM_BITS     1
#define           V2_EN_2_INDEX    21
#define           V2_EN_2_NUM_BITS     1
#define     AUDIG_PRI_ADJ_INDEX    22
#define     AUDIG_PRI_ADJ_NUM_BITS     1
#define        TX_PRI_ADJ_INDEX    23
#define        TX_PRI_ADJ_NUM_BITS     2


/*Register AUD_CODEC definition*/
#define    PCAP_AUD_CODEC_INDEX    0
#define    PCAP_AUD_CODEC_NUM_BITS     24
#define           AUDIHPF_INDEX    0
#define           AUDIHPF_NUM_BITS     1
#define               SMB_INDEX    1
#define               SMB_NUM_BITS     1
#define           AUDOHPF_INDEX    2
#define           AUDOHPF_NUM_BITS     1
#define             CD_TS_INDEX    3
#define             CD_TS_NUM_BITS     1
#define               DLM_INDEX    4
#define               DLM_NUM_BITS     1
#define            ADITH_INDEX    5
#define            ADITH_NUM_BITS     1
#define           CDC_CLK_INDEX    6
#define           CDC_CLK_NUM_BITS     3
#define           CLK_INV_INDEX    9
#define           CLK_INV_NUM_BITS     1
#define            FS_INV_INDEX    10
#define            FS_INV_NUM_BITS     1
#define          DF_RESET_INDEX    11
#define          DF_RESET_NUM_BITS     1
#define            CDC_EN_INDEX    12
#define            CDC_EN_NUM_BITS     1
#define        CDC_CLK_EN_INDEX    13
#define        CDC_CLK_EN_NUM_BITS     1
#define         FS_8K_16K_INDEX    14
#define         FS_8K_16K_NUM_BITS     1
#define        DIG_AUD_IN_INDEX    15
#define        DIG_AUD_IN_NUM_BITS     1
#define        CLK_IN_SEL_INDEX    16
#define        CLK_IN_SEL_NUM_BITS     1
#define          MIC2_MUX_INDEX    17
#define          MIC2_MUX_NUM_BITS     1
#define            MIC2IG_INDEX    18
#define            MIC2IG_NUM_BITS     5
#define    MIC2IG_PRI_ADJ_INDEX    23
#define    MIC2IG_PRI_ADJ_NUM_BITS     1
#define       CDC_PRI_ADJ_INDEX    24
#define       CDC_PRI_ADJ_NUM_BITS     1


/*ST_DAC register definition*/
#define       PCAP_ST_DAC_INDEX    0
#define       PCAP_ST_DAC_NUM_BITS     24
#define        SMB_ST_DAC_INDEX    0
#define        SMB_ST_DAC_NUM_BITS     1
#define          STDET_EN_INDEX    1
#define          STDET_EN_NUM_BITS     1
#define            ST_CLK_INDEX    2
#define            ST_CLK_NUM_BITS     3
#define         ST_CLK_EN_INDEX    5
#define         ST_CLK_EN_NUM_BITS     1
#define   DF_RESET_ST_DAC_INDEX    6
#define   DF_RESET_ST_DAC_NUM_BITS     1
#define         ST_DAC_EN_INDEX    7
#define         ST_DAC_EN_NUM_BITS     1
#define         ST_DAC_SR_INDEX    8
#define         ST_DAC_SR_NUM_BITS     4
#define DIG_AUD_IN_ST_DAC_INDEX    12
#define DIG_AUD_IN_ST_DAC_NUM_BITS     1 
#define        DIG_AUD_FS_INDEX    13
#define        DIG_AUD_FS_NUM_BITS     2
#define              BCLK_INDEX    15
#define              BCLK_NUM_BITS     2
#define        ST_CLK_INV_INDEX    17
#define        ST_CLK_INV_NUM_BITS     1
#define         ST_FS_INV_INDEX    18
#define         ST_FS_INV_NUM_BITS     1
#define ST_DAC_CLK_IN_SEL_INDEX    19
#define ST_DAC_CLK_IN_SEL_NUM_BITS     1
#define    ST_DAC_PRI_ADJ_INDEX    24
#define    ST_DAC_PRI_ADJ_NUM_BITS     1

/* vibrator register AUX_VREG definition */
#define PCAP_VIBRATOR_ONOFF_INDEX  19
#define PCAP_VIBRATOR_ONOFF_NUM_BITS   1
#define PCAP_VIBRATOR_LEVEL_INDEX  20
#define PCAP_VIBRATOR_LEVEL_NUM_BITS   2

/* definition of value to bit clean and bit set  */
#define    PCAP_BIT_SET_VALUE      1 
#define    PCAP_BIT_CLEAN_VALUE    0

/* Max values of the gain of RX and TX of PCAP*/
#define PCAP_AUDIG_MAX_VALUE       31
#define PCAP_AUDOG_MAX_VALUE       15


/*================================================================================
                           GLOBAL MACROS DECLARATION
================================================================================*/
                                                                                                                             

                                                                                                                             
/*================================================================================
                                     ENUMS
================================================================================*/
/* The bits 19 and 20 of RX_AUD_AMPS register in PCAP indicate the MONO type
   ___________________________________________________________________________________________
  |    definitions     |MONO_1(bit:20)|MONO_0(bit:19)|      Output Selected                   |
  |--------------------+--------------+--------------+----------------------------------------|
  |MONO_PGA_R_L_STEREO |      0       |      0       |Right PGA and Left PGA Separated(Stereo)|     
  |MONO_PGA_RL         |      0       |      1       |Right PGA + Left PGA(MONO)              |
  |MONO_PGA_RL_3DB     |      1       |      0       |(Right PGA + Left PGA)-3dB              |
  |MONO_PGA_RL_6DB     |      1       |      1       |(Right PGA + Left PGA)-6dB              |
  +-------------------------------------------------------------------------------------------+
*/
typedef enum pcapMono_Type
{
        MONO_PGA_R_L_STEREO             = 0,
        MONO_PGA_RL                     = 1,
        MONO_PGA_RL_3DB                 = 2,
        MONO_PGA_RL_6DB                 = 3
}PCAP_MONO_TYPE;

/* The bits 15 and 16 of ST_DAC register in PCAP indicate the ST_DAC Bit-CLK
   ___________________________________________________________________________________
  |    definitions |BCLK1(bit:16)|BCLK0(bit:15)|BitCLK Output(Master Mode|Slave Mode) |
  |----------------+-------------+-------------+--------------------------------------|
  |ST_BCLK_SLOT_16 |      0      |      0      |          MCLK | BitCLK               |       
  |ST_BCLK_SLOT_8  |      0      |      1      |        MCLK/2 | 2*BitCLK             |
  |ST_BCLK_SLOT_4  |      1      |      0      |        MCLK/4 | 4*BitCLK             |
  |ST_BCLK_SLOT_2  |      1      |      1      |        MCLK/8 | 8*BitCL              |
  +-----------------------------------------------------------------------------------+
*/
typedef enum pcapST_BCLK_Type
{
        ST_BCLK_SLOT_16                  = 0,
        ST_BCLK_SLOT_8                   = 1,
        ST_BCLK_SLOT_4                   = 2,
        ST_BCLK_SLOT_2                   = 3,
}PCAP_ST_BCLK_TIME_SLOT_TYPE;

/* The bits 13 and 14 of ST_DAC register in PCAP indicate the data timing for the digital audio interface
   ____________________________________________________________________________________________________________
  |          definitions           |BCLK1(bit:14)|BCLK0(bit:13)| data timing for the digital audio interface   |
  |--------------------------------+-------------+-------------+-----------------------------------------------|
  |DIGITAL_AUDIO_INTERFACE_NORMAL  |      0      |      0      |Normal/MSB justified Mode; Word wide frame sync|
  |DIGITAL_AUDIO_INTERFACE_NETWORK |      0      |      1      |Time slotted Network Mode; Bit wide frame sync |
  |DIGITAL_AUDIO_INTERFACE_I2S     |      1      |      0      |I2S Mode; Word wide frame sync                 |
  +------------------------------------------------------------------------------------------------------------+
*/
typedef enum pcapDigitalAudioInterfaceMode_Type
{
       DIGITAL_AUDIO_INTERFACE_NORMAL   = 0,
       DIGITAL_AUDIO_INTERFACE_NETWORK  = 1,
       DIGITAL_AUDIO_INTERFACE_I2S      = 2
}PCAP_DIG_AUD_MODE_TYPE;


/* The bits 2,3 and 4 of ST_DAC register in PCAP indicate the Stereo Input clock select
   _________________________________________________________________________________________________________________
  |      definitions          |ST_CLK2(bit:4)|ST_CLK1(bit:3)|ST_CLK0(bit:2)| PLL Clock Input  |condition(SMB_ST_DAC)|
  |---------------------------+--------------+--------------+--------------+------------------+---------------------|
  |ST_CLK_PLL_CLK_IN_13M0     |       0      |       0      |       0      |    13.0MHz       |         0           |
  |ST_CLK_PLL_CLK_IN_15M36    |       0      |       0      |       1      |    15.36MHz      |         0           |
  |ST_CLK_PLL_CLK_IN_16M8     |       0      |       1      |       0      |    16.8MHz       |         0           |
  |ST_CLK_PLL_CLK_IN_19M44    |       0      |       1      |       1      |    19.44MHz      |         0           |
  |ST_CLK_PLL_CLK_IN_26M0     |       1      |       0      |       0      |    26.0MHz       |         0           |
  |ST_CLK_PLL_CLK_IN_EXT_MCLK |       1      |       0      |       1      |   Ext MCLK Pin   |         1           | 
  |ST_CLK_PLL_CLK_IN_FSYNC    |       1      |       1      |       0      |  FSYNC0[1] pin   |         1           |
  |ST_CLK_PLL_CLK_IN_BITCLK   |       1      |       1      |       1      |  BitCLK0[1] pin  |         1           |
  +-----------------------------------------------------------------------------------------------------------------+
*/
typedef enum pcapST_CLK_Type
{
        ST_CLK_PLL_CLK_IN_13M0           = 0,
        ST_CLK_PLL_CLK_IN_15M36          = 1,
        ST_CLK_PLL_CLK_IN_16M8           = 2,
        ST_CLK_PLL_CLK_IN_19M44          = 3,
        ST_CLK_PLL_CLK_IN_26M0           = 4,
        ST_CLK_PLL_CLK_IN_EXT_MCLK       = 5,
        ST_CLK_PLL_CLK_IN_FSYNC          = 6,
        ST_CLK_PLL_CLK_IN_BITCLK         = 7
}PCAP_ST_CLK_TYPE;

/* The bits 6,7 and 8 of AUD_CODEC register in PCAP indicate the Telephone CODEC Clock selection SPI Bits
   ______________________________________________________________________________________
  |      definitions   |CDC_CLK2(bit:8)|CDC_CLK1(bit:7)|CDC_CLK0(bit:6)| CLK_IN Frequency|
  |--------------------+---------------+---------------+---------------+-----------------|
  |CDC_CLK_IN_13M0     |       0       |       0       |       0       |    13.0MHz      |
  |CDC_CLK_IN_15M36    |       0       |       0       |       1       |    15.36MHz     |
  |CDC_CLK_IN_16M8     |       0       |       1       |       0       |    16.8MHz      |
  |CDC_CLK_IN_19M44    |       0       |       1       |       1       |    19.44MHz     |
  |CDC_CLK_IN_26M0     |       1       |       0       |       0       |    26.0MHz      |
  +--------------------------------------------------------------------------------------+
*/
typedef enum pcapCDCClk_Type
{
        CDC_CLK_IN_13M0                  = 0,
        CDC_CLK_IN_15M36                 = 1,
        CDC_CLK_IN_16M8                  = 2,
        CDC_CLK_IN_19M44                 = 3,
        CDC_CLK_IN_26M0                  = 4
}PCAP_PHONE_CDC_CLOCK_TYPE;


/* The bits 20 and 21 of AUX_VREG register in PCAP indicate the vibrator level 
   __________________________________________________________________________________
  |      definitions       |V_VIB_1(bit:21)|V_VIB_0(bit:20)|      Vibrator Level     |
  |------------------------+---------------+---------------+-------------------------|
  |VIBRATOR_VOLTAGE_LEVEL0 |       0       |       0       |Vibrator Voltage Level 0 |
  |VIBRATOR_VOLTAGE_LEVEL1 |       0       |       1       |Vibrator Voltage Level 1 |
  |VIBRATOR_VOLTAGE_LEVEL2 |       1       |       0       |Vibrator Voltage Level 2 |
  |VIBRATOR_VOLTAGE_LEVEL3 |       1       |       1       |Vibrator Voltage Level 3 |
  +----------------------------------------------------------------------------------+
*/
typedef enum pcapVibratorLevelType
{
        VIBRATOR_VOLTAGE_LEVEL0         = 0,
        VIBRATOR_VOLTAGE_LEVEL1         = 1,
        VIBRATOR_VOLTAGE_LEVEL2         = 2,
        VIBRATOR_VOLTAGE_LEVEL3         = 3
}PCAP_VibratorLevel_TYPE;


/* The bits 8,9,10 and 11 of ST_DAC register in PCAP indicate the ST_DAC sample rate
   __________________________________________________________________________________
  |     definitions   |SR3(bit:11)|SR2(bit:10)|SR1(bit:9)|SR0(bit:8)| ST Sample Rate |
  |-------------------+-----------+-----------+----------+----------+----------------|
  |ST_SAMPLE_RATE_8K  |     0     |     0     |    0     |    0     |    8000        | 
  |ST_SAMPLE_RATE_11K |     0     |     0     |    0     |    1     |    11025       |
  |ST_SAMPLE_RATE_12K |     0     |     0     |    1     |    0     |    12000       | 
  |ST_SAMPLE_RATE_16K |     0     |     0     |    1     |    1     |    16000       | 
  |ST_SAMPLE_RATE_22K |     0     |     1     |    0     |    0     |    22050       |
  |ST_SAMPLE_RATE_24K |     0     |     1     |    0     |    1     |    20000       |
  |ST_SAMPLE_RATE_32K |     0     |     1     |    1     |    0     |    32000       |
  |ST_SAMPLE_RATE_44K |     0     |     1     |    1     |    1     |    44000       |
  |ST_SAMPLE_RATE_48K |     1     |     0     |    0     |    0     |    48000       |
  +----------------------------------------------------------------------------------+
*/
typedef enum pcapSTSRType
{
        ST_SAMPLE_RATE_8K                = 0,
        ST_SAMPLE_RATE_11K               = 1,
        ST_SAMPLE_RATE_12K               = 2,
        ST_SAMPLE_RATE_16K               = 3,
        ST_SAMPLE_RATE_22K               = 4,
        ST_SAMPLE_RATE_24K               = 5,
        ST_SAMPLE_RATE_32K               = 6,
        ST_SAMPLE_RATE_44K               = 7,
        ST_SAMPLE_RATE_48K               = 8
}PCAP_ST_SAMPLE_RATE_TYPE;


/*================================================================================
                          STRUCTURES AND OTHER TYPEDEFS
================================================================================*/

                                                                                                                            
/*================================================================================
                               UNION DECLARATION
================================================================================*/

                                                                                                                            
/*================================================================================
                          GLOBAL VARIABLE DECLARATIONS
================================================================================*/
                                                                                                                             
                                                                                                                             
                                                                                                                             
/*================================================================================
                          GLOBAL POINTER DECLARATIONS
================================================================================*/
                                                                                                                             
                                                                                                                             
/*================================================================================
                          GLOBAL FUNCTION PROTOTYPES
================================================================================*/
int PCAP_MONO_set( PCAP_MONO_TYPE  monoType );         /* set the MONO type for AUD RX AMPS of PCAP */
int PCAP_AUDOG_set( int audioOutGain  );               /* set the gain of output from AUD RX AMPS of PCAP */
int PCAP_AUDIG_set( int audioInGain   );               /* set the gain of input to AUD TX AMPS of PCAP */ 
int PCAP_CDC_SR_set(PCAP_ST_SAMPLE_RATE_TYPE srType);  /* set the sample rate of ST DAC of PCAP */
void PCAP_vibrate_start_command(void);                  /* let the vibrator to start vibrate */
void PCAP_vibrate_stop_command(void);                  /* let the vibrator to stop vibrate */
void PCAP_V_VIB_level_set(PCAP_VibratorLevel_TYPE VIBLevel);    /* set the sample rate of ST DAC of PCAP */
void set_pcap_telephone_codec(void);

#ifdef __cplusplus
}
#endif
                                                                                                                             
#endif  /*PCAP_AUDIO_H*/
                                                                            

