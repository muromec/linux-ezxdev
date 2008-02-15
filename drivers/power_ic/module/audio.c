/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2005 - Motorola
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 * Motorola 2005-Feb-28 - Rewrote the software for PCAP.
 * Motorola 2005-Apr-04 - Added Atlas support to the file.
 * Motorola 2005-Jun-09 - Tuned stereo DAC and CODEC
 * Motorola 2005-Nov-22 - Finalized code and clean up.
 *
 */

/*!
 * @file audio.c
 *
 * @ingroup poweric_audio
 *
 * @brief Power IC audio module
 *
 * This file contains all of the functions and data structures required to implement the
 * interface module between audio manager from user space and power IC driver. This audio
 * module gives the user more flexibility to control the audio registers.
 */

/*====================================================================================================
                                        INCLUDE FILES
==================================================================================================*/
#include <asm/semaphore.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/moto_accy.h>
#include <linux/module.h>
#include <stdbool.h>
#include "audio.h"
#include "emu.h"
#include "../core/os_independent.h"

/*==================================================================================================
                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
typedef struct
{
    unsigned int dev_type;
    unsigned int write_mask1;
    unsigned int write_mask2;
} AUDIO_DATA_T;

/*==================================================================================================
                                     LOCAL CONSTANTS
==================================================================================================*/

/**********************************  PCAP2 definitions *************************************/

#ifdef CONFIG_MOT_POWER_IC_PCAP2     
#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*RX_AUD_AMPS register definition*/
#define A1_EN_MASK            0x00000001
#define A2_EN_MASK            0x00000002
#define PGA_OUTL_EN_MASK      0x00000004
#define A4_EN_MASK            0x00000010
#define ARIGHT_EN_MASK        0x00000020
#define ALEFT_EN_MASK         0x00000040
#define PGA_R_EN_MASK         0x00000800
#define PGA_L_EN_MASK         0x00001000

#define AUDIO_OUT_GAIN_INDEX    13
#define AUDIO_OUT_GAIN_NUM_BITS 4

/*TX_AUD_AMPS register definition*/
#define A3_MUX_MASK           0x00000040
#define A5_MUX_MASK           0x00000100
#define EXT_MIC_MUX_MASK      0x00000200
#define MB_ON2_MASK           0x00000400
#define MB_ON1_MASK           0x00000800
#define V2_EN_2_MASK          0x00200000

#define AUDIO_IN_GAIN_INDEX     0
#define AUDIO_IN_GAIN_NUM_BITS  5
#define CD_BYP_INDEX            7
#define CD_BYP_NUM_BITS         1

/*Register AUD_CODEC definition*/
#define AUDIHPF_INDEX         0
#define AUDIHPF_NUM_BITS      1
#define SMB_INDEX             1
#define SMB_NUM_BITS          1
#define AUDOHPF_INDEX         2
#define AUDOHPF_NUM_BITS      1
#define AUDITH_INDEX          5
#define AUDITH_NUM_BITS       1
#define CDC_RESET_INDEX       11
#define CDC_RESET_NUM_BITS    1
#define CDC_EN_INDEX          12
#define CDC_EN_NUM_BITS       1
#define FS_8K_16K_INDEX       14
#define FS_8K_16K_NUM_BITS    1

/*ST_DAC register definition*/
#define SMB_ST_DAC_INDEX            0
#define SMB_ST_DAC_NUM_BITS         1
#define ST_DAC_CLK_INDEX            2
#define ST_DAC_CLK_NUM_BITS         3
#define ST_DAC_RESET_INDEX          6
#define ST_DAC_RESET_NUM_BITS       1
#define ST_DAC_EN_INDEX             7
#define ST_DAC_EN_NUM_BITS          1
#define ST_DAC_SR_INDEX             8
#define ST_DAC_SR_NUM_BITS          4
#define ST_DAC_DIG_AUD_FS_INDEX    13
#define ST_DAC_DIG_AUD_FS_NUM_BITS  2
#define ST_DAC_NUM_TS_INDEX        15
#define ST_DAC_NUM_TS_NUM_BITS      2
#endif /* Doxygen skip bit definitions. */

/* defines for all power_ic_audio functions */
#define RX_AUDIO_0     POWER_IC_REG_PCAP_RX_AUD_AMPS
#define RX_AUDIO_1     POWER_IC_REG_PCAP_RX_AUD_AMPS
#define TX_AUDIO       POWER_IC_REG_PCAP_TX_AUD_AMPS
#define AUDIO_CODEC    POWER_IC_REG_PCAP_AUD_CODEC
#define ST_DAC         POWER_IC_REG_PCAP_ST_DAC
#define TIME_SLOTS     POWER_IC_REG_PCAP_ST_DAC
#define LOOPBACK       POWER_IC_REG_PCAP_RX_AUD_AMPS

/**************************************** ATLAS definitions ***************************************/

/* Use the same names as used for PCAP, real register name in commments */
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)                
#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*RX_AUD_AMPS register definition*/
#define A1_EN_MASK            0x00000008  /* ASPEN */
#define A2_EN_MASK            0x00000020  /* ALSPEN */
#define PGA_OUTL_EN_MASK      0x00000100  /* LSPLEN */
#define ARIGHT_EN_MASK        0x00000200  /* AHSREN */
#define ALEFT_EN_MASK         0x00000400  /* AHSLEN */
#define ARXOUTR_EN_MASK       0x00008000  /* ARXOUTREN */
#define ARXOUTL_EN_MASK       0x00010000  /* ARXOUTLEN */
#define PGA_R_EN_MASK         0x00000001  /* PGARXEN */
#define PGA_L_EN_MASK         0x00000020  /* PGASTEN */
#define VAUDIOON_MASK         0x00000001
#define BIASEN_MASK           0x00000002
#define BIASSPEED_MASK        0x00000004
#define HSPGDIS_MASK          0x00001000
#define HSDETEN_MASK          0x00002000
#define HSDETAUTOB_MASK       0x00004000
#define ADDCDC_MASK           0x00200000
#define ADDSTDC_MASK          0x00400000
#define AHSSEL_MASK           0x00000800
#define ASPSEL_MASK           0x00000010
#define ALSPSEL_MASK          0x00000080
#define ARXOUTSEL_MASK        0x00020000

#define AUDIO_OUT_CODEC_GAIN_INDEX     1
#define AUDIO_OUT_STDAC_GAIN_INDEX     6
#define AUDIO_OUT_GAIN_NUM_BITS        4
#define PGARXEN_INDEX                  0
#define PGARXEN_NUM_BITS               1
#define PGASTEN_INDEX                  5
#define PGASTEN_NUM_BITS               1

/*TX_AUD_AMPS register definition*/
#define A3_MUX_MASK           0x00000020  /* AMC1REN */
#define A5_MUX_MASK           0x00000200  /* AMC2EN */
#define EXT_MIC_MUX_MASK      0x00000800  /* ATXINEN */
#define MB_ON2_MASK           0x00000002  /* MC2BEN */
#define MB_ON1_MASK           0x00000001  /* MC1BEN */

#define AUDIO_IN_GAIN_INDEX     14
#define AUDIO_IN_GAIN_NUM_BITS  5

/*Register AUD_CODEC definition*/
#define FS_8K_16K_INDEX       10
#define FS_8K_16K_NUM_BITS    1
#define CDC_EN_INDEX          11
#define CDC_EN_NUM_BITS       1
#define AUDITH_INDEX          14
#define AUDITH_NUM_BITS       1
#define CDC_RESET_INDEX       15
#define CDC_RESET_NUM_BITS    1
#define CD_BYP_INDEX          16
#define CD_BYP_NUM_BITS       1
#define AUDIHPF_INDEX         19
#define AUDIHPF_NUM_BITS      1
#define AUDOHPF_INDEX         20
#define AUDOHPF_NUM_BITS      1

/* Register AUD_CODEC Mask definition */
#define CDC_CLKSEL_MASK       1<<1    /* Bit 1 should be cleared to select CLIA */
#define CDC_SMB_MASK          1<<2    /* Slave / Master mode */
#define CDC_FS_MASK           1<<5    /* Set Bit 5, Network mode */
#define CDC_CLK0_MASK         1<<7    
#define CDC_CLK1_MASK         1<<8    
#define CDC_CLK2_MASK         1<<9    /* Set Bit 9, Bit[987] = 100, for selecting clock as 26Mhz */
#define CDC_CLKEN_MASK        1<<12   /* Set Bit 12 */
#define CDC_RESET_MASK        1<<15   /* Set Bit 15, Filter reset*/

/*ST_DAC register definition*/
#define ST_DAC_DIG_AUD_FS_INDEX      5
#define ST_DAC_DIG_AUD_FS_NUM_BITS   2
#define ST_DAC_EN_INDEX              11
#define ST_DAC_EN_NUM_BITS           1
#define ST_DAC_RESET_INDEX           15
#define ST_DAC_RESET_NUM_BITS        1
#define ST_DAC_SR_MASK               0x1E0000
#define ST_DAC_SR_INDEX              17
#define ST_DAC_SR_NUM_BITS           4

/* Register STDAC Mask definiton */
#define ST_DAC_SSISEL_MASK           1<<0  /* Set Bit 0, Select SSI2 */
#define ST_DAC_CLKSEL_MASK           1<<1  /* Bit 1 should be cleared to select CLIA */
#define ST_DAC_SMB_MASK              1<<2  /* Slave / Master mode */
#define ST_DAC_CLK0_MASK             1<<7  /* Set Bit 7, Bit[987] = 111, when STDAC in slave mode  */
#define ST_DAC_CLK1_MASK             1<<8  /* Set Bit 8 */
#define ST_DAC_CLK2_MASK             1<<9  /* Set Bit 9, Bit[987] = 100, for selecting clock as 26Mhz  */
#define ST_DAC_CLKEN_MASK            1<<12 /* Set Bit 12 */
#define ST_DAC_RESET_MASK            1<<15 /* Set Bit 15, Filter reset */  

/* SSI Network */
#define ST_DAC_NUM_TS_INDEX          12
#define ST_DAC_NUM_TS_NUM_BITS       2
#define ST_DAC_TIMESLOTS             0x002060
#endif /* Doxygen skip bit definitions. */

/* defines for all power_ic_audio functions */
#define RX_AUDIO_0     POWER_IC_REG_ATLAS_AUDIO_RX_0
#define RX_AUDIO_1     POWER_IC_REG_ATLAS_AUDIO_RX_1
#define TX_AUDIO       POWER_IC_REG_ATLAS_AUDIO_TX
#define AUDIO_CODEC    POWER_IC_REG_ATLAS_AUDIO_CODEC
#define ST_DAC         POWER_IC_REG_ATLAS_AUDIO_STEREO_DAC
#define TIME_SLOTS     POWER_IC_REG_ATLAS_SSI_NETWORK
#define LOOPBACK       POWER_IC_REG_ATLAS_AUDIO_CODEC

#endif /* ATLAS */

/* The output path type is using the mask in order to enable multiple paths*/
static const AUDIO_DATA_T audio_out_path[POWER_IC_NUM_OF_OUT_PATH] ={    
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    {POWER_IC_NO_SPEAKER_MASK,0,0},                                                             /*NO speaker*/
    {POWER_IC_HEADSET_SPEAKER_MASK,PGA_R_EN_MASK,ARIGHT_EN_MASK},                               /*Headset speaker*/
    {POWER_IC_HANDSET_SPEAKER_MASK,PGA_R_EN_MASK,A1_EN_MASK},                                   /*Handset speaker*/
    {POWER_IC_ALERT_SPEAKER_MASK,PGA_R_EN_MASK,A2_EN_MASK},                                     /*Alert speaker*/
    {POWER_IC_ST_HEADSET_SPEAKER_MASK,PGA_R_EN_MASK|PGA_L_EN_MASK,ARIGHT_EN_MASK|ALEFT_EN_MASK},/*ST headset speaker*/
    {POWER_IC_BUS_SPEAKER_MASK,PGA_R_EN_MASK,A4_EN_MASK},                                       /*Bus Speaker*/
    {POWER_IC_ST_HANDSET_SPEAKER_MASK,PGA_L_EN_MASK,PGA_OUTL_EN_MASK},                          /*ST headset speaker*/
    {POWER_IC_EMU_HEADSET_SPEAKER_MASK, PGA_R_EN_MASK,ARIGHT_EN_MASK},                          /*EMU mono headset*/
    {POWER_IC_EMU_ST_HEADSET_SPEAKER_MASK, PGA_R_EN_MASK|PGA_L_EN_MASK,ARIGHT_EN_MASK|ALEFT_EN_MASK}  /*EMU stereo headset*/
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    {POWER_IC_NO_SPEAKER_MASK,0,0},                                                             /*NO speaker*/
    {POWER_IC_HEADSET_SPEAKER_MASK,0,ARIGHT_EN_MASK},                                           /*Headset speaker*/
    {POWER_IC_HANDSET_SPEAKER_MASK,0,A1_EN_MASK},                                               /*Handset speaker*/
    {POWER_IC_ALERT_SPEAKER_MASK,0,A2_EN_MASK|PGA_OUTL_EN_MASK},                                /*Alert speaker*/    
    {POWER_IC_ST_HEADSET_SPEAKER_MASK,0,ARIGHT_EN_MASK|ALEFT_EN_MASK},                          /*ST headset speaker*/
    {POWER_IC_BUS_SPEAKER_MASK,0,0},                                                            /*Bus Speaker (carkit)*/
    {POWER_IC_ST_HANDSET_SPEAKER_MASK,0,A2_EN_MASK|PGA_OUTL_EN_MASK},                           /*ST handset speaker*/    
    {POWER_IC_EMU_HEADSET_SPEAKER_MASK, 0, 0},                                                  /*EMU mono headset*/
    {POWER_IC_EMU_ST_HEADSET_SPEAKER_MASK, 0, 0}                                                /*EMU stereo headset*/
#endif /* ATLAS */
}; 


/*Audio input path */
static const AUDIO_DATA_T audio_in_path[POWER_IC_NUM_OF_IN_PATH]={
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    {POWER_IC_NO_MIC,0,0},                                                                     /*NO MIC*/
    {POWER_IC_HEADSET_MIC,A5_MUX_MASK |V2_EN_2_MASK, MB_ON1_MASK},                             /*Headset MIC*/
    {POWER_IC_HANDSET_MIC,(A3_MUX_MASK | V2_EN_2_MASK), MB_ON2_MASK},                          /*Handset MIC*/
    {POWER_IC_EXTERNAL_MIC,(EXT_MIC_MUX_MASK |V2_EN_2_MASK),(EXT_MIC_MUX_MASK |V2_EN_2_MASK)}  /*External MIC*/
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    {POWER_IC_NO_MIC,0,0},                                                                     /*NO MIC*/
    {POWER_IC_HEADSET_MIC,A5_MUX_MASK, MB_ON2_MASK},                                           /*Headset MIC*/
    {POWER_IC_HANDSET_MIC,A3_MUX_MASK, MB_ON1_MASK},                                           /*Handset MIC*/
    {POWER_IC_EXTERNAL_MIC,0,EXT_MIC_MUX_MASK}                                                 /*External MIC*/
#endif /* ATLAS */
};
              

/*==================================================================================================
                                        LOCAL MACROS
==================================================================================================*/

/*==================================================================================================
                                      LOCAL VARIABLES
==================================================================================================*/
static unsigned int prev_input_path = POWER_IC_NO_MIC;
static unsigned int prev_output_path = POWER_IC_NO_SPEAKER_MASK;
static DECLARE_MUTEX(audio_mutex);
/*==================================================================================================
                                     GLOBAL VARIABLES
==================================================================================================*/

/*==================================================================================================
                                     EXPORTED SYMBOLS
==================================================================================================*/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
EXPORT_SYMBOL(power_ic_audio_write_output_path);
EXPORT_SYMBOL(power_ic_audio_read_output_path);
EXPORT_SYMBOL(power_ic_audio_write_input_path);
EXPORT_SYMBOL(power_ic_audio_read_input_path);
EXPORT_SYMBOL(power_ic_audio_write_igain);
EXPORT_SYMBOL(power_ic_audio_read_igain);
EXPORT_SYMBOL(power_ic_audio_write_ogain);
EXPORT_SYMBOL(power_ic_audio_read_ogain);
EXPORT_SYMBOL(power_ic_audio_codec_en);
EXPORT_SYMBOL(power_ic_audio_st_dac_en);
EXPORT_SYMBOL(power_ic_audio_set_codec_sm);
EXPORT_SYMBOL(power_ic_audio_set_st_dac_sm);
EXPORT_SYMBOL(power_ic_audio_set_st_dac_sr);
EXPORT_SYMBOL(power_ic_audio_set_codec_sr);
EXPORT_SYMBOL(power_ic_audio_config_loopback);
EXPORT_SYMBOL(power_ic_audio_set_output_hp);
EXPORT_SYMBOL(power_ic_audio_set_input_hp);
EXPORT_SYMBOL(power_ic_audio_dither_en);
EXPORT_SYMBOL(power_ic_audio_set_st_dac_net_mode);
EXPORT_SYMBOL(power_ic_audio_set_st_dac_num_ts);
EXPORT_SYMBOL(power_ic_audio_power_off);
EXPORT_SYMBOL(power_ic_audio_st_dac_reset);
EXPORT_SYMBOL(power_ic_audio_codec_reset);
EXPORT_SYMBOL(power_ic_audio_conn_mode_set);
#endif

/*==================================================================================================
                                     Local Functions
==================================================================================================*/
/*!
 * @brief Configure the audio ouput path.
 *  This function enables/disables the audio output paths. It allows the multiple paths enabled at the same time.
 *  The audio output path masks are used for the output path combination.
 *
 * @param  out_type  The audio path requested.                        
 * @param  en_val    true for enable, false for disable
 *
 * @return returns 0 if successful.
 */

static int audio_config_output_path(unsigned int out_type, bool en_val) 
{
    unsigned short i;
    unsigned int out_mask1 = 0;
    unsigned int out_mask2 = 0;
    int ret_val = 0;
    unsigned int mask_val1 =0;
    unsigned int mask_val2 =0;
    
    for(i=0; i<POWER_IC_NUM_OF_OUT_PATH; i++)
    {
        if(out_type & audio_out_path[i].dev_type)
        {
            tracemsg(_k_d("The path is 0x%7x"),audio_out_path[i].dev_type);
            if(en_val)
            {
                out_mask1 |= audio_out_path[i].write_mask1;
                out_mask2 |= audio_out_path[i].write_mask2;
                mask_val1 = out_mask1;
                mask_val2 = out_mask2;
            }
            else
            {
                out_mask1 |= audio_out_path[i].write_mask2;
                out_mask2 |= audio_out_path[i].write_mask1;
            }
        }
    }
     
    tracemsg(_k_d("The mask value is: mask1 = 0x%7x, mask2 = 0x%7x"),
             out_mask1,out_mask2);

    ret_val = power_ic_set_reg_mask(RX_AUDIO_1,out_mask1,mask_val1);

    if(ret_val)
    {
        return ret_val;
    }

    ret_val = power_ic_set_reg_mask(RX_AUDIO_0,out_mask2,mask_val2);
        
    return ret_val;
   
}

/*!
 * @brief Configure the audio input path.
 *  This function enable/disable the audio input path. Only one path is enabled/disabled.
 *
 * @param  in_dev   the requested input audio path.
 * @param  en_val   true for enable, false for disable
 *
 * @return returns 0 if successful.
 */

static int audio_config_input_path(POWER_IC_AUD_IN_PATH_T in_dev, bool en_val) 
{
    unsigned short i;
    unsigned int ret_val = 0;
    unsigned int in_mask1 = 0;
    unsigned int in_mask2 = 0;
    unsigned int mask_val1 =0;
    unsigned int mask_val2 =0;
    
    for(i=0; i< POWER_IC_NUM_OF_IN_PATH; i++)
    {
        if(in_dev == audio_in_path[i].dev_type)
        {
            tracemsg(_k_d("The input path is passed in: %d"),in_dev);
            if(en_val)
            {
                in_mask1 = audio_in_path[i].write_mask1;
                in_mask2 = audio_in_path[i].write_mask2;
                mask_val1 = in_mask1;
                mask_val2 = in_mask2;
            }
            else
            {
                in_mask1 = audio_in_path[i].write_mask2;
                in_mask2 = audio_in_path[i].write_mask1;
            }
            break;
        }
       
    }
    tracemsg(_k_d("The mask value is write_mask1 = 0x%7x, write_mask2 = 0x%7x"),
             in_mask1, in_mask2);

    ret_val = power_ic_set_reg_mask(TX_AUDIO, in_mask1, mask_val1);

    if(ret_val)
    {
        return ret_val;
    }
    if(in_dev != POWER_IC_EXTERNAL_MIC)
    {          
        ret_val = power_ic_set_reg_mask(TX_AUDIO, in_mask2, mask_val2);
    }
    return ret_val;
        
}

/*==================================================================================================
                                     GLOBAL FUNCTIONS
==================================================================================================*/
/*!
 * @brief Reset STDAC by setting STDCRESET bit
 * This function resets the STDAC
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_st_dac_reset(void)
{
    return (power_ic_set_reg_value(ST_DAC,
                                   ST_DAC_RESET_INDEX,
                                   1,
                                   ST_DAC_RESET_NUM_BITS));
}

/*!
 * @brief Reset CODEC by setting CDCRESET bit
 * This function resets the CODEC
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_codec_reset(void)
{
    return (power_ic_set_reg_value(AUDIO_CODEC,
                                   CDC_RESET_INDEX,
                                   1,
                                   CDC_RESET_NUM_BITS));
}

/*!
 * @brief Switch of power to the audio section of the Power IC chip
 * This function switches off the audio section of Power IC chip 
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_power_off(void)
{
    int error;
    tracemsg(_k_d("\nAudio power being turned off"));
    error = power_ic_write_reg_value(AUDIO_CODEC, 0x0);
    error |= power_ic_write_reg_value(ST_DAC, 0x0);
    error |= power_ic_write_reg_value(RX_AUDIO_0, 0x02000);
    error |= power_ic_write_reg_value(RX_AUDIO_1, 0x0);
    return error;
}

/*!
 * @brief Set audio output path
 * This function sets up the audio output paths based on the device type mask passed in. It allows
 * the multiple paths set up at the same time.
 *
 * @param  out_type    the audio paths requested.
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_write_output_path(unsigned int out_type)
{
    int ret_val = 0;
    tracemsg(_k_d("Disable old out path %d"), prev_output_path);
   
    /* If a signal caused us to continue without getting the mutex, then quit with an error. */
    if(down_interruptible(&audio_mutex) != 0)
    {
        tracemsg(_k_d("   process received signal while waiting for audio mutex. Exiting."));
        return -EINTR;
    }
    
    /*Disable the old audio path*/
    ret_val = audio_config_output_path(prev_output_path,false);
    if(ret_val)
    {
        tracemsg(_k_d("Error in disable the old audio path."));  
        prev_output_path = POWER_IC_ALL_SPEAKER_MASK;
        up(&audio_mutex);
        return ret_val;
    }
    
    /*Enable the new audio path*/
    tracemsg(_k_d("Enable new out path %d"), out_type);
    ret_val = audio_config_output_path(out_type,true); 
    if(ret_val)
    {
        tracemsg(_k_d("Error in enable the new audio path."));  
        prev_output_path = POWER_IC_NO_SPEAKER_MASK;
        up(&audio_mutex);
        return ret_val;
    }       
    /* Release the lock */
    up(&audio_mutex);
    prev_output_path = out_type;
    power_ic_audio_conn_mode_set(prev_output_path, prev_input_path);
    return ret_val;
    
}

/*!
 * @brief Read the audio output path
 * This function read out the current audio output paths.
 *
 * @return returns the current saved output paths.
 */

int power_ic_audio_read_output_path(void)
{
    tracemsg(_k_d("The out path is %d"), prev_output_path);
    return prev_output_path;
}

/*!
 * @brief Set the audio input path
 * This function sets up the audio input path based on the device type passed in.
 *
 * @param in_type   the audio path requested.
 *
 * @return   returns 0 if successful.
 */

int power_ic_audio_write_input_path(POWER_IC_AUD_IN_PATH_T in_type)
{
    int ret_val = 0;

    tracemsg(_k_d("Disable the old input path %d"), prev_input_path);
    if(in_type >= POWER_IC_NUM_OF_IN_PATH)
    {
        return -EINVAL;
    }
    
    /* If a signal caused us to continue without getting the mutex, then quit with an error. */
    if(down_interruptible(&audio_mutex) != 0)
    {
        tracemsg(_k_d("   process received signal while waiting for audio mutex. Exiting."));
        return -EINTR;
    }
     
    ret_val = audio_config_input_path(prev_input_path,false);
    if(ret_val)
    {
        prev_input_path = POWER_IC_NUM_OF_IN_PATH;
        up(&audio_mutex);
        return ret_val;
    }
    tracemsg(_k_d("Enable the new input path %d"), in_type);
    ret_val = audio_config_input_path(in_type,true);
    if(ret_val)
    {
        prev_input_path = POWER_IC_NO_MIC;
        up(&audio_mutex);
        return ret_val;
    }
    /* Release the lock */
    up(&audio_mutex);
    prev_input_path = in_type;
    power_ic_audio_conn_mode_set(prev_output_path, prev_input_path);
    return ret_val;
}

/*!
 * @brief read the audio input path
 * This function read the current audio input path.
 *
 * @return return the current input path.
 */

POWER_IC_AUD_IN_PATH_T power_ic_audio_read_input_path(void)
{
    return prev_input_path;
}

/*!
 * @brief Write the input gain
 * This function writes the input audio gain into register.
 *
 * @param  igain     audio input gain value.
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_write_igain(int igain)
{
    tracemsg(_k_d("The input gain is %d"),igain);

    return (power_ic_set_reg_value(TX_AUDIO,
                                   AUDIO_IN_GAIN_INDEX,
                                   igain,
                                   AUDIO_IN_GAIN_NUM_BITS));
}

/*!
 * @brief Read the output gain
 * This function read the audio input gain from the register.
 *
 * @param  igain    audio input gain pointer
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_read_igain(int *igain)
{
    return (power_ic_get_reg_value(TX_AUDIO,
                                   AUDIO_IN_GAIN_INDEX,
                                   igain,
                                   AUDIO_IN_GAIN_NUM_BITS ));
}

/*!
 * @brief Write the output gain
 * This function write the output gain value into register
 *
 * @param  ogain    audio output gain value
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_write_ogain(int ogain)
{
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    return (power_ic_set_reg_value(RX_AUDIO_1,
                                   AUDIO_OUT_GAIN_INDEX,
                                   ogain,
                                   AUDIO_OUT_GAIN_NUM_BITS));
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    int error;
   
    error = power_ic_set_reg_value(RX_AUDIO_1,
                                   AUDIO_OUT_CODEC_GAIN_INDEX,
                                   ogain,
                                   AUDIO_OUT_GAIN_NUM_BITS);
    error |= power_ic_set_reg_value(RX_AUDIO_1,
                                    AUDIO_OUT_STDAC_GAIN_INDEX,
                                    ogain,
                                    AUDIO_OUT_GAIN_NUM_BITS);
    return error;
#endif
}

/*!
 * @brief Read the output gain
 * This function read the output gain from the register
 *
 * @param  ogain   audio output gain value
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_read_ogain(int* ogain)
{
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    return (power_ic_get_reg_value(RX_AUDIO_1,
                                   AUDIO_OUT_GAIN_INDEX,
                                   ogain,
                                   AUDIO_OUT_GAIN_NUM_BITS));
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    return (power_ic_get_reg_value(RX_AUDIO_1,
                                   AUDIO_OUT_STDAC_GAIN_INDEX,
                                   ogain,
                                   AUDIO_OUT_GAIN_NUM_BITS ));
#endif
}

/*!
 * @brief Enable/Disable codec
 * This function configures the audio codec.
 *
 * @param  val    true for enable, false for disable
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_codec_en(bool val)
{
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    return (power_ic_set_reg_value(AUDIO_CODEC,
                                   CDC_EN_INDEX,
                                   (val!=false),
                                   CDC_EN_NUM_BITS));
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    int error;
    int mask;
    int setup = 0;
    
    /* For RX1, turn on bit 0 */
    error = power_ic_set_reg_value(POWER_IC_REG_ATLAS_AUDIO_RX_1, PGARXEN_INDEX, (val != false), PGARXEN_NUM_BITS);
    tracemsg(_k_d("\nRX1 is setup"));
    
    /* For RX0, turn on bits 0,1,12,13,14,21 */
    mask =  VAUDIOON_MASK | BIASEN_MASK | HSPGDIS_MASK | HSDETEN_MASK | HSDETAUTOB_MASK | ADDCDC_MASK;
    setup = VAUDIOON_MASK | BIASEN_MASK | HSPGDIS_MASK | HSDETEN_MASK | HSDETAUTOB_MASK | ADDCDC_MASK;
    
    error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_AUDIO_RX_0, mask, setup);
    tracemsg(_k_d("\nRX0 is set up"));
    
    /* Enable timeslots for CODEC and ST DAC on SSI network register */
    error |= power_ic_write_reg_value(POWER_IC_REG_ATLAS_SSI_NETWORK, ST_DAC_TIMESLOTS);
    tracemsg(_k_d("Writing register SSI"));

    /* Enable the codec */
    error |= power_ic_set_reg_value(AUDIO_CODEC, CDC_EN_INDEX,(val!=false),CDC_EN_NUM_BITS);

    tracemsg(_k_d("\nCodec is set up"));

    return (error);
#endif    
}

/*!
 * @brief Enable/Disable st dac
 * This function configures the audio st dac.
 *
 * @param   val    true for enable, false for disable
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_st_dac_en(bool val)
{ 
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    return (power_ic_set_reg_value(ST_DAC,
                                   ST_DAC_EN_INDEX,
                                   (val!=false),
                                   ST_DAC_EN_NUM_BITS));
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    int error;
    int mask;
    int setup = 0;
    /* For RX1, turn on bit 5 */
    error = power_ic_set_reg_value(POWER_IC_REG_ATLAS_AUDIO_RX_1, PGASTEN_INDEX, (val != false), PGASTEN_NUM_BITS);

    /* For RX0, turn on bits 0, 1, 2, 4, 7, 11, 12, 13, 14, 17, 22 */
    mask =  VAUDIOON_MASK | BIASEN_MASK | BIASSPEED_MASK | HSPGDIS_MASK | HSDETEN_MASK | HSDETAUTOB_MASK | 
            ADDSTDC_MASK | AHSSEL_MASK | ASPSEL_MASK | ALSPSEL_MASK | ARXOUTSEL_MASK;
    
    setup = VAUDIOON_MASK | BIASEN_MASK | BIASSPEED_MASK | HSPGDIS_MASK | HSDETEN_MASK | HSDETAUTOB_MASK | 
                ADDSTDC_MASK | AHSSEL_MASK | ASPSEL_MASK | ALSPSEL_MASK | ARXOUTSEL_MASK;
    
    error |= power_ic_set_reg_mask(POWER_IC_REG_ATLAS_AUDIO_RX_0, mask, setup);

    /* Enable timeslots for CODEC and ST DAC on SSI network register */
    error |= power_ic_write_reg_value(POWER_IC_REG_ATLAS_SSI_NETWORK, ST_DAC_TIMESLOTS);
    tracemsg(_k_d("Writing register SSI"));
    
    /* Enable ST DAC */
    error |= power_ic_set_reg_value(ST_DAC,ST_DAC_EN_INDEX,(val!=false),ST_DAC_EN_NUM_BITS);

    tracemsg(_k_d("\nSTDAC is set up"));

    return (error);
#endif
}

/*!
 * @brief Set the codec slave or master mode
 * This function configures the audio codec slave/master bit.
 *
 * @param  val  true for slave, false for master
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_set_codec_sm(bool val)
{
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    return (power_ic_set_reg_value(AUDIO_CODEC,
                                   SMB_INDEX,
                                   (val!=false),
                                   SMB_NUM_BITS));
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    int codec_mask, codec_setup;

    codec_mask = CDC_CLKSEL_MASK | CDC_FS_MASK | CDC_CLK0_MASK | CDC_CLK1_MASK | CDC_CLK2_MASK | 
                 CDC_SMB_MASK | CDC_CLKEN_MASK | CDC_RESET_MASK ;
    /* Codec is master */
    if (val == false)
    {    
        codec_setup = CDC_FS_MASK | CDC_CLK2_MASK | CDC_CLKEN_MASK | CDC_RESET_MASK;
    }
    else /* Codec is slave */
    {
        codec_setup = CDC_FS_MASK | CDC_CLK2_MASK | CDC_SMB_MASK | CDC_RESET_MASK;
    }

    return (power_ic_set_reg_mask(AUDIO_CODEC, codec_mask, codec_setup));
#endif
}

/*!
 * @brief Set the st dac slave or master mode
 * This function configures the audio st dac slave/master bit.
 *
 * @param  val  true for slave, false for master
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_set_st_dac_sm(bool val)
{
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    return (power_ic_set_reg_value(ST_DAC,
                                   SMB_ST_DAC_INDEX,
                                   (val!=false),
                                   SMB_ST_DAC_NUM_BITS));
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    int stdac_mask, stdac_setup;

     stdac_mask = ST_DAC_SSISEL_MASK |ST_DAC_CLKSEL_MASK | ST_DAC_CLK0_MASK | ST_DAC_CLK1_MASK | 
                  ST_DAC_CLK2_MASK | ST_DAC_CLKEN_MASK | ST_DAC_RESET_MASK | ST_DAC_SMB_MASK | 
                  ST_DAC_SR_MASK;  

    /*  If STDAC is master */
    if (val == false)
    {
        stdac_setup = ST_DAC_SSISEL_MASK | ST_DAC_CLK2_MASK | ST_DAC_CLKEN_MASK | ST_DAC_RESET_MASK; 
    }
    else /* STDAC is slave */
    {
        stdac_setup = ST_DAC_SSISEL_MASK | ST_DAC_CLK0_MASK | ST_DAC_CLK1_MASK | ST_DAC_CLK2_MASK | 
                      ST_DAC_RESET_MASK | ST_DAC_SMB_MASK; 
    }
    
    return (power_ic_set_reg_mask(ST_DAC, stdac_mask, stdac_setup));
#endif
}

/*!
 * @brief Set st dac sample rate
 * This function configures the audio st dac sample rate.
 * @param  val    st dac sample rate value
 *
 * @return  returns 0 if successful.
 */

int power_ic_audio_set_st_dac_sr(POWER_IC_ST_DAC_SR_T val)
{
    tracemsg(_k_d("STDAC SAMPLING RATE =%d"), val);
    if((val < POWER_IC_ST_DAC_SR_8000) || (val > POWER_IC_ST_DAC_SR_48000))
    {
        return  -EINVAL;
    }

    return(power_ic_set_reg_value(ST_DAC,
                                  ST_DAC_SR_INDEX,
                                  val,
                                  ST_DAC_SR_NUM_BITS));
}

/*!
 * @brief Set codec sample rate
 * This function configures the audio codec sample rate.
 *
 * @param  val     codec sample rate value
 *
 * @return returns 0 if successful.
 */
int power_ic_audio_set_codec_sr(POWER_IC_CODEC_SR_T val)
{
    if((val < POWER_IC_CODEC_SR_8000) || (val > POWER_IC_CODEC_SR_16000))
    {
        return  -EINVAL;
    }

    return(power_ic_set_reg_value(AUDIO_CODEC,
                                  FS_8K_16K_INDEX,
                                  val,
                                  FS_8K_16K_NUM_BITS));
}

/*!
 * @brief Set loopback path
 * This function configures the audio loopback test.
 *
 * @param   val    true for enable, false for disable
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_config_loopback(bool val)
{
    
    /*The EZX code put the codec enable first if the lp mode enabled here.
     *But there is not disable. Here we allow the test command or audio manager
     *team to control this. If it needs to enable codec, call the codec enable
     *ioctl command. Here we only enable the lp back mode.*/
    return(power_ic_set_reg_value(LOOPBACK,
                                  CD_BYP_INDEX,
                                  (val!=false),
                                  CD_BYP_NUM_BITS ));
}

/*!
 * @brief Set up the output high pass filter
 * This function configures the audio codec output highpass filter.
 *
 * @param  val    true for enable, false for disable 
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_set_output_hp(bool val)
{
    return(power_ic_set_reg_value(AUDIO_CODEC,
                                  AUDOHPF_INDEX,
                                  (val != false),
                                  AUDOHPF_NUM_BITS));
}

/*!
 * @brief Set up the input high pass filter
 * This function configures the audio codec input highpass filter.
 *
 * @param  val    true for enable, false for disable 
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_set_input_hp(bool val)
{
    return(power_ic_set_reg_value(AUDIO_CODEC,
                                  AUDIHPF_INDEX,
                                  (val!=false),
                                  AUDIHPF_NUM_BITS));
}

/*!
 * @brief Enable/Disable the dither bit
 * This function configures the audio codec dither bit.
 *
 * @param  val    true for enable, false for disable 
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_dither_en(bool val)
{
    return (power_ic_set_reg_value(AUDIO_CODEC,
                                   AUDITH_INDEX,
                                   (val!=false),
                                   AUDITH_NUM_BITS));
}

/*!
 * @brief Set the network mode
 * This function configures the audio network mode in st dac register.
 *
 * @param  val  network mode value
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_set_st_dac_net_mode(int val)
{
    return (power_ic_set_reg_value(ST_DAC,
                                   ST_DAC_DIG_AUD_FS_INDEX,
                                   val,
                                   ST_DAC_DIG_AUD_FS_NUM_BITS));
}

 /* @brief Set up the time slot
 * This function configures the audio time slots in st dac register.
 *
 * @param  val       time slot value
 *
 * @return returns 0 if successful.
 */

int power_ic_audio_set_st_dac_num_ts(int val)
{
    return (power_ic_set_reg_value(TIME_SLOTS,
                                   ST_DAC_NUM_TS_INDEX,
                                   val,
                                   ST_DAC_NUM_TS_NUM_BITS));
}

 /* @brief Set up the conn mode for EMU
 * This function configures the conn mode for EMU Headsets
 *
 * @param  out_type       output path
 * @param  in_type        input path
 *
 */

void power_ic_audio_conn_mode_set(unsigned int out_type, POWER_IC_AUD_IN_PATH_T in_type)
{
    MOTO_ACCY_MASK_T connected_device = moto_accy_get_all_devices();;
    
    /* If the outpath is set to EMU Stereo Headset, set the headset to stereo mode*/
    if((out_type & POWER_IC_EMU_ST_HEADSET_SPEAKER_MASK) != 0)
    {
        emu_util_set_emu_headset_mode(MOTO_ACCY_HEADSET_MODE_STEREO);
    }
    /* If the input path is headset mic or out put path is EMU headset speaker, set the headset to MONO*/
    else if((out_type & (POWER_IC_EMU_HEADSET_SPEAKER_MASK) != 0))
    {
        emu_util_set_emu_headset_mode(MOTO_ACCY_HEADSET_MODE_MONO);
    }
    /* If the inpath is External Mic or the outpath is an EMU Carkit, then set the conn mode to Mono */ 
    else if(( out_type & POWER_IC_BUS_SPEAKER_MASK) != 0)
    {
        EMU_SET_EMU_CONN_MODE(EMU_CONN_MODE_MONO_AUDIO);
    }
    /*For external mic, the conn mode is set to mono audio.*/
    else if(in_type == POWER_IC_EXTERNAL_MIC)
    {
        /* If any type of EMU headset connected, we need to set the headset mode to none.*/
        if ((ACCY_BITMASK_ISSET(connected_device, MOTO_ACCY_TYPE_HEADSET_EMU_MONO))||
            (ACCY_BITMASK_ISSET(connected_device,MOTO_ACCY_TYPE_HEADSET_EMU_STEREO)))
        {
            emu_util_set_emu_headset_mode(MOTO_ACCY_HEADSET_MODE_MONO);
        }
        else
        {
            EMU_SET_EMU_CONN_MODE( EMU_CONN_MODE_MONO_AUDIO);
        }
    }
    /* For everything else, set the conn mode to None, which will be USB */
    else
    {
        /* If any type of EMU headset connected, we need to set the headset mode to none.*/
        if ((ACCY_BITMASK_ISSET(connected_device, MOTO_ACCY_TYPE_HEADSET_EMU_MONO))||
            (ACCY_BITMASK_ISSET(connected_device,MOTO_ACCY_TYPE_HEADSET_EMU_STEREO)))
        {
            emu_util_set_emu_headset_mode(MOTO_ACCY_HEADSET_MODE_NONE);
        }
        else
        {
            EMU_SET_EMU_CONN_MODE(EMU_CONN_MODE_USB);
        }
    }
}

/*!
 * @brief Audio module ioctl function.
 * This function implements IOCTL controls on a Power_Ic audio module.
 *
 * @param  cmd     the command
 * @param  arg     the parameter
 *
 * @return returns 0 if successful.
 */


int audio_ioctl(unsigned int cmd, unsigned long arg)
{
        unsigned long val= arg;
        int ret_val = 0;

        switch (cmd) 
        {
            case  POWER_IC_IOCTL_AUDIO_WRITE_OUTPATH:
                ret_val = power_ic_audio_write_output_path((unsigned int)val);
                break;
                
            case  POWER_IC_IOCTL_AUDIO_READ_OUTPATH:
                tracemsg(_k_d("The out path is %d"), prev_output_path);
                /* Return the previous audio path */
                ret_val = put_user(prev_output_path, (unsigned int *)arg);
                break;
                
            case POWER_IC_IOCTL_AUDIO_WRITE_INPATH:
                ret_val = power_ic_audio_write_input_path((POWER_IC_AUD_IN_PATH_T)val);
                break;
            case POWER_IC_IOCTL_AUDIO_READ_INPATH:
                tracemsg(_k_d("The in path is %d"), prev_input_path);
                ret_val = put_user(prev_input_path, (POWER_IC_AUD_IN_PATH_T*)arg);
                break;
                
            case POWER_IC_IOCTL_AUDIO_WRITE_IGAIN:
                ret_val = power_ic_audio_write_igain((int) val);
                break;

            case POWER_IC_IOCTL_AUDIO_READ_INGAIN:
                ret_val = power_ic_audio_read_igain( (int*) (&val));
                if(ret_val != 0)
                {
                    return ret_val;
                }
                tracemsg(_k_d("The input gain is %d"),(int)val);
                ret_val = put_user(val, (unsigned int *)arg);
                break;
                
            case POWER_IC_IOCTL_AUDIO_WRITE_OGAIN:
                tracemsg(_k_d("The output gain is %d"),(int)val);
                ret_val = power_ic_audio_write_ogain((int) val);
                break;

            case POWER_IC_IOCTL_AUDIO_READ_OGAIN:
                ret_val = power_ic_audio_read_ogain( (int*) (&val));
                if(ret_val != 0)
                {
                    return ret_val;
                }
                tracemsg(_k_d("The output gain is %d"),(int)val);
                ret_val =  put_user(val,(unsigned int*)arg);
                break;
                
            case POWER_IC_IOCTL_AUDIO_CODEC_EN:
                tracemsg(_k_d("Enable/disable codec %d"), (int)val);
                ret_val = power_ic_audio_codec_en((bool) val);
                break;
                
            case POWER_IC_IOCTL_AUDIO_ST_DAC_EN:
                tracemsg(_k_d("Enable/disable ST DAC %d"), (int)val);
                ret_val = power_ic_audio_st_dac_en((bool) val);
                break;
                
            case POWER_IC_IOCTL_AUDIO_SET_CODEC_SM:
                tracemsg(_k_d("Set up the codec master/slave bit %d"),(int)val); 
                ret_val = power_ic_audio_set_codec_sm((bool) val);
                break;
                
            case POWER_IC_IOCTL_AUDIO_SET_ST_DAC_SM:
                tracemsg(_k_d("Set up the ST DAC master/slave bit %d"),(int)val); 
                ret_val = power_ic_audio_set_st_dac_sm((bool) val);
                break;

            case POWER_IC_IOCTL_AUDIO_SET_ST_DAC_SR:
                tracemsg(_k_d("Set up the ST DAC sample rate %d"),(POWER_IC_ST_DAC_SR_T)val);
                ret_val = power_ic_audio_set_st_dac_sr((POWER_IC_ST_DAC_SR_T ) val);
                break;
                
            case POWER_IC_IOCTL_AUDIO_SET_CODEC_SR:
                tracemsg(_k_d("Set up the CODEC sample rate %d"),(POWER_IC_CODEC_SR_T)val);
                ret_val = power_ic_audio_set_codec_sr((POWER_IC_CODEC_SR_T) val);
                break;
  
            case POWER_IC_IOCTL_AUDIO_CONFIG_CODEC_LP:
                tracemsg(_k_d("Set up the loopback %d"),(bool)val); 
                ret_val = power_ic_audio_config_loopback((bool)val);
                break;
                
            case POWER_IC_IOCTL_AUDIO_SET_OUTPUT_HP:
                tracemsg(_k_d("Set up the output HP %d"),(int)val);
                ret_val = power_ic_audio_set_output_hp((bool) val);
                break;

            case POWER_IC_IOCTL_AUDIO_SET_INPUT_HP:
                tracemsg(_k_d("Set up the input HP %d"),(int)val);
                ret_val =  power_ic_audio_set_input_hp((bool) val);
               break;

            case POWER_IC_IOCTL_AUDIO_DITHER_EN:
                tracemsg(_k_d("Set up the dither bit %d"),(int)val);
                ret_val = power_ic_audio_dither_en((bool) val);
                break;
               
            case POWER_IC_IOCTL_AUDIO_SET_ST_DAC_NET_MODE:
                tracemsg(_k_d("Set up the network mode %d"),(int)val);
                ret_val = power_ic_audio_set_st_dac_net_mode((int) val);
                break;
                
            case POWER_IC_IOCTL_AUDIO_SET_ST_DAC_NUM_TS:
                tracemsg(_k_d("Set up the time slot %d"),(int)val);
                ret_val = power_ic_audio_set_st_dac_num_ts((int) val);    
                break;
                
            case POWER_IC_IOCTL_AUDIO_POWER_OFF:
                tracemsg(_k_d("Power off audio"));
                ret_val = power_ic_audio_power_off();
                break;
                
            case POWER_IC_IOCTL_AUDIO_ST_DAC_RESET:
                tracemsg(_k_d("Resetting ST DAC\n"));
                ret_val = power_ic_audio_st_dac_reset();
                break;
                
            case POWER_IC_IOCTL_AUDIO_CODEC_RESET:
                tracemsg(_k_d("Resetting CODEC\n"));
                ret_val = power_ic_audio_codec_reset();
                break;
                
        default :
                tracemsg(_k_d("%d unsupported audio ioctl command"),(int) cmd);
                return -ENOTTY;
        }
        return ret_val;
}


