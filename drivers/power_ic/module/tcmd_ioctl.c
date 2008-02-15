/*
 * /vobs/ezx_linux/code/linux/linux-2.4.17/drivers/power_ic/module/tcmd_ioctl.c
 *
 * Description - This module will contain all of the ioctl commands that are going
 * to be specifically used by the TCMD.
 *
 * Copyright (C) 2005 Motorola, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 * 2005-Apr-29 - Design of the ioctl commands for TCMD. 
 */
 
 /*!
 * @file tcmd_ioctl.c
 *
 * @ingroup poweric_tcmd_ioctl
 * 
 * @brief TCMD specific ioctls.
 *
 * This module will contain all of the ioctl commands that are going
 * to be specifically used by the TCMD.  <strong>Since some of these ioctls
 * can over-ride the current signal paths in ways that are not valid 
 * for normal operation these should only be used when the phone is 
 * suspended. </strong>
 *
 */

#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/errno.h>
#include "emu.h"
#include "emu_eoc.h"
#include "tcmd_ioctl.h"

#include "../core/os_independent.h"


/******************************************************************************
* Constants
******************************************************************************/ 

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/* PCAP2 Defined parameters */
#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define MONO_ADDER_REG       POWER_IC_REG_PCAP_RX_AUD_AMPS    
#define MONO_ADDER_INDEX     19
#define IDFLOATS_REG         POWER_IC_REG_EOC_INT_SENSE
#define IDFLOATS_INDEX       4
#define IDGNDS_REG           POWER_IC_REG_EOC_INT_SENSE    
#define IDGNDS_INDEX         5
#define A1SNS_REG            POWER_IC_REG_PCAP_PSTAT    
#define A1SNS_INDEX          12
#define MB2SNS_REG           POWER_IC_REG_PCAP_PSTAT    
#define MB2SNS_INDEX         11
#define VUSBIN_REG           POWER_IC_REG_EOC_POWER_CONTROL_1   
#define VUSBIN_INDEX         0
#define CLKSTAT_REG          POWER_IC_REG_PCAP_PSTAT    
#define CLKSTAT_INDEX        17
#define COINCHEN_REG         POWER_IC_REG_PCAP_BATT_DAC    
#define COINCHEN_INDEX       15 

#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define MONO_ADDER_REG       POWER_IC_REG_ATLAS_AUDIO_RX_1
#define MONO_ADDER_INDEX     16
#define IDFLOATS_REG         POWER_IC_REG_ATLAS_INT_SENSE_0    
#define IDFLOATS_INDEX       19
#define IDGNDS_REG           POWER_IC_REG_ATLAS_INT_SENSE_0    
#define IDGNDS_INDEX         20    
/* The A1SNS settings below correspond to HSDETI bit for Headset attach 
 * not addressed is the ATLAS HSLI bit 19 for Stereo headset detect  */
#define A1SNS_REG            POWER_IC_REG_ATLAS_INT_STAT_1    
#define A1SNS_INDEX          18
#define MB2SNS_REG           POWER_IC_REG_ATLAS_INT_STAT_1    
#define MB2SNS_INDEX         17
#define VUSBIN_REG           POWER_IC_REG_ATLAS_CHARGE_USB_1    
#define VUSBIN_INDEX         0
#define CLKSTAT_REG          POWER_IC_REG_ATLAS_INT_SENSE_1    
#define CLKSTAT_INDEX        14
#define COINCHEN_REG         POWER_IC_REG_ATLAS_PWR_CONTROL_0    
#define COINCHEN_INDEX       23 

#endif /* ATLAS */


#define MONO_ADDER_NUM_BITS  2
#define VUSBIN_NUM_BITS      2    

#endif /* For DOXYGEN_SHOULD_SKIP_THIS */

/******************************************************************************
* Local type definitions
******************************************************************************/

/******************************************************************************
* Exported Symbols
******************************************************************************/

/******************************************************************************
* Global functions.
*******************************************************************************/


/*!
 * @brief ioctl() handler for the TCMD interface.
 *
 * This function is the ioctl() interface handler for all TCMD ioctl operations. It is 
 * not called directly through an ioctl() call on the power IC device, but is executed 
 * from the core ioctl handler for all ioctl requests in the range for TCMD operations.
 *
 * @param        cmd       ioctl() request received.
 * @param        arg       additional information about request, specific to each request.
 *
 * @return 0 if successful.
 */
 
int tcmd_ioctl(unsigned int cmd, unsigned long arg)
{       
    unsigned int val= arg;
    int ret_val = 0;
    
    tracemsg(_k_d("TCMD specific ioctl(), request 0x%X (cmd 0x%X)"),(int) cmd, _IOC_NR(cmd));

    /* Handle the request. */
    switch(cmd)  {
        case POWER_IC_IOCTL_CMD_TCMD_EMU_TRANS_STATE:
            /* Configures the transeiver to one states in EMU_XCVR_T */
            tracemsg(_k_d("Configure the transceiver to %d"),(int)val);
            emu_configure_usb_xcvr((int)arg);
            break;
 
        case POWER_IC_IOCTL_CMD_TCMD_MONO_ADDER_STATE:
            /* Sets the MONO adder to one of the states in POWER_IC_TCMD_MONO_ADDER_T */
            tracemsg(_k_d("Set the state of the MONO adder bits to %d"),(int)val);
            if ( val >= POWER_IC_TCMD_MONO_ADDER_NUM)
            {
                tracemsg(_k_d("MONO ADDER - error requested state out of range"));
                ret_val = -EFAULT;
                break;
            }  
            ret_val = (power_ic_set_reg_value(MONO_ADDER_REG, MONO_ADDER_INDEX,
                                            val, MONO_ADDER_NUM_BITS)); 
            break;
	    
        case POWER_IC_IOCTL_CMD_TCMD_IDFLOATS_READ:
            /* Read the bits requested and pass the data back the the caller in user space. */
            tracemsg(_k_d("Read IDFLOATS bit"));
            ret_val = power_ic_get_reg_value(IDFLOATS_REG, IDFLOATS_INDEX, 
                                                ( (int*) (&val)), 1);
            if(ret_val == 0)
            {
                tracemsg(_k_d("The IDFLOAT bit is %d"),(int)val);
                ret_val = put_user(val,(unsigned int*)arg);
            }
            break;
	    
        case POWER_IC_IOCTL_CMD_TCMD_IDGNDS_READ:
            /* Read the bits requested and pass the data back the the caller in user space. */
            tracemsg(_k_d("Read IDGNDS bit"));
            ret_val = power_ic_get_reg_value(IDGNDS_REG, IDGNDS_INDEX, 
                                               ( (int*) (&val)), 1);
            if(ret_val == 0)
            {
                tracemsg(_k_d("The IDGNDS bit is %d"),(int)val);
                ret_val = put_user(val,(unsigned int*)arg);
            }
            break;
	    
        case POWER_IC_IOCTL_CMD_TCMD_A1SNS_READ:
            /* Read the bits requested and pass the data back the the caller in user space. */
            tracemsg(_k_d("Read A1SNS bit"));
            ret_val = power_ic_get_reg_value(A1SNS_REG, A1SNS_INDEX, 
                                                ( (int*) (&val)), 1);
            if(ret_val == 0)
            {
                tracemsg(_k_d("The A1SNS bit is %d"),(int)val);
                ret_val = put_user(val,(unsigned int*)arg);
            }
            break;
	    
        case POWER_IC_IOCTL_CMD_TCMD_MB2SNS_READ:
            /* Read the bits requested and pass the data back the the caller in user space. */
            tracemsg(_k_d("Read MB2SNS bit"));
            ret_val = power_ic_get_reg_value(MB2SNS_REG, MB2SNS_INDEX, 
                                                ( (int*) (&val)), 1);
            if(ret_val == 0)
            {
                tracemsg(_k_d("The MB2SNS bit is %d"),(int)val);
                ret_val = put_user(val,(unsigned int*)arg);
            }
            break;
	    
        case POWER_IC_IOCTL_CMD_TCMD_REVERSE_MODE_STATE:
            /* Passing 1 sets the reverse mode to on passing 0 sets it to off */
            ret_val = EMU_SET_REVERSE_MODE((int)arg);
            break;

        case POWER_IC_IOCTL_CMD_TCMD_VUSBIN_READ:
            /* Read the bits requested and pass the data back the the caller in user space. */
            tracemsg(_k_d("Read VUSBIN bits"));
            ret_val = power_ic_get_reg_value(VUSBIN_REG, VUSBIN_INDEX, 
                                                ( (int*) (&val)), VUSBIN_NUM_BITS);
            if(ret_val == 0)
            {
                tracemsg(_k_d("The VUSBIN bits are %d"),(int)val);
                ret_val = put_user(val,(unsigned int*)arg);
            }
            break;

        case POWER_IC_IOCTL_CMD_TCMD_VUSBIN_STATE:
            /* Set the bits that determine the source of the VUSB input */
            tracemsg(_k_d("Set the VUSBIN bits to %d"),(int)val);
            ret_val = (power_ic_set_reg_value(VUSBIN_REG, VUSBIN_INDEX, val, VUSBIN_NUM_BITS)); 
            break;

        case POWER_IC_IOCTL_CMD_TCMD_CLKSTAT_READ:
            /* Read the bits requested and pass the data back the the caller in user space. */
            tracemsg(_k_d("Read CLKSTAT bit"));
            ret_val = power_ic_get_reg_value(CLKSTAT_REG, CLKSTAT_INDEX, 
                                                ( (int*) (&val)), 1);
            if(ret_val == 0)
            {
                tracemsg(_k_d("The CLKSTAT bit is %d"),(int)val);
                ret_val = put_user(val,(unsigned int*)arg);
            }
            break;

        case POWER_IC_IOCTL_CMD_TCMD_COINCHEN_STATE:
            /* Passing 1 Enables the charging of the coincell passing 0 sets it to no charge */
            tracemsg(_k_d("Set the COINCHEN bit to %d"),(int)val);
            ret_val = (power_ic_set_reg_bit(COINCHEN_REG, COINCHEN_INDEX,(int)val)); 
            break;

        case POWER_IC_IOCTL_CMD_TCMD_EMU_CONN_STATE:
            tracemsg(_k_d("Configure the connection mode to %d"),(int)val);
            if ( val >= EMU_CONN_MODE__NUM)
            {
                tracemsg(_k_d("EMU CONN STATE - error requested state out of range"));
                ret_val = -EFAULT;
                break;
            }  
            EMU_SET_EMU_CONN_MODE((int)arg);
            break;

        default: /* This shouldn't be able to happen, but just in case... */
            tracemsg(_k_d("=> 0x%X unsupported tcmd ioctl command"), (int) cmd);
            ret_val = -ENOTTY;
            break;
    
    }
    return ret_val;
}

