/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2004-2006 Motorola, Inc. All Rights Reserved.
 */

/* 
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 * Motorola 2004-Dec-06 - Redesign of the core functions for power ic 
 *
 * Motorola 2006-Jan-10 - Finalize the design of the core functions for power ic.
 */


/*!
 * @file core.c
 *
 * @ingroup poweric_core
 *
 * @brief The main file and user-space interface to the power IC driver.
 *
 * This file includes all of the initialization and tear-down code for the power IC
 * low-level driver and the basic user-mode interface (open(), close(), ioctl(), etc.).
 *
 * @todo For Atlas, does power control need to be set for software override dual-path
 * like AUL and EMU one-chip? On other hardware, allowing the hardware to control this
 * results in ambulances when a charger is attached while the battery is low.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/config.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

#include <linux/power_ic.h>
#include <linux/lights_funlights.h>

#include <linux/moto_accy.h>

#include "os_independent.h"
#include "event.h"
#ifdef CONFIG_MOT_POWER_IC_PCAP2
#include "spi_main.h"
#include "pcap_register.h"
#include "eoc_register.h"
#include "fl_register.h"
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#include "atlas_register.h"
#include "/vobs/jem/hardhat/linux-2.6.x/drivers/mxc/atlas/core/atlas_spi_inter.h"
#endif /* ATLAS */

#ifdef CONFIG_MOT_POWER_IC_PCAP2
#include "../module/charger.h"
#include "../module/power_management.h"
#include "../module/tcmd_ioctl.h"
#include "../module/touchscreen.h"
#endif /* PCAP2 */

#include "../module/emu.h"
#include "../module/debounce.h"
#include "../module/rtc.h"
#include "../module/peripheral.h"
#include "../module/audio.h"
/*! LCD initialization function protottype */
extern void pxafb_init(void);

#include "../module/atod.h"

/******************************************************************************
* Constants
******************************************************************************/
 
/*! Number of registers and value pairs in #tab_init_reg */
#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define NUM_INIT_REGS 30
#define USB2V0_MASK 0x00004
#define IDF_MASK    0x00010
#define IDG_MASK    0x00020
#define SE1_MASK    0x00040
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define NUM_INIT_REGS 38
#define USB2V0_MASK 0x0010000
#define IDF_MASK    0x0040000
#define IDG_MASK    0x0080000
#define SE1_MASK    0x0100000
#endif /* ATLAS */

/******************************************************************************
* Local Variables
******************************************************************************/

/*! Set of initialization values written to the power IC during initialization.
 *  Note: charger power control register 0 cannot have a fixed initialisation 
 *  here - this is done through charger_init() instead. */
static const unsigned int tab_init_reg[NUM_INIT_REGS][2] = 
{
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    {POWER_IC_REG_PCAP_IMR,            0x1FFFFFF},
    {POWER_IC_REG_PCAP_INT_SEL,        0x0000000},
    {POWER_IC_REG_PCAP_SWCTRL,         0x0002EDE},
    {POWER_IC_REG_PCAP_VREG1,          0x15778E3},
    {POWER_IC_REG_PCAP_VREG2,          0x0810234}, 

    {POWER_IC_REG_PCAP_AUX_VREG,       0x1024BEC}, 
    {POWER_IC_REG_PCAP_BATT_DAC,       0x0228E00},
    {POWER_IC_REG_PCAP_ADC1,           0x0200000},
    {POWER_IC_REG_PCAP_AUD_CODEC,      0x0000800},
    {POWER_IC_REG_PCAP_RX_AUD_AMPS,    0x0030000},
    
    {POWER_IC_REG_PCAP_ST_DAC,         0x0099700},
    {POWER_IC_REG_PCAP_PWRCTRL,        0x0094108},
    {POWER_IC_REG_PCAP_BUSCTRL,        0x00002A0},
#ifdef CONFIG_ARCH_EZX_BP_CONTROL_SW2
    {POWER_IC_REG_PCAP_AUX_VREG_MASK,  0x0214D48},
#else
    {POWER_IC_REG_PCAP_AUX_VREG_MASK,  0x0214D40},
#endif /* CONFIG_ARCH_EZX_BP_CONTROL_SW2 */
    {POWER_IC_REG_PCAP_LOWPWR_CTRL,    0x1D9610C},

    {POWER_IC_REG_PCAP_PERIPH_MASK,    0x0000000},
    {POWER_IC_REG_PCAP_TX_AUD_AMPS,    0x000E000},
    {POWER_IC_REG_PCAP_GP,             0x0000107},
    {POWER_IC_REG_EOC_INT_MASK,        0x0000FFF},

   
    {POWER_IC_REG_EOC_POWER_CONTROL_0, 0x0080000},  

    {POWER_IC_REG_EOC_POWER_CONTROL_1, 0x0000006},
    {POWER_IC_REG_EOC_CONN_CONTROL,    0x0020060},
    {POWER_IC_REG_FL_INPUT1,           0x0000000},
    {POWER_IC_REG_FL_REGISTER1,        0x0000000},    
    {POWER_IC_REG_FL_PSC0,             0x0000000},

    {POWER_IC_REG_FL_PWM0,             0x0000080},
    {POWER_IC_REG_FL_PSC1,             0x0000000},
    {POWER_IC_REG_FL_PWM1,             0x0000080},
    {POWER_IC_REG_FL_LS0,              0x0000000},
    {POWER_IC_REG_FL_LS1,              0x0000000}
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    {POWER_IC_REG_ATLAS_INT_MASK_0,        0x0FFFFFF},      
    {POWER_IC_REG_ATLAS_INT_MASK_1,        0x0FFFFFF},      
    {POWER_IC_REG_ATLAS_SEMAPHORE,         0x0000000},         
    {POWER_IC_REG_ATLAS_ARB_PERIPH_AUDIO,  0x0001000},  
    {POWER_IC_REG_ATLAS_ARB_SWITCHERS,     0x0000000},
      
    {POWER_IC_REG_ATLAS_ARB_REG_0,         0x0000000}, 
    {POWER_IC_REG_ATLAS_ARB_REG_1,         0x0000000}, 
    {POWER_IC_REG_ATLAS_PWR_CONTROL_0,     0x0CA0C43},
    {POWER_IC_REG_ATLAS_PWR_CONTROL_1,     0x000A005},
    {POWER_IC_REG_ATLAS_PWR_CONTROL_2,     0x0000000},
     
    {POWER_IC_REG_ATLAS_REGEN_ASSIGN,      0x0000000},    
    {POWER_IC_REG_ATLAS_SWITCHERS_0,       0x000431C}, 
    {POWER_IC_REG_ATLAS_SWITCHERS_1,       0x000431C},
    {POWER_IC_REG_ATLAS_SWITCHERS_2,       0x0024924},
    {POWER_IC_REG_ATLAS_SWITCHERS_3,       0x0024924},
    
    {POWER_IC_REG_ATLAS_SWITCHERS_4,       0x0036749},     
    {POWER_IC_REG_ATLAS_SWITCHERS_5,       0x0132709},
    {POWER_IC_REG_ATLAS_REG_SET_0,         0x006FEEC},
    {POWER_IC_REG_ATLAS_REG_SET_1,         0x0000FFC}, 
    {POWER_IC_REG_ATLAS_REG_MODE_0,        0x06791FF},     
    
    {POWER_IC_REG_ATLAS_REG_MODE_1,        0x003F209},   
    {POWER_IC_REG_ATLAS_PWR_MISC,          0x0000000},  
    {POWER_IC_REG_ATLAS_AUDIO_RX_0,        0x0003000}, 
    {POWER_IC_REG_ATLAS_AUDIO_RX_1,        0x000D35A},  
    {POWER_IC_REG_ATLAS_AUDIO_TX,          0x0420008},  
    
    {POWER_IC_REG_ATLAS_SSI_NETWORK,       0x0013060},  
    {POWER_IC_REG_ATLAS_AUDIO_CODEC,       0x0180027},  
    {POWER_IC_REG_ATLAS_AUDIO_STEREO_DAC,  0x00E0004},
    {POWER_IC_REG_ATLAS_ADC_0,             0x0000020},      
    {POWER_IC_REG_ATLAS_CHARGER_0,         0x0080000},
        
    {POWER_IC_REG_ATLAS_USB_0,             0x0000060},
    {POWER_IC_REG_ATLAS_CHARGE_USB_1,      0x000000E},  
    {POWER_IC_REG_ATLAS_LED_CONTROL_0,     0x0000001}, 
    {POWER_IC_REG_ATLAS_LED_CONTROL_1,     0x0000000},  
    {POWER_IC_REG_ATLAS_LED_CONTROL_2,     0x0000000},    
    
    {POWER_IC_REG_ATLAS_LED_CONTROL_3,     0x000002A},
    {POWER_IC_REG_ATLAS_LED_CONTROL_4,     0x000002A}, 
    {POWER_IC_REG_ATLAS_LED_CONTROL_5,     0x000002A}
#endif /* ATLAS */
};

/*! Holds the power-up reason (determined at module initialization) */
static POWER_IC_POWER_UP_REASON_T power_up_reason = POWER_IC_POWER_UP_REASON_NONE;

/******************************************************************************
* Local Functions
******************************************************************************/


/*!
 * @brief determines power-up reason
 *
 * This function determines the reason that the phone powered up.  This
 * determination is done by reading the contents of the interrupt status
 * and sense registers in the power IC(s).
 *
 * @return power-up reason
 */

static POWER_IC_POWER_UP_REASON_T determine_power_up_reason (void)
{
    POWER_IC_POWER_UP_REASON_T retval = POWER_IC_POWER_UP_REASON_NONE;
    int i;
    int value;

    /* Table of power-up reasons */
    static const struct
    {
        POWER_IC_REGISTER_T reg;
        int bit;
        int value;
        POWER_IC_POWER_UP_REASON_T reason;
    } reasons[] =
    {
        /* Reasons are listed in priority order with the highest priority power-up reason first */
#ifdef CONFIG_MOT_POWER_IC_PCAP2
        /* register                    bit value  reason */
        { POWER_IC_REG_PCAP_PSTAT,     7,    0, POWER_IC_POWER_UP_REASON_FIRST_POWER_KEY_LONG },
        { POWER_IC_REG_PCAP_ISR,       7,    1, POWER_IC_POWER_UP_REASON_FIRST_POWER_KEY_SHORT },
        { POWER_IC_REG_PCAP_PSTAT,     8,    0, POWER_IC_POWER_UP_REASON_SECOND_POWER_KEY_LONG },
        { POWER_IC_REG_PCAP_ISR,       8,    1, POWER_IC_POWER_UP_REASON_SECOND_POWER_KEY_SHORT },
        { POWER_IC_REG_PCAP_ISR,       14,   1, POWER_IC_POWER_UP_REASON_POWER_CUT },
        { POWER_IC_REG_PCAP_ISR,       5,    1, POWER_IC_POWER_UP_REASON_ALARM },
        { POWER_IC_REG_EOC_INT_STATUS, 0,    1, POWER_IC_POWER_UP_REASON_CHARGER },
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)        
        { POWER_IC_REG_ATLAS_INT_SENSE_1,     3,    0, POWER_IC_POWER_UP_REASON_FIRST_POWER_KEY_LONG },
        { POWER_IC_REG_ATLAS_INT_STAT_1,      3,    1, POWER_IC_POWER_UP_REASON_FIRST_POWER_KEY_SHORT },
        { POWER_IC_REG_ATLAS_INT_SENSE_1,     4,    0, POWER_IC_POWER_UP_REASON_SECOND_POWER_KEY_LONG },
        { POWER_IC_REG_ATLAS_INT_STAT_1,      4,    1, POWER_IC_POWER_UP_REASON_SECOND_POWER_KEY_SHORT },
        { POWER_IC_REG_ATLAS_INT_SENSE_1,     5,    0, POWER_IC_POWER_UP_REASON_THIRD_POWER_KEY_LONG },
        { POWER_IC_REG_ATLAS_INT_STAT_1,      5,    1, POWER_IC_POWER_UP_REASON_THIRD_POWER_KEY_SHORT },
        { POWER_IC_REG_ATLAS_INT_STAT_1,      8,    1, POWER_IC_POWER_UP_REASON_POWER_CUT },
        { POWER_IC_REG_ATLAS_INT_STAT_1,      1,    1, POWER_IC_POWER_UP_REASON_ALARM },
        { POWER_IC_REG_ATLAS_INT_SENSE_0,     6,    1, POWER_IC_POWER_UP_REASON_CHARGER }
#endif /* ATLAS */
    };

    /* Loop through the reasons (in order) to find the power-up reason */
    for (i = 0; i < sizeof(reasons) / sizeof(reasons[0]); i++)
    {
        /* Read the bit from the power IC.  If set, we have found the reason */
        if ((power_ic_get_reg_value (reasons[i].reg, reasons[i].bit, &value, 1) == 0) &&
            (value == reasons[i].value))
        {
            retval = reasons[i].reason;
            break;
        }
    }

    /* Return the reason that we found (or will default to none if we didn't find anything) */
    return retval;
}
/*!
 * @brief determines power path
 *
 * This function determines the power path at power up.  This
 * determination is done by reading the contents of the
 * sense registers in the power IC(s).
 *
 * @return none
 */

static void determines_power_path(void)
{
    int value;
    /* We need to read out 2V VBUS, ID_FLOAT, ID_GND, SE1 bits from sense register.*/
#ifdef CONFIG_MOT_POWER_IC_PCAP2    
    if(power_ic_read_reg(POWER_IC_REG_EOC_INT_SENSE,&value)==0)     
    {
        /*For Charger and factory cable, the path is set to dual path. FETOVRD = 1,FETCTRL = 0*/
        if(((value & (USB2V0_MASK|IDF_MASK|SE1_MASK)) == (USB2V0_MASK|SE1_MASK))||
           ((value & (USB2V0_MASK|IDF_MASK|IDG_MASK|SE1_MASK)) == (USB2V0_MASK|IDF_MASK|IDG_MASK)))
        {
            tracemsg(_k_d("Core: the power path is dual path"));         
            power_ic_set_reg_mask(POWER_IC_REG_EOC_POWER_CONTROL_0,0x0000400,0x0000400);

        }
        else
        {
            tracemsg(_k_d("Core: the power path is share current"));
            power_ic_set_reg_mask(POWER_IC_REG_EOC_POWER_CONTROL_0,0x0000C00,0x0000C00);
        }
    }
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    if(power_ic_read_reg(POWER_IC_REG_ATLAS_INT_SENSE_0,&value)==0)
    {
        /*For Charger and factory cable, the path is set to dual path. FETOVRD = 1,FETCTRL = 0*/
        if(((value & (USB2V0_MASK|IDF_MASK|SE1_MASK)) == (USB2V0_MASK|SE1_MASK))||
           ((value & (USB2V0_MASK|IDF_MASK|IDG_MASK|SE1_MASK)) == (USB2V0_MASK|IDF_MASK|IDG_MASK)))
        {
            tracemsg(_k_d("Core: the power path is dual path"));  
            power_ic_set_reg_mask(POWER_IC_REG_ATLAS_CHARGER_0,0x0000400,0x0000400);
        }
        else
        {
           tracemsg(_k_d("Core: the power path is share current"));
           power_ic_set_reg_mask(POWER_IC_REG_ATLAS_CHARGER_0,0x0000C00,0x0000C00);
        }
    }
#endif        
}
/*!
 * @brief Retrieves information about the hardware.
 *
 * This function populates a POWER_IC_HARDWARE_T structure with information about the hardware
 * of the phone. This includes the type of the power IC hardware (PCAP/Atlas/whatever) and some
 * revision information about the hardware, if available.
 *
 * @param        info       Pointer to structure to be populated with hardware information.
 */

static void get_hardware_info(POWER_IC_HARDWARE_T * info)
{
    int revision;
    
    info->revision1 = POWER_IC_HARDWARE_NO_REV_INFO;
    info->revision2 = POWER_IC_HARDWARE_NO_REV_INFO;
    
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    info->chipset = POWER_IC_CHIPSET_PCAP2_EOC;
    
    /* There is no revision information in PCAP's register set. However, there is
     * some information available for EMU one-chip. */
    if(power_ic_get_reg_value(POWER_IC_REG_EOC_INT_SENSE, 10, &revision, 3) == 0)
    {
        info->revision2 = revision;
    }

#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    info->chipset = POWER_IC_CHIPSET_ATLAS;
    
    /* Atlas has revision information available. There is currently no other identifiable
     * hardware alongside Atlas. */
    if(power_ic_read_reg(POWER_IC_REG_ATLAS_REVISION, &revision) == 0)
    {
        info->revision1 = revision;
    }

#else /* Don't know what this is. */
    info->chipset = POWER_IC_CHIPSET_UNKNOWN;
#endif
}

/*!
 * @brief the ioctl() handler for the power IC device node
 *
 * This function implements the ioctl() system call for the power IC device node.
 * Based on the provided command, the function will pass the request on to the correct
 * power IC module for handling.  In the case that the command one of the "core" power
 * IC requests (i.e., read/write register commands), the command is handled directly
 * by calling the appropriate function (with any required user/kernel-space conversions).
 *
 * @param        inode       inode pointer
 * @param        file        file pointer
 * @param        cmd         the ioctl() command
 * @param        arg         the ioctl() argument
 *
 * @return 0 if successful
 */

static int power_ic_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    POWER_IC_REG_ACCESS_T reg_access;
    POWER_IC_HARDWARE_T hardware_info;
    int * usr_spc_data_ptr = NULL; 
    int retval;
    
    /* Get the actual command from the ioctl request. */
    unsigned int cmd_num = _IOC_NR(cmd);
    
    if((cmd_num >= POWER_IC_IOC_CMD_CORE_BASE) && (cmd_num <= POWER_IC_IOC_CMD_CORE_LAST_CMD))
    {
        tracemsg(_k_d("core ioctl(), request 0x%X (cmd %d)"),(int) cmd, (int)cmd_num);

        /* All of the register access functions share the same data, so handle it commonly */
        if ((cmd != POWER_IC_IOCTL_GET_POWER_UP_REASON) &&
            (cmd != POWER_IC_IOCTL_GET_HARDWARE_INFO)   &&
            (cmd != POWER_IC_IOCTL_GET_POWER_UP_REASON) &&
            (cmd != POWER_IC_IOCTL_TIMING_SHOW_INFO)    &&
            (cmd != POWER_IC_IOCTL_TIMING_RESET))
        {
            /* Fetch the data passed from user space. Access OK checking is done for us. */
            if(copy_from_user((void *)&reg_access, (void *)arg, sizeof(reg_access)) != 0)
            {
                tracemsg(_k_d("error copying data from user space."));
                return -EFAULT;
            }

            usr_spc_data_ptr = &(((POWER_IC_REG_ACCESS_T *)arg)->value);
        }

        /* Handle the request. */
        switch(cmd)
        {
            case POWER_IC_IOCTL_READ_REG:
                /* Read the entire register and pass the data back the the caller in user space. */
                tracemsg(_k_d("=> Read register %d"), reg_access.reg);              
                
                retval = power_ic_read_reg(reg_access.reg, &(reg_access.value));
                if (retval != 0)
                {
                    return retval;
                }
                
                /* Only the value read needs to be sent back to the caller. */
                if(put_user(reg_access.value, usr_spc_data_ptr) != 0)
                {
                    tracemsg(_k_d("error copying read value to user space."));
                    return -EFAULT;
                }
                break;
                
            case POWER_IC_IOCTL_WRITE_REG:
                /* Write the entire register with the data provided. No data needs to be passed
                 * back to user-space. */
                tracemsg(_k_d("=> Write register %d, value 0x%X"), reg_access.reg, reg_access.value);
                
                retval = power_ic_write_reg(reg_access.reg, &(reg_access.value));
                if (retval != 0)
                {
                    return retval;
                }

                break;
                
            case POWER_IC_IOCTL_READ_REG_BITS:
                /* Read the bits requested and pass the data back the the caller in user space. */
                tracemsg(_k_d("=> Read register bits - reg %d, index %d, num bits %d"),
                               reg_access.reg, reg_access.index, reg_access.num_bits);
                
                retval = power_ic_get_reg_value(reg_access.reg, reg_access.index, 
                                                &(reg_access.value), reg_access.num_bits);
                if (retval != 0)
                {
                    return retval;
                }
                
                /* Only the value read needs to be sent back to the caller. */
                if(put_user(reg_access.value, usr_spc_data_ptr) != 0)
                {
                    tracemsg(_k_d("error copying read bits to user space."));
                    return -EFAULT;
                }
                break;
                
            case POWER_IC_IOCTL_WRITE_REG_BITS:
                /* Write the bits specified. No data needs to be passed back to user-space. */
                tracemsg(_k_d("=> Write register bits - reg %d, index %d, num bits %d, value 0x%X"),
                               reg_access.reg, reg_access.index, 
                               reg_access.num_bits, reg_access.value);
                
                retval = power_ic_set_reg_value(reg_access.reg, reg_access.index, 
                                                reg_access.value, reg_access.num_bits);
                if (retval != 0)
                {
                    return retval;
                }
                    
                break;
                
            case POWER_IC_IOCTL_WRITE_REG_MASK:
                /* Write the data specified. No data needs to be passed back to user-space. */
                tracemsg(_k_d("=> Write register mask - reg %d, mask %d, value 0x%X"),
                               reg_access.reg, reg_access.index, reg_access.value);
                
                retval = power_ic_set_reg_mask(reg_access.reg, reg_access.index, reg_access.value);
                if (retval != 0)
                {
                    return retval;
                }
                    
                break;            

            case POWER_IC_IOCTL_GET_POWER_UP_REASON:
                /* Copy the power-up reason to user-space */
                retval = copy_to_user ((void *)arg, (void *)&power_up_reason,
                    sizeof(power_up_reason));

                /* If the copy failed, return an error */
                if (retval != 0)
                {
                    return -EFAULT;
                }

                break;
                
            case POWER_IC_IOCTL_GET_HARDWARE_INFO:
                /* The caller wants to know about the hardware. Go get the necessary information. */
                get_hardware_info(&hardware_info);
            
                /* Copy the hardware info over to user-space */
                retval = copy_to_user ((void *)arg, (void *)&hardware_info,
                    sizeof(hardware_info));

                /* If the copy failed, return an error */
                if (retval != 0)
                {
                    return -EFAULT;
                }

                break;           
                
            case POWER_IC_IOCTL_TIMING_SHOW_INFO:
                /* This is just a placeholder for now. Call various get timing info
                 * functions from here. Arg is a bool that indicates whether the full
                 * set of timing info should be displayed (true) or a summary should
                 * be shown (false), which can be passed to the output functions in the
                 * timing module. */
                tracemsg(_k_d("Timing info not supported in this kernel build."));
                return -ENOTTY;
                break;
                
            case POWER_IC_IOCTL_TIMING_RESET:
                /* This is just a placeholder for now. Call various get timing info
                 * reset functions from here. */
                tracemsg(_k_d("Timing info not supported in this kernel build."));
                return -ENOTTY;
                break;    

            default: /* This shouldn't be able to happen, but just in case... */
                tracemsg(_k_d("0x%X unsupported ioctl command"), (int) cmd);
                return -ENOTTY;
                break;
        }
    }

    /* Is this a request for the RTC interface? */
    else if((cmd_num >= POWER_IC_IOC_RTC_BASE) && (cmd_num <= POWER_IC_IOC_RTC_LAST_CMD))
    {
        return rtc_ioctl(cmd, arg);
    }
    
    /* Is this a request for the peripheral interface? */
    else if((cmd_num >= POWER_IC_IOC_CMD_PERIPH_BASE) && (cmd_num <= POWER_IC_IO_CMD_PERIPH_LAST_CMD))
    {
        return periph_ioctl(cmd, arg);
    }

    /*audio interface module*/
    else if((cmd_num >= POWER_IC_IOC_CMD_AUDIO_BASE) && (cmd_num <= POWER_IC_IOC_CMD_AUDIO_LAST_CMD))
    {
        return audio_ioctl(cmd,arg);
    }
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    /* This will handle the request that address the power management module*/
    else if((cmd_num >= POWER_IC_IOC_PMM_BASE) && (cmd_num <= POWER_IC_IOC_PMM_LAST_CMD))
    { 
        return pmm_ioctl(cmd, arg);
    }
#endif /* PCAP */
    /* Is this a request for the AtoD interface? */
    else if((cmd_num >= POWER_IC_IOC_ATOD_BASE) && (cmd_num <= POWER_IC_IOC_ATOD_LAST_CMD))
    {
        return atod_ioctl(cmd, arg);
    }    

    /* Lights request */
    else if((cmd_num >= POWER_IC_IOC_LIGHTS_BASE) && (cmd_num <= POWER_IC_IOC_LIGHTS_LAST_CMD))
    {
        return lights_ioctl(cmd,arg);
    }
#ifdef CONFIG_MOT_POWER_IC_PCAP2    
    /* Charger control request */
    else if((cmd_num >= POWER_IC_IOC_CMD_CHARGER_BASE) && (cmd_num <= POWER_IC_IOC_CHARGER_LAST_CMD))
    {
        return charger_ioctl(cmd, arg);
    }
    
    /* Touchscreen control request */
    else if((cmd_num >= POWER_IC_IOC_CMD_TOUCHSCREEN_BASE) && (cmd_num <= POWER_IC_IOC_TOUCHSCREEN_LAST_CMD))
    {
        return touchscreen_ioctl(cmd, arg);
    }
    
    /* This will handle the request that address test commands*/
    else if((cmd_num >= POWER_IC_IOC_CMD_TCMD_BASE) && (cmd_num <= POWER_IC_IOC_CMD_TCMD_LAST_CMD))
    { 
        return tcmd_ioctl(cmd, arg);
    }
    
    /* ioctl handling for other modules goes here... */
#endif /* PCAP2 */
     
    else /* The driver doesn't support this request. */
    {
        tracemsg(_k_d("0x%X unsupported ioctl command"), (int) cmd);
        return -ENOTTY;
    }
    
    return 0;
}

/*!
 * @brief the open() handler for the power IC device node
 *
 * This function implements the open() system call on the power IC device.  Currently,
 * this function does nothing.
 *
 * @param        inode       inode pointer
 * @param        file        file pointer
 *
 * @return 0
 */

static int power_ic_open(struct inode *inode, struct file *file) 
{
    tracemsg(_k_d("Power IC: open()"));
    return 0;
}

/*!
 * @brief the close() handler for the power IC device node
 *
 * This function implements the close() system call on the power IC device.   Currently,
 * this function does nothing.
 *
 * @param        inode       inode pointer
 * @param        file        file pointer
 *
 * @return 0
 */

static int power_ic_free(struct inode *inode, struct file *file)
{
    tracemsg(_k_d("Power IC: free()"));
    return 0;
}

#ifdef CONFIG_MOT_POWER_IC_PCAP2
/*!
 * @brief Initializes the PCAP registers
 *
 * Using the register initialization table, #tab_init_reg, this function initializes
 * all of the required PCAP registers.
 *
 */

static void __init initialize_pcap_registers (void)
{
    int i;

    for (i = 0; i < NUM_INIT_REGS; i++)
    {
        if (POWER_IC_REGISTER_IS_PCAP(tab_init_reg[i][0]))
        {
            power_ic_write_reg_value (tab_init_reg[i][0], tab_init_reg[i][1]);
        }
    }
}

/*!
 * @brief initialzes the EMU One Chip registers
 *
 * Using the register initialization table, #tab_init_reg, this function initializes
 * all of the required EMU One Chip registers.
 *
 */

static void initialize_eoc_registers (void)
{
    int i;

    for (i = 0; i < NUM_INIT_REGS; i++)
    {
        if (POWER_IC_REGISTER_IS_EOC(tab_init_reg[i][0]))
        {
            power_ic_write_reg_value (tab_init_reg[i][0], tab_init_reg[i][1]);
        }
    }
}

/*!
 * @brief initialzes the Funlight registers
 *
 * Using the register initialization table, #tab_init_reg, this function initializes
 * all of the required Funlight registers.
 *
 */
static void initialize_fl_registers (void)
{
    int i;

    for (i = 0; i < NUM_INIT_REGS; i++)
    {
        if (POWER_IC_REGISTER_IS_FL(tab_init_reg[i][0]))
        {
            power_ic_write_reg_value(tab_init_reg[i][0], tab_init_reg[i][1]);
        }
    }
}
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
/*!
 * @brief initialzes the Atlas registers
 *
 * Using the register initialization table, #tab_init_reg, this function initializes
 * all of the required Atlas registers.
 *
 */
static void __init initialize_atlas_registers (void)
{
    int i;
                      
    for (i = 0; i < NUM_INIT_REGS; i++)
    {
        if (POWER_IC_REGISTER_IS_ATLAS(tab_init_reg[i][0]))
        {
            power_ic_write_reg_value (tab_init_reg[i][0], tab_init_reg[i][1]);
        }
    }
}
#endif /* ATLAS */

/*! This structure defines the file operations for the power IC device */
static struct file_operations poweric_fops =
{
    .owner =    THIS_MODULE,
    .ioctl =    power_ic_ioctl,
    .open =     power_ic_open,
    .release =  power_ic_free,
    .poll =     atod_poll,
    .read =     atod_read
};

/******************************************************************************
* Global Functions
******************************************************************************/

/*!
 * @brief power IC initialization function
 *
 * This function implements the initialization function of the power IC low-level
 * driver.  The function is called during system initialization or when the module
 * is loaded (if compiled as a module).  It is responsible for performing the
 * power IC register initialization, initializing the power IC interrupt (event)
 * handling, and performing any initialization required for the handling the
 * power IC device node.
 *
 * @return 0
 */

int __init power_ic_init(void)
{
    int ret;
    
    tracemsg ("Power IC driver init");

    ret = register_chrdev(POWER_IC_MAJOR_NUM, POWER_IC_DEV_NAME, &poweric_fops);
    if (ret < 0)
    {
        tracemsg(_k_d("unable to get a major (%d) for power ic"), (int)POWER_IC_MAJOR_NUM);
        return ret;
    }

#ifdef CONFIG_ARCH_SCMA11   
    devfs_mk_cdev(MKDEV(POWER_IC_MAJOR_NUM,0), S_IFCHR | S_IRUGO | S_IWUSR, "power_ic");
#endif

#ifdef CONFIG_MOT_POWER_IC_PCAP2
    /* Initialize the spi management queue and hardware. */
    spi_initialize();

    /* Initialize the EMU One Chip I2C interface */
    eoc_initialize(initialize_eoc_registers);

    /* Initialize all PCAP registers */
    initialize_pcap_registers();

    /* Initialize the funlights driver */
    fl_initialize(initialize_fl_registers);
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    spi_init();
    
    /* Initialize the ATLAS driver */
    initialize_atlas_registers();
    
#endif /* ATLAS */
   
    /* Before initializing events or any sub-modules, determine the power-up reason */
    power_up_reason = determine_power_up_reason();

    /*Determine the power mode */
    determines_power_path();
    /* Initialize the event handling system */
    power_ic_event_initialize();
     
    /* Initialize the AtoD interface. */
    atod_init();
    
    /* Initialize the power ic debounce thread */
    power_ic_debounce_init();
    tracemsg ("Power IC driver init complete\n");

    /* Initialize the accessory driver */
    moto_accy_init();
    
    /* Initialize the EMU accessory detection driver */
    emu_init();
    
    tracemsg ("EMU accessory detection driver init complete\n");

    /*Set fun lights mode on */
    lights_fl_set_control(LIGHTS_FL_APP_CTL_DEFAULT,LIGHTS_FL_ALL_REGIONS_MSK);
    
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    /* Set the power path, etc. */
    charger_init();
    
    /* Initialize Touchscreen Driver */
    touchscreen_init();
     
    /* Initialize the LCD driver */
    pxafb_init();

#endif /* PCAP2 */

    return 0;
}

/*!
 * @brief power IC cleanup function
 *
 * This function implements the exit function of the power IC low-level driver.
 * The function is called when the kernel is being taken down or when the module
 * is unloaded (if compiled as a module).  It is currently responsible for
 * unregistering the power IC character device and calling the tear-down function
 * for each of the modules that require tear-down.
 */

static void __exit power_ic_exit(void)
{
    unregister_chrdev(POWER_IC_MAJOR_NUM, POWER_IC_DEV_NAME);

    /* Tear-down the accessory driver */
    moto_accy_destroy();

    tracemsg("Power IC core: successfully unloaded\n");
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*
 * Module entry points
 */
module_init(power_ic_init);
module_exit(power_ic_exit);

MODULE_DESCRIPTION("Power IC char device driver");
MODULE_AUTHOR("Motorola/FreeScale");
MODULE_LICENSE("GPL");
#endif
