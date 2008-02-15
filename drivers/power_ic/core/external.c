/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2004-2005 Motorola, Inc. All Rights Reserved.
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
 * Motorola 2004-Dec-06 - Rewrote the kernel-level interface to access power IC registers.
 *
 * Motorola 2005-Aug-24 - Finalize the interface.
 */

/*!
 * @file external.c
 *
 * @ingroup poweric_core
 *
 * @brief The kernel-level interface to access power IC registers
 *
 * The functions in this file implement the kernel-level interface for reading
 * and writing power IC registers.  Low-level register access to the power IC is
 * controlled using a spin lock and disabled bottom halves.  In our system, the
 * spin lock will compile away (since this is a uni-processor system), but the
 * disabling of bottom halves is still critical and will stay.  This will ensure
 * that only one execution thread will ever be accessing a power IC register at
 * any time.  While this is not so important for the read functions, it becomes
 * critical for the read-modify-write functions.
 *
 * A RAM copy of the power IC registers is maintained.  This read functions will
 * always read the value of the register directly from the power IC instead of
 * using the RAM copy, but the RAM copy will be updated with whatever is read.
 * The write functions use the RAM copy to get the value of the bits in the
 * register that are not being explicitly changed.
 *
 * @todo The spin locks have been compiled out for now because of problems in the
 * I2C driver.  The I2C driver can only run from task context because it sleeps
 * and schedule() will refuse to sleep when bottom halves are disabled.  So, when
 * the I2C driver gets fixed to not sleep, the spin locks should be added back.
 */

#include <linux/power_ic.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <asm/softirq.h>

#include "os_independent.h"
#include "event.h"
#ifdef CONFIG_MOT_POWER_IC_PCAP2
#include "pcap_register.h"
#include "eoc_register.h"
#include "fl_register.h"
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#include "atlas_register.h"
#endif /* ATLAS */

#ifndef DOXYGEN_SHOULD_SKIP_THIS
EXPORT_SYMBOL(power_ic_read_reg);
EXPORT_SYMBOL(power_ic_write_reg);
EXPORT_SYMBOL(power_ic_write_reg_value);
EXPORT_SYMBOL(power_ic_set_reg_bit);
EXPORT_SYMBOL(power_ic_set_reg_value);
EXPORT_SYMBOL(power_ic_get_reg_value);
EXPORT_SYMBOL(power_ic_set_reg_mask);
#endif

/******************************************************************************
* Constants
******************************************************************************/
/*! No need to read before writing to the register and no need to read from cache. */
#define NO_SPECIAL_READ          0x00  
/*! Must read before writing to the register. */
#define READ_BEFORE_WRITE        0x01  
/*! Read the register contents from cache instead of hardware. */
#define READ_FROM_CACHE          0x02
/*! Must read before writing to the register and must read from cache. */
#define READ_CACHE_BEFORE_WRITE  0x03

#ifdef CONFIG_MOT_POWER_IC_PCAP2
#define POWER_IC_REG_NUM_REGS POWER_IC_REG_NUM_REGS_PCAP
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
#define POWER_IC_REG_NUM_REGS POWER_IC_REG_NUM_REGS_ATLAS
#endif

/******************************************************************************
* Local Variables
******************************************************************************/

/*! @brief RAM copies of all power IC registers */
static unsigned int power_ic_registers[POWER_IC_REG_NUM_REGS];

/*!
 * @brief Masks indicating which bits are maintained in the RAM copy of each register
 *
 * A '1' bit indicates that the corresponding bit in the power IC register will
 * not be saved in the RAM copy of the register.  That is, all bits that can change
 * autonomously in the power IC should be set to '1's.
 */
static const unsigned int power_ic_register_no_write_masks[POWER_IC_REG_NUM_REGS] =
{
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    /* PCAP Registers */
    0x01FFFFFF, /* ISR */
    0x01840000, /* IMR */
    0x01FFFFFF, /* PSTAT */
    0x01840000, /* INT_SEL */
    0x01000001, /* SWCTRL */
    0x00000000, /* VREG1 */
    0x00000001, /* VREG2 */
    0x00000001, /* AUX_VREG */
    0x00010000, /* BATT_DAC */
    0x00000000, /* ADC1 */
    0x010FFFFF, /* ADC2 */
    0x00000000, /* AUD_CODEC */
    0x00000000, /* RX_AUD_AMPS */
    0x00F00000, /* ST_DAC */
    0x01FFFFFF, /* RTC_TOD */
    0x00000000, /* RTC_TODA */
    0x01FFFFFF, /* RTC_DAY */
    0x00000000, /* RTC_DAYA */
    0x00000000, /* MTRTMR */
    0x00000000, /* PWRCTRL */
    0x00FFE000, /* BUSCTRL */
    0x00000000, /* PERIPH */
    0x01C00001, /* AUX_VREG_MASK */
    0x01FFFFFF, /* VENDOR_REV */
    0x00000000, /* LOWPWR_CTRL */
    0x01F57716, /* PERIPH_MASK */
    0x00070000, /* TX_AUD_AMPS */
    0x00000000, /* GP_REG */
    0x01C00000, /* TEST1 */
    0x01FFE000, /* TEST2 */
    0x01FFFFFF, /* VENDOR_TEST */
    0x01FFFFFF, /* VENDOR_TEST */

    /* EMU Registers */
    0x00FFFFFF, /* Interrupt status */
    0x00FFF000, /* Interrupt mask */
    0x00FFFFFF, /* Interrupt sense */
    0x00F3C000, /* Power control 0 */
    0x00FFFFC0, /* Power control 1 */
    0x00000000, /* Connectivity control */
   
    /* Funlight Registers */
    0x00000000, /* LED0-7 Input Register */
    0x00000000, /* Register 1...no one cares */
    0x00000000, /* Frequency Prescaler 0 */
    0x00000000, /* PWM Register 0 */
    0x00000000, /* Frequency Prescaler 1 */
    0x00000000, /* PWM Register 1 */
    0x00000000, /* LED0-3 Selector */
    0x00000000, /* LED4-7 Selector */
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    /* Atlas Registers */
    0x0FFFFFF,  /* Interrupt Status 0 */
    0x0160020,  /* Interrupt Mask 0 */
    0x0FFFFFF,  /* Interrupt Sense 0 */
    0x0FFFFFF,  /* Interrupt Status 1 */
    0x0C10004,  /* Interrupt Mask 1 */
    0x0FFFFFF,  /* Interrupt Sense 1 */
    0x0FFFFFF,  /* Power Up Mode Sense */
    0x0FFFFFF,  /* Revision */
    0x0000000,  /* Semaphore */
    0x0F80000,  /* Arbitration Peripheral Audio */
    0x0FE0000,  /* Arbitration Switchers */
    0x0000000,  /* Arbitration Regulators 0 */
    0x0C03C00,  /* Arbitration Regulators 1 */
    0x0000000,  /* Power Control 0 */
    0x0E00000,  /* Power Control 1 */
    0x0FFF000,  /* Power Control 2 */
    0x000C000,  /* Regen Assignment */
    0x0FFFFFF,  /* Control Spare */
    0x0000000,  /* Memory A */
    0x0000000,  /* Memory B */
    0x0FFFFFF,  /* RTC Time */
    0x0000000,  /* RTC Alarm */
    0x0FFFFFF,  /* RTC Day */
    0x0000000,  /* RTC Day Alarm */
    0x0FC0000,  /* Switchers 0 */
    0x0FC0000,  /* Switchers 1 */
    0x0FC0000,  /* Switchers 2 */
    0x0FC0000,  /* Switchers 3 */ 
    0x0C00030,  /* Switchers 4 */
    0x0800030,  /* Switchers 5 */
    0x0F80003,  /* Regulator Setting 0 */
    0x0FFF000,  /* Regulator Setting 1 */
    0x0000000,  /* Regulator Mode 0 */
    0x0000000,  /* Regulator Mode 1 */
    0x0FE003F,  /* Power Miscellaneous */
    0x0FFFFFF,  /* Power Spare */
    0x0100000,  /* Audio Rx 0 */
    0x0C00000,  /* Audio Rx 1 */
    0x0000010,  /* Audio Tx */
    0x0E00003,  /* SSI Network */
    0x0E00000,  /* Audio Codec */
    0x0E06400,  /* Audio Stereo DAC */
    0x0FFFFFF,  /* Audio Spare */
    0x0780300,  /* ADC 0 */
    0x0000004,  /* ADC 1 */
    0x0FFFFFF,  /* ADC 2 */
    0x07F8000,  /* ADC 3 */
    0x0FFFFFF,  /* ADC 4 */
    0x0F05000,  /* Charger 0 */
    0x0000000,  /* USB 0 */
    0x0FFFF10,  /* USB 1 */
    0x0010400,  /* LED Control 0 */
    0x0780000,  /* LED Control 1 */
    0x0000000,  /* LED Control 2 */
    0x0000000,  /* LED Control 3 */
    0x0000000,  /* LED Control 4 */
    0x0000000,  /* LED Control 5 */
    0x0FFFFFF,  /* Spare */
    0x0FFFFFF,  /* Trim 0 */
    0x0FFFFFF,  /* Trim 1 */
    0x0FFFFFF,  /* Test 0 */
    0x0FFFFFF,  /* Test 1*/
    0x0FFFFFF,  /* Test 2 */
    0x0FFFFFF   /* Test 3 */
#endif /* ATLAS */
};

/*! @brief Table that defines which registers need to be read before written 
 *  
 *  A '1' in bit 0 will indicate that the register should be read before being written and
 *  a '1' in bit 1 will indicate that the register should always be read from cache instead
 *  of from the hardware. */
static const unsigned char read_before_write[POWER_IC_REG_NUM_REGS] =
{
#ifdef CONFIG_MOT_POWER_IC_PCAP2
    /* PCAP Registers */
    NO_SPECIAL_READ,   /* ISR */
    NO_SPECIAL_READ,   /* IMR */
    NO_SPECIAL_READ,   /* PSTAT */
    READ_FROM_CACHE,   /* INT_SEL */
    READ_FROM_CACHE,   /* SWCTRL */
    READ_FROM_CACHE,   /* VREG1 */
    READ_FROM_CACHE,   /* VREG2 */
    READ_FROM_CACHE,   /* AUX_VREG */
    NO_SPECIAL_READ,   /* BATT_DAC */
    READ_BEFORE_WRITE, /* ADC1 */
    READ_BEFORE_WRITE, /* ADC2 */
    READ_FROM_CACHE,   /* AUD_CODEC */
    READ_FROM_CACHE,   /* RX_AUD_AMPS */
    READ_FROM_CACHE,   /* ST_DAC */
    READ_BEFORE_WRITE, /* RTC_TOD */
    READ_FROM_CACHE,   /* RTC_TODA */
    READ_BEFORE_WRITE, /* RTC_DAY */
    READ_FROM_CACHE,   /* RTC_DAYA */
    READ_FROM_CACHE,   /* MTRTMR */
    READ_FROM_CACHE,   /* PWRCTRL */
    READ_FROM_CACHE,   /* BUSCTRL */
    READ_FROM_CACHE,   /* PERIPH */
    READ_FROM_CACHE,   /* AUX_VREG_MASK */
    NO_SPECIAL_READ,   /* VENDOR_REV */
    READ_FROM_CACHE,   /* LOWPWR_CTRL */
    READ_FROM_CACHE,   /* PERIPH_MASK */
    READ_FROM_CACHE,   /* TX_AUD_AMPS */
    READ_FROM_CACHE,   /* GP_REG */
    NO_SPECIAL_READ,   /* TEST1 */
    NO_SPECIAL_READ,   /* TEST2 */
    NO_SPECIAL_READ,   /* VENDOR_TEST */
    NO_SPECIAL_READ,   /* VENDOR_TEST */

    /* EMU Registers */
    NO_SPECIAL_READ,   /* Interrupt status */
    NO_SPECIAL_READ,   /* Interrupt mask */
    NO_SPECIAL_READ,   /* Interrupt sense */
    NO_SPECIAL_READ,   /* Power control 0 */
    NO_SPECIAL_READ,   /* Power control 1 */
    NO_SPECIAL_READ,   /* Connectivity control */

    /* Funlight Registers */
    READ_FROM_CACHE,   /* LED0-7 Input Register */ 
    READ_FROM_CACHE,   /* Register 1...no one cares about this one */
    READ_FROM_CACHE,   /* Frequency Prescaler 0 */
    READ_FROM_CACHE,   /* PWM Register 0 */
    READ_FROM_CACHE,   /* Frequency Prescaler 1 */
    READ_FROM_CACHE,   /* PWM Register 1 */
    READ_FROM_CACHE,   /* LED0-3 Selector */
    READ_FROM_CACHE,   /* LED4-7 Selector */
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    /* Atlas Registers */
    READ_BEFORE_WRITE,   /* Interrupt Status 0 */
    READ_BEFORE_WRITE,   /* Interrupt Mask 0 */
    READ_BEFORE_WRITE,   /* Interrupt Sense 0 */
    READ_BEFORE_WRITE,   /* Interrupt Status 1 */
    READ_BEFORE_WRITE,   /* Interrupt Mask 1 */
    READ_BEFORE_WRITE,   /* Interrupt Sense 1 */
    READ_BEFORE_WRITE,   /* Power Up Mode Sense */
    READ_BEFORE_WRITE,   /* Revision */
    READ_BEFORE_WRITE,   /* Semaphore */
    READ_BEFORE_WRITE,   /* Arbitration Peripheral Audio */
    READ_BEFORE_WRITE,   /* Arbitration Switchers */
    READ_BEFORE_WRITE,   /* Arbitration Regulators 0 */
    READ_BEFORE_WRITE,   /* Arbitration Regulators 1 */
    READ_BEFORE_WRITE,   /* Power Control 0 */
    READ_BEFORE_WRITE,   /* Power Control 1 */
    READ_BEFORE_WRITE,   /* Power Control 2 */
    READ_BEFORE_WRITE,   /* Regen Assignment */
    READ_BEFORE_WRITE,   /* Control Spare */
    READ_BEFORE_WRITE,   /* Memory A */
    READ_BEFORE_WRITE,   /* Memory B */
    READ_BEFORE_WRITE,   /* RTC Time */
    READ_BEFORE_WRITE,   /* RTC Alarm */
    READ_BEFORE_WRITE,   /* RTC Day */
    READ_BEFORE_WRITE,   /* RTC Day Alarm */
    READ_BEFORE_WRITE,   /* Switchers 0 */
    READ_BEFORE_WRITE,   /* Switchers 1 */
    READ_BEFORE_WRITE,   /* Switchers 2 */
    READ_BEFORE_WRITE,   /* Switchers 3 */ 
    READ_BEFORE_WRITE,   /* Switchers 4 */
    READ_BEFORE_WRITE,   /* Switchers 5 */
    READ_BEFORE_WRITE,   /* Regulator Setting 0 */
    READ_BEFORE_WRITE,   /* Regulator Setting 1 */
    READ_BEFORE_WRITE,   /* Regulator Mode 0 */
    READ_BEFORE_WRITE,   /* Regulator Mode 1 */
    READ_BEFORE_WRITE,   /* Power Miscellaneous */
    READ_BEFORE_WRITE,   /* Power Spare */
    READ_BEFORE_WRITE,   /* Audio Rx 0 */
    READ_BEFORE_WRITE,   /* Audio Rx 1 */
    READ_BEFORE_WRITE,   /* Audio Tx */
    READ_BEFORE_WRITE,   /* SSI Network */
    READ_BEFORE_WRITE,   /* Audio Codec */
    READ_BEFORE_WRITE,   /* Audio Stereo DAC */
    READ_BEFORE_WRITE,   /* Audio Spare */
    READ_BEFORE_WRITE,   /* ADC 0 */
    READ_BEFORE_WRITE,   /* ADC 1 */
    READ_BEFORE_WRITE,   /* ADC 2 */
    READ_BEFORE_WRITE,   /* ADC 3 */
    READ_BEFORE_WRITE,   /* ADC 4 */
    READ_BEFORE_WRITE,   /* Charger 0 */
    READ_BEFORE_WRITE,   /* USB 0 */
    READ_BEFORE_WRITE,   /* USB 1 */
    READ_BEFORE_WRITE,   /* LED Control 0 */
    READ_BEFORE_WRITE,   /* LED Control 1 */
    READ_BEFORE_WRITE,   /* LED Control 2 */
    READ_BEFORE_WRITE,   /* LED Control 3 */
    READ_BEFORE_WRITE,   /* LED Control 4 */
    READ_BEFORE_WRITE,   /* LED Control 5 */
    READ_BEFORE_WRITE,   /* Spare */
    READ_BEFORE_WRITE,   /* Trim 0 */
    READ_BEFORE_WRITE,   /* Trim 1 */
    READ_BEFORE_WRITE,   /* Test 0 */
    READ_BEFORE_WRITE,   /* Test 1*/
    READ_BEFORE_WRITE,   /* Test 2 */
    READ_BEFORE_WRITE    /* Test 3 */
#endif /* ATLAS */
};

#if 0
/*! @brief Spin lock to control access to the power IC and associated data structures */
static spinlock_t power_ic_lock = SPIN_LOCK_UNLOCKED;
#endif

/******************************************************************************
* Global Functions
******************************************************************************/

/*!
 * @brief Read an entire register from the power IC
 *
 * This function is used to read an entire register from the power IC.  This read
 * function will always read the value of the register directly from the power IC
 * instead of using the RAM copy, but the RAM copy will be updated with whatever
 * is read.
 *
 * @param        reg        register number
 * @param        reg_value  location to store the register value
 *
 * @return 0 if successful
 */ 

int power_ic_read_reg(POWER_IC_REGISTER_T reg, unsigned int *reg_value) 
{
    unsigned int value;
    int retval = -EINVAL;

#if 0
    /* Lock out access from other tasks or the bottom half interrupt handler */
    spin_lock_bh(&power_ic_lock);
#endif
    
    /* Check to see if we should read directly from hardware or from cache */
    if((read_before_write[reg] & READ_FROM_CACHE) == READ_FROM_CACHE)
    {
        value = power_ic_registers[reg];
        retval = 0;
    }
    else
    {
#ifdef CONFIG_MOT_POWER_IC_PCAP2
        /* Read the register from PCAP If it is a PCAP register */
        if (POWER_IC_REGISTER_IS_PCAP(reg))
        {
            retval = pcap_reg_read (reg - POWER_IC_REG_PCAP_FIRST_REG, &value);
        }

        /* Else, check to see if we need to read from EMU One Chip */
        else if (POWER_IC_REGISTER_IS_EOC(reg))
        {
            retval = eoc_reg_read (reg - POWER_IC_REG_EOC_FIRST_REG, &value);
        }

        /* Else, check to see if we need to read from Funlight Chip */
        else if (POWER_IC_REGISTER_IS_FL(reg))
        {
            retval = fl_reg_read (reg - POWER_IC_REG_FL_FIRST_REG, &value);
        }
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
        /* Read the register from ATLAS */
        if (POWER_IC_REGISTER_IS_ATLAS(reg))
        {
            retval = atlas_reg_read (reg - POWER_IC_REG_ATLAS_FIRST_REG, &value);
        }
#endif /* ATLAS */
    }

    /* If the read was successful, store the value in the RAM copy */
    if (retval == 0)
    {
        power_ic_registers[reg] = value & ~power_ic_register_no_write_masks[reg];

        if (reg_value != NULL)
        {
            *reg_value = value;
        }
    }

#if 0
    /* Release the lock */
    spin_unlock_bh(&power_ic_lock);
#endif
    return retval;
}

/*!
 * @brief Write an entire register to the power IC
 *
 * This function is used to write an entire register to the power IC.  Since
 * the entire register is being written, the RAM copy of the register is not
 * read in this function, but the RAM copy will be updated with the value that
 * is written to the register.
 *
 * @param        reg        register number
 * @param        reg_value  pointer to the new register value
 *
 * @return 0 if successful
 */ 

int power_ic_write_reg(POWER_IC_REGISTER_T reg, unsigned int *reg_value) 
{
    int retval = -EINVAL;

#if 0
    /* Lock out access from other tasks or the bottom half interrupt handler */
    spin_lock_bh(&power_ic_lock);
#endif

#ifdef CONFIG_MOT_POWER_IC_PCAP2
    /* Write the register to PCAP if it is a PCAP register */
    if (POWER_IC_REGISTER_IS_PCAP(reg))
    {
        retval = pcap_reg_write (reg - POWER_IC_REG_PCAP_FIRST_REG, *reg_value);
    }

    /* Else, check to see if an EMU One Chip register should be written */
    else if (POWER_IC_REGISTER_IS_EOC(reg))
    {
        retval = eoc_reg_write (reg - POWER_IC_REG_EOC_FIRST_REG, *reg_value);
    }

    /* Else, check to see if the Funlight register should be written */
    else if (POWER_IC_REGISTER_IS_FL(reg))
    {
        retval = fl_reg_write (reg - POWER_IC_REG_FL_FIRST_REG, *reg_value);
    }
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    /* Write the register to ATLAS if it is an ATLAS register */
    if (POWER_IC_REGISTER_IS_ATLAS(reg))
    {
        retval = atlas_reg_write (reg - POWER_IC_REG_ATLAS_FIRST_REG, *reg_value);
    }
#endif /* ATLAS */
    
    /* If the write was successful, save the new register contents */
    if (retval == 0)
    {
        power_ic_registers[reg] = *reg_value & ~power_ic_register_no_write_masks[reg];
    }

#if 0
    /* Release the lock */
    spin_unlock_bh(&power_ic_lock);
#endif

    return retval;
}

/*!
 * @brief Write an entire register to the power IC
 *
 * This function is used to write an entire register to the power IC.  Since
 * the entire register is being written, the RAM copy of the register is not
 * read in this function, but the RAM copy will be updated with the value that
 * is written to the register.
 *
 * This function differs from power_ic_write_reg() in that it takes the new
 * register value directly instead of expecting a pointer to the new value.
 * The function is simply implemented as a call to power_ic_write_reg().
 *
 * @param        reg        register number
 * @param        reg_value  new register value
 *
 * @return 0 if successful
 */ 

int power_ic_write_reg_value(POWER_IC_REGISTER_T reg, unsigned int reg_value)
{
    return power_ic_write_reg (reg, &reg_value);
}

/*!
 * @brief Set the value of a single bit in a power IC register
 *
 * This function is used to set an individual bit in a power IC register.  The
 * function is implemented simply as a call to power_ic_set_reg_value() with
 * the number of bits parameter set to 1.
 *
 * @param        reg        register number
 * @param        index      bit index to set (0 = least-significant bit)
 * @param        value      new bit value
 *
 * @return 0 if successful
 */ 
 
int power_ic_set_reg_bit(POWER_IC_REGISTER_T reg, int index, int value) 
{
    return power_ic_set_reg_value (reg, index, value ? 1 : 0, 1);
}

/*!
 * @brief Set the value of a range of bits in a power IC register
 *
 * This function is used to set a range of bits in a power IC register.  The
 * function is implemented as a call to power_ic_set_reg_mask() by converting
 * the input parameters index and nb_bits into a bitmask.
 *
 * @param        reg        register number
 * @param        index      starting bit index (0 = least-significant bit)
 * @param        value      new value
 * @param        nb_bits    number of bits to set
 *
 * @return 0 if successful
 */ 
int power_ic_set_reg_value(POWER_IC_REGISTER_T reg, int index, int value, int nb_bits) 
{
    return power_ic_set_reg_mask (reg, (((1 << nb_bits) - 1) << index), value << index);
}

/*!
 * @brief Read the value of a range of bits in a power IC register
 *
 * This function is used to read a contiguous range of bits in a power IC register.
 * The function is implemented by calling power_ic_read_reg() which reads the
 * entire register contents and then masking and shifting the returned register
 * value to match the input parameters.
 *
 * @param        reg        register number
 * @param        index      starting bit index (0 = least-significant bit)
 * @param        value      location to store the read value
 * @param        nb_bits    number of bits to read
 *
 * @return 0 if successful
 */ 
int power_ic_get_reg_value(POWER_IC_REGISTER_T reg, int index, int *value, int nb_bits) 
{
    unsigned int reg_value;
    int retval;

    /* Read the whole register first */
    retval = power_ic_read_reg(reg, &reg_value);

    if (retval == 0 && value != NULL)
    {
        *value = (reg_value >> index) & ((1 << nb_bits) - 1);
    }

    return retval;
}

/*!
 * @brief Set the value of a set of (possibly) non-contiguous bits in a power IC register
 *
 * This function is used to set a possibly non-contiguous set of bits in a power IC
 * register.  This function performs a read-modify-write operation on a power IC register.
 * In the case of those registers that must be physically read before being written (as
 * indicated in the #read_before_write array), the register is read directly from the
 * power IC.  Otherwise, the RAM copy of the register is used as the starting point for
 * the modification.  Using the passed in mask parameter, the set of bits are first
 * cleared and then the new bits are OR'ed into the register.  This new register value
 * is then written out to the power IC and then saved into the RAM copy.
 *
 * This function is the basis for a number of other functions, namely power_ic_set_reg_bit()
 * and power_ic_set_reg_value().  Since all three functions perform nearly the same operation,
 * the implementation was combined into this single generic function which the other two
 * call.
 *
 * @param        reg        register number
 * @param        mask       bitmask indicating which bits are to be modified
 * @param        value      new values for modified bits  
 *
 * @return 0 if successful
 */
int power_ic_set_reg_mask(POWER_IC_REGISTER_T reg, int mask, int value)
{
    int retval = -EINVAL;
    unsigned int old_value;

    /* If the register needs to be read before written, read it now */
    if (read_before_write[reg] & READ_BEFORE_WRITE)
    {
        /*
         * Read the register, but discard the value.  We're only interested in
         * updating the RAM copy right now.
         */
        power_ic_read_reg(reg, &old_value);
    }
    else
    {
        /* Get the RAM copy of the register */
        old_value = power_ic_registers[reg];
    }

#if 0
    /* Lock out access from other tasks or the bottom half interrupt handler */
    spin_lock_bh(&power_ic_lock);
#endif

    /* Clear the bits in question */
    old_value &= ~mask;

    /* Set the new value of the bits */
    old_value |= value & mask;

#ifdef CONFIG_MOT_POWER_IC_PCAP2
    /* Write the bit to PCAP if it is a PCAP register */
    if (POWER_IC_REGISTER_IS_PCAP(reg))
    {
        retval = pcap_reg_write (reg - POWER_IC_REG_PCAP_FIRST_REG, old_value);
    }
    
    /* Else, check to see if the bit should be written to EMU One Chip */
    else if (POWER_IC_REGISTER_IS_EOC(reg))
    {
        retval = eoc_reg_write (reg - POWER_IC_REG_EOC_FIRST_REG, old_value);
    }

    /* Else, check to see if the bit should be written to Funlight Chip */
    else if (POWER_IC_REGISTER_IS_FL(reg))
    {
        retval = fl_reg_write (reg - POWER_IC_REG_FL_FIRST_REG, old_value);
    }
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    /* Write the bit to ATLAS if it is an ATLAS register */
    if (POWER_IC_REGISTER_IS_ATLAS(reg))
    {
        retval = atlas_reg_write (reg - POWER_IC_REG_ATLAS_FIRST_REG, old_value);
    }
#endif /* ATLAS */

    /* If the write was successful, save the new register contents */
    if (retval == 0)
    {
        power_ic_registers[reg] = old_value & ~power_ic_register_no_write_masks[reg];
    }

#if 0
    /* Release the lock */
    spin_unlock_bh(&power_ic_lock);
#endif

    return retval;
}

#ifdef CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT
extern int spi_int_reg_read (int reg, u32 *value_ptr);
extern int spi_int_reg_write (int reg, u32 value);

/*!
 * @brief Set the value of a set of (possibly) non-contiguous bits in a power IC register, even in interrupt context.
 *
 * This function is used to set a possibly non-contiguous set of bits in a power IC
 * register.  This function performs a read-modify-write operation on a power IC register.
 * In the case of those registers that must be physically read before being written (as
 * indicated in the #read_before_write array), the register is read directly from the
 * power IC.  Otherwise, the RAM copy of the register is used as the starting point for
 * the modification.  Using the passed in mask parameter, the set of bits are first
 * cleared and then the new bits are OR'ed into the register.  This new register value
 * is then written out to the power IC and then saved into the RAM copy.
 *
 * This function is the basis for a number of other functions, namely power_ic_set_reg_bit()
 * and power_ic_set_reg_value().  Since all three functions perform nearly the same operation,
 * the implementation was combined into this single generic function which the other two
 * call.
 *
 * @param        reg        register number
 * @param        mask       bitmask indicating which bits are to be modified
 * @param        value      new values for modified bits  
 *
 * @return 0 if successful
 */
int spi_int_set_reg_mask(POWER_IC_REGISTER_T reg, int mask, int value)
{
    int retval = -EINVAL;
    unsigned int old_value;

    /* If the register needs to be read before written, read it now */
    if (read_before_write[reg] & READ_BEFORE_WRITE)
    {
        /*
         * Read the register, but discard the value.  We're only interested in
         * updating the RAM copy right now.
         */
        spi_int_reg_read(reg, &old_value);
    }
    else
    {
        /* Get the RAM copy of the register */
        old_value = power_ic_registers[reg];
    }

#if 0
    /* Lock out access from other tasks or the bottom half interrupt handler */
    spin_lock_bh(&power_ic_lock);
#endif

    /* Clear the bits in question */
    old_value &= ~mask;

    /* Set the new value of the bits */
    old_value |= value & mask;

#ifdef CONFIG_MOT_POWER_IC_PCAP2
    /* Write the bit to PCAP if it is a PCAP register */
    if (POWER_IC_REGISTER_IS_PCAP(reg))
    {
        retval = spi_int_reg_write (reg - POWER_IC_REG_PCAP_FIRST_REG, old_value);
    }
    
    /* Else, check to see if the bit should be written to EMU One Chip */
    else if (POWER_IC_REGISTER_IS_EOC(reg))
    {
        retval = eoc_reg_write (reg - POWER_IC_REG_EOC_FIRST_REG, old_value);
    }

    /* Else, check to see if the bit should be written to Funlight Chip */
    else if (POWER_IC_REGISTER_IS_FL(reg))
    {
        retval = fl_reg_write (reg - POWER_IC_REG_FL_FIRST_REG, old_value);
    }
#elif defined(CONFIG_MOT_POWER_IC_ATLAS)
    /* Write the bit to ATLAS if it is an ATLAS register */
    if (POWER_IC_REGISTER_IS_ATLAS(reg))
    {
        retval = atlas_reg_write (reg - POWER_IC_REG_ATLAS_FIRST_REG, old_value);
    }
#endif /* ATLAS */

    /* If the write was successful, save the new register contents */
    if (retval == 0)
    {
        power_ic_registers[reg] = old_value & ~power_ic_register_no_write_masks[reg];
    }

#if 0
    /* Release the lock */
    spin_unlock_bh(&power_ic_lock);
#endif

    return retval;
}
#endif  /* CONFIG_MOT_POWER_IC_PCAP2_SPI_BUS_INTERRUPT */
