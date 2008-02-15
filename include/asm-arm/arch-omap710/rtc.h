/*
 *  linux/include/asm-arm/arch-omap710/rtc.h
 *
 * BRIEF MODULE DESCRIPTION
 *   OMAP710 RTC (Real Time Clock) Support
 *
 * Copyright (C) 2001, 2002 RidgeRun, Inc. (http://www.ridgerun.com)
 * Author: RidgeRun, Inc.
 *         Brian Mahaffy (brianm@ridgerun.com) or info@ridgerun.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
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
 */

#ifndef __ASM_ARCH_710_RTC_H
#define __ASM_ARCH_710_RTC_H
 
#include <asm/io.h>
#include <linux/rtc.h>
#include <asm/arch/sizes.h>
#include <asm/arch/irqs.h>
#include <asm/arch/hardware.h>

extern spinlock_t rtc_lock;

// RTC IRQs
#define RTC_IRQ             INT_RTC_PER
#define RTC_IRQ_ALRM        INT_RTC_ALRM

// Names, Bit Masks, and Bit value Defines for the RTC Registers

// Seconds Reg
#define RTC_SECONDS         OMAP710_RTC_SEC
#define SEC1    0xF0
#define SEC0    0x0F

// Minutes Register
#define RTC_MINUTES         OMAP710_RTC_MIN
#define MIN1    0xF0
#define MIN0    0x0F

// Hours Register
#define RTC_HOURS           OMAP710_RTC_HOUR
#define PM_NAM  0x80
#define AM      0x00
#define PM      0x01
#define HOUR1   0x70
#define HOUR0   0x0F

// Day of Month Register
#define RTC_DAY_OF_MONTH    OMAP710_RTC_DAY
#define DAY1    0xF0
#define DAY0    0x0F

// Month of Year Register
#define RTC_MONTH           OMAP710_RTC_MONTH
#define MONTH1    0xF0
#define MONTH0    0x0F

// Year Register
#define RTC_YEAR            OMAP710_RTC_YEAR
#define YEAR1    0xF0
#define YEAR0    0x0F


// Alarm Seconds Reg
#define RTC_SECONDS_ALARM   OMAP710_RTC_ALRM_SEC
#define ALRM_SEC1    0xF0
#define ALRM_SEC0    0x0F

// Alarm Minutes Register
#define RTC_MINUTES_ALARM   OMAP710_RTC_ALRM_MIN
#define ALRM_MIN1    0xF0
#define ALRM_MIN0    0x0F

// Alarm Hours Register
#define RTC_HOURS_ALARM     OMAP710_RTC_ALRM_HOUR
#define ALRM_PM_NAM  0x80
#define ALRM_HOUR1   0x70
#define ALRM_HOUR0   0x0F

// Alarm Day of Month Register
#define ALRM_DAY1    0xF0
#define ALRM_DAY0    0x0F

// Alarm Month of Year Register
#define ALRM_MONTH1    0xF0
#define ALRM_MONTH0    0x0F

// Alarm Year Register
#define ALRM_YEAR1    0xF0
#define ALRM_YEAR0    0x0F


// Control Register.
#define RTC_CONTROL         OMAP710_RTC_CTRL
#define SLEEP_MODE      1<<6
#define RTC_AWAKE       0
#define RTC_ASLEEP      1<<6

#define SET_32_COUNTER  1<<5
#define SET_32          1<<5

#define TEST_MODE       1<<4

#define RTC_24H         1<<3
#define MODE_12_24      1<<3
#define SET24HR         0
#define SET12HR         1<<3

#define AUTO_COMP       1<<2

#define ROUND_30S       1<<1

#define RTC_SET         1<<0
#define STOP_RTC        1<<0
#define RTC_RUNNING     1<<0
#define RTC_FREEZE      0

// Status Register
#define RTC_FREQ_SELECT     OMAP710_RTC_STAT
#define RTC_INTR_FLAGS      OMAP710_RTC_STAT
#define POWER_UP        1<<7
#define ALARM           1<<6
#define D_EVENT         1<<5
#define H_EVENT         1<<4
#define M_EVENT         1<<3
#define S_EVENT         1<<2
#define RUN             1<<1
#define RTC_UIP         1<<0
#define BUSY            1<<0

// Interrupt Register
#define RTC_INTERRUPT       OMAP710_RTC_INTR

#define IT_ALARM        1<<3
#define RTC_AIE         1<<3

#define IT_TIMER        1<<2
#define RTC_PIE         1<<2

#define EVERY           0x03
#define EVERY_SEC       0x00
#define EVERY_MIN       0x01
#define EVERY_HOUR      0x02
#define EVERY_DAY       0x03

//Compatability macros for the RTC code to minimize changes from the standard Linux driver.
#define RTC_PORT(x) (OMAP710_RTC_SEC + (x))

#define CMOS_READ(addr) inb(addr)

#define CMOS_WRITE(val, addr) outb(val,addr)

/* 
 * The following are for conversion from hex(bin) to BCD that the RTC uses for it's
 * time registers.
 */
#define RTC_ALWAYS_BCD  1
#define RTC_DM_BINARY   0
#ifndef BCD_TO_BIN
#define BCD_TO_BIN(val) ((val)=((val)&15) + ((val)>>4)*10)
#endif
 
#ifndef BIN_TO_BCD
#define BIN_TO_BCD(val) ((val)=(((val)/10)<<4) + (val)%10)
#endif

#endif //__ASM_ARCH_710_RTC_H
