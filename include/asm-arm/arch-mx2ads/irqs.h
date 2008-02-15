/*
 *  linux/include/asm-arm/arch-mx2ads/irqs.h
 *
 *  MX2 Interrupt numbers
 *
 *  Copyright (C) 2004 MontaVista Software Inc.
 *  Copyright (C) 2004 Motorola Semiconductors HK Ltd
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef __ASM_ARCH_IRQS_H__
#define __ASM_ARCH_IRQS_H__

#define INT_GPIO                    8
#define INT_FIRI                    9
#define INT_SDHC2                   10
#define INT_SDHC1                   11
#define INT_I2C                     12
#define INT_SSI2                    13
#define INT_SSI1                    14
#define INT_CSPI2                   15
#define INT_CSPI1              	    16
#define INT_UART4                   17
#define INT_UART3                   18
#define INT_UART2                   19
#define INT_UART1                   20
#define INT_KPP_TX                  21
#define INT_RTC_RX                  22
#define INT_PWM                     23
#define INT_GPT3                    24
#define INT_GPT2                    25
#define INT_GPT1                    26
#define INT_WDOG                    27
#define INT_PCMCIA                  28
#define INT_NFC                     29
#define INT_BMI                     30
#define INT_CSI                     31
#define INT_DMACH0                  32
#define INT_DMACH1                  33
#define INT_DMACH2                  34
#define INT_DMACH3                  35
#define INT_DMACH4                  36
#define INT_DMACH5                  37
#define INT_DMACH6                  38
#define INT_DMACH7                  39
#define INT_DMACH8                  40
#define INT_DMACH9                  41
#define INT_DMACH10                 42
#define INT_DMACH11                 43
#define INT_DMACH12                 44
#define INT_DMACH13                 45
#define INT_DMACH14                 46
#define INT_DMACH15                 47
#define INT_EMMAENC                 49
#define INT_EMMADEC                 50
#define INT_EMMAPRP                 51
#define INT_EMMAPP                  52
#define INT_USBWKUP                 53
#define INT_USBDMA                  54
#define INT_USBHOST                 55
#define INT_USBFUNC                 56
#define INT_USBHNP                  57
#define INT_USBCTRL                 58
#define INT_SAHARA                  59
#define INT_SLCDC                   60
#define INT_LCDC                    61

#define MAXIRQNUM                       62
#define MAXFIQNUM                       62
#define MAXSWINUM                       62

#define NR_IRQS                         (MAXIRQNUM + 1)
#define NR_FIQS				(MAXFIQNUM + 1)

#endif
