/*
 *  linux/include/asm-arm/arch-ti925/irqs.h
 *
 *  Copyright (C) 1997,1998 Russell King
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
#ifndef __ASM_ARCH_TI925_IRQS_H
#define __ASM_ARCH_TI925_IRQS_H
/* ---------------------------------------------------------------------------
 *  Interrupts
 * ---------------------------------------------------------------------------
 */

/* 
 *  Interrupt numbers
 * 
 */
#define INT_SINT0                     0
#define INT_SINT1                     1
#define INT_SINT2                     2
#define INT_SINT3                     3
#define INT_SINT4                     4
#define INT_SINT5                     5
#define INT_SINT6                     6
#define INT_SINT7                     7
#define INT_TIN0                      9
#define INT_TIN1                      9
#define INT_TC1                      11
#define INT_EXT_INT1                 12
#define INT_RTC1                     14 
#define INT_RTC2                     15
#define INT_CLKMOD                   16
#define INT_LCD_CTRL                 17
#define INT_UART1                    18
#define INT_UART2                    19
#define INT_EXT_INT2                 20
#define INT_GPIO                     21

#define MAX_INT_BASE                 31
/* 
 * Interrupt numbers for the HIO interrupt controller
 * inside the interrupt code, they are added to the MAX_INT_BASE.  Thus,
 * to see the INT_HIO_PID interrupt, the IRQ number is 31 + 6 = 37
 */
#define INT_HIO_ATN                   0
#define INT_HIO_ACK                   1 
#define INT_HIO_DSPS                  2
#define INT_HIO_PIA                   3
#define INT_HIO_PIB                   4
#define INT_HIO_PIC                   5 
#define INT_HIO_PID                   6
#define INT_HIO_PEN                   7
#define INT_HIO_DEG                   8
#define INT_HIO_SER                   9
#define INT_HIO_PER                   10
#define INT_HIO_LINT                  11
#define INT_HIO_LSR                   12
#define INT_HIO_ETHR                  13
#define INT_HIO_UART1                 14
#define INT_HIO_UART2                 15

#define MAX_INT_HIO_BASE              15

/* 
 *  Interrupt bit positions
 * 
 */
#define INTMASK_SINT0                 (1 << INT_SINT0)
#define INTMASK_SINT1                 (1 << INT_SINT1)
#define INTMASK_SINT2                 (1 << INT_SINT2)
#define INTMASK_SINT3                 (1 << INT_SINT3)
#define INTMASK_SINT4                 (1 << INT_SINT4)
#define INTMASK_SINT5                 (1 << INT_SINT5)
#define INTMASK_SINT6                 (1 << INT_SINT6)
#define INTMASK_SINT7                 (1 << INT_SINT7)
#define INTMASK_TIN0                  (1 << INT_TIN0)
#define INTMASK_TIN1                  (1 << INT_TIN1)
#define INTMASK_TC1                   (1 << INT_TC1)
#define INTMASK_EXT_INT1              (1 << INT_EXT_INT1)
#define INTMASK_RTC1                  (1 << INT_RTC1)
#define INTMASK_RTC2                  (1 << INT_RTC2)
#define INTMASK_CLKMOD                (1 << INT_CLKMOD)
#define INTMASK_LCD                   (1 << INT_LCD)
#define INTMASK_UART1                 (1 << INT_UART1)
#define INTMASK_UART2                 (1 << INT_UART2)
#define INTMASK_EXT_INT2              (1 << INT_EXT_INT2)
#define INTMASK_GPIO                  (1 << INT_GPIO)

/* 
 *  TI925_CM_INT0      - Interrupt number of first CM interrupt
 *  TI925_SC_VALID_INT - Mask of valid system controller interrupts
 */
#define TI925_CM_INT0              INT_CM_SOFTINT
#define TI925_SC_VALID_INT         0xffffffff

#define MAXIRQNUM                       (MAX_INT_HIO_BASE + MAX_INT_BASE) 
#define MAXFIQNUM                       MAXIRQNUM
#define MAXSWINUM                       MAXIRQNUM

#define NR_IRQS                         (MAXIRQNUM + 2)
 
#endif

