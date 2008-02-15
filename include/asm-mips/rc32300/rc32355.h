/*
 *
 * BRIEF MODULE DESCRIPTION
 *	Definitions for IDT RC32355 CPU.
 *
 * Copyright 2002 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *         	stevel@mvista.com or source@mvista.com
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

#ifndef _RC32355_H_
#define _RC32355_H_

#include <linux/delay.h>
#include <asm/io.h>

/* Base address of internal registers */
#define RC32355_REG_BASE   0x18000000

/* System ID Registers */
#define CPU_SYSID          (RC32355_REG_BASE + 0x00018)
#define CPU_BTADDR         (RC32355_REG_BASE + 0x0001c)
#define CPU_REV            (RC32355_REG_BASE + 0x0002c)

/* Reset Controller */
#define RESET_CNTL         (RC32355_REG_BASE + 0x08000)

/* Device Controller */
#define DEV0_BASE          (RC32355_REG_BASE + 0x10000)
#define DEV0_MASK          (RC32355_REG_BASE + 0x10004)
#define DEV0_CNTL          (RC32355_REG_BASE + 0x10008)
#define DEV0_TIMING        (RC32355_REG_BASE + 0x1000c)
#define DEV_REG_OFFSET     0x10

/* SDRAM Controller */
#define SDRAM0_BASE        (RC32355_REG_BASE + 0x18000)
#define SDRAM0_MASK        (RC32355_REG_BASE + 0x18004)
#define SDRAM1_BASE        (RC32355_REG_BASE + 0x18008)
#define SDRAM1_MASK        (RC32355_REG_BASE + 0x1800c)
#define SDRAM_CNTL         (RC32355_REG_BASE + 0x18010)

/* Bus Arbiter */
#define BUS_ARB_CNTL0      (RC32355_REG_BASE + 0x20000)
#define BUS_ARB_CNTL1      (RC32355_REG_BASE + 0x20004)

/* Counters/Timers */
#define TIMER0_COUNT       (RC32355_REG_BASE + 0x28000)
#define TIMER0_COMPARE     (RC32355_REG_BASE + 0x28004)
#define TIMER0_CNTL        (RC32355_REG_BASE + 0x28008)
#define TIMER_REG_OFFSET   0x0C

/* System Integrity */

/* Interrupt Controller */
#define IC_GROUP0_PEND     (RC32355_REG_BASE + 0x30000)
#define IC_GROUP0_MASK     (RC32355_REG_BASE + 0x30004)
#define IC_GROUP_OFFSET    0x08

#define NUM_INTR_GROUPS    5
/*
 * The IRQ mapping is as follows:
 *
 *    IRQ         Mapped To
 *    ---     -------------------
 *     0      SW0  (IP0) SW0 intr
 *     1      SW1  (IP1) SW1 intr
 *     -      Int0 (IP2) mapped to GROUP0_IRQ_BASE
 *     -      Int1 (IP3) mapped to GROUP1_IRQ_BASE
 *     -      Int2 (IP4) mapped to GROUP2_IRQ_BASE
 *     -      Int3 (IP5) mapped to GROUP3_IRQ_BASE
 *     -      Int4 (IP6) mapped to GROUP4_IRQ_BASE
 *     7      Int5 (IP7) CP0 Timer
 *
 * IRQ's 8 and up are all mapped to Int0-4 (IP2-IP6), which
 * internally on the RC32355 is routed to the Expansion
 * Interrupt Controller.
 */
#define MIPS_CPU_TIMER_IRQ 7

#define GROUP0_IRQ_BASE  8                      // Counter/Timers, UCW
#define GROUP1_IRQ_BASE  (GROUP0_IRQ_BASE + 6)  // DMA
#define GROUP2_IRQ_BASE  (GROUP1_IRQ_BASE + 16) // ATM
#define GROUP3_IRQ_BASE  (GROUP2_IRQ_BASE + 10) // TDM, Eth, USB, UARTs, I2C
#define GROUP4_IRQ_BASE  (GROUP3_IRQ_BASE + 24) // GPIO

#define RC32355_NR_IRQS  (GROUP4_IRQ_BASE + 32)

/* DMA - see rc32355_dma.h for full list of registers */

#define RC32355_DMA_BASE (RC32355_REG_BASE + 0x38000)
#define DMA_CHAN_OFFSET  0x14

/* GPIO Controller */

/* TDM Bus */

/* 16550 UARTs */
#ifdef __MIPSEB__
#define RC32300_UART0_BASE (RC32355_REG_BASE + 0x50003)
#define RC32300_UART1_BASE (RC32355_REG_BASE + 0x50023)
#else
#define RC32300_UART0_BASE (RC32355_REG_BASE + 0x50000)
#define RC32300_UART1_BASE (RC32355_REG_BASE + 0x50020)
#endif
#define RC32300_BASE_BAUD  (IDT_BUS_FREQ * 1000 * 1000 / 16)

#define RC32300_UART0_IRQ  GROUP3_IRQ_BASE + 14
#define RC32300_UART1_IRQ  GROUP3_IRQ_BASE + 17

/* ATM */

/* Ethernet - see rc32355_eth.h for full list of registers */

#define RC32355_ETH_BASE   (RC32355_REG_BASE + 0x60000)

#endif /* _RC32355_H_ */
