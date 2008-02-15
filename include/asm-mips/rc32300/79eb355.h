/*
 *
 * BRIEF MODULE DESCRIPTION
 *	Definitions for IDT 79EB355 evaluation board.
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

#ifndef _79EB355_H_
#define _79EB355_H_

#define IDT_BUS_FREQ   75 // MHz
#define IDT_CLOCK_MULT 2

/* Memory map of 79EB355 board */

/* DRAM */
#define RAM_BASE        0x00000000
#define RAM_SIZE	(32*1024*1024)

/* SRAM (device 1) */
#define SRAM_BASE       0x02000000
#define SRAM_SIZE       0x00100000

/* FLASH (device 2) */
#define FLASH_BASE      0x0C000000
#define FLASH_SIZE      0x00C00000

/* ATM PHY (device 4) */
#define ATM_PHY_BASE    0x14000000

/* TDM switch (device 3) */
#define TDM_BASE        0x1A000000

/* LCD panel (device 3) */
#define LCD_BASE        0x1A002000

/* RTC (DS1511W) (device 3) */
#define RTC_BASE        0x1A004000

/* NVRAM (256 bytes internal to the DS1511 RTC) */
#define NVRAM_ADDR      RTC_BASE + 0x10
#define NVRAM_DATA      RTC_BASE + 0x13
#define NVRAM_ENVSIZE_OFF  4
#define NVRAM_ENVSTART_OFF 32

#include <asm/rc32300/rc32355.h>
#include <asm/rc32300/ds1501rtc.h>

#endif /* _79EB355_H_ */
