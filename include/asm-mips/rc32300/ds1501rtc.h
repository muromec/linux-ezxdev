/*
 *
 * BRIEF MODULE DESCRIPTION
 *	Definitions for Dallas DS1501 RTC present on IDT 79EB355 board.
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

#ifndef _DS1501RTC_H_
#define _DS1501RTC_H_

/* Image of DS1501 registers */
typedef struct {
	u8 secs;
	u8 mins;
	u8 hours;
	u8 weekday;
	u8 date;
	u8 month;
	u8 year;
	u8 century;
	u8 alarm_secs;
	u8 alarm_mins;
	u8 alarm_hours;
	u8 alarm_day_date;
	u8 watchdog_a;
	u8 watchdog_secs;
	u8 control_a;
	u8 control_b;
	u8 nvram_addr;
	u8 dummy[2];
	u8 nvram_data;
} ds1501_regs_t;

#define rtc ((ds1501_regs_t*)KSEG1ADDR(RTC_BASE))

/*
 * Control register bit definitions
 */
#define TDC_ENA_BUFF    0x80
#define TDC_DIS_BUFF    0x7f

#define TDS_STOP    	0x80

#endif /* _DS1501RTC_H */
