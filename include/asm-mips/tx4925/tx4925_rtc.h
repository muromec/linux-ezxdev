#ifndef TX4925_TX4925_RTC_H
#define TX4925_TX4925_RTC_H

/*
 * linux/include/asm-mips/tx4925/tx4925_rtc.h
 *
 * tx4925_rtc defs
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * Copyright 2002 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <asm/tx4925/tx4925.h>
#include <asm/tx4925/tx4925_mips.h>

#define TX4925_RTC_BASE 0xff1ff900

#define TX4925_RTC_HI_MASK    0x00000ff
#define TX4925_RTC_FREQ_SHIFT 15	/* 32768Hz = 15bits */

#define TX4925_RTC_RTCCTRL_DISRTINT BM_07_07
#define TX4925_RTC_RTCCTRL_DISALINT BM_06_06
#define TX4925_RTC_RTCCTRL_RTCCLR   BM_03_03

struct tx4925_rtc_st {
	vu32 rtchi;
	vu32 rtclo;
	vu32 alarmhi;
	vu32 alarmlo;
	vu32 rtcctrl;
	vu32 rtcint;
};

extern struct tx4925_rtc_st *tx4925_rtc_ptr;

extern unsigned long tx4925_rtc_get_time(void);
int tx4925_rtc_set_time(unsigned long t);
void tx4925_rtc_init(unsigned long base);

/* wait for seconds to change */
extern inline void
tx4925_rtc_wait(void)
{
	u32 v1;
	u32 v2;
	u32 v3;
	u32 v4;

	do {
		v1 = tx4925_rtc_ptr->rtclo;
		v2 = tx4925_rtc_ptr->rtclo;
	} while (v1 != v2);

	v3 = v2 >> TX4925_RTC_FREQ_SHIFT;
	do {
		v4 = tx4925_rtc_ptr->rtclo >> TX4925_RTC_FREQ_SHIFT;
	} while (v3 == v4);

	return;
}

#endif
