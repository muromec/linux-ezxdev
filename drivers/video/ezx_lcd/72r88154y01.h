/*
 * linux/drivers/video/ezx_lcd/72r88154y01.h
 * Intel Bulverde/PXA250/210 LCD Controller for smart panel of 72r88154y01
 *
 * Copyright (C) 2004 - Motorola
 *
 * Copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
 * Copyright (C) 1999 Eric A. Thomas
 * Based on acornfb.c Copyright (C) Russell King.
 *
 * 2001-08-03: Cliff Brake <cbrake@acclent.com>
 *	- ported SA1100 code to PXA
 *  
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  2004/06/28  Susan
 *      - Define parameter used by 72r88154y01 LCD panel;
 */

#ifndef _PXA_FB_72R88154Y01_H_
#define _PXA_FB_72R88154Y01_H_

/* Panel specific setting */
#define SMART_LCD_BPP                         16 // 18 //16
#define SMART_LCD_XRES                        96 // 128 //104
#define SMART_LCD_YRES                        80 //80
#define SMART_LCD_BEGIN_OF_LINE_WAIT_COUNT      10 /*LCLK = 104MHz(26MHz), LCD_CLK_PERIOD = 9.6ns (38ns), PWcsh(l) is 125ns, L_PCLK_WR pulse width, in LCD_CLK_PERIOD */
#define SMART_LCD_END_OF_LINE_WAIT_COUNT        10 /* Setup, hold time for CS, Data, A0, in LCD_CLK_PERIOD */
#define SMART_LCD_LCCR0		0x01F008F8  //0x03F008F8, we need to clear EOFM0
#define SMART_LCD_LCCR1		(0x0A0A0000 | (SMART_LCD_XRES - 1)) 
#define SMART_LCD_LCCR2		(0x00000000 | (SMART_LCD_YRES - 1))
#define SMART_LCD_LCCR3		0xC600FF06  //6-18bpp packed or 5-18bpp unpacked (just set it by ss).(ss)From SSD1789AZ, cmd_inhibit_time should be (Tcycle-PWcsl)== 65ns ? 
#define SMART_LCD_LCCR4		0x00000000
#define SMART_LCD_LCCR5		0x3F3F3F3F

//For 72R88154Y01, hw interface between Bulverde LCD controller and Smart panel are inconsistent, use send display data -- Tom
#define CMD_BUF_MAX_NUM		(96*80*2+4)
/* Smart panel specific CMD definition */
#define SLEEPOUT	0x0294 			/* Sleep OUT */
#define IOSON	  	0x02D1	 			/* Internal Oscillator ON */
#define POWERCONTROLSET_0  0x0220
#define POWERCONTROLSET_1  0x030B
#define BIASRATIOSET_0       0x02FB
#define BIASRATIOSET_1       0x0304
#define PWM_FRC_SCROLL_0     0x02F7
#define PWM_FRC_SCROLL_1     0x0328
#define PWM_FRC_SCROLL_2     0x032C
#define PWM_FRC_SCROLL_3     0x0305
#define TEMPE_GRADIENT_0     0x0282
#define TEMPE_GRADIENT_1     0x0301
#define COM_SCAN_DIRECTION_0 0x02BB
#define COM_SCAN_DIRECTION_1 0x0302
#define DATA_SCAN_DIRECTION_0  0x02BC
#define DATA_SCAN_DIRECTION_1  0x0300
#define DATA_SCAN_DIRECTION_2  0x0300
#define DATA_SCAN_DIRECTION_3  0x0300
#define FRAME_FREQ_NLINE_0    0x02F2
#define FRAME_FREQ_NLINE_1    0x0305
#define FRAME_FREQ_NLINE_2    0x030C
#define CONTRAST_CONTROL_0    0x0281
#define CONTRAST_CONTROL_1    0x0315
#define CONTRAST_CONTROL_2    0x0306
#define DISPLAY_CONTROL_0     0x02CA
#define DISPLAY_CONTROL_1     0x0300
#define DISPLAY_CONTROL_2     0x0313
#define DISPLAY_CONTROL_3     0x0300
#define FIRST_DISPLAY_CMD     0x0244
#define START_LINE	      0x0300       
#define PAGE_ADDR_SET_0       0x0275
#define PAGE_ADDR_SET_1	      0x0300
#define PAGE_ADDR_SET_2       0x034F
#define COLUMN_ADDRESS_SET_0   0x0215 
#define COLUMN_ADDRESS_SET_1   0x0304
#define COLUMN_ADDRESS_SET_2   0x0363
#define INVERSE_DISPLAY	       0x02A7
#define SET_COM_SEQ_0		0x02F1
#define SET_COM_SEQ_1		0x03C0
#define SET_COM_SEQ_2		0x0300
#define SET_COM_SEQ_3		0x0300
#define DISPLAY_ON		0x02AF
#define DISPLAY_OFF		0x02AE
#define IOSOFF			0x02D2
#define SLEEP_IN		0x0295
#define SLEEP_OUT		0x0294
#define WRITE_DISP_DATA         0x005C


#endif /* _PXA_FB_72R888154Y01_H_ */
