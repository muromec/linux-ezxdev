/*
 * linux/drivers/video/ezx_lcd/72r89803y01.h
 * Intel Bulverde/PXA250/210 LCD Controller for TFT panel of 72r89803y01
 *
 * Copyright (C) 2004-2005 - Motorola
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
 *      - Define parameter used by 72r89803y01 LCD panel;
 *  2005/12/12  Wang limei
 *	 - Update delay time of disabling LCD panel according to HW,
 *	    it can reduce about 200uA current drain;
 */

#ifndef _PXA_FB_72R89803Y01_H_
#define _PXA_FB_72R89803Y01_H_

#define TFT_PANEL_TURNON_DELAY  200  //200ms
#define TFT_PANEL_TURNOFF_DELAY 120  //120ms
#define TO_JIFFIES(msec) (((msec + (1000/HZ -1)) * HZ) / 1000)
#define TFT_PANEL_TURNON_DELAY_IN_JIFFIES   TO_JIFFIES(TFT_PANEL_TURNON_DELAY)
#define TFT_PANEL_TURNOFF_DELAY_IN_JIFFIES  TO_JIFFIES(TFT_PANEL_TURNOFF_DELAY)

#ifdef CONFIG_FB_PXA_QVGA

#define LCD_PIXCLOCK			192308//150000
//#define LCD_BPP				PXAFB_BPP
#define LCD_XRES			240
#define LCD_YRES			320
#define LCD_HORIZONTAL_SYNC_PULSE_WIDTH        10
#define LCD_VERTICAL_SYNC_PULSE_WIDTH          2
#define LCD_BEGIN_OF_LINE_WAIT_COUNT           20
#define LCD_BEGIN_FRAME_WAIT_COUNT             3
#define LCD_END_OF_LINE_WAIT_COUNT             10
#define LCD_END_OF_FRAME_WAIT_COUNT            2
#define LCD_SYNC                        (FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT)
#ifdef CONFIG_FB_PXA_18BPP
#define LCD_LCCR0                       0x002008b8 //TEST EOF0; 0x002008F8 // clear OUC for overlays  // 0x023008F8   //mask-BSM0 0x003008F8
#define LCD_LCCR3                       0xC630FF13  /* 18 bpp */
#elif defined(CONFIG_FB_PXA_19BPP)
#define LCD_LCCR0                       0x002008b8 //TEST EOF0; 0x002008F8 // clear OUC for overlays  // 0x023008F8   //mask-BSM0 0x003008F8
#define LCD_LCCR3                       0x2030FF13  /* 19 bpp -- basefb */
#elif defined(CONFIG_FB_PXA_24BPP)
#define LCD_LCCR0                       0x022008b8 // set OUC for overlays  // 0x023008F8   //mask-BSM0 0x003008F8
#define LCD_LCCR3                       0xC130FF13  /* base 2bpp/overlay1 25bpp */
#else
#define LCD_LCCR0                       0x002008b8 // clear OUC for overlays  // 0x023008F8   //mask-BSM0 0x003008F8
#define LCD_LCCR3			0x0430FF13  /* 16 bpp */
#endif
#define LCD_LCCR4			0X82080000  //Enable LCLK/(PCD+1) and 13MHz PCD value setting//

#endif //CONFIG_FB_PXA_QVGA


#endif /* _PXA_FB_72R89803Y01_H_ */

