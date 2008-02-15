/*
 *  linux/include/asm-arm/arch-ti925/uncompress.h
 *
 * BRIEF MODULE DESCRIPTION
 *   uncompress and ugly serial code to startup the bootup of the board.
 *
 * Copyright (C) 2000 RidgeRun, Inc. (http://www.ridgerun.com)
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
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

#define CONSOLE_CHANNEL 0

#define IO_ADDRESS(x) (*(unsigned char *)x)

#define RBR 0x00
#define THR 0x00
#define DLL 0x00
#define IER 0x01
#define DLM 0x01
#define IIR 0x02
#define FCR 0x02
#define LCR 0x03
#define MCR 0x04
#define LSR 0x05
#define MSR 0x06
#define SCR 0x07

#define DUART 0xfffce800
#define DUART_DELTA 0x1
#define CHANNELOFFSET 0x0

#define LCR_DLAB 0x80
#define XTAL  1843200
//#define XTAL    3686400
#define LSR_THRE 0x20
#define LSR_BI   0x10
#define LSR_DR   0x01
#define MCR_LOOP 0x10
#define ACCESS_DELAY 0x10000
/******************************
 Routine:
 Description:
 ******************************/
int
inreg(int channel, int reg)
{
  int val;
  val = *((volatile unsigned char *)DUART+(channel * CHANNELOFFSET)
          +(reg * DUART_DELTA));
  //  delay();
  return val;
}

/******************************
 Routine:
 Description:
 ******************************/
void
outreg(int channel, int reg, unsigned char val)
{
  *((volatile unsigned char *)DUART+(channel * CHANNELOFFSET)
    +(reg * DUART_DELTA)) = val;
  /* delay(); */
}

/******************************
 Routine:
 Description:
   Transmit a character.
 ******************************/
void
serial_putc(int channel,int c)
{

  while ((inreg(channel, LSR) & LSR_THRE) == 0)
  ;
  outreg(channel, THR, c);
}

static void puts(const char *s)
{

  while (*s) {
    serial_putc(CONSOLE_CHANNEL,*s);
    if (*s == '\n') {
      serial_putc(CONSOLE_CHANNEL,'\r');
    }
    s++;
  }
}

#define arch_decomp_setup()

#define arch_decomp_wdog()
