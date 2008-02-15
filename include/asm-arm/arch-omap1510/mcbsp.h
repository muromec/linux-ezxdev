/*
 * File: mcbsp.h
 *
 * Defines for Multi-Channel Buffered Serial Port
 *
 * Copyright (C) 2002 RidgeRun, Inc.
 * Author: Steve Johnson
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
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
 *
 */
#ifndef mcbsp_h
#define mcbsp_h

#include <asm/arch/hardware.h>

#define AUDIO_DRR2  (OMAP1510_MCBSP1_BASE + 0x800)
#define AUDIO_DRR1  (OMAP1510_MCBSP1_BASE + 0x802)
#define AUDIO_DXR2  (OMAP1510_MCBSP1_BASE + 0x804)
#define AUDIO_DXR1  (OMAP1510_MCBSP1_BASE + 0x806)
#define AUDIO_SPCR2 (OMAP1510_MCBSP1_BASE + 0x808)
#define AUDIO_SPCR1 (OMAP1510_MCBSP1_BASE + 0x80a)
#define AUDIO_RCR2  (OMAP1510_MCBSP1_BASE + 0x80c)
#define AUDIO_RCR1  (OMAP1510_MCBSP1_BASE + 0x80e)
#define AUDIO_XCR2  (OMAP1510_MCBSP1_BASE + 0x810)
#define AUDIO_XCR1  (OMAP1510_MCBSP1_BASE + 0x812)
#define AUDIO_SRGR2 (OMAP1510_MCBSP1_BASE + 0x814)
#define AUDIO_SRGR1 (OMAP1510_MCBSP1_BASE + 0x816)
#define AUDIO_MCR2  (OMAP1510_MCBSP1_BASE + 0x818)
#define AUDIO_MCR1  (OMAP1510_MCBSP1_BASE + 0x81a)
#define AUDIO_RCERA (OMAP1510_MCBSP1_BASE + 0x81c)
#define AUDIO_RCERB (OMAP1510_MCBSP1_BASE + 0x81e)
#define AUDIO_XCERA (OMAP1510_MCBSP1_BASE + 0x820)
#define AUDIO_XCERB (OMAP1510_MCBSP1_BASE + 0x822)
#define AUDIO_PCR0  (OMAP1510_MCBSP1_BASE + 0x824)

#define MCBSP2_DRR2  (OMAP1510_MCBSP2_BASE + 0x00)
#define MCBSP2_DRR1  (OMAP1510_MCBSP2_BASE + 0x02)
#define MCBSP2_DXR2  (OMAP1510_MCBSP2_BASE + 0x04)
#define MCBSP2_DXR1  (OMAP1510_MCBSP2_BASE + 0x06)
#define MCBSP2_SPCR2 (OMAP1510_MCBSP2_BASE + 0x08)
#define MCBSP2_SPCR1 (OMAP1510_MCBSP2_BASE + 0x0a)
#define MCBSP2_RCR2  (OMAP1510_MCBSP2_BASE + 0x0c)
#define MCBSP2_RCR1  (OMAP1510_MCBSP2_BASE + 0x0e)
#define MCBSP2_XCR2  (OMAP1510_MCBSP2_BASE + 0x10)
#define MCBSP2_XCR1  (OMAP1510_MCBSP2_BASE + 0x12)
#define MCBSP2_SRGR2 (OMAP1510_MCBSP2_BASE + 0x14)
#define MCBSP2_SRGR1 (OMAP1510_MCBSP2_BASE + 0x16)
#define MCBSP2_MCR2  (OMAP1510_MCBSP2_BASE + 0x18)
#define MCBSP2_MCR1  (OMAP1510_MCBSP2_BASE + 0x1a)
#define MCBSP2_RCERA (OMAP1510_MCBSP2_BASE + 0x1c)
#define MCBSP2_RCERB (OMAP1510_MCBSP2_BASE + 0x1e)
#define MCBSP2_XCERA (OMAP1510_MCBSP2_BASE + 0x20)
#define MCBSP2_XCERB (OMAP1510_MCBSP2_BASE + 0x22)
#define MCBSP2_PCR0  (OMAP1510_MCBSP2_BASE + 0x24)

/************************** McBSP SPCR1 bit definitions ***********************/
#define RRST                    0x0001
#define RRDY                    0x0002
#define RFULL                   0x0004
#define RSYNC_ERR               0x0008
#define RINTM(value)            ((value)<<4)       /* bits 4:5   */
#define ABIS                    0x0040
#define DXENA                   0x0080
#define CLKSTP(value)           ((value)<<11)      /* bits 11:12 */
#define RJUST(value)            ((value)<<13)      /* bits 13:14 */
#define DLB                     0x8000

/************************** McBSP SPCR2 bit definitions ***********************/
#define XRST           0x0001
#define XRDY           0x0002
#define XEMPTY         0x0004
#define XSYNC_ERR      0x0008
#define XINTM(value)   ((value)<<4)           /* bits 4:5   */
#define GRST           0x0040
#define FRST           0x0080
#define SOFT           0x0100
#define FREE           0x0200

/************************** McBSP PCR bit definitions *************************/
#define CLKRP          0x0001
#define CLKXP          0x0002
#define FSRP           0x0004
#define FSXP           0x0008
#define DR_STAT        0x0010
#define DX_STAT        0x0020
#define CLKS_STAT      0x0040
#define SCLKME         0x0080
#define CLKRM          0x0100
#define CLKXM          0x0200
#define FSRM           0x0400
#define FSXM           0x0800
#define RIOEN          0x1000 
#define XIOEN          0x2000
#define IDLE_EN        0x4000

/************************** McBSP RCR1 bit definitions ************************/
#define RWDLEN1(value)      ((value)<<5)      /* Bits 5:7  */
#define RFRLEN1(value)      ((value)<<8)      /* Bits 8:14 */

/************************** McBSP XCR1 bit definitions ************************/
#define XWDLEN1(value)      ((value)<<5)      /* Bits 5:7  */
#define XFRLEN1(value)      ((value)<<8)      /* Bits 8:14 */

/*************************** McBSP RCR2 bit definitions ***********************/
#define RDATDLY(value)      (value)         /* Bits 0:1 */
#define RFIG                0x0004
#define RCOMPAND(value)     ((value)<<3)      /* Bits 3:4 */
#define RWDLEN2(value)      ((value)<<5)      /* Bits 5:7 */
#define RFRLEN2(value)      ((value)<<8)      /* Bits 8:14 */
#define RPHASE              0x8000

/*************************** McBSP XCR2 bit definitions ***********************/
#define XDATDLY(value)      (value)         /* Bits 0:1 */
#define XFIG                0x0004
#define XCOMPAND(value)     ((value)<<3)      /* Bits 3:4 */
#define XWDLEN2(value)      ((value)<<5)      /* Bits 5:7 */
#define XFRLEN2(value)      ((value)<<8)      /* Bits 8:14 */
#define XPHASE              0x8000

/************************* McBSP SRGR1 bit definitions ************************/
#define CLKGDV(value)      (value)         /* Bits 0:7 */
#define FWID(value)        ((value)<<8)      /* Bits 8:15 */

/************************* McBSP SRGR2 bit definitions ************************/
#define FPER(value)       (value)         /* Bits 0:11 */
#define FSGM              0x1000
#define CLKSM             0x2000
#define CLKSP             0x4000
#define GSYNC             0x8000

/************************* McBSP MCR1 bit definitions *************************/
#define RMCM              0x0001
#define RCBLK(value)      ((value)<<2)      /* Bits 2:4 */
#define RPABLK(value)     ((value)<<5)      /* Bits 5:6 */
#define RPBBLK(value)     ((value)<<7)      /* Bits 7:8 */

/************************* McBSP MCR2 bit definitions *************************/
#define XMCM(value)       (value)         /* Bits 0:1 */
#define XCBLK(value)      ((value)<<2)      /* Bits 2:4 */
#define XPABLK(value)     ((value)<<5)      /* Bits 5:6 */
#define XPBBLK(value)     ((value)<<7)      /* Bits 7:8 */

#endif
