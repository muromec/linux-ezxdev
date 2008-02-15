/*
 * drivers/char/shm_vio.h
 *
 * SH-7300 VIO3 driver for Super-H
 *
 * Author: Takashi SHUDO
 *
 * 2003 (c) Takashi SHUDO. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __SH7300VIO3
#define __SH7300VIO3

/*
  ioctl() Operation definition
*/
#define VIOCTL_CAPTURE	0x1000	/* Only capture(Only YUV data is created) */
#define VIOCTL_YUV2RGB	0x1001	/* YUV->RGB conversion */

/*
  Graphics format
*/
#define	M_WIDTH		176	/* hsize */
#define	M_HEIGHT	144	/* vsize */

#define Y_SIZE		(M_WIDTH * M_HEIGHT)

#define H_OFFSET	((640-M_WIDTH)/2)	/* hoffset */
#define V_OFFSET	((480-M_HEIGHT)/2)	/* voffset */

#define RESIZERATIO_1_1	0x0000

/*
  Picture memory address definition
*/
#define YC_REVERSE		/* A definition is given when a camera is upside-down. */

#define VMEM_SIZE	0x10000	/* Size for one picture memory */

#define VIOMEM_BASE	0xafd00000	/* Buffer address */

#define Y_BUFF_OS	0x00000000	/* Y */
#define C_BUFF_OS       0x00100000      /* C */
#define RGB_BUFF_OS	0x00200000	/* RGB */

#define Y_BUFF		(VIOMEM_BASE+Y_BUFF_OS)
#define C_BUFF          (VIOMEM_BASE+C_BUFF_OS)
#define RGB_BUFF	(VIOMEM_BASE+RGB_BUFF_OS)

#define SIZEOFVIOMEM	0x00300000

/*
  VIO3 Registers
*/
#define	VIO3_REG_BASE	0xA4910000

#define	VIO3_CAPSR		(VIO3_REG_BASE+0x0000)
#define	VIO3_CAPCR		(VIO3_REG_BASE+0x0004)
#define	VIO3_CAMCR		(VIO3_REG_BASE+0x0008)
#define	VIO3_CAMOR		(VIO3_REG_BASE+0x000C)
#define	VIO3_CAPWR		(VIO3_REG_BASE+0x0010)
#define	VIO3_CSTCR		(VIO3_REG_BASE+0x0014)
#define	VIO3_CFLCR		(VIO3_REG_BASE+0x0018)
#define	VIO3_CFSZR		(VIO3_REG_BASE+0x001C)
#define	VIO3_CDWDR		(VIO3_REG_BASE+0x0020)
#define	VIO3_CDAYR		(VIO3_REG_BASE+0x0024)
#define	VIO3_CDACR		(VIO3_REG_BASE+0x0028)
#define	VIO3_CDMCR		(VIO3_REG_BASE+0x002C)
#define	VIO3_CDBCR		(VIO3_REG_BASE+0x0030)
#define	VIO3_CROTR		(VIO3_REG_BASE+0x0034)
#define	VIO3_VIPSR		(VIO3_REG_BASE+0x0040)
#define	VIO3_VVSWR		(VIO3_REG_BASE+0x0044)
#define	VIO3_VVSSR		(VIO3_REG_BASE+0x0048)
#define	VIO3_VDWDR		(VIO3_REG_BASE+0x0050)
#define	VIO3_VSARR		(VIO3_REG_BASE+0x0054)
#define	VIO3_VSAGR		(VIO3_REG_BASE+0x0058)
#define	VIO3_VSABR		(VIO3_REG_BASE+0x005C)
#define	VIO3_VDAYR		(VIO3_REG_BASE+0x0060)
#define	VIO3_VDACR		(VIO3_REG_BASE+0x0064)
#define	VIO3_VTRCR		(VIO3_REG_BASE+0x0068)
#define	VIO3_VFLCR		(VIO3_REG_BASE+0x006C)
#define	VIO3_VFSZR		(VIO3_REG_BASE+0x0070)
#define VIO3_VLUTR		(VIO3_REG_BASE+0x0074)
#define VIO3_VROTR		(VIO3_REG_BASE+0x0078)
#define	VIO3_VOSCR		(VIO3_REG_BASE+0x0080)
#define	VIO3_VOSWR		(VIO3_REG_BASE+0x0084)
#define	VIO3_VODSR		(VIO3_REG_BASE+0x0088)
#define	VIO3_VODOR		(VIO3_REG_BASE+0x008C)
#define	VIO3_VOSAR		(VIO3_REG_BASE+0x0090)
#define	VIO3_VOAYR		(VIO3_REG_BASE+0x0094)
#define	VIO3_VOACR		(VIO3_REG_BASE+0x0098)
#define	VIO3_VHCCR		(VIO3_REG_BASE+0x00A0)
#define	VIO3_VHDSR		(VIO3_REG_BASE+0x00A4)
#define	VIO3_VHDOR		(VIO3_REG_BASE+0x00A8)
#define	VIO3_VHAYR		(VIO3_REG_BASE+0x00AC)
#define	VIO3_VHACR		(VIO3_REG_BASE+0x00B0)
#define	VIO3_FXADR		(VIO3_REG_BASE+0x00D0)
#define	VIO3_DSWPR		(VIO3_REG_BASE+0x00E0)
#define	VIO3_EVTCR		(VIO3_REG_BASE+0x0100)
#define	VIO3_EVTSR		(VIO3_REG_BASE+0x0104)
#define	VIO3_STATR		(VIO3_REG_BASE+0x0110)
#define	VIO3_SRSTR		(VIO3_REG_BASE+0x0120)

#define	VIO3_CLUT_OSD	(VIO3_REG_BASE+0x1000)
#define	VIO3_CLUT_HWC	(VIO3_REG_BASE+0x1400)
#define	VIO3_HWC_TEX	(VIO3_REG_BASE+0x2000)
#define	VIO3_CLUT_VIP	(VIO3_REG_BASE+0x3000)

#define	PSELA		0xA4050140
#define	PACR		0xA4050100
#define	PECR		0xA4050108
#define	PMCR		0xA4050118
#define	HIZCRA		0xA4050146
#define	DRVCR		0xA4050150
#define	IPRE		0xA414001A
#define	IMCR1		0xA4080062
#define	IMR1		0xA4080042
#define	STBCR3		0xA40A0000

#define CIAON (1<<9)

#endif	/* __SH7300VIO3 */
