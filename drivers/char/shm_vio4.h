/*
 * drivers/char/shm_vio4.c
 *
 * SH73180 VIO4 dirver for Super-H
 *
 * Auther: Kenichi KITAZAWA
 *
 * 2004(c) Kenichi KITAZAWA. This file licensed under
 * the terms of the GNU General Public License version 2. THis program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __SH_MOBILE_73180_VIO4_H__
#define __SH_MOBILE_73180_VIO4_H__

#include <linux/ioctl.h>
#include <linux/wait.h>

/*
  struct

  This correspond with VIO4 Register. See hardware manual.
*/
#define CAPRESIZE_1_0 0x0000
#define CAPRESIZE_7_8 0x1248
#define CAPRESIZE_3_4 0x1550
#define CAPRESIZE_5_8 0x1998
#define CAPRESIZE_1_2 0x2000
#define CAPRESIZE_3_8 0x2aa8
#define CAPRESIZE_1_3 0x3000
#define CAPRESIZE_1_4 0x4000
#define CAPRESIZE_1_5 0x5000
#define CAPRESIZE_1_6 0x6000
#define CAPRESIZE_1_7 0x7000
#define CAPRESIZE_1_8 0x8000

struct vio4_capture {
	long img_width;
	long img_height;
	long h_offset;
	long v_offset;
	long vdpol;
	long hdpol;
	long img_capture;	/* 0:image capture via camera 1:getting data */
	long hds_coef;		/* down sample filter */
	long vds_coef;		/* down sample filter */
	long clip_width;	/* clip width with down sample filter */
	long clip_height;	/* clip height with down sample filter */
	long capcr_cds;		/* CAPCR register cds bit */
	long running;           /* state of ceu */
};
#define VEURESIZE_VER_4_0 0x0400
#define VEURESIZE_VER_2_0 0x0200
#define VEURESIZE_VER_3_2 0x0aa8
#define VEURESIZE_VER_1_0 0x0000
#define VEURESIZE_VER_7_8 0x1248
#define VEURESIZE_VER_3_4 0x1550
#define VEURESIZE_VER_5_8 0x1998
#define VEURESIZE_VER_1_2 0x2000
#define VEURESIZE_VER_3_8 0x2aa8
#define VEURESIZE_VER_1_3 0x3000
#define VEURESIZE_VER_1_4 0x4000
#define VEURESIZE_VER_1_5 0x5000
#define VEURESIZE_VER_1_6 0x6000
#define VEURESIZE_VER_1_7 0x7000
#define VEURESIZE_VER_1_8 0x8000

#define VEURESIZE_HOR_4_0 0x0400
#define VEURESIZE_HOR_2_0 0x0200
#define VEURESIZE_HOR_3_2 0x0aa8
#define VEURESIZE_HOR_1_0 0x0000
#define VEURESIZE_HOR_7_8 0x1248
#define VEURESIZE_HOR_3_4 0x1550
#define VEURESIZE_HOR_5_8 0x1998
#define VEURESIZE_HOR_1_2 0x2000
#define VEURESIZE_HOR_3_8 0x2aa8
#define VEURESIZE_HOR_1_3 0x3000
#define VEURESIZE_HOR_1_4 0x4000
#define VEURESIZE_HOR_1_5 0x5000
#define VEURESIZE_HOR_1_6 0x6000
#define VEURESIZE_HOR_1_7 0x7000
#define VEURESIZE_HOR_1_8 0x8000

struct vio4_video {
	long img_width;		/* input image width */
	long img_height;	/* input image height */
	long clip_width;	/* output image width */
	long clip_height;	/* output image height */
	long v_resize;		/* vertical resize coefficient */
	long h_resize;		/* horizon resize coefficient */
	long bit_pixel;		/* bit per pixel */
	long rgb_pad;		/* rgb pad */
	long chds;		/* VTRCR register chds bit */
	long chrr;		/* VTRCR register chrr bit */
	long wpkf;		/* output rgb format */
	long rpkf;		/* input rgb format */
	long dith;		/* dith on/off */
	long tery;		/* VTRCR register TE,RY bit */
	long running;           /* state of veu */
};

/*
  image buffer memory
*/
struct vio4_img_buff {
	long viomem_base;
	long vmem_size;
	long vmem_num;
	long y_buff_os;
	long c_buff_os;
	long rgb_buff_os;
	long rgb_c_buff_os;
	long disp_buff_os;
	long disp_c_buff_os;
	long sizeofviomem;
};
#define VIO4_DEV_NAME	"vio4"
#define VIO4_DEV_MAJOR  240
/*
  ioctl command
*/
#define VIO4_IOC_MAGIC	'v'
#define VIO4_IOCRESET _IO(VIO4_IOC_MAGIC,0)
/*
 * S(set) configration via pointer
 * T(tell) notice via argument
 * G(get) get via pointer
 * Q(query) query via pointer
 */
#define VIO4_IOCTCAPTURE _IO(VIO4_IOC_MAGIC,1)	/* capture start */
#define VIO4_IOCTYUV2RGB _IO(VIO4_IOC_MAGIC,2)	/* run transfer YUV to RGB */
#define VIO4_IOCTROTATE  _IO(VIO4_IOC_MAGIC,3)	/* run resize */

#define VIO4_IOCSCAPCFG _IO(VIO4_IOC_MAGIC,4)	/* set configure captuer */
#define VIO4_IOCSVEUCFG _IO(VIO4_IOC_MAGIC,5)	/* set configure VEU */
#define VIO4_IOCSFEUCFG _IO(VIO4_IOC_MAGIC,6)	/* set configure FEU */
#define VIO4_IOCSIMGBUF _IO(VIO4_IOC_MAGIC,7)	/* set vio4_img_buff */

#define VIO4_IOCGCAPCFG _IO(VIO4_IOC_MAGIC,8)	/* get configure capture */
#define VIO4_IOCGVEUCFG _IO(VIO4_IOC_MAGIC,9)	/* get configure VEU */
#define VIO4_IOCGFEUCFG _IO(VIO4_IOC_MAGIC,10)	/* get configure FEU */
#define VIO4_IOCGIMGBUF _IO(VIO4_IOC_MAGIC,11)	/* get vio4_img_buff */

#define VIO4_IOC_MAXNR 11

/*************************************************************
 * register bit
 *************************************************************/
/*
 * CETCR(capture event flag clear register)
 */
#define VIO_CETCR_NVD	(long)(1<<25)
#define VIO_CETCR_NHD	(long)(1<<24)
#define VIO_CETCR_VBP	(long)(1<<20)
#define VIO_CETCR_IGVS	(long)(1<<18)
#define VIO_CETCR_IGHS	(long)(1<<17)
#define VIO_CETCR_CDTOR (long)(1<<16)
#define VIO_CETCR_VD	(long)(1<<9)
#define VIO_CETCR_HD	(long)(1<<8)
#define VIO_CETCR_IGRW	(long)(1<<4)
#define VIO_CETCR_CPE	(long)(1<<0)

#define VIO_VEVTR_FEEND (long)(1<<4)
#define VIO_VEVTR_VEEND (long)(1<<0)

#define VIO_BEVTR_INTREQ (unsigned long)(1<<16)
#define VIO_BEVTR_BEVIO (unsigned long)(1<<1)
#define VIO_BEVTR_BEEND (unsigned long)(1<<0)
/*
 * register address
 */

#define STBCR6 0xA40A0032

/* CEU register */
#define CEU_BASE	0xfe910000
#define VIO_CAPSR	(CEU_BASE+0x0000)
#define VIO_CAPCR	(CEU_BASE+0x0004)
#define VIO_CAMCR	CEU_BASE+0x0008
#define VIO_CMCYR	CEU_BASE+0x000c
#define VIO_CAMOR	CEU_BASE+0x0010
#define VIO_CAPWR	CEU_BASE+0x0014
#define VIO_CSTCR	CEU_BASE+0x0020
#define VIO_CSECR	CEU_BASE+0x0024
#define VIO_CFLCR	CEU_BASE+0x0030
#define VIO_CFSZR	CEU_BASE+0x0034
#define VIO_CDWDR	CEU_BASE+0x0038
#define VIO_CDAYR	CEU_BASE+0x003c
#define VIO_CDACR	CEU_BASE+0x0040
#define VIO_CFXAR	CEU_BASE+0x0060
#define VIO_CDSWR	CEU_BASE+0x0064
#define VIO_CEIER	CEU_BASE+0x0070
#define VIO_CETCR	CEU_BASE+0x0074
#define VIO_CETSR	CEU_BASE+0x0078
#define VIO_CSTSR	CEU_BASE+0x007c
#define VIO_CSRTR	CEU_BASE+0x0080

/* VEU register */
#define VEU_BASE	0xfe920000
#define VIO_VESTR	(VEU_BASE+0x0000)
#define VIO_VESWR	VEU_BASE+0x0010
#define VIO_VESSR	VEU_BASE+0x0014
#define VIO_VSAYR	VEU_BASE+0x0018
#define VIO_VSACR	VEU_BASE+0x001c
#define VIO_VEDWR	VEU_BASE+0x0030
#define VIO_VDAYR	VEU_BASE+0x0034
#define VIO_VDACR	VEU_BASE+0x0038
#define VIO_VTRCR	VEU_BASE+0x0050
#define VIO_VRFCR	VEU_BASE+0x0054
#define VIO_VRFSR	VEU_BASE+0x0058

/* FEM register */
#define FEM_BASE	VEU_BASE
#define VIO_FESTR	FEM_BASE+0x0100
#define VIO_FESWR	FEM_BASE+0x0110
#define VIO_FESSR	FEM_BASE+0x0114
#define VIO_FSAYR	FEM_BASE+0x0118
#define VIO_FSACR	FEM_BASE+0x011c
#define VIO_FEDWR	FEM_BASE+0x0130
#define VIO_FDAYR	FEM_BASE+0x0134
#define VIO_FDACR	FEM_BASE+0x0138
#define VIO_FEFCR	FEM_BASE+0x0140
#define VIO_FEMCR	FEM_BASE+0x0150
#define VIO_FVTCR	FEM_BASE+0x0158
#define VIO_FHTCR	FEM_BASE+0x015c

/* VEU register */
#define VIO_VAFXR	VEU_BASE+0x0200
#define VIO_VSWPR	VEU_BASE+0x0210
#define VIO_VEIER	VEU_BASE+0x0220
#define VIO_VEVTR	VEU_BASE+0x0224
#define VIO_VSTAR	VEU_BASE+0x0240
#define VIO_VSWRR	VEU_BASE+0x0300

/* BEU register */
#define BEU_BASE	0xfe930000

#define VIO_BESTR	BEU_BASE+0x0000
#define VIO_BSMWR1	BEU_BASE+0x0010
#define VIO_BSSZR1	BEU_BASE+0x0014
#define VIO_BSAYR1	BEU_BASE+0x0018
#define VIO_BSACR1	BEU_BASE+0x001c
#define VIO_BSIFR1	BEU_BASE+0x0020
#define VIO_BSMWR2	BEU_BASE+0x0030
#define VIO_BSSZR2	BEU_BASE+0x0034
#define VIO_BSAYR2	BEU_BASE+0x0038
#define VIO_BSACR2	BEU_BASE+0x003c
#define VIO_BSIFR2	BEU_BASE+0x0040
#define VIO_BSMWR3	BEU_BASE+0x0050
#define VIO_BSSZR3	BEU_BASE+0x0054
#define VIO_BSAYR3	BEU_BASE+0x0058
#define VIO_BSACR3	BEU_BASE+0x005c
#define VIO_BSIFR3	BEU_BASE+0x0060
#define VIO_BTPSR	BEU_BASE+0x0064
#define VIO_BBLCR0	BEU_BASE+0x0070
#define VIO_BBLCR1	BEU_BASE+0x0074
#define VIO_BLOCR1	BEU_BASE+0x0078
#define VIO_BLOCR2	BEU_BASE+0x007c
#define VIO_BLOCR3	BEU_BASE+0x0080

#define VIO_BPKFR	BEU_BASE+0x0084
#define VIO_BAPCR1	BEU_BASE+0x0088
#define VIO_BAPCR2	BEU_BASE+0x008c
#define VIO_BDMWR	BEU_BASE+0x0090
#define VIO_BDAYR	BEU_BASE+0x0094
#define VIO_BDACR	BEU_BASE+0x0098
#define VIO_BAFXR	BEU_BASE+0x0200
#define VIO_BSWPR	BEU_BASE+0x0210
#define VIO_BEIER	BEU_BASE+0x0220
#define VIO_BEVTR	BEU_BASE+0x0224
#define VIO_BSTAR	BEU_BASE+0x0240
#define VIO_BBRSTR	BEU_BASE+0x0300

#define VIO_CLUT	0xfe931000

#endif				/* __SH_MOBILE_JV_VIO4_H__ */
