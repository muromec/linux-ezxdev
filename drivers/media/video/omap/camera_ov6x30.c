/*
 * camera_ov6x30.c
 *
 * Implementation of Camera for OmniVision OV6630 digital camera.
 *
 * Copyright (C) 2003-2004 MontaVista Software, Inc.
 *
 * Author: MontaVista Software, Inc.
 *              stevel@mvista.com or source@mvista.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/videodev.h>
#include <asm/uaccess.h>
#define MODULE_NAME "ov6x30"
#include "common.h"
#include "camif.h"

#define OV6X30_BPP 2

#define REG_GAIN		0x00	/* gain setting (5:0) */
#define REG_BLUE		0x01	/* blue channel balance */
#define REG_RED			0x02	/* red channel balance */
#define REG_SAT			0x03	/* saturation */
					/* 04 reserved */
#define REG_CNT			0x05	/* Y contrast */
#define REG_BRT			0x06	/* Y brightness */
					/* 08-0b reserved */
#define REG_BLUE_BIAS		0x0C	/* blue channel bias (5:0) */
#define REG_RED_BIAS		0x0D	/* read channel bias (5:0) */
#define REG_GAMMA_COEFF		0x0E	/* gamma settings */
#define REG_WB_RANGE		0x0F	/* AEC/ALC/S-AWB settings */
#define REG_EXP			0x10	/* manual exposure setting */

/* Window parameters */
#define HWSBASE 0x38
#define HWEBASE 0x3A
#define VWSBASE 0x05
#define VWEBASE 0x06

#define HORZCONST 0x38 // as per CCD chip specs.
#define VERTCONST 0x03 // as per CCD chip specs.
#define MAXWIDTH  CIF_WIDTH
#define MAXHEIGHT CIF_HEIGHT

static struct camera * this;

#define DEF_BRIGHTNESS  149
#define DEF_CONTRAST     10
#define DEF_SATURATION   24
#define DEF_BLUE        153
#define DEF_RED         (255 - DEF_BLUE)
#define DEF_AWB           1
#define DEF_EXPOSURE    154
#define DEF_GAIN         31
#define DEF_AUTOGAIN      1
#define DEF_HFLIP         1
#define DEF_BANDFILT      1
#define DEF_AUTOBRIGHT    1
#define DEF_FREEZE_AGCAEC 0
#define DEF_DISABLE_AEC   0

// Our own specific controls
#define V4L2_CID_BANDFILT   V4L2_CID_PRIVATE_BASE+0
#define V4L2_CID_AUTOBRIGHT V4L2_CID_PRIVATE_BASE+1
#define V4L2_CID_DISABLE_AEC V4L2_CID_PRIVATE_BASE+2
#define V4L2_CID_FREEZE_AGCAEC V4L2_CID_PRIVATE_BASE+3
#define V4L2_CID_LAST_PRIV  V4L2_CID_FREEZE_AGCAEC

static int current_frame_period = 333667; // .1 usec (30 fps)
static int current_xclk = 24000000; // Hz

/*  Video controls  */
static struct vcontrol {
	int current_value;
	u8 reg;
	u8 mask;
	u8 start_bit;
	struct v4l2_queryctrl qc;
} control[] = {
	{ 0, 0x05, 0x0f, 0,
	  { V4L2_CID_CONTRAST,   "Contrast",   0,  15, 1, DEF_CONTRAST,
	    V4L2_CTRL_TYPE_INTEGER }},
	{ 0, REG_BRT, 0xff, 0,
	  { V4L2_CID_BRIGHTNESS, "Brightness", 0, 255, 1, DEF_BRIGHTNESS,
	    V4L2_CTRL_TYPE_INTEGER }},
	{ 0, REG_SAT, 0xf8, 3,
	  { V4L2_CID_SATURATION, "Saturation", 0,  31, 1, DEF_SATURATION,
	    V4L2_CTRL_TYPE_INTEGER }},
	{ 0, 0x12, 0x04, 2,
	  { V4L2_CID_AUTO_WHITE_BALANCE, "Auto White Balance", 0, 1, 0, DEF_AWB,
	    V4L2_CTRL_TYPE_BOOLEAN }},
	{ 0, REG_BLUE, 0xff, 0,
	  { V4L2_CID_BLUE_BALANCE, "Blue Balance", 0, 255, 1, DEF_BLUE,
	    V4L2_CTRL_TYPE_INTEGER }},
	{ 0, REG_RED, 0xff, 0,
	  { V4L2_CID_RED_BALANCE, "Red Balance", 0, 255, 1, DEF_RED,
	    V4L2_CTRL_TYPE_INTEGER }},
	{ 0, 0x12, 0x20, 5,
	  { V4L2_CID_AUTOGAIN, "Auto Gain", 0, 1, 0, DEF_AUTOGAIN,
	    V4L2_CTRL_TYPE_BOOLEAN }},
	{ 0, 0x00, 0xff, 0,
	  { V4L2_CID_GAIN, "Gain",             0,  63, 1, DEF_GAIN,
	    V4L2_CTRL_TYPE_INTEGER }},
	{ 0, 0x29, 0x80, 7,
	  { V4L2_CID_DISABLE_AEC, "Disable Auto Exposure", 0, 1, 0, DEF_DISABLE_AEC,
	    V4L2_CTRL_TYPE_BOOLEAN }},
	{ 0, REG_EXP, 0xff, 0,
	  { V4L2_CID_EXPOSURE, "Exposure",     0, 255, 1, DEF_EXPOSURE,
	    V4L2_CTRL_TYPE_INTEGER }},
	{ 0, 0x28, 0x10, 4,
	  { V4L2_CID_FREEZE_AGCAEC, "Freeze AGC/AEC", 0, 1, 0, DEF_FREEZE_AGCAEC,
	    V4L2_CTRL_TYPE_BOOLEAN }},
	{ 0, 0x12, 0x40, 6,
	  { V4L2_CID_HFLIP, "Mirror Image",    0,   1, 0, DEF_HFLIP,
	    V4L2_CTRL_TYPE_BOOLEAN }},
	{ 0, 0x2d, 0x04, 2,
	  { V4L2_CID_BANDFILT, "Band Filter",  0,   1, 0, DEF_BANDFILT,
	    V4L2_CTRL_TYPE_BOOLEAN }},
	{ 0, 0x2d, 0x10, 4,
	  { V4L2_CID_AUTOBRIGHT, "Auto Black Expand", 0, 1, 0, DEF_AUTOBRIGHT,
	    V4L2_CTRL_TYPE_BOOLEAN }},
};

#define NUM_CONTROLS (sizeof(control)/sizeof(control[0]))

static struct {
	u8 reg;
	u8 val;
	int no_verify;
} init_regtbl[] = {
	{ 0x12, 0x80, 1 }, /* reset */
	{ 0x00, DEF_GAIN }, /* Gain */
	{ 0x01, DEF_BLUE }, /* Blue gain */
	{ 0x02, DEF_RED }, /* Red gain */
	{ 0x03, DEF_SATURATION<<3 }, /* Saturation */
	{ 0x05, DEF_CONTRAST }, /* Contrast */
	{ 0x06, DEF_BRIGHTNESS}, /* Brightness */
	{ 0x07, 0x2d }, /* Sharpness */
	{ 0x0c, 0x20 },
	{ 0x0d, 0x20 },
	{ 0x0e, 0x20 },
	{ 0x0f, 0x05 },
	{ 0x10, DEF_EXPOSURE }, // exposure check
	{ 0x11, 0x40 }, /* Pixel clock = fastest, VSYNC negative */
	{ 0x12, (DEF_AWB<<2) | (DEF_AUTOGAIN<<5) | (DEF_HFLIP<<6) | (1<<3) },
	{ 0x13, 0x29 },
	{ 0x14, 0xb0 },
	{ 0x15, 0x01 },
	{ 0x16, 0x03 },
	{ 0x17, 0x38 },
	{ 0x18, 0xe8 }, // ea
	{ 0x19, 0x03 }, // 04
	{ 0x1a, 0x92 }, // 93
	{ 0x1b, 0x00 },
	{ 0x1c, 0x7f },
	{ 0x1d, 0xa2 },
	{ 0x1e, 0xc4 },
	{ 0x1f, 0x04 },
	{ 0x20, 0x20 },
	{ 0x21, 0x10 },
	{ 0x22, 0x88 },
	{ 0x23, 0xc0 }, /* Crystal circuit power level */
	{ 0x25, 0x9a }, /* Increase AEC black pixel ratio */
	{ 0x26, 0xb2 }, /* BLC enable */
	{ 0x27, 0xa2 },
	{ 0x28, (DEF_FREEZE_AGCAEC<<4) | (1<<0) },
	{ 0x29, (DEF_DISABLE_AEC<<7) },
	{ 0x2c, 0xa0 },
	/* This next setting is critical. It seems to improve
	 * the gain or the contrast. The "reserved" bits seem
	 * to have some effect in this case. */
	{ 0x2d, (DEF_AUTOBRIGHT<<4) | (DEF_BANDFILT<<2) | (1<<7) | (1<<0) },
	{ 0x2e, 0x88 },
	{ 0x33, 0x26 },
	// Reg 0x34: More 6620 reserved junk
	{ 0x34, 0x03 },
	{ 0x36, 0x8f }, 
	{ 0x37, 0x80 },
	{ 0x38, 0x91 },
	{ 0x39, 0x80 },
	{ 0x3a, 0x0f }, 
	{ 0x3b, 0x3c },
	{ 0x3c, 0x1a },
	{ 0x3d, 0x80 },
	{ 0x3e, 0x80 },
	{ 0x3f, 0x0e }, 
	{ 0x40, 0x00 }, /* White bal */
	{ 0x41, 0x00 }, /* White bal */
	{ 0x42, 0x80 },
	{ 0x43, 0x3f }, /* White bal */
	{ 0x44, 0x80 },
	{ 0x45, 0x20 },
	{ 0x46, 0x20 },
	{ 0x47, 0x80 },
	{ 0x48, 0x7f },
	{ 0x49, 0x00 },
	{ 0x4a, 0x00 }, 
	{ 0x4b, 0x80 },
	{ 0x4c, 0xd0 },
	{ 0x4d, 0x10 }, /* U = 0.563u, V = 0.714v */
	{ 0x4e, 0x40 }, /* AEC/AGC reference voltage level */
	{ 0x4f, 0x07 }, /* UV average mode, color killer: strongest */
	{ 0x50, 0xff },
	{ 0x54, 0x23 }, /* Max AGC gain: 18dB */
	{ 0x55, 0xff },
	{ 0x56, 0x12 },
	{ 0x57, 0x81 }, /* (default) */
	{ 0x58, 0x75 },
	{ 0x59, 0x01 }, /* AGC dark current compensation: +1 */
	{ 0x5a, 0x2c },
	{ 0x5b, 0x0f }, /* AWB chrominance levels */
	{ 0x5c, 0x10 },
	{ 0x3d, 0x80 },
	{ 0x27, 0xa6 },
	// Toggle AWB off and on
	{ 0x12, (0<<2) | (DEF_AUTOGAIN<<5) | (DEF_HFLIP<<6) | (1<<3) },
	{ 0x12, (DEF_AWB<<2) | (DEF_AUTOGAIN<<5) | (DEF_HFLIP<<6) | (1<<3) },
};

#define INIT_REGTBL_SIZE (sizeof(init_regtbl)/sizeof(init_regtbl[0]))

typedef struct {
	u8 R;
	u8 G1;
	u8 G2;
	u8 B;
} ov6x30_rgb_t;

static inline void get_qcif_rgb(u8* s, ov6x30_rgb_t* rgb)
{
	rgb->G2 = *s++;
	rgb->B  = *s++;
	rgb->G1 = *s++;
	rgb->R  = *s;
}

static inline void get_cif_rgb(u8* s, ov6x30_rgb_t* rgb)
{
	rgb->G2 = *s++;
	rgb->R  = *s++;
	rgb->G1 = *s++;
	rgb->B  = *s;
}

static inline void convert_to_RGB565(u8* s, u16* d, int to_user, int qcif)
{
	ov6x30_rgb_t rgb;
	u16 p1, p2;
	
	if (qcif)
		get_qcif_rgb(s, &rgb);
	else
		get_cif_rgb(s, &rgb);

	p1  = ((rgb.R & 0xf8)  << 8); // upper 5 bits of red at bit 11
	p1 |= ((rgb.G1 & 0xFC) << 3); // upper 6 bits of grn at bit 5
	p1 |= ((rgb.B & 0xf8)  >> 3); // upper 5 bits of blu at bit 0
	
	p2  = ((rgb.R & 0xf8)  << 8);
	p2 |= ((rgb.G2 & 0xFC) << 3);
	p2 |= ((rgb.B & 0xf8)  >> 3);

	if (to_user) {
		put_user(p1, d++);
		put_user(p2, d);
	} else {
		*d++ = p1;
		*d = p2;
	}
}

static inline void convert_to_BGR24(u8* s, u8* d, int to_user, int qcif)
{
	ov6x30_rgb_t rgb;
	u8 pixbuf[6];
	
	if (qcif)
		get_qcif_rgb(s, &rgb);
	else
		get_cif_rgb(s, &rgb);
	
	pixbuf[0] = rgb.R;
	pixbuf[1] = rgb.G1;
	pixbuf[2] = rgb.B;

	pixbuf[3] = rgb.R;
	pixbuf[4] = rgb.G2;
	pixbuf[5] = rgb.B;
	
	if (to_user)
		copy_to_user(d, pixbuf, sizeof(pixbuf));
	else
		memcpy(d, pixbuf, sizeof(pixbuf));
}



// **************************
// Routine:
// Description:
// **************************
static int ov6x30_convert_image(u8* src,
				void* dest,
				int to_user,
				int dest_stride,
				struct v4l2_pix_format* fmt)
{
	int x, y;
	int line_gap;
	int dest_BPP = (fmt->depth+7) >> 3;
	int width = omap_image_size[this->imgfmt].width;
	int height = omap_image_size[this->imgfmt].height;
	int qcif = (this->imgfmt == QCIF);
	
	line_gap = dest_stride - (width * dest_BPP);
	
#ifndef CONFIG_OMAP_INNOVATOR
	// Special Case, Since the first 32bit word transfered
	// from the DMA is garbage. The second 32bit word is
	// really the start of our image frame data. So, advance
	// the pointer to that first pixel.
	// This of course means that the last two
	// pixels of the frame will now be random data since we will
	// be reading 4 bytes beyond the real end of the dma buffer
	// but we do this since having a frame with only two bad
	// pixels is better than displaying the *whole* image
	// shifted by two pixels. This feels like a work around
	// for a h/w issue. Why is that first 32bit value xfered
	// by the DMA an extra one that always proceeds the real
	// data we are interested in? In my analysis that first
	// pixel is a left over one from the prior frame. It is
	// as if enabling the DMA xfer *immediately* triggers a
	// transfer of that left over 32bit value which is then
	// followed by the real pixels of the new image we're
	// capturing on each new Lclk rising edge.
	src+=4;
#endif  

	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_RGB565:
		// Convert camera data into rgb565.
		for (y = 0; y < height; y++) {
			for (x = 0 ; x < width/2 ; x++) {
				convert_to_RGB565(src, (u16*)dest,
						  to_user, qcif);
				src += 2*OV6X30_BPP;
				dest += 2*dest_BPP;
			}
			dest += line_gap;
		}
		break;
	case V4L2_PIX_FMT_BGR24:
		// Convert camera data into BGR24.
		for (y = 0; y < height; y++) {
			for (x = 0 ; x < width/2 ; x++) {
				convert_to_BGR24(src, (u8*)dest,
						 to_user, qcif);
				src += 2*OV6X30_BPP;
				dest += 2*dest_BPP;
			}
			dest += line_gap;
		}
		break;
	default:
		err("unsupported conversion request.\n");
		return -ENXIO;
	}

	return 0;
}


#if 0
static void ov6x30_dump(void)
{
	struct camera_serial_bus * sbus = this->camif->sbus;
	u8 buf[0x60];

	memset(buf, 0, sizeof(buf));
	sbus->read(0x00, buf, 0x5c);
	DUMP_BUF(buf, sizeof(buf));
}
#endif

static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE || id > V4L2_CID_LAST_PRIV)
		return -EDOM;
	for (i = NUM_CONTROLS - 1; i >= 0; i--)
		if (control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

static int
ov6x30_querymenu(struct v4l2_querymenu *qm)
{
#if 0
	static char *expo_menu[] = {
		"1/60",
		"1/100",
		"1/250",
		"1/1000",
		"1/5000",
	};

	if (qm->id == V4L2_CID_EXPOSURE) {
		if (qm->index < 0 ||
		    qm->index >= sizeof(expo_menu)/sizeof(char *))
			return -EINVAL;
		memcpy(qm->name, expo_menu[qm->index], sizeof(qm->name));
		return 0;
	}
#else
	// No menu controls have been defined
	return -EINVAL;
#endif
}

static int
ov6x30_query_control(struct v4l2_queryctrl *qc)
{
	int i;
	i = find_vctrl(qc->id);
	if (i == -EINVAL) {
		qc->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}
	if (i < 0)
		return -EINVAL;

	/*  V4L2 filled in category and group, preserve them */
	control[i].qc.category = qc->category;
	memcpy(control[i].qc.group, qc->group, sizeof(qc->group));

	*qc = control[i].qc;
	return 0;
}


static int ov6x30_write_control(struct vcontrol * lvc, u8 value)
{
	struct camera_serial_bus * sbus = this->camif->sbus;
	u8 oldval, newval;
	u8 reg = lvc->reg;
	u8 mask = lvc->mask;
	int rc;
	
	value <<= lvc->start_bit;

	if (mask == 0xff) {
		newval = value;
	} else {
		if ((rc = sbus->read(reg, &oldval, 1)))
			return rc;
		
		oldval &= ~mask;              /* Clear the masked bits */
		value &= mask;                /* Enforce mask on value */
		newval = oldval | value;      /* Set the desired bits */
	}
	
	if ((rc = sbus->write(reg, &newval, 1)))
		return rc;
	if ((rc = sbus->read(reg, &newval, 1)))
		return rc;

	return (newval & mask) >> lvc->start_bit;
}

static int ov6x30_read_control(struct vcontrol * lvc)
{
	struct camera_serial_bus * sbus = this->camif->sbus;
	u8 val;
	u8 reg = lvc->reg;
	u8 mask = lvc->mask;
	int rc;
	
	if ((rc = sbus->read(reg, &val, 1)))
		return rc;
	return (val & mask) >> lvc->start_bit;
}

static int
ov6x30_control(struct v4l2_control *vc, int write)
{
	int i, val;
	struct vcontrol * lvc;
	
	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &control[i];
	
	if (write)
		val = ov6x30_write_control(lvc, vc->value);
	else
		val = ov6x30_read_control(lvc);
	
	if (val >= 0) {
		vc->value = lvc->current_value = val;
		return 0;
	} else
		return val;
}


static int
ov6x30_set_control(struct v4l2_control *vc)
{
	return ov6x30_control(vc, 1);
}
static int
ov6x30_get_control(struct v4l2_control *vc)
{
	return ov6x30_control(vc, 0);
}


static int
ov6x30_restore_current_controls(void)
{
	struct v4l2_control vc;
	int i, rc=0;
	
	for (i=0; i<NUM_CONTROLS; i++) {
		vc.id = control[i].qc.id;
		vc.value = control[i].current_value;
		if ((rc = ov6x30_set_control(&vc)))
			break;
	}

	return rc;
}

static int
ov6x30_save_current_controls(void)
{
	struct v4l2_control vc;
	int i, rc=0;
	
	for (i=0; i<NUM_CONTROLS; i++) {
		vc.id = control[i].qc.id;
		if ((rc = ov6x30_get_control(&vc)))
			break;
	}

	return rc;
}


#define MIN_FP_24MHZ  245820 // 40.68 fps
#define MIN_FP_12MHZ  492410 // 20.31 fps
#define MAX_FP_24MHZ  512560 // 19.51 fps
#define MAX_FP_12MHZ 1022890 //  9.78 fps

#define MIN_FP 250000 // 40 fps
#define MAX_FR 40
#define MAX_FP 666667 // 15 fps
#define MIN_FR 15

/* starts at MAX_FP, increments by 0.2 sec (5 Hz) */
static struct {
	int exclk; // MHz
	int regval;
} fp_tbl[6] = {
	{ 12000000, 165 }, // 15 fps
	{ 12000000,   7 }, // 20 fps
	{ 24000000, 295 }, // 25 fps
	{ 24000000, 167 }, // 30 fps
	{ 24000000,  77 }, // 35 fps
	{ 24000000,   8 }  // 40 fps
};


static int ov6x30_set_fp(int fp, int xclk, int test)
{
	int ret;
	unsigned int fr_x100, i;
	
	ENTRY();

	if (fp < MIN_FP || fp > MAX_FP) {
		err("fp out of range: %d\n", fp);
		return -EINVAL;
	}

	if (fp == 333667)
		fr_x100 = 30 * 100;
	else
		fr_x100 = (1000000000 + fp/2) / fp; // desired framerate * 100

	if (fr_x100 % 500) {
		err("fp must be in intervals of 0.2 sec (5 Hz): %d\n", fp);
		return -EINVAL;
	}

	i = (fr_x100 - MIN_FR*100) / 500;

	if (xclk != fp_tbl[i].exclk) {
		err("need %d MHz XCLK for fp %d\n", fp_tbl[i].exclk, fp);
		return -EINVAL;
	}

	if (!test) {
		struct camera_serial_bus * sbus = this->camif->sbus;
		unsigned int regval = fp_tbl[i].regval;
		
		ret = sbus->write_verify(0x2a,
					 0x84 | ((regval & (1<<8))?(1<<5):0));
		if (ret < 0)
			return ret;
		
		ret = sbus->write_verify(0x2b, regval & ~(1<<8));
		if (ret < 0)
			return ret;
		
		current_frame_period = fp;
		current_xclk = xclk;
	}

	return fp;
}


static int ov6x30_get_fp(void)
{
	return current_frame_period;
}


static int ov6x30_set_mode(void)
{
#if 0
	int H_start,H_end;
	int V_start,V_end;
	int width = omap_image_size[this->imgfmt].width;
	int height = omap_image_size[this->imgfmt].height;
#endif
	struct camera_serial_bus * sbus = this->camif->sbus;
	int qcif = (this->imgfmt == QCIF);
	int ret = 0;
	u8 reg14;
	
	if ((ret = sbus->read(0x14, &reg14, 1)))
		return ret;
	if (qcif)
		reg14 |= 0x20;
	else
		reg14 &= ~0x20;
	if ((ret = sbus->write_verify(0x2b, reg14)) < 0)
		return ret;

#if 0
       // FIXME: fix this
       // Calculate a centered image within the full CCD grid.

	H_start = ((MAXWIDTH - width)/4) + HORZCONST;
	H_end = H_start + (width/2);
	if (H_start < HORZCONST) {
		H_start = HORZCONST; // keep within bounds.
		H_end = MAXWIDTH/2;
	}
	
	V_start = ((MAXHEIGHT - height)/4) + VERTCONST;
	V_end = V_start + (height/2);
	
	if (V_start < VERTCONST) {  // keep within bounds.
		V_start = VERTCONST;
		V_end = MAXHEIGHT/2;
	}

	ret = sbus->write_verify(0x17, (u8)H_start);
	ret = sbus->write_verify(0x18, (u8)H_end);
	ret = sbus->write_verify(0x19, (u8)V_start);
	ret = sbus->write_verify(0x1A, (u8)V_end);
#endif

	return ret;
}

/*
 * This camera only supports RGB565 pixel format, but it can
 * convert to any of the following V4L2 pixel formats.
 */
static int
ov6x30_find_format(struct v4l2_pix_format* fmt)
{
	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_BGR24:
		return RGB565;
	}

	return -1;
}

// **************************
// Routine:
// Description:
//   This the setup that must occur to adjust the
//   reg settings as per any new VIDIOC_S_FMT command
//   the client may have just performed to the driver.
// **************************
static int ov6x30_setup(struct v4l2_pix_format* fmt)
{
	enum pixel_format pixfmt;
	enum image_format imgfmt;

	if ((pixfmt = ov6x30_find_format(fmt)) < 0)
		return -EINVAL;

	if (fmt->width >= omap_image_size[CIF].width) {
		imgfmt = CIF;
	} else {
		imgfmt = QCIF;
	}

	fmt->width = omap_image_size[imgfmt].width;
	fmt->height = omap_image_size[imgfmt].height;

	this->imgfmt = imgfmt;
	this->pixfmt = pixfmt;
	
	return ov6x30_set_mode();
}


static int ov6x30_open(void)
{
	struct camera_serial_bus * sbus = this->camif->sbus;
	int i, rc;

	for (i=0; i < INIT_REGTBL_SIZE; i++) {
		if (init_regtbl[i].no_verify)
			sbus->write(init_regtbl[i].reg,
				    &init_regtbl[i].val, 1);
		else
			sbus->write_verify(init_regtbl[i].reg,
					   init_regtbl[i].val);
			
	}

	if ((rc = ov6x30_set_fp(current_frame_period, current_xclk, 0)) < 0) {
		err("failed to set frame period\n");
		return rc;
	}
	if ((rc = ov6x30_set_mode()))
		return rc;
	return ov6x30_restore_current_controls();
}



static int ov6x30_close(void)
{
	/*
	 * save the current controls so they can be restored on
	 * next open.
	 */
	return ov6x30_save_current_controls();
}

static int ov6x30_init(void)
{
	int i;

	for (i=0; i<NUM_CONTROLS; i++)
		control[i].current_value = control[i].qc.default_value;
	
	this->imgfmt = QCIF;
	this->pixfmt = RGB565;
	return 0;
}

static void ov6x30_cleanup(void)
{
	// nothing to do
}


static int ov6x30_detect(void)
{
	struct camera_serial_bus * sbus;

	this = &camera_ov6x30;
	sbus = this->camif->sbus;
	
	sbus->set_devid(CAMERA_OV6630_DEV_ID);

	// Just an arbitrary OmniVision camera command
	return sbus->write_verify(0x11, 0x40);
}

struct camera camera_ov6x30 = {
	imgfmt: QCIF,
	pixfmt: RGB565,
	
	detect:  ov6x30_detect,
	init:    ov6x30_init,
	cleanup: ov6x30_cleanup,

	open:    ov6x30_open,
	close:   ov6x30_close,
	set_format:   ov6x30_setup,
	set_frame_period: ov6x30_set_fp,
	get_frame_period: ov6x30_get_fp,
	
	convert_image: ov6x30_convert_image,

	query_control: ov6x30_query_control,
	get_control:   ov6x30_get_control,
	set_control:   ov6x30_set_control,
	query_menu:    ov6x30_querymenu,
};

