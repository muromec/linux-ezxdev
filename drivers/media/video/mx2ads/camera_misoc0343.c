/*
 *  drivers/media/video/mx2ads/camera_misoc0343.c
 *
 *  driver for Motorola MX2ADS on-board CMOS Image Sensor
 *
 *  Author: MontaVista Software, Inc. <source@mvista.com>
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/config.h>

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/videodev.h>
#include <asm/uaccess.h>

#include <asm/hardware.h>
#define MODULE_NAME "misoc0343"

#include <asm/hardware/misoc0343.h>

#include "common.h"
#include "camif.h"

#define DEF_BLUE        (0)
#define DEF_RED         (0)
#define DEF_AWB         (0)
#define DEF_HFLIP       (0)
#define DEF_VFLIP       (0)

static struct camera * this;

extern const struct image_size mx2ads_image_size[]; 
extern const int mx2ads_pixfmt_depth[];

#define V4L2_CID_LAST_PRIV  V4L2_CID_PRIVATE_BASE+0

enum regdomain {IC, IFP};

/*  Video controls  */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
        enum regdomain domain;
	u16 reg;
	u16 mask;
	u8 start_bit;
} control[] = {
        { { V4L2_CID_RED_BALANCE, "Red Balance", 0, 255, 1, DEF_RED,
            V4L2_CTRL_TYPE_INTEGER },
          0, IFP, MISOC0343_R33, 0xff00, 8 },
        { { V4L2_CID_BLUE_BALANCE, "Blue Balance", 0, 255, 1, DEF_BLUE,
            V4L2_CTRL_TYPE_INTEGER },
          0, IFP, MISOC0343_R33, 0xff, 0 },
        { { V4L2_CID_AUTO_WHITE_BALANCE, "Auto White Balance", 0,1,0, DEF_AWB,
            V4L2_CTRL_TYPE_BOOLEAN },
          0, IFP, MISOC0343_R6, 0x02, 1 },
        { { V4L2_CID_HFLIP, "Mirror Image", 0, 1, 0, DEF_HFLIP,
            V4L2_CTRL_TYPE_BOOLEAN },
          0, IC, MISOC0343_IC_RDMODE, 0x4000, 14 },
        { { V4L2_CID_VFLIP, "Vertical Flip", 0, 1, 0, DEF_VFLIP,
            V4L2_CTRL_TYPE_BOOLEAN },
          0, IC, MISOC0343_IC_RDMODE, 0x8000, 15 },
};

#define NUM_CONTROLS (sizeof(control)/sizeof(control[0]))

#define MISOC0343_WRITE16(_sw, _reg, _val)                             \
    do {                                                               \
	    struct camera_serial_bus * sbus = this->camif->sbus;       \
            u8 sw[2] = {(u8)(((MISOC0343_##_sw) >> 8) & 0xFF),         \
                         (u8)(MISOC0343_##_sw & 0xFF)};                \
            u8 data[2] = {(u8)(((_val) >> 8) & 0xFF),                  \
                         (u8)((_val) & 0xFF)};                         \
	    sbus->write(MISOC0343_R1, sw, 2);                          \
	    sbus->write((_reg), data, 2);                              \
    } while(0)

#define MISOC0343_READ16(_sw, _reg, _val)                              \
    do {                                                               \
	    struct camera_serial_bus * sbus = this->camif->sbus;       \
            u8 sw[2] = {(u8)(((MISOC0343_##_sw) >> 8) & 0xFF),         \
                         (u8)(MISOC0343_##_sw & 0xFF)};                \
	    sbus->write(MISOC0343_R1, sw, 2);                          \
	    sbus->read((_reg), (uint8_t*)&(_val), 2);                  \
    } while(0)

#define MISOC0343_IFP_WRITE16(_reg, _val) \
    MISOC0343_WRITE16(IFP, (_reg), (_val))

#define MISOC0343_IC_WRITE16(_reg, _val) \
    MISOC0343_WRITE16(IC, (_reg), (_val))

#define MISOC0343_IFP_READ16(_reg, _val) \
    MISOC0343_READ16(IFP, (_reg), (_val))

#define MISOC0343_IC_READ16(_reg, _val) \
    MISOC0343_READ16(IC, (_reg), (_val))

/*
 * Reset camera
 */
static void
misoc0343_reset(void)
{
        /*
	 * Reset Image core
	 */
	MISOC0343_IC_WRITE16(MISOC0343_IC_RESET, 0x01);
	MISOC0343_IC_WRITE16(MISOC0343_IC_RESET, 0x00);

	/*
	 * Reset Image Flow
	 */
	MISOC0343_IFP_WRITE16(MISOC0343_IFP_RESET, 0x01);
	MISOC0343_IFP_WRITE16(MISOC0343_IFP_RESET, 0x00);

	return;
}

/*
 * Initialise board-specfic settings to init Micron camera
 */
static int
misoc0343_init(void)
{
	return 0;
}

static int misoc0343_set_fp(int fp, int xclk, int test)
{
        dbg("Not implemented yet\n");
        return -1;
}

static int misoc0343_get_fp(void)
{
        dbg("Not implemented yet\n");
	return -1;
}

static int misoc0343_write_control(struct vcontrol * lvc, u16 value)
{
	u16 oldval, newval;
	u16 reg = lvc->reg;
	u16 mask = lvc->mask;
	
	value <<= lvc->start_bit;

        if (lvc->domain == IC) {
            MISOC0343_IC_READ16(reg, oldval);
        } else { /*IFP */
            MISOC0343_IFP_READ16(reg, oldval);
        }
        oldval &= ~mask;
        value &= mask;                /* Enforce mask on value */
        newval = oldval | value;      /* Set the desired bits */

        if (lvc->domain == IC) {
            MISOC0343_IC_WRITE16(reg, newval);
            MISOC0343_IC_READ16(reg, newval);
        } else { /*IFP */
            MISOC0343_IFP_WRITE16(reg, newval);
            MISOC0343_IFP_READ16(reg, newval);
        }
	return (newval & mask) >> lvc->start_bit;
}

static int misoc0343_read_control(struct vcontrol * lvc)
{
	u16 val;
	u16 reg = lvc->reg;
	u16 mask = lvc->mask;
	
        if (lvc->domain == IC) {
            MISOC0343_IC_READ16(reg, val);
        } else { /*IFP */
            MISOC0343_IFP_READ16(reg, val);
        }
	return (val & mask) >> lvc->start_bit;
}

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
misoc0343_control(struct v4l2_control *vc, int write)
{
	int i, val;
	struct vcontrol * lvc;
	
	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &control[i];
	
	if (write)
		val = misoc0343_write_control(lvc, vc->value);
	else
		val = misoc0343_read_control(lvc);
	
	if (val >= 0) {
		vc->value = lvc->current_value = val;
		return 0;
	} else
		return val;
}

static int
misoc0343_set_control(struct v4l2_control *vc)
{
	return misoc0343_control(vc, 1);
}

static int
misoc0343_get_control(struct v4l2_control *vc)
{
	return misoc0343_control(vc, 0);
}


static int
misoc0343_querymenu(struct v4l2_querymenu *qm)
{
	/* No menu controls have been defined */
	return -EINVAL;
}

static int
misoc0343_query_control(struct v4l2_queryctrl *qc)
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


static int
misoc0343_find_size(struct v4l2_pix_format *fmt)
{
        int isize;

        for (isize = QQVGA; isize < VGA; isize++) {
                if ((mx2ads_image_size[isize].height >= fmt->height) &&
		    (mx2ads_image_size[isize].width >= fmt->width)) {
			fmt->width = mx2ads_image_size[isize].width;
			fmt->height = mx2ads_image_size[isize].height;
                        return isize;
                }
        }

        fmt->width = mx2ads_image_size[VGA].width;
        fmt->height = mx2ads_image_size[VGA].height;
        return VGA;
}

int
misoc0343_find_format(struct v4l2_pix_format* fmt)
{
	switch (fmt->pixelformat) {
            case V4L2_PIX_FMT_YUYV:
		return YUV;
            case V4L2_PIX_FMT_RGB565:
            case V4L2_PIX_FMT_BGR24: /* will have to convert to BGR24 */
		return RGB565;
	}

	return -1;
}


static int
misoc0343_set_format(struct v4l2_pix_format *fmt)
{
	enum pixel_format pixfmt;
	uint16_t reg_val;
        
        this->imgfmt = misoc0343_find_size(fmt); 

        switch(this->imgfmt) {
            case QQVGA:
		reg_val = MISOC0343_DECIMATION_QQVGA;
                break;
            case QCIF:
		reg_val = MISOC0343_DECIMATION_QCIF;
                break;
            case QVGA:
		reg_val = MISOC0343_DECIMATION_QVGA;
                break;
            case CIF:
		reg_val = MISOC0343_DECIMATION_CIF;
                break;
            case VGA:
		reg_val = MISOC0343_DECIMATION_VGA;
                break;
            default:
                return -EINVAL;
        }

	MISOC0343_IFP_WRITE16(MISOC0343_IFP_DECIMATION, reg_val);

	/*
	 * Invert picture
	 */
	MISOC0343_IC_WRITE16(MISOC0343_IC_RDMODE,
			     (MISOC0343_RDMODE_UPSDOWN |
			      MISOC0343_RDMODE_MIRRORED |
			      MISOC0343_RDMODE_BOOSTRST |
			      MISOC0343_RDMODE_ALLFRAMES));

	MISOC0343_IFP_WRITE16(MISOC0343_IFP_SHWIDTH, MISOC0343_SHWIDTH_DEFAULT);

        /*
         * turn camera into approprative mode
         */
	MISOC0343_IFP_READ16(MISOC0343_IFP_FORMAT, reg_val);

	if ((pixfmt = misoc0343_find_format(fmt)) < 0)
		return -EINVAL;
        switch(pixfmt) {
                case YUV:
                    reg_val = MISOC0343_FORMAT_FLCKDTC |
                        MISOC0343_FORMAT_YUV422 |
                        MISOC0343_FORMAT_LENSSHADE;
                    break;
                case RGB565:
	            reg_val |= MISOC0343_FORMAT_565RGB;
                    break;
                default:
		    return -EINVAL;
        }
	MISOC0343_IFP_WRITE16(MISOC0343_IFP_FORMAT, reg_val);

	return 0;
}

/* 
 * Open - open device
 */
static int misoc0343_open(void)
{
    return 0;
}


/* 
 * Close - close the device
 */
static int misoc0343_close(void)
{
    return 0;
}

static void misoc0343_cleanup(void)
{
	/* nothing to do */
}

/*
 * Check - whether Micron compatible camera present
 */
static int
misoc0343_detect(void)
{
	uint16_t reg_val;
	struct camera_serial_bus * sbus;

	this = &camera_misoc0343;
	sbus = this->camif->sbus;
	
	sbus->set_devid(CAMERA_MISOC0343_DEV_ID);

        misoc0343_init();
        
	misoc0343_reset();

	MISOC0343_IC_READ16(MISOC0343_IC_VERSION, reg_val);

	if (reg_val != MISOC0343_CORE_VERSION)
		return -1;

	MISOC0343_IFP_READ16(MISOC0343_IFP_VERSION, reg_val);

	if (reg_val != MISOC0343_IMAGE_VERSION)
		return -1;

	return 0;
}

struct camera camera_misoc0343 = {
	.imgfmt             = QVGA,
	.pixfmt             = RGB565,
	
	.detect             = misoc0343_detect,
	.init               = misoc0343_init,
	.cleanup            = misoc0343_cleanup,

	.open               = misoc0343_open,
	.close              = misoc0343_close,
	.set_format         = misoc0343_set_format,
	.set_frame_period   = misoc0343_set_fp,
	.get_frame_period   = misoc0343_get_fp,
	
	.query_control      = misoc0343_query_control,
	.get_control        = misoc0343_get_control,
	.set_control        = misoc0343_set_control,
	.query_menu         = misoc0343_querymenu,
};

