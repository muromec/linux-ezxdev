/*
 * camera.h
 *
 * Definition of Camera object. Common definitaion for MX21ADS CSI control
 *
 * Author:  stevel@mvista.com or source@mvista.com
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __MX2ADS_CAMERA_H__
#define __MX2ADS_CAMERA_H__

#include <linux/videodev.h>

#define CAMERA_OV9640_DEV_ID            (0x60)
#define CAMERA_MISOC0343_DEV_ID         (0xB8)

#define CAMERA_OV9640_WORK_FREQ         (24000000)
#define CAMERA_MISOC0343_WORK_FREQ      (24000000)

/* image and pixel formats supported by the various OMAP cameras */
enum image_format { QQCIF, QQVGA, QCIF, QVGA, CIF, VGA, SXGA };
enum pixel_format { YUV, RGB565, RGB555 };
#define NUM_IMAGE_SIZES 7
#define NUM_PIXEL_FORMATS 3

struct image_size {
	unsigned long width;
	unsigned long height;
};

extern const struct image_size omap_image_size[];
extern const int omap_pixfmt_depth[];

struct camera_interface;

typedef struct camera {
	/* interface in which this camera is attached */
	struct camera_interface * camif;

	enum image_format imgfmt; /* current image/pixel format */
	enum pixel_format pixfmt;
	
	int (*detect)(void); /* detect this camera */

	int (*init)(void);   /* one-time setup */
	void (*cleanup)(void);

	int (*open)(void);
	int (*close)(void);
	
	/* called when image/pixel format changes */
	int (*set_format)(struct v4l2_pix_format* fmt);

	/* frame period is in .1 usec units, xclk in Hz */
	int (*set_frame_period)(int fp, int xclk, int test);
	int (*get_frame_period)(void);
	
	int (*query_control)(struct v4l2_queryctrl *qc);
	int (*get_control)(struct v4l2_control *vc);
	int (*set_control)(struct v4l2_control *vc);
	int (*query_menu)(struct v4l2_querymenu *qm);
} camera_t;

/* Implementations of camera_t */
extern struct camera camera_ov9640;
extern struct camera camera_misoc0343; 

#endif  /* __MX2ADS_CAMERA_H__ */
