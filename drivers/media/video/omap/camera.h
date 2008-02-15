/*
 * camera.h
 *
 * Definition of Camera object.
 *
 * Copyright (C) 2003 MontaVista Software, Inc.
 *
 * Author: MontaVista Software, Inc.
 *              stevel@mvista.com or source@mvista.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/videodev.h>

#define CAMERA_OV9640_DEV_ID  0x60
#define CAMERA_OV6630_DEV_ID  0xC0
#define CAMERA_SANYO_DEV_ID   0x78

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
	// interface in which this camera is attached
	struct camera_interface * camif;

	enum image_format imgfmt; // current image/pixel format
	enum pixel_format pixfmt;
	
	int (*detect)(void); // detect this camera

	int (*init)(void);   // one-time setup
	void (*cleanup)(void);

	int (*open)(void);
	int (*close)(void);
	
	// called when image/pixel format changes
	int (*set_format)(struct v4l2_pix_format* fmt);

	// frame period is in .1 usec units, xclk in Hz
	int (*set_frame_period)(int fp, int xclk, int test);
	int (*get_frame_period)(void);
	
	/*
	 * convert and decode raw camera image if needed, or simply
	 * copy src to dest if no decoding necessary.
	 */
	int (*convert_image)(u8* src, void* dest, int to_user,
			     int dest_stride,
			     struct v4l2_pix_format* fmt);

	int (*query_control)(struct v4l2_queryctrl *qc);
	int (*get_control)(struct v4l2_control *vc);
	int (*set_control)(struct v4l2_control *vc);
	int (*query_menu)(struct v4l2_querymenu *qm);
} camera_t;

// Implementations of camera_t
extern struct camera camera_ov9640;
extern struct camera camera_ov6x30;
extern struct camera camera_sanyo;

