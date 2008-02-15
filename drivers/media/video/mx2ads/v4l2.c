/*
 * drivers/media/video/mx2ads/v4l2.c
 *
 * Description:
 *   Video for Linux Two Interface for mx2ads Capture device.
 *
 *   This driver based on capture driver for OMAP camera
 * Conceptual Usage: (exact syntax may vary)
 *   1. insmod videodevX
 *   2. insmod camera [unit_video=0]
 *      driver registers major 81, minor 0, which
 *      on most systems equates to either /dev/video0
 *      or sometimes simply /dev/video
 *   3. Now you can run apps that use the v4l2 interface
 *      of the installed camera driver. (see apps/v4l2/<>.c)
 *
 *
 * Author: MontaVista Software, Inc.
 *              source@mvista.com
 *
 * Created 2002, Copyright (C) 2002 RidgeRun, Inc.  All rights reserved.
 * Created 2002, Copyright (C) 2002 Texas Instruments  All rights reserved.
 * 2003 - 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Original Author: Bill Dirks <bdirks@pacbell.net>
 *                  based on code by Alan Cox, <alan@cymru.net>
 */
#include <linux/config.h>	/* retrieve the CONFIG_* macros */
#include <linux/module.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/videodev.h>
#include <linux/fb.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/arch/pll.h>
#define MODULE_NAME "v4l2-mx2ads"
#include "common.h"
#include "camif.h"
#include "v4l2.h"

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

static struct tq_struct fbinfo_tsk_q_entry;
static void update_fbinfo_task (void *);

extern long sys_ioctl (unsigned int fd, unsigned int cmd, unsigned long arg);

#define DEFAULT_FRAME_BUFF "/dev/fb0"

#define NUM_CAPFMT (4)
#define MAX_BPP 2 /* max bytes per pixel */


/*
 * Supported pixel formats. All the underlying cameras must support
 * these pixel formats. If the camera doesn't support a pixel format
 * in hardware, it will program the camera for the closest supported
 * format and then use its convert_image() method.
 */
static struct v4l2_fmtdesc capfmt[CAMIF_CHANNELS_NUM][NUM_CAPFMT] = {
    {
	{0, {"RGB-16 (5-5-5)"},
	 V4L2_PIX_FMT_RGB555, 0, 16, {0, 0},
	},
	{1, {"RGB-16 (5-6-5)"},
	 V4L2_PIX_FMT_RGB565, 0, 16, {0, 0},
	},
	{2, {"YUV 4:2:2 (Y-U-Y-V)"},
	 V4L2_PIX_FMT_YUYV, V4L2_FMT_CS_601YUV, 16, {0, 0},
	},
	{3, {"YUV 4:2:2 (U-Y-V-Y)"},
	 V4L2_PIX_FMT_YUYV, V4L2_FMT_CS_601YUV, 16, {0, 0},
	},
    },
    {
	{0, {"YUV 4:2:0 (Planar)"},
	 V4L2_PIX_FMT_YUV420, 0, 12, {0, 0},
	},
	{1, {"YUV 4:2:2 (Planar)"},
	 V4L2_PIX_FMT_YUV422P, 0, 16, {0, 0},
	},
	{2, {"YUV 4:2:0 (Planar)"},
	 V4L2_PIX_FMT_YVU420, 0, 12, {0, 0},
	},
	{3, {"YUV 4:2:2 (Planar)"},
	 V4L2_PIX_FMT_YVU422P, 0, 16, {0, 0},
	},
    }
#if 0
	{4, {"RGB-32 (B-G-R-?)"},
	 V4L2_PIX_FMT_BGR32, 0, 32, {0, 0},
	},
	{5, {"Greyscale-8"},
	 V4L2_PIX_FMT_GREY, V4L2_FMT_CS_601YUV, 8, {0, 0},
	},
	{6, {"YUV 4:2:0 (planar)"},
	 V4L2_PIX_FMT_YUV420, V4L2_FMT_CS_601YUV, 12, {0, 0},
	},
#endif
};

/*
 * Array of image formats supported by the various cameras used on
 * OMAP. These must be ordered from smallest image size to largest.
 * The specific camera will support all or a subset of these.
 */
const struct image_size mx2ads_image_size[] = {
	{   88,  72 },	/* QQCIF */
	{  160, 120 },	/* QQVGA */
	{  176, 144 },	/* QCIF */
	{  320, 240 },	/* QVGA */
	{  352, 288 },	/* CIF */
	{  640, 480 },	/* VGA */
	{ 1280, 960 },	/* SXGA */
};

/*
 * Array of pixel formats supported by the various cameras used on
 * OMAP. The camera uses its convert_image() method to convert from
 * a native pixel format to one of the above capfmt[] formats.
 */
const int mx2ads_pixfmt_depth[] = {
	16, /* YUV */
	16, /* RGB565 */
	15  /* RGB555 */
};

/* Extreme video dimensions */
#define MIN_WIDTH  32
#define MIN_HEIGHT 24
#define MAX_WIDTH  (mx2ads_image_size[SXGA].width)
#define MAX_HEIGHT (mx2ads_image_size[SXGA].height)
#define MAX_IMAGE_SIZE (MAX_WIDTH * MAX_HEIGHT * MAX_BPP)

#define MAX_FRAME_AGE           (200)	/* ms */

/*
 *      The Capture device structure array. This is the only global
 *      variable in the module besides those used by the device probing
 *      and enumeration routines (command line overrides)
 */
static struct capture_device capture[CAMIF_CHANNELS_NUM];
static struct capture_pwm_t capture_pwm;


static const int unit_video = 0;

static int
get_framebuffer_info (struct capture_device *dev)
{
	int fbfd, retcode;

	dev->fbinfo_valid = 0;
	fbfd = sys_open (DEFAULT_FRAME_BUFF, O_RDWR, 0);
	if (fbfd < 0) {
		err ("Error: cannot open framebuffer device.\n");
		return fbfd;
	}
	/* Get fixed screen information */
	if ((retcode = sys_ioctl (fbfd, FBIOGET_FSCREENINFO,
				  (unsigned long) (&dev->fbfix)))) {
		err ("Error reading fb fixed information.\n");
		return retcode;
	}
	/* Get variable screen information */
	if ((retcode = sys_ioctl (fbfd, FBIOGET_VSCREENINFO,
				  (unsigned long) (&dev->fbvar)))) {
		err ("Error reading fb var information.\n");
		return retcode;
	}

	sys_close (fbfd);
	dev->fbinfo_valid = 1;

	return 0;
}


static inline struct capture_device *
capture_device_from_file (struct file *file)
{
	return (struct capture_device *) v4l2_device_from_file (file);
}


static int
isqrt (unsigned int q)
{				/*     A little integer square root routine */
	int i;
	int r;
	unsigned int b2 = 0x40000000;
	unsigned int t;

	for (i = 16, r = 0; i > 0 && q; --i) {
		t = ((unsigned int) r << i) + b2;
		if (t <= q) {
			q -= t;
			r |= (1 << (i - 1));
		}
		b2 >>= 2;
	}
	return r;
}

static unsigned long
current_time_ms (void)
{
	struct timeval now;

	do_gettimeofday (&now);
	return now.tv_sec * 1000 + now.tv_usec / 1000;
}


/*
 *
 *      V I D E O   D E C O D E R S
 *
 */

static int
decoder_initialize (struct capture_device *dev)
{
	/*  Video decoder information fields    */
	dev->videc.standards = (1 << V4L2_STD_NTSC) | (1 << V4L2_STD_PAL);
	dev->videc.ntsc_hskip = 30;
	dev->videc.ntsc_vskip = 12;
	dev->videc.ntsc_width = 640;
	dev->videc.ntsc_height = 480;
	dev->videc.ntsc_field_order = 0;
	dev->videc.pal_hskip = 62;
	dev->videc.pal_vskip = 14;
	dev->videc.pal_width = 640;
	dev->videc.pal_height = 480;
	dev->videc.pal_field_order = 0;
	dev->videc.preferred_field = 0;

	dev->videc.num_inputs = 2;

	dev->videc.decoder_is_stable = 1;

	return 1;
}

static int
decoder_set_input (struct capture_device *dev, int i)
{
	ENTRY ();
	dev->input = i;
	/* dev->videc.decoder_is_stable = 0; */
	/*  TODO: Switch the hardware to the new input  */
	return 1;
}

static int
decoder_set_frame_period (struct capture_device *dev, int fp)
{
	int retcode = -EINVAL;
	
	if (dev->camif->set_frame_period)
		retcode = dev->camif->set_frame_period (fp);

	if (retcode >= 0) {
		dev->videc.frame_period = retcode;
		return 1;
	}

	dev->videc.frame_period = 333667;
	return retcode;
}

static int
decoder_set_standard (struct capture_device *dev, int x)
{
	dev->videc.standard = x;
	int fp;
	
	switch (x) {
	case V4L2_STD_NTSC:
		fp = 333667;
		break;
	case V4L2_STD_PAL:
	case V4L2_STD_SECAM:
		fp = 400000;
		break;
	default:
		fp = 333667;
		break;
	}

	return decoder_set_frame_period (dev, fp);
}

static int
decoder_set_vcrmode (struct capture_device *dev, int x)
{
	dev->source[dev->input].vcrmode = x;
	/*  TODO: Switch decoder to VCR sync timing mode  */
	return 1;
}

static int
decoder_is_stable (struct capture_device *dev)
{
	/*  TODO: Check if decoder is synced to input  */
	return 1;
}

static int
decoder_probe (struct capture_device *dev)
{
	/*  TODO: Probe I2C bus or whatever for the video decoder */

	/*  Fill in the method fields  */
	dev->videc.initialize = decoder_initialize;
	dev->videc.set_input = decoder_set_input;
	dev->videc.set_standard = decoder_set_standard;
	dev->videc.set_vcrmode = decoder_set_vcrmode;
	dev->videc.is_stable = decoder_is_stable;
	dev->videc.set_frame_period = decoder_set_frame_period;

	dbg("Found decoder chip\n");
	return 1;		/*  Found  */
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *
 *      Probe I2C bus for video decoder and fill in the device fields
 */
static int
find_decoder (struct capture_device *dev)
{
	if (!decoder_probe (dev))
		return 0;	/*  Failure  */
	return 1;
}

static void
set_video_input (struct capture_device *dev, int i)
{
	if (i < 0 || i >= dev->videc.num_inputs)
		return;
	dev->videc.set_input (dev, i);
	dev->videc.set_vcrmode (dev, dev->source[i].vcrmode);
}



/*
 *
 *      V I D E O   C A P T U R E   F U N C T I O N S
 *
 */


/*  Stop the music!
 */
static void
capture_abort (struct capture_device *dev)
{
	ENTRY();

	/*  Turn off the capture hardware  */
	dev->camif->abort (dev->id);
	dev->capture_started = 0;

	/*
	 * Wake up any processes that might be waiting for a frame
	 * and let them return an error
	 */
	wake_up_interruptible (&dev->new_video_frame);
	EXIT();
}


/*  The image format has changed, width, height, pixel format.
 *  Decide if the format is ok or take the closest valid format.
 */
static int
capture_new_format (struct capture_device *dev)
{
	int max_image_size;
	int max_height;
	int max_width;
	int max_pixels;
	int t, retcode = 0;

	ENTRY();

	dev->ready_to_capture = 0;

	max_width = MAX_WIDTH;
	max_height = MAX_HEIGHT;

	dev->clientfmt.flags = V4L2_FMT_CS_601YUV;
	/* desired default. */
	dev->clientfmt.flags |= V4L2_FMT_FLAG_ODDFIELD;

	switch (dev->clientfmt.pixelformat) {
	case V4L2_PIX_FMT_GREY:
		dev->clientfmt.depth = 8;
		break;
	case V4L2_PIX_FMT_YUV420:
		dev->clientfmt.depth = 12;
		break;
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB565:
		dev->clientfmt.flags = 0;
		/* fall thru */
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		dev->clientfmt.depth = 16;
		break;
	case V4L2_PIX_FMT_BGR24:
		dev->clientfmt.depth = 24;
		dev->clientfmt.flags = 0;
		break;
	case V4L2_PIX_FMT_BGR32:
		dev->clientfmt.depth = 32;
		dev->clientfmt.flags = 0;
		break;
	default:
		dbg("unknown format %4.4s\n",
		        (char *)&dev->clientfmt.pixelformat);
		dev->clientfmt.depth = 16;
		dev->clientfmt.pixelformat = V4L2_PIX_FMT_YUYV;
		dev->clientfmt.flags = 0;
		break;
	}

	dev->capture_bypp = (dev->clientfmt.depth + 7) >> 3;

	if (dev->clientfmt.width < MIN_WIDTH)
		dev->clientfmt.width = MIN_WIDTH;
	if (dev->clientfmt.height < MIN_HEIGHT)
		dev->clientfmt.height = MIN_HEIGHT;

	max_image_size = MAX_IMAGE_SIZE;
	if (dev->stream_buffers_mapped) {
		/* Limited by size of existing buffers  */
		max_image_size = dev->stream_buf[0].vidbuf.length;
	}
	max_pixels = max_image_size / dev->capture_bypp;
	t = isqrt ((max_pixels * dev->clientfmt.width) /
		   dev->clientfmt.height);
	if (t < max_width)
		max_width = t;
	t = isqrt ((max_pixels * dev->clientfmt.height) /
		   dev->clientfmt.width);
	if (t < max_height)
		max_height = t;

	if (dev->clientfmt.width > max_width)
		dev->clientfmt.width = max_width;
	if (dev->clientfmt.height > max_height)
		dev->clientfmt.height = max_height;

	dev->clientfmt.width &= ~3;
	dev->clientfmt.height &= ~3;

	/* tell the camera about the format, it may modify width
	   and height. */
	if (dev->camera) {
		if ((retcode = dev->camif->set_format (dev->id,
                                                &dev->clientfmt))) {
			EXIT();
			return retcode;
		}
	}
	dev->clientfmt.sizeimage =
		(dev->clientfmt.width * dev->clientfmt.height *
		 dev->clientfmt.depth) / 8;
	dev->capture_size =
		dev->clientfmt.width * dev->clientfmt.height *
		dev->capture_bypp;
	EXIT();
	return 0;
}


/****************************
 * Routine:
 * Description:
 ****************************/
static void
DeallocateBuffer (struct capture_device *dev)
{
	if (dev->capture_buffer != NULL) {
		if (dev->camera && dev->input == 0) {
			dev->camif->free_image_buffer(dev->capture_buffer,
						      dev->capture_buffer_size);
		} else {
			vfree (dev->capture_buffer);
		}

		dev->capture_buffer = NULL;
	}
}


/****************************
 * Routine:
 * Description:
 ****************************/
static void
AllocateBuffer (struct capture_device *dev)
{
	DeallocateBuffer (dev);
	/* dev->input=0 is camera image (if real h/w present).*/
	/* dev->input=1 is color bar test pattern.*/
	if (dev->camera && dev->input == 0) {
		dev->capture_buffer_size = dev->capture_size;
		dev->capture_buffer =
			dev->camif->alloc_image_buffer(dev->capture_buffer_size);
	} else {
		dev->capture_buffer_size =
			(dev->capture_size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);
		dev->capture_buffer =
			(__u8 *) vmalloc (dev->capture_buffer_size);
	}
}


/*  Allocate buffers, and get everything ready to capture
 *  an image, but don't start capturing yet.
 */
static int
capture_begin (struct capture_device *dev)
{
	ENTRY();
	capture_abort (dev);
	if (dev->ready_to_capture) {
		EXIT();
		return dev->ready_to_capture;
	}
	
	if ((dev->capture_buffer_size < dev->capture_size) ||
	    dev->SwitchInputs) {
		dev->SwitchInputs = 0;
		AllocateBuffer (dev);
		if (dev->capture_buffer == NULL) {
			dev->capture_buffer_size = 0;
			err ("Can't allocate capture buffer"
			     " %d bytes\n", dev->capture_size);
			EXIT();
			return dev->ready_to_capture;
		}
	}
	EXIT();
	return (dev->ready_to_capture = 1);
}

/*  Start an image capture
 */
static void
capture_grab_frame (struct capture_device *dev)
{
	if (dev->ready_to_capture && dev->capture_started)
		return;

	capture_begin (dev);
	if (!dev->ready_to_capture)
		return;

	if (dev->camera && dev->input == 0) {
		/* Start the camera h/w. It will call us back
		   on image completion.*/
                if (dev->camif->snapshot (dev->id,
                    dev->capture_buffer, dev->capture_size) == 0) {
			dev->capture_started = 1;
			dev->capture_completed = 0;
		}
	} else {
		dev->capture_started = 1;
		dev->capture_completed = 0;
	}
}

/*  Start an image stream
 */
static void
capture_stream_start (struct capture_device *dev)
{
	capture_begin (dev);
	if (!dev->ready_to_capture)
		return;

	/*  Set up stream_capture_buffer to point to the buffer to  */
	/*  capture the next frame into  */
	dev->stream_capture_buffer = dev->capture_buffer;

	if (dev->camera && dev->input == 0) {
		/* Start the camera h/w. It will call us back
		   on image completion.*/
		if (dev->camif->start_streaming (dev->id,
                                        dev->capture_buffer,
						 dev->capture_size) == 0) {
			dev->capture_started = 1;
			dev->capture_completed = 0;
		}
	} else {
		dev->capture_started = 1;
		dev->capture_completed = 0;
	}
}

/*
 *      STREAMING CAPTURE
 */

/* 1 = success; 0 = failed */
static int
capture_queuebuffer (struct capture_device *dev, struct v4l2_buffer *vidbuf)
{
	int i = vidbuf->index;
	struct stream_buffer *buf = NULL;

	if (!dev->stream_buffers_mapped) {
		dbg("QBUF no buffers mapped\n");
		return 0;
	}
	if (vidbuf->type != V4L2_BUF_TYPE_CAPTURE) {
		dbg("QBUF wrong type\n");
		return 0;
	}
	if (i < 0 || i >= MAX_CAPTURE_BUFFERS
	    || !dev->stream_buf[i].requested) {
		dbg("QBUF buffer index %d is out of range\n", i);
		return 0;
	}

	buf = &dev->stream_buf[i];

	if (!(buf->vidbuf.flags & V4L2_BUF_FLAG_MAPPED)) {
		dbg("QBUF buffer %d is not mapped\n", i);
		return 0;
	}
	if ((buf->vidbuf.flags & V4L2_BUF_FLAG_QUEUED)) {
		dbg("QBUF buffer %d is already queued\n", i);
		return 0;
	}

	buf->vidbuf.flags &= ~V4L2_BUF_FLAG_DONE;
	v4l2_q_add_tail (&dev->stream_q_capture, &buf->qnode);
	buf->vidbuf.flags |= V4L2_BUF_FLAG_QUEUED;
	return 1;
}

static int			/* 1 = got a buffer; 0 = no buffers */
capture_dequeuebuffer (struct capture_device *dev, struct v4l2_buffer *buf)
{
	struct stream_buffer *newbuf;

	if (buf->type != V4L2_BUF_TYPE_CAPTURE) {
		dbg("DQBUF wrong buffer type\n");
		return 0;
	}
	newbuf = v4l2_q_del_head (&dev->stream_q_done);
	if (newbuf == NULL) {
		dbg("DQBUF nothing on done queue\n");
		return 0;
	}
	newbuf->vidbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;

	*buf = newbuf->vidbuf;
	return 1;
}

static int
capture_streamon (struct capture_device *dev, __u32 type)
{
	struct stream_buffer *buf;

	ENTRY ();
	if (type != V4L2_BUF_TYPE_CAPTURE) {
		dbg("STREAMON wrong buffer type\n");
		EXIT();
		return 0;
	}
	if (dev->streaming) {
		EXIT();
		return 1;
	}

	capture_abort (dev); /* cancel any capture that might be in progress */

	/*  -2 is a magic number that triggers start-of-stream logic in */
	/*    capture_interrupt()  */
	dev->stream_last_frame = -2;

	dev->perf.frames = 0;
	dev->perf.framesdropped = 0;
	dev->perf.bytesout = 0;
        

	/*  Can't capture frames faster than the video input  */
	if (dev->capture.timeperframe < dev->videc.frame_period)
		dev->capture.timeperframe = dev->videc.frame_period;

	/*  Move any leftover DONE buffers to the free pool */
	while ((buf = v4l2_q_del_head (&dev->stream_q_done)))
		buf->vidbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;

	/*  Kick off the machine in continuous capture mode */
	dev->streaming = 1;
	capture_stream_start (dev);
	EXIT();
	return 1;
}

static void
capture_streamoff (struct capture_device *dev, __u32 type)
{
	ENTRY();
	if (!dev->streaming) {
		EXIT();
		return;
	}
	if (type != V4L2_BUF_TYPE_CAPTURE) {
		err("STREAMOFF wrong buffer type\n");
		EXIT();
		return;
	}
	capture_abort (dev);
	dev->streaming = 0;

	/* Note: should really delay this till next capture */
	dev->perf.frames = 0;
	dev->perf.framesdropped = 0;
	dev->perf.bytesout = 0;
	EXIT();
}


/*
 * Convert raw camera image directly into framebuffer (used by
 * preview mode). Returns length of data or negative for error.
 */
static int
capture_display_image (struct capture_device *dev, __u8 * capture_buffer)
{
	if (!dev->capture_started) {
		/* If we get here is probably because the PREVIEW
		   mode was turned off just prior to the interrupt
		   routines completing the last image. We don't
		   want anymore images. Just return.*/
		dbg ("capture not started??\n");
		return 0;
	}

	if (dev->camera) {
		int dest_stride;

		if (!dev->fbinfo_valid) {
			err ("preview set but no valid fb info\n");
			return -EINVAL;
		}

		dest_stride = (dev->fbvar.xres_virtual *
			       dev->fbvar.bits_per_pixel) / 8;

		dev->camif->convert_image (dev->id,
                                        (u8 *) capture_buffer,
					(u8 *) phys_to_virt (dev->fbfix.
								 smem_start),
					    0, dest_stride, &dev->clientfmt);
	}

	++dev->perf.frames;
	dev->perf.bytesout += dev->clientfmt.sizeimage;

	return dev->clientfmt.sizeimage;
}


/*
 * Convert the next frame.
 * Returns length of data or negative for error.
 */
static int
capture_convert_image (struct capture_device *dev,
		       __u8 * capture_buffer,
		       __u8 * output_buffer, int output_is_user)
{
	if (!dev->capture_started) {
		/* If we get here is probably because the PREVIEW
		   mode was turned off just prior to the interrupt
		   routines completing the last image. We don't
		   want anymore images. Just return.*/
		dbg ("capture not started??\n");
		return 0;
	}

	if (dev->camera) {
		int dest_stride;

		dest_stride = (dev->clientfmt.width *
			       dev->clientfmt.depth) / 8;

		dev->camif->convert_image (dev->id,
                                            (u8 *) capture_buffer,
					    (void *) output_buffer,
					    output_is_user,
					    dest_stride, &dev->clientfmt);
	}

	++dev->perf.frames;
	dev->perf.bytesout += dev->clientfmt.sizeimage;

	return dev->clientfmt.sizeimage;
}

/*  The hardware has issued the interrupt signal, do any post-capture
 *  processing that may be necessary.
 *  [This function is called indirectly through the immediate task queue;
 *  it executes at elevated IRQL, but it is interruptible. (It's a b.h.)]
 */
static void
capture_interrupt (void *v)
{
	struct capture_device *dev = (struct capture_device *) v;
	struct stream_buffer *buf;
	int len;
	stamp_t timestamp_rough;
	unsigned long raw_frame_num;
	unsigned long next_raw_frame_to_keep;
	unsigned long stream_frame_num;
	stamp_t temp64;

	if (!dev->capture_started || dev->capture_completed)
		return;
	if (!dev->streaming)
		dev->capture_completed = 1;

	if (dev->capture_buffer == NULL) {
		err ("Can't process the interrupt\n");
		return;
	}

	if (!dev->streaming) {
		dev->time_acquired = current_time_ms ();
		if (dev->preview) {
			capture_display_image (dev, dev->capture_buffer);
			dev->capture_started = 0;
			capture_grab_frame (dev); /* Start another capture.*/
		}

		wake_up_interruptible (&dev->new_video_frame);
		return;
	}

	/*  Only get here in streaming mode  */

	if (dev->stream_last_frame == -2) {
		/* First frame of the stream  */
		v4l2_masterclock_gettime (&dev->stream_begin);
		dev->stream_last_frame = -1;
	}

	if (dev->stream_capture_buffer == NULL) {
		dbg ("stream_capture_buffer is NULL!\n");
		return;
	}

	buf = v4l2_q_peek_head (&dev->stream_q_capture);
	if (buf == NULL) {
		/*
		 * No available buffers. Skip this frame. This is not an
		 * error, it's a normal way to throttle the capture rate
		 */
		return;
	}

	/*  Compute current stream time  */
	v4l2_masterclock_gettime (&timestamp_rough);
	/*  We do computations on a relative timestamp (since beginning  */
	/*  of stream), then add 'dev->stream_begin' back in later       */
	timestamp_rough -= dev->stream_begin;

	/*  Capture rate control  */
	if (dev->videc.frame_period == 0) {
		/* Sanity check to avoid divide-by-zero  */
		err ("videc.frame_period == 0 -- divide by 0\n");
		dev->videc.frame_period = 333667;
	        dev->capture.timeperframe= 333667; 
	}
	raw_frame_num =
		v4l2_timestamp_divide (timestamp_rough,
				       dev->videc.frame_period);
	temp64 = (stamp_t) dev->capture.timeperframe *
		(dev->stream_last_frame + 1)
		+ (dev->videc.frame_period >> 1);
	next_raw_frame_to_keep =
		v4l2_math_div6432 ((u64) temp64, dev->videc.frame_period,
				   NULL);

	if (raw_frame_num < next_raw_frame_to_keep) {
		/* Not time yet, don't keep this frame */
		return;
	}

        /*  Want this frame  */
	len = capture_convert_image (dev, dev->stream_capture_buffer,
				     buf->vaddress, 0);
	if (len <= 0) {
		/* Frame no good, skip it. */
		dbg ("frame no good\n");
		return;
	}

	/*  Fill in the buffer information fields  */
	buf->vidbuf.bytesused = len;
	buf->vidbuf.flags |= V4L2_BUF_FLAG_DONE | V4L2_BUF_FLAG_KEYFRAME;

	stream_frame_num = v4l2_timestamp_correct (&timestamp_rough,
						   dev->capture.timeperframe);
	
	/* Returned timestamp should be absolute system time  */
	buf->vidbuf.timestamp = timestamp_rough + dev->stream_begin;
	buf->vidbuf.sequence = stream_frame_num;

	if (stream_frame_num > dev->stream_last_frame + 1) {
		/* We have missed one or more frames  */
		dev->perf.framesdropped +=
			(stream_frame_num - (dev->stream_last_frame + 1));
	}
	dev->stream_last_frame = stream_frame_num;

	/*  Move buffer to done queue  */
	buf = v4l2_q_del_head (&dev->stream_q_capture);
	v4l2_q_add_tail (&dev->stream_q_done, &buf->qnode);

	/*  A new frame is ready!  */
	wake_up_interruptible (&dev->new_video_frame);
}

/*
 * This is the callback from the Camera Interface, when
 * a new image has been captured by the camif h/w. This
 * is called at interrupt time, so just mark a bh and do
 * nothing more.
 */
static void
camif_capture_callback (void *data)
{
	struct capture_device *dev = (struct capture_device *) data;

	dev->tqnode_dpc.routine = capture_interrupt;
	dev->tqnode_dpc.data = dev;
	queue_task (&dev->tqnode_dpc, &tq_immediate);
	mark_bh (IMMEDIATE_BH);
}


/*  Read captured data into a user buffer.
 *  Return: negative = error
 *          0        = keep waiting
 *          positive = count of bytes read successfully
 */
static long
capture_read (struct capture_device *dev,
	      __u8 * user_buffer, int user_buffer_size)
{
	int len = user_buffer_size;
	unsigned long now;

	if (!dev->capture_completed) {
		/* No interrupt has occurred yet, or DMA didn't finish.  */
		dbg("No data ready.\n");
		if (!dev->capture_started) {
			capture_grab_frame (dev);
		}
		return 0;	/* caller should keep waiting */
	}

	now = current_time_ms ();
	if (now - dev->time_acquired > MAX_FRAME_AGE) {
		/* Frame in buffer is stale, get a new one */
		dbg("Stale frame, re-acquiring.\n");
		dev->capture_started = 0;
		capture_grab_frame (dev);
		return 0;	/* caller should keep waiting */
	}

	len = capture_convert_image (dev, dev->capture_buffer,
				     user_buffer, 1);
	dev->capture_started = 0;
	capture_grab_frame (dev);
	return len;
}

/*  Stop capturing and free all resources used for capture.
 */
static void
capture_close (struct capture_device *dev)
{
	ENTRY();
	if (dev->streaming)
		capture_streamoff (dev, V4L2_BUF_TYPE_CAPTURE);
	capture_abort (dev);
	dev->ready_to_capture = 0;
	if (dev->capture_dma_list)
		free_page ((unsigned long) dev->capture_dma_list);
	dev->capture_dma_list = 0;
	DeallocateBuffer (dev);
	dev->capture_buffer = NULL;
	dev->capture_buffer_size = 0;
	EXIT();
}



/*
 *
 *      P O W E R   M A N A G E M E N T
 *
 */
#if 1   /* power management */
static void
mx2ads_camera_suspend(void)
{
	int i;
	capture_pwm.suspended = 1;
	for (i = 0; i < CAMIF_CHANNELS_NUM; i++) {
		if (capture[i].open_count || capture[i].preview) {
			capture_close (&capture[i]);
			capture[i].camif->close (i);
		}
	}
}

static void
mx2ads_camera_resume(void)
{
	/*
	* Resume is getting called in an interrupt context and
	* resume requires waiting on queues etc. to power
	* up the camera. So we can't resume here. So we have to
	* use a kernel thread for resume requests (PITA).
	*/
	up(&capture_pwm.resume_wait);
}

#ifdef CONFIG_PM
static struct pm_dev *mx2ads_image_pmdev;

static int
mx2ads_image_pm_callback(struct pm_dev *pmdev, pm_request_t rqst, void *data)
{
	switch (rqst) {
	case PM_SUSPEND:
		mx2ads_camera_suspend();
		break;
	case PM_RESUME:
		mx2ads_camera_resume();
		break;
	}
	return 0;
}
#endif /* CONFIG_PM*/

#include <linux/device.h>
#include <linux/dpm.h>

#ifdef CONFIG_DPM
static struct constraints mx2cam_constraints = {
	.count = 1,
	.param = {{DPM_MD_HCLK, 0, 0}},	/*to be initialized at module init time */
	.asserted = 1,
};
#endif


extern void mx21_ldm_bus_register(struct device *device,
                          struct device_driver *driver);
extern void mx21_ldm_bus_unregister(struct device *device,
                          struct device_driver *driver);

static int
mx2ads_camera_dpm_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		mx2ads_camera_suspend();
		break;
	}
	return 0;
}

static int
mx2ads_camera_dpm_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		mx2ads_camera_resume();
		break;
	}
	return 0;
}

static struct device_driver mx2ads_camera_driver_ldm = {
	.name       = "mx2ads-cam",
	.suspend    = mx2ads_camera_dpm_suspend,
	.resume     = mx2ads_camera_dpm_resume,
};

static struct device mx2ads_camera_device_ldm = {
	.name           = "MX2ADS Camera",
	.bus_id         = "camera",
	.driver         = &mx2ads_camera_driver_ldm,
	.power_state    = DPM_POWER_ON,
#ifdef CONFIG_DPM
	.constraints = &mx2cam_constraints,
#endif
};

static int
resume_thread(void *data)
{
        int i;
	daemonize();
	reparent_to_init();
	strcpy(current->comm, "camera_resume");
	
	for (;;) {

		down(&capture_pwm.resume_wait);

		if (capture_pwm.resume_thread_exit)
			break;

		for (i = 0; i < CAMIF_CHANNELS_NUM; i++) {
			if (capture[i].open_count || capture[i].preview) {
				capture[i].camif->open(i);
				capture_abort (&capture[i]);
				capture_new_format (&capture[i]);
				if (capture[i].streaming)
					capture_stream_start (&capture[i]);
				else if (capture[i].preview)
					capture_grab_frame (&capture[i]);
			}
		}
		capture_pwm.suspended = 0;
	}
	
	complete_and_exit(&capture_pwm.resume_thread_sync, 0);
	return 0;
}

#endif   /* power management */


/*
 *
 *      M E M O R Y   M A P P I N G
 *
 */

static struct stream_buffer *
mmap_stream_buffer_from_offset (struct capture_device *dev,
				unsigned long offset)
{
	int i;

	offset = offset * PAGE_SIZE;

	for (i = 0; i < MAX_CAPTURE_BUFFERS; ++i)
		if (offset == dev->stream_buf[i].vidbuf.offset)
			return &dev->stream_buf[i];
	return NULL;
}

static int
mmap_request_buffers (struct capture_device *dev,
		      struct v4l2_requestbuffers *req)
{
	int i;
	u32 buflen;
	u32 type;

	if (dev->stream_buffers_mapped)
		return 0;	/* can't make requests if buffers are mapped */
	if (req->count < 1)
		req->count = 1;
	if (req->count > MAX_CAPTURE_BUFFERS)
		req->count = MAX_CAPTURE_BUFFERS;
	type = V4L2_BUF_TYPE_CAPTURE;
	dev->stream_contig_map = 0;
	if (req->type & V4L2_BUF_REQ_CONTIG) {
		dev->stream_contig_map = 1;
		req->type = type | V4L2_BUF_REQ_CONTIG;
		/* note: _REQ_CONTIG is only used in v4l2_requestbuffers */
	}

	/*  The buffer length needs to be a multiple of the page size  */
	buflen = (dev->clientfmt.sizeimage + PAGE_SIZE - 1)
		& ~(PAGE_SIZE - 1);

	/*  Now initialize the buffer structures. Don't allocate the */
	/*  buffers until they're mapped. */
	for (i = 0; i < req->count; ++i) {
		dev->stream_buf[i].requested = 1;
		dev->stream_buf[i].vidbuf.index = i;
		dev->stream_buf[i].vidbuf.type = type;
		/* offset must be unique for each buffer, and a multiple */
		/* of PAGE_SIZE on 2.4.x  */
		dev->stream_buf[i].vidbuf.offset = PAGE_SIZE * (i + 1);
		dev->stream_buf[i].vidbuf.length = buflen;
		dev->stream_buf[i].vidbuf.bytesused = 0;
		dev->stream_buf[i].vidbuf.flags = 0;
		dev->stream_buf[i].vidbuf.timestamp = 0;
		dev->stream_buf[i].vidbuf.sequence = 0;
		memset (&dev->stream_buf[i].vidbuf.timecode, 0,
			sizeof (struct v4l2_timecode));
	}
	for (i = req->count; i < MAX_CAPTURE_BUFFERS; ++i)
		dev->stream_buf[i].requested = 0;
	dev->stream_buffers_requested = req->count;

	return 1;
}

static void
mmap_unrequest_buffers (struct capture_device *dev)
{
	int i;

	if (dev->stream_buffers_requested == 0 || dev->stream_buffers_mapped)
		return;
	for (i = 0; i < MAX_CAPTURE_BUFFERS; ++i)
		dev->stream_buf[i].requested = 0;
	dev->stream_buffers_requested = 0;
}

static void
mmap_vma_open (struct vm_area_struct *vma)
{
	struct capture_device *dev = capture_device_from_file (vma->vm_file);
	struct stream_buffer *buf;

#if 1   /* power management */
	if (capture_pwm.suspended) return;
#endif /* power management */

	if (dev == NULL)
		return;
	buf = mmap_stream_buffer_from_offset (dev, vma->vm_pgoff);
	if (dev->stream_contig_map)
		buf = &dev->stream_buf[0];
	++buf->vma_refcount;
	dbg("vma_open called\n");
}

static void
mmap_vma_close (struct vm_area_struct *vma)
{
	struct capture_device *dev = capture_device_from_file (vma->vm_file);
	struct stream_buffer *buf =
		mmap_stream_buffer_from_offset (dev, vma->vm_pgoff);
	int i, n = 1;

#if 1   /* power management */
	if (capture_pwm.suspended) return;
#endif /* power management */

	if (dev->stream_contig_map) {	/*     Unmap all the buffers in one stroke  */
		n = dev->stream_buffers_mapped;
		buf = &dev->stream_buf[0];
	}
	--buf->vma_refcount;
	if (buf->vma_refcount > 0)
		return;
	for (i = 0; i < n; ++i) {
		if (dev->streaming) {
			dbg("Warning- munmap() called while streaming\n");
			capture_streamoff (dev, buf->vidbuf.type);
		}
		v4l2_q_yank_node (&dev->stream_q_capture, &buf->qnode);
		v4l2_q_yank_node (&dev->stream_q_done, &buf->qnode);

		if (buf->vaddress != NULL && i == 0)
			vfree (buf->vaddress);
		buf->vaddress = NULL;
		if (buf->dma_list)
			free_page ((unsigned long) buf->dma_list);
		buf->dma_list = NULL;
		buf->vidbuf.flags = 0;
		dbg("Buffer %d deallocated\n",(int)vma->vm_pgoff);
		++buf;
		if (dev->stream_buffers_mapped > 0)
			--dev->stream_buffers_mapped;
	}
}

static struct page *
mmap_vma_nopage (struct vm_area_struct *vma, unsigned long address, int write)
{
	struct capture_device *dev;
	struct stream_buffer *buf;
	unsigned long offset_into_buffer;
	struct page *page;
	int n = 1;

	dev = capture_device_from_file (vma->vm_file);
	if (dev == NULL)
		return 0;

#if 1   /* power management */
	if (capture_pwm.suspended) return 0;
#endif /* power management */

	if (dev->stream_contig_map) {
		buf = &dev->stream_buf[0];
		n = dev->stream_buffers_requested;
	} else
		buf = mmap_stream_buffer_from_offset (dev, vma->vm_pgoff);

	if (buf == NULL)
		return 0;
	offset_into_buffer = address - vma->vm_start;
	if (offset_into_buffer >= buf->vidbuf.length * n) {
		err ("Attempt to read past end of mmap() buffer\n");
		return 0;
	}
	page = v4l2_vmalloc_to_page (buf->vaddress + offset_into_buffer);
	if (page == 0)
		return 0;

	atomic_inc (&page->count);

	return page;
}

static struct vm_operations_struct capture_vma_operations = {
	mmap_vma_open, mmap_vma_close, mmap_vma_nopage,
};

/*
 *
 *      V 4 L 2   I N T E R F A C E
 *
 */

static int
v4l2_open (struct v4l2_device *v, int flags, void **idptr)
{
	struct capture_device *dev = (struct capture_device *) v;
	int i, n;
	int cap;
	int retcode;
        
	dbg("Trying to open\n");

#if 1   /* power management */
	if (capture_pwm.suspended) return -EPERM;
#endif /* power management */

	for (i = 0, n = -1, cap = 0; i < MAX_OPENS; ++i) {
		if (!dev->open_data[i].isopen)
			n = i;	/* available open_data structure */
		else if (!dev->open_data[i].noncapturing)
			cap = 1;	/* another open is already capturing */
	}
	if (n == -1) {		/* No available open_data structures */
		dbg("No more opens on this device\n");
		return -EBUSY;
	}

	if (flags & O_NONCAP)	/*  Non-capturing open */
		dev->open_data[n].noncapturing = 1;
	else if (cap) {
		dbg("No more capturing opens on this device\n");
		return -EBUSY;
	} else {
		dev->open_data[n].noncapturing = 0;
		/*  Keep track of whether there is a capturing open  */
		++dev->capturing_opens;
		dev->perf.frames = 0;
		dev->perf.framesdropped = 0;
		dev->perf.bytesout = 0;
	}

	MOD_INC_USE_COUNT;
	++dev->open_count;
	dev->open_data[n].isopen = 1;
	dev->open_data[n].dev = dev;
	*idptr = &dev->open_data[n];

	if (dev->open_count == 1) {
		if (dev->preview) {
			/* If preview mode is on then we will refrain
			   from any furthur device initialization since
			   it isn't needed and actually will disrupt the
			   automatic preview operation that the driver
			   is in the process of doing. Recall that apps
			   can open() the video device, turn on preview,
			   and then close() it; preview continues. Sometime
			   later they may re-open() the device and we don't
			   want that disrupting the on-going preview
			   processing.*/
			return 0;
		}

		if ((retcode = dev->camif->open (dev->id))) {
			dev->open_count = 0;
			dev->open_data[n].isopen = 0;
			return retcode;
		}
	
		dev->ready_to_capture = 0;	/* benchmark changes parameters! */
		dev->capture_completed = 0;
		dev->capture_started = 0;
		v4l2_q_init (&dev->stream_q_capture);
		v4l2_q_init (&dev->stream_q_done);
	}
	dbg("Open succeeded\n");

	/* frame counter for test images only */
	if (!dev->open_data[n].noncapturing)
		dev->h = dev->m = dev->s = dev->f = 0;
	return 0;
}

static void
v4l2_close (void *id)
{
	struct device_open *o = (struct device_open *) id;
	struct capture_device *dev = o->dev;

	if (!o->noncapturing) {
		--dev->capturing_opens;
		dbg("Close\n");
	}
	o->isopen = 0;
	--dev->open_count;
	MOD_DEC_USE_COUNT;

	if (dev->preview) {
		/* If we have been placed into preview mode then
		   we want to keep that going even if the app
		   closes the video device. We expect sometime
		   later the app, or some app, will open the
		   video device and turn off preview when it
		   desires. This video driver handles the
		   preview operation and all the app needs to
		   do is briefly open() us and turn on or
		   off the preview mode and then, if desired,
		   close() us. */
		return;
	}
	/* shut things down.*/      /* power management considered */
	if ((dev->open_count == 0) && (!capture_pwm.suspended)) {
		capture_close (dev);
		dev->camif->close (dev->id);
	}
}

static long
v4l2_write (void *id, const char *buf, unsigned long count, int noblock)
{
	dbg("Write() not handled\n");
	return -EINVAL;
}

/*  The arguments are already copied into kernel memory, so don't use
    copy_from_user() or copy_to_user() on arg.  */
static int
v4l2_ioctl (void *id, unsigned int cmd, void *arg)
{
	struct device_open *o = (struct device_open *) id;
	struct capture_device *dev = o->dev;

#if 1   /* power management */
	if (capture_pwm.suspended) return -EPERM;
#endif /* power management */

	switch (cmd) {
	case VIDIOC_QUERYCAP:
	{
		struct v4l2_capability *b = arg;
		strcpy (b->name, dev->v.name);
		b->type = V4L2_TYPE_CAPTURE;
		b->flags = V4L2_FLAG_READ |
			V4L2_FLAG_STREAMING |
			V4L2_FLAG_PREVIEW | V4L2_FLAG_SELECT;
		b->inputs = dev->videc.num_inputs;
		b->outputs = 0;
		b->audios = 0;
		b->maxwidth = MAX_WIDTH;
		b->maxheight = MAX_HEIGHT;
		b->minwidth = MIN_WIDTH;
		b->minheight = MIN_HEIGHT;
		b->maxframerate = 30;
		return 0;
	}

	case VIDIOC_ENUM_PIXFMT:
	{
		struct v4l2_fmtdesc *f = arg;
		if (f->index < 0 || f->index >= NUM_CAPFMT)
			return -EINVAL;
		*f = capfmt[dev->id][f->index];
		return 0;
	}

	case VIDIOC_G_FMT:
	{
		struct v4l2_format *fmt = arg;
		if (fmt->type != V4L2_BUF_TYPE_CAPTURE) {
			dbg("G_FMT wrong buffer type %d\n",fmt->type);
			return -EINVAL;
		}
		fmt->fmt.pix = dev->clientfmt;
		return 0;
	}

	case VIDIOC_S_FMT:
	{
		struct v4l2_format *fmt = arg;

		if (o->noncapturing) {
			dbg("S_FMT illegal in non-capturing open\n");
			return -EPERM;
		}
		if (fmt->type != V4L2_BUF_TYPE_CAPTURE) {
			dbg("S_FMT wrong buffer type %d\n",fmt->type);
			return -EINVAL;
		}
		dev->clientfmt = fmt->fmt.pix;

		capture_abort (dev);
		if (capture_new_format (dev))
			return -EINVAL;
		if (dev->streaming)
			capture_stream_start (dev);
		else if (dev->preview)
			capture_grab_frame (dev);
		fmt->fmt.pix = dev->clientfmt;
		return 0;
	}

	case VIDIOC_G_COMP:
		return -EINVAL;
	case VIDIOC_S_COMP:
		return -EINVAL;

	case VIDIOC_REQBUFS:
	{
		struct v4l2_requestbuffers *req = arg;
		if (o->noncapturing) {
			dbg("REQBUFS illegal in non-capturing open\n");
			return -EPERM;
		}
		if (dev->stream_buffers_mapped) {
			dbg("Can't request buffers if buffers are "
			         "already mapped\n");
			return -EPERM;
		}
		mmap_unrequest_buffers (dev);
		capture_begin (dev);
		if (!mmap_request_buffers (dev, req))
			return -EINVAL;
		return 0;
	}

	case VIDIOC_QUERYBUF:
	{
		struct v4l2_buffer *buf = arg;
		int i;
		if (o->noncapturing) {
			dbg("QUERYBUF illegal in non-capturing open\n");
			return -EPERM;
		}
		i = buf->index;
		if (i < 0 || i >= MAX_CAPTURE_BUFFERS ||
		    !dev->stream_buf[i].requested ||
		    (buf->type & V4L2_BUF_TYPE_field) !=
		    (dev->stream_buf[i].vidbuf.
		     type & V4L2_BUF_TYPE_field)) {
			dbg("QUERYBUF bad parameter\n");
			return -EINVAL;
		}
		*buf = dev->stream_buf[i].vidbuf;
		return 0;
	}

	case VIDIOC_QBUF:
	{
		struct v4l2_buffer *buf = arg;
		if (o->noncapturing) {
			dbg("QBUF illegal in non-capturing open\n");
			return -EPERM;
		}
		if (!dev->stream_buffers_mapped) {
			dbg("QBUF no buffers are mapped\n");
			return -EINVAL;
		}
		if (!capture_queuebuffer (dev, buf))
			return -EINVAL;
		return 0;
	}

	case VIDIOC_DQBUF:
	{
		struct v4l2_buffer *buf = arg;
		if (o->noncapturing) {
			dbg("DQBUF illegal in non-capturing open\n");
			return -EPERM;
		}
		if (!capture_dequeuebuffer (dev, buf))
			return -EINVAL;
		return 0;
	}

	case VIDIOC_STREAMON:
	{
		__u32 *type = arg;
		if (o->noncapturing) {
			dbg("STREAMON illegal in non-capturing open\n");
			return -EPERM;
		}
		if (!capture_streamon (dev, *type))
			return -EINVAL;
		return 0;
	}

	case VIDIOC_STREAMOFF:
	{
		__u32 *type = arg;
		if (o->noncapturing) {
			dbg("STREAMOFF illegal in non-capturing open\n");
			return -EPERM;
		}
		capture_streamoff (dev, *type);
		return 0;
	}

	case VIDIOC_ENUM_FBUFFMT:
		return -EINVAL;
	case VIDIOC_G_FBUF:
		return -EINVAL;
	case VIDIOC_S_FBUF:
		return -EINVAL;
	case VIDIOC_G_WIN:
		return -EINVAL;
	case VIDIOC_S_WIN:
		return -EINVAL;

	case VIDIOC_PREVIEW:
	{
		int *on = (unsigned int *) arg;
		dbg ("VIDIOC_PREVIEW %d\n", *on); /* *revisit-skranz* temp.*/
		if (!dev->preview && (*on == 1)) {
			/* It is "off" now but we want it "on", so...*/

			/*
			 * schedule the framebuffer info updater to run
			 * (it has to run under keventd).
			 */
			dev->fbinfo_valid = 0;
			fbinfo_tsk_q_entry.routine =
				update_fbinfo_task;
			fbinfo_tsk_q_entry.data = (void *) dev;
			schedule_task (&fbinfo_tsk_q_entry);
			interruptible_sleep_on (&dev->fbinfo_wait);
			if (signal_pending (current))
				return -ERESTARTSYS;

			dev->preview = 1;
			capture_grab_frame (dev);
			if (!dev->ready_to_capture) {
				dbg ("ioctl(..,VIDIOC_PREVIEW,...); Can't grab frames!\n");
				return -EINVAL;
			}
		} else {
			if (dev->preview && (*on == 0)) {
				/* It is "on" now but we want it "off", so...*/
				capture_abort (dev);
				dev->ready_to_capture = 0;
				dev->preview = 0;
			}
		}
		return 0;
	}

	case VIDIOC_G_PERF:
	{
		memcpy (arg, &dev->perf, sizeof (dev->perf));
		return 0;
	}

	case VIDIOC_G_INPUT:
	{
		memcpy (arg, &dev->input, sizeof (dev->input));
		return 0;
	}

	case VIDIOC_S_INPUT:
	{
		int input = *(int *) arg;
		if (input < 0 || input >= dev->videc.num_inputs) {
			dbg("Input out of range %d\n", input);
			return -EINVAL;
		}
		if (input != dev->input) {
			dev->SwitchInputs = 1;
			dev->input = input;
			set_video_input (dev, input);
		}
		return 0;
	}

	case VIDIOC_G_PARM:
	{
		struct v4l2_streamparm *sp = arg;
		if (sp->type != V4L2_BUF_TYPE_CAPTURE)
			return -EINVAL;
		sp->parm.capture = dev->capture;
		return 0;
	}

	case VIDIOC_S_PARM:
	{
		struct v4l2_streamparm *sp = arg;
		struct v4l2_captureparm *vp = &sp->parm.capture;

		if (vp->capturemode & ~dev->capture.capability) {
			dbg("PARM unsupported capture capability %08X\n",
			    vp->capturemode);
			return -EINVAL;
		}
		if ((dev->capture.capability & V4L2_CAP_TIMEPERFRAME)
		    && vp->timeperframe < 10000) {
			dbg("PARM time per frame out of range %ld\n",
			    vp->timeperframe);
			return -EINVAL;
		}
		if (vp->capturemode != dev->capture.capturemode &&
		    !o->noncapturing && dev->streaming) {
			dbg("S_PARM state error\n"); 
			return -EINVAL;
		}
		
		if (o->noncapturing)
			return 0;
		
		if (vp->capturemode != dev->capture.capturemode ||
		    vp->timeperframe != dev->capture.timeperframe) {
			int frame_period;
			dev->capture.capturemode = vp->capturemode;

			if (vp->capturemode & V4L2_MODE_HIGHQUALITY)
				frame_period = 666667; /* 15 fps */
			else
				frame_period = vp->timeperframe;
			
			capture_abort (dev);
			dev->videc.set_frame_period (dev, frame_period);

			if (frame_period >= dev->videc.frame_period)
				dev->capture.timeperframe = frame_period;
			else
				dev->capture.timeperframe =
					dev->videc.frame_period;

			if (dev->streaming)
				capture_stream_start (dev);
			else if (dev->preview)
				capture_grab_frame (dev);
			
		}
		return 0;
	}

	case VIDIOC_G_STD:
	{
		struct v4l2_standard *std = arg;
		v4l2_video_std_construct (std, dev->videc.standard,
					  0);
		return 0;
	}

	case VIDIOC_S_STD:
	{
		struct v4l2_standard *std = arg;
		int id;
		if ((o->noncapturing && dev->capturing_opens))
			return -EPERM;
		id = v4l2_video_std_confirm (std);
		if (!((1 << id) & dev->videc.standards)) {
			dbg("Bad standard: %u\n", (unsigned)id);
			return -EINVAL;
		}
		capture_abort (dev);
		dev->videc.set_standard (dev, id);
		if (capture_new_format (dev))
			return -EINVAL;
		if (dev->streaming)
			capture_stream_start (dev);
		else if (dev->preview)
			capture_grab_frame (dev);
		return 0;
	}

	case VIDIOC_ENUMSTD:
	{
		struct v4l2_enumstd *estd = arg;
		__u32 b, i;
		if (estd->index < 0 || estd->index > 30)
			return -EINVAL;
		for (b = 1, i = 0; b < 32; ++b) {
			if (((1 << b) & dev->videc.standards) == 0)
				continue;
			if (i == estd->index) {
				v4l2_video_std_construct (&estd->std,
							  b, 0);
				estd->inputs = (__u32) - 1;	/* all inputs */
				estd->outputs = 0;
				return 0;
			}
			++i;
		}
		return -EINVAL;
	}

	case VIDIOC_ENUMINPUT:
	{
		struct v4l2_input *vi = arg;
		if (vi->index < 0
		    || vi->index >= dev->videc.num_inputs)
			return -EINVAL;
		*vi = dev->source[vi->index].input;
		return 0;
	}

	case VIDIOC_QUERYCTRL:
	{
		struct v4l2_queryctrl *qc = arg;
		if (dev->camera)
			return dev->camera->query_control (qc);
		return -ENODEV;
	}

	case VIDIOC_QUERYMENU:
	{
		struct v4l2_querymenu *qm = arg;
		if (dev->camera)
			return dev->camera->query_menu (qm);
		return -ENODEV;
	}

	case VIDIOC_G_CTRL:
	{
		struct v4l2_control *vc = arg;
		if (dev->camera)
			return dev->camera->get_control (vc);
		return -ENODEV;
	}

	case VIDIOC_S_CTRL:
	{
		struct v4l2_control *vc = arg;
		if (dev->camera)
			return dev->camera->set_control (vc);
		return -ENODEV;
	}

	case VIDIOC_G_TUNER:
		return -EINVAL;
	case VIDIOC_S_TUNER:
		return -EINVAL;
	case VIDIOC_G_FREQ:
		return -EINVAL;
	case VIDIOC_S_FREQ:
		return -EINVAL;

	case VIDIOC_G_AUDIO:
		return -EINVAL;
	case VIDIOC_S_AUDIO:
		return -EINVAL;

	default:
        /* private device specific ioctl routies */
            if (dev->camif) {
                return dev->camif->ioctl(cmd, arg);
            } else {
	        return -ENOIOCTLCMD;
            }
	}
	return 0;
}

static int
v4l2_mmap (void *id, struct vm_area_struct *vma)
{
	struct device_open *o = (struct device_open *) id;
	struct capture_device *dev = o->dev;
	struct stream_buffer *buf;
	int i, n = 1;

#if 1   /* power management */
	if (capture_pwm.suspended) return -EPERM;
#endif  /* power management */

	if (o->noncapturing) {
		dbg("mmap() called on non-capturing open\n");
		return -ENODEV;
	}
	buf = mmap_stream_buffer_from_offset (dev, vma->vm_pgoff);

	if (dev->stream_contig_map) {	/*     N buffers in one contiguous map  */
		buf = &dev->stream_buf[0];
		n = dev->stream_buffers_requested;
	}
	if (buf == NULL) {
		dbg("mmap() Invalid offset parameter\n");
		return -EINVAL;	/* no such buffer */
	}
	if (buf->vidbuf.length * n != vma->vm_end - vma->vm_start) {
		dbg("mmap() Wrong length parameter\n");
		return -EINVAL;	/* wrong length */
	}
	for (i = 0; i < n; ++i) {
		if (!buf->requested) {
			dbg("mmap() Buffer is not available for"
			 " mapping\n");
			return -EINVAL;	/* not requested */
		}
		if (buf->vidbuf.flags & V4L2_BUF_FLAG_MAPPED) {
			dbg("mmap() Buffer is already mapped\n");
			return -EINVAL;	/* already mapped */
		}

		if (buf->vaddress != NULL)
			vfree (buf->vaddress);
		if (i == 0)
			buf->vaddress = vmalloc (buf->vidbuf.length * n);
		else
			buf->vaddress = buf[-1].vaddress + buf->vidbuf.length;
		if (buf->vaddress == NULL) {
			err ("Could not allocate mmap() buffer\n");
			return -ENODEV;
		}
#if 0				/*  TODO: build scatter list for buffer if using DMA  */
		if ((using DMA) &&
		    !bm_build_scatter_list (dev, buf->vaddress,
					    &buf->dma_list))
			return -ENODEV;
#endif
		buf->vidbuf.flags |= V4L2_BUF_FLAG_MAPPED;
		++dev->stream_buffers_mapped;
		++buf;
	}

	vma->vm_ops = &capture_vma_operations;
	if (vma->vm_ops->open)
		vma->vm_ops->open (vma);
	/*  Note: vma->vm_file will be set up by V4L2  */

	return 0;
}

static int
v4l2_poll (void *id, struct file *file, poll_table * table)
{
	struct device_open *o = (struct device_open *) id;
	struct capture_device *dev = o->dev;

#if 1   /* power management */
	if (capture_pwm.suspended) return -EPERM;
#endif /* power management */

	if (o->noncapturing) {
		dbg("poll() illegal in non-capturing open\n");
		return POLLERR;
	}

	if (dev->streaming) {
		void *node;
		node = v4l2_q_peek_head (&dev->stream_q_done);
		if (node != NULL)
			return (POLLIN | POLLRDNORM);	/* data is ready now */
		node = v4l2_q_peek_head (&dev->stream_q_capture);
		if (node == NULL)
			return POLLERR;	/* no frames queued */
		poll_wait (file, &dev->new_video_frame, table);
		return 0;
	}

	/*  Capture is through read() call */

	if (dev->capture_completed)	/* data is ready now */
		return (POLLIN | POLLRDNORM);

	capture_grab_frame (dev); /* does nothing if capture is in progress */
	if (!dev->ready_to_capture) {
		/* Can't grab frames! */
		dbg ("Can't grab frames!\n");
		return POLLERR;
	}

	poll_wait (file, &dev->new_video_frame, table);
	return 0;
}

static long
v4l2_read (void *id, char *buf, unsigned long count, int noblock)
{
	struct device_open *o = (struct device_open *) id;
	struct capture_device *dev = o->dev;
	long len = 0;
	long my_timeout;

#if 1   /* power management */
	if (capture_pwm.suspended) return -EPERM;
#endif /* power management */

	if (o->noncapturing) {
		dbg("read() illegal in non-capturing open\n");
		return -EPERM;
	}
	if (dev->streaming) {
		dbg("Can't read() when streaming is on\n");
		return -EPERM;
	}

	capture_grab_frame (dev); /* does nothing if capture is in progress */

	if (!dev->ready_to_capture) {
		dbg ("Can't grab frames!\n");
		return 0;
	}

	my_timeout = HZ / 5;
	while (len == 0) {
		if (!dev->capture_completed) {
			if (noblock)
				return -EAGAIN;
			my_timeout =
				interruptible_sleep_on_timeout (&dev->
								new_video_frame,
								my_timeout);
		}

		if (my_timeout == 0) {
			dbg("Timeout on read\n");
			break;
		}
		len = capture_read (dev, buf, count);
	}

	return len;
}

/*
 *      Remaining initialization of video decoder etc. This is only
 *      done when the device is successfully identified and registered.
 */
static int
v4l2_init_done (struct v4l2_device *v)
{
	struct capture_device *dev = (struct capture_device *) v;
	int i;
        dbg("Initialization completion\n");

	/*  Initialize video input array        */
	for (i = 0; i < VSOURCE_COUNT; ++i) {
		dev->source[i].input.index = i;
		dev->source[i].input.type = V4L2_INPUT_TYPE_CAMERA;
		dev->source[i].input.capability = 0;
	}
	strcpy (dev->source[VSOURCE_CAMERA].input.name, "Camera");
	strcpy (dev->source[VSOURCE_TEST].input.name, "Color Bar Test");
	/*strcpy(dev->source[VSOURCE_TUNER].input.name, "Tuner"); */
	/*dev->source[VSOURCE_TUNER].input.type = V4L2_INPUT_TYPE_TUNER; */

	/*  Initialize the video decoder hardware       */
	dev->videc.initialize (dev);
	dev->camif->open (dev->id);

	/*  BUG: get defaults from user somehow...  */
	dev->videc.set_standard (dev, V4L2_STD_NTSC);
	dev->videc.set_vcrmode (dev, 0);
	set_video_input (dev, VSOURCE_CAMERA);

	/*  Capture parameters  */
	dev->capture.capability =
		V4L2_CAP_TIMEPERFRAME | V4L2_MODE_HIGHQUALITY;
	dev->capture.capturemode = 0;
	dev->capture.extendedmode = 0;
	dev->capture.timeperframe = dev->videc.frame_period;

	/*  Default capture dimensions  */
	dev->clientfmt.depth = 16;
        dev->clientfmt.pixelformat = dev->id == 0 ?
                V4L2_PIX_FMT_RGB565: V4L2_PIX_FMT_YUV420;

	dev->clientfmt.width = mx2ads_image_size[QVGA].width;
	dev->clientfmt.height = mx2ads_image_size[QVGA].height;
	dev->clientfmt.flags = 0;
	dev->clientfmt.bytesperline = 0;
	dev->clientfmt.sizeimage = 0;

	capture_new_format (dev);

	dev->camif->close (dev->id);
	return 0;
}

/*  =====================================================================
 *      The functions below this point are only called during loading
 *      and unloading of the driver.
 */


/*
 *      D E V I C E   I N I A L I Z A T I O N   R O U T I N E S
 *
 *      These routines locate and enable the hardware, and initialize
 *      the device structure.
 */



/*  Initialize v4l2_device fields       */
static void
init_device_fields (struct capture_device *dev)
{
        int devnum = dev - &(capture[0]);
	sprintf (dev->v.name, "DGMX2ADS V4L2 Capture Driver %d", devnum);
        
	init_waitqueue_head (&dev->new_video_frame);
	init_waitqueue_head (&dev->fbinfo_wait);
   
	dev->v.type = V4L2_TYPE_CAPTURE;
	dev->v.minor = unit_video + devnum;

	dev->v.open = v4l2_open;
	dev->v.close = v4l2_close;
	dev->v.read = v4l2_read;
	dev->v.write = v4l2_write;
	dev->v.ioctl = v4l2_ioctl;
	dev->v.mmap = v4l2_mmap;
	dev->v.poll = v4l2_poll;
	dev->v.initialize = v4l2_init_done;
	dev->v.priv = NULL;
	dev->preview = 0;
}

static int
config_a_device (struct capture_device *dev)
{
	sprintf (dev->shortname, "capture");

	init_device_fields (dev);

	if (!find_decoder (dev)) {
		err ("Bad or unrecognized video decoder\n");
		return -ENODEV;
	}
	return 0;
}

static void
unconfig_a_device (struct capture_device *dev)
{
	capture_close (dev);

	if (dev->is_registered) {
		v4l2_unregister_device ((struct v4l2_device *) dev);
		dbg("Removed device %s\n", dev->shortname);
	}

	memset (dev, 0, sizeof (struct capture_device));
}


static void
update_fbinfo_task (void *dev)
{
	struct capture_device *device = (struct capture_device *) dev;

	if (device) {
		get_framebuffer_info (device);
		wake_up (&device->fbinfo_wait);
	}
}

#define MX2ADS_CAM_MIN_HCLK 33000
/*
 *      M O D U L E   I N I T   A N D   C L E A N U P
 */
int
mx2ads_v4l2_init (void)
{
        struct capture_device *dev;
	int retcode;
	int i;

        camif_t *camif = &camif_mx2ads;
        struct camera * camera;

#if 1 /*CEE LDM*/
	if (MX2ADS_CAM_MIN_HCLK > mx_module_get_clk(HCLK) / 1000) {
		err ("cannot initialize - HCLK too slow\n");
		return -EPERM;
	}
#endif
        
        /* Common initialization of the camera interface */
        if ((retcode = camif->init ())) {
                err ("Camera Interface init failed\n");
                return retcode;
        }
        
        camera = camif->camera_detect ();

        if (!camera) {
                info ("No camera detected.\n");
        }

#if 1   /* power management */
	init_completion(&capture_pwm.resume_thread_sync);
	init_MUTEX_LOCKED(&capture_pwm.resume_wait);
	capture_pwm.suspended = 0;	
	capture_pwm.resume_thread_exit = 0;
	capture_pwm.resume_thread_pid =
		kernel_thread(&resume_thread, &(capture[0]),
			      CLONE_FS | CLONE_FILES | CLONE_SIGHAND);
	if (capture_pwm.resume_thread_pid < 0) {
		err("could not start resume thread\n");
		return -ENODEV;
	}

#ifdef CONFIG_DPM
	mx2cam_constraints.param[0].min = MX2ADS_CAM_MIN_HCLK;	/*in KHz */
	mx2cam_constraints.param[0].max = mx_module_get_clk(MPLL) / 1000;	/* unlimited */
#endif

        mx21_ldm_bus_register(&mx2ads_camera_device_ldm,
                &mx2ads_camera_driver_ldm);

#ifdef CONFIG_PM
	mx2ads_image_pmdev = pm_register(PM_UNKNOWN_DEV, PM_SYS_UNKNOWN,
					 mx2ads_image_pm_callback);
	if (!mx2ads_image_pmdev)
		printk(KERN_WARNING
		       "MX2 Camera: failed to initialize power management\n");
#endif /* CONFIG_PM */
#endif /* power management */

        for (i = 0; i < CAMIF_CHANNELS_NUM; i++) {
                dev = &capture[i];
                dev->id = i;
                dev->camif  = camif;
                dev->camera = camera;
                if ((retcode = camif->init_chan(i,
                                         camif_capture_callback, dev))) {
                        err("Failed to init camif channel %d\n",i);
                        return -ENODEV;
                }

                if (config_a_device (dev)) {
                        unconfig_a_device (dev);
                        return -ENODEV;
                }

                if (v4l2_register_device ((struct v4l2_device *) dev) != 0) {
                        err ("Couldn't register the driver.\n");
                        unconfig_a_device (dev);
                        return -ENODEV;
                }

                dev->is_registered = 1;
        }
	dbg("MX2ADS CMOS Image Sensor Driver initialised\n");
        
	return 0;
}

static void
mx2ads_v4l2_cleanup (void)
{
        int i;
#if 1   /* power management */
	if (capture_pwm.resume_thread_pid >= 0) {
		capture_pwm.resume_thread_exit = 1;
		up(&capture_pwm.resume_wait);
		wait_for_completion (&capture_pwm.resume_thread_sync);
	}

	mx21_ldm_bus_unregister( &mx2ads_camera_device_ldm,
                &mx2ads_camera_driver_ldm);

#ifdef CONFIG_PM
	if (mx2ads_image_pmdev)
		pm_unregister(mx2ads_image_pmdev);
#endif
#endif /* power management */
        
        for (i = 0; i < CAMIF_CHANNELS_NUM; i++) {
	        unconfig_a_device (&capture[i]);
        }

	camif_mx2ads.cleanup ();

}

#ifndef MODULE
int init_v4l2_mx2ads (struct video_init *ignore)
{
	return mx2ads_v4l2_init();
}
#else
module_init (mx2ads_v4l2_init);
#endif
module_exit (mx2ads_v4l2_cleanup);
MODULE_LICENSE ("GPL");
