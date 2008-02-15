/*
 * drivers/media/video/mx2ads/v4l2.h
 *
 * Description:
 *   Video for Linux Two Interface for mx2ads Capture device.
 *   This driver based on capture driver for OMAP camera
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


struct capture_device;

struct video_decoder {
	int is_initialized;
	int ntsc_hskip;
	int ntsc_vskip;
	int ntsc_width;
	int ntsc_height;
	int ntsc_field_order;
	int pal_hskip;
	int pal_vskip;
	int pal_width;
	int pal_height;
	int pal_field_order;
	int preferred_field;

	int num_inputs;
	int input;
	__u32 standards;
	__u32 standard;
	__u32 frame_period;
	int decoder_is_stable;
	__u32 decoder_stable_time;

	/*  Changable method functions helps us support multiple        */
	/*  different types of video decoders easily                    */
	int (*initialize) (struct capture_device * dev);
	int (*set_input) (struct capture_device * dev, int x);
	int (*set_standard) (struct capture_device * dev, int x);
	int (*set_vcrmode) (struct capture_device * dev, int x);
	int (*is_stable) (struct capture_device * dev);
	int (*set_frame_period) (struct capture_device * dev, int fp);
};


struct video_source {
	struct v4l2_input input;
	struct v4l2_tuner tuner;
	int vcrmode;
};

/*  Indices into the array of video_source's */
#define VSOURCE_CAMERA   0
#define VSOURCE_TEST     1
//#define VSOURCE_TUNER  2
#define VSOURCE_COUNT    2

/*  Bus-master scatter list  */
struct scatter_node {
	__u32 addr;
	__u32 len;
};
#define END_OF_SCATTER_LIST     0x80000000

/*  Per-open data for handling multiple opens on one device */
struct device_open {
	int isopen;
	int noncapturing;
	struct capture_device *dev;
};
#define MAX_OPENS       3

/*  Streaming data buffer  */
struct stream_buffer {
	struct v4l2_q_node qnode;
	struct v4l2_buffer vidbuf;
	int requested;
	__u8 *vaddress;		/* vmalloc() */
	struct scatter_node *dma_list;	/* get_free_page() */
	int vma_refcount;
};
#define MAX_CAPTURE_BUFFERS     30
#define MAX_LOCKED_MEMORY       2000000

/* Power Management Suspend/Resume stuff */
struct capture_pwm_t {
	int suspended;
	int resume_thread_exit;
	int resume_thread_pid;
	struct semaphore resume_wait;
	struct completion resume_thread_sync;
};

/*
 *      Capture device structure
 *
 *      One for each handled device in the system.
 *      This structure holds all the global information the driver
 *      needs about each device.
 */
struct capture_device {
	struct v4l2_device v;	/*  Must be first */
	char shortname[16];
	int is_registered;
	int open_count;
	struct device_open open_data[MAX_OPENS];
	int capturing_opens;
	int id;

/*      Per-bus index number for each device    */
	int index;
/*      General type of device  */
	int type;

	struct tq_struct tqnode_dpc;	/* for Bottom Half routine */
//        struct timer_list       tlnode;/* for polling interrupts */
	wait_queue_head_t new_video_frame;
	wait_queue_head_t fbinfo_wait;

/*      Video decoder stuff             */
	struct video_decoder videc;
	struct video_source source[VSOURCE_COUNT];

/*      Client capture format and capture modes */
	struct v4l2_pix_format clientfmt;
	struct v4l2_captureparm capture;
	int input;		/* which video source is selected */

/*      Hardware capture format  */
	int capture_bypp;
	int capture_size;
	__u8 *capture_buffer;	/* vmalloc() */
	int capture_buffer_size;
	struct scatter_node *capture_dma_list;	/* get_free_page() */

/*      Capture state   */
	int ready_to_capture;
	int capture_started;
	int capture_completed;
	unsigned long time_acquired;	/* millisecond time stamp */
	int streaming;
	struct stream_buffer stream_buf[MAX_CAPTURE_BUFFERS];
	int stream_buffers_requested;
	int stream_buffers_mapped;
	int stream_contig_map;
	struct v4l2_queue stream_q_capture;
	struct v4l2_queue stream_q_done;
	stamp_t stream_begin;
	unsigned long stream_last_frame;
	__u8 *stream_capture_buffer;

/*      Performance statistics  */
	struct v4l2_performance perf;
	int preview;		/* 1==preview on, 0==preview off. */

	/* The Camera Interface Hardware object */
	struct camera_interface *camif;
	/* The Camera object */
	struct camera *camera;

	int SwitchInputs;	/* 1==user has requested different video input.*/

	/* info from framebuffer */
	struct fb_fix_screeninfo fbfix;
	struct fb_var_screeninfo fbvar;
	int fbinfo_valid;

	/* frame counter for test images */
	int h, m, s, f;
};


/*      Values for type field   */
#define DEVICE_TYPE_0           0

