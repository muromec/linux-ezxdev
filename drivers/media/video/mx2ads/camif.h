/*
 * drivers/media/video/mx2ads/camif.h
 *
 * Definition of Camera Interface. Abstracts the low-level
 * camera interface hardware.
 *
 * Author: MontaVista Software, Inc.
 *              stevel@mvista.com or source@mvista.com
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/videodev.h>
#include "camera.h"

#define  CSI_EXP_IO (*((volatile uint16_t *)(MX2ADS_PER_IOBASE + 0x800000)))

#define  EXP_IO_CSI_CTL2    0x0010
#define  EXP_IO_CSI_CTL1    0x0020
#define  EXP_IO_CSI_CTL0    0x0040

#define CSI_REG_READ(_reg, _var) \
     do {                                 \
	     (_var) = CSI_ ##_reg ;  \
     } while (0)

#define CSI_REG_WRITE(_reg, _var) \
     do {                                 \
	     CSI_ ##_reg = (_var);   \
     } while (0)
#define CSI_IRQ_ALL   (CSICR1_SOF_INTEN |  \
		       CSICR1_STATFF_INTEN | CSICR1_RXFF_INTEN | \
		       CSICR1_RXFFOR_INTEN | CSICR1_STATFFOR_INTEN |  \
		       CSICR1_COF_INTEN | CSICR1_EOF_INTEN)

#define CSI_IRQ_SOF             (CSICR1_SOF_INTEN)
#define CSI_GPIO_MASK           (0x003ffc00)

#ifndef CONFIG_MX2TO1
#define MX2CAM_PERCLKDIV        (0x2)
#endif

#define CAMIF_CHANNELS_NUM      (2)

/* The Camera Serial Bus */
typedef struct camera_serial_bus {
	int dev_id;
	
	int (*init)(void);
	void (*cleanup)(void);

	int (*set_devid)(int id);
	int (*read)(u8 addr, u8* buf, int len);
	int (*write)(u8 addr, u8* buf, int len);
	int (*write_verify)(u8 addr, u8 val);
} camera_sbus_t;

/* Implementations of camera serial bus */
extern struct camera_serial_bus camera_mx2ads_ov9640_i2c;
extern struct camera_serial_bus camera_mx2ads_misoc0343_i2c;
    
typedef struct camera_interface {
	struct camera_serial_bus * sbus;
	struct camera * camera;
	
	struct camera * (*camera_detect)(void);

	/* initialize and start camera interface */
	int (*init)(void);
        
        int (*init_chan)(unsigned int chan, void (*v4l2_callback)(void *),
                        void* data);
	/* shutdown camera interface */
	void (*cleanup)(void);

	int (*open)(int channel);  /* acquire h/w resources (irq,DMA), etc. */
	int (*close)(int channel); /* free h/w resources, stop i/f */
	
	/* allocate and free h/w buffers for image capture */
	void* (*alloc_image_buffer)(int size);
	void (*free_image_buffer)(void* buffer, int size);

	/* set frame period (units of .1 usec) */
	int (*set_frame_period)(int fp);
	
	/* starts h/w capture of one frame and then stop */
	int (*snapshot)(unsigned int chan, u8* buf, int size);
	/* starts continuous h/w capture into buf */
	int (*start_streaming)(unsigned int chan, u8* buf, int size);
	/* abort any active capture */
	int (*abort)(unsigned int chan);
        /* Private ioctl intrface */
	int (*ioctl)(unsigned int cmd, void *arg);
        /* set pixel formar fot camera interface interface */
	int (*set_format)(unsigned int chan, struct v4l2_pix_format* fmt);
	/*
	 * convert and decode raw camera image if needed, or simply
	 * copy src to dest if no decoding necessary.
	 */
	int (*convert_image)(int chan, u8* src, void* dest, int to_user,
			     int dest_stride,
			     struct v4l2_pix_format* fmt);

} camif_t;


typedef struct camif_chan_stat_tag {
        int opened;
        int snapshot_active;
        int streaming_active;
        void (*capture_callback)(void *);
        void* callback_data;
        int width;
        int height;
        int depth;
        unsigned int pixfmt;
        wait_queue_head_t abortqueue;
        int abort;              /* Abort request */
} canif_chan_stat_t;

extern struct camera_interface camif_mx2ads;
