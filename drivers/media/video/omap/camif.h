/*
 * camif.h
 *
 * Definition of Camera Interface. Abstracts the low-level
 * camera interface hardware.
 *
 * Copyright (C) 2003 MontaVista Software, Inc.
 *
 * Author: MontaVista Software, Inc.
 *              stevel@mvista.com or source@mvista.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/videodev.h>
#include "camera.h"

// The Camera Serial Bus
typedef struct camera_serial_bus {
	int dev_id;
	
	int (*init)(void);
	void (*cleanup)(void);

	int (*set_devid)(int id);
	int (*read)(u8 addr, u8* buf, int len);
	int (*write)(u8 addr, u8* buf, int len);
	int (*write_verify)(u8 addr, u8 val);
} camera_sbus_t;

// Implementations of camera serial bus
extern struct camera_serial_bus camera_sbus_sccb;
extern struct camera_serial_bus camera_sbus_old_i2c;
extern struct camera_serial_bus camera_sbus_new_i2c;

typedef struct camera_interface {
	struct camera_serial_bus * sbus;
	struct camera * camera;
	
	struct camera * (*camera_detect)(void);

	// initialize and start camera interface
	int (*init)(void (*v4l2_callback)(void *), void* data);
	// shutdown camera interface
	void (*cleanup)(void);

	int (*open)(void);  // acquire h/w resources (irq,DMA), etc.
	int (*close)(void); // free h/w resources, stop i/f
	
	// allocate and free h/w buffers for image capture
	void* (*alloc_image_buffer)(int size);
	void (*free_image_buffer)(void* buffer, int size);

	// set frame period (units of .1 usec)
	int (*set_frame_period)(int fp);
	
	// starts h/w capture of one frame and then stop
	int (*snapshot)(u8* buf, int size);
	// starts continuous h/w capture into buf
	int (*start_streaming)(u8* buf, int size);
	// abort any active capture
	int (*abort)(void);
} camif_t;

// Implementations of camera interface
extern struct camera_interface camif_innovator;
extern struct camera_interface camif_evm;

