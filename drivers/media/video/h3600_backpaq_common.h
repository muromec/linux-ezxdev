/*
 * Driver for the Compaq iPAQ Mercury Backpaq camera
 * Video4Linux interface
 *
 * Copyright 2002 Compaq Computer Corporation.
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * COMPAQ COMPUTER CORPORATION MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
 * AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
 * FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 * Author: Andrew Christian 
 *         <andyc@handhelds.org>
 *         4 May 2001
 *
 * Common data structures
 */


/* A single frame of raw data from the camera       */
/* We can always tell the active mode from the size */

#ifndef _H3600_BACKPAQ_COMMON_H
#define _H3600_BACKPAQ_COMMON_H

#include <asm/arch/h3600_backpaq_camera.h>

enum frame_state {
	FRAME_DONE = 0,
	FRAME_PENDING,
	FRAME_ACTIVE,
	FRAME_FULL
};

struct frame {
	unsigned char *data;           /* May point to true_data OR to shared memory segment */
	unsigned char *local_data;     /* Our local storage area for raw frames */
	unsigned char *shared_data;    /* Points into shared memory "frame_buf" */
	int            bytes_read;     /* How full it is          */
	int            bytes_to_read;  /* Normally width * height */
	int            width;
	int            height;
	volatile enum frame_state state;
	struct frame *next;    /* Linked list */
};

#endif /* _H3600_BACKPAQ_COMMON_H */

