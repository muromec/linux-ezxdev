/*
 * Common audio handling for TI OMAP
 *
 * Copyright (c) 2000 Nicolas Pitre <nico@cam.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

/*
 *audio_stream_t and audio_buf_t moved to <asm/arch/dma-sound-omap1610.h>
 *(arch == arch-omap1610), because dma-omap.c exports
 *a fiew sound-dma-dedicated functions, which need some knowledge
 *about audio_stream_t.dma_regs
 */



/* Buffer Management */
#include <asm/arch/dma-sound-omap1610.h>
 

/*
 * State structure for one instance
 */

typedef struct {
	audio_stream_t *output_stream;
	audio_stream_t *input_stream;
	int rd_ref:1;		/* open reference for recording */
	int wr_ref:1;		/* open reference for playback */
	int need_tx_for_rx:1;	/* if data must be sent while receiving */
	void *data;
	void (*hw_init)(void *);
	void (*hw_shutdown)(void *);
	int (*client_ioctl)(struct inode *, struct file *, uint, ulong);
	struct pm_dev *pm_dev;
	struct semaphore sem;	/* to protect against races in attach() */
} audio_state_t;

/*
 * Functions exported by this module
 */
extern int omap_audio_attach( struct inode *inode, struct file *file,
				audio_state_t *state);
