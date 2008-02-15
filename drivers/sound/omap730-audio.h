/*
 *  linux/drivers/sound/omap730-audio.h -- audio interface for the OMAP730 chip
 *  Author: Jean Pihet <j-pihet@ti.com>
 *
 *  From linux/drivers/sound/pxa-audio.h
 *  Author:	Nicolas Pitre
 *  Created:	Aug 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

// User buffers struct
typedef struct {
	int offset;			/* current buffer position in the buffer */
	char *data; 	           	/* actual buffer */
} audio_user_buf_t;

// DMA buffer struct
typedef struct {
	char *vaddr;			/* DMA virtual address */
	dma_addr_t dma_addr;		/* DMA physical address */
} audio_dma_buf_t;

typedef struct {
	char *name;			/* stream identifier */
	audio_user_buf_t *user_buf;   	/* pointer to user audio buffers array */
	audio_dma_buf_t dma_buf;	/* DMA buffer info */
	u_int usr_frag;			/* user fragment index */
	u_int dma_frag;			/* DMA fragment index */
	u_int fragsize;			/* fragment size */
	u_int nbfrags;			/* number of fragments */
	u_int dma_ch;			/* DMA channel number */
	int bytecount;			/* nbr of processed bytes */
	int fragcount;			/* nbr of fragment transitions */
	struct semaphore sem;		/* account for fragment usage */
	wait_queue_head_t frag_wq;	/* for poll(), etc. */
	int mapped:1;			/* mmap()'ed buffers */
	int output:1;			/* 0 for input, 1 for output */
	int dma_running:1;		/* 0 if stopped, 1 if running */
} audio_stream_t;
	
typedef struct {
	audio_stream_t *output_stream;
	audio_stream_t *input_stream;
	int dev_dsp;			/* audio device handle */
	int rd_ref:1;           /* open reference for recording */
	int wr_ref:1;           /* open reference for playback */
	void (*hw_init)(void *);
	void (*hw_shutdown)(void *);
	int (*client_ioctl)(struct inode *, struct file *, uint, ulong);
	struct semaphore sem;		/* prevent races in attach/release */
} audio_state_t;

extern int omap_audio_attach(struct inode *inode, struct file *file,
				audio_state_t *state);
extern void omap_audio_clear_buf(audio_stream_t *s);


/* User buffers mgt macros */

// Has place: at least one buffer is free to use
#define USER_BUF_READ_HAS_PLACE(s)	( !(((s->dma_frag + 1) % s->nbfrags) == s->usr_frag) )

// Is empty: all the buffers are free to use
#define USER_BUF_IS_EMPTY(s)		( s->dma_frag == s->usr_frag )

