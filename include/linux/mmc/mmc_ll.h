/*
 * Header for MultiMediaCard (MMC)
 *
 * Copyright 2002 Hewlett-Packard Company
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * HEWLETT-PACKARD COMPANY MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
 * AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
 * FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 * Many thanks to Alessandro Rubini and Jonathan Corbet!
 *
 * Based strongly on code by:
 *
 * Author: Yong-iL Joh <tolkien@mizi.com>
 * Date  : $Date: 2002/10/03 05:21:14 $ 
 *
 * Author:  Andrew Christian
 *          15 May 2002
 *
 * Copyright (C) 2005 Motorola, Inc.
 * - Port to EzX platform.
 */

#ifndef MMC_MMC_LL_H
#define MMC_MMC_LL_H

#include <linux/types.h>
#include <linux/mmc/mmc_protocol.h>

#ifdef __KERNEL__

/* Error codes */
enum mmc_result_t {
	MMC_NO_RESPONSE        = -1,
	MMC_NO_ERROR           = 0,
	MMC_ERROR_OUT_OF_RANGE,
	MMC_ERROR_ADDRESS,
	MMC_ERROR_BLOCK_LEN,
	MMC_ERROR_ERASE_SEQ,
	MMC_ERROR_ERASE_PARAM,
	MMC_ERROR_WP_VIOLATION,
	MMC_ERROR_CARD_IS_LOCKED,
	MMC_ERROR_LOCK_UNLOCK_FAILED,
	MMC_ERROR_COM_CRC,
	MMC_ERROR_ILLEGAL_COMMAND,
	MMC_ERROR_CARD_ECC_FAILED,
	MMC_ERROR_CC,
	MMC_ERROR_GENERAL,
	MMC_ERROR_UNDERRUN,
	MMC_ERROR_OVERRUN,
	MMC_ERROR_CID_CSD_OVERWRITE,
	MMC_ERROR_STATE_MISMATCH,
	MMC_ERROR_HEADER_MISMATCH,
	MMC_ERROR_TIMEOUT,
	MMC_ERROR_CRC,
	MMC_ERROR_DRIVER_FAILURE,
};

struct mmc_request {
	int               index;      /* Slot index - used for CS lines */
	int               cmd;        /* Command to send */
	u32               arg;        /* Argument to send */
	enum mmc_rsp_t    rtype;      /* Response type expected */

	/* Data transfer (these may be modified at the low level) */
	u16               nob;        /* Number of blocks to transfer*/
	u16               block_len;  /* Block length */
	u8               *buffer;     /* Data buffer */
	u32               cnt;        /* Data length, for PIO */

	/* Results */
	u8                response[18]; /* Buffer to store response - CRC is optional */
	enum mmc_result_t result;
};

#define MMC_SDFLAG_SPI_MODE   (1<<0)    /* Can use SPI mode */
#define MMC_SDFLAG_MMC_MODE   (1<<1)    /* Can use MMC mode */
#define MMC_SDFLAG_SD_MODE    (1<<2)    /* Can use SD mode */
#define MMC_SDFLAG_VOLTAGE    (1<<3)    /* Can change voltage range */
#define MMC_SDFLAG_BLOCKADDR  (1<<4)    /* Uses block address */

struct module;

struct mmc_slot_driver {
	struct module  *owner;
	char           *name;
	u32             ocr;         /* Valid voltage ranges */
	u32             flags;       /* Slot driver flags */

	int  (*init)(void);   
	void (*cleanup)(void);
	int  (*is_empty)(int);
	int  (*is_wp)(int);
	int  (*set_clock)(u32 rate);
	void (*send_cmd)(struct mmc_request *);
};

/* Calls made by the hardware-specific slot driver code */
extern int  mmc_register_slot_driver( struct mmc_slot_driver *, int );
extern void mmc_unregister_slot_driver( struct mmc_slot_driver * );
extern void mmc_cmd_complete( struct mmc_request * );
extern void mmc_insert( int );
extern void mmc_eject( int );

#endif /* #ifdef __KERNEL__ */

#endif /* MMC_MMC_LL_H */

