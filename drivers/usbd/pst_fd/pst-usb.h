/*
 *  Copyright (C) Intrinsyc, Inc., 2002
 *
 *  usb_ep0.h - PXA USB controller driver.
 *              Endpoint zero management
 *
 *  Please see:
 *    linux/Documentation/arm/SA1100/SA1100_USB
 *  for details.
 *
 *  02-May-2002
 *   Frank Becker (Intrinsyc) - 
 * 
 */
/*
 * Copyright (C) 2005 - Motorola
 *
 * 2003-JUL-01 - Modified for EZXBASE By Levis <Levis@motorola.com>
 *
 */
#ifndef __USB_EP0_H
#define __USB_EP0_H
#include <asm/byteorder.h>

//LIN change code by the swith
#define PST_BULKINTF 1
#define PST_DUOINTF 1



#define EP0_FIFO_SIZE	16
#define SETUP_READY (UDCCS0_SA | UDCCS0_OPR)

#define make_word_c( w ) __constant_cpu_to_le16(w)
#define make_word( w )   __cpu_to_le16(w)
#define make_word_b( w )   __cpu_to_be16(w)
/*================================================
 * USB Protocol Stuff
 */

/* Request Codes   */
enum { 
	GET_STATUS		=0,
	CLEAR_FEATURE		=1,
	/* reserved		=2 */
	SET_FEATURE		=3,
	/* reserved		=4 */
	SET_ADDRESS		=5,        
	GET_DESCRIPTOR		=6,
	SET_DESCRIPTOR		=7,
	GET_CONFIGURATION	=8,
	SET_CONFIGURATION	=9,
	GET_INTERFACE		=10,
	SET_INTERFACE		=11,
	SYNCH_FRAME		=12
};

typedef enum {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_END_XFER,
	EP0_OUT_DATA_PHASE
} EP0_state;

/* USB Device Requests */
typedef struct
{
	__u8 bmRequestType;
	__u8 bRequest;
	__u16 wValue;
	__u16 wIndex;
	__u16 wLength;
} usb_dev_request_t  __attribute__ ((packed));

/* Data extraction from usb_request_t fields */
enum { 
	kTargetDevice	=0,
	kTargetInterface=1,
	kTargetEndpoint	=2 
};

#define DATA_READY           1
#define DATA_NOT_READY       0

typedef struct {
  __u16 size;
} msg_t;

typedef struct {
  struct list_head list;
  __u16 size;
  __u8* body;
//  __u8* nouse;
} msg_node_t;

typedef struct {
  int enable;
  int list_len;
  struct list_head in_msg_list;
  struct list_head out_msg_list;
} interface_t;

#define MAX_MESSAGE_NUM 3

typedef struct {
  __u8 status;
  __u8 numMessages;
  __u16 msgsiz[MAX_MESSAGE_NUM]; 
} polling_data_t;

typedef struct {
  __u8 numMsgs;
  __u8 numQueuedMsgs;
  __u16 numQueuedBytes;  
} queued_data_t;

extern interface_t intf6, intf8;

/* Stage definitions in Control Read/Write */
enum { 
  invalid_stage=-1,
  setup_stage=0,
  data_stage=1,
  status_stage=2,
  bulkin_stage=3, /* this is an extension to the protocol. hiahia */
};

/* 0x01 -- Device to Host 
 * 0x00 -- Host to Device
 */
#define DATA_IN  0x01
#define DATA_OUT 0x00

#define DATA_DIRECTION(req) ((req>>7)&0x01)

enum
{
   USB_CDC_CFG,
   USB_FULL_CFG,
   USB_DSPLOGGER_CFG,
   USB_CDC_PLUS_DSPDEBUG_CFG,
   USB_MCULOGGER_PLUS_DSPDEBUG_CFG,
   USB_TEST_CMD_CFG,
   USB_DSPDEBUG_CFG,
   USB_MCU_TESTING_CFG,
   USB_DATALOGGER_ONLY_CFG,
   USB_PLUGFEST_TEST_CFG,
   USB_NUM_OF_CFG_SETS
}; 
int usb_gpio_init(void);
int stop_usb(void);

#endif
