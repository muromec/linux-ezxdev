/*
 * usbd/usbd-chap9.h 
 *
 *      Copyright (c) 2004 Belcarra
 *
 * Adapted from earlier work:
 *      Copyright (c) 2002, 2003 Belcarra
 *      Copyright (c) 2000, 2001, 2002 Lineo
 *      Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@belcarra.com>, 
 *      Tom Rushworth <tbr@belcarra.com>, 
 *      Bruce Balden <balden@belcarra.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 *  2005-Nov-11  change for usb2.0,  Li xin
 *  2005-Dec-12  change for re-enum,  Zhao liang
 */

/*
 * USB Descriptors are used to build a configuration database for each USB
 * Function driver.
 *
 */


/*
 * Device and/or Interface Class codes
 */
#define USB_CLASS_PER_INTERFACE         0	/* for DeviceClass */
#define USB_CLASS_AUDIO                 1
#define USB_CLASS_COMM                  2
#define USB_CLASS_HID                   3
#define USB_CLASS_PHYSICAL              5
#define USB_CLASS_PRINTER               7
#define USB_CLASS_MASS_STORAGE          8
#define USB_CLASS_HUB                   9
#define USB_CLASS_DATA                  10
#define USB_CLASS_APP_SPEC              0xfe
#define USB_CLASS_VENDOR_SPEC           0xff

/*
 * USB Device Request Types (bmRequestType)
 * C.f. USB 2.0 Table 9-2
 */
#define USB_TYPE_STANDARD               (0x00 << 5)
#define USB_TYPE_CLASS                  (0x01 << 5)
#define USB_TYPE_VENDOR                 (0x02 << 5)
#define USB_TYPE_RESERVED               (0x03 << 5)

#define USB_RECIP_DEVICE                0x00
#define USB_RECIP_INTERFACE             0x01
#define USB_RECIP_ENDPOINT              0x02
#define USB_RECIP_OTHER                 0x03

#define USB_DIR_OUT                     0
#define USB_DIR_IN                      0x80
#define	USB_ENDPOINT_OPT		0x40



/*
 * Descriptor types
 * C.f. USB Table 9-5
 */
#define USB_DT_DEVICE                   0x01
#define USB_DT_CONFIG                   0x02
#define USB_DT_STRING                   0x03
#define USB_DT_INTERFACE                0x04
#define USB_DT_ENDPOINT                 0x05
#define USB_DT_DEVICE_QUALIFIER            0x06
#define USB_DT_OTHER_SPEED_CONFIGURATION   0x07
#define USB_DT_INTERFACE_POWER             0x08



#define USB_DT_HID                      (USB_TYPE_CLASS | 0x01)
#define USB_DT_REPORT                   (USB_TYPE_CLASS | 0x02)
#define USB_DT_PHYSICAL                 (USB_TYPE_CLASS | 0x03)
#define USB_DT_HUB                      (USB_TYPE_CLASS | 0x09)

/*
 * Descriptor sizes per descriptor type
 */
#define USB_DT_DEVICE_SIZE              18
#define USB_DT_CONFIG_SIZE              9
#define USB_DT_INTERFACE_SIZE           9
#define USB_DT_ENDPOINT_SIZE            7
#define USB_DT_ENDPOINT_AUDIO_SIZE      9	/* Audio extension */
#define USB_DT_HUB_NONVAR_SIZE          7
#define USB_DT_HID_SIZE                 9

/*
 * Endpoints
 */
#define USB_ENDPOINT_NUMBER_MASK        0x0f	/* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK           0x80

#define USB_ENDPOINT_MASK               0x03	/* in bmAttributes */
#define USB_ENDPOINT_CONTROL            0x00
#define USB_ENDPOINT_ISOCHRONOUS        0x01
#define USB_ENDPOINT_BULK               0x02
#define USB_ENDPOINT_INTERRUPT          0x03

/*
 * USB Packet IDs (PIDs)
 */
#define USB_PID_UNDEF_0                        0xf0
#define USB_PID_OUT                            0xe1
#define USB_PID_ACK                            0xd2
#define USB_PID_DATA0                          0xc3
#define USB_PID_PING                           0xb4	/* USB 2.0 */
#define USB_PID_SOF                            0xa5
#define USB_PID_NYET                           0x96	/* USB 2.0 */
#define USB_PID_DATA2                          0x87	/* USB 2.0 */
#define USB_PID_SPLIT                          0x78	/* USB 2.0 */
#define USB_PID_IN                             0x69
#define USB_PID_NAK                            0x5a
#define USB_PID_DATA1                          0x4b
#define USB_PID_PREAMBLE                       0x3c	/* Token mode */
#define USB_PID_ERR                            0x3c	/* USB 2.0: handshake mode */
#define USB_PID_SETUP                          0x2d
#define USB_PID_STALL                          0x1e
#define USB_PID_MDATA                          0x0f	/* USB 2.0 */

/*
 * Standard requests
 */
#define USB_REQ_GET_STATUS              0x00
#define USB_REQ_CLEAR_FEATURE           0x01
#define USB_REQ_SET_FEATURE             0x03
#define USB_REQ_SET_ADDRESS             0x05
#define USB_REQ_GET_DESCRIPTOR          0x06
#define USB_REQ_SET_DESCRIPTOR          0x07
#define USB_REQ_GET_CONFIGURATION       0x08
#define USB_REQ_SET_CONFIGURATION       0x09
#define USB_REQ_GET_INTERFACE           0x0A
#define USB_REQ_SET_INTERFACE           0x0B
#define USB_REQ_SYNCH_FRAME             0x0C

/*
 * HID requests
 */
#define USB_REQ_GET_REPORT              0x01
#define USB_REQ_GET_IDLE                0x02
#define USB_REQ_GET_PROTOCOL            0x03
#define USB_REQ_SET_REPORT              0x09
#define USB_REQ_SET_IDLE                0x0A
#define USB_REQ_SET_PROTOCOL            0x0B


/*
 * USB Spec Release number
 */

#define USB_BCD_VERSION                 0x0200

/*
 * Audio
 */
#define CS_AUDIO_UNDEFINED              0x20
#define CS_AUDIO_DEVICE                 0x21
#define CS_AUDIO_CONFIGURATION          0x22
#define CS_AUDIO_STRING                 0x23
#define CS_AUDIO_INTERFACE              0x24
#define CS_AUDIO_ENDPOINT               0x25

#define AUDIO_HEADER                    0x01
#define AUDIO_INPUT_TERMINAL            0x02
#define AUDIO_OUTPUT_TERMINAL           0x03
#define AUDIO_MIXER_UNIT                0x04
#define AUDIO_SELECTOR_UNIT             0x05
#define AUDIO_FEATURE_UNIT              0x06
#define AUDIO_PROCESSING_UNIT           0x07
#define AUDIO_EXTENSION_UNIT            0x08


/*
 * Device Requests      (c.f Table 9-2)
 */

#define USB_REQ_DIRECTION_MASK          0x80
#define USB_REQ_TYPE_MASK               0x60
#define USB_REQ_RECIPIENT_MASK          0x1f

#define USB_REQ_DEVICE2HOST             0x80
#define USB_REQ_HOST2DEVICE             0x00

#define USB_REQ_TYPE_STANDARD           0x00
#define USB_REQ_TYPE_CLASS              0x20
#define USB_REQ_TYPE_VENDOR             0x40

#define USB_REQ_RECIPIENT_DEVICE        0x00
#define USB_REQ_RECIPIENT_INTERFACE     0x01
#define USB_REQ_RECIPIENT_ENDPOINT      0x02
#define USB_REQ_RECIPIENT_OTHER         0x03

/*
 * get status bits
 */

#define USB_STATUS_SELFPOWERED          0x01
#define USB_STATUS_REMOTEWAKEUP         0x02

#define USB_STATUS_HALT                 0x01

/* 
 * descriptor types
 */

#define USB_DESCRIPTOR_TYPE_DEVICE                      0x01
#define USB_DESCRIPTOR_TYPE_CONFIGURATION               0x02
#define USB_DESCRIPTOR_TYPE_STRING                      0x03
#define USB_DESCRIPTOR_TYPE_INTERFACE                   0x04
#define USB_DESCRIPTOR_TYPE_ENDPOINT                    0x05
#define USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER            0x06
#define USB_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION   0x07
#define USB_DESCRIPTOR_TYPE_INTERFACE_POWER             0x08

/*
 * standard feature selectors
 */
#define	USB_ENDPOINT_HALT		0x00
#define USB_DEVICE_REMOTE_WAKEUP	0x01
#define	USB_TEST_MODE			0x02


/* USB Requests
 *
 */

struct usb_device_request {
	__u8 bmRequestType;
	__u8 bRequest;
	__u16 wValue;
	__u16 wIndex;
	__u16 wLength;
} __attribute__ ((packed));


/*
 * Class-Specific Request Codes
 * C.f. CDC Table 46
 */
#define CDC_CLASS_REQUEST_SEND_ENCAPSULATED	0x00
#define CDC_CLASS_REQUEST_GET_ENCAPSULATED	0x01

#define CDC_CLASS_REQUEST_SET_COMM_FEATURE	0x02
#define CDC_CLASS_REQUEST_GET_COMM_FEATURE	0x03
#define CDC_CLASS_REQUEST_CLEAR_COMM_FEATURE	0x04

#define CDC_CLASS_REQUEST_SET_LINE_CODING 	0x20
#define CDC_CLASS_REQUEST_GET_LINE_CODING 	0x21

#define CDC_CLASS_REQUEST_SET_CONTROL_STATE	0x22
#define CDC_CLASS_REQUEST_SEND_BREAK        	0x23

/*
 * Notification codes
 * c.f. CDC Table 68
 */

#define CDC_NOTIFICATION_NETWORK_CONNECTION         0x00
#define CDC_NOTIFICATION_RESPONSE_AVAILABLE         0x01
#define CDC_NOTIFICATION_AUX_JACK_HOOK_STATE        0x08
#define CDC_NOTIFICATION_RING_DETECT                0x09
#define CDC_NOTIFICATION_SERIAL_STATE               0x20
#define CDC_NOTIFICATION_CALL_STATE_CHANGE          0x28
#define CDC_NOTIFICATION_LINE_STATE_CHANGE          0x29
#define CDC_NOTIFICATION_CONNECTION_SPEED_CHANGE    0x2a

/*
 * HID - Class Descriptors
 * C.f. 7.1.1
 */
#define HID		0x21
#define	HID_REPORT	0x22
#define	HID_PHYSICAL	0x23

/*
 * HID Descriptor
 * C.f. E.8
 */
struct hid_descriptor {
	u8 bLength;
	u8 bDescriptorType;
	u16 bcdHID;
	u8 bCountryCode;
	u8 bNumDescriptors;
	u8 bReportType;
	u16 wItemLength;
} __attribute__((packed));

/*
 * ACM - Line Coding structure
 * C.f CDC Table 50
 */
struct cdc_acm_line_coding {
	__u32 dwDTERate;
	__u8 bCharFormat;
	__u8 bParityType;
	__u8 bDataBits;
} __attribute__((packed));

/*
 * ACM
 * C.f CDC Table 50
 */


/*
 * ACM
 * C.f CDC Table 51
 */


/* USB Notification
 *
 */

struct cdc_notification_descriptor {
        __u8 bmRequestType;
        __u8 bNotification;
	__u16 wValue;
	__u16 wIndex;
        __u16 wLength;
	__u8 data[2];
} __attribute__ ((packed));




/*
 * communications class types
 *
 * c.f. CDC  USB Class Definitions for Communications Devices 
 * c.f. WMCD USB CDC Subclass Specification for Wireless Mobile Communications Devices
 *
 */

#define CLASS_BCD_VERSION		0x0110

// c.f. CDC 4.1 Table 14
#define AUDIO_CLASS	                0x01
#define COMMUNICATIONS_DEVICE_CLASS	0x02

// c.f. CDC 4.2 Table 15
#define COMMUNICATIONS_INTERFACE_CLASS	0x02

// c.f. CDC 4.3 Table 16
#define COMMUNICATIONS_NO_SUBCLASS	0x00
#define COMMUNICATIONS_DLCM_SUBCLASS	0x01
#define COMMUNICATIONS_ACM_SUBCLASS	0x02
#define COMMUNICATIONS_TCM_SUBCLASS	0x03
#define COMMUNICATIONS_MCCM_SUBCLASS	0x04
#define COMMUNICATIONS_CCM_SUBCLASS	0x05
#define COMMUNICATIONS_ENCM_SUBCLASS	0x06
#define COMMUNICATIONS_ANCM_SUBCLASS	0x07

#define AUDIO_CONTROL_SUBCLASS          0x01
#define AUDIO_STREAMING_SUBCLASS        0x02

// c.f. WMCD 5.1
#define COMMUNICATIONS_WHCM_SUBCLASS	0x08
#define COMMUNICATIONS_DMM_SUBCLASS	0x09
#define COMMUNICATIONS_MDLM_SUBCLASS	0x0a
#define COMMUNICATIONS_OBEX_SUBCLASS	0x0b

// c.f. CDC 4.6 Table 18
#define DATA_INTERFACE_CLASS		0x0a

// c.f. CDC 4.7 Table 19
#define COMMUNICATIONS_NO_PROTOCOL	0x00


// c.f. CDC 5.2.3 Table 24
// c.f. Audio Appendix A.4
#define CS_UNDEFINED             	0x20
#define CS_DEVICE                	0x21
#define CS_CONFIG                	0x22
#define CS_STRING                	0x23
#define CS_INTERFACE			0x24
#define CS_ENDPOINT			0x25

// Audio Interface Class - c.f Appendix A.1
#define	AUDIO_INTERFACE_CLASS		0x01

// Audio Interface Subclass - c.f Appendix A.2
#define	AUDIO_SUBCLASS_UNDEFINED	0x00
#define	AUDIO_AUDIOCONTROL		0x01
#define	AUDIO_AUDIOSTREAMING		0x02
#define	AUDIO_MIDISTREAMING		0x03

// Audio Interface Proctol - c.f. Appendix A.3
#define	AUDIO_PR_PROTOCOL_UNDEFINED	0x00

// Audio Class-Specific AC Interface Descriptor Subtypes - c.f. A.5
#define	AUDIO_AC_DESCRIPTOR_UNDEFINED 	0x00
#define	AUDIO_AC_HEADER			0x01
#define	AUDIO_AC_INPUT_TERMINAL		0x02
#define	AUDIO_AC_OUTPUT_TERMINAL	0x03
#define	AUDIO_AC_MIXER_UNIT		0x04
#define	AUDIO_AC_SELECTOR_UNIT		0x05
#define	AUDIO_AC_FEATURE_UNIT		0x06
#define	AUDIO_AC_PROCESSING_UNIT	0x07
#define	AUDIO_AC_EXTENSION_UNIT		0x08

// Audio Class-Specific AS Interface Descriptor Subtypes - c.f. A.6
#define	AUDIO_AS_DESCRIPTOR_UNDEFINED 	0x00
#define	AUDIO_AS_GENERAL		0x01
#define	AUDIO_AS_FORMAT_TYPE		0x02
#define	AUDIO_AS_FORMAT_UNSPECFIC	0x03

// Audio Class Processing Unit Processing Types - c.f. A.7
#define	AUDIO_PROCESS_UNDEFINED		0x00
#define	AUDIO_UP_DOWN_MUIX_PROCESS	0x01
#define	AUDIO_DOLBY_PROLOGIC_PROCESS	0x02
#define	AUDIO_3D_STEREO_EXTENDER_PROCESS 0x03
#define	AUDIO_REVERBERATION_PROCESS	0x04
#define	AUDIO_CHORUS_PROCESS		0x05
#define	AUDIO_DYN_RANGE_COMP_PROCESS	0x06

// Audio Class-Specific Endpoint Descriptor Subtypes - c.f. A.8
#define	AUDIO_DESCRIPTOR_UNDEFINED	0x00
#define	AUDIO_EP_GENERAL		0x01

// Audio Class-Specific Request Codes - c.f. A.9
#define	AUDIO_REQUEST_CODE_UNDEFINED	0x00
#define	AUDIO_SET_CUR			0x01
#define	AUDIO_GET_CUR			0x81
#define	AUDIO_SET_MIN			0x02
#define	AUDIO_GET_MIN			0x82
#define	AUDIO_SET_MAX			0x03
#define	AUDIO_GET_MAX			0x83
#define	AUDIO_SET_RES			0x04
#define	AUDIO_GET_RES			0x84
#define	AUDIO_SET_MEM			0x05
#define	AUDIO_GET_MEM			0x85
#define	AUDIO_GET_STAT			0xff






/*
 * bDescriptorSubtypes
 *
 * c.f. CDC 5.2.3 Table 25
 * c.f. WMCD 5.3 Table 5.3
 */

#define USB_ST_HEADER			0x00
#define USB_ST_CMF			0x01
#define USB_ST_ACMF			0x02
#define USB_ST_DLMF			0x03
#define USB_ST_TRF			0x04
#define USB_ST_TCLF			0x05
#define USB_ST_UF			0x06
#define USB_ST_CSF			0x07
#define USB_ST_TOMF			0x08
#define USB_ST_USBTF			0x09
#define USB_ST_NCT			0x0a
#define USB_ST_PUF			0x0b
#define USB_ST_EUF			0x0c
#define USB_ST_MCMF			0x0d
#define USB_ST_CCMF			0x0e
#define USB_ST_ENF			0x0f
#define USB_ST_ATMNF			0x10

#define USB_ST_WHCM			0x11
#define USB_ST_MDLM			0x12
#define USB_ST_MDLMD			0x13
#define USB_ST_DMM			0x14
#define USB_ST_OBEX			0x15
#define USB_ST_CS			0x16
#define USB_ST_CSD			0x17
#define USB_ST_TCM			0x18



/*
 * commuications class description structures
 *
 * c.f. CDC 5.1
 * c.f. WCMC 6.7.2
 *
 * XXX add the other dozen class descriptor description structures....
 */

struct usb_header_description {
	__u8 bDescriptorSubtype;
	__u16 bcdCDC;
};

struct usb_call_management_description {
	__u8 bmCapabilities;
	__u8 bDataInterface;
};

struct usb_abstract_control_description {
	__u8 bmCapabilities;
};

struct usb_union_function_description {
	__u8 bMasterInterface;
	__u8 bSlaveInterface[1];
	//__u8        bSlaveInterface[0];                    // XXX FIXME
};

struct usb_ethernet_networking_description {
	char *iMACAddress;
	__u8 bmEthernetStatistics;
	__u16 wMaxSegmentSize;
	__u16 wNumberMCFilters;
	__u8 bNumberPowerFilters;
};

struct usb_mobile_direct_line_model_description {
	__u16 bcdVersion;
	__u8 bGUID[16];
};

struct usb_mobile_direct_line_model_detail_description {
	__u8 bGuidDescriptorType;
	__u8 bDetailData[2];
	//__u8      bDetailData[0];                         // XXX FIXME
};

struct usb_class_description {
	__u8 bDescriptorSubtype;
	__u8 elements;
	union {
		struct usb_header_description header;
		struct usb_call_management_description call_management;
		struct usb_abstract_control_description abstract_control;
		struct usb_union_function_description union_function;
		struct usb_ethernet_networking_description ethernet_networking;
		struct usb_mobile_direct_line_model_description mobile_direct;
		struct usb_mobile_direct_line_model_detail_description mobile_direct_detail;
	} description;
};

/* endpoint modifiers
 * static struct usb_endpoint_description function_default_A_1[] = {
 *
 *     {this_endpoint: 0, attributes: CONTROL,   max_size: 8,  polling_interval: 0 },
 *     {this_endpoint: 1, attributes: BULK,      max_size: 64, polling_interval: 0, direction: IN},
 *     {this_endpoint: 2, attributes: BULK,      max_size: 64, polling_interval: 0, direction: OUT},
 *     {this_endpoint: 3, attributes: INTERRUPT, max_size: 8,  polling_interval: 0},
 *
 *
 */
#define OUT		0x00
#define IN		0x80

#define CONTROL		0x00
#define ISOCHRONOUS	0x01
#define BULK		0x02
#define INTERRUPT	0x03


/* configuration modifiers
 */
#define BMATTRIBUTE_RESERVED		0x80
#define BMATTRIBUTE_SELF_POWERED	0x40

#define POWER_CURRENT_UNIT	2	// multiplue of 2mA from USB SPEC
#define POWER_CURRENT_100MA_SETTING	(100/POWER_CURRENT_UNIT)
#define POWER_CURRENT_500MA_SETTING	(500/POWER_CURRENT_UNIT)


/*
 * The UUT tester specifically tests for MaxPower to be non-zero (> 0).
 */
#if !defined(CONFIG_USBD_MAXPOWER) || (CONFIG_USBD_MAXPOWER == 0)
	#define BMATTRIBUTE BMATTRIBUTE_RESERVED | BMATTRIBUTE_SELF_POWERED
	#define BMAXPOWER 1
#else
	#define BMATTRIBUTE BMATTRIBUTE_RESERVED
	#define BMAXPOWER CONFIG_USBD_MAXPOWER
#endif




/* 
 * standard usb descriptor structures
 */

struct usb_endpoint_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;	// 0x5
	__u8 bEndpointAddress;
	__u8 bmAttributes;
	__u16 wMaxPacketSize;
	__u8 bInterval;
} __attribute__ ((packed));

struct usb_interface_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;	// 0x04
	__u8 bInterfaceNumber;
	__u8 bAlternateSetting;
	__u8 bNumEndpoints;
	__u8 bInterfaceClass;
	__u8 bInterfaceSubClass;
	__u8 bInterfaceProtocol;
	__u8 iInterface;
} __attribute__ ((packed));

struct usb_configuration_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;	// 0x2
	__u16 wTotalLength;
	__u8 bNumInterfaces;
	__u8 bConfigurationValue;
	__u8 iConfiguration;
	__u8 bmAttributes;
	__u8 bMaxPower;
} __attribute__ ((packed));

struct usb_device_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;	// 0x01
	__u16 bcdUSB;
	__u8 bDeviceClass;
	__u8 bDeviceSubClass;
	__u8 bDeviceProtocol;
	__u8 bMaxPacketSize0;
	__u16 idVendor;
	__u16 idProduct;
	__u16 bcdDevice;
	__u8 iManufacturer;
	__u8 iProduct;
	__u8 iSerialNumber;
	__u8 bNumConfigurations;
} __attribute__ ((packed));

struct usb_string_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;	// 0x03
	__u16 wData[0];
} __attribute__ ((packed));

struct usb_device_qualifier_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;
	__u16 bcdUSB;
	__u8 bDeviceClass;
	__u8 bDeviceSubClass;
	__u8 bDeviceProtocol;
	__u8 bMaxPacketSize0;
	__u8 bNumConfigurations;
	__u8 bReserved;
} __attribute__ ((packed));

struct usb_generic_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;
} __attribute__ ((packed));

struct usb_generic_class_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;
} __attribute__ ((packed));


/* 
 * communications class descriptor structures
 *
 * c.f. CDC 5.2 Table 25c
 */

struct usb_class_function_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;
} __attribute__ ((packed));

struct usb_class_function_descriptor_generic {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;
	__u8 bmCapabilities;
} __attribute__ ((packed));

struct usb_class_header_function_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x00
	__u16 bcdCDC;
} __attribute__ ((packed));

struct usb_class_call_management_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x01
	__u8 bmCapabilities;
	__u8 bDataInterface;
} __attribute__ ((packed));

struct usb_class_abstract_control_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x02
	__u8 bmCapabilities;
} __attribute__ ((packed));

struct usb_class_direct_line_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x03
} __attribute__ ((packed));

struct usb_class_telephone_ringer_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x04
	__u8 bRingerVolSeps;
	__u8 bNumRingerPatterns;
} __attribute__ ((packed));

struct usb_class_telephone_call_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x05
	__u8 bmCapabilities;
} __attribute__ ((packed));

struct usb_class_union_function_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x06
	__u8 bMasterInterface;
	__u8 bSlaveInterface0[0];
} __attribute__ ((packed));

struct usb_class_country_selection_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x07
	__u8 iCountryCodeRelDate;
	__u16 wCountryCode0[0];
} __attribute__ ((packed));


struct usb_class_telephone_operational_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x08
	__u8 bmCapabilities;
} __attribute__ ((packed));


struct usb_class_usb_terminal_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x09
	__u8 bEntityId;
	__u8 bInterfaceNo;
	__u8 bOutInterfaceNo;
	__u8 bmOptions;
	__u8 bChild0[0];
} __attribute__ ((packed));

struct usb_class_network_channel_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x0a
	__u8 bEntityId;
	__u8 iName;
	__u8 bChannelIndex;
	__u8 bPhysicalInterface;
} __attribute__ ((packed));

struct usb_class_protocol_unit_function_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x0b
	__u8 bEntityId;
	__u8 bProtocol;
	__u8 bChild0[0];
} __attribute__ ((packed));

struct usb_class_extension_unit_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x0c
	__u8 bEntityId;
	__u8 bExtensionCode;
	__u8 iName;
	__u8 bChild0[0];
} __attribute__ ((packed));

struct usb_class_multi_channel_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x0d
	__u8 bmCapabilities;
} __attribute__ ((packed));

struct usb_class_capi_control_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x0e
	__u8 bmCapabilities;
} __attribute__ ((packed));

struct usb_class_ethernet_networking_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x0f
	__u8 iMACAddress;
	__u32 bmEthernetStatistics;
	__u16 wMaxSegmentSize;
	__u16 wNumberMCFilters;
	__u8 bNumberPowerFilters;
} __attribute__ ((packed));

struct usb_class_atm_networking_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x10
	__u8 iEndSystermIdentifier;
	__u8 bmDataCapabilities;
	__u8 bmATMDeviceStatistics;
	__u16 wType2MaxSegmentSize;
	__u16 wType3MaxSegmentSize;
	__u16 wMaxVC;
} __attribute__ ((packed));


struct usb_class_mdlm_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x12
	__u16 bcdVersion;
	__u8 bGUID[16];
} __attribute__ ((packed));

struct usb_class_mdlmd_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;	// 0x13
	__u8 bGuidDescriptorType;
	__u8 bDetailData[0];

} __attribute__ ((packed));


/* 
 * descriptor union structures
 */

struct usb_descriptor {
	union {
		struct usb_generic_descriptor generic;
		struct usb_generic_class_descriptor generic_class;
		struct usb_endpoint_descriptor endpoint;
		struct usb_interface_descriptor interface;
		struct usb_configuration_descriptor configuration;
		struct usb_device_descriptor device;
		struct usb_string_descriptor string;
	} descriptor;

} __attribute__ ((packed));

struct usb_class_descriptor {
	union {
		struct usb_class_function_descriptor function;
		struct usb_class_function_descriptor_generic generic;
		struct usb_class_header_function_descriptor header_function;
		struct usb_class_call_management_descriptor call_management;
		struct usb_class_abstract_control_descriptor abstract_control;
		struct usb_class_direct_line_descriptor direct_line;
		struct usb_class_telephone_ringer_descriptor telephone_ringer;
		struct usb_class_telephone_operational_descriptor telephone_operational;
		struct usb_class_telephone_call_descriptor telephone_call;
		struct usb_class_union_function_descriptor union_function;
		struct usb_class_country_selection_descriptor country_selection;
		struct usb_class_usb_terminal_descriptor usb_terminal;
		struct usb_class_network_channel_descriptor network_channel;
		struct usb_class_extension_unit_descriptor extension_unit;
		struct usb_class_multi_channel_descriptor multi_channel;
		struct usb_class_capi_control_descriptor capi_control;
		struct usb_class_ethernet_networking_descriptor ethernet_networking;
		struct usb_class_atm_networking_descriptor atm_networking;
		struct usb_class_mdlm_descriptor mobile_direct;
		struct usb_class_mdlmd_descriptor mobile_direct_detail;
	} descriptor;

} __attribute__ ((packed));


/*
 * Audio Status Word Format - c.f. Table 3.1
 */
#define AUDIO_STATUS_INTERRUPT_PENDING		1<<7
#define AUDIO_STATUS_MEMORY_CONTENT_CHANGED	1<<6
#define AUDIO_STATUS_ORIGINATOR_AUDIO_CONTROL_INTERFACE			0
#define AUDIO_STATUS_ORIGINATOR_AUDIO_STREAMING_INTERFACE		1
#define AUDIO_STATUS_ORIGINATOR_AUDIO_STREAMING_ENDPOINT		2
#define AUDIO_STATUS_ORIGINATOR_AUDIOCONTROL_INTERFACE			0

struct usb_audio_status_word {
        __u8 bStatusType;
        __u8 bOriginator;
} __attribute__ ((packed));

struct usb_audio_ac_interface_header_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;
        __u16 bcdADC;
        __u16 wTotalLength;
        __u8 binCollection;
        __u8 bainterfaceNr[0];
} __attribute__ ((packed));

struct usb_audio_input_terminal_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;
        __u8 bTerminalID;
        __u16 wTerminalType;
        __u8 bAssocTerminal;
        __u8 bNrChannels;
        __u16 wChannelConfig;
        __u8 iChannelNames;
        __u8 iTerminal;
} __attribute__ ((packed));

struct usb_audio_input_terminal_description {
        struct usb_audio_input_terminal_descriptor *audio_input_terminal_descriptor;
        __u16 wTerminalType;
        __u16 wChannelConfig;
        char * iChannelNames;
        char * iTerminal;
}; 

struct usb_audio_output_terminal_descriptor {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;
        __u8 bTerminalID;
        __u16 wTerminalType;
        __u8 bAssocTerminal;
        __u8 bSourceID;
        __u8 iTerminal;
} __attribute__ ((packed));

struct usb_audio_output_terminal_description {
        struct usb_audio_output_terminal_descriptor *audio_output_terminal_descriptor;
        __u16 wTerminalType;
        char * iTerminal;
}; 

struct usb_audio_mixer_unit_descriptor_a {
	__u8 bFunctionLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;
        __u8 bUniteID;
        __u8 bNrinPins;
} __attribute__ ((packed));

struct usb_audio_mixer_unit_descriptor_b {
        __u8 baSourceID;
        __u8 bNrChannels;
        __u16 wChannelConfig;
        __u8 iChannelNames;
        __u8 bmControls;
        __u8 iMixer;
} __attribute__ ((packed));

struct usb_audio_mixer_unit_description {
        struct usb_audio_mixer_unit_descriptor *audio_mixer_unit_descriptor;
        __u16 wChannelConfig;
        char * iMixer;
}; 


struct usb_audio_as_general_interface_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;	
	__u8 bDescriptorSubtype;	
        __u8 bTerminalLink;
        __u8 bDelay;
        __u16 wFormatTag;
} __attribute__ ((packed));

struct usb_audio_as_general_interface_description {
        struct usb_audio_as_general_interface_descriptor *audio_as_general_interface_descriptor;
        __u16 wFormatTag;
}; 

struct usb_audio_as_format_type_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;	
	__u8 bDescriptorSubtype;	
        __u8 bFormatType;
        __u8 bNrChannels;
        __u8 bSubFrameSize;
        __u8 bBitResolution;
        __u8 bSamFreqType;
        __u8 iSamFreq[3];
} __attribute__ ((packed));

struct usb_audio_as_format_type_description {
        struct usb_audio_as_format_type_descriptor *audio_as_format_type_descriptor;
        __u16 wFormatTag;
}; 




struct usb_audio_descriptor {
	union {
                struct usb_audio_ac_interface_header_descriptor header;
                struct usb_audio_input_terminal_descriptor input;
                struct usb_audio_output_terminal_descriptor output;
	} descriptor;
} __attribute__ ((packed));



struct usb_ac_endpoint_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;
	__u8 bEndpointAddress;
	__u8 bmAttributes;
	__u16 wMaxPacketSize;
	__u8 bInterval;
        __u8 bRefresh;
        __u8 bSynchAddress;
} __attribute__ ((packed));


struct usb_as_iso_endpoint_descriptor {
	__u8 bLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;
	__u8 bmAttributes;
        __u8 bLockDelayUnits;
        __u16 wLockDelay;
} __attribute__ ((packed));


