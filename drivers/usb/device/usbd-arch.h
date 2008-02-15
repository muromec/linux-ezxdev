/*
 * linux/drivers/usbd/usbd-arch.h 
 *
 * Copyright (c) 2000, 2001, 2002 Lineo
 * Copyright (c) 2001 Hewlett Packard
 *
 * By: 
 *      Stuart Lynne <sl@lineo.com>, 
 *      Tom Rushworth <tbr@lineo.com>, 
 *      Bruce Balden <balden@lineo.com>
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
 * This file contains pre-canned configurations for specific architectures
 * and systems.
 *
 * The architecture specific areas concentrates on setting defaults that
 * will work with each architecture (endpoint restrictions etc).
 *
 * The system specific areas set defaults appropriate for a specific
 * system or board (vendor id etc).
 *
 * Typically during early development you will set these via the kernel
 * configuration and migrate the values into here once specific values
 * have been tested and will no longer change.
 */


/*
 * Lineo specific 
 */

#define VENDOR_SPECIFIC_CLASS           0xff
#define VENDOR_SPECIFIC_SUBCLASS        0xff
#define VENDOR_SPECIFIC_PROTOCOL        0xff

/*
 * Lineo Classes
 */
#define LINEO_CLASS            		0xff

#define LINEO_SUBCLASS_SAFENET          0x01
#define LINEO_SUBCLASS_SAFESERIAL       0x02

/*
 * Lineo Protocols
 */
#define	LINEO_SAFENET_CRC 		0x01
#define	LINEO_SAFENET_CRC_PADDED	0x02

#define	LINEO_SAFESERIAL_CRC 		0x01
#define	LINEO_SAFESERIAL_CRC_PADDED	0x02


/*
 * Architecture specific endpoint configurations
 */

#if defined(CONFIG_ARCH_SA1100) || defined(CONFIG_ARCH_SA1100_MODULE)
#warning
#warning SETTING DEFAULTS FOR SA1110
/*
 * The StrongArm SA-1110 has fixed endpoints and the bulk endpoints
 * are limited to 64 byte packets and does not support ZLP.
 */

        #define ABS_OUT_ADDR                            1
        #define ABS_IN_ADDR                             2
        #define ABS_INT_ADDR                            0

        #define MAX_OUT_PKTSIZE                         64
        #define MAX_IN_PKTSIZE                          64
        #undef MAX_INT_PKTSIZE

        #undef CONFIG_USBD_SERIAL_OUT_ENDPOINT
        #undef CONFIG_USBD_SERIAL_IN_ENDPOINT
        #undef CONFIG_USBD_SERIAL_INT_ENDPOINT

        #define CONFIG_USBD_NO_ZLP_SUPPORT
	
	#define	CONFIG_USBD_MAC_AS_SERIAL_NUMBER	1

#endif

#if defined(CONFIG_ARCH_L7200)|| defined(CONFIG_ARCH_L7200_MODULE)
#warning
#warning SETTING DEFAULTS FOR L7200
/*
 * The Linkup L7205/L7210 has fixed endpoints and the bulk endpoints
 * are limited to 32 byte packets and does not support ZLP.
 *
 * To get around specific hardware problems the Linkup eliminates optional 
 * strings in the configuration, reverses the CDC data interfaces.
 * 
 * It also requires all USB packets sent and received to be padded. They
 * must be either 31 or 32 bytes in length.
 *
 * XXX The new version of the L7210 may not require padding.
 */
        #define ABS_OUT_ADDR                            1
        #define ABS_IN_ADDR                             2
        #define ABS_INT_ADDR                            3

        #define MAX_OUT_PKTSIZE                         32
        #define MAX_IN_PKTSIZE                          32
        #define MAX_INT_PKTSIZE                         16

        #undef CONFIG_USBD_SERIAL_OUT_ENDPOINT
        #undef CONFIG_USBD_SERIAL_IN_ENDPOINT
        #undef CONFIG_USBD_SERIAL_INT_ENDPOINT

        #define CONFIG_USBD_NO_ZLP_SUPPORT
        #define CONFIG_USBD_NET_NO_STRINGS
        #define CONFIG_USBD_NET_PADDED
	#define CONFIG_USBD_NET_CDC_DATA_REVERSED

	#undef CONFIG_USBD_NET_CDC
	#undef CONFIG_USBD_NET_MDLM
	#undef CONFIG_USBD_NET_SAFE

	#define CONFIG_USBD_NET_MDLM			1
	#define	CONFIG_USBD_MAC_AS_SERIAL_NUMBER	1

#endif

#if defined(CONFIG_USBD_SL11_BUS)|| defined(CONFIG_USBD_SL11_BUS_MODULE)
#warning
#warning SETTING DEFAULTS FOR SL11
/*
 * The SL11 endpoints can be a mix of 1, 2, or 3. The SL11 endpoints have
 * fixed addresses but each can be used for Bulk IN, Bulk OUT and Interrupt.
 *
 * The default addresses can be overridden by using the kernel configuration
 * and setting the net-fd IN, OUT and INT addresses. See Config.in
 */

        #define DFL_OUT_ADDR                            1
        #define DFL_IN_ADDR                             2
        #define DFL_INT_ADDR                            3

        #define MAX_OUT_ADDR                            3
        #define MAX_IN_ADDR                             3
        #define MAX_INT_ADDR                            3

        #define MAX_OUT_PKTSIZE                         64
        #define MAX_IN_PKTSIZE                          64
        #define MAX_INT_PKTSIZE                         16

#endif

#if defined(CONFIG_USBD_SUPERH_BUS)|| defined(CONFIG_USBD_SUPERH_BUS_MODULE)
#warning
#warning SETTING DEFAULTS FOR SUPERH
/*
 * The SuperH 7727 has fixed endpoints and does not support ZLP.
 */

        #define ABS_OUT_ADDR                            1
        #define ABS_IN_ADDR                             2
        #define ABS_INT_ADDR                            3

        #define MAX_OUT_PKTSIZE                         64
        #define MAX_IN_PKTSIZE                          64
        #define MAX_INT_PKTSIZE                         16

	#undef CONFIG_USBD_NET_CDC
	#undef CONFIG_USBD_NET_MDLM
	#undef CONFIG_USBD_NET_SAFE

	#define CONFIG_USBD_NET_MDLM			1
	#define	CONFIG_USBD_MAC_AS_SERIAL_NUMBER	1
#endif

#if defined(CONFIG_USBD_PXA_BUS)|| defined(CONFIG_USBD_PXA_BUS_MODULE)
#warning
#warning SETTING DEFAULTS FOR PXA
/*
 * The PXA has fixed endpoints 
 */
        #define ABS_IN_ADDR                             1
        #define ABS_OUT_ADDR                            2
        #define ABS_INT_ADDR                            5

        #define MAX_OUT_PKTSIZE                         64
        #define MAX_IN_PKTSIZE                          64
        #define MAX_INT_PKTSIZE                         8

	#undef CONFIG_USBD_NET_CDC
	#undef CONFIG_USBD_NET_MDLM
	#undef CONFIG_USBD_NET_SAFE

	#define CONFIG_USBD_NET_MDLM			1
	#define	CONFIG_USBD_MAC_AS_SERIAL_NUMBER	1
#endif

/* ********************************************************************************************* */

/*
 * known vendor and product ids
 * Serial and network vendor ids can be overidden with kernel config.
 */

/*
 * For itsy we will use Compaq iPaq vendor ID and 0xffff for product id
 */
#if defined(CONFIG_SA1100_ITSY) || defined(CONFIG_SA1100_H3XXX) || defined(CONFIG_SA1100_ITSY_MODULE) || defined(CONFIG_SA1100_H3XXX_MODULE)

	#warning CONFIGURING FOR IPAQ H3XXX
	
	#undef CONFIG_USBD_VENDORID
	#undef CONFIG_USBD_PRODUCTID
	#define CONFIG_USBD_VENDORID		0x049f
	#define CONFIG_USBD_PRODUCTID		0xffff

	#undef CONFIG_USBD_VENDORID
	#define CONFIG_USBD_VENDORID		0x04dd
	#undef CONFIG_USBD_SERIAL_PRODUCTID
	#define CONFIG_USBD_SERIAL_PRODUCTID	0x8001
	#undef CONFIG_USBD_NET_PRODUCTID
	#define CONFIG_USBD_NET_PRODUCTID	0x8003


	#undef CONFIG_USBD_SELFPOWERED
	#undef	CONFIG_USBD_MANUFACTURER
	#undef CONFIG_USBD_PRODUCT_NAME

	#define CONFIG_USBD_SELFPOWERED		1
	#define	CONFIG_USBD_MANUFACTURER	"Compaq"
	#define CONFIG_USBD_PRODUCT_NAME	"iPaq"
	
	#undef CONFIG_USBD_NET_CDC
	#undef CONFIG_USBD_NET_MDLM
	#undef CONFIG_USBD_NET_SAFE

	#define CONFIG_USBD_NET_MDLM			1
	#define	CONFIG_USBD_MAC_AS_SERIAL_NUMBER	1
#endif

/*
 * Assigned vendor and product ids for HP Calypso
 */
#if defined(CONFIG_SA1100_CALYPSO) || defined(CONFIG_SA1110_CALYPSO) || defined(CONFIG_SA1100_CALYPSO_MODULE) || defined(CONFIG_SA1110_CALYPSO_MODULE)
	
	#warning CONFIGURING FOR CALYPSO

	#undef CONFIG_USBD_VENDORID
	#undef CONFIG_USBD_PRODUCTID
	#define CONFIG_USBD_VENDORID		0x03f0
	#define CONFIG_USBD_PRODUCTID		0x2101

	#undef CONFIG_USBD_SELFPOWERED
	#undef	CONFIG_USBD_MANUFACTURER
	#undef CONFIG_USBD_PRODUCT_NAME

	#define CONFIG_USBD_SELFPOWERED		1
	#define	CONFIG_USBD_MANUFACTURER	"HP"
	#define CONFIG_USBD_PRODUCT_NAME	"Journada X25"
#endif

/*
 * Assigned vendor and serial/network product ids for Sharp Iris
 */
#if defined(CONFIG_ARCH_L7200) && defined(CONFIG_IRIS) || defined(CONFIG_ARCH_L7200_MODULE) && defined(CONFIG_IRIS_MODULE)

	#warning CONFIGURING FOR LINKUP

	#undef CONFIG_USBD_VENDORID
	#define CONFIG_USBD_VENDORID		0x04dd
	#undef CONFIG_USBD_SERIAL_PRODUCTID
	#define CONFIG_USBD_SERIAL_PRODUCTID	0x8001
	#undef CONFIG_USBD_NET_PRODUCTID
	#define CONFIG_USBD_NET_PRODUCTID	0x8003

	#undef CONFIG_USBD_SELFPOWERED
	#undef	CONFIG_USBD_MANUFACTURER
	#undef CONFIG_USBD_PRODUCT_NAME

	#define CONFIG_USBD_SELFPOWERED		1
	#define	CONFIG_USBD_MANUFACTURER	"Sharp"
	#define CONFIG_USBD_PRODUCT_NAME	"Iris"

	#undef CONFIG_USBD_NET_CDC
	#undef CONFIG_USBD_NET_MDLM
	#undef CONFIG_USBD_NET_SAFE

	#define CONFIG_USBD_NET_MDLM			1
	#define	CONFIG_USBD_MAC_AS_SERIAL_NUMBER	1
#endif


/*
 * Assigned vendor and serial/network product ids for Sharp Collie
 */
#if defined(CONFIG_SA1100_COLLIE) || defined(CONFIG_SA1100_COLLIE_MODULE)
	
	#warning CONFIGURING FOR COLLIE

	#undef CONFIG_USBD_VENDORID
	#define CONFIG_USBD_VENDORID		0x04dd
	#undef CONFIG_USBD_SERIAL_PRODUCT ID
	#define CONFIG_USBD_SERIAL_PRODUCT ID	0x8002
	#undef CONFIG_USBD_NET_PRODUCTID
	//#define CONFIG_USBD_NET_PRODUCTID	0x8004
	#define CONFIG_USBD_NET_PRODUCTID	0x8003

	#undef CONFIG_USBD_SELFPOWERED
	#undef	CONFIG_USBD_MANUFACTURER
	#undef CONFIG_USBD_PRODUCT_NAME

	#define CONFIG_USBD_SELFPOWERED		1
	#define	CONFIG_USBD_MANUFACTURER	"Sharp"
	#define CONFIG_USBD_PRODUCT_NAME	"Zaurus"

	#undef CONFIG_USBD_NET_CDC
	#undef CONFIG_USBD_NET_MDLM
	#undef CONFIG_USBD_NET_SAFE

	#define CONFIG_USBD_NET_MDLM			1
	#define	CONFIG_USBD_MAC_AS_SERIAL_NUMBER	1
	
#endif


/*
 * Assigned vendor and serial/network product ids for Vercel UD1
 */
#if defined(CONFIG_SA1100_IDR)
	
	#warning CONFIGURING FOR Vercel UD1

	#define	CONFIG_USBD_MAC_AS_SERIAL_NUMBER	1
	
#endif

#ifdef CONFIG_SABINAL_DISCOVERY

	#warning CONFIGURING FOR DISCOVERY

	#undef CONFIG_USBD_VENDORID
	#define CONFIG_USBD_VENDORID		0x04dd
	#undef CONFIG_USBD_SERIAL_PRODUCT ID
	#define CONFIG_USBD_SERIAL_PRODUCT ID	0x8005
	#undef CONFIG_USBD_NET_PRODUCTID
	#define CONFIG_USBD_NET_PRODUCTID	0x8005

	#undef CONFIG_USBD_SELFPOWERED
	#undef	CONFIG_USBD_MANUFACTURER
	#undef CONFIG_USBD_PRODUCT_NAME

	#define CONFIG_USBD_SELFPOWERED		1
	#define	CONFIG_USBD_MANUFACTURER	"Sharp"
	#define CONFIG_USBD_PRODUCT_NAME	"SL-A300"

	#undef CONFIG_USBD_NET_CDC
	#undef CONFIG_USBD_NET_MDLM
	#undef CONFIG_USBD_NET_SAFE

	#define CONFIG_USBD_NET_MDLM			1
	#define	CONFIG_USBD_MAC_AS_SERIAL_NUMBER	1

#endif
