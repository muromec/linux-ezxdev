/*
*
* Driver for H3600 Extension Packs
*
* Copyright 2000 Compaq Computer Corporation.
*
* Use consistent with the GNU GPL is permitted,
* provided that this copyright notice is
* preserved in its entirety in all copies and derived works.
*
* COMPAQ COMPUTER CORPORATION MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
* AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
* FITNESS FOR ANY PARTICULAR PURPOSE.
*
* Author: Jamey Hicks.
*
*/
#ifndef _INCLUDE_H3600_SLEEVE_H_
#define _INCLUDE_H3600_SLEEVE_H_

#ifdef CONFIG_DRIVERFS_FS
#include <linux/device.h>
#endif

enum sleeve_vendor {
        SLEEVE_NO_VENDOR     = 0,
        COMPAQ_VENDOR_ID     = 0x1125,
        TALON_VENDOR         = 0x0100,
        BIOCENTRIC_SOLUTIONS = 0x0101,
        HANVIT_INFO_TECH     = 0x0102,
        SYMBOL_VENDOR        = 0x0103,
        OMRON_CORPORATION    = 0x0104,
        HAGIWARA_SYSCOM_CO   = 0x0105,
        NEXIAN_VENDOR        = 0x010a,
        SLEEVE_ANY_ID = -1,
};

enum sleeve_device {
        SLEEVE_NO_DEVICE           = 0,

        /* COMPAQ_VENDOR_ID sleeve devices */
        SINGLE_COMPACTFLASH_SLEEVE = 0xCF11,
        SINGLE_PCMCIA_SLEEVE       = 0xD7C3,
        DUAL_PCMCIA_SLEEVE         = 0x0001,
        EXTENDED_BATTERY_SLEEVE    = 0x0002,
        SINGLE_CF_PLUS_SLEEVE      = 0x0003,
        SINGLE_PCMCIA_PLUS_SLEEVE  = 0x0004,
        GPRS_EXPANSION_PACK        = 0x0010,
        BLUETOOTH_EXPANSION_PACK   = 0x0011,
        MERCURY_BACKPAQ            = 0x0100,

        /* TALON_VENDOR sleeve devices */
        NAVMAN_GPS_SLEEVE          = 0x0001,
	NAVMAN_GPS_SLEEVE_ALT      = 0x0010,

        /* Nexian sleeve devices */
        NEXIAN_DUAL_CF_SLEEVE     = 0x0001,
        NEXIAN_CAMERA_CF_SLEEVE   = 0x0003,

        /* Symbol sleeve devices */
        SYMBOL_WLAN_SCANNER_SLEEVE = 0x0011,
};

#define H3600_SLEEVE_ID(vendor,device) ((vendor)<<16 | (device))

struct sleeve_driver;

/**
 * struct sleeve_dev
 * @driver: which driver allocated this device
 * @procent: device entry in /proc/bus/sleeve
 * @vendor:  vendor id copied from eeprom
 * @device:  device id copied from eeprom
 * @driver_data: private data for the driver
 * @dev: 
 */ 
struct sleeve_dev {
        struct sleeve_driver  *driver;	        /* which driver has allocated this device */
        struct proc_dir_entry *procent;	        /* device entry in /proc/bus/sleeve */
        unsigned short	       vendor;          /* Copied from EEPROM */
        unsigned short	       device;  
        void		      *driver_data;	/* data private to the driver */
#ifdef CONFIG_DRIVERFS_FS
        struct device         dev;
#endif
};

struct sleeve_device_id {
        unsigned int vendor, device;		/* Vendor and device ID or SLEEVE_ANY_ID */
};

#define SLEEVE_HAS_CF_MASK               0x00000003  /* 0 to 3 */
#define SLEEVE_HAS_CF(x)                        (x)
#define SLEEVE_GET_CF(x)                 ((x)&SLEEVE_HAS_CF_MASK)
#define SLEEVE_HAS_PCMCIA_MASK           0x0000000c
#define SLEEVE_HAS_PCMCIA(x)               ((x)<<2)
#define SLEEVE_GET_PCMCIA(x)             (((x)&SLEEVE_HAS_PCMCIA_MASK)>>2)

#define SLEEVE_HAS_BLUETOOTH             0x00000010
#define SLEEVE_HAS_GPS                   0x00000020
#define SLEEVE_HAS_GPRS                  0x00000040
#define SLEEVE_HAS_CAMERA                0x00000080
#define SLEEVE_HAS_ACCELEROMETER         0x00000100
#define SLEEVE_HAS_FLASH                 0x00000200
#define SLEEVE_HAS_BARCODE_READER        0x00000400
#define SLEEVE_HAS_802_11B               0x00000800
#define SLEEVE_HAS_EBAT                  0x00001000

#define SLEEVE_HAS_FIXED_BATTERY         0x10000000
#define SLEEVE_HAS_REMOVABLE_BATTERY     0x20000000

struct h3600_battery;

/**
 * struct sleeve_driver: driver for iPAQ extension pack
 * @name: name of device type
 * @id_table: table of &struct_sleeve_device_id identifying matching sleeves by vendor,device
 * @features: SLEEVE_HAS flags 
 * @driver: used by driverfs
 * @probe: called when new sleeve inserted
 * @remove: called when sleeve removed
 * @suspend: called before suspending iPAQ
 * @resume: called while resuming iPAQ
 *
 */
struct sleeve_driver {
        struct list_head               node;
        char                          *name;
        const struct sleeve_device_id *id_table;   /* NULL if wants all devices */
	unsigned int                   features;   /* SLEEVE_HAS flags */
#ifdef CONFIG_DRIVERFS_FS
        struct device_driver          *driver;
#endif
        int  (*probe)(struct sleeve_dev *dev, const struct sleeve_device_id *did);  /* New device inserted */
        void (*remove)(struct sleeve_dev *dev);
        void (*suspend)(struct sleeve_dev *dev);
        void (*resume)(struct sleeve_dev *dev);
};

extern struct iobus h3600_sleeve_iobus;

extern int  h3600_current_sleeve( void );   /* returns H3600_SLEEVE_ID(vendor,dev) */

/**
 * h3600_sleeve_register_driver: Registers a driver for an iPAQ extension pack.
 * @drv: the sleeve driver
 */
extern int  h3600_sleeve_register_driver(struct sleeve_driver *drv);

/**
 * h3600_sleeve_unregister_driver: Unregisters a driver for an iPAQ extension pack.
 * @drv: the sleeve driver
 */
extern void h3600_sleeve_unregister_driver(struct sleeve_driver *drv);

#endif /* _INCLUDE_H3600_SLEEVE_H_ */
