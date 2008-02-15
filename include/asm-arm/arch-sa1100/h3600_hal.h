/*
* Abstraction interface for microcontroller connection to rest of system
*
* Copyright 2000,1 Compaq Computer Corporation.
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
*
*/

#ifndef __H3600_HAL_H
#define __H3600_HAL_H

#include <linux/h3600_ts.h>

enum h3600_asset_type {
	ASSET_TCHAR = 0,
	ASSET_SHORT,
	ASSET_LONG
};

#define TTYPE(_type)           (((unsigned int)_type) << 8)
#define TCHAR(_len)            (TTYPE(ASSET_TCHAR) | (_len))
#define TSHORT                 TTYPE(ASSET_SHORT)
#define TLONG                  TTYPE(ASSET_LONG)
#define ASSET(_type,_num)      ((((unsigned int)_type)<<16) | (_num))

#define ASSET_HM_VERSION        ASSET( TCHAR(10), 0 )   /* 1.1, 1.2 */
#define ASSET_SERIAL_NUMBER     ASSET( TCHAR(40), 1 )   /* Unique iPAQ serial number */
#define ASSET_MODULE_ID         ASSET( TCHAR(20), 2 )   /* E.g., "iPAQ 3700" */    
#define ASSET_PRODUCT_REVISION  ASSET( TCHAR(10), 3 )   /* 1.0, 2.0 */
#define ASSET_PRODUCT_ID        ASSET( TSHORT,    4 )   /* 2 = Palm-sized computer */
#define ASSET_FRAME_RATE        ASSET( TSHORT,    5 )
#define ASSET_PAGE_MODE         ASSET( TSHORT,    6 )   /* 0 = Flash memory */
#define ASSET_COUNTRY_ID        ASSET( TSHORT,    7 )   /* 0 = USA */
#define ASSET_IS_COLOR_DISPLAY  ASSET( TSHORT,    8 )   /* Boolean, 1 = yes */
#define ASSET_ROM_SIZE          ASSET( TSHORT,    9 )   /* 16, 32 */
#define ASSET_RAM_SIZE          ASSET( TSHORT,   10 )   /* 32768 */
#define ASSET_HORIZONTAL_PIXELS ASSET( TSHORT,   11 )   /* 240 */
#define ASSET_VERTICAL_PIXELS   ASSET( TSHORT,   12 )   /* 320 */

#define ASSET_TYPE(_asset)       (((_asset)&0xff000000)>>24)
#define ASSET_TCHAR_LEN(_asset)  (((_asset)&0x00ff0000)>>16)
#define ASSET_NUMBER(_asset)     ((_asset)&0x0000ffff)

#define MAX_TCHAR_LEN 40

struct h3600_asset {
	unsigned int type;
	union {
		unsigned char  tchar[ MAX_TCHAR_LEN ];
		unsigned short vshort;
		unsigned long  vlong;
	} a;
};

/* Interface to the hardware-type specific functions */
struct h3600_hal_ops {
	/* Functions provided by the underlying hardware */
	int (*get_version)( struct h3600_ts_version * );
	int (*eeprom_read)( unsigned short address, unsigned char *data, unsigned short len );
	int (*eeprom_write)( unsigned short address, unsigned char *data, unsigned short len );
	int (*get_thermal_sensor)( unsigned short * );
	int (*set_notify_led)( unsigned char mode, unsigned char duration, 
			       unsigned char ontime, unsigned char offtime );
	int (*read_light_sensor)( unsigned char *result );
	int (*get_battery)( struct h3600_battery * );
	int (*spi_read)( unsigned short address, unsigned char *data, unsigned short len );
	int (*spi_write)( unsigned short address, unsigned char *data, unsigned short len );
	int (*codec_control)( unsigned char, unsigned char );
	int (*get_option_detect)( int *result );
	int (*audio_clock)( long samplerate );
	int (*audio_power)( long samplerate );
	int (*audio_mute)( int mute );
	int (*asset_read)( struct h3600_asset *asset );
	int (*set_ebat)( void );

	/* Functions indirectly provided by the underlying hardware */
	int (*backlight_control)( enum flite_pwr power, unsigned char level );
	int (*contrast_control)( unsigned char level );

        /* for module use counting */ 
        struct module *owner;
};

/* Used by the device-specific hardware module to register itself */
extern int  h3600_hal_register_interface( struct h3600_hal_ops *ops );
extern void h3600_hal_unregister_interface( struct h3600_hal_ops *ops );

/* 
 * Calls into HAL from the device-specific hardware module
 * These run at interrupt time 
 */
extern void h3600_hal_keypress( unsigned char key );
extern void h3600_hal_touchpanel( unsigned short x, unsigned short y, int down );
extern void h3600_hal_option_detect( int present );

/* Callbacks registered by device drivers */
struct h3600_driver_ops {
	void (*keypress)( unsigned char key );
	void (*touchpanel)( unsigned short x, unsigned short y, int down );
	void (*option_detect)( int present );
};

extern int  h3600_hal_register_driver( struct h3600_driver_ops * );
extern void h3600_hal_unregister_driver( struct h3600_driver_ops * );


/* Calls into HAL from device drivers and other kernel modules */
extern void h3600_get_flite( struct h3600_ts_backlight *bl );
extern void h3600_get_contrast( unsigned char *contrast );
extern int  h3600_set_flite( enum flite_pwr pwr, unsigned char brightness );
extern int  h3600_set_contrast( unsigned char contrast );
extern int  h3600_toggle_frontlight( void );

extern int h3600_apm_get_power_status(unsigned char *ac_line_status, unsigned char *battery_status, 
				      unsigned char *battery_flag, unsigned char *battery_percentage, 
				      unsigned short *battery_life);

extern struct h3600_hal_ops *h3600_hal_ops;

/* Do not use this macro in driver files - instead, use the inline functions defined below */
#define CALL_HAL( f, args... ) \
        { int __result = -EIO;                             \
          if ( h3600_hal_ops && h3600_hal_ops->f ) {       \
                __MOD_INC_USE_COUNT(h3600_hal_ops->owner); \
                __result = h3600_hal_ops->f(args);         \
                __MOD_DEC_USE_COUNT(h3600_hal_ops->owner); \
          }                                                \
          return __result; }

#define HFUNC  static __inline__ int

/* The eeprom_read/write address + len has a maximum value of 512.  Both must be even numbers */
HFUNC h3600_eeprom_read( u16 addr, u8 *data, u16 len )  CALL_HAL(eeprom_read,addr,data,len)
HFUNC h3600_eeprom_write( u16 addr, u8 *data, u16 len)  CALL_HAL(eeprom_write,addr,data,len)
HFUNC h3600_spi_read( u8 addr, u8 *data, u16 len) 	CALL_HAL(spi_read,addr,data,len)
HFUNC h3600_spi_write( u8 addr, u8 *data, u16 len) 	CALL_HAL(spi_write,addr,data,len)
HFUNC h3600_get_version( struct h3600_ts_version *v )   CALL_HAL(get_version,v)
HFUNC h3600_get_thermal_sensor( u16 *thermal ) 	        CALL_HAL(get_thermal_sensor,thermal)
HFUNC h3600_set_led( u8 mode, u8 dur, u8 ont, u8 offt ) CALL_HAL(set_notify_led, mode, dur, ont, offt)
HFUNC h3600_get_light_sensor( u8 *result ) 	        CALL_HAL(read_light_sensor,result)
HFUNC h3600_get_battery( struct h3600_battery *bat )	CALL_HAL(get_battery,bat)
HFUNC h3600_get_option_detect( int *result)             CALL_HAL(get_option_detect,result)
HFUNC h3600_audio_clock( long samplerate )              CALL_HAL(audio_clock,samplerate)
HFUNC h3600_audio_power( long samplerate )              CALL_HAL(audio_power,samplerate)
HFUNC h3600_audio_mute( int mute )                      CALL_HAL(audio_mute,mute)
HFUNC h3600_asset_read( struct h3600_asset *asset )     CALL_HAL(asset_read,asset)
HFUNC h3600_set_ebat( void )                            CALL_HAL(set_ebat)

/* Don't use these functions directly - rather, call {get,set}_{flite,contrast} */
	/* Functions indirectly provided by the underlying hardware */
HFUNC h3600_backlight_control( enum flite_pwr p, u8 v ) CALL_HAL(backlight_control,p,v)
HFUNC h3600_contrast_control( u8 level )                CALL_HAL(contrast_control,level)
#endif
