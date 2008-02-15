/*
* Driver interface to the ASIC Complasion chip on the iPAQ H3800
*
* Copyright 2001 Compaq Computer Corporation.
*
* Use consistent with the GNU GPL is permitted,
* provided that this copyright notice is
* preserved in its entirety in all copies and derived works.
*
* COMPAQ COMPUTER CORPORATION MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
* AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
* FITNESS FOR ANY PARTICULAR PURPOSE.
*
* Author:  Andrew Christian
*          <Andrew.Christian@compaq.com>
*          October 2001
*
* Restrutured June 2002
*/

#ifndef H3600_ASIC_IO_H
#define H3600_ASIC_IO_H

struct pt_regs;
struct owm_net_address;

void h3600_asic_power_isr(int irq, void *dev_id, struct pt_regs *regs);
void h3600_asic_ac_in_isr(int irq, void *dev_id, struct pt_regs *regs);
void h3600_asic_sleeve_isr(int irq, void *dev_id, struct pt_regs *regs);

enum owm_state {
	OWM_STATE_IDLE = 0,
	OWM_STATE_RESET,
	OWM_STATE_WRITE,
	OWM_STATE_READ,
	OWM_STATE_DONE
};

extern unsigned char *owm_state_names[];

int h3600_asic_owm_read_bytes( struct owm_net_address *net, unsigned char address, 
			       unsigned char *data, unsigned short len );
int h3600_asic_spi_read_pcmcia_battery( unsigned char *chem, unsigned char *percent, unsigned char *flag );
int h3600_asic_adc_read_channel( int mux );

enum charging_state {
	CHARGING_STATE_INIT = 0,
	CHARGING_STATE_NO_AC,      /* No AC */
	CHARGING_STATE_ACTIVE,     /* Actively charging the battery */
	CHARGING_STATE_FULL        /* The battery is fully charged */
};

int  h3600_asic_battery_get_state(void);
void h3600_asic_battery_set_led( enum charging_state state );
int  h3600_asic_battery_read( struct h3600_battery *query );
int  h3600_asic_spi_read(unsigned short address, unsigned char *data, unsigned short len);

int h3600_asic_audio_clock( long samplerate );
int h3600_asic_audio_power( long samplerate );
int h3600_asic_audio_mute( int mute );
int h3600_asic_backlight_control( enum flite_pwr power, unsigned char level );
int h3600_asic_asset_read( struct h3600_asset *asset );
int h3600_asic_spi_set_ebat( void );

/* common initialization/cleanup functions */

int  h3600_asic_adc_init(void);
void h3600_asic_adc_cleanup(void);
int  h3600_asic_adc_suspend(void);
void h3600_asic_adc_resume(void);

int  h3600_asic_key_init(void);
void h3600_asic_key_cleanup(void);
int  h3600_asic_key_suspend(void);
void h3600_asic_key_resume(void);

int  h3600_asic_spi_init(void);
void h3600_asic_spi_cleanup(void);
int  h3600_asic_spi_suspend(void);
void h3600_asic_spi_resume (void);

int  h3600_asic_backlight_init(void);   
void h3600_asic_backlight_cleanup(void);

int  h3600_asic_touchscreen_init(void); 
void h3600_asic_touchscreen_cleanup(void);
int  h3600_asic_touchscreen_suspend(void); 
void h3600_asic_touchscreen_resume (void);

int  h3600_asic_owm_init(void); 
void h3600_asic_owm_cleanup(void);
int  h3600_asic_owm_suspend(void); 
void h3600_asic_owm_resume (void);

int  h3600_asic_battery_init(void); 
void h3600_asic_battery_cleanup(void);
int  h3600_asic_battery_suspend(void); 
void h3600_asic_battery_resume (void);

int  h3600_asic_audio_init(void); 
void h3600_asic_audio_cleanup(void);
int  h3600_asic_audio_suspend(void); 
void h3600_asic_audio_resume  (void);

int  h3600_asic_asset_init(void);   
void h3600_asic_asset_cleanup(void);

int  h3600_asic_bluetooth_init(void);
void h3600_asic_bluetooth_cleanup(void);
int  h3600_asic_bluetooth_suspend(void);
void h3600_asic_bluetooth_resume(void);



#endif /* H3600_ASIC_IO_H */
