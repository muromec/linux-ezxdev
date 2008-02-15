/*
 *  adcm2650_hw.c
 *
 *  ADCM 2650 Camera Module driver.
 *
 *  Copyright (C) 2003, Intel Corporation
 *  Copyright (C) 2003, Montavista Software Inc.
 *
 *  Author: Intel Corporation Inc.
 *          MontaVista Software, Inc.
 *           source@mvista.com
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/types.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/dma.h>
#include <asm/irq.h>

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/wrapper.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include "adcm2650_hw.h"

#define I2C_SLAVE_ID 0x4E  /* 0100_111x */ /* The Phillips spec says it must be a value between 0001_000xB and 1110_111xB */

firm_update_t firm_update[] = {
{0x00fc, 0x0008},  // index 1
{0x00fe, 0x000c},  // index 2
{0x42cc, 0x0002},  // index 3
{0x42ce, 0xfd13},  // index 4
{0x4308, 0x0002},  // index 5
{0x430a, 0xfd2d},  // index 6
{0x430c, 0x0002},  // index 7
{0x430e, 0xfd00},  // index 8
{0x4400, 0x0000},  // index 9
{0x4402, 0x060e},  // index 10
{0x4404, 0x0003},  // index 11
{0x4406, 0x03b9},  // index 12
{0x4408, 0x000d},  // index 13
{0x440a, 0x3e80},  // index 14
{0x440c, 0x0000},  // index 15
{0x440e, 0x02ac},  // index 16
{0x4410, 0x0000},  // index 17
{0x4412, 0x5c04},  // index 18
{0x4414, 0x0000},  // index 19
{0x4416, 0x02a7},  // index 20
{0x4418, 0x0000},  // index 21
{0x441a, 0x093e},  // index 22
{0x441c, 0x0000},  // index 23
{0x441e, 0x0436},  // index 24
{0x4420, 0x0000},  // index 25
{0x4422, 0x5445},  // index 26
{0x4424, 0x0000},  // index 27
{0x4426, 0x042d},  // index 28
{0x4428, 0x0000},  // index 29
{0x442a, 0x041d},  // index 30
{0x442c, 0x0000},  // index 31
{0x442e, 0x026e},  // index 32
{0x4430, 0x0000},  // index 33
{0x4432, 0x01a6},  // index 34
{0x4434, 0x000d},  // index 35
{0x4436, 0x3300},  // index 36
{0x4438, 0x0000},  // index 37
{0x443a, 0x0175},  // index 38
{0x443c, 0x000e},  // index 39
{0x443e, 0xc350},  // index 40
{0x4440, 0x0007},  // index 41
{0x4442, 0x0681},  // index 42
{0x4444, 0x000e},  // index 43
{0x4446, 0x61a8},  // index 44
{0x4448, 0x0002},  // index 45
{0x444a, 0x0681},  // index 46
{0x444c, 0x0003},  // index 47
{0x444e, 0x01ba},  // index 48
{0x4450, 0x0003},  // index 49
{0x4452, 0x03b9},  // index 50
{0x4454, 0x0000},  // index 51
{0x4456, 0x5c04},  // index 52
{0x4458, 0x0000},  // index 53
{0x445a, 0x02bc},  // index 54
{0x445c, 0x000f},  // index 55
{0x445e, 0x010b},  // index 56
{0x4460, 0x0000},  // index 57
{0x4462, 0x0267},  // index 58
{0x4464, 0x0000},  // index 59
{0x4466, 0x01a7},  // index 60
{0x4468, 0x0000},  // index 61
{0x446a, 0x043f},  // index 62
{0x446c, 0x0000},  // index 63
{0x446e, 0x01fb},  // index 64
{0x4470, 0x0004},  // index 65
{0x4472, 0xfd1e},  // index 66
{0x4474, 0x0000},  // index 67
{0x4476, 0x0197},  // index 68
{0x4478, 0x0000},  // index 69
{0x447a, 0x5845},  // index 70
{0x447c, 0x000d},  // index 71
{0x447e, 0x3000},  // index 72
{0x4480, 0x0000},  // index 73
{0x4482, 0x02ae},  // index 74
{0x4484, 0x0000},  // index 75
{0x4486, 0x0196},  // index 76
{0x4488, 0x0000},  // index 77
{0x448a, 0x0177},  // index 78
{0x448c, 0x0007},  // index 79
{0x448e, 0xfd25},  // index 80
{0x4490, 0x0000},  // index 81
{0x4492, 0x0937},  // index 82
{0x4494, 0x0000},  // index 83
{0x4496, 0x5b30},  // index 84
{0x4498, 0x0000},  // index 85
{0x449a, 0x0416},  // index 86
{0x449c, 0x0000},  // index 87
{0x449e, 0x0436},  // index 88
{0x44a0, 0x0000},  // index 89
{0x44a2, 0x041f},  // index 90
{0x44a4, 0x0000},  // index 91
{0x44a6, 0x0417},  // index 92
{0x44a8, 0x0000},  // index 93
{0x44aa, 0x037e},  // index 94
{0x44ac, 0x0000},  // index 95
{0x44ae, 0x7b30},  // index 96
{0x44b0, 0x0000},  // index 97
{0x44b2, 0x01ff},  // index 98
{0x44b4, 0x0000},  // index 99
{0x44b6, 0x5c44},  // index 100
{0x44b8, 0x0000},  // index 101
{0x44ba, 0x5845},  // index 102
{0x44bc, 0x0000},  // index 103
{0x44be, 0x0416},  // index 104
{0x44c0, 0x0000},  // index 105
{0x44c2, 0x0277},  // index 106
{0x44c4, 0x0000},  // index 107
{0x44c6, 0x01a7},  // index 108
{0x44c8, 0x0000},  // index 109
{0x44ca, 0x043f},  // index 110
{0x44cc, 0x0000},  // index 111
{0x44ce, 0x0647},  // index 112
{0x44d0, 0x0004},  // index 113
{0x44d2, 0xfd36},  // index 114
{0x44d4, 0x0000},  // index 115
{0x44d6, 0x0427},  // index 116
{0x44d8, 0x0000},  // index 117
{0x44da, 0x5863},  // index 118
{0x44dc, 0x0000},  // index 119
{0x44de, 0x017e},  // index 120
{0x44e0, 0x0007},  // index 121
{0x44e2, 0xfd41},  // index 122
{0x44e4, 0x0000},  // index 123
{0x44e6, 0x5864},  // index 124
{0x44e8, 0x0000},  // index 125
{0x44ea, 0x017e},  // index 126
{0x44ec, 0x0007},  // index 127
{0x44ee, 0xfd41},  // index 128
{0x44f0, 0x0000},  // index 129
{0x44f2, 0x5865},  // index 130
{0x44f4, 0x0000},  // index 131
{0x44f6, 0x017e},  // index 132
{0x44f8, 0x0007},  // index 133
{0x44fa, 0xfd41},  // index 134
{0x44fc, 0x0000},  // index 135
{0x44fe, 0x0607},  // index 136
{0x4500, 0x0002},  // index 137
{0x4502, 0x05e7},  // index 138
{0x4504, 0x0000},  // index 139
{0x4506, 0x0627},  // index 140
{0x4508, 0x0000},  // index 141
{0x450a, 0x5c29},  // index 142
{0x450c, 0x000e},  // index 143
{0x450e, 0x0100},  // index 144
{0x4510, 0x0000},  // index 145
{0x4512, 0x00f7},  // index 146
{0x4514, 0x0000},  // index 147
{0x4516, 0x0407},  // index 148
{0x4518, 0x0000},  // index 149
{0x451a, 0x0077},  // index 150
{0x451c, 0x0002},  // index 151
{0x451e, 0x05ed},  // index 152
};

extern int i2c_adcm2650_init(void);
extern int i2c_adcm2650_cleanup(void);
extern int adcm2650_read(u16 addr, u16 *pvalue);
extern int adcm2650_write(u16 addr, u16 value);

void adcm2650_wait( int ms )
{
	mdelay( ms );
}


/*****************************************************************************
*									     *
*        I2C Management		 					     *
*									     *
*****************************************************************************/
void i2c_init(void)
{
	i2c_adcm2650_init();
	return;	
}

void i2c_deinit(void)
{
	i2c_adcm2650_cleanup();
	return;
}

int adcm2650_pipeline_read( u16 reg_addr, u16 * reg_value)
{
	adcm2650_read (reg_addr, reg_value);
	return ADCM_ERR_TIMEOUT;
}

int adcm2650_pipeline_write( u16 reg_addr, u16 reg_value)
{
        return adcm2650_write(reg_addr, reg_value);
}

int adcm2650_pipeline_read_rl( u16 reg_addr, u16 mask, u16 field )
{
	u16 reg_value;
	int retry = adcm2650__TIMEOUT;

	while( retry-- )
	{
		adcm2650_pipeline_read( reg_addr, &reg_value );
		if( (reg_value & mask) == field )
		{
			return ADCM_ERR_NONE;
		}   

		adcm2650_wait( 1 );
	}

	return ADCM_ERR_TIMEOUT;

}

int adcm2650_pipeline_write_wa( u16 reg_addr, u16 mask )
{
	u16 reg_value;

	adcm2650_pipeline_read( reg_addr, &reg_value );
	reg_value &= mask;
	adcm2650_pipeline_write( reg_addr, reg_value );
	
	return ADCM_ERR_NONE;
}


int adcm2650_pipeline_write_wo( u16 reg_addr, u16 field )
{
	u16 reg_value;

	adcm2650_pipeline_read( reg_addr, &reg_value );
	reg_value |= field;
	adcm2650_pipeline_write( reg_addr, reg_value );
	
	return ADCM_ERR_NONE;
}


/////////////////////////////////////////////////////////////////////////


int adcm2650_sensor_read_rs( u8 reg_addr, u8 * reg_value )
{
	u16 value;
	int   retry = adcm2650__TIMEOUT;

	// Loads SENSOR_ADDRESS (0x004a) with address 0x55 and register
	// address for a read operation.
	value = SENSOR_SLAVE_ADDR | (reg_addr << 8);
	adcm2650_pipeline_write( SENSOR_ADDRESS, value );
	
	// Loads SENSOR_CTRL (0x0050) with a read request
	value = SENSOR_CTRL_RW | SENSOR_CTRL_GO;
	adcm2650_pipeline_write( SENSOR_CTRL, value );
	
	// Checks if the request is complete and if data is available to read
	while( --retry )
	{
		// Reads the SENSOR_CTRL (0x0050) address for status
		value = 0xFF;
		adcm2650_pipeline_read( SENSOR_CTRL, &value );
	
		if (!(value & SENSOR_CTRL_GO))
		{
			adcm2650_pipeline_read( SENSOR_DATA_1, &value );
			*reg_value = value & 0xFF;
			return ADCM_ERR_NONE;
		}   
		adcm2650_wait( 1 );
	}
	return ADCM_ERR_TIMEOUT;
}

int adcm2650_sensor_write_ws( u8 reg_addr, u8 reg_value )
{
	u16 value;
	int   retry = adcm2650__TIMEOUT;

	// Loads SENSOR_ADDRESS (0x004a) with address 0x55 and register
	// a write operation.
	value = SENSOR_SLAVE_ADDR | (reg_addr << 8);
	adcm2650_pipeline_write( SENSOR_ADDRESS, reg_value );
	
	// Loads SENSOR_DATA0 (0x004c)
	value = reg_value;
	adcm2650_pipeline_write( SENSOR_DATA_1, reg_value );
	
	// Loads SENSOR_CTRL (0x0050) with a write request
	value = SENSOR_CTRL_GO;
	adcm2650_pipeline_write( SENSOR_CTRL, value | 0x1); // write one byte
	
	while( --retry )
	{
		value = 0xFF;
		adcm2650_pipeline_read( SENSOR_CTRL, &value );
	
		if (!(value & SENSOR_CTRL_GO))
		{
			return ADCM_ERR_NONE;
		}   
		adcm2650_wait( 1 );
	}
	return ADCM_ERR_TIMEOUT;
}


///////////////////////////////////////////////////////////////
//
//   Programming Guide Chapter 1: Basic Programming
//
///////////////////////////////////////////////////////////////

int adcm2650_power_on( u8 clk )
{
	// 1 Turn on M_VCC and wait for 20 ms.
	// 2 Turn on M_CLK using xx MHz and wait for 150 ms.
	// 3 Complete auto configuration 
	adcm2650_auto_config_complete();
	
	// 4 Load the firmware upgrade
	//adcm2650_FirmwareUpgrade();
	
	// 5 Program the master clock
	adcm2650_master_clock( clk );
	 
	// 6 Select the sensor voltage mode 
	//adcm2650_SensorVoltage( VOLTS_28 );
	
	// 7 Change factory overwrite 
	adcm2650_factory_overwrite();
	
	/* The following are optional steps that may be required to meet 
		specifications.
	// 8 Configure gamma correction 
	// 9 Configure color correction 
	// 10 Configure flare correction 
	// 11 Configure for JPEG quality for still frame mode 
	*/

	return ADCM_ERR_NONE;
}

int adcm2650_power_off(void)
{
	// 1 Turn off M_CLK and wait for 30 ms.
	// 2 Turn off M_VCC and wait for 30 ms.
	
	return ADCM_ERR_NONE;   
}

int adcm2650_change_viewfinder_mode(adcm_window_size * input_win, adcm_window_size *vf_output_win, adcm_window_size *sf_output_win )
{
	// 1 Turn viewfinder off 
	adcm2650_viewfinder_off();

	// 2 Set the input size for viewfinder and still frame modes 
	adcm2650_viewfinder_input_size( input_win );
	adcm2650_stillframe_input_size( input_win );
	
	// 3 Set the output size for viewfinder mode 
	adcm2650_viewfinder_output_size( vf_output_win );
	
	// 4 Set the output size for still frame mode 
	adcm2650_stillframe_output_size( sf_output_win );
	
	// 5 Turn the viewfinder back on 
	adcm2650_viewfinder_on();
	
	return ADCM_ERR_NONE;   
}


int adcm2650_jpeg_slow_still_frame(void)
{
	return 0;
}

int adcm2650_jpeg_fast_still_frame(void)
{
	return 0;
}

int adcm2650_pll(void)
{
	return 0;
}



/////////////////////////////////////////////////////////////////////////////////////
//   
//  Programming Guide Chapter 2: Configuration Methods 
//
/////////////////////////////////////////////////////////////////////////////////////

int adcm2650_auto_config_complete(void)
{
	int ret;
	ret = RL( CMD_2, CMD_2_ACS|CMD_2_AVT|CMD_2_AST, 0x00 );
	return ret;
}

int adcm2650_firmware_upgrade(void)
{
	int i;
	firm_update_t *pfu;

	pfu = firm_update;
	for( i = 0; i < 152; i++ )
	{
		W(pfu->addr, pfu->value);
		pfu++;  
	}

	return ADCM_ERR_NONE;
}


int adcm2650_version_revision(u16 * cm_revision, u8 *sensor_revision)
{
	// Camera module version is 0x0600
	R( REV, cm_revision );

	// Image sensor version is 0x60
	RS( IDENT, sensor_revision );

	return ADCM_ERR_NONE;
}

int adcm2650_viewfinder_on(void)
{
	// Enables the viewfinder bits in VE
	WO( CMD_1, CMD_1_VE );

	// Once the viewfinder is turned on, wait until the camera
	// module is ready to produce quality images.
	RL( AF_STATUS, AF_STATUS_CC, AF_STATUS_CC );
	
	return ADCM_ERR_NONE;
}


int adcm2650_viewfinder_off(void)
{
	// Turns the viewfinder off
	WA( CMD_1, ~CMD_1_VE );

	// Resets the converged bit
	WA( AF_STATUS, ~AF_STATUS_CC );
	
	return ADCM_ERR_NONE;
}


int adcm2650_power_low(void)
{   
	// Enables low power mode bit LPE
	WO( CMD_1, CMD_1_LPE );
	
	return ADCM_ERR_NONE;
}

int adcm2650_power_normal(void)
{
	// Returns to normal power mode
	WA( CMD_1, ~CMD_1_LPE );

	return ADCM_ERR_NONE;
}


int adcm2650_suspend(void)
{
	// 1 Stops viewfinder mode 
	W( CMD_1, 0x0000 );

	// 2 turn off M_CLK Turns off the clock
	
	// 3 for 150 ms
	WAIT( 150 );
	
	return ADCM_ERR_NONE;
}


int adcm2650_wakeup(void)
{
	// 1 Turns on the clock M_CLK
	
	// 2 Waits for 150 ms
	WAIT( 150 );

	// 3 Turns on viewfinder mode
	W( CMD_1, 0x0001 );
	
	return ADCM_ERR_NONE;
}


int adcm2650_abort_image(void)
{
	// 1 Aborts the progressive viewfinder image bit IA
	WO( CMD_2, CMD_2_IA );

	// 2 Waits until the IA bit is auto cleared
	RL( CMD_2, CMD_2_IA, 0x0000 );
	
	return ADCM_ERR_NONE;
}


int adcm2650_snapshot_trigger(void)
{
	// Requests a capture snapshot
	WO( CMD_2, CMD_2_SNAP );
	return ADCM_ERR_NONE;
}

int adcm2650_snapshot_complete(void)
{
	// Waits until the snapshot is complete
	RL( CMD_2, CMD_2_SNAP, 0x0000 );
	return ADCM_ERR_NONE;
}

int adcm2650_snapshot_status(u16 *status)
{
	R( CMD_2, status );
	return ADCM_ERR_NONE;
}

int adcm2650_image_flip_v(u16 mode)
{
	if( mode == VIEWFINDER_MODE )
	{
		WO( VIDEO_CONFIG, VIDEO_CONFIG_V_MIRROR );
	}
	else if( mode == STILLFRAME_MODE )
	{
		WO( STILL_CONFIG, STILL_CONFIG_V_MIRROR );
	}
	else
	{
		return ADCM_ERR_PARAMETER;
	}

	return ADCM_ERR_NONE;
}


int adcm2650_image_flip_h(u16 mode)
{
	if( mode == VIEWFINDER_MODE )
	{
		WO( VIDEO_CONFIG, VIDEO_CONFIG_H_MIRROR );
	}
	else if( mode == STILLFRAME_MODE )
	{
		WO( STILL_CONFIG, STILL_CONFIG_H_MIRROR );
	}
	else
	{
		return ADCM_ERR_PARAMETER;
	}

	return ADCM_ERR_NONE;
}


int adcm2650_image_flip_vh(u16 mode)
{
	if( mode == VIEWFINDER_MODE )
	{
		WO( VIDEO_CONFIG, VIDEO_CONFIG_H_MIRROR|VIDEO_CONFIG_V_MIRROR );
	}
	else if( mode == STILLFRAME_MODE )
	{
		WO( STILL_CONFIG, STILL_CONFIG_H_MIRROR|STILL_CONFIG_V_MIRROR );
	}
	else
	{
		return ADCM_ERR_PARAMETER;
	}
	
	return ADCM_ERR_NONE;
}


int adcm2650_pll_active(void)
{
	// Starts PLL request
	WO( CMD_2, CMD_2_PLL_ON | CMD_2_UCGS );
	
	// Checks PLL request is complete
	RL( CMD_2, CMD_2_PLL_ON | CMD_2_UCGS, 0x0 );
	
	return ADCM_ERR_NONE;   
}
	

int adcm2650_pll_deactive(void)
{
	// Disables PLL
	WO( CMD_2, CMD_2_PLL_OFF );
	
	// Checks PLL disable request is complete
	RL( CMD_2, CMD_2_PLL_OFF, 0x0 );
	
	return ADCM_ERR_NONE;
}


int adcm2650_pll_configuration(void)
{
	return ADCM_ERR_NONE;
}

int adcm2650_pll_synchronized(void)
{
	// Waits until normal operation
	RL( S_PLL_CTRL_0, S_PLL_CTRL_0_S_PLL_SE, 0x00 );

	return ADCM_ERR_NONE;
}

int adcm2650_factory_overwrite(void)
{
	// Disable the packet code
	WO( BYPASS_CTRL, BYPASS_CTRL_PCD );

	// Free running VCLK
	W( OUTPUT_CTRL, 0x0 ); 
	
	return ADCM_ERR_NONE;
}

int adcm2650_master_clock( u8 clk )
{
	W( MASTER_CLK_FREQ, ((u16)clk) << 8 );
	return ADCM_ERR_NONE;
}


int adcm2650_sensor_voltage( u8 vltg )
{
	// Changes the sensor voltage to 2.8 //????
	WS( 0x001c, 0xa0 );
	return ADCM_ERR_NONE;   
}

int adcm2650_viewfinder_input_size( adcm_window_size * window )
{
	// Input size width - viewfinder mode
	W( SZR_IN_W_VID, window->width );

	// Input size height - viewfinder mode
	W( SZR_IN_H_VID, window->height );
	
	return ADCM_ERR_NONE;
}

int adcm2650_stillframe_input_size( adcm_window_size * window )
{
	// Input size width - still frame mode
	W( SZR_IN_W_STL, window->width );

	// Input size height - still frame mode
	W( SZR_IN_H_STL, window->height );

	return ADCM_ERR_NONE;
}


int adcm2650_viewfinder_output_size(adcm_window_size * window)
{
	// Output size width
	W( SZR_OUT_W_VID, window->width );
	
	// Output size height
	W( SZR_OUT_H_VID, window->height);
	
	return ADCM_ERR_NONE;
}

int adcm2650_stillframe_output_size(adcm_window_size * window)
{
	// Output size width
	W( SZR_OUT_W_STL, window->width );

	// Output size height
	W( SZR_OUT_H_STL, window->height);

	return ADCM_ERR_NONE;
}

int adcm2650_flicker_rate(u16 rate)
{
	if( rate == AEF_50HZ )
	{
		// Clears the AE_F bit and sets it to 50 Hz
		WA( 0x2022, 0xfffd );
	}
	else if( rate == AEF_60HZ )
	{
		// Sets the AE_F bit to 60 Hz
		WO( 0x2022, 0x0002 );
	}
	else 
	{
		return ADCM_ERR_PARAMETER;
	}

	return ADCM_ERR_NONE;
}


int adcm2650_low_light(void)
{
	// Sets the bit to 60 Hz; sample duration is 25
	WS( 0x0e, 0x19);
	return ADCM_ERR_NONE;   
}

int adcm2650_normal_light(void)
{
	// Sets the bit back to normal mode; sample duration is 4
	WS( 0x0e, 0x04 );
	return ADCM_ERR_NONE;
}

int adcm2650_discard_viewfinder_image_data(void)
{
	// Discards the current image viewfinder data
	W( 0x0004, 0x1000 );

	// Waits until the image abort is complete
	RL(0x0004, 0x1000, 0x0000 );
	
	return ADCM_ERR_NONE;   
}

int adcm2650_stillframe_cfg_output(u16 format)
{
	//Clear bits 8..11
	WA( STILL_CONFIG, ~STILL_CONFIG_O_FORMAT_MASK );

	// Where format is bits 8..11
	WO( STILL_CONFIG, format << STILL_CONFIG_O_FORMAT_SHIFT ); 
	return ADCM_ERR_NONE;
}

int adcm2650_viewfinder_cfg_output(u16 format)
{
	//Clear bits 8..11
	WA( VIDEO_CONFIG, ~VIDEO_CONFIG_O_FORMAT_MASK );

	// Where format is bits 8..11
	WO( VIDEO_CONFIG, format << VIDEO_CONFIG_O_FORMAT_SHIFT ); 
	return ADCM_ERR_NONE;
}


int adcm2650_switch_to_jpeg(u16 mode)
{
	if( mode == STILLFRAME_MODE )
	{
		// Switches to JPEG output mode
		WO( STILL_CONFIG, STILL_CONFIG_JPEG_MASK );
	}
	else if( mode == VIEWFINDER_MODE )
	{
		//Turns the viewfinder off
		//  adcm2650_ViewfinderOff();
	
		//Changes still frame output mode to JPEG
		WO( VIDEO_CONFIG, VIDEO_CONFIG_JPEG_MASK );
	
		//Turns the viewfinder back on
		//  adcm2650_ViewfinderOn();
	}
	else
	{
		return ADCM_ERR_PARAMETER;
	}

	return ADCM_ERR_NONE;
}

int adcm2650_switch_to_normal(u16 mode)
{
	if( mode == STILLFRAME_MODE )
	{
		// Switches to Normal output mode
		WA( STILL_CONFIG, ~STILL_CONFIG_JPEG_MASK );
	}
	else if( mode == VIEWFINDER_MODE )
	{
		//Turns the viewfinder off
		//  adcm2650_ViewfinderOff();
	
		//Changes still frame output mode to NORMAL
		WA( VIDEO_CONFIG, ~VIDEO_CONFIG_JPEG_MASK );
	
		//Turns the viewfinder back on
		//  adcm2650_ViewfinderOn();
	}
	else
	{
		return ADCM_ERR_PARAMETER;
	}

	return ADCM_ERR_NONE;
}

int adcm2650_gamma_correction(void)
{
	return ADCM_ERR_NONE;
}

int adcm2650_get_output_frame_rate(u16 * pfps)
{
	u16 fps;

	// Resets the frame counter
	W( OUT_FRAME_CNT, 0x0000 );

	// Waits for one second
	WAIT( 1000 );
	
	// Number of frames outputting the lower 8 bits
	R( OUT_FRAME_CNT, &fps ); 

	*pfps = fps & 0xFF;

	return ADCM_ERR_NONE;
}

int adcm2650_detect_camera_mode(u16 *mode)
{
	u16 status;
	// Note!! Document is wrong. 0x0076 shall be 0x0074 instead.
	// Reads the Status_flag
	R( STATUS_FLAGS, &status );
	
	if( (status & 0x08) == 0x08 )   
	{
		*mode = STILLFRAME_MODE;
	}
	else
	{
		*mode = VIEWFINDER_MODE;
	}

	return ADCM_ERR_NONE;   
}

int adcm2650_wait_till_in_stillframe_mode(void)
{
	// Note!! Document is wrong. 0x0076 shall be 0x0074 instead.
	RL( STATUS_FLAGS, 0x0008, 0x0008 ); 
	return ADCM_ERR_NONE;   
}

int adcm2650_camera_ready(void)
{
	RL(AF_STATUS, 0x0008, 0x0008 );
	return ADCM_ERR_NONE;   
}

int adcm2650_halt_video_output(void)
{
	// Halts the viewfinder output  
	W( UART_CREDITS, 0x0000 );
	return ADCM_ERR_NONE;
}

int adcm2650_get_single_image(void)
{
	//Outputs a single frame
	W( UART_CREDITS, 0x001 );   
	return ADCM_ERR_NONE;
}

int adcm2650_resume_to_full_output_mode(void)
{
	// Output still frames continuously
	W( UART_CREDITS, 0x0100 );
	return ADCM_ERR_NONE;
}


int adcm2650_manual_q_table(void)
{
	return ADCM_ERR_NONE;
}

void adcm2650_dump_pipeline_register(u16 startRegAddr, u16 endRegAddr, u16* buffer)
{
	u16 addr;
	for(addr=startRegAddr; addr<=endRegAddr; addr+=2)
	adcm2650_pipeline_read( addr, buffer++ );
}

void adcm2650_dump_sensor_register(u16 startRegAddr, u16 endRegAddr, u8* buffer)
{
	u16 addr;
	for(addr=startRegAddr; addr<=endRegAddr; addr++)
	adcm2650_sensor_read_rs((u8) addr, buffer++ );
}
