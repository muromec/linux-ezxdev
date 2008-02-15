/*
 *  adcm2650_hw.h
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

#ifndef _PXA_ADCM_2650_HW_H__
#define _PXA_ADCM_2650_HW_H__


/***********************************************************************
 *
 * Constants & Structures
 *
 ***********************************************************************/
typedef struct {
    u16 addr;
    u16 value;
} firm_update_t;

typedef struct {
    u16 width;
    u16 height;
} adcm_window_size;
//ADCM_WINDOWSIZE;  

// I2C address
#define PIPE_SLAVE_ADDR     0x0052
#define SENSOR_SLAVE_ADDR   0x0055

// Revision constants
#define PIPE_REV            0x0600
#define SENSOR_REV          0x60

// Calculating the Module Block Number
#define BLOCK(a)            (u8)((a) >> 7)        // register's module block address.
#define OFFSET(a)           (u8)((a) & 0x7F ) // register's offset to this block.

// Return codes
#define ADCM_ERR_NONE       0x00
#define ADCM_ERR_TIMEOUT    -1
#define ADCM_ERR_PARAMETER  -2  

// Auto Exposure Frequency
#define AEF_50HZ    0x20
#define AEF_60HZ    0x40

// Non JEPG Output Format
#define O_FORMAT_888RGB         0   //0b0000    // 888 RGB (1 pixel in 3 bytes )
#define O_FORMAT_666_A_RGB      1   //0b0001    // 666 A RGB (tight pack, 4 pixels in 9 bytes)
#define O_FORMAT_666_B_RGB      2   //0b0010    // 666 B RGB (loose pack, 1 pixel in 3 bytes,left or right justified)
#define O_FORMAT_565_RGB        3   //0b0011    // 565 RGB (1 pixel in 2 bytes)
#define O_FORMAT_444_A_RGB      4   //0b0100    // 444 A RGB (tight pack, 2 pixels per 3 bytes, RG BR GB)
#define O_FORMAT_444_B_RGB      5   //0b0101    // 444 B RGB (loose pack, 1 pixel per 2 bytes,RG B0 or 0R GB)
#define O_FORMAT_444_C_RGV      6   //0b0110    // 444 C RGB (sparse pack, 1 pixel per three bytes,R0 G0 B0 or 0R 0G 0B)
#define O_FORMAT_332_RGB        7   //0b0111    // 332 RGB (1 pixel in 1 byte)
#define O_FORMAT_422_A_YCbYCr   8   //0b1000    // 4:2:2 A YCbYCr (Y1 Cb12 Y2 CRL2 order)
#define O_FORMAT_422_B_YCbYCr   9   //0b1001    // 4:2:2 B YCbYCr (Cb12 Y1 CRL2 Y2 order)
#define O_FORMAT_422_C_YCbYCr   10  //0b1010    // 4:2:2 C YCbYCr (Y1 CRL2 Y2 Cb12 order)
#define O_FORMAT_422_D_YCbYCr   11  //0b1011    // 4:2:2 D YCbYCr (CRL2 Y1 Cb12 Y2 order)
#define O_FORMAT_444_YCbYCr     12  //0b1100    // 4:4:4 YCbCr (1 pixels per 3 bytes)
#define O_FORMAT_400_B_YCbYCr   13  //0b1101    // 4:0:0 YCbCr (Greyscale, 1 pixel per 1 byte)
#define O_FORMAT_RAWBPA         14  //0b1110    // RAWBPA (with AWB and BPA)
#define O_FORMAT_RAW            15  //0b1111    // RAW (without AWB and BPA)

// Camera Mode
#define VIEWFINDER_MODE     0x10
#define STILLFRAME_MODE     0x20

// Others
#define adcm2650__TIMEOUT    1000    // ms to timeout.
#define BLOCK_SWITCH_CMD    ((u8)0xFE)        // Block Switch Code: 0x7F, CMD = Code << 1
#define CLOCK_13M   13
#define VOLTS_28    0x28


/***********************************************************************
 *
 * Pipeline Register Offset
 *
 ***********************************************************************/
// Image Processor Register List
// Page 414
#define REV             0x0000
#define CMD_1           0x0002
#define CMD_2           0x0004
#define VIDEO_CONFIG    0x000c
#define STILL_CONFIG    0x000e
#define TM_SELECT       0x0010
#define BYPASS_CTRL     0x0012
#define PROCESS_CTRL    0x0014
#define OUTPUT_CTRL     0x0016
#define CCIR_TIMING_1   0x0018
#define Y_MAX_MIN       0x001a
#define CbCr_MAX_MIN    0x001c
#define BAD_FRAME_DIS   0x001e
#define UART_PCKT_SIZE  0x0020
#define UART_CRDT_ADD   0x0022
#define UART_CREDITS    0x0024
#define SZR_IN_W_VID    0x0026  
#define SZR_IN_H_VID    0x0028  
#define SZR_OUT_W_VID   0x002a  
#define SZR_OUT_H_VID   0x002c  
#define SZR_IN_W_STL    0x002e  
#define SZR_IN_H_STL    0x0030  
#define SZR_OUT_W_STL   0x0032  
#define SZR_OUT_H_STL   0x0034  
#define QTABLE_SELECT   0x0036

// Page 415
#define QTABLE_MAX_MIN  0x0038
#define UFL_LIMIT_VID   0x003a
#define UFL_TARGET_VID  0x003c
#define OFL_TARGET_VID  0x003e
#define OFL_LIMIT_VID   0x0040
#define UFL_LIMIT_STL   0x0042
#define UFL_TARGET_STL  0x0044
#define OFL_TARGET_STL  0x0046
#define OFL_LIMIT_STL   0x0048
#define SENSOR_ADDRESS  0x004a
#define SENSOR_DATA_1   0x004c
#define SENSOR_DATA_2   0x004e
#define SENSOR_CTRL     0x0050
#define PLL_CTRL_0      0x0052
#define PLL_CTRL_1      0x0054
#define PLL_CTRL_2      0x0056
#define DIVBY_UART      0x0058
#define EXT_DIVBY_VID   0x005a
#define EXT_DIVBY_STL   0x005c
#define STAT_CAP_CTRL   0x005e
#define STAT_MODE_CTRL  0x0060
#define GREEN1_SUM      0x0062
#define READ_SUM        0x0064
#define BLUE_SUM        0x0066
#define GREEN2_SUM      0x0068
#define NEG_CLIP_CNT    0x006a
#define POS_CLIP_CNT    0x006c
#define PEAK_DATA       0x006e
#define I_WIDTH         0x0070
#define I_HEIGHT        0x0072

// Page 416
#define STATUS_FLAGS    0x0074
#define STATUS_REG      0x0076
#define PLL_DIVBY_VID   0x0078
#define PLL_DIVBY_STL   0x007a
#define OUTPUT_CTRL_2   0x007c
#define CC_COEF_00      0x0080
#define CC_COEF_01      0x0082
#define CC_COEF_02      0x0084
#define CC_COEF_10      0x0086
#define CC_COEF_11      0x0088
#define CC_COEF_12      0x008a
#define CC_COEF_20      0x008c
#define CC_COEF_21      0x008e
#define CC_COEF_22      0x0090
#define CC_OS_0         0x0092
#define CC_OS_1         0x0094
#define CC_OS_2         0x0096
#define CSC_COEF_00V    0x0098
#define CSC_COEF_01V    0x009a
#define CSC_COEF_02V    0x009c
#define CSC_COEF_10V    0x009e
#define CSC_COEF_11V    0x00a0
#define CSC_COEF_12V    0x00a2
#define CSC_COEF_20V    0x00a4
#define CSC_COEF_21V    0x00a6
#define CSC_COEF_22V    0x00a8
#define CSC_OS_0V       0x00aa
#define CSC_OS_1V       0x00ac
#define CSC_OS_2V       0x00ae

// Page 417
#define CSC_COEF_00S    0x00b0
#define CSC_COEF_01S    0x00b2
#define CSC_COEF_02S    0x00b4
#define CSC_COEF_10S    0x00b6
#define CSC_COEF_11S    0x00b8
#define CSC_COEF_12S    0x00ba
#define CSC_COEF_20S    0x00bc
#define CSC_COEF_21S    0x00be
#define CSC_COEF_22S    0x00c0
#define CSC_OS_0S       0x00c2
#define CSC_OS_1S       0x00c4
#define CSC_OS_2S       0x00c6
#define APS_COEF_GRN1   0x0100
#define APS_COEF_RED    0x0102
#define APS_COEF_BLUE   0x0104
#define APS_COEF_GRN2   0x0106
#define T_DGEN_M        0x0108
#define CCIR_TIMING_2   0x010a
#define CCIR_TIMING_3   0x010c
#define SERIAL_CTRL     0x010e
#define DM_COEF_GRN1    0x0110
#define DM_COEF_RED     0x0112
#define DM_COEF_BLUE    0x0114
#define DM_COEF_GRN2    0x0116
#define SSC_TIMING      0x0118
#define SSC_PERIOD      0x011a
#define JPEG_RESTART    0x011c

// Page 418
#define HSYNC_PER_VID   0x011e
#define HSYNC_PER_STL   0x0120
#define G1_G2_THRESH    0x0124
#define S_PLL_CTRL_0    0x0126
#define S_PLL_CTRL_1    0x0128
#define S_PLL_CTRL_2    0x012a
#define S_DIVBY_UART    0x012c
#define S_EXT_DIVBY_VID 0x012e
#define S_EXT_DIVBY_STL 0x0130
#define BAD_FRAME_CNT_1 0x0132
#define BAD_FRAME_CNT_2 0x0134
#define BAD_FRAME_CNT_3 0x0136
#define BAD_FRAME_CNT_4 0x0138
#define OUT_FRAME_CNT   0x013a
#define DROP_FRAME_CNT  0x013c
#define BPA_BADPIX_CNT  0x013e
#define APS_FRAME_CNT   0x0140
#define BPA_OUTL_PED    0x0142
#define BPA_SCALE       0x0144
#define S_PLL_DIVBY_VID 0x0146
#define S_PLL_DIVBY_STL 0x0148

#define TCTRL_VID       0x2000
#define HBLANK_VID      0x2002
#define VBLANK_VID      0x2004
#define MIN_MAX_F_VID   0x2006
#define RPT_VID         0x2008

// Page 419
#define TCTRL_STL       0x2010
#define HBLANK_STL      0x2012
#define VBLANK_STL      0x2014
#define MIN_MAX_F_STL   0x2016
#define RPT_STL         0x2018
#define AF_CTRL_1       0x2020
#define AF_CTRL_2       0x2022
#define AF_STATUS       0x2024
#define MASTER_CLK_FREQ 0x2026
#define AE_GAIN_MIN     0x2028
#define AE_GAIN_MIN_P   0x202a
#define AE_GAIN_MAX     0x202c
#define AE_GAIN_DFLT    0x202e
#define AE2_ETIME_MIN   0x2030
#define AE2_ETIME_MAX   0x2032
#define AE2_ETIME_DFLT  0x2034
#define AE_TARGET       0x2036
#define AE_TOL_ACQ      0x2038
#define AE_TOL_MON      0x203a
#define AE_MARGIN       0x203c
#define AE_DOE_FACTOR   0x203e
#define AE_DOE_MARGIN   0x2040
#define AWB_RED_MIN     0x2044
#define AWB_RED_MAX     0x2046

// Page 420
#define AWB_RED_DFLT    0x2048
#define AWB_BLUE_MIN    0x204a
#define AWB_BLUE_MAX    0x204c
#define AWB_BLUE_DFLT   0x204e
#define AWB_TOL_ACQ     0x2050
#define AWB_TOL_MON     0x2052
#define FLICK_1         0x2054
#define FLICK_2         0x2056
#define ERROR_FLAGS     0x205e
#define NACC_TABLE      0x2060


/***********************************************************************
 *
 * Sensor Register Offset
 *
 ***********************************************************************/
#define IDENT           0x00
#define STATUS          0x01
#define ICTRL           0x05
#define ITMG            0x06
#define FWROW           0x0a
#define FWCOL           0x0b
#define LWROW           0x0c
#define LWCOL           0x0d
#define TCTRL           0x0e
#define ERECPGA         0x0f
#define EROCPGA         0x10
#define ORECPGA         0x11
#define OROCPGA         0x12
#define ROWEXPL         0x13
#define ROWEXPH         0x14
#define SROWEXP         0x15
#define ERROR           0x16
#define HBLANK          0x19
#define VBLANK          0x1a
#define CONFIG          0x1b
#define CONTROL         0x1c
#define TEST0           0x1d
#define TEST1           0x1e
#define TEST2           0x1f
#define TEST3           0x20
#define TEST4           0x24
#define TEST5           0x25
#define CONFIG_2        0x27


/***********************************************************************
 *
 * Bit/Field definitions
 *
 ***********************************************************************/
#define CMD_1_VE        0x0001
#define CMD_1_LPE       0x0002

#define CMD_2_SNAP      0x0001
#define CMD_2_ACS       0x0002
#define CMD_2_AVT       0x0004
#define CMD_2_AST       0x0008
#define CMD_2_SGO       0x0010
#define CMD_2_P_RESET   0x0020
#define CMD_2_S_RESET   0x0040
#define CMD_2_U_RESET   0x0080
#define CMD_2_PLL_ON    0x0100
#define CMD_2_UCGS      0x0200
#define CMD_2_PLL_OFF   0x0400
#define CMD_2_IA        0x1000
#define CMD_2_UART_F    0x2000
#define CMD_2_SVWC      0x4000
#define CMD_2_SVHC      0x8000

#define BYPASS_CTRL_BPA     0x0001
#define BYPASS_CTRL_G1G2    0x0002
#define BYPASS_CTRL_SB      0x0004
#define BYPASS_CTRL_AQA     0x0008
#define BYPASS_CTRL_PCD     0x0010
#define BYPASS_CTRL_QT_HEAD 0x0020
#define BYPASS_CTRL_WGF     0x0040
#define BYPASS_CTRL_OLP     0x0080
#define BYPASS_CTRL_CAM_CON 0x0100
#define BYPASS_CTRL_DDFF    0x0200

#define VIDEO_CONFIG_SS             0x0004
#define VIDEO_CONFIG_SHP            0x0008
#define VIDEO_CONFIG_H_MIRROR       0x0010
#define VIDEO_CONFIG_V_MIRROR       0x0020
#define VIDEO_CONFIG_O_FORMAT_SHIFT 8
#define VIDEO_CONFIG_O_FORMAT_MASK  (0xF<<VIDEO_CONFIG_O_FORMAT_SHIFT)
#define VIDEO_CONFIG_JPEG_MASK      0x2000

#define STILL_CONFIG_SS             0x0004
#define STILL_CONFIG_SHP            0x0008
#define STILL_CONFIG_H_MIRROR       0x0010
#define STILL_CONFIG_V_MIRROR       0x0020
#define STILL_CONFIG_O_FORMAT_SHIFT 8
#define STILL_CONFIG_O_FORMAT_MASK  (0xF<<STILL_CONFIG_O_FORMAT_SHIFT)
#define STILL_CONFIG_JPEG_MASK      0x2000

#define BYPASS_CTRL_SB              0x0004

#define SENSOR_CTRL_RW              0x0010
#define SENSOR_CTRL_GO              0x0020

#define AF_STATUS_CC                0x0008

#define S_PLL_CTRL_0_S_PLL_SE       0x0010



/***********************************************************************
 *
 * Function Prototype
 *
 ***********************************************************************/
// Read and Write from/to Image Pipeline Registers
int adcm2650_pipeline_read( u16 reg_addr, u16 * reg_value);
int adcm2650_pipeline_write( u16 reg_addr, u16 reg_value);
int adcm2650_pipeline_read_rl( u16 reg_addr, u16 mask, u16 field);
int adcm2650_pipeline_write_wa( u16 reg_addr, u16 mask);
int adcm2650_pipeline_write_wo( u16 reg_addr, u16 mask);

// Read and Write from/to Image Sensor Registers: The image sensor registers are 8bit wide.
int adcm2650_sensor_read_rs( u8 reg_addr, u8 * reg_value );
int adcm2650_sensor_write_ws( u8 reg_addr, u8 reg_value );

// adcm2650_ Wait
void adcm2650_wait( int ms );

// Configuration Procedures
int adcm2650_power_on( u8 );
int adcm2650_power_off( void );
int adcm2650_change_viewfinder_mode(adcm_window_size * input_win, adcm_window_size *vf_output_win, adcm_window_size *sf_output_win );
int adcm2650_jpeg_slow_still_frame( void );
int adcm2650_jpeg_fast_still_frame( void );
int adcm2650_pll(void);
int adcm2650_auto_config_complete( void );
int adcm2650_firmware_upgrade( void );
int adcm2650_version_revision(u16 * cm_revision, u8 *sensor_revision);
int adcm2650_viewfinder_on( void );
int adcm2650_viewfinder_off( void );
int adcm2650_power_low( void );
int adcm2650_power_normal( void );
int adcm2650_suspend( void );
int adcm2650_wakeup( void );
int adcm2650_abort_image( void );
int adcm2650_snapshot_trigger( void );
int adcm2650_snapshot_complete( void );
int adcm2650_snapshot_status(u16 *);
int adcm2650_image_flip_v(u16 mode);
int adcm2650_image_flip_h(u16 mode);
int adcm2650_image_flip_vh(u16 mode);
int adcm2650_pll_active( void );
int adcm2650_pll_deactive( void );
int adcm2650_pll_configuration( void );
int adcm2650_pll_synchronized( void );
int adcm2650_master_clock( u8 clk );
int adcm2650_sensor_voltage( u8 vltg );
int adcm2650_factory_overwrite( void );
int adcm2650_viewfinder_input_size( adcm_window_size * win );
int adcm2650_stillframe_input_size( adcm_window_size * win );
int adcm2650_viewfinder_output_size(adcm_window_size * win);
int adcm2650_stillframe_output_size(adcm_window_size * win);
int adcm2650_flicker_rate(u16 rate);
int adcm2650_low_light( void );
int adcm2650_normal_light( void );
int adcm2650_discard_Viewfinder_image_data( void );
int adcm2650_set_output_to_jpeg(u16 mode);
int adcm2650_stillframe_cfg_output(u16 format);
int adcm2650_viewfinder_cfg_output(u16 format);
int adcm2650_switch_to_jpeg(u16 mode);
int adcm2650_switch_to_normal(u16 mode);
int adcm2650_gamma_correction( void );
int adcm2650_get_output_frame_rate(u16 * fps);
int adcm2650_detect_camera_mode(u16 *mode);
int adcm2650_camera_ready( void );
int adcm2650_wait_till_in_stillframe_mode( void );
int adcm2650_halt_video_output( void );
int adcm2650_get_single_image( void );
int adcm2650_resume_to_full_output_mode( void );
int adcm2650_manual_a_table( void );
int adcm2650_auto_q_table( void );

// i2c 
void i2c_init(void);
void i2c_deinit(void);

// helper apis
void adcm2650_dump_pipeline_register(u16 start_reg_addr, u16 end_reg_addr, u16* buffer);
void adcm2650_dump_sensor_register(u16 start_reg_addr, u16 end_reg_addr, u8* buffer);
#define WO( a, b )      adcm2650_pipeline_write_wo((a), (b))
#define WA( a, b )      adcm2650_pipeline_write_wa((a), (b))
#define RL( a, m, f )   adcm2650_pipeline_read_rl((a), (m), (f))
#define R( a, p )       adcm2650_pipeline_read((a), (p))
#define W( a, v )       adcm2650_pipeline_write((a), (v))
#define RS( a, p )      adcm2650_sensor_read_rs((a), (p))
#define WS( a, v )      adcm2650_sensor_write_ws((a), (v))
#define WAIT( ms )      adcm2650_wait( ms )

#endif
