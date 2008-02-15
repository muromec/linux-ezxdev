/*
 *  adcm3800_hw.h
 *
 *  Agilent ADCM 3800 Camera Module driver.
 *
 *  Copyright (C) 2005 Motorola Inc.
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
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision History:
 *                             Modification     Tracking
 *  Author                 Date          Number     Description of Changes
 *  ----------------   ------------    ----------   -------------------------
 *  Mu Chenliang        06/14/2004      LIBee41682   Created, modified from adcm2700_hw.h
 *  Mu Chenliang        09/29/2004      LIBff19648   Update
*/

/*
 * ==================================================================================
 *                                  INCLUDE FILES
 * ==================================================================================
 */

#ifndef _PXA_ADCM3800_HW_H__
#define _PXA_ADCM3800_HW_H__

#include "camera.h"

/***********************************************************************
 * 
 * Constants
 *
 ***********************************************************************/
 
// Revision constants
#define PIPE_REV            0x68
//#define PIPE_REV_NEW        0x68
#define SENSOR_REV          0x68

// Others
#define adcm3800__TIMEOUT   400               // times to timeout.

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

#define O_FORMAT_NONE        65535  //

/*************************************************************************************
 *
 * Simple Control Registers Address
 *
 *************************************************************************************/
//      name                addr    description                        default   page
#define SREG_ID             0x0000  //Chip ID                           0x0060   139
#define SREG_CONTROL        0x0002  //Camera control                    0x0001   140
#define SREG_STATUS         0x0004  //Camera status                     0x0004   142
#define SREG_CLK_FREQ       0x0006  //Input clock frequency             0x32c8   144
#define SREG_SIZE           0x0008  //Image size and orientation        0x0605   145
#define SREG_OUTPUT_FORMAT  0x000a  //Output format                     0x0909   147
#define SREG_EXPOSURE       0x000c  //Exposure                          0x03e8   149
#define SREG_EXP_ADJ        0x000e  //Exposure adjustment               0x0000   150
#define SREG_ILLUM          0x0010  //Illumination                      0x0000   151
#define SREG_FRAME_RATE     0x0012  //Requested frame rate              0x0096   152
#define SREG_A_FRAME_RATE   0x0016  //Actual frame rate                 0x0096   154
#define SREG_SENSOR_WID_V   0x0018  //Sensor window width, video mode   0x0000   155
#define SREG_SENSOR_HGT_V   0x001a  //Sensor window height, video mode  0x0000   156
#define SREG_OUTPUT_WID_V   0x001c  //Output window width, video mode   0x0000   157
#define SREG_OUTPUT_HGT_V   0x001e  //Output window height, video mode  0x0000   158
#define SREG_SENSOR_WID_S   0x0020  //Sensor window width, still mode   0x0000   159
#define SREG_SENSOR_HGT_S   0x0022  //Sensor window height, still mode  0x0000   160
#define SREG_OUTPUT_WID_S   0x0024  //Output window width, still mode   0x0000   161
#define SREG_OUTPUT_HGT_S   0x0026  //Output window height, still mode  0x0000   162

/*************************************************************************************
 *
 * Expert Hardware Registers
 *
 *************************************************************************************/
//      name                addr       description                        default   page
#define  EREG_I_CLK_DIV      0x3000    //Initial clock divider              0x0001  166
#define  EREG_CTL_CLK_DIV    0x3002    //Clock dividers for control 
                                       //and serial interfaces              0x4000  167
#define  EREG_SEN_CLK_DIV    0x3004    //Sensor clock dividers              0x0000  168
#define  EREG_IP_CLK_DIV     0x3006    //Clock dividers for image pipeline  0x0000  169
#define  EREG_TST_MODE       0x3008    //Latched test mode                  0x0000  170
#define  EREG_SER_ADDR       0x300a    //Serial interface device address    0x0053  171
#define  EREG_SER_PARM       0x300c    //Serial Interface parameters        0x0000  172
#define  EREG_OUT_CTRL       0x300e    //Output control                     0x0000  173
#define  EREG_PLL_CTRL       0x3010    //PLL control                        0x0014  470
#define  EREG_PLL_DIV_L      0x3012    //PLL divisors, large values         0x500C  471
#define  EREG_PLL_DIV_S      0x3014    //PLL divisors, small values         0x500C  472

/*************************************************************************************
 *
 * Expert Control Registers
 *
 *************************************************************************************/
//      name                  addr      description                             default  page
#define  EREG_SZR_IN_WID_V    0x0100   //Sizer input width, video mode           0x0280  184
#define  EREG_SZR_IN_HGT_V    0x0102   //Sizer input height, video mode          0x01e0  185
#define  EREG_SZR_OUT_WID_V   0x0104   //Sizer output width, video mode          0x0140  186
#define  EREG_SZR_OUT_HGT_V   0x0106   //Sizer output height, video mode         0x00f0  187
#define  EREG_CPP_V           0x0108   //Clocks per pixel, video mode            0x0002  188
#define  EREG_HBLANK_V        0x010a   //Horizontal blanking period, video mode  0x0000  189
#define  EREG_VBLANK_V        0x010c   //Vertical blanking period, video mode    0x0000  190
#define  EREG_MIN_MAX_F_V     0x010e   //Frame convergence rates, video mode     0x0000  191
#define  EREG_OUTPUT_CTRL_V   0x0110   //Output control, video mode              0x9019  192
#define  EREG_PROC_CTRL_V     0x0112   //Processing control, video mode          0x0280  194
#define  EREG_RPT_V           0x0114   //Row processing time, video mode         0x0546  196
#define  EREG_HYSNC_PER_V     0x0116   //HSYNC period, video mode                0x0a8b  197
#define  EREG_CLK_DIV_V       0x0118   //Clock divisors, video mode              0x0000  198
#define  EREG_PARALLEL_CTRL_V 0x011a   //Parallel output control, video mode     0x0003  199
#define  EREG_SEN_CTRL_V      0x011c   //Sensor control, video mode              0x0000  200

#define  EREG_SZR_IN_WID_S    0x0120   //Sizer input width, still mode           0x0280  202
#define  EREG_SZR_IN_HGT_S    0x0122   //Sizer input height, still mode          0x01e0  203
#define  EREG_SZR_OUT_WID_S   0x0124   //Sizer output width, still mode          0x0280  204
#define  EREG_SZR_OUT_HGT_S   0x0126   //Sizer output height, still mode         0x01e0  205
#define  EREG_CPP_S           0x0128   //Clocks per pixel, still mode            0x0002  206
#define  EREG_HBLANK_S        0x012a   //Horizontal blanking period, still mode  0x0000  207
#define  EREG_VBLANK_S        0x012c   //Vertical blanking period, still mode    0x0000  208
#define  EREG_MIN_MAX_F_S     0x012e   //Frame convergence rates, still mode     0x0002  209
#define  EREG_OUTPUT_CTRL_S   0x0130   //Output control, still mode              0x8019  210
#define  EREG_PROC_CTRL_S     0x0132   //Processing control, still mode          0x0280  212
#define  EREG_RPT_S           0x0134   //Row processing time, still mode         0x0546  214
#define  EREG_HYSNC_PER_S     0x0136   //HSYNC period, still mode                0x0545  215
#define  EREG_CLK_DIV_S       0x0138   //Clock divisors, still mode              0x0000  216
#define  EREG_PARALLEL_CTRL_S 0x013a   //Parallel output control, still mode     0x0000  217
#define  EREG_SEN_CTRL_S      0x013c   //Sensor control, still mode              0x0000  218

#define  EREG_AF_CTRL1        0x0140   //Auto functions control 1                0x0013  220
#define  EREG_AF_CTRL2        0x0142   //Auto functions control 2                0x0001  221
#define  EREG_AF_STATUS       0x0144   //Auto functions status                   0x0000  222
#define  EREG_SOF_CODES       0x0146   //Start of frame codes                    0xfeff  223
#define  EREG_EOF_CODES       0x0148   //End of frame codes                      0x0100  224
#define  EREG_ABL_TARGET      0x014a   //Auto black level target                 0x0005  225
#define  EREG_ABL_MAX_BIN     0x014c   //Auto black level maximum bin            0x0003  226
#define  EREG_ABL_MAX_BLACK   0x014e   //Auto black level maximum black          0x0010  227
#define  EREG_AE_GAIN_MIN     0x0150   //Auto exposure gain minimum              0x01c0  228
#define  EREG_AE_GAIN_MIN_P   0x0152   //Auto exposure gain minimum, preferred   0x0200  229
#define  EREG_AE_GAIN_MAX     0x0154   //Auto exposure gain maximum              0x0500  230
#define  EREG_AE_GAIN_DFLT    0x0156   //Auto exposure gain default              0x0200  231
#define  EREG_AE_ETIME_MIN    0x0158   //Auto exposure time minimum              0x0005  232
#define  EREG_AE_ETIME_MAX    0x015a   //Auto exposure time maximum              0x4e20  233
#define  EREG_AE_ETIME_DFLT   0x015c   //Auto exposure time default              0x03e8  234
#define  EREG_AE_TARGET       0x015e   //Auto exposure target                    0x0040  235
#define  EREG_AE_TOL_ACQ      0x0160   //Auto exposure tolerance acquire         0x0118  236
#define  EREG_AE_TOL_MON      0x0162   //Auto exposure tolerance monitor         0x0118  237
#define  EREG_AE_MARGIN       0x0164   //Auto exposure margin                    0x0120  238
#define  EREG_AE_DOE_FACTOR   0x0166   //AE deliberate overexposure factor       0x014e  239
#define  EREG_AE_DOE_MARGIN   0x0168   //AE deliberate overexposure margin       0x0140  240

#define  EREG_AWB_RED_MIN     0x0170   //AWB minimum red/green ratio             0x00c0  242
#define  EREG_AWB_RED_MAX     0x0172   //AWB maximum red/green ratio             0x01a6  243
#define  EREG_AWB_RED_DFLT    0x0174   //AWB default red/green ratio             0x0134  244
#define  EREG_AWB_BLUE_MIN    0x0176   //AWB minimum blue/green ratio            0x00c0  245
#define  EREG_AWB_BLUE_MAX    0x0178   //AWB maximum blue/green ratio            0x02a4  246
#define  EREG_AWB_BLUE_DFLT   0x017a   //AWB default blue/green ratio            0x01e4  247
#define  EREG_AWB_TOL_ACQ     0x017c   //Auto white balance tolerance acquire    0x0110  248
#define  EREG_AWB_TOL_MON     0x017e   //Auto white balance tolerance monitor    0x0120  249
#define  EREG_FIRMWARE_REV    0x0180   //Current firmware revision               0x0152  250
#define  EREG_FLICK_CFG_1     0x0182   //Flicker configuration 1                 0x2aeb  251
#define  EREG_FLICK_CFG_2     0x0184   //Flicker configuration 2                 0x0005  252

#define  EREG_MAX_SCLK        0x018a   //Maximum sensor clock                    0x1964  254

#define  EREG_CSC_00_V        0x0190   //Color conversion coefficient 00, video  0x0026  256
#define  EREG_CSC_01_V        0x0192   //Color conversion coefficient 01, video  0x004b  256
#define  EREG_CSC_02_V        0x0194   //Color conversion coefficient 02, video  0x000f  256
#define  EREG_CSC_10_V        0x0196   //Color conversion coefficient 10, video  0x01ed  256
#define  EREG_CSC_11_V        0x0198   //Color conversion coefficient 11, video  0x01db  256
#define  EREG_CSC_12_V        0x019a   //Color conversion coefficient 12, video  0x0038  256
#define  EREG_CSC_20_V        0x019c   //Color conversion coefficient 20, video  0x004f  256
#define  EREG_CSC_21_V        0x019e   //Color conversion coefficient 21, video  0x01be  256
#define  EREG_CSC_22_V        0x01a0   //Color conversion coefficient 22, video  0x01f3  256

#define  EREG_CSC_OS0_V       0x01a2   //Color space conversion offset 0, video  0x0000  257
#define  EREG_CSC_OS1_V       0x01a4   //Color space conversion offset 1, video  0x0080  257
#define  EREG_CSC_OS2_V       0x01a6   //Color space conversion offset 2, video  0x0080  257

#define  EREG_CSC_00_S        0x01a8   //Color conversion coefficient 00, still  0x0026  258
#define  EREG_CSC_01_S        0x01aa   //Color conversion coefficient 01, still  0x004b  258
#define  EREG_CSC_02_S        0x01ac   //Color conversion coefficient 02, still  0x000f  258
#define  EREG_CSC_10_S        0x01ae   //Color conversion coefficient 10, still  0x01ed  258
#define  EREG_CSC_11_S        0x01b0   //Color conversion coefficient 11, still  0x01db  258
#define  EREG_CSC_12_S        0x01b2   //Color conversion coefficient 12, still  0x0038  258
#define  EREG_CSC_20_S        0x01b4   //Color conversion coefficient 20, still  0x004f  258
#define  EREG_CSC_21_S        0x01b6   //Color conversion coefficient 21, still  0x01be  258
#define  EREG_CSC_22_S        0x01b8   //Color conversion coefficient 22, still  0x01f3  258
                                                                                       
#define  EREG_CSC_OS0_S       0x01ba   //Color space conversion offset 0, still  0x0000  259
#define  EREG_CSC_OS1_S       0x01bc   //Color space conversion offset 1, still  0x0080  259
#define  EREG_CSC_OS2_S       0x01be   //Color space conversion offset 2, still  0x0080  259


#define  EREG_TM_COEF_00_V    0x01c0   //Tonemap coefficient 00, video           0x0000  261
#define  EREG_TM_COEF_01_V    0x01c2   //Tonemap coefficient 01, video           0x0017  261
#define  EREG_TM_COEF_02_V    0x01c4   //Tonemap coefficient 02, video           0x0032  261
#define  EREG_TM_COEF_03_V    0x01c6   //Tonemap coefficient 03, video           0x0046  261
#define  EREG_TM_COEF_04_V    0x01c8   //Tonemap coefficient 04, video           0x0056  261
#define  EREG_TM_COEF_05_V    0x01ca   //Tonemap coefficient 05, video           0x0064  261
#define  EREG_TM_COEF_06_V    0x01cc   //Tonemap coefficient 06, video           0x0071  261
#define  EREG_TM_COEF_07_V    0x01ce   //Tonemap coefficient 07, video           0x007c  261
#define  EREG_TM_COEF_08_V    0x01d0   //Tonemap coefficient 08, video           0x0086  261
#define  EREG_TM_COEF_09_V    0x01d2   //Tonemap coefficient 09, video           0x0099  261
#define  EREG_TM_COEF_10_V    0x01d4   //Tonemap coefficient 10, video           0x00a9  261
#define  EREG_TM_COEF_11_V    0x01d6   //Tonemap coefficient 11, video           0x00b8  261
#define  EREG_TM_COEF_12_V    0x01d8   //Tonemap coefficient 12, video           0x00c6  261
#define  EREG_TM_COEF_13_V    0x01da   //Tonemap coefficient 13, video           0x00df  261
#define  EREG_TM_COEF_14_V    0x01dc   //Tonemap coefficient 14, video           0x00f5  261
#define  EREG_TM_COEF_15_V    0x01de   //Tonemap coefficient 15, video           0x0109  261
#define  EREG_TM_COEF_16_V    0x01e0   //Tonemap coefficient 16, video           0x011b  261
#define  EREG_TM_COEF_17_V    0x01e2   //Tonemap coefficient 17, video           0x013d  261
#define  EREG_TM_COEF_18_V    0x01e4   //Tonemap coefficient 18, video           0x015a  261
#define  EREG_TM_COEF_19_V    0x01e6   //Tonemap coefficient 19, video           0x0175  261
#define  EREG_TM_COEF_20_V    0x01e8   //Tonemap coefficient 20, video           0x018d  261
#define  EREG_TM_COEF_21_V    0x01ea   //Tonemap coefficient 21, video           0x01ba  261
#define  EREG_TM_COEF_22_V    0x01ec   //Tonemap coefficient 22, video           0x01e1  261
#define  EREG_TM_COEF_23_V    0x01ee   //Tonemap coefficient 23, video           0x0205  261
#define  EREG_TM_COEF_24_V    0x01f0   //Tonemap coefficient 24, video           0x0225  261
#define  EREG_TM_COEF_25_V    0x01f2   //Tonemap coefficient 25, video           0x0261  261
#define  EREG_TM_COEF_26_V    0x01f4   //Tonemap coefficient 26, video           0x0295  261
#define  EREG_TM_COEF_27_V    0x01f6   //Tonemap coefficient 27, video           0x02c5  261
#define  EREG_TM_COEF_28_V    0x01f8   //Tonemap coefficient 28, video           0x02f1  261
#define  EREG_TM_COEF_29_V    0x01fa   //Tonemap coefficient 29, video           0x033f  261
#define  EREG_TM_COEF_30_V    0x01fc   //Tonemap coefficient 30, video           0x0385  261
#define  EREG_TM_COEF_31_V    0x01fe   //Tonemap coefficient 31, video           0x03c5  261
#define  EREG_TM_COEF_32_V    0x0200   //Tonemap coefficient 32, video           0x0400  261

#define  EREG_TM_COEF_00_S    0x0202   //Tonemap coefficient 00, still           0x0000  262
#define  EREG_TM_COEF_01_S    0x0204   //Tonemap coefficient 01, still           0x0017  262
#define  EREG_TM_COEF_02_S    0x0206   //Tonemap coefficient 02, still           0x0032  262
#define  EREG_TM_COEF_03_S    0x0208   //Tonemap coefficient 03, still           0x0046  262
#define  EREG_TM_COEF_04_S    0x020a   //Tonemap coefficient 04, still           0x0056  262
#define  EREG_TM_COEF_05_S    0x020c   //Tonemap coefficient 05, still           0x0064  262
#define  EREG_TM_COEF_06_S    0x020e   //Tonemap coefficient 06, still           0x0071  262
#define  EREG_TM_COEF_07_S    0x0210   //Tonemap coefficient 07, still           0x007c  262
#define  EREG_TM_COEF_08_S    0x0212   //Tonemap coefficient 08, still           0x0086  262
#define  EREG_TM_COEF_09_S    0x0214   //Tonemap coefficient 09, still           0x0099  262
#define  EREG_TM_COEF_10_S    0x0216   //Tonemap coefficient 10, still           0x00a9  262
#define  EREG_TM_COEF_11_S    0x0218   //Tonemap coefficient 11, still           0x00b8  262
#define  EREG_TM_COEF_12_S    0x021a   //Tonemap coefficient 12, still           0x00c6  262
#define  EREG_TM_COEF_13_S    0x021c   //Tonemap coefficient 13, still           0x00df  262
#define  EREG_TM_COEF_14_S    0x021e   //Tonemap coefficient 14, still           0x00f5  262
#define  EREG_TM_COEF_15_S    0x0220   //Tonemap coefficient 15, still           0x0109  262
#define  EREG_TM_COEF_16_S    0x0222   //Tonemap coefficient 16, still           0x011b  262
#define  EREG_TM_COEF_17_S    0x0224   //Tonemap coefficient 17, still           0x013d  262
#define  EREG_TM_COEF_18_S    0x0226   //Tonemap coefficient 18, still           0x015a  262
#define  EREG_TM_COEF_19_S    0x0228   //Tonemap coefficient 19, still           0x0175  262
#define  EREG_TM_COEF_20_S    0x022a   //Tonemap coefficient 20, still           0x018d  262
#define  EREG_TM_COEF_21_S    0x022c   //Tonemap coefficient 21, still           0x01ba  262
#define  EREG_TM_COEF_22_S    0x022e   //Tonemap coefficient 22, still           0x01e1  262
#define  EREG_TM_COEF_23_S    0x0230   //Tonemap coefficient 23, still           0x0205  262
#define  EREG_TM_COEF_24_S    0x0232   //Tonemap coefficient 24, still           0x0225  262
#define  EREG_TM_COEF_25_S    0x0234   //Tonemap coefficient 25, still           0x0261  262
#define  EREG_TM_COEF_26_S    0x0236   //Tonemap coefficient 26, still           0x0295  262
#define  EREG_TM_COEF_27_S    0x0238   //Tonemap coefficient 27, still           0x02c5  262
#define  EREG_TM_COEF_28_S    0x023a   //Tonemap coefficient 28, still           0x02f1  262
#define  EREG_TM_COEF_29_S    0x023c   //Tonemap coefficient 29, still           0x033f  262
#define  EREG_TM_COEF_30_S    0x023e   //Tonemap coefficient 30, still           0x0385  262
#define  EREG_TM_COEF_31_S    0x0240   //Tonemap coefficient 31, still           0x03c5  262
#define  EREG_TM_COEF_32_S    0x0242   //Tonemap coefficient 32, still           0x0400  262

#define  EREG_NACC_EGP_1      0x0250   //NACC EGP 1                              0x05dc  265
#define  EREG_NACC_SAT_1      0x0252   //NACC saturation 1                       0x0000  265
#define  EREG_NACC_EGP_2      0x0254   //NACC EGP 2                              0x0465  265
#define  EREG_NACC_SAT_2      0x0256   //NACC saturation 2                       0x0040  265
#define  EREG_NACC_EGP_3      0x0258   //NACC EGP 3                              0x02ee  265
#define  EREG_NACC_SAT_3      0x025a   //NACC saturation 3                       0x0080  265
#define  EREG_NACC_EGP_4      0x025c   //NACC EGP 4                              0x0177  265
#define  EREG_NACC_SAT_4      0x025e   //NACC saturation 4                       0x00c0  265
#define  EREG_NACC_EGP_5      0x0260   //NACC EGP 5                              0x0000  265
#define  EREG_NACC_SAT_5      0x0262   //NACC saturation 5                       0x0100  265
#define  EREG_NACC_EGP_6      0x0264   //NACC EGP 6                              0x0000  265
#define  EREG_NACC_SAT_6      0x0266   //NACC saturation 6                       0x0000  265
#define  EREG_NACC_EGP_7      0x0268   //NACC EGP 7                              0x0000  265
#define  EREG_NACC_SAT_7      0x026a   //NACC saturation 7                       0x0000  265
#define  EREG_NACC_EGP_8      0x026c   //NACC EGP 8                              0x0000  265
#define  EREG_NACC_SAT_8      0x026e   //NACC saturation 8                       0x0000  265
#define  EREG_NACC_BC_00      0x0270   //NACC NACC bright coefficients 00        0x0235  266
#define  EREG_NACC_BC_01      0x0272   //NACC NACC bright coefficients 01        0xff46  266
#define  EREG_NACC_BC_02      0x0274   //NACC bright coefficients 02             0xff85  266
#define  EREG_NACC_BC_10      0x0276   //NACC bright coefficients 10             0xff64  266
#define  EREG_NACC_BC_11      0x0278   //NACC bright coefficients 11             0x01fc  266
#define  EREG_NACC_BC_12      0x027a   //NACC bright coefficients 12             0xff9f  266
#define  EREG_NACC_BC_20      0x027c   //NACC bright coefficients 20             0x0008  266
#define  EREG_NACC_BC_21      0x027e   //NACC bright coefficients 21             0xfe8d  266
#define  EREG_NACC_BC_22      0x0280   //NACC bright coefficients 22             0x026b  266
#define  EREG_NACC_DC_00      0x0282   //NACC dark coefficients 00               0x0048  266
#define  EREG_NACC_DC_01      0x0284   //NACC dark coefficients 01               0x010b  266
#define  EREG_NACC_DC_02      0x0286   //NACC dark coefficients 02               0xffaa  266
#define  EREG_NACC_DC_10      0x0288   //NACC dark coefficients 10               0x0048  266
#define  EREG_NACC_DC_11      0x028a   //NACC dark coefficients 11               0x010b  266
#define  EREG_NACC_DC_12      0x028c   //NACC dark coefficients 12               0xffaa  266
#define  EREG_NACC_DC_20      0x028e   //NACC dark coefficients 20               0x0048  266
#define  EREG_NACC_DC_21      0x0290   //NACC dark coefficients 21               0x010b  266
#define  EREG_NACC_DC_22      0x0292   //NACC dark coefficients 22               0xffaa  266
                         
/*************************************************************************************
 *
 * Expert Sensor Registers
 *
 *************************************************************************************/
//      name                  addr      description                             default  page
#define  EREG_IDENT          0x0800    //Image sensor identification             0x60     272
#define  EREG_IS_STATUS      0x0801    //Image sensor status                     0x00     273
#define  EREG_ICTRL          0x0805    //Interface control                       0x00     275

#define  EREG_ADC_CTRL       0x0809    //ADC control                             0x01     277
#define  EREG_FWROW          0x080a    //Window first row address                0x01     278
#define  EREG_FWCOL          0x080b    //Window first column address             0x01     279
#define  EREG_LWROW          0x080c    //Window last row address                 0x7a     280
#define  EREG_LWCOL          0x080d    //Window last column address              0xa2     281
#define  EREG_CLK_PIXEL      0x080e    //Clocks per pixel                        0x02     282
#define  EREG_EREC_PGA       0x080f    //Even row, even column(green 1)PGA gain  0x00     283
#define  EREG_EROC_PGA       0x0810    //Even row, odd column (red) PGA gain     0x00     284
#define  EREG_OREC_PGA       0x0811    //Odd row, even column (blue) PGA gain    0x00     285
#define  EREG_OROC_PGA       0x0812    //Odd row, odd column (green 2) PGA gain  0x00     286
#define  EREG_ROWEXP_L       0x0813    //Row exposure low                        0x54     287
#define  EREG_ROWEXP_H       0x0814    //Row exposure high                       0x00     288
#define  EREG_SROWEXP        0x0815    //Sub row exposure                        0x31     289
#define  EREG_ERROR          0x0816    //Error control                           0x00     290

#define  EREG_HBLANK         0x0819    //Horizontal blank                        0x00     292
#define  EREG_VBLANK         0x081a    //Vertical blank                          0x00     293
#define  EREG_CONFIG_1       0x081b    //Image sensor configuration 1            0x0e     294
#define  EREG_CONTROL_1      0x081c    //Image sensor control 1                  0x24     295

#define  EREG_CONFIG_2       0x0827    //Image sensor configuration 2            0x00     298
#define  EREG_GRR_CTRL       0x0828    //Ground reset reference control          0x00     299

#define  EREG_BIAS_TRM       0x0837    //Bias trim                               0x00     301
#define  EREG_SMP_GR_E2      0x08d7    //Sample ground reference edge 2          0x00     303
#define  EREG_SMP_GR_E1      0x08d8    //Sample ground reference edge 1          0x10     304
#define  EREG_SMP_GR_E0      0x08d9    //Sample ground reference edge 0          0x0a     305

#define  EREG_EXP_GR_E1      0x08dc    //Exposure, ground reference edge 1       0x10     307
#define  EREG_EXP_GR_E0      0x08dd    //Exposure, ground reference edge 0       0x06     308
#define  EREG_GR_POL         0x08df    //Ground reference polarity               0xd3     310

#define  EREG_SMP_RST_E2     0x08eb    //Sample, reset edge 2                    0x04     312
#define  EREG_SMP_RST_E1     0x08ec    //Sample, reset edge 1                    0x10     313
#define  EREG_SMP_RST_E0     0x08ed    //Sample, reset edge 0                    0x07     314

#define  EREG_EXP_RST_E1     0x08f0    //Exposure, reset edge 1                  0x10     316
#define  EREG_EXP_RST_E0     0x08f1    //Exposure, reset edge 1                  0x03     317
#define  EREG_RESET_POL      0x08f3    //Reset polarity enable                   0xd3     319

#define  EREG_SMP_PRST_E2    0x08f5    //Sample, preset edge 2                   0x00     321
#define  EREG_SMP_PRST_E1    0x08f6    //Sample, preset edge 1                   0x02     322
#define  EREG_SMP_PRST_E0    0x08f7    //Sample, preset edge 0                   0x0a     323

#define  EREG_EXP_PRST_E1    0x08fa    //Exposure, preset edge 1                 0x02     325
#define  EREG_EXP_PRST_E0    0x08fb    //Exposure, preset edge 1                 0x06     326
#define  EREG_PRESET_POL     0x08fd    //Preset polarity enable                  0xd3     328

/*************************************************************************************
 *
 * Expert Image Pipeline Registers
 *
 *************************************************************************************/
//      name                  addr      description                               default  page
                                    
#define  EREG_CMD_1           0x1002    //Main command 1                           0x0000  335
#define  EREG_CMD_2           0x1004    //Main command 2 (write 1¡¯s only)         0x0002  336
#define  EREG_OUTPUT_CTRL     0x1008    //Output control, working                  0x9019  338
#define  EREG_PARALLEL_CTRL   0x100a    //Parallel output control working copy     0x0000  340
#define  EREG_SOF_CODE_W      0x100c    //Start of frame code working copy         0x00ff  341
#define  EREG_PEOF_CODES      0x100e    //End of frame codes working copy          0x0100  342
#define  EREG_CCIR_TIMING     0x1010    //CCIR interface timing                    0x0000  343
#define  EREG_R_Y_MAX_MIN     0x1012    //Luminance, Y (or red) maximum/minimum    0xff00  344
#define  EREG_G_CB_MAX_MIN    0x1014    //Chrominance,Cb(or green)maximum/minimum  0xff00  345
#define  EREG_B_CR_MAX_MIN    0x1016    //Chrominance,Cr(or blue)maximum/minimum   0xff00  346
#define  EREG_PROCESS_CTRL    0x1018    //Processing control working copy          0x0280  347
#define  EREG_BPA_SF_GTHRESH  0x101a    //BPA scale factor,green filter threshold  0x0220  349
#define  EREG_BPA_OUTL_PED    0x101c    //BPA outlier, pedestal                    0x4008  350
#define  EREG_BPA_BADPIX_CNT  0x101e    //BPA bad pixel count (read only)          0x0000  351
#define  EREG_SZR_IN_W        0x1020    //Sizer input width                        0x0280  352
#define  EREG_SZR_IN_H        0x1022    //Sizer input height                       0x01e0  353
#define  EREG_SZR_OUT_W       0x1024    //Sizer output width                       0x0140  354
#define  EREG_SZR_OUT_H       0x1026    //Sizer output height                      0x00f0  355


#define  EREG_CC_COEF_00      0x1028    //Color correction coefficient 00          0x02f9  358
#define  EREG_CC_COEF_01      0x102a    //Color correction coefficient 01          0x0f03  358
#define  EREG_CC_COEF_02      0x102c    //Color correction coefficient 02          0x0f02  358
#define  EREG_CC_COEF_10      0x102e    //Color correction coefficient 10          0x0f4f  358
#define  EREG_CC_COEF_11      0x1030    //Color correction coefficient 11          0x025c  358
#define  EREG_CC_COEF_12      0x1032    //Color correction coefficient 12          0x0f54  358
#define  EREG_CC_COEF_20      0x1034    //Color correction coefficient 20          0x0fe0  358
#define  EREG_CC_COEF_21      0x1036    //Color correction coefficient 21          0x0e4a  358
#define  EREG_CC_COEF_22      0x1038    //Color correction coefficient 22          0x02d5  358
                                                                                         
#define  EREG_CC_PRE_OS_0     0x103a    //Color correction pre-offset 0            0x01f8  360
#define  EREG_CC_PRE_OS_1     0x103c    //Color correction pre-offset 1            0x01f8  360
#define  EREG_CC_PRE_OS_2     0x103e    //Color correction pre-offset 2            0x01f8  360
#define  EREG_CC_POST_OS_0    0x1040    //Color correction post-offset 0           0x0000  360
#define  EREG_CC_POST_OS_1    0x1042    //Color correction post-offset 1           0x0000  360
#define  EREG_CC_POST_OS_2    0x1044    //Color correction post-offset 2           0x0000  360

#define  EREG_CSC_COEF_00     0x1046    //Color space conversion coefficient 00    0x0026  363
#define  EREG_CSC_COEF_01     0x1048    //Color space conversion coefficient 01    0x004b  363
#define  EREG_CSC_COEF_02     0x104a    //Color space conversion coefficient 02    0x000f  363
#define  EREG_CSC_COEF_10     0x104c    //Color space conversion coefficient 10    0x01ed  363
#define  EREG_CSC_COEF_11     0x104e    //Color space conversion coefficient 11    0x01db  363
#define  EREG_CSC_COEF_12     0x1050    //Color space conversion coefficient 12    0x0038  363
#define  EREG_CSC_COEF_20     0x1052    //Color space conversion coefficient 20    0x004f  363
#define  EREG_CSC_COEF_21     0x1054    //Color space conversion coefficient 21    0x01be  363
#define  EREG_CSC_COEF_22     0x1056    //Color space conversion coefficient 22    0x01f3  363
#define  EREG_CSC_OS_0        0x1058    //Color space conversion offset 0          0x0000  364
#define  EREG_CSC_OS_1        0x105a    //Color space conversion offset 1          0x0080  364
#define  EREG_CSC_OS_2        0x105c    //Color space conversion offset 2          0x0080  364
#define  EREG_DATA_GEN        0x105e    //Test data generator                      0x0000  365
#define  EREG_HSYNC_PER       0x1060    //Horizontal synchronization period        0x0a8b  366
#define  EREG_APS_COEF_GRN1   0x1062    //Green 1 AWB gain                         0x0080  368
#define  EREG_APS_COEF_RED    0x1064    //Red AWB gain                             0x0080  368


#define  EREG_APS_COEF_BLUE     0x1066  //Blue AWB gain                            0x0080  368
#define  EREG_APS_COEF_GRN2     0x1068  //Green 2 AWB gain                         0x0080  368
#define  EREG_AV_LEFT_TOP       0x106a  //Anti-v,sensor first row and column       0x0101  369
#define  EREG_AV_RIGHT_BOT      0x106c  //Anti-v, sensor last row and column       0xa27a  370
#define  EREG_AV_CENTER_COL     0x106e  //Anti-v, sensor center column             0x0148  371
#define  EREG_AV_CENTER_ROW     0x1070  //Anti-v, sensor center row                0x00f8  372
#define  EREG_STAT_CAP_CTRL     0x1072  //Image statistics capture control         0x0021  373
#define  EREG_STAT_MODE_CTRL    0x1074  //Image statistics mode control            0x0000  374
#define  EREG_GREEN_1_SUM       0x1076  //Green 1 pixel sum                        0x0000  375
#define  EREG_RED_SUM           0x1078  //Red pixel sum                            0x0000  375
#define  EREG_BLUE_SUM          0x107a  //Blue pixel sum                           0x0000  375
#define  EREG_GREEN_2_SUM       0x107c  //Green 2 pixel sum                        0x0000  375
#define  EREG_I_WIDTH           0x107e  //Current image width                      0x0000  376
#define  EREG_I_HEIGHT          0x1080  //Current image height                     0x0000  377
#define  EREG_STATUS_FLAGS      0x1082  //Status flags (read only)                 0x0000  378
#define  EREG_CLK_GATE_DIS      0x1084  //Clock gate disable                       0x0000  379
#define  EREG_CCIR_TIMING2      0x1086  //CCIR interface timing 2                  0x0000  381
#define  EREG_CCIR_TIMING3      0x1088  //CCIR interface timing 3                  0x0010  382
#define  EREG_G1G2_DIAG_THRESH  0x108a  //Green 1/green 2 diagonal threshold       0x0040  383
#define  EREG_BPA_D2_THRESH     0x108c  //BPA second derivative threshold          0x0100  384
#define  EREG_SERIAL_CTRL       0x108e  //Serial control                           0x0000  385
#define  EREG_INTP_CTRL_1       0x1090  //Interpolation control 1(demosaic)        0x0188  387
#define  EREG_INTP_CTRL_2       0x1092  //Interpolation control 2(demosaic)        0x00c8  388
#define  EREG_AV_OVAL_FACT      0x1094  //Anti-vignetting oval factor              0x0100  389
#define  EREG_AV_OS_GREEN1      0x1096  //Anti-vignetting green 1 offset           0x0000  391
#define  EREG_AV_OS_RED         0x1098  //Anti-vignetting red offset               0x0000  391
#define  EREG_AV_OS_BLUE        0x109a  //Anti-vignetting blue offset              0x0000  391
#define  EREG_AV_OS_GREEN2      0x109c  //Anti-vignetting green 2 offset           0x0000  391
                                    

/***********************************************************************
 * 
 * typedefs & Structures
 *
 ***********************************************************************/
 
struct adcm3800_context_s
{
    int video_running;      // =1 video is running
    int SCL_partial;        // =1 partial SCL updated
    int SCL_restart;        // =1 SCL & restart updated

    u32 chipid;

    u16 format;
    u16 sensor_w;
    u16 sensor_h;
    u16 output_w;
    u16 output_h;

    // frame rate control
    unsigned int fps;
    unsigned int mclk;

	int            bright;
	V4l_PIC_STYLE  style;
	V4l_PIC_WB     light;
	int            flicker_freq;

    V4l_NM expo_mode;
    int max_expotime;
};

typedef struct adcm3800_context_s adcm3800_context_t, *p_adcm3800_context_t;
                                                                            

/***********************************************************************                   
 *                                                                                         
 * Function Prototype                 
 *                                    
 ***********************************************************************/

// Configuration Procedures
int adcm3800_power_on(u32 clk);
int adcm3800_power_off(void );
int adcm3800_viewfinder_on( void );
int adcm3800_viewfinder_off( void );
int adcm3800_reconfigure(p_camera_context_t cam_ctx, int frames);

int adcm3800_set_output_format(u16 format);
int adcm3800_set_output_size(u16 width, u16 height);
int adcm3800_set_sensor_size(u16 width, u16 height);

int adcm3800_set_fps(u16 fps);

/*set picture style(normal/black white/sepia/solarize/neg.art)*/
int adcm3800_set_style(V4l_PIC_STYLE style);
/*set picture light(direct sun/incandescent/fluorescent)*/     
int adcm3800_set_light(V4l_PIC_WB light);
/*set picture brightness*/
int adcm3800_set_bright(int bright);
int adcm3800_set_flicker(int freq);
int adcm3800_set_exposure_mode(V4l_NM mode, int maxexpotime);

extern int i2c_adcm3800_read(u16 addr, u16 *pvalue);
extern int i2c_adcm3800_write(u16 addr, u16 value);
extern int i2c_adcm3800_read_byte(u16 addr, u8 *pvalue);
extern int i2c_adcm3800_write_byte(u16 addr, u8 value);

#endif /* _PXA_ADCM3800_HW_H__ */

