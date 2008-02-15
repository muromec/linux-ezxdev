/*
 * include/asm-arm/hardware/fs453.h
 *
 * Register definitions for the FS453/454 TVOUT device
 *
 * Author: Ruslan Sushko <rsushko@ru.mvista.com, or source@mvista.com>
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __FS453_H__
#define __FS453_H__

/* Multi bit register write access */
#define FS453_SET_REG_F(_field_, _val_)             \
    (((_val_) << FS453_I2C_##_field_##_S) &         \
     FS453_I2C_##_field_)

/* Multi bit register read access */
#define FS453_GET_REG_F(_reg_, _field_, _val_)      \
    (((_val_) & FS453_I2C_##_field_)                \
      >> FS453_I2C_##_field_##_S)

#define FS453_I2C_DEV_ID    (0x6A)  /* PullUp:  0xD5 - read, 0xD4 - write */
                                    /* PullDown:0x95 - read, 0x94 - write */

/* IHO -- Input horizontal offset */
#define FS453_I2C_IHO               (0x00)
#define FS453_I2C_IHO_SIZE          (2)
#define FS453_I2C_IHO_IHO           (0x07FF)
#define FS453_I2C_IHO_IHO_S         (0)

/* IVO -- Input vertical offset */
#define FS453_I2C_IVO               (0x02)
#define FS453_I2C_IVO_SIZE          (2)
#define FS453_I2C_IVO_IVO           (0x07FF)
#define FS453_I2C_IVO_IVO_S         (0)

/* IHW -- Input Horizontal Width */
#define FS453_I2C_IHW               (0x04)
#define FS453_I2C_IHW_SIZE          (2)
#define FS453_I2C_IHW_IHW           (0x03FF)
#define FS453_I2C_IHW_IHW_S         (0)

/* VSC -- Vertical Scaling Coeficient */
#define FS453_I2C_VSC               (0x06)
#define FS453_I2C_VSC_SIZE          (2)
#define FS453_I2C_VSC_VSC           (0xFFFF)
#define FS453_I2C_VSC_VSC_S         (0)

/* HSC -- Horizontal Scaling Coeficient */
#define FS453_I2C_HSC               (0x08)
#define FS453_I2C_HSC_SIZE          (2)
#define FS453_I2C_HSC_HDSC          (0x00FF)
#define FS453_I2C_HSC_HDSC_S        (0)
#define FS453_I2C_HSC_HUSC          (0xFF00)
#define FS453_I2C_HSC_HUSC_S        (8)

/* BYPASS */
#define FS453_I2C_BYPASS            (0x0A)
#define FS453_I2C_BYPASS_SIZE       (2)
#define FS453_I2C_BYPASS_HDS_BYPASS (0x0002)
#define FS453_I2C_BYPASS_CAC_BYPASS (0x0008)
#define FS453_I2C_BYPASS_B_BYPASS   (0x0010)

/* CR -- Command register  */
#define FS453_I2C_CR                (0x0C)
#define FS453_I2C_CR_SIZE           (2)
#define FS453_I2C_CR_SRESET         (0x0001)
#define FS453_I2C_CR_NCOEN          (0x0002)
#define FS453_I2C_CR_CDEC_BP        (0x0010)
#define FS453_I2C_CR_CACQ_CLR       (0x0020)
#define FS453_I2C_CR_FIFO_CLR       (0x0040)
#define FS453_I2C_CR_SYNC_MS        (0x0080)
#define FS453_I2C_CR_PAL_NTSCIN     (0x0100)
#define FS453_I2C_CR_CBAR_480P      (0x0200)
#define FS453_I2C_CR_P656_OUT       (0x0400)
#define FS453_I2C_CR_P656_LVL       (0x1000)
#define FS453_I2C_CR_GCC_CK_LVL     (0x2000)
#define FS453_I2C_CR_GCC_S_LVL      (0x4000)

/* MISC -- Miscellaneous Bits Register */
#define FS453_I2C_MISC              (0x0E)
#define FS453_I2C_MISC_SIZE         (2)
#define FS453_I2C_MISC_UIM_MOD      (0x000F)
#define FS453_I2C_MISC_UIM_MOD_S    (0)
#define UIM_MOD_INTEL       (0)
#define UIM_MOD_NVIDIA      (1)
#define UIM_MOD_NATIONAL    (3)
#define FS453_I2C_MISC_UIM_DCLK     (0x0010)
#define FS453_I2C_MISC_UIM_CCLK     (0x0020)
#define FS453_I2C_MISC_UIM_DEC      (0x0080)
#define FS453_I2C_MISCUV_SWAP       (0x0100)
#define FS453_I2C_MISC_UIM_E        (0x0200)
#define FS453_I2C_MISC_BRDG_RST     (0x0400)
#define FS453_I2C_MISC_P_ORDER      (0x0800)

/* NCON -- Numerator of NCO Word */
#define FS453_I2C_NCON              (0x10)
#define FS453_I2C_NCON_SIZE         (4)
#define FS453_I2C_NCON_NCON         (0x01FFFFFF)
#define FS453_I2C_NCON_NCON_S       (0)

/* NCOD -- Denumerator of NCO Word */
#define FS453_I2C_NCOD              (0x14)
#define FS453_I2C_NCOD_SIZE         (4)
#define FS453_I2C_NCOD_NCOD         (0x01FFFFFF)
#define FS453_I2C_NCOD_NCOD_S       (0)

/* PLL M and Pump Control */
#define FS453_I2C_PLLMPC            (0x18)
#define FS453_I2C_PLLMPC_SIZE       (2)
#define FS453_I2C_PLLMPC_PLLG       (0x7000)
#define FS453_I2C_PLLMPC_PLLG_S     (12)
#define FS453_I2C_PLLMPC_PLLM       (0x0FFF)
#define FS453_I2C_PLLMPC_PLLM_S     (0)

/* PLL N */
#define FS453_I2C_PLLN              (0x1A)
#define FS453_I2C_PLLN_SIZE         (2)
#define FS453_I2C_PLLN_PLLN         (0x001F)
#define FS453_I2C_PLLN_PLLN_S       (0)

/* PLL Post Divider*/
#define FS453_I2C_PLLPD             (0x1C)
#define FS453_I2C_PLLPD_SIZE        (2)
#define FS453_I2C_PLLPD_IP          (0x007F)
#define FS453_I2C_PLLPD_IP_S        (0)
#define FS453_I2C_PLLPD_EP          (0x7F00)
#define FS453_I2C_PLLPD_EP_S        (8)

/* ID -- Part identification number */
#define FS453_I2C_ID                (0x32)
#define FS453_I2C_ID_SIZE           (2)
#define FS453_I2C_ID_ID             (0xFE05)

/* FIFO_LAT -- FIFO Latency */
#define FS453_I2C_FIFO_LAT          (0x38)
#define FS453_I2C_FIFO_LAT_SIZE     (2)
#define FS453_I2C_FIFO_LAT_FIFO_LAT (0x00FF)
#define FS453_I2C_FIFO_LAT_FIFO_LAT_S    (0)
#define FS353_FIFO_LAT_DOWNSCALING_VAL  (164)
#define FS353_FIFO_LAT_UPSCALING_VAL    (130)

/* MISC_45 Miscellaneous Bits Register 45 */
#define FS453_I2C_MISC_45           (0x45)
#define FS453_I2C_MISC_45_SIZE      (1)
#define FS453_I2C_MISC_45_BYPYCLP   (0x01)
#define FS453_I2C_MISC_45_CLRBAR    (0x02)

/* VID_CNTL0 -- Video Control 0 */
#define FS453_I2C_VID_CNTL0         (0x92)
#define FS453_I2C_VID_CNTL0_SIZE    (2)
#define FS453_I2C_VID_CNTL0_VID_MODE    (0x3)
#define FS453_I2C_VID_CNTL0_VID_MODE_S  (0)
#define VID_MODE_COMP_SVIDEO    (0)
#define VID_MODE_SDTV_YPRPB     (1)
#define VID_MODE_SCART          (1)
#define VID_MODE_HDTV_YPRPB     (2)
#define VID_MODE_VGA_RGB        (2)
#define FS453_I2C_VID_CNTL0_MATRIX_BYP    (0x0004)
#define FS453_I2C_VID_CNTL0_SYNC_ADD      (0x0008)
#define FS453_I2C_VID_CNTL0_SYNC_BI_TRI   (0x0010)
#define FS453_I2C_VID_CNTL0_SYNC_LVL      (0x0020)
#define FS453_I2C_VID_CNTL0_FIELD_MS      (0x0040)
#define FS453_I2C_VID_CNTL0_INT_PROG      (0x0080)
#define FS453_I2C_VID_CNTL0_HSYNC_INV     (0x0100)
#define FS453_I2C_VID_CNTL0_VSYNC_INV     (0x0200)
#define FS453_I2C_VID_CNTL0_FIELD_INV     (0x0400)
#define FS453_I2C_VID_CNTL0_BLANK_INV     (0x0800)
#define FS453_I2C_VID_CNTL0_VSYNC5_6      (0x1000)
#define FS453_I2C_VID_CNTL0_PRPB_SYNC     (0x2000)
#define FS453_I2C_VID_CNTL0_OBIN_USIG     (0x4000)
#define FS453_I2C_VID_CNTL0_TOP_FIELD     (0x8000)

/* DAC_CNTL DAC Control */
#define FS453_I2C_DAC_CNTL                  (0x9E)
#define FS453_I2C_DAC_CNTL_SIZE             (2)
#define FS453_I2C_DAC_CNTL_DAC_AMUX         (0x0003)
#define FS453_I2C_DAC_CNTL_DAC_AMUX_S       (0)
#define DAC_AMUX_0_A        (0)
#define DAC_AMUX_1_A        (1)
#define DAC_AMUX_2_A        (2)
#define DAC_AMUX_3_A        (3)
#define FS453_I2C_DAC_CNTL_DAC_BMUX         (0x000C)
#define FS453_I2C_DAC_CNTL_DAC_BMUX_S       (2)
#define DAC_BMUX_0_B        (0)
#define DAC_BMUX_1_B        (1)
#define DAC_BMUX_2_B        (2)
#define DAC_BMUX_3_B        (3)
#define FS453_I2C_DAC_CNTL_DAC_CMUX         (0x0030)
#define FS453_I2C_DAC_CNTL_DAC_CMUX_S       (4)
#define DAC_CMUX_0_C        (0)
#define DAC_CMUX_1_C        (1)
#define DAC_CMUX_2_C        (2)
#define DAC_CMUX_3_C        (3)
#define FS453_I2C_DAC_CNTL_DAC_DMUX         (0x00C0)
#define FS453_I2C_DAC_CNTL_DAC_DMUX_S       (6)
#define DAC_DMUX_0_D        (0)
#define DAC_DMUX_1_D        (1)
#define DAC_DMUX_2_D        (2)
#define DAC_DMUX_3_D        (3)

/* PWR_MGNT -- Power Management */
#define FS453_I2C_PWR_MGNT                  (0xA0)
#define FS453_I2C_PWR_MGNT_SIZE             (0x2)
#define FS453_I2C_PWR_MGNT_DAC_A_OFF        (0x0001)
#define FS453_I2C_PWR_MGNT_DAC_B_OFF        (0x0002)
#define FS453_I2C_PWR_MGNT_DAC_C_OFF        (0x0004)
#define FS453_I2C_PWR_MGNT_DAC_D_OFF        (0x0008)
#define FS453_I2C_PWR_MGNT_BGAP_OFF         (0x0010)
#define FS453_I2C_PWR_MGNT_DAC_A_LP         (0x0020)
#define FS453_I2C_PWR_MGNT_DAC_B_LP         (0x0040)
#define FS453_I2C_PWR_MGNT_DAC_C_LP         (0x0080)
#define FS453_I2C_PWR_MGNT_DAC_D_LP         (0x0100)
#define FS453_I2C_PWR_MGNT_CLK_SOFF         (0x0600)
#define FS453_I2C_PWR_MGNT_CLK_SOFF_S       (9)
#define CLK_SOFF_HDTV       (1)
#define CLK_SOFF_SDTV       (2)
#define FS453_I2C_PWR_MGNT_CLKOFF           (0x0800)
#define FS453_I2C_PWR_MGNT_PLL_PD           (0x1000)
#define FS453_I2C_PWR_MGNT_GTLIO_PD         (0x2000)

/* Quic program register */
#define FS453_I2C_QPR                       (0xC4)
#define FS453_I2C_QPR_SIZE                      (2)
#define FS453_I2C_QPR_QK_PN                 (0x0001)
#define FS453_I2C_QPR_QK_PN_S               (0)
#define PN_NTSC             (0)
#define PN_PAL              (1)
#define FS453_I2C_QPR_QK_MODE               (0x0006)
#define FS453_I2C_QPR_QK_MODE_S             (1)
#define MODE_640x480        (0)
#define MODE_720x480        (1)
#define MODE_800x600        (2)
#define MODE_1024x768       (3)
#define FS453_I2C_QPR_QK_UO                 (0x0008)
#define FS453_I2C_QPR_QK_OM                 (0x0030)
#define FS453_I2C_QPR_QK_OM_S               (4)
#define OM_COMP_SVID                (0)
#define OM_YPRPB                    (1)
#define OM_SCART                    (2)
#define OM_VGA                      (3)
#define FS453_I2C_QPR_QK_FF                 (0x0040)
#define FS453_I2C_QPR_QK_YC_IN              (0x0080)
#define FS453_I2C_QPR_QK_OS                 (0x0300)
#define FS453_I2C_QPR_QK_OS_S               (8)
#define OS_SDTV             (0)
#define OS_480P             (1)
#define OS_720P             (2)
#define OS_1080I            (3)
#define FS453_I2C_QPR_QK_UIM                (0x0C00)
#define FS453_I2C_QPR_QK_UIM_S              (10)
#define UIM_NVIDIA          (1)
#define UIM_INTEL           (2)
#define UIM_NATIONAL        (3)
#define FS453_I2C_QPR_QK_INIT               (0xF000)
#define FS453_I2C_QPR_QK_INIT_S             (12)
#define QK_INIT_CODE               (9)


#define FS453_XTAL_IN       (27000000ull)


#endif /* __FS453_H__ */
