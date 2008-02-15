/*
 * FILE NAME include/asm/arch-omap730/omap730_config.h
 *
 * BRIEF MODULE DESCRIPTION
 *
 * Author: MPC-Data Limited
 *         Dave Peverley <dpeverley@mpc-data.co.uk>
 *
 * Copyright (c) 2004, MPC-Data Limited
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */


#ifndef _INCLUDED_ASM_ARCH_OMAP730_CONFIG_H_
#define _INCLUDED_ASM_ARCH_OMAP730_CONFIG_H_


#include <linux/config.h>
#include <linux/types.h>
#include <asm/hardware.h>
#include <asm/arch/hardware.h>


/* 
 * To define a MUX, give it a name "MY_MUX", preferably to match the TRM,
 * then define the following e.g. :
 * 
 *   #define MY_MUX_CONFREG  PERSEUS2_IO_CONF4
 *   #define MY_MUX_POS      5                  // Lowest bit of the field is 5
 *   #define MY_MUX_MSK      0x7                // Mask is 7 as MUX uses 3 bits
 *   #define MY_MUX_STATE1   0x3                // Set bits to 011 for STATE1
 *   #define MY_MUX_STATE2   0x6                // Set bits to 110 for STATE2
 *   
 * You can then use the macro thus :
 *   OMAP730_PIN_MUX(MY_MUX, STATE1);
 *   OMAP730_PIN_MUX(MY_MUX, STATE2);
 *   OMAP730_PIN_PE(MY_MUX, PE_ENABLE);
 *   OMAP730_PIN_PE(MY_MUX, PE_DISABLE);
 *
 */

#define OMAP730_PIN_MUX(mux_name, mux_state)                                                         \
               *((volatile u32 *) mux_name##_CONFREG ) &= ~( mux_name##_MSK << mux_name##_POS );     \
               *((volatile u32 *) mux_name##_CONFREG ) |= mux_name##_##mux_state << mux_name##_POS;  

#define PE_DISABLE                      0
#define PE_ENABLE                       1

#define OMAP730_PIN_PE(mux_name, pe_state)                                                               \
               *((volatile u32 *) mux_name##_CONFREG ) &= ~( 1 << ( mux_name##_POS - 1 ));               \
               *((volatile u32 *) mux_name##_CONFREG ) |= ( pe_state & 1)  << ( mux_name##_POS - 1 );


/* 
 * Configuration macros for other OMAP730 config registers. They work in 
 * much the same way as the mux conf ones do, but the other registers don't 
 * have the same format with 3 bits mode, 1, PE.
 */

#define OMAP730_CONFIGURE(conf_name, conf_state)                                                        \
               *((volatile u32 *) conf_name##_CONFREG ) &= ~( conf_name##_MSK << conf_name##_POS );     \
               *((volatile u32 *) conf_name##_CONFREG ) |= conf_name##_##conf_state << conf_name##_POS;  


/* PERSEUS2_MODE1 bit positions */

#define USB_TRANSCEIVER_SEL_CONFREG     PERSEUS2_MODE_1
#define USB_TRANSCEIVER_SEL_POS         26
#define USB_TRANSCEIVER_SEL_MSK         0x1
#define USB_TRANSCEIVER_SEL_EXT_XCVR    0x0
#define USB_TRANSCEIVER_SEL_INT_XCVR    0x1

#define USB_TRANSCEIVER_SPEED_CONFREG   PERSEUS2_MODE_1
#define USB_TRANSCEIVER_SPEED_POS       15
#define USB_TRANSCEIVER_SPEED_MSK       0x1
#define USB_TRANSCEIVER_SPEED_LOW       0x0
#define USB_TRANSCEIVER_SPEED_HIGH      0x1

/* PERSEUS2_IO_CONF0 bit positions */

#define D_TPU_TSPEN1_CONFREG            PERSEUS2_IO_CONF0
#define D_TPU_TSPEN1_POS                29
#define D_TPU_TSPEN1_MSK                0x07
#define D_TPU_TSPEN1_GPIO_8             0x06

/* PERSEUS2_IO_CONF1 bit positions */

#define D_BB_SIM_PWR_CONFREG            PERSEUS2_IO_CONF1
#define D_BB_SIM_PWR_POS                13
#define D_BB_SIM_PWR_MSK                0x7

#define D_BB_SIM_CD_CONFREG             PERSEUS2_IO_CONF1
#define D_BB_SIM_CD_POS                 17
#define D_BB_SIM_CD_MSK                 0x7

#define D_RFEN_CONFREG                  PERSEUS2_IO_CONF1
#define D_RFEN_POS                      29
#define D_RFEN_MSK                      0x7
#define D_RFEN_GPIO_19                  0x6

/* PERSEUS2_IO_CONF2 bit positions */

#define D_SDMC_CONFREG                  PERSEUS2_IO_CONF2
#define D_SDMC_POS                      9
#define D_SDMC_MSK                      0x7
#define D_SDMC_SDMC_PINS                0x0 // SDMC_CLK, SDMC_CMD, SDMC_DAT0, SDMC_DAT1

#define D_SDMC_DAT2_CONFREG             PERSEUS2_IO_CONF2
#define D_SDMC_DAT2_POS                 13
#define D_SDMC_DAT2_MSK                 0x7
#define D_SDMC_DAT2_SDMC_DAT_2          0x0

#define D_SDMC_DAT3_CONFREG             PERSEUS2_IO_CONF2
#define D_SDMC_DAT3_POS                 17
#define D_SDMC_DAT3_MSK                 0x7
#define D_SDMC_DAT3_SDMC_DAT_3          0x0

#define D_DM_CONFREG                    PERSEUS2_IO_CONF2
#define D_DM_POS                        21
#define D_DM_MSK                        0x7
#define D_DM_OTG                        0x3  // USB_SEO_VM, USB_TXD_VP 

#define D_PU_EN_CONFREG                 PERSEUS2_IO_CONF2
#define D_PU_EN_POS                     25
#define D_PU_EN_MSK                     0x7
#define D_PU_EN_USB_RCV                 0x3

#define D_VBUSI_CONFREG                 PERSEUS2_IO_CONF2
#define D_VBUSI_POS                     29
#define D_VBUSI_MSK                     0x7
#define D_VBUSI_USB_TXEN                0x3

/* PERSEUS2_IO_CONF3 bit positions */

#define D_MCLK_OUT_CONFREG              PERSEUS2_IO_CONF3
#define D_MCLK_OUT_POS                  1
#define D_MCLK_OUT_MSK                  0x7
#define D_MCLK_OUT_GPIO_35              0x6

#define D_CRESET_CONFREG                PERSEUS2_IO_CONF3
#define D_CRESET_POS                    5
#define D_CRESET_MSK                    0x7
#define D_CRESET_GPIO_36                0x6

#define D_UART_IRDA_TX_CONFREG          PERSEUS2_IO_CONF3
#define D_UART_IRDA_TX_POS              9
#define D_UART_IRDA_TX_MSK              0x7
#define D_UART_IRDA_TX_MPU_UART_TX_IR2  0x0

#define D_UART_IRDA_RX_CONFREG          PERSEUS2_IO_CONF3
#define D_UART_IRDA_RX_POS              13
#define D_UART_IRDA_RX_MSK              0x7
#define D_UART_IRDA_RX_MPU_UART_RX_IR2  0x0

#define D_UART_IRDA_SD_CONFREG          PERSEUS2_IO_CONF3
#define D_UART_IRDA_SD_POS              17
#define D_UART_IRDA_SD_MSK              0x7
#define D_UART_IRDA_SD_MPU_UART_SD2     0x0

#define D_UART_TX_RX_CONFREG            PERSEUS2_IO_CONF3
#define D_UART_TX_RX_POS                21
#define D_UART_TX_RX_MSK                0x7
#define D_UART_TX_RX_MPU_UART1          0x0
#define D_UART_TX_RX_MPU_UART2          0x2

#define D_LCD_PXL_15_12_CONFREG         PERSEUS2_IO_CONF3
#define D_LCD_PXL_15_12_POS             29
#define D_LCD_PXL_15_12_MSK             0x7
#define D_LCD_PXL_15_12_LCD_PIXEL       0x0

/* PERSEUS2_IO_CONF4 bit positions */

#define D_LCD_PXL_10_CONFREG            PERSEUS2_IO_CONF4
#define D_LCD_PXL_10_POS                1
#define D_LCD_PXL_10_MSK                0x7
#define D_LCD_PXL_10_LCD_PIXEL          0x0

#define D_LCD_PXL_11_CONFREG            PERSEUS2_IO_CONF4
#define D_LCD_PXL_11_POS                5
#define D_LCD_PXL_11_MSK                0x7
#define D_LCD_PXL_11_LCD_PIXEL          0x0

#define D_LCD_PXL_9_2_CONFREG           PERSEUS2_IO_CONF4
#define D_LCD_PXL_9_2_POS               9
#define D_LCD_PXL_9_2_MSK               0x7
#define D_LCD_PXL_9_2_LCD_PIXEL         0x0

#define D_LCD_UWIRE_CONFREG             PERSEUS2_IO_CONF4
#define D_LCD_UWIRE_POS                 13
#define D_LCD_UWIRE_MSK                 0x7
#define D_LCD_UWIRE_LCD_MISC            0x0

#define D_LCD_VSYNC_CONFREG             PERSEUS2_IO_CONF4
#define D_LCD_VSYNC_POS                 17
#define D_LCD_VSYNC_MSK                 0x7
#define D_LCD_VSYNC_LCD_VSYNC_AC        0x0

#define D_EAC_CDI_CONFREG               PERSEUS2_IO_CONF4
#define D_EAC_CDI_POS                   25
#define D_EAC_CDI_MSK                   0x07
#define D_EAC_CDI_GPIO_67               0x06

/* PERSEUS2_IO_CONF5 bit positions */

#define D_I2C_SDA_CONFREG               PERSEUS2_IO_CONF5
#define D_I2C_SDA_POS                   1
#define D_I2C_SDA_MSK                   0x7
#define D_I2C_SDA_MPU_I2C_SDA           0x0

#define D_I2C_SCK_CONFREG               PERSEUS2_IO_CONF5
#define D_I2C_SCK_POS                   5
#define D_I2C_SCK_MSK                   0x7
#define D_I2C_SCK_MPU_I2C_SCK           0x0

#define D_DDR_CONFREG                   PERSEUS2_IO_CONF5
#define D_DDR_POS                       13
#define D_DDR_MSK                       0x7
#define D_DDR_GPIO_72_73_74             0x6  // GPIO_72, GPIO_73, GPIO_74, 

/* PERSEUS2_IO_CONF9 bit positions */

#define D_SPI1_SEN2_CONFREG             PERSEUS2_IO_CONF9
#define D_SPI1_SEN2_POS                 5
#define D_SPI1_SEN2_MSK                 0x7
#define D_SPI1_SEN2_GPIO_134            0x6

#define D_SMC_IO_CONFREG                PERSEUS2_IO_CONF9
#define D_SMC_IO_POS                    9
#define D_SMC_IO_MSK                    0x7
#define D_SMC_IO_KBR_5                  0x3

#define D_SMC_CLK_CONFREG               PERSEUS2_IO_CONF9
#define D_SMC_CLK_POS                   13
#define D_SMC_CLK_MSK                   0x7
#define D_SMC_CLK_MPU_UW_SCLK           0x5

#define D_SMC_RST_CONFREG               PERSEUS2_IO_CONF9  
#define D_SMC_RST_POS                   17
#define D_SMC_RST_MSK                   0x7
#define D_SMC_RST_MPU_UW_SDO            0x5

#define D_SMC_CD_CONFREG                PERSEUS2_IO_CONF9
#define D_SMC_CD_POS                    21
#define D_SMC_CD_MSK                    0x7
#define D_SMC_CD_MPU_UW_nSCS1           0x5

#define D_MPU_NIRQ_CONFREG              PERSEUS2_IO_CONF9
#define D_MPU_NIRQ_POS                  29
#define D_MPU_NIRQ_MSK                  0x7
#define D_MPU_NIRQ_MPU_EXT_NIRQ         0x0

/* PERSEUS2_IO_CONF10 bit positions */

#define D_CLK13MREQ_CONFREG             PERSEUS2_IO_CONF10
#define D_CLK13MREQ_POS                 29
#define D_CLK13MREQ_MSK                 0x7
#define D_CLK13MREQ_GPIO_145            0x6

/* PERSEUS2_IO_CONF11 bit positions */

#define D_CAM_LCLK_CONFREG		PERSEUS2_IO_CONF11
#define D_CAM_LCLK_POS			1
#define D_CAM_LCLK_MSK			0x7
#define D_CAM_LCLK_CAM_LCLK		0x0
#define D_CAM_LCLK_GPIO_146		0x6

#define D_CAM_EXCLK_CONFREG		PERSEUS2_IO_CONF11
#define D_CAM_EXCLK_POS			5
#define D_CAM_EXCLK_MSK			0x7
#define D_CAM_EXCLK_CAM_EXCLK		0x0
#define D_CAM_EXCLK_GPIO_147		0x6

#define D_CAM_HS_CONFREG		PERSEUS2_IO_CONF11
#define D_CAM_HS_POS			9
#define D_CAM_HS_MSK			0x7
#define D_CAM_HS_CAM_HS			0x0
#define D_CAM_HS_GPIO_148		0x6

#define D_CAM_VS_CONFREG		PERSEUS2_IO_CONF11
#define D_CAM_VS_POS			13
#define D_CAM_VS_MSK			0x7
#define D_CAM_VS_CAM_VS			0x0
#define D_CAM_VS_GPIO_149		0x6

#define D_CAM_RSTZ_CONFREG		PERSEUS2_IO_CONF11
#define D_CAM_RSTZ_POS			17
#define D_CAM_RSTZ_MSK			0x7
#define D_CAM_RSTZ_CAM_RSTZ		0x0
#define D_CAM_RSTZ_GPIO_150		0x6

#define D_CAM_DAT0_CONFREG		PERSEUS2_IO_CONF11
#define D_CAM_DAT0_POS			21
#define D_CAM_DAT0_MSK			0x7
#define D_CAM_DAT0_CAM_DATA_0		0x0
#define D_CAM_DAT0_GPIO_151		0x6

#define D_CAM_DAT1_CONFREG		PERSEUS2_IO_CONF11
#define D_CAM_DAT1_POS			25
#define D_CAM_DAT1_MSK			0x7
#define D_CAM_DAT1_CAM_DATA_1		0x0
#define D_CAM_DAT1_GPIO_152		0x6

#define D_CAM_DAT2_CONFREG		PERSEUS2_IO_CONF11
#define D_CAM_DAT2_POS			29
#define D_CAM_DAT2_MSK			0x7
#define D_CAM_DAT2_CAM_DATA_2		0x0
#define D_CAM_DAT2_GPIO_153		0x6

/* PERSEUS2_IO_CONF12 bit positions */

#define D_CAM_DAT3_CONFREG		PERSEUS2_IO_CONF12
#define D_CAM_DAT3_POS			1
#define D_CAM_DAT3_MSK			0x7
#define D_CAM_DAT3_CAM_DATA_3		0x0
#define D_CAM_DAT3_GPIO_154		0x6

#define D_CAM_DAT4_CONFREG		PERSEUS2_IO_CONF12
#define D_CAM_DAT4_POS			5
#define D_CAM_DAT4_MSK			0x7
#define D_CAM_DAT4_CAM_DATA_4		0x0
#define D_CAM_DAT4_GPIO_155		0x6

#define D_CAM_DAT5_CONFREG		PERSEUS2_IO_CONF12
#define D_CAM_DAT5_POS			9
#define D_CAM_DAT5_MSK			0x7
#define D_CAM_DAT5_CAM_DATA_5		0x0
#define D_CAM_DAT5_GPIO_156		0x6

#define D_CAM_DAT6_CONFREG		PERSEUS2_IO_CONF12
#define D_CAM_DAT6_POS			13
#define D_CAM_DAT6_MSK			0x7
#define D_CAM_DAT6_CAM_DATA_6		0x0
#define D_CAM_DAT6_GPIO_157		0x6

#define D_CAM_DAT7_CONFREG		PERSEUS2_IO_CONF12
#define D_CAM_DAT7_POS			17
#define D_CAM_DAT7_MSK			0x7
#define D_CAM_DAT7_CAM_DATA_7		0x0
#define D_CAM_DAT7_GPIO_158		0x6

#define D_KB0_CONFREG                   PERSEUS2_IO_CONF12
#define D_KB0_POS                       21
#define D_KB0_MSK                       0x7
#define D_KB0_KBR_0                     0x0

#define D_KB1_CONFREG                   PERSEUS2_IO_CONF12
#define D_KB1_POS                       25
#define D_KB1_MSK                       0x7
#define D_KB1_KBR_1                     0x0

#define D_KB2_CONFREG                   PERSEUS2_IO_CONF12
#define D_KB2_POS                       29
#define D_KB2_MSK                       0x7
#define D_KB2_KBR_2                     0x0

/* PERSEUS2_IO_CONF13 bit positions */

#define D_KB3_CONFREG                   PERSEUS2_IO_CONF13
#define D_KB3_POS                       1 
#define D_KB3_MSK                       0x7
#define D_KB3_KBR_3                     0x0

#define D_KB4_CONFREG                   PERSEUS2_IO_CONF13
#define D_KB4_POS                       5
#define D_KB4_MSK                       0x7
#define D_KB4_KBR_4                     0x0

#define D_KB5_CONFREG                   PERSEUS2_IO_CONF13
#define D_KB5_POS                       9
#define D_KB5_MSK                       0x7
#define D_KB5_KBC_0                     0x0

#define D_KB6_CONFREG                   PERSEUS2_IO_CONF13
#define D_KB6_POS                       13
#define D_KB6_MSK                       0x7
#define D_KB6_KBC_1                     0x0

#define D_KB7_CONFREG                   PERSEUS2_IO_CONF13
#define D_KB7_POS                       17
#define D_KB7_MSK                       0x7
#define D_KB7_KBC_2                     0x0

#define D_KB8_CONFREG                   PERSEUS2_IO_CONF13
#define D_KB8_POS                       21
#define D_KB8_MSK                       0x7
#define D_KB8_KBC_3                     0x0

#define D_KB9_CONFREG                   PERSEUS2_IO_CONF13
#define D_KB9_POS                       25
#define D_KB9_MSK                       0x7
#define D_KB9_KBC_4                     0x0

/* PERSEUS2_PCC_CONF_REG Bit Positions */

#define PCONF_MMC_DPLL_REQ_CONFREG      PERSEUS_PCC_CONF_REG
#define PCONF_MMC_DPLL_REQ_POS          2
#define PCONF_MMC_DPLL_REQ_MSK          0x1
#define PCONF_MMC_DPLL_REQ_INACTIVE     0x0
#define PCONF_MMC_DPLL_REQ_ACTIVE       0x1

#define UART1_DPLL_REQ_CONFREG          PERSEUS_PCC_CONF_REG
#define UART1_DPLL_REQ_POS              3
#define UART1_DPLL_REQ_MSK              0x1
#define UART1_DPLL_REQ_INACTIVE         0x0
#define UART1_DPLL_REQ_ACTIVE           0x1

#define UART3_DPLL_REQ_CONFREG          PERSEUS_PCC_CONF_REG
#define UART3_DPLL_REQ_POS              4
#define UART3_DPLL_REQ_MSK              0x1
#define UART3_DPLL_REQ_INACTIVE         0x0
#define UART3_DPLL_REQ_ACTIVE           0x1

#define PCC_CAM_CLK_REQ_CONFREG		PERSEUS_PCC_CONF_REG
#define PCC_CAM_CLK_REQ_POS		7
#define PCC_CAM_CLK_REQ_MSK		0x1
#define PCC_CAM_CLK_REQ_INACTIVE	0x0
#define PCC_CAM_CLK_REQ_ACTIVE		0x1

#endif   /* ! _INCLUDED_ASM_ARCH_OMAP730_CONFIG_H_ */
