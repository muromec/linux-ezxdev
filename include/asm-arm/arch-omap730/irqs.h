/*
 * linux/include/asm-arm/arch-omap730/irqs.h
 *
 * Copyright (C) 2004, MPC-Data Limited
 *   Dave Peverley <dpeverley@mpc-data.co.uk>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_OMAP730_IRQS_H
#define __ASM_ARCH_OMAP730_IRQS_H

/* ---------------------------------------------------------------------------
 *  Interrupts
 * ---------------------------------------------------------------------------
 */

/* These interrupt vectors are for the OMAP730.
 * The interrupt vectors for the OMAP1509/OMAP1610DC/OMAP1610 are different.
 */

#define IH1_BASE                   0

/* 
 *  IRQ Numbers for INT1
 * 
 */

#define INT_IH2_FIQ                (IH1_BASE + 0)
#define INT_IH2_IRQ                (IH1_BASE + 1)
#define INT_USB_NON_ISO            (IH1_BASE + 2)
#define INT_USB_ISO                (IH1_BASE + 3)
#define INT_ICR                    (IH1_BASE + 4)
#define INT_EAC                    (IH1_BASE + 5)
#define INT_GPIO1                  (IH1_BASE + 6)  /* MUPIO_1 in the TRM : Renamed for compatibility */
#define INT_GPIO2                  (IH1_BASE + 7)  /* MUPIO_2 in the TRM : Renamed for compatibility */
#define INT_GPIO3                  (IH1_BASE + 8)  /* MUPIO_3 in the TRM : Renamed for compatibility */
#define INT_ABORT                  (IH1_BASE + 9) 
#define INT_McBSP2_TX              (IH1_BASE + 10)
#define INT_McBSP2_RX              (IH1_BASE + 11)
#define INT_COM_McBSP2_RO          (IH1_BASE + 12)
#define INT_BRIDGE_PRIV            (IH1_BASE + 13)
#define INT_LCD_LINE               (IH1_BASE + 14)
#define INT_GSM_PROTECT            (IH1_BASE + 15)
#define INT_TIMER3                 (IH1_BASE + 16)
#define INT_GPIO5                  (IH1_BASE + 17)  /* MUPIO_5 in the TRM : Renamed for compatibility */
#define INT_GPIO6                  (IH1_BASE + 18)  /* MUPIO_6 in the TRM : Renamed for compatibility */
#define INT_DMA_CH0                (IH1_BASE + 19)
#define INT_DMA_CH1                (IH1_BASE + 20)
#define INT_DMA_CH2                (IH1_BASE + 21)
#define INT_DMA_CH3                (IH1_BASE + 22)
#define INT_DMA_CH4                (IH1_BASE + 23)
#define INT_DMA_CH5                (IH1_BASE + 24)
#define INT_DMA_LCD                (IH1_BASE + 25)
#define INT_TIMER1                 (IH1_BASE + 26)
#define INT_WD_TIMER               (IH1_BASE + 27)
#define INT_BRIDGE_PUB             (IH1_BASE + 28)
#define INT_SPGIO_WR               (IH1_BASE + 29)
#define INT_TIMER2                 (IH1_BASE + 30)
#define INT_LCD_CTRL               (IH1_BASE + 31)

#define INT_IH1_LAST               INT_LCD_CTRL


/* 
 * IRQ Numbers for Interrupt Handler 2
 */
#define IH2_BASE                   (INT_IH1_LAST + 1)
 
#define INT_HW_ERRORS              (IH2_BASE + 0)
#define INT_PWR_FAIL               (IH2_BASE + 1)
#define INT_CF_CFCD                (IH2_BASE + 2)
#define INT_CF_CFIREQ              (IH2_BASE + 3)
#define INT_I2C                    (IH2_BASE + 4)
#define INT_PCC                    (IH2_BASE + 5)
#define INT_MPU_EXT_NIRQ           (IH2_BASE + 6)
#define INT_SPI_100K_1             (IH2_BASE + 7) 
#define INT_SYSREN_SPI             (IH2_BASE + 8) 
#define INT_VLYNQ                  (IH2_BASE + 9)
#define INT_GPIO4                  (IH2_BASE + 10)  /* MUPIO_4 in the TRM : Renamed for compatibility */
#define INT_McBSP1TX               (IH2_BASE + 11)
#define INT_McBSP1RX               (IH2_BASE + 12)
#define INT_COM_McBSP1_RO          (IH2_BASE + 13)
#define INT_UART2                  (IH2_BASE + 14)
#define INT_UART1                  (IH2_BASE + 15)
#define INT_BT_MCS1TX              (IH2_BASE + 16)
#define INT_uWireTX                (IH2_BASE + 17)
#define INT_uWireRX                (IH2_BASE + 18)
#define INT_SMC_CD                 (IH2_BASE + 19)
#define INT_SMC_IREQ               (IH2_BASE + 20)
#define INT_1WIRE                  (IH2_BASE + 21)
#define INT_OS_32kHz_TIMER         (IH2_BASE + 22)
#define INT_MMC_SDIO1              (IH2_BASE + 23)
#define INT_GAUGE_32K              (IH2_BASE + 24)
#define INT_RTC_TIMER              (IH2_BASE + 25)
#define INT_RTC_ALARM              (IH2_BASE + 26)
#define INT_USB_HHC1               (IH2_BASE + 27)
#define INT_USB_HHC2               (IH2_BASE + 28)
#define INT_USB_Geni               (IH2_BASE + 29)
#define INT_USB_OTG                (IH2_BASE + 30)
#define INT_CAMERA                 (IH2_BASE + 31)
#define INT_RNG                    (IH2_BASE + 32)
#define INT_DMODE_TIMER            (IH2_BASE + 33)
#define INT_DBB_RF_EN              (IH2_BASE + 34)
#define INT_KEYBOARD               (IH2_BASE + 35)
#define INT_SHA1_MD5               (IH2_BASE + 36)
#define INT_SPI_100K_2             (IH2_BASE + 37)
#define INT_RNG_IDLE_MODE          (IH2_BASE + 38)
#define INT_MPUIO                  (IH2_BASE + 39)  /* ARMIO_GPIO in the TRM : Renamed for compatibility */
#define INT_RES40                  (IH2_BASE + 40)
#define INT_RES41                  (IH2_BASE + 41)
#define INT_RES42                  (IH2_BASE + 42)
#define INT_RES43                  (IH2_BASE + 43)
#define INT_RES44                  (IH2_BASE + 44)
#define INT_RES45                  (IH2_BASE + 45)
#define INT_PERIF_WAKEUP           (IH2_BASE + 46)
#define INT_RES47                  (IH2_BASE + 47)
#define INT_RES48                  (IH2_BASE + 48)
#define INT_RES49                  (IH2_BASE + 49)
#define INT_RES50                  (IH2_BASE + 50)
#define INT_RES51                  (IH2_BASE + 51)
#define INT_RES52                  (IH2_BASE + 52)
#define INT_DMA_CH6                (IH2_BASE + 53)
#define INT_DMA_CH7                (IH2_BASE + 54)
#define INT_DMA_CH8                (IH2_BASE + 55)
#define INT_DMA_CH9                (IH2_BASE + 56)
#define INT_DMA_CH10               (IH2_BASE + 57)
#define INT_DMA_CH11               (IH2_BASE + 58)
#define INT_DMA_CH12               (IH2_BASE + 59)
#define INT_DMA_CH13               (IH2_BASE + 60)
#define INT_DMA_CH14               (IH2_BASE + 61)
#define INT_DMA_CH15               (IH2_BASE + 62)
#define INT_NAND_FLASH             (IH2_BASE + 63)

#define INT_IH2_LAST               INT_NAND_FLASH


/*
 * IRQ Numbers for interrupts muxed through MPU GPIO
 */

#define IH_GPIO_BASE               (INT_IH2_LAST + 1)

#define INT_GPIO_0                 (IH_GPIO_BASE + 0)
#define INT_GPIO_1                 (IH_GPIO_BASE + 1)
#define INT_GPIO_2                 (IH_GPIO_BASE + 2)
#define INT_GPIO_3                 (IH_GPIO_BASE + 3)
#define INT_GPIO_4                 (IH_GPIO_BASE + 4)
#define INT_GPIO_5                 (IH_GPIO_BASE + 5)
#define INT_GPIO_6                 (IH_GPIO_BASE + 6)
#define INT_GPIO_7                 (IH_GPIO_BASE + 7)
#define INT_GPIO_8                 (IH_GPIO_BASE + 8)
#define INT_GPIO_9                 (IH_GPIO_BASE + 9)
#define INT_GPIO_10                (IH_GPIO_BASE + 10)
#define INT_GPIO_11                (IH_GPIO_BASE + 11)
#define INT_GPIO_12                (IH_GPIO_BASE + 12)
#define INT_GPIO_13                (IH_GPIO_BASE + 13)
#define INT_GPIO_14                (IH_GPIO_BASE + 14)
#define INT_GPIO_15                (IH_GPIO_BASE + 15)
#define INT_GPIO_16                (IH_GPIO_BASE + 16)
#define INT_GPIO_17                (IH_GPIO_BASE + 17)
#define INT_GPIO_18                (IH_GPIO_BASE + 18)
#define INT_GPIO_19                (IH_GPIO_BASE + 19)
#define INT_GPIO_20                (IH_GPIO_BASE + 20)
#define INT_GPIO_21                (IH_GPIO_BASE + 21)
#define INT_GPIO_22                (IH_GPIO_BASE + 22)
#define INT_GPIO_23                (IH_GPIO_BASE + 23)
#define INT_GPIO_24                (IH_GPIO_BASE + 24)
#define INT_GPIO_25                (IH_GPIO_BASE + 25)
#define INT_GPIO_26                (IH_GPIO_BASE + 26)
#define INT_GPIO_27                (IH_GPIO_BASE + 27)
#define INT_GPIO_28                (IH_GPIO_BASE + 28)
#define INT_GPIO_29                (IH_GPIO_BASE + 29)
#define INT_GPIO_30                (IH_GPIO_BASE + 30)
#define INT_GPIO_31                (IH_GPIO_BASE + 31)
#define INT_GPIO_32                (IH_GPIO_BASE + 32)
#define INT_GPIO_33                (IH_GPIO_BASE + 33)
#define INT_GPIO_34                (IH_GPIO_BASE + 34)
#define INT_GPIO_35                (IH_GPIO_BASE + 35)
#define INT_GPIO_36                (IH_GPIO_BASE + 36)
#define INT_GPIO_37                (IH_GPIO_BASE + 37)
#define INT_GPIO_38                (IH_GPIO_BASE + 38)
#define INT_GPIO_39                (IH_GPIO_BASE + 39)
#define INT_GPIO_40                (IH_GPIO_BASE + 40)
#define INT_GPIO_41                (IH_GPIO_BASE + 41)
#define INT_GPIO_42                (IH_GPIO_BASE + 42)
#define INT_GPIO_43                (IH_GPIO_BASE + 43)
#define INT_GPIO_44                (IH_GPIO_BASE + 44)
#define INT_GPIO_45                (IH_GPIO_BASE + 45)
#define INT_GPIO_46                (IH_GPIO_BASE + 46)
#define INT_GPIO_47                (IH_GPIO_BASE + 47)
#define INT_GPIO_48                (IH_GPIO_BASE + 48)
#define INT_GPIO_49                (IH_GPIO_BASE + 49)
#define INT_GPIO_50                (IH_GPIO_BASE + 50)
#define INT_GPIO_51                (IH_GPIO_BASE + 51)
#define INT_GPIO_52                (IH_GPIO_BASE + 52)
#define INT_GPIO_53                (IH_GPIO_BASE + 53)
#define INT_GPIO_54                (IH_GPIO_BASE + 54)
#define INT_GPIO_55                (IH_GPIO_BASE + 55)
#define INT_GPIO_56                (IH_GPIO_BASE + 56)
#define INT_GPIO_57                (IH_GPIO_BASE + 57)
#define INT_GPIO_58                (IH_GPIO_BASE + 58)
#define INT_GPIO_59                (IH_GPIO_BASE + 59)
#define INT_GPIO_60                (IH_GPIO_BASE + 60)
#define INT_GPIO_61                (IH_GPIO_BASE + 61)
#define INT_GPIO_62                (IH_GPIO_BASE + 62)
#define INT_GPIO_63                (IH_GPIO_BASE + 63)
#define INT_GPIO_64                (IH_GPIO_BASE + 64)
#define INT_GPIO_65                (IH_GPIO_BASE + 65)
#define INT_GPIO_66                (IH_GPIO_BASE + 66)
#define INT_GPIO_67                (IH_GPIO_BASE + 67)
#define INT_GPIO_68                (IH_GPIO_BASE + 68)
#define INT_GPIO_69                (IH_GPIO_BASE + 69)
#define INT_GPIO_70                (IH_GPIO_BASE + 70)
#define INT_GPIO_71                (IH_GPIO_BASE + 71)
#define INT_GPIO_72                (IH_GPIO_BASE + 72)
#define INT_GPIO_73                (IH_GPIO_BASE + 73)
#define INT_GPIO_74                (IH_GPIO_BASE + 74)
#define INT_GPIO_75                (IH_GPIO_BASE + 75)
#define INT_GPIO_76                (IH_GPIO_BASE + 76)
#define INT_GPIO_77                (IH_GPIO_BASE + 77)
#define INT_GPIO_78                (IH_GPIO_BASE + 78)
#define INT_GPIO_79                (IH_GPIO_BASE + 79)
#define INT_GPIO_80                (IH_GPIO_BASE + 80)
#define INT_GPIO_81                (IH_GPIO_BASE + 81)
#define INT_GPIO_82                (IH_GPIO_BASE + 82)
#define INT_GPIO_83                (IH_GPIO_BASE + 83)
#define INT_GPIO_84                (IH_GPIO_BASE + 84)
#define INT_GPIO_85                (IH_GPIO_BASE + 85)
#define INT_GPIO_86                (IH_GPIO_BASE + 86)
#define INT_GPIO_87                (IH_GPIO_BASE + 87)
#define INT_GPIO_88                (IH_GPIO_BASE + 88)
#define INT_GPIO_89                (IH_GPIO_BASE + 89)
#define INT_GPIO_90                (IH_GPIO_BASE + 90)
#define INT_GPIO_91                (IH_GPIO_BASE + 91)
#define INT_GPIO_92                (IH_GPIO_BASE + 92)
#define INT_GPIO_93                (IH_GPIO_BASE + 93)
#define INT_GPIO_94                (IH_GPIO_BASE + 94)
#define INT_GPIO_95                (IH_GPIO_BASE + 95)
#define INT_GPIO_96                (IH_GPIO_BASE + 96)
#define INT_GPIO_97                (IH_GPIO_BASE + 97)
#define INT_GPIO_98                (IH_GPIO_BASE + 98)
#define INT_GPIO_99                (IH_GPIO_BASE + 99)
#define INT_GPIO_100               (IH_GPIO_BASE + 100)
#define INT_GPIO_101               (IH_GPIO_BASE + 101)
#define INT_GPIO_102               (IH_GPIO_BASE + 102)
#define INT_GPIO_103               (IH_GPIO_BASE + 103)
#define INT_GPIO_104               (IH_GPIO_BASE + 104)
#define INT_GPIO_105               (IH_GPIO_BASE + 105)
#define INT_GPIO_106               (IH_GPIO_BASE + 106)
#define INT_GPIO_107               (IH_GPIO_BASE + 107)
#define INT_GPIO_108               (IH_GPIO_BASE + 108)
#define INT_GPIO_109               (IH_GPIO_BASE + 109)
#define INT_GPIO_110               (IH_GPIO_BASE + 110)
#define INT_GPIO_111               (IH_GPIO_BASE + 111)
#define INT_GPIO_112               (IH_GPIO_BASE + 112)
#define INT_GPIO_113               (IH_GPIO_BASE + 113)
#define INT_GPIO_114               (IH_GPIO_BASE + 114)
#define INT_GPIO_115               (IH_GPIO_BASE + 115)
#define INT_GPIO_116               (IH_GPIO_BASE + 116)
#define INT_GPIO_117               (IH_GPIO_BASE + 117)
#define INT_GPIO_118               (IH_GPIO_BASE + 118)
#define INT_GPIO_119               (IH_GPIO_BASE + 119)
#define INT_GPIO_120               (IH_GPIO_BASE + 120)
#define INT_GPIO_121               (IH_GPIO_BASE + 121)
#define INT_GPIO_122               (IH_GPIO_BASE + 122)
#define INT_GPIO_123               (IH_GPIO_BASE + 123)
#define INT_GPIO_124               (IH_GPIO_BASE + 124)
#define INT_GPIO_125               (IH_GPIO_BASE + 125)
#define INT_GPIO_126               (IH_GPIO_BASE + 126)
#define INT_GPIO_127               (IH_GPIO_BASE + 127)
#define INT_GPIO_128               (IH_GPIO_BASE + 128)
#define INT_GPIO_129               (IH_GPIO_BASE + 129)
#define INT_GPIO_130               (IH_GPIO_BASE + 130)
#define INT_GPIO_131               (IH_GPIO_BASE + 131)
#define INT_GPIO_132               (IH_GPIO_BASE + 132)
#define INT_GPIO_133               (IH_GPIO_BASE + 133)
#define INT_GPIO_134               (IH_GPIO_BASE + 134)
#define INT_GPIO_135               (IH_GPIO_BASE + 135)
#define INT_GPIO_136               (IH_GPIO_BASE + 136)
#define INT_GPIO_137               (IH_GPIO_BASE + 137)
#define INT_GPIO_138               (IH_GPIO_BASE + 138)
#define INT_GPIO_139               (IH_GPIO_BASE + 139)
#define INT_GPIO_140               (IH_GPIO_BASE + 140)
#define INT_GPIO_141               (IH_GPIO_BASE + 141)
#define INT_GPIO_142               (IH_GPIO_BASE + 142)
#define INT_GPIO_143               (IH_GPIO_BASE + 143)
#define INT_GPIO_144               (IH_GPIO_BASE + 144)
#define INT_GPIO_145               (IH_GPIO_BASE + 145)
#define INT_GPIO_146               (IH_GPIO_BASE + 146)
#define INT_GPIO_147               (IH_GPIO_BASE + 147)
#define INT_GPIO_148               (IH_GPIO_BASE + 148)
#define INT_GPIO_149               (IH_GPIO_BASE + 149)
#define INT_GPIO_150               (IH_GPIO_BASE + 150)
#define INT_GPIO_151               (IH_GPIO_BASE + 151)
#define INT_GPIO_152               (IH_GPIO_BASE + 152)
#define INT_GPIO_153               (IH_GPIO_BASE + 153)
#define INT_GPIO_154               (IH_GPIO_BASE + 154)
#define INT_GPIO_155               (IH_GPIO_BASE + 155)
#define INT_GPIO_156               (IH_GPIO_BASE + 156)
#define INT_GPIO_157               (IH_GPIO_BASE + 157)
#define INT_GPIO_158               (IH_GPIO_BASE + 158)
#define INT_GPIO_159               (IH_GPIO_BASE + 159)
#define INT_GPIO_160               (IH_GPIO_BASE + 160)
#define INT_GPIO_161               (IH_GPIO_BASE + 161)
#define INT_GPIO_162               (IH_GPIO_BASE + 162)
#define INT_GPIO_163               (IH_GPIO_BASE + 163)
#define INT_GPIO_164               (IH_GPIO_BASE + 164)
#define INT_GPIO_165               (IH_GPIO_BASE + 165)
#define INT_GPIO_166               (IH_GPIO_BASE + 166)
#define INT_GPIO_167               (IH_GPIO_BASE + 167)
#define INT_GPIO_168               (IH_GPIO_BASE + 168)
#define INT_GPIO_169               (IH_GPIO_BASE + 169)
#define INT_GPIO_170               (IH_GPIO_BASE + 170)
#define INT_GPIO_171               (IH_GPIO_BASE + 171)
#define INT_GPIO_172               (IH_GPIO_BASE + 172)
#define INT_GPIO_173               (IH_GPIO_BASE + 173)
#define INT_GPIO_174               (IH_GPIO_BASE + 174)
#define INT_GPIO_175               (IH_GPIO_BASE + 175)
#define INT_GPIO_176               (IH_GPIO_BASE + 176)
#define INT_GPIO_177               (IH_GPIO_BASE + 177)
#define INT_GPIO_178               (IH_GPIO_BASE + 178)
#define INT_GPIO_179               (IH_GPIO_BASE + 179)
#define INT_GPIO_180               (IH_GPIO_BASE + 180)
#define INT_GPIO_181               (IH_GPIO_BASE + 181)
#define INT_GPIO_182               (IH_GPIO_BASE + 182)
#define INT_GPIO_183               (IH_GPIO_BASE + 183)
#define INT_GPIO_184               (IH_GPIO_BASE + 184)
#define INT_GPIO_185               (IH_GPIO_BASE + 185)
#define INT_GPIO_186               (IH_GPIO_BASE + 186)
#define INT_GPIO_187               (IH_GPIO_BASE + 187)
#define INT_GPIO_188               (IH_GPIO_BASE + 188)
#define INT_GPIO_189               (IH_GPIO_BASE + 189)
#define INT_GPIO_190               (IH_GPIO_BASE + 190)
#define INT_GPIO_191               (IH_GPIO_BASE + 191)

#define INT_GPIO_LAST              INT_GPIO_191

/*
 * IRQ Numbers for interrupts muxed through MPUIO
 */

#define IH_MPUIO_BASE              (INT_GPIO_LAST + 1)

#define INT_MPUIO_0                (IH_MPUIO_BASE + 0)   /* Only availiable after MUX conf */
#define INT_MPUIO_1                (IH_MPUIO_BASE + 1)
#define INT_MPUIO_2                (IH_MPUIO_BASE + 2)
#define INT_MPUIO_3                (IH_MPUIO_BASE + 3)
#define INT_MPUIO_4                (IH_MPUIO_BASE + 4)
#define INT_MPUIO_5                (IH_MPUIO_BASE + 5)
#define INT_MPUIO_6                (IH_MPUIO_BASE + 6)   /* Only availiable after MUX conf */
#define INT_MPUIO_7                (IH_MPUIO_BASE + 7)   /* Only availiable after MUX conf */
#define INT_MPUIO_RES8             (IH_MPUIO_BASE + 8)   /* RESERVED : Not actually available for use */
#define INT_MPUIO_RES9             (IH_MPUIO_BASE + 9)   /* RESERVED : Not actually available for use */
#define INT_MPUIO_RES10            (IH_MPUIO_BASE + 10)  /* RESERVED : Not actually available for use */
#define INT_MPUIO_11               (IH_MPUIO_BASE + 11)  /* Only availiable after MUX conf */
#define INT_MPUIO_12               (IH_MPUIO_BASE + 12)  /* Only availiable after MUX conf */
#define INT_MPUIO_RES13            (IH_MPUIO_BASE + 13)  /* RESERVED : Not actually available for use */
#define INT_MPUIO_RES14            (IH_MPUIO_BASE + 14)  /* RESERVED : Not actually available for use */
#define INT_MPUIO_RES15            (IH_MPUIO_BASE + 15)  /* RESERVED : Not actually available for use */

#define INT_MPUIO_LAST             INT_MPUIO_RES15


/* Total numbers of irqs... */

#define MAXIRQNUM                  (INT_MPUIO_LAST + 1)
#define MAXFIQNUM                  MAXIRQNUM
#define MAXSWINUM                  MAXIRQNUM

#define NR_IRQS                    (MAXIRQNUM + 1)

#ifndef __ASSEMBLY__
extern void omap730_init_irq(void);
#endif

#endif
