/*
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
 *
 * Copyright (C) 2002 Motorola Semiconductors HK Ltd
 * Copyright (c) 2003 MontaVista Software Inc. <source@mvista.com>
 *
 */

#ifndef INC_MX2
#define INC_MX2

/* ep0 states */
#define WAIT_FOR_SETUP          0
#define DATA_STATE_XMIT         1
#define DATA_STATE_NEED_ZLP     2
#define WAIT_FOR_OUT_STATUS     3

#define EP0_PACKETSIZE  0x8

#define UDC_MAX_ENDPOINTS       6

#define UDC_NAME        "MX2 dragonball USBD"

#define STORAGE_BLOCK_SIZE	512

#define	TRUE							1
#define	FALSE							0

#define BIT0	0x00000001
#define BIT1	0x00000002
#define BIT2	0x00000004
#define BIT3	0x00000008
#define BIT4	0x00000010
#define BIT5	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9	0x00000200
#define BIT10	0x00000400
#define BIT11	0x00000800
#define BIT12	0x00001000
#define BIT13	0x00002000
#define BIT14	0x00004000
#define BIT15	0x00008000
#define BIT16	0x00010000
#define BIT17	0x00020000
#define BIT18	0x00040000
#define BIT19	0x00080000
#define BIT20	0x00100000
#define BIT21	0x00200000
#define BIT22	0x00400000
#define BIT23	0x00800000
#define BIT24	0x01000000
#define BIT25	0x02000000
#define BIT26	0x04000000
#define BIT27	0x08000000
#define BIT28	0x10000000
#define BIT29	0x20000000
#define BIT30	0x40000000
#define BIT31	0x80000000

#define ENABLE  0x1
#define DISABLE 0x0

#define DELAY_VALUE 0x10000

#define EP_OUT_DIR       0
#define EP_IN_DIR        1
#define EP_NO_STALL      0
#define EP_STALL         1
#define EP_NO_SETUP      0
#define EP_SETUP         1

#define EP0OUT	BIT0
#define EP0IN	BIT1
#define EP1OUT	BIT2
#define EP1IN	BIT3
#define EP2OUT	BIT4
#define EP2IN	BIT5
#define EP3OUT	BIT6
#define EP3IN	BIT7
#define EP4OUT	BIT8
#define EP4IN	BIT9
#define EP5OUT	BIT10
#define EP5IN	BIT11
#define EP6OUT	BIT12
#define EP6IN	BIT13
#define EP7OUT	BIT14
#define EP7IN	BIT15
#define EP8OUT	BIT16
#define EP8IN	BIT17
#define EP9OUT	BIT18
#define EP9IN	BIT19
#define EP10OUT	BIT20
#define EP10IN	BIT21
#define EP11OUT	BIT22
#define EP11IN	BIT23
#define EP12OUT	BIT24
#define EP12IN	BIT25
#define EP13OUT	BIT26
#define EP13IN	BIT27
#define EP14OUT	BIT28
#define EP14IN	BIT29
#define EP15OUT	BIT30
#define EP15IN	BIT31

#define CTL_TYPE         0x0
#define BLK_TYPE         0x2

#define EP0 0x0
#define EP1 0x1
#define EP2 0x2

#define I2C_MASK_DP_PULLUP      0x01
#define I2C_MASK_DM_PULLUP      0x02
#define I2C_MASK_DP_PULLDOWN    0x04
#define I2C_MASK_DM_PULLDOWN    0x08

#define TIMEOUT_VALUE 100000

#define I2C_DEV_ADDR	0x2D
#define I2C_XCVR_LOWADR         0x02
#define I2C_TEST_SCLK_SCL             0xF0	/* configure as 100kHz */
#define I2C_TEST_SINGLE_WRITE         0x0
#define I2C_TEST_DOUBLE_WRITE         0x1
#define I2C_TEST_BLOCK_WRITE          0x2
#define I2C_TEST_RANDOM_WRITE         0x3

#define VENDOR_ID_REG0_ADD      _OTG_I2C_BASE+0x00
#define VENDOR_ID_REG1_ADD      _OTG_I2C_BASE+0x01
#define PRODUCT_ID_REG0_ADD     _OTG_I2C_BASE+0x02
#define PRODUCT_ID_REG1_ADD     _OTG_I2C_BASE+0x03
#define MODE_REG1_SET_ADD       _OTG_I2C_BASE+0x04
#define MODE_REG1_CLR_ADD       _OTG_I2C_BASE+0x05
#define OTG_CTRL_REG1_SET_ADD   _OTG_I2C_BASE+0x06
#define OTG_CTRL_REG1_CLR_ADD   _OTG_I2C_BASE+0x07
#define INT_SRC_REG_ADD         _OTG_I2C_BASE+0x08
#define INT_LAT_REG_SET_ADD     _OTG_I2C_BASE+0x0a
#define INT_LAT_REG_CLR_ADD     _OTG_I2C_BASE+0x0b
#define INT_FALSE_REG_SET_ADD   _OTG_I2C_BASE+0x0c
#define INT_FALSE_REG_CLR_ADD   _OTG_I2C_BASE+0x0d
#define INT_TRUE_REG_SET_ADD    _OTG_I2C_BASE+0x0e
#define INT_TRUE_REG_CLR_ADD    _OTG_I2C_BASE+0x0f
#define OTG_CTRL_REG2_SET_ADD   _OTG_I2C_BASE+0x10
#define OTG_CTRL_REG2_CLR_ADD   _OTG_I2C_BASE+0x11
#define MODE_REG2_SET_ADD       _OTG_I2C_BASE+0x12
#define MODE_REG2_CLR_ADD       _OTG_I2C_BASE+0x13
#define BCD_DEV_REG0_ADD        _OTG_I2C_BASE+0x14
#define BCD_DEV_REG1_ADD        _OTG_I2C_BASE+0x15

#define BYTECOUNT 512
#define BYTECOUNTMASK 0x001fffff

#define OTG_FUNC_DWORD0_EP0OUT        __REG32((OTG_EP_BASE+16*(0*2 + 0)))
#define OTG_FUNC_DWORD0_EP0IN         __REG32((OTG_EP_BASE+16*(0*2 + 1)))
#define OTG_FUNC_DWORD0_EP1OUT        __REG32((OTG_EP_BASE+16*(1*2 + 0)))
#define OTG_FUNC_DWORD0_EP1IN         __REG32((OTG_EP_BASE+16*(1*2 + 1)))
#define OTG_FUNC_DWORD0_EP2OUT        __REG32((OTG_EP_BASE+16*(2*2 + 0)))
#define OTG_FUNC_DWORD0_EP2IN         __REG32((OTG_EP_BASE+16*(2*2 + 1)))

#define OTG_FUNC_DWORD3_EP0OUT        __REG32((OTG_EP_BASE+16*(0*2 + 0)+12))
#define OTG_FUNC_DWORD3_EP0IN         __REG32((OTG_EP_BASE+16*(0*2 + 1)+12))
#define OTG_FUNC_DWORD3_EP1OUT        __REG32((OTG_EP_BASE+16*(1*2 + 0)+12))
#define OTG_FUNC_DWORD3_EP1IN         __REG32((OTG_EP_BASE+16*(1*2 + 1)+12))
#define OTG_FUNC_DWORD3_EP2OUT        __REG32((OTG_EP_BASE+16*(2*2 + 0)+12))
#define OTG_FUNC_DWORD3_EP2IN         __REG32((OTG_EP_BASE+16*(2*2 + 1)+12))

#define OTG_DMA_EP1_I_MSA               __REG32((OTG_DMA_BASE+0x18C))
#define OTG_DMA_EP1_O_MSA               __REG32((OTG_DMA_BASE+0x188))
#define OTG_DMA_EP2_I_MSA               __REG32((OTG_DMA_BASE+0x194))
#define OTG_DMA_EP2_O_MSA               __REG32((OTG_DMA_BASE+0x190))

/*            Function related tasks          */

void WriteEPToggleBit(u8 ep, u8 dir, u8 toggle_val);
void Enable_EP(u8 ep_num);
u8 check_ep_ready(u8 qEpNumberDirection);
void Ready_EP_IN(u8 ep);
void Ready_EP_OUT(u8 ep);
void unready_ep_in(u8 ep);
void unready_ep_out(u8 ep);
void Clear_active_ep(u8 ep_num);

/*     Common tasks       */
void SetOtgPort_FunctionMode(void);
void OtgPort_ExtPullUpDisabled(void);
void OtgPort_PullUpEnabled(void);

/*     i2c tasks         */
void i2cDpPullUp(void);

void i2cSetSeqOpReg(u8 qWrData);
void i2cCheckBusy(void);
void i2cSetSeqRdStartAd(u8 qWrData);
void i2cSingleRegWrite(u32 qRegAddr, u8 qWrData);
void ToggleNextBufferToService(u8 mask);
void ClearXYBufferInt(void);
void EP0OutForRequest(void);

#endif
