/*
 * usbd/isp_bi/isp.h
 *
 *      Copyright (c) 2005 Motorola
 *
 * By: 
 *      Richard Xiao <a2590c@motorola.com>, 
 *      Yuan Zhiming <a14194@motorola.com>, 
 *      Li Xin <a16157@motorola.com>, 
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __ISP_H__
#define __ISP_H__

/* define endpoint max packet size */
#define EP0_PACKETSIZE           64 
#define USB_FS_INT_MAXPKTSIZE    16 
#define USB_HS_INT_MAXPKTSIZE    16 
#define USB_FS_BULK_MAXPKTSIZE   64
#define USB_HS_BULK_MAXPKTSIZE   512 
#define USB_FS_ISO_MAXPKTSIZE    64
#define USB_HS_ISO_MAXPKTSIZE    512 

/* define supported endpoint number */
#define UDC_MAX_ENDPOINTS     15

/* 
   The endpoint 0 is a dual direction endpoint. 
   Except endpoint 0, funtion driver can use up to 7 IN and 7 OUT endpoints.
 */
#define UDC_MAX_IN_ENDPOINTS      8   /* contain endpoint 0 */
#define UDC_MAX_OUT_ENDPOINTS     8   /* contain endpoint 0 */

/* define high/full speed flag */
#define ISP_FULL_SPEED        0x00
#define ISP_HIGH_SPEED        0x01

/* driver name */
#define UDC_NAME              "ISP1583"

/* isp1583 register physical address */
#define ISP_REGBASE_ADDR      PXA_CS3_PHYS 

/* isp1583 IRQ */
#define IRQ_ISP_USB            (IRQ_GPIO(GPIO_USB20_INT))

/* the following definitions are used for endpoint index access */
#define ISP_EPINDEX_EP0SETUP   0x20
#define ISP_EPINDEX_EP_OFFSET  1
#define ISP_EPINDEX_IN_DIR     1
#define ISP_EPINDEX_OUT_DIR    0

/* the following definitions are used for endpoint type access */
#define ISP_CTLFUN_VENDP       0x10

/* the following definitions are used for endpoint type access */
#define ISP_EPTYPE_BULK        0x16
#define ISP_EPTYPE_INTERRUPT   0x17

/* the following definitions are used for unlock access */
#define ISP_UNLOCK_REGS        0xAA37

/* dma burst register access */
#define ISP_DMA_BURST_LEN      32

/* isp1583 buffer status */
#define ISP_BUFFER_FULL        3

/* define data prot register offset */
#define ISP_DMA_DATA_PORT_OFFSET   0x440

#pragma pack(1)

/* ISP1583 register definition */
typedef union ADDRESS_REG {
	struct ADDRESS_BITS {
		u8 DEVADDR   :  7;
		u8 DEVEN     :  1;
	}__attribute__((packed)) BITS;
	u16 VALUE;
} __attribute__((packed)) ADDRESS_REG;

typedef union USB_MODE {
	struct USB_MODE_BITS {
		u8 SOFTCT      :    1;
		u8 PWRON       :    1;
		u8 WKUPCS      :    1;
		u8 GLINTENA    :    1;
		u8 SFRESET     :    1;
		u8 GOSUSP      :    1;
		u8 SNDRSU      :    1;
		u8 CLKAON      :    1;
		u8 VBUS_STATUS :    1;
		u8 DMACLKON    :    1;
		u8 RESERVED    :    3;
		u8 BUS_CONFIG  :    1;
		u8 MODE0       :    1;
		u8 MODE1       :    1;
	}__attribute__((packed)) BITS;
	u16     VALUE;
}__attribute__((packed)) USB_MODE;

#define ISP_MODE_GLINTENA           0x0008
#define ISP_MODE_MASK_GLINTENA      0xFFF7

typedef union INT_CONFIG {
	struct INT_CONFIG_BITS {
		u8 INTPOL      :    1;
		u8 INTLVL      :    1;
		u8 DDBGMODOUT  :    2;
		u8 DDBGMODIN   :    2;
		u8 CDBGMOD     :    2;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) INT_CONFIG;

typedef union OTG_MODE {
	struct OTG_MODE_BITS {
		u8 OTG         :    1;
		u8 VP          :    1;
		u8 DISCV       :    1;
		u8 INIT        :    1;
		u8 BSESSVALID  :    1;
		u8 DP          :    1;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) OTG_MODE;

typedef union INT_ENABLE_LSB  {
	struct INT_ENABLE_BITS_LSB {
		u8 IERST      :     1;
		u8 IESOF      :     1;
		u8 IEPSOF     :     1;
		u8 IESUSP     :     1;
		u8 IERESM     :     1;
		u8 IEHS_STA   :     1;
		u8 IEDMA      :     1;
		u8 IEVBUS     :     1;
		u8 IEP0SETUP  :     1;
		u8 RESERVED2  :     1;
		u8 IEP0RX     :     1;
		u8 IEP0TX     :     1;      
		u8 IEP1RX     :     1;
		u8 IEP1TX     :     1;      
		u8 IEP2RX     :     1;
		u8 IEP2TX     :     1;      
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) INT_ENABLE_LSB;

typedef union INT_ENABLE_MSB  {
	struct INT_ENABLE_BITS_MSB    {
		u8 IEP3RX     :     1;
		u8 IEP3TX     :     1;
		u8 IEP4RX     :     1;
		u8 IEP4TX     :     1;      
		u8 IEP5RX     :     1;
		u8 IEP5TX     :     1;      
		u8 IEP6RX     :     1;
		u8 IEP6TX     :     1;      
		u8 IEP7RX     :     1;
		u8 IEP7TX     :     1;
		u8 RESERVED1  :     6;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) INT_ENABLE_MSB;

typedef union CONTROL_REG {
	struct CONTROL_BITS {
		u8 STALL      :    1;
		u8 STATUS     :    1;
		u8 DSEN       :    1;
		u8 VENDP      :    1;
		u8 CLBUF      :    1;
		u8 RESERVED1  :    3;
	}__attribute__((packed)) BITS;
	u16      VALUE;
}__attribute__((packed)) CONTROL_REG;


typedef union BUFFER_STATUS {
	struct BUFFER_STATUS_BIT {
		u8 BUF0       :    1;
		u8 BUF1       :    1;
		u8 RES        :    6;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) BUFFER_STATUS;

typedef union ENDPT_MAXSIZE {
	struct ENDPT_MAXSIZE_BITS   {
		u8 FFOSZ7_0   :    8;
		u8 FFOSZ10_8  :    3;
		u8 NTRANS     :    2;
		u8 RESERVED2  :    3;
	}__attribute__((packed)) BITS;
	u16     VALUE;
}__attribute__((packed)) ENDPT_MAXSIZE;

typedef union ENDPT_TYPE {
	struct ENDPT_TYPE_BITS {
		u8 ENDPTYP    :    2;
		u8 DBLBUF     :    1;
		u8 ENABLE     :    1;
		u8 NOEMPKT    :    1;
		u8 RESERVED1  :    3;
		u8 RESRVED2   :    8;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) ENDPT_TYPE;

#define ISP_EPT_NOEMPKT    0x10
#define ISP_EPT_ENABLE     0x08
#define ISP_EPT_DBLBUF     0x04

typedef union DMA_CONFIG {
	struct DMA_CONFIG_BITS {
		u8 WIDTH        :  1;
		u8 DREQ_MODE    :  1;
		u8 MODE         :  2;
		u8 RES1         :  3;
		u8 DIS_XFER_CNT :  1;
		u8 PIO_MODE     :  3;
		u8 DMA_MODE     :  2;
		u8 ATA_MODE     :  1;
		u8 IGNORE_IORDY :  1;
		u8 STREAMING    :  1;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) DMA_CONFIG;

typedef union DMA_HARDWARE {
	struct DMA_HARDWARE_BITS {
		u8 READ_POL   :    1;
		u8 WRITE_POL  :    1;
		u8 DREQ_POL   :    1;
		u8 ACK_POL    :    1;
		u8 MASTER     :    1;
		u8 EOT_POL    :    1;
		u8 ENDIAN     :    2;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) DMA_HARDWARE;

typedef union DMA_STROBE {
	struct DMA_STROBE_BITS {
		u8 DMA_STROBE :    5;
		u8 RES        :    3;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) DMA_STROBE;


typedef union DMA_INT {
	struct DMA_INT_BITS {
		u8 RES                     :       1;
		u8 CMD_INTRQ_OK            :       1;
		u8 TASKFILE_READ_COMPLETE  :       1;
		u8 BSY_DRQ_POLL_DONE       :       1;
		u8 START_READ_1F0_RD_FIFO  :       1;
		u8 RD_1F0_FIFO_EMPTY       :       1;
		u8 WR_1F0_FIFO_FULL        :       1;
		u8 WR_1F0_FIFO_EMPTY       :       1;
		u8 DMA_XFER_OK             :       1;
		u8 INTRQ_SEEN              :       1;
		u8 INT_EOT                 :       1;
		u8 EXT_EOT                 :       1;
		u8 RES1                    :       3;
		u8 TEST3                   :       1;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) DMA_INT;

typedef union DMA_INT_ENABLE {
	struct DMA_INT_ENABLE_BITS {
		u8 RES                        :       1;
		u8 IE_CMD_INTRQ_OK            :       1;
		u8 IE_TASKFILE_READ_COMPLETE  :       1;
		u8 IE_BSY_DRQ_POLL_DONE       :       1;
		u8 IE_START_READ_1F0_RD_FIFO  :       1;
		u8 IE_RD_1F0_FIFO_EMPTY       :       1;
		u8 IE_WR_1F0_FIFO_FULL        :       1;
		u8 IE_WR_1F0_FIFO_EMPTY       :       1;
		u8 IE_DMA_XFER_OK             :       1;
		u8 IE_INTRQ_SEEN              :       1;
		u8 IE_INT_EOT                 :       1;
		u8 IE_EXT_EOT                 :       1;
		u8 RES1                       :       3;
		u8 TEST4                      :       1;
	}__attribute__((packed)) BITS;
    u16     VALUE;
}__attribute__((packed)) DMA_INT_ENABLE;
        
typedef union INTERRUPT_STATUS_LSB {  
	struct INTERRUPT_STATUS_BITS_LSB {
		u8 RESET           :       1;
		u8 SOF             :       1;
		u8 PSOF            :       1;
		u8 SUSP            :       1;
		u8 RESUME          :       1;
		u8 HS_STAT         :       1;
		u8 DMA             :       1;
		u8 VBUS            :       1;
		u8 EP0SETUP        :       1;
		u8 RESERVED2       :       1;
		u8 EP0RX           :       1;
		u8 EP0TX           :       1;
		u8 EP1RX           :       1;
		u8 EP1TX           :       1;      
		u8 EP2RX           :       1;
		u8 EP2TX           :       1;      
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) INTERRUPT_STATUS_LSB;

typedef union INTERRUPT_STATUS_MSB {
	struct INTERRUPT_STATUS_BITS_MSB     {
		u8 EP3RX           :       1;
		u8 EP3TX           :       1;
		u8 EP4RX           :       1;
		u8 EP4TX           :       1;      
		u8 EP5RX           :       1;
		u8 EP5TX           :       1;      
		u8 EP6RX           :       1;
		u8 EP6TX           :       1;      
		u8 EP7RX           :       1;
		u8 EP7TX           :       1;
		u8 RESERVED1       :       6;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) INTERRUPT_STATUS_MSB;

typedef union FRAME_NO   {
	struct FRAME_NO_BITS    {
		u8 SOFL            :       8;
		u8 SOFH            :       3;	
		u8 USOF            :       3;
		u8 RESERVED        :       2;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) FRAME_NO; 

#define ISP_FNO_MASK     	0x07FF
#define ISP_FNO_MICRMASK 	0x3800
#define ISP_FNO_MICROFF  	11


typedef union TESTMODE {    
	struct TESTMODE_BITS {
		u8 SE0_NAK    :    1;
		u8 JSTATE     :    1;
		u8 KSTATE     :    1;
		u8 PRBS       :    1;
		u8 FORCEFS    :    1;
		u8 LPBK       :    1;
		u8 PHYTEST    :    1;
		u8 FORCEHS    :    1;
	}__attribute__((packed)) BITS;
	u16 VALUE;
}__attribute__((packed)) TESTMODE;

#define ISP_DMA_RESET       	0x11
#define ISP_DMA_CLEAR_BUFFER    0x0F
#define ISP_GDMA_Write_Command	0x01
#define ISP_GDMA_Read_Command	0x00


/*
 * For some reason, HW designer connected Bulverde address bus 1 to ISP1583 address bus 0, 2 to 1, ...
 * So the offset must be doubled in software when access any 16-bit word. 
 */
typedef struct D14_CNTRL_REG   {
	/* Offest 0 */
	ADDRESS_REG		D14_ADDRESS;
	u16			rev1;
	u16			DUMMY_01;
	u16			rev2;
	
	/* Offest 8 */
	ENDPT_MAXSIZE		D14_ENDPT_MAXPKTSIZE;
	u16			rev3;
	u16			DUMMY_06;
	u16			rev4;
	
	/* Offest 16 */
	ENDPT_TYPE		D14_ENDPT_TYPE;
	u16			rev5;
	u16			DUMMY_0A;
	u16			rev6;
	
	/* Offest 24 */
	USB_MODE		D14_MODE;
	u16			rev7;
	u16			DUMMY_0E;
	u16			rev8;
	
	/* Offest 32 */
	INT_CONFIG		D14_INT_CONFIG;
	u16			rev9;
	OTG_MODE		D14_OTG_MODE;
	u16			rev10;
	
	/* Offest 40 */
	INT_ENABLE_LSB		D14_INT_ENABLE_LSB;
	u16			rev11;
	INT_ENABLE_MSB		D14_INT_ENABLE_MSB;
	u16			rev12;
	
	/* Offest 48 */
	INTERRUPT_STATUS_LSB	D14_INT_LSB;
	u16			rev13;
	INTERRUPT_STATUS_MSB	D14_INT_MSB;
	u16			rev14;
	
	/* Offest 56 */
	u16			D14_BUFFER_LENGTH;
	u16			rev15;
	
	/* Offest 60 */
	u16			D14_BUFFER_STATUS;
	u16			rev16;
	
	/* Offest 64 */
	u16			D14_DATA_PORT;
	u16			rev17;
	u16			DUMMY_22;
	u16			rev18;
	u16			DUMMY_24;
	u16			rev19;
	u16			DUMMY_26;
	u16			rev20;
	
	/* Offest 80 */
	CONTROL_REG		D14_CONTROL_FUNCTION;
	u16			rev21;
	u16			DUMMY_2A;
	u16			rev22;
	
	/* Offest 88 */
	u16			D14_ENDPT_INDEX;
	u16			rev23;
	u16			DUMMY_2E;
	u16			rev24;
	
	/* Offest 96 */
	u16			D14_DMA_COMMAND;
	u16			rev25;
	u16			DUMMY_32;
	u16			rev26;
	
	/* Offest 104 */
	u16			D14_DMA_TRANSFER_COUNTER_LSB;
	u16			rev27;
	u16			D14_DMA_TRANSFER_COUNTER_MSB;
	u16			rev28;
	
	/* Offest 112 */
	DMA_CONFIG		D14_DMA_CONFIG;
	u16			rev29;
	u16			DUMMY_3A;
	u16			rev30;
	
	/* Offest 120 */
	DMA_HARDWARE		D14_DMA_HARDWARE;
	u16			rev31;
	u16			DUMMY_3E;
	u16			rev32;
	u16			D14_DATA_TASKFILE_LSB;
	u16			rev33;
	u16			DATA_TASKFILE_MSB;
	u16			rev34;
	u16			D14_CMD_STATUS_TASKFILE;
	u16			rev35;
	u16			DUMMY_46;
	u16			rev36;
	u16			DUMMY_47;
	u16			rev37;
	u16			DUMMY_48;
	u16			rev38;
	u16			DUMMY_49;
	u16			rev39;
	u16			DUMMY_50;
	u16			rev40;
	
	/* Offest 160 */
	DMA_INT			D14_DMA_INT;
	u16			rev41;
	u16			DUMMY_51;
	u16			rev42;
	
	/* Offest 168 */
	DMA_INT_ENABLE		D14_DMA_INT_ENABLE;
	u16			rev43;
	u16			DUMMY_56;
	u16			rev44;
	
	/* Offest 176 */
	u16			D14_DMA_ENDPOINT;
	u16			rev45;
	u16			DUMMY_5A;
	u16			rev46;
	u16			DUMMY_5C;
	u16			rev47;
	u16			DUMMY_5E;
	u16			rev48;
	
	/* Offest 192 */
	DMA_STROBE		DMA_STROBE_TIMING;
	u16			rev49;
	u16			DUMMY_62;
	u16			rev50;
	
	/* Offest 200 */
	u16			D14_DMA_BURST_COUNTER;
	u16			rev51;
	u16			DUMMY_66;
	u16			rev52;
	u16			DUMMY_68;
	u16			rev53;
	u16			DUMMY_6A;
	u16			rev54;
	u16			DUMMY_6C;
	u16			rev55;
	u16			DUMMY_6E;
	u16			rev56;
	
	/* Offest 224 */
	u16			D14_CHIP_ID_LSB;
	u16			rev57;
	u16			D14_CHIP_ID_MSB;
	u16			rev58;
	
	/* Offest 232 */
	FRAME_NO		D14_FRAME_NUMBER;
	u16			rev59;
	u16			DUMMY_76;
	u16			rev60;
	
	/* Offest 240 */
	u16			D14_SCRATCH_REGISTER;
	u16			rev61;
	u16			DUMMY_7A;
	u16			rev62;
	
	/* Offest 248 */
	u16			D14_UNLOCK_DEVICE;
	u16			rev63;
	u16			DUMMY_7E;
	u16			rev64;
	u16			DUMMY_80;
	u16			rev65;
	u16			DUMMY_82;
	u16			rev66;
	
	/* Offest 264 */
	TESTMODE    D14_TEST_MODE;
} D14_CNTRL_REG;

#endif /* __ISP_H__*/

