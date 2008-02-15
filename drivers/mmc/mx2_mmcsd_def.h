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
 *
 */

#ifndef  MMCSDLOCAL_H_adkf	/* PROTECT DOUBLE INCLUDE */
#define  MMCSDLOCAL_H_adkf

#define MMCSD_TYPE_MMC        1
#define MMCSD_TYPE_SD         2
#define MMCSD_TYPE_SDIO       3

#define    MMCSD_REQUEST_SOURCE         21

/*--------------------------------------------------------------------------*/
/*                          DEFINITIONS FOR CSD/CID                         */
/*--------------------------------------------------------------------------*/
#define    MMCSD_CSD_LEN        0x0080        /* BIT LENGTH OF CSD REGISTER */
#define    MMCSD_CID_LEN        0x0080        /* BIT LENGTH OF CID REGISTER */
#define    MMCSD_CSD_BYTESIZE        16        /* BYTE LENGTH OF CSD REGISTER */
#define    MMCSD_CID_BYTESIZE        16        /* BYTE LENGTH OF CID REGISTER */

/*--------------------------------------------------------------------------*/
/*                           DEFINE COMMANDS                                */
/*--------------------------------------------------------------------------*/
#define    MMCSD_CMD0                 0x00
#define    MMCSD_CMD1                 0x01
#define    MMCSD_CMD2                 0x02
#define    MMCSD_CMD3                 0x03
#define    MMCSD_CMD4                 0x04
#define    MMCSD_CMD5                 0x05
#define    MMCSD_CMD6                 0x06
#define    MMCSD_CMD7                 0x07
#define    MMCSD_CMD8                 0x08
#define    MMCSD_CMD9                 0x09
#define    MMCSD_CMD10                0x0A
#define    MMCSD_CMD11                0x0B
#define    MMCSD_CMD12                0x0C
#define    MMCSD_CMD13                0x0D
#define    MMCSD_CMD14                0x0E
#define    MMCSD_CMD15                0x0F
#define    MMCSD_CMD16                0x10
#define    MMCSD_CMD17                0x11
#define    MMCSD_CMD18                0x12
#define    MMCSD_CMD19                0x13
#define    MMCSD_CMD20                0x14
#define    MMCSD_CMD21                0x15
#define    MMCSD_CMD22                0x16
#define    MMCSD_CMD23                0x17
#define    MMCSD_CMD24                0x18
#define    MMCSD_CMD25                0x19
#define    MMCSD_CMD26                0x1A
#define    MMCSD_CMD27                0x1B
#define    MMCSD_CMD28                0x1C
#define    MMCSD_CMD29                0x1D
#define    MMCSD_CMD30                0x1E
#define    MMCSD_CMD31                0x1F
#define    MMCSD_CMD32                0x20
#define    MMCSD_CMD33                0x21
#define    MMCSD_CMD34                0x22
#define    MMCSD_CMD35                0x23
#define    MMCSD_CMD36                0x24
#define    MMCSD_CMD37                0x25
#define    MMCSD_CMD38                0x26
#define    MMCSD_CMD39                0x27
#define    MMCSD_CMD40                0x28
#define    MMCSD_CMD41                0x29
#define    MMCSD_CMD42                0x2A
#define    MMCSD_CMD43                0x2B
#define    MMCSD_CMD44                0x2C
#define    MMCSD_CMD45                0x2D
#define    MMCSD_CMD46                0x2E
#define    MMCSD_CMD47                0x2F
#define    MMCSD_CMD48                0x30
#define    MMCSD_CMD49                0x31
#define    MMCSD_CMD50                0x32
#define    MMCSD_CMD51                0x33
#define    MMCSD_CMD52                0x34
#define    MMCSD_CMD53                0x35
#define    MMCSD_CMD54                0x36
#define    MMCSD_CMD55                0x37
#define    MMCSD_CMD56                0x38
#define    MMCSD_CMD57                0x39
#define    MMCSD_CMD58                0x3A
#define    MMCSD_CMD59                0x3B
#define    MMCSD_CMD60                0x3C
#define    MMCSD_CMD61                0x3D
#define    MMCSD_CMD62                0x3E
#define    MMCSD_CMD63                0x3F

#define    MMCSD_ACMD6                      0x06
#define           MMCSD_ACMD13                      0x0D
#define    MMCSD_ACMD22               0x16
#define    MMCSD_ACMD23               0x17
#define    MMCSD_ACMD41               0x29
#define    MMCSD_ACMD42               0x2A
#define    MMCSD_ACMD51               0x33

/*--------------------------------------------------------------------------*/
/*                         DEFINE CARD DOMAIN                               */
/*--------------------------------------------------------------------------*/
#define    MMCSD_CAPA4M               4        /* 4 MByte                     */
#define    MMCSD_CAPA8M               8        /* 8 MByte                     */
#define    MMCSD_CAPA16M             16        /* 16 MByte                    */
#define    MMCSD_CAPA32M             32        /* 32 MByte                    */
#define    MMCSD_CAPA48M             48        /* 48 MByte                    */
#define    MMCSD_CAPA64M             64        /* 64 MByte                    */
#define    MMCSD_CAPA128M           128        /* 128 MByte                   */

/*--------------------------------------------------------------------------*/
/*                          DEFINE RESPONSES                                */
/*--------------------------------------------------------------------------*/
#define    MMCSD_RES1LEN              0x0030        /* (MMC)RESPONSE1BIT LENGTH  */
#define    MMCSD_RES2LEN              0x0088        /* (MMC)RESPONSE2BIT LENGTH  */
#define    MMCSD_RES3LEN              0x0030        /* (MMC)RESPONSE3BIT LENGTH  */
#define    MMCSD_RES4LEN              0x0030        /* (MMC)RESPONSE4BIT LENGTH  */
#define    MMCSD_RES5LEN              0x0030        /* (MMC)RESPONSE5BIT LENGTH  */

/*--------------------------------------------------------------------------*/
/*                        DEFINE WAIT CLOCK                                 */
/*--------------------------------------------------------------------------*/
#define    MMCSD_WAIT_UNIT                8
#define    MMCSD_NCS_MIN                  0*WAIT_UNIT
#define    MMCSD_NCS_MAX                  1*WAIT_UNIT
#define    MMCSD_NCR_MIN                  1*WAIT_UNIT
#define    MMCSD_NCR_MAX                  8*WAIT_UNIT
#define    MMCSD_NRC_MIN                  1*WAIT_UNIT
#define    MMCSD_NRC_MAX                  1*WAIT_UNIT
#define    MMCSD_NAC_MIN                  1*WAIT_UNIT
#define    MMCSD_NAC_MAX                  16*WAIT_UNIT
#define    MMCSD_NWR_MIN                  1*WAIT_UNIT
#define    MMCSD_NWR_MAX                  1*WAIT_UNIT
#define    MMCSD_BUSY_WAIT                0xFFFFFFFF        /* = 65535(decimal)            */

#define                MMCSDB_R1                0x01
#define         MMCSDB_R2                0x02
#define         MMCSDB_R3                0x03
#define         MMCSDB_R4                0x04
#define         MMCSDB_R5                0x05
#define         MMCSDB_R6                0x06
#define         MMCSDB_DATEN             0x08
#define         MMCSDB_WRRD              0x10
#define         MMCSDB_STRBLK            0x20
#define         MMCSDB_BSY               0x40
#define         MMCSDB_INIT              0x80
#define         MMCSD_BUS_4BIT           0x200

/*-------------------- Mask Definitions --------------------------------------*/
#define         MMCSD_CLOCK_MASK        0x0000003

/*--------------------Finish wait condition--------------------------------------*/
#define     MMCSD_CMD_DONE                  0x01
#define     MMCSD_DATA_TRANS_DONE           0x02

/*--------------------------------------------------------------------------*/
/*               ILLEGAL ERRORS THAT SOFTWARE CHECKS                        */
/*--------------------------------------------------------------------------*/

#define    MMCSD_CSD_MISS                 0x1000        /* ABNORMAL CSD               */
#define    MMCSD_CID_MISS                 0x2000        /* ABNORMAL CID               */
#define    MMCSD_MBR_ERROR                0x3000        /* ABNORMAL MBR DATA          */
#define    MMCSD_BPB_ERROR                0x4000        /* ABNORMAL BPB DATA          */
#define    MMCSD_NO_CARD                  0x5000        /* CANNOT IDENTIFY CARD       */
#define    MMCSD_NOT_EXIST_AREA           0x6000        /* ACCESS TO NON-EXIST DOMAIN */
#define    MMCSD_ILLEGAL_STATE            0x7000

#define    MMCSD_ILLEGAL_ERROR_MASK                               0xF000

#define    MMCSD_STATUS_NOERROR    0xFDFF0000

#define                MMCSD_READ_TIMEOUT 0x1
#define                MMCSD_CMD_TIMEOUT  0x2
#define                MMCSD_WRITE_CRC    0x4
#define                MMCSD_READ_CRC     0x8
#define                MMCSD_HARDWARE_REV 0x10
#define                MMCSD_RESP_CRC_ERR  0x20
#define                MMCSD_END_CMD_RESP 0x2000

#define                MMCSD_STATUS_ERR  (MMCSD_READ_TIMEOUT|MMCSD_CMD_TIMEOUT|MMCSD_WRITE_CRC)

#define                MMCSD_STATUS         u32

#define          MMCSD_MMC_CARD_VOLTAGE    0x00ff8000
#define          MMCSD_CARD_VOLTAGE    0x00ff8000
#define          MMCSD_CARD_SET_VOLTAGE    0x00008000

/*--------------------------------------------------------------------------*/
/*               Pin Configuration                                          */
/*--------------------------------------------------------------------------*/

#define MX2STAT_CARD_PRESENCE   (1<<15)
#define MX2STAT_SDIO_INT_ACTIVE            (1<<14)
#define MX2STAT_END_CMD_RESP     (1<<13)
#define MX2STAT_WRITE_OP_DONE      (1<<12)
#define MX2STAT_DATA_TRANS_DONE              (1<<11)
#define MX2STAT_WRITE_CRC_ERROR_CODE  ((1<<9) | (1<<10))
#define MX2STAT_CARD_BUS_CLK_RUN      (1<< 8)
#define MX2STAT_APPL_BUFF_FF     (1<< 7)
#define MX2STAT_APPL_BUFF_FE     (1<< 6)
#define MX2STAT_RESP_CRC_ERR    (1<< 5)
#define MX2STAT_CRC_READ_ERR (1<< 3)
#define MX2STAT_CRC_WRITE_ERR  (1<< 2)
#define MX2STAT_TIME_OUT_RESP    (1<< 1)
#define MX2STAT_TIME_OUT_READ   (1<< 0)

#define MX2_AUTO_CARD_DETECT_MASK   0x3f
#define MX2_DATA_EN_MASK   0x5f
#define MX2_SDIO_MASK      0x6f
#define MX2_BUF_READY_MASK   0x77
#define MX2_END_CMD_RES_MASK   0x7b
#define MX2_WRITE_OP_DONE_MASK  0x7d
#define MX2_DATA_TRAN_MASK    0x7e
#define MX2_ALL_MASK    0x7f

#endif
