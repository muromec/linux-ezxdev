/*
 * mux_macro.h
 *
 * Header file for Mux
 *
 * Copyright (C) 2004-2005 - Motorola
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
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
 * 2004-Dec-6  Provide baseline aplogger driver on A780.
 * 2005-Mar-17 Revise to apply the aplogger on EzX common platform
 *
 */

/*
* This header file should be included by both MUX and other applications
* which access MUX device files. It gives the additional macro definitions
* shared between MUX and applications. 
*/

#if __i386
#define MUX_FOR_X86
#endif

#ifdef MUX_FOR_X86

/* MUX DLCI(Data Link Connection Identifier) Configuration */
/* 
*  DLCI     Service
*   0    Control Channel
*   1    Voice Call & Network-related
*   2    SMS MO
*   3    SMS MT
*   4    Phonebook & related
*   5    MISC
*   6    CSD/FAX
*   7    GPRS1
*   8    GPRS2
*   9    Logger CMD
*   10   Logger Data
*   11   Test CMD
*   12   AGPS
*   13   Net Monitor
*/

/* Mapping between DLCI and MUX device files */
/*
*   File Name   Minor  DLCI  AT Command/Data
*   /dev/mux0     0     1     AT Command
*   /dev/mux1     1     2     AT Command
*   /dev/mux2     2     3     AT Command
*   /dev/mux3     3     4     AT Command
*   /dev/mux4     4     5     AT Command
*   /dev/mux5     5     6     AT Command
*   /dev/mux6     6     7     AT Command
*   /dev/mux7     7     8     AT Command
*   /dev/mux8     8     6     Data
*   /dev/mux9     9     7     Data
*   /dev/mux10    10    8     Data
*   /dev/mux11    11    9     Data
*   /dev/mux12    12    10    Data
*   /dev/mux13    13    11    Data
*   /dev/mux14    14    12    Data
*   /dev/mux15    15    13    Data
*/

#define MUX_CMD_FILE_VOICE_CALL   "/dev/mux0"
#define MUX_CMD_FILE_SMS_MO       "/dev/mux1"
#define MUX_CMD_FILE_SMS_MT       "/dev/mux2"
#define MUX_CMD_FILE_PHONEBOOK    "/dev/mux3"
#define MUX_CMD_FILE_MISC         "/dev/mux4"
#define MUX_CMD_FILE_CSD          "/dev/mux5"
#define MUX_CMD_FILE_GPRS1        "/dev/mux6"
#define MUX_CMD_FILE_GPRS2        "/dev/mux7"
#define MUX_CMD_FILE_GPRS3        "/dev/mux_none"
#define MUX_CMD_FILE_GPRS4        "/dev/mux_none"

#define MUX_DATA_FILE_CSD         "/dev/mux8"
#define MUX_DATA_FILE_GPRS1       "/dev/mux9"
#define MUX_DATA_FILE_GPRS2       "/dev/mux10"
#define MUX_DATA_FILE_LOGGER_CMD  "/dev/mux11"
#define MUX_DATA_FILE_LOGGER_DATA "/dev/mux12"
#define MUX_DATA_FILE_TEST_CMD    "/dev/mux13"
#define MUX_DATA_FILE_AGPS        "/dev/mux14"
#define MUX_DATA_FILE_NET_MONITOR "/dev/mux15"
#define MUX_DATA_FILE_UMA_AUDIO   "/dev/mux_none"
#define MUX_DATA_FILE_GPRS3       "/dev/mux_none"
#define MUX_DATA_FILE_GPRS4       "/dev/mux_none"
#define MUX_DATA_FILE_UMA_WIFI1   "/dev/mux_none"
#define MUX_DATA_FILE_UMA_WIFI2   "/dev/mux_none"

#else

/* MUX DLCI(Data Link Connection Identifier) Configuration */
/* 
*  DLCI     Service
*   0    Control Channel
*   1    Voice Call & Network-related
*   2    SMS MO
*   3    SMS MT
*   4    Phonebook & related
*   5    MISC
*   6    UMA Wi-Fi Audio
*   7    CSD/FAX
*   8    GPRS1
*   9    GPRS2
*   10   GPRS3
*   11   GPRS4
*   12   UMA Wi-Fi Channel 1
*   13   UMA Wi-Fi Channel 2
*   14   AGPS
*   15   Net Monitor
*   16   Test CMD
*   17   Logger CMD
*   18   Logger Data
*/

/* Mapping between DLCI and MUX device files */
/*
*   File Name   Minor  DLCI  AT Command/Data
*   /dev/mux0     0     1     AT Command
*   /dev/mux1     1     2     AT Command
*   /dev/mux2     2     3     AT Command
*   /dev/mux3     3     4     AT Command
*   /dev/mux4     4     5     AT Command
*   /dev/mux5     5     7     AT Command
*   /dev/mux6     6     8     AT Command
*   /dev/mux7     7     9     AT Command
*   /dev/mux8     8     10    AT Command
*   /dev/mux9     9     11    AT Command
*   /dev/mux10    10    6     Data
*   /dev/mux11    11    7     Data
*   /dev/mux12    12    8     Data
*   /dev/mux13    13    9     Data
*   /dev/mux14    14    10    Data
*   /dev/mux15    15    11    Data
*   /dev/mux16    16    12    Data
*   /dev/mux17    17    13    Data
*   /dev/mux18    18    14    Data
*   /dev/mux19    19    15    Data
*   /dev/mux20    20    16    Data
*   /dev/mux21    21    17    Data
*   /dev/mux22    22    18    Data
*/

#define MUX_CMD_FILE_VOICE_CALL   "/dev/mux0"
#define MUX_CMD_FILE_SMS_MO       "/dev/mux1"
#define MUX_CMD_FILE_SMS_MT       "/dev/mux2"
#define MUX_CMD_FILE_PHONEBOOK    "/dev/mux3"
#define MUX_CMD_FILE_MISC         "/dev/mux4"
#define MUX_CMD_FILE_CSD          "/dev/mux5"
#define MUX_CMD_FILE_GPRS1        "/dev/mux6"
#define MUX_CMD_FILE_GPRS2        "/dev/mux7"
#define MUX_CMD_FILE_GPRS3        "/dev/mux8"
#define MUX_CMD_FILE_GPRS4        "/dev/mux9"

#define MUX_DATA_FILE_UMA_AUDIO   "/dev/mux10"
#define MUX_DATA_FILE_CSD         "/dev/mux11"
#define MUX_DATA_FILE_GPRS1       "/dev/mux12"
#define MUX_DATA_FILE_GPRS2       "/dev/mux13"
#define MUX_DATA_FILE_GPRS3       "/dev/mux14"
#define MUX_DATA_FILE_GPRS4       "/dev/mux15"
#define MUX_DATA_FILE_UMA_WIFI1   "/dev/mux16"
#define MUX_DATA_FILE_UMA_WIFI2   "/dev/mux17"
#define MUX_DATA_FILE_AGPS        "/dev/mux18"
#define MUX_DATA_FILE_NET_MONITOR "/dev/mux19"
#define MUX_DATA_FILE_TEST_CMD    "/dev/mux20"
#define MUX_DATA_FILE_LOGGER_CMD  "/dev/mux21"
#define MUX_DATA_FILE_LOGGER_DATA "/dev/mux22"

#endif


#define NUM_MUX_CMD_FILES 10
#define NUM_MUX_DATA_FILES 13
#define NUM_MUX_FILES ( NUM_MUX_CMD_FILES  +  NUM_MUX_DATA_FILES )

/* Special ioctl() upon a MUX device file for hanging up a call */
#define TS0710MUX_IO_MSC_HANGUP 0x54F0

/* Special ioctl() upon a MUX device file for MUX loopback test */
#define TS0710MUX_IO_TEST_CMD 0x54F1


/* Special Error code might be return from write() to a MUX device file  */
#define EDISCONNECTED 900  /* Logical data link is disconnected */

/* Special Error code might be return from open() to a MUX device file  */
#define EREJECTED 901 /* Logical data link connection request is rejected */





/* 
* MUX data counter 
*/
#define TS0710MUX_CSD_RECV_COUNT_IDX   0
#define TS0710MUX_CSD_SEND_COUNT_IDX   1
#define TS0710MUX_GPRS1_RECV_COUNT_IDX 2
#define TS0710MUX_GPRS1_SEND_COUNT_IDX 3
#define TS0710MUX_GPRS2_RECV_COUNT_IDX 4
#define TS0710MUX_GPRS2_SEND_COUNT_IDX 5
#define TS0710MUX_GPRS3_RECV_COUNT_IDX 6
#define TS0710MUX_GPRS3_SEND_COUNT_IDX 7
#define TS0710MUX_GPRS4_RECV_COUNT_IDX 8
#define TS0710MUX_GPRS4_SEND_COUNT_IDX 9

#define TS0710MUX_COUNT_MAX_IDX        9
#define TS0710MUX_COUNT_IDX_NUM (TS0710MUX_COUNT_MAX_IDX + 1)

typedef unsigned int TS0710MUX_COUNT_A[ TS0710MUX_COUNT_IDX_NUM ];

