/*
 * File: omap1510_hid.h
 *
 * The USAR driver layer for the HID (Human Input Device) controller 
 *   for the TI P1
 *
 * REFERENCES
 *   Technical Reference Manual USAR Juno: UR8HC007-001 Jupiter class 
 *   HID & Power Management Control
 *
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: Alex McMains <aam@ridgerun.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
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
 *
 * Please report all bugs/problems to the author or <support@dsplinux.net>
 *
 *  key: RRGPLCR (do not remove)
 *
 */

/* 
   The registers are divided into two pages where each page contains 255 
   registers:
       Page 0: Configuration Control and Status
       Page 1: Scanned Matrix and Alternate Layout Keys
*/

#ifndef __HID_H__
#define __HID_H__

#include <linux/types.h>

// struct sizes since they are being padded
#define SIZE_SIMPLE         3
#define SIZE_WRITE_BIT      4
#define SIZE_READ_BIT       4
#define SIZE_WRITE_REGISTER 4
#define SIZE_READ_REGISTER  3
#define SIZE_WRITE_BLOCK    4
#define SIZE_READ_BLOCK     4

// Shared protocol header
#define SIMPLE              0x80

// Protocol Headers for HOST issued commands
#define WRITE_REGISTER_BIT  0x81
#define READ_REGISTER_BIT   0x82
#define WRITE_REGISTER      0x83
#define READ_REGISTER       0x84
#define WRITE_BLOCK         0x85
#define READ_BLOCK          0x86

// Protocol Headers for CONTROLLER issued responses
#define REPORT_REGISTER_BIT 0x81
#define REPORT_REGISTER     0x83
#define REPORT_BLOCK        0x85
#define POINTING_REPORT     0x87
#define KEYBOARD_REPORT     0x88
#define ASYNC_REPORT        0x99 // this is not a valid Juno HID protocol header

// Simple Commands
#define INITIALIZE              0x00
#define INITIALIZATION_COMPLETE 0x01
#define RESEND_REQUEST          0x05

#define PAGE_NUMBER 0xFF // changes pages, accepts 0 or 1

// Errors 
#define E_BAD_HEADER    1
#define E_BAD_LRC       2
#define E_ZERO_BYTES    3
#define E_BAD_VALUE     4
#define E_BAD_MODE      5
#define E_REPORT_MODE   6
#define E_BAD_ACK       7
#define E_BAD_DEVICE_ID 8

// Registers
#define GPE_STATUS_0_REGISTER              20
#define PS2_DEVICE_DIRECT_CONTROL_REGISTER 22
#define CONTROL_REGISTER                   128
#define HOST_TRANSMIT_REGISTER             129

// Values
#define IP_CON            0x04 // internal PS/2 device connect
#define REP_MODE          0x80 // report mode turned on
#define PS2_DEVICE_ENABLE 0x01
#define SET_SAMPLING_RATE 0xf3
#define REPORS_60         0x3c
#define REPORS_80         0x50
#define REPORS_100        0x64
#define READ_DEVICE_TYPE  0xf2
#define VALID_DEVICE_ID   0x10
#define MODE_ACK          0xfa

typedef struct _st_simple
{
  u8 header;
  u8 cmd_code;
  u8 LRC; 
} st_simple __attribute__ ((packed));

// host command structures
typedef struct _st_write_register_bit
{
  u8 header;
  u8 reg_offset;
  u8 value_bit;
  u8 LRC;
} st_write_register_bit __attribute__ ((packed));
 
typedef struct _st_read_register_bit
{
  u8 header;
  u8 reg_offset;
  u8 bit;
  u8 LRC;
} st_read_register_bit __attribute__ ((packed));

typedef struct _st_write_register
{
  u8 header;
  u8 reg_offset;
  u8 value;
  u8 LRC;
} st_write_register __attribute__ ((packed));

typedef struct _st_read_register
{
  u8 header;
  u8 reg_offset;
  u8 LRC;
} st_read_register __attribute__ ((packed));

typedef struct _st_write_block
{
  u8 header;
  u8 reg_offset;
  u8 block_length;
  u8 block[32];
  u8 LRC;
} st_write_block __attribute__ ((packed));

typedef struct _st_read_block
{
  u8 header;
  u8 reg_offset;
  u8 block_length;
  u8 block[32];
  u8 LRC;
} st_read_block __attribute__ ((packed));

// controller command structures
typedef struct _st_report_register_bit
{
  u8 header;
  u8 reg_offset;
  u8 value_bit;
  u8 LRC;
} st_report_register_bit __attribute__ ((packed));
 
typedef struct _st_report_register
{
  u8 header;
  u8 reg_offset;
  u8 value;
  u8 LRC;
} st_report_register __attribute__ ((packed));

typedef struct _st_report_block
{
  u8 header;
  u8 reg_offset;
  u8 block_length;
  u8 block[32];
  u8 LRC;
} st_report_block __attribute__ ((packed));

typedef struct _st_pointing_report
{
  u8 header;
  u8 buttons;
  u8 Xdisplacement;
  u8 Ydisplacement;
  u8 Zdisplacement;
  u8 LRC;
} st_pointing_report __attribute__ ((packed));

typedef struct _st_keyboard_report
{
  u8 header;
  u8 keynum; // up > 0x80, down < 0x7E, all keys up 0x00
  u8 LRC;
} st_keyboard_report __attribute__ ((packed));

#endif

/*---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
