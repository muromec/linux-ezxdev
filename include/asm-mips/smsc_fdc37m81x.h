/*
 * linux/include/asm-mips/smsc_fdc37m81x.h
 *
 * Interface for smsc fdc48m81x Super IO chip
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * Copyright 2001-2002 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _SMSC_FDC37M81X_H_
#define _SMSC_FDC37M81X_H_

/* Common Registers */
#define SMSC_FDC37M81X_CONFIG_INDEX  0x00
#define SMSC_FDC37M81X_CONFIG_DATA   0x01
#define SMSC_FDC37M81X_CONF          0x02
#define SMSC_FDC37M81X_INDEX         0x03
#define SMSC_FDC37M81X_DNUM          0x07
#define SMSC_FDC37M81X_DID           0x20
#define SMSC_FDC37M81X_DREV          0x21
#define SMSC_FDC37M81X_PCNT          0x22
#define SMSC_FDC37M81X_PMGT          0x23
#define SMSC_FDC37M81X_OSC           0x24
#define SMSC_FDC37M81X_CONFPA0       0x26
#define SMSC_FDC37M81X_CONFPA1       0x27
#define SMSC_FDC37M81X_TEST4         0x2B
#define SMSC_FDC37M81X_TEST5         0x2C
#define SMSC_FDC37M81X_TEST1         0x2D
#define SMSC_FDC37M81X_TEST2         0x2E
#define SMSC_FDC37M81X_TEST3         0x2F

/* Logical device numbers */
#define SMSC_FDC37M81X_FDD           0x00
#define SMSC_FDC37M81X_PARALLEL      0x03
#define SMSC_FDC37M81X_SERIAL1       0x04
#define SMSC_FDC37M81X_SERIAL2       0x05
#define SMSC_FDC37M81X_KBD           0x07
#define SMSC_FDC37M81X_AUXIO         0x08
#define SMSC_FDC37M81X_NONE          0xff

/* Logical device Config Registers */
#define SMSC_FDC37M81X_ACTIVE        0x30
#define SMSC_FDC37M81X_BASEADDR0     0x60    
#define SMSC_FDC37M81X_BASEADDR1     0x61    
#define SMSC_FDC37M81X_INT           0x70
#define SMSC_FDC37M81X_INT2          0x72
#define SMSC_FDC37M81X_LDCR_F0       0xF0

/* Chip Config Values */
#define SMSC_FDC37M81X_CONFIG_ENTER  0x55
#define SMSC_FDC37M81X_CONFIG_EXIT   0xaa
#define SMSC_FDC37M81X_CHIP_ID       0x4d

unsigned long __init
smsc_fdc37m81x_init( unsigned long port );

void
smsc_fdc37m81x_config_beg( void );

void
smsc_fdc37m81x_config_end( void );

void
smsc_fdc37m81x_config_set( u8 reg, u8 val );

#endif
