/*
 * File: omap1510_hid.c
 *
 * The USAR driver layer for the HID (Human Input Device) controller 
 *   for the TI P1.
 *  
 * This layer sends and recvs commands to the JUNO part and provides 
 *   JUNO Layer error checking.  The caller must parse the command.
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
 * key: RRGPLCR (do not remove)
 *
 */

#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include "omap1510_hid.h"

#undef DEBUG
#define DEBUG
#ifdef DEBUG
#  define DBG(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args)
#  define ENTRY() DBG(":entry\n")
#else
#  define DBG(fmt, args...) ;
#  define ENTRY() ;
#endif

extern void spi_snd_packet(u8 *databyte, u8 num_bytes);
extern void spi_rcv_packet(u8 *buffer, u8 *num_bytes);

// calculate the longitudinal Redundancy Check
static u8 calculate_LRC(char *buffer, int size)
{
        char LRC;
        int index;

        // Init the LRC using the first two message u8s
        LRC = buffer[0] ^ buffer[1];
  
        // Update the LRC using the remainder of the bufffer
        for ( index = 2; index < size; index++ )
                LRC ^= buffer[index];

        // If the MSB is set then clear the MSB and change the next most 
        // significant bit
        if ( LRC & 0x80 )
                LRC ^= 0xC0;

        return LRC;
}

// calculate the longitudinal Redundancy Check
static u8 calculate_block_LRC(char *buffer, char *block, u8 size)
{
        char LRC;
        int index;

        // Init the LRC using the first two message u8s
        LRC = buffer[0] ^ buffer[1];
        LRC ^= buffer[2];
  
        // Update the LRC using the block
        for ( index = 0; index < size; index++ )
                LRC ^= block[index];
  
        // If the MSB is set then clear the MSB and change the next most 
        // significant bit
        if ( LRC & 0x80 )
                LRC ^= 0xC0;

        return LRC;
}

// host commands 
static int hcmd_simple(st_simple *simple, u8 cmd_code)
{
        simple->header   = SIMPLE;
        simple->cmd_code = cmd_code;
        simple->LRC      = calculate_LRC((char*)simple, 2);

        return 0;
}
 
static int hcmd_write_register_bit(st_write_register_bit *write_bit, 
                                   u8 reg_offset, u8 bit, u8 value)
{
        u8 value_bit = 0;
  
        write_bit->header     = WRITE_REGISTER_BIT;
        write_bit->reg_offset = reg_offset;

        value_bit = bit;
        value_bit <<= 1;

        if ( value )
                value_bit |= 0x1;

        write_bit->value_bit  = value_bit;
        write_bit->LRC        = calculate_LRC((char*)write_bit, 3);

        return 0;
}

static int hcmd_read_register_bit(st_read_register_bit *read_bit, 
                                  u8 reg_offset, u8 bit)
{
        read_bit->header     = READ_REGISTER_BIT;
        read_bit->reg_offset = reg_offset;
        read_bit->bit        = bit;
        read_bit->LRC        = calculate_LRC((char*)read_bit, 3);

        return 0;
}

static int hcmd_write_register(st_write_register *write_reg, u8 reg_offset,
                               u8 value)
{
        write_reg->header     = WRITE_REGISTER;
        write_reg->reg_offset = reg_offset;
        write_reg->value      = value;
        write_reg->LRC        = calculate_LRC((char*)write_reg, 3);

        return 0;
}

static int hcmd_read_register(st_read_register *read_reg, u8 reg_offset)
{
        read_reg->header     = READ_REGISTER;
        read_reg->reg_offset = reg_offset;
        read_reg->LRC        = calculate_LRC((char*)read_reg, 2);

        return 0;
}

static int hcmd_write_block(st_write_block *write_block, u8 reg_offset, 
                            u8 block_length, u8 *block)
{
        int i;

        write_block->header       = WRITE_BLOCK;
        write_block->reg_offset   = reg_offset;
        write_block->block_length = block_length;

        for ( i = 0; i < block_length; i++ )
                write_block->block[i] = block[i];

        if ( block_length == 32 )
                write_block->LRC = calculate_block_LRC((char*)write_block, 
                                                       block, block_length);
        else
                write_block->block[block_length] = 
                        calculate_block_LRC((char*)write_block, block, 
                                            block_length);

        return 0;
}

// NOTE: untested
static int hcmd_read_block(st_read_block *read_block, u8 reg_offset, 
                           u8 block_length, u8 *block)
{
        int i;

        read_block->header       = READ_BLOCK;
        read_block->reg_offset   = reg_offset;
        read_block->block_length = block_length;

        for ( i = 0; i < block_length; i++ )
                read_block->block[i] = block[i];

        if ( block_length == 32 )
                read_block->LRC = calculate_block_LRC((char*)read_block, 
                                                      block, block_length);
        else
                read_block->block[block_length] = 
                        calculate_block_LRC((char*)read_block, block, 
                                            block_length);


        return 0;
}

// controller responses
static int cres_simple(st_simple *simple, u8 nbytes)
{
        u8 LRC;

        if ( nbytes == 0 )
                return -E_ZERO_BYTES;

        if ( simple->header != SIMPLE )
                return -E_BAD_HEADER;

        LRC = calculate_LRC((char*)simple, 2);

        if ( LRC != simple->LRC )
                return -E_BAD_LRC;

        return 0;
}

static int cres_report_register_bit(st_report_register_bit *report_bit, 
                                    u8 nbytes, u8 data)
{
        u8 LRC;
 
        if ( nbytes == 0 )
                return -E_ZERO_BYTES;

        if ( report_bit->header != REPORT_REGISTER_BIT )
                return -E_BAD_HEADER;

        LRC = calculate_LRC((char*)report_bit, 3);

        if ( LRC != report_bit->LRC )
                return -E_BAD_LRC;

        if ( report_bit->value_bit != data )
                return -E_BAD_VALUE;

        return 0;
}

static int cres_report_register(st_report_register *report_reg, u8 nbytes, 
                                u8 data)
{
        u8 LRC;

        if ( nbytes == 0 )
                return -E_ZERO_BYTES;

        if ( report_reg->header != REPORT_REGISTER )
                return -E_BAD_HEADER;

        LRC = calculate_LRC((char*)report_reg, 3);

        if ( LRC != report_reg->LRC )
                return -E_BAD_LRC;

        if ( report_reg->value != data )
                return -E_BAD_VALUE;

        return 0;
}

static int cres_report_block(st_report_block *report_block, u8 nbytes)
{
        u8 LRC;

        if ( nbytes == 0 )
                return -E_ZERO_BYTES;

        if ( report_block->header != REPORT_BLOCK )
                return -E_BAD_HEADER;

        LRC = calculate_block_LRC((char*)report_block, report_block->block, 
                                  report_block->block_length);

        if ( report_block->block_length == 32 )
        {
                if ( LRC != report_block->LRC )
                        return -E_BAD_LRC;
                else if ( LRC != 
                          report_block->block[report_block->block_length] )
                        return -E_BAD_LRC;
        }

        return 0;
}

static int cres_pointing_report(st_pointing_report *pointing, u8 nbytes)
{
        u8 LRC;

        if ( pointing->header != POINTING_REPORT )
                return -E_BAD_HEADER;

        LRC = calculate_LRC((char*)pointing, 5);

        if ( LRC != pointing->LRC )
                return -E_BAD_LRC;

        return POINTING_REPORT;
}

static int cres_keyboard_report(st_keyboard_report *keyboard) 
{
        u8 LRC;

        if ( keyboard->header != KEYBOARD_REPORT )
                return -E_BAD_HEADER;

        LRC = calculate_LRC((char*)keyboard, 2);

        if ( LRC != keyboard->LRC )
                return -E_BAD_LRC;

        return KEYBOARD_REPORT;
}

static int report_type(u8 *type)
{
        // check the header to find out what kind of report it is
        if ( (*type) == KEYBOARD_REPORT )
                return KEYBOARD_REPORT;
        else if ( (*type) == POINTING_REPORT )
                return POINTING_REPORT;
        else return -E_BAD_HEADER;
}

void send_juno_command(u8 type, u8 reg_offset, u8 data1, u8 data2,
                       int length, u8 *block)
{
        st_simple simple;
        st_write_register_bit write_bit;
        st_read_register_bit read_bit;
        st_write_register write_reg;
        st_read_register read_reg;
        st_write_block write_block;
        st_read_block read_block;

        switch ( type )
        {
        case SIMPLE:
                hcmd_simple(&simple, data1);
                spi_snd_packet((u8 *)&simple, SIZE_SIMPLE);
                break;
        case WRITE_REGISTER_BIT:
                hcmd_write_register_bit(&write_bit, reg_offset, data1, data2);
                spi_snd_packet((u8 *)&write_bit, SIZE_WRITE_BIT);
                break;
        case READ_REGISTER_BIT:
                hcmd_read_register_bit(&read_bit, reg_offset, data1);
                spi_snd_packet((u8 *)&read_bit, SIZE_READ_BIT);
                break;
        case WRITE_REGISTER:
                hcmd_write_register(&write_reg, reg_offset, data1);
                spi_snd_packet((u8 *)&write_reg, SIZE_WRITE_REGISTER);
                break;
        case READ_REGISTER:
                hcmd_read_register(&read_reg, reg_offset);
                spi_snd_packet((u8 *)&read_reg, SIZE_READ_REGISTER);
                break;
        case WRITE_BLOCK:
                hcmd_write_block(&write_block, reg_offset, length, block);
                spi_snd_packet((u8 *)&write_block, 
                               length + SIZE_WRITE_BLOCK);
                break;
        case READ_BLOCK:
                hcmd_read_block(&read_block, reg_offset, length, block);
                spi_snd_packet((u8 *)&read_block, length + SIZE_READ_BLOCK);
                break;
        default:
                printk("Unrecognized command: %d\n", type);
        }
}

int receive_juno_response(u8 type, u8 *buffer, u8 *nbytes, u8 data)
{
        int ret = 0;

        switch ( type )
        {
        case SIMPLE:
                spi_rcv_packet(buffer, nbytes);
                ret = cres_simple((st_simple *)buffer, *nbytes);
                break;
        case REPORT_REGISTER_BIT:
                spi_rcv_packet(buffer, nbytes);
                ret = cres_report_register_bit(
                        (st_report_register_bit *)buffer, *nbytes, data);
                break;
        case REPORT_REGISTER:
                spi_rcv_packet(buffer, nbytes);
                ret = cres_report_register((st_report_register *)buffer, 
                                           *nbytes, data);
                break;
        case REPORT_BLOCK:
                spi_rcv_packet(buffer, nbytes);
                ret = cres_report_block((st_report_block *)buffer, *nbytes);
                break;        
        case ASYNC_REPORT:
                spi_rcv_packet(buffer, nbytes);

                if ( report_type(buffer) == POINTING_REPORT )
                        ret = cres_pointing_report((st_pointing_report *)buffer, 
                                                   *nbytes);
                else if ( report_type(buffer) == KEYBOARD_REPORT )
                        ret = cres_keyboard_report((st_keyboard_report *)buffer);
                else 
                        ret = -1;

                break;

        default:
                printk("Unrecognized command: %d\n", type);
        }

        return ret;
}

/*
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
