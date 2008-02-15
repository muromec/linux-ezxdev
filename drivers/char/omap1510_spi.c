/*
 * File: omap1510_spi.c  
 *
 * OMAP1510 SPI interface for the HID controller
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

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include "omap1510_hid.h"

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#  define DBG(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args)
#  define ENTRY() DBG(":entry\n")
#else
#  define DBG(fmt, args...) ;
#  define ENTRY() ;
#endif
#if defined(CONFIG_ARCH_TI925)
#define OMAP_FPGA_BASE IO_ADDRESS(0x11000000)
#define OMAP_FPGA_HID_SPI 0x08
#elif defined(CONFIG_OMAP_1510P1) || defined(CONFIG_OMAP_INNOVATOR)
#define OMAP_FPGA_BASE 0xE8000000
#define OMAP_FPGA_HID_SPI 0x4
#else
#error "not supported Architecture"
#endif

extern void send_juno_command(u8 type, u8 reg_offset, u8 data1, u8 data2, 
                              int length, u8 *block);
extern int receive_juno_response(u8 type, u8 *buffer, u8 *nbytes, u8 data);

#define SPI_WRITE_REG (OMAP_FPGA_BASE + OMAP_FPGA_HID_SPI)
#define SPI_READ_REG  SPI_WRITE_REG

// FPGA HID Register Bit Patterns
#define MOSI_CLK_HI      0x03
#define HID_HOST_ACTIVE  0x80
#define _SS_LOW          0x04
#define _SS_HIGH         0xfb
#define CLK_LOW          0xfe
#define CLK_HIGH         0x01
#define MOSI_LOW         0xfd
#define MOSI_HI          0x02
#define MISO_BIT         0x10
#define ATN              0x20

static inline void spi_write(unsigned long val) 
{
        outb(val,SPI_WRITE_REG);
}

static inline unsigned long spi_read(void) 
{
        return inb(SPI_READ_REG);
}

static inline void spi_reset(void) 
{
        spi_read();
        spi_write(MOSI_CLK_HI);
        spi_write(HID_HOST_ACTIVE | MOSI_CLK_HI);
}

unsigned long atn_high(void) 
{
        return !(spi_read() & ATN);
}

// timings
#define TMAC 100    
#define TSLOW 2    
#define TMIB 150  
#define TSNEXT 120
#define TSSC 100
#define TSNA 100

void spi_snd_packet(u8 *databyte, u8 num_bytes);
void spi_rcv_packet(u8 *buffer, u8 *num_bytes);
int spi_init(void);
unsigned long atn_high(void);

extern unsigned long jiffies;

/*****************************************************************************
 USAR doc8-007-001-tr-0721.pdf

 (1) The Host asserts _SS to wake up the slave (USAR UR8HC007) device and 
     prepare it for a data transfer 
 (2) The slave asserts _ATN to indicate that it is ready to receive data and 
     prepares the output register (MISO line) to output FFh whenever clocks 
     are provided by the host. 
 (3) The host starts placing the clock signals and data on the SCLK and the 
     MOSI lines respectively (8 pulses for 8 data bits).
 (4) After the completion of one byte transfer the host continues with the 
     remaining bytes of the packet until the packet transfer is complete. 
     The _SS line is kept low to indicate to the slave that more bytes are 
     following. At least 150ms must elapse between the beginning of one byte 
     transfer and the beginning of the next byte transfer so the slave has 
     time to process each byte. 
 (5) The whole packet is transmitted. 
 (6) After the last byte of the packet is transferred the host de-asserts the 
     _SS line to indicate that the transfer has been completed. 
 (7) The slave releases the _ATN line indicating it is busy. 
 (8) If the host has another packet to send it can only assert _SS after _ATN 
     has been de-asserted for at least 120ms. This is necessary in order to 
     provide the USAR Juno TM with sufficient time to process the received 
     packet and assert _ATN associated with a Resend request if the packet was
     received in error
 *****************************************************************************/

void spi_snd_packet(u8 *databyte, u8 num_bytes)
{
        u8 *next_byte;
        u8 reg_val;
        u8 bit_val;
        u8 nbytes;
        u8 next_bit;
        static unsigned long timeout;

        next_byte = databyte;
        nbytes    = num_bytes;
                
        // (1) The Host asserts _SS to wake up the slave (USAR UR8HC007) 
        //     device and prepare it for a data transfer
        reg_val = spi_read();
        spi_write(reg_val | _SS_LOW);

        // (2) The slave asserts _ATN to indicate that it is ready to receive 
        //     data and prepares the output register (MISO line) to output 
        //     FFh whenever clocks are provided by the host. 

        // wait for ATN to be ASSERTED (i.e. LOW - bit 5 == 1)
        timeout = jiffies + 10;
        while ( atn_high() )
        {
                if ( jiffies > timeout )
                        panic("DSPLinux: Hid SPI send timeout (1)");
        }
                
        udelay(TMAC);


        // (3) The host starts placing the clock signals and data on the SCLK 
        //     and the MOSI lines respectively (8 pulses for 8 data bits).
        // (4) After the completion of one byte transfer the host continues 
        //     with the remaining bytes of the packet until the packet 
        //     transfer is complete. The _SS line is kept low to indicate to 
        //     the slave that more bytes are following. At least 150ms must 
        //     elapse between the beginning of one byte transfer and the 
        //     beginning of the next byte transfer so the slave has time to 
        //     process each byte. 
        // (5) The whole packet is transmitted. 
        while ( nbytes > 0 )
        {
                DBG("byte: 0x%x\n", *next_byte);

                next_bit = 8;
                
                while ( next_bit > 0 )
                {
                     bit_val = (*next_byte >> (next_bit - 1)) & 1;

                     reg_val = spi_read();

                     if ( bit_val )
                             reg_val |= MOSI_HI;
                     else
                             reg_val &= MOSI_LOW;

                     // clock low
                     spi_write(reg_val & CLK_LOW);
                     udelay(TSLOW);

                     // clock high
                     spi_write(reg_val | CLK_HIGH);
                     udelay(TSLOW);
                     
                     next_bit--;
                }  
       
                next_byte++;
                nbytes--;

                // wait TMIB between bytes
                udelay(TMIB);
        }

        // (6) After the last byte of the packet is transferred the host 
        //     de-asserts the _SS line to indicate that the transfer has been 
        //     completed. 
 
        spi_write(reg_val & _SS_HIGH);

        // (7) The slave releases the _ATN line indicating it is busy. 

        // wait for ATN to be DE-ASSERTED (i.e. HIGH - bit 5 == 0)
        timeout = jiffies + 10;
        while ( !atn_high() )
        {
                if ( jiffies > timeout )
                        panic("DSPLinux: Hid SPI send timeout (2)");
        }

        // (8) If the host has another packet to send it can only assert _SS 
        //     after _ATN has been de-asserted for at least 120ms. This is 
        //     necessary in order to provide the USAR Juno TM with sufficient 
        //     time to process the received packet and assert _ATN associated 
        //     with a Resend request if the packet was received in error
        
        udelay(TSNEXT);
}

/*****************************************************************************
 USAR doc8-007-001-tr-0721.pdf

 (1) The slave asserts the _ATN signal to alert the host that data are ready 
     for transfer and waits for a _SS signal for a maximum of 5 ms
 (2) Host asserts _SS to select the slave and indicate that clock pulses will 
     be issued. The slave prepares the output register (MISO) with the first 
     byte to be transferred.
 (3) The host starts producing clock pulses. The slave s outputs each bit at 
     the falling edge of each clock pulse.
 (4) The host receives one byte. If the _ATN signal is still low the host 
     starts providing clocks in order to read subsequent bytes. At least 150ms
     must elapse between the beginning of one byte transfer and the beginning 
     of the next byte transfer. 
 (5) (6) When all the packet data have been transferred the slave de-asserts 
     the _ATN signal to indicate that no more data are available. 
 (7) The host de-asserts the _SS line to release the bus. 
 (8) The slave may start a new transfer after the _SS has been de-asserted for
     at least 120ms.
 *****************************************************************************/

void spi_rcv_packet(u8 *buffer, u8 *num_bytes)
{
        u8 reg_val;
        u8 *next_byte;
        u8 next_bit;
        u8 reg_out;
        static unsigned long timeout;

        next_byte = buffer;

        // (1) The slave asserts the _ATN signal to alert the host that data 
        //     are ready for transfer and waits fora _SS signal for a maximum 
        //     of 5 ms

        // wait for ATN to be ASSERTED (i.e. LOW - bit 5 == 1)
        timeout = jiffies + 10;
        while ( atn_high() )
        {
                if ( jiffies > timeout )
                        panic("DSPLinux: Hid SPI receive timeout");
        }
        
        // (2) Host asserts _SS to select the slave and indicate that clock 
        //     pulses will be issued. The slave prepares the output register 
        //     (MISO) with the first byte to be transferred.

        reg_val = spi_read();
        spi_write(reg_val | _SS_LOW);

        // wait TMAC before reading first byte
        udelay(TMAC);

        // (3) The host starts producing clock pulses. The slave s outputs 
        //     each bit at the falling edge of each clock pulse.
        // (4) The host receives one byte. If the _ATN signal is still low the
        //     host starts providing clocks in order to read subsequent bytes.
        //     At least 150ms must elapse between the beginning of one byte 
        //     transfer and the beginning of the next byte transfer. 
        // (5) (6) When all the packet data have been transferred the slave 
        //     de-asserts the _ATN signal to indicate that no more data are 
        //     available. 

        reg_val = spi_read();

        while ( !atn_high() )
        {
                next_bit = 8;
                reg_out  = 0;

                while ( next_bit > 0 )
                {
                        spi_write(reg_val & CLK_LOW);
                        udelay(TSLOW);

                        spi_write(reg_val | CLK_HIGH);
                        udelay(TSLOW);

                        // receive bit
                        reg_val = spi_read();
                        udelay(TSLOW);
        
                        reg_out <<= 1;

                        if ( reg_val & MISO_BIT )                
                                reg_out |= 1;
 
                        next_bit--;
                        udelay(TSNA);
                }
                udelay(TMIB);

                DBG("reg_out: 0x%x\n", reg_out);

                *next_byte++  = reg_out;
                (*num_bytes)++;
        }

        // (7) The host de-asserts the _SS line to release the bus. 
        spi_write(reg_val & _SS_HIGH);

        // (8) The slave may start a new transfer after the _SS has been 
        //     de-asserted for at least 120ms. 
        udelay(TSNEXT);
}

static int verify_init(u8 *buffer)
{
        st_simple *simple = (st_simple *) buffer;

        if ( simple->cmd_code != 0x01 )
                return -1;

        return 0;
}
        
int spi_init(void)
{
        u8 buffer[50];
        int nbytes;
        int ret;
        static unsigned long timeout;

        spi_reset();
        
        // wait for ATN to be DE-ASSERTED coming out of reset
        timeout = jiffies + 10;
        while ( !atn_high() )
        {
                if ( jiffies > timeout )
                {
                        printk("DSPLinux: spi timeout\n");
                        return -1;        
                }
        }

        // wait for ATN release to SS re-assertion = 120 microseconds
        udelay(150);

        send_juno_command(SIMPLE, 0, 0, 0, 0, NULL);

        if ( (ret = receive_juno_response(SIMPLE, (u8 *)&buffer, 
                                          (u8 *)&nbytes, 0) != 0) )
        {
                DBG(__FUNCTION__ " Error: Bad Juno return value: %d\n", 
                       ret);
                printk("DSPLinux: spi init failed\n");
                return -1;
        }
       
        if ( (ret = verify_init((u8 *)&buffer) != 0) )
        {
                printk("DSPLinux: spi init verify failed\n");
                printk("DSPLinux: spi init failed\n");
                return -1;
        }
        return 0;
}

/*---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
