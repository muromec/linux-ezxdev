/*
 * File: keypad.c
 *
 * omap710 keypad driver
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
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <asm/arch/irqs.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/hardware.h>

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#  define DBG(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args)
#  define ENTRY() DBG(":entry\n")
#else
#  define DBG(fmt, args...)
#  define ENTRY()
#endif

// MPUIO Configuration Registers
// PERSEUS_IO_CONF3
#define DUAL_MODE_CRESET_MASK 0xc000
// PERSEUS_IO_CONF4
#define DUAL_MODE_EXT_IO_2 0x0c
#define DUAL_MODE_EXT_IO_3 0x03
// PERSEUS_IO_CONF8
#define DUAL_MODE_I2C_MASK 0x30
#define DUAL_MODE_PWT_MASK 0x0c
// PERSEUS_IO_CONF9
#define DUAL_MODE_RESERVED_10 0xc0
#define DUAL_MODE_RESERVED_9  0x30
#define DUAL_MODE_UWIRE       0x0c
#define DUAL_MODE_EXTERN_MPU  0x03

// MPUIO Registers
#define DATA_INPUT_MPUIO        0xFFFCE000
#define DATA_OUTPUT_MPUIO       0xFFFCE004
#define DIRECTION_CONTROL_MPUIO 0xFFFCE008
#define INTERRUPT_CONTROL_MPUIO 0xFFFCE00C
#define INTERRUPT_MASK_MPUIO    0xFFFCE010
#define INTERRUPT_STATUS_MPUIO  0xFFFCE014
#define PIN_CONTROL_REG_MPUIO   0xFFFCE018

// GPIO Module 1 Registers
#define DATA_INPUT_GPIO1         0xFFFBC800
#define DATA_OUTPUT_GPIO1        0xFFFBC804
#define DIRECTION_CONTROL_GPIO1  0xFFFBC808
#define INTERRUPT_CONTROL_GPIO1  0xFFFBC80C
#define INTERRUPT_MASK_GPIO1     0xFFFBC810
#define INTERRUPT_STATUS_GPIO1   0xFFFBC814

// GPIO Module 2 Registers
#define DATA_INPUT_GPIO2         0xFFFBD000
#define DATA_OUTPUT_GPIO2        0xFFFBD004
#define DIRECTION_CONTROL_GPIO2  0xFFFBD008
#define INTERRUPT_CONTROL_GPIO2  0xFFFBD00C
#define INTERRUPT_MASK_GPIO2     0xFFFBD010
#define INTERRUPT_STATUS_GPIO2   0xFFFBD014

// GPIO Module 3 Registers
#define DATA_INPUT_GPIO3         0xFFFBD800
#define DATA_OUTPUT_GPIO3        0xFFFBD804
#define DIRECTION_CONTROL_GPIO3  0xFFFBD808
#define INTERRUPT_CONTROL_GPIO3  0xFFFBD80C
#define INTERRUPT_MASK_GPIO3     0xFFFBD810
#define INTERRUPT_STATUS_GPIO3   0xFFFBD814

#define ALL_COL_AT_ONE_MPUIO    0x000001E0  // MPUIO 5, 6, 7 & 8
#define ALL_COL_AT_ZERO_MPUIO   0x00000000

#define KP_ROWS	5
#define KP_COLS	4

/*
const char keypad[KP_ROWS][KP_COLS]=
{                       // Electrical & physical
        {'c','-',' ','^'},    // row 0 (highest)
        {'1','2','*','3'},    // row 1
        {'7','8','0','9'},    // row 2
        {'4','5','#','6'},    // row 3 (lowest)
        {'<','>',' ','v'}     // row 4
};
*/

/* Notes on the following table:
 * The numbers correspond to the correct scan codes.  
 *
 * The * and # are handled the following way: 
 *  a * is considered to be shift+8 and a # is shift+3.
 *  The shift scancodes are 2a (left) and 36 (right).
 *  Since we only have two of these anomalies to deal with
 *  now, i return a left shift for * and a right shift for #.
 *
 * The arrow keys are also double coded.  They are all prefaced
 *  by sending e0.  We just return the code sans e0.
 */
const char keypad[KP_ROWS][KP_COLS]=
{                       // Electrical & physical
        {0x0e, 0x0c, 0x00, 0x48 },    // row 0 (highest)
        {0x02, 0x03, 0x2a, 0x04 },    // row 1
        {0x08, 0x09, 0x0b, 0x0a },    // row 2
        {0x05, 0x06, 0x36, 0x07 },    // row 3 (lowest)
        {0x4b, 0x4d, 0x00, 0x50 }     // row 4
};

static void do_omap710_keypad_tasklet(unsigned long);

DECLARE_TASKLET(omap710_keypad_tasklet, do_omap710_keypad_tasklet, 0);

void handle_scancode(unsigned char scancode, int down);

static void handle_star(void)
{
        handle_scancode(0x2a, 1);
        handle_scancode(0x09, 1);
        handle_scancode(0x2a, 0);
        handle_scancode(0x09, 0);
}

static void handle_sharp(void)
{
        handle_scancode(0x36, 1);
        handle_scancode(0x04, 1);
        handle_scancode(0x36, 0);
        handle_scancode(0x04, 0);
}

static void handle_arrow(unsigned char key)
{
        handle_scancode(0xe0, 1);
        handle_scancode(key, 1);
        handle_scancode(0xe0, 0);
        handle_scancode(key, 0);
}

static void set_col(u32 col)
{
        outl((1 << (col+5)), DATA_OUTPUT_MPUIO);
}

static int read_row(u32 row)
{
        int status = 0;

        switch ( row )
        {
        case 0: // GPIO_51
                status = (inl(DATA_INPUT_GPIO2) & 0x080000);
                break;
        case 1: // MPUS_IO_4
                status = (inl(DATA_INPUT_MPUIO) & 0x0010);
                break;
        case 2: // MPUS_IO_13
                status = (inl(DATA_INPUT_MPUIO) & 0x2000);
                break;
        case 3: // GPIO_70
                status = (inl(DATA_INPUT_GPIO3) & 0x0040);
                break;
        case 4: // GPIO_71
                status = (inl(DATA_INPUT_GPIO3) & 0x0080); 
                break;
        } // end switch

        return status;
}

static void do_omap710_keypad_tasklet(unsigned long data)
{ 
        u8 key = 0;
        u32 col;
        u32 row;
        u32 reg;
        u16 regw;
        int loop_wait;

        // unknown purpose - TI did it
        for ( loop_wait = 0; loop_wait < 0x00ff; loop_wait++ );

        // set output columns to zero
        outl(ALL_COL_AT_ZERO_MPUIO, DATA_OUTPUT_MPUIO);        

        for ( col = 0; col < KP_COLS; col++ )
        {
                set_col(col);
                for ( row = 0; row < KP_ROWS; row++ )
                {
                        if ( read_row(row) )
                        {
                                DBG("row: %d, col: %d\n", row, col);
                                key = keypad[row][col];
                                break;
                        } // end if
                } // end for

                if ( key ) 
                        break;
        } // end for

        if ( key == 0 )
                goto end; /* spurious interrupt? */

        // wait for key free
        {
                u16 reg1;
                u16 reg2;
                u16 reg3;
                do
                {
                        reg1 = inl(DATA_INPUT_MPUIO);
                        reg1 &= 0x2010;
                        reg2 = inl(DATA_INPUT_GPIO2);
                        reg2 &= 0x80000;
                        reg3 = inl(DATA_INPUT_GPIO3);
                        reg3 &= 0x00c0;
                }
                while ( reg1 || reg2 || reg3);
        }

        // unknown purpose - TI did it
        for ( loop_wait = 0; loop_wait < 0xffff; loop_wait++ );

        // send scancode
        if ( key == 0x2a )
                handle_star();
        else if ( key == 0x36 )
                handle_sharp();
        else if ( key >= 0x48 )
                handle_arrow(key);
        else
        {
                handle_scancode(key, 1);
                handle_scancode(key, 0);
        }
        
 end:

        // set output columns to one
        outl(ALL_COL_AT_ONE_MPUIO, DATA_OUTPUT_MPUIO);

        // unmask interrupt on row inputs
        reg = inl(INTERRUPT_MASK_MPUIO);
        outl(reg & 0xFFFFD7EF, INTERRUPT_MASK_MPUIO);
        reg = inl(INTERRUPT_MASK_GPIO2);
        outl(reg & 0xFFF7FFFF, INTERRUPT_MASK_GPIO2);
        reg = inl(INTERRUPT_MASK_GPIO3);
        outl(reg & 0xFFFFFF3F, INTERRUPT_MASK_GPIO3);
}

static void omap710_keypad_interrupt(int irq, void *dev_id, 
                                      struct pt_regs *regs) 
{
        u32 reg;

        // mask interrupt
        reg = inl(INTERRUPT_MASK_MPUIO);
        outl(reg | 0x2010, INTERRUPT_MASK_MPUIO);
        reg = inl(INTERRUPT_MASK_GPIO2);
        outl(reg | 0x80000, INTERRUPT_MASK_GPIO2);
        reg = inl(INTERRUPT_MASK_GPIO3);
        outl(reg | 0xc0, INTERRUPT_MASK_GPIO3);

        // acknowledge interrupt
        reg = inl(INTERRUPT_STATUS_MPUIO);
        outl(reg, INTERRUPT_STATUS_MPUIO);
        reg = inl(INTERRUPT_STATUS_GPIO2);
        outl(reg, INTERRUPT_STATUS_GPIO2);
        reg = inl(INTERRUPT_STATUS_GPIO3);
        outl(reg, INTERRUPT_STATUS_GPIO3);

        // schedule tasklet
        omap710_keypad_tasklet.data = 0;
        tasklet_schedule(&omap710_keypad_tasklet);
}

static int __init omap710_keypad_init(void)
{
        u32 regl;
        u16 regw;

        ENTRY();
        printk(KERN_INFO "DSPLinux keypad driver (c) 2002 RidgeRun, Inc.\n");

        /* Configure the dual MPIO registers */
        regw = inw(PERSEUS_IO_CONF4);
        regw |= DUAL_MODE_EXT_IO_2; // GPIO_70
        regw |= DUAL_MODE_EXT_IO_3; // GPIO_71
        outw(regw, PERSEUS_IO_CONF4);

        regw = inw(PERSEUS_IO_CONF8);
        regw |= DUAL_MODE_I2C_MASK; // MPUS_GPIO_5 & MPUS_GPIO_6
        regw |= DUAL_MODE_PWT_MASK; // MPUS_GPIO_7
        //regw = 0xf;
        outw(regw, PERSEUS_IO_CONF8);
  
        regl = inl(PERSEUS_IO_CONF9);
        regl |= DUAL_MODE_RESERVED_10; // MPUS_GPIO_13
        regl |= DUAL_MODE_UWIRE;       // MPUS_GPIO_8
        regl |= DUAL_MODE_EXTERN_MPU;  // MPUS_GPIO_4
        outl(regl, PERSEUS_IO_CONF9);

        regl = inl(PERSEUS_IO_CONF3);
        regl |= DUAL_MODE_CRESET_MASK; // GPIO_51
        outl(regl, PERSEUS_IO_CONF3);

        /* Set Direction */
        // configure 5 Rows : MPUS IO 4, 13 + GPIO 51, 70, 71 as input
        regl = inl(DIRECTION_CONTROL_MPUIO);
        outl(regl | 0x00002010, DIRECTION_CONTROL_MPUIO);
        // configure 4 columns : MPUS IO 5, 6, 7, 8 as output
        regl = inl(DIRECTION_CONTROL_MPUIO);
        outl(regl & 0xfffffe1f, DIRECTION_CONTROL_MPUIO);

        // GPIO 51 as input
        regl = inl(DIRECTION_CONTROL_GPIO2);
        outl(regl | 0x80000, DIRECTION_CONTROL_GPIO2);
        // GPIO 70 & 71 as input
        regw = inw(DIRECTION_CONTROL_GPIO3);
        outw(regw | 0x000000c0, DIRECTION_CONTROL_GPIO3);

        /* Set Output */
        // set output columns to one
        outl(ALL_COL_AT_ONE_MPUIO, DATA_OUTPUT_MPUIO);

        /* Set Interrupt Mask */
        // unmask interrupt on row inputs
        regl = inl(INTERRUPT_MASK_MPUIO);
        outl(regl & 0xFFFFD7EF, INTERRUPT_MASK_MPUIO);
        regl = inl(INTERRUPT_MASK_GPIO2);
        outl(regl & 0xFFF7FFFF, INTERRUPT_MASK_GPIO2);
        regl = inl(INTERRUPT_MASK_GPIO3);
        outl(regl & 0xFFFFFF3f, INTERRUPT_MASK_GPIO3);

        /* Set Interrupt Control */
        // configure clock on rising edge on all row inputs
        regl = inl(INTERRUPT_CONTROL_MPUIO);
        outl(regl | 0x00002010, INTERRUPT_CONTROL_MPUIO);
        regl = inl(INTERRUPT_CONTROL_GPIO2);
        outl(regl | 0x80000, INTERRUPT_CONTROL_GPIO2);
        regl = inl(INTERRUPT_CONTROL_GPIO3);
        outl(regl | 0x000000c0, INTERRUPT_CONTROL_GPIO3);

        if ( request_irq(INT_GPIO, omap710_keypad_interrupt, 
                         0, "keypad_mpus", 0) < 0 )
                return -EINVAL;

        if ( request_irq(INT_MPUIO2, omap710_keypad_interrupt, 
                         0, "keypad_gpio2", 0) < 0 )
                return -EINVAL;

        if ( request_irq(INT_MPUIO3, omap710_keypad_interrupt, 
                         0, "keypad_gpio3", 0) < 0 )
                return -EINVAL;

        return 0;
}

static void __exit omap710_keypad_exit(void)
{
        ENTRY();

        free_irq(INT_GPIO, 0);
        free_irq(INT_MPUIO2, 0);
        free_irq(INT_MPUIO3, 0);

        return;
}

module_init(omap710_keypad_init);
module_exit(omap710_keypad_exit);

MODULE_AUTHOR("Alex McMains <aam@rigderun.com>");
MODULE_DESCRIPTION("DSPLinux OMAP710 Keypad driver");

/*
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
