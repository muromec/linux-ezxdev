/*
 * File: keypad.c
 *
 * omap1510 keypad driver
 * 
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: Greg Lonnon <glonnon@ridgerun.com>
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

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#  define DBG(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args)
#  define ENTRY() DBG(":entry\n")
#else
#  define DBG(fmt, args...)
#  define ENTRY()
#endif

#define ARMIO_KBD_DCR     OMAP1510_ARMIO_BASE + 0x10
#define ARMIO_KBD_DOR     OMAP1510_ARMIO_BASE + 0x8
#define ARMIO_KBD_IMR     OMAP1510_ARMIO_BASE + 0x20
#define ARMIO_KBD_ICR     OMAP1510_ARMIO_BASE + 0x18

#define ARMIO_KBD_MASKIT  (OMAP1510_ARMIO_BASE + 0x28)
#define ARMIO_KBD_LATCH   (OMAP1510_ARMIO_BASE + 0x10)  
#define ARMIO_KBD_ROW     ARMIO_KBD_LATCH
#define ARMIO_KBD_REG     (OMAP1510_ARMIO_BASE + 0x14) 
#define ARMIO_KBD_COL     ARMIO_KBD_REG
#define GPIO_DEBOUNCING_REG  OMAP1510_ARMIO_BASE + 0x30

static struct timer_list timer;

#define KP_ROWS	5
#define KP_COLS	6

#define KEYPAD_DEBOUNCE_TIME 25

/*
const char keypad_map[KP_ROWS * KP_COLS] =
{                             // Electrical & physical
  'c','-',' ','^',' ',' ',    // row 0 (highest)
  '1','2','*','3',' ',' ',    // row 1
  '7','8','0','9',' ',' ',    // row 2
  '4','5','#','6',' ',' ',    // row 3
  '<','>',' ','v',' ',' ',    // row 4 (lowest)
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
const char keypad_map[KP_ROWS * KP_COLS] =
{                          // Electrical & physical
        0x0e,0x0c,0x00,0x48,0x00,0x00, // row 0 (highest)
        0x02,0x03,0x2a,0x04,0x00,0x00, // row 1
        0x08,0x09,0x0b,0x0a,0x00,0x00, // row 2
        0x05,0x06,0x36,0x07,0x00,0x00, // row 3
        0x4b,0x4d,0x00,0x50,0x00,0x00, // row 4 (lowest)
};

static void do_omap1510_keypad_tasklet(unsigned long);

DECLARE_TASKLET(omap1510_keypad_tasklet,do_omap1510_keypad_tasklet,0);

void handle_scancode(unsigned char scancode, int down);

#define ALL_COL_AT_ONE  (0xF << KP_ROWS)
#define ALL_COL_AT_ZERO (~(0xF << KP_ROWS))

static int omap1510_keypad_read(void)
{
        int row = 0;
        int col = -1;
        int ret;
        int row_number;
        int col_number;
        int save_row=0x1F;
        int read_row=0x1F;

        save_row = inw(ARMIO_KBD_ROW) & 0x1F;

        for (row_number=0 ; row_number<KP_ROWS ; row_number++) {
                if ((inw(ARMIO_KBD_ROW) & (1<< row_number)) == 0) {
                        row = row_number;
                }
        }
        
        for (col_number=0 ; col_number<KP_COLS ; col_number++) {
                int loop_wait;
                outw((1 << col_number) & 0x3f, ARMIO_KBD_COL);
                // Wait for stable status
                
                udelay(1);
                
                read_row = inw(ARMIO_KBD_ROW) & 0x1F;  
                DBG("save row %x read_row %x\n",save_row,read_row);
                if (read_row != save_row) {  
                        col = col_number;
                }
        }

        if(row > KP_ROWS)
                return -EINVAL;

        if(col > KP_COLS)
                return -EINVAL;

        if ( col == -1 )
                return -EINVAL;

        ret = keypad_map[row * KP_ROWS + row + col];
        
        DBG("row %d col %d\n", row, col);

        return ret;
}

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

static void do_omap1510_keypad_tasklet(unsigned long data)
{ 
        unsigned char key;
        int ret;

        ENTRY();
        omap1510_keypad_tasklet.data = 1;

        if ( data == 0 ) 
        {  // this is an interrupt
                ret = omap1510_keypad_read();
                if ( ret < 0 )
                        goto reset_timer;

                key = (unsigned char) ret;
                timer.data = (unsigned long) key;          
                
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

                DBG("key %c\n", key);
        } 
        else 
        {  // this is a timer
                data = timer.data;
                key = omap1510_keypad_read();
                if ( key == data ) 
                {
                        DBG("same key is still down, reset timer and go\n");
                }
                else 
                {
                        DBG("different key is down, turn on int\n");
                        outw(0x0,ARMIO_KBD_MASKIT);                
                        return;
                }
        }

 reset_timer:
        mod_timer(&timer, jiffies + KEYPAD_DEBOUNCE_TIME);
        return;

}
static void omap1510_timer(unsigned long null) 
{
        ENTRY();
        omap1510_keypad_tasklet.data = 1;
        tasklet_schedule(&omap1510_keypad_tasklet);
}
static void omap1510_keypad_interrupt(int irq, void *dev_id, 
                                      struct pt_regs *regs) 
{
        ENTRY();
        outw(1,ARMIO_KBD_MASKIT);
        omap1510_keypad_tasklet.data = 0;
        tasklet_schedule(&omap1510_keypad_tasklet);
}

static int __init omap1510_keypad_init(void)
{
        ENTRY();
        printk(KERN_INFO "DSPLinux keypad driver (c) 2001 RidgeRun, Inc.\n");

        outw(0xff,GPIO_DEBOUNCING_REG);        
        outw(0x1f,ARMIO_KBD_DCR);
        outw(ALL_COL_AT_ONE, ARMIO_KBD_DOR);
        outw(0xe0,ARMIO_KBD_IMR);
        outw(0x1f,ARMIO_KBD_ICR);

        timer.data = 0;
        timer.function = omap1510_timer;

        if ( request_irq(INT_KEYBOARD, omap1510_keypad_interrupt, 
                         0, "keypad", 0) < 0 )
		return -EINVAL;

        return 0;
}

static void __exit omap1510_keypad_exit(void)
{
        ENTRY();

        free_irq(INT_KEYBOARD, 0);

        return;
}

module_init(omap1510_keypad_init);
module_exit(omap1510_keypad_exit);

MODULE_AUTHOR("Greg Lonnon <glonnon@rigderun.com>");
MODULE_DESCRIPTION("DSPLinux OMAP1510 Keypad driver");

/*
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
