/*
 * File: omap1510_ps2.c
 *
 * The USAR driver layer for the HID (Human Input Device) controller 
 *   for the TI P1
 *
 * REFERENCES
 *   Technical Reference Manual USAR Juno UR8HC007-001: doc8-007-001-tr-072
 *   Extended Protocol for PS/2 Pointing Devices: doc7-tsd-001-tr-080
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
#include <linux/ptrace.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <asm/arch/irqs.h>
#include <asm/irq.h>
#include <asm/io.h>
#include "omap1510_hid.h"

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#  define DBG(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
#  define ENTRY() DBG(":entry\n")
#else
#  define DBG(fmt, args...) ;
#  define ENTRY() ;
#endif

extern void spi_snd_packet(u8 *databyte, u8 num_bytes);
extern void spi_rcv_packet(u8 *buffer, u8 *num_bytes);
extern int spi_init(void);
extern unsigned long atn_high(void);
extern void send_juno_command(u8 type, u8 reg_offset, u8 data1, u8 data2, 
                              int length, u8 *block);
extern int receive_juno_response(u8 type, u8 *buffer, u8 *nbytes, u8 data);

// module_parm support values
static int mouse = 0;
static int kbd   = 0;
static int ts    = 0;

// used to handle e0 codes (except usar 100 & 101) 
// i divided these codes arbitrarly to fit within two 32 bit numbers (AAM)
static unsigned long int e0_codes1 = 0x030003ff;
static unsigned long int e0_codes2 = 0x038e0a00;

// i made this table using usar doc8-007-001-TR0-072 Appendix A
// it converts usar numbers to XT PS2 scancodes (AAM)
static unsigned char usar2scancode[128] =
{
	0x00, 0x29, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x2b, 0x1e, 0x1f, 0x20,
	0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
	0x1c, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32,
	0x33, 0x34, 0x35, 0x39, 0x01, 0x52, 0x53, 0x4b,
	0x47, 0x4f, 0x48, 0x50, 0x49, 0x51, 0x4d, 0x37,
	0x4e, 0x4f, 0x50, 0x51, 0x4b, 0x4c, 0x4d, 0x47,	
	0x48, 0x49, 0x52, 0x53, 0x4a, 0x1c, 0x35, 0x3b,
	0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
	0x44, 0x57, 0x58, 0x2a, 0x36, 0x38, 0x38, 0x1d,
	0x1d, 0x3a, 0x45, 0x46, 0x2a, 0x1d, 0x5b, 0x5c,
	0x5d, 0xff, 0x00, 0x00, 0x5e, 0x5f, 0x63, 0x70,
	0x7b, 0x79, 0x7d, 0x73, 0x5b, 0x5c, 0x5d, 0x63, 
	0x65, 0x66, 0x68, 0x69, 0x6b, 0x56, 0x54, 0x00
};

// from drivers/input/ps2serkbd.c
static unsigned char ps2_kbd_keycode[512] = {
    0, 67, 65, 63, 61, 59, 60, 88, 0, 68, 66, 64, 62, 15, 41, 85,
    0, 56, 42, 0, 29, 16, 2, 89, 0, 0, 44, 31, 30, 17, 3, 90,
    0, 46, 45, 32, 18, 5, 4, 91, 0, 57, 47, 33, 20, 19, 6, 0,
    0, 49, 48, 35, 34, 21, 7, 0, 0, 0, 50, 36, 22, 8, 9, 0,
    0, 51, 37, 23, 24, 11, 10, 0, 0, 52, 53, 38, 39, 25, 12, 0,
    122, 89, 40,120, 26, 13, 0, 0, 58, 54, 28, 27, 0, 43, 0, 0,
    85, 86, 90, 91, 92, 93, 14, 94, 95, 79, 0, 75, 71,121, 0,123,
    82, 83, 80, 76, 77, 72, 1, 69, 87, 78, 81, 74, 55, 73, 70, 99,
    252, 0, 0, 65, 99, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,251, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    252,253, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    254, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,255,
    0, 0, 92, 90, 85, 0,137, 0, 0, 0, 0, 91, 89,144,115, 0,
    136,100,255, 0, 97,149,164, 0,156, 0, 0,140,115, 0, 0,125,
    0,150, 0,154,152,163,151,126,112,166, 0,140, 0,147, 0,127,
    159,167,139,160,163, 0, 0,116,158, 0,150,165, 0, 0, 0,142,
    157, 0,114,166,168, 0, 0, 0,155, 0, 98,113, 0,148, 0,138,
    0, 0, 0, 0, 0, 0,153,140, 0, 0, 96, 0, 0, 0,143, 0,
    133, 0,116, 0,143, 0,174,133, 0,107, 0,105,102, 0, 0,112,
    110,111,108,112,106,103, 0,119, 0,118,109, 0, 99,104,119
};

#undef BLOCK_SIZE
#define BLOCK_SIZE     32
#define BUFFER_SIZE    50
#define CHIP_WAIT_TIME 1000000

static int check_device_return(u8 *buffer)
{
        int ret;
        u8 nbytes;

        if ( (ret = receive_juno_response(REPORT_BLOCK, buffer, &nbytes, 0)) 
             != 0 )
        {
                DBG(KERN_WARNING __FUNCTION__ " Error: Bad Juno return value: %d\n",ret);
                return ret;
        }
        else
        {
                if ( buffer[4] != MODE_ACK )
                {
                        DBG(KERN_WARNING "hid device returned error: %x, %x, %x, %x, 
                               %x, %x\n", buffer[0], buffer[1], buffer[2], 
                               buffer[3], buffer[4], buffer[5]);   

                        return -E_BAD_ACK;
                }
                else
                        DBG("Device Return Successful!\n");
        }

	return 0;
}

int mode_switch_to_absolute(void)
{
        u8 buffer[BUFFER_SIZE];
        u8 block[BLOCK_SIZE];
        u8 nbytes;
        int ret;
        int i   = 0;
        int err = 0;

        DBG("Touchscreen: waiting for internal ps/2 chip.");
        // Make sure the GPE register has gotten an internal ps/2 connect 
        // (IP_CON)
        while ( i < CHIP_WAIT_TIME ) // Give the chip time to come up
        {
                send_juno_command(READ_REGISTER, GPE_STATUS_0_REGISTER, 0, 0, 
                                  0, NULL);
  
                if ( (ret = receive_juno_response(REPORT_REGISTER, 
                                                  (u8 *)&buffer, &nbytes, 
                                                  IP_CON) != 0) )
                        i++;
                else
                        break;
        }
        DBG("\n");

        if ( i >= CHIP_WAIT_TIME ) 
        {
                printk(KERN_DEBUG "Internal pointing device not detected\n");
                return -1;
        }
        else 
                DBG("Touchscreen: Internal PS/2 connection established\n");

        // Turn on report mode
        send_juno_command(WRITE_REGISTER, PS2_DEVICE_DIRECT_CONTROL_REGISTER, 
                          REP_MODE, 0, 0, NULL);
        send_juno_command(READ_REGISTER, PS2_DEVICE_DIRECT_CONTROL_REGISTER, 
                          0, 0, 0, NULL);
        if ( (ret = receive_juno_response(REPORT_REGISTER, (u8 *)&buffer, 
                                          &nbytes, REP_MODE) != 0) )
        {
                printk(KERN_DEBUG __FUNCTION__ "Error: Bad Juno return value: %d\n", ret);
        }

        // Indicate that the Control Register (Direct Port) should be enabled 
        // Send first "Set Sampling Rate (0xF3) via block write.
        DBG("ENABLE PS/2\n");        
        block[0] = PS2_DEVICE_ENABLE;
        block[1] = SET_SAMPLING_RATE;
        send_juno_command(WRITE_BLOCK, CONTROL_REGISTER, 0, 0, 2, 
                          (u8 *)&block);
        err += check_device_return(buffer);

        // Write 80 Repors/Second
        DBG("80 Repors/Second\n");
        send_juno_command(WRITE_REGISTER, HOST_TRANSMIT_REGISTER, REPORS_80, 
                          0, 0, NULL);
        err += check_device_return(buffer);

        // Write Set Sampling Rate
        DBG("Set Sampling Rate\n");
        send_juno_command(WRITE_REGISTER, HOST_TRANSMIT_REGISTER, 
                          SET_SAMPLING_RATE, 0, 0, NULL);
        err += check_device_return(buffer);

        // Write 60 Repors/Second
        DBG("60 Repors/Second\n");
        send_juno_command(WRITE_REGISTER, HOST_TRANSMIT_REGISTER, REPORS_60, 
                          0, 0, NULL);
        err += check_device_return(buffer);

        // Write Set Sampling Rate
        DBG("Set Sampling Rate\n");
        send_juno_command(WRITE_REGISTER, HOST_TRANSMIT_REGISTER, 
                          SET_SAMPLING_RATE, 0, 0, NULL);
        err += check_device_return(buffer);

        // Write 100 Repors/Second
        DBG("100 Repors/Second\n");
        send_juno_command(WRITE_REGISTER, HOST_TRANSMIT_REGISTER, REPORS_100, 
                          0, 0, NULL);
        err += check_device_return(buffer);

        // Write new command block for capturing the device ID
        // block[0] = 0x3; AAM - is it 2 or 3
        DBG("Device ID\n");
        block[0] = 0x3;
        block[1] = READ_DEVICE_TYPE;
        send_juno_command(WRITE_BLOCK, CONTROL_REGISTER, 0, 0, 2, 
                          (u8 *)&block);
        if ( (ret = check_device_return(buffer) != 0) )
        {
                printk(KERN_DEBUG "Error: Invalid device ID\n");
                err += -E_BAD_ACK;
        }
        else
        {
                if ( buffer[5] != VALID_DEVICE_ID )
                {
                        printk(KERN_DEBUG "Error: device ID: %x, %x, %x, %x, %x, %x\r\n", 
                               buffer[0], buffer[1], buffer[2], buffer[3], 
                               buffer[4], buffer[5]);

                        err += -E_BAD_DEVICE_ID;
                }
                else
                {
                        DBG("Touchscreen: Juno device ID successfully 
                                retrieved\n");
                }
        }

        // Set Sampling Rate 100 this time for real!  The block command also 
        // sets the length back to 1
        DBG("Set Sampling Rate\n");
        block[0] = PS2_DEVICE_ENABLE;
        block[1] = SET_SAMPLING_RATE;
        send_juno_command(WRITE_BLOCK, CONTROL_REGISTER, 0, 0, 2, 
                          (u8 *)&block);
        err += check_device_return(buffer);
  
        // Write xmit data 
        DBG("Set 0x64\n");
        send_juno_command(WRITE_REGISTER, HOST_TRANSMIT_REGISTER, 0x64, 0, 0, 
                          NULL);
        err += check_device_return(buffer);

        // Set Stream Mode 
        DBG("0xea\n");
        send_juno_command(WRITE_REGISTER, HOST_TRANSMIT_REGISTER, 0xea, 0, 0, 
                          NULL);
        err += check_device_return(buffer);
  
        // Enable Device 
        DBG("0xf4\n");
        send_juno_command(WRITE_REGISTER, HOST_TRANSMIT_REGISTER, 0xf4, 0, 0, 
                          NULL);
        err += check_device_return(buffer);

         // Set Byte Length to three
        DBG("0x3\n");
        send_juno_command(WRITE_REGISTER, CONTROL_REGISTER, 3, 0, 0, NULL);
        send_juno_command(READ_REGISTER, CONTROL_REGISTER, 0, 0, 0, NULL);
        //check_device_return(buffer);
        if ( (ret = receive_juno_response(REPORT_REGISTER, (u8 *)&buffer, 
                                          &nbytes, 3) != 0) )
        {
                printk(KERN_DEBUG __FUNCTION__ " Error: Bad Juno return"
                        " value: %d\n", ret);
                err += -E_BAD_VALUE;
        }

        if ( err == 0 )
                printk(KERN_DEBUG "Touchscreen: Switched to absolute mode\n");
        else
                printk(KERN_DEBUG "Touchscreen: Error could not switch to 
                                   absolute mode!\n");
        
        send_juno_command(WRITE_REGISTER, PS2_DEVICE_DIRECT_CONTROL_REGISTER, 
                          0, 0, 0, NULL);
        send_juno_command(READ_REGISTER, PS2_DEVICE_DIRECT_CONTROL_REGISTER, 
                          0, 0, 0, NULL);
        if ( (ret = receive_juno_response(REPORT_REGISTER, (u8 *)&buffer, 
                                          &nbytes, 0) != 0) )
        {
                printk(KERN_DEBUG __FUNCTION__ " Error: Bad Juno return value: %d\n", 
                       ret);
        }
        return 0;
} 

struct omap1510_hid_dev 
{
        struct input_dev mouse;
        struct input_dev kbd;
        int open;
        int abs_mode;
};

static void process_async_report(struct omap1510_hid_dev *);
static void do_hid_tasklet(unsigned long);
DECLARE_TASKLET(hid_tasklet,do_hid_tasklet,0);

static struct omap1510_hid_dev *hid;

static int omap1510_hid_interrupt(int irq, void *dev_id, 
                                  struct pt_regs *regs) 
{
        hid = (struct omap1510_hid_dev *) dev_id;
        if ( !atn_high() ) 
        {
                disable_irq(INT_FPGA_ATN);
                tasklet_schedule(&hid_tasklet);
        }

        return 0;
}

static void do_hid_tasklet(unsigned long unused) 
{
        process_async_report(hid);
        enable_irq(INT_FPGA_ATN);
}

static int omap1510_hid_open(struct input_dev *dev)
{
        extern int spi_init(void);
        ENTRY();

        if ( hid->open++ )
                return 0;

        if ( ! hid->abs_mode ) 
        { 

                if ( spi_init() < 0 ) 
                {
                        printk(KERN_WARNING "DSPLinux: unable to initialize 
                                             spi\n");
                        return -EINVAL;
                }
      
                if ( mode_switch_to_absolute() < 0 )
                        return -EINVAL;
        }

        hid->abs_mode = 1;

        if ( request_irq(INT_FPGA_ATN, (void *)omap1510_hid_interrupt,
                         SA_INTERRUPT, "hid", hid) < 0 )
                return -EINVAL;

        return 0;
} 

static void omap1510_hid_close(struct input_dev *dev) 
{
        if ( hid->open-- ) 
                return;

        if ( hid == NULL )
                return;

        kfree(hid);
}

static int  __init omap1510_ps2_init(void) 
{
        int i;

        ENTRY();
        printk(KERN_INFO "DSPLinux HID Driver (c) 2001 RidgeRun, Inc.\n");

#if CONFIG_DSPLINUX_OMAP1510_TOUCHSCREEN
        ts = 1;
#endif
#if CONFIG_DSPLINUX_OMAP1510_MOUSE
        mouse = 1;
#endif
#if CONFIG_DSPLINUX_OMAP1510_KEYBOARD
        kbd = 1;
#endif
        DBG("HID: ts %d mouse %d kbd %d\n",ts,mouse,kbd);

        if ( mouse && ts ) 
        {
                printk(KERN_WARNING "concurrent mouse and touchscreen not supported\n");
                return -EINVAL;
        }

        hid = kmalloc(sizeof(*hid),GFP_KERNEL);

        if ( hid == NULL ) 
        {
                printk(KERN_WARNING "unable to allocate space for HID device\n");
                return -ENOMEM;
        }

        memset(hid,0,sizeof(*hid));
        hid->open = 0;
        hid->abs_mode = 0;

        // setup the pen and mouse
        if ( ts ) 
        {
                hid->mouse.evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
                hid->mouse.keybit[LONG(BTN_MOUSE)] = BIT(BTN_LEFT) | 
                        BIT(BTN_RIGHT) | BIT(BTN_MIDDLE) | BIT(BTN_TOUCH);
                hid->mouse.absbit[0] = BIT(ABS_X) | BIT(ABS_Y);
                hid->mouse.absmax[ABS_X] = 1000;
                hid->mouse.absmin[ABS_X] = 1;
                hid->mouse.absmax[ABS_Y] = 1000;
                hid->mouse.absmin[ABS_Y] = 1;
                hid->mouse.absflat[0] = 1;
                hid->mouse.absfuzz[0] = 4;
        } 
        else if ( mouse ) 
        {

                hid->mouse.evbit[0] = BIT(EV_KEY) | BIT(EV_REL);
                hid->mouse.keybit[LONG(BTN_MOUSE)] = BIT(BTN_LEFT) | 
                        BIT(BTN_RIGHT) | BIT(BTN_MIDDLE) | BIT(BTN_TOUCH);
                hid->mouse.relbit[0] = BIT(REL_X) | BIT(REL_Y);
        }

        hid->mouse.private = hid;
        hid->mouse.open = omap1510_hid_open;
        hid->mouse.close = omap1510_hid_close;
        hid->mouse.name = "omap1510_mouse";
        hid->mouse.idbus = 0;
        hid->mouse.idvendor = 0;
        hid->mouse.idproduct = 0;
        hid->mouse.idversion = 0;

        input_register_device(&hid->mouse);
        
        if ( kbd ) 
        {
                // setup the keyboard
                hid->kbd.evbit[0] = BIT(EV_KEY);
                for (i = 0; i < 511; i++)
                        set_bit(ps2_kbd_keycode[i], hid->kbd.keybit);
                clear_bit(0, hid->kbd.keybit);
                hid->kbd.private = hid;
                hid->kbd.open = omap1510_hid_open;
                hid->kbd.close = omap1510_hid_close;
                hid->kbd.name = "omap1510_kbd";
                hid->kbd.idbus = 0;
                hid->kbd.idvendor = 0;
                hid->kbd.idproduct = 0;
                hid->kbd.idversion = 0;
       
                input_register_device(&hid->kbd);
        }
        omap1510_hid_open(&hid->mouse);

        return 0;
}

static void __exit omap1510_ps2_exit(void) 
{
        ENTRY();

        free_irq(INT_FPGA_ATN, hid);

        if ( hid != NULL )
                kfree(hid);

        return;
}

#define FUNCTION_BIT_0 (1<<0)
#define FUNCTION_BIT_1 (1<<1)
#define FUNCTION_BIT_2 (1<<2)
#define FUNCTION_MASK  (FUNCTION_BIT_0 | FUNCTION_BIT_1 | FUNCTION_BIT_2)
#define X_MSB_SHIFT    (8-4)
#define X_MSB_MASK     (3<<4)
#define Y_MSB_SHIFT    (8-6)
#define Y_MSB_MASK     (3<<6)

static void process_pointing_report(struct omap1510_hid_dev *hid, 
                                    u8 *buffer)
{
        int x;
        int y;
        int btn;
        int prev_x;
        int prev_y;
        int prev_btn;
        int functions;

        functions = buffer[1] & FUNCTION_MASK;

        if ( buffer[1] & (1<<3) )
        { // relative device
                if ( ts )
                {
                        DBG(KERN_WARNING "HID: relative packet, configured for" 
                               " absolute\n");
                        return;
                }

                x = buffer[2];
                y = buffer[3];

                // check the sign and convert from 2's complement if negative
                if ( buffer[1] & 0x10 )
                        x = ~(-x) - 255;
              
                // input driver wants -y
                if ( buffer[1] & 0x20 )
                        y = -(~(-y) - 255);
                else
                        y = -y;

                input_report_key(&hid->mouse, BTN_LEFT, 
                                 buffer[1] & FUNCTION_BIT_0);
                input_report_rel(&hid->mouse, REL_X, x);
                input_report_rel(&hid->mouse, REL_Y, y);
        }
        else
        { // absolute device
                if ( mouse )
					 {
                        DBG(KERN_WARNING "HID: absolute packet, configured for" 
                               " relative\n");
								return;
					 }

                x   = buffer[2] + ((buffer[1] & X_MSB_MASK) << X_MSB_SHIFT);
                y   = buffer[3] + ((buffer[1] & Y_MSB_MASK) << Y_MSB_SHIFT);
                btn = buffer[1] & FUNCTION_BIT_0;
 
                if ( (prev_x == x) && (prev_y == y) && (prev_btn == btn) )
                        return;
 
                input_report_key(&hid->mouse, BTN_LEFT, btn);
                input_report_abs(&hid->mouse, ABS_X, x);
                input_report_abs(&hid->mouse, ABS_Y, y);

                prev_x   = x; 
                prev_y   = y;
                prev_btn = btn;
        }

        DBG("HID X: %d Y: %d Functions: %x\n", x, y, functions);
}

/* NOTE: this function is taken verbatim from the standard kernel file:
 * drivers/char/pc_keyb.c.
 */

/*
 * Translation of escaped scancodes to keycodes.
 * This is now user-settable.
 * The keycodes 1-88,96-111,119 are fairly standard, and
 * should probably not be changed - changing might confuse X.
 * X also interprets scancode 0x5d (KEY_Begin).
 *
 * For 1-88 keycode equals scancode.
 */

#define E0_KPENTER 96
#define E0_RCTRL   97
#define E0_KPSLASH 98
#define E0_PRSCR   99
#define E0_RALT    100
#define E0_BREAK   101  /* (control-pause) */
#define E0_HOME    102
#define E0_UP      103
#define E0_PGUP    104
#define E0_LEFT    105
#define E0_RIGHT   106
#define E0_END     107
#define E0_DOWN    108
#define E0_PGDN    109
#define E0_INS     110
#define E0_DEL     111

#define E1_PAUSE   119

/*
 * The keycodes below are randomly located in 89-95,112-118,120-127.
 * They could be thrown away (and all occurrences below replaced by 0),
 * but that would force many users to use the `setkeycodes' utility, where
 * they needed not before. It does not matter that there are duplicates, as
 * long as no duplication occurs for any single keyboard.
 */
#define SC_LIM 89

#define FOCUS_PF1 85           /* actual code! */
#define FOCUS_PF2 89
#define FOCUS_PF3 90
#define FOCUS_PF4 91
#define FOCUS_PF5 92
#define FOCUS_PF6 93
#define FOCUS_PF7 94
#define FOCUS_PF8 95
#define FOCUS_PF9 120
#define FOCUS_PF10 121
#define FOCUS_PF11 122
#define FOCUS_PF12 123

#define JAP_86     124
/* tfj@olivia.ping.dk:
 * The four keys are located over the numeric keypad, and are
 * labelled A1-A4. It's an rc930 keyboard, from
 * Regnecentralen/RC International, Now ICL.
 * Scancodes: 59, 5a, 5b, 5c.
 */
#define RGN1 124
#define RGN2 125
#define RGN3 126
#define RGN4 127

static unsigned char high_keys[128 - SC_LIM] = {
  RGN1, RGN2, RGN3, RGN4, 0, 0, 0,                   /* 0x59-0x5f */
  0, 0, 0, 0, 0, 0, 0, 0,                            /* 0x60-0x67 */
  0, 0, 0, 0, 0, FOCUS_PF11, 0, FOCUS_PF12,          /* 0x68-0x6f */
  0, 0, 0, FOCUS_PF2, FOCUS_PF9, 0, 0, FOCUS_PF3,    /* 0x70-0x77 */
  FOCUS_PF4, FOCUS_PF5, FOCUS_PF6, FOCUS_PF7,        /* 0x78-0x7b */
  FOCUS_PF8, JAP_86, FOCUS_PF10, 0                   /* 0x7c-0x7f */
};

/* BTC */
#define E0_MACRO   112
/* LK450 */
#define E0_F13     113
#define E0_F14     114
#define E0_HELP    115
#define E0_DO      116
#define E0_F17     117
#define E0_KPMINPLUS 118
/*
 * My OmniKey generates e0 4c for  the "OMNI" key and the
 * right alt key does nada. [kkoller@nyx10.cs.du.edu]
 */
#define E0_OK  124
/*
 * New microsoft keyboard is rumoured to have
 * e0 5b (left window button), e0 5c (right window button),
 * e0 5d (menu button). [or: LBANNER, RBANNER, RMENU]
 * [or: Windows_L, Windows_R, TaskMan]
 */
#define E0_MSLW   125
#define E0_MSRW   126
#define E0_MSTM   127

static unsigned char e0_keys[128] = {
  0, 0, 0, 0, 0, 0, 0, 0,			      /* 0x00-0x07 */
  0, 0, 0, 0, 0, 0, 0, 0,			      /* 0x08-0x0f */
  0, 0, 0, 0, 0, 0, 0, 0,			      /* 0x10-0x17 */
  0, 0, 0, 0, E0_KPENTER, E0_RCTRL, 0, 0,	      /* 0x18-0x1f */
  0, 0, 0, 0, 0, 0, 0, 0,			      /* 0x20-0x27 */
  0, 0, 0, 0, 0, 0, 0, 0,			      /* 0x28-0x2f */
  0, 0, 0, 0, 0, E0_KPSLASH, 0, E0_PRSCR,	      /* 0x30-0x37 */
  E0_RALT, 0, 0, 0, 0, E0_F13, E0_F14, E0_HELP,	      /* 0x38-0x3f */
  E0_DO, E0_F17, 0, 0, 0, 0, E0_BREAK, E0_HOME,	      /* 0x40-0x47 */
  E0_UP, E0_PGUP, 0, E0_LEFT, E0_OK, E0_RIGHT, E0_KPMINPLUS, E0_END,/* 0x48-0x4f */
  E0_DOWN, E0_PGDN, E0_INS, E0_DEL, 0, 0, 0, 0,	      /* 0x50-0x57 */
  0, 0, 0, E0_MSLW, E0_MSRW, E0_MSTM, 0, 0,	      /* 0x58-0x5f */
  0, 0, 0, 0, 0, 0, 0, 0,			      /* 0x60-0x67 */
  0, 0, 0, 0, 0, 0, 0, E0_MACRO,		      /* 0x68-0x6f */
  0, 0, 0, 0, 0, 0, 0, 0,			      /* 0x70-0x77 */
  0, 0, 0, 0, 0, 0, 0, 0			      /* 0x78-0x7f */
};

static int pckbd_translate(unsigned char scancode, unsigned char *keycode,
		    char raw_mode)
{
	static int prev_scancode;

	/* special prefix scancodes.. */
	if (scancode == 0xe0 || scancode == 0xe1) {
		prev_scancode = scancode;
		return 0;
	}

	/* 0xFF is sent by a few keyboards, ignore it. 0x00 is error */
	if (scancode == 0x00 || scancode == 0xff) {
		prev_scancode = 0;
		return 0;
	}

	scancode &= 0x7f;

	if (prev_scancode) {
	  /*
	   * usually it will be 0xe0, but a Pause key generates
	   * e1 1d 45 e1 9d c5 when pressed, and nothing when released
	   */
	  if (prev_scancode != 0xe0) {
	      if (prev_scancode == 0xe1 && scancode == 0x1d) {
		  prev_scancode = 0x100;
		  return 0;
	      } else if (prev_scancode == 0x100 && scancode == 0x45) {
		  *keycode = E1_PAUSE;
		  prev_scancode = 0;
	      } else {
#ifdef KBD_REPORT_UNKN
		  if (!raw_mode)
		    printk(KERN_INFO "keyboard: unknown e1 escape sequence\n");
#endif
		  prev_scancode = 0;
		  return 0;
	      }
	  } else {
	      prev_scancode = 0;
	      /*
	       *  The keyboard maintains its own internal caps lock and
	       *  num lock statuses. In caps lock mode E0 AA precedes make
	       *  code and E0 2A follows break code. In num lock mode,
	       *  E0 2A precedes make code and E0 AA follows break code.
	       *  We do our own book-keeping, so we will just ignore these.
	       */
	      /*
	       *  For my keyboard there is no caps lock mode, but there are
	       *  both Shift-L and Shift-R modes. The former mode generates
	       *  E0 2A / E0 AA pairs, the latter E0 B6 / E0 36 pairs.
	       *  So, we should also ignore the latter. - aeb@cwi.nl
	       */
	      if (scancode == 0x2a || scancode == 0x36)
		return 0;

	      if (e0_keys[scancode])
		*keycode = e0_keys[scancode];
	      else {
#ifdef KBD_REPORT_UNKN
		  if (!raw_mode)
		    printk(KERN_INFO "keyboard: unknown scancode e0 %02x\n",
			   scancode);
#endif
		  return 0;
	      }
	  }
	} else if (scancode >= SC_LIM) {
	    /* This happens with the FOCUS 9000 keyboard
	       Its keys PF1..PF12 are reported to generate
	       55 73 77 78 79 7a 7b 7c 74 7e 6d 6f
	       Moreover, unless repeated, they do not generate
	       key-down events, so we have to zero up_flag below */
	    /* Also, Japanese 86/106 keyboards are reported to
	       generate 0x73 and 0x7d for \ - and \ | respectively. */
	    /* Also, some Brazilian keyboard is reported to produce
	       0x73 and 0x7e for \ ? and KP-dot, respectively. */

	  *keycode = high_keys[scancode - SC_LIM];

	  if (!*keycode) {
	      if (!raw_mode) {
#ifdef KBD_REPORT_UNKN
		  printk(KERN_INFO "keyboard: unrecognized scancode (%02x)"
			 " - ignored\n", scancode);
#endif
	      }
	      return 0;
	  }
 	} else
	  *keycode = scancode;
 	return 1;
}

static unsigned char handle_print_scr(struct omap1510_hid_dev *hid, 
                                      u8 *buffer,int up)
{
        unsigned char keycode; 

        if ( up )
        {
                pckbd_translate(0xe0, &keycode, 0);
                pckbd_translate(0xb7, &keycode, 0);  
                pckbd_translate(0xe0, &keycode, 0);
                pckbd_translate(0xaa, &keycode, 0);  
        }
        else
        {
                pckbd_translate(0xe0, &keycode, 0);
                pckbd_translate(0x2a, &keycode, 0);
                pckbd_translate(0xe0, &keycode, 0);
                pckbd_translate(0x37, &keycode, 0);
        }                

        return keycode;
}
 
static unsigned char handle_pause(struct omap1510_hid_dev *hid, u8 *buffer)
{
        unsigned char keycode;

        pckbd_translate(0xe1, &keycode, 0);
        pckbd_translate(0x1d, &keycode, 0);
        pckbd_translate(0x45, &keycode, 0);
        pckbd_translate(0xe1, &keycode, 0);
        pckbd_translate(0x9d, &keycode, 0);
        pckbd_translate(0xc5, &keycode, 0);

        return keycode;
}

static void process_keyboard_report(struct omap1510_hid_dev *hid, 
                                    u8 *buffer)
{
        unsigned char ch      = buffer[1] & 0x7f;
        int is_e0             = 0;
        unsigned char keycode = 0;

        if ( (ch == 106) || (ch == 107) )
                return; // no code

        if ( ch == 100 )
                keycode = handle_print_scr(hid, buffer,  buffer[1] & 0x80);
        
        if ( ch == 101 )
                keycode = handle_pause(hid, buffer);

        // first block of e0 codes
        if ( (ch >= 53) && (ch <= 84) )
                is_e0 = e0_codes1 & (1 << (ch - 53));

        // second block of e0 codes
        if ( (ch >= 85) && (ch <= 116) )
                is_e0 = e0_codes2 & (1 << (ch - 85));

        if ( is_e0 )
                pckbd_translate(0xe0, &keycode, 0);

        if ( !keycode )
                pckbd_translate(usar2scancode[ch], &keycode, 0);

        input_report_key(&hid->kbd, keycode, 
                         buffer[1] & 0x80 ? 0 : 1);

        DBG("HID: key: %d, state: %s\n", buffer[1] & 0x7f, 
            buffer[1] & 0x80 ? "up" : "down"); 
}

static void process_async_report(struct omap1510_hid_dev *hid)
{
        u8 buffer[BUFFER_SIZE];
        u8 nbytes;
        int ret;
        
        if ( (ret = receive_juno_response(ASYNC_REPORT, (u8 *)&buffer, 
                                          &nbytes, REP_MODE)) == -1 )
        {
                DBG(__FUNCTION__ " Error: Bad Juno return value: %d\n", 
                       ret);
        }

        if ( ret == KEYBOARD_REPORT )
                process_keyboard_report(hid, buffer);
        else if ( ret == POINTING_REPORT )
                process_pointing_report(hid, buffer);
        else
                DBG(KERN_INFO "ERROR: bad report\n");
}

module_init(omap1510_ps2_init);
module_exit(omap1510_ps2_exit);

MODULE_PARM(mouse,"i");
MODULE_PARM_DESC(mouse,"mouse support for DSPLinux HID controller");
MODULE_PARM(ts,"i");
MODULE_PARM_DESC(ts,"touchscreen support for DSPLinux HID controller");
MODULE_PARM(kbd,"i");
MODULE_PARM_DESC(kbd,"keyboard support for DSPLinux HID controller");

MODULE_AUTHOR("Alex McMains <aam@rigderun.com>");
MODULE_DESCRIPTION("DSPLinux HID Driver");

/*
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
