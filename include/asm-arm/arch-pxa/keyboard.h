/*
 * include/asm-arm/arch-pxa/keyboard.h
 * BRIEF MODULE DESCRIPTION
 *	Keyboard-related definitions for different PXA boards.
 *
 * Original file copyright information is missed.
 *
 * Changes copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */
/*
 *  linux/include/asm-arm/arch-pxa/keyboard.h
 *
 *  This file contains the architecture specific keyboard definitions
 */

#ifndef _PXA_KEYBOARD_H
#define _PXA_KEYBOARD_H

#include <linux/config.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>

extern struct kbd_ops_struct *kbd_ops;

#define kbd_disable_irq()	do { } while(0);
#define kbd_enable_irq()	do { } while(0);

#define kbd_request_region()  (0)
#define kbd_request_irq(handler) (0)

#define kbd_read_input()  (0)
#define kbd_read_status() (0)
#define kbd_write_output(val) (0)
#define kbd_write_command(val) (0)

#define kbd_pause() do { } while(0)

#define aux_request_irq(hand,dev_id) (0)
#define aux_free_irq(dev_id) (0)

extern int sa1111_kbd_init_hw(void);

extern int pckbd_setkeycode(unsigned int scancode, unsigned int keycode);
extern int pckbd_getkeycode(unsigned int scancode);
extern int pckbd_translate(unsigned char scancode, unsigned char *keycode,
                           char raw_mode);
extern char pckbd_unexpected_up(unsigned char keycode);
extern void pckbd_leds(unsigned char leds);
extern void pckbd_init_hw(void);
extern unsigned char pckbd_sysrq_xlate[128];


static inline void kbd_init_hw(void)
{
	if (machine_is_lubbock())
		sa1111_kbd_init_hw();
	if (machine_is_mainstone()) {
		k_setkeycode	= pckbd_setkeycode;
		k_getkeycode	= pckbd_getkeycode;
		k_translate	= pckbd_translate;
		k_unexpected_up	= pckbd_unexpected_up;
		k_leds		= pckbd_leds;
#ifdef CONFIG_MAGIC_SYSRQ
		k_sysrq_key     = 0x54;
		k_sysrq_xlate   = pckbd_sysrq_xlate;
#endif
	}
}


#endif  /* _PXA_KEYBOARD_H */

