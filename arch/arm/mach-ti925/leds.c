/*
 *  linux/arch/arm/mach-ti925/leds.c
 *
 * BRIEF MODULE DESCRIPTION
 *   TI925 7-line LED driver
 *
 * Copyright (C) 2000 RidgeRun, Inc. (http://www.ridgerun.com)
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
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
 */
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/leds.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/arch/hardware.h>

#define LED_WRITE(v)  outb((v),(IO_ADDRESS(TI925_FP_LED)))
#define LED_READ()    inb(IO_ADDRESS((TI925_FP_LED)))

#define MAX_LED_STATE 8
static unsigned char led_array[MAX_LED_STATE] =
    { 0x3, 0x7, 0x6, 0x16, 0x18, 0x38, 0x30, 0x70 };

static void
ti925_leds_event(led_event_t ledevt)
{
	static int led_state = 0;
	unsigned long led_mask;
	unsigned long new_led_mask;
	unsigned long flags;

	local_irq_save(flags);
	switch (ledevt) {
	case led_idle_start:
		led_mask = LED_READ() & 0xfe;
		LED_WRITE(led_mask);
		break;

	case led_idle_end:
		led_mask = LED_READ() | 0x1;
		LED_WRITE(led_mask);
		break;

	case led_timer:
		led_mask = LED_READ() & 0x1;
		led_state++;

		LED_WRITE(~(led_array[led_state++ % MAX_LED_STATE]) | led_mask);
		break;

	default:
		break;
	}
	local_irq_restore(flags);
}

static int __init
leds_init(void)
{
	if (machine_is_ti925())
		leds_event = ti925_leds_event;

	return 0;
}

__initcall(leds_init);
