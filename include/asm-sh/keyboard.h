#ifndef	__ASM_SH_KEYBOARD_H
#define	__ASM_SH_KEYBOARD_H
/*
 *	$Id: keyboard.h,v 1.12 2001/09/06 04:01:41 gniibe Exp $
 */

#include <linux/kd.h>
#include <linux/config.h>
#include <asm/machvec.h>

#if defined(CONFIG_SH_7751_SOLUTION_ENGINE) || defined(CONFIG_SH_SOLUTION_ENGINE)
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <asm/io.h>

#define KEYBOARD_IRQ                    1
#define DISABLE_KBD_DURING_INTERRUPTS   0

extern int pckbd_setkeycode(unsigned int scancode, unsigned int keycode);
extern int pckbd_getkeycode(unsigned int scancode);
extern int pckbd_translate(unsigned char scancode, unsigned char *keycode,
                           char raw_mode);
extern char pckbd_unexpected_up(unsigned char keycode);
extern void pckbd_leds(unsigned char leds);
extern void pckbd_init_hw(void);
extern unsigned char pckbd_sysrq_xlate[128];

#define kbd_setkeycode          pckbd_setkeycode
#define kbd_getkeycode          pckbd_getkeycode
#define kbd_translate           pckbd_translate
#define kbd_unexpected_up       pckbd_unexpected_up
#define kbd_leds                pckbd_leds
#define kbd_init_hw             pckbd_init_hw
#define kbd_sysrq_xlate         pckbd_sysrq_xlate

#define SYSRQ_KEY 0x54

/* resource allocation */
#define kbd_request_region()
#define kbd_request_irq(handler) request_irq(KEYBOARD_IRQ, handler, 0, \
                                             "keyboard", NULL)

/* How to access the keyboard macros on this platform.  */
#define kbd_read_input() inb(KBD_DATA_REG)
#define kbd_read_status() inb(KBD_STATUS_REG)
#define kbd_write_output(val) outb(val, KBD_DATA_REG)
#define kbd_write_command(val) outb(val, KBD_CNTL_REG)

/* Some stoneage hardware needs delays after some operations.  */
#define kbd_pause() do { } while(0)

/*
 * Machine specific bits for the PS/2 driver
 */

#define AUX_IRQ 12

#define aux_request_irq(hand, dev_id)                                   \
        request_irq(AUX_IRQ, hand, SA_SHIRQ, "PS/2 Mouse", dev_id)

#define aux_free_irq(dev_id) free_irq(AUX_IRQ, dev_id)

#elif CONFIG_SH_EC3104
#include <asm/keyboard-ec3104.h>
#else
static __inline__ int kbd_setkeycode(unsigned int scancode,
				     unsigned int keycode)
{
    return -EOPNOTSUPP;
}

static __inline__ int kbd_getkeycode(unsigned int scancode)
{
    return scancode > 127 ? -EINVAL : scancode;
}

#ifdef CONFIG_SH_DREAMCAST
extern int kbd_translate(unsigned char scancode, unsigned char *keycode,
			 char raw_mode);
#else
static __inline__ int kbd_translate(unsigned char scancode,
				    unsigned char *keycode, char raw_mode)
{
    *keycode = scancode;
    return 1;
}
#endif

static __inline__ char kbd_unexpected_up(unsigned char keycode)
{
    return 0200;
}

static __inline__ void kbd_leds(unsigned char leds)
{
}

extern void hp600_kbd_init_hw(void);
extern void dreamcast_kbd_init_hw(void);
extern void shmse_kbd_init_hw(void);

static __inline__ void kbd_init_hw(void)
{
	if (MACH_HP600) {
		hp600_kbd_init_hw();
	}
	if (CONFIG_SH_MOBILE_SOLUTION_ENGINE) {
		shmse_kbd_init_hw();
	}

}

#endif
#endif
