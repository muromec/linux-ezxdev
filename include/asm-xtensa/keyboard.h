/*
 *  linux/include/asm-xtensa/keyboard.h
 *
 */

#ifndef __ASM_XTENSA_KEYBOARD_H
#define __ASM_XTENSA_KEYBOARD_H

#ifdef __KERNEL__

#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/kd.h>
#include <asm/io.h>

#define KEYBOARD_IRQ			1
#define DISABLE_KBD_DURING_INTERRUPTS	0
#define INIT_KBD

static inline int kbd_setkeycode(unsigned int scancode, unsigned int keycode)
{
		return 0;
}
  
static inline int kbd_getkeycode(unsigned int scancode)
{
		return 0;
}
  
static inline int kbd_translate(unsigned char keycode, unsigned char *keycodep,
				char raw_mode)
{
		return 0;
}
  
static inline int kbd_unexpected_up(unsigned char keycode)
{
		return 0;
}
  
static inline void kbd_leds(unsigned char leds)
{
}

static inline void kbd_init_hw(void)
{
}

#define kbd_sysrq_xlate	

extern unsigned long SYSRQ_KEY;

/* resource allocation */
#define kbd_request_region() ;
#define kbd_request_irq(handler) ;

//request_irq(KEYBOARD_IRQ, handler, 0, 
//                                             "keyboard", NULL)

/* How to access the keyboard macros on this platform.  */
#define kbd_read_input() ;
//inb(KBD_DATA_REG)
#define kbd_read_status()  ;
//inb(KBD_STATUS_REG)
#define kbd_write_output(val) ;
//outb(val, KBD_DATA_REG)
#define kbd_write_command(val) ;
//outb(val, KBD_CNTL_REG)

/* Some stoneage hardware needs delays after some operations.  */
#define kbd_pause() do { } while(0)

/*
 * Machine specific bits for the PS/2 driver
 */

#define AUX_IRQ 12

#define aux_request_irq(hand, dev_id)					\
	request_irq(AUX_IRQ, hand, SA_SHIRQ, "PS/2 Mouse", dev_id)

#define aux_free_irq(dev_id) free_irq(AUX_IRQ, dev_id)

#endif /* __KERNEL__ */

#endif /* __ASMPPC_KEYBOARD_H */
