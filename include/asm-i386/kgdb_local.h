#ifndef __KGDB_LOCAL
#define ___KGDB_LOCAL
#include <linux/config.h>
#include <linux/types.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <asm/kgdb.h>
#define PORT 0x3f8
#ifdef CONFIG_KGDB_PORT
#undef PORT
#define PORT CONFIG_KGDB_PORT
#endif
#define IRQ 4
#ifdef CONFIG_KGDB_IRQ
#undef IRQ
#define IRQ CONFIG_KGDB_IRQ
#endif
#define SB_CLOCK 1843200
#define SB_BASE (SB_CLOCK/16)
#define SB_BAUD9600 SB_BASE/9600
#define SB_BAUD192  SB_BASE/19200
#define SB_BAUD384  SB_BASE/38400
#define SB_BAUD576  SB_BASE/57600
#define SB_BAUD1152 SB_BASE/115200
#ifdef CONFIG_KGDB_9600BAUD
#define SB_BAUD SB_BAUD9600
#endif
#ifdef CONFIG_KGDB_19200BAUD
#define SB_BAUD SB_BAUD192
#endif
#ifdef CONFIG_KGDB_38400BAUD
#define SB_BAUD SB_BAUD384
#endif
#ifdef CONFIG_KGDB_57600BAUD
#define SB_BAUD SB_BAUD576
#endif
#ifdef CONFIG_KGDB_115200BAUD
#define SB_BAUD SB_BAUD1152
#endif
#ifndef SB_BAUD
#define SB_BAUD SB_BAUD1152        /* Start with this if not given*/
#endif
 
#define SB_IER 1
#define SB_MCR UART_MCR_OUT2 | UART_MCR_DTR | UART_MCR_RTS

#define FLAGS 0
#define SB_STATE { \
     magic: SSTATE_MAGIC, \
     baud_base: SB_BASE,  \
     port:      PORT,     \
     irq:       IRQ,      \
     flags:     FLAGS,    \
     custom_divisor:SB_BAUD}
#define SB_INFO  { \
      magic: SERIAL_MAGIC, \
      port:  PORT,0,FLAGS, \
      state: &state,       \
      tty:   (struct tty_struct *)&state, \
      IER:   SB_IER,       \
      MCR:   SB_MCR}
extern void putDebugChar(int);
#ifdef CONFIG_SERIAL
extern void shutdown_for_kgdb(struct async_struct * info);
#endif
#define INIT_KDEBUG putDebugChar("+");
#endif /* __KGDB_LOCAL */

