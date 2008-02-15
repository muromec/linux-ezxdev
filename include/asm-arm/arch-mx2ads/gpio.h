/*
 *  The Motorola MX2 GPIO driver/framework
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Copyright 2002 Sony Corporation.
 *  Copyright (c) 2003 MontaVista Software Inc. <source@mvista.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  version 2 of the  License.
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
 *
 */

#ifndef MX2_GPIO_H__
#define MX2_GPIO_H__

#include <asm/arch/hardware.h>

#define PORT_A                 0
#define PORT_B                 1
#define PORT_C                 2
#define PORT_D                 3
#define PORT_E                 4
#define PORT_F                 5

#define GPIO                  (1 << 0)
#define PRIMARY               (1 << 1)
#define SECONDARY             (1 << 2)
#define INPUT                 (1 << 3)
#define OUTPUT                (1 << 4)
#define NOINTERRUPT           (1 << 7)

#define OCR_BIT                     5
#define   OCR_A               (0 << OCR_BIT)	/* default */
#define   OCR_B               (1 << OCR_BIT)
#define   OCR_C               (2 << OCR_BIT)
#define   OCR_DATA            (3 << OCR_BIT)
#define ICONFA_BIT                  7
#define   ICONFA_IN           (0 << ICONFA_BIT)	/* default */
#define   ICONFA_INT          (1 << ICONFA_BIT)
#define   ICONFA_0            (2 << ICONFA_BIT)
#define   ICONFA_1            (3 << ICONFA_BIT)
#define ICONFB_BIT                  9
#define   ICONFB_IN           (0 << ICONFB_BIT)	/* default */
#define   ICONFB_INT          (1 << ICONFB_BIT)
#define   ICONFB_0            (2 << ICONFB_BIT)
#define   ICONFB_1            (3 << ICONFB_BIT)
#define PUEN_BIT                   11
#define   PULLUP              (0 << PUEN_BIT)	/* default */
#define   TRISTATE            (1 << PUEN_BIT)
#define INIT_DATA_BIT             12
#define   INIT_DATA_0         (0 << INIT_DATA_BIT)	/* default */
#define   INIT_DATA_1         (1 << INIT_DATA_BIT)

#define   ICONF_GPIO_IN           0x0
#define   ICONF_ISR               0x1
#define   ICONF_0                 0x2
#define   ICONF_1                 0x3

#define   OCR_A_IN                0x0
#define   OCR_B_IN                0x1
#define   OCR_C_IN                0x2
#define   OCR_DATA_REG            0x3

#define POSITIVE_EDGE              0x0
#define NEGATIVE_EDGE              0x1
#define POSITIVE_LEVEL             0x2
#define NEGATIVE_LEVEL             0x3

typedef struct mx2_gpio {
	unsigned int used;
	struct {
		unsigned int mode;
		char name[8];
	} bit[32];
} mx2_gpio_t;

typedef void (*gpio_callback_t) (int irq, void *dev_id, struct pt_regs * regs);

int mx2_register_gpio(unsigned int port, unsigned int bitnum,
		      unsigned int mode);
int mx2_unregister_gpio(unsigned int port, unsigned int bitnum);

int mx2_register_gpios(unsigned int port, unsigned long bitmask,
		       unsigned int mode);
int mx2_unregister_gpios(unsigned int port, unsigned long bitmask);

int mx2_gpio_init(void);
void mx2_gpio_cleanup(void);
unsigned mx2_gpio_get_bit(int port, int bitnum);
unsigned mx2_gpio_get(int port);
int mx2_gpio_set_bit(int port, int bitnum, int value);
int mx2_gpio_config_intr(int port, int bitnum, int intrconf,
			 gpio_callback_t callback);
int mx2_gpio_get_intr_config(int port, int bitnum);
int mx2_gpio_intr_status_bit(int port, int bitnum);
unsigned mx2_gpio_intr_status(int port);
int mx2_gpio_clear_intr(int port, int bitnum);
int mx2_gpio_mask_intr(int port, int bitnum);
int mx2_gpio_unmask_intr(int port, int bitnum);

#endif
