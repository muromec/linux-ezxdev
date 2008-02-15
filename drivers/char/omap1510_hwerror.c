/*
 * File: hwerror.c
 *
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: Greg Lonnon glonnon@ridgerun.com 
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
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/ptrace.h>
#include <linux/interrupt.h>
//#include <linux/irq.h>
#include <asm/irq.h>

static int hw_error_count;
static void hw_error_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
        hw_error_count++;
        if(hw_error_count % 10)
                printk(__FUNCTION__ ":entry");
}
void install_hw_error_irq(void) {
        request_irq(INT_HW_ERRORS, hw_error_interrupt,
                    0, "hw error",
                    NULL);
}
static int __init omap1510_hwerror_init(void) {
        printk(KERN_INFO "DSPLinux hw error monitor (c) 2001 RidgeRun, Inc.\n");
        install_hw_error_irq();
        hw_error_count = 0;
        return 0;
}
static void __exit omap1510_hwerror_exit(void) {

}
module_init(omap1510_hwerror_init);
module_exit(omap1510_hwerror_exit);

MODULE_AUTHOR("Greg Lonnon <glonnon@rigderun.com>");
MODULE_DESCRIPTION("DSPLinux HW Error IRQ");
/*
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
