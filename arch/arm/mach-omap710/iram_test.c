/*
 * File: onchip.c
 *
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: Gordon McNutt <gmcnutt@ridgerun.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>

static char *testBuf1, *testBuf2;

static int __init
onchip_init(void)
{
	testBuf1 = kmalloc(512, GFP_KERNEL | GFP_IRAM);
	if (!testBuf1)
		return -ENOMEM;
	printk("IRAM: virt=0x%x, phys=0x%x\n", (int) testBuf1,
	       (int) virt_to_phys(testBuf1));

	testBuf2 = kmalloc(512, GFP_KERNEL);
	if (!testBuf2) {
		kfree(testBuf1);
		return -ENOMEM;
	}
	printk("Offchip: virt=0x%x, phys=0x%x\n", (int) testBuf2,
	       (int) virt_to_phys(testBuf2));

	return 0;
}

static void __exit
onchip_exit(void)
{
	kfree(testBuf1);
	kfree(testBuf2);
}

module_init(onchip_init);
module_exit(onchip_exit);
