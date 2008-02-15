/*
 *  linux/arch/arm/mach-ti925/arch.c
 *
 * BRIEF MODULE DESCRIPTION
 *   sets up the mach_type definition
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
#include <linux/fs.h>
#include <linux/major.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

static void __init
ti925_fixup(struct machine_desc *desc, struct param_struct *params,
	    char **cmdline, struct meminfo *mi)
{
#if defined(CONFIG_BLK_DEV_INITRD)
	ROOT_DEV = MKDEV(RAMDISK_MAJOR, 0);
	setup_ramdisk(1, 0, 0, 8192);
	setup_initrd(0xc4b00000, 6 * 1024 * 1024);
#endif
	*cmdline = CONFIG_CMDLINE;
#define RRLOAD_MAGIC "kcmdline-->"
#define RRLOAD_CMDLINE ((char*) 0xC4000100)
	if (!strncmp(RRLOAD_MAGIC, RRLOAD_CMDLINE, strlen(RRLOAD_MAGIC))) {
		*cmdline = &RRLOAD_CMDLINE[strlen(RRLOAD_MAGIC) + 1];
	}
}

extern void ti925_map_io(void);
MACHINE_START(TI925, "ARM-TI925")
    MAINTAINER("RidgeRun,INC")
    BOOT_MEM(0x04000000, 0x10000000, 0x11000000)
    FIXUP(ti925_fixup)
    MAPIO(ti925_map_io)
    MACHINE_END
