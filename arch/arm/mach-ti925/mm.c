/*
 *  linux/arch/arm/mach-ti925/mm.c
 *
 * BRIEF MODULE DESCRIPTION
 *   maps in the hardware correctly
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
#include <linux/init.h>

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/arch/sizes.h>
#include <asm/mach/map.h>

/*
 * Map the various IO regions onto 0xExxx.xxxx.  For
 * example, TI925_SC_BASE (0x1100.0000) gets mapped
 * to 0xF110.0000
 */

#define TI925_MAP_DESC(start,size) { IO_ADDRESS(start),start,size,DOMAIN_IO,0,1,0,0 }

static struct map_desc ti925_io_desc[] __initdata = {
	TI925_MAP_DESC(TI925_RHEA_START, TI925_RHEA_SIZE),
	TI925_MAP_DESC(TI925_HIO_FPGA_START, TI925_HIO_FPGA_SIZE),
	TI925_MAP_DESC(TI925_ETHER_START, TI925_ETHER_SIZE),
	TI925_MAP_DESC(TI925_TC_START, TI925_TC_SIZE),
	TI925_MAP_DESC(TI925_FLASH_START, TI925_FLASH_SIZE),
	LAST_DESC
};

void __init
ti925_map_io(void)
{
	iotable_init(ti925_io_desc);
	// map in the traffic controller buses
	outl(0x0fffff00, IO_ADDRESS(TI925_TC_BASE));
	// stop the darn reset to the ethernet card... 
	outb(0x2, IO_ADDRESS(0x11000017));
	// turn on the clocks for everything (Default)
	outb(0xff, IO_ADDRESS(0x11000014));
#if defined(CONFIG_DBG_DSPLINUX_BOOT)
	{
		extern void ti925_serial_console_init(void);
		ti925_serial_console_init();
		printk("DSPLinux: Low Level serial debug\n");
	}
#endif
}

/*
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */
