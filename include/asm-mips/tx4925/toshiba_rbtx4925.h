#ifndef TX4925_TOSHIBA_RBTX4925
#define TX4925_TOSHIBA_RBTX4925

/*
 * linux/include/asm-mips/tx4925/toshiba_rbtx4925.h
 *
 * rbtx4925 defs
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * Copyright 2001-2002 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifdef CONFIG_PCI
#include <asm/tx4925/tx4925_pci.h>
#endif

#define TOSHIBA_RBTX4925_WR08(a,b) do { TX4925_WR08(a,b); wbflush(); } while ( 0 )

#define TX4925_PCIMEM           0x08000000
#define TX4925_PCIMEM_SIZE      0x08000000      /* 128M */
#define TX4925_PCIIO            0x04000000
#define TX4925_PCIIO_SIZE       0x01000000      /* 16M */

#ifdef CONFIG_PCI
#define TBTX4925_ISA_IO_OFFSET TX4925_PCIIO
#else
#define TBTX4925_ISA_IO_OFFSET 0
#endif

#define RBTX4925_SW_RESET_DO         0xbb007000
#define RBTX4925_SW_RESET_DO_SET                0x01

#define RBTX4925_SW_RESET_ENABLE     0xbb007002
#define RBTX4925_SW_RESET_ENABLE_SET            0x01

#define RBTX4925_SW_RESET_PCI        0xbb007004
#define RBTX4925_SW_RESET_PCI_SET                0x01

#define RBTX4925_RTL_8019_BASE (0x1b020280-TBTX4925_ISA_IO_OFFSET)
#define RBTX4925_RTL_8019_IRQ  (27)

#define RBTX4925_PIOSEL              0xbb005000

/* bits for PIOSEL */
#define TX4925_PIOSEL_SIO(ch)        (0x01<<(ch))
#define TX4925_PIOSEL_NOPCTOE        0x04
#define TX4925_PIOSEL_NOSMART        0x08
#define TX4925_PIOSEL_NOACLINK       0x10
#define TX4925_PIOSEL_NOPCMCIA       0x20

#define tx4925_pioselptr        ((volatile unsigned char *)RBTX4925_PIOSEL)

#endif
