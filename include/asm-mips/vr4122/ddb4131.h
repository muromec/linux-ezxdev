/*
 *
 * BRIEF MODULE DESCRIPTION
 *	Include file for NEC Eagle board.
 *
 * Copyright 2001 MontaVista Software Inc.
 * Author: Yoichi Yuasa
 *		yyuasa@mvista.com or source@mvista.com
 *
 *  Copyright (C) 2002 MontaVista Software Inc.
 *  Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *	Modifed for CMB-VR4131 board on Rockhopper
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

#ifndef __ASM_NEC_DDB4131_H
#define __ASM_NEC_DDB4131_H

/*
 * Board specific address mapping
 */
#define	DDB4131_DS1386_BASE		0x0a000000

#define VR4122_PCI_MEM_BASE		0x10000000
#define VR4122_PCI_MEM_SIZE		0x06000000

#define VR4122_PCI_IO_BASE		0x16000000
#define VR4122_PCI_IO_SIZE		0x02000000

#define VR4122_PCI_IO_START		0x01000000
#define VR4122_PCI_IO_END		0x01ffffff

#define VR4122_PCI_MEM_START		0x12000000
#define VR4122_PCI_MEM_END		0x15ffffff

#define	DDB4131_FLASH_BASE		0x1e000000
#define	DDB4131_FLASH_SIZE		0x02000000	/* 32MB */

#define VR4122_ISA_IO_BASE		KSEG1ADDR(VR4122_EXTERNAL_IO_BASE)
#define VR4122_IO_PORT_BASE		KSEG1ADDR(VR4122_PCI_IO_BASE)


/*
 * IRQ block assignment
 */
#define	NUM_I8259_IRQ		16

#define	I8259_IRQ_BASE		0
#define VR4122_CPU_IRQ_BASE	(I8259_IRQ_BASE + NUM_I8259_IRQ)
#define VR4122_SYSINT1_IRQ_BASE (VR4122_CPU_IRQ_BASE + VR4122_NUM_CPU_IRQ)
#define VR4122_SYSINT2_IRQ_BASE (VR4122_SYSINT1_IRQ_BASE + VR4122_NUM_SYSINT1_IRQ)
#define VR4122_GIUINTL_IRQ_BASE (VR4122_SYSINT2_IRQ_BASE + VR4122_NUM_SYSINT2_IRQ)
#define VR4122_GIUINTH_IRQ_BASE (VR4122_GIUINTL_IRQ_BASE + VR4122_NUM_GIUINTL_IRQ)

/* 
 * IRQ aliases
 */
#define	IRQ_PCI_INTA			VR4122_IRQ_GPIO3
#define	IRQ_PCI_INTB			VR4122_IRQ_GPIO4
#define	IRQ_PCI_INTC			VR4122_IRQ_GPIO5
#define	IRQ_PCI_INTD			VR4122_IRQ_GPIO7
#define	IRQ_PCI_INTE			VR4122_IRQ_GPIO8

#define	IRQ_GPIO_I8259_CASCADE		IRQ_PCI_INTC
#define	IRQ_SYSINT_GPIO_CASCADE		VR4122_IRQ_GIU
#define	IRQ_CPU_SYSINT_CASCADE		VR4122_IRQ_INT0		/* ip #2 */

#endif /* __ASM_NEC_DDB4131_H */
