/******************************************************************************

	usb-mx2otg.h
	driver for Motorola Dragonball MX2 USB OTG - header file

	Copyright (c) 2004 MontaVista Software, Inc. <source@mvista.com>

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

********************************************************************************/

/* allocates "size" (normaly equal to maxpacket) of bytes in OTG DATA memory.
returns a relative starting address of the block requested or -ENOMEM */
extern int
 mx2otg_alloc_DATA_mem(u16 size);

/* frees previously allocated OTG DATA memory
descr_addr - realtive starting address in the OTG DATA memory */
extern void
 mx2otg_free_DATA_mem(u16 descr_addr);

struct mx2otg_host_descr_st {
	int (*enable) (void);
	void (*disable) (void);
	void (*dma_err) (void);
};

struct mx2otg_device_descr_st {
	int (*enable) (void);
	void (*disable) (void);
	void (*dma_err) (void);
};

extern int mx2otg_register_host(struct mx2otg_host_descr_st *mx2otg_host_descr);
extern void mx2otg_unregister_host(struct mx2otg_host_descr_st
				   *mx2otg_host_descr);
extern int mx2otg_register_device(struct mx2otg_device_descr_st
				  *mx2otg_device_descr);
extern void mx2otg_unregister_device(struct mx2otg_device_descr_st
				     *mx2otg_device_descr);

#define mx2otg_clear_toggling_bit(reg,mask) do { \
if (reg & mask) reg = mask;\
} while (0)

#define mx2otg_set_toggling_bit(reg,mask) do { \
if (!(reg & mask)) reg = mask;\
} while (0)

#define OTG_CORE_HCINTEN 1
#define OTG_CORE_FCINTEN 2
#define OTG_CORE_HNPINTEN 4
