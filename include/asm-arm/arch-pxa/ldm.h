/*
 * include/asm-arch/mach-pxa/ldm.h
 *
 * Device model prototypes
 *
 * Copyright (C) 2003, Montavista Software Inc.
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef _PXA_LDM_H_
#define _PXA_LDM_H_ 

#ifndef _DEVICE_H_
#error "Please include linux/device.h first"
#endif

extern void pxasys_device_register(struct device *);
extern void pxasys_driver_register(struct device_driver *);
extern void pxasys_driver_unregister(struct device_driver *);
extern void pxasys_device_unregister(struct device *);

extern void pxaopb_device_register(struct device *);
extern void pxaopb_driver_register(struct device_driver *);
extern void pxaopb_driver_unregister(struct device_driver *);
extern void pxaopb_device_unregister(struct device *);

extern void pxaebc_device_register(struct device *);
extern void pxaebc_driver_register(struct device_driver *);
extern void pxaebc_driver_unregister(struct device_driver *);
extern void pxaebc_device_unregister(struct device *);

#endif /* _PXA_LDM_H_ */
