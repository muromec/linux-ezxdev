/* ------------------------------------------------------------------------- */
/* i2c-adap-ite.h i2c-hw access for ITE IIC bus adapters	             */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 1995-97 Simon G. Vogl
                   1998-99 Hans Berglund

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

/* With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and even
   Frodo Looijaard <frodol@dds.nl> */

/* Modified by MontaVista Software, 2001
   Added support for the ITE IIC peripheral */


#ifndef I2C_ADAP_ITE_H
#define I2C_ADAP_ITE_H 1

/*
 * This struct contains the hw-dependent functions of ITE IIC adapters 
 * to manipulate the registers, and to init any hw-specific features. 
 */

struct iic_ite {
	int iic_base;
	int iic_irq;
	int iic_clock;
	int iic_own;
};

#endif /* I2C_ADAP_ITE_H */
