/*
 *  adcm3800.h
 *
 *  Agilent ADCM 3800 Camera Module driver.
 *
 *  Copyright (C) 2005 Motorola Inc.
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
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision History:
 *                             Modification     Tracking
 *  Author                 Date          Number     Description of Changes
 *  ----------------   ------------    ----------   -------------------------
 *  Mu Chenliang        06/14/2004      LIBee41682   Created, modified from adcm2700.h
 *  Mu Chenliang        09/29/2004      LIBff19648   Update
 *
*/

/*
 * ==================================================================================
 *                                  INCLUDE FILES
 * ==================================================================================
 */

#ifndef _ADCM3800_H_
#define _ADCM3800_H_

#include "camera.h"

//////////////////////////////////////////////////////////////////////////////////////
//
//          Prototypes
//
//////////////////////////////////////////////////////////////////////////////////////

/* WINDOW SIZE */
typedef struct {
    u16 width;
    u16 height;
} window_size;

#endif /* _ADCM3800_H_ */

