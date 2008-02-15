/*
 * omap_pm.h
 *
 * BRIEF MODULE DESCRIPTION
 *
 *        This file defines the interface to the TI925-specific Power 
 *        Management (PM) module.
 *
 * Copyright (C) 2000 RidgeRun, Inc. (http://www.ridgerun.com)
 * Author: Gordon McNutt <gmcnutt@ridgerun.com>
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
 *
 */
#ifndef OMAP_PM_H
#define OMAP_PM_H

#include <linux/ioctl.h>

#define OMAP_PM__IOCTL__MAGIC      0xb2
#define OMAP_PM__IOCTL__SUSPEND    _IO(OMAP_PM__IOCTL__MAGIC, 0)
#define OMAP_PM__IOCTL__RESUME     _IO(OMAP_PM__IOCTL__MAGIC, 1)
#define OMAP_PM__IOCTL__ENABLE     _IO(OMAP_PM__IOCTL__MAGIC, 2)
#define OMAP_PM__IOCTL__DISABLE    _IO(OMAP_PM__IOCTL__MAGIC, 3)

// Set/Get LCD and DSP are in MHz
#define OMAP_PM__IOCTL__SET_LCD    _IOR(OMAP_PM__IOCTL__MAGIC, 4, sizeof(unsigned long))
#define OMAP_PM__IOCTL__GET_LCD    _IOW(OMAP_PM__IOCTL__MAGIC, 5, sizeof(unsigned long))
#define OMAP_PM__IOCTL__SET_DSP    _IOR(OMAP_PM__IOCTL__MAGIC, 6, sizeof(unsigned long))
#define OMAP_PM__IOCTL__GET_DSP    _IOW(OMAP_PM__IOCTL__MAGIC, 7, sizeof(unsigned long))

// Set/Get Time is in milliseconds
#define OMAP_PM__IOCTL__SET_TIME   _IOR(OMAP_PM__IOCTL__MAGIC, 8, sizeof(unsigned long))
#define OMAP_PM__IOCTL__GET_TIME   _IOW(OMAP_PM__IOCTL__MAGIC, 9, sizeof(unsigned long))

#if NOT_PLUMBED_YET

// Set/Get Blank time is in seconds
#define OMAP_PM__IOCTL__SET_BLANK  _IOR(OMAP_PM__IOCTL__MAGIC, 10, sizeof(unsigned long))
#define OMAP_PM__IOCTL__GET_BLANK  _IOW(OMAP_PM__IOCTL__MAGIC, 11, sizeof(unsigned long))

#endif  // not plumbed yet

#define OMAP_PM__IOCTL__MAX        7

#ifdef __KERNEL__

// Functions exposed for /proc interface

/*
  These set/get a value for one of the settable parameters.  "cmd"
  is one of the ioctl defines from above.
*/
int pm_set_rate(unsigned int cmd, unsigned long value);
int pm_get_rate(unsigned int cmd);

/*  These enable/disable the more aggressive PM */
void pm_enable(void);
void pm_disable(void);

/*  This returns a 1 if the more aggressive PM is enabled, 0 otherwise  */
int pm_enabled(void);

#endif  /*  __KERNEL  */

#endif  /* OMAP_PM_H  */
