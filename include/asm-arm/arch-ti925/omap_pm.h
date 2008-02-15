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

#define OMAP_PM__IOCTL__MAGIC   0xb2
#define OMAP_PM__IOCTL__SUSPEND _IO(OMAP_PM__IOCTL__MAGIC, 0)
#define OMAP_PM__IOCTL__RESUME  _IO(OMAP_PM__IOCTL__MAGIC, 1)

#ifdef __KERNEL__

#define OMAP_PM__DEV_FIRST      omap_pm_dev_SBI
#define OMAP_PM__DEV_LAST       omap_pm_dev_UART1
#define OMAP_PM__DEV_STATE__ON  0
#define OMAP_PM__DEV_STATE__OFF 1

/*
 *        The value of these enums equal their corresponding bit in the Device
 *        Clock Enable Register. Please keep it that way.
 */
typedef enum omap_pm_dev {
	omap_pm_dev_SBI = 0x001,
	omap_pm_dev_LCD = 0x002,
	omap_pm_dev_DSP = 0x004,
	omap_pm_dev_IC = 0x008,
	omap_pm_dev_GPIO = 0x010,
	omap_pm_dev_Timer2 = 0x020,
	omap_pm_dev_Timer1 = 0x040,
	omap_pm_dev_UART2 = 0x080,
	omap_pm_dev_UART1 = 0x100
} omap_pm_dev_t;

extern void omap_pm_clock_device(omap_pm_dev_t dev);
extern void omap_pm_unclock_device(omap_pm_dev_t dev);

#endif  /*  __KERNEL__  */

#endif  /* OMAP_PM_H  */
