/*
 *  mx21-wdt.h -- Dragonball MX21 Watch Dog Timer driver
 *
 */
/*
 *  Copyright 2002 Sony Corporation.
 *  Copyright (C) 2003 MontaVista Software Inc. <source@mvista.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  version 2 of the  License.
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

#ifndef MX21_WDT_H_
#define MX21_WDT_H_

#define WDT_DEV_NAME    "mx21-wdt"	/* Device Name */

#define WDT_TIMEOUT     (2*10)<<9	/* Watchdog Timeout value */

#define WDE             1<<2	/* Watchdog Enable  */
#define WDOG_WDOG       1<<3	/* Generate WDOG_ signal  */
#define WDOG_SRS        1<<4	/* Software reset */
#define WDOG_DBG        1<<1	/* Debug mode enable */

#define WDT_DATA1       0x00005555	/* Watchdog Service data 1 */
#define WDT_DATA2       0x0000AAAA	/*                       2 */

#define WDT_DISABLE1    0x43	/* Watchdog disable data 1 */
#define WDT_DISABLE2    0x67	/*                       2 */
#define WDT_DISABLE3    0x28	/*                       3 */

#define WRSR_SFTW            1	/* Software reset */
#define WRSR_TOUT            1<<1	/* Timeout */
#define WRSR_EXT             1<<3	/* External reset */
#define WRSR_PWR             1<<4	/* Power-on reset */

#endif				/* !MX21_WDT_H_ */
