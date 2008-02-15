/*****************************************************************************
  NAME
        ck.h -- generic interface to clock control
     
  DESCRIPTION
        This interface will be implemented by arch-specific DPLL/CLKM/Clock
        Controller drivers. It's NOT a user interface -- if you want to
        provide a user interface then lay a proc or dev driver over it. The
        idea is that you can either do that or you can keep clock control
        entirely inside the kernel by having a clock policy module use this
        interface for control/sense.

        This shows some possible architectures for supporting power management
        or whatever on top of the clock control interface:

        +--------------------------+
        | pm app                   |
        +--------------------------+
        +-----------+ +------------+ +-----------+
        | proc "pm" | | /dev/clock | | pm module |
        +-----------+ +------------+ +-----------+
        /----------------------------------------\
        | ck.h ifc                               |
        \----------------------------------------/
        +----------------------------------------+
        | arch-specific clock driver(s)          |
        +----------------------------------------+
        
        Since this is a kernel-only interface it doesn't belong in the kernel
        includes -- only other modules should include this file.

        For a platform to support this interface it needs at least a clock
        driver module to implement the functions defined here and another
        include file (asm/arch/ck.h) to enumerate the different clocks.

  AUTHOR
        gmcnutt@ridgerun.com

  COPYRIGHT
        (C) 2001 RidgeRun, Inc. (http://www.ridgerun.com)

   This program is free software; you can redistribute  it and/or modify it
   under  the terms of  the GNU General  Public License as published by the
   Free Software Foundation;  either version 2 of the  License, or (at your
   option) any later version.
 
   THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
   WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
   NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
   NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
   USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
   You should have received a copy of the  GNU General Public License along
   with this program; if not, write  to the Free Software Foundation, Inc.,
   675 Mass Ave, Cambridge, MA 02139, USA.
 
   Please report all bugs and problems to the author or <support@dsplinux.net> 

   key: RRGPLCR (do not remove)
 *****************************************************************************/
#ifndef ck_h
#define ck_h

/* This enumerates the ck_t values on a per-platform basis. */
#include <asm/arch/arch_ck.h>

/* For some clocks you have a choice of which "parent" clocks they are derived
   from. Use this to select a "parent". See the platform documentation for 
   valid combinations. */
int ck_can_set_input(ck_t);
int ck_set_input(ck_t ck, ck_t input);
int ck_get_input(ck_t ck, ck_t *input);

/* Use this to set a clock rate. If other clocks are derived from this one, 
   their rates will all change too. If this is a derived clock and I can't 
   change it to match your request unless I also change the parent clock, then
   tough luck -- I won't change the parent automatically. I'll return an error
   if I can't get the clock within 10% of what you want. Otherwise I'll return
   the value I actually set it to. If I have to switch parents to get the rate
   then I will do this automatically (since it only affects this clock and its
   descendants). */
int ck_can_set_rate(ck_t);
int ck_set_rate(ck_t ck, int val_in_mhz);
int ck_get_rate(ck_t ck);

/* Use this to get a bitmap of available rates for the clock. Caller allocates 
   the buffer and passes in the length. Clock module fills up to len bytes of 
   the buffer & passes back actual bytes used. */
int ck_get_rates(ck_t ck, void *buf, int len);

/* Idle a clock. What happens next depends on the clock ;). For example, if 
   you idle the ARM_CK you might well end up in sleep mode on some platforms.
   If you try to idle a clock that doesn't support it I'll return an error. 
   Note that idling a clock does not always take affect until certain h/w 
   conditions are met. Consult the platform specs to learn more. */
int ck_can_idle(ck_t);
int ck_idle(ck_t);
int ck_activate(ck_t);
int ck_is_idle(ck_t);

/* Enable/disable a clock. I'll return an error if the h/w doesn't support it.
   If you disable a clock being used by an active device then you probably 
   just screwed it. YOU are responsible for making sure this doesn't happen. 
   Good luck. */
int ck_can_disable(ck_t);
int ck_enable(ck_t);
int ck_disable(ck_t);
int ck_is_enabled(ck_t);

/* Enable/reset ARM peripherals (remove/set reset signal) */
void ck_enable_peripherals(void);
void ck_reset_peripherals(void);

/* Generate/clear a MPU or DSP reset */
void ck_generate_reset(reset_t reset);
void ck_release_from_reset(reset_t reset);

/* This gets a string representation of the clock's name. Useful for proc. */
char *ck_get_name(ck_t);

#endif
