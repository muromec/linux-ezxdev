/******************************************************************************
 NAME                                                                          
        timer.h -- kernel API for the timer driver
 DESCRIPTION                                                                   
        Interface to a minimal, basic, archictecture-agnostic kernel timer 
        library.
 AUTHOR                                                                        
        gmcnutt <gmcnutt@ridgerun.com>
 COPYRIGHT                                                                     
        (C) 2004 MontaVista Software, Inc.
          <source@mvista.com>.
        (C) 2001 RidgeRun, Inc. (http://www.ridgerun.com)                      

 ----------------------------------------------------------------------------
  This program is free software; you can redistribute  it and/or modify it
  under  the terms of  the GNU General  Public License as published by the
  Free Software Foundation;  either version 2 of the  License, or (at your
  option) any later version.
 
  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
  You should have received a copy of the  GNU General Public License along
  with this program; if not, write  to the Free Software Foundation, Inc.,
  675 Mass Ave, Cambridge, MA 02139, USA.
 ----------------------------------------------------------------------------
 *****************************************************************************/

#ifndef __ASM_ARCH_TIMER_H
#define __ASM_ARCH_TIMER_H

#define TIMER_UNIT_MIN timer_unit_ns
#define TIMER_UNIT_MAX timer_unit_Ms

#define TIMERF_PERIODIC 0x1
#define TIMERF_ONESHOT  0x2
#define TIMERF_INT      0x4
#define TIMERF_RST      0x8

typedef struct {
        u32 cntl;     // CNTL_TIMER, R/W
        u32 load_tim; // LOAD_TIM,   W
        u32 read_tim; // READ_TIM,   R
} mputimer_regs_t;

typedef enum { timer_mode_oneshot = 0, timer_mode_periodic } timer_mode_t;
typedef enum { timer_action_interrupt = 0, timer_action_reset } timer_action_t;

typedef void (*timer_cb_t)(int timer, void *data);

extern int timer_get_num_timers(void);
extern int timer_get_features(int timer, int *features);
extern int timer_start(int timer);
extern int timer_stop(int timer);
extern int timer_is_running(int timer);
extern int timer_set_mode(int timer, timer_mode_t mode);
extern int timer_get_mode(int timer, timer_mode_t *mode);
extern int timer_set_action(int timer, timer_action_t action);
extern int timer_get_action(int timer, timer_action_t *action);
extern int timer_set_period(int timer, unsigned long usecs);
extern int timer_get_period(int timer, unsigned long *usecs);
extern int timer_get_time_remaining(int timer, unsigned long *usecs);
extern int timer_register_cb(int timer, timer_cb_t cb, void *data);
extern int timer_unregister_cb(int timer, void *data);

extern void start_mputimer1(unsigned long load_val);
extern void start_mputimer2(unsigned long load_val);
extern void start_mputimer3(unsigned long load_val);
extern unsigned long read_mputimer1(void);
extern unsigned long read_mputimer2(void);
extern unsigned long read_mputimer3(void);

#endif /* __ASM_ARCH_TIMER_H */
