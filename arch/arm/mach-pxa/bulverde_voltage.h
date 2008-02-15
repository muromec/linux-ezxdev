#ifndef BULVERDE_VOLTAGE_H
#define BULVERDE_VOLTAGE_H

/*
 * linux/include/linux/bulverde_voltage.h
 *
 * Bulverde-specific support for changing CPU voltages via DPM
 *
 * Author: <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */

#include <linux/config.h>
#include <linux/notifier.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>

#define BLVD_MAX_VOL        1400    /*  in mV.  */
#define BLVD_MIN_VOL         850     /*  in Mv.  */
#define BLVD_DEF_VOL        1400    /*  the default voltage.    */

unsigned int mv2DAC(unsigned int mv);
void vm_setvoltage(unsigned int);
unsigned int bulverde_validate_voltage(unsigned int mv);
extern void cpu_voltage_init(unsigned int, unsigned int, unsigned int);
extern unsigned int  bulverde_read_clkcfg(void);
void bulverde_set_voltage(unsigned int mv);
void bulverde_prep_set_voltage(unsigned int mv);
int bulverde_vcs_init(void);

#define cpu_voltage_current(cpu)		( cpu_voltage_cur)
#define cpu_voltage_max(cpu)			( cpu_voltage_max)
#define cpu_voltage_min(cpu)			( cpu_voltage_min)

struct cpu_voltage_info {
	unsigned int old_voltags;
	unsigned int new_voltags;
};

/*
 * The max and min voltagsuency rates that the registered device
 * can tolerate.  Never set any element this structure directly -
 * always use cpu_updateminmax.
 */
struct cpu_voltage_minmax {
	unsigned int min_voltage;
	unsigned int max_voltage;
	unsigned int cur_voltage;
	unsigned int new_voltage;
};

static inline
void cpu_voltage_updateminmax(void *arg, unsigned int min, unsigned int max)
{
	struct cpu_voltage_minmax *vol_minmax = arg;

	if (vol_minmax->min_voltage < min)
		vol_minmax->min_voltage = min;
	if (vol_minmax->max_voltage > max)
		vol_minmax->max_voltage = max;
}

#define CPUVOLTAGE_MINMAX	(0)
#define CPUVOLTAGE_PRECHANGE	(1)
#define CPUVOLTAGE_POSTCHANGE	(2)

int cpu_voltage_register_notifier(struct notifier_block *nb);
int cpu_voltage_unregister_notifier(struct notifier_block *nb);

int cpu_voltage_setmax(void);
int cpu_voltage_restore(void);
int cpu_voltage_set(unsigned int mv);
unsigned int cpu_voltage_get(int cpu);

/*
 * These two functions are only available at init time.
 */
void cpu_voltage_init(unsigned int khz,
		  unsigned int min_voltags,
		  unsigned int max_voltags);

void cpu_voltage_setfunctions(unsigned int (*validate)(unsigned int),
			      void (*setspeed)(unsigned int));

#endif
