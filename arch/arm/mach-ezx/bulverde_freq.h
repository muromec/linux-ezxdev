#ifndef BULVERDE_FREQ_H
#define BULVERDE_FREQ_H

/*
 * linux/include/linux/bulverde_freq.h
 *
 * Bulverde-specific support for changing CPU (and System Bus and
 * Memory Clock) frequencies.
 *
 * Author: <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */
/*
 * Copyright (C) 2005 Motorola Inc.
 *
 * 2005-May-11  copy from arch/arm/mach-pxa/bulverde_freq.h and porting to EZX platform,  Zhuang Xiaofan
 * 2005-Nov-11  add Montavista 3.1 patch,  Zhuang Xiaofan
 *
 */

#define CCLKCFG_TURBO   0x1
#define CCLKCFG_FCS     0x2

#define L_NUM   31  /*  30 different L numbers. */
#define N_NUM   7   /*  7 N numbers.*/

#define BLVD_MIN_FREQ       13000
// latest PowerPoint documentation indicates 624000
#define BLVD_MAX_FREQ       520000

extern  struct  dpm_config  global_conf;
extern  void    dump_dpm_config(char *, struct dpm_config *);
void    bulverde_set_freq(unsigned int);

int bulverde_clk_init(void);

#endif /* BULVERDE_FREQ_H */
