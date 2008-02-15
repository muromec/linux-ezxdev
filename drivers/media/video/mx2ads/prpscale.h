/* drivers/media/video/mx2ads/prpscale.h
 *
 * MX21 PRP image scaling primitives
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * Author: MontaVista Software, Inc.
 *              source@mvista.com
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __MX21_PRP_SCALE_H__
#define __MX21_PRP_SCALE_H__

/* resize table dimensions 
    ave: (coeff, out) 
		sum coeff while (!out) 
		pix = sum / base
	bil: (lcoeff, out) 
		coeff * p0 + (base - coeff) * p1, out?
		coeff * p1 + (base - coeff) * p2, out?
		:
*/

#define BC_COEF		3
#define MAX_TBL		20
#define SZ_COEF		(1 << BC_COEF)

/* values defined below must not be changed
  otherwise decision logic will be wrong */
#define ALGO_AUTO	0
#define ALGO_BIL	1
#define ALGO_AVG	2

extern int	prp_ver;

typedef struct
{
	char	tbl[20];	/* table entries */
	char	len;		/* table length used */
	char	algo;		/* ALGO_xxx */
	char	ratio[20];	/* ratios used */
}
scale_t;

/*
	t	out	coefficient table
	ch	in	1=ch1 0=ch2
	dir	in	1=vertical 0=horizontal
	num	in	scale numerator
	den	in	scale denominator
	din	in	pre-scale dimension
	dout	in/out	post-scale output dimension
	dout	out	post-scale internal dimension [opt]
*/
int	prp_scale(scale_t *sctbl, int ch, int dir, int num,
                unsigned short *dout, unsigned short *iout);

#endif /* __MX21_PRP_SCALE_H__ */
