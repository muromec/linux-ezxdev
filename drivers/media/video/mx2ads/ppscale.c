/* drivers/media/video/mx2ads/ppscale.c
 *
 * MX21 PP image scaling primitives
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * Author: MontaVista Software, Inc.
 *              source@mvista.com
 *
 * This file is based on scale.c, pphw.c from Motorola Dragonball MX2 ADS BSP
 * Copyright 2002, 2003, 2004  Motorola, Inc. All Rights Reserved.
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/kernel.h>
#include <asm/hardware.h>
#include <linux/errno.h>

#include "common.h"

#define MODULE_NAME "PP scaler"

/* resize table dimensions 
    dest pixel index    left/32    right/32    #src pixels to read
    0                   [BC_COEF]  [BC_COEF]   [BC_NXT]
    :
    pp_tbl_max-1
*/
#define SCALE_RETRY      16
#define BC_NXT		2
#define BC_COEF		5
#define SZ_COEF		(1 << BC_COEF)
#define SZ_NXT		(1 << BC_NXT)



static const int pp_tbl_max = 40;
static const int pp_skip  = 1;
static unsigned short	scale_tbl[40];



static int
gcd(int x, int y)
{
	int k;

	if (x < y)
	{
		k = x;
		x = y;
		y = k;
	}
	while ((k = x % y)) 
	{
		x = y;
		y = k;
	}
	return y;
}


static int
ratio(int x, int y, int *den)
{
	int	g;

	if (!x || !y)
	{
		return 0;
	}
	g = gcd(x, y);
	*den = y / g;
	return x / g;
}


static int
scale_PP(int k, int coeff, int base, int nxt)
{
	if (k >= pp_tbl_max)
	{
		/* no more space in table */
                err("pp: no space in scale table, k=%d\n", k); 
		return -1;
	}

	coeff = ((coeff << BC_COEF) + (base >> 1)) / base;
	if (coeff >= SZ_COEF - 1)
	{
		/* 31 means 32, so 30 will handle 30&31 */
		coeff--;
	}
	else if (coeff == 1)
	{
		coeff++;
	}
	coeff = coeff << BC_NXT;

	if (nxt < SZ_NXT)
	{
		coeff |= nxt;
                coeff <<= 1;
                coeff |= 1;
	}
	else
	{
		/* 
			src inc field is 2 bit wide
			for 4+, use special code 0:0:1 to prevent dest inc
		*/
		coeff |= pp_skip;
                coeff <<= 1;
                coeff |= 1;
		nxt -= pp_skip;
		do
		{
			scale_tbl[k++] = coeff;
			coeff = (nxt > pp_skip) ? pp_skip : nxt;
			coeff <<= 1;
		}
		while ((nxt -= pp_skip) > 0);
	}
        dbg("pp: tbl=%03X\n", coeff);
	scale_tbl[k++] = coeff;

	return k;
}

/*
	inv	input resolution reduced ratio
	outv	output resolution reduced ratio
	k	index into free table entry
*/
static int 
scale(int inv, int outv, int k)
{
	int	v;		/* overflow counter */
	int	coeff, nxt;	/* table output */

	if (inv == outv)
	{
		/* force scaling */
		return scale_PP(k, 1, 1, 1);
	}

	v = 0;
	if (inv < outv)
	{
		/* upscale: mix <= 2 input pixels per output pixel */
		do
		{
			coeff = outv - v;
			v += inv;
			if (v >= outv)
			{
				v -= outv;
				nxt = 1;
			}
			else
			{
				nxt = 0;
			}
                        dbg("upscale: coeff=%d/%d nxt=%d\n", coeff, outv, nxt);
			k = scale_PP(k, coeff, outv, nxt);
			if (k < 0)
			{
				return -1;
			}
		}
		while (v);
	}
	else if (inv >= 2 * outv)
	{
		/* downscale: >=2:1 bilinear approximation */
		coeff  = inv - 2 * outv;
		v = 0; 
		nxt = 0;
		do
		{
			v += coeff;
			nxt = 2;
			while (v >= outv)
			{
				v -= outv;
				nxt++;
			}
                        dbg("dnscale: coeff=1/2 nxt=%d\n", nxt);
			k = scale_PP(k, 1, 2, nxt);
			if (k < 0)
			{
				return -1;
			}
		}
		while (v);
	}
	else
	{
		/* downscale: bilinear */
		int	inPosInc = 2 * outv;
		int	outPos = inv;
		int	outPosInc = 2 * inv;
		int	init_carry = inv - outv;
		int	carry = init_carry;

		v = outv + inPosInc;
		do
		{
			coeff = v - outPos;
			outPos += outPosInc;
			carry += outPosInc;
			for (nxt = 0; v < outPos; nxt++)
			{
				v += inPosInc;
				carry -= inPosInc;
			}
                        dbg("dnscale: coeff=%d/%d nxt=%d\n", coeff, inPosInc, nxt);
			k = scale_PP(k, coeff, inPosInc, nxt);
			if (k < 0)
			{
				return -1;
			}
		} 
		while (carry != init_carry);
	}
	return k;
}


/*
	To facilitate quick scaling
*/
int  
pp_rescale(int in_width, int in_height, int *out_width, int *out_height)
{
	int	hlen;
	int	vlen;
	int	numx;
	int	denx;
	int	numy;
	int	deny;
	int	tin;
	int	temp;
	int	retry;
        volatile unsigned int *resize_coef_table = 
                (volatile unsigned int *)(EMMA_PP_BASE + 0x100);

        tin = in_width;
        temp = *out_width;

	retry = SCALE_RETRY;
h_retry:
	numx = ratio(tin, temp, &denx);
	if (!numx)
	{
                err("pp: x:Bad ratio err\n");
		return -EINVAL;
	}

	hlen = scale(numx, denx, 0);
	if (hlen < 0)
	{
		temp++;
		if (retry--)
		{
			goto h_retry;
		}
                err("pp: hscale err\n");
		return -EINVAL;
	}
        
        *out_width = temp;

        tin = in_height;
        temp = *out_height;
        
	retry = SCALE_RETRY;
v_retry:
	numy = ratio(tin, temp, &deny);
	if (!numy)
	{
                err("pp: y:Bad ratio err\n");
		return -EINVAL;
	}
	if (numy == numx && deny == denx)
	{
		/* same ratio - share table entries */
		vlen = hlen;
	}
	else
	{
		vlen = scale(numy, deny, hlen);
		if (vlen < 0)
		{
			temp++;
			if (retry--)
			{
				goto v_retry;
			}
                        err("pp: vscale err\n");
			return -EINVAL;
		}
	}

	EMMA_PP_DIMAGE_SIZE = (*out_width << 16) | (*out_height);
	EMMA_PP_RSIZE_IDX = ((hlen - 1) << 16) | 
				(vlen == hlen ? 0 : (hlen << 8)) | (vlen - 1);
	for (hlen = 0; hlen < vlen; hlen++)
	{
		resize_coef_table[hlen] = scale_tbl[hlen];
	}
        *out_height = temp;

	*out_width &= ~1;
	return 0;
}


