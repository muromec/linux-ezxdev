/* drivers/media/video/mx2ads/prpscale.c
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
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include "prpscale.h"
#include "common.h"

#define MODULE_NAME "prp_scale"


#define MEAN_COEF	(SZ_COEF >> 1)

/*
	t		table
	i		table index
	out		bilinear	# input pixels to advance 
			average		whether result is ready for output
	ret		coefficient
*/
static unsigned char
scale_get(scale_t *t, unsigned char *i, unsigned char *out)
{
	unsigned char	c;

	c = t->tbl[*i];
	if ((*i) == t->len - 1)
	{
		*i = 0;
	}
	else
	{
		(*i)++;
	}

	if (out)
	{
		if (t->algo == ALGO_BIL)
		{
			for ((*out) = 1; 
				(*i) && ((*i) < t->len) && !t->tbl[(*i)]; 
				(*i)++)
			{
				(*out)++;
			}
			if ((*i) == t->len)
			{
				(*i) = 0;
			}
		}
		else
		{
			*out = c >> BC_COEF;
		}
	}
	c &= SZ_COEF - 1;
	if (c == SZ_COEF - 1)
	{
		c = SZ_COEF;
	}

	return c;
}



/* Euclid algorithm to search of Greater common divider */
static int  
euclid_gcd(int x, int y)
{
	int k;

	if (x < y) {
		k = x;
		x = y;
		y = k;
	}
	while ((k = x % y)) {
		x = y;
		y = k;
	}
	return y;
}


/* ratio  calculation between input and output size */
static int  
ratio(int in_x, int out_x, int *den)
{
	int	g;

	if (!in_x || !out_x) {
		return 0;
	}
	g = euclid_gcd(in_x, out_x);
	*den = out_x / g;
	return in_x / g;
}


#define SCALE_RETRY	16
#define TEST_FIX	1
#define PRP_ERR_RESIZE	-1


int static
prp_scale_bilinear(scale_t *t, int coeff, int base, int nxt)
{
	int	i;

	if (t->len >= sizeof(t->tbl))
	{
		return -1;
	}

	coeff = ((coeff << BC_COEF) + (base >> 1)) / base;
	if (coeff >= SZ_COEF - 1)
	{
		coeff--;
	}
#if 0	/* 15/10/2003: Gopala says w1=1 w2=7 is now supported 
			but not w1=7, w2=1 which as before becomes w1=6 w2=2 */
	else if (coeff == 1)
	{
		coeff++;
	}
#endif

	coeff |= SZ_COEF;
	t->tbl[(int)t->len++] = (unsigned char)coeff;
	for (i = 1; i < nxt; i++)
	{
		if (t->len >= MAX_TBL)
		{
			return -1;
		}
		t->tbl[(int)t->len++] = 0;
	}
	return t->len;
}


#define _bary(name)	static const unsigned char name[]
_bary(c1)  = {7};
_bary(c2)  = {4, 4};
_bary(c3)  = {2, 4, 2};
_bary(c4)  = {2, 2, 2, 2};
_bary(c5)  = {1, 2, 2, 2, 1};
_bary(c6)  = {1, 1, 2, 2, 1, 1};
_bary(c7)  = {1, 1, 1, 2, 1, 1, 1};
_bary(c8)  = {1, 1, 1, 1, 1, 1, 1, 1};
_bary(c9)  = {1, 1, 1, 1, 1, 1, 1, 1, 0};
_bary(c10) = {0, 1, 1, 1, 1, 1, 1, 1, 1, 0};
_bary(c11) = {0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0};
_bary(c12) = {0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0};
_bary(c13) = {0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0};
_bary(c14) = {0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0};
_bary(c15) = {0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0};
_bary(c16) = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0};
_bary(c17) = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0};
_bary(c18) = {0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0};
_bary(c19) = {0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0};
_bary(c20) = {0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0};

static const unsigned char	*ave_coeff[] = {
	c1, c2, c3, c4, c5, c6, c7, c8, c9, c10,
	c11, c12, c13, c14, c15, c16, c17, c18, c19, c20
};

static int
prp_scale_ave(scale_t *t, unsigned char base)
{
	if (t->len + base > sizeof(t->tbl)) {
		return -1;
	}
	memcpy(&t->tbl[(int)t->len], ave_coeff[(int)base - 1], base);
	t->len = (unsigned char)(t->len + base);
	t->tbl[t->len - 1] |= SZ_COEF;

	return t->len;
}


static int
ave_scale(scale_t *t, int inv, int outv)
{
	int	ratio_count;

	ratio_count = 0;
	if (outv != 1) {
		unsigned char	a[20];
		int		v;

		/* split n:m into multiple n[i]:1 */
		for (v = 0; v < outv; v++)
		{
			a[v] = (unsigned char)(inv / outv);
		}
		inv %= outv;
		if (inv)
		{
			/* find start of next layer */
			v = (outv - inv) >> 1;
			inv += v;
			for ( ; v < inv; v++)
			{
				a[v]++;
			}
		}
		for (v = 0; v < outv; v++)
		{
			if (prp_scale_ave(t, a[v]) < 0)
			{
					return -1;
			}
			t->ratio[ratio_count] = a[v];
			ratio_count++;
		}
	} else if (prp_scale_ave(t, inv) < 0) {
		return -1;
	} else {
		t->ratio[ratio_count++] = (char)inv;
		ratio_count++;
	}

	return t->len;
}

/*
	inv		input resolution reduced ratio
	outv	output resolution reduced ratio
	k		index into free table entry
*/
int static
scale(scale_t *t, int inv, int outv)
{
	int	v;				/* overflow counter */
	int	coeff, nxt;		/* table output */

	t->len = 0;
	if (t->algo == ALGO_AUTO) {
		/* automatic choice - bilinear for shrinking less than 2:1 */
		t->algo = ((outv != inv) && ((2 * outv) > inv)) ? 
			ALGO_BIL : ALGO_AVG;
	}

/* 1:1 resize must use averaging, bilinear will hang */
	if((inv == outv) && (t->algo == ALGO_BIL)) {
		t->algo = ALGO_AVG;
	}

	memset(t->tbl, 0, sizeof(t->tbl));
	if (t->algo == ALGO_BIL) {
		t->ratio[0] = (char)inv;
		t->ratio[1] = (char)outv;
	}
	else {
		memset(t->ratio, 0, sizeof(t->ratio));
	}

	if (inv == outv) {
		/* force scaling */
		t->ratio[0] = 1;
		if (t->algo == ALGO_BIL) {
			t->ratio[1] = 1;
		}
		return prp_scale_ave(t, 1);
	}

	if (inv < outv) {
		printk("no upscaling %d:%d\n", inv, outv);
		return -1;
	}

	if (t->algo != ALGO_BIL) {
		return ave_scale(t, inv, outv);
	}

	v = 0;
	if (inv >= 2 * outv)
	{
		/* downscale: >=2:1 bilinear approximation */
		coeff = inv - 2 * outv;
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
			if (prp_scale_bilinear(t, 1, 2, nxt) < 0)
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
			if (prp_scale_bilinear(t, coeff, inPosInc, nxt) < 0)
			{
				return -1;
			}
		} 
		while (carry != init_carry);
	}
	return t->len;
}


int  prp_scale(scale_t *sctbl, int ch, int dir, 
		int inv, unsigned short *vout, unsigned short *pout)
{
	int	num;
	int	den;
#if SCALE_RETRY
	int	temp;
	int	retry;
#endif
	unsigned short	outv;
        int din = inv;
        int dout = *vout;

        if ((!inv) || (!vout)) {
                return -EINVAL;
        }

	/* auto-generation of values */
        
	if (din < dout)
	{
		err("scale err: ch%d %c unsupported ratio %d:%d\n", 
			ch, dir ? 'v' : 'h', din, dout);
		return PRP_ERR_RESIZE;
	}

	temp = dout;
	retry = SCALE_RETRY;

        while(retry--){
	        num = ratio(din, temp, &den);
                if (!num) {
                        err("Ratio calculation error: ch=%d dir=%d %d:%d. Try "
                                         "to change image resolution\n",
                                ch, dir, din, temp);
                        return -EINVAL;
                }
                if ((num <= MAX_TBL) &&/* The ratio coeficients are in table*/
                   (scale(sctbl, num , den) > 0)){ /* Succesfully scaled */
                        break;

                }
                /* Try to change output resolution to achive successful
                 * scaling*/
                temp++;
                dbg("Attempting again\n");
                if (!retry) {
                        err("Scaling error: Try to change resolution\n");
                        return -EINVAL;
                }
        };

	if (sctbl->algo == ALGO_BIL)
	{
		unsigned char	i, j, k;

		outv = (unsigned short)
                        (inv / sctbl->ratio[0] * sctbl->ratio[1]);
                
		inv %= sctbl->ratio[0];
		for (i = j = 0; inv > 0; j++)
		{
			unsigned char	nxt;

			k = scale_get(sctbl, &i, &nxt);
#if 0
			if (!dir && inv == 1 && k < SZ_COEF)
#else // FPGA findings
			if (inv == 1 && k < SZ_COEF)
#endif
			{
				/* needs 2 pixels for this output */
				break;
			}
			inv -= nxt;
		}
		outv = outv + j;
	}
	else
	{
		unsigned char	i, tot;

		for (tot = i = 0; sctbl->ratio[i]; i++)
		{
			tot = tot + sctbl->ratio[i];
		}
		outv = (unsigned short)(inv / tot) * i;
		inv %= tot;
		for (i = 0; inv > 0; i++)
		{
			outv++;
                        inv -= sctbl->ratio[i];
			if (!dir && inv < 0)
			{
				break;
			}
		}
	}

	if (!(*vout) || ((*vout) > outv))
	{
		*vout = outv;
	}
	if (pout)
	{
		*pout = outv;
	}

	return 0;
}

