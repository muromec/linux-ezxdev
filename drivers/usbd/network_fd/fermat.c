/*
 * usbd/network_fd/fermat.c - Network Function Driver
 *
 *      Copyright (c) 2003, 2004 Belcarra
 *
 * By: 
 *      Bruce Balden <balden@belcarra.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#ifdef CONFIG_USBD_NETWORK_BLAN_FERMAT

#include "fermat.h"

#ifndef FERMAT_DEFINED
typedef unsigned char BYTE;
typedef struct fermat {
	int length;
	BYTE power[256];
} FERMAT;
#endif


static int fermat_setup(FERMAT *p, int seed){
	int i = 0;
	unsigned long x,y;
	y = 1;
	do{
		x = y;
		p->power[i] = ( x == 256 ? 0 : x);
		y = ( seed * x ) % 257;
		i += 1;
	}while( y != 1);
	p->length = i;
	return i;
}

static void fermat_xform(FERMAT *p, BYTE *data, int length){
	BYTE *pw = p->power;
	int   i, j;
	BYTE * q ;
	for(i = 0, j=0, q = data; i < length; i++, j++, q++){
		if(j>=p->length){
			j = 0;
		}
		*q ^= pw[j];
	}
}

static FERMAT default_fermat;
static const int primitive_root = 5;
void fermat_init(){
	(void) fermat_setup(&default_fermat, primitive_root); 
}

// Here are the public official versions.
// Change the primitive_root above to another primitive root
// if you need better scatter. Possible values are 3 and 7


void fermat_encode(BYTE *data, int length){
	fermat_xform(&default_fermat, data, length);
}

void fermat_decode(BYTE *data, int length){
	fermat_xform(&default_fermat, data, length);
}

		
// Note: the seed must be a "primitive root" of 257. This means that
// the return value of the setup routine must be 256 (otherwise the
// seed is not a primitive root.  The routine will still work fine
// but will be less pseudo-random.

#undef TEST 
#if TEST
#include <stdio.h>
#include <memory.h>

// Use FERMAT in two ways: to encode, and to generate test data.

main(){
	//Note 3, 5, and 7 are primitive roots of 257
	// 11 is not a primitive root
	FERMAT three, five, seven;
	
	FERMAT three2;
	printf("Cycle lengths: 3,5,7 %d %d %d \n", 
			fermat_setup(&three, 3), 
			fermat_setup(&five, 5), 
			fermat_setup(&seven, 7));
	three2=three; // Copy data from three
	fermat_xform(&three,three2.power,three2.length);
	fermat_xform(&five,three2.power,three2.length);
	fermat_xform(&seven,three2.power,three2.length);
	fermat_xform(&seven,three2.power,three2.length);
	fermat_xform(&five,three2.power,three2.length);
	fermat_xform(&three,three2.power,three2.length);

	//At this stage, three2 and three should be identical
	if(memcpy(&three,&three2,sizeof(FERMAT))){
		printf("Decoded intact\n");
	}

	fermat_init();
	fermat_encode(three2.power,256);
	
}
#endif

#endif /* CONFIG_USBD_NETWORK_BLAN_FERMAT */

