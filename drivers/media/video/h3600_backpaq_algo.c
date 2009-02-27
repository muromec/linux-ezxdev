/*
 * Driver for the Compaq iPAQ Mercury Backpaq camera
 * Video4Linux interface
 *
 * Copyright 2001 Compaq Computer Corporation.
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * COMPAQ COMPUTER CORPORATION MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
 * AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
 * FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 * Author: Andrew Christian 
 *         <andyc@handhelds.org>
 *         4 May 2001
 *
 * Image processing algorithms for the Mercury Backpaq Camera
 */


/* 
   Decimate in place, assuming a standard Bayer pattern starting at 0,0
   The subsampled image starts at offset x,y.
   We update "columns" and "rows" to return the new number of columns 
   and rows in the image.

   Bayer pattern is assumed to be:

       R G 
       G B

   starting at the top left corner.
   "Width" and "Height" should be multiples of two with x + width <= *columns, etc.

   We make _tons_ of assumptions

   Assumptions
   1. Error checking of bounds has been done.
   2. Width and height are multiples of 4
*/

#include "h3600_backpaq_common.h"

void decimate( struct frame *frame )
{
	/* Pick results that make sense */
	int width  = (frame->width / 2) & 0xfffc;
	int height = (frame->height / 2) & 0xfffc;

	unsigned char *p    = frame->data;   /* We'll store data at here */
	unsigned char *colp = frame->data;

	unsigned char *q;   /* We'll read data from here */
	int onecol = frame->width;
	int twocol = frame->width * 2;
	int i,j;

	for ( j = 0 ; j < height ; j+= 2) {
		q = colp;
		for ( i = 0 ; i < width ; i+=2 ) {
			/* Do red */
			*p++ = (((unsigned short) *q) * 2 
				+ (unsigned short) *(q + 2) 
				+ (unsigned short) *(q + twocol)) / 4;
			q+=2;
			/* Do green */
			*p++ = (((unsigned short) *(q + 1)
				 + (unsigned short) *(q + onecol)) / 2 );
			q+=2;
		}
		colp += twocol;
		q = colp;
		for ( i = 0 ; i < width ; i+=2 ) {
			/* Do green */
			*p++ = (((unsigned short) *(q + 1)
				 + (unsigned short) *(q + onecol)) / 2 );
			q+=2;
			/* Do blue */
			*p++ = (((unsigned short) *(q + onecol + 1)) * 2 
				+ (unsigned short) *(q + onecol - 1) 
				+ (unsigned short) *(q - onecol + 1)) / 4;
			q+=2;
		}
		colp += twocol;
	}
	frame->width = width;
	frame->height = height;
}


/******************************************************************************
 *
 * Processing macros
 *
 *   The Bayer pattern on the screen of the SMaL camera requires some
 *   special calculations to extract RGB values.  The core pattern
 *   looks like:
 * 
 *           R G R G
 *           G B G B
 *           R G R G
 *           G B G B
 *
 *   We can estimate the values of the colors at each of the 16 possible
 *   positions (4 inner locations, 4 corners, and 8 edge positions.
 * 
 *   The RGB values at each of these locations are stored in macros named
 *   "X##" in the following pattern (where X can be R,G, or B):
 *
 *           X11 X12 X13 X14
 *           X21 X22 X23 X24
 *           X31 X32 X33 X34
 *           X41 X42 X43 X44
 *
 ******************************************************************************/

/* Uses "from", "width" */

#define p32 int

#define PVAL       ((p32)*from)
#define PVAL1(a)   ((p32)*(from+a))
#define PVAL2(a,b) ((((p32)*(from+a)) + ((p32)*(from+b))) >> 1)

#define PVALUE(a1,a2,a3,b1,b2,b3,c1,c2,c3) \
  (( \
    a1 * ((p32)*(from-row_width-1)) + \
    a2 * ((p32)*(from-row_width)) +   \
    a3 * ((p32)*(from-row_width+1)) + \
    b1 * ((p32)*(from-1)) +       \
    b2 * ((p32)*(from)) +         \
    b3 * ((p32)*(from+1)) +       \
    c1 * ((p32)*(from+row_width-1)) + \
    c2 * ((p32)*(from+row_width)) +   \
    c3 * ((p32)*(from+row_width+1))   \
  ) / (a1 + a2 + a3 + b1 + b2 + b3 + c1 + c2 + c3 ))


#define R11 PVAL
#define G11 PVAL2(1,row_width)
#define B11 PVAL1(row_width+1)

#define R12 PVAL2(-1,1)
#define G12 PVAL
#define B12 PVAL1(row_width)

#define R13 PVAL
#define G13 PVALUE(0,0,0, 1,0,1, 0,2,0)
#define B13 PVAL2(row_width-1,row_width+1)

#define R14 PVAL1(-1)
#define G14 PVAL
#define B14 PVAL1(row_width)

#define R21 PVAL2(-row_width,row_width)
#define G21 PVAL
#define B21 PVAL1(1)

#define R22 PVALUE(1,0,1, 0,0,0, 1,0,1)
#define G22 PVALUE(0,1,0, 0,0,0, 0,1,0)
#define B22 PVAL

#define R23 PVAL2(-row_width,row_width)
#define G23 PVAL
#define B23 PVAL2(-1,1)

#define R24 PVAL2(-row_width-1,row_width-1)
#define G24 PVALUE(0,1,0, 2,0,0, 0,1,0)
#define B24 PVAL

#define R31 PVAL
#define G31 PVALUE(0,1,0, 0,0,2, 0,1,0)
#define B31 PVAL2(-row_width+1,row_width+1)

#define R32 PVAL2(-1,1)
#define G32 PVAL
#define B32 PVAL2(-row_width,row_width)

#define R33 PVAL
#define G33 PVALUE(0,1,0, 1,0,1, 0,1,0)
#define B33 PVALUE(1,0,1, 0,0,0, 1,0,1)

#define R34 PVAL1(-1)
#define G34 PVAL
#define B34 PVAL2(-row_width,row_width)

#define R41 PVAL1(-row_width)
#define G41 PVAL
#define B41 PVAL1(1)

#define R42 PVAL2(-row_width-1,-row_width+1)
#define G42 PVALUE(0,2,0, 1,0,1, 0,0,0)
#define B42 PVAL

#define R43 PVAL1(-row_width)
#define G43 PVAL
#define B43 PVAL2(-1,1)

#define R44 PVAL1(-row_width-1)
#define G44 PVAL2(-row_width,-1)
#define B44 PVAL

/* See "Video Demystified, pg 16, third edition */
#define FLOAT_SCALE_FACTOR   16
#define FLOAT_SCALE_VALUE   (1<<FLOAT_SCALE_FACTOR)
#define MAKE_FLOAT(x) ((p32)(x * FLOAT_SCALE_VALUE + 0.5))

#define R_TO_GREY  MAKE_FLOAT(0.299)
#define G_TO_GREY  MAKE_FLOAT(0.587)
#define B_TO_GREY  MAKE_FLOAT(0.114)

#define R_TO_Y     MAKE_FLOAT(0.257)
#define G_TO_Y     MAKE_FLOAT(0.504)
#define B_TO_Y     MAKE_FLOAT(0.098)
#define Y_OFFSET   16

#define R_TO_Cb    MAKE_FLOAT(-.148)
#define G_TO_Cb    MAKE_FLOAT(-.291)
#define B_TO_Cb    MAKE_FLOAT(0.439)
#define Cb_OFFSET  128

#define R_TO_Cr    MAKE_FLOAT(0.439)
#define G_TO_Cr    MAKE_FLOAT(-.368)
#define B_TO_Cr    MAKE_FLOAT(-.071)
#define Cr_OFFSET  128

#define GREY_VALUE(red,green,blue) \
     (((R_TO_GREY * red) + (G_TO_GREY * green) + (B_TO_GREY * blue)) >> FLOAT_SCALE_FACTOR)

/* pg. 18 - conversion of saturated RGB to YCbCr for SDTV */

#define Y_VALUE(red,green,blue) \
     ((((R_TO_Y * red) + (G_TO_Y * green) + (B_TO_Y * blue)) >> FLOAT_SCALE_FACTOR) + Y_OFFSET)

#define Cb_VALUE(red,green,blue) \
     ((((R_TO_Cb * red) + (G_TO_Cb * green) + (B_TO_Cb * blue)) >> FLOAT_SCALE_FACTOR)  + Cb_OFFSET)

#define Cr_VALUE(red,green,blue) \
     ((((R_TO_Cr * red) + (G_TO_Cr * green) + (B_TO_Cr * blue)) >> FLOAT_SCALE_FACTOR) + Cr_OFFSET)

/* 
   Uses:  to, from
*/

#define DO_GREY_PIXEL_FORWARD(red,green,blue) \
   *to++ = GREY_VALUE(red,green,blue); from++

#define DO_GREY_APIXEL_FORWARD(red,green,blue) DO_GREY_PIXEL_FORWARD(red,green,blue)
#define DO_GREY_BPIXEL_FORWARD(red,green,blue) DO_GREY_PIXEL_FORWARD(red,green,blue)

#define DO_GREY_PIXEL_REVERSE(red,green,blue) \
   *--to = GREY_VALUE(red,green,blue); from++

#define DO_GREY_APIXEL_REVERSE(red,green,blue) DO_GREY_PIXEL_REVERSE(red,green,blue)
#define DO_GREY_BPIXEL_REVERSE(red,green,blue) DO_GREY_PIXEL_REVERSE(red,green,blue)

#define DO_COLOR_PIXEL_FORWARD(red,green,blue) \
   *to++ = red; *to++ = green; *to++ = blue; from++

#define DO_COLOR_APIXEL_FORWARD(red,green,blue) DO_COLOR_PIXEL_FORWARD(red,green,blue)
#define DO_COLOR_BPIXEL_FORWARD(red,green,blue) DO_COLOR_PIXEL_FORWARD(red,green,blue)

#define DO_COLOR_PIXEL_REVERSE(red,green,blue) \
   *--to = blue; *--to = green; *--to = red; from++

#define DO_COLOR_APIXEL_REVERSE(red,green,blue) DO_COLOR_PIXEL_REVERSE(red,green,blue)
#define DO_COLOR_BPIXEL_REVERSE(red,green,blue) DO_COLOR_PIXEL_REVERSE(red,green,blue)

/* 
   Uses:  to, from, rvalue, gvalue, bvalue, stored_value
*/

#define DO_YUV_APIXEL_FORWARD(red,green,blue) \
    rvalue = red; \
    gvalue = green;  \
    bvalue = blue;  \
   *to++ = Y_VALUE(rvalue, gvalue, bvalue );    \
   *to++ = Cb_VALUE(rvalue, gvalue, bvalue );   \
    stored_value = Cr_VALUE(rvalue, gvalue, bvalue ); \
    from++

#define DO_YUV_BPIXEL_FORWARD(red,green,blue) \
   *to++ = Y_VALUE(red,green,blue); \
   *to++ = stored_value; \
    from++

#define DO_YUV_APIXEL_REVERSE(red,green,blue) \
    rvalue = red; \
    gvalue = green;  \
    bvalue = blue;  \
   *--to = Cr_VALUE(rvalue, gvalue, bvalue );   \
   *--to = Y_VALUE(rvalue, gvalue, bvalue );    \
    stored_value = Cb_VALUE(rvalue, gvalue, bvalue ); \
    from++

#define DO_YUV_BPIXEL_REVERSE(red,green,blue) \
   *--to = stored_value; \
   *--to = Y_VALUE(red,green,blue); \
    from++
    
/* 
   Uses:  streaming, buf, temp_buf, temp_buf_size, to, result 
*/

#define DO_FLUSH_PIXELS_FORWARD \
    if (!streaming) { __copy_to_user( buf, temp_buf, temp_buf_size ); to = temp_buf;}    \
    buf += temp_buf_size; \
    result += temp_buf_size

#define DO_FLUSH_PIXELS_REVERSE \
    buf -= temp_buf_size; \
    if (!streaming) { __copy_to_user( buf, temp_buf, temp_buf_size ); to = temp_buf + temp_buf_size;}    \
    result += temp_buf_size

#define DO_P(value,flush,color) \
        DO_ ## color ## _PIXEL_ ## flush ##(R ## value, G ## value, B ## value)
      
#define DO_PA(value,flush,color) \
        DO_ ## color ## _APIXEL_ ## flush ##(R ## value, G ## value, B ## value)
      
#define DO_PB(value,flush,color) \
        DO_ ## color ## _BPIXEL_ ## flush ##(R ## value, G ## value, B ## value)
      
#define WRITE_QVGA_IMAGE(flush,color) \
        do { int i,j;                                            \
	DO_PA(11,flush,color);                                    \
	for ( i = 1 ; i < width - 1 ; i+=2 ) {	                 \
                DO_PB(12,flush,color);	                         \
                DO_PA(13,flush,color); 	                         \
        }                                                        \
	DO_PB(14,flush,color);	                                 \
	DO_FLUSH_PIXELS_ ## flush;                               \
	for ( j = 1 ; j < height - 1 ; j += 2 ) {                \
		DO_PA(21,flush,color);	                         \
		for ( i = 1 ; i < width - 1 ; i+= 2 ) {	         \
                        DO_PB(22,flush,color);                    \
                        DO_PA(23,flush,color);                    \
         	}                                                \
		DO_PB(24,flush,color);	                         \
                DO_FLUSH_PIXELS_ ## flush;                       \
		DO_PA(31,flush,color);                            \
		for ( i = 1 ; i < width - 1 ; i+= 2 ) {          \
 	                DO_PB(32,flush,color);                    \
                	DO_PA(33,flush,color);	                 \
                }                                                \
		DO_PB(34,flush,color);                            \
		DO_FLUSH_PIXELS_ ## flush;                       \
	}                                                        \
	DO_PA(41,flush,color);                                    \
	for ( i = 1 ; i < width - 1 ; i+= 2 ) {                  \
        	DO_PB(42,flush,color);	                         \
                DO_PA(43,flush,color);	                         \
        }                                                        \
	DO_PB(44,flush,color);                                    \
	DO_FLUSH_PIXELS_ ## flush; } while(0)

#define WRITE_VGA_IMAGE(flush,color) \
	do {  	int i, j;                                        \
		for ( j = 0 ; j < height ; ) {                   \
			from = source + (++j) * row_width + 1;   \
			for ( i = 0 ; i < width ; i += 2 ) {     \
                                DO_PA(22,flush,color);            \
                                DO_PB(23,flush,color);            \
                        }                                        \
			DO_FLUSH_PIXELS_ ## flush;               \
			from = source + (++j) * row_width + 1;   \
			for ( i = 0 ; i < width ; i += 2 ) {     \
                                DO_PA(32,flush,color);            \
                                DO_PB(33,flush,color);            \
                        }                                        \
			DO_FLUSH_PIXELS_ ## flush;               \
		}                                                \
	} while (0)


/*********************************************/

unsigned long write_qvga( struct frame *frame, unsigned char *buf, 
			  int streaming, int flipped, int palette )
{
	register unsigned char *from = frame->data;
	int width                    = frame->width;
	int height                   = frame->height;
	register int row_width       = width;
	unsigned char temp_buf[960];  /* 320 * 3 */
	register unsigned char *to;
	unsigned long           result = 0;
	int                     temp_buf_size;
	register p32            rvalue, gvalue, bvalue, stored_value;
	
	switch (palette) {
	case VIDEO_PALETTE_RGB24:
		temp_buf_size = width * 3;
		if ( !flipped ) {
			to = (streaming ? buf : temp_buf );
			WRITE_QVGA_IMAGE(FORWARD,COLOR);
		}
		else {
			buf += width * height * 3;
			to = (streaming ? buf : temp_buf + temp_buf_size );
			WRITE_QVGA_IMAGE(REVERSE,COLOR);
		}
		break;
	case VIDEO_PALETTE_GREY:
		temp_buf_size = width;
		if ( !flipped ) {
			to = (streaming ? buf : temp_buf );
			WRITE_QVGA_IMAGE(FORWARD,GREY);
		}
		else {
			buf += width * height;
			to = (streaming ? buf : temp_buf + temp_buf_size);
			WRITE_QVGA_IMAGE(REVERSE,GREY);
		}
		break;
	case VIDEO_PALETTE_YUV422:
		temp_buf_size = width * 2;
		if ( !flipped ) {
			to = (streaming ? buf : temp_buf );
			WRITE_QVGA_IMAGE(FORWARD,YUV);
		}
		else {
			buf += width * height * 2;
			to = (streaming ? buf : temp_buf + temp_buf_size);
			WRITE_QVGA_IMAGE(REVERSE,YUV);
		}
		break;
	}
	return result;
}

unsigned long write_vga( struct frame *frame, unsigned char *buf, 
			 int streaming, int flipped, int palette )
{
	unsigned char *source = frame->data;
	register unsigned char *from;
	int width                    = 640;
	int height                   = 480;
	register int row_width       = frame->width;
	unsigned char temp_buf[640 * 3];
	register unsigned char *to;
	unsigned long           result = 0;
	int                     temp_buf_size;
	register p32            rvalue, gvalue, bvalue, stored_value;

	switch (palette) {
	case VIDEO_PALETTE_RGB24:
		temp_buf_size = width * 3;
		if ( !flipped ) {
			to = (streaming ? buf : temp_buf );
			WRITE_VGA_IMAGE(FORWARD,COLOR);
		}
		else {
			buf += width * height * 3;
			to = (streaming ? buf : temp_buf + temp_buf_size );
			WRITE_VGA_IMAGE(REVERSE,COLOR);
		}
		break;
	case VIDEO_PALETTE_GREY:
		temp_buf_size = width;
		if ( !flipped ) {
			to = (streaming ? buf : temp_buf );
			WRITE_VGA_IMAGE(FORWARD,GREY);
		}
		else {
			buf += width * height;
			to = (streaming ? buf : temp_buf + temp_buf_size );
			WRITE_VGA_IMAGE(REVERSE,GREY);
		}
		break;
	case VIDEO_PALETTE_YUV422:
		temp_buf_size = width * 2;
		if ( !flipped ) {
			to = (streaming ? buf : temp_buf );
			WRITE_VGA_IMAGE(FORWARD,YUV);
		}
		else {
			buf += width * height * 2;
			to = (streaming ? buf : temp_buf + temp_buf_size);
			WRITE_VGA_IMAGE(REVERSE,YUV);
		}
		break;
	}
	return result;
}

/* Routines to handle 5:8 decimation */

/* from, row_width */

#define PR0_0 ((169 *((p32)*(from))+ \
       39 *((p32)*(from+2))+ \
       39 *((p32)*(from+2*row_width))+ \
       9 *((p32)*(from+2*row_width+2))) >> 8)

#define PG0_0 ((25 *((p32)*(from-row_width))+ \
       25 *((p32)*(from-1))+ \
       94 *((p32)*(from+1))+ \
       94 *((p32)*(from+row_width))+ \
       9 *((p32)*(from+row_width+2))+ \
       9 *((p32)*(from+2*row_width+1))) >> 8)

#define PB0_0 ((25 *((p32)*(from-row_width-1))+ \
       55 *((p32)*(from-row_width+1))+ \
       55 *((p32)*(from+row_width-1))+ \
       121 *((p32)*(from+row_width+1))) >> 8)

#define PR1_0 ((26 *((p32)*(from-1))+ \
       169 *((p32)*(from+1))+ \
       13 *((p32)*(from+3))+ \
       6 *((p32)*(from+2*row_width-1))+ \
       39 *((p32)*(from+2*row_width+1))+ \
       3 *((p32)*(from+2*row_width+3))) >> 8)

#define PG1_0 ((25 *((p32)*(from-row_width+1))+ \
       71 *((p32)*(from))+ \
       48 *((p32)*(from+2))+ \
       6 *((p32)*(from+row_width-1))+ \
       94 *((p32)*(from+row_width+1))+ \
       3 *((p32)*(from+row_width+3))+ \
       6 *((p32)*(from+2*row_width))+ \
       3 *((p32)*(from+2*row_width+2))) >> 8)

#define PB1_0 ((45 *((p32)*(from-row_width))+ \
       35 *((p32)*(from-row_width+2))+ \
       99 *((p32)*(from+row_width))+ \
       77 *((p32)*(from+row_width+2))) >> 8)

#define PR2_0 ((13 *((p32)*(from-1))+ \
       39 *((p32)*(from+1))+ \
       3 *((p32)*(from+2*row_width-1))+ \
       9 *((p32)*(from+2*row_width+1))) >> 6)

#define PG2_0 ((5 *((p32)*(from-row_width+1))+ \
       28 *((p32)*(from))+ \
       5 *((p32)*(from+2))+ \
       3 *((p32)*(from+row_width-1))+ \
       20 *((p32)*(from+row_width+1))+ \
       3 *((p32)*(from+2*row_width))) >> 6)

#define PB2_0 ((15 *((p32)*(from-row_width))+ \
       5 *((p32)*(from-row_width+2))+ \
       33 *((p32)*(from+row_width))+ \
       11 *((p32)*(from+row_width+2))) >> 6)

#define PR3_0 ((91 *((p32)*(from))+ \
       117 *((p32)*(from+2))+ \
       21 *((p32)*(from+2*row_width))+ \
       27 *((p32)*(from+2*row_width+2))) >> 8)

#define PG3_0 ((5 *((p32)*(from-row_width))+ \
       10 *((p32)*(from-row_width+2))+ \
       5 *((p32)*(from-1))+ \
       130 *((p32)*(from+1))+ \
       10 *((p32)*(from+3))+ \
       32 *((p32)*(from+row_width))+ \
       49 *((p32)*(from+row_width+2))+ \
       15 *((p32)*(from+2*row_width+1))) >> 8)

#define PB3_0 ((5 *((p32)*(from-row_width-1))+ \
       65 *((p32)*(from-row_width+1))+ \
       10 *((p32)*(from-row_width+3))+ \
       11 *((p32)*(from+row_width-1))+ \
       143 *((p32)*(from+row_width+1))+ \
       22 *((p32)*(from+row_width+3))) >> 8)

#define PR4_0 ((143 *((p32)*(from))+ \
       65 *((p32)*(from+2))+ \
       33 *((p32)*(from+2*row_width))+ \
       15 *((p32)*(from+2*row_width+2))) >> 8)

#define PG4_0 ((15 *((p32)*(from-row_width))+ \
       15 *((p32)*(from-1))+ \
       130 *((p32)*(from+1))+ \
       66 *((p32)*(from+row_width))+ \
       15 *((p32)*(from+row_width+2))+ \
       15 *((p32)*(from+2*row_width+1))) >> 8)

#define PB4_0 ((15 *((p32)*(from-row_width-1))+ \
       65 *((p32)*(from-row_width+1))+ \
       33 *((p32)*(from+row_width-1))+ \
       143 *((p32)*(from+row_width+1))) >> 8)

#define PR0_1 ((26 *((p32)*(from-row_width))+ \
       6 *((p32)*(from-row_width+2))+ \
       169 *((p32)*(from+row_width))+ \
       39 *((p32)*(from+row_width+2))+ \
       13 *((p32)*(from+3*row_width))+ \
       3 *((p32)*(from+3*row_width+2))) >> 8)

#define PG0_1 ((6 *((p32)*(from-row_width+1))+ \
       71 *((p32)*(from))+ \
       6 *((p32)*(from+2))+ \
       25 *((p32)*(from+row_width-1))+ \
       94 *((p32)*(from+row_width+1))+ \
       48 *((p32)*(from+2*row_width))+ \
       3 *((p32)*(from+2*row_width+2))+ \
       3 *((p32)*(from+3*row_width+1))) >> 8)

#define PB0_1 ((45 *((p32)*(from-1))+ \
       99 *((p32)*(from+1))+ \
       35 *((p32)*(from+2*row_width-1))+ \
       77 *((p32)*(from+2*row_width+1))) >> 8)

#define PR1_1 ((4 *((p32)*(from-row_width-1))+ \
       26 *((p32)*(from-row_width+1))+ \
       2 *((p32)*(from-row_width+3))+ \
       26 *((p32)*(from+row_width-1))+ \
       169 *((p32)*(from+row_width+1))+ \
       13 *((p32)*(from+row_width+3))+ \
       2 *((p32)*(from+3*row_width-1))+ \
       13 *((p32)*(from+3*row_width+1))+ \
       ((p32)*(from+3*row_width+3))) >> 8)

#define PG1_1 ((4 *((p32)*(from-row_width))+ \
       2 *((p32)*(from-row_width+2))+ \
       4 *((p32)*(from-1))+ \
       71 *((p32)*(from+1))+ \
       2 *((p32)*(from+3))+ \
       71 *((p32)*(from+row_width))+ \
       48 *((p32)*(from+row_width+2))+ \
       2 *((p32)*(from+2*row_width-1))+ \
       48 *((p32)*(from+2*row_width+1))+ \
       ((p32)*(from+2*row_width+3))+ \
       2 *((p32)*(from+3*row_width))+ \
       ((p32)*(from+3*row_width+2))) >> 8)

#define PB1_1 ((81 *((p32)*(from))+ \
       63 *((p32)*(from+2))+ \
       63 *((p32)*(from+2*row_width))+ \
       49 *((p32)*(from+2*row_width+2))) >> 8)

#define PR2_1 ((2 *((p32)*(from-row_width-1))+ \
       6 *((p32)*(from-row_width+1))+ \
       13 *((p32)*(from+row_width-1))+ \
       39 *((p32)*(from+row_width+1))+ \
       ((p32)*(from+3*row_width-1))+ \
       3 *((p32)*(from+3*row_width+1))) >> 6)

#define PG2_1 ((2 *((p32)*(from-row_width))+ \
       2 *((p32)*(from-1))+ \
       15 *((p32)*(from+1))+ \
       28 *((p32)*(from+row_width))+ \
       5 *((p32)*(from+row_width+2))+ \
       ((p32)*(from+2*row_width-1))+ \
       10 *((p32)*(from+2*row_width+1))+ \
       ((p32)*(from+3*row_width))) >> 6)

#define PB2_1 ((27 *((p32)*(from))+ \
       9 *((p32)*(from+2))+ \
       21 *((p32)*(from+2*row_width))+ \
       7 *((p32)*(from+2*row_width+2))) >> 6)

#define PR3_1 ((14 *((p32)*(from-row_width))+ \
       18 *((p32)*(from-row_width+2))+ \
       91 *((p32)*(from+row_width))+ \
       117 *((p32)*(from+row_width+2))+ \
       7 *((p32)*(from+3*row_width))+ \
       9 *((p32)*(from+3*row_width+2))) >> 8)

#define PG3_1 ((10 *((p32)*(from-row_width+1))+ \
       23 *((p32)*(from))+ \
       36 *((p32)*(from+2))+ \
       5 *((p32)*(from+row_width-1))+ \
       130 *((p32)*(from+row_width+1))+ \
       10 *((p32)*(from+row_width+3))+ \
       14 *((p32)*(from+2*row_width))+ \
       23 *((p32)*(from+2*row_width+2))+ \
       5 *((p32)*(from+3*row_width+1))) >> 8)

#define PB3_1 ((9 *((p32)*(from-1))+ \
       117 *((p32)*(from+1))+ \
       18 *((p32)*(from+3))+ \
       7 *((p32)*(from+2*row_width-1))+ \
       91 *((p32)*(from+2*row_width+1))+ \
       14 *((p32)*(from+2*row_width+3))) >> 8)

#define PR4_1 ((22 *((p32)*(from-row_width))+ \
       10 *((p32)*(from-row_width+2))+ \
       143 *((p32)*(from+row_width))+ \
       65 *((p32)*(from+row_width+2))+ \
       11 *((p32)*(from+3*row_width))+ \
       5 *((p32)*(from+3*row_width+2))) >> 8)

#define PG4_1 ((10 *((p32)*(from-row_width+1))+ \
       49 *((p32)*(from))+ \
       10 *((p32)*(from+2))+ \
       15 *((p32)*(from+row_width-1))+ \
       130 *((p32)*(from+row_width+1))+ \
       32 *((p32)*(from+2*row_width))+ \
       5 *((p32)*(from+2*row_width+2))+ \
       5 *((p32)*(from+3*row_width+1))) >> 8)

#define PB4_1 ((27 *((p32)*(from-1))+ \
       117 *((p32)*(from+1))+ \
       21 *((p32)*(from+2*row_width-1))+ \
       91 *((p32)*(from+2*row_width+1))) >> 8)

#define PR0_2 ((13 *((p32)*(from-row_width))+ \
       3 *((p32)*(from-row_width+2))+ \
       39 *((p32)*(from+row_width))+ \
       9 *((p32)*(from+row_width+2))) >> 6)

#define PG0_2 ((3 *((p32)*(from-row_width+1))+ \
       28 *((p32)*(from))+ \
       3 *((p32)*(from+2))+ \
       5 *((p32)*(from+row_width-1))+ \
       20 *((p32)*(from+row_width+1))+ \
       5 *((p32)*(from+2*row_width))) >> 6)

#define PB0_2 ((15 *((p32)*(from-1))+ \
       33 *((p32)*(from+1))+ \
       5 *((p32)*(from+2*row_width-1))+ \
       11 *((p32)*(from+2*row_width+1))) >> 6)

#define PR1_2 ((2 *((p32)*(from-row_width-1))+ \
       13 *((p32)*(from-row_width+1))+ \
       ((p32)*(from-row_width+3))+ \
       6 *((p32)*(from+row_width-1))+ \
       39 *((p32)*(from+row_width+1))+ \
       3 *((p32)*(from+row_width+3))) >> 6)

#define PG1_2 ((2 *((p32)*(from-row_width))+ \
       ((p32)*(from-row_width+2))+ \
       2 *((p32)*(from-1))+ \
       28 *((p32)*(from+1))+ \
       ((p32)*(from+3))+ \
       15 *((p32)*(from+row_width))+ \
       10 *((p32)*(from+row_width+2))+ \
       5 *((p32)*(from+2*row_width+1))) >> 6)

#define PB1_2 ((27 *((p32)*(from))+ \
       21 *((p32)*(from+2))+ \
       9 *((p32)*(from+2*row_width))+ \
       7 *((p32)*(from+2*row_width+2))) >> 6)

#define PR2_2 ((((p32)*(from-row_width-1))+ \
       3 *((p32)*(from-row_width+1))+ \
       3 *((p32)*(from+row_width-1))+ \
       9 *((p32)*(from+row_width+1))) >> 4)

#define PG2_2 ((((p32)*(from-row_width))+ \
       ((p32)*(from-1))+ \
       6 *((p32)*(from+1))+ \
       6 *((p32)*(from+row_width))+ \
       ((p32)*(from+row_width+2))+ \
       ((p32)*(from+2*row_width+1))) >> 4)

#define PB2_2 ((9 *((p32)*(from))+ \
       3 *((p32)*(from+2))+ \
       3 *((p32)*(from+2*row_width))+ \
       ((p32)*(from+2*row_width+2))) >> 4)

#define PR3_2 ((7 *((p32)*(from-row_width))+ \
       9 *((p32)*(from-row_width+2))+ \
       21 *((p32)*(from+row_width))+ \
       27 *((p32)*(from+row_width+2))) >> 6)

#define PG3_2 ((5 *((p32)*(from-row_width+1))+ \
       10 *((p32)*(from))+ \
       15 *((p32)*(from+2))+ \
       ((p32)*(from+row_width-1))+ \
       28 *((p32)*(from+row_width+1))+ \
       2 *((p32)*(from+row_width+3))+ \
       ((p32)*(from+2*row_width))+ \
       2 *((p32)*(from+2*row_width+2))) >> 6)

#define PB3_2 ((3 *((p32)*(from-1))+ \
       39 *((p32)*(from+1))+ \
       6 *((p32)*(from+3))+ \
       ((p32)*(from+2*row_width-1))+ \
       13 *((p32)*(from+2*row_width+1))+ \
       2 *((p32)*(from+2*row_width+3))) >> 6)

#define PR4_2 ((11 *((p32)*(from-row_width))+ \
       5 *((p32)*(from-row_width+2))+ \
       33 *((p32)*(from+row_width))+ \
       15 *((p32)*(from+row_width+2))) >> 6)

#define PG4_2 ((5 *((p32)*(from-row_width+1))+ \
       20 *((p32)*(from))+ \
       5 *((p32)*(from+2))+ \
       3 *((p32)*(from+row_width-1))+ \
       28 *((p32)*(from+row_width+1))+ \
       3 *((p32)*(from+2*row_width))) >> 6)

#define PB4_2 ((9 *((p32)*(from-1))+ \
       39 *((p32)*(from+1))+ \
       3 *((p32)*(from+2*row_width-1))+ \
       13 *((p32)*(from+2*row_width+1))) >> 6)

#define PR0_3 ((91 *((p32)*(from))+ \
       21 *((p32)*(from+2))+ \
       117 *((p32)*(from+2*row_width))+ \
       27 *((p32)*(from+2*row_width+2))) >> 8)

#define PG0_3 ((5 *((p32)*(from-row_width))+ \
       5 *((p32)*(from-1))+ \
       32 *((p32)*(from+1))+ \
       130 *((p32)*(from+row_width))+ \
       15 *((p32)*(from+row_width+2))+ \
       10 *((p32)*(from+2*row_width-1))+ \
       49 *((p32)*(from+2*row_width+1))+ \
       10 *((p32)*(from+3*row_width))) >> 8)

#define PB0_3 ((5 *((p32)*(from-row_width-1))+ \
       11 *((p32)*(from-row_width+1))+ \
       65 *((p32)*(from+row_width-1))+ \
       143 *((p32)*(from+row_width+1))+ \
       10 *((p32)*(from+3*row_width-1))+ \
       22 *((p32)*(from+3*row_width+1))) >> 8)

#define PR1_3 ((14 *((p32)*(from-1))+ \
       91 *((p32)*(from+1))+ \
       7 *((p32)*(from+3))+ \
       18 *((p32)*(from+2*row_width-1))+ \
       117 *((p32)*(from+2*row_width+1))+ \
       9 *((p32)*(from+2*row_width+3))) >> 8)

#define PG1_3 ((5 *((p32)*(from-row_width+1))+ \
       23 *((p32)*(from))+ \
       14 *((p32)*(from+2))+ \
       10 *((p32)*(from+row_width-1))+ \
       130 *((p32)*(from+row_width+1))+ \
       5 *((p32)*(from+row_width+3))+ \
       36 *((p32)*(from+2*row_width))+ \
       23 *((p32)*(from+2*row_width+2))+ \
       10 *((p32)*(from+3*row_width+1))) >> 8)

#define PB1_3 ((9 *((p32)*(from-row_width))+ \
       7 *((p32)*(from-row_width+2))+ \
       117 *((p32)*(from+row_width))+ \
       91 *((p32)*(from+row_width+2))+ \
       18 *((p32)*(from+3*row_width))+ \
       14 *((p32)*(from+3*row_width+2))) >> 8)

#define PR2_3 ((7 *((p32)*(from-1))+ \
       21 *((p32)*(from+1))+ \
       9 *((p32)*(from+2*row_width-1))+ \
       27 *((p32)*(from+2*row_width+1))) >> 6)

#define PG2_3 ((((p32)*(from-row_width+1))+ \
       10 *((p32)*(from))+ \
       ((p32)*(from+2))+ \
       5 *((p32)*(from+row_width-1))+ \
       28 *((p32)*(from+row_width+1))+ \
       15 *((p32)*(from+2*row_width))+ \
       2 *((p32)*(from+2*row_width+2))+ \
       2 *((p32)*(from+3*row_width+1))) >> 6)

#define PB2_3 ((3 *((p32)*(from-row_width))+ \
       ((p32)*(from-row_width+2))+ \
       39 *((p32)*(from+row_width))+ \
       13 *((p32)*(from+row_width+2))+ \
       6 *((p32)*(from+3*row_width))+ \
       2 *((p32)*(from+3*row_width+2))) >> 6)

#define PR3_3 ((49 *((p32)*(from))+ \
       63 *((p32)*(from+2))+ \
       63 *((p32)*(from+2*row_width))+ \
       81 *((p32)*(from+2*row_width+2))) >> 8)

#define PG3_3 ((((p32)*(from-row_width))+ \
       2 *((p32)*(from-row_width+2))+ \
       ((p32)*(from-1))+ \
       48 *((p32)*(from+1))+ \
       2 *((p32)*(from+3))+ \
       48 *((p32)*(from+row_width))+ \
       71 *((p32)*(from+row_width+2))+ \
       2 *((p32)*(from+2*row_width-1))+ \
       71 *((p32)*(from+2*row_width+1))+ \
       4 *((p32)*(from+2*row_width+3))+ \
       2 *((p32)*(from+3*row_width))+ \
       4 *((p32)*(from+3*row_width+2))) >> 8)

#define PB3_3 ((((p32)*(from-row_width-1))+ \
       13 *((p32)*(from-row_width+1))+ \
       2 *((p32)*(from-row_width+3))+ \
       13 *((p32)*(from+row_width-1))+ \
       169 *((p32)*(from+row_width+1))+ \
       26 *((p32)*(from+row_width+3))+ \
       2 *((p32)*(from+3*row_width-1))+ \
       26 *((p32)*(from+3*row_width+1))+ \
       4 *((p32)*(from+3*row_width+3))) >> 8)

#define PR4_3 ((77 *((p32)*(from))+ \
       35 *((p32)*(from+2))+ \
       99 *((p32)*(from+2*row_width))+ \
       45 *((p32)*(from+2*row_width+2))) >> 8)

#define PG4_3 ((3 *((p32)*(from-row_width))+ \
       3 *((p32)*(from-1))+ \
       48 *((p32)*(from+1))+ \
       94 *((p32)*(from+row_width))+ \
       25 *((p32)*(from+row_width+2))+ \
       6 *((p32)*(from+2*row_width-1))+ \
       71 *((p32)*(from+2*row_width+1))+ \
       6 *((p32)*(from+3*row_width))) >> 8)

#define PB4_3 ((3 *((p32)*(from-row_width-1))+ \
       13 *((p32)*(from-row_width+1))+ \
       39 *((p32)*(from+row_width-1))+ \
       169 *((p32)*(from+row_width+1))+ \
       6 *((p32)*(from+3*row_width-1))+ \
       26 *((p32)*(from+3*row_width+1))) >> 8)

#define PR0_4 ((143 *((p32)*(from))+ \
       33 *((p32)*(from+2))+ \
       65 *((p32)*(from+2*row_width))+ \
       15 *((p32)*(from+2*row_width+2))) >> 8)

#define PG0_4 ((15 *((p32)*(from-row_width))+ \
       15 *((p32)*(from-1))+ \
       66 *((p32)*(from+1))+ \
       130 *((p32)*(from+row_width))+ \
       15 *((p32)*(from+row_width+2))+ \
       15 *((p32)*(from+2*row_width+1))) >> 8)

#define PB0_4 ((15 *((p32)*(from-row_width-1))+ \
       33 *((p32)*(from-row_width+1))+ \
       65 *((p32)*(from+row_width-1))+ \
       143 *((p32)*(from+row_width+1))) >> 8)

#define PR1_4 ((22 *((p32)*(from-1))+ \
       143 *((p32)*(from+1))+ \
       11 *((p32)*(from+3))+ \
       10 *((p32)*(from+2*row_width-1))+ \
       65 *((p32)*(from+2*row_width+1))+ \
       5 *((p32)*(from+2*row_width+3))) >> 8)

#define PG1_4 ((15 *((p32)*(from-row_width+1))+ \
       49 *((p32)*(from))+ \
       32 *((p32)*(from+2))+ \
       10 *((p32)*(from+row_width-1))+ \
       130 *((p32)*(from+row_width+1))+ \
       5 *((p32)*(from+row_width+3))+ \
       10 *((p32)*(from+2*row_width))+ \
       5 *((p32)*(from+2*row_width+2))) >> 8)

#define PB1_4 ((27 *((p32)*(from-row_width))+ \
       21 *((p32)*(from-row_width+2))+ \
       117 *((p32)*(from+row_width))+ \
       91 *((p32)*(from+row_width+2))) >> 8)

#define PR2_4 ((11 *((p32)*(from-1))+ \
       33 *((p32)*(from+1))+ \
       5 *((p32)*(from+2*row_width-1))+ \
       15 *((p32)*(from+2*row_width+1))) >> 6)

#define PG2_4 ((3 *((p32)*(from-row_width+1))+ \
       20 *((p32)*(from))+ \
       3 *((p32)*(from+2))+ \
       5 *((p32)*(from+row_width-1))+ \
       28 *((p32)*(from+row_width+1))+ \
       5 *((p32)*(from+2*row_width))) >> 6)

#define PB2_4 ((9 *((p32)*(from-row_width))+ \
       3 *((p32)*(from-row_width+2))+ \
       39 *((p32)*(from+row_width))+ \
       13 *((p32)*(from+row_width+2))) >> 6)

#define PR3_4 ((77 *((p32)*(from))+ \
       99 *((p32)*(from+2))+ \
       35 *((p32)*(from+2*row_width))+ \
       45 *((p32)*(from+2*row_width+2))) >> 8)

#define PG3_4 ((3 *((p32)*(from-row_width))+ \
       6 *((p32)*(from-row_width+2))+ \
       3 *((p32)*(from-1))+ \
       94 *((p32)*(from+1))+ \
       6 *((p32)*(from+3))+ \
       48 *((p32)*(from+row_width))+ \
       71 *((p32)*(from+row_width+2))+ \
       25 *((p32)*(from+2*row_width+1))) >> 8)

#define PB3_4 ((3 *((p32)*(from-row_width-1))+ \
       39 *((p32)*(from-row_width+1))+ \
       6 *((p32)*(from-row_width+3))+ \
       13 *((p32)*(from+row_width-1))+ \
       169 *((p32)*(from+row_width+1))+ \
       26 *((p32)*(from+row_width+3))) >> 8)

#define PR4_4 ((121 *((p32)*(from))+ \
       55 *((p32)*(from+2))+ \
       55 *((p32)*(from+2*row_width))+ \
       25 *((p32)*(from+2*row_width+2))) >> 8)

#define PG4_4 ((9 *((p32)*(from-row_width))+ \
       9 *((p32)*(from-1))+ \
       94 *((p32)*(from+1))+ \
       94 *((p32)*(from+row_width))+ \
       25 *((p32)*(from+row_width+2))+ \
       25 *((p32)*(from+2*row_width+1))) >> 8)

#define PB4_4 ((9 *((p32)*(from-row_width-1))+ \
       39 *((p32)*(from-row_width+1))+ \
       39 *((p32)*(from+row_width-1))+ \
       169 *((p32)*(from+row_width+1))) >> 8)


//Offsets: 0 1 0 1 1 

#define DO_BIGP(x,y,flush,color) \
        DO_ ## color ## _PIXEL_ ## flush ##(PR ## x ## _ ## y, \
                                            PG ## x ## _ ## y, \
                                            PB ## x ## _ ## y )
      
#define DO_BIGPA(x,y,flush,color) \
        DO_ ## color ## _APIXEL_ ## flush ##(PR ## x ## _ ## y, \
                                            PG ## x ## _ ## y, \
                                            PB ## x ## _ ## y )
      
#define DO_BIGPB(x,y,flush,color) \
        DO_ ## color ## _BPIXEL_ ## flush ##(PR ## x ## _ ## y, \
                                            PG ## x ## _ ## y, \
                                            PB ## x ## _ ## y )
            
/* Offsets: 0 1 0 1 1  */

/* Uses:  from, source, source_row, row_width, 
          streaming, buf, temp_buf, temp_buf_size, 
	  rvalue, gvalue, bvalue, stored_value */

#define CIF_ROW(x,flush,color) \
    do { int i; \
	for ( i = 0 ; i < 35 ; i++ ) { \
		DO_BIGPA(1,x,flush,color); from++; \
		DO_BIGPB(2,x,flush,color); \
		DO_BIGPA(3,x,flush,color); from++; \
		DO_BIGPB(4,x,flush,color); from++; \
                DO_BIGPA(0,x,flush,color); \
		DO_BIGPB(1,x,flush,color); from++; \
		DO_BIGPA(2,x,flush,color); \
		DO_BIGPB(3,x,flush,color); from++; \
		DO_BIGPA(4,x,flush,color); from++; \
                DO_BIGPB(0,x,flush,color); \
        } \
        DO_BIGPA(1,x,flush,color); from++; \
	DO_BIGPB(2,x,flush,color); \
       } while (0)

#define QCIF_ROW(x,flush,color) \
    do { int i; \
	for ( i = 0 ; i < 17 ; i++ ) { /* Do 17 * 10 = 170 columns */ \
		DO_BIGPA(1,x,flush,color); from++; \
		DO_BIGPB(2,x,flush,color); \
		DO_BIGPA(3,x,flush,color); from++; \
		DO_BIGPB(4,x,flush,color); from++; \
                DO_BIGPA(0,x,flush,color); \
		DO_BIGPB(1,x,flush,color); from++; \
		DO_BIGPA(2,x,flush,color); \
		DO_BIGPB(3,x,flush,color); from++; \
		DO_BIGPA(4,x,flush,color); from++; \
                DO_BIGPB(0,x,flush,color); \
        } \
        DO_BIGPA(1,x,flush,color); from++; \
	DO_BIGPB(2,x,flush,color); \
	DO_BIGPA(3,x,flush,color); from++; \
	DO_BIGPB(4,x,flush,color); from++; \
        DO_BIGPA(0,x,flush,color); \
	DO_BIGPB(1,x,flush,color); from++; \
    } while (0)


#define WRITE_CIF_ROW(therow) \
int write_cif_row_ ## therow (  register unsigned char *from, \
                                register unsigned char *to,   \
		                int row_width,                \
		                int palette,                  \
		                int flipped )                 \
{                                                             \
	register p32 rvalue, gvalue, bvalue, stored_value;    \
        int byte_count = 0;                                   \
                                                              \
	switch ( palette ) {                                  \
	case VIDEO_PALETTE_RGB24:                             \
                byte_count = 352 * 3;                         \
		if ( flipped ) {                              \
         		to += byte_count;                     \
			CIF_ROW(therow,REVERSE,COLOR);        \
		} else {                                      \
			CIF_ROW(therow,FORWARD,COLOR);        \
                }                                             \
		break;                                        \
	case VIDEO_PALETTE_GREY:                              \
                byte_count = 352;                             \
		if ( flipped ) {                              \
			to += byte_count;                     \
			CIF_ROW(therow,REVERSE,GREY);         \
		} else {                                      \
			CIF_ROW(therow,FORWARD,GREY);         \
                }                                             \
		break;                                        \
	case VIDEO_PALETTE_YUV422:                            \
                byte_count = 352 * 2;                         \
		if ( flipped ) {                              \
			to += byte_count;                     \
			CIF_ROW(therow,REVERSE,YUV);          \
		} else {                                      \
			CIF_ROW(therow,FORWARD,YUV);          \
                }                                             \
		break;                                        \
	}                                                     \
        return byte_count;                                    \
}

WRITE_CIF_ROW(0)
WRITE_CIF_ROW(1)
WRITE_CIF_ROW(2)
WRITE_CIF_ROW(3)
WRITE_CIF_ROW(4)

#define WRITE_QCIF_ROW(therow) \
int write_qcif_row_ ## therow (  register unsigned char *from, \
                                register unsigned char *to,   \
		                int row_width,                \
		                int palette,                  \
		                int flipped )                 \
{                                                             \
	register p32 rvalue, gvalue, bvalue, stored_value;    \
        int byte_count = 0;                                   \
                                                              \
	switch ( palette ) {                                  \
	case VIDEO_PALETTE_RGB24:                             \
                byte_count = 176 * 3;                         \
		if ( flipped ) {                              \
         		to += byte_count;                     \
			QCIF_ROW(therow,REVERSE,COLOR);       \
		} else {                                      \
			QCIF_ROW(therow,FORWARD,COLOR);       \
                }                                             \
		break;                                        \
	case VIDEO_PALETTE_GREY:                              \
                byte_count = 176;                             \
		if ( flipped ) {                              \
			to += byte_count;                     \
			QCIF_ROW(therow,REVERSE,GREY);        \
		} else {                                      \
			QCIF_ROW(therow,FORWARD,GREY);        \
                }                                             \
		break;                                        \
	case VIDEO_PALETTE_YUV422:                            \
                byte_count = 176 * 2;                         \
		if ( flipped ) {                              \
			to += byte_count;                     \
			QCIF_ROW(therow,REVERSE,YUV);         \
		} else {                                      \
			QCIF_ROW(therow,FORWARD,YUV);         \
                }                                             \
		break;                                        \
	}                                                     \
        return byte_count;                                    \
}

WRITE_QCIF_ROW(0)
WRITE_QCIF_ROW(1)
WRITE_QCIF_ROW(2)
WRITE_QCIF_ROW(3)
WRITE_QCIF_ROW(4)

#define YUV_FLUSH_PIXELS_FORWARD \
    if (streaming) { \
        to += bytes_written; \
    } else { \
        __copy_to_user( buf, temp_buf, bytes_written ); \
        to = temp_buf; \
        buf += bytes_written; \
    } \
    result += bytes_written

#define YUV_FLUSH_PIXELS_REVERSE \
    if (streaming) { \
        to -= bytes_written; \
    } else { \
        __copy_to_user( buf, temp_buf, bytes_written ); \
        buf -= bytes_written; \
        to = temp_buf; \
    } \
    result += bytes_written

#define DO_CIF_ROW(x,flush) \
	from = source + source_row * row_width + 1; \
	bytes_written = write_cif_row_ ## x( from, to, row_width, palette, flipped ); \
	YUV_FLUSH_PIXELS_ ## flush

#define DO_QCIF_ROW(x,flush) \
	from = source + source_row * row_width + 1; \
	bytes_written = write_qcif_row_ ## x( from, to, row_width, palette, flipped ); \
	YUV_FLUSH_PIXELS_ ## flush


#define WRITE_CIF_IMAGE(flush) \
	do {  	\
		int j;                                                      \
                int bytes_written;                                          \
                to = ( streaming ? buf : temp_buf );                        \
		for ( j = 0 ; j < 57 ; j++ ) {  /* Do 5 * 57 = 285 rows */  \
			source_row++;    DO_CIF_ROW(1,flush); \
			source_row += 2; DO_CIF_ROW(2,flush); \
			source_row++;    DO_CIF_ROW(3,flush); \
			source_row += 2; DO_CIF_ROW(4,flush); \
			source_row += 2; DO_CIF_ROW(0,flush); \
		} \
		source_row++;    DO_CIF_ROW(1,flush); \
		source_row += 2; DO_CIF_ROW(2,flush); \
		source_row++;    DO_CIF_ROW(3,flush); \
	} while (0)

#define WRITE_QCIF_IMAGE(flush) \
	do {  	\
		int j, bytes_written;  \
                to = ( streaming ? buf : temp_buf ); \
		for ( j = 0 ; j < 28 ; j++ ) { \
			source_row++;    DO_QCIF_ROW(1,flush); \
			source_row += 2; DO_QCIF_ROW(2,flush); \
			source_row++;    DO_QCIF_ROW(3,flush); \
			source_row += 2; DO_QCIF_ROW(4,flush); \
			source_row += 2; DO_QCIF_ROW(0,flush); \
		} \
		source_row++;    DO_QCIF_ROW(1,flush); \
		source_row += 2; DO_QCIF_ROW(2,flush); \
		source_row++;    DO_QCIF_ROW(3,flush); \
		source_row += 2; DO_QCIF_ROW(4,flush); \
	} while (0)

unsigned long write_cif( struct frame *frame, unsigned char *buf, 
			 int streaming, int flipped, int palette )
{
	unsigned char *source    = frame->data;
	register int row_width   = frame->width;

	unsigned char temp_buf[352 * 3];
	register unsigned char *to;
	register unsigned char *from;
	unsigned long           result = 0;
	int                     source_row = 0;

	if ( flipped ) {
		switch (palette) {
		case VIDEO_PALETTE_RGB24:
			buf += 352 * (288 - 1) * 3;
			break;
		case VIDEO_PALETTE_GREY:
			buf += 352 * (288 - 1);
			break;
		case VIDEO_PALETTE_YUV422:
			buf += 352 * (288 - 1) * 2;
			break;
		}
		WRITE_CIF_IMAGE(REVERSE);
	}
	else {
		WRITE_CIF_IMAGE(FORWARD);
	}
	return result;
}

unsigned long write_qcif( struct frame *frame, unsigned char *buf, 
			  int streaming, int flipped, int palette )
{
	unsigned char *source    = frame->data;
	register int row_width   = frame->width;

	unsigned char temp_buf[176 * 3];
	register unsigned char *to;
	register unsigned char *from;
	unsigned long           result = 0;
	int                     source_row = 0;

	if ( flipped ) {
		switch (palette) {
		case VIDEO_PALETTE_RGB24:
			buf += 176 * (144 - 1) * 3;
			break;
		case VIDEO_PALETTE_GREY:
			buf += 176 * (144 - 1);
			break;
		case VIDEO_PALETTE_YUV422:
			buf += 176 * (144 - 1) * 2;
			break;
		}
		WRITE_QCIF_IMAGE(REVERSE);
	}
	else {
		WRITE_QCIF_IMAGE(FORWARD);
	}
	return result;
}
