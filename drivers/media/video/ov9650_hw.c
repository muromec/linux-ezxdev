/* 
    ov9650_hw.c - Omnivision 9650 CMOS sensor driver 

    Copyright (C) 2003, Intel Corporation
    Copyright (C) 2004 Motorola Inc.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

/*
Revision History:
                            Modification     Tracking
Author                 Date          Number     Description of Changes
----------------   ------------    ----------   -------------------------
Mu Chenliang       08/03/2004                   Modified from ov9640_hw.c   
Mu Chenliang       12/06/2004                   Add 1280x1024 support
*/

#include <linux/types.h>
#include <linux/delay.h>
#include <asm/hardware.h>
#include <asm/dma.h>

#include "camera.h"
#include "ov9650.h"
#include "ov9650_hw.h"

ov9650 g_ov9650;
/***********************************************************************
*  Attention: This is platform related!
***********************************************************************/
volatile int ov9650_step = 0;

/***********************************************************************
*  Register Settings
***********************************************************************/

static u8 gInit_9650[] = {
    /*
	//0x12,  0x80,	//0x12
	0x11,  0x87,	//0x11
	0x39,  0x40,	//0x39
	0x13,  0x80,	//0x13
	0x01,  0x80,	//0x01
	0x02,  0x80,	//0x
	0x00,  0x00,	//0x
	0x0c,  0x05,	//0x            stop frame
	0x0d,  0x80,	//0x
	0x12,  0x10,	//0x
	0x10,  0xf0,	//0x
	0xa1,  0x00,	//0x
	0x3a,  0x0c,	//0x            ???
	0x8c,  0x23,	//0x
	0x3d,  0x92,	//0x
	0x18,  0xc6,	//0x
	0x17,  0x26,	//0x
	0x1b,  0x01,	//0x
	0x16,  0x02,	//0x
	0x33,  0x10,	//0x
	0x34,  0x38,	//0x
	0xa8,  0x81,	//0x
	0x41,  0x10,	//0x
	0x96,  0x04,	//0x
*/
	//0x12,  0x80,	//0x12
	0x11,  0x83,	//0x11
	0x39,  0x40,	//0x39
	0x0e,  0x01,	//0x
	0x38,  0x12,	//0x
	0x13,  0xc7,	//0x13
	0x14,  0x2a,	//Gain Control
	0x1e,  0x04,	//0x01

	0x01,  0x80,	//0x
	0x02,  0x80,	//0x
	0x00,  0x00,	//0x
	0x10,  0xf0,	//0x

	0x12,  0x10,	//0x
	0x0c,  0x05,	//0x            stop frame
	0x0d,  0x80,	//0x
	//0x3a,  0x0d,	//enable Digital BLC, Rev0 0x0c

	//0x1b,  0x00,	//0x
	0x16,  0x06,	//0x
	0x33,  0x10,	//0x
	0x34,  0xbf,	//0x
	0xa8,  0x81,	//0x
	0x41,  0x10,	//0x
	0x96,  0x04,	//0x
	0x3d,  0x19,	//0x
	0x1b,  0x01,	//0x

        0x18,  0xc6,
        0x17,  0x26,
        0x32,  0xa4,
        0x03,  0x36,
        //0x1a,  0x1e,
        //0x19,  0x00,
        0x8e,  0x00,
        
        0x3c,  0x60,
        0x8f,  0xcf,
        0x8b,  0x06,
        0x35,  0x91,
        0x94,  0x99,
        0x95,  0x99,
        0x40,  0xc1,
        0x29,  0x2f,
        //0x0f,  0x42,    //Rev0 0x4a
        0x27,  0x90,
        0x28,  0x8c,
        0x2c,  0x08,
        0xa5,  0x80,

        0x41,  0x00,
        0x13,  0xc5,
        0x26,  0xe2,

        0x3d,  0x92,
        0x69,  0x80,
        0x43,  0xf0,
        0x44,  0x10,
        0x45,  0x67,
        0x46,  0x96,
        0x47,  0x4e,
        0x48,  0x9a,
        0x59,  0x36,
        0x5a,  0x73,
        0x5b,  0x55,
        0x5c,  0xa8,
        0x5d,  0x84,
        0x5e,  0x0f,
        0x5f,  0xf0,
        0x60,  0x0c,
        0x61,  0x20,

        0xa5,  0xd9,
        0xa4,  0x74,
        0x8d,  0x02,

        0x13,  0xc7,

        0x4f,  0x32,
        0x50,  0x27,
        0x51,  0x0b,
        0x52,  0x0d,
        0x53,  0x48,
        0x54,  0x55,
        0x41,  0x32,

        0x8c,  0x23,
        0x3d,  0x92,
        0x3e,  0x02,
        0xa9,  0x97,

        0x8f,  0xcf,
        0x90,  0x00,
        0x91,  0x00,
        0x9f,  0x00,
        0xa0,  0x00,
	0xff,0,	// End of list delimiter.        

	0xff,0	// End of list delimiter.        
};

const static u8 gYUV_QQVGA[] = {
	OV9650_COM10, 0x20,
	OV9650_COM7, 0x10,
	OV9650_COM1, 0x24,
	OV9650_CLKRC, 0x01,

	// From OmniVision
	OV9650_REGEND, 0x00	// End of list delimiter.
};

const static u8 gYUV_QQCIF[] = {
	OV9650_COM10, 0x20,
	OV9650_COM7, 0x08,
	OV9650_COM1, 0x24,
	OV9650_CLKRC, 0x01,

	// From OmniVision
	OV9650_REGEND, 0x00	// End of list delimiter.
};

const static u8 gYUV_VGA[] = {
	0x0d, 0x80,
	0x11, 0x81,
	0x12, 0x40,
        0x18, 0xc6,
        0x17, 0x26,
        0x32, 0xad,
        0x03, 0x00,
        0x1a, 0x3d,
        0x19, 0x01,
	OV9650_REGEND, 0x00	// End of list delimiter.
};
const static u8 gYUV_QVGA[] = {
	//0x0c, 0x05,
	0x0d, 0x80,
	0x11, 0x83,
	0x12, 0x10,
        0x18, 0xc6,
        0x17, 0x26,
        0x32, 0xa4,
        0x03, 0x36,
        0x1a, 0x1e,
        0x19, 0x00,
	OV9650_REGEND, 0x00	// End of list delimiter.
};
const static u8 gYUV_CIF[] = {
	OV9650_REGEND, 0x00	// End of list delimiter.
};

const static u8 gYUV_QCIF[] = {
	OV9650_REGEND, 0x00	// End of list delimiter.
};

const static u8 gYUV_SXGA[] = {
	// From OmniVision
	0x0d, 0x40,
	0x11, 0x80,
	0x12, 0x00,
        0x18, 0xbd,
        0x17, 0x1d,
        0x32, 0xad,
        0x03, 0x12,
        0x1a, 0x81,
        0x19, 0x01,
	OV9650_REGEND, 0x00	// End of list delimiter.        
};


const static u8 gRGB_QQVGA[] = {
	OV9650_COM7, 0x80,
	OV9650_REGEND, 0x00	// End of list delimiter.
};

const static u8 gRGB_QVGA[] = {
	OV9650_COM7, 0x80,
	OV9650_REGEND, 0x00	// End of list delimiter.
};

const static u8 gRGB_QCIF[] = {
	OV9650_COM7, 0x80,
	OV9650_REGEND, 0x00	// End of list delimiter.
};


const static u8 gRGB_VGA[] = {
	OV9650_COM7, 0x80,
	OV9650_REGEND, 0x00	// End of list delimiter.
};

const static u8 gRGB_CIF[] = {
	OV9650_COM7, 0x80,
	OV9650_REGEND, 0x00	// End of list delimiter.
};

const static u8 gRGB_SXGA[] = {
	OV9650_COM7, 0x80,
	OV9650_REGEND, 0x00	// End of list delimiter.
};

/***********************************************************************
*  Private/helper api
***********************************************************************/
static void ov9650_wait(int ms)
{
	mdelay(ms);
}

/***********************************************************************
*  Sensor read/write 
***********************************************************************/
// TODO: temp
static int ov9650_read(u8 addr, u8 *pvalue)
{
    return i2c_ov9650_read(addr, pvalue);
}

static int ov9650_write(u8 addr, u8 value)
{
    return i2c_ov9650_write(addr, value);
}
// end TODO

static int prv_read_sensor_reg(const u8 subAddress, u8 * bufP)
{
	return ov9650_read(subAddress, bufP);
}

static int prv_write_sensor_reg(const u8 subAddress, u8 * bufP)
{
	return ov9650_write(subAddress, *bufP);
}

static int ov9650_set_regs(u8 * regP)
{
	u32 curReg = 0;
	int status = 0;

	// The list is a register number followed by the value.
	while (regP[curReg << 1] < OV9650_REGEND) {
		status = prv_write_sensor_reg(regP[curReg << 1], &regP[(curReg << 1) + 1]);
		if (curReg == 0)
			ov9650_wait(1);

		curReg++;
	}
	return status;
}

static void ov9650_print_all_regs(void)
{
#if 0
    // read all register after power up
	ov9650 *pov;
	pov = &g_ov9650;
    int i, maxreg;
    maxreg = OV9650_LASTREG;

    dbg_print("start read all default register (0x%x)", maxreg);
    for(i=0; i<=maxreg; i++)
    {
        char value;
	    ov9650_read(i, &value);
        //dbg_print("reg addr(0x%x) value(0x%x)", i, value);
    }
    dbg_print("end read all default register");
#endif
}

/***********************************************************************
*  Power & Reset
***********************************************************************/
void ov9650_power_down(int powerDown)
{
	// OV9650 PWRDWN, 0 = NORMAL, 1=POWER DOWN
	//GPDR1 |= GPIO_bit(GPIO_CAM_EN);
	//OV9650 reset CIF_RST, 0 = NORMAL, 1=RESET
	//GPDR0 |= GPIO_bit(GPIO_CAM_RST);
	if (powerDown == 1) {
		GPSR1 = GPIO_bit(GPIO_CAM_EN);
	}
	else {
		GPCR1 = GPIO_bit(GPIO_CAM_EN);
		//GPSR0 = GPIO_bit(GPIO_CAM_RST);
		//mdelay(20);
		//GPCR0 = GPIO_bit(GPIO_CAM_RST);
	}
	mdelay(100);
}

void ov9650_soft_reset(void)
{
    ov9650 *pov = &g_ov9650;
	u8 regValue;
	regValue = 0x80;
	prv_write_sensor_reg(OV9650_COM7, &regValue);
	mdelay(10);

    ov9650_print_all_regs();

    ov9650_set_regs(gInit_9650);

    //0x3a,  0x0d,	//enable Digital BLC, Rev0 0x0c
    //0x0f,  0x42,    //Rev0 0x4a
    if ( (pov->version&0xff) == PID_9650_0 ) 
    {
		ov9650_write(0x3a, 0x0c);
		ov9650_write(0x0f, 0x4a);
    }
    else
    {
		ov9650_write(0x3a, 0x0d);
		ov9650_write(0x0f, 0x42);
    }

    dbg_print("end initial register");
	return;
}

int ov9650_output_stoped()
{
	ov9650 *pov;
	pov = &g_ov9650;
	return pov->stoped;
}

void ov9650_set_start()
{
	ov9650 *pov;
	pov = &g_ov9650;

	pov->stoped = 0;
}

void ov9650_set_stop(p_camera_context_t cam_ctx)
{
	ov9650 *pov;
	pov = &g_ov9650;

	pov->stoped = 1;
}

/***********************************************************************
*  Settings
***********************************************************************/
int ov9650_version_revision(u8 * pCmRevision, u8 * pSensorRevision)
{
	prv_read_sensor_reg(OV9650_PID, pCmRevision);
	prv_read_sensor_reg(OV9650_VER, pSensorRevision);
	return 0;
}

void ov9650_auto_function_on(void)
{
	u8 val;
	DPRINTK("in function %s\n", __FUNCTION__);
	prv_read_sensor_reg(OV9650_COM8, &val);
	val |= 0x07;
	prv_write_sensor_reg(OV9650_COM8, &val);
}

void ov9650_auto_function_off(void)
{
	u8 val;
	DPRINTK("in function %s\n", __FUNCTION__);
	prv_read_sensor_reg(OV9650_COM8, &val);
	val &= ~0x07;
	prv_write_sensor_reg(OV9650_COM8, &val);
}


/***********************************************************************
*  Viewfinder, still 
***********************************************************************/
int ov9650_viewfinder_on(void)
{
	u8 com3;

    ov9650_print_all_regs();

	prv_read_sensor_reg(OV9650_COM3, &com3);
	com3 &= ~0x01;
	prv_write_sensor_reg(OV9650_COM3, &com3);

	return OV_ERR_NONE;
}


int ov9650_viewfinder_off(void)
{
	u8 com3;

	prv_read_sensor_reg(OV9650_COM3, &com3);
	com3 |= 0x01;
	prv_write_sensor_reg(OV9650_COM3, &com3);

    ov9650_print_all_regs();

	mdelay(200);
	return OV_ERR_NONE;
}


static u8 *ov9650_get_regs_list(u32 captureSizeFormat, u32 colorFormat)
{
	char *formatNameP=NULL;
	u8 *defaultDataP=NULL;
	ov9650 *pov;

	pov = &g_ov9650;

	// Get the default setting.
	if (colorFormat == OV_FORMAT_YUV_422) {
		switch (captureSizeFormat) {
		case OV_SIZE_QQVGA:
			defaultDataP = (u8 *)gYUV_QQVGA;
			formatNameP = "QQVGA.422";
			break;
		case OV_SIZE_QQCIF:
			defaultDataP = (u8 *)gYUV_QQCIF;
			formatNameP = "QQCIF.422";
			break;
		case OV_SIZE_QVGA:
			defaultDataP = (u8 *)gYUV_QVGA;
			formatNameP = "QVGA.422";
			break;
		case OV_SIZE_QCIF:
			defaultDataP = (u8 *)gYUV_QCIF;
			formatNameP = "QCIF.422";
			break;
		case OV_SIZE_VGA:
			defaultDataP = (u8 *)gYUV_VGA;
			formatNameP = "VGA.422";
			break;
		case OV_SIZE_CIF:
			defaultDataP = (u8 *)gYUV_CIF;
			formatNameP = "CIF.422";
			break;
		case OV_SIZE_SXGA:
			defaultDataP = (u8 *)gYUV_SXGA;
			formatNameP = "SXGA.422";
			break;
		default:
			return NULL;
		}
	}

	if (colorFormat == OV_FORMAT_RGB_565) {
		switch (captureSizeFormat) {
		case OV_SIZE_QQVGA:
			defaultDataP = (u8 *)gRGB_QQVGA;
			formatNameP = "QQVGA.RGB";
			break;
		case OV_SIZE_QCIF:
			defaultDataP = (u8 *)gRGB_QCIF;
			formatNameP = "QCIF.RGB";
			break;

		case OV_SIZE_QVGA:
			defaultDataP = (u8 *)gRGB_QVGA;
			formatNameP = "QVGA.RGB";
			break;
		case OV_SIZE_VGA:
			defaultDataP = (u8 *)gRGB_VGA;
			formatNameP = "VGA.RGB";
			break;
		case OV_SIZE_CIF:
			defaultDataP = (u8 *)gRGB_CIF;
			formatNameP = "CIF.RGB";
			break;
		case OV_SIZE_SXGA:
			defaultDataP = (u8 *)gRGB_SXGA;
			formatNameP = "SXGA.RGB";
			break;
		default:
			return NULL;
		}
	}
    dbg_print("%s", formatNameP);
	return defaultDataP;
}

/***********************************************************************
*  Format 
***********************************************************************/
static int ov9650_switch_format(u32 captureSizeFormat, u32 colorFormat)
{
	int status;
	u8 *regsP;
	ov9650 *pov;

	pov = &g_ov9650;

	regsP = (u8 *) ov9650_get_regs_list(captureSizeFormat, colorFormat);
	// Get the pointer to the basic setting.  The pointer must be freed after exiting.

	if (regsP == NULL)
		return OV_ERR_PARAMETER;

    status = ov9650_set_regs(regsP);
	mdelay(50);
	if (pov->night_mode == 1) {
		//night mode 
		ov9650_set_night_mode();
	}
	else if (pov->night_mode == 2) {	//action mode
		ov9650_set_action_mode();
	}
	else
		ov9650_set_auto_mode();

	return status;
}

int ov9650_set_format(u32 captureSizeFormat, u32 colorFormat)
{
	int status = OV_ERR_PARAMETER;
	u8 *regsP;
	static u32 prev_colorFormat = OV_FORMAT_NONE, prev_size = OV_SIZE_NONE;

	if (prev_colorFormat == colorFormat && captureSizeFormat == prev_size)
		return 0;
	if ((captureSizeFormat == OV_SIZE_NONE) && (colorFormat == OV_FORMAT_NONE))
		goto no_set;

	if ((prev_colorFormat == OV_FORMAT_NONE) && (prev_size == OV_SIZE_NONE)) {
		regsP = (u8 *) ov9650_get_regs_list(captureSizeFormat, colorFormat);
		// Get the pointer to the basic setting.  The pointer must be freed after exiting.

		if (regsP == NULL)
			return OV_ERR_PARAMETER;
		//ov9650_soft_reset();
		// Blast the entire parameter tree into the part.
		status = ov9650_set_regs(regsP);
	}
	else {
		status = ov9650_switch_format(captureSizeFormat, colorFormat);
	}
no_set:
	prev_colorFormat = colorFormat;
	prev_size = captureSizeFormat;
	return status;
}

void ov9650_save_gains(void)
{
	u8 gainh, gainl, aech, aechm, aecl, laec, blue, red;
	u32 gain, gaina, exp_value;
	ov9650 *pov;
	u8 regValue;
	static u32 prevSize;

	pov = &g_ov9650;

	// Get current size
	prv_read_sensor_reg(OV9650_COM7, &regValue);
	switch (regValue) {
	case 0x00:
		prevSize = OV_SIZE_SXGA;
		break;
	case 0x08:
		prv_read_sensor_reg(OV9650_COM1, &regValue);
		if (regValue & 0x20)
			prevSize = OV_SIZE_QQCIF;
		else
			prevSize = OV_SIZE_QCIF;
		break;
	case 0x10:
		prv_read_sensor_reg(OV9650_COM1, &regValue);
		if (regValue & 0x20)
			prevSize = OV_SIZE_QQVGA;
		else
			prevSize = OV_SIZE_QVGA;
		break;
	case 0x20:
		prevSize = OV_SIZE_CIF;
		break;
	case 0x40:
		prevSize = OV_SIZE_VGA;
		break;
	default:
		prevSize = OV_SIZE_SXGA;
		break;
	}
	pov->pre_size = prevSize;

	// Get the awb, gain, exposure values
	prv_read_sensor_reg(OV9650_BLUE, &blue);
	prv_read_sensor_reg(OV9650_RED, &red);
	prv_read_sensor_reg(OV9650_GAIN, &gainl);
	prv_read_sensor_reg(OV9650_VREF, &gainh);
	gain = gainl + ((gainh>>6)&0x03);
	prv_read_sensor_reg(OV9650_AECHM, &aechm);
	prv_read_sensor_reg(OV9650_AECH, &aech);
	prv_read_sensor_reg(OV9650_COM1, &aecl);
	exp_value = (((aechm&0x3f)<<10) + (aech << 2)) | (aecl&0x03);
	prv_read_sensor_reg(OV9650_LAEC, &laec);

	pov->gain = gain;
	pov->blue_gain = blue;
	pov->red_gain = red;
	pov->exp_value = exp_value;
	pov->exp_time = laec;

    //TODO:
    gaina = (gainl & 0x0f) + 16;
    if(gainl&0x20)
        gaina *= 2;
    if(gainl&0x10)
        gaina *= 2;
	pov->gaina = gaina;

	DPRINTK("gain=%x gaina=%x blue=%x red=%x, expv=%x expt=%x\n", 
             gain, gaina, blue, red, exp_value, laec);
}

static void ov9650_adjust_gains(u32 prev_size, u32 cur_size)
{
	ov9650 *pov;

	pov = &g_ov9650;
	if (prev_size == OV_SIZE_QVGA) {
		if (cur_size == OV_SIZE_VGA) {
			{
				/* capture 15 fps */
				pov->pclock = 0x81;
				pov->adjusted_exp_value = pov->exp_value * 2;
			}
			if (pov->night_mode == 1) {
				//night mode 
				//pov->pclock = 0x87;
			}
			else if (pov->night_mode == 2) {	//action mode
				pov->pclock = 0x80;
			}
		}
		else if (cur_size == OV_SIZE_SXGA) {
            int adjexp = 0;
			adjexp = pov->exp_value * 2;
#if 0
			if (pov->gain & 0x80) {
				/* 8x gain capture 7.5 fps to 4x gain */
                if(adjexp<500)
                {
				    //pov->pclock = 0x80;
			        adjexp = adjexp * 2;
			        pov->gain = pov->gain & ~0x80;
                }
			}
			if (pov->gain & 0x40) {
				/* 4x gain capture 7.5 fps to 2x gain */
                if(adjexp<500)
                {
				    //pov->pclock = 0x80;
			        adjexp = adjexp * 2;
				    pov->gain = pov->gain & ~0x40;
                }
			}
			if (pov->gain & 0x20) {
                if(adjexp<500)
                {
				    //pov->pclock = 0x80;
			        adjexp = adjexp * 2;
				    pov->gain = pov->gain & ~0x20;
                }
			}
			if (pov->gain & 0x10) {
                if(adjexp<500)
                {
				    //pov->pclock = 0x80;
			        adjexp = adjexp * 2;
				    pov->gain = pov->gain & ~0x10;
                }
			}
#endif
			if (pov->night_mode == 1) {
				//night mode 
				//pov->pclock = 0x81;
			}
			else if (pov->night_mode == 2) {	//action mode
				pov->pclock = 0x80;
			}
			pov->adjusted_exp_value = adjexp;
		}
		else if (cur_size == OV_SIZE_QVGA) {
			if (pov->gain & 0x20) {
				/* hight gain capture 7.5 fps */
				pov->pclock = 0x87;
				pov->gain = pov->gain & ~0x20;
				pov->adjusted_exp_value = pov->exp_value;
			}
			else {
				/* capture 15 fps */
				pov->pclock = 0x83;
				pov->adjusted_exp_value = pov->exp_value;
			}
			if (pov->night_mode == 1) {
				//night mode 
				//pov->pclock = 0x89;
			}
			else if (pov->night_mode == 2) {	//action mode
				pov->pclock = 0x81;
			}
		}
	}
}

static void ov9650_upload_gains(void)
{
	ov9650 *pov;
	u32 expValue, gain, gaina;
	u8 gainh, gainl, aechm, aech, aecl, laec, blue, red;
	u8 regValue;

	pov = &g_ov9650;

	gain = pov->gain;
	gaina = pov->gaina;
	blue = pov->blue_gain;
	red = pov->red_gain;
	expValue = pov->adjusted_exp_value;
	laec = pov->exp_time;
	// Set awb
	prv_write_sensor_reg(OV9650_BLUE, &blue);
	prv_write_sensor_reg(OV9650_RED, &red);

	// Set gain 
    gaina = (gaina*5)/7;
    gain = 0;
    if(gaina>62)
    {
        gain |= 0x20;
        gaina /= 2;
    }
    if(gaina>31)
    {
        gain |= 0x10;
        gaina /= 2;
    }
    if( gaina>16 )
        gaina -= 16;
    else
        gaina = 0;
    gain = gain | gaina;

    gainl = gain & 0xff;
    gainh = (gain&0x0300)>>2;
	prv_write_sensor_reg(OV9650_GAIN, &gainl);
	prv_read_sensor_reg(OV9650_VREF, &regValue);
	regValue = (regValue & 0x3f) | gainh;
	prv_write_sensor_reg(OV9650_VREF, &regValue);
	// Set exposure
        aechm = (expValue>>10)&0x3f;
        aech  = (expValue>>2)&0xff;
        aecl  = expValue&0x03;
	prv_read_sensor_reg(OV9650_AECHM, &regValue);
	regValue = (regValue & 0xc0) | aechm;
	prv_write_sensor_reg(OV9650_AECHM, &regValue);
	prv_write_sensor_reg(OV9650_AECH, &aech);
	prv_read_sensor_reg(OV9650_COM1, &regValue);
	regValue = (regValue & 0xfc) | aecl;
	prv_write_sensor_reg(OV9650_COM1, &regValue);

	//prv_write_sensor_reg(OV9650_CLKRC, &pov->pclock);
	DPRINTK("gain=%x gaina=%x blue=%x red=%x, expv=%x, pclock=%x\n", 
                gain, gaina, blue, red, expValue, pov->pclock);
}

static int
ov9650_set_dma_pages(pxa_dma_desc ** pdes,
		     pxa_dma_desc ** des_physical, int num, struct page *array[], int total_size, int dsadr, int flags)
{
	int remain_size, des_transfer_size;
	int j, target_page_num = num;
	pxa_dma_desc *cur_des_virtual = *pdes;
	pxa_dma_desc *cur_des_physical = *des_physical;

	// in each iteration, generate one dma chain for one frame
	remain_size = total_size;

	for (j = 0; j < num; j++) {
		// set descriptor
		if (remain_size > PAGE_SIZE)
			des_transfer_size = PAGE_SIZE;
		else
			des_transfer_size = remain_size;
		cur_des_virtual->ddadr = (unsigned) cur_des_physical + sizeof(pxa_dma_desc);
		cur_des_virtual->dsadr = dsadr;	// FIFO0 physical address
		cur_des_virtual->dtadr = page_to_bus(array[j]);
		cur_des_virtual->dcmd = des_transfer_size | flags;

		// advance pointers
		remain_size -= des_transfer_size;
		cur_des_virtual++;
		cur_des_physical++;
		target_page_num++;
	}
	*pdes = cur_des_virtual;
	*des_physical = cur_des_physical;
    return 0;
}

static int
ov9650_set_dma_page1(pxa_dma_desc ** pdes,
		     pxa_dma_desc ** des_physical, int num, struct page *page1, int total_size, int dsadr, int flags)
{
	int remain_size, des_transfer_size;
	int j, target_page_num = num;
	pxa_dma_desc *cur_des_virtual = *pdes;
	pxa_dma_desc *cur_des_physical = *des_physical;
	int dump_page;

	// in each iteration, generate one dma chain for one frame
	remain_size = total_size;
	dump_page = page_to_bus(page1);
	DPRINTK("dump_page=%x", dump_page);

	for (j = 0; j < num; j++) {
		// set descriptor
		if (remain_size > PAGE_SIZE)
			des_transfer_size = PAGE_SIZE;
		else
			des_transfer_size = remain_size;
		cur_des_virtual->ddadr = (unsigned) cur_des_physical + sizeof(pxa_dma_desc);
		cur_des_virtual->dsadr = dsadr;	// FIFO0 physical address
		cur_des_virtual->dtadr = dump_page;
		cur_des_virtual->dcmd = des_transfer_size | flags;

		// advance pointers
		remain_size -= des_transfer_size;
		cur_des_virtual++;
		cur_des_physical++;
		target_page_num++;
	}
	*pdes = cur_des_virtual;
	*des_physical = cur_des_physical;
    return 0;
}

static int ov9650_update_still_dma_chain(p_camera_context_t cam_ctx)
{
	pxa_dma_desc *cur_des_virtual, *cur_des_physical, *last_des_virtual = NULL;
	int remain_size;
	unsigned int i;

	int target_page_num;

	DPRINTK("ov9650_update_still_dma_chain\n");
	// clear descriptor pointers
	cam_ctx->fifo0_descriptors_virtual = cam_ctx->fifo0_descriptors_physical = 0;
	cam_ctx->fifo1_descriptors_virtual = cam_ctx->fifo1_descriptors_physical = 0;
	cam_ctx->fifo2_descriptors_virtual = cam_ctx->fifo2_descriptors_physical = 0;

	// calculate how many descriptors are needed per frame
	cam_ctx->fifo0_num_descriptors = cam_ctx->pages_per_fifo0;

	cam_ctx->fifo1_num_descriptors = cam_ctx->pages_per_fifo1;

	cam_ctx->fifo2_num_descriptors = cam_ctx->pages_per_fifo2;

	// check if enough memory to generate descriptors
	DPRINTK("in %s, cam_ctx->block_number =%d\n", __FUNCTION__, cam_ctx->block_number);
	if ((cam_ctx->fifo0_num_descriptors + cam_ctx->fifo1_num_descriptors +
	     cam_ctx->fifo2_num_descriptors) * cam_ctx->block_number > cam_ctx->dma_descriptors_size)
		return -1;

	// generate fifo0 dma chains
	cam_ctx->fifo0_descriptors_virtual = (unsigned) cam_ctx->dma_descriptors_virtual;
	cam_ctx->fifo0_descriptors_physical = (unsigned) cam_ctx->dma_descriptors_physical;
	cur_des_virtual = (pxa_dma_desc *) cam_ctx->fifo0_descriptors_virtual;
	cur_des_physical = (pxa_dma_desc *) cam_ctx->fifo0_descriptors_physical;

	DPRINTK("pages_allocated=%d,fifo0_descriptors_virtual=%p\n", cam_ctx->pages_allocated, cur_des_virtual);

	for (i = 0; i < 2; i++) {
		// in each iteration, generate one dma chain for one frame
		remain_size = cam_ctx->fifo0_transfer_size;
		ov9650_set_dma_page1(&cur_des_virtual, &cur_des_physical,
				     cam_ctx->fifo0_num_descriptors,
				     cam_ctx->page_array[cam_ctx->
							 pages_allocated -
							 1], remain_size, CIBR0_PHY, DCMD_FLOWSRC | DCMD_BURST32);

	}
	DPRINTK("after ov9650_set_dma_page1=%d\n", cam_ctx->pages_allocated);
	for (i = 0; i < 1; i++) {
		// in each iteration, generate one dma chain for one frame
		remain_size = cam_ctx->fifo0_transfer_size;

		// assume the blocks are stored consecutively
		target_page_num = cam_ctx->pages_per_block * i;
		DPRINTK("target_page_num=%d\n", target_page_num);
		ov9650_set_dma_pages(&cur_des_virtual, &cur_des_physical,
				     cam_ctx->fifo0_num_descriptors,
				     &cam_ctx->page_array[target_page_num],
				     remain_size, CIBR0_PHY, DCMD_FLOWSRC | DCMD_BURST32 | DCMD_INCTRGADDR);

		// stop the dma transfer on one frame captured
		last_des_virtual = cur_des_virtual - 1;
	}
	last_des_virtual->ddadr = ((unsigned) cam_ctx->fifo0_descriptors_physical);
	last_des_virtual->ddadr |= 0x1;
	last_des_virtual->dcmd |= DCMD_ENDIRQEN;

	// generate fifo1 dma chains
	if (cam_ctx->fifo1_transfer_size) {
		// record fifo1 descriptors' start address
		cam_ctx->fifo1_descriptors_virtual = (unsigned) cur_des_virtual;
		cam_ctx->fifo1_descriptors_physical = (unsigned) cur_des_physical;

		for (i = 0; i < 2; i++) {
			// in each iteration, generate one dma chain for one frame
			remain_size = cam_ctx->fifo1_transfer_size;
			ov9650_set_dma_page1(&cur_des_virtual,
					     &cur_des_physical,
					     cam_ctx->
					     fifo1_num_descriptors,
					     cam_ctx->page_array[cam_ctx->
								 pages_allocated
								 - 2],
					     remain_size, CIBR1_PHY, DCMD_FLOWSRC | DCMD_BURST32);

			// stop the dma transfer on one frame captured
			last_des_virtual = cur_des_virtual - 1;
			//last_des_virtual->ddadr |= 0x1;
		}
		for (i = 0; i < 1; i++) {
			// in each iteration, generate one dma chain for one frame
			remain_size = cam_ctx->fifo1_transfer_size;

			target_page_num = cam_ctx->pages_per_block * i + cam_ctx->pages_per_fifo0;
			ov9650_set_dma_pages(&cur_des_virtual, &cur_des_physical,
					     cam_ctx->fifo1_num_descriptors,
					     &cam_ctx->page_array[target_page_num],
					     remain_size, CIBR1_PHY, DCMD_FLOWSRC | DCMD_BURST32 | DCMD_INCTRGADDR);

			// stop the dma transfer on one frame captured
			last_des_virtual = cur_des_virtual - 1;
		}
		last_des_virtual->ddadr = ((unsigned) cam_ctx->fifo1_descriptors_physical);
		last_des_virtual->ddadr |= 0x1;
	}
	// generate fifo2 dma chains
	if (cam_ctx->fifo2_transfer_size) {
		// record fifo1 descriptors' start address
		cam_ctx->fifo2_descriptors_virtual = (unsigned) cur_des_virtual;
		cam_ctx->fifo2_descriptors_physical = (unsigned) cur_des_physical;

		for (i = 0; i < 2; i++) {
			// in each iteration, generate one dma chain for one frame
			remain_size = cam_ctx->fifo2_transfer_size;
			ov9650_set_dma_page1(&cur_des_virtual,
					     &cur_des_physical,
					     cam_ctx->
					     fifo2_num_descriptors,
					     cam_ctx->page_array[cam_ctx->
								 pages_allocated
								 - 3],
					     remain_size, CIBR2_PHY, DCMD_FLOWSRC | DCMD_BURST32);

			// stop the dma transfer on one frame captured
			last_des_virtual = cur_des_virtual - 1;
			//last_des_virtual->ddadr |= 0x1;
		}
		DPRINTK("last target_page_num=%d\n", target_page_num + cam_ctx->fifo2_num_descriptors);
		for (i = 0; i < 1; i++) {
			// in each iteration, generate one dma chain for one frame
			remain_size = cam_ctx->fifo2_transfer_size;
			target_page_num =
			    cam_ctx->pages_per_block * i + cam_ctx->pages_per_fifo0 + cam_ctx->pages_per_fifo1;
			ov9650_set_dma_pages(&cur_des_virtual, &cur_des_physical,
					     cam_ctx->fifo2_num_descriptors,
					     &cam_ctx->page_array[target_page_num],
					     remain_size, CIBR2_PHY, DCMD_FLOWSRC | DCMD_BURST32 | DCMD_INCTRGADDR);

			// stop the dma transfer on one frame captured
			last_des_virtual = cur_des_virtual - 1;
			DPRINTK("last target_page_num=%d\n", target_page_num + cam_ctx->fifo2_num_descriptors);
		}
		last_des_virtual->ddadr = ((unsigned) cam_ctx->fifo2_descriptors_physical);
		last_des_virtual->ddadr |= 0x1;
	}
	return 0;
}

static int ov9650_stop_third_des(p_camera_context_t cam_ctx)
{
	pxa_dma_desc *pdesc, *ptmp;
	int i;

	// stop the dma transfer on one frame captured
	pdesc = (pxa_dma_desc *) (cam_ctx->fifo0_descriptors_virtual);
	ptmp = pdesc + cam_ctx->fifo0_num_descriptors * 2;
	for (i = 0; i < cam_ctx->fifo0_num_descriptors; i++)
		(ptmp + i)->dtadr = (pdesc + i)->dtadr;

	pdesc += cam_ctx->fifo0_num_descriptors - 1;
	pdesc->dcmd = (pdesc->dcmd & DCMD_LENGTH) | DCMD_FLOWSRC | DCMD_BURST32 | DCMD_INCTRGADDR;
	pdesc += cam_ctx->fifo0_num_descriptors;
	pdesc->dcmd = (pdesc->dcmd & DCMD_LENGTH) | DCMD_FLOWSRC | DCMD_BURST32 | DCMD_INCTRGADDR;
	pdesc += cam_ctx->fifo0_num_descriptors;
	pdesc->ddadr |= 0x1;

	pdesc = (pxa_dma_desc *) (cam_ctx->fifo1_descriptors_virtual);
	ptmp = pdesc + cam_ctx->fifo1_num_descriptors * 2;
	for (i = 0; i < cam_ctx->fifo1_num_descriptors; i++)
		(ptmp + i)->dtadr = (pdesc + i)->dtadr;
	pdesc += cam_ctx->fifo1_num_descriptors * 3 - 1;
	pdesc->ddadr |= 0x1;

	pdesc = (pxa_dma_desc *) (cam_ctx->fifo2_descriptors_virtual);
	ptmp = pdesc + cam_ctx->fifo2_num_descriptors * 2;
	for (i = 0; i < cam_ctx->fifo2_num_descriptors; i++)
		(ptmp + i)->dtadr = (pdesc + i)->dtadr;
	pdesc += cam_ctx->fifo2_num_descriptors * 3 - 1;
	pdesc->ddadr |= 0x1;
    return 0;
}

int ov9650_prepare_capture(p_camera_context_t cam_ctx, u32 captureSizeFormat, u32 colorFormat)
{
	ov9650 *pov;

	pov = &g_ov9650;

	stop_dma_transfer(cam_ctx);
	ov9650_switch_format(captureSizeFormat, colorFormat);
	ov9650_adjust_gains(pov->pre_size, captureSizeFormat);
	ov9650_upload_gains();
	if (captureSizeFormat == OV_SIZE_VGA) {
		//ov9650_update_still_dma_chain(cam_ctx);
		ov9650_stop_third_des(cam_ctx);
		/*
		 */
	}
	else if (captureSizeFormat == OV_SIZE_SXGA) {
		camera_set_int_mask(cam_ctx, 0x3fd | 0x0400);
		ov9650_update_still_dma_chain(cam_ctx);
		//ci_clear_int_status(0xFFFFFFFF);
		//DPRINTK("before camera_sleep \n");
		//camera_sleep();
	}
	ci_reset_fifo();
	ci_clear_int_status(0xFFFFFFFF);
	start_dma_transfer(cam_ctx, 1);
	DPRINTK("after ov9650_prepare_capture \n");
    return 0;
}

int ov9650_set_special_effect(int style)
{
#if 0
	int ret = 0;
	u32 index, curReg;
	u8 *regsP;
	u8 regValue;

	DPRINTK("in function %s parameter=%d\n", __FUNCTION__, style);
	curReg = 0x3a;
	ov9650_read(0x3a, &regValue);
	regValue = regValue & 0xf;
	switch (style) {
	case V4l_STYLE_NORMAL:
		DPRINTK("V4l_STYLE_NORMAL\n");
		regValue &= 0x7f;
		//ov9650_write(0x3a, 0x08);
		ov9650_write(0x3a, regValue);
		ov9650_write(0x67, 0x80);
		ov9650_write(0x68, 0x80);
		break;
	case V4l_STYLE_BLACK_WHITE:
		DPRINTK("V4l_STYLE_BLACK_WHITE\n");
		regValue |= 0x10;
		//ov9650_write(0x3a, 0x18);
		ov9650_write(0x3a, regValue);
		ov9650_write(0x67, 0x80);
		ov9650_write(0x68, 0x80);
		break;
	case V4l_STYLE_SEPIA:
		DPRINTK("V4l_STYLE_SEPIA\n");
		regValue |= 0x10;
		//ov9650_write(0x3a, 0x18);
		ov9650_write(0x3a, regValue);
		ov9650_write(0x67, 0xa0);
		ov9650_write(0x68, 0x40);
		break;
		//case V4l_STYLE_BULISH:
		DPRINTK("V4l_STYLE_BULISH\n");
		ov9650_write(0x3a, 0x18);
		ov9650_write(0x67, 0x80);
		ov9650_write(0x68, 0xc0);
		break;
	default:
		DPRINTK("case default ????????????????????\n");
		//ret=OV_ERR_PARAMETER;
	}
	return ret;
#else
	return 0;
#endif
}

int ov9650_set_brightness(int bright)
{
#if 0
	int ret = 0;
	const u8 BN[] = {
		//BN-3
		0x0f, 0x4f,
		0x27, 0xe8,
		0x28, 0xe0,
		0x29, 0xe0,
		0x2c, 0xe0,
		//BN-2
		0x0f, 0x4f,
		0x27, 0xc8,
		0x28, 0xc0,
		0x29, 0xc0,
		0x2c, 0xc0,
		//BN-1
		0x0f, 0x4f,
		0x27, 0xa8,
		0x28, 0xa0,
		0x29, 0xa0,
		0x2c, 0xa0,
		//BN-0
		0x0f, 0x4f,
		0x27, 0x88,
		0x28, 0x80,
		0x29, 0x80,
		0x2c, 0x80,
		//BN+1
		0x0f, 0x4f,
		0x27, 0x28,
		0x28, 0x20,
		0x29, 0x20,
		0x2c, 0x20,
		//BN+2
		0x0f, 0x4f,
		0x27, 0x48,
		0x28, 0x40,
		0x29, 0x40,
		0x2c, 0x40,
		//BN+3
		0x0f, 0x4f,
		0x27, 0x68,
		0x28, 0x60,
		0x29, 0x60,
		0x2c, 0x60,
	};
	u8 *regs;
	int n = 5;
	int i;

	DPRINTK("in function %s bright =%d \n", __FUNCTION__, bright);
	if (bright < -3)
		bright = -3;
	if (bright > 3)
		bright = 3;
	//bright = -4 .. 4
	regs = &BN[(bright + 3) * n * 2];
	//for (i = 0; i < n * 2; i += 2)
	//	ret |= ov9650_write(regs[i], regs[i + 1]);
#endif
	return OV_ERR_NONE;
}

#if 0
static int ov9650_set_color_saturation(int saturation)
{
	const u8 CS[] = {
		//Saturation: 0.25
		0x4f, 0x14,
		0x50, 0x10,
		0x51, 0x3,
		0x52, 0x6,
		0x53, 0x13,
		0x54, 0x19,
		//Saturation 0.5
		0x4f, 0x28,
		0x50, 0x22,
		0x51, 0x6,
		0x52, 0xc,
		0x53, 0x26,
		0x54, 0x33,
		//Saturation 0.75 (Default)
		0x4f, 0x3c,
		0x50, 0x32,
		0x51, 0x9,
		0x52, 0x13,
		0x53, 0x39,
		0x54, 0x4c,
		//Saturation 1.0
		0x4f, 0x50,
		0x50, 0x43,
		0x51, 0xd,
		0x52, 0x19,
		0x53, 0x4d,
		0x54, 0x66,
		//Saturation 1.25
		0x4f, 0x64,
		0x50, 0x53,
		0x51, 0x10,
		0x52, 0x1f,
		0x53, 0x5f,
		0x54, 0x7f,
	};
	u8 *regs;
	int n = 6;
	int i;

	DPRINTK("in function %s ,parameter=%d\n", __FUNCTION__, saturation);
	if (saturation < 0)
		saturation = 0;
	if (saturation > 4)
		saturation = 4;

	regs = &CS[saturation * n * 2];
	for (i = 0; i < n * 2; i += 2)
		ov9650_write(regs[i], regs[i + 1]);
	return OV_ERR_NONE;

}
#endif

int ov9650_set_white_balance(V4l_PIC_WB light)
{
#if 0
	int ret = 0;
	u8 *regs;
	int n = 13;
	int i = 0;
	ov9650 *pov;

	pov = &g_ov9650;

	DPRINTK("in function %s ,parameter=%d\n", __FUNCTION__, light);
	switch (light) {
	case V4l_WB_DIRECT_SUN:
		//ov9650_set_color_saturation(3);
		DPRINTK("V4l_WB_DIRECT_SUN\n");
		i = 0 * n * 2;
		//ov9650_set_night_mode();
		break;
	case V4l_WB_INCANDESCENT:
		//ov9650_set_color_saturation(2);
		DPRINTK("V4l_WB_INCANDESCENT\n");
		i = 1 * n * 2;
		//ov9650_set_action_mode();
		break;
	case V4l_WB_FLUORESCENT:
		//ov9650_set_color_saturation(2);
		DPRINTK("V4l_WB_FLUORESCENT\n");
		i = 2 * n * 2;
		break;
	default:
		/* auto */
		//ov9650_set_color_saturation(2);
		DPRINTK("case default ????????????????????\n");
		i = 3 * n * 2;
		n = 19;
		//ov9650_set_auto_mode();
		break;
	}
    //TODO:
#endif

	return OV_ERR_NONE;
}

int ov9650_set_night_mode()
{
#if 0
	ov9650 *pov;

	pov = &g_ov9650;
	pov->night_mode = 1;
	//ov9650_write(0x11, 0x89);
	ov9650_write(0x3b, 0xe1);
#endif
    return 0;
}

int ov9650_set_action_mode()
{
#if 0
	ov9650 *pov;

	pov = &g_ov9650;
	pov->night_mode = 2;
	ov9650_write(0x11, 0x81);
	ov9650_write(0x3b, 0x01);
	ov9650_write(0x2d, 0x0);
	ov9650_write(0x2e, 0x0);
#endif
    return 0;
}

int ov9650_set_auto_mode()
{
#if 0
	ov9650 *pov;

	pov = &g_ov9650;
	pov->night_mode = 0;
	ov9650_write(0x11, 0x83);
	ov9650_write(0x3b, 0x01);
	ov9650_write(0x2d, 0x0);
	ov9650_write(0x2e, 0x0);
#endif
    return 0;
}

int ov9650_set_flicker(int freq)
{
#if 0
	int ret = 0;
	const u8 LE_v3[] = {
		//OutDoor
		0x3b, 0x00,
		0x2a, 0,
		0x2b, 0,
		0x6a, 0x3d,
		//Indoor 60Hz(Default)
		0x3b, 0x01,
		0x2a, 0,
		0x2b, 0,
		0x6a, 0x3d,
		//Indoor 50Hz
		0x3b, 0x01,
		0x2a, 0x10,
		0x2b, 0x40,
		0x6a, 0x3d,
	};
	const u8 LE_v2[] = {
		//OutDoor
		0x13, 0x8d,
		0x2a, 0,
		0x2b, 0,
		0x6a, 0x3d,
		//Indoor 60Hz(Default)
		0x13, 0x8d,
		0x2a, 0,
		0x2b, 0,
		0x6a, 0x3d,
		//Indoor 50Hz
		0x13, 0x8d,
		0x2a, 0x10,
		0x2b, 0x14,
		0x6a, 0x3d,
	};
	u8 *regs;
	int n = 4;
	int i = 0;
	ov9650 *pov;

	pov = &g_ov9650;
	DPRINTK("in function %s ,parameter=%d\n", __FUNCTION__, freq);
	switch (freq) {
	case 0:
		i = 0 * n * 2;
		break;
	case 60:
		i = 1 * n * 2;
		break;
	case 50:
		i = 2 * n * 2;
		break;
	default:
		i = 0;
		break;
	}

	//regs = &LE_v2[i];
	regs = &LE_v3[i];


	for (i = 0; i < n * 2; i += 2)
		ret |= ov9650_write(regs[i], regs[i + 1]);
#endif
	return OV_ERR_NONE;
}

int ov9650_set_contrast(int contrast)
{
#if 0
	const u8 CO[] = {
		//Low contrast
		0x6C, 0x20,
		0x6D, 0x50,
		0x6E, 0xc0,
		0x6F, 0xa8,
		0x70, 0x88,
		0x71, 0x80,
		0x72, 0x78,
		0x73, 0x70,
		0x74, 0x68,
		0x75, 0x58,
		0x76, 0x40,
		0x77, 0x30,
		0x78, 0x28,
		0x79, 0x20,
		0x7A, 0x1e,
		0x7B, 0x18,
		0x7C, 0x04,
		0x7D, 0x07,
		0x7E, 0x1f,
		0x7F, 0x49,
		0x80, 0x5a,
		0x81, 0x6a,
		0x82, 0x79,
		0x83, 0x87,
		0x84, 0x94,
		0x85, 0x9f,
		0x86, 0xaf,
		0x87, 0xbb,
		0x88, 0xcf,
		0x89, 0xdf,
		0x8A, 0xee,
		//Middle contrast (default)
		0x6C, 0x40,
		0x6D, 0x30,
		0x6E, 0x4B,
		0x6F, 0x60,
		0x70, 0x70,
		0x71, 0x70,
		0x72, 0x70,
		0x73, 0x70,
		0x74, 0x60,
		0x75, 0x60,
		0x76, 0x50,
		0x77, 0x48,
		0x78, 0x3A,
		0x79, 0x2E,
		0x7A, 0x28,
		0x7B, 0x22,
		0x7C, 0x04,
		0x7D, 0x07,
		0x7E, 0x10,
		0x7F, 0x28,
		0x80, 0x36,
		0x81, 0x44,
		0x82, 0x52,
		0x83, 0x60,
		0x84, 0x6C,
		0x85, 0x78,
		0x86, 0x8C,
		0x87, 0x9E,
		0x88, 0xBB,
		0x89, 0xD2,
		0x8A, 0xE6,
		//High contrast
		0x6c, 0x50,
		0x6d, 0x60,
		0x6e, 0x58,
		0x6f, 0x58,
		0x70, 0x58,
		0x71, 0x50,
		0x72, 0x50,
		0x73, 0x50,
		0x74, 0x50,
		0x75, 0x50,
		0x76, 0x4c,
		0x77, 0x4c,
		0x78, 0x45,
		0x79, 0x3c,
		0x7a, 0x2c,
		0x7b, 0x24,
		0x7c, 0x05,
		0x7d, 0x0b,
		0x7e, 0x16,
		0x7f, 0x2c,
		0x80, 0x37,
		0x81, 0x41,
		0x82, 0x4b,
		0x83, 0x55,
		0x84, 0x5f,
		0x85, 0x69,
		0x86, 0x7c,
		0x87, 0x8f,
		0x88, 0xb1,
		0x89, 0xcf,
		0x8a, 0xe5,
	};

	u8 *regs;
	int n = 31;
	int i;

	DPRINTK("in function %s parameter=%d \n", __FUNCTION__, contrast);
	if (contrast < 0)
		contrast = 0;
	if (contrast > 2)
		contrast = 2;

	regs = &CO[contrast * n * 2];
	for (i = 0; i < n * 2; i += 2)
		ov9650_write(regs[i], regs[i + 1]);
#endif
	return OV_ERR_NONE;

}

static int ov9650_find_window(struct video_window *vw, int *sub_win)
{
	int ret = OV_SIZE_NONE;

	*sub_win = 1;
	if (vw->width > 1280 || vw->height > 1024 || vw->width < 88|| vw->height < 72) {
		ret = OV_SIZE_NONE;
	}
	else if (vw->width == 1280 && ((vw->height == 960)||(vw->height == 1024))) {
		*sub_win = 0;
		ret = OV_SIZE_SXGA;
	}
	else if (vw->width >= 640 && vw->height >= 480) {
		if (vw->width == 640 && vw->height == 480)
			*sub_win = 0;
		ret = OV_SIZE_VGA;
	}
	else if (vw->width >= 352 && vw->height >= 288) {
		if (vw->width == 352 && vw->height == 288)
			*sub_win = 0;
		ret = OV_SIZE_CIF;
	}
	else if (vw->width >= 320 && vw->height >= 240) {
		if (vw->width == 320 && vw->height == 240)
			*sub_win = 0;
		ret = OV_SIZE_QVGA;
	}
	else if (vw->width >= 176 && vw->height >= 144) {
		if (vw->width == 176 && vw->height == 144)
			*sub_win = 0;
		ret = OV_SIZE_QCIF;
	}
	else if (vw->width >= 160 && vw->height >= 120) {
		if (vw->width == 160 && vw->height == 120)
			*sub_win = 0;
		ret = OV_SIZE_QQVGA;
	}
	else if (vw->width >= 88 && vw->height >= 72) {
		if (vw->width == 88 && vw->height == 72)
			*sub_win = 0;
		ret = OV_SIZE_QQCIF;
	}
	DPRINTK("in %s,ret = %d, subwin=%d\n", __FUNCTION__, ret, *sub_win);
	return ret;
}

int ov9650_set_window(struct video_window *vw)
{
	int ret = 0;
	int x_end;
	int y_end;
	struct video_window window;
	int sub_win;
	ov9650 *pov;

	pov = &g_ov9650;
	vw->width = (vw->width + 7) & (~0x7);
	vw->height = (vw->height + 7) & (~0x7);
	vw->x = vw->y = 0;
	x_end = window.width = (vw->width + vw->x);
	y_end = window.height = (vw->height + vw->y);
	DPRINTK("in %s, vw-x =%d, vw-y=%d,vw->width=%d,vw->height=%d\n",
		__FUNCTION__, vw->x, vw->y, vw->width, vw->height);
	ret = ov9650_find_window(&window, &sub_win);
	if (ret <= OV_SIZE_NONE)
		return -1;

	ret = ov9650_set_format(ret, OV_FORMAT_YUV_422);
	if (ret < 0)
		return -1;

	pov->win = *vw;
	return ret;
}

int ov9650_get_window(struct video_window *vw)
{
	ov9650 *pov;
	pov = &g_ov9650;
	*vw = pov->win;
	return 0;
}

struct win_size {
	int width;
	int height;
};
int ov9650_set_sensor_size(void *w_size)
{
	struct win_size size;
	ov9650 *pov;
	pov = &g_ov9650;
	if (copy_from_user(&size, w_size, sizeof(struct win_size))) {
		return -EFAULT;
	}
//make it in an even of multiple of 8
	size.height = (size.height + 7) / 8 * 8;
	pov->sensor_width = size.width;
	pov->sensor_height = size.height;
	return 0;
}

int ov9650_get_sensor_size(void *w_size)
{
    return 0;
}

int ov9650_set_output_size(void *w_size)
{
	struct win_size size;
	ov9650 *pov;
	pov = &g_ov9650;
	if (copy_from_user(&size, w_size, sizeof(struct win_size))) {
		return -EFAULT;
	}
//make it in an even of multiple of 8
	size.height = (size.height + 7) / 8 * 8;
	pov->sensor_width = size.width;
	pov->sensor_height = size.height;
	return 0;
}

int ov9650_get_output_size(void *w_size)
{
    return 0;
}

#if 0
static int test_divider(int res, int fps)
{
	int max_hz = 48 * 1000000;
	int div = 1;
	int i;
	int ov_fps[5] = { 3, 7, 15, 30, 60 };
	u32 value = 320 * 240;
	/*
	   switch (prevSize) {
	   case OV_SIZE_QQCIF:
	   value = 88 * 72;
	   break;
	   case OV_SIZE_QQVGA:
	   value = 176 * 144;
	   break;
	   case OV_SIZE_QCIF:
	   value = 160 * 120;
	   break;
	   case OV_SIZE_QVGA:
	   value = 320 * 240;
	   break;
	   case OV_SIZE_CIF:
	   value = 352 * 288;
	   break;
	   case OV_SIZE_VGA:
	   value = 640 * 480;
	   break;
	   case OV_SIZE_SXGA:
	   value = 1280 * 960;
	   break;
	   }

	   while (max_hz / res / div > fps)
	   div++;
	   if (div > 64)
	   div = 64;
	   return (div - 1);

	   for (i =0;i<5;i++)
	   if ( fps < ov_fps[i] ) 
	   continue;
	 */
	if (fps == 0)
		return 0;
	if (fps > 60)
		fps = 60;
	return (div = 60 / fps - 1);
}
#endif

int ov9650_set_fps(int fps, int min_fps)
{
#if 0
	u32 res = 0;
	u8 value;
	if (fps < 0) {
		DPRINTK("in %s fps = %d divider value =%d\n", __FUNCTION__, fps, res);
		fps = 15;
	}
	res = test_divider(0, fps);
	ov9650_read(OV9650_CLKRC, &value);
	value = (value & 0xc0) | res;
	ov9650_write(OV9650_CLKRC, value);
	DPRINTK("in %s fps = %d divider value =%d\n", __FUNCTION__, fps, res);
	/*
		ov9650_set_night_mode();
		ov9650_set_action_mode();
		ov9650_set_auto_mode();
	 */
#endif
	return 0;
}

