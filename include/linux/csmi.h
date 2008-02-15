/*
 * include/linux/csmi.h
 *
 * Copyright (C) Jean Pihet, Texas Instruments (03/2004).
 * 
 * Redistribution of this file is permitted under the terms of the GNU 
 * Public License (GPL)
 *
 */
 
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
//
// CSMI IoCtls definitions.
//
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8

#ifndef __CSMI_API_H__
#define __CSMI_API_H__

#include <linux/types.h>
#include <asm/hardware.h>


#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

// PRINT MACROS
#define PRINT_TEST(x...)	printk(KERN_INFO x);
#define PRINT_ERROR(x...) 	printk(KERN_ERR x);

// DEBUG MACROS 
//#define DEBUG
#undef DEBUG 
#ifdef DEBUG

#define FUNC(x)          	printk(KERN_INFO "%s\n", x);  
#define ERRMSG(x)        	printk(KERN_ERR "%s(%d) : [ERROR] %s\n", __FILE__, __LINE__, x);
#define DEBUG_TEST(x...)	printk(KERN_INFO x);

#else

#define FUNC(x)
#define ERRMSG(x)
#define DEBUG_TEST(x...)

#endif


//*****************************************************************************
//
// Virtual port definitions
//
//*****************************************************************************
#define	CSMI_NB_PORTS	    9

#define GTI_VPORT_UNUSED    "/dev/csmi/0"    // CSMI_PORT_UNUSED_0
#define GTI_VPORT_CTRL      "/dev/csmi/1"    // CSMI_PORT_CTRL_AND_PWR_MANAGEMENT
#define GTI_VPORT_UNUSED_2  "/dev/csmi/2"    // CSMI_PORT_UNUSED_2
#define GTI_VPORT_UNUSED_3  "/dev/csmi/3"    // CSMI_PORT_UNUSED_3
#define GTI_VPORT_MISC      "/dev/csmi/4"    // CSMI_PORT_CTRL_MISC
#define GTI_VPORT_AT_5      "/dev/csmi/5"    // CSMI_PORT_AT_COMMANDS_5
#define GTI_VPORT_AT_6      "/dev/csmi/6"    // CSMI_PORT_AT_COMMANDS_6
#define GTI_VPORT_TR_7      "/dev/csmi/7"    // CSMI_PORT_TRACE_7
#define GTI_VPORT_PACKET_8  "/dev/csmi/8"    // CSMI_PORT_PACKET_8


//*****************************************************************************
//
// CSMI module public interface. Exposed to other kernel modules.
// For user space access, just use the regular open/read/write/ioctl/close
//  functions.
//
//*****************************************************************************
int			GTI_Init      ( void );
void			GTI_Deinit    ( void );
u32			GTI_Close     ( u32 hOpenContext );
u32			GTI_Open      ( u32 hDeviceContext, u32 AccessCode, u32 ShareMode );
int			GTI_Read      ( u32 hOpenContext, void* pBuffer, u32 Count );
int			GTI_Write     ( u32 hOpenContext, void* pSourceBytes, u32 NumberOfBytes );
int			GTI_SetupAudio( int on_off );
int			GTI_RX_Avail  ( void );

#endif //__CSMI_API_H__

