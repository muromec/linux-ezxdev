/*
 * include/asm/arch/mx2-v4l2.h
 *
 * Extern v4l2 definitions for Camera and PP module interfaces.
 *
 * Author: MontaVista Software, Inc.
 *              source@mvista.com
 *
 * 2004 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __MX2ADS_V4L2_H__
#define __MX2ADS_V4L2_H__

#define	VIDIOC_S_IN_FMT _IOW ('v', BASE_VIDIOCPRIVATE+0,  struct v4l2_format)
#define VIDIOC_G_IN_FMT _IOR ('v', BASE_VIDIOCPRIVATE+1,  struct v4l2_format)
#define VIDIOC_S_IN_PTR _IOW ('v', BASE_VIDIOCPRIVATE+2,  void*)
#define VIDIOC_S_OUT_PTR _IOW ('v', BASE_VIDIOCPRIVATE+3,  void*)
#define VIDIOC_START _IOW ('v', BASE_VIDIOCPRIVATE+4,  int)
#define VIDIOC_POLL _IOR ('v', BASE_VIDIOCPRIVATE+5,  int)
#define VIDIOC_S_OUT_STRIDE _IOW ('v', BASE_VIDIOCPRIVATE+6,  int)

#endif /* __MX2ADS_V4L2_H__ */
