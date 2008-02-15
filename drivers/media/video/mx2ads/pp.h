/* drivers/media/video/mx2ads/pp.h
 *
 * MX21 PP module common definitions
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

#ifndef __MX21_ADS_EMMA_PP_H__
#define __MX21_ADS_EMMA_PP_H__

#include <linux/videodev.h>

typedef struct pp_dev_tag {
        struct  v4l2_format     in_format;
        int                     in_stride;
        struct v4l2_format      out_format;
        int                     out_stride;
        int                     qframe_width;    /* quantizer frame width */
        unsigned  int           Yptr;
        void                    *outptr;
} pp_dev_t; 


#endif /* __MX21_ADS_EMMA_PP_H__ */

