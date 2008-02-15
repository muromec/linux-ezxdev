#ifndef __AUDIO_H__
#define __AUDIO_H__
/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2005 - Motorola
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 * Motorola 2005-Feb-28 - Rewrote the software for PCAP.
 *
 */



/*!
 * @file audio.h
 *
 * @ingroup poweric_audio
 *
 * @brief This is the header of internal definitions the power IC audio interface.
 *
 */

#include <linux/power_ic.h>

int audio_ioctl(unsigned int cmd, unsigned long arg);


#endif /* __AUDIO_H__ */
