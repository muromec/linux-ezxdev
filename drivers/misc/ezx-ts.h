/*
 * Copyright (C) 2002 Motorola Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*
 *  June, 2002 Lili Jiang, create
 */

#ifndef _EZXTS_H
#define _EZXTS_H

#define EZX_TS_MINOR_ID 14

#define PEN_UP 0
#define PEN_DOWN 0xffff

/* extern functio nfor PCAP */
void ezx_ts_touch_interrupt(int irq, void* dev_id, struct pt_regs *regs);
void ezx_ts_dataReadok_interrupt(int irq, void* dev_id, struct pt_regs *regs);

#endif //_EZXTS_H
