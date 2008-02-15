/*
 * Kernel panic log interface for EZX Platform.
 *
 * Copyright (C) 2005 Motorola Inc.
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
 *
 * Hisotry:
 *      2005-10-10 Roy King <e5537c@motorola.com>
 *       for ezxbase platform
 */
#ifndef _EZX_PLOG_H_
#define _EZX_PLOG_H_
#include <linux/config.h>

#define A760_LOG_NAME	"A760"
#define A760_LOG_START	0x01fc0000
#define A760_LOG_SIZE	0x00020000

#define E680_LOG_NAME	"E680"
#define E680_LOG_START	0x01fc0000
#define E680_LOG_SIZE	0x00020000

#define A780_LOG_NAME	"A780"
#define A780_LOG_START	0x01fc0000
#define A780_LOG_SIZE	0x00020000

#define BAR_LOG_NAME	"BARBADOS"
#define BAR_LOG_START	0x01dc0000
#define BAR_LOG_SIZE	0x00020000

#define MAR_LOG_NAME	"MARTINIQUE"
#define MAR_LOG_START	0x01dc0000
#define MAR_LOG_SIZE	0x00020000

#define SUM_LOG_NAME	"SUMATRA"
#define SUM_LOG_START	0x01dc0000
#define SUM_LOG_SIZE	0x00020000

#define HAI_LOG_NAME	"HAINAN"
#define HAI_LOG_START	0x019c0000
#define HAI_LOG_SIZE	0x00020000

#ifdef CONFIG_ARCH_EZX_E680
#define LOG_NAME	E680_LOG_NAME
#define LOG_START	E680_LOG_START
#define LOG_SIZE	E680_LOG_SIZE
#elif CONFIG_ARCH_EZX_A780
#define LOG_NAME	A780_LOG_NAME
#define LOG_START	A780_LOG_START
#define LOG_SIZE	A780_LOG_SIZE
#elif CONFIG_ARCH_EZXBASE

#ifdef CONFIG_ARCH_EZX_HAINAN
#define LOG_NAME	HAI_LOG_NAME
#define LOG_START	HAI_LOG_START
#define LOG_SIZE	HAI_LOG_SIZE
#elif CONFIG_ARCH_EZX_BARBADOS
#define LOG_NAME	BAR_LOG_NAME
#define LOG_START	BAR_LOG_START
#define LOG_SIZE	BAR_LOG_SIZE
#elif CONFIG_ARCH_EZX_MARTINIQUE
#define LOG_NAME	MAR_LOG_NAME
#define LOG_START	MAR_LOG_START
#define LOG_SIZE	MAR_LOG_SIZE
#elif CONFIG_ARCH_EZX_SUMATRA
#define LOG_NAME	SUM_LOG_NAME
#define LOG_START	SUM_LOG_START
#define LOG_SIZE	SUM_LOG_SIZE
#else
#define LOG_NAME	BAR_LOG_NAME
#define LOG_START	BAR_LOG_START
#define LOG_SIZE	BAR_LOG_SIZE
#endif

#else
#define LOG_NAME	A760_LOG_NAME
#define LOG_START	A760_LOG_START
#define LOG_SIZE	A760_LOG_SIZE
#endif

#ifdef CONFIG_ARCH_EZXBASE
#define PANIC_LOG_DEV 13
#else
#define PANIC_LOG_DEV 3
#endif

struct panic_log_header{
	char magic[8];
	char version[4];
	char panic_time[18];
	uint32_t panic_jiffies;
	char reserved[222];
} __attribute__((packed));
#define LOG_HEAD_LEN (sizeof(struct panic_log_header))

static ssize_t log_read(const char *buf, size_t count, loff_t *ppos);
static ssize_t log_write(const char *buf, size_t count);
extern void printout_string(void);

#endif //_EZX_PLOG_H_
