/*
 * linux/arch/arm/mm/discontig.c
 *
 * Discontiguous memory support.
 *
 * Copyright (C) 2004-2005 Motorola - support for 16 nodes.
 * Initial code: Copyright (C) 1999-2000 Nicolas Pitre
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/bootmem.h>

#if NR_NODES != 16
#error Fix Me Please
#endif

/*
 * Our node_data structure for discontiguous memory.
 */

static bootmem_data_t node_bootmem_data[NR_NODES];

pg_data_t discontig_node_data[NR_NODES] = {
  { bdata: &node_bootmem_data[0] },
  { bdata: &node_bootmem_data[1] },
  { bdata: &node_bootmem_data[2] },
  { bdata: &node_bootmem_data[3] },
  { bdata: &node_bootmem_data[4] },
  { bdata: &node_bootmem_data[5] },
  { bdata: &node_bootmem_data[6] },
  { bdata: &node_bootmem_data[7] },
  { bdata: &node_bootmem_data[8] },
  { bdata: &node_bootmem_data[9] },
  { bdata: &node_bootmem_data[10] },
  { bdata: &node_bootmem_data[11] },  
  { bdata: &node_bootmem_data[12] },
  { bdata: &node_bootmem_data[13] },
};

EXPORT_SYMBOL(discontig_node_data);

