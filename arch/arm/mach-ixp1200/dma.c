/*
 * arch/arm/kernel/dma-ixp.c
 *
 * Copyright (C) 1998 Phil Blundell
 *
 * DMA functions specific to IXP1200 architectures
 *
 * Mar-28-2000   Uday Naik       Created
 */

#include <linux/config.h>
#include <linux/sched.h>
#include <linux/malloc.h>
#include <linux/mman.h>
#include <linux/init.h>

#include <asm/page.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/hardware.h>

int ixp_request_dma(dmach_t channel, const char *dev_name)
{
  
  switch (channel) {

  case 0:
  case 1:	/* internal channels */
    return 0;

  }

  return -EINVAL;

}

void ixp_free_dma(dmach_t channel)
{
  /* nothing to do */
}

int ixp_get_dma_residue(dmach_t channel)
{

  return 0;  /* Not implemented yet */
}

void ixp_enable_dma(dmach_t channel)
{
  return;  /* Not implemented yet */
}

void ixp_disable_dma(dmach_t channel)
{
  return;  /* Not implemented yet */
}

int __init arch_dma_init(dmach_t channel)
{
  return;  /* Not implemented yet */
}



