/*
* Driver interface to the ASIC Complasion chip on the iPAQ H3800
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
* Author:  Andrew Christian
*          <Andrew.Christian@compaq.com>
*          October 2001
*
* Restrutured June 2002
*/

#ifndef H3600_ASIC_MMC_H
#define H3600_ASIC_MMC_H

int  h3600_asic_mmc_init(void);
void h3600_asic_mmc_cleanup(void);
int  h3600_asic_mmc_suspend(void);
void h3600_asic_mmc_resume(void);

#endif /* H3600_ASIC_MMC_H */
