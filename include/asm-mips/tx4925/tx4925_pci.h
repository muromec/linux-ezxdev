/* $Id: tx4925_pci.h,v 1.2 2002/10/04 19:32:53 mpruznick Exp $
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2000-2001 Toshiba Corporation
 */
#ifndef TX4925_TX4925_PCI_H
#define TX4925_TX4925_PCI_H

#include <asm/tx4925/tx4925.h>

#define TX4925_NR_IRQ_LOCAL     TX4925_IRQ_PIC_BEG

#define TX4925_IR_PCIC  	20
#define TX4925_IR_PCIERR        29
#define TX4925_IR_PCIPMA        30
#define TX4925_IRQ_IRC_PCIC     (TX4925_NR_IRQ_LOCAL + TX4925_IR_PCIC)
#define TX4925_IRQ_IRC_PCIERR   (TX4925_NR_IRQ_LOCAL + TX4925_IR_PCIERR)

#ifndef _LANGUAGE_ASSEMBLY
#include <asm/byteorder.h>

struct tx4925_sdramc_reg {
	volatile unsigned long cr[4];
	volatile unsigned long unused0[4];
	volatile unsigned long tr;
	volatile unsigned long unused1[2];
	volatile unsigned long cmd;
};

struct tx4925_ebusc_reg {
       struct {
               volatile unsigned long cr;
               volatile unsigned long bar;
       } ch[8];
};                                      

struct tx4925_ccfg_reg {
	volatile unsigned long ccfg;
	volatile unsigned long revid;
	volatile unsigned long pcfg;
	volatile unsigned long toea;
	volatile unsigned long pdnctr;
	volatile unsigned long unused0;
	volatile unsigned long garbp;
	volatile unsigned long unused1;
	volatile unsigned long tocnt;
	volatile unsigned long drqctr;
	volatile unsigned long clkctr;
	volatile unsigned long garbc;
	volatile unsigned long ramp;
};

struct tx4925_irc_reg {
	volatile unsigned long irden;	/* +000 */
	volatile unsigned long irdm[2];	/* +004 */
	volatile unsigned long unused0;	/* +00c */
	volatile unsigned long irlvl[8];	/* +010 */
	volatile unsigned long unused1[4];	/* +030 */
	volatile unsigned long irmsk;	/* +040 */
	volatile unsigned long unused2[7];	/* +044 */
	volatile unsigned long iredc;	/* +060 */
	volatile unsigned long unused3[7];	/* +064 */
	volatile unsigned long irpnd;	/* +080 */
	volatile unsigned long unused4[7];	/* +084 */
	volatile unsigned long ircs;	/* +0a0 */
	volatile unsigned long unused5[27];	/* +0a4 */
	volatile unsigned long irflag[2];	/* +110 */
	volatile unsigned long irpol;	/* +118 */
	volatile unsigned long irrcnt;	/* +11c */
	volatile unsigned long irmaskint;	/* +120 */
	volatile unsigned long irmaskext;	/* +124 */
};

struct tx4925_pcic_reg {
	volatile unsigned long pciid;	/* +000 */
	volatile unsigned long pcistatus;	/* +004 */
	volatile unsigned long pciccrev;	/* +008 */
	volatile unsigned long pcicfg1;	/* +00c */
	volatile unsigned long p2gm0pbase;	/* +010 */
	volatile unsigned long p2gm1pbase;	/* +014 */
	volatile unsigned long p2gm2pbase;	/* +018 */
	volatile unsigned long p2giopbase;	/* +01c */
	volatile unsigned long unused0;	/* +020 */
	volatile unsigned long unused1;	/* +024 */
	volatile unsigned long unused2;	/* +028 */
	volatile unsigned long pcisid;	/* +02c */
	volatile unsigned long unused3;	/* +030 */
	volatile unsigned long pcicapptr;	/* +034 */
	volatile unsigned long unused4;	/* +038 */
	volatile unsigned long pcicfg2;	/* +03c */
	volatile unsigned long g2ptocnt;	/* +040 */
	volatile unsigned long unused5;	/* +044 */
	volatile unsigned long unused6;	/* +048 */
	volatile unsigned long unused7;	/* +04c */
	volatile unsigned long unused8;	/* +050 */
	volatile unsigned long unused9;	/* +054 */
	volatile unsigned long unused10;	/* +058 */
	volatile unsigned long unused11;	/* +05c */
	volatile unsigned long g2pcfg;	/* +060 */
	volatile unsigned long g2pstatus;	/* +064 */
	volatile unsigned long g2pmask;	/* +068 */
	volatile unsigned long unused12;	/* +06c */
	volatile unsigned long unused13;	/* +070 */
	volatile unsigned long unused14;	/* +074 */
	volatile unsigned long unused15;	/* +078 */
	volatile unsigned long unused16;	/* +07c */
	volatile unsigned long unused17;	/* +080 */
	volatile unsigned long unused18;	/* +084 */
	volatile unsigned long pcisstatus;	/* +088 */
	volatile unsigned long pcimask;	/* +08C */
	volatile unsigned long p2gcfg;	/* +090 */
	volatile unsigned long p2gstatus;	/* +094 */
	volatile unsigned long p2gmask;	/* +098 */
	volatile unsigned long p2gccmd;	/* +09c */
	volatile unsigned long unused19;	/* +0a0 */
	volatile unsigned long unused20;	/* +0a4 */
	volatile unsigned long unused21;	/* +0a8 */
	volatile unsigned long unused22;	/* +0ac */
	volatile unsigned long unused23;	/* +0b0 */
	volatile unsigned long unused24;	/* +0b4 */
	volatile unsigned long unused25;	/* +0b8 */
	volatile unsigned long unused26;	/* +0bc */
	volatile unsigned long unused27;	/* +0c0 */
	volatile unsigned long unused28;	/* +0c4 */
	volatile unsigned long unused29;	/* +0c8 */
	volatile unsigned long unused30;	/* +0cc */
	volatile unsigned long unused31;	/* +0d0 */
	volatile unsigned long unused32;	/* +0d4 */
	volatile unsigned long unused33;	/* +0d8 */
	volatile unsigned char capid;	/* +0dc */
	volatile unsigned char nextitemptr;	/* +0dd */
	volatile unsigned short pmc;	/* +0de */
	volatile unsigned short pmcsr;	/* +0e0 */
	volatile unsigned short unused34;	/* +0e2 */
	volatile unsigned long unused35;	/* +0e4 */
	volatile unsigned long unused36;	/* +0e8 */
	volatile unsigned long unused37;	/* +0ec */
	volatile unsigned long unused38;	/* +0f0 */
	volatile unsigned long unused39;	/* +0f4 */
	volatile unsigned long unused40;	/* +0f8 */
	volatile unsigned long unused41;	/* +0fc */
	volatile unsigned long pbareqport;	/* +100 */
	volatile unsigned long pbacfg;	/* +104 */
	volatile unsigned long pbastatus;	/* +108 */
	volatile unsigned long pbamask;	/* +10c */
	volatile unsigned long pbabm;	/* +110 */
	volatile unsigned long pbacreq;	/* +114 */
	volatile unsigned long pbacgnt;	/* +118 */
	volatile unsigned long pbacstate;	/* +11c */
	volatile unsigned long g2pm0gbase;	/* +120 */
	volatile unsigned long unused42;	/* +124 */
	volatile unsigned long g2pm1gbase;	/* +128 */
	volatile unsigned long unused43;	/* +12c */
	volatile unsigned long g2pm2gbase;	/* +130 */
	volatile unsigned long unused44;	/* +134 */
	volatile unsigned long g2piogbase;	/* +138 */
	volatile unsigned long unused45;	/* +13c */
	volatile unsigned long g2pm0mask;	/* +140 */
	volatile unsigned long g2pm1mask;	/* +144 */
	volatile unsigned long g2pm2mask;	/* +148 */
	volatile unsigned long g2piomask;	/* +14c */
	volatile unsigned long g2pm0pbase;	/* +150 */
	volatile unsigned long unused46;	/* +154 */
	volatile unsigned long g2pm1pbase;	/* +158 */
	volatile unsigned long unused47;	/* +15c */
	volatile unsigned long g2pm2pbase;	/* +160 */
	volatile unsigned long unused48;	/* +164 */
	volatile unsigned long g2piopbase;	/* +168 */
	volatile unsigned long unused49;	/* +16c */
	volatile unsigned long pciccfg;	/* +170 */
	volatile unsigned long pcicstatus;	/* +174 */
	volatile unsigned long pcicmask;	/* +178 */
	volatile unsigned long unused50;	/* +17c */
	volatile unsigned long p2gm0gbase;	/* +180 */
	volatile unsigned long p2gm0ctr;	/* +184 */
	volatile unsigned long p2gm1gbase;	/* +188 */
	volatile unsigned long p2gm1ctr;	/* +18c */
	volatile unsigned long p2gm2gbase;	/* +190 */
	volatile unsigned long p2gm2ctr;	/* +194 */
	volatile unsigned long p2giogbase;	/* +198 */
	volatile unsigned long p2gioctr;	/* +19c */
	volatile unsigned long g2pcfgadrs;	/* +1a0 */
	volatile unsigned long g2pcfgdata;	/* +1a4 */
	volatile unsigned long unused51;	/* +1a8 */
	volatile unsigned long unused52;	/* +1ac */
	volatile unsigned long g2pidadrs;	/* +1b0 */
	volatile unsigned long unused53;	/* +1b4 */
	volatile unsigned long g2piddata;	/* +1b8 */
	volatile unsigned long unused54;	/* +1bc */
	volatile unsigned long g2pidcmd;	/* +1c0 */
	volatile unsigned long unused55;	/* +1c4 */
	volatile unsigned long g2pintack;	/* +1c8 */
	volatile unsigned long g2pspc;	/* +1cc */
	volatile unsigned long unused56;	/* +1d0 */
	volatile unsigned long unused57;	/* +1d4 */
	volatile unsigned long unused58;	/* +1d8 */
	volatile unsigned long unused59;	/* +1dc */
	volatile unsigned long pcidata0;	/* +1e0 */
	volatile unsigned long pcidata1;	/* +1e4 */
	volatile unsigned long pcidata2;	/* +1e8 */
	volatile unsigned long pcidata3;	/* +1ec */
	volatile unsigned long unused60;	/* +1f0 */
	volatile unsigned long unused61;	/* +1f4 */
	volatile unsigned long unused62;	/* +1f8 */
	volatile unsigned long unused63;	/* +1fc */
	volatile unsigned long pdmca;	/* +200 */
	volatile unsigned long pdmga;	/* +204 */
	volatile unsigned long pdmpa;	/* +208 */
	volatile unsigned long pdmctr;	/* +20c */
	volatile unsigned long pdmcfg;	/* +210 */
	volatile unsigned long pdmstatus;	/* +214 */
};

#endif				/* _LANGUAGE_ASSEMBLY */

/* IRCS : Int. Current Status */
#define TX4925_IRCS_IF			0x00010000
#define TX4925_IRCS_ILV_MASK		0x00000700
#define TX4925_IRCS_CAUSE_MASK		0x0000001f

/*
 * PCIC
 */

/* bits for G2PCFG */
#define TX4925_PCIC_G2PCFG_BSWAPM0      0x00000800
#define TX4925_PCIC_G2PCFG_BSWAPM1      0x00000400
#define TX4925_PCIC_G2PCFG_BSWAPM2      0x00000200
#define TX4925_PCIC_G2PCFG_BSWAPIO      0x00000100
#define TX4925_PCIC_G2PCFG_G2PM0EN      0x00000080
#define TX4925_PCIC_G2PCFG_G2PM1EN      0x00000040
#define TX4925_PCIC_G2PCFG_G2PM2EN      0x00000020
#define TX4925_PCIC_G2PCFG_G2PIOEN      0x00000010
#define TX4925_PCIC_G2PCFG_IRBER        0x00000008
#define TX4925_PCIC_G2PCFG_BSWAPI       0x00000002
#define TX4925_PCIC_G2PCFG_ASERR        0x00000001

/* bits for G2PSTATUS/G2PMASK */
#define TX4925_PCIC_G2PSTATUS_ALL       0x0000007f
#define TX4925_PCIC_G2PSTATUS_MDFE	0x00000040
#define TX4925_PCIC_G2PSTATUS_MDPE	0x00000020
#define TX4925_PCIC_G2PSTATUS_IDICC	0x00000010
#define TX4925_PCIC_G2PSTATUS_MIDPE	0x00000008
#define TX4925_PCIC_G2PSTATUS_MIDFE	0x00000004
#define TX4925_PCIC_G2PSTATUS_IDTTOE	0x00000002
#define TX4925_PCIC_G2PSTATUS_IDRTOE	0x00000001

/* bits for PCIMASK (see also PCI_STATUS_XXX in linux/pci.h */
#define TX4925_PCIC_PCISTATUS_ALL       0x0000f900

/* bits for PBACFG */
#define TX4925_PCIC_PBACFG_FIXPA 	0x00000008
#define TX4925_PCIC_PBACFG_RPBA 	0x00000004
#define TX4925_PCIC_PBACFG_PBAEN	0x00000002
#define TX4925_PCIC_PBACFG_BMCEN	0x00000001

/* bits for PCICCFG */
#define TX4925_PCIC_PCICCFG_GBWC_MASK   0x000fff00
#define TX4925_PCIC_PCICCFG_RESERVE     0x00000010
#define TX4925_PCIC_PCICCFG_HRST        0x00000008
#define TX4925_PCIC_PCICCFG_SRST        0x00000004
#define TX4925_PCIC_PCICCFG_TCAR	0x00000002
#define TX4925_PCIC_PCICCFG_LCFG	0x00000001

/* bits for PCICSTATUS/PCICMASK */
#define TX4925_PCIC_PCICSTATUS_ALL      0x00000001

/* bits for P2GMnCTR */
#define TX4925_PCIC_P2GMnCTR_AM_MASK	0x1ff00000
#define TX4925_PCIC_P2GMnCTR_TPRBL_1DW	0x00000000
#define TX4925_PCIC_P2GMnCTR_TPRBL_4DW	0x00000100
#define TX4925_PCIC_P2GMnCTR_TPRBL_8DW	0x00000200
#define TX4925_PCIC_P2GMnCTR_TPRBL_16DW	0x00000300
#define TX4925_PCIC_P2GMnCTR_TPRBL_32DW	0x00000400
#define TX4925_PCIC_P2GMnCTR_TMCC	0x00000010
#define TX4925_PCIC_P2GMnCTR_RESERVE	0x00000008
#define TX4925_PCIC_P2GMnCTR_MEMnPE	0x00000004
#define TX4925_PCIC_P2GMnCTR_P2GMnEN	0x00000002
#define TX4925_PCIC_P2GMnCTR_BSWAP	0x00000001

/* bits for P2GIOCTR */
#define TX4925_PCIC_P2GIOCTR_AM_MASK	0x0000ff00
#define TX4925_PCIC_P2GIOCTR_P2GIOEN	0x00000002
#define TX4925_PCIC_P2GIOCTR_BSWAP	0x00000001

#define TX4925_PCIC_IDSEL_AD_TO_SLOT(ad)        ((ad) - 11)
#define TX4925_PCIC_MAX_DEVNU   TX4925_PCIC_IDSEL_AD_TO_SLOT(31)

#ifndef _LANGUAGE_ASSEMBLY

#define tx4925_sdramcptr	((struct tx4925_sdramc_reg *)TX4925_REG(TX4925_SDRAMC_BASE))
#define tx4925_pcicptr		((struct tx4925_pcic_reg *)TX4925_REG(TX4925_PCIC_BASE))
#define tx4925_ccfgptr		((struct tx4925_ccfg_reg *)TX4925_REG(TX4925_CONFIG_BASE))
#define tx4925_ebuscptr		((struct tx4925_ebusc_reg *)TX4925_REG(TX4925_EBUSC_BASE))
#define tx4925_ircptr		((struct tx4925_irc_reg *)TX4925_REG(TX4925_IRC_BASE))

#endif				/* _LANGUAGE_ASSEMBLY */

#endif				/*  TX4925_PCI_H */
