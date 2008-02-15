#ifndef TX4925_TX4925_H
#define TX4925_TX4925_H

/*
 * linux/include/asm-mips/tx4925/tx4925.h
 *
 * tx4925 defs
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * Copyright 2001-2002 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <asm/tx4925/tx4925_mips.h>

/*
 This register naming came from the intergrate cpu/controoler name TX4925
 followed by the device name from table 4.2.2 on page 4-3 and then followed
 by the register name from table 4.2.3 on pages 4-4 to 4-8.  The manaul
 used is "TMPR4925BT Preliminary Rev 0.1 20.Jul.2001".
 */

/* TX4925 controller */
#define TX4925_BASE                     0xff1f0000
#define TX4925_LIMIT                    0xff1fffff

#define TX4925_MKA(x) ((u32)( ((u32)(TX4925_BASE)) | ((u32)(x)) ))

/* TX4925 SDRAM controller (64-bit registers) */
#define TX4925_SDRAMC_BASE              0x8000
#define TX4925_SDRAMC_SDCCR0            0x8000
#define TX4925_SDRAMC_SDCCR1            0x8008
#define TX4925_SDRAMC_SDCCR2            0x8010
#define TX4925_SDRAMC_SDCCR3            0x8018
#define TX4925_SDRAMC_SDCTR             0x8040
#define TX4925_SDRAMC_SDCMD             0x8058
#define TX4925_SDRAMC_LIMIT             0x8fff

/* TX4925 external bus controller (64-bit registers) */
#define TX4925_EBUSC_BASE               0x9000
#define TX4925_EBUSC_EBCCR0             0x9000
#define TX4925_EBUSC_EBCCR1             0x9008
#define TX4925_EBUSC_EBCCR2             0x9010
#define TX4925_EBUSC_EBCCR3             0x9018
#define TX4925_EBUSC_EBCCR4             0x9020
#define TX4925_EBUSC_EBCCR5             0x9028
#define TX4925_EBUSC_EBCCR6             0x9030
#define TX4925_EBUSC_EBCCR7             0x9008
#define TX4925_EBUSC_LIMIT              0x9fff

/* TX4925 SDRRAM Error Check Correction (64-bit registers) */
#define TX4925_ECC_BASE                 0xa000
#define TX4925_ECC_ECCCR                0xa000
#define TX4925_ECC_ECCSR                0xa008
#define TX4925_ECC_LIMIT                0xafff

/* TX4925 DMA Controller (64-bit registers) */
#define TX4925_DMAC_BASE                0xb000
#define TX4925_DMAC_TBD                 0xb000
#define TX4925_DMAC_LIMIT               0xbfff

/* TX4925 NAND Flash Memory Controller (32-bit registers) */
#define TX4925_NDFMC_BASE               0xc000
#define TX4925_NDFMC_NDFDTR             0xc000
#define TX4925_NDFMC_NDFMCR             0xc004
#define TX4925_NDFMC_NDFSR              0xc008
#define TX4925_NDFMC_NDFISR             0xc00c
#define TX4925_NDFMC_NDFIMR             0xc010
#define TX4925_NDFMC_NDFSPR             0xc014
#define TX4925_NDFMC_NDFRSTR            0xc018

/* TX4925 PCI Controller (32-bit registers) */
#define TX4925_PCIC_BASE                0xd000
#define TX4925_PCIC_TBD                 0xb000
#define TX4925_PCIC_LIMIT               0xdfff

/* TX4925 Configuration registers (64-bit registers) */
#define TX4925_CONFIG_BASE                       0xe000
#define TX4925_CONFIG_CCFG                       0xe000
#define TX4925_CONFIG_CCFG_RESERVED_26_31                BM_31_26
#define TX4925_CONFIG_CCFG_RF                            BM_25_24
#define TX4925_CONFIG_CCFG_BOOTME                        BM_23_21
#define TX4925_CONFIG_CCFG_PCIMODE                       BM_20_20
#define TX4925_CONFIG_CCFG_RESERVED_18_19                BM_19_18
#define TX4925_CONFIG_CCFG_TINTDIS                       BM_17_17
#define TX4925_CONFIG_CCFG_BEOW                          BM_16_16
#define TX4925_CONFIG_CCFG_WR                            BM_15_15
#define TX4925_CONFIG_CCFG_TOE                           BM_14_14
#define TX4925_CONFIG_CCFG_PCIARB                        BM_13_13
#define TX4925_CONFIG_CCFG_RESERVED_08_12                BM_12_08
#define TX4925_CONFIG_CCFG_SYSSP                         BM_07_06
#define TX4925_CONFIG_CCFG_RESERVED_04_05                BM_05_04
#define TX4925_CONFIG_CCFG_PCTRCE                        BM_03_03
#define TX4925_CONFIG_CCFG_ENDIAN                        BM_02_02
#define TX4925_CONFIG_CCFG_WDRST                         BM_01_01
#define TX4925_CONFIG_CCFG_UAEHOLD                       BM_00_00
#define TX4925_CONFIG_REVID                      0xe004
#define TX4925_CONFIG_REVID_PCODE                        BM_16_31
#define TX4925_CONFIG_REVID_MJERREV                      BM_12_15
#define TX4925_CONFIG_REVID_MINEREV                      BM_08_11
#define TX4925_CONFIG_REVID_MJREV                        BM_04_07
#define TX4925_CONFIG_REVID_MINREV                       BM_00_03
#define TX4925_CONFIG_PCFG                       0xe008
#define TX4925_CONFIG_PCFG_SYSCLKEN                      BM_31_31
#define TX4925_CONFIG_PCFG_SDRCLKEN                      BM_30_29
#define TX4925_CONFIG_PCFG_PCICLKEN                      BM_28_27
#define TX4925_CONFIG_PCFG_PCICLKIOEN                    BM_26_26
#define TX4925_CONFIG_PCFG_RESERVED_22_25                BM_25_22
#define TX4925_CONFIG_PCFG_SELSPL                        BM_21_21
#define TX4925_CONFIG_PCFG_SELCHI                        BM_20_20
#define TX4925_CONFIG_PCFG_SELCARD                       BM_19_18
#define TX4925_CONFIG_PCFG_SELCE                         BM_17_16
#define TX4925_CONFIG_PCFG_SELSIOC                       BM_15_14
#define TX4925_CONFIG_PCFG_SELSIO                        BM_13_12
#define TX4925_CONFIG_PCFG_ACKIN                         BM_11_11
#define TX4925_CONFIG_PCFG_SELTMR                        BM_10_09
#define TX4925_CONFIG_PCFG_SELDONE                       BM_08_08
#define TX4925_CONFIG_PCFG_RESERVED_04_07                BM_07_04
#define TX4925_CONFIG_PCFG_SELACLC                       BM_03_03
#define TX4925_CONFIG_PCFG_SELNAND                       BM_02_02
#define TX4925_CONFIG_PCFG_SELDMA                        BM_01_00
#define TX4925_CONFIG_TOEA                       0xe00c
#define TX4925_CONFIG_TOEA_TOEA                          BM_00_31
#define TX4925_CONFIG_CLKCTR                     0xe028
#define TX4925_CONFIG_CLKCTR_RESERVED_28_31              BM_31_28
#define TX4925_CONFIG_CLKCTR_PCICKE                      BM_27_27
#define TX4925_CONFIG_CLKCTR_DMACKE                      BM_26_26
#define TX4925_CONFIG_CLKCTR_RESERVED_25_25              BM_25_25
#define TX4925_CONFIG_CLKCTR_SIO0CKE                     BM_24_24
#define TX4925_CONFIG_CLKCTR_SIO1CKE                     BM_23_23
#define TX4925_CONFIG_CLKCTR_TMR0CKE                     BM_22_22
#define TX4925_CONFIG_CLKCTR_TMR1CKE                     BM_21_21
#define TX4925_CONFIG_CLKCTR_TMR2CKE                     BM_20_20
#define TX4925_CONFIG_CLKCTR_CHICKE                      BM_19_19
#define TX4925_CONFIG_CLKCTR_SPICKE                      BM_18_18
#define TX4925_CONFIG_CLKCTR_ACLCKE                      BM_17_17
#define TX4925_CONFIG_CLKCTR_PIOCKE                      BM_16_16
#define TX4925_CONFIG_CLKCTR_RESERVED_12_15              BM_15_12
#define TX4925_CONFIG_CLKCTR_PCIRSTI                     BM_11_11
#define TX4925_CONFIG_CLKCTR_DMARSTI                     BM_10_10
#define TX4925_CONFIG_CLKCTR_RESERVED_09_09              BM_09_09
#define TX4925_CONFIG_CLKCTR_SIO0RSTI                    BM_08_08
#define TX4925_CONFIG_CLKCTR_SIO1RSTI                    BM_07_07
#define TX4925_CONFIG_CLKCTR_TMR0RSTI                    BM_06_06
#define TX4925_CONFIG_CLKCTR_TMR1RSTI                    BM_05_05
#define TX4925_CONFIG_CLKCTR_TMR2RSTI                    BM_04_04
#define TX4925_CONFIG_CLKCTR_CHIRSTI                     BM_03_03
#define TX4925_CONFIG_CLKCTR_SPIRSTI                     BM_02_02
#define TX4925_CONFIG_CLKCTR_ACLRSTI                     BM_01_01
#define TX4925_CONFIG_CLKCTR_PIORSTI                     BM_00_00
#define TX4925_CONFIG_GARBC                      0xe02c
#define TX4925_CONFIG_GARBC_RESERVED_04_31               BM_31_04
#define TX4925_CONFIG_GARBC_ARBMD                        BM_03_00
#define TX4925_CONFIG_RAMP                       0xe030
#define TX4925_CONFIG_RAMP_RESERVED_16_31                BM_31_16
#define TX4925_CONFIG_RAMP_RAMP                          BM_15_00
#define TX4925_CONFIG_LIMIT                      0xe034

/* TX4925 Timer 0 (32-bit registers) */
#define TX4925_TMR0_BASE                0xf000
#define TX4925_TMR0_TMTCR0              0xf004
#define TX4925_TMR0_TMTISR0             0xf008
#define TX4925_TMR0_TMCPRA0             0xf008
#define TX4925_TMR0_TMCPRB0             0xf00c
#define TX4925_TMR0_TMITMR0             0xf010
#define TX4925_TMR0_TMCCDR0             0xf020
#define TX4925_TMR0_TMPGMR0             0xf030
#define TX4925_TMR0_TMTRR0              0xf0f0
#define TX4925_TMR0_LIMIT               0xf0ff

/* TX4925 Timer 1 (32-bit registers) */
#define TX4925_TMR1_BASE                0xf100
#define TX4925_TMR1_TMTCR1              0xf104
#define TX4925_TMR1_TMTISR1             0xf108
#define TX4925_TMR1_TMCPRA1             0xf108
#define TX4925_TMR1_TMCPRB1             0xf10c
#define TX4925_TMR1_TMITMR1             0xf110
#define TX4925_TMR1_TMCCDR1             0xf120
#define TX4925_TMR1_TMPGMR1             0xf130
#define TX4925_TMR1_TMTRR1              0xf1f0
#define TX4925_TMR1_LIMIT               0xf1ff

/* TX4925 Timer 2 (32-bit registers) */
#define TX4925_TMR2_BASE                0xf200
#define TX4925_TMR2_TMTCR2              0xf104
#define TX4925_TMR2_TMTISR2             0xf208
#define TX4925_TMR2_TMCPRA2             0xf208
#define TX4925_TMR2_TMCPRB2             0xf20c
#define TX4925_TMR2_TMITMR2             0xf210
#define TX4925_TMR2_TMCCDR2             0xf220
#define TX4925_TMR2_TMPGMR2             0xf230
#define TX4925_TMR2_TMTRR2              0xf2f0
#define TX4925_TMR2_LIMIT               0xf2ff

/* TX4925 serial port 0 (32-bit registers) */
#define TX4925_SIO0_BASE                         0xf300
#define TX4925_SIO0_SILCR0                       0xf300
#define TX4925_SIO0_SILCR0_RESERVED_16_31                BM_16_31
#define TX4925_SIO0_SILCR0_RWUB                          BM_15_15
#define TX4925_SIO0_SILCR0_TWUB                          BM_14_14
#define TX4925_SIO0_SILCR0_UODE                          BM_13_13
#define TX4925_SIO0_SILCR0_RESERVED_07_12                BM_07_12
#define TX4925_SIO0_SILCR0_SCS                           BM_05_06
#define TX4925_SIO0_SILCR0_SCS_IMBUSCLK_IC             (~BM_05_06)
#define TX4925_SIO0_SILCR0_SCS_IMBUSCLK_BRG              BM_05_05
#define TX4925_SIO0_SILCR0_SCS_SCLK_EC                   BM_06_06
#define TX4925_SIO0_SILCR0_SCS_SCLK_BRG                  BM_05_06
#define TX4925_SIO0_SILCR0_UEPS                          BM_04_04
#define TX4925_SIO0_SILCR0_UPEN                          BM_03_03
#define TX4925_SIO0_SILCR0_USBL                          BM_02_02
#define TX4925_SIO0_SILCR0_UMODE                         BM_00_01
#define TX4925_SIO0_SILCR0_UMODE_DATA_8_BIT              BM_00_01
#define TX4925_SIO0_SILCR0_UMODE_DATA_7_BIT            (~BM_00_01)
#define TX4925_SIO0_SILCR0_UMODE_DATA_8_BIT_MC           BM_01_01
#define TX4925_SIO0_SILCR0_UMODE_DATA_7_BIT_MC           BM_00_01
#define TX4925_SIO0_SIDICR0                      0xf304
#define TX4925_SIO0_SIDICR0_RESERVED_16_31               BM_16_31
#define TX4925_SIO0_SIDICR0_TDE                          BM_15_15
#define TX4925_SIO0_SIDICR0_RDE                          BM_14_14
#define TX4925_SIO0_SIDICR0_TIE                          BM_13_13
#define TX4925_SIO0_SIDICR0_RIE                          BM_12_12
#define TX4925_SIO0_SIDICR0_SPIE                         BM_11_11
#define TX4925_SIO0_SIDICR0_CTSAC                        BM_09_10
#define TX4925_SIO0_SIDICR0_CTSAC_NONE                 (~BM_09_10)
#define TX4925_SIO0_SIDICR0_CTSAC_RISE                   BM_09_09
#define TX4925_SIO0_SIDICR0_CTSAC_FALL                   BM_10_10
#define TX4925_SIO0_SIDICR0_CTSAC_BOTH                   BM_09_10
#define TX4925_SIO0_SIDICR0_RESERVED_06_08               BM_06_08
#define TX4925_SIO0_SIDICR0_STIE                         BM_00_05
#define TX4925_SIO0_SIDICR0_STIE_NONE                  (~BM_00_05)
#define TX4925_SIO0_SIDICR0_STIE_OERS                    BM_05_05
#define TX4925_SIO0_SIDICR0_STIE_CTSAC                   BM_04_04
#define TX4925_SIO0_SIDICR0_STIE_RBRKD                   BM_03_03
#define TX4925_SIO0_SIDICR0_STIE_TRDY                    BM_02_02
#define TX4925_SIO0_SIDICR0_STIE_TXALS                   BM_01_01
#define TX4925_SIO0_SIDICR0_STIE_UBRKD                   BM_00_00
#define TX4925_SIO0_SIDISR0                      0xf308
#define TX4925_SIO0_SIDISR0_RESERVED_16_31               BM_16_31
#define TX4925_SIO0_SIDISR0_UBRK                         BM_15_15
#define TX4925_SIO0_SIDISR0_UVALID                       BM_14_14
#define TX4925_SIO0_SIDISR0_UFER                         BM_13_13
#define TX4925_SIO0_SIDISR0_UPER                         BM_12_12
#define TX4925_SIO0_SIDISR0_UOER                         BM_11_11
#define TX4925_SIO0_SIDISR0_ERI                          BM_10_10
#define TX4925_SIO0_SIDISR0_TOUT                         BM_09_09
#define TX4925_SIO0_SIDISR0_TDIS                         BM_08_08
#define TX4925_SIO0_SIDISR0_RDIS                         BM_07_07
#define TX4925_SIO0_SIDISR0_STIS                         BM_06_06
#define TX4925_SIO0_SIDISR0_RESERVED_05_05               BM_05_05
#define TX4925_SIO0_SIDISR0_RFDN                         BM_00_04
#define TX4925_SIO0_SISCISR0                     0xf30c
#define TX4925_SIO0_SISCISR0_RESERVED_06_31              BM_06_31
#define TX4925_SIO0_SISCISR0_OERS                        BM_05_05
#define TX4925_SIO0_SISCISR0_CTSS                        BM_04_04
#define TX4925_SIO0_SISCISR0_RBRKD                       BM_03_03
#define TX4925_SIO0_SISCISR0_TRDY                        BM_02_02
#define TX4925_SIO0_SISCISR0_TXALS                       BM_01_01
#define TX4925_SIO0_SISCISR0_UBRKD                       BM_00_00
#define TX4925_SIO0_SIFCR0                       0xf310
#define TX4925_SIO0_SIFCR0_RESERVED_16_31                BM_16_31
#define TX4925_SIO0_SIFCR0_SWRST                         BM_16_31
#define TX4925_SIO0_SIFCR0_RESERVED_09_14                BM_09_14
#define TX4925_SIO0_SIFCR0_RDIL                          BM_16_31
#define TX4925_SIO0_SIFCR0_RDIL_BYTES_1                (~BM_07_08)
#define TX4925_SIO0_SIFCR0_RDIL_BYTES_4                  BM_07_07
#define TX4925_SIO0_SIFCR0_RDIL_BYTES_8                  BM_08_08
#define TX4925_SIO0_SIFCR0_RDIL_BYTES_12                 BM_07_08
#define TX4925_SIO0_SIFCR0_RESERVED_05_06                BM_05_06
#define TX4925_SIO0_SIFCR0_TDIL                          BM_03_04
#define TX4925_SIO0_SIFCR0_TDIL_BYTES_1                (~BM_03_04)
#define TX4925_SIO0_SIFCR0_TDIL_BYTES_4                  BM_03_03
#define TX4925_SIO0_SIFCR0_TDIL_BYTES_8                  BM_04_04
#define TX4925_SIO0_SIFCR0_TDIL_BYTES_0                  BM_03_04
#define TX4925_SIO0_SIFCR0_TFRST                         BM_02_02
#define TX4925_SIO0_SIFCR0_RFRST                         BM_01_01
#define TX4925_SIO0_SIFCR0_FRSTE                         BM_00_00
#define TX4925_SIO0_SIFLCR0                      0xf314
#define TX4925_SIO0_SIFLCR0_RESERVED_13_31               BM_13_31
#define TX4925_SIO0_SIFLCR0_RCS                          BM_12_12
#define TX4925_SIO0_SIFLCR0_TES                          BM_11_11
#define TX4925_SIO0_SIFLCR0_RESERVED_10_10               BM_10_10
#define TX4925_SIO0_SIFLCR0_RTSSC                        BM_09_09
#define TX4925_SIO0_SIFLCR0_RSDE                         BM_08_08
#define TX4925_SIO0_SIFLCR0_TSDE                         BM_07_07
#define TX4925_SIO0_SIFLCR0_RESERVED_05_06               BM_05_06
#define TX4925_SIO0_SIFLCR0_RTSTL                        BM_01_04
#define TX4925_SIO0_SIFLCR0_TBRK                         BM_00_00
#define TX4925_SIO0_SIBGR0                       0xf318
#define TX4925_SIO0_SIBGR0_RESERVED_10_31                BM_10_31
#define TX4925_SIO0_SIBGR0_BCLK                          BM_08_09
#define TX4925_SIO0_SIBGR0_BCLK_T0                     (~BM_08_09)
#define TX4925_SIO0_SIBGR0_BCLK_T2                       BM_08_08
#define TX4925_SIO0_SIBGR0_BCLK_T4                       BM_09_09
#define TX4925_SIO0_SIBGR0_BCLK_T6                       BM_08_09
#define TX4925_SIO0_SIBGR0_BRD                           BM_00_07
#define TX4925_SIO0_SITFIF00                     0xf31c
#define TX4925_SIO0_SITFIF00_RESERVED_08_31              BM_08_31
#define TX4925_SIO0_SITFIF00_TXD                         BM_00_07
#define TX4925_SIO0_SIRFIFO0                     0xf320
#define TX4925_SIO0_SIRFIFO0_RESERVED_08_31              BM_08_31
#define TX4925_SIO0_SIRFIFO0_RXD                         BM_00_07
#define TX4925_SIO0_SIRFIFO0                     0xf320
#define TX4925_SIO0_LIMIT                        0xf3ff

/* TX4925 serial port 1 (32-bit registers) */
#define TX4925_SIO1_BASE                0xf400
#define TX4925_SIO1_SILCR1              0xf400
#define TX4925_SIO1_SIDICR1             0xf404
#define TX4925_SIO1_SIDISR1             0xf408
#define TX4925_SIO1_SISCISR1            0xf40c
#define TX4925_SIO1_SIFCR1              0xf410
#define TX4925_SIO1_SIFLCR1             0xf414
#define TX4925_SIO1_SIBGR1              0xf418
#define TX4925_SIO1_SITFIF01            0xf41c
#define TX4925_SIO1_SIRFIFO1            0xf420
#define TX4925_SIO1_LIMIT               0xf4ff

/* TX4925 parallel port (32-bit registers) */
#define TX4925_PIO_BASE                 0xf500
#define TX4925_PIO_PIOD0                0xf500
#define TX4925_PIO_PIODI                0xf504
#define TX4925_PIO_PIODIR               0xf508
#define TX4925_PIO_PIOOD                0xf50c
#define TX4925_PIO_LIMIT                0xf50f

/* TX4925 Interrupt Controller (32-bit registers) */
#define TX4925_IRC_BASE                 0xf510
#define TX4925_IRC_IRFLAG0              0xf510
#define TX4925_IRC_IRFLAG1              0xf514
#define TX4925_IRC_IRPOL                0xf518
#define TX4925_IRC_IRRCNT               0xf51c
#define TX4925_IRC_IRMASKINT            0xf520
#define TX4925_IRC_IRMASKEXT            0xf524
#define TX4925_IRC_IRDEN                0xf600
#define TX4925_IRC_IRDM0                0xf604
#define TX4925_IRC_IRDM1                0xf608
#define TX4925_IRC_IRLVL0               0xf610
#define TX4925_IRC_IRLVL1               0xf614
#define TX4925_IRC_IRLVL2               0xf618
#define TX4925_IRC_IRLVL3               0xf61c
#define TX4925_IRC_IRLVL4               0xf620
#define TX4925_IRC_IRLVL5               0xf624
#define TX4925_IRC_IRLVL6               0xf628
#define TX4925_IRC_IRLVL7               0xf62c
#define TX4925_IRC_IRMSK                0xf640
#define TX4925_IRC_IREDC                0xf660
#define TX4925_IRC_IRPND                0xf680
#define TX4925_IRC_IRCS                 0xf6a0
#define TX4925_IRC_LIMIT                0xf6ff

/* TX4925 AC-link controller (32-bit registers) */
#define TX4925_ACLC_BASE                0xf700
#define TX4925_ACLC_ACCTLEN             0xf700
#define TX4925_ACLC_ACCTLDIS            0xf704
#define TX4925_ACLC_ACREGACC            0xf708
#define TX4925_ACLC_ACINTSTS            0xf710
#define TX4925_ACLC_ACINTMSTS           0xf714
#define TX4925_ACLC_ACINTEN             0xf718
#define TX4925_ACLC_ACINTDIS            0xfR71c
#define TX4925_ACLC_ACSEMAPH            0xf720
#define TX4925_ACLC_ACGPIDAT            0xf740
#define TX4925_ACLC_ACGPODAT            0xf744
#define TX4925_ACLC_ACSLTEN             0xf748
#define TX4925_ACLC_ACSLTDIS            0xf74c
#define TX4925_ACLC_ACFIFOSTS           0xf750
#define TX4925_ACLC_ACDMASTS            0xf780
#define TX4925_ACLC_ACDMASEL            0xf784
#define TX4925_ACLC_ACAUDODAT           0xf7a0
#define TX4925_ACLC_ACSURRDAT           0xf7a4
#define TX4925_ACLC_ACCENTDAT           0xf7a8
#define TX4925_ACLC_ACLFEDAT            0xf7ac
#define TX4925_ACLC_ACAUDIDAT           0xf7b0
#define TX4925_ACLC_ACMODODAT           0xf7b8
#define TX4925_ACLC_ACMODIDAT           0xf7bc
#define TX4925_ACLC_ACREVID             0xf7fc
#define TX4925_ACLC_LIMIT               0xf7ff

#define TX4925_REG(x) ((TX4925_BASE)+(x))

#define TX4925_RD08( reg      )   (*(vu08*)(reg))
#define TX4925_WR08( reg, val )  ((*(vu08*)(reg))=(val))

#define TX4925_RD16( reg      )   (*(vu16*)(reg))
#define TX4925_WR16( reg, val )  ((*(vu16*)(reg))=(val))

#define TX4925_RD32( reg      )   (*(vu32*)(reg))
#define TX4925_WR32( reg, val )  ((*(vu32*)(reg))=(val))

#define TX4925_RD64( reg      )   (*(vu64*)(reg))
#define TX4925_WR64( reg, val )  ((*(vu64*)(reg))=(val))

#define TX4925_RD( reg      ) TX4925_RD32( reg )
#define TX4925_WR( reg, val ) TX4925_WR32( reg, val )

#define MI8259_IRQ_ISA_RAW_BEG   0	/* optional backplane i8259 */
#define MI8259_IRQ_ISA_RAW_END  15
#define TX4925_IRQ_CP0_RAW_BEG   0	/* tx4925 cpu built-in cp0 */
#define TX4925_IRQ_CP0_RAW_END   7
#define TX4925_IRQ_PIC_RAW_BEG   0	/* tx4925 cpu build-in pic */
#define TX4925_IRQ_PIC_RAW_END  31

#define MI8259_IRQ_ISA_BEG                          MI8259_IRQ_ISA_RAW_BEG	/*  0 */
#define MI8259_IRQ_ISA_END                          MI8259_IRQ_ISA_RAW_END	/* 15 */

#define TX4925_IRQ_CP0_BEG  ((MI8259_IRQ_ISA_END+1)+TX4925_IRQ_CP0_RAW_BEG)	/* 16 */
#define TX4925_IRQ_CP0_END  ((MI8259_IRQ_ISA_END+1)+TX4925_IRQ_CP0_RAW_END)	/* 23 */

#define TX4925_IRQ_PIC_BEG  ((TX4925_IRQ_CP0_END+1)+TX4925_IRQ_PIC_RAW_BEG)	/* 24 */
#define TX4925_IRQ_PIC_END  ((TX4925_IRQ_CP0_END+1)+TX4925_IRQ_PIC_RAW_END)	/* 55 */

#define TOSHIBA_RBTX4925_IRQ_IOC_RAW_BEG   0
#define TOSHIBA_RBTX4925_IRQ_IOC_RAW_END   7

#define TOSHIBA_RBTX4925_IRQ_IOC_BEG  ((TX4925_IRQ_PIC_END+1)+TOSHIBA_RBTX4925_IRQ_IOC_RAW_BEG)
#define TOSHIBA_RBTX4925_IRQ_IOC_END  ((TX4925_IRQ_PIC_END+1)+TOSHIBA_RBTX4925_IRQ_IOC_RAW_END)

#define TOSHIBA_RBTX4925_PCID     0
#define TOSHIBA_RBTX4925_PCIC     1
#define TOSHIBA_RBTX4925_PCIB     2
#define TOSHIBA_RBTX4925_PCIA     3

#define TOSHIBA_RBTX4925_IRQ_IOC_PCI_A  TOSHIBA_RBTX4925_IRQ_IOC_BEG + TOSHIBA_RBTX4925_PCIA
#define TOSHIBA_RBTX4925_IRQ_IOC_PCI_B  TOSHIBA_RBTX4925_IRQ_IOC_BEG + TOSHIBA_RBTX4925_PCIB
#define TOSHIBA_RBTX4925_IRQ_IOC_PCI_C  TOSHIBA_RBTX4925_IRQ_IOC_BEG + TOSHIBA_RBTX4925_PCIC
#define TOSHIBA_RBTX4925_IRQ_IOC_PCI_D  TOSHIBA_RBTX4925_IRQ_IOC_BEG + TOSHIBA_RBTX4925_PCID

#define TX4925_IRQ_USER0            (TX4925_IRQ_CP0_BEG+0)
#define TX4925_IRQ_USER1            (TX4925_IRQ_CP0_BEG+1)
#define TX4925_IRQ_NEST_PIC_ON_CP0  (TX4925_IRQ_CP0_BEG+2)
#define TX4925_IRQ_CPU_TIMER        (TX4925_IRQ_CP0_BEG+7)

#define TX4925_IRQ_NEST_EXT_ON_PIC  (TX4925_IRQ_PIC_BEG+2)

#endif
