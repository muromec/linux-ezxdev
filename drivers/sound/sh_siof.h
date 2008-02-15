/* SIOF REGS */
#define SIOF_IOBASE	0xa4410000UL
#define	SIMDR	(SIOF_IOBASE + 0x00)
#define	SISCR	(SIOF_IOBASE + 0x02)
#define SITDAR	(SIOF_IOBASE + 0x04)
#define	SIRDAR	(SIOF_IOBASE + 0x06)
#define	SICDAR	(SIOF_IOBASE + 0x08)
#define	SICTR	(SIOF_IOBASE + 0x0c)
#define	SIFCTR	(SIOF_IOBASE + 0x10)
#define	SISTR	(SIOF_IOBASE + 0x14)
#define	SIIER	(SIOF_IOBASE + 0x16)
#define	SITDR	(SIOF_IOBASE + 0x20)
#define	SIRDR	(SIOF_IOBASE + 0x24)
#define	SITCR	(SIOF_IOBASE + 0x28)
#define	SIRCR	(SIOF_IOBASE + 0x2c)
#define	SPICR	(SIOF_IOBASE + 0x30)

#define SCKE	(1 << 15)
#define FSE	(1 << 14)

#define TDMAE	(1 << 15)
#define RDMAE	(1 << 11)

#define TXRST   (1 << 1)
#define TXE	(1 << 9)
#define TFEMP   (1 << 13)
#define TFEMPE  (1 << 13)
#define TDREQE	(1 << 12)
#define	TDREQ	(1 << 12)
#define TFOVR	(1 << 3)
#define TFOVRE  (1 << 3)
#define TFUDRE  (1 << 2)
#define TFUDR   (1 << 2)

#define RXRST   (1 << 0)
#define RFFULE	(1 << 9)
#define RFFUL	(1 << 9)
#define RXE	(1 << 8)
#define RDREQE  (1 << 8)
#define RDREQ   (1 << 8)
#define RFUDRE  (1 << 1)
#define RFUDR   (1 << 1)
#define RFOVRE  (1 << 0)
#define RFOVR   (1 << 0)

#define M8B_MONO	1
#define M16B_MONO	2
#define M8B_STER	3
#define M16B_STER	4

/* DMA REGS */
#define DMAC_BASE 0xa4010000
#define DMAC2_BASE (DMAC_BASE+0x40)
#define DMAC3_BASE (DMAC_BASE+0x50)
#define SAR_OFF 0x00
#define DAR_OFF 0x04
#define DMATCR_OFF 0x08
#define CHCR_OFF 0x0c

#define DE	(1 << 0)
#define TE	(1 << 1)
/// tx
#define SAR3	(DMAC3_BASE+SAR_OFF)
#define DAR3	(DMAC3_BASE+DAR_OFF)
#define	DMATCR3	(DMAC3_BASE+DMATCR_OFF)
#define CHCR3	(DMAC3_BASE+CHCR_OFF)
/// rx
#define SAR2	(DMAC2_BASE+SAR_OFF)
#define DAR2	(DMAC2_BASE+DAR_OFF)
#define DMATCR2	(DMAC2_BASE+DMATCR_OFF)
#define CHCR2   (DMAC2_BASE+CHCR_OFF)

#define DMAOR	(DMAC_BASE+0x60)
#define DMARS0	0xa4090000
#define DMARS1	0xa4090004
#define DMARS2	0xa4090008

#define	STBCR3		0xA40A0000

 /**/
