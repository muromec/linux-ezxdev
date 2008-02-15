#ifndef __ASM_SH_UBC_H
#define __ASM_SH_UBC_H

/* User Break Controller */

#if defined(CONFIG_CPU_SUBTYPE_SH7300) || defined(CONFIG_CPU_SUBTYPE_SH7707) || \
    defined(CONFIG_CPU_SUBTYPE_SH7709)
#define UBC_TYPE_SH7729	(cpu_data->type == CPU_SH7729)
#else
#define UBC_TYPE_SH7729	0
#endif

#if defined(__sh3__)
#define UBC_BARA                0xffffffb0
#define UBC_BAMRA               0xffffffb4
#define UBC_BBRA                0xffffffb8
#define UBC_BASRA               0xffffffe4
#define UBC_BARB                0xffffffa0
#define UBC_BAMRB               0xffffffa4
#define UBC_BBRB                0xffffffa8
#define UBC_BASRB               0xffffffe8
#define UBC_BDRB                0xffffff90
#define UBC_BDMRB               0xffffff94
#define UBC_BRCR                0xffffff98
#elif defined(__SH4__)
#define UBC_BARA		0xff200000
#define UBC_BAMRA		0xff200004
#define UBC_BBRA		0xff200008
#define UBC_BASRA		0xff000014
#define UBC_BARB		0xff20000c
#define UBC_BAMRB		0xff200010
#define UBC_BBRB		0xff200014
#define UBC_BASRB		0xff000018
#define UBC_BDRB		0xff200018
#define UBC_BDMRB		0xff20001c
#define UBC_BRCR		0xff200020
#endif

#if defined(CONFIG_CPU_SUBTYPE_SH73180)
#define UBC_BASE0		0xff200000
#define	UBC_CBR0		(UBC_BASE0 + 0x00)
#define	UBC_CRR0		(UBC_BASE0 + 0x04)
#define	UBC_CAR0		(UBC_BASE0 + 0x08)
#define	UBC_CAMR0		(UBC_BASE0 + 0x0c)

#define UBC_BASE1               0xff200020
#define	UBC_CBR1		(UBC_BASE1 + 0x00)
#define	UBC_CRR1		(UBC_BASE1 + 0x04)
#define	UBC_CAR1		(UBC_BASE1 + 0x08)
#define	UBC_CAMR1		(UBC_BASE1 + 0x0c)

#define	UBC_CDR1		0xff200030
#define	UBC_CDMR1		0xff200034
#define	UBC_CETR1		0xff200038

#define	UBC_CCMFR		0xff200600
#define	UBC_CBCR		0xff200620

#define CBR_AIE                 (1 << 30)
#define CBR_AIV_SHIFT           16
#define CBR_INST                (1 << 4)
#define CBR_READ                (3 << 1)
#define CBR_ENABLE              (1 << 0)
#define CBR_INIT                (CBR_AIE | CBR_INST | CBR_READ | CBR_ENABLE)
#define CBR_DISABLE             (CBR_AIE | CBR_INST | CBR_READ)


#define CRR_PCB                 (1 << 1)
#define CRR_BIE                 (1 << 0)
#define CRR_INIT                ((1 << 13) | CRR_BIE)

#define CBCR_UBDE               (1 << 0)
#define CBCR_INIT               0x0

#else /* CONFIG_CPU_SUBTYPE_SH73180 */

#define BAMR_ASID		(1 << 2)
#define BAMR_NONE		0
#define BAMR_10			0x1
#define BAMR_12			0x2
#define BAMR_ALL		0x3
#define BAMR_16			0x8
#define BAMR_20			0x9

#define BBR_INST		(1 << 4)
#define BBR_DATA		(2 << 4)
#define BBR_READ		(1 << 2)
#define BBR_WRITE		(2 << 2)
#define BBR_BYTE		0x1
#define BBR_HALF		0x2
#define BBR_LONG		0x3
#define BBR_QUAD		(1 << 6)	/* SH7750 */
#define BBR_CPU			(1 << 6)	/* SH7709A,SH7729 */
#define BBR_DMA			(2 << 6)	/* SH7709A,SH7729 */

#define BRCR_CMFA		(1 << 15)
#define BRCR_CMFB		(1 << 14)
#define BRCR_PCTE		(1 << 11)
#define BRCR_PCBA		(1 << 10)	/* 1: after execution */
#define BRCR_DBEB		(1 << 7)
#define BRCR_PCBB		(1 << 6)
#define BRCR_SEQ		(1 << 3)
#define BRCR_UBDE		(1 << 0)

#endif /* CONFIG_CPU_SUBTYPE_SH73180 */

#ifndef __ASSEMBLY__
/* arch/sh/kernel/ubc.S */
extern void ubc_wakeup(void);
extern void ubc_sleep(void);
#endif

#endif /* __ASM_SH_UBC_H */

