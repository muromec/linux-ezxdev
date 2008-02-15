#ifndef __IRDA_MX21_h__
#define __IRDA_MX21_h__

#include<asm/arch/hardware.h>

typedef unsigned int U32;

/*
 * Refer to mx21 user manual, the memory map chapter
 * AIPI range:
 * AIPI1 -- 0x1000 0000 - 0x1001 ffff
 * AIPI2 -- 0x1002_0000 - 0x1003_ffff
 *
 * SIR/UART3 registers:
 *   0x1000_c000 - 0x1000_cfff
 * FIR registers:
 *   0x1002_8000 - 0x1002_8fff
 */

typedef struct _reg_layout {
	unsigned long shift;
	unsigned long mask;
} reg_layout_t;

/* URXD_3: UART3 Receiver Register -- 0x1000_C000
 *
 * URXD_3 layout (common part) is:
 * [15:0]
 *   CHARRDY ERR OVRRUN FRMERR BRK PRERR RX_DATA
 *
 */

/* Index of elements of URXD_3 layout*/
#define URXD_3_CHARRDY		6
#define URXD_3_ERR			5
#define URXD_3_OVRRUN		4
#define URXD_3_FRMERR		3
#define URXD_3_BRK			2
#define URXD_3_PRERR		1
#define URXD_3_RX_DATA		0

#define URXD_3_MAX_INDEX	7
reg_layout_t urxd_3_layout[URXD_3_MAX_INDEX]=
{
	{ 0,  0xff   },
	{ 10, 0x400  },
	{ 11, 0x800  },
	{ 12, 0x1000 },
	{ 13, 0x2000 },
	{ 14, 0x4000 },
	{ 15, 0x8000 }
};

/* UTXD_3: UART3 Transmitter Register -- 0x1000_C040
 *
 * UTXD_3 layout (common part) is:
 * [7:0]
 *   TX_DATA
 *
 */

/* Index of elements of UTXD_3 layout*/
#define UTXD_3_TX_DATA		0

#define UTXD_3_MAX_INDEX	1
reg_layout_t utxd_3_layout[UTXD_3_MAX_INDEX]=
{
	{ 0, 0xff}
};

/*******************************************************

				SIR Memory Map

*******************************************************/

/* UCR1_3: UART3 Control Register 1 -- 0x1000_C080
 *
 * UCR1_3 layout (common part) is:
 * [15:0]
 *  ADEN ADBR TRDYEN IDEN ICD RRDYEN RXDMAEN IREN TXMPTYEN RTSDEN SNDBRK TXDMAEN DOZE UARTEN
 *
 */

/* Index of elements of UCR1_3 layout*/
#define UCR1_3_ADEN			13
#define UCR1_3_ADBR			12
#define UCR1_3_TRDYEN		11
#define UCR1_3_IDEN			10
#define UCR1_3_ICD			9
#define UCR1_3_RRDYEN		8
#define UCR1_3_RXDMAEN		7
#define UCR1_3_IREN			6
#define UCR1_3_TXMPTYEN		5
#define UCR1_3_RTSDEN		4
#define UCR1_3_SNDBRK		3
#define UCR1_3_TXDMAEN		2
#define UCR1_3_DOZE			1
#define UCR1_3_UARTEN		0

#define UCR1_3_MAX_INDEX	14
reg_layout_t ucr1_3_layout[UCR1_3_MAX_INDEX]=
{
	{ 0,  0x1  },
	{ 1,  0x2  },
	{ 3,  0x8  },
	{ 4,  0x10 },
	{ 5,  0x20 },
	{ 6,  0x40 },
	{ 7,  0x80 },
	{ 8,  0x100},
	{ 9,  0x200},
	{ 10, 0xc00},
	{ 12, 0x1000},
	{ 13, 0x2000},
	{ 14, 0x4000},
	{ 15, 0x8000}
};

/* UCR2_3: UART3 Control Register 2 -- 0x1000_C084
 *
 * UCR2_3 layout (common part) is:
 * [15:0]
 *  ESCI IRTS CTSC CTS ESCEN RTEC PREN PROE STPB WS RTSEN ATEN TXEN RXEN ~SRST
 *
 */

/* Index of elements of UCR2_3 layout*/
#define UCR2_3_ESCI			14
#define UCR2_3_IRTS			13
#define UCR2_3_CTSC			12
#define UCR2_3_CTS			11
#define UCR2_3_ESCEN		10
#define UCR2_3_RTEC			9
#define UCR2_3_PREN			8
#define UCR2_3_PROE			7
#define UCR2_3_STPB			6
#define UCR2_3_WS			5
#define UCR2_3_RTSEN		4
#define UCR2_3_ATEN			3
#define UCR2_3_TXEN			2
#define UCR2_3_RXEN			1
#define UCR2_3_SRST			0

#define UCR2_3_MAX_INDEX	15
reg_layout_t ucr2_3_layout[UCR2_3_MAX_INDEX]=
{
	{ 0,  0x1  },
	{ 1,  0x2  },
	{ 2,  0x4  },
	{ 3,  0x8  },
	{ 4,  0x10 },
	{ 5,  0x20 },
	{ 6,  0x40 },
	{ 7,  0x80 },
	{ 8,  0x100},
	{ 9,  0x600},
	{ 11, 0x800},
	{ 12, 0x1000},
	{ 13, 0x2000},
	{ 14, 0x4000},
	{ 15, 0x8000}
};

/* UCR3_3: UART3 Control Register 3 -- 0x1000_C088
 *
 * UCR3_3 layout (common part) is:
 * [15:0]
 *  DPEC DTREN PARERREN FRAERREN DSR DCD RI ADNIMP RXDSEN AIRINTEN AWAKEN RXDMUXSEL INVT ACIEN
 *
 */

/* Index of elements of UCR3_3 layout*/
#define UCR3_3_DPEC			13
#define UCR3_3_DTREN		12
#define UCR3_3_PARERREN		11
#define UCR3_3_FRAERREN		10
#define UCR3_3_DSR			9
#define UCR3_3_DCD			8
#define UCR3_3_RI			7
#define UCR3_3_ADNIMP		6
#define UCR3_3_RXDSEN		5
#define UCR3_3_AIRINTEN		4
#define UCR3_3_AWAKEN		3
#define UCR3_3_RXDMUXSEL	2
#define UCR3_3_INVT			1
#define UCR3_3_ACIEN		0

#define UCR3_3_MAX_INDEX	14
reg_layout_t ucr3_3_layout[UCR3_3_MAX_INDEX]=
{
	{ 0,  0x1  },
	{ 1,  0x2  },
	{ 2,  0x4  },
	{ 4,  0x10 },
	{ 5,  0x20 },
	{ 6,  0x40 },
	{ 7,  0x80 },
	{ 8,  0x100},
	{ 9,  0x200},
	{ 10, 0x400},
	{ 11, 0x800},
	{ 12, 0x1000},
	{ 13, 0x2000},
	{ 14, 0xC000}
};

/* UCR4_3: UART3 Control Register 4 -- 0x1000_C08C
 *
 * UCR4_3 layout (common part) is:
 * [15:0]
 *  CTSTL INVR ENIRI WKEN IRSC LPBYP TCEN BKEN OREN DREN
 *
 */

/* Index of elements of UCR4_3 layout*/
#define UCR4_3_CTSTL		9
#define UCR4_3_INVR			8
#define UCR4_3_ENIRI		7
#define UCR4_3_WKEN			6
#define UCR4_3_IRSC			5
#define UCR4_3_LPBYP		4
#define UCR4_3_TCEN			3
#define UCR4_3_BKEN			2
#define UCR4_3_OREN			1
#define UCR4_3_DREN			0

#define UCR4_3_MAX_INDEX	10
reg_layout_t ucr4_3_layout[UCR4_3_MAX_INDEX]=
{
	{ 0,  0x1  },
	{ 1,  0x2  },
	{ 2,  0x4  },
	{ 3,  0x8  },
	{ 4,  0x10 },
	{ 5,  0x20 },
	{ 7,  0x80 },
	{ 8,  0x100},
	{ 9,  0x200},
	{ 10, 0xfc00}
};

/* UFCR_3: UART3 FIFO Control Register -- 0x1000_C090
 *
 * UFCR_3 layout is:
 *  PBA<31:0>
 *	 TXTL RFDIV DCEDTE RXTL
 */
#define UFCR_3_TXTL		3
#define UFCR_3_RFDIV	2
#define UFCR_3_DCEDTE	1
#define UFCR_3_RXTL		0

#define UFCR_3_MAX_INDEX 4
reg_layout_t ufcr_3_layout[UFCR_3_MAX_INDEX]=
{
	{0,  0x3f  },
	{6,  0x40  },
	{7,  0x380 },
	{10, 0xfc00}
};

/* USR1_3: UART3 Status Register 1 -- 0x1000_C094
 *
 * USR1_3 layout is:
 *
 *   PARITYERR RTSS TRDY RTSD ESCF FRAMERR RRDY AGTIM RXDS AIRINT AWAKE
 *
 */

/* Index of elements of USR1_3 layout*/
#define USR1_3_PARITYERR	10
#define USR1_3_RTSS			9
#define USR1_3_TRDY			8
#define USR1_3_RTSD			7
#define USR1_3_ESCF			6
#define USR1_3_FRAMERR		5
#define USR1_3_RRDY			4
#define USR1_3_AGTIM		3
#define USR1_3_RXDS			2
#define USR1_3_AIRINT		1
#define USR1_3_AWAKE		0

#define USR1_3_MAX_INDEX	11
reg_layout_t usr1_3_layout[USR1_3_MAX_INDEX]=
{
	{ 4,  0x10  },
	{ 5,  0x20  },
	{ 6,  0x40  },
	{ 8,  0x100 },
	{ 9,  0x200 },
	{ 10, 0x400 },
	{ 11, 0x800 },
	{ 12, 0x1000},
	{ 13, 0x2000},
	{ 14, 0x4000},
	{ 15, 0x8000}
};

/* USR2_3: UART3 Status Register 2 -- 0x1000_C098
 *
 * USR2_3 layout is:
 *
 *   ADET TXFE DTRF IDLE ACST RIDELT RIIN IRINT WAKE DCDDELT DCDIN RTSF TXDC BRCD ORE RDR
 *
 */

/* Index of elements of USR2_3 layout*/
#define USR2_3_ADET			15
#define USR2_3_TXFE			14
#define USR2_3_DTRF			13
#define USR2_3_IDLE			12
#define USR2_3_ACST			11
#define USR2_3_RIDELT		10
#define USR2_3_RIIN			9
#define USR2_3_IRINT		8
#define USR2_3_WAKE			7
#define USR2_3_DCDDELT		6
#define USR2_3_DCDIN		5
#define USR2_3_RTSF			4
#define USR2_3_TXDC 		3
#define USR2_3_BRCD 		2
#define USR2_3_ORE  		1
#define USR2_3_RDR			0

#define USR2_3_MAX_INDEX	16
reg_layout_t usr2_3_layout[USR2_3_MAX_INDEX]=
{
	{ 0,  0x1   },
	{ 1,  0x2   },
	{ 2,  0x4   },
	{ 3,  0x8   },
	{ 4,  0x10  },
	{ 5,  0x20  },
	{ 6,  0x40  },
	{ 7,  0x80  },
	{ 8,  0x100 },
	{ 9,  0x200 },
	{ 10, 0x400 },
	{ 11, 0x800 },
	{ 12, 0x1000},
	{ 13, 0x2000},
	{ 14, 0x4000},
	{ 15, 0x8000}
};

/* UESC_3: UART3 Escape Character Register -- 0x1000_C09C
 *
 * UESC_3 layout is:
 *
 *   ESC_CHAR
 *
 */
#define UESC_3_ESC_CHAR		0
#define UESC_3_MAX_INDEX	1
reg_layout_t uesc_3_layout[UESC_3_MAX_INDEX]=
{
	{0, 0xff}
};

/* UTIM_3: UART3 Escape Timer Register -- 0x1000_C0A0
 *
 * UTIM_3 layout is:
 *
 *   TIM
 *
 */
#define UTIM_3_TIM			0
#define UTIM_3_MAX_INDEX	1
reg_layout_t utim_3_layout[UTIM_3_MAX_INDEX]=
{
	{0, 0xfff}
};

/* UBIR_3: UART3 BRM Incremental Register -- 0x1000_C0A4
 *
 * UBIR_3 layout is:
 *
 *   INC
 *
 */
#define UBIR_3_INC			0
#define UBIR_3_MAX_INDEX	1
reg_layout_t ubir_3_layout[UBIR_3_MAX_INDEX]=
{
	{0, 0xffff}
};

/* UBMR_3: UART3 BRM Modulator Register -- 0x1000_C0A8
 *
 * UBMR_3 layout is:
 *
 *   MOD
 *
 */

#define UBMR_3_MOD			0
#define UBMR_3_MAX_INDEX	1
reg_layout_t ubmr_3_layout[UBMR_3_MAX_INDEX]=
{
	{0, 0xffff}
};

/* UBRC_3: UART3 Baud Rate Count Register -- 0x1000_C0AC
 *
 * UBRC_3 layout is:
 *
 *   BCNT
 *
 */

#define UBRC_3_INC			0
#define UBRC_3_MAX_INDEX	1
reg_layout_t ubrc_3_layout[UBRC_3_MAX_INDEX]=
{
	{0, 0xffff}
};

/* ONEMS_3: UART3 One Millisecond Register -- 0x1000_C0B0
 *
 * ONEMS_3 layout is:
 *
 *   ONEMS
 *
 */

#define ONEMS_3_INC			0
#define ONEMS_3_MAX_INDEX	1
reg_layout_t onems_3_layout[ONEMS_3_MAX_INDEX]=
{
	{0, 0xffff}
};

/*************************************************

				FIR Memory Map

*************************************************/

/* FIRICR: FIRI Control Register -- 0x1002_8028
 *
 * FIRICR layout is:
 *
 *   BL OSF
 *
 */

/* Index of elements of FIRICR layout*/
#define FIRICR_BL		1
#define FIRICR_OSF		0

#define FIRICR_MAX_INDEX	2
reg_layout_t firicr_layout[FIRICR_MAX_INDEX]=
{
	{ 0,  0xf   },
	{ 5,  0xfe0 }
};

 /* FIRITCR: FIRI Transmitter Control Register -- 0x1002_8000
 *
 * FIRITCR layout is:
 *
 *   HAG TPA SRF TDT TCIE TPEIE TFUIE PCF PC SIP TPP TM TE
 *
 */

/* Index of elements of FIRITCR layout*/
#define FIRITCR_HAG		12
#define FIRITCR_TPA		11
#define FIRITCR_SRF		10
#define FIRITCR_TDT		9
#define FIRITCR_TCIE	8
#define FIRITCR_TPEIE	7
#define FIRITCR_TFUIE	6
#define FIRITCR_PCF		5
#define FIRITCR_PC		4
#define FIRITCR_SIP		3
#define FIRITCR_TPP		2
#define FIRITCR_TM		1
#define FIRITCR_TE		0


#define FIRITCR_MAX_INDEX	13
reg_layout_t firitcr_layout[FIRITCR_MAX_INDEX]=
{
	{ 0,  0x1		 },
	{ 1,  0x6		 },
	{ 3,  0X8		 },
	{ 4,  0X10		 },
	{ 5,  0x20		 },
	{ 6,  0x40		 },
	{ 7,  0X80		 },
	{ 8,  0X100		 },
	{ 9,  0X200 	 },
	{ 10,  0x1c00	 },
	{ 13,  0x6000    },
	{ 16,  0Xff0000  },
	{ 24,  0X1000000 },
};

/* FIRITCTR: FIRI Transmitter Count Register  -- 0x1002_8004
 *
 * FIRITCTR layout is:
 *
 *   TPL
 *
 */

/* Index of elements of FIRITCTR layout*/
#define FIRITCTR_TPL	0

#define FIRITCTR_MAX_INDEX	1
reg_layout_t firitctr_layout[FIRITCTR_MAX_INDEX]=
{
	{ 0,  0x7ff }
};

/* FIRIRCR: FIRI Receiver Control Register  -- 0x1002_8008
 *
 * FIRIRCR layout is:
 *
 *   RAM RA RPEDE RDT RPA RPEIE PAIE RFOIE RPP RM RE
 *
 */

/* Index of elements of FIRIRCR layout*/
#define FIRIRCR_RAM		10
#define FIRIRCR_RA		9
#define FIRIRCR_RPEDE	8
#define FIRIRCR_RDT		7
#define FIRIRCR_RPA		6
#define FIRIRCR_RPEIE	5
#define FIRIRCR_PAIE	4
#define FIRIRCR_RFOIE	3
#define FIRIRCR_RPP		2
#define FIRIRCR_RM		1
#define FIRIRCR_RE		0

#define FIRIRCR_MAX_INDEX	11
reg_layout_t firircr_layout[FIRIRCR_MAX_INDEX]=
{
	{ 0,  0x1		 },
	{ 1,  0x6		 },
	{ 3,  0x8		 },
	{ 4,  0x10		 },
	{ 5,  0x20		 },
	{ 6,  0x40		 },
	{ 7,  0x80		 },
	{ 8,  0x700		 },
	{ 11,  0x800 	 },
	{ 16,  0xFF0000  },
	{ 24,  0x3000000 }
};

/*  FIRITSR: FIRI Transmit Status Register  -- 0x1002_800C
 *
 * FIRITSR layout is:
 *
 *   TC SIPE TPE TFU
 *
 */

/* Index of elements of FIRITSR layout*/
#define FIRITSR_TC		3
#define FIRITSR_SIPE	2
#define FIRITSR_TPE		1
#define FIRITSR_TFU		0

#define FIRITSR_MAX_INDEX	4
reg_layout_t firitsr_layout[FIRITSR_MAX_INDEX]=
{
	{ 0,  0x1 },
	{ 1,  0x2 },
	{ 2,  0x4 },
	{ 3,  0x8 }
};

/*  FIRIRSR: FIRI Receive Status Register  -- 0x1002_8010
 *
 * FIRIRSR layout is:
 *
 *   RFP PSA RPE RFO BAM CRCE DDE
 *
 */

/* Index of elements of FIRIRSR layout*/
#define FIRIRSR_RFP		6
#define FIRIRSR_PSA		5
#define FIRIRSR_RPE		4
#define FIRIRSR_RFO		3
#define FIRIRSR_BAM		2
#define FIRIRSR_CRCE	1
#define FIRIRSR_DDE		0

#define FIRIRSR_MAX_INDEX	7
reg_layout_t firirsr_layout[FIRIRSR_MAX_INDEX]=
{
	{ 0,  0x1	 },
	{ 1,  0x2	 },
	{ 2,  0x4	 },
	{ 3,  0x8	 },
	{ 4,  0x10 	 },
	{ 5,  0x20 	 },
	{ 6,  0xFF00 }
};

/*  FIRIRFIFO: FIRI Receive FIFO  -- 0x1002_8018
 *
 * FIRIRFIFO layout is:
 *
 *   FIFO
 *
 */

/* Index of elements of FIRIRFIFO layout*/
#define FIRIRFIFO_FIFO		0

#define FIRIRFIFO_MAX_INDEX	1
reg_layout_t firirfifo_layout[FIRIRFIFO_MAX_INDEX]=
{
	{ 0,  0xffffffff }
};

/*  FIRITFIFO: FIRI Transmitter FIFO  -- 0x1002_8014
 *
 * FIRITFIFO layout is:
 *
 *   FIFO
 *
 */

/* Index of elements of FIRITFIFO layout*/
#define FIRITFIFO_FIFO		0

#define FIRITFIFO_MAX_INDEX	1
reg_layout_t firitfifo_layout[FIRITFIFO_MAX_INDEX]=
{
	{ 0,  0xffffffff }
};
#define reg_element_set(reg, shift, mask, value) \
	( (reg) = ((reg)&(~(mask))) | (((value)<<(shift))&(mask)) )

#define reg_element_get(reg, shift, mask) \
	( ((reg)&(mask))>>(shift) )

#define urxd_3_read(element) reg_element_get(UART_URXD(UART_3), urxd_3_layout[element].shift, \
	urxd_3_layout[element].mask)

#define urxd_3_write(element,value) reg_element_set(UART_URXD(UART_3), urxd_3_layout[element].shift, \
	urxd_3_layout[element].mask,value)

#define utxd_3_read(element) reg_element_get(UART_UTXD(UART_3), utxd_3_layout[element].shift, \
	utxd_3_layout[element].mask)

#define utxd_3_write(element,value) reg_element_set(UART_UTXD(UART_3), utxd_3_layout[element].shift, \
	utxd_3_layout[element].mask,value)

#define ucr1_3_read(element) reg_element_get(UART_UCR1(UART_3), ucr1_3_layout[element].shift, \
	ucr1_3_layout[element].mask)

#define ucr1_3_write(element,value) reg_element_set(UART_UCR1(UART_3), ucr1_3_layout[element].shift, \
	ucr1_3_layout[element].mask,value)

#define ucr2_3_read(element) reg_element_get(UART_UCR2(UART_3), ucr2_3_layout[element].shift, \
	ucr2_3_layout[element].mask)

#define ucr2_3_write(element,value) reg_element_set(UART_UCR2(UART_3), ucr2_3_layout[element].shift, \
	ucr2_3_layout[element].mask,value)

#define ucr3_3_read(element) reg_element_get(UART_UCR3(UART_3), ucr3_3_layout[element].shift, \
	ucr3_3_layout[element].mask)

#define ucr3_3_write(element,value) reg_element_set(UART_UCR3(UART_3), ucr3_3_layout[element].shift, \
	ucr3_3_layout[element].mask,value)

#define ucr4_3_read(element) reg_element_get(UART_UCR4(UART_3), ucr4_3_layout[element].shift, \
	ucr4_3_layout[element].mask)

#define ucr4_3_write(element,value) reg_element_set(UART_UCR4(UART_3), ucr4_3_layout[element].shift, \
	ucr4_3_layout[element].mask,value)

#define ufcr_3_read(element) reg_element_get(UART_UFCR(UART_3), ufcr_3_layout[element].shift, \
	ufcr_3_layout[element].mask)

#define ufcr_3_write(element,value) reg_element_set(UART_UFCR(UART_3), ufcr_3_layout[element].shift, \
	ufcr_3_layout[element].mask,value)

#define usr1_3_read(element) reg_element_get(UART_USR1(UART_3), usr1_3_layout[element].shift, \
	usr1_3_layout[element].mask)

#define usr1_3_write(element,value) reg_element_set(UART_USR1(UART_3), usr1_3_layout[element].shift, \
	usr1_3_layout[element].mask,value)

#define usr2_3_read(element) reg_element_get(UART_USR2(UART_3), usr2_3_layout[element].shift, \
	usr2_3_layout[element].mask)

#define usr2_3_write(element,value) reg_element_set(UART_USR2(UART_3), usr2_3_layout[element].shift, \
	usr2_3_layout[element].mask,value)

#define uesc_3_read(element) reg_element_get(UART_UESC(UART_3), uesc_3_layout[element].shift, \
	uesc_3_layout[element].mask)

#define uesc_3_write(element,value) reg_element_set(UART_UESC(UART_3), uesc_3_layout[element].shift, \
	uesc_3_layout[element].mask,value)

#define utim_3_read(element) reg_element_get(UART_UTIM(UART_3), utim_3_layout[element].shift, \
	utim_3_layout[element].mask)

#define utim_3_write(element,value) reg_element_set(UART_UTIM(UART_3), utim_3_layout[element].shift, \
	utim_3_layout[element].mask,value)

#define ubir_3_read(element) reg_element_get(UART_UBIR(UART_3), ubir_3_layout[element].shift, \
	ubir_3_layout[element].mask)

#define ubir_3_write(element,value) reg_element_set(UART_UBIR(UART_3), ubir_3_layout[element].shift, \
	ubir_3_layout[element].mask,value)

#define ubmr_3_read(element) reg_element_get(UART_UBMR(UART_3), ubmr_3_layout[element].shift, \
	ubmr_3_layout[element].mask)

#define ubmr_3_write(element,value) reg_element_set(UART_UBMR(UART_3), ubmr_3_layout[element].shift, \
	ubmr_3_layout[element].mask,value)

#define ubrc_3_read(element) reg_element_get(UART_UBRC(UART_3), ubrc_3_layout[element].shift, \
	ubrc_3_layout[element].mask)

#define ubrc_3_write(element,value) reg_element_set(UART_UBRC(UART_3), ubrc_3_layout[element].shift, \
	ubrc_3_layout[element].mask,value)

#define onems_3_read(element) reg_element_get(UART_ONEMS(UART_3), onems_3_layout[element].shift, \
	onems_3_layout[element].mask)

#define onems_3_write(element,value) reg_element_set(UART_ONEMS(UART_3), onems_3_layout[element].shift, \
	onems_3_layout[element].mask,value)

#define firicr_read(element) reg_element_get(FIRI_FIRICR, firicr_layout[element].shift, \
	firicr_layout[element].mask)

#define firicr_write(element,value) reg_element_set(FIRI_FIRICR, firicr_layout[element].shift, \
	firicr_layout[element].mask,value)

#define firitcr_read(element) reg_element_get(FIRI_FIRITCR, firitcr_layout[element].shift, \
	firitcr_layout[element].mask)

#define firitcr_write(element,value) reg_element_set(FIRI_FIRITCR, firitcr_layout[element].shift, \
	firitcr_layout[element].mask,value)

#define firitctr_read(element) reg_element_get(FIRI_FIRITCTR, firitctr_layout[element].shift, \
	firitctr_layout[element].mask)

#define firitctr_write(element,value) reg_element_set(FIRI_FIRITCTR, firitctr_layout[element].shift, \
	firitctr_layout[element].mask,value)

#define firircr_read(element) reg_element_get(FIRI_FIRIRCR, firircr_layout[element].shift, \
	firircr_layout[element].mask)

#define firircr_write(element,value) reg_element_set(FIRI_FIRIRCR, firircr_layout[element].shift, \
	firircr_layout[element].mask,value)

#define firitsr_read(element) reg_element_get(FIRI_FIRITSR, firitsr_layout[element].shift, \
	firitsr_layout[element].mask)

#define firitsr_write(element,value) reg_element_set(FIRI_FIRITSR, firitsr_layout[element].shift, \
	firitsr_layout[element].mask,value)

#define firirsr_read(element) reg_element_get(FIRI_FIRIRSR, firirsr_layout[element].shift, \
	firirsr_layout[element].mask)

#define firirsr_write(element,value) reg_element_set(FIRI_FIRIRSR, firirsr_layout[element].shift, \
	firirsr_layout[element].mask,value)

#define firitfifo_read(FIRITFIFO_FIFO) FIRI_TFIFO

#define firitfifo_write(FIRITFIFO_FIFO) FIRI_TFIFO

#define firirfifo_read(FIRIRFIFO_FIFO) FIRI_RFIFO

#define firirfifo_write(FIRIRFIFO_FIFO) FIRI_RFIFO

#endif
