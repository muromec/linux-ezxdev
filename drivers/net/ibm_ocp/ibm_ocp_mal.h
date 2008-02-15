/*
 * 06/02/02 -Armin
 *    added new mal functions and defines from ibm_ocp_enet.h
 */

#ifndef _IBM_OCP_MAL_H
#define _IBM_OCP_MAL_H

#define GET_MAL_STANZA(base,dcrn) \
	case base: \
		x = mfdcr(dcrn(base)); \
		break;

#define SET_MAL_STANZA(base,dcrn, val) \
	case base: \
		mtdcr(dcrn(base), (val)); \
		break;

#define GET_MAL0_STANZA(dcrn) GET_MAL_STANZA(DCRN_MAL_BASE,dcrn)
#define SET_MAL0_STANZA(dcrn,val) SET_MAL_STANZA(DCRN_MAL_BASE,dcrn,val)

#ifdef DCRN_MAL1_BASE
#define GET_MAL1_STANZA(dcrn) GET_MAL_STANZA(DCRN_MAL1_BASE,dcrn)
#define SET_MAL1_STANZA(dcrn,val) SET_MAL_STANZA(DCRN_MAL1_BASE,dcrn,val)
#else /* ! DCRN_MAL1_BASE */
#define GET_MAL1_STANZA(dcrn) 
#define SET_MAL1_STANZA(dcrn,val)
#endif


#define get_mal_dcrn(fep, dcrn) ({ \
	u32 x; \
	switch ((fep)->mal) { \
		GET_MAL0_STANZA(dcrn) \
		GET_MAL1_STANZA(dcrn) \
	default: \
		BUG(); \
	} \
x; })

#define set_mal_dcrn(fep, dcrn, val) do { \
	switch ((fep)->mal) { \
		SET_MAL0_STANZA(dcrn,val) \
		SET_MAL1_STANZA(dcrn,val) \
	default: \
		BUG(); \
	} } while (0)


extern void config_mal(struct ocp_enet_private *fep);
extern void disable_mal_chan(struct ocp_enet_private *fep);
extern void enable_mal_chan(struct ocp_enet_private *fep);
extern void set_mal_chan_addr(struct ocp_enet_private *fep);


/* the following defines are for the MadMAL status and control registers. */
/* MADMAL transmit and receive status/control bits  */
#define MAL_RX_CTRL_EMPTY		0x8000
#define MAL_RX_CTRL_WRAP		0x4000
#define MAL_RX_CTRL_CM			0x2000
#define MAL_RX_CTRL_LAST		0x1000
#define MAL_RX_CTRL_FIRST		0x0800
#define MAL_RX_CTRL_INTR		0x0400

#define MAL_TX_CTRL_READY		0x8000
#define MAL_TX_CTRL_WRAP		0x4000
#define MAL_TX_CTRL_CM			0x2000
#define MAL_TX_CTRL_LAST		0x1000
#define MAL_TX_CTRL_INTR		0x0400


#endif /* _IBM_OCP_MAL_H */
