/* $Id: xemac_intr_fifo.c,v 1.3 2002/05/02 20:52:10 moleres Exp $ */
/******************************************************************************
*
*     Author: Xilinx, Inc.
*     
*     
*     This program is free software; you can redistribute it and/or modify it
*     under the terms of the GNU General Public License as published by the
*     Free Software Foundation; either version 2 of the License, or (at your
*     option) any later version.
*     
*     
*     XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
*     COURTESY TO YOU. BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
*     ONE POSSIBLE IMPLEMENTATION OF THIS FEATURE, APPLICATION OR STANDARD,
*     XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION IS FREE
*     FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE FOR
*     OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
*     XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
*     THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO ANY
*     WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE FROM
*     CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND
*     FITNESS FOR A PARTICULAR PURPOSE.
*     
*     
*     Xilinx products are not intended for use in life support appliances,
*     devices, or systems. Use in such applications is expressly prohibited.
*     
*     
*     (c) Copyright 2002 Xilinx Inc.
*     All rights reserved.
*     
*     
*     You should have received a copy of the GNU General Public License along
*     with this program; if not, write to the Free Software Foundation, Inc.,
*     675 Mass Ave, Cambridge, MA 02139, USA.
*
******************************************************************************/
/*****************************************************************************/
/**
*
* @file xemac_intr_fifo.c
*
* Contains functions related to interrupt mode using direct FIFO communication.
*
* The interrupt handler, XEmac_IntrHandlerFifo(), must be connected by the user
* to the interrupt controller.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a rpm  07/31/01 First release
* 1.00b rpm  02/20/02 Repartitioned files and functions
* </pre>
*
******************************************************************************/

/***************************** Include Files *********************************/

#include "xbasic_types.h"
#include "xemac_i.h"
#include "xio.h"
#include "xipif_v1_23_b.h"	/* Uses v1.23b of the IPIF */

/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Variable Definitions *****************************/

/************************** Function Prototypes ******************************/

static void HandleEmacFifoIntr(XEmac * InstancePtr);

/*****************************************************************************/
/**
*
* Send an Ethernet frame using packet FIFO with interrupts. The caller provides
* a contiguous-memory buffer and its length. The buffer must be word-aligned.
* The callback function set by using SetFifoSendHandler is invoked when the
* transmission is complete.
*
* It is assumed that the upper layer software supplies a correctly formatted
* Ethernet frame, including the destination and source addresses, the
* type/length field, and the data field.
*
* This function can only be used when the device is not configured with DMA.
*
* @param InstancePtr is a pointer to the XEmac instance to be worked on.
* @param BufPtr is a pointer to a word-aligned buffer containing the Ethernet
*        frame to be sent.
* @param ByteCount is the size of the Ethernet frame.
*
* @return
*
* - XST_SUCCESS if the frame was successfully written to the EMAC's packet
*   FIFO. An interrupt is generated when the EMAC transmits the frame and the
*   driver calls the callback set with XEmac_SetFifoSendHandler()
* - XST_DEVICE_IS_STOPPED  if the device has not yet been started
* - XST_NOT_INTERRUPT if the device is not in interrupt mode
* - XST_NO_FEATURE if the device has DMA, so direct I/O to FIFOs is not allowed
* - XST_FIFO_NO_ROOM if there is no room in the FIFO for this frame
*
* @note
*
* This function is not thread-safe. The user must provide mutually exclusive
* access to this function if there are to be multiple threads that can call it.
*
* @internal
*
* The Ethernet MAC uses FIFOs behind its length and status registers. For this
* reason, it is important to keep the length, status, and data FIFOs in sync
* when reading or writing to them.
*
******************************************************************************/
XStatus
XEmac_FifoSend(XEmac * InstancePtr, u8 * BufPtr, u32 ByteCount)
{
	XStatus Result;
	u32 IntrStatus;

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(BufPtr != NULL);
	XASSERT_NONVOID(ByteCount > XEM_HDR_SIZE);	/* send at least 1 byte */
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/*
	 * Be sure the device is configured for interrupt mode and it is started
	 */
	if (InstancePtr->IsPolled) {
		return XST_NOT_INTERRUPT;
	}

	if (InstancePtr->IsDmaSg) {
		return XST_NO_FEATURE;	/* TODO: do we force them to use DMA if it exists? */
	}

	if (InstancePtr->IsStarted != XCOMPONENT_IS_STARTED) {
		return XST_DEVICE_IS_STOPPED;
	}

	/*
	 * Before writing to the data FIFO, make sure the length FIFO is not
	 * full.  The data FIFO might not be full yet even though the length FIFO
	 * is. This avoids an overrun condition on the length FIFO and keeps the
	 * FIFOs in sync.
	 */
	IntrStatus = XIIF_V123B_READ_IISR(InstancePtr->BaseAddress);
	if (IntrStatus & XEM_EIR_XMIT_LFIFO_FULL_MASK) {
		return XST_FIFO_NO_ROOM;
	}

	/*
	 * This is a non-blocking write. The packet FIFO returns an error if there
	 * is not enough room in the FIFO for this frame.
	 */
	Result =
	    XPacketFifoV100b_Write(&InstancePtr->SendFifo, BufPtr, ByteCount);
	if (Result != XST_SUCCESS) {
		return Result;
	}

	/*
	 * Set the MAC's transmit packet length register to tell it to transmit
	 */
	XIo_Out32(InstancePtr->BaseAddress + XEM_TPLR_OFFSET, ByteCount);

	/*
	 * Bump stats here instead of the Isr since we know the byte count
	 * here but would have to save it in the instance in order to know the
	 * byte count at interrupt time.
	 */
	InstancePtr->Stats.XmitFrames++;
	InstancePtr->Stats.XmitBytes += ByteCount;

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* Receive an Ethernet frame into the buffer passed as an argument. This
* function is called in response to the callback function for received frames
* being called by the driver. The callback function is set up using
* SetFifoRecvHandler, and is invoked when the driver receives an interrupt
* indicating a received frame. The driver expects the upper layer software to
* call this function, FifoRecv, to receive the frame. The buffer supplied
* should be large enough to hold a maximum-size Ethernet frame.
*
* The buffer into which the frame will be received must be word-aligned.
*
* This function can only be used when the device is not configured with DMA.
*
* @param InstancePtr is a pointer to the XEmac instance to be worked on.
* @param BufPtr is a pointer to a word-aligned buffer into which the received
*        Ethernet frame will be copied.
* @param ByteCountPtr is both an input and an output parameter. It is a pointer
*        to a 32-bit word that contains the size of the buffer on entry into
*        the function and the size the received frame on return from the
*        function.
*
* @return
*
* - XST_SUCCESS if the frame was sent successfully
* - XST_DEVICE_IS_STOPPED if the device has not yet been started
* - XST_NOT_INTERRUPT if the device is not in interrupt mode
* - XST_NO_FEATURE if the device has DMA, so direct I/O to FIFOs is not allowed
* - XST_NO_DATA if there is no frame to be received from the FIFO
* - XST_BUFFER_TOO_SMALL if the buffer to receive the frame is too small for
*   the frame waiting in the FIFO.
*
* @note
*
* The input buffer must be big enough to hold the largest Ethernet frame.
*
* @internal
*
* The Ethernet MAC uses FIFOs behind its length and status registers. For this
* reason, it is important to keep the length, status, and data FIFOs in sync
* when reading or writing to them.
*
******************************************************************************/
XStatus
XEmac_FifoRecv(XEmac * InstancePtr, u8 * BufPtr, u32 * ByteCountPtr)
{
	XStatus Result;
	u32 PktLength;
	u32 IntrStatus;

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(BufPtr != NULL);
	XASSERT_NONVOID(ByteCountPtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	/*
	 * Be sure the device is not configured for polled mode and it is started
	 */
	if (InstancePtr->IsPolled) {
		return XST_NOT_INTERRUPT;
	}

	if (InstancePtr->IsDmaSg) {
		return XST_NO_FEATURE;	/* TODO: do we force them to use DMA if it exists? */
	}

	if (InstancePtr->IsStarted != XCOMPONENT_IS_STARTED) {
		return XST_DEVICE_IS_STOPPED;
	}

	/*
	 * Make sure the buffer is big enough to hold the maximum frame size.
	 * We need to do this because as soon as we read the MAC's packet length
	 * register, which is actually a FIFO, we remove that length from the
	 * FIFO.  We do not want to read the length FIFO without also reading the
	 * data FIFO since this would get the FIFOs out of sync.  So we have to
	 * make this restriction.
	 */
	if (*ByteCountPtr < XEM_MAX_FRAME_SIZE) {
		return XST_BUFFER_TOO_SMALL;
	}

	/*
	 * Before reading from the length FIFO, make sure the length FIFO is not
	 * empty. We could cause an underrun error if we try to read from an
	 * empty FIFO.
	 */
	IntrStatus = XIIF_V123B_READ_IISR(InstancePtr->BaseAddress);
	if (IntrStatus & XEM_EIR_RECV_LFIFO_EMPTY_MASK) {
		/*
		 * Clear the empty status so the next time through the current status
		 * of the hardware is reflected (we have to do this because the status
		 * is level in the device but latched in the interrupt status register).
		 */
		XIIF_V123B_WRITE_IISR(InstancePtr->BaseAddress,
				      XEM_EIR_RECV_LFIFO_EMPTY_MASK);
		return XST_NO_DATA;
	}

	/*
	 * Determine, from the MAC, the length of the next packet available
	 * in the data FIFO (there should be a non-zero length here)
	 */
	PktLength = XIo_In32(InstancePtr->BaseAddress + XEM_RPLR_OFFSET);
	if (PktLength == 0) {
		return XST_NO_DATA;
	}

	/*
	 * We assume that the MAC never has a length bigger than the largest
	 * Ethernet frame, so no need to make another check here.
	 */

	/*
	 * This is a non-blocking read. The FIFO returns an error if there is
	 * not at least the requested amount of data in the FIFO.
	 */
	Result =
	    XPacketFifoV100b_Read(&InstancePtr->RecvFifo, BufPtr, PktLength);
	if (Result != XST_SUCCESS) {
		return Result;
	}

	*ByteCountPtr = PktLength;

	InstancePtr->Stats.RecvFrames++;
	InstancePtr->Stats.RecvBytes += PktLength;

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* The interrupt handler for the Ethernet driver when configured for direct FIFO
* communication (as opposed to DMA).
*
* Get the interrupt status from the IpIf to determine the source of the
* interrupt.  The source can be: MAC, Recv Packet FIFO, or Send Packet FIFO.
* The packet FIFOs only interrupt during "deadlock" conditions.  All other
* FIFO-related interrupts are generated by the MAC.
*
* @param InstancePtr is a pointer to the XEmac instance that just interrupted.
*
* @return
*
* None.
*
* @note
*
* None.
*
******************************************************************************/
void
XEmac_IntrHandlerFifo(void *InstancePtr)
{
	u32 IntrStatus;
	XEmac *EmacPtr = (XEmac *) InstancePtr;

	EmacPtr->Stats.TotalIntrs++;

	/*
	 * Get the interrupt status from the IPIF. There is no clearing of
	 * interrupts in the IPIF. Interrupts must be cleared at the source.
	 */
	IntrStatus = XIIF_V123B_READ_DIPR(EmacPtr->BaseAddress);

	if (IntrStatus & XEM_IPIF_EMAC_MASK) {	/* MAC interrupt */
		EmacPtr->Stats.EmacInterrupts++;
		HandleEmacFifoIntr(EmacPtr);
	}

	if (IntrStatus & XEM_IPIF_RECV_FIFO_MASK) {	/* Receive FIFO interrupt */
		EmacPtr->Stats.RecvInterrupts++;
		XEmac_CheckFifoRecvError(EmacPtr);
	}

	if (IntrStatus & XEM_IPIF_SEND_FIFO_MASK) {	/* Send FIFO interrupt */
		EmacPtr->Stats.XmitInterrupts++;
		XEmac_CheckFifoSendError(EmacPtr);
	}

	if (IntrStatus & XIIF_V123B_ERROR_MASK) {
		/*
		 * An error occurred internal to the IPIF. This is more of a debug and
		 * integration issue rather than a production error. Don't do anything
		 * other than clear it, which provides a spot for software to trap
		 * on the interrupt and begin debugging.
		 */
		XIIF_V123B_WRITE_DISR(EmacPtr->BaseAddress,
				      XIIF_V123B_ERROR_MASK);
	}
}

/*****************************************************************************/
/**
*
* Set the callback function for handling confirmation of transmitted frames when
* configured for direct memory-mapped I/O using FIFOs. The upper layer software
* should call this function during initialization. The callback is called by the
* driver once per frame sent. The callback is responsible for freeing the
* transmitted buffer if necessary.
*
* The callback is invoked by the driver within interrupt context, so it needs
* to do its job quickly. If there are potentially slow operations within the
* callback, these should be done at task-level.
*
* @param InstancePtr is a pointer to the XEmac instance to be worked on.
* @param CallBackRef is a reference pointer to be passed back to the adapter in
*        the callback. This helps the adapter correlate the callback to a
*        particular driver.
* @param FuncPtr is the pointer to the callback function.
*
* @return
*
* None.
*
* @note
*
* None.
*
******************************************************************************/
void
XEmac_SetFifoRecvHandler(XEmac * InstancePtr, void *CallBackRef,
			 XEmac_FifoHandler FuncPtr)
{
	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(FuncPtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	InstancePtr->FifoRecvHandler = FuncPtr;
	InstancePtr->FifoRecvRef = CallBackRef;
}

/*****************************************************************************/
/**
*
* Set the callback function for handling received frames when configured for
* direct memory-mapped I/O using FIFOs. The upper layer software should call
* this function during initialization. The callback is called once per frame
* received. During the callback, the upper layer software should call FifoRecv
* to retrieve the received frame.
*
* The callback is invoked by the driver within interrupt context, so it needs
* to do its job quickly. Sending the received frame up the protocol stack
* should be done at task-level. If there are other potentially slow operations
* within the callback, these too should be done at task-level.
*
* @param InstancePtr is a pointer to the XEmac instance to be worked on.
* @param CallBackRef is a reference pointer to be passed back to the adapter in
*        the callback. This helps the adapter correlate the callback to a
*        particular driver.
* @param FuncPtr is the pointer to the callback function.
*
* @return
*
* None.
*
* @note
*
* None.
*
******************************************************************************/
void
XEmac_SetFifoSendHandler(XEmac * InstancePtr, void *CallBackRef,
			 XEmac_FifoHandler FuncPtr)
{
	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(FuncPtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	InstancePtr->FifoSendHandler = FuncPtr;
	InstancePtr->FifoSendRef = CallBackRef;
}

/******************************************************************************
*
* Handle an interrupt from the Ethernet MAC when configured for direct FIFO
* communication.  The interrupts handled are:
* - Transmit done (transmit status FIFO is non-empty). Used to determine when
*   a transmission has been completed.
* - Receive done (receive length FIFO is non-empty). Used to determine when a
*   valid frame has been received.
*
* In addition, the interrupt status is checked for errors.
*
* @param InstancePtr is a pointer to the XEmac instance to be worked on.
*
* @return
*
* None.
*
* @note
*
* None.
*
******************************************************************************/
static void
HandleEmacFifoIntr(XEmac * InstancePtr)
{
	u32 IntrStatus;

	/*
	 * The EMAC generates interrupts for errors and generates the transmit
	 * and receive done interrupts for data. We clear the interrupts
	 * immediately so that any latched status interrupt bits will reflect the
	 * true status of the device, and so any pulsed interrupts (non-status)
	 * generated during the Isr will not be lost.
	 */
	IntrStatus = XIIF_V123B_READ_IISR(InstancePtr->BaseAddress);
	XIIF_V123B_WRITE_IISR(InstancePtr->BaseAddress, IntrStatus);

	if (IntrStatus & XEM_EIR_RECV_DONE_MASK) {
		/*
		 * Configured for direct memory-mapped I/O using FIFO with interrupts.
		 * This interrupt means the RPLR is non-empty, indicating a frame has
		 * arrived.
		 */
		InstancePtr->Stats.RecvInterrupts++;

		InstancePtr->FifoRecvHandler(InstancePtr->FifoRecvRef);

		/*
		 * The upper layer has removed as many frames as it wants to, so we
		 * need to clear the RECV_DONE bit before leaving the ISR so that it
		 * reflects the current state of the hardware (because it's a level
		 * interrupt that is latched in the IPIF interrupt status register).
		 * Note that if we've reached this point the bit is guaranteed to be
		 * set because it was cleared at the top of this ISR before any frames
		 * were serviced, so the bit was set again immediately by hardware
		 * because the RPLR was not yet emptied by software.
		 */
		XIIF_V123B_WRITE_IISR(InstancePtr->BaseAddress,
				      XEM_EIR_RECV_DONE_MASK);
	}

	/*
	 * If configured for direct memory-mapped I/O using FIFO, the xmit status
	 * FIFO must be read and the callback invoked regardless of success or not.
	 */
	if (IntrStatus & XEM_EIR_XMIT_DONE_MASK) {
		u32 XmitStatus;

		InstancePtr->Stats.XmitInterrupts++;

		XmitStatus =
		    XIo_In32(InstancePtr->BaseAddress + XEM_TSR_OFFSET);

		/*
		 * Collision errors are stored in the transmit status register
		 * instead of the interrupt status register
		 */
		if (XmitStatus & XEM_TSR_EXCESS_DEFERRAL_MASK) {
			InstancePtr->Stats.XmitExcessDeferral++;
		}

		if (XmitStatus & XEM_TSR_LATE_COLLISION_MASK) {
			InstancePtr->Stats.XmitLateCollisionErrors++;
		}

		InstancePtr->FifoSendHandler(InstancePtr->FifoSendRef);

		/*
		 * Only one status is retrieved per interrupt. We need to clear the
		 * XMIT_DONE bit before leaving the ISR so that it reflects the current
		 * state of the hardware (because it's a level interrupt that is latched
		 * in the IPIF interrupt status register). Note that if we've reached
		 * this point the bit is guaranteed to be set because it was cleared at
		 * the top of this ISR before any statuses were serviced, so the bit was
		 * set again immediately by hardware because the TSR was not yet emptied
		 * by software.
		 */
		XIIF_V123B_WRITE_IISR(InstancePtr->BaseAddress,
				      XEM_EIR_XMIT_DONE_MASK);
	}

	/*
	 * Check the MAC for errors
	 */
	XEmac_CheckEmacError(InstancePtr, IntrStatus);
}
