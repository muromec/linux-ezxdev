/*
 * File: fpga.h
 *
 * FPGA read/write prototypes
 *
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: RidgeRun, Inc.  (stevej@ridgerun.com)
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Please report all bugs/problems to the author or <support@dsplinux.net>
 *
 */

#ifndef __fpga_h
#define __fpga_h

void fpga_write(unsigned char val, int reg);
unsigned char fpga_read(int reg);

// Power up Giga UART driver, turn on HID clock.
// Turn off BT power, since we're not using it and it
// draws power.
#define FPGA_RESET_VALUE 0x42

#define FPGA_PCR_IF_PD0         (1 << 7)
#define FPGA_PCR_COM2_EN        (1 << 6)
#define FPGA_PCR_COM1_EN        (1 << 5)
#define FPGA_PCR_EXP_PD0        (1 << 4)
#define FPGA_PCR_EXP_PD1        (1 << 3)
#define FPGA_PCR_48MHZ_CLK      (1 << 2)
#define FPGA_PCR_4MHZ_CLK       (1 << 1)
#define FPGA_PCR_RSRVD_BIT0     (1 << 0)

#endif

