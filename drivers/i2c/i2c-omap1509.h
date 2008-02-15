/*
 * File: i2c-omap1509.h
 *
 * I2C registers/values for OMAP1509 
 *
 * Copyright (C) 2002 RidgeRun, Inc.
 * Author: Steve Johnson
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
 * key: RRGPLCR (do not remove)
 *
 */

#ifndef i2c_omap1509_h
#define i2c_omap1509_h

/*
  I2C Registers
*/
#define I2C_BASE                  (0xfffb3800)
#define I2C_DEVICE_REG            (I2C_BASE + 0x0)    
#define I2C_ADDRESS_REG           (I2C_BASE + 0x4)    
#define I2C_DATA_WRITE_REG        (I2C_BASE + 0x8)    
#define I2C_DATA_READ_REG         (I2C_BASE + 0xc)    
#define I2C_CMD_REG               (I2C_BASE + 0x10)    
#define I2C_CONF_FIFO_REG         (I2C_BASE + 0x14)    
#define I2C_CONF_CLK_REG          (I2C_BASE + 0x18)    
#define I2C_CONF_CLK_REF_REG      (I2C_BASE + 0x1c)    
#define I2C_STATUS_FIFO_REG       (I2C_BASE + 0x20)    
#define I2C_STATUS_ACTIVITY_REG   (I2C_BASE + 0x24)    

/*
  Command Register
*/

#define I2C_SOFT_RESET (1 << 0)
#define I2C_EN_CLK     (1 << 1)
#define I2C_START      (1 << 2)
#define I2C_READ       (1 << 3)         // 0 write, 1 read
#define I2C_COMB_READ  (1 << 4)         // 0 master read immediately, 0 combined read access
#define I2C_IRQ_EN     (1 << 5)

#define I2C_START_WRITE (I2C_EN_CLK | I2C_START)
#define I2C_START_READ (I2C_EN_CLK | I2C_START | I2C_READ)


/*
  FIFO Register
  16 bytes (not words)
*/

#define I2C_FIFO_MAX 0x10
#define I2C_FIFO_MASK 0xf

/*
  Configuration Clock Register
*/

#define I2C_PTV         (7 << 0)  // Prescale clock divisor (divisor_1)
                              // Value is 1 greater than value set (2 = divisor of 3)
                              // This contradicts documentation, but follows example code
#define I2C_SPK_F_SHIFT 3
#define I2C_SPK_F       (7 << I2C_SPK_F_SHIFT)
#define I2C_CCR_DEFAULT 0xd2

/*
  Configuration Clock Functional Reference Register
*/

#define I2C_CLK_REF_MAX     0x7f   // Functional SCL clock reference (divisor_2)
#define I2C_CLK_REF_DEFAULT 0x9    // Value is 1 greater than value set (9 = divisor of 10)

/*
  Status FIFO Register
*/
#define I2C_FIFO_FULL      (1 << 0)
#define I2C_FIFO_EMPTY     (1 << 1)
#define I2C_READ_CPT_SHIFT 2
#define I2C_READ_CPT       (0xf << I2C_READ_CPT_SHIFT)    // read FIFO count value

/*
  Status Activity Register (read clears)
*/
#define I2C_ERROR_DATA   (1 << 0)          // 1 = error during bus access (subaddress or data)
#define I2C_ERROR_DEVICE (1 << 1)          // 1 = error on device address transmit
#define I2C_NOT_IDLE     (1 << 2)          // 1 = busy
#define I2C_INTERRUPT    (1 << 3)          // 1 = interrupt occurred
#define I2C_ERROR        (I2C_ERROR_DATA | I2C_ERROR_DEVICE)

#endif  // i2c_omap1509_h
