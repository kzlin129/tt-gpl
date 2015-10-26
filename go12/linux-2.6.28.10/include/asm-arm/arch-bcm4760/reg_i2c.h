/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/


/*
*
*****************************************************************************
*
*  reg_i2c.h
*
*  PURPOSE:
*
*     This file contains definitions for the I2C Registers.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( __ASM_ARCH_REG_I2C_H )
#define __ASM_ARCH_REG_I2C_H

/* ---- Include Files ---------------------------------------------------- */

#include <asm/arch/hardware.h>
#include <asm/arch/bcm4760_reg.h>

/* ---- Constants and Types ---------------------------------------------- */
#define REG_SMT_CLK_L       IO_ADDRESS( RPC_REG_BASE_ADDR + 0 )     	// RPC (Ripple Counter Low)
#define REG_SMT_CLK_H       IO_ADDRESS( RPC_REG_BASE_ADDR + 4 )     	// RPC (Ripple Counter High)

#define	HW_I2C_BASE	IO_ADDRESS(I2C0_REG_BASE_ADDR+0x20)

#define REG_I2C_CS(base)          __REG8(IO_ADDRESS((base) + 0x20 + 3))     // I2C Control and Status Register
#define REG_I2C_TIM(base)         __REG8(IO_ADDRESS((base) + 0x24 + 3))     // I2C Timing Register
#define REG_I2C_DAT(base)         __REG8(IO_ADDRESS((base) + 0x28 + 3))     // I2C Data Register
#define REG_I2C_TOUT(base)        __REG8(IO_ADDRESS((base) + 0x2c + 3))     // I2C Timeout Register
//#define REG_I2C_RCM(base)         __REG8(IO_ADDRESS((base) + 0x30 + 3))     // I2C CRC Main Register
//#define REG_I2C_RCP(base)         __REG8(IO_ADDRESS((base) + 0x34 + 3))     // I2C CRC Polynomial Register
//#define REG_I2C_RCD(base)         __REG8(IO_ADDRESS((base) + 0x38 + 3))     // I2C CRC Data Register
#define REG_I2C_FCR(base)         __REG8(IO_ADDRESS((base) + 0x3c + 3))     // I2C FIFO Control Register
#define REG_I2C_FIFORDOUT(base)   __REG8(IO_ADDRESS((base) + 0x40 + 3))     // I2C FIFO Read Out Register
#define REG_I2C_IER(base)         __REG8(IO_ADDRESS((base) + 0x44 + 3))     // I2C FIFO Interrupt Enable Register
#define REG_I2C_ISR(base)         __REG8(IO_ADDRESS((base) + 0x48 + 3))     // I2C FIFO Interrupt Status Register
#define REG_I2C_CLKEN(base)       __REG8(IO_ADDRESS((base) + 0x4c + 3))     // I2C clock enable

// REG_I2C_CCS bits
#define REG_I2C_CS_EN           0x01        // 1:Enable I2C hardware 0:Reset I2C hardware
#define REG_I2C_CS_CMDMASK      0x06        // bits 2:1
#define REG_I2C_CS_CMDNULL      0x00        // No immediate action
#define REG_I2C_CS_CMDSTART     0x02        // Generate start condition
#define REG_I2C_CS_CMDSTOP      0x04        // Generate stop condition
#define REG_I2C_CS_CMDREAD      0x06        // Read a byte
#define REG_I2C_CS_ACK          0x08        // ACK bit
                                            // Write: CMD=11 ACK to send out after I2C byte read
                                            //        CMD=01 0: Normal start 1: Repeated start
                                            // Read: ACK returned from last I2C byte write
#define REG_I2C_CS_RDY          0x10        // BSC ready
#define REG_I2C_CS_BUSY         0x20        // BSC busy
#define REG_I2C_CS_SCL          0x40        // I2C clock Write:Set SCL pin value when EN=0 Read:SCL pin value
#define REG_I2C_CS_SDA          0x80        // I2C data  Write:Set SDA pin value when EN=0 Read:SDA pin value
#define REG_I2C_CS_CMDRESTART   (REG_I2C_CS_CMDSTART | REG_I2C_CS_ACK)


// REG_I2C_TIM bits
// --- I2C speed. Master clock is 24MHz
#define REG_I2C_TIM_DIVMSK      0x03        // clock division bits
#define REG_I2C_TIM_DIV16       0           // MCLK /16
#define REG_I2C_TIM_DIV8        1           // MCLK /8
#define REG_I2C_TIM_DIV4        2           // MCLK /4
#define REG_I2C_TIM_DIV2        03          // MCLK /2
#define REG_I2C_TIM_PMSK        0x38        // # cycles for SCL, high=2*p+1, low=2*p+2

#define REG_I2C_TIM_P0          (0<<3)
#define REG_I2C_TIM_P1          (1<<3)
#define REG_I2C_TIM_P2          (2<<3)
#define REG_I2C_TIM_P3          (3<<3)
#define REG_I2C_TIM_P4          (4<<3)
#define REG_I2C_TIM_P6          (6<<3)
#define REG_I2C_TIM_P7          (7<<3)

// REG_I2C_CLKEN bits
//
#define REG_I2C_CLKEN_MMSK      (7<<4)
#define REG_I2C_CLKEN_M0        (0<<4)
#define REG_I2C_CLKEN_M1        (1<<4)
#define REG_I2C_CLKEN_M2        (2<<4)
#define REG_I2C_CLKEN_M3        (3<<4)
#define REG_I2C_CLKEN_M4        (4<<4)
#define REG_I2C_CLKEN_M5        (5<<4)
#define REG_I2C_CLKEN_M6        (6<<4)
#define REG_I2C_CLKEN_M7        (7<<4)

#define REG_I2C_CLKEN_NMSK      (7<<1)
#define REG_I2C_CLKEN_N0        (0<<1)
#define REG_I2C_CLKEN_N1        (1<<1)
#define REG_I2C_CLKEN_N2        (2<<1)
#define REG_I2C_CLKEN_N3        (3<<1)
#define REG_I2C_CLKEN_N4        (4<<1)
#define REG_I2C_CLKEN_N5        (5<<1)
#define REG_I2C_CLKEN_N6        (6<<1)
#define REG_I2C_CLKEN_N7        (7<<1)

#define BSC_MASTER_CLK_FREQ 12000000UL
//#define BSC_MASTER_CLK_FREQ 24000000UL

#if (BSC_MASTER_CLK_FREQ == 12000000UL) 
// --- SCL speed
// --- I2CTIM_P bits = 111 group
#define REG_I2C_CLK_26K     (REG_I2C_TIM_DIV16 | REG_I2C_TIM_P7)
#define REG_I2C_CLK_52K     (REG_I2C_TIM_DIV8| REG_I2C_TIM_P7)
#define REG_I2C_CLK_105K    (REG_I2C_TIM_DIV4| REG_I2C_TIM_P7)
#define REG_I2C_CLK_210K    (REG_I2C_TIM_DIV2| REG_I2C_TIM_P7)

// I2CTIM_P bits = 110 group
#define REG_I2C_CLK_30K     (REG_I2C_TIM_DIV16 | REG_I2C_TIM_P6)
#define REG_I2C_CLK_60K     (REG_I2C_TIM_DIV8| REG_I2C_TIM_P6)
#define REG_I2C_CLK_120K    (REG_I2C_TIM_DIV4| REG_I2C_TIM_P6)
#define REG_I2C_CLK_240K    (REG_I2C_TIM_DIV2| REG_I2C_TIM_P6)

// I2CTIM_P bits = 100 group
#define REG_I2C_CLK_43K     (REG_I2C_TIM_DIV16 | REG_I2C_TIM_P4)
#define REG_I2C_CLK_85K     (REG_I2C_TIM_DIV8| REG_I2C_TIM_P4)
#define REG_I2C_CLK_171K    (REG_I2C_TIM_DIV4| REG_I2C_TIM_P4)
#define REG_I2C_CLK_342K    (REG_I2C_TIM_DIV2| REG_I2C_TIM_P4)

// I2CTIM_P bits = 011 group
#define REG_I2C_CLK_54K     (REG_I2C_TIM_DIV16 | REG_I2C_TIM_P3)
#define REG_I2C_CLK_108K    (REG_I2C_TIM_DIV8| REG_I2C_TIM_P3)
#define REG_I2C_CLK_217K    (REG_I2C_TIM_DIV4| REG_I2C_TIM_P3)
#define REG_I2C_CLK_433K    (REG_I2C_TIM_DIV2| REG_I2C_TIM_P3)

// I2CTIM_P bits = 010 group
#define REG_I2C_CLK_50K     (REG_I2C_TIM_DIV16 | REG_I2C_TIM_P2)
#define REG_I2C_CLK_100K    (REG_I2C_TIM_DIV8| REG_I2C_TIM_P2)
#define REG_I2C_CLK_200K    (REG_I2C_TIM_DIV4| REG_I2C_TIM_P2)
#define REG_I2C_CLK_400K    (REG_I2C_TIM_DIV2| REG_I2C_TIM_P2) 

#else // BCM4760 I2C clock is 24Mhz

#define REG_I2C_CLK_100K    (REG_I2C_TIM_DIV16 | REG_I2C_TIM_P2)
#define REG_I2C_CLK_200K    (REG_I2C_TIM_DIV8  | REG_I2C_TIM_P2)
#define REG_I2C_CLK_266K    (REG_I2C_TIM_DIV4  | REG_I2C_TIM_P4)

#if 0 // comment out because of bug in the i2c dividor, this produces 300kbs instead
#define REG_I2C_CLK_400K    (REG_I2C_TIM_DIV4| REG_I2C_TIM_P2)
#else
#define REG_I2C_CLK_400K    (REG_I2C_TIM_DIV2| REG_I2C_TIM_P6)    // this value produces 392kbs
#endif

#endif

// REG_I2C_I2CTOUT bits
#define REG_I2C_TOUT_EN             0x80        // Enables the Timeout counter

// REG_I2C_RCM bits
#define REG_I2C_RCM_EN              0x01        // enable CRC generation
#define REG_I2C_RCM_LSB0            0x02        // =1, Force LSB to be 0

// REG_I2C_FCR bits
#define REG_I2C_FCR_FLUSH           0x80        // Writing 1 will flush the Tx FIFO; Write Only
#define REG_I2C_FCR_EN              0x40        // FIFO Enable
#define REG_I2C_FCR_CNTMASK         0x07        // FIFO Data Counter; Read Only

// REG_I2C_IER bits
#define REG_I2C_IER_INT_EN          0x08        // 1: Enable I2C busy status interrupt
#define REG_I2C_IER_ERRINT_EN       0x04        // 1: Enable I2C error status interrupt
#define REG_I2C_IER_FIFOINT_EN      0x02        // 1: Enable I2C Tx FIFO empty interrupt
#define REG_I2C_IER_NOACK_EN        0x01        // 1: Enable I2C No Acknowledge interrupt

// REG_I2C_ISR bits
#define REG_I2C_ISR_CMD_BUSY        0x80        // Command busy
#define REG_I2C_ISR_SES_DONE        0x08        // Read 0: I2C session busy 1: I2C session done
                                                // Write 0: No action 1: Clear the interrupt
#define REG_I2C_ISR_I2CERR          0x04        // I2C Bus Error Status
#define REG_I2C_ISR_TXFIFOEMPTY     0x02        // I2C Tx FIFO Empty
#define REG_I2C_ISR_NOACK           0x01        // I2C No Acknowledge interrupt

// REG_I2C_CLKEN bits
#define REG_I2C_CLKEN_AUTO_SENSE_OFF 0x80       // 1: auto sense off
#define REG_I2C_CLKEN_CLKEN         0x01        // 1: Enable I2C clock
#endif
