/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
 *  include/asm-arm/arch-BCM47XX/platform.h
 *  BCM47XX platform header file.
 */

#ifndef __ASM_ARCH_PLATFORM_H
#define __ASM_ARCH_PLATFORM_H

#include "bcm4760_addressmap.h"
	/*
 	 * BCM47XX memory map
 	 */
#define BCM47XX_ARM_DRAM	        0x30000000
#define BCM47XX_ARM_DRAM_SIZE		SZ_32M
#define BCM47XX_ARM_SCRATCH	        0x00100000
#define BCM47XX_ARM_SCRATCH_SIZE	SZ_64K
//#define BCM47XX_ARM_VC2_SRAM        BCM47XX_VCSRAM_ADDRBASE0
//#define BCM47XX_ARM_VC2_SRAM_SIZE   SZ_2M
//#define BCM47XX_ARM_VC2_PERIPH_BASE BCM47XX_DEBUGC_ADDRBASE0
#define BCM47XX_ARM_PERIPH_BASE		0x00080000
#define BCM47XX_ARM_DMA_BASE		DMA_REG_BASE_ADDR

	/*
 	 *  NOR Flash
 	 */
// #define BCM47XX_NOR_BASE		0x60000000	/* NOR flash address base */

    /*
     *  NAND Flash Controller
     */
#define BCM47XX_NAND_BASE		0x01000000	/* NAND flash controller base */


/* Inidividual peripheral device reset id */
#define IDE_RESET_ID		0
#define CEATA_RESET_ID		1
#define NAND_RESET_ID		2
#define USB_RESET_ID		3
#define SDIO_0_RESET_ID		4
#define SDIO_1_RESET_ID		5
#define VFIR_RESET_ID		6
#define CRYPTO_RESET_ID		7
#define AMC_RESET_ID		8
#define PWM_RESET_ID		9
#define UART_0_RESET_ID		10
#define UART_1_RESET_ID 	11
#define UART_2_RESET_ID 	12
#define PKE_RESET_ID	 	13
#define OTP_RESET_ID		14
#define TIMER_RESET_ID		15
#define TWSPI_RESET_ID		16
#define SPI_0_RESET_ID		17
#define SPI_1_RESET_ID		18
#define INT_RESET_ID		19
#define RNG_RESET_ID		20
#define RTC_RESET_ID		21
#define I2C_RESET_ID		22
#define SYSM_RESET_ID		23
#define GPIO_RESET_ID		24
#define PM_RESET_ID			25
#define I2S_RESET_ID		26
#define RPC_RESET_ID		28
#define WDT_RESET_ID		29

#define M_MTX_RESET_ID		(32 + 0)
#define M_DMA_RESET_ID		(32 + 1)
#define M_ROM_RESET_ID		(32 + 2)
#define M_NOR_RESET_ID		(32 + 3)
#define M_SCRAM_RESET_ID	(32 + 4)
#define M_EMI_RESET_ID		(32 + 5)
#define M_IPS_RESET_ID		(32 + 6)
#define C_UART_RESET_ID		(32 + 7)
#define C_I2S_RESET_ID		(32 + 8)
#define C_DC_RESET_ID		(32 + 9)
#define C_SMI_RESET_ID		(32 + 10)
#define V_CPU_RESET_ID		(32 + 11)
#define V_DC_RESET_ID		(32 + 12)
#define V_CAM_RESET_ID		(32 + 13)
#define V_BRG_RESET_ID		(32 + 14)
#define V_INT_RESET_ID		(32 + 15)
#define V_TIMER_RESET_ID	(32 + 16)
#define V_I2C_RESET_ID		(32 + 17)
#define V_I2S_RESET_ID		(32 + 18)
#define V_TVOUT_RESET_ID	(32 + 19)


	/*
 	 * Interrupt Controller
 	 */
/*
#define BCM47XX_INT_ISEL_LO		0x00
#define BCM47XX_INT_ISEL_HI		0x04
#define BCM47XX_INT_MASK_LO		0x08
#define BCM47XX_INT_MASK_HI		0x0c
#define BCM47XX_INT_MASK_SET_LO		0x10
#define BCM47XX_INT_MASK_SET_HI		0x14
#define BCM47XX_INT_MASK_CLR_LO		0x18
#define BCM47XX_INT_MASK_CLR_HI		0x1c
#define BCM47XX_INT_FIQ_STATUS_LO	0x20
#define BCM47XX_INT_FIQ_STATUS_HI	0x24
#define BCM47XX_INT_STATUS_LO		0x28
#define BCM47XX_INT_STATUS_HI		0x2c
#define BCM47XX_INT_FIQ_MSTATUS_LO	0x30
#define BCM47XX_INT_FIQ_MSTATUS_HI	0x34
*/
/*
#define BCM47XX_INT_MSTATUS_LO		0x28
#define BCM47XX_INT_MSTATUS_HI		0x2c
*/
/*
#define BCM47XX_INT_SWINTR_LO		0x58
#define BCM47XX_INT_SWINTR_HI		0x5c
#define BCM47XX_INT_SWINTR_CLR_LO	0x60
#define BCM47XX_INT_SWINTR_CLR_HI	0x64
#define BCM47XX_INT_VC2_MASK_LO		0x68
#define BCM47XX_INT_VC2_MASK_HI		0x6c
#define BCM47XX_INT_VC2_MSTATUS_LO	0x70
#define BCM47XX_INT_VC2_MSTATUS_HI	0x74
*/
	/*
 	 * System Timer
 	 */

#define MAX_TIMER                       2
#define MAX_PERIOD                      699050
#define TICKS_PER_uSEC                  24
#define SYS_TIMER_FREQ_IN_KHZ           24000U

/*
 *  These are useconds NOT ticks.
 *
 */
#define mSEC_1                          1000
#define mSEC_5                          (mSEC_1 * 5)
#define mSEC_10                         (mSEC_1 * 10)
#define mSEC_25                         (mSEC_1 * 25)
#define SEC_1                           (mSEC_1 * 1000)




#endif /* __ASM_ARCH_PLATFORM_H */

