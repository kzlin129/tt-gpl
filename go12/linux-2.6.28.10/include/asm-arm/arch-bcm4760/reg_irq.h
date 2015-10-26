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
 *  Interrupt numbers
 */

// low 32 bits of interrupt vector
#define IRQ_LO_START		1
#define IRQ_INTC0_START     0
//#define IRQ_IDE			1	
#define IRQ_USB			BCM4760_INTR_USB		// USB controller interrupt
#define IRQ_SPI_0		BCM4760_INTR_SPI0		// SPI controller 0 interrupt
#define IRQ_SPI_1		BCM4760_INTR_SPI1		// SPI controller 1 interrupt
//#define IRQ_ARM_DMA_0		5
//#define IRQ_ARM_DMA_1		6
#define IRQ_SDIO_0		BCM4760_INTR_SDM0		// SDHCI controller 0 interrupt
#define IRQ_SDIO_1		BCM4760_INTR_SDM1		// SDHCI controller 1 interrupt
#define IRQ_ARM_PMU		BCM4760_INTR_ARM_PMU	// ARM1136 Power Management interrupt
//#define IRQ_KEY_XCHG		10
#define IRQ_RANDGEN		BCM4760_INTR_RNG		// True Random Number Generator interrupt
#define IRQ_RTC			BCM4760_INTR_RTC		// Real-time Clock interrupt
//#define IRQ_CUSTOMER		13
#define IRQ_CRYPTO		BCM4760_INTR_SPM		// SPU-M interrupt
//#define IRQ_VFIR		15
//#define IRQ_GPIO_TIMER_0	16
//#define IRQ_GPIO_TIMER_1	17
//#define IRQ_GPIO_TIMER_2	18
//#define IRQ_GPIO_TIMER_3	19
#define IRQ_UART0       BCM4760_INTR_URT0
#define IRQ_UART1       BCM4760_INTR_URT1
//#define IRQ_CEATA		22
#define IRQ_NAND		BCM4760_INTR_FLS
//#define IRQ_KEYPAD		24
#define IRQ_GPIO_0		BCM4760_INTR_GPIO00		// GPIO0 pins 0-6 interrupt
#define IRQ_GPIO_1		BCM4760_INTR_GPIO10		// GPIO1 pins 7-38 interrupt
#define IRQ_GPIO_2		BCM4760_INTR_GPIO11		// GPIO1 pins 39-70 interrupt
#define IRQ_GPIO_3		BCM4760_INTR_GPIO12		// GPIO1 pins 71-95 interrupt
//#define IRQ_GPIO_4		29
//#define IRQ_TOUCHWHEEL_SPI	30
#define IRQ_ARM_IIS		BCM4760_INTR_I2S		// audio codec I2S interrupt
#define IRQ_UART2       BCM4760_INTR_URT2
#define IRQ_UARTG       BCM4760_INTR_URTG		// GPS UART interrupt
#define IRQ_LO_END		32

#define IRQ_HI_START		33
#define IRQ_IIC		    BCM4760_INTR_I2C0		// General Purpose I2C interrupt
#define IRQ_BT_UART	    BCM4760_INTR_URT0
#define IRQ_BT_IIS	       	(33 + 2)
#define IRQ_LCD_SMI	       	(33 + 3)
#define IRQ_POWER_MGMT	    	(33 + 4)	
#define IRQ_EMI	       	BCM4760_INTR_DDR_EMI	// DDR/EMI interrupt
// 6-27 unassigned
#define IRQ_ARM_DOORBELL	(33 + 28)
#define IRQ_ARM_WATCHDOG	BCM4760_INTR_WDT	// Watchdog timer interrupt
//#define IRQ_VC2_WATCHDOG	(33 + 30)
//#define IRQ_VC2_INTR		(33 + 31)
#define IRQ_HI_END		64

/* ARM interrupt handling code assumes IRQ number
 * always starts from 0.
 */

#define  NUM_GPIO_IRQS              96

/* ARM interrupt handling code assumes IRQ number
 * always starts from 0.
 */
/*
 * IRQs 64 thru 159 are mapped onto GPIO lines 0 thru 96
 *
 * So, to request 
 */

#define  IRQ_GPIO_FIRST             IRQ_HI_END

#define  GPIO_TO_IRQ(gpio)          ((gpio) + IRQ_GPIO_FIRST )
#define  IRQ_TO_GPIO(irq)           ((irq) - IRQ_GPIO_FIRST )


/*
 * IRQs 64 thru 158 are mapped onto GPIO lines 0 thru 95
 *
 * So, to request an IRQ for GPIO pin 0 use IRQ 64, and so on. 
 */
