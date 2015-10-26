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
#ifndef IRQS_H
#define IRQS_H
// VIC0
#define IRQ_INTC0_START     0
#define BCM4760_INTR_VIC0_UNUSED0  0               // Unused interrupt 
#define BCM4760_INTR_PML0          1               // PML0 interrupt
#define BCM4760_INTR_PML1          2               // PML1 interrupt
#define BCM4760_INTR_WDT           3               // Watchdog timer interrupt
#define BCM4760_INTR_TIM0_CNTR1    4               // Timer-0, counter-1 interrupt
#define BCM4760_INTR_TSC           5               // Touch screen interrupt
#define BCM4760_INTR_USB           6               // USB interrupt
#define BCM4760_INTR_GPIO00        7               // GIO-00 interrupt (GPIO 0 Pins 0-6, for power lost)
#define BCM4760_INTR_GPIO10        8               // GIO-10 interrupt (GPIO 1 Pins 7-38)
#define BCM4760_INTR_GPIO11        9               // GIO-11 interrupt (GPIO 1 Pins 39-70)
#define BCM4760_INTR_GPIO12        10              // GIO-12 interrupt (GPIO 1 Pins 71-95)
#define BCM4760_INTR_TIM0_CNTR2    11              // Timer-0, counter-2 interrupt
#define BCM4760_INTR_TIM1_CNTR1    12              // Timer-1, counter-1 interrupt
#define BCM4760_INTR_URTG          13              // UART-GPS interrupt
//#define BCM4760_INTR_URT0          14              // UART-0 interrupt
//#define BCM4760_INTR_URT1          15              // UART-1 interrupt
#define BCM4760_INTR_URT1          14              // UART-1 interrupt
#define BCM4760_INTR_URT0          15              // UART-0 interrupt
#define BCM4760_INTR_URT2          16              // UART-2 interrupt
#define BCM4760_INTR_AUD_AUDIO     17              // Audio data fifo interrupt
#define BCM4760_INTR_AUD_VOICE     18              // Audio Voice interrupt
#define BCM4760_INTR_SPI0          19              // SPI0 interrupt
#define BCM4760_INTR_SPI1          20              // SPI1 interrupt
#define BCM4760_INTR_LCD           21              // LCD combined interrupt
#define BCM4760_INTR_VIC0_UNUSED22 22              // 
#define BCM4760_INTR_GFX_CORE      23              // Graphics core interrupt
#define BCM4760_INTR_GFX_MMU       24              // Graphics MMU interrupt
#define BCM4760_INTR_DMA_DONE      25              // DMA terminal count reached
#define BCM4760_INTR_ARM_PMU       26              // ARM PMU interrupt
#define BCM4760_INTR_SDM0          27              // SDM0 combined interrupt
#define BCM4760_INTR_RTC           28              // RTC interrupt
#define BCM4760_INTR_RNG           29              // RNG interrupt
#define BCM4760_INTR_TIM2_CNTR1    30              // Timer-2, counter-1 interrupt
#define BCM4760_INTR_VIC0_UNUSED31 31              // Unused interrupt
#define IRQ_INTC0_END      31

// VIC1
#define IRQ_INTC1_START    32
#define BCM4760_INTR_VIC1_UNUSED0  32              // Unused interrupt
#define BCM4760_INTR_VIC1_UNUSED1  33              // Unused interrupt
#define BCM4760_INTR_TIM1_CNTR2    34              // Timer-1, counter-2 interrupt
#define BCM4760_INTR_TIM0_CNTR3    35              // Timer-0, counter-3 interrupt
#define BCM4760_INTR_TIM1_CNTR3    36              // Timer-1, counter-3 interrupt
#define BCM4760_INTR_RTC_MTCH      37              // RTC Matched interrupt
#define BCM4760_INTR_TIM2_CNTR2    38              // Timer-2, counter-2 interrupt
#define BCM4760_INTR_VIC1_UNUSED7  39              // Unused interrupt
#define BCM4760_INTR_VIC1_UNUSED8  40              // Unused interrupt
#define BCM4760_INTR_VIC1_UNUSED9  41              // Unused interrupt
#define BCM4760_INTR_VIC1_UNUSED10 42              // Unused interrupt
#define BCM4760_INTR_VIC1_UNUSED11 43              // Unused interrupt
#define BCM4760_INTR_VIC1_UNUSED12 44              //  interrupt
#define BCM4760_INTR_VIC1_UNUSED13 45              // LCD vertical compare interrupt
#define BCM4760_INTR_I2S           46              // I2S interrupt
#define BCM4760_INTR_DDR_EMI       47              // DDR interrupt
#define BCM4760_INTR_DDR_SEC       48              // DDR secure interrupt
#define BCM4760_INTR_I2C0          49              // I2C0 interrupt
#define BCM4760_INTR_I2C1          50              // I2C1 interrupt
#define BCM4760_INTR_SDM1          51              // SDM1 interrupt
#define BCM4760_INTR_SDM2          52              // SDM2 interrupt
#define BCM4760_INTR_SPM           53              // SPM interrupt
#define BCM4760_INTR_DMA_ERR       54              // DMA error interrupt
#define BCM4760_INTR_AUD_BTMIXER   55              // Audio btmixer interrupt
#define BCM4760_INTR_TIM0_CNTR4    56              // Timer-0, counter-4 interrupt
#define BCM4760_INTR_TIM1_CNTR4    57              // Timer-1, counter-4 interrupt
#define BCM4760_INTR_FLS           58              // FLS interrupt
#define BCM4760_INTR_AXA           59              // AXA interrupt
#define BCM4760_INTR_VIC1_UNUSED28 60              // Unused interrupt
#define BCM4760_INTR_VIC1_UNUSED29 61              // Unused interrupt
#define BCM4760_INTR_VIC1_UNUSED30 62              // Unused interrupt
#define BCM4760_INTR_ETB           63              // ETB full or Acquision completed interrupt
#define IRQ_INTC1_END      63

#define NUM_INTERNAL_IRQS          (IRQ_INTC1_END+1)

#define IRQ_UNKNOWN                 -1

// Tune these bits to preclude noisy or unsupported interrupt sources as required.
#define IRQ_INTC0_VALID_MASK        0xffffffff
#define IRQ_INTC1_VALID_MASK        0xffffffff

#include <asm/arch/reg_irq.h>

#undef NR_IRQS
#define NR_IRQS			((NUM_INTERNAL_IRQS) + (NUM_GPIO_IRQS))

#include <linux/autoconf.h>

#ifdef CONFIG_PMU_DEVICE_BCM59040

/*
 * The following definition, for IRQ_BCM59XXX_FIRST, and that for NR_IRQS
 * must be the same! If they aren't mayhem will occur!
 */
#define	IRQ_BCM59XXX_FIRST	((NUM_INTERNAL_IRQS) + (NUM_GPIO_IRQS))

#include <asm/arch/bcm59040_irqs.h>

	#if !defined(BCM59XXX_MAX_IRQS)
	#define BCM59XXX_MAX_IRQS	BCM59040_NUM_IRQS
	#elif (BCM59040_NUM_IRQS > BCM59XXX_MAX_IRQS)
	#undef BCM59XXX_MAX_IRQS
	#define BCM59XXX_MAX_IRQS	BCM59040_NUM_IRQS
	#endif

#undef NR_IRQS
#define NR_IRQS			((IRQ_HI_END) + (NUM_GPIO_IRQS) + (BCM59XXX_MAX_IRQS))

#endif

#ifdef CONFIG_PMU_DEVICE

/*
 * The following definition, for IRQ_BCM59XXX_FIRST, and that for NR_IRQS
 * must be the same! If they aren't mayhem will occur!
 */
#define	IRQ_BCM59XXX_FIRST	((NUM_INTERNAL_IRQS) + (NUM_GPIO_IRQS))

#ifdef CONFIG_PMU_DEVICE_BCM59002

#include <asm/arch/bcm59002_irqs.h>

	#if !defined(BCM59XXX_MAX_IRQS)
	#define BCM59XXX_MAX_IRQS	BCM59002_NUM_IRQS
	#elif (BCM59002_NUM_IRQS > BCM59XXX_MAX_IRQS)
	#undef BCM59XXX_MAX_IRQS
	#define BCM59XXX_MAX_IRQS	BCM59002_NUM_IRQS
	#endif

#undef NR_IRQS
#define NR_IRQS			((IRQ_HI_END) + (NUM_GPIO_IRQS) + (BCM59XXX_MAX_IRQS))

#endif

#endif

#endif
