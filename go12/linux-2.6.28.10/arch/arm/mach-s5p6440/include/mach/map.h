/* linux/arch/arm/mach-s5p6440/include/mach/map.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S5P64XX - Memory map definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_MAP_H
#define __ASM_ARCH_MAP_H __FILE__

#include <plat/map-base.h>

/* HSMMC units */
#define S5P64XX_PA_HSMMC(x)	(0xED800000 + ((x) * 0x100000))
#define S5P64XX_PA_HSMMC0	S5P64XX_PA_HSMMC(0)
#define S5P64XX_PA_HSMMC1	S5P64XX_PA_HSMMC(1)
#define S5P64XX_PA_HSMMC2	S5P64XX_PA_HSMMC(2)

#define S3C_PA_UART		(0xEC005000)
#define S3C_PA_UART0		(S3C_PA_UART + 0x00)
#define S3C_PA_UART1		(S3C_PA_UART + 0x400)
#define S3C_PA_UART2		(S3C_PA_UART + 0x800)
#define S3C_PA_UART3		(S3C_PA_UART + 0xC00)
#define S3C_UART_OFFSET		(0x400)

/* See notes on UART VA mapping in debug-macro.S */
#define S3C_VA_UARTx(x)		(S3C_VA_UART + (S3C_PA_UART & 0xfffff) + ((x) * S3C_UART_OFFSET))

#define S3C_VA_UART0		S3C_VA_UARTx(0)
#define S3C_VA_UART1		S3C_VA_UARTx(1)
#define S3C_VA_UART2		S3C_VA_UARTx(2)
#define S3C_VA_UART3		S3C_VA_UARTx(3)
#define S3C_SZ_UART		SZ_256

#define S5P64XX_PA_SYSCON	(0xE0100000)
#define S5P64XX_PA_TIMER	(0xEA000000)
#define S5P64XX_PA_IIC0		(0xEC104000)
#define S5P64XX_PA_IIC1		(0xEC20F000)

#define S5P64XX_PA_SPI0		(0xEC400000)
#define S5P64XX_PA_SPI1		(0xEC500000)
#define S5P64XX_SZ_SPI0		SZ_4K
#define S5P64XX_SZ_SPI1		SZ_4K

#define S5P64XX_PA_GPIO		(0xE0308000)
#define S5P64XX_VA_GPIO		S3C_ADDR(0x00500000)
#define S5P64XX_SZ_GPIO		SZ_4K

#define S5P64XX_PA_SDRAM	(0x20000000)
#define S5P64XX_PA_VIC0		(0xE4000000)
#define S5P64XX_PA_VIC1		(0xE4100000)

#define S5P64XX_VA_SROMC	S3C_VA_SROMC
#define S5P64XX_PA_SROMC	(0xE7000000)
#define S5P64XX_SZ_SROMC	SZ_1M

#define S5P64XX_VA_LCD	   	S3C_VA_LCD
#define S5P64XX_PA_LCD	   	(0xEE000000)
#define S5P64XX_SZ_LCD		SZ_1M

#define S5P64XX_PA_ADC		(0xF3000000)

#define S5P64XX_PA_IIS_V40 	(0xF2000000)
#define S3C_SZ_IIS		SZ_8K

#define S5P64XX_PA_PCM		(0xF2100000)

#define S5P64XX_PA_RTC	   	(0xEA100000)

/* place VICs close together */
#define S3C_VA_VIC0		(S3C_VA_IRQ + 0x00)
#define S3C_VA_VIC1		(S3C_VA_IRQ + 0x10000)

////////////////////////////////////////////////////
//These are not changed to support for s5p6440

/* DMA controller */
#define S5P64XX_PA_DMA		(0xE9000000)

#define S5P64XX_PA_SMC9115	(0x18000000)
#define S5P64XX_SZ_SMC9115	SZ_512M

#define S5P64XX_PA_IIS	   	(0xF2000000)
/* Host I/F Indirect & Direct */
#define S5P64XX_VA_HOSTIFA	S3C_ADDR(0x00B00000)
#define S5P64XX_PA_HOSTIFA	(0x74000000)
#define S5P64XX_SZ_HOSTIFA	SZ_1M

#define S5P64XX_VA_HOSTIFB	S3C_ADDR(0x00C00000)
#define S5P64XX_PA_HOSTIFB	(0x74100000)
#define S5P64XX_SZ_HOSTIFB	SZ_1M

///////////////////////////////////////////////////

/* Watchdog */
#define S5P64XX_PA_WATCHDOG 	(0xEA200000)
#define S5P64XX_SZ_WATCHDOG 	SZ_4K

/* NAND flash controller */
#define S5P64XX_PA_NAND	   	(0xE7100000)
#define S5P64XX_SZ_NAND	   	SZ_1M

/* USB OTG */
#define S5P64XX_VA_OTG		S3C_ADDR(0x03900000)
#define S5P64XX_PA_OTG		(0xED100000)
#define S5P64XX_SZ_OTG		SZ_1M

/* USB OTG SFR */
#define S5P64XX_VA_OTGSFR	S3C_ADDR(0x03a00000)
#define S5P64XX_PA_OTGSFR	(0xED200000)
#define S5P64XX_SZ_OTGSFR	SZ_1M

/* Post Processor */
#define S5P64XX_PA_POST		(0xEE100000)
#define S5P64XX_SZ_POST		SZ_1M

/*FIMG VG*/
#define S5P64XX_PA_GVG	   	(0xF0000000)
#define S5P64XX_SZ_GVG		SZ_64K

/*FIMG 2D*/
#define S5P64XX_PA_G2D	   	(0xEF100000)
#define S5P64XX_SZ_G2D		SZ_1M

#define S5P64XX_PA_DMC		(0xE6000000)
#define S5P64XX_VA_DMC		S3C_ADDR(0x03b00000)
#define S5P64XX_SZ_DMC		SZ_1M

/* compatibiltiy defines. */
#define S3C_PA_TIMER		S5P64XX_PA_TIMER
#define S3C_PA_HSMMC0		S5P64XX_PA_HSMMC0
#define S3C_PA_HSMMC1		S5P64XX_PA_HSMMC1
#define S3C_PA_HSMMC2		S5P64XX_PA_HSMMC2
#define S3C_SZ_HSMMC		SZ_4K

#define S3C_PA_SPI0		S5P64XX_PA_SPI0
#define S3C_PA_SPI1		S5P64XX_PA_SPI1
#define S3C_SZ_SPI0		S5P64XX_SZ_SPI0
#define S3C_SZ_SPI1		S5P64XX_SZ_SPI1

#define S3C_PA_IIC		S5P64XX_PA_IIC0
#define S3C_PA_IIC1		S5P64XX_PA_IIC1

#define S3C_PA_RTC		S5P64XX_PA_RTC

#define S3C_PA_IIS		S5P64XX_PA_IIS
#define S3C_PA_ADC		S5P64XX_PA_ADC
#define S3C_PA_DMA		S5P64XX_PA_DMA

#define S3C_VA_OTG		S5P64XX_VA_OTG
#define S3C_PA_OTG		S5P64XX_PA_OTG
#define S3C_SZ_OTG		S5P64XX_SZ_OTG

#define S3C_VA_OTGSFR		S5P64XX_VA_OTGSFR
#define S3C_PA_OTGSFR		S5P64XX_PA_OTGSFR
#define S3C_SZ_OTGSFR		S5P64XX_SZ_OTGSFR

#endif /* __ASM_ARCH_6440_MAP_H */
