/* arch/arm/boot/compressed/tomtomgo-init.c
 *
 * Special initialization for TomTom GO.
 *
 * Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/arch/tomtomgo-init.h>

#include <asm/io.h>
#include <asm/arch/map.h>

/* We need to define these locally to prevent mistakes */
#define S3C2410_GSTATUS1	(S3C2410_PA_GPIO   + 0xB0)
#define S3C2440_DSC0		(S3C2410_PA_GPIO   + 0xC4)
#define S3C2440_DSC1		(S3C2410_PA_GPIO   + 0xC8)
#define S3C2410_CAMDIVN		(S3C2410_PA_CLKPWR + 0x18)
#define S3C2410_CLKDIVN		(S3C2410_PA_CLKPWR + 0x14)
#define S3C2410_MPLLCON		(S3C2410_PA_CLKPWR + 0x04)
#define S3C2410_UPLLCON		(S3C2410_PA_CLKPWR + 0x08)
#define S3C2410_ULCON0		(S3C2410_PA_UART   + 0x00)
#define S3C2410_UCON0		(S3C2410_PA_UART   + 0x04)
#define S3C2410_UFCON0		(S3C2410_PA_UART   + 0x08)
#define S3C2410_UMCON0		(S3C2410_PA_UART   + 0x0C)
#define S3C2410_UBRDIV0		(S3C2410_PA_UART   + 0x28)
#define S3C2410_ULCON1		(S3C2410_PA_UART + 0x4000)
#define S3C2410_UCON1		(S3C2410_PA_UART + 0x4004)
#define S3C2410_UFCON1		(S3C2410_PA_UART + 0x4008)
#define S3C2410_UMCON1		(S3C2410_PA_UART + 0x400C)
#define S3C2410_UBRDIV1		(S3C2410_PA_UART + 0x4028)
#define S3C2410_ULCON2		(S3C2410_PA_UART + 0x8000)
#define S3C2410_UCON2		(S3C2410_PA_UART + 0x8004)
#define S3C2410_UFCON2		(S3C2410_PA_UART + 0x8008)
#define S3C2410_UMCON2		(S3C2410_PA_UART + 0x800C)
#define S3C2410_UBRDIV2		(S3C2410_PA_UART + 0x8028)
#define S3C2410_ULCON3		(S3C2410_PA_UART + 0xC000)
#define S3C2410_UCON3		(S3C2410_PA_UART + 0xC004)
#define S3C2410_UFCON3		(S3C2410_PA_UART + 0xC008)
#define S3C2410_UMCON3		(S3C2410_PA_UART + 0xC00C)
#define S3C2410_UBRDIV3		(S3C2410_PA_UART + 0xC028)
#define S3C2410_PRIORITY	(S3C2410_PA_IRQ    + 0x0C)

#define S3C2412_GSTATUS1	(S3C2410_PA_GPIO   + 0xC0)
#define S3C2412_CLKSRC		(S3C2410_PA_CLKPWR + 0x1C)

#define S3C2443_PCLKCON		(S3C2410_PA_CLKPWR + 0x34)
#define S3C2410_GPHCON		(S3C2410_PA_GPIO   + 0x70)

#define S3C2443_HCLKCON		(S3C2410_PA_CLKPWR + 0x30)
#define S3C2443_SCLKCON		(S3C2410_PA_CLKPWR + 0x38)

#define S3C2443_BUSPRI0		(S3C2410_PA_CLKPWR + 0x50)

#define S3C2443_BPRIORITY0	(S3C24XX_PA_MATRIX + 0x00)
#define S3C2443_BPRIORITY1	(S3C24XX_PA_MATRIX + 0x04)
#define S3C2443_EBICON		(S3C24XX_PA_MATRIX + 0x08)

#define S3C2443_BPRIORITY0_PRITYP_FIXED		(0<<2)
#define S3C2443_BPRIORITY0_PRITYP_ROTATE	(1<<2)
#define S3C2443_BPRIORITY0_FIXPRITYP_SABOVEI	(0<<0)
#define S3C2443_BPRIORITY0_FIXPRITYP_IABOVES	(1<<0)

#define S3C2443_BPRIORITY1_PRITYP_FIXED		(0<<2)
#define S3C2443_BPRIORITY1_PRITYP_ROTATE	(1<<2)
#define S3C2443_BPRIORITY1_FIXPRITYP_SABOVEI	(0<<0)
#define S3C2443_BPRIORITY1_FIXPRITYP_IABOVES	(1<<0)

#define S3C2443_EBICON_FIXPRITYP_MASK		(0x3<<0)
#define S3C2443_EBICON_FIXPRITYP_SNCE		(0x0<<0)
#define S3C2443_EBICON_FIXPRITYP_SCNE		(0x1<<0)
#define S3C2443_EBICON_FIXPRITYP_SENC		(0x2<<0)
#define S3C2443_EBICON_FIXPRITYP_ESNC		(0x3<<0)
#define S3C2443_EBICON_PRI_TYP_FIXED		(0<<2)
#define S3C2443_EBICON_PRI_TYP_ROTATE		(1<<2)
#define S3C2443_EBICON_BANK1CFG_SROM		(0<<8)
#define S3C2443_EBICON_BANK1CFG_NAND		(1<<8)
#define S3C2443_EBICON_BANK2CFG_SROM		(0<<9)
#define S3C2443_EBICON_BANK2CFG_CF		(1<<9)
#define S3C2443_EBICON_BANK3CFG_SROM		(0<<10)
#define S3C2443_EBICON_BANK3CFG_CF		(1<<10)

#define S3C2443_BUSPRI0_ORDERI_MASK		(0x7<<0)
#define S3C2443_BUSPRI0_ORDERI_0123456		(0x0<<0)
#define S3C2443_BUSPRI0_ORDERI_1234560		(0x1<<0)
#define S3C2443_BUSPRI0_ORDERI_2345601		(0x2<<0)
#define S3C2443_BUSPRI0_ORDERI_3456012		(0x3<<0)

#define S3C2443_BUSPRI0_TYPEI_MASK		(0x3<<6)
#define S3C2443_BUSPRI0_TYPEI_FIXED		(0x0<<6)
#define S3C2443_BUSPRI0_TYPEI_LGML		(0x1<<6)
#define S3C2443_BUSPRI0_TYPEI_ROTATE		(0x2<<6)
#define S3C2443_BUSPRI0_TYPEI_UNDEF		(0x3<<6)

#define S3C2443_BUSPRI0_ORDERS_MASK		(0xf<<8)
#define S3C2443_BUSPRI0_ORDERS_0123456789101112	(0x0<<8)
#define S3C2443_BUSPRI0_ORDERS_1234567891011120	(0x1<<8)
#define S3C2443_BUSPRI0_ORDERS_2345678910111201	(0x2<<8)
#define S3C2443_BUSPRI0_ORDERS_3456789101112012	(0x3<<8)
#define S3C2443_BUSPRI0_ORDERS_4567891011120123	(0x4<<8)
#define S3C2443_BUSPRI0_ORDERS_5678910111201234	(0x5<<8)
#define S3C2443_BUSPRI0_ORDERS_6789101112012345	(0x6<<8)
#define S3C2443_BUSPRI0_ORDERS_7891011120123456	(0x7<<8)
#define S3C2443_BUSPRI0_ORDERS_8910111201234567	(0x8<<8)
#define S3C2443_BUSPRI0_ORDERS_9101112012345678	(0x9<<8)
#define S3C2443_BUSPRI0_ORDERS_1011120123456789	(0xa<<8)
#define S3C2443_BUSPRI0_ORDERS_1112012345678910	(0xb<<8)
#define S3C2443_BUSPRI0_ORDERS_1201234567891011	(0xc<<8)
#define S3C2443_BUSPRI0_ORDERS_UNDEF_D		(0xd<<8)
#define S3C2443_BUSPRI0_ORDERS_UNDEF_E		(0xe<<8)
#define S3C2443_BUSPRI0_ORDERS_UNDEF_F		(0xf<<8)

#define S3C2443_BUSPRI0_TYPES_MASK		(0x3<<14)
#define S3C2443_BUSPRI0_TYPES_FIXED		(0x0<<14)
#define S3C2443_BUSPRI0_TYPES_LGML		(0x1<<14)
#define S3C2443_BUSPRI0_TYPES_ROTATE		(0x2<<14)
#define S3C2443_BUSPRI0_TYPES_UNDEF		(0x3<<14)

#define S3C2443_HCLKCON_DMA0			(1<<0)
#define S3C2443_HCLKCON_DMA1			(1<<1)
#define S3C2443_HCLKCON_DMA2			(1<<2)
#define S3C2443_HCLKCON_DMA3			(1<<3)
#define S3C2443_HCLKCON_DMA4			(1<<4)
#define S3C2443_HCLKCON_DMA5			(1<<5)
#define S3C2443_HCLKCON_CAMIF			(1<<8)
#define S3C2443_HCLKCON_DISPCON			(1<<9)
#define S3C2443_HCLKCON_LCDCON			(1<<10)
#define S3C2443_HCLKCON_USBHOST			(1<<11)
#define S3C2443_HCLKCON_USBDEV			(1<<12)
#define S3C2443_HCLKCON_HSMMC			(1<<16)
#define S3C2443_HCLKCON_CFC			(1<<17)
#define S3C2443_HCLKCON_SSMC			(1<<18)
#define S3C2443_HCLKCON_DRAMC			(1<<19)

#define S3C2443_SCLKCON_USBHOST		(1<<1)
#define S3C2443_SCLKCON_UARTCLK		(1<<8)
#define S3C2443_SCLKCON_IISCLK		(1<<9)
#define S3C2443_SCLKCON_DISPCLK		(1<<10)
#define S3C2443_SCLKCON_CAMCLK		(1<<11)
#define S3C2443_SCLKCON_HSMMCCLK	(1<<12)
#define S3C2443_SCLKCON_HSMMCCLK_EXT	(1<<13)
#define S3C2443_SCLKCON_SPICLK		(1<<14)
#define S3C2443_SCLKCON_SSMCCLK		(1<<15)
#define S3C2443_SCLKCON_DDRCLK		(1<<16)

#define S3C2410_GSTATUS1_IDMASK	0xfffffff0
#define S3C2410_GSTATUS1_M2410	0x32410000
#define S3C2410_GSTATUS1_M2412	0x32412000
#define S3C2410_GSTATUS1_M2440	0x32440000
#define S3C2410_GSTATUS1_M2442	0x32440aa0
#define S3C2410_GSTATUS1_M2443	0x32443000
#define S3C2410_GSTATUS1_M2450	0x32450000

#define S3C2450_HCLKCON				(S3C2410_PA_CLKPWR + 0x30)
#define S3C2450_PCLKCON				(S3C2410_PA_CLKPWR + 0x34)
#define S3C2450_SCLKCON				(S3C2410_PA_CLKPWR + 0x38)

#define S3C2450_HCLKCON_DMA0			(1<<0)
#define S3C2450_HCLKCON_DMA1			(1<<1)
#define S3C2450_HCLKCON_DMA2			(1<<2)
#define S3C2450_HCLKCON_DMA3			(1<<3)
#define S3C2450_HCLKCON_DMA4			(1<<4)
#define S3C2450_HCLKCON_DMA5			(1<<5)
#define S3C2450_HCLKCON_DMA6			(1<<6)
#define S3C2450_HCLKCON_DMA7			(1<<7)
#define S3C2450_HCLKCON_DISPCON			(1<<9)
#define S3C2450_HCLKCON_USBHOST			(1<<11)
#define S3C2450_HCLKCON_USBDEV			(1<<12)
#define S3C2450_HCLKCON_IROM			(1<<13)
#define S3C2450_HCLKCON_HSMMC0			(1<<15)
#define S3C2450_HCLKCON_HSMMC1			(1<<16)
#define S3C2450_HCLKCON_SSMC			(1<<18)
#define S3C2450_HCLKCON_DRAMC			(1<<19)
#define S3C2450_HCLKCON_2D			(1<<20)

#define S3C2450_SCLKCON_USBHOST			(1<<1)
#define S3C2450_SCLKCON_IISCLK_1		(1<<5)
#define S3C2450_SCLKCON_HSMMCCLK_0		(1<<6)
#define S3C2450_SCLKCON_SPICLK_1		(1<<7)
#define S3C2450_SCLKCON_UARTCLK			(1<<8)
#define S3C2450_SCLKCON_IISCLK_0		(1<<9)
#define S3C2450_SCLKCON_DISPCLK			(1<<10)
#define S3C2450_SCLKCON_CAMCLK			(1<<11)
#define S3C2450_SCLKCON_HSMMCCLK_1		(1<<12)
#define S3C2450_SCLKCON_HSMMCCLK_EXT		(1<<13)
#define S3C2450_SCLKCON_SPICLK_0		(1<<14)
#define S3C2450_SCLKCON_SSMCCLK			(1<<15)
#define S3C2450_SCLKCON_DDRCLK			(1<<16)
#define S3C2450_SCLKCON_PCM0_EXT		(1<<17)
#define S3C2450_SCLKCON_PCM1_EXT		(1<<18)
#define S3C2450_SCLKCON_SPICLK_MPLL0		(1<<19)
#define S3C2450_SCLKCON_SPICLK_MPLL1		(1<<20)

/* Chose baud rate and divisor calculation */
#define CURRENT_BAUD_RATE	115200
#define UBRDIV(pclk, baud)	(pclk / 16 / baud - 1)

/* Chosen CPU speeds for different models */
#define S3C2410_PLLSPEED 199200000

#define S3C2412_PLLSPEED 266000000

#define S3C2442_PLLSPEED 376800000 // 399000000 // 376800000

#ifdef CONFIG_SMDK2440_BOARD
#define S3C2440_PLLSPEED 399651840
#else
#define S3C2440_PLLSPEED 376800000 // 376800000 // 393000000 // 376800000 // 405600000
#endif

/* 
 * Dividers for S3C2410 
 */
#if S3C2410_PLLSPEED == 199200000
#define S3C2410_HDIVN	1
#define S3C2410_PDIVN	1
#define S3C2410_MDIV	0x9e
#define S3C2410_PDIV	3
#define S3C2410_SDIV	1
#define S3C2410_HCLKDIV	2
#define S3C2410_PCLKDIV	4
#elif S3C2410_PLLSPEED == 202000000
#define S3C2410_HDIVN	1
#define S3C2410_PDIVN	1
#define S3C2410_MDIV	0x5d
#define S3C2410_PDIV	1
#define S3C2410_SDIV	1
#define S3C2410_HCLKDIV	2
#define S3C2410_PCLKDIV	4
#elif S3C2410_PLLSPEED == 202800000
#define S3C2410_HDIVN	1
#define S3C2410_PDIVN	1
#define S3C2410_MDIV	0xa1
#define S3C2410_PDIV	3
#define S3C2410_SDIV	1
#define S3C2410_HCLKDIV	2
#define S3C2410_PCLKDIV	4
#else
#error "Select a proper S3C2410 PLL speed"
#endif
#define S3C2410_PCLK	(S3C2410_PLLSPEED / S3C2410_PCLKDIV)

/* 
 * Dividers for S3C2412 / S3C2413 
 */
//;	(42,1,1)=200Mhz, (47,1,1)=220Mhz, (72,2,1)=240Mhz, (57,1,1)=260Mhz, (125,4,1)=266Mhz
//;	(62,1,1)=280Mhz, (67,1,1)=300Mhz, (72,4,0)=320Mhz, (63,3,0)=340Mhz, (52,2,0)=360Mhz
//;   (42,1,0)=400Mhz,
#if S3C2412_PLLSPEED == 200000000
#define S3C2412_HDIVN	1
#define S3C2412_PDIVN	1
#define S3C2412_MDIV	42
#define S3C2412_PDIV	1
#define S3C2412_SDIV	1
#define S3C2412_HCLKDIV	2
#define S3C2412_PCLKDIV	4
#elif S3C2412_PLLSPEED == 266000000
#define S3C2412_HDIVN	1
#define S3C2412_PDIVN	1
#define S3C2412_MDIV	125
#define S3C2412_PDIV	4
#define S3C2412_SDIV	1
#define S3C2412_HCLKDIV	2
#define S3C2412_PCLKDIV	4
#else
#error "Select a proper S3C2412 PLL speed"
#endif
#define S3C2412_PCLK	(S3C2412_PLLSPEED / S3C2412_PCLKDIV)

/* 
 * Dividers for S3C2440 
 */
#if S3C2440_PLLSPEED == 399651840
#define S3C2440_HDIVN	2
#define S3C2440_PDIVN	1
#define S3C2440_MDIV	0x6e
#define S3C2440_PDIV	3
#define S3C2440_SDIV	1
#define S3C2410_HCLKDIV	4
#define S3C2410_PCLKDIV	8

// a1h, 3h,0h |405600000|405M|  2.4M| 0.1| 0.1| 0.2|0|
#elif S3C2440_PLLSPEED == 405600000
#define S3C2440_HDIVN	3
#define S3C2440_PDIVN	1
#define S3C2440_MDIV	0xa1
#define S3C2440_PDIV	3
#define S3C2440_SDIV	1
#define S3C2440_HCLKDIV	3
#define S3C2440_PCLKDIV	6

// 95h, 3h,0h |376800000|376M|  2.4M| 0.1| 0.1| 0.2|0|
#elif S3C2440_PLLSPEED == 376800000
#define S3C2440_HDIVN	2
#define S3C2440_PDIVN	1
#define S3C2440_MDIV	0x95
#define S3C2440_PDIV	3
#define S3C2440_SDIV	1
#define S3C2440_HCLKDIV	4
#define S3C2440_PCLKDIV	8

// 95h, 3h,0h |376800000|376M|  2.4M| 0.1| 0.1| 0.2|0|
#elif S3C2440_PLLSPEED == 400000000
#define S3C2440_HDIVN	2
#define S3C2440_PDIVN	1
#define S3C2440_MDIV	42
#define S3C2440_PDIV	1
#define S3C2440_SDIV	1
#define S3C2440_HCLKDIV	4
#define S3C2440_PCLKDIV	8

// 7bh, 2h,0h |393000000|393M|  3.0M| 0.1| 0.1| 0.3|0|
#elif S3C2440_PLLSPEED == 393000000
#define S3C2440_HDIVN	2
#define S3C2440_PDIVN	1
#define S3C2440_MDIV	0x7b
#define S3C2440_PDIV	2
#define S3C2440_SDIV	1
#define S3C2440_HCLKDIV	4
#define S3C2440_PCLKDIV	8

// 9ch, 3h,0h |393600000|393M|  2.4M| 0.1| 0.1| 0.2|0|
#elif S3C2440_PLLSPEED == 393000000
#define S3C2440_HDIVN	2
#define S3C2440_PDIVN	1
#define S3C2440_MDIV	0x9c
#define S3C2440_PDIV	3
#define S3C2440_SDIV	1
#define S3C2440_HCLKDIV	4
#define S3C2440_PCLKDIV	8

// 78h, 3h,0h |307200000|307M|  2.4M| 0.1| 0.1| 0.2|0|
#elif S3C2440_PLLSPEED == 307200000
#define S3C2440_HDIVN	3
#define S3C2440_PDIVN	1
#define S3C2440_MDIV	0x78
#define S3C2440_PDIV	3
#define S3C2440_SDIV	1
#define S3C2440_HCLKDIV	3
#define S3C2440_PCLKDIV	6

#else
#error "Select a proper S3C2440 PLL speed"
#endif
#define S3C2440_PCLK	(S3C2440_PLLSPEED / S3C2440_PCLKDIV)

#if S3C2442_PLLSPEED == 399000000
#define S3C2442_HDIVN	3
#define S3C2442_PDIVN	1
#define S3C2442_MDIV	125
#define S3C2442_PDIV	6
#define S3C2442_SDIV	0
#define S3C2442_HCLKDIV	3
#define S3C2442_PCLKDIV	6

#elif S3C2442_PLLSPEED == 376800000
#define S3C2442_HDIVN	2
#define S3C2442_PDIVN	1
#define S3C2442_MDIV	0x95
#define S3C2442_PDIV	8
#define S3C2442_SDIV	0
#define S3C2442_HCLKDIV	4
#define S3C2442_PCLKDIV	8

#else
#error "Select a proper S3C2442 PLL speed"
#endif
#define S3C2442_PCLK	(S3C2442_PLLSPEED / S3C2442_PCLKDIV)

#define S3C2443_PCLK	66000000
#define S3C2450_PCLK	66000000


/*
 *	interrupt contention arbitration settings
 */
#define PRIO_012345		0x00
#define PRIO_0_234_15	0x01
#define PRIO_0_34_125	0x02
#define PRIO_0_4_1235	0x03

/* USBH/UART1 over (TIMER, DMA, SDI, HDD) */
#define ARB_SEL0		PRIO_012345
#define ARB_SEL1		PRIO_012345
#define ARB_SEL2		PRIO_012345
#define ARB_SEL3		PRIO_012345
/* USBH over UART1 */
#define ARB_SEL4		PRIO_0_4_1235
#define ARB_SEL5		PRIO_012345
#define ARB_SEL6		PRIO_0_4_1235

#define ARB_MODE0		0
#define ARB_MODE1		0
#define ARB_MODE2		0
#define ARB_MODE3		0
#define ARB_MODE4		0
#define ARB_MODE5		0
#define ARB_MODE6		0

void tomtomgo_init(void)
{
	int s3c2412_detected = 0;
	unsigned long cpu;
	unsigned long cpu_m;
	unsigned long clkdivn;
	unsigned long mpllcon;
	unsigned long ubrdiv;
	unsigned long clksrc = 0; /* prevent warning */
	unsigned long upllcon;
	unsigned long confreg;
	volatile int i;

	/* Determine CPU type */
	cpu = __raw_readl(S3C2410_GSTATUS1);
	cpu_m = cpu & S3C2410_GSTATUS1_IDMASK;

	/* determine parameters */
	switch (cpu_m) {
	case S3C2410_GSTATUS1_M2410:
		/* S3C2410 */
		clkdivn = (S3C2410_HDIVN<<1) | S3C2410_PDIVN;
		mpllcon = (S3C2410_MDIV<<12) | (S3C2410_PDIV<<4) | S3C2410_SDIV;
		ubrdiv = UBRDIV(S3C2410_PCLK, CURRENT_BAUD_RATE);
		/* configure USB clock at 48 MHz */
		upllcon = 0x38022;
		break;
	case S3C2410_GSTATUS1_M2440:
		/* S3C2440 */
		clkdivn = (S3C2440_HDIVN<<1) | S3C2440_PDIVN;
		ubrdiv = UBRDIV(S3C2440_PCLK, CURRENT_BAUD_RATE);
		mpllcon = (S3C2440_MDIV<<12) | (S3C2440_PDIV<<4) | S3C2440_SDIV;
		/* reduce the bus driver power to lower the noise floor on S3C2440 */
		__raw_writel(0x7fffffff, S3C2440_DSC0);
		__raw_writel(0xffffffff, S3C2440_DSC1);
		/* configure USB clock at 48 MHz */
		upllcon = 0x38022;
		break;
	case S3C2410_GSTATUS1_M2442:
		/* S3C2442 */
		clkdivn = (S3C2442_HDIVN<<1) | S3C2442_PDIVN;
		ubrdiv = UBRDIV(S3C2442_PCLK, CURRENT_BAUD_RATE);
		mpllcon = (S3C2442_MDIV<<12) | (S3C2442_PDIV<<4) | S3C2442_SDIV;
		/* don't reduce the bus driver power because of 1.8 volt logic */
		__raw_writel(0x00000000, S3C2440_DSC0);
		__raw_writel(0x00000000, S3C2440_DSC1);
		/* configure USB clock at 48 MHz */
		upllcon = 0x38022;
		break;
	case S3C2410_GSTATUS1_M2443:
		/* Don't touch anything, for now. */
		ubrdiv = UBRDIV(S3C2443_PCLK, CURRENT_BAUD_RATE);

		__raw_writel(0x0000,  S3C2410_UFCON0);  /* FIFO control register, FIFO disable */
		__raw_writel(0x0000,  S3C2410_UMCON0);  /* Modem control register, AFC disable */
		__raw_writel(0x0003,  S3C2410_ULCON0);  /* Line control register, 8N1 */
		__raw_writel(0x0245,  S3C2410_UCON0);   /* Control register */
		__raw_writel(ubrdiv,  S3C2410_UBRDIV0); /* Baud rate divisor register */

		__raw_writel(0x0000,  S3C2410_UFCON1);  /* FIFO control register, FIFO disable */
		__raw_writel(0x0000,  S3C2410_UMCON1);  /* Modem control register, AFC disable */
		__raw_writel(0x0003,  S3C2410_ULCON1);  /* Line control register, 8N1 */
		__raw_writel(0x0245,  S3C2410_UCON1);   /* Control register */
		__raw_writel(ubrdiv,  S3C2410_UBRDIV1); /* Baud rate divisor register */

		__raw_writel(0x0000,  S3C2410_UFCON2);  /* FIFO control register, FIFO disable */
		__raw_writel(0x0000,  S3C2410_UMCON2);  /* Modem control register, AFC disable */
		__raw_writel(0x0003,  S3C2410_ULCON2);  /* Line control register, 8N1 */
		__raw_writel(0x0245,  S3C2410_UCON2);   /* Control register */
		__raw_writel(ubrdiv,  S3C2410_UBRDIV2); /* Baud rate divisor register */

		__raw_writel(0x0000,  S3C2410_UFCON3);  /* FIFO control register, FIFO disable */
		__raw_writel(0x0000,  S3C2410_UMCON3);  /* Modem control register, AFC disable */
		__raw_writel(0x0003,  S3C2410_ULCON3);  /* Line control register, 8N1 */
		__raw_writel(0x0245,  S3C2410_UCON3);   /* Control register */
		__raw_writel(ubrdiv,  S3C2410_UBRDIV3); /* Baud rate divisor register */

		/* Turn on all UART clocks */
		__raw_writel(__raw_readl(S3C2443_PCLKCON) | 0xf, S3C2443_PCLKCON);

		/* Turn on all UART-related GPIO pins */
		__raw_writel((__raw_readl(S3C2410_GPHCON) & ~0xffffff) | 0xaaaaaa, S3C2410_GPHCON);
		
		/* turn off clocks we don't need */
		__raw_writel(__raw_readl(S3C2443_HCLKCON) & 
			~(S3C2443_HCLKCON_DISPCON | 
			  S3C2443_HCLKCON_CFC | 
			  S3C2443_HCLKCON_CAMIF), 
			S3C2443_HCLKCON);
		/* do not use the EPLL based UART, IIS, DISP clock (yet) */
		__raw_writel(__raw_readl(S3C2443_SCLKCON) &
			~(S3C2443_SCLKCON_CAMCLK |
			  S3C2443_SCLKCON_DISPCLK |
//			  S3C2443_SCLKCON_USBHOST |
			  S3C2443_SCLKCON_IISCLK |
			  S3C2443_SCLKCON_UARTCLK 
			),
			S3C2443_SCLKCON);

		return;
	case S3C2410_GSTATUS1_M2450:
		/* Don't touch anything, for now. */
		ubrdiv = UBRDIV(S3C2450_PCLK, CURRENT_BAUD_RATE);

		__raw_writel(0x0000,  S3C2410_UFCON0);  /* FIFO control register, FIFO disable */
		__raw_writel(0x0000,  S3C2410_UMCON0);  /* Modem control register, AFC disable */
		__raw_writel(0x0003,  S3C2410_ULCON0);  /* Line control register, 8N1 */
		__raw_writel(0x0245,  S3C2410_UCON0);   /* Control register */
		__raw_writel(ubrdiv,  S3C2410_UBRDIV0); /* Baud rate divisor register */

		__raw_writel(0x0000,  S3C2410_UFCON1);  /* FIFO control register, FIFO disable */
		__raw_writel(0x0000,  S3C2410_UMCON1);  /* Modem control register, AFC disable */
		__raw_writel(0x0003,  S3C2410_ULCON1);  /* Line control register, 8N1 */
		__raw_writel(0x0245,  S3C2410_UCON1);   /* Control register */
		__raw_writel(ubrdiv,  S3C2410_UBRDIV1); /* Baud rate divisor register */

		__raw_writel(0x0000,  S3C2410_UFCON2);  /* FIFO control register, FIFO disable */
		__raw_writel(0x0000,  S3C2410_UMCON2);  /* Modem control register, AFC disable */
		__raw_writel(0x0003,  S3C2410_ULCON2);  /* Line control register, 8N1 */
		__raw_writel(0x0245,  S3C2410_UCON2);   /* Control register */
		__raw_writel(ubrdiv,  S3C2410_UBRDIV2); /* Baud rate divisor register */

		__raw_writel(0x0000,  S3C2410_UFCON3);  /* FIFO control register, FIFO disable */
		__raw_writel(0x0000,  S3C2410_UMCON3);  /* Modem control register, AFC disable */
		__raw_writel(0x0003,  S3C2410_ULCON3);  /* Line control register, 8N1 */
		__raw_writel(0x0245,  S3C2410_UCON3);   /* Control register */
		__raw_writel(ubrdiv,  S3C2410_UBRDIV3); /* Baud rate divisor register */

		/* Turn on all UART clocks */
		confreg = __raw_readl(S3C2450_PCLKCON) | 0xf; 
		__raw_writel(confreg, S3C2450_PCLKCON);

		/* turn off clocks we don't need */
		confreg = __raw_readl(S3C2450_HCLKCON) & ~(S3C2450_HCLKCON_2D |
							   S3C2450_HCLKCON_IROM);
		__raw_writel(confreg, S3C2450_HCLKCON);

		/* do not use the EPLL based DISPCLK and UARTCLK */
		confreg = __raw_readl(S3C2450_SCLKCON) & ~(S3C2450_SCLKCON_DISPCLK |
							   S3C2450_SCLKCON_UARTCLK);
		__raw_writel(confreg, S3C2450_SCLKCON);
		return;
	default:
		s3c2412_detected = 1;
		// return; /* S3C2412 or unknown CPU, just don't touch anything */
		
		// 266 Mhz: do touch everything
		clkdivn =__raw_readl(S3C2410_CLKDIVN);
		clkdivn &= ~(0x1 << 2);
		clkdivn &= ~(0x3 << 0);
		clkdivn |= (S3C2412_PDIVN<<2) | S3C2412_HDIVN;
		ubrdiv = UBRDIV(S3C2412_PCLK, CURRENT_BAUD_RATE);
		mpllcon = (S3C2412_MDIV<<12) | (S3C2412_PDIV<<4) | S3C2412_SDIV;

		/* configure USB clock at 48 MHz */
		clksrc = __raw_readl(S3C2412_CLKSRC);
		clksrc	&= ~(3 << 12);
		clksrc	|= (0x2 << 12); // SELUREF UPLL reference = xtlpll (was OM4)
		clkdivn	|= (1<<6);		// USB 48 Div/2 on
		clksrc	|= (1<<5); 		// USYSCLK = FOUTupll
		clksrc	&= ~(1 << 10);  // USBSRCCLK = USYSCLK

		/* 2412 has different 12 Mhz UPLL setting */
		upllcon = 0x40070;
	}

	/* Setup all registers */
	__raw_writel(clkdivn, S3C2410_CLKDIVN); /* Clock divider */
        if (!s3c2412_detected) {
		__raw_writel(mpllcon, S3C2410_MPLLCON); /* MPLL configuration */
        }
	__raw_writel(upllcon, S3C2410_UPLLCON); /* UPLL configuration */
	
	if (s3c2412_detected) {
		__raw_writel(clksrc,  S3C2412_CLKSRC);  /* Clock divider */
	} else {
		__raw_writel(__raw_readl(S3C2410_CAMDIVN) & ~(3<<8), S3C2410_CAMDIVN); /* Clock divider */
	}

	__raw_writel(0x0000,  S3C2410_UFCON0);  /* FIFO control register, FIFO disable */
	__raw_writel(0x0000,  S3C2410_UMCON0);  /* Modem control register, AFC disable */
	__raw_writel(0x0003,  S3C2410_ULCON0);  /* Line control register, 8N1 */
	__raw_writel(0x0245,  S3C2410_UCON0);   /* Control register */
	__raw_writel(ubrdiv,  S3C2410_UBRDIV0); /* Baud rate divisor register */
	
	__raw_writel(	(ARB_MODE0 << 0) | (ARB_MODE1 << 1) | (ARB_MODE2 << 2) | 
					(ARB_MODE3 << 3) | (ARB_MODE4 << 4) | (ARB_MODE5 << 5) | 
					(ARB_MODE6 << 6) |
					(ARB_SEL0 << 7)  | (ARB_SEL1 << 9)  | (ARB_SEL2 << 11) | 
					(ARB_SEL3 << 13) | (ARB_SEL4 << 15) | (ARB_SEL5 << 17) | 
					(ARB_SEL6 << 19) ,
					S3C2410_PRIORITY);

	/* Delay for a little while (won't this be optimized away, KM) */
	for (i = 0; i < 100; ++i)
		;
}

/* EOF */
