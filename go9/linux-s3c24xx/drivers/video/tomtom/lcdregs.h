/* drivers/video/tomtom/lcdregs.h
 *
 * LCD register macros for compatibility with our bootloader.
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DRIVERS_VIDEO_TOMTOMGO_LCDREGS_H
#define __DRIVERS_VIDEO_TOMTOMGO_LCDREGS_H

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <asm/arch/map.h>
#include <asm/arch/regs-lcd.h>
#include <asm/arch/regs-dyn.h>
#endif /* __KERNEL__ */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __KERNEL__

/* Miscellaneous GPIO control register */
#define rMISCCR     (*(volatile unsigned *)(s3c24xx_misccr))

/* S3C2410 LCD controller registers */
#define rLCDCON1    (*(volatile unsigned *)(S3C2410_LCDCON1))
#define rLCDCON2    (*(volatile unsigned *)(S3C2410_LCDCON2))
#define rLCDCON3    (*(volatile unsigned *)(S3C2410_LCDCON3))
#define rLCDCON4    (*(volatile unsigned *)(S3C2410_LCDCON4))
#define rLCDCON5    (*(volatile unsigned *)(S3C2410_LCDCON5))
#define rLCDSADDR1  (*(volatile unsigned *)(S3C2410_LCDSADDR1))
#define rLCDSADDR2  (*(volatile unsigned *)(S3C2410_LCDSADDR2))
#define rLCDSADDR3  (*(volatile unsigned *)(S3C2410_LCDSADDR3))
#define rLCDINTMSK  (*(volatile unsigned *)(S3C2410_LCDINTMSK))
#define rTCONSEL    (*(volatile unsigned *)(S3C2410_LPCSEL))
#define rTPAL       (*(volatile unsigned *)(S3C2410_TPAL))

/* S32C2443 TFT LCD controller registers */
#define rVIDCON0      (*(volatile unsigned *)(S3C2443_VIDCON0))
#define rVIDCON1      (*(volatile unsigned *)(S3C2443_VIDCON1))
#define rVIDTCON0     (*(volatile unsigned *)(S3C2443_VIDTCON0))
#define rVIDTCON1     (*(volatile unsigned *)(S3C2443_VIDTCON1))
#define rVIDTCON2     (*(volatile unsigned *)(S3C2443_VIDTCON2))
#define rWINCON0      (*(volatile unsigned *)(S3C2443_WINCON0))
#define rWINCON1      (*(volatile unsigned *)(S3C2443_WINCON1))
#define rVIDOSD0A     (*(volatile unsigned *)(S3C2443_VIDOSD0A))
#define rVIDOSD0B     (*(volatile unsigned *)(S3C2443_VIDOSD0B))
#define rVIDOSD1A     (*(volatile unsigned *)(S3C2443_VIDOSD1A))
#define rVIDOSD1B     (*(volatile unsigned *)(S3C2443_VIDOSD1B))
#define rVIDOSD1C     (*(volatile unsigned *)(S3C2443_VIDOSD1C))
#define rVIDW00ADD0B0 (*(volatile unsigned *)(S3C2443_VIDW00ADD0B0))
#define rVIDW00ADD0B1 (*(volatile unsigned *)(S3C2443_VIDW00ADD0B1))
#define rVIDW01ADD0   (*(volatile unsigned *)(S3C2443_VIDW01ADD0 ))
#define rVIDW00ADD1B0 (*(volatile unsigned *)(S3C2443_VIDW00ADD1B0))
#define rVIDW00ADD1B1 (*(volatile unsigned *)(S3C2443_VIDW00ADD1B1))
#define rVIDW01ADD1   (*(volatile unsigned *)(S3C2443_VIDW01ADD1))
#define rVIDW00ADD2B0 (*(volatile unsigned *)(S3C2443_VIDW00ADD2B0))
#define rVIDW00ADD2B1 (*(volatile unsigned *)(S3C2443_VIDW00ADD2B1))
#define rVIDW01ADD2   (*(volatile unsigned *)(S3C2443_VIDW01ADD2))
#define rVIDINTCON    (*(volatile unsigned *)(S3C2443_VIDINTCON))
#define rW1KEYCON0    (*(volatile unsigned *)(S3C2443_W1KEYCON0))
#define rW1KEYCON1    (*(volatile unsigned *)(S3C2443_W1KEYCON1))
#define rWIN0MAP      (*(volatile unsigned *)(S3C2443_WIN0MAP))
#define rWIN1MAP      (*(volatile unsigned *)(S3C2443_WIN1MAP))
#define rSYSIFCON0    (*(volatile unsigned *)(S3C2443_SYSIFCON0))
#define rSYSIFCON1    (*(volatile unsigned *)(S3C2443_SYSIFCON1))
#define rDITHMODE     (*(volatile unsigned *)(S3C2443_DITHMODE))
#define rCPUTRIGCON2  (*(volatile unsigned *)(S3C2443_CPUTRIGCON2))
#endif /* __KERNEL__ */

#ifdef __BOOTLOADER__
#include "24x0addr.h"
#define S3C2410_LCDBANK(x)      ((x) << 21)
#define S3C2410_LCDBASEU(x)	(x)
#define S3C2410_OFFSIZE(x)	((x) << 11)
#define S3C2410_PAGEWIDTH(x)	(x)
#define NULL 0
#endif /* __BOOTLOADER__ */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DRIVERS_VIDEO_TOMTOMGO_LCDREGS_H */

/* EOF */
