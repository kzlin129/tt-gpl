/* arch/arm/mach-s3c2410/devs.h
 *
 * Copyright (c) 2004 Simtec Electronics
 * Ben Dooks <ben@simtec.co.uk>
 *
 * Header file for s3c2410 standard platform devices
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications:
 *      18-Aug-2004 BJD  Created initial version
 *	27-Aug-2004 BJD  Added timers 0 through 3
 *	10-Feb-2005 BJD	 Added camera from guillaume.gourat@nexvision.tv
*/
#include <linux/config.h>
#include <linux/device.h>
#include <asm/arch/iic.h>

extern struct platform_device s3c_device_usb;
extern struct platform_device s3c_device_lcd;
extern struct platform_device s3c_device_wdt;
extern struct platform_device s3c_device_i2c;
extern struct platform_device s3c_device_iis;
extern struct platform_device s3c_device_rtc;
extern struct platform_device s3c_device_adc;
extern struct platform_device s3c_device_sdi;

extern struct platform_device s3c_device_spi0;
extern struct platform_device s3c_device_spi1;

extern struct platform_device s3c_device_nand;

extern struct platform_device s3c_device_timer0;
extern struct platform_device s3c_device_timer1;
extern struct platform_device s3c_device_timer2;
extern struct platform_device s3c_device_timer3;

extern struct platform_device s3c_device_usbgadget;

/* s3c2412 specific devices */

#ifdef CONFIG_CPU_S3C2412
#ifdef CONFIG_VIDEO_S3C241X_CAMIF
#define S3C241X_CAMIF_BOOTMEM_IDX		2		/* This one should match the index of the resource */
								/* descriptor for the memory to be allocated using */
								/* alloc_bootmem_low. */
#define S3C241X_CAMIF_MAX_WIDTH			1280		/* Maximum width to be used in pixels. */
#define S3C241X_CAMIF_MAX_HEIGHT		1024		/* Maximum height to be used in pixels. */
#define S3C241X_CAMIF_MAX_DEPTH			16		/* Maximum depth in bits to be used. */
#define S3C241X_CAMIF_BOOTMEM_SIZE		((S3C241X_CAMIF_MAX_WIDTH * S3C241X_CAMIF_MAX_HEIGHT * \
						  S3C241X_CAMIF_MAX_DEPTH / 8) + PAGE_SIZE)
								/* One page extra to ensure the buffer is aligned on */
								/* page boundary. */
#endif /* CONFIG_VIDEO_S3C241X_CAMIF */
extern struct platform_device s3c_device_camif;
extern struct platform_device s3c_device_i2c_camif;

#endif /* CONFIG_CPU_S3C2412 */

/* s3c2443/s3c2450 specific devices */
#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450)
extern struct platform_device s3c_device_hsmmc0;
extern struct platform_device s3c_device_hsmmc1;
extern struct platform_device s3c_device_hsmmc;
extern struct platform_device s3c_device_hsudc;
extern struct platform_device s3c_device_tft_lcd;
#endif /* CONFIG_CPU_S3C2443 || CONFIG_CPU_S3C2450 */

/* Steppingstone */
#if 1 /* TODO: prepare infrastructure for SRAM */
extern struct platform_device s3c_device_socsram;
#endif

/* I2C configuration */
#if defined(CONFIG_I2C_S3C2410)
extern struct s3c2410_platform_i2c s3c_i2c_settings;
#endif /* CONFIG_I2C_S3C2410 */

