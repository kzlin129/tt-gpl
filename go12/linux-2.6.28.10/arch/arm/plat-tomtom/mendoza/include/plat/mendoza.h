/*
 * Configuration for Mendoza carrier board
 *
 * Copyright 2009 TomTom B.V.
 *      Marc Zyngier <marc.zyngier@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _PLAT_MENDOZA_H
#define _PLAT_MENDOZA_H

#include <linux/vgpio.h>

/* Virtual GPIOs
 * Cordoba support adds a lot of complexity because of the I2C GPIO expander
 * Bcause of this expander, we must have all the I2C GPIOs as part of a
 * second VGPIO chip, which have the expander as parent. Be careful when
 * adding any pin here...
 *
 * Also make sure that VGPIO_BASE is not too low (or it would clash with
 * real GPIOs), nor too high (would excess the standard limit of 256 GPIOs
 * for gpiolib). For reference, S3C6410 has 204 defined GPIOs.
 */

#define MENDOZA_VGPIO_BASE	210

enum mendoza_vgpio_pins {
	// VGPIO0
	TT_VGPIO0_BASE = MENDOZA_VGPIO_BASE,

	TT_VGPIO_PU_I2C0 = TT_VGPIO0_BASE, // TorinoS: TT_VGPIO_I2C_SWPWR
	TT_VGPIO_PU_I2C1,		// TorinoS: TT_VGPIO_DOCK_I2C_SWPWR
	TT_VGPIO_BOOT_DEV0,
	TT_VGPIO_SD0_PWR_EN,
	TT_VGPIO_BARRACUDA_CS,
	TT_VGPIO_BACKLIGHT_EN,
	TT_VGPIO_LCM_RESET,
	TT_VGPIO_PWR_BUTTON,		// TorinoS: TT_VGPIO_ON_OFF
	TT_VGPIO_RFS_BOOT_CLK,		// TorinoS: TT_VGPIO_RFS_BOOT_CLOCK 
	TT_VGPIO_RFS_BOOT_Q,
	TT_VGPIO_USB_HOST_DETECT,
	TT_VGPIO_GPS_STANDBY, 
	TT_VGPIO_GPS_RESET,
	TT_VGPIO_BT_RST,
	TT_VGPIO_WALL_ON,
	TT_VGPIO_ID_PIN,
	TT_VGPIO_SD_CD,
	TT_VGPIO_SD_BASE,
	TT_VGPIO_SD_BOOT_BASE,
	TT_VGPIO_MIC_STBY,
	TT_VGPIO_L3_MODE,
	TT_VGPIO_GPS_1PPS,
	TT_VGPIO_GPS_CS,
	TT_VGPIO_PWR_KILL,
	TT_VGPIO_USB_3V3_ON,
	TT_VGPIO_USB_1V2_ON,
	TT_VGPIO_DOCK_RESET,
	TT_VGPIO_DOCK_DET0,
	TT_VGPIO_DOCK_DET1,
	TT_VGPIO_ACCESSORY_PWR_EN,
	TT_VGPIO_DOCK_DET_PWR,
	TT_VGPIO_DOCK_FM_INT,
	TT_VGPIO_DOCK_I2C_EN,
	TT_VGPIO_HPDETECT,
	TT_VGPIO_HRESET,
	TT_VGPIO_CHARGING,
	TT_VGPIO_DOCK_TX_SOC_RX,
	TT_VGPIO_MIC_RESET,
	TT_VGPIO_MUTE,
	TT_VGPIO_LCM_ID,
	TT_VGPIO_TSP_ATTN,
	TT_VGPIO_TSP_CE,
	TT_VGPIO_TSP_PWR,
	TT_VGPIO_DR_PWR_EN,
	TT_VGPIO_KXR94_CS,
	TT_VGPIO_BARO_CS,
	TT_VGPIO_TILT_PWR,
	TT_VGPIO_TILT_OUT,

	// VGPIO1
	TT_VGPIO1_BASE,

	TT_VGPIO_GSM_SYS_EN = TT_VGPIO1_BASE,
	TT_VGPIO_GSM_SYS_RST,
	TT_VGPIO_LCM_CS,		// TorinoS: LCM_DISP_ON
	TT_VGPIO_LCM_PWR_EN,
	TT_VGPIO_CODEC_PWR_EN,
	TT_VGPIO_AMP_PWR_EN,
	TT_VGPIO_LCD_SPI_CS,
	TT_VGPIO_LCD_SPI_SDI,
	TT_VGPIO_LCD_SPI_SCL
};

extern struct platform_device mendoza_device_battery;
extern struct platform_device mendoza_device_vbus;

void mendoza_s3cfb_setup(char *);
void mendoza_sdhci_setup(void);
void mendoza_i2c_setup(int bitmap);
int  tomtom_bl_setup(void);
int gprs_init( void );

#endif 
