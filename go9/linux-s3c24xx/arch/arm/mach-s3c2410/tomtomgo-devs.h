/* arch/arm/mach-s3c2410/tomtomgo-devs.h
 *
 * Declarations of TomTom GO platform devices.
 *
 * Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_S3C2410_TOMTOMGO_DEVS_H
#define __ARCH_ARM_MACH_S3C2410_TOMTOMGO_DEVS_H

#include <linux/config.h>
#include <linux/device.h>

//#if defined CONFIG_BARCELONA_IDE || defined CONFIG_BARCELONA_IDE_MODULE
extern struct platform_device tomtomgo_device_ide;
//#endif /* CONFIG_BARCELONA_IDE || CONFIG_BARCELONA_IDE_MODULE */

#if defined CONFIG_BARCELONA_HW || defined CONFIG_BARCELONA_HW_MODULE
extern struct platform_device tomtomgo_device_hw;
#endif /* CONFIG_BARCELONA_HW || CONFIG_BARCELONA_HW_MODULE */

#if defined CONFIG_BARCELONA_ADC || defined CONFIG_BARCELONA_ADC_MODULE
extern struct platform_device tomtomgo_device_adc;
#endif /* CONFIG_BARCELONA_ADC || CONFIG_BARCELONA_ADC_MODULE */

#if defined CONFIG_BARCELONA_GADC || defined CONFIG_BARCELONA_GADC_MODULE
extern struct platform_device tomtomgo_device_s3c24xx_gadc;
#endif /* CONFIG_BARCELONA_GADC || CONFIG_BARCELONA_GADC_MODULE */

#if defined CONFIG_BARCELONA_ACC || defined CONFIG_BARCELONA_ACC_MODULE
extern struct platform_device tomtomgo_device_acc;
#endif /* CONFIG_BARCELONA_ACC || CONFIG_BARCELONA_ACC_MODULE */

#if defined CONFIG_BARCELONA_BAT || defined CONFIG_BARCELONA_BAT_MODULE
extern struct platform_device tomtomgo_device_bat;
#endif /* CONFIG_BARCELONA_BAT || CONFIG_BARCELONA_BAT_MODULE */

#if defined CONFIG_BARCELONA_BUSPOWER || defined CONFIG_BARCELONA_BUSPOWER_MODULE
extern struct platform_device tomtomgo_device_buspower;
#endif /* CONFIG_BARCELONA_BUSPOWER || CONFIG_BARCELONA_BUSPOWER_MODULE */

#if defined CONFIG_BARCELONA_GPIO || defined CONFIG_BARCELONA_GPIO_MODULE
extern struct platform_device tomtomgo_device_gpio;
#endif /* CONFIG_BARCELONA_GPIO || CONFIG_BARCELONA_GPIO_MODULE */

#if defined CONFIG_BARCELONA_GPS || defined CONFIG_BARCELONA_GPS_MODULE
extern struct platform_device tomtomgo_device_gps;
#endif /* CONFIG_BARCELONA_GPS || CONFIG_BARCELONA_GPS_MODULE */

#if defined CONFIG_BARCELONA_PWM || defined CONFIG_BARCELONA_PWM_MODULE
extern struct platform_device tomtomgo_device_pwm;
#endif /* CONFIG_BARCELONA_PWM || CONFIG_BARCELONA_PWM_MODULE */

#if defined CONFIG_BACKLIGHT_S3C || defined CONFIG_BACKLIGHT_S3C_MODULE
extern struct platform_device tomtomgo_device_s3c;
#endif /* CONFIG_BACKLIGHT_S3C || CONFIG_BACKLIGHT_S3C_MODULE */

#if defined CONFIG_BACKLIGHT_S3C_LTC || defined CONFIG_BACKLIGHT_S3C_LTC_MODULE
extern struct platform_device tomtomgo_device_s3c_ltc;
#endif /* CONFIG_BACKLIGHT_S3C_LTC || CONFIG_BACKLIGHT_S3C_MODULE_LTC */

#if defined CONFIG_BARCELONA_RC || defined CONFIG_BARCELONA_RC_MODULE
extern struct platform_device tomtomgo_device_rc;
#endif /* CONFIG_BARCELONA_RC || CONFIG_BARCELONA_RC_MODULE */

#if defined CONFIG_BARCELONA_TS || defined CONFIG_BARCELONA_TS_MODULE
extern struct platform_device tomtomgo_device_ts;
#endif /* CONFIG_BARCELONA_TS || CONFIG_BARCELONA_TS_MODULE */

#if defined CONFIG_BARCELONA_TSINPUT || defined CONFIG_BARCELONA_TSINPUT_MODULE
extern struct platform_device tomtomgo_device_tsinput;
#endif /* CONFIG_BARCELONA_TSINPUT || CONFIG_BARCELONA_TSINPUT_MODULE */

#if defined CONFIG_BARCELONA_USBMODE|| defined CONFIG_BARCELONA_USBMODE_MODULE
extern struct platform_device tomtomgo_device_usbmode;
#endif /* CONFIG_BARCELONA_USBMODE|| CONFIG_BARCELONA_USBMODE_MODULE*/


#if defined CONFIG_TOMTOMGO_TOUCHPAD || defined CONFIG_TOMTOMGO_TOUCHPAD_MODULE
extern struct platform_device tomtomgo_device_tp;
#endif /* CONFIG_TOMTOMGO_TOUCHPAD || CONFIG_TOMTOMGO_TOUCHPAD_MODULE */

#if defined CONFIG_BARCELONA_BUZ || defined CONFIG_BARCELONA_BUZ_MODULE
extern struct platform_device tomtomgo_device_buz;
#endif /* CONFIG_BARCELONA_BUZ || CONFIG_BARCELONA_BUZ_MODULE */

#if defined CONFIG_BARCELONA_EXTERNAL_UART
#include <linux/serial_8250.h>
#include "s3c2410-bus.h"
extern struct s3c2410_bank_settings tomtomgo_external_uart_bus_settings[];
extern struct plat_serial8250_port tomtomgo_external_uart_ports[];
extern struct platform_device tomtomgo_device_external_uart;
#endif /* CONFIG_BARCELONA_EXTERNAL_UART */

#if defined(CONFIG_SPI_MASTER)
#include <linux/spi/spi.h>
#include <asm/arch/spi.h>
enum {
#if defined(CONFIG_BARCELONA_GACC_SMB365) || defined(CONFIG_BARCELONA_GACC_SMB365_MODULE) 
	SPIDEV_SMB365, 
#endif /* CONFIG_BARCELONA_GACC_SMB365 || CONFIG_BARCELONA_GACC_SMB365_MODULE */
#if defined(CONFIG_BARCELONA_GACC_KXP74) || defined(CONFIG_BARCELONA_GACC_KXP74_MODULE) 
	SPIDEV_KXP74, 
#endif /* CONFIG_BARCELONA_GACC_KXP74 || CONFIG_BARCELONA_GACC_KXP74MODULE	*/
#if defined CONFIG_BARCELONA_GADC || defined CONFIG_BARCELONA_GADC_MODULE
#if defined CONFIG_BARCELONA_BARO || defined CONFIG_BARCELONA_BARO_MODULE
	SPIDEV_SCP1000,
#endif /* CONFIG_BARCELONA_BARO || CONFIG_BARCELONA_BARO_MODULE */
#if defined CONFIG_BARCELONA_GADC_KXR94 || defined CONFIG_BARCELONA_GADC_KXR94_MODULE
	SPIDEV_KXR94_GADC,
#endif /* CONFIG_BARCELONA_GADC_KXR94 || CONFIG_BARCELONA_GADC_KXR94_MODULE */
#endif /* CONFIG_BARCELONA_GADC || CONFIG_BARCELONA_GADC_MODULE */
	SPIDEV_DUMMY	/* not in use  but prevent compiler errors if enum would end up empty */
};

#if defined(CONFIG_BARCELONA_GACC_KXP74) || defined(CONFIG_BARCELONA_GACC_KXP74_MODULE) 
extern struct gadc_platform_data tomtomgo_kxr94_adc_info;
#endif /* CONFIG_BARCELONA_GACC_KXP74 || CONFIG_BARCELONA_GACC_KXP74MODULE	*/

#if defined(CONFIG_SPI_S3C24XX_GPIO) || defined(CONFIG_SPI_S3C24XX_GPIO_MODULE)
extern struct platform_device s3c24xx_device_spigpio;
#endif /* SPI_S3C24XX_GPIO || SPI_S3C24XX_GPIO_MODULE	*/

#if defined(CONFIG_SPI_S3C24XX) || defined(CONFIG_SPI_S3C24XX_MODULE)
extern struct platform_device tomtomgo_device_spi1;
#endif /* SPI_S3C24XX || SPI_S3C24XX_MODULE	*/

#if defined(CONFIG_SPI_S3C24XX_GPIO) || defined(CONFIG_SPI_S3C24XX_GPIO_MODULE) || defined(CONFIG_SPI_S3C24XX) || defined(CONFIG_SPI_S3C24XX_MODULE) 
extern struct spi_board_info tomtomgo_spi_board_info[];
extern struct s3c2410_spi_info tomtomgo_spi_info;
#endif /* SPI_S3C24XX_GPIO || SPI_S3C24XX_GPIO_MODULE	|| CONFIG_SPI_S3C24XX) || CONFIG_SPI_S3C24XX*/ 

#endif /*	 CONFIG_SPI_MASTER */

#if defined CONFIG_LTC3555_PMIC || defined CONFIG_LTC3555_PMIC_MODULE
extern struct platform_device	tomtomgo_device_s3c24xx_ltc3555_pmic;
#endif /* CONFIG_LTC3555_PMIC */

#if defined CONFIG_LTC3577_PMIC || defined CONFIG_LTC3577_PMIC_MODULE
extern struct platform_device	tomtomgo_device_s3c24xx_ltc3577_pmic;
#endif /* CONFIG_LTC3577_PMIC */

#if defined CONFIG_BARCELONA_LTC3455 || defined CONFIG_BARCELONA_LTC3455_MODULE
extern struct platform_device	tomtomgo_device_s3c24xx_ltc3455;
#endif /* CONFIG_BARCELONA_LTC3455 */

extern struct barcelona_sound_info tomtomgo_sound_info;
#if defined (CONFIG_BARCELONA_GYRO_FUJITSU) || defined (CONFIG_BARCELONA_GYRO_FUJITSU_MODULE) || \
	defined(CONFIG_BARCELONA_GYRO_ING300) || defined(CONFIG_BARCELONA_GYRO_ING300_MODULE)
extern struct gyro_platform_data tomtomgo_gyro_info;
extern struct platform_device tomtomgo_device_gyro;
#endif /*	defined(CONFIG_BARCELONA_GYRO) || defined(CONFIG_BARCELONA_GYRO_MODULE) */

#if defined( CONFIG_I2C )

#if defined( CONFIG_SI4703_FM ) || defined ( CONFIG_SI4703_FM_MODULE )
extern struct platform_device tomtomgo_device_tmcrx_si4703;
#endif /* defined( CONFIG_SI4703_FM) */

#if defined( CONFIG_SI4705_FM )
extern struct platform_device tomtomgo_device_tmcrx_si4705;
#endif /* defined( CONFIG_SI4705_FM) */

#if defined( CONFIG_SI4710_FM ) 
extern struct platform_device tomtomgo_device_fmx;
#endif /* defined( CONFIG_SI4710_FM )	*/ 

#ifdef CONFIG_I2C_S3C24XX_GPIO
extern struct platform_device s3c24xx_gpio_i2c_controller;

#endif /* ifdef I2C_S3C24XX_GPIO */
#endif /* CONFIG_I2C */

#ifdef CONFIG_BARCELONA_DOCK
extern struct platform_device s3c24xx_dock_dongle;
extern struct platform_device s3c24xx_dock_cagliari_dongle;
#endif /* CONFIG_BARCELONA_DOCK */

#ifdef CONFIG_GPRS_INTERFACE
extern struct platform_device tomtomgo_gprs_modem;
#endif /* CONFIG_GPRS_INTERFACE */
#endif /* __ARCH_ARM_MACH_S3C2410_TOMTOMGO_DEVS_H */

/* EOF */
