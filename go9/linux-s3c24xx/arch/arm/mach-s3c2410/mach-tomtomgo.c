/* arch/arm/mach-s3c2410/mach-tomtomgo.c
 *
 * Machine support for the TomTom GO, an all-in-one car navigation product.
 * See <http://www.tomtomgo.com/>.
 *
 * Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
 * Authors: Dimitry Andric <dimitry.andric@tomtom.com>
 *          Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/bootmem.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/arch/regs-serial.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-irq.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/tomtomgo-irq.h>
#include <asm/arch/tomtomgo-map.h>
#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include <barcelona/Barc_snd.h>
#include <barcelona/gyro.h>

#include <asm/arch/pm.h>
#include <asm/hardware/clock.h>

#include "s3c2410.h"
#include "s3c2440.h"
#include "clock.h"
#include "devs.h"
#include "tomtomgo-devs.h"
#include "tomtomgo-clocks.h"
#include "cpu.h"
#include "s3c2410-bus.h"
#include <linux/si4703.h>
#include <linux/si4710.h>

#if defined CONFIG_BARCELONA_LTC3455
#include <linux/ltc3455-pmic.h>
#endif
#if defined CONFIG_LTC3555_PMIC
#include <linux/ltc3555-pmic.h>
#endif
#if defined CONFIG_LTC3577_PMIC
#include <linux/ltc3577-pmic.h>
#endif
#if defined(CONFIG_SPI_MASTER)
#include <linux/spi/spi.h>
#include <asm/arch/spi-gpio.h>
#include <barcelona/gacc.h>
#endif /* CONFIG_SPI_MASTER	*/

#ifdef CONFIG_I2C_S3C24XX_GPIO
#include <barcelona/s3c24xx_gpio_i2c.h>
#endif

static struct map_desc tomtomgo_iodesc[] __initdata = {
	{ (u32)S3C24XX_VA_USBDEV, S3C2410_PA_USBDEV, S3C24XX_SZ_USBDEV, MT_DEVICE },
	{ (u32)S3C_VA_HSUDC,      S3C_PA_HSUDC,      S3C_SZ_HSUDC,      MT_DEVICE },
	{ (u32)S3C24XX_VA_SDI,    S3C2410_PA_SDI,    S3C24XX_SZ_SDI,    MT_DEVICE },
	{ (u32)S3C2443_VA_HSMMC0, S3C2443_PA_HSMMC0, S3C2443_SZ_HSMMC0, MT_DEVICE },
	{ (u32)S3C2443_VA_HSMMC1, S3C2443_PA_HSMMC1, S3C2443_SZ_HSMMC1, MT_DEVICE },
	{ (u32)S3C24XX_VA_IIS,    S3C2410_PA_IIS,    S3C24XX_SZ_IIS,    MT_DEVICE },
	{ (u32)TOMTOMGO_VA_IDE,   TOMTOMGO_PA_IDE,   TOMTOMGO_SZ_IDE,   MT_DEVICE },
	{ (u32)S3C24XX_VA_CAMIF,  S3C2413_PA_CAMIF,  S3C24XX_SZ_CAMIF,  MT_DEVICE },
	{ (u32)S3C24XX_VA_RTC,	  S3C2410_PA_RTC,    S3C24XX_SZ_RTC,    MT_DEVICE },
	{ (u32)S3C24XX_VA_MEMCTRL,S3C2410_PA_MEMCTRL,S3C24XX_SZ_MEMCTRL,MT_DEVICE },
	{ (u32)S3C2443_VA_TFTLCD, S3C2443_PA_TFTLCD, S3C2443_SZ_TFTLCD, MT_DEVICE },
};

#define UCON	(S3C2410_UCON_TXILEVEL  | \
		 S3C2410_UCON_RXILEVEL  | \
		 S3C2410_UCON_TXIRQMODE | \
		 S3C2410_UCON_RXIRQMODE | \
		 S3C2410_UCON_RXFIFO_TOI)
#define ULCON	(S3C2410_LCON_CS8 | \
		 S3C2410_LCON_PNONE)

#define UFCON_2440	(S3C2410_UFCON_FIFOMODE | \
			 S3C2440_UFCON_TXTRIG0  | \
			 S3C2440_UFCON_RXTRIG32)

#define UFCON_2410	(S3C2410_UFCON_FIFOMODE | \
			 S3C2410_UFCON_TXTRIG0  | \
			 S3C2410_UFCON_RXTRIG12)

static struct s3c2410_uartcfg tomtomgo_uartcfgs[] = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON_2440,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON_2440,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON_2440,
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON_2440,
	},
};


/* Standard TomTom GO devices */

static struct platform_device *tomtomgo_devices[] __initdata = {
	&s3c_device_usb,
	&s3c_device_wdt,
	&s3c_device_rtc,
#if defined CONFIG_BARCELONA_HW || defined CONFIG_BARCELONA_HW_MODULE
	&tomtomgo_device_hw,
#endif /* CONFIG_BARCELONA_HW || CONFIG_BARCELONA_HW_MODULE */
#if defined CONFIG_BARCELONA_ADC || defined CONFIG_BARCELONA_ADC_MODULE
	&tomtomgo_device_adc,
#endif /* CONFIG_BARCELONA_ADC || CONFIG_BARCELONA_ADC_MODULE */
#if defined CONFIG_BARCELONA_ACC || defined CONFIG_BARCELONA_ACC_MODULE
	&tomtomgo_device_acc,
#endif /* CONFIG_BARCELONA_ACC || CONFIG_BARCELONA_ACC_MODULE */
#if defined CONFIG_BARCELONA_GADC || defined CONFIG_BARCELONA_GADC_MODULE
	&tomtomgo_device_s3c24xx_gadc,
#endif /* CONFIG_BARCELONA_GADC || CONFIG_BARCELONA_GADC_MODULE */
#if defined CONFIG_BARCELONA_BAT || defined CONFIG_BARCELONA_BAT_MODULE
	&tomtomgo_device_bat,
#endif /* CONFIG_BARCELONA_BAT || CONFIG_BARCELONA_BAT_MODULE */
#if defined CONFIG_BARCELONA_BUSPOWER || defined CONFIG_BARCELONA_BUSPOWER_MODULE
	&tomtomgo_device_buspower,
#endif /* CONFIG_BARCELONA_BUSPOWER || CONFIG_BARCELONA_BUSPOWER_MODULE */
#if defined CONFIG_BARCELONA_GPIO || defined CONFIG_BARCELONA_GPIO_MODULE
	&tomtomgo_device_gpio,
#endif /* CONFIG_BARCELONA_GPIO || CONFIG_BARCELONA_GPIO_MODULE */
#if defined CONFIG_BARCELONA_GPS || defined CONFIG_BARCELONA_GPS_MODULE
	&tomtomgo_device_gps,
#endif /* CONFIG_BARCELONA_GPS || CONFIG_BARCELONA_GPS_MODULE */
#if defined CONFIG_BARCELONA_PWM || defined CONFIG_BARCELONA_PWM_MODULE
	&tomtomgo_device_pwm,
#endif /* CONFIG_BARCELONA_PWM || CONFIG_BARCELONA_PWM_MODULE */
#if defined CONFIG_BARCELONA_SOCSRAM
	&s3c_device_socsram,
#endif
};

static struct s3c24xx_board tomtomgo_board __initdata = {
	.devices       = tomtomgo_devices,
	.devices_count = ARRAY_SIZE(tomtomgo_devices)
};

#ifdef CONFIG_SMDK2440_BOARD_NOT_ANYMORE
#define TOMTOMGO_XTAL 16934400
#else
#define TOMTOMGO_XTAL 12000000
#endif

static void __init tomtomgo_map_io(void)
{
	s3c24xx_init_io(tomtomgo_iodesc, ARRAY_SIZE(tomtomgo_iodesc));
	s3c24xx_init_clocks(TOMTOMGO_XTAL);
	s3c24xx_init_uarts(tomtomgo_uartcfgs, ARRAY_SIZE(tomtomgo_uartcfgs));
	s3c24xx_set_board(&tomtomgo_board);
	IO_Init();

	/* adjust uarts for 16-byte 2410 FIFO */
	if (IO_GetCpuType() == GOCPU_S3C2410) {
		int i;
		for (i=0; i<2; i++) {
			tomtomgo_uartcfgs[i].ufcon = UFCON_2410;
		}
	}

#if defined CONFIG_VIDEO_S3C241X_CAMIF
	if( IO_HaveCamera() )
	{
		/* Allocate the memory for the picture buffer. Ensure that the buffer starts aligned on page boundary. */
		/* As this buffer is never released, it doesn't matter that we lose the start address. */
		s3c_device_camif.resource[S3C241X_CAMIF_BOOTMEM_IDX].start=(__u32) alloc_bootmem_low( S3C241X_CAMIF_BOOTMEM_SIZE );
		if( s3c_device_camif.resource[S3C241X_CAMIF_BOOTMEM_IDX].start == 0 )
			printk( KERN_WARNING "WARNING! Could not allocate framebuffer for camera capture. DO NOT USE CAMERA\n" );
		else
		{       
			/* In the S3C241X_CAMIF_BOOTMEM_SIZE the alignment on page boundary has already been taken in account. */
			/* No need to allocate extra memory for this. */
			s3c_device_camif.resource[S3C241X_CAMIF_BOOTMEM_IDX].end=
					s3c_device_camif.resource[S3C241X_CAMIF_BOOTMEM_IDX].start + S3C241X_CAMIF_BOOTMEM_SIZE;
			s3c_device_camif.resource[S3C241X_CAMIF_BOOTMEM_IDX].start=
					PAGE_ALIGN( s3c_device_camif.resource[S3C241X_CAMIF_BOOTMEM_IDX].start );
		}       
	}
#endif /* CONFIG_VIDEO_S3C241X_CAMIF */
}

int
tomtomgo_setup_dynamic_devices(void)
{
	/* Initialize clock structures depending on hardware type	*/
	tomtomgo_init_clocks( );

#ifdef CONFIG_CPU_S3C2450
	if (IO_GetCpuType() == GOCPU_S3C2450)
		platform_device_register( &s3c_device_tft_lcd );
	else
#endif
	platform_device_register( &s3c_device_lcd );

#if defined(CONFIG_BACKLIGHT_S3C)  ||  defined(CONFIG_BACKLIGHT_S3C_MODULE)
	if( IO_GetChargerType( ) != GOCHARGER_LTC3577 ) {
		platform_device_register( &tomtomgo_device_s3c );
	}
#endif /* CONFIG_BACKLIGHT_S3C || CONFIG_BACKLIGHT_S3C_MODULE */

#if defined(CONFIG_I2C_S3C2410) && defined(CONFIG_FM2010_DSP)
	if (IO_HaveForteMediaDSP() && IO_HaveHardwareI2C()) {
		/* Limit speed of I2C bus as ForteMedia DSP can't cope with it otherwise */
		s3c_i2c_settings.bus_freq = 100*1000;
		s3c_i2c_settings.max_freq = 100*1000;
	}
#endif /* CONFIG_I2C_S3C2410 */

#if defined CONFIG_I2C_S3C2410 || defined CONFIG_I2C_S3C2410_MODULE
	if (IO_HaveHardwareI2C())
		platform_device_register(&s3c_device_i2c);
#endif /* CONFIG_I2C_S3C2410 || CONFIG_I2C_S3C2410_MODULE */

#ifdef CONFIG_I2C
#ifdef CONFIG_I2C_S3C24XX_GPIO
	if(IO_HaveSoftwareI2C()) {
		((struct s3c24xx_gpio_i2c_pins *) (s3c24xx_gpio_i2c_controller.dev.platform_data))->scl_pin = IO_Pin(SW_SCL);
		((struct s3c24xx_gpio_i2c_pins *) (s3c24xx_gpio_i2c_controller.dev.platform_data))->sda_pin = IO_Pin(SW_SDA);
		platform_device_register( &s3c24xx_gpio_i2c_controller );
	}
#endif /* I2C_S3C24XX_GPIO */
#endif /*  CONFIG_I2C */

#if defined(CONFIG_LTC3555_PMIC) ||  defined(CONFIG_LTC3577_PMIC)
	/* If we have the LTC3555 or LTC3577 PMIC we need to register it first, to ensure its the last */
	/* device called on suspend/resume. */
	if( IO_GetChargerType( ) == GOCHARGER_LTC3555 ) {
		((struct ltc3555_platform *) tomtomgo_device_s3c24xx_ltc3555_pmic.dev.platform_data)->acpwr_pin=
			IO_Pin( ACPWR );
		((struct ltc3555_platform *) tomtomgo_device_s3c24xx_ltc3555_pmic.dev.platform_data)->wall_pwr_pin=
			IO_Pin( USB_PWR_BYPASS );
		platform_device_register( &tomtomgo_device_s3c24xx_ltc3555_pmic );
	}

	if( IO_GetChargerType( ) == GOCHARGER_LTC3577 ) {
		((struct ltc3577_platform *) tomtomgo_device_s3c24xx_ltc3577_pmic.dev.platform_data)->acpwr_pin=
			IO_Pin( ACPWR );
		((struct ltc3577_platform *) tomtomgo_device_s3c24xx_ltc3577_pmic.dev.platform_data)->wall_pwr_pin=
			IO_Pin( USB_PWR_BYPASS );
		platform_device_register( &tomtomgo_device_s3c24xx_ltc3577_pmic );
	}
#endif /* CONFIG_LTC3555_PMIC || CONFIG_LTC3577_PMIC*/

#if defined(CONFIG_BACKLIGHT_S3C_LTC)  ||  defined(CONFIG_BACKLIGHT_S3C_LTC_MODULE)
	if( IO_GetChargerType( ) == GOCHARGER_LTC3577 ) {
		platform_device_register( &tomtomgo_device_s3c_ltc );
	}
#endif /* CONFIG_BACKLIGHT_S3C_LTC || CONFIG_BACKLIGHT_S3C_LTC_MODULE */

#if defined CONFIG_BARCELONA_LTC3455
	/* If we have the LTC3455 PMIC we need to register it first, to ensure its the last */
	/* device called on suspend/resume. */
//	if( (IO_GetChargerType( ) == GOCHARGER_LTC3455) && (IO_HasPin( LOW_CORE )) )
	if( ( IO_GetChargerType( ) == GOCHARGER_LTC3455) )
	{
		((struct ltc3455_pdata *) tomtomgo_device_s3c24xx_ltc3455.dev.platform_data)->acpwr_pin=
			IO_Pin( ACPWR );
		((struct ltc3455_pdata *) tomtomgo_device_s3c24xx_ltc3455.dev.platform_data)->wall_pwr_pin=
			IO_Pin( USB_PWR_BYPASS );
//		((struct ltc3455_pdata *) tomtomgo_device_s3c24xx_ltc3455.dev.platform_data)->low_core_pin=
//			IO_Pin( LOW_CORE );
		((struct ltc3455_pdata *) tomtomgo_device_s3c24xx_ltc3455.dev.platform_data)->pwr_mode_pin=
			IO_Pin( PWR_MODE );
		platform_device_register( &tomtomgo_device_s3c24xx_ltc3455 );
	}
#endif /* CONFIG_BARCELONA_LTC3455 */

#if defined CONFIG_BARCELONA_BAT || defined CONFIG_BARCELONA_BAT_MODULE
#if defined CONFIG_BARCELONA_LTC3455 || defined CONFIG_LTC3555_PMIC || defined CONFIG_LTC3577_PMIC
	/* NEED new config option for this? */
	if(IO_HaveGpioLowBattDetect()) {
		tomtomgo_device_bat.resource[0].start = IO_GetInterruptNumber(LOW_DC_VCC);
		platform_device_register( &tomtomgo_device_bat );
	}
#endif
#endif

	/* Setup Barcelona sound. */
	if( IO_GetCodecMaster() == GOCODECCFG_INTERNAL_MASTER ){
		/* Currently, if we're using Internal Master we want to end up with a 6Mhz CODEC clock */
		const unsigned internal_master_clock_rate = 6000000;
		struct clk* iis = clk_get(NULL, "iis");
		unsigned real_rate;

		if (iis) {
			tomtomgo_sound_info.divider = clk_get_rate(iis) / internal_master_clock_rate;
			real_rate = clk_get_rate(iis) / tomtomgo_sound_info.divider;

			if (real_rate != internal_master_clock_rate)
				printk(KERN_WARNING "iis clock divider %uHz off!\n", abs(internal_master_clock_rate - real_rate));

			clk_put(iis);
		}
	}

	s3c_device_iis.dev.platform_data = &tomtomgo_sound_info;
	platform_device_register(&s3c_device_iis );

#if defined CONFIG_USB_GADGET
	if (IO_GetUSBSlaveType() == GOUSB_S3C24XX)
		platform_device_register(&s3c_device_usbgadget);
	else if (IO_GetUSBSlaveType() == GOUSB_S3C2443)
		platform_device_register(&s3c_device_hsudc);
#endif /* CONFIG_USB_GADGET */

#if defined CONFIG_MMC_S3C || defined CONFIG_MMC_S3C_MODULE
	if (IO_HaveSdCardInterface())
		platform_device_register(&s3c_device_sdi);
#endif /* CONFIG_MMC_S3C2410 || CONFIG_MMC_S3C2410_MODULE */

#if defined CONFIG_BLK_DEV_IDE_TOMTOMGO || defined CONFIG_BLK_DEV_IDE_TOMTOMGO_MODULE
	if (IO_HaveHarddisk())
		platform_device_register(&tomtomgo_device_ide);
#endif /* CONFIG_BLK_DEV_IDE_TOMTOMGO || CONFIG_BLK_DEV_IDE_TOMTOMGO_MODULE */

#if defined CONFIG_BARCELONA_RC || defined CONFIG_BARCELONA_RC_MODULE
	if (IO_HaveRemote())
		platform_device_register(&tomtomgo_device_rc);
#endif /* CONFIG_BARCELONA_RC || CONFIG_BARCELONA_RC_MODULE */

#if defined CONFIG_BARCELONA_TS || defined CONFIG_BARCELONA_TS_MODULE
	if (IO_HaveTS())
		platform_device_register(&tomtomgo_device_ts);
#endif /* CONFIG_BARCELONA_TS || CONFIG_BARCELONA_TS_MODULE */

#if defined CONFIG_BARCELONA_TSINPUT || defined CONFIG_BARCELONA_TSINPUT_MODULE
	if (IO_HaveTS())
		platform_device_register(&tomtomgo_device_tsinput);
#endif /* CONFIG_BARCELONA_TS || CONFIG_BARCELONA_TS_MODULE */

#if defined CONFIG_BARCELONA_BUSPOWER || defined CONFIG_BARCELONA_BUSPOWER_MODULE
	platform_device_register(&tomtomgo_device_buspower);
#endif /* CONFIG_BARCELONA_BUSPOWER || CONFIG_BARCELONA_BUSPOWER_MODULE */


#if defined CONFIG_BARCELONA_USBMODE || defined CONFIG_BARCELONA_USBMODE_MODULE
	/* probe the usb mode detect */
	platform_device_register(&tomtomgo_device_usbmode);
#endif /* CCONFIG_BARCELONA_USBMODE || CONFIG_BARCELONA_USBMODE_MODULE*/
	
#if defined CONFIG_TOMTOMGO_TOUCHPAD || defined CONFIG_TOMTOMGO_TOUCHPAD_MODULE
	if (IO_HaveTP())
		platform_device_register(&tomtomgo_device_tp);
#endif /* CONFIG_TOMTOMGO_TOUCHPAD || CONFIG_TOMTOMGO_TOUCHPAD_MODULE */

#if defined CONFIG_BARCELONA_BUZ || defined CONFIG_BARCELONA_BUZ_MODULE
	if (IO_HaveBuzzer())
		platform_device_register(&tomtomgo_device_buz);
#endif /* CONFIG_BARCELONA_BUZ || CONFIG_BARCELONA_BUZ_MODULE */

	/* Quick hack for DR guys in Tapei, will be done by SRM module */
	if( IO_HasPin ( EN_DR_PWR ) ){
		IO_SetFunction( DR_GYRO_HPS );
		IO_Activate( DR_GYRO_HPS );
		msleep( 10 );
		
		IO_SetFunction( EN_DR_PWR ); /* OUTPUT	*/
		IO_Activate( EN_DR_PWR );	/* Enable the DR power plane!	*/
		
		msleep(200);
	}

#if defined (CONFIG_BARCELONA_GYRO_FUJITSU) || defined (CONFIG_BARCELONA_GYRO_FUJITSU_MODULE) || \
	defined(CONFIG_BARCELONA_GYRO_ING300) || defined(CONFIG_BARCELONA_GYRO_ING300_MODULE)
	if( IO_HaveDeadReckoning() ){
		tomtomgo_gyro_info.gyro_enable_pin = IO_Pin( GYRO_EN );
		tomtomgo_gyro_info.adc_channel[0] = ADC_GYRO_X;
		tomtomgo_gyro_info.adc_channel[1] = ADC_GYRO_Y;
		platform_device_register(&tomtomgo_device_gyro);
	}
#endif /*	defined(CONFIG_BARCELONA_GYRO) || defined(CONFIG_BARCELONA_GYRO_MODULE) */

#if defined CONFIG_VIDEO_S3C241X_CAMIF
	if (IO_HaveCamera()) {
		/* Ensure that the I2C interface is available. */
		IO_SetFunction( CAMCLKOUT );
		IO_Activate( CAMRESET );

		/* Register the platform device. */
		platform_device_register(&s3c_device_camif);
		platform_device_register(&s3c_device_i2c_camif);
	}
#endif /* CONFIG_VIDEO_S3C241X_CAMIF */

#if defined (CONFIG_BARCELONA_EXTERNAL_UART) || defined (CONFIG_BARCELONA_EXTERNAL_UART_MODULE)
	if (IO_HaveExternalUart()) {
		if (IO_HaveSingleExternalUart()) {
			/* Assign IRQ pins runtime */

			tomtomgo_external_uart_ports[0].irq = IO_GetInterruptNumber(UART_INTA);
			tomtomgo_external_uart_ports[0].mapbase  = IO_GetPAForGCSNumber( IO_GetGCSNumber(UART_CSA) );
			tomtomgo_external_uart_ports[1].flags = 0; /* serves as a terminator for serial8250 driver */
			
			/* Setup the bus to the uart	*/
			tomtomgo_external_uart_bus_settings[0].bank_nr = IO_GetGCSNumber(UART_CSA);
			s3c2410_write_bank_settings( &tomtomgo_external_uart_bus_settings[0] );
			
			platform_device_register(&tomtomgo_device_external_uart);
		} else if (IO_HaveDoubleExternalUart()) {	
			/* Assign IRQ pins runtime */
			tomtomgo_external_uart_ports[0].irq = IO_GetInterruptNumber(UART_INTA);
			tomtomgo_external_uart_ports[1].irq = IO_GetInterruptNumber(UART_INTB);
			tomtomgo_external_uart_ports[0].mapbase  = IO_GetPAForGCSNumber( IO_GetGCSNumber(UART_CSA) );
			tomtomgo_external_uart_ports[1].mapbase  = IO_GetPAForGCSNumber( IO_GetGCSNumber(UART_CSB) );
			
			/* Setup the bus to the uart	*/
			tomtomgo_external_uart_bus_settings[0].bank_nr = IO_GetGCSNumber(UART_CSA);
			s3c2410_write_bank_settings( &tomtomgo_external_uart_bus_settings[0] );
			tomtomgo_external_uart_bus_settings[1].bank_nr = IO_GetGCSNumber(UART_CSB);
			s3c2410_write_bank_settings( &tomtomgo_external_uart_bus_settings[1] );
			
			platform_device_register(&tomtomgo_device_external_uart);
		} else {
			printk(KERN_INFO "No external UART found.\n");
		}
	}
#endif /* CONFIG_BARCELONA_EXTERNAL_UART */
#if defined(CONFIG_SPI_MASTER)
	/* Only do this if the accelerometer exists. No need otherwise. */
	if( IO_HasPin( ACC_SPI_CSB ) )
	{
#if defined(CONFIG_BARCELONA_GACC_SM365) || defined(CONFIG_BARCELONA_GACC_SMB365_MODULE)
		tomtomgo_spi_board_info[ SPIDEV_SMB365 ].chip_select = IO_Pin( ACC_SPI_CSB );
		((struct gacc_platform_data*) (tomtomgo_spi_board_info[ SPIDEV_SMB365 ].platform_data))->irq_pin = IO_Pin( ACC_SPI_IRQ );
#endif /* CONFIG_BARCELONA_GACC_SMB365 || CONFIG_BARCELONA_GACC_365_MODULE	*/

#if defined CONFIG_BARCELONA_GADC_KXR94 || defined CONFIG_BARCELONA_GADC_KXR94_MODULE
		if( IO_GetAccType( ) == GOACC_KXR94 || IO_GetAccType() == GOACC_KXP74 )
			tomtomgo_spi_board_info[ SPIDEV_KXR94_GADC ].chip_select = IO_Pin( ACC_SPI_CSB );
#endif /* CONFIG BARCELONA_GADC_KXR94 || CONFIG_BARCELONA_GADC_KXR94_MODULE */
	
		IO_SetFunction( CMP_SPI_CSB );
		IO_Deactivate( CMP_SPI_CSB );	/* make sure CS of Compass is deselected	*/
	}
	else tomtomgo_spi_board_info[ 0 ].platform_data = NULL;

#if defined CONFIG_BARCELONA_BARO || defined CONFIG_BARCELONA_BARO_MODULE
	if( IO_HasBaro(  ) )
	{
		tomtomgo_spi_board_info[ SPIDEV_SCP1000 ].chip_select=IO_Pin( BARO_SPI_CSB );
		tomtomgo_spi_board_info[ SPIDEV_SCP1000 ].irq = IO_GetInterruptNumber( BARO_SPI_IRQ );
		IO_SetInterruptOnActivation( BARO_SPI_IRQ );
		IO_Deactivate( BARO_SPI_CSB );
	}
#endif /* CONFIG_BARCELONA_BARO || CONFIG_BARCELONA_BARO_MODULE */

#if defined(CONFIG_SPI_S3C24XX_GPIO) || defined(CONFIG_SPI_S3C24XX_GPIO_MODULE)

	((struct s3c2410_spigpio_info*) s3c24xx_device_spigpio.dev.platform_data)->pin_clk	= IO_Pin( SPICLK );
	((struct s3c2410_spigpio_info*) s3c24xx_device_spigpio.dev.platform_data)->pin_mosi	= IO_Pin( SPIMSI );
	((struct s3c2410_spigpio_info*) s3c24xx_device_spigpio.dev.platform_data)->pin_miso	= IO_Pin( SPIMSO );

	platform_device_register( &s3c24xx_device_spigpio );
#endif /* CONFIG_SPI_S3C24XX_GPIO || CONFIG_SPI_S3C24XX_GPIO_MODULE */
  
#if defined(CONFIG_SPI_S3C24XX) || defined(CONFIG_SPI_S3C24XX_MODULE)
	platform_device_register( &tomtomgo_device_spi1 );
#endif /* CONFIG_SPI_S3C24XX || CONFIG_SPI_S3C24XX_MODULE */
		
#endif /* CONFIG_SPI_MASTER */

#if 0 /* def CONFIG_CPU_S3C2412 */
/* Removed: PQC75 workaround in kernel */
	if (IO_GetCpuType() == GOCPU_S3C2412) {
		platform_device_register( &s3c_device_sdi );
	}
#endif /* CONFIG_CPU_S3C2412 */

#ifdef CONFIG_CPU_S3C2443
	if (IO_GetCpuType() == GOCPU_S3C2443) {
		platform_device_register( &s3c_device_hsmmc );
	}
#endif /* CONFIG_CPU_S3C2443 */

#ifdef CONFIG_CPU_S3C2450
	if (IO_GetCpuType() == GOCPU_S3C2450) {
		platform_device_register( &s3c_device_hsmmc0 );
		if ( IO_HaveSdSlot() )
			platform_device_register( &s3c_device_hsmmc1 );
	}
#endif /* CONFIG_CPU_S3C2450 */

#ifdef CONFIG_I2C
#ifdef CONFIG_BARCELONA_DOCK
	if( IO_HasPin( DOCK_INT ) )
	{
		/* Register the dock dongle platform device. This handles the DOCK_INT interrupt. */
		s3c24xx_dock_dongle.resource->start=IO_GetInterruptNumber( DOCK_INT );
		s3c24xx_dock_dongle.resource->end=s3c24xx_dock_dongle.resource->start;
		platform_device_register( &s3c24xx_dock_dongle );
	}
	if (IO_HasPin(DOCK_SENSE1))
	{
		/* Register the Cagliari dock dongle platform device. */
		s3c24xx_dock_cagliari_dongle.resource[0].start =
			IO_GetInterruptNumber(DOCK_SENSE);
		s3c24xx_dock_cagliari_dongle.resource[0].end =
			s3c24xx_dock_cagliari_dongle.resource->start;
		s3c24xx_dock_cagliari_dongle.resource[1].start =
			IO_GetInterruptNumber(DOCK_SENSE1);
		s3c24xx_dock_cagliari_dongle.resource[1].end =
			s3c24xx_dock_cagliari_dongle.resource->start;
		platform_device_register(&s3c24xx_dock_cagliari_dongle);
	}
#endif /* CONFIG_BARCELONA_DOCK */

	/* The RDS/TMC reception requires interrupt support */
#if defined( CONFIG_SI4703_FM ) || defined ( CONFIG_SI4703_FM_MODULE )
	if (IO_GetTMCReceiverType() == GOTMC_SI4703 )
	{
		((struct fm_receiver_info*) (tomtomgo_device_tmcrx_si4703.dev.platform_data))->fm_reset_pin = IO_Pin( RDS_RST ) ;
		((struct fm_receiver_info*) (tomtomgo_device_tmcrx_si4703.dev.platform_data))->fm_sen_pin = IO_Pin( RDS_SEN ) ;
		((struct fm_receiver_info*) (tomtomgo_device_tmcrx_si4703.dev.platform_data))->fm_irq_pin = IO_Pin( FM_RDS_INT ) ;
		if (IO_HasPin(DOCK_I2CEN)) {
				((struct fm_receiver_info*) (tomtomgo_device_tmcrx_si4703.dev.platform_data))->fm_dock_i2c_en_pin = IO_Pin( DOCK_I2CEN ) ;
		}

		tomtomgo_device_tmcrx_si4703.resource[0].start = IO_GetInterruptNumber(FM_RDS_INT);
		tomtomgo_device_tmcrx_si4703.resource[0].end = tomtomgo_device_tmcrx_si4703.resource->start;

		IO_SetInput(FM_RDS_INT); /* note that this call MUST precede IO_SetInterruptOnActivation(), otherwise no interrupts */
		IO_SetInterruptOnActivation( FM_RDS_INT );

		platform_device_register(&tomtomgo_device_tmcrx_si4703);
	}
#endif /* CONFIG_SI4703_FM ) */

#if defined( CONFIG_SI4705_FM )
	/* At the moment there is no irq support for SI4705 */
	/* But that will change very soon! */
	if (IO_GetTMCReceiverType() == GOTMC_SI4705 ) {
		((struct fm_receiver_info*) (tomtomgo_device_tmcrx_si4705.dev.platform_data))->fm_reset_pin = IO_Pin( RDS_RST ) ;
		((struct fm_receiver_info*) (tomtomgo_device_tmcrx_si4705.dev.platform_data))->fm_sen_pin = IO_Pin( RDS_SEN ) ;
		platform_device_register(&tomtomgo_device_tmcrx_si4705);
	}
#endif /* CONFIG_SI4705_FM ) */

#if defined( CONFIG_SI4710_FM ) 
	if( IO_HaveFMTransmitter() ){
		((struct fm_transmitter_info*) (tomtomgo_device_fmx.dev.platform_data))->fm_clock_pin = IO_Pin( EN_FM_RCLK );
		((struct fm_transmitter_info*) (tomtomgo_device_fmx.dev.platform_data))->fm_power_pin = IO_Pin( EN_FM_PWR );
		((struct fm_transmitter_info*) (tomtomgo_device_fmx.dev.platform_data))->fm_reset_pin = IO_Pin( FM_RST );
		platform_device_register( &tomtomgo_device_fmx );
	}
#endif /* CONFIG_SI4710_FM	*/
#endif /* CONFIG_I2C */

#ifdef CONFIG_GPRS_INTERFACE
	if( IO_HaveGprsModem( ) )
		platform_device_register( &tomtomgo_gprs_modem );
#endif /* CONFIG_GPRS_INTERFACE */

	return 0;
}

device_initcall(tomtomgo_setup_dynamic_devices);

#ifdef CONFIG_PM
static void tomtomgo_power_off(void)
{
	/* Sleep without simulated wake and without resume to Linux */
	s3c_pm_sleep(0,0);
}

static void __init tomtomgo_init_machine(void)
{
	s3c_pm_init();
	pm_power_off = tomtomgo_power_off;
}
#endif /* CONFIG_PM */

MACHINE_START(TOMTOMGO, "TomTom GO")
	/* Maintainer: Dimitry Andric <dimitry.andric@tomtom.com> */
	.phys_ram     = S3C2410_SDRAM_PA,
	.phys_io      = S3C2410_PA_UART,
	.io_pg_offst  = (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params  = S3C2410_SDRAM_PA + 0x0,
	.map_io       = tomtomgo_map_io,
	.init_irq     = s3c24xx_init_irq,
	.timer        = &s3c24xx_timer,
#ifdef CONFIG_PM
	.init_machine = tomtomgo_init_machine,
#endif /* CONFIG_PM */
MACHINE_END
