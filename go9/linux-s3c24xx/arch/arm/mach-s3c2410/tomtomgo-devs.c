/* arch/arm/mach-s3c2410/tomtomgo-devs.h
 *
 * Definitions of TomTom GO platform devices.
 *
 * Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/device.h>
#include <asm/hardware.h>
#include <linux/serial_8250.h>
#include <linux/spi/spi.h>
#include <asm/arch/tomtomgo-map.h>
#include <asm/arch/tomtomgo-irq.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <asm/arch/spi.h>
#include <asm/arch/spi-gpio.h>
#include <barcelona/gacc.h>
#include <barcelona/gadc.h>
#include <barcelona/gyro.h>
#include <barcelona/Barc_snd.h>
#include "s3c2410-bus.h"
#include "tomtomgo-devs.h"
#include <linux/si4703.h>
#include <linux/si4705.h>
#include <linux/si4710.h>
#ifdef CONFIG_I2C_S3C24XX_GPIO
#include <barcelona/gopins.h>
#include <barcelona/s3c24xx_gpio_i2c.h>
#endif
#if defined CONFIG_LTC3555_PMIC || CONFIG_LTC3555_PMIC_MODULE
#include <linux/ltc3555-pmic.h>
#endif
#if defined CONFIG_LTC3577_PMIC || CONFIG_LTC3577_PMIC_MODULE
#include <linux/ltc3577-pmic.h>
#endif
#if defined CONFIG_BARCELONA_LTC3455 || CONFIG_BARCELONA_LTC3455_MODULE
#include <linux/ltc3455-pmic.h>
#endif

/* TomTom GO IDE device */
//#if defined CONFIG_BARCELONA_IDE || defined CONFIG_BARCELONA_IDE_MODULE
static struct resource tomtomgo_ide_resource[] = {
	[0] = {
		.start = TOMTOMGO_PA_IDE,
		.end   = TOMTOMGO_PA_IDE + TOMTOMGO_SZ_IDE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = TOMTOMGO_IRQ_IDE,
		.end   = TOMTOMGO_IRQ_IDE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device tomtomgo_device_ide = {
	.name          = "tomtomgo-ide",
	.id            = -1,
	.num_resources = ARRAY_SIZE(tomtomgo_ide_resource),
	.resource      = tomtomgo_ide_resource,
};

EXPORT_SYMBOL(tomtomgo_device_ide);
//#endif /* CONFIG_BARCELONA_IDE || CONFIG_BARCELONA_IDE_MODULE */

/* TomTom GO hardware detection device */
#if defined CONFIG_BARCELONA_HW || defined CONFIG_BARCELONA_HW_MODULE
struct platform_device tomtomgo_device_hw = {
	.name          = "tomtomgo-hw",
	.id            = -1,
};

EXPORT_SYMBOL(tomtomgo_device_hw);
#endif /* CONFIG_BARCELONA_HW || CONFIG_BARCELONA_HW_MODULE */

#if defined CONFIG_BARCELONA_GADC || defined CONFIG_BARCELONA_GADC_MODULE
static struct gadc_platform_data tomtomgo_device_s3c24xx_gadc_resource[]=
{
	{GADC_MAJOR, 1, ADC_GYRO_X},	/* X-Channel Gyro */
	{GADC_MAJOR, 2, ADC_GYRO_Y},	/* Y-Channel Gyro */
	{-1, -1, -1}
};

struct platform_device tomtomgo_device_s3c24xx_gadc =
{
	.name			= "s3c24xx-gadc",
	.id			= -1,
	.dev.platform_data	= &tomtomgo_device_s3c24xx_gadc_resource,
};

EXPORT_SYMBOL( tomtomgo_device_s3c24xx_gadc );
#endif /* CONFIG_BARCELONA_GADC || CONFIG_BARCELONA_GADC_MODULE */

#if defined CONFIG_BARCELONA_LTC3455 || defined CONFIG_BARCELONA_LTC3455_MODULE
static struct ltc3455_volt	s3c24xx_ltc3455_cpu_power[]=
{
	{133000, 1100}, {400000, 1300}, {0, 0}
};

struct ltc3455_pdata		s3c24xx_ltc3455_pdata=
{
	.power_setting		= s3c24xx_ltc3455_cpu_power,
	.voltage=
	{
		.high		= 1300,
		.low		= 1100,
	},
	.cmode=
	{
		.suspend	= LTC3455_PWM,
		.batt		= LTC3455_PWM,
		.acadapt	= LTC3455_PWM,
	},
};

struct platform_device		tomtomgo_device_s3c24xx_ltc3455 =
{
	.name			= LTC3455_DEVNAME,
	.id			= -1,
	.dev.platform_data	= &s3c24xx_ltc3455_pdata,
};
EXPORT_SYMBOL( tomtomgo_device_s3c24xx_ltc3455 );

#endif /* CONFIG_BARCELONA_LTC3455 || defined CONFIG_BARCELONA_LTC3455_MODULE */

#if defined CONFIG_LTC3555_PMIC || defined CONFIG_LTC3555_PMIC_MODULE
/* The following struct is used for CPU power on S3C24XX platforms with this PMIC. */
static struct ltc3555_volt	s3c24xx_ltc3555_sw2_power[]=
{
	{66000, 1000}, {133000, 1100}, {400000, 1300}, {0, 0}
};

/* The following struct is used for VDD3V3 on S3C24XX platforms with this PMIC. */
static struct ltc3555_volt	s3c24xx_ltc3555_sw3_power[]=
{
	{400000, 3230}, {0, 0}
};

static struct ltc3555_platform	s3c24xx_ltc3555_pdata=
{
	.sw2 = {62000, 100000, s3c24xx_ltc3555_sw2_power},
	.sw3 = {316000, 100000, s3c24xx_ltc3555_sw3_power},
	.swmode = {LTC3555_BURST, LTC3555_PULSE_SKIP, LTC3555_PULSE_SKIP}
};

struct platform_device		tomtomgo_device_s3c24xx_ltc3555_pmic =
{
	.name			= LTC3555_DEVNAME,
	.id			= -1,
	.dev.platform_data	= &s3c24xx_ltc3555_pdata,
};
EXPORT_SYMBOL( tomtomgo_device_s3c24xx_ltc3555_pmic );
#endif /* CONFIG_LTC3555_PMIC || CONFIG_LTC3555_PMIC_MODULE */

#if defined CONFIG_LTC3577_PMIC || defined CONFIG_LTC3577_PMIC_MODULE
static struct ltc3577_platform	s3c24xx_ltc3577_pdata=
{
	.swmode = {LTC3577_BURST, LTC3577_PWM}
};

struct platform_device		tomtomgo_device_s3c24xx_ltc3577_pmic =
{
	.name			= LTC3577_DEVNAME,
	.id			= -1,
	.dev.platform_data	= &s3c24xx_ltc3577_pdata,
};
EXPORT_SYMBOL( tomtomgo_device_s3c24xx_ltc3577_pmic );
#endif /* CONFIG_LTC3577_PMIC || CONFIG_LTC3577_PMIC_MODULE */

/* TomTom GO ADC device */
#if defined CONFIG_BARCELONA_ADC || defined CONFIG_BARCELONA_ADC_MODULE
static struct resource tomtomgo_adc_resource[] = {
	[0] = {
		.start = S3C2410_PA_ADC,
		.end   = S3C2410_PA_ADC + S3C24XX_SZ_ADC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TC,
		.end   = IRQ_ADC,
		.flags = IORESOURCE_IRQ,
	},
};

/* struct adc_channels is declared in include/barcelona/Barc_adc.h */
struct adc_channels tomtomgo_adc_channels [] =
{
	{
		.name   = "light_sensor",
		.number = ADC_LX_OUT
	},

    /* Additional ADC channels attributes may be defined here ...*/

	{
		/* This last item MUST remain at the last position! */
		.name   = NULL,
		.number = ADC_CHANNELS
	}
} ;

struct platform_device tomtomgo_device_adc = {
	.name          = "tomtomgo-adc",
	.id            = -1,
	.num_resources = ARRAY_SIZE(tomtomgo_adc_resource),
	.resource      = tomtomgo_adc_resource,
	.dev.platform_data = &tomtomgo_adc_channels
};

EXPORT_SYMBOL(tomtomgo_device_adc);
#endif /* CONFIG_BARCELONA_ADC || CONFIG_BARCELONA_ADC_MODULE */

/* TomTom GO accelerometer device */
#if defined CONFIG_BARCELONA_ACC || defined CONFIG_BARCELONA_ACC_MODULE
struct platform_device tomtomgo_device_acc = {
	.name          = "tomtomgo-acc",
	.id            = -1,
};

EXPORT_SYMBOL(tomtomgo_device_acc);
#endif /* CONFIG_BARCELONA_ACC || CONFIG_BARCELONA_ACC_MODULE */

/* TomTom GO battery device */
#if defined CONFIG_BARCELONA_BAT || defined CONFIG_BARCELONA_BAT_MODULE

static struct resource tomtomgo_bat_resource[] = {
	[0] = {
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device tomtomgo_device_bat = {
	.name          = "tomtomgo-bat",
	.id            = -1,
	.num_resources = ARRAY_SIZE(tomtomgo_bat_resource),
	.resource	   = tomtomgo_bat_resource,
};

EXPORT_SYMBOL(tomtomgo_device_bat);
#endif /* CONFIG_BARCELONA_BAT || CONFIG_BARCELONA_BAT_MODULE */

/* buspower driver */
#if defined CONFIG_BARCELONA_BUSPOWER || defined CONFIG_BARCELONA_BUSPOWER_MODULE
struct platform_device tomtomgo_device_buspower = {
	.name          = "tomtomgo-buspower",
	.id            = -1,
};
EXPORT_SYMBOL(tomtomgo_device_buspower);
#endif /* CONFIG_BARCELONA_BUSPOWER || CONFIG_BARCELONA_BUSPOWER_MODULE */

/* TomTom GO GPIO device */
#if defined CONFIG_BARCELONA_GPIO || defined CONFIG_BARCELONA_GPIO_MODULE
struct platform_device tomtomgo_device_gpio = {
	.name          = "tomtomgo-gpio",
	.id            = -1,
};

EXPORT_SYMBOL(tomtomgo_device_gpio);
#endif /* CONFIG_BARCELONA_GPIO || CONFIG_BARCELONA_GPIO_MODULE */

/* TomTom GO GPS device */
#if defined CONFIG_BARCELONA_GPS || defined CONFIG_BARCELONA_GPS_MODULE
static struct resource tomtomgo_gps_resource[] = {
	[0] = {
		.start = TOMTOMGO_IRQ_GPS_PPS,
		.end   = TOMTOMGO_IRQ_GPS_PPS,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device tomtomgo_device_gps = {
	.name          = "tomtomgo-gps",
	.id            = -1,
	.num_resources = ARRAY_SIZE(tomtomgo_gps_resource),
	.resource      = tomtomgo_gps_resource,
};

EXPORT_SYMBOL(tomtomgo_device_gps);
#endif /* CONFIG_BARCELONA_GPS || CONFIG_BARCELONA_GPS_MODULE */

/* TomTom GO PWM device */
#if defined CONFIG_BARCELONA_PWM || defined CONFIG_BARCELONA_PWM_MODULE
struct platform_device tomtomgo_device_pwm = {
	.name          = "tomtomgo-pwm",
	.id            = -1,
};

EXPORT_SYMBOL(tomtomgo_device_pwm);
#endif /* CONFIG_BARCELONA_PWM || CONFIG_BARCELONA_PWM_MODULE */

/* TomTom GO S3C Backlight device */
#if defined CONFIG_BACKLIGHT_S3C || defined CONFIG_BACKLIGHT_S3C_MODULE
struct platform_device tomtomgo_device_s3c = {
	.name          = "backlight_s3c",
	.id            = -1,
};

EXPORT_SYMBOL(tomtomgo_device_s3c);
#endif /* CONFIG_BACKLIGHT_S3C || CONFIG_BACKLIGHT_S3C_MODULE */

#if defined CONFIG_BACKLIGHT_S3C_LTC || defined CONFIG_BACKLIGHT_S3C_LTC_MODULE
struct platform_device tomtomgo_device_s3c_ltc = {
	.name          = "backlight_s3c_ltc",
	.id            = -1,
};

EXPORT_SYMBOL(tomtomgo_device_s3c_ltc);
#endif /* CONFIG_BACKLIGHT_S3C_LTC || CONFIG_BACKLIGHT_S3C_LTC_MODULE */

/* TomTom GO remote control device */
#if defined CONFIG_BARCELONA_RC || defined CONFIG_BARCELONA_RC_MODULE
struct platform_device tomtomgo_device_rc = {
	.name          = "tomtomgo-rc",
	.id            = -1,
};

EXPORT_SYMBOL(tomtomgo_device_rc);
#endif /* CONFIG_BARCELONA_RC || CONFIG_BARCELONA_RC_MODULE */

/* TomTom GO touchscreen */
#if defined CONFIG_BARCELONA_TS || defined CONFIG_BARCELONA_TS_MODULE
struct platform_device tomtomgo_device_ts = {
	.name          = "tomtomgo-ts",
	.id            = -1,
};

EXPORT_SYMBOL(tomtomgo_device_ts);
#endif /* CONFIG_BARCELONA_TS || CONFIG_BARCELONA_TS_MODULE */


/* TomTom GO touchscreen input system*/
#if defined CONFIG_BARCELONA_TSINPUT || defined CONFIG_BARCELONA_TSINPUT_MODULE
struct platform_device tomtomgo_device_tsinput = {
	.name          = "tomtomgo-tsinput",
	.id            = -1,
};

EXPORT_SYMBOL(tomtomgo_device_tsinput);
#endif /* CONFIG_BARCELONA_TSINPUT || CONFIG_BARCELONA_TSINPUT_MODULE */

/* TomTom GO legacy usb mode detection driver */
#if defined CONFIG_BARCELONA_USBMODE || defined CONFIG_BARCELONA_USBMODE_MODULE
struct platform_device tomtomgo_device_usbmode = {
	.name          = "tomtomgo-usbmode",
	.id            = -1,
};
EXPORT_SYMBOL(tomtomgo_device_usbmode);
#endif /* CONFIG_BARCELONA_USBMODE || CONFIG_BARCELONA_USBMODE_MODULE */

/* TomTom GO Touchpad device */
#if defined CONFIG_TOMTOMGO_TOUCHPAD
static struct resource tomtomgo_tp_resource[] = {
	[0] = {
		.start = IRQ_EINT20,
		.end   = IRQ_EINT23,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device tomtomgo_device_tp = {
	.name          = "synaptics-mep-bus",
	.id            = -1,
	.num_resources = ARRAY_SIZE(tomtomgo_tp_resource),
	.resource      = tomtomgo_tp_resource
};
EXPORT_SYMBOL(tomtomgo_device_tp);
#endif /* CONFIG_TOMTOMGO_TOUCHPAD */

#if defined (CONFIG_BARCELONA_GYRO_FUJITSU) || defined (CONFIG_BARCELONA_GYRO_FUJITSU_MODULE) || \
	defined(CONFIG_BARCELONA_GYRO_ING300) || defined(CONFIG_BARCELONA_GYRO_ING300_MODULE)

struct gyro_platform_data tomtomgo_gyro_info = {
  	.device_nr	= MKDEV( GYRO_MAJOR, 0),
};
EXPORT_SYMBOL(tomtomgo_gyro_info);

struct platform_device tomtomgo_device_gyro = {
	.name				= "tomtomgo-gyro",
	.id					= -1, 	
	.dev.platform_data 	= &tomtomgo_gyro_info
 };
EXPORT_SYMBOL(tomtomgo_device_gyro);
#endif /*	defined(CONFIG_BARCELONA_GYRO) || defined(CONFIG_BARCELONA_GYRO_MODULE) */

/* TomTom GO buzzer device */
#if defined CONFIG_BARCELONA_BUZ || defined CONFIG_BARCELONA_BUZ_MODULE
struct platform_device tomtomgo_device_buz = {
	.name          = "tomtomgo-buz",
	.id            = -1,
};
EXPORT_SYMBOL(tomtomgo_device_buz);
#endif /* CONFIG_BARCELONA_BUZ || CONFIG_BARCELONA_BUZ_MODULE */

#if defined CONFIG_BARCELONA_EXTERNAL_UART || defined CONFIG_BARCELONA_EXTERNAL_UART_MODULE

/**
 * Some fields in these structure are determined at runtime!
 * Provide two channels, tomtomgo_setup_dynamic_devices() will
 * terminate the list if there is only one external uart.  
 */
 
struct s3c2410_bank_settings tomtomgo_external_uart_bus_settings[] = { 
	[0] = {
		.bank_nr      = -1, 	/* Is filled in dynamically later	*/
		.bus_width    = BUS_WIDTH_16,
		.wait_status  = WAIT_DISABLE,
		.timing.t_acs = T_ACS_CLKS_4,
		.timing.t_cos = T_COS_CLKS_4,
		.timing.t_acc = T_ACC_CLKS_14,
		.timing.t_coh = T_COH_CLKS_4,
		.timing.t_cah = T_CAH_CLKS_4,
		.timing.t_acp = T_ACP_CLKS_2,
		.timing.pmc   = PMC_NORMAL,
	},
	[1] = {
		.bank_nr      = -1, 	/* Is filled in dynamically later	*/
		.bus_width    = BUS_WIDTH_16,
		.wait_status  = WAIT_DISABLE,
		.timing.t_acs = T_ACS_CLKS_4,
		.timing.t_cos = T_COS_CLKS_4,
		.timing.t_acc = T_ACC_CLKS_14,
		.timing.t_coh = T_COH_CLKS_4,
		.timing.t_cah = T_CAH_CLKS_4,
		.timing.t_acp = T_ACP_CLKS_2,
		.timing.pmc   = PMC_NORMAL,
	}
};
EXPORT_SYMBOL( tomtomgo_external_uart_bus_settings );

struct plat_serial8250_port tomtomgo_external_uart_ports[] = {
	{
		.uartclk   = 0xffffffff, /* Indicate to 8250.c that clock rate must be read from tomtom-uart.c */
		.regshift  = 1,
		.iotype    = UPIO_MEM,
		.flags     = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_IOREMAP,
	},
	{
		.uartclk   = 0xffffffff, /* Indicate to 8250.c that clock rate must be read from tomtom-uart.c */
		.regshift  = 1,
		.iotype    = UPIO_MEM,
		.flags     = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_IOREMAP,
	},
	{
		.flags     = 0, /* serves as a terminator for serial8250 driver */
	}
};
EXPORT_SYMBOL(tomtomgo_external_uart_ports);

struct platform_device tomtomgo_device_external_uart = {
  	.name              = "serial8250",
  	.id                = 0,
  	.dev.platform_data = &tomtomgo_external_uart_ports[0],
};

EXPORT_SYMBOL(tomtomgo_device_external_uart);

#endif /* CONFIG_BARCELONA_EXTERNAL_UART */


#if defined(CONFIG_SPI_MASTER)

#if defined(CONFIG_BARCELONA_GACC_SMB365) || defined(CONFIG_BARCELONA_GACC_SMB365_MODULE)
static struct gacc_platform_data tomtomgo_smb365_info = {
	.device_nr	= MKDEV( 168, 0),
	.adc		= NULL,
};
#endif /* CONFIG_BARCELONA_GACC_SMB365_GACC	*/

#if defined(CONFIG_BARCELONA_GACC_KXP74) || defined(CONFIG_BARCELONA_GACC_KXP74_MODULE)
struct gadc_platform_data tomtomgo_kxr94_adc_info = {
	.minor		= 0,
	.major		= GADC_MAJOR,
	.channel	= 0,
};
EXPORT_SYMBOL( tomtomgo_kxr94_adc_info );

static struct gacc_platform_data tomtomgo_kxp74_info = {
	.device_nr	= MKDEV( 168, 0),
	.adc		= NULL,
};
#endif /* CONFIG_BARCELONA_GACC_KXP74_GACC	*/

#if defined CONFIG_BARCELONA_GADC_KXR94 || defined CONFIG_BARCELONA_GADC_KXR94_MODULE
static struct gadc_platform_data tomtomgo_kxr94_info = {
	.major		= 168,
	.minor		= 0,
	.channel	= 0,
};
#endif /* CONFIG_BARCELONA_GADC_KXR94 || CONFIG_BARCELONA_GADC_KXR94_MODULE */

#if defined CONFIG_BARCELONA_BARO || defined CONFIG_BARCELONA_BARO_MODULE
static struct gadc_platform_data tomtomgo_device_baro_private[]=
{
	{GADC_MAJOR, 32, 0},
	{GADC_MAJOR, 64, 0}
};
#endif /* CONFIG_BARCELONA_BARO || CONFIG_BARCELONA_BARO_MODULE */

struct spi_board_info tomtomgo_spi_board_info[] = {
#if defined(CONFIG_BARCELONA_GACC_SMB365) || defined(CONFIG_BARCELONA_GACC_SMB365_MODULE)
	[SPIDEV_SMB365] = {	
		/* Bosch SMB365 Accelerometer	*/
		.modalias	= "smb365",
		.platform_data	= &tomtomgo_smb365_info,
		.mode		= SPI_MODE_3 | SPI_CS_HIGH,	/* Always active high, the gopin_t IO libs will handle invertedness	*/
		//.max_speed_hz	= 6000000, /* SMB365 data sheets says 8 MHz is abs. max	*/
		.max_speed_hz	= 1000000, /* DEBUG  */
		.bus_num	= 0,
		/* Note: chip_select is setup runtime	*/
	},
#endif /* CONFIG_BARCELONA_GACC_SMB365 || CONFIG_BARCELONA_GACC_SMB365_MODULE */
#if defined(CONFIG_BARCELONA_GACC_KXP74) || defined(CONFIG_BARCELONA_GACC_KXP74_MODULE)
	[SPIDEV_KXP74] = {
                /* Kionix KXP74/KXR94 Accelerometer   */
                .modalias       = "kxp74",			/* This is the default. If this is the KXR94, then we will */
								/* change this name. */
                .platform_data  = &tomtomgo_kxp74_info,
                .mode           = SPI_MODE_3 | SPI_CS_HIGH,     /* Always active high, the gopin_t IO libs will handle invertedness     */
                //.max_speed_hz = 6000000, /* SMB365 data sheets says 8 MHz is abs. max */
                .max_speed_hz   = 1000000, /* DEBUG  */
                .bus_num        = 0,
                /* Note: chip_select is setup runtime   */
        },
#endif /* defined(CONFIG_BARCELONA_GACC_KXP74) || CONFIG_BARCELONA_GACC_KXP74_MODULE	*/
#if defined CONFIG_BARCELONA_BARO || defined CONFIG_BARCELONA_BARO_MODULE
	[SPIDEV_SCP1000] = {
		/* VTI SCP1000 Barometer */
		.modalias	= "scp1000-baro",
		.platform_data	= &tomtomgo_device_baro_private,
                .mode           = SPI_MODE_3 | SPI_CS_HIGH,     /* Always active high, the gopin_t IO libs will handle invertedness     */
                .max_speed_hz   = 1000000, /* DEBUG  */
                .bus_num        = 0,
	},
#endif /* CONFIG_BARCELONA_BARO || CONFIG_BARCELONA_BARO_MODULE */
#if defined CONFIG_BARCELONA_GADC_KXR94 || defined CONFIG_BARCELONA_GADC_KXR94_MODULE
	[SPIDEV_KXR94_GADC] = {
		/* Kionix KXR94 Accelerometer - GADC driver interface */
		.modalias	= "kxr94-gadc",
		.platform_data	= &tomtomgo_kxr94_info,
                .mode           = SPI_MODE_3 | SPI_CS_HIGH,     /* Always active high, the gopin_t IO libs will handle invertedness     */
                .max_speed_hz   = 1000000, /* DEBUG  */
                .bus_num        = 0,
	},
#endif /* CONFIG_BARCELONA_GADC_KXR94 || CONFIG_BARCELONA_GADC_KXR94_MODULE */
};
EXPORT_SYMBOL(tomtomgo_spi_board_info);


#if defined(CONFIG_SPI_S3C24XX_GPIO) || defined(CONFIG_SPI_S3C24XX_GPIO_MODULE)
/* Additional info for spi_s3c24xxbitbang.c SPI master */

extern void tomtom_spigpio_set_cs(struct spi_device *spi, int value, int pol); /* in drivers/spi/spi_s3c24xx_gpio.c	*/

struct s3c2410_spigpio_info tomtomgo_spigpio_info = {
	/* 
	 * N.B: pin_cs is ignored because the chip select is device specific. We define tomtom_spigpio_set_cs()
	 * that will use the chip_select field from the spi device.
	 */
	.board_size	= ARRAY_SIZE(tomtomgo_spi_board_info),
	.board_info	= &tomtomgo_spi_board_info[0],
	.chip_select	= tomtom_spigpio_set_cs,
};	

EXPORT_SYMBOL(tomtomgo_spigpio_info);

struct platform_device s3c24xx_device_spigpio = {
        .name              = "s3c24xx-spi-gpio",
        .id                = -1,
        .dev.platform_data = &tomtomgo_spigpio_info,
};
EXPORT_SYMBOL(s3c24xx_device_spigpio);

#endif /* SPI_S3C24XX_GPIO || SPI_S3C24XX_GPIO_MODULE	*/

#if defined(CONFIG_SPI_S3C24XX) || defined(CONFIG_SPI_S3C24XX_MODULE)
/* Additional info for spi_s3c24xx.c SPI master */


extern void tomtom_spi_set_cs(struct spi_device *spi, int value, int pol); /* in drivers/spi/spi_s3c24xx_.c	*/

struct s3c2410_spi_info tomtomgo_spi_info = {
	/* 
	 * N.B: pin_cs is ignored because the chip select is device specific. We define tomtom_spi_set_cs()
	 * that will use the chip_select field from the spi device.
	 */
	.board_size	  = ARRAY_SIZE(tomtomgo_spi_board_info),
	.board_info 	= &tomtomgo_spi_board_info[0],
	.chip_select	= tomtom_spi_set_cs,
};	

EXPORT_SYMBOL(tomtomgo_spi_info);

/* TomTomGo SPI1 - Slow SPI */

static struct resource tomtomgo_spi1_resource[] = {
  [0] = {
    .start = S3C2410_PA_SPI,
    .end   = S3C2410_PA_SPI + 0x28,
    .flags = IORESOURCE_MEM
  },
  [1] = {
    .start = IRQ_SPI1,
    .end   = IRQ_SPI1,
    .flags = IORESOURCE_IRQ
  }
};

struct platform_device tomtomgo_device_spi1 = {
  .name               = "s3c2410-spi",
  .id                 = 1,
  .num_resources      = ARRAY_SIZE(tomtomgo_spi1_resource),
  .resource           = tomtomgo_spi1_resource,
  .dev.platform_data  = &tomtomgo_spi_info
};

EXPORT_SYMBOL(tomtomgo_device_spi1);

#endif /* SPI_S3C24XX || SPI_S3C24XX_MODULE	*/

#endif /* CONFIG_SPI_MASTER	*/

struct barcelona_sound_info tomtomgo_sound_info = {
	.divider = 0,
};

EXPORT_SYMBOL( tomtomgo_sound_info );

#if defined( CONFIG_I2C )

#if defined( CONFIG_SI4703_FM ) ||  defined ( CONFIG_SI4703_FM_MODULE  )
static struct resource fm_receiver_resource_si4703[] = {
	[0] = {
		.start = -1,
		.end   = -1,
		.flags = IORESOURCE_IRQ,
	}
};

static struct fm_receiver_info tomtomgo_tmcrx_si4703_info = {
        .slave_address = SI4703_I2C_SLAVE_ADDR,
        .device_nr = MKDEV( SI4703_MAJOR, SI4703_MINOR),
};

struct platform_device tomtomgo_device_tmcrx_si4703 = {
        .name = SI4703_DEVNAME,
        .id = -1,
        .dev.platform_data = &tomtomgo_tmcrx_si4703_info,
	.num_resources = ARRAY_SIZE(fm_receiver_resource_si4703),
	.resource = fm_receiver_resource_si4703,
};

EXPORT_SYMBOL(tomtomgo_device_tmcrx_si4703);

#endif /* CONFIG_SI4703_FM */

#if defined( CONFIG_SI4705_FM )

static struct resource fm_receiver_resource_si4705[] = {
	[0] = {
		.start = -1,
		.end   = -1,
		.flags = IORESOURCE_IRQ,
	}
};

static struct fm_receiver_info tomtomgo_tmcrx_si4705_info = {
        .slave_address = SI4705_I2C_SLAVE_ADDR,
        .device_nr = MKDEV( SI4705_MAJOR, SI4705_MINOR),
};

struct platform_device tomtomgo_device_tmcrx_si4705 = {
        .name = SI4705_DEVNAME,
        .id = -1,
        .dev.platform_data = &tomtomgo_tmcrx_si4705_info,
	.num_resources = ARRAY_SIZE(fm_receiver_resource_si4705),
	.resource = fm_receiver_resource_si4705,
};

EXPORT_SYMBOL(tomtomgo_device_tmcrx_si4705);

#endif /* CONFIG_SI4705_FM */

#if defined( CONFIG_SI4710_FM ) 

static struct fm_transmitter_info tomtomgo_fmx_info = {
	.slave_address	= SI4710_I2C_SLAVE_ADDR,
	.device_nr		= MKDEV( SI4710_MAJOR, SI4710_MINOR),
};

struct platform_device tomtomgo_device_fmx = {
	.name				= SI4710_DEVNAME,
	.id					= -1,
	.dev.platform_data	= &tomtomgo_fmx_info
};

EXPORT_SYMBOL(tomtomgo_device_fmx);
#endif /* defined( CONFIG_SI4710_FM ) */

#ifdef CONFIG_I2C_S3C24XX_GPIO
static struct s3c24xx_gpio_i2c_pins s3c24xx_i2c_gpio_pins = {
	.sda_pin	= 0,
	.scl_pin	= 0,
};

struct platform_device s3c24xx_gpio_i2c_controller = {
	.name		= "S3C24XX-GPIO-I2C",
	.id		= 0,
	.dev		= {
		.platform_data = &s3c24xx_i2c_gpio_pins,
	},
        .num_resources  = 0
};
EXPORT_SYMBOL( s3c24xx_gpio_i2c_controller );
#endif /* ifdef I2C_S3C24XX_GPIO */

#ifdef CONFIG_GPRS_INTERFACE
struct platform_device tomtomgo_gprs_modem = {
	.name		= "gprs-interface",
	.id		= -1,
	.num_resources	= 0
};

EXPORT_SYMBOL( tomtomgo_gprs_modem );
#endif /* ifdef CONFIG_GPRS_INTERFACE */

#ifdef CONFIG_BARCELONA_DOCK
static struct resource s3c24xx_dock_dongle_resource[]= {
	[0] = {
		.start	=0,
		.end	=0,
		.flags	=IORESOURCE_IRQ,
	}
};

struct platform_device s3c24xx_dock_dongle = {
	.name		= "dock-dongle",
	.id		= 0,
	.num_resources	= ARRAY_SIZE( s3c24xx_dock_dongle_resource ),
	.resource	= s3c24xx_dock_dongle_resource,
};
EXPORT_SYMBOL( s3c24xx_dock_dongle );

/* Cagliari Dock */
static struct resource s3c24xx_dock_cagliari_dongle_resource[]= {
	[0] = {
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device s3c24xx_dock_cagliari_dongle = {
	.name		= "dock-cagliari-dongle",
	.id		= 0,
	.num_resources	= ARRAY_SIZE( s3c24xx_dock_cagliari_dongle_resource ),
	.resource	= s3c24xx_dock_cagliari_dongle_resource,
};
EXPORT_SYMBOL( s3c24xx_dock_cagliari_dongle );
#endif /* CONFIG_BARCELONA_DOCK */

#endif /* defined( CONFIG_I2C )	*/ 

/* EOF */
