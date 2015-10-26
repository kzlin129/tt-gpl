/*
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/>
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/vgpio.h>
#include <plat/map.h>
#include <plat/irqs.h>
#include <plat/gpio-cfg.h>
#include <plat/spi.h>
#include <plat/devs.h>
#include <mach/spi-gpio.h>
#include <mach/gpio.h>
#include <plat/mendoza.h>
#include <linux/a043fw05v4_pdata.h>

#ifdef CONFIG_SPI_S5P6440
static void spi_cs_toggle(int pin, int level)
{
	gpio_set_value(pin, level);
}

static void spi_cs_config(int pin, int foo, int bar)
{
	pr_info("spi_cs_config: configuring pin #%d\n", pin);
	if (gpio_request(pin, "SPI CS")) {
		pr_err("SPI: Can't request GPIO %d\n", pin);
		return;
	}

	gpio_direction_output(pin, 1);
}

static struct s3c_spi_pdata spi_cs0_pdata[] __initdata = {
	[0] = { 
		.cs_level	= CS_FLOAT,
		.cs_pin		= TT_VGPIO_LCD_SPI_CS,
		.cs_mode	= 0,
		.cs_set		= spi_cs_toggle,
		.cs_config	= spi_cs_config,
	},
};


static struct spi_board_info spi_devices[] = {
	[0] = {
		.modalias		= "a043fw05v4_spi",
		.max_speed_hz		= 1000000, //AUO at max 5MHz.
		.bus_num		= 0,
		.mode			= SPI_MODE_0,
		.chip_select		= 0,
	},

};

void __init catania_spi_setup( void )
{
	int err=0;

	spi_devices[0].platform_data	= setup_a043fw05v4_pdata();

	s3c_spi_set_slaves(0, ARRAY_SIZE(spi_cs0_pdata), spi_cs0_pdata);

	if ((err = platform_device_register(&s3c_device_spi0))) {
		pr_err("Can't register s3c_device_spi0\n");
		return;
	}

	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));

}
#endif

