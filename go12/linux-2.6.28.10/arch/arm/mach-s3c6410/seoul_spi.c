/*
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Author: Marc Zyngier <marc.zyngier@tomtom.com>
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
#include <linux/spi/spi_gpio.h>
#include <linux/interrupt.h>
#include <plat/map.h>
#include <plat/irqs.h>
#include <plat/gpio-cfg.h>
#include <plat/spi.h>
#include <plat/devs.h>
#include <mach/gpio.h>
#include <plat/lms480_pdata.h>
#include <plat/bcm4750_pdata.h>
#include <plat/mendoza.h>

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
	[0] = { /* bcm4750 CS */
		.cs_level	= CS_FLOAT,
		.cs_pin		= TT_VGPIO_BARRACUDA_CS,
		.cs_mode	= 0,
		.cs_set		= spi_cs_toggle,
		.cs_config	= spi_cs_config,
		//.cs_suspend	= sam_cs_suspend,
		//.cs_resume	= sam_cs_resume,
	},
	[1] = { /* dummy_spi CS */
		.cs_level	= CS_FLOAT,
		.cs_pin		= S3C64XX_GPK(5),
		.cs_mode	= 0,
		.cs_set		= spi_cs_toggle,
		.cs_config	= spi_cs_config,
		//.cs_suspend	= sam_cs_suspend,
		//.cs_resume	= sam_cs_resume,
	},
};

static struct spi_gpio_platform_data spi_gpio_pdata = {
	.sck		= S3C64XX_GPM(3),
	.mosi		= S3C64XX_GPM(4),
	.miso		= S3C64XX_GPK(4), // Unused...
	.num_chipselect	= 1,
};

static struct platform_device spi_gpio = {
	.name		= "spi_gpio",
	.id		= 2,
	.dev		= {
		.platform_data	= &spi_gpio_pdata,
	},
};

static struct spi_board_info spi_devices[] = {
	[0] = {
		.modalias		= "bcm4750_spi_tty",
		.irq			= IRQ_EINT(1),
		.max_speed_hz		= 1000000,
		.bus_num		= 0,
		.mode			= SPI_MODE_0,
		.chip_select		= 0,
	},
	[1] = {
		.modalias		= "lms480wv_spi",
		.max_speed_hz		= 1000000,
		.bus_num		= 2,
		.mode			= SPI_MODE_3,
		.controller_data	= (void *) TT_VGPIO_LCM_CS,
	},
	[2] = {
		.modalias		= "dummy_spi",
		.max_speed_hz		= 1000000,
		.bus_num		= 0,
		.mode			= SPI_MODE_3,
		.chip_select		= 1,
	},
};

void __init seoul_spi_setup(void)
{
	int err=0;

	spi_devices[0].platform_data	= setup_bcm4750_pdata();
	spi_devices[1].platform_data	= setup_lms480wv_pdata();

	s3c_spi_set_slaves(0, ARRAY_SIZE(spi_cs0_pdata), spi_cs0_pdata);

	if ((err = platform_device_register(&s3c_device_spi0))) {
		pr_err("Can't register s3c_device_spi0\n");
		return;
	}

	if ((err = platform_device_register(&spi_gpio))) {
		pr_err("Can't register spi_gpio\n");
		return;
	}

	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));
}

