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
#include <plat/map.h>
#include <plat/irqs.h>
#include <plat/gpio-cfg.h>
#include <plat/spi.h>
#include <plat/devs.h>
#include <mach/spi-gpio.h>
#include <mach/gpio.h>
#include <plat/lms480_pdata.h>
#include <plat/bcm4750_pdata.h>

#include <mach/cordoba.h>

#ifndef CONFIG_SPI_S5P6440
// This is only here to check for regressions with the HW SPI controller.
static struct spi_board_info spi_devices[] = {
	[0] = {
		.modalias		= "bcm4750_spi_tty",
		.controller_data	= NULL,
		.irq			= IRQ_EINT(4),
		.max_speed_hz		= 2500000,
		.bus_num		= 0,
		.mode			= SPI_MODE_0,
		.chip_select		= TT_VGPIO_BARRACUDA_CS,
	},
	[1] = {
		.modalias		= "lms480wv_spi",
		.controller_data	= NULL,
		.max_speed_hz		= 1000000,
		.bus_num		= 0,
		.chip_select		= TT_VGPIO_LCM_CS,
		.mode			= SPI_MODE_2,
	},
};

static struct s3c2410_spigpio_info spi_gpio = {
	.pin_clk	= LCM_SPI_CLK(LCM_SPI_CH),
	.pin_mosi	= LCM_SPI_MOSI(LCM_SPI_CH),
	.pin_miso	= S5P64XX_GPC(0),
	.bus_num	= 0,
	.board_info	= spi_devices,
	.board_size	= ARRAY_SIZE(spi_devices),
	.chip_select	= NULL,
};

static struct platform_device spi_gpio_data = {
	.name = "spi_s3c64xx_gpio",
	.id = -1,
	.dev = {
		.platform_data = &spi_gpio,
	},
}; 

void __init cordoba_spi_setup( void )
{
	int err=0;

	printk(KERN_INFO "SPI_DEVICES: registering spi devices to the SPI controller\n");

	spi_devices[0].platform_data  = setup_bcm4750_pdata();
	spi_devices[1].platform_data = setup_lms480wv_pdata();

	spi_gpio.board_info	= spi_devices;

	
	if ((err = platform_device_register(&spi_gpio_data))) {
		pr_err("Can't register spi_gpio_data\n");
	}
}
#else

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
	[1] = { /* lms480wv CS */
		.cs_level	= CS_FLOAT,
		.cs_pin		= TT_VGPIO_LCM_CS,
		.cs_mode	= 0,
		.cs_set		= spi_cs_toggle,
		.cs_config	= spi_cs_config,
		//.cs_suspend	= sam_cs_suspend,
		//.cs_resume	= sam_cs_resume,
	},
	[2] = { // Stupid SPI test module
		.cs_level	= CS_FLOAT,
		.cs_pin		= S5P64XX_GPN(15),
		.cs_mode	= 0,
		.cs_set		= spi_cs_toggle,
		.cs_config	= spi_cs_config,
		//.cs_suspend	= sam_cs_suspend,
		//.cs_resume	= sam_cs_resume,
	},
};

static struct s3c_spi_pdata spi_cs1_pdata[] __initdata = {
	[0] = { // Stupid SPI test module, #2
		.cs_level	= CS_FLOAT,
		.cs_pin		= S5P64XX_GPN(13),
		.cs_mode	= 0,
		.cs_set		= spi_cs_toggle,
		.cs_config	= spi_cs_config,
		//.cs_suspend	= sam_cs_suspend,
		//.cs_resume	= sam_cs_resume,
	},
};

static struct spi_board_info spi_devices[] = {
	[0] = {
		.modalias		= "bcm4750_spi_tty",
		.irq			= IRQ_EINT(4),
		.max_speed_hz		= 1000000,
		.bus_num		= 0,
		.mode			= SPI_MODE_0,
		.chip_select		= 0,
	},
	[1] = {
		.modalias		= "lms480wv_spi",
		.max_speed_hz		= 1000000,
		.bus_num		= 0,
		.mode			= SPI_MODE_3,
		.chip_select		= 1,
	},
	[2] = {
		.modalias		= "dummy_spi",
		.max_speed_hz		= 1000000,
		.bus_num		= 0,
		.mode			= SPI_MODE_0,
		.chip_select		= 2,
	},
	[3] = {
		.modalias		= "dummy_spi",
		.max_speed_hz		= 1000000,
		.bus_num		= 1,
		.mode			= SPI_MODE_0,
		.chip_select		= 0,
	},
};

void __init cordoba_spi_setup( void )
{
	int err=0;

	spi_devices[0].platform_data	= setup_bcm4750_pdata();
	spi_devices[1].platform_data	= setup_lms480wv_pdata();

	s3c_spi_set_slaves(0, ARRAY_SIZE(spi_cs0_pdata), spi_cs0_pdata);
	s3c_spi_set_slaves(1, ARRAY_SIZE(spi_cs1_pdata), spi_cs1_pdata);

	if ((err = platform_device_register(&s3c_device_spi0))) {
		pr_err("Can't register s3c_device_spi0\n");
		return;
	}

	if ((err = platform_device_register(&s3c_device_spi1))) {
		pr_err("Can't register s3c_device_spi1\n");
		return;
	}

	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));
}
#endif

