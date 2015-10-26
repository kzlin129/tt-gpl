/* linux/arch/arm/plat-tomtom/mendoza/mendoza-i2c.c
 *
 * Copyright 2009 TomTom B.V.
 *      Marc Zyngier <marc.zyngier@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/devs.h>
#include <plat/iic.h>
#include <plat/regs-iic.h>
#include <plat/gpio-cfg.h>

#include <mach/gpio.h>
#include <plat/mendoza.h>

static void mendoza_config_i2c(struct platform_device *dev)
{
	/* On mendoza, we need to setup the GPIO-controlled pullups
	 * on the I2C busses. Once GPIOs are properly configured,
	 * proceed to the standard configuration */

	switch(dev->id) {
		case 0:
		case -1:
			gpio_direction_output(TT_VGPIO_PU_I2C0, 1);
			s3c_i2c0_cfg_gpio(dev);
			break;

		case 1:
			gpio_direction_output(TT_VGPIO_PU_I2C1, 1);
			s3c_i2c1_cfg_gpio(dev);
			break;

		default:
			dev_err(&dev->dev, "Unknown I2C device\n");
			return;
	}
}

static struct s3c2410_platform_i2c mendoza_i2c0 __initdata = {
	.flags          = 0,
	.bus_num	= 0,
	.slave_addr     = 0x10,
	.bus_freq       = 100*1000,
	.max_freq       = 400*1000,
	.sda_delay      = S3C2410_IICLC_SDA_DELAY5 | S3C2410_IICLC_FILTER_ON,
	.cfg_gpio	= mendoza_config_i2c,
};

static struct s3c2410_platform_i2c mendoza_i2c1 __initdata = {
	.flags          = 0,
	.bus_num	= 1,
	.slave_addr     = 0x10,
	.bus_freq       = 100*1000,
	.max_freq       = 400*1000,
	.sda_delay      = S3C2410_IICLC_SDA_DELAY5 | S3C2410_IICLC_FILTER_ON,
	.cfg_gpio	= mendoza_config_i2c,
};

void __init mendoza_i2c_setup(int bitmap)
{
	if (bitmap & 1) {
		if (gpio_request(TT_VGPIO_PU_I2C0, "PU I2C0"))
			pr_err("Can't request PU I2C0\n");

		s3c_i2c0_set_platdata(&mendoza_i2c0);
		platform_device_register(&s3c_device_i2c0);
	}

	if (bitmap & 2) {
		if (gpio_request(TT_VGPIO_PU_I2C1, "PU I2C1"))
			pr_err("Can't request PU I2C1\n");

		s3c_i2c1_set_platdata(&mendoza_i2c1);
		platform_device_register(&s3c_device_i2c1);
	}
}

