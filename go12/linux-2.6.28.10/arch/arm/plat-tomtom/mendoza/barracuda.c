/*
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * 	Benoit Leffray <benoit.leffray@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <plat/gps.h>
#include <plat/tt_setup_handler.h>

#include <mach/gpio.h>
#include <plat/irqs.h>
#include <plat/mendoza.h>

#define BARRACUDA_DEVNAME "barracuda"
#define PFX	BARRACUDA_DEVNAME ": "

static void barracuda_set_power(int power)
{
	msleep( 200 );
	/* PWR == !STANDBY */
	gpio_set_value(TT_VGPIO_GPS_STANDBY, !power);
	msleep( 200 );

	if (power)
		pr_info("barracuda gps switched ON\n");
	else
		pr_info("barracuda gps switched OFF\n");
}

void barracuda_reset(void)
{
	gpio_set_value(TT_VGPIO_GPS_RESET, 1);
	mdelay( 50 );
	gpio_set_value(TT_VGPIO_GPS_RESET, 0);
	mdelay( 100 );

	printk("barracuda gps reset\n");
}	

static struct generic_gps_info machinfo =
{
	.name			= BARRACUDA_DEVNAME,
	.gps_set_power		= barracuda_set_power,
	.gps_reset		= barracuda_reset
};

static void barracuda_dev_release(struct device *dev){}

static struct resource barracuda_resource = {
	.name  = "barracuda_1PPS",
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	.start = IRQ_EINT(10), //Beware, may not always be common across SoCs...
	.end   = IRQ_EINT(10),
};

static struct platform_device barracuda_pdev =
{
	.name			= "tomtom-gps",
	.id			= -1,
	.num_resources		= 1,
	.resource		= &barracuda_resource,
	.dev 			= {
		.platform_data	= &machinfo,
		.release	= barracuda_dev_release,
	},
};

static __init int barracuda_init(char * identifier)
{
	int err;

	if ((err = gpio_request(TT_VGPIO_GPS_STANDBY, "STANDBY"))) {
		pr_err("Can't request STANDBY GPIO\n");
		return err;
	}

	if ((err = gpio_request(TT_VGPIO_GPS_RESET, "RESET"))) {
		pr_err("Can't request RESET GPIO\n");
		gpio_free(TT_VGPIO_GPS_RESET);
		return err;
	}

	gpio_direction_output(TT_VGPIO_GPS_RESET, 0);
	gpio_direction_output(TT_VGPIO_GPS_STANDBY, 0);

	barracuda_reset();

	if ((err = platform_device_register(&barracuda_pdev))) {
		pr_err("Can't register barracuda_pdev (%d)\n", err);
		return err;
	}
	
	return 0;
};

TT_SETUP_CB(barracuda_init , "tomtom-gps-barracuda");
