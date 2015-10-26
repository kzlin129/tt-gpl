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
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>

static void atheros_set_power(int power)
{
	msleep( 200 );
	/* PWR == !STANDBY */
	gpio_set_value(TT_VGPIO_GPS_STANDBY, !power);
	msleep( 200 );

	if (power)
		pr_info("atheros gps switched ON\n");
	else
		pr_info("atheros gps switched OFF\n");
}

void atheros_reset(void)
{
	gpio_set_value(TT_VGPIO_GPS_RESET, 1);
	mdelay(5);
	gpio_set_value(TT_VGPIO_GPS_RESET, 0);
	mdelay(5);

	pr_info("atheros gps reset\n");
}	

static struct generic_gps_info machinfo =
{
	.name			= "ar1511",
	.gps_set_power		= atheros_set_power,
	.gps_reset		= atheros_reset
};

static void atheros_dev_release(struct device *dev){}

static struct resource atheros_resource = {
	.name  = "atheros_1PPS",
#ifndef CONFIG_MACH_CATANIA_S
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
#else
	// CATANIA WS1.1 (before) EINT8(TILT_OUT) and EINT9(GPS_1PPS) trigger edge conflict, force GPS_1PPS LOWEDGE as well.
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
#endif
	.start = IRQ_EINT(9), //Beware, may not always be common across SoCs...
	.end   = IRQ_EINT(9),
};

static struct platform_device atheros_pdev =
{
	.name			= "tomtom-gps",
	.id			= -1,
	.num_resources		= 1,
	.resource		= &atheros_resource,
	.dev 			= {
		.platform_data	= &machinfo,
		.release	= atheros_dev_release,
	},
};

static __init int atheros_init(char * identifier)
{
	int err;
	int irqnr;

	if ((err = gpio_request(TT_VGPIO_GPS_STANDBY, "STANDBY"))) {
		pr_err("Can't request STANDBY GPIO\n");
		return err;
	}

	if ((err = gpio_request(TT_VGPIO_GPS_RESET, "RESET"))) {
		pr_err("Can't request RESET GPIO\n");
		gpio_free(TT_VGPIO_GPS_STANDBY);
		return err;
	}

	if ((err = gpio_request(TT_VGPIO_GPS_1PPS, "1PPS"))) {
		pr_err("Can't request 1PPS GPIO\n");
		gpio_free(TT_VGPIO_GPS_RESET);
		gpio_free(TT_VGPIO_GPS_STANDBY);
		return err;
	}

	irqnr  = gpio_to_irq(vgpio_to_gpio(TT_VGPIO_GPS_1PPS));
	atheros_resource.start = irqnr;
	atheros_resource.end = irqnr;

	/* 
	 * Powersave on Catania-s is largely achieved by powering down the
	 * perihperals iso taking down the voltage domains. In order to have 
	 * this work some of the GPIOs for GPS need to be configured such
	 * that they stay in the correct state on powerdown. The GPS has its 
	 * own little firmware running which can go into powerdown. The 
	 * suspend will be done from daemon. But the gps will wakeup on hw 
	 * signal reset, or break on uart. So these GPIOs need to be configured
	 * correctly for sleep state.
	 */
	gpio_direction_output(TT_VGPIO_GPS_RESET, 0);
	/* 
	 * we have to go through s3c functions, which means we have to configure
	 * real levels iso virtual levels, so output 1 !!
	 */
	s3c_gpio_cfgpin_slp(vgpio_to_gpio(TT_VGPIO_GPS_RESET), S3C_GPIO_SLEEP_OUTPUT1);
	/* pullup needs to be configured otherwise it wont work */
	s3c_gpio_setpull_slp(vgpio_to_gpio(TT_VGPIO_GPS_RESET),S3C_GPIO_PULL_UP);
	/* 
	 * The uart port is not configured, so it has no vgpio version. Lets use
	 * direct pin numbering instead.
	 */
	/* Rx/Tx */ 
	s3c_gpio_cfgpin_slp(S5P64XX_GPA(4), S3C_GPIO_SLEEP_PREVIOUSSTATE);
	s3c_gpio_cfgpin_slp(S5P64XX_GPA(5), S3C_GPIO_SLEEP_PREVIOUSSTATE);
	s3c_gpio_setpull_slp(S5P64XX_GPA(4), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull_slp(S5P64XX_GPA(5), S3C_GPIO_PULL_UP);
	/* RTS/CTS */ 
	s3c_gpio_cfgpin_slp(S5P64XX_GPB(5), S3C_GPIO_SLEEP_PREVIOUSSTATE);
	s3c_gpio_cfgpin_slp(S5P64XX_GPB(6), S3C_GPIO_SLEEP_PREVIOUSSTATE);
	s3c_gpio_setpull_slp(S5P64XX_GPB(5), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull_slp(S5P64XX_GPB(6), S3C_GPIO_PULL_UP);

	gpio_direction_output(TT_VGPIO_GPS_STANDBY, 0);
	gpio_direction_input(TT_VGPIO_GPS_1PPS);

	atheros_reset();

	if ((err = platform_device_register(&atheros_pdev))) {
		pr_err("Can't register atheros_pdev (%d)\n", err);
		return err;
	}
	
	return 0;
};

TT_SETUP_CB(atheros_init , "tomtom-gps-atheros");
