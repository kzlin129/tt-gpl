/*
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
 
#include <linux/platform_device.h>
#include <linux/vgpio.h>
#include <linux/gpio.h>

#include <plat/tt_setup_handler.h>
#include <plat/fdt.h>
#include <plat/irvine.h>
#include <plat/gprs_types.h>
#include <plat/gprs.h>

#include <mach/pinmux.h>

static int gprs_power( int ena_dis )
{
	if( ena_dis )
		gpio_direction_output( TT_VGPIO_GSM_POWER, 1 );
	else
		gpio_direction_output( TT_VGPIO_GSM_POWER, 0 );
	return 0;
}

static int gprs_reset( int ena_dis )
{
	if( ena_dis )
		gpio_direction_output( TT_VGPIO_GSM_RESET, 1 );
	else
		gpio_direction_output( TT_VGPIO_GSM_RESET, 0 );
	
	return 0;
}

static void gprs_suspend( void )
{
	gpio_direction_output( TT_VGPIO_GSM_RESET, 0 );
	gpio_direction_output( TT_VGPIO_GSM_POWER, 0 );
}

static void gprs_setup_port(void)
{
	/* Do nothing for now. */
}

static struct gprs_platform_data gprs_pdata =
{
	.gprs_power	= gprs_power,
	.gprs_reset	= gprs_reset,
	.gprs_suspend   = gprs_suspend,
	.gprs_setup_port= gprs_setup_port,
	.gprs_id	= GOGPRS_FARO,
};

struct platform_device gprs_pdev =
{
	.name	= "gprs",
	.id	= -1,
	.dev    = {
		.platform_data=((void *) &gprs_pdata),
	},
};

static int __init gprs_setup (char *str)
{
	printk ("Initializing GPRS\n");

	/* Configure the GPIO pins (Reset and Power) */
	bcm4760_set_pin_mux( vgpio_to_gpio(TT_VGPIO_GSM_POWER), BCM4760_PIN_MUX_GPIO );
	bcm4760_set_pin_mux( vgpio_to_gpio(TT_VGPIO_GSM_RESET), BCM4760_PIN_MUX_GPIO );

	gpio_direction_output( TT_VGPIO_GSM_POWER, 0 );
	gpio_direction_output( TT_VGPIO_GSM_RESET, 0 );

	platform_device_register( &gprs_pdev );

	return 0;
}

static int __init tt_gprs_setup3 (char *str)
{
	return gprs_setup(str);
}

static int __init tt_gprs_setup2 (char *str)
{
	return gprs_setup(str);
}

TT_SETUP_CB(tt_gprs_setup3, "tomtom-bcm-gprs-faro3");
TT_SETUP_CB(tt_gprs_setup2, "tomtom-bcm-gprs-faro2");

