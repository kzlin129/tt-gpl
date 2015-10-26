/*
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/> 
 * Author: Andrzej Zukowski <andrzej.zukowski@tomtom.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation. 
 *
 */
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/vgpio.h>
#include <linux/gpio.h>

#include <plat/tt_setup_handler.h>
#include <plat/csr_bc_mode.h>
#include <plat/irvine.h>
#include <plat/fdt.h>

#include <mach/pinmux.h>

static int power_enabled = 0;

static int bt_gpio_request(void)
{
	return 0;
}

static void bt_gpio_free(void)
{
	gpio_free(TT_VGPIO_BT_RST);
	
}

static void bt_config_gpio(void)
{
	gpio_direction_output(TT_VGPIO_BT_RST, 1);
}

static void bt_reset(int activate)
{
	if (activate)
		gpio_set_value(TT_VGPIO_BT_RST, 0);
	else
		gpio_set_value(TT_VGPIO_BT_RST, 1);
}

/* 
 * The pio lines can be connected to the SoCs GPIO or hardwired to GND or VCC
 * In our case all pins are hardwired to GND
 *
 * PIO [0][1][4]
 * 0 0 0 UART,BCSP
 * 0 0 1 BCSP(2 stop bits,no parity)
 * 0 1 0 USB,16MHz
 * 0 1 1 USB,26MHz
 * 1 0 0 Three-wire UART
 * 1 0 1 H4DS
 * 1 1 0 UART(H4)
 * 1 1 1 Undefine internal weak pull-down
 */
static int bt_pio0(void) {
	return 0;
}

static int bt_pio1(void) {
	return 0;
}

static int bt_pio4(void) {
	return 0;
}

static void bt_suspend (void)
{
	gpio_set_value(TT_VGPIO_BT_EN, 0);
}

static void bt_resume (void)
{
	gpio_set_value(TT_VGPIO_BT_EN, power_enabled);
}

static void bt_init(void)
{
	static int init = 0;

	if (init)
		return;

	gpio_direction_output (TT_VGPIO_BT_RST, 0);
	gpio_direction_output (TT_VGPIO_BT_EN, 0);

	gpio_set_value (TT_VGPIO_BT_EN, 0);
	gpio_set_value (TT_VGPIO_BT_RST, 0);

	init = 1;
}


static void power(int activate)
{
	if ( activate )
	{
		gpio_set_value (TT_VGPIO_BT_EN, 0);
		gpio_set_value (TT_VGPIO_BT_RST, 0);
		mdelay(100);
		gpio_set_value (TT_VGPIO_BT_EN, 1);
	}
	else
	{
		gpio_set_value (TT_VGPIO_BT_EN, 0);
		gpio_set_value (TT_VGPIO_BT_RST, 0);
	}
}

/* Call back from the CSR-BC subsystem: te the chip when writing something into /sys/devices/platform/csr-bc/power */
static void bt_power(struct csr_bc_info *info, int activate)
{
	printk (KERN_INFO "Bluetooth power[%d]\n", activate);

	bt_init();

	power(activate);
	power_enabled = activate;
}

/* Call back from the CSR-BC subsystem: reset the chip when writing something into /sys/devices/platform/csr-bc/reset */
static void bt_do_reset(struct csr_bc_info *info)
{
	printk (KERN_INFO "Bluetooth reset\n");

	bt_init();

	bt_reset(1);
	mdelay(100);
	bt_reset(0);
}

static struct csr_bc_info bt_pdata = {
//	.reset			= bcm4760_reset,
//	.set_pio0		= NULL,
//	.set_pio1		= NULL,
//	.set_pio4		= NULL,

	.get_pio0		= bt_pio0,
	.get_pio1		= bt_pio1,
	.get_pio4		= bt_pio4,

	.suspend		= bt_suspend,
	.resume			= bt_resume,

	.config_gpio		= bt_config_gpio,
	.request_gpio		= bt_gpio_request,
	.free_gpio		= bt_gpio_free,

	.bt_power		= bt_power,
	.do_reset		= bt_do_reset,
};

static struct platform_device bt_pdev = 
{
	.name 	= "csr-bc",
	.id	= -1,
	.dev 	= {
		.platform_data = ((void *) &bt_pdata),
	},
};

static int  __init tt_bcm2070_setup (char *str)
{
	printk (KERN_INFO "Initializing standalone bluetooth\n");

	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_BT_EN) , BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio(TT_VGPIO_BT_RST), BCM4760_PIN_MUX_GPIO);

	if (platform_device_register(&bt_pdev))
		printk(KERN_ERR "Could not register csr-bc platform device\n");
	else
		printk(KERN_INFO "csr-bc platform device registered\n");

	return 0;
}

TT_SETUP_CB(tt_bcm2070_setup, "tomtom-bcm-bluetooth-bcm2070");
