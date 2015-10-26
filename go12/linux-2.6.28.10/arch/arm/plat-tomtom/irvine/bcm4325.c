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
#include <linux/broadcom/gpio.h>
#include <linux/delay.h>
#include <linux/vgpio.h>
#include <linux/gpio.h>

#include <plat/tt_setup_handler.h>
#include <plat/csr_bc_mode.h>
#include <plat/irvine.h>
#include <plat/fdt.h>

#include <mach/pinmux.h>

#define GPIO_SDIO2_CLK		82
#define GPIO_SDIO2_CMD		83
#define GPIO_SDIO2_DATA0	84
#define GPIO_SDIO2_DATA1	85
#define GPIO_SDIO2_DATA2	86
#define GPIO_SDIO2_DATA3	87

#define GPIO_PULLUP		1
#define GPIO_PULLDOWN		0

static int bcm4760_gpio_request(void)
{
	/* NOTE: Since this GPIO has been autorequested in the init method, we
	   don't need to request it yet again, it would return error if we did */

	return 0;
}

static void bcm4760_gpio_free(void)
{
	gpio_free(TT_VGPIO_BT_RST);
}

static void bcm4760_config_gpio(void)
{
	gpio_direction_output(TT_VGPIO_BT_RST, 1);
}

static void bcm4760_reset(int activate)
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
static int bcm4760_pio0(void) {
	return 0;
}

static int bcm4760_pio1(void) {
	return 0;
}

static int bcm4760_pio4(void) {
	return 0;
}

static void bcm4760_suspend (void)
{
	gpio_direction_output(TT_VGPIO_BT_RST, 0);
}

static void bcm4760_resume (void)
{
	gpio_direction_output(TT_VGPIO_BT_RST, 1);
}

/* Call back from the CSR-BC subsystem: reset the chip when writing something into /sys/devices/platform/csr-bc/reset */
static void bcm4760_bt_do_reset(struct csr_bc_info *info)
{
	bcm4760_reset(1);

	mdelay(100);

	/* At the moment, we do not wish to activate the wireless in that chip */
	bcm4760_reset(0);

	mdelay(100);
}

static struct csr_bc_info bcm4760_pdata = 
{
//	.reset		= bcm4760_reset,
//	.set_pio0	= NULL,
//	.set_pio1	= NULL,
//	.set_pio4	= NULL,

	.get_pio0	= bcm4760_pio0,
	.get_pio1	= bcm4760_pio1,
	.get_pio4	= bcm4760_pio4,

	.suspend	= bcm4760_suspend,
	.resume		= bcm4760_resume,

	.config_gpio	= bcm4760_config_gpio,
	.request_gpio	= bcm4760_gpio_request,
	.free_gpio	= bcm4760_gpio_free,

	.do_reset	= bcm4760_bt_do_reset,
};

static struct platform_device bt_pdev = 
{
	.name 	= "csr-bc",
	.id 	= -1,
	.dev = {
		.platform_data = ((void *) &bcm4760_pdata),
	},
};

static int  __init tt_bcm4325_setup (char *str)
{
	printk("Initializing bluetooth-wifi combo\n");

	bcm4760_set_pin_mux( 7, BCM4760_PIN_MUX_SF1);

	bcm4760_set_pin_mux( vgpio_to_gpio(TT_VGPIO_BT_RST), BCM4760_PIN_MUX_GPIO );
	bcm4760_set_pin_mux( vgpio_to_gpio(TT_VGPIO_REG_ON), BCM4760_PIN_MUX_GPIO );
	bcm4760_set_pin_mux( vgpio_to_gpio(TT_VGPIO_WL_RST), BCM4760_PIN_MUX_GPIO );

	gpio_direction_output(TT_VGPIO_WL_RST, 0);
	gpio_direction_output(TT_VGPIO_BT_RST, 0);
	gpio_direction_output(TT_VGPIO_REG_ON, 0);
	mdelay(1);
	
	gpio_set_value(TT_VGPIO_REG_ON, 0);
	gpio_set_value(TT_VGPIO_WL_RST, 0);
	gpio_set_value(TT_VGPIO_BT_RST, 0);
	mdelay(100);

	gpio_set_value(TT_VGPIO_REG_ON, 1);
	mdelay(100);

	gpio_set_value(TT_VGPIO_BT_RST, 1);
	gpio_set_value(TT_VGPIO_WL_RST, 1);

	/*
	 * TODO: We shouldn't pull up resistors on SDIO bus, however 
	 * bcm4325 seems to have them disabled. Establish with broadcom 
	 * why this is the current state of affairs.
	 */
	reg_gpio_set_pull_up_down( GPIO_SDIO2_CLK,   GPIO_PULLUP);
	reg_gpio_set_pull_up_down( GPIO_SDIO2_CMD,   GPIO_PULLUP);
	reg_gpio_set_pull_up_down( GPIO_SDIO2_DATA0, GPIO_PULLUP);
	reg_gpio_set_pull_up_down( GPIO_SDIO2_DATA1, GPIO_PULLUP);
	reg_gpio_set_pull_up_down( GPIO_SDIO2_DATA2, GPIO_PULLUP);
	reg_gpio_set_pull_up_down( GPIO_SDIO2_DATA3, GPIO_PULLUP);

	reg_gpio_set_pull_up_down_enable(GPIO_SDIO2_CLK);
	reg_gpio_set_pull_up_down_enable(GPIO_SDIO2_CMD);
	reg_gpio_set_pull_up_down_enable(GPIO_SDIO2_DATA0);
	reg_gpio_set_pull_up_down_enable(GPIO_SDIO2_DATA1);
	reg_gpio_set_pull_up_down_enable(GPIO_SDIO2_DATA2);
	reg_gpio_set_pull_up_down_enable(GPIO_SDIO2_DATA3);

	if (platform_device_register(&bt_pdev))
		printk(KERN_ERR "Could not register csr-bc platform device\n");
	else
		printk(KERN_INFO "csr-bc platform device registered\n");

	return 0;
}
TT_SETUP_CB(tt_bcm4325_setup, "tomtom-bcm-bluetooth-bcm4325");

