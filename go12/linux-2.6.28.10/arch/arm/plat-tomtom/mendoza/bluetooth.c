/*
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Authors: Niels Langendorff <niels.langendorff@tomtom.com>
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/mendoza.h>
#include <plat/csr_bc_mode.h>

static int bluetooth_gpio_request(void)
{
	int err = 0;

	if (gpio_is_valid(TT_VGPIO_BT_RST)) {
		if ((err = gpio_request(TT_VGPIO_BT_RST, "TT_VGPIO_BT_RST"))) {
			printk("%s: Can't request TT_VGPIO_BT_RST GPIO\n", __func__);
			return err;
		}
	}
	return 0;
}

static void bluetooth_gpio_free(void)
{
	gpio_free(TT_VGPIO_BT_RST);
}

static void bluetooth_config_gpio(void)
{
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_BT_RST), S3C_GPIO_PULL_NONE);
	gpio_direction_output(TT_VGPIO_BT_RST, 0);
	
	/* The bluetooth device will exit deepsleep on a reset detected on */
	/* uart. To prevent this from happening in suspend the rx/tx uart  */
	/* signals need to be configured in case of suspend so they keep   */
	/* the current state. The pull up setting is needed otherwise the  */
	/* the sleep state setting will not be done.                       */
	s3c_gpio_cfgpin_slp(S5P64XX_GPA(0), S3C_GPIO_SLEEP_PREVIOUSSTATE);
	s3c_gpio_cfgpin_slp(S5P64XX_GPA(1), S3C_GPIO_SLEEP_PREVIOUSSTATE);
	s3c_gpio_setpull_slp(S5P64XX_GPA(0), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull_slp(S5P64XX_GPA(1), S3C_GPIO_PULL_UP);
}

static void bluetooth_reset(int activate)
{
	gpio_set_value(TT_VGPIO_BT_RST, activate);
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
static int bluetooth_pio0(void) {
	return 0;
}

static int bluetooth_pio1(void) {
	return 0;
}

static int bluetooth_pio4(void) {
	return 0;
}

static void bluetooth_suspend (void)
{
	gpio_direction_output(TT_VGPIO_BT_RST, 0);
}

static void bluetooth_resume (void)
{
	gpio_direction_output(TT_VGPIO_BT_RST, 0);
}

static struct csr_bc_info bluetooth_pdata = {
	.reset			= bluetooth_reset,
//	.set_pio0		= NULL,
//	.set_pio1		= NULL,
//	.set_pio4		= NULL,

	.get_pio0		= bluetooth_pio0,
	.get_pio1		= bluetooth_pio1,
	.get_pio4		= bluetooth_pio4,

	.suspend		= bluetooth_suspend,
	.resume			= bluetooth_resume,

	.config_gpio	= bluetooth_config_gpio,
	.request_gpio	= bluetooth_gpio_request,
	.free_gpio		= bluetooth_gpio_free,

//	.do_reset		= NULL,
};

static struct platform_device bt_pdev = {
	.name = "csr-bc",
	.id = -1,
	.dev = {
		.platform_data = ((void *) &bluetooth_pdata),
	},
};

static int __init bluetooth_init( void )
{
	if (platform_device_register(&bt_pdev))
		printk(KERN_ERR "Could not register csr-bc platform device\n");

	return 0;
}
arch_initcall( bluetooth_init );

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Mendoza bluetooth module");
