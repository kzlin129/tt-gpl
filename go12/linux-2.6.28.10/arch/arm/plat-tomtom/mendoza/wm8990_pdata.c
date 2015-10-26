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
#include <linux/i2c.h>
#include <plat/wm8990_pdata.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <plat/mendoza.h>
#include <plat/gpio-cfg.h>

static int wm8990_gpio_request(void)
{
	int err = 0;

	if (gpio_is_valid(TT_VGPIO_CODEC_PWR_EN)) {
		if ((err = gpio_request(TT_VGPIO_CODEC_PWR_EN, "TT_VGPIO_CODEC_PWR_EN"))) {
			printk("%s: Can't request TT_VGPIO_CODEC_PWR_EN GPIO\n", __func__);
			goto err_free;
		}
	}

	if (gpio_is_valid(TT_VGPIO_AMP_PWR_EN)) {
		if ((err = gpio_request(TT_VGPIO_AMP_PWR_EN, "TT_VGPIO_AMP_PWR_EN"))) {
			printk("%s: Can't request TT_VGPIO_AMP_PWR_EN GPIO\n", __func__);
			goto err_free_pwr;
		}
	}


err_free_pwr:
	gpio_free(TT_VGPIO_CODEC_PWR_EN);
err_free_amp:
	gpio_free(TT_VGPIO_AMP_PWR_EN);
err_free:
	return err;
}

static void wm8990_gpio_free(void)
{
	gpio_free(TT_VGPIO_L3_MODE);
	gpio_free(TT_VGPIO_CODEC_PWR_EN);
	gpio_free(TT_VGPIO_AMP_PWR_EN);
}

static void wm8990_config_gpio(void)
{
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_CODEC_PWR_EN), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_AMP_PWR_EN), S3C_GPIO_PULL_NONE);
	gpio_direction_output(TT_VGPIO_CODEC_PWR_EN, 1);
	gpio_direction_output(TT_VGPIO_AMP_PWR_EN, 1);
}

static void codec_pwr_en(int activate)
{
	gpio_set_value(TT_VGPIO_CODEC_PWR_EN, activate);
}

static void amp_pwr_en(int activate)
{
	gpio_set_value(TT_VGPIO_AMP_PWR_EN, activate);
}

static void wm8990_suspend (void)
{
	gpio_direction_output(TT_VGPIO_CODEC_PWR_EN, 0);
	gpio_direction_output(TT_VGPIO_AMP_PWR_EN, 0);
	gpio_direction_output(TT_VGPIO_L3_MODE, 0);
}

static void wm8990_resume (void)
{
	gpio_direction_output(TT_VGPIO_CODEC_PWR_EN, 1);
	gpio_direction_output(TT_VGPIO_AMP_PWR_EN, 1);
	gpio_direction_output(TT_VGPIO_L3_MODE, 1);
}

static wm8990_platform_t wm8990_pdata = {
	.codec_pwr_en	= codec_pwr_en,
	.amp_pwr_en		= amp_pwr_en,

	.suspend		= wm8990_suspend,
	.resume			= wm8990_resume,

	.config_gpio	= wm8990_config_gpio,
	.request_gpio	= wm8990_gpio_request,
	.free_gpio		= wm8990_gpio_free,
};

static struct i2c_board_info	wm8990_i2c_info = {
	I2C_BOARD_INFO(WM8990_DEVNAME, 0x1A),
	.platform_data = &wm8990_pdata,
};

static int __init wm8990_register(void)
{
	int err;
	struct i2c_adapter * adapter;
	struct i2c_client * client;

	/* Set L3_MODE pin which determines the connection type */
	if (gpio_is_valid(TT_VGPIO_L3_MODE)) {
		if ((err = gpio_request(TT_VGPIO_L3_MODE, "TT_VGPIO_L3_MODE"))) {
			printk("%s: Can't request TT_VGPIO_L3_MODE GPIO\n", __func__);
			return err;
		}
	}
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_L3_MODE), S3C_GPIO_PULL_NONE);
	gpio_direction_output(TT_VGPIO_L3_MODE, 1);
	gpio_set_value(TT_VGPIO_L3_MODE, 0);

	adapter = i2c_get_adapter(0);
	if (adapter) {
		client = i2c_new_device(adapter, &wm8990_i2c_info);
		if (!client) {
			printk( "Can't register wm8990 I2C Board\n" );
			return -ENODEV;
		}
	} else {
		if((err = i2c_register_board_info(0, &wm8990_i2c_info, 1)) < 0) {
			printk( "Can't register wm8990 I2C Board\n" );
			return err;
		}
	}

	return 0;
}
arch_initcall(wm8990_register);

