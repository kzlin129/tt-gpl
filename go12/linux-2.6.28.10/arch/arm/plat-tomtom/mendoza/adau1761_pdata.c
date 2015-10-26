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
#include <plat/adau1761_pdata.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <plat/mendoza.h>
#include <plat/gpio-cfg.h>

static int adau1761_gpio_request(void)
{
	int err = 0;

	if ((err = gpio_request(TT_VGPIO_MIC_STBY, "TT_VGPIO_MIC_STBY"))) {
		printk("%s: Can't request TT_VGPIO_MIC_STBY GPIO\n", __func__);
		goto request_err;
	}

	if ((err = gpio_request(TT_VGPIO_CODEC_PWR_EN, "TT_VGPIO_CODEC_PWR_EN"))) {
		printk("%s: Can't request TT_VGPIO_CODEC_PWR_EN GPIO\n", __func__);
		goto request_pwr_en_err;
	}

	if ((err = gpio_request(TT_VGPIO_AMP_PWR_EN, "TT_VGPIO_AMP_PWR_EN"))) {
		printk("%s: Can't request TT_VGPIO_AMP_PWR_EN GPIO\n", __func__);
		goto request_amp_en_err;
	}

	return err;

request_amp_en_err:
	gpio_free(TT_VGPIO_CODEC_PWR_EN);
request_pwr_en_err:
	gpio_free(TT_VGPIO_MIC_STBY);
request_err:
	return err;
}

static void adau1761_gpio_free(void)
{
	gpio_free(TT_VGPIO_MIC_STBY);
	gpio_free(TT_VGPIO_CODEC_PWR_EN);
	gpio_free(TT_VGPIO_AMP_PWR_EN);
}

static void adau1761_config_gpio(void)
{
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_MIC_STBY), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_CODEC_PWR_EN), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_AMP_PWR_EN), S3C_GPIO_PULL_NONE);
	gpio_direction_output(TT_VGPIO_MIC_STBY, 0);
	gpio_direction_output(TT_VGPIO_CODEC_PWR_EN, 0);
	gpio_direction_output(TT_VGPIO_AMP_PWR_EN, 0);
}

static void mic_stby_n(int activate)
{
	gpio_set_value(TT_VGPIO_MIC_STBY, activate);
}

static void codec_pwr_en(int activate)
{
	gpio_set_value(TT_VGPIO_CODEC_PWR_EN, activate);

	/* Looks like the codec needs some time to relax after PWR_EN... */
	if (activate)
		msleep(10);
}

static void amp_pwr_en(int activate)
{
	gpio_set_value(TT_VGPIO_AMP_PWR_EN, activate);
}

static void adau1761_suspend (void)
{
	gpio_direction_output(TT_VGPIO_MIC_STBY, 0);
	gpio_direction_output(TT_VGPIO_CODEC_PWR_EN, 0);
	gpio_direction_output(TT_VGPIO_AMP_PWR_EN, 0);
}

static void adau1761_resume (void)
{
	gpio_direction_output(TT_VGPIO_MIC_STBY, 1);
	gpio_direction_output(TT_VGPIO_CODEC_PWR_EN, 1);
	gpio_direction_output(TT_VGPIO_AMP_PWR_EN, 1);
}

static adau1761_platform_t adau1761_pdata = {
	.codec_pwr_en	= codec_pwr_en,
	.amp_pwr_en		= amp_pwr_en,
	.mic_stby_n		= mic_stby_n,

	.suspend		= adau1761_suspend,
	.resume			= adau1761_resume,

	.config_gpio	= adau1761_config_gpio,
	.request_gpio	= adau1761_gpio_request,
	.free_gpio		= adau1761_gpio_free,
};

static struct i2c_board_info	adau1761_i2c_info = {
	I2C_BOARD_INFO(ADAU1761_DEVNAME, 0x38),
	.platform_data = &adau1761_pdata,
};

static int __init adau1761_register(void)
{
	int err;
	struct i2c_adapter * adapter;
	struct i2c_client * client;

	adapter = i2c_get_adapter(0);
	if (adapter) {
		client = i2c_new_device(adapter, &adau1761_i2c_info);
		if (!client) {
			printk( "Can't register adau1761 I2C Board\n" );
			adau1761_gpio_free();
			return -ENODEV;
		}
	} else {
		if((err = i2c_register_board_info(0, &adau1761_i2c_info, 1)) < 0) {
			printk( "Can't register adau1761 I2C Board\n" );
			adau1761_gpio_free();
			return err;
		}
	}

	return 0;
}
arch_initcall(adau1761_register);

