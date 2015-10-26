/*
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Vincent Dejouy <vincent.dejouy@tomtom.com>
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
#include <linux/psoc-ctsic.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <linux/delay.h>

#include <plat/mendoza.h>
#include <plat/tt_setup_handler.h>
#include <plat/gpio-cfg.h>
#include <plat/fdt.h>

#define CAP_TOUCH_SCREEN_E 	0
#define CAP_TOUCH_SCREEN_D	1

static int psoc_gpio_request(void)
{
	int err = 0;

	if ((err = gpio_request(TT_VGPIO_TSP_PWR, "TT_VGPIO_TSP_PWR"))) {
		printk(KERN_ERR PSOC_DEVNAME ":Can't request TT_VGPIO_TSP_PWR GPIO\n");
		gpio_free(TT_VGPIO_TSP_PWR);
		return err;
	}

	if ((err = gpio_request(TT_VGPIO_TSP_CE, "TT_VGPIO_TSP_CE"))) {
		printk(KERN_ERR PSOC_DEVNAME ":Can't request TT_VGPIO_TSP_CE GPIO\n");
		gpio_free(TT_VGPIO_TSP_PWR);
		gpio_free(TT_VGPIO_TSP_CE);
		return err;
	}

	if ((err = gpio_request(TT_VGPIO_TSP_ATTN, "TT_VGPIO_TSP_ATTN"))) {
		printk(KERN_ERR PSOC_DEVNAME ":Can't request TT_VGPIO_TSP_ATTN GPIO\n");
		gpio_free(TT_VGPIO_TSP_PWR);
		gpio_free(TT_VGPIO_TSP_CE);
		gpio_free(TT_VGPIO_TSP_ATTN);
		return err;
	}

	return err;

}

static void psoc_config_gpio(int touch_screen)
{
	gpio_direction_output(TT_VGPIO_TSP_PWR, 0);
	gpio_direction_input(TT_VGPIO_TSP_ATTN);
	s3c_gpio_setpull(vgpio_to_gpio(TT_VGPIO_TSP_ATTN), S3C_GPIO_PULL_UP);

	if (touch_screen == CAP_TOUCH_SCREEN_D)
        	gpio_direction_output(TT_VGPIO_TSP_CE, 1);
	else if (touch_screen == CAP_TOUCH_SCREEN_E)
		gpio_direction_output(TT_VGPIO_TSP_CE, 0);
}

static void psoc_reset_ts_E(void)
{
	//Reset active low
	gpio_set_value(TT_VGPIO_TSP_CE, 0);
	msleep(100);
	gpio_set_value(TT_VGPIO_TSP_CE, 1);
}

static void psoc_reset_ts_D(void)
{
	//Reset active high
	gpio_set_value(TT_VGPIO_TSP_CE, 1);
	msleep(100);
	gpio_set_value(TT_VGPIO_TSP_CE, 0);
}

static void psoc_init(int touch_screen)
{
	if (psoc_gpio_request())
	{
		printk(KERN_ERR PSOC_DEVNAME ": Couldn't initialize Capacitive Touch Screen Controller\n");
		return;
	}

	psoc_config_gpio(touch_screen);
	if (touch_screen == CAP_TOUCH_SCREEN_D)
		psoc_reset_ts_D();
	else if (touch_screen == CAP_TOUCH_SCREEN_E)
		psoc_reset_ts_E();
	printk(KERN_INFO PSOC_DEVNAME ": Initialized\n");
}

static psoc_pdata_t psoc_pdata;

static struct i2c_board_info psoc_i2c_info = 
{
	.addr = 0x20,
	.platform_data = &psoc_pdata,
};

static struct i2c_board_info psoc_bootloader_i2c_info = 
{
	.type = PSOC_BOOTLOADER_DEVNAME,
	.platform_data = &psoc_pdata,
};

int __init irvine_psoc_init (void)
{
	printk (KERN_INFO "Initializing Cypress PSOC\n");

	return 0;
}

static int __init tt_psoc_setup(char *str)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	const char *ts = NULL;
	int i2c_bus;

	i2c_bus = 0;		/* PSOC is on I2C bus 0 */

	ts = fdt_get_string ("/features", "ts", "");
	
	if ( strncmp(ts, "TMA300D", 7) == 0 ) {
		psoc_pdata.reset_ts = psoc_reset_ts_D;
		strcpy(psoc_i2c_info.type, PSOC_340_DEVNAME);
		psoc_init(CAP_TOUCH_SCREEN_D);
	} else if (strncmp(ts, "TMA300E", 7) == 0) {
		psoc_pdata.reset_ts = psoc_reset_ts_E;
		strcpy(psoc_i2c_info.type, PSOC_340_DEVNAME);
		psoc_init(CAP_TOUCH_SCREEN_E);
	} else if (strncmp(ts, "TMA301D", 7) == 0) {
		psoc_pdata.reset_ts = psoc_reset_ts_D;
		strcpy(psoc_i2c_info.type, PSOC_301_DEVNAME);
		psoc_init(CAP_TOUCH_SCREEN_D);
	} else {
		printk(KERN_ERR PSOC_DEVNAME ": Error, ts feature in the factory data is wrong, expected TMA300E or TMA300D\n");
		return -1;
	}

	if ( strstr(ts, "FW_V2") != NULL )
		psoc_bootloader_i2c_info.addr = 0x0A;
	else 
		psoc_bootloader_i2c_info.addr = 0x9;

	psoc_i2c_info.irq = gpio_to_irq(TT_VGPIO_TSP_ATTN);

	adapter = i2c_get_adapter(i2c_bus);
	if (adapter) {
		client = i2c_new_device(adapter, &psoc_i2c_info);
		if (!client) {
			printk(KERN_ERR PSOC_DEVNAME ": Can't add new I2C device\n" );
			return -ENODEV;
		}
		client = i2c_new_device(adapter, &psoc_bootloader_i2c_info);
		if (!client) {
			printk(KERN_ERR PSOC_BOOTLOADER_DEVNAME ": Can't add new I2C device\n" );
			return -ENODEV;
		}
	}

	
	printk(KERN_INFO "Cypress PSOC %s initialized\n", ts);

	return 0;
}

TT_SETUP_CB(tt_psoc_setup, "tomtom-touchscreen-cypress");

