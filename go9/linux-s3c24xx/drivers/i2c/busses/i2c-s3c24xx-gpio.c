/*
 * drivers/i2c/i2c-adap-ixp4xx.c
 *
 * S3C24XX I2C driver using GPIO pins (to get a second I2C controller).
 *
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 * Based on ixp4xx GPIO I2C driver, originally written by Deepax Saxana.
 *
 * Copyright (c) 2007 TomTom International BV.
 *
 * This file is licensed under the terms of the GNU General Public 
 * License version 2. This program is licensed "as is" without any 
 * warranty of any kind, whether express or implied.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <barcelona/s3c24xx_gpio_i2c.h>

static inline gopin_t s3c24xx_gpio_scl_pin( void *data )
{
	return ((struct s3c24xx_gpio_i2c_pins *)data)->scl_pin;
}

static inline gopin_t s3c24xx_gpio_sda_pin( void *data )
{
	return ((struct s3c24xx_gpio_i2c_pins *)data)->sda_pin;
}

static void s3c24xx_gpio_bit_setscl( void *data, int val )
{
	gopin_t	scl_pin=s3c24xx_gpio_scl_pin( data );

	val ? IOP_SetInput( scl_pin ) : IOP_Deactivate( scl_pin ); 
	return;
}

static void s3c24xx_gpio_bit_setsda( void *data, int val )
{
	gopin_t	sda_pin=s3c24xx_gpio_sda_pin( data );

	val ? IOP_SetInput( sda_pin ) : IOP_Deactivate( sda_pin ); 
	return;
}

static int s3c24xx_gpio_bit_getscl( void *data )
{
	gopin_t	scl_pin=s3c24xx_gpio_scl_pin( data );

	IOP_SetInput( scl_pin );
	return IOP_GetInput( scl_pin ) ? 1 : 0;
}	

static int s3c24xx_gpio_bit_getsda( void *data )
{
	gopin_t	sda_pin=s3c24xx_gpio_sda_pin( data );

	IOP_SetInput( sda_pin );
	return IOP_GetInput( sda_pin ) ? 1 : 0;
}	

struct s3c24xx_gpio_i2c_data {
	struct s3c24xx_gpio_i2c_pins *gpio_pins;
	struct i2c_adapter adapter;
	struct i2c_algo_bit_data algo_data;
};

static int s3c24xx_gpio_i2c_remove(struct device *dev)
{
	struct platform_device *plat_dev = to_platform_device(dev);
	struct s3c24xx_gpio_i2c_data *drv_data = dev_get_drvdata(&plat_dev->dev);

	dev_set_drvdata(&plat_dev->dev, NULL);

	i2c_bit_del_bus(&drv_data->adapter);

	kfree(drv_data);

	return 0;
}

static int s3c24xx_gpio_i2c_probe(struct device *dev)
{
	int err;
	struct platform_device *plat_dev = to_platform_device(dev);
	struct s3c24xx_gpio_i2c_pins *gpio = plat_dev->dev.platform_data;
	struct s3c24xx_gpio_i2c_data *drv_data = 
		kmalloc(sizeof(struct s3c24xx_gpio_i2c_data), GFP_KERNEL);

	if(!drv_data)
		return -ENOMEM;

	memzero(drv_data, sizeof(struct s3c24xx_gpio_i2c_data));
	drv_data->gpio_pins = gpio;

	/*
	 * We could make a lot of these structures static, but
	 * certain platforms may have multiple GPIO-based I2C
	 * buses for various device domains, so we need per-device
	 * algo_data->data. 
	 */
	drv_data->algo_data.data = gpio;
	drv_data->algo_data.setsda = s3c24xx_gpio_bit_setsda;
	drv_data->algo_data.setscl = s3c24xx_gpio_bit_setscl;
	drv_data->algo_data.getsda = s3c24xx_gpio_bit_getsda;
	drv_data->algo_data.getscl = s3c24xx_gpio_bit_getscl;
	drv_data->algo_data.udelay = 10;
	drv_data->algo_data.mdelay = 10;
	drv_data->algo_data.timeout = 100;

	drv_data->adapter.id = I2C_HW_B_S3C24xx_GPIO;
	drv_data->adapter.algo_data = &drv_data->algo_data;

	drv_data->adapter.dev.parent = &plat_dev->dev;

	IOP_SetInput( gpio->scl_pin );
	IOP_SetInput( gpio->sda_pin );

	if ((err = i2c_bit_add_bus(&drv_data->adapter) != 0)) {
		printk(KERN_ERR "ERROR: Could not install %s\n", dev->bus_id);

		kfree(drv_data);
		return err;
	}

	dev_set_drvdata(&plat_dev->dev, drv_data);

	return 0;
}

static struct device_driver s3c24xx_gpio_i2c_driver = {
	.name		= "S3C24XX-GPIO-I2C",
	.bus		= &platform_bus_type,
	.probe		= s3c24xx_gpio_i2c_probe,
	.remove		= s3c24xx_gpio_i2c_remove,
};

static int __init s3c24xx_gpio_i2c_init(void)
{
	return driver_register(&s3c24xx_gpio_i2c_driver);
}

static void __exit s3c24xx_gpio_i2c_exit(void)
{
	driver_unregister(&s3c24xx_gpio_i2c_driver);
}

module_init(s3c24xx_gpio_i2c_init);
module_exit(s3c24xx_gpio_i2c_exit);

MODULE_DESCRIPTION("GPIO-based I2C adapter for S3C24xx systems");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rogier Stam <rogier.stam@tomtom.com>");

