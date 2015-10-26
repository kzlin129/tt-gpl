/*
 * s5m8751-i2c.c  --  S5M8751 Power-Audio IC ALSA Soc Audio driver
 *
 * Copyright 2009 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mfd/s5m8751/s5m8751_core.h>

int s5m8751_i2c_read_device(struct s5m8751 *s5m8751, char reg, int bytes, void *dest)
{
	int ret;
	
	ret = i2c_master_send(s5m8751->i2c_client, &reg, 1);
	if (ret < 0)
		return ret;
	ret = i2c_master_recv(s5m8751->i2c_client, dest, bytes);
	if (ret < 0)
		return ret;
	if (ret != bytes)
		return -EIO;
	return 0;
}
EXPORT_SYMBOL_GPL(s5m8751_i2c_read_device);

int s5m8751_i2c_write_device(struct s5m8751 *s5m8751, char reg, int bytes, void *src)
{
	u8 msg[(S5M8751_MAX_REGISTER << 1) + 1];
	int ret;

	if (bytes > ((S5M8751_MAX_REGISTER << 1) + 1))
		return -EINVAL;

	msg[0] = reg;
	memcpy(&msg[1], src, bytes);
	ret = i2c_master_send(s5m8751->i2c_client, msg, bytes + 1);
	if (ret < 0)
		return ret;
	if (ret != bytes + 1)
		return -EIO;
	return 0;
}
EXPORT_SYMBOL_GPL(s5m8751_i2c_write_device);

static int s5m8751_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct s5m8751 *s5m8751;
	struct s5m8751_platform_data *pdata;
	int ret = 0;
	u8 chip_id;

	s5m8751 = kzalloc(sizeof(struct s5m8751), GFP_KERNEL);
	if (s5m8751 == NULL) {
		kfree(i2c);
		return -ENOMEM;
	}

	pdata = i2c->dev.platform_data;
	i2c_set_clientdata(i2c, s5m8751);
	s5m8751->dev = &i2c->dev;
	s5m8751->i2c_client = i2c;
	s5m8751->read_dev = s5m8751_i2c_read_device;
	s5m8751->write_dev = s5m8751_i2c_write_device;
	
	s5m8751->read_dev(s5m8751, S5M8751_CHIP_ID, sizeof(chip_id), &chip_id);

	if (chip_id == 0x0)
		printk("s5m8751 device is ready!!\n");
	else
		printk("s5m8751 device is not ready!!\n");

	mutex_init(&s5m8751->irq_mutex);

	s5m8751->chip_irq = i2c->irq;	

	s5m8751_reg_write(s5m8751, S5M8751_IRQB_MASK1, 0x0); 
	s5m8751_reg_write(s5m8751, S5M8751_IRQB_MASK2, 0x0);

	if (pdata->init && (ret = pdata->init(s5m8751)))
		goto err;

	return 0;

err:
	kfree(s5m8751);
	return ret;
}

static int s5m8751_i2c_remove(struct i2c_client *i2c)
{
	struct s5m8751 *s5m8751 = i2c_get_clientdata(i2c);
	
	s5m8751_device_exit(s5m8751);	
	kfree(s5m8751);
	return 0;
}

static const struct i2c_device_id s5m8751_i2c_id[] = {
	{"s5m8751", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, s5m8751_i2c_id); 

static struct i2c_driver s5m8751_i2c_driver = {
	.driver = {
		.name = "s5m8751",
		.owner = THIS_MODULE,
	},
	.probe = s5m8751_i2c_probe,
	.remove = s5m8751_i2c_remove,
	.id_table = s5m8751_i2c_id,
};

static int __init s5m8751_i2c_init(void)
{
	return i2c_add_driver(&s5m8751_i2c_driver);
}
subsys_initcall(s5m8751_i2c_init);

static void __exit s5m8751_i2c_exit(void)
{
	i2c_del_driver(&s5m8751_i2c_driver);
}
module_exit(s5m8751_i2c_exit);

MODULE_DESCRIPTION("I2C support for the S5M8751 Power-Audio IC");
MODULE_LICENSE("GPL");
