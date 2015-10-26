/*
 *  linux/drivers/mfd/ltc3577-core.c
 *
 *  Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 *  Author: Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/irq.h>

#include <linux/mfd/ltc3577/core.h>

static int __ltc3577_read(struct ltc3577 *ltc, int cnt, uint8_t *buf)
{
	int ret;

	ret = i2c_master_recv(ltc->i2c_client, buf, cnt);
	if (ret < 0)
		return ret;

	if (ret != cnt)
		return -EIO;
	return 0;
}

static int __ltc3577_write(struct ltc3577 *ltc, uint8_t reg, int cnt, uint8_t *buf)
{
	uint8_t msg[LTC3577_MAX_REGISTER + 1];
	int ret;

	if (cnt > LTC3577_MAX_REGISTER)
		return -EINVAL;

	msg[0] = reg;
	memcpy(&msg[1], buf, cnt);

	ret = i2c_master_send(ltc->i2c_client, msg, cnt + 1);
	if (ret < 0)
		return ret;

	if (ret != cnt + 1)
		return -EIO;
	return 0;
}

static int __ltc3577_write_n(struct ltc3577 *ltc, uint8_t pwm, uint8_t dac, uint8_t led)
{
	struct  i2c_client *clt = ltc->i2c_client;

	uint8_t buf_pwm[] = {LTC3577_PWM, pwm};
	uint8_t buf_dac[] = {LTC3577_DAC, dac};
	uint8_t buf_led[] = {LTC3577_LED, led};

	struct  i2c_msg msgs[] = {
		{
			.addr	= clt->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= buf_pwm,
		}, {
			.addr	= clt->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= buf_dac,
		}, {
			.addr	= clt->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= buf_led,
		}
	};

	if (i2c_transfer(clt->adapter, msgs, ARRAY_SIZE(msgs)) == ARRAY_SIZE(msgs))
		return 0;

	return -EINVAL;
}

static uint8_t ltc3577_reg_read(struct ltc3577 *ltc)
{
	uint8_t val;

	mutex_lock(&ltc->lock);
	__ltc3577_read(ltc, 1, &val);
	mutex_unlock(&ltc->lock);

	return val;
}

static int ltc3577_reg_write(struct ltc3577 *ltc, uint8_t reg, uint8_t val)
{
	int ret;

	mutex_lock(&ltc->lock);
	ret = __ltc3577_write(ltc, reg, 1, &val);
	mutex_unlock(&ltc->lock);

	return ret;
}

static int ltc3577_reg_write_n(struct ltc3577 *ltc, uint8_t pwm, uint8_t dac, uint8_t led)
{
	int ret;

	mutex_lock(&ltc->lock);
	ret = __ltc3577_write_n(ltc, pwm, dac, led);
	mutex_unlock(&ltc->lock);

	return ret;
}

int ltc3577_client_register(struct ltc3577 *ltc, struct platform_device *pdev)
{
	int ret;

	if (!pdev)
		return -ENODEV;

	pdev->dev.parent = ltc->dev;

	if ((ret = platform_device_register(pdev)))
		dev_err(ltc->dev, "Failed to register %s: %d\n", pdev->name, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(ltc3577_client_register);

static int __devinit ltc3577_probe(struct i2c_client *client,
				   const struct i2c_device_id *ids)
{
	struct ltc3577_platform_data *pdata;
	struct ltc3577               *ltc;
	int ret = 0;

	ltc = kzalloc(sizeof(struct ltc3577), GFP_KERNEL);
	if (!ltc) {
		dev_err(ltc->dev, "Insufficient amount of memory: %d\n", ret);
		ret = -ENOMEM;
		goto err;
	}

	i2c_set_clientdata(client, ltc);
	mutex_init(&ltc->lock);

	ltc->dev         = &client->dev;
	ltc->i2c_client  = client;
	ltc->reg_read    = ltc3577_reg_read;
	ltc->reg_write   = ltc3577_reg_write;
	ltc->reg_write_n = ltc3577_reg_write_n;

	pdata = client->dev.platform_data;
	if (pdata->init && (ret = pdata->init(ltc))) {
		dev_err(ltc->dev, "Failed to initialize I2C clients: %d\n", ret);
		goto err_devs;
	}

	dev_info(ltc->dev, "LTC3577 Core driver\n");
	return 0;

err_devs:
	kfree(ltc);
err:
	return ret;
}

static int __devexit ltc3577_remove(struct i2c_client *client)
{
	struct ltc3577 *ltc = i2c_get_clientdata(client);

	kfree(ltc);
	return 0;
}

static struct i2c_device_id ltc3577_id_table[] = {
	{"ltc3577", 0},
	{}
};

static struct i2c_driver ltc3577_driver = {
	.driver = {
		.name    = "ltc3577",
	},
	.probe    = ltc3577_probe,
	.remove   = __devexit_p(ltc3577_remove),
	.id_table = ltc3577_id_table,
};

static int __init ltc3577_init(void)
{
	return i2c_add_driver(&ltc3577_driver);
}
subsys_initcall(ltc3577_init);

static void __exit ltc3577_exit(void)
{
	i2c_del_driver(&ltc3577_driver);
}
module_exit(ltc3577_exit);

MODULE_DESCRIPTION("LTC3577 based Core driver");
MODULE_AUTHOR("Andrzej Zukowski <andrzej.zukowski@tomtom.com>");
MODULE_LICENSE("GPL");
