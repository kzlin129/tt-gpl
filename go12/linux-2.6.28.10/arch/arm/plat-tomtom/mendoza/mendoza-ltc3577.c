/*
 *  I2C board info for ltc3577 wired to a mendoza board
 *
 *  Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 *  Author: Marc Zyngier <marc.zyngier@tomtom.com>
 *          Andrzej Zukowski <andrzej.zukowski@tomtom.com>

 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mfd/ltc3577/core.h>

#include <plat/mendoza_ltc3577.h>

struct platform_device *mendoza_ltc3577_devs[] = {
	&mendoza_ltc3577_bl,
	&mendoza_ltc3577_pmic,
};

static int mendoza_ltc3577_init(struct ltc3577 *ltc)
{
	int i, ret;

	for (i = 0 ; i < ARRAY_SIZE(mendoza_ltc3577_devs); i++)
	{
		ret = ltc3577_client_register(ltc, mendoza_ltc3577_devs[i]);
		if (ret)
		{
			dev_err(ltc->dev, "Can't register %s (%d)\n",
				mendoza_ltc3577_devs[i]->name, ret);
			return ret;
		}
	}

	return 0;
}

static struct ltc3577_platform_data mendoza_ltc3577_pdata = {
	.init	= mendoza_ltc3577_init,
};

static struct i2c_board_info mendoza_ltc3577_i2c_info = {
	I2C_BOARD_INFO("ltc3577", 0x09),
	.platform_data = &mendoza_ltc3577_pdata,
};

int __init mendoza_ltc3577_i2c_init(void)
{
	printk(KERN_ERR "xxx %s\n", __func__);
	return i2c_register_board_info(0, &mendoza_ltc3577_i2c_info, 1);
}

