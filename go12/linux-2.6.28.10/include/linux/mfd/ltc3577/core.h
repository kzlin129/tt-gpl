/*
 *  core.h  -- Core driver for LTC3577
 *
 *  Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 *  Author: Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_LTC3577_CORE_H_
#define __LINUX_MFD_LTC3577_CORE_H_

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/platform_device.h>

/*
 * PMIC sub register definiton
 */
#define LTC3577_BCK			0x0
#define LTC3577_LED			0x1
#define LTC3577_DAC			0x2
#define LTC3577_PWM			0x3
#define LTC3577_MAX_REGISTER		1

struct ltc3577 {
	struct device		*dev;
	struct i2c_client	*i2c_client;

	struct mutex		lock;

	uint8_t (*reg_read)(struct ltc3577 *ltc);
	int     (*reg_write)(struct ltc3577 *ltc, uint8_t reg, uint8_t val);
	int     (*reg_write_n)(struct ltc3577 *ltc, uint8_t pwm, uint8_t dac, uint8_t led);
};

struct ltc3577_platform_data {
	int (*init)(struct ltc3577 *ltc);
};

static inline struct ltc3577 *dev_to_ltc3577(struct device *dev)
{
	return dev_get_drvdata(dev->parent);
}

int ltc3577_client_register(struct ltc3577 *ltc, struct platform_device *pdev);

#endif /*__LINUX_MFD_LTC3577_CORE_H_ */
