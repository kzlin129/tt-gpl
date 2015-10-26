/* ltc3577-pmic.h
 *
 * Control driver for LTC3577 PMIC.
 *
 * Copyright (C) 2008 TomTom BV <http://www.tomtom.com/>
 * Authors: Benoit Leffray <benoit.leffray@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef INCLUDE_LINUX_LTC3577_REGULATOR_H
#define INCLUDE_LINUX_LTC3577_REGULATOR_H

#define LTC3577_REG_POWER_MIN	0
#define LTC3577_REG_POWER_MAX	100

typedef struct
{
	int (*init)(void);
	int (*set_power)(int power);
}  ltc3577_reg_pdata_t;

int ltc3577_regulator_set_power(int power);

#define LTC3577_REG_DEVNAME			"tomtom-ltc3577-regulator"
#endif
