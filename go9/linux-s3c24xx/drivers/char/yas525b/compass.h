/*
 * Compass driver for TomTom GO devices.
 *
 *              This program is free software; you can redistribute it and/or
 *              modify it under the terms of the GNU General Public License
 *              as published by the Free Software Foundation; either version
 *              2 of the License, or (at your option) any later version.
 *
 * Author:      Ithamar R. Adema, <ithamar.adema@tomtom.com>
 */

#ifndef __DRIVERS_CHAR_YAS525B_COMPASS_H
#define __DRIVERS_CHAR_YAS525B_COMPASS_H	__FILE__

#include <linux/cdev.h>
#include <linux/i2c.h>

/* ADDR is the address of the register we're trying to read/write */
#define YAS525B_ADDR_MASK		0xC0
#define YAS525B_ADDR_SHIFT		6

/* YAS525B Registers */
#define YAS525B_CMDR			0x00

#define YAS525B_CMDR_CMD_MASK		7
#define YAS525B_CMDR_CMD_XNORM		0
#define YAS525B_CMDR_CMD_YNORM		1
#define YAS525B_CMDR_CMD_XROUGH		2
#define YAS525B_CMDR_CMD_YROUGH		3
#define YAS525B_CMDR_CMD_TEMP		4

#define YAS525B_OFFSETR			0x01

#define YAS525B_OFFSETR_OFFSET_MASK	0x3f
#define YAS525B_OFFSETR_SELX		(0 << 5)
#define YAS525B_OFFSETR_SELY		(1 << 5)

#define YAS525B_CONFR			0x02

#define YAS525B_CONFR_CAL_ENABLE	(1 << 5)
#define YAS525B_CONFR_COILE_ENABLE	(1 << 3)
#define YAS525B_CONFR_COILSEL_MASK	7

#define YAS525B_TESTR			0x03

#define YAS525B_TESTR_DISABLE		0x00

#define MEASUREMENT_SHIFT		4
#define MEASUREMENT_MASK		0x7ff0
#define MEASUREMENT_BUSY		(1 << 15)

struct compass_data {
	struct cdev cdev;
	struct i2c_client* client;

	int xr, yr;	/* Rough offset X/Y read after initialisation */
	u32 caldata;	/* Calibration data (read from/written to CAL register) */
};

#endif /* __DRIVERS_CHAR_YAS525B_COMPASS_H */

