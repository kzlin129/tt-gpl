/* alc5628-i2c.c
 *
 * Control driver for Realtek ALC5628 audio codec
 *
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Authors: Ard Biesheuvel <ard.biesheuvel@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "alc5628-i2c.h"
#include "alc5628.h"
#include "codec.h"

#include <linux/module.h>
#include <linux/i2c.h>

MODULE_AUTHOR("Ard Biesheuvel <ard.biesheuvel@tomtom.com>");
MODULE_DESCRIPTION("Control driver for Realtek ALC5682 audio codec");

static unsigned short probe_addr[] = { 0, 0x18, I2C_CLIENT_END };
static unsigned short ignore[] = { I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c	= ignore,
	.probe		= probe_addr,
	.ignore		= ignore,
	.force		= ignore,
};

static struct i2c_driver alc5628_i2c_driver;

static int alc5628_attach(struct i2c_adapter *adapter, int address, int kind)
{	
	struct i2c_client *c;
	int rc;

	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	memset(c, 0, sizeof(*c));
	c->addr = address;
	c->adapter = adapter;
	c->driver = &alc5628_i2c_driver;

	if (0 == (rc = i2c_attach_client(c)))
		alc5628_init_i2c_control(c);

	return rc;
}

static int alc5628_probe(struct i2c_adapter *adap)
{
	return i2c_probe( adap, &addr_data, alc5628_attach);
}

static int alc5628_detach(struct i2c_client *c)
{
	i2c_detach_client(c);
	kfree(c);
	return 0;
}

static struct i2c_driver alc5628_i2c_driver	= {
	.name			= "ALC5628",
	.id				= I2C_DRIVERID_ALC5628,
	.flags			= I2C_DF_NOTIFY,
	.attach_adapter	= alc5628_probe,
	.detach_client	= alc5628_detach,
};

/* called by alc5628_init() */
int alc5628_i2c_init(void)
{
	return i2c_add_driver(&alc5628_i2c_driver);
}

