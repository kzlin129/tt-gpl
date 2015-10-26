/* drivers/char/mcp23008/mcp23008.c
 *
 * Control driver for Microchip MCP23008 chip.
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Authors: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/mcp23008.h>
#include <barcelona/gopins.h>
#include <linux/delay.h>
#include <asm-arm/arch-s3c2410/hardware.h>

#define BARCELONA_DEBUG	__FILE__
#define PFX "MCP23008: "
#define PK_DBG PK_DBG_FUNC
#include <barcelona/debug.h>

/* Forward declarations */
static struct i2c_client client_template;

static unsigned short normal_i2c[] = { MCP23008_I2C_SLAVE_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static int mcp23008_attach(struct i2c_adapter* adapter);
static int mcp23008_detach(struct i2c_client* client);
static int mcp23008_i2c_probe(struct device *dev);
static int mcp23008_i2c_remove(struct device *dev);

/* If the dock device is present, it'll unregister the platform devices before suspending, */
/* causing this driver to have problems upon resume. Therefore, let the unregister handle  */
/* the resume action. */
#if !defined(CONFIG_BARCELONA_DOCK) && defined(CONFIG_PM)
static int mcp23008_i2c_suspend(struct device *dev, pm_message_t state, u32 level);
static int mcp23008_i2c_resume(struct device *dev, u32 level);
static int mcp23008_powersuspend(struct i2c_client *client);
static int mcp23008_powerresume(struct i2c_client *client);
#else
#define mcp23008_i2c_suspend	(NULL)
#define mcp23008_i2c_resume 	(NULL)
#endif /* !defined(CONFIG_BARCELONA_DOCK) && defined(CONFIG_PM) */

static struct i2c_driver driver = {
	.id             = I2C_DRIVERID_MCP23008,
	.owner          = THIS_MODULE,
	.name           = "mcp23008_i2c",
	.flags          = I2C_DF_NOTIFY,
	.attach_adapter = mcp23008_attach,
	.detach_client  = mcp23008_detach,
	.driver = {
		.name   = "mcp23008_i2c",
	},
};

static struct device_driver mcp23008_driver = {
	.name    = MCP23008_DEVNAME,
	.probe   = mcp23008_i2c_probe,
	.suspend = mcp23008_i2c_suspend,
	.remove  = mcp23008_i2c_remove,
	.resume  = mcp23008_i2c_resume,
	.owner   = THIS_MODULE,
	.bus     = &platform_bus_type,
};

static struct i2c_client client_template = {
	.name        = "mcp23008_i2c_client",
	.flags       = I2C_CLIENT_ALLOW_USE,
	.usage_count = 0,
	.driver      = &driver,
	.addr        = MCP23008_I2C_SLAVE_ADDR,
};

static int mcp23008_startup(struct i2c_client *c)
{
	unsigned char init_array[][2] = {
		{ 0x00, 0x7B }, /* IO7 and IO2=output, rest is input.*/
		{ 0x01, 0x00 }, /* IN7:0 are all non-inverted. */
		{ 0x02, 0x00 }, /* GPINT7:0 are all disabled (no interrupt-on-change event). */
		{ 0x03, 0x00 }, /* Default value 7:0 are all 0. */
		{ 0x04, 0x00 }, /* INTCON7:0 - Pin value is compared to previous value for all bits. */
		{ 0x05, 0x00 }, /* SEQOP=ena, DISSLW=ena, HAEN=Dis, ODR=Active driver output, INTPOL=Active Low. */
		{ 0x06, 0x5B }, /* GPPU7,5,2=pullup disabled, rest pullup enabled. */
		{ 0x07, 0x00 }, /* INTF7:0 = all interrupt not pending (read only) */
		{ 0x08, 0x00 }, /* INTCAP7:0 = all pins logic-low (read only) */
		{ 0x09, 0x00 }, /* GPIO7:0 - all logic low. */
		{ 0x0A, 0x00 }, /* OL7:0 - all logic low.*/
	};
	int count = 0;
	int rc = 0;

	/* Write the default values. */
	for (count = 0; count < ARRAY_SIZE(init_array); count++) {
		i2c_smbus_write_byte_data(c, init_array[count][0], init_array[count][1]);
		if (rc < 0)
			return rc;
	}

	/* Since the USB dock for valencia/murcia pulls down the SDA pin to ground, the data received will */
	/* be all 0. This means that whatever we wrote before can't match to what we read back now if the  */
	/* usb dock is connected. */
	for (count = 0; count < ARRAY_SIZE(init_array); count++) {
		rc = i2c_smbus_read_byte_data(c, init_array[count][0]);
		if (rc < 0)
			return rc;
		if (rc != 0)
			break;
	}

	if (count == ARRAY_SIZE(init_array))
		return -1;

	return 0;
}

static int mcp23008_SetIOExpConfig(struct i2c_client *c, unsigned char cfg_flags)
{
	return i2c_smbus_write_byte_data(c, 0x05, cfg_flags);
}

static int mcp23008_GetIntFlagPinMask(struct i2c_client *c)
{
	return i2c_smbus_read_byte_data(c, 0x07);
}

static int mcp23008_GetPinVal(struct i2c_client *c, int pin)
{
	int rc = 0;

	if ((pin < 0) || (pin > 7))
		return -1;

	rc=i2c_smbus_read_byte_data(c, 0x09);
	if (rc < 0)
		return rc;

	return (rc & (1 << pin) ? 1 : 0);
}

static int mcp23008_SetPinVal(struct i2c_client *c, int pin, int val)
{
	int rc = 0;

	rc=i2c_smbus_read_byte_data(c, 0x09);
	if (rc < 0)
		return rc;
	if (val)
		rc |= (1 << pin);
	else
		rc &= ~(1 << pin);

	return i2c_smbus_write_byte_data(c, 0x09, rc);
}

static int mcp23008_GetPinConfig(struct i2c_client *c, int pin)
{
	int index = 0;
	unsigned char pincfg_array[][2] = {
		{ 0x00, 0x00 },
		{ 0x01, 0x00 },
		{ 0x02, 0x00 },
		{ 0x03, 0x00 },
		{ 0x04, 0x00 },
		{ 0x06, 0x00 },
	};
	int retval = 0;
	int pin_mask = 1 << pin;

	/* Ensure no more than 8 bits in the mask. We don't have any more pins. */
	if ((pin < 0) || (pin > 7))
		return -1;

	/* Get the current config of all pins, then mask in the new config and write back. */
	for (index = 0; index < ARRAY_SIZE(pincfg_array); index++) {
		/* Get the value. */
		retval = i2c_smbus_read_byte_data(c, pincfg_array[index][0]);
		if (retval < 0)
			return retval;
		pincfg_array[index][1] = retval;
	}

	/* Check the value. */
	retval = 0;
	if (pincfg_array[0][1] & pin_mask)
		retval |= MCP23008_IOPIN_INPUT;
	else
		retval |= MCP23008_IOPIN_OUTPUT;

	if (pincfg_array[1][1] & pin_mask)
		retval |= MCP23008_IOPIN_INVERTED;
	else
		retval |= MCP23008_IOPIN_NONINVERTED;

	if (pincfg_array[2][1] & pin_mask)
		retval |= MCP23008_IOPIN_INTERRUPT;
	else
		retval |= MCP23008_IOPIN_NOINTERRUPT;

	if (pincfg_array[3][1] & pin_mask)
		retval |= MCP23008_IOPIN_DEFVAL_HIGH;
	else
		retval |= MCP23008_IOPIN_DEFVAL_LOW;

	if (pincfg_array[4][1] & pin_mask)
		retval |= MCP23008_IOPIN_IRQ_CMPDEFVAL;
	else
		retval |= MCP23008_IOPIN_IRQ_CMPPREVVAL;

	if (pincfg_array[5][1] & pin_mask)
		retval |= MCP23008_IOPIN_INTPULLUP;
	else
		retval |= MCP23008_IOPIN_NOINTPULLUP;

	/* Done. */
	return retval;
}

static int mcp23008_SetPinConfig(struct i2c_client *c, int pin_mask, int cfg_mask)
{
	int index = 0;
	unsigned char pincfg_array[][2] = {
		{ 0x00, 0x00 },
		{ 0x01, 0x00 },
		{ 0x02, 0x00 },
		{ 0x03, 0x00 },
		{ 0x04, 0x00 },
		{ 0x06, 0x00 },
	};
	int rc = 0;

	/* Ensure no more than 8 bits in the mask. We don't have any more pins. */
	pin_mask &= 0xFF;

	/* Get the current config of all pins, then mask in the new config and write back. */
	for (index = 0; index < ARRAY_SIZE(pincfg_array); index++) {
		/* Get the value. */
		rc = i2c_smbus_read_byte_data(c, pincfg_array[index][0]);
		if (rc < 0)
			return rc;
		pincfg_array[index][1] = rc;
	}

	/* Check the value. */
	if (cfg_mask & MCP23008_IOPIN_OUTPUT)
		pincfg_array[0][1] &= ~pin_mask;
	else
		pincfg_array[0][1] |= pin_mask;

	if (cfg_mask & MCP23008_IOPIN_NONINVERTED)
		pincfg_array[1][1] &= ~pin_mask;
	else
		pincfg_array[1][1] |= pin_mask;

	if (cfg_mask & MCP23008_IOPIN_INTERRUPT)
		pincfg_array[2][1] |= pin_mask;
	else
		pincfg_array[2][1] &= ~pin_mask;

	if (cfg_mask & MCP23008_IOPIN_DEFVAL_HIGH)
		pincfg_array[3][1] |= pin_mask;
	else
		pincfg_array[3][1] &= ~pin_mask;

	if (cfg_mask & MCP23008_IOPIN_IRQ_CMPDEFVAL)
		pincfg_array[4][1] |= pin_mask;
	else
		pincfg_array[4][1] &= ~pin_mask;

	if (cfg_mask & MCP23008_IOPIN_INTPULLUP)
		pincfg_array[5][1] |= pin_mask;
	else
		pincfg_array[5][1] &= ~pin_mask;

	/* Write it all back. */
	for (index = 0; index < ARRAY_SIZE(pincfg_array); index++) {
		/* Get the value. */
		rc = i2c_smbus_write_byte_data(c, pincfg_array[index][0], pincfg_array[index][1]);
		if (rc < 0)
			return rc;
	}

	/* Done. */
	return rc;
}

struct ioexp_handler mcp23008_handle = {
	.is_valid            = ATOMIC_INIT(-1),
	.use_count           = ATOMIC_INIT(0),
	.client              = NULL,
	.set_ioexp_config    = mcp23008_SetIOExpConfig,
	.get_intflag_pinmask = mcp23008_GetIntFlagPinMask,
	.get_pinval          = mcp23008_GetPinVal,
	.set_pinval          = mcp23008_SetPinVal,
	.set_pincfg          = mcp23008_SetPinConfig,
	.get_pincfg          = mcp23008_GetPinConfig,
};

int mcp23008_handle_state = 0;
EXPORT_SYMBOL(mcp23008_handle);

#if !defined(CONFIG_BARCELONA_DOCK) && defined(CONFIG_PM)
/* The suspend array contains default working values to ensure the device can work if someone uses an ioctl to enable it. */
static unsigned char mcp23008_suspend_array[][2]= {
	{ 0x00, 0x00 }, { 0x01, 0x00 }, { 0x02, 0x00 }, { 0x03, 0x00 },
	{ 0x04, 0x00 }, { 0x05, 0x00 }, { 0x06, 0x00 }, { 0x07, 0x00 },
	{ 0x08, 0x00 }, { 0x09, 0x00 }, { 0x0A, 0x00 }, { 0xFF, 0x00 },
};

static int mcp23008_powersuspend(struct i2c_client *client)
{
	int rc = 0;
	int index = 0;

	for (index = 0; mcp23008_suspend_array[index][0] != 0xFF; index++) {
		rc = i2c_smbus_read_byte_data(client, mcp23008_suspend_array[index][0]);
		if (rc < 0)
			return rc;
		mcp23008_suspend_array[index][1] = rc;
	}

	mcp23008_handle_state = atomic_read(&mcp23008_handle.is_valid);
	atomic_set(&mcp23008_handle.is_valid, 0);
	return rc;
}

static int mcp23008_powerresume(struct i2c_client *client)
{
	int rc = 0;
	int index = 0;

	/* Restore settings. */
	for (index = 0; mcp23008_suspend_array[index][0] != 0xFF; index++) {
		rc = i2c_smbus_write_byte_data(client, mcp23008_suspend_array[index][0], mcp23008_suspend_array[index][1]);
		if (rc < 0)
			return rc;
	}
	atomic_set(&mcp23008_handle.is_valid, mcp23008_handle_state);
	return rc;
}
#endif /* !defined(CONFIG_BARCELONA_DOCK) && defined(CONFIG_PM) */

static int mcp23008_i2c_remove(struct device *dev)
{
	return i2c_del_driver(&driver);
}

static int mcp23008_i2c_probe(struct device *dev)
{
	return i2c_add_driver(&driver);
}

#if !defined(CONFIG_BARCELONA_DOCK) && defined(CONFIG_PM)
static int mcp23008_i2c_suspend(struct device *dev, pm_message_t state, u32 level)
{
	struct i2c_client *c = NULL;
	int rc = 0;

	c=i2c_get_client(I2C_DRIVERID_MCP23008, 0, NULL);
	if (c != NULL) {
		if (level == SUSPEND_POWER_DOWN) {
			rc = mcp23008_powersuspend(c);
			if (rc)
				printk(KERN_ERR PFX"Error while doing powersuspend: rc = %d\n", rc);
		}
	}

	/* Never return an error here. It will block the kernel from going into suspend further
	 * and the IO expander is not important enough to block that!
	 */
	return 0;
}

static int mcp23008_i2c_resume(struct device *dev, u32 level)
{
	struct i2c_client *c = NULL;
	int rc = 0;

	c = i2c_get_client(I2C_DRIVERID_MCP23008, 0, NULL);
	if (c != NULL) {
		if (level == RESUME_POWER_ON)
			rc = mcp23008_powerresume(c);
	}

	return (rc < 0 ? rc : 0);
}
#endif /* !defined(CONFIG_BARCELONA_DOCK) && defined(CONFIG_PM) */

static int mcp23008_detect_client(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *c;
	int rc = 0;

	c = kmalloc(sizeof *c, GFP_KERNEL);
	if (!c) {
		rc = -ENOMEM;
		goto no_client;
	}
	memcpy(c, &client_template, sizeof *c);
	c->adapter = adapter;
	strcpy(c->name, MCP23008_DEVNAME);

	i2c_set_clientdata(c, NULL);

	i2c_attach_client(c);

	/* Now setup our character device to the fm */
	rc=mcp23008_startup(c);
	if (rc)
		goto no_dev;

	/* Set handle as valid. */
	mcp23008_handle.client=c;
	atomic_set(&mcp23008_handle.is_valid, 1);
	printk("Microchip MCP23008 IO Expander v1.0 driver loaded\n");
	return 0;

no_dev:
	i2c_detach_client(c);
	kfree(c);
no_client:
	PK_DBG("%s: ERROR %d!\n", __func__, rc);

	return rc;

}

/* ----------------------------------------------------------------------- */

static int mcp23008_attach(struct i2c_adapter *adap)
{
	int ret = 0;

	ret = i2c_probe(adap, &addr_data, mcp23008_detect_client);
	udelay(100);

	return ret;
}

static int mcp23008_detach(struct i2c_client *c)
{
	/* Set handle as valid. */
	if (atomic_read(&mcp23008_handle.is_valid) > 0) {
		printk("Microchip MCP23008 IO Expander v1.0 driver unloaded\n");

		atomic_set(&(mcp23008_handle.is_valid), 0);

		/* Wait for use count to go 0. */
		while (atomic_read(&(mcp23008_handle.use_count)))
			msleep(10);

		mcp23008_handle.client=NULL;
		i2c_detach_client(c);

		kfree(c);
	}
	return 0;
}


/* ----------------------------------------------------------------------- */

static int __init mcp23008_init(void)
{
	return driver_register(&mcp23008_driver);
}

static void __exit mcp23008_exit(void)
{
	driver_unregister(&mcp23008_driver);
}

MODULE_AUTHOR("Rogier Stam <rogier.stam@tomtom.com>");
MODULE_DESCRIPTION("Driver for I2C connected Microchip MCP23008 IO Expander");

module_init(mcp23008_init);
module_exit(mcp23008_exit);
