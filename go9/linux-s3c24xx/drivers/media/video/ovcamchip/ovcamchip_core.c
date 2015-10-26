/* Shared Code for OmniVision Camera Chip Drivers
 *
 * Copyright (c) 2003-2006 Mark McClelland <mark@ovcam.org>
 * http://ovcam.org/ov511/
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version. NO WARRANTY OF ANY KIND is expressed or implied.
 */

#include <linux/config.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <asm/hardware/clock.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 0)
#   include <linux/moduleparam.h>
#endif
#include <linux/slab.h>
#include <linux/delay.h>
#include "ovcamchip_priv.h"

#define DRIVER_VERSION "v2.32"
#define DRIVER_VERSION_CODE KERNEL_VERSION(2, 32, 0)

#define DRIVER_AUTHOR "Mark McClelland <mark@ovcam.org>"
#define DRIVER_DESC "OV camera chip I2C driver"

#ifdef OVCAMCHIP_DEBUG
int ovcamchip_debug = 0;
static int debug;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug,
  "Debug level: 0=none, 1=inits, 2=warning, 3=config, 4=functions, 5=all");
#endif

/* By default, let bridge driver tell us if chip is monochrome. mono=0
 * will ignore that and always treat chips as color. mono=1 will force
 * monochrome mode for all chips. */
static int mono = -1;
module_param(mono, int, 0);
MODULE_PARM_DESC(mono,
  "1=chips are monochrome (OVx1xx), 0=force color, -1=autodetect (default)");

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
#if defined(MODULE_LICENSE)	/* Introduced in ~2.4.10 */
MODULE_LICENSE("GPL");
#endif

/* Registers common to all chips, that are needed for detection */
#define GENERIC_REG_ID_HIGH       0x1C	/* manufacturer ID MSB */
#define GENERIC_REG_ID_LOW        0x1D	/* manufacturer ID LSB */
#define GENERIC_REG_COM_I         0x29	/* misc ID bits */

extern struct ovcamchip_ops ov6x20_ops;
extern struct ovcamchip_ops ov6x30_ops;
extern struct ovcamchip_ops ov7x10_ops;
extern struct ovcamchip_ops ov7x20_ops;
extern struct ovcamchip_ops ov76be_ops;
extern struct ovcamchip_ops ov9xx0_ops;

static char *chip_names[NUM_CC_TYPES] = {
	[CC_UNKNOWN]	= "Unknown chip",
	[CC_OV76BE]	= "OV76BE",
	[CC_OV7610]	= "OV7610",
	[CC_OV7620]	= "OV7620",
	[CC_OV7620AE]	= "OV7620AE",
	[CC_OV6620]	= "OV6620",
	[CC_OV6630]	= "OV6630",
	[CC_OV6630AE]	= "OV6630AE",
	[CC_OV6630AF]	= "OV6630AF",
	[CC_OV9655_REV4]= "OV9655r4",
	[CC_OV9655_REV5]= "OV9655r5"
};

/* Forward declarations */
static struct i2c_driver driver;
static struct i2c_client client_template;

/* ----------------------------------------------------------------------- */

int ov_write_regvals(struct i2c_client *c, struct ovcamchip_regvals *rvals)
{
	int rc;

	while (rvals->reg != 0xff) {
		rc = ov_write(c, rvals->reg, rvals->val);
		if (rc < 0)
			return rc;
		rvals++;
	}

	return 0;
}

/* Writes bits at positions specified by mask to an I2C reg. Bits that are in
 * the same position as 1's in "mask" are cleared and set to "value". Bits
 * that are in the same position as 0's in "mask" are preserved, regardless
 * of their respective state in "value".
 */
int ov_write_mask(struct i2c_client *c,
		  unsigned char reg,
		  unsigned char value,
		  unsigned char mask)
{
	int rc;
	unsigned char oldval, newval;

	if (mask == 0xff) {
		newval = value;
	} else {
		rc = ov_read(c, reg, &oldval);
		if (rc < 0)
			return rc;

		oldval &= (~mask);		/* Clear the masked bits */
		value &= mask;			/* Enforce mask on value */
		newval = oldval | value;	/* Set the desired bits */
	}

	return ov_write(c, reg, newval);
}

/* ----------------------------------------------------------------------- */
/* Reset the chip and ensure that I2C is synchronized. Returns <0 if failure.
 */
#include <asm/io.h>
#include <asm/arch/map.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-camif.h>
#include <asm/arch/regs-gpio.h>
#include <barcelona/gopins.h>
static int init_camchip(struct i2c_client *c)
{
	int i, success;
	unsigned char high, low;

	/* Reset the chip */
	ov_write(c, 0x12, 0x80);

	/* Wait for it to initialize */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 9)
	msleep(150);
#else
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(1 + 150 * HZ / 1000);
#endif

	for (i = 0, success = 0; i < I2C_DETECT_RETRIES && !success; i++) {
		if (ov_read(c, GENERIC_REG_ID_HIGH, &high) >= 0) {
			if (ov_read(c, GENERIC_REG_ID_LOW, &low) >= 0) {
				if (high == 0x7F && low == 0xA2) {
					success = 1;
					continue;
				}
			}

		}

		/* Reset the chip */
		ov_write(c, 0x12, 0x80);

		/* Wait for it to initialize */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 9)
		msleep(150);
#else
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(1 + 150 * HZ / 1000);
#endif

		/* Dummy read to sync I2C */
		ov_read(c, 0x00, &low);
	}

	if (!success)
		return -EIO;

	PDEBUG(1, "I2C synced in %d attempt%s", i, (i == 1 ? "" : "s"));

	return 0;
}

/* This detects the OV7610, OV7620, or OV76BE chip. */
static int ov7xx0_detect(struct i2c_client *c)
{
	struct ovcamchip *ov = i2c_get_clientdata(c);
	int rc;
	unsigned char val;

	PDEBUG(4, "");

	/* Detect chip (sub)type */
	rc = ov_read(c, GENERIC_REG_COM_I, &val);
	if (rc < 0) {
		err("Error detecting camera chip type");
		return rc;
	} 

	if ((val & 3) == 3) {
		info("Camera chip is an OV7610");
		ov->subtype = CC_OV7610;
	} else if ((val & 3) == 1) {
		rc = ov_read(c, 0x15, &val);
		if (rc < 0) {
			err("Error detecting camera chip type");
			return rc;
		} 

		if (val & 1) {
			info("Camera chip is an OV7620AE");
			/* OV7620 is a close enough match for now. There are
			 * some definite differences though, so this should be
			 * fixed */
			ov->subtype = CC_OV7620;
		} else {
			info("Camera chip is an OV76BE");
			ov->subtype = CC_OV76BE;
		}
	} else if ((val & 3) == 0) {
		info("Camera chip is an OV7620");
		ov->subtype = CC_OV7620;
	} else {
		err("Unknown camera chip version: %d", val & 3);
		return -1;
	}

	if (ov->subtype == CC_OV76BE)
		ov->sops = &ov76be_ops;
	else if (ov->subtype == CC_OV7620)
		ov->sops = &ov7x20_ops;
	else
		ov->sops = &ov7x10_ops;

	return 0;
}

/* This detects the OV6620, OV6630, OV6630AE, or OV6630AF chip. */
static int ov6xx0_detect(struct i2c_client *c)
{
	struct ovcamchip *ov = i2c_get_clientdata(c);
	int rc;
	unsigned char val;

	PDEBUG(4, "");

	/* Detect chip (sub)type */
	rc = ov_read(c, GENERIC_REG_COM_I, &val);
	if (rc < 0) {
		err("Error detecting camera chip type");
		return -1;
	}

	if ((val & 3) == 0) {
		ov->subtype = CC_OV6630;
		info("Camera chip is an OV6630");
	} else if ((val & 3) == 1) {
		ov->subtype = CC_OV6620;
		info("Camera chip is an OV6620");
	} else if ((val & 3) == 2) {
		ov->subtype = CC_OV6630;
		info("Camera chip is an OV6630AE");
	} else if ((val & 3) == 3) {
		ov->subtype = CC_OV6630;
		info("Camera chip is an OV6630AF");
	}

	if (ov->subtype == CC_OV6620)
		ov->sops = &ov6x20_ops;
	else
		ov->sops = &ov6x30_ops;

	return 0;
}

static struct clk *ov9xx0_clock_enable( struct i2c_client *c )
{
	struct clk	*clk;

	clk=clk_get( &c->dev, "cam" );
	if( IS_ERR(clk) )
		return NULL;

	if( clk_use( clk ) )
	{
		clk_put( clk );
		return NULL;
	}

	if( clk_enable( clk ) )
	{
		clk_unuse( clk );
		clk_put( clk );
		return NULL;
	}

	return clk;
}

static void ov9xx0_clock_disable( struct clk *clk )
{
	clk_disable( clk );
	clk_unuse( clk );
	clk_put( clk );
	return;
}

static int ov9xx0_detect( struct i2c_client *c )
{
	struct ovcamchip	*ov=i2c_get_clientdata( c );
	unsigned short		ProductID;
	unsigned char		val;
	int			retval;

	/* Detect subtype. For now only OV9550. */
	ov->subtype = CC_UNKNOWN;
	if( ov_read( c, 0x0A, &val ) < 0 )
	{
		err( "Error detecting 9xx0 subtype MSB." );
		return -1;
	}
	else ProductID=((unsigned short) val) << 8;

	if( ov_read( c, 0x0B, &val ) < 0 )
	{
		err( "Error detecting 9xx0 subtype LSB." );
		return -1;
	}
	else ProductID|=(unsigned short) val;

	switch( ProductID )
	{
		case 0x9657 :
			/* REV 5 chip. NOTE!! This chip has the reset line inverted when compared to REV4. */
			ov->subtype=CC_OV9655_REV5;
			break;

		case 0x9656 :
			/* REV 4 chip. */
			ov->subtype=CC_OV9655_REV4;
			break;

		default :
			/* Unknown chip. */
			err( "Unknown 9xx0 subtype found." );
			return -1;
	}

	ov->sops = &ov9xx0_ops;

	/* Do the init here, to ensure it doesn't need to be done when the camera is opened. */
	retval=ov->sops->init(c);

	/* Put it back to sleep. */
	ov->sops->command( c, OVCAMCHIP_CAM_SLEEP, NULL ); 
	return retval;
}

static int ovcamchip_detect(struct i2c_client *c)
{
	struct clk	*clk;

	/* Ideally we would just try a single register write and see if it NAKs.
	 * That isn't possible since the OV518 can't report I2C transaction
	 * failures. So, we have to try to initialize the chip (i.e. reset it
	 * and check the ID registers) to detect its presence. */

	/* Test for 7xx0 */
	PDEBUG(3, "Testing for 0V7xx0");
	c->addr = OV7xx0_SID;
	if (init_camchip(c) < 0) {
		/* Test for 6xx0 */
		PDEBUG(3, "Testing for 0V6xx0");
		c->addr = OV6xx0_SID;
		if (init_camchip(c) < 0) {
			PDEBUG( 3, "Testing for OV9xx0" );
			c->addr = OV_HIRES_SID;

			/* Ensure the clock is enabled. Otherwise it can't be detected. */
			clk=ov9xx0_clock_enable( c );
			if( clk == NULL )
			{
				err( "Failed to enable ovcamchip clock" );
				return -EIO;
			}
			else if( init_camchip(c) < 0 )
			{
				ov9xx0_clock_disable( clk );
	 			return -ENODEV;
			}
			else if( ov9xx0_detect(c) < 0 )
			{
				ov9xx0_clock_disable( clk );
				err( "Failed to init OV9xx0" );
				return -EIO;
			}
			else ov9xx0_clock_disable( clk );
		} else {
			if (ov6xx0_detect(c) < 0) {
				err("Failed to init OV6xx0");
 				return -EIO;
			}
		}
	} else {
		if (ov7xx0_detect(c) < 0) {
			err("Failed to init OV7xx0");
 			return -EIO;
		}
	}

	return 0;
}

/* ----------------------------------------------------------------------- */

#if defined(HAVE_V4L2)
/* ====== V4L2 queryctrl templates ======= */

#if 0
/* Placeholder for nonexistent controls */
static const struct v4l2_queryctrl ovcamchip_qct_none = {
	.name = "(no control)",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};
#endif

static const struct v4l2_queryctrl ovcamchip_qct_brightness = {
	.id   = V4L2_CID_BRIGHTNESS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Brightness",
	.step = 1,
};

static const struct v4l2_queryctrl ovcamchip_qct_contrast = {
	.id   = V4L2_CID_CONTRAST,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Contrast",
	.step = 1,
};

static const struct v4l2_queryctrl ovcamchip_qct_saturation = {
	.id   = V4L2_CID_SATURATION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Saturation",
	.step = 1,
};

static const struct v4l2_queryctrl ovcamchip_qct_red_balance = {
	.id   = V4L2_CID_RED_BALANCE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Red Balance",
	.step = 1,
};

static const struct v4l2_queryctrl ovcamchip_qct_blue_balance = {
	.id   = V4L2_CID_BLUE_BALANCE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Blue Balance",
	.step = 1,
};

static const struct v4l2_queryctrl ovcamchip_qct_agc = {
	.id   = V4L2_CID_AUTOGAIN,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "Auto Gain",
	.maximum = 1,
};

static const struct v4l2_queryctrl ovcamchip_qct_gain = {
	.id   = V4L2_CID_GAIN,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Gain",
	.step = 1,
};

static const struct v4l2_queryctrl ovcamchip_qct_exposure = {
	.id   = V4L2_CID_EXPOSURE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Exposure",
	.step = 1,
};

static const struct v4l2_queryctrl ovcamchip_qct_awb = {
	.id   = V4L2_CID_AUTO_WHITE_BALANCE,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "Auto White Balance",
	.maximum = 1,
};

/* Private controls */

static const struct v4l2_queryctrl ovcamchip_qct_aec = {
	.id   = OVCAMCHIP_V4L2_CID_AEC,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "Auto Exposure",
	.maximum = 1,
};

struct v4l2_queryctrl *ovcamchip_get_qc(struct ovcamchip *ov, __u32 id)
{
	switch (id) {
	case V4L2_CID_BRIGHTNESS:           return &ov->qc_brightness;
	case V4L2_CID_CONTRAST:             return &ov->qc_contrast;
	case V4L2_CID_SATURATION:           return &ov->qc_saturation;
	case V4L2_CID_RED_BALANCE:          return &ov->qc_red_balance;
	case V4L2_CID_BLUE_BALANCE:         return &ov->qc_blue_balance;
	case V4L2_CID_AUTOGAIN:             return &ov->qc_agc;
	case V4L2_CID_GAIN:                 return &ov->qc_gain;
	case V4L2_CID_EXPOSURE:             return &ov->qc_exposure;
	case V4L2_CID_AUTO_WHITE_BALANCE:   return &ov->qc_awb;
	case OVCAMCHIP_V4L2_CID_AEC:        return &ov->qc_aec;
	default:                            return NULL;
	}
}
#endif

/* Copy querycontrol templates into ovcamchip struct */
static inline void ovcamchip_init_qct(struct ovcamchip *ov)
{
#if defined(HAVE_V4L2)
	/* Set V4L2 controls to defaults */
	ov->qc_brightness   = ovcamchip_qct_brightness;
	ov->qc_contrast     = ovcamchip_qct_contrast;
	ov->qc_saturation   = ovcamchip_qct_saturation;
	ov->qc_red_balance  = ovcamchip_qct_red_balance;
	ov->qc_blue_balance = ovcamchip_qct_blue_balance;
	ov->qc_agc          = ovcamchip_qct_agc;
	ov->qc_gain         = ovcamchip_qct_gain;
	ov->qc_exposure     = ovcamchip_qct_exposure;
	ov->qc_awb          = ovcamchip_qct_awb;
	ov->qc_aec          = ovcamchip_qct_aec;
#endif
}

static int ovcamchip_attach(struct i2c_adapter *adap)
{
	int rc = 0;
	struct ovcamchip *ov;
	struct i2c_client *c;

	/* I2C is not a PnP bus, so we can never be certain that we're talking
	 * to the right chip. To prevent damage to EEPROMS and such, only
	 * attach to adapters that are known to contain OV camera chips. */

	switch (adap->id) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 14)
	case (I2C_ALGO_SMBUS | I2C_HW_SMBUS_OV511):
	case (I2C_ALGO_SMBUS | I2C_HW_SMBUS_OV518):
	case (I2C_ALGO_SMBUS | I2C_HW_SMBUS_OVFX2):
	case (I2C_ALGO_SMBUS | I2C_HW_SMBUS_W9968CF):
	case (I2C_HW_SMBUS_OV9655):
#else
	case (I2C_HW_SMBUS_OV511):
	case (I2C_HW_SMBUS_OV518):
	case (I2C_HW_SMBUS_OVFX2):
	case (I2C_HW_SMBUS_W9968CF):
	case (I2C_HW_SMBUS_OV9655):
#endif
		PDEBUG(1, "Adapter ID 0x%06x accepted", adap->id);
		break;
	default:
		PDEBUG(1, "Adapter ID 0x%06x rejected", adap->id);
		return -ENODEV;  // FIXME: Should we be returning zero?
	}

	c = kmalloc(sizeof *c, GFP_KERNEL);
	if (!c) {
		rc = -ENOMEM;
		goto no_client;
	}
	memcpy(c, &client_template, sizeof *c);
	c->adapter = adap;
	strcpy(c->name, "OV????");

	ov = kmalloc(sizeof *ov, GFP_KERNEL);
	if (!ov) {
		rc = -ENOMEM;
		goto no_ov;
	}
	memset(ov, 0, sizeof *ov);
	i2c_set_clientdata(c, ov);
	ovcamchip_init_qct(ov);

	rc = ovcamchip_detect(c);
	if (rc < 0)
		goto error;

	strcpy(c->name, chip_names[ov->subtype]);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
	MOD_INC_USE_COUNT;
#endif

	PDEBUG(1, "Camera chip detection complete");

	i2c_attach_client(c);

	return rc;
error:
	kfree(ov);
no_ov:
	kfree(c);
no_client:
	PDEBUG(1, "returning %d", rc);
	return rc;
}

static int ovcamchip_detach(struct i2c_client *c)
{
	struct ovcamchip *ov = i2c_get_clientdata(c);
	int rc;

	rc = ov->sops->free(c);
	if (rc < 0)
		return rc;

	i2c_detach_client(c);

	kfree(ov);
	kfree(c);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
	MOD_DEC_USE_COUNT;
#endif
	return 0;
}

static int ovcamchip_command(struct i2c_client *c, unsigned int cmd, void *arg)
{
	struct ovcamchip *ov = i2c_get_clientdata(c);

	if( !ov->initialized )
	{
		switch( cmd )
		{
			case OVCAMCHIP_CMD_Q_SUBTYPE :
			case OVCAMCHIP_CMD_INITIALIZE:
			case OVCAMCHIP_CAM_SLEEP :
			case OVCAMCHIP_CAM_WAKE:
				break;

			default:
				err("ERROR: Camera chip is not initialized yet!");
				return -EPERM;
		}
	}

	switch (cmd) {
	case OVCAMCHIP_CMD_Q_SUBTYPE:
	{
		*(int *)arg = ov->subtype;
		return 0;
	}
	case OVCAMCHIP_CMD_INITIALIZE:
	{
		int rc;

		/* First ensure that the chip is NOT sleeping. */
		if( !ov->initialized )
			ovcamchip_command( c, OVCAMCHIP_CAM_WAKE, NULL );

		if (mono == -1)
			ov->mono = *(int *)arg;
		else
			ov->mono = mono;

		if (ov->mono) {
			if (ov->subtype != CC_OV7620)
				warn("Warning: chip doesn't do monochrome");
			else
				info("Initializing chip as monochrome");
		}

		rc = ov->sops->init(c);
		if (rc < 0)
			return rc;

		return 0;		
	}
#if defined(HAVE_V4L2)
	case VIDIOC_QUERYCTRL:
	{
		struct v4l2_queryctrl *c = arg;
		struct v4l2_queryctrl *ret;

		if ((c->id < V4L2_CID_BASE ||
		     c->id >= V4L2_CID_LASTP1) &&
		    (c->id < V4L2_CID_PRIVATE_BASE ||
		     c->id >= OVCAMCHIP_V4L2_CID_LASTP1))
			return -EINVAL;

		ret = ovcamchip_get_qc(ov, c->id);
		if (ret == NULL)
			c->flags = V4L2_CTRL_FLAG_DISABLED;
		else
			memcpy(c, ret, sizeof(*c));

		return 0;
	}
#endif
	default:
		return ov->sops->command(c, cmd, arg);
	}
}

/* ----------------------------------------------------------------------- */

static struct i2c_driver driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
#  if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 0)
	.owner =		THIS_MODULE,
#  endif
	.name =			"ovcamchip",
#else
	.driver = {
		.name = "ovcamchip",
	},
#endif
	.id =			I2C_DRIVERID_OVCAMCHIP,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 7)
	.class =		I2C_CLASS_CAM_DIGITAL,
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
	.flags =		I2C_DF_NOTIFY,
#endif
	.attach_adapter =	ovcamchip_attach,
	.detach_client =	ovcamchip_detach,
	.command =		ovcamchip_command,
};

static struct i2c_client client_template = {
	.name =		"(unset)",
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 12)
	.id =		-1,
#endif
	.flags =	I2C_CLIENT_ALLOW_USE,
	.usage_count =	0,
	.driver =	&driver,
};

static int ovcamchip_probe(struct device *dev)
{
#ifdef OVCAMCHIP_DEBUG
	ovcamchip_debug = debug;
#endif

	info(DRIVER_VERSION " : " DRIVER_DESC);
	return i2c_add_driver(&driver);
}

static int ovcamchip_remove(struct device *dev)
{
	i2c_del_driver(&driver);
	return 0;
}

static void ovcamchip_shutdown(struct device *dev)
{
	ovcamchip_remove( dev );
	return;
}

struct device_driver ovcamchip_driver =
{
        .owner          = THIS_MODULE,
        .name           = "s3c2412-i2c-camif",
        .bus            = &platform_bus_type,
        .probe          = ovcamchip_probe,
        .remove         = ovcamchip_remove,
        .shutdown       = ovcamchip_shutdown,
};

static int __init ovcamchip_init(void)
{
	int	ret;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
	EXPORT_NO_SYMBOLS;
#endif

#ifdef OVCAMCHIP_DEBUG
	ovcamchip_debug = debug;
#endif
	ret = driver_register(&ovcamchip_driver);
	if (ret) {
		printk(KERN_ERR "Unable to register ovcamchip driver (%x)\n", ret);
		return ret;
	}
	return 0;
}

static void __exit ovcamchip_exit(void)
{
	driver_unregister(&ovcamchip_driver);
}

module_init(ovcamchip_init);
module_exit(ovcamchip_exit);
