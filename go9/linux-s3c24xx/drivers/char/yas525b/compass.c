/*
 * Driver for Yamaha YAS525B Magnetic Field Sensor.
 *
 *              This program is free software; you can redistribute it and/or
 *              modify it under the terms of the GNU General Public License
 *              as published by the Free Software Foundation; either version
 *              2 of the License, or (at your option) any later version.
 *
 * Author:      Ithamar R. Adema, <ithamar.adema@tomtom.com>
 *
 * NOTE: - i2c_transfer should never be used with the compass device, as it 
 *         breaks on 'continued' transfers  (i.e. multiple transfers without
 *         stop bit inbetween), or so say the specs.
 * 	 - This driver sets up both a normal i2c_driver, as well as a cdev 
 * 	   once the compass is detected.
 */

#include <linux/config.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#include <linux/yas525b.h>
#include "compass.h"

#define MAXTRIES	100

/* Slowness of the compass device ;) */
#define DELAY_COIL	1	/*  1.0ms */
#define DELAY_XYROUGH	2	/*  1.5ms */
#define DELAY_TEMP	5	/*  4.5ms */
#define DELAY_MEASURE	10	/* 10.0ms */

/* Specs say the compass is fixed on address 0x2e.... */
static unsigned short
normal_i2c[] = { 0x2e, 0x2f, I2C_CLIENT_END };

/* Magic definition of all other i2c client module variables and things */
I2C_CLIENT_INSMOD;

static int compass_attach_adapter(struct i2c_adapter* adapter);
static int compass_detach_client(struct i2c_client* client);

static struct i2c_driver compass_i2c_driver = {
	.owner          = THIS_MODULE,
	.name           = "yas525b",
	.flags          = I2C_DF_NOTIFY,
	.attach_adapter = compass_attach_adapter,
	.detach_client  = compass_detach_client,
	.command        = NULL
};

/* Write 6 bits of data to a specific write-only register of the compass */
static inline int
compass_write(struct compass_data* compass, int reg, int val)
{
	int rc;
	
	/* Combine register address and value to the actual byte to output */
	unsigned char out = (reg << YAS525B_ADDR_SHIFT) + (val & ~(YAS525B_ADDR_MASK));

	/* Output to device */
	rc = i2c_master_send(compass->client, &out, sizeof(out));
	if (rc != sizeof(out)) {
		dev_err(&compass->client->dev, "Error writing %x to register %x\n", val, reg);
		return -EIO;
	}

	return 0;
}

static inline u32
compass_readl(struct compass_data* compass)
{
	u8 data[4];
	int rc;

	/* Try and read the measurement results */
	rc = i2c_master_recv(compass->client, (char*)&data, sizeof(data));
	if (rc != sizeof(data)) {
		dev_err(&compass->client->dev, "Error reading in %s (received %x bytes)\n", __FUNCTION__, rc);
		return -EIO;
	}
	
	return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
}

static inline u16
compass_readw(struct compass_data* compass)
{
	u16 word = 0;
	int rc, try = 0;
	u8 data[2];

	do
	{
                /* Try and read the measurement results */
                rc = i2c_master_recv(compass->client, data, sizeof(data));
                if (rc != sizeof(data)) {
			dev_err(&compass->client->dev, "Error reading in %s (received %x bytes)\n", __FUNCTION__, rc);
			return -EIO;
		}
		
		word = (data[0] << 8) | data[1];
	} while((word & MEASUREMENT_BUSY) && try++ < MAXTRIES);

	if (word & MEASUREMENT_BUSY) {
		dev_err(&compass->client->dev, "Error reading in %s (device still busy adter %d retries!)\n", __FUNCTION__, try);
		word = 0xFFFF;
	} else {
		word = (word & MEASUREMENT_MASK) >> MEASUREMENT_SHIFT;
	}
	
	return word;
}

static inline int
compass_read_measurement(struct compass_data* compass, int kind)
{
	compass_write(compass, YAS525B_CMDR, kind);
	msleep(DELAY_MEASURE);
	return compass_readw(compass);
}

static inline int
compass_init_coils(struct compass_data* compass)
{
	int ci;
	
	compass_write(compass, YAS525B_TESTR, YAS525B_TESTR_DISABLE);

	for (ci=0; ci < 7; ci++) {
		compass_write(compass, YAS525B_CONFR, YAS525B_CONFR_COILE_ENABLE | ci);
		msleep(DELAY_COIL);
		compass_write(compass, YAS525B_CONFR, ci +1);
	}

	compass_write(compass, YAS525B_CONFR, YAS525B_CONFR_COILE_ENABLE | 7);
	msleep(DELAY_COIL);
        compass_write(compass, YAS525B_CONFR, 0);

	return 0;
}

static inline int
compass_get_rough_offset(struct compass_data* compass, u16* x, u16* y)
{
	compass_write(compass, YAS525B_CMDR, YAS525B_CMDR_CMD_XROUGH);
	msleep(DELAY_XYROUGH);
	compass->xr = *x = compass_readw(compass);

	compass_write(compass, YAS525B_CMDR, YAS525B_CMDR_CMD_YROUGH);
	msleep(DELAY_XYROUGH);
	compass->yr = *y = compass_readw(compass);

	return 0;
}

static inline int
compass_set_rough_offset(struct compass_data* compass, u16 x, u16 y)
{
	x &= YAS525B_OFFSETR_OFFSET_MASK;
	y &= YAS525B_OFFSETR_OFFSET_MASK;
	
	compass_write(compass, YAS525B_OFFSETR, YAS525B_OFFSETR_SELX | x);
	msleep(DELAY_MEASURE);
	compass_write(compass, YAS525B_OFFSETR, YAS525B_OFFSETR_SELY | y);

	return 0;
}

/* Returns CALibration settings from compass */
static inline u32
compass_read_calibration(struct compass_data* compass)
{
	u32 res;

	/* Enable CAL Register access */
	compass_write(compass, YAS525B_CONFR, YAS525B_CONFR_CAL_ENABLE);
	
	/* First read of CAL register returns dummy data (4.2.4) */
	compass_readl(compass);
        
	/* This read should return the actual calibration (CAL register) data */
	res = compass_readl(compass);

	/* Disable CAL Register access */
	compass_write(compass, YAS525B_CONFR, 0);

	return res;
}

static int
compass_open(struct inode* nodep, struct file* filep)
{
	struct compass_data* compass = container_of(nodep->i_cdev, struct compass_data, cdev);

	filep->private_data = compass; /* for easy reference */

	compass_init_coils(compass);
	
	compass_write(compass, YAS525B_TESTR, YAS525B_TESTR_DISABLE);

	/* Read rough offset for both X and Y */
	compass->xr = compass_read_measurement(compass, YAS525B_CMDR_CMD_XROUGH);
	compass->yr = compass_read_measurement(compass, YAS525B_CMDR_CMD_YROUGH);

	/* Write back the results of the rough offset measurements */
	compass_write(compass, YAS525B_OFFSETR, YAS525B_OFFSETR_SELX | ((compass->xr-5) & YAS525B_OFFSETR_OFFSET_MASK));
	compass_write(compass, YAS525B_OFFSETR, YAS525B_OFFSETR_SELY | ((compass->yr-5) & YAS525B_OFFSETR_OFFSET_MASK));
	
	/* Read calibration data */
	compass->caldata = compass_read_calibration(compass);

	dev_dbg(&compass->client->dev, "r(%d,%d), cal=%x\n", compass->xr, compass->yr, compass->caldata);

	return (compass->xr < 0 || compass->yr < 0 || compass->caldata < 0) ? -EIO : 0;
}

static int
compass_release(struct inode* nodep, struct file* filep)
{
	struct compass_data* compass = (struct compass_data*) filep->private_data;
	if (compass) {
		filep->private_data = NULL;
	}
	
	return 0;
}

static int 
compass_ioctl(struct inode* nodep, struct file* filep, unsigned int cmd, unsigned long arg)
{
        struct compass_data* compass = (struct compass_data*) filep->private_data;

        if (_IOC_TYPE(cmd) != YAS525B_DRIVER_MAGIC)
		return -ENOTTY;

        if (_IOC_DIR(cmd) & _IOC_WRITE && !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd))) 
		return -EFAULT;

        switch(cmd) {
		case YAS525B_COILINIT:
			compass_init_coils(compass);
			return 0;

		case YAS525B_GETROUGHOFFSET:
			{
				u16 x,y;
				u32 rough;
				compass_get_rough_offset(compass,&x,&y);
				rough = x << 16 | y;
				copy_to_user((void*)arg, &rough, sizeof(rough));
			}
			return 0;

		case YAS525B_SETROUGHOFFSET:
			{
				u32 rough;
				copy_from_user(&rough, (void*)arg, sizeof(rough));
				compass_set_rough_offset(compass,rough >> 16, rough & 0xffff);
			}
			return 0;

		case YAS525B_GETCALIBDATA:
			{
				u32 calib = compass_read_calibration(compass);
				copy_to_user((void*)arg, &calib, sizeof(calib));
			}
			return 0;
	}

	return -ENOTTY;
}

static ssize_t
compass_read(struct file* filep, char __user * bufp, size_t bufsz, loff_t* loc)
{
	struct compass_data* compass = (struct compass_data*) filep->private_data;
	s16 measurements[3]; /* X,Y,temperature */
	int mdone = 0;

	if (compass) {
		/* Just read X/Y/temp normal measurement, and return the values in the passed buffer.
		 * take into account that the buffer size might specify only reading partial data, and if so,
		 * only request the parts that 'fit' into the passed buffer (saving time). 
		 */

		if (bufsz < sizeof(u16)) {
			/* Not even enough space to read X measurement, bail out */
			return -EINVAL;
		}

		/* Read X measurement */
		measurements[mdone] = compass_read_measurement(compass, YAS525B_CMDR_CMD_XNORM);
		if (measurements[mdone++] < 0) {
			dev_err(&compass->client->dev, "Error on reading X measurements!\n");
			return -EIO;
		}

		if (bufsz >= sizeof(u16)*2) {
			measurements[mdone] = compass_read_measurement(compass, YAS525B_CMDR_CMD_YNORM);
			if (measurements[mdone++] < 0) {
				dev_err(&compass->client->dev, "Error on reading Y measurements!\n");
				return -EIO;
			}
		}

		if (bufsz >= sizeof(u16)*3) {
			measurements[mdone] = compass_read_measurement(compass, YAS525B_CMDR_CMD_TEMP);
			if (measurements[mdone++] < 0) {
				dev_err(&compass->client->dev, "Error on reading Temperature measurements!\n");
				return -EIO;
			}
		}

		/* By now, we know how much has been measured, time to fill the output buffer, and tell user what we did */
		copy_to_user(bufp, measurements, mdone*sizeof(u16));
		return mdone*sizeof(u16);
	}

	return -ENODEV;
}

struct file_operations compass_fops = {
	.owner = THIS_MODULE,
	.open = compass_open,
	.release = compass_release,
	.read = compass_read,
	.ioctl = compass_ioctl
};

static int
compass_detect_client(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client* new_client;
	struct compass_data* data;
	int devno, err = 0;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_WRITE_BYTE))
		goto ERROR0;

	/* OK. For now, we presume we have a valid client. We now create the
	client structure, even though we cannot fill it completely yet.
	But it allows us to access several i2c functions safely */

	if (!(new_client=kcalloc(1,sizeof(struct i2c_client)+sizeof(struct compass_data),GFP_KERNEL))) {
		err = -ENOMEM;
		goto ERROR0;
	}

	/* This is tricky, but it will set the data to the right value. */
	data = (struct compass_data*)(new_client+1);
	i2c_set_clientdata(new_client, data);
	data->client = new_client;
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &compass_i2c_driver;
	new_client->flags = 0;

	/* Now, we do the remaining detection. If no `force' parameter is used. */

	/* First, the generic detection (if any), that is skipped if any force
	   parameter was used. */
	if (kind < 0) {
		/* Check if we can write to the compass, using a valid register. If this fails, no device is attached or i2c module is loaded
		 * while pins are not connected to a 'real' i2c bus. */
		u8 cmd = (YAS525B_TESTR << YAS525B_ADDR_SHIFT) + YAS525B_TESTR_DISABLE; 
		if (i2c_master_send(new_client, &cmd, sizeof(cmd)) <= 0)
			goto ERROR1;
	}

	/* Fill in the remaining client fields. */
	strcpy(new_client->name, "yas525b");

	/* Any initializations in data must be done here too. */

	/* Tell the i2c layer a new client has arrived */
	if ((err=i2c_attach_client(new_client)))
		goto ERROR1;

	/* Now setup our character device to the compass */
	devno = MKDEV(YAS525B_MAJOR, YAS525B_MINOR);
	cdev_init(&data->cdev, &compass_fops);
	data->cdev.owner = THIS_MODULE;
	data->cdev.ops = &compass_fops;
	if ((err=cdev_add(&data->cdev, devno, 1)))
		goto ERROR2;

	printk("yas525b: Successfully detected\n");

	return 0;

ERROR2:
	i2c_detach_client(new_client);
ERROR1:
	kfree(new_client);
ERROR0:
	return err;
}

static int
compass_attach_adapter(struct i2c_adapter* adapter)
{
	return i2c_probe(adapter,&addr_data,compass_detect_client);
}

static int 
compass_detach_client(struct i2c_client* client)
{
	int err;

	/* Try to detach the client from i2c space */
	if ((err = i2c_detach_client(client))) {
		printk("yas525b: Client deregistration failed, client not detached.\n");
		return err;
	}

	kfree(client); /* Frees 'data' too */

	return 0;
}

static int compass_initialized = 0;

static int __init
compass_init(void)
{
	int res;

	printk("Yamaha YAS525B Magnetic Field Sensor driver, (C) 2006 Tom Tom BV\n");
	
	if ((res=i2c_add_driver(&compass_i2c_driver))) {
		printk("yas525b: Driver registration failed, module not inserted.\n");
	        return res;
	}
	
	compass_initialized ++;
	return 0;
}

static void
compass_cleanup(void)
{
	if (compass_initialized == 1) {
		if (i2c_del_driver(&compass_i2c_driver)) {
			printk("yas525b: Driver registration failed, module not removed.\n");
			return;
		}
		
		compass_initialized --;
	}
}

MODULE_AUTHOR("Ithamar Adema <ithamar.adema@tomtom.com>");
MODULE_DESCRIPTION("Driver for I2C connected YAS525B Magnetic Field Sensor");

module_init(compass_init);
module_exit(compass_cleanup);

