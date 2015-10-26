/* fm2010.c
 *
 * Control driver for ForteMedia chip.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Ithamar Adema <ithamar.adema@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

#include <linux/fm2010.h>

#include <barcelona/gopins.h>

struct fm2010_data {
	struct cdev cdev;
	/* Any per-client required data (cache) */
};

/* Spec says FM2010 sits on address 0x60 */
static unsigned short normal_i2c[] = { 0x60, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static int fm2010_attach(struct i2c_adapter* adapter);
static int fm2010_detach(struct i2c_client* client);

static struct i2c_driver driver = {
	.owner =           THIS_MODULE,
	.name =            "fm2010",
	.flags =           I2C_DF_NOTIFY,
	.attach_adapter =  fm2010_attach,
	.detach_client =   fm2010_detach,
};

static int
fm2010_open(struct inode* nodep, struct file* filep)
{
	struct fm2010_data *dev = container_of(nodep->i_cdev, struct fm2010_data, cdev);

	filep->private_data = ((char*)dev) - sizeof(struct i2c_client); /* for easy reference */

	return 0;
}

static int
fm2010_release(struct inode* nodep, struct file* filep)
{
	if (filep && filep->private_data)
		filep->private_data = NULL;

	return 0;
}

static int
fm2010_ioctl(struct inode* nodep, struct file* filep, unsigned int cmd, unsigned long arg)
{
	struct i2c_client* client = filep ? filep->private_data : NULL;

	struct i2c_msg msgs[3]; /* R & W multiple messages */
	char out[7] = { 0xFC, 0xF3 };
	struct fm2010args args;
	int rc = -ENOTTY;
	char in[4];

	if (_IOC_TYPE(cmd) != FM2010_DRIVER_MAGIC)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_WRITE && !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd)))
		return -EFAULT;

	copy_from_user(&args, (void*)arg, sizeof(args));
	
	switch(cmd) {
		case FM2010_MEMWRITE:
			out[2] = 0x3B;
			out[3] = args.addr >> 8;
			out[4] = args.addr & 0xFF;
			out[5] = args.data >> 8;
			out[6] = args.data & 0xFF;
			rc = i2c_master_send(client, out, 7);
		break;

		case FM2010_MEMREAD:
			out[2] = 0x37;
			out[3] = args.addr >> 8;
			out[4] = args.addr & 0xFF;
			rc = i2c_master_send(client, out, 5);
		break;
			
		case FM2010_LONGREGWRITE:
			out[2] = 0x6A;
			out[3] = args.addr & 0xFF;
			out[4] = args.data >> 8;
			out[5] = args.data & 0xFF;
			rc = i2c_master_send(client, out, 6);
		break;
			
		case FM2010_REGREAD:
			out[2] = 0x60;
			out[3] = args.addr & 0xFF;
			msgs[0].buf = out; msgs[1].buf = in;
			msgs[0].addr = msgs[1].addr = client->addr;
			msgs[0].flags = 0; msgs[1].flags = I2C_M_RD;
			msgs[0].len = 4; msgs[1].len = 1;
			if ((rc=i2c_transfer(client->adapter, msgs, 2)) == 0) {
				args.data = in[0];
				copy_to_user((void*)arg, &args, sizeof(args));
			}
		break;

		case FM2010_READADDR:
			/* mem read */
			args.data = 0;
			out[2] = 0x37;
			out[3] = args.addr >> 8;
			out[4] = args.addr & 0xFF;
			rc = i2c_master_send(client, out, 5);

			/* reg read 0x25 */
			out[2] = 0x60;
			out[3] = 0x25;
			msgs[0].buf = out; msgs[1].buf = in;
			msgs[0].addr = msgs[1].addr = client->addr;
			msgs[0].flags = 0; msgs[1].flags = I2C_M_RD; /* | I2C_M_NO_RD_ACK; */
			msgs[0].len = 4; msgs[1].len = 1;
			if ((rc=i2c_transfer(client->adapter, msgs, 2)) == 0) {
				args.data = in[0] << 8;
				
				/* reg read 0x26 */
				out[2] = 0x60;
				out[3] = 0x26;
				msgs[0].buf = out; msgs[1].buf = in;
				msgs[0].addr = msgs[1].addr = client->addr;
				msgs[0].flags = 0; msgs[1].flags = I2C_M_RD; /* | I2C_M_NO_RD_ACK; */
				msgs[0].len = 4; msgs[1].len = 1;
				if ((rc=i2c_transfer(client->adapter, msgs, 2)) == 0) {
					args.data |= in[0];
					copy_to_user((void*)arg, &args, sizeof(args));
				}
			}
		break;

		case FM2010_ANALOGBYPASS_ENABLE:
			IO_Activate(VP_IRQ_ANA);
		break;
		
		case FM2010_ANALOGBYPASS_DISABLE:
			IO_Deactivate(VP_IRQ_ANA);
		break;
		
		case FM2010_ACTIVATE_PWRDN:
			IO_Activate(VP_PWRDN);
		break;
		
		case FM2010_DEACTIVATE_PWRDN:
			IO_Deactivate(VP_PWRDN);
		break;
		
		case FM2010_ACTIVATE_RST:
			IO_Activate(VP_RST);
		break;
		
		case FM2010_DEACTIVATE_RST:
			IO_Deactivate(VP_RST);
		break;
	}

	return rc;
}

struct file_operations fm2010_fops = {
	.owner = THIS_MODULE,
	.open = fm2010_open,
	.release = fm2010_release,
	.ioctl = fm2010_ioctl
};


/* ----------------------------------------------------------------------- */

int fm2010_detect_client(struct i2c_adapter *adapter, int address, int kind)
{
	const char *client_name = "ForteMedia 2010";
	struct i2c_client *new_client;
	struct fm2010_data *data;
	int devno, err = 0;

	/* Let's see whether this adapter can support what we need.
		Please substitute the things you need here!
	if (!i2c_check_functionality(adapter,I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_WRITE_BYTE))
		goto ERROR0;
*/
	/* OK. For now, we presume we have a valid client. We now create the
		client structure, even though we cannot fill it completely yet.
		But it allows us to access several i2c functions safely */

	/* Note that we reserve some space for fm2010_data too.
		We do it here to help to lessen memory fragmentation. */
	if (!(new_client=kcalloc(1, sizeof(struct i2c_client) + sizeof(struct fm2010_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto ERROR0;
	}

	/* This is tricky, but it will set the data to the right value. */
	i2c_set_clientdata(new_client, new_client+1);
	data = (struct fm2010_data *) i2c_get_clientdata(new_client);

	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &driver;
	new_client->flags = 0;

	/* Now, we do the remaining detection. If no `force' parameter is used. */
#if 0
	/* First, the generic detection (if any), that is skipped if any force
		parameter was used. */
	if (kind < 0) {
		if (foo_read(new_client,FOO_REG_GENERIC) != FOO_GENERIC_VALUE)
			goto ERROR1;
	}
#endif

	/* Fill in the remaining client fields. */
	strcpy(new_client->name,client_name);

	/* Tell the i2c layer a new client has arrived */
	if ((err=i2c_attach_client(new_client)) != 0)
		goto ERROR3;

	/* Now setup our character device to the fm2010 */
	devno = MKDEV(FM2010_MAJOR, FM2010_MINOR);
	cdev_init(&data->cdev, &fm2010_fops);
	data->cdev.owner = THIS_MODULE;
	data->cdev.ops = &fm2010_fops;
	if ((err=cdev_add(&data->cdev, devno, 1)))
		goto ERROR4;

	return 0;

ERROR4:
	i2c_detach_client(new_client);
ERROR3:
	kfree(new_client);
ERROR0:
	return err;
}


static int
fm2010_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap,&addr_data,fm2010_detect_client);
}

static int
fm2010_detach(struct i2c_client *c)
{
	printk("%s: %p\n", __func__, c);

	i2c_detach_client(c);
	kfree(c);

	return 0;
}

/* ----------------------------------------------------------------------- */

static int __init 
fm2010_init(void)
{
	printk("%s: init\n", __func__);
	IO_Deactivate(VP_IRQ_ANA);
	IO_Deactivate(VP_PWRDN);
	IO_Activate(DAC_PWR_ON);
	mdelay(100);
	IO_Activate(VP_RST);
	mdelay(50);
	IO_Deactivate(VP_RST);
	mdelay(50);
	return i2c_add_driver(&driver);
}

static void __exit
fm2010_exit(void)
{
	printk("%s: exit\n", __func__);
	i2c_del_driver(&driver);
	IO_Activate(VP_RST);
	IO_Activate(VP_PWRDN);
}

MODULE_AUTHOR("Ithamar Adema <ithamar.adema@tomtom.com>");
MODULE_DESCRIPTION("Driver for I2C connected ForteMedia 2010 DSP");

module_init(fm2010_init);
module_exit(fm2010_exit);

