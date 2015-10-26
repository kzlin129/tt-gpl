/**
 * \file
 * Synaptics RMI4 over I2C Physical Layer Driver.
 * Copyright (c) 2007-2009 Synaptics Incorporated
 *
 * This file is triple licensed under the GPL2, MPL, and Apache licenses.
 */
/*
 *
 *#############################################################################
 * GPL
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 *#############################################################################
 *
 * Mozilla Public License
 *
 * The contents of this file are subject to the Mozilla Public License
 * Version 1.1 (the "License"); you may not use this file except in
 * compliance with the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS IS"
 * basis, WITHOUT WARRANTY OF ANY KIND, either express or implied. See the
 * License for the specific language governing rights and limitations
 * under the License.
 *
 * The Original Code is this file.
 *
 * The Initial Developer of the Original Code is Synaptics, Inc.  Portions
 * created by Synaptics, Inc. are Copyright (c) 2007-2008 Synaptics, Inc. All
 * Rights Reserved.
 *
 * Alternatively, the contents of this file may be used under the terms of the
 * GNU General Public License version 2 or Apache License version 2.0 (the
 * "Alternate License"), in which case the provisions of Alternate License are
 * applicable instead of those above. If you wish to allow use of your version
 * of this file only under the terms of one of the Alternate Licenses and not
 * to allow others to use your version of this file under the MPL, indicate
 * your decision by deleting the provisions above and replace them with the
 * notice and other provisions required by the Alternate License. If you do not
 * delete the provisions above, a recipient may use your version of this file
 * under either the MPL or one of the Alternate Licenses."
 *
 *#############################################################################
 *
 * Apache License
 *
 * Copyright (c) 2007-2008 Synaptics, Inc. Licensed under the Apache License,
 * Version 2.0 (the "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 *#############################################################################
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include "linux/rmi_i2c.h"
#include "rmi.h"

/** Used to lock access to the page address.
 * \see rmi_set_page()
 */
static DEFINE_MUTEX(page_mutex);

/**
 * This delay will hopefully go away.  It should be unnecessary, but on the
 * GHI dev platform, reads fail without it.
 * \note It is only required for the GHI development system.  It should not be
 * required for the FIC phone.  As mentioned previously, it will be deleted
 * once we migrate off the GHI development system. (It should not be
 * required for the GHI development system, and it turns out it is required
 * for the phone, too.  Perhaps it has something to do with the bus noise.)
 */
#if 0
#   warning UNEXPLAINED_DELAY should not be necessary.
#   define UNEXPLAINED_DELAY  udelay(90)
#else
#   define UNEXPLAINED_DELAY  do { } while(0)
#endif

/** This is a count of how many clients are accessing this driver.
 */
static int num_boards;
static struct i2c_board_info *rmi_boards_info;

/** This list is used in definition of the I2C bus scanning parameters.
 * There is nothing on our "normal" i2c list.
 */
static const unsigned short normal_i2c[] = { I2C_CLIENT_END,
	I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END,
	I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END, I2C_CLIENT_END,
};
/** This list is used in definition of the I2C bus scanning parameters.
 * There is nothing on our "force" i2c list.
 */
static const unsigned short force[] = { I2C_CLIENT_END, I2C_CLIENT_END };
/** This list is used in definition of the I2C bus scanning parameters.
 * The only thing on our "forces" list is a single empty list.
 */
static const unsigned short *forces[] = { force, 0 };
/** Call to a kernel macro to set up our I2C bus scanning parameters.
 * This relies on the forces and normal_i2c data structures.
 */
I2C_CLIENT_INSMOD_COMMON;

static struct i2c_driver rmi_i2c_driver;

/**
 * This is the data kept on a per instance (client) basis.  This data is
 * always accessible by using the container_of() macro of the various elements
 * inside.
 */
struct instance_data {
	int instance_no;
	int irq;
	struct rmi_phys_driver rpd;
	struct i2c_client i2cclient;
	int page;
	int (*get_attention)(void);
};

/**
 * RMI devices have 16-bit addressing, but some of the physical
 * implementations (like SMBus) only have 8-bit addressing.  So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.  This function sets the page.
 * \pre The page_mutex lock must be held when this function is entered.
 * \param[in] id TBD
 * \param[in] page The new page address.
 * \return zero on success, non-zero on failure.
 */
int
rmi_set_page(struct instance_data *id, unsigned int page)
{
	char txbuf[2];
	int retval;
	txbuf[0] = 0xff;
	txbuf[1] = page;

	/*printk(KERN_INFO "\nrmi_set_page\n");

	printk(KERN_INFO "client: addr=[0x%x] name=[%s] drv_name=[%s] irq=[%d] flags=[%d]\n", id->i2cclient.addr, id->i2cclient.name, id->i2cclient.driver_name, id->i2cclient.irq, id->i2cclient.flags);

	printk(KERN_INFO "page: [0x%x]\n\n", page);*/
	
	retval = i2c_master_send(&id->i2cclient, txbuf, 2);
	if(retval != 2) {
		printk(KERN_ERR "rmi_i2c: Set page fail: %d\n", retval);
	} else {
		retval = 0;
		id->page = page;
	}
	return retval;
}

/**
 * Read a single register through i2c.
 * \param[in] pd TBD
 * \param[in] address The address at which to start the data read.
 * \param[out] valp Pointer to the buffer where the data will be stored.
 * \return xero upon success (with the byte read in valp), non-zero upon error.
 */
static int
rmi_i2c_read(struct rmi_phys_driver *pd, unsigned short address, char *valp)
{
	struct instance_data *id = container_of(pd, struct instance_data, rpd);
	char txbuf[2];
	int retval = 0;
	int retry_count = 0;

	
	/*printk(KERN_INFO "\nrmi_i2c_read\n");

	printk(KERN_INFO "client: addr=[0x%x] name=[%s] drv_name=[%s] irq=[%d] flags=[%d]\n", id->i2cclient.addr, id->i2cclient.name, id->i2cclient.driver_name, id->i2cclient.irq, id->i2cclient.flags);

	printk(KERN_INFO "addr of read: [0x%x]\n", address);
	printk(KERN_INFO "addr of buffer: [0x%x]\n\n", valp);*/

	/* Can't have anyone else changing the page behind our backs */
	mutex_lock(&page_mutex);

	if(((address >> 8) & 0xff) != id->page) {
		/* Switch pages */
		retval = rmi_set_page(id, ((address >> 8) & 0xff));
		if(retval) {
			goto exit;
		}
	}

retry:
	txbuf[0] = address & 0xff;
	retval = i2c_master_send(&id->i2cclient, txbuf, 1);
	UNEXPLAINED_DELAY;
	if(retval != 1) {
		printk(KERN_ERR "rmi_i2c.rmi_i2c_read: Write fail: %d\n",
			retval);
		goto exit;
	}
	retval = i2c_master_recv(&id->i2cclient, txbuf, 1);
	UNEXPLAINED_DELAY;
	if(retval != 1) {
		if(++retry_count == 5) {
			printk(KERN_ERR "rmi_i2c.rmi_i2c_read: "
				"Read of 0x%04x fail: %d\n", address, retval);
		} else {
			mdelay(10);
			rmi_set_page(id, ((address >> 8) & 0xff));
			goto retry;
		}
	} else {
		retval = 0;
		*valp = txbuf[0];
	}
exit:
	mutex_unlock(&page_mutex);
	return retval;
}

/**
 * Same as rmi_i2c_read, except that multiple bytes are allowed to be read.
 * \param[in] pd TBD
 * \param[in] address The address at which to start the data read.
 * \param[out] valp Pointer to the buffer where the data will be stored.  This
 * buffer must be at least size bytes long.
 * \param[in] size The number of bytes to be read.
 * \return zero upon success (with the byte read in valp), non-zero upon error.
 * \see rmi_i2c_read()
 */
static int
rmi_i2c_read_multiple(struct rmi_phys_driver *pd, unsigned short address,
	void *valp, int size)
{
	struct instance_data *id = container_of(pd, struct instance_data, rpd);
	char txbuf[2];
	int retval = 0;
	int retry_count = 0;


	/*printk(KERN_INFO "\nrmi_i2c_read_multiple\n");

	printk(KERN_INFO "client: addr=[0x%x] name=[%s] drv_name=[%s] irq=[%d] flags=[%d]\n", id->i2cclient.addr, id->i2cclient.name, id->i2cclient.driver_name, id->i2cclient.irq, id->i2cclient.flags);

	printk(KERN_INFO "addr of read: [0x%x] size of read: [%d]\n", address, size );
	printk(KERN_INFO "addr of buffer: [0x%x]\n\n", valp);*/


	/* Can't have anyone else changing the page behind our backs */
	mutex_lock(&page_mutex);

	if(((address >> 8) & 0xff) != id->page) {
		/* Switch pages */
		retval = rmi_set_page(id, ((address >> 8) & 0xff));
		if(retval) {
			goto exit;
		}
	}

retry:
	txbuf[0] = address & 0xff;
	retval = i2c_master_send(&id->i2cclient, txbuf, 1);
	UNEXPLAINED_DELAY;
	if(retval != 1) {
		printk(KERN_ERR "rmi_i2c.rmi_i2c_read: Write fail: %d\n",
			retval);
		goto exit;
	}
	retval = i2c_master_recv(&id->i2cclient, valp, size);
	UNEXPLAINED_DELAY;
	if(retval != size) {
		if(++retry_count == 5) {
			printk(KERN_ERR "rmi_2ic.rmi_i2c_read_multiple: "
				"Read of 0x%04x size %d fail: %d\n",
				address, size, retval);
		} else {
			mdelay(10);
			rmi_set_page(id, ((address >> 8) & 0xff));
			goto retry;
		}
	} else {
		retval = 0;
	}
exit:
	mutex_unlock(&page_mutex);
	return retval;
}


/**
 * Write a single register through i2c.
 * You can write multiple registers at once, but I made the functions for that
 * seperate for performance reasons.  Writing multiple requires allocation and
 * freeing.
 * \param[in] pd TBD
 * \param[in] address The address at which to start the write.
 * \param[in] data The data to be written.
 * \return one upon success, something else upon error.
 */
static int
rmi_i2c_write(struct rmi_phys_driver *pd, unsigned short address, char data)
{
	struct instance_data *id = container_of(pd, struct instance_data, rpd);
	unsigned char txbuf[2];
	int retval = 0;

	/*printk(KERN_INFO "\nrmi_i2c_write\n");

	printk(KERN_INFO "client: addr=[0x%x] name=[%s] drv_name=[%s] irq=[%d] flags=[%d]\n", id->i2cclient.addr, id->i2cclient.name, id->i2cclient.driver_name, id->i2cclient.irq, id->i2cclient.flags);

	printk(KERN_INFO "addr of write: [0x%x]\n", address);
	printk(KERN_INFO "addr of buffer: [0x%x]\n\n", &data);*/

	/* Can't have anyone else changing the page behind our backs */
	mutex_lock(&page_mutex);

	if(((address >> 8) & 0xff) != id->page) {
		/* Switch pages */
		retval = rmi_set_page(id, ((address >> 8) & 0xff));
		if(retval) {
			goto exit;
		}
	}

	txbuf[0] = address & 0xff;
	txbuf[1] = data;
	retval = i2c_master_send(&id->i2cclient, txbuf, 2);
	UNEXPLAINED_DELAY;
	if(retval != 2) {
		printk(KERN_ERR "rmi_i2c.rmi_i2c_write: Write fail: %d\n",
			retval);
		goto exit; /* Leave this in case we add code below */
	}
exit:
	mutex_unlock(&page_mutex);
	return retval;
}

/**
 * Write multiple registers.
 * \param[in] pd TBD
 * \param[in] address The address at which to start the write.
 * \param[in] valp A pointer to a buffer containing the data to be written.
 * \param[in] size The number of bytes to write.
 * \return one upon success, something else upon error.
 */
static int
rmi_i2c_write_multiple(struct rmi_phys_driver *pd, unsigned short address,
	void *valp, int size)
{
	struct instance_data *id = container_of(pd, struct instance_data, rpd);
	unsigned char *txbuf;
	unsigned char txbuf_most[16];
	int retval = 0;

	/*printk(KERN_INFO "\nrmi_i2c_write_multiple\n");

	printk(KERN_INFO "client: addr=[0x%x] name=[%s] drv_name=[%s] irq=[%d] flags=[%d]\n", id->i2cclient.addr, id->i2cclient.name, id->i2cclient.driver_name, id->i2cclient.irq, id->i2cclient.flags);

	printk(KERN_INFO "addr of write: [0x%x] size of write: [%d]\n", address, size );
	printk(KERN_INFO "addr of buffer: [0x%x]\n\n", valp);*/

	if(size < 15) {
		/* Avoid an allocation if we can help it. */
		txbuf = txbuf_most;
	} else {
		txbuf = kmalloc(size + 1, GFP_KERNEL);
		if(!txbuf) return -ENOMEM;
	}

	/* Yes, it stinks here that we have to copy the buffer */
	{
		int i;
		for(i = 0; i < size; i++) {
			txbuf[i + 1] = ((char *)valp)[i];
		}
	}

	/* Can't have anyone else changing the page behind our backs */
	mutex_lock(&page_mutex);

	if(((address >> 8) & 0xff) != id->page) {
		/* Switch pages */
		retval = rmi_set_page(id, ((address >> 8) & 0xff));
		if(retval) {
			goto exit;
		}
	}

	txbuf[0] = address & 0xff;
	retval = i2c_master_send(&id->i2cclient, txbuf, size + 1);
	UNEXPLAINED_DELAY;
	if(retval != 1) {
		printk("rmi_i2c.rmi_i2c_read: Write fail: %d\n", retval);
		goto exit;
	}
exit:
	mutex_unlock(&page_mutex);
	if(txbuf != txbuf_most) kfree(txbuf);
	return retval;
}

/** Get the state of the attention line.
 * This function returns 1 for an active attention regardless of the
 * polarity of the ATTN signal.  If the get_attention function of the instance
 * is not available (probably because ATTN is not implemented), then it always
 * returns inactive.
 */ 
static int
rmi_i2c_get_attention(struct rmi_phys_driver *rpd)
{
	struct instance_data *id = container_of(rpd, struct instance_data, rpd);
	if(id->get_attention) {
		return id->get_attention();
	} else {
		return 0;
	}
}

/**
 * This is the Interrupt Service Routine.  It just notifies the application
 * layer that attention is required.
 *
 * This is a work in progress.  It has yet to be figured out how the interrupt
 * should be disabled/enabled for interrupt-driven platforms.
 */
static irqreturn_t
i2c_attn_isr(int irq, void *info)
{
	struct instance_data *id = info;
	if(id->rpd.attention) {
		id->rpd.attention(&id->rpd, id->instance_no);
	}
	return IRQ_HANDLED;
}


/**
 * This function sets up the structures for each device on the i2c bus
 */
static int rmi_i2c_init(struct i2c_client* client)
{
	struct instance_data *id;
	int retval;
	//int i;
	rmi_pdata_t *mid;
	static int counter_instances=0;

	//for (i = 0; i < num_boards; i++) {

	printk(KERN_INFO SYNAPTIC_DEVNAME ": rmi i2c board found: addr %08x\n", client->addr);
	printk(KERN_INFO SYNAPTIC_DEVNAME ": counter instances i2c rmi phys driver [%d]\n", counter_instances);
 
	mid = (rmi_pdata_t *) (client->dev.platform_data);
	id = kmalloc(sizeof(*id) * 2, GFP_KERNEL);
	
	if(!id) {
		printk(KERN_ERR SYNAPTIC_DEVNAME ": Out of memory\n");
		return -ENOMEM;
	}

	memset(id, 0, sizeof(*id));
	
	id->rpd.name           = SYNAPTIC_DEVNAME;
	id->rpd.write          = rmi_i2c_write;
	id->rpd.read           = rmi_i2c_read;
	id->rpd.write_multiple = rmi_i2c_write_multiple;
	id->rpd.read_multiple  = rmi_i2c_read_multiple;
	id->rpd.get_attention  = rmi_i2c_get_attention;
	id->rpd.module         = THIS_MODULE;
	id->page               = 0xffff;    /* So we set the page correctly */

	id->i2cclient	   	= *client;
	i2c_set_clientdata(&(id->i2cclient), id);

	id->instance_no = counter_instances++;
	id->get_attention = mid->get_attention;

	/*
	 * Determine if we need to poll (inefficient) or use
	 * interrupts.
	 */
	if(client->irq) {
		int irqtype;
		id->irq = client->irq;
		irqtype = IRQF_TRIGGER_FALLING;
		if((retval = request_irq(id->irq, i2c_attn_isr,
			    IRQF_DISABLED | irqtype, "rmi_i2c", id))) {
				printk(KERN_INFO "rmi_i2c: Unable to get attn "
				  "irq %d.  Reverting to polling.\n",
				  id->irq);
				goto do_polling;
			}
		printk(KERN_INFO SYNAPTIC_DEVNAME ": got irq\n");
		id->rpd.polling_required = 0;
	} else {
do_polling:
		id->rpd.polling_required = 1;
		printk(KERN_INFO SYNAPTIC_DEVNAME ": No IRQ info given. "
			"Polling required.\n");
	}

	if((retval = rmi_register_phys_driver(&(id->rpd)))) {
		if(id->irq) {
			free_irq(id->irq, id);
		}
		i2c_detach_client(&(id->i2cclient));
		kfree(id);
		return retval;
	}

	printk(KERN_INFO SYNAPTIC_DEVNAME ": Successfully Registered %s phys driver\n", id->rpd.name);
	rmi_set_page(id, 0x04);
	//}

	return 0;
}

/**
 * This function tears down the structures for each instance.
 */
static int
detach_client(struct i2c_client *client)
{
	struct instance_data *id =
		container_of(client, struct instance_data, i2cclient);
	int err;

	/* flush_scheduled_work(); */

	printk(KERN_INFO SYNAPTIC_DEVNAME ": Unregistering phys driver %s\n", id->rpd.name);

	rmi_unregister_phys_driver(&id->rpd);

	printk(KERN_INFO SYNAPTIC_DEVNAME ": Unregistered phys driver %s\n", id->rpd.name);

	if ((err = i2c_detach_client(client))) {
		printk(KERN_INFO SYNAPTIC_DEVNAME ": i2c_detach_client failed: %d\n", err);
		return err;
	}

	if(id->irq) {
		free_irq(id->irq, id);
	}

	kfree(id);
	printk(KERN_INFO SYNAPTIC_DEVNAME ": detach_client successful\n");
	return 0;

}


/**
 * The Platform Driver probe function.  We just tell the i2c subsystem about
 * ourselves in this call.
 */
static int
rmi_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	rmi_pdata_t *mid = (rmi_pdata_t*) (client->dev.platform_data);
	//int i;

	printk(KERN_INFO SYNAPTIC_DEVNAME ": Probing i2c RMI device\n");
	//if(!mid) {
	//	printk("A platform_device for \"rmi_i2c\" must contain "
	//		"rmi_i2c_data\n");
	//	return -ENXIO;
	//}

	//num_boards = mid->num_i2c_boards;
	//if(num_boards > ARRAY_SIZE(normal_i2c) - 1) {
	//	num_boards = ARRAY_SIZE(normal_i2c) - 1;
	//	printk(KERN_WARNING "Too many boards defined for i%s. "
	//		"Limiting to %d\n", client->dev.name, num_boards);
	//}

	//for(i = 0; i < ARRAY_SIZE(normal_i2c); i++) {
	//	normal_i2c[i] = I2C_CLIENT_END;
	//}

	//for(i = 0; i < num_boards; i++) {
	//	normal_i2c[i] = (&(client[i]))->addr;
	//}
	
	//init GPIO low level stuffs in platform dev/arch dependent part of code
	//mid->init();

	//msleep(1000);

	//init rmi phys driver struct
	rmi_i2c_init(client);


	return 0;
}

/**
 * Tell the i2c subsystem that we're done.
 * \param[in] dev TBD
 * \return Always returns 0.
 */
static int
rmi_i2c_remove(struct i2c_client *client)
{
	int i;
	for (i = 0; i < num_boards; i++) {

		detach_client(&client[i]);

	}
	return 0;
}


static const struct i2c_device_id rmi_id[] = {
	{ SYNAPTIC_DEVNAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rmi_id);

/**
 * This structure tells the i2c subsystem about us.
 */
#define rmi_i2c_suspend		NULL
#define rmi_i2c_resume		NULL

static struct i2c_driver rmi_i2c_driver = {
	.driver = {
		.name 	= SYNAPTIC_DEVNAME,
		.owner	= THIS_MODULE,
	},
	.id_table 	= rmi_id,
	.id 		= 0xfefe,	/* This ID is apparently unused. */
	.probe 		= rmi_i2c_probe,
	.remove		= rmi_i2c_remove,
	.suspend	= rmi_i2c_suspend,
	.resume		= rmi_i2c_resume,
};


/** Print an informational message to the kernel
 * log and register ourselves with the Platform Driver Subsystem.
 * This will be called when the module is inserted.
 * \return the result of our call to platform_driver_register()
 */
static int __init mod_init(void)
{
	int err;

	printk(KERN_INFO SYNAPTIC_DEVNAME ": RMI I2C Driver\n");
	if(RMI_ALLOC_STATS) {
		printk(KERN_INFO SYNAPTIC_DEVNAME ": Allocation Stats Enabled\n");
	}
	
	printk(KERN_INFO SYNAPTIC_DEVNAME ": Calling i2c_add_driver\n");

    	if ((err = i2c_add_driver(&rmi_i2c_driver))) {
		printk(KERN_ERR SYNAPTIC_DEVNAME ": Could Not Be Added. Err Code: [%i]\n", err);
	}

	return err;

}

/** Un-register ourselves from the Platform Driver Subsystem.
 * This will be called when the module is removed.
 */
static void __exit mod_exit(void)
{
	
	printk(KERN_INFO SYNAPTIC_DEVNAME ": start exit\n");
	i2c_del_driver(&rmi_i2c_driver);
}

/** Standard driver module information - the author of the module.
 */
MODULE_AUTHOR("Synaptics, Inc.");
/** Standard driver module information - a summary description of this module.
 */
MODULE_DESCRIPTION("RMI4 I2C Driver");
/** Standard driver module information - the license under which this module
 * is included in the kernel.
 */
MODULE_LICENSE("GPL");

/** Specifies to the kernel that the mod_init() function should be called when
 * the module is loaded.
 * \see mod_init()
 */
module_init(mod_init);
/** Specifies to the kernel that the mod_exit() function should be called when
 * the module is unloaded.
 * \see mod_exit()
 */
module_exit(mod_exit);

/* vim600: set noexpandtab sw=8 ts=8 :*/
