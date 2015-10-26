/*
 * Synaptics embedded Touchpad driver for TomTom GO devices.
 *
 *              This program is free software; you can redistribute it and/or
 *              modify it under the terms of the GNU General Public License
 *              as published by the Free Software Foundation; either version
 *              2 of the License, or (at your option) any later version.
 *
 * Author:      Ithamar R. Adema, <ithamar.adema@tomtom.com>
 *
 * WARNING:	The minor device # is used as the device ID for the MEP device you want to talk to.
 *		However, this has all only been tested on a single-module MEP bus. So although care
 *		has been taken to make it fully functional, it _could_ (and probably will) not function
 *		as expected when multiple modules are connected!
 */

#include <linux/config.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/fs.h>

#include "mep_bus.h"

extern mep_bus_t mep_bus;

static int
mep_dev_open(struct inode *inode, struct file *file)
{
	int id = iminor(inode);
	int rc = 0;

	if (id >= 0 && id < MEP_MAX_DEVICES) {
		if (mep_bus.devices[id] == NULL) {
			mep_dev_t* dev = kmalloc(sizeof(mep_dev_t), GFP_KERNEL);
			if (dev != NULL) {
				init_waitqueue_head(&dev->inq);
				dev->lock = SPIN_LOCK_UNLOCKED;
				dev->bus = &mep_bus;
				dev->id = id;
				dev->changed = 0;
				dev->state.w = 0;
				dev->state.x = 0;
				dev->state.y = 0;
				dev->state.z = 0;
				dev->state.buttons = 0;
				dev->curr_buttons = 0;
				mep_bus.devices[id] = dev;
				file->private_data = dev;
			} else {
				rc = -ENOMEM;
			}
		} else {
			rc = -EBUSY;
		}
	} else {
		rc = -EINVAL;
	}

	return rc;
}

static int
mep_dev_close(struct inode *inode, struct file *file)
{
	mep_dev_t* dev = (mep_dev_t*)file->private_data;
	if (dev != NULL) {
		mep_bus.devices[dev->id] = NULL;
		kfree(dev);
		file->private_data = NULL;
	}

	return 0;
}

static ssize_t
mep_dev_write(struct file* file, const char __user* buf, size_t count, loff_t* ppos)
{
	mep_dev_t* dev = (mep_dev_t*)file->private_data;
	mep_packet_t pkt;
	int res = 0;

	if (count < 1 || count > 8)
		return -EINVAL;

	if (dev != NULL) {
		copy_from_user(pkt.rawPkt, buf, count);

		res = mep_tx(&pkt);
		if (res != MEP_NOERR)
			count = -EIO;
	}

	return count;
}

static ssize_t
mep_dev_read(struct file *file, char __user * buf, size_t count, loff_t *ppos)
{
	unsigned long flags;
	int bytes_copied = 0;

	mep_dev_t* dev = (mep_dev_t*)file->private_data;
	if (dev != NULL) {
	
		spin_lock_irqsave(&dev->lock, flags);
		
		if (count >= sizeof(mep_state) && dev->changed)
		{
			copy_to_user(buf, &(dev->state), sizeof(mep_state));
			bytes_copied = sizeof(mep_state);
			if (dev->state.buttons != dev->curr_buttons) {
				dev->state.buttons = dev->curr_buttons;
			} else {
				dev->changed = 0;
			}
		}
	
		spin_unlock_irqrestore(&dev->lock, flags);
	} else
		return -EINVAL;

	return bytes_copied;
}

static unsigned int
mep_dev_poll(struct file* file, struct poll_table_struct* wait)
{
	mep_dev_t* dev = (mep_dev_t*)file->private_data;
	unsigned int mask = 0;
	unsigned long flags;
	extern wait_queue_head_t mep_wait;

	poll_wait(file, &mep_wait, wait);

	spin_lock_irqsave(&dev->lock, flags);
	poll_wait(file, &dev->inq, wait);
		if (dev->changed)
			mask |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&dev->lock, flags);

	return mask;
}	

struct file_operations
mep_dev_fops = {
        .owner          = THIS_MODULE,
	.open           = mep_dev_open,
	.release        = mep_dev_close,
	.read           = mep_dev_read,
	.write		= mep_dev_write,
	.poll		= mep_dev_poll,
	.llseek         = no_llseek,
};

