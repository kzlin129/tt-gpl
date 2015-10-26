/*
 * 
 *  Broadcom USB gadget hotplugging device driver  
 *
 *  Copyright (C) 2006-2009 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#define __KERNEL_SYSCALLS__

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/stat.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/wait.h>
#include <linux/init.h>
#include <linux/smp_lock.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <asm/io.h>

#include "udc_hotplug.h"


#ifdef MODULE
#define MOD_IN_USE_COUNT (GET_USE_COUNT (&__this_module))
#endif	/* MODULE */

#define DRIVER_VERSION "Version 0.2"
#define DRIVER_DATE    "Jan 05, 2009"
#define DRIVER_DESC    "BRCM USB gadget hotplugging driver."
#define DRIVER_INFO    DRIVER_DESC " " DRIVER_VERSION " " DRIVER_DATE

#define UDCHP_DEV_NAME	"udchp"
#define UDCHP_MAJOR_NR	226

	/* 
	 *    Global variables		
	 */

static int	udchp_major_nr = UDCHP_MAJOR_NR;
static struct class *udchp_class;
static struct class_device *uhchp_class_dev;
static int udchp_connected = 0;
static spinlock_t udchp_spinlock;
static struct work_struct udchp_work;

static int udchp_open(struct inode *inode, struct file *file)
{
	if (MAJOR(inode->i_rdev) != udchp_major_nr)
	{
		BUG();
		return -1;
	}

#ifdef MODULE	
	MOD_INC_USE_COUNT;
#endif

	return 0;
}

static int udchp_release(struct inode *inode, struct file *file)
{
	if (MAJOR(inode->i_rdev) != udchp_major_nr)
	{
		BUG();
		return -1;
	}

#ifdef MODULE
		MOD_DEC_USE_COUNT;
#endif
	
	return 0;
}


static struct file_operations udchp_fops = {
	.owner		= THIS_MODULE,
	.open		= udchp_open,
	.release	= udchp_release,
};


static ssize_t udchp_state_show(struct class *class_dev, char *buf)
{
	int len;

	spin_lock(&udchp_spinlock);
	
	len = udchp_connected ? sprintf(buf, "connected\n") : 
							sprintf(buf, "disconnected\n"); 

	spin_unlock(&udchp_spinlock);
	
	return ((ssize_t) len);
}

static CLASS_ATTR(state, S_IRUGO, udchp_state_show, NULL);

static void udchp_notify_add(void)
{
	spin_lock(&udchp_spinlock);

	/* In case we run into some GPIO issues, like debounce.  
	 */
	if (udchp_connected)
	{
		printk(KERN_INFO "USB add event already generated\n");
		spin_unlock(&udchp_spinlock);
		return;
	}

	udchp_connected = 1;
	uhchp_class_dev = 
		device_create(udchp_class, MKDEV(udchp_major_nr, 0),
					 		NULL, NULL, "cable_connected");

	spin_unlock(&udchp_spinlock);
}

static void udchp_notify_remove(void)
{
	spin_lock(&udchp_spinlock);

	/* In case we run into some GPIO issues, like debounce.  
	 */
	if (!udchp_connected)
	{
		printk(KERN_INFO "USB remove event already generated\n");
		spin_unlock(&udchp_spinlock);
		return;
	}

	device_del(uhchp_class_dev);
	udchp_connected = 0;

	spin_unlock(&udchp_spinlock);
}

static int isConnected(void)
{
    /* Need to add a test routine. Test USB registers?? */
	return (1);
}

static void udchp_notify_user(void *ptr)
{
	if (isConnected())
		udchp_notify_add();
	else
		udchp_notify_remove();
}

void udchp_trigger_callback(unsigned int param)
{
	schedule_work(&udchp_work);
}

static void udchp_coldplug(void)
{
	if (isConnected())
		udchp_notify_add();
}

static int __init udchp_init(void)
{
	int err;
	
	spin_lock_init(&udchp_spinlock);

	/* Register the device.
	 */
	if (register_chrdev(udchp_major_nr, UDCHP_DEV_NAME, &udchp_fops)) {
		printk(KERN_NOTICE "Can't allocate major number %d for UDC Hotplug device.\n",
		       UDCHP_MAJOR_NR);
		return -EAGAIN;
	}

	/* Create the sysfs class. 
	 */
	udchp_class = class_create(THIS_MODULE, "usb_gadget");

	if (IS_ERR(udchp_class)) {
		printk(KERN_ERR "Error creating usb_gadget class.\n");
		unregister_chrdev(udchp_major_nr, UDCHP_DEV_NAME);
		return PTR_ERR(udchp_class);
	}

	/* Create a sysfs file for user to see the connection state. 
	 * Coldplugging is handled by reading this file. 
	 */
	if ((err = class_create_file(udchp_class, &class_attr_state))) {
		printk(KERN_ERR "Error creating udc_hotplug state file.\n");
		class_destroy(udchp_class);
		unregister_chrdev(udchp_major_nr, UDCHP_DEV_NAME);
		return err;
	}

	INIT_WORK(&udchp_work, udchp_notify_user);

	/* Detect USB connection state for coldplugging. 
	 */
	//udchp_coldplug();
	
	printk(KERN_INFO "BRCM USB gadget hotplugging driver loaded.\n");
	
	return 0;
}

static void __exit udchp_exit(void)
{
	
	if (udchp_connected)
		udchp_notify_remove();
	class_remove_file(udchp_class, &class_attr_state);
	class_destroy(udchp_class);
	unregister_chrdev(udchp_major_nr, UDCHP_DEV_NAME);

	printk(KERN_INFO "BRCM USB gadget hotplugging driver unloaded.\n");
}



#ifdef MODULE
 MODULE_DESCRIPTION (DRIVER_INFO);
#endif

 module_init(udchp_init);
 module_exit(udchp_exit);
 
