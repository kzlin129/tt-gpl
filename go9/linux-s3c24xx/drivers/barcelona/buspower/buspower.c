/* drivers/barcelona/gpio/buspower.c
 *
 * Implementation of the TomTom VBus power detection driver.
 *
 * Copyright (C) 2004,2005,2006,2007 TomTom BV <http://www.tomtom.com/>
 * Authors:	Ithamar Adema <Ithamar.adema@tomtom.com>
 *			Dimitry Andric <dimitry.andric@tomtom.com>
 *			Koen Martens <kmartens@sonologic.nl>
 *			Jeroen Taverne <jeroen.taverne@tomtom.com>
 *			Kees Jongenburger <kees.jongenburger@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/hardware/clock.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>

#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-hsudc.h>
#include <asm/arch/regs-udc.h>
#include <asm/arch/regs-dyn.h>

#include <barcelona/gopins.h>
#include <barcelona/Barc_buspower.h>

//#include "buspower.h"
//#include <barcelona/usbmode.h>

/* Defines */
#define DRIVER_DESC_SHORT  "TomTom GO Buspower monitor"
#define DRIVER_DESC_LONG   "TomTom GO Buspower monitor, (C) 2007 TomTom BV"

#define ERR " ERROR"
#define PK_DBG PK_DBG_FUNC
#define PFX "buspower:"

/* Interval we will check if there is buspower on the device */
#define VBUS_PROBE_INTERVAL_MILISEC 200


struct buspower_data {
	
	/* current power state when we are using vbus polling */
	int power_on;
	
	/* timer to keep track the v bus power when we use polling */
	struct timer_list power_timer;

	struct device *dev;/*back ref to the device */
};

static struct notifier_block * buspower_notifier_list;
static struct buspower_data * __data;


/**
 * sysfs entry attached to the driver to output the buspower status
 **/
static ssize_t buspower_sysfs_read_vbus(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", buspower_get_status());
}

extern int rds_tmc_hack_enabled;
static int buspower_state_change_event_requested = 0;

/* define driver_attr_mode variable this will be used to define the "vbus"
entry in the sysfs*/
static DEVICE_ATTR(vbus, S_IWUGO | S_IRUGO,  buspower_sysfs_read_vbus,NULL);

/* Function used when we use poll based USB_HOST_DETECT monitoring */
void buspower_probe_timer(unsigned long data_pointer)
{
	struct buspower_data * data = (struct buspower_data *)data_pointer;
	unsigned int power_on =-1;

	/* USB_HOST_DETECT == INPUT_BUS_POWER */
	if (rds_tmc_hack_enabled) {
		power_on = 0;
	} else {
		power_on = IO_GetInput(USB_HOST_DETECT);
	}

	if ((power_on != data->power_on) || buspower_state_change_event_requested) {
		buspower_state_change_event_requested = 0;
    
		data->power_on = power_on;
		/*if the power is on*/
		if (power_on){
			/* notify power on */
			notifier_call_chain(&buspower_notifier_list,1,NULL);
		} else {
			/* notify power off */
			notifier_call_chain(&buspower_notifier_list,0,NULL);
		}
		/* notify that the vbus mode  has changed */
		kobject_uevent_atomic(&data->dev->kobj,KOBJ_CHANGE,& dev_attr_vbus.attr);
	}
	
	/* schedule again in VBUS_PROBE_INTERVAL_MILISEC miliseconds */
	data->power_timer.expires = jiffies + (VBUS_PROBE_INTERVAL_MILISEC * HZ / 1000);
	add_timer(&data->power_timer);
};


/* start the power monitor */
void start_buspower_mon(struct buspower_data * data){
	
	/* poll based implementation 
	this casing has no direct usb port and the gpio in use there
	cases for usb vbus can not be configured for interrupt driven input.
	*/
	data->power_on=0;
	init_timer(&data->power_timer); /* only clears the timer */
	data->power_timer.expires = jiffies + (VBUS_PROBE_INTERVAL_MILISEC * HZ / 1000);
	data->power_timer.data = (unsigned long)data;
	data->power_timer.function = buspower_probe_timer;
	add_timer(&data->power_timer);
}



int buspower_register_notifier(struct notifier_block *nb)
{
	return notifier_chain_register(&buspower_notifier_list,nb);
};
EXPORT_SYMBOL(buspower_register_notifier);

int buspower_unregister_notifier(struct notifier_block *nb)
{
	return notifier_chain_unregister(&buspower_notifier_list,nb);
};
EXPORT_SYMBOL(buspower_unregister_notifier);

int buspower_get_status(void)
{
	if(__data){
		return __data->power_on;
	} 
	return 0;	
};
EXPORT_SYMBOL(buspower_get_status);

void buspower_request_state_change_event(void)
{
  buspower_state_change_event_requested = 1;
}

static int buspower_probe(struct device *dev)
{
	printk(KERN_DEBUG PFX  " %s\n",__FUNCTION__);
	if(! (__data = kmalloc(sizeof(struct buspower_data),GFP_KERNEL))){
		/*make sure we don't have dangling pointers */
		__data = NULL;
		printk(KERN_DEBUG PFX " failed to allocate memory for the buspower_data struct \n");
		return -ENOMEM;
	}
	buspower_notifier_list = NULL;
	/*clear the data and initialize */
	memset(__data, 0, sizeof(struct buspower_data));
	__data->dev = dev;
	device_create_file(dev, &dev_attr_vbus);
	start_buspower_mon(__data);
	return 0;
}

static int buspower_remove(struct device *dev)
{
	del_timer(&__data->power_timer);
	device_remove_file(dev, &dev_attr_vbus);
	return 0;
}

/*This module registers a tomtomgo-buspower bus type driver*/
static struct device_driver buspower_bus_driver = {
	.name = "tomtomgo-buspower",
	.bus = &platform_bus_type,
	.probe = buspower_probe,
	.remove = buspower_remove,
};


static int __init buspower_init_module(void)
{		
	int ret;
	__data = NULL;
	/*display driver information when the driver is started */
	printk(KERN_INFO DRIVER_DESC_LONG " (" PFX ")\n");
	
	ret = driver_register(&buspower_bus_driver);
	if (ret) {
		printk(KERN_ERR DRIVER_DESC_SHORT " unable to register driver (%d)\n", ret);
		return ret;
	}
	printk(KERN_DEBUG "Done\n");
	return 0;
}


static void __init buspower_exit_module(void)
{	
	driver_unregister(&buspower_bus_driver);
	printk(KERN_INFO DRIVER_DESC_SHORT " exit\n");
}

module_init(buspower_init_module);
module_exit(buspower_exit_module);

MODULE_DESCRIPTION(DRIVER_DESC_LONG);
MODULE_AUTHOR("Kees Jongenburger (kees.jongenburger@tomtom.com)");
MODULE_LICENSE("GPL");

