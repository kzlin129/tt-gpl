/* drivers/barcelona/tsinput/tsinput.c

Implementation of the touchscreen input driver.
This code is based on the original barcelona ts driver but
as difference use the input framework. This modules is based
on the idea that calibration and transforming the data
to screen points should happen in the OS abstraction layer.
Warning:This implementation supports "only" one touchscreen

TODO:kejo:03-2007 add /sysfs configuration for magic numbers
TODO:kejo:03-2007 test the tsinput_remove. this code has not been tested
TODO:checkout other touchscreen based input method, perhaps that when no
change applied to the input we don't need to output the data. This might
also help removing the TSINPUT_PRESSURE_TRIGGER define

Copyright (C) 2007,2008,2009,2010 TomTom BV <http://www.tomtom.com/>
Author: Kees Jongenburger <kees.jongenburger@tomtom.com>
        Jeroen Taverne <jeroen.taverne@tomtom.com>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 as
published by the Free Software Foundation.
*/

/*Normal kernel includes*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

/*TomTom specific includes */
#include <barcelona/Barc_adc.h>	/* The Analog digital converter */


/**
 * Declarations for the driver, the strings are used in different place one of
 * them in the /dev/input/event driver. Perhaps we need the define to move to an
 * include file so other libs can find the touchscreen by using the name of the
 * driver.
 **/
#define DRIVER_DESC_SHORT  "TomTom GO Touchscreen Input Driver"
#define DRIVER_DESC_LONG   "TomTom GO Touchscreen Input Driver, (C) 2007 TomTom BV"

/**structure for touchscreen specific data**/
struct tsinput {
	/* The structure for /dev/input/eventX */
	struct input_dev inputdev;
	/* Counter for the amount of times
	the device is read , this value is increased and decreased by
	the dev_input_event_open and close callbacks */
	int open_count;
	
	/**
	 * The kernel tasklet used in order to minimize the impact of
	 * the touchscreen diver interrupts
	 **/
	struct tasklet_struct adc_tasklet_handle;
	
	/*Adc_buffers with x,y and z (pressure)
	this data writen within the adc interrupts but read inside the tasklet
	to access the vars you should first acquire a lock */
	unsigned short adc_x;
	unsigned short adc_y;
	unsigned short adc_z;
	
	/* this code is copied from ts/ts.c to provide about the
	same input events as the old module provides, the calibration*/
	
	unsigned short averaged_x;
	unsigned short averaged_y;
	unsigned short last_x;
	unsigned short last_y;
	int pendown;
	
	/* Lock for syncrhonisation between adc interrupts and the tasklet */
	spinlock_t lock;
};

/*Define a tasklet method used to handle the adc data*/
static void tsinput_adc_tasklet(unsigned long dev_address);

/* function prototypes used by the dev_input_event_open/close callbacks */
static int dev_input_event_open_callback(struct input_dev *dev);
static void dev_input_event_close_callback(struct input_dev *dev);

/**
 * Callback for the ADC (analog to digital converter).
 * This method is called about 50 times a second
 **/
static void tsinput_adc_poll(short passedbuf[ADC_CHANNELS], void *arg)
{
	struct tsinput *ts_input =
	(struct tsinput *) ((struct device *) arg)->driver_data;

	/* Set last X and Y to large values at penup to force recalculation of average values */
	if (passedbuf[ADC_TS_DOWN] >= TSINPUT_PRESSURE_TRIGGER) {
		ts_input->last_x = 10000;
		ts_input->last_y = 10000;
	}
	
	/* Make sure the tasklet is executed at pendown, or till debounce has completed */
	if ((ts_input->pendown) || (passedbuf[ADC_TS_DOWN] < TSINPUT_PRESSURE_TRIGGER)) {
		spin_lock(&ts_input->lock);
		/* copy wanted adc data to the tsinput structure */
		ts_input->adc_x = passedbuf[ADC_TS_X];
		ts_input->adc_y = passedbuf[ADC_TS_Y];
		ts_input->adc_z = passedbuf[ADC_TS_DOWN];
		spin_unlock(&ts_input->lock);

		/* schedule the right tasklet */
		tasklet_schedule(&ts_input->adc_tasklet_handle);
	}
}

static void tsinput_adc_tasklet(unsigned long data)
{
	struct tsinput *ts_input =
	(struct tsinput *) ((struct device *) data)->driver_data;
	static int penup_count = 0;	
	int x,y,z;
	spin_lock(&ts_input->lock);
	x = ts_input->adc_x;
	y = ts_input->adc_y;
	z = ts_input->adc_z;
	spin_unlock(&ts_input->lock);
	
	/* Perform button up / down calculation, the default pressure
	is 1023 and goes down the harder we press */
	if (z < TSINPUT_PRESSURE_TRIGGER) {
		
		short diff_x = ts_input->last_x -x ;
		short diff_y = ts_input->last_y -y ;
		
		if (diff_x < 0){
			diff_x = -diff_x;
		}
		
		if (diff_y < 0){
			diff_y = -diff_y;
		}
		if (diff_x < 50 && diff_y < 50){
			ts_input->averaged_x = (ts_input->last_x  + x) >> 1;
			ts_input->averaged_y = (ts_input->last_y  + y) >> 1;

			/* Send pen down event */
			input_report_abs(&ts_input->inputdev, ABS_X, ts_input->averaged_x);
			input_report_abs(&ts_input->inputdev, ABS_Y, ts_input->averaged_y );
			input_report_abs(&ts_input->inputdev, ABS_PRESSURE, 1023);

			/*output to the /dev/input/x device */
			input_sync(&ts_input->inputdev);

			/* This tasklet is executed till debounce has completed */
			ts_input->pendown = 1;
			/* Set debounce to 1/10 sec */
			penup_count = ADC_RATE_TS / 10;
		}

		/* keep track of the last received value when the pen was down */
		ts_input->last_x = x;
		ts_input->last_y = y;
		
	} else {
		if (ts_input->pendown) {
			if (penup_count == 0) {
				/* Send pen up event with last stored known position */
				input_report_abs(&ts_input->inputdev, ABS_X,  ts_input->averaged_x);
				input_report_abs(&ts_input->inputdev, ABS_Y,  ts_input->averaged_y);
				input_report_abs(&ts_input->inputdev, ABS_PRESSURE,0);
				
				/*output to the /dev/input/x device */
				input_sync(&ts_input->inputdev);
	
				/* This tasklet isn't executed anymore till touchscreen is touched again */
				ts_input->pendown=0;
			} else penup_count--;
		}
	}
}

/*=============================================================================
dev_input_event code
=============================================================================*/
static int dev_input_event_open_callback(struct input_dev *dev)
{
	/*this method is just a callback not the implementation
	of the open method of the input_dev */
	int ret =0;
	
	//TODO:check is dev != NULL and driver_data != NULL??
	struct tsinput * ts_input = (struct tsinput *)dev->dev->driver_data;
	
	//increase the open count
	ts_input->open_count ++;
	
	if (ts_input->open_count ==1){
		/* We then register a callback from the adc also  giving the current device
		as user data */
		ret = adc_register_poll(tsinput_adc_poll, dev->dev, ADC_RATE_TS);
		if (ret != 0) {
			printk(KERN_INFO DRIVER_DESC_SHORT " unable to register  ADC pollfunc (%d)\n", ret);
			ts_input->open_count --;
			return -EBUSY;
		}
	}
	return 0;
};


void dev_input_event_close_callback(struct input_dev *dev){
	/*this method is just a callback not the implementation
	of the close method of the input_dev */
	
	//TODO:check is dev != NULL and driver_data != NULL??
	struct tsinput * ts_input = (struct tsinput *)dev->dev->driver_data;
	
	if (ts_input->open_count <=0){
		printk(KERN_ERR DRIVER_DESC_SHORT " input_event close called while the open_count ==%i\n",ts_input->open_count);
	}
	
	//decrease the open count
	ts_input->open_count --;
	
	//if this is the last open for this device unregister the adc callback
	if (ts_input->open_count ==0){
		/*stop the adc callback, note that this is a
		bit of a hack,tsinput_adc_poll is a bus-device wide function.
		while it should be device local */
		adc_unregister_poll(tsinput_adc_poll);
	}
};


/*=============================================================================
Start class and bus driver code
=============================================================================*/
static int tsinput_probe(struct device *dev)
{
	struct tsinput *ts_input;
	
	/*allocate, clean and initialize the main structure */
	if (!(ts_input = kmalloc(sizeof(struct tsinput), GFP_KERNEL))) {
		/*make sure we don't have dangling pointers */
		dev->driver_data = NULL;
		return -ENOMEM;
	}
	memset(ts_input, 0, sizeof(struct tsinput));
	
	/*initialize the input device capabilities */
	init_input_dev(&ts_input->inputdev);
	
	ts_input->pendown = 0;
	ts_input->open_count = 0;
	
	/*The Touchscreen reports absolute positioning events, we
	*will also use the pressure information to generate button events*/
	ts_input->inputdev.evbit[0] = BIT(EV_ABS) | BIT(EV_KEY);
	ts_input->inputdev.absbit[0] =
	BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	ts_input->inputdev.keybit[LONG(BTN_TOUCH)] = BIT(BTN_TOUCH);
	
	/*put the default values for min and max */
	/*input_set_abs_params(int axis, int min, int max, int fuzz, int flat);
	normal values for an unpressed x ==0 ,y ==1023 and pressure  1023 */
	input_set_abs_params(&ts_input->inputdev, ABS_X, 0, 1023, 0, 0);
	input_set_abs_params(&ts_input->inputdev, ABS_Y, 0, 1023, 0, 0);
	input_set_abs_params(&ts_input->inputdev, ABS_PRESSURE, 0, 1023, 0, 0);
	
	ts_input->inputdev.open = dev_input_event_open_callback;
	ts_input->inputdev.close = dev_input_event_close_callback;
	
	/*TODO:convert the button using fuzz + flat so we don't need to code that
	inside the driver */
	
	ts_input->inputdev.name = DRIVER_DESC_SHORT;
	
	/* TODO:keesj 03-2007 find out what the phys member is used for
	perhaps to be able to find the right device in the /dev/input/ map? */
	ts_input->inputdev.phys = "tsinput/input0";
	
	ts_input->inputdev.dev = dev;
	
	input_register_device(&ts_input->inputdev);
	
	/*Store the input data into the struct driver data so we can find it back
	when needed , the way back from input_dev(ts_input) to to dev can be done
	using ts_input->dev. does this need to be set by hand??*/
	dev->driver_data = ts_input;
	
	/* initialize the spinlock at this point */
	spin_lock_init(&ts_input->lock);
	
	/**
	 * The adc will call the tsinput_adc_poll many times,
	 * the adc poll in it's turn will use tasklet_schedule
	 * method to schedule the data to be pushed. the tasklet_schedule requires an
	 * initialized tasklet_handler so we first initialize the tasklet structure
	 * with as data the current dev
	 **/
	tasklet_init(&ts_input->adc_tasklet_handle, tsinput_adc_tasklet,
	(unsigned long) dev);
	
	/*NOTE: we do not register the adc here, this happens when the first devices
	is opened */
	
	return 0;
}

/**
 * Called when the device is removed from the system
 **/
static int tsinput_remove(struct device *dev)
{
	/*get the driver data from the device */
	struct tsinput *ts_input = (struct tsinput *) dev->driver_data;

	/*First stop any pending tasklets requests */
	tasklet_disable(&ts_input->adc_tasklet_handle);
	
	/*Remove the actual /dev/input device */
	input_unregister_device(&ts_input->inputdev);
	

	
	/*free the memory used by the tsinput structure allocated in probe*/
	kfree(ts_input);
	return 0;
};

/*This driver registers a tomtomgo-tsinput "Touchscreen input" bus type driver*/
static struct device_driver tsinput_bus_driver = {
	.name = "tomtomgo-tsinput",
	.bus = &platform_bus_type,
	.probe = tsinput_probe,
	.remove = tsinput_remove,
};

/**
 * Touchscreen module initialization routine for the linux kernel.
 * This method registers a bus driver for tomtomgo-tsinput, it does not
 * register to the adc or create drivers
 **/
static int __init tsinput_init_module(void)
{
	int ret;
	/*display driver information when the driver is started */
	printk(KERN_INFO DRIVER_DESC_LONG "\n");
	
	ret = driver_register(&tsinput_bus_driver);
	if (ret) {
		printk(KERN_ERR DRIVER_DESC_SHORT " unable to register driver (%d)\n", ret);
		return ret;
	}
	printk(KERN_DEBUG "Done\n");
	return 0;
}

static void __exit tsinput_exit_module(void)
{
	driver_unregister(&tsinput_bus_driver);
	printk(KERN_INFO DRIVER_DESC_SHORT " exit\n");
}


module_init(tsinput_init_module);
module_exit(tsinput_exit_module);

MODULE_DESCRIPTION(DRIVER_DESC_LONG);
MODULE_AUTHOR("Kees Jongenburger (kees.jongenburger@tomtom.com)");
MODULE_LICENSE("GPL");
