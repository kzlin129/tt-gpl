/*
 * 
 *
 *
 *  Copyright (C) 2008 by TomTom International BV. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation.
 *
 *
 *
 *
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/syscalls.h>
#include <linux/apm_bios.h>
#include <linux/apm-emulation.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <asm/atomic.h>
#include <plat/tilt_sensor.h>

//#define TILT_DEBUG 0
#define PFX "tl: "
#ifdef TILT_DEBUG
#define TILT_DBG(fmt, args...) printk(KERN_DEBUG PFX fmt "\n", ##args )
#else
#define TILT_DBG(gmt, args...)
#endif

/* debounce times are all in milliseconds */
#define MIN_DEBOUNCE_TIME	( 10)
#define MAX_DEBOUNCE_TIME	(500)
#define DEFAULT_DEBOUNCE_TIME	(100)
#define DEBOUNCE_TIMER_TIME	( 10)

#define DEBOUNCE_TIMER_INTERVAL		msecs_to_jiffies(DEBOUNCE_TIMER_TIME)
#define DEFAULT_DEBOUNCE_INTERVAL	msecs_to_jiffies(DEFAULT_DEBOUNCE_TIME)

static int tilt_state, tilt_debounced_state;
static unsigned int tilt_debounce_interval;
static int debouncing = 0;
static int new_state = -1;
static struct timer_list debounce_timer;
static unsigned int tilt_raw; /* for get raw GPIOs states */

/* spinlock protects above state and pd-> operations */
static DEFINE_SPINLOCK(tilt_lock);

static int tilt_debounce(int * raw_state, int * cur_state, int * debouncing);
static void stop_timer(void);
static void start_timer(void);

static void timer_callback(unsigned long arg)
{
	unsigned long flags;

	spin_lock_irqsave(&tilt_lock, flags);

	if (tilt_debounce( &tilt_state, &tilt_debounced_state, &debouncing ))
		start_timer(); /* not debounced yet, keep going */
	else
		stop_timer();

	spin_unlock_irqrestore(&tilt_lock, flags);
}

static void start_timer(void)
{
	init_timer( &debounce_timer );
	debounce_timer.expires = jiffies + DEBOUNCE_TIMER_INTERVAL;
	debounce_timer.function = timer_callback;
	debounce_timer.data = (unsigned long)0;
	add_timer( &debounce_timer );
}

static void stop_timer(void)
{
	del_timer( &debounce_timer );
}

/* returns 1 if debouncing, 0 if state is stable */
static int tilt_debounce(int * raw_state, int * cur_state, int * debouncing)
{
	static int count;
	*debouncing=1;

	TILT_DBG("raw_state=%d, cur_state=%d, count=%d", *raw_state, *cur_state, count);

	if (*raw_state == new_state)
	{
		if (count <= 0)
		{
			*cur_state = *raw_state;
			TILT_DBG("debounced (cur_state=%d)", *cur_state);
			*debouncing=0; /* state changed */
		} else {
			count--;
		}
	}
	else
	{
		new_state = *raw_state;
		count = tilt_debounce_interval / DEBOUNCE_TIMER_INTERVAL;
		printk(KERN_INFO PFX "new tilt state=%d, debounce count=%d (was stable for %ld/%ld)\n",
				new_state,
				count, 
				(tilt_debounce_interval / DEBOUNCE_TIMER_INTERVAL)-count, 
				(tilt_debounce_interval / DEBOUNCE_TIMER_INTERVAL));
	}

	return *debouncing;
}

/* Userland sysfs interface */
static ssize_t store_tilt_power(struct device *dev,
				struct device_attribute *attr,
				const char *buffer, size_t size)
{
	struct tilt_sensor_platform_data *pd = dev->platform_data;
	unsigned long flags;

	/* Check the size of the buffer, if> 1, second character should be '\n' */
	switch (size) {
	case 1:
		break;
	case 2:
		if (buffer[1] != '\n')
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&tilt_lock, flags);

	/* Power. Should match the entry power in the gprs attributes structure. */
	switch (buffer[0]) {
	case '0':
		pd->tilt_power_set(0);
		break;
	case '1':
		pd->tilt_power_set(1);
		break;
	default:
		return -EINVAL;
	}

	spin_unlock_irqrestore(&tilt_lock, flags);

	return 1;
}

/* Userland sysfs interface */
static ssize_t show_tilt_power(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int tilt_power = 0;
	struct tilt_sensor_platform_data *pd = dev->platform_data;
	unsigned long flags;

	spin_lock_irqsave(&tilt_lock, flags);
	tilt_power = pd->tilt_power_get();
	spin_unlock_irqrestore(&tilt_lock, flags);

	snprintf(buf, PAGE_SIZE, "%d\n", tilt_power);
	return strlen(buf) + 1;
}

/* Userland sysfs interface */
static ssize_t show_tilt_state(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	int state;

	spin_lock_irqsave(&tilt_lock, flags);
	state = tilt_state;
	spin_unlock_irqrestore(&tilt_lock, flags);

	snprintf(buf, PAGE_SIZE, "%d\n", state);
	return strlen(buf) + 1;
}

/* Userland sysfs interface */
static ssize_t store_tilt_debounce_time(struct device *dev,
					struct device_attribute *attr,
					const char *buffer, size_t size)
{
	unsigned long flags;
	char *endp;
	long debounce_time = simple_strtol(buffer, &endp, 10);
	if (debounce_time < MIN_DEBOUNCE_TIME || debounce_time > MAX_DEBOUNCE_TIME)
	{
		printk(KERN_ERR " debounce_time %ld msec out of range (min=%d, max=%d, current=%d)\n",
				debounce_time, 
				MIN_DEBOUNCE_TIME, 
				MAX_DEBOUNCE_TIME, 
				jiffies_to_msecs(tilt_debounce_interval));
		return -EINVAL;
	}

	spin_lock_irqsave(&tilt_lock, flags);
	tilt_debounce_interval = msecs_to_jiffies(debounce_time);
	spin_unlock_irqrestore(&tilt_lock, flags);

	printk(KERN_INFO PFX "new debounce_time = %ld msec (min=%d, max=%d)\n",
				debounce_time, 
				MIN_DEBOUNCE_TIME, 
				MAX_DEBOUNCE_TIME);
	return size;
}

/* Userland sysfs interface */
static ssize_t show_tilt_debounce_time(struct device *dev,
			               struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	unsigned int interval;

	spin_lock_irqsave(&tilt_lock, flags);
	interval = tilt_debounce_interval;
	spin_unlock_irqrestore(&tilt_lock, flags);

	snprintf(buf, PAGE_SIZE, "%d\n", jiffies_to_msecs(interval));
	return strlen(buf) + 1;
}

/* Userland sysfs interface */
static ssize_t show_tilt_debounced_state(struct device *dev,
			                 struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	int state;

	spin_lock_irqsave(&tilt_lock, flags);
	state = tilt_debounced_state;
	spin_unlock_irqrestore(&tilt_lock, flags);

	snprintf(buf, PAGE_SIZE, "%d\n", state);
	return strlen(buf) + 1;
}

/* Userland sysfs interface */
static ssize_t show_tilt_raw(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct tilt_sensor_platform_data *pd = dev->platform_data;
	unsigned int raw = 0;
	
	if(pd->tilt_raw)
		raw = pd->tilt_raw();

	snprintf(buf, PAGE_SIZE, "%d\n", raw);
	return strlen(buf) + 1;
}

DEVICE_ATTR(tilt_power          , S_IRUGO | S_IWUGO, show_tilt_power          , store_tilt_power);
DEVICE_ATTR(tilt_state          , S_IRUGO          , show_tilt_state          , NULL);
DEVICE_ATTR(tilt_debounce_time  , S_IRUGO | S_IWUGO, show_tilt_debounce_time  , store_tilt_debounce_time);
DEVICE_ATTR(tilt_debounced_state, S_IRUGO          , show_tilt_debounced_state, NULL);
DEVICE_ATTR(tilt_raw	        , S_IRUGO          , show_tilt_raw          , NULL);

/* Set up main userland sysfs interface */
int inline setup_userland_interface(struct platform_device *pdev)
{
	if (device_create_file(&pdev->dev, &dev_attr_tilt_power)) {
		dev_err(&pdev->dev,
			"Could not create tilt_power sysfs attribute\n");
		return -1;
	}

	if (device_create_file(&pdev->dev, &dev_attr_tilt_state)) {
		dev_err(&pdev->dev,
			"Could not create tilt_state sysfs attribute\n");
		return -1;
	}

	if (device_create_file(&pdev->dev, &dev_attr_tilt_debounce_time)) {
		dev_err(&pdev->dev,
			"Could not create tilt_debounce_time sysfs attribute\n");
		return -1;
	}

	if (device_create_file(&pdev->dev, &dev_attr_tilt_debounced_state)) {
		dev_err(&pdev->dev,
			"Could not create tilt_debounced_state sysfs attribute\n");
		return -1;
	}
	
	if (device_create_file(&pdev->dev, &dev_attr_tilt_raw)) {
		dev_err(&pdev->dev,
			"Could not create tilt_raw sysfs attribute\n");
		return -1;
	}

	return 0;
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	struct platform_device *pdev;
	struct tilt_sensor_platform_data *pd;
	int debounce;

	pdev = (struct platform_device *)dev_id;
	pd = pdev->dev.platform_data;

	spin_lock(&tilt_lock);
	tilt_state = pd->tilt_value();
	debounce = debouncing;
	spin_unlock(&tilt_lock);

	if (!debounce) {
	        timer_callback(0);
	} else {
		dev_info(&pdev->dev, "Already debouncing...\n");
	}

	return IRQ_HANDLED;
}

/* note that the order below is such that locking is NOT needed */
static int tilt_sensor_probe(struct platform_device *pdev)
{
	struct resource *irqres[1];
	int res_index, irq_index;
	int res = 0;
	struct tilt_sensor_platform_data *pd = pdev->dev.platform_data;

	/* config gpio for tilt control*/
	pd->config_gpio();

	for(res_index = 0, pd->irq_max = 0; res_index < pdev->num_resources; res_index++)
	{
		if(pdev->resource[res_index].flags == IORESOURCE_IRQ)
		{
			pd->irq_max++;
			BUG_ON(pd->irq_max > TILT_SENSOR_MAX_IRQ);
		}
		else
			continue;
	
		irq_index = pd->irq_max-1;
		/* Get IRQ */
		irqres[0] = platform_get_resource(pdev, IORESOURCE_IRQ, res_index);
		if (!irqres[0]) {
			dev_err(&pdev->dev, "Missing IRQ resources[%d]!\n", irq_index);
			return -EINVAL;
		}

		/* Register the tilt_out as interrupt source */
		pd->irq[irq_index] = irqres[0]->start;
		res = request_irq(pd->irq[irq_index], irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			pdev->resource[res_index].name, pdev);
		if (res) {
			dev_err(&pdev->dev, "Can't register TILT_OUT interrupt %d!\n", pd->irq[irq_index]);
			return res;
		}
	}
	
	/* Power up tilt */
	pd->tilt_power_set(1);

	/* get initial value */
	tilt_debounced_state = tilt_state = pd->tilt_value();
	dev_info(&pdev->dev, "Initial value %d.\n", tilt_state);

	/* Set up userland interface */
	setup_userland_interface(pdev);

	return 0;
}

#ifdef CONFIG_PM
static int tilt_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;
	struct tilt_sensor_platform_data *pd = pdev->dev.platform_data;

	for(i = 0; i < pd->irq_max; i++)
		disable_irq(pd->irq[i]);

	del_timer_sync(&debounce_timer);

	/* make sure the device is off on suspend so it doesn't use power */
	pd->tilt_power_set(0);

	return 0;
}

static int tilt_resume(struct platform_device *pdev)
{
	int i;
	struct tilt_sensor_platform_data *pd = pdev->dev.platform_data;

	pd->tilt_power_set(1);

	/* clear state and simulate irq */
	tilt_state = pd->tilt_value();
	debouncing = 0;
	new_state = -1;
	timer_callback(0);

	for(i = 0; i < pd->irq_max; i++)
		enable_irq(pd->irq[i]);
	    
	dev_info(&pdev->dev, "Initial value after resume %d.\n", tilt_state);

	return 0;
}
#endif

static struct platform_driver tilt_sensor_driver = {
	.probe = tilt_sensor_probe,
#ifdef CONFIG_PM
	.suspend = tilt_suspend,
	.resume = tilt_resume,
#endif
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "tilt_sensor",
		   },
};

static int __init tilt_sensor_init_module(void)
{
	int ret;

	tilt_debounce_interval = DEFAULT_DEBOUNCE_INTERVAL;

	ret = platform_driver_register(&tilt_sensor_driver);
	if (ret) {
		printk(KERN_ERR " Unable to register Tilt Sensor driver (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit tilt_sensor_exit_module(void)
{
	platform_driver_unregister(&tilt_sensor_driver);
}

module_init(tilt_sensor_init_module);
module_exit(tilt_sensor_exit_module);

MODULE_DESCRIPTION("TomTom Tilt Sensor Driver");
MODULE_LICENSE("GPL");
