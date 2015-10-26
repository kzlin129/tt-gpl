/*
 * Include file for the power button device.
 *
 * Author: Mark Vels <mark.vels@tomtom.com>
 *  * (C) Copyright 2008 TomTom International BV.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/stringify.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <asm/atomic.h>
#include <linux/apm_bios.h>
#include <linux/apm-emulation.h>
#include <linux/reboot.h>

#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/sysfs.h>
#include <linux/powerbutton.h>

#ifdef CONFIG_PLAT_IRVINE
#include <plat/tt_vbusmon.h>
#endif

#include <linux/syscalls.h>

/* Defines */
//#define PB_DEBUG 1
#define PFX "pb: "
#define DRIVER_DESC_LONG "TomTom Power Button driver, (C) 2008 TomTom BV "

#define PB_ERR(fmt, args...) \
	printk(KERN_ERR PFX "##ERROR## :" fmt "\n", ##args )

#ifdef PB_DEBUG
#define PB_DBG(fmt, args...) printk( KERN_DEBUG PFX fmt "\n", ##args )
#else
#define PB_DBG(fmt, args...)
#endif

#define RESUME_DEBOUNCE_TIME	(HZ*500/1000)		/*  500 msecs in jiffies */
#define ACTION_DEBOUNCE_TIME	(HZ*500/1000)		/*  400 msecs in jiffies */
#define DEBOUNCE_POLL		(HZ*50/1000)		/*   50 msecs in jiffies */

/* power button confirm */
#define PWB_CONFIRM_STEP        40     			/* Power button confirm step in ms */

struct powerbutton_data {
	int irq;

	atomic_t enable;
	atomic_t action;

	atomic_t resume_debounce_state;
	atomic_t resume_debounce_time_remaining;
	struct timer_list resume_debounce;

	atomic_t action_debounce_state;
	atomic_t action_debounce_time_remaining;
	struct timer_list action_debounce;	

	struct powerbutton_pdata *pdata;
#ifdef CONFIG_PLAT_IRVINE
	struct notifier_block notifier;
#endif
	char usb_is_on;
};

struct powerbutton_data pb_data;

static void pb_suicide (struct work_struct *w);
static DECLARE_WORK (suicide_worker, pb_suicide);
static DECLARE_COMPLETION(debounce);

#define PB_ACTION_SUSPEND 0
#define PB_ACTION_SUICIDE 1

static char *action_strs[] = {
	"suspend",
	"suicide",
	NULL
};

void pb_action_debounce_timeout_func (unsigned long data_pointer);

ssize_t pb_state_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t pb_cold_debounce_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t pb_resume_debounce_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t pb_enable_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t pb_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t pb_action_store (struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t pb_action_show (struct device *dev, struct device_attribute *attr, char *buf);
ssize_t pb_early_resume_state_show (struct device *dev, struct device_attribute *attr, char *buf);
ssize_t pb_usbstat_show(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(pb_action, 0644, pb_action_show, pb_action_store);
static DEVICE_ATTR(pb_enable, 0644, pb_enable_show, pb_enable_store);
static DEVICE_ATTR(pb_state, S_IRUGO, pb_state_show, NULL);
static DEVICE_ATTR(pb_cold_debounce_state, S_IRUGO, pb_cold_debounce_show, NULL);
static DEVICE_ATTR(pb_resume_debounce_state, S_IRUGO, pb_resume_debounce_show, NULL);
static DEVICE_ATTR(pb_early_resume_state, S_IRUGO, pb_early_resume_state_show, NULL);
static DEVICE_ATTR(pb_usbstat, S_IRUGO, pb_usbstat_show, NULL);

ssize_t pb_action_store (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rc, i;

	PB_DBG("Got sysfs write request for device@ 0x%x, attr@ 0x%x", dev, attr);
	if (dev == NULL) {
		PB_ERR("DEV is null!");
		return -EINVAL;
	}

	rc = -EINVAL;

	for (i = 0; action_strs[i]; i++)
	{
		if (strncmp(action_strs[i], buf, strlen(action_strs[i])) == 0) {
			atomic_set (&pb_data.action, i);
			rc = count;
			break;
		}
	}

	return rc;
}

ssize_t pb_action_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	int action;

	PB_DBG("Got sysfs read request for device@ 0x%x, attr@ 0x%x", dev, attr);
	if (dev == NULL) {
		PB_ERR("DEV is null!");
		return -EINVAL;
	}

	action = atomic_read (&pb_data.action);
	snprintf (buf, PAGE_SIZE, "%s", action_strs[action]);

	return strlen(buf)+1;
}

ssize_t pb_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;
	size_t size;
	int val, rc;

	PB_DBG("Got sysfs write request for device@ 0x%x, attr@ 0x%x", dev, attr);
	if (dev == NULL) {
		PB_ERR("DEV is null!");
		return -EINVAL;
	}

	val = simple_strtoul(buf, &endp, 0);
	size = endp - buf;
	rc = -ENXIO;

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	val = simple_strtoul(buf, &endp, 0);
	if (val != 0 && val != 1)
		return rc;

	atomic_set (&pb_data.enable, val);
	rc = count;

	return rc;
}

ssize_t pb_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val;

	PB_DBG("Got sysfs read request for device@ 0x%x, attr@ 0x%x", dev, attr);
	if (dev == NULL) {
		PB_ERR("DEV is null!");
		return -EINVAL;
	}

	val = atomic_read(&pb_data.enable);

	snprintf(buf, PAGE_SIZE, "%d", val);

	return strlen(buf)+1; 
}

ssize_t pb_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct powerbutton_pdata *pdata;
	int val;

	PB_DBG("Got sysfs read request for device@ 0x%x, attr @ 0x%x", dev, attr);
	if( dev == NULL){
		PB_ERR("DEV is null!");
		return -EINVAL;
	} 

	pdata = (struct powerbutton_pdata *) dev->platform_data;
	if( pdata == NULL){
		PB_ERR("PDATA is null!");
		return -EINVAL;
	}

	PB_DBG("Reading pwr button file\n");

	val = pdata->get_value();
	snprintf (buf, PAGE_SIZE, "%d", val);

	if (val) {
		PB_DBG("Power button is pressed\n");
	} else {
		PB_DBG("Power button is not pressed\n");
	}

	/* return number of characters +1 for trailing '\0'*/
	return strlen(buf)+1; 
}

ssize_t pb_cold_debounce_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct powerbutton_pdata *pdata;
	int val;

	PB_DBG("Got sysfs read request for device@ 0x%x, attr @ 0x%x", dev, attr);
	if (dev == NULL) {
		PB_ERR("DEV is null!");
		return -EINVAL;
	} 

	pdata = (struct powerbutton_pdata *) dev->platform_data;
	if (pdata == NULL) {
		PB_ERR("PDATA is null!");
		return -EINVAL;
	}

	PB_DBG("Reading pwr button file\n");

	val = pdata->get_timed_value();	

	snprintf (buf, PAGE_SIZE, "%d", val);

	if (val) {
		PB_DBG("Power button is pressed\n");
	} else {
		PB_DBG("Power button is not pressed\n");
	}

	/* return number of characters +1 for trailing '\0'*/
	return strlen(buf)+1; 
}


ssize_t pb_resume_debounce_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct powerbutton_pdata *pdata;
	int val;

	PB_DBG("Got sysfs read request for device@ 0x%x, attr @ 0x%x", dev, attr);
	if (dev == NULL) {
		PB_ERR("DEV is null!");
		return -EINVAL;
	} 

	pdata = (struct powerbutton_pdata *) dev->platform_data;
	if( pdata == NULL){
		PB_ERR("PDATA is null!");
		return -EINVAL;
	}

	PB_DBG("Reading pwr button file\n");

	val = atomic_read (&pb_data.resume_debounce_time_remaining);
	if (val == 0) {
		/* We don't need to wait because the value is zero, meaning: 
		   the button has been released */
		val = atomic_read (&pb_data.resume_debounce_state);
	} else {
		/* Wait for the debounce time to finish */
		wait_for_completion (&debounce);
		val = atomic_read (&pb_data.resume_debounce_state);
	}

	snprintf (buf, PAGE_SIZE, "%d", val);

	if (val){
		PB_DBG("Power button is pressed\n");
	} else {
		PB_DBG("Power button is not pressed\n");
	}

	/* return number of characters +1 for trailing '\0'*/
	return strlen(buf)+1; 
}



ssize_t pb_early_resume_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct powerbutton_pdata *pdata;

	if( dev == NULL){
		PB_ERR("DEV is null!");
		return -EINVAL;
	} 

	pdata = (struct powerbutton_pdata *) dev->platform_data;
	if( pdata == NULL){
		PB_ERR("PDATA is null!");
		return -EINVAL;
	}

	PB_DBG("Reading pwr button file\n");

	snprintf (buf, PAGE_SIZE, "%d", pdata->resume_state);

	if (pdata->resume_state) {
		PB_DBG("Resumed from power button\n");
	} else {
		PB_DBG("Not resumed from power button\n");
	}

	/* return number of characters +1 for trailing '\0'*/
	return strlen(buf)+1; 
}

ssize_t pb_usbstat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	PB_DBG("%d", pb_data.usb_is_on);
	snprintf(buf, PAGE_SIZE, "%d", pb_data.usb_is_on);
	return strlen(buf)+1;
}

#ifdef CONFIG_PLAT_IRVINE
static int pb_usbstat_state_change( struct notifier_block *self, unsigned long vbus_on, void *arg )
{
	pb_data.usb_is_on	= (vbus_on == 0 ? 0 : 1);
	return 0;	
}
#endif

/**
 * power_button_confirm - remove button bounces
 * returns: 1 - real button press, 0 - not a real button press
 */
static int power_button_confirm(void)
{
	struct powerbutton_pdata *pdata;
	int reg, pwb_held = 1;

	pdata = pb_data.pdata;
	mdelay(PWB_CONFIRM_STEP);

	reg = pdata->get_value();

	if (!reg)
		pwb_held = 0;

	PB_DBG("reg ! %d,%s,%s,line=%d\n",reg,__FILE__,__FUNCTION__,__LINE__);

	return pwb_held;
}

/*
 * pm_irq - Intrupt handler
 * 
 * Copied from static irqreturn_t at4x0a_pm_irq(int irq, void *pdata)
*/
static irqreturn_t powerbutton_irq(int irq, void *data)
{
	struct powerbutton_pdata *pdata;
	unsigned int reg, enabled;

	pdata = pb_data.pdata;

	reg = pdata->get_value();

	PB_DBG("ATE: power-button : getting interrupt on level %d\n", irq);
	PB_DBG("Power Button. line=%d\n",__LINE__);

	PB_DBG("reg  %d,%s,%s,line=%d\n",reg,__FILE__,__FUNCTION__,__LINE__);

	/* We are the button was released return */
	if (!reg)
		return IRQ_HANDLED;

	if ((!timer_pending(&pb_data.action_debounce) || !timer_pending(&pb_data.resume_debounce)) && power_button_confirm())
	{
		enabled = atomic_read(&pb_data.enable);
		if (!enabled){
			return IRQ_HANDLED;
		}
		
		PB_DBG ("Starting action poll\n");

		atomic_set (&pb_data.action_debounce_time_remaining, ACTION_DEBOUNCE_TIME);
		atomic_set (&pb_data.action_debounce_state, 1);	// We have to start with 1, otherwise the first test will fail
		pb_data.action_debounce.function = pb_action_debounce_timeout_func;

		mod_timer(&pb_data.action_debounce, jiffies + 1);
	}

	return IRQ_HANDLED;
}

static int pb_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	printk (DRIVER_DESC_LONG "\n");

	pb_data.pdata = (struct powerbutton_pdata *) pdev->dev.platform_data;
	pb_data.usb_is_on	= 0;

#ifdef CONFIG_PLAT_IRVINE
	pb_data.notifier.notifier_call = pb_usbstat_state_change;
#endif

	res = platform_get_resource( pdev, IORESOURCE_IRQ, 0 );
	pb_data.irq = res->start;

	ret = request_irq(pb_data.irq, powerbutton_irq, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, pdev->name, &pdev->dev );
	if (ret) {
		PB_ERR("Could not request IRQ: %d\n", pb_data.irq);
		goto err_irq;
	}

	/*
	 * Wakeup event can only be level high at the input of the PML
	 * As the defult level on Otavalo for the PONKEY is high this will not work.
	 */
	enable_irq_wake(pb_data.irq);

	ret = device_create_file(&pdev->dev, &dev_attr_pb_state);
	if (ret){
		PB_ERR("Failed to create sysfs file for state attribute\n");
		goto err_sysfs;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_pb_cold_debounce_state);
	if (ret) {
		PB_ERR("Failed to create sysfs file for cold_debounce_state attribute\n");
		goto err_sysfs_cold_debounce_state;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_pb_resume_debounce_state);
	if (ret) {
		PB_ERR("Failed to create sysfs file for resume_debounce_state attribute\n");
		goto err_sysfs_resume_debounce_state;
	} 

	ret = device_create_file(&pdev->dev, &dev_attr_pb_enable);
	if (ret) {
		PB_ERR("Failed to create sysfs file for enable attribute\n"); 
		goto err_sysfs_enable;
	}

	ret = device_create_file (&pdev->dev, &dev_attr_pb_action);
	if (ret) {
		PB_ERR("Failed to create sysfs file for action attribute\n"); 
		goto err_sysfs_action;
	}

	ret = device_create_file (&pdev->dev, &dev_attr_pb_early_resume_state);
	if (ret) {
		PB_ERR("Failed to create sysfs file for action attribute\n"); 
		goto err_sysfs_action;
	}

	ret = device_create_file (&pdev->dev, &dev_attr_pb_usbstat);
	if (ret) {
		PB_ERR("Failed to create sysfs file for action attribute\n"); 
		goto err_sysfs_action;
	}

        init_timer(&pb_data.resume_debounce);
	init_timer(&pb_data.action_debounce);

	atomic_set (&pb_data.enable, 0);
	atomic_set (&pb_data.action, PB_ACTION_SUICIDE);
	atomic_set (&pb_data.action_debounce_state, 0);
	atomic_set (&pb_data.resume_debounce_state, 0);
	atomic_set (&pb_data.action_debounce_time_remaining, 0);
	atomic_set (&pb_data.resume_debounce_time_remaining, 0);

#ifdef CONFIG_PLAT_IRVINE
	/* Register to vbusmon. This has to be done here in order to avoid ordering issues with the init above */
	vbusmon_register_notifier(&pb_data.notifier);
	PB_DBG("at this point, action is %d\n", atomic_read (&pb_data.action));
#endif

	return 0;

err_sysfs_action:
	device_remove_file(&pdev->dev, &dev_attr_pb_action);

err_sysfs_enable:
	device_remove_file(&pdev->dev, &dev_attr_pb_enable);

err_sysfs_resume_debounce_state:
	device_remove_file(&pdev->dev, &dev_attr_pb_cold_debounce_state);

err_sysfs_cold_debounce_state:
	device_remove_file(&pdev->dev, &dev_attr_pb_state);

err_sysfs:
	free_irq(pb_data.irq, &pdev->dev);

err_irq:

	return ret;
}
 
static int pb_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_pb_state );
	device_remove_file(&pdev->dev, &dev_attr_pb_cold_debounce_state );
	device_remove_file(&pdev->dev, &dev_attr_pb_resume_debounce_state );

	free_irq(pb_data.irq, &pdev->dev);

	return 0;
}


void pb_suicide (struct work_struct *w)
{
	struct powerbutton_pdata *pdata;
	pdata = pb_data.pdata;

	sys_sync();
	msleep(500);

	while (pdata->get_value() == 1) {
                mdelay(200);
        }

	lock_kernel();
	kernel_power_off();
	unlock_kernel();
}

void pb_action_debounce_timeout_func (unsigned long data_pointer)
{
	struct powerbutton_pdata *pdata;
	int tmp, tmptime;
	unsigned int action;

	pdata = pb_data.pdata;

	tmp = atomic_read (&pb_data.action_debounce_state);
	tmptime = atomic_read (&pb_data.action_debounce_time_remaining);

	if (tmp) {
		tmp = pdata->get_value();
		tmptime = tmptime - DEBOUNCE_POLL;

		atomic_set (&pb_data.action_debounce_time_remaining, tmptime);
		atomic_set (&pb_data.action_debounce_state, tmp);

		if (tmptime <= 0) {
			action = atomic_read (&pb_data.action);
	
			if (action == PB_ACTION_SUSPEND) {
				apm_queue_event(APM_USER_SUSPEND);
			} else if (action == PB_ACTION_SUICIDE) { 
				schedule_work (&suicide_worker);
			}

			return;
		}

		mod_timer (&pb_data.action_debounce, jiffies + DEBOUNCE_POLL);
		return;
	}

	atomic_set (&pb_data.action_debounce_time_remaining, 0);
	atomic_set (&pb_data.action_debounce_state, 0);
}

void pb_resume_debounce_timeout_func (unsigned long data_pointer)
{
	struct powerbutton_pdata *pdata;
        int tmp, tmptime;

	pdata = pb_data.pdata;

        tmp = atomic_read (&pb_data.resume_debounce_state);
	tmptime = atomic_read (&pb_data.resume_debounce_time_remaining);

        if (tmp) {
                tmp = pdata->get_value();
		tmptime = tmptime - DEBOUNCE_POLL;

		atomic_set (&pb_data.resume_debounce_time_remaining, tmptime);
        	atomic_set (&pb_data.resume_debounce_state, tmp);

		if (tmptime <= 0) {
			goto complete;
		}

		mod_timer(&pb_data.resume_debounce, jiffies + DEBOUNCE_POLL);
		return;
        }

	atomic_set(&pb_data.resume_debounce_time_remaining, 0);
	atomic_set(&pb_data.resume_debounce_state, tmp);

complete:
	complete(&debounce);
}


#ifdef CONFIG_PM
static int pb_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct powerbutton_pdata *pdata;

	pdata = (struct powerbutton_pdata *)pdev->dev.platform_data;

	while (pdata->get_value() == 1) {
                pr_info("Waiting for pb to go off...\n");
                mdelay(200);
        }

	return 0;
}
 
static int pb_resume_early (struct platform_device *pdev)
{
	struct powerbutton_pdata *pdata;
	pdata = (struct powerbutton_pdata *)pdev->dev.platform_data;
	pdata->resume_state = pdata->get_value();

	return 0;
}

static int pb_resume(struct platform_device *dev)
{
	pb_data.resume_debounce.function = pb_resume_debounce_timeout_func;

	atomic_set (&pb_data.resume_debounce_time_remaining, RESUME_DEBOUNCE_TIME);
	atomic_set (&pb_data.resume_debounce_state, 1);	// We have to start with 1, otherwise the first test will fail

	mod_timer (&pb_data.resume_debounce, jiffies + 1);

	return 0;
}
#else
#define pb_suspend	NULL
#define pb_resume	NULL
#define pb_resume_early	NULL
#endif

static struct platform_driver pb_driver = {
	.driver		= {
		.name	= "powerbutton", /* WARNING, the platform device is shared with drivers/tomtom/pb/pb.c! */
	},
	.probe		= pb_probe,
	.remove		= pb_remove,
	.suspend	= pb_suspend,
	.resume		= pb_resume,
	.resume_early   = pb_resume_early,
};
 
static int __init pb_init (void)
{
	int ret;

	ret = platform_driver_register(&pb_driver);
	if(ret) {
		PB_ERR("Could not register pb platform driver! Error=%d", ret);
	} 

	return ret;
}

static void __exit pb_exit (void)
{
	platform_driver_unregister(&pb_driver);
	PB_DBG("Unregistered pb platform driver.");

}

module_init (pb_init);
module_exit (pb_exit);

MODULE_DESCRIPTION(DRIVER_DESC_LONG);
MODULE_AUTHOR("Mark Vels");
MODULE_LICENSE("GPL");
