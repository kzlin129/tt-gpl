/*
 *  drivers/mfd/ltc3577-pmic.c
 *
 *  Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 *  Author: Andrzej Zukowski <andrzej.zukowski@tomtom.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mfd/ltc3577/core.h>
#include <linux/mfd/ltc3577/pmic.h>
#include <plat/usbmode.h>

/*
 * Conversion macros
 */
#define LTC3577_GET_LED(burst, slew)		(burst | slew)

static	struct ltc3577           *ltc;

static ssize_t ltc3577_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int count = 0;

	len += sprintf(buf + len, "%02X", ltc->reg_read(ltc));
	len += sprintf(buf + len, "\n");
	len += sprintf(buf + len, "\n");

	return len;
}

static ssize_t ltc3577_reg_store(struct device *dev, struct device_attribute *attr,  const char *buf, size_t count)
{
	int val;
	int reg;
	int ret;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret != 2) {
		printk(KERN_ERR "%s: wrong arguments\n", __func__);
		return 0;
	}

	ret = ltc->reg_write(ltc, reg, val);

	printk("%02X <-- %02X\n", reg, val);
	return count;
}

//for factory test
static ssize_t ltc3577_charge_mode_store(struct device *dev, struct device_attribute *attr,  const char *buf, size_t count)
{
	struct ltc3577_pmic_data *pdata = dev->platform_data;
	int val;
	int ret;

	ret = sscanf(buf, "%d",&val);
	if (ret != 1) {
		printk(KERN_ERR "%s: wrong arguments\n", __func__);
		return 0;
	}
	
	if(pdata->set_charge!=NULL)
	{	
  	if(val>0)
  		pdata->set_charge(eCHARGING_1A);
  	else
  		pdata->set_charge(eCHARGING_500mA);
  	return count;	
  }		  	
  return 0;
}

static DEVICE_ATTR(reg, 0644, ltc3577_reg_show, ltc3577_reg_store);
static DEVICE_ATTR(charge_mode, 0644, NULL, ltc3577_charge_mode_store);

static void ltc3577_pmic_bucks(struct ltc3577 *ltc, uint8_t burst, uint8_t slew)
{
	ltc->reg_write(ltc, LTC3577_LED, LTC3577_GET_LED(burst, slew));
}

static void ltc3577_state_change_listener(USB_STATE previous_state, USB_STATE current_state, void* arg)
{
	struct platform_device *pdev = (struct platform_device *) arg;
	struct ltc3577_pmic_data *pdata = pdev->dev.platform_data;
	
	switch (current_state)
	{			
		case USB_STATE_HOST:
		case USB_STATE_CLA:
			/* CLA or wall pwr connected, enable 1A charging. */
			if(pdata->set_charge!=NULL)
				pdata->set_charge(eCHARGING_1A);
			printk(KERN_INFO LTC3577_DEVNAME 
						": USB_HOST|CLA [%i]\n", current_state);			
			break;
		case USB_STATE_DEVICE :
		case USB_STATE_DEVICE_WAIT :
			/* something connected, enable 500mA charging. */
			if(pdata->set_charge!=NULL)
				pdata->set_charge(eCHARGING_500mA);
			printk(KERN_INFO LTC3577_DEVNAME 
						": USB_DEVICE | DEVICE_WAIT [%i]\n", current_state);			
			break;		
		case USB_STATE_IDLE :
		default:
			if(pdata->set_charge!=NULL)
				pdata->set_charge(eCHARGING_500mA);
			printk(KERN_INFO LTC3577_DEVNAME 
						": USB_IDLE [%i]\n", current_state);			
			break;

	}							
	return;
}

static int __devinit ltc3577_pmic_probe(struct platform_device *pdev)
{
	struct ltc3577_pmic_data *pdata;
	int ret;
	USB_STATE curr_usb_state;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Failed to get platform data %s\n", pdev->name);
		ret = -ENODEV;
		goto err;
	}

	ltc = dev_to_ltc3577(&pdev->dev);
	if (!ltc) {
		dev_err(&pdev->dev, "Failed to get parent device %s\n", pdev->name);
		ret = -ENODEV;
		goto err;
	}

	if (!ltc->reg_read) {
		dev_err(&pdev->dev, "I2C not ready for receiving %s\n", pdev->name);
		ret = -ENODEV;
		goto err;
	}

	if (!ltc->reg_write) {
		dev_err(&pdev->dev, "I2C not ready for sending %s\n", pdev->name);
		ret = -ENODEV;
		goto err;
	}

	if ((device_create_file(&pdev->dev, &dev_attr_reg)) < 0) {
		dev_err(&pdev->dev, "failed to create reg file\n");
	}

	if ((device_create_file(&pdev->dev, &dev_attr_charge_mode)) < 0) {
		dev_err(&pdev->dev, "failed to create reg file\n");
	}
	
	ltc3577_pmic_bucks(ltc, pdata->burst_mode, pdata->slew_rate);
	
	if(0 != add_usb_state_change_listener(ltc3577_state_change_listener, pdev, &curr_usb_state))
	{
		printk(KERN_ERR  LTC3577_DEVNAME": Can't register USBMODE change state listener. Aborting.\n");
		return -ENODEV;
	}
	
	
	return 0;
err:
	return ret;
}



static int __devexit ltc3577_pmic_remove(struct platform_device *pdev)
{
	// anzu: add usb/gpio code here

	if( remove_usb_state_change_listener( ltc3577_state_change_listener, NULL ) != 0 )
		printk(KERN_ERR LTC3577_DEVNAME 
				": Couldn't unregister USBMODE change state listener!\n" );
				
	device_remove_file(&pdev->dev, &dev_attr_charge_mode);
	device_remove_file(&pdev->dev, &dev_attr_reg);
	return 0;
}

#ifdef CONFIG_PM
static int ltc3577_pmic_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ltc3577_pmic_data *pdata;
	
	if(remove_usb_state_change_listener(ltc3577_state_change_listener, NULL) != 0)
		printk(KERN_ERR LTC3577_DEVNAME 
				": Couldn't unregister USBMODE change state listener!\n");
				
	pdata = pdev->dev.platform_data;	
	// anzu: add usb code here
	if(pdata->set_charge!=NULL)
	  pdata->set_charge(eCHARGING_500mA);
	return 0;
}

static int ltc3577_pmic_resume(struct platform_device *pdev)
{
	// anzu: add usb code here
		USB_STATE curr_usb_state;
		
	if( add_usb_state_change_listener( ltc3577_state_change_listener, pdev, &curr_usb_state ) != 0) {
		printk( KERN_ERR LTC3577_DEVNAME 
				": Can't register USBMODE change state listener. Aborting.\n" );
		return -ENODEV;
	}	
	return 0;
}
#else
#define ltc3577_pmic_suspend	NULL
#define ltc3577_pmic_resume	NULL
#endif

static struct platform_driver ltc3577_pmic_driver = {
	.driver		= {
		.name	= LTC3577_DEVNAME,
		.owner	= THIS_MODULE,
	},
	.probe		= ltc3577_pmic_probe,
	.remove		= __devexit_p(ltc3577_pmic_remove),
	.suspend	= ltc3577_pmic_suspend,
	.resume		= ltc3577_pmic_resume,
};

static int __init ltc3577_pmic_init(void)
{
	return platform_driver_register(&ltc3577_pmic_driver);
}
module_init(ltc3577_pmic_init);

static void __exit ltc3577_pmic_exit(void)
{
	platform_driver_unregister(&ltc3577_pmic_driver);
}
module_exit(ltc3577_pmic_exit);

MODULE_DESCRIPTION("LTC3577 based PMIC Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrzej Zukowski <andrzej.zukowski@tomtom.com>");

