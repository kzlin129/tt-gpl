/*
 *  linux/drivers/mfd/s5m8751_pmic.c
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
#include <linux/mfd/s5m8751/s5m8751_core.h>

#include <mach/gpio.h>
#include <plat/usbmode.h>

/*
 * Conversion macros
 */
#define S5M8751_GET_ONOFF2(s) \
		((s & S5M8751_LDO1_EN ) ? S5M8751_LDO1_ENA	: 0) |\
		((s & S5M8751_LDO2_EN ) ? S5M8751_LDO2_ENA	: 0) |\
		((s & S5M8751_LDO3_EN ) ? S5M8751_LDO3_ENA	: 0) |\
		((s & S5M8751_LDO4_EN ) ? S5M8751_LDO4_ENA	: 0) |\
		((s & S5M8751_LDO5_EN ) ? S5M8751_LDO_MEMORY_ENA: 0)
#define S5M8751_GET_ONOFF3(s) \
		((s & S5M8751_LDO6_EN ) ? S5M8751_LDO_AUDIO_ENA	: 0) |\
		((s & S5M8751_BCK1_EN ) ? S5M8751_BUCK1_ENA	: 0) |\
		((s & S5M8751_BCK2_EN ) ? S5M8751_BUCK2_ENA	: 0)
#define S5M8751_GET_SLEEP_CNTL1(s) \
		((s & S5M8751_LDO1_EN ) ? S5M8751_LDO1_ENA	: 0) |\
		((s & S5M8751_LDO2_EN ) ? S5M8751_LDO2_ENA	: 0) |\
		((s & S5M8751_LDO3_EN ) ? S5M8751_LDO3_ENA	: 0) |\
		((s & S5M8751_LDO4_EN ) ? S5M8751_LDO4_ENA	: 0) |\
		((s & S5M8751_LDO5_EN ) ? S5M8751_LDO_MEMORY_ENA: 0)
#define S5M8751_GET_SLEEP_CNTL2(s) \
		((s & S5M8751_LDO6_EN ) ? S5M8751_LDO_AUDIO_ENA	: 0) |\
		((s & S5M8751_BCK1_EN ) ? S5M8751_BUCK1_ENA	: 0) |\
		((s & S5M8751_BCK2_EN ) ? S5M8751_BUCK2_ENA	: 0)

static struct s5m8751 *s5m8751;


//for factory test
static ssize_t s5m8751_charge_mode_store(struct device *dev, struct device_attribute *attr,  const char *buf, size_t count)
{
    struct platform_s5m8751_pmic_data *pdata = dev->platform_data;
    int val;
    int ret;

    ret = sscanf(buf, "%d",&val);
    if (ret != 1) {
        printk(KERN_ERR "%s: wrong arguments\n", __func__);
        return 0;
    }

    if(pdata->wall_pwr_pin)
    {	
        if(val>0)
            gpio_direction_output(pdata->wall_pwr_pin, 1);
        else
            gpio_direction_output(pdata->wall_pwr_pin, 0);
        return count;	
    }		  	
    return 0;
}

static ssize_t s5m8751_get_charging(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct platform_s5m8751_pmic_data *pdata = dev->platform_data;
    ssize_t len = 0;
    int count = 0;

    if(pdata->get_charging)
        len += sprintf(buf + len, "%02X", pdata->get_charging());
    else
        len += sprintf(buf + len, "0");

    len += sprintf(buf + len, "\n");
    len += sprintf(buf + len, "\n");

    return len;
}

static DEVICE_ATTR(charge_mode, 0644, NULL, s5m8751_charge_mode_store);
static DEVICE_ATTR(charging, 0644, s5m8751_get_charging, NULL);

static void s5m8751_pmic_set_sleep_state(uint32_t state)
{
	uint8_t reg_sleep_cntl1	= S5M8751_GET_SLEEP_CNTL1(state);
	uint8_t reg_sleep_cntl2	= S5M8751_GET_SLEEP_CNTL2(state);
	uint8_t reg_onoff1	= S5M8751_SLEEPB_PIN_EN;

	s5m8751_reg_write(s5m8751, S5M8751_SLEEP_CNTL1, reg_sleep_cntl1);
	s5m8751_reg_write(s5m8751, S5M8751_SLEEP_CNTL2, reg_sleep_cntl2);
	s5m8751_reg_write(s5m8751, S5M8751_ONOFF1, reg_onoff1);
}

static void s5m8751_pmic_set_wall_current(uint8_t wall_pwr_current)
{
	uint8_t reg_chg_iv_set = s5m8751_reg_read(s5m8751, S5M8751_CHG_IV_SET);

	reg_chg_iv_set &= ~S5M8751_FCHG_CSET_MASK;
	reg_chg_iv_set |= wall_pwr_current;
	s5m8751_reg_write(s5m8751, S5M8751_CHG_IV_SET, reg_chg_iv_set);
}

static void s5m8751_pmic_state_listener(USB_STATE prev_state, USB_STATE curr_state, void *arg)
{
	struct platform_s5m8751_pmic_data *pdata = (struct platform_s5m8751_pmic_data *) arg;

	if (curr_state == prev_state)
		return;

	switch (curr_state)
	{
		case USB_STATE_IDLE :
		default:
			/* nothing connected, or in transition between states, disable charging */
			printk("S5M8751: USB_STATE: [IDLE|DEFAULT]\n");
			gpio_direction_output(pdata->wall_pwr_pin, 0);
			break;
		case USB_STATE_CLA:
			/* CLA or wall pwr connected, enable 800mA charging. */
			printk("S5M8751: USB_STATE: [CLA]\n");
			gpio_direction_output(pdata->wall_pwr_pin, 1);
			break;
		case USB_STATE_DEVICE :
		case USB_STATE_DEVICE_WAIT :
		case USB_STATE_DEVICE_DETECT:
			/* something connected, enable 450mA charging. */
			printk("S5M8751: USB_STATE: [DEVICE|DEVICE_WAIT|DEVICE_DETECT]\n");
			gpio_direction_output(pdata->wall_pwr_pin, 0);
			break;
	}
}

static int s5m8751_pmic_probe(struct platform_device *pdev)
{
	struct platform_s5m8751_pmic_data *pdata = pdev->dev.platform_data;
	USB_STATE curr_state;
	int ret;

	s5m8751 = dev_to_s5m8751(&pdev->dev);

	s5m8751_pmic_set_sleep_state(pdata->sleep_mode_state);
	s5m8751_pmic_set_wall_current(pdata->wall_pwr_current);

	if (add_usb_state_change_listener(s5m8751_pmic_state_listener, pdata, &curr_state) != 0)
	{
		printk( KERN_ERR "S5M8751: Can't register USBMODE change state listener.\n" );
		ret = -ENODEV;
		goto err_usb;
	}

	if (pdata->request_gpio)
	{
		if ((ret = pdata->request_gpio()))
		{
			printk(KERN_ERR "S5M8751: Can't initialize GPIOs.\n");
			goto err_gpio;
		}

		if (pdata->config_gpio)
			pdata->config_gpio();
	}

	s5m8751_pmic_state_listener(USB_STATE_INITIAL, curr_state, pdata);
	
	if (device_create_file(&pdev->dev, &dev_attr_charge_mode) < 0) {
		dev_err(&pdev->dev, "failed to create charge mode file\n");
		device_remove_file(&pdev->dev, &dev_attr_charge_mode);
	}

	if (device_create_file(&pdev->dev, &dev_attr_charging) < 0) {
		dev_err(&pdev->dev, "failed to create charging file\n");
		device_remove_file(&pdev->dev, &dev_attr_charging);
	}
	
	return 0;

err_gpio:
	remove_usb_state_change_listener( s5m8751_pmic_state_listener, NULL);

err_usb:
	return ret;
}

static int s5m8751_pmic_remove(struct platform_device *pdev)
{
	struct platform_s5m8751_pmic_data *pdata = pdev->dev.platform_data;

	device_remove_file(&pdev->dev, &dev_attr_charge_mode);
	device_remove_file(&pdev->dev, &dev_attr_charging);


	if (pdata->free_gpio)
		pdata->free_gpio();

	if (remove_usb_state_change_listener( s5m8751_pmic_state_listener, NULL) != 0)
		printk("S5M8751: Couldn't remove usb state change listener!\n");

	return 0;
}

#ifdef CONFIG_PM
static int s5m8751_pmic_suspend(struct platform_device *pdev, pm_message_t state)
{	
	if (remove_usb_state_change_listener( s5m8751_pmic_state_listener, NULL) != 0)
		printk("S5M8751: Couldn't remove usb state change listener!\n");

	return 0;
}

static int s5m8751_pmic_resume(struct platform_device *pdev)
{
	struct platform_s5m8751_pmic_data *pdata = pdev->dev.platform_data;
	USB_STATE curr_state;

	if (add_usb_state_change_listener(s5m8751_pmic_state_listener, pdata, &curr_state) != 0)
	{
		printk(KERN_ERR "S5M8751: Can't register USBMODE change state listener.\n");
		return -ENODEV;
	}

	s5m8751_pmic_state_listener(USB_STATE_INITIAL, curr_state, pdata);
		
	return 0;
}
#else
#define s5m8751_pmic_suspend	NULL
#define s5m8751_pmic_resume	NULL
#endif

static struct platform_driver s5m8751_pmic_driver = {
	.driver		= {
		.name	= "s5m8751-pmic",
		.owner	= THIS_MODULE,
	},
	.probe		= s5m8751_pmic_probe,
	.remove		= s5m8751_pmic_remove,
	.suspend	= s5m8751_pmic_suspend,
	.resume		= s5m8751_pmic_resume,
};

static int __init s5m8751_pmic_init(void)
{
	return platform_driver_register(&s5m8751_pmic_driver);
}
module_init(s5m8751_pmic_init);

static void __exit s5m8751_pmic_exit(void)
{
	platform_driver_unregister(&s5m8751_pmic_driver);
}
module_exit(s5m8751_pmic_exit);

MODULE_DESCRIPTION("s5m8751 based PMIC Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrzej Zukowski <andrzej.zukowski@tomtom.com>");

