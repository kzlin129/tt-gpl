/*
 *  drivers/mfd/ltc3577-bl.c
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
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/mfd/ltc3577/bl.h>
#include <linux/mfd/ltc3577/core.h>

/*
 * Conversion macros
 */
#define LTC3577_GET_LED(r, p, g, s) \
		(((r >> LTC3577_LED_SHIFT) & LTC3577_LED_MASK) | p | g | s)
#define LTC3577_GET_DAC(r) \
		((r >> LTC3577_DAC_SHIFT) & LTC3577_DAC_MASK)
#define LTC3577_GET_PWM(r) \
		((r >> LTC3577_PWM_SHIFT) & LTC3577_PWM_MASK)


static struct ltc3577 *ltc;

static int ltc3577_bl_update_status(struct backlight_device *bl)
{
	struct ltc3577_bl_data *pdata = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	uint32_t reg;
	uint8_t  dac;
	uint8_t  pwm;
	uint8_t  led;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	reg = pdata->get_intensity(brightness);
	dac = LTC3577_GET_DAC(reg);
	pwm = LTC3577_GET_PWM(reg);
	led = LTC3577_GET_LED(reg, pdata->pwm, pdata->gr, pdata->slew);

	ltc->reg_write_n(ltc, pwm, dac, led);
	return 0;
}

static int ltc3577_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static struct backlight_ops ltc3577_bl_ops = {
	.update_status	= ltc3577_bl_update_status,
	.get_brightness	= ltc3577_bl_get_brightness,
};

static int __devinit ltc3577_bl_probe(struct platform_device *pdev)
{
	struct ltc3577_bl_data  *pdata = pdev->dev.platform_data;
	struct backlight_device *bl;
	int ret;

	bl = backlight_device_register(pdev->name, &pdev->dev,
			pdata, &ltc3577_bl_ops);
	if (IS_ERR(bl)) {
		ret = PTR_ERR(bl);
		dev_err(&pdev->dev, "Failed to register backlight %s: %d\n",
			pdev->name, ret);
		goto err;
	}

	ltc = dev_to_ltc3577(&pdev->dev);

	if (!ltc->reg_read) {
		dev_err(&pdev->dev, "I2C not ready for receiving %s\n", pdev->name);
		ret = -ENODEV;
		goto errbl;
	}

	if (!ltc->reg_write) {
		dev_err(&pdev->dev, "I2C not ready for sending %s\n", pdev->name);
		ret = -ENODEV;
		goto errbl;
	}

	bl->props.max_brightness = pdata->max_brightness;
	bl->props.brightness     = pdata->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

errbl:
	backlight_device_unregister(bl);
err:
	return ret;
}

static int __devexit ltc3577_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_device_unregister(bl);
	return 0;
}

#ifdef CONFIG_PM
static int ltc3577_bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int ltc3577_bl_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define ltc3577_bl_suspend	NULL
#define ltc3577_bl_resume	NULL
#endif

static struct platform_driver ltc3577_bl_driver = {
	.driver		= {
		.name	= "ltc3577-bl",
		.owner	= THIS_MODULE,
	},
	.probe		= ltc3577_bl_probe,
	.remove		= __devexit_p(ltc3577_bl_remove),
	.suspend	= ltc3577_bl_suspend,
	.resume		= ltc3577_bl_resume,
};

static int __init ltc3577_bl_init(void)
{
	return platform_driver_register(&ltc3577_bl_driver);
}
module_init(ltc3577_bl_init);

static void __exit ltc3577_bl_exit(void)
{
	platform_driver_unregister(&ltc3577_bl_driver);
}
module_exit(ltc3577_bl_exit);

MODULE_DESCRIPTION("LTC3577 based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrzej Zukowski <andrzej.zukowski@tomtom.com>");

