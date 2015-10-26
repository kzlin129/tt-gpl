/*
 * linux/drivers/video/backlight/s5m8751_bl.c
 *
 * simple s5m8751 based backlight control, proper i2c driver should be in place
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/reboot.h>
#include <linux/mfd/s5m8751/s5m8751_backlight.h>
#include <linux/mfd/s5m8751/s5m8751_core.h>

static struct s5m8751 *s5m8751;

static void s5m8751_shutdown_bl(void)
{
	s5m8751_reg_write(s5m8751, S5M8751_WLED_CNTRL, 0x0);
}

static int s5m8751_backlight_update_status(struct backlight_device *bl)
{
	struct platform_s5m8751_backlight_data *data = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (data->convert)
		brightness = data->convert(brightness);

	if (brightness == 0) {
		s5m8751_shutdown_bl();
	} else {
		s5m8751_reg_write(s5m8751, S5M8751_WLED_CNTRL,
				(0x1 << S5M8751_WLED_EN_SHIFT) |
				((!!data->pwm_freq) << S5M8751_FREQ_SHIFT) |
				brightness);
	}

	return 0;
}

static int s5m8751_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static struct backlight_ops s5m8751_backlight_ops = {
	.update_status	= s5m8751_backlight_update_status,
	.get_brightness	= s5m8751_backlight_get_brightness,
};

static int s5m8751_bl_reboot(struct notifier_block *block,
			     unsigned long cause, void* dummy)
{
	pr_emerg("Shutting down backlight\n");
	s5m8751_shutdown_bl();
	return 0;
}

static struct notifier_block s5m8751_bl_reboot_block = {
	.notifier_call	= s5m8751_bl_reboot,
};

static int s5m8751_backlight_probe(struct platform_device *pdev)
{
	struct platform_s5m8751_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	int ret;

	bl = backlight_device_register(pdev->name, &pdev->dev,
			data, &s5m8751_backlight_ops);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	s5m8751 = dev_to_s5m8751(&pdev->dev);

	bl->props.max_brightness = data->max_brightness;
	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	register_reboot_notifier(&s5m8751_bl_reboot_block);
	return 0;

err_bl:
	return ret;
}

static int s5m8751_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_device_unregister(bl);
	return 0;
}

#ifdef CONFIG_PM
static int s5m8751_backlight_suspend(struct platform_device *pdev,
				     pm_message_t state)
{
	s5m8751_shutdown_bl();
	return 0;
}

static int s5m8751_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define s5m8751_backlight_suspend	NULL
#define s5m8751_backlight_resume	NULL
#endif

static struct platform_driver s5m8751_backlight_driver = {
	.driver		= {
		.name	= "s5m8751-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= s5m8751_backlight_probe,
	.remove		= s5m8751_backlight_remove,
	.suspend	= s5m8751_backlight_suspend,
	.resume		= s5m8751_backlight_resume,
};

static int __init s5m8751_backlight_init(void)
{
	return platform_driver_register(&s5m8751_backlight_driver);
}
module_init(s5m8751_backlight_init);

static void __exit s5m8751_backlight_exit(void)
{
	platform_driver_unregister(&s5m8751_backlight_driver);
}
module_exit(s5m8751_backlight_exit);

MODULE_DESCRIPTION("s5m8751 based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrzej Zukowski <andrzej.zukowski@tomtom.com>");

