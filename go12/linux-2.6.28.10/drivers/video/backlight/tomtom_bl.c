/*
 *  TomTom GENERIC Backlight Driver 
 *
 *  Copyright (c) 2004-2009 Benoit Leffray
 *
 *  Based on Sharp's 2.4 Backlight Driver
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

#define TOMTOM_BL_NAME		"tomtom-bl" 
#define TOMTOM_BL_PFX		TOMTOM_BL_NAME " Driver: " 

#define TOMTOM_BL_NONE		0x00
#define TOMTOM_BL_SUSPENDED	0x01
#define TOMTOM_BL_RESUMED	0x02

typedef struct {
	int intensity;
	int suspend_intensity;
	unsigned long flags;
	struct generic_bl_info *machinfo;
} tomtombl_data_t;

static int tomtombl_send_intensity(struct backlight_device *bd)
{
	tomtombl_data_t *bl_drv_data = bl_get_data(bd);
	int intensity;

	BUG_ON(!bl_drv_data);

	intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bl_drv_data->flags & TOMTOM_BL_SUSPENDED) {
		bl_drv_data->suspend_intensity = intensity;
		intensity = 0;
	}
	if (bl_drv_data->flags & TOMTOM_BL_RESUMED)
		intensity = bl_drv_data->suspend_intensity;

	bl_drv_data->machinfo->set_bl_intensity(intensity);

	bl_drv_data->intensity = intensity;

	return 0;
}

#ifdef CONFIG_PM
static int tomtombl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	tomtombl_data_t *bl_drv_data = bl_get_data(bd);
	BUG_ON(!bl_drv_data);

	bl_drv_data->flags |= TOMTOM_BL_SUSPENDED;
	backlight_update_status(bd);

	printk(KERN_INFO TOMTOM_BL_PFX "Suspended.\n");
	return 0;
}

static int tomtombl_resume(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	tomtombl_data_t *bl_drv_data = bl_get_data(bd);
	BUG_ON(!bl_drv_data);

	bl_drv_data->flags &= ~TOMTOM_BL_SUSPENDED;
	bl_drv_data->flags |= TOMTOM_BL_RESUMED;

	backlight_update_status(bd);

	bl_drv_data->flags &= ~TOMTOM_BL_RESUMED;

	printk(KERN_INFO TOMTOM_BL_PFX "Resumed.\n");
	return 0;
}
#else
#define tomtombl_suspend	NULL
#define tomtombl_resume	NULL
#endif

static int tomtombl_get_intensity(struct backlight_device *bd)
{
	tomtombl_data_t *bl_drv_data = bl_get_data(bd);
	BUG_ON(!bl_drv_data);
	return bl_drv_data->intensity;
}

static struct backlight_ops tomtombl_ops = {
	.get_brightness = tomtombl_get_intensity,
	.update_status  = tomtombl_send_intensity,
};

static int tomtombl_probe(struct platform_device *pdev)
{
	struct backlight_device *bl_device;
	struct generic_bl_info *machinfo;
	const char *name = TOMTOM_BL_NAME;
	tomtombl_data_t *bl_drv_data = NULL;

	BUG_ON(!pdev);
	BUG_ON(!(&pdev->dev));

	printk(KERN_INFO TOMTOM_BL_PFX "Probed.\n");

	machinfo = pdev->dev.platform_data;
	BUG_ON(!machinfo);

	bl_drv_data = kmalloc(GFP_KERNEL, sizeof(tomtombl_data_t));
	if (NULL == bl_drv_data)
		return -ENOMEM;

	memset(bl_drv_data, 0x00, sizeof(tomtombl_data_t)); 
	bl_drv_data->machinfo = machinfo;
	bl_drv_data->flags = TOMTOM_BL_NONE;

	if (!machinfo->limit_mask)
		machinfo->limit_mask = -1;

	if (machinfo->name)
		name = machinfo->name;

	bl_device = backlight_device_register (name,
									&pdev->dev, NULL, &tomtombl_ops);
	if (IS_ERR (bl_device))
		return PTR_ERR (bl_device);

	dev_set_drvdata(&bl_device->dev, bl_drv_data);

	platform_set_drvdata(pdev, bl_device);

	bl_device->props.max_brightness = machinfo->max_intensity;
	bl_device->props.power = FB_BLANK_UNBLANK;
	bl_device->props.brightness = machinfo->default_intensity;

	backlight_update_status(bl_device);

	printk(KERN_INFO TOMTOM_BL_PFX "Initialized.\n");
	return 0;
}

static int tomtombl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	tomtombl_data_t *bl_drv_data = bl_get_data(bd);

	bd->props.power = 0;
	bd->props.brightness = 0;
	backlight_update_status(bd);

	backlight_device_unregister(bd);

	kfree(bl_drv_data);

	printk(KERN_INFO TOMTOM_BL_PFX "Removed.\n");
	return 0;
}

static struct platform_driver tomtombl_driver = {
	.probe		= tomtombl_probe,
	.remove		= tomtombl_remove,
	.suspend	= tomtombl_suspend,
	.resume		= tomtombl_resume,
	.driver		= {
		.name	= TOMTOM_BL_NAME,
	},
};

static int __init tomtombl_init(void)
{
	return platform_driver_register(&tomtombl_driver);
}

static void __exit tomtombl_exit(void)
{
	platform_driver_unregister(&tomtombl_driver);
}

module_init(tomtombl_init);
module_exit(tomtombl_exit);

MODULE_AUTHOR("Benoit Leffray <benoit.leffray@tomtom.com>");
MODULE_DESCRIPTION("TomTom Backlight Driver");
MODULE_LICENSE("GPL");
