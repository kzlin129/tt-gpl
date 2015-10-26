/*
 *  Backlight Driver for Sharp Corgi
 *
 *  Copyright (c) 2004-2005 Richard Purdie
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
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/ltc3577-regulator.h>

#include <linux/delay.h>
#include <barcelona/Barc_pwm.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>

/* Defines */
#define PFX 				"s3c_ltc: "
#define PK_DBG				PK_DBG_FUNC
#define SL_DBG				PK_DBG_NONE

#define FLAG_STARTED                    1

static unsigned int s3c_flags		=~(FLAG_STARTED);
static          int s3c_powermode	= FB_BLANK_UNBLANK;
static unsigned int s3c_level		= PWM_BACKLIGHT_MAX;
static unsigned int s3c_range		= 100;
static unsigned int s3c_hw_initialised	= 0;

static int  s3c_hw_init (struct device *dev);
static void s3c_shutdown(struct device *dev);
static int  s3c_set_intensity(struct backlight_device *bd, int intensity);
static int  s3c_get_intensity(struct backlight_device *bd) ;
static int  s3c_hw_setlevel(unsigned level);


static void s3c_hw_setlevel_direct(unsigned level)
{
//	unsigned int bl_i;
//anzu	/* Map values if mapping is defined */ 
//anzu	if (IO_GetBacklightMapping()) {
//anzu		bl_i = level / 5;
//anzu		/* bl_i = (level - 10) * 20 / 90; */
//anzu		level = (int)((unsigned char*)IO_GetBacklightMapping())[bl_i];
//anzu		SL_DBG("%s(mapping):  level: %d \n", __FUNCTION__, level); 
//anzu	}
        
	if (!s3c_hw_initialised) {
		s3c_hw_initialised = 1;
	}
	ltc3577_regulator_set_power(level);


}

static int s3c_hw_setlevel(unsigned level)
{
	static unsigned old_level = PWM_BACKLIGHT_MAX+1; // Backlight is already enabled by bootloader at unknown value, so make sure level will be different

	PK_DBG("level=%u, old_level=%u\n", level, old_level);

	if (old_level == level)
		return 0;

	if (level < PWM_BACKLIGHT_MIN || level > PWM_BACKLIGHT_MAX) { 
		PK_WARN("Invalid backlight level %u\n", level);
		return -EINVAL;
	}

	s3c_hw_setlevel_direct(level);
	old_level = level;
	return 0;
}

static inline int s3c_setlevel(unsigned level)
{
	int ret;

	ret = s3c_hw_setlevel(level);
	if (ret < 0)
		return ret;

	s3c_level = level;
	return 0;
}

static int s3c_hw_init(struct device *dev)
{
	s3c_hw_initialised = 0;

	s3c_flags |= FLAG_STARTED;

	PK_DBG("Done\n");
	return 0;
}

static void s3c_hw_exit(void)
{
	PK_DBG("Turning off backlight\n");
	s3c_hw_initialised = 0;
}

static void s3c_blank(struct device *dev, int blank)
{
	switch(blank) {
		case FB_BLANK_NORMAL:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_POWERDOWN:
				s3c_shutdown(dev);
				s3c_powermode = blank;
			break;
		case FB_BLANK_UNBLANK:
				s3c_powermode = blank;
				s3c_hw_init(dev);
				/* Set back the current intensity level */
				s3c_set_intensity(NULL, PWM_BACKLIGHT_MAX) ;
			break;
	} /* End switch */
}

static int s3cbl_set_power(struct backlight_device *bd, int blank)
{
	int rc;

	switch(blank) {
		case FB_BLANK_NORMAL:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_POWERDOWN:
		{
			if (s3c_powermode == FB_BLANK_UNBLANK) {
				s3c_powermode = blank;

			rc = s3c_setlevel(PWM_BACKLIGHT_MIN);
			if( rc == 0 ) 
				s3c_flags &= ~FLAG_STARTED;
			}
			break;
		}
		case FB_BLANK_UNBLANK:
		{
			if (s3c_powermode != FB_BLANK_UNBLANK) {
				s3c_powermode = blank;

			rc = s3c_setlevel(PWM_BACKLIGHT_MAX);
			if( rc == 0 ) 
				s3c_flags |= FLAG_STARTED;
			}
			break;
		}
	} /* End Switch */

	return 0;
}

static int s3cbl_get_power(struct backlight_device *bd)
{
	return s3c_powermode;
}

static int s3c_set_intensity(struct backlight_device *bd, int intensity)
{
	int rc;

	if (intensity > PWM_BACKLIGHT_MAX)
		intensity = PWM_BACKLIGHT_MAX;

	if (s3c_powermode != FB_BLANK_UNBLANK)
		intensity = PWM_BACKLIGHT_MIN;

	rc = s3c_setlevel(intensity);
	if( rc == 0 ) {
		if (intensity != PWM_BACKLIGHT_MIN) 
			s3c_flags |= FLAG_STARTED;
		else 
			s3c_flags &= ~FLAG_STARTED;
	}

	return 0;
}

static int s3c_get_intensity(struct backlight_device *bd)
{
	return s3c_level;
}

static struct backlight_properties s3cbl_data = {
	.owner		= THIS_MODULE,
	.get_power      = s3cbl_get_power,
	.set_power      = s3cbl_set_power,
	.max_brightness = PWM_BACKLIGHT_MAX,
	.get_brightness = s3c_get_intensity,
	.set_brightness = s3c_set_intensity,
};

static struct backlight_device *s3c_backlight_device;

static int s3c_probe(struct device *dev)
{
	int ret;

	ret = s3c_hw_init(dev);
	if (ret < 0) {
		PK_ERR("Failed to initialize hardware (%d)\n", ret);
		return ret;
	}

	/* Set the level to the same as the bootloader uses. */
	s3c_level = (IO_HaveUsbBusPowered( ) ? 30 : 80);
	s3c_hw_setlevel( s3c_level );

	PK_DBG("Registering Backlight\n");
	s3c_backlight_device = backlight_device_register ("s3c_ltc", NULL, &s3cbl_data);
	if (IS_ERR (s3c_backlight_device))
		return PTR_ERR (s3c_backlight_device);

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#endif
	PK_DBG("Done\n");
	return 0;
}

static int s3c_remove(struct device *dev)
{
	s3c_hw_exit();

	PK_DBG("Unregistering Backlight\n");
	backlight_device_unregister(s3c_backlight_device);

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#endif
	PK_DBG("Done\n");
	return 0;
}

static void s3c_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
	s3c_hw_exit();
}

#ifdef CONFIG_PM
static int s3c_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);

	if (level == SUSPEND_POWER_DOWN) {
		s3c_blank(dev, FB_BLANK_POWERDOWN);
	}
	return 0;
}

static int s3c_resume(struct device *dev, u32 level)
{
	PK_DBG("dev = %p, level = %u\n", dev, level);

	if (level == RESUME_POWER_ON) {
		s3c_blank(dev, FB_BLANK_UNBLANK);
	}
	return 0;
}
#else /* CONFIG_PM */

#define s3c_suspend NULL
#define s3c_resume  NULL

#endif /* CONFIG_PM */

static struct device_driver s3c_driver = {
	.name		= "backlight_s3c_ltc",
	.bus		= &platform_bus_type,
	.probe		= s3c_probe,
	.remove		= s3c_remove,
	.shutdown	= s3c_shutdown,
	.suspend	= s3c_suspend,
	.resume		= s3c_resume,
};

static int __init s3c_bl_mod_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO Backlight S3C LTC, (C) 2009 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&s3c_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit s3c_bl_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&s3c_driver);
	PK_DBG("Done\n");
}


module_init(s3c_bl_mod_init);
module_exit(s3c_bl_mod_exit);

MODULE_AUTHOR("Andrzej Zukowski andrzej.zukowski@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO S3C Driver");
MODULE_LICENSE("GPL");

/* EOF */
