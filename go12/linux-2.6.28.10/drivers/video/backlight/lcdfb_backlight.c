/*****************************************************************************
* Copyright 2003 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/


/*
*
*   lcdfb_backlight.c
*
*   This file contains code which enables backlight support through the
*   backlight device.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/broadcom/lcd.h>
#include <linux/broadcom/lcd_backlight.h>

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

static char gBanner[] __initdata = KERN_INFO "LCDFB-Backlight: 0.1\n";

/* ---- Private Variables ------------------------------------------------ */

/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*   Return the current backlight brightness; accounting for power, blanking
*   etc.
*
****************************************************************************/

static int lcdfb_backlight_get_brightness( struct backlight_device *bl_dev )
{
    (void) bl_dev;

    return lcd_backlight_curr_level();

} // lcdfb_backlight_get_brightness

/****************************************************************************
*
*   Callback which is called whenever a property has been changed
*
****************************************************************************/

static int lcdfb_backlight_update_status( struct backlight_device *bl_dev )
{
    LCD_BACKLIGHT_LEVEL level;

    level = bl_dev->props.brightness;

    if (( bl_dev->props.power    != FB_BLANK_UNBLANK )
    ||  ( bl_dev->props.fb_blank != FB_BLANK_UNBLANK ))
    {
        level = 0;
    }
    lcd_backlight_enable( level );

#if defined( CONFIG_FB_LCD )
    lcd_set_power( bl_dev->props.power == FB_BLANK_UNBLANK );
#endif

    return 0;

} // lcdfb_backlight_update_status

/****************************************************************************
*
*   Data structure which defines the backlight operations
*
****************************************************************************/

static struct backlight_ops lcdfb_backlight_ops =
{
    .update_status      = lcdfb_backlight_update_status,
    .get_brightness     = lcdfb_backlight_get_brightness,
};

/****************************************************************************
*
*   Function called to initialize our driver.
*
****************************************************************************/

static int __init lcdfb_backlight_probe( struct platform_device *dev )
{
    struct backlight_device *bl_dev;

    lcd_backlight_init();

    bl_dev = backlight_device_register( "lcdfb-bl", &dev->dev, NULL, &lcdfb_backlight_ops );
    if ( IS_ERR( bl_dev ))
    {
        printk( KERN_ERR "lcdfb: Registration of backlight device failed\n" );
        return PTR_ERR( bl_dev );
    }

    platform_set_drvdata( dev, bl_dev );

    bl_dev->props.max_brightness = lcd_backlight_max_level();
    bl_dev->props.brightness = bl_dev->props.max_brightness;
    bl_dev->props.power = FB_BLANK_UNBLANK;
    bl_dev->props.fb_blank = FB_BLANK_UNBLANK;

    backlight_update_status( bl_dev );

    printk( gBanner );

    return 0;

} // lcdfb_backlight_probe

/****************************************************************************
*
*   Function called to remove our driver.
*
****************************************************************************/

static int lcdfb_backlight_remove( struct platform_device *dev )
{
    struct backlight_device *bl_dev = platform_get_drvdata( dev );

    bl_dev->props.brightness = 0;
    bl_dev->props.power = 0;
    backlight_update_status( bl_dev );

    backlight_device_unregister( bl_dev );

    lcd_backlight_deinit();

    return 0;

} // lcdfb_backlight_remove

/****************************************************************************
*
*   Data structure which defines the backlight driver
*
****************************************************************************/

static struct platform_driver lcdfb_backlight_driver =
{
    .probe		= lcdfb_backlight_probe,
	.remove		= lcdfb_backlight_remove,
	.driver		= 
    {
		.name	= "lcdfb-bl",
	},
};

/****************************************************************************
*
*   Module initialization and cleanup.
*
****************************************************************************/

static int __init lcdfb_backlight_init( void )
{
    return platform_driver_register( &lcdfb_backlight_driver );
}

static void __exit lcdfb_backlight_exit( void )
{
    platform_driver_unregister( &lcdfb_backlight_driver );
}

/****************************************************************************/

module_init( lcdfb_backlight_init);
module_exit( lcdfb_backlight_exit);

MODULE_AUTHOR( "Broadcom");
MODULE_DESCRIPTION("Broadcom LCD Backlight Driver");
MODULE_LICENSE( "GPL" );

