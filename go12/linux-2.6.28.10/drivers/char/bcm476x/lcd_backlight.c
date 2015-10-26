/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
*****************************************************************************
*
*  lcd_backlight.c
*
*  PURPOSE:
*
*   This implements the 476x specific LCD backlight driver.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/broadcom/lcd_backlight.h>
#include <linux/broadcom/gpio.h>
#include <linux/broadcom/regaccess.h>
#include <asm/atomic.h>
#ifdef CONFIG_BCM_SLEEP_MODE
   #include <linux/broadcom/cpu_sleep.h>
#endif


extern struct platform_device bcm_backlight_device;

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */

static LCD_BACKLIGHT_LEVEL  sCurrLevel = LCD_BACKLIGHT_FULL_ON / 2;
#define gpio_bl_en	((unsigned) platform_get_resource(&bcm_backlight_device, IORESOURCE_IO, 0)->start)
#define gpio_bl_pwm	((unsigned) platform_get_resource(&bcm_backlight_device, IORESOURCE_IO, 1)->start)
#define gpio_lcd_on	((unsigned) platform_get_resource(&bcm_backlight_device, IORESOURCE_IO, 2)->start)

/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  lcd_backlight_init
*
*
***************************************************************************/
void lcd_backlight_init( void )
{
	int rc;

	/* LCD_EN pin */
	rc = gpio_request(gpio_bl_en, "LCD Backlight Enable" );
	BUG_ON(rc < 0);
	rc = gpio_direction_output(gpio_bl_en, 1 );
	BUG_ON(rc < 0);

	/* LCD_ON pin */
	rc = gpio_request( gpio_lcd_on, "LCD ON" );
	BUG_ON(rc < 0);
	rc = gpio_direction_output( gpio_lcd_on, 1 );
	BUG_ON(rc < 0);

	/* PWM pin */
	rc = gpio_request( gpio_bl_pwm, "LCD Backlight PWM" );
	BUG_ON(rc < 0);
	rc = gpio_direction_output( gpio_bl_pwm, sCurrLevel );
	BUG_ON(rc < 0);

	/* Set bl intensity */
	lcd_backlight_enable( sCurrLevel );
}

/****************************************************************************
*
*  lcd_backlight_deinit
*
*
***************************************************************************/
void lcd_backlight_deinit( void )
{
    lcd_backlight_enable(gpio_bl_en);
    gpio_free(gpio_bl_en);
    gpio_free(gpio_bl_pwm);
    gpio_free(gpio_lcd_on);
}

/****************************************************************************
*
*  lcd_enable_backlight
*
*
***************************************************************************/
void lcd_backlight_enable( LCD_BACKLIGHT_LEVEL level )
{
#ifdef CONFIG_BCM_SLEEP_MODE
   static DEFINE_MUTEX(modSem);
   static int modulated_backlight = 0;
#endif

   if (( level == LCD_BACKLIGHT_OFF ) || ( level >= LCD_BACKLIGHT_MAX_LEVEL ))
   {
      // Disable backlight controller (PWM not required)
      //DCN, incompleted wait for hw definition for PWM registers.
      //REG_LCD_BLCR = 0;
      //regaccess_and_bits( &REG_SYS_IOCR0, ~REG_SYS_IOCR0_BKLIGHT );
      gpio_set_value(gpio_bl_en, level );

      #ifdef CONFIG_BCM_SLEEP_MODE
      {
         // If backlight was previously modulated, we no longer need to override sleep mode
         mutex_lock(&modSem);
         if (modulated_backlight)
         {
            modulated_backlight = 0;
            atomic_dec(&cpu_sleep_override);
         }
         mutex_unlock(&modSem);
      }
      #endif
   }
   else
   {
      #ifdef CONFIG_BCM_SLEEP_MODE
      {
         // Modulated backlight requires 13MHz PLL so we need to override sleep mode
         mutex_lock(&modSem);
         if (!modulated_backlight)
         {
            modulated_backlight = 1;
            atomic_inc(&cpu_sleep_override);
         }
         mutex_unlock(&modSem);
      }
      #endif


      // Enable modulated backlight controller
      // Other values allow backlight dimmer control
      // DCN, incompleted, wait for hw register definition for PMW
      //regaccess_or_bits( &REG_SYS_IOCR0, REG_SYS_IOCR0_BKLIGHT );
      //REG_LCD_BLCR = (min((int)level, LCD_BACKLIGHT_MAX_LEVEL) << REG_LCD_BLCR_DUTYSHFT) |
      //                REG_LCD_BLCR_FREQ_50KHZ | REG_LCD_BLCR_MOD_ON;
   }
   sCurrLevel = level;
}

/****************************************************************************
*
*   Returns the highest LCD Backlight level that this device supports.
*
***************************************************************************/

LCD_BACKLIGHT_LEVEL lcd_backlight_max_level( void )
{
    return LCD_BACKLIGHT_FULL_ON;
}

/****************************************************************************
*
*   Returns the current LCD Backlight level.
*
***************************************************************************/

LCD_BACKLIGHT_LEVEL lcd_backlight_curr_level( void )
{
    return sCurrLevel;
}

MODULE_AUTHOR( "Broadcom Corporation" );
MODULE_LICENSE( "GPL" );

