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
/**
*
*  @file    halaudio_platform_src.c
*
*  @brief   HAL Audio platform specific extensions
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include <linux/proc_fs.h>          /* For /proc/halAudioExternalAK4642 */
#include <linux/sched.h>            /* For schedule_timeout */

#include <linux/broadcom/halaudio.h>/* HALAUDIO API */
#include <linux/broadcom/halaudio_settings.h>/* HALAUDIO API */
#include <linux/broadcom/gpio.h>    /* GPIO register accesses */

#ifndef HW_GPIO_SPKR_PWDN_PIN
#error  HW_GPIO_SPKR_PWDN_PIN must be defined 
#endif

#define GPIO_SUPPORT    1

/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Function Prototypes -------------------------------------- */
static int setAnaGain( HAL_AUDIO_BLOCK block, short db );

/* ---- Private Variables ------------------------------------------------- */
static struct hal_audio_platform_ops platform_ops =
{
   .ana_gain   = setAnaGain,     /* Platform analog gain set */
};

/* ---- Public Variables ------------------------------------------------- */

/* ---- Functions -------------------------------------------------------- */

/***************************************************************************/
/**
*  Platform specific analog gain set routine. The following implementation
*  sets a GPIO to enable an external op-amp for a particular block
*
*  @return  
*     0     - Success
*     -ve   - error code
*/
static int setAnaGain(
   HAL_AUDIO_BLOCK block,              /**< (in) HAL block ID */
   short db                            /**< (in) gain in db */
)
{
   (void) block; (void) db;   /* avoid compiler warnings */

   if ( block == HAL_AUDIO_SPKRPHONE_SPKR )
   {
      static int spkrPaEnabled = 0;

      if ( db > HAL_AUDIO_GAIN_MUTE )
      {  
         if ( !spkrPaEnabled )
         {
#if GPIO_SUPPORT
            /* enable external speaker op-amp */
            gpio_set_value( HW_GPIO_SPKR_PWDN_PIN, 1 );
#endif
            spkrPaEnabled = 1;

            /* Make sure that this code path is never taken from an atomic context. */
            might_sleep(); 

            /* Delay 300ms to allow the external PA to power up (wake up period) */
            set_current_state( TASK_INTERRUPTIBLE );
            schedule_timeout( HZ * 300 / 1000 );
         }
      }
      else
      {
#if GPIO_SUPPORT
         gpio_set_value( HW_GPIO_SPKR_PWDN_PIN, 0 );
#endif
         spkrPaEnabled = 0;
      }
   }

   return 0;
}

/***************************************************************************/
/**
*  Platform specific extension contructor
*
*  @return  
*     0     - Success
*     -ve   - error code
*/
static int __init halaudio_platform_init( void )
{
   int rc = 0;

#if GPIO_SUPPORT
   rc = gpio_request( HW_GPIO_SPKR_PWDN_PIN, "External speaker op-amp" );
   if ( rc ) 
   {
      printk( KERN_ERR "%s: Failed to request GPIO pin %i\n", __FUNCTION__, HW_GPIO_SPKR_PWDN_PIN );
   }

   /* Initially shutdown op-amp */
   gpio_direction_output( HW_GPIO_SPKR_PWDN_PIN, 0 );
#endif

   /* Register platform specific operations with HAL Audio */
   if ( rc == 0 )
   {
      rc = halAudio_RegisterPlatform( &platform_ops );
   }

   if ( rc == 0 )
   {
      printk( KERN_INFO "HAL Audio platform extensions installed\n" );
   }

   return rc;
}

/***************************************************************************/
/**
*  Platform specific extension destructor
*/
static void __exit halaudio_platform_exit( void )
{
   struct hal_audio_platform_ops ops;

   /* De-register platform ops */
   memset( &ops, 0, sizeof(ops) );
   halAudio_RegisterPlatform( &ops );

#if GPIO_SUPPORT
   gpio_free( HW_GPIO_SPKR_PWDN_PIN );
#endif
}

module_init( halaudio_platform_init );
module_exit( halaudio_platform_exit );

