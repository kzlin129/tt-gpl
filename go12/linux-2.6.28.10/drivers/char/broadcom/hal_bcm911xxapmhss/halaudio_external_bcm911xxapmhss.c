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



/**
*
*  @file    halaudio_external_bcm911xxapmhss.c
*
*  @brief   HAL Audio External routines for the BCM911xx APM and HSS 
*           reference boards. An example board that uses this external 
*           HAL is the BCM91103EVM.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include <linux/module.h>
#include <stddef.h>                 /* Needed for NULL */

#include <linux/broadcom/hw_cfg.h>  /* Hardware configuration of GPIOs */
#include <linux/broadcom/halaudio.h>/* HALAUDIO API */
#include <linux/broadcom/halaudio_settings.h>/* HALAUDIO Settings */
#include <linux/broadcom/gpio.h>
#include <linux/sched.h>            /* Needed for schedule_timeout() */

/* definition of the HAL Audio HSS (addon) module) */
#include "halaudio_bcm911xxhssaddon.h"

/* ---- Public Variables ------------------------------------------------- */
/**
 * @addtogroup HAL
 * @{
 */

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */


static int  halAudioExternal_Init( void );
static int  halAudioExternal_Exit( void );
static int  halAudioExternal_GainSetAnalogHardware( HAL_AUDIO_BLOCK block, short db );
/**
* External Function table
*/
static HAL_AUDIO_EXTERNAL_FNCS halAudioExternalFncs =
{
   halAudioExternal_Init,
   halAudioExternal_Exit,
   halAudioExternal_GainSetAnalogHardware,
   NULL,    /* gainSetDigitalHardware */
   NULL,    /* getCapabilities */
   NULL,    /* getFrequencies */
   NULL,    /* SetFrequency */
   NULL,
   NULL,
   NULL,
   NULL
};

/* ---- Private Functions ------------------------------------------------ */

/***************************************************************************/
/**
*  External audio block int routine. Alloc resources, initialize hardware.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternal_Init(void)
{
   int codecOffset;

   /* register the HSS module with the internal driver */
   codecOffset = halAudio_registerAddOnModule( &halAudioHssModule );
   if( codecOffset >= 0 )
   {
      /* we have to save the HAL Audio codec offset number */
      halAudioHSSCodecOffset = codecOffset;
   }
#ifdef HW_GPIO_SPKR_PWDN_PIN
   gpio_request( HW_GPIO_SPKR_PWDN_PIN, "Speaker Enable" );
#endif
   return 0;
}

/***************************************************************************/
/**
*  External audio block exit routine. Frees resources.
*
*  @return  0 on success, otherwise failure
*/
static int halAudioExternal_Exit(void)
{
#ifdef HW_GPIO_SPKR_PWDN_PIN
   gpio_free( HW_GPIO_SPKR_PWDN_PIN );
#endif
   return 0;
}

/***************************************************************************/
/**
*  HAL audio gain set routine used to customize hardware gains for the chosen
*  block. Only add any overrides to the default behavior here.
*
*  @return  0 on success, otherwise failure
*/
#ifndef HZ  /* check that HZ is defined */
#error HZ not defined 
#endif
static int halAudioExternal_GainSetAnalogHardware(
   HAL_AUDIO_BLOCK block,        /**< (in) HAL block ID */
   short db                      /**< (in) gain in db */
)
{
#ifdef HW_GPIO_SPKR_PWDN_PIN
   if ( block == HAL_AUDIO_SPKRPHONE_SPKR )
   {
      /* PA enable state var. used to avoid the long power up sequence 
       * if already powered up
       */
      static int spkrPaEnabled = 0;

      /* Custom GPIO handling to enable external speaker on the EVM board 
       */
      if ( db > HAL_AUDIO_GAIN_MUTE )
      {  
         if ( !spkrPaEnabled )
         {
            /* enable the GPIO 23 to enable the external speaker driver */
            gpio_direction_output( HW_GPIO_SPKR_PWDN_PIN, 1 );
            spkrPaEnabled = 1;

            /* Delay 300ms to allow the external PA to power up (wake up period) */
            set_current_state( TASK_INTERRUPTIBLE );
            schedule_timeout( HZ * 300 / 1000 );
         }
      }
      else
      {
         gpio_direction_output( HW_GPIO_SPKR_PWDN_PIN, 0 );
         spkrPaEnabled = 0;
      }
   }
#else
   (void)block; (void)db;
#endif
   return 0;
}

/* ---- Public Functions ------------------------------------------------- */

/***************************************************************************/
/**
*  halAudioExternalGetParams
*
*  @return  0 if no parameters, 1 if parameters
*/
int halAudioExternalGetParams(const HAL_AUDIO_EXTERNAL_FNCS **paramp)
{
   printk( KERN_INFO "External HAL support installed for the BCM11xx APM and HSS.\n" );
   *paramp = &halAudioExternalFncs;
   return 0;
}


/** @} */
