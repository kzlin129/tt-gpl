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
*  @file    halaudio_external_stub.c
*
*  @brief   HAL Audio External routines.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include <linux/kernel.h> 
#include <linux/broadcom/halaudio.h>


/* ---- Public Variables ------------------------------------------------- */

/**
 * @addtogroup HAL
 * @{
 */

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

int halAudioExternalGetParams(const HAL_AUDIO_EXTERNAL_FNCS **paramp)
{
   printk( KERN_INFO "NO EXTERNAL CODEC SUPPORT\n");
   if ( paramp )
   {
      *paramp = NULL; /* Return NULL pointer to indicate no external fncs */
   }
   /* Returning 0 indicates no external codec. Internal Codec is to be used */
   return 0;
}

