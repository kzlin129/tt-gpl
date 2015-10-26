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

/* ---- Public Variables ------------------------------------------------- */

/**
 * @addtogroup HAL
 * @{
 */

/* ---- Public Variables ------------------------------------------------- */
extern int halAudioHSSCodecOffset;
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
extern HAL_AUDIO_ADDONMODULE halAudioHssModule;
/* ---- Private Functions ------------------------------------------------ */

/** @} */
