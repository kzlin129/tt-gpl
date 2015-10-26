/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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
*  rtc.h
*
*  PURPOSE:
*
*  This file defines the internal rtc interface to the Broadcom BCM59001 PMU chip
*
*  NOTES:
*
*****************************************************************************/


#if !defined( BCM59040_RTC_H )
#define BCM59040_RTC_H

/* ---- Include Files ---------------------------------------------------- */

//#include <linux/broadcom/pmu_chip.h>

/* ---- Constants and Types ---------------------------------------------- */

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

int rtc59040_init( void );

#endif  /* BCM59001_RTC_H */


