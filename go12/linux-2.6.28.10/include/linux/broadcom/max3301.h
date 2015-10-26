/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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
*  max3301.h
*
*  PURPOSE:
*
*   This file describes the interface to the USBI2C driver.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_MAX3301_H )
#define LINUX_MAX3301_H

/* ---- Include Files ---------------------------------------------------- */

/* ---- Constants and Types ---------------------------------------------- */

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

#if defined( __KERNEL__ )

#include <linux/module.h>

int __init max3301_init( void );
void __exit max3301_cleanup( void );

#endif  // __KERNEL__

#endif  /* LINUX_MAX3301_H */
