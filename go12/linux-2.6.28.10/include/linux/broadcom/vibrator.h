/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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
*  vibrator.h
*
*  PURPOSE:
*
*  This file defines the interface to the VIBRATOR driver.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_VIBRATOR_H )
#define LINUX_VIBRATOR_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/ioctl.h>

/* ---- Constants and Types ---------------------------------------------- */

#define VIBRATOR_MAGIC   'v'

#define VIBRATOR_CMD_FIRST               0x80
#define VIBRATOR_CMD_ENABLE              0x80
#define VIBRATOR_CMD_DISABLE             0x81
#define VIBRATOR_CMD_LAST                0x81

#define VIBRATOR_IOCTL_ENABLE   _IO( VIBRATOR_MAGIC, VIBRATOR_CMD_ENABLE )
#define VIBRATOR_IOCTL_DISABLE  _IO( VIBRATOR_MAGIC, VIBRATOR_CMD_DISABLE )

/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

#endif  /* LINUX_VIBRATOR_H */
