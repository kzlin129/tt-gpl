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
*  comctl.h
*
*  PURPOSE:
*
*  This file defines the interface to the Component Control driver for BCM1160.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( BCM1160_COMCTL_H )
#define BCM1160_COMCTL_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/ioctl.h>

/* ---- Constants and Types ---------------------------------------------- */

#define BCM1160_COMCTL_MAGIC   'X'

#define BCM1160_COMCTL_CMD_FIRST            0x80

#define BCM1160_COMCTL_CMD_ENABLE           0x80
#define BCM1160_COMCTL_CMD_DISABLE          0x81
#define BCM1160_COMCTL_CMD_IS_ENABLED       0x82
#define BCM1160_COMCTL_CMD_SET_SLEEP_MODE   0x83
#define BCM1160_COMCTL_CMD_GET_SLEEP_MODE   0x84
#define BCM1160_COMCTL_CMD_SET_POWER_SLEEP  0x85

#define BCM1160_COMCTL_CMD_LAST             0x85

#define BCM1160_COMCTL_IOCTL_ENABLE         _IO(  BCM1160_COMCTL_MAGIC, BCM1160_COMCTL_CMD_ENABLE )     // arg is component ID
#define BCM1160_COMCTL_IOCTL_DISABLE        _IO(  BCM1160_COMCTL_MAGIC, BCM1160_COMCTL_CMD_DISABLE )    // arg is component ID
#define BCM1160_COMCTL_IOCTL_IS_ENABLED     _IO(  BCM1160_COMCTL_MAGIC, BCM1160_COMCTL_CMD_IS_ENABLED ) // arg is component ID
#define BCM1160_COMCTL_IOCTL_SET_SLEEP_MODE _IO(  BCM1160_COMCTL_MAGIC, BCM1160_COMCTL_CMD_SET_SLEEP_MODE ) // arg is int
#define BCM1160_COMCTL_IOCTL_GET_SLEEP_MODE _IO(  BCM1160_COMCTL_MAGIC, BCM1160_COMCTL_CMD_GET_SLEEP_MODE ) // no arg
#define BCM1160_COMCTL_IOCTL_SET_POWER_SLEEP _IO(  BCM1160_COMCTL_MAGIC, BCM1160_COMCTL_CMD_SET_POWER_SLEEP ) // arg is int

#ifdef __KERNEL__

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */


void * comctl_allocate_client ( void );
void comctl_free_client ( void * client );

int comctl_set_sleep_mode ( void * client, int enable );    // stubbed so leave these here without CONFIG
int comctl_get_sleep_mode ( void * client );

#endif

#endif  /* BCM1160_COMCTL_H */


