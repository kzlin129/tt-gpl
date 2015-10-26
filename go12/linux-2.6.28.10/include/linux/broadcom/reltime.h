/*****************************************************************************
* Copyright 2005 - 2008 Broadcom Corporation.  All rights reserved.
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
*  reltime.c
*
*  PURPOSE:
*
*     This implements a relative time driver used to retrieve a time
*     value not affected by changes to the system time (settime()).
*     Basically, this driver used used in the same way a call to
*     clock_gettime() with clock type CLOCK_MONOTONIC would work if
*     that clock type was available on our system.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_RELTIME_H )
#define LINUX_RELTIME_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/ioctl.h>

/* ---- Constants and Types ---------------------------------------------- */

typedef struct timespec Timespec_t;

#define RELTIME_MAGIC   'R'

#define RELTIME_CMD_FIRST               0x80

#define RELTIME_CMD_GET                 0x80
#define RELTIME_CMD_DIFF                0x81
#define RELTIME_CMD_LAST                0x81

#define RELTIME_IOCTL_GET   _IOR ( RELTIME_MAGIC, RELTIME_CMD_GET,  Timespec_t)
#define RELTIME_IOCTL_DIFF  _IOWR( RELTIME_MAGIC, RELTIME_CMD_DIFF, Timespec_t)

#endif  /* LINUX_RELTIME_H */
