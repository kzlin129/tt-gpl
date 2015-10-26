/*****************************************************************************
* Copyright 2004 - 2010 Broadcom Corporation.  All rights reserved.
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
*  This file defines the interface to the RTC driver.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( LINUX_BROADCOM_RTC_H )
#define LINUX_BROADCOM_RTC_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/ioctl.h>

typedef unsigned long rtc_time_t; // avoids many conflicts with time.h

/* ---- Constants and Types ---------------------------------------------- */

#define RTC_MAGIC   'r'

#define RTC_CMD_FIRST               0x80
#define RTC_CMD_GET_TIME            0x80
#define RTC_CMD_SET_TIME            0x81
#define RTC_CMD_GET_ALARM           0x82
#define RTC_CMD_SET_ALARM           0x83
#define RTC_CMD_ENABLE_ALARM_INT    0x84
#define RTC_CMD_DISABLE_ALARM_INT   0x85
#define RTC_CMD_RESET_GPS_TIME      0x86
#define RTC_CMD_IS_RTC_GPS_TIME     0x87
#define RTC_CMD_SET_GPS_TIME        0x88
#define RTC_CMD_GET_GPS_TIME        0x89
#define RTC_CMD_GET_HW_RTC          0x8A
#define RTC_CMD_LAST                0x8B

// time is in UTC time format
#define RTC_IOCTL_GET_TIME _IOR( RTC_MAGIC, RTC_CMD_GET_TIME, rtc_time_t )
#define RTC_IOCTL_SET_TIME _IOW( RTC_MAGIC, RTC_CMD_SET_TIME, rtc_time_t )
#define RTC_IOCTL_GET_ALARM _IOR( RTC_MAGIC, RTC_CMD_GET_ALARM, rtc_time_t )
#define RTC_IOCTL_SET_ALARM _IOW( RTC_MAGIC, RTC_CMD_SET_ALARM, rtc_time_t )
#define RTC_IOCTL_ENABLE_ALARM_INT _IO( RTC_MAGIC, RTC_CMD_ENABLE_ALARM_INT )
#define RTC_IOCTL_DISABLE_ALARM_INT _IO( RTC_MAGIC, RTC_CMD_DISABLE_ALARM_INT )
#define RTC_IOCTL_RESET_GPS_TIME _IOR( RTC_MAGIC, RTC_CMD_RESET_GPS_TIME, int )
#define RTC_IOCTL_IS_RTC_GPS_TIME _IOR( RTC_MAGIC, RTC_CMD_IS_RTC_GPS_TIME, int )
#define RTC_IOCTL_SET_GPS_TIME _IOW( RTC_MAGIC, RTC_CMD_SET_GPS_TIME, struct rtc_time )
#define RTC_IOCTL_GET_GPS_TIME _IOR( RTC_MAGIC, RTC_CMD_GET_GPS_TIME, struct rtc_time )
#define RTC_IOCTL_GET_HW_RTC _IOR( RTC_MAGIC, RTC_CMD_GET_HW_RTC, struct rtc_time )

/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

#endif  /* LINUX_RTC_H */
