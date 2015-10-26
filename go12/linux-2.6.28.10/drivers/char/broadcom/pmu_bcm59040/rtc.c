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
*  rtc.c
*
*  PURPOSE:
*
*     This implements the real time clock interface for the BCM59040 chips.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */
#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/sysctl.h>
#include <linux/rtc.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/rtc.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59040.h>
#include <linux/delay.h>
#include <linux/time.h>

#include "rtc.h"

#define LEAPS_THRU_END_OF(y) ((y)/4 - (y)/100 + (y)/400)
#define LEAP_YEAR(year) ((!(year % 4) && (year % 100)) || !(year % 400))

static void rtc59040_utc_to_tm(unsigned long time, struct rtc_time *tm);

static int rtc59040_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
    rtc_time_t utc;
    struct rtc_time time;
	unsigned char val;
	u8 rtcReg[BCM59040_NUM_RTC_REG];

   //printk("type: '%c' cmd: 0x%x\n", _IOC_TYPE( cmd ), _IOC_NR( cmd ));

    switch ( _IOC_NR(cmd) )
    {
        case _IOC_NR(RTC_IOCTL_GET_TIME):
           pmu_i2c_read_bytes(BCM59040_REG_RTCSC, rtcReg, BCM59040_NUM_RTC_REG);

           time.tm_sec = rtcReg[0];
		   time.tm_min = rtcReg[1];
		   time.tm_hour = rtcReg[2];
		   time.tm_mday = rtcReg[4];
		   time.tm_mon = rtcReg[5];
		   time.tm_year = rtcReg[6];

		   utc = mktime(time.tm_year + 2000, time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);

           if( copy_to_user( (unsigned long *)arg, &utc, sizeof( rtc_time_t ) ) != 0 )
           {
              return -EFAULT;
           }

           break;

        case _IOC_NR(RTC_IOCTL_SET_TIME):
            if ( copy_from_user( &utc, (rtc_time_t *)arg, sizeof( rtc_time_t )) != 0 )
            {
                printk("RTC_IOCTL_SET_TIME copy_from_user failed\n");
                return -EFAULT;
            }
            
			rtc59040_utc_to_tm(utc, &time);     

			rtcReg[0] = time.tm_sec;
			rtcReg[1] = time.tm_min;
			rtcReg[2] = time.tm_hour;
			rtcReg[3] = time.tm_wday;
			rtcReg[4] = time.tm_mday;
			rtcReg[5] = time.tm_mon;
			rtcReg[6] = time.tm_year;

			pmu_i2c_write_bytes(BCM59040_REG_RTCSC, rtcReg, BCM59040_NUM_RTC_REG);

            break;

        case _IOC_NR(RTC_IOCTL_GET_ALARM):
           pmu_i2c_read_bytes(BCM59040_REG_RTCSC_A1, rtcReg, BCM59040_NUM_RTC_REG);

		   time.tm_sec = rtcReg[0];
		   time.tm_min = rtcReg[1];
		   time.tm_hour = rtcReg[2];
		   time.tm_mday = rtcReg[4];
		   time.tm_mon = rtcReg[5];
		   time.tm_year = rtcReg[6];

		   utc = mktime(time.tm_year + 2000, time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);

           if( copy_to_user( (unsigned long *)arg, &utc, sizeof( rtc_time_t ) ) != 0 )
           {
              printk("RTC_IOCTL_GET_ALARM copy_to_user failed\n");
              return -EFAULT;
           }

           break;

        case _IOC_NR(RTC_IOCTL_SET_ALARM):
            if ( copy_from_user( &utc, (rtc_time_t *)arg, sizeof( rtc_time_t )) != 0 )
            {
                printk("RTC_IOCTL_SET_ALARM copy_from_user failed\n");
                return -EFAULT;
            }
            rtc59040_utc_to_tm(utc, &time);     

			rtcReg[0] = time.tm_sec;
			rtcReg[1] = time.tm_min;
			rtcReg[2] = time.tm_hour;
			rtcReg[3] = time.tm_wday;
			rtcReg[4] = time.tm_mday;
			rtcReg[5] = time.tm_mon;
			rtcReg[6] = time.tm_year;

			pmu_i2c_write_bytes(BCM59040_REG_RTCSC_A1, rtcReg, BCM59040_NUM_RTC_REG);

            break;

        case _IOC_NR(RTC_IOCTL_ENABLE_ALARM_INT):
            val = (unsigned char) pmu_i2c_read(BCM59040_REG_INT5M);
            val &= ~BCM59040_INT5_RTCA1;
            pmu_i2c_write( BCM59040_REG_INT5M, val);
            break;

        case _IOC_NR(RTC_IOCTL_DISABLE_ALARM_INT):
            val = (unsigned char) pmu_i2c_read(BCM59040_REG_INT5M);
            val |= BCM59040_INT5_RTCA1;
            pmu_i2c_write( BCM59040_REG_INT5M, val);
            break;

        default:
            printk("rtc_ioctl - Unrecognized ioctl: '0x%x'\n", cmd);
            return -ENOTTY;
    }
    return 0;
}


/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/
struct file_operations rtc59040_fops =
{
    owner:      THIS_MODULE,
    ioctl:      rtc59040_ioctl,
};

/****************************************************************************
*
*  rtc59001_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

int rtc59040_init( void )
{
    int rc;
    printk( "rtc59040: register_chrdev\n");
    if (( rc = register_chrdev( BCM_RTC_MAJOR, "rtc", &rtc59040_fops )) < 0 )
    {
        printk( "rtc59040: register_chrdev failed for major %d\n", BCM_RTC_MAJOR );
        return rc;
    }
    return 0;
}


/*
 * Convert seconds since 01-01-1970 00:00:00 to Gregorian date.
 * For PMU59040, 00 in RTCY register means year 2000.
 */
static void rtc59040_utc_to_tm(unsigned long time, struct rtc_time *tm)
{
	unsigned int month, year;
	int days;

	days = time / 86400;  //1 day = 86400s
	time -= (unsigned int) days * 86400;

	/* day of the week, 1970-01-01 was a Thursday */
	tm->tm_wday = (days + 4) % 7;

	year = 1970 + days / 365;
	days -= (year - 1970) * 365
		+ LEAPS_THRU_END_OF(year - 1)
		- LEAPS_THRU_END_OF(1970 - 1);
	if (days < 0) {
		year -= 1;
		days += 365 + LEAP_YEAR(year);
	}
	tm->tm_year = year - 2000;
	tm->tm_yday = days + 1;

	for (month = 0; month < 11; month++) {
		int newdays;

		newdays = days - rtc_month_days(month, year);
		if (newdays < 0)
			break;
		days = newdays;
	}
	tm->tm_mon = month + 1;
	tm->tm_mday = days + 1;

	tm->tm_hour = time / 3600;
	time -= tm->tm_hour * 3600;
	tm->tm_min = time / 60;
	tm->tm_sec = time - tm->tm_min * 60;
}
