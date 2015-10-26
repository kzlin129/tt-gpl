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
*     This implements the real time clock interface for the BCM59001 chips.
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
#include <linux/broadcom/pmu_bcm59001.h>
#include <linux/delay.h>
#include <linux/time.h>

#include "rtc.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

static int rtc59001_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
    rtc_time_t utc;
    struct rtc_time time;
    unsigned char val;

    switch ( cmd )
    {
        case RTC_IOCTL_GET_TIME:
           time.tm_year = pmu_i2c_read( BCM59001_REG_RTCYR );
           time.tm_mon = pmu_i2c_read( BCM59001_REG_RTCMT ) - 1;
           time.tm_mday = pmu_i2c_read( BCM59001_REG_RTCDT );
           time.tm_hour = pmu_i2c_read( BCM59001_REG_RTCHR );
           time.tm_min = pmu_i2c_read( BCM59001_REG_RTCMN );
           time.tm_sec = pmu_i2c_read( BCM59001_REG_RTCSC );

           rtc_tm_to_time(&time, &utc);

           if( copy_to_user( (unsigned long *)arg, &utc, sizeof( rtc_time_t ) ) != 0 )
           {
              return -EFAULT;
           }
           break;

        case RTC_IOCTL_SET_TIME:
            if ( copy_from_user( &utc, (rtc_time_t *)arg, sizeof( rtc_time_t )) != 0 )
            {
                printk("RTC_IOCTL_SET_TIME copy_from_user failed\n");
                return -EFAULT;
            }
            rtc_time_to_tm(utc, &time);     // tm_mon is in 0.. form here

            pmu_i2c_write( BCM59001_REG_RTCYR, time.tm_year );
            pmu_i2c_write( BCM59001_REG_RTCMT, (time.tm_mon + 1) );  // pmu expects 1.. numbering here
            pmu_i2c_write( BCM59001_REG_RTCDT, time.tm_mday );
            pmu_i2c_write( BCM59001_REG_RTCHR, time.tm_hour );
            pmu_i2c_write( BCM59001_REG_RTCMN, time.tm_min );
            pmu_i2c_write( BCM59001_REG_RTCSC, time.tm_sec );
            break;

        case RTC_IOCTL_GET_ALARM:
           time.tm_year = pmu_i2c_read( BCM59001_REG_RTCYR_A1 );
           time.tm_mon = pmu_i2c_read( BCM59001_REG_RTCMT_A1 ) - 1;
           time.tm_mday = pmu_i2c_read( BCM59001_REG_RTCDT_A1 );
           time.tm_hour = pmu_i2c_read( BCM59001_REG_RTCHR_A1 );
           time.tm_min = pmu_i2c_read( BCM59001_REG_RTCMN_A1 );
           time.tm_sec = pmu_i2c_read( BCM59001_REG_RTCSC_A1 );

           rtc_tm_to_time(&time, &utc);

           if( copy_to_user( (unsigned long *)arg, &utc, sizeof( rtc_time_t ) ) != 0 )
           {
              printk("RTC_IOCTL_GET_ALARM copy_to_user failed\n");
              return -EFAULT;
           }
           break;

        case RTC_IOCTL_SET_ALARM:
            if ( copy_from_user( &utc, (rtc_time_t *)arg, sizeof( rtc_time_t )) != 0 )
            {
                printk("RTC_IOCTL_SET_ALARM copy_from_user failed\n");
                return -EFAULT;
            }
            rtc_time_to_tm(utc, &time);     // tm_mon is in 0.. form here

            pmu_i2c_write( BCM59001_REG_RTCYR_A1, time.tm_year );
            pmu_i2c_write( BCM59001_REG_RTCMT_A1, (time.tm_mon + 1) );  // pmu expects 1.. numbering here
            pmu_i2c_write( BCM59001_REG_RTCDT_A1, time.tm_mday );
            pmu_i2c_write( BCM59001_REG_RTCHR_A1, time.tm_hour );
            pmu_i2c_write( BCM59001_REG_RTCMN_A1, time.tm_min );
            pmu_i2c_write( BCM59001_REG_RTCSC_A1, time.tm_sec );
            break;

        case RTC_IOCTL_ENABLE_ALARM_INT:
            val = (unsigned char) pmu_i2c_read(BCM59001_REG_INT1M);
            val &= ~BCM59001_BIT_INT1_RTCA1;
            pmu_i2c_write( BCM59001_REG_INT1M, val);
            break;

        case RTC_IOCTL_DISABLE_ALARM_INT:
            val = (unsigned char) pmu_i2c_read(BCM59001_REG_INT1M);
            val |= BCM59001_BIT_INT1_RTCA1;
            pmu_i2c_write( BCM59001_REG_INT1M, val);
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
struct file_operations rtc59001_fops =
{
    owner:      THIS_MODULE,
    ioctl:      rtc59001_ioctl,
};

/****************************************************************************
*
*  rtc59001_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

int rtc59001_init( void )
{
    int rc;
    printk( "rtc59001: register_chrdev\n");
    if (( rc = register_chrdev( BCM_RTC_MAJOR, "rtc", &rtc59001_fops )) < 0 )
    {
        printk( "rtc59001: register_chrdev failed for major %d\n", BCM_RTC_MAJOR );
        return rc;
    }
    return 0;
}

