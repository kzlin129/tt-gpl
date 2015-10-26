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
*     This implements the real time clock interface for the PCF506xx chips.
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
#include <linux/broadcom/pmu_pcf506xx.h>
#include <linux/delay.h>
#include <linux/time.h>

#include "rtc.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */

static BCM_PMU_Chip_t gPcfChip = PMU_NUM_CHIPS;

/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  rtc506xx_ioctl
*
***************************************************************************/
static int rtc506xx_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
    rtc_time_t utc;
    unsigned char val;

    switch ( cmd )
    {
        case RTC_IOCTL_GET_TIME:
            // Assume this represents seconds since 1970
           utc = 0;
           val = pmu_i2c_read( PCF506XX_REG_RTC4(gPcfChip) );
           utc += ((val & 0xff) << 24);
           val = pmu_i2c_read( PCF506XX_REG_RTC3(gPcfChip) );
           utc += ((val & 0xff) << 16);
           val = pmu_i2c_read( PCF506XX_REG_RTC2(gPcfChip) );
           utc += ((val & 0xff) << 8);
           val = pmu_i2c_read( PCF506XX_REG_RTC1(gPcfChip) );
           utc += (val & 0xff);
           if( copy_to_user( (unsigned long *)arg, &utc, sizeof( utc ) ) != 0 )
           {
              return -EFAULT;
           }
           break;

        case RTC_IOCTL_SET_TIME:
            if ( copy_from_user( &utc, (rtc_time_t *)arg, sizeof( rtc_time_t )) != 0 )
            {
                return -EFAULT;
            }
            val = (unsigned char) (utc & 0xff);
            pmu_i2c_write( PCF506XX_REG_RTC1(gPcfChip), val );
            val = (unsigned char)(utc >>= 8);
            pmu_i2c_write( PCF506XX_REG_RTC2(gPcfChip), val );
            val = (unsigned char)(utc >>= 8);
            pmu_i2c_write( PCF506XX_REG_RTC3(gPcfChip), val );
            val = (unsigned char)(utc >>= 8);
            pmu_i2c_write( PCF506XX_REG_RTC4(gPcfChip), val );
            break;

        case RTC_IOCTL_GET_ALARM:
           utc = 0;
           val = pmu_i2c_read( PCF506XX_REG_RTC4A(gPcfChip) );
           utc += ((val & 0xff) << 24);
           val = pmu_i2c_read( PCF506XX_REG_RTC3A(gPcfChip) );
           utc += ((val & 0xff) << 16);
           val = pmu_i2c_read( PCF506XX_REG_RTC2A(gPcfChip) );
           utc += ((val & 0xff) << 8);
           val = pmu_i2c_read( PCF506XX_REG_RTC1A(gPcfChip) );
           utc += (val & 0xff);
           if( copy_to_user( (unsigned long *)arg, &utc, sizeof( rtc_time_t ) ) != 0 )
           {
              return -EFAULT;
           }
           break;

        case RTC_IOCTL_SET_ALARM:
            if ( copy_from_user( &utc, (rtc_time_t *)arg, sizeof( rtc_time_t )) != 0 )
            {
                return -EFAULT;
            }

            val = (unsigned char) (utc & 0xff);
            pmu_i2c_write( PCF506XX_REG_RTC1A(gPcfChip), val );
            val = (unsigned char)(utc >>= 8);
            pmu_i2c_write( PCF506XX_REG_RTC2A(gPcfChip), val );
            val = (unsigned char)(utc >>= 8);
            pmu_i2c_write( PCF506XX_REG_RTC3A(gPcfChip), val );
            val = (unsigned char)(utc >>= 8);
            pmu_i2c_write( PCF506XX_REG_RTC4A(gPcfChip), val );

            // Enable Wakeup on RTC Alarm
            val = (unsigned char) pmu_i2c_read( PCF506XX_REG_OOCC(gPcfChip) );
            val |= PCF506XX_BIT_RTC_WAK;
            pmu_i2c_write( PCF506XX_REG_OOCC(gPcfChip), val);
            break;

        case RTC_IOCTL_ENABLE_ALARM_INT:
            val = (unsigned char) pmu_i2c_read( PCF506XX_REG_INT1M(gPcfChip));
            val &= ~PCF506XX_BIT_ALARM;
            pmu_i2c_write( PCF506XX_REG_INT1M(gPcfChip), val);
            break;

        case RTC_IOCTL_DISABLE_ALARM_INT:
            val = (unsigned char) pmu_i2c_read(PCF506XX_REG_INT1M(gPcfChip));
            val |= PCF506XX_BIT_ALARM;
            pmu_i2c_write( PCF506XX_REG_INT1M(gPcfChip), val);
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

struct file_operations rtc506xx_fops =
{
    owner:      THIS_MODULE,
    ioctl:      rtc506xx_ioctl,
};

/****************************************************************************
*
*  rtc506xx_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

int rtc506xx_init( BCM_PMU_Chip_t chip )
{
    int rc;

    if ((chip != PMU_PCF50603) && (chip != PMU_PCF50611))
    {
       printk( "rtc506xx: chip ID %d not supported.\n", chip);
       return -ENODEV;
    }

    if (gPcfChip != PMU_NUM_CHIPS)
    {
       printk( "rtc506xx: driver already loaded for chip %d.\n", gPcfChip);
       return -EBUSY;
    }

    printk( "rtc506xx: register_chrdev\n");
    if (( rc = register_chrdev( BCM_RTC_MAJOR, "rtc", &rtc506xx_fops )) < 0 )
    {
        printk( "rtc506xx: register_chrdev failed for major %d\n", BCM_RTC_MAJOR );
        return rc;
    }

    /* Save chip ID */
    gPcfChip = chip;

    return 0;
}


