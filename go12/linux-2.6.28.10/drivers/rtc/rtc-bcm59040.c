/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
#include <linux/io.h>

#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/rtc.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59040.h>
#include <linux/delay.h>
#include <linux/time.h>

#include "rtc.h"

#include <asm/arch/rtcHw.h>

#define RTC_BCM59040_MODULE_DESC     "Broadcom BCM59040 RTC Driver"
#define RTC_BCM59040_MODULE_VERSION  "1.1"

#define RTC_SET_PMU 0x1239
#define RTC_GET_PMU 0x123A

#define LEAPS_THRU_END_OF(y) ((y)/4 - (y)/100 + (y)/400)
#define LEAP_YEAR(year) ((!(year % 4) && (year % 100)) || !(year % 400))

#ifdef DEBUG
#define dprintk(s, arg...)	printk(s, arg)
#else
#define dprintk(s, arg...)
#endif

void bcm476x_rtc_update(int intType);

static DEFINE_SPINLOCK( bcm59040_476x_rtc_lock );
static void bcm59040_476x_rtc_setaie( int to );

static int bcm59040_476x_rtc_getalarm( struct device *dev, struct rtc_wkalrm *alrm );


static struct rtc_device *rtc4760;
static int freq4760;


static int
bcm59040_476x_rtc_setpie( struct device *dev, int enabled )
{
unsigned char val;
   spin_lock_irq( &bcm59040_476x_rtc_lock );

   if ( enabled )
    {
       val = (unsigned char) pmu_i2c_read(BCM59040_REG_INT5M);
 	   dprintk("BCM59040_REG_INT5M=%x\n",val);

       val &= ~(BCM59040_INT5_RTC1S );
       pmu_i2c_write( BCM59040_REG_INT5M, val);

    }
    else
    {
    	if (!signal_pending(current))
    	{
    		dprintk("From:%s enabled:%x\n",__FUNCTION__,enabled);

    		val = (unsigned char) pmu_i2c_read(BCM59040_REG_INT5M);
    		val |= BCM59040_INT5_RTC1S;
    		pmu_i2c_write( BCM59040_REG_INT5M, val);
    	}

    }
   spin_unlock_irq( &bcm59040_476x_rtc_lock );

   return 0;
}
static int
bcm59040_476x_rtc_setfreq( struct device *dev, int freq )
{
	freq4760 = 1;			//support only 1s
	return 0;
}

static int
bcm59040_476x_rtc_getfreq( struct device *dev, int *freq )
{

   *freq = freq4760;          //
   return 0;
}
/* Time read/write */

static int
bcm59040_476x_rtc_gettime( struct device *dev, struct rtc_time *rtc_tm )
{

u8 rtcReg[BCM59040_NUM_RTC_REG];

	pmu_i2c_read_bytes(BCM59040_REG_RTCSC, rtcReg, BCM59040_NUM_RTC_REG);

    rtc_tm->tm_sec = rtcReg[0];
    rtc_tm->tm_min = rtcReg[1];
    rtc_tm->tm_hour = rtcReg[2];
    rtc_tm->tm_mday = rtcReg[4];
    rtc_tm->tm_mon = rtcReg[5] - 1;
    rtc_tm->tm_year = rtcReg[6] + 100;

   dprintk( "read time 0x%02x.0x%02x.0x%02x 0x%02x/0x%02x/0x%02x\n",
             rtc_tm->tm_year, rtc_tm->tm_mon, rtc_tm->tm_mday, rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec );

   return 0;
}

static int
bcm59040_476x_rtc_settime( struct device *dev, struct rtc_time *time )
{
u8 rtcReg[BCM59040_NUM_RTC_REG];

    rtcReg[0] = time->tm_sec;
	rtcReg[1] = time->tm_min;
	rtcReg[2] = time->tm_hour;
	rtcReg[3] = time->tm_wday;
	rtcReg[4] = time->tm_mday;
	rtcReg[5] = time->tm_mon + 1;
	rtcReg[6] = (time->tm_year >= 100) ? time->tm_year - 100 : 0;

	pmu_i2c_write_bytes(BCM59040_REG_RTCSC, rtcReg, BCM59040_NUM_RTC_REG);

    dprintk( "set time %02d.%02d.%02d %02d/%02d/%02d\n", time->tm_year, time->tm_mon, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec );

   return 0;
}

static int alarm_enabled;

static int
bcm59040_476x_rtc_getalarm( struct device *dev, struct rtc_wkalrm *alrm )
{
struct rtc_time *alm_tm = &alrm->time;
u8 rtcReg[BCM59040_NUM_RTC_REG];

	pmu_i2c_read_bytes(BCM59040_REG_RTCSC_A1, rtcReg, BCM59040_NUM_RTC_REG);

	alm_tm->tm_sec = rtcReg[0];
	alm_tm->tm_min = rtcReg[1];
	alm_tm->tm_hour = rtcReg[2];
	alm_tm->tm_mday = rtcReg[4];
	alm_tm->tm_mon = rtcReg[5] - 1;
	alm_tm->tm_year = rtcReg[6] + 100;

	alrm->enabled = alarm_enabled;

	dprintk( "getalarm 0x%02x.0x%02x.0x%02x 0x%02x/0x%02x/0x%02x\n",
			alm_tm->tm_year, alm_tm->tm_mon, alm_tm->tm_mday, alm_tm->tm_hour, alm_tm->tm_min, alm_tm->tm_sec );

    return 0;
}

static int
bcm59040_476x_rtc_setalarm( struct device *dev, struct rtc_wkalrm *alrm )
{
struct rtc_time *time = &alrm->time;
u8 rtcReg[BCM59040_NUM_RTC_REG];
dprintk( "setalarm 0x%02x.0x%02x.0x%02x 0x%02x/0x%02x/0x%02x\n",
		time->tm_year, time->tm_mon, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec );

	rtcReg[0] = time->tm_sec;
	rtcReg[1] = time->tm_min;
	rtcReg[2] = time->tm_hour;
	rtcReg[3] = time->tm_wday;
	rtcReg[4] = time->tm_mday;
	rtcReg[5] = time->tm_mon + 1;
	rtcReg[6] = (time->tm_year >= 100) ? time->tm_year - 100 : 0;

	pmu_i2c_write_bytes(BCM59040_REG_RTCSC_A1, rtcReg, BCM59040_NUM_RTC_REG);

	if (alrm->enabled)
		alarm_enabled = 1;
	else
		alarm_enabled = 0;

    return 0;
}
static void
bcm59040_476x_rtc_setaie( int to )
{
unsigned char val;

   pr_debug( "%s: aie=%d\n", __func__, to );
   spin_lock_irq( &bcm59040_476x_rtc_lock );
   if ( to )
   {
	   val = (unsigned char) pmu_i2c_read(BCM59040_REG_INT5);
       val &= ~(BCM59040_INT5_RTCA1 );
       pmu_i2c_write( BCM59040_REG_INT5M, val);
       udelay(10);
   }
   else
   {
	   if (!signal_pending(current))
	   {
		    val = (unsigned char) pmu_i2c_read(BCM59040_REG_INT5M);
			val |= BCM59040_INT5_RTCA1;
			pmu_i2c_write( BCM59040_REG_INT5M, val);
	   }
   }
   spin_unlock_irq( &bcm59040_476x_rtc_lock );
}
static void
bcm59040_476x_startPeriodicTimer(void)
{
unsigned int val;
	val = (unsigned char) pmu_i2c_read(BCM59040_REG_INT5M);
	   dprintk("BCM59040_REG_INT5M=%x\n",val);

    val &= ~(BCM59040_INT5_RTC1S );
    pmu_i2c_write( BCM59040_REG_INT5M, val);

}
static void
bcm59040_476x_stopPeriodicTimer(void)
{
unsigned int val;
	val = (unsigned char) pmu_i2c_read(BCM59040_REG_INT5M);
		   dprintk("BCM59040_REG_INT5M=%x\n",val);

	    val |= (BCM59040_INT5_RTC1S );
	    pmu_i2c_write( BCM59040_REG_INT5M, val);

}

static int
bcm59040_476x_rtc_ioctl( struct device *dev, unsigned int cmd, unsigned long arg )
{

	switch ( cmd )
   {
      case RTC_AIE_OFF:
      {
          bcm59040_476x_rtc_setaie( 0 );
         return 0;
      }
      case RTC_AIE_ON:
      {
         bcm59040_476x_rtc_setaie( 1 );
         return 0;
      }
      case RTC_PIE_OFF:
      {
         bcm59040_476x_stopPeriodicTimer();
         return 0;
      }
      case RTC_PIE_ON:
      {
         bcm59040_476x_startPeriodicTimer();
         return 0;
      }
       case RTC_IRQP_READ:
      {
         int freq;
         int ret = bcm59040_476x_rtc_getfreq( dev, &freq );
         if ( ret != 0 )
         {
            return ret;
         }
         return put_user( freq, ( unsigned long * ) arg );
      }
      case RTC_IRQP_SET:
         return bcm59040_476x_rtc_setfreq( dev, ( int ) arg );
   }
   dprintk("From:%s cmd:%x task name=%s pid=%d\n",__FUNCTION__,cmd, current->comm, current->pid);
   return -ENOIOCTLCMD;
}

static void
bcm59040_476x_rtc_release( struct device *dev )
{
   if (!signal_pending(current))
   {
	   dprintk("From:%s\n",__FUNCTION__);
	   bcm59040_476x_rtc_setaie( 0 );
	   bcm59040_476x_rtc_setpie( dev, 0 );
   }
}

static const struct rtc_class_ops bcm59040_476x_rtcops = {
   .open = NULL,
   .release = bcm59040_476x_rtc_release,
   .ioctl = bcm59040_476x_rtc_ioctl,
   .read_time = bcm59040_476x_rtc_gettime,
   .set_time = bcm59040_476x_rtc_settime,
   .read_alarm = bcm59040_476x_rtc_getalarm,
   .set_alarm = bcm59040_476x_rtc_setalarm,
   .proc = NULL,
   .set_mmss = NULL,
   .irq_set_state = bcm59040_476x_rtc_setpie,
   .irq_set_freq = bcm59040_476x_rtc_setfreq,
   .read_callback = NULL,
};


static int 
bcm59040_476x_rtc_remove( struct platform_device *dev )
{
   struct rtc_device *rtc = platform_get_drvdata( dev );

   device_init_wakeup( &dev->dev, 0 );

   platform_set_drvdata( dev, NULL );

   bcm59040_476x_rtc_setpie( &dev->dev, 0 );
   bcm59040_476x_rtc_setaie( 0 );

   rtc_device_unregister( rtc );
   return 0;
}

static int
bcm59040_476x_rtc_probe( struct platform_device *pdev )
{
   int ret;


   pr_debug( "%s: probe=%p\n", __func__, pdev );

   device_init_wakeup( &pdev->dev, 1 );
   rtc4760 = rtc_device_register( "bcm59040_476x", &pdev->dev, &bcm59040_476x_rtcops, THIS_MODULE );

   if ( IS_ERR( rtc4760 ) )
   {
      ret = PTR_ERR( rtc4760 );
      pr_debug( "cannot attach rtc\n" );
      rtc_device_unregister( rtc4760 );
      return ret;;
   }
   freq4760 = 1;			//1s
   bcm59040_476x_rtc_setfreq( &pdev->dev, (int )freq4760 );
   platform_set_drvdata( pdev, rtc4760 );
   return 0;
}

#define bcm59040_476x_rtc_suspend NULL
#define bcm59040_476x_rtc_resume  NULL

static struct platform_driver bcm59040_476x_rtcdrv = {
   .remove = __exit_p( bcm59040_476x_rtc_remove ),
   .suspend = bcm59040_476x_rtc_suspend,
   .resume = bcm59040_476x_rtc_resume,
   .driver = {
              .name = "bcm59040_476x-rtc0",
              .owner = THIS_MODULE,
              },
};

static char __initdata banner[] = "BCM59040_476X RTC, (c) 2009 Broadcom Corporation\n";

static int __init
bcm59040_476x_rtc_init( void )
{
   printk( banner );
#ifndef CONFIG_PMU_DEVICE_BCM59040
   bcm59040_rtc_complete_register(&bcm476x_rtc_update);
#endif
   return platform_driver_probe( &bcm59040_476x_rtcdrv, bcm59040_476x_rtc_probe );
}

static void __exit
bcm59040_476x_rtc_exit( void )
{
#ifndef CONFIG_PMU_DEVICE_BCM59040
   bcm59040_rtc_complete_register(NULL);
#endif
   platform_driver_unregister( &bcm59040_476x_rtcdrv );
}


void bcm476x_rtc_update(int intType)
{
	if (intType == BCM59040_INT5_RTCA1)
	{
		rtc_update_irq( rtc4760, 1, RTC_AF | RTC_IRQF );
	}
	if (intType == BCM59040_INT5_RTC1S)
	{
		rtc_update_irq( rtc4760, 1, RTC_PF | RTC_IRQF );
	}
	if (intType == BCM59040_INT5_RTC60S)
	{
		rtc_update_irq( rtc4760, 1, RTC_PF | RTC_IRQF );
	}

}
EXPORT_SYMBOL (bcm476x_rtc_update);

rootfs_initcall( bcm59040_476x_rtc_init );
module_exit( bcm59040_476x_rtc_exit );

MODULE_DESCRIPTION( RTC_BCM59040_MODULE_DESC );
MODULE_AUTHOR( "Broadcom Corporation" );
MODULE_LICENSE( "GPL" );
MODULE_VERSION( RTC_BCM59040_MODULE_VERSION );
MODULE_ALIAS( "platform:bcm59040_476x-rtc" );


