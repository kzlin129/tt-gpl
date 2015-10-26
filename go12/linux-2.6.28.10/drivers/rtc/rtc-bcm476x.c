/* drivers/rtc/rtc-bcm476x.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * BCM476X RTC Driver
*/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <asm/arch/rtcHw.h>
#include <asm/arch/rtcHw_reg.h>

#define RTC_BCM4760_MODULE_DESC     "Broadcom BCM476X RTC Driver"
#define RTC_BCM4760_MODULE_VERSION  "1.1"

static unsigned int epoch = 1970;
static DEFINE_SPINLOCK( bcm476x_rtc_lock );

typedef struct _bcm476x_rtc_ctxt
{
	struct rtc_device *rtc;
	unsigned int one_shot_irq;
	unsigned int periodic_irq;
} bcm476x_rtc_ctxt_t;

/* IRQ Handlers */

/*
 * RTC IRQ hanlder. This routine is invoked when a RTC oneshot timer completes
 */
static irqreturn_t
rtc_alm_isr( int irq, void *data )
{
   struct rtc_device *rdev = data;
//   uint32_t intstatus;

   rtcHw_clearOneshotTimerInterrupt();
   pr_debug( "%s: oneshot interrupted\n", __func__ );
   rtc_update_irq( rdev, 1, RTC_AF | RTC_IRQF );
	 
	/* Clear VIC1 interrupt */
    writel(0, IO_ADDRESS(VIC1_R_VICADDRESS_MEMADDR));
	/* Clear VIC0 interrupt because VIC1 is daisy chained to VIC0 */
    writel(0, IO_ADDRESS(VIC0_R_VICADDRESS_MEMADDR));
		
   return IRQ_HANDLED;
}

/*
 * RTC IRQ hanlder. This routine is invoked when periodic interrupts occur
 */
static irqreturn_t
rtc_per_isr( int irq, void *data )
{
   struct rtc_device *rdev = data;
//   uint32_t intstatus;

   rtcHw_clearPeriodicTimerInterrupt();
   pr_debug( "%s: periodic interrupted\n", __func__ );
   rtc_update_irq( rdev, 1, RTC_PF | RTC_IRQF );
	 
	/* Clear VIC0 interrupt */
    writel(0, IO_ADDRESS(VIC0_R_VICADDRESS_MEMADDR));
		
   return IRQ_HANDLED;
}

/* Update control registers */
static void
bcm476x_rtc_setaie( int to )
{
   pr_debug( "%s: aie=%d\n", __func__, to );

   if ( to )
   {
      rtcHw_enableOneshotInt();
   }
   else
   {
      rtcHw_disableOneshotInt();
   }
}

static int
bcm476x_rtc_setpie( struct device *dev, int enabled )
{
   pr_debug( "%s: pie=%d\n", __func__, enabled );

   spin_lock_irq( &bcm476x_rtc_lock );

   if ( enabled )
   {
      rtcHw_startPeriodicTimer(); // enables the interrupt
   }
   else
   {
      rtcHw_stopPeriodicTimer();  // disables the interrupt
   }
   spin_unlock_irq( &bcm476x_rtc_lock );

   return 0;
}

static int
bcm476x_rtc_setfreq( struct device *dev, int freq )
{
   rtcHw_INTERVAL_e interval = 0xffffffff;   // invalid

   pr_debug( "%s: freq=%d\n", __func__, freq );
   spin_lock_irq( &bcm476x_rtc_lock );
   switch ( freq )
   {
      case 1:
         interval = rtcHw_INTERVAL_1000ms;
         break;
      case 2:
         interval = rtcHw_INTERVAL_2000ms;
         break;
      case 4:
         interval = rtcHw_INTERVAL_4000ms;
         break;
      case 8:
         interval = rtcHw_INTERVAL_8000ms;
         break;
      case 16:
         interval = rtcHw_INTERVAL_16000ms;
         break;
      case 32:
         interval = rtcHw_INTERVAL_32000ms;
         break;
      case 64:
         interval = rtcHw_INTERVAL_64000ms;
         break;
      case 128:
         interval = rtcHw_INTERVAL_128000ms;
         break;
   }
   spin_unlock_irq( &bcm476x_rtc_lock );

   if ( interval != 0xffffffff )
   {
      pr_debug( "%s: OKAY freq=%d interval=%d\n", __func__, freq, interval );
      rtcHw_setPeriodicTimerInterval( interval );
   }
   else
   {
      pr_debug( "%s: BAD freq=%d\n", __func__, freq );
      return -EINVAL;
   }
   return 0;
}

int
bcm476x_rtc_getfreq( struct device *dev, int *freq )
{
   rtcHw_INTERVAL_e interval;
   *freq = 0xffffffff;          // invalid

   spin_lock_irq( &bcm476x_rtc_lock );
   interval = rtcHw_readReg( rtchw_PERIODIC_TIMER_ADDR );
   switch ( interval )
   {
      case rtcHw_INTERVAL_125ms:   // avoid compiler warnings
      case rtcHw_INTERVAL_250ms:
      case rtcHw_INTERVAL_500ms:
         break;
      case rtcHw_INTERVAL_1000ms:
         *freq = 1;
         break;
      case rtcHw_INTERVAL_2000ms:
         *freq = 2;
         break;
      case rtcHw_INTERVAL_4000ms:
         *freq = 4;
         break;
      case rtcHw_INTERVAL_8000ms:
         *freq = 8;
         break;
      case rtcHw_INTERVAL_16000ms:
         *freq = 16;
         break;
      case rtcHw_INTERVAL_32000ms:
         *freq = 32;
         break;
      case rtcHw_INTERVAL_64000ms:
         *freq = 64;
         break;
      case rtcHw_INTERVAL_128000ms:
         *freq = 128;
         break;
   }
   spin_unlock_irq( &bcm476x_rtc_lock );
   if ( *freq == 0xffffffff )
   {
      pr_debug( "%s: Bad interval=%d\n", __func__, interval );
      return -EINVAL;
   }
   pr_debug( "%s: interval=%d, freq=%d\n", __func__, interval, *freq );
   return 0;
}

/* Time read/write */

static int
bcm476x_rtc_gettime( struct device *dev, struct rtc_time *rtc_tm )
{
   unsigned int epoch_sec, elapsed_sec;

   epoch_sec = mktime( epoch, 1, 1, 0, 0, 0 );
   elapsed_sec = rtcHw_getTime();

   pr_debug( "%s: epoch_sec=%u, elapsed_sec=%u\n", __func__, epoch_sec, elapsed_sec );
   rtc_time_to_tm( epoch_sec + elapsed_sec, rtc_tm );

   pr_debug( "read time 0x%02x.0x%02x.0x%02x 0x%02x/0x%02x/0x%02x\n",
             rtc_tm->tm_year, rtc_tm->tm_mon, rtc_tm->tm_mday, rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec );

   return 0;
}

static int
bcm476x_rtc_settime( struct device *dev, struct rtc_time *time )
{
   unsigned int epoch_sec, current_sec;

   epoch_sec = mktime( epoch, 1, 1, 0, 0, 0 );
   current_sec = mktime( time->tm_year + 1900, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec );

   rtcHw_setTime( current_sec - epoch_sec );

   pr_debug( "%s: current_sec=%u, epoch_sec=%u\n", __func__, current_sec, epoch_sec );

   pr_debug( "set time %02d.%02d.%02d %02d/%02d/%02d\n", time->tm_year, time->tm_mon, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec );

   return 0;
}

int
bcm476x_rtc_getalarm( struct device *dev, struct rtc_wkalrm *alrm )
{
   unsigned int epoch_sec, elapsed_sec, alarm_elapsed_sec;
   rtcHw_TIME_t alm_reg_secs;
   struct rtc_time *alm_tm = &alrm->time;
   alrm->enabled = rtcHw_isOneshotEnabled();
   alrm->pending = ( rtcHw_readReg( rtchw_INTERRUPT_STATUS_ADDR ) & rtchw_CMD_ONESHOT_INTERRUPT_STATUS ) ? 1 : 0;

   epoch_sec = mktime( epoch, 1, 1, 0, 0, 0 );
   elapsed_sec = rtcHw_getTime() & ~0xffff;   // clear lower 16 bits for 16-bit alarm match register below

   alm_reg_secs = rtcHw_getOneshotTimer();
   alarm_elapsed_sec = elapsed_sec + alm_reg_secs;
   pr_debug( "%s: epoch_sec=%u, elapsed_sec=%u, alm_reg_secs=%u=0x%x, alarm_elapsed_sec=%u=0x%x\n",
             __func__, epoch_sec, elapsed_sec, alm_reg_secs, alm_reg_secs, alarm_elapsed_sec, alarm_elapsed_sec );

   rtc_time_to_tm( epoch_sec + alarm_elapsed_sec, alm_tm );
   pr_debug( "read alarm %02x %02x.%02x.%02x %02x/%02x/%02x\n",
             alrm->enabled, alm_tm->tm_year, alm_tm->tm_mon, alm_tm->tm_mday, alm_tm->tm_hour, alm_tm->tm_min, alm_tm->tm_sec );

   return 0;
}


static int
bcm476x_rtc_setalarm( struct device *dev, struct rtc_wkalrm *alrm )
{
   unsigned int epoch_sec, elapsed_sec;
   struct rtc_time *time = &alrm->time;
   rtcHw_TIME_t alm_secs;

   pr_debug( "%s: %d, %02x/%02x/%02x %02x.%02x.%02x\n",
             __func__, alrm->enabled, time->tm_mday & 0xff, time->tm_mon & 0xff, time->tm_year & 0xff, time->tm_hour & 0xff, time->tm_min & 0xff,
             time->tm_sec );

   epoch_sec = mktime( epoch, 1, 1, 0, 0, 0 );
   elapsed_sec = rtcHw_getTime();
   alm_secs = mktime( time->tm_year + 1900, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec );

   pr_debug( "%s: epoch_sec=%u, elapsed_sec=%u, alm_secs=%u\n", __func__, epoch_sec, elapsed_sec, alm_secs );

   rtcHw_setOneshotTimer( alm_secs );

   if (alrm->enabled)
   	rtcHw_enableOneshotInt();
   else
	rtcHw_disableOneshotInt();

   return 0;
}

int
bcm476x_rtc_proc( struct device *dev, struct seq_file *seq )
{
   seq_printf( seq, "\nperiodic timer: 0x%x\n", rtcHw_readReg( rtchw_PERIODIC_TIMER_ADDR ) );
   seq_printf( seq, "match register: 0x%x\n", rtcHw_readReg( rtchw_MATCH_REGISTER_ADDR ) );
   seq_printf( seq, "rtc register: 0x%x\n", rtcHw_readReg( rtchw_RTC_REGISTER_ADDR ) );
   seq_printf( seq, "clear intr register: 0x%x\n", rtcHw_readReg( rtchw_CLEAR_INTR_ADDR ) );
   seq_printf( seq, "current time register: 0x%x\n", rtcHw_readReg( rtchw_CURRENT_TIME_ADDR ) );
   seq_printf( seq, "intr status register: 0x%x\n", rtcHw_readReg( rtchw_INTERRUPT_STATUS_ADDR ) );
   seq_printf( seq, "control addr register: 0x%x\n", rtcHw_readReg( rtchw_CONTROL_ADDR ) );
   return 0;
}

static int
bcm476x_rtc_ioctl( struct device *dev, unsigned int cmd, unsigned long arg )
{
   switch ( cmd )
   {
      case RTC_AIE_OFF:
         spin_lock_irq( &bcm476x_rtc_lock );
         bcm476x_rtc_setaie( 0 );
         spin_unlock_irq( &bcm476x_rtc_lock );
         return 0;
      case RTC_AIE_ON:
         spin_lock_irq( &bcm476x_rtc_lock );
         bcm476x_rtc_setaie( 1 );
         spin_unlock_irq( &bcm476x_rtc_lock );
         return 0;
      case RTC_PIE_OFF:
         spin_lock_irq( &bcm476x_rtc_lock );
         rtcHw_stopPeriodicTimer();
         spin_unlock_irq( &bcm476x_rtc_lock );
         return 0;
      case RTC_PIE_ON:
         spin_lock_irq( &bcm476x_rtc_lock );
         rtcHw_startPeriodicTimer();
         spin_unlock_irq( &bcm476x_rtc_lock );
         return 0;
      case RTC_IRQP_READ:
      {
         int freq;
         int ret = bcm476x_rtc_getfreq( dev, &freq );
         if ( ret != 0 )
         {
            return ret;
         }
         return put_user( freq, ( unsigned long * ) arg );
      }
      case RTC_IRQP_SET:
         return bcm476x_rtc_setfreq( dev, ( int ) arg );
   }
   return -ENOIOCTLCMD;
}

static void
bcm476x_rtc_release( struct device *dev )
{
   bcm476x_rtc_setaie( 0 );
   bcm476x_rtc_setpie( dev, 0 );
}

static const struct rtc_class_ops bcm476x_rtcops = {
   .ioctl = bcm476x_rtc_ioctl,
   .release = bcm476x_rtc_release,
   .read_time = bcm476x_rtc_gettime,
   .set_time = bcm476x_rtc_settime,
   .read_alarm = bcm476x_rtc_getalarm,
   .set_alarm = bcm476x_rtc_setalarm,
   .irq_set_freq = bcm476x_rtc_setfreq,
   .irq_set_state = bcm476x_rtc_setpie,
   .proc = bcm476x_rtc_proc,
};

static void
bcm476x_rtc_enable( struct platform_device *pdev, int en )
{
   if ( !en )
   {
      rtcHw_stopTimer();
   }
   else
   {
      rtcHw_startTimer();
   }
}

static int __exit
bcm476x_rtc_remove(struct platform_device *dev)
{
	bcm476x_rtc_ctxt_t *ctxt = platform_get_drvdata(dev);

	BUG_ON(!ctxt);

	device_init_wakeup(&dev->dev, 0);

	platform_set_drvdata(dev, NULL);

	bcm476x_rtc_setpie(&dev->dev, 0);
	bcm476x_rtc_setaie(0);

	free_irq(ctxt->one_shot_irq, ctxt->rtc);
	disable_irq_wake(ctxt->one_shot_irq);

	free_irq(ctxt->periodic_irq, ctxt->rtc);

	rtc_device_unregister(ctxt->rtc);
	return 0;
}

static int __init
bcm476x_rtc_probe( struct platform_device *pdev )
{
	int ret;
	struct resource *res;
	bcm476x_rtc_ctxt_t *ctxt = NULL;

	pr_debug( "%s: probe=%p\n", __func__, pdev );

	bcm476x_rtc_enable( pdev, 1 );

	bcm476x_rtc_setfreq( &pdev->dev, 1 );
	device_init_wakeup( &pdev->dev, 1 );


	ctxt = kzalloc(sizeof(bcm476x_rtc_ctxt_t), GFP_KERNEL);
	if (NULL == ctxt) {
		printk(KERN_ERR 
			"bcm476x rtc: cannot allocate context\n");
		return -ENOMEM;		
	}

	ctxt->rtc = rtc_device_register( "bcm476x", &pdev->dev, &bcm476x_rtcops, THIS_MODULE );

	if (IS_ERR(ctxt->rtc))
	{
		ret = PTR_ERR(ctxt->rtc);
		pr_debug( "cannot attach rtc\n" );
		goto err_device_unregister;
	}

	ctxt->rtc->max_user_freq = 128;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, e_irq_one_shot);
	if (!res) {
		ret = -ENODEV;
		printk(KERN_ERR 
			"bcm476x rtc: platform_get_resource "
			"IORESOURCE_IRQ one shot error.\n");
		goto err_device_unregister;
	}

	ret = request_irq(ctxt->one_shot_irq, rtc_alm_isr,
	          #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
               SA_INTERRUPT,
            #else
               IRQF_DISABLED,
            #endif
							"bcm476x-rtc-alm", ctxt->rtc);

	if ( ret < 0 )
	{
		printk(KERN_ERR "IRQ%u error %d\n", ctxt->one_shot_irq, ret );
		goto err_device_unregister;
	} else {
		ctxt->one_shot_irq = res->start;
		enable_irq_wake(ctxt->one_shot_irq);
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, e_irq_periodic);
	if (!res) {
		ret = -ENODEV;
		printk(KERN_ERR 
			"bcm476x rtc: platform_get_resource "
			"IORESOURCE_IRQ periodic error.\n");
		goto err_free_irq1;
	}

	ret = request_irq(res->start, rtc_per_isr,
						#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
               SA_INTERRUPT,
            #else
               IRQF_DISABLED,
            #endif
						"bcm476x-rtc-per", ctxt->rtc);
						
	if ( ret < 0 )
	{
		pr_debug( "IRQ%d error %d\n", res->start, ret);
		goto err_free_irq1;
	}
	
	ctxt->periodic_irq = res->start;

	platform_set_drvdata(pdev, ctxt);
	return 0;

err_free_irq1:
	free_irq(ctxt->one_shot_irq, pdev );
	disable_irq_wake(ctxt->one_shot_irq);

err_device_unregister:
	//bcm476x_rtc_enable (pdev, 0);
	rtc_device_unregister(ctxt->rtc);
	kfree(ctxt);
	return ret;
}

#ifdef CONFIG_PM

/* RTC Power management control */

static int period_cnt;

int
bcm476x_rtc_suspend( struct platform_device *pdev, pm_message_t state )
{
   period_cnt = rtcHw_readReg( rtchw_PERIODIC_TIMER_ADDR );
   bcm476x_rtc_enable( pdev, 0 );
   return 0;
}

static int
bcm476x_rtc_resume( struct platform_device *pdev )
{
//   IF_BCM4760_A0
   {
      writel(0xbcbc4760, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));
      writel(readl(IO_ADDRESS(CMU_R_BLOCK_RESET2_MEMADDR)) & ~CMU_F_CMU_RTC_PRESETN_MASK, IO_ADDRESS(CMU_R_BLOCK_RESET2_MEMADDR));
      udelay(100);
      writel(readl(IO_ADDRESS(CMU_R_BLOCK_RESET2_MEMADDR)) | CMU_F_CMU_RTC_PRESETN_MASK, IO_ADDRESS(CMU_R_BLOCK_RESET2_MEMADDR));
      writel(0, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));
   }
   
   bcm476x_rtc_enable( pdev, 1 );
   rtcHw_writeReg( rtchw_PERIODIC_TIMER_ADDR, period_cnt );

   return 0;
}
#else
#define bcm476x_rtc_suspend NULL
#define bcm476x_rtc_resume  NULL
#endif

static struct platform_driver bcm476x_rtcdrv = {
   .remove = __exit_p( bcm476x_rtc_remove ),
   .suspend = bcm476x_rtc_suspend,
   .resume = bcm476x_rtc_resume,
   .driver = {
              .name = "bcm476x-rtc0",
              .owner = THIS_MODULE,
              },
};

static char __initdata banner[] = "BCM476X RTC, (c) 2009 Broadcom Corporation\n";

static int __init
bcm476x_rtc_init( void )
{
   printk( banner );
   return platform_driver_probe( &bcm476x_rtcdrv, bcm476x_rtc_probe );
}

static void __exit
bcm476x_rtc_exit( void )
{
   platform_driver_unregister( &bcm476x_rtcdrv );
}

module_init( bcm476x_rtc_init );
module_exit( bcm476x_rtc_exit );

MODULE_DESCRIPTION( RTC_BCM4760_MODULE_DESC );
MODULE_AUTHOR( "Broadcom Corporation" );
MODULE_LICENSE( "GPL" );
MODULE_VERSION( RTC_BCM4760_MODULE_VERSION );
MODULE_ALIAS( "platform:bcm476x-rtc" );
