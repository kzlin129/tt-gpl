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
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/io.h>

#define PFX "watchdog: "
#define OSCR_FREQ           1000    /* Watchdog clock = 1Khz */
#define WATCHDOG_TIMEOUT    60		/* 60 sec default timeout */
#define LOCK_THE_DOG

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=CONFIG_WATCHDOG_NOWAYOUT)");

static unsigned int pre_margin;
static int boot_status;

static DECLARE_MUTEX(open_lock);

typedef enum close_state {
	CLOSE_STATE_NOT,
	CLOSE_STATE_ALLOW=0x4021
} close_state_t;
static close_state_t allow_close;


/*
 * Uncomment this define to be able to test the bcm_adc driver.
 * With this defined, sending an echo of '0' to /dev/watchdog
 * will send a request for all channels on the 4760 ADC.  Sending
 * an echo of '1' to /dev/watchdog will send a request for all
 * channels on the 59040 ADC.
 */
//#define BCM_ADC_TEST
#ifdef BCM_ADC_TEST
#include <linux/broadcom/bcm_adc.h>

bcm_adc_request_t bcm_adc_req00;
bcm_adc_request_t bcm_adc_req01;
bcm_adc_request_t bcm_adc_req02;
bcm_adc_request_t bcm_adc_req03;
bcm_adc_request_t bcm_adc_req10;
bcm_adc_request_t bcm_adc_req11;
bcm_adc_request_t bcm_adc_req12;
bcm_adc_request_t bcm_adc_req13;
bcm_adc_request_t bcm_adc_req14;
bcm_adc_request_t bcm_adc_req15;
bcm_adc_request_t bcm_adc_req16;
bcm_adc_request_t bcm_adc_req17;
bcm_adc_request_t bcm_adc_req18;
bcm_adc_request_t bcm_adc_req19;
#endif

/*****************************************************************************
 ** BCM4760 hardware register access routines
 **/
static void bcm4760_dog_unlock(void)
{
	writel(0x1ACCE551, IO_ADDRESS(WDT_R_WDOGLOCK_MEMADDR));
}
static void bcm4760_dog_lock(void)
{
	writel(0, IO_ADDRESS(WDT_R_WDOGLOCK_MEMADDR));
}

static void bcm4760_dog_disable(void)
{
	writel((WDT_F_RESEN_MASK	|
			WDT_F_PREEN_MASK), IO_ADDRESS(WDT_R_WDOGCONTROL_MEMADDR));
}
static void bcm4760_dog_enable(void)
{
	writel((WDT_F_RESEN_MASK	|
			WDT_F_PREEN_MASK	|
			WDT_F_INTEN_MASK), IO_ADDRESS(WDT_R_WDOGCONTROL_MEMADDR));
}

static void bcm4760_dog_loadcnt(void)
{
	/* load with half the time because we have to make two complete
	 * trips through the ISR before the reset will happen.
	 */
    writel(pre_margin >> 1, IO_ADDRESS(WDT_R_WDOGLOAD_MEMADDR));
}

static void bcm4760_dog_clrint(void)
{
	writel(0, IO_ADDRESS(WDT_R_WDOGINTCLR_MEMADDR));		/* Clear int */
}
/**
 ******************************************************************************/
static void watchdog_start(void)
{
#ifdef LOCK_THE_DOG
	bcm4760_dog_unlock();
	udelay(100);
#endif
	bcm4760_dog_disable();
	udelay(100);
	bcm4760_dog_loadcnt();
	udelay(100);
	bcm4760_dog_clrint();
	udelay(100);
	bcm4760_dog_enable();
#ifdef LOCK_THE_DOG
	udelay(100);
	bcm4760_dog_lock();
#endif
}
static void watchdog_stop(void)
{
#ifdef LOCK_THE_DOG
	bcm4760_dog_unlock();
	udelay(100);
#endif
	bcm4760_dog_disable();
#ifdef LOCK_THE_DOG
	udelay(100);
	bcm4760_dog_lock();
#endif
}
static void watchdog_ping(void)
{
#ifdef LOCK_THE_DOG
	bcm4760_dog_unlock();
	udelay(100);
#endif
	bcm4760_dog_loadcnt();
	udelay(100);
	bcm4760_dog_clrint();
#ifdef LOCK_THE_DOG
	udelay(100);
	bcm4760_dog_lock();
#endif
}

#ifdef ALLOW_ISR_TO_CLEAR_INT_AND_DISALLOW_RESET
/****************************************************************************
*
*  wdog_interrupt
*
***************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t wdog_interrupt( int irq, void *dev_id, struct pt_regs *regs )
#else
static irqreturn_t wdog_interrupt( int irq, void *dev_id )
#endif
{
    /* Clear source interrupt and reloads counter automaticly. */
    bcm4760_dog_clrint();
	/* Clear VIC0 interrupt */
    writel(0, IO_ADDRESS(VIC0_R_VICADDRESS_MEMADDR));
	return( IRQ_HANDLED );
}
#endif

/*
 *	Allow only one person to hold it open
 */
static int watchdog_open(struct inode *inode, struct file *file)
{
    int rc;

	if (down_trylock(&open_lock))
		return -EBUSY;

    if (nowayout)
	{
	    __module_get(THIS_MODULE);
		allow_close = CLOSE_STATE_NOT;
	}
	else
	{
		allow_close = CLOSE_STATE_ALLOW;
	}

#ifdef ALLOW_ISR_TO_CLEAR_INT_AND_DISALLOW_RESET
    /* Setup interrupt handler for 1/2 done watchdog interrupt */
    rc = request_irq( BCM4760_INTR_WDT, wdog_interrupt, 0, "wdog", NULL );
    if( rc )
    {
      printk( KERN_ERR "Failed to get wdog IRQ\n" );
      return( rc );
    } 
    //enable_irq(BCM4760_INTR_WDT);
#endif

    /* Activate timer */
    watchdog_start();

    printk(KERN_INFO "Started watchdog timer.\n");

	return nonseekable_open(inode, file);
}

static int watchdog_release(struct inode *inode, struct file *file)
{
    /* Shut off the timer.
     * Lock it in if it's a module and we defined ...NOWAYOUT */
    if (allow_close == CLOSE_STATE_ALLOW)
	{
	    watchdog_stop();		/* Turn the WDT off */
	}
	else
	{
		watchdog_ping();
	}

#ifdef ALLOW_ISR_TO_CLEAR_INT_AND_DISALLOW_RESET
	disable_irq( BCM4760_INTR_WDT ); 
	free_irq( BCM4760_INTR_WDT, NULL );
#endif

	allow_close = CLOSE_STATE_NOT;
	up(&open_lock);
    return 0;
}

static ssize_t watchdog_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
    /* Refresh the timer. */
    if (len) {
		if (!nowayout) {
			size_t i;

			allow_close = CLOSE_STATE_NOT;

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					allow_close = CLOSE_STATE_ALLOW;
#ifdef BCM_ADC_TEST
				else if (c == '0')
				{
					bcm_adc_request(&bcm_adc_req00);
					bcm_adc_request(&bcm_adc_req01);
					bcm_adc_request(&bcm_adc_req02);
					bcm_adc_request(&bcm_adc_req03);
				}
				else if (c == '1')
				{
					bcm_adc_request(&bcm_adc_req10);
					bcm_adc_request(&bcm_adc_req11);
					bcm_adc_request(&bcm_adc_req12);
					bcm_adc_request(&bcm_adc_req13);
					bcm_adc_request(&bcm_adc_req14);
					bcm_adc_request(&bcm_adc_req15);
					bcm_adc_request(&bcm_adc_req16);
					bcm_adc_request(&bcm_adc_req17);
					bcm_adc_request(&bcm_adc_req18);
					bcm_adc_request(&bcm_adc_req19);
				}
#endif
			}
		}
	    watchdog_ping();
    }
    return len;
}

static struct watchdog_info ident = {
   .options	= WDIOF_CARDRESET | WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING  |
        WDIOF_MAGICCLOSE,
    .firmware_version	= 0,
    .identity	= "Hardware Watchdog",
};

static int watchdog_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int ret = -ENOIOCTLCMD;
	int time, options;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user(argp, &ident,
				   sizeof(ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
		ret = put_user(0, p);
		break;

	case WDIOC_GETBOOTSTATUS:
		ret = put_user(boot_status, p);
		break;

   case WDIOC_SETOPTIONS:
		ret = get_user(options, p);
		if (ret)
			break;

		if (options & WDIOS_DISABLECARD) {
			watchdog_stop();
			ret = 0;
		}

		if (options & WDIOS_ENABLECARD) {
			watchdog_start();
			ret = 0;
		}
      break;

	case WDIOC_SETTIMEOUT:
		ret = get_user(time, p);
		if (ret)
			break;

		if (time <= 0 || time > 60) {
			ret = -EINVAL;
			break;
		}

		pre_margin = OSCR_FREQ * time;
		watchdog_ping();
		/*fall through*/

	case WDIOC_GETTIMEOUT:
		ret = put_user(pre_margin / OSCR_FREQ, p);
		break;

	case WDIOC_KEEPALIVE:
		watchdog_ping();
		ret = 0;
		break;

    default:
      break;

	}
	return ret;
}

/*
 *	Notifier for system down
 */

static int watchdog_notify_sys(struct notifier_block *this, unsigned long code, void *unused)
{
	if (code == SYS_DOWN || code == SYS_HALT)
		watchdog_stop();		/* Turn the WDT off */

	return NOTIFY_DONE;
}

static struct file_operations watchdog_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= watchdog_write,
	.ioctl		= watchdog_ioctl,
	.open		= watchdog_open,
	.release	= watchdog_release,
};

static struct miscdevice watchdog_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &watchdog_fops,
};

static struct notifier_block watchdog_notifier = {
	.notifier_call = watchdog_notify_sys,
};

static char banner[] __initdata =
	KERN_INFO PFX "BROADCOM BCM476X WDT driver\n";


#ifdef BCM_ADC_TEST
int bcm_adc_test_handler(
	bcm_adc_request_t* req,
	unsigned short sample,
	int error)
{
	printk("BCM-ADC_TEST(%d/%d/0x%lx): Sample=0x%x Error=0x%x\n",
		   req->device, req->channel, req->param, sample, error);
}
#endif


static int __init watchdog_init(void)
{
   int ret;

   /* We do not have a way of knowing if the watchdog reset occured or if it was a normal power-up */
	boot_status = 0;
	pre_margin = OSCR_FREQ * WATCHDOG_TIMEOUT;

	ret = register_reboot_notifier(&watchdog_notifier);
	if (ret) {
		printk(KERN_ERR PFX "cannot register reboot notifier (err=%d)\n",
			ret);
      return ret;
	}

	ret = misc_register(&watchdog_miscdev);
	if (ret) {
		printk(KERN_ERR PFX "cannot register miscdev on minor=%d (err=%d)\n",
			WATCHDOG_MINOR, ret);
		unregister_reboot_notifier(&watchdog_notifier);
		return ret;
	}

#ifndef LOCK_THE_DOG
	/*
	 * The dog may be locked up prior to us running...
	 * Make sure to take the leash off
	 */
	bcm4760_dog_unlock();
	udelay(100);
#endif

	bcm4760_dog_disable();

	printk(banner);

#ifdef BCM_ADC_TEST
	bcm_adc_req00.device	= BCM4760_ADC_DEVICE;
	bcm_adc_req00.channel	= 0;
	bcm_adc_req00.param		= 0x4200;
	bcm_adc_req00.callback	= bcm_adc_test_handler;
	bcm_adc_req01.device	= BCM4760_ADC_DEVICE;
	bcm_adc_req01.channel	= 1;
	bcm_adc_req01.param		= 0x4201;
	bcm_adc_req01.callback	= bcm_adc_test_handler;
	bcm_adc_req02.device	= BCM4760_ADC_DEVICE;
	bcm_adc_req02.channel	= 2;
	bcm_adc_req02.param		= 0x4202;
	bcm_adc_req02.callback	= bcm_adc_test_handler;
	bcm_adc_req03.device	= BCM4760_ADC_DEVICE;
	bcm_adc_req03.channel	= 3;
	bcm_adc_req03.param		= 0x4203;
	bcm_adc_req03.callback	= bcm_adc_test_handler;

	bcm_adc_req10.device	= BCM59040_ADC_DEVICE;
	bcm_adc_req10.channel	= 0;
	bcm_adc_req10.param		= 0x4210;
	bcm_adc_req10.callback	= bcm_adc_test_handler;
	bcm_adc_req11.device	= BCM59040_ADC_DEVICE;
	bcm_adc_req11.channel	= 1;
	bcm_adc_req11.param		= 0x4211;
	bcm_adc_req11.callback	= bcm_adc_test_handler;
	bcm_adc_req12.device	= BCM59040_ADC_DEVICE;
	bcm_adc_req12.channel	= 2;
	bcm_adc_req12.param		= 0x4212;
	bcm_adc_req12.callback	= bcm_adc_test_handler;
	bcm_adc_req13.device	= BCM59040_ADC_DEVICE;
	bcm_adc_req13.channel	= 3;
	bcm_adc_req13.param		= 0x4213;
	bcm_adc_req13.callback	= bcm_adc_test_handler;
	bcm_adc_req14.device	= BCM59040_ADC_DEVICE;
	bcm_adc_req14.channel	= 4;
	bcm_adc_req14.param		= 0x4214;
	bcm_adc_req14.callback	= bcm_adc_test_handler;
	bcm_adc_req15.device	= BCM59040_ADC_DEVICE;
	bcm_adc_req15.channel	= 5;
	bcm_adc_req15.param		= 0x4215;
	bcm_adc_req15.callback	= bcm_adc_test_handler;
	bcm_adc_req16.device	= BCM59040_ADC_DEVICE;
	bcm_adc_req16.channel	= 6;
	bcm_adc_req16.param		= 0x4216;
	bcm_adc_req16.callback	= bcm_adc_test_handler;
	bcm_adc_req17.device	= BCM59040_ADC_DEVICE;
	bcm_adc_req17.channel	= 7;
	bcm_adc_req17.param		= 0x4217;
	bcm_adc_req17.callback	= bcm_adc_test_handler;
	bcm_adc_req18.device	= BCM59040_ADC_DEVICE;
	bcm_adc_req18.channel	= 8;
	bcm_adc_req18.param		= 0x4218;
	bcm_adc_req18.callback	= bcm_adc_test_handler;
	bcm_adc_req19.device	= BCM59040_ADC_DEVICE;
	bcm_adc_req19.channel	= 9;
	bcm_adc_req19.param		= 0x4219;
	bcm_adc_req19.callback	= bcm_adc_test_handler;
#endif
	return 0;
}

static void __exit watchdog_exit(void)
{
	misc_deregister(&watchdog_miscdev);
	unregister_reboot_notifier(&watchdog_notifier);
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("BROADCOM");
MODULE_DESCRIPTION("BROADCOM BCM476X Watchdog");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
