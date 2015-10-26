/* drivers/barcelona/pwm/pwm.c
 *
 * Implementation of the pwm driver.
 *
 * Copyright (C) 2004,2005,2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Dimitry Andric <dimitry.andric@tomtom.com>
 *          Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Includes */ 
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/hardware/clock.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-timer.h>
#include <barcelona/timer.h>
#include <barcelona/Barc_pwm.h>
#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include <barcelona/debug.h>
#include <barcelona/Barc_adc.h>
#include <linux/cpufreq.h>

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <barcelona/cpufreq_order.h>

static struct notifier_block	freq_transition;
static struct notifier_block	freq_policy;

static int pwm_freq_transition(struct notifier_block *nb, unsigned long val, void *data);
static int pwm_freq_policy(struct notifier_block *nb, unsigned long val, void *data);
#endif

/* Defines */
#define PFX "pwm: "
#define PK_DBG PK_DBG_FUNC
#define SL_DBG PK_DBG_NONE

/* Flags. */
#define FLAG_OPEN                       1
#define FLAG_INFREQ_INVALID             2
#define FLAG_STARTED                    4
static unsigned int pwm_flags=~(FLAG_OPEN | FLAG_INFREQ_INVALID | FLAG_STARTED);

/* Enable to skip powering up the tps61042 chip. */
#undef PWM_SKIP_POWERUP

static unsigned long pwm_tcnt;
static unsigned pwm_level = PWM_BACKLIGHT_MAX;
static unsigned pwm_div = SYS_TIMER0_DIVIDER;
static unsigned pwm_range = 100;
static int pwm_hw_initialised = 0;
static struct device* pwm_device = NULL;

static unsigned short photosensor_data = 0;
static spinlock_t photosensor_data_lock = SPIN_LOCK_UNLOCKED;

static int pwm_hw_init(struct device *dev);

static void photosensor_adc_poll(short buf[ADC_CHANNELS], void* arg)
{
	unsigned long flags;

	/* Store photosensor data */
	spin_lock_irqsave(&photosensor_data_lock, flags);

	photosensor_data = buf[ADC_LX_OUT];

	spin_unlock_irqrestore(&photosensor_data_lock, flags);
}

static inline unsigned long divrnd(unsigned long nom, unsigned long den)
{
	return (nom + den / 2) / den;
}

inline void pwm_hw_setlevel_direct(unsigned level)
{
	unsigned long tcmp, tcon;
	unsigned long flags;

	if (!pwm_hw_initialised)
	{
		PK_DBG("Starting timer 0\n");
		__raw_writew(((__u16) pwm_tcnt), S3C2410_TCNTB(0));
		local_irq_save(flags);
        	local_fiq_disable( );
		tcon = __raw_readl(S3C2410_TCON);
		tcon &= ~(0xf << 0);
		tcon |= S3C2410_TCON_T0RELOAD;
		tcon |= S3C2410_TCON_T0MANUALUPD;
		__raw_writel(tcon, S3C2410_TCON); /* Stop and manual load */

		tcon &= ~(0xf << 0);
		tcon |= S3C2410_TCON_T0RELOAD;
		tcon |= S3C2410_TCON_T0START;
		__raw_writel(tcon, S3C2410_TCON); /* Start */
		local_irq_restore(flags);
		pwm_hw_initialised = 1;
	}

	tcmp = divrnd(level * pwm_tcnt, pwm_range);

	if (tcmp == 0)
	{
		/* Set PWM to minimum value */
		IO_Deactivate(BACKLIGHT_PWM);
		SL_DBG("level = %u, gpio pin off\n", level);
	}
	else
	if (tcmp == pwm_tcnt)
	{
		/* Set PWM to maximum value */
		IO_Activate(BACKLIGHT_PWM);
		SL_DBG("level = %u, gpio pin on\n", level);
	}
	else 
	{
		/* Set PWM to requested level */
		SL_DBG("level = %u, tcnt = %u, tcmp = %u\n", level, pwm_tcnt, tcmp);
		__raw_writew(((__u16) tcmp), S3C2410_TCMPB(0));
		IO_SetFunction(BACKLIGHT_PWM);
	}
}

static int pwm_hw_setlevel(unsigned level)
{
	static unsigned old_level = PWM_BACKLIGHT_MAX+1; // Backlight is already enabled by bootloader at unknown value, so make sure level will be different

	PK_DBG("level=%u, old_level=%u\n", level, old_level);

	if (!pwm_hw_initialised && pwm_device)
		pwm_hw_init(pwm_device);

	if (old_level == level) return 0;

	IO_Activate(BACKLIGHT_EN);

	if (level < PWM_BACKLIGHT_MIN || level > PWM_BACKLIGHT_MAX) 
	{
		PK_WARN("Invalid backlight level %u\n", level);
		return -EINVAL;
	}

	/* Something special need to be done when last level was zero and new level is above 0 */
	if ((old_level == 0) && (level > 0))
	{
		if (IO_HaveUsbBusPowered())
		{
			/* Increase to requested level in requested level * 1msecs to avoid big current surge */
			for (old_level = 1;old_level < level;old_level++)
			{
				pwm_hw_setlevel_direct(old_level);
				mdelay(1);
			}
		}
		else
		{
			/* Set PWM to 100% for a small amount of time to turn backlight chip on */
			pwm_hw_setlevel_direct(pwm_range);
			mdelay(2);
		}
	}

	pwm_hw_setlevel_direct(level);

	old_level = level;

	return 0;
}

static inline int pwm_setlevel(unsigned level)
{
	int ret;

	ret = pwm_hw_setlevel(level);
	if (ret < 0)
		return ret;

	pwm_level = level;
	return 0;
}

static int pwm_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int	rc;

	switch (cmd) {
	case IOW_BACKLIGHT_OFF:
		rc=pwm_setlevel(PWM_BACKLIGHT_MIN);
		if( rc == 0 ) pwm_flags&=~FLAG_STARTED;
		break;
	case IOW_BACKLIGHT_ON:
		rc=pwm_setlevel(PWM_BACKLIGHT_MAX);
		if( rc == 0 ) pwm_flags|=FLAG_STARTED;
		break;
	case IOW_BACKLIGHT_UPDATE:
		rc=pwm_setlevel(arg);
		if( rc == 0 )
		{
			if( arg != PWM_BACKLIGHT_MIN ) pwm_flags|=FLAG_STARTED;
			else pwm_flags&=~FLAG_STARTED;
		}
		break;
	case IOR_BACKLIGHT_CURRENT:
		return pwm_level;
	case IOR_LIGHTSENSOR_CURRENT:
		return photosensor_data;
	default:
		PK_WARN("Invalid ioctl command %u\n", cmd);
		return -EINVAL;
	}
	return rc; 
}

static int pwm_open(struct inode *inode, struct file *file)
{
	int	rc;

	if( pwm_flags & (FLAG_OPEN | FLAG_INFREQ_INVALID) )
		return -ENODEV;

	rc=nonseekable_open(inode, file);
	if( rc == 0 ) pwm_flags|=FLAG_OPEN;
	return rc;
}

static int pwm_release(struct inode *inode, struct file *file)
{
	if( (pwm_flags & FLAG_OPEN) == 0 )
		return -ENODEV;

	pwm_flags&=~FLAG_OPEN;

	return 0;
}

/* Kernel interface */
static struct file_operations pwm_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= pwm_ioctl,
	.open		= pwm_open,
	.release	= pwm_release,
};

static int pwm_hw_init(struct device *dev)
{
	struct clk *clock;
	unsigned long rate;
	unsigned long tcnt_mod;
	unsigned long pwm_divider;
	unsigned long freq;

	if (IO_GetTftType() == GOTFT_SAMSUNG_LTE430WQ) pwm_range = 160; else pwm_range = 100;

	clock = clk_get(dev, "pclk");
	if (IS_ERR(clock)) {
		PK_ERR("Failed to get PCLK\n");
		return -ENOENT;
	}
	rate = clk_get_rate(clock);
	PK_DBG("PCLK rate: %lu\n", rate);
	clk_put(clock);

	freq = IO_GetBacklightFreq();
	if (freq == 0) freq = 1000;
	pwm_divider = ((SYS_TIMER01_PRESCALER + 1) * pwm_div * freq);

	pwm_tcnt = rate / pwm_divider - 1;
	tcnt_mod = rate % pwm_divider;
	PK_DBG("pwm_tcnt = %#lx, tcnt_mod = %#lx\n", pwm_tcnt, tcnt_mod);

	pwm_flags|=FLAG_STARTED;
	
	PK_DBG("Done\n");
	return 0;
}

void pwm_restore_level(void)
{
	pwm_hw_setlevel(pwm_level);
}
EXPORT_SYMBOL(pwm_restore_level);

void pwm_disable(void)
{
	pwm_hw_setlevel(PWM_BACKLIGHT_MIN);
}
EXPORT_SYMBOL(pwm_disable);
  	 
static void pwm_hw_exit(void)
{
	unsigned long flags;
	unsigned long tcon;

	PK_DBG("Turning off backlight\n");
	pwm_disable( );
	pwm_flags &=~(FLAG_STARTED);

	PK_DBG("Stopping timer 0\n");
	local_irq_save(flags);
        local_fiq_disable( );
	tcon = __raw_readl(S3C2410_TCON);
	tcon &= ~(0xf << 0);
	tcon |= S3C2410_TCON_T0RELOAD;
	tcon |= S3C2410_TCON_T0MANUALUPD;
	__raw_writel(tcon, S3C2410_TCON); /* Stop and manual load */
	local_irq_restore(flags);
	pwm_hw_initialised = 0;
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
/*
 * CPU clock speed change handler. Change PWM freq accordingly, or refuse policy.
 */
static int
pwm_freq_transition(struct notifier_block *nb, unsigned long val,
			 void *data)
{
	struct cpufreq_freqs	*f = data;
	unsigned long int	pclk_new;
	unsigned long int	pclk_old;
	unsigned long int	freq=IO_GetBacklightFreq( );
	unsigned long int	min_pclk;
	unsigned long int	max_pclk;

	/* IO_GetBacklightFreq( ) will return 0 if the default value (1000) is assumed. */
	if( !freq ) freq=1000;

	/* Calculate the minimum and maximum PCLK. */
	min_pclk=(1 * (SYS_TIMER01_PRESCALER + 1) * 2 * freq)/1000;
	max_pclk=(65536 * (SYS_TIMER01_PRESCALER + 1) * 16 * freq)/1000;

	/* Get the old and new values. New one will be set. */
	f->trans2pclk( f, &pclk_old, &pclk_new );

	switch (val) {
	case CPUFREQ_PRECHANGE:
		/* Shutdown the hardware. */
		pwm_hw_exit( );

		if( (pclk_new >= min_pclk) && (pclk_new <= max_pclk) )
		{
			/* Flag that it's valid. */
			pwm_flags&=~(FLAG_INFREQ_INVALID);

			/* Determine the right divider. new_divider will likely not be one of the possible dividers */
			/* (2, 4, 8, 16). */
			pwm_div=1;
			do
			{
				pwm_div<<=1;
				pwm_tcnt=pclk_new/((SYS_TIMER01_PRESCALER + 1) * pwm_div * freq);
			}
			while( (pwm_div < 16) && (pwm_tcnt > 0xFFFF) );

			if( (pwm_div > 16) || (pwm_tcnt > 0xFFFF) )
			{
				pwm_tcnt=0xFFFF;
				pwm_div=16;
				printk( "WARNING! PWM cannot be set to right rate during transition from %lu to %lu. Unexpected results may occur.\n", f->old, f->new );
			}
		}
		else
		{
			/* Flag that it's invalid. */
			pwm_flags|=FLAG_INFREQ_INVALID;
		}
		break;

	case CPUFREQ_POSTCHANGE:
		/* Reinit the hardware. */
		if( pwm_hw_init( NULL ) != 0 )
		{
			printk( "Can't reset PWM timer!\n" );
			return -ENODEV;
		}

		/* Restore the previous level. */
		pwm_restore_level( );

		/* Ensure that if it was disabled, it stays disabled. */
		if( pwm_level == PWM_BACKLIGHT_MIN )
			IO_Deactivate(BACKLIGHT_PWM);
		break;
	}
	return 0;
}

static int
pwm_freq_policy(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	struct cpufreq_policy	*policy = data;
	unsigned long int	freq=IO_GetBacklightFreq( );
	unsigned long int	min_pclk;
	unsigned long int	max_pclk;
	unsigned long int	low_pclk;
	unsigned long int	high_pclk;

	if (freq == 0) freq = 1000;
	policy->policy2pclk( policy, &low_pclk, &high_pclk );

	/* Minimum and maximum are dependend on the value that can be written in the timer register, */
	/* and the divider for the timer configuration register. The prescaler is fixed, as it would */
	/* influence other devices attached to it. */
	min_pclk=(1 * (SYS_TIMER01_PRESCALER + 1) * 2 * freq)/1000;
	max_pclk=(65536 * (SYS_TIMER01_PRESCALER + 1) * 16 * freq)/1000;

	switch (val) {
	case CPUFREQ_ADJUST:
		policy->pclk2policy( policy, min_pclk, max_pclk );
		break;
	case CPUFREQ_INCOMPATIBLE:
		if( (low_pclk >= min_pclk) && (low_pclk <= max_pclk) )
			min_pclk=low_pclk;

		if( (high_pclk >= min_pclk) && (high_pclk <= max_pclk) )
			max_pclk=high_pclk;

		if( (min_pclk != low_pclk) || (max_pclk != high_pclk) )
			policy->pclk2policy( policy, min_pclk, max_pclk );
		break;
	case CPUFREQ_NOTIFY:
		/* Handling of illegal values is in the transition routine. */
		break;
	}
	return 0;
}
#endif

static int pwm_probe(struct device *dev)
{
	int ret;

	if (IO_HaveLightSensor())
	{
		PK_DBG("Registering ADC pollfunc\n");
		ret = adc_register_poll(photosensor_adc_poll, NULL );
		if (ret < 0) {
			PK_ERR("Unable to register ADC pollfunc (%d)\n", ret);
			return ret;
		}
	}

	ret = pwm_hw_init(dev);
	if (ret < 0) {
		PK_ERR("Failed to initialize hardware (%d)\n", ret);
		return ret;
	}

	/* Set the level to the same as the bootloader uses. */
	pwm_level=(IO_HaveUsbBusPowered( ) ? 30 : 80);
	pwm_hw_setlevel( pwm_level );

	/* Register the character device for ioctls. */
	PK_DBG("Registering chardev\n");
	ret = register_chrdev(PWM_MAJOR, PWM_DEVNAME, &pwm_fops);
	if (ret < 0) {
		PK_ERR("Unable to register chardev on major=%d (%d)\n", PWM_MAJOR, ret);
		return ret;
	}

	pwm_device = dev;

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	freq_transition.notifier_call = pwm_freq_transition;
	freq_transition.priority = CPUFREQ_ORDER_S3C24XX_PWM_PRIO;
	freq_policy.notifier_call = pwm_freq_policy;
	cpufreq_register_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	PK_DBG("Done\n");
	return 0;
}

static int pwm_remove(struct device *dev)
{
	pwm_hw_exit();

	if (IO_HaveLightSensor())
	{
		PK_DBG("Unregistering ADC poll\n");
		adc_unregister_poll(photosensor_adc_poll);
	}

	PK_DBG("Unregistering chardev\n");
	unregister_chrdev(PWM_MAJOR, PWM_DEVNAME);

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	cpufreq_unregister_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
	freq_transition.notifier_call = NULL;
	freq_policy.notifier_call = NULL;
#endif

	pwm_device = NULL;

	PK_DBG("Done\n");
	return 0;
}

static void pwm_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
	pwm_hw_exit();
}

#ifdef CONFIG_PM

static int pwm_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);
	if (level == SUSPEND_POWER_DOWN) {
		pwm_shutdown(dev);
	}
	return 0;
}

static int pwm_resume(struct device *dev, u32 level)
{
	PK_DBG("dev = %p, level = %u\n", dev, level);
	if (level == RESUME_POWER_ON) {
		int ret = pwm_hw_init(dev);
		if (ret < 0) {
			PK_ERR("Failed to initialize hardware (%d)\n", ret);
			return ret;
		}
	}
	return 0;
}

#else /* CONFIG_PM */
#define pwm_suspend NULL
#define pwm_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver pwm_driver = {
	.name		= "tomtomgo-pwm",
	.bus		= &platform_bus_type,
	.probe		= pwm_probe,
	.remove		= pwm_remove,
	.shutdown	= pwm_shutdown,
	.suspend	= pwm_suspend,
	.resume		= pwm_resume,
};

static int __init pwm_mod_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO Backlight/Lightsensor Driver, (C) 2004,2005,2006 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&pwm_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit pwm_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&pwm_driver);
	PK_DBG("Done\n");
}

module_init(pwm_mod_init);
module_exit(pwm_mod_exit);

MODULE_AUTHOR("Dimitry Andric <dimitry.andric@tomtom.com> Jeroen Taverne <jeroen.taverne@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO PWM Driver");
MODULE_LICENSE("GPL");

/* EOF */
