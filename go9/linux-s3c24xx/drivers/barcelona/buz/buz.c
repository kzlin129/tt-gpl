/* drivers/barcelona/buz/buz.c
 *
 * Implementation of the buzzer driver.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

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
#include <asm/arch/irqs.h>
#include <barcelona/timer.h>
#include <barcelona/buz.h>
#include <barcelona/debug.h>
#include <barcelona/gopins.h>

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <linux/cpufreq.h>
#include <linux/notifier.h>
#include <barcelona/cpufreq_order.h>

static struct notifier_block	freq_transition;
static struct notifier_block	freq_policy;

static int buz_freq_transition(struct notifier_block *nb, unsigned long val, void *data);
static int buz_freq_policy(struct notifier_block *nb, unsigned long val, void *data);
#endif

/* Defines */
#define PFX "buz: "
#define PK_DBG PK_DBG_FUNC
#define SL_DBG PK_DBG_NONE

/* Flags. */
#define FLAG_OPEN			1
#define FLAG_INFREQ_INVALID		2
#define FLAG_STARTED			4
static unsigned int duty=BUZ_DEFAULT_DUTYCYCLE;
static unsigned int freq=BUZ_FREQUENCY_DEFAULT;
static unsigned int buz_flags=~(FLAG_OPEN | FLAG_INFREQ_INVALID | FLAG_STARTED);

static unsigned long int buz_get_input_divider( void )
{
	unsigned long int	prescale=((__raw_readl( S3C2410_TCFG0 ) & 0x0000FF00) >> 8) + 1;

	return prescale;
}

static unsigned long int buz_get_pclk( struct device *dev )
{
	unsigned long int	rate;
	struct clk		*pclk;

	/* Get the HCLK rate. */
	pclk=clk_get( dev, "pclk" );
	if( IS_ERR( pclk ) )
		return 0;

	/* Determine the frequency of the timer's clock. */
	rate=clk_get_rate( pclk );

	/* Release the clock. */
	clk_put( pclk );
	return rate;
}

static unsigned long int buz_get_clock_rate( unsigned long pclk_rate )
{
	unsigned long int	rate;
	unsigned long int	div=(__raw_readl( S3C2410_TCFG1 ) >> 12) & 0x07;

	/* If the source is set to external TCLK1, exit. Else, save the right div. */
	if( div >= 0x04 )
		return 0;
	else
		div=2 << div;

	rate=pclk_rate/(div * buz_get_input_divider( ) );
	return rate;
}

static int GetTCFGMuxSetting( unsigned long int divider )
{
	int	count=0;

	while( divider > 1 )
	{
		divider>>=1;
		count+=1;
	}

	return count;
}

static int buz_open(struct inode *inode, struct file *file)
{
	int			rc;

	if( (buz_flags & FLAG_OPEN) || (buz_flags & FLAG_INFREQ_INVALID) )
		return -ENODEV; 
	rc=nonseekable_open(inode, file);
	if( rc == 0 ) buz_flags|=FLAG_OPEN;
	return rc;
}

static int buz_release(struct inode *inode, struct file *file)
{
	duty=BUZ_DEFAULT_DUTYCYCLE;
	freq=BUZ_FREQUENCY_DEFAULT;
	buz_flags&=~(FLAG_OPEN);
	return 0;
}

static int buz_prg( signed long int frequency, unsigned int dutycycle )
{
	unsigned long int	regval;
	signed long int		freq;
	unsigned long		flags;
	unsigned long int	pclk_rate=buz_get_pclk( NULL );
	unsigned long int	curr_freq=buz_get_clock_rate( pclk_rate );

	/* Make sure we do nothing if the input clock is unusable. */
	if( buz_flags & FLAG_INFREQ_INVALID )
		return -1; 

	/* Absolute value. */
	freq=(frequency < 0 ? -frequency : frequency);	

	/* Verify the frequency and dutycycle are in the legal ranges. */
	if( (frequency < BUZ_FREQUENCY_MIN) || (frequency > BUZ_FREQUENCY_MAX) || (dutycycle > 100) )
		return -2;
	
	/* No interrupts. */
	local_irq_save(flags);
	local_irq_disable( );

	/* Make sure auto reload is set. */
	regval=__raw_readl( S3C2410_TCON );
	regval|=S3C2410_TCON_T2RELOAD;
	__raw_writel( regval, S3C2410_TCON );

	/* The formula to calculate the register value from the frequency is: */
	/* 1/((0.0601/1000000) * frequency). By approximation this comes down to: */
	/* 16638935/frequency. */
	regval=(curr_freq/frequency) & 0xFFFF;

	/* Program the TCNTB2 and TCMPB2 registers (TCMPB2=TCNT2/2 for 50% duty cycle) */
	__raw_writew( ((__u16) regval), S3C2410_TCNTB(2) );
	__raw_writew( (__u16) ((regval*dutycycle)/100), S3C2410_TCMPB(2) );

	/* Set manual update to allow us to write to the registers. */
	regval=__raw_readl( S3C2410_TCON );
	regval|=S3C2410_TCON_T2MANUALUPD;
	__raw_writel( regval, S3C2410_TCON );

	/* Now set the inverter or clear it. */
	if( frequency < 0 ) regval|=S3C2410_TCON_T2INVERT;
	else regval&=~S3C2410_TCON_T2INVERT;
	__raw_writel( regval, S3C2410_TCON );

	/* Clear the manual update bit. */
	__raw_writel( (regval & ~S3C2410_TCON_T2MANUALUPD), S3C2410_TCON );

	/* Restore interrupts. */
	local_irq_restore(flags);

	/* Done. */
	return 0;
}

static int buz_prg_freq( signed long int frequency )
{
	if( buz_prg( frequency, duty ) == 0 )
	{
		freq=frequency;
		return 0;
	}
	else return -1;
}

static int buz_prg_duty( unsigned int dutycycle )
{
	if( buz_prg( freq, dutycycle ) == 0 )
	{
		duty=dutycycle;
		return 0;
	}
	else return -1;
}

static int buz_stop( void )
{
	unsigned long int	regval;
	unsigned long int	flags;

	/* No interrupts. */
	local_irq_save(flags);
	local_irq_disable( );

	/* Set the output to function (TOUT2). */
	IO_Deactivate( BUZZER_EN );

	/* Clear the timer start bit. */
	regval=__raw_readl( S3C2410_TCON );
	regval&=~S3C2410_TCON_T2START;
	__raw_writel( regval, S3C2410_TCON );

	/* Set the buzzer as stopped. */
	buz_flags&=~(FLAG_STARTED);

	/* Restore interrupts. */
	local_irq_restore(flags);

	/* Done. */
	return 0;
}

static int buz_start( void )
{
	unsigned long int	regval;
	unsigned long int	flags;

	/* No interrupts. */
	local_irq_save(flags);
	local_irq_disable( );

	/* Set the output to function (TOUT2). */
	IO_SetFunction( BUZZER_EN );

	/* Clear the timer start bit. */
	regval=__raw_readl( S3C2410_TCON );
	regval|=S3C2410_TCON_T2START;
	__raw_writel( regval, S3C2410_TCON );

	/* Flag as started. */
	buz_flags|=FLAG_STARTED;

	/* Restore interrupts. */
	local_irq_restore(flags);

	/* Done. */
	return 0;
}

static int buz_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case IOW_BUZZER_OFF:
		/* Check if it's already stopped. */
		if( ((buz_flags & FLAG_STARTED) == 0) || (buz_flags & FLAG_INFREQ_INVALID) )
			return -EINVAL;
		else
			return buz_stop( );
	case IOW_BUZZER_ON:
		/* Check if it's already started. */
		if( buz_flags & (FLAG_STARTED | FLAG_INFREQ_INVALID) )
			return -EINVAL;
		else
			return buz_start( );
	case IOW_BUZ_SETFREQ:
		return buz_prg_freq( ((signed long int) arg) );
	case IOW_BUZ_SETDUTYCYCLE:
		return buz_prg_duty( ((unsigned int) arg ) );
	default:
		PK_WARN("Invalid ioctl command %u\n", cmd);
		return -EINVAL;
	}
}

/* Kernel interface */
static struct file_operations buz_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= buz_ioctl,
	.open		= buz_open,
	.release	= buz_release,
};

static int buz_hw_init(unsigned long int pclk_rate)
{
	unsigned long int	flags;
	unsigned long int	divider;
	unsigned long int	curr_divider;
	int			retval;

	/* No interrupts. */
	local_irq_save(flags);
	local_irq_disable( );

	/* Disable the IRQ. We don't need it. */
	disable_irq( IRQ_TIMER2 );

	/* Get the currently programmed divider. */
	curr_divider=2 << ((__raw_readl( S3C2410_TCFG1 ) >> 12) & 0x07);
	if( (curr_divider > 16) || (curr_divider <= 1) )
	{
		printk( "buz: Can't determine current divider.\n" );
		return -1;
	}

	/* Default the HCLK runs at about 133MHz. With the default dividiers, this means that the counter is counting with a */
	/* frequency of 11.000 MHz. From this follows that the 16 bit counter's lowest frequency is 335.693 Hz, and the */
	/* highest frequency is 5.500 MHz. */
	/* Since the HCLK (and with it PCLK) clock rate can vary, we try to keep the frequency at about 11.000MHz. We do */
	/* this by finding the nearest divider to get this frequency. */
	divider=(curr_divider * buz_get_clock_rate( pclk_rate ) + BUZ_DEFFREQ_NOM - 1)/BUZ_DEFFREQ_NOM;
	retval=GetTCFGMuxSetting( divider*curr_divider );
	if( (retval < 0) || ((curr_divider*divider) > 16) || ((curr_divider*divider) <= 1) )
	{
		printk( "buz: Can't calculate correct divider.\n" );
		return -2;
	}

	/* Set the correct divider. This will be as close as it's going to get. */
	__raw_writel( (__raw_readl( S3C2410_TCFG1 ) & 0xFFFFF0FF) |
		    ((((unsigned long int) retval) << 8) & 0x0F), S3C2410_TCFG1 ); 

	/* Make sure the buzzer is stopped. */
	buz_stop( );

	/* Set the timer to it's default. */
	buz_prg( freq, duty );

	/* No interrupts. */
	local_irq_restore(flags);

	/* Don't start it. Wait for the ioctl to do that. */
	return 0;
}

static void buz_hw_exit(void)
{
	/* Disable the timer and set it to default. */
	buz_stop( );
	buz_prg_freq( BUZ_FREQUENCY_DEFAULT );
	return;
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
/*
 * CPU clock speed change handler. Change buzzer freq, or refuse policy.
 */
static int
buz_freq_transition(struct notifier_block *nb, unsigned long val,
			 void *data)
{
	struct cpufreq_freqs		*f = data;
	unsigned long int		buz_max_freq=(BUZ_FREQUENCY_MAX * buz_get_input_divider( ) * 16)/1000;
	unsigned long int		buz_min_freq=(BUZ_FREQUENCY_MAX * buz_get_input_divider( ) * 2)/1000;
	unsigned long int		pclk_new;
	unsigned long int		pclk_old;

	f->trans2pclk( f, &pclk_old, &pclk_new );

	switch (val) {
	case CPUFREQ_PRECHANGE:
		if( (pclk_new < buz_min_freq) || (pclk_new > buz_max_freq) )
		{
			printk( "buz: Buzzer would be programmed to an illegal value. Buzzer closed.\n" );
			buz_flags|=FLAG_INFREQ_INVALID;
			if( buz_flags & FLAG_STARTED )
			{
				buz_stop( );
			}
		}
		else buz_flags&=~(FLAG_INFREQ_INVALID);

		/* Deactivate the buzzer if there is going to be a change. */
		IO_Deactivate( BUZZER_EN );
		break;

	case CPUFREQ_POSTCHANGE:
		/* Check if the current setting is valid. */
		if( (pclk_new >= buz_min_freq) && (pclk_new <= buz_max_freq) )
		{
			/* Reprogram the timer. */
			buz_hw_init( pclk_new*1000 );

			/* Check if it is running. */
			if( buz_flags & FLAG_STARTED )
				buz_start( );
		}
		break;
	}
	return 0;
}

static int
buz_freq_policy(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	unsigned long int	buz_max_freq=(BUZ_FREQUENCY_MAX * buz_get_input_divider( ) * 16)/1000;
	unsigned long int	buz_min_freq=(BUZ_FREQUENCY_MAX * buz_get_input_divider( ) * 2)/1000;
	unsigned long int	pclk_min;
	unsigned long int	pclk_max;
	struct cpufreq_policy	*policy = data;

	policy->policy2pclk( policy, &pclk_min, &pclk_max );

	switch (val) {
	case CPUFREQ_ADJUST:
		if( buz_flags & FLAG_OPEN )
		{
			/* The divider's minimum value is 2, the maximum 16. This means that the */
			policy->pclk2policy( policy, buz_min_freq, buz_max_freq );
		}
		break;
	case CPUFREQ_INCOMPATIBLE:
		if( buz_flags & FLAG_OPEN )
		{
			if( (pclk_min >= buz_min_freq) && (pclk_min <= buz_max_freq) )
				buz_min_freq=pclk_min;

			if( (pclk_max >= buz_min_freq) && (pclk_max <= buz_max_freq) )
				buz_max_freq=pclk_max;

			if( (buz_min_freq != pclk_min) || (buz_max_freq != pclk_max) )
				policy->pclk2policy( policy, buz_min_freq, buz_max_freq );
		}
		break;
	case CPUFREQ_NOTIFY:
		/* Handling of illegal clocks is done in the transition routine. */
		break;
	}
	return 0;
}
#endif

static int buz_probe(struct device *dev)
{
	int ret=0;
	unsigned long int	pclk_rate=buz_get_pclk( dev );

	ret = buz_hw_init(pclk_rate);
	if (ret < 0) {
		PK_ERR("Failed to initialize hardware (%d)\n", ret);
		return ret;
	}

	PK_DBG("Registering chardev\n");
	ret = register_chrdev(BUZ_MAJOR, BUZ_DEVNAME, &buz_fops);
	if (ret < 0) {
		PK_ERR("Unable to register chardev on major=%d (%d)\n", BUZ_MAJOR, ret);
		return ret;
	}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	freq_transition.notifier_call = buz_freq_transition;
	freq_transition.priority = CPUFREQ_ORDER_S3C24XX_BUZ_PRIO;
	freq_policy.notifier_call = buz_freq_policy;
	cpufreq_register_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	PK_DBG("Done\n");
	return 0;
}

static int buz_remove(struct device *dev)
{
	buz_hw_exit();

	PK_DBG("Unregistering chardev\n");
	unregister_chrdev(BUZ_MAJOR, BUZ_DEVNAME);

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	cpufreq_unregister_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif
	PK_DBG("Done\n");
	return 0;
}

static void buz_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
	buz_hw_exit();
}

#ifdef CONFIG_PM

static int buz_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);
	if (level == SUSPEND_POWER_DOWN) {
		buz_shutdown(dev);
	}
	return 0;
}

static int buz_resume(struct device *dev, u32 level)
{
	int			ret=0;
	unsigned long int	pclk_rate=buz_get_pclk( dev );

	PK_DBG("dev = %p, level = %u\n", dev, level);
	if (level == RESUME_POWER_ON) {
		ret = buz_hw_init(pclk_rate);
		if (ret < 0) {
			PK_ERR("Failed to initialize hardware (%d)\n", ret);
			return ret;
		}
	}
	return 0;
}

#else /* CONFIG_PM */
#define buz_suspend NULL
#define buz_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver buz_driver = {
	.name		= "tomtomgo-buz",
	.bus		= &platform_bus_type,
	.probe		= buz_probe,
	.remove		= buz_remove,
	.shutdown	= buz_shutdown,
	.suspend	= buz_suspend,
	.resume		= buz_resume,
};

static int __init buz_mod_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO Buzzer Driver, (C) 2006 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&buz_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit buz_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&buz_driver);
	PK_DBG("Done\n");
}

module_init(buz_mod_init);
module_exit(buz_mod_exit);

MODULE_AUTHOR("Dimitry Andric <dimitry.andric@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO Buzzer Driver");
MODULE_LICENSE("GPL");

/* EOF */
