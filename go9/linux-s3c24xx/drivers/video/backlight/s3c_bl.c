/*
 *  Backlight Driver for Sharp Corgi
 *
 *  Copyright (c) 2004-2005 Richard Purdie
 *
 *  Based on Sharp's 2.4 Backlight Driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>


#include <linux/config.h>
#include <linux/module.h>
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

#include <../arch/arm/mach-s3c2410/tomtomgo-iopins.h>

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <barcelona/cpufreq_order.h>

static struct notifier_block    freq_transition;
static struct notifier_block    freq_policy;

static int s3c_freq_transition(struct notifier_block *nb, unsigned long val, void *data);
static int s3c_freq_policy(struct notifier_block *nb, unsigned long val, void *data);
#endif

/* Defines */
#define PFX "s3c: "
#define PK_DBG PK_DBG_FUNC
#define SL_DBG PK_DBG_NONE


/* Flags. */
#define FLAG_INFREQ_INVALID             2
#define FLAG_STARTED                    4
static unsigned int s3c_flags         =~(FLAG_INFREQ_INVALID | FLAG_STARTED);
static          int s3c_powermode     = FB_BLANK_UNBLANK;

static unsigned long s3c_tcnt;
static unsigned s3c_level = PWM_BACKLIGHT_MAX;
static unsigned s3c_div = ( IO_GetBacklightTimer() ?  SYS_TIMER0_DIVIDER: SYS_TIMER1_DIVIDER); 
static unsigned s3c_range = 100;
static int s3c_hw_initialised = 0;
static struct device* s3c_device = NULL;
static struct device_driver s3c_driver ;

static int  s3c_hw_init (struct device *dev);
static void s3c_shutdown(struct device *dev);
static int  s3c_set_intensity(struct backlight_device *bd, int intensity);
static int  s3c_get_intensity(struct backlight_device *bd) ;
static int  s3c_hw_setlevel(unsigned level);

/* reset state defined in arch/arm/kernel/setup.c */
extern unsigned int reset_state;

static inline unsigned long divrnd(unsigned long nom, unsigned long den)
{
	return (nom + den / 2) / den;
}


inline void start_timer(unsigned timer)
{
	unsigned long tcon;
	unsigned long flags;
    
        PK_DBG("Starting timer %d\n",timer);
        __raw_writew(((__u16) s3c_tcnt), S3C2410_TCNTB(timer));
        local_irq_save(flags);
        local_fiq_disable( );
        tcon = __raw_readl(S3C2410_TCON);

        switch(timer)
        {
          case 0: 
                tcon &=  ~(0xf << S3C2410_TCON_T0OFFSET ) ;
                tcon |= S3C2410_TCON_T0RELOAD | S3C2410_TCON_T0MANUALUPD;
		if (IO_GetBacklightInverted()) tcon |= S3C2410_TCON_T0INVERT;
          case 1: 
                tcon &=  ~(0xf << S3C2410_TCON_T1OFFSET ) ;
                tcon |= S3C2410_TCON_T1RELOAD | S3C2410_TCON_T1MANUALUPD;
		if (IO_GetBacklightInverted()) tcon |= S3C2410_TCON_T1INVERT;
        }

        __raw_writel(tcon, S3C2410_TCON); /* Stop and manual load */

        switch(timer)
        {
          case 0: 
                tcon &=  ~(0xf << S3C2410_TCON_T0OFFSET ) ;
                tcon |= S3C2410_TCON_T0RELOAD | S3C2410_TCON_T0START;
		if (IO_GetBacklightInverted()) tcon |= S3C2410_TCON_T0INVERT;
                break;
          case 1: 
                tcon &=  ~(0xf << S3C2410_TCON_T1OFFSET ) ;
                tcon |= S3C2410_TCON_T1RELOAD | S3C2410_TCON_T1START;
		if (IO_GetBacklightInverted()) tcon |= S3C2410_TCON_T1INVERT;
                break;

        }

        __raw_writel(tcon, S3C2410_TCON); /* Start */
	local_irq_restore(flags);

}

inline void stop_timer(unsigned timer)
{
  	unsigned long flags;
	unsigned long tcon;

	s3c_hw_setlevel(PWM_BACKLIGHT_MIN);
	s3c_flags &=~(FLAG_STARTED);

	PK_DBG("Stopping timer %d\n",timer);
	local_irq_save(flags);
	local_fiq_disable();
	tcon = __raw_readl(S3C2410_TCON);

        switch(timer)
        {
          case 0:
                tcon &= ~(0xf << S3C2410_TCON_T0OFFSET);
                tcon |= S3C2410_TCON_T0RELOAD | S3C2410_TCON_T0MANUALUPD;
                break;
          case 1:
                tcon &= ~(0xf << S3C2410_TCON_T1OFFSET);
                tcon |= S3C2410_TCON_T1RELOAD | S3C2410_TCON_T1MANUALUPD;
                break;
        }

	__raw_writel(tcon, S3C2410_TCON); /* Stop and manual load */
	local_irq_restore(flags);
}

#define IO_Down(x) do { if (PIN_IS_INVERTED(IO_Pin(x))) IO_Activate(x); else IO_Deactivate(x); } while (0)
#define IO_Up(x)   do { if (PIN_IS_INVERTED(IO_Pin(x))) IO_Deactivate(x); else IO_Activate(x); } while (0)

inline void s3c_hw_setlevel_direct(unsigned level)
{
	unsigned long tcmp;
	unsigned int bl_i;

	/* Map values if mapping is defined */ 
	if (IO_GetBacklightMapping()) {
		bl_i = level / 5;
		/* bl_i = (level - 10) * 20 / 90; */
		level = (int)((unsigned char*)IO_GetBacklightMapping())[bl_i];
		SL_DBG("%s(mapping):  level: %d \n", __FUNCTION__, level); 
	}
        
	if (!s3c_hw_initialised) {
                //IO_Deactivate(BACKLIGHT_PWM);
                //mdelay(1);
                printk("Activate backlight_en\n");
                IO_Activate(BACKLIGHT_EN);
                mdelay(1);
                start_timer(IO_GetBacklightTimer());
		s3c_hw_initialised = 1;
	}

	tcmp = divrnd(level * s3c_tcnt, s3c_range);
	SL_DBG("%s: level: %d, tcmp: %d \n", __FUNCTION__, level, tcmp); 

	if (tcmp == 0) 	{
		if (IO_GetBacklightInverted())	{
			IO_Up(BACKLIGHT_PWM);
		        SL_DBG("level = %u, gpio pin up\n", level);
		} else {
			IO_Down(BACKLIGHT_PWM);
	        	SL_DBG("level = %u, gpio pin down\n", level);
		}
	} else if (tcmp == s3c_tcnt) {
		if (IO_GetBacklightInverted()) {
			IO_Down(BACKLIGHT_PWM);
			SL_DBG("level = %u, gpio pin down\n", level);
		} else {
			IO_Up(BACKLIGHT_PWM);
			SL_DBG("level = %u, gpio pin up\n", level);
		}
	} else 	{
		/* Set PWM to requested level */
		SL_DBG("level = %u%s, tcnt = %u, tcmp = %u\n", level, IO_GetBacklightInverted() ? 
				" (inverted)" : "", s3c_tcnt, tcmp);
		__raw_writew(((__u16) tcmp), S3C2410_TCMPB(IO_GetBacklightTimer())); 
		IO_SetFunction(BACKLIGHT_PWM);
	}
}

static int s3c_hw_setlevel(unsigned level)
{
	static unsigned old_level = PWM_BACKLIGHT_MAX+1; // Backlight is already enabled by bootloader at unknown value, so make sure level will be different

	PK_DBG("level=%u, old_level=%u\n", level, old_level);

	if (!s3c_hw_initialised && s3c_device)
		s3c_hw_init(s3c_device);

	if (old_level == level) return 0;

	//IO_Activate(BACKLIGHT_EN);

	if (level < PWM_BACKLIGHT_MIN || level > PWM_BACKLIGHT_MAX) { 
		PK_WARN("Invalid backlight level %u\n", level);
		return -EINVAL;
	}

	/* Something special need to be done when last level was zero and new level is above 0 */
	if ((old_level == 0) && (level > 0)) {
		if (IO_HaveUsbBusPowered()) {
			/* Increase to requested level in requested level * 1msecs to avoid big current surge */
			for (old_level = 1;old_level < level;old_level++) {
				s3c_hw_setlevel_direct(old_level);
				mdelay(1);
			}
		} else {
			/* Set PWM to 100% for a small amount of time to turn backlight chip on */
			s3c_hw_setlevel_direct(s3c_range);
			mdelay(2);
		}
	}

	s3c_hw_setlevel_direct(level);

	old_level = level;

	return 0;
}

static inline int s3c_setlevel(unsigned level)
{
	int ret;

	ret = s3c_hw_setlevel(level);
	if (ret < 0)
		return ret;

	s3c_level = level;
	return 0;
}

static int s3c_hw_init(struct device *dev)
{
	struct clk *clock;
	unsigned long rate;
	unsigned long tcnt_mod;
	unsigned long s3c_divider;
	unsigned long freq;

	s3c_hw_initialised = 0;

	if (IO_GetBacklightType() == GOBACKLIGHT_CH0_1000_CAT3238TD_430) {
		/* Hack for brigthness range Limerick/Knock/Milan/Rome/Modena screens */
		s3c_range = 160; 
	} else {
		/* for all devices, including cagliari */
		s3c_range = 100;
	}

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
        PK_DBG(" -----------------backlight:freq: %d\n",freq); 
	s3c_divider = ((SYS_TIMER01_PRESCALER + 1) * s3c_div * freq);

	s3c_tcnt = rate / s3c_divider - 1;
	tcnt_mod = rate % s3c_divider;
	PK_DBG("s3c_tcnt = %#lx, tcnt_mod = %#lx\n", s3c_tcnt, tcnt_mod);

	s3c_flags|=FLAG_STARTED;
    
	PK_DBG("Done\n");
	return 0;
}

void s3c_restore_level(void)
{
	s3c_hw_setlevel(s3c_level);
}
EXPORT_SYMBOL(s3c_restore_level);

static void s3c_hw_exit(void)
{
	PK_DBG("Turning off backlight\n");
        IO_Deactivate(BACKLIGHT_PWM); // actually to deactivate it because the pin is inverted
        mdelay(1);
        IO_Deactivate(BACKLIGHT_EN);
        stop_timer(IO_GetBacklightTimer());
	s3c_hw_initialised = 0;
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
/*
 * CPU clock speed change handler. Change PWM freq accordingly, or refuse policy.
 */
static int s3c_freq_transition(struct notifier_block *nb, unsigned long val, void *data)
{
        struct cpufreq_freqs    *f = data;
        unsigned long int       pclk_new;
        unsigned long int       pclk_old;
        unsigned long int       freq=IO_GetBacklightFreq( );
        unsigned long int       min_pclk;
        unsigned long int       max_pclk;

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
                s3c_hw_exit( );

                if( (pclk_new >= min_pclk) && (pclk_new <= max_pclk) )
                {
                        /* Flag that it's valid. */
                        s3c_flags&=~(FLAG_INFREQ_INVALID);

                        /* Determine the right divider. new_divider will likely not be one of the possible dividers */
                        /* (2, 4, 8, 16). */
                        s3c_div=1;
                        do
                        {
                                s3c_div<<=1;
                                s3c_tcnt=pclk_new/((SYS_TIMER01_PRESCALER + 1) * s3c_div * freq);
                        }
                        while( (s3c_div < 16) && (s3c_tcnt > 0xFFFF) );

                        if( (s3c_div > 16) || (s3c_tcnt > 0xFFFF) )
                        {
                                s3c_tcnt=0xFFFF;
                                s3c_div=16;
                                printk( "WARNING! PWM cannot be set to right rate during transition from %u to %u. Unexpected results may occur.\n", f->old, f->new );
                        }
                }
                else
                {
                        /* Flag that it's invalid. */
                        s3c_flags|=FLAG_INFREQ_INVALID;
                }
                break;

        case CPUFREQ_POSTCHANGE:
                /* Reinit the hardware. */
                if( s3c_hw_init( NULL ) != 0 )
                {
                        printk( "Can't reset PWM timer!\n" );
                        return -ENODEV;
                }

                /* Restore the previous level. */
                s3c_restore_level( );

                /* Ensure that if it was disabled, it stays disabled. */
                if( s3c_level == PWM_BACKLIGHT_MIN )
                        IO_Deactivate(BACKLIGHT_PWM);
                break;
        }
        return 0;
}

static int s3c_freq_policy(struct notifier_block *nb, unsigned long val, void *data)
{
        struct cpufreq_policy   *policy = data;
        unsigned long int       freq=IO_GetBacklightFreq( );
        unsigned long int       min_pclk;
        unsigned long int       max_pclk;
        unsigned long int       low_pclk;
        unsigned long int       high_pclk;

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

static void s3c_blank(struct device *dev, int blank)
{
	switch(blank) {
		case FB_BLANK_NORMAL:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_POWERDOWN:
				s3c_shutdown(dev);

				/* XIAMEN: set to maximum brightness on resume from suspend */
				s3c_level = PWM_BACKLIGHT_MAX;

				s3c_powermode = blank;
			break;
		case FB_BLANK_UNBLANK:
				s3c_powermode = blank;
				s3c_hw_init(dev);
				/* Set back the current intensity level */
				s3c_set_intensity(NULL, s3c_get_intensity(NULL)) ;
			break;
	} /* End switch */
}

static int s3cbl_set_power(struct backlight_device *bd, int blank)
{
	int rc;

	switch(blank) {
		case FB_BLANK_NORMAL:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_POWERDOWN:
		{
			if (s3c_powermode == FB_BLANK_UNBLANK) {
				s3c_powermode = blank;
            
			rc=s3c_setlevel(PWM_BACKLIGHT_MIN);
			if( rc == 0 ) 
				s3c_flags&=~FLAG_STARTED;
			}
			break;
		}
		case FB_BLANK_UNBLANK:
		{
			if (s3c_powermode != FB_BLANK_UNBLANK) {
				s3c_powermode = blank;
            
				rc=s3c_setlevel(PWM_BACKLIGHT_MAX);
				if( rc == 0 ) 
					s3c_flags|=FLAG_STARTED;
			}
			break;
		}
	} /* End Switch */

	return 0;
}

static int s3cbl_get_power(struct backlight_device *bd)
{
	return s3c_powermode ;
}

static int s3c_set_intensity(struct backlight_device *bd, int intensity)
{
	int rc ;

	if (intensity > PWM_BACKLIGHT_MAX)
		intensity = PWM_BACKLIGHT_MAX;

	if (s3c_powermode != FB_BLANK_UNBLANK)
		intensity = PWM_BACKLIGHT_MIN;

	rc=s3c_setlevel(intensity);
	if( rc == 0 ) {
		if (intensity != PWM_BACKLIGHT_MIN) 
			s3c_flags|=FLAG_STARTED;
		else 
			s3c_flags&=~FLAG_STARTED;
	}

	return 0;
}

static int s3c_get_intensity(struct backlight_device *bd)
{
	return s3c_level ;
}

static struct backlight_properties s3cbl_data = {
	.owner		= THIS_MODULE,
	.get_power      = s3cbl_get_power,
	.set_power      = s3cbl_set_power,
	.max_brightness = PWM_BACKLIGHT_MAX,
	.get_brightness = s3c_get_intensity,
	.set_brightness = s3c_set_intensity,
};

static struct backlight_device *s3c_backlight_device;

static int s3c_probe(struct device *dev)
{
	int ret;

	ret = s3c_hw_init(dev);
	if (ret < 0) {
		PK_ERR("Failed to initialize hardware (%d)\n", ret);
		return ret;
	}

	/* Set the level to the same as the bootloader uses. */
	s3c_level=(IO_HaveUsbBusPowered( ) ? 30 : 80);
	s3c_hw_setlevel( s3c_level );

	PK_DBG("Registering Backlight\n");
	s3c_backlight_device = backlight_device_register ("s3c", NULL, &s3cbl_data);
	if (IS_ERR (s3c_backlight_device))
		return PTR_ERR (s3c_backlight_device);

	s3c_device = dev;

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	freq_transition.notifier_call = s3c_freq_transition;
	freq_transition.priority = CPUFREQ_ORDER_S3C24XX_PWM_PRIO;
	freq_policy.notifier_call = s3c_freq_policy;
	cpufreq_register_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	PK_DBG("Done\n");
	return 0;
}

static int s3c_remove(struct device *dev)
{
	s3c_hw_exit();

	PK_DBG("Unregistering Backlight\n");
	backlight_device_unregister(s3c_backlight_device);

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	cpufreq_unregister_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
	freq_transition.notifier_call = NULL;
	freq_policy.notifier_call = NULL;
#endif

	s3c_device = NULL;

	PK_DBG("Done\n");
	return 0;
}

static void s3c_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
	s3c_hw_exit();
}

#ifdef CONFIG_PM

static int s3c_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);

	if (level == SUSPEND_POWER_DOWN) {
		s3c_blank(dev, FB_BLANK_POWERDOWN);
	}
	return 0;
}

static int s3c_resume(struct device *dev, u32 level)
{
	PK_DBG("dev = %p, level = %u\n", dev, level);

	if (level == RESUME_POWER_ON && reset_state != 2) {
		s3c_blank(dev, FB_BLANK_UNBLANK);
	}
	return 0;
}

#else /* CONFIG_PM */
#define s3c_suspend NULL
#define s3c_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver s3c_driver = {
	.name		= "backlight_s3c",
	.bus		= &platform_bus_type,
	.probe		= s3c_probe,
	.remove		= s3c_remove,
	.shutdown	= s3c_shutdown,
	.suspend	= s3c_suspend,
	.resume		= s3c_resume,
};

static int __init s3c_bl_mod_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO Backlight S3C, (C) 2007 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&s3c_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit s3c_bl_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&s3c_driver);
	PK_DBG("Done\n");
}


module_init(s3c_bl_mod_init);
module_exit(s3c_bl_mod_exit);

MODULE_AUTHOR("Dimitry Andric <dimitry.andric@tomtom.com> Jeroen Taverne <jeroen.taverne@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO S3C Driver");
MODULE_LICENSE("GPL");

/* EOF */
