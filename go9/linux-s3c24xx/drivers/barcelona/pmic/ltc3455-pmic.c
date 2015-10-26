/* ltc3455-pmic.c
 *
 * Control driver for LTC3455 PMIC.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/si4710.h>
#include <barcelona/gopins.h>
#include <linux/delay.h>
#include <asm/arch/hardware.h>
#include <asm/hardware/clock.h>
#include <barcelona/usbmode.h>
#include <barcelona/debug.h>
#include <linux/ltc3455-pmic.h>
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
#include <linux/cpufreq.h>
#include <barcelona/cpufreq_order.h>
#endif
#include "ltc3455-pmic.h"

static void ltc3455_state_change_listener( USB_STATE previous_state, USB_STATE current_state, void* arg );
static uint32_t ltc3455_get_match_volt( uint32_t frequency, struct ltc3455_volt *power_setting )
{
        int     count=0;

        for( count=0; (power_setting[count].frequency != 0) &&
                      (power_setting[count].voltage != 0); count++ )
        {
                if( frequency <= power_setting[count].frequency )
                        return power_setting[count].voltage;
        }

        return 0;
}

static void ltc3455_set_voltage( struct ltc3455_pdata *pdata, uint32_t voltage )
{
	if( (voltage > pdata->voltage.low) && (voltage <= pdata->voltage.high ) )
		IOP_Deactivate( pdata->low_core_pin );
	else
		IOP_Activate( pdata->low_core_pin );
	return;
}

static int ltc3455_set_voltage_from_frequency( struct ltc3455_pdata *pdata, uint32_t frequency )
{
	uint32_t	voltage;

	voltage=ltc3455_get_match_volt( frequency, pdata->power_setting );
	if( voltage == 0 ) return -1;

	ltc3455_set_voltage( pdata, voltage );
	return 0;
}

static int ltc3455_set_cmode( struct ltc3455_pdata *pdata, enum ltc3455_cmode mode )
{
	switch( mode )
	{
		case LTC3455_PWM:
			IOP_Deactivate( pdata->pwr_mode_pin );
			break;

		case LTC3455_BURST:
			IOP_Activate( pdata->pwr_mode_pin );
			break;
		default:
			return -1;
	}

	return 0;
}

static void ltc3455_set_high_current( struct ltc3455_pdata *pdata, int enable )
{
	if( enable )
		IOP_Activate( pdata->wall_pwr_pin );
	else
		IOP_Deactivate( pdata->wall_pwr_pin );
	return;
}

static int ltc3455_probe( struct device *dev )
{	
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	struct ltc3455_driver		*driver=(struct ltc3455_driver *) container_of( dev->driver, struct ltc3455_driver, driver );
#endif
	struct ltc3455_pdata		*pdata=(struct ltc3455_pdata *) dev->platform_data;
	struct clk			*fclk;
	uint32_t			curr_cpufreq;
	int				rc;

	/* Signon. */
	printk( "LTC3455 Power Management IC I2C driver v1.0 (C) 2008 TomTom B.V.\n" );

	/* Set the acpwr_pin to input. */
	IOP_SetInput( pdata->acpwr_pin );

	/* Determine the current clock frequency. */
	fclk=clk_get( dev, "fclk" );
	if( fclk == NULL )
	{
		printk( KERN_ERR "LTC3455: Can't get current CPU frequency. Aborting.\n" );
		return -ENODEV;
	}

	curr_cpufreq=clk_get_rate( fclk )/1000;
	clk_put( fclk );

	/* Set the voltage matching the current frequency. */
	if( ltc3455_set_voltage_from_frequency( pdata, curr_cpufreq ) )
	{
		printk( KERN_ERR "LTC3455: Can't get current CPU voltage. Aborting.\n" );
		return -ENODEV;
	}

	/* Set the Switching Regulator Mode. */
	if( IOP_GetInput( pdata->acpwr_pin ) )
	{
		/* Running from AC. So set it to the AC preference. */
		ltc3455_set_cmode( pdata, pdata->cmode.acadapt );
	}
	else
	{
		/* Running from battery. So set it to the Batt preference. */
		ltc3455_set_cmode( pdata, pdata->cmode.batt );
	}

	/* Register the usbmode statechange listener. This will inform us when to change the */
	/* PMIC's settings. */
	if( IO_HaveUsbBusPowered() )
	{
		rc=add_usb_state_change_listener( ltc3455_state_change_listener, pdata );
		if( rc != 0 )
		{
			printk( KERN_ERR "LTC3455: Can't register USBMODE change state listener. Aborting.\n" );
			return -ENODEV;
		}
	}

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	/* For the CPU Frequency scaling code we need access to the ltc3455_pdata structure. */
	driver->pdata=pdata;

	/* Register the CPUFREQ entries. These will set the right voltage when the frequency */
	/* changes. */
	cpufreq_register_notifier( &driver->freq_transition, CPUFREQ_TRANSITION_NOTIFIER );
	cpufreq_register_notifier( &driver->freq_policy, CPUFREQ_POLICY_NOTIFIER );
#endif

	/* Done initializing. */
	return 0;
}

/* This routine exists to change charging mode if the device is powered from USB. */
static void ltc3455_state_change_listener( USB_STATE previous_state, USB_STATE current_state, void* arg )
{
	struct ltc3455_pdata		*pdata=(struct ltc3455_pdata *) arg;

	switch (previous_state)
	{
		case USB_STATE_INITIAL :
		case USB_STATE_IDLE :
		case USB_STATE_DEVICE_DETECT :
		case USB_STATE_DEVICE :
			/* If we are switching from non-powered to powered mode. */
			if( current_state == USB_STATE_HOST_DETECT ||
			    current_state ==  USB_STATE_HOST ||
			    current_state == USB_STATE_CLA )
			{
				ltc3455_set_high_current( pdata, 1 );
				ltc3455_set_cmode( pdata, pdata->cmode.acadapt );
			}
			break;
		case USB_STATE_HOST_DETECT:
		case USB_STATE_HOST:
		case USB_STATE_CLA:
			if( current_state == USB_STATE_INITIAL ||
			    current_state ==  USB_STATE_IDLE ||
			    current_state == USB_STATE_DEVICE_DETECT ||
			    current_state == USB_STATE_DEVICE )
			{
				ltc3455_set_high_current( pdata, 0 );
				ltc3455_set_cmode( pdata, pdata->cmode.batt );
			}
			break;
	}
	return;
}

#ifdef CONFIG_PM
static int ltc3455_suspend(struct device *dev, pm_message_t state, u32 level)
{
	struct ltc3455_pdata		*pdata=(struct ltc3455_pdata *) dev->platform_data;

	if( level == SUSPEND_POWER_DOWN )
	{
		/* Set low charge current charge when we are sleeping. */
		ltc3455_set_high_current( pdata, 0 );

		/* Upon suspend we should make sure the right Switching Regulator Mode is set. */
		if( ltc3455_set_cmode( pdata, pdata->cmode.suspend ) < 0 )
			printk( KERN_WARNING "LTC3455: WARNING! Could not set Switching Regulator Mode. Expect higher current draw!\n" );
	}
	return 0;
}

static int ltc3455_resume(struct device * dev, u32 level)
{
	struct ltc3455_pdata		*pdata=(struct ltc3455_pdata *) dev->platform_data;
	int				rc;

	if( level == RESUME_POWER_ON )
	{
		/* Set low charge current charge when waking up */
		ltc3455_set_high_current( pdata, 0 );

		/* Determine the current state of power, e.g on battery or on AC. */
		if( IOP_GetInput( pdata->acpwr_pin ) )
		{
			/* Running from AC. So set it to the AC preference. */
			rc=ltc3455_set_cmode( pdata, pdata->cmode.acadapt );
		}
		else
		{
			/* Running from battery. So set it to battery preference. */
			rc=ltc3455_set_cmode( pdata, pdata->cmode.batt );
		}

		/* Check exitcode. */
		if( rc < 0 )
			printk( KERN_WARNING "LTC3455: WARNING! Could not set Switching Regulator Mode. Expect higher current draw!\n" );

	}
	return 0;
}
#else
#define ltc3455_suspend		NULL
#define ltc3455_resume		NULL
#endif /* CONFIG_PM	*/

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
static int ltc3455_freq_transition( struct notifier_block *nb, unsigned long val, void *data )
{
	struct ltc3455_driver		*driver=(struct ltc3455_driver *) container_of( nb, struct ltc3455_driver, freq_transition );
	struct ltc3455_pdata		*pdata=driver->pdata;
	struct cpufreq_freqs		*f = data;

	switch( val )
	{
		case CPUFREQ_PRECHANGE :
			/* If we are going from a lower to a higher frequency, */
			/* set the voltage now, before any frequency change. */
			if( f->new > f->old )
			{
				if( ltc3455_set_voltage_from_frequency( pdata, f->new ) )
					printk( KERN_WARNING "LTC3455: Can't set new CPU voltage for frequency %u. Expect instability!\n", f->new );
			}
			break;

		case CPUFREQ_POSTCHANGE :
			/* If we are going from a higher to a lower frequency,  */
			/* set the voltage now, after the frequency has changed.*/
			if( f->new < f->old )
			{
				if( ltc3455_set_voltage_from_frequency( pdata, f->new ) )
					printk( KERN_WARNING "LTC3455: Can't set new CPU voltage for frequency %u. Expect instability!\n", f->new );
			}
			break;

		/* Suspend and resume are pretty much the same, in the sense that the voltage */
		/* is set as the handler is called. */
		case CPUFREQ_RESUMECHANGE :
		case CPUFREQ_SUSPENDCHANGE :
			if( ltc3455_set_voltage_from_frequency( pdata, f->new ) )
				printk( KERN_WARNING "LTC3455: Can't set new CPU voltage for frequency %u. Expect instability!\n", f->new );

			break;

	}
	return 0;
}

static int ltc3455_freq_policy( struct notifier_block *nb, unsigned long val, void *data )
{
	struct cpufreq_policy		*policy=data;
	struct ltc3455_driver		*driver=(struct ltc3455_driver *) container_of( nb, struct ltc3455_driver, freq_policy );
	struct ltc3455_pdata		*pdata=(struct ltc3455_pdata *) driver->pdata;
	unsigned int			min_freq=policy->min;
	unsigned int			max_freq=0;
	int				count;

	/* Check which are the highest/lowest frequencies we accept. */
	/* The lowest frequency we don't need to change, as we can */
	/* always run at the lowest frequency with a higher voltage. */
	for( count=0; (pdata->power_setting[count].frequency != 0) && (pdata->power_setting[count].voltage != 0); count++ )
	{
		if( pdata->power_setting[count].frequency > max_freq )
			max_freq=pdata->power_setting[count].frequency;
	}

	/* Lowest/highest frequencies are now known. */
	switch (val)
	{
		case CPUFREQ_ADJUST :
		case CPUFREQ_INCOMPATIBLE :
			if( policy->min < min_freq )
				policy->min=min_freq;
			if( policy->max > max_freq )
				policy->max=max_freq;
			break;

		case CPUFREQ_NOTIFY:
			/* Can't do much about it. Live with it. */
			break;
	}
	return 0;
}
#endif /* defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ) */

static int ltc3455_remove( struct device *dev )
{
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	struct ltc3455_driver	*driver=(struct ltc3455_driver *) container_of( dev->driver, struct ltc3455_driver, driver );
#endif

	/* Unregister the USBMODE callback if we used it. */
	if( IO_HaveUsbBusPowered() )
	{
		if( remove_usb_state_change_listener( ltc3455_state_change_listener, NULL ) != 0 )
			printk( KERN_ERR "LTC3455: Couldn't unregister USBMODE change state listener!\n" );
	}

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	/* Unregister CPU frequency scaling. */
	cpufreq_unregister_notifier( &driver->freq_transition, CPUFREQ_TRANSITION_NOTIFIER );
	cpufreq_unregister_notifier( &driver->freq_policy, CPUFREQ_POLICY_NOTIFIER );
#endif
	return 0;
}

static struct ltc3455_driver	ltc3455_driver=
{
	.driver=
	{
		.bus			= &platform_bus_type,
		.probe			= ltc3455_probe,
		.name			= LTC3455_DEVNAME,
		.owner			= THIS_MODULE,
		.remove			= ltc3455_remove,
		.shutdown		= NULL,
		.suspend		= ltc3455_suspend,
		.resume			= ltc3455_resume,
	},
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	.freq_transition=
	{
		.notifier_call		= ltc3455_freq_transition,
		.priority		= CPUFREQ_ORDER_S3C24XX_PMIC_PRIO,
	},
	.freq_policy=
	{
		.notifier_call		= ltc3455_freq_policy,
		.priority		= CPUFREQ_ORDER_S3C24XX_PMIC_PRIO,
	},
#endif
};

static int __init ltc3455_init(void)
{
	return driver_register( &ltc3455_driver.driver );
}

static void __exit ltc3455_exit(void)
{
	driver_unregister( &ltc3455_driver.driver );
	return;
}

MODULE_AUTHOR("Rogier Stam <rogier.stam@tomtom.com>");
MODULE_DESCRIPTION("Driver for LTC3455 PM IC.");

module_init( ltc3455_init );
module_exit( ltc3455_exit );

