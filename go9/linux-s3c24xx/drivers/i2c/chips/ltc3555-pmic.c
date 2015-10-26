/* ltc3555-pmic.c
 *
 * Control driver for LTC3555 PMIC.
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
#include <linux/ltc3555-pmic.h>
#include <barcelona/debug.h>
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
#include <linux/cpufreq.h>
#include <barcelona/cpufreq_order.h>
#endif
#include "ltc3555-pmic.h"

#define RAMP_UP_T_DVS	(130)

struct semaphore ltc3555_register_cache_lock;
/* readback cache for what's in chip itself */
static uint8_t	ltc3555_register_cache[2];
static uint32_t	ltc3555_sw2_volttable[LTC3555_VOLTTBL_SIZE];
static uint32_t	ltc3555_sw3_volttable[LTC3555_VOLTTBL_SIZE];
/* i2c_change -> i2c_commit storage */
static uint8_t	write_ltc3555[2]={0, 0};

/* This routine fills the specified table with the supported voltages, as determined by */
/* the resistor dividers for the feedback pins R1 and R2. */
static void ltc3555_build_volttable( uint32_t *table, uint32_t r1, uint32_t r2 )
{
	uint32_t	count=0;

	/* Calculate the table, using: Vout=Vbx * (R1/R2 + 1) */
	/* Where Vbx is 425mV + 25mV * count, and count is max*/
	/* 16 entries. */
	for( count=0; count < LTC3555_VOLTTBL_SIZE; count++ )
	{
		table[count]=425 + 25 * count;
		table[count]+=(table[count] * r1)/r2;
	}
	return;
}

/* Find the closest matching voltage to the specified voltage, and return */
/* it's index in the table or -1 if it can't be found. */
static int ltc3555_get_closest_voltage_idx( uint32_t voltage, uint32_t *table )
{
	int		count=0;
	uint32_t	low_diff=0xFFFFFFFF;
	int		low_diff_idx=-1;
	uint32_t	diff;
	uint32_t	min_voltage=(table[0] * (100 - LTC3555_VOLT_PRECISION))/100;
	uint32_t	max_voltage=(table[LTC3555_VOLTTBL_SIZE - 1] * (100 + LTC3555_VOLT_PRECISION))/100;

	/* Check if the voltage is legal. */
	if( (voltage  < min_voltage) || (voltage > max_voltage) )
		return -1;

	/* Get nearest match. */
	for( count=0; count < LTC3555_VOLTTBL_SIZE; count++ )
	{
		if( table[count] > voltage )
			diff=table[count] - voltage;
		else
			diff=voltage - table[count];

		if( diff < low_diff )
		{
			low_diff=diff;
			low_diff_idx=count;
		}
	}
	return low_diff_idx;
}

static int ltc3555_i2c_commit( struct i2c_client * client, int lock )
{
	int		rc;
	
	/* Get the lock. This before someone else modifies something. */
	if (lock)
		down( &ltc3555_register_cache_lock );

	/* Send the command over I2C. If it passes, we will save the written value. */
	rc=i2c_master_send( client, write_ltc3555, sizeof( write_ltc3555 ) );
	if( rc < 0 )
	{
		if (lock)
			up( &ltc3555_register_cache_lock );
		return rc;
	}

	/* It passed. Save in the cache. */
	ltc3555_register_cache[0] = write_ltc3555[0];
	ltc3555_register_cache[1] = write_ltc3555[1];

	if (lock)
		up( &ltc3555_register_cache_lock );
	return 0;	
}

static int ltc3555_i2c_change( struct i2c_client * client, uint16_t data, uint16_t mask, int lock )
{
	/* Get the lock. This before we modify anything. */
	if (lock)
		down( &ltc3555_register_cache_lock );

	/* Now its safe to prepare & modify. */
	write_ltc3555[0]=write_ltc3555[0] & ~((uint8_t) (mask & 0x00FF));
	write_ltc3555[1]=write_ltc3555[1] & ~((uint8_t) (mask >> 8));
	write_ltc3555[0]|=(uint8_t) ((data & mask) & 0x00FF);
	write_ltc3555[1]|=(uint8_t) ((data & mask) >> 8);
	
	if (lock)
		up( &ltc3555_register_cache_lock );
	return 0;
}

/* Mask has the bits that should change set to 1, the rest set to 0. Data specifies the value */
/* of these bits to write. */
static int ltc3555_i2c_write( struct i2c_client * client, uint16_t data, uint16_t mask )
{
	int		rc;

	/* do all in using single lock */
	down( &ltc3555_register_cache_lock );
	ltc3555_i2c_change(client, data, mask, 0);
	rc = ltc3555_i2c_commit(client, 0);
	up( &ltc3555_register_cache_lock );

	return rc;
}

static int ltc3555_set_voltage( struct i2c_client *client, uint32_t voltage, uint32_t *table )
{
	int		shift=(table == ltc3555_sw2_volttable ? 4 : 0);
	int		rc;

	/*printk("%lu: setting swt%d voltage %d ", jiffies, (table == ltc3555_sw2_volttable ? 2 : 3), voltage);*/

	/* Get the nearest voltage, and if found put it into the prepared message. */
	rc=ltc3555_get_closest_voltage_idx( voltage, table );
	if( rc < 0 ) return rc;

	/*printk("(really %d)\n", rc);*/

	/* Write to the power chip. */
	rc=ltc3555_i2c_change( client, (uint16_t) (rc << shift), (uint16_t) (0xF << shift), 1);
	if( rc < 0 ) return rc;
	return 0;
}

static int ltc3555_set_swmode( struct i2c_client *client, enum ltc3555_swmode swmode )
{
	return ltc3555_i2c_change( client, ((uint16_t) swmode) << 13, 0x0003 << 13, 1 );
}

static int ltc3555_set_battery_charge( struct i2c_client *client, int enable )
{
	return ltc3555_i2c_change( client, (enable ? 0x0000 : 0x8000), 0x8000, 1 );
}

static uint32_t ltc3555_get_match_volt( uint32_t frequency, struct ltc3555_volt *power_setting )
{
	int	count=0;

	for( count=0; (power_setting[count].frequency != 0) &&
	              (power_setting[count].voltage != 0); count++ )
	{
		if( frequency <= power_setting[count].frequency )
			return power_setting[count].voltage;
	}

	return 0;
}

static int ltc3555_probe( struct device *dev )
{	
	struct ltc3555_platform		*pdata=(struct ltc3555_platform *) dev->platform_data;
	struct ltc3555_i2c_driver	*driver=(struct ltc3555_i2c_driver *) container_of( dev->driver, struct ltc3555_i2c_driver, plat_driver );

	/* Signon. */
	printk( "LTC3555 Power Management IC I2C driver v1.0 (C) 2008 TomTom B.V.\n" );

	/* Set the acpwr_pin to input. */
	IOP_SetInput( pdata->acpwr_pin );

	/* Set the platform data into the I2C struct. Again, not neat, but for now the best */
	/* we can do. */
	driver->pdata=pdata;

	/* Register the I2C driver part. */
	return i2c_add_driver( &driver->i2c_driver );
}

/* This routine exists to change charging mode if the device is powered from USB. */
static void ltc3555_state_change_listener( USB_STATE previous_state, USB_STATE current_state, void* arg )
{
	struct ltc3555_i2c_driver	*driver=(struct ltc3555_i2c_driver *) arg;
	struct ltc3555_platform		*pdata=driver->pdata;

	switch (current_state)
	{
		case USB_STATE_IDLE :
			/* nothing connected, lower charge current to 500 mA max */
			/* Do NOT disable charging, unit might be pin-reset and PMIC keep charger disabled forever */
			ltc3555_set_battery_charge( &driver->client, 1 );
			IOP_Deactivate( pdata->wall_pwr_pin );
			break;
		case USB_STATE_HOST:
		case USB_STATE_CLA:
			/* CLA or wall pwr connected, enable 1A charging. */
			IOP_Activate( pdata->wall_pwr_pin );
			ltc3555_set_battery_charge( &driver->client, 1 );
			break;
		case USB_STATE_DEVICE :
			/* something connected, enable 500mA charging. */
			IOP_Deactivate( pdata->wall_pwr_pin );
			ltc3555_set_battery_charge( &driver->client, 1 );
			break;
		case USB_STATE_INITIAL :
		case USB_STATE_HOST_DETECT:
		case USB_STATE_DEVICE_DETECT :
		default:
			/* make sure 1A charging is disabled */
			IOP_Deactivate( pdata->wall_pwr_pin );
			break;
	}
	/* write changes to chip */
	ltc3555_i2c_commit(&driver->client, 1);
	return;
}

static struct ltc3555_i2c_driver	ltc3555_driver;
static int ltc3555_attach( struct i2c_adapter *adap )
{
	struct ltc3555_i2c_driver	*driver=&ltc3555_driver;	/* YUCK! However no other way to get it. */
	struct ltc3555_platform		*pdata=(struct ltc3555_platform *) driver->pdata;
	int				rc;
	uint16_t			ltc3555_reg;
	uint32_t			curr_cpufreq;
	uint32_t			curr_sw2volt;
	uint32_t			curr_sw3volt;
	struct clk			*fclk;

	/* First build the voltage tables. This we need for later lookup. */
	ltc3555_build_volttable( ltc3555_sw2_volttable, pdata->sw2.R1, pdata->sw2.R2 );
	ltc3555_build_volttable( ltc3555_sw3_volttable, pdata->sw3.R1, pdata->sw2.R2 );

	/* Now initialize the semaphore. */
	init_MUTEX( &ltc3555_register_cache_lock );

	/* Now initialize the initial values for the PMIC. */
	/* Default values are 0. */
	memset( ltc3555_register_cache, 0, sizeof( ltc3555_register_cache ) );

	/* Determine the current clock frequency. */
	fclk=clk_get( &adap->dev, "fclk" );
	if( fclk == NULL )
	{
		printk( KERN_ERR "LTC3555: Can't get current CPU frequency. Aborting.\n" );
		return -ENODEV;
	}

	curr_cpufreq=clk_get_rate( fclk )/1000;
	clk_put( fclk );

	/* Determine the current matching voltages. */
	curr_sw2volt=ltc3555_get_match_volt( curr_cpufreq, pdata->sw2.power_setting );
	curr_sw3volt=ltc3555_get_match_volt( curr_cpufreq, pdata->sw3.power_setting );
	
	/* Since we don't want any invalid values we will now write the whole register */
	/* in one go. */
	rc=ltc3555_get_closest_voltage_idx( curr_sw2volt, ltc3555_sw2_volttable );
	if( rc < 0 )
	{
		printk( KERN_ERR "LTC3555: Can't get current SW2 voltage. Aborting.\n" );
		return -ENODEV;
	}

	/* Set the SW2 voltage. */
	ltc3555_reg=(((uint16_t) rc) & 0x0F) << 4;

	/* Get the SW3 voltage. */
	rc=ltc3555_get_closest_voltage_idx( curr_sw3volt, ltc3555_sw3_volttable );
	if( rc < 0 )
	{
		printk( KERN_ERR "LTC3555: Can't get current SW3 voltage. Aborting.\n" );
		return -ENODEV;
	}

	/* Set the SW3 voltage. */
	ltc3555_reg|=(((uint16_t) rc) & 0x0F) << 0;

	/* Set the Switching Regulator Mode. */
	if( IOP_GetInput( pdata->acpwr_pin ) )
	{
		/* Running from AC. So set it to the AC preference. */
		ltc3555_reg|=(((uint16_t) pdata->swmode.acadapt) & 0x03) << 13;

		/* Ensure the Disable Battery Charger bit is off. We don't want it disabled. */
		ltc3555_reg&=0x7FFF;
	}
	else
	{
		/* Running from battery. So set it to the Batt preference. */
		ltc3555_reg|=(((uint16_t) pdata->swmode.batt) & 0x03) << 13;

		/* Ensure the Disable Battery Charger bit is set. We're not charging. */
		ltc3555_reg|=0x8000;
	}

	/* Now initialize and register the client. */
	i2c_set_clientdata( &driver->client, driver );
	driver->client.adapter=adap;
	rc=i2c_attach_client( &driver->client );
	if( rc )
	{
		printk( KERN_ERR "LTC3555: Error registring i2C driver to device core. Aborting.\n" ); 
		return -ENODEV;
	}
	
	/* We're done determining and registring. Write the whole word and update the I2C chip. */
	rc=ltc3555_i2c_write( &driver->client, ltc3555_reg, 0xFFFF );
	if( rc < 0 )
	{
		printk( KERN_ERR "LTC3555: Error writing initial value to PMIC. Aborting.\n" );
		i2c_detach_client( &driver->client );
		return -ENODEV;
	}

	/* Register the usbmode statechange listener. This will inform us when to change the */
	/* PMIC's settings. */
	if( IO_HaveUsbBusPowered() )
	{
		rc=add_usb_state_change_listener( ltc3555_state_change_listener, driver );
		if( rc != 0 )
		{
			printk( KERN_ERR "LTC3555: Can't register USBMODE change state listener. Aborting.\n" );
			i2c_detach_client( &driver->client );
			return -ENODEV;
		}
	}
	
	atomic_set(&driver->suspended, 0);

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	/* Register the CPUFREQ entries. These will set the right voltage when the frequency */
	/* changes. */
	cpufreq_register_notifier( &driver->freq_transition, CPUFREQ_TRANSITION_NOTIFIER );
	cpufreq_register_notifier( &driver->freq_policy, CPUFREQ_POLICY_NOTIFIER );
#endif

	/* Done initializing. */
	return 0;
}

static int ltc3555_detach( struct i2c_client *c )
{
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	struct ltc3555_i2c_driver	*driver=(struct ltc3555_i2c_driver *) container_of( c, struct ltc3555_i2c_driver, client );
#endif

	/* First unregister the I2C client's members. */
	if( IO_HaveUsbBusPowered() )
	{
		if( remove_usb_state_change_listener( ltc3555_state_change_listener, NULL ) != 0 )
			printk( KERN_ERR "LTC3555: Couldn't unregister USBMODE change state listener!\n" );
	}

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	/* Unregister CPU frequency scaling. */
	cpufreq_unregister_notifier( &driver->freq_transition, CPUFREQ_TRANSITION_NOTIFIER );
	cpufreq_unregister_notifier( &driver->freq_policy, CPUFREQ_POLICY_NOTIFIER );
#endif

	/* Now remove the I2C client. */
	i2c_detach_client(c);
	return 0;
}

#ifdef CONFIG_PM
static int ltc3555_suspend(struct device *dev, pm_message_t state, u32 level)
{
	struct ltc3555_platform		*pdata=(struct ltc3555_platform *) dev->platform_data;
	struct ltc3555_i2c_driver	*driver=(struct ltc3555_i2c_driver *) container_of( dev->driver, struct ltc3555_i2c_driver, plat_driver );
	struct i2c_client		*client=&driver->client;

	if( level == SUSPEND_POWER_DOWN )
	{
		/* First unregister the I2C client's members. */
		if( IO_HaveUsbBusPowered() )
		{
			if( remove_usb_state_change_listener( ltc3555_state_change_listener, NULL ) != 0 )
				printk( KERN_ERR "LTC3555: Couldn't unregister USBMODE change state listener!\n" );
		}
		/* Enable battery charge when we are sleeping. */
		ltc3555_set_battery_charge( client, 1 );

		/* Disable high current mode in suspend. */
		IOP_Deactivate( pdata->wall_pwr_pin );

		/* Upon suspend we should make sure the right Switching Regulator Mode is set. */
		ltc3555_set_swmode( client, pdata->swmode.suspend );

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
		/* Set up voltage for the highest frequency before suspend */		
		ltc3555_set_sw2_volt_from_freq( client, driver->max_freq, pdata );
		ltc3555_set_sw3_volt_from_freq( client, driver->max_freq, pdata );
#endif
		ltc3555_i2c_commit( client, 1 );

		atomic_set(&driver->suspended, 1);
	}
	return 0;
}

static int ltc3555_resume(struct device * dev, u32 level)
{
	struct ltc3555_platform		*pdata=(struct ltc3555_platform *) dev->platform_data;
	struct ltc3555_i2c_driver	*driver=(struct ltc3555_i2c_driver *) container_of( dev->driver, struct ltc3555_i2c_driver, plat_driver );
	struct i2c_client		*client=&driver->client;
	struct clk			*fclk;
	unsigned long int		curr_freq;
	int				rc;

	if( level == RESUME_POWER_ON )
	{
		/* Get the current clock frequency. */
		fclk=clk_get( dev, "fclk" );
		curr_freq=clk_get_rate( fclk )/1000;
		clk_put( fclk );
		
		rc=add_usb_state_change_listener( ltc3555_state_change_listener, driver );
		if( rc != 0 )
		{
			printk( KERN_ERR "LTC3555: Can't register USBMODE change state listener. Aborting.\n" );
			i2c_detach_client( &driver->client );
			return -ENODEV;
		}

		/* Determine the current state of power, e.g on battery or on AC. */
		if( IOP_GetInput( pdata->acpwr_pin ) )
		{
			/* Running from AC. So set it to the AC preference. */
			ltc3555_set_swmode( client, pdata->swmode.acadapt );

			/* Make sure battery is charging. */
			ltc3555_set_battery_charge( &driver->client, 1 );
		}
		else
		{
			/* Running from battery. So set it to battery preference. */
			ltc3555_set_swmode( client, pdata->swmode.batt );

			/* We do not need to disable battery charging, does not save power */
			ltc3555_set_battery_charge( &driver->client, 1 );
		}

		/* Set the right voltages for the right frequencies. */
		/* This should actually be done from CPUFREQ_RESUMECHANGE, but as we cannot do any I2C before the */
		/* I2C driver is resumed, we do this now since then we can be sure I2C is done. */
		ltc3555_set_sw2_volt_from_freq( client, curr_freq, pdata );
		ltc3555_set_sw3_volt_from_freq( client, curr_freq, pdata );

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
		/* Set up voltage for the last frequency notified */		
		ltc3555_set_sw2_volt_from_freq( client, driver->last_freq, pdata );
		ltc3555_set_sw3_volt_from_freq( client, driver->last_freq, pdata );
		printk("%s: last_freq: %d is set.\n", __FUNCTION__, driver->last_freq);
#endif
		/* write changed values to the chip (single, slow, i2c sequence) */
		ltc3555_i2c_commit( client, 1 );

		atomic_set(&driver->suspended, 0);
	}
	return 0;
}
#else
#define ltc3555_suspend		NULL
#define ltc3555_resume		NULL
#endif /* CONFIG_PM	*/

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
static int ltc3555_freq_transition( struct notifier_block *nb, unsigned long val, void *data )
{
	struct ltc3555_i2c_driver	*driver=(struct ltc3555_i2c_driver *) container_of( nb, struct ltc3555_i2c_driver, freq_transition );
	struct ltc3555_platform		*pdata=(struct ltc3555_platform *) driver->pdata;
	struct i2c_client		*client=&driver->client;
	struct cpufreq_freqs		*f = data;

	/* Always remember last frequency change */
	driver->last_freq = f->new;

	/* If driver is suspended we only store the desired frequency, 
	   after resuming we set it to the last value */
	if (atomic_read(&driver->suspended)) return 0;

	switch( val )
	{
		case CPUFREQ_PRECHANGE :
			/* If we are going from a lower to a higher frequency, */
			/* set the voltage now, before any frequency change. */
			if( f->new > f->old )
			{
				ltc3555_set_sw2_volt_from_freq( client, f->new, pdata );
				ltc3555_set_sw3_volt_from_freq( client, f->new, pdata );
				ltc3555_i2c_commit( client , 1);
				/* Wait for the CPU to stabilize at the new, higher voltage... */
				udelay(RAMP_UP_T_DVS);
				
			}
			break;

		case CPUFREQ_POSTCHANGE :
			/* If we are going from a higher to a lower frequency,  */
			/* set the voltage now, after the frequency has changed.*/
			if( f->new < f->old )
			{
				ltc3555_set_sw2_volt_from_freq( client, f->new, pdata );
				ltc3555_set_sw3_volt_from_freq( client, f->new, pdata );
				ltc3555_i2c_commit( client, 1 );
			}
			break;

		/* Suspend and resume are pretty much the same, in the sense that the voltage */
		/* is set as the handler is called. However, since during resume the I2C is still */
		/* not available at this time, we do the resume part in the resume handler for this*/
		/* driver, not in this code. */
		case CPUFREQ_SUSPENDCHANGE :
			ltc3555_set_swmode( client, LTC3555_BURST );
			ltc3555_set_sw2_volt_from_freq( client, f->new, pdata );
			ltc3555_set_sw3_volt_from_freq( client, f->new, pdata );
			ltc3555_i2c_commit(client, 1);
			break;

	}
	return 0;
}

static int ltc3555_freq_policy( struct notifier_block *nb, unsigned long val, void *data )
{
	struct cpufreq_policy		*policy=data;
	struct ltc3555_i2c_driver       *driver=(struct ltc3555_i2c_driver *) container_of( nb, struct ltc3555_i2c_driver, freq_policy );
	struct ltc3555_platform         *pdata=(struct ltc3555_platform *) driver->pdata;
	unsigned int			min_freq=0xFFFFFFFF;
	unsigned int			max_freq=0;
	int				count;

	/* Check which are the highest/lowest frequencies we accept. */
	for( count=0; (pdata->sw2.power_setting[count].frequency != 0) && (pdata->sw2.power_setting[count].voltage != 0); count++ )
	{
		if( pdata->sw2.power_setting[count].frequency > max_freq )
			max_freq=pdata->sw2.power_setting[count].frequency;
		if( pdata->sw2.power_setting[count].frequency < min_freq )
			min_freq=pdata->sw2.power_setting[count].frequency;
	}
	for( count=0; (pdata->sw3.power_setting[count].frequency != 0) && (pdata->sw3.power_setting[count].voltage != 0); count++ )
	{
		if( pdata->sw3.power_setting[count].frequency > max_freq )
			max_freq=pdata->sw3.power_setting[count].frequency;
		if( pdata->sw3.power_setting[count].frequency < min_freq )
			min_freq=pdata->sw3.power_setting[count].frequency;
	}

	/* Store values */
	driver->max_freq = max_freq;
	driver->min_freq = min_freq;

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

/* Not exactly nice, but since the i2c_add_driver routine already the ltc3555_i2c_driver.driver */
/* structure, and since in there its put to the I2C bus type, it wouldn't be nice to register */
/* the same structure with a platform bus type. To prevent this, register a separate structure*/
/* to actually get the platform data over here. Not exactly neat, but no choice. */
static struct ltc3555_i2c_driver	ltc3555_driver=
{
        .pdata				= NULL,
        .i2c_driver=
        {
                .id                     = I2C_DRIVERID_LTC355_PMIC,
                .owner                  = THIS_MODULE,
                .name                   = LTC3555_DEVNAME "_i2c",
                .flags                  = I2C_DF_NOTIFY,
                .attach_adapter         = ltc3555_attach,
                .detach_client          = ltc3555_detach,
        },
        .plat_driver=
        {
                .bus                    = &platform_bus_type,
                .probe                  = ltc3555_probe,
                .name                   = LTC3555_DEVNAME,
                .owner                  = THIS_MODULE,
		/* Even though this is not entirely neat, we put suspend and resume */
		/* here. This is because with platform_device_register we can be    */
		/* sure that the platform_device registered is called in lifo order */
		/* upon suspend. With I2C we cannot ensure this. We want this driver*/
		/* to be suspended last, but resumed first. This since we will use  */
		/* suspend/resume to set the Switching Regulator Mode, which will   */
		/* cause the regulator itself to become more efficient in suspend.  */
		/* The downside of this is that it can't draw too much current when */
		/* it is just resuming. Summarizing, this platform_device_register  */
		/* should be called first of ALL devices. */
		.suspend		= ltc3555_suspend,
		.resume			= ltc3555_resume,
        },
	.client=
	{
		.name			= LTC3555_DEVNAME "_i2c_client",
		.flags			= I2C_CLIENT_ALLOW_USE,
		.usage_count		= 0,
		.driver			= &ltc3555_driver.i2c_driver,
		.addr			= LTC3555_I2C_SLAVE_ADDR,
	},
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	.freq_transition=
	{
		.notifier_call		= ltc3555_freq_transition,
		.priority		= CPUFREQ_ORDER_S3C24XX_PMIC_PRIO,
	},
	.freq_policy=
	{
		.notifier_call		= ltc3555_freq_policy,
		.priority		= CPUFREQ_ORDER_S3C24XX_PMIC_PRIO,
	},
#endif
};

static int __init ltc3555_init(void)
{
	return driver_register( &ltc3555_driver.plat_driver );
}

static void __exit ltc3555_exit(void)
{
	driver_unregister( &ltc3555_driver.plat_driver );
	return;
}

MODULE_AUTHOR("Rogier Stam <rogier.stam@tomtom.com>");
MODULE_DESCRIPTION("Driver for I2C connected LTC3555 PM IC.");

module_init( ltc3555_init );
module_exit( ltc3555_exit );

