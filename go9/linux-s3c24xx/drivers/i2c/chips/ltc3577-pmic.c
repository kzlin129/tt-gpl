/* ltc3577-pmic.c
 *
 * Control driver for LTC3577 PMIC.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Rogier Stam <rogier.stam@tomtom.com>
 *          Andrzej Zukowski <andrzej.zukowski@tomtom.com>
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
#include <linux/ltc3577-pmic.h>
#include <barcelona/debug.h>
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
#include <linux/cpufreq.h>
#include <barcelona/cpufreq_order.h>
#endif
#include "ltc3577-pmic.h"

#define RAMP_UP_T_DVS	(130)

struct semaphore ltc3577_register_cache_lock;

#define TRUE				(1)
#define FALSE				(0)

// Register Sub Address
#define LTC3577_ADDR_BUCK_CTRL		(0x0)
#define LTC3577_ADDR_LED_CTRL		(0x1)
#define LTC3577_ADDR_LED_DAC		(0x2)
#define LTC3577_ADDR_LED_PWM		(0x3)
#define LTC3577_ADDR_MAX		(0x4)

/* Masks */
#define LTC3577_BUCK_CTRL_MASK_BK1BRST	(0x1 << 2)
#define LTC3577_BUCK_CTRL_MASK_BK2BRST	(0x1 << 3)
#define LTC3577_BUCK_CTRL_MASK_BK3BRST	(0x1 << 4)
#define LTC3577_BUCK_CTRL_MASK_SLEWCTL	(0x3 << 5)

#define LTC3577_LED_CTRL_MASK_EN	(0x1 << 0)
#define LTC3577_LED_CTRL_MASK_GR	(0x3 << 1)
#define LTC3577_LED_CTRL_MASK_MD	(0x3 << 3)
#define LTC3577_LED_CTRL_MASK_PWMC	(0x3 << 5)
#define LTC3577_LED_CTRL_MASK_SLEWLED	(0x1 << 7)

#define LTC3577_LED_DAC_MASK_DAC	(0x3f << 0)

#define LTC3577_LED_PWM_MASK_PWMPER	(0xf << 0)
#define LTC3577_LED_PWM_MASK_PWMDC	(0xf << 4)

#define LTC3577_READ_REG_MASK_CHARGE	(0x1 << 0)
#define LTC3577_READ_REG_MASK_STAT	(0x3 << 1)
#define LTC3577_READ_REG_MASK_PGLDO_0	(0x1 << 3)
#define LTC3577_READ_REG_MASK_PGLDO_1	(0x1 << 4)
#define LTC3577_READ_REG_MASK_PGBCK1	(0x1 << 5)
#define LTC3577_READ_REG_MASK_PGBCK2	(0x1 << 6)
#define LTC3577_READ_REG_MASK_PGBCK3	(0x1 << 7)

/* Shifts */
#define LTC3577_BUCK_CTRL_SHIFT_BK1BRST	(2)
#define LTC3577_BUCK_CTRL_SHIFT_BK2BRST	(3)
#define LTC3577_BUCK_CTRL_SHIFT_BK3BRST	(4)
#define LTC3577_BUCK_CTRL_SHIFT_SLEWCTL	(5)

#define LTC3577_LED_CTRL_SHIFT_EN	(0)
#define LTC3577_LED_CTRL_SHIFT_GR	(1)
#define LTC3577_LED_CTRL_SHIFT_MD	(3)
#define LTC3577_LED_CTRL_SHIFT_PWMC	(5)
#define LTC3577_LED_CTRL_SHIFT_SLEWLED	(7)

#define LTC3577_LED_DAC_SHIFT_DAC	(0)

#define LTC3577_LED_PWM_SHIFT_PWMPER	(0)
#define LTC3577_LED_PWM_SHIFT_PWMDC	(4)

#define LTC3577_READ_REG_SHIFT_CHARGE	(0)
#define LTC3577_READ_REG_SHIFT_STAT	(1)
#define LTC3577_READ_REG_SHIFT_PGLDO_0	(3)
#define LTC3577_READ_REG_SHIFT_PGLDO_1	(4)
#define LTC3577_READ_REG_SHIFT_PGBCK1	(5)
#define LTC3577_READ_REG_SHIFT_PGBCK2	(6)
#define LTC3577_READ_REG_SHIFT_PGBCK3	(7)

static ltc3577_msg_t ltc3577_write[LTC3577_ADDR_MAX];

static struct ltc3577_i2c_driver ltc3577_driver;

static int ltc3577_i2c_commit( struct i2c_client *client, 
			       ltc3577_msg_t     *write_buf, 
			       ltc3577_msg_t     *read_buf, 
			       uint8_t lock )
{
	struct  ltc3577_i2c_driver *driver = (struct ltc3577_i2c_driver *) container_of(client, struct ltc3577_i2c_driver, client);
	struct  i2c_msg msgs[LTC3577_ADDR_MAX];
	uint8_t count = 0;
	uint8_t i;
	int     rc;

	/* Get the lock. This before someone else modifies something. */
	if (lock)
		down( &ltc3577_register_cache_lock );

	if (atomic_read(&driver->suspended))  {
		printk( KERN_ERR "LTC3577: Cannot commit transaction. LTC3577 driver suspended or not initialized\n" );
		if (lock)
			up( &ltc3577_register_cache_lock );
		return -1;
	}

	/* Prepare one transfer */
	for (i = 0; i < LTC3577_ADDR_MAX; i++) {
		if (write_buf[i].commit) {

			msgs[count].addr    = client->addr;
			msgs[count].flags   = client->flags & I2C_M_TEN;
			msgs[count].len     = LTC3577_WRITE_SIZE;
			msgs[count].buf     = write_buf[i].data;
			write_buf[i].commit = FALSE;
			count++;
		}
	}

	/* Send all messages at once */
	rc = i2c_transfer(client->adapter, msgs, count);
	if( rc < 0 ) {
		if (lock)
			up( &ltc3577_register_cache_lock );
		return rc;
	}

	/* Check whether we need to read sth */
	if (NULL == read_buf) {
		if (lock)
			up( &ltc3577_register_cache_lock );
		return 0;
	}

	/* Read LTC3577 status register */
	if (read_buf->commit) {
		uint8_t data;

		read_buf->commit = FALSE;
		rc = i2c_master_recv( client, &data, sizeof(data));
		if( rc < 0 ) {
			if (lock)
				up( &ltc3577_register_cache_lock );
			return rc;
		}
		read_buf->data[LTC3577_DATA] = data;
	}

	if (lock)
		up( &ltc3577_register_cache_lock );
	return 0;	
}

static int ltc3577_i2c_change(ltc3577_msg_t *write_buf,
			      uint8_t       addr, 
			      uint8_t       data, 
			      uint8_t       shift, 
			      uint8_t       mask) 
{
	if (addr >= LTC3577_ADDR_MAX)
		return -1;

	write_buf[addr].commit  = TRUE;
	write_buf[addr].data[LTC3577_SUBADDR] = addr; 
	write_buf[addr].data[LTC3577_DATA]    &= (~mask); 
	write_buf[addr].data[LTC3577_DATA]    |= (data << shift); 
	return 0;
}

static int ltc3577_set_swmode( ltc3577_msg_t *write_buf, enum ltc3577_swmode swmode )
{
	int rc;
	rc = ltc3577_i2c_change(write_buf,
				LTC3577_ADDR_BUCK_CTRL, 
				swmode,
				LTC3577_BUCK_CTRL_SHIFT_BK1BRST,
				LTC3577_BUCK_CTRL_MASK_BK1BRST);
	if (rc) return rc;

	rc = ltc3577_i2c_change(write_buf,
				LTC3577_ADDR_BUCK_CTRL, 
				swmode,
				LTC3577_BUCK_CTRL_SHIFT_BK2BRST,
				LTC3577_BUCK_CTRL_MASK_BK2BRST);
	if (rc) return rc;

	rc = ltc3577_i2c_change(write_buf,
				LTC3577_ADDR_BUCK_CTRL, 
				swmode,
				LTC3577_BUCK_CTRL_SHIFT_BK3BRST,
				LTC3577_BUCK_CTRL_MASK_BK3BRST);
	return rc;
}

int ltc3577_set_reg( ltc3577_msg_t *write_buf, uint8_t addr, uint8_t data)
{
	if (addr >= LTC3577_ADDR_MAX)
		return -1;

	write_buf[addr].commit                = TRUE;
	write_buf[addr].data[LTC3577_SUBADDR] = addr; 
	write_buf[addr].data[LTC3577_DATA]    = data; 
	return 0;
}
EXPORT_SYMBOL(ltc3577_set_reg);

int ltc3577_commit( ltc3577_msg_t *write_buf, ltc3577_msg_t *read_buf, uint8_t lock)
{
	return ltc3577_i2c_commit( &ltc3577_driver.client, 
				   write_buf, 
				   read_buf, 
				   lock);
}
EXPORT_SYMBOL(ltc3577_commit);

static int ltc3577_set_leds( ltc3577_msg_t *write_buf, enum ltc3577_ledmode onoff )
{
	return ltc3577_i2c_change(write_buf,
				  LTC3577_ADDR_LED_CTRL, 
				  onoff,
				  LTC3577_LED_CTRL_SHIFT_EN,
				  LTC3577_LED_CTRL_MASK_EN);
}

static int ltc3577_probe( struct device *dev )
{	
	struct ltc3577_platform		*pdata=(struct ltc3577_platform *) dev->platform_data;
	struct ltc3577_i2c_driver	*driver=(struct ltc3577_i2c_driver *) container_of( dev->driver, struct ltc3577_i2c_driver, plat_driver );

	/* Signon. */
	printk( "LTC3577 Power Management IC I2C driver v1.0 (C) 2008 TomTom B.V.\n" );

	/* Set the acpwr_pin to input. (JTA: why is this done here? Signal isn't even used in this driver) */
	IOP_SetInput( pdata->acpwr_pin );

	/* Set the platform data into the I2C struct. Again, not neat, but for now the best */
	/* we can do. */
	driver->pdata=pdata;

	/* Register the I2C driver part. */
	return i2c_add_driver( &driver->i2c_driver );
}

/* This routine exists to change charging mode if the device is powered from USB. */
static void ltc3577_state_change_listener( USB_STATE previous_state, USB_STATE current_state, void* arg )
{
	struct ltc3577_i2c_driver	*driver=(struct ltc3577_i2c_driver *) arg;
	struct ltc3577_platform		*pdata=driver->pdata;

	switch (current_state)
	{
		case USB_STATE_IDLE :
 			/* nothing connected, lower charge current to 500 mA max */
 			/* Do NOT disable charging, unit might be pin-reset and PMIC keep charger disabled forever */
			IOP_Deactivate( pdata->wall_pwr_pin );
			break;
		case USB_STATE_HOST:
		case USB_STATE_CLA:
			/* CLA or wall pwr connected, enable 1A charging. */
			IOP_Activate( pdata->wall_pwr_pin );
			break;
		case USB_STATE_DEVICE :
			/* something connected, enable 500mA charging. */
			IOP_Deactivate( pdata->wall_pwr_pin );
			break;
		case USB_STATE_INITIAL :
			if (!IO_HaveFastChargingDuringSuspend()) {
				/* make sure 1A charging is disabled */
				IOP_Deactivate( pdata->wall_pwr_pin );
			}
			break;
		case USB_STATE_HOST_DETECT:
		case USB_STATE_DEVICE_DETECT :
		default:
			/* make sure 1A charging is disabled */
			IOP_Deactivate( pdata->wall_pwr_pin );
			break;
	}
	return;
}

static int ltc3577_attach( struct i2c_adapter *adap )
{
	struct ltc3577_i2c_driver	*driver = &ltc3577_driver;	/* YUCK! However no other way to get it. */
	int				rc;

	/* Now initialize the semaphore. */
	init_MUTEX( &ltc3577_register_cache_lock );

	/* Now initialize and register the client. */
	i2c_set_clientdata( &driver->client, driver );
	driver->client.adapter=adap;
	rc=i2c_attach_client( &driver->client );
	if( rc )
	{
		printk( KERN_ERR "LTC3577: Error registring i2C driver to device core. Aborting.\n" ); 
		return -ENODEV;
	}

	down( &ltc3577_register_cache_lock );
	
	/* Set PWM mode */
	ltc3577_set_swmode( ltc3577_write, LTC3577_PWM );

	if( rc < 0 )
	{
		printk( KERN_ERR "LTC3577: Error writing initial value to PMIC. Aborting.\n" );
		i2c_detach_client( &driver->client );
		up( &ltc3577_register_cache_lock );
		return -ENODEV;
	}

	/* Register the usbmode statechange listener. This will inform us when to change the */
	/* PMIC's settings. */
	if( IO_HaveUsbBusPowered() )
	{
		rc=add_usb_state_change_listener( ltc3577_state_change_listener, driver );
		if( rc != 0 )
		{
			printk( KERN_ERR "LTC3577: Can't register USBMODE change state listener. Aborting.\n" );
			i2c_detach_client( &driver->client );
			up( &ltc3577_register_cache_lock );
			return -ENODEV;
		}
	}
	
	atomic_set(&driver->suspended, 0);
	ltc3577_i2c_commit( &driver->client, ltc3577_write, NULL, 0);

	up( &ltc3577_register_cache_lock );

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	/* Register the CPUFREQ entries. These will set the right voltage when the frequency */
	/* changes. */
	cpufreq_register_notifier( &driver->freq_transition, CPUFREQ_TRANSITION_NOTIFIER );
	cpufreq_register_notifier( &driver->freq_policy, CPUFREQ_POLICY_NOTIFIER );
#endif

	/* Done initializing. */
	return 0;
}

static int ltc3577_detach( struct i2c_client *c )
{
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	struct ltc3577_i2c_driver *driver=(struct ltc3577_i2c_driver *) container_of( c, struct ltc3577_i2c_driver, client );
#endif

	/* First unregister the I2C client's members. */
	if( IO_HaveUsbBusPowered() )
	{
		if( remove_usb_state_change_listener( ltc3577_state_change_listener, NULL ) != 0 )
			printk( KERN_ERR "LTC3577: Couldn't unregister USBMODE change state listener!\n" );
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
static int ltc3577_suspend(struct device *dev, pm_message_t state, u32 level)
{
	struct ltc3577_platform		*pdata=(struct ltc3577_platform *) dev->platform_data;
	struct ltc3577_i2c_driver	*driver=(struct ltc3577_i2c_driver *) container_of( dev->driver, struct ltc3577_i2c_driver, plat_driver );
	struct i2c_client		*client=&driver->client;

	if( level == SUSPEND_POWER_DOWN )
	{
		/* First unregister the I2C client's members. */
		if( IO_HaveUsbBusPowered() )
		{
			if( remove_usb_state_change_listener( ltc3577_state_change_listener, NULL ) != 0 )
				printk( KERN_ERR "LTC3577: Couldn't unregister USBMODE change state listener!\n" );
		}

		down( &ltc3577_register_cache_lock );

		if (!IO_HaveFastChargingDuringSuspend()) {
			/* Disable high current mode in suspend. */
			IOP_Deactivate( pdata->wall_pwr_pin );
		}

		/* Switch off leds */
		ltc3577_set_leds( ltc3577_write, LTC3577_OFF);

		/* Upon suspend we should make sure the right Switching Regulator Mode is set. */
		ltc3577_set_swmode( ltc3577_write, pdata->swmode.suspend );

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
#endif
		ltc3577_i2c_commit( client, ltc3577_write, NULL, 0 );
		atomic_set(&driver->suspended, 1);

		up( &ltc3577_register_cache_lock );
	}
	return 0;
}

static int ltc3577_resume(struct device * dev, u32 level)
{
	struct ltc3577_i2c_driver	*driver = (struct ltc3577_i2c_driver *) container_of( dev->driver, struct ltc3577_i2c_driver, plat_driver );
	struct i2c_client		*client = &driver->client;
	int				rc;

	if( level == RESUME_POWER_ON )
	{
		rc=add_usb_state_change_listener( ltc3577_state_change_listener, driver );
		if( rc != 0 )
		{
			printk( KERN_ERR "LTC3577: Can't register USBMODE change state listener. Aborting.\n" );
			i2c_detach_client( &driver->client );
			return -ENODEV;
		}

		down( &ltc3577_register_cache_lock );

		/* Set PWM mode */
		ltc3577_set_swmode( ltc3577_write, LTC3577_PWM );

		/* Switch on leds */
		ltc3577_set_leds( ltc3577_write, LTC3577_ON);

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
#endif
		atomic_set(&driver->suspended, 0);
		ltc3577_i2c_commit( client, ltc3577_write, NULL, 0 );

		up( &ltc3577_register_cache_lock );
	}
	return 0;
}
#else
#define ltc3577_suspend		NULL
#define ltc3577_resume		NULL
#endif /* CONFIG_PM	*/

#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
static int ltc3577_freq_transition( struct notifier_block *nb, unsigned long val, void *data )
{
	struct ltc3577_i2c_driver	*driver=(struct ltc3577_i2c_driver *) container_of( nb, struct ltc3577_i2c_driver, freq_transition );
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
			break;

		case CPUFREQ_POSTCHANGE :
			break;

		/* Suspend and resume are pretty much the same, in the sense that the voltage */
		/* is set as the handler is called. However, since during resume the I2C is still */
		/* not available at this time, we do the resume part in the resume handler for this*/
		/* driver, not in this code. */
		case CPUFREQ_SUSPENDCHANGE :
			down( &ltc3577_register_cache_lock );
			ltc3577_set_swmode( ltc3577_write, LTC3577_BURST );
			ltc3577_i2c_commit(client, ltc3577_write, NULL, 0);
			up( &ltc3577_register_cache_lock );
			break;

	}
	return 0;
}

static int ltc3577_freq_policy( struct notifier_block *nb, unsigned long val, void *data )
{
  return 0;
}
#endif /* defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ) */

/* Not exactly nice, but since the i2c_add_driver routine already the ltc3577_i2c_driver.driver */
/* structure, and since in there its put to the I2C bus type, it wouldn't be nice to register */
/* the same structure with a platform bus type. To prevent this, register a separate structure*/
/* to actually get the platform data over here. Not exactly neat, but no choice. */
static struct ltc3577_i2c_driver	ltc3577_driver=
{
        .pdata				= NULL,
	.suspended=
	{
		.counter		= 1,
	},
        .i2c_driver=
        {
                .id                     = I2C_DRIVERID_LTC355_PMIC,
                .owner                  = THIS_MODULE,
                .name                   = LTC3577_DEVNAME "_i2c",
                .flags                  = I2C_DF_NOTIFY,
                .attach_adapter         = ltc3577_attach,
                .detach_client          = ltc3577_detach,
        },
        .plat_driver=
        {
                .bus                    = &platform_bus_type,
                .probe                  = ltc3577_probe,
                .name                   = LTC3577_DEVNAME,
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
		.suspend		= ltc3577_suspend,
		.resume			= ltc3577_resume,
        },
	.client=
	{
		.name			= LTC3577_DEVNAME "_i2c_client",
		.flags			= I2C_CLIENT_ALLOW_USE,
		.usage_count		= 0,
		.driver			= &ltc3577_driver.i2c_driver,
		.addr			= LTC3577_I2C_SLAVE_ADDR,
	},
#if defined CONFIG_CPU_FREQ && (defined CONFIG_S3C24XX_DVS_CPUFREQ || defined S3C24XX_DFS_CPUFREQ)
	.freq_transition=
	{
		.notifier_call		= ltc3577_freq_transition,
		.priority		= CPUFREQ_ORDER_S3C24XX_PMIC_PRIO,
	},
	.freq_policy=
	{
		.notifier_call		= ltc3577_freq_policy,
		.priority		= CPUFREQ_ORDER_S3C24XX_PMIC_PRIO,
	},
#endif
};

static int __init ltc3577_init(void)
{
	return driver_register( &ltc3577_driver.plat_driver );
}

static void __exit ltc3577_exit(void)
{
	driver_unregister( &ltc3577_driver.plat_driver );
	return;
}

MODULE_AUTHOR("Andrzej Zukowski <andrzej.zukowski@tomtom.com>");
MODULE_DESCRIPTION("Driver for I2C connected LTC3577 PM IC.");

module_init( ltc3577_init );
module_exit( ltc3577_exit );

