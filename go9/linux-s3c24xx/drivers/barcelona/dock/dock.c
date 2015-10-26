/* drivers/barcelona/dock/dock.c
 *
 * Dock driver for liverpool
 *
 * Copyright (C) 2007 TomTom BV <http://www.tomtom.com/>
 * Author: Mark Vels <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uio.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ns73.h>
#include <linux/mcp23008.h>
#include <linux/kdev_t.h>
#include <linux/workqueue.h>
#include <barcelona/gopins.h>
#include <linux/i2c.h>
#include <../arch/arm/mach-s3c2410/tomtomgo-iopins.h>


#define MODPFX	"Dock: "

/* Empty placeholder to stop the kernel from nagging on a detach. */
static void dock_platform_release( struct device *dev )
{
	return;
}

static u32 virt_dock_pins=0;

static struct fm_transmitter_info	tomtomgo_fmx_info = {
	.slave_address			= NS73_I2C_SLAVE_ADDR,
	.device_nr			= MKDEV( NS73_MAJOR, NS73_MINOR),
};

static struct platform_device		tomtomgo_device_fmx = {
        .name				= NS73_DEVNAME,
        .id				= -1,
        .dev =	{
			.platform_data	= &tomtomgo_fmx_info,
			.release	= dock_platform_release,
		}
};

static struct ioexp_info		tomtomgo_ioexp_info = {
	.slave_address			= MCP23008_I2C_SLAVE_ADDR,
};

static struct platform_device		tomtomgo_device_ioexp = {
	.name				= MCP23008_DEVNAME,
	.id				= -1,
        .dev =	{
			.platform_data	= &tomtomgo_ioexp_info,
			.release	= dock_platform_release,
		}
};

static atomic_t platform_devices_registered = ATOMIC_INIT( 0 );
static struct device *probe_dev = NULL;

extern struct ioexp_handler mcp23008_handle;
static int dock_is_usb_dock( void )
{
	int ret = 0;

	/* Detection is done by first figuring out if the MCP23008 driver is registered or not. */
	/* This means that this routine can only be called AFTER the platform device for the */
	/* MCP23008 is registered. If the device is actually registered, we can assume there */
	/* is no USB dock. */
	switch( atomic_read( &(mcp23008_handle.is_valid) ) )
	{
		case 0 :
		{
			ret = 1;
			break;
		}

		case -1 :
		{
			ret = 1;
			break;
		}

		default :
		{
			ret = 0;
			break;
		}

	}

	return ret;
}

static void dock_irq_debounce_handler( void *not_used )
{
	/* Check the status of our pin. */
	if( IO_GetInput( DOCK_INT ) )
	{
		/* Ensure the device is not already registered. */
		if( atomic_inc_return( &platform_devices_registered ) == 1 )
		{
			/* Non-zero, e.g device is present. Fill in the info. */
			tomtomgo_fmx_info.fm_power_pin=IO_Pin( DOCK_PWREN );

			/* Register the platform devices. This will cause the drivers themselves to start working. */
			platform_device_register( &tomtomgo_device_ioexp );
			platform_device_register( &tomtomgo_device_fmx ); 
		}

		/* Now (for the virtual pins) check if this is a USB dock or not. */
		if( dock_is_usb_dock( ) )
		{
			/* Update the virtual pins. */
			if( GET_PORTNR( IO_Pin( DOCK_DESK_SENSE ) ) == PORT_GPIIC )
				virt_dock_pins|=1 << (GET_PINNR( IO_Pin( DOCK_DESK_SENSE ) ));

			if( GET_PORTNR( IO_Pin( USB_HOST_DETECT ) ) == PORT_GPIIC )
				virt_dock_pins|=1 << (GET_PINNR( IO_Pin( USB_HOST_DETECT ) ));
		}
		else
		{
			if( GET_PORTNR( IO_Pin( DOCK_DESK_SENSE ) ) == PORT_GPIIC )
				virt_dock_pins&=~(1 << (GET_PINNR( IO_Pin( DOCK_DESK_SENSE ) )));
		}

	}
	else
	{
		if( !dock_is_usb_dock( ) )
		{
			if( GET_PORTNR( IO_Pin( DOCK_DESK_SENSE ) ) == PORT_GPIIC )
				virt_dock_pins&=~(1 << (GET_PINNR( IO_Pin( DOCK_DESK_SENSE ) )));
		}

		if( GET_PORTNR( IO_Pin( USB_HOST_DETECT ) ) == PORT_GPIIC )
			virt_dock_pins&=~(1 << (GET_PINNR( IO_Pin( USB_HOST_DETECT ) )));

		/* Ensure the device is not already unregistered. */
		if( atomic_read( &platform_devices_registered ) )
		{
			/* Zero, e.g device is not present any longer. Unregister the devices. */
			platform_device_unregister( &tomtomgo_device_fmx );
			platform_device_unregister( &tomtomgo_device_ioexp );
			atomic_set( &platform_devices_registered, 0 );
		}
	}

	/* Back to interrupt pin. */
	IO_SetInterruptOnToggle( DOCK_INT );
	return;
}

static struct work_struct dock_irq_wq;

static irqreturn_t dock_irq_handler( int irq, void *dev_id, struct pt_regs *regs )
{
	/* Getting a DOCK_INT interrupt. E.g dock was inserted or removed. */  
	/* schedule a tasklet which can wait for the interrupt to debounce. */
	/* We use tasklets so that even if another DOCK_INT IRQ occurs, */
	/* no new tasklet will be scheduled. The tasklet itself can do */
	/* the waiting for the DOCK_INT to be debounced. Note that we */
	/* disable the IRQ also to ensure no spurious interrupts. */
	IO_SetInput( DOCK_INT );

	/* Schedule the workqueue. */
	schedule_delayed_work( &dock_irq_wq, HZ/10 );
	return IRQ_HANDLED;
}

static int dock_probe( struct device *dev )
{
	probe_dev = dev;

	/* Remove power from the DOCK. This to prevent problems when the dock driver is loaded before the MCP23008 driver. */
	/* In late init (after all drivers have been initialized) we will apply power once more, thereby generating an IRQ */
	/* if the dock is present. */
	IO_Deactivate( DOCK_PWREN );
	return 0;
}

static void dock_detect( struct device *dev )
{
	struct platform_device	*pdev;
	struct resource		*res;
	int			dock_stat=0;

	pdev = to_platform_device( dev );
	res = platform_get_resource( pdev, IORESOURCE_IRQ, 0 );

	/* Get the status of the DOCK_INT pin. If it's there the pin is high. */
	/* This to detect whether the dock is there on power up. */
	IO_SetInput( DOCK_INT );
	dock_stat=(IO_GetInput( DOCK_INT ) ? 1 : 0);

	/* Register the interrupt. */
	if( (res == NULL) || request_irq( res->start, dock_irq_handler, SA_INTERRUPT | SA_SAMPLE_RANDOM, "dock_irq", dev ) )
	{
		printk( KERN_ERR MODPFX "Can't register dock interrupt!!\n" );
		return;
	} 

	/* Create the work queue. */
	INIT_WORK( &dock_irq_wq, dock_irq_debounce_handler, NULL );

	/* Now check if a dock is already there. If so, no interrupt is generated. */
	if( dock_stat )
		schedule_work( &dock_irq_wq );
	else
		IO_SetInterruptOnToggle( DOCK_INT );

	/* Apply power to the dock. If a dock is present this will cause an IRQ. */
	IO_Activate( DOCK_PWREN );
	return;
}


static int __init dock_late_probe( void )
{
	if( probe_dev != NULL )
		dock_detect( probe_dev );

	return 0;
}
late_initcall( dock_late_probe );


static int dock_remove( struct device *dev )
{
	struct platform_device	*pdev=to_platform_device( dev );
	struct resource		*res=platform_get_resource( pdev, IORESOURCE_IRQ, 0 );

	/* Release the dock irq. */
	if( res != NULL )
		free_irq( res->start, dev );
	else
	{
		printk( KERN_ERR MODPFX "Can't unregister dock interrupt!!\n" );
		return -ENODEV;
	}

	/* Configure the irq pin back to input. */
	IO_SetInput( DOCK_INT );

	/* Ensure no tasks are on the work queue is dead. */
	cancel_delayed_work( &dock_irq_wq );
	flush_scheduled_work( );

	/* Ensure the device is unregistered. */
	if( atomic_read( &platform_devices_registered ) )
	{
		/* Zero, e.g device is not present any longer. Unregister the devices. */
		platform_device_unregister( &tomtomgo_device_ioexp );
		platform_device_unregister( &tomtomgo_device_fmx );
		atomic_set( &platform_devices_registered, 0 );
	}

	probe_dev = NULL;
	virt_dock_pins=0;

	return 0;
}

static void dock_shutdown( struct device *dev )
{
	dock_remove( dev );
	return;
}

static int dock_suspend( struct device *dev, pm_message_t state, u32 level )
{
	if( level == SUSPEND_POWER_DOWN )
		return dock_remove( dev );
	else
		return 0;
}

static int dock_resume( struct device *dev, u32 level )
{
        if( level == RESUME_POWER_ON ) {
		dock_detect( dev );
	}
	
	return 0;
}

static struct device_driver dock_driver = {
	.owner		= THIS_MODULE,
	.name		= "dock-dongle",
	.bus		= &platform_bus_type,
	.probe		= dock_probe,
	.remove		= dock_remove,
	.shutdown	= dock_shutdown,
	.suspend	= dock_suspend,
	.resume		= dock_resume,

};

int DOCK_TryLock( struct ioexp_handler *ioexp_handle )
{
	unsigned long int	flags;
	int			retval=0;

	local_irq_save( flags );

	if( atomic_read( &(ioexp_handle->is_valid) ) > 0 )
	{
		atomic_inc( &(ioexp_handle->use_count) );
		retval=1;
	}
	else retval=0;

	local_irq_restore( flags );
	return retval;
}

void DOCK_Unlock( struct ioexp_handler *ioexp_handle )
{
	unsigned long int	flags;

	local_irq_save( flags );

	if( atomic_read( &ioexp_handle->use_count ) )
		atomic_dec( &(ioexp_handle->use_count) );

	local_irq_restore( flags );
	return;
}

/* Same interface routines as in tomtomgo-type.c, but now specifically for the IO Expander. */
void DOCK_DisablePullResistor( unsigned pinnr )
{
	int			retval;

	if( pinnr > 8 )
		return;

	if( DOCK_TryLock( &mcp23008_handle ) )
	{
		retval=IOEXP_GetPincfg( &mcp23008_handle, pinnr );
		if( retval < 0 ) goto dock_dispull_done;
		retval &= ~(MCP23008_IOPIN_INTPULLUP);
		if( IOEXP_SetPincfg( &mcp23008_handle, (1 << pinnr), retval ) )
			goto dock_dispull_done;
	}
	else return ; 

dock_dispull_done:
	DOCK_Unlock( &mcp23008_handle );
	return;
}
EXPORT_SYMBOL( DOCK_DisablePullResistor );

void DOCK_SetPullResistor(gopin_t pin)
{
	int			retval;
	unsigned		pinnr=GET_PINNR( pin );

	if( pinnr > 8 )
		return;

	if( DOCK_TryLock( &mcp23008_handle ) )
	{
		retval=IOEXP_GetPincfg( &mcp23008_handle, pinnr );
		if( retval < 0 ) goto dock_setpull_done;
		if( pin & PIN_PULL_UP )
			retval |= MCP23008_IOPIN_INTPULLUP;
		else
			retval &= MCP23008_IOPIN_INTPULLUP;
		if( IOEXP_SetPincfg( &mcp23008_handle, (1 << pinnr), retval ) )
			goto dock_setpull_done;
	}
	else return ;

dock_setpull_done:
	DOCK_Unlock( &mcp23008_handle );
	return;
}
EXPORT_SYMBOL( DOCK_SetPullResistor );

void DOCK_SetBit(gopin_t pin, unsigned value)
{
	int			retval;
	int			pinnr=GET_PINNR( pin );

	if( pinnr < 0 )
		return;

	/* Update the local value also for the virtual pins. */
	if( value )
		virt_dock_pins |= (1 << pinnr);
	else
		virt_dock_pins &= ~(1 << pinnr);

	/* Now update in the chip. */
	if( pinnr < 8 )
	{
		if( DOCK_TryLock( &mcp23008_handle ) )
		{
			/* Set config to output. */
			retval=IOEXP_GetPincfg( &mcp23008_handle, pinnr );
			if( retval < 0 ) goto dock_setbit_done;
			retval|=MCP23008_IOPIN_OUTPUT;
			if( IOEXP_SetPincfg( &mcp23008_handle, (1 << pinnr), retval ) )
				goto dock_setbit_done;

			/* Set the bit. */
			if( IOEXP_SetPinval( &mcp23008_handle, pinnr, value ) < 0 )
				goto dock_setbit_done;

			/* Disable the pull resistor. */
			DOCK_DisablePullResistor( pinnr );
		}
	}
	else return ;

dock_setbit_done:
	DOCK_Unlock( &mcp23008_handle );
	return;
}
EXPORT_SYMBOL( DOCK_SetBit );

signed DOCK_GetInterruptNumber( gopin_t pin )
{
	if( GET_PORTNR( pin ) == PORT_GPIIC )
		return IOP_GetInterruptNumber( IO_Pin( DOCK_INT ) );
	else return -1;
}
EXPORT_SYMBOL( DOCK_GetInterruptNumber );

signed DOCK_SetInterrupt( gopin_t pin )
{
	int			retval;
	int			pinnr=GET_PINNR( pin );

	if( pinnr > 8 )
		return -1;

	if( DOCK_TryLock( &mcp23008_handle ) )
	{
		retval=IOEXP_GetPincfg( &mcp23008_handle, pinnr );
		if( retval < 0 ) goto dock_setint_done;
		retval |= MCP23008_IOPIN_INTERRUPT;
		retval=IOEXP_SetPincfg( &mcp23008_handle, (1 << pinnr), retval );
		if( retval < 0 ) goto dock_setint_done;

		DOCK_SetPullResistor( pin );
	}
	else retval=0;

dock_setint_done:
	DOCK_Unlock( &mcp23008_handle );
	return retval;
}
EXPORT_SYMBOL( DOCK_SetInterrupt );

signed DOCK_SetInterruptOnActivation( gopin_t pin )
{
	int			retval;
	int			pinnr=GET_PINNR( pin );

	if( pinnr > 8 )
		return -1;

	if( DOCK_TryLock( &mcp23008_handle ) )
	{
		retval=IOEXP_GetPincfg( &mcp23008_handle, pinnr );
		if( retval < 0 ) goto dock_setirqonact;

		retval |= MCP23008_IOPIN_INTERRUPT | MCP23008_IOPIN_IRQ_CMPDEFVAL;
		if( PIN_IS_INVERTED(pin) )
			retval|=MCP23008_IOPIN_DEFVAL_HIGH;
		else
			retval&=~(MCP23008_IOPIN_DEFVAL_HIGH);

		retval=IOEXP_SetPincfg( &mcp23008_handle, (1 << pinnr), retval );
		if( retval < 0 ) goto dock_setirqonact;

		DOCK_SetPullResistor( pin );
	}
	else retval=0;

dock_setirqonact:
	DOCK_Unlock( &mcp23008_handle );
	return retval;
}
EXPORT_SYMBOL( DOCK_SetInterruptOnActivation );

signed DOCK_SetInterruptOnDeactivation( gopin_t pin )
{
	int			retval;
	int			pinnr=GET_PINNR( pin );

	if( pinnr > 8 )
		return -1;

	if( DOCK_TryLock( &mcp23008_handle ) )
	{
		retval=IOEXP_GetPincfg( &mcp23008_handle, pinnr );
		if( retval < 0 ) goto dock_setirqondeact;

		retval |= MCP23008_IOPIN_INTERRUPT | MCP23008_IOPIN_IRQ_CMPDEFVAL;
		if( PIN_IS_INVERTED(pin) )
			retval&=~(MCP23008_IOPIN_DEFVAL_HIGH);
		else
			retval|=MCP23008_IOPIN_DEFVAL_HIGH;

		retval=IOEXP_SetPincfg( &mcp23008_handle, (1 << pinnr), retval );
		if( retval < 0 ) goto dock_setirqondeact;

		DOCK_SetPullResistor( pin );
	}
	else retval=0;

dock_setirqondeact:
	DOCK_Unlock( &mcp23008_handle );
	return retval;
}
EXPORT_SYMBOL( DOCK_SetInterruptOnDeactivation );

signed DOCK_SetInterruptOnToggle( gopin_t pin )
{
	int			retval;
	int			pinnr=GET_PINNR( pin );

	if( pinnr > 8 )
		return -1;

	if( DOCK_TryLock( &mcp23008_handle ) )
	{
		retval=IOEXP_GetPincfg( &mcp23008_handle, pinnr );
		if( retval < 0 ) goto dock_setirqontoggle;

		retval |= MCP23008_IOPIN_INTERRUPT;
		retval &= ~(MCP23008_IOPIN_IRQ_CMPDEFVAL);

		retval=IOEXP_SetPincfg( &mcp23008_handle, (1 << pinnr), retval );
		if( retval < 0 ) goto dock_setirqontoggle;

		DOCK_SetPullResistor( pin );
	}
	else retval=0;

dock_setirqontoggle:
	DOCK_Unlock( &mcp23008_handle );
	return retval;
}
EXPORT_SYMBOL( DOCK_SetInterruptOnToggle );

void DOCK_SetInput( gopin_t pin )
{
	int			retval;
	int			pinnr=GET_PINNR( pin );

	if( pinnr > 8 )
		return;

	if( DOCK_TryLock( &mcp23008_handle ) )
	{
		retval=IOEXP_GetPincfg( &mcp23008_handle, pinnr );
		if( retval < 0 ) goto dock_setinput;

		retval &= ~(MCP23008_IOPIN_OUTPUT); 

		if( IOEXP_SetPincfg( &mcp23008_handle, (1 << pinnr), retval ) < 0 )
			goto dock_setinput;

		DOCK_SetPullResistor( pin );
	}
	else return ;

dock_setinput:
	DOCK_Unlock( &mcp23008_handle );
	return;
}
EXPORT_SYMBOL( DOCK_SetInput );

int DOCK_GetInput( gopin_t pin )
{
	int			retval;
	int			pinnr=GET_PINNR( pin );
	int			locked;

	locked=DOCK_TryLock( &mcp23008_handle );
	if( !locked )
		virt_dock_pins &= ~(1 << GET_PINNR( IO_Pin( DOCK_SENSE ) ));
	else
		virt_dock_pins |= 1 << GET_PINNR( IO_Pin( DOCK_SENSE ) );

	if( pinnr < 0 )
	{
		if( locked ) DOCK_Unlock( &mcp23008_handle );
		return 0;
	}
	else if( pinnr >= 8 )
	{
		if( locked ) DOCK_Unlock( &mcp23008_handle );
		retval=(virt_dock_pins & (1 << pinnr) ? 1 : 0 );
		return (PIN_IS_INVERTED(pin) ? (retval ^ 1) : retval);
	}

	if( locked )
	{
		retval=IOEXP_GetPinval( &mcp23008_handle, pinnr );
		if( retval < 0 ) retval=0;
		if( retval )
			retval=(PIN_IS_INVERTED( pin ) ? 0 : 1);
		else
			retval=(PIN_IS_INVERTED( pin ) ? 1 : 0);
	}
	else return 0;

	DOCK_Unlock( &mcp23008_handle );
	return retval;
}
EXPORT_SYMBOL( DOCK_GetInput );

static int __init dock_init( void )
{
	/* Signon message. */
	printk( "Valencia/Murcia dock driver - v1.0\n" );
	return driver_register( &dock_driver );
}

static void __exit dock_exit( void )
{
	driver_unregister( &dock_driver );
}

module_init( dock_init );
module_exit( dock_exit );

MODULE_DESCRIPTION( "Dock driver for Valencia and Murcia" );
MODULE_LICENSE( "GPL" );
MODULE_AUTHOR( "Rogier Stam <rogier.stam@tomtom.com>" );

