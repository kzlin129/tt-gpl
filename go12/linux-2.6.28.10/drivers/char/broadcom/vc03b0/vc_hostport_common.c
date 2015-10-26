/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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



/*****************************************************************************
*
*   This file contains functions described in vc03b0_hostport.h which are
*   common across all platforms.
*
*****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/preempt.h>
#include <linux/broadcom/vc03/vc03b0_hostport.h>
#include <linux/broadcom/videocore_settings.h>
#include <linux/broadcom/knllog.h>
#include <linux/broadcom/gpio.h>

/* ---- Public Variables ------------------------------------------------- */

// Setting gVcDebugHostPortTrace causes entering and exiting functions to be printed
//      A value of 1 will cause printk to be used, and a value of 2 will cause KNLLOG to be used
// Setting gVcDebugHostPortData  causes upto gVcDebugHostPortDataSize bytes to be printed for each read/write

int     gVcDebugHostPortTrace       = 0;    ///< Mapped to /proc/vc/debug/host-port-trace
int     gVcDebugHostPortData        = 0;    ///< Mapped to /proc/vc/debug/host-port-data
int     gVcDebugHostPortDataSize    = 64;   ///< Mapped to /proc/vc/debug/host-port-data-size
int     gVcDebugHostPortRwPerf      = 0;

EXPORT_SYMBOL( gVcDebugHostPortTrace );
EXPORT_SYMBOL( gVcDebugHostPortData );
EXPORT_SYMBOL( gVcDebugHostPortDataSize );
EXPORT_SYMBOL( gVcDebugHostPortRwPerf );

/* ---- Private Constants and Types -------------------------------------- */

/* ---- Private Variables ------------------------------------------------ */

/* ---- Private Function Prototypes -------------------------------------- */

/* ---- Functions  ------------------------------------------------------- */

/****************************************************************************/
/**
*   Logging function used for reporting debug prints
*/
/****************************************************************************/

void vchost_vlog( const char *function, int logType, const char *fmt, va_list args )
{
    if (( logType & 1 ) != 0 )
    {
        // Wrap the multiple calls to printk in preempt_dsiable/enable so that 
        // klogd doesn't try to treat them as 3 separate messages
        
        preempt_disable();
        printk( KERN_INFO "%s: ", function );
        vprintk( fmt, args );
        printk( "\n" );
        preempt_enable();
    }
    if (( logType & 2 ) != 0 )
    {
        knllog_ventry( function, fmt, args );
    }
}
EXPORT_SYMBOL( vchost_vlog );

/****************************************************************************/
/**
*   Logging function used for reporting debug prints
*/
/****************************************************************************/

void vchost_log( const char *function, int logType, const char *fmt, ... )
{
    va_list args;

    va_start( args, fmt );
    vchost_vlog( function, logType, fmt, args );
    va_end( args );
}
EXPORT_SYMBOL( vchost_log );

#if defined( CONFIG_HAVE_GPIO_LIB )

// For platforms which support gpiolib, these common implementations of the 
// functions will suffice. Otherwise, the platform will need to implement 
// the functions.

/****************************************************************************/
/**
*   Requests that a Gpio pin be reserved for a speific purpose.
*/
/****************************************************************************/

int vchost_gpio_request( unsigned pin, const char *label )
{
    return gpio_request( pin, label );
}
EXPORT_SYMBOL( vchost_gpio_request );

/****************************************************************************/
/**
*   Releases a gpio pin so that it's no longer used.
*/
/****************************************************************************/

void vchost_gpio_free( unsigned pin )
{
    gpio_free( pin );
}
EXPORT_SYMBOL( vchost_gpio_free );

/****************************************************************************/
/**
*   Sets a gpio pin to be used for input
*/
/****************************************************************************/

int vchost_gpio_direction_input( unsigned pin )
{
    return gpio_direction_input( pin );
}
EXPORT_SYMBOL( vchost_gpio_direction_input );

/****************************************************************************/
/**
*   Sets a gpio pin to be used for input
*/
/****************************************************************************/

int vchost_gpio_direction_output( unsigned pin, int initialValue )
{
    return gpio_direction_output( pin, initialValue );
}
EXPORT_SYMBOL( vchost_gpio_direction_output );

/****************************************************************************/
/**
*   Sets a gpio pin to be used for input
*/
/****************************************************************************/

void vchost_gpio_set_value( unsigned pin, int value )
{
    gpio_set_value( pin, value );
}
EXPORT_SYMBOL( vchost_gpio_set_value );

/****************************************************************************/
/**
*   Retrieves the value of a gpio pin
*/
/****************************************************************************/

int vchost_gpio_get_value( unsigned pin )
{
    return gpio_get_value( pin );
}
EXPORT_SYMBOL( vchost_gpio_get_value );

#endif  // CONFIG_HAVE_GPIO_LIB

/****************************************************************************/
/**
*   Initializes the pins required to talk to the VideoCore.
*
*   This function is deprecated and will be replaced by 
*   vchost_gpio_set_value()
*/
/****************************************************************************/

#if 0
void vchost_pininit( void )
{
   vchost_gpio_request( HW_VC03_INT_GPIO, "vc-irq" );
   vchost_gpio_request( HW_VC03_RUN_GPIO, "vc-run" );

   vchost_gpio_direction_output( HW_VC03_RUN_GPIO, 0 );

} // vchost_pininit

EXPORT_SYMBOL( vchost_pininit );
#endif

/****************************************************************************/
/**
*   Delays for the indicated number of milliseconds. Note that this function
*   will delay for at least the amount of time indicated, and may delay for
*   longer.
*/
/****************************************************************************/

void vchost_delay_msec( unsigned msec )
{
    // Make sure that we're not called from interrupt context

    might_sleep();

    if ( msec < 20 )
    {
        mdelay( msec );
    }
    else
    {
        msleep( msec );
    }

} // vchost_delay_msec

EXPORT_SYMBOL( vchost_delay_msec );

