/* drivers/serial/tomtom-uart.c
 *
 * Implementation of several functions needed to let the 
 * standard 8250 serial driver initialize the external 
 * UART found on some TomTom GO boards.  
 *
 * Copyright (C) 2004,2006 TomTom BV <http://www.tomtom.com/>
 * Authors:
 * Mark Vels <mark.vels@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
 
#if defined(CONFIG_BARCELONA_EXTERNAL_UART) || defined(CONFIG_BARCELONA_EXTERNAL_UART_MODULE)
 
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/err.h>


#include <asm/io.h>
#include <asm/hardware/clock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-dsc.h>
#include <asm/arch/regs-irq.h>


#include <barcelona/gopins.h>
#include <barcelona/gotype.h>

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

/*
#define SAY(X...)    { char b[128];sprintf(b, X);printk( KERN_CRIT "%s line %d %s() says: \"%s\"\n"            , __FILE__, __LINE__, __FUNCTION__, b); }
#define TELL(X)   { printk( KERN_CRIT "%s line %d %s() tells: %s = %d (0x%02X)\n" , __FILE__, __LINE__, __FUNCTION__, #X, X, X); }
#define POINTER(X){ printk( KERN_CRIT "%s line %d %s() tells: %s = 0x%02X\n" , __FILE__, __LINE__, __FUNCTION__, #X, X); }
#define YELL(X)   { printk( KERN_CRIT "%s line %d %s() yells: %s = \"%s\"\n"      , __FILE__, __LINE__, __FUNCTION__, #X, X); }
#define HERE()    { printk( KERN_CRIT "%s line %d %s() HERE!\n"                   , __FILE__, __LINE__, __FUNCTION__); }
*/

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

static struct clk* xuart_clk;


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------


void hw_tomtom_external_uart_suspend( void )
{
	if( xuart_clk != NULL )
	{
		clk_disable( xuart_clk );
	}
}
EXPORT_SYMBOL(hw_tomtom_external_uart_suspend);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------


void hw_tomtom_external_uart_resume( void )
{
	if( xuart_clk != NULL )
	{
		clk_enable( xuart_clk );
	}
}
EXPORT_SYMBOL(hw_tomtom_external_uart_resume);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

unsigned int hw_tomtom_external_uart_get_clock_rate ( void )
{
	unsigned int rate;
	
	if( xuart_clk == NULL )
	{
		printk( "TomTom GO External Uart: get_clock_rate() : xuart_clk is NULL!\n");
		return 0; // will be regarded by 8250.c as error and not register this port.
	}
	
	rate = (unsigned int) clk_get_rate( xuart_clk );
	printk( "TomTom GO External Uart: xuart_clk clock rate is %u Hz\n", rate );
	return rate;
}
EXPORT_SYMBOL(hw_tomtom_external_uart_get_clock_rate);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void hw_tomtom_external_uart_reset( void )
{
	IO_Activate( UART_RESET );
	ndelay( 400 ); /* according to spec, 40 ns would do already	*/
	IO_Deactivate( UART_RESET );
}
EXPORT_SYMBOL(hw_tomtom_external_uart_reset);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void hw_tomtom_external_uart_init( void )
{
	int ret;

	if( IO_HaveExternalUart() )
  { 
		printk( "TomTom GO External Uart support is initializing...");
		/* Configure	*/
		IO_SetInterruptOnActivated( UART_INTA );
		if( IO_HasPin( UART_INTB ) ){	/* newcastle hw, has two uart channels	*/
			IO_SetInterruptOnActivated( UART_INTB );
		}
		IO_SetFunction( UART_CSA );
		if( IO_HasPin( UART_CSB ) ){
			IO_SetFunction( UART_CSB );
		}

		/* UART_PWRSAVE and UART_RESET are outputs, so no init needed.	*/
		
		/* now set the correct states	*/
		IO_Deactivate( UART_PWRSAVE ); /* we are not in Power save mode	now */
		IO_Deactivate( UART_RESET );
	
		xuart_clk = clk_get( NULL, "xuart");
		if( IS_ERR( xuart_clk) )
		{
			// Ooops, clock not found.
			printk( KERN_ERR "Unable to find clock \"xuart\"\n");
			xuart_clk = NULL; // indicate error for calls in this file.
		}
		else if( (ret = clk_use( xuart_clk ) ) )
		{
			printk( KERN_ERR "Unable to mark clock \"xuart\" as in use! clk_use() returned %d\n", ret);
			xuart_clk = NULL;
		}
		else
		{
			clk_enable( xuart_clk );
			// TODO: we can decrease power consumption more by not enabling the clock
			// until it is realy being used. Create a hook from serial8250_startup(struct uart_port *port)
	  }
	  
	  hw_tomtom_external_uart_reset();
		printk( "done\n" );
	}
}
EXPORT_SYMBOL(hw_tomtom_external_uart_init);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void hw_tomtom_external_uart_exit( void )
{
		/* Only start poking the PIO config after we made sure that we're correct hardware type.	*/
	if( IO_HaveExternalUart() )
  { 
  	printk( "TomTom GO External Uart support is deinitializing...");
		IO_Deactivate( UART_INTA );
		if( IO_HasPin( UART_INTB ) )
		{	/* newcastle hw, has two uart channels	*/
				IO_Deactivate( UART_INTB );
		}
		
		IO_Activate( UART_PWRSAVE ); /* put IC in power save mode	*/
		if( xuart_clk != NULL )
		{
			clk_disable( xuart_clk );
			clk_unuse( xuart_clk );
			clk_put( xuart_clk );
			xuart_clk = NULL;
		}
		
		printk( "done\n");
	}
}
EXPORT_SYMBOL(hw_tomtom_external_uart_exit);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

#endif // CONFIG_BARCELONA_EXTERNAL_UART || CONFIG_BARCELONA_EXTERNAL_UART_MODULE

// EOF

