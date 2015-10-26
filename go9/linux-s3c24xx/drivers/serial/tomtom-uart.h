/* tomtom-uart.h
 *
 * Extra definitions for the tomtomGo external uart on some GO models.
 * The standard 8250 driver needs these functions to be able to 
 * init the hardware apropriately.  
 *
 * Copyright (C) 2004,2006 TomTom BV <http://www.tomtom.com/>
 * Author: Mark Vels <mark.vels@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __SERIAL_8250_TOMTOM_UART__
#define __SERIAL_8250_TOMTOM_UART__

#include <linux/config.h>



/**
 * Get the clock rate of the external uart clock.
 */ 
extern unsigned int hw_tomtom_external_uart_get_clock_rate ( void );


/**
 * Put the external uart in power save mode.
 */ 
extern void hw_tomtom_external_uart_suspend	( void );

/**
 * Revive the external uart back from power save mode.
 */ 
extern void hw_tomtom_external_uart_resume	( void );

/**
 * Hardware reset of the external uart
 */ 
extern void hw_tomtom_external_uart_reset		( void );

/**
 * Initialize all the necessary IO lines to start using the external UART.
 */ 
extern void hw_tomtom_external_uart_init		( void );

/**
 * Deinitialize all IO lines.
 */ 
extern void hw_tomtom_external_uart_exit		( void );

#endif // __SERIAL_8250_TOMTOM_UART__
