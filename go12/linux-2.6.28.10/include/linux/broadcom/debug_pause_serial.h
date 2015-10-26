/*****************************************************************************
* Copyright 2005 - 2008 Broadcom Corporation.  All rights reserved.
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




/*
*
*****************************************************************************
*
*  debug_pause_serial.h
*
*  PURPOSE:
*
*  Platform independent API for helper functions used by debug pause, which
*  allows the kernel to pause during the boot process. This allows us to
*  break into the kernel using the JTAG or KGDB.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( DEBUG_PAUSE_SERIAL_H )
#define DEBUG_PAUSE_SERIAL_H

/* ---- Include Files ---------------------------------------------------- */

#if CONFIG_BCM_DEBUG_PAUSE


/* ---- Constants and Types ---------------------------------------------- */
/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

/****************************************************************************
*
*  debug_pause_puts
*
*       Write string to serial console.
*
***************************************************************************/
void debug_pause_puts( const char *str );


/****************************************************************************
*
*  debug_pause_getc
*
*       Get char from serial console. This function is non-blocking.
*       It returns 0 if a char is available, 1 otherwise.
*
***************************************************************************/
int debug_pause_getc( char *ch );

#endif   /* CONFIG_BCM_DEBUG_PAUSE */
#endif  // DEBUG_PAUSE_SERIAL_H
