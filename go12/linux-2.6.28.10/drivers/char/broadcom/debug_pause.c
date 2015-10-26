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
*  debug_pause.c
*
*  PURPOSE:
*
*   Allows the kernel to pause during the boot process. This allows us to
*   break into the kernel using the JTAG.
*
*  NOTES:
*
*****************************************************************************/


#include <asm/types.h>
#include <asm/setup.h>

#include <linux/string.h>
#include <linux/init.h>
#include <linux/broadcom/debug_pause_serial.h>

#if defined( CONFIG_KGDB )
#   include <linux/kgdb.h>
#endif



static int __init debug_waitkey( void )
{
    int  i;
    char ch;

    for ( i = 0; i < 600000; i++ )
    {
        if ( debug_pause_getc( &ch ) == 0 )
        {
            return ( ch );
        }
    }

    // No character detected

    return -1;
}

int __initdata debug_pause_on_cmdline = 0;

int __init debug_pause( const char *prompt )
{
    int i;
    int keyPressed = 0;

    debug_pause_puts( "Press a key to pause. - " );
    debug_pause_puts( prompt );

    for ( i = 0; i < 10; i++ )
    {
        int ch;

        if (( ch =  debug_waitkey()) >= 0 )
        {
            // User pressed a key. Wait for another to continue

            keyPressed = 1;
            debug_pause_puts( "\nPress a key to continue..." );
            while (( ch = debug_waitkey()) < 0 )
            {
                ;
            }
            break;
        }
        debug_pause_puts( "." );
    }
    debug_pause_puts( "\nContinuing...\n" );

    return ( keyPressed );
}

void __init debug_break( const char *prompt )
{
    int i;

    debug_pause_puts( prompt );

    for ( i = 0; i < 10; i++ )
    {
        int ch;

        if (( ch =  debug_waitkey()) >= 0 )
        {
#if defined( CONFIG_KGDB )

            // User pressed a key. Do the KGDB breakpoint

            breakpoint();
#else

            debug_pause_puts( "KGDB not configured\n" );

#endif  // CONFIG_KGDB

            return;
        }
        debug_pause_puts( "." );
    }
    debug_pause_puts( "\nContinuing...\n" );
}
