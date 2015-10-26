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




#include <stdarg.h>

#include "vchlfunc.h"
#include "vcinterface.h"
#include "vcgencmd.h"

/******************************************************************************
Global data.
******************************************************************************/

// this should be the only module where vc_gencmd_send_list is referenced,
// hence it is not declared in any headers
extern int vc_gencmd_send_list ( const char *format, va_list a );

/******************************************************************************
Local types and defines.
******************************************************************************/

/******************************************************************************
Static data.
******************************************************************************/

/******************************************************************************
Static functions.
******************************************************************************/

/******************************************************************************
Global functions.
******************************************************************************/

/******************************************************************************
NAME
   vc_hlfunc_gencmd

SYNOPSIS
   int vc_hlfunc_gencmd(char *response, int maxlen, const char *command, ...)

FUNCTION
   Take in a printf style command string, a pointer to a string into which the
   response can be put, the string length and finally any parameters that relate
   to the input command string. Returns non-zero to indicate an error

RETURNS
   int
******************************************************************************/

int vc_hlfunc_gencmd(char *response, int maxlen, const char *command, ...)
{
  va_list   a;
  int       rv;

  // send the command, and get the return code to check
  va_start( a, command );
  rv = vc_gencmd_send_list( command, a );
  va_end( a );

  if( rv == 0 ) {
    // we're still here and expecting a response
    rv = vc_gencmd_read_response( response, maxlen );
  }

  return rv;
}
