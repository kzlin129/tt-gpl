/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
/** 
 * 
 *   @file   bcm_log.c 
 * 
 *   @brief  Logging functions.
 * 
 ****************************************************************************/


#ifndef DOXYGEN_IGNORE_EXTERNAL
//----------------------------------------------------------
/**
 @file bcm_log.c
	
 @author Joyjit Nath

    Copyright 2004 - 2007 Broadcom Corporation.  All rights reserved.

    Unless you and Broadcom execute a separate written software license
    agreement governing use of this software, this software is licensed to you
    under the terms of the GNU General Public License version 2, available at
    http://www.gnu.org/copyleft/gpl.html (the "GPL").

    Notwithstanding the above, under no circumstances may you combine this
    software in any way with any other Broadcom software provided under a
    license other than the GPL, without Broadcom's express prior written
    consent.
*/
//----------------------------------------------------------
#endif // DOXYGEN_IGNORE_EXTERNAL

#include "tahoe_log.h"
#include "bcm_log.h"


static unsigned int g_loglevel = BCMLOG_INFO;


//public function for external consumption
void tahoe_set_log_level(unsigned int log_level)
{
	g_loglevel = log_level;
}

//public function for external consumption
unsigned int  tahoe_get_log_level(void)
{
	return g_loglevel;
}

#ifdef SYM_ENABLE_LOG
// public function, but only for internal consumption
int bcm_log(unsigned int log_level, char *fmt, ...) 
{
	const char *prefix;
    int			ret;
    va_list		args;

    //Logging message at one level is too restrictive.
    //We allow the user to select a log and all messages that are
    //less than equal to the requested level will be printed.

    //For ex: if the use chose BCMLOG_WARN, 
    //        the following will be displayed: CRIT, ERROR and WARN.
    if (log_level > g_loglevel)
		return 0;

	switch(log_level)
	{
		case BCMLOG_CRIT:
			prefix = "CRITICAL: ";
			break;

		case BCMLOG_ERROR:
			prefix = "ERROR: ";
			break;

		case BCMLOG_WARN:
			prefix = "WARNING: ";
			break;

		case BCMLOG_INFO:
			prefix = "INFO: ";
			break;

		case BCMLOG_DEBUG:
		default:
			prefix = "DEBUG: ";
			break;

	}

	ret = printk(prefix);
    va_start(args, fmt);
    ret += bcm_vprintf(fmt, args);
    va_end(args);

    return(ret);
}  


#endif /* #ifdef SYM_ENABLE_LOG */
