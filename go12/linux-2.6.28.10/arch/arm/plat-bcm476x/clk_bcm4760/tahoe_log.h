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
 *   @file   tahoe_log.h
 * 
 *   @brief  Logging interface.
 * 
 ****************************************************************************/


#ifndef DOXYGEN_IGNORE_EXTERNAL
//----------------------------------------------------------
/**

 @file tahoe_log.h

    PUBLIC logging interface. Users of Tahoe library can do things
    like setting the various log levels

 @todo 

 @author Seetharam Samptur

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

#ifndef _TAHOE_LOG_H_
#define _TAHOE_LOG_H_


typedef enum _bcmlog_level_t
{
    BCMLOG_CRIT  = 0x00000001, /**< Critical message       */
    BCMLOG_ERROR = 0x00000002, /**< Error message          */
    BCMLOG_WARN  = 0x00000004, /**< Warning messages       */
    BCMLOG_INFO  = 0x00000008, /**< Informational messages */
    BCMLOG_DEBUG = 0x00000010, /**< Debug messages         */
    BCMLOG_ALL   = 0x0000001f  /**< ALL messages           */ 

} bcmlog_level_t;

void		    tahoe_set_log_level(unsigned int log_level);
unsigned int    tahoe_get_log_level(void);

#endif /* _TAHOE_LOG_H_ */
