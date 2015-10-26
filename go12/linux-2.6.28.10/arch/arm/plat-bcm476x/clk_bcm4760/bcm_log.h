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
 *   @file   bcm_log.h
 * 
 *   @brief  Logging api
 * 
 ****************************************************************************/


#ifndef _BCM_LOG_H_
#define _BCM_LOG_H_

/***********************************************************
*
* Included files
*
***********************************************************/

#include "bcm_os_support.h"
#include "bcm_basetypes.h"
#include "tahoe_log.h"

#define SYM_ENABLE_LOG  //Enable for dev.. later we can modify the makefile to include/exclude this define



#ifdef SYM_ENABLE_LOG
//--------------------------------------------------
/**
  Log level variable that has to be defined by the module that
  will use the logger. 

  @ingroup BCM_BASEDEFS
*/
//--------------------------------------------------
// defined in bcm_log.c
int			bcm_log(unsigned int log_level, char *fmt, ...);

#define bcm_log_crit(fmt, args...)         bcm_log(BCMLOG_CRIT, fmt, ##args)

#define bcm_log_error(fmt, args...)         bcm_log(BCMLOG_ERROR, fmt, ##args)

#define bcm_log_warn(fmt, args...)         bcm_log(BCMLOG_WARN, fmt, ##args)

#define bcm_log_info(fmt, args...)         bcm_log(BCMLOG_INFO, fmt, ##args)

#define bcm_log_debug(fmt, args...)        bcm_log(BCMLOG_DEBUG, fmt, ##args)

#else  //Compile time logging OFF
#define bcm_log_crit(fmt, args ...)             do { } while (0)

#define bcm_log_error(fmt, args ...)             do { } while (0)

#define bcm_log_warn(fmt, args...)         do { } while (0)

#define bcm_log_info(fmt, args...)         do { } while (0)

#define bcm_log_debug(fmt, args...)        do { } while (0)

#endif


#endif /* _BCM_LOG_H_ */
