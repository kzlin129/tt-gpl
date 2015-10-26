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
 *   @file   bcm_basedefs.h 
 * 
 *   @brief  This file contains standard macros to be within the
 *           broadcom MOBM source domain.  Defines in this header should be
 *           application neutral.
 * 
 ****************************************************************************/


#ifndef DOXYGEN_IGNORE_EXTERNAL
//----------------------------------------------------------
/**

 @file bcm_basedefs.h

    This file contains standard macros to be within the
    broadcom MOBM source domain.  Defines in this header should be
    application neutral.  

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

#ifndef _BCM_BASEDEFS_H_
#define _BCM_BASEDEFS_H_

#if defined __GNUC__
#define BCM_INLINE static inline
#elif defined __arm__
#define BCM_INLINE __inline
#error "Error: Other compilers not supported yet"
#endif

#include "bcm_basetypes.h"
#include "bcm_log.h"



/***********************************************************
*
* Included files
*
***********************************************************/

/***********************************************************
*
* Defined values
*
***********************************************************/

/***********************************************************
*
* Enumerated types
*
***********************************************************/

/***********************************************************
*
* Macro definitions
*
***********************************************************/

//--------------------------------------------------
/**
  ARRAYSIZE 

  @ingroup BCM_BASETYPES
*/
//--------------------------------------------------
#define ARRAYSIZE(a)	((sizeof(a))/(sizeof((a)[0])))

//--------------------------------------------------
/**
  offsetof requires definition on some platforms

  @ingroup BCM_BASETYPES
*/
//--------------------------------------------------
#ifndef offsetof
#define offsetof(type, memb) ((uint32_t)(&((type *)0)->memb))
#endif                       

#ifndef assert
BCM_INLINE void breakpoint(void)
{
    __asm("BKPT 0x2730");    
}

#ifdef  NDEBUG
    #define BCM_ASSERT(cond) (void)0
#else
    #define BCM_ASSERT(cond)    \
        if (cond)           \
        {                   \
          /** do nothing */ \
        }                   \
        else                \
        {                   \
			bcm_log_crit("Assert failed: '%s' [%s:%d, %s()]\n", \
								#cond, __FILE__, __LINE__, __FUNCTION__ ); \
			breakpoint(); \
        }
#endif
#else //assert already defined
    #define BCM_ASSERT assert
#endif

/**
    The following are register read/write macros
  */


#endif /* _BCM_BASEDEFS_H_ */

