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




/*=============================================================================

Project  :  VCFW
Module   :  chip driver
File     :  $RCSfile$
Revision :  $Revision$

FILE DESCRIPTION
Configuration parameters for the MPHI chip driver
=============================================================================*/

#include "vchios.h"


/******************************************************************************
 Parameters that can be overridden by the platform makefile
 *****************************************************************************/

#ifndef MPHI_CFG_H_
#define MPHI_CFG_H_


// typedef function declarations (NOT function pointers)
typedef int32_t  MPHI_OPEN_FUNC( void (*lisr_callback)(void) );
typedef int32_t  MPHI_CLOSE_FUNC( void );
typedef void     MPHI_ENABLE_INTERRUPTS_FUNC( void );
typedef void     MPHI_DISABLE_INTERRUPTS_FUNC( void );
typedef uint32_t MPHI_READ_FUNC( uint32_t reg );
typedef void     MPHI_WRITE_FUNC( uint32_t reg, uint32_t value );
typedef void     MPHI_WRITE_AXIPRIV_FUNC( uint32_t value );


#ifdef MPHI_OPEN
extern MPHI_OPEN_FUNC MPHI_OPEN;
#else
#   define MPHI_OPEN NULL
#endif

#ifdef MPHI_CLOSE
extern MPHI_CLOSE_FUNC MPHI_CLOSE;
#else
#   define MPHI_CLOSE NULL
#endif

#ifdef MPHI_ENABLE_INTERRUPTS
extern MPHI_ENABLE_INTERRUPTS_FUNC MPHI_ENABLE_INTERRUPTS;
#else
#   define MPHI_ENABLE_INTERRUPTS NULL
#endif

#ifdef MPHI_DISABLE_INTERRUPTS
extern MPHI_DISABLE_INTERRUPTS_FUNC MPHI_DISABLE_INTERRUPTS;
#else
#   define MPHI_DISABLE_INTERRUPTS NULL
#endif

#ifdef MPHI_READ
extern MPHI_READ_FUNC MPHI_READ;
#else
#   define MPHI_READ NULL
#endif

#ifdef MPHI_WRITE
extern MPHI_WRITE_FUNC MPHI_WRITE;
#else
#   define MPHI_WRITE NULL
#endif

#ifdef MPHI_WRITE_AXIPRIV
extern MPHI_WRITE_AXIPRIV_FUNC MPHI_WRITE_AXIPRIV;
#else
#   define MPHI_WRITE_AXIPRIV NULL
#endif


#endif /* MPHI_CFG_H_ */
