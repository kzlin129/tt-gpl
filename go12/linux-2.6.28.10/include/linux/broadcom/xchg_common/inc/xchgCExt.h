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





/*
*
****************************************************************************
*
*    Description:
*      This file contains macros for using compiler dependent C language
*      extensions
*
*****************************************************************************/


#ifndef _XCHG_CEXT_H
#define _XCHG_CEXT_H

/* ---- Include Files ---------------------------------------- */
#include <xchgCExtCfgCustom.h>

/* ---- Constants and Types ---------------------------------- */

#ifndef XCHG_CEXT_SECTION_CFG_ENABLE
#define XCHG_CEXT_SECTION_CFG_ENABLE   1
#endif

#if XCHG_CEXT_SECTION_CFG_ENABLE
#ifdef __GNUC__
/* GNU C compiler */
#define XCHG_CEXT_SECTION(x)           __attribute__((section(x)))
#else
/* Default */
#define XCHG_CEXT_SECTION(x)
#endif
#endif

#ifndef XCHG_CEXT_EXPORT_SYMBOL
#define XCHG_CEXT_EXPORT_SYMBOL(x)
#endif

#ifndef XCHG_CEXT_SECTION_FAST
/* The ".text.xchg_fast" section can be applied to functions that are called
*  often.  Since these functions are grouped contiguously in memory, the
*  probablity of the code overlapping within the cache sets is minimized,
*  reducing the need for the functions to swap with each other in cache.
*  Obviously, the chances of overlap/swapping is dependent on the cache size.
*/
#define XCHG_CEXT_SECTION_FAST         XCHG_CEXT_SECTION( ".text.xchg_fast" )
#endif


/* ---- Variable Externs ------------------------------------- */
/* ---- Function Prototypes ---------------------------------- */


#endif /* XCHG_CEXT_H */
