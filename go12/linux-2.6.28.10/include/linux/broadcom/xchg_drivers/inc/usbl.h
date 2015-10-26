/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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





/****************************************************************************/
/**
*  @file    usbl.h
*
*  @brief   Lower USB device driver header file
*
*  This file contains the lower device driver header information for the
*  USB module.
*/
/****************************************************************************/

#ifndef USBL_H
#define USBL_H


/* ---- Include Files ----------------------------------------------------- */

#include <xdrvTypes.h>


#ifdef __cplusplus
extern "C"
{
#endif

/* ---- Public Constants and Types ---------------------------------------- */

typedef struct
{
    XDRV_SINT32 (* initFp)( XDRV_UINT32 usbNum, XDRV_UINT32 byteSwap );
    XDRV_SINT32 (* ohciRegBaseAddrGetFp)( XDRV_UINT32 usbNum, XDRV_UINT32 *addrP  );
}
USBL_FUNCS;


typedef struct
{
   /* This must be the first structure member (but why??) */
   const USBL_FUNCS *funcP;
   const XDRV_UINT32 usbCnt;
}
USBL_DRV;


/* ---- Public Variables -------------------------------------------------- */


/* ---- Public Function Prototypes ---------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif /* USBL_H */
