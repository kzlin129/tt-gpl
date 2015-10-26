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
*  @file    ebil.h
*
*  @brief   Lower ebi device driver header file
*
*  This file contains the lower device driver header information for the
*  ebi module.
*/
/****************************************************************************/

#ifndef EBIL_H
#define EBIL_H


/* ---- Include Files ----------------------------------------------------- */

/* ---- Public Constants and Types ---------------------------------------- */

typedef struct
{
   unsigned int (*ebilBaseAddrSet)(  int ch, unsigned int addr );
   unsigned int (*ebilBaseAddrGet)(  int ch );
   unsigned int (*ebilSizeSet)(  int ch, unsigned int size );
   unsigned int (*ebilSizeGet)(  int ch );
   unsigned int (*ebilWaitStatesSet)(  int ch, unsigned int waitStates );
   unsigned int (*ebilWaitStatesGet)(  int ch );
   unsigned int (*ebilBusWidthSet)(  int ch, unsigned int width );
   unsigned int (*ebilBusWidthGet)(  int ch );
   int (*ebilEnable)(  int ch );
   int (*ebilDisable)(  int ch );
   int (*ebilIsEnabled)( int ch );
   int (*ebilIsChValid)( int ch );  
   int (*ebilTransmitModeSet)( int ch, unsigned int mode ); 
   int (*ebilTransmitModeGet)( int ch );
} EBIL_FUNCS;

#define ebilBaseAddrSet( drvp, ch, addr )          (drvp)->funcp->ebilBaseAddrSet( ch, addr )
#define ebilBaseAddrGet( drvp, ch )                (drvp)->funcp->ebilBaseAddrGet( ch ) 
#define ebilSizeSet( drvp, ch, size )              (drvp)->funcp->ebilSizeSet( ch, size ) 
#define ebilSizeGet( drvp, ch )                    (drvp)->funcp->ebilSizeGet( ch )
#define ebilWaitStatesSet( drvp, ch, waitStates )  (drvp)->funcp->ebilWaitStatesSet( ch, waitStates )
#define ebilWaitStatesGet( drvp, ch )              (drvp)->funcp->ebilWaitStatesGet( ch )
#define ebilBusWidthSet( drvp, ch, width )         (drvp)->funcp->ebilBusWidthSet( ch, width ) 
#define ebilBusWidthGet( drvp, ch )                (drvp)->funcp->ebilBusWidthGet( ch ) 
#define ebilEnable( drvp, ch )                     (drvp)->funcp->ebilEnable( ch )
#define ebilDisable( drvp, ch )                    (drvp)->funcp->ebilDisable( ch ) 
#define ebilIsEnabled( drvp,  ch )                 (drvp)->funcp->ebilIsEnabled( ch )    
#define ebilIsChValid( drvp,  ch )                 (drvp)->funcp->ebilIsChValid( ch )    
#define ebilTransmitModeSet( drvp, ch, mode)       (drvp)->funcp->ebilTransmitModeSet( ch , mode ) 
#define ebilTransmitModeGet( drvp, ch )            (drvp)->funcp->ebilTransmitModeGet( ch ) 

typedef struct
{
   /* This must be the first structure member. */
   const EBIL_FUNCS *funcp;

} EBIL_DRV;


#ifdef __cplusplus
extern "C"
{
#endif

/* ---- Public Variables -------------------------------------------------- */


/* ---- Public Function Prototypes ---------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif
