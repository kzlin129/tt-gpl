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
*  @file    dmal.h
*
*  @brief   Lower DMA (Direct Memory Access) device driver header file
*
*  This file contains the lower device driver header information for the
*  DMA module.
*/
/****************************************************************************/

#ifndef DMAL_H
#define DMAL_H


/* ---- Include Files ----------------------------------------------------- */


/* ---- Public Constants and Types ---------------------------------------- */

typedef struct
{
   int (*dmalBurstLenGetp)( int ch );
   int (*dmalBurstLenSetp)( int ch, int len );
   int (*dmalChValidp)( int ch );
   int (*dmalDescInitp)( int ch, char *bufp, int len, int num );
   int (*dmalDescLenp)( int num );
   int (*dmalDisablep)( int ch );
   int (*dmalEnablep)( int ch );
   int (*dmalInitp)( int ch );
   int (*dmalIsEnabledp)( int ch );
   int (*dmalIsrRxp)( int ch );
   int (*dmalIsrTxp)( int ch );
   int (*dmalOptionGetp)( int ch );
   int (*dmalOptionSetp)( int ch, int opt );
   int (*dmalResetp)( int ch );
   int (*dmalRxp)( int ch, char **bufpp, int *lenp );
   int (*dmalRxNump)( int ch );
   int (*dmalRxUpdatep)( int ch, void *(*allocp)( int len ), int len, int num );
   int (*dmalRxFreep)( int ch, void (*freep)( void *bufp ) );
   int (*dmalTxp)( int ch, char *bufp, int len );
   int (*dmalTxUpdatep)( int ch, void (*freep)( void *bufp ) );
   int (*dmalTxFreep)( int ch, void (*freep)( void *bufp ) );
   int (*dmalChUpperToLowerRxp)( int ch );
   int (*dmalChUpperToLowerTxp)( int ch );
   int (*dmalIrqRxp)( int ch );
   int (*dmalIrqRxEnablep)( int ch );
   int (*dmalIrqRxDisablep)( int ch );
   int (*dmalIrqTxp)( int ch );
   int (*dmalSingleTxGetp)( int ch );
   int (*dmalSingleTxSetp)( int ch, int enable );
} DMAL_FUNCS;


#define dmalBurstLenGet( drvp, ch )                (drvp)->funcp->dmalBurstLenGetp( ch )
#define dmalBurstLenSet( drvp, ch, len )           (drvp)->funcp->dmalBurstLenSetp( ch, len )
#define dmalChValid( drvp, ch)                     (drvp)->funcp->dmalChValidp( ch )
#define dmalDescInit( drvp, ch, bufp, len, num )   (drvp)->funcp->dmalDescInitp( ch, bufp, len, num )
#define dmalDescLen( drvp, num )                   (drvp)->funcp->dmalDescLenp( num )
#define dmalDisable( drvp, ch )                    (drvp)->funcp->dmalDisablep( ch )
#define dmalEnable( drvp, ch )                     (drvp)->funcp->dmalEnablep( ch )
#define dmalInit( drvp, ch )                       (drvp)->funcp->dmalInitp( ch )
#define dmalIsEnabled( drvp, ch )                  (drvp)->funcp->dmalIsEnabledp( ch )
#define dmalIsrRx( drvp, ch )                      (drvp)->funcp->dmalIsrRxp( ch )
#define dmalIsrTx( drvp, ch )                      (drvp)->funcp->dmalIsrTxp( ch )
#define dmalOptionGet( drvp, ch )                  (drvp)->funcp->dmalOptionGetp( ch )
#define dmalOptionSet( drvp, ch, opt )             (drvp)->funcp->dmalOptionSetp( ch, opt )
#define dmalReset( drvp, ch )                      (drvp)->funcp->dmalResetp( ch )
#define dmalRx( drvp, ch, bufpp, lenp )            (drvp)->funcp->dmalRxp( ch, bufpp, lenp )
#define dmalRxNum( drvp, ch )                      (drvp)->funcp->dmalRxNump( ch )
#define dmalRxUpdate( drvp, ch, allocp, len, num ) (drvp)->funcp->dmalRxUpdatep( ch, allocp, len, num )
#define dmalRxFree( drvp, ch, freep )              (drvp)->funcp->dmalRxFreep( ch, freep )
#define dmalTx( drvp, ch, bufp, lenp )             (drvp)->funcp->dmalTxp( ch, bufp, lenp )
#define dmalTxUpdate( drvp, ch, freep )            (drvp)->funcp->dmalTxUpdatep( ch, freep )
#define dmalTxFree( drvp, ch, freep )              (drvp)->funcp->dmalTxFreep( ch, freep )
#define dmalChUpperToLowerRx( drvp, ch )           (drvp)->funcp->dmalChUpperToLowerRxp( ch )
#define dmalChUpperToLowerTx( drvp, ch )           (drvp)->funcp->dmalChUpperToLowerTxp( ch )
#define dmalIrqRx( drvp, ch )                      (drvp)->funcp->dmalIrqRxp( ch )
#define dmalIrqRxEnable( drvp, ch )                (drvp)->funcp->dmalIrqRxEnablep( ch )
#define dmalIrqRxDisable( drvp, ch )               (drvp)->funcp->dmalIrqRxDisablep( ch )
#define dmalIrqTx( drvp, ch )                      (drvp)->funcp->dmalIrqTxp( ch )
#define dmalSingleTxGet( drvp, ch )                (drvp)->funcp->dmalSingleTxGetp( ch )
#define dmalSingleTxSet( drvp, ch, enable )        (drvp)->funcp->dmalSingleTxSetp( ch, enable )

typedef struct
{
   /* This must be the first structure member. */
   const DMAL_FUNCS *funcp;

} DMAL_DRV;

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
