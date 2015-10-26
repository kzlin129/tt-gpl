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




/**
*
*  @file    resampler.c
*
*  @brief   C model for the resampler.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include "halmixer_resamp.h"
/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  saturate16
*
***************************************************************************/
short saturate16( int num )
{
   short result = (short)num;
   if ( num > 0x07fff )
   {
      result = 0x7fff;
   }
   else if ( num < (-32768) )
   {
      result = 0x8000;
   }
   return result;
}

/****************************************************************************
*
*  mixerAdder - block add function
*
***************************************************************************/
void mixerAdder
(
   short *dstp,
   short *src1p,
   short *src2p,
   int numSamp
)
{
   int k;

   for(k=0; k < numSamp; k++ )
   {
      /* saturate the result if necessary */
      dstp[k] = saturate16( ((int)src1p[k] + (int)src2p[k]) );
   }
}

/****************************************************************************
*
*  mixerApplyAtten - apply attenuation to a block of samples
*
***************************************************************************/
void mixerApplyAtten
(
   short *dstp,
   short *srcp,
   short numSamp,
   unsigned int gainVal
)
{
   int k;
   if( gainVal != 65536 )
   {
      for(k = 0; k < numSamp; k++ )
      {
         dstp[k] = (short)((srcp[k] * gainVal) >> 16 );
      }
   }
}

