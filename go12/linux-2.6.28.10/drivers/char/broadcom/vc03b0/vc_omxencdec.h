/*****************************************************************************
* Copyright 2002 - 2008 Broadcom Corporation.  All rights reserved.
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
*  @file    vc_omxencdec.h
*
*  @brief   encoder and decoder path definition file dedicated for 2 way video calls
*
****************************************************************************/
#ifndef VC_OMXENCDEC_H
#define VC_OMCENCDEC_H

typedef struct
{
   int data_length;
   unsigned int flags; /* flags describing the buffer*/
   char * data;

} VC_DECPKT;

void vc_start_decThd( void );
int vc_decProcessPkt( VC_DECPKT * data );

void vc_do_dectest( int framenum, int offset, const char * path );

#endif // VC_OMXENCDEC_H

