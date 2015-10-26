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
*  @file    vchost_audio.h
*
*  @brief   Contains function prototypes for public functions in vchost_audio.c
*
****************************************************************************/

extern FRMFWD_FRAME_T * CB_AllocAudioBuffer( int streamNum, int length_in_bytes );
extern void CB_AudioFrameReceived( FRMFWD_FRAME_T *fwdFrame, unsigned int lengthinbyte, int streamNum );

extern int vc_audiohdl_init( void );
extern void vc_audio_start( VC_Audio_t * audiop );
extern void vc_audio_stop( VC_Audio_t * audiop );
extern int vc_audio_redirect( VC_Audio_reDirect_t *audiop );
